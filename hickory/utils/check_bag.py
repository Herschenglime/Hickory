"""Preflight check for a recorded ROS bag, before spending hours of GPU time on it.

Loads the bag through the same ROMAN `DataParams` path that `main.py` uses and reports what the pipeline
will actually see: stream rates and gaps, image/depth sizes, depth validity, RGB/depth/pose synchronisation,
and whether the resulting camera poses are plausible (camera looks along the direction of travel, trajectory
has no jumps). Prints PASS / WARN / FAIL lines and exits non-zero on FAIL.

usage (inside the container):
    python hickory/utils/check_bag.py params/TURTLEBOT4/data.yaml [--depth-scale 1000] [--max-depth 3.0]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
from hickory.utils.third_party import add_third_party_paths  # noqa: E402

add_third_party_paths()
from roman.params.data_params import DataParams  # noqa: E402

RESULTS = []


def report(level, msg):
    RESULTS.append(level)
    print(f"[{level}] {msg}")


def stream_stats(name, times):
    times = np.asarray(times, dtype=float)
    if len(times) < 2:
        report("FAIL", f"{name}: {len(times)} messages")
        return
    gaps = np.diff(times)
    hz = 1.0 / np.median(gaps)
    print(f"      {name:6s} n={len(times):6d}  span={times[-1] - times[0]:7.1f}s  median={hz:5.1f} Hz  max gap={gaps.max():.2f}s")
    if gaps.max() > 1.0:
        report("WARN", f"{name}: gap of {gaps.max():.2f}s (dropped frames / paused recording?)")


def nearest_gap(t, ref_times):
    return float(np.min(np.abs(np.asarray(ref_times) - t)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config", help="bag YAML config (e.g. params/TURTLEBOT4/data.yaml)")
    ap.add_argument("--run", default=None)
    ap.add_argument("--depth-scale", type=float, default=1000.0, help="depth units per metre (1000 for uint16 mm)")
    ap.add_argument("--max-depth", type=float, default=3.0, help="fastsam.yaml max_depth, for the usable-pixel fraction")
    ap.add_argument("--samples", type=int, default=40, help="number of frames to inspect")
    args = ap.parse_args()

    params = DataParams.from_yaml(args.config, run=args.run)
    print("Loading streams (this reads the whole bag) ...")
    img, dep, pose = params.load_img_data(), params.load_depth_data(), params.load_pose_data()
    dt = params.dt

    print("\n== Streams")
    stream_stats("rgb", img.times)
    stream_stats("depth", dep.times)
    pt = getattr(pose, "times", None)
    if pt is not None:
        stream_stats("pose", pt)
    t_lo = max(img.times[0], dep.times[0], pose.t0)
    t_hi = min(img.times[-1], dep.times[-1], pose.tf)
    if t_hi <= t_lo:
        report("FAIL", "rgb, depth and pose time ranges do not overlap")
        return finish()
    dur = t_hi - t_lo
    print(f"      overlap of all three: {dur:.1f}s -> about {int(np.ceil(dur * 6))} frames processed at the pipeline's 6 fps cap")
    if dur < 60:
        report("WARN", f"only {dur:.0f}s of usable data; aim for 3-10 min")

    ts = np.linspace(t_lo, t_hi, args.samples)

    print("\n== Images")
    t_mid = float(ts[len(ts) // 2])
    rgb = img.img(img.nearest_time(t_mid)) if hasattr(img, "nearest_time") else img.img(t_mid)
    d = dep.img(dep.nearest_time(t_mid)) if hasattr(dep, "nearest_time") else dep.img(t_mid)
    print(f"      rgb   shape={rgb.shape} dtype={rgb.dtype}")
    print(f"      depth shape={d.shape} dtype={d.dtype} range=[{d.min()}, {d.max()}]")
    if rgb.shape[:2] != d.shape[:2]:
        report("WARN", f"depth {d.shape[:2]} != rgb {rgb.shape[:2]}: the pipeline will nearest-resize depth, which is only correct if "
                       "the depth is already registered to the RGB camera (aligned depth output)")
    cp = getattr(img, "camera_params", None)
    if cp is not None and hasattr(cp, "width"):
        print(f"      camera_info: {cp.width}x{cp.height}  fx={cp.K[0, 0]:.1f} fy={cp.K[1, 1]:.1f} cx={cp.K[0, 2]:.1f} cy={cp.K[1, 2]:.1f}")
        if (cp.height, cp.width) != rgb.shape[:2]:
            report("FAIL", f"camera_info is {cp.width}x{cp.height} but images are {rgb.shape[1]}x{rgb.shape[0]}: intrinsics do not match")
    if d.dtype == np.uint16 and args.depth_scale != 1000.0:
        report("WARN", "uint16 depth is normally millimetres (--depth-scale 1000)")
    if d.dtype == np.float32 and args.depth_scale == 1000.0:
        report("WARN", "float32 depth is normally metres; set depth_scale: 1.0 in fastsam.yaml")

    valid, near = [], []
    for t in ts:
        dd = dep.img(dep.nearest_time(float(t))).astype(float) / args.depth_scale
        valid.append(float(np.mean((dd > 0.1) & (dd < args.max_depth))))
        near.append(float(np.percentile(dd[dd > 0], 5)) if np.any(dd > 0) else np.nan)
    print(f"      depth pixels in (0.1 m, {args.max_depth} m): median {np.median(valid):.0%}, worst {np.min(valid):.0%}; 5th-percentile range {np.nanmedian(near):.2f} m")
    if np.median(valid) < 0.3:
        report("WARN", "less than 30% of depth pixels are usable: check depth units, max_depth, and scene distance")

    print("\n== Synchronisation (rgb frame vs nearest depth / pose)")
    gd = np.array([nearest_gap(t, dep.times) for t in ts])
    gp = np.array([nearest_gap(t, pt) for t in ts]) if pt is not None else np.zeros(len(ts))
    print(f"      depth gap: median {np.median(gd) * 1e3:.0f} ms, max {gd.max() * 1e3:.0f} ms;  pose gap: median {np.median(gp) * 1e3:.0f} ms, max {gp.max() * 1e3:.0f} ms")
    tol = dt if dt else 0.05
    if gd.max() > tol:
        report("WARN", f"depth is up to {gd.max() * 1e3:.0f} ms from the RGB frame (pipeline tolerance is dt={tol:.3f}s; those frames are skipped)")
    if gp.max() > 0.2:
        report("WARN", f"nearest pose is up to {gp.max() * 1e3:.0f} ms away: pose topic too slow or wrong clock")

    print("\n== Camera trajectory (as the pipeline will compute it)")
    T = []
    for t in np.linspace(t_lo, t_hi, 400):
        try:
            T.append((float(t), np.asarray(pose.pose(float(t)), dtype=float)))
        except Exception:
            pass
    if len(T) < 50:
        report("FAIL", f"pose lookup succeeded for only {len(T)}/400 sample times")
        return finish()
    tt = np.array([a for a, _ in T]); P = np.stack([b for _, b in T])
    pos = P[:, :3, 3]
    steps = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    path = steps.sum()
    print(f"      path length {path:.1f} m, bounding box {np.ptp(pos, axis=0).round(2)} m, max step between samples {steps.max():.2f} m")
    if path < 2.0:
        report("WARN", f"robot only moved {path:.1f} m: object tracking and view selection benefit from parallax (walk a loop)")
    if steps.max() > 0.5:
        report("WARN", f"pose jump of {steps.max():.2f} m between samples {(tt[1] - tt[0]) * 1e3:.0f} ms apart: a localization jump (AMCL relocalisation?) smears objects")
    # camera optical axis (OpenCV z) should point along the direction of travel for a forward-facing camera
    cos = []
    for i in range(0, len(P), 4):
        j = int(np.searchsorted(tt, tt[i] + 2.0))
        if j >= len(P):
            break
        v = pos[j] - pos[i]
        if np.linalg.norm(v) < 0.15:  # needs >= 15 cm of travel within 2 s to define a direction
            continue
        cos.append(float(P[i, :3, 2] @ (v / np.linalg.norm(v))))
    if len(cos) >= 3:
        c = np.median(cos)
        print(f"      camera z-axis . direction of travel (translating segments): median {c:+.2f}")
        if c < 0.3:
            report("FAIL", "camera z-axis is not along the direction of travel: T_odombase_camera (the only extrinsic the pipeline applies) is wrong, "
                           "or the camera is not forward-facing")
        elif c < 0.6:
            report("WARN", "camera z-axis does not follow the direction of travel (expected ~+1 for a forward-facing camera while driving straight)")
        else:
            report("PASS", "camera looks along the direction of travel")
    else:
        report("WARN", "not enough straight driving to check the camera axis; include about a metre of forward driving at >= 0.1 m/s")
    down = np.median(P[:, :3, 1], axis=0)
    print(f"      camera y-axis (image down) in the pose frame: {down.round(2)}  (expect about [0, 0, -1] for a z-up frame)")
    if abs(down[2]) < 0.6:
        report("WARN", "camera image-down axis is not aligned with the pose frame's vertical axis (tilted mount or wrong extrinsics)")
    return finish()


def finish():
    n_fail, n_warn = RESULTS.count("FAIL"), RESULTS.count("WARN")
    print(f"\nSummary: {RESULTS.count('PASS')} pass, {n_warn} warn, {n_fail} fail")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
