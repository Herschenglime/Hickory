"""Self-contained sanity checks for a ROS 2 bag that is meant to be fed to Hickory.

Needs only `rosbags` and `numpy` (pip install rosbags numpy; opencv is optional and only used to decode
compressed depth). No ROS install, no Hickory environment, no GPU: copy this one file to the robot, a laptop or the
workstation and run it where the bag already is. It reads .mcap and .db3 bags.

What it can and cannot tell you:
  * It reports data rates and per-topic sizes (use --list on a short test recording to project storage).
  * It checks the things Hickory needs: image/depth sizes, camera_info match, depth validity, timestamps,
    RGB/depth/pose synchronisation, a plausible trajectory, and whether the camera extrinsic points forward.
  * It does NOT run Hickory's own loader. `hickory/utils/check_bag.py` (needs the Hickory container) does, and is the
    authoritative check; use this one for fast feedback close to the robot.

Examples
  # 1. what is in the bag, how big is each topic, how long would 5 minutes be?
  python check_bag_standalone.py BAG_DIR --list

  # 2. full check; odometry as the pose source, camera mounted at the base looking forward
  python check_bag_standalone.py BAG_DIR --rgb-topic RGB --depth-topic DEPTH --rgb-info RGB_INFO --depth-info DEPTH_INFO \
      --pose-topic ODOM --extrinsic flurdf

  # 3. pose from the tf tree (e.g. localisation in a map frame) and the camera mount from tf_static
  python check_bag_standalone.py BAG_DIR --rgb-topic ... --depth-topic ... --tf-pose map:base_link \
      --tf-topic /ns/tf --tf-static-topic /ns/tf_static --extrinsic tf --base-frame base_link --camera-frame camera_optical
"""
import argparse
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

RESULTS = []


def report(level, msg):
    RESULTS.append(level)
    print(f"[{level}] {msg}")


# ----------------------------------------------------------------------------------------- helpers
def stamp(msg, fallback_ns):
    try:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    except Exception:
        return fallback_ns * 1e-9


def jpeg_size(data):
    """(height, width) from a JPEG byte string, without decoding."""
    b = bytes(data); i = 2
    while i + 9 < len(b):
        if b[i] != 0xFF:
            i += 1; continue
        m = b[i + 1]
        if 0xC0 <= m <= 0xCF and m not in (0xC4, 0xC8, 0xCC):
            h, w = struct.unpack(">HH", b[i + 5:i + 9]); return h, w
        i += 2 + struct.unpack(">H", b[i + 2:i + 4])[0]
    return None


def png_size(data):
    b = bytes(data)
    if b[:8] == b"\x89PNG\r\n\x1a\n":
        w, h = struct.unpack(">II", b[16:24]); return h, w
    return None


def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w); x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_from(tr, q):
    T = np.eye(4); T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w); T[:3, 3] = [tr.x, tr.y, tr.z]; return T


T_FLURDF = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])  # pose of an RDF (optical) camera in an FLU base


def conn_by_topic(reader, topic):
    c = [c for c in reader.connections if c.topic == topic]
    if not c:
        topics = sorted({x.topic for x in reader.connections})
        sys.exit(f"topic {topic!r} not in bag. Topics present:\n  " + "\n  ".join(topics))
    return c


def stream_stats(name, times):
    times = np.asarray(times, dtype=float)
    if len(times) < 2:
        report("FAIL", f"{name}: {len(times)} messages"); return
    gaps = np.diff(times)
    print(f"      {name:6s} n={len(times):6d}  span={times[-1] - times[0]:7.1f}s  median={1.0 / np.median(gaps):5.1f} Hz  max gap={gaps.max():.2f}s")
    if gaps.max() > 1.0:
        report("WARN", f"{name}: gap of {gaps.max():.2f}s (dropped frames or paused recording)")


# ----------------------------------------------------------------------------------------- --list
def list_bag(reader, ref_minutes):
    dur = (reader.duration or 0) * 1e-9
    print(f"Bag: {reader.start_time * 1e-9:.1f} .. {reader.end_time * 1e-9:.1f}  duration {dur:.1f}s  ({len(reader.connections)} connections)\n")
    size = defaultdict(int); n = defaultdict(int)
    for c, t, raw in reader.messages():
        size[c.topic] += len(raw); n[c.topic] += 1
    rows = []
    for c in reader.connections:
        if c.topic in {r[0] for r in rows}: continue
        rows.append((c.topic, c.msgtype, n[c.topic], size[c.topic]))
    print(f"{'topic':55s} {'type':38s} {'msgs':>7s} {'Hz':>6s} {'MB':>8s} {'MB/s':>7s} {'MB per %g min' % ref_minutes:>14s}")
    tot = 0
    for topic, mt, cnt, sz in sorted(rows, key=lambda r: -r[3]):
        hz = cnt / dur if dur else 0; mbs = sz / 1e6 / dur if dur else 0; tot += sz
        print(f"{topic:55s} {mt:38s} {cnt:7d} {hz:6.1f} {sz / 1e6:8.1f} {mbs:7.2f} {mbs * 60 * ref_minutes:14.0f}")
    if dur:
        print(f"{'TOTAL':55s} {'':38s} {'':7s} {'':6s} {tot / 1e6:8.1f} {tot / 1e6 / dur:7.2f} {tot / 1e6 / dur * 60 * ref_minutes:14.0f}")
        print(f"\nProjection assumes the rest of the recording looks like this one; compare the last column with free space (df -h).")


# ----------------------------------------------------------------------------------------- loaders
def load_images(reader, topic, want_pixels, n_pixel_samples=0):
    """header stamps, bag-receive times, (h, w) of the first message, encoding/format, and optionally a few decoded depth frames."""
    conns = conn_by_topic(reader, topic); times, recv = [], []; hw = None; enc = None; samples = []
    total = sum(c.msgcount for c in conns)
    pick = set(np.linspace(0, max(total - 1, 0), min(n_pixel_samples, total)).astype(int).tolist()) if want_pixels else set()
    for k, (c, t, raw) in enumerate(reader.messages(connections=conns)):
        m = reader.deserialize(raw, c.msgtype)
        times.append(stamp(m, t)); recv.append(t * 1e-9)
        if hw is None:
            if hasattr(m, "format"):          # CompressedImage
                enc = m.format; hw = jpeg_size(m.data) or png_size(m.data) or (png_size(bytes(m.data)[12:]) if "compressedDepth" in enc else None)
            else:                              # Image
                enc = m.encoding; hw = (m.height, m.width)
        if k in pick:                          # decode only the sampled frames (depth is ~2 MB each)
            d = decode_depth(m)
            if d is not None: samples.append((times[-1], d))
    return np.array(times), np.array(recv), hw, enc, samples


def decode_depth(m):
    if hasattr(m, "format"):  # compressed
        try:
            import cv2
        except ImportError:
            return None
        buf = np.frombuffer(bytes(m.data)[12:] if "compressedDepth" in m.format else bytes(m.data), np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    dt = {"16UC1": np.uint16, "32FC1": np.float32, "mono16": np.uint16}.get(m.encoding)
    if dt is None:
        return None
    a = np.frombuffer(bytes(m.data), dtype=dt)
    return a.reshape(m.height, m.step // np.dtype(dt).itemsize)[:, :m.width]


def load_camera_info(reader, topic):
    c = conn_by_topic(reader, topic)
    for conn, t, raw in reader.messages(connections=c):
        m = reader.deserialize(raw, conn.msgtype); return m.width, m.height, np.array(m.k).reshape(3, 3)


def load_odom(reader, topic):
    conns = conn_by_topic(reader, topic); times, P = [], []
    for c, t, raw in reader.messages(connections=conns):
        m = reader.deserialize(raw, c.msgtype)
        p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
        T = np.eye(4); T[:3, :3] = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        T[:3, 3] = [p.position.x, p.position.y, p.position.z]
        times.append(stamp(m, t)); P.append(T)
    return np.array(times), np.stack(P)


def load_tf(reader, topic, static=False):
    """dict (parent, child) -> (times, [T_parent_child])"""
    out = defaultdict(lambda: ([], []))
    for c, t, raw in reader.messages(connections=conn_by_topic(reader, topic)):
        m = reader.deserialize(raw, c.msgtype)
        for tr in m.transforms:
            ts, Ts = out[(tr.header.frame_id.lstrip("/"), tr.child_frame_id.lstrip("/"))]
            ts.append(stamp(tr, t) if not static else 0.0); Ts.append(T_from(tr.transform.translation, tr.transform.rotation))
    return {k: (np.array(v[0]), v[1]) for k, v in out.items()}


def tf_lookup(edges, parent, child, t):
    """T_parent_child from a graph of (parent, child) edges, latest sample at or before t (static edges: first)."""
    adj = defaultdict(list)
    for (p, c) in edges:
        adj[p].append((c, p, c, False)); adj[c].append((p, p, c, True))
    prev = {parent: None}; queue = [parent]
    while queue:
        n = queue.pop(0)
        for nxt, p, c, inv in adj[n]:
            if nxt not in prev:
                prev[nxt] = (n, p, c, inv); queue.append(nxt)
    if child not in prev:
        return None
    T = np.eye(4); n = child; chain = []
    while prev[n] is not None:
        chain.append(prev[n]); n = prev[n][0]
    for (_, p, c, inv) in reversed(chain):
        ts, Ts = edges[(p, c)]
        i = max(int(np.searchsorted(ts, t, side="right")) - 1, 0)   # latest sample at or before t (static edges are all stamped 0)
        E = Ts[i]; T = T @ (np.linalg.inv(E) if inv else E)
    return T


# ----------------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="bag directory (contains metadata.yaml) or a single .mcap/.db3 file's directory")
    ap.add_argument("--list", action="store_true", help="list topics with sizes and a storage projection, then exit")
    ap.add_argument("--project-minutes", type=float, default=5.0, help="recording length used for the --list projection")
    ap.add_argument("--rgb-topic"); ap.add_argument("--depth-topic"); ap.add_argument("--rgb-info"); ap.add_argument("--depth-info")
    ap.add_argument("--pose-topic", help="nav_msgs/Odometry or PoseStamped topic")
    ap.add_argument("--tf-pose", help="PARENT:CHILD frames, pose from the tf tree instead (e.g. map:base_link)")
    ap.add_argument("--tf-topic", default="/tf"); ap.add_argument("--tf-static-topic", default="/tf_static")
    ap.add_argument("--extrinsic", choices=["flurdf", "identity", "tf"], default="flurdf",
                    help="camera pose in the base frame: flurdf = at the base origin looking forward (Hickory's T_FLURDF); tf = from tf_static")
    ap.add_argument("--base-frame"); ap.add_argument("--camera-frame")
    ap.add_argument("--depth-scale", type=float, default=1000.0); ap.add_argument("--max-depth", type=float, default=3.0)
    ap.add_argument("--dt", type=float, default=0.25, help="Hickory's sync tolerance (dt in the bag yaml)")
    ap.add_argument("--samples", type=int, default=40)
    a = ap.parse_args()

    with AnyReader([Path(a.bag)]) as reader:
        if a.list:
            return list_bag(reader, a.project_minutes)
        need = [a.rgb_topic, a.depth_topic, a.rgb_info, a.depth_info]
        if not all(need) or not (a.pose_topic or a.tf_pose):
            sys.exit("need --rgb-topic --depth-topic --rgb-info --depth-info and one of --pose-topic / --tf-pose (run --list first)")
        print("Reading bag ...")
        rt, rrecv, rhw, renc, _ = load_images(reader, a.rgb_topic, False)
        dt_, drecv, dhw, denc, dsamples = load_images(reader, a.depth_topic, True, a.samples)
        rinfo, dinfo = load_camera_info(reader, a.rgb_info), load_camera_info(reader, a.depth_info)
        if a.pose_topic:
            pt, P = load_odom(reader, a.pose_topic)
        else:
            parent, child = a.tf_pose.split(":")
            edges = {**load_tf(reader, a.tf_topic), **load_tf(reader, a.tf_static_topic, static=True)}
            tl = np.array(sorted({t for k, (ts, _) in edges.items() if len(ts) > 1 for t in ts}))
            pt = tl; P = []
            for t in tl:
                T = tf_lookup(edges, parent, child, t)
                P.append(T if T is not None else np.full((4, 4), np.nan))
            P = np.stack(P); ok = ~np.isnan(P[:, 0, 0]); pt, P = pt[ok], P[ok]
            if len(pt) == 0:
                sys.exit(f"no tf path {parent} -> {child} in {a.tf_topic}/{a.tf_static_topic}; frames seen: " + ", ".join(sorted({f for k in edges for f in k})))
        extr_candidates = {"flurdf": [("T_FLURDF", T_FLURDF)], "identity": [("identity", np.eye(4))]}.get(a.extrinsic)
        if a.extrinsic == "tf":
            if not (a.base_frame and a.camera_frame):
                sys.exit("--extrinsic tf needs --base-frame and --camera-frame")
            sedges = load_tf(reader, a.tf_static_topic, static=True)
            Tbc = tf_lookup(sedges, a.base_frame, a.camera_frame, 0.0)
            if Tbc is None:
                sys.exit(f"no static path {a.base_frame} -> {a.camera_frame}; frames seen: " + ", ".join(sorted({f for k in sedges for f in k})))
            extr_candidates = [("T_base_camera (tf_static, as is)", Tbc), ("inverse of that", np.linalg.inv(Tbc))]
            print(f"      camera mount from tf_static, T_{a.base_frame}_{a.camera_frame}:\n{np.round(Tbc, 3)}")

    print("\n== Streams (header timestamps)")
    stream_stats("rgb", rt); stream_stats("depth", dt_); stream_stats("pose", pt)
    t_lo, t_hi = max(rt[0], dt_[0], pt[0]), min(rt[-1], dt_[-1], pt[-1])
    if t_hi <= t_lo:
        report("FAIL", "rgb, depth and pose time ranges do not overlap (different clocks?)"); return finish()
    dur = t_hi - t_lo
    print(f"      overlap of all three: {dur:.1f}s -> about {int(np.ceil(dur * 6))} frames processed at Hickory's 6 fps cap")
    if dur < 60: report("WARN", f"only {dur:.0f}s of usable data; aim for 3-6 min")
    for name, ts_, rv in (("rgb", rt, rrecv), ("depth", dt_, drecv)):
        off = np.median(rv - ts_)
        if abs(off) > 1.0:
            report("WARN", f"{name}: header stamps differ from bag receive time by a median {off:+.1f}s (camera clock vs recorder clock)")

    print("\n== Images")
    print(f"      rgb   {rhw[1]}x{rhw[0]} ({renc})  camera_info {rinfo[0]}x{rinfo[1]} fx={rinfo[2][0, 0]:.1f} fy={rinfo[2][1, 1]:.1f}")
    print(f"      depth {dhw[1]}x{dhw[0]} ({denc})  camera_info {dinfo[0]}x{dinfo[1]} fx={dinfo[2][0, 0]:.1f}")
    if (rinfo[1], rinfo[0]) != tuple(rhw): report("FAIL", f"rgb camera_info {rinfo[0]}x{rinfo[1]} != rgb image {rhw[1]}x{rhw[0]}")
    if (dinfo[1], dinfo[0]) != tuple(dhw): report("FAIL", f"depth camera_info {dinfo[0]}x{dinfo[1]} != depth image {dhw[1]}x{dhw[0]}")
    if tuple(rhw) != tuple(dhw):
        report("WARN", f"depth {dhw[1]}x{dhw[0]} != rgb {rhw[1]}x{rhw[0]}: Hickory nearest-resizes depth, valid only if it is already registered to the RGB camera")
    if not np.allclose(rinfo[2][[0, 1], [0, 1]], dinfo[2][[0, 1], [0, 1]], rtol=0.02) and tuple(rhw) == tuple(dhw):
        report("WARN", "rgb and depth intrinsics differ although the sizes match: depth is probably not aligned to the RGB camera")
    if dsamples:
        valid = [float(np.mean(((d / (a.depth_scale if d.dtype != np.float32 else 1.0)) > 0.1) & ((d / (a.depth_scale if d.dtype != np.float32 else 1.0)) < a.max_depth))) for _, d in dsamples]
        print(f"      depth pixels in (0.1 m, {a.max_depth} m): median {np.median(valid):.0%}, worst {np.min(valid):.0%}")
        if np.median(valid) < 0.3: report("WARN", "less than 30% of depth pixels usable: check depth units, max_depth, scene distance")
    else:
        report("WARN", f"could not decode depth pixels ({denc}); install opencv-python-headless for compressed depth")

    print("\n== Synchronisation (rgb frame vs nearest depth / pose)")
    ts = np.linspace(t_lo, t_hi, a.samples)
    gd = np.array([np.min(np.abs(dt_ - t)) for t in ts]); gp = np.array([np.min(np.abs(pt - t)) for t in ts])
    print(f"      depth gap: median {np.median(gd) * 1e3:.0f} ms, max {gd.max() * 1e3:.0f} ms;  pose gap: median {np.median(gp) * 1e3:.0f} ms, max {gp.max() * 1e3:.0f} ms")
    if gd.max() > a.dt: report("WARN", f"depth up to {gd.max() * 1e3:.0f} ms from the RGB frame (Hickory tolerance dt={a.dt}s; those frames are skipped)")
    if gp.max() > 0.2: report("WARN", f"nearest pose up to {gp.max() * 1e3:.0f} ms away")

    print("\n== Trajectory")
    grid = np.linspace(t_lo, t_hi, 400); idx = np.clip(np.searchsorted(pt, grid), 0, len(pt) - 1); Pg = P[idx]; tt = pt[idx]
    pos = Pg[:, :3, 3]; steps = np.linalg.norm(np.diff(pos, axis=0), axis=1); path = steps.sum()
    print(f"      path length {path:.1f} m, bounding box {np.ptp(pos, axis=0).round(2)} m, max step between samples {steps.max():.2f} m")
    if path < 2.0: report("WARN", f"robot only moved {path:.1f} m")
    if steps.max() > 0.5: report("WARN", f"pose jump of {steps.max():.2f} m between samples {(grid[1] - grid[0]) * 1e3:.0f} ms apart (localisation jump?)")

    for label, Tbc in extr_candidates:
        Pc = Pg @ Tbc                         # T_W_camera = T_W_base @ T_base_camera
        cos = []
        for i in range(0, len(Pc), 4):
            j = int(np.searchsorted(grid, grid[i] + 2.0))
            if j >= len(Pc): break
            v = Pc[j, :3, 3] - Pc[i, :3, 3]
            if np.linalg.norm(v) < 0.15: continue  # at least 15 cm of travel within 2 s
            cos.append(float(Pc[i, :3, 2] @ (v / np.linalg.norm(v))))
        down = np.median(Pc[:, :3, 1], axis=0)
        if len(cos) < 3:
            report("WARN", f"[{label}] not enough forward driving to check the camera axis (needs >=15 cm travel within 2 s, a few times)"); continue
        c = float(np.median(cos))
        print(f"      [{label}] camera z-axis . direction of travel: median {c:+.2f};  camera image-down axis {down.round(2)} (expect ~[0,0,-1] in a z-up frame)")
        if c >= 0.6 and abs(down[2]) > 0.6: report("PASS", f"[{label}] camera looks along the direction of travel with image-down pointing at the floor")
        elif a.extrinsic == "tf" and label.startswith("inverse"): pass
        else: report("WARN" if c >= 0.3 else "FAIL", f"[{label}] camera axis does not follow the direction of travel (cos {c:+.2f})")
    if a.extrinsic == "tf":
        print("      (with --extrinsic tf, only one of the two candidates should PASS: that one tells you the `inv` setting for Hickory's T_odombase_camera)")
    return finish()


def finish():
    print(f"\nSummary: {RESULTS.count('PASS')} pass, {RESULTS.count('WARN')} warn, {RESULTS.count('FAIL')} fail")
    sys.exit(1 if RESULTS.count("FAIL") else 0)


if __name__ == "__main__":
    main()
