"""Headless demo renders for a Hickory output dir (run inside the container).

usage: python render_demo.py <objects_dir> <out_dir> <tag>
Poses: pose_foundation.txt (T_CO, FoundationPose-refined) -> T_WO = T_WC @ T_CO,
same convention as hickory/visualization/scene.py with --pose-source fp.
"""
import sys, glob, os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import trimesh
import cv2

sys.path.insert(0, "/workspace")
from hickory.utils.third_party import add_third_party_paths
add_third_party_paths()
from hickory.visualization.scene import load_transform, load_camera_pose, superquadrics_to_mesh

root, out, tag = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(out, exist_ok=True)
cm = plt.get_cmap("tab20")
DISP = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])  # after M (dataset-world, y up): plot z = y, plot y = -z
LIGHT = np.array([0.4, -0.5, 0.8]); LIGHT /= np.linalg.norm(LIGHT)


def _first_world_pose(root):
    for l in open(root + "/camera_poses_world.txt"):
        if l.startswith("#") or not l.strip():
            continue
        nums = []
        for x in l.split():
            try: nums.append(float(x))
            except ValueError: pass
        return np.array(nums[-16:]).reshape(4, 4)

M = _first_world_pose(root)
print("odom->dataset-world (frame 0):\n", M.round(3))


def load_obj(d):
    """Return dict with posed mesh (V,F), SQ meshes, point cloud, all in world frame."""
    try:
        T_WC = load_camera_pose(Path(d) / "camera_pose.txt")
        T_WO = M @ T_WC @ load_transform(Path(d) / "pose_foundation.txt")
    except Exception as e:
        print("pose load failed", d, repr(e)); return None
    r = {"dir": d, "name": os.path.basename(d), "T_WO": T_WO}
    tm = trimesh.load(d + "/obj_mesh.glb", force="mesh")
    if len(tm.faces) > 6000:
        import open3d as o3d
        om = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(tm.vertices)), o3d.utility.Vector3iVector(np.asarray(tm.faces)))
        om = om.simplify_quadric_decimation(target_number_of_triangles=6000)
        tm = trimesh.Trimesh(np.asarray(om.vertices), np.asarray(om.triangles), process=False)
    v = (T_WO[:3, :3] @ np.asarray(tm.vertices).T).T + T_WO[:3, 3]
    r["mesh"] = (v @ DISP.T, np.asarray(tm.faces))
    for key, fn in (("sq", "obj_sq_params.npy"), ("sqx", "obj_sq_params_expanded.npy")):
        p = np.load(f"{d}/{fn}")
        m = superquadrics_to_mesh(p, resolution=14)
        v = np.asarray(m.vertices); v = (T_WO[:3, :3] @ v.T).T + T_WO[:3, 3]
        r[key] = (v @ DISP.T, np.asarray(m.triangles)); r[key + "_n"] = len(p)
    pc = np.load(d + "/point_cloud_accumulated_world.npy")
    pc = pc[:, :3] if pc.ndim == 2 else pc
    if len(pc) > 3000:
        pc = pc[np.random.RandomState(0).choice(len(pc), 3000, replace=False)]
    r["pc"] = ((M[:3, :3] @ pc.T).T + M[:3, 3]) @ DISP.T
    return r


def draw(ax, items, elev=28, azim=-60, lim=None, pc=None, zoom=1.25):
    """items: list of (V,F,color). Lambert-shaded triangles."""
    allv = np.vstack([v for v, _, _ in items]) if items else pc
    for v, f, c in items:
        tri = v[f]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        sh = 0.45 + 0.55 * np.abs(n @ LIGHT)
        col = np.clip(np.array(c[:3])[None] * sh[:, None], 0, 1)
        pcoll = Poly3DCollection(tri, facecolors=col, edgecolors="none", linewidths=0)
        ax.add_collection3d(pcoll)
    if pc is not None:
        ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], s=0.4, color="0.55", alpha=0.5)
    lo, hi = allv.min(0), allv.max(0)
    ctr = (lo + hi) / 2; half = (hi - lo).max() / 2 * 1.05 if lim is None else lim
    ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half); ax.set_zlim(ctr[2] - half, ctr[2] + half)
    ax.set_box_aspect((1, 1, 1), zoom=zoom); ax.view_init(elev, azim); ax.set_axis_off()


dirs = sorted(glob.glob(root + "/obj_*"))
objs = []
for d in dirs:
    o = load_obj(d)
    if o is not None:
        objs.append(o)
print(f"loaded {len(objs)}/{len(dirs)} objects")

# ---- 1. per-object grid: RGB | point cloud+mesh | SQ | expanded SQ -------------------
sizes = [np.ptp(o["pc"], axis=0).max() for o in objs]
order = np.argsort(sizes)[::-1]
# pick 8 objects spread over the size distribution (not just the biggest), prefer many SQs
pick = [objs[i] for i in order[np.linspace(2, min(len(objs) - 1, 28), 8).astype(int)]]
fig = plt.figure(figsize=(15, 3.3 * len(pick)))
for r, o in enumerate(pick):
    c = cm(objs.index(o) % 20)
    img = cv2.imread(o["dir"] + "/view.png")[..., ::-1].copy()
    msk = cv2.imread(o["dir"] + "/mask.png", 0)
    if msk is not None and msk.shape[:2] == img.shape[:2]:
        cnts, _ = cv2.findContours((msk > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, (255, 40, 40), 3)
        ys, xs = np.nonzero(msk)
        pad = 40
        y0, y1, x0, x1 = max(ys.min() - pad, 0), ys.max() + pad, max(xs.min() - pad, 0), xs.max() + pad
        img = img[y0:y1, x0:x1]
    ax = fig.add_subplot(len(pick), 4, r * 4 + 1); ax.imshow(img); ax.axis("off")
    ax.set_title(f"{o['name']}: input view + mask" if r == 0 else o["name"], fontsize=9)
    lim = np.ptp(np.vstack([o["mesh"][0], o["sqx"][0]]), axis=0).max() / 2 * 1.1
    for j, (title, key) in enumerate((("SAM3D mesh (FoundationPose pose)", "mesh"), (f"superquadrics", "sq"), ("expanded superquadrics", "sqx"))):
        ax = fig.add_subplot(len(pick), 4, r * 4 + 2 + j, projection="3d")
        items = [(o[key][0], o[key][1], c)]
        draw(ax, items, lim=lim, pc=o["pc"] if key == "mesh" else None)
        n = f" ({o[key + '_n']})" if key != "mesh" else ""
        if r == 0:
            ax.set_title(title + n, fontsize=9)
        else:
            ax.set_title(n.strip(), fontsize=9)
fig.suptitle(f"Hickory {tag}: per-object pipeline stages (grey dots = observed point cloud)", y=0.995)
plt.tight_layout(); plt.savefig(f"{out}/{tag}_objects.png", dpi=95); plt.close(fig)

# ---- 2. scene-level renders --------------------------------------------------------
def scene(key, fname, title):
    fig = plt.figure(figsize=(15, 7.5))
    for k, (elev, azim, nm) in enumerate(((28, -55, "oblique"), (90, -90, "top-down"))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        items = [(o[key][0], o[key][1], cm(i % 20)) for i, o in enumerate(objs)]
        allv = np.vstack([v for v, _, _ in items])
        # drop gross outliers from the view window
        lo, hi = np.percentile(allv, 1, axis=0), np.percentile(allv, 99, axis=0)
        ctr = (lo + hi) / 2; half = (hi - lo).max() / 2 * 1.1
        draw(ax, items, elev=elev, azim=azim, lim=half, zoom=1.35)
        ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half); ax.set_zlim(ctr[2] - half, ctr[2] + half)
        ax.set_title(nm, fontsize=10)
    fig.suptitle(title); plt.tight_layout(); plt.savefig(f"{out}/{fname}", dpi=95); plt.close(fig)

scene("mesh", f"{tag}_scene_meshes.png", f"{tag}: {len(objs)} SAM3D meshes placed by FoundationPose poses (world frame)")
scene("sqx", f"{tag}_scene_superquadrics.png", f"{tag}: {len(objs)} expanded superquadric objects (world frame)")

# ---- 3. numbers ---------------------------------------------------------------------
nsq = [o["sq_n"] for o in objs]
print("SQs per object: min %d median %d max %d total %d" % (min(nsq), np.median(nsq), max(nsq), sum(nsq)))
# sanity: distance from observed point cloud to the posed mesh surface
dists = []
for o in objs:
    tm = trimesh.Trimesh(*o["mesh"], process=False)
    try:
        _, d, _ = trimesh.proximity.closest_point(tm, o["pc"][:1500])
        dists.append(np.median(d))
    except Exception:
        pass
print("median point-cloud->mesh distance per object [cm]: median %.1f  p90 %.1f" % (100 * np.median(dists), 100 * np.percentile(dists, 90)))
