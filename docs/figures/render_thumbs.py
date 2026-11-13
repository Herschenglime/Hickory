"""Per-object thumbnails (point cloud, SAM3D mesh, SQ, expanded SQ) for the pipeline diagram. Run in container."""
import sys, os
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import trimesh, cv2
sys.path.insert(0, "/workspace")
from hickory.utils.third_party import add_third_party_paths; add_third_party_paths()
from hickory.visualization.scene import load_transform, load_camera_pose, superquadrics_to_mesh

root, name, out = sys.argv[1], sys.argv[2], sys.argv[3]
d = f"{root}/{name}"
DISP = np.array([[1.0, 0, 0], [0, 0, -1.0], [0, 1.0, 0]])
LIGHT = np.array([0.4, -0.5, 0.8]); LIGHT /= np.linalg.norm(LIGHT)
M = None
for l in open(root + "/camera_poses_world.txt"):
    if l.startswith("#") or not l.strip(): continue
    nums = []
    for x in l.split():
        try: nums.append(float(x))
        except ValueError: pass
    M = np.array(nums[-16:]).reshape(4, 4); break
T = M @ load_camera_pose(Path(d) / "camera_pose.txt") @ load_transform(Path(d) / "pose_foundation.txt")
tm = trimesh.load(d + "/obj_mesh.glb", force="mesh")
v = (T[:3, :3] @ np.asarray(tm.vertices).T).T + T[:3, 3]
meshes = {"mesh": (v @ DISP.T, np.asarray(tm.faces))}
for key, fn in (("sq", "obj_sq_params.npy"), ("sqx", "obj_sq_params_expanded.npy")):
    p = np.load(f"{d}/{fn}"); print(key, len(p), "primitives")
    m = superquadrics_to_mesh(p, resolution=16)
    vv = (T[:3, :3] @ np.asarray(m.vertices).T).T + T[:3, 3]
    meshes[key] = (vv @ DISP.T, np.asarray(m.triangles))
pc = np.load(d + "/point_cloud_accumulated_world.npy"); pc = pc[:, :3] if pc.ndim == 2 else pc
pc = ((M[:3, :3] @ pc.T).T + M[:3, 3]) @ DISP.T
if len(pc) > 6000: pc = pc[np.random.RandomState(0).choice(len(pc), 6000, replace=False)]
allv = np.vstack([meshes["sqx"][0], meshes["mesh"][0]]); ctr = (allv.min(0) + allv.max(0)) / 2; half = np.ptp(allv, axis=0).max() / 2 * 1.02

def fig_ax():
    f = plt.figure(figsize=(3, 3), dpi=200); f.patch.set_alpha(0)
    ax = f.add_axes([0, 0, 1, 1], projection="3d"); ax.patch.set_alpha(0)
    ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half); ax.set_zlim(ctr[2] - half, ctr[2] + half)
    ax.set_box_aspect((1, 1, 1), zoom=1.45); ax.view_init(24, -58); ax.set_axis_off(); return f, ax
cols = {"mesh": (0.93, 0.66, 0.30), "sq": (0.30, 0.55, 0.85), "sqx": (0.28, 0.72, 0.45)}
for key, (vv, ff) in meshes.items():
    f, ax = fig_ax(); tri = vv[ff]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    sh = 0.45 + 0.55 * np.abs(n @ LIGHT)
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(np.array(cols[key])[None] * sh[:, None], 0, 1), edgecolors="none"))
    f.savefig(f"{out}/{key}.png", transparent=True); plt.close(f)
f, ax = fig_ax(); ax.scatter(pc[:, 0], pc[:, 1], pc[:, 2], s=1.2, color=(0.35, 0.35, 0.4), alpha=0.8, depthshade=True)
f.savefig(f"{out}/pc.png", transparent=True); plt.close(f)
img = cv2.imread(d + "/view.png")[..., ::-1].copy(); msk = cv2.imread(d + "/mask.png", 0)
cnts, _ = cv2.findContours((msk > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
raw = img.copy(); cv2.drawContours(img, cnts, -1, (255, 40, 40), 3)
ys, xs = np.nonzero(msk); pad = 30
sl = (slice(max(ys.min() - pad, 0), ys.max() + pad), slice(max(xs.min() - pad, 0), xs.max() + pad))
cv2.imwrite(f"{out}/view_mask.png", img[sl][..., ::-1]); cv2.imwrite(f"{out}/view_raw.png", raw[sl][..., ::-1])
print("done")
