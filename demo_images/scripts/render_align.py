"""Before/after CLIPPER alignment render (headless). Run inside the container.
usage: python render_align.py <root> <scene_a> <scene_b> <out_png>
Transform file is b_to_a: maps scene B world coords into scene A's frame (clipper_solve.py default)."""
import sys, glob, os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

sys.path.insert(0, "/workspace")
from hickory.utils.third_party import add_third_party_paths
add_third_party_paths()
from hickory.visualization.scene import load_transform, load_camera_pose, superquadrics_to_mesh

root, sa, sb, out = sys.argv[1:5]
T_AB = np.loadtxt(f"{root}/{sa}_{sb}_sq.txt").reshape(4, 4)
assoc = np.load(f"{root}/{sa}_{sb}_sq_associations.npy")
print("associations", assoc.shape)
DISP = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, -1.0, 0]])  # reconstruction frame is y-down; plot z = -y
LIGHT = np.array([0.4, -0.5, 0.8]); LIGHT /= np.linalg.norm(LIGHT)


def load_scene(name):
    objs = []
    for d in sorted(glob.glob(f"{root}/{name}/obj_*")):
        d = Path(d)
        T_WC = load_camera_pose(d / "camera_pose.txt")
        T_WO = T_WC @ load_transform(d / "pose_foundation.txt")
        m = superquadrics_to_mesh(np.load(d / "obj_sq_params_expanded.npy"), resolution=12)
        v = np.asarray(m.vertices); v = (T_WO[:3, :3] @ v.T).T + T_WO[:3, 3]
        objs.append((v, np.asarray(m.triangles)))
    return objs


def xf(v, T):
    return (T[:3, :3] @ v.T).T + T[:3, 3]


A, B = load_scene(sa), load_scene(sb)
Bt = [(xf(v, T_AB), f) for v, f in B]
cA = np.array([v.mean(0) for v, _ in A]); cBt = np.array([v.mean(0) for v, _ in Bt])
# the association file indexes objects of each scene; report residual of matched centroids
ia, ib = assoc[:, 0].astype(int), assoc[:, 1].astype(int)
res = np.linalg.norm(cA[ia] - cBt[ib], axis=1)
print("matched-object centroid distance after alignment [m]: median %.2f mean %.2f max %.2f (n=%d)" % (np.median(res), res.mean(), res.max(), len(res)))
res0 = np.linalg.norm(cA[ia] - np.array([v.mean(0) for v, _ in B])[ib], axis=1)
print("same pairs before alignment [m]: median %.2f" % np.median(res0))


def draw(ax, sets, elev, azim):
    allv = []
    for objs, col in sets:
        for v, f in objs:
            vd = v @ DISP.T
            tri = vd[f]
            n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
            sh = 0.5 + 0.5 * np.abs(n @ LIGHT)
            ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(np.array(col)[None] * sh[:, None], 0, 1), edgecolors="none"))
            allv.append(vd)
    allv = np.vstack(allv)
    lo, hi = np.percentile(allv, 0.5, axis=0), np.percentile(allv, 99.5, axis=0)
    c = (lo + hi) / 2; h = (hi - lo).max() / 2 * 1.05
    ax.set_xlim(c[0] - h, c[0] + h); ax.set_ylim(c[1] - h, c[1] + h); ax.set_zlim(c[2] - h, c[2] + h)
    ax.set_box_aspect((1, 1, 1), zoom=1.7); ax.view_init(elev, azim); ax.set_axis_off()


BLUE, ORANGE = (0.20, 0.45, 0.85), (0.95, 0.55, 0.10)
fig = plt.figure(figsize=(16, 7.5))
panels = [("Before: both scenes in their own frames", [(A, BLUE), (B, ORANGE)], 90, -90),
          ("After CLIPPER alignment (B -> A): top-down", [(A, BLUE), (Bt, ORANGE)], 90, -90),
          ("After alignment: oblique", [(A, BLUE), (Bt, ORANGE)], 30, -60)]
for k, (t, sets, el, az) in enumerate(panels):
    ax = fig.add_subplot(1, 3, k + 1, projection="3d"); draw(ax, sets, el, az); ax.set_title(t, fontsize=10)
fig.suptitle(f"{sa} (blue) and {sb} (orange): expanded superquadrics, {len(assoc)} CLIPPER object associations", y=0.97)
plt.tight_layout(); plt.savefig(out, dpi=95)
print("saved", out)
