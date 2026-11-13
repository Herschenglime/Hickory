"""Build docs/figures/hickory_pipeline.png (+ .pdf): the Hickory pipeline, one panel per phase, naming each tool.

Style follows Fig. 2 of the paper (arXiv:2606.01545): left-to-right, one rounded colour-coded panel per layer,
dashed inner boxes per tool. Thumbnails are real outputs from this repo's REPLICA apt0/1 run
(demo_images/*) plus per-stage renders of one object (obj_023, a chair) from render_thumbs.py.

usage (host python with matplotlib + Pillow):
  1. render per-stage object thumbnails inside the container (needs the reconstruction output):
       docker compose -f docker-compose.yml -f docker-compose.aarch64.yml run --rm -T -v $PWD/docs/figures:/figs hickory \
         python -I /figs/render_thumbs.py reconstruction/REPLICA/apt_0_1_test obj_023 /figs/thumbs
  2. python docs/figures/make_pipeline_diagram.py docs/figures/thumbs . docs/figures/hickory_pipeline
"""
import sys, textwrap
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from PIL import Image

thumbs, repo, out = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
W, H = 2800, 1500
fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")

C = {"in": ("#f1f1f1", "#6b6b6b"), "l1": ("#f7eefa", "#7a3b8e"), "l2": ("#fbf3de", "#b8862b"),
     "l3": ("#e9f2fb", "#2f6fa8"), "l4": ("#eaf6ec", "#3a8a4a"), "dn": ("#f4f4f8", "#44475a")}
FS, FT = 13, 16.5          # body / tool-name font sizes (pt at 100 dpi)


def rbox(x, y, w, h, fc, ec, lw=2.2, ls="-", r=22, z=1):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}", fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))


def autocrop(im):
    im = im.convert("RGBA"); a = np.asarray(im).astype(int)
    keep = (a[..., 3] > 10) & ~((a[..., :3] > 244).all(-1))
    ys, xs = np.nonzero(keep)
    if len(ys) == 0: return im
    return im.crop((xs.min(), ys.min(), xs.max() + 1, ys.max() + 1))


def put(im, x, y, w, h, z=3, align="center"):
    if isinstance(im, (str, Path)): im = Image.open(im)
    im = autocrop(im) if not isinstance(im, np.ndarray) else im
    arr = np.asarray(im); ih, iw = arr.shape[:2]
    s = min(w / iw, h / ih); dw, dh = iw * s, ih * s
    x0 = x + (w - dw) / 2; y0 = y + (h - dh) / 2
    ax.imshow(arr, extent=(x0, x0 + dw, y0 + dh, y0), zorder=z, interpolation="lanczos"); return (x0, y0, dw, dh)


def wrap(t, w_px, fs=FS):
    return "\n".join(textwrap.fill(p, max(8, int(w_px / (fs * 100 / 72 * 0.53)))) for p in t.split("\n"))


def step(x, y, w, h, title, sub, desc, thumb=None, mode="side", frac=0.42, tint="#ffffff", ec="#888", caption=None, formula=None):
    rbox(x, y, w, h, tint, ec, lw=1.6, ls=(0, (5, 4)), r=14, z=2)
    tw = w * (1 - frac) - 26 if (thumb is not None and mode == "side") else w - 28
    ax.text(x + 14, y + 12, title, fontsize=FT, fontweight="bold", va="top", color="#1c1c1c", zorder=4)
    ax.text(x + 14, y + 12 + 30, sub, fontsize=FS - 1.5, va="top", color="#666", style="italic", zorder=4)
    body = wrap(desc, tw)
    ax.text(x + 14, y + 12 + 56, body, fontsize=FS, va="top", color="#222", linespacing=1.28, zorder=4)
    nlines = body.count("\n") + 1; text_h = 56 + nlines * FS * 100 / 72 * 1.28 + 14
    if formula:
        ax.text(x + w / 2, y + h - 22, formula, fontsize=FS - 2.2, ha="center", va="bottom", color="#222", zorder=4)
    if thumb is None: return
    if mode == "side":
        put(thumb, x + w * (1 - frac), y + 10, w * frac - 10, h - 20 - (22 if caption else 0))
    else:
        put(thumb, x + 10, y + text_h, w - 20, h - text_h - 8 - (22 if caption else 0))
    if caption:
        ax.text(x + w - 12, y + h - 8, caption, fontsize=FS - 2, ha="right", va="bottom", color="#555", zorder=4)


def panel(x, w, key, kicker, name, path, steps, writes, letter, top=130, bottom=1360):
    fc, ec = C[key]
    rbox(x, top, w, bottom - top, fc, ec, lw=3)
    ax.text(x + w / 2, top + 16, kicker, fontsize=14, ha="center", va="top", color=ec, fontweight="bold")
    ax.text(x + w / 2, top + 44, name, fontsize=21, ha="center", va="top", color=ec, fontweight="bold")
    ax.text(x + w / 2, top + 84, path, fontsize=FS - 1.5, ha="center", va="top", color="#555", family="monospace")
    y0, y1 = top + 124, bottom - 92; gap = 14
    tot = sum(s["wt"] for s in steps); avail = (y1 - y0) - gap * (len(steps) - 1); y = y0
    for s in steps:
        h = avail * s["wt"] / tot
        step(x + 16, y, w - 32, h, **{k: v for k, v in s.items() if k != "wt"}, ec=ec); y += h + gap
    ax.text(x + 16, bottom - 82, "writes", fontsize=FS - 2, color=ec, va="top", fontweight="bold")
    ax.text(x + 16, bottom - 62, wrap(writes, w - 32, FS - 2), fontsize=FS - 2, va="top", color="#333", family="monospace", linespacing=1.25)
    ax.text(x + w / 2, bottom + 14, letter, fontsize=15, ha="center", va="top", color="#333")


def arrow(x0, x1, y):
    ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle="simple,head_width=22,head_length=22,tail_width=9", color="#1f5f78", zorder=5, lw=0))


def chip(x, y, text, color):
    ax.text(x, y, text, fontsize=FS - 1.5, color="white", fontweight="bold", ha="right", va="top", zorder=6,
            bbox=dict(boxstyle="round,pad=0.3", fc=color, ec="none"))


# ---- data for thumbnails ---------------------------------------------------------------------
sc = repo / "dataset/REPLICA/apt0/1"
rgb = Image.open(sc / "rgb/frame_000150.png").convert("RGB")
dep = np.asarray(Image.open(sc / "depth/frame_000150.png")).astype(float)
dn = np.clip(dep / max(np.percentile(dep[dep > 0], 99), 1), 0, 1)
depc = Image.fromarray((plt.get_cmap("turbo_r")(dn)[..., :3] * 255).astype(np.uint8))
poses = []
for l in open(sc / "camera_poses.txt"):
    if l.startswith("#") or not l.strip(): continue
    p = []
    for v in l.split():
        try: p.append(float(v))
        except ValueError: pass
    p = p[-16:]
    if len(p) == 16: poses.append(p)
poses = np.array(poses)
demo = repo / "demo_images"
sc_mesh = Image.open(demo / "apt0_1_scene_meshes.png").convert("RGBA"); sc_mesh = sc_mesh.crop((0, 40, sc_mesh.width // 2, sc_mesh.height))
sc_sqx = Image.open(demo / "apt0_1_scene_superquadrics.png").convert("RGBA"); sc_sqx = sc_sqx.crop((0, 40, sc_sqx.width // 2, sc_sqx.height))
al = Image.open(demo / "alignment_apt0_1_to_apt0_2.png").convert("RGBA"); al = al.crop((505, 140, 1015, 640))
nav = Image.open(demo / "navigation_apt0_1.png").convert("RGBA"); nav = nav.crop((60, 55, 700, 790))

# ---- title ---------------------------------------------------------------------------------------
ax.text(W / 2, 22, "Hickory: from posed RGB-D to a hierarchical object representation", fontsize=30, fontweight="bold", ha="center", va="top", color="#1c1c1c")
ax.text(W / 2, 74, "Each phase builds one layer; map alignment uses Layer 3 and navigation uses Layer 4.  Thumbnails: REPLICA apt0/1, run on the DGX Spark port.", fontsize=15, ha="center", va="top", color="#555")

# ---- columns ---------------------------------------------------------------------------------------
m, g = 40, 36
widths = [300, 470, 450, 400, 400, 520]
xs = [m]
for w_ in widths[:-1]: xs.append(xs[-1] + w_ + g)

# Input
fc, ec = C["in"]
x, w = xs[0], widths[0]
rbox(x, 130, w, 1230, fc, ec, lw=3)
ax.text(x + w / 2, 146, "Input", fontsize=21, ha="center", va="top", fontweight="bold", color=ec)
ax.text(x + w / 2, 190, "image folder or ROS bag", fontsize=FS - 1.5, ha="center", va="top", color="#555", family="monospace")
yy = 254
for lab, sub, content, hh in (("RGB images", "", rgb, 195), ("Depth images", "registered to RGB, metric", depc, 195), ("Camera poses", "T_WC per frame + intrinsics", None, 330)):
    rbox(x + 16, yy, w - 32, hh + 58, "white", ec, lw=1.6, ls=(0, (5, 4)), r=14, z=2)
    ax.text(x + 30, yy + 12, lab, fontsize=FT - 1, fontweight="bold", va="top", color="#1c1c1c", zorder=4)
    if sub: ax.text(x + 30, yy + 38, sub, fontsize=FS - 2, va="top", style="italic", color="#666", zorder=4)
    if content is not None:
        put(np.asarray(content), x + 28, yy + 62, w - 56, hh - 14)
    else:
        bx, by, bw, bh = x + 28, yy + 38, w - 56, hh - 8
        t = poses[:, [3, 11]]; t = t - t.mean(0); sc_ = min(bw, bh) * 0.44 / max(np.abs(t).max(), 1e-6)
        ax.plot(bx + bw / 2 + t[:, 0] * sc_, by + bh / 2 + t[:, 1] * sc_, color="#3a6ea5", lw=2.2, zorder=4)
        ax.scatter([bx + bw / 2 + t[0, 0] * sc_], [by + bh / 2 + t[0, 1] * sc_], s=90, c="#2a9d3f", zorder=5)
        ax.scatter([bx + bw / 2 + t[-1, 0] * sc_], [by + bh / 2 + t[-1, 1] * sc_], s=90, c="#d1453b", zorder=5)
    yy += hh + 58 + 14
ax.text(x + 16, 1278, wrap("REPLICA / HOPE folders, or a ROS 2 bag with RGB, depth, camera_info and an odometry or tf pose source", w - 32, FS - 1.5), fontsize=FS - 1.5, va="top", color="#333", linespacing=1.25)
ax.text(x + w / 2, 1374, "", fontsize=1)

S1 = [dict(wt=1.0, title="FastSAM", sub="CASIA-LMC-Lab, class-agnostic", desc="Segments every keyframe into instance masks without a fixed object list (open-set).", thumb=thumbs / "view_mask.png", frac=0.34),
      dict(wt=0.8, title="OneFormer (ADE20K)", sub="SHI-Labs, semantic filter", desc="Drops walls, floors, ceilings, sky and people so only reconstructable objects continue."),
      dict(wt=1.15, title="ROMAN mapper", sub="MIT-ACL, 3D object tracking", desc="Lifts each mask to 3D with depth and pose, then tracks it across frames by 3D IoU of oriented boxes: one object library with multi-view point clouds.", thumb=thumbs / "pc.png", frac=0.36),
      dict(wt=1.0, title="DINOv2 best-view score", sub="one view per object", desc="Scores views by feature variance σ², mask area A and valid-depth ratio D:\nS = ln(1+σ²)·A^α·D^β\nOnly the best view goes on to SAM3D.", thumb=thumbs / "view_raw.png", frac=0.30)]
S2 = [dict(wt=0.8, title="SAM3D", sub="Meta, image to 3D object", desc="Best RGB view + mask in, complete textured mesh out (also a coarse pose). Runs on GPU and is the slowest stage: about 25 min per 40 objects on the GB10.", thumb=thumbs / "mesh.png", frac=0.38),
      dict(wt=1.25, title="FoundationPose", sub="NVIDIA, 6-D pose", desc="Uses the SAM3D mesh as the template to refine a reliable object pose in the camera frame; the camera trajectory then places it in the world, giving an object-level scene.", thumb=sc_mesh, mode="below", caption="41 posed meshes")]
S3 = [dict(wt=1.0, title="Marching-Primitives (mps)", sub="ChirikjianLab, mesh to SQs", desc="Converts the mesh to a signed-distance field and decomposes it into superquadrics: a few analytic primitives per object (31 for this chair).", thumb=thumbs / "sq.png", mode="below"),
      dict(wt=0.75, title="Shape descriptor", sub="first K superquadrics", desc="Each primitive is (a1,a2,a3, ε1,ε2, pose). The first K are stable across views, so they serve as a compact, pose-invariant object signature; volume and centroid are analytic.", formula=r"$\left(|x/a_1|^{2/\epsilon_2}+|y/a_2|^{2/\epsilon_2}\right)^{\epsilon_2/\epsilon_1}+|z/a_3|^{2/\epsilon_1}=1$")]
S4 = [dict(wt=1.0, title="SQ expansion", sub="hickory/sq/expansion.py", desc="Takes the mesh vertices (optionally pushed out by a safety margin), assigns each to its nearest primitive, then scales that primitive until its vertices sit inside it (inside-outside value ≤ target).", thumb=thumbs / "sqx.png", mode="below"),
      dict(wt=0.9, title="Expanded scene", sub="Layer 4 for the whole scene", desc="", thumb=sc_sqx, mode="below", caption="41 objects")]

panel(xs[1], widths[1], "l1", "Layer 1", "Raw Perception", "hickory/mapping/", S1,
      "view.png  mask.png  depth.png\npoint_cloud_*.npy  camera_pose.txt", "(a)")
panel(xs[2], widths[2], "l2", "Layer 2", "Mesh Reconstruction", "hickory/reconstruction/", S2,
      "obj_mesh.glb  pose_sam3d.txt\npose_foundation.txt", "(b)")
panel(xs[3], widths[3], "l3", "Layer 3", "Shape Abstraction", "hickory/sq/fitting.py", S3,
      "obj_sq_params.npy", "(c)")
panel(xs[4], widths[4], "l4", "Layer 4", "SQ Inflation", "hickory/sq/expansion.py", S4,
      "obj_sq_params_expanded.npy", "(d)")

# Downstream
x, w = xs[5], widths[5]
fc, ec = C["dn"]
rbox(x, 130, w, 1230, fc, ec, lw=3)
ax.text(x + w / 2, 146, "Downstream uses", fontsize=21, ha="center", va="top", fontweight="bold", color=ec)
ax.text(x + w / 2, 190, "hickory/align/  hickory/navigation/", fontsize=FS - 1.5, ha="center", va="top", color="#555", family="monospace")
hh = (1360 - 240 - 24 - 14) / 2
step(x + 16, 240, w - 32, hh, "Map alignment: ROMAN + CLIPPER", "align maps from separate runs",
     "ROMAN's registration scores object pairs by superquadric shape (unary) and inter-object distances (pairwise). CLIPPER finds the largest mutually consistent set of matches, and a rigid fit to them gives the relative transform T_AB between maps.",
     thumb=al, mode="below", ec=ec, caption="apt0/1 vs apt0/2: 23 matches, 5.3 cm / 0.40°")
chip(x + w - 28, 282, "uses Layer 3", "#2f6fa8")
step(x + 16, 240 + hh + 14, w - 32, hh, "Navigation: RRT + analytic check", "collision checks without meshes",
     "A sampling-based RRT planner asks one question per point: is G_k(T_k⁻¹p) > 1 for every inflated superquadric? That is an algebraic test, with no distance queries against dense meshes or point clouds.",
     thumb=nav, mode="below", ec=ec, caption="10/10 random queries solved, path 1.09x straight line")
chip(x + w - 28, 240 + hh + 14 + 42, "uses Layer 4", "#3a8a4a")
ax.text(x + w / 2, 1374, "(e)", fontsize=15, ha="center", va="top", color="#333")

# arrows between panels
ya = 745
for i in range(5):
    arrow(xs[i] + widths[i] + 4, xs[i + 1] - 4, ya)
# footer
ax.text(W / 2, 1418, "Tools named in the paper: FastSAM, OneFormer, DINOv2, ROMAN, SAM3D, FoundationPose, Marching-Primitives, CLIPPER.  Layers 1-4 are built once, offline, per scene; alignment and navigation then reuse the compact SQ layers.",
        fontsize=14, ha="center", va="top", color="#444")
fig.savefig(out + ".png", dpi=100); fig.savefig(out + ".pdf")
print("saved", out)
