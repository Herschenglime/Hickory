"""Top-down navigation image: planner's SQ occupancy for the robot height band + the RRT path saved by the CLI."""
import sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
sys.path.insert(0, "/workspace")
from hickory.utils.third_party import add_third_party_paths; add_third_party_paths()
from hickory.visualization.scene import find_default_camera_trajectory
from hickory.navigation.planner_3d import load_scene_world_sq_params
from hickory.navigation.planning_core import SQImplicitCollisionChecker, compute_sq_world_aabbs

objs, path_csv, out = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
traj = find_default_camera_trajectory(objs)
params, _ = load_scene_world_sq_params(objects_dir=objs, camera_trajectory_path=traj, pose_source="fp",
                                       expanded_sq_params_name="obj_sq_params_expanded.npy")
aabb = compute_sq_world_aabbs(params)
chk = SQImplicitCollisionChecker(params, aabb, robot_base_height=0.0, robot_height=0.2, vertical_samples=5)
lo = aabb[:, 0, [0, 2]].min(0) - 0.5; hi = aabb[:, 1, [0, 2]].max(0) + 0.5
res = 0.04
xs = np.arange(lo[0], hi[0], res); zs = np.arange(lo[1], hi[1], res)
X, Z = np.meshgrid(xs, zs)
occ = chk.collides(np.column_stack([X.ravel(), Z.ravel()])).reshape(X.shape)
path = np.loadtxt(path_csv, delimiter=",", skiprows=1)
print("grid", X.shape, "occupied fraction %.2f" % occ.mean(), "| path pts", len(path), "| length %.2f m" % np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
print("path points in collision:", int(chk.collides(path).sum()), "of", len(path))
# densely sample the straight segments too
seg = np.vstack([np.linspace(a, b, 40) for a, b in zip(path[:-1], path[1:])])
print("dense samples along path in collision:", int(chk.collides(seg).sum()), "of", len(seg))

fig, ax = plt.subplots(figsize=(8.5, 9))
for a in aabb:  # all objects, light outline (incl. those above the robot band)
    ax.add_patch(Rectangle((a[0, 0], a[0, 2]), a[1, 0] - a[0, 0], a[1, 2] - a[0, 2], fill=False, ec="0.7", lw=0.6))
ax.imshow(np.where(occ, 1.0, np.nan), extent=[xs[0], xs[-1] + res, zs[0], zs[-1] + res], origin="lower", cmap="Reds", vmin=0, vmax=1.6, alpha=0.9)
ax.plot(path[:, 0], path[:, 1], "-o", color="tab:blue", lw=2.5, ms=4, label=f"RRT path ({np.linalg.norm(np.diff(path, axis=0), axis=1).sum():.1f} m)")
ax.plot(*path[0], "g*", ms=18, label="start"); ax.plot(*path[-1], "m*", ms=18, label="goal")
ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("z [m]"); ax.legend(loc="upper left")
ax.set_title("apt0/1: RRT on expanded superquadrics\nred = SQ occupancy in the robot's 0-0.2 m height band, grey boxes = all object AABBs")
plt.tight_layout(); plt.savefig(out, dpi=100); print("saved", out)
