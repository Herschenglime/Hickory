# Running Hickory on an NVIDIA DGX Spark (aarch64, GB10, CUDA 13)

Report of the port and of the end-to-end verification, written 2026-11-13.
Operational notes for day-to-day use are in [`CLAUDE.md`](../CLAUDE.md); this
document records what was changed, why, and what was measured.

## Summary

The stock `Dockerfile` targets x86_64 + CUDA 12.1 and cannot build on a DGX
Spark (aarch64, Blackwell GB10 = sm_121, driver CUDA 13.0). This branch adds an
aarch64 image variant and runs the **full pipeline end to end on all eight
REPLICA scenes (apt0 to apt3, two runs each, 41 to 51 objects) and the three
HOPE scenes (10 to 19 objects)**: extraction, SAM3D, FoundationPose,
superquadric (SQ) fitting and expansion, CLIPPER map alignment, and the
navigation planner. The default Dockerfile and compose file are unchanged.

```bash
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml build hickory
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml run --rm hickory \
  python main.py --scene dataset/REPLICA/apt0/1/ --param params/REPLICA/ \
  --output-dir reconstruction/REPLICA/apt_0_1_test
```

A full SAM3D + FoundationPose + SQ run takes roughly 25 minutes per 40 objects.

## Results

All numbers are from this host. Scene-level detail for apt0 is below; the table after it covers all four apartments.

| Check | Result |
|---|---|
| Stages completed | Every object in both scenes has `obj_mesh.glb`, `pose_foundation.txt`, `obj_sq_params.npy`, `obj_sq_params_expanded.npy` |
| CLIPPER alignment vs ground truth (camera poses) | 5.3 cm, 0.40 deg error. True offset between scenes: 4.9 m, 29.6 deg. 23 object associations |
| Matched-object centroid distance after alignment | median 9 cm (3.6 m before alignment) |
| Observed point cloud to posed SAM3D mesh | median 1.0 cm per object on apt0/1 (p90 5 cm); 1.2 cm on apt0/2 (p90 11 cm) |
| Upstream FoundationPose demo (driller, 166 frames with reference poses) | 0.18 mm, 0.13 deg mean error |
| SQs per object (apt0/1) | min 1, median 18, max 50, total 771 |
| HOPE 0000 / 0001 / 0002 (real RGB-D, tabletop) | 10 / 19 / 18 objects, median 22 / 27 / 27 SQs each, point cloud to mesh median 0.1 to 0.2 cm; all stages completed on the first attempt |

CLIPPER map alignment on all four ReplicaCAD apartments (two runs each), error
against ground truth, next to the numbers in the paper (Table 3):

| Scene | Objects (run 1 / run 2) | Associations | Ours | Paper |
|---|---|---|---|---|
| apt0 | 41 / 42 | 23 | 5.3 cm, 0.40 deg | 4.2 cm, 0.34 deg |
| apt1 | 49 / 51 | 24 | 16.6 cm, 1.2 deg | 4.7 cm, 0.83 deg |
| apt2 | 48 / 45 | 25 | 3.1 cm, 0.57 deg | 2.4 cm, 0.23 deg |
| apt3 | 46 / 45 | 27 | 3.3 cm, 0.90 deg | 4.4 cm, 0.19 deg |

The apt1 pair is the outlier (16.6 cm against 4.7 cm in the paper); the cause
has not been investigated. Each scene was run once, and SAM3D is stochastic.

Navigation, 10 random start/goal pairs on apt0/1, RRT planner, default `xz`
ground plane:

| Representation | Success | Mean path-efficiency ratio | Build time | Map storage |
|---|---|---|---|---|
| Expanded SQ | 100% | 1.09 | 0.24 s | 66 KiB |
| Mesh | 100% | 1.22 | 283 s | 1.7 MiB |
| Oriented bounding boxes | 100% | 1.20 | (shared with mesh) | 4.8 KiB |

A single 7.0 m query (`navigation_apt0_1.png`) had zero collisions on 280
densely sampled points along the path.

Demo images are in [`demo_images/`](../demo_images) (scripts in
`demo_images/scripts/`): per-object stages, whole-scene meshes and SQs for both
scenes, the alignment before/after, and the navigation path.

## Changes

### New files

| File | Purpose |
|---|---|
| `Dockerfile.aarch64` | aarch64/CUDA 13 image (header comments list each difference from the x86 image) |
| `environment.aarch64.yml` | conda/pip environment for it |
| `docker-compose.aarch64.yml` | compose override: `build.dockerfile`, `image: hickory:aarch64`, `CUDA_ARCH`/`NATTEN_CUDA_ARCH` = 12.1 |
| `docker/patch_cumm_aarch64.py` | patches cumm/spconv sources for aarch64 + CUDA 13 + sm_121 (asserts on every edit count) |
| `CLAUDE.md` | guidance for Claude Code / newcomers |
| `demo_images/` | renders and the headless matplotlib scripts that made them |
| `docs/dgx-spark-port.md` | this report |

### Modified

`.dockerignore`: excludes the 1.7 GB `datasets` zip from the build context.

### Not committed

`datasets` (zip), `dataset/`, `weights/`, `reconstruction/`, the
`third_party/FoundationPose` and `third_party/sam-3d-objects` clones, model
checkpoints. Pinned upstream commits are listed in the README.

## Port details: each change fixes a concrete failure

| Problem on aarch64 / CUDA 13 | Fix |
|---|---|
| No cu121 aarch64 wheels; sm_121 needs CUDA >= 12.8 | torch 2.9.1+cu130, conda `cuda-toolkit=13.0` (`targets/sbsa-linux`, no `lib64`), `TORCH_CUDA_ARCH_LIST=12.1`. torch warns "capability 12.1 > max 12.0" but works |
| aarch64 torchvision/torchaudio wheels have no `+cu130` tag | plain `torchvision==0.24.1`, `torchaudio==2.9.1` |
| `pycolmap` has no aarch64 PyPI wheel | install from conda-forge |
| aarch64 gtsam pulls numpy 2, breaking cv2 | pin `numpy==1.26.4` in the pip section |
| natten 0.14.6 CPU kernels do not compile against torch 2.9 (`at::vec::map2<double>`) | build natten from sdist with those two calls replaced by a scalar loop (CUDA kernels untouched) |
| `pymeshfix` narrowing errors (`char` is unsigned on aarch64) | `CFLAGS/CXXFLAGS=-fsigned-char` |
| pytorch3d link errors (CUDA 13 static global template stubs break cross-TU pulsar kernels) | `NVCC_FLAGS=-static-global-template-stub=false` |
| `warp-lang` 1.13 needs glibc 2.34; base image has 2.31 | `warp-lang==1.6.2` (kernel verified on sm_121; harmless `cuDeviceGetUuid` warning) |
| CLIPPER: `assert` undeclared | `-DCMAKE_CXX_FLAGS="-include cassert"` |
| kaolin 0.17.0 uses `AT_DISPATCH(x.type())`, removed in torch 2.9 | build kaolin master `ee8b7ec` (0.18.0) with `IGNORE_TORCH_VER=1` |
| spconv has no aarch64 wheel; cumm hard-codes x86 conda CUDA paths; CUDA 13 NVRTC needs C++17; cumm 0.7.x has no sm_12x | build spconv 2.3.8 + cumm master `d971d02` with `docker/patch_cumm_aarch64.py`, `CUMM_CUDA_VERSION=13.0 CUMM_CUDA_ARCH_LIST=12.1`, prebuilt kernels only for sm_121 |
| spconv declares `cumm<0.8.0` | installed `--no-deps` (pin bypassed because 0.7.x cannot target Blackwell). `pip check` reports it. Numerics checked against dense `conv3d`: max diff about 3e-7 with TF32 off |
| `igraph` wheel bundles a second `libgomp`: "cannot allocate memory in static TLS block" (glibc 2.31 lacks the tunable; `main.py`'s `LD_PRELOAD` re-exec overwrites any env `LD_PRELOAD`) | rebuild igraph 0.11.8 from source (`--no-binary`) |

`docker build` build-args are part of the layer cache key; always pass the same
`--build-arg`s (compose does) or the 10-minute conda layer rebuilds.

## Known issues

- **Intermittent spconv crash.** SAM3D runs die with
  `cudaErrorIllegalAddress` in spconv's `sort_1d_by_key_allocator`. Over the 11
  scenes it hit 3 of the 8 REPLICA scenes (apt0/1 once, apt1/1 twice, apt2/1
  once; four crashed attempts) and none of the 3 HOPE scenes. It is not tied to
  an object count (after 27 objects, 28, 39, and in one case straight after
  resuming), and not to an input: the same object crashed one fresh process and
  passed the next. Host memory was not under pressure. The error surfaces at the
  sort's synchronise call, which only reports an earlier asynchronous fault, so
  the faulting kernel is unknown; `CUDA_LAUNCH_BLOCKING=1` was tried only on the
  first, non-reproducing case. Root cause not found. The
  CUDA error is sticky, so `main.py`'s in-process retry cannot recover. Resume by
  re-running the same command with `--skip-extract`; finished objects are skipped.
- **Bad pose.** apt0/1 `obj_019` (a thin, partially seen stool) is posed
  upside-down, so its SQs are too.
- **Not run.** The ROS-bag example (NUS-CLB data was unavailable). A TurtleBot
  procedure and bag checks were written and tested on synthetic bags only; see
  `docs/turtlebot4-collection-procedure.md`.
- `scene.py`, `alignment.py` and the interactive navigation picker are Open3D /
  matplotlib GUIs and need a display; the images here come from headless scripts.

## Things that are easy to get wrong

- **Frames.** Per-object `camera_pose.txt` and `camera_poses.txt` are relative
  to the first camera view (OpenCV axes, y down, tilted by the first frame's
  pitch), so object poses are in that frame. `camera_poses_world.txt` is the
  dataset world (y up, floor at y = 0). `M = T_WC_world[frame 0]` maps the
  first-view frame to dataset world; the planner and `scene.py` use the
  trajectory to move objects into dataset world. `clipper_solve.py`'s transform
  (`b_to_a`) relates the first-view frames of the two scenes. The demo renders
  apply `M` so furniture is upright.
- **Navigation ground plane.** REPLICA uses the default `xz`. `--ground-plane xy`
  assumes a z-up source frame and gives wrong results here.
- **Headless navigation.** `MPLBACKEND=Agg python hickory/navigation/interactive_scene_rrt_3d.py
  --objects-dir-a <dir> --method all --benchmark-random-pairs N`, or a single
  query with `--start-xz X Z --goal-xz X Z --save-path out.csv` (planner-frame
  coordinates inside the object-defined boundary).
- **Baked wrapper files.** `fp_demo.py` and `sam3d_inference.py` are copied into
  the image over the upstream files; editing them needs an image rebuild.
- **File ownership.** Files written by the container under `reconstruction/` are
  root-owned; create or remove directories there via `docker compose run`.
- **Gated model.** SAM3D checkpoints (`facebook/sam-3d-objects`) need an approved
  Hugging Face account; download steps are in the README.
