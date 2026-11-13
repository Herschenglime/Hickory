# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Hickory builds a hierarchical object representation of a scene from RGB-D input: raw point clouds → SAM3D meshes → FoundationPose-refined poses → fitted superquadrics (SQ) → expanded SQs. Downstream tools use the SQs for map alignment (CLIPPER) and RRT navigation. There is no test suite or linter configured; verification is done by running the pipeline.

## Running (Docker only)

Everything is run through `docker compose`; the host has no Python env. `hickory/`, `main.py`, and `params/` are bind-mounted read-only, so edits there take effect without rebuilding. `weights/`, `dataset/`, `reconstruction/`, `third_party/FoundationPose/weights`, `third_party/sam-3d-objects/checkpoints` are mounted from the host and gitignored.

```bash
# x86_64 + CUDA 12.1 hosts (the README path)
docker compose build hickory
docker compose run --rm hickory python main.py --scene dataset/REPLICA/apt0/1/ --param params/REPLICA/ --output-dir reconstruction/REPLICA/apt_0_1_test

# aarch64 / Blackwell hosts (e.g. DGX Spark GB10, sm_121, CUDA 13) -- the default Dockerfile cannot build there
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml build hickory   # pass --build-arg INSTALL_FOUNDATIONPOSE_RUNTIME_DEPS=1 if building via `docker build` directly (compose sets it)
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml run --rm hickory python main.py ...
```

Other entry points (all `python <path>` inside the container): `hickory/visualization/scene.py --objects-dir`, `hickory/align/clipper_solve.py --root --scene-a --scene-b`, `hickory/visualization/alignment.py`, `hickory/navigation/interactive_scene_rrt_3d.py`. See README.md for full flags and custom-data layouts.

`main.py` flags to resume/skip stages: `--skip-extract`, `--skip-reconst` (skips extraction **and** SAM3D, runs only SQ fitting), `--skip-pose`, `--skip-fitting`, `--skip-expansion`, `--limit N`. There is no flag to skip only SAM3D; without its checkpoints it retries each object 5×.

## Architecture

`main.py` orchestrates four stages, each writing into `<output-dir>/obj_NNN/`:

1. **Extraction** (`hickory/mapping/`): FastSAM (or OneFormer, `--seg-model`) segments each frame; `OneFormerClassMaskFilter` (DiNAT ADE20K model) drops structural classes; the ROMAN `Mapper` (`third_party/roman`) fuses observations into 3D segments; best view/mask/depth/point clouds are saved per object. `scene_extractor.py` is the driver, `extraction_utils.py` the bulk of the logic. Scene format is auto-detected (`hope`, `replica`, or a ROS-bag YAML passed as the positional arg). Params come from `params/<DATASET>/{fastsam,mapper,...}.yaml`.
2. **SAM3D** (`hickory/reconstruction/sam3d_demo.py`): per-object mesh `obj_mesh.glb` + `pose_sam3d.txt`. Needs gated HF checkpoints at `third_party/sam-3d-objects/checkpoints/hf/pipeline.yaml`.
3. **FoundationPose** (`fp_demo.py`): refines pose → `pose_foundation.txt`; launched as a subprocess (`third_party/FoundationPose/run_demo.py --map_dir ... --obj all`). Needs weights in `third_party/FoundationPose/weights/{2023-10-28-18-33-37,2024-01-11-20-02-45}`.
4. **SQ fitting/expansion** (`hickory/sq/fitting.py`, `expansion.py`, built on `third_party/mps`): `obj_sq_params.npy`, then `obj_sq_params_expanded.npy`.

Stages are idempotent per object (existing `obj_mesh.glb`, `pose_foundation.txt`, expanded params are skipped), so re-running `main.py` on the same `--output-dir` resumes.

### Third-party layout gotcha

`third_party/FoundationPose` and `third_party/sam-3d-objects` are untracked upstream clones pinned to specific commits (see README). The Dockerfile patches them (Boost/OpenMP/gsplat seds) and then **copies `hickory/reconstruction/fp_demo.py` over `FoundationPose/run_demo.py` and `sam3d_inference.py` over `sam-3d-objects/notebook/inference.py` inside the image**. Those two files are therefore baked into the image: editing them under `hickory/reconstruction/` requires an image rebuild (the bind mount does not cover `third_party/` code). `hickory/utils/third_party.py` puts all vendored dirs on `sys.path`. `main.py` re-execs itself with conda's `libstdc++` preloaded (`ensure_native_runtime`) for compiled extensions like mesh2sdf.

## aarch64 / CUDA 13 port notes (Dockerfile.aarch64, environment.aarch64.yml)

Full report with measured results: [docs/dgx-spark-port.md](docs/dgx-spark-port.md).

Differences from the x86 image, each driven by a concrete failure: torch 2.9.1+cu130 (no cu121 aarch64 wheels; sm_121 needs CUDA ≥12.8) with `torchvision==0.24.1`/`torchaudio==2.9.1` (aarch64 wheels carry no `+cu130` tag); `numpy==1.26.4` must be pinned in the pip section (aarch64 gtsam otherwise pulls numpy 2, breaking cv2); `pycolmap` from conda-forge (no PyPI aarch64 wheel); natten 0.14.6 CPU kernels patched (`at::vec::map2<double>` doesn't compile against torch 2.9; CUDA kernels untouched); `pymeshfix` built with `-fsigned-char` (char is unsigned on aarch64); pytorch3d built with `NVCC_FLAGS=-static-global-template-stub=false` (CUDA 13 default breaks cross-TU pulsar kernels); `warp-lang==1.6.2` (newer wheels need glibc 2.34, base image is older); CLIPPER built with `-include cassert`. `docker build` build-args are part of the layer cache key, so always pass the same `--build-arg`s or the 10-minute conda layer rebuilds.

SAM3D's `spconv` and `kaolin` have no aarch64 wheels, so they are built from source in the image: kaolin from master commit `ee8b7ec` (0.17.0 uses `AT_DISPATCH(x.type())`, removed in torch 2.9); spconv 2.3.8 + cumm master `d971d02` patched by `docker/patch_cumm_aarch64.py` (cumm hard-codes x86 conda CUDA paths, CUDA 13 needs C++17, sm_121 missing from its arch whitelist). spconv kernels are prebuilt **only for sm_121** (`CUMM_CUDA_ARCH_LIST`); spconv declares `cumm<0.8.0` but 0.7.x has no Blackwell support, so the pin is bypassed (`pip check` reports it; verified numerically against dense conv3d). `igraph` is rebuilt from source because its wheel bundles a second `libgomp` that fails with "cannot allocate memory in static TLS block" on aarch64 (the base image's glibc 2.31 lacks the `rtld.optional_static_tls` tunable; `main.py`'s libstdc++ `LD_PRELOAD` re-exec also overwrites any env `LD_PRELOAD`).

Verified on the aarch64 image (end to end on all eight REPLICA scenes, apt0 to apt3 with two runs each, and the three HOPE scenes): extraction, SAM3D, `main.py`'s FoundationPose stage, SQ fitting + expansion, CLIPPER alignment (`clipper_solve.py` on the four REPLICA pairs: 3.1 to 16.6 cm and 0.40 to 1.2 deg vs ground truth; apt1 is the loose one), headless RRT navigation benchmark; also the upstream FoundationPose demo (0.18 mm / 0.13 deg vs reference poses). Not verified: ROS-bag input on a real robot (NUS-CLB unavailable; a TurtleBot procedure, `hickory/utils/check_bag*.py` and `params/TURTLEBOT4/` exist and were tested on synthetic bags only). A full SAM3D+FP+SQ run takes ~25 min per 40 objects. Long REPLICA runs often die (3 of 8 scenes, none of the HOPE scenes) with `cudaErrorIllegalAddress` in spconv's `sort_1d_by_key_allocator`; it is intermittent and not tied to an object or an object count, and the error is sticky so the in-process retry cannot recover. Just re-run the same `main.py` command (`--skip-extract`), up to a few times: finished objects are skipped. To run upstream FoundationPose's own demo, mount the host's `third_party/FoundationPose/run_demo.py` under another name (the image's `run_demo.py` is Hickory's replacement) and use `--debug 0` (higher levels call `cv2.imshow`).

Headless demo/visualization: the visualizers are interactive Open3D GUIs, so images were produced with matplotlib scripts instead. Navigation can run without a GUI: `MPLBACKEND=Agg python hickory/navigation/interactive_scene_rrt_3d.py --objects-dir-a <dir> --method all --benchmark-random-pairs N` (benchmark), or `--start-xz X Z --goal-xz X Z --save-path out.csv` (single query; coordinates are in the planner frame and must lie inside the object-defined boundary). REPLICA ground plane is `xz` (the default); do **not** pass `--ground-plane xy`.

Frame conventions (easy to get wrong): per-object `camera_pose.txt` / `camera_poses.txt` are relative to the first camera view (OpenCV axes, y down, tilted by the first frame's pitch), so object poses in `obj_NNN` are in that frame. `camera_poses_world.txt` is the dataset world (y up, floor y=0). `M = T_WC_world[frame 0]` maps the first-view frame to dataset world; the planner and `scene.py` use the trajectory to move objects into dataset world. `clipper_solve.py`'s transform (`b_to_a`) relates the first-view frames of the two scenes.

`main.py` skips a non-empty `--output-dir` unless `--skip-extract` is given. `scene.py` and the other visualizers are interactive Open3D GUIs and need a display.
