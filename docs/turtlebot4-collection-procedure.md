# Collecting a Hickory-ready run with a TurtleBot 4 Lite

**Audience:** the agent/workspace that knows the lab's TurtleBot (namespace, camera driver, nav2 setup).
**Author's scope:** I know what Hickory needs and how it fails; I do **not** know your robot's topic names, frame
names, driver parameters or storage. Anything robot-specific below is a **suggestion** and is labelled with its source.
Treat every `<ns>/...` name as a placeholder to be replaced from the real robot.

Source labels used below: **[code]** read in this repo's source, **[tested]** exercised on a synthetic bag in the Hickory
container, **[measured]** measured on files in this repo, **[docs]** read in the TurtleBot 4 manual, **[reasoned]**
my inference, not checked on the robot.

---

## 1. What Hickory needs from a run (the contract)

Satisfy these and the details of how are up to you. Each item says how Hickory uses it and what breaks otherwise.

| # | Need | Why / what breaks | Source |
|---|---|---|---|
| R1 | **Color images** with header timestamps. Resolution large enough to segment objects (the extractor runs at 736x1280; thumbnails of ~250 px are unusable). Raw or JPEG-compressed both work. | FastSAM masks every frame; SAM3D is fed the best view. | [code] `params/*/fastsam.yaml`; [tested] compressed + raw |
| R2 | **Metric depth registered to the color camera** (aligned depth, same resolution as R1), header timestamps. 16-bit millimetres (`depth_scale: 1000`) or float metres (`1.0`). | Masks from color are lifted to 3D with this depth. If sizes differ Hickory nearest-resizes depth, which is only right when it is already registered. Unregistered depth gives objects offset from their masks. | [code] `extraction_utils.py`; [tested] |
| R3 | **`camera_info` for both streams**, matching the image resolution. | Intrinsics are read from the bag. Wrong resolution gives wrongly scaled 3D. Distortion handling is not verified here, so rectified images are safest. | [code] |
| R4 | **A camera pose over time**: a `nav_msgs/Odometry`, `PoseStamped` or `Path` topic, or a tf chain (e.g. `map` to `base_link`), at 10 Hz or more, in one fixed frame. | Every frame is placed in 3D with it; there is no SLAM or loop closure inside Hickory, so the map is only as good as these poses. Jumps (e.g. a relocalization) smear objects. | [code] `data_params.py` |
| R5 | **The camera mount**: the pose of the optical frame in the base frame, rotation right and translation to a few cm. | Applied as `T_WC = T_W_base @ T_odombase_camera`. Wrong rotation puts objects in the wrong places; a wrong height or offset shifts everything. | [code], [tested] |
| R6 | **One clock**: RGB, depth and pose header stamps from the same (or synchronised) clock. | Frames are matched by nearest timestamp within `dt` (0.25 s by default); poorer matches are skipped. | [code] |
| R7 | **Slow, smooth motion**: 0.2 m/s or less and about 30 deg/s or less (suggestion). Include roughly 1 m of forward driving at 0.1 m/s or more. | Motion blur and large inter-frame motion break tracking. The forward segment is only what the checker uses to verify the camera axis (see section 5). | [reasoned]; the 1 m / 0.1 m/s figure is the checker's threshold [code] |
| R8 | **Objects that survive Hickory's filters**: largest dimension at least 25 cm, seen in at least 2 frames, 0.7 to 3 m from the camera, not moving, not people. Place them at least 0.5 m apart, in even light. | Segments with a maximum extent below `min_max_extent: 0.25` m, fewer than `min_sightings: 2` sightings, or beyond `max_depth: 3.0` m are dropped. All three are tunable in `params/TURTLEBOT4/`. | [code] `mapper.yaml`, `fastsam.yaml` |
| R9 | **Enough objects**: 10 to 25 distinct ones per session. | Fewer than about 8 shared objects makes the map-alignment demo unreliable (the apt0 pair matched 23, the apt1 pair 24). | [measured] on REPLICA |
| R10 | **A bag containing all of the above**, as a rosbag2 directory (`metadata.yaml` plus `.mcap` or `.db3`), including the tf and tf_static topics if poses or the camera mount come from tf. | The loader reads the whole bag from that directory. | [code], [tested] `.mcap` |
| R11 | **Two sessions in one fixed frame** (for the alignment demo): same localization or odometry session between them, no reboot, no re-initialisation, at least about 8 objects visible in both. | Alignment error can only be scored automatically if both sessions share a frame. | [reasoned] |
| R12 | **3 to 6 minutes per session**, 2 minutes at the very least. | Short runs give few objects and no parallax. | [reasoned] |

Not needed (checked against the code, since these were asked about):
- **Loop closure.** Hickory has none and does not need the robot to return to its start.
- **A straight 3 m segment.** Only about 1 m of forward driving is needed, and only for the camera-axis sanity check.
- **Several views of every object.** Hickory reconstructs each object from a single best view; extra views only help tracking and best-view selection. I overstated this in the earlier draft.
- **Objects like SAM3D's training data.** There is no such requirement that I know of. See section 3.

## 2. Pose source and namespacing (loader facts)

- With `pose_data.type: bag` the topic must be Odometry, PoseStamped or Path. [code]
- With `pose_data.type: bag_tf` poses come from the tf tree and the YAML accepts **`tf_topic_namespace`** for a namespaced `/<ns>/tf`. [code: robotdatapy `PoseData.from_bag_tf`; tested on a synthetic namespaced bag, see `params/TURTLEBOT4/data.yaml` variant B]
- **Gotcha:** the camera mount, `T_odombase_camera` with `input_type: tf`, does **not** forward a namespace, so it reads only the un-namespaced `/tf_static` and will fail on a namespaced robot. [code: ROMAN `find_transformation`] Options: use `input_type: matrix` ([tested] through the loader) with the 4x4 from `check_bag_standalone.py --extrinsic tf` (it prints the matrix and tells you whether it or its inverse is the right one), or patch ROMAN to forward the namespace (it is baked into the image, so that needs a rebuild). Frame ids may also carry the namespace prefix.
- `T_camera_flu` is parsed but **not applied**; only `T_odombase_camera` affects the pose. [code, tested]
- Localized poses (AMCL on your map) have no drift but jump when the filter corrects itself, and a jump smears objects (R4). Wheel odometry has no jumps but drifts. Recording the whole tf tree lets both be tried offline. [reasoned]
- The extractor re-bases each session to its first pose, and also writes `camera_poses_world.txt` in the original frame, which is what a ground-truth comparison between two sessions uses. [code]

## 3. Objects and scene

- **Floor-level objects.** Your robot is short (TurtleBot 4 Lite is 192 mm tall overall [docs]), so the camera sits low and looks horizontally; tabletop surfaces above it are mostly not visible. Floor objects are the right plan. [reasoned]
- **Size:** at least 25 cm in one dimension (R8), so boxes, bins, bags, backpacks, chairs, plants in pots, cones. Items such as bottles or mugs are dropped by default. Flat items (mats) are seen at a grazing angle and reconstruct poorly.
- **Do they have to resemble SAM3D's training data?** No requirement that I know of; I do not know its training mix. In our runs it handled furniture, plants, bicycles, rugs (synthetic REPLICA) and toy grocery items (real photographs, HOPE). What matters is a clean mask, an object mostly in view, and enough pixels. Known weak spots from our runs: flat objects become slabs, plants come out blobby, and one object (obj_019 of apt0/1) was posed upside-down. Reflective, transparent and very thin objects are the likely trouble.
- Even lighting, no strong backlight, nobody walking through the scene.
- Place one object whose position you measure with a tape from a fixed floor mark (for the accuracy check, section 6).

## 4. Bag size and where to record

Hickory processes at most 6 frames/s (`MAX_FRAMES_PER_SECOND = 6` [code]) and samples every `dt` = 0.25 s, so **recording at about 4 to 5 Hz is enough**; recording faster only costs space.

Arithmetic (the depth size is exact, the RGB size is [measured] on a 1280x720 REPLICA frame: 70 / 108 / 208 KB at JPEG quality 70 / 85 / 95; real camera frames are noisier, so assume 100 to 200 KB):

| Depth format | Per depth frame | 4 Hz | 5 Hz | 10 Hz |
|---|---|---|---|---|
| 1280x720, 16-bit raw | 1.84 MB | 7.4 MB/s | 9.2 MB/s | 18.4 MB/s |
| 640x360, 16-bit raw | 0.46 MB | 1.8 MB/s | 2.3 MB/s | 4.6 MB/s |

Add roughly 0.4 to 1 MB/s for compressed RGB at those rates, and under 10 MB per session for odometry, tf and camera_info.

| Session of 5 minutes (300 s) | Size |
|---|---|
| 1280x720 depth + RGB, 5 Hz | about 3 to 3.5 GB |
| 1280x720 depth + RGB, 4 Hz | about 2.5 to 3 GB |
| 1280x720 depth + RGB, 10 Hz | about 6 GB |
| 640x360 depth (and matching RGB), 5 Hz | under 1 GB |

So two 5-minute sessions at 5 Hz, full resolution, is about 6 to 7 GB. Compressing the bag (zstd) would shrink depth, but I have not measured it on real stereo depth, and it costs CPU on a Pi that is also running nav2, so I would not rely on it.

**Do not trust my numbers for your robot:** the driver may publish a different depth size or rate. Record a 30 to 60 s test, then:
```
python check_bag_standalone.py <bag> --list --project-minutes 5
```
prints MB and MB/s per topic and the projected size for 5 minutes (use the last column). Compare it with `df -h` on the Pi for the recording location; leave at least 2x headroom. Also compare the printed rates with what you expected: missing messages on the Pi's SD card show up as a lower Hz than the publisher's. How you get to 4 to 5 Hz (driver rate, throttled republish, record fewer messages) is up to you; throttling adds CPU load.

## 5. Checks you can run before moving the bag

Two scripts, same idea, different environments:

| Script | Needs | Use for |
|---|---|---|
| `hickory/utils/check_bag_standalone.py` | `pip install rosbags numpy` (opencv only if depth is compressed). No ROS, no Hickory, no GPU. **One file; copy it to the Pi or a laptop and run it where the bag is.** | Fast feedback right after recording: storage projection (`--list`), rates, sizes, depth validity, sync, trajectory, camera-axis check, and the camera-mount transform from tf_static. |
| `hickory/utils/check_bag.py` | The Hickory container | The authoritative check: runs the same loader as `main.py`. Needs the bag on the workstation. |

[tested] on synthetic mcap bags in the container (Python 3.11, rosbags 0.11.9), including a namespaced one with a tf tree. **Not tested on a Raspberry Pi**; rosbags is pure Python so it should install, but check the Python and rosbags versions. Reading a multi-GB bag from an SD card will be slow.

The bag has to reach the DGX Spark anyway for the real run (SAM3D needs the GPU), so plan one copy over your own route; the standalone check lets you find a bad recording before that trip.

Typical use (topics are yours to fill in):
```
python check_bag_standalone.py BAG --list
python check_bag_standalone.py BAG --rgb-topic <ns>/... --depth-topic <ns>/... --rgb-info <ns>/... --depth-info <ns>/... \
    --pose-topic <ns>/odom --extrinsic flurdf
# poses from tf, camera mount from tf_static (namespaced tf):
python check_bag_standalone.py BAG ... --tf-pose map:<ns>/base_link --tf-topic /<ns>/tf --tf-static-topic /<ns>/tf_static \
    --extrinsic tf --base-frame <ns>/base_link --camera-frame <ns>/<optical frame>
```

How to read the results:

| Message | Meaning |
|---|---|
| `camera_info ... != ... image` (FAIL) | Intrinsics are for another resolution: R3 |
| `depth != rgb` (WARN) / intrinsics differ although sizes match (WARN) | Depth probably not registered to RGB: R2 |
| header stamps differ from bag receive time (WARN) | Camera and recorder clocks differ: R6 |
| `pose jump` (WARN) | Localization jump or gap: R4 |
| camera axis does not follow the direction of travel (FAIL) | Camera mount rotation wrong, or camera not forward-facing: R5. With `--extrinsic tf` it tests the transform and its inverse; whichever PASSES is the correct one. |
| fewer than 30% usable depth pixels (WARN) | Depth units, range, or objects too far/near |

## 6. Driving and recording protocol (suggestions)

Use whatever nav2 route setup you have. What matters is the contract above:
1. Start the recording, then stand still for about 5 s, camera facing the scene.
2. Two waypoint routes, one per session. Out-and-back along the same corridor is acceptable as two sessions: each session needs its own objects in view (R8, R9), and the opposite viewing direction on the return is a harder, more convincing alignment demo. Stop recording between them; keep the robot, localization and odometry running so the frame is shared (R11).
3. Keep the speed at 0.2 m/s or less. Gentle arcs rather than spinning in place. Include about 1 m of forward driving (R7).
4. Stand still for about 3 s at the end, then stop the recording. Do not dock or bump anything.
5. Record to the Pi if there is space (section 4). Run the standalone `--list` and full check on the Pi or a laptop.
6. Copy the bag directory (with `metadata.yaml`) to `dataset/TURTLEBOT4/run1/` and `run2/` on the Spark.

## 7. Running Hickory on the bag

After the checks pass (config in `params/TURTLEBOT4/`, **draft, untested on real data**):
```
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml run --rm hickory \
  python hickory/utils/check_bag.py params/TURTLEBOT4/data.yaml
docker compose -f docker-compose.yml -f docker-compose.aarch64.yml run --rm hickory \
  python main.py params/TURTLEBOT4/data.yaml --param params/TURTLEBOT4/ \
  --output-dir reconstruction/TURTLEBOT4/run1 --limit 5
```
Look at `reconstruction/TURTLEBOT4/run1/obj_*/view.png` and `mask.png`: one real object per mask? Then run without `--limit`
(about 15 minutes of SAM3D for 20 objects on the GB10). If the run dies with the sticky `cudaErrorIllegalAddress`, re-run with `--skip-extract`; it is an
intermittent, known issue (`docs/dgx-spark-port.md`; so far it has hit 3 of the 5 REPLICA scenes and none of the 3 HOPE scenes, and one fresh-process retry on the same object passed while another crashed).

Then map alignment (`hickory/align/clipper_solve.py --root reconstruction/TURTLEBOT4 --scene-a run1 --scene-b run2`) and navigation (`hickory/navigation/interactive_scene_rrt_3d.py`, headless
flags in `CLAUDE.md`). The automatic error report in `clipper_solve.py` needs a Kimera ground-truth CSV, which does not exist here; with a shared frame I can compute the error from `camera_poses_world.txt`.

Accuracy check: the tape-measured object's point-cloud centroid, in the shared frame, should be within about 10 cm of the measurement. A constant offset points at the camera mount; an error growing with distance travelled points at the pose source.

## 8. For the robot expert to decide (things I cannot know)

- The real RGB, depth and camera_info topic names, whether depth is aligned to RGB at the same resolution, and how to get 4 to 5 Hz.
- The real frame names and the camera mount transform (run the standalone check with `--extrinsic tf`).
- Pose source: odometry, or localized poses in the lab map; whether tf is recorded under the namespace; whether to use the `matrix` form of `T_odombase_camera` (section 2).
- Free storage on the Pi and whether it is enough for section 4's projection.
- Whether recording the bag while nav2 runs overloads the Pi.
- Where depth is poor on your camera (minimum range, textureless surfaces), and whether your driver's stock settings already satisfy R2.
- Anything in this document that disagrees with what you see on the robot: the robot wins, and the contract in section 1 is what has to hold.
