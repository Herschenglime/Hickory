# Collecting a Hickory-ready run with `tb4lite1`

Written 2026-11-13. This is the robot-side answer to Hickory's handoff,
`Hickory/docs/turtlebot4-collection-procedure.md` (on the Hickory repo's `dgx-spark` branch). That document
lists what Hickory needs (requirements **R1 to R12**) and leaves the robot-specific facts open (its section 8).
This one fills them in from what this workspace holds, and turns the rest into a procedure.

Source labels: **[bag]** read from the recorded staleness-demo bags in `staleness_demo/bags/`,
**[install]** read from `/opt/ros/jazzy`, **[repo]** from this workspace's scripts, `CLAUDE.md` and routes,
**[inferred]** worked out from those but never observed on the robot, **[unverified]** a guess to be settled in
section 2. The robot was **not reachable** when this was written (the graph held only `/rosout` and
`/parameter_events`), so nothing about the live camera has been observed. No bag in this workspace has camera
topics: the staleness runs recorded only `scan`, `odom`, `tf`, `tf_static` and `amcl_pose`.

---

## 1. What this workspace establishes about the robot

| Fact | Value | Source |
|---|---|---|
| Namespace | `tb4lite1`. Topics are namespaced, **frame ids are not** (`base_link`, `odom`, `map`). | [bag], [repo] |
| It has a camera | OAK-D Lite on the camera bracket; the frames `oakd_rgb_camera_optical_frame`, `oakd_left/right_camera_optical_frame` are in `/tb4lite1/tf_static`. | [bag], [install] URDF |
| Camera mount `T_base_camera` (`base_link` to `oakd_rgb_camera_optical_frame`) | Optical centre **3.7 cm behind** and **18.6 cm above** `base_link`, level (no pitch). Matrix in section 4. Chain: bracket (-0.09, 0, 0.0933), then `oakd_link` (+0.0529, 0, +0.0925), then identity to the RGB frame, then the standard optical rotation. | [bag], recomputed |
| Pose frames | `map` to `odom` (AMCL, only while localization runs) and `odom` to `base_link` (Create 3 wheel odometry), all on `/tb4lite1/tf`. `/tb4lite1/odom` is `odom` to `base_link`. | [bag] |
| Pose source for Hickory | `bag_tf` with `parent_frame: map`, `child_frame: base_link`, `tf_topic_namespace: tb4lite1`. Not `/tb4lite1/amcl_pose`: that is `PoseWithCovarianceStamped`, which Hickory's `bag` type does not accept. | [bag], [inferred] from handoff section 2 |
| Rates seen in the old bags | odom ~20 Hz, `tf` ~37 msgs/s, scan ~7 Hz. | [bag] |
| Nav2 limits (PC's installed defaults) | MPPI `vx_max` 0.5 m/s, `wz_max` 1.9 rad/s (109 deg/s); velocity smoother caps at 0.26 m/s and 1.0 rad/s (57 deg/s); waypoint pause 200 ms. The robot's own config may differ: read it live (section 3). | [install] |
| Slow-run settings already in use | `staleness_demo/between_runs.sh` sets `FollowPath.vx_max 0.2` and a 2 s pause at every waypoint. | [repo] |
| Route | `wps_refined.yaml`: 14 waypoints from the dock out along y of about 0 to x -5.9 (facing -x), a turn at waypoint 3, back along y of about -0.45 (facing +x) to the dock approach. Mapped on `maps/RoboPI_DryLab_corridor.yaml` (25 x 12 m). | [repo] |
| Dock/undock | `./dock.sh undock tb4lite1`, `./dock.sh dock tb4lite1`. | [repo] |
| Discovery | Domain 5, Fast DDS discovery server at `192.168.0.101:11811` (the robot). Interactive shells only for the full graph. | [repo] `setup.bash` |
| Time | NTP only; `sync_time.sh` is broken. | [repo] `CLAUDE.md` |
| PC storage | 202 GB free on `/`. | measured |
| Tools on the PC | `topic_tools`, `image_transport`, `compressed_image_transport`, `compressed_depth_image_transport`, `rosbag2_compression_zstd` installed. The `rosbags` Python package (needed by Hickory's standalone checker) is **not**. | [install] |

Two corrections to the handoff's assumptions: the robot **does** have an RGB-D camera (the Lite URDF includes
the OAK-D Lite), and the handoff's "192 mm overall height" should not be used for planning. What matters for
what the camera can see is its height from tf: 18.6 cm above `base_link`.

## 2. Open items that need the live robot

Settle these in the order shown, in section 3 step 2. Everything below is **[unverified]** until then.

| # | Question | Expected (guess) | How to settle it |
|---|---|---|---|
| O1 | RGB, depth and `camera_info` topic names | `/tb4lite1/oakd/rgb/image_raw`, `/tb4lite1/oakd/rgb/camera_info`, `/tb4lite1/oakd/stereo/image_raw`, `/tb4lite1/oakd/stereo/camera_info`. The `oakd` part is **[inferred]** from the frame names and from `depthai_ros_driver`'s launch file (`<name>/rgb/image_raw`, `<name>/stereo/image_raw`); the rest follows the usual TurtleBot 4 setup. | `ros2 topic list \| grep -i oakd` in an interactive shell |
| O2 | A **full-size** RGB topic exists. The stock `.../rgb/preview/image_raw` is a small preview (about 250 px); Hickory needs about 1280x720 (R1). | `.../rgb/image_raw` and maybe `.../rgb/image_raw/compressed` | `ros2 topic info <topic>`, then `ros2 topic echo --once <topic> --field width` and `--field height` |
| O3 | Depth is 16-bit mm, **same size as the RGB** and registered to it (R2, R3) | `16UC1`; alignment depends on the driver config | `--field encoding`, `--field width/height` on depth; compare with RGB. The checker's `depth != rgb` warning is the confirmation. |
| O4 | Publish rates | probably 15 to 30 Hz | `ros2 topic hz <topic>` (Hickory needs only 4 to 5 Hz, section 5) |
| O5 | RGB `CompressedImage` available? | maybe | `ros2 topic list \| grep compressed` |
| O6 | Where AMCL and Nav2 run, and the clock offset between the Pi and this PC (R6) | PC or Pi; unknown | Section 3 step 3 |
| O7 | Nav2 limits actually in force | as in section 1, or different | `ros2 param get /tb4lite1/controller_server FollowPath.vx_max` and `.wz_max`; `/tb4lite1/velocity_smoother max_velocity` |
| O8 | Free disk on the Pi, and whether it can write a bag while Nav2 runs | unknown | `df -h` on the Pi; only needed if recording there (section 5) |

If O2 or O3 fail (no full-size RGB, depth not registered to RGB or at a different size), that is a **camera
driver configuration change on the Pi**, not something this workspace can do. Get it fixed there before
recording, because Hickory cannot repair unregistered depth.

## 3. Preparation (once)

1. **Bring the system up** as in `staleness_demo/PROCEDURE.md` "Once per session": robot on the dock, Nav2 and
   localization up on `maps/RoboPI_DryLab_corridor.yaml`, RViz open, **2D Pose Estimate** at the dock pose.
   Keep it running for the whole collection: a shared `map` frame between sessions (R11) depends on it.
   ```bash
   source setup.bash && export ROS_SUPER_CLIENT=True
   ```
2. **Settle O1 to O5.** From an interactive shell, with the camera running:
   ```bash
   ros2 topic list | grep -i -E 'oakd|image|camera'
   ros2 topic hz /tb4lite1/oakd/rgb/image_raw        # then the depth topic; Ctrl-C each
   ros2 topic echo --once /tb4lite1/oakd/rgb/image_raw --field header.frame_id
   ros2 topic echo --once /tb4lite1/oakd/rgb/image_raw --field width
   ros2 topic echo --once /tb4lite1/oakd/stereo/image_raw --field encoding
   ```
   Replace the guessed topic names with what O1 shows. Write the answers into section 7's table.
   Expect `header.frame_id` to be `oakd_rgb_camera_optical_frame`; if not, the mount matrix in section 4 is for
   the wrong frame.
3. **Clock check (O6, R6).** Hickory matches frames to poses by timestamp, and a 0.3 s offset at 0.2 m/s is a
   6 cm error. Compare a camera stamp with this PC's clock, and see where AMCL runs:
   ```bash
   ros2 topic echo --once /tb4lite1/oakd/rgb/image_raw --field header.stamp; date +%s.%N
   ros2 node list | grep -i amcl
   ```
   The seconds should agree to well under 0.1 s. If they do not, fix NTP on the Pi or the PC before recording
   (`sync_time.sh` is broken: do not use it). The checker's "header stamps differ from bag receive time" warning
   catches the camera-vs-recorder case later.
4. **Make the speed limits slow enough.** Handoff R7 asks for at most 0.2 m/s and about 30 deg/s. The first is
   already what `between_runs.sh` does; the second is not. After `./staleness_demo/between_runs.sh`:
   ```bash
   ros2 param set /tb4lite1/controller_server FollowPath.wz_max 0.5
   ros2 param get /tb4lite1/controller_server FollowPath.vx_max
   ros2 param get /tb4lite1/controller_server FollowPath.wz_max
   ```
   `vx_max` is set the same way by the existing script, so `wz_max` should behave alike; the `get` confirms it.
   (0.5 rad/s is about 29 deg/s.)
5. **Split the route into two legs** (so the two sessions start from different poses and see the objects from
   opposite directions, as the handoff recommends). Waypoints 0 to 3 are the outbound leg, 4 to 13 the return:
   ```bash
   python3 -I - <<'EOF'
   import yaml
   w = yaml.safe_load(open('wps_refined.yaml'))['waypoints']
   w = [w[f'waypoint{i}'] for i in range(len(w))]
   for name, part in (('hickory_legA', w[:4]), ('hickory_legB', w[4:])):
       yaml.safe_dump({'waypoints': {f'waypoint{i}': p for i, p in enumerate(part)}},
                      open(name + '.yaml', 'w'), sort_keys=False)
   EOF
   ```
   This was run in a scratch directory and gives 4 and 10 waypoints, renumbered from 0. The two files have **not**
   been loaded into the RViz panel yet: load each once with the robot parked and check the markers before
   driving. Leg A ends at waypoint 3 (x -5.88, y -0.24), the far end, where leg B begins.

## 4. Hickory configuration for this robot

For `Hickory/params/TURTLEBOT4/data.yaml` (which is a placeholder template on the `dgx-spark` branch). Everything
here is known, except the camera topic names (O1) and the dataset paths.

```yaml
dt: 0.25
img_data:
  path: "dataset/TURTLEBOT4/run1"
  topic: /tb4lite1/oakd/rgb/image_raw/compressed     # or .../rgb/image_raw with compressed: False; per O1/O5
  camera_info_topic: /tb4lite1/oakd/rgb/camera_info
  compressed: True
depth_data:
  path: "dataset/TURTLEBOT4/run1"
  topic: /tb4lite1/oakd/stereo/image_raw
  camera_info_topic: /tb4lite1/oakd/stereo/camera_info
  compressed: False
pose_data:
  type: bag_tf
  path: "dataset/TURTLEBOT4/run1"
  parent_frame: map
  child_frame: base_link                 # frames carry no namespace prefix on this robot [bag]
  tf_topic_namespace: tb4lite1           # reads /tb4lite1/tf and /tb4lite1/tf_static
  time_tol: 1.0
  T_camera_flu: {input_type: "string", string: "T_RDFFLU"}
  T_odombase_camera:                     # must be a matrix: `tf` input does not forward the namespace (handoff section 2)
    input_type: "matrix"
    matrix: [0, 0, 1, -0.0371,
             -1, 0, 0, 0.0,
             0, -1, 0, 0.1858,
             0, 0, 0, 1]
```
The matrix is `T_base_camera` for `base_link` to `oakd_rgb_camera_optical_frame`, computed from the recorded
`tf_static`. It is the camera pose in the base frame, which is what `T_WC = T_W_base @ T_odombase_camera` needs.
Confirm it on the first real bag: `check_bag_standalone.py ... --extrinsic tf --base-frame base_link
--camera-frame oakd_rgb_camera_optical_frame` prints the matrix, and tests it and its inverse against the
direction of travel. Only the right one passes. The second session uses the same file with `run2`.

If the localization jumps too much (R4), switch `parent_frame` to `odom` (pure wheel odometry, no jumps but
drift). Both come from the same recorded tf, so this is a decision to make offline; do not record twice.

## 5. Recording

**Topics to record** (names per O1):

```
/tb4lite1/tf  /tb4lite1/tf_static  /tb4lite1/odom
<rgb image>  <rgb camera_info>  <depth image>  <depth camera_info>
/tb4lite1/scan  /tb4lite1/amcl_pose        # optional and cheap (about 70 KB/s in the old bags); lets score_bag.py and an AMCL-quality look work on the same bag
```
`tf_static` is latched, so start the recording **after** the robot and camera are up (the old bags caught it
with one message). Do not forget it: without it neither the camera mount nor `oakd_*` frames exist in the bag.

**Rate and size.** Hickory needs only 4 to 5 Hz (handoff section 4). A camera at 15 to 30 Hz recorded in full costs
3 to 6 times the space. At full 1280x720 16-bit depth, 5 Hz is about 9 MB/s, about 3 GB per 5 minutes; 30 Hz
is about 6 times that. Settle the real numbers with a 60 s test (section 6 step 1), not with these.

**Where to record.** `staleness_demo/record_run.sh` records from the PC over DDS, and that is fine for
`scan/odom/tf`. Raw depth is a different load: several MB/s across the Pi-to-PC link, on a robot whose network
`CLAUDE.md` already notes as easy to oversaturate (turtlebot4#574). Order of preference:

1. **Record on the PC first, with a short test.** Compare the bag's per-topic rates (`--list`) with `ros2 topic hz`:
   a lower rate in the bag means messages were dropped. If that happens, or Nav2 starts misbehaving during the
   test, go to 2.
2. **Record on the Pi** (the discovery server runs at 192.168.0.101; SSH details are not in this workspace). Needs O8
   settled, and the same `ROS_DOMAIN_ID`/namespace environment. To reach 4 to 5 Hz, throttle on the Pi, not on the
   PC (a PC-side throttle still pulls full rate over the link):
   ```bash
   ros2 run topic_tools throttle messages /tb4lite1/oakd/rgb/image_raw/compressed 5.0 /hickory/rgb &
   ros2 run topic_tools throttle messages /tb4lite1/oakd/stereo/image_raw 5.0 /hickory/depth &
   ```
   Record the throttled names and put those in `data.yaml`. `camera_info` can be recorded unthrottled (tiny).
   Throttling adds CPU load on a Pi that is also running Nav2: watch for that in the test.
   Compression of the bag (`--compression-mode file --compression-format zstd`) is available on the PC but is
   not measured on this robot's depth, so treat it as a bonus rather than a plan.
3. If raw RGB is the only RGB topic (no `/compressed`, O5), the bag will be dominated by RGB (1280x720x3 is about
   2.8 MB per frame, about 14 MB/s at 5 Hz). Run `ros2 run image_transport republish raw compressed ...` on the Pi, or
   lower the RGB resolution in the camera driver config, before spending disk on it.

A recording command for the PC route, modelled on `record_run.sh`:
```bash
source setup.bash && export ROS_SUPER_CLIENT=True
ros2 bag record -o staleness_demo/bags/hk_legA \
  /tb4lite1/tf /tb4lite1/tf_static /tb4lite1/odom /tb4lite1/scan /tb4lite1/amcl_pose \
  /tb4lite1/oakd/rgb/image_raw/compressed /tb4lite1/oakd/rgb/camera_info \
  /tb4lite1/oakd/stereo/image_raw /tb4lite1/oakd/stereo/camera_info      # names per O1
```
(`record_run.sh` hard-codes its topic list; either copy it and extend the list, or use the command above.)

## 6. Procedure

### 6.1 Test recording, once (30 to 60 s)

1. Robot undocked and still, camera facing the scene. Record about 60 s with the command above under the name
   `hk_test`, driving about 1.5 m forward at 0.1 to 0.2 m/s in the middle of it (the checker needs this to verify the camera axis).
2. Install the checker's dependency in a venv (not in the system Python: Ubuntu 24.04 blocks it) and copy the
   script out of the Hickory checkout (`dgx-spark` branch):
   ```bash
   python3 -m venv ~/venvs/bagcheck && ~/venvs/bagcheck/bin/pip install rosbags numpy opencv-python-headless
   cp Hickory/hickory/utils/check_bag_standalone.py staleness_demo/
   ~/venvs/bagcheck/bin/python staleness_demo/check_bag_standalone.py staleness_demo/bags/hk_test --list --project-minutes 5
   ```
   (`opencv` is only needed if depth is compressed.) `--list` prints the bytes and rate per topic and the projected
   size for 5 minutes. Compare the size with free disk (2x headroom) and the rates with what you expected.
3. Full check, with the names from O1:
   ```bash
   ~/venvs/bagcheck/bin/python staleness_demo/check_bag_standalone.py staleness_demo/bags/hk_test \
     --rgb-topic /tb4lite1/oakd/rgb/image_raw/compressed --depth-topic /tb4lite1/oakd/stereo/image_raw \
     --rgb-info /tb4lite1/oakd/rgb/camera_info --depth-info /tb4lite1/oakd/stereo/camera_info \
     --tf-pose map:base_link --tf-topic /tb4lite1/tf --tf-static-topic /tb4lite1/tf_static \
     --extrinsic tf --base-frame base_link --camera-frame oakd_rgb_camera_optical_frame
   ```
   Interpret warnings with the table in handoff section 5 (every FAIL must be fixed; each WARN understood). In
   particular a `depth != rgb` warning is O3 failing, and the camera-axis test tells you which of the matrix
   and its inverse is right. **The checker has never been run on this robot's data**; treat the first result as
   a test of the checker as much as of the bag.

Do not move on to 6.3 until the test bag has no FAIL.

### 6.2 Scene (before each collection day)

Hickory's filters (handoff R8, R9; `params/TURTLEBOT4/{fastsam,mapper}.yaml`) decide what survives:

- 10 to 25 **distinct floor-level objects**, each at least 25 cm in one dimension, still, not people, not
  reflective or transparent, in even light. Boxes, bins, backpacks, chairs, potted plants, cones. Bottles and
  mugs will be dropped.
- Between **0.7 and 3 m** of the route and **at least 0.5 m apart**. The route spans x -5.9 to -0.4 and y -1.1
  to 0.0 (section 1), so objects belong within about 3 m of that line, on both sides.
- Camera height is 18.6 cm (section 1): it sees the lower part of objects and no tabletops. Prefer objects that
  are fully visible from that height (not tall things that only show their base).
- The corridor objects from the staleness demo, restored to their tape marks, give repeatable placement if they
  meet the criteria above. Use the **unchanged** layout.
- Choose one object and **tape-measure its position** from a fixed floor mark; this is the accuracy check
  (handoff section 6).
- Leg A must show at least 8 objects and leg B at least 8 of the **same** ones (R9, R11): objects near the far end
  and along the corridor centre are seen from both directions.

### 6.3 Per session (leg A, then leg B)

| # | Do | Command / where |
|---|---|---|
| 1 | Localization still up, pose still correct in RViz (the robot sits at the dock on the map). | RViz |
| 2 | Slow settings (clears costmaps too) and the angular cap from section 3 step 4 | `./staleness_demo/between_runs.sh`, then the `wz_max` command |
| 3 | Leg A only: undock | `./dock.sh undock tb4lite1` |
| 4 | Wait for scans, then about 2 s | `ros2 topic echo --once /tb4lite1/scan --field header.stamp` |
| 5 | Start the bag (second terminal), name `hk_legA` or `hk_legB`. Stand still about 5 s, camera facing the scene. | section 5 command |
| 6 | Load the leg's route, then **Start Waypoint Following** | RViz Nav2 panel: Load WPs, `hickory_legA.yaml` (or `legB`) |
| 7 | Arrived at the leg's last waypoint: stay still about 3 s, then stop the bag. **Do not dock or touch anything between legs.** | Ctrl-C in the bag terminal |
| 8 | Leg A done: **leave the robot where it is** (waypoint 3), keep localization and Nav2 up, and go to step 5 for leg B. Leg B done: dock. | `./dock.sh dock tb4lite1` |
| 9 | Run the checker on both bags (section 6.1 step 3, full check). Log both in `staleness_demo/bags/runs.txt`: date, objects, anything odd (bumps, AMCL jumps, people in view). | |

If anything breaks during a leg (a bump, a relocalization jump seen in RViz, the robot stalling for more than
a few seconds, someone walking through), discard that leg's bag and repeat it. Do not repeat from the dock
unless leg A is the one that failed.

Why this ordering: the shared frame is `map` from AMCL on one map, localized once at the start. Rebooting the robot or
re-running the 2D Pose Estimate between legs breaks that (R11).

## 7. Acceptance, and what to hand back

A leg is usable for Hickory when all of this holds (items refer to the handoff):

- [ ] Checker: no FAIL; RGB and depth the same size and registered (R2), `camera_info` matches the images (R3),
  stamps consistent (R6), no `pose jump` (R4), camera axis follows the direction of travel (R5).
- [ ] Duration 3 to 6 minutes (R12) and speed within the limits (R7); depth usable (about 30% or more of pixels in range, 0.7 to 3 m).
- [ ] 10 to 25 qualifying objects in view (R8, R9); leg A and leg B share at least 8.
- [ ] Both bags are full rosbag2 directories with `metadata.yaml`, including `tf_static` (R10).
- [ ] Positions: one tape-measured object, and the log entry.

Then copy the bag directories to the Spark as `dataset/TURTLEBOT4/run1/` and `run2/` and follow handoff section 7
(`check_bag.py` in the container first, then `main.py ... --limit 5`, and inspect `obj_*/view.png` and `mask.png`
before a full run). Send back with the bags:

| Item | Value |
|---|---|
| RGB / depth / info topic names, RGB size, depth encoding and size (O1 to O3) | |
| Rates recorded (O4), throttling used | |
| Filled-in `data.yaml` (section 4), and which pose parent frame (`map` or `odom`) | |
| Clock offset result (O6), where AMCL runs | |
| Nav2 limits in force (O7) | |
| Where it was recorded (PC or Pi) and the checker output | |
| The tape-measured object's position in `map` | |

## 8. Known gaps in this document

- Camera topics, resolution, depth registration and rates are guesses until section 3 step 2 is done (section 2).
- `hickory_legA.yaml` and `hickory_legB.yaml` are generated mechanically from `wps_refined.yaml` and have not been
  driven. Nothing here has been run against the live robot.
- Whether `FollowPath.wz_max` is accepted at runtime is expected from how `vx_max` behaves, not observed.
- Pi-side recording needs SSH access and the Pi's disk space, neither of which this workspace documents.
- The checker (`check_bag_standalone.py`) has been tested by the Hickory side on synthetic bags only.
