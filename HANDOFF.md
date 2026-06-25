# HANDOFF.md

Handoff for the P-AgBot vision-based row-centering controller work. Written so a fresh Claude Code session can continue with zero context loss.

## 1. GOAL

Build a ROS1 node for the Purdue P-AgBot (a Clearpath Jackal UGV) that takes the live output of an already-trained DINOv3 segmentation model (classes: sky/traversable/obstacle) on a forward camera feed, and publishes `cmd_vel` so the robot drives down the center of a corn row — the vision-based analog of the lab's already-deployed LiDAR row-centering controller. The user's professor assigned this as a field-test/integration task; prior students built the robot platform and the segmentation training pipeline, but nobody has closed the loop into an actual ROS controller yet. The immediate, narrowly-scoped milestone (explicitly set by the user) is: **drive straight down a perfect, straight simulated corn row in Gazebo** — no turning/U-turns yet.

## 2. CURRENT STATE

**Done and verified (in this sandbox):**
- New catkin package `agbot_vision_nav/` created at `/home/weichien241/ag_bot/src/agbot_vision_nav/`, fully coded (package skeleton, all Python modules, launch file, config, tests).
- All 22 unit tests pass: `cd /home/weichien241/ag_bot/src/agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v`
- `python3 -m py_compile scripts/vision_nav_node.py` succeeds (syntax-only check; cannot actually run it here — see GOTCHAS).
- Launch XML, `package.xml`, and `config/params.yaml` all validated as well-formed (parsed with `xml.dom.minidom` / `yaml.safe_load`).
- `/home/weichien241/ag_bot/src/CLAUDE.md` written (repo-wide guidance for future Claude sessions — covers the whole repo, not just this controller).

**Not done / not verified (needs the user's actual WSL ROS1 Noetic machine):**
- `lightly_train`/`torch` have **never been run outside Google Colab**. Whether they even install on the user's ROS1 Noetic Python (3.8 on Ubuntu 20.04) is completely unknown. This is the single biggest open risk — see NEXT STEPS #1 and GOTCHAS.
- The package has never been `catkin_make`/`catkin build`-ed (impossible in this sandbox — see GOTCHAS).
- `vision_nav_node.py` has never actually run against rospy/cv_bridge/a real or simulated camera topic.
- No Gazebo/Jackal simulation exists yet anywhere — the user is setting that up themselves, separately, on their own machine.
- Bag-replay test (using the user's own already-recorded rosbags) not yet attempted.

## 3. KEY DECISIONS

These were deliberated and approved — don't re-litigate without new information:

- **Controller is a plain P-controller on a scanline-midpoint image offset, NOT MPC.** The lab's advisor explicitly said MPC is the long-term goal but "you can use a controller that seems fit" for this first pass. A P-controller is the simplest design that isn't obviously worse, matches the advisor's sanction, and is the direct image-space analog of the lab's already-validated LiDAR `d_l`/`d_r`-balancing approach (see `Papers/1_P-AgBot_...md` Section III). Bang-bang was rejected (visibly oscillates, teaches nothing toward an MPC upgrade). Jumping straight to MPC was rejected as unwarranted complexity for "drive straight in a straight row."
- **Centerline algorithm**: adapted from the Agronav paper's per-scanline boundary-midpoint definition (`Papers/Agronav...md`, Sec 4.3): at fixed image rows, scan outward from the image's vertical centerline column until hitting a non-traversable pixel; the per-row midpoint is the lane center; average across rows weighted toward the nearest (most reliable, least foreshortened) row. ROW-SLAM's paper explicitly found single-frame vanishing-point/line-fit heading estimation to be the LEAST accurate baseline they tested — this is why we avoid line-fitting/vanishing-point heading and use direct per-scanline midpoints instead.
- **Sign convention** (locked in by a unit test, not just eyeballed): image x increases rightward. If the traversable centerline is left of image center, `offset_norm < 0`, and the robot must turn left = positive `angular.z` under ROS REP-103. Hence `angular_z = -(k_p * offset_norm + k_slope * slope_term)`. This is asserted in `test/test_controller.py::test_sign_convention_negative_offset_turns_left_positive_angular_z`.
- **`slope_term` exists but is disabled by default** (`k_slope=0.0`). It's a free-to-compute cross-row heading proxy (far-row offset minus near-row offset), included so the data the controller would need for an MPC upgrade later already exists, without building the MPC now. Enable only as a deliberate, separately-tested second experiment.
- **Known, accepted limitation**: if the robot drifts farther than the visible corridor's half-width, the image centerline column falls entirely outside the corridor, and `centerline_estimator.py` reports "no signal" for that row rather than guessing a wrong number. This is intentional — recovery from large drift is the job of the invalid-frame-counter/watchdog stop logic, not the offset math. Documented and tested in `test/test_centerline_estimator.py::test_drift_beyond_corridor_width_is_invalid`.
- **All perception/control logic is kept rospy-free** (`segmentation_model.py`, `centerline_estimator.py`, `controller.py`, `debug_viz.py` have zero `rospy` imports — only `scripts/vision_nav_node.py` touches ROS types). This was a deliberate design choice so the algorithmic core is unit-testable without ROS, `lightly_train`, or even a GPU, and so that if `lightly_train` turns out to be incompatible with the ROS1 Python env, only `segmentation_model.py` needs to move into a separate process — the rest is untouched.
- **Inference runs in a separate thread, decoupled from the camera callback** (single-slot frame buffer guarded by a `threading.Condition`, overwrite-not-queue). This was built in from day one, not retrofitted, because inference latency on the user's WSL2 box (CPU vs. possible GPU passthrough) is completely unmeasured — a slow model must never be allowed to back up the subscriber's callback queue.
- **Camera message format is a parameter, not an assumption**: `camera_topic_is_compressed` (bool param) because the real robot publishes `sensor_msgs/CompressedImage` on `/usb_cam/image_raw/compressed`, but Gazebo's camera plugin will very likely publish raw `sensor_msgs/Image` instead. Must be set correctly per environment at launch time.
- **`model_path` has no default value** — required, fails loudly if unset, since the `.pt` file lives outside any catkin package and a fragile relative-path guess would be worse than an explicit error.
- **User preference**: wants each file edit manually approved during implementation (not auto-accepted) — respect this in future sessions unless they say otherwise.

## 4. FILES

### New package: `/home/weichien241/ag_bot/src/agbot_vision_nav/`

| Path | Contents |
|---|---|
| `package.xml`, `CMakeLists.txt` | Catkin package manifest/build file. Python-only package: `catkin_python_setup()` is enabled in CMakeLists so `src/agbot_vision_nav/*.py` is importable. Depends on `rospy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `cv_bridge`. Modeled on `rgb_camera-main/package.xml` and `rgb_camera-main/CMakeLists.txt`'s pattern. |
| `setup.py` | Standard catkin Python package setup (`generate_distutils_setup`), required alongside `catkin_python_setup()`. |
| `src/agbot_vision_nav/__init__.py` | Empty, makes the directory a package. |
| `src/agbot_vision_nav/segmentation_model.py` | `class SegmentationModel`: wraps `lightly_train.load_model(model_path)` / `.predict()`. `predict(frame_bgr)` returns a `(H,W)` uint8 class-index mask, force-resized to match the input frame via `cv2.resize(..., interpolation=cv2.INTER_NEAREST)` if `lightly_train`'s output resolution doesn't already match. **Cannot be unit-tested in this sandbox** (no `lightly_train`/`torch` installed here). Has a docstring with a standalone smoke-test snippet to run on the target machine. |
| `src/agbot_vision_nav/centerline_estimator.py` | Pure numpy. `CLASS_SKY=0, CLASS_TRAVERSABLE=1, CLASS_OBSTACLE=2`. Function `estimate_centerline(mask, scan_row_fractions=(0.65,0.78,0.92), scan_row_weights=(0.2,0.3,0.5), min_traversable_fraction=0.10) -> CenterlineResult` (namedtuple: `offset_norm, slope_term, valid, traversable_fraction, scan_rows`). Internal helper `_scan_row_boundaries(row, cx)`. |
| `src/agbot_vision_nav/controller.py` | `class RowCenteringController(k_p=1.0, k_slope=0.0, linear_x_cruise=0.15, angular_z_max=0.3, invalid_frame_stop_count=5)`. Method `compute(offset_norm, slope_term, valid) -> (linear_x, angular_z)` (plain floats, not `Twist`). Method `reset()`. Property `consecutive_invalid`. |
| `src/agbot_vision_nav/debug_viz.py` | `render_debug_image(frame_bgr, mask, centerline_result, linear_x=None, angular_z=None, alpha=0.5) -> annotated BGR image`. Green traversable overlay (matches `Testing_Segmentation.py`'s convention), scan-row lines, midpoint markers, center line, text readout. |
| `scripts/vision_nav_node.py` | **Only file touching rospy/cv_bridge.** `class VisionNavNode`: subscribes to camera topic (`CompressedImage` via `cv2.imdecode` or raw `Image` via `cv_bridge`, chosen by `~camera_topic_is_compressed`), single-slot frame buffer (`threading.Condition`), separate `_inference_loop` thread, publishes `Twist` to `~cmd_vel_topic` and debug `Image` to `~debug_image_topic`, 5 Hz watchdog (`rospy.Timer`) zeroes velocity if stale beyond `~max_data_age_sec`. Entry point `main()`. Executable (`chmod +x` already applied). |
| `config/params.yaml` | All default params (see table in NEXT STEPS / below). `model_path` deliberately absent (required, no default). |
| `launch/vision_nav.launch` | Loads `config/params.yaml`, exposes `model_path` (required arg, no default), `camera_topic`, `camera_topic_is_compressed`, `cmd_vel_topic` as launch args/overrides. |
| `test/test_centerline_estimator.py` | 9 tests: centered/shifted-left/shifted-right corridor offset correctness, tapered-corridor slope_term sign, drift-beyond-corridor-width → invalid, all-sky/all-obstacle → invalid, mismatched-length ValueError, low-traversable-fraction → invalid. |
| `test/test_controller.py` | 11 tests: sign convention both directions, zero-offset-straight, clamping, slope_term sign + disabled-by-default, invalid-frame hold-then-stop state machine, reset behavior, first-frame-invalid-with-no-history. |
| `test/test_debug_viz.py` | 2 smoke tests: valid-frame shape/dtype, invalid-frame doesn't crash. |

### Repo-wide reference (pre-existing, not modified)

| Path | Relevance |
|---|---|
| `/home/weichien241/ag_bot/src/CLAUDE.md` | Just-written repo guidance for future Claude sessions — read this first in a fresh session, it covers the whole repo (DINOv3 pipeline commands, ROS1/ROS2 environment split, `agbot_vision_nav` architecture summary). |
| `DINOv3-Segmentation-Training/Testing_Segmentation.py` | The canonical, already-working reference for the model's inference contract (`lightly_train.load_model()`/`.predict()`, BGR→RGB→PIL→mask). `segmentation_model.py` mirrors this exactly. |
| `DINOv3-Segmentation-Training/Train.py` | Shows `model="dinov3/vits16-eomt"`, `transform_args={"image_size":(224,224)}`, and the authoritative `classes={0:"sky",1:"traversable",2:"obstacle"}` mapping. Has Windows paths (`C:/Users/Paul/...`) — training happened on Colab/Windows, never on the ROS1 box. |
| `DINOv3-Segmentation-Training/Convert_type.py` | Confirms class mapping again (`class_mapping = {1:0, 2:1, 3:2}` from COCO category id → mask class id). |
| `DINOv3-Segmentation-Training/out/corn_field_navigation/exported_models/exported_best.pt` | The trained model (~89MB). User mentioned there may be a newer/better version trained on additional video — `model_path` param lets you point at whichever one without code changes. |
| `rgb_camera-main/package.xml`, `rgb_camera-main/CMakeLists.txt` | Pattern that `agbot_vision_nav`'s package skeleton was copied from. |
| `rgb_camera-main/launch/usb_cam_launch.launch` | Real robot's camera launch — publishes `/usb_cam/image_raw/compressed`, 320x240 raw → republished as `CompressedImage`. (Note: training images/masks are 640x480 — there may be a resolution mismatch between this launch file's raw camera resolution and the dataset; not yet investigated.) |
| `ros_bags/2026-06-11-11-47-03.bag` (and 3 others) | Real recorded footage on topic `/usb_cam/image_raw/compressed`, used for the bag-replay test (NEXT STEPS #2). |
| `Papers/1_P-AgBot_...md` | Section III: the LiDAR `d_l`/`d_r` row-centering approach this controller's design mirrors. |
| `Papers/Agronav Autonomous Navigation Framework for Agricultural Robots.md` | Section 4.3: per-scanline boundary-midpoint centerline definition, directly adapted into `centerline_estimator.py`. |
| `Papers/ROW-SLAM Under-Canopy Cornfield Semantic SLAM.md` | Table III: vanishing-point/line-fit heading was their least-accurate baseline — justification for avoiding that approach here. |
| `/home/weichien241/.claude/plans/background-i-am-smooth-ritchie.md` | The full approved plan with the complete design rationale, default-parameter justification table, and verification plan. Read this for any deeper "why" not captured here. |

## 5. NEXT STEPS (priority order)

1. **Model environment check — do this first, on the user's WSL Noetic machine, outside ROS entirely.** In whatever Python env will run `vision_nav_node.py` (system Python 3.8, or a venv with `--system-site-packages` so `rospy` stays importable), run:
   ```python
   pip install lightly_train
   # then:
   from agbot_vision_nav.segmentation_model import SegmentationModel
   import cv2
   m = SegmentationModel('/path/to/exported_best.pt')
   frame = cv2.imread('/path/to/a/640x480/frame.jpg')  # e.g. from dataset_v2/train/images/
   mask = m.predict(frame)
   print(mask.shape, mask.dtype, set(mask.flatten().tolist()))
   ```
   - **If it works**: in-process inference is viable, proceed to step 2.
   - **If it fails** (e.g. Python 3.8 incompatibility): pivot to running `segmentation_model.py` standalone in its own venv (newer Python, just needs `torch`/`lightly_train`/`opencv`, no `rospy`), publishing the mask as a `mono8` `sensor_msgs/Image` on its own topic; modify `vision_nav_node.py` to subscribe to that mask topic instead of calling `SegmentationModel` directly. Because the module boundary is already clean (zero rospy imports in `segmentation_model.py`), this is a small, contained change.
2. **`catkin_make`/`catkin build` the workspace** from `~/ag_bot` (the workspace root, one level above this `src/`), then `source devel/setup.bash`. Fix any manifest issues that surface (none anticipated, but unverified since this sandbox has no ROS1).
3. **Bag-replay test** — highest-value test before touching Gazebo, and has zero dependency on the user's still-in-progress sim setup:
   ```
   roslaunch agbot_vision_nav vision_nav.launch model_path:=<path> camera_topic:=/usb_cam/image_raw/compressed camera_topic_is_compressed:=true
   rosbag play ros_bags/2026-06-11-11-47-03.bag
   rqt_image_view   # watch /vision_nav_node/debug/image
   rostopic echo /cmd_vel   # or rqt_plot the angular.z field
   ```
   Cross-check against `videos/2026-06-11-11-47-03.mp4` — `angular.z` should be small/low-variance when the recorded robot was visibly centered, and react in the geometrically correct direction during any visible drift.
4. **Gazebo integration** (the user is setting up the Jackal + cornfield world themselves, separately): once their sim publishes a camera topic, update the `camera_topic`/`camera_topic_is_compressed` launch args to match it (almost certainly raw `Image`, `camera_topic_is_compressed:=false`). Before trusting sustained behavior: manually drive the simulated robot off-center with teleop (still facing straight), launch the node, and confirm it commands a turn *toward* re-centering — this is the empirical, in-sim version of the sign-convention check already locked by the unit test.
5. **Tune `k_p`** (`config/params.yaml`, currently `1.0`) empirically once real closed-loop behavior is observable — this is explicitly a starting guess, not a derived value.
6. Only after a successful sustained straight-line run: revisit turning/U-turn logic (explicitly out of scope until now) and consider enabling `k_slope` as a separate, deliberately-tested experiment.

## 6. GOTCHAS

- **This sandbox cannot run ROS1 anything.** It has ROS2 Humble installed/sourced, not ROS1 Noetic — confirmed directly by the user. Do not attempt `catkin_make`, `roslaunch`, `rospy` imports, or `rosbag` commands here; they will fail or give misleading results. All ROS1 execution/verification must happen on the user's WSL Ubuntu 20.04 / ROS1 Noetic machine.
- **`lightly_train`/`torch` are not installed in this sandbox either** — `segmentation_model.py` cannot be imported or tested here. Don't waste time trying; that module's correctness rests on matching `Testing_Segmentation.py`'s already-proven contract, not on sandbox execution.
- **The model has only ever run on Google Colab, never locally on any machine the user controls for ROS.** Do not assume `pip install lightly_train` will "just work" on Noetic's Python 3.8 — this is a real open risk, not a checked box. See NEXT STEPS #1.
- **Synthetic-mask test pitfall already hit once**: when writing `test_centerline_estimator.py`, an early version used a corridor shift (40px) larger than the corridor's half-width (30px), which pushed the image centerline column entirely outside the corridor — `estimate_centerline()` correctly returned "invalid" (no signal), and the test's assumption that the offset would just be computed was wrong, not the code. Fixed by using a smaller, realistic shift (15px) for the "small drift" tests, and adding a dedicated test (`test_drift_beyond_corridor_width_is_invalid`) documenting the large-drift behavior as expected, not a bug. If you see similar "valid=False" surprises, check whether the test's geometry actually keeps the image-center column inside the traversable region — `_scan_row_boundaries()` requires that by design.
- **Sandbox's system Python is 3.10** (`/usr/bin/python3`), used to run the unit tests here via `PYTHONPATH=src python3 -m pytest test/`. This is NOT the same Python the ROS1 node will run under (Noetic = Python 3.8) — don't assume parity; re-run the test suite on the target machine too if anything seems off.
- **Resolution mismatch, not yet resolved**: `rgb_camera-main/launch/usb_cam_launch.launch` configures the camera at 320x240, but the training dataset images (`DINOv3-Segmentation-Training/dataset_v1/`, `dataset_v2/`) are 640x480. Unclear whether this is a stale launch file, a different camera, or a resize step that happens elsewhere. Worth checking before trusting real-robot inference resolution end-to-end.
- **`out/corn_field_navigation/exported_models/exported_best.pt` is overwritten every time `Train.py` runs** (per its own README warning) — if the user mentions "a newer version trained on more video," confirm which physical `.pt` file they mean before wiring up `model_path`, since the file may have been moved/renamed to avoid being clobbered.
- **User preference**: wants individual edits manually approved (not auto-accept) during implementation — ask/expect approval prompts, don't be surprised by them or try to work around them.
- **Don't re-propose MPC, an EKF, or speed-tapering for v1** — these were explicitly considered and deferred (see KEY DECISIONS). They're documented upgrade paths, not omissions to "fix."
- To run the full existing test suite from a fresh session:
  ```
  cd /home/weichien241/ag_bot/src/agbot_vision_nav
  PYTHONPATH=src python3 -m pytest test/ -v
  ```
  Expect: 22 passed.
