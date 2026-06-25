# Vision-Based Row-Centering Controller (`agbot_vision_nav`)

## Context

The P-AgBot lab has already proven autonomous in-row navigation using 2D/3D LiDAR (P-AgBot, P-AgNav papers). The grad students now want to add a camera-based AI navigation module because the LiDAR's sparse point cloud has little inherent scene understanding (it can't tell sky from plant, for instance), and the vision module should be able to handle situations where the LiDAR-only approach struggles. A prior student already built and trained a DINOv3-based semantic segmentation model (`sky` / `traversable` / `obstacle`) on real corn/sorghum row footage recorded from the robot's camera, and the user has confirmed the trained model (`exported_best.pt`) produces good-looking segmentation video offline.

The user's assigned task is to close the loop: write a ROS1 node that takes the live segmentation mask and publishes `cmd_vel` so the Jackal drives down the center of a row, the same way the LiDAR controller already keeps `d_l`/`d_r` balanced, but using image-space geometry instead of LiDAR ranges. The grad students were explicit that a full MPC (as used in the lab's other papers) is the long-term goal but not required for this first pass ("you can use a controller that seems fit"). The immediate milestone is narrow and explicitly scoped by the user: get the robot to drive straight down a perfect, straight simulated corn row in Gazebo. Turning/U-turns are explicitly deferred. The user is separately handling spawning the Jackal + cornfield world in Gazebo and confirming the camera topic — this plan covers only the controller node itself, designed to be agnostic to exactly how the image arrives (sim vs. real robot).

Three research passes (repo/workspace exploration, model inference code inspection, and the lab's own papers) plus an architecture validation pass converged on one design, detailed below.

## Key established facts

- **Workspace**: `/home/weichien241/ag_bot/src` is used as a catkin workspace's `src/` dir — packages sit directly inside it (e.g. `rgb_camera-main/`, a ROS1 package wrapping `usb_cam`), though `catkin_make`/`catkin build` hasn't been run yet. The new package will be a sibling directory: `/home/weichien241/ag_bot/src/agbot_vision_nav/`.
- **Real robot's camera topic** (for later field use): `/usb_cam/image_raw/compressed`, `sensor_msgs/CompressedImage`, 640x480. Gazebo's camera plugin will likely publish raw `sensor_msgs/Image` instead — the node must support both via a param, since this is exactly the kind of sim-vs-real divergence that's easy to get bitten by.
- **Model** (`DINOv3-Segmentation-Training/`): DINOv3 ViT-S/16 + EoMT head, trained via `lightly_train.train_semantic_segmentation(...)` at `image_size=(224,224)`. Exported to `out/corn_field_navigation/exported_models/exported_best.pt` (~89MB) automatically by `lightly_train` — no separate export script exists.
- **Inference contract**, confirmed directly from `DINOv3-Segmentation-Training/Testing_Segmentation.py:1-56` (already working code, read in full):
  ```python
  model = lightly_train.load_model(MODEL_PATH); model.eval()
  frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
  mask = model.predict(Image.fromarray(frame_rgb))   # class-index mask, NOT raw logits — predict() argmaxes internally
  if torch.is_tensor(mask): mask = mask.cpu().numpy()
  mask = np.squeeze(mask).astype(np.uint8)
  ```
  `predict()`'s exact output resolution (224x224 vs. upsampled to input size) is unverified — the original script's own debug print (`Testing_Segmentation.py:34-38`) shows even the original author didn't hard-code an assumption here. Our design must not assume either way (see Risk A below).
- **Class mapping**, confirmed from `Train.py`, `Convert_type.py:19-28`, `README.md:20`, and empirically from real mask files: **0 = sky, 1 = traversable, 2 = obstacle/untraversable**.
- **Existing viz convention**: traversable colorized green `[0,255,0]` BGR, `cv2.addWeighted` alpha=0.5 (`Testing_Segmentation.py:51-54`) — reuse this for the new debug overlay so it looks familiar.
- **Control-law precedent**: the lab's deployed LiDAR controller centers the robot by minimizing `d_l - d_r` (min distances left/right of robot). The Agronav paper's per-scanline boundary-midpoint centerline definition is the direct image-space analog of this. P-AgNav's pixel-offset-to-MPC structure (`x_r = x_t - x_d`) and CropFollow's EKF+MPC are richer alternatives but need either a different sensor geometry (P-AgNav: 360° LiDAR range-view) or a different model output (CropFollow: a `(heading, distance-ratio)` regression head we don't have) — not directly reusable, but their *structure* (lateral offset error → velocity command, optional heading term) is.
- **Confirmed, higher risk than initially assumed**: `lightly_train`/`torch` have so far only ever been run on **Google Colab** to produce `exported_best.pt` (training and the working offline test both happened there) — they have never been run on the user's WSL Noetic laptop at all, in any environment. This is a real open risk, not just an unverified assumption: ROS1 Noetic on Ubuntu 20.04 ships system **Python 3.8**, and it's unconfirmed whether `lightly_train`'s dependencies (and whatever `torch`/CUDA build matches the user's WSL2 GPU situation, if any) even support Python 3.8 — Colab environments typically run a much newer Python. The user also mentioned having trained more than one `.pt` version (a newer one trained on additional video performed well) and wants to be able to pick which exported model to use — the design already supports this for free since `model_path` is a required, explicit param with no hardcoded default. Design must default to attempting in-process inference but keep the clean swap-out seam (described below) ready, since the separate-process fallback is now a plausible real path, not a remote contingency.

## Recommended architecture

A pure proportional controller on a scanline-midpoint image offset — the simplest design that isn't obviously worse than the alternatives, matches the advisor's explicit sanction of "a controller that seems fit," and is structurally the direct image-space analog of the lab's already-validated LiDAR `d_l`/`d_r`-balancing approach. Bang-bang control would be simpler but visibly hunts/oscillates and teaches nothing toward the MPC upgrade path; jumping straight to MPC is unwarranted complexity for "drive straight in a straight row" and isn't what was asked for in this milestone.

**Algorithm** (`centerline_estimator.py`): at 3 scan rows in the lower-middle portion of the frame (fractions of image height, e.g. `[0.65, 0.78, 0.92]` — avoids the sky/horizon entirely and avoids the very bottom edge), scan outward from the image's vertical centerline (`cx = W/2`) left and right through the mask until hitting a non-traversable pixel, giving per-row `x_left`/`x_right`; the per-row midpoint is the centerline point for that row (this is literally Agronav's definition, adapted from line-fit boundaries to direct mask-pixel boundaries). Average the per-row midpoints, weighted toward the nearest row (most geometrically reliable, least foreshortened — and avoids the single-frame vanishing-point/line-fit heading estimate that the lab's own ROW-SLAM paper found to be their least accurate baseline). `offset_norm = (x_center_avg - cx) / (W/2)`, clamped to `[-1, 1]`. Also compute a validity check: if the fraction of traversable pixels in the lower half of the frame falls below a threshold, mark the frame invalid (lost the row / occlusion / not yet in a row).

**Control law** (`controller.py`): `angular_z = -k_p * offset_norm`, clamped to `[-angular_z_max, angular_z_max]`; `linear_x = linear_x_cruise` constant (no speed-tapering yet — deferred as a documented future tweak, not built preemptively, since the first milestone is a straight perfect row where large corrections shouldn't be needed). Sign convention, locked in by a unit test before any sim run: image x increases rightward; if the traversable centerline appears left of image center, `offset_norm < 0`, and the robot must turn left, which is positive `angular.z` under ROS/REP-103 convention — so `angular_z = -k_p * offset_norm` is correct. A small state machine tracks consecutive invalid frames; after `invalid_frame_stop_count` (default 5) consecutive invalid/stale frames, publish zero `Twist` instead of extrapolating blindly.

**Model wrapper** (`segmentation_model.py`): wraps `lightly_train.load_model()`/`.predict()` exactly as `Testing_Segmentation.py` does, but additionally guarantees a fixed contract to the rest of the pipeline — always returns a mask the same `(H, W)` as the input frame, resizing with `cv2.resize(..., interpolation=cv2.INTER_NEAREST)` if `predict()`'s raw output doesn't already match (nearest-neighbor is mandatory since these are class indices, not continuous values). This makes the unresolved 224x224-vs-640x480 question (Risk A) a non-issue for every other module, which can assume `mask.shape == frame.shape` unconditionally.

**Node lifecycle** (`vision_nav_node.py`, the only file touching `rospy`/`cv_bridge`): the camera subscriber (queue_size=1) only stashes the latest frame (single-slot, lock-protected, overwrite-not-queue); a separate thread loop grabs the freshest frame, runs the model + centerline math + controller, and publishes `cmd_vel` right after each successful inference (event-driven, not a fixed-rate timer — there's only one signal source, so a second timer would just add a place for staleness to hide). A watchdog checked at 5 Hz publishes zero `Twist` if `max_data_age_sec` (default 0.5s) is exceeded. This decoupling is built in from day one (not retrofitted later) because inference latency on the user's WSL2 box (CPU vs. GPU passthrough) is unmeasured, and a slow model must never let the camera-callback queue back up.

**Debug visualization** (`debug_viz.py`): publishes an annotated BGR image (green traversable overlay matching the existing convention, scan-row lines, per-row midpoint markers, image-center line, `offset_norm`/`angular_z` as text) on `~debug/image` for `rqt_image_view` — important given the user has no low-level controller background and will be tuning gains by observation.

**Design principle carried through every module**: `segmentation_model.py`, `centerline_estimator.py`, and `controller.py` have zero `rospy` imports. Only `vision_nav_node.py` and `debug_viz.py`'s ROS-message-construction wrapper touch ROS types. This means the actual control-law and perception logic can be unit-tested with synthetic numpy arrays in plain `pytest`, independent of having ROS, `lightly_train`, or even a GPU available — useful both for fast iteration and because this very environment (no ROS1, no `lightly_train` installed) can still run those tests directly. It also means that if `lightly_train`/`torch` turns out not to coexist with `rospy` in one Python env, only `segmentation_model.py` needs to move into a separate process (publishing a `mono8` mask image on its own topic) without touching the control-law modules at all — the seam is already clean.

## Files to create

```
agbot_vision_nav/
  package.xml, CMakeLists.txt      # copy rgb_camera-main's pattern (read in full: rgb_camera-main/package.xml,
                                    # rgb_camera-main/CMakeLists.txt) — Python-only variant: uncomment
                                    # catkin_python_setup(), depend on rospy/sensor_msgs/geometry_msgs/cv_bridge
                                    # instead of roscpp/usb_cam
  setup.py                          # required alongside catkin_python_setup() for src/agbot_vision_nav/ to be importable
  launch/vision_nav.launch          # loads config/params.yaml, exposes camera_topic/cmd_vel_topic as remap args
  config/params.yaml                # see defaults table below
  scripts/vision_nav_node.py        # executable rospy entry point (chmod +x, #!/usr/bin/env python3)
  src/agbot_vision_nav/
    __init__.py
    segmentation_model.py           # wraps lightly_train.load_model/.predict + shape-normalization resize
    centerline_estimator.py         # mask -> (offset_norm, slope_term placeholder, valid, per-row debug info)
    controller.py                   # (offset_norm, valid) -> (linear_x, angular_z), invalid-frame state machine
    debug_viz.py                    # mask + frame + centerline result -> annotated BGR image
  test/
    test_centerline_estimator.py    # synthetic masks: centered/shifted-left/shifted-right corridor, all-sky/all-obstacle
    test_controller.py              # sign-convention lock-in test (offset_norm=-0.5 -> angular_z > 0), clamping, invalid-count stop
```

**Recommended order, given the model-environment risk above is now the biggest unknown:** do the standalone environment check (Verification step 2 below) **first**, before writing any ROS code — it's cheap, decoupled from everything else, and its outcome (in-process vs. separate-process inference) determines whether `vision_nav_node.py`'s subscriber wiring needs to call `segmentation_model.py` directly or instead subscribe to a mask topic published by a separate script. Everything else can proceed in parallel/after: `centerline_estimator.py` → its unit tests → `controller.py` → its unit tests (zero dependencies beyond numpy, fully verifiable right now, in this very sandbox) → `segmentation_model.py` (cannot be unit-tested here; no `lightly_train` installed in this sandbox) → `debug_viz.py` → `vision_nav_node.py` wiring everything together → `launch`/`config`.

## Default parameters (starting points, to be tuned empirically in sim)

| Param | Default | Why |
|---|---|---|
| `camera_topic` / `camera_topic_is_compressed` | `/usb_cam/image_raw/compressed` / `true` | Matches real robot; **must be overridden via launch arg for Gazebo**, whose camera plugin likely publishes raw `Image` instead |
| `cmd_vel_topic` | `/cmd_vel` | Jackal convention |
| `model_path` | required, no default | Lives outside any catkin package — fail loudly if unset rather than guessing a fragile relative path |
| `scan_row_fractions` | `[0.65, 0.78, 0.92]` | Lower-middle-to-bottom of frame; skips sky/horizon and the very bottom edge |
| `scan_row_weights` | `[0.2, 0.3, 0.5]` | Weight nearest row highest (most reliable) |
| `k_p` | `1.0` | `offset_norm` is pre-normalized to [-1,1]; this is the #1 tuning knob |
| `linear_x_cruise` | `0.15 m/s` | Conservative crawl for the very first closed-loop sim run |
| `angular_z_max` | `0.3 rad/s` | Safety clamp during tuning |
| `max_data_age_sec` | `0.5 s` | Stale-frame cutoff for the watchdog |
| `min_traversable_fraction` | `0.10` | Permissive v1 validity threshold; raise once real mask outputs are observed |
| `invalid_frame_stop_count` | `5` | Tolerate a few noisy frames before E-stopping |

## Verification plan

**Note on this sandbox**: this Claude Code terminal only has ROS2 (Humble) installed/sourced, not ROS1 — confirmed by the user. I can write and edit all the files in this plan here, and I can run the rospy-free unit tests (step 2 below) here since they need only `numpy`/`pytest`. But I cannot run `catkin_make`, `rospy`, `roslaunch`, `rosbag play`, or anything else that needs an actual ROS1 environment from this terminal — those steps (1, 3, 4 below, plus building the catkin package itself) need to be run by the user on their WSL Ubuntu 20.04 / ROS1 Noetic machine. I'll flag clearly at each step whether it's something I can execute here or something that needs to happen on your machine.

1. **Standalone model environment check — do this first** (user's WSL laptop, outside ROS entirely): in whatever Python environment is intended to run `vision_nav_node.py` (the system Python 3.8 that ROS1 Noetic uses, or a venv with `--system-site-packages` so `rospy` stays importable), try `pip install lightly_train` and run `model.predict()` on one real 640x480 frame, printing `mask.shape`/`dtype`/`np.unique(mask)`. Two concrete outcomes determine the rest of the build:
   - **It installs and runs**: confirms in-process inference is viable, resolves whether the shape-normalization resize in `segmentation_model.py` actually triggers, and the user can also pick whichever `.pt` version they want (e.g. the newer one trained on additional video) by pointing `model_path` at it.
   - **It fails (e.g. Python 3.8 incompatibility, or `torch`/CUDA build mismatch with WSL2)**: switch to the separate-process fallback — run `segmentation_model.py` in its own venv with a newer Python (only needs `torch`/`lightly_train`/`opencv`, no `rospy`), publishing the class-index mask as a `sensor_msgs/Image` (`mono8` encoding) on its own topic; `vision_nav_node.py` then subscribes to that mask topic instead of running the model itself. Because `segmentation_model.py` was designed with zero `rospy` imports, this is a small, contained change, not a rewrite.
2. **Unit tests, no ROS needed** (`pytest test/`): synthetic mask arrays with known corridor geometry verify `centerline_estimator.py`'s offset math and the controller's sign convention (`offset_norm=-0.5` must produce `angular_z > 0`) before any code touches a robot or sim. Can run in parallel with step 1, including in this sandbox right now.
3. **Bag replay — highest-value pre-Gazebo test**: `rosbag play ros_bags/2026-06-11-11-47-03.bag`, launch the real node against `/usb_cam/image_raw/compressed`, watch the debug image topic in `rqt_image_view` and `cmd_vel` in `rqt_plot`/`rostopic echo` simultaneously. This exercises the entire real pipeline against real, already-recorded, roughly-centered footage with zero Gazebo/Jackal setup dependency, and can happen before the user's sim environment is even ready.
4. **Gazebo integration** (user's own task to set up the Jackal + cornfield world + confirm camera topic): launch teleop, manually offset the simulated robot to one side of the row while still pointed straight, then launch the node and confirm it commands a turn *toward* re-centering before evaluating sustained straight-line performance over distance — this is the empirical sign-convention check in real sim geometry, on top of the unit test's algebraic one.

## What's explicitly out of scope (per the user's own framing)

- Spawning the Jackal in Gazebo and building/finding the cornfield world — the user is handling this themselves.
- Turning/U-turn logic at row ends — explicitly deferred to after a successful straight-line run.
- MPC / EKF — noted as the advisor's long-term recommendation and as a natural upgrade path (the offset/validity signal this design produces is exactly the kind of feature an MPC cost function would consume later), but not built now.
