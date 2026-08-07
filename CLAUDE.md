# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Git and GitHub workflow — MANDATORY, follow every session

This repo is tracked in git and mirrored on GitHub. **Commit and push after every meaningful unit of work — not just at session end.** If a session ends with uncommitted changes, that work is at risk. Treat each logical step (new file written, bug fixed, launch tested) as a commit boundary.

**This instruction is standing authorization to commit and push without asking first**, as long as each commit follows the rules below (specific files staged by name, clean present-tense message, nothing in the "what NOT to commit" list). Don't wait for the user to say "commit this" — do it proactively as you finish each unit of work, and push right after so nothing sits unpushed between sessions.

**Commit rules:**
- **After each file created or meaningfully changed** — don't batch a whole session into one commit
- **Before switching tasks** — if you were working on the world file and are now changing a launch file, commit the world file first
- **After a successful test** — commit the state that works so you can roll back if needed
- **Never use `git add .`** — always stage specific files by name to avoid accidentally committing build artifacts, model weights, or generated files

```bash
# Stage only the files you changed
git add agbot_bringup/launch/agbot_gazebo.launch agbot_bringup/urdf/agbot_camera.urdf.xacro

# Commit with a clear present-tense imperative message
git commit -m "Fix camera URDF injection via load_robot_description.sh"

# Push immediately — don't let commits pile up
git push
```

**Commit message style**: present-tense imperative, concise, describes the change not the task.
- Good: `"Add lightweight corn-row world with 4 rows"`, `"Fix spawn position default to x=-2.0"`
- Bad: `"fixed the world thing"`, `"changes"`, `"wip"`

**Branch strategy**: work on `main` for now (small team, single user). If an experiment might break things, create a feature branch:
```bash
git checkout -b feature/vision-nav-tuning
# ... work ...
git push -u origin feature/vision-nav-tuning
```

**What NOT to commit**: `*.pt` model weights (tracked in .gitignore — distribute via Google Drive/shared storage), Gazebo generated world files (`~/.ros/virtual_maize_field/`), build artifacts (`build/`, `devel/`).

**Third-party packages** (`jackal/`, `virtual_maize_field/`) are excluded from this repo via `.gitignore` — they are separate upstream repos cloned alongside the custom packages. To recreate the workspace from scratch:
```bash
cd ~/agbot_control_ws/src
git clone https://github.com/jackal/jackal.git -b noetic-devel
git clone https://github.com/FieldRobotEvent/virtual_maize_field.git
```

---

## Project overview

This is Purdue's P-AgBot project: an agricultural robot (built on a Clearpath Jackal UGV) for in-row and under-canopy crop monitoring. The workspace (`agbot_control_ws/src`) contains:

1. **`agbot_bringup/`** — simulation bringup package: launches the Jackal in the virtual maize field (Gazebo) with a simulated forward camera and opens RViz. Main entry point: `roslaunch agbot_bringup agbot_gazebo.launch`.
2. **`agbot_vision_nav/`** — vision-based row-centering controller: subscribes to a camera topic, runs a DINOv3 segmentation model, publishes `cmd_vel` to keep the robot centered in a crop row.
3. **`DINOv3-Segmentation-Training/`** *(if present)* — model training pipeline (trains on annotated rosbag footage, produces `exported_best.pt`).
4. **`Papers/`** — lab papers (P-AgBot, P-AgSLAM, P-AgNav) and related external papers (Agronav, ROW-SLAM, CropFollow). Read before designing navigation/control logic.

**Third-party packages in this workspace** (not tracked in this repo):
- `jackal/` — Clearpath Jackal ROS1 driver (noetic-devel branch)
- `virtual_maize_field/` — procedural corn field world generator for Gazebo

## Critical environment split — read this before running anything

- **This dev sandbox has ROS2 (Humble) installed, not ROS1.** The actual robot and the user's development laptop (WSL2, Ubuntu 20.04) run **ROS1 Noetic**. Any `catkin_make`/`catkin build`/`roslaunch`/`rospy`/`rosbag play` command must be run by the user on their ROS1 machine.
- `agbot_control_ws` is used as a catkin workspace root; `src/` holds the packages directly. Build from the workspace root, not from inside `src/`.
- The segmentation model (`lightly_train` + `torch`) has so far only been trained/run on **Google Colab** — never on the ROS1 Noetic box. ROS1 Noetic on Ubuntu 20.04 ships system Python 3.8; whether `lightly_train`'s dependencies even install there is unconfirmed.

## agbot_bringup — simulation launch package

Single command to start everything in simulation:
```bash
roslaunch agbot_bringup agbot_gazebo.launch
# Optional overrides:
# x:=-2.0 y:=0.0        — robot spawn position (default: 2 m before row start)
# gui:=false             — headless Gazebo
# joystick:=false        — disable teleop (autonomous-only run)
```

**World**: the launch includes `virtual_maize_field/launch/simulation.launch`,
which loads whatever generated world is active in `~/.ros/virtual_maize_field/`.
Two snapshots are kept under `~/.ros/virtual_maize_field_snapshots/` and swapped
with `scripts/switch_maize_world.sh full|small`:
- **small** (default; matches the launch spawn-pose defaults): generated from
  `config/agbot_maize_small.yaml` — 4 straight rows × 6 m, coarse flat
  heightmap, seed 42. RTF ≈ 1.0 on the dev laptop. Regenerate with
  `scripts/generate_small_maize_world.sh` (snapshots the current world first).
- **full**: the original FRE-style world (curved rows, dense heightmap).
  RTF < 0.1 on the laptop; spawn pose x:=3.16 y:=-9.31 z:=0.36 yaw:=1.791.

A legacy lightweight `agbot_corn_rows.world` (plus its `dirt_ground` model) used
to live in `agbot_bringup/`; it was removed on 2026-08-06 — the maize worlds are
the only simulation option now, because segmentation quality is much better on
their visuals. Recover it from git history if it is ever needed again.

**Camera topics** (simulation): front `/camera/image_raw`, rear `/camera_rear/image_raw` (both `sensor_msgs/Image`, raw, 640×480, 30 Hz). The rear camera is only consumed in mission mode with `rear_camera_enabled:=true` (blocked-row back-out). `roslaunch agbot_bringup display.launch.xml` shows the URDF in RViz without Gazebo (camera-placement iteration).
When launching the vision-nav controller in simulation:
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/path/to/exported_best.pt \
  camera_topic:=/camera/image_raw \
  camera_topic_is_compressed:=false
```

**Camera URDF injection**: The camera is added to the Jackal via `JACKAL_URDF_EXTRAS`. The launch file calls `scripts/load_robot_description.sh` which exports that env var and runs xacro — this overrides the `robot_description` param set by `spawn_jackal.launch` so both `robot_state_publisher` and `spawn_model` see the camera-inclusive URDF.

## agbot_vision_nav — row-centering controller

Architecture (rospy-free algorithmic core, unit-testable without ROS):
- `src/agbot_vision_nav/segmentation_model.py` — wraps `lightly_train.load_model()`/`.predict()`, force-resizes output mask to input resolution with nearest-neighbor interpolation.
- `src/agbot_vision_nav/centerline_estimator.py` — pure numpy: scans mask rows outward from image centre until hitting a non-traversable pixel, returns normalized lateral `offset_norm`.
- `src/agbot_vision_nav/controller.py` — `MPCRowController`: SLSQP receding-horizon MPC (N=8) over state `[offset_norm, slope_term]` in normalized image space. Requires `scipy`. Sign convention: centerline left of image-centre → positive `angular.z` (left turn, REP-103).
- `src/agbot_vision_nav/row_exit_detector.py` — detects end-of-row from the mask (corridor widening to open field, or blocked-ahead wall). Open fires when at least `exit_open_rows_required` (1) scan rows — ANY of them — are wide. Do NOT require specific (e.g. farthest) rows: beyond the field the segmentation of distant ground is garbage, so the far rows can stay invalid forever (field-tested: a farthest-rows requirement never fired and the robot drove off the world edge). Per-signature debounce (`exit_detect_frames` 5 open, `blocked_detect_frames` 8 blocked — leaves brushing the lens must not trigger a back-out) and per-signature arming: open after `min_in_row_distance` m, blocked already after `blocked_arming_distance` m (0.3) so mid-row obstacles near the entrance are still caught.
- `src/agbot_vision_nav/mission_fsm.py` — multi-row mission state machine (FOLLOW_ROW → EXIT_CLEAR → TURN_1 → TRAVERSE → TURN_2 → REACQUIRE): odometry-closed-loop 90° headland turns, boustrophedon direction alternation, `num_rows` termination (0 = until no rows left), EXIT_CLEAR runs at `exit_clear_speed` (0.10, slower than cruise — the post-exit leg is where overshoot hurts). **EXIT_CLEAR is REAR-STEERED when a rear camera is present** (`exit_clear_rear_steering`, default true): the rear view looks back down the row being left, so the headland leg steers from it and turns only when the rear ALSO reads open field — the tail has cleared the last plants. ⚠ **The MPC state is NEGATED there**, unlike BACKOUT: the rule is *negate iff exactly ONE of {mirrored view, reversed motion}*, and BACKOUT is the both-apply case where they cancel. ⚠ `headland_clearance` does NOT end that leg — it stays the terminator of the open-loop leg, which runs with no rear camera and is also the automatic fallback if no rear frame arrives at all. Revocation is likewise open-loop-only (it needs the front camera); rear-steered mode uses `exit_clear_max_distance` instead — driving that far without the rear opening means there was no row end, so the row is un-counted and FOLLOW_ROW resumes. Sim-validated: first Gazebo mission run (small maize world, 3 rows) succeeded with the default thresholds. Blocked-row branch (BACKOUT → BACKOUT_CLEAR → BACKOUT_TURN_1 → BACKOUT_TRAVERSE → BACKOUT_TURN_2 → REACQUIRE): on a blocked-ahead signal the robot reverses out the end it entered (rear-camera-steered, odometry-bounded), S-turns into the next row (traveled in the SAME direction; the boustrophedon flip is suppressed once), records `blocked_events` reported at mission DONE. Rear steering reuses the MPC controller with UNCHANGED signs (image mirror + reversed motion cancel). The back-out is gated on `rear_camera_enabled`: without the rear camera the BACKOUT states are unreachable and a blocked signal stops the robot and ends the mission (reported).
- `src/agbot_vision_nav/metrics_logger.py` — per-run CSV performance metrics: one row per processed frame (tracking error, control, detector status, odometry, timing) plus `summarize()`. ON by default (`metrics_csv_dir: ~/agbot_logs`); `metrics_csv_dir:=none` skips a run. Report with `scripts/analyze_run.py <csv>...`. ⚠ `offset_norm` is normalized IMAGE space, **not meters**, and shifts with camera mount height (~0.5 tall vs ~0.7 low) — never compare across rigs; quote the FOLLOW_ROW row (TURN/TRAVERSE are odometry open loop).
- `src/agbot_vision_nav/intervention_detector.py` — the autonomy metric's definition of a human intervention: a **joystick takeover** (deadman held on `/bluetooth_teleop/joy`, buttons 4/5). Activity within `intervention_gap_seconds` (3.0) of the previous activity is the SAME intervention, so one messy rescue scores 1 and not 5. Nothing has to be pressed or remembered during a run. `intervention_joy_topic:=none` disables. The node writes a `teleop` column on every CSV row and an `INTERVENTION` event; `summarize()` turns those plus odometry into **meters per intervention (MDBI)**, the number field papers report — teleop-driven meters are subtracted, never credited to the controller. `/odometry/filtered` is now subscribed on **every** run (not just missions), because distance is the denominator.
- `src/agbot_vision_nav/debug_viz.py` — debug overlay image for `rqt_image_view` (mask wash, scan rows, midpoints, per-row corridor width `w=`, mission state HUD).
- `src/agbot_vision_nav/launch_args.py` — rospy-free roslaunch argv builder for the operator panel. Its one rule: **a blank field is not passed**, so `params.yaml` stays the source of truth and the panel cannot silently re-pin every knob at its own defaults.
- `scripts/vision_nav_node.py` — only file touching `rospy`/`cv_bridge`. Single-slot frame buffer, separate inference thread, 5 Hz watchdog, optional odometry subscriber. Mission mode is gated behind `~mission_enabled` (default **false** → plain row-following, identical to pre-mission behavior). Offers `~pause` (`std_srvs/SetBool`) and a latched `~status` string.
- `scripts/operator_panel.py` — PyQt panel: start frame source (cameras, or Gazebo when `simulation` is ticked) / start mission with form fields / **pause + resume** / stop. `rosrun agbot_vision_nav operator_panel.py`. Starting Gazebo from it is optional — running it in its own terminal and using the panel only for the mission works identically.

⚠ **`--` is illegal inside an XML comment** (reserved for `-->`), and this repo writes `--` as an em-dash in prose everywhere. It broke `vision_nav.launch` outright on 2026-08-06. `test/test_launch_files.py` parses every launch/xacro and names the offending line — the sandbox has no ROS1, so pytest is the only place a launch-file error is caught before the robot sees it.

**Pause vs Ctrl-C** — they are not the same thing. Ctrl-C destroys the mission: `rows_driven`, the boustrophedon turn direction, the detector's arming distance and the row-entry pose all live in the node, so a restart begins at row 1. `~pause` publishes zero and **skips the FSM update**, so all of that survives. Resuming resets the detectors and the MPC, because the BLOCKED timer counts in ROS *seconds* — without that, a 30 s pause would deposit the whole `blocked_confirm_seconds` on the first frame back and fire a back-out nothing justified.

Run unit tests (no ROS or `lightly_train` needed):
```bash
cd agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v      # expected: 187 passed
```

Performance report from a run (no ROS; CSVs are written automatically):
```bash
python3 agbot_vision_nav/scripts/analyze_run.py ~/agbot_logs/vision_nav_*.csv
```
The report's **Autonomy** section is the paper-comparable one: distance
travelled, autonomous distance, interventions, and meters per intervention.
Unlike `offset_norm` it is in meters and mount-independent. A run with zero
interventions has no mean — only a `>=` lower bound; pool runs (sum distances,
sum interventions) before quoting a figure.

Distance travelled (and interventions) straight from a rosbag — ROS1 only,
works on bags recorded before the metrics logger existed and on hand-driven
runs:
```bash
rosbag record /odometry/filtered /bluetooth_teleop/joy   # + whatever else
python3 agbot_vision_nav/scripts/bag_distance.py ~/bags/field_*.bag
```
It reports path length from the pose (EKF jitter below `--min-step` dropped)
and cross-checks it against integrated wheel speed `∫|twist.linear.x| dt`;
the two disagreeing by >10% means wheel slip or a jumpy EKF, and the script
says so. Neither is ground truth — say which one you quote.

## Commands

Build the catkin workspace (ROS1 Noetic only):
```bash
cd ~/agbot_control_ws && catkin build
source devel/setup.bash
```

Launch simulation (Gazebo + Jackal + RViz):
```bash
roslaunch agbot_bringup agbot_gazebo.launch
```

Launch vision-nav controller (real robot):
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/absolute/path/to/exported_best.pt \
  camera_topic:=/usb_cam/image_raw/compressed \
  camera_topic_is_compressed:=true
```

Launch vision-nav controller (simulation):
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/absolute/path/to/exported_best.pt \
  camera_topic:=/camera/image_raw \
  camera_topic_is_compressed:=false
```

Speed tuning happens on the real robot (RTX 4080), not in the laptop sim
(RTF-limited): raise `linear_x_cruise` via launch args, scaling
`angular_z_max`, `delta_angular_z_max`, and `mpc_alpha` proportionally with
speed. Defaults stay at the sim-validated 0.15 m/s envelope.

**Where to change a parameter**: `agbot_vision_nav/config/params.yaml`. Edit a
value there and it takes effect. Launch `<arg>`s default to EMPTY and their
`<param>` tags are conditional, so a launch arg overrides the file only when you
actually pass one — use that for per-run field iteration, then write the settled
value back into `params.yaml`. Four keys are exceptions, computed by the launch
file from `sim:=true|false` and therefore not editable in the yaml:
`camera_topic`, `camera_topic_is_compressed`, `rear_camera_topic`,
`rear_camera_topic_is_compressed` (plus `model_path`, which has no yaml entry).

⚠ Before 2026-07-30 this was the opposite: every knob was declared in BOTH
files and the launch `<param>` silently won for all 44 duplicated keys, so
editing `params.yaml` did nothing. If you see a value that "isn't taking",
check what actually resolved:

```bash
roslaunch --dump-params agbot_vision_nav vision_nav.launch [args...]
```

The node also logs its full resolved config once at startup, right before
`vision_nav_node ready` — that line is the fastest way to confirm a knob.

Multi-row mission mode (headland turns between rows; requires `/odometry/filtered`):
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=... camera_topic:=/camera/image_raw camera_topic_is_compressed:=false \
  mission_enabled:=true num_rows:=3
```

Smoke-test the segmentation model on one saved image (no ROS; uses the
`~/agbot_venv` virtualenv where `lightly_train`/`torch` are installed):
```bash
source ~/agbot_venv/bin/activate
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 smoke_test_segmentation.py   # edit paths at top of file
```
