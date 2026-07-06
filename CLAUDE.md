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

Maize worlds are preferred over the legacy `agbot_bringup/worlds/agbot_corn_rows.world`
because segmentation quality is much better on their visuals.

**Camera topic** (simulation): `/camera/image_raw` (`sensor_msgs/Image`, raw, 640×480, 30 Hz).
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
- `src/agbot_vision_nav/row_exit_detector.py` — detects end-of-row from the mask (corridor widening to open field, or blocked-ahead wall); debounced, armed only after `min_in_row_distance` m of odometry travel.
- `src/agbot_vision_nav/mission_fsm.py` — multi-row mission state machine (FOLLOW_ROW → EXIT_CLEAR → TURN_1 → TRAVERSE → TURN_2 → REACQUIRE): odometry-closed-loop 90° headland turns, boustrophedon direction alternation, `num_rows` termination (0 = until no rows left). Sim-validated: first Gazebo mission run (small maize world, 3 rows) succeeded with the default thresholds.
- `src/agbot_vision_nav/debug_viz.py` — debug overlay image for `rqt_image_view` (mask wash, scan rows, midpoints, per-row corridor width `w=`, mission state HUD).
- `scripts/vision_nav_node.py` — only file touching `rospy`/`cv_bridge`. Single-slot frame buffer, separate inference thread, 5 Hz watchdog, optional odometry subscriber. Mission mode is gated behind `~mission_enabled` (default **false** → plain row-following, identical to pre-mission behavior).

Run unit tests (no ROS or `lightly_train` needed):
```bash
cd agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v
```

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
