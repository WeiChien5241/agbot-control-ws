# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Git and GitHub workflow — MANDATORY, follow every session

This repo is tracked in git and mirrored on GitHub. **Commit and push after every meaningful unit of work — not just at session end.** If a session ends with uncommitted changes, that work is at risk. Treat each logical step (new file written, bug fixed, launch tested) as a commit boundary.

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

**World**: `agbot_bringup/worlds/agbot_corn_rows.world` — lightweight custom world,
4 rows of corn (36 plants each, 0.75 m row spacing) on flat ground. No heightmap terrain.
No pre-generation step needed (unlike the old virtual_maize_field generated.world).

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
- `src/agbot_vision_nav/controller.py` — P-controller: `angular_z = -(k_p * offset_norm + k_slope * slope_term)`. Sign convention: centerline left of image-centre → positive `angular.z` (left turn, REP-103).
- `src/agbot_vision_nav/debug_viz.py` — debug overlay image for `rqt_image_view`.
- `scripts/vision_nav_node.py` — only file touching `rospy`/`cv_bridge`. Single-slot frame buffer, separate inference thread, 5 Hz watchdog.

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
