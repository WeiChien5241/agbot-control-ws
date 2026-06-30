# HANDOFF3.md

Handoff for the P-AgBot vision-nav MPC controller + Gazebo simulation work.
Written so a fresh Claude Code session can continue with zero context loss.

---

## 1. GOAL

Build and validate a vision-based row-centering controller for the Purdue P-AgBot
(Clearpath Jackal UGV) that uses a DINOv3 semantic segmentation model to produce
`cmd_vel` commands keeping the robot centered in a corn row. The immediate milestone
is: **drive straight down a simulated corn row in Gazebo**, which is now set up and
working. The controller is upgraded to MPC (per advisor and grad student request)
and all unit tests pass. The next step is closed-loop testing in simulation on the
user's ROS1 Noetic machine.

---

## 2. CURRENT STATE

### Done and committed to GitHub (`WeiChien5241/agbot-control-ws`, branch `main`)

**`agbot_bringup/` — simulation environment (working):**
- Custom lightweight Gazebo world (`worlds/agbot_corn_rows.world`) with 4 corn rows,
  36 plants each (144 total static models), flat ground, no heightmap terrain.
  Layout: rows at y = −1.125, −0.375, +0.375, +1.125 m; robot corridor at y = 0;
  corn runs from x = 0 to x = 11.7 m at 0.3 m spacing.
- `launch/agbot_gazebo.launch` updated to use the new world via `gazebo_ros/empty_world.launch`
  instead of `virtual_maize_field/simulation.launch` (the old world was too heavy).
  Robot spawns at x = −2.0, y = 0.0 (2 m before the corn field starts).
- Camera topic: `/camera/image_raw` (raw `sensor_msgs/Image`, 640×480, 30 Hz).
- URDF camera injection via `scripts/load_robot_description.sh` — confirmed working
  in a prior session (camera was publishing; see HANDOFF2.md for details).

**`agbot_vision_nav/` — MPC controller (unit tests passing, not yet run in sim):**
- `src/agbot_vision_nav/controller.py` — `MPCRowController` (replaced old `RowCenteringController`).
  SLSQP-based receding-horizon MPC, N=8, same public interface as the old P-controller.
- `test/test_controller.py` — 15 MPC tests. All 26 suite tests pass:
  ```bash
  cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v
  ```
- `config/params.yaml` — MPC params in place (`mpc_horizon`, `mpc_alpha`, `mpc_beta`, etc.).
- `scripts/vision_nav_node.py` — updated to instantiate `MPCRowController`.
- `launch/vision_nav.launch` — primary MPC tuning knobs exposed as launch args.

### Not yet done / not verified on the user's ROS1 machine
- `scipy` not yet installed in the ROS1 Python environment (required for MPC solver).
- `catkin build` not yet run after the MPC changes.
- No closed-loop simulation run has occurred yet. The MPC has never run against a real
  or simulated camera image.
- `alpha` and `beta` kinematic coupling parameters are starting guesses (0.10); need
  empirical tuning once in simulation.

---

## 3. KEY DECISIONS

**Do not re-litigate these without new information:**

### MPC state: image-space, not metric
State vector: `x = [offset_norm, slope_term]` — both derived geometrically from the
segmentation mask by `centerline_estimator.py`. Not converted to physical meters/radians.
The MPC dynamics model (A, B matrices) is parameterized in this normalized image space.
This makes the controller camera-resolution-agnostic and avoids needing camera intrinsics.

### Linear kinematic model with tunable coupling constants
```
x[k+1] = [[1, alpha], [0, 1]] x[k] + [[0], [beta]] u[k]
```
`alpha` (lateral coupling) and `beta` (control effectiveness) cannot be derived
analytically without field measurements — they are exposed as ROS params and must
be tuned empirically. Starting values: alpha = 0.10, beta = 0.10.

### No EKF
CropFollow uses an EKF + IMU. We don't: `r_delta` in the MPC cost provides temporal
smoothing without requiring IMU integration. Simpler, fewer dependencies.

### slope_term is a free heading proxy
`slope_term = far_row_midpoint_offset − near_row_midpoint_offset` is already computed
by `centerline_estimator.py` at zero extra cost. No separate heading regression model
needed (unlike CropFollow's ResNet-18). This is one of the key novel aspects of our
approach vs. the papers.

### scipy SLSQP solver — not a heavy dependency
`scipy.optimize.minimize` with `method='SLSQP'` solves the 8-variable QP in < 1 ms.
scipy is installable via pip in the ROS1 Python 3.8 environment (it's not installed by
default but has no conflicts). Do not switch to a heavier solver (osqp, casadi, etc.)
without a concrete reason.

### Lightweight world: agbot_corn_rows.world, NOT generated.world
`virtual_maize_field/generate_world.py` produces a 615-model world with a heightmap —
too heavy for the user's laptop. The replacement uses 144 static corn models on flat
ground. Do not revert to the generated world. The world file is committed at:
`agbot_bringup/worlds/agbot_corn_rows.world`.

### Camera URDF injection via load_robot_description.sh (not `<env>` tag)
`<env>` tags in ROS1 launch files do NOT propagate to `<param command="...">` subprocesses.
The fix (committed and working) is a shell script that explicitly exports
`JACKAL_URDF_EXTRAS` before calling xacro. Do NOT revert to the `<env>` approach.
Details in HANDOFF2.md GOTCHA #1.

### Sign convention (locked in by tests, do not change)
```
offset_norm < 0  →  centerline LEFT of image center  →  robot RIGHT of row  →  turn LEFT  →  angular_z > 0
slope_term > 0   →  corridor tilts right in image  →  heading LEFT  →  need right correction  →  angular_z < 0
```
Control law satisfies: `angular_z = -(k_p * offset_norm + k_slope * slope_term)` at
steady state. The MPC optimizer produces the same corrective signs because the cost
penalizes non-zero state.

### MPC vs. papers — distinct approach, not copying
See `src/agbot_vision_nav/controller.py` module docstring and the plan file
(`~/.claude/plans/read-the-claude-md-and-logical-lighthouse.md`) for the full
differentiation table. Short version: we use DINOv3 segmentation + geometric
scan-row state estimation + image-space MPC. CropFollow uses ResNet-18 regression +
EKF. P-AgNav uses 3D LiDAR. Agronav uses segmentation + a separate Deep Hough
line detector. Our combination is not present in any single paper.

---

## 4. FILES

### `agbot_bringup/` — simulation bringup
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Main entry point. Starts Gazebo (corn world) → spawns Jackal → overrides robot_description with camera URDF → starts RViz. Args: `gui`, `x` (default −2.0), `y` (default 0.0), `z`, `joystick`. |
| `worlds/agbot_corn_rows.world` | Lightweight SDF: flat ground + 144 static corn plants in 4 rows. Generated by Python script (embedded in the prior session). |
| `urdf/agbot_camera.urdf.xacro` | Camera URDF: mount stand on `mid_mount`, `camera_link`, `camera_optical_frame` (CV convention). Gazebo plugin publishes `/camera/image_raw`. |
| `scripts/load_robot_description.sh` | Key fix: exports `JACKAL_URDF_EXTRAS` then runs xacro. Called via `<param command="...">` AFTER `spawn_jackal.launch` so our param wins. |
| `rviz/agbot.rviz` | RViz config: fixed frame=odom, Camera display on `/camera/image_raw`. |

### `agbot_vision_nav/` — vision controller
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/controller.py` | `MPCRowController`: SLSQP MPC. State [offset_norm, slope_term], linear model, SLSQP solver, rate+magnitude constraints, invalid-frame safety stop. |
| `src/agbot_vision_nav/centerline_estimator.py` | Pure numpy. Scans mask at 3 row fractions, returns `CenterlineResult(offset_norm, slope_term, valid, ...)`. No changes needed. |
| `src/agbot_vision_nav/segmentation_model.py` | Wraps `lightly_train.load_model().predict()`. Returns `(H,W)` uint8 mask resized to input frame size. Requires `lightly_train`+`torch` (not yet confirmed working on ROS1 Python 3.8). |
| `src/agbot_vision_nav/debug_viz.py` | Renders annotated debug image (green traversable overlay, scan rows, midpoints, offset/angular_z text) for `rqt_image_view`. |
| `scripts/vision_nav_node.py` | Only file touching rospy/cv_bridge. Single-slot frame buffer, separate inference thread, 5 Hz watchdog. |
| `config/params.yaml` | All default params. MPC: `mpc_horizon=8, mpc_alpha=0.10, mpc_beta=0.10, mpc_q_offset=10.0, mpc_q_heading=1.0, mpc_r_control=0.1, mpc_r_delta=0.5, delta_angular_z_max=0.2`. |
| `launch/vision_nav.launch` | Loads params.yaml, exposes `model_path` (required), `camera_topic`, `camera_topic_is_compressed`, and primary MPC tuning knobs as launch args. |
| `test/test_controller.py` | 15 MPC tests: sign convention, clamping, rate constraint, smoothness, anticipatory heading, invalid-frame state machine. |
| `test/test_centerline_estimator.py` | 9 tests for the estimator (unchanged from prior sessions). |
| `test/test_debug_viz.py` | 2 smoke tests for debug_viz. |

### Third-party (not in repo, must be cloned separately)
| Path | Notes |
|---|---|
| `jackal/` | `git clone https://github.com/jackal/jackal.git -b noetic-devel` |
| `virtual_maize_field/` | `git clone https://github.com/FieldRobotEvent/virtual_maize_field.git` — needed for the maize_01/maize_02 models even though we don't use its generated world. |
| `DINOv3-Segmentation-Training/out/.../exported_best.pt` | ~89 MB model weights, not in git (in .gitignore). Pass path via `model_path` launch arg. |

### Key system files (binary packages, read-only reference)
| Path | Relevance |
|---|---|
| `/opt/ros/noetic/share/jackal_gazebo/launch/spawn_jackal.launch` | Spawns robot (no Gazebo), sets robot_description without camera. |
| `/home/chien21/agbot_control_ws/src/jackal/jackal_description/urdf/jackal.urdf.xacro` | Line 268: `$(optenv JACKAL_URDF_EXTRAS empty.urdf)` — the hook our shell script targets. |

---

## 5. NEXT STEPS (priority order)

### 1. Install scipy on the ROS1 machine (required before ANY sim run)
```bash
# In whatever Python env runs the ROS1 node (system Python 3.8 or a venv):
pip install scipy
# Verify:
python3 -c "from scipy.optimize import minimize; print('scipy ok')"
```

### 2. Build the workspace
```bash
cd ~/agbot_control_ws && catkin build
source devel/setup.bash
```

### 3. Verify camera is still publishing (regression check from HANDOFF2)
```bash
roslaunch agbot_bringup agbot_gazebo.launch
# In another terminal:
rostopic hz /camera/image_raw    # should show ~30 Hz
rosparam get /robot_description | grep -c camera_link   # should print ≥ 1
```
If `/camera/image_raw` shows 0 Hz, check that `load_robot_description.sh` is
executable (`chmod +x agbot_bringup/scripts/load_robot_description.sh`) and that
`agbot_gazebo.launch` puts the `<param name="robot_description" command="..."/>` 
block AFTER `<include spawn_jackal.launch>`.

### 4. First closed-loop MPC sim run (sign-convention verification)
With the bringup launch running, manually drive the robot off-center with teleop
(still facing +x), then:
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/path/to/exported_best.pt \
  camera_topic:=/camera/image_raw \
  camera_topic_is_compressed:=false

# Watch output:
rostopic echo /cmd_vel
rqt_image_view /vision_nav_node/debug/image
```
**Pass criteria**: `angular_z` sign points toward re-centering when the robot is
visibly off to one side. If the sign is wrong, check HANDOFF.md KEY DECISIONS
sign convention section — the tests lock this in but a model that produces wrong
class mappings (sky/traversable/obstacle) could flip the mask.

### 5. Tune MPC parameters (in order)
Primary knobs exposed as launch args for quick iteration:
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=... camera_topic:=/camera/image_raw camera_topic_is_compressed:=false \
  mpc_q_offset:=15.0 mpc_r_delta:=1.0 mpc_alpha:=0.15 mpc_beta:=0.10
```
Tuning order:
1. `mpc_alpha` / `mpc_beta` — model accuracy. If robot overshoots with small heading
   error → increase `alpha`. If heading corrections feel too weak → increase `beta`.
2. `mpc_q_offset` — how aggressively robot re-centers laterally. Increase if slow.
3. `mpc_r_delta` — smoothness. Increase if steering is jerky or oscillatory.
4. `mpc_q_heading` / `mpc_r_control` — secondary knobs, rarely need touching first.

### 6. Confirm lightly_train / model works in the ROS Python env
If the model fails to load (see GOTCHA #3 below), follow the separate-process fallback
described in HANDOFF.md NEXT STEPS #1: run `segmentation_model.py` in its own venv
and publish the mask as a `sensor_msgs/Image` (`mono8`) on a separate topic. The
`centerline_estimator.py` and `MPCRowController` are unaffected by this change.

### 7. Commit and push after each step
Per CLAUDE.md (mandatory, not optional): commit after each meaningful unit of work.
```bash
git add <specific files>
git commit -m "Present-tense message"
git push
```

---

## 6. GOTCHAS

### GOTCHA 1: scipy not installed by default in ROS1 Noetic Python
`scipy` is NOT part of the standard ROS1 Noetic Python packages. The MPC solver
(`MPCRowController._solve`) will fail with `ModuleNotFoundError` at runtime without it.
Install before the first sim run: `pip install scipy`.
scipy 1.7.x is the last version supporting Python 3.8 (which Noetic uses).

### GOTCHA 2: `<env>` tag does not work for `<param command="...">` in ROS1 launches
Already fixed via `load_robot_description.sh`. Do not revert to `<env name="JACKAL_URDF_EXTRAS" .../>`.
The `<env>` tag only reaches `<node>` processes, not subprocesses created by `<param command>`.
Full explanation in HANDOFF2.md GOTCHA #1.

### GOTCHA 3: lightly_train / torch may not work under ROS1 Python 3.8
The model has only ever been run on Google Colab. Whether `pip install lightly_train`
works on the ROS1 Noetic system Python (3.8, Ubuntu 20.04) is UNCONFIRMED. If it fails,
the fallback is to run `segmentation_model.py` in a separate Python process (newer venv),
publishing the mask as `sensor_msgs/Image mono8`. The module boundary is already clean —
`segmentation_model.py` has zero rospy imports. Only `vision_nav_node.py` needs a small
change (subscribe to mask topic instead of calling `SegmentationModel` directly).

### GOTCHA 4: GAZEBO_MODEL_PATH must include virtual_maize_field/models
Even though we don't use the virtual_maize_field generated world, the corn plant models
(`model://maize_01`, `model://maize_02`) come from `virtual_maize_field/models/`. The
`agbot_gazebo.launch` sets this via `<env name="GAZEBO_MODEL_PATH">` — this works for
Gazebo (a `<node>`) unlike for `<param command>`. If the corn models don't appear in
Gazebo, source the workspace first: `source ~/agbot_control_ws/devel/setup.bash`.

### GOTCHA 5: MPC alpha and beta are NOT physically calibrated
`mpc_alpha=0.10` and `mpc_beta=0.10` are starting guesses, not measured values.
The model is formulated in normalized image space, so these have no direct physical
analog — they must be tuned empirically. If the robot oscillates: try reducing
`mpc_alpha`. If it converges too slowly: increase `mpc_q_offset` before touching `alpha`.

### GOTCHA 6: Robot spawns at x=-2.0 BEFORE the corn field starts at x=0
The default spawn is intentional — the robot needs some clear space before entering the
row so it can accelerate to cruise speed before the controller takes over. If you
override x to something inside the corn (e.g. x=3.0), the robot may collide at spawn.

### GOTCHA 7: jackal/ and virtual_maize_field/ are gitignored
These are upstream repos excluded from this repo. On a fresh clone:
```bash
cd ~/agbot_control_ws/src
git clone https://github.com/jackal/jackal.git -b noetic-devel
git clone https://github.com/FieldRobotEvent/virtual_maize_field.git
```

### GOTCHA 8: `gh` CLI is at ~/.local/bin/gh (not on system PATH)
Every session needs: `export PATH="$HOME/.local/bin:$PATH"` before `git push`.
Or add to `~/.bashrc` to make it permanent.

### GOTCHA 9: This dev sandbox has ROS2 Humble, NOT ROS1 Noetic
All `catkin`, `roslaunch`, `rostopic`, `rosbag` commands must be run by the user on
their WSL Ubuntu 20.04 / ROS1 Noetic machine. Do not attempt them in the Claude Code
terminal. The unit tests (pytest, no rospy imports) CAN be run in this sandbox since
pytest and scipy were installed via `python3 -m pip install pytest scipy --user`.

### GOTCHA 10: Test suite requires scipy to be installed
The unit tests import `MPCRowController` which imports scipy at module load time.
```bash
# In this sandbox (already installed):
cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v
# Expected: 26 passed
```
On the user's ROS1 machine, run the same command after `pip install scipy pytest`.

### GOTCHA 11: Do NOT regenerate virtual_maize_field generated.world
`rosrun virtual_maize_field generate_world.py` is no longer needed and changes the
random corn layout (invalidating any hardcoded spawn coordinates). We use our own
`agbot_corn_rows.world` instead. The old `generated.world` at
`~/.ros/virtual_maize_field/generated.world` is harmless to leave in place.

---

## Quick-start commands (ROS1 Noetic machine)

```bash
# 0. One-time setup
pip install scipy pytest

# 1. Build
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# 2. Launch simulation
roslaunch agbot_bringup agbot_gazebo.launch

# 3. In another terminal: launch controller
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/home/chien21/agbot_control_ws/src/DINOv3-Segmentation-Training/out/corn_field_navigation/exported_models/exported_best.pt \
  camera_topic:=/camera/image_raw \
  camera_topic_is_compressed:=false

# 4. Monitor
rostopic echo /cmd_vel
rqt_image_view /vision_nav_node/debug/image

# 5. Run unit tests (no ROS needed)
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v
# Expected: 26 passed
```
