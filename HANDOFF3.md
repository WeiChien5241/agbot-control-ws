# HANDOFF3.md

Handoff for the P-AgBot vision-nav work: MPC row-following (demoed in Gazebo)
plus the multi-row headland-turn mission (now SIM-VALIDATED: first Gazebo run
succeeded with default thresholds). Written so a fresh Claude Code session can
continue with zero context loss. Supersedes the previous HANDOFF3 content.

---

## 1. GOAL

Build a vision-based navigation stack for the Purdue P-AgBot (Clearpath Jackal
UGV): a DINOv3 segmentation model turns camera frames into traversability
masks; pure-geometry state estimation + image-space MPC keep the robot
centered in a corn row. **Row-following works — there is a working demo in
Gazebo (recorded, on YouTube).** **Multi-row coverage also works**: with
`mission_enabled:=true num_rows:=3` the robot followed a corridor, executed
the odometry-closed-loop headland turns, drove 3 rows boustrophedon, and
stopped cleanly at the end of row 3 — on the FIRST sim run, with the default
(guessed) thresholds, in the small maize world. Next milestones: mission
robustness runs, field deployment (speed tuning happens there), go-home.

---

## 2. CURRENT STATE

### Done, committed, and pushed (`WeiChien5241/agbot-control-ws`, branch `main`)

**Row-following (DEMOED WORKING in Gazebo):**
- Full pipeline runs closed-loop in sim: `/camera/image_raw` → `SegmentationModel`
  → `estimate_centerline` → `MPCRowController` → `/cmd_vel`.
- `smoke_test_segmentation.py` (repo root of `agbot_vision_nav/`) validated the
  model standalone and saves a visual overlay
  (`~/agbot_control_ws/src/tmp/segmentation_overlay.jpg` — good frame: green
  wash on dirt path, red midpoint dots centered, `valid=True`).
- **MPC speed tuning is DEFERRED TO THE FIELD.** A 0.3 m/s attempt oscillated
  badly in the laptop sim — but that sim ran at RTF < 0.1 with ~3.5 Hz jittery
  `/cmd_vel`, so gain tuning there does not transfer to the real robot
  (RTX 4080). Defaults are reverted to the demoed-working 0.15 m/s envelope
  (`linear_x_cruise=0.15, angular_z_max=0.3, delta_angular_z_max=0.2,
  mpc_alpha=0.10`). Field-tuning interface: all of these plus the MPC weights
  are `vision_nav.launch` args. **Scaling rule**: when raising
  `linear_x_cruise`, scale `angular_z_max`, `delta_angular_z_max`, and
  `mpc_alpha` proportionally (preserves path curvature; alpha is per-step
  lateral drift, which grows with speed). Field protocol: measure
  `rostopic hz /cmd_vel` first, then step 0.2 → 0.25 → 0.3 watching
  angular_z for oscillation. Note: `mpc_dt` is accepted but UNUSED by
  `MPCRowController` — alpha/beta ARE the per-step dynamics.

**Multi-row mission (commits `45a4c78`, `9b77ff9`, `c39c390`, `0fe6dc3` —
SIM-VALIDATED: first Gazebo run succeeded with default thresholds, 3 rows,
clean DONE):**
- `row_exit_detector.py` — end-of-row detection from the mask.
- `mission_fsm.py` — headland state machine with odometry-closed-loop turns.
- `vision_nav_node.py` — odom subscriber + FSM wiring behind `~mission_enabled`
  (default false → behavior identical to before; safe for MPC tuning).
- `debug_viz.py` — HUD now shows `state=` and per-scan-row corridor width `w=`.
- `params.yaml` / `vision_nav.launch` — all mission params + `mission_enabled`,
  `num_rows` launch args. `package.xml` gained `nav_msgs`.
- **38/38 unit tests pass** (`cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v`).

**Small maize world workflow (commits `e427cc6`, `738c9cd`):** the user's
laptop ran the full virtual_maize_field world at RTF < 0.1 / ~2 fps; the small
world runs at **RTF 1.0 / ~44 fps** with identical visuals (same maize models
+ ground texture, so segmentation quality is preserved — the user confirmed
segmentation is much better in maize worlds than in `agbot_corn_rows.world`).
- `agbot_bringup/config/agbot_maize_small.yaml` — 4 straight rows × 6 m
  (3 corridors), `ground_resolution 0.15` (vs 0.02 full — the main RTF win),
  nearly flat, no row holes, `seed: 42` (reproducible layout + spawn pose).
- `agbot_bringup/scripts/generate_small_maize_world.sh` — snapshots the current
  world to `~/.ros/virtual_maize_field_snapshots/full/` first, generates the
  small world, snapshots it as `small/`, prints the spawn pose.
- `agbot_bringup/scripts/switch_maize_world.sh full|small` — swaps a snapshot
  into the canonical cache folder (the world SDF hard-references its heightmap
  there) and clears Gazebo's terrain paging cache.
- Spawn poses: small world = launch defaults (x=0.777, y=-4.331, z=0.35,
  yaw=1.536); full world = x=3.16, y=-9.31, z=0.36, yaw=1.791 (pass as args
  after switching). Each generated world's pose lives in
  `~/.ros/virtual_maize_field/robot_spawner.launch`.

**Presentation:** `AgBot_MPC.pptx` at repo root (12 slides, plain black/white,
explains the whole pipeline for the professor). Not committed (binary
deliverable). Slide 4 is a placeholder for the YouTube demo video.

### Not yet done
- Mission robustness beyond the one successful run: different start corridors,
  `first_turn_direction:=right`, `num_rows:=0` (no-rows-left termination),
  full-world run.
- Field deployment + speed tuning on the real robot (see scaling rule above).
- Go-home functionality — deliberately deferred (see NEXT STEPS / plan file
  `~/.claude/plans/as-for-now-we-wild-moth.md`).

---

## 3. KEY DECISIONS

**Do not re-litigate these without new information:**

### MPC in image space, no EKF, slope_term as free heading proxy
State `x = [offset_norm, slope_term]`, both pure geometry from the mask
(`centerline_estimator.py`). Linear model `x[k+1] = [[1,alpha],[0,1]]x + [[0],[beta]]u`,
SLSQP solver (scipy), smoothing via `r_delta` cost instead of an EKF/IMU.
Camera-resolution-agnostic, no intrinsics needed. Distinct from CropFollow
(ResNet regression + EKF), P-AgNav (LiDAR), Agronav (Deep Hough). Full
differentiation table in `controller.py`'s module docstring.

### Sign convention (locked in by tests)
`offset_norm < 0` → centerline LEFT of image center → robot too far RIGHT →
turn LEFT → `angular_z > 0` (REP-103). `slope_term > 0` → heading LEFT →
corrective `angular_z < 0`.

### Headland maneuvers are odometry-closed-loop (decided with user this session)
Turns integrate measured yaw from `/odometry/filtered` (Jackal EKF, wheel+IMU)
until 90° swept; straights integrate measured displacement. The between-rows
traverse is exactly `row_spacing` (0.75 m) — **measured, not guessed** — which
is also what guarantees the robot enters a NEW row rather than the one it
exited. Rejected alternatives: timed open-loop (slip-fragile), vision-only
(can't distinguish exited row from next row mid-turn).

### Exit detection: two mask signatures, debounced, distance-armed (user's insight)
1. **Open-field**: leaving the row, corridor width blows up toward full image
   width — require ALL valid scan rows ≥ `exit_width_threshold` (0.8).
2. **Blocked-ahead**: no corridor at any scan row but lower half still largely
   traversable → wall of crop dead ahead.
Both need `exit_detect_frames` (5) consecutive frames, and detection only arms
after `min_in_row_distance` (2.0 m) of odom travel in-row — otherwise the
open-field view at row entry false-triggers.

### Boustrophedon turn alternation (user's insight)
Two lefts into this row ⇒ two rights into the next. FSM flips `_turn_sign`
after each completed REACQUIRE. First direction is the `first_turn_direction`
param.

### Termination and failure semantics
`num_rows: N` → after the Nth row's exit fires, go straight to DONE (no final
turn). `num_rows: 0` → unlimited; ends when REACQUIRE creeps
`reacquire_max_distance` (1.5 m) without finding a corridor = no rows left.
REACQUIRE failure is terminal DONE in either mode (stop, don't wander).

### mission_enabled defaults to FALSE
Plain row-following is untouched with the flag off. MPC tuning continues
undisturbed. Do not change the default until mission mode is sim-validated.

### Go-home: deferred, approach already chosen
When built: U-turn and re-run the mission in reverse with mirrored turn
directions (reuses `MissionFSM` as-is). Vision re-centers every row so odom
drift doesn't accumulate. Do NOT do pure odom path-replay.

### Sim world: SMALL generated maize world (supersedes the old corn-rows decision)
The user found segmentation quality much better in virtual_maize_field worlds
than in the lightweight `agbot_corn_rows.world`, but the full maize world ran
at RTF < 0.1 on the laptop. Resolution: generate a small maize world
(`agbot_maize_small.yaml` — same maize models/ground texture, 4×6 m straight
rows, coarse heightmap → RTF 1.0) and keep full/small snapshots switchable via
`switch_maize_world.sh`. Do NOT go back to `agbot_corn_rows.world` for
vision work.

### Speed tuning belongs in the field, not the laptop sim
The 0.3 m/s attempt oscillated in a sim running at RTF < 0.1 with ~3.5 Hz
jittery /cmd_vel — that timing has nothing in common with the RTX 4080 robot.
Sim validates LOGIC (FSM, sign conventions, thresholds); gains and speed are
tuned on hardware. Defaults stay at the demoed 0.15 m/s envelope; launch args
override in the field, scaling clamps and mpc_alpha with speed.

### Environment/infra decisions (from earlier sessions, still binding)
- Camera URDF injection via `scripts/load_robot_description.sh` exporting
  `JACKAL_URDF_EXTRAS` — `<env>` tags do NOT reach `<param command>` in ROS1.
- scipy SLSQP is fine (<1 ms for the 8-var QP) — don't switch to osqp/casadi
  without a concrete reason.

---

## 4. FILES

### `agbot_vision_nav/` — vision controller + mission
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/segmentation_model.py` | Wraps `lightly_train.load_model().predict()`; nearest-neighbor-resizes mask to frame size. Classes: 0=sky, 1=traversable, 2=obstacle. |
| `src/agbot_vision_nav/centerline_estimator.py` | Pure numpy. Scans 3 rows outward from image-center column; returns `CenterlineResult(offset_norm, slope_term, valid, traversable_fraction, scan_rows)`. `scan_rows` carry `x_left/x_right` per row (the exit detector reuses these). |
| `src/agbot_vision_nav/controller.py` | `MPCRowController`: SLSQP MPC, N=8, magnitude+rate constraints, invalid-frame hold-then-stop. `compute(offset_norm, slope_term, valid) -> (linear_x, angular_z)`. |
| `src/agbot_vision_nav/row_exit_detector.py` | `RowExitDetector.update(result, image_width, distance_in_row)` → `EXIT_NONE / EXIT_ROW_END_OPEN / EXIT_ROW_END_BLOCKED`. Also exports `normalized_corridor_widths()`. |
| `src/agbot_vision_nav/mission_fsm.py` | `MissionFSM.update(result, odom_pose, image_width) -> (linear_x, angular_z, state, done)`. States: FOLLOW_ROW, EXIT_CLEAR, TURN_1, TRAVERSE, TURN_2, REACQUIRE, DONE. Owns detector + controller; calls `controller.reset()` + `detector.reset()` on each new row. |
| `src/agbot_vision_nav/debug_viz.py` | Debug overlay: green mask wash, scan rows, midpoints, `w=` width labels, `state=` HUD line. |
| `scripts/vision_nav_node.py` | Only rospy file. Frame buffer + inference thread + 5 Hz watchdog. `_odom_cb` stores `(x, y, yaw)` (manual quaternion→yaw, no tf dep). `_process_frame` routes through FSM iff `mission_enabled`. |
| `config/params.yaml` | All defaults incl. the mission block (thresholds documented inline). |
| `launch/vision_nav.launch` | Args: `model_path` (required), camera args, MPC knobs, `mission_enabled`, `num_rows`. |
| `smoke_test_segmentation.py` | Standalone model check + visual overlay writer. Edit `MODEL_PATH`/`IMAGE_PATH` at top; run in `~/agbot_venv`. |
| `test/` | 38 tests: controller (15), centerline (9), debug_viz (2), row_exit_detector (7), mission_fsm (5). |

### `agbot_bringup/` — simulation
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Gazebo (virtual_maize_field generated world) + Jackal spawn (defaults = small-world pose) + camera URDF override + RViz. |
| `config/agbot_maize_small.yaml` | Small maize world config (4×6 m straight rows, coarse flat heightmap, seed 42). |
| `scripts/generate_small_maize_world.sh` | Snapshots current world, generates the small one, prints spawn pose. |
| `scripts/switch_maize_world.sh` | Swap active world between `full`/`small` snapshots; clears Gazebo terrain cache. |
| `worlds/agbot_corn_rows.world` | Lightweight non-maize fallback (4 rows × 36 corn, straight, flat). Segmentation is noticeably worse here than in maize worlds — kept for reference only. |
| `scripts/load_robot_description.sh` | The `JACKAL_URDF_EXTRAS` export fix. |

### Not in git
- `config/exported_best.pt` (~89 MB weights; gitignored) — also at `DINOv3-Segmentation-Training/out/corn_field_navigation/exported_models/`.
- `jackal/`, `virtual_maize_field/` — clone separately (see CLAUDE.md).
- `tmp/` — test frames + `segmentation_overlay.jpg`.
- `AgBot_MPC.pptx` — presentation deck (upload to Google Drive → open with Google Slides).
- Plan file: `~/.claude/plans/as-for-now-we-wild-moth.md` (mission design rationale).

---

## 5. NEXT STEPS (priority order)

### 1. Mission robustness runs (small maize world, RTF 1.0)
The first mission run passed with default thresholds. Now vary:
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=... camera_topic:=/camera/image_raw camera_topic_is_compressed:=false \
  mission_enabled:=true num_rows:=3          # + variations below
```
- Start from a different corridor / opposite end (pass spawn x/y/yaw to
  agbot_gazebo.launch), with `first_turn_direction:=right` where the field
  lies to the robot's right.
- `num_rows:=0` — verify the no-rows-left termination (REACQUIRE creeps
  `reacquire_max_distance` without a corridor → DONE).
- One run in the FULL world (`switch_maize_world.sh full`, spawn
  x:=3.16 y:=-9.31 z:=0.36 yaw:=1.791) to confirm behavior in curved rows —
  accept slow RTF, it's a logic check.
If any run misbehaves, tune from the HUD: exit early/late →
`exit_width_threshold` (0.8) / `exit_detect_frames` (5); never arms →
`min_in_row_distance` (2.0 — rows are only 6 m in the small world); clips last
plants → `headland_clearance` (1.0); turn overshoot → `turn_rate` (0.4) /
`yaw_tolerance_deg` (5); REACQUIRE never accepts → `reacquire_max_width` (0.6).

### 2. Field deployment (real robot, RTX 4080) — speed tuning happens HERE
1. `rostopic hz /cmd_vel` to measure the real control rate.
2. Start at the 0.15 defaults; raise via launch args stepwise
   (0.2 → 0.25 → 0.3), scaling `angular_z_max`, `delta_angular_z_max`,
   `mpc_alpha` proportionally. Watch angular_z: square-wave at the clamp =
   too fast for the loop; growing smooth oscillation = raise `mpc_r_delta`
   or lower `mpc_q_offset`; drifts wide on bends = raise `mpc_q_heading`.

### 3. Then: go-home (design already decided — see KEY DECISIONS)

### 4. Commit and push after each step (CLAUDE.md mandate)

---

## 6. GOTCHAS

### GOTCHA 1: This dev sandbox is ROS2 Humble — NOT ROS1
All `catkin`/`roslaunch`/`rostopic` commands run on the user's WSL2 Ubuntu
20.04 ROS1 Noetic machine. Unit tests DO run in the sandbox
(`pytest`+`scipy` installed via pip --user).

### GOTCHA 2: The model runs in `~/agbot_venv`, not system Python
`lightly_train`/`torch` are installed in the `~/agbot_venv` virtualenv on the
user's machine (this is how the smoke test and the working demo ran). Any
command importing `segmentation_model` needs that venv activated.

### GOTCHA 3: Mission thresholds validated in ONE configuration only
The first mission run (small maize world, 3 rows, default thresholds) passed,
but the thresholds have not been exercised across start corridors, turn
directions, `num_rows:=0`, or the curved full world. Use the `state=`/`w=`
debug HUD when a new configuration misbehaves.

### GOTCHA 4: Exit detector arms on odometry distance
If `/odometry/filtered` is missing/renamed, `distance_in_row` stays None and
the robot will NEVER leave FOLLOW_ROW (detector unarmed) — it will just stop
at the row end via the MPC's invalid-frame logic. Check the odom topic first.

### GOTCHA 5: `min_in_row_distance` interacts with slow row entry
FOLLOW_ROW's entry pose is recorded at the first odom fix (lazily at mission
start). The spawn point is 2 m before the corn, so by row end the distance is
well past 2.0 m — but if you respawn the robot mid-row for testing, the
detector may not arm before the row ends.

### GOTCHA 6: `<env>` tag does not reach `<param command>` (ROS1)
Camera URDF injection must stay via `load_robot_description.sh`. Do not revert.

### GOTCHA 7: scipy required at import time
`controller.py` imports scipy at module load; tests and node both fail without
it. `pip install scipy` (1.7.x is the last py3.8-compatible line).

### GOTCHA 8: GAZEBO_MODEL_PATH needs virtual_maize_field/models
Corn models `maize_01`/`maize_02` come from there even though we use our own
world. `source ~/agbot_control_ws/devel/setup.bash` first if corn is invisible.

### GOTCHA 9: `gh` CLI lives at `~/.local/bin/gh`
`export PATH="$HOME/.local/bin:$PATH"` may be needed before push (sandbox).

### GOTCHA 10: Don't `git add .` — ever
Stage files by name (CLAUDE.md rule). `*.pt`, `tmp/`, build artifacts and the
`.pptx` deck must stay out of commits.

### GOTCHA 11: Uploading .pptx to Google Drive via API from the sandbox fails
The Drive MCP tool requires inline base64 (~2× file size in characters) which
exceeds what fits through a tool call for a 140 KB deck. Just drag the file
into drive.google.com → Open with Google Slides.

---

## Quick-start (ROS1 Noetic machine)

```bash
# 1. Build
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# 2. Simulation
roslaunch agbot_bringup agbot_gazebo.launch

# 3. Row-following only (current tuning workflow)
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/home/chien21/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt \
  camera_topic:=/camera/image_raw camera_topic_is_compressed:=false

# 3b. Multi-row mission (sim-validated)
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=... camera_topic:=/camera/image_raw camera_topic_is_compressed:=false \
  mission_enabled:=true num_rows:=3

# 4. Monitor
rostopic echo /cmd_vel
rqt_image_view /vision_nav_node/debug/image

# 5. Unit tests (sandbox or ROS1 machine, no ROS needed)
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v     # expected: 38 passed
```
