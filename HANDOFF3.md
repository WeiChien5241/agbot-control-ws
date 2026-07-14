# HANDOFF3.md

Handoff for the P-AgBot vision-nav work. This session added: RViz-only URDF
viewer, a front-deck camera relocation experiment (NOT yet working well), a
rear camera + blocked-row back-out feature (NOT yet sim-verified), and a
two-round rework of row-exit detection (one field-tested regression and its
fix — read GOTCHA 1 before touching the exit detector). Written so a fresh
session can continue with zero context. Supersedes the previous HANDOFF3.

---

## 1. GOAL

Vision-based navigation for the Purdue P-AgBot (Clearpath Jackal): DINOv3
segmentation → traversability mask → centerline estimation → image-space MPC
keeps the robot centered in a corn row. Row-following is demoed working
(Gazebo, YouTube video); multi-row boustrophedon missions are sim-validated.
Current thrusts: (a) relocate the front camera low on the front deck to avoid
leaf occlusions, (b) rear camera + reverse-out-of-row behavior for blocked
rows, (c) make row-exit detection fast and robust enough that the robot never
overruns the row end (the sim world has a cliff past the field edge; real
fields have crops/holes).

---

## 2. CURRENT STATE

### Working & committed (branch `main`, all pushed)

Recent commits (this session):
- `1dc8754` display.launch.xml + camera xacro macro + rear camera + commented
  front-deck origin
- `5ce633c` blocked-row back-out (BACKOUT states, rear-camera steering)
- `54e8ef8` far-rows exit criterion + back-out gating — **the exit part of
  this commit was a REGRESSION, do not restore its detector logic**
- `0ef6b55` regression fix: any-N-rows open criterion + `exit_clear_speed`

**Sim/bringup:**
- `roslaunch agbot_bringup display.launch.xml` — RViz-only URDF view (no
  Gazebo) for camera-placement iteration. ROS1 syntax; needs
  `ros-noetic-joint-state-publisher-gui`.
- `agbot_bringup/urdf/agbot_camera.urdf.xacro` — cameras are now a xacro macro
  `agbot_cam(name, xyz, rpy)`. Front instantiation `name="camera"` preserves
  all historical names/topics (`camera_link`, `/camera/image_raw`,
  `camera_optical_frame`). Rear camera `name="camera_rear"` at
  `xyz="-0.05 0 0.225" rpy="0 0 pi"` → `/camera_rear/image_raw`,
  `camera_rear_optical_frame`.

**Vision-nav (53/53 unit tests pass —
`cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v`):**
- **Row-exit detection (row_exit_detector.py), current semantics:**
  - OPEN: at least `exit_open_rows_required` (**1**) scan rows — ANY of them —
    have corridor width ≥ `exit_width_threshold` (0.8), for
    `exit_detect_frames` (5) consecutive frames, armed after
    `min_in_row_distance` (2.0 m).
  - BLOCKED: zero corridors at ALL scan rows + `traversable_fraction` ≥ 0.15,
    for `blocked_detect_frames` (**8**, deliberately longer — foliage brushing
    the lens must not trigger a back-out), armed already after
    `blocked_arming_distance` (**0.3 m**) so mid-row obstacles near the row
    entrance are caught.
- **Blocked-row back-out (mission_fsm.py):** FOLLOW_ROW --blocked-->
  BACKOUT (reverse the odometry-recorded in-row distance, steering from the
  REAR camera) → BACKOUT_CLEAR (reverse `headland_clearance` more, straight)
  → BACKOUT_TURN_1 (`+turn_sign` 90°) → BACKOUT_TRAVERSE (`row_spacing`,
  forward) → BACKOUT_TURN_2 (`-turn_sign` 90°, S-shaped lane change) →
  REACQUIRE. Next row is entered from the SAME end / traveled the SAME world
  direction, so the boustrophedon `_turn_sign` flip at the following
  REACQUIRE is suppressed exactly once (`_suppress_flip`). Row still counts
  toward `num_rows`; `blocked_events` (row, distance) reported in the
  mission-DONE log. Blocked FINAL row backs fully out before DONE.
  - **Gated**: `backout_enabled` = `rear_camera_enabled`. Rear camera off ⇒
    BACKOUT states unreachable; a blocked signal stops the robot and ends the
    mission with the blocked row reported (user's explicit choice).
  - **Rear steering sign rule (derived + unit-tested): NO negation.** The
    180° image mirror and the reversed motion cancel exactly — rear
    `offset_norm`/`slope_term` go into `MPCRowController.compute()` unchanged;
    only linear velocity is negative (`backout_speed` 0.10).
- **vision_nav_node.py:** optional rear subscriber (only when
  `mission_enabled` AND `rear_camera_enabled`), second single-slot frame
  buffer sharing one Condition; the single inference thread reads the REAR
  frame only while `fsm.state == STATE_BACKOUT` (front otherwise → zero extra
  cost in normal operation). Debug overlay shows `state=... (REAR)` during
  back-out. One-shot mission-DONE log lists blocked rows.
- **Exit-leg speed:** EXIT_CLEAR now drives `exit_clear_speed` (0.10) instead
  of cruise (0.15); `headland_clearance` reduced 1.0 → **0.5 m**.
- **Launch args added to vision_nav.launch:** `headland_clearance`,
  `exit_width_threshold`, `exit_open_rows_required`, `exit_detect_frames`,
  `blocked_detect_frames`, `exit_clear_speed`, `rear_camera_enabled`,
  `rear_camera_topic`, `rear_camera_topic_is_compressed`, `backout_speed`.

### NOT working / unverified (start here next session)

1. **Front-deck camera position** — UNCOMMITTED experiment in
   `agbot_camera.urdf.xacro`: front camera currently ACTIVE at
   `xyz="0.19 0 0.025"` (low, front deck), tall-stand line
   `xyz="0 0 ${cam_z}"` commented out, stand moved to `x=-0.025`. User
   reports it "still not working": in-row nav is fine at the low position,
   but exits misbehaved across two test rounds (see GOTCHA 1 timeline). The
   `0ef6b55` fix has NOT yet been re-tested in sim at either camera position.
   The user may revert to the tall mount — treat the toggle as theirs.
2. **Backward movement (BACKOUT)** — implemented + unit-tested but NEVER
   exercised in Gazebo. Needs: rear camera image sanity check
   (`rostopic hz /camera_rear/image_raw`), then a blocker run with
   `rear_camera_enabled:=true` (drop a ~0.5 m box mid-corridor via Gazebo
   Insert). Watch for steering sign errors during reverse (unit tests say no
   negation is correct; verify by nudging the robot off-center in BACKOUT).
3. **Exit re-validation** — after `0ef6b55`, run the plain 3-row mission at
   BOTH camera positions and confirm the robot turns before the world edge.

---

## 3. KEY DECISIONS (do not re-litigate without new information)

### Exit detection: open = "ANY N rows wide", never "specific rows" (GOTCHA 1)
Two field-tested iterations landed here:
- v1 (original): all VALID rows ≥ 0.8 → fired only when the NEAREST row saw
  open field → 1.5–2.5 m overshoot with the low camera (robot off the world).
- v2 (`54e8ef8`, REGRESSION): required the 2 FARTHEST rows valid+wide →
  NEVER fired, because beyond the field edge the segmentation of distant
  ground is garbage (far rows permanently invalid) → robot off the cliff at
  every exit, both camera heights.
- v3 (current, `0ef6b55`): ≥ `exit_open_rows_required` (1) rows wide, ANY
  position. Provably never later than v1, earlier where far rows segment
  well, immune to far-row garbage. Regression test:
  `test_open_fires_when_only_near_row_wide`.

### Back-out gating and no-rear fallback
Without a rear camera there is no safe maneuver on blocked: stop + DONE +
report (user chose this over "wait with timeout" and "ignore blocked").

### Rear-steering signs: unchanged (mirror × reverse = identity)
Documented in mission_fsm docstring; guarded by
`test_backout_rear_steering_signs`. Don't "fix" the signs without failing
that test first.

### Back-out geometry: S-turn, same-direction next row, suppress ONE flip
After backing out the entry end, the next row is boustrophedon-shifted, not
alternated. User explicitly accepted this.

### Rear inference only during BACKOUT
Normal-operation cost unchanged; the other BACKOUT_* states are
odometry-only. Don't run both cameras' inference concurrently on the 2 Hz
CPU robot.

### Earlier decisions still binding
- Image-space MPC, no EKF; scipy SLSQP; sign conventions locked by tests.
- Headland maneuvers odometry-closed-loop; `row_spacing` 0.75 m measured.
- `mission_enabled` defaults false; speed tuning belongs on the real robot
  (0.15 m/s envelope in sim; scale `angular_z_max`, `delta_angular_z_max`,
  `mpc_alpha` with speed).
- Small maize world for sim (RTF 1.0); `switch_maize_world.sh full|small`.
- Camera URDF injection via `load_robot_description.sh` (JACKAL_URDF_EXTRAS).
- Go-home deferred (mirrored-mission approach chosen, not started).

---

## 4. FILES

### `agbot_vision_nav/`
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/segmentation_model.py` | lightly_train model wrapper; mask classes 0=sky,1=traversable,2=obstacle. |
| `src/agbot_vision_nav/centerline_estimator.py` | 3 scan rows outward from center column → `CenterlineResult(offset_norm, slope_term, valid, traversable_fraction, scan_rows)`. |
| `src/agbot_vision_nav/controller.py` | `MPCRowController` (SLSQP, N=8). Reused unchanged for rear-camera reverse steering. |
| `src/agbot_vision_nav/row_exit_detector.py` | OPEN/BLOCKED signatures, per-signature debounce AND arming (see §2). |
| `src/agbot_vision_nav/mission_fsm.py` | Mission FSM incl. BACKOUT branch, `backout_enabled` gate, `blocked_events`, `exit_clear_speed`, `BACKOUT_STATES` export. |
| `src/agbot_vision_nav/debug_viz.py` | HUD overlay (`state=`, per-row `w=`); rear frames get `state=... (REAR)`. |
| `scripts/vision_nav_node.py` | Only rospy file. Dual frame slots, FSM-state-driven camera source selection, watchdog, mission-DONE report. |
| `config/params.yaml` + `launch/vision_nav.launch` | All knobs; exit/back-out params now launch args (list in §2). |
| `test/` | 53 tests (controller 15, centerline 9, viz 2, detector 13, fsm 14). |

### `agbot_bringup/`
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Gazebo + Jackal + camera URDF override + RViz. |
| `launch/display.launch.xml` | RViz-only URDF viewer (robot_state_publisher + joint_state_publisher_gui + rviz). |
| `urdf/agbot_camera.urdf.xacro` | `agbot_cam` macro; front + rear instantiations; **currently carries the user's UNCOMMITTED front-deck toggle**. |
| `config/agbot_maize_small.yaml`, `scripts/*maize*` | Small-world workflow (unchanged). |
| `scripts/load_robot_description.sh` | JACKAL_URDF_EXTRAS injection. |

### Not in git
Model weights (`config/exported_best.pt`), `jackal/`, `virtual_maize_field/`,
`tmp/`, `AgBot_MPC.pptx`. Memory dir has `project_camera_relocation.md`
(experiment state + the far-rows lesson).

---

## 5. NEXT STEPS (priority order)

1. **Re-validate exits after `0ef6b55`** (ROS1 machine, small maize world):
   plain 3-row mission, rear camera off, at the CURRENT (front-deck) camera
   position, then at the tall position (swap the two `xacro:agbot_cam
   name="camera"` lines). Success = state flips to EXIT_CLEAR when any `w=`
   hits ≥0.8 for 5 frames; robot turns well before the world edge. Knobs if
   late: `exit_detect_frames:=3`; if a sparse row false-fires:
   `exit_open_rows_required:=2`.
2. **Decide the front camera position.** If the low position still can't see
   exits acceptably (its scan rows only look ~0.5–1.4 m ahead vs ~0.9–2.5 m
   from the tall mount), consider: revert to tall mount; or an intermediate
   height; or re-annotating/fine-tuning the model with low-viewpoint frames.
   Commit the URDF once decided (the toggle is currently uncommitted).
3. **First Gazebo test of the back-out**: box blocker mid-corridor,
   `mission_enabled:=true rear_camera_enabled:=true num_rows:=3`. Verify:
   BACKOUT (REAR) overlay, negative linear.x, reverse steering direction,
   S-turn, next-row reacquire, `Mission DONE ... row N blocked at X m`.
   Also the no-rear case: same blocker, rear off → robot stops + reports.
4. Then: mission robustness matrix (other corridors,
   `first_turn_direction:=right`, `num_rows:=0`, full world), field
   deployment speed tuning, go-home.

---

## 6. GOTCHAS

### GOTCHA 1 (NEW, hard-won): never key the exit on far scan rows
Beyond the field edge the model does not segment ground reliably: the far
scan rows can be invalid FOREVER. Any exit criterion that requires specific
(especially farthest) rows will simply never fire and the robot will drive
off the cliff/into whatever is past the row. Field-tested twice. Current
any-N-rows criterion is the fix; `test_open_fires_when_only_near_row_wide`
is the tripwire.

### GOTCHA 2 (NEW): low camera ⇒ short lookahead + leaf-level false BLOCKED
At the front-deck position (~0.27 m up) the scan rows map to ~0.5–1.4 m of
ground and row-end foliage crosses the lens height — this produced a real
false-BLOCKED-triggered reverse in testing. Mitigations in place: 8-frame
blocked debounce, back-out gated on rear camera. If false blocks persist,
raise `blocked_detect_frames` or `blocked_min_traversable_fraction`.

### GOTCHA 3 (NEW): the front-deck URDF toggle is UNCOMMITTED user state
`agbot_camera.urdf.xacro` working-tree change = the user's live experiment
(front-deck active, tall commented, stand at x=-0.025). Don't revert, don't
commit without asking, don't assume the committed state matches disk.

### GOTCHA 4 (NEW): back-out is unit-tested only — no Gazebo run yet
Especially the reverse steering sign (tests prove the FSM applies no
negation; they cannot prove the physical convention). First sim test should
nudge the robot off-center during BACKOUT and check it re-centers.

### GOTCHA 5: this dev sandbox is ROS2 Humble — NOT ROS1
All catkin/roslaunch/rostopic commands run on the user's WSL2 ROS1 Noetic
machine. Unit tests DO run in the sandbox.

### GOTCHA 6: model runs in `~/agbot_venv` on the user's machine.

### GOTCHA 7: exit detector arms on odometry distance
No `/odometry/filtered` ⇒ detector never arms ⇒ robot never leaves
FOLLOW_ROW. Blocked arms at 0.3 m, open at 2.0 m — respawning mid-row for
testing can leave open detection unarmed at the row end (6 m rows).

### GOTCHA 8: `<env>` doesn't reach `<param command>` (ROS1) — keep
`load_robot_description.sh`. GOTCHA 9: scipy needed at import. GOTCHA 10:
GAZEBO_MODEL_PATH needs virtual_maize_field/models. GOTCHA 11: `gh` at
`~/.local/bin/gh`. GOTCHA 12: never `git add .` (weights/tmp/pptx stay out).

---

## Quick-start (ROS1 Noetic machine)

```bash
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# URDF-only view (camera placement iteration)
roslaunch agbot_bringup display.launch.xml

# Simulation
roslaunch agbot_bringup agbot_gazebo.launch

# Mission, no rear camera (blocked => stop + report)
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  model_path:=/home/chien21/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt \
  mission_enabled:=true num_rows:=3

# Mission with back-out enabled
#   + rear_camera_enabled:=true
# Exit tuning knobs:
#   headland_clearance:=0.5 exit_clear_speed:=0.10 exit_detect_frames:=5
#   exit_open_rows_required:=1 blocked_detect_frames:=8

# Monitor
rqt_image_view /vision_nav_node/debug/image   # state= HUD + per-row w=
rostopic echo /cmd_vel

# Unit tests (sandbox or ROS1 machine, no ROS needed)
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v     # expected: 53 passed
```
