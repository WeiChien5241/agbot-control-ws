# HANDOFF3.md

Handoff for the P-AgBot vision-nav work, updated end of session 2026-07-21.
Field status: in-row nav + headland turns WORK on the real robot (tall
camera). This session sim-validated the blocked-row back-out through four
fix iterations; ONE open bug remains (§0). Also added: pipeline timing
instrumentation, model_device/CUDA handling, offline benchmark script,
traverse_distance param. Written so a fresh session picks up with zero
context. Supersedes all previous HANDOFF3 content.

---

## 0. START HERE — open bug: after back-out, no turn, mission ends DONE

Last sim run of the session (small maize world, box blocker,
`sim:=true mission_enabled:=true rear_camera_enabled:=true num_rows:=3`):
the back-out itself WORKED — blocked fired, robot reversed steering from
the rear camera (direction correct), and the new rear-exit watcher ended
the reverse near the row entrance instead of overshooting. But then the
robot "did not turn and just returned the state as finished": no visible
S-turn, state went to DONE, mission over. NOT diagnosed yet.

Ranked hypotheses (mission_fsm.py):
1. **REACQUIRE gave up** (`REACQUIRE` → DONE when no corridor within
   `reacquire_max_distance` 2.0 m). The S-turn (BACKOUT_TURN_1 →
   BACKOUT_TRAVERSE → BACKOUT_TURN_2) is quick and easy to miss in rqt;
   if it DID happen, the failure is REACQUIRE not recognizing the next
   row (`_corridor_looks_like_row`: needs valid centerline + mean
   corridor width < `reacquire_max_width` 0.6 for `reacquire_frames` 3
   consecutive frames). From the headland the corridor may read wider
   than 0.6 until the robot is fairly close.
2. **`_done_after_backout` path** (BACKOUT_CLEAR → DONE directly, no
   turn): taken only when the blocked row was the FINAL counted row
   (`rows_driven >= num_rows` at block time — that behavior is BY
   DESIGN). If the blocker was in row 1 of 3 and this path fired,
   `rows_driven` was somehow inflated — that would be a real bug (each
   exit/blocked event increments it).
3. Odometry hiccup is unlikely (maneuver states just hold, not DONE).

**How to disambiguate in one run:** watch the console for the one-shot
`Mission DONE: rows_driven=N, blocked rows: row K blocked at X m` line —
N tells you instantly whether hypothesis 2 applies (N should be 1 if the
box was in row 1). Watch the HUD `state=` sequence right before DONE:
`BACKOUT_CLEAR → DONE` = hypothesis 2; `...TURN_2 → REACQUIRE → DONE` =
hypothesis 1. Record a bag of `/odometry/filtered /cmd_vel
/vision_nav_node/debug/image`. If it's hypothesis 1, candidate knobs:
`reacquire_max_width` (0.6 → 0.7) or `reacquire_max_distance` (2.0 →
2.5); also check BACKOUT_TURN_2's end pose actually faces the next row.

---

## 1. GOAL

Vision-based navigation for the Purdue P-AgBot (Clearpath Jackal): DINOv3
segmentation → traversability mask → centerline estimation → image-space MPC
keeps the robot centered in a corn row. Multi-row boustrophedon missions
with odometry headland turns; blocked-row back-out via a rear camera.
Row-following and headland turns are FIELD-PROVEN (real corn, tall camera,
2026-07); back-out is sim-validated except the §0 bug.

---

## 2. CURRENT STATE

### Field (real robot, 2026-07, tall camera)
- In-row navigation: works. Headland turns: work.
- One field issue fixed in code afterwards: at the end of TRAVERSE the nose
  got too close to the NEXT row's corn → new `traverse_distance` param
  (see below). NOT yet field-re-tested.
- The front-deck (low) camera experiment is CONCLUDED — user is back on the
  tall mount. `agbot_camera.urdf.xacro` still carries an UNCOMMITTED
  working-tree diff (tall line active, front-deck line commented, stand at
  x=-0.025). It is the user's state: don't commit or revert unprompted.

### Sim back-out validation (2026-07-21, four iterations — all committed)
1. Blocked never fired at a big box (robot stopped, stayed FOLLOW_ROW):
   ground-fraction gate too strict up close. `blocked_min_traversable_fraction`
   0.15 → 0.08 → **0.02** (HUD showed frac=0.04 in front of the box; the
   gate now only rejects a truly black view; 0.0 disables it). Blocked
   debounce made LEAKY (noise frames decrement, not reset, the counter).
2. Reverse leg overshot meters past the row entrance: BACKOUT unwound the
   full odometry distance d_block, which includes the PRE-ROW approach
   (row 1's FOLLOW_ROW starts at the SPAWN point ~2 m before the row) →
   REACQUIRE (then 1.5 m) never reached the next row → mission ended.
   Fix: **rear-exit watcher** — during BACKOUT the rear centerline runs
   the open-exit signature (same thresholds, armed immediately,
   `MissionFSM.rear_exit_detector`); reversing ends as soon as the row
   opens up behind the robot; d_block stays as the upper bound.
   `reacquire_max_distance` 1.5 → 2.0.
3. Confirmed working in sim: blocked fires, reverse steering direction
   correct (no negation — as the unit tests predicted), early rear-exit
   stop works.
4. Remaining: §0 (no turn after back-out, straight to DONE).

### This session's other additions (all committed + pushed)
- **Timing instrumentation** (`src/agbot_vision_nav/timing_stats.py`,
  rospy-free): node logs a 5 s-throttled `timing:` line and a debug-HUD
  line — camera Hz, processed Hz (= control rate), inference ms
  (mean/p95), end-to-end camera-stamp→cmd_vel latency, dropped-frame %
  (high dropped % is BY DESIGN of latest-frame-wins). Sim laptop measured
  ~5.7 Hz / 165 ms inference / 288 ms e2e.
- **`model_device` param (default `auto`)**: SegmentationModel moves the
  model to CUDA when available; node logs `Model loaded on device: ...`.
  **On the RTX 4060 robot check that line first** — `cpu` there means
  torch has no usable CUDA and explains any slowness. `nvidia-smi` while
  the node runs must show the python process.
- **`scripts/benchmark_inference.py`** (no ROS): identical run on laptop /
  CPU robot / GPU robot → comparison table (device, mean/p50/p95 ms, FPS).
  Run inside `~/agbot_venv`.
- **`traverse_distance`** (default 0.6 m): TRAVERSE and BACKOUT_TRAVERSE
  drive this instead of `row_spacing` (0.75, unchanged — still the
  physical spacing). Stops ~0.15 m short of the next row's centerline;
  REACQUIRE + MPC close the offset. ⚠ **Precedence gotcha:** the user set
  `traverse_distance: 0.65` in params.yaml, but the launch file's
  `<param>` (from the launch ARG default 0.6) OVERRIDES params.yaml —
  `<param>` tags come after the `<rosparam file>` load. To actually get
  0.65 pass `traverse_distance:=0.65` at launch, or change the launch
  arg default. This applies to EVERY knob that is both a launch arg and a
  params.yaml entry: the launch arg default wins.
- **Diagnostics on the HUD** (debug_viz/vision_nav_node): FOLLOW_ROW shows
  `exit: blk n/8 open n/5 rows= frac= armed o: b:` (why the exit detector
  is/isn't firing); BACKOUT shows `rear exit: open n/5 wide=` plus 1 Hz
  BACKOUT telemetry log (rear offset/slope → angular_z, reverse progress).

Session commits (oldest first): `b6fcc3f` timing stats module, `cd880ff`
traverse_distance + backout_progress, `05dd56c` node wiring (timing,
device, telemetry), benchmark script, `778c202` blocked deadlock fix +
detector HUD, `6bc8c52` gate → 0.02, `e8b8746` rear-exit early stop.

### Detector semantics (row_exit_detector.py, current)
- OPEN: ≥ `exit_open_rows_required` (1) scan rows — ANY of them — with
  corridor width ≥ `exit_width_threshold` (0.8), `exit_detect_frames` (5)
  CONSECUTIVE frames, armed after `min_in_row_distance` (2.0 m).
- BLOCKED: zero corridors at ALL scan rows + `traversable_fraction` ≥
  `blocked_min_traversable_fraction` (**0.02**), accumulated over
  `blocked_detect_frames` (8) with a LEAKY counter (non-signature frames
  decrement by 1, floor 0), armed after `blocked_arming_distance` (0.3 m).
- `last_status` (ExitDetectorStatus namedtuple) exposes all internals per
  frame — that's what the HUD renders.

### Mission FSM (mission_fsm.py, current)
FOLLOW_ROW → EXIT_CLEAR (0.10 m/s, `headland_clearance` 0.75) → TURN_1 →
TRAVERSE (`traverse_distance`) → TURN_2 → REACQUIRE → FOLLOW_ROW;
boustrophedon sign flip per transition. Blocked branch: BACKOUT (reverse,
rear-steered, ends on rear open-exit OR d_block bound) → BACKOUT_CLEAR
(reverse `headland_clearance` more) → BACKOUT_TURN_1 → BACKOUT_TRAVERSE →
BACKOUT_TURN_2 (counter-rotate, S-shape) → REACQUIRE with ONE suppressed
sign flip; next row same world direction. Blocked FINAL row: backs fully
out then DONE (no turn — remember this when judging §0). Gated:
`backout_enabled` = mission_enabled AND rear_camera_enabled; without rear
camera a blocked signal stops + DONE + report. Rear steering signs
UNCHANGED (mirror × reverse = identity; `test_backout_rear_steering_signs`).

### Node (vision_nav_node.py)
Single-slot latest-frame buffers (front + rear share one Condition), one
inference thread, rear frames consumed ONLY in STATE_BACKOUT, 10 Hz
watchdog (hold last cmd until `max_data_age_sec`, then zero). Frame slots
carry (frame, header stamp, recv time) for the timing stats. Debug topic:
`/vision_nav_node/debug/image` — during BACKOUT it automatically shows the
REAR camera (`state=BACKOUT (REAR)`); there is NO separate rear debug
topic. Zero-code rear preview: launch the node with
`camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false
cmd_vel_topic:=/cmd_vel_rear_preview`.

---

## 3. KEY DECISIONS (do not re-litigate without new information)

- **Exit detection: open = "ANY N rows wide", never specific/farthest rows.**
  Field-tested twice: beyond the field edge, far scan rows can stay invalid
  FOREVER (garbage segmentation of distant ground); a farthest-rows
  criterion never fired and the robot drove off the world edge. Tripwire
  test: `test_open_fires_when_only_near_row_wide`.
- **Blocked ground-fraction gate ≈ 0** (0.02): up close a blocker fills the
  frame (sim-measured frac=0.04), so any meaningful gate deadlocks the
  back-out. The 8-frame debounce is the real occlusion guard.
- **BACKOUT ends on rear-camera open-exit, odometry as upper bound only** —
  odometry-only reversing overshoots whenever d_block includes pre-row
  travel (always true for row 1 from spawn).
- **Rear-steering signs: NO negation** (mirror × reverse cancel) — sim-
  confirmed this session. Don't "fix" without failing the unit test first.
- **Back-out geometry: S-turn, same-direction next row, suppress ONE flip.**
- **Rear inference only during STATE_BACKOUT** (other BACKOUT_* states are
  odometry-only) — don't run two cameras' inference on the 2 Hz CPU robot.
- **No rear camera ⇒ blocked = stop + DONE + report** (user's choice).
- Image-space MPC (scipy SLSQP, N=8), no EKF; `mpc_dt` scales alpha/beta/
  rate-limit (specified at 0.1 s reference; CPU robot uses mpc_dt:=0.5).
- `mission_enabled` defaults false. Small maize world for sim
  (`switch_maize_world.sh full|small`). Camera URDF injection via
  `load_robot_description.sh` (JACKAL_URDF_EXTRAS — `<env>` doesn't reach
  `<param command>` in ROS1). Go-home deferred (mirrored-mission chosen).

---

## 4. FILES

### `agbot_vision_nav/`
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/segmentation_model.py` | lightly_train wrapper; classes 0=sky,1=traversable,2=obstacle; `device=` param + `device_str`. |
| `src/agbot_vision_nav/centerline_estimator.py` | 3 scan rows outward from center column → `CenterlineResult`. |
| `src/agbot_vision_nav/controller.py` | `MPCRowController` (SLSQP, N=8, dt-scaled). Reused unchanged for rear reverse steering. |
| `src/agbot_vision_nav/row_exit_detector.py` | OPEN/BLOCKED signatures, leaky blocked debounce, per-signature arming, `last_status`. |
| `src/agbot_vision_nav/mission_fsm.py` | Mission FSM incl. BACKOUT branch, `rear_exit_detector`, `backout_progress()`, `traverse_distance`, `blocked_events`. |
| `src/agbot_vision_nav/timing_stats.py` | Rolling pipeline metrics (`format_summary()`, `hud_line()`). |
| `src/agbot_vision_nav/debug_viz.py` | HUD overlay: state, per-row `w=`, `timing_line`, `detector_line`. |
| `scripts/vision_nav_node.py` | Only rospy file. Dual frame slots + stamps, camera source by FSM state, watchdog, timing/detector/BACKOUT logging. |
| `scripts/benchmark_inference.py` | Offline cross-machine inference benchmark (no ROS). |
| `config/params.yaml` + `launch/vision_nav.launch` | All knobs. ⚠ launch-arg defaults override params.yaml (see §2). |
| `test/` | **71 tests**: controller 17, centerline 9, viz 3, detector 17, fsm 18, timing 7. |

### `agbot_bringup/`
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Gazebo + Jackal + camera URDF override + RViz. |
| `launch/display.launch.xml` | RViz-only URDF viewer (needs ros-noetic-joint-state-publisher-gui). |
| `urdf/agbot_camera.urdf.xacro` | `agbot_cam` macro; front (tall, `xyz="0 0 0.225"`) + rear (`xyz="-0.05 0 0.225"` yaw π → `/camera_rear/image_raw`). **UNCOMMITTED user working-tree diff** (stand x=-0.025, front-deck line commented). |
| `scripts/load_robot_description.sh` | JACKAL_URDF_EXTRAS injection. |
| `config/agbot_maize_small.yaml`, `scripts/*maize*` | Small-world workflow. |

### Not in git
Model weights (`config/exported_best.pt`), `jackal/`, `virtual_maize_field/`,
`tmp/`, `AgBot_MPC.pptx`. Claude memory dir: `project_camera_relocation.md`
(experiment concluded), `project_robot_deployment.md`.

---

## 5. NEXT STEPS (priority order)

1. **Diagnose + fix §0** (no turn after back-out → DONE). One instrumented
   rerun disambiguates; see §0 for the exact readout and knobs.
2. **Full back-out mission end-to-end in sim**: blocked row 1 → back out →
   S-turn → reacquire row 2 → finish 3 rows → check
   `Mission DONE: rows_driven=3, blocked rows: row 1 blocked at X m`.
   Also the no-rear case (same blocker, `rear_camera_enabled` off →
   stop + report) and a nudge test during BACKOUT (drag robot off-center;
   telemetry must show angular_z re-centering).
3. **Field re-test** of the traverse fix (`traverse_distance` — decide
   0.6 vs the user's intended 0.65, see the precedence gotcha in §2) and
   of exits on the CPU robot (pull latest first; gentle recipe + mpc_dt
   0.5 as documented in vision_nav.launch comments).
4. **GPU robot bring-up**: check the `Model loaded on device:` line; run
   `benchmark_inference.py` on laptop / CPU robot / GPU robot and build
   the FPS comparison table the user's colleagues asked for.
5. Then: mission robustness matrix (`first_turn_direction:=right`,
   `num_rows:=0`, full world), speed tuning on the GPU robot, go-home.

---

## 6. GOTCHAS

1. **Never key the exit on far scan rows** (see §3 first bullet) — the
   hard-won one. Don't restore commit `54e8ef8`'s detector logic.
2. **Launch-arg defaults override params.yaml** for every duplicated knob
   (`<param>` after `<rosparam file>` wins). The user's params.yaml edit
   `traverse_distance: 0.65` is currently INEFFECTIVE (launch default 0.6
   applies) — resolve per §5.3.
3. **`agbot_camera.urdf.xacro` working-tree diff is the user's** (tall
   camera active — the experiment is over, but the diff is uncommitted).
   Ask before committing/reverting.
4. **Exit detector arms on odometry distance**: no `/odometry/filtered` ⇒
   never arms ⇒ never leaves FOLLOW_ROW. Blocked arms at 0.3 m, open at
   2.0 m — respawning mid-row can leave open detection unarmed (6 m rows).
5. **Blocked FINAL row goes DONE without turning by design** — don't
   misread that as the §0 bug; check `rows_driven` in the DONE log.
6. **This dev sandbox is ROS2 Humble, not ROS1** — catkin/roslaunch/
   rostopic run on the user's WSL2 ROS1 Noetic machine (same filesystem).
   Unit tests DO run in the sandbox. Model runs in `~/agbot_venv`.
7. scipy needed at import. GAZEBO_MODEL_PATH needs virtual_maize_field/
   models. `gh` at `~/.local/bin/gh`. **Never `git add .`** (weights/tmp/
   pptx stay out). High dropped-frame % in the timing log is BY DESIGN.

---

## Quick-start (ROS1 Noetic machine)

```bash
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# Simulation world
roslaunch agbot_bringup agbot_gazebo.launch

# Mission with back-out (the §0 repro command)
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  mission_enabled:=true rear_camera_enabled:=true num_rows:=3

# Rear-camera segmentation preview (robot idle, cmd_vel diverted)
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false \
  cmd_vel_topic:=/cmd_vel_rear_preview

# Monitor
rqt_image_view /vision_nav_node/debug/image  # state HUD, w=, timing, exit/blk counters
rostopic echo /cmd_vel
# Console: 'timing:' every 5 s, 'exit: blk n/8' countdown, BACKOUT telemetry,
#          one-shot 'Mission DONE: rows_driven=N, blocked rows: ...'

# Offline inference benchmark (per machine, in the venv)
source ~/agbot_venv/bin/activate
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 scripts/benchmark_inference.py \
  --model config/exported_best.pt --image /path/to/frame.jpg

# Unit tests (no ROS needed)
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v     # expected: 71 passed
```
