# HANDOFF3.md

Handoff for the P-AgBot vision-nav work, updated end of session 2026-07-24.
Field status: in-row nav + headland turns WORK on the real robot. This
session (2026-07-24) brought up the new NVIDIA-GPU robot, found and FIXED a
mid-row false-EXIT_CLEAR bug caused by the low camera + fast inference, and
surfaced a safety issue (joystick takeover) that is DEFERRED. Written so a
fresh session picks up with zero context. Supersedes all previous HANDOFF3
content.

---

## 0. START HERE — this session (2026-07-24)

### DONE + committed: mid-row false EXIT_CLEAR fixed (commit `bcf7d6d`)
On the new GPU robot (fast inference) the mission FSM falsely fired
`EXIT_CLEAR` **in the middle of a row**, drove into the corn, and — because
the pipeline is so fast — committed before anyone could react. Root cause:
1. That robot runs a **LOW-mounted camera**. Low mount raises the *normal*
   in-row corridor width at the nearest scan row from ~0.5 (tall) to ~0.7.
2. A few **missing corn plants on the sides** push the near-row normalized
   corridor width to ~0.83 — above `exit_width_threshold` (0.8).
3. The OPEN signature was **width-only**, so it read that as open field.
4. Fast inference satisfied the 5-frame debounce almost instantly.

**Fix (matches the user's rule — "look left/right of the corridor; if corn
still flanks it we're still in the row; only fire when it reaches the image
edges on both sides"):** a scan row now counts toward the open exit only if
its corridor is **wide AND flank-clear** — i.e. it reaches within
`exit_flank_edge_margin` (0.05 of image width) of **BOTH** image borders, so
no corn borders the corridor. A mid-row gap widens the corridor but leaves
corn short of the edge → NOT an open row → no false exit. A true row end runs
edge-to-edge → fires. Uses the corridor bounds the detector already has, so
the segmentation model, `centerline_estimator.py`, and `CenterlineResult` are
**untouched**. Rear back-out watcher mirrors the same margin. New knob
`exit_flank_edge_margin` (>= 1.0 disables → pre-fix width-only behavior);
disabled by using a large value. HUD now shows `wide=` and `openrows=`.
**78 tests pass** (72 → 78; +6). All prior tests stayed green, including the
early-approach and world-edge tripwires (open field reaches the edges).

**Next:** field-validate on the GPU robot — at a mid-row gap the HUD should
read `openrows=0` even when a `w=` shows ≥0.8; `openrows≥1` only at the true
row end. Tune `exit_flank_edge_margin` from there (smaller = must reach the
edges more exactly; larger = more permissive). See §5.

### DEFERRED (user scoped out this session) — two open items
- **SAFETY: joystick takeover on the fast robot.** The grad student could
  NOT override cmd_vel when the robot hit corn; killing the node was the only
  recourse. Rate-dependent (fine on the 2 Hz robot, broken on the fast GPU
  robot). BUT: the node publishes to `/cmd_vel` = twist_mux `external` input
  (priority 1) and the joystick is `bluetooth_teleop/cmd_vel` (priority 9), so
  twist_mux SHOULD override regardless of publish rate. A rate-dependent
  failure is the fingerprint of the mux being bypassed — verify on the robot:
  (a) is the deployed node really on `/cmd_vel` or was `cmd_vel_topic`
  overridden onto the controller/teleop topic; (b) is `twist_mux` from
  `jackal_control/launch/control.launch` actually running and the sole
  publisher to `jackal_velocity_controller/cmd_vel`; (c) nothing remapping
  `/cmd_vel` onto the controller. Recommended: fix the wiring AND add an
  in-node operator-override subscriber (yield when joystick input present).
  **Address soon — safety-relevant.** Full analysis in the plan file
  `~/.claude/plans/read-the-handoff3-md-to-squishy-hennessy.md`.
- **GPS RTK (Emlid Reach RS2) for trailer↔row transit.** A separate, larger
  subsystem: fuse GPS via the Jackal's existing `robot_localization` EKF, then
  `move_base` or a GPS-waypoint follower to drive trailer→row and row→trailer.
  Yes, ground robots do RTK waypoint nav much like PX4/QGC on a drone.
  Turning-via-GPS (professor's idea) is optional — vision headland turns
  already work. Not needed for any current bug; future capability plan.

### Prior §0 (blocked-count) — remains FIXED in code (commit `7003a3b`)
`mission_fsm` used to increment `rows_driven` on EVERY exit (including
blocked) and end the mission early. Fixed 2026-07-22: `rows_driven` increments
ONLY on an OPEN exit; a block ALWAYS backs out + S-turns to the next physical
row and never ends the mission on its own. Tests
`test_blocked_row_does_not_count_continues_to_next_row` and
`test_blocked_middle_row_still_requires_full_num_rows` cover it. STILL WANTS a
sim/field confirmation run (`num_rows=2`, blocker in row 2 → back-out → S-turn
→ drive row 3 → `Mission DONE: rows_driven=2, blocked rows: row 2 blocked at
X m`). If the S-turn happens but REACQUIRE fails to latch the next row (a
SEPARATE issue), try `reacquire_max_width` 0.6→0.7 or `reacquire_max_distance`
2.0→2.5.

---

## 1. GOAL

Vision-based navigation for the Purdue P-AgBot (Clearpath Jackal): DINOv3
segmentation → traversability mask → centerline estimation → image-space MPC
keeps the robot centered in a corn row. Multi-row boustrophedon missions
with odometry headland turns; blocked-row back-out via a rear camera.
Row-following and headland turns are FIELD-PROVEN (real corn, 2026-07);
back-out is sim-validated.

---

## 2. CURRENT STATE

### Robots (multiple; camera height is per-robot and matters)
- **New GPU robot (this session, 2026-07-24):** fast inference (user reports
  much faster than the CPU robot). Runs a **LOW-mounted camera** — normal
  near-row corridor width ~0.7 (vs ~0.5 tall). The low mount is what caused
  the mid-row false EXIT_CLEAR now fixed (§0). ⚠ **Segmentation caveat:** the
  model was trained on ~300 tall-camera annotations but only ~100 low-camera
  ones, so low-camera masks may be weaker on the sides — more low-camera
  annotations would help and could also reduce the missing-corn misreads.
- **CPU robot cpr-j100-0463:** ~2 Hz CPU inference; live row-following
  achieved. Use `mpc_dt:=0.5`, raise `max_data_age_sec` (~1.5-3.0).
- **Tall-camera field runs (2026-07):** in-row nav + headland turns proven.

### Field behavior (real robot, 2026-07)
- In-row navigation: works. Headland turns: work.
- End-of-TRAVERSE nose-too-close fix: `traverse_distance` param (below), NOT
  yet field-re-tested.
- `agbot_camera.urdf.xacro` still carries an UNCOMMITTED working-tree diff
  (tall line active, front-deck line commented, stand at x=-0.025). It is the
  user's state: don't commit or revert unprompted. (The physical GPU robot's
  low camera is a separate real-robot mount, independent of this sim URDF.)

### Sim back-out validation (2026-07-21, four iterations — all committed)
1. Blocked never fired at a big box: ground-fraction gate too strict up close.
   `blocked_min_traversable_fraction` 0.15 → 0.08 → **0.02** (HUD showed
   frac=0.04 in front of the box; 0.0 disables it). Blocked debounce made
   LEAKY (noise frames decrement, not reset, the counter).
2. Reverse leg overshot past the row entrance: BACKOUT unwound the full
   odometry d_block, which includes the PRE-ROW approach (row 1's FOLLOW_ROW
   starts at the SPAWN point ~2 m before the row). Fix: **rear-exit watcher** —
   during BACKOUT the rear centerline runs the open-exit signature (armed
   immediately, `MissionFSM.rear_exit_detector`); reversing ends as soon as the
   row opens up behind; d_block stays the upper bound. `reacquire_max_distance`
   1.5 → 2.0.
3. Confirmed in sim: blocked fires, reverse steering correct (no negation),
   early rear-exit stop works.
4. §0 blocked-count fix (2026-07-22) + mid-row flank-clear fix (2026-07-24).

### Prior instrumentation (all committed + pushed, earlier sessions)
- **Timing** (`src/agbot_vision_nav/timing_stats.py`, rospy-free): 5 s-throttled
  `timing:` line + HUD line. **`inf` = wall-clock around ONLY `model.predict()`
  (`time.monotonic()`), mean + p95 over the last 50 inferences.** **`dropped %`
  = `1 − frames_processed/frames_received`, cumulative since startup** — with
  latest-frame-wins semantics a frame that arrives while inference is busy is
  overwritten and never processed, so it counts frame *skipping*, not lost
  commands; high % is BY DESIGN (camera outruns inference). Control rate = the
  `proc` Hz figure. Sim laptop: ~5.7 Hz / 165 ms inf / 288 ms e2e.
- **`model_device` param (default `auto`):** SegmentationModel moves to CUDA
  when available; node logs `Model loaded on device: ...`. **Check that line on
  the GPU robot first** — `cpu` there means torch has no usable CUDA.
- **`scripts/benchmark_inference.py`** (no ROS): cross-machine inference table.
- **`traverse_distance`** (default 0.6 m): TRAVERSE/BACKOUT_TRAVERSE drive this
  instead of `row_spacing` (0.75). ⚠ Precedence gotcha (see §6).

### Detector semantics (row_exit_detector.py, current)
- OPEN: ≥ `exit_open_rows_required` (1) scan rows — ANY of them — that are BOTH
  wide (corridor width ≥ `exit_width_threshold` 0.8) AND **flank-clear**
  (corridor reaches within `exit_flank_edge_margin` 0.05 of BOTH image edges,
  i.e. no corn flanking it). `exit_detect_frames` (5) CONSECUTIVE frames,
  armed after `min_in_row_distance` (2.0 m). `exit_flank_edge_margin >= 1.0`
  disables the flank term (width-only). Helper: `flank_clear_flags()`.
- BLOCKED: zero corridors at ALL scan rows + `traversable_fraction` ≥
  `blocked_min_traversable_fraction` (0.02), accumulated over
  `blocked_detect_frames` (8) with a LEAKY counter, armed after
  `blocked_arming_distance` (0.3 m). UNCHANGED this session.
- `last_status` (ExitDetectorStatus namedtuple) exposes internals incl.
  `wide_rows` and the new `open_rows` (wide AND flank-clear count) — that's
  what the HUD renders.

### Mission FSM (mission_fsm.py, current)
FOLLOW_ROW → EXIT_CLEAR (0.10 m/s, `headland_clearance` 0.75) → TURN_1 →
TRAVERSE (`traverse_distance`) → TURN_2 → REACQUIRE → FOLLOW_ROW;
boustrophedon sign flip per transition. Blocked branch: BACKOUT (reverse,
rear-steered, ends on rear open-exit OR d_block bound) → BACKOUT_CLEAR →
BACKOUT_TURN_1 → BACKOUT_TRAVERSE → BACKOUT_TURN_2 (counter-rotate, S-shape) →
REACQUIRE with ONE suppressed sign flip; next row same world direction. A
blocked row does NOT count toward num_rows; only open-exit rows count. Gated:
`backout_enabled` = mission_enabled AND rear_camera_enabled; without rear
camera a blocked signal stops + DONE + report. Rear steering signs UNCHANGED
(mirror × reverse = identity). `rear_exit_detector` now also mirrors the
front `exit_flank_edge_margin`.

### Node (vision_nav_node.py)
Single-slot latest-frame buffers (front + rear share one Condition), one
inference thread, rear frames consumed ONLY in STATE_BACKOUT, 10 Hz watchdog
(hold last cmd until `max_data_age_sec`, then zero). Publishes to `cmd_vel_topic`
(default `/cmd_vel`). Debug topic `/vision_nav_node/debug/image` (during BACKOUT
shows the REAR camera; no separate rear debug topic). Zero-code rear preview:
`camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false
cmd_vel_topic:=/cmd_vel_rear_preview`.

---

## 3. KEY DECISIONS (do not re-litigate without new information)

- **Open exit = wide AND flank-clear** (2026-07-24). A row counts toward the
  open exit only if its corridor reaches within `exit_flank_edge_margin` of
  BOTH image edges (no corn flanking). Rejects the low-camera mid-row
  side-gap false positive (width ~0.83 but corn short of the edge). Strictly
  better than raising the width threshold: requires BOTH sides clear, not just
  a large total width. Implemented in the detector from existing corridor
  bounds — segmentation/centerline untouched. Tripwire tests:
  `test_open_blocked_by_flank_corn_mid_row_gap`, `test_one_sided_edge_does_not_fire`.
- **Exit detection: open = "ANY N rows wide", never specific/farthest rows.**
  Beyond the field edge, far scan rows can stay invalid FOREVER (garbage
  segmentation); a farthest-rows criterion never fired and the robot drove off
  the world edge. Tripwire: `test_open_fires_when_only_near_row_wide`. The
  flank-clear gate does NOT re-couple to far rows — it's per-row on whatever
  row is wide.
- **Blocked ground-fraction gate ≈ 0** (0.02): up close a blocker fills the
  frame (frac=0.04); the 8-frame debounce is the real occlusion guard.
- **BACKOUT ends on rear-camera open-exit, odometry as upper bound only.**
- **Rear-steering signs: NO negation** (mirror × reverse cancel). Don't "fix"
  without failing the unit test first.
- **Back-out geometry: S-turn, same-direction next row, suppress ONE flip.**
- **Rear inference only during STATE_BACKOUT.**
- **No rear camera ⇒ blocked = stop + DONE + report** (user's choice).
- Image-space MPC (scipy SLSQP, N=8), no EKF; `mpc_dt` scales alpha/beta/
  rate-limit (0.1 s reference; CPU robot uses mpc_dt:=0.5).
- `mission_enabled` defaults false. Small maize world for sim
  (`switch_maize_world.sh full|small`). Camera URDF injection via
  `load_robot_description.sh` (JACKAL_URDF_EXTRAS). Go-home deferred.
- **Joystick takeover: keep the node on `/cmd_vel`** (twist_mux `external`,
  priority 1); the joystick at priority 9 is meant to override it. Do NOT
  point `cmd_vel_topic` at the controller/teleop topic (that bypasses the mux
  and is the likely cause of the takeover failure — see §0 DEFERRED).

---

## 4. FILES

### `agbot_vision_nav/`
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/segmentation_model.py` | lightly_train wrapper; classes 0=sky,1=traversable,2=obstacle; `device=` param + `device_str`. |
| `src/agbot_vision_nav/centerline_estimator.py` | 3 scan rows outward from center column → `CenterlineResult` (untouched this session). |
| `src/agbot_vision_nav/controller.py` | `MPCRowController` (SLSQP, N=8, dt-scaled). Reused unchanged for rear reverse steering. |
| `src/agbot_vision_nav/row_exit_detector.py` | OPEN (wide AND flank-clear via `flank_clear_flags()` + `exit_flank_edge_margin`) / BLOCKED signatures, leaky blocked debounce, per-signature arming, `last_status` (incl. `open_rows`). |
| `src/agbot_vision_nav/mission_fsm.py` | Mission FSM incl. BACKOUT branch, `rear_exit_detector` (mirrors flank margin), `backout_progress()`, `traverse_distance`, `blocked_events`. |
| `src/agbot_vision_nav/timing_stats.py` | Rolling pipeline metrics; `inf` = predict() wall time, `dropped %` = skipped frames (by design). |
| `src/agbot_vision_nav/debug_viz.py` | HUD overlay: state, per-row `w=`, `timing_line`, `detector_line` (renders whatever string the node builds — no change needed for new fields). |
| `scripts/vision_nav_node.py` | Only rospy file. Frame slots + stamps, camera source by FSM state, watchdog, timing/detector/BACKOUT logging; HUD detector line now shows `wide=`/`openrows=`. |
| `scripts/benchmark_inference.py` | Offline cross-machine inference benchmark (no ROS). |
| `config/params.yaml` + `launch/vision_nav.launch` | All knobs incl. `exit_flank_edge_margin` (0.05). ⚠ launch-arg defaults override params.yaml (§6). |
| `test/` | **78 tests**: controller 17, centerline 9, viz 3, detector 22, fsm 20, timing 7. |

### `agbot_bringup/`
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Gazebo + Jackal + camera URDF override + RViz. |
| `launch/display.launch.xml` | RViz-only URDF viewer. |
| `urdf/agbot_camera.urdf.xacro` | `agbot_cam` macro; front + rear (yaw π → `/camera_rear/image_raw`). **UNCOMMITTED user working-tree diff** — don't touch. |
| `scripts/load_robot_description.sh` | JACKAL_URDF_EXTRAS injection. |
| `config/agbot_maize_small.yaml`, `scripts/*maize*` | Small-world workflow. |

### Not in git
Model weights (`config/exported_best.pt`), `jackal/`, `virtual_maize_field/`,
`tmp/`, `AgBot_MPC.pptx`. Claude memory dir: `project_camera_relocation.md`,
`project_robot_deployment.md`, `project_perf_benchmarks.md`.

---

## 5. NEXT STEPS (priority order)

1. **Field-validate the mid-row flank-clear fix (`bcf7d6d`)** on the GPU robot:
   drive a row with a known side gap → must NOT flip to EXIT_CLEAR mid-row
   (HUD `openrows=0` even when a `w=` shows ≥0.8; `openrows≥1` only at the true
   row end). Tune `exit_flank_edge_margin` (0.05) from the HUD if a real
   open-field frame ever fails to fire (raise it) or a gap still trips it
   (lower it, or check segmentation).
2. **SAFETY: fix joystick takeover** (§0 DEFERRED). Verify the mux wiring on
   the robot and/or add an in-node operator override. Do before more
   autonomous field runs.
3. **Full back-out mission end-to-end in sim** (confirms the 2026-07-22
   blocked-count fix): blocked row 1 → back out → S-turn → reacquire → finish
   3 rows → `Mission DONE: rows_driven=3, blocked rows: row 1 blocked at X m`.
   Also the no-rear case and a BACKOUT nudge test.
4. **Field re-test** of the `traverse_distance` fix (decide 0.6 vs 0.65 — see
   §6 precedence gotcha) and exits on the CPU robot (`mpc_dt:=0.5`).
5. **GPU robot bring-up finish**: confirm `Model loaded on device: cuda`, run
   `benchmark_inference.py` on laptop / CPU robot / GPU robot for the FPS table.
   Consider collecting **more low-camera annotations** to strengthen
   segmentation on the low-mounted rigs.
6. **GPS RTK plan** (§0 DEFERRED) if/when trailer autonomy is prioritized.
7. Then: mission robustness matrix (`first_turn_direction:=right`,
   `num_rows:=0`, full world), speed tuning on the GPU robot, go-home.

---

## 6. GOTCHAS

1. **Open exit now needs BOTH wide AND flank-clear** (2026-07-24). If exits
   stop firing in real open field, the flanks aren't reaching the image edges —
   raise `exit_flank_edge_margin` or check segmentation, don't revert. To fully
   restore old width-only behavior set `exit_flank_edge_margin >= 1.0`.
2. **Never key the exit on far scan rows** — the hard-won one. Don't restore
   commit `54e8ef8`'s detector logic. (The flank gate is per-row, not far-row.)
3. **Launch-arg defaults override params.yaml** for every duplicated knob
   (`<param>` after `<rosparam file>` wins). Applies to `exit_flank_edge_margin`
   (0.05 in both), `traverse_distance` (launch 0.6 wins over the user's
   params.yaml 0.65 — pass `traverse_distance:=0.65` to actually get it), etc.
4. **`agbot_camera.urdf.xacro` working-tree diff is the user's** — ask before
   committing/reverting. Physical robot camera height is separate from this sim
   URDF.
5. **Exit detector arms on odometry distance**: no `/odometry/filtered` ⇒ never
   arms ⇒ never leaves FOLLOW_ROW. Blocked arms at 0.3 m, open at 2.0 m.
6. **Blocked rows do NOT count toward num_rows** (2026-07-22).
7. **Joystick takeover**: keep the node on `/cmd_vel`; don't override
   `cmd_vel_topic` onto the controller/teleop topic (bypasses twist_mux
   priority — the suspected cause of the takeover failure).
8. **This dev sandbox is ROS2 Humble, not ROS1** — catkin/roslaunch/rostopic
   run on the user's WSL2 ROS1 Noetic machine (same filesystem). Unit tests DO
   run in the sandbox. Model runs in `~/agbot_venv`.
9. scipy needed at import. GAZEBO_MODEL_PATH needs virtual_maize_field/models.
   `gh` at `~/.local/bin/gh`. **Never `git add .`** (weights/tmp/pptx stay out).
   High dropped-frame % in the timing log is BY DESIGN.

---

## Quick-start (ROS1 Noetic machine)

```bash
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# Simulation world
roslaunch agbot_bringup agbot_gazebo.launch

# Mission with back-out
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  mission_enabled:=true rear_camera_enabled:=true num_rows:=3

# Tune the new flank gate if needed (0.05 default; >=1.0 disables)
#   ... exit_flank_edge_margin:=0.05

# Rear-camera segmentation preview (robot idle, cmd_vel diverted)
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false \
  cmd_vel_topic:=/cmd_vel_rear_preview

# Monitor
rqt_image_view /vision_nav_node/debug/image
# HUD detector line: 'exit: blk n/8 open n/5 rows= wide= openrows= frac= armed o: b:'
#   openrows=0 with wide>=1 at a mid-row gap == the flank gate working.
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
PYTHONPATH=src python3 -m pytest test/ -v     # expected: 78 passed
```
