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
3. **`agbot_gps_nav/`** — GPS waypoint navigation for open-sky transit
   (trailer → row entrance), where vision nav then takes over. Blank-world sim:
   `roslaunch agbot_bringup agbot_gps_sim.launch` + `roslaunch agbot_gps_nav
   gps_nav.launch sim:=true`.
4. **`DINOv3-Segmentation-Training/`** *(if present)* — model training pipeline (trains on annotated rosbag footage, produces `exported_best.pt`).
5. **`Papers/`** — lab papers (P-AgBot, P-AgSLAM, P-AgNav) and related external papers (Agronav, ROW-SLAM, CropFollow). Read before designing navigation/control logic.

**Third-party packages in this workspace** (not tracked in this repo):
- `jackal/` — Clearpath Jackal ROS1 driver (noetic-devel branch)
- `virtual_maize_field/` — procedural corn field world generator for Gazebo

## Critical environment split — read this before running anything

- ⚠ **Corrected 2026-09-06: the dev machine IS the ROS1 Noetic box.** Ubuntu
  20.04 (WSL2), `/opt/ros/noetic`, `DISPLAY=:0` via WSLg, and
  `~/agbot_control_ws/devel` already built. `catkin build`, `roslaunch`,
  `rostopic` and headless Gazebo (`gui:=false`) all run here directly, and a
  sim run can be driven end to end from this session. The previous note said
  this sandbox had ROS2 Humble and no ROS1; that was wrong (or has since
  changed) and cost a session's worth of "the user must run this" hedging.
  The **robot** is still a separate machine reached by git bundle (HANDOFF3 §0b).
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
Snapshots are kept under `~/.ros/virtual_maize_field_snapshots/` and swapped
with `scripts/switch_maize_world.sh <name>` (it lists what exists):
- **small** (default; matches the launch spawn-pose defaults): generated from
  `config/agbot_maize_small.yaml` — 4 straight rows × 6 m, 173 plants, no
  gaps, coarse flat heightmap, seed 42. RTF ≈ 1.0 on the dev laptop. 3
  corridors ≈ 20 m of driving, `num_rows:=3`.
- **long** (endurance): `config/agbot_maize_long.yaml` — 6 straight rows × 9 m,
  342 plants, same models and heightmap as small so segmentation sees identical
  visuals. 5 corridors ≈ 45 m, **`num_rows:=5`**, spawn pose
  `x:=1.531 y:=-5.830 z:=0.35 yaw:=1.575` (NOT the launch defaults — and read
  the pose the generator PRINTS; that one wins). Per-row `hole_prob` gives
  light missing stand in the four inner rows; **every hole is exactly ONE
  plant** (`hole_size_max: 2`, and `rng.integers(1, n)` is half-open), so the
  widest gap in the world is 0.38 m against 0.75 m row spacing. ⚠ The two
  OUTER rows are solid on purpose — a gap there opens onto the headland and is
  indistinguishable from a row end.
  ⚠ **Revised 2026-09-02 (second pass).** This world was 8 rows × 451 plants
  with holes up to 7 plants (1.09 m) wide, and it was not usable: the dev
  laptop's RTF sagged until Gazebo stalled in bursts and the debug overlay
  stopped tracking in real time, and the wide holes read as row ends often
  enough that a crash could not be attributed to any one cause. It is now a
  clean endurance baseline — no gap in it is ambiguous, so an `EXIT_CLEAR`
  fired mid-row here is a detector defect with nothing to hide behind. The
  8-row flank-check stress world is recoverable from git history; if that case
  is wanted back it belongs in its OWN config, not in the endurance baseline.
- **full**: the original FRE-style world (curved rows, dense heightmap).
  RTF < 0.1 on the laptop; spawn pose x:=3.16 y:=-9.31 z:=0.36 yaw:=1.791.

Generate any of them with `scripts/generate_maize_world.sh <config> <snapshot>`
(e.g. `agbot_maize_long long`); it snapshots the current world first.
`generate_small_maize_world.sh` remains as a wrapper for the small one.

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
- `src/agbot_vision_nav/mission_fsm.py` — multi-row mission state machine (FOLLOW_ROW → EXIT_CLEAR → TURN_1 → TRAVERSE → TURN_2 → REACQUIRE): odometry-closed-loop 90° headland turns, boustrophedon direction alternation, `num_rows` termination (0 = until no rows left), EXIT_CLEAR runs at `exit_clear_speed` (0.10, slower than cruise — the post-exit leg is where overshoot hurts). **EXIT_CLEAR is REAR-STEERED when a rear camera is present** (`exit_clear_rear_steering`, default true): the rear view looks back down the row being left, so the headland leg steers from it and turns only when the rear ALSO reads open field — the tail has cleared the last plants. ⚠ **The rear result is CONVERTED, not negated** (`rear_to_front_state`): the 180° mirror flips the sign of the *lateral* term but not the *heading* term (a yaw left moves the vanishing point to image-right in BOTH views, since both cameras rotate with the robot), so no single sign flip works — the old "negate iff exactly ONE of {mirrored view, reversed motion}" rule inverted heading feedback and steadily walked the robot off the row (sim, 2026-08-07). `slope_term` is the heading-free lateral readout that makes the inversion possible; `exit_clear_rear_offset_gain` (κ=2.0) is its one constant, set by the ratio of the scan rows' ground distances and therefore not a per-robot calibration. BACKOUT is a third case and still passes the rear result through UNCHANGED, because reversing flips the lateral dynamics too. ⚠ `headland_clearance` does NOT end that leg — it stays the terminator of the open-loop leg, which runs with no rear camera and is also the automatic fallback if no rear frame arrives at all. The rear open point is **back-dated to the streak start**, like the front path, and the rear watcher is a SEPARATE detector (`exit_clear_detector`) whose only difference from the front's thresholds is a much shorter `exit_clear_rear_confirm_distance` (0.1 vs 0.4): the front detector decides whether the row ended *at all* from inside the row, where a mid-row gap looks identical to a row end; by the time the rear one runs that is already settled and it only asks "has my tail passed the last plants". Charging the front's 0.4 m twice doubled the leg. ⚠ It is deliberately NOT the same object as `rear_exit_detector`, which terminates the field-proven BACKOUT reverse and must not inherit a headland tuning change. The node infers on the rear camera for the whole leg and switches back afterwards, so the debug image shows the rear view (HUD: `(REAR)`) while it lasts. ⚠ **The leg steers only on a rear corridor whose edges are INSIDE the image** (`nearest_row_corridor_is_bounded`) — `valid` cannot see that the corridor ran off the frame, and a border-clipped midpoint is fiction, which in a headland is the normal case: steering on it saturated `angular_z` for a whole leg and nearly drove the robot out of the world. An unusable rear frame is then handled exactly as the front controller handles an unusable front frame — no steering that tick. `exit_clear_max_distance` (1.5 m) is now a plain ceiling that *turns* — it used to un-count the row and resume FOLLOW_ROW mid-headland, where the detector re-arms over another 2 m of open field, which is how the sim robot reached the world edge. Blocked-row branch (BACKOUT → BACKOUT_CLEAR → BACKOUT_TURN_1 → BACKOUT_TRAVERSE → BACKOUT_TURN_2 → REACQUIRE): on a blocked-ahead signal the robot reverses out the end it entered (rear-camera-steered, odometry-bounded), S-turns into the next row (traveled in the SAME direction; the boustrophedon flip is suppressed once), records `blocked_events` reported at mission DONE. Rear steering reuses the MPC controller with UNCHANGED signs (image mirror + reversed motion cancel). The back-out is gated on `rear_camera_enabled`: without the rear camera the BACKOUT states are unreachable and a blocked signal stops the robot and ends the mission (reported).
- `src/agbot_vision_nav/metrics_logger.py` — per-run CSV performance metrics: one row per processed frame (tracking error, control, detector status, odometry, timing) plus `summarize()`. ON by default (`metrics_csv_dir: ~/agbot_logs`); `metrics_csv_dir:=none` skips a run. Report with `scripts/analyze_run.py <csv>...`. ⚠ `offset_norm` is normalized IMAGE space, **not meters**, and shifts with camera mount height (~0.5 tall vs ~0.7 low) — never compare across rigs; quote the FOLLOW_ROW row (TURN/TRAVERSE are odometry open loop).
- `src/agbot_vision_nav/intervention_detector.py` — the autonomy metric's definition of a human intervention: a **joystick takeover** (deadman held on `/bluetooth_teleop/joy`, buttons 4/5). Activity within `intervention_gap_seconds` (3.0) of the previous activity is the SAME intervention, so one messy rescue scores 1 and not 5. Nothing has to be pressed or remembered during a run. `intervention_joy_topic:=none` disables. The node writes a `teleop` column on every CSV row and an `INTERVENTION` event; `summarize()` turns those plus odometry into **meters per intervention (MDBI)**, the number field papers report — teleop-driven meters are subtracted, never credited to the controller. `/odometry/filtered` is now subscribed on **every** run (not just missions), because distance is the denominator.
- `src/agbot_vision_nav/debug_viz.py` — debug overlay image for `rqt_image_view` (mask wash, scan rows, midpoints, per-row corridor width `w=`, mission state HUD).
- `src/agbot_vision_nav/launch_args.py` — rospy-free roslaunch argv builder for the operator panel. Its one rule: **a blank field is not passed**, so `params.yaml` stays the source of truth and the panel cannot silently re-pin every knob at its own defaults.
- `scripts/vision_nav_node.py` — only file touching `rospy`/`cv_bridge`. Single-slot frame buffer, separate inference thread, 5 Hz watchdog, optional odometry subscriber. Mission mode is gated behind `~mission_enabled` (default **false** → plain row-following, identical to pre-mission behavior). Offers `~pause` (`std_srvs/SetBool`) and a latched `~status` string.
- `scripts/operator_panel.py` — PyQt panel: start the real robot's cameras / start mission with form fields / **pause + resume** / stop. `rosrun agbot_vision_nav operator_panel.py`. **The panel never starts Gazebo** (it did until 2026-08-07): the sim workflow is Gazebo in its own terminal (`roslaunch agbot_bringup agbot_gazebo.launch`), because its output otherwise buries the vision-nav startup config block, which is the part worth reading. Ticking `simulation` therefore does exactly one thing — puts `sim:=true` on the vision-nav launch, which picks the Gazebo camera topics — and greys the frame-source button out with a reminder.

⚠ **`--` is illegal inside an XML comment** (reserved for `-->`), and this repo writes `--` as an em-dash in prose everywhere. It broke `vision_nav.launch` outright on 2026-08-06. `test/test_launch_files.py` parses every launch/xacro and names the offending line — the sandbox has no ROS1, so pytest is the only place a launch-file error is caught before the robot sees it.

**Pause vs Ctrl-C** — they are not the same thing. Ctrl-C destroys the mission: `rows_driven`, the boustrophedon turn direction, the detector's arming distance and the row-entry pose all live in the node, so a restart begins at row 1. `~pause` publishes zero and **skips the FSM update**, so all of that survives. Resuming resets the detectors and the MPC, because the BLOCKED timer counts in ROS *seconds* — without that, a 30 s pause would deposit the whole `blocked_confirm_seconds` on the first frame back and fire a back-out nothing justified.

Run unit tests (no ROS or `lightly_train` needed):
```bash
cd agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v      # expected: 240 passed
cd ../agbot_gps_nav
PYTHONPATH=src python3 -m pytest test/ -v      # expected: 149 passed
```
`agbot_vision_nav/test/test_launch_files.py` walks the WHOLE workspace, so it
covers `agbot_gps_nav`'s and `agbot_bringup`'s launch files too — which is why
its count rose from 214 to 240 as the GPS package grew.

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

## agbot_gps_nav — GPS waypoint navigation (trailer → row)

Gets the robot from the trailer to the front of a row across open ground, then
vision nav takes over. GPS is used **here and nowhere else** — the lab's own
P-AgNav paper is explicit that in-row navigation must work without GNSS.

Same architecture as `agbot_vision_nav`: a rospy-free core under
`src/agbot_gps_nav/`, unit-tested with plain pytest, and exactly one rospy file.

```bash
# blank world (empty Gazebo, bare Jackal, simulated GNSS at the datum)
roslaunch agbot_bringup agbot_gps_sim.launch rviz:=true
roslaunch agbot_gps_nav gps_nav.launch sim:=true
# then: RViz "2D Nav Goal" (Fixed Frame MUST be map), or a lat/lon:
rostopic pub -1 /gps_nav_node/goal_wgs84 geometry_msgs/PointStamped \
  '{header: {frame_id: wgs84}, point: {x: -86.9910, y: 40.4695}}'   # x=LON, y=LAT

# THE WHOLE CAPABILITY: trailer -> row entrance -> hand off -> 3-row mission
rosrun agbot_bringup switch_maize_world.sh gps
roslaunch agbot_bringup agbot_gazebo.launch gps:=true \
  x:=-0.798 y:=-21.361 z:=0.35 yaw:=1.5708     # the trailer, 18 m out
roslaunch agbot_gps_nav gps_vision_mission.launch sim:=true num_rows:=3
rostopic echo /mission_supervisor/status
```

**The handoff.** `mission_supervisor.py` sequences it; the decisions live in
the rospy-free `handoff_fsm.py`. ⚠ **A disabled node goes SILENT, not zeros.**
Both nodes publish `/cmd_vel`, which twist_mux takes as ONE input at priority 1
and arbitrates by **priority, not rate** — two publishers on it are not
arbitrated at all, they interleave, and zeros from a "stopped" node shred the
active one's commands. So `~set_enabled` false emits one zero then goes quiet,
while `~pause` keeps emitting zeros (a paused node is still in charge and should
hold the robot immediately). Measured: driving 19.8 Hz non-zero, paused 19.8 Hz
zero, disabled **0.0 Hz**. ⚠ Opposite polarity: `~pause` true = stop,
`~set_enabled` true = go. Enabling vision nav **resets** the mission
(`MissionFSM.reset()`) so the exit detector arms from the row entrance and not
from wherever the transit began.

**Arriving on a bearing.** A row entrance is a point *plus a direction*. Given
`approach_bearing`, the follower stages back along the axis, turns to it
(`ALIGN`), and runs the final leg tracking the **axis line, not the goal
point** — steering at the point corrects lateral offset in the last metre and
swings the robot as it arrives (measured: 41.2° → 0.4°). ⚠ And arrival is
**crossing the goal plane** within half a row spacing of the axis, never
entering a radius: a radius let the robot pass 0.46 m to the side and drive on
up the field with the distance stuck at 0.47 m.

⚠ **`first_turn_direction` must match the corridor.** Out of the leftmost
corridor the next one is to the robot's RIGHT. Turning left out of `corridor_0`
drives out of the field and ends the mission at `rows=1/3`.
`scripts/rows_to_waypoints.py` generates the waypoints (and the matching turn
direction) from the world's own `gt_map.csv`, because the layout is seeded and
procedural — regenerate them whenever the world is regenerated.

⚠ **Not bugs**, recorded so nobody chases them: the ~1/s `Transform from ...
unavailable ... Using latest instead` warnings are throttled upstream behaviour
(`navsat_transform` has no `transform_timeout` at all; measured position
agreement is 2 cm), and `World frame->cartesian transform is Origin:
(-501151.9, ...)` is an internal cartesian origin, not a datum error. ⚠ **Do not
teleport the robot with `/gazebo/set_model_state`** and expect the map-frame
heading to follow — the EKF's yaw is dead-reckoned and a teleport is invisible
to it. Restart the sim at the pose you want.

- `src/agbot_gps_nav/geo.py` — WGS84 ↔ local ENU about a datum. Local tangent
  plane using the WGS84 meridional **and** prime-vertical radii at the datum
  latitude (they differ by 0.39 % at Purdue; using one for both is a real bug a
  round-trip test cannot see). No pyproj/geodesy — neither is installed and
  neither is needed. Measured against Vincenty: 1.5 mm at 280 m, 3.7 cm at
  1.4 km, 0.94 m at 7 km, so **the answer to a bigger site is a nearer datum**.
- `src/agbot_gps_nav/waypoint_follower.py` — `IDLE → GOTO → APPROACH → ARRIVED`,
  same `update()` signature and 4-tuple return as `MissionFSM`. Turns in place
  above `turn_in_place_deg` (prevents the go-to-goal orbit, where a close
  off-axis goal sits inside a turning circle that can never close) and derates
  forward speed with heading error. ARRIVED is **latched**; only a new goal
  releases it.
- `scripts/gps_nav_node.py` — the only rospy file. Publishes `/cmd_vel` so
  twist_mux keeps the joystick (priority 9-10) above autonomy (priority 1).
- `launch/mapviz.launch` — the map view. ⚠ Use this, not `rosrun mapviz mapviz`
  plus File > Open Config: mapviz needs `/local_xy_origin` to know where `map`
  sits on Earth, nothing published it, and without it the canvas is blank grey
  and looks like a broken tile source. The launch runs `initialize_origin.py`
  off the datum that `gps_nav_node` latches on `~datum_fix`, so the datum still
  has exactly one definition.
- `config/gps_datum.yaml` — ⚠ **the single definition of the field origin**,
  read by both `navsat_transform` (as `datum`) and, in sim, by
  `load_robot_description.sh` as `GAZEBO_WORLD_LAT/LON`. Setting them equal
  makes map coordinates and Gazebo coordinates the same numbers. **Still a
  TODO(datum) placeholder near ACRE — replace with a surveyed fix before saving
  any field waypoint; every stored waypoint is relative to it.**

**Localization is a dual EKF.** ⚠ The Jackal's own EKF is untouched and keeps
publishing `/odometry/filtered` + `odom→base_link`, which `vision_nav_node`
depends on. We only add `ekf_map` (`world_frame: map`) publishing
**`/odometry/filtered/global`** and owning `map→odom`. Never collide with
`/odometry/filtered`: an odom-frame estimate must stay smooth for the row
controller, while a map-frame one jumps whenever GPS corrects.

### ⚠ Heading — measured 2026-09-06, and NOT what the plan assumed

GPS gives position, never orientation. This robot has no usable compass
(Clearpath say so), a single Reach M2 outputs no yaw, and the stock EKF
deliberately does not fuse absolute yaw — so its heading is gyro-integrated
from whatever direction the robot booted facing. Gazebo's hector IMU publishes
a **fake absolute heading**; consuming it would make every sim pass and every
field run fail, so `use_odometry_yaw: true` is mandatory and `imu0_config`
leaves absolute yaw `false`.

The expectation was that this blocks everything until a heading estimator
exists. **It does not** — but it is not free either, and 2026-09-07 measured
where the line falls. Measured in the blank world:

| condition | heading error vs Gazebo ground truth |
|---|---|
| spawned at `yaw:=1.2` (74° wrong), at rest | 74° |
| after 3 m of the bootstrap (yaw process noise 0.3) | **6.6°** |
| after 10 m of the same leg | **0.7°** |
| after 10 m with the OLD 0.01 tuning | **~48°** |
| stationary, any start | **drifts ~17°/min, unbounded** |

`ekf_map` fuses body-frame wheel velocities with absolute GPS positions, so
**while the robot is moving, yaw is observable** — GPS-observed direction of
travel disagrees with the direction predicted from yaw, and the filter corrects
it. ⚠ **Observable is not observed**: until 2026-09-07 `ekf_map.yaml`'s
`process_noise_covariance` gave x/y `1.0` against yaw `0.01`, and since position
and yaw compete for that one GPS innovation, a term 100× stiffer never moved.
Yaw is now `0.3`. Measured A/B, blank world, 66° start: `0.01` gave up at the
10 m backstop still **47.9°** wrong; `0.3` converged at **3.0 m with 6.3°**, and
0.7° by 10 m. ⚠ The drift is continuous, not just a bad start —
`jackal.gazebo` models `rateDrift 0.005 rad/s` = **17°/min**, measured as −17.4°
of odom yaw while the robot sat still for 50 s waiting for the model to load.

That is course-over-ground heading estimation happening implicitly. A→B
therefore works at **any** spawn yaw today:

```bash
roslaunch agbot_bringup agbot_gps_sim.launch yaw:=1.2   # still arrives
```

What it costs: converging from 74° took a **6.2 m lateral excursion** on a 20 m
leg. Near a row entrance that is a collision — and in the maize world it was: a
92° start with no bootstrap missed the row entrance by **5.9 m**.

So `heading_init_distance` (a **bootstrap**, not the full estimator) is now
required for the maize transit and is on by default in
`gps_vision_mission.launch`. It drives STRAIGHT — deliberately not steering,
since steering on the estimate is the circularity being broken — until **course
over ground** agrees with the yaw estimate. ⚠ It ends on that MEASUREMENT, not
a distance: a fixed 4 m was tried and was not enough, because how far it needs
depends on how wrong the estimate started. It needs clear ground ahead, which is
why the `gps` maize world has a 20 m headland.

⚠ The bootstrap drives at **cruise**, not `approach_speed`: it is an
observability manoeuvre and the signal grows with ground speed, so 0.15 m/s was
both the weakest signal available and the longest wait. It also now logs a
`logerr` when it ends on the backstop rather than on the measurement — sharing
one "done" loginfo with the success case is how a transit ran its whole length
71.9° wrong with nothing flagged.

⚠ **Arriving is not arriving pointed the right way, and the handoff is gated on
that.** GPS pins position, never orientation. `ALIGN` turns until the *estimate*
reads the bearing, so heading-error-vs-bearing proves nothing; the independent
check is the course actually driven over the final on-axis leg
(`approach_course_error()` → `~arrival_heading_error_deg` → `handoff_fsm`,
refused over `max_arrival_heading_error_deg` 12°). An unmeasurable residual is
allowed through with a warning: the gate fires on evidence of being wrong, not
on its absence. `arrival_cross_tolerance` is 0.20 in the mission launch, not
params.yaml's 0.35 — the Jackal is ~0.43 m wide in a 0.75 m corridor, so 0.35 m
off axis is already inside the plant row.

A full `heading_estimator.py` (Phase 3) is still unbuilt and still wanted: the
bootstrap fixes the start of a run, not the unbounded at-rest drift. ⚠ Do not
"fix" that drift by fusing the IMU's absolute yaw — that is the sim-only signal
above.

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

**Speed: never raise `linear_x_cruise` alone.** Defaults are the sim-validated
0.15 m/s envelope, and `angular_z_max`, `delta_angular_z_max` and `mpc_alpha`
are tuned *at that speed* and do not transfer. Path curvature is what keeps the
robot in a row — `kappa = angular_z / linear_x` — so an unchanged
`angular_z_max` of 0.175 is a 0.86 m turn radius at 0.15 m/s and a **2.86 m
radius at 0.5 m/s**: 3.3× less turning per metre driven, against ~0.16 m of
clearance per side in a 0.75 m row. A sim run that raised only the speed sat
pinned at the clamp and drove over every plant (2026-09-02,
`~/agbot_logs/vision_nav_20260902_173947.csv`; HANDOFF3 §0f). Use:

```bash
roslaunch agbot_vision_nav vision_nav.launch model_path:=... \
  $(python3 agbot_vision_nav/scripts/speed_args.py 0.5 --control-period 0.13)
```

`src/agbot_vision_nav/speed_profile.py` holds the arithmetic and states which
knobs deliberately do NOT scale: `mpc_beta` (a heading rate, speed-independent),
`mpc_dt` (the MEASURED control period of the machine — 0.431 s on the laptop
sim, ~0.04 s on the GPU robot — not a speed constant), and every
distance-valued detector threshold (speed-invariant by design; that is what
fixed the 2026-07-24 field failure).

⚠ **Do not raise `max_data_age_sec` to silence a fast run's `WATCHDOG_ZERO`
events** — at 0.5 m/s a 1.0 s window is half a metre of blind driving, and the
watchdog is correctly reporting that the loop cannot keep up. In sim the answer
is `agbot_bringup/scripts/set_sim_rtf.sh <factor>`: `use_sim_time` is on, so
slowing wall-clock time raises the *sim-time* inference rate and leaves every
threshold self-consistent (RTF 0.3 turns a 0.431 s wall cycle into a 0.13 s sim
cycle, ~7.7 Hz — the same feedback-per-metre as the proven 0.15 m/s runs). The
run then measures the controller instead of the laptop. On the robot the answer
is a faster machine.

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

Endurance run on the **long** world (45 m, 5 corridors, light gaps), at the
proven speed and then at a scaled 0.5 m/s with the simulator throttled so the
run measures the controller rather than the laptop:

```bash
rosrun agbot_bringup switch_maize_world.sh long
roslaunch agbot_bringup agbot_gazebo.launch x:=1.531 y:=-5.830 z:=0.35 yaw:=1.575

# baseline, proven envelope
roslaunch agbot_vision_nav vision_nav.launch model_path:=... sim:=true \
  mission_enabled:=true num_rows:=5 rear_camera_enabled:=true

# fast run: throttle Gazebo FIRST, then scale the coupled knobs
rosrun agbot_bringup set_sim_rtf.sh 0.3
roslaunch agbot_vision_nav vision_nav.launch model_path:=... sim:=true \
  mission_enabled:=true num_rows:=5 rear_camera_enabled:=true \
  $(python3 agbot_vision_nav/scripts/speed_args.py 0.5 --control-period 0.13)
```

Compare the two CSVs with `analyze_run.py`. The fast run passes only if
`WATCHDOG_ZERO` is 0, `|angular_z|` p95 is strictly below the new clamp
(0.583 — still pinned means the scaling did not take), and the FOLLOW_ROW RMS
`offset_norm` is comparable to the baseline. Same world and mount, so that
comparison is legitimate; across rigs it never is.

Smoke-test the segmentation model on one saved image (no ROS; uses the
`~/agbot_venv` virtualenv where `lightly_train`/`torch` are installed):
```bash
source ~/agbot_venv/bin/activate
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 smoke_test_segmentation.py   # edit paths at top of file
```
