# GPS RTK trailer→row transit, plus the exit-overshoot fix

> ⚠ **STATUS 2026-09-06 — partly IMPLEMENTED, and one premise below is now
> known to be WRONG. Read `HANDOFF3.md` §0g before acting on this file.**
>
> **Done:** Phase 0 (sim-first), Phase 2 (dual EKF + `navsat_transform`),
> Phase 4 (waypoint follower + `gps_nav_node`, now with approach bearings),
> Phase 5 (RViz + mapviz configs; mapviz installed but never opened), and
> **Phase 6 (the vision handoff)** — `mission_supervisor.py` runs trailer → row
> entrance → vision nav → 3 corridors → DONE, end to end in simulation
> (2026-09-07, HANDOFF3 §0h).
>
> **Partly done:** Phase 3. The *bootstrap* half exists as
> `heading_init_distance` and the maize transit needs it; the full
> course-over-ground estimator does not, and the unbounded at-rest drift is
> still unaddressed.
>
> **Not done:** Phase 1 (Reach M2 hardware) **and Part A below**
> (`headland_clearance` 0.75 → 1.0), still open.
>
> ⚠ **"The one hard constraint: heading" (below) overstates the case.** It is
> true that GPS gives no orientation and that the sim's absolute IMU heading
> must not be used. It is NOT true that nothing works without a heading
> estimator. Measured 2026-09-06: `ekf_map` fusing body-frame velocities
> against absolute GPS positions makes yaw observable **while moving**, and a
> robot started 74° wrong converged to 5–10° over one 20 m leg and reached its
> goal to 0.29 m. Phase 3 is therefore a QUALITY item — it removes a 6.2 m
> opening excursion and bounds an unbounded ~10–20°/min at-rest drift — not a
> prerequisite — but it is not free: in the MAIZE world a 92° start with no
> bootstrap missed the row entrance by 5.9 m, so a straight-drive bootstrap is
> now required there and is on by default. The datum is Purdue ACRE
> (40.494928, −86.996323) but still approximate, read off a map, not surveyed.

## Context

Two things came out of the 2026-07-29/30 field run on the GPU robot:

1. **The row-exit rebuild worked.** The same route that previously produced a
   mid-row false `EXIT_CLEAR` ran clean — no false exits. This closes next-step
   #2 in `HANDOFF3.md` §5. The one remaining defect is small and understood:
   after the exit fires, the robot does not drive far enough forward before
   starting its headland turn, so it clips the corn. `headland_clearance` is the
   knob and it needs raising.

2. **Focus shifts to GPS RTK.** In-row navigation is done and field-proven. The
   missing capability is getting the robot from the trailer to the front of a
   corn row autonomously, then handing off to vision nav. Above the canopy the
   sky is open, so RTK has a clean signal — this is exactly the regime GPS is
   good at, and it is deliberately the *only* place GPS is used. (The lab's own
   P-AgNav paper is explicit that in-row navigation must work *without* GNSS.)

   Assumptions for v1, per the user: flat ground, no obstacles between trailer
   and row. That removes the need for costmaps and obstacle avoidance, which is
   why this plan does **not** use `move_base`.

The mental model the user already has — QGroundControl: see the robot on a
satellite map, click a spot, robot drives there — is the right one. The ROS1
equivalent is `mapviz` + `tile_map` for the map UI, `robot_localization`'s
`navsat_transform_node` for the GPS→metric-frame conversion, and a waypoint
follower for the driving. Nav2/AMCL/gmapping are not applicable: those localize
against a *pre-built occupancy map* from a lidar, which is a different problem
from georeferenced outdoor transit.

### Decisions already made (do not re-litigate)

- **Dual EKF + `navsat_transform_node`**, the standard `robot_localization`
  architecture. Templates are already installed at
  `/opt/ros/noetic/share/robot_localization/{launch,params}/dual_ekf_navsat_example.*`.
- **One Reach M2**, heading from GPS course-over-ground + gyro. A second M2 as a
  dual-antenna compass is a documented fallback, not the v1 design.
- **`headland_clearance` 0.75 → 1.0 m.**
- **All three waypoint UIs** get built and tried: mapviz click, saved YAML file,
  RViz 2D Nav Goal.

### The one hard constraint: heading

Position is easy; **heading is the whole problem.** `navsat_transform_node`
requires an *earth-referenced* yaw to place the robot in the UTM grid.

- A single Reach M2 gives position, not orientation. Emlid state plainly that
  the M2 "doesn't output yaw, pitch, and roll values" and that "moving-base is
  not an officially supported mode for Reach units."
- The Jackal's internal magnetometer exists but Clearpath's own manual says it
  is **not recommended for use** due to its location in the chassis.
- ⚠ **Gazebo hides this.** The hector IMU plugin
  (`jackal/jackal_description/urdf/jackal.gazebo:10-21`) publishes a *fake
  absolute* heading on `/imu/data`. So `navsat_transform_node` will work in sim
  with default settings and then fail on the real robot. **Every sim test in
  this plan must run with `use_odometry_yaw: true`,** i.e. on the same code path
  the real robot uses, so sim success actually transfers.

The v1 answer: heading is carried by the EKF's gyro-integrated yaw (stable over
a 1–2 minute transit), with the constant offset into ENU estimated from GPS
course-over-ground whenever the robot is moving faster than ~0.3 m/s. It is
bootstrapped by a short straight drive at mission start.

Second, independent safety net: the final waypoint before a row is placed *on
the row axis, a few metres out*. Driving that last leg physically forces the
robot to arrive pointing down the row, regardless of heading estimate quality.
Vision nav then closes the remaining alignment in REACQUIRE. **This is what
makes a cheap heading estimate sufficient.**

---

## What already exists (found during exploration — reuse, don't rebuild)

| Fact | Where | Why it matters |
|---|---|---|
| One EKF only, `world_frame: odom`, publishes `/odometry/filtered` + `odom→base_link` TF | `jackal/jackal_control/config/robot_localization.yaml`, started by `control.launch` | Leave it **completely untouched**; `vision_nav_node` depends on `/odometry/filtered` |
| No `navsat_transform_node` anywhere in the workspace | verified by grep across `src/` | Everything GPS is greenfield |
| `robot_localization` + `move_base` installed; `nmea_navsat_driver`, `mapviz`, `geodesy` **not** | `/opt/ros/noetic/share` | apt installs needed |
| Gazebo already publishes `/navsat/fix` (40 Hz) and `/navsat/vel`, **zero consumers** | `jackal.gazebo:25-37`; ref lat/lon from `GAZEBO_WORLD_LAT`/`GAZEBO_WORLD_LON` | The entire stack is developable in the existing maize sim before hardware arrives |
| `navsat_link` already in TF | `jackal_description/urdf/jackal.urdf.xacro:~212-224` | Antenna frame exists; only needs the real offset measured |
| `jackal_navigation` costmaps require a `front/scan` laser the robot doesn't have | `jackal_navigation/params/costmap_common_params.yaml` | Confirms `move_base` is not drop-in; another reason to skip it |
| `JACKAL_CONTROL_EXTRAS` / `_PATH` hook in `control.launch` | `jackal_control/launch/control.launch` | Available injection point, **but not needed** — we add a separate launch file instead |
| `vision_nav_node.py` has no start/stop hook; drives on first frame | `agbot_vision_nav/scripts/vision_nav_node.py:262-267` | A runtime enable must be added for the handoff |
| `headland_clearance`: launch arg `0.75` wins over ctor default `1.0` | `vision_nav.launch:77`, `params.yaml:98`, `mission_fsm.py:121` | Three layers disagree; align them |

---

## Part A — Exit-overshoot fix (small, do first)

`EXIT_CLEAR` ends when `travelled >= exit_clear_min_distance` **and**
`travelled + _exit_clear_offset >= headland_clearance`
(`mission_fsm.py:449-454`), where `_exit_clear_offset` back-dates to where the
exit was first seen. So raising `headland_clearance` directly buys forward
travel before `TURN_1`, which is exactly the observed defect.

Change `0.75 → 1.0` in **all three** places so they finally agree:

- `agbot_vision_nav/launch/vision_nav.launch:77` — the arg default (this is what
  actually runs); update the trailing comment, which already anticipated this
  ("raise toward 1.0 if turns still hit"), to record that the 2026-07-29 field
  run clipped corn at 0.75.
- `agbot_vision_nav/config/params.yaml:98`
- `agbot_vision_nav/src/agbot_vision_nav/mission_fsm.py:121` — already `1.0`;
  leave the value, but this is the one under test.

Add a regression test in `agbot_vision_nav/test/test_mission_fsm.py` beside
`test_exit_clear_back_dates_to_first_sighting` (line 681) and
`test_exit_clear_min_distance_is_always_driven` (line 695), built with the
existing `make_fsm(...)` helper (line 109), asserting the leg length is
`max(exit_clear_min_distance, headland_clearance - offset)` at the new default.

⚠ Note for whoever does this: the existing tests exercise the **ctor** defaults,
not the launch values, so a launch-only change is invisible to the suite. That
is precisely why all three layers get aligned here.

Expected: 107 tests pass. Commit this on its own before starting Part B.

---

## Part B — GPS transit stack

New package `agbot_gps_nav`, following the same architecture as
`agbot_vision_nav`: a **rospy-free algorithmic core** under
`src/agbot_gps_nav/`, unit-testable with plain pytest, and **exactly one file
that touches rospy** under `scripts/`.

### Phase 0 — Sim-first (no hardware needed; start here)

Gazebo is already emitting `/navsat/fix`. Set `GAZEBO_WORLD_LAT` /
`GAZEBO_WORLD_LON` to the **real Purdue field coordinates** so sim and field use
the same numbers throughout, and add a `gps:=true` arg to
`agbot_bringup/launch/agbot_gazebo.launch` that includes the new GPS stack.

This lets Phases 2–6 be written and debugged entirely in sim. Its limits, stated
honestly: hector GPS noise is not RTK-realistic (`drift 0.0001`), and the fake
absolute IMU heading must be bypassed via `use_odometry_yaw: true` or the sim
result is meaningless.

### Phase 1 — Reach M2 → ROS

Hardware/config work, independent of everything above:

1. Mount the antenna with a clear sky view; measure its offset from `base_link`
   and set `JACKAL_NAVSAT_*` env vars (documented in
   `jackal_description/README.md:84-105`) so `navsat_link` is physically correct.
2. In Emlid Flow / ReachView 3, enable **Position output → NMEA**, over USB
   serial (`/dev/ttyACM0`, 115200) or TCP.
3. Corrections: NTRIP from Emlid Caster or the free Indiana CORS network
   (INCORS) if there is cell coverage at the field; otherwise a second Emlid as
   a local base on the trailer. **Decide this before the first field trip** —
   without corrections you get ~2 m accuracy, not 2 cm, and the row-entrance
   waypoint becomes useless.
4. Driver: `nmea_navsat_driver` (apt) for serial, or `nmea_tcp_driver` for TCP.
   Publishes `sensor_msgs/NavSatFix` on `/gps/fix` and `geometry_msgs/TwistStamped`
   on `/gps/vel`. Remap so sim (`/navsat/fix`) and real (`/gps/fix`) present the
   **same topic name** to everything downstream.
5. ✅ Acceptance: `rostopic echo /gps/fix` shows `status.status == 2`
   (`STATUS_GBAS_FIX`, i.e. RTK fixed) and `position_covariance` around
   `0.0004` (2 cm), not 1.0+. Do not proceed until RTK *fix* — not float — holds.

### Phase 2 — Localization: `navsat_transform_node` + map-frame EKF

New: `agbot_gps_nav/launch/gps_localization.launch` and
`agbot_gps_nav/config/{navsat_transform.yaml,ekf_map.yaml}`, adapted from
`/opt/ros/noetic/share/robot_localization/params/dual_ekf_navsat_example.yaml`.

⚠ **Critical: do not restructure the Jackal's existing EKF.** The upstream
`ekf_localization` keeps publishing `/odometry/filtered` and `odom→base_link`
exactly as today, so `vision_nav_node` is unaffected. We only *add*:

- `navsat_transform_node` — inputs `/imu/data`, `/odometry/filtered`,
  `/gps/fix`; outputs `/odometry/gps`. Key params: `use_odometry_yaw: true`
  (**mandatory**, see heading constraint), `two_d_mode: true`,
  `zero_altitude: true`, `publish_filtered_gps: true`, `broadcast_utm_transform: false`,
  `wait_for_datum` / `datum` set to a fixed field origin so the map frame is
  **repeatable across sessions** (otherwise every boot gets a different origin
  and saved waypoints drift).
- A second EKF, `ekf_map`, `world_frame: map`, consuming the same wheel odom +
  IMU plus `/odometry/gps`, publishing on **`/odometry/filtered/global`** (a new
  name — never `/odometry/filtered`) and the `map→odom` TF.

✅ Acceptance: `rosrun tf tf_echo map base_link` tracks the robot; driving a
known 10 m straight line in the field produces ~10 m of `map`-frame motion.

### Phase 3 — Heading estimator

New: `agbot_gps_nav/src/agbot_gps_nav/heading_estimator.py` (pure, no rospy).

Estimates the constant offset `yaw_enu − yaw_odom`:

- Compute course-over-ground from successive `/gps/fix` positions (or directly
  from `/gps/vel`) whenever ground speed > `min_speed_for_course` (~0.3 m/s) and
  the robot is going roughly straight (|angular.z| below a threshold — a turning
  robot's COG lags its heading).
- Maintain the offset with a slow circular-mean filter; hold the last value when
  stopped or slow, so **turn-in-place works once bootstrapped**.
- Expose `is_initialized` and a confidence figure so the follower can refuse to
  commit before heading is trustworthy.

The node republishes `/odometry/filtered` with yaw rotated into ENU on
`/gps_nav/odom_heading_corrected`, and `navsat_transform_node` consumes *that*
with `use_odometry_yaw: true`.

Bootstrap: a `HEADING_INIT` state in the follower drives ~3 m straight at low
speed at mission start. Document it as expected behaviour, not a bug.

Keep the interface narrow enough that a **dual-antenna compass node** (two
independent RTK rovers, heading from the baseline vector — ~1.6° at 1 m
baseline, ~3° at 0.5 m) can replace this module later without touching the
navigation layer.

### Phase 4 — Waypoint follower

New, all rospy-free and unit-tested:

- `src/agbot_gps_nav/geo.py` — WGS84 ↔ local ENU about a datum. Prefer
  `geodesy`/`pyproj` UTM if apt-installable; otherwise a small documented
  equirectangular projection, which is exact enough over a field-sized area.
- `src/agbot_gps_nav/waypoint_follower.py` — pure-pursuit / go-to-goal over a
  state machine: `IDLE → HEADING_INIT → GOTO → APPROACH → ARRIVED → HANDOFF`.
  `GOTO` cruises; `APPROACH` runs the final on-axis leg slowly. Returns
  `(linear_x, angular_z, state)` given a map-frame pose and a goal — mirroring
  the `MissionFSM.update()` signature so the pattern is familiar.
- `scripts/gps_nav_node.py` — **the only rospy file.** Subscribes
  `/odometry/filtered/global`, `/gps/fix`; publishes `/cmd_vel`; exposes an
  action or service to accept goals.

Waypoint spec (YAML): `lat`, `lon`, optional `approach_bearing_deg` and
`approach_distance_m`. When an approach bearing is given (i.e. the row axis),
the follower auto-inserts the on-axis approach point in front of the goal — the
mechanism that makes arrival heading correct without a compass.

⚠ **Publish to `/cmd_vel`, never to the controller/teleop topic.** That keeps
`twist_mux` priority intact so the joystick (priority 9) always outranks
autonomy (priority 1) — the same rule already recorded in `HANDOFF3.md` §3 for
vision nav, and it matters more here since the robot is moving in open ground.

Safety, given the robot drives unattended across open ground:
- fix-quality gate: refuse to move, and stop if already moving, unless the fix
  is RTK fixed/float and fresh (a `max_fix_age_sec` watchdog mirroring the
  vision node's `max_data_age_sec`);
- geofence: a max distance from the datum beyond which it stops — a bad datum or
  a bad fix otherwise drives the robot arbitrarily far;
- heading-confidence gate: no committing to a goal before `is_initialized`;
- conservative speed cap, well under the vision-nav cruise.

### Phase 5 — Waypoint UIs (all three, as requested)

1. **mapviz click** — `sudo apt install ros-noetic-mapviz ros-noetic-mapviz-plugins
   ros-noetic-tile-map`. Config in `agbot_gps_nav/config/mapviz_agbot.mvc`:
   `tile_map` first, then `navsat` (robot fix), then `point_click_publisher`.
   ⚠ Plugin order matters — `tile_map` must be listed **first** or it draws over
   everything else. The click publishes a WGS84 `PointStamped` the node consumes
   as a goal. This is the QGC-equivalent. Satellite imagery beyond OSM needs a
   MapProxy container.
2. **Saved YAML route** — `agbot_gps_nav/config/waypoints_<field>.yaml`, surveyed
   once with the Reach. This is the one that will actually get used day to day:
   trailer → row 1 is the same route every session.
3. **RViz 2D Nav Goal** — subscribe `/move_base_simple/goal` in the map frame.
   Free, no new dependencies, useful in sim.

### Phase 6 — Handoff to vision nav

Two changes:

1. **Runtime enable on the vision node.** Add a `std_srvs/SetBool` service
   (`~set_enabled`) to `agbot_vision_nav/scripts/vision_nav_node.py`, checked in
   the frame-processing path before publishing twist, **defaulting to enabled**
   so every existing launch behaves exactly as today. Disabling must publish a
   zero twist and re-enabling must reset the FSM to a clean `FOLLOW_ROW` with
   fresh entry pose — otherwise the exit detector's odometry arming
   (`min_in_row_distance` 2.0 m) is measured from the wrong origin.
   Note: `git log` shows a removed `operator_override` module (orphan `.pyc`
   files remain in `src/agbot_vision_nav/__pycache__/`); check that history
   before designing the gate, and delete the stale `.pyc` files.
2. **Supervisor** — `agbot_gps_nav/scripts/mission_supervisor.py` sequences:
   GPS nav disabled-vision → drive route → `ARRIVED` at the row entrance →
   enable vision nav (`mission_enabled:=true`) → wait for `Mission DONE` →
   optionally re-enable GPS nav for the return leg to the trailer. This also
   finally gives "go home" a home, which earlier sessions deferred.

---

## Files

**New package `agbot_gps_nav/`** (`package.xml`, `CMakeLists.txt` mirroring
`agbot_vision_nav`):

```
src/agbot_gps_nav/geo.py                 WGS84 <-> local ENU (pure)
src/agbot_gps_nav/heading_estimator.py   COG + gyro yaw offset (pure)
src/agbot_gps_nav/waypoint_follower.py   go-to-goal FSM (pure)
scripts/gps_nav_node.py                  only rospy file
scripts/mission_supervisor.py            GPS <-> vision handoff
config/navsat_transform.yaml
config/ekf_map.yaml
config/mapviz_agbot.mvc
config/waypoints_<field>.yaml
launch/gps_localization.launch           navsat_transform + ekf_map
launch/gps_nav.launch                    driver + localization + follower
test/test_geo.py  test_heading_estimator.py  test_waypoint_follower.py
```

**Modified:**

- `agbot_vision_nav/launch/vision_nav.launch:77` — `headland_clearance` 0.75 → 1.0
- `agbot_vision_nav/config/params.yaml:98` — same
- `agbot_vision_nav/test/test_mission_fsm.py` — new regression test
- `agbot_vision_nav/scripts/vision_nav_node.py` — `~set_enabled` service
- `agbot_bringup/launch/agbot_gazebo.launch` — `gps:=true` arg
- `CLAUDE.md`, `HANDOFF3.md` — record the field result and the new subsystem

⚠ `agbot_bringup/urdf/agbot_camera.urdf.xacro` carries the user's uncommitted
working-tree diff. **Do not touch or commit it.**

---

## Verification

**Part A** — `cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v`
(expect 107 passed). Then a sim mission
(`mission_enabled:=true num_rows:=3`) confirming the post-exit leg is visibly
longer before `TURN_1`. Field-confirm on the next GPU-robot run: the turn should
start ~1.0 m past first sighting and no longer clip corn.

**Part B, per phase:**

| Phase | Check |
|---|---|
| 0 | `rostopic echo /navsat/fix` in the maize sim; lat/lon match `GAZEBO_WORLD_LAT/LON` |
| 1 | `/gps/fix` shows `status == 2` (RTK fixed) and covariance ~0.0004 |
| 2 | `rosrun tf tf_echo map base_link` tracks; 10 m driven ≈ 10 m in map frame |
| 3 | Drive a known compass bearing; the estimated ENU yaw matches within a few degrees. **Repeat in sim with the fake IMU heading explicitly ignored.** |
| 4 | pytest on the pure modules; then sim: publish a goal 20 m away, robot arrives within tolerance and stops. Test the safety gates by publishing a stale/bad-quality fix and confirming it stops. |
| 5 | mapviz shows the robot on tiles; a click drives it there. Same goal via YAML and via RViz. |
| 6 | Sim end-to-end: robot starts away from the rows → drives to the row-entrance waypoint → vision nav enables → 3-row mission → `Mission DONE` |

**First field test should be in an open lot, not the cornfield** — a heading-sign
error sends the robot in exactly the wrong direction, and the geofence is the
only thing that stops it.

---

## Resources to look into

**Core reading (highest value first)**
- `robot_localization` — *Integrating GPS Data*:
  http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html
  and `navsat_transform_node`:
  http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
- The templates already on disk:
  `/opt/ros/noetic/share/robot_localization/launch/dual_ekf_navsat_example.launch`
  and `params/dual_ekf_navsat_example.yaml` — read these before writing any config.
- `nickcharron/waypoint_nav` — a complete ROS1 outdoor GPS waypoint system
  (move_base + robot_localization + waypoint files):
  https://github.com/nickcharron/waypoint_nav — closest existing analogue to
  what you're building; worth reading even though we're skipping move_base.
- Nav2's GPS localization tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
  — ROS2, but the *concepts* (datum, dual EKF, mapviz click-to-goal) map 1:1 and
  it is the best-written explanation of the architecture.

**Emlid / hardware**
- Emlid Reach docs (position output, NTRIP, base/rover): https://docs.emlid.com
- `ros-drivers/nmea_navsat_driver`: https://github.com/ros-drivers/nmea_navsat_driver
- `CearLab/nmea_tcp_driver` (TCP variant, built specifically for Reach RTK):
  https://github.com/CearLab/nmea_tcp_driver
- `AnthonyZJiang/emlid_reach_ros` — Reach-specific driver publishing NavSatFix,
  TimeReference, Twist: https://github.com/AnthonyZJiang/emlid_reach_ros
- The two threads that settle the heading question:
  https://community.emlid.com/t/emlid-reach-m2-yaw-pitch-roll-angles/30797 and
  https://community.emlid.com/t/car-heading-in-moving-base-mode/8770

**Map UI**
- mapviz: http://wiki.ros.org/mapviz — and
  https://roboticsknowledgebase.com/wiki/tools/mapviz/
- Satellite tiles via MapProxy:
  https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite
- Known gotcha thread (plugin draw order): https://answers.ros.org/question/390156/

**Context**
- Clearpath OutdoorNav hardware manual — how Clearpath themselves do RTK on a
  Jackal, including base-station/TCP correction plumbing:
  https://www.clearpathrobotics.com/assets/manuals/outdoornav/cpr_hardware.html
- `Papers/4_P-AgNav_...md` — the lab's own position that in-row navigation must
  not depend on GNSS. Worth citing in whatever you write up.

---

## Suggested commit boundaries

Per `CLAUDE.md`, commit and push at each of these — do not batch:

1. Part A (`headland_clearance` + test)
2. `agbot_gps_nav` skeleton + `geo.py` + tests
3. `heading_estimator.py` + tests
4. `gps_localization.launch` + configs
5. `waypoint_follower.py` + tests
6. `gps_nav_node.py`
7. mapviz/RViz/YAML goal sources
8. `vision_nav_node.py` `~set_enabled` + supervisor
9. Docs (`CLAUDE.md`, `HANDOFF3.md`)
