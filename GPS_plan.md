# GPS module — state, how to run it, and what is left

> **What this file is.** It began as a plan and is now the working reference for
> `agbot_gps_nav`. The transit-and-handoff capability is BUILT and runs end to
> end in simulation; the hardware half is not started. Read §1 to run it, §3 for
> the findings that will not be obvious from the code, and §5 for what is left.
>
> Companion reading: `HANDOFF3.md` §0g (first GPS increment, the heading
> measurement) and §0h (the handoff, and the five defects integration exposed).
> `CLAUDE.md` has the condensed version of both.
>
> Last updated end of session **2026-09-07**.

---

## 0. Status at a glance

| | |
|---|---|
| **Works today, in simulation** | trailer → GPS transit → arrive on the row axis → hand off → vision nav drives 3 corridors → DONE |
| **Never run on the robot** | no Reach M2 yet; the whole GPS stack is sim-only |
| **Datum** | Purdue ACRE `40.494928, −86.996323` — approximate, **read off a map, not surveyed** |
| **Tests** | `agbot_gps_nav` 149, `agbot_vision_nav` 240, all passing |
| **Phases done** | 0 (sim-first), 2 (dual EKF), 4 (follower + node), 5 (RViz + mapviz configs), 6 (handoff) |
| **Phase 3** | *bootstrap* half built and required; full course-over-ground estimator NOT built |
| **Phase 1** | not started (Reach M2, NTRIP) |
| **Part A** | not done — `headland_clearance` 0.75 → 1.0, unrelated vision-nav fix, still open |

---

## 1. HOW TO RUN IT

Everything below runs on the dev laptop. ⚠ **This machine IS the ROS1 Noetic
box** — Ubuntu 20.04, `/opt/ros/noetic`, WSLg display. Older notes claiming the
sandbox has no ROS1 are wrong.

### 1.0 Once, after pulling

```bash
cd ~/agbot_control_ws
catkin build            # agbot_gps_nav is a new package
source devel/setup.bash # in EVERY terminal you use below
```

Offline tests, no ROS or Gazebo needed:

```bash
cd ~/agbot_control_ws/src/agbot_gps_nav   && PYTHONPATH=src python3 -m pytest test/ -q   # 149
cd ~/agbot_control_ws/src/agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -q   # 240
```

### 1.1 The headline demo — trailer to row, then the row mission

**This is the one to run first.** Three terminals, each with `source
devel/setup.bash`.

```bash
# terminal 1 — the world. Gazebo goes in its OWN terminal: its output is noisy
# enough to bury both nodes' startup config blocks, which are the part worth
# reading. Drop gui:=false to watch it.
rosrun agbot_bringup switch_maize_world.sh gps
roslaunch agbot_bringup agbot_gazebo.launch gps:=true gui:=false \
  x:=-0.798 y:=-21.361 z:=0.35 yaw:=1.5708

# terminal 2 — the sequence
roslaunch agbot_gps_nav gps_vision_mission.launch sim:=true num_rows:=3

# terminal 3 — watch it
rostopic echo /mission_supervisor/status
```

Expect, over roughly 6 minutes of wall time:

```
TRANSIT       robot drives ~18 m from the trailer, arrives at the corridor 0
              entrance pointing down the row
ROW_MISSION   vision nav takes over, drives corridor 0 north, 1 south, 2 north
FINISHED      rows=3/3, state=DONE
```

`rostopic echo /vision_nav_node/status` shows `rows=N/3` climbing. The debug
overlay is `rqt_image_view /vision_nav_node/debug/image`.

⚠ **The spawn pose is not the launch default.** `x:=-0.798 y:=-21.361` is the
simulated trailer, 18 m south of the rows; it comes from `start_pose` in
`agbot_gps_nav/config/waypoints_maize_gps.yaml`. Launching without it puts the
robot at the small-world spawn, inside the field, and the transit is nonsense.

### 1.2 The blank world — fastest way to see GPS driving on its own

No maize, no camera, no segmentation model, loads in seconds at RTF 1.0. Use
this to check anything about the follower itself.

```bash
# terminal 1
roslaunch agbot_bringup agbot_gps_sim.launch rviz:=true

# terminal 2
roslaunch agbot_gps_nav gps_nav.launch sim:=true
```

Then give it a goal, three ways:

```bash
# a) RViz: press "2D Nav Goal" and click.
#    ⚠ RViz's Fixed Frame MUST be map. A goal in any other frame is REFUSED,
#    with a log line naming the frame it got. The bundled rviz:=true config is
#    already set to map.

# b) a latitude/longitude. ⚠ x is LONGITUDE, y is latitude -- mapviz's
#    convention, and the reverse of how the pair is spoken.
rostopic pub -1 /gps_nav_node/goal_wgs84 geometry_msgs/PointStamped \
  '{header: {frame_id: wgs84}, point: {x: -86.99620, y: 40.49500}}'

# c) a goal WITH an approach bearing (arrive pointing a particular way).
#    ~goal_pose always reads its orientation as the approach bearing.
rostopic pub -1 /gps_nav_node/goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 12.0, y: 6.0},
    orientation: {z: 0.7071, w: 0.7071}}}'   # z,w = yaw 90 deg = arrive facing north
```

Watch with `rostopic echo /gps_nav_node/status`.

### 1.3 mapviz — the QGroundControl-style map view

Installed but **never opened**; this is the one untried piece of Phase 5.

```bash
rosrun mapviz mapviz
# File > Open Config > ~/agbot_control_ws/src/agbot_gps_nav/config/mapviz_agbot.mvc
```

It should show OSM tiles over ACRE with the robot on them; clicking publishes to
`/gps_nav_node/goal_wgs84`, the same topic as 1.2(b). ⚠ `tile_map` is listed
first in the config on purpose — mapviz paints plugins in list order and a tile
layer listed later covers the robot.

### 1.4 Regenerating the world, if the snapshot is ever lost

`~/.ros/virtual_maize_field_snapshots/` is outside git. Snapshots present today:
`full`, `small`, `long`, `gps`.

```bash
rosrun agbot_bringup generate_maize_world.sh agbot_maize_gps gps
# then ALWAYS regenerate the waypoints, because the layout is seeded and
# procedural -- every plant moves a few cm and hand-copied numbers go stale:
cd ~/agbot_control_ws/src/agbot_gps_nav
python3 scripts/rows_to_waypoints.py ~/.ros/virtual_maize_field/gt_map.csv \
  --datum-file config/gps_datum.yaml -o config/waypoints_maize_gps.yaml \
  --start-offset 18.0
```

The generator PRINTS a spawn pose; that is the pose for driving the rows by
hand, not the trailer pose. The trailer pose is `start_pose` in the waypoints
file.

### 1.5 Useful checks when something looks wrong

```bash
rostopic echo -n1 /navsat/fix                     # lat/lon should match the datum
rosrun tf tf_echo map base_link                   # map frame tracking the robot
rostopic echo /gps_nav_node/status                # GOTO / ALIGN / APPROACH / ARRIVED / FAILED
rostopic echo /vision_nav_node/status             # running|disabled, state, rows
rostopic hz /cmd_vel                              # ~20 Hz GPS, ~2 Hz vision
roslaunch --dump-params agbot_gps_nav gps_nav.launch | grep <knob>
```

Both nodes log a full resolved-config block at startup, right before
`... ready`. That block is the fastest way to confirm a knob actually took.

---

## 2. WHAT EXISTS

### Architecture

```
/navsat/fix (sim) or /gps/fix (real) ──┐
/imu/data ─────────────────────────────┤
                                       ├─> navsat_transform ─> /odometry/gps
/odometry/filtered/global ─────────────┘      use_odometry_yaw: true
                                                       │
/jackal_velocity_controller/odom ──┐                   │
/imu/data (NO absolute yaw) ───────┼─> ekf_map ────────┘
/odometry/gps ─────────────────────┘   /odometry/filtered/global + map->odom TF
                                                       │
   waypoint ──> gps_nav_node ──┐                       │
                               ├─> /cmd_vel <──────────┴── vision_nav_node
   mission_supervisor ─────────┘   (exactly ONE enabled at a time)
```

⚠ **The Jackal's own EKF is untouched.** It keeps publishing
`/odometry/filtered` and owning `odom→base_link`, which `vision_nav_node`
depends on. `ekf_map` is purely additive: different world frame, different
output topic, different TF edge. Never publish to `/odometry/filtered`.

### Files

```
agbot_gps_nav/
  src/agbot_gps_nav/geo.py               WGS84 <-> local ENU about a datum (pure)
  src/agbot_gps_nav/waypoint_follower.py IDLE > HEADING_INIT > GOTO > ALIGN >
                                         APPROACH > ARRIVED / FAILED (pure)
  src/agbot_gps_nav/handoff_fsm.py       TRANSIT > ROW_MISSION > FINISHED (pure)
  scripts/gps_nav_node.py                rospy: the follower
  scripts/mission_supervisor.py          rospy: the handoff
  scripts/rows_to_waypoints.py           dev tool: world -> waypoints
  config/gps_datum.yaml                  THE datum, one definition
  config/params.yaml                     all follower tuning
  config/navsat_transform.yaml  config/ekf_map.yaml
  config/waypoints_maize_gps.yaml        GENERATED, do not hand-edit
  config/mapviz_agbot.mvc                rviz/gps_nav.rviz
  launch/gps_localization.launch         navsat_transform + ekf_map
  launch/gps_nav.launch                  localization + follower
  launch/gps_vision_mission.launch       the whole sequence
  test/                                  149 tests, no ROS needed

agbot_bringup/
  config/agbot_maize_gps.yaml            maize world with a 20 m headland
  launch/agbot_gps_sim.launch            blank world + GPS
  launch/agbot_gazebo.launch             gained gps:=true and the datum
  scripts/load_robot_description.sh      gained optional datum arg

agbot_vision_nav/
  scripts/vision_nav_node.py             gained ~set_enabled, ~mission_done
  src/agbot_vision_nav/mission_fsm.py    gained reset()
```

`geo.py` uses a local tangent plane with the WGS84 meridional **and**
prime-vertical radii at the datum latitude (they differ 0.39 % at Purdue; using
one for both is a real bug a round-trip test cannot see). No pyproj or geodesy —
neither is installed and neither is needed. Measured against Vincenty: 1.5 mm at
280 m, 3.7 cm at 1.4 km, 0.94 m at 7 km — so the answer to a bigger site is a
**nearer datum**, not a better projection.

---

## 3. WHAT WAS MEASURED — read this before changing anything

These are the things that will not be obvious from reading the code, and each
one cost a failed run to find.

### 3.1 Heading is recovered by MOTION, not by magic

GPS gives position, never orientation. The Jackal has no usable compass
(Clearpath say so), a single Reach M2 outputs no yaw, and the stock EKF
deliberately does not fuse absolute yaw — so map-frame yaw is gyro-integrated
from zero at boot and is wrong by however far the robot happens to be pointing.

⚠ Gazebo's hector IMU publishes a **fake absolute heading**. Consuming it makes
every sim pass and every field run fail. `use_odometry_yaw: true` is mandatory
and `imu0_config` leaves absolute yaw `false`, both on purpose.

Measured against Gazebo ground truth:

| condition | heading error |
|---|---|
| spawned 74° wrong, at rest | 74° |
| after ONE 20 m driven leg | **5–10°** |
| stationary, any start | **drifts 10–20°/min, unbounded** |

`ekf_map` fuses body-frame wheel velocities against absolute GPS positions, so
**yaw is observable whenever the robot moves** — that is course-over-ground
estimation arriving out of the dual-EKF structure rather than from a module.

But it is not free. In the maize world a 92° start with no bootstrap **missed
the row entrance by 5.9 m**. So `heading_init_distance` (the bootstrap) is
required there and is on by default in `gps_vision_mission.launch`.

⚠ **The bootstrap ends on a MEASUREMENT, not a distance.** A fixed 4 m was tried
and was not enough. It now drives straight until the course actually driven
agrees with the yaw estimate, because how far that takes depends on how wrong
the estimate started, which nothing knows in advance. It deliberately does NOT
steer — steering on the estimate is the circularity being broken. It needs clear
ground ahead, which is what the 20 m headland is for.

### 3.2 Arrival on a directed approach is crossing a PLANE

⚠ Not entering a radius. A radius fails in a way that looks like nothing: the
robot passed **0.46 m to the side** of the row entrance, Euclidean distance
bottomed out at 0.47 m, never reached the 0.30 m tolerance, and the follower
drove calmly on up the field reporting a growing "distance to goal". Crossing a
plane is a thing that definitely happens; entering a radius is not.

Crossing further off-axis than `arrival_cross_tolerance` (0.35 m, under half a
row spacing so "arrived" cannot mean the next corridor) re-stages and retries,
up to `max_approach_attempts`, then fails cleanly.

### 3.3 The final leg tracks the AXIS, not the goal point

Steering at the point corrects lateral offset in the last metre, so the robot
swings as it arrives: measured **41.2° off** a bearing it had aligned to
correctly moments earlier. Tracking the line makes the commanded heading equal
the bearing once the robot is on it. Same run after the fix: **0.4°**.

### 3.4 A receding goal is a failure nothing else notices

One run drove **100 m away from a goal 6.5 m distant**, off the heightmap, and
was still falling minutes later at z = −464 km. The geofence is measured from
the **datum**, so 100 m sat well inside it. `max_goal_distance_growth` now
aborts when the goal recedes past the closest approach — *closest*, not starting
distance, because getting within 2 m and wandering 8 m off is a failure too.

### 3.5 Disabled means SILENT, not zeros

Both nodes publish `/cmd_vel`, which twist_mux takes as **one** input at
priority 1 and arbitrates by **priority, not rate** — so two publishers on it
are not arbitrated at all, they interleave, and zeros from a "stopped" node
shred the active node's commands.

- `~set_enabled` false → one zero, then **silence**. Handing over.
- `~pause` true → keeps publishing zeros. Still in charge, holds the robot
  immediately rather than waiting out the mux's 0.5 s timeout.

Measured on the GPS node: driving 19.8 Hz all non-zero, paused 19.8 Hz all zero,
disabled **0.0 Hz**. ⚠ Opposite polarity too: `~pause` true = stop,
`~set_enabled` true = go.

Enabling vision nav **resets** the mission (`MissionFSM.reset()`), it does not
resume it: after a transit under another controller the row-entry pose is wrong,
not stale, and `min_in_row_distance` arms off it.

### 3.6 `first_turn_direction` must match the corridor

Rows are ordered by increasing x and the robot enters along the bearing, so out
of the **leftmost** corridor the next one is to its **RIGHT**. Turning left out
of `corridor_0` drove straight out of the field and ended the mission at
`rows=1/3` in open ground. `rows_to_waypoints.py` emits the direction that
belongs with each corridor.

Also: without `rear_camera_enabled` a blocked-ahead signal **ends the mission**
(the BACKOUT states are unreachable), which in a 3-row run reads as a mission
that quietly finished at `rows=1/3`. It is on by default in the mission launch.

### 3.7 Things that are NOT bugs — do not chase them

- **`Transform from ... unavailable for the time requested. Using latest
  instead`**, ~1/s from `ekf_map` and `navsat_transform`. Throttled; the
  fallback is at most one TF period stale (~8 mm at 0.4 m/s) and measured
  position agreement with ground truth is **2 cm**. `navsat_transform` has **no
  `transform_timeout` parameter at all**, and upstream's own
  `dual_ekf_navsat_example.yaml` ships `0.0` on both EKFs.
- **`World frame->cartesian transform is Origin: (-501151.9, ...)`** is an
  internal cartesian origin, not a datum error. Verified: robot at map
  `(0.003, 0.000)` reports exactly the datum lat/lon.
- ⚠ **Do not teleport the robot with `/gazebo/set_model_state`** and expect the
  map-frame heading to follow. The EKF's yaw is dead-reckoned and a teleport is
  invisible to it, so the estimate keeps the old heading and the robot drives
  off in that direction. Two confusing runs came from exactly this. **Restart
  the sim at the pose you want instead.**
- The first ~15 s after launch, `ekf_map` is still converging from a zero
  initialisation and `navsat_transform` has a 3 s `delay`. Sampling the map pose
  before that shows `(0,0)` and looks like a datum bug. Wait, then look.

---

## 4. THE KNOBS THAT MATTER

All in `agbot_gps_nav/config/params.yaml`, which is the source of truth. Launch
`<arg>`s default to EMPTY and their `<param>` tags are conditional, so a launch
argument overrides the file only when actually passed.

| knob | default | why you would touch it |
|---|---|---|
| `linear_x_cruise` | 0.4 | transit speed; open ground, so faster than in-row |
| `approach_speed` | 0.15 | speed at the goal. ⚠ a FLOOR, not a second derate — it used to be multiplied again and the last metre crawled at 3.75 cm/s |
| `staging_distance` | 3.0 (5.0 in the mission launch) | length of the on-axis final leg; longer gives the cross-track term more room |
| `arrival_cross_tolerance` | 0.35 | must stay under half a row spacing |
| `heading_init_distance` / `_max_distance` | 0.0 / 8.0 | bootstrap min and backstop. ⚠ **0 disables it — fine in a blank world, NOT on the robot** |
| `max_goal_distance_growth` | 5.0 | receding-goal abort |
| `geofence_radius_m` | 200.0 | max distance from the datum |
| `min_fix_status` | 0 | ⚠ **0 accepts a non-RTK fix.** Correct for sim (hector reports 0 and cannot report anything else). **SET TO 2 BEFORE ANY FIELD RUN** |
| `use_goal_orientation` | false | make RViz 2D Nav Goal's drag direction the approach bearing |

---

## 5. WHAT IS LEFT

### 5.1 Replace the datum with a surveyed fix — blocks all field work

`agbot_gps_nav/config/gps_datum.yaml` holds ACRE at `40.494928, −86.996323`,
read off a map. At RTK's 2 cm a map-read origin is off by orders of magnitude
more than the fix it anchors. **Every stored waypoint is relative to it**, so
moving the datum moves them all — re-record anything captured beforehand.

### 5.2 Phase 1 — Reach M2 → ROS (not started)

1. Mount the antenna with clear sky view; measure its offset from `base_link`
   and set `JACKAL_NAVSAT_*` (see `jackal_description/README.md:84-105`) so
   `navsat_link` is physically correct. ⚠ Note the Gazebo GPS plugin is
   hardcoded to `bodyName=navsat_link`, and enabling the `JACKAL_NAVSAT`
   accessory does **not** move that link — it creates a separate `rear_navsat`
   chain.
2. Emlid Flow / ReachView 3: **Position output → NMEA**, over USB serial
   (`/dev/ttyACM0`, 115200) or TCP.
3. Corrections: NTRIP from Emlid Caster or the free Indiana CORS network
   (INCORS) if there is cell coverage at the field; otherwise a second Emlid as
   a local base on the trailer. **Decide before the first field trip** — without
   corrections you get ~2 m, not 2 cm, and the row-entrance waypoint is useless.
4. Driver: `nmea_navsat_driver` (apt, **not installed**) for serial, or
   `nmea_tcp_driver` for TCP. Publishes `sensor_msgs/NavSatFix` on `/gps/fix`.
   The launch already computes `/navsat/fix` vs `/gps/fix` from `sim:=`.
5. ✅ Acceptance: `rostopic echo /gps/fix` shows `status.status == 2`
   (`STATUS_GBAS_FIX`, RTK **fixed**, not float) and `position_covariance`
   around `0.0004`. Then set `min_fix_status: 2`.

### 5.3 Phase 3 — the full heading estimator (bootstrap only, so far)

`heading_estimator.py` was never written. What exists is the bootstrap: drive
straight until the course driven agrees with the yaw estimate. That fixes the
START of a run. It does **not** fix the unbounded ~10–20°/min at-rest drift, so
a robot parked for several minutes mid-mission still starts its next leg on a
rotten heading.

The design, unchanged: estimate the constant offset `yaw_enu − yaw_odom` from
course-over-ground whenever ground speed > ~0.3 m/s and `|angular.z|` is small
(a turning robot's COG lags its heading); hold the last value when stopped, so
turn-in-place works once bootstrapped; expose `is_initialized` and a confidence
so the follower can refuse to commit.

⚠ **The seam is one remap.** `gps_localization.launch` points
`navsat_transform`'s `odometry/filtered` at `/odometry/filtered/global`; the
estimator inserts itself by republishing a yaw-corrected odometry and changing
that one line. Nothing else moves. Keep the interface narrow enough that a
dual-antenna compass (two RTK rovers, heading from the baseline vector — ~1.6°
at 1 m) could replace it later.

⚠ **Do not "fix" the at-rest drift by fusing the IMU's absolute yaw.** That is
the sim-only signal from §3.1.

### 5.4 Part A — `headland_clearance` 0.75 → 1.0 (unrelated, still open)

From the 2026-07-29 field run: after the exit fires the robot does not drive far
enough before starting its headland turn, so it clips corn. `EXIT_CLEAR` ends on
`travelled + _exit_clear_offset >= headland_clearance`, so raising it directly
buys forward travel before `TURN_1`.

Change in **all three** places, which currently disagree:
`agbot_vision_nav/launch/vision_nav.launch` (the arg default, and what actually
runs), `config/params.yaml`, and `src/agbot_vision_nav/mission_fsm.py` (already
1.0). ⚠ Existing tests exercise the **ctor** defaults, so a launch-only change
is invisible to the suite — which is why all three get aligned. Add a regression
test beside `test_exit_clear_back_dates_to_first_sighting`.

### 5.5 Smaller open items

- **mapviz has never been opened** (§1.3). Config written, packages installed.
- **The return leg** (row → trailer) is not built. The supervisor stops at
  `FINISHED`; "go home" still has no home.
- **Only `corridor_0` has been driven end to end.** `waypoint:=corridor_1` /
  `corridor_2` exist and should work — pass the matching `first_turn_direction`
  from the waypoints file.
- **Whether GPS-driven metres should count** toward the autonomy metric in
  `metrics_logger` is undecided. Today the GPS leg is not logged at all.
- **Multi-waypoint routes** (a saved YAML route walked in sequence) are not
  built; the supervisor takes exactly one waypoint.

---

## 6. Design decisions — settled, do not re-litigate

- **Dual EKF + `navsat_transform_node`**, the standard `robot_localization`
  architecture. Templates on disk at
  `/opt/ros/noetic/share/robot_localization/{launch,params}/dual_ekf_navsat_example.*`.
  ⚠ Three of that template's values are wrong for us and the reasons are in our
  config files: `use_odometry_yaw` true (not false), `yaw_offset` 0 (not π/2 —
  that assumes an IMU reading zero at magnetic north), `magnetic_declination` 0
  (the template's is Edinburgh's).
- **No `move_base`.** v1 assumes flat ground with no obstacles between trailer
  and row; its costmaps want a laser this robot does not have
  (`jackal_navigation/params/costmap_common_params.yaml` requires `front/scan`),
  and a global planner buys nothing when the correct path is a straight line.
  Revisit only if obstacle avoidance is actually needed.
- **GPS is used for the transit and nowhere else.** The lab's own P-AgNav paper
  is explicit that in-row navigation must work without GNSS.
- **Publish to `/cmd_vel`**, never the controller/teleop topic, so twist_mux
  keeps the joystick (priority 9-10) above autonomy (priority 1).
- **One Reach M2**, heading from course-over-ground. A second M2 as a
  dual-antenna compass is a documented fallback, not the v1 design.
- **The final waypoint sits on the row axis, a few metres out**, so driving that
  last leg physically forces the arrival heading. This is what makes a cheap
  heading estimate sufficient — and §3.2/§3.3 are what make it actually work.

---

## 7. Resources

**Core reading**
- `robot_localization` — *Integrating GPS Data*:
  http://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html
  and `navsat_transform_node`:
  http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
- Templates on disk:
  `/opt/ros/noetic/share/robot_localization/launch/dual_ekf_navsat_example.launch`
  and `params/dual_ekf_navsat_example.yaml`.
- `nickcharron/waypoint_nav` — a complete ROS1 outdoor GPS waypoint system:
  https://github.com/nickcharron/waypoint_nav — closest existing analogue, worth
  reading even though we skip move_base.
- Nav2's GPS localization tutorial:
  https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html — ROS2, but the
  concepts (datum, dual EKF, mapviz click-to-goal) map 1:1 and it is the
  best-written explanation of the architecture.

**Emlid / hardware**
- Emlid Reach docs: https://docs.emlid.com
- `ros-drivers/nmea_navsat_driver`: https://github.com/ros-drivers/nmea_navsat_driver
- `CearLab/nmea_tcp_driver` (built for Reach RTK):
  https://github.com/CearLab/nmea_tcp_driver
- `AnthonyZJiang/emlid_reach_ros`: https://github.com/AnthonyZJiang/emlid_reach_ros
- The two threads that settle the heading question:
  https://community.emlid.com/t/emlid-reach-m2-yaw-pitch-roll-angles/30797 and
  https://community.emlid.com/t/car-heading-in-moving-base-mode/8770

**Map UI**
- mapviz: http://wiki.ros.org/mapviz — and
  https://roboticsknowledgebase.com/wiki/tools/mapviz/
- Satellite tiles via MapProxy:
  https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite
- Plugin draw-order gotcha: https://answers.ros.org/question/390156/

**Context**
- Clearpath OutdoorNav hardware manual — how Clearpath do RTK on a Jackal:
  https://www.clearpathrobotics.com/assets/manuals/outdoornav/cpr_hardware.html
- `Papers/4_P-AgNav_...md` — the lab's position that in-row navigation must not
  depend on GNSS.
