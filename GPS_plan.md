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
> Last updated end of session **2026-09-07 (second session, evening)**. That
> session found and fixed the reason the transit had been unreliable — §3.1's
> process-noise starvation — and added the two guards that would have caught it:
> the handoff heading gate (§3.5b) and goal ownership (§3.7). §1.3 explains why
> mapviz used to draw nothing, and why clicking it used to hijack a run.

---

## 0. Status at a glance

| | |
|---|---|
| **Works today, in simulation** | trailer → GPS transit → arrive on the row axis → hand off → vision nav drives 3 corridors → DONE |
| **Never run on the robot** | no Reach M2 yet; the whole GPS stack is sim-only |
| **Datum** | Purdue ACRE `40.494928, −86.996323` — approximate, **read off a map, not surveyed** |
| **Tests** | `agbot_gps_nav` 161, `agbot_vision_nav` 243, all passing |
| **Phases done** | 0 (sim-first), 2 (dual EKF), 4 (follower + node), 5 (RViz + mapviz, both now actually working), 6 (handoff) |
| **Phase 3** | *bootstrap* half built and required; full course-over-ground estimator NOT built |
| **Phase 1** | not started (Reach M2, NTRIP) |
| **Part A** | not done — `headland_clearance` 0.75 → 1.0, unrelated vision-nav fix, still open |
| **Guards that stop a bad run** | heading bootstrap must converge or it logs a red error (§3.1); the handoff is refused if the arrival heading is >12° off (§3.5b); interactive goals are locked out during a mission (§3.7) |
| **Known cost** | every open GUI eats real-time factor and shows up as `WATCHDOG_ZERO`; see §1.1 |

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
cd ~/agbot_control_ws/src/agbot_gps_nav   && PYTHONPATH=src python3 -m pytest test/ -q   # 161
cd ~/agbot_control_ws/src/agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -q   # 243
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

Measured on the 2026-09-07 21:06 run, after the heading fixes (§3.1, §3.5b),
**with the Gazebo GUI on**:

```
bootstrap converged at 2.7 deg      (the run before it gave up at 71.9 deg)
ARRIVED 0.30 m from goal, arrival heading check -0.5 deg -> handoff accepted
rows 3/3, DONE.  42.6 m, 0 interventions, 0 BLOCKED, 0 BACKOUT
FOLLOW_ROW rms offset_norm 0.095, invalid frames 0.0%
```

And the 22:20 run the same evening, **headless**, driven by the operator:

```
bootstrap converged 1.9 deg, then -2.1 deg on the real goal
ARRIVED 0.30 m from goal, arrival heading check +0.9 deg -> handoff accepted
Mission DONE: rows_driven=3, blocked rows: none, revoked exits: 0
44.0 m, 0 interventions.  FOLLOW_ROW rms 0.091, invalid frames 0.0%
```

### `WATCHDOG_ZERO` — it is the MPC, not the GUI

⚠ **Corrected 2026-09-07 late.** The first reading of this was "every open GUI
costs real-time factor, so close them". That is not what the data says, and
acting on it wastes time. Three passing runs the same evening:

| run | on screen | `WATCHDOG_ZERO` | of which in `FOLLOW_ROW` | frames over 0.5 s |
|---|---|---|---|---|
| 21:06 | Gazebo GUI + RViz | 190 | 169 (89 %) | 318 / 801 |
| 22:20 | RViz + mapviz | 97 | 91 (94 %) | 235 / 850 |
| 23:46 | RViz only | **123** | 113 (92 %) | 294 / 735 |

Closing mapviz made it **worse**, not better (97 → 123). The Gazebo GUI does
cost something real — 190 is clearly the outlier — but run-to-run variance
swamps the rest, and **~90 % of every trip lands in `FOLLOW_ROW`**. That is the
tell.

Latency split by mission state, mean ms, consistent across both late runs:

| state | inference | end-to-end | **the difference** |
|---|---|---|---|
| `FOLLOW_ROW` | 205 | 522 | **317** |
| `EXIT_CLEAR` | 191 | 417 | 225 |
| `TURN_1` | 182 | 280 | **99** |
| `TRAVERSE` | 205 | 309 | 104 |

Inference is flat at ~200 ms everywhere. The non-inference part of the loop is
~100 ms in the odometry-open-loop states and **~300 ms in `FOLLOW_ROW`** — and
`FOLLOW_ROW` is the only state that runs the MPC. So the SLSQP solve
(`mpc_horizon` 8) costs roughly **200 ms per cycle**, which puts the row-following
cycle at ~520 ms against a `max_data_age_sec` of **500 ms**. It is sitting
exactly on the threshold, so roughly every other cycle is late. Nothing on
screen changes that.

⚠ **Do not raise `max_data_age_sec` to make the count go away** — the watchdog
is correctly reporting that the loop cannot keep up, and at 0.5 m/s that window
would be half a metre of blind driving. In sim the sanctioned answer is
`rosrun agbot_bringup set_sim_rtf.sh 0.5`: `use_sim_time` is on, so slowing
wall-clock time halves the *sim-time* cycle to ~260 ms and leaves every
threshold self-consistent. On the robot the answers are a faster machine or a
shorter MPC horizon.

**It is not currently hurting anything.** At 0.15 m/s these runs still returned
the best tracking of the four (`FOLLOW_ROW` rms 0.082) with 0 interventions and
0 % invalid frames — a watchdog tick commands zero, so the robot stutters rather
than drifts. It would matter at the 0.5 m/s envelope, which is exactly what
`speed_args.py` and the RTF throttle exist for.

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

```bash
roslaunch agbot_gps_nav mapviz.launch      # alongside a running sim or field session
```

OSM tiles over ACRE with the robot on them; clicking publishes to
`/gps_nav_node/goal_wgs84`, the same topic as 1.2(b).

⚠ **Do not start it with `rosrun mapviz mapviz` and File > Open Config.** That
was the instruction here until 2026-09-07 and it gives a **flat grey canvas**,
which reads as a broken tile source and is not — the tiles are fine
(`tile.openstreetmap.org` answers 200). mapviz places everything through
swri_transform_util's TransformManager, which has to know where the `map` frame
sits on Earth and learns it from `/local_xy_origin`. Nothing published that, so
mapviz could not turn a map coordinate into a latitude and painted its own
`background: "#a0a0a4"` instead. `mapviz.launch` runs `initialize_origin.py`
alongside mapviz to supply it.

⚠ The origin comes from `gps_datum.yaml` and is **not written out a second
time**: `gps_nav_node` latches the datum as a `NavSatFix` on `~datum_fix`, and
`initialize_origin.py` takes the first fix on that topic. So `gps_nav_node` must
be running. It is deliberately NOT `local_xy_origin: auto` against the robot's
real fix topic — that anchors the map frame wherever the robot booted, which in
the sim is the trailer 18 m south of the field.

⚠ **CLICKING THE MAP STEERS THE ROBOT.** `point_click_publisher` fires on
*every* click on `/gps_nav_node/goal_wgs84`, and clicking is also how you pan
and inspect — so looking at the map sends a goal. That is the feature in the
§1.2 hand-driving workflow and a hazard during a mission.

It bit a real run on 2026-09-07: two clicks during the 44 s the segmentation
model takes to load sent the robot to map `(46.05, -10.88)` — 47 m east — then
to `(-14.08, -15.84)` behind it, before the supervisor's real goal arrived at
t=54. The robot drove a large triangle across the field, then completed the
mission correctly. The transit looked like a navigation fault and was not one;
the only trace was two ordinary `goal from WGS84:` INFO lines buried in the
timing stream.

Fixed at the node, not at mapviz: `gps_vision_mission.launch` now passes
`external_goals_enabled:=false`, so during a sequence the RViz and mapviz
channels are **refused and logged by name** while the supervisor's `~goal_pose`
still works. The startup config block says which:
`goals: ~goal_pose always | RViz + mapviz clicks LOCKED OUT`. And in either
mode, a goal that displaces one already being driven is now a WARN naming both
points. Standalone (§1.2) click-to-goal is unchanged.

⚠ `tile_map` is listed first in the config on purpose — mapviz paints plugins in
list order and a tile layer listed later covers the robot.

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

**The five lines that say a transit is healthy**, in the order they appear:

| line | meaning | bad version |
|---|---|---|
| `goals: ~goal_pose always \| RViz + mapviz clicks LOCKED OUT` | the supervisor owns the node (§3.7) | `... ACCEPTED` during a mission — a stray click can redirect the run |
| `heading bootstrap converged: yaw vs course driven = N deg` | heading locked on (§3.1) | red `HEADING BOOTSTRAP FAILED` — it ran out of ground |
| `ARRIVED (0.30 m from goal) ... arrival heading check = N deg` | arrived pointing the right way (§3.5b) | over 12° and the handoff is refused |
| `ROW_MISSION: gps=off vision=on` | handoff taken | `FAILED` with the arrival-heading reason |
| `Mission DONE: rows_driven=3, blocked rows: none` | the row mission finished | `rows_driven` short of `num_rows` (see §3.6) |

⚠ Two warnings worth knowing on sight:
`goal from ... REPLACES the goal being driven` means something redirected the
robot mid-transit, and `IGNORING goal from ...` means the lock refused one.
Both are §3.7.

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
  launch/mapviz.launch                   mapviz + initialize_origin (see 1.3)
  test/                                  161 tests, no ROS needed

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
| after 3 m of the bootstrap (yaw process noise `0.3`) | **6.6°** |
| after 10 m of the same leg | **0.7°** |
| after 10 m with the OLD `0.01` tuning | **~48°** — it never converged |
| stationary, any start | **drifts ~17°/min, unbounded** |

`ekf_map` fuses body-frame wheel velocities against absolute GPS positions, so
**yaw is observable whenever the robot moves** — that is course-over-ground
estimation arriving out of the dual-EKF structure rather than from a module.

But it is not free. In the maize world a 92° start with no bootstrap **missed
the row entrance by 5.9 m**. So `heading_init_distance` (the bootstrap) is
required there and is on by default in `gps_vision_mission.launch`.

⚠ **And observable is not the same as observed.** Until 2026-09-07 that
correction was starved by this filter's own tuning:
`ekf_map.yaml`'s `process_noise_covariance` gave x/y `1.0` and yaw `0.01`.
Nothing observes yaw directly, so position and yaw compete for the SAME GPS
innovation, and a term 100× stiffer never wins it — the filter spent every
disagreement on position. Yaw is now `0.3`, matching the roll/pitch entries.

Measured the same day, blank world, robot spawned ~66° off, identical in every
other respect (`agbot_gps_sim.launch yaw:=1.2`, one 20 m goal on a bearing):

| yaw process noise | what the bootstrap did |
|---|---|
| `0.01` (before) | **gave up at its 10 m backstop, still 47.9° wrong**, then wandered — ground-truth error was still 48° at 10 m along |
| `0.3` (after) | **converged at 3.0 m with 6.3°**; ground truth 66° → 6.6° over those 3 m, → 0.7° by 10 m; arrived 0.29 m from the goal |

⚠ **The drift is CONTINUOUS, not just a bad starting value.** `jackal.gazebo`
models the gyro with `rateDrift 0.005 rad/s` — **17°/min**, and it was measured
at that: the 2026-09-07 19:05 run's odom yaw moved −17.4° in the ~50 s the
robot sat still waiting for the segmentation model to load, before it had moved
at all. A one-shot bootstrap cannot hold a 160 s transit against that on its
own; what makes it hold is the EKF correcting continuously underneath, which is
what the process-noise change turned back on. §5.3 is still wanted for the
at-rest case.

⚠ **The bootstrap also drives at CRUISE now, not `approach_speed`.** It is an
observability manoeuvre and the signal it feeds the filter grows with ground
speed, so 0.15 m/s made it both the weakest signal available and the longest
wait — the 10 m backstop took 67 s, laying down ~19° of fresh drift while it
ran.

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

Crossing further off-axis than `arrival_cross_tolerance` re-stages and retries,
up to `max_approach_attempts`, then fails cleanly. The generic default is 0.35 m
— under half a row spacing, so "arrived" cannot mean the next corridor — but
`gps_vision_mission.launch` tightens it to **0.20**, because fitting through a
row entrance is a stricter question than being at the right point. See §3.5b for
the arithmetic.

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

### 3.5b Arriving is not the same as arriving pointed the right way

⚠ `gps_state == ARRIVED` used to be the ONLY condition on the handoff. GPS pins
position and never orientation, so it says the robot reached the right *place*
and nothing about which way it faces. On 2026-09-07 it reached the corridor 0
entrance 0.41 m from the goal and ~72° off it; vision nav's first frames read
`w=1.0 edges=1/1` — open field, no corridor — then `obst=0.78`, a wall of corn.
Two `BLOCKED` events, a back-out, and the mission ended at rows=0/3.

⚠ **`ALIGN` cannot catch this**, and neither can final-heading-error-vs-bearing:
ALIGN turns the robot until the *estimate* reads the bearing, so that number is
the estimate agreeing with itself. The independent witness is the course the
robot actually drove. The final on-axis leg is `staging_distance` of straight
driving, so it is a free course-over-ground sample — `approach_course_error()`
compares it against the yaw estimate, `gps_nav_node` publishes it on
`~arrival_heading_error_deg` (a number on its own latched topic, deliberately
not words scraped out of `~status`), and `handoff_fsm` refuses the handoff over
`max_arrival_heading_error_deg` (12°, the bootstrap's tolerance). Measured on a
good run: **−0.4°**.

An *unmeasurable* residual — no bearing on the goal, or a leg shorter than
`waypoint_follower.MIN_COURSE_DISTANCE` — is allowed through with a warning.
The gate fires on evidence of being wrong, not on the absence of evidence.

⚠ `arrival_cross_tolerance` is **0.20 in `gps_vision_mission.launch`**, tighter
than params.yaml's generic 0.35. The Jackal is ~0.43 m wide in a 0.75 m
corridor — 0.16 m of clearance per side — so a 0.35 m "arrival" is already
0.13 m inside the plant row.

### 3.6 `first_turn_direction` must match the corridor

Rows are ordered by increasing x and the robot enters along the bearing, so out
of the **leftmost** corridor the next one is to its **RIGHT**. Turning left out
of `corridor_0` drove straight out of the field and ended the mission at
`rows=1/3` in open ground. `rows_to_waypoints.py` emits the direction that
belongs with each corridor.

Also: without `rear_camera_enabled` a blocked-ahead signal **ends the mission**
(the BACKOUT states are unreachable), which in a 3-row run reads as a mission
that quietly finished at `rows=1/3`. It is on by default in the mission launch.

### 3.7 The GPS node used to obey ANYONE, silently

⚠ `gps_nav_node` accepted a goal from any publisher at any time — including
mid-transit while a supervisor was driving it — and logged it as an ordinary
INFO indistinguishable from the first goal. The supervisor sends its goal and
then *assumes* it owns the node. It did not.

That is not a theoretical hole. mapviz's `point_click_publisher` fires on
**every** click on `~goal_wgs84`, and clicking is also how you pan and inspect
the map — so looking at the map steered the robot. On 2026-09-07 two clicks
during the **44 s the segmentation model takes to load** (the supervisor waits
for `/vision_nav_node/set_enabled` before sending its goal, so the GPS node is
live and unowned that whole time) did this:

```
t=32.5  goal from WGS84 ... = map ( 46.05, -10.88)   <- click 1: 47 m ENE, 77 deg right turn
t=40.1  heading bootstrap converged 1.9 deg           <- bootstrapping toward the WRONG goal
t=50.5  goal from WGS84 ... = map (-14.08, -15.84)   <- click 2: behind, 151 deg swing back
t=54.2  goal sent: corridor_0 -> map (-0.80, -3.86)  <- the supervisor's REAL goal
```

The robot drove a large triangle across the field and then completed the mission
perfectly. **That is the shape of a fault that gets blamed on navigation**, and
the only trace was two INFO lines buried in the timing stream.

The fix is at the node, not at mapviz:

- `~external_goals_enabled` (**true** by default, **false** from
  `gps_vision_mission.launch`) gates the two INTERACTIVE channels — RViz's
  `/move_base_simple/goal` and mapviz's `~goal_wgs84`. Refused goals are logged
  by name and land on `~status`.
- ⚠ `~goal_pose` is **deliberately not gated**. It is the supervisor's
  programmatic channel, no GUI publishes to it, and gating it would lock the
  mission out of its own node.
- In **either** mode, a goal that displaces one already being driven is now a
  `logwarn` naming both points. That warning is the general fix; the lock is the
  specific one.

Standalone hand-driving (§1.2) is unchanged — click-to-goal is the whole point
there.

### 3.8 Things that are NOT bugs — do not chase them

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
- **`heading bootstrap converged` appearing TWICE in one run** is not a
  double-bootstrap bug. Every new goal restarts the follower at `HEADING_INIT`
  (`set_goal` does), so a run that received two goals bootstraps twice. If you
  did not send two goals, that second line is evidence something else did —
  see §3.7.
- The ~44 s gap between `gps_nav_node ready` and `goal sent:` is the
  segmentation model loading. The supervisor waits on
  `/vision_nav_node/set_enabled` before it starts the transit, so the robot sits
  still and the gyro drifts (§3.1) for that whole window. It is why the
  bootstrap has to exist rather than merely being nice to have.

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
| `arrival_cross_tolerance` | 0.35 (**0.20 in the mission launch**) | must stay under half a row spacing, and well under it for a row entrance: the Jackal is ~0.43 m wide in a 0.75 m corridor |
| `external_goals_enabled` | true (**false in the mission launch**) | whether RViz 2D Nav Goals and mapviz clicks are honoured. ⚠ true is right for hand-driving; under a supervisor a stray click silently redirects the run |
| `max_arrival_heading_error_deg` | 12.0 (`mission_supervisor`) | ⚠ the handoff gate. Over this the supervisor refuses to enable vision nav and stops. Raising it is how the robot ends up in the corn |
| `heading_init_distance` / `_max_distance` | 0.0 / 8.0 (**3.0 / 10.0 in the mission launch**) | bootstrap min and backstop. ⚠ **0 disables it — fine in a blank world, NOT on the robot.** It drives STRAIGHT, so the backstop must fit the open ground ahead |
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
straight until the course driven agrees with the yaw estimate, plus the
`ekf_map` process-noise fix (§3.1) that lets GPS keep correcting yaw *while the
robot moves*. Together those fix the start of a run and hold it through a
transit.

They do **not** fix the unbounded **~17°/min** at-rest drift (`jackal.gazebo`
models `rateDrift 0.005 rad/s`; measured as −17.4° over 50 s of standing still).
Yaw is only observable from motion, so a robot parked for several minutes
mid-mission still starts its next leg on a rotten heading — it will re-converge
once moving, but it steers on the bad estimate first. That is the remaining
gap.

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
