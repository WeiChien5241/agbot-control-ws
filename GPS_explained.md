# GPS waypoint navigation, explained in plain terms

Summary of a Q&A session on 2026-10-07. It covers how the GPS navigation works,
what was built, what the 10/5 and 10/6 field tests showed, the answers to the
follow-up questions, and the next steps. Detailed references:
`GPS_plan.md` (design and measurements), `GPS_testing_10-05.md` (field notes),
`CLAUDE.md` (project rules).

---

## 1. The problem GPS solves

- **Vision nav** (`agbot_vision_nav`) already drives the robot *inside* a corn
  row: the camera sees the corn, the segmentation model finds the gap, the
  controller steers down the middle.
- It cannot get the robot **from the trailer to the start of a row**. That
  stretch is open ground with no corn to follow.
- **GPS covers only that stretch.** It is not used in the rows: the canopy
  blocks and reflects the signal, and the lab's P-AgNav paper requires in-row
  navigation to work without GNSS.
- **Goal:** "the row starts at this lat/lon, facing this way". The robot drives
  there, lines up, and hands control to vision nav, which runs the multi-row
  mission.

## 2. How it works

### 2.1 Where am I? (localization)

| Source | Good at | Bad at |
|---|---|---|
| GPS (Emlid Reach M2) | Absolute position on Earth | 1 update/s; **no heading at all** |
| Wheel odometry | Smooth, fast "moved 10 cm forward" | Error builds up |
| IMU gyro | "Turning left at 5°/s" | Drifts slowly, even when parked |

- An **EKF** (Extended Kalman Filter) blends all three into one estimate:
  position (x, y) and heading θ.
- **Datum:** one chosen lat/lon point that is (0, 0). All positions become
  "metres east, metres north of the datum". It is defined once, in
  `agbot_gps_nav/config/gps_datum.yaml`, which is still a **placeholder** and
  must be surveyed at the real field.
- **Two filters, on purpose.** The Jackal's stock EKF (`/odometry/filtered`) is
  left untouched, because vision nav needs a smooth estimate that never jumps.
  Ours, `ekf_map` (`/odometry/filtered/global`), is a second filter that can
  jump when GPS corrects it.

### 2.2 The heading problem, and the bootstrap

- GPS gives position, never direction: one antenna is a single dot, and a dot
  has no "front". The robot has no usable compass (the motors disturb it).
- **Driving reveals heading.** Picture being blindfolded with someone calling
  out your position. Standing still, you can't tell which way you face. Walk
  two steps and they say "you moved north", so you face north. The EKF does the
  same: it predicts "I face east and drove 1 m, so GPS should show 1 m east".
  GPS says "1 m north", so the heading must be wrong, and the filter corrects
  it. This is called *course over ground*.
- **"Drive before steering."** Steering uses the heading. If the heading is
  107° wrong, the robot confidently turns the wrong way. So the **heading
  bootstrap** (`heading_init_distance`) first drives straight, without
  steering, until the GPS trail agrees with the heading estimate (within 12°).
  Only then does it steer. It ends on that measurement, with
  `heading_init_max_distance` as a backstop.

### 2.3 Driving from A to B (`waypoint_follower.py`)

States: `IDLE → HEADING_INIT → GOTO → (ALIGN → APPROACH) → ARRIVED / FAILED`

Every 50 ms (20 Hz):
1. The robot's pose (x, y, θ) comes from `ekf_map`. The goal's lat/lon was
   converted to x/y on the same map.
2. Compute the direction from robot to goal (the *bearing*):
   heading error = bearing − θ.
3. Decide:

| Heading error | What the robot does |
|---|---|
| > 30° (`turn_in_place_deg`) | Stop and spin in place at 0.4 rad/s toward the goal |
| ≤ 30° | Drive forward and curve: turn = 1.2 × error (`heading_gain`, radians), capped at 0.6 rad/s (`angular_z_max`); forward = 0.4 × (1 − \|error\|/60°) (`slow_down_deg`) |
| Last 2 m | Base speed ramps down from 0.4 to 0.15 m/s |
| Within 0.3 m (`goal_tolerance`) | ARRIVED, stopped and **latched** (stays stopped until a new goal) |

The forward speed and turn rate this produces:

| Error | Forward | Turn rate | Turn radius |
|---|---|---|---|
| 0° | 0.40 m/s | 0 | straight |
| 10° | 0.33 m/s | 0.21 rad/s | ~1.6 m |
| 20° | 0.27 m/s | 0.42 rad/s | ~0.6 m |
| 30° | 0.20 m/s | 0.60 rad/s (cap) | ~0.33 m |
| > 30° | 0 | 0.4 rad/s | spins on the spot |

**Example: goal 90° to the right.** After the bootstrap, the error is −90°, so
the robot spins right in place until it is within 30°. Then it drives while
curving to finish the turn, then drives straight.

**Lessons built in:**
- Spinning in place above 30° prevents *orbiting* a close, off-to-the-side goal.
- Slowing down while turning keeps arcs tight. The curve depends on
  turn rate ÷ forward speed (the vision-nav lesson from HANDOFF3 §0f).
- For row entrances, the final leg follows the **row's center line**, not the
  goal point (aiming at the point gave 41° off; following the line gave 0.4°).
- Arrival at a row entrance is **crossing a finish line** across the row axis
  (within 0.20 m of the center line), not entering a circle. With a circle, the
  robot once passed 0.46 m to the side and kept driving.

**Where the numbers came from.** The *shape* of each rule is documented in
commit `3998099`; the *exact numbers* are not, so the following is a
reconstruction:
- 0.6 cap ÷ 1.2 gain = 0.5 rad ≈ 29°, so the turn rate saturates right where
  the 30° spin-in-place takes over.
- 60° = 2 × 30°, so the robot leaves spin mode at half speed, which gives a
  tightest drive-mode arc of ~0.33 m.
- 1.2 is in the usual 1–2 range for a proportional heading controller, and a
  heading error shrinks by about two-thirds every ~0.8 s.
- 0.4 m/s is comfortable to walk beside with a hand on the deadman.
- Small mismatch: spin-in-place turns at 0.4 rad/s, but drive mode just below
  30° turns at 0.6.
- **None of these has been tuned on the real robot.** They work in sim and
  passed one 8 m real leg.

### 2.4 Multi-point routes (`route.py`)

- Click several points on mapviz. The middle points are passed through (within
  1.0 m, or by crossing them) without stopping; only the last one is "arrived".
- **Default `click_mode: queue`**: clicks only append points, and nothing moves
  until `~route/go`. Clicking is also how you pan the map, and two stray clicks
  once drove the sim robot 47 m off course. `click_mode:=direct` makes every
  click a goal.
- Routes save and load as lat/lon files.

### 2.5 Handing off to vision nav (`mission_supervisor.py` + `handoff_fsm.py`)

`TRANSIT (GPS drives) → ROW_MISSION (vision drives) → FINISHED` (or `FAILED`)

For a row entrance:
1. GPS drives to a staging point 5 m before the row, turns to face down the row
   (ALIGN), drives in along the center line (APPROACH), and crosses the finish
   line (ARRIVED).
2. The supervisor checks the heading: the course actually driven on the final
   leg must agree with the heading estimate within 12°. Otherwise it **aborts**
   instead of handing over.
3. It switches GPS off **first**, then vision nav on. Only one may drive:
   both publish `/cmd_vel`, and a disabled node goes **silent** rather than
   sending zeros, which would fight the active one.
4. Vision nav is reset (its mission restarts at row 1) and starts in
   **FOLLOW_ROW**. It does **not** search for the row first.
   - Possible improvement: start in **REACQUIRE**, which finds the row and
     centres on it before FOLLOW_ROW. It would forgive a slightly off GPS
     arrival.

## 3. Packages and libraries

**Existing packages we use:**

| Package | Role |
|---|---|
| `robot_localization` | ROS package for "where am I". `ekf_localization_node` = the filter (our `ekf_map`, and the Jackal's stock EKF). `navsat_transform_node` = converts GPS lat/lon into map x/y. We **configure** it (`ekf_map.yaml`, `navsat_transform.yaml`), including **not** trusting the IMU's absolute yaw. |
| `reach_ros_node` | Reach M2 driver (from a colleague; we fixed two bugs). Reads NMEA over USB-Ethernet TCP and publishes `/gps/fix`. |
| `twist_mux` | Picks who drives. The joystick (priority 9–10) always beats autonomy (priority 1). |
| `hector_gazebo_plugins` | Simulated GPS in Gazebo |
| `mapviz` | Satellite-style map for clicking goals |
| RViz | Debug view |

**Deliberately not used:**
- `move_base` (ROS1's equivalent of Nav2): it needs a laser, and the trailer →
  row path is a straight line.
- `pyproj` / `geodesy`: plain math is accurate to millimetres at field scale.

**What we wrote (`agbot_gps_nav/`):**
- `src/agbot_gps_nav/geo.py`: lat/lon ↔ map x/y math
- `src/agbot_gps_nav/waypoint_follower.py`: the drive-to-goal state machine
- `src/agbot_gps_nav/route.py`: multi-point routes
- `src/agbot_gps_nav/handoff_fsm.py`: GPS → vision handoff logic
- `scripts/gps_nav_node.py`: the follower's ROS node
- `scripts/mission_supervisor.py`: the handoff's ROS node
- `scripts/rows_to_waypoints.py`: row-entrance waypoints from the Gazebo world
- 193 unit tests that run without ROS

### `/gps/fix`

The raw GPS reading (`sensor_msgs/NavSatFix`), published at 1 Hz by the Reach
driver:

| Field | Meaning |
|---|---|
| `latitude`, `longitude` | Antenna position |
| `altitude` | Ignored (2D navigation) |
| `status` | −1 none; 0 single (~0.4–2 m); 1 RTK float **or** DGPS (the driver maps both to 1); **2 RTK fixed (~1 cm)** |
| `position_covariance` | The receiver's own accuracy estimate (from the GST sentence) |

`min_fix_status:=2` makes the robot refuse to move without RTK fixed.

### Why our own lat/lon conversion and not `/fromLL`

- `navsat_transform` offers `/fromLL` and `/toLL`, and using them would be a
  valid choice. Our `geo.latlon_to_map` was **checked against `/fromLL`**: an
  earlier version was 0.04% off (20 cm at 500 m) and was fixed. They now agree
  to 0.1 mm.
- We kept a function because:
  - a service only works while `navsat_transform` is up with its datum, and we
    convert at startup and when saving or loading routes;
  - every service call is a round trip that can fail or hang;
  - the 193 tests run without ROS;
  - the Gazebo tools need true ground metres, which `/fromLL` doesn't give.
- **Risk:** if `robot_localization` changes its math, our copy would silently
  disagree. **To do:** call `/fromLL` once at startup and log an error if they
  differ by more than 1 cm.

## 4. What was built in Gazebo

- **Blank world** (`agbot_gps_sim.launch`): empty ground, Jackal, simulated GPS.
  Used for A → B tests.
- **GPS maize world** (`switch_maize_world.sh gps`): the corn field plus a 20 m
  headland, with the robot spawned at a "trailer" 18 m out. This is the full
  demo: trailer → row entrance → handoff → 3-row vision mission
  (`gps_vision_mission.launch`).
- **Sim findings:**
  - From 74° wrong, heading was 6.6° off after 3 m and 0.7° after 10 m, once
    the EKF yaw process noise went from 0.01 to 0.3. Before that fix it was
    still 48° off after 10 m.
  - Without the bootstrap, a bad start heading missed the row entrance by 5.9 m.
  - The sim IMU gives a fake perfect compass heading. It is deliberately
    ignored, because the real robot has none.

## 5. Field tests

### 10/5: first Reach M2 → Jackal test
- ✅ The chain works: Reach → USB-Ethernet → driver → `/gps/fix`. Single fix
  outdoors, about ±42 cm.
- 🐛 Driver bug fixed: an empty GGA field (no satellites) crashed the parser,
  so nothing was published (commit `9ff4490`).
- ❌ No RTK: a damaged antenna cable, and no NTRIP credentials on either unit.
- Found: `/navsat/*` is the Jackal's built-in consumer GPS chip (never had a
  fix). The `/ns1`, `/ns2` Velodyne topics are leftover drivers; no LiDAR is
  mounted.

### 10/6: NTRIP RTK, localization, first self-driven leg
A third Reach (`Reach:46:A2`) had an NTRIP profile. Corrections come over the
iPhone hotspot, so **one Reach and one antenna give RTK, with no base station**.

**`rtk_static_1513.bag`** (parked 238 s):
- RTK fixed on all 237 messages. Wander ±0.4 cm (E), ±0.3 cm (N), about 100×
  better than 10/5's single fix.
- Gyro drift while parked:

| IMU | Bias (mean gyro z) | Drift | Noise (1σ) |
|---|---|---|---|
| Jackal built-in `/imu/data` | +0.100°/s | +6.0°/min | 0.054°/s |
| MicroStrain 3DM-GX5 `/gx5/imu/data` | −0.067°/s | −4.0°/min | 0.036°/s |

  The stock EKF's heading drifted +24.3° in 4 min (6.1°/min), almost exactly
  the built-in IMU's bias. The sim's 17°/min was only a sim parameter.

**`stage5_rtk_1553.bag`** (joystick, 13 m line plus a square):
- Returned to the tape mark within **2.5 cm** (RTK fixed) after the square.
- **Heading went from 107° wrong to within 5° after about 1.7 m (3.6 s) of
  driving** and stayed within ±5° (typically 0–3°) on every later straight,
  including in reverse. Course-over-ground heading works on real hardware.
- Fix status per second: fixed 0–74 s → **status 1 for 75–197 s (123 s,
  starting right as driving began)** → fixed 198–240 s.

**`nmea_1620.log`** (only 26 s; the logger died early): all GGA quality 4
(RTK fixed), 16 satellites, HDOP 2.3, correction age 0.4 s, σ ≈ 1 cm.

**Stage 7, first GPS-driven leg (~8 m, RTK, deadman held):**
- Bootstrap 3 m, converged at −7.2° → GOTO → APPROACH → ARRIVED.
- 0.29 m from the goal by the EKF, **0.50 m by raw GPS**. Expected: it stops
  when it enters the 0.3 m tolerance. Today's test A tries 0.15.
- RTK flickered to float for 1–2 s while parked afterwards (16 satellites, HDOP
  2.3, weaker sky there). The fix gate paused and resumed the robot correctly.

### On RTK float
- The operator saw **fixed ~99% of the time** on the phone, with brief 1–2 s
  float flickers. That matches Stage 7 and the overall picture.
- The bags do contain **one** 123 s non-fixed stretch (≈ 15:54:27–15:56:29 on
  10/6). It may be a one-off (someone near the antenna, the hotspot phone
  carried around). ROS status 1 can't tell float from DGPS; the phone, the
  NMEA log or the Reach's own logs can.
- Short flickers are harmless. Logging (next steps) will show whether long
  dropouts recur.

### What the tests tell us

| Question | Answer |
|---|---|
| GPS → ROS chain works? | ✅ |
| RTK accurate enough? | ✅ centimetre level |
| Heading from driving works on the real robot? | ✅ < 5° within 2–3 m |
| Robot drives itself to a GPS point? | ✅ one 8 m leg |
| RTK reliable while moving? | ⚠ mostly; one 2-min dropout to explain |
| Heading at boot or at rest? | ❌ drifts ~6°/min, must drive before steering |
| Field datum set? | ❌ still a placeholder |

## 6. Heading and the IMUs

- **6°/min while parked** means the robot thinks it is slowly turning while it
  sits still. A gyro measures *turn rate*, which the robot adds up into a
  direction. A constant error in that rate (the *bias*) adds up without limit.
- **There are two IMUs**: the Jackal's built-in one (used by our EKF now) and
  an added MicroStrain 3DM-GX5. Check the label: a GX5-45 has its own GNSS/INS,
  a -25 does not.
- **"GX5 is better"** comes from the same one parked bag: about one-third lower
  bias and noise. That's one 4-minute sample at one temperature, so it's a hint,
  not proof. To confirm: repeat parked bags (cold start, warm, other days);
  compare datasheets ("bias instability", "angle random walk"); check whether
  the GX5's onboard filter (`/gx5/ekf/...`) already removes the bias.
- **Why bias estimation still has to be built:** the 10/6 numbers were measured
  afterwards on the laptop, and the robot doesn't use them. Bias changes at
  every power-up and with temperature, so it can't be a fixed config value.
  The robot must measure it each time it's parked (wheels still means true
  turn rate zero, so any reading is bias) and subtract it. `robot_localization`
  does not estimate gyro bias itself.
- **Clearpath's calibration:** as far as we know it is the magnetometer
  (compass) calibration (`calibrate_compass` in `jackal_base`; confirm against
  the link you saw). It won't fix gyro drift, but a working compass would give
  **heading at rest**. The motors' own magnetic field may still corrupt it
  while driving. Cheap to try: calibrate, then compare compass heading against
  GPS course in a bag (parked and on straights).

**Options for heading, cheapest first:**

| Option | Fixes | Status |
|---|---|---|
| Bootstrap | Wrong heading at the start of a drive | Built; needs 2–3 m of clear ground |
| Gyro bias estimated while parked | Most of the 6°/min drift | Not built |
| Use the GX5 | Lower noise and bias | Config change + test |
| Heading from the trailer | Heading at drop-off | Depends on the trailer |
| Compass calibration | Possibly heading at rest | Experiment |
| Second Reach M2 (dual antenna) | Heading at rest | Hardware; a few degrees given the Jackal's length |

## 7. Why not the colleague's `pagslam_mapping`

- It is one preprocessing piece of a **3D-LiDAR** system. It converts Velodyne
  point clouds from a bag into `.pcd` files and a 2D occupancy map, and records
  the map's GPS origin.
- Navigating with it needs the rest of that stack: `pagslam` (LiDAR SLAM),
  `gtsam_test`, `gps`, `autonomous_navigation` (P-AgNav), `octomap_server`, and
  a RealSense T265.
- It doesn't fit us because:
  - the Velodynes are no longer on the robot, so there is no data to feed it;
  - it needs a manual mapping run before navigating;
  - its heading "solution" is pointing the robot east with an iPhone compass
    before boot;
  - it's overkill for a straight open-ground transit.
- What we kept from it: the `reach_ros_node` driver (two bugs fixed) and the
  habit of pointing the robot roughly east at boot.

## 8. Depth cameras

- **Yes, an RGB-D camera** (RealSense D435/D455, ZED, OAK-D) gives a normal
  colour image **plus** a depth image (metres per pixel), and can publish a 3D
  point cloud. Set the driver to *align* depth to colour (e.g.
  `align_depth:=true`) so their pixels match.
- **It can feed obstacle avoidance**, either through
  `depthimage_to_laserscan` (a fake 2D laser) or with the point cloud going
  straight into the costmap. On our ROS1 Noetic robot that means `move_base`;
  Nav2 is ROS2.
- **Caveats:**
  - weeds look like walls, just as they do to a LiDAR;
  - limited view (~87° wide, reliable to ~3–6 m);
  - sunlight degrades some depth types (time-of-flight worst, stereo better).
- **Not needed for v1** (clear straight transit). It would be the right add-on
  for a stop-on-obstacle check (see §9).

## 9. When the path isn't straight and clear

1. **Known fixed obstacles** (poles, ditches, irrigation lines): a
   **multi-point route** around them. This exists already: survey once, save,
   reuse.
2. **Unexpected obstacles** (people, vehicles): a **stop-and-wait check**, e.g.
   a depth camera stops the robot if anything is within ~1.5 m ahead. Not
   built; small.
3. **Real path planning around unknown obstacles:** `move_base` with our GPS
   localization plus a depth-camera costmap. A large project; only if 1 and 2
   are not enough.

## 10. Use case: deploying from the autonomous trailer

The trailer parks and lowers its ramp. Each Jackal reverses off, then drives
to the corn field, which is to the right and slightly ahead.

1. **Survey once:** set the real datum; joystick to each row entrance and
   record its lat/lon and row direction. Goals are then fixed on Earth, so
   where the trailer parks doesn't matter.
2. **Reverse down the ramp** a set distance by wheel odometry, slowly, with no
   GPS steering.
3. **Learn heading in reverse.** ⚠ After backing off, the robot **faces the
   trailer**, so the current forward bootstrap would drive into the ramp. Keep
   reversing straight 2–3 m instead: the EKF learns heading from reverse motion
   too (confirmed 10/6), and the robot gains clearance. Needs a code change
   (a reverse bootstrap).
   - **Better, if available:** if the trailer has its own RTK heading, the
     Jackal's heading = trailer heading + 180° at drop-off. That solves heading
     at boot with no driving. Ask the trailer team.
4. **Drive to the row** with the existing follower: an optional clearance point
   so it doesn't cut past the trailer, then the row entrance with its row
   direction (staging point → align → approach on the center line). The
   diagonal "front-right" layout needs no special handling; the robot turns in
   place toward each point. The spin just has to happen clear of the trailer.
5. **Hand off to vision nav** (exists, sim-tested).

Also:
- With several Jackals, give each its own row and release them one after
  another. Nothing prevents robot-robot collisions.
- Wait for RTK fixed before step 3 (step 2 is odometry only).
- **Needed from the team:** a layout sketch (trailer position and heading, ramp
  direction, row starts) to make the route concrete and test it in Gazebo.

## 11. Next steps (priority order)

Big goal: **real robot, trailer → row entrance → vision handoff → rows driven.**

### A. Field (10/7 plan in `GPS_testing_10-05.md`)
1. Run tests A–E:
   - A: `goal_tolerance:=0.15`
   - B: 90° bad start heading
   - C: 20–30 m leg
   - D: 2–3 point route
   - E: parked bag at a new spot

   Add a **`heading_gain` sweep (0.8 / 1.2 / 1.8)** to test B, with a bag of
   `/odometry/filtered/global`, `/cmd_vel` and `/gps_nav_node/status`.
2. Turn on the **Reach's own logging** (position + raw + corrections) and keep
   the NMEA log running, to explain any float dropout and tell float from DGPS.
3. Stop every `rosbag record` with **Ctrl-C** in its own pane (no more
   `.bag.active`).
4. Read the **GX5 model** off the label; record **Reach B's serial**.

### B. Laptop
5. **Bag analysis script**: arrival error vs raw GPS, bootstrap distance, gain
   sweep (wobble, overshoot, settle distance, time at the 0.6 cap), and the fix
   status timeline.
6. **Start the handoff in REACQUIRE** instead of FOLLOW_ROW.
7. **Startup cross-check of `geo.latlon_to_map` against `/fromLL`.**

### C. Blocks the first real-field mission
8. **Survey the real field**: datum in `gps_datum.yaml`, plus every row
   entrance (lat/lon and row direction).
9. **First real GPS → vision handoff** on one row. It has only run in sim.

### D. Trailer deployment
10. **Reverse heading bootstrap.**
11. **"Deploy from trailer" sequence** (reverse down the ramp → reverse
    bootstrap → route → handoff), tested in Gazebo with a trailer spawn pose.
12. **Ask the trailer team** whether the trailer has an RTK heading to share.

### E. Heading improvements
13. Repeat parked bags (cold, warm, different days) to see how much bias
    changes.
14. Gyro bias estimation while parked.
15. Clearpath compass calibration experiment.
16. Dual-antenna Reach, only if 14–15 are not enough.

### F. Later, if needed
17. Depth-camera obstacle stop.
18. Reach driver logs fix-quality changes live (e.g. "RTK fixed → float").

**Shortest path:** 1–3 in the field, then 6, then 8–9.
