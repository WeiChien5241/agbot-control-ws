# HANDOFF — LiDAR transit work (sessions of 2026-10-13 / 14)

Start here next time. The detailed record is `LIDAR_notes.md`; this file is
where things stand and what to do next.

## Goal

The full workflow is drop-off → row entrance → vision in-row nav → back.

- The robot is set down near the corn by hand. **The trailer is out of
  scope for now** (it needs ArUco marker work).
- **Transit** (start → row entrance → back) uses the standard ROS1 stack: a
  2D map (slam_toolbox or gmapping) → **AMCL** → **move_base**. It runs on
  the robot's 3D Velodyne. This is the same workflow as Nav2: map, then
  "2D Pose Estimate", then the "2D Nav Goal" arrow.
- **In-row** driving stays camera vision nav (`agbot_vision_nav`), unchanged.
- **GPS stack** (`agbot_gps_nav`): kept and untouched, and the user wants it
  back in. The plan is in "Next steps" below.

## What exists

| piece | where |
|---|---|
| LiDAR on its housing in the URDF (opt-in `lidar:=true`) | `agbot_bringup/urdf/agbot_lidar.urdf.xacro` |
| the nav package | `agbot_nav/` |
| scan / mapping / localization / move_base / everything | `agbot_nav/launch/{scan,mapping,localization,move_base,transit,deploy_mission}.launch` |
| deployment sequencing (rospy-free, tested) | `agbot_nav/src/agbot_nav/deploy_fsm.py` + `scripts/deploy_supervisor.py` |
| save + pad a SLAM map | `rosrun agbot_nav save_map.sh <name>` |
| nav RViz view | `agbot_nav/rviz/nav.rviz` (map, scan, particles, costmaps, plans, drive handle, Pose Estimate / Nav Goal / Publish Point) |
| user's field world, built from blocks of rows | `agbot_bringup/config/agbot_field_layout.yaml` → `rosrun agbot_bringup build_field_world.py <yaml>` → snapshot `field` + `agbot_nav/maps/field.yaml` + `field_preview.png` |
| real-robot `base_link → velodyne` TF | `agbot_nav/launch/lidar_tf.launch` (a test keeps it equal to the xacro) |
| SIM-ONLY helpers | `gt_occupancy_map.py` (map from plant positions), `amcl_truth.py` (AMCL vs Gazebo truth), `vision_standin.py` (vision nav stand-in; `standin_oracle` steers from ground truth) |

**Robot model** (since 2026-10-21, matches the real robot):
- VLP-16 on the CAD holder on a 5 cm spacer, 0.125 m behind base_link,
  facing BACKWARDS, frame `velodyne1`, 0.49 m above the floor.

**The field world** (active world; all nav launches default to it):
- The robot starts at (-7.92, -1.30) facing north, with the corn on its
  right. It turns right (east) through a 1.9 m corridor between two short
  3-row blocks.
- Then left (north) about 1.3 m, then right into the north block's 3rd/4th
  row corridor. The entrance is (-0.5, 0.0) facing east.
- North block: 4 rows. South block: 3 rows. 8 m long.
- 333 tall plants on a 20 m square of ground.
- ⚠ Open question to the user: should the north block be 3 rows too?
- Rebuild after any edit to the yaml. The layout has NOT been driven since
  its last two changes.

## How to run (sim)

```bash
cd ~/agbot_control_ws && source devel/setup.bash
rosrun agbot_bringup switch_maize_world.sh field
roslaunch agbot_bringup agbot_gazebo.launch lidar:=true rviz:=false x:=-7.92 y:=-1.30 z:=0.35 yaw:=1.5917

# A. map it yourself (drive with the RViz drive handle), then save + pad
roslaunch agbot_nav mapping.launch rviz:=true            # or slam:=gmapping (apt install ros-noetic-gmapping first)
rosrun agbot_nav save_map.sh my_map

# B. navigate on it: 2D Nav Goal (map origin = where mapping started)
roslaunch agbot_nav transit.launch rviz:=true map_file:=$(rospack find agbot_nav)/maps/my_map.yaml \
  initial_x:=0 initial_y:=0 initial_yaw:=0

# C. full sequence: then RViz "2D Nav Goal" at a row mouth, arrow DOWN the row.
#    It drives there, does num_rows rows, comes back, waits for the next goal.
roslaunch agbot_nav deploy_mission.launch vision_standin:=true rviz:=true num_rows:=2
#    or the operator panel, section 5 (rows + speeds from section 2):
rosrun agbot_vision_nav operator_panel.py
#    scripted (fixed goal, ends after the way home):
#    ... entrance_x:=-0.5 entrance_y:=0.0 entrance_yaw_deg:=0
```

⚠ `vision_standin` is NOT vision nav: no camera, no MPC, no
`/vision_nav_node/debug/image`. It steers on Gazebo truth along the CLICKED
axis, so a sloppy arrow drives it into the corn. The real node needs torch,
lightly_train and the weights, none of which are on this laptop.

End a sim session with `rosrun agbot_bringup sim_teardown.sh`.

**Tests:**

```bash
cd agbot_nav && PYTHONPATH=src:$PYTHONPATH python3 -m pytest test -q        # 28
cd ../agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/test_launch_files.py -q   # 77
cd ../agbot_gps_nav && PYTHONPATH=src python3 -m pytest test -q            # 193
```

## Results so far (sim, against Gazebo ground truth)

- **First field layout** (start 4.7 m from the entrance):
  - true handoff 0.03 m off the corridor axis, +4.5°;
  - return 0.22 m / 8° from the start;
  - 0 plants hit (oracle stand-in).
- **gps_tall world:** full deploy completed, but one handoff landed 0.21 m
  off-axis. The corridor leaves ~0.16 m a side, so this is the weak spot.
- **AMCL** is 0.05–0.15 m at the corn and up to ~0.95 m in open ground.

## 2026-10-19: move_base fixed in sim (costmap saw leaves as walls)

- **Localization was fine.** `amcl_truth.py` over a 12 m user drive on the
  ground-truth map: AMCL within 0.22 m / 5.9° of truth throughout.
- **The planner was the problem.** The costmap read the same 0.10–1.20 m
  band as AMCL, so every hanging leaf was lethal. Measured canopy (tall sim
  maize): up to ~0.55 m above ground only stalks return (within 0.1 m of
  each); above that the leaves reach 0.3–0.55 m into the gap.
- **Fix** (`eef899f`): the costmaps read `/scan_obstacles` (max 0.40 m in
  `base_link`, `obstacle_max_height` arg on `scan.launch`). AMCL and SLAM
  keep the full `/scan`. Inflation 0.45/6.0 → 0.35/8.0.
- **A/B, spawn → entrance staging point (-1.2, 0), same sim:**

  | | old | new |
  |---|---|---|
  | time | 43 s | 23 s |
  | global plans | 158 | 23 |
  | path | stalls at start and mid-corridor, weaves 0.5 m | smooth, 7.4 m for 6.8 m straight |
  | closest stalk | 0.49 m | 0.53 m |

  The return leg on the new config succeeded too (25 s, 0.07 m from start).
- ⚠ **Real corn has leaves lower down.** Re-check `obstacle_max_height` on
  the robot before trusting it.
- ⚠ A map YOU build with SLAM still holds the leaves in its static layer (it
  is built from `/scan`). The ground-truth map does not. If planning on your
  own map is still tight, that is the next thing to change.
- **Creep measured again:** the robot moved ~55 mm in about a minute,
  standing still after a teleport.

## 2026-10-20: first indoor lab run on the robot

- Nav packages were already installed; code went over by bundle (`3a71b32`).
- LiDAR 9.9 Hz; `base_link -> velodyne1` = (-0.125, 0, 0.402), yaw 180°.
- Mapped the lab twice with fake corn (`lab`, `lab_new`; bag
  `mapping_2026-10-20-13-49-31.bag`, 139 s, on the laptop in `~/bags`).
- AMCL on `lab_new`: tracked smoothly, no jumps, spread ~5–8 cm.
- ⚠ **The map's (0, 0) is where odom started (robot BOOT), not where
  mapping started.** The taped start spot is (0.245, -2.623, 16.5°) in
  `lab_new`, read from the bag; "initial 0 0 0" put AMCL 2.6 m / 70° off.
  Fixed: `record_start_pose.py` (in mapping.launch) logs it and
  `save_map.sh` prints the right `initial_*` args.
- With the right initial pose (0.245, -2.623, 0.288): AMCL matched the walls,
  move_base reached goals at 0.3 m/s and re-planned around a person.
- ⚠ **Low obstacles are invisible:** the fake corn's ~10 cm wooden bases were
  clipped by the wheels. Both scans start at 0.10 m in `base_link` (0.165 m
  above the floor), and the lowest VLP-16 beam (-15° from 0.49 m up) meets
  the floor 1.8 m out, so a 10 cm board leaves the view inside ~1.5 m. Not a
  field issue (stalks are tall). For lab rows: map with a lower band so the
  bases land in the static map; do not lower the live obstacle band.
- ⚠ `~/bags` did not exist, so the first lap was not recorded. Fixed:
  `record_mapping_bag.sh` creates it.

## 2026-10-21: sim LiDAR moved to the real mount; full deploy re-run

- The sim LiDAR now matches cpr-j100-0864 (`f75d82d`): 0.125 m behind
  base_link, facing backwards, optical centre 0.425 m above base_link, frame
  `velodyne1`, on a holder modelled from the STEP files.
- Re-ran `deploy_mission.launch vision_standin:=true` on the `field` world
  with it (headless): LOCALIZE 2 s → TRANSIT 22 s → handoff
  **-0.109 m / +4.8°** off the axis (AMCL) → stand-in row 10.7 m → RETURN
  42 s → DEPLOYMENT COMPLETE. Gazebo truth at the end: 0.13 m and ~5° from
  the start pose.
- "REQUIRED process [deploy_supervisor] has died" after FINISHED is the
  supervisor exiting on purpose (`required="true"` takes the launch down
  with it), not a crash.

## 2026-10-21: interactive deployment (pick the entrance in RViz)

- No `entrance_*` → after LOCALIZE the supervisor waits (`WAIT_GOAL`) for a
  2D Nav Goal = the row mouth, arrow down the row; after RETURN, or a
  failure, it waits again. move_base's own `/move_base_simple/goal` is
  remapped away in this launch, so a click never drives move_base unchecked.
  `/deploy_supervisor/entrance` (latched) shows the goal in RViz and feeds
  the stand-in.
- Stand-in now drives `num_rows` (boustrophedon, `first_turn_direction`,
  `standin_row_spacing` 0.76) at `linear_x_cruise` (blank: 0.3).
  `transit_speed` sets move_base's top speed.
- Operator panel section 5 starts `deploy_mission.launch`.
- ⚠ **The parked sim Jackal SLIDES** (zero cmd_vel, wheels ~still): 3 cm
  in the first minute, then accelerating, 0.45-0.56 m in a later minute,
  with or without the LiDAR, at 5 ms and 2 ms physics steps. Odometry can't
  see it, so AMCL stays put. After a 5-minute wait, move_base planned from a
  pose 0.5 m stale and drove into the pen wall ("valid control could not be
  found"). `sim_creep_guard.py` (sim only) re-seeds AMCL from Gazebo truth
  on every goal. Root cause not found.
- Verified (field world, headless, 2 rows each): waited 3 min (crept
  0.45 m), goal (-0.5, 0, 0°) → arrived -0.09 m / -7.6° → 2 rows → home →
  WAIT_GOAL; goal (8.5, 1.52, 180°, entering from the EAST end) → 0.10 m /
  -7.5° → 2 rows → home, 0.06 m from the start.
- ⚠ The field world's default corridor is the north block's SOUTHERNMOST,
  so `first_turn_direction` defaults to `left` now (right led into the
  cross alley).

## Replaying a robot bag into SLAM (no robot needed)

Verified 2026-10-19 on `old_bags/2024-07-01-15-29-38.bag` (real corn, 6.5 m
drive): slam_toolbox drew both crop rows of the corridor. The bag's TF is
`odom -> base_link -> velodyne1` with no `map`, so it feeds SLAM as-is, and
it carries its own (2024) LiDAR TF.

```bash
roscore                                   # terminal 1
rosparam set /use_sim_time true           # terminal 2, BEFORE any node starts
roslaunch agbot_nav mapping.launch rviz:=true
rosbag play --clock old_bags/2024-07-01-15-29-38.bag \
  --topics /ns1/velodyne_points /tf /tf_static /odometry/filtered   # terminal 3
rosrun agbot_nav save_map.sh bag2024      # when playback ends
```

- Play only those topics. The bag also has the robot's own `/ns1/scan`;
  ours is `/scan`, so they don't collide, but nothing needs the extra load.
- Add `-r 0.5` to play slower if the laptop falls behind.
- The same works for `mapping.launch record:=true` bags from our own laps, and
  for `localization.launch` (AMCL on a saved map) instead of mapping.

## Hard-won lessons (each is commented where it lives)

- **AMCL must use the BEAM model.** The likelihood field aliases along corn
  rows; it once converged 1.4 m inside the corridor.
- **The sim LiDAR must be the GPU ray sensor.** The CPU one froze Gazebo.
- **Stock sim maize (0.63 m) is invisible to the LiDAR** past ~12 m. The
  sensor is ~0.58 m up. Use the tall variant (~1.26 m).
- **Scan settings:**
  - `angle_increment` 0.003491, not 0.0035, or slam_toolbox drops every scan;
  - no-return beams are filled at 39 m (`inf_epsilon: -1`), or SLAM never
    marks open ground as free.
- **SLAM maps only span real returns.** Pad saved maps (`save_map.sh` does
  it) and map with the rolling global costmap.
- **move_base stops 0.7 m before the entrance** (`entrance_standoff`): corn
  leaves reach ~0.6 m into the headland, so the costmap makes the entrance
  itself lethal.
- **AMCL thresholds:** the arrival gate is AMCL spread ≤ 0.15 m / 8° and
  ≤ 0.20 m / 12° from the axis. LOCALIZE is looser (0.30 m / 10°). AMCL is
  weak in open ground.
- **Sim SLAM is unreliable.** The sim Jackal creeps ~1–2 mm/s with its
  wheels still and odometry can't see it. One slam_toolbox lap went ~10 m
  wrong. Validate mapping on the real robot.
- **The Gazebo camera stops rendering with no subscriber** (headless, with
  `rviz:=false`). This doesn't affect vision nav.
- **`--` inside an XML comment breaks a launch/xacro.** This happened THREE
  times in these sessions. Run `test_launch_files.py` after every launch or
  xacro edit.
- **Shell scripts go under `install(PROGRAMS)`,** not
  `catkin_install_python`, which wraps them in a Python launcher.
- **Never `pkill -f <name>` from a shell whose command line contains that
  name.** It kills itself; use `sim_teardown.sh`.
- **Run ROS commands unsandboxed.** Topic data didn't flow from the
  sandboxed Bash; XML-RPC did.

## Next steps

1. **Sim:** the user builds their own map (A), navigates on it (B), then runs
   C with that map. Read the entrance coordinates with RViz "Publish Point" →
   `/clicked_point`.
2. **Robot Phase 0:** fill in the table in `LIDAR_notes.md`.
   - LiDAR topic, frame and model (the topics were only known as `/ns1/*`).
   - Whether a stale TF exists for the old mount.
   - GX5 model.
   - The real housing measurements.
3. **Robot mapping lap** with `mapping.launch`, recording a bag. Then
   `transit.launch` with 2D Nav Goals only, watching AMCL in the open.
4. **Hand over into REACQUIRE (centering) instead of FOLLOW_ROW.** This is
   a vision_nav change; it absorbs the 0.1–0.2 m handoff offset.
5. **GPS back in** (agreed direction, not started):
   - georeference the SLAM map: record `/gps/fix` during the mapping lap and
     fit map↔lat/lon;
   - take the start pose from RTK;
   - take the row entrance from surveyed lat/lon (`waypoints_maize_gps.yaml`
     format) → map frame → move_base;
   - later, maybe fuse GPS + AMCL so GPS covers open ground.
6. **Not done:**
   - fusing the GX5 yaw rate into the local EKF;
   - driving up the ramp (trailer team).
