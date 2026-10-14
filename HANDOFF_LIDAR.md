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

**Robot model:**
- 15 cm housing on a 7 cm standoff, VLP-16 on top.
- The scan centre is 0.258 m above the deck, 0.10 m forward of the deck
  centre, just behind the front camera.
- The old camera stand is removed.
- The sizes are hand estimates; re-measure.

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

# C. full sequence (no args = ground-truth field map and its entrance)
roslaunch agbot_nav deploy_mission.launch vision_standin:=true rviz:=true
```

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
