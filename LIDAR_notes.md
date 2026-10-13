# LiDAR transit: map + AMCL + move_base (trailer → row → back)

Started 2026-10-13. The professor approved using the robot's 3D Velodyne. It
is plug-and-play: the driver starts at boot on `/ns1/*` (horizontal) and
`/ns2/*` (vertical, not used here).

⚠ This corrects `GPS_testing_10-05.md` §5, which says no LiDAR is mounted.
That was true on 10/6 and is not anymore.

## The approach

- **Transit** (trailer → row entrance, and back) uses the standard ROS stack
  that `GPS_research.md` describes: a 2D map → AMCL → `move_base`. The LiDAR
  replaces GPS as the localizer.
- **In-row** driving stays camera vision nav, unchanged.
- **The GPS stack** (`agbot_gps_nav`) is kept as it is. It is not deleted.

**Why:** AMCL gets heading from scan matching. That is the problem GPS never
solved: no heading at rest (6°/min drift), the bootstrap, and the reverse
bootstrap needed to leave the trailer.

**Why not the colleague's P-AgSLAM:** it is built to localize *inside* the
rows, which our camera already does. We only take its LiDAR network setup
(below) and the idea of recording the map's GPS origin.

**Field layout** (north up): the trailer parks west of the corn, heading
north. The robot reverses off the ramp, still facing north, then drives east
and a little north to the row entrance, which faces east. It drives the rows,
then returns to the point where it left the trailer.

## Robot model (done 2026-10-13)

`agbot_bringup/urdf/agbot_lidar.urdf.xacro`, opt-in with `lidar:=true` on
`agbot_gazebo.launch` / `display.launch.xml`:

| part | geometry | top above deck |
|---|---|---|
| standoff | 10×10 cm, 7 cm tall | 0.07 m |
| housing | 15 cm cube | 0.22 m |
| VLP-16 | optical centre 3.77 cm above its base | **0.258 m** |

- Everything is centred on `mid_mount`. These are rough hand measurements;
  re-measure them and edit the xacro properties.
- The sim LiDAR publishes `/ns1/velodyne_points` in frame `velodyne`, at 10 Hz
  with 440 samples (not 1875, to protect the laptop's RTF).
- The Gazebo plugin needs `sudo apt install ros-noetic-velodyne-gazebo-plugins`.
  It is not installed on the laptop yet, so the plugin has not been spawned in
  Gazebo.
- The URDF itself has been checked: `velodyne` is 0.2577 m above `mid_mount`,
  and the URDF is unchanged when `lidar:=false`.

## Phase 0: check on the robot (needed before any SLAM config)

Record the answers here.

```bash
rostopic list | grep -E "ns1|ns2"                    # exact topic names
rostopic hz /ns1/velodyne_points                     # expect ~10 Hz
rostopic echo -n1 /ns1/velodyne_points/header/frame_id
rosparam list | grep ns1                             # driver params: model, rpm
rosrun tf tf_echo base_link <frame_id from above>    # stale TF from the old mount?
rostopic echo -n1 /gx5/imu/data/orientation          # populated, or all zeros?
rostopic list | grep gx5                             # /gx5/ekf/* = onboard filter
```

- **Model:** read the label. Failing that, 16 distinct `ring` values = VLP-16
  and 32 = HDL-32E. The xacro assumes a VLP-16.
- **GX5:** read the label. A -45 has GNSS/INS; a -25 does not.
- **Network** (from the colleague's README, if the topics are silent):
  `sudo ip addr flush dev br0; sudo ifconfig br0 192.168.1.77;
  sudo route add 192.168.1.201 br0; sudo route add 192.168.1.202 br0`,
  then `ping 192.168.1.201`.

| item | answer |
|---|---|
| topic | |
| frame_id | |
| model | |
| existing base_link → lidar TF | |
| GX5 model / orientation populated | |

## Next phases

1. **`agbot_nav` package**, adapted from `jackal/jackal_navigation`:
   - `pointcloud_to_laserscan`: `/ns1/velodyne_points` → `/scan`, using a
     0.25–1.2 m slice that sits above weeds and catches corn and the trailer.
   - `slam_toolbox` for mapping.
   - `map_server` + `amcl`.
   - `move_base` with `global_planner` + DWA.
   - ⚠ AMCL owns `map → odom`, so `ekf_map` must not run at the same time.
   - The GX5 yaw *rate* goes into the local EKF; absolute yaw is not fused.
2. **Supervisor:** a `transit_backend: gps | move_base` option in
   `mission_supervisor.py` / `handoff_fsm.py`.
   - New states REVERSE_OFF → WAIT_LOCALIZED → TRANSIT → ROW_MISSION →
     RETURN_TRANSIT (back to the pose recorded after leaving the ramp).
   - Driving up the ramp is left out until we talk with the trailer team.
3. **Sim:** the `gps` maize world with `lidar:=true`. Map it, then run
   AMCL + move_base to the corridor-0 entrance, then the full mission.
4. **Robot:** map the trailer area and headland by joystick, test AMCL alone,
   then one row, then the full deploy-and-return run.
   - Watch AMCL in the most open stretch. If nothing is within ~30 m at slice
     height it can drift; GPS is the fallback there.

**Packages to install (laptop + robot):**

```bash
sudo apt-get install -y ros-noetic-velodyne-gazebo-plugins ros-noetic-amcl \
  ros-noetic-map-server ros-noetic-slam-toolbox ros-noetic-pointcloud-to-laserscan \
  ros-noetic-dwa-local-planner ros-noetic-global-planner
```
