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

- The stack sits on the centreline, **0.10 m forward of `mid_mount`**: the
  housing's front face is 1 cm behind the front camera body (moved forward
  2026-10-14; it was centred). These are rough hand measurements; re-measure
  them and edit the xacro properties (`lidar_mount_x` and the sizes), then
  `lidar_tf.launch` to match — its test fails until you do.
- The old 20 cm camera stand (`camera_mount_link`) is gone from the URDF.
- The sim LiDAR publishes `/ns1/velodyne_points` in frame `velodyne`, at
  10.0 Hz sim time with full VLP-16 resolution (1875 × 16). The URDF is
  unchanged when `lidar:=false`.
- **Measured in the `gps` world (2026-10-13):**
  - `base_link → velodyne` z = 0.442.
  - RTF 1.0, against 1.2 without the LiDAR.
  - ⚠ **It must be the GPU sensor (`gpu="true"`).** The CPU ray sensor froze
    sim time at 0.11 s, with gzserver pegged on the heightmap and plant
    meshes.
  - From the trailer pose (14.5 m from the nearest plant), 57 points land on
    the corn, all from the two rings just below horizontal.
  - ⚠ **The sim corn is a pessimistic stand-in.** The maize models are about
    0.63 m tall, while real corn is 1–2.5 m. Expect far more returns in the
    field.
  - The ground falls away by about 2° with range, so the slice band has to
    tolerate it.

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
| topic | `/ns1/velodyne_points` at 9.9 Hz (also `/ns1/scan`; a second unit on `/ns2/*`) |
| frame_id | `velodyne1` |
| model | VLP-16 (user, 2026-10-16) |
| existing base_link → lidar TF | **yes**, published by the robot: `[-0.125, 0, 0.402]`, yaw **180°** (xacro guess was `[0.10, 0, 0.4417]`, yaw 0) |
| GX5 model / orientation populated | **3DM-GX5-AHRS** (6253-4220, fw 1.1.76): no GNSS (`gnss_state: Unsupported`), magnetometer present; orientation populated, filter "Solution Valid". A hand rotation read -65.6° → +15.0° yaw (80.6°); `filter_heading_source: 1` (magnetometer on the GX5 line, likely) with `filter_enable_magnetometer_aiding: false`; whether yaw is a true compass heading is unverified (north test pending) |
| second LiDAR | `/ns2/velodyne_points`, frame `velodyne2`, VERTICAL: `[-0.236, 0, 0.292]`, RPY `[-90°, 0, 90°]` |
| robot env | `JACKAL_URDF_EXTRAS=~/catkin_ws/src/pur61_jackal/pur61_jackal_description/urdf/jackal_description.urdf.xacro`; GX5 at `-0.155 -0.07 0.09` from `base_link` |
| nav packages on robot | **none installed** (no slam_toolbox, amcl, map_server, move_base, dwa, global_planner, pointcloud_to_laserscan) |
| 180° yaw check | **confirmed** (2026-10-16): person in front of the robot → −178°, behind → +2° in `velodyne1`, so the sensor faces backwards and the factory yaw 180° is right |
| TF history | the 2024-06-10 bags have `base_link→velodyne1` = `[-0.14, 0, 0.55]` yaw 180°; today it is `[-0.125, 0, 0.402]`, so it was re-measured after the mount changed. Today's z puts the scan centre 0.218 m above the deck (`mid_mount` is 0.184 above `base_link`) |

## What exists (2026-10-13): package `agbot_nav`

| piece | file |
|---|---|
| Velodyne → `/scan` (slice 0.10–1.20 m in `base_link`) | `launch/scan.launch`, `config/scan.yaml` |
| mapping: slam_toolbox (+ optional move_base) | `launch/mapping.launch` |
| map padding for open ground | `scripts/pad_map.py` |
| map_server + AMCL (beam model) | `launch/localization.launch`, `config/amcl.yaml` |
| move_base: GlobalPlanner + DWA | `launch/move_base.launch`, `config/move_base.yaml` |
| the whole deployment | `launch/deploy_mission.launch` → `scripts/deploy_supervisor.py` → `src/agbot_nav/deploy_fsm.py` (rospy-free, tested) |
| real-robot `base_link → velodyne` | `launch/lidar_tf.launch` (a test keeps it equal to the xacro) |
| SIM ONLY: ground-truth map, AMCL-vs-truth logger, trailer box, tall maize, vision stand-in | `scripts/gt_occupancy_map.py`, `scripts/amcl_truth.py`, `launch/sim_trailer.launch`, `models/`, `scripts/vision_standin.py` |

**The deployment** (`deploy_fsm.py`):

```
REVERSE_OFF -> LOCALIZE -> TRANSIT -> ROW_MISSION -> RETURN -> FINISHED
```

- **REVERSE_OFF:** a blind reverse of 1.5 m on wheel odometry.
- **LOCALIZE:** waits until AMCL's spread is ≤ 0.30 m / 10° for 2 s, then
  records that pose as the way home.
- **TRANSIT:** move_base drives to a point 0.7 m *behind* the row entrance,
  on the row's axis.
- **Handoff:** only if AMCL is ≤ 0.15 m / 8° spread *and* within 0.20 m / 12°
  of the corridor axis.
- **ROW_MISSION:** vision nav drives the rows.
- **RETURN:** move_base drives back to the recorded drop-off pose.
- Exactly one node drives on `/cmd_vel` at any time.
- Every timeout, refusal or move_base failure ends in FAILED with nothing
  driving.

**Sim run:**

```bash
rosrun agbot_bringup switch_maize_world.sh gps_tall   # make it first: make_tall_maize_world.sh gps gps_tall
roslaunch agbot_bringup agbot_gazebo.launch lidar:=true x:=9.0 y:=-16.0 z:=0.35 yaw:=3.14159
roslaunch agbot_nav sim_trailer.launch
roslaunch agbot_nav deploy_mission.launch vision_standin:=true
rosrun agbot_nav amcl_truth.py _log:=true > truth.csv     # optional
```

The sim layout is the real field rotated 90°. The rows run north (+y). The
trailer sits south of the field heading west, and the robot starts at the
ramp foot facing it.

## Try it yourself (sim, the `field` world) — 2026-10-14

**Every nav launch now defaults to the `field` world**: its map, its start
pose and its entrance. There is no trailer leg (`reverse_distance` defaults
to 0).

**A. Build your own map**

```bash
rosrun agbot_bringup switch_maize_world.sh field
roslaunch agbot_bringup agbot_gazebo.launch lidar:=true rviz:=false x:=-7.92 y:=-1.30 z:=0.35 yaw:=1.5917
roslaunch agbot_nav mapping.launch rviz:=true          # or slam:=gmapping (sudo apt install ros-noetic-gmapping)
```

- Drive a slow lap with RViz's **drive handle** (drag the arrows around the
  robot). To click goals instead, add `move_base:=true` and use "2D Nav Goal".
- Then save it: `rosrun agbot_nav save_map.sh my_map`. This saves AND pads
  the map.
- The map's (0, 0) is where the robot was when mapping started, facing the way
  it faced.

**B. Navigate on it**

```bash
roslaunch agbot_nav transit.launch rviz:=true map_file:=$(rospack find agbot_nav)/maps/my_map.yaml \
  initial_x:=0 initial_y:=0 initial_yaw:=0
```

- Use "2D Nav Goal" to drive.
- Use "Publish Point" (then `rostopic echo /clicked_point`) to read the map
  coordinates of the row entrance for C.

**C. The full sequence**

```bash
roslaunch agbot_nav deploy_mission.launch vision_standin:=true rviz:=true \
  map_file:=... initial_x:=0 initial_y:=0 initial_yaw:=0 \
  entrance_x:=... entrance_y:=... entrance_yaw_deg:=...
```

Without those arguments it uses the ground-truth `field` map.

## Your own field: `field` world (2026-10-14)

Describe the field as BLOCKS OF ROWS in
`agbot_bringup/config/agbot_field_layout.yaml`: where each block starts,
which way it runs, how many rows, the spacing and the length. Then:

```bash
rosrun agbot_bringup build_field_world.py $(rospack find agbot_bringup)/config/agbot_field_layout.yaml
# -> ~/.ros/virtual_maize_field_snapshots/field/field_preview.png (top-down check)
rosrun agbot_bringup switch_maize_world.sh field
roslaunch agbot_bringup agbot_gazebo.launch lidar:=true x:=-7.92 y:=-1.30 z:=0.35 yaw:=1.5917
roslaunch agbot_nav deploy_mission.launch vision_standin:=true standin_row_distance:=10.7 \
  reverse_distance:=0 map_file:=$(rospack find agbot_nav)/maps/field.yaml \
  initial_x:=-1.2 initial_y:=-4.6 initial_yaw:=1.5708 \
  entrance_x:=-0.5 entrance_y:=0.0 entrance_yaw_deg:=0 rviz:=true
```

The builder writes the AMCL map (`agbot_nav/maps/<snapshot>.yaml`) and
prints the spawn and entrance arguments.

Current layout (2026-10-14, from the user's Gazebo screenshot):
- The robot starts just west of a 1.9 m corridor between two short 3-row
  blocks (4 m long), facing NORTH with the corn on its right. It turns right
  and drives east through the corridor.
- It turns left (north) across a 3 m gap, drives ~1.3 m, then turns right
  into the north block's 3rd/4th-row corridor.
- The north block has 4 east-west rows and the south block 3, 8 m long, with
  a cross alley between them.
- 333 plants, on a flat 20 m square of ground. It was a 48 m square before;
  the ground is now its own heightmap, sized by `ground:` in the layout.

The run below is from the FIRST version: a N-S strip west of the headland,
start 4.7 m south of the entrance, 48 m ground. The current layout has not
been driven yet.

**Full run:**

| | result |
|---|---|
| transit (4.7 m) | 16 s |
| true handoff | 0.03 m off the corridor axis, +4.5° |
| row (perfect-vision stand-in) | within 0.03 m of the centre line, 0 plants knocked over |
| return | 0.22 m / 8° from the start pose |
| AMCL, worst over the run | 0.41 m |

The stand-in now steers from Gazebo ground truth (`standin_oracle`, on by
default). Run blind instead (`standin_oracle:=false`), a 5.6° handoff put it
0.8 m into the corn over 8 m and stopped it inside the row. AMCL then lost the
robot in the flattened plants, and the return failed. Neither happens with a
row mission that keeps the robot centred and exits into the headland, which
is what real vision nav does.

## Sim results (2026-10-13, `gps_tall` world)

**Run 3 completed end to end** (REVERSE_OFF → … → FINISHED), in 3 min of sim
time. Compared with Gazebo ground truth:

| | result |
|---|---|
| drop-off, after AMCL settled | AMCL 0.13 m / +9.6° off |
| transit | 41 s; AMCL worst ~0.95 m mid-headland, 0.12 m at the corn |
| **true handoff pose** | **0.21 m off the corridor axis**, +1.3° (AMCL believed 0.10 m) |
| return | stopped **0.09 m / 2°** from the true drop-off pose |

Run 2's true handoff was 0.03 m / −1.4°. It was refused only because the
heading-spread limit was then 3°, which is why it is now 8°.

⚠ **The handoff position is the weak link.** AMCL is ~0.1–0.2 m off at the
entrance, and a 0.75 m corridor leaves the Jackal ~0.16 m a side. The blind
straight stand-in therefore clipped the plants in run 3. Real vision nav
re-centres, which the stand-in cannot. **Next step: start the handoff in
REACQUIRE's centring phase instead of FOLLOW_ROW** (already a to-do in
`GPS_explained.md` §11 B6). It exists for exactly this offset.

**Findings that shaped the configs** (each is commented where it lives):

- **AMCL must use the beam model** (`amcl.yaml`).
  - The likelihood field aliases along the row. Seeded 0.6 m down the row, it
    stayed 0.81 m off, and once it converged confidently 1.4 m *inside* the
    corridor.
  - The beam model ended 0.08–0.13 m from every start.
- **AMCL is weak in open ground and strong near corn.** At the drop-off the
  unmapped trailer fills the scan. Forcing updates there made the heading
  worse (33°), so LOCALIZE limits are loose and the strict check is at the
  entrance.
- **move_base stops 0.7 m short of the entrance** (`entrance_standoff`). The
  LiDAR marks leaves, which reach ~0.6 m into the headland, so the costmap
  made the 0.5 m entrance itself lethal and GlobalPlanner refused it.
- **Empty beams are filled at 39 m** (`scan.yaml`). Inf, or anything ≥
  range_max, is not raytraced by slam_toolbox at all.
- **SLAM maps only span real returns.** Karto sizes its grid from returns, so
  pad saved maps (`pad_map.py`), and map with a rolling costmap.
- **pointcloud_to_laserscan vs Karto beam count** (`angle_increment`
  0.003491, not 0.0035). Otherwise Karto drops every scan.

**What the sim could NOT test, and why:**

- **Mapping.** Online slam_toolbox went ~10 m wrong on a lap: few plant
  returns in the open, plus the sim's creep below. The runs use a map
  rasterized from `gt_map.csv` instead. **Mapping must be validated on the
  robot**, where tall corn and real odometry are a different problem.
- **Stock maize is invisible to the LiDAR beyond ~12 m.** It is 0.63 m tall
  with the sensor at 0.58 m. `gps_tall` uses our 2× copies (≈1.26 m). The
  camera sees taller plants there too, so a vision result from that world is a
  new result.
- **Ground returns.** The GPU sensor reports the ground too far and too low:
  the rendered terrain disagrees with the collision terrain. Those returns
  never rise into the slice, so the effect is harmless.
- **Creep.** The sim Jackal creeps forward with its wheels stopped, ~0.65 mm/s
  without the LiDAR and ~2 mm/s with it. Odometry cannot see it. This is
  pre-existing; avoid long idle periods in sim tests.
- **Vision nav.** It cannot run on the laptop (no torch or weights).
  `vision_standin.py` drives the row blind and straight.
- **AMCL heading bias.** AMCL reads ~+4–10° at the entrance in sim, where a
  direct scan-to-map fit at the true pose says ~+3.5°. The cause is likely
  stalk-disk map vs leaf returns. A real SLAM map is built from the same
  returns AMCL sees, so this should not carry over, but check it.

## Next steps

1. **Robot:** fill in the Phase 0 table above, then run `lidar_tf.launch`
   only if the TF is missing or stale.
2. **Robot mapping:** joystick a lap of the trailer area and headland with
   `mapping.launch`, without the trailer present. Save the map, then
   `pad_map.py`.
   - Record a bag too (`/ns1/velodyne_points /odometry/filtered /imu/data
     /gx5/imu/data /tf /tf_static`) so mapping can be redone offline.
3. **Robot AMCL alone:** joystick the transit route with `localization.launch`
   and watch the spread in the open stretch.
4. **Handoff into REACQUIRE** (a vision_nav change), then one row on the
   robot, then the full `deploy_mission.launch`.
5. **Not done yet:**
   - fusing the GX5's yaw rate into the local EKF;
   - seeding AMCL's initial pose from RTK GPS plus the trailer heading (today
     it comes from the launch args / RViz 2D Pose Estimate);
   - driving back up the ramp (ask the trailer team).

**Packages to install (laptop + robot):**

```bash
sudo apt-get install -y ros-noetic-velodyne-gazebo-plugins ros-noetic-amcl \
  ros-noetic-map-server ros-noetic-slam-toolbox ros-noetic-pointcloud-to-laserscan \
  ros-noetic-dwa-local-planner ros-noetic-global-planner
```
