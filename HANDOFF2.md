# HANDOFF2.md

Handoff for the P-AgBot simulation bringup work. Written so a fresh Claude Code
session can continue with zero context loss.

---

## 1. GOAL

Build and validate a ROS1 Gazebo simulation environment where the Clearpath
Jackal UGV spawns inside a virtual corn field (virtual_maize_field) with a
simulated forward-facing camera, so the existing `agbot_vision_nav`
row-centering controller can be tested in simulation before being deployed on
the real robot. The immediate milestone is: **camera publishes `/camera/image_raw`
at 30 Hz, robot spawns at the start of a corn row, and the vision-nav node can
drive the Jackal down the row in simulation**.

---

## 2. CURRENT STATE

### Done and committed to GitHub (`WeiChien5241/agbot-control-ws`, branch `main`)

- **`agbot_bringup/` catkin package** fully written — launch file, camera URDF
  xacro, RViz config, shell script for URDF injection. All XML validated.
- **Root cause of camera-not-working bug identified and fixed** (see GOTCHAS #1).
  The fix (`load_robot_description.sh`) was written and committed but **has NOT
  been re-tested yet** — this is the first thing to verify in the next session.
- **Git repo** initialized at `/home/chien21/agbot_control_ws/src/`, pushed to
  GitHub. `git push` works without prompts (gh credential helper configured).
- **`agbot_vision_nav/`** — the controller package — already existed from a
  prior session. 22 unit tests pass. Has not been run against the simulated
  camera yet.
- **CLAUDE.md** updated with git workflow, environment split, and run commands.

### Not yet done / not verified

- Camera publishing (the fix is in place but untested — highest priority).
- Robot spawn position inside a corn row (currently spawns inside plants at
  x=0, y=0 — needs manual tuning).
- Vision-nav node running against the simulated camera end-to-end.
- Camera image orientation (may need rpy adjustment on `base_camera_joint` if
  the image appears rotated in RViz — see NEXT STEPS #3).

---

## 3. KEY DECISIONS

**Do not re-litigate these without new information:**

### Camera URDF injection: `load_robot_description.sh`, not `<env>`
`<env>` tags in ROS1 launch files propagate to `<node>` processes but NOT to
the subprocess created by `<param command="...">`. We discovered this when
`JACKAL_URDF_EXTRAS` was silently ignored by xacro and the camera never
appeared in TF. The fix is a shell script (`scripts/load_robot_description.sh`)
that explicitly `export`s `JACKAL_URDF_EXTRAS` before calling xacro. The
launch file calls this script via `<param name="robot_description" command="..."/>`
**placed AFTER** `<include spawn_jackal.launch>` so our value wins (ROS1
evaluates all `<param>` in document order before starting any node).

### `spawn_jackal.launch`, not `jackal_world.launch`
`jackal_world.launch` starts its own Gazebo instance. We use
`virtual_maize_field/simulation.launch` for Gazebo + `jackal_gazebo/spawn_jackal.launch`
for the robot only. Using `jackal_world.launch` would start a second Gazebo
and both would crash.

### Camera parented to `mid_mount`, not `base_link` or `chassis_link`
`mid_mount` is the Jackal's top-centre accessory plate. Its joint origin is
`xyz="0 0 ${chassis_height}"` = `xyz="0 0 0.184"` above `chassis_link`.
Parenting there keeps camera placement independent of chassis geometry.

### Camera mount box: behind the camera, not below it
Box (20×5×20 cm) centre is at `xyz="-0.10 0 0.10"` relative to `mid_mount`.
Front face at x=0 (flush with camera), body extends to x=-0.20 m (behind).
Camera centre at `xyz="0 0 0.225"` — camera bottom (0.20 m) sits exactly on
box top (0.20 m).

### Camera topic is raw `sensor_msgs/Image`, not CompressedImage
The Gazebo plugin publishes `/camera/image_raw` (raw). The real robot uses
`/usb_cam/image_raw/compressed` (compressed). When running `vision_nav_node`
in simulation, always pass:
```
camera_topic:=/camera/image_raw  camera_topic_is_compressed:=false
```

### Camera optical frame: CV convention separate from camera_link
`camera_link` uses ROS convention (x forward, y left, z up). A separate
`camera_optical_frame` child link uses CV convention (z forward, x right, y
down) via `rpy="-1.5707963 0 -1.5707963"`. The Gazebo plugin's `<frameName>`
points to `camera_optical_frame`. This is standard ROS practice so image
headers have the correct frame for projection math.

### Virtual maize field world is pre-generated
The world at `~/.ros/virtual_maize_field/generated.world` was already generated
by a previous `rosrun virtual_maize_field generate_world.py` call. The launch
file uses it directly. Regenerating produces a new random layout, which changes
the valid spawn positions — only regenerate intentionally.

### Git: `jackal/` and `virtual_maize_field/` are gitignored
They are upstream repos with their own `.git` dirs. They are listed in
`.gitignore` and cloned separately. Any session that re-clones the workspace
needs to clone these two manually (see CLAUDE.md for the commands).

---

## 4. FILES

### `agbot_bringup/` — new package, everything in this session

| Path | Purpose |
|---|---|
| `agbot_bringup/CMakeLists.txt` | Catkin build file (launch-only package, installs launch/urdf/rviz/scripts dirs) |
| `agbot_bringup/package.xml` | Package manifest (format 2); depends on jackal_description, jackal_gazebo, jackal_control, virtual_maize_field, gazebo_ros, rviz, xacro |
| `agbot_bringup/launch/agbot_gazebo.launch` | **Main entry point.** Starts Gazebo (corn world) → spawns Jackal → overrides robot_description with camera → starts RViz. Args: `gui` (bool), `x` `y` `z` (spawn pos), `joystick` (bool). |
| `agbot_bringup/scripts/load_robot_description.sh` | **The key fix.** `$1` = path to camera xacro, `$2` = path to jackal.urdf.xacro. Exports `JACKAL_URDF_EXTRAS=$1` then `exec xacro $2 --inorder`. Called by `<param name="robot_description" command="..."/>` in the launch file. |
| `agbot_bringup/urdf/agbot_camera.urdf.xacro` | Camera + mount stand URDF. Defines: `camera_mount_link` (box stand), `camera_link` (sensor), `camera_optical_frame` (CV frame), Gazebo camera sensor with `libgazebo_ros_camera.so`. Included via `JACKAL_URDF_EXTRAS` into `jackal.urdf.xacro` at line 268. |
| `agbot_bringup/rviz/agbot.rviz` | RViz config: fixed frame=odom, Grid + RobotModel + TF displays, Camera display showing `/camera/image_raw`. |

### `agbot_bringup/urdf/agbot_camera.urdf.xacro` — key dimensions

```
mid_mount (Jackal chassis top-centre, z=0.184 m above chassis_link)
  │
  ├─ camera_mount_joint  xyz="-0.10 0 0.10"
  │    camera_mount_link: box 0.20×0.05×0.20 m  (length×width×height)
  │    NOTE: user swapped length/width in the file (mount_length=0.05, mount_width=0.20)
  │    so the box is actually 5 cm in x and 20 cm in y — visually a narrow slab
  │
  └─ base_camera_joint   xyz="0 0 0.225"  rpy="0 0 0"
       camera_link: box 0.01×0.10×0.05 m
       └─ camera_optical_joint  xyz="0 0 0"  rpy="-1.5707963 0 -1.5707963"
            camera_optical_frame (empty link, CV frame)
```

**Gazebo plugin** (inside `<gazebo reference="camera_link">`):
- `cameraName`: `camera`
- `imageTopicName`: `image_raw`  → publishes `/camera/image_raw`
- `cameraInfoTopicName`: `camera_info`  → publishes `/camera/camera_info`
- `frameName`: `camera_optical_frame`
- Resolution: 640×480, 30 Hz, 80° horizontal FOV

### `agbot_vision_nav/` — pre-existing, not modified this session

| Path | Purpose |
|---|---|
| `agbot_vision_nav/config/params.yaml` | All tunable params: `k_p=1.0`, `k_slope=0.0`, `linear_x_cruise=0.15`, default camera topic `/usb_cam/image_raw/compressed` (override at launch for sim) |
| `agbot_vision_nav/scripts/vision_nav_node.py` | Only file with rospy imports. Single-slot frame buffer + inference thread + 5 Hz watchdog. |
| `agbot_vision_nav/src/agbot_vision_nav/centerline_estimator.py` | Pure numpy. `estimate_centerline(mask) -> CenterlineResult(offset_norm, slope_term, valid, ...)` |
| `agbot_vision_nav/src/agbot_vision_nav/controller.py` | P-controller. `compute(offset_norm, slope_term, valid) -> (linear_x, angular_z)` |
| `agbot_vision_nav/launch/vision_nav.launch` | Requires `model_path` arg (no default). |
| `agbot_vision_nav/test/` | 22 unit tests — run with `PYTHONPATH=src python3 -m pytest test/ -v` from `agbot_vision_nav/` dir. |

### Key Jackal system files (binary package, read-only reference)

| Path | Relevance |
|---|---|
| `/opt/ros/noetic/share/jackal_gazebo/launch/spawn_jackal.launch` | Loads robot_description, controllers, teleop, and calls spawn_model. Does NOT start Gazebo. |
| `/opt/ros/noetic/share/jackal_description/launch/description.launch` | Runs `env_run <config> xacro jackal.urdf.xacro --inorder` to produce robot_description. |
| `/opt/ros/noetic/lib/jackal_description/env_run` | Shell script: `source $1; exec $@` — inherits parent env but `<env>` doesn't reach it from `<param command>`. |
| `/home/chien21/agbot_control_ws/src/jackal/jackal_description/urdf/jackal.urdf.xacro` | Line 268: `<xacro:include filename="$(optenv JACKAL_URDF_EXTRAS empty.urdf)"/>` — the injection hook our script targets. |

### Repo and tooling

| Item | Detail |
|---|---|
| Git repo root | `/home/chien21/agbot_control_ws/src/` |
| GitHub | `https://github.com/WeiChien5241/agbot-control-ws` (private) |
| Default branch | `main` |
| `gh` CLI location | `~/.local/bin/gh` — add to PATH with `export PATH="$HOME/.local/bin:$PATH"` (or add to `~/.bashrc`) |
| Credential helper | Configured in repo-local git config: `credential.helper = ~/.local/bin/gh auth git-credential` |

---

## 5. NEXT STEPS (priority order)

### 1. Verify the camera fix works
```bash
cd ~/agbot_control_ws && catkin build agbot_bringup
source devel/setup.bash
roslaunch agbot_bringup agbot_gazebo.launch
```
In another terminal:
```bash
rostopic hz /camera/image_raw       # should show ~30 Hz
rosrun tf tf_echo base_link camera_link  # should show transform
```
In RViz, the Camera display should show a corn field image. If it still shows
"no publishers", check the `robot_description` param:
```bash
rosparam get /robot_description | grep camera_link
```
If `camera_link` is absent, the script isn't running — check that
`agbot_bringup/scripts/load_robot_description.sh` is executable (`chmod +x`).

### 2. Fix camera image orientation (if needed)
If the image in RViz is rotated 90° or upside-down, adjust the `rpy` on
`base_camera_joint` in `agbot_camera.urdf.xacro`. Common fix if camera looks
sideways: change `rpy="0 0 0"` to `rpy="0 -1.5707963 0"` (tilt down 90° to
look forward along Gazebo's convention). Re-build and re-launch to test.

### 3. Tune robot spawn position
The default `x=0.0 y=0.0` spawns inside corn plants. Find a clear start
position by checking the world layout:
```bash
cat ~/.ros/virtual_maize_field/gt_map.csv   # row centreline coordinates
```
Then override at launch:
```bash
roslaunch agbot_bringup agbot_gazebo.launch x:=-2.0 y:=0.0
```
Try negative x values first (field tends to extend in +x, so -x is before the
first row). Update the default in `launch/agbot_gazebo.launch` once found.

### 4. Run vision-nav node against the simulated camera
With the bringup launch running:
```bash
roslaunch agbot_vision_nav vision_nav.launch \
  model_path:=/path/to/exported_best.pt \
  camera_topic:=/camera/image_raw \
  camera_topic_is_compressed:=false
```
Watch `/cmd_vel` output:
```bash
rostopic echo /cmd_vel
```
For visual debugging:
```bash
rqt_image_view /vision_nav_node/debug/image
```
Expected: `angular.z` should be small and low-variance when the robot is
centred; should react in the geometrically correct direction when drifted.

### 5. Tune `k_p` gain
Default is `1.0` in `agbot_vision_nav/config/params.yaml`. Once closed-loop
behavior is observable, tune this empirically. Change the value in params.yaml
and re-launch the vision_nav node (no rebuild needed — it's a ROS param).

### 6. Commit and push after each step
```bash
git add <changed files>
git commit -m "Descriptive message"
git push
```

---

## 6. GOTCHAS

### GOTCHA 1: `<env>` does not work for `<param command="...">` in ROS1
**Already hit once — do not try again.** The `<env name="JACKAL_URDF_EXTRAS" value="..."/>` approach
in the launch file sets env vars for `<node>` processes only, not for the
subprocess spawned by `<param command="...">`. The xacro run by
`description.launch` uses `<param command="env_run ...">`, so `JACKAL_URDF_EXTRAS`
was silently ignored and xacro fell back to `empty.urdf`. **Fix is
`load_robot_description.sh`.** Do not revert to the `<env>` approach.

### GOTCHA 2: IDE creates `agbot_gazebo.launch.xml` alongside `agbot_gazebo.launch`
VS Code's ROS extension renames `.launch` to `.launch.xml` for XML highlighting.
This creates a duplicate that confuses the build. Always delete `.launch.xml`:
```bash
rm agbot_bringup/launch/agbot_gazebo.launch.xml
```
and commit only `agbot_gazebo.launch`. `roslaunch` needs the `.launch` extension.

### GOTCHA 3: `jackal_gazebo` and `jackal_viz` are NOT in the source tree
These are binary system packages at `/opt/ros/noetic/share/jackal_gazebo/` and
`/opt/ros/noetic/share/jackal_viz/`. Do not `find` them in the `src/` workspace.
The source workspace has `jackal_description`, `jackal_control`, `jackal_msgs`,
`jackal_navigation`, `jackal_tutorials` — but NOT `jackal_gazebo` or `jackal_viz`.

### GOTCHA 4: `spawn_jackal.launch` sets `robot_description` without camera
`spawn_jackal.launch` → `description.launch` sets `/robot_description` to the
base Jackal URDF (no camera). Our `<param name="robot_description" command="...load_robot_description.sh..."/>` in `agbot_gazebo.launch` **must come
AFTER** `<include spawn_jackal.launch>` to override it. If you move it before
the include, the spawn_jackal include will overwrite it and the camera disappears.

### GOTCHA 5: Virtual maize field world must be pre-generated
`~/.ros/virtual_maize_field/generated.world` must exist before launching.
If it doesn't:
```bash
rosrun virtual_maize_field generate_world.py
```
Regenerating changes the random layout and invalidates any hardcoded spawn
position you've tuned.

### GOTCHA 6: Camera mount_length and mount_width are swapped in the xacro
The user edited `agbot_camera.urdf.xacro` and swapped the values:
```xml
<xacro:property name="mount_length" value="0.05"/>  <!-- actually the x dimension -->
<xacro:property name="mount_width"  value="0.20"/>  <!-- actually the y dimension -->
```
So the physical box in Gazebo is 5 cm in x (short, narrow) and 20 cm in y
(wide). If you want to change the box shape, these are the values to edit.
The overall design intent (narrow pillar supporting the camera) is preserved.

### GOTCHA 7: `gh` is not on system PATH by default
`gh` is at `~/.local/bin/gh`. Every session needs `export PATH="$HOME/.local/bin:$PATH"`.
Add to `~/.bashrc` to make it permanent:
```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
```

### GOTCHA 8: This sandbox has ROS2, not ROS1
The Claude Code sandbox (Ubuntu where this file lives) has ROS2 Humble. The
user's WSL machine (Ubuntu 20.04) has ROS1 Noetic. All `catkin`, `roslaunch`,
`rostopic`, and `rosbag` commands must be run by the user on their machine —
they cannot be executed or verified in this sandbox. Do not try to run them
here.

### GOTCHA 9: Model weights are not in the repo
`exported_best.pt` (~89 MB) is excluded by `.gitignore`. It lives at
`DINOv3-Segmentation-Training/out/corn_field_navigation/exported_models/exported_best.pt`
on the user's machine. The `model_path` arg to `vision_nav.launch` must be an
absolute path to wherever it actually is.

### GOTCHA 10: `rosparam get /robot_description` is the ground-truth check
If the camera is not showing up in TF or Gazebo, always check the actual URDF
that was loaded:
```bash
rosparam get /robot_description | grep -c camera_link
# Should print 1 (or more). If 0, the injection didn't work.
```
