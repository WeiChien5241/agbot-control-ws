# DEPLOYMENT.md — running vision-nav on the real P-AgBot

Target: Jackal UGV, ROS1 Noetic (Ubuntu 20.04, Python 3.8), RTX 4080, camera on
`/usb_cam/image_raw/compressed`. Goal: `git clone` → build → drive, keeping the
same codebase working on the laptop sim.

**Sim vs robot is a single launch flag — no file editing:**

```bash
# Real robot (defaults: compressed /usb_cam/image_raw/compressed)
roslaunch agbot_vision_nav vision_nav.launch

# Laptop Gazebo sim (raw /camera/image_raw)
roslaunch agbot_vision_nav vision_nav.launch sim:=true
```

Explicit `camera_topic:=` / `camera_topic_is_compressed:=` overrides still win
over the `sim` flag if the topic ever differs.

---

## 0. Prep on the laptop (the night before)

The robot may not have internet. Build an offline bundle and `scp` it over
(SSH to the robot works, so `scp` works regardless of its internet access):

```bash
mkdir -p ~/robot_wheels

# CUDA torch for the RTX 4080 (Ada, sm_89). torch 2.4.1 is the LAST release
# supporting Python 3.8. cu121 wheels need NVIDIA driver >= 525 on the robot
# (check `nvidia-smi` first; if the driver is older, download cu118 instead).
pip download torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121 -d ~/robot_wheels

# lightly_train + scipy + all their deps (record the known-good laptop versions too)
source ~/agbot_venv/bin/activate
pip freeze > ~/robot_wheels/laptop_freeze.txt
pip download lightly-train "scipy<1.11" -d ~/robot_wheels
deactivate

# Ship the bundle + the model weights (.pt is gitignored — never in the repo)
scp -r ~/robot_wheels <robot>:~/
scp ~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt <robot>:~/
```

If the robot turns out to be online, skip the wheels and `pip install` directly
(same package list, same pins).

---

## 1. Recon on the robot (read-only, do this first)

```bash
rostopic list                                  # expect /usb_cam/image_raw/compressed,
                                               # /odometry/filtered, /cmd_vel
rostopic hz /usb_cam/image_raw/compressed      # camera alive? note the rate
rostopic hz /odometry/filtered                 # REQUIRED for mission mode (GOTCHA:
                                               # without odom the exit detector never arms)
nvidia-smi                                     # driver >= 525 → cu121 wheels OK
python3 --version                              # expect 3.8.x
ls ~/agbot_venv 2>/dev/null                    # a torch venv may already exist — check
                                               # before installing anything
```

## 2. Workspace

```bash
mkdir -p ~/agbot_control_ws/src && cd ~/agbot_control_ws/src
git clone https://github.com/WeiChien5241/agbot-control-ws.git .
# agbot_bringup (sim-only) comes along but is never built — see step 4.
# Papers/ is not in the repo.

# Only if missing (a configured Jackal normally has these):
sudo apt install python3-catkin-tools ros-noetic-cv-bridge
```

## 3. Python venv (mirrors the laptop setup exactly)

```bash
python3 -m venv ~/agbot_venv --system-site-packages   # keeps rospy/cv_bridge importable
~/agbot_venv/bin/pip install --no-index --find-links ~/robot_wheels \
  torch torchvision lightly-train scipy
```

- `--system-site-packages` is what makes this venv ROS-safe: same interpreter
  as system python3.8, just overlaid with torch/lightly_train.
- Do **not** `pip install opencv-python` on top — system `cv2` from ROS must
  win or `cv_bridge` breaks. `opencv-python-headless` pulled in as a
  lightly-train dep is fine.

Quick check:

```bash
~/agbot_venv/bin/python3 -c "import torch, rospy, cv2, scipy, lightly_train; \
print('cuda:', torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# must print cuda: True ... 4080
```

## 4. Build with the venv shebang (the known catkin/shebang fix)

```bash
cd ~/agbot_control_ws
catkin config --cmake-args -DPYTHON_EXECUTABLE=$HOME/agbot_venv/bin/python3
catkin build agbot_vision_nav        # ONLY this package; bringup/jackal-sim not needed
source devel/setup.bash

# Verify the generated wrapper points at the venv:
head -1 devel/.private/agbot_vision_nav/lib/agbot_vision_nav/vision_nav_node.py
# → #!/home/<robot-user>/agbot_venv/bin/python3
```

With the shebang pointing at the venv you never need to `source activate` —
`roslaunch` works from any shell.

Put the model where the launch default expects it:

```bash
cp ~/exported_best.pt ~/agbot_control_ws/src/agbot_vision_nav/config/
```

(If the robot's home dir differs from `/home/chien21`, pass
`model_path:=/absolute/path/to/exported_best.pt` on every launch instead.)

## 5. GPU smoke test — no ROS, no motion

Save one real camera frame, then run the standalone check:

```bash
# grab a frame (any of these works)
rosrun image_view image_saver image:=/usb_cam/image_raw _image_transport:=compressed

cd ~/agbot_control_ws/src/agbot_vision_nav
# edit MODEL_PATH / IMAGE_PATH at the top of smoke_test_segmentation.py, then:
~/agbot_venv/bin/python3 smoke_test_segmentation.py
```

Check: runs on CUDA, overlay image shows green wash on the drivable ground,
note the per-frame inference time (sets your expected /cmd_vel rate).

## 6. Dry run — node live, wheels never commanded

```bash
roslaunch agbot_vision_nav vision_nav.launch cmd_vel_topic:=/vision_nav_check
```

In other terminals:

```bash
rostopic echo /vision_nav_check          # linear.x = 0.15, |angular.z| <= 0.3
rostopic hz /vision_nav_check            # real control rate on the 4080
rqt_image_view /vision_nav_node/debug/image
# (over `ssh -X`, or run rqt on the laptop with ROS_MASTER_URI pointed at the robot)
```

**Sign check** (point the robot so the row corridor sits LEFT of image
center): expect `angular.z > 0` (left turn, REP-103). If the sign is wrong,
stop — do not proceed to live driving.

## 7. Live row-following — 0.15 m/s validated defaults

Drop the `cmd_vel_topic` override so it publishes to `/cmd_vel` (Jackal's
twist_mux keeps joystick/e-stop priority above autonomous cmd_vel):

```bash
roslaunch agbot_vision_nav vision_nav.launch
```

E-stop in hand, joystick ready to override. Watch the debug HUD.

## 8. Speed tuning ladder (this is THE field task — see HANDOFF3)

1. `rostopic hz /cmd_vel` first — know your real control rate.
2. Step `linear_x_cruise` 0.15 → 0.2 → 0.25 → 0.3, scaling `angular_z_max`,
   `delta_angular_z_max`, and `mpc_alpha` **proportionally** with speed:

```bash
roslaunch agbot_vision_nav vision_nav.launch \
  linear_x_cruise:=0.3 angular_z_max:=0.6 delta_angular_z_max:=0.4 mpc_alpha:=0.20
```

3. Read the symptoms: square-wave angular.z at the clamp = too fast for the
   loop rate; growing smooth oscillation = raise `mpc_r_delta` or lower
   `mpc_q_offset`; drifts wide on bends = raise `mpc_q_heading`.

## 9. (Stretch) mission mode — multi-row with headland turns

Only after row-following looks good, and only if `/odometry/filtered` is
publishing (step 1):

```bash
roslaunch agbot_vision_nav vision_nav.launch \
  mission_enabled:=true num_rows:=2 first_turn_direction:=right \
  row_spacing:=0.75          # set to the REAL measured row spacing in meters!
```

`row_spacing` is the traverse distance between rows — measure it in the field;
0.75 m is the sim default. `first_turn_direction` picks the first headland
turn (field on the robot's right at row end → `right`); it alternates
automatically afterwards (boustrophedon).

---

## Troubleshooting quick refs

| Symptom | Check |
|---|---|
| `ImportError: torch` at launch | Shebang didn't regenerate: `catkin clean agbot_vision_nav`, re-run step 4 |
| Node runs, robot never leaves FOLLOW_ROW in mission mode | `/odometry/filtered` missing/renamed → pass `odom_topic:=<actual>` |
| No frames / "Failed to decode CompressedImage" | Wrong topic → `rostopic list`, pass `camera_topic:=...` |
| cv_bridge import error inside venv | A pip opencv shadowed system cv2 → `pip uninstall opencv-python` in the venv |
| Watchdog spams zero Twist | Inference slower than `max_data_age_sec` (0.5 s) — check GPU is actually used (step 3 check) |
