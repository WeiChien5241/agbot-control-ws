# DEPLOYMENT.md — running vision-nav on a real P-AgBot

Target: Jackal UGV, ROS1 Noetic (Ubuntu 20.04, Python 3.8), camera on
`/usb_cam/image_raw/compressed`. Goal: clone → build → drive, keeping the same
codebase working on the laptop sim.

**A GPU is optional.** Step 1 recon decides which profile the robot runs:

- **GPU profile** — NVIDIA GPU with driver ≥ 525: inference in tens of ms,
  10+ Hz control, full speed-tuning ladder (step 8).
- **CPU profile** — no NVIDIA GPU (e.g. the stock Jackal PC, Intel iGPU only):
  same code, inference ~0.5 s/frame ≈ 2 Hz on a Haswell-class Xeon. Live
  row-following works at ≤ 0.15 m/s after the CPU-profile tuning in step 5b.
  Field-validated 2026-07-09 on `cpr-j100-0463` (fake-corn lab row).

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
(SSH to the robot works, so `scp` works regardless of its internet access).

The bundle directory name is arbitrary (`~/robot_wheels`, `~/nav_setup`, …) —
just use the SAME path in every `-d` / `--find-links` below.

```bash
mkdir -p ~/robot_wheels

# CUDA torch (works on CPU-only robots too — CUDA is just unavailable).
# torch 2.4.1 is the LAST release supporting Python 3.8. cu121 wheels need
# NVIDIA driver >= 525 on the robot; if step 1 finds an older driver,
# re-download with cu118 instead. No GPU at all -> keep cu121, it runs on CPU.
pip download torch==2.4.1 torchvision==0.19.1 \
  --index-url https://download.pytorch.org/whl/cu121 -d ~/robot_wheels

# lightly_train + scipy + all their deps (record the known-good laptop versions too)
source ~/agbot_venv/bin/activate
pip freeze > ~/robot_wheels/laptop_freeze.txt
pip download lightly-train "scipy<1.11" -d ~/robot_wheels
deactivate

# GOTCHA: the second download drags in CPU-only duplicates of torch/torchvision
# (plain PyPI wheels, no +cu121). Delete them so pip can never pick the wrong one:
rm -f ~/robot_wheels/torch-*-cp38-cp38-manylinux1_x86_64.whl \
      ~/robot_wheels/torchvision-*-cp38-cp38-manylinux1_x86_64.whl

# Ship the bundle + the model weights (.pt is gitignored — never in the repo)
scp -r ~/robot_wheels <robot>:~/
scp ~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt <robot>:~/
```

**Repo without internet — git bundle.** If the robot is offline, GitHub is
unreachable (and even online, HTTPS git needs a personal access token —
password auth died in 2021). A bundle sidesteps both:

```bash
# laptop
cd ~/agbot_control_ws/src
git bundle create ~/agbot_repo.bundle main
scp ~/agbot_repo.bundle <robot>:~/
```

If the robot turns out to be online, skip the wheels and `pip install`
directly (same package list, same pins) and `git clone` from GitHub.

---

## 1. Recon on the robot (read-only, do this first)

```bash
rostopic list                                  # expect /usb_cam/image_raw/compressed,
                                               # /odometry/filtered, /cmd_vel
rostopic hz /usb_cam/image_raw/compressed      # camera alive? note the rate
rostopic hz /odometry/filtered                 # REQUIRED for mission mode (GOTCHA:
                                               # without odom the exit detector never arms)
nvidia-smi                                     # driver >= 525 -> cu121 wheels OK
lspci | grep -i -E 'nvidia|vga|3d'             # if nvidia-smi is missing: is there
                                               # even an NVIDIA card in this box?
python3 --version                              # expect 3.8.x
ls ~/agbot_venv 2>/dev/null                    # a torch venv may already exist — check
                                               # before installing anything
```

**Branch point:** `nvidia-smi` works and driver ≥ 525 → **GPU profile**.
`lspci` shows no NVIDIA device → **CPU profile** — not a dead end; continue,
and apply step 5b before live driving. (NVIDIA card present but no driver →
install `nvidia-driver-535` + reboot, needs internet, then GPU profile.)

## 2. Workspace

```bash
mkdir -p ~/agbot_control_ws/src && cd ~/agbot_control_ws/src

# online:
git clone https://github.com/WeiChien5241/agbot-control-ws.git .
# offline (bundle from step 0) — note the trailing dot, or it clones into a subdir:
git clone -b main ~/agbot_repo.bundle .
# (later, to re-point a bundle clone at GitHub:
#  git remote set-url origin https://github.com/WeiChien5241/agbot-control-ws.git)

# agbot_bringup (sim-only) comes along but is never built — see step 4.
# Papers/ is not in the repo.

# Only if missing (a configured Jackal normally has catkin + cv_bridge):
which catkin || sudo apt install python3-catkin-tools
python3 -c "import cv_bridge" || sudo apt install ros-noetic-cv-bridge
```

## 3. Python venv (mirrors the laptop setup exactly)

```bash
# venv module is often NOT preinstalled — the first command fails with an
# "ensurepip is not available" error without it:
sudo apt install python3.8-venv
# (offline fallback: python3 -m venv --without-pip --system-site-packages ~/agbot_venv,
#  then use `~/agbot_venv/bin/python3 -m pip` — the system pip — for the installs below)

python3 -m venv ~/agbot_venv --system-site-packages   # keeps rospy/cv_bridge importable

# GOTCHA: the stock pip (20.0.2) predates the manylinux_2_28 wheel tag and will
# report "No matching distribution" for wheels sitting right there in the bundle
# (pyarrow, cryptography, ...). Upgrade pip FIRST:
~/agbot_venv/bin/pip install --upgrade pip

~/agbot_venv/bin/pip install --no-index --find-links ~/robot_wheels \
  torch torchvision lightly-train scipy
```

- `--system-site-packages` is what makes this venv ROS-safe: same interpreter
  as system python3.8, just overlaid with torch/lightly_train.
- Do **not** let a pip OpenCV shadow the system `cv2` from ROS, or `cv_bridge`
  breaks. `opencv-python-headless` gets pulled in as a lightly-train dep —
  **uninstall it after the install** (verified necessary on the robot: the
  pip cv2 5.0 makes every debug-image publish fail with `KeyError: 16`):

```bash
~/agbot_venv/bin/pip uninstall -y opencv-python-headless opencv-python
~/agbot_venv/bin/python3 -c "import cv2; print(cv2.__file__)"
# must point at /usr/lib/python3/dist-packages or /opt/ros/... — NOT ~/agbot_venv
```

Quick check:

```bash
~/agbot_venv/bin/python3 -c "import torch, rospy, cv2, scipy, lightly_train; \
print('cuda:', torch.cuda.is_available())"
# GPU profile: must print cuda: True
# CPU profile: prints cuda: False — expected, carry on
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

Put the model where the launch default expects it (the default is
`$(find agbot_vision_nav)/config/exported_best.pt`, so this works for any
robot user/home dir):

```bash
cp ~/exported_best.pt ~/agbot_control_ws/src/agbot_vision_nav/config/
```

(Weights living elsewhere → pass `model_path:=/absolute/path` on each launch.)

## 5. Inference smoke test — no ROS, no motion

Save one real camera frame, then run the standalone check:

```bash
# grab a frame (any of these works)
rosrun image_view image_saver image:=/usb_cam/image_raw _image_transport:=compressed

cd ~/agbot_control_ws/src/agbot_vision_nav
# edit MODEL_PATH / IMAGE_PATH at the top of smoke_test_segmentation.py, then:
~/agbot_venv/bin/python3 smoke_test_segmentation.py
# NOTE: the overlay is written to src/tmp/ — `mkdir -p ~/agbot_control_ws/src/tmp`
# first, or cv2.imwrite fails silently and no image appears.
```

Check: overlay image shows green wash on the drivable ground. Then measure the
per-frame inference time — it sets your expected /cmd_vel rate and, on the CPU
profile, whether live driving is viable at all:

```bash
cd ~/agbot_control_ws/src/agbot_vision_nav
~/agbot_venv/bin/python3 - <<'EOF'
import os, sys, time, cv2
sys.path.insert(0, "src")
from agbot_vision_nav.segmentation_model import SegmentationModel
model = SegmentationModel(os.path.expanduser(
    "~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt"))
frame = cv2.imread(os.path.expanduser("~/left0000.jpg"))  # your saved frame
model.predict(frame)  # warm-up
N = 10
t0 = time.time()
for _ in range(N):
    model.predict(frame)
dt = (time.time() - t0) / N
print(f"avg inference: {dt*1000:.0f} ms/frame  ->  {1.0/dt:.2f} Hz")
EOF
```

Decision line: **≲ 1 s/frame → live driving is viable at reduced speed** (apply
step 5b). Multiple seconds/frame → dry-run validation only on this machine.
Reference points: RTX-class GPU ≈ tens of ms; Haswell Xeon CPU ≈ 500 ms.

## 5b. CPU profile — retune before any live driving

Three defaults assume a 10+ Hz loop and actively misbehave at ~2 Hz. All are
launch args (also settable in `config/params.yaml`):

```bash
roslaunch agbot_vision_nav vision_nav.launch \
  max_data_age_sec:=3.0 mpc_dt:=0.5
```

- **`max_data_age_sec`** (default 0.5): the watchdog zeroes cmd_vel when the
  newest inference is older than this. At ~0.5 s/frame every frame arrives at
  the deadline → constant stop-go stutter. Raise to 1.5–3.0. Trade-off: this
  is also the max time the robot keeps driving blind after a camera/model
  stall (3.0 s at 0.15 m/s ≈ 45 cm) — tighten it back once the rate is proven.
- **`mpc_dt`** (default 0.1): the MPC's assumed re-steer period. Leaving it at
  0.1 while commands are actually held ~0.5 s makes every correction ~5× too
  strong → slams to the ±0.3 clamp on any offset. Set to the real period
  (1 / inference Hz).
- **cmd_vel keep-alive** (in the node since commit `4151a26`, no knob): the
  node republishes the last command at 10 Hz between inferences. Without it
  the Jackal base brakes whenever cmd_vel goes silent for a few hundred ms —
  publishing only per-inference (2 Hz) makes the robot surge-brake-surge even
  with the watchdog relaxed. If motion stutters despite fresh commands, verify
  the robot is running a node version with this fix (`rostopic hz /cmd_vel`
  should read ~10 Hz, not ~2).

Still overcorrecting/weaving after `mpc_dt` is honest? Tune in this order:
`mpc_q_offset:=5.0` (halve centering aggression) → `mpc_r_delta:=1.0` (damp
command changes) → `linear_x_cruise:=0.1` (less travel per correction).

Speed ceiling on the CPU profile is the sim-validated **0.15 m/s**. The step 8
ladder is GPU-profile only — a faster robot needs a faster loop, not a slower one.

## 6. Dry run — node live, wheels never commanded

```bash
roslaunch agbot_vision_nav vision_nav.launch cmd_vel_topic:=/vision_nav_check
# CPU profile: add max_data_age_sec:=3.0 mpc_dt:=0.5
```

In other terminals:

```bash
rostopic echo /vision_nav_check          # linear.x = 0.15, |angular.z| <= 0.3
rostopic hz /vision_nav_check            # ~10 Hz (keep-alive); inference sets
                                         # how often the VALUES change
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
# CPU profile: add max_data_age_sec:=3.0 mpc_dt:=0.5 (and tuning from 5b)
```

E-stop in hand, joystick ready to override. Watch the debug HUD.

## 8. Speed tuning ladder — GPU profile only (this is THE field task on a GPU robot)

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

GOTCHA: when the mission finishes (`num_rows` reached), the node publishes
zeros forever by design — a robot that "stops for no reason" in mission mode
may simply be DONE. The debug HUD `state=` line tells you.

---

## Troubleshooting quick refs

| Symptom | Check |
|---|---|
| `python3 -m venv` fails: "ensurepip is not available" | `sudo apt install python3.8-venv` (or `--without-pip` + system pip, see step 3) |
| pip: "No matching distribution" for a wheel that IS in the bundle | pip too old for `manylinux_2_28` tags → `pip install --upgrade pip` first |
| `ValueError: Unknown model name or checkpoint path` at launch | Model file not at the path the node got — put weights in `config/` or pass `model_path:=` |
| `ImportError: torch` at launch | Shebang didn't regenerate: `catkin clean agbot_vision_nav`, re-run step 4 |
| `Failed to publish debug image: 16` (debug view empty, control still works) | pip OpenCV shadowing system cv2 → `pip uninstall opencv-python-headless opencv-python` in the venv |
| A `:=` launch override silently does nothing | The arg must exist in `vision_nav.launch` — roslaunch ignores unknown `:=` args without error; check the roslaunch SUMMARY block prints your value |
| cmd_vel alternates real command / zero Twist | Watchdog: inference slower than `max_data_age_sec` → raise it (step 5b) |
| Commands look right but the robot surge-brakes between them | Node predates the 10 Hz keep-alive (commit `4151a26`) → update `scripts/vision_nav_node.py` |
| Robot slams steering to the ±0.3 clamp on small offsets | `mpc_dt` still 0.1 on a slow loop → set to real period (step 5b), then `mpc_q_offset`/`mpc_r_delta` |
| Node runs, robot never leaves FOLLOW_ROW in mission mode | `/odometry/filtered` missing/renamed → pass `odom_topic:=<actual>` |
| Robot stops permanently in mission mode | Mission may be DONE (`num_rows` reached) — check HUD `state=` |
| No frames / "Failed to decode CompressedImage" | Wrong topic → `rostopic list`, pass `camera_topic:=...` |
| Smoke test says "Saved overlay" but no file appears | `src/tmp/` dir missing — `mkdir -p` it; cv2.imwrite fails silently |
| cv_bridge import error inside venv | A pip opencv shadowed system cv2 → `pip uninstall opencv-python` in the venv |
