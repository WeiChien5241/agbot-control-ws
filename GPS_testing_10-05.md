# GPS testing 10/5 (2026-10-05): first Reach M2 → Jackal test

First hardware test of the GPS stack (GPS_plan.md §5.2, Phase 1). Robot:
`cpr-j100-0864`. Two Reach M2 units, labelled **A** (the first unit connected,
now the base) and **B** (the rover, on the Jackal over USB).

## Result in one line

**The full chain works outdoors with a single fix.** Reach B → USB-Ethernet →
`reach_ros_node` → `/gps/fix` gave `status: 0` and real coordinates on campus
(40.422207, −86.920292, σ ≈ 0.42 m per axis from GST). **RTK was not reached**,
because one antenna cable is damaged (see below), and neither M2 holds INCORS
credentials.

---

## 1. Getting the code and packages onto the Jackal

The robot has no internet, so the code went over by git bundle (HANDOFF3 §0b).

```bash
# laptop
git bundle create ~/agbot-20261005.bundle main
apt-get download ros-noetic-roslint ros-noetic-nmea-msgs python3-serial telnet \
  ros-noetic-robot-localization          # -> ~/jackal_debs/
```

On the Jackal, everything except `ros-noetic-roslint` was already installed
(`python3-serial`, `ros-noetic-nmea-msgs`, `ros-noetic-robot-localization`,
`telnet`). `roslint` is needed to BUILD `reach_ros_node`.

```bash
sudo dpkg -i ~/ros-noetic-roslint_*.deb
cd ~/agbot_control_ws/src
git diff agbot_vision_nav/config/params.yaml    # robot-local edit: turn_rate 0.4,
                                                # yaw_tolerance_deg 5.0 = the 2026-09-15
                                                # revert, already in main, so discarded
git bundle verify ~/agbot-20261005.bundle
git fetch ~/agbot-20261005.bundle main:refs/remotes/bundle/main
git merge --ff-only bundle/main                 # 985020d -> b3b3743
cd ~/agbot_control_ws && catkin build && source devel/setup.bash   # 4/4 packages OK
rospack find reach_ros_node
```

⚠ **The robot's tree has one local edit**, the parser fix in §4, applied by
`sed`. It is identical to commit `9ff4490` on main. Before the next bundle:
`git checkout -- reach_ros_node`, then `git merge --ff-only` goes through.

## 2. Emlid Reach M2 configuration

**As found (both units the same):** firmware **32.2**, receiver name `Reach`
on both, Position streaming 1 = Bluetooth NMEA, Position streaming 2 = TCP
server `localhost:9001` LLH, Correction input = Serial S1 (UART) 38400.
**Neither unit has an NTRIP profile, so the INCORS credentials are NOT on
either M2**, despite what we were told. Unit A serial: `82437E8BCF4E6B20`
(write B's down next time).

**As configured today**, following Emlid's base-rover-over-Wi-Fi guide
(https://docs.emlid.com/reach/quickstart/base-rover-setup/):

| | A, base | B, rover (on the Jackal) |
|---|---|---|
| Name | renamed (was `Reach`) | renamed (was `Reach`) |
| Wi-Fi | client on the iPhone hotspot | client on the iPhone hotspot |
| GNSS | 1 Hz | same constellations as A, 5 Hz |
| Base output | **TCP server, port 9000**, RTCM3 (M2: ARP 0.1 Hz, MSM4 1 Hz, GLONASS biases 0.1 Hz) | — |
| Correction input | off (normal for a base) | **TCP client → A's IP, port 9000** |
| Position streaming 2 | TCP server 7777 NMEA (harmless, left) | **TCP server, port 7777, NMEA, GGA + GST + RMC + VTG** |
| Position streaming 1 | Bluetooth NMEA (unchanged) | Bluetooth NMEA (unchanged) |

Gotchas hit along the way:
- **Two receivers with the same name** → Emlid Flow lists only one of them,
  even though the hotspot shows both connected. Rename them.
- **Emlid Flow cannot see a Reach from the phone that is hosting the hotspot**
  on some phones. Joining the Reach to the hotspot first, then reconnecting,
  worked today.
- **The iPhone hotspot needs "Maximize Compatibility" on**, because the Reach is
  2.4 GHz only.
- **Finding A's IP:** Emlid Flow → A → Wi-Fi, or scan `172.20.10.2–14` (the
  iPhone hotspot range). ⚠ A may get a new IP after a power cycle, and B's
  TCP client address must then be updated.

## 3. Jackal ↔ Reach link (indoors): PASS

```bash
nmcli device | grep -i enx                        # enx46b99a8b969e (name differs per unit)
sudo ip addr add 192.168.2.2/24 dev enx46b99a8b969e
sudo ip link set enx46b99a8b969e up
ping -c3 192.168.2.15                             # 0% loss
telnet 192.168.2.15 7777                          # $GNGGA, $GNRMC, $GNVTG, $GNGST streaming
roslaunch reach_ros_node reach_gps_fix.launch     # terminal 1
rostopic hz /gps/fix                              # 1.0 Hz
rostopic echo -n1 /gps/fix                        # status -1, nan, frame navsat_link
```

- The `ip addr` setting is **lost on unplug or reboot**. Re-run it, or you get
  `[Errno 101] Network is unreachable`.
- `/gps/fix` is 1 Hz because the driver publishes once per GST sentence, and
  the Reach outputs GST at 1 Hz.
- `shutdown request: new node registered with same name` means a second copy
  of the driver was launched. Check `rosnode list | grep reach` first.
- The `invalid checksum '$'` warning appears only at Ctrl-C (a half-received
  line) and is harmless.

## 4. Driver bug found and fixed (commit `9ff4490`)

With no satellites, every GGA arrived empty and the driver logged
`Value error ... invalid literal for int() with base 10: ''` at 5 Hz and
**never published `/gps/fix`**. The cause was that `parser.py` read GGA's
fix-quality field with plain `int()`, while every other field used `safe_int`.
It is now `safe_int`: an empty GGA → 0 → `STATUS_NO_FIX` with NaN position.
`gps_nav_node` (`min_fix_status`) and `navsat_transform` both already reject
that. Losing the fix mid-run therefore reads as "no fix" instead of the topic
going silent.

## 5. Other things found on the robot

- **`/navsat/fix`, `/navsat/heading`, `/navsat/nmea_sentence`** come from the
  **Jackal's built-in consumer GPS chip**: `jackal_node` publishes the NMEA, and
  the stock `nmea_navsat_driver` turns it into a fix. It has never had a fix
  (date `060180` = GPS epoch), and `/navsat/heading` is empty. **Not the Reach,
  and not a heading source.** Our stack uses `/gps/fix` only, so the names do
  not clash.
- **`/gx5/imu/data` at 100 Hz** is a MicroStrain 3DM-GX5 IMU, a second and
  better IMU than the Jackal's own. Note the exact model; a GX5-45 also has
  GNSS/INS. Relevant to the heading work (GPS_plan §5.3).
- **`/ns1/*`, `/ns2/*` Velodyne topics exist.** The project notes say this robot
  has no LiDAR; check whether any LiDAR is actually mounted or whether these are
  leftover drivers. ⚠ **Answered 10/6:** no LiDAR is mounted (only the cameras
  and the Reach). The robot used to carry two Velodynes, and their drivers
  still start at boot and advertise the topics with nothing behind them.

## 6. Outdoor test

**Base/rover RTK: NOT reached. One antenna cable is damaged.**
- First try: base A saw **3 satellites, no solution**, so it sent no
  corrections and rover B sat at "Waiting for corrections" with 26 satellites
  in view.
- After **swapping the antenna cables**, the fault followed the cable: A was
  fine (22 sats, Single) and B now had 22 in view and **no solution**. The
  crooked/damaged cable (or its antenna) is the faulty part, not either
  receiver.
- B still said "Waiting for corrections" with A healthy. Not resolved before
  the lab closed. Check A's averaging status, Base output on, and A's current
  IP in B's TCP client.

**Single fix through ROS: PASS** (good antenna on B):
```
status: 0   latitude: 40.422207203   longitude: -86.920291965   altitude: 163.525
position_covariance: [0.1849, 0, 0, 0, 0.1764, 0, 0, 0, 0.3969]   (type 1, from GST)
```
Status held steady at 0.

**Not done (lab closed):** the 5-minute parked bag, the Stage 5 localization
test, and the correction-link check.

---

## 7. Next steps (tomorrow)

**Before going outside**
1. Photograph the bad antenna/cable and look for a bent centre pin, a crushed
   section or a loose plug. If damaged, order a replacement Emlid helical
   antenna or cable.
2. Ask the previous student for the **INCORS NTRIP address, port, mount point,
   username and password** ("Host IP" is only the address), or ask the advisor
   whether the lab has an INCORS account. With INCORS, **one M2 and one antenna
   give RTK**, and no base is needed. Keep the password out of git.
3. Jackal: run `tmux`, redo `ip addr add` / `ip link set`, then
   `rosnode list | grep reach` and launch the driver once.

**Outside, in this order**
1. `rostopic echo /gps/fix/status` → 0 (or 2 if RTK is working).
2. **The parked bag**, 5 min, robot untouched:
   ```bash
   mkdir -p ~/bags
   rosbag record -O ~/bags/single_static_$(date +%H%M).bag /gps/fix /tcpvel /tcptime \
     /imu/data /gx5/imu/data /odometry/filtered /jackal_velocity_controller/odom /tf /tf_static
   rosbag info ~/bags/single_static_*.bag      # ~300 /gps/fix msgs
   ```
   This measures the single-fix wander and the **real at-rest yaw drift** (the
   17°/min in GPS_plan is only the simulator's parameter).
3. **Stage 5, localization on real data:**
   ```bash
   echo "datum: [40.422207, -86.920292, 0.0]" > ~/lab_datum.yaml   # campus only, NOT gps_datum.yaml
   roslaunch agbot_gps_nav gps_localization.launch datum_file:=$HOME/lab_datum.yaml
   rostopic echo -n1 /odometry/gps          # after ~15 s, near (0,0)
   rosrun tf tf_echo map base_link
   ```
   Joystick about 10 m straight; the map pose should move about 10 m (±1–3 m on
   a single fix). Bag it with `/odometry/gps /odometry/filtered/global` added.
4. **If a second good antenna or INCORS is available:** RTK. Want
   `status: 2`, covariance about 1e-4–1e-3, held ≥ 2 min, then re-bag.
5. **Stage 7 (GPS-driven leg): only with status 2**, with `min_fix_status:=2`,
   open ground, and someone on the deadman.

**Afterwards (laptop)**
- Copy the bags back and analyse the yaw drift and fix wander.
- Record B's serial and the GX5 model here.
- Optional: raise GST to 5 Hz in Emlid Flow, if the message list allows it, for
  a 5 Hz `/gps/fix`.

---

## 10/6 (2026-10-06): found the Reach with NTRIP → RTK fixed, no base needed

A third Reach M2 turned up: the unit the previous student set up, with an
**NTRIP correction profile already on it**. With NTRIP, **one M2 and one
antenna give RTK**. The base (unit A) and the damaged cable are no longer
needed; they are spares.

```
NTRIP caster ──internet──► iPhone hotspot ──Wi-Fi──► Reach (rover, on Jackal)
                                                       │ USB-Ethernet, NMEA :7777
                                                       ▼
                                                 reach_ros_node → /gps/fix
```

The Jackal still needs no internet. No ROS code changed.

### The NTRIP unit, as found

| | |
|---|---|
| Name | `Reach:46:A2` |
| Serial | `82432B838BC746A2` |
| Firmware | 33 |
| Correction input | **NTRIP via Reach**, caster `108.59.49.226` (credentials are on the unit only, keep them out of git) |
| Position streaming 1 | TCP server `localhost:7777`, NMEA ← what `reach_ros_node` reads |
| Position streaming 2 | Serial, USB-to-PC, 38400, NMEA (did not block the USB-Ethernet link) |
| Base output | TCP server 9000, base averaging single 2 min, antenna height 1 m. Left over from use as a base; **turn it off**, a rover does not need it |
| Bluetooth | paired to the previous student's phone (harmless) |

Joined to the iPhone hotspot (Maximize Compatibility on). In Emlid Flow,
outdoors: 26 satellites, PDOP 2.4, **Solution: Fix**, receiving corrections,
age 0.6 s. ⚠ Indoors it sits at "Waiting for corrections": network mount
points need the rover's own position (GGA) first, so no satellites means no
corrections.

Jackal link, same as 10/5 but **the interface name is per unit**
(`enxca6d34048f2b` for this one; `ip -br addr | grep enx` came up `DOWN`):
```bash
sudo ip addr add 192.168.2.2/24 dev enxca6d34048f2b
sudo ip link set enxca6d34048f2b up
ping -c2 192.168.2.15
roslaunch reach_ros_node reach_gps_fix.launch
```

Harmless driver warnings: `ZDA`/`EBP` "not in parse map", and `$GBGSA`/`$GBGSV`
"Regex didn't match". `GB` is the BeiDou talker ID, and the validity regex
(`reach_ros_node/src/reach_ros_node/parser.py:191`) only accepts GP/GA/GN/GL.
Positions come from `$GNGGA` + `$GNGST`, which parse fine. To silence them,
turn GSA, GSV, ZDA and EBP off in Position streaming 1.

### RTK parked bag: PASS (`rtk_static_1513.bag`, 238 s, robot untouched)

The bag is not committed (38 MB); it is on the laptop at `src/rtk_static_1513.bag`.

| | Result |
|---|---|
| `/gps/fix` status | **2 (RTK fixed) on all 237 messages** |
| Rate | 1.0 Hz, largest gap 1.02 s |
| Wander, 1σ | **E 0.44 cm, N 0.31 cm, U 1.25 cm** (10/5 single fix: σ ≈ 42 cm) |
| Largest horizontal deviation | 1.15 cm |
| Reported covariance | 1e-4 m² (σ = 1 cm, slightly cautious, fine for the EKF) |
| Mean position | 40.4222361, −86.9203275, alt 155.30 m |

The altitude is 8 m below 10/5's single fix (163.5 m). The new Reach housing
moves the antenna by centimetres, not metres. An 8 m single-fix vertical error
is normal, plus about a metre if the network's frame is NAD83 rather than
WGS84. 2D navigation ignores altitude (`zero_altitude: true`).

### Heading drift at rest: the real number

Same bag, robot parked, wheels still:

| Source | Drift | Gyro noise (1σ) |
|---|---|---|
| `/odometry/filtered` (stock Jackal EKF) | **+24.3° in 4 min = 6.1°/min** | |
| `/imu/data` (Jackal's built-in IMU), mean gyro z | +0.100°/s = **+6.0°/min** | 0.054°/s |
| `/gx5/imu/data` (MicroStrain 3DM-GX5, frame `gx5_link`), mean gyro z | −0.067°/s = **−4.0°/min** | 0.036°/s |
| `/jackal_velocity_controller/odom` | 0° | |

- The real drift is **6°/min**, not the simulator's 17°/min. All of it is the
  built-in IMU's uncorrected gyro z bias, which the stock EKF integrates
  directly (6.0 ≈ 6.1).
- **The GX5 is an extra, non-stock IMU.** It has lower noise but also a raw
  bias. `/gx5/ekf/status` exists, so it runs its own onboard filter. Check the
  label for the exact model (-25 AHRS / -45 GNSS-INS).
- A gyro bias estimated while parked (zero-velocity update) would remove most
  of this. That is the cheapest next step toward the heading estimator
  (GPS_plan §5.3).

**The heading problem is NOT solved.** RTK makes course-over-ground heading
much sharper once the robot moves (1 cm position noise over 1 m of travel is
about 0.6°), but there is still no absolute heading at boot and the drift at
rest is still unbounded.

### Stage 5, localization on real data: ran, but GPS fusion is NOT confirmed

`gps_localization.launch datum_file:=~/lab_datum.yaml` (datum = 10/5's single
fix) started cleanly. `tf_echo map base_link` while joysticking about 6.7 m
forward and back (not perfectly straight):

- start `(0.000, 0.000)` → farthest `(6.732, −0.126)` → end `(−0.371, −0.458)`, yaw within ±8°.

⚠ **This looks like dead reckoning only, not GPS.** Two signs:
1. The robot was parked near today's bag position, which is **about 4.4 m from
   the datum** (≈ −3.0 m E, +3.2 m N). A fused GPS pose would start near
   `(−3.0, 3.2)`, not exactly `(0, 0)`.
2. At the end the pose holds at exactly `(−0.371, −0.458)` for 6 s. RTK noise
   (σ ≈ 3–4 mm) would move the third decimal.

`navsat_transform`'s "Transform world frame pose" log line does **not** prove a
fix arrived: with `wait_for_datum` the transform comes from the datum alone.
The likely cause is that `/gps/fix` was not being published at that moment
(driver stopped, or the `ip addr` was lost). Recheck **before** driving:
```bash
rostopic hz /gps/fix               # 1 Hz
rostopic echo -n1 /odometry/gps    # should be near (-3, 3) here, NOT (0, 0)
rostopic echo -n1 /odometry/filtered/global
```
Then redo Stage 5 with a bag (`/gps/fix /odometry/gps /odometry/filtered/global
/odometry/filtered /tf /tf_static`), ideally driving out and back to a marked
start point so the end position has a ground truth.

### Next
1. Turn off Base output (and GSA/GSV/ZDA/EBP in Position streaming 1) on `Reach:46:A2`.
2. Redo Stage 5 with `/odometry/gps` confirmed (above).
3. Stage 7, the GPS-driven leg: `min_fix_status:=2`, open ground, someone on the deadman, phone near the robot.
4. Heading: estimate the gyro bias at rest (both IMUs), then revisit dual-antenna (GPS_plan).
5. Optional: stop the Velodyne drivers at boot on this robot.
