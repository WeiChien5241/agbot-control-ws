# GPS testing 11/9 (2026-11-09): first RTK georef of a LiDAR map on the robot

Robot `cpr-j100-0864`, Purdue campus, empty ground (no fake corn set out).
Goal: validate GPS step 1 of `HANDOFF_LIDAR.md`. That means tying RTK lat/lon
to a saved LiDAR map with one rigid fit (`fit_georef.py`), checking the fit
against AMCL, and driving to a lat/lon goal.

## Result in one line

**It works on the real robot.** The fit is 4.6 cm RMS. AMCL and GPS then agreed
to 7.1 cm on a second, independent drive. A lat/lon goal brought the robot back
to its start spot. The remaining problem is that **RTK fix is unreliable**: it
comes and goes, and that is not caused by the LiDAR.

---

## 1. Setup

| item | value |
|---|---|
| Reach M2 mount | module sitting **on top of the Velodyne** |
| antenna lever arm (base_link) | **(-0.125, 0.0) m**: the LiDAR's x/y, from `tf_echo base_link velodyne1` = (-0.125, 0, 0.402) |
| Reach update rate | 5 Hz (already set) |
| `/gps/fix` rate | **1 Hz** (see §5.2) |
| Velodyne | `/ns1/velodyne_points` at 9.9 Hz |
| corrections | NTRIP, age < 2 s (often < 1 s), > 30 satellites visible |

## 2. What was run and what came out

### 2.1 Pre-flight: OK
`/gps/fix` status 2 (RTK fixed), covariance 1e-4 (1 cm σ), LiDAR at 9.9 Hz, TF as above.

### 2.2 Mapping lap: OK, but GPS dropped out halfway
```bash
roslaunch agbot_nav mapping.launch
rosbag record -o ~/bags/georef_map /ns1/velodyne_points /scan /tf /tf_static /odometry/filtered /gps/fix
rosrun agbot_nav save_map.sh georef_test
```
- Bag `georef_map_2026-11-09-15-42-08.bag`: 96 s, 22.5 m of odometry, 50 fixes (45 fixed, 5 float).
- ⚠ **`/gps/fix` stopped at t = 49.4 s** while the robot was still driving, so the
  second half of the lap (11 m) has no GPS. Cause unknown: nobody looked at the
  driver terminal at the time.
- Map `georef_test` (807×1029 cells at 0.05 m, padded to 1007×1229).
  Start pose in the map: **(-21.634, -2.910), yaw -2.3552 rad (-134.9°)**.

### 2.3 Fit: PASS
```bash
rosrun agbot_nav fit_georef.py ~/bags/georef_map_2026-11-09-15-42-08.bag --lever -0.125,0 \
  --out $(rospack find agbot_nav)/maps/georef_test_georef.yaml
```
| lever | RMS | max | p95 | pairs | span |
|---|---|---|---|---|---|
| 0,0 | 0.134 m | 0.220 m | 0.214 m | 24 | 4.6 m |
| **-0.125,0** | **0.046 m** | 0.101 m | 0.075 m | 24 | 4.6 m |

The RMS drops by two-thirds once the lever arm is included, which confirms the
measured mount. **The fit:**
- datum **40.42218631, -86.92023753166667**
- yaw **+43.05°** (map +x points at compass **133.1°**, roughly SE)
- t **(-21.565, -2.838) m**
- map origin (0, 0) at lat 40.42207241, lon -86.92002901

Weakness: the span is only 4.6 m, because of the dropout. That gives roughly
0.5–1° of yaw uncertainty, about 0.2–0.35 m at 20 m.

### 2.4 RTK lost: float, nothing touched
After the fit, RTK fell to float and stayed there. Ruled out:
- corrections: age < 2 s
- sky: > 30 satellites, clear
- update rate: already 5 Hz
- the LiDAR: on 11/6 the same fix-then-float happened with the LiDAR **fully
  removed** and the antenna held up high

Fix came back after **lifting the module up until it fixed, then slowly setting
it back on the LiDAR**. It then held, with float stretches of 5–25 s.

### 2.5 AMCL vs GPS (independent check): PASS
```bash
roslaunch agbot_nav transit.launch rviz:=true \
  map_file:=/home/administrator/agbot_control_ws/src/agbot_nav/maps/georef_test.yaml \
  initial_x:=-21.634 initial_y:=-2.910 initial_yaw:=-2.3552
rosbag record -o ~/bags/georef_amcl /tf /tf_static /gps/fix /amcl_pose /odometry/filtered
```
Bag `georef_amcl_2026-11-09-16-47-17.bag`: 190 s, 191 fixes (140 fixed, 51 float), span 13.5 m.

Below, "AMCL vs GPS" means AMCL's pose plus the lever arm, compared with GPS
pushed through the **mapping** fit from §2.3:

| part of the run | fixed points | AMCL vs GPS |
|---|---|---|
| all except t = 112–146 s | 105 | **RMS 0.071 m, max 0.146 m** |
| t = 112–146 s | 34 | RMS 0.463 m, max 0.86 m |

In the bad window **AMCL was wrong, not GPS**:
- each 1 s GPS step matched odometry to within 3 cm (0.36–0.40 m per second, no jumps)
- AMCL's own spread tripled, from 0.10 to 0.37 m, about 12 m from the start
  (map x ≈ -33 to -26, y ≈ -1 to -8)

A blind refit of this whole bag printed RMS 0.246 m, yaw +44.43°, t (-21.491,
-2.748). Those numbers are skewed only by that window.
**Takeaway: AMCL in open ground is the weak link, and GPS can see it slip.**

### 2.6 Lat/lon goal: robot arrived (precision not measured)
1. A fixed reading near the mapping start spot (40.42218625833333,
   -86.92023792833334) gave map **(-21.586, -2.865)**.
2. The robot was driven 5–10 m away. A goal was sent at
   **x = -21.586 + 0.125 = -21.461, y = -2.865**, facing map +x.
3. The robot drove back on AMCL and stopped "pretty close" by eye.
4. Arrival reading (**float**): 40.42218865833333, -86.92023669833333 = map
   (-21.691, -2.599), **0.29 m** from the goal. That is within float's own
   error plus move_base's 0.12 m goal tolerance, so it is not a real
   precision number.
5. Heading "a bit off": the goal said "face map +x" = compass 133°, while the
   start faced 268° (W). That came from the command, not from a navigation
   error.

GPS is not in the driving loop. That is why float during the drive did not
matter. It **does** matter when reading targets and making fits.

## 3. Numbers to keep

| what | value |
|---|---|
| map | `agbot_nav/maps/georef_test.{yaml,pgm,start_pose}`, on the **robot** (copy on the laptop at `src/georef_test.*`) |
| georef | `agbot_nav/maps/georef_test_georef.yaml` on the **laptop**, copied to the robot |
| datum | 40.42218631, -86.92023753166667 |
| fit | yaw +43.05°, t (-21.565, -2.838), RMS 0.046 m, 24 pairs, span 4.6 m |
| start spot | antenna at about 40.4221863, -86.9202375; robot facing compass ~268° (W); map (-21.634, -2.910, -2.3552) |
| return-to-start goal, start heading | x -21.674, y -2.953, qz -0.9237, qw 0.3831 |
| lever arm | (-0.125, 0.0) m |
| bags (laptop, `~/agbot_control_ws/src/`, untracked) | `georef_map_2026-11-09-15-42-08.bag`, `georef_amcl_2026-11-09-16-47-17.bag` |

These all belong to **this map at this site only**. A new map needs a new fit.

## 4. What failed, and what to type instead

| what was typed | what happened | correct command / fix |
|---|---|---|
| `rosrun agbot_nav ...` in a fresh robot terminal | `package 'agbot_nav' not found` | `cd ~/agbot_control_ws && source devel/setup.bash` first, in **every** terminal |
| `fit_georef.py ... --lever -0.125,0` | `argument --lever: expected one argument` (argparse read `-0.125,0` as a flag) | `--lever=-0.125,0` works everywhere. **Fixed** in `fit_georef.py` today, so both spellings work on the laptop |
| `fit_georef.py ~/bags/georef_amcl_*.bag ... --datum LAT0,LON0` | copied the runbook's placeholders literally | the real bag path, and `--datum 40.42218631,-86.92023753166667` |
| `latlon_to_map.py .../georef_test_georef.yaml` on the **robot** | `FileNotFoundError`: the fit only existed on the laptop | `scp ~/agbot_control_ws/src/agbot_nav/maps/georef_test_georef.yaml administrator@192.168.0.231:agbot_control_ws/src/agbot_nav/maps/` |
| `latlon_to_map.py .../georef_test.yaml` | `KeyError: 'datum'`: that is the **map** yaml, not the georef | pass `georef_test_georef.yaml`. The script now says this plainly (laptop copy) |
| `rostopic pub ... x: <X+0.125>, y: <Y>` | placeholders sent literally, not a valid goal | numbers: `x: -21.461, y: -2.865` |
| first real goal | robot did not move | cause not recorded; a later attempt arrived. If it happens again, check `rosnode list \| grep -E "move_base\|amcl"`, `rostopic echo -n1 /move_base/status` and the transit terminal |

Mistakes on Claude's side, recorded so they are not repeated:
- The first runbook used `<placeholders>` without saying "replace these", and
  did not say the georef yaml has to be copied to the robot.
- It said the Reach was "set to 1 Hz". The Reach was at 5 Hz; see §5.2.
- It kept pointing at the LiDAR after 11/6 had already shown fix-then-float
  without it.
- The goal faced map +x instead of the start heading.

## 5. Open problems

### 5.1 RTK fix-then-float (the blocker)
The same pattern on 11/6 and 11/9: fix for a while, then float that lasts,
whether or not the LiDAR is there and however high the antenna is. Not yet
checked:
- **baseline** to the NTRIP base (> 20 km makes fixing hard)
- whether the **mountpoint's constellations / message types** match what the
  Reach tracks
- **satellites used vs visible** (11/6 in float: 11 of 33 used)
- **elevation mask**

Lifting the module to get a fix, then setting it back, worked today. That hints
that the robot's surroundings matter for *acquiring* a fix (multipath from the
chassis), even if they don't explain *losing* it.

### 5.2 `/gps/fix` is 1 Hz although the Reach is at 5 Hz
`reach_ros_node` publishes once per GGA+GST pair, so the slower sentence sets
the rate. GST is probably 1 Hz. Check: `telnet 192.168.2.15 7777` and count
`$GNGGA` vs `$GNGST` lines per second. Then raise GST's rate in the Reach's
NMEA output, if Emlid Flow offers it.

### 5.3 GPS stopped mid-lap (t = 49.4 s)
Unexplained. The driver logs `Device disconnected. Reconnecting...` on a TCP
drop, so keep its terminal visible.

### 5.4 AMCL drifts in open ground
It did so 12 m from the start today. The planned live **AMCL-vs-GPS monitor**
would flag exactly this. Not written yet.

## 6. Next field run (outside the lab)

### New Reach mount (built 2026-11-11)
A 3D-printed **cap** over the Velodyne replaces the stuck-on housing used on
11/9, which sat lower and had gaps. The Reach module sits about **20 mm above
the LiDAR**, and the antenna about **12 cm above the cap**.
- It should help with *getting* a fix. On 11/9, lifting the module was the
  only thing that brought fix back.
- It probably won't stop fix being *lost* on its own. 11/6 lost fix with no
  LiDAR and the antenna held high, so still do the off-robot checks below.
- **Measure the antenna's x/y from `base_link` again before fitting.** If it
  is on the LiDAR's vertical axis, the lever stays `-0.125,0`. Height does not
  enter the 2D fit, but at about 0.6 m above ground, 5° of tilt moves the
  antenna about 5 cm sideways.
- First on-robot test: park on open ground and check the robot holds status 2
  for 10 minutes with the new cap.

### Before going
1. **Debug RTK on the Reach alone, off the robot** (tripod or box, open ground),
   changing one thing at a time and giving each about 10 min. In Emlid Flow:
   - baseline; try a nearer or VRS mountpoint
   - used vs visible satellites
   - turn off constellations the mountpoint doesn't correct (BeiDou first)
   - elevation mask 15–20°

   Decide on a mount that holds fix for 10+ minutes before using robot time.
2. `telnet 192.168.2.15 7777`: count GGA vs GST per second (§5.2).
3. Bring the latest code over by bundle (`DEPLOYMENT.md`) so the robot has
   today's `fit_georef.py` and `latlon_to_map.py` fixes:
   `git bundle create ~/agbot-20261109.bundle main`.
4. Optional code: the live AMCL-vs-GPS monitor (§5.4).

### At the field: each step has a pass/fail
**In every robot terminal first:** `cd ~/agbot_control_ws && source devel/setup.bash`.

**0. Pre-flight**
```bash
roslaunch reach_ros_node reach_gps_fix.launch          # R1, keep visible
rostopic hz /gps/fix; rostopic echo -n1 /gps/fix       # status 2
rostopic hz /ns1/velodyne_points                       # ~10 Hz
```

**1. Mapping lap.** Mark the start spot and heading.
```bash
roslaunch agbot_nav mapping.launch                                      # R3
rosbag record -o ~/bags/field_map /ns1/velodyne_points /scan /tf /tf_static /odometry/filtered /gps/fix   # R4
rostopic echo /gps/fix/status/status                                    # R2: watch it
```
- Sit still 10 s, then drive a ≥ 15×10 m loop at walking pace with rounded
  turns, and return to the mark.
- **If R2 goes quiet, stop and read R1.**

Save the map:
```bash
rosrun agbot_nav save_map.sh field_test     # write down the transit.launch line it prints
```
Ctrl-C R4, then R3.

**2. Fit (laptop)**
```bash
scp administrator@192.168.0.231:bags/field_map_*.bag ~/bags/
rosrun agbot_nav fit_georef.py ~/bags/field_map_<STAMP>.bag --lever -0.125,0 \
  --out $(rospack find agbot_nav)/maps/field_test_georef.yaml
scp $(rospack find agbot_nav)/maps/field_test_georef.yaml \
  administrator@192.168.0.231:agbot_control_ws/src/agbot_nav/maps/
```
- **Pass:** RMS ≤ 0.05 (≤ 0.10 usable), pairs ≥ 40, span ≥ 10 m.
- Write down the datum, yaw and t.

**3. AMCL check.** Back on the mark, run the transit line from step 1, then:
```bash
rosbag record -o ~/bags/field_amcl /tf /tf_static /gps/fix /amcl_pose /odometry/filtered
```
Drive a different path for 1–2 min. Then on the laptop:
```bash
rosrun agbot_nav fit_georef.py ~/bags/field_amcl_<STAMP>.bag --lever -0.125,0 --datum <DATUM FROM STEP 2>
```
- **Pass:** yaw within 1° and t within 0.10 m of step 2.
- If it fails, check whether AMCL's spread rose in the bad stretch, as on 11/9,
  before blaming the fit.

**4. Lat/lon goal, with a FIXED arrival reading this time**
```bash
rostopic echo -n1 /gps/fix          # status 2; note LAT LON; mark the ground under the module
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/field_test_georef.yaml <LAT> <LON>
# drive 5-10 m away, then, with the NUMBERS from that output:
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: <X + 0.125>, y: <Y>}, orientation: {w: 1.0}}}"
rostopic echo -n1 /gps/fix          # status 2 again, then convert with latlon_to_map.py
```
- **Pass:** within 0.15 m of (X, Y).

**5. Row entrance by lat/lon.** This is the reason for the whole effort. Only do
it once steps 2–4 pass.
- On fix, stand the module at the row entrance and read its lat/lon.
- Get the row's compass bearing (down the row).
```bash
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/field_test_georef.yaml <LAT> <LON> <BEARING> --args
# -> entrance_x:=... entrance_y:=... entrance_yaw_deg:=... row_bearing_deg:=...
roslaunch agbot_nav deploy_mission.launch map_file:=... initial_x:=... initial_y:=... initial_yaw:=... <those args> num_rows:=...
```
Note that `latlon_to_map.py` reports the antenna's point, and the deploy
standoff (0.7 m) is applied on top. A 0.125 m lever is small against that, but
note which spot was surveyed.

At the end, Ctrl-C each terminal (R4, R3, R1). **Never `pkill -f`.**
