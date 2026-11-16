# Field runbook 11/17 (2026-11-17): full workflow on fake corn, lat/lon entrance

Robot `cpr-j100-0864`, outside the lab, fake corn rows set out.
Goal: run the **whole workflow** for the first time on the robot:

> start spot → (AMCL + move_base) → row entrance given as **lat/lon** →
> vision nav drives the rows → move_base back to the start spot

No RViz clicks pick the entrance. It comes from two RTK fixes taken in the row.
On 11/11 the steps before this all passed: fit RMS 8.1 cm, AMCL check, and a
lat/lon goal reached to 6.8 cm (`GPS_testing_11-11.md`). `deploy_mission.launch`
has only ever run in **sim**. Today is its first run on the robot.

## At a glance

**Order of the day**
1. **Tonight:** a 10-second check that the robot has the model weights and the deploy args (§2). No bundle is needed.
2. **Field:** pre-flight (§4) → survey the entrance (§5) → mapping lap (§6) → fit on the laptop (§7).
3. **Check the new map:** AMCL check, then a lat/lon goal to the start spot (§8).
4. **Vision check:** one corridor by itself (§10). The model has never been checked on the fake corn.
5. **Full run:** `deploy_mission.launch` with `num_rows:=1`, then with all corridors (§11).

**The lat/lon readings.** Read every one with `/gps/fix` on status 2 (RTK fixed).

| placeholder | where the robot is when you read it |
|---|---|
| `⟨ENT_LAT⟩ ⟨ENT_LON⟩` | **inside** the chosen corridor, at its mouth: centred between the two plant rows, facing down the row, robot centre about level with the first plants |
| `⟨FAR_LAT⟩ ⟨FAR_LON⟩` | **out the far end** of the same corridor: driven straight through it and stopped just past the last plants, in open headland, still on the corridor's line and facing the same way. Only used to get the row's direction |
| `⟨START_LAT⟩ ⟨START_LON⟩` | on the taped start mark, facing the arrow, right after the mapping lap |
| `⟨ARR_LAT⟩ ⟨ARR_LON⟩` | where the robot stopped after the lat/lon goal test (§8.2) |
| `⟨HOME_LAT⟩ ⟨HOME_LON⟩` | where the robot stopped after a deploy run came home (§11) |

**What gets copied**
- **Robot → laptop:** the mapping bag and `corn_1117.{yaml,pgm,start_pose}` straight after mapping (§7.1). The AMCL-check bag (§8.1). At the end of the day, all of the day's bags and the vision-nav CSVs from `~/agbot_logs` (§12).
- **Laptop → robot:** only the georef, as a backup (§7.5). No bundle (§2). Every `fit_georef.py` and `latlon_to_map.py` command runs on the laptop.

**What could go wrong**
- `deploy_mission.launch` has only run in sim. Both passes use `transit_speed:=0.3`, and the second also uses `traverse_speed:=0.3`.
- The handoff into the row is refused if AMCL's spread is over 0.15 m. On 11/11 it was 10–19 cm while driving in the open. If it refuses, write the numbers down and retry once.
- The fake corn's wooden bases are invisible to the LiDAR within about 1.5 m. Keep the joystick in hand near the corn.
- `latlon_to_map.py --toward` (the row direction from the two in-row fixes) is new and has not been used in the field yet.

---

## 0. Names used today (all new, nothing collides with 11/9 or 11/11)

| thing | name |
|---|---|
| map | **`corn_1117`** → `agbot_nav/maps/corn_1117.{yaml,pgm,start_pose}` |
| georef (fit) | **`corn_1117_georef.yaml`** (same name as the map + `_georef`) |
| mapping bag | `~/bags/mapping_2026-11-17-HH-MM-SS.bag` (named by `mapping.launch`) |
| AMCL-check bag | `~/bags/corn_1117_amcl_<stamp>.bag` |
| deploy bags | `~/bags/corn_1117_deploy_<stamp>.bag` |

If you have to map again, use **`corn_1117b`**, then `corn_1117c`, and so on,
with a matching `corn_1117b_georef.yaml`. **A georef belongs to exactly one
map.** Write the map name in the notes sheet (§13) next to every number.

Placeholders are written **`⟨LIKE_THIS⟩`**. Replace the whole thing,
brackets included, with the number. Everything not in `⟨ ⟩` is typed
exactly as shown.

## 1. Terminals

`R` = a terminal ssh'd into the robot (`ssh administrator@192.168.0.231`).
`L` = a terminal on the laptop.

**In every R terminal, first:**
```bash
cd ~/agbot_control_ws && source devel/setup.bash
```
**In every L terminal, first:**
```bash
cd ~/agbot_control_ws && source devel/setup.bash
```

| terminal | what runs in it | how long |
|---|---|---|
| **R1** | Reach driver | **all day**, keep visible |
| **R2** | cameras (front WDR + rear Brio) | **all day** from step 3 on |
| **R3** | the main launch. It changes per step: mapping → transit → vision check → deploy | Ctrl-C between steps (said each time) |
| **R4** | `rosbag record`, `save_map.sh` | per step |
| **R5** | one-off checks (`rostopic echo -n1 /gps/fix`, etc.) | as needed |
| **L1** | laptop: `scp`, `fit_georef.py`, `latlon_to_map.py` | as needed |

⚠ Only **one** of mapping / transit / deploy may run in R3 at a time. Each one
owns `map → odom`, and two of them fight. **Always Ctrl-C R3 and wait for it
to finish before starting the next launch there.**

⚠ Never clean up with `pkill -f`. Ctrl-C each terminal.

---

## 2. Tonight (robot, in the lab): no code update needed

The robot already has every file it runs tomorrow. It ran `mapping.launch
record:=true` on 11/11, and that arrived in commit `aedc7e2` (10/26). The
deploy files (`deploy_mission.launch`, `deploy_supervisor.py`,
`deploy_fsm.py`) were last changed on 10/21, before that. Everything changed
since then is a laptop-only script (`fit_georef.py`, `latlon_to_map.py`, which
now has `--toward`) or the sim-only trailer model. **No bundle.**

### 2.1 Confirm in 10 seconds
R1:
```bash
ls -lh ~/agbot_control_ws/src/agbot_vision_nav/config/exported_best.pt   # the model; must exist
roslaunch agbot_nav deploy_mission.launch --ros-args | grep -E "entrance_x|row_bearing|plan_on_map"
```
All three args listed means the robot's deploy code is the current one. If any
are missing, bundle it over (the flow is in `GPS_testing_10-05.md` §1).

### 2.2 Pack
Tape and marker for the start spot, a measuring tape (row spacing), the
joystick (charged), the robot battery (charged), the phone/hotspot for
corrections, this sheet.

---

## 3. Site setup (before powering anything)

```
                       ≥ 2.5 m clear headland
            ┌────────────────────────────────────┐
  plant row │ ●  ●  ●  ●  ●  ●  ●  ●  ●  ●  ●   │
            │            corridor 1  →           │   ← 0.75 m between plant rows
  plant row │ ●  ●  ●  ●  ●  ●  ●  ●  ●  ●  ●   │
            │            corridor 2  →           │
  plant row │ ●  ●  ●  ●  ●  ●  ●  ●  ●  ●  ●   │
            └────────────────────────────────────┘
       ≥ 2.5 m headland        ≥ 1.5 m clear all round (the way home)

                  8–15 m of open ground

                    ✚ START SPOT (tape cross + arrow for the heading)
```

- **Row spacing 0.75 m**: vision nav is tuned for it. Rows as straight as you
  can make them.
- At least **3 plant rows** = 2 corridors.
- **≥ 2.5 m of clear headland at both ends.** The exit leg runs up to 1.5 m,
  and then the robot turns.
- **≥ 1.5 m clear all round the block.** After the rows, move_base drives home
  around it.
- **Start spot 8–15 m from the row mouth**, across open ground, so the transit
  is real. Tape a cross, and an arrow for the heading. Every launch today
  starts with the robot **on this mark, facing the arrow**.
- ⚠ The fake corn's ~10 cm wooden bases are **invisible to the LiDAR** within
  ~1.5 m (HANDOFF_LIDAR, 10/20). The stalks are seen. Keep the joystick in
  hand whenever the robot is near the corn.

---

## 4. Pre-flight (robot on, start spot)

R1 (keep it visible all day):
```bash
roslaunch reach_ros_node reach_gps_fix.launch
```
R2 (all day):
```bash
roslaunch agbot_vision_nav cameras.launch
```
R5:
```bash
rostopic echo -n1 /gps/fix | grep -A1 "status:"   # status: 2 (RTK fixed)
rostopic hz /gps/fix                               # ~1 Hz. Ctrl-C after a few lines
rostopic hz /ns1/velodyne_points                   # ~10 Hz. Ctrl-C
rostopic hz /usb_cam/image_raw/compressed          # front camera. Ctrl-C
rostopic hz /brio_rear/image_raw/compressed        # rear camera. Ctrl-C
rostopic hz /odometry/filtered                     # Ctrl-C
```
**Pass:** status 2, and every topic has a rate. Park on fix for **2 minutes**
before moving on. If it drops, see §9.

---

## 5. Survey the row entrance (only R1 is needed)

This only needs lat/lon, not the map, so do it now while RTK is fixed.

1. Pick the corridor to enter, normally an **outer** one. Look **down the
   row** from the mouth:
   - if the other corridors are to your **right** → `first_turn_direction:=right`
   - if they are to your **left** → `first_turn_direction:=left`

   Write it down. (Turning the wrong way drives out of the block.)
2. Joystick the robot into that corridor's mouth: **centred between the two
   plant rows, facing down the row, robot centre about level with the first
   plants**. Stop. Wait 3 s. Then, in R5:
   ```bash
   rostopic echo -n1 /gps/fix | grep -E "status: |latitude|longitude"
   ```
   status must be **2**. Write latitude → **`⟨ENT_LAT⟩`** and longitude →
   **`⟨ENT_LON⟩`**.
   If it won't reach status 2 within ~30 s in the mouth (corn shading the
   antenna), back it straight out ~0.5 m into the headland, still on the
   corridor's line, and read it there. The entrance then sits 0.5 m earlier,
   which is harmless: move_base stops 0.7 m before it, and vision nav drives
   the rest.
3. Joystick it **straight through the whole corridor and out the far end**.
   Stop just past the last plants, in open headland, without turning, so it
   is still on the corridor's line and facing the same way. Wait 3 s, run the
   same command, status **2**. Write **`⟨FAR_LAT⟩`**, **`⟨FAR_LON⟩`**. Out in
   the open the sky is clear, and the two fixes are a whole row length apart,
   which gives a better bearing than 2 m would. If it lost fix in the corridor,
   wait at this spot until it's back on 2.
4. Measure: the row length, the corridor number counted from the side you
   entered, and the number of corridors. Then joystick back out to the start
   spot.

`⟨ENT_*⟩` and `⟨FAR_*⟩` are the **antenna's** positions. Both are taken with
the robot facing down the row, so the script turns them into base_link and the
row axis itself (§7.3).

---

## 6. Mapping lap → map `corn_1117`

Robot **on the start mark, facing the arrow**.

R3:
```bash
roslaunch agbot_nav mapping.launch record:=true
```
R4 (watch it during the lap; it prints a number every second):
```bash
rostopic echo /gps/fix/status/status
```
Drive:
- Sit still **10 s**.
- Walking pace, rounded turns. Drive to the corn and go **all the way round
  the block** along the headlands, not through the corridors. Then come back
  to the start mark and stop facing the arrow.
- **Want ≥ 10 m of span and most of the lap on 2.** If R4 shows 0 or 1 for
  more than a few seconds, note the time and read R1 (§9).

Save, still in R4 (Ctrl-C the status echo first):
```bash
rosrun agbot_nav save_map.sh corn_1117
```
**Copy the last line it prints** (`roslaunch agbot_nav transit.launch ...
initial_x:=… initial_y:=… initial_yaw:=…`) into the notes sheet. These are
**`⟨SX⟩ ⟨SY⟩ ⟨SYAW⟩`**, used by every later launch.

Then **Ctrl-C R3** (mapping; this also closes the bag). Find the bag's name in R4:
```bash
ls -lht ~/bags | head -3          # newest first: mapping_2026-11-17-HH-MM-SS.bag
```
Write it down as **`⟨MAP_BAG⟩`** (the file name only).

While the robot is still on the mark, facing the arrow, R5:
```bash
rostopic echo -n1 /gps/fix | grep -E "status: |latitude|longitude"
```
status **2** → **`⟨START_LAT⟩ ⟨START_LON⟩`**. This is the target for the
lat/lon goal test in §8.2.

---

## 7. Fit on the laptop → `corn_1117_georef.yaml`

### 7.1 Copy robot → laptop
L1:
```bash
mkdir -p ~/bags
scp administrator@192.168.0.231:bags/⟨MAP_BAG⟩ ~/bags/
scp administrator@192.168.0.231:agbot_control_ws/src/agbot_nav/maps/corn_1117.{yaml,pgm,start_pose} \
    $(rospack find agbot_nav)/maps/
```

### 7.2 Fit
L1:
```bash
rosrun agbot_nav fit_georef.py ~/bags/⟨MAP_BAG⟩ --lever=-0.125,0 \
  --out $(rospack find agbot_nav)/maps/corn_1117_georef.yaml
```
| | pass | usable | 11/11 |
|---|---|---|---|
| residual RMS | ≤ 0.05 m | ≤ 0.10 m | 0.081 |
| pairs used | ≥ 40 | | 113 |
| span | ≥ 10 m | | 16.7 m |

Write down: RMS, pairs, span, yaw, t. **RMS > 0.10 → map again (`corn_1117b`).**

### 7.3 Turn the surveyed entrance into launch args
L1 (all four numbers come from §5):
```bash
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/corn_1117_georef.yaml \
  ⟨ENT_LAT⟩ ⟨ENT_LON⟩ --toward ⟨FAR_LAT⟩ ⟨FAR_LON⟩
```
It prints:
```
row axis from 2 fixes N.NN m apart: compass BBB.B deg; entrance moved by the lever arm ...
map (X, Y), facing D deg (compass BBB.B)
fit RMS ...
entrance_x:=… entrance_y:=… entrance_yaw_deg:=… row_bearing_deg:=…
```
**Copy the whole last line.** That is **`⟨ENTRANCE_ARGS⟩`**. Sanity check: the
compass bearing should match the row direction you saw (e.g. rows running
roughly east = ~90°).

### 7.4 Start-spot goal (for the lat/lon check in §8)
`⟨START_LAT⟩ ⟨START_LON⟩` were read at the end of §6. L1:
```bash
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/corn_1117_georef.yaml \
  ⟨START_LAT⟩ ⟨START_LON⟩ --goal-yaw ⟨SYAW⟩
```
Copy the `rostopic pub -1 /move_base_simple/goal ...` line it prints:
**`⟨START_GOAL_CMD⟩`**. Also write down the `antenna at map (…, …)` numbers:
**`⟨START_MAP_X⟩ ⟨START_MAP_Y⟩`**.

### 7.5 Copy laptop → robot (backup only)
The robot's launches don't read the georef; every lat/lon conversion runs on
the laptop. Keep a copy on the robot anyway:
```bash
scp $(rospack find agbot_nav)/maps/corn_1117_georef.yaml \
    administrator@192.168.0.231:agbot_control_ws/src/agbot_nav/maps/
```

---

## 8. Check the new map before trusting it (transit.launch)

Robot **on the start mark, facing the arrow**. R3 is free (mapping was stopped in §6).

R3:
```bash
roslaunch agbot_nav transit.launch rviz:=true \
  map_file:=/home/administrator/agbot_control_ws/src/agbot_nav/maps/corn_1117.yaml \
  initial_x:=⟨SX⟩ initial_y:=⟨SY⟩ initial_yaw:=⟨SYAW⟩
```
(This is the same line `save_map.sh` printed. You can paste that one.)

R4:
```bash
rosbag record -o ~/bags/corn_1117_amcl /tf /tf_static /gps/fix /amcl_pose /odometry/filtered
```

### 8.1 AMCL check (1–2 min)
Joystick a path that differs from the mapping lap, out to the corn headland
and back. Check RViz: the scan should sit on the map, with no jumps. Stop
back near the start. **Ctrl-C R4** (the bag). Then L1:
```bash
scp administrator@192.168.0.231:bags/corn_1117_amcl_*.bag ~/bags/
DATUM=$(python3 -c "import yaml;d=yaml.safe_load(open('$(rospack find agbot_nav)/maps/corn_1117_georef.yaml'));print('%.10f,%.10f'%tuple(d['datum']))")
echo $DATUM
rosrun agbot_nav fit_georef.py ~/bags/corn_1117_amcl_⟨STAMP⟩.bag --lever=-0.125,0 --datum=$DATUM
```
(No `--out` here. It must **not** overwrite the georef.) The `$DATUM` line
gets the real datum from the yaml. On 11/11 the map-origin line was pasted by
mistake.

**Pass:** yaw within **1°** and t within **0.10 m** of §7.2.

### 8.2 Lat/lon goal to the start spot (repeat of 11/11 on the new map)
Joystick the robot 5–10 m away from the start mark. Release the joystick. R5:
```bash
⟨START_GOAL_CMD⟩            # the rostopic pub line from §7.4, pasted as is
```
When it stops, R5:
```bash
rostopic echo -n1 /gps/fix | grep -E "status: |latitude|longitude"   # status 2 → ⟨ARR_LAT⟩ ⟨ARR_LON⟩
```
L1:
```bash
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/corn_1117_georef.yaml ⟨ARR_LAT⟩ ⟨ARR_LON⟩
python3 -c "import math; print(math.hypot(⟨ARR_MAP_X⟩-(⟨START_MAP_X⟩), ⟨ARR_MAP_Y⟩-(⟨START_MAP_Y⟩)))"
```
`⟨ARR_MAP_X⟩ ⟨ARR_MAP_Y⟩` are the `map (…, …)` numbers from the first line.
**Pass:** ≤ 0.15 m (11/11: 0.068).

**Then Ctrl-C R3 (transit).** The deploy launch starts its own AMCL and move_base.

---

## 9. RTK trouble at any point

Look at the status number before doing anything else:

| status | meaning | do |
|---|---|---|
| 2 | RTK fixed | carry on |
| 1 | float: corrections arrive, the Reach can't resolve | wait ≤ 2 min (11/11 recovered by itself). Don't take survey readings |
| 0 | **no corrections at all** | check the correction link (phone hotspot / Wi-Fi / Emlid Flow). **Write down the time and Emlid Flow's correction age.** This is open item 1 of 11/11 |
| R1 prints `Device disconnected` | the Reach's TCP dropped | wait for the reconnect; write down the time |

GPS is **not** in the driving loop (AMCL is). A drop while driving changes
nothing. It only matters for the survey, the fit, and the arrival readings.

---

## 10. Vision check in one corridor (≈ 3 min, before the full run)

The model was trained on real corn. **Find out whether it reads the fake corn
before the deploy depends on it.** R3 is free.

Joystick the robot into the surveyed corridor mouth, facing down the row.
Release the joystick. R3:
```bash
roslaunch agbot_vision_nav vision_nav.launch mission_enabled:=true num_rows:=1
```
Wait for `vision_nav_node ready`. It drives the corridor, should stop at the
row end (`EXIT_CLEAR` → `DONE`), and stops. **Joystick deadman = instant
takeover.** Watch the overlay on the laptop:
```bash
# L1. ⟨LAPTOP_IP⟩ = the laptop's address on the robot's network (`hostname -I`)
export ROS_MASTER_URI=http://192.168.0.231:11311 ROS_IP=⟨LAPTOP_IP⟩
rqt_image_view /vision_nav_node/debug/image
```
- **Pass:** stays centred and stops past the row end.
- **Fail** (drifts into plants, or exits mid-row): stop here. The deploy can't
  pass. Still do §11 with `num_rows:=1` if you want to test only the transit
  and handoff, and take over by joystick at handoff.

**Ctrl-C R3.** Joystick back to the start mark.

---

## 11. THE FULL RUN: `deploy_mission.launch`

Robot **on the start mark, facing the arrow**. R1 (Reach) and R2 (cameras)
are still running. R3 is free.

### 11.1 First pass: one corridor
R4:
```bash
rosbag record -o ~/bags/corn_1117_deploy /tf /tf_static /gps/fix /amcl_pose /odometry/filtered \
  /cmd_vel /deploy_supervisor/status /vision_nav_node/status /bluetooth_teleop/joy \
  /move_base/current_goal /move_base/status
```
R3:
```bash
roslaunch agbot_nav deploy_mission.launch rviz:=true \
  map_file:=/home/administrator/agbot_control_ws/src/agbot_nav/maps/corn_1117.yaml \
  initial_x:=⟨SX⟩ initial_y:=⟨SY⟩ initial_yaw:=⟨SYAW⟩ \
  ⟨ENTRANCE_ARGS⟩ \
  num_rows:=1 first_turn_direction:=⟨left|right⟩ transit_speed:=0.3
```
R5 (status, one line per change):
```bash
rostopic echo /deploy_supervisor/status
```

What to expect, in order. Write down each **bold** item from R3's output:

| state | what happens | write down |
|---|---|---|
| LOCALIZE | robot sits still until AMCL settles (≤ 0.30 m / 10°) | time taken |
| TRANSIT | move_base drives to a point **0.7 m before** the entrance, on the row axis | time; did it hesitate or re-plan? |
| handoff | gate: ≤ 0.20 m / 12° off the axis, AMCL spread ≤ 0.15 m | **`arrived X m off the corridor axis, ±Y deg`** |
| ROW_MISSION | vision nav drives the row(s) | rows driven, any takeover |
| RETURN | move_base drives home around the block | time |
| done | **`DEPLOYMENT COMPLETE`** (or **`DEPLOYMENT FAILED: <reason>`**) | the line, as printed |

When it ends, R3 exits by itself ("REQUIRED process [deploy_supervisor] has
died" after FINISHED is normal). **Ctrl-C R4** (the bag). At the start mark,
take a fix in R5 → `⟨HOME_LAT⟩ ⟨HOME_LON⟩` (status 2). Convert it in L1 with
`latlon_to_map.py` like §8.2, and compare it with `⟨START_MAP_X/Y⟩`. That is
how close the robot came home.

If it **refuses the handoff** (off-axis or AMCL spread), the supervisor fails
and the robot stops. Note the printed numbers, joystick back to the start
mark, and run 11.1 again once. If it refuses twice in the same way, stop and
write it down. That is a result too.

### 11.2 Second pass: all corridors
Only if 11.1 completed. Robot back on the mark. Same R4 record command (new
bag), then R3:
```bash
roslaunch agbot_nav deploy_mission.launch rviz:=true \
  map_file:=/home/administrator/agbot_control_ws/src/agbot_nav/maps/corn_1117.yaml \
  initial_x:=⟨SX⟩ initial_y:=⟨SY⟩ initial_yaw:=⟨SYAW⟩ \
  ⟨ENTRANCE_ARGS⟩ \
  num_rows:=⟨NUM_CORRIDORS⟩ first_turn_direction:=⟨left|right⟩ transit_speed:=0.3 traverse_speed:=0.3
```
`traverse_speed:=0.3` slows the blind headland leg (default 0.5) for this
first run. The in-row speed stays at params.yaml's 0.15.

### If move_base won't plan
Symptom: `Failed to get a plan` in R3 and the robot doesn't leave the start.
The SLAM map holds the fake-corn leaves and the inflation closed a gap. Add
**`plan_on_map:=false`** to the deploy line. It plans on the live scan and
uses the map for AMCL only.

---

## 12. End of day

Ctrl-C in this order: R4 (if recording), R3, R2, R1. Never `pkill -f`.

### Robot → laptop (all of it, from L1)
```bash
scp "administrator@192.168.0.231:bags/*2026-11-17*.bag" ~/bags/   # every bag of the day: mapping_, corn_1117_amcl_, corn_1117_deploy_
mkdir -p ~/agbot_logs/robot_1117
scp "administrator@192.168.0.231:agbot_logs/vision_nav_20261117_*.csv" ~/agbot_logs/robot_1117/
scp administrator@192.168.0.231:agbot_control_ws/src/agbot_nav/maps/corn_1117.{yaml,pgm,start_pose} \
    $(rospack find agbot_nav)/maps/        # already done in §7.1; again in case of a remap (then corn_1117b.*)
```
Then on the laptop:
```bash
python3 ~/agbot_control_ws/src/agbot_vision_nav/scripts/analyze_run.py ~/agbot_logs/robot_1117/*.csv
```

### Laptop → robot
| file | when |
|---|---|
| `corn_1117_georef.yaml` | after the fit (§7.5), backup only |

Nothing else goes to the robot. Every `latlon_to_map.py` and `fit_georef.py`
run happens on the laptop.

---

## 13. Notes sheet: fill in at the field

| item | value |
|---|---|
| map name (`corn_1117` / `b` / `c`) | |
| start mark: robot heading (compass, rough) | |
| `⟨SX⟩ ⟨SY⟩ ⟨SYAW⟩` (save_map.sh's last line) | |
| `⟨MAP_BAG⟩` | |
| fit: RMS / pairs / span / yaw / t | |
| `⟨ENT_LAT⟩ ⟨ENT_LON⟩` (status 2?) | |
| `⟨FAR_LAT⟩ ⟨FAR_LON⟩` (status 2?) | |
| row length, corridor entered, corridors total | |
| `first_turn_direction` | |
| `⟨ENTRANCE_ARGS⟩` line | |
| compass bearing of the rows (script) | |
| `⟨START_LAT⟩ ⟨START_LON⟩` → `⟨START_MAP_X⟩ ⟨START_MAP_Y⟩` | |
| AMCL check: yaw / t vs fit | |
| lat/lon goal: arrival distance | |
| vision check: pass/fail, notes | |
| deploy 11.1: handoff `arrived … off axis, … deg` | |
| deploy 11.1: rows, takeovers, COMPLETE/FAILED line | |
| deploy 11.1: home fix → distance from start | |
| deploy 11.2: same three rows | |
| RTK: every status 0/1 with time and correction age | |
| anything odd (time + terminal) | |
