# Field test 11/17 (2026-11-17): first full deploy on the robot, entrance from lat/lon

Robot `cpr-j100-0864`, outside the lab, fake corn set out (3 plant rows = 2
corridors, ~3.6 m long). Runbook: `GPS_testing_11-17_runbook.md`. Map
**`corn_1117`**, georef **`corn_1117_georef.yaml`**. Every number below is
recomputed from the bags on the laptop (`~/bags/*2026-11-17*`), not copied from
the terminal, unless it says so.

## Result in one line

**The whole workflow ran on the robot three times, and every handoff was
accepted.** Start spot → AMCL + move_base → a row entrance given only as two
RTK fixes → vision nav through both corridors → move_base back home. 6
corridors and 42.6 m with **zero joystick takeovers**. Two of the three runs
finished `DEPLOYMENT COMPLETE`. Run 1 asked for 3 corridors when the block
has 2; it looked for a third, correctly gave up, and was stopped by hand as
it started home. In the surveyed corridor the robot ran **within ±7 cm of the
surveyed line** (GPS), mean −1 cm.

Problems found, none of which stopped a run: the supervisor's arrival heading
reads a **stale AMCL pose** (it printed +7 to +9°; the real figure was +4°).
Run 3 **spun about 1.3 turns** on the home spot to hit move_base's yaw
tolerance. **AMCL jumped 9 m twice** in the AMCL checks, both times 7–10 m
outside the area the mapping lap covered. The Reach's USB-Ethernet link
dropped repeatedly.

---

## 1. What was run, in order

| time | run | outcome |
|---|---|---|
| 11:29 | `mapping_2026-11-17-11-29-47.bag` | 2.9 s, stopped straight away |
| 11:30 | **mapping lap used** (`mapping_2026-11-17-11-30-20.bag`) | 203/203 fixed, 19.7 m, no slips; saved as `corn_1117` |
| — | fit on the laptop | RMS 4.8 cm, 49 pairs, span **7.0 m** |
| 11:45 | AMCL drive (`corn_1117_amcl_…11-45-04`) | agreed with GPS to 2–11 cm for 105 s, then **AMCL jumped** (§3.3) |
| 11:49 | AMCL bag (`…11-49-03`) + `mapping_…11-49-09` (0.7 s) | 55 s, all float, robot parked |
| 11:53 | AMCL drive (`…11-53-09`) | same as 11:45: good for 95 s, then the **same jump at the same spot** |
| 11:56 | AMCL drive (`…11-56-35`) | GPS **cut out at 60 s**, just as the robot started moving → fit refused (1 pair) |
| ~12:00 | lat/lon goal to the start spot (§8.2) | **13.7 cm** (pass ≤ 15) |
| 12:33 | `mapping_2026-11-17-12-33-25.bag` | 5.4 s, stopped straight away |
| 12:42 | **deploy run 1**, `num_rows:=3` (`corn_1117_deploy_…12-41-20`) | 2 corridors; hunted 2 m for a 3rd, `Mission DONE`; Ctrl-C as RETURN began |
| 12:46 | **deploy run 2**, `num_rows:=2` (same bag) | **DEPLOYMENT COMPLETE**, home in 29 s |
| 12:52 | **deploy run 3**, `num_rows:=2` (`…12-50-38`) | **DEPLOYMENT COMPLETE**, home in 76 s (spin, §4.6) |

All deploys used `transit_speed:=0.15 plan_on_map:=false
camera_topic:=/brio_front/image_raw/compressed`, in-row 0.15 m/s,
`traverse_speed` at its default 0.5.

⚠ The start mark was only **~1.8 m** from the row mouth, not the 8–15 m the
runbook asked for, so TRANSIT drove ~1.3 m. The handoff is tested; a long
transit is not.

## 2. Survey, map and fit

### 2.1 Surveyed points (all status 2)

| point | lat, lon | map (base_link) |
|---|---|---|
| entrance `ENT` | 40.42219652833, −86.920235635 | (0.588, 0.070) |
| far end `FAR` | 40.42222866333, −86.92023539667 | 3.57 m from ENT |
| row bearing | compass **0.3°** (rows run north) | map yaw **91.3°** |
| start mark `START` | 40.42218242167, −86.92022331333 | antenna (1.679, −1.592) |
| lat/lon-goal arrival `ARR` | 40.42218236833, −86.92022493667 | antenna (1.542, −1.601) |
| home after a deploy | 40.42218298167, −86.92022657 | antenna (1.401, −1.537) |

`first_turn_direction:=left`. Start pose `initial_x:=1.500 initial_y:=-1.543
initial_yaw:=1.5887`.

### 2.2 Fit (mapping lap 11:30): pass on RMS, short on span

| | 11/11 | 11/17 | target |
|---|---|---|---|
| RMS | 8.1 cm | **4.8 cm** | ≤ 5 cm |
| max / p95 | 15.7 / 12.1 cm | 8.4 / 7.4 cm | |
| pairs | 113 | 49 | ≥ 40 |
| span | 16.7 m | **7.0 m** | ≥ 10 m |
| lap | ~45 m | 19.7 m | |
| GPS in the lap | 184/195 fixed | **203/203 fixed** | |

Fit: datum **40.4221819, −86.92022488**; yaw **+1.589°** (map +x → compass
91.6°); t **(+1.548, −1.653) m**.

The lap went round the block only: x −2.6…1.5, y −1.8…4.7 in the map. That
is enough for the deploy, which stayed inside x −1.4…1.6, y −1.5…4.1. It is
not enough for anything further out (§3.3).

### 2.3 Lat/lon goal to the start spot: pass

Target `START` antenna (1.679, −1.592), arrival `ARR` (1.542, −1.601):
**13.7 cm** (pass ≤ 15; 11/11 was 6.8). The miss is almost all in x (−13.7
cm), with move_base's 12 cm `xy_goal_tolerance` inside it.

## 3. AMCL checks

### 3.1 The live check was refused, and why

```
fit_georef.py ~/bags/corn_1117_amcl_2026-11-17-11-56-35.bag ... → pairs after 0.25 m thinning: 1
```
The 11:56 bag has 61 fixes in 175 s. **`/gps/fix` stopped at t = 60.5 s, and
the robot started moving at t ≈ 61 s.** Every fix it has was taken while
parked, so thinning by distance leaves one pair. The fit refused correctly;
the Reach link is the cause (§5).

### 3.2 Offline check from the good parts of 11:45 and 11:53: pass

Fitted only up to the jump (`agbot_nav.georef.fit_georef`, mapping datum):

| | mapping fit | 11:45 (t < 104 s) | 11:53 (t < 94 s) | limit |
|---|---|---|---|---|
| yaw | 1.589° | 0.722° (**−0.87°**) | 0.833° (**−0.76°**) | < 1° |
| t | (1.548, −1.653) | (1.549, −1.713), **6.0 cm** | (1.533, −1.712), **6.1 cm** | < 10 cm |
| pairs / span | 49 / 7.0 m | 51 / 8.4 m | 39 / 8.3 m | |
| RMS | 4.8 cm | 6.3 cm | 5.7 cm | |

The two AMCL drives agree with each other to **0.11°**, and both differ from the
mapping fit by ~0.8°. 11/11's difference was 0.91°. A ~0.8–0.9° gap between
"SLAM lap" and "AMCL drive" has now shown up twice. It looks systematic
rather than noise. Over the 4 m of the block it is 6 cm, which is negligible;
over 15 m it would be 22 cm.

### 3.3 AMCL jumped twice, both times outside the mapped area

GPS stayed RTK fixed and smooth at ~0.37 m/s through both drives, so GPS is
the reference here. AMCL, not GPS, is what failed:

| bag | good until | then | `map→odom` |
|---|---|---|---|
| 11:45 | t ≈ 105 s, err 2–11 cm | err grows to **9.3 m** | swung from (2.5, 0.2, −62°) to (0.9, 8.7, −7°) in ~6 s |
| 11:53 | t ≈ 95 s, err 6–10 cm | err grows to **9.3 m** | swung from (2.6, −0.8, −80°) to (1.5, 8.5, −24°) |

Both jumps started at the **same place**: map x ≈ −6.8, y ≈ 3, driving
north. That is 4 m west of anything the mapping lap drove (the lap's west edge
was x = −2.6), and both drives went on to x ≈ −12.8. The map only has what
the LiDAR saw from the block. Out there AMCL is matching a scan against the
far edge of the map, seen once from 5–10 m away. It locked onto a wrong pose
with a low spread (0.16–0.3 m), which is the worst case: it looked confident.

**This is probably the "robot jumped" you saw in RViz.** It happened during
the AMCL drives, not the mapping lap. The 11:30 lap has no jumps
(map-vs-odom step never off by more than 0.15 m), and the other three mapping
bags last 0.7–5.4 s each, so there is nothing in them to check.

The 11:56 drive also went out to x = −8.1. `map→odom` wobbled once at
t ≈ 116 s ((0.6, −3.5, −103°) for a moment) and came back. With GPS gone by then
there is no independent check of it.

**Rule from this: AMCL is only trustworthy where the mapping lap drove.**
Either keep every later drive inside the lap, or map the whole area that will
be driven.

## 4. The deploys

### 4.1 Timeline (seconds from launch)

| | LOCALIZE | TRANSIT | handoff | ROW_MISSION | RETURN | total |
|---|---|---|---|---|---|---|
| run 1 | 2.6 s | 13.5 s | 16.1 | 115.7 s | Ctrl-C at start | — |
| run 2 | 2.4 s | 14.1 s | 16.5 | 69.4 s | 29.4 s | **115.3 s** |
| run 3 | 2.6 s | 13.8 s | 16.4 | 69.0 s | 76.5 s | **161.9 s** |

GPS during all three: **100% RTK fixed, no gaps** (138, 125, 175 fixes).
AMCL vs GPS (antenna, mapping fit): **RMS 5.4 / 5.4 / 5.6 cm**, max 11 / 9 /
12 cm, against 13.1 cm on 11/11. AMCL's spread peaked at 0.40–0.43 m only while
LOCALIZE was settling.

### 4.2 Handoff: accepted all three times, and very repeatable

| | cross-track | heading, as printed | heading, actual (TF at handoff) | AMCL std |
|---|---|---|---|---|
| run 1 | −0.056 m | +9.3° | **+4.0°** | 0.078 m / 3.3° |
| run 2 | −0.036 m | +8.7° | **+4.2°** | 0.087 m / 3.8° |
| run 3 | −0.046 m | +6.8° | **+4.2°** | 0.079 m / 3.4° |
| limit | 0.20 m | 12° | | 0.15 m / 8° |

⚠ **The printed heading is stale.** move_base reaches the standoff point
arriving ~50° off the row axis and finishes with a turn in place (TF: +55° at
−6 s, +18° at −2 s, +4° at handoff). AMCL publishes `/amcl_pose` only after
0.1 m or 0.1 rad of motion. Its last message came 0.6–1.0 s before "Goal
reached", in the middle of that turn, and the supervisor checks that message.
It is off by 3–5° in the cautious direction, so nothing wrong got through. But
it uses up 40% of the 12° budget, and a slower final turn would make the gap
bigger. **Fix: read `map→base_link` from TF at the moment of the check**
(`deploy_supervisor.py`, `axis_errors(estimate, …)` input).

Vision corrected the remaining +4° within ~3 s (TF +0.7° at +3 s).

### 4.3 In the rows: lateral position in metres, from GPS

The surveyed line (`ENT` + row bearing) is corridor 1's axis, so GPS gives
real-metre tracking error there without trusting vision or AMCL. Lever arm
removed, RTK fixed, ~1 Hz:

| | corridor 1 (surveyed): mean / sd / range | corridor 2: centre / sd / range | GPS track vs axis |
|---|---|---|---|
| run 1 | −0.8 / 3.3 / −5.3…+7.3 cm | +0.928 / 2.6 cm / 0.900…0.995 m | +1.8° / −1.2° |
| run 2 | −1.0 / 3.1 / −4.8…+7.4 cm | +0.972 / 3.9 cm / 0.938…1.076 m | +1.6° / −1.9° |
| run 3 | −0.9 / 2.5 / −4.9…+6.6 cm | +0.955 / 5.9 cm / 0.895…1.093 m | +1.3° / −3.4° |

- **Corridor 1:** vision put the robot on the same line the operator drove when
  surveying, to about 1 cm, three times in a row. AMCL (TF) says +1.5 to +2 cm;
  the two agree.
- ⚠ **Corridor 2 sits 0.93–0.97 m from corridor 1**, not 0.75 m. Either the
  plant rows were set ~0.95 m apart, or vision ran ~20 cm off-centre in
  corridor 2. The second is unlikely, because the spread was 3–6 cm with no
  takeovers and the centre was the same on all three runs. **Please confirm the
  tape-measured row spacing.** `row_spacing` is 0.75 (traverse 0.6 m); the
  robot still landed at 0.88–0.95 m, so REACQUIRE took up the difference.
- Vision's own figure (FOLLOW_ROW RMS `offset_norm`, image space, from the
  terminal): 0.067 / 0.085 / 0.085.
- GPU timing: inference 17–18 ms, e2e 66–81 ms, 17–25 Hz; no `WATCHDOG_ZERO`.

### 4.4 Headland

| | | run 1 (×2) | run 2 | run 3 |
|---|---|---|---|---|
| EXIT_CLEAR | starts at along | 2.55 / 0.75 m | 2.50 m | 2.56 m |
| | leg length | 1.44 / 1.56 m | 1.42 m | 1.43 m |
| | rear camera read "open" | no / **at 1.28 m** | no | no |
| TURN_1 | odom swept at stop / coast | 85.5 / +1.6°, −85.5 / −1.3° | 85.4 / +1.4° | 85.5 / +1.2° |
| TURN_2 | odom swept at stop / coast | 85.6 / +1.8°, −85.2 / −1.4° | 85.5 / +1.1° | 85.6 / (+5.8°, REACQUIRE steering mixed in) |
| REACQUIRE | duration | 4.0 s / 27.9 s (no 3rd row) | 5.0 s | 2.5 s |
| heading into corridor 2 (AMCL) | | −3.9° | −0.6° | −3.3° |

- **The turn coast-down is now measured, on concrete: 1.1–1.8° in
  0.13–0.27 s.** CLAUDE.md asks for this number before anyone touches
  `yaw_tolerance_deg`. The 5° band stops at 85.5° and the coast brings it to
  86.5–87.4°, so each turn ends ~3° short. Treat it as a measurement for the
  sim-vs-field decision, not as a reason to retune. It comes from 7 clean
  turns on one surface at 0.27 rad/s.
- **Actual yaw rate is 0.27–0.29 rad/s against 0.40 commanded** (~70%):
  skid-steer tyre scrub (the test was on **concrete**, not grass). A 90° turn
  takes 5.5–6 s, not 3.9 s. At low commands it is worse: 0.20 commanded
  gave ~0.10 rad/s on the home spot.
- The exit leg ran to its 1.5 m ceiling in 5 of 6 exits; the rear camera
  confirmed "open" only once. That leaves the tail ~0.4–0.6 m past the `FAR`
  point before it turns, with no harm done. The rear-steered leg drifted
  5–13 cm left.
- Run 1's third REACQUIRE: there was no third corridor. It crawled
  `reacquire_max_distance` 2.0 m into open ground at 0.08 m/s (to map lateral
  +1.85 m, west of the block) and ended `Mission DONE … rows_driven=2`. That is
  the correct ending.

### 4.5 Joystick

`/bluetooth_teleop/joy` deadman (buttons 4/5) was never pressed in any of the
three runs. **0 interventions over 42.6 m** (vision logger: 14.8 + 13.9 +
13.9 m). Still only ~6 corridors of 3.6 m, so it says little about MDBI yet.

### 4.6 Home

| | home goal (AMCL at LOCALIZE) | GPS, run start → end | TF, start → end | time |
|---|---|---|---|---|
| run 2 | (1.55, −1.54) | **0.21 m** | 0.11 m | 29 s |
| run 3 | (1.51, −1.51) | **0.11 m** | 0.13 m | 76 s |

- Runs 2 and 3 ended **3 cm apart** (GPS). The pasted "second test" home fix
  matches both to 3–4 cm.
- That spot is 0.28 m from the `START` mark, because the robot was put down
  12–24 cm off the mark at the start of each run. The home goal is wherever
  AMCL said the robot was at LOCALIZE, not the mark.
- ⚠ **Run 3 spun on the spot for ~50 s, because it was 1–3 cm outside the
  12 cm position tolerance.** *(Corrected 2026-11-18. The first version of
  this note blamed the 4.6° yaw window and the turn rate. The bag shows
  otherwise.)* What happened:
  - At t = 24 s it was **8.6 cm** from the home goal: inside
    `xy_goal_tolerance` 0.12 m. DWA latched "position reached" and began
    turning to the goal heading.
  - A skid-steer turn in place does not pivot exactly on `base_link`, so the
    turn moved base_link out to **11.8 → 13.3 → 14.7 cm** from the goal.
  - `planner_frequency: 1.0` hands DWA a new global plan every second, and
    every new plan **resets DWA's latch** (`setPlan` → `resetLatching`). From
    then on DWA was back in "reach the position" mode, 1–3 cm short, with the
    robot facing the wrong way. It can't drive 2 cm (`min_vel_trans` 0.05,
    1.7 s sim time), so the best-scoring command was turning in place, and it
    kept turning straight **through** the goal heading (82→92° at t = 30–32 s
    with +0.2 rad/s still commanded).
  - The two "Clearing both costmaps" warnings are `oscillation_timeout` (20 s
    without moving 0.3 m) firing at t ≈ 42 and 62 s.
  - At t ≈ 66 s a recovery burst nudged it to 11.3 cm. It latched again and
    finished at 76 s.
  - Run 2 stayed at **8.7–9.5 cm** through its turn, so the latch held and it
    finished in 5 s.

  **Not a heading or IMU problem:** the commands and the measured rotation
  agree throughout, and AMCL's `map→odom` yaw stayed within 149–153° for the
  whole spin.

  **Fix:** a looser position tolerance for the home goal only (e.g.
  `xy_goal_tolerance` 0.25 and `yaw_goal_tolerance` 0.3 rad, set through
  dynamic_reconfigure by the supervisor before it sends RETURN). The transit
  goal keeps 0.12, because the handoff gate depends on it. Alternative:
  `planner_frequency: 0.0`, so the latch is never reset; that also stops
  replanning during transit.

## 5. RTK / Reach link

Reported on the day: the Reach "went disconnected quite frequently", and each
time the interface had to be brought back with `sudo ip link set $IF up`.
What the bags show:

| bag | GPS |
|---|---|
| 11:30 mapping | 203/203 fixed, no gaps |
| 11:45 AMCL | fixed, then a **12 s gap** at t = 174 s (bag end) |
| 11:49 AMCL | **55 s all float**, parked |
| 11:53 AMCL | 129 fixed / 6 float, **11 s gap** at t = 135 s |
| 11:56 AMCL | **cut at t = 60.5 s** for the remaining 115 s, as the robot started to move |
| 12:42–12:55 deploys | 100% fixed, **no gaps** |

The GPS gaps are **not** what made AMCL jump: the jumps came 70 s and 38 s
before them, and GPS isn't in the AMCL loop at all.

"Had to bring the link back up" means the USB-Ethernet adapter went
down, not that corrections stopped (status 0) or the fix went to float (status
1). Things to check next time it happens, on the robot:
```bash
dmesg -T | grep -iE "usb|enx|r8152|cdc_ether" | tail -20   # USB disconnect/reset at that time?
nmcli device status                                         # is NetworkManager managing enx*?
journalctl -u NetworkManager --since "10 min ago" | grep -i enx
```
- A USB `disconnect` / `new device` pair means it's physical: cable, connector
  or power. The 11:56 drop at the first metre of motion points that way. Strain-
  relieve the cable and try a different port, ideally not on the same hub as
  the cameras.
- NetworkManager deactivating `enx*` after a failed DHCP means it's software:
  give the interface a static profile once, and it comes back on its own:
  ```bash
  nmcli con add type ethernet ifname enx… con-name reach ipv4.method manual ipv4.addresses 192.168.2.2/24
  ```

## 6. Open items

| | item | state |
|---|---|---|
| fix | supervisor arrival heading from stale `/amcl_pose` | read TF `map→base_link` at the check (§4.2) |
| fix | RETURN spin on the home spot | home goal 1–3 cm outside `xy_goal_tolerance`, latch reset by the 1 Hz replan: looser xy/yaw tolerance for RETURN only (§4.6) |
| fix | Reach USB-Ethernet drops | run the `dmesg` / `nmcli` checks; static NM profile (§5) |
| field | map the whole operating area | mapping lap ≥ 10 m span and covering every later drive (§3.3) |
| field | a real transit | start mark 8–15 m out, as in the runbook |
| field | confirm the row spacing (0.95 vs 0.75 m) | §4.3 |
| data | copy the vision CSVs to the laptop | `scp "administrator@192.168.0.231:agbot_logs/vision_nav_20261117_*.csv" ~/agbot_logs/robot_1117/` (CSVs 124212, 124650, 125202) |
| note | ~0.8–0.9° SLAM-vs-AMCL yaw gap, seen on 11/11 and 11/17 | systematic; matters only on long transits |
| note | exit leg always hits the 1.5 m ceiling; rear "open" rarely fires on fake corn | no harm with ≥ 2.5 m headland |
| cosmetic | `deploy_supervisor` traceback on Ctrl-C (`publish() to a closed topic`) | guard `_tick` with `rospy.is_shutdown()` |

## 7. Numbers to keep

| what | value |
|---|---|
| map | `agbot_nav/maps/corn_1117.{yaml,pgm,start_pose}` (padded 1153×1374, origin (−25.94, −31.47)) |
| georef | `corn_1117_georef.yaml`: datum 40.4221819, −86.92022488; yaw 0.027734 rad; t (1.5485, −1.6533) |
| start pose | `initial_x:=1.500 initial_y:=-1.543 initial_yaw:=1.5887` |
| entrance args | `entrance_x:=0.588 entrance_y:=0.070 entrance_yaw_deg:=91.3 row_bearing_deg:=91.3` |
| lever arm | (−0.125, 0.0) m |
| bags | `mapping_2026-11-17-11-30-20`, `corn_1117_amcl_2026-11-17-{11-45-04,11-49-03,11-53-09,11-56-35}`, `corn_1117_deploy_2026-11-17-{12-41-20,12-50-38}` |
| vision CSVs (robot) | `vision_nav_20261117_{124212,124650,125202}.csv` |
