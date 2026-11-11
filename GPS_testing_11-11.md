# GPS testing 11/11 (2026-11-11): RTK georef, second run, new Reach cap

Robot `cpr-j100-0864`, Purdue campus, same empty ground as 11/9 (no corn set
out). Same goal as `GPS_testing_11-09.md`: tie RTK lat/lon to a LiDAR map,
check it against AMCL, drive to a lat/lon. This time the Reach sat on the new
3D-printed cap.

## Result in one line

**Every step passed, and this time the precision was measured.** A lat/lon goal
put the antenna **6.8 cm** from its target, with both readings on RTK fixed. RTK
held through the final map lap and the whole AMCL drive. It dropped out on the
earlier attempts, and those dropouts were of **two different kinds**.

---

## 1. What changed since 11/9

| item | 11/9 | 11/11 |
|---|---|---|
| Reach mount | housing stuck onto the LiDAR, lower, with gaps | 3D-printed cap: module ~20 mm above the LiDAR, antenna ~12 cm above the cap |
| antenna x/y | centred on the LiDAR | unchanged (still centred), so the lever stays **(-0.125, 0) m** |
| mapping bag | `rosbag record` by hand | `mapping.launch record:=true` (`/gps/fix` included) |
| goal command | worked out by hand | `latlon_to_map.py --goal-yaw` prints it, lever arm applied |

## 2. What was run, in order

| time | run | outcome |
|---|---|---|
| 10:49 | mapping attempt 1 (`mapping_2026-11-11-10-49-19.bag`, GPS saved as `gps_1049.bag`) | fixed 155 s, then **no corrections** (status 0) for the last 32 s |
| 11:01 | mapping attempt 2 (`mapping_2026-11-11-11-01-08.bag`), map saved as `field_test` | fixed 45 s, **float** 109 s, fixed 174 s. Recovered with nothing touched |
| 11:19 | **mapping lap used** (`mapping_2026-11-11-11-19-23.bag`), map saved as **`field_test_2`** | 184 of 195 fixed; one 11 s float stretch at t ≈ 140 s |
| — | fit on the laptop | RMS 8.1 cm, 113 pairs, span 16.7 m |
| 11:27 | AMCL drive on `field_test_2` (`field_amcl_2026-11-11-11-27-12.bag`) | 169 of 169 fixed |
| 11:35 | lat/lon target read at a marked spot | status 2 |
| 11:44 | robot driven away, goal sent, arrival read | status 2, **6.8 cm** off |
| — | operator drove the robot during a goal | it re-planned and returned to the goal around obstacles (watched, not logged) |

## 3. Results

### 3.1 Fit (mapping lap, 11:19): usable

```bash
rosrun agbot_nav fit_georef.py ~/bags/mapping_2026-11-11-11-19-23.bag --lever=-0.125,0 \
  --out $(rospack find agbot_nav)/maps/field_test_georef.yaml
```

| | 11/9 | 11/11 | target |
|---|---|---|---|
| RMS | 4.6 cm | **8.1 cm** | ≤ 5 cm (≤ 10 usable) |
| max / p95 | 10.1 / 7.5 cm | 15.7 / 12.1 cm | |
| pairs | 24 | **113** | ≥ 40 |
| span | 4.6 m | **16.7 m** | ≥ 10 m |
| lap length | 22.5 m | ~45 m | |

The RMS is higher, but the lap was twice as long and the fit had almost five
times as many points. The 11/9 fit was low partly because it only covered 4.6 m.
A wider span pins the yaw much better.

**The fit:** datum **40.42218610, -86.92023739**; yaw **+165.96°** (map +x points
at compass **256.0°**); t **(-22.688, +1.076) m**.

### 3.2 AMCL check (11:27): pass, close to both limits

| | mapping fit | AMCL-drive fit | difference | limit |
|---|---|---|---|---|
| yaw | 165.964° | 166.875° | **0.91°** | < 1° |
| t | (-22.688, 1.076) | (-22.721, 0.999) | **8.4 cm** | < 10 cm |
| fit RMS | 8.1 cm | 8.8 cm | | |

Point by point (AMCL pose + lever arm vs GPS through the mapping fit), over 169
fixed readings: **RMS 13.1 cm, max 26.3 cm, p95 20.8 cm**.

| part of the drive | AMCL vs GPS |
|---|---|
| parked at the start (0–30 s) | ~8 cm |
| far side, ~12 m from the start (40–90 s) | 15–20 cm |
| on the way back (110–160 s) | 5–10 cm |

Unlike 11/9 there was **no slip**: AMCL's own spread stayed at 10–19 cm while
driving, against 37 cm on 11/9. The error grows with distance from the start.
That fits the 0.9° rotation between the two fits: 0.9° over 12 m is about
19 cm. Either the map is slightly bent, or the mapping fit's yaw is that far
off. Not yet resolved.

⚠ The first run of this check used `--datum 40.42214595,-86.92049982`. That is
the **map origin** line of the fit output, not the datum. The numbers above are
from a rerun with the real datum (the `ENU about ...` line).

### 3.3 Lat/lon goal: PASS, measured on fix

```bash
rosrun agbot_nav latlon_to_map.py $(rospack find agbot_nav)/maps/field_test_georef.yaml \
  40.422186178333334 -86.92023758166667 --goal-yaw -0.1711
# -> rostopic pub -1 /move_base_simple/goal ... x: -22.551, y: 1.042, z: -0.0854, w: 0.9963
```

| | lat, lon | map (antenna) | status |
|---|---|---|---|
| target | 40.422186178, -86.920237582 | (-22.674, 1.064) | 2 |
| arrival | 40.422185843, -86.920238255 | (-22.610, 1.086) | 2 |

**6.8 cm** from target (about 5 cm W and 4 cm S), against a 15 cm pass mark. This
includes move_base's own goal tolerance. Arrival heading was not measured.

### 3.4 RTK: two different failure modes

`/gps/fix` status tells them apart:

| status | GGA quality | meaning | seen |
|---|---|---|---|
| 2 | 4 | RTK fixed | most of the day |
| 1 | 5 | RTK float: corrections arrive, the Reach can't resolve | 11:01, 109 s; 11:19, 11 s |
| 0 | 1 | single: **no corrections at all** | 10:49, last 32 s; σ jumped to ~1 m |

- **Status 0 means the correction link dropped**, not the antenna or the sky. It
  needs a different fix from float.
- **Float recovered by itself** in under 2 minutes with nothing touched, and did
  not come back on the same path in the later runs. That fits a cause that
  changes with time (satellite geometry, the base station) better than a
  robot or mount problem.
- The new cap held fix for whole runs, which 11/9's mount did not after its first
  drop. It looks better, but one day is not proof.

## 4. Numbers to keep (this map, this site)

| what | value |
|---|---|
| map | `agbot_nav/maps/field_test_2.{yaml,pgm,start_pose}` (robot and laptop) |
| georef | `agbot_nav/maps/field_test_georef.yaml` (laptop). ⚠ It belongs to **field_test_2**; rename it `field_test_2_georef.yaml` |
| datum | 40.42218610, -86.92023739 |
| fit | yaw +165.96°, t (-22.688, +1.076), RMS 0.081 m, 113 pairs, span 16.7 m |
| start pose | `initial_x:=-22.577 initial_y:=1.068 initial_yaw:=-0.1711` (robot facing ~266°, W) |
| lever arm | (-0.125, 0.0) m |
| bags (laptop `~/bags/`) | `mapping_2026-11-11-11-19-23.bag`, `field_amcl_2026-11-11-11-27-12.bag`, `mapping_2026-11-11-11-01-08.bag`, `gps_1049.bag` |

`field_test` (from 11:01) is a separate map with no fit. Don't pair it with this
georef.

## 5. What went wrong with commands today

| what happened | cause | next time |
|---|---|---|
| `ls -lh ~/bags/ \| tail -3` showed no new bag | `ls` sorts by name, and `mapping_*` sorts before `transit1_*` (Claude's runbook error) | `ls -lht ~/bags \| head` |
| AMCL check gave t = (-0.019, 0.283) | `--datum` was given the **map origin** lat/lon | use the `ENU about ...` line, or the `datum:` line of the georef yaml |
| georef named `field_test_georef.yaml` | the map used was `field_test_2` | name the fit after the map: `--out .../<MAP>_georef.yaml` |

What went right: every terminal was sourced, `--lever=` was used, both lat/lon
readings were taken on fix, and the goal was pasted from `--goal-yaw` output
instead of being worked out by hand.

## 6. Still open

| # | problem | state |
|---|---|---|
| 1 | **Correction-link dropouts** (status 0) | new today. How corrections reach the Reach (phone hotspot? campus Wi-Fi? Emlid Flow over LTE?) is not recorded |
| 2 | **RTK float** (status 1) | still unexplained; self-recovers in < 2 min |
| 3 | AMCL vs GPS grows to 15–20 cm ~12 m from the start | consistent with a 0.9° yaw mismatch; marginal for a 0.75 m row with ~16 cm clearance per side |
| 4 | `/gps/fix` at 1 Hz although the Reach is at 5 Hz | `telnet 192.168.2.15 7777` GGA vs GST count still not done |
| 5 | live AMCL-vs-GPS monitor | not written |
| 6 | **row entrance by lat/lon** (step 5) | not tried: no rows set out |
| 7 | arrival heading | not measured |

## 7. Next steps

1. **Row entrance by lat/lon (step 5)**, the reason for all of this. Set out fake
   corn (or go to a real field). Survey the entrance with the antenna on fix,
   measure the row bearing, and run `deploy_mission.launch` with
   `latlon_to_map.py ... <BEARING> --args`. Map a fresh lap there; this fit
   belongs to this site only.
2. **Find out how corrections reach the Reach**, and log Emlid Flow's correction
   age the moment status 0 appears. If it is a phone hotspot, try a steadier link.
3. **Settle the 0.9°.** Fit the 11:01 bag (219 fixed readings, same area) and
   compare yaws. If they disagree, the map is bent. If they agree with each
   other, the AMCL drive is the odd one out.
4. **GGA vs GST count** (1 min, Reach powered): decides whether `/gps/fix` can
   reach 5 Hz.
5. **Write the AMCL-vs-GPS monitor** (laptop code). It would flag item 3 and
   11/9's slip live instead of in post-processing.
