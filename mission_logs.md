# Field test — 2026-09-09 morning

Vision-nav multi-row mission runs on the real Jackal, analysed from
`mission_logs/vision_nav_20260909_*.csv` with
`agbot_vision_nav/scripts/analyze_run.py` plus per-state distance and
saturation breakdowns.

**Scope**: the five mission-mode runs (09:06–09:31). Three further logs from the
same morning — `094500` (plain row-following / manual repositioning at
0.15 m/s), `102012` and `102256` (blocked-row back-out tests at 0.5 m/s) — were
exercising other functionality and are excluded from every figure below.

## What ran

| # | Time | Cruise | Rows | Distance | In-row interventions | FOLLOW_ROW RMS |
|---|------|--------|------|----------|------|------|
| 1 | 09:06 | **0.30** m/s | 6 ✅ | 93.4 m | **2** | 0.064 |
| 2 | 09:16 | **0.60** m/s | 6 ✅ | 92.8 m | 0 | 0.045 |
| 3 | 09:22 | **0.90** m/s | 3 ⚠ aborted | 49.3 m | **2** | 0.080 |
| 4 | 09:25 | **0.60** m/s | 6 ✅ | 98.2 m | 0 | 0.068 |
| 5 | 09:31 | **0.60** m/s | **8** ✅ | 119.5 m | 0 | 0.059 |

**Totals: 453.2 m driven, 441.6 m autonomous, 29 rows completed.**
Runs 1, 2 and 4 reached `MISSION_DONE` cleanly at 6/6 rows. Run 5 completed
8 rows and was paused by the operator during the 9th headland leg. Run 3 was
aborted after 3 rows.

Pooled autonomy: **441.6 m autonomous / 4 in-row interventions = 110.4 m per
intervention (MDBI)**.

⚠ Three further interventions are logged in state `DONE` — the operator picking
the robot up after the mission had already ended. They are handling, not
rescues, and are excluded from the MDBI above. `analyze_run.py` charges them to
the run, so its per-run intervention counts read 3/1/2/1/0 rather than the
2/0/2/0/0 used here.

## The headline

The **0.6 m/s set (runs 2, 4, 5)** is the result worth quoting:

- 3 runs, **20 rows**, **310.5 m** driven, 302.9 m autonomous
- **zero in-row interventions**
- MDBI has no mean, only a lower bound: **≥ 302.9 m per intervention**
- FOLLOW_ROW RMS `offset_norm` 0.045 / 0.068 / 0.059 — tight and consistent

Run 5 alone is the single best run of the morning: 8 rows, 119.5 m, zero
interventions, zero teleop.

## Failure modes

### 1. Run 1 (0.30 m/s) — stalk contact and an occluded leaf

Both interventions land in `rows_driven=1`, the first row after the first
headland turn, at 0.64 m and 6.90 m into it.

| d_row | offset_norm | corridor edges | traversable | obstacle |
|---|---|---|---|---|
| 0.64 m | 0 | 0.00 / 1.00 | 0.19 | 0.77 |
| 6.90 m | 0 | 0.00 / 0.00 | 0.02 | 0.98 |

Both are essentially all-obstacle masks — the camera was looking at foliage,
not a corridor. Consistent with the operator's account: contact with a stalk,
then a leaf over the lens, each needing a short nudge forward on the joystick.

### 2. REACQUIRE hands off short — and the first turn is the worst case

Residual `offset_norm` at the moment `REACQUIRE` gives way to `FOLLOW_ROW`:

| run | cruise | exit residuals, in order |
|---|---|---|
| 1 | 0.30 | **+0.261**, −0.043, −0.116, −0.200, −0.088 |
| 2 | 0.60 | −0.072, +0.075, −0.063, −0.107, −0.099 |
| 3 | 0.90 | **−0.237**, +0.081 |
| 4 | 0.60 | +0.004, +0.100, −0.090, −0.102, −0.087 |
| 5 | 0.60 | −0.144, −0.035, +0.054, −0.101, −0.060, −0.038, −0.142 |

Averaged over the first 25 frames of the new row, run 1 entered at
|offset| 0.189 and run 3 at 0.255, against 0.02–0.11 everywhere else.

So REACQUIRE systematically leaves ~0.1 of offset and occasionally 0.25, and
**both bad rows of the morning are the two worst cases**. The state exits
before it has actually re-centred; at low speed FOLLOW_ROW absorbs that, at
speed it does not.

### 3. Run 3 (0.90 m/s) — a steering-authority failure, not a perception one

`angular_z_max` stayed at its 0.15 m/s default of **0.175 rad/s in every run** —
only `linear_x_cruise` was raised, so `speed_args.py` was not used. Minimum turn
radius is `linear_x / angular_z_max`:

| cruise | min turn radius | % of in-row frames pinned at the clamp |
|---|---|---|
| 0.15 (proven envelope) | 0.86 m | — |
| 0.30 | 1.71 m | 8.1% |
| 0.60 | 3.43 m | 7.3 / 11.7 / 15.6% |
| **0.90** | **5.14 m** | **28.5%** |

At 0.9 m/s more than a quarter of in-row frames sat at the clamp: the
controller was continuously asking for more turn than it was allowed. Against
~0.16 m of clearance per side in a 0.75 m row, a 5.14 m radius cannot recover
a 0.24 m handoff error before the next plant — which is why the overshoot
after turn 1 was still uncorrected at turn 2, where the run failed.

Invalid frames also rose to 20.6% overall (4.9% within FOLLOW_ROW), against
0.2–3.6% at 0.6 m/s — motion blur on top of the geometry.

This is exactly the coupling CLAUDE.md warns about: **never raise
`linear_x_cruise` alone.** A repeat attempt at 0.9 m/s should go through
`speed_args.py` so `angular_z_max`, `delta_angular_z_max` and `mpc_alpha` scale
with it.

## Data-quality note — timestamps are unusable in these CSVs

Every timestamp in these files reads `1.78896e+09`. `metrics_logger._fmt`
formats all floats with `%.6g`, which on a 1.79-billion epoch leaves ~1000 s of
resolution, so `t_ros`, `t_wall` and `frame_stamp` are identical within a file.
`analyze_run.py` therefore reports duration 0 s and teleop time 0 s for every
run.

Distances, intervention counts and offsets are **unaffected** — interventions
are counted live in the node against full-precision time, and distance comes
from odometry — so nothing above is compromised. But there is no wall-clock
duration for this test, and there will not be for any future one until the
timestamp columns are written at full precision.

## Takeaways

1. **0.6 m/s is validated** — 20 rows and 310.5 m with no in-row intervention.
   It is the speed to quote and to run again.
2. **REACQUIRE is the top defect.** It exits with up to 0.26 of residual
   offset, most often on the first headland turn. Fixing the exit criterion
   would have prevented both of run 1's rescues and probably run 3's failure.
3. **0.9 m/s is not a perception limit, it is a clamp.** Re-run it with the
   coupled knobs scaled before concluding anything about top speed.

---

## What was changed in response (2026-09-12)

Commits `1db0c8a` (implementation) and `efaf3a2` (tests). Every number below is
from the logs in `mission_logs/`; nothing here was tuned by taste.

**Takeaway 1 — 0.6 m/s is validated.** `linear_x_cruise` is untouched (0.15
default, set per run from the operator panel), and it is now the IN-ROW speed
and nothing else. TRAVERSE and BACKOUT_TRAVERSE used to read it directly, so a
0.9 m/s test run also crossed the headland blind at 0.9; they have their own
`traverse_speed` (0.5). `exit_clear_speed` 0.10 → 0.25.

**Takeaway 2 — REACQUIRE is the top defect.** It is now two phases. Phase A
latches the row exactly as before; phase B stays in REACQUIRE (logged as
`REACQUIRE (CENTER)`) and steers at 0.15 m/s until `|offset_norm| <= 0.06`
holds over 0.10 m, or warns and hands off at 0.5 m. The residual column in the
table above is the acceptance test: it should read ≤ 0.06 everywhere.
`reacquire_max_distance` → DONE is now reachable in phase A only.

**Takeaway 3 — 0.9 m/s is a clamp, not a perception limit.** `angular_z_max`
is deliberately left at 0.175 for the retry rather than scaled by
`speed_args.py`. Both inputs to that run's overshoot — the 0.237 handoff and
the turn undershoot below — are fixed, so the retry measures the clamp with
nothing else in front of it. The number to report is the % of FOLLOW_ROW frames
at `|angular_z| >= 0.1749`; 28.5% is the figure to beat.

### Two further findings from the same logs

**4. Every turn undershoots by exactly `yaw_tolerance_deg`.** All 12 logged
`TURN_1` legs swept 1.477–1.483 rad, and `pi/2 - 5°` is 1.4835 — the test is
`swept >= pi/2 - tol`, so the tolerance is a one-sided stop-early band and not
a ± band. TURN_1 + TURN_2 therefore handed REACQUIRE a repeatable ~10° heading
bias to absorb, upstream of takeaway 2. Now 1.5°, which at the measured 58 Hz
(`inference_s` median 0.017 s) and the raised 0.7 rad/s turn rate is still 2×
one control tick. ⚠ On the 2 Hz CPU Jackal one tick at 0.7 rad/s is 20° and
this pairing does not port.

**5. The rear-steered EXIT_CLEAR leg never terminates on the rear camera.**
12 of the 13 logged legs ran 1.498–1.520 m — `exit_clear_max_distance` (1.5)
exactly, which the FSM treats as a ceiling that turns anyway. The rear-open
terminator is not firing in the field, so the "positive evidence that the tail
has cleared the last plants" the design is built around is not being collected;
the leg is an open-loop 1.5 m in practice. **Not addressed here** — raising
`exit_clear_speed` makes that leg 2.5× faster but does not change its length.
It needs its own session with the rear debug view (`(REAR)` on the HUD) and the
`exit_clear_detector` status line.

### The leaf-occlusion deadlock (runs 102012 and 102256)

Both back-out test runs contain the same sequence, and it is not a back-out
test result — it is a false positive:

| index | state | trav | obst | distance_in_row | linear_x |
|---|---|---|---|---|---|
| 114 | FOLLOW_ROW | 0.131 | 0.868 | 2.540 | 0.5 |
| 119 | FOLLOW_ROW | 0.000 | 1.000 | 2.640 | **0** |
| 139 | FOLLOW_ROW | 0.001 | 0.999 | **2.798** | 0 |
| 209 | FOLLOW_ROW | 0.008 | 0.992 | **2.798** | 0 |
| 214 | BACKOUT | 0.001 | 0.999 | 2.798 | 0 — `BLOCKED` |

`distance_in_row` is frozen at 2.798 m for 90 frames. The robot stopped, which
is the one response that guarantees the leaf never leaves the view, and 4 s
later backed out of a row that was not blocked. `STATE_NUDGE` now creeps 0.12 m
and looks again, up to twice, before the back-out is allowed to commit — total
forward creep 0.24 m, against a standoff the blocked signature leaves well
clear of a real crop wall. `nudge_max_attempts:=0` restores the old behaviour
for an A/B.
