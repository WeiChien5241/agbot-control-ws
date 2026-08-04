# HANDOFF3.md

Handoff for the P-AgBot vision-nav work, updated end of session 2026-08-04.
Field status: in-row nav + headland turns WORK on the real robot.
Sim status: blocked-row BACK-OUT now works end to end.
Hardware status: GPU robot cpr-j100-0864 now runs the WDR camera FRONT (low
mount) + a Logitech Brio REAR, and is updated to `0936f3b`.

---

## 0c. SESSION 2026-08-04 — per-run performance metrics

No algorithm changed. A grad student asked for performance numbers from
testing — distance from the centerline, tracking error, failure modes — and the
pipeline had no way to produce them.

### Why the console log could not answer it

Every quantity needed is already computed on every frame in `_process_frame`.
None of it was written down. The console only ever sees a throttled prose
summary: `timing:` every 5 s, the detector line at 1 Hz while a meter moves.
That is **one frame in ten on the 2 Hz CPU robot and roughly one in 120 on the
GPU robot at ~24 Hz**. An RMS or a p95 from a 5 %-duty sample is not a
measurement — and this exact throttling already made a partially-firing exit
signature read as no signature at all (§0 item 8). Mining `rosout` is the wrong
instrument; the fix is a file.

### What was added

- **`src/agbot_vision_nav/metrics_logger.py`** — rospy-free. `COLUMNS` is the
  one schema definition; `RunMetricsLogger` writes one row per processed frame
  and **never raises** (it runs on the inference thread — an instrument that can
  kill the node it measures is worse than none). Flushes every 20 rows *and* on
  every event row, because field runs end by losing power as often as by Ctrl-C.
  `summarize()` defines the statistics once, so the node's shutdown log and the
  offline report cannot disagree.
- **`scripts/vision_nav_node.py`** — one `log()` call after the existing
  `record_inference()`, plus event marks: `INFERENCE_FAILED`, `EXIT_REVOKED`,
  `BLOCKED`, `MISSION_DONE`, `WATCHDOG_ZERO`. The watchdog mark is
  **edge-triggered** — it fires at 10 Hz while stale, so a level-triggered mark
  would flood the file and bury every other event. On shutdown the node reads
  its own CSV back and logs the summary, so a console log always carries its
  headline numbers.
- **`scripts/analyze_run.py`** — no ROS, stdlib only. Tracking error per
  mission state, control effort, chatter, perception health, timing, and a
  failure table where each event carries the perception state that caused it.
  Multiple CSVs print side by side.
- **22 new tests; 113 → 135 pass.**

### Running it

ON by default (`metrics_csv_dir: ~/agbot_logs` in `params.yaml`) — a run you
forgot to instrument is a run you cannot report on, and a field pass does not
come round twice. The startup config block prints the CSV path.

```bash
# skip recording one run (an empty launch arg means "use the yaml", so the
# off-switch is the literal string "none")
roslaunch agbot_vision_nav vision_nav.launch metrics_csv_dir:=none ...

python3 agbot_vision_nav/scripts/analyze_run.py ~/agbot_logs/vision_nav_*.csv
```

### ⚠ The units caveat — carry this into any writeup

`offset_norm` is **normalized IMAGE space, not meters**. It is the right
control-loop error (it is what the MPC minimises) but it is *not* "the robot was
8 cm off the row centerline", and it **shifts with camera mount height** (~0.5
tall vs ~0.7 low at the near scan row). Never compare a number from one rig
against a number from another; always state which robot and mount produced it.
Quote the **FOLLOW_ROW** row of the per-state table — TURN/TRAVERSE are odometry
open loop and their offsets describe the headland, not the controller.

A figure in real meters needs either Gazebo ground truth against the row
geometry in `agbot_bringup/config/agbot_maize_small.yaml`, or a per-mount
pixel→meter scale from the tape calibration in next-step #3. Neither was
invented here.

### Still open after this session
- **Not yet run on a robot or in sim** — the module and report are unit-tested
  and were exercised end-to-end on a synthetic CSV, but no real run has produced
  a file yet. First sim mission is the test: row count should track
  `frames_processed`, `state` should walk the FSM, and Ctrl-C should print the
  summary.
- Everything in §0b's and §0a's "Still open" lists, unchanged.

---

## 0b. SESSION 2026-08-03 — camera config + robot deployment

No algorithm changed today. This was the hardware decision, the launch plumbing
it needed, and getting `9d54d59`/`191274c`/`4bf7a6f` onto the GPU robot, which
had been sitting on `3563fd9` since 2026-07-28.

### The camera decision is SETTLED (commit `0936f3b`)

After field testing: **original 5MP WDR camera on the LOW FRONT mount** drives
row-following; **the Logitech Brio moves to the REAR** for the blocked-row
back-out. The front-Brio experiment is over — its slot stays in
`cameras.launch` but is disabled.

That means the front camera source is back to the real-robot default, so the
run command LOSES its `camera_topic:=` override and GAINS the rear flag:

```bash
# before (front Brio)
roslaunch agbot_vision_nav vision_nav.launch \
  camera_topic:=/brio_front/image_raw/compressed mission_enabled:=true num_rows:=3
# now
roslaunch agbot_vision_nav cameras.launch brio_rear:=true          # terminal 1
roslaunch agbot_vision_nav vision_nav.launch \
  mission_enabled:=true num_rows:=3 rear_camera_enabled:=true      # terminal 2
```

### The rear topic name must match on BOTH sides — the silent-failure trap

`cameras.launch` names the rear node `brio_rear`, so it publishes
`/brio_rear/image_raw/compressed`. `vision_nav.launch:153` previously computed
the real-robot `rear_camera_topic` default as `/usb_cam_rear/image_raw/compressed`
(the name in the original commented-out block). **Both were edited to
`/brio_rear`.**

⚠ This pair is load-bearing and fails SILENTLY: a mismatch subscribes to a
topic nobody publishes, nothing errors, and the gap only surfaces at the first
blocked row — when the robot is stopped in front of an obstacle and the rear
frames it needs to reverse never arrive. Rear frames are consumed ONLY in
STATE_BACKOUT, so a dead rear camera is invisible for an entire otherwise-normal
mission. **Always `rostopic hz` the rear topic before a run**, not after.

Same trap caught a typo: the rear `<node>` had `naem=` instead of `name=`, which
aborts the whole launch file (a `<node>` requires `name`).

### Not applied — two known rough edges in `0936f3b`

Both were proposed and consciously left; fix them the next time this file is
touched:

1. **`brio_rear` still defaults to `false`** — every run needs `brio_rear:=true`
   or the rear camera silently never comes up. Flipping the default to `true`
   matches the settled config.
2. **`brio_front_device` and `brio_rear_device` name the SAME serial**
   (`2512LVC36YS8`); the second Brio (`2511LVF11WK8`) sits on a commented line,
   deliberately kept as a record of the other unit we own. Harmless while
   `brio_front` is false, but `brio_front:=true` on this robot would put two
   `usb_cam` nodes on one `/dev` node — the second dies with a device-busy /
   select-timeout error that reads like a USB bandwidth problem.

### Empty rosbags: the topic name, not the command

`rosbag record -O bag.bag /camera/image_raw/compressed` on the ROBOT produced a
valid but empty bag. `/camera/image_raw` is the GAZEBO topic; the real robot
publishes `/usb_cam/image_raw/compressed`. **`rosbag record` neither errors nor
warns on a topic that does not exist** — it subscribes and waits. Check with
`rostopic list | grep image_raw` first, and `rosbag info` after (an empty bag is
~4 KB). Two other silent-empty causes worth ruling out: a `.bag.active` file
left by a non-Ctrl-C kill (`rosbag reindex` it), and a shell whose
`ROS_MASTER_URI` does not point at the robot.

### Offline update flow to cpr-j100-0864 (no internet on the robot)

`origin` on the robot points at a stale bundle FILE, so `git pull` falsely
reports "already up to date" and `git status` claims the branch is "ahead of
origin/main by N commits". Both are artifacts — ignore them. Real flow:

```bash
# laptop
git bundle create ~/agbot-$(date +%Y%m%d).bundle main
# robot, after copying the bundle over
cd ~/agbot_control_ws/src
git diff agbot_vision_nav/launch/cameras.launch   # LOOK before discarding
git bundle verify ~/agbot-20260803.bundle
git stash push -m "robot-local cameras.launch pre-20260803"
git fetch ~/agbot-20260803.bundle main:refs/remotes/bundle/main
git merge --ff-only bundle/main
cd ~/agbot_control_ws && catkin build && source devel/setup.bash
```

The robot used to carry `cameras.launch` as a permanent local diff (its own
by-id paths), which made every update a stash dance. **The camera config now
lives in the repo** and the robot's tree is clean; keep it that way. Per-robot
serials stay overridable as launch args, so a second robot passes
`brio_rear_device:=...` rather than editing the file.

### Deployment verified on the robot

`roslaunch --dump-params` on cpr-j100-0864 after the merge confirmed all three
things this update was for:

- `blocked_min_obstacle_fraction: 0.2` — the back-out deadlock fix is live (the
  old `blocked_min_traversable_fraction` name is absent, as intended)
- `traverse_distance: 0.6`, `headland_clearance: 1.0` — reading from
  `params.yaml`, so the source-of-truth fix took
- `rear_camera_enabled: True`, `rear_camera_topic: /brio_rear/image_raw/compressed`

⚠ That dump also showed `cmd_vel_topic: /vision_nav_check` — a deliberate
dry-run diversion passed on the command line, NOT the `params.yaml` value
(`/cmd_vel`). The robot cannot move in that configuration. Drop the arg for a
driving run; see §3/§6 on why `cmd_vel_topic` must stay `/cmd_vel`.

**A parameter dump only proves what LOADED.** Three runtime checks it cannot
cover, all of which fail quietly:
`Model loaded on device: cuda:0` at startup (a `cpu` fallback costs ~10x),
`rostopic hz` on BOTH camera topics, and `rostopic hz /odometry/filtered` (no
odometry ⇒ the exit detector never arms ⇒ the robot never leaves FOLLOW_ROW).

### Still open after this session
- The full mission has NOT been re-run in real corn on the new camera pair.
  The low front mount is not new to this robot, but the rear Brio has never
  driven a back-out outside sim.
- Everything in §0a's "Still open" list, unchanged.

---

## 0a. SESSION 2026-07-30 — read this first

**Field result: the row-exit rebuild WORKED.** The GPU robot re-ran the route
that previously produced a mid-row false `EXIT_CLEAR` and had **no false
detections**. Next-step #2 from the previous session is CLOSED. One defect
remained: after a real exit the robot began its headland turn too early and
would have hit corn without an operator stop → `headland_clearance` 0.75 → 1.0.

**Sim result: the blocked-row BACK-OUT now works end to end** (previous
next-step #4, never before validated). Getting there required a real fix:

### The blocked-row deadlock (fixed, commits `191274c` + `4bf7a6f`)

Box placed mid-row-2, `num_rows:=2`, rear camera on. The robot saw the box,
stopped, and **never backed out**. Log:

```
t=258.2  blk 0.6/4.0 s  rows=0  frac=0.03   <- banking evidence
t=263.4  blk 0.0/4.0 s  rows=0  frac=0.00   <- drained to zero
t=268.5+ blk 0.0/4.0 s  rows=0  frac=0.00   <- deadlocked
```

`blocked_signature` required `traversable_fraction >= 0.02` as proof the view
was real rather than garbage. **But a blocker at close range leaves zero visible
ground** — the signature disqualified itself at the exact moment it was most
certain, and the symmetric leak drained the banked seconds back to zero.

⚠ **This threshold had already been walked 0.15 → 0.08 → 0.02 chasing the same
symptom (see §2 back-out iteration 1). No value can work** — the correct reading
in front of a real blocker is genuinely 0.00. The quantity was wrong, not the
number.

**Fix:** `blocked_min_traversable_fraction` → **`blocked_min_obstacle_fraction`**
(0.2), measured on the OBSTACLE class over the same lower-half slice. It rises as
the robot approaches instead of falling, and still rejects a dead camera or an
all-sky frame. `centerline_estimator` now reports `obstacle_fraction` (defaulted
None, same pattern as the flank fields). The old kwarg raises `TypeError`.
The detector line prints `trav=` and `obst=` together — the pair distinguishes a
healthy blocker (trav falls, obst climbs) from a garbage frame (both zero).
Tripwires: `test_blocked_fires_when_obstacle_fills_the_view`,
`test_blocked_evidence_survives_ground_fraction_reaching_zero`,
`test_blocked_held_off_when_nothing_is_actually_there`.

### Config: params.yaml is now the source of truth (commit `9d54d59`)

⚠ **Gotcha 3 is GONE — the behaviour is inverted from what older notes say.**
44 knobs were declared in both `config/params.yaml` and the launch file, and the
launch `<param>` (loaded after `<rosparam file>`) silently won every time.
Editing the yaml did nothing. It bit twice: `headland_clearance` set to 1.5 in
the yaml while the robot ran 0.75, and `traverse_distance` reading 0.65 while
**every run to date used 0.6**.

Launch `<arg>`s now default to EMPTY with conditional `<param>` tags, so
**`params.yaml` holds the tuning and a launch arg overrides it only when
actually passed**. Verified a pure no-op by diffing `roslaunch --dump-params`
across the default/sim/mission configurations. Five keys stay launch-owned:
`model_path` and the four camera-topic args computed from `sim:=true|false`.

⚠ `traverse_distance` is pinned to **0.6** in the yaml — the value everything
was actually tested at. 0.6 vs 0.65 remains the open field question it was.

### Node logs its resolved config at startup (commit `ceb053e`)

One block before `vision_nav_node ready`, read back from the private parameter
namespace, so it shows the real merge of yaml + launch + command line. Read it
to confirm a knob took effect; it caught the `traverse_distance` mismatch on its
first run.

### Rear camera mirrored to the low mount (commit, agbot_bringup)

The front camera had moved to the front deck (`z=0.025`) while the rear stayed
on the 0.225 m stand. Now `front x=+0.19, rear x=-0.19`, same height, exact
mirror. **Load-bearing:** the back-out steers from the rear camera reusing the
MPC with UNCHANGED signs, which assumes the reverse view is the geometric twin
of the forward view. The tall mount is kept commented as a record.

**113 tests pass.**

### Still open after this session
- `traverse_distance` 0.6 vs 0.65 — decide with a run.
- The `open 0.44/0.40 m armed o:Y` line one frame after entering row 2
  (2026-07-30 log). Almost certainly a stale `last_status` printed before the
  row-entry reset, since it did not fire and re-armed normally — but if
  `update()` can run with a post-reset distance while the accumulator still
  holds the previous row's evidence, that is a latent false-exit path.
- Joystick takeover (below) — still unreproduced.
- GPS RTK for trailer→row transit: plan written up in `GPS_plan.md`.

Session 2026-07-24 brought up the NVIDIA-GPU robot and found a mid-row
false-EXIT_CLEAR bug (low camera + fast inference). Session 2026-07-28 found
the deeper cause — a frame-counted debounce — **rebuilt row-exit detection
around physical units**, sim-tested it, fixed the two failures that exposed
(an accumulator that leaked as fast as it filled, and a REACQUIRE latch keyed
on a camera-height-specific width), and validated a full 3-row mission. It also
investigated the joystick-takeover safety item, failed to reproduce it, and
reverted the attempted fix.

§0 has the complete list of what changed. Written so a fresh session picks up
with zero context. Supersedes all previous HANDOFF3 content.

---

## 0. START HERE — state of play (end of session 2026-07-28)

**Row-exit detection was rebuilt today and is SIM-VALIDATED**: a full 3-row
mission ran clean (`Mission DONE: rows_driven=3, blocked rows: none, revoked
exits: 0`). Nothing about it is field-validated yet — that is next step #2.
The joystick-takeover safety item is an OPEN QUESTION, not a fixed bug: the
attempted fix was reverted (see DEFERRED below) so the field re-test runs on
exactly the sim-validated configuration.

| Commit | What |
|---|---|
| `a9c6ed3` | Debounce in meters/seconds not frames; revocable EXIT_CLEAR; strip-occupancy flank test; back-dated headland; split exit scan rows |
| `3fd99ad` | Asymmetric leak (the meter must fill faster than it drains); REACQUIRE latches on corn not width; 1 Hz diagnostics |
| `971ff55` | Sim validation record — **this is the known-good tree** |
| `291e0db`, `a9ace39` | Joystick override + single-publisher — **REVERTED** by `1500fa5` |

**113 tests pass.** `cd agbot_vision_nav && PYTHONPATH=src python3 -m pytest test/ -v`

---

### What changed in row-exit detection today — the full list

Read this before touching the detector or the FSM. Every item is a behaviour
change with a knob and a tripwire test.

**1. Debounce is in PHYSICAL UNITS, never frames.** `exit_detect_frames=5` was
2.5 s on the 2 Hz CPU robot and 0.2 s at ~25 Hz — the same constant meaning two
different things, which is how a mid-row gap committed the GPU robot into the
corn. OPEN now accumulates **meters travelled** (`exit_confirm_distance` 0.4 m);
BLOCKED accumulates **seconds** (`blocked_confirm_seconds` 4.0 s). The unit
split is load-bearing: a blocked view stops the robot, so a distance counter
would never fill and the back-out would deadlock. Defaults reproduce the
field-proven CPU-robot timings at any rate. Tripwire:
`test_open_fires_at_same_distance_at_any_frame_rate`.

**2. The OPEN meter drains slower than it fills** (`exit_leak_ratio` 0.5).
Symmetric leak sounds neutral but is not: a signature true HALF the frames nets
exactly zero and can **never** fire, however far the robot drives. That is
precisely a real-but-marginal exit — the sim row end labels the ground
traversable but imperfectly (width ~0.8–0.9, patchy edges), and the meter
reached 0.13 of 0.40 m and drained back while the robot headed for the world
edge. At 0.5, anything true more than ~1/3 of the time still climbs; a short
mid-row gap still drains away. Tripwires:
`test_marginal_exit_fires_despite_a_flickering_signature` plus its
counter-example `test_symmetric_leak_reproduces_the_failure`.
  - *Sub-bug this exposed:* the first frame of a streak used to credit no
    distance. With a flickering signature the accumulator returns to zero
    between bursts, making **every** open frame a "first" frame that banks
    nothing — a hard deadlock. The streak now starts at the previous sample.
  - ⚠ *Cost of a marginal signal:* net fill rate is `1.5*duty - 0.5`. At 50%
    duty, 0.4 m of evidence needs ~1.6 m of driving (measured: 1.45 m). At 70%,
    ~0.7 m. **Raise signal quality rather than loosening the leak further.**

**3. EXIT_CLEAR is REVOCABLE** for its first `exit_revoke_distance` (0.5 m): if
the **nearest scan row** stays corn-flanked for `exit_revoke_fail_distance`
(0.25 m), the FSM falls back to FOLLOW_ROW and un-counts the row. A false exit
is now a steering wobble instead of a collision. **Only the near row is
consulted** — during a genuine exit the FAR rows legitimately see the corn block
across the headland, so a global "corn beside the corridor" test would revoke
every real exit. Tripwire: `test_far_row_corn_across_headland_does_not_revoke`.
A revoked exit must NOT reset the row-entry pose (see §6 gotcha 1c).

**4. `headland_clearance` is back-dated to where the exit was FIRST seen**, so
the confirmation distance is not added on top — otherwise the robot overruns by
0.4 + 0.75 = 1.15 m. `exit_clear_min_distance` (0.2 m) is always driven anyway,
which also stops a slow marginal confirmation from compounding the overshoot.

**5. The flank test measures outer-strip OCCUPANCY, not edge-reach.** The
corridor scan stops at the FIRST non-traversable column, so one stray
misclassified pixel used to veto a real exit — a live risk on the thinly-trained
low-camera masks. Now the outer `exit_flank_edge_margin` (0.05) strip on each
side must be ≥ `exit_flank_min_clear_fraction` (0.8) traversable. Computed in
`estimate_centerline`, carried on `ScanRowResult` as defaulted fields, so
hand-built results still take the old path. Tripwire:
`test_stray_edge_pixel_does_not_veto_a_real_exit`.
  - **Be honest about what the flank rule is:** reaching within 0.05 of both
    edges implies width ≥ 0.90, so it is mostly a stricter width bar. Its real
    addition is rejecting ONE-SIDED openings (corridor at the left edge, corn
    still on the right) — exactly what a gap in a single row of corn looks like.
    That is why it is kept.

**6. REACQUIRE latches on CORN ON BOTH SIDES of the near scan row**, not on
corridor width. The old `mean width < reacquire_max_width (0.6)` was a
camera-height constant (~0.5 tall, ~0.7 low) and could never be satisfied
*inside a row* on the low mount; it also required `traversable_fraction >= 0.10`
while the sim headland measured 0.09. The FSM therefore crept the full 2.0 m at
0.08 m/s — **25 seconds** — with `angular_z` hard-coded to 0.0, holding whatever
lateral error the turn left behind, and nearly drove into the corn. It now
reuses `nearest_row_flank_clear()` inverted, confirms over
`reacquire_confirm_distance` (0.12 m) of travel, and **steers while creeping**.
`reacquire_max_width` and `reacquire_frames` are GONE. Tripwire:
`test_low_camera_row_latches_reacquire`.

**7. The exit detector can use its own scan rows** — `exit_scan_row_fractions`
(empty = share the steering rows). Steering wants lookahead spread; exit
detection wants maximum in-row vs open-field separation, and the best rows
differ per camera mount. Lets the exit rows move UP for the low camera without
touching field-proven steering. Rear stays on the steering rows (the FSM uses
one rear result both to watch for the exit behind and to steer the reverse leg).

**8. Diagnostics, because this cost a whole session to interpret.** The detector
line prints at **1 Hz whenever the meter is moving** (was 5 s, so at 2 Hz only
1 frame in 10 was visible — a partially-firing signature looked like no
signature at all). It now carries `near w=` and `edges=`: without them, "width
under the bar" and "edges under the bar" both render as `openrows=0` with no way
to tell which knob to turn. A revocation logs `EXIT REVOKED: row N at X m`
unthrottled, and the DONE line reports the revoked count.

#### Knob changes at a glance

| Added | Default | Removed |
|---|---|---|
| `exit_confirm_distance` | 0.4 m | `exit_detect_frames` |
| `blocked_confirm_seconds` | 4.0 s | `blocked_detect_frames` |
| `exit_leak_ratio` | 0.5 | `reacquire_max_width` |
| `blocked_leak_ratio` | 1.0 | `reacquire_frames` |
| `exit_detect_min_frames` | 2 | |
| `exit_flank_min_clear_fraction` | 0.8 | |
| `exit_revoke_enabled` / `_distance` / `_fail_distance` | true / 0.5 / 0.25 | |
| `exit_clear_min_distance` | 0.2 m | |
| `exit_scan_row_fractions` / `_weights` | [] (share steering rows) | |
| `reacquire_confirm_distance` | 0.12 m | |
| `reacquire_steering_enabled` | true | |

The removed names raise `TypeError` if passed rather than being silently
ignored — deliberate.

---

### ✅ SIM-VALIDATED (2026-07-28): full 3-row mission
`Mission DONE: rows_driven=3, blocked rows: none, revoked exits: 0`. Smooth
driving, all three exits fired at the right place, no world-edge run, REACQUIRE
latched normally at both turns. Two numbers worth keeping:

- **Exit meter duty ~0.87, not 0.5.** Each exit banked its 0.4 m in ~0.5 m of
  driving (2–3.5 s). Inverting `net = 1.5*duty - 0.5` gives ~0.87 — the real sim
  row-end signal is far better than the 50% worst case, with plenty of margin
  over the ~1/3 the asymmetric leak needs. No further loosening warranted.
- **The flank gate is what prevents mid-row false exits, confirmed with
  numbers.** Mid-row samples regularly showed `wide=1` at `near w=0.80–0.97`,
  i.e. AT OR ABOVE `exit_width_threshold`. The standout, t=209.96:
  `wide=1 openrows=0 near w=0.97 edges=1.00/0.38` — width 0.97 **mid-row**,
  which a width-only rule would have fired on even at a 0.9 bar. The right edge
  at 0.38 rejected it. `openrows` stayed 0 for every such frame.

⚠ **Margin note:** mid-row widths run close to the threshold, so
`exit_width_threshold` is NOT what holds the line — the edges are. Don't relax
`exit_flank_min_clear_fraction` without re-checking mid-row `edges=` values. The
revocation backstop never had to fire this run and should stay that way.

**If a future run's exit will not fire**, read `near w=` / `edges=` and lower
`exit_flank_min_clear_fraction` (edges under 0.8) or `exit_width_threshold`
(width under 0.8) — do NOT reach for `exit_leak_ratio` first.

---

### Decisions this session — do not re-propose without new data
- **No odometry row-length fallback**, and **no re-reading "lost view past the
  row end" as an exit.** Consequence, recorded once: a genuine segmentation
  failure over open ground still has nothing catching it, and a blocked signal
  with no rear camera still ends the mission where it stands.
- **No adaptive/relative width threshold.** Would learn the median in-row width
  and fire at ~1.35x baseline, auto-calibrating across mounts. Unnecessary while
  the flank rule pins the bar near 1.0, and it adds state that fails silently.
  Revisit only if per-mount tuning proves painful.
- **No offline mask-vs-prediction harness.** The reported widths are accurate
  (the low camera genuinely reads ~1.0 at a real row end and ~0.83 at a gap), so
  this is detector logic, not segmentation quality.
- **Sim domain gap is accepted, not worked around.** The sim's open ground
  segments imperfectly; the real world is reported fine. That is why item 2
  above (asymmetric leak) was the right fix rather than a sim-specific hack.

---

### Prior session (2026-07-24): mid-row flank-clear fix (commit `bcf7d6d`)
On the new GPU robot (fast inference) the mission FSM falsely fired
`EXIT_CLEAR` **in the middle of a row**, drove into the corn, and — because
the pipeline is so fast — committed before anyone could react. Root cause:
1. That robot runs a **LOW-mounted camera**. Low mount raises the *normal*
   in-row corridor width at the nearest scan row from ~0.5 (tall) to ~0.7.
2. A few **missing corn plants on the sides** push the near-row normalized
   corridor width to ~0.83 — above `exit_width_threshold` (0.8).
3. The OPEN signature was **width-only**, so it read that as open field.
4. Fast inference satisfied the 5-frame debounce almost instantly.

**Fix (matches the user's rule — "look left/right of the corridor; if corn
still flanks it we're still in the row; only fire when it reaches the image
edges on both sides"):** a scan row now counts toward the open exit only if
its corridor is **wide AND flank-clear** — i.e. it reaches within
`exit_flank_edge_margin` (0.05 of image width) of **BOTH** image borders, so
no corn borders the corridor. A mid-row gap widens the corridor but leaves
corn short of the edge → NOT an open row → no false exit. A true row end runs
edge-to-edge → fires. Uses the corridor bounds the detector already has, so
the segmentation model, `centerline_estimator.py`, and `CenterlineResult` are
**untouched**. Rear back-out watcher mirrors the same margin. New knob
`exit_flank_edge_margin` (>= 1.0 disables → pre-fix width-only behavior);
disabled by using a large value. HUD now shows `wide=` and `openrows=`.
**78 tests pass** (72 → 78; +6). All prior tests stayed green, including the
early-approach and world-edge tripwires (open field reaches the edges).

(Superseded in mechanism by `a9c6ed3` above: the flank rule survives but now
measures strip occupancy, and the frame debounce is gone.)

### DEFERRED — two open items

- **SAFETY: joystick takeover — OPEN QUESTION, could not be reproduced.**
  Field report (2026-07-24): a grad student could not override cmd_vel while
  the robot drove into corn; killing the node was the only recourse.

  **Bench re-test 2026-07-28 — it WORKED.** Same GPU robot that failed, robot
  **on the ground and driving** under the vision-nav node, old (unmodified)
  code, `/bluetooth_teleop/joy` steady at **20 Hz**: the joystick took control
  normally. Nothing reproduced.

  **Two theories are DEAD. Do not resurrect them without new data:**
  1. *"Rate-dependent failure ⇒ twist_mux bypassed."* The rate-dependence was
     an INTERPRETATION of the field event ("processing is too fast so the
     joystick can't overtake"), never a measurement. twist_mux arbitrates by
     PRIORITY, not rate (`jackal_control/config/twist_mux.yaml`: bt_joy
     priority 9 on `bluetooth_teleop/cmd_vel`, external priority 1 on
     `cmd_vel`), and the bench test shows it is wired correctly on this robot.
  2. *Bluetooth range/attenuation in the corn.* The operator was standing
     **right next to the robot** during the field failure.

  **Leading candidate now: operator technique.** The field attempt was made by
  a grad student and it is not known whether the L1 deadman was held correctly
  (`teleop_ps4.yaml`: `enable_button: 4`; `teleop_twist_joy` publishes ONLY
  while it is held). A second candidate that fits "couldn't take over" without
  any fault: the robot yields while L1 is held but resumes within 0.5 s of
  release, so control can never be *kept*.

  **Re-test in the field (user driving personally, 2026-07-29).** Capture:
  ```bash
  rostopic hz /bluetooth_teleop/joy      # must hold ~20 Hz for the whole run
  rosnode list | grep -E "joy_node|teleop_twist_joy"
  ```
  then with the robot driving under the node, hold **L1** and steer, and record
  (a) did it respond while held, (b) did it resume immediately on release,
  (c) did the joy heartbeat ever drop. If it works cleanly, the field failure
  was operator technique and this item closes with no code at all.

  **Code was written for this and then REVERTED** (commits `291e0db` +
  `a9ace39`, reverted 2026-07-28): an in-node operator override, a
  single-publisher cmd_vel timer, and a startup warning when `cmd_vel_topic`
  is not `/cmd_vel`. Reverted so the field re-test runs on exactly the
  sim-validated configuration (`971ff55`) and anything observed is
  attributable to the exit-detection work. `git show 291e0db` to resurrect if
  a real cause is confirmed. ⚠ Note the override there fires on *receiving*
  joystick messages, so it would NOT help if the link were ever dead — that
  case needs the inverse (a heartbeat watchdog).

  Superseded analysis in the plan file
  `~/.claude/plans/read-the-handoff3-md-to-squishy-hennessy.md`.
- **GPS RTK (Emlid Reach RS2) for trailer↔row transit.** A separate, larger
  subsystem: fuse GPS via the Jackal's existing `robot_localization` EKF, then
  `move_base` or a GPS-waypoint follower to drive trailer→row and row→trailer.
  Yes, ground robots do RTK waypoint nav much like PX4/QGC on a drone.
  Turning-via-GPS (professor's idea) is optional — vision headland turns
  already work. Not needed for any current bug; future capability plan.

### Prior §0 (blocked-count) — remains FIXED in code (commit `7003a3b`)
`mission_fsm` used to increment `rows_driven` on EVERY exit (including
blocked) and end the mission early. Fixed 2026-07-22: `rows_driven` increments
ONLY on an OPEN exit; a block ALWAYS backs out + S-turns to the next physical
row and never ends the mission on its own. Tests
`test_blocked_row_does_not_count_continues_to_next_row` and
`test_blocked_middle_row_still_requires_full_num_rows` cover it. STILL WANTS a
sim/field confirmation run (`num_rows=2`, blocker in row 2 → back-out → S-turn
→ drive row 3 → `Mission DONE: rows_driven=2, blocked rows: row 2 blocked at
X m`). If the S-turn happens but REACQUIRE fails to latch the next row (a
SEPARATE issue), raise `reacquire_max_distance` 2.0→2.5. (`reacquire_max_width`
no longer exists — REACQUIRE latches on corn flanking the near row; see §0
item 6.)

---

## 1. GOAL

Vision-based navigation for the Purdue P-AgBot (Clearpath Jackal): DINOv3
segmentation → traversability mask → centerline estimation → image-space MPC
keeps the robot centered in a corn row. Multi-row boustrophedon missions
with odometry headland turns; blocked-row back-out via a rear camera.
Row-following and headland turns are FIELD-PROVEN (real corn, 2026-07);
back-out is sim-validated.

---

## 2. CURRENT STATE

### Robots (multiple; camera height is per-robot and matters)
- **GPU robot cpr-j100-0864 (current deployment target):** as of 2026-08-03 on
  commit `0936f3b`, running the WDR camera on the LOW FRONT mount plus a rear
  Logitech Brio (§0b). Updated offline by git bundle; its tree is clean, so
  `git merge --ff-only bundle/main` is the whole update.
- **New GPU robot (this session, 2026-07-24):** fast inference (user reports
  much faster than the CPU robot). Runs a **LOW-mounted camera** — normal
  near-row corridor width ~0.7 (vs ~0.5 tall). The low mount is what caused
  the mid-row false EXIT_CLEAR now fixed (§0). ⚠ **Segmentation caveat:** the
  model was trained on ~300 tall-camera annotations but only ~100 low-camera
  ones, so low-camera masks may be weaker on the sides — more low-camera
  annotations would help and could also reduce the missing-corn misreads.
- **CPU robot cpr-j100-0463:** ~2 Hz CPU inference; live row-following
  achieved. Use `mpc_dt:=0.5`, raise `max_data_age_sec` (~1.5-3.0).
- **Tall-camera field runs (2026-07):** in-row nav + headland turns proven.

### Field behavior (real robot, 2026-07)
- In-row navigation: works. Headland turns: work.
- Full 3-row mission SIM-VALIDATED 2026-07-28 on the small maize world with
  the current defaults (§0). Not yet re-validated in real corn.
- End-of-TRAVERSE nose-too-close fix: `traverse_distance` param (below), NOT
  yet field-re-tested.
- `agbot_camera.urdf.xacro` still carries an UNCOMMITTED working-tree diff
  (tall line active, front-deck line commented, stand at x=-0.025). It is the
  user's state: don't commit or revert unprompted. (The physical GPU robot's
  low camera is a separate real-robot mount, independent of this sim URDF.)

### Sim back-out validation (2026-07-21, four iterations — all committed)
1. Blocked never fired at a big box: ground-fraction gate too strict up close.
   `blocked_min_traversable_fraction` 0.15 → 0.08 → **0.02** (since REPLACED
   outright by `blocked_min_obstacle_fraction` — see §0a; HUD showed
   frac=0.04 in front of the box; 0.0 disables it). Blocked debounce made
   LEAKY (noise frames decrement, not reset, the counter).
2. Reverse leg overshot past the row entrance: BACKOUT unwound the full
   odometry d_block, which includes the PRE-ROW approach (row 1's FOLLOW_ROW
   starts at the SPAWN point ~2 m before the row). Fix: **rear-exit watcher** —
   during BACKOUT the rear centerline runs the open-exit signature (armed
   immediately, `MissionFSM.rear_exit_detector`); reversing ends as soon as the
   row opens up behind; d_block stays the upper bound. `reacquire_max_distance`
   1.5 → 2.0.
3. Confirmed in sim: blocked fires, reverse steering correct (no negation),
   early rear-exit stop works.
4. §0 blocked-count fix (2026-07-22) + mid-row flank-clear fix (2026-07-24).

### Prior instrumentation (all committed + pushed, earlier sessions)
- **Timing** (`src/agbot_vision_nav/timing_stats.py`, rospy-free): 5 s-throttled
  `timing:` line + HUD line. **`inf` = wall-clock around ONLY `model.predict()`
  (`time.monotonic()`), mean + p95 over the last 50 inferences.** **`dropped %`
  = `1 − frames_processed/frames_received`, cumulative since startup** — with
  latest-frame-wins semantics a frame that arrives while inference is busy is
  overwritten and never processed, so it counts frame *skipping*, not lost
  commands; high % is BY DESIGN (camera outruns inference). Control rate = the
  `proc` Hz figure. Sim laptop: ~5.7 Hz / 165 ms inf / 288 ms e2e.
- **`model_device` param (default `auto`):** SegmentationModel moves to CUDA
  when available; node logs `Model loaded on device: ...`. **Check that line on
  the GPU robot first** — `cpu` there means torch has no usable CUDA.
- **`scripts/benchmark_inference.py`** (no ROS): cross-machine inference table.
- **`traverse_distance`** (default 0.6 m): TRAVERSE/BACKOUT_TRAVERSE drive this
  instead of `row_spacing` (0.75). ⚠ Precedence gotcha (see §6).

### Detector semantics (row_exit_detector.py, current)
- OPEN: ≥ `exit_open_rows_required` (1) scan rows — ANY of them — that are BOTH
  wide (corridor width ≥ `exit_width_threshold` 0.8) AND **flank-clear** (the
  outer `exit_flank_edge_margin` 0.05 strip on EACH side is ≥
  `exit_flank_min_clear_fraction` 0.8 traversable). Confirmed over
  `exit_confirm_distance` (0.4 m) of **travel**, leaky, floored at
  `exit_detect_min_frames` (2) frames; armed after `min_in_row_distance`
  (2.0 m). `exit_flank_edge_margin >= 1.0` disables the flank term
  (width-only). Helpers: `flank_clear_flags()`, `nearest_row_flank_clear()`,
  `open_streak_start` (feeds the FSM's back-dating).
- BLOCKED: zero corridors at ALL scan rows + `traversable_fraction` ≥
  `blocked_min_obstacle_fraction` (0.2) — measured on the OBSTACLE class,
  NEVER on traversable ground (see §0a), accumulated over
  `blocked_confirm_seconds` (4.0) of **elapsed time** with a LEAKY counter
  (`blocked_leak_ratio` 1.0 = symmetric), armed after
  `blocked_arming_distance` (0.3 m). Seconds, not meters — the
  robot is stopped by then.
- `update()` takes `now=` (the node passes `rospy.get_time()`, which follows
  sim time under Gazebo; the default `time.monotonic()` is for tests).
- `last_status` exposes `wide_rows`, `open_rows`, `open_distance`,
  `blocked_seconds`, `open_streak_start`, `near_row_flank_clear`, and
  `near_row_width` / `near_row_edges` (which threshold is blocking an exit).

### Mission FSM (mission_fsm.py, current)
REACQUIRE latches on the near scan row having corn on BOTH sides (NOT a width
threshold — that was camera-height-specific and unlatchable on the low mount),
confirmed over `reacquire_confirm_distance` (0.12 m), steering while creeping.
FOLLOW_ROW → EXIT_CLEAR (0.10 m/s, `headland_clearance` 0.75) → TURN_1 →
TRAVERSE (`traverse_distance`) → TURN_2 → REACQUIRE → FOLLOW_ROW;
boustrophedon sign flip per transition. Blocked branch: BACKOUT (reverse,
rear-steered, ends on rear open-exit OR d_block bound) → BACKOUT_CLEAR →
BACKOUT_TURN_1 → BACKOUT_TRAVERSE → BACKOUT_TURN_2 (counter-rotate, S-shape) →
REACQUIRE with ONE suppressed sign flip; next row same world direction. A
blocked row does NOT count toward num_rows; only open-exit rows count. Gated:
`backout_enabled` = mission_enabled AND rear_camera_enabled; without rear
camera a blocked signal stops + DONE + report. Rear steering signs UNCHANGED
(mirror × reverse = identity). `rear_exit_detector` now also mirrors the
front `exit_flank_edge_margin`.

### Node (vision_nav_node.py)
Single-slot latest-frame buffers (front + rear share one Condition), one
inference thread, rear frames consumed ONLY in STATE_BACKOUT, 10 Hz watchdog
(hold last cmd until `max_data_age_sec`, then zero). Publishes to `cmd_vel_topic`
(default `/cmd_vel`). Debug topic `/vision_nav_node/debug/image` (during BACKOUT
shows the REAR camera; no separate rear debug topic). Zero-code rear preview:
`camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false
cmd_vel_topic:=/cmd_vel_rear_preview`.

---

## 3. KEY DECISIONS (do not re-litigate without new information)

- **Never debounce in frames** (2026-07-28). Any frame count means a different
  thing on every robot, and the fleet spans 2 Hz to ~25 Hz. Use meters for
  anything confirmed by driving, seconds for anything confirmed while
  stopped. Tripwire: `test_open_fires_at_same_distance_at_any_frame_rate`.
- **Leaky, never strictly-consecutive, for distance/time debounces.** At
  25 Hz a 0.4 m window is ~65 frames; a reset-on-any-dropout rule would never
  complete. Tripwire: `test_open_tolerates_a_dropout_frame`.
- **The OPEN leak must be ASYMMETRIC** (2026-07-28). Draining as fast as it
  fills means a signature true half the frames nets zero and can never fire —
  which is what a real-but-marginal exit looks like. Tripwires:
  `test_marginal_exit_fires_despite_a_flickering_signature` and its
  counter-example `test_symmetric_leak_reproduces_the_failure`.
- **Never latch REACQUIRE on corridor WIDTH** (2026-07-28). Width is
  camera-height-specific (~0.5 tall, ~0.7 low) and the old 0.6 bar was
  unlatchable inside a row on the low mount. Ask whether corn is on both
  sides. Tripwire: `test_low_camera_row_latches_reacquire`.
- **Revocation reads the NEAREST scan row only** (2026-07-28). The far rows
  legitimately see the corn block across the headland during a genuine exit,
  so "corn beside the corridor" as a global test would revoke every real exit.
  Tripwire: `test_far_row_corn_across_headland_does_not_revoke`.
- **Open exit = wide AND flank-clear** (2026-07-24, refined 2026-07-28). Be
  honest about what this is: reaching within 0.05 of both edges implies
  width ≥ 0.90, so it is mostly a stricter width bar. Its real addition is
  rejecting ONE-SIDED openings (corridor at the left edge, corn still on the
  right) — which is exactly what a gap in a single row of corn looks like.
  Kept for that. Now measured as outer-strip occupancy so a stray pixel can't
  veto it. Tripwires: `test_open_blocked_by_flank_corn_mid_row_gap`,
  `test_one_sided_edge_does_not_fire`,
  `test_stray_edge_pixel_does_not_veto_a_real_exit`.
- **Exit detection: open = "ANY N rows wide", never specific/farthest rows.**
  Beyond the field edge, far scan rows can stay invalid FOREVER (garbage
  segmentation); a farthest-rows criterion never fired and the robot drove off
  the world edge. Tripwire: `test_open_fires_when_only_near_row_wide`. The
  flank-clear gate does NOT re-couple to far rows — it's per-row on whatever
  row is wide.
- **Blocked ground-fraction gate ≈ 0** (0.02): up close a blocker fills the
  frame (frac=0.04); the `blocked_confirm_seconds` (4.0 s) debounce is the real
  occlusion guard.
- **BACKOUT ends on rear-camera open-exit, odometry as upper bound only.**
- **Rear-steering signs: NO negation** (mirror × reverse cancel). Don't "fix"
  without failing the unit test first.
- **Back-out geometry: S-turn, same-direction next row, suppress ONE flip.**
- **Rear inference only during STATE_BACKOUT.**
- **No rear camera ⇒ blocked = stop + DONE + report** (user's choice).
- Image-space MPC (scipy SLSQP, N=8), no EKF; `mpc_dt` scales alpha/beta/
  rate-limit (0.1 s reference; CPU robot uses mpc_dt:=0.5).
- `mission_enabled` defaults false. Small maize world for sim
  (`switch_maize_world.sh full|small`). Camera URDF injection via
  `load_robot_description.sh` (JACKAL_URDF_EXTRAS). Go-home deferred.
- **Joystick takeover: keep the node on `/cmd_vel`** (twist_mux `external`,
  priority 1); the joystick at priority 9 outranks it. Do NOT point
  `cmd_vel_topic` at the controller/teleop topic — that bypasses the mux. NOTE
  this is correct-by-design, not the cause of the field failure: the mux was
  bench-verified working on the GPU robot 2026-07-28 (see §0 DEFERRED).

---

## 4. FILES

### `agbot_vision_nav/`
| File | Purpose |
|---|---|
| `src/agbot_vision_nav/segmentation_model.py` | lightly_train wrapper; classes 0=sky,1=traversable,2=obstacle; `device=` param + `device_str`. |
| `src/agbot_vision_nav/centerline_estimator.py` | 3 scan rows outward from center column → `CenterlineResult`; also reports per-row outer-strip traversable fractions (`left_clear_frac`/`right_clear_frac`, defaulted None) for the flank test. |
| `src/agbot_vision_nav/controller.py` | `MPCRowController` (SLSQP, N=8, dt-scaled). Reused unchanged for rear reverse steering. |
| `src/agbot_vision_nav/row_exit_detector.py` | OPEN (wide AND flank-clear; strip occupancy via `flank_clear_flags()`) debounced in METERS, BLOCKED debounced in SECONDS, both leaky; `nearest_row_flank_clear()` for revocation; `open_streak_start` for back-dating; per-signature arming; `last_status`. |
| `src/agbot_vision_nav/mission_fsm.py` | Mission FSM incl. BACKOUT branch, `rear_exit_detector`, `backout_progress()`, `traverse_distance`, `blocked_events`; EXIT_CLEAR back-dating + revocation (`_revoke_exit()`, `revoked_exits`), `_row_entry_xy` separate from `_entry_xy`, optional `exit_centerline_result`. |
| `src/agbot_vision_nav/timing_stats.py` | Rolling pipeline metrics; `inf` = predict() wall time, `dropped %` = skipped frames (by design). |
| `src/agbot_vision_nav/debug_viz.py` | HUD overlay: state, per-row `w=`, `timing_line`, `detector_line` (renders whatever string the node builds — no change needed for new fields). |
| `scripts/vision_nav_node.py` | Only rospy file. Frame slots + stamps, camera source by FSM state, watchdog, timing/detector/BACKOUT logging; optional second `estimate_centerline` pass for `exit_scan_row_fractions`; detector line at 1 Hz while the meter moves, with `near w=`/`edges=`; `EXIT REVOKED` warnings. |
| `scripts/benchmark_inference.py` | Offline cross-machine inference benchmark (no ROS). |
| `config/params.yaml` + `launch/vision_nav.launch` | All knobs incl. `exit_confirm_distance`, `blocked_confirm_seconds`, `exit_revoke_*`, `exit_flank_min_clear_fraction`, `exit_scan_row_fractions`. params.yaml is the source of truth since 2026-07-30 (§0a). |
| `launch/cameras.launch` | Real-robot camera bringup: WDR front (`/usb_cam`, yuyv) + rear Brio (`/brio_rear`, mjpeg) + a disabled front-Brio slot. Devices pinned by `/dev/v4l/by-id/` serials. The rear node name MUST match `vision_nav.launch`'s `rear_camera_topic` — see §0b. |
| `test/` | **113 tests**: controller 17, centerline 11, viz 3, detector 40, fsm 35, timing 7. |

### `agbot_bringup/`
| File | Purpose |
|---|---|
| `launch/agbot_gazebo.launch` | Gazebo + Jackal + camera URDF override + RViz. |
| `launch/display.launch.xml` | RViz-only URDF viewer. |
| `urdf/agbot_camera.urdf.xacro` | `agbot_cam` macro; front + rear (yaw π → `/camera_rear/image_raw`). **UNCOMMITTED user working-tree diff** — don't touch. |
| `scripts/load_robot_description.sh` | JACKAL_URDF_EXTRAS injection. |
| `config/agbot_maize_small.yaml`, `scripts/*maize*` | Small-world workflow. |

### Not in git
Model weights (`config/exported_best.pt`), `jackal/`, `virtual_maize_field/`,
`tmp/`, `AgBot_MPC.pptx`. Claude memory dir: `project_camera_relocation.md`,
`project_robot_deployment.md`, `project_perf_benchmarks.md`.

---

## 5. NEXT STEPS (priority order)

0. **First real-corn run on the new camera pair** (WDR front low + rear Brio,
   §0b). Drop `cmd_vel_topic:=/vision_nav_check` so the robot actually drives,
   confirm `Model loaded on device: cuda:0`, and `rostopic hz` both cameras and
   `/odometry/filtered` before rolling. The rear Brio has never driven a
   back-out outside sim. While in there, apply the two `cameras.launch` items
   left undone in `0936f3b` (`brio_rear` default → true; give
   `brio_front_device` the OTHER serial).
1. **Re-test the joystick takeover in the field, user driving personally**
   (§0 DEFERRED — it worked on the bench and could not be reproduced, so this
   is now a measurement, not a fix). Watch `rostopic hz /bluetooth_teleop/joy`
   for the whole run, hold **L1** while the node is driving, and record whether
   it responds while held / resumes on release / ever loses the heartbeat. No
   code is deployed for this — the attempted fix was reverted so the run goes
   out on the sim-validated configuration.
2. ✅ **DONE 2026-07-29 — no false exits on the previously-failing route.**
   (Original text kept for the tuning guidance.) **Field-validate the exit
   path** on the GPU robot (sim is now green — see
   §0 SIM-VALIDATED — so this is the next real unknown):
   - mid-row gap → HUD `openrows=0` even when a `w=` reads ≥0.8, and the
     `open x/0.40 m` bar drains instead of filling;
   - true row end → the bar fills smoothly and fires, and the turn happens
     `headland_clearance` after FIRST sighting (not 1.15 m later);
   - if a false exit still slips through, it should now log
     `EXIT REVOKED: ...` and return to FOLLOW_ROW rather than commit.
   Tune from the HUD: `exit_confirm_distance` up if gaps still confirm,
   `exit_flank_min_clear_fraction` down (or `exit_flank_edge_margin` up) if a
   real open field fails to fire.
3. **Calibrate scan rows per camera mount** (lab, ~10 min each): tape at
   1/2/3 m ahead, one frame per mount, read off the pixel rows, convert to
   fractions. Put the exit rows in `exit_scan_row_fractions` (steering rows
   stay put). The current `0.65/0.78/0.92` were heuristic and were never
   re-derived for the low mount — on the low camera the bottom row images
   ground so close that it is wide (~0.7) BOTH in-row and at an exit, which is
   what made it a weak discriminator in the first place.
4. ✅ **DONE 2026-07-30 — back-out runs end to end** (after the
   `blocked_min_obstacle_fraction` fix, §0a). **Full back-out mission
   end-to-end in sim** (confirms the 2026-07-22
   blocked-count fix): blocked row 1 → back out → S-turn → reacquire → finish
   3 rows → `Mission DONE: rows_driven=3, blocked rows: row 1 blocked at X m`.
   Also the no-rear case and a BACKOUT nudge test.
5. **Field re-test** of the `traverse_distance` fix (decide 0.6 vs 0.65 — see
   §6 precedence gotcha) and exits on the CPU robot (`mpc_dt:=0.5`).
6. **GPU robot bring-up finish**: confirm `Model loaded on device: cuda`, run
   `benchmark_inference.py` on laptop / CPU robot / GPU robot for the FPS table.
   Consider collecting **more low-camera annotations** to strengthen
   segmentation on the low-mounted rigs.
7. **GPS RTK plan** (§0 DEFERRED) if/when trailer autonomy is prioritized.
8. Then: mission robustness matrix (`first_turn_direction:=right`,
   `num_rows:=0`, full world), speed tuning on the GPU robot, go-home.

---

## 6. GOTCHAS

1. **Open exit needs BOTH wide AND flank-clear.** If exits stop firing in real
   open field, the outer strips aren't reading clear — lower
   `exit_flank_min_clear_fraction` or raise `exit_flank_edge_margin`, and check
   segmentation. Don't revert. `exit_flank_edge_margin >= 1.0` fully restores
   width-only behavior.
1b. **Debounce knobs are meters (open) and seconds (blocked), not frames.**
   `exit_detect_frames` / `blocked_detect_frames` no longer exist anywhere —
   passing them raises TypeError rather than being silently ignored, which is
   intentional.
1c. **`reacquire_max_width` / `reacquire_frames` no longer exist.** REACQUIRE
   asks whether corn flanks the near scan row and confirms over meters. Do not
   reintroduce a width bar there — it is unlatchable on the low camera.
1d. **A revoked exit must NOT reset the row-entry pose.** `_row_entry_xy` is
   deliberately separate from `_entry_xy`; calling `_enter(STATE_FOLLOW_ROW)`
   on a revert would disarm the detector for another 2 m inside a row it never
   left. Tripwire: the re-arm assertion in
   `test_exit_revoked_when_near_row_stays_corn_flanked`.
2. **Never key the exit on far scan rows** — the hard-won one. Don't restore
   commit `54e8ef8`'s detector logic. (The flank gate is per-row, not far-row.)
3. ~~**Launch-arg defaults override params.yaml**~~ **FIXED 2026-07-30 —
   this is now the OPPOSITE.** `config/params.yaml` is the source of truth;
   launch `<arg>`s default to empty with conditional `<param>` tags, so a launch
   arg wins only when actually passed. Edit the yaml and it takes effect.
   Exceptions: `model_path` and the four camera-topic args computed from `sim`.
   Confirm any knob with `roslaunch --dump-params ...` or the node's startup
   config log. See §0a.
4. **`agbot_camera.urdf.xacro` working-tree diff is the user's** — ask before
   committing/reverting. Physical robot camera height is separate from this sim
   URDF.
5. **Exit detector arms on odometry distance**: no `/odometry/filtered` ⇒ never
   arms ⇒ never leaves FOLLOW_ROW. Blocked arms at 0.3 m, open at 2.0 m.
6. **Blocked rows do NOT count toward num_rows** (2026-07-22).
7. **Joystick takeover**: keep the node on `/cmd_vel`; don't override
   `cmd_vel_topic` onto the controller/teleop topic (bypasses twist_mux
   priority). The mux itself is fine — bench-verified on the GPU robot
   2026-07-28. The field failure is UNEXPLAINED; see §0 DEFERRED before
   theorising.
8. **This dev sandbox is ROS2 Humble, not ROS1** — catkin/roslaunch/rostopic
   run on the user's WSL2 ROS1 Noetic machine (same filesystem). Unit tests DO
   run in the sandbox. Model runs in `~/agbot_venv`.
9b. **The rear camera fails SILENTLY.** Rear frames are consumed only in
   STATE_BACKOUT, so a wrong topic name, a `brio_rear:=true` you forgot, or an
   unplugged Brio costs nothing until the robot is stopped in front of an
   obstacle and cannot reverse. `rostopic hz` the rear topic BEFORE every
   mission run. The node name in `cameras.launch` and `rear_camera_topic` in
   `vision_nav.launch` must agree (`/brio_rear/image_raw/compressed`). See §0b.
9c. **`rosbag record` does not warn on a nonexistent topic** — it subscribes and
   waits, leaving a valid ~4 KB empty bag. Real-robot front camera is
   `/usb_cam/image_raw/compressed`; `/camera/image_raw` is Gazebo only.
9. scipy needed at import. GAZEBO_MODEL_PATH needs virtual_maize_field/models.
   `gh` at `~/.local/bin/gh`. **Never `git add .`** (weights/tmp/pptx stay out).
   High dropped-frame % in the timing log is BY DESIGN.

---

## Quick-start (ROS1 Noetic machine)

```bash
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# Simulation world
roslaunch agbot_bringup agbot_gazebo.launch

# REAL ROBOT (cpr-j100-0864): cameras first, then the node
roslaunch agbot_vision_nav cameras.launch brio_rear:=true
rostopic hz /usb_cam/image_raw/compressed /brio_rear/image_raw/compressed
rostopic hz /odometry/filtered
roslaunch agbot_vision_nav vision_nav.launch \
  mission_enabled:=true num_rows:=3 rear_camera_enabled:=true
#   add cmd_vel_topic:=/vision_nav_check for a DRY RUN (robot will not move)

# Mission with back-out
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  mission_enabled:=true rear_camera_enabled:=true num_rows:=3

# Tune if needed:
#   exit_confirm_distance:=0.4          m of travel to confirm an exit
#   exit_leak_ratio:=0.5                meter drain rate vs fill (1.0 = old)
#   reacquire_confirm_distance:=0.12    m of in-row view to latch a new row
#   exit_flank_min_clear_fraction:=0.8  lower if real exits fail to fire
#   exit_revoke_fail_distance:=0.25     m of near-row corn that withdraws an exit
#   exit_scan_row_fractions:="[0.55, 0.70, 0.82]"   exit rows only (per mount)

# Rear-camera segmentation preview (robot idle, cmd_vel diverted)
roslaunch agbot_vision_nav vision_nav.launch sim:=true \
  camera_topic:=/camera_rear/image_raw camera_topic_is_compressed:=false \
  cmd_vel_topic:=/cmd_vel_rear_preview

# Monitor
rqt_image_view /vision_nav_node/debug/image
# HUD detector line:
#   'exit: blk 0.0/4.0 s open 0.12/0.40 m rows= wide= openrows=
#    near w=0.87 edges=0.72/0.95 frac= armed o: b:'
#   near w=/edges= say WHICH threshold is blocking an exit.
#   openrows=0 with wide>=1 at a mid-row gap == the flank gate working;
#   the 'open x/0.40 m' bar draining instead of filling == the gap being rejected.
rostopic echo /cmd_vel
# Console: 'timing:' every 5 s, the detector line at 1 Hz while the open meter
#          or blocked timer is moving (5 s otherwise), BACKOUT telemetry,
#          'EXIT REVOKED: row N at X m' (unthrottled) when a false exit is
#          withdrawn, one-shot 'Mission DONE: rows_driven=N, blocked rows: ...,
#          revoked exits: N'

# Offline inference benchmark (per machine, in the venv)
source ~/agbot_venv/bin/activate
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 scripts/benchmark_inference.py \
  --model config/exported_best.pt --image /path/to/frame.jpg

# Unit tests (no ROS needed)
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v     # expected: 106 passed
```
