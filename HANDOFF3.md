# HANDOFF3.md

Handoff for the P-AgBot vision-nav work, updated end of session 2026-07-28.
Field status: in-row nav + headland turns WORK on the real robot. Session
2026-07-24 brought up the new NVIDIA-GPU robot and found a mid-row
false-EXIT_CLEAR bug (low camera + fast inference). Session 2026-07-28 found
the DEEPER cause — a frame-counted debounce — rebuilt the exit path around
it, then sim-tested that rebuild and fixed the two failures it exposed (a
symmetric accumulator leak that can never fire on a marginal signal, and a
REACQUIRE latch keyed on a camera-height-specific width). Written so a fresh
session picks up with zero context. Supersedes all previous HANDOFF3 content.

---

## 0. START HERE — this session (2026-07-28)

### DONE + committed: sim run of `a9c6ed3` failed twice; both fixed (`3fd99ad`)

**Read the log correctly.** The detector line was throttled to one print per
5 s, so at 2 Hz only 1 frame in 10 was visible. The first reading of the run
("the corridor never widened, `wide=0`") was WRONG. The giveaway is in the log
itself: `open 0.13/0.40 m`. That meter only grows on a frame where a row is
wide AND flank-clear, so the open signature *was* firing, in frames that were
never printed. It reached 0.13 m and drained back to 0.00.

**Failure 1 — the meter leaked as fast as it filled.** Symmetric leak sounds
neutral. It is not: a signature true HALF the frames nets exactly zero and can
**never** fire, however far the robot drives. At the sim row end the model
*does* label the ground traversable — it is just imperfect, width ~0.8–0.9
with patchy edges — so the signature flickers, and the robot drove past the
row end toward the world edge. Fixed with `exit_leak_ratio` (0.5): drains at
half the fill rate, so anything true more than ~1/3 of the time still climbs,
while a short mid-row gap still drains away. `exit_leak_ratio: 1.0` restores
the old behavior, and `test_symmetric_leak_reproduces_the_failure` pins that
the old behavior *is* the bug.
  - Sub-bug this exposed: the first frame of a streak credited no distance (so
    `open_distance` would equal "meters since first sighting" exactly). With a
    flickering signature the accumulator returns to zero between bursts, making
    **every** open frame a "first" frame that banks nothing — a hard deadlock.
    The streak now starts at the previous sample and every frame credits.
  - ⚠ **Cost of a marginal signal:** net fill rate is `1.5*duty - 0.5`. At 50%
    duty that is 0.25, so 0.4 m of evidence needs ~1.6 m of driving (measured:
    fires at 1.45 m). At 70% duty, ~0.7 m. **Raise the signal quality rather
    than loosening the leak further** — that is what the new HUD numbers are
    for. The `exit_clear_min_distance` clamp keeps the overshoot from
    compounding: back-dating has already consumed `headland_clearance`, so
    EXIT_CLEAR ends after 0.2 m instead of another 0.75 m.

**Failure 2 — REACQUIRE could not latch, crept 25 s blind, nearly hit corn.**
It latched on `mean corridor width < reacquire_max_width (0.6)` — a
camera-height constant (normal in-row width ~0.5 tall, ~0.7 low), so on the
low camera it could never be satisfied *inside a row*. It also required
`traversable_fraction >= 0.10` while the sim headland measured 0.09 — failing
on both counts. The FSM then crept the full `reacquire_max_distance` (2.0 m at
0.08 m/s = **25 s**) with `angular_z` hard-coded to 0.0, holding whatever
lateral error the turn left behind. Now it latches on **the near scan row
having corn on BOTH sides** (inverse of the open-exit test, reusing
`nearest_row_flank_clear`), confirms over `reacquire_confirm_distance`
**meters**, and **steers while creeping**. `reacquire_max_width` and
`reacquire_frames` are removed.

**Diagnostics** (this run cost a session to interpret): the detector line
prints at **1 Hz whenever the meter is moving** and now carries `near w=` and
`edges=`. Without those, "width under the bar" and "edges under the bar" both
render as `openrows=0` with no way to tell which knob to turn.

**106 tests pass** (93 → 106).

**Next:** re-run the sim. If the exit still will not fire, read `near w=` /
`edges=` and lower `exit_flank_min_clear_fraction` (edges under 0.8) or
`exit_width_threshold` (width under 0.8) — do NOT reach for `exit_leak_ratio`
first.

### User decisions this session (do not re-propose)
- **No odometry row-length fallback**, and **no re-reading "lost view past the
  row end" as an exit.** Consequence, recorded once: a genuine segmentation
  failure over open ground still has nothing catching it, and a blocked signal
  with no rear camera still ends the mission where it stands.

### Earlier the same session: exit detection rebuilt (commit `a9c6ed3`)
The 2026-07-24 flank fix treated a symptom. The user pushed back with the
right question — *"doesn't requiring corn beside the corridor just mean the
exit needs width 1.0? Where is the actual threshold?"* — and that was right:
`x_left <= 0.05W AND x_right >= 0.95W` implies `width >= 0.90`, so the flank
rule was a width bar in disguise. Its ONLY addition over a plain width bar is
rejecting **one-sided** openings (corridor pinned to the left edge with corn
still on the right), which happens to be exactly what a gap in a single row of
corn looks like — so it was worth keeping, but it was not the fix.

The real defects, all now fixed:

1. **The debounce was counted in FRAMES.** `exit_detect_frames=5` = 2.5 s at
   2 Hz but 0.2 s at ~25 Hz. Same constant, 12x different meaning; the
   confirmation that was field-proven on the CPU robot effectively did not
   exist on the GPU robot. **OPEN now accumulates METERS TRAVELLED**
   (`exit_confirm_distance` 0.4 m) and **BLOCKED accumulates SECONDS**
   (`blocked_confirm_seconds` 4.0 s). The unit split is deliberate and load
   bearing: a blocked view stops the robot (MPC goes invalid), so a
   distance-based blocked counter would never fill and the back-out would
   deadlock. Both accumulators are **leaky, not strictly consecutive** — 0.4 m
   at 25 Hz is ~65 frames and one flickery frame resetting the streak would
   turn the debounce into a never-fires bug. Defaults reproduce the
   field-proven CPU-robot timings at any rate (5 frames @ 2 Hz @ 0.15 m/s =
   0.375 m; 8 frames @ 2 Hz = 4 s).
2. **EXIT_CLEAR was a one-way commit**, so a false positive was a collision
   rather than a wobble. It is now **revocable** for its first
   `exit_revoke_distance` (0.5 m): if the **NEAREST scan row** stays
   corn-flanked for `exit_revoke_fail_distance` (0.25 m), the FSM falls back
   to FOLLOW_ROW and un-counts the row. **Only the near row is consulted** —
   the user correctly objected that during a genuine exit the FAR rows see the
   corn block across the headland, so "corn reappears beside the corridor"
   would revoke every real exit. Tripwire:
   `test_far_row_corn_across_headland_does_not_revoke`.
3. **The flank test was vetoed by one stray pixel**, because the corridor scan
   stops at the FIRST non-traversable column — a false-negative risk on the
   thinly-trained low-camera masks. It now measures outer-strip **occupancy**
   (`exit_flank_min_clear_fraction` 0.8), computed in `estimate_centerline`
   and carried on `ScanRowResult` as defaulted fields (hand-built results
   still take the old edge-reach path).

Also landed:
- **`headland_clearance` is back-dated to where the exit was FIRST seen**, so
  the confirmation distance is not added on top. Without this the robot would
  overrun by 0.4 + 0.75 = 1.15 m. `exit_clear_min_distance` (0.2 m) is always
  driven regardless.
- **`exit_scan_row_fractions`** (empty = share the steering rows): the exit
  detector can use its own scan rows without touching field-proven steering.
  Rear stays on the steering rows — the FSM uses one rear result both to watch
  for the exit behind and to steer the reverse leg.
- HUD/log now read `open 0.12/0.40 m`, `blk 1.5/4.0 s`, `nearflank=Y/N/-`;
  a revocation logs `EXIT REVOKED: row N at X m` (unthrottled) and the DONE
  line reports the revoked count.

**93 tests pass** (78 → 93). All prior tripwires stayed green.

**Next:** everything below is desk-work-complete and wants the lab/field.
See §5.

### Deliberately NOT done (decided this session)
- **Adaptive/relative width threshold.** Would learn the median in-row width
  over the last few meters and fire at ~1.35x baseline, auto-calibrating
  across camera mounts. Unnecessary while the flank rule already pins the bar
  near 1.0, and it adds state that can fail silently. Revisit only if
  per-mount tuning proves painful.
- **Offline mask-vs-prediction harness.** The user judged the reported widths
  accurate (the low camera genuinely reads ~1.0 at a real row end and ~0.83 at
  a gap), so this is detector logic, not segmentation quality.

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

### DEFERRED (user scoped out this session) — two open items
- **SAFETY: joystick takeover on the fast robot.** The grad student could
  NOT override cmd_vel when the robot hit corn; killing the node was the only
  recourse. Rate-dependent (fine on the 2 Hz robot, broken on the fast GPU
  robot). BUT: the node publishes to `/cmd_vel` = twist_mux `external` input
  (priority 1) and the joystick is `bluetooth_teleop/cmd_vel` (priority 9), so
  twist_mux SHOULD override regardless of publish rate. A rate-dependent
  failure is the fingerprint of the mux being bypassed — verify on the robot:
  (a) is the deployed node really on `/cmd_vel` or was `cmd_vel_topic`
  overridden onto the controller/teleop topic; (b) is `twist_mux` from
  `jackal_control/launch/control.launch` actually running and the sole
  publisher to `jackal_velocity_controller/cmd_vel`; (c) nothing remapping
  `/cmd_vel` onto the controller. Recommended: fix the wiring AND add an
  in-node operator-override subscriber (yield when joystick input present).
  **Address soon — safety-relevant.** Full analysis in the plan file
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
SEPARATE issue), try `reacquire_max_width` 0.6→0.7 or `reacquire_max_distance`
2.0→2.5.

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
- End-of-TRAVERSE nose-too-close fix: `traverse_distance` param (below), NOT
  yet field-re-tested.
- `agbot_camera.urdf.xacro` still carries an UNCOMMITTED working-tree diff
  (tall line active, front-deck line commented, stand at x=-0.025). It is the
  user's state: don't commit or revert unprompted. (The physical GPU robot's
  low camera is a separate real-robot mount, independent of this sim URDF.)

### Sim back-out validation (2026-07-21, four iterations — all committed)
1. Blocked never fired at a big box: ground-fraction gate too strict up close.
   `blocked_min_traversable_fraction` 0.15 → 0.08 → **0.02** (HUD showed
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
  `blocked_min_traversable_fraction` (0.02), accumulated over
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
  frame (frac=0.04); the 8-frame debounce is the real occlusion guard.
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
  priority 1); the joystick at priority 9 is meant to override it. Do NOT
  point `cmd_vel_topic` at the controller/teleop topic (that bypasses the mux
  and is the likely cause of the takeover failure — see §0 DEFERRED).

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
| `scripts/vision_nav_node.py` | Only rospy file. Frame slots + stamps, camera source by FSM state, watchdog, timing/detector/BACKOUT logging; HUD detector line now shows `wide=`/`openrows=`. |
| `scripts/benchmark_inference.py` | Offline cross-machine inference benchmark (no ROS). |
| `config/params.yaml` + `launch/vision_nav.launch` | All knobs incl. `exit_confirm_distance`, `blocked_confirm_seconds`, `exit_revoke_*`, `exit_flank_min_clear_fraction`, `exit_scan_row_fractions`. ⚠ launch-arg defaults override params.yaml (§6). |
| `test/` | **106 tests**: controller 17, centerline 9, viz 3, detector 35, fsm 35, timing 7. |

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

1. **SAFETY: fix joystick takeover** (§0 DEFERRED). Verify the mux wiring on
   the robot and/or add an in-node operator override. **Do this before any
   further autonomous field run** — right now there is no reliable human
   override on the fast robot. Ten-minute bench test, robot on blocks:
   `rostopic info jackal_velocity_controller/cmd_vel` with the node running
   must show exactly ONE publisher (twist_mux). If the node or the teleop node
   publishes there directly, that is the whole bug — a rate-dependent override
   failure is the fingerprint of the mux being bypassed, since twist_mux
   arbitrates by priority, not publish rate. Then also cap the node's publish
   rate (~20 Hz) so inference speed stops leaking into control behavior.
2. **Re-run the sim mission** (`3fd99ad`): the exit meter should now climb
   through the patchy row-end mask instead of stalling near 0.13 and draining,
   and REACQUIRE should latch within ~0.2 m of entering the new row instead of
   creeping for ~25 s (verifiable at the low-camera turn 1→2, which already
   worked). If the exit still will not fire, read `near w=` / `edges=` and
   lower `exit_flank_min_clear_fraction` or `exit_width_threshold` — not
   `exit_leak_ratio`.
3. **Field-validate the exit path** on the GPU robot:
   - mid-row gap → HUD `openrows=0` even when a `w=` reads ≥0.8, and the
     `open x/0.40 m` bar drains instead of filling;
   - true row end → the bar fills smoothly and fires, and the turn happens
     `headland_clearance` after FIRST sighting (not 1.15 m later);
   - if a false exit still slips through, it should now log
     `EXIT REVOKED: ...` and return to FOLLOW_ROW rather than commit.
   Tune from the HUD: `exit_confirm_distance` up if gaps still confirm,
   `exit_flank_min_clear_fraction` down (or `exit_flank_edge_margin` up) if a
   real open field fails to fire.
4. **Calibrate scan rows per camera mount** (lab, ~10 min each): tape at
   1/2/3 m ahead, one frame per mount, read off the pixel rows, convert to
   fractions. Put the exit rows in `exit_scan_row_fractions` (steering rows
   stay put). The current `0.65/0.78/0.92` were heuristic and were never
   re-derived for the low mount — on the low camera the bottom row images
   ground so close that it is wide (~0.7) BOTH in-row and at an exit, which is
   what made it a weak discriminator in the first place.
3. **Full back-out mission end-to-end in sim** (confirms the 2026-07-22
   blocked-count fix): blocked row 1 → back out → S-turn → reacquire → finish
   3 rows → `Mission DONE: rows_driven=3, blocked rows: row 1 blocked at X m`.
   Also the no-rear case and a BACKOUT nudge test.
4. **Field re-test** of the `traverse_distance` fix (decide 0.6 vs 0.65 — see
   §6 precedence gotcha) and exits on the CPU robot (`mpc_dt:=0.5`).
5. **GPU robot bring-up finish**: confirm `Model loaded on device: cuda`, run
   `benchmark_inference.py` on laptop / CPU robot / GPU robot for the FPS table.
   Consider collecting **more low-camera annotations** to strengthen
   segmentation on the low-mounted rigs.
6. **GPS RTK plan** (§0 DEFERRED) if/when trailer autonomy is prioritized.
7. Then: mission robustness matrix (`first_turn_direction:=right`,
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
1d. **`reacquire_max_width` / `reacquire_frames` no longer exist.** REACQUIRE
   asks whether corn flanks the near scan row and confirms over meters. Do not
   reintroduce a width bar there — it is unlatchable on the low camera.
1c. **A revoked exit must NOT reset the row-entry pose.** `_row_entry_xy` is
   deliberately separate from `_entry_xy`; calling `_enter(STATE_FOLLOW_ROW)`
   on a revert would disarm the detector for another 2 m inside a row it never
   left. Tripwire: the re-arm assertion in
   `test_exit_revoked_when_near_row_stays_corn_flanked`.
2. **Never key the exit on far scan rows** — the hard-won one. Don't restore
   commit `54e8ef8`'s detector logic. (The flank gate is per-row, not far-row.)
3. **Launch-arg defaults override params.yaml** for every duplicated knob
   (`<param>` after `<rosparam file>` wins). Applies to `exit_flank_edge_margin`
   (0.05 in both), `traverse_distance` (launch 0.6 wins over the user's
   params.yaml 0.65 — pass `traverse_distance:=0.65` to actually get it), etc.
4. **`agbot_camera.urdf.xacro` working-tree diff is the user's** — ask before
   committing/reverting. Physical robot camera height is separate from this sim
   URDF.
5. **Exit detector arms on odometry distance**: no `/odometry/filtered` ⇒ never
   arms ⇒ never leaves FOLLOW_ROW. Blocked arms at 0.3 m, open at 2.0 m.
6. **Blocked rows do NOT count toward num_rows** (2026-07-22).
7. **Joystick takeover**: keep the node on `/cmd_vel`; don't override
   `cmd_vel_topic` onto the controller/teleop topic (bypasses twist_mux
   priority — the suspected cause of the takeover failure).
8. **This dev sandbox is ROS2 Humble, not ROS1** — catkin/roslaunch/rostopic
   run on the user's WSL2 ROS1 Noetic machine (same filesystem). Unit tests DO
   run in the sandbox. Model runs in `~/agbot_venv`.
9. scipy needed at import. GAZEBO_MODEL_PATH needs virtual_maize_field/models.
   `gh` at `~/.local/bin/gh`. **Never `git add .`** (weights/tmp/pptx stay out).
   High dropped-frame % in the timing log is BY DESIGN.

---

## Quick-start (ROS1 Noetic machine)

```bash
cd ~/agbot_control_ws && catkin build && source devel/setup.bash

# Simulation world
roslaunch agbot_bringup agbot_gazebo.launch

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
# Console: 'timing:' every 5 s, 'exit: blk n/8' countdown, BACKOUT telemetry,
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
