# AIAgNav — Supplementary Material

Companion to *AIAgNav: A Semantic Segmentation-Based Autonomous Navigation
System for Cornfields* and to `AIAgNav_Technical.md`.

The conference paper carries the design; this document carries what a reader
who wants to reproduce or extend the system needs and a reviewer may ask for:
the sign conventions, the tuned parameter values, the derivation behind the
rear-steered headland leg, the runtime architecture, and the design history
that produced each of them. Section numbers in the form §III-C refer to
`AIAgNav_Technical.md`.

| Section | Contents |
|---|---|
| S1 | Sign convention and controller reference-frame resets (§III-C.7) |
| S2 | Controller tuned values and tuning order (§III-C.9) |
| S3 | The rear-steered headland exit leg and the rear-to-front state conversion (§III-E.4) |
| S4 | Runtime architecture and real-time behaviour (§III-G) |
| S5 | Parameter reference (Appendix B) |
| S6 | Design history (Appendix A) |

---

## S1. Sign Convention and Reference-Frame Resets

*Moved from §III-C.7.*

Sign errors are the most persistent class of bug in this system, so the convention is
stated explicitly and is pinned by unit tests rather than by inspection. Image $x$
increases rightward; positive $\omega_z$ is a left turn under REP-103.

| Observation | Meaning | Correct response |
|---|---|---|
| $e_d < 0$ | Lane appears left of image centre; robot is too far **right** | $\omega_z > 0$ — turn **left** |
| $e_d > 0$ | Lane appears right of image centre; robot is too far **left** | $\omega_z < 0$ — turn **right** |
| $s > 0$ | Lane tilts rightward with depth | $\omega_z < 0$ — corrective **right** |

Note that these signs are **not hard-coded anywhere**. They emerge from the optimization:
with $\alpha, \beta > 0$, reducing a positive $e_d$ requires driving $s$ negative, which
requires $u < 0$. This is a property worth stating in the paper, because it means the sign
convention is a consequence of the model rather than a convention that could drift out of
agreement with it.

The controller's internal state — $u_{-1}$ and the invalid-frame counter — is reset
(`reset()`) whenever the *steering reference frame* changes: on entering row following, on
entering the reverse manoeuvre, and on entering the rear-steered headland leg. Without
this, the rate limiter of Eq. (19) would carry a command issued under one sign convention
into a state that uses another.

---

## S2. Controller Tuned Values and Tuning Order

*Moved from §III-C.9.*

| Symbol | Parameter | Value | Source of the value |
|---|---|---|---|
| $N$ | `mpc_horizon` | 8 | Sub-millisecond solve; longer adds no observed benefit |
| $\Delta t$ | `mpc_dt` | 0.1 s (GPU) / 0.5 s (CPU) | Must equal the true control period |
| $\alpha$ | `mpc_alpha` | 0.10 | Empirical; scales with cruise speed |
| $\beta$ | `mpc_beta` | 0.10 | Empirical |
| $q_d$ | `mpc_q_offset` | 10.0 | Primary tuning knob |
| $q_\psi$ | `mpc_q_heading` | 1.0 | Raise if the robot drifts wide on bends |
| $r$ | `mpc_r_control` | 0.1 | Kept small; authority is needed in a narrow row |
| $r_\Delta$ | `mpc_r_delta` | 0.5 | Raise to suppress growing oscillation |
| $v_{\text{cruise}}$ | `linear_x_cruise` | 0.15 m/s | Sim- and field-validated envelope |
| $\omega_{\max}$ | `angular_z_max` | 0.175 rad/s | 0.3 wobbled into corn (field, 2026-07-15) |
| $\Delta\omega_{\max}$ | `delta_angular_z_max` | 0.2 rad/s per 0.1 s | Slew limit |
| $n_{\text{invalid}}$ | `invalid_frame_stop_count` | 5 | Consecutive unusable frames before stop |

The documented tuning order is: **verify the sign convention first**, then $\alpha,\beta$,
then $q_d$, then $r_\Delta$, then $q_\psi$, then $r$. Diagnostic rules recorded from field
sessions: a square-wave $\omega_z$ riding the clamp means the robot is too fast for the
loop rate; a smooth, growing oscillation means raise $r_\Delta$ or lower $q_d$; drifting
wide on bends means raise $q_\psi$.

---

## S3. The Rear-Steered Headland Exit Leg

*Moved from §III-E.4. This is the derivation the paper's Section III-E defers to.*

**Motivation.** With `EXIT_CLEAR` driving straight and blind, the 2026-08-05 field run
clipped the end-of-row corn and left the robot misaligned, which in turn put its nose into
a plant at the following `TURN_2`. The forward camera cannot help here: once the robot is
past the row end it is looking at open headland, which contains no row axis to steer on.
The **rear** camera, however, is looking back down the row the robot is leaving — the best
available reference for the row axis at exactly that moment.

The leg therefore: switches inference to the rear camera for its whole duration (the
operator debug view shows the rear image, labelled), steers from the rear view, and turns
only when the rear view *also* reads open field — meaning the robot's tail has cleared the
last plants.

**The sign problem, and why no single sign flip solves it.** Steering from a
180°-rotated camera is not a matter of negating the error. Model the row axis in both
views. With $e$ the lateral offset (positive = robot left of the axis) and $\theta$ the
heading error (positive = yawed left):

$$
\begin{aligned}
\text{front:} \quad & e_d^{\,\text{f}} = +c_1 e + c_2 \theta, \qquad s^{\,\text{f}} = -c_3 e\\[4pt]
\text{rear:} \quad & e_d^{\,\text{r}} = -c_1 e + c_2 \theta, \qquad s^{\,\text{r}} = +c_3 e
\end{aligned}
\tag{35}
$$

**The mirror flips the lateral term but not the heading term.** A yaw to the left moves the
vanishing point to image-*right* in **both** views, because both cameras are rigidly
attached to the robot and rotate with it. Consequently no single sign serves the rear
view: negating stabilises $e$ and **inverts the heading feedback** — positive feedback on
$\theta$, which is a slow, steady walk off the row axis; not negating does the reverse.
A rule of the form "negate if and only if exactly one of {mirrored view, reversed motion}
applies" was implemented, unit-tested, and documented before being disproved by
Eq. (35) in simulation on 2026-08-07.

**The fix is a reconstruction, not a sign.** Inverting Eq. (35) — which is possible for
exactly the reason given in §III-B.4, that the map is invertible — gives the front-equivalent
state directly:

$$
\boxed{\;
e_d^{\,\text{f}} \;=\; e_d^{\,\text{r}} + \kappa\, s^{\,\text{r}},
\qquad
s^{\,\text{f}} \;=\; -\,s^{\,\text{r}}
\;}
\tag{36}
$$

$$
\kappa \;=\; \frac{2c_1}{c_3}
\;=\; \frac{2 \sum_i w_i/d_i}{\;1/d_{\text{near}} - 1/d_{\text{far}}\;}
\tag{37}
$$

With the deployed scan rows $f = (0.65, 0.78, 0.92)$, weights $w = (0.2, 0.3, 0.5)$, and
imaged ground distances of approximately 3 m, 2 m and 1 m:

$$
c_1 = \tfrac{0.2}{3} + \tfrac{0.3}{2} + \tfrac{0.5}{1} = 0.717,
\qquad
c_3 = \tfrac{1}{1} - \tfrac{1}{3} = 0.667,
\qquad
\kappa = \frac{2(0.717)}{0.667} \approx 2.15
$$

The deployed value is $\kappa = 2.0$ (`exit_clear_rear_offset_gain`).

> **In plain terms:** the rear camera sees a mirror image, so the sideways error reads
> backwards while the pointing error reads the same way round. You cannot fix both with one
> minus sign. Instead, Eq. (36) reconstructs what the *front* camera would have reported if
> it could see the row — and hands that to the unmodified controller.

**$\kappa$ is a geometric constant, not a per-robot calibration.** Equation (37) shows it
depends only on the *ratios* of the scan rows' ground distances, and those ratios are
identical in both cameras because the rear mount is an exact geometric mirror (§II-A). It
does not need to be re-measured for a new robot. Setting $\kappa = 0$ drops the correcting
term and reproduces the original broken behaviour, so it should be raised, never lowered.

**Steering is gated on a bounded corridor.** The leg steers only when the rear result is
valid **and** the nearest rear scan row satisfies Eq. (11). In a headland, a corridor
running off the edge of the frame is not an anomaly — it is the normal case, and Eq. (5)
returns a fictitious midpoint for it. Steering on that fiction pinned the command at the
maximum steering rate for an entire 15 s leg, roughly 150° of unintended rotation, and
nearly drove the robot out of the simulated world. When the rear corridor is unusable, the
leg simply does not steer that tick — the same policy the forward controller applies to an
unusable forward frame.

**Termination.** Three terminators, in priority order:

| Condition | Terminator |
|---|---|
| Rear view opened | $d \ge \max\big(0.2,\; d_{\text{rear open}} + 0.2\big)$ m → `TURN_1` |
| No rear frame arrived at all during the leg | fall back to the open-loop rule: $d \ge 0.2$ and $d + \delta_{\text{exit}} \ge$ `headland_clearance` (1.0 m) → `TURN_1` |
| Rear view never opened | ceiling at $d \ge 1.5$ m → `TURN_1` |

Note the third row **turns**; it does not un-count the row and resume following. An earlier
version did exactly that, dropping the machine back into `FOLLOW_ROW` in the middle of a
headland — where the exit detector must then re-arm over 2.0 m of open field before it can
fire again. That was the most direct path to the world edge that the system ever had.

**The rear watcher is a separate detector with one different threshold.** It reuses the
`RowExitDetector` of §III-D with the front thresholds, except that its confirmation
distance is **0.1 m** rather than 0.4 m. The reasoning is the cleanest argument in the
design record and is worth reproducing:

> The two detectors answer different questions. The **front** one decides whether the row
> has ended *at all*, from inside the row, with nothing to corroborate it — a mid-row gap
> looks identical to a row end, and 0.4 m of driving is what buys the confidence to
> separate them. By the time the **rear** one runs, that is settled; all it is asked is
> *has my tail passed the last plants*, a fact about where the robot **is**. Charging the
> front's evidence again is paying twice for one decision.

Before this was corrected the leg always ran about 0.4 m past the row end. The rear watcher
is also deliberately a **different object** from the reverse-manoeuvre watcher of §III-F,
so that a tuning change to the headland leg cannot silently shorten the field-proven
reverse leg. And its open point is back-dated exactly as the front's is (Eq. 31) — the same
double-charging mistake, rediscovered in a new place.

**Revocation cannot be rebuilt on the rear view**, and this is a genuine impossibility
rather than an omission: immediately after a *genuine* exit, the rear near-row still
legitimately has corn on both sides, so a rear-based revocation test would revoke every
real exit. Revocation therefore runs only on the fallback front frames.

**A deliberate design line.** Two mechanisms were proposed for this leg and removed: a
separate steering clamp and a maximum-yaw limit for the leg alone. Both were new control
mechanisms existing nowhere else in the pipeline, invented to bound a *symptom* whose cause
was a bad measurement. The governing principle recorded for this subsystem — and worth
stating in the paper — is that **the rear exit leg is the front mechanism pointed
backwards**: same detector, same thresholds, same controller. Every extra knob is a place
where the two can silently diverge. If the leg ever appears to need a gentler hand than
in-row driving does, that is evidence the measurement is still wrong, not that it needs its
own limits.

---

## S4. Runtime Architecture and Real-Time Behaviour

*Moved from §III-G.*

The ROS node is the only file in the package that imports `rospy`. Its structure is
dictated by one fact: **the segmentation forward pass is between 30 and 1000 times slower
than the camera frame period**, and it varies by a factor of thirty across the fleet.

**Single-slot frame buffer, overwrite rather than queue.** Camera callbacks decode the
incoming frame and overwrite a single slot, incrementing a sequence counter and notifying
a condition variable. Frames that were not processed are discarded. A queue would be
actively harmful here: on the 2 Hz robot a queue would serve the inference thread a frame
that is already several hundred milliseconds stale, and the control loop would be steering
on the past. Dropping frames is the correct behaviour, and the dropped fraction —
$1 - n_{\text{processed}}/n_{\text{received}}$ — is reported in the timing line so that
"dropped 15 %" reads as by-design rather than as a fault.

**A dedicated inference thread.** It waits on the condition variable, selects the front or
rear camera according to the current mission state (re-evaluated on every wakeup, so a
state change switches cameras on the next frame), and runs the pipeline outside the lock.
If the mission wants the rear camera but no fresh rear frame arrives within 0.5 s, it
processes a front frame instead — otherwise a dead or mis-topiced rear camera would hang
the loop forever, which would also make the state machine's own "no rear frames arrived"
fallback unreachable.

**A 10 Hz timer that does two jobs.** It republishes the last command as a keep-alive (the
Jackal base brakes on `cmd_vel` silence), and it acts as a watchdog: if no successful
inference has completed within `max_data_age_sec`, it publishes zero and marks a
`WATCHDOG_ZERO` event exactly once on the rising edge — the timer fires at 10 Hz while
stale, and a level-triggered mark would flood the log.

**Pause is not Ctrl-C.** A `SetBool` service pauses the node. Pausing publishes zero and
**skips the state-machine update**, so the row count, the boustrophedon turn direction, the
detector arming distance and the row-entry pose all survive — all of which live in the node
and would be destroyed by a restart, which would begin again at row 1. Resuming resets the
detectors and the controller, because the BLOCKED accumulator of Eq. (28) counts in ROS
seconds: without the reset, a 30 s pause would deposit the entire confirmation threshold on
the first frame back and fire a recovery that nothing justified.

**Measured pipeline timing.**

| Machine | Device | Inference (mean, p95) | Control rate | End-to-end latency |
|---|---|---|---|---|
| `cpr-j100-0864` (GPU robot) | `cuda:0` | 16 ms, 16 ms | ≈ 24 Hz | 48–63 ms (p95 ≈ 80 ms) |
| `cpr-j100-0463` (CPU robot) | `cpu` | ≈ 500 ms | ≈ 2 Hz | — |
| Development laptop (WSL2) | `cpu` | 165 ms | ≈ 5.7 Hz | ≈ 288 ms |

On the GPU robot the **camera, not the model, is the bottleneck** — inference at 16 ms
could sustain over 60 Hz against a 25 Hz camera. End-to-end latency is measured from the
camera header stamp to the moment the command is published, which is the true control
staleness; inference time alone understates it. The first timing line after startup shows
several hundred milliseconds of one-time CUDA warm-up and should be ignored.

Two parameters are re-profiled per machine: `mpc_dt`, which must equal the true control
period for Eq. (15) to be correct, and `max_data_age_sec`, which must exceed the inference
period or every frame arrives at the deadline and the robot stutters. On the CPU robot the
latter is raised to 1.5–3.0 s, and the cost is stated explicitly: 3.0 s at 0.15 m/s is
45 cm of blind travel.

---

## S5. Parameter Reference

*Moved from Appendix B.*

Every tuned constant, its value, and the reason it holds that value. All parameters live in
`agbot_vision_nav/config/params.yaml` unless noted. Launch arguments default to empty and
their parameter tags are conditional, so a launch argument overrides the file only when one
is explicitly passed.

### S5.1 Segmentation (`segmentation/Train.py`)

| Parameter | Value | Reason |
|---|---|---|
| `model` | `dinov3/vits16-eomt` | DINOv3 ViT-S/16 + EoMT; small variant for real-time inference on a mobile robot |
| `transform_args.image_size` | (224, 224) | Native ViT resolution; faster training |
| `steps` | 2500 | ≈ 500 steps per 20 images |
| `batch_size` | 2 | VRAM-constrained |
| `precision` | `16-mixed` | FP16 mixed precision |
| `ignore_classes` | `[]` | No class excluded from the loss |
| `model_device` | `auto` | CUDA when available; the node logs the resolved device so a silent CPU fallback is visible |

### S5.2 Centerline estimation

| Parameter | Value | Reason |
|---|---|---|
| `scan_row_fractions` | `[0.65, 0.78, 0.92]` | Three depths ≈ 3 m / 2 m / 1 m ahead; drives steering |
| `scan_row_weights` | `[0.2, 0.3, 0.5]` | Near row weighted most — it is what must be acted on soonest |
| `min_traversable_fraction` | 0.10 | Validity floor (Eq. 10) |
| `exit_scan_row_fractions` | `[]` | Empty ⇒ the exit detector shares the steering rows |

### S5.3 MPC

| Parameter | Value | Reason |
|---|---|---|
| `mpc_horizon` | 8 | Sub-millisecond solve; no observed benefit from longer |
| `mpc_dt` | 0.1 (GPU) / 0.5 (CPU) | Must equal the real control period (Eq. 15) |
| `mpc_alpha` | 0.10 | Lateral coupling at the 0.1 s reference period; tuned for 0.15 m/s |
| `mpc_beta` | 0.10 | Control effectiveness at the reference period |
| `mpc_q_offset` | 10.0 | Dominant term; primary tuning knob |
| `mpc_q_heading` | 1.0 | Raise if the robot drifts wide on bends |
| `mpc_r_control` | 0.1 | Small — authority is needed in a narrow row |
| `mpc_r_delta` | 0.5 | Smoothing; raise to suppress growing oscillation |
| `linear_x_cruise` | 0.15 m/s | Sim- and field-validated envelope |
| `angular_z_max` | 0.175 rad/s | 0.3 wobbled into corn (2026-07-15) |
| `delta_angular_z_max` | 0.2 rad/s per 0.1 s | Slew limit |
| `invalid_frame_stop_count` | 5 | Consecutive unusable frames before a full stop |
| `max_data_age_sec` | 0.5 (GPU) / 1.5–3.0 (CPU) | Below the inference period ⇒ constant stop-go stutter |

### S5.4 Row-exit detector

| Parameter | Value | Reason |
|---|---|---|
| `exit_width_threshold` | 0.8 | Normalized corridor width reading as open field — but see §III-D.2(b): the flanks hold the line, not this |
| `exit_flank_edge_margin` | 0.05 | Outer-strip width each side |
| `exit_flank_min_clear_fraction` | 0.8 | Below 1.0 deliberately — one stray pixel must not veto a real exit on the weaker low-mount masks |
| `exit_open_rows_required` | 1 | ANY row may fire it; never a specific row (A.4) |
| `exit_confirm_distance` | 0.4 m | Reproduces the field-proven 5 frames at 2 Hz and 0.15 m/s, on a robot of any rate |
| `exit_detect_min_frames` | 2 | Floor: no single large-delta frame may fire an exit |
| `exit_leak_ratio` | 0.5 | Asymmetric — a symmetric leak can never fire a marginal exit (Eq. 30) |
| `blocked_leak_ratio` | 1.0 | Symmetric; a reverse manoeuvre is a much larger commitment |
| `blocked_confirm_seconds` | 4.0 s | The same validated 8 frames at 2 Hz |
| `min_in_row_distance` | 2.0 m | OPEN arming — the view at row entry is open field by definition |
| `blocked_arming_distance` | 0.3 m | BLOCKED arms early; it cannot false-fire at entry |
| `blocked_min_obstacle_fraction` | 0.2 | Measured on the OBSTACLE class — never on traversable (§III-D.2(c)) |

### S5.5 Mission FSM

| Parameter | Value | Reason |
|---|---|---|
| `num_rows` | 3 | 0 = until no rows remain |
| `first_turn_direction` | `left` | Alternates thereafter (boustrophedon) |
| `row_spacing` | 0.75 m | Field geometry |
| `traverse_distance` | 0.6 m | Deliberately shorter than the row spacing — the full spacing put the nose too close to the next row's corn |
| `headland_clearance` | 1.0 m | Walked 0.5 → 0.75 → 1.0; 0.75 still clipped corn in the field (2026-07-29) |
| `exit_clear_min_distance` | 0.2 m | Always driven after confirmation, regardless of back-dating |
| `exit_clear_speed` | 0.10 m/s | Slower than cruise; this is where overshoot causes contact |
| `exit_clear_rear_steering` | true | Steer the headland leg from the rear camera |
| `exit_clear_rear_offset_gain` ($\kappa$) | 2.0 | Derived 2.15 from Eq. (37); geometric, not a per-robot calibration; 0 reproduces the broken behaviour |
| `exit_clear_rear_confirm_distance` | 0.1 m | The one threshold the rear watcher does not inherit (§III-E.4) |
| `exit_clear_post_rear_distance` | 0.2 m | Driven after the rear view opens |
| `exit_clear_max_distance` | 1.5 m | Ceiling; it *turns* rather than resuming row following mid-headland |
| `exit_revoke_enabled` | true | Makes a false exit a wobble rather than a collision |
| `exit_revoke_distance` | 0.5 m | Window after confirmation during which revocation is possible |
| `exit_revoke_fail_distance` | 0.25 m | Continuous corn beside the near row that triggers reversion |
| `turn_rate` | 0.4 rad/s | 90° turns |
| `yaw_tolerance_deg` | 5.0 | Turn termination band (Eq. 33) |
| `reacquire_speed` | 0.08 m/s | Creep speed |
| `reacquire_confirm_distance` | 0.12 m | Sustained in-row view before latching |
| `reacquire_steering_enabled` | true | Steer while creeping — it used to creep blind |
| `reacquire_max_distance` | 2.0 m | Creep with no row ⇒ the field has run out |
| `rear_camera_enabled` | false by default | The recovery branch is unreachable without it |
| `backout_speed` | 0.10 m/s | Reverse speed |

### S5.6 Instrumentation

| Parameter | Value | Reason |
|---|---|---|
| `metrics_csv_dir` | `~/agbot_logs` | On by default — a field pass does not come round twice |
| `intervention_joy_topic` | `/bluetooth_teleop/joy` | `none` disables intervention counting |
| `intervention_deadman_buttons` | `[4, 5]` | L1/R1 on the stock Jackal pad |
| `intervention_gap_seconds` | 3.0 | One messy rescue scores 1, not 5 |
| `intervention_hold_seconds` | 0.5 | How long one joystick message keeps a frame marked as teleoperated |

---

---

## S6. Design History

*Moved from Appendix A.*

This appendix records how the system reached its present form. It is included because
several of the design decisions in §III only make sense in the light of the failure that
produced them, and because the failures themselves are useful results.

### S6.1 Sensing and the departure from LiDAR (2026-06)

The project began from a stated limitation of the lab's existing systems: *the LiDAR's
sparse point cloud has little inherent scene understanding — it cannot tell sky from
plant.* The segmentation training pipeline had already been built by a previous student;
the task taken up here was that nobody had closed the loop from that model into a working
ROS controller.

The controller was designed from the outset as the **image-space analogue of the lab's
validated LiDAR approach**, which centres the robot by balancing the measured left and
right distances $d_l$ and $d_r$. Three alternatives were considered and rejected at the
design stage:

| Rejected | Reason |
|---|---|
| Vanishing-point / single-frame line-fit heading estimation | ROW-SLAM reports this as the *least* accurate baseline it tested. Per-scanline midpoints were adopted instead. |
| CropFollow-style direct $(\varphi, d)$ regression | Requires a model output this project does not have, and a labelled regression dataset that does not exist. |
| P-AgNav's range-view structure | Requires a 360° LiDAR. |
| EKF fusion with the IMU | Explicitly deferred as a documented upgrade path, not an omission. Never needed: the $r_\Delta$ term in Eq. (16) supplied the smoothing. |

### S6.2 Segmentation dataset and the mount-height experiment

The dataset grew across several revisions to its present 443 annotated frames (80/20
split, ≈ 75 % tall mount / ≈ 25 % low mount, mIoU 0.8717), with deliberate curation of
edge cases — turning into a row, dead-end rows, missing plants, downed corn.

The **mount-height change is the most consequential hardware decision in the project.** The
original tall stand worked for the early field season and then failed as the corn grew: the
elevated camera was repeatedly occluded by leaves. Moving to the deck reduced occlusion and
freed the stand volume for a sampling arm. It also produced a chain of downstream
consequences that took a month to work through:

- nominal near-row corridor width rose from ≈ 0.5 to ≈ 0.7;
- a mid-row gap now read ≈ 0.83, above the 0.8 exit threshold → **the 2026-07-24 field
  failure** (A.4);
- `REACQUIRE`'s width-based latch (< 0.6) became unsatisfiable inside a row → the
  twenty-five-second blind creep (§III-E.5);
- the training set remained ≈ 75 % tall-mount, so low-mount masks are weaker at the image
  sides, which is why the flank threshold is 0.8 rather than 1.0.

A separate camera experiment ran in parallel: a Logitech Brio was tried on the *front*
(2026-07-14) and retired (2026-08-03). The settled configuration is the original 5 MP WDR
camera on the low front mount for navigation, with the Brio moved to the **rear** for the
recovery and headland manoeuvres.

An earlier **hand-built Gazebo corn world was abandoned** because the segmentation model
failed frequently on its visuals while working well on the procedurally generated
`virtual_maize_field` worlds; the full FRE-style maize world was in turn too heavy
(RTF < 0.1 on the development laptop) and was replaced by a four-row, 6 m, flat-terrain
world at RTF ≈ 1.0.

### S6.3 Controller: from P to MPC (2026-06-25 → 2026-06-30)

The first controller was **a plain proportional law, deliberately**, not an interim hack:

$$
\omega_z = -\big(k_p\, e_d + k_s\, s\big), \qquad k_p = 1.0,\; k_s = 0.0
$$

The recorded reasoning is worth quoting because it explains the shape of the whole project:
bang-bang was rejected as visibly oscillatory and as teaching nothing toward an MPC; jumping
straight to MPC was rejected as unwarranted complexity for "drive straight in a straight
row"; and the advisor had sanctioned "a controller that seems fit" for the first pass.
Critically, **$s$ was computed and disabled** ($k_s = 0$) from the very first commit,
specifically so that the data an MPC would need already existed before the MPC did. That
decision is why the upgrade five days later was a controller swap rather than a perception
rewrite.

The MPC was adopted because both the professor and the graduate students asked for one, and
because it matches the approach in all three relevant papers. The design review recorded at
the time identified CropFollow's two-signal state → MPC structure as the right template,
with $[e_d, s]$ as the direct analogue of CropFollow's $(\varphi, d)$.

Tuning history, each entry with its cause:

| Change | Date | Cause |
|---|---|---|
| $v_{\text{cruise}}$ 0.15 → 0.3 → back to 0.15 | 2026-07-06 | Reverted to the demonstrated and validated envelope |
| `mpc_dt` scaling implemented (was a no-op) | 2026-07-15 | At 2 Hz with $\Delta t$ left at 0.1 s, every correction was ≈ 5× too strong and saturated the clamp |
| $\omega_{\max}$ 0.3 → 0.175 | 2026-07-15 | Wobbled and contacted plants on real corn |
| `max_data_age_sec` 0.5 → 1.5–3.0 on CPU | 2026-07-09 | Every frame arrived at the deadline → constant stop-go stutter |
| 10 Hz `cmd_vel` keep-alive added | 2026-07-09 | Publishing only per inference at 2 Hz produced surge–brake–surge |
| $\omega_{\max}$ tried at 0.25 | 2026-08-05 | Changed nothing — the controller was never saturating (see A.6) |

**A known and deliberately unfixed property.** The controller has no disturbance state and
re-reads $\mathbf{x}_0$ raw every frame, so the loop is a static gain: any constant bias
settles at a permanent lateral offset by construction. Integral action was considered and
not added, on a correct argument — it helps against an *actuation* disturbance and does
nothing against a *measurement* bias, where it would merely drive the biased reading to
zero harder, at the position the robot already occupies.

### S6.4 Row-exit detector: four rewrites

**Generation 1 (2026-07-03).** Width-only signature, debounced in **frames** (5 open, 8
blocked).

**Generation 1a (2026-07-13, reverted the same day).** Firing from the *farthest* scan rows.
Reverted within hours: beyond the field edge the far rows can stay invalid forever, the
criterion never fired, and the robot drove off the world edge. Now a permanent regression
test.

**Generation 2 (2026-07-13 → 07-22).** Blocked-row recovery added. Four recorded iterations:
the blocked gate never fired at close range (threshold walked 0.15 → 0.08 → 0.02); the
reverse leg overshot because the odometry reference included the pre-row approach (fixed by
the rear-camera terminator); blocked rows were incorrectly counted toward `num_rows` and
ended missions early.

**Generation 3 (2026-07-24) — the field failure that reshaped the detector.** On the GPU
robot with the low mount, `EXIT_CLEAR` fired **in the middle of a row**, the robot drove
into the corn, and — because the pipeline was fast — it committed before anyone could react.
Four-part root cause: low mount raised nominal width 0.5 → 0.7; missing side plants pushed
it to ≈ 0.83, above the 0.8 bar; the signature was width-only; and fast inference satisfied
the 5-frame debounce almost instantly. The fix was the flank-clearance gate of Eq. (24),
using bounds the detector already had — the segmentation model, the centerline estimator and
its result type were untouched.

**Generation 4 (2026-07-28) — the rebuild in physical units.** Eight changes, each with a
regression test: debounce in metres and seconds (Eqs. 27–29); asymmetric OPEN leak (Eq. 30);
revocable `EXIT_CLEAR` (Eq. 34); back-dated clearance (Eq. 31); flank test changed from
edge-reach to strip occupancy; `REACQUIRE` re-latched on corn rather than width; the exit
detector permitted its own scan rows; and diagnostics raised to 1 Hz whenever an accumulator
is moving — at 2 Hz only one frame in ten had been visible, so a partially firing signature
looked identical to no signature at all.

**Generation 5 (2026-07-30) — the blocked-gate quantity fix.** Described in §III-D.2(c). The
canonical example in this project of a wrong *quantity* rather than a wrong *number*.

### S6.5 Mission FSM and the headland leg

The mission layer was added on 2026-07-03. Its most-revised component is the headland exit
leg, rebuilt over three simulation runs on 2026-08-07, each exposing a different defect:

1. **The sign rule was wrong.** The "negate if and only if exactly one of {mirrored view,
   reversed motion}" rule inverted heading feedback and steadily walked the robot off the
   row. Replaced by the reconstruction of Eq. (36). The unit test that had pinned the old
   rule was deleted, because it pinned the bug.
2. **Steering on a corridor that had run off the edge of the image.** With
   `edges = 1.00/0.00`, the command sat at the 0.175 rad/s clamp for the whole leg —
   roughly 150° of unintended rotation, which is also why the rear exit never fired. Fixed
   by the bounded-corridor gate of Eq. (11).
3. **The rear watcher re-bought evidence the front camera had already paid for**, having
   inherited the 0.4 m confirmation distance, so the leg always ran ≈ 0.4 m past the row end.
   Fixed by a dedicated 0.1 m threshold and by separating the two rear watcher objects.

Three further bugs from the same session are recorded because each is a general trap: a
tick that carried no rear frame was being read as revocation evidence; the inference loop
could wait forever for a rear frame, making the state machine's own no-rear-frames fallback
unreachable; and the leg's distance ceiling used to resume `FOLLOW_ROW` mid-headland, where
the detector must re-arm over 2.0 m of open field.

### S6.6 Field trials

| Date | Configuration | Outcome |
|---|---|---|
| 2026-07-09 | CPU robot, tall mount, lab row | First deployment; live row following validated at ≤ 0.15 m/s |
| 2026-07 | CPU robot, tall mount, real corn | First successful field test; **0–1 interventions per row**; exits and turns worked |
| 2026-07-15 | real corn | `headland_clearance` 0.5 m clipped the last plants; $\omega_{\max} = 0.3$ wobbled into corn |
| 2026-07-24 | GPU robot, low mount | **Mid-row false exit, drove into corn** (A.4) |
| 2026-07-29 | GPU robot, same route | **No false detections** — the rebuild worked. One defect: turn began too early → `headland_clearance` 0.75 → 1.0 m |
| 2026-08-05 | GPU robot, WDR front low + Brio rear | **Full multi-row mission in real corn with no interventions**, including the first real-corn blocked-row reverse recovery |

**The 2026-08-05 confound, which must be disclosed.** That run was executed with a **flat
right-rear tyre**, discovered afterwards. Three simultaneous effects, all of which bear on
the results:

1. A constant steering disturbance — the Jackal belts both wheels on a side together, so a
   flat wheel turns at the same rate on a smaller rolling radius and scrubs. Against a
   controller with no integral action (A.3) this settles at a permanent lateral offset by
   construction, which is exactly the "row hugging" that was observed.
2. It explains the one observation that had looked anomalous: raising $\omega_{\max}$ from
   0.175 to 0.25 changed nothing, because the controller was never saturating. **That it did
   not saturate is positive evidence for a constant disturbance and against "the MPC is too
   weak."**
3. Wheel odometry converts encoder counts with a *nominal* radius, so distance is
   over-reported. Every distance-valued threshold in the system — the 2.0 m arming, the
   0.4 m confirmation, the 1.0 m clearance, the 0.6 m traverse — then fires **early in real
   metres**.

The agreed protocol before drawing any conclusion from that run is two short vision-free
tests, executed both before and after re-inflation: straight-line drift over 5 m at
0.15 m/s (under ≈ 10 cm is fine; half a metre is the hugging problem), and an odometry scale
check against a tape measure (5.3 m reported over a true 5.00 m means every distance
threshold is 6 % off).

### S6.7 Instrumentation and autonomy measurement

Built on 2026-08-04 in response to a direct request for performance numbers, which the
pipeline at that point had no way to produce. The design constraints recorded at the time —
one schema definition, a logger that can never raise, a flush on every event row, and a
single `summarize()` shared by the live console and the offline report so that the two
cannot disagree — are described in §III-H. The instrument still has **not produced data from
a field run**.

### S6.8 Rejected approaches, open defects, and gaps

**Rejected, with reasons:**

| Proposal | Why rejected |
|---|---|
| Adaptive/relative exit-width threshold (learn the in-row median, fire at ≈ 1.35×) | Unnecessary while the flank rule pins the bar near 1.0; adds state that fails silently |
| Odometry row-length fallback for the exit | Rejected 2026-07-28; the consequence — a genuine segmentation failure over open ground has nothing catching it — was recorded rather than hidden |
| A dedicated steering clamp and yaw limit for the headland leg | New control mechanisms invented to bound a *symptom* whose cause was a bad measurement (§III-E.4) |
| Alternating cameras through the headland leg | Halved the frame rate and made the operator view flip several times a second, at exactly the moment an operator must judge whether the robot is leaving the row straight |
| Offline mask-vs-prediction harness for the false-exit bug | The reported widths were accurate; the fault was detector logic, not segmentation quality |

**Open defects (deliberately not fixed as of 2026-08-07):**

1. **Row hugging** — three candidate causes remain, of which border clipping of the near
   scan row (22–33 % of in-row frames) is the best supported; confounded by the flat tyre.
2. **Large plant with side gaps** — Eq. (26) requires zero corridor at *every* scan row, but
   the near row at the bumper still images ground under a tall plant, so BLOCKED can never
   fire in that geometry.
3. **Leaves on the lens** are indistinguishable from a corn wall, producing a 4 s stop and a
   spurious recovery. The unused sky class is the natural discriminator (§III-A.3).
4. **A joystick-takeover report from 2026-07-24** was never reproduced; a bench re-test on
   2026-07-28 worked on unmodified code with the joystick topic steady at 20 Hz. Two theories
   are dead (arbitration is by priority, not rate; the operator was standing beside the
   robot), leaving operator technique as the leading candidate. The attempted fix was
   reverted so that the field re-test runs on exactly the validated configuration.

**Gaps that will be raised in review, listed so they are not discovered late:**

- No comparison of DINOv3 against any alternative backbone, and no ablation of the
  three-class scheme.
- No lateral tracking error in metres anywhere — every offset figure is normalized image
  space, and cross-rig comparison is explicitly invalid.
- No MDBI figure from any run.
- The headline 2026-08-05 field result is confounded by the flat tyre.

### S6.9 Cross-cutting principles

These emerged repeatedly across subsystems and are candidates for the paper's discussion
section, since several are transferable beyond this system:

1. **Never debounce in frames.** A frame count means a different thing on every robot, and
   this fleet spans 2 Hz to 25 Hz. Use metres for anything confirmed by driving, seconds for
   anything confirmed while stopped.
2. **Leaky, never strictly consecutive.** At 25 Hz a 0.4 m window is ≈ 65 frames; a
   reset-on-any-dropout rule would never complete.
3. **An evidence leak must be asymmetric** or a 50 %-duty signature nets exactly zero
   forever.
4. **Never latch on corridor width** — width is a camera-height constant.
5. **The quantity can be wrong, not just the number.** A threshold retuned three times
   against the same symptom is a sign that the wrong thing is being measured.
6. **A false positive should cost a wobble, not a collision.** Revocation exists because the
   2026-07-24 failure was unrecoverable, not because the classifier was unusually bad.
7. **Diagnose before tuning.** When the input is fiction, the controller did exactly what it
   was told; raising its limits is the wrong response.
8. **Instrument before the field pass**, because a field pass does not come round twice.
9. **One home per knob.** For a period, launch-file parameters silently overrode the
   configuration file for 44 duplicated keys, so editing the configuration did nothing. It
   caused two separate misdiagnoses before it was found.

---
