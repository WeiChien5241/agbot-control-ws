# AIAgNav: A Semantic Segmentation-Based Autonomous Navigation System for Cornfields

**Authors:** Wei-Wei Chien, *et al.*, and David J. Cappelleri
School of Mechanical Engineering, Purdue University, West Lafayette, IN 47907 USA

**Target venue:** IEEE ICRA 2027 — 8 pages including references
**Revision date:** 2026-08-24
**Companion document:** `AIAgNav_Supplementary.md` (S1–S12)

---

## Abstract

Under-canopy navigation in cornfields must be closed on onboard perception: the corridor
between rows is under a metre wide and GNSS degrades exactly where the canopy is densest.
Existing under-canopy systems solve this with 3D LiDAR, which measures geometry and cannot
separate a rigid stalk from a leaf hanging into the corridor. This paper presents AIAgNav, a
navigation system that drives a ground robot in-row, under-canopy, and across multiple rows
from a single monocular RGB camera. A DINOv3 backbone with a mask transformer head segments
each frame into sky, traversable ground and obstacle; closed-form geometry reduces the mask
to a lateral error and a heading proxy; and a receding-horizon model predictive controller in
normalized image coordinates issues the steering command, with no filter, no camera
calibration and no state estimator in between. A mission layer detects the end of a row from
the same mask, turns onto the headland, re-enters the next row, and reverses out of blocked
rows. The system is evaluated in simulation and in a real cornfield,
**[TODO: headline result — distance covered, rows completed, MDBI]**.

**Index Terms** — Robotics and automation in agriculture and forestry, agricultural
automation, semantic segmentation, visual navigation, model predictive control.

---

## I. INTRODUCTION

Precision agriculture depends on measurements taken close to the plant, and the measurements
that matter most for breeding and nutrient management — stalk diameter, lower-leaf condition,
soil surface state — are the ones the canopy hides [7], [8]. Aerial platforms and
high-clearance tractors observe a field from above and lose access to the plant interior as
the season advances [9]. An under-canopy ground robot is the most direct route to per-plant
measurement in row crops: a robot small enough to drive between two corn rows reaches the
stalk, the lower leaves and the soil surface directly, and can carry a sampling arm to the
plant rather than the plant to the laboratory [2], [18].

The environment such a robot must drive in is among the least forgiving available to a mobile
robot. The corridor between two rows of mature corn is under a metre wide, bounded by plants
rigid enough to damage a robot and flexible enough to sweep across its sensors, and lit by a
canopy that produces order-of-magnitude illumination changes over a few metres of travel.
Classical map-and-plan navigation is a poor fit: hanging leaves and weeds that were absent
when a map was built register as obstacles, and the planner either detours around growth the
robot could simply drive through or fails outright [12]. GNSS, which carries most agricultural
autonomy above the canopy [10], [11], degrades under it exactly where the corridor is
narrowest, because the plants that define the corridor also block and reflect the signal.
Navigation under the canopy therefore has to be closed on onboard perception.

This work follows a line of under-canopy systems developed on the same platform. P-AgBot [2]
centres the robot in a row by balancing left and right distances from a 2D LiDAR and
demonstrates in-row physical sampling. P-AgSLAM [3] builds an under-canopy state estimate from
3D LiDAR features. P-AgNav [1] extends that sensing into a complete navigation system: it
projects the 3D point cloud into a range view, extracts the row corridor from that image, and
sequences in-row driving, end-of-row detection and headland turning into multi-row missions
without GNSS or pre-defined waypoints. The navigation problem in a cornfield is, in that
sense, not open. What this work questions is the sensor the solution is built on.

A range sensor measures geometry, and the question a row-following controller asks is not
geometric. A return at 0.6 m to the left of the robot is the same measurement whether it came
from a stalk, which must be avoided, or from a leaf hanging into the corridor, which the robot
drives through several times a minute; no post-processing of the point cloud recovers a
distinction that was never measured. The consequence is that a geometric system must treat the
corridor conservatively, and that its notion of the row end is a matter of where returns stop
rather than of what the robot is looking at. A 3D LiDAR is also the most expensive component
on the robot by a wide margin, which matters for a platform intended to be deployed in numbers
across a research farm.

Camera-based under-canopy navigation addresses the semantic gap and has taken two broad forms.
The first regresses the navigation state directly from the image. CropFollow [5] predicts
heading and lateral distance from a monocular frame with a learned predictor and fuses the
estimate with inertial measurements in a filter; the approach is robust and has been
demonstrated over long distances, but it places the entire navigation state inside a network
whose failures are difficult to inspect and whose training set must be large. The second
extracts an explicit geometric structure from an intermediate representation. Agronav [4]
segments the traversable region and fits a semantic line to it, an approach much closer to the
one taken here, but is developed for above-canopy field roads and inter-row lanes rather than
for the occluded, low-contrast interior of a mature corn row. Classical crop-row detection
from images [13] shares that limitation. Neither line of work addresses the full multi-row
mission — exit, headland turn, re-entry, and recovery when the row is blocked — which is what
separates a row follower from a system that can cover a plot.

AIAgNav is a monocular, camera-only navigation system for that setting. A single
forward-facing RGB frame is segmented into sky, traversable ground and obstacle by a DINOv3
[6] backbone with a mask transformer head, chosen for label efficiency because the annotations
must be produced by hand from footage of the target field. The mask is reduced to a
two-element navigation state by closed-form geometry rather than by a learned regression head,
and that state is consumed directly by a receding-horizon model predictive controller
formulated in normalized image coordinates. A mission layer detects the end of the row from
the same mask, turns onto the headland, re-enters the next row, and reverses out of rows that
are blocked rather than finished. No GNSS, no map, no waypoints, no camera calibration and no
LiDAR are used at any stage. Fig. 1 shows the resulting pipeline.

The contributions are:

- A complete in-row and multi-row cornfield navigation system driven by a single low-cost
  monocular camera, with a second identical camera used only for reverse and headland
  manoeuvres. The control problem is posed entirely in normalized image coordinates, so no
  intrinsics, extrinsics or metric scale enter the control law.
- A navigation state read off a semantic segmentation mask by geometry rather than regression,
  giving a lateral error and a heading proxy from a single frame, at several depths, with no
  filter and no inertial input between perception and control.
- A mission layer for multi-row operation whose end-of-row evidence accumulates in metres and
  seconds rather than in frame counts, so that one set of thresholds transfers unchanged
  between robots whose inference rates differ by an order of magnitude, together with a
  rear-view-steered headland leg and a blocked-row reverse recovery.

GNSS is used nowhere in the system described here. An RTK receiver is planned for the
above-canopy transit from the field edge to the row entrance, which is the one regime in which
it is reliable, and it remains outside the navigation system presented in this paper.

---

## II. SYSTEM OVERVIEW

This section describes the robot, explains why a monocular RGB camera was selected as the
primary navigation sensor, and gives the overall structure of the framework. Section III then
treats each module in detail.

### A. System Hardware

AIAgNav runs on P-AgBot [2], a Clearpath Robotics Jackal J100 skid-steer platform, the same
robot used in the lab's earlier under-canopy work [1], [3]. For this work it carries no LiDAR.
The navigation sensing is two commodity USB cameras, and the only other input is the
platform's stock state estimator, which fuses wheel encoders with an onboard IMU.

The two cameras are mounted on the top deck as an exact geometric mirror about the robot
centre: the same height, the same inset from their respective deck edges, and the same field
of view, one facing forward and one facing back. The symmetry is load-bearing rather than
cosmetic, because the headland manoeuvre of Section III-E steers on the rear image, and the
conversion of that view into the forward navigation state assumes the reverse view is the
geometric twin of the forward one. Both cameras run at the resolution and frame rate of the
annotation footage, so no resolution domain gap separates training from deployment.

The cameras sit on the deck rather than on a mast, because as the canopy closes an elevated
camera is repeatedly occluded by leaves. The mount pose is not a free choice downstream: it
sets the nominal corridor width that the row-exit logic of Section III-D reads, near seven
tenths of the image width at the nearest scan row here against roughly half at mast height.
Width thresholds are properties of a camera pose, not of a cornfield.

Two physical Jackals were used, and the difference between them shaped the real-time design.
One carries the stock computer with no discrete GPU and completes a segmentation forward pass
in about 500 ms, giving a control rate near 2 Hz; the other carries a discrete GPU and runs
the same weights in 16 ms, near 24 Hz. A twelvefold spread in control rate between two
instances of the same platform is why the row-exit evidence of Section III-D accumulates in
metres and seconds rather than in frames, and why the controller of Section III-C rescales its
own dynamics to the measured control period. Odometry is used for exactly three purposes —
measuring distance travelled inside a row, closing the loop on the headland turns, and
measuring path length for the autonomy metric. No map is built and no global localization is
performed.

### B. Visual Representation

The central argument of P-AgNav [1] is that a 3D LiDAR range view is a compact,
illumination-independent representation from which navigation features can be extracted
cheaply. That argument is not in dispute here. The motivation for the present system is a
different limitation: a range sensor cannot answer the question the controller actually asks.

Navigation inside a corn row reduces to a per-region question — can the robot drive there? A
range return at 0.6 m to the left is the same measurement whether it came from a rigid stalk,
which must be avoided, or from a hanging leaf, which the robot drives through routinely. No
geometric post-processing recovers that distinction, because the information was never in the
measurement. Semantic segmentation of an RGB frame answers the drivability question per pixel
by construction, and it does so with a sensor that costs a small fraction of a 3D LiDAR. A
forward- and slightly downward-looking camera additionally images the ground ahead of the
robot at many depths in a single frame, which Section III-B exploits to recover both the
lateral and the heading component of the navigation state without any temporal filtering.

The costs of the choice are absorbed by the design rather than denied. Light under a closed
canopy varies by more than an order of magnitude within a single row; the system meets this by
learning the ground/crop distinction from footage spanning those conditions, and by defining
an explicit safe behaviour for frames whose mask is unusable. A monocular camera supplies no
metric depth; the entire control problem is therefore posed in normalized image coordinates,
and no intrinsics, extrinsics or metric scale enter the control law at any point. A camera
sees far less than a 360° LiDAR; the mirrored rear view covers the two manoeuvres the forward
view cannot serve — clearing the headland and reversing out of a blocked row. One failure mode
is not absorbed: a leaf resting on the lens and a crop wall directly ahead produce the same
mask, and the system treats both as blocked.

### C. Framework Overview

The field is assumed, as in [1], to consist of approximately straight parallel rows of uniform
spacing, arranged in plots separated by headland space wide enough for the robot to turn, with
each row aligned to a corresponding row in the opposite plot. Within that setting the system
covers five operational stages: in-row navigation under the canopy; classification of the end
of the row from the mask alone; row switching across the headland; recovery from a row that is
blocked rather than ended; and termination after a commanded number of rows, with the blocked
rows reported.

Fig. 1 shows the resulting data flow. One frame enters and one velocity command leaves. Two
properties of the pipeline are worth stating at the overview level.

There is no state estimator between perception and control. The class mask is reduced to a
two-element navigation state by closed-form geometry and handed to the model predictive
controller directly: no Kalman filter, no learned regression head, and no fusion with inertial
measurements. Temporal smoothing is instead a property of the controller, expressed as a rate
penalty and a rate constraint inside the same optimization that produces the command
(Section III-C), so it is subject to the same limits as the rest of the control problem
instead of forming a separate tuning surface.

Perception is shared rather than duplicated. The row-exit detector consumes the same boundary
search the centerline estimator already performs; corridor widths and border occupancy are
by-products of that scan. End-of-row classification therefore costs essentially nothing beyond
the segmentation forward pass, which is what allows it to run at the control rate on the
CPU-only robot.

> **[Figure 1 — AIAgNav processing pipeline.]** *Source: `paper/fig/pipeline.tex`.*
> A single camera frame is segmented into sky, traversable ground and obstacle; a scan-row
> geometry stage reduces the mask to a lateral error and a heading proxy, which drive the
> image-space MPC; the same scan feeds the row-exit detector, and the mission state machine
> sequences row following, headland turns and blocked-row recovery. Odometry enters only at
> the detector and the state machine.

### Table I — Summary of Notations

| Symbol | Description |
|---|---|
| $H,\ W$ | Mask height and width, px |
| $c_x$ | Image centre column, $c_x = W/2$ |
| $f_i,\ \lambda_i$ | Height fraction and pooling weight of scan row $i$ |
| $x_{L,i},\ x_{R,i}$ | Corridor bounds on scan row $i$ |
| $x_{\text{mid},i},\ w_i$ | Corridor midpoint and width on scan row $i$ |
| $e_i,\ e$ | Per-row and pooled normalized lateral error |
| $s$ | Heading proxy (slope term) |
| $\tau,\ \tau_{\min}$ | Traversable fraction of the lower half of the mask; its threshold |
| $\mathbf{x},\ u$ | MPC state $[\,e\ \ s\,]^\top$ and control $u = \omega$ |
| $A,\ B$ | System and input matrices |
| $\alpha,\ \beta$ | Lateral coupling and control effectiveness |
| $Q,\ r,\ r_\Delta$ | State, control-effort and control-rate weights |
| $N$ | Prediction horizon |
| $dt,\ dt_0$ | Control period and reference control period |
| $\omega_{\max},\ \Delta\omega_{\max}$ | Angular rate and slew limits |
| $v$ | Commanded linear velocity |
| $m,\ \Delta_k$ | Accumulated exit evidence and its per-frame increment |
| $\rho,\ m_c$ | Evidence leak ratio and confirmation threshold |

---

## III. SYSTEM DESIGN

### A. Semantic Segmentation

The navigation state is derived from a per-pixel semantic segmentation of a single
forward-facing RGB frame. Under a corn canopy the appearance cues that separate drivable
ground from crop are unstable: illumination changes by more than an order of magnitude between
direct sun and deep shade within one row, hanging leaves throw high-contrast shadows across
the corridor floor, and soil colour shifts with moisture and tillage. Hand-tuned colour or
texture thresholds are adequate over a short segment but do not survive these transitions, and
the row-following controller carries no independent state estimate to fall back on when they
fail. The system therefore learns the ground/crop distinction rather than specifying it.

The network pairs a DINOv3 [6] ViT-S/16 backbone [14] with an encoder-only mask transformer
head [16]. The choice is driven by label efficiency rather than by accuracy on a public
benchmark. Annotated data for this task cannot be inherited from an existing dataset: it must
be produced by hand from footage recorded in the target field, at the growth stage and camera
mounting of the deployed robot, and the achievable quantity is bounded by how much one
operator can label in a season. The training set here is 443 annotated frames, split 80/20
between training and validation, curated to cover the cases that break a row follower rather
than the average frame: entering a row, dead-end rows, gaps where plants are missing, and
downed corn. Self-supervised features of this family transfer to dense prediction under very
little supervision [15], which is what makes a set of that size viable; a backbone trained
from scratch, or one supervised on ImageNet classification, would demand substantially more
annotation before producing masks clean enough to measure corridor boundaries on. ViT-S/16 is
the smallest variant in the family, which matters because one set of weights must serve both
robots in the fleet, only one of which carries a GPU. Measured segmentation quality on the
held-out split is 0.8717 mIoU.

The model predicts three classes: sky, traversable ground, and obstacle. The controller reads
only the traversable class, so the presence of a sky class deserves an explanation. At a row
end, and throughout a headland, the upper part of the frame opens onto bright sky. A two-class
model must assign those pixels to either ground or crop, and their brightness and near-absence
of texture make them resemble open ground far more closely than they resemble a corn plant.
Because the exit detector of Section III-D fires when the traversable corridor widens toward
the full image width, folding sky into the traversable class reproduces the exact signature of
a row end at arbitrary points inside a row. A dedicated sky label removes that failure mode
for the cost of one output channel. The obstacle class, conversely, is deliberately coarse: it
covers the corn plants bounding the corridor, fallen stalks, and any foreign object, because
the control decision they induce is identical — do not drive there. Separating them would add
annotation burden without changing a single command.

At inference the model is given the full-resolution camera frame and returns a class-index
mask, which is resized to the frame resolution when the two differ. That resize must be
nearest-neighbour. Mask values are class indices, not intensities, so any interpolating kernel
synthesises intermediate values that correspond to no class at all, and it does so precisely
at region boundaries — which is exactly where Section III-B measures the corridor edges. A
single misplaced boundary column biases the lateral error for that frame. The training
configuration and the full inference contract are given in Supplementary S7.

### B. Corridor and Centerline Estimation

The mask is reduced to a two-dimensional navigation state by pure geometry. No regression head
is trained to predict heading or lateral distance, and no camera intrinsics enter the
computation: the state is read directly off the class mask, so it is defined in normalized
image coordinates and is independent of camera resolution.

The system measures the corridor on a small set of horizontal scan rows (Fig. 2) placed at
fractions $f_i$ of the image height, ordered from farthest to nearest. On each scan row it
starts at the image centre column $c_x$ and expands outward while the mask remains traversable,
giving the bounds $x_{L,i}$ and $x_{R,i}$ of the contiguous traversable run that straddles the
centre column. Each valid scan row then gives a lateral error, normalized by the half-width so
that it is dimensionless and bounded, and these are pooled into a single lateral state $e$ by
a weighted mean over the valid rows:

$$
\begin{aligned}
x_{\text{mid},i} &= \tfrac{1}{2}\big(x_{L,i} + x_{R,i}\big),
&\qquad w_i &= x_{R,i} - x_{L,i}, \\[4pt]
e_i &= \frac{x_{\text{mid},i} - c_x}{W/2},
&\qquad e &= \operatorname{clip}\!\left(
  \frac{\sum_i \lambda_i e_i}{\sum_i \lambda_i},\; -1,\; 1 \right), \\[4pt]
s &= e_{\text{far}} - e_{\text{near}}. &&
\end{aligned}
\tag{1}
$$

Anchoring the search at $c_x$ rather than taking the widest traversable run in the row is what
makes the measurement unambiguous when the mask contains more than one open region, as it does
whenever a gap in the crop wall exposes the neighbouring row: the corridor the robot is
actually in is the one aligned with its own heading. If the centre column is not traversable,
that scan row yields no measurement and is skipped. The width $w_i$ is retained and consumed
by the exit detector of Section III-D.

A positive $e$ places the corridor midpoint right of the image centre, meaning the robot sits
left of the corridor centre. The weights $\lambda_i$ favour the near rows, because near and far
rows fail in opposite ways: the near rows report where the robot is now and segment cleanly,
whereas the far rows carry the earliest warning of a curve or a row end but sit where
perspective compresses the corridor to a few pixels. Weighting toward the near field keeps
steering stable while leaving the far rows enough influence to anticipate.

Because the same scan already yields an offset at several distances, the robot's heading
relative to the row follows from their difference at no additional cost. The slope term $s$ is
evaluated between the farthest and nearest valid rows: a corridor that tilts rightward with
distance indicates the robot is heading left of the row direction. This is the term that
separates the two ways a robot can be wrong inside a row — displaced from the centreline, and
pointed across it — which a single lateral reading cannot distinguish. It is computed across
rows within one frame rather than across time, so it needs no history, no filter and no
inertial measurement. Under a pinhole model the pair $(e, s)$ is an invertible linear transform
of the physical (lateral offset, heading error) pair, so no navigation information is lost by
never leaving image space; the derivation is given in Supplementary S8.

A frame is accepted only if at least one scan row produced a measurement and the traversable
fraction $\tau$ of the lower half of the mask reaches a threshold $\tau_{\min}$. The second
test rejects the case where the mask has collapsed — an occluded lens, a lost row — while a
narrow spurious run at the centre column still yields a confident-looking midpoint. On
rejection the controller holds its previous command rather than steering on the frame, and
stops if rejections persist.

> **[Figure 2 — Corridor measurement on a class mask.]** *Placeholder: to be replaced with a
> debug-overlay capture from a field run.* The traversable region is washed over the camera
> frame; on each scan row the search starts at the image centre column and expands outward to
> the corridor bounds, whose midpoint gives the per-row lateral error. The difference between
> the far and near rows is the heading proxy.

### C. Model Predictive Row Following

The controller converts the state $(e, s)$ into an angular velocity. A proportional law on $e$
alone is sufficient to hold a straight row but couples poorly to the heading term, and it
offers no principled way to bound how sharply the command may change between frames — a real
constraint here, since a single badly segmented frame must not produce a step in steering. The
system therefore uses a receding-horizon model predictive controller [17], which expresses
both requirements as costs and constraints on the same optimization.

The state is $\mathbf{x} = [\,e\ \ s\,]^\top$ and the control is the angular velocity
$u = \omega$. The cost penalizes deviation from the row over the horizon $N$ while
regularizing the command:

$$
J = \sum_{k=1}^{N} \mathbf{x}_k^\top Q\, \mathbf{x}_k
  \;+\; \sum_{k=0}^{N-1} \Big[\, r\,u_k^2
  \;+\; r_\Delta \big(u_k - u_{k-1}\big)^2 \,\Big]
\tag{2}
$$

with $Q = \operatorname{diag}(q_e, q_s)$. The two state weights encode a priority: being
off-centre is penalized considerably more than being misaligned, because a heading error
corrects itself as the robot advances whereas a lateral offset does not. The term in $r$
discourages large commands, and the term in $r_\Delta$ penalizes change between consecutive
commands. That last term is what supplies temporal smoothing in this system. Learned
under-canopy controllers typically obtain smoothing by fusing the visual estimate with
inertial measurements in a Kalman filter [5]; here it falls out of the cost function, so the
pipeline needs neither an IMU nor a filter state that must be re-initialized whenever the
mission is paused.

Over one control period the corridor midpoint drifts laterally in proportion to the current
heading error, and the heading error itself is driven by the commanded rate. The state is
updated by a linear time-invariant model in normalized image space, subject to a magnitude
bound and a slew bound on the command:

$$
\begin{aligned}
\mathbf{x}_{k+1} &= A\,\mathbf{x}_k + B\,u_k,
&\quad A &= \begin{bmatrix} 1 & \alpha \\ 0 & 1 \end{bmatrix},
&\quad B &= \begin{bmatrix} 0 \\ \beta \end{bmatrix}, \\[4pt]
|u_k| &\le \omega_{\max},
&\quad |u_k - u_{k-1}| &\le \Delta\omega_{\max}. &&
\end{aligned}
\tag{3}
$$

Here $\alpha$ is the lateral coupling — how much heading error translates into lateral drift
per step — and $\beta$ is the control effectiveness, how much the commanded rate moves the
heading term per step. Both are identified empirically. This is a point of departure from
range-view systems such as P-AgNav [1], where a 360° horizontal field of view maps angular
velocity to image displacement in closed form. A perspective camera admits no such constant:
the pixel displacement produced by a given rotation depends on depth, which is unobserved.
Fitting the two scalars on logged runs avoids requiring the depth that would make them
analytic. In (3), $u_{-1}$ is the command applied on the previous frame, so the slew constraint
binds across frames and not merely within the planned sequence. The problem is solved by
sequential least-squares quadratic programming at every accepted frame; only the first element
$u_0$ is applied, and the remainder of the horizon is discarded and recomputed on the next
frame.

One practical requirement shapes the parameterization. The same controller runs on two robots
whose inference rates differ by more than an order of magnitude, and a model calibrated per
*step* is wrong on both unless the step is fixed. The parameters $\alpha$, $\beta$ and
$\Delta\omega_{\max}$ are therefore specified at a reference control period $dt_0$ and scaled
by $dt/dt_0$ at construction. The per-step drift and control effectiveness then match the real
elapsed time per step, and the slew limit stays constant in rad s⁻² rather than silently
tightening as the loop slows.

The linear velocity is held at a constant cruise value while the robot is following a row.
This is a deliberate simplification relative to P-AgNav [1], which couples speed inversely to
commanded curvature so the platform slows through sharp corrections. Under the row geometry
considered here the commanded rate stays small enough that the coupling changes little, and a
constant speed makes the distance-based logic of Section III-D easier to reason about. Speed
is reduced only during the headland manoeuvre, where overshoot is most costly. The sign
convention, the tuned weights and the order in which they were identified are given in
Supplementary S1, S2 and S9.

### D. Row Exit Detection

The system recognizes two end-of-row signatures, both read from quantities Section III-B has
already computed. In the open signature the corridor widens from the narrow band characteristic
of an occupied row toward the full image width, and the outer strips at the image borders
become traversable: the crop walls have ended. The test requires a sufficient number of scan
rows to be simultaneously wide and flank-clear, but deliberately does not require *particular*
rows. Requiring the far rows is the intuitive choice and fails in the field, because beyond the
last plants the distant ground segments poorly and those rows can remain invalid indefinitely.
In the blocked signature no scan row yields a corridor at all while the lower half of the mask
still contains obstacle pixels, which is a crop wall square ahead rather than a lost mask.

Neither signature may be trusted on a single frame, and the debounce is where the design
matters. Evidence accumulates in *physical* units rather than in frames:

$$
m_{k+1} =
\begin{cases}
  m_k + \Delta_k, & \text{signature present},\\[4pt]
  \max\!\big(m_k - \rho\,\Delta_k,\; 0\big), & \text{otherwise},
\end{cases}
\qquad \text{firing when } m_k \ge m_c .
\tag{4}
$$

The increment $\Delta_k$ is the distance driven since the previous frame for the open signature
and the elapsed time for the blocked signature. The units differ because the two signatures
fail differently: an exit must be confirmed by driving through it, so if the robot stalls the
safe outcome is not to fire, whereas a blocked view stops the robot outright, so a
distance-based counter would never fill and the recovery would deadlock.

Counting frames instead would not be a simplification but a defect: the same count means more
than a second of evidence on the CPU-only robot and a fraction of one on the GPU robot, so a
mid-row gap that is harmless on one fires an exit on the other. Nor may the accumulator demand
strictly consecutive frames, since a single flickering frame would then reset a streak spanning
tens of them. The leak is asymmetric, $\rho < 1$, because a symmetric leak sounds neutral and is
not: a signature holding on half the frames nets exactly zero and can never fire however far the
robot drives, which is precisely what a real but marginal exit looks like.

Each signature is armed by distance travelled into the row — the open signature only after the
robot is far enough in that the open field behind it has left the frame, the blocked signature
much earlier, since an obstacle just past the row entrance must still be caught and that
signature cannot false-fire there. When an exit fires, the row-entry distance is back-dated to
the start of the evidence streak rather than to the confirming frame, so the confirmation
distance is not charged twice against the headland manoeuvre that follows. Both signatures, the
duty-cycle analysis that sets $\rho$, and the arming rule are given in full in
Supplementary S10.

### E. Row Switching

Multi-row operation is sequenced by a finite state machine, Fig. 3. On a confirmed open exit
the robot leaves row following and drives clear of the last plants, then executes a quarter
turn onto the headland, traverses to the neighbouring row, turns again to face down it, and
reacquires row following once the segmentation mask presents a corridor. The two turns and the
traverse are closed on wheel odometry rather than on vision, because the headland offers no
corridor to steer by; each turn terminates on accumulated swept yaw rather than on elapsed
time, and vision resumes as the authority the moment the robot is inside the next row. Turn
direction alternates between switches, so the robot covers the plot in a boustrophedon pattern,
and the mission ends after a commanded number of rows or when reacquisition finds no corridor,
which is how an unbounded mission recognizes that no rows are left.

The clearing leg runs slower than the row-following cruise, since overshoot there is what
misaligns the entry into the next row. Where a rear camera is available, that leg is steered
from the rear view looking back down the row just left, which reports directly whether the tail
has cleared the last plants; the conversion from the rear view to the same state used in
Section III-C is not a sign flip, and is derived in Supplementary S3. A false exit is made
cheap rather than made impossible: if the robot has left row following but the nearest scan row
still reports crop on both flanks over a short distance, the exit is revoked and row following
resumes with the row count unchanged.

The blocked signature enters a separate branch. Because the system drives only forward within a
row, a crop wall ahead cannot be turned around in place without striking plants, so the robot
reverses out of the end it entered under rear-view guidance and odometry bounds, then joins the
next row with an S-turn. That row is traversed in the same direction as the blocked one rather
than the alternating direction, since the robot never reached the far end, and the boustrophedon
flip is suppressed once to account for it. Blocked events are counted and reported at mission
completion, as they mark rows that were not fully covered. The branch is gated on the presence
of the rear camera; without it a blocked signature stops the robot and ends the mission. The
full state machine is given in Supplementary S11.

> **[Figure 3 — Mission state machine.]** *Source: `paper/fig/fsm.tex`.*
> The upper chain is the nominal cycle: a confirmed open exit sends the robot across the
> headland and back into row following. The lower chain is the blocked-row branch, which
> reverses out of the row and joins the next one in the same direction of travel. The mission
> terminates after the commanded number of rows, and also when reacquisition finds no corridor.

---

## IV. EXPERIMENTAL RESULTS

> **[RESERVED — approximately 850 words / one page, plus Tables II–V and Fig. 4.
> Deferred until field data exists; see the measurement checklist below.]**

Experiments are conducted to demonstrate the capabilities of the proposed system across
cornfield scenarios in both simulation and a real cornfield, referred to as SIM and ACRE (the
Agronomy Center for Research and Education at Purdue) respectively, matching the evaluation
structure of [1].

Performance is evaluated on three criteria. The first two follow [1]: collisions with crops,
counted under the same definition, and human interventions. The third is **meters between
interventions (MDBI)**, defined as the distance driven autonomously divided by the number of
interventions, where an intervention is a joystick takeover by the supervising operator and
takeover distance is subtracted rather than credited to the controller. MDBI is reported
because it is the figure the field literature compares on, and because unlike a normalized
image-space tracking error it is expressed in metres and is independent of camera mounting. A
run with zero interventions yields only a lower bound on MDBI, so trials are pooled — distances
summed, interventions summed — before a figure is quoted. Instrumentation is described in
Supplementary S12.

**Table II — Specifications of experimental environments.** SIM and ACRE: row spacing, row
length, plant growth stage, robot configuration, camera mount, and compute variant.

**Table III — Navigation performance in SIM.** Rows attempted, rows completed, collisions,
interventions, revoked exits, blocked events.

**Table IV — Navigation performance in ACRE.** The same columns, plus total distance driven and
pooled MDBI.

**Table V — Perception.** Segmentation mIoU on the held-out split (0.8717 measured) and, if it
can be produced, a per-class IoU breakdown.

**Figure 4 — Unforeseen field challenges,** matching Fig. 7 of [1]: sections with missing
plants, downed corn, and leaves occluding the lens.

**What must be measured before this section can be written:**

1. At least one field mission producing a metrics log. None exists; earlier field runs predate
   the instrumentation and would have to be recovered from recorded bags.
2. A pooled MDBI figure across multiple trials on the same robot and camera mount.
3. A re-run of the multi-row mission with all four tyres correctly inflated — see the confound
   in Supplementary S6.6, which may account for the row-hugging behaviour observed in the
   2026-08-05 run in its entirety, and which biases every distance-valued threshold through
   odometry over-reporting.
4. A repeat of the full three-row simulated mission on the current code, confirming the
   2026-08-07 result.
5. A collision count under the explicit definition adopted from [1], for comparability.

The simulation runs currently on record report zero interventions, which yields a lower bound on
MDBI and no mean. They are not field results and must not be presented as such.

---

## V. CONCLUSION

This paper presented AIAgNav, a navigation system that drives a ground robot in-row and
under-canopy in cornfields, and across multiple rows, from a single monocular RGB camera. The
navigation state is recovered from a semantic segmentation mask by closed-form geometry and
consumed directly by a model predictive controller formulated in normalized image coordinates,
so the pipeline uses no GNSS, no map, no waypoints, no LiDAR, no camera calibration and no
state estimator between perception and control. A mission layer detects the end of a row from
the same mask, sequences headland turns, and recovers from blocked rows by reversing out under
rear-view guidance. Expressing end-of-row evidence in metres and seconds rather than in frame
counts allows one set of thresholds to transfer between robots whose inference rates differ by
an order of magnitude, which is what makes the same system deployable on a GPU-equipped
platform and on a stock CPU-only one.

Three directions follow. The first is above-canopy transit: an RTK receiver used only between
the field edge and the row entrance, the one regime in which GNSS is dependable, would close
the gap between deployment and the first row. The second is integration with the physical
sampling module the platform already carries, so that a covered plot yields measurements rather
than a trajectory. The third is the sky class, which the controller currently ignores: a lens
occluded by a leaf and a crop wall directly ahead produce the same mask today, and sky
visibility is the most direct signal available for separating them.

---

## REFERENCES

> **Note.** Entries [1]–[6] are verified and are cited in the body. Entries [7]–[18] are
> **reserved slots** holding the space a submission-quality reference list requires; their
> bibliographic data must be pulled from the source, never hand-written. Filling them is a
> separate pass.

[1] K. Kim, A. Deb, and D. J. Cappelleri, "P-AgNav: Range view-based autonomous navigation
system for cornfields," *IEEE Robot. Autom. Lett.*, vol. 10, no. 4, pp. 3366–3373, Apr. 2025.

[2] K. Kim, A. Deb, and D. J. Cappelleri, "P-AgBot: In-row & under-canopy agricultural robot
for monitoring and physical sampling," *IEEE Robot. Autom. Lett.*, vol. 7, no. 3, pp.
7942–7949, Jul. 2022.

[3] K. Kim, A. Deb, and D. J. Cappelleri, "P-AgSLAM: In-row and under-canopy SLAM for
agricultural monitoring in cornfields," *IEEE Robot. Autom. Lett.*, vol. 9, no. 6, pp.
4982–4989, Jun. 2024.

[4] S. K. Panda, Y. Lee, and M. K. Jawed, "Agronav: Autonomous navigation framework for
agricultural robots and vehicles using semantic segmentation and semantic line detection," in
*Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. Workshops*, 2023, pp. 6272–6281.

[5] A. N. Sivakumar, S. Modi, M. V. Gasparino, C. Ellis, A. E. Baquero Velasquez, G.
Chowdhary, and S. Gupta, "Learned visual navigation for under-canopy agricultural robots," in
*Proc. Robotics: Science and Systems*, 2021.

[6] O. Siméoni, H. V. Vo, M. Seitzer, F. Baldassarre, M. Oquab, C. Jose, V. Khalidov, M.
Szafraniec, S. Yi, M. Ramamonjisoa, *et al.*, "DINOv3," *arXiv:2508.10104*, 2025.

[7] *[RESERVED — precision agriculture / remote sensing review.]*

[8] *[RESERVED — IoT for precision agriculture (IoT4Ag).]*

[9] *[RESERVED — agricultural robotics survey; aerial vs. ground platform trade-off.]*

[10] *[RESERVED — GNSS-based agricultural navigation.]*

[11] *[RESERVED — GNSS-based agricultural navigation, second entry.]*

[12] *[RESERVED — review of classical path-planning strategies for mobile robots.]*

[13] *[RESERVED — crop-row detection from images in maize fields.]*

[14] *[RESERVED — vision transformers for image recognition at scale.]*

[15] *[RESERVED — self-supervised dense visual representation learning (DINOv2).]*

[16] *[RESERVED — encoder-only mask transformer segmentation head (EoMT).]*

[17] *[RESERVED — model predictive control for mobile robot trajectory tracking.]*

[18] *[RESERVED — under-canopy phenotyping / crop monitoring platform.]*
