# Section III — System Design: outline for approval

Budget **2,831 words** (from P-AgNav, 8 pages). Draft currently 11,812 → 4.2× cut.
Allocation below sums to 2,831.

| § | Title | Words | Equations |
|---|---|---:|---|
| III-A | Semantic Segmentation | 700 | — (Table II: class definitions) |
| III-B | Corridor and Centerline Estimation | 600 | (1) (2) (3) |
| III-C | Model Predictive Row Following | 800 | (4) (5) (6) |
| III-D | Row Exit Detection | 400 | (7) |
| III-E | Row Switching | 331 | — (Fig: FSM state diagram) |

Display equations cost ~25–40 words of vertical space each; 7 equations ≈ 250 words
of the budget. That is affordable. More than ~8 is not.

---

## Equation inventory

Numbering assumes Section III owns (1)–(7). Symbols per Table I (notation).

**III-B — Corridor and centerline**

(1) Per-row corridor and midpoint. For scan row *i* at height `y_i = f_i (H−1)`,
scan outward from the image centre column `c_x = W/2` through the contiguous
traversable run:

    x_mid,i = ½ (x_L,i + x_R,i),    w_i = x_R,i − x_L,i

`w_i` is the row-width readout consumed by III-D. One equation covers both.

(2) Normalized lateral error, per-row and pooled:

    e_i = (x_mid,i − c_x) / (W/2),    e = Σ_i λ_i e_i / Σ_i λ_i ∈ [−1, 1]

(3) Heading proxy from the same scan (no extra model, no history):

    s = e_far − e_near

Validity gate (`τ ≥ τ_min` over the lower half, ≥1 valid row) stated in prose,
not numbered — it is a guard, not a contribution.

**III-C — MPC**

(4) State and dynamics. `x = [e, s]ᵀ`, control `u = ω`:

    x_{k+1} = A x_k + B u_k,   A = [[1, α],[0, 1]],   B = [0, β]ᵀ

(5) Cost over horizon *N*:

    J = Σ_{k=1..N} x_kᵀ Q x_k + Σ_{k=0..N−1} [ r u_k² + r_Δ (u_k − u_{k−1})² ]

    Q = diag(q_e, q_s)

(6) Constraints:

    |u_k| ≤ ω_max,    |u_k − u_{k−1}| ≤ Δω_max

Solved by SLSQP each frame; only `u_0` is applied (receding horizon).

**III-D — Row exit detection**

(7) Leaky evidence accumulator, per signature, in physical units:

    m_{k+1} = max(0, m_k + Δ)         if the signature holds
    m_{k+1} = max(0, m_k − ρ Δ)       otherwise
    fire when m ≥ m_confirm

with `Δ = Δd` (metres driven) for ROW_END_OPEN and `Δ = Δt` (seconds) for
ROW_END_BLOCKED, and asymmetric leak `ρ < 1`.

---

## What each subsection argues

**III-A (700 w).** Why a learned mask instead of hand-tuned colour/texture
thresholds; why DINOv3 features (label-efficiency — the training set is one
season of annotated rosbag frames, not a public benchmark); why exactly three
classes, sky / traversable / obstacle, and specifically why *sky* earns a class
when the controller never uses it — it absorbs the bright over-canopy region
that would otherwise be misread as open ground at the end of a row, which is
exactly where III-D has to make its decision. How the mask is produced at
inference and why the resize to frame resolution must be nearest-neighbour
(class indices, not intensities — any interpolation invents classes that do not
exist at region boundaries). Cite DINOv3 and move on; no ViT tutorial.

**III-B (600 w).** The geometric readout. Three scan rows, weighted toward the
near field. Equations (1)–(3). The argument worth making: `e` and `s` together
give a full lateral+heading state from a single frame with no regression head,
no EKF, and no camera intrinsics — the state lives in normalized image space,
so it is resolution- and mount-agnostic in form.

**III-C (800 w).** Equations (4)–(6) plus the reasoning: why a 2-state model
rather than P-AgNav's scalar blob position (heading error is observable
per-frame here and drives lateral drift, so it belongs in the state); what `α`
and `β` mean physically; why `r_Δ` replaces the EKF for temporal smoothing;
why control period scaling (`α, β, Δω_max ∝ dt/dt₀`) is necessary when the same
controller runs at 2 Hz on the CPU robot and ~24 Hz on the GPU robot.
**Explicit contrast to state:** we hold `v` at a constant cruise, where P-AgNav
couples it to curvature via `v = min(c/|ω|, v_max)`.

**III-D (400 w).** The two exit signatures (corridor widens to open field;
crop wall dead ahead). Equation (7) and the one insight that makes it work:
debounce must be in metres and seconds, never in frames, because the same frame
count is 2.5 s on the CPU robot and 0.2 s on the GPU robot. State the asymmetric
leak and why symmetric leak can never fire on a marginal-but-real exit.
Per-signature arming distances in one sentence.

**III-E (331 w).** The FSM figure carries this. Prose covers only: the state
sequence, odometry-closed-loop 90° turns, boustrophedon alternation, and the
blocked-row back-out branch in one sentence. **Recommendation:** the rear-camera
headland leg — the `rear_to_front_state` conversion and the κ gain — is one
sentence here and a full derivation in the supplementary. It is subtle, it cost
the draft heavily, and no reviewer will ask about it at this length.

---

## Table I — Notation (draft)

| Symbol | Meaning |
|---|---|
| `H, W` | mask height, width (px) |
| `c_x` | image centre column, `W/2` |
| `f_i, λ_i` | scan-row height fraction and pooling weight |
| `x_L,i, x_R,i` | corridor bounds on scan row *i* |
| `x_mid,i, w_i` | corridor midpoint and width on scan row *i* |
| `e_i, e` | per-row and pooled normalized lateral error |
| `s` | heading proxy (slope term) |
| `τ, τ_min` | traversable fraction of lower half, and its threshold |
| `x, u` | MPC state `[e, s]ᵀ` and control `ω` |
| `A, B, α, β` | system matrices and their scalar parameters |
| `Q, r, r_Δ` | state, control, and control-rate weights |
| `N, dt` | horizon length and control period |
| `ω_max, Δω_max` | angular-rate and slew limits |
| `v` | commanded linear velocity (constant cruise) |
| `m, ρ, m_confirm` | exit evidence accumulator, leak ratio, threshold |

---

## Three decisions I need from you

1. **Numeric parameter values in the paper?** P-AgNav gives only `v_max` and
   `ω_max` and leaves the rest symbolic. Recommend the same: symbols in III,
   values in one line of IV, full table in supplementary. Costs ~0 words and
   removes the biggest source of bloat in the current draft.
2. **Rear-camera headland leg** — one sentence in III-E with the derivation in
   supplementary (recommended), or a full subsection III-F at ~350 words taken
   from the other subsections?
3. **Is the leaky accumulator a contribution or a detail?** I have it as a
   numbered equation with ~150 words. It is a real failure mode with a
   principled fix and reviewers tend to reward that, but it is defensible to
   cut it to two sentences and reclaim the space for III-C.
