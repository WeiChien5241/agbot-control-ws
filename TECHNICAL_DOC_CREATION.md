# Technical Doc Creation — AIAgNav ICRA 2027 paper

Handoff updated 2026-08-13. Head commit at time of writing: `17b1e09`.
Read this file plus `paper/OUTLINE_III.md` and you have everything.

---

## 1. GOAL

Turn `AIAgNav_Technical.md` (19,098 words, ~63 pages — an excellent design doc,
not a paper) into an **8-page IEEE conference paper** for ICRA 2027, formatted
and pitched like the lab's P-AgNav RA-L paper. The system is AIAgNav: monocular
camera + DINOv3 semantic segmentation + image-space MPC for in-row, under-canopy
cornfield navigation with autonomous multi-row switching. The technical section
is the priority; introduction, results, and references come last.

The deliverable is `paper/paper.tex`, not a markdown file.

---

## 2. CURRENT STATE

**Everything except Section IV is drafted.** Sections I, II, III and V are
written, three figures are in place (two real TikZ diagrams, one placeholder),
and the document compiles clean with no undefined references.

```
pages: 6 / 8      0 undefined references      \nocite{*} removed
```

| Section | Words | Budget | Status |
|---|---:|---:|---|
| Abstract | 177 | 150 | written; one `\TODO` for the headline number |
| I. Introduction | 857 | 1200 | written; 3-item contribution list; **only 6 references** |
| II. System Overview | 1005 | 990 | written (hardware / visual representation / framework) |
| III. System Design | 2528 | 2831 | complete |
| IV. Experimental Results | — | 1003 | **deliberately deferred** (2026-08-13 decision) |
| V. Conclusion | 256 | 247 | written |

**Figures:** `fig/pipeline.tex` (Fig. 1, TikZ block diagram) and `fig/fsm.tex`
(Fig. 2 in III-E, TikZ state machine) are finished and readable at print size.
Fig. 2 in III-B is a `\framebox` **placeholder** for the segmentation overlay —
the user will supply a `debug_viz.py` capture (torch cannot run in this sandbox).

**Two pages are left and three things still want them**: Section IV (~1 page
with tables), the real overlay figure, and a reference list that is currently
6 entries where a submission wants roughly 20. Expect to cut prose from I or II
when related work lands. `make pages` is still the arbiter.

| Subsection | Words | Budget |
|---|---:|---:|
| III-A Semantic Segmentation | 592 | 700 |
| III-B Corridor and Centerline Estimation | 554 | 600 |
| III-C Model Predictive Row Following | 630 | 800 |
| III-D Row Exit Detection | 450 | 400 |
| III-E Row Switching | 313 | 331 |
| **Section III total** | **2,491** | **2,831** |

(`make budget` reports 2,491 for the section; summing subsections gives 2,539.
The small delta is markup the two counters strip differently. `make budget` is
the authority.)

Seven numbered equations, all labelled: `eq:midpoint`, `eq:offset`, `eq:slope`,
`eq:mpc_dynamics`, `eq:mpc_cost`, `eq:mpc_constraints`, `eq:accumulator`.
Section labels: `sec:centerline`, `sec:mpc`, `sec:exit`, `sec:switching`.
Notation table `tab:notation` is filled and matches the symbols actually used.

**The word budget, measured from the P-AgNav paper (exactly 8 pages):**

| Section | Budget | Status |
|---|---:|---|
| Abstract + I. Introduction | 1,344 | stub |
| II. System Overview | 989 | stub |
| III. System Design | 2,831 | **DONE, 2,491** |
| IV. Experimental Results | 1,003 | stub |
| V. Conclusion | 247 | stub |
| References | 958 | 5 entries in `refs.bib` |
| **Total** | **7,372** | |

**Space is running hotter than words suggest.** P-AgNav averages ~920 words per
page; Section III averages ~630, because seven display equations and the
notation table cost vertical space that word counts do not see. Budget the
figures nearer 1.5 pages than 2. **`make pages` is the arbiter, not
`make budget`.**

---

## 3. KEY DECISIONS

**Approved by the user — do not re-litigate:**

1. **Parameters stay symbolic in Section III.** No numeric values ($N=8$,
   $\alpha=0.10$, $q_e=10$, thresholds) in the technical section. The few a
   reader needs (cruise speed, $\omega_{\max}$) go in one line of Section IV;
   the full table goes to the supplementary doc. This is what P-AgNav does, and
   parameter tables were the single largest source of bloat in the old draft.
2. **The rear-camera headland leg gets one sentence.** III-E states that the
   clearing leg is steered from the rear view and that the conversion "is not a
   sign flip", and defers the derivation to supplementary material. Do not
   expand this into a subsection — it consumed enormous space in the old draft
   and no reviewer will question it at this length.
3. **The leaky evidence accumulator is a contribution**, with equation
   `eq:accumulator` and ~150 words in III-D. Keep it.

**Editorial rules (encoded in `.claude/skills/icra-paper/SKILL.md`):**

- **Cut, don't compress.** Over budget means delete a subsection, not squeeze
  every paragraph into unreadable density.
- **Nothing is deleted, only moved** to `AIAgNav_Supplementary.md` (not yet
  created — task 9).
- **Outline before prose** for any section over ~400 words: propose headings
  with word allocations and surviving equations, get approval, then write.
- **One section per session**, then `/clear`. The full draft plus the codebase
  plus the example paper will not co-exist in context at usable fidelity.
- **Equations carry the content; prose is connective tissue.**
- **Out of the paper:** design history, "we first tried X", parameter tables,
  sign-convention debugging, ROS topics/launch args/file names/class names,
  tutorials on standard material (what a ViT is, what MPC is).

**Technical framings established in Section III** (keep consistent elsewhere):

- Why DINOv3: **label efficiency**, not benchmark accuracy. Annotations must be
  hand-made from footage in the target field; the set is small.
- Why ViT-S/16: smallest variant, one set of weights must serve a GPU robot and
  a CPU-only robot.
- Why a **sky** class when the controller never reads it: sky is bright and
  untextured and a two-class model assigns it to traversable, which reproduces
  the exact corridor-widening signature of a row end at arbitrary points inside
  a row. One output channel buys the removal of that failure mode.
- Why the obstacle class is deliberately coarse: stalks, fallen plants, and
  foreign objects all induce the identical control decision.
- Why the corridor scan is anchored at the image centre column: when a gap
  exposes the neighbouring row the mask has two open regions, and the one the
  robot is in is the one aligned with its heading.
- Why $\alpha,\beta$ are empirical: a $360^\circ$ range view maps angular
  velocity to image displacement in closed form; a perspective camera cannot,
  because the mapping depends on unobserved depth.
- Why $r_\Delta$ instead of an EKF: smoothing falls out of the cost function,
  so no IMU and no filter state to re-initialize on pause.
- Why control-period scaling: the same controller runs at ~2 Hz and ~24 Hz;
  a per-step model is wrong on both unless the step is fixed.
- Stated contrast with P-AgNav: they couple $v$ to curvature
  ($v=\min(c/|\omega|,v_{\max})$); AIAgNav holds $v$ at constant cruise and
  slows only on the headland.

---

## 4. FILES

**The paper (all under `/home/chien21/agbot_control_ws/src/paper/`)**

| Path | What it is |
|---|---|
| `paper.tex` | **The deliverable.** IEEEtran `conference` class. Section III complete; II, IV, V, intro are `TODO` stubs. Per-section word budgets are in the comments. |
| `Makefile` | `make` build, `make pages` page count, `make budget` word check, `make words` texcount, `make watch` continuous, `make clean`. |
| `budget.py` | Per-section word count vs. budget. Parses `paper.tex` directly — no TeX, no texcount needed. Strips comments, math, and float environments. |
| `refs.bib` | 5 verified entries (P-AgNav, P-AgBot, P-AgSLAM, Agronav, CropFollow). DINOv3 and lightly-train still missing — a TODO comment says so. |
| `OUTLINE_III.md` | The approved Section III outline, equation inventory, and Table I draft. Still the reference for what Section III contains. |
| `pagecheck.sh` | PostToolUse hook script: rebuilds and reports page count when `paper.tex` is edited. Parses the hook payload with `python3` (no `jq` on this box). |
| `.gitignore` | LaTeX build artifacts + `paper.pdf`. |
| `fig/` | Empty. Figures go here (task 10). |

**Configuration**

| Path | What it is |
|---|---|
| `.claude/skills/icra-paper/SKILL.md` | `/icra-paper` — budget table, working rules, keep/cut lists, style. **Load this before writing any paper prose.** |
| `.claude/settings.json` | The page-count hook, `PostToolUse` on `Write|Edit`. **Not yet live** — see gotchas. |

**Sources of truth for content**

| Path | What it is |
|---|---|
| `AIAgNav_Technical.md` | The 19,098-word design doc Section III was cut from. Sections II (2,274 w) and the appendices (3,784 w) are still the raw material for tasks 8 and 9. |
| `4_P-AgNav_Range_View-Based_Autonomous_Navigation_System_for_Cornfields.md` | The style, density, and structure target. An exactly-8-page paper. Read its Section III before writing anything. |
| `Wei-Wei MSRAL Summer Research Report.md` | The author's own framing and reflection. Material for the introduction. |
| `Papers/` | P-AgBot, P-AgSLAM, Agronav, ROW-SLAM, CropFollow. |
| `CLAUDE.md` | Authoritative system description and design rationale. |
| `HANDOFF*.md` | Design history. Read for facts; do not import the narrative. |

**Code — ground truth for the equations** (`agbot_vision_nav/src/agbot_vision_nav/`)

| File | Carries |
|---|---|
| `centerline_estimator.py` | `estimate_centerline()` → `eq:midpoint`, `eq:offset`, `eq:slope`. Scan rows, boundary search from `c_x`, weighted pooling, validity gate. |
| `controller.py` | `MPCRowController` → `eq:mpc_dynamics`, `eq:mpc_cost`, `eq:mpc_constraints`. The class docstring states the model and cost explicitly. |
| `row_exit_detector.py` | `eq:accumulator`. The module docstring (lines 1–69) explains both signatures and every debounce decision with the field incident that motivated it. |
| `mission_fsm.py` | III-E state machine, 906 lines. |
| `segmentation_model.py` | Inference contract, nearest-neighbour resize rationale. |
| `../../segmentation/Train.py` | Training config: `dinov3/vits16-eomt`, 2,500 steps, batch 2, 224×224, 16-mixed, classes `{0: sky, 1: traversable, 2: obstacle}`. |

---

## 5. NEXT STEPS

Tasks 1–13 are done except Section IV. Remaining, in priority order:

**A. Section IV Experimental Results (~1,003 w).** Deferred by the user on
2026-08-13 — write it when field data exists. What is on this machine today is
10 **simulation** runs in `~/agbot_logs` (2026-08-06/07, ≈144 m total,
0 interventions, so MDBI is only a `>=` bound) plus perception mIoU 0.8717 from
Supplementary S6.2. No field CSV exists; old field runs predate the metrics
logger and would have to come from rosbags via `scripts/bag_distance.py`.
The abstract carries a `\TODO` for the headline number.

**B. Swap in the real segmentation-overlay figure** (`fig:overlay` in III-B) once
the user captures a debug-overlay frame into `paper/fig/`. The placeholder
reserves 3.6 cm so the page count is already honest.

**C. Related work.** Six references is thin for a submission. Adding ~12 more
costs roughly half a page that the current layout does not have — plan to trim
Section II or the introduction's third and fourth paragraphs when they go in.

**D. Author block and funding line** are still `Author One, Author Two` and
"supported by ...".

Historical task list (all now complete except as noted above):

**8. Cut Section II System Overview to ~989 words** (from 2,274).
Three parts: hardware; why a monocular camera and a learned mask as the
representation (the AIAgNav analogue of P-AgNav's §II-B range-view argument —
this is the section that must justify the whole approach); field assumptions
and the pipeline figure. **Partially blocked** — see the open questions below.
The representation argument and field assumptions can be written now.

**9. Create `AIAgNav_Supplementary.md`.** Move — do not delete — the design
history, parameter tables, sign conventions, the rear-camera `rear_to_front_state`
derivation and the $\kappa$ gain, and ROS specifics out of
`AIAgNav_Technical.md`. Section III-E already promises this document exists.

**10. Produce figures** into `paper/fig/`. Budget ~1.5 pages:
pipeline block diagram; segmentation mask panel with scan rows, corridor
bounds, and midpoints overlaid (`debug_viz.py` already renders this — capture
from a real run); FSM state diagram for III-E; a metrics plot from
`scripts/analyze_run.py` CSVs. Add `\ref{}`s to III-E and II when they exist.

**11. Draft Section IV Experimental Results (~1,003 w).** Needs a decision on
which runs to quote. Report in the units the field uses — distance, collisions,
interventions, **MDBI (meters between interventions)** — never `offset_norm`,
which is image space and mount-dependent. `scripts/analyze_run.py` produces the
Autonomy section; `scripts/bag_distance.py` recovers distance from old rosbags.

**12. Draft Abstract + Section I Introduction (~1,344 w).** Write last, once III
is settled. End with an explicit three-item contribution list, as P-AgNav does.

**13. Section V Conclusion (~247 w) + finish `refs.bib`.** Add real DINOv3 and
lightly-train entries. **Delete the `\nocite{*}`** once the body cites things.

**14. Final pass to 8 pages.** `make pages` must report ≤ 8 with figures and
full references in place.

**Questions answered on 2026-08-13 — do not re-ask:**

- **Training set size: 443 annotated frames**, 80/20 split, mIoU 0.8717. The
  number was in the design doc's own Appendix A.2 (now Supplementary S6.2), not
  in `Train.py`. III-A states it and the `\TODO` is gone.
- **Platform: both Jackals stay in the paper.** The 12x control-rate spread
  between the CPU-only and GPU robots is what motivates the metric/time
  parameterization in III-C and III-D, so it earns its space.
- **Results: deferred**, see next steps A. **Overlay figure: placeholder**, see B.

---

## 6. GOTCHAS

**Build traps — both already cost a debugging cycle:**

1. **An empty bibliography is a FATAL error, not a warning.** `\bibliography{refs}`
   with zero `\cite` commands aborts the build with a confusing
   "Something's wrong--perhaps a missing \item" at `\end{thebibliography}`.
   Fixed with `\nocite{*}` in `paper.tex` (which also keeps the reference list's
   page cost visible while drafting). **Remove it in task 13.**
2. **`latexmk -C` leaves `paper.bbl` behind.** A stale empty `.bbl` is re-read
   before bibtex can regenerate it and aborts the build. `make clean` now
   removes it unconditionally. If a build fails inexplicably, `rm -f paper.bbl`
   first.

**Environment:**

- **No passwordless sudo.** Package installs must be run by the user. Suggest
  `! sudo apt install ...` in the session (the `!` prefix puts output in context).
- **No `jq` on this machine.** `pagecheck.sh` uses `python3` to parse hook JSON.
  Do not rewrite it with `jq`.
- **This dev sandbox has ROS2 Humble, not ROS1.** Every `catkin build`,
  `roslaunch`, `rosbag play`, and any run of the actual robot code must be done
  by the user on their ROS1 Noetic machine. `lightly_train`/`torch` are not
  installed here either — `segmentation_model.py` cannot be executed or tested
  in this sandbox.
- System `matplotlib` is 3.1.2 on Python 3.8. Use `~/agbot_venv` for figures.

**The page-count hook is written and tested but NOT LIVE.** `.claude/settings.json`
did not exist when the session started, so the settings watcher never picked it
up; an edit to `paper.tex` produced no hook output. The script and the JSON are
both correct and verified. **Fix: open `/hooks` once, or restart Claude Code.**
Until then, run `make pages` manually after edits.

**Do not repeat these:**

- Do not ask for "everything from every md file" in a prompt. That instruction
  is what produced the 63-page draft — it was a prompt failure, not a model
  failure.
- Do not try to hit the budget by uniformly compressing prose. Delete whole
  topics instead.
- Do not put ROS specifics (topic names, launch args, `params.yaml` keys, class
  names) into the paper. A reviewer cannot run the code.
- Do not quote `offset_norm` as a performance number anywhere. It is normalized
  image space, not meters, and it shifts with camera mount height.
- Do not hand-write bibliographic data. `refs.bib` entries were transcribed from
  P-AgNav's reference list; the missing DINOv3 entry is deliberately left as a
  TODO rather than guessed.
- `--` is illegal inside an XML comment (it broke `vision_nav.launch` once).
  Irrelevant in `.tex`, but this repo writes `--` as an em-dash in prose
  everywhere, so watch it if you touch launch or xacro files.

**Commands:**

```bash
# Build and check the only limit that binds
cd ~/agbot_control_ws/src/paper
make              # build paper.pdf
make pages        # page count; exits nonzero past 8
make budget       # per-section words vs. budget (works without TeX)
make words        # texcount second opinion
make clean        # full clean, including the stale .bbl trap

# Load the paper's rules and budget before writing prose
/icra-paper

# Repo unit tests (no ROS, no lightly_train needed) — expected: 199 passed
cd ~/agbot_control_ws/src/agbot_vision_nav
PYTHONPATH=src python3 -m pytest test/ -v
```

**Git:** this repo has standing authorization to commit and push after each
meaningful unit of work. Stage files by name, never `git add .`. Present-tense
imperative messages. Section III was written one subsection per commit
(`c191789`, `480975a`, `d57e6d3`, `245d100`) — keep that granularity.
