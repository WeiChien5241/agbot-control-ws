# Technical Doc Creation — AIAgNav ICRA 2027 paper

Handoff updated 2026-08-13 (second session). Head commit at time of writing:
`ddb60dd`. Read this file plus `paper/OUTLINE_III.md` and you have everything.

**Session 2 in one paragraph:** Section II was written, the introduction,
abstract and conclusion were written, the design doc was split into
`AIAgNav_Technical.md` + `AIAgNav_Supplementary.md`, and two TikZ figures were
built. The paper is at 6/8 pages with every section drafted except IV, which the
user deferred until field data exists. Commits: `9e243b0`, `63f3d01`, `d10aea7`,
`17b1e09`, `ddb60dd`.

---

## 1. GOAL

Turn the design doc (originally `AIAgNav_Technical.md`, 19,098 words, ~63 pages
— an excellent design doc, not a paper; now split into a 13.3k-word technical
doc and a 6.4k-word supplementary) into an **8-page IEEE conference paper**
for ICRA 2027, formatted
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
| III-A Semantic Segmentation | ~620 | 700 |
| III-B Corridor and Centerline Estimation | ~560 | 600 |
| III-C Model Predictive Row Following | 630 | 800 |
| III-D Row Exit Detection | 450 | 400 |
| III-E Row Switching | 313 | 331 |
| **Section III total** | **2,528** | **2,831** |

(III-A and III-B each grew slightly: III-A when the training-set count replaced
the `\TODO`, III-B when the overlay figure got its `\ref`. Summing subsections
does not exactly match `make budget` — the two counters strip markup
differently, and `make budget` is the authority.)

Seven numbered equations, all labelled: `eq:midpoint`, `eq:offset`, `eq:slope`,
`eq:mpc_dynamics`, `eq:mpc_cost`, `eq:mpc_constraints`, `eq:accumulator`.
Section labels: `sec:centerline`, `sec:mpc`, `sec:exit`, `sec:switching`.
Notation table `tab:notation` is filled and matches the symbols actually used.

**The word budget, measured from the P-AgNav paper (exactly 8 pages):**

| Section | Budget | Status |
|---|---:|---|
| Abstract + I. Introduction | 1,344 | written (1,034) |
| II. System Overview | 989 | written (1,005) |
| III. System Design | 2,831 | complete (2,528) |
| IV. Experimental Results | 1,003 | deferred |
| V. Conclusion | 247 | written (256) |
| References | 958 | 6 entries in `refs.bib`, all cited |
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
4. **Section IV is deferred until field data exists** (2026-08-13). Do not
   fabricate results and do not quietly fill it from the simulation CSVs.
5. **Both robots stay in the paper.** The 12x control-rate spread is the reason
   III-C scales to the control period and III-D counts in metres and seconds;
   removing one platform removes the motivation for two design decisions.
6. **The III-B overlay figure stays a placeholder** until the user supplies a
   `debug_viz.py` capture. It reserves 3.6 cm so the page count stays honest.

**Framings established in Sections I and II** (keep consistent if either is
rewritten):

- The lab's lineage is stated as **solved**, not as a gap: P-AgBot (2D LiDAR
  balancing) → P-AgSLAM (3D LiDAR state estimate) → P-AgNav (range view, full
  multi-row navigation). AIAgNav is a **sensor-modality** contribution on top of
  a solved navigation problem. Do not claim the problem was open.
- The gap is **semantic, not geometric**: a range return cannot separate a rigid
  stalk from a leaf hanging into the corridor. Cost is the secondary argument,
  never the lead.
- Related work is two-pronged: CropFollow puts the whole navigation state inside
  a learned regressor and needs a filter; Agronav is segmentation + semantic
  line but is aimed above canopy / at field roads. Neither does the full
  multi-row mission — that framing is what earns III-E its space.
- Section II-A states the **mirrored camera pair** as load-bearing (the rear
  conversion assumes geometric twinning) and states that width thresholds are a
  property of the camera pose, not of the field.
- Section II-B keeps **one unabsorbed failure mode on the record**: a leaf on
  the lens and a crop wall ahead produce the same mask. Reviewers reward this;
  do not delete it to save words.
- The conclusion's three future directions are RTK for above-canopy transit
  only, integration with the sampling module, and using the sky class to
  separate an occluded lens from a genuine obstruction.

**Editorial rules (encoded in `.claude/skills/icra-paper/SKILL.md`):**

- **Cut, don't compress.** Over budget means delete a subsection, not squeeze
  every paragraph into unreadable density.
- **Nothing is deleted, only moved** to `AIAgNav_Supplementary.md`, which now
  exists (S1 signs, S2 tuning, S3 rear-leg derivation, S4 runtime, S5 parameters,
  S6 design history).
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
| `paper.tex` | **The deliverable.** IEEEtran `conference` class. Everything drafted except Section IV. Per-section word budgets are in the comments. |
| `Makefile` | `make` build, `make pages` page count, `make budget` word check, `make words` texcount, `make watch` continuous, `make clean`. |
| `budget.py` | Per-section word count vs. budget. Parses `paper.tex` directly — no TeX, no texcount needed. Strips comments, math, and float environments. |
| `refs.bib` | 6 entries, **all cited** in the body: P-AgNav, P-AgBot, P-AgSLAM, Agronav, CropFollow, DINOv3 (`simeoni2025dinov3`, arXiv:2508.10104, verified by web search rather than guessed). No `lightly-train` entry: the training framework is a tooling detail and the paper never names it. |
| `OUTLINE_III.md` | The approved Section III outline, equation inventory, and Table I draft. Still the reference for what Section III contains. |
| `pagecheck.sh` | PostToolUse hook script: rebuilds and reports page count when `paper.tex` is edited. Parses the hook payload with `python3` (no `jq` on this box). |
| `.gitignore` | LaTeX build artifacts + `paper.pdf`. |
| `fig/pipeline.tex` | Fig. 1, TikZ block diagram, `\input` from `paper.tex`. Two cameras → single-slot buffer → segmentation → corridor scan → {detector, MPC} → FSM → command, with odometry entering from the left and a "camera select" feedback edge. |
| `fig/fsm.tex` | Fig. 3 (III-E), TikZ mission state machine: nominal chain on top, blocked-row chain below, `DONE` reached on the last row. |
| (Fig. 2, in III-B) | A `\framebox` placeholder **inside `paper.tex`**, not a file. Replace with the real overlay capture. |

**Configuration**

| Path | What it is |
|---|---|
| `.claude/skills/icra-paper/SKILL.md` | `/icra-paper` — budget table, working rules, keep/cut lists, style. **Load this before writing any paper prose.** |
| `.claude/settings.json` | The page-count hook, `PostToolUse` on `Write|Edit`. **Not yet live** — see gotchas. |

**Sources of truth for content**

| Path | What it is |
|---|---|
| `AIAgNav_Technical.md` | The design doc the paper was cut from, now 13.3k words: history, parameters, sign conventions, the rear-leg derivation and the runtime section were moved out with pointers left in place. Still holds §I–§V, the notation table, and §III-A…§III-F/H in full. |
| `AIAgNav_Supplementary.md` | **New (2026-08-13).** 6.4k words. S1 sign convention + reference-frame resets (was §III-C.7); S2 tuned values and tuning order (§III-C.9); S3 the rear-steered headland leg incl. the $\kappa$ derivation (§III-E.4) — this is what the paper's III-E defers to; S4 runtime architecture and measured timing (§III-G); S5 parameter reference (was Appendix B); S6 design history (was Appendix A). Cross-references were rewritten everywhere: "Appendix A.4" now reads "S6.4". |
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

**E. Final pass to 8 pages.** `make pages` must report ≤ 8 with the real
overlay figure, Section IV and the full reference list in place. This is the
last task, not an ongoing one.

Historical task list — what was done and where it landed:

**1–7 (session 1).** Scaffolding, budget tooling, and all of Section III.

**8. Section II System Overview** — done, 1,005 w, commit `9e243b0`. Three
subsections: *System Hardware* (platform, mirrored camera pair, deck mount and
its effect on corridor width, the two compute variants, the three uses of
odometry); *Visual Representation* (the AIAgNav analogue of P-AgNav §II-B — the
stalk-versus-leaf argument, cost, multi-depth single frame, then the three
absorbed costs and the one that is not); *Framework Overview* (field
assumptions, five operational stages, the pipeline figure, no state estimator,
shared perception). Also resolved the III-A `\TODO` to 443 frames in the same
commit.

**9. `AIAgNav_Supplementary.md`** — done, commit `63f3d01`. Built by a script
that extracts verbatim line ranges from `AIAgNav_Technical.md`, writes them into
the new file under S1–S6 headings, and replaces each range in place with a
3–6 line pointer stub. Word totals before/after (19,098 → 13,330 + 6,356)
confirm nothing was dropped; the surplus is the stub text. The script is
disposable and was not committed.

**10. Figures** — two of four done, commit `d10aea7`. `fig/pipeline.tex` and
`fig/fsm.tex` are hand-written TikZ, `\input` from `paper.tex`, and both were
checked by rasterizing the built PDF (see gotchas). The segmentation-overlay
panel is a placeholder (task B). The metrics plot was dropped with Section IV.

**11. Section IV** — deferred, see task A.

**12. Abstract + Introduction** — done, commit `17b1e09`, 177 + 857 w. Written
against the outline that was already in `AIAgNav_Technical.md` §I, which is why
no separate outline approval round was needed. Seven paragraphs: problem,
lineage, the semantic gap, camera-based related work, what AIAgNav is, the
three contributions as an `itemize`, and the GNSS scope note.

**13. Conclusion + `refs.bib`** — done, same commit. 256 w, three future
directions. DINOv3 entry added after verifying it by web search;
**`\nocite{*}` has been removed** and all six entries are cited in the body.

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
   This was handled with `\nocite{*}`, which has now been **removed** because the
   body cites all six entries. If you ever strip the citations again, put it back.
2. **`latexmk -C` leaves `paper.bbl` behind.** A stale empty `.bbl` is re-read
   before bibtex can regenerate it and aborts the build. `make clean` now
   removes it unconditionally. If a build fails inexplicably, `rm -f paper.bbl`
   first.
3. **The Makefile does not know about `fig/*.tex`.** `$(PDF)` depends only on
   `paper.tex` and `refs.bib`, and latexmk additionally skips a rebuild when
   `paper.tex` is unchanged by content (touching it is not enough — latexmk
   hashes). **After editing a figure file, `rm -f paper.pdf && make`.** Half an
   hour was lost to reviewing a stale PDF and "fixing" a figure that was already
   correct.
4. **A new `\cite` needs two `make` runs** (bibtex, then the reference
   resolution). `grep -c undefined paper.log` is the check.

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
- **TikZ is available** (`/usr/share/texlive/texmf-dist/tex/latex/pgf`), so
  diagrams are written as `.tex` and need no external tool. `paper.tex` loads
  `arrows.meta,positioning,calc`.
- **There is no `pdftoppm` and no ImageMagick**, but **ghostscript and PIL are
  installed**. To eyeball a figure, rasterize the built PDF and crop:
  ```bash
  gs -q -dNOPAUSE -dBATCH -sDEVICE=png16m -r280 \
     -dFirstPage=5 -dLastPage=5 -sOutputFile=/tmp/pg5.png paper.pdf
  python3 -c "from PIL import Image; im=Image.open('/tmp/pg5.png'); w,h=im.size; \
              im.crop((int(w*.05),int(h*.03),int(w*.55),int(h*.14))).save('/tmp/crop.png')"
  ```
  Both TikZ figures had label collisions that only this caught — a clean build
  says nothing about whether a diagram is readable.

**The page-count hook is still NOT FIRING.** `.claude/settings.json` exists and
is correct (`PostToolUse` on `Write|Edit` running `paper/pagecheck.sh`), and the
script is verified, but no hook output appeared for any `paper.tex` edit in
session 1 or session 2. **Fix: open `/hooks` once, or restart Claude Code**, and
check that it actually fires before relying on it. Until then, `make pages`
manually after edits — that is what both sessions did.

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
  P-AgNav's reference list; the DINOv3 entry was added only after checking it
  with a web search. Do the same for the ~12 related-work entries still to come.
- Do not treat the open questions in this file as unanswerable without the user.
  The 2026-08-12 handoff listed the training-set size as blocked; the number
  (443 frames, mIoU 0.8717) was sitting in the design doc's own Appendix A.2.
  **Search the repo's own documents before asking.**
- Do not describe the sim runs in `~/agbot_logs` as field results. They are
  Gazebo, 0 interventions, and MDBI from them is a `>=` bound with no
  denominator.
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
(`c191789`, `480975a`, `d57e6d3`, `245d100`) — keep that granularity. Session 2
kept it too: `9e243b0` Section II, `63f3d01` the supplementary split, `d10aea7`
the figures, `17b1e09` intro/abstract/conclusion/DINOv3, `ddb60dd` this handoff.

**To resume cold, in order:**

```bash
cd ~/agbot_control_ws/src/paper
make && make pages && make budget     # expect 6/8 pages, 0 undefined refs
```

Then read §2 (state), §3 (decisions and framings — these are what keep a
rewrite consistent), and pick up at §5 task A or B. Load `/icra-paper` before
writing any prose. Nothing in the paper is half-written: every section either
reads as final or is the single word `TODO` (Section IV) — there are no
partial paragraphs to reconstruct.
