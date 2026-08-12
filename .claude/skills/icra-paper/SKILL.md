---
name: icra-paper
description: Rules and word budget for revising the AIAgNav ICRA 2027 submission. Use whenever writing, cutting, or reviewing paper/paper.tex or AIAgNav_Technical.md — anything aimed at the 8-page conference submission rather than at internal documentation.
---

# AIAgNav → ICRA 2027

Target: **8 IEEE two-column pages including references.** Hard limit.

The source of truth for the paper is `paper/paper.tex`. `AIAgNav_Technical.md`
is the design-doc baseline it is cut down *from*, not the paper itself, and it
is ~19,000 words — roughly 2.6× an entire conference paper.

## The budget

Measured from `4_P-AgNav_Range_View-Based_Autonomous_Navigation_System_for_Cornfields.md`,
which is exactly eight pages. These are the numbers to write against:

| Section | Budget | Draft today |
|---|---:|---:|
| Abstract + I. Introduction | 1,344 | 378 |
| II. System Overview | 989 | 2,274 |
| III. System Design | **2,831** | **11,812** |
| IV. Experimental Results | 1,003 | 302 |
| V. Conclusion | 247 | 120 |
| References | 958 | — |
| **Total** | **7,372** | 19,098 |

Suggested split inside Section III: segmentation ~700, centerline/row-width
~600, MPC ~800, exit detection ~400, mission FSM ~330.

Check with `cd paper && make budget` (works without TeX) and, once TeX is
installed, `make pages` — the page count is the real limit; words only predict
it.

## Working rules

1. **One section per session.** Revise, build, commit, then `/clear`. The full
   draft plus the codebase plus the example paper will not fit in context at
   usable fidelity.
2. **Outline before prose.** For any section over ~400 words, propose
   subsection headings with a word allocation each and the list of surviving
   equations, and get approval before writing. This is what prevents another
   over-long draft.
3. **Cut, don't compress.** When over budget, delete a subsection. Uniformly
   squeezing every paragraph produces dense unreadable text that is still over.
4. **Nothing is deleted, only moved.** Everything cut goes to
   `AIAgNav_Supplementary.md`, which survives for the thesis and for reviewer
   questions.

## What belongs in the paper

- Numbered equations, and the minimum prose to define every symbol in them.
  Equations carry the content; prose is connective tissue.
- A notation table (P-AgNav Table I).
- One paragraph per subsystem: what it does, and the single design choice a
  reviewer would challenge.
- Figures. They are the space *savings*, not overhead — P-AgNav's pipeline
  figure replaces about a page of prose. Budget ~2 pages of figures.
- Quantitative results in the units the field reports: distance, collisions,
  interventions, MDBI. Not `offset_norm`, which is image space and
  mount-dependent.

## What does not

- Design history, alternatives tried, "we first attempted X." This is the
  single largest source of excess length in the current draft.
- Parameter tables, default values, tuning knobs.
- Sign conventions, debugging narratives, coordinate-frame corrections.
- ROS specifics: topics, launch arguments, file paths, class or function names.
  A reviewer cannot run the code and does not care what the file is called.
- Tutorials on standard material — what a ViT is, what MPC is, what DINOv3 is
  in general. Cite and move on. Explain only what is *particular* to this
  system: why this backbone, why three classes, why this cost function.

## Style

Match P-AgNav: present tense, "the system"/"the robot" as subject, short
declarative sentences, no hedging, no bullet lists in the body (only in the
contribution list at the end of the introduction). Every claim either is
supported by a number in Section IV or is stated as a design assumption.

## Reference material

- `4_P-AgNav_...md` — style, density, and structure target.
- `Papers/` — P-AgBot, P-AgSLAM, Agronav, ROW-SLAM, CropFollow.
- `Wei-Wei MSRAL Summer Research Report.md` — the author's own framing and
  reflection; useful for the introduction and for what mattered in the field.
- `HANDOFF*.md`, `CLAUDE.md` — design history. Read for facts; do not import
  the narrative into the paper.
