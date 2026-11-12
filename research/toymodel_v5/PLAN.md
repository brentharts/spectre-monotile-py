# toymodel_v5: plan (revision 6, after Stage 5)

Status: Stages 0 to 5 done (`STAGE0.md` to `STAGE5.md`). Labels as in `quantgrav.tex` and `toymodel_v2/PLAN.md`: **Exact**, **Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**.
Framing as in the paper: nothing here is gravity or the standard model. "Letter", "source", "family" and "arrow" name parts of small models and are used for a result only when a test shows the part behaves that way.

## 0. Why a v5

v1 and v2 built a timeline model (six directions, retrocausal contracts, a wave sheet) whose alphabet had to be argued for: v2 chose the six roots of an A2 over the cubic six. Since then the repository gained a way to mix several Spectre tiles with Chair44 (`models/chair_fibers.py`, `models/chair_fibers_14th.py`): three full tiles, three vertex-only Hat tiles and four groups of the six points around the 14th vertex use the 192 pyramid tips exactly (3·42 + 3·14 + 4·6 = 192). v5 rebuilds the v2 model on that ensemble, so that the alphabet, the sources and the layers come from the geometry instead of being chosen.

## 1. Dictionary from v1/v2 to the ensemble (Definitions, to be tested, not assumed)

| v1/v2 | v5 |
|---|---|
| six letters of a timeline | the six points around P8 (P7, P9 and the four handles of the edges 7 and 8) |
| letter and its inverse (the entangled origin pair v, -v) | antipodal pair about P8 (exact for the convex bit, `chair_fibers_14th.antipodes`) |
| hexagonal alphabet (A2) | the six points at k = sqrt3 a / 2 (Stage 0, E5.1) |
| cubic alphabet | the six sides of the cube; its shadow along a body diagonal is the hexagon (E5.2) |
| smooth sheet S, density rho | the three curved Spectre tiles (above the chair) |
| discrete timelines D | the 14th-vertex groups and their bit |
| flat limit, no bit | the Hat vertex tiles (below the chair) |
| pair snap = dipole, no net source | the bump and dent of one depth (Chair44 has monopole 0 and dipole 0 exactly) |

## 2. Stages

| stage | what | size | fails if |
|---|---|---|---|
| 0 | the alphabet as a function of k; the hexagon as the shadow of the cube; the chair's three-fold axis (done) | S | -- |
| 1 | v2 Stage 4 (history weight, CHSH, no-signalling) on the actual six points: unequal letters for k != sqrt3/2, the hexagon reproducing v2 (done: no-signalling stays exact; the curvature sets the strength; the cubic six are local) | M | the rule loses exact no-signalling away from the hexagon |
| 2 | Chair44 as a neutral source: multipoles of the 192 signed depths, symmetry, far field (done: monopole 0, dipole 0, a saddle quadrupole falling as r^-3, charges odd under x <-> y); the field on the v2 lattice is not done | M | the field is not what the multipoles predict |
| 3 | the ensemble as S + D: tiles as density, groups as bits, the Hat layer as the frozen end; couplings between layers (done: reversible with a stated flip term; Hat silent; no neutral pattern at tau_group = 0; bits cancel the xx - yy part of the chair's quadrupole only) | L | a coupling breaks reversibility or energy conservation without a stated term |
| 4 | the arrow audit of v2 Stage 3 on the ensemble: its five inputs plus the new binary data (layer order, the cyclic order of the pairs, which diagonal) (done: no new arrow; live side = layer order x flow direction, a dipole; chair's x <-> y oddness restored by bit conjugation; cyclic order invisible) | M | more than one input is needed and they disagree |
| 5 | dynamics and the clock (v2 Stage 5) with the chair as the source (done: **C7 fails for the chair alone**, r^-3 and sign-changing; the ensemble's unpaired live bit restores 1/r beyond r = 8.4, first order) | L | the C7 identity fails |

## 3. Hypotheses kept apart (nothing here tests them)

- H5.1. The chair's three-fold rotation permutes the three antipodal pairs of the six points (x -> y -> z); the "three families" reading of the 3 x 3 of the counting is a Hypothesis, not a result.
- H5.2. That the letters of the dynamics are the six points of the tile. Stage 0 shows the algebra; it does not make them letters.
- H5.3. That the tile plane lies perpendicular to the chair's axis. Exact for the 120 degree choice; a choice of sign convention otherwise.

## 4. Rules (as in v2)

Define then compute; a test that can fail in every stage; a Negative result is a deliverable; do not match what should be derived; single-file modules with `--tests-only`; every number in the paper a macro of `models/paper_numbers.py`; v1, v2, v4 stay as they are.

## 5. Layout

```
research/toymodel_v5/
  PLAN.md             this file
  t5_alphabet.py      Stage 0 (done)
  STAGE0.md
  t5_rule.py          Stage 1 (done)
  STAGE1.md
  t5_source.py        Stage 2 (done)
  STAGE2.md
  t5_ensemble.py      Stage 3 (done)
  STAGE3.md
  t5_arrow.py         Stage 4 (done)
  STAGE4.md
  t5_clock.py         Stage 5 (done)
  STAGE5.md
  figures/
```

```
python3 research/toymodel_v5/t5_alphabet.py --tests-only
python3 research/toymodel_v5/t5_alphabet.py --no-show --save research/toymodel_v5/figures/t5_stage0_alphabet.png
```
