# Stage 6 (started): the Chair44 as a three-dimensional substrate

Labels as in `PLAN.md`. Files: `t3_escape.py` (E2.5) and `t3_chair_handedness.py` (4 tests, about 8 s). Background: `CONNECTIONS_SPECTRE.md` §1.1.

```
python3 research/toymodel_v3/t3_escape.py --tests-only
python3 research/toymodel_v3/t3_chair_handedness.py --tests-only --story
```

| id | statement | label |
|---|---|---|
| E2.5 | The walk on the Chair44 tile graph (depth-5 supertile, 32768 tiles) is **transient**: h ∝ 1/r, flat to 5% for r = 5 to 18. Rule (C) gives the clock and a horizon on it | Exact / Numeric |
| E6a.1 | The bare supertile equals its x ↔ y mirror image, cell by cell (k = 0 to 3): every pseudoscalar of the bare geometry vanishes, as proved in the Hat and Spectre repository | Exact |
| E6a.2 | On the reference chair the mirror maps the decoration to its bump/dent flip, feature by feature (192 of 192) | Exact |
| E6a.3 | The decoration's height-weighted dipole is 0, but its axial moment `A = Σ h u × n = (−93/14, −93/14, 114/7)`. With the chair's own polar vector g (its centroid minus the centre of its bounding cube, `−(1/14)(1, 1, 1)`), **ψ = A·g = −3/14**. ψ is odd under the mirror and under the bump/dent swap separately, unchanged under both together and under all 24 rotations | Exact |
| E6a.4 | On the level-k supertile ψ adds up tile by tile: χ_k = N ψ = −12/7, −96/7, −768/7 for k = 1, 2, 3, with **one sign at every level** | Exact |

## What it says

The obstruction "the chiral substrate is recurrent, the transient one is achiral" holds for the bare geometry and is lifted for the decorated tile.

- The decorated Chair44 is chiral in the PC sense: its mirror image is its bump/dent-swapped image, and ψ tells the two apart.
- A 3D substrate can therefore fix the sign of the area term with a fixed-sign handedness, but only through its decoration.
  - Swapping every bump and dent reverses the arrow.
  - A mirror image of the world reverses it too, so their combination keeps it.
- That the decoration is physical (a charge-like label) and that the arrow couples to ψ are **Hypotheses**.

Still to do: isotropy of the Chair44 graph metric (the analogue of the Spectre's 1.50); rule (C) and the optics of `t3_optics.py` on the Chair44 graph together.
