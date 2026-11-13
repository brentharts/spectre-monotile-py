# Stage 3: results (the layout of the 192 tips, under rules stated in advance)

Labels as in `PLAN.md`. File: `t7_layout.py` (3 tests, under 1 s), figure `figures/t7_stage3_layout.png`.

```
python3 research/toymodel_v7/t7_layout.py --tests-only
python3 research/toymodel_v7/t7_layout.py --no-show --save research/toymodel_v7/figures/t7_stage3_layout.png
```

## 1. Counts and rules (written before the enumeration)

Point counts a tile supplies:
- **Spectre end:** 42 (curved: 14 vertices + 28 handles), 28 (handles only), 14 (vertices only), 6 (the group around P8);
- **Hat end:** 13 (P8 is gone at tau = 1, so 13 corners).

Live bits come from Spectre-end tiles (42, 28) and groups; the 14 and 13 types are silent (h = 0).

Rules:
- **R1:** the counts add to 192.
- **R2:** the chair's only rotation, the three-fold axis (1,1,1). A Spectre tile has no rotational symmetry, so every tile type's count is a multiple of 3. At most one group sits on each side, and the rotation permutes the sides in two orbits of three, so there are 0, 3 or 6 groups.
- **R3:** an even number of live bits (the pair rule).

## 2. Results

| id | statement | label |
|---|---|---|
| E7.9 | R1 has 126 solutions. v5's layout (3 x 42 + 3 x 14 + 4 x 6) is one of them and fails R2 (four groups break the three-fold axis) and R3 (seven live bits). With only v5's three types (42, 14, 6), no layout passes R2 | Exact, **Negative** for v5's layout |
| E7.10 | **R2 alone selects one layout: 12 x 13 + 6 x 6.** Twelve Hat tiles (four orbits of the rotation) and six groups, one on every side of the cube. It passes R3 by itself (6 live bits). Without the group clause of R2, 7 layouts remain | Exact |
| E7.11 | **The selected layout is neutral.** The six groups can be paired into bump-and-dent pairs compatibly with the rotation in 3 ways, one of them antipodal. Every such pairing has net bit source 0. With the bump on the + sides, the antipodal one has dipole 2 h R (1,1,1), along the chair's own axis | Exact; **Negative** for C7 |

## 3. What this says

1. **A rule selects the layout, and it is not v5's.** The chair's three-fold axis, plus "one group per side", leaves exactly one way to write 192 in the tile's own point counts. It contains no curved Spectre tile at all: the Spectre survives only as the six groups of points around its 14th vertex, one on each side of the cube, on a body of twelve Hat tiles. The rule that the pair rule needs (even live bits) then holds automatically.
2. **The layout that the rules select has no monopole.** v5 Stage 5 rescued the 1/r clock through an unpaired live bit; the selected layout pairs all six. Its far field is the chair's quadrupole plus a bit dipole along the chair's axis. So the gravity side of the model cannot take its 1/r from the layout, and has to get it elsewhere (Stage 6). The scoreboard gains this as an open row.
3. **What depends on a choice.** The 13-corner Hat type is what makes a solution exist; it was in the list before the count (from v4), but a different list of point counts could change the answer (H7.4). The rules R2 and R3 are written down in the module.

## 4. Not done

The 3D placement of the selected layout (twelve Hat tiles in four orbits around the chair, six groups on the sides) and its fibre pairing to the tips, as `models/chair_fibers_14th.py` did for v5's layout; the field of the selected layout on the sheet.
