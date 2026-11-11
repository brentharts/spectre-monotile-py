# Stage 6: the Chair44 as a three-dimensional substrate

Labels as in `PLAN.md`. Files: `t3_escape.py` (E2.5), `t3_chair_handedness.py` (4 tests, about 8 s) and `t3_chair_metric.py` (4 tests, about 40 s). Background: `CONNECTIONS_SPECTRE.md` §1.1.

```
python3 research/toymodel_v3/t3_escape.py --tests-only
python3 research/toymodel_v3/t3_chair_handedness.py --tests-only --story
python3 research/toymodel_v3/t3_chair_metric.py --tests-only --story
```

| id | statement | label |
|---|---|---|
| E2.5 | The walk on the Chair44 tile graph (depth-5 supertile, 32768 tiles) is **transient**: h ∝ 1/r, flat to 5% for r = 5 to 18. Rule (C) gives the clock and a horizon on it | Exact / Numeric |
| E6a.1 | The bare supertile equals its x ↔ y mirror image, cell by cell (k = 0 to 3): every pseudoscalar of the bare geometry vanishes, as proved in the Hat and Spectre repository | Exact |
| E6a.2 | On the reference chair the mirror maps the decoration to its bump/dent flip, feature by feature (192 of 192) | Exact |
| E6a.3 | The decoration's height-weighted dipole is 0, but its axial moment `A = Σ h u × n = (−93/14, −93/14, 114/7)`. With the chair's own polar vector g (its centroid minus the centre of its bounding cube, `−(1/14)(1, 1, 1)`), **ψ = A·g = −3/14**. ψ is odd under the mirror and under the bump/dent swap separately, unchanged under both together and under all 24 rotations | Exact |
| E6a.4 | On the level-k supertile ψ adds up tile by tile: χ_k = N ψ = −12/7, −96/7, −768/7 for k = 1, 2, 3, with **one sign at every level** | Exact |
| E6b.1 | The unrelaxed walk tensor `M = Σ_edges e eᵀ` (centroid to centroid) has equal diagonal entries at every level k = 1 to 5. Its off-diagonal entries are all −22/49 from k = 2 on, while tr M grows with the number of tiles, so **M / tr M → I/3** | Exact |
| E6b.2 | **The relaxed (homogenised) tensor is isotropic in the bulk.** Level 6, a cube of half-width 30 around the centre of a full octant: eigenvalues 4.0024, 4.0024, 4.0033 (relative spread 2×10⁻⁴), 4% below the unrelaxed value in every direction. *Warning:* measured around the supertile's centre, which is the inner corner of its missing octant, with the notch's faces free, the same procedure gives a tensor uniaxial along (1,1,1) with eigenvalue ratio 9. That is a boundary artefact | Numeric |
| E6b.3 | **The hop count is cubic.** Per unit length (bulk, level 6, r = 16 to 26): 0.554 on all 3 axes, 0.707–0.708 on all 6 face diagonals, 0.660–0.667 on all 4 body diagonals; spread 0.24. Controls: taxicab Z³, 0.5 / 0.707 / 0.866, spread 0.53; the Spectre, 0.06† over 24 angular bins (a coarser measure) | Numeric; Negative for an isotropic metric |
| E6b.4 | **The clock is isotropic at large r.** Bulk, source radius 3, absorbing radius 30: the excess of h along body diagonals over the axes is 2.2% at r = 6–10, 1.3% at r = 10–15 and −0.1% at r = 15–20, a short-range effect of the discrete source | Numeric |

## Two notions of distance (E6b)

The Chair44 has two notions of distance that disagree.

- **Diffusion and long waves see an isotropic medium.** The relaxed tensor, which sets both, is proportional to I to 2×10⁻⁴, and so the clock c² = 1 − h of rule (C) is the same in every direction at large r.
- **Hop count sees a cubic one.** A signal that moves one tile per tick would be about 25% faster along the diagonals than along the axes. Its unit ball is cube-like, rounder than Z³'s.
- **Consequence.** On the Chair44, light has to be a long wave, not a hop count, or the substrate has a preferred frame. The Spectre has no such split (both are isotropic to 0.06).
- **Optics.** Neither notion supplies a space factor B. Rule (C) with the optics of `t3_optics.py` gives the clock-only half of light bending (E1.2 with r_s the escape coefficient), on the Chair44 as on Z³. The combined item of the plan therefore reduces to the open question of B.

**A correction to the first version of this stage.** The first measurements (hop count and h by direction class) were made around the supertile's centre, the corner of its missing octant, and folded the signed directions together. That folding would have hidden the notch's uniaxial artefact. Redone in the bulk with each line kept apart, the hop-count values stand (body diagonals 0.66, not 0.69) and the clock is isotropic. E2.5 was also measured at the notch; in the bulk the coefficient is 1.96 to 2.10 over r = 6 to 25 (R = 3, L = 30), within its quoted 7% flatness.

Still to do: a reason for B = 1/A (Stage 1).
