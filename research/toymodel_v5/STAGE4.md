# Stage 4: results (the arrow audit on the ensemble)

Labels as in `PLAN.md`. Nothing here derives an arrow of time. File: `t5_arrow.py` (4 tests, about 3 s), figure `figures/t5_stage4_arrow.png`.

```
python3 research/toymodel_v5/t5_arrow.py --tests-only
python3 research/toymodel_v5/t5_arrow.py --no-show --save research/toymodel_v5/figures/t5_stage4_arrow.png
```

## 1. What was audited

The five inputs of `toymodel_v2/STAGE3.md` (pinned end, damping, orientation of tau, handedness, scale ordering) plus the binary data the ensemble adds: the order of the layers, the direction of the flow (which end is silent), the chair's chirality, the cyclic order of the six points.

## 2. Results

| id | statement | label |
|---|---|---|
| E5.20 | Exchange of x and y maps the chair's charges to their negatives and the four groups (+x, +y, -x, -y) to (+y, +x, -y, -x): `C P S(chair, s) = S(chair, -sigma s)` on all 16 bit patterns, exactly. The chair alone is not invariant, so the bits restore the symmetry. Mirroring the tile (y -> -y, cyclic order of the six points reversed) changes the CHSH value by 0.0 for k = 0, 1/2, sqrt3/2, 1 and three stiffnesses: the cyclic order is not an orientation of the rule | Exact |
| E5.21 | The live side is a dipole. Only the three full tiles are live, so p_z = 3 sqrt3 Z = 41.6 (Z = 8) when all bits agree and at least sqrt3 Z for any pattern (an odd count, as in E5.17). Exchanging the layers reverses p exactly. The Coulomb energy of the three live bits with the chair's charges is -0.187 for the tiles up and +0.076 mirrored down: the chair sees the layer order | Exact |
| E5.22 | Layer order and flow direction are one datum in the field: the dipole of the live bits is +, -, -, + over (up, down) x (flow, reversed flow). Only the product is observable | Exact |
| E5.23 | Damping on the ensemble: the damped leapfrog returns the data (6e-15) only if gamma is reversed with the time; with gamma kept the error is 3.9. At gamma = 0 it is 3e-15 | Exact (finite scheme) |

## 3. What this says

1. **The ensemble has no new arrow input.** The binary data it adds are: the live side (one bit, the product of layer order and flow direction, seen as the dipole), the chair's chirality (x <-> y odd, compensated by bit conjugation) and the cyclic order of the six points (invisible). None of them is a time orientation. The three arrow inputs of v2 (boundary data, damping, orientation of tau) are unchanged and independent.
2. **The live side orients space, not time.** The field of the ensemble points to the live layer, and a mirror in z with a reversed flow cannot be told from the original. This agrees with v2 Stage 3 item 4: the record and the flow orientation disagree as conventions, and here the second one is not even visible in the field alone.
3. **The mirror symmetry of the chair is restored by the bits, not by the chair.** x <-> y with all charges conjugated (chair and bits) is exact on the chair-plus-groups sector; the three full tiles are not invariant (their triangle is not).
4. **Damping stays a separate input.** It makes the dynamics irreversible regardless of the layers (E5.23).

## 4. Not done

The pin and the flux base on the ensemble's own timelines (the letters of Stage 1 are not yet placed on the sheet); scale ordering; a coupling of damping to the live side.
