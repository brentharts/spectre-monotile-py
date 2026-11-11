# Stage 2: results

Labels as in `PLAN.md`. Nothing here connects to gravity. File: `tm2_directed.py` (9 tests, about 2 s), figure `figures/tm2_stage2_directed.png`.

```
python3 research/toymodel_v2/tm2_directed.py --tests-only
python3 research/toymodel_v2/tm2_directed.py --story
python3 research/toymodel_v2/tm2_directed.py --no-show --save research/toymodel_v2/figures/tm2_stage2_directed.png
```

## 1. What was built

The open item of Stage 1 (the letter timeline and the bit are different objects) is closed by a **Definition**: the letter at tick j is `b_j u_j`, with `b_j` the chain's bit entering tick j and `u_j` one of the six hexagonal roots, prescribed per tick. The bit keeps the chain's amplitudes unchanged. The chain's scalar ledger `Q = sum b_j w_j` becomes the planar flux `Phi = sum b_j w_j u_j`, and "compatible with the reference" means same final bit and `|Phi - Phi_ref| <= delta`. With all `u_j` equal this is the chain's rule again.

This puts the bit in the hexagonal alphabet as the sign of a letter. It does not give the axis of the letter an amplitude: a history with six free letters per tick on top of the bit (6^n histories) is not defined here.

## 2. Results

| id | statement | label |
|---|---|---|
| E2.1 | **Regression.** With equal axes, `Phi` equals the chain's `Q` bit for bit; compatible sets, reach, backward-snap tables (toggles, relative probability, candidates), constrained amplitudes and the reversed flow equal `spectre_hat_backward_snap_model.py` to 1e-12, at n = 12 and n = 8 | Exact |
| E2.2 | Rotating all axes by a root, or negating them, changes nothing | Exact |
| E2.3 | The vector constraint cannot lower the amplitude cost of going back: the best compatible backward snap has relative probability at most the single toggle's `tan^2(theta_j/2)/tan^2(theta_k/2)`, with equality when the single toggle is compatible. Constraint off: 2^(n-1) compatible histories | Exact |
| E2.4 | The vector constraint is not the scalar one in disguise: at delta = 0.4 there are histories compatible in one sense and not the other (43 scalar, 70 for three-root axes, 711 for six-root axes) | Exact |
| E2.5 | Search cost: the exhaustive work stays 2^n for every axes sequence; the number of compatible histories grows and the reach moves earlier with delta | Exact |
| E2.6 | **Scale ladder** (C8): `c_j = c_0^((4+sqrt15)^j)` equals `exp(-(lam^j d_0)^2/8 sigma^2)` to 1e-12; 4 + sqrt15 is the largest eigenvalue of `spectre_patch.substitution_matrix()`; from c_0 = 0.872 (tau = 0.9): 0.872, 0.340, 2.0e-4. Valid under the stated assumption (sigma fixed, d scales with the tile) | Exact under an assumption (Definition D2.5) |
| E2.7 | Where no compensating toggles are needed, the direction asymmetry is the amplitude ratio: negative on the forward flow, positive on the reversed flow, for every axes sequence and delta; constraint off, the reversed flow gives exactly the mirrored value | Exact |
| N2.1 | **The binding vector constraint can reverse the sign of the net asymmetry.** Grid k in {4,6,8}, m in {1,2,3}, delta in {0.3,0.6,1.0}: structured axes (scalar, periods 2, 3, 6): none reversed in 108 cases per flow; 200 random axes: reversed in 18 of 994 forward and 145 of 572 reversed cases that need compensation, always at delta 0.3 or 0.6, never at 1.0. Below the grid the six-root axes also reverse it on the reversed flow (k = 6, m = 1: -16.1 at delta 0.2, -14.6 at 0.25, then +24.9) | Numeric (sample) |

## 3. What this says about the plan

1. **The regression holds, so the chain is a special case of the history-sum.** The two-state, scalar-flux model is exactly the six-root model with all axes equal, which is what the plan asked for. The chain's numbers are not altered by this.
2. **The three costs survive in different ways.** Amplitude: the constraint can only remove histories, so the cost of going back is never lower than the single toggle's (E2.3). Search: still exhaustive 2^n, and the count of compatible histories depends strongly on the axes (43 to 711 at one delta); no cheaper algorithm is claimed or tested. Reach: monotone in delta, but it is not the useful number: with six-root axes the reach is tick 0 at every delta tried, and the amplitude there is around 10^-35, so reach says what is allowed and the amplitude says what is possible.
3. **The plan's failure condition ("the vector constraint removes the asymmetry") is partly met.** The asymmetry in the amplitudes is untouched. The net asymmetry (best compatible backward snap against best compatible forward one) keeps its sign whenever single toggles are compatible, and can be reversed when the constraint is tight and compensation is needed. So the orientation is carried by the amplitudes (the flow law), and a tight vector constraint can override it; whether the tiling does depends on the axes it supplies, which is not known here (Stage 0 gives the flat vertex per tile but not an axis sequence along a timeline).
4. **The scale ladder is a one-line consequence of an assumption**, not a result about the tiling: the same Pisot number controls the tiling's smoothing (g^j) and the bit's readability under that assumption. The assumption (sigma fixed, d proportional to the tile) is the thing to test, and nothing here does.

## 4. Not done, and carried forward

- An axes sequence taken from the tiling (the opening direction of the flat vertex tile by tile) instead of periodic and random stand-ins.
- Amplitudes for six free letters per tick.
- A reference that is a history of the substrate rather than the chain's single late toggle.
- Stage 3 (the arrow audit) starts from the fact that the action is reversal symmetric on closed timelines (E1.2) and the orientation here comes from the amplitude schedule; the audit should list every place an orientation enters, with this one added.
