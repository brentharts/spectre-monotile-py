# Stage 1: results (the history-weight rule on the six points of the 14th vertex)

Labels as in `PLAN.md`. Nothing here is physics or models the singlet state. File: `t5_rule.py` (5 tests, about 4 s), figure `figures/t5_stage1_rule.png`.

```
python3 research/toymodel_v5/t5_rule.py --tests-only
python3 research/toymodel_v5/t5_rule.py --no-show --save research/toymodel_v5/figures/t5_stage1_rule.png
```

## 1. What was built

The two-wing history of `toymodel_v2/tm2_rule.py` (Stage 4), with the letters taken from the tile: the six points around P8, V = (0, +-1), (+-k, +-1/2), sorted by angle so that V[i+3] = -V[i], and the step cost exp(-beta |V_i - V_j|^2). A setting m is the letter V[m] (outcome +1) and its antipode (outcome -1); the pass weight, the table and the CHSH value are those of v2 (hard pins). The cubic six is the control.

## 2. Results

| id | statement | label |
|---|---|---|
| E5.6 | **Regression.** At k = sqrt3/2 the tables, the settings and the CHSH value equal `tm2_rule.py` to 1e-12 (beta in {0.5, 1, 1.255, 2}, n = 1, 2); the transfer-matrix tables equal the enumeration of both wings for k in {0, 1/2, sqrt3/2, 1} | Exact |
| E5.7 | No-signalling is exact for every k tried and for the cubic six (marginals 1/2 to 1e-14): the alphabet is antipodal. A bias on the wing letters breaks it | Exact |
| E5.8 | **The geometry sets the strength.** At beta = 1.255, n = 1 CHSH rises strictly with k: 1.670 (k = 0), 1.943 (1/2), 2.828 (sqrt3/2), 3.058 (1), 3.236 (1.3). The stiffness needed to pass 2 is 1.308 at k = 1/2, 0.841 at sqrt3/2, 0.718 at 1; to pass 2 sqrt2 it is 1.255 at sqrt3/2 (v2's value), 1.094 at 1, and not reached below beta = 8 at k = 1/2 | Exact tables; Numeric thresholds |
| E5.9 | **The violating window closes along the flow.** With k = k0(1 - tau), CHSH falls strictly with tau. For beta = 1.255, k0 = sqrt3/2: 2.828 at the Spectre end, below 2 after tau = 0.391, 1.670 at the Hat. For beta = 2: below 2 after tau = 0.718 (k0 = sqrt3/2) and 0.695 (k0 = 0.8); below 2 sqrt2 after tau = 0.304 (k0 = sqrt3/2) | Numeric |
| E5.10 | **The cube's own six are local; their shadow is not.** The cubic six give CHSH <= 2 for every beta tried and tend to 2 from below (1.992 at beta = 2, 2.000 at beta >= 4); the planar six reach 4 (3.9993 at beta = 8, k = 1) | Numeric (grid) |

## 3. What this says

1. **v2's stiffness beta is partly a geometric quantity.** v2 found "no free kappa, the strength is set by (beta, n, mu)". With the tile's six points the squared step distances carry the curvature, so at fixed beta the curvature k sets how violating the table is, and along the flow the strength falls with k(tau). The window in which the shared past can be stiff enough for CHSH > 2 closes at a flow time of order a half of the way to the Hat, depending on beta and k0 (E5.9).
2. **The Hat end is not a flip of the sign of the correlation but a loss of range.** At k = 0 two letters coincide, the table is still defined, and CHSH sits just below 2 (1.67 to 1.95 for beta = 1.255 to 2).
3. **The cubic alphabet cannot violate in this rule at all; its shadow can.** v2 argued for the hexagon through E8; here the same conclusion comes out of the correlations, with the projection of the cube along its diagonal as the passage between them. Not a derivation of why nature would use one: the weight is still v2's Definition.
4. **A caution about reading 2 sqrt 2.** v2 E4.10 stays: the family passes through 2 sqrt 2 and goes above it, so no Tsirelson bound appears here either.

## 4. Not done

Soft read-outs (mu finite) on the unequal-length alphabet; a flux term on the wings; the ties of beta and n to the chain's overlap c (v2 C6 b); the tile placement with the 120 degree choice in 3D; wings on the Spectre tiling.
