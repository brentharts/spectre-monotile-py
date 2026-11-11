# Stage 1: results (an arrow that does not depend on how tau is written)

Labels as in `PLAN.md`. File: `t4_arrow.py` (5 tests, about 5 s).

```
python3 research/toymodel_v4/t4_arrow.py --tests-only --story
```

## What was tested

Rule D6 of `spectre_hat_time_model.py` forbids running tau backwards. Reparameterising tau cannot change which quantity is monotone, so the test has to be stronger. The flow moves three parameters linearly in tau: `b` from `a` to `a sqrt3`, the curve strength `k` from `k_0` to 0, the corner angle `omega` from `omega_0` to 0. Here each parameter gets its **own** monotone law (any shape, `u = tau^p` with one exponent per parameter); call such a path *admissible*. A quantity is a **robust arrow** if it is monotone along every admissible path, and **law-dependent** if it is monotone along the linear flow but not along every admissible path. Candidates, all computed from the outline alone: the area `A`, the outline length `L`, the bending energy `B` (integral of curvature squared over the Bezier edges), validity of each bit value, and the jump `d`.

## Results

| id | statement | label |
|---|---|---|
| E1.1 | The area splits exactly: `A(b, k, omega, s) = A0(b) + s a tan(omega/2)`, `A0(b) = 2 sqrt3 a^2 + 3ab + sqrt3 b^2`, independent of `k` and `omega`; `A0` strictly increases in `b` | Exact |
| E1.2 | **Robust arrows:** `A0`, the reflex tile's area `A_- = A0 - a tan(omega/2)`, and the jump `d = 2a tan(omega/2)` are monotone (up, up, down) along every one of 300 random admissible paths | Exact |
| E1.3 | **Law-dependent:** `A_+` (convex tile) and `B` are monotone along the linear flow (`A_+`: 9.93 to 13.86; `B`: 145.6 to 0) but not along every path: with exponents `(p_b, p_k, p_omega) = (6, 1, 1)` `A_+` falls somewhere (9.93, 9.20, 8.85, 9.34, 13.86), with `(6, 6, 1)` `B` rises (145.6, 150.2, 149.7, 111.9, 0). The outline length `L` is not monotone even along the linear flow (18.97, 17.68, 17.35, 17.59, 18.39) | Exact |
| E1.4 | **Validity is one-way only outside a band.** `s = +1` is a simple curve at all 275 grid points (omega up to 170 degrees). `s = -1` is simple at every point with `omega <= 119` and at none with `omega >= 153`; at `omega = 136` it depends on `(b, k)`, and 5 of the 690 grid steps toward the Hat go valid to invalid, all at `omega = 136` | Exact (grid) |
| E1.5 | The two bit values give the same tile iff `omega = 0`, for every `b` and `k` tested | Exact |

## What it says

1. **The robust arrows are the parameters in disguise.** `A0` is a function of `b`, `A_-` and `d` of `omega` (and `b`). Each increases (`A0`, `A_-`) or decreases (`d`) toward the Hat along every admissible path, and the three agree on the orientation. Nothing else measured has that property: the convex tile's area, the bending energy and the length depend on how the three laws are balanced.
2. **So the arrow is not derived.** Direction D6 is the orientation from the end where the bit is readable and the area small to the end where the bit merges and the area is largest. Three robust quantities point the same way; that is a consistency check on the toy, not an explanation of the direction. The Negative from v2 (an orientation put in by hand) stands, with a sharper statement of what is and is not put in.
3. **The allowed bit values are not monotone.** For `omega_0` above about 130 degrees the reflex value is excluded early (E1.4, and E0.4 of Stage 0), so the number of allowed, distinguishable bit values runs 1, then 2, then 1 (merged) along the flow. The capacity of the bit peaks in the middle; the frozen early end is a stretch where the geometry allows one value only.
4. **The exclusion is not a clean threshold in the parameter box.** The critical `omega_c` depends on `(b, k)` (Stage 3: 133.0, 132.1, 120.5 degrees at `k_0` = 0.3, 0.5, 0.8), so on an admissible path that lingers at `omega` near 136 the reflex value can become valid, invalid and valid again. Outside the band (below about 120 or above about 153 degrees on the grid) validity is one-way.

## What is not done

- The functionals are those of the outline: area, length, bending energy, validity, the jump. No distance to the Hat in a shape metric (turning function, Procrustes) and no gradient-flow test: a one-parameter family is a descent path of some functional trivially, so such a test needs the three-parameter manifold with its induced metric, not built here.
- Paths are `u = tau^p`, one exponent each; other monotone shapes were not sampled. A path that reaches `omega = 0` before `b` reaches `a sqrt3` is admissible and covered by large `p_b`.
- Random paths use `omega_0` between 20 and 160 degrees and `k_0` between 0.1 and 0.9 on the closed form of E1.1; the real tiles were checked on three paths and on the grid.
