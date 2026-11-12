# Stage 4: results (open questions of Section 2, and coupled bits on a patch)

Labels as in `PLAN.md`. Files: `t4_open.py` (4 tests), `t4_couple.py` (3 tests). Both run in well under a minute (`python3 research/toymodel_v4/<file> --tests-only`).

## A. Two open questions

Decision on 'entropy' (D5.1, D5.2): two definitions, both reported. *Counting*: `S_count = N * 1[two valid distinct open states]`, `N = 82` pairs of the level-3 patch. *Resolved*: the two states count only if they differ by more than the resolution of the model (the snap scale 0.5): `S_res = N log2(1 + d/eps)` where the reflex outline is simple.

| id | statement | label |
|---|---|---|
| E5.1 | On a grid of 17 values of `omega_0` (10 to 170 degrees) and 5 of `k_0` (0.1 to 0.9), both entropies are non-increasing along the linear flow in exactly the 62 of 85 cells in which the reflex outline is simple at `tau = 0` (`omega_0 < omega_c(k_0)`), and in none of the others, where S rises from 0 and falls | Numeric (grid) |
| E5.2 | The counting entropy does not fall 'toward' the Hat: it is constant up to `tau < 1` and drops to 0 at `omega = 0`, a step at the Hat. The resolved entropy falls continuously (1 per cent of its Spectre value at `tau = 0.99`) | Exact (omega_0 = 30) |
| E5.3 | On 40 random admissible paths per `omega_0`, S_res is monotone on all for `omega_0` = 30 and 100 and on none for 136 and 165 | Numeric |
| E5.4 | **No displacement of P_8 gives the relative sign -1.** Over valid outlines (both bit values simple) with P_8 displaced in any direction `phi` and by any `h`, `Delta = turn_- - turn_+` at P_8 satisfies `|Delta| < 2 pi`. The largest found is 313 degrees (coarse scan; 315 on a finer one) at `phi = -40`, `h` about 4; the chord-normal displacement of the flow gives at most `2 omega_c` = 264 degrees. `-1` needs 360 | Numeric (scan); Negative |

What it says. (1) The fall of the bit count survives every choice of `omega_0` and `k_0` for which the Spectre end is valid, and for both definitions; above `omega_c` there is no monotone entropy of either kind, so the arrow it gives is not just opposed to D6 but conditional on a parameter. (2) Tilting the displacement of P_8 gains about 50 degrees over the chord normal and does not approach 360: the lift's relative sign `-1` of Stage 3 is not reached by moving P_8. (3) Not done: moving other vertices, per-edge curve strength.

## B. Coupled bits on a patch

Definitions D6.1 to D6.4 are in `t4_couple.py`. The interaction (D6.3) is a choice: area charges `+-h` at the centroids of the two tiles of a pair, with a kernel `-log r`, `1/r` or `1/r^2`, no free coefficient.

| id | statement | label |
|---|---|---|
| E4.1 | The edges 7 and 8 of all 164 tiles of flat pairs meet only the partner (7 with 8, 8 with 7): 328 seam rows, nothing else. So the outer boundary of a pair is unchanged by its opening, and nothing outside sees it. Opening every pair at once, each with its own `h` and sign, closes every seam (random states at two `(b, k)`) | Exact |
| E4.2 | `b` and `k` are one clock for the whole patch: changing either for a single tile breaks seams around it (12 tiles tested for each), opening a pair by any `h` does not. `omega` is the only parameter free per pair | Exact |
| E4.3 | **No ordering emerges.** Under D6.3 the ground state is independent of `omega` (energy scales as `h^2`) and, for the 16 pairs nearest the centre, independent of `b` (the same state at all 21 values of `b`) and of the kernel; its energy drifts monotonically with `b`. On 8 random 12-pair subsets the ground state changes 0, 1, 1 times in total for the three kernels | Numeric; Negative |

What it says. Geometry couples the tiles of a pair and nothing else: pairs are independent, and the patch's only shared parameters are `b` and `k`, i.e. the same quantities whose order Stage 1 found. The interaction of D6.3 has to be put in, and when it is the cluster has one ground state throughout, so no sequence of events along the flow appears. The one dependence on the flow is the scale `h^2`, which is a function of `omega` and falls to 0 at the Hat. So the answer to 'does an ordering of tau emerge that does not depend on its parameterisation' is: the orderings available are those of `b`, `k` and `omega` themselves, and coupling the bits adds none.

## What is not done

- The whole patch (82 spins): no exact solution; annealing was not done.
- Validity in the coupled model: above `omega_c` a pair cannot open, so the set of spins changes along the flow; the cluster was solved for all pairs open.
- Per-pair `omega_p` in the interaction; kernels other than the three; interaction through anything but area.
- Whether the area of a tile is a source of anything: Hypothesis.
