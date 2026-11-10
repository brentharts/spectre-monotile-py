# Stage 1: results

Labels as in `PLAN.md`. Nothing here connects to gravity. File: `tm2_history_sum.py` (8 tests, about 14 s), figure `figures/tm2_stage1_influence.png`.

```
python3 research/toymodel_v2/tm2_history_sum.py --tests-only
python3 research/toymodel_v2/tm2_history_sum.py --story
python3 research/toymodel_v2/tm2_history_sum.py --no-show --save research/toymodel_v2/figures/tm2_stage1_influence.png
```

## 1. What was built

An exact solver for the history-sum on the hexagonal alphabet: weight `exp(-beta A(H))` with the paper's action in its literal form (D1.1), an optional flux constraint `N(flux) <= d2`, and an optional pin (a forced node, part of the history). It is a transfer matrix over (last node, prefix flux), exact for any n up to 40, and equals brute-force enumeration to 1e-12 for n ≤ 7. Everything below is computed from it, with no sampling except where stated.

## 2. Results

| id | statement | label |
|---|---|---|
| E1.1 | transfer matrix = brute force (5 parameter sets, with and without pin and constraint) | Exact |
| E1.2 | The action is reversal symmetric on all 2040 closed 6-node timelines and not on open ones (more than half of sampled open ones differ). With closure, marginals about a central pin are mirror symmetric to 1e-12. So the paper's action carries no arrow on closed histories (C5 of the plan, now proved for all n = 6 and checked on the solver) | Exact |
| E1.3 | the pinned node is pinned; marginals are covariant under the sixfold rotation of the alphabet | Exact |
| E1.4 | **Influence of a pin.** Without the flux constraint it decays strictly with distance (n = 13: 0.436, 0.322, 0.229, 0.161, 0.113, 0.079 at distance 1 to 6). With closure (n = 21) it falls to 0.036 at distance 4 and then **rises** to 0.199 at distance 8, larger than at distance 3 (0.069). A pinned node acts through the global constraint on the whole timeline, not as a ripple that fades with distance | Exact (finite); **Negative** for the ripple picture |
| E1.5 | Repair after one forced step: hexagonal n = 5: 1 edit in 7560 of 9000 shocks, 2 in 1440; n = 6: 57960 of 61200, 3240. Equal to the ledger | Exact |
| E1.6 | The v1 greedy contract with its sign error fixed, on the hexagonal alphabet (n = 13, pin at 6, 4000 seeded runs): reaches the admissible set in 85% of runs; per-node marginals within TV 0.10 of the exact uniform-over-admissible ones; but it collapses to one repeated letter in 14% of runs, and the exact probability of that is 0 (a constant timeline is not admissible) | Numeric (sampled, seed 5) with an Exact reference |
| E1.7 | The flux constraint is **blind to the bit**: opening the flat vertex leaves every tile outline closed (to 1e-15) and moves an edge of all 71 tiles off the alphabet. The seam constraint is **strictly inside** the area ledger: on the 6 flat pairs of the level-2 patch, of 4096 sign assignments, 64 close all seams, 924 have total Q = 0, and the 64 are a subset of the 924. At level 3 (82 pairs): 2^82 ≈ 4.8·10^24 against C(164, 82) ≈ 1.5·10^48 | Exact (finite) |

## 3. What this says about the plan

1. **The "ripple" of v1 is not a property of the constraint.** The plan's failure condition for Stage 1 ("influence does not decay with distance") is met when the flux constraint is on. What a forced node does to the rest of the history is set by a conserved total, and it reaches the far end as much as the near neighbours. v1's picture of a wave of rewrites spreading outward from the shock (and the paper's wording "ripple outwards") is true of the greedy solver, not of the constraint it was meant to solve. Without the constraint, influence does fade, geometrically.
2. **The corrected greedy contract is a decent per-node approximation and a poor joint one.** Its marginals are close to the exact ones, which is why v1 looked reasonable; the collapse runs show where it differs.
3. **The two candidate constraints measure different things.** The paper's flux J is conserved by every opening (outline closure), so it cannot distinguish a bit from no bit. The chain's compatibility quantity was always the area ledger `Q = Σ b_j w_j`, and the tiling's seam rule is a strict subset of `Q = 0`. So P4 should be stated with the area ledger, with the seam rule as the substrate's own refinement, and the flux constraint is what governs the letter timeline, not the bit.
4. **A gap Stage 1 does not close.** In these models the *letter timeline* (flux, hexagonal roots) and the *bit* (signed opening at a flat vertex, seams) are two different objects: a bit flip is not a letter change, and a letter change is not an opening. Nothing yet says how the two sit in one history. Stage 2, which brings in the chain's weights, has to do it; the plan should not assume it is automatic.

## 4. Not done in Stage 1

- A history-sum for the seam constraint with weights (all assignments on flat pairs are equally weighted here; the chain's tunnelling weights come in Stage 2).
- The clustering of flat pairs across levels and the limit of the pair fraction.
- Larger n with non-zero λ: the influence curves for λ > 0 are shown but not characterised (they are non-monotone too).
