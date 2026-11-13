# Stage 8: results (renormalisation by the Spectre's own substitution)

Labels as in `PLAN.md`. File: `t7_rg.py` (4 tests, about 2 s), figure `figures/t7_stage8_rg.png`.

```
python3 research/toymodel_v7/t7_rg.py --tests-only
python3 research/toymodel_v7/t7_rg.py --no-show --save research/toymodel_v7/figures/t7_stage8_rg.png
```

## 1. What was built

The patch of level L is L substitutions of the Spectre. The construction of `spectre_patch` is repeated with each tile's path of slot indices and its labels recorded, and it reproduces the placements exactly. A level-1 supertile of the level-L patch is then the set of tiles whose paths differ only in the last slot, and it corresponds to one tile of the level-(L-1) patch. The question is whether the pair model of v6 (flat pairs as qubits, dipolar couplings) at level L maps onto the one at level L-1 under this correspondence, and whether the map closes.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.25 | **No pair lies inside a supertile.** At levels 2, 3, 4, none of the 6, 82, 804 flat pairs has both tiles in one level-1 supertile. Every pair joins two supertiles, so a block map by supertiles cuts every pair qubit: the pairs live on the boundaries of the blocks, as Stage 3's gauge links live on the boundaries of tiles | Exact |
| E7.26 | **Pairs are inherited.** Every flat pair of the level-(L-1) patch reappears as exactly one flat pair of the level-L patch, between the two supertiles that correspond to its tiles (6 of 6, 82 of 82). The other pairs (76 at level 3, 722 at level 4) join adjacent supertiles whose tiles are not a pair; none joins non-adjacent supertiles. So 82 = 6 + 76 and 804 = 82 + 722 | Exact |
| E7.27 | **The pair set closes by a local rule.** The number of pairs on the boundary between two adjacent supertiles is a function of the two tiles' labels and shared edges: 194 types at level 4 and 96 at level 3, each with a single count, never more than one pair. The 74 types seen at both levels agree. 120 types appear first at level 4, so the table is not yet complete | Exact |
| E7.28 | **The couplings of inherited pairs close too, and are irrelevant.** All inherited dipoles at a level transform by one fixed orthogonal map (angles equal to 1e-9, reflection included), and their midpoints inflate by 2.803 (lambda = 2.806). For inherited pairs more than 60 apart, the coupling is lambda^-3 times the coupling of the transformed parents (median ratio 1.002, quartiles 0.975 and 1.025). A pair's tunnelling is unchanged, so K / Gamma falls by lambda^-3 = 0.045 per level: the inherited pairs flow to the paramagnet | Exact (dipole map), Numeric (ratio) |

## 3. What this says

1. **The substitution is a renormalisation map for the pair model, in a specific sense.** Blocks are supertiles, and the degrees of freedom live on block boundaries, not inside blocks. The old pairs survive one-to-one and new ones appear on boundaries by a local rule. That is closure of the pair set; the plan's stop condition (no closure at levels 2 to 4) is not met.
2. **The fixed point is trivial for the inherited part.** Pairs are tile-scale dipoles. Under inflation their separations grow by lambda and their size does not, so their couplings fall as lambda^-3 while their tunnelling stays: the coupling is irrelevant. At large scale the inherited pairs are a paramagnet, the Hat-end phase of v6 Stage 1. The interesting physics stays at the tile scale, carried by the new pairs of each level, which makes the ensemble self-similar rather than flowing to a nontrivial interacting fixed point.
3. **Abelian only.** This is the Ising (Z2) pair model. A non-abelian version (the paper's Wilson-loop problem) would need non-abelian degrees of freedom on the same boundaries; the boundary structure found here (pairs on supertile boundaries, like links) is where they would sit.

## 4. Not done

The complete table of the local rule (level 5, about 34,000 tiles); the coupling map for new pairs (tile-scale, not a scaling); a non-abelian version on the supertile boundaries.
