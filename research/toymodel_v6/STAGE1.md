# Stage 1: results (the pairs of a Spectre patch as a transverse-field Ising model)

Labels as in `PLAN.md`. Nothing here is physics. File: `t6_patch.py` (6 tests, about 13 s), figure `figures/t6_stage1_patch.png`.

```
python3 research/toymodel_v6/t6_patch.py --tests-only
python3 research/toymodel_v6/t6_patch.py --no-show --save research/toymodel_v6/figures/t6_stage1_patch.png
```

## 1. What was built

The pairs are the flat pairs of `toymodel_v4` (`t4_patch.flat_pairs`). Each pair is the qubit of Stage 0. The coupling between pairs is **not chosen**: it is the Coulomb energy of the bits read as charges (toymodel_v5's Definition, a bit is a source of strength h at its tile), here +h tau at the centroid of the pair's first tile and -h tau at the second. Tiles outside pairs cannot open and carry no charge. Then

```
H = - Gamma(tau) sum_A tau^x_A + kappa h(tau)^2 sum_{A<B} W_AB tau^z_A tau^z_B,
W_AB = 1/r(A1,B1) - 1/r(A1,B2) - 1/r(A2,B1) + 1/r(A2,B2)
```

with the geometry recomputed at b(tau), one constant kappa fixed so the strongest coupling at tau = 0 is 1, and Gamma = c(tau) as in Stage 0. Exact diagonalisation on the central 6, 8, 10, 12, 14 pairs of the level-3 patch (open boundary), in the sector even under the global flip.

## 2. Results

| id | statement | label |
|---|---|---|
| E6.6 | The pairs are disjoint (6, 82, 804 at levels 2, 3, 4) and the same list for every b in [1, sqrt3]. At each b every pair has the same centroid separation: 3 at the Spectre, sqrt 21 at the Hat | Exact |
| E6.7 | The charges give an exact Ising term (1e-14 on all sign combinations). It has both signs (1/3 antiferromagnetic at level 2; 0.49 and 0.50 at levels 3 and 4) and falls as r^-3 (slope -2.99 at level 4): the pairs are dipoles | Exact |
| E6.8 | The patch is frustrated: the classical ground state leaves 0.060 of the bond weight unsatisfied at level 2, and 0.128 to 0.165 on the level-3 clusters, rising with the cluster | Exact (brute force) |
| E6.9 | **The frozen end is a cat, not an entangled pair.** As Gamma -> 0 the ground state of every cluster is (\|s> + \|-s>)/sqrt2 of its classical ground state (overlap 1 - 1e-10), and every pairwise concurrence is below 1e-10. For two pairs that cat is a Bell state, which is why Stage 0 found the most entanglement at the Spectre end | Exact; **Negative** for Stage 0 E6.5 on a patch |
| E6.10 | Along the flow the largest pairwise concurrence peaks at 0.38 to 0.65 at tau = 0.05 to 0.13 (depending on the cluster), below the two-pair value 0.913. The largest CHSH of any reduced two-pair state is 2.33 (10 pairs, tau = 0.05), at most 2.12 with the visibility of Stage 0, and above 2 with visibility only for the 10-pair cluster. The gap minimum moves with the cluster (tau = 0 to 0.11) | Numeric (finite clusters) |
| E6.11 | The Hat end is a paramagnet: gap 4 Gamma, concurrence 0, CHSH 2 | Exact |

## 3. What this says

1. **The coupling comes from the geometry, and it is frustrated.** Taking v5's charges at face value fixes the sign and the range of every coupling: dipole-dipole, half ferro, half antiferro. This replaces the hand-chosen K of Stage 0 and the "kernel from E8 distances" of the proposal with a derived one, given the charge Definition.
2. **Stage 0's picture of the flow does not survive on a patch.** Two pairs are maximally entangled at the Spectre end because a two-qubit cat is a Bell state. With more pairs the frozen end is a many-body cat whose two-pair reductions are classical. Pairwise entanglement then lives in the middle of the flow, where tunnelling and coupling compete, and vanishes at both ends.
3. **The "fragile" end changes too.** For two pairs the gap was smallest at the Spectre (0.095). On the clusters the gap there is 0.4 to 0.9 except for the 10-pair cluster (0.09), so the frozen end is protected by frustration-selected order, and the soft region is where the entanglement peaks.
4. **Bell violation is marginal and cluster-dependent.** Below 2 sqrt 2 always (automatic), above 2 only in a window around tau = 0.05 to 0.15, and with the read-out visibility of Stage 0 in one cluster of five. Together with Stage 0 E6.2 (one pair has only one read-out on the tile), a Bell test on the patch is not yet defined (PLAN H6.3, Stage 5).
5. **No transition is claimed.** The minimum of the gap and the peak of the entanglement move with the cluster; finite-size scaling beyond 14 pairs (DMRG or tensor networks on the level-4 pair graph) would be needed.

## 4. Not done

Larger clusters; periodic or embedded boundaries (the pairs outside the cluster as a mean field); thermal states; the hopping and gauge sectors (Stages 2, 3); a second read-out of the pair qubit.
