# Stage 7b of toymodel_v8: the four hypotheses of STAGE7.md, run

Engine: `spectre_flip_stats.py` (run from `wada-coudene`, class 0, level-3 frame, placements within 7 of the tile at the origin, 3180 of them, all 12 classes; membership in a window by nearest point of the depth-7 cloud, IN_T = 0.003, OUT_T = 0.010, samples with any ambiguous decision dropped). All results are **Counted**, one class and one footprint, with the sample sizes given. Logs: `stats_d0.03_partB_log.txt`, `stats_d0.015_log.txt`; JSON `spectre_flip_stats_d0.03.json` (Part A only; the first run's Part B numbers were overwritten by a rerun and survive in the log).

A selection effect applies to every rate below: dropping samples whose present set is ambiguous removes the phases that lie within 0.01 of a window boundary, which are the ones that would change under a small step, so absolute rates fall faster than delta^0.89 when delta is lowered (changed tiles at rho = 7: 0.488 at delta = 0.03 against 0.165 at 0.015, a factor 3.0 where 1.85 is expected). Ratios between discs at one delta are not affected.

## H8.5: changed tiles in a disc grow as rho^2 (supported)

2274 usable samples, delta = 0.03. Mean number of tiles present in a disc of radius rho about the tile's centre, and mean number whose presence changed:

| rho | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|
| present | 2.2 | 6.2 | 7.3 | 12.6 | 17.3 |
| changed | 0.060 | 0.169 | 0.212 | 0.348 | 0.488 |
| changed per present tile | 0.027 | 0.027 | 0.029 | 0.028 | 0.028 |

Log-log slope of changed tiles against rho (rho 2 to 7): 2.22 (2.35 at delta = 0.015); the present count itself has slope about 2.3. The changed fraction per tile is constant at 0.028 (this delta), so changes have a uniform positive density and are never confined to a bounded region. This is what local recoverability requires (H2 in the notes: two tilings of one hull cannot differ only inside a bounded region), and it means a flip cluster is a part of a plane-filling pattern of changes, not an isolated event. It does not test H2 itself.

## H8.4: the Mystic-pair count is not locally conserved (supported, small samples)

The conserved quantity of a closed cluster is the area, n_O + 2 n_G (G = Mystic pair, even-parity pair of Stage 7). Among events in a disc in which that quantity is unchanged (a stand-in for an interior event; the number is small), the number of Mystic pairs still changed:

| rho | 1.5 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|
| area-conserving events | 6 | 5 | 18 | 18 | 15 | 27 |
| with n_G changed | 0 | 4 | 4 | 1 | 5 | 9 |

At rho = 7, 9 of 27. A change with the area fixed is a pair exchanged for two singles (n_G - 1, n_O + 2), the tile-count change of one seen in Stage 3. So the even-parity (orientation) count of a region changes under flips while the total area does not: the parity is a density (E8.3), not a local conserved charge. Limits: n of 5 to 27; area-conserving in a disc is not the same as a closed cluster, boundary-crossing events that happen to balance are included; one class.

## H8.6: multiplicity and erasure (not supported: negative)

Erase the tile at the origin. The tiles outside it (all placements within 7 that do not meet it) are kept. Take 50 phases within 0.05 of a true phase and keep those whose outside tiles are identical to the true ones; ask whether the erased tile's neighbourhood (the set of tiles meeting it) is then the same in all of them. 159 true phases:

| multiplicity of outcomes in the ball (distinct inside sets) | n | P(inside not determined by outside) |
|---|---|---|
| 3 or more | 24 | 0.000 |
| 2 | 10 | 0.000 |
| 1 | 125 | 0.000 |

(At ball radius 0.025: 5, 7, 147 cases, all 0.) In no case was the erased region ambiguous, including the 24 cases where the unconstrained ball has three or more outcomes: the outside tiles always rule the extra outcomes out. So the places where 3 or more window pieces meet are not where this erasure is hardest to recover. The only trace of the multiplicity is that fewer ball points are consistent with the outside when the multiplicity is high (0.57 of the ball against 0.86), i.e. the outside constrains more strongly there, not less. Limits: one erased tile, exterior radius 7 (finite), ball radius 0.05, 159 phases. In the infinite exterior the consistent cell shrinks to a point, so a finite exterior is the harder case, and still no ambiguity appeared.

## H8.7: rank-4 cohomology and the lift Z^4 (a map is defined; the rank statement is not settled)

Definition D8.3. For each integer functional l on the lift Z^4, the closed one-form on the tiling that assigns to every edge the l-value of its lift vector (exact in the plane, since it is the differential of l on the vertices). Candidates for classes of exact one-forms with a bounded primitive are those l whose range on the lifted vertices stays bounded as the patch grows.

Exact/Counted (`t8_parity.py`, E8.5): no integer functional with |l_i| <= 3 has a bounded range: the smallest range at level 4 is 20.1 (l = (0, -2, 3, 2)), growing 1.7 times per level; all four coordinates grow by about 2.7 per level (114.7, 123.2, 85.9, 67.3 at level 4). The real bounded plane of `lift4d.py` (the internal space) has range 2.17 at level 3 and 2.22 at level 4. So the four integer coordinate forms are all non-exact in the bounded-primitive sense, which agrees with a rank-4 group of classes, while the two real combinations in the internal plane do have bounded primitives. Tension, not resolved: if a bounded primitive made a class zero in the real cohomology, the real span of the four classes would have dimension 2, not 4; whether the hull cohomology of Baake et al. counts it that way (continuous primitives that are not locally derived) we have not checked, because we did not read that paper or the Batle and Bednorz Section 5. **No map to H^1 of the hull is claimed.** What stands: the Z^4 coordinate forms are four Z-independent non-exact one-forms on the tiling, and the internal plane is a real two-dimensional subspace of them with bounded primitives.

## Summary

| hypothesis | result | label |
|---|---|---|
| H8.4 parity changes locally | 9 of 27 area-conserving events at rho = 7 change the pair count | Counted, small n |
| H8.5 changes grow as rho^2 | slope 2.2, constant 0.028 per tile | Counted |
| H8.6 multiplicity locates hard erasures | no ambiguous erasure at all | **Negative** |
| H8.7 H^1 rank 4 against Z^4 | four integer forms non-exact; real internal plane bounded; no map claimed | Counted, open |
