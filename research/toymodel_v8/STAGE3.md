# Stage 3 of toymodel_v8: is a flip a standard local move? (attempted; **not answered**)

Status: the question is **unresolved**. What was learned is listed below, with what failed. Script: `spectre_flip_moves.py` (run from `wada-coudene`, `IN_T=0.003 OUT_T=0.010 NS=4000 DELTA=0.03 R2=11 DIAM=8 python3 figures/spectre_flip_moves.py 3 0`).

## What the stage set out to do

Describe a closed flip cluster (tiles R removed, tiles A added, the same footprint) as an operation on the cell complex (vertices V, edges E, tiles F): count (dV, dE, dF) inside the footprint, which satisfy dV - dE + dF = 0 on a disc, and compare with the elementary moves of a 2D cell complex: edge flip (0,0,0), vertex split/merge (+-1,+-1,0), edge insertion/deletion (0,+-1,+-1), and for triangulations the Pachner moves 1-3 (+1,+3,+2) and 2-2 (0,0,0). The lower bound on the number of elementary moves in a cluster is |dV| + |dF|.

## Two limits stated before the results

- In two dimensions any two cell decompositions of a disc with the same boundary are connected by local moves, so "a flip is a composite of local moves" is close to automatic. The informative quantities would be the length of the word and the diameter of the cluster, not membership in the class of moves.
- Pachner moves in four dimensions are the ones that matter for quantum gravity. A planar tiling is a two-dimensional slice of the lift in Z^4, which is not a simplicial complex. A match here would be an analogy at best (**Hypothesis**).

## What was found (Counted)

1. **The overlaps that made 397 of 740 samples "invalid" are tolerance.** Fraction of unambiguous samples whose present tiles overlap on the footprint of the tile at the origin, class 0, 1500 samples: 4.5% at IN_T = 0.008, 2.3% at 0.004, 0.6% at 0.002. The ratio is about the tolerance ratio (the window boundary has dimension 1.11), the same trend as the covering test. So the overlaps are an artefact of the raster tolerance and fall with it.
2. **Holes are a different problem, and not fully resolved.** In a larger region (P1 buffered by 5.5) most samples had holes at the periphery, with the same areas (1.321, 0.344, 3.17) at the same places for most phases, i.e. a systematic missing tile. One cause was found and fixed: the placement list cut off the internal offset at 2 RAD, but z lies up to 0.3 beyond the window radius, so tiles near the edge were dropped (now 2 RAD + 0.6). It did not remove the holes (valid fraction on the large region 35 of 408 changed samples); on the buffer-3 region it is 225 of 408. The remaining cause is unknown (candidates: placement cut-offs of the lattice candidate box, the pair shape 'G'). Until it is found the closed-cluster test cannot be trusted at large size.
3. **Clusters are large.** At step 0.03, clusters of overlap-linked removed and added tiles, inspected for the first twelve of 72, have 1 to 13 tiles on each side. The total area removed equals the total added when the cluster is closed (73.77 against 73.77, 90.16 against 90.16, 131.14 against 131.14), as it must. The number of tiles differs by one between the two sides in 7 of these twelve (9 to 8, 7 to 6, 5 to 6, 6 to 7, 1 to 2, 12 to 13, 11 to 10), and by more in the rest; so a flip changes the tile count by +-1, which is a face-split or face-merge type change (dF = +-1), but also exchanges many tiles at equal count. This is a count on unvalidated clusters (see 4).
4. **No cluster passed the closure test.** Of 72 clusters, none had equal footprints with a valid tiling on both sides: the symmetric difference of the two footprints is 8.2, 10.4, 18.6 or 20.8 (multiples of an elementary area), so the clusters are not closed inside the validated region. So (dV, dE, dF) was not computed for any cluster and **no move classification is claimed**.

## Reading

- "A flip is a standard local move" is **not shown**, and "a flip is not a standard move" is **not shown** either. The stage is unresolved and not a Negative.
- Useful for the next attempt: flips at step 0.03 involve many tiles (up to 13 per side in the sample), so the flip is far from local in the sense of one tile; a smaller step (0.005 to 0.01), a reliable placement list (find the missing tile) and a validity check over the whole footprint of each cluster, not over a fixed region, are needed. The data to settle it is one run of the engine with those three changes.
- Nothing in this stage supports or contradicts a connection with quantum gravity.
