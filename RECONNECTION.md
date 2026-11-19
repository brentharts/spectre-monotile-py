# Gravitational reconnection (Comisso & Asenjo) and this repository

Source: Comisso & Asenjo, "Reconnection of Gravitational Fields", arXiv:2609.25241v1 / Phys. Rev. D 114, 084095 (2026). Read in full from the arXiv PDF; derivations not checked; published version not compared. Paper section: `sec:toward-reconnect` in `quantgrav.tex`.

| Idea | Status | Statement |
|---|---|---|
| Flux and connectivity | **Exact** (flux), no analogue (connectivity) | In the Z2 gauge form, the flux through a plaquette set S equals the product of Z_l over the cut of S, so X_l changes it iff l is in the cut (discrete Eq. 29). Test `test_e622_...` in `research/toymodel_v6/t6_gauge.py`: level 3 has 3534 links, 186 change one plaquette flux, 3348 change two. Sheet connectivity is not defined: flux is pointlike on a 2D complex. |
| Phason flip = reconnection | **Negative** as stated | Lift edge vectors are differences of lattice points, so the tetrad 1-forms are exact and F = 0 for every phase. Flips are ideal. A different object (tile contact graph) is needed; the Wada-type multiplicity (about 10% at r = 0.064) is a candidate statistic, untested. The identification tetrad index = lift coordinate is ours. |
| Helicity = handedness | **Idea** | Helicity needs A.B in 3+1; the tiling is planar. Only a parity-odd analogy (Im Z_k, chiral outline). |
| Sparling exchange = missing source | **Negative** | X is proportional to F.T, vanishing in vacuum, so it is no monopole source. Only a structural analogy to the pair ledger (sum conserved, parts exchange). |

## Test: is a window crossing a one-for-one swap? (Counted, from the covering data of wada-coudene)

Question. If a phason shift moves a tile's internal point out of its window, does the tile vanish, split, or get replaced by one tile? Only a replacement by one overlapping tile can be absorbed by relabelling; a split or merge would change the tile count and would be a candidate non-absorbable (reconnection-like) event.

Data. `figures/spectre_window_covering_N3_c0{,_n1400,_n2800}.npz` (class 0): for each pixel of the internal plane, `cnt` = number of overlapping tile placements present at that internal point, `own` = inside the tile's own window. Outside the window:

| raster | outside pixels | exactly 1 present | 2 present | 3+ | none |
|---|---|---|---|---|---|
| 700 px | 358 283 | 97.0% | 2.96% | 0.03% | 0 |
| 1400 px | 1 440 271 | 98.67% | 1.32% | 0.01% | 0 |
| 2800 px | 5 776 722 | 99.58% | 0.42% | 0.00% | 0 |

The 2-present fraction falls about threefold per doubling of resolution, the same trend as the tolerance run, so it is raster tolerance. Counted result: to this resolution, a crossing is replaced by exactly one overlapping tile, never zero (no holes) and never two or more. Inside the window the count is 0 up to the same tolerance (nonzero pixels 5.0%, 2.2%, 0.7% of the inside).

Reading. The tile count is conserved at every crossing, so by the paper's criterion (reconnection needs a change not absorbable by a redefinition) these events are slips. The Wada multiplicity (3+ distinct neighbour windows within r of a boundary point, about 10% at r = 0.064) measures how uncertain which replacement occurs at a given phase resolution, not a split. So the candidate statistic does not by itself give a non-absorbable event: Negative for "Wada multiplicity = reconnection".

(Superseded in part by the next section: the count test says one overlapping tile covers the probe point, but a flip is a cluster of tiles, not a single swap.)

Not tested (then). Whether the replacement tile b (offset w, class change) changes the contact graph of its neighbours needs an actual tiling with the swap carried out, and a rule for which neighbours stay compatible. The covering data identify the pixel's present tiles only by count. That is the next computation.


## Test: contact graph under actual phason flips (Counted; one class, one footprint, 343 pairs)

Script: `spectre_flip_contacts.py` (in `wada-coudene/figures/`; copy in `research/`), run as `NS=3000 DELTA=0.05 R2=7 DIAM=5.5 python3 figures/spectre_flip_contacts.py 3 0`. Output: `spectre_flip_contacts_pattern_c0.json`.

Method. Tile a (class 0, level-3 frame) sits at the origin. For every placement (class b, offset w) with footprint within 7 of a (2793 placements, all 12 classes, offsets in the index-36 coset) the tile is present at internal point z iff its window contains B z + d. A random z and z + 0.05 e^{i phi} (3000 samples) give two phases; a sample is kept only if every membership decision is unambiguous (nearest-point distance < 0.008 or > 0.016): 1924 of 3000. 740 of these change the present tile set anywhere in the region. A pair is kept as a valid flip only if the present tiles tile the region within 1.5 of a with no overlap and no hole before and after: 343 of 740. On the 343, take the tiles that meet the footprint of a, build their contact graph (shared edge segment) before and after, and compare.

Counted result:

| | pairs |
|---|---|
| tile set meeting the footprint unchanged (change happens elsewhere) | 313 |
| tile set changed, contact graph not isomorphic | 20 |
| tile set changed, graph isomorphic as a graph but not with classes | 10 |

Among the 30 pairs that change the footprint's tile set, the number of tiles meeting it changes in 16 (e.g. 1 to 2, 2 to 3) and is unchanged in 14.

Reading.
- A flip is a cluster of tiles exchanged, not one tile swapped for one tile; the single-swap picture of the previous section is wrong as a description of the tiling (it is right only for the count at one point).
- In 20 of 30 the local contact graph changes topology, so some changes are not relabellings of the contact graph. That is evidence for the hypothesis that flips can change connectivity of the contact graph, but only for this definition of connectivity (tiles meeting a fixed footprint). It is not a reconnection in the paper's sense: no F, no R, no transport velocity are defined.
- The 10 class-only changes are relabellings (same graph, different tile types).

Limits (stated, not fixed): one class and one footprint; the step 0.05 can combine several flips; 397 of 740 changed samples were discarded as not valid tilings of the region (these are probably phases outside the domain on which this window system is a tiling, or tolerance, not checked); clusters are not required to be closed, so cluster-level statistics (footprint equality, linkage of outside tiles) were attempted and gave no valid closed clusters in the region; the numbers have no error bars; they depend on DELTA and on the footprint chosen.
