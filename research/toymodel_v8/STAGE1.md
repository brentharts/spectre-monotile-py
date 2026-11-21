# Stage 1 of toymodel_v8: why some phases do not tile the region (partly answered; **open**)

Engine: `spectre_flip_moves.py` / `spectre_flip_stats.py` (class 0, level-3 frame, all 12 classes, depth-7 window clouds). Question: of the 740 changed phase pairs, 397 failed to tile the region without overlaps or holes. Findings (Counted):

1. **Overlaps are raster tolerance.** Fraction of unambiguous phases whose present tiles overlap on the footprint of the tile at the origin: 4.5%, 2.3%, 0.6% at IN_T = 0.008, 0.004, 0.002 (STAGE3.md). They fall with the tolerance at about the rate of the boundary dimension (1.11).
2. **A placement cut-off dropped tiles.** The internal offset of a candidate was cut at 2 RAD, but the phase lies up to 0.3 outside the window radius; with the cut-off at 2 RAD + 0.6 the missing tiles at the edge of the phase box returned. This did not remove the holes.
3. **Holes are real gaps between present tiles.** In the region within 3 of the origin tile, holes of the same area (0.152) appear at the same place (-6.45, 0.33) for every phase tried (four). At that point two present tiles (class 3 at (-4,-4,-2,4), class 1 at (-3,-3,-4,5); their window distances are 0.0007) leave a gap of width 0.15 to 0.17, and no present tile covers it. The tile that would cover it (class 8, pair shape G, offset (-1,-4,-2,1)) has its window at distance 0.022 from the phase, outside the 0.010 band, so it is counted absent; the other covering candidates are at 0.050, 0.081 and 0.155.
4. **So the missing tile is a window that the phase just misses**, not a placement-list error. Either the phase box (Z_a bounding box plus 0.3) reaches beyond the domain of phases for which the window system tiles far from the origin, or the window of that G class is larger than the depth-7 cloud shows (the cloud approximates the attractor to about RAD / lambda^7 = 0.003, which is smaller than 0.022). Not decided.

**Not resolved.** A test that would decide it: compute the covering of the plane by present tiles on a grid of phases inside Z_a only (phases where the tile at the origin is present, hence a tiling exists) and see whether any gap appears; if none, the holes are the phase-domain edge (restrict sampling to the domain, and closed clusters can then be sought); if gaps appear, the G windows need a deeper cloud. Until then no closed-cluster statistic of Stage 3 is claimed.

## Update (2026-11-20, first version; **superseded by the correction below**): the cause is found at the level of the placement list

Tests run on the phases inside Z_a, where a tiling certainly exists (all Counted):

| check | result |
|---|---|
| holes among phases drawn uniformly in the box | 48 of 306 unambiguous (15.7%) inside Z_a; 5 of 616 (0.8%) outside it. Phases drawn from the window cloud itself: 10.8% (so not a boundary bias) |
| where the holes are | a single place, (-6.4, 0.3), area 0.152, in every one of the 25 and 48 cases |
| real level-3 patch built with the same polygon code (496 tiles) | no gap and no overlap in discs of radius 4, 8 and 12 |
| predicted z_b = B z + d against the actual z of the other tile, all 245 520 ordered pairs of tiles of the real patch | largest error 3.1e-14 |
| real tile z against the window clouds Z_0(S) | distance 0.0000 for all 12 classes (and 0.8 to 4.9 from Z_1(S)) |
| depth of the cloud | the distance from the hole phases to the window of the class-9 candidate is 0.150 median at depth 7 and 0.149 at depth 8: not a cloud error |
| **all placements containing the hole point, ignoring the lattice-coset restriction** | **placements of classes 3, 4, 2, 6, 1, 1 whose windows contain the phase (median distance 0.0003 to 0.0008, i.e. present) and whose lattice vectors are outside the coset used so far** |

So the coset restriction of Result `res:index36` (every tile vector of a class lies in one coset of the index-36 sublattice, found on a level-3 patch) is **too strict for the continuum of phases**: at about 16% of the phases of Z_a a valid tile lies in another coset relative to the tile at the origin. The partition at the probe point of the covering test is unaffected (it is exact there), but the placement list is incomplete at far points, and the holes are the missing tiles. Without the restriction the covering test overcounts (36 to 39 windows per interior point), so the coset is a function of the phase, not a constant.

Also found: at an even number of substitutions (N = 4) the leaf coordinates no longer fit the Z_0 frames (the affine fit fails), i.e. the two parities of patch correspond to the two LI classes of Stage 7, with different window systems. All level-3 results here belong to one LI class.

Consequences for earlier results (to be stated in the paper if used):
- The Wada-type covering test and the tolerance refinement (probe point and its overlapping tiles) are not affected.
- The flip statistics of `RECONNECTION.md` and Stages 3 and 7b used the coset-restricted list. Samples with holes were excluded as invalid (hence 343 of 740 kept); the kept phases are those where the restricted list happens to be complete, so they are a biased subset of the phases, not a uniform sample. The rate and scaling numbers (0.028 per tile, slope 2.2) and the erasure test were computed on all unambiguous phases and are affected the same way; they are lower bounds on change counts, not unbiased estimates. The relative-coset rule needs to be found before they can be called representative.

Next test: for each hole phase find the coset offset of the missing tiles (the 36 residues of Z^4/H) and see whether it is constant on pieces of Z_a (for instance on the thirteen children Z_1(S') of Z_a); if so, the offset is a finer label of the phase and the window system with cosets is complete.


## Correction and coset test (2026-11-20, later): the coset explanation above is **not supported**

Scripts: `spectre_coset_label.py`, `analyse_coset_depth.py`, `analyse_coset_depth2.py`, `plot_coset_label.py` (figure `spectre_coset_label.png`, data `spectre_coset_label_q-6.4_0.3.json`).

Test. For 2309 unambiguous phases drawn uniformly in Z_0(O) (the window of the tile at the origin), the tiles of all 36 relative cosets whose footprint contains the point q = (-6.4, 0.3) were tested for presence.

| result | value |
|---|---|
| placements containing q, all cosets | 3934, in 36 relative cosets |
| phases where every coset has a present covering tile | 1936 of 2309 |
| phases where only coset 0 has none | 284 (12.3%), children 2, 5 and 11 of Z_0(O) only |
| phases where one other coset has none | 89 (3.9%), children 2, 8, 12 |
| is coset 0 missing constant on the 13 children? | **no**: 0.12, 0.38, 0.42 of the phases of children 2, 5, 11, none elsewhere |
| purity of the flag on pieces of depth 1, 2, 3 against a shuffle null | 0.57 (null 0.57), 0.81 (0.57 +- 0.003), 0.97 (0.65 +- 0.007) |

So the missing-tile flag is structured on pieces finer than the children (depth 2 or 3), and it is not constant on the 13 children, which refutes that form of the proposed label.

The important check, made afterwards: **real tilings never use another coset.** Patches of 433 or 496 tiles built from each of the nine supertile types (Gamma, Delta, Theta, Lambda, Xi, Pi, Sigma, Phi, Psi) at level 3 have, for every tile of every one of the 12 classes, the same coset as the Delta-rooted representative (relative coset 0), and every tile's z lies exactly in its window cloud (distance 0.0000). Together with the earlier exact z_b formula (3e-14 on 245 520 pairs) this means the coset-0 model reproduces every real tiling exactly.

What this changes:
- The statement above that "valid tiles in other cosets exist at 16% of phases" is an **overcount**: at every phase 35 or 36 of the 36 cosets have a present tile covering q, so the presence of a covering tile in another coset says nothing. The restriction to coset 0 is correct for real tilings. The claim in the update that earlier flip statistics are biased because the restriction is too strict is **withdrawn**.
- What is left unexplained: at 12% of the phases of Z_0(O) the coset-0 family has no window containing the phase at q, although every real tiling is covered. Real phases (cloud points) are dense in Z_0(O), so a hole region of positive measure should not exist. Candidates not yet tested: the sampled point q may sit on a tile edge (it is a rounded copy of the hole's representative point (-6.45, 0.33), and `contains` fails on boundaries); the placement list may miss coset-0 tiles by a cut-off (the lattice box or the 7-radius filter); or the windows have thin parts that the depth-7 cloud does not resolve at tolerance 0.003 (the class-9 test showed no change from depth 7 to 8, which argues against it).
- The flip statistics keep the status they had before this update: computed on phases where the restricted list tiled the region; the validity filter is the open issue, not a proven bias.

The next test is simple: take a real phase (a cloud point of Z_0(O) with its real neighbourhood from a patch) and check the hole point directly in that patch, then move the phase by 0.001, 0.003, 0.01 and watch where the covering tile drops out of the list.

## Result (2026-11-20, real-phase test): the holes are a **box cut-off in the placement list**, not a coset effect. Status: Counted, the cause confirmed.

Scripts: `spectre_realpairs.py` (real pairs against the model filters), `spectre_coset_realphase.py` (hole test with `PBOX`, `RADQ`, `DCUT` switches).

1. Real pairs. For every class-0 tile of a real level-3 patch and 41 offsets (q and random points within 6.5), the real covering tile gives a pair (b, w = v_b - v_a). 1934 pairs: all have relative coset 0 (as expected), 1929 pass every placement filter, and **5 fail one filter**: the lattice box `|PH w| < 12`. All five are the same Mystic-pair tile (class 5, w = (-5,-5,-6,6)), whose footprint is displaced from the physical part of w: |PH w| = 12.77 while the tile covers a point at distance below 7 from the origin tile.
2. Hole test with the box widened from 12 to 20 (everything else unchanged, same q, same phase sampler, 722 unambiguous phases): **0 phases have a missing coset, in every child** (before: 12.3% coset 0 missing, 3.9% one other coset). The lattice candidates grew from 48 847 to 135 693 and the placements containing q from 3934 to 3960.
3. The 12% matches the window-area fraction of the Mystic-pair classes (G/O = 0.1455 in total), which is why the holes sat on structured pieces of the window: they were the phases where the covering tile is a class-5 tile.

Consequences.
- The window system with the coset-0 restriction is **complete**; no finer label is needed. The hypothesis of the test ("offset is a finer label") is not needed: the offset is the constant 0.
- The structure found earlier (purity 0.81 at depth 2, 0.97 at depth 3) was the shape of the class-5 window inside Z_0(O); it is a property of the missing tile, not of a coset.
- Withdrawn: the first update of this file, the "coset restriction too strict" claim and the statement that valid tiles in other cosets exist. Already withdrawn above and now the real reason is known.
- **Affected earlier results.** `spectre_flip_contacts.py`, `spectre_flip_moves.py` and `spectre_flip_stats.py` use the same cut-off (`np.hypot(12.0, ymax)`, `< 12.0`). Tiles of Mystic-pair classes at the periphery of the placement list were missing in them. So: (i) the flip counts and the area/ρ² statistics are lower bounds with a systematic loss concentrated on Mystic-pair tiles, in particular the parity result of Stage 7b needs a rerun; (ii) the failure of Stage 3 to close a cluster (holes in the list) has a known cause and is not a negative result about Pachner moves; (iii) the contact-graph result (20 of 30 changed) used a footprint whose tile list may have lacked Mystic tiles. None of these is shown to be wrong, none is confirmed.

Next: set the box to 20 in the three engines (a parameter, default 20), rerun Stage 3 cluster closure, then the statistics and the contact-graph test, and compare with the old numbers.
