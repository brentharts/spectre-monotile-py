# toymodel_v11: flip clusters, swaps, connectivity, bit sign

Run with `export PYTHONPATH=<qwla checkout>` (for `spectre_exact`). Each script: `python3 <script>.py --tests-only`.

| script | content | result file / log |
|---|---|---|
| `t11_swap_scan.py` | exhaustive refilling of every vertex-connected hole of k = 1..4 tiles (radius 18 of the level-4 patch, 123 tiles), chiral and with reflected tiles; oracle = brute-force exact cover (shapely); tests s111-s115 | `results_swap_scan.json`, `log_swap_scan.txt` |
| `t11_flip_exact.py` | the six flips of `toymodel_v8/flip_examples.json` made exact (Z[zeta12] vertices, mirrored to the chirality of patch(4)), validity before/after, closed clusters, refill | `results_flip_exact.json`, `log_flip_exact.txt` |
| `t11_connectivity.py` | worm (strip) connectivity; matching of boundary edges through a hole; contact graph | `results_connectivity.json`, `log_connectivity.txt` |
| `t11_bit_sign.py` | corner-turn relative sign at all 14 vertices; B1-B5 | `log_bit_sign_table.txt` |

## Bug fixed in `toymodel_v10/t10_cavity.py`
`Search` forced every vertex of the cavity, including interior ones that the old tiles covered completely. Only boundary vertices are now tracked (`bound`). Chiral results are unchanged (k = 80, 150, 300: one filling, 104, 604, 9,652 nodes). With reflections 8 of 57 cavities of `test_e113` now have a second filling.

## Results
* Second fillings (chiral): none at k = 1..4 (123, 352, 1,251, 4,909 connected sets; 70, 259, 1,087, 4,623 translation classes).
* With reflected tiles: 0 / 13 / 108 / 680 second fillings at k = 1 / 2 / 3 / 4. Every one changes exactly 2 tiles, replaces a pair whose outline is the Mystic 20-gon by its mirror dissection (two reflected tiles), and has (dV, dE, dF) = (0, 0, 0). It is therefore the reported 2-tile swap; it needs reflected tiles and is not a move of the strictly chiral Spectre. The k = 3, 4 cases are embeddings of the k = 2 swap (the extra tiles are unchanged).
* Six flips of the point cloud: all valid exact tilings before and after; none of their clusters closes (equal boundary edge sets) within the listing, so no (dV, dE, dF) of a closed flip cluster is available. Exact refills of their holes are unique chirally.
* Connectivity: worms (1,599 in the patch, none closed). A swap leaves the matching of worm ends unchanged (121 of 121 swaps, k = 2, 3) while the contact graph of the ring changes (121 of 121). So the swap is not a worm reconnection; no notion tried makes it one.
* Bit sign: for every vertex of every simple outline |Delta| < 2 pi (the corner turn lies strictly inside (-pi, pi)); largest per vertex 306 to 352 degrees in the family scanned (`log_bit_sign_table.txt`). Moving a vertex of a tiling with all its tiles gives sum Delta = 0 (B3); two congruent tiles of a pair give relative sign +1 (B4). Moving two vertices (P_0, P_4) of one isolated tile reaches sum Delta = 360 degrees with simple outlines (B5), but that is not a deformation of a tiling.
