# toymodel_v10

Stage L (Lockwood merge): James Lockwood's `toymodel_v9` pull request (fork `newjordan/spectre-monotile-py`, branch `feat/toymodel-v9-rigidity-pair-proof-dipole`, notes PDF "Spectre Tiling Upgrade for James", 2026-11-20) was **not merged**. It used C++20 and CaDiCaL/`drat-trim`. The parts that can be done in plain Python are rebuilt here; the rest is kept only as reported data. Name here: `toymodel_v10`, not v9 (v9 is Track A/B of this repo).

| Lockwood claim | status here | where |
|---|---|---|
| Result 36 proof (O/G collapse, 15-state Z[w]^4 seam automaton, Krylov dim 4, order-5 recurrence) and Result 20 closure at level 4 (66 directed / 53 canonical / 35 positive) | **Reproduced** (4 of 4 tests, 1.5 s, his module copied with names changed; pure Python, exact) | `t10_pair_proof.py` |
| Dipolar pair-qubit ground states, exact to N = 28 | **Reproduced independently** in numpy (meet in the middle, no Gray code, no C++). Checked against Result 17 (N = 6..14), brute force at N = 12, and his reported values N = 16..22. N = 24, 26, 28: f = 0.206186, 0.211077, 0.209125, deg 1, 0.6 s at N = 28 | `t10_dipole.py` |
| Dipole SA bounds, Level 3 (N = 82) and Level 4 (N = 804) | **Numeric, partly reproduced.** Ours: f <= 0.229310 (equal to his) at N = 82; f <= 0.2938 at N = 804 (his 0.2948). SA agrees with exact at N = 14, 20. At N >= 82 the best energy is hit by 1 restart of 32 to 128, so these are upper bounds, **not** converged, and "bulk plateau 0.295" is not established | `t10_dipole.py --full`, `results_dipole_py.json` |
| **Exact cavity completion without SAT** (`t10_cavity.py`, see below) | **Reproduced for the chiral part**, not for the reflection control | `t10_cavity.py` |
| SAT/DRAT cavity rigidity (his results, kept as data): no closed chiral flip cluster up to 300 tiles, 180 tiles DRAT-certified, chirality control (2-tile swap with reflections at k = 4), D_min = 20, 121/121 random erasure recovery, 3D Chair44 depth 3 | **Not reproduced.** Kept as reported data in `lockwood_reported/*.json` (his JSON, unchanged). Label **Reported** (dagger) in any paper text. His `t9_cavity_rigidity.py` and the dipole test only read those JSON files, so his "13 of 13 passing" is 4 recomputed tests plus 9 that check stored numbers | `lockwood_reported/` |
| LaTeX blocks | Copied unchanged as `LATEX_ADDITIONS_lockwood.tex`; **not** to be pasted into `quantgrav.tex` until the SAT-based Result blocks are relabelled Reported | |

## Things to settle before using the cavity result

1. It says no closed chiral flip cluster exists up to 300 tiles under exact arithmetic, while v8 and v9 (point cloud, step 0.03) find flips of up to 13 tiles per side that are never closed. These need not conflict (point-cloud flips may include non-valid tilings, or his search may assume a pinned collar and a given tile set), but this is the first thing to reconcile.
2. Done: `t10_cavity.py`, a propagation-only exact search (below).
3. The dipole N = 804 value depends on SA effort.

Run: `SPECTRE_DIR=/path/to/spectre make quick`. Sources of the unmodified originals: the fork branch above.

## Stage L2: cavity completion by propagation (`t10_cavity.py`)

Pure Python, no SAT. Corner-filling search on the exact level-4 patch (4401 tiles, all of one chirality, vertices in Z[zeta_12] as integer 4-vectors): take the open vertex with fewest candidate tiles, a forced move when only one fits, a branch otherwise; candidates come from the free direction sectors at the vertex, and exact overlap and edge-to-edge tests on Q(sqrt 3) (floats only for clear cases). Complete under the assumption that tilings are edge to edge (checked on the 18 central tiles). Area is the same for every tile, so a filling that avoids overlaps and uses k tiles covers the hole.

| test | result |
|---|---|
| `test_e110` patch is edge to edge, no overlaps (central tiles) | pass |
| `test_e111` erase one tile, 12 orientations | unique refilling, 24 nodes |
| `test_e112` centre cavities k = 10, 20, 40 | unique; nodes = k + 1 (every step forced) |
| scan (`log_cavity_scan.txt`) k = 80, 150, 300 | **unique**; 88, 170, 371 nodes; 459 s at k = 300 |
| `test_e113` reflected tiles allowed, 57 small cavities | original found; after the bug fix (see v11) **8 have a second filling: the 2-tile Mystic swap** (before the fix: none) |

Counted/Exact: for these cavities (the k tiles nearest the centre tile 2675 of this ordering; Lockwood's cavity selection may differ) the refilling is unique, found with no branching at all, which is the propagation-only version of his unrestricted "Engine A" result (his node count was 54k at k = 300, ours is 371, a different cavity and a different propagation rule). This reproduces his main rigidity claim (no closed chiral cluster up to 300 tiles) without SAT.

**Chirality control: superseded.** The first version of this search forced interior vertices of the cavity and so missed second fillings; with the fix (`bound` vs `track` in `Search`) the 2-tile swap is found. See `../toymodel_v11/README.md`. The chiral results above (unique fillings, k = 80/150/300) are unchanged by the fix (k = 300: 9,652 nodes, one filling).

Not done here: the relaxed-SAT defect D_min = 20 (needs a defect model, not an exact cover), random i.i.d. erasure (needs his pinned disc), Chair44 in 3D. Run: `make cavity` (about 1 minute); k = 300: `python3 t10_cavity.py --scan 300` (about 8 minutes).
