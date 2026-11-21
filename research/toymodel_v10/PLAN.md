# toymodel_v10: exact rigidity and pair qubits (plan, revision 0, draft)

Status: draft. Labels as in `quantgrav.tex`: **Exact**, **Counted/Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**, and **Reported** (a number taken from another author's output that is not recomputed here; paper text must mark it with a dagger).

## 0. Starting point

From toymodel_v8 and v9 (this repo): a phason flip of the Spectre is a cluster of tiles at uniform density; the contact graph changes in 48% of valid pairs; flips are symmetric under reversal, so no arrow; no closed cluster was found (point cloud, region radius 7 to 8.5); `t9_engine.py` carries the v8 numbers. Track B: Danzer ABCK model set, flips are 1.176-hops, re-tiling rule (B2b) unknown.

From James Lockwood's unmerged v9 pull request (notes PDF of 2026-11-20; fork `newjordan/spectre-monotile-py`), after the stage L work of this folder (`README.md`):
- **Exact, reproduced:** Result 36 proof (15-state Z[w]^4 seam automaton, order-5 recurrence) and Result 20 closure at level 4 (`t10_pair_proof.py`, 4 of 4 tests).
- **Exact, reproduced in numpy:** pair-qubit dipole ground states, N = 6 to 28, always unique up to the global flip (`t10_dipole.py`); f_unsat = 0.060 (L2, N = 6), 0.209 (N = 28).
- **Numeric, not converged:** SA bounds f <= 0.2293 (L3, N = 82), f <= 0.294 (L4, N = 804).
- **Exact, reproduced without SAT (chiral part):** the refilling of a cavity of k erased tiles is unique, k = 1 to 300, found by forced moves only (`t10_cavity.py`).
- **Not reproduced:** the chirality control (2-tile swap with reflections at k = 4), the D_min = 20 defect, random-erasure recovery, 3D Chair44 depth 3. Kept as **Reported** in `lockwood_reported/`.

## 1. Candidate stages (cheap and exact first; edit freely)

| stage | content | progress means | stop if |
|---|---|---|---|
| L0 | **Done.** Merge of the rebuildable parts (pair proof, dipole, cavity) | tests pass without compiled code | a count differs from the reported one (then list it) |
| L1 | **Reflection control.** Ask James for the erased tile list of his k = 4 swap (`exp1c_reflect_control_L4`), run it through `t10_cavity.py --reflect`; if no tile list exists, run his `spectre_cavity_sat.py` selection rule | his cavity gives a second filling in our search, or a stated reason it does not | the two programs disagree on the same cavity: find which is wrong before anything uses either |
| L2 | **Rigidity against flips.** Reconcile exact rigidity (unique refilling to k = 300) with the v8 flips (up to 13 tiles per side, never closed). Test: take the v8 flip samples that are *valid* in the exact sense (no overlap, edge to edge), feed the erased tiles to the cavity search and see whether any has a second filling | a table: v8 flips that are real tilings vs raster artefacts | v8 flips are all artefacts (then v8 section 4 must say so) |
| L3 | **Other cavity shapes.** Non-central, non-round and non-simply-connected cavities, other level-4 classes; cavity near the patch boundary; node counts with and without forced moves. Define a cavity by its erased set, not by "nearest k" | does any cavity ever branch (node count > k + 1)? a first branching cavity is the object of interest | none branches in a stated large sample |
| L4 | **Dipole convergence.** Raise SA effort at N >= 82 until the best energy is hit repeatedly, or give a different solver; then f_unsat against level (0.060, 0.229, 0.294 at L2, L3, L4 as first values, **Numeric**): is it monotone in level beyond a change of units (feeds v9 stage 5) | converged values with repeat counts, and a fit or a stated non-fit | best energy still found by 1 restart at N = 804 with more effort (then say the plateau is open) |
| L5 | **Quantum step.** Transverse-field Ising on the same pair graph, `H = -Gamma sum sigma^x + sum W_ij sigma^z sigma^z`, exact diagonalisation (sparse, numpy/scipy) for N <= 20: gap and ground-state overlap with the classical state as Gamma grows (Lockwood's Experiment E, small sizes only) | a gap curve with the classical unique ground state as the Gamma = 0 limit | no structure (a Negative is a result) |
| L6 | **Re-tiling rule for Danzer (v9 B2b).** The corner-filling search is a re-tiling rule for 2D tilings. Hypothesis: a 3D analogue (fill the open vertex of the Danzer vertex set with the ABCK tetrahedra that fit the free solid angle) re-tiles the shifted vertex set and so unlocks the v8-style contact-graph comparison | the 3D search finds a tiling of a shifted patch, or a stated vertex where no tile fits | the shifted vertex set has no tiling in a region (then the model set is not a tiling there) |
| L7 | **Relaxed defect.** Rebuild D_min = 20 as an exact problem: minimal number of unmatched edges for a hole refilled with at least one non-original tile. Needs a definition first (what counts as a defect edge) | a definition, then a number at k = 6 and 10 | the definition is not exact (then keep it Reported) |

Cheapest first: L1 and L2 only reuse what exists. L4 and L5 are small numerics. L6 is the one that connects to the Danzer track and to a general model.

## 2. What would count as progress toward a model of quantum gravity

Stated modestly. v10 supplies three exact ingredients for a discrete model and none of the dynamics:
1. **Rigid local geometry:** a hole in a chiral Spectre patch has one refilling (L0, L3). This is a statement of **local recoverability** (v8 H8 stage 7 hypotheses), not a result about gravity.
2. **A finite quantum degree of freedom per pair:** the bump-and-dent pairs with dipole couplings, with a frustrated unique classical ground state (L4) and a quantum extension (L5).
3. **A re-tiling rule** that turns a changed vertex set back into tiles (L6), without which flips cannot be compared with Pachner moves (v9 stage 3).

Not established and not claimed: any action, any dynamics of `(dV, dE, dF)`, any arrow, any link to a clock (v9 stages 1, 5, 6 remain open).

## 3. Hypotheses kept apart

- H10.1. The exact refilling is unique for every cavity of the chiral level-4 patch (tested to k = 300 for one family). A first cavity that branches would falsify it.
- H10.2. Chirality is what locks the cavity (Lockwood). Untested here: L1 is the test.
- H10.3. f_unsat approaches a bulk constant near 0.29 with level. Upper bounds only; L4.
- H10.4. The v8 flips that look like clusters are artefacts of the point cloud, not tilings. L2.
- H10.5. The corner-filling search extends to the Danzer model set. L6.

## 4. Rules (as v2 to v9)

Define then compute; every stage has a test that can fail; a Negative is a deliverable; single-file modules with `--tests-only`; no compiled code and no SAT (decision of 2026-11-21); numbers from the Lockwood notes enter the paper only as **Reported** with a dagger until reproduced here; hold off on runs that may take an hour (k = 300 cavity takes about 8 minutes here, more for larger sets, the author runs those locally); earlier toymodels stay as they are; no commits unless asked; credit J. Lockwood for the pair proof module (copied with names changed) and for the problem statements of the dipole and cavity work.

```
research/toymodel_v10/
  PLAN.md  README.md  Makefile
  t10_pair_proof.py    stage L0 (Lockwood's module, renamed)
  t10_dipole.py        stage L0, L4, L5 (numpy)
  t10_cavity.py        stage L0, L1, L2, L3, L6 (propagation search)
  lockwood_reported/   his JSON, unchanged, Reported
  LATEX_ADDITIONS_lockwood.tex   not for quantgrav.tex until relabelled
  log_*.txt  results_dipole_py.json
```
