# toymodel_v9: from flips to a model (plan, revision 0, draft)

Status: draft. Section 2 holds the author's Danzer ideas (second track), with what a first search did and did not confirm. Labels as in `quantgrav.tex`: **Exact**, **Counted/Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**.

## 0. Starting point (from toymodel_v8)

Established (Counted unless marked; class 0, level 3, step 0.03, box 20, `IN_T`/`OUT_T` = 0.0015/0.0035; see `../toymodel_v8/RERUN.md`):
- A flip is a cluster of tiles at uniform density (changed tiles in a disc grow as rho^2.25; 0.056 changed per present tile, a lower bound). The flip relation is symmetric under reversal (Exact), so flips alone give no arrow.
- The contact graph of the tiles meeting a footprint changes in 224 of 468 valid pairs (48%), non-isomorphic in 169 of them (75%), while the tetrad flux of the lift is zero for every flip.
- Area `n_O + 2 n_G` is conserved in a closed cluster, `n_G` is not (0.32 of 163 events at rho = 7). Parity frequencies are window areas: ratio `4 + sqrt(15) = lambda^2`.
- Erasure of one tile is never ambiguous (Negative for a Landauer-type cost from that test).

Not established: a closed cluster (needs region radius about 26), any `(dV, dE, dF)`, Pachner-move content, a flip rate that depends on level, any arrow of time, a link from `rho` to a clock.

## 1. Candidate stages (cheap and exact first; edit freely)

| stage | content | progress means | stop if |
|---|---|---|---|
| 0 | **Done** (`STAGE_A0.md`, `t9_engine.py`; contacts test only). **Inventory and interfaces.** Move the flip engine (`wada-coudene/figures/spectre_flip_*.py`) into one module here with `--tests-only`: box 20, tolerance, reversal audit, regression against the v8 numbers (468/224/169, 1934 pairs) | the module reproduces the v8 counts exactly | any v8 count changes |
| 1 | **Closed clusters.** Region radius about 26 (a long run: the author runs it locally and pushes JSON); then cluster type, tiles removed and added, `(dV, dE, dF)` | a table of cluster types with counts | no cluster closes at any step |
| 2 | **Done** (`STAGE_A2.md`): resolved down to delta 0.015, cluster size depends weakly on delta, elementary flip about two tiles at the origin. **Step scan.** delta = 0.015, 0.0075 with the tight tolerance: does a flip split into smaller flips, or is the cluster size independent of delta | cluster size against delta with a stated law | sizes are unresolved at the tolerance |
| 3 | **Move set.** Each closed cluster as an operation on the dual graph; compare with Pachner (bistellar) moves | a cluster type equals a named move or a short word of moves | a type is not a composition of local moves |
| 4 | **Other classes and level 4.** The 11 classes not yet run; classes 2 and 5 of the covering script with box 20 | the class-0 numbers are typical, or the differences are listed | class 0 is special (then say so in the paper) |
| 5 | **Inflation direction.** Flip rate, pattern entropy and cluster size against level k (delta -> lambda^(+-1) delta): is anything monotone in k beyond a change of units | a monotone quantity with a mechanism | everything is scale-covariant |
| 6 | **Clock.** Define `rho(x,t)` from a flip process (fraction of tiles changed per step) and test `Delta tau = Delta t0 / rho` against something other than its own definition | a prediction that can fail | `rho` is constant (it is, to the precision of v8, so a clock needs a non-uniform process) |
| 7 | **Phason dynamics.** Langevin motion of `z` against the `T^(alpha/2)` scaling; flip rate as a function of temperature | measured exponent with an error bar | no scaling regime |
| 8 | **Hull cohomology.** A map from the integer coordinate forms (non-exact, v8 H8.7) to cohomology of the hull | an explicit map, or a stated reason none exists | not definable |

## 2. The author's ideas for v9: a second track on Danzer tilings

Four ideas were proposed (2026-11-20). They are a second track (**B**) beside the Spectre flip track (**A**, sections 0 and 1). A search was done before writing this section; what it did and did not confirm is stated per idea. Nothing below is cited in `quantgrav.bib` yet (bib gate: only sources actually read).

**B1. Root-lattice grammars, A2+A2 to D6.** Encode a boundary by inner products of roots, as the Spectre plot does in R^4, and compare with the Danzer ABCK tiling, a cut-and-project model set from the root lattice D6 (the F-type icosahedral family) projected along H3 (confirmed: Frettloh's ABCK note; Koca et al., arXiv:2003.13449, which gives the ABCK vertices as fundamental weights of H3 and the tiling as rotations and translations in D6; both read only as abstracts or snippets). Questions with a test that can fail: does the same inner-product code determine the faces of A, B, C, K, and how does the local matching behave in the 4D to 2D scheme against the 6D to 3D one? **Definition first:** what "boundary code" means for a tetrahedron.

**B2. Discrete spacetime and noncommutative geometry.**
- Confirmed: the cohomology of the Danzer tiling has torsion; Gahler, Hunton and Kellendonk state it for icosahedral tilings including the Danzer one (arXiv:math-ph/0505048, arXiv:1202.2240; snippets only). Their sentence says no 3D icosahedral tiling is known with vanishing torsion in H_0. I did **not** confirm the claim that this is unlike "standard 2D projection tilings"; drop it unless a source is read.
- Not tested: whether the torsion changes spectral gap labelling (it is a K-theory statement; the link to gap labelling needs a source).
- Lorentzian cut-and-project (R^{3,1} or R^{2,1}) is **Hypothesis**; no source has been checked, and a causal set built from it needs a definition of the ordering and of the sprinkling density before any test.
- Link to track A: v8 H8.7 found four non-exact integer coordinate forms for the Spectre lift; the same computation for the Danzer lift is the natural comparison (hull cohomology, stage 8).

**B3. The 3D einstein problem and the screw axis.** Resolved at stage B0 (`STAGE_B0.md`): the arXiv entry 2609.19214 is Tsiokos's Chair44 paper, already cited in `quantgrav.tex`. It claims a strongly aperiodic 3D monotile in the sense of Mozes (only mildly aperiodic in a stricter taxonomy), unrefereed, Lean-checked modulo 21 compiler hooks. So B3 is no longer "an open problem": it becomes a comparison of the Chair44 hierarchy with the Spectre model-set picture, plus the SCD biprism (weakly aperiodic: screw motions allowed) as the weak case. The reading of the screw axis as a helical time axis for a 2D tiling remains an idea, not a result.

**B4. 7-fold Danzer triangle tiling.** Confirmed (Tilings Encyclopedia, Nischke and Danzer 1996): three triangle prototiles, substitution with inflation factor 1 + sin(2 pi/7)/sin(pi/7), edge lengths proportional to sin(pi/7), sin(2 pi/7), sin(3 pi/7), local matching rules from the list of vertex stars, and the encyclopedia says the inflation is **not a PV number**. That matters for track A: the Spectre inflation lambda^2 = 4 + sqrt(15) is Pisot (quadratic), so the window-fractal and diffraction questions differ. Candidate stages: B4a generate the tiling and check the inflation factor; B4b diffraction and the window shape (a non-Pisot inflation suggests the model-set description needs checking before use); B4c quantum walk or localization on the contact graph, with the Spectre graph as the control.

### How the tracks combine

| stage | track | content | stop if |
|---|---|---|---|
| B0 | B | **Done** (`STAGE_B0.md`): Chair44 paper read, ABCK inflation matrix checked (Perron root tau^3). Still to read: Gahler et al. beyond snippets | a claim here is not in the source |
| B1 | B | **Partly done** (`STAGE_B1.md`, `t9_danzer.py`): D6/H3 split, inflation matrix S (entries +-1/2, preserves D6), tile K, ABCK matrix. Cross-checked against Frettloh's ABCK note (matrix, Table 1 volumes, frequencies, densities). Table 1 vertices are exact lattice vectors in our frame (STAGE_B1 item 6); the window orientation, assignment and scale are still open (item 7). Class II window derived (STAGE_B1 item 9). Dissections of tau T found by exact cover, window attractor not yet resolved (`STAGE_B1b.md`). Still to do: the vertex set (windows in our frame) | counts disagree with the literature |
| B2 | B | Flip engine for the Danzer model set: phase move, tile-set change, contact graph; compare with Spectre (same four numbers as v8) | the window is not computable at depth needed |
| B3 | B | 7-fold tiling: inflation factor **done** (not PV, min. poly x^3-4x^2+3x+1); vertex stars and matching rules still to do | factor disagrees |
| B4 | B | Lorentzian scheme: definition only, then one test | no ordering can be defined |

Cheapest first: B0, B1, B3 are finite and exact. The flip comparison (B2) uses the engine of track A stage 0, so A0 comes first.

## 3. Hypotheses kept apart

- H9.1. A flip changes connectivity in a way that a graph-rewriting rule of quantum-gravity type can describe (this needs stages 1 to 3).
- H9.2. An arrow, if any, is the initial condition or the inflation direction, not the flip relation (v8 H8.3; stage 5).
- H9.4. The contact-graph rewiring seen for the Spectre (v8) is generic for model sets, not special to the Spectre windows (stage B2).
- H9.3. A clock rate can be read from a flip rate only if the rate varies in space or in time (stage 6).

## 4. Rules (as v2 to v8)

Define then compute; every stage has a test that can fail; a Negative is a deliverable; single-file modules with `--tests-only`; numbers in the paper only as macros of `models/paper_numbers.py` or marked with a dagger when copied from another run; hold off on runs that may take an hour (the author runs those locally and pushes JSON, figures and logs); earlier toymodels stay as they are; no commits unless asked.

```
research/toymodel_v9/
  PLAN.md
  t9_engine.py       Stage 0 (module and regression)
  t9_clusters.py     Stages 1 to 3
  t9_levels.py       Stages 4, 5
  t9_clock.py        Stages 6, 7
  t9_danzer.py       Track B (D6 to H3 ABCK, 7-fold, flips)
  STAGE0.md ...
```
