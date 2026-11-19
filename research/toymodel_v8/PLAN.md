# toymodel_v8: flips of the contact graph, and what an arrow would need (plan, revision 1)

Status: Stage 0 done (`STAGE0.md`); Stage 3 attempted and unresolved (`STAGE3.md`), blocked by Stage 1 (validity). Labels as in `quantgrav.tex`: **Exact**, **Counted/Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**.

## 0. Starting point (what is established, and what is not)

Established (`../../RECONNECTION.md`, Counted, one class, one footprint, 343 valid phase pairs): when the internal phase moves by 0.05, a Spectre tiling changes by a cluster of tiles. In 30 of the 343 pairs the set of tiles meeting the footprint of a fixed tile changes; in 20 of the 30 the contact graph of those tiles is not isomorphic before and after, in 10 it is isomorphic but the tile classes differ.

Not established, and stated here so that the model starts honest:
- **No arrow of time has been found.** The pair (z, z') is a flip forward and (z', z) the same flip backward, so every statistic of "fraction of flips that change topology" is symmetric under reversal by construction. An arrow needs an asymmetry that the flip relation does not have: an initial condition (v7 Stage 7), a coarse-graining direction (the inflation, contraction 1/lambda), or a non-equilibrium state. Stage 0 checks the symmetry numerically.
- **No link to quantum gravity has been shown.** The 20 cases support the Hypothesis that a flip can change the connectivity of the tile contact graph. That is not reconnection in the sense of Comisso and Asenjo (no F, R or transport velocity). Whether the flips are moves of a known quantum-gravity type (Pachner moves, spin-foam moves) is a question for Stage 3, and a Negative is possible.
- The method discards 397 of 740 changed samples as not valid tilings of the region; the reason is not known (Stage 1).

## 1. Stages (cheap and exact first)

| stage | content | what would count as progress | stop if |
|---|---|---|---|
| 0 | **Reversal audit.** Run the flip statistics for z -> z' and z' -> z on the same samples; the counts must agree exactly. State what an arrow would need (initial condition, inflation direction, non-equilibrium state). | the symmetry is Exact by construction and confirmed on data; the list of candidate asymmetries is fixed before Stage 4 | the two orientations disagree (then there is a bug or a real asymmetry; find which) |
| 1 | **Validity.** Why do 397 of 740 changed samples fail to tile the region? Test: tolerance (IN/OUT thresholds), the domain of phases on which the 12 windows tile, the lattice coset. Report the invalid fraction against tolerance and against the position of z in internal space | invalid fraction goes to 0 with tolerance, or is identified as a phase region where the window system is not a tiling | the invalid fraction does not move with tolerance and has no geometric description |
| 2 | **Closed clusters.** Larger region (R2 about 11) so that clusters close; footprint equality, number of tiles removed and added, linkage of outside tiles through the cluster, for several classes and footprints and for several steps DELTA (0.01 to 0.05) to separate single flips from combined ones | a table of cluster types with counts, and a step size at which clusters are single | no closed cluster exists at any step (then flips are not local) |
| 3 | **Move set.** Describe each closed cluster as an operation on the dual graph (tiles = nodes, shared edges = links) and ask whether it is a composition of standard local moves (Pachner or bistellar moves; vertex splitting). Definition first, then count | each cluster type matches a named move or a short word of them | a cluster type is not a composition of local moves |
| 4 | **Markov chain on local patterns.** States: isomorphism classes of the contact graph around a footprint (with and without tile classes). Rates from the boundary measure of the windows. Test: detailed balance with stationary distribution = window area; relative entropy to the stationary law falls monotonically from any start (H-theorem of a reversible chain) | an exact statement that the arrow is the initial condition, with its numbers; or a violation of detailed balance, which would be a real asymmetry | the chain is not reversible and the violation is not a tolerance artefact (then find the source) |
| 5 | **Inflation direction.** The substitution contracts internal space by 1/lambda. Compare pattern statistics at footprint scales lambda^k: entropy per tile, number of distinct patterns, flip rate. Is any of them monotone in k in a way that is not just a change of units | a monotone quantity with a stated mechanism | everything is scale-covariant (then inflation gives no arrow either) |
| 7 | **Parity sectors and erasure codes (from qwla).** Orientation parity of patches and windows (done: E8.1 to E8.5, N8.1; hypotheses H8.4 to H8.7 run in `STAGE7B.md`: one supported with small n, one supported, one negative, one open) | exact parity frequencies reproduced; parity change per flip measured | parity counts disagree with the note (they do not: E8.3) |
| 6 | **Link to gravity, only if Stage 3 succeeds.** Compare the move set with the v7 gravity entries (horizon, space factor) and with the exchange picture of `RECONNECTION.md`; define a transport velocity on the tiling if one exists | a defined F, R and u for the tiling, or a precise reason none exists | no definition is available (then say so in the paper and stop) |

Order: 0, 1, 2, 3, then 4 and 5, then 6; Stage 7 (qwla) runs alongside.

## 2. Hypotheses kept apart

- H8.1. The tile contact graph is the right object for connectivity. Alternatives: the vertex graph, the dual graph with edge labels.
- H8.2. A step of 0.05 in internal space is one flip. Stage 2 tests this.
- H8.3. An arrow, if there is one, comes from the initial condition or the inflation direction, not from the flip relation.

## 3. Rules (as v2 to v7)

Define then compute; every stage has a test that can fail; a Negative is a deliverable; single-file modules with `--tests-only`; numbers in the paper only as macros of `models/paper_numbers.py`; earlier toymodels stay as they are. The flip engine of `wada-coudene/figures/spectre_flip_contacts.py` is the starting point; Stage 1 moves it into a module here with tests.

```
research/toymodel_v8/
  PLAN.md
  t8_flips.py        Stages 0 to 2 (engine, reversal audit, validity, closed clusters)
  t8_moves.py        Stage 3
  t8_chain.py        Stage 4
  t8_inflation.py    Stage 5
  t8_parity.py       Stage 7 (E8.1 to E8.5)
  spectre_flip_stats.py  Stage 7b (disc counts, erasure ambiguity)
  STAGE0.md ...
```
