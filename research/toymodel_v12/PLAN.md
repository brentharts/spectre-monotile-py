# toymodel_v12: a closed substrate (the Hat Trick sphere) for the pair-qubit model (plan, revision 0, draft)

Status: revision 1 (2026-11-25). D1 = A and D3 = yes decided; S0, S1, S2, S5(a) done, see `README.md`. The original text is kept below; section 8 says what the results change. Labels as in `quantgravalt.tex`: **Exact**, **Numeric**, **Definition**, **Hypothesis**, **Negative**. Nothing here is a derivation of gravity.

## 0. Where v11 leaves the model, and why a sphere

`quantgravalt.tex` (v4) and v11 give an exact planar substrate and a Hamiltonian, and the list of what is missing is the paper's own:

| gap in v11 / `quantgravalt.tex` | where it is stated |
|---|---|
| No closed flip cluster has ever been reached, so no `(dV, dE, dF)`, no Pachner comparison. Planar flips are clusters of about 10 tiles that reach outside every window the engine can certify (radius 8.5 against about 26). | Results flip-clusters, flip-exact; v11 README |
| The only closed change found is the 2-tile Mystic swap `(0,0,0)`, and it needs reflected tiles. | Result swap |
| The contact graph changes in 75% of tile-changing flips, the pair count changes, and no rule makes this unitary. | Result flip-contact; qwla Stage 3 |
| Entanglement peaks at `tau_eff` 0.4 to 1.2 but does not grow with size; the unsatisfied weight `f` of the frustrated dipoles is not converged beyond 28 pairs (SA bounds only at 82, 804). | Results ent-flow, ham-dipole-large |
| Planar patches have a boundary, so `f`, entanglement and relative entropy all carry edge effects; the volume-law relative entropy may be one of them. | Result rel-volume |
| No curvature, no source, no horizon, no factor 2 of light bending; the substrate is flat. | abstract, Section toward-physics |

A closed surface removes the boundary and carries curvature. Two exact objects from the ballgen / hexasphere / qwla work are candidates:

* **The Hat Trick sphere**: 60 hats on the Goldberg polyhedron `GP(3,0)` with 12 pentagonal holes. The tetrahedral group `T` (order 12) acts freely on the 60 hats (5 orbits) and the icosahedral group `I` (order 60) is a symmetry of exactly 2 tilings. The tiling space is finite and enumerable by exact cover.
* **Closed Spectre folds** (qwla Stage 7): 237 random closed balls of 4 to 21 tiles, exact curvature ledger 720 degrees, cone deficits in {90, 120, 180, 240, 270} only.

The plan is built around the first. The reason is the largest gap: on a closed surface two different tilings differ in a set of hats whose connected pieces are closed clusters by construction, so the first `(dV, dE, dF)` table of closed flips can be computed.

## 1. What carries over from qwla (files, with results)

| qwla result | file | use in v12 |
|---|---|---|
| Hat Trick: 60 hats, 12 holes of 10-gon, deficit 60 degrees each, ledger 720 | `cone_ledger.py` | curvature ledger of the substrate (S5) |
| Only `GP(3,0)` carries a hat tiling with 12 pentagon holes for odd `T` up to 39; counts I 2, T 2, C5 16, C3 16, C2 64, C1 at least 1000 unexhausted | `gp_hat_scan.py` | the flip space of S2 is finite; the hat sphere is a single size, so **no RG ladder** on it |
| Dormant flip rule: unitary on register x presence, reversible, needs no conserved pair count | `flip_rule.py` (qwla Stage 3) | the channel of S4 |
| Gauge-invariant local pair qubits do not exist (invariant algebra of one pair is a commutative 2-dimensional algebra; `T`-decomposition of 12 punctures: 20, 14, 14, 28 of `1, 1', 1'', 3`) | `puncture_hilbert.py` | **do not** try to make the qubits gauge-invariant local objects; qubits stay a code on the tiling |
| Boundary one-tile moves change the cone count in 54% of 660 sampled pairs | `cone_moves.py` | on a closed sphere the cone data are fixed by the hole set, so curvature is not dynamical unless holes can move (S5) |
| Spin-`j` sphere, Trionda flow, Ehrenfest error 0.085 to 0.012 for `j` 10 to 80, `T`- and `I`-invariant Hamiltonians `H3`, `H6` | `spin_sphere.py` | optional ambient clock layer (S6) |
| Hat vs Trionda: `T` inside `I`, exposure CV 0.163 against 0.0089 | ballgen `compare_trionda_hat.py` | symmetry-resolved sectors of S3 |

## 2. A fact to settle first (it changes the design)

In the flow of v4, `h(tau) = tan(60 degrees (1 - tau))`, `J = exp(-h^2/2)`, `kappa = kappa_norm h^2`. At the Hat (`tau = 1`) the jump of the 14th vertex closes: `h = 0`, `J = 1`, `kappa = 0`. So the Hat Trick sits at the end of the flow where the bit is a free spin in a transverse field and the ground state is a product state. Two ways forward, to be decided (D1 below):

* **A. Decouple `h` from `tau`** on the sphere: keep the hat geometry fixed and treat `h`, `J`, `kappa` as independent parameters (a Definition). The couplings `W_AB` are then dipole sums over exact hat centroids on the sphere.
* **B. Follow the flow on a sphere**: deform the sphere along `tau`. The Hat Trick is not an exact isometric tiling already (kite relaxation: rms strain 2.1%, maximum 5.3%), so the deformation would be a relaxation problem, not exact arithmetic.

Recommendation: A for v12, B only if A shows something.

## 3. Stages

| stage | content | progress means | stop if |
|---|---|---|---|
| S0 | **Set-up.** `research/toymodel_v12/`, import qwla modules by path (as v11 does for `spectre_exact`), run the v11 tests, copy nothing | v11 and the qwla tests pass from this folder | a v11 or qwla test fails here |
| S1 | **The bit on a hat.** Define the flat vertex `P8` of each of the 60 hats on the sphere, find the pairs (two hats whose flat vertices meet alone, the rule of `find_pairs_keys`), per tiling and per hat orbit of `T` | a pair count `p` for the `I`-symmetric tilings and for the 64 `C2` tilings, and its distribution; a table by `T`-orbit | no tiling has a pair (then the bit must live on the Spectre fold side, S1b) or `p` is too large for exact work (then use symmetry blocks) |
| S1b | (fallback) **Closed Spectre folds as the substrate.** Pairs and cone points of the 237 qwla balls | a pair count and cone multiset per ball | none has a pair |
| S2 | **Closed flip graph.** Enumerate tilings of `GP(3,0)` by exact cover (all 2 `I`, 2 `T`, 16 `C5` or `C3`, 64 `C2`; a stated sample of `C1`, the search being unexhausted at 1000), take the symmetric difference of two tilings and split into connected components; for each component record `(dV, dE, dF)`, number of hats on each side, the contact-graph change and the pair-count change. A move is a minimal component | the first table of **closed** flip clusters, with sizes and Euler changes, and a statement of which are Pachner-type counts; compare 75% contact change (plane) | all differences are single global rotations or reflections (then there are no local flips on this sphere: a Negative) |
| S3 | **Pair-qubit Hamiltonian on the sphere.** `H = -J sum X_A - kappa sum W_AB Z_A Z_B` with dipole sums over hat centroids, chordal and great-circle distance both; exact diagonalisation within `T` or `C2` symmetry blocks up to about 28 qubits; ground-state entanglement, `f` unsatisfied, gap | `f(N)` and the concurrence curve on the closed surface, against the planar values (0.060 at `N` = 6, 0.209 at 28; peak 0.4 to 1.2); resolved by irrep of `T` (`1, 1', 1'', 3`) | the closed surface only reproduces the planar numbers (then the planar edge effects were not the issue) |
| S4 | **Flip channel on the sphere.** The dormant rule of qwla Stage 3 on every flip of S2: register of all slots of all tilings, presence flags; unitarity, flip then reversed flip, energy after the flip, ledger | a unitary on a fixed finite space with no boundary, the first one; and the energy cost of each closed flip | a flip needs a rule that is not a permutation of the presence flags (then it is a channel, not a unitary) |
| S5 | **Curvature as a dynamical variable.** (a) Light: the deflection of a geodesic passing a cone point of deficit `d` is exactly `d` in a flat cone metric, independent of the impact parameter; measure it on the relaxed Hat Trick (deficit 60 degrees) and on Spectre folds (deficits 90 to 270 degrees). (b) Mobility of curvature: can the 12 holes be moved by an exact cover with a different hole set on the same `GP(3,0)`? (`gp_hat_scan.py` only allows pentagon holes; here other hole sets are allowed.) | (a) a gravitational signature v11 does not have, deflection 60 degrees at a cone point, labelled 2+1; (b) a count of hole sets that tile | (b) no other hole set tiles: curvature is frozen, flips never change it, and the model stays a toy with a fixed background (a Negative on R3) |
| S6 | **Ambient space layer (optional).** Couple the qubit register to the spin-`j` sphere of qwla Stage 9a, with `H3` or `H6` as the space Hamiltonian, `j` about 10 to 20 | a coupled model whose classical limit is the Trionda flow; a clock that is a physical variable of the sphere | no coupling that respects `T` and keeps the pair ledger is natural (then S6 is dropped) |
| S7 | **Paper bookkeeping.** `paper_numbers.py` rows for S1 to S5; one new section in `quantgravalt.tex`; update the table of gaps (R1 local move set, R3 dynamic geometry, R5 curvature) | every number from a test | -- |

Cheapest first: S0, S1, S2 only reuse exact search that already exists. S3 and S4 are small numerics. S5(a) is a one-hour exact geometry computation. S6 is the speculative stage.

## 4. What counts as progress, stated modestly

v12 would supply, if S1, S2 and S4 succeed: a finite closed substrate with a finite flip graph, closed flip clusters with `(dV, dE, dF)`, and a unitary dormant-rule channel on it. That removes three of the gaps in section 0 (no closed cluster, boundary effects, no unitary), within a 2+1 geometry with 60 hats and one size. It does not give an action, a continuum limit, an arrow of time, a Newtonian force, or 3+1.

## 5. Hypotheses kept apart

* H12.1. The tilings of `GP(3,0)` are connected by local clusters, not only by global symmetry moves. S2 tests it; a Negative is a result.
* H12.2. The closed-flip `(dV, dE, dF)` are Pachner-type for some clusters. If the minimal clusters have large sizes, the planar finding (no small closed cluster) is confirmed.
* H12.3. The unsatisfied weight `f` on the closed sphere is smaller than in planar patches of the same size, because edges are gone. S3.
* H12.4. Flips never change the cone ledger on this substrate (the holes are fixed by the boundary condition), so curvature is non-dynamical here. S5(b) tests whether holes can move.
* H12.5. The qwla kill row 7 (flips change the number of cone points) is untestable on `GP(3,0)` with fixed holes, and tested only on Spectre folds (`cone_moves.py`: boundary moves change it).

## 6. Decisions needed before S1

* **D1.** A or B of section 2 (recommended A).
* **D2.** C1 sample size for S2: the search at 1000 tilings is unexhausted, so statements about all tilings are limited to the symmetric classes. A sample of a few thousand with a seed, or exhausting `C2` and `C3` only.
* **D3.** Whether S5(a) (light deflection at a cone point) is acceptable as a "gravitational signature", given that 2+1 gravity has no Newtonian force and the deflection is pure topology. Recommendation: yes, labelled that way.

## 7. Rules (as v2 to v11)

Define then compute; every stage has a test that can fail; a Negative is a deliverable; single-file modules with `--tests-only`; no compiled code and no SAT; numbers from qwla, ballgen and hexasphere enter only through scripts that recompute them (paths by environment variable: `QWLA_DIR`, `BALLGEN_DIR`, `HEXASPHERE_DIR`, `SPECTRE_DIR`); earlier toymodels stay as they are; no commits unless asked; hold files until told to present.

```
research/toymodel_v12/
  PLAN.md  README.md  Makefile
  t12_pairs_sphere.py    S0, S1, S1b
  t12_flip_graph.py      S2
  t12_hamiltonian.py     S3
  t12_flip_channel.py    S4
  t12_curvature.py       S5
  t12_ambient.py         S6 (optional)
  results_*.json  log_*.txt
```

## 8. Revision 1: what S1, S2 and S5(a) change

* H12.1 is settled (**Exact**): the tiling space is `{0,1}^12`, one bit per pentagon, and every flip is a closed 5-for-5 cluster on 40 kites. D2 is moot: all 4096 tilings are enumerated.
* H12.2: `(dV, dE, dF) = (-k, -k, 0)` with `k` the number of neighbours in state 1. Only `k = 0` is a Pachner-type count.
* S3 becomes a model on the icosahedron: qubits on edges with both ends 0 (at most 30), couplings from hat centroids. The all-0 tiling has 30 pairs and full icosahedral symmetry, so its Hamiltonian can be block-diagonalised by `I`.
* S4 becomes: 12 geometry bits and 30 qubit slots, presence `(1 - sigma_p)(1 - sigma_q)`. New option (**Hypothesis**): make the geometry bits quantum, `H_geo = -Gamma sum X_p`, so that the state is a superposition of tilings.
* S5(b) is still open: the holes are fixed here, so the 720 degrees are frozen.
* New question for S3: a flat vertex facing a hole (state 1) has no partner. Whether its bit can open with the hole as the area reservoir needs a Definition.

## 9. Revision 2 (2026-11-25): S3 and S4 done with quantum geometry bits

Decision: the 12 geometry bits are qubits with a tunnelling `Gamma` and a cost `g`. Results in `README.md`. H12.3 is refuted (`f = 0.363` on the closed sphere). New open items: an icosahedral symmetry basis to converge the joint model at 30 pairs; time evolution (does a flip spread as a light cone on the icosahedron?); a derivation of `Gamma` and `g`; S5(b).

## 10. Revision 3 (2026-11-26): time evolution at `kappa = 0`

Done (`t12_evolution.py`, 5 checks): ordered spreading of a flip (OTOC half-maximum at 2.75, 4.0, 4.5 for distance 1, 2, 3), no arrow (entropy even in `t`), local back-reaction of an excited slot on the rotors, and a global one at the symmetric point. Open: the same at `kappa != 0` (needs the symmetry basis), a larger closed substrate to fit a front (none exists for hats: only `GP(3,0)` tiles), S5(b), S6.

## 11. Revision 4 (2026-11-26): the joint model at `kappa != 0`, second pass

A symmetry basis cannot reach 2^42. Done instead (`t12_dressed.py`, 4 checks): CI(4) matter at every tiling with a convergence table against CI(6) and exact, and the joint model in the span of the 4096 dressed tilings. Result: two variational bounds, each better in one regime (dressed for slow flips, truncated-matter for fast flips); the geometry-matter entanglement is bracketed, not determined. Open: an ansatz containing both; S5(b); S6.

## 12. Revision 5 (2026-11-26): S5(b) done, Negative

`t12_holes.py` (3 checks). Holes are forced onto the 12 pentagons, hats never touch a cone point, no tiling mixes hands, and no complex within one edge flip (all 270) or in a sample of 600 multi-flip complexes has a hat tiling. H12.4 holds: curvature is not dynamical here. Consequence: on the hat substrate the only quantum geometry is the rotor register. A substrate with movable curvature has to come from elsewhere; the candidate already in hand is the closed Spectre folds (qwla `cone_moves.py`: one-tile moves change the cone multiset in 63% of 660 pairs), which is S1b of this plan.

## 13. Revision 6 (2026-11-26): S7, write-up

`quantgravalt.tex` has the new Section 8 (ten results, one figure, 19 pages, builds with no error or undefined reference). Not done in S7: macros from `models/paper_numbers.py` for these numbers (they are dagger numbers), and the same section for `quantgrav.tex`.

## 14. Revision 7 (2026-11-26): S1b, closed Spectre folds

`t12_folds.py` (4 checks): 2,709 folds in a window of 18 tiles, exhaustively; a connected move graph of 2,703 folds on which 55% of swaps change the cone multiset; a model in which the number of cone points has a quantum spread. Open: all gluings of a patch (not only the DP's first); a local definition of a move (adjacent tiles only) and an amplitude that is not uniform; larger windows; dipole couplings on a fold (needs an embedding or intrinsic distances); this result is not in `quantgravalt.tex` yet.

## 15. Revision 8 (2026-11-26): folds tightened, and in the paper

`t12_folds_local.py` (3 checks): all 8,136 gluings, local swaps, a definition-free measure (22% of local swaps must change the curvature), and the model on (patch, outcome) states (variance of the cone-point number 0.74 to 0.88). `quantgravalt.tex` has the subsection (three results; 20 pages). Open: a substrate with both movable curvature and many pairs; amplitudes from the geometry; larger windows.
