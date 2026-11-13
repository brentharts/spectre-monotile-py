# toymodel_v6: plan (revision 5, after Stage 4)

Aim: give the toy a quantum Hamiltonian and test it, in place of the classical history weights of v2 and v5. Labels as in `quantgrav.tex`: **Exact**, **Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**. Nothing here is gravity; a Hamiltonian written down is a Definition until something is derived from it.

## 1. The proposal and what is kept of it

The starting point is a three-part Hamiltonian, `H = H_hop + H_bit + H_constraint` (hopping on the tile graph; a field `h(tau)` and an Ising coupling on the bits; a penalty that enforces a constraint). Five corrections, each of which changes what is built:

1. **Tsirelson's bound is automatic.** Any state of any Hamiltonian on a tensor-product Hilbert space, read out locally, has CHSH at most 2 sqrt 2. The constraint term does not generate the bound, and a `g -> infinity` limit is not needed for it. The open question is the opposite one: whether the ground state violates 2 at all, where along the flow, and with which read-outs.
2. **The freeze is at the Spectre, not the Hat.** In v2 and in the paper, records sit at the Spectre end (bit readable, overlap `c` small); the Hat is the plastic end where the two values merge. `h(tau) -> 0` at the Hat means the bias vanishes and the bit is free, not frozen.
3. **The constraint the tiling supplies is the area ledger.** v4 found that a bit can open only on a bump-and-dent pair, `Q = s_i + s_j = 0`. That is a local conserved quantity and is used as the constraint term. A Wilson-loop form is Stage 3; it is not assumed to be the same thing.
4. **The area term drops out.** On a pair with `Q = 0` a uniform field `h(tau) sum sigma^z` is identically zero (Stage 0, E6.1). So `h(tau)` cannot enter as a field on the constrained bits; it enters through the tunnelling (the overlap of the two positions) and through the coupling between pairs.
5. **"A kernel derived from E8 distances" is a choice.** v4 found the pairs geometrically independent; any `J(r)` is put in. It is kept as a parameter and labelled Definition.

Also: a hopping Hamiltonian has a Lieb-Robinson bound, not the exact light cone of the leapfrog wave of v2 (Stage 2 states which).

## 2. Stages

| stage | content | size | stop if |
|---|---|---|---|
| 0 | one and two bump-and-dent pairs: the constrained Hamiltonian, the pair qubit, entanglement and CHSH along the flow (**done**, `STAGE0.md`) | S | the constraint and the coupling do not commute in any useful way |
| 1 | the pair graph of a real patch (`toymodel_v4/t4_patch.flat_pairs`): a transverse-field Ising model on the pairs with K from the geometry (**done**, `STAGE1.md`: dipolar, frustrated; the frozen end is a cat with no pairwise entanglement, so Stage 0's Spectre-end entanglement is a two-pair artifact; entanglement peaks mid-flow) | M | pairs share tiles, so the pair qubit is not defined |
| 2 | the hopping sector on the Spectre and Chair44 graphs: walk tensor of v3 against the actual spreading; Lieb-Robinson against the exact cone of v2 (**done**, `STAGE2.md`: Spectre isotropic and sub-ballistic, t^0.56; Chair44 packet cubic with the axes slowest, opposite to the hop count, and uniaxial about (1,1,1); no exact cone) | M | the band edge is not quadratic |
| 3 | a gauge form of the constraint: Z2 link variables on seams, a plaquette term, Gauss law = the ledger?; ground-state degeneracy on a patch (**done**, `STAGE3.md`: the complex is a disc and the theory is consistent, unique plaquette ground state, dual to an Ising model on vertices; the ledger is not a Gauss law but a Wilson line whose pair multiplies to the flux at P8; the plaquette limit has no two-link Bell pair) | L | no plaquette structure on the seam graph |
| 4 | coupling: a hopping particle whose amplitude depends on the pair qubits; phase rate against tau as a clock (the C7 question of v2 and v5 in Hamiltonian form) (**done**, `STAGE4.md`: Franck-Condon narrowing makes the overlap a rate; seam bits give c and pass C7 to first order, tile bits give c^2 and fail; a slow region delays a packet by the time of flight) | L | the clock is put in by the coupling and nothing is derived |
| 5 | a Bell test without post-selection: two distant pairs, read-outs that are defined on the tile, time evolution from a product state; comparison with the two-ended rule of v2 and v5 | L | only one read-out per pair exists on the tile |

## 3. Hypotheses kept apart

- H6.1. Tunnelling `Gamma = J0 c(tau)` with the overlap of v2's quantum bit. A choice of form.
- H6.2. Coupling `K ~ h(tau)^2` (two sources of strength h, v5 Stage 3). A choice of form in Stage 0; in Stage 1 it follows, with sign and range, from v5's Definition of a bit as a charge at the tile centroid.
- H6.3. A second read-out of the pair qubit (anything other than "which tile is the bump") exists physically. Stage 5 depends on it; nothing in the tile model supplies it yet.

## 4. Rules (as in v2 and v5)

Define then compute; a test that can fail in every stage; a Negative result is a deliverable; single-file modules with `--tests-only`; numbers in the paper only as macros of `models/paper_numbers.py`; earlier toymodels stay as they are.

```
research/toymodel_v6/
  PLAN.md
  t6_pair.py          Stage 0 (done)
  STAGE0.md
  t6_patch.py         Stage 1 (done)
  STAGE1.md
  t6_hop.py           Stage 2 (done)
  STAGE2.md
  t6_gauge.py         Stage 3 (done)
  STAGE3.md
  t6_clock.py         Stage 4 (done)
  STAGE4.md
  figures/
```
