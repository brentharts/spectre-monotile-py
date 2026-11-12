# Stage 0: results (a Hamiltonian for bump-and-dent pairs)

Labels as in `PLAN.md`. Nothing here is physics. File: `t6_pair.py` (5 tests, under 2 s), figure `figures/t6_stage0_pair.png`.

```
python3 research/toymodel_v6/t6_pair.py --tests-only
python3 research/toymodel_v6/t6_pair.py --no-show --save research/toymodel_v6/figures/t6_stage0_pair.png
```

## 1. What was built (Definitions)

One qubit per tile, `sigma^z = +1` convex, `-1` reflex. On the tiles:

```
H = - J sum_pairs sx_i sx_j + g sum_pairs (sz_i + sz_j)^2 - K sum_<A,B> sz_A1 sz_B1 - eps sum_i sz_i
```

The second term is the area ledger of v4 as a penalty. In the sector `Q = 0` a pair is one qubit (`|+->`, `|-+>`), and `H_eff = - Gamma sum tau^x - K sum tau^z tau^z` with `Gamma = J`. Along the flow: `Gamma = J0 c(tau)`, `K = K0 (h/h0)^2`, read-out visibility `1 - c^2`, with `c = exp(-d^2/8 sigma^2)`, `d = 2h`, `omega0 = 120` degrees, `sigma = 1`.

## 2. Results

| id | statement | label |
|---|---|---|
| E6.1 | `sx sx` flips both tiles of a pair, so it commutes with `Q^2`: the ledger sector is invariant **for every g**, and the spectrum of `H_eff` is contained in that of `H` to 1e-12. It is the low-energy sector once g is of order J. The uniform field vanishes identically in the sector | Exact |
| E6.2 | One pair cannot violate CHSH: the only one-tile observables that commute with the ledger are functions of `sz`, and with them the pair is a shared classical bit (E = -1, CHSH = 2) | Exact |
| E6.3 | Two pairs: concurrence `C = K / sqrt(4 Gamma^2 + K^2)`, largest CHSH `2 sqrt(1 + C^2)`, gap `sqrt(4 Gamma^2 + K^2) - K`. No state of 2000 random ones exceeds 2 sqrt 2 (largest 2.823) | Exact |
| E6.4 | Maximally entangled state with read-outs on six directions 60 degrees apart: CHSH = 5/2, the paper's hexagonal value | Exact |
| E6.5 | Along the flow (J0 = K0 = 1): C from 0.913 to 0, CHSH of the state from 2.708 to 2, observable CHSH from 2.574 to 0, below 2 after tau = 0.102. Gap 0.095 at the Spectre, 2 at the Hat | Exact given the Definitions |

## 3. What this says

1. **The proposal's Ising term and the tiling's constraint fit exactly.** No strong-coupling limit is needed: the Ising coupling within a pair is a tunnelling term of the pair qubit. This is the one place where the Hamiltonian is forced rather than chosen.
2. **The unit of quantum information is the pair, not the tile.** Inside a pair there is only a classical anticorrelation. Entanglement that a Bell test could see lives between pairs, and needs a coupling K between them and a second read-out of the pair qubit (H6.3, not supplied by the tile).
3. **The area term is invisible to the constrained bits.** The quantity that made the bit interesting classically (A = A0 +- a h) cancels on a pair. `h(tau)` survives only through the forms chosen for Gamma and K.
4. **Direction along the flow agrees with v5 Stage 1, with a quantum bound.** With the forms of D6.3 the violating window is at the Spectre end and closes toward the Hat, as for the classical two-wing rule, and never exceeds 2 sqrt 2. With visibility included it closes early (tau = 0.102).
5. **The entangled end is the fragile end.** Near the Spectre the gap is 0.095: the ground state there is nearly degenerate with its partner, so the "frozen, readable" end is also where the entangled state is least protected. A temperature or a noise scale is needed to say more (not done).

**Revised by Stage 1.** Items 4 and 5 hold for two pairs only. On a patch the frozen end is a many-body cat with no pairwise entanglement, and the entanglement peaks in the middle of the flow (`STAGE1.md`, E6.9, E6.10).

## 4. Not done

More than two pairs and a real pair graph (Stage 1); a derivation of the forms of Gamma, K and the visibility; thermal states; dynamics (the ground state is assumed, not prepared); the hopping and gauge sectors.
