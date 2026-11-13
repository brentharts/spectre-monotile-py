# Stage 7: results (entropy and the arrow in a unitary dynamics)

Labels as in `PLAN.md`. File: `t7_entropy.py` (3 tests, about 1 minute; the figure about 3 minutes), figure `figures/t7_stage7_entropy.png`.

```
python3 research/toymodel_v7/t7_entropy.py --tests-only
python3 research/toymodel_v7/t7_entropy.py --no-show --save research/toymodel_v7/figures/t7_stage7_entropy.png
```

## 1. What was asked

- **(a) An audit:** does any term of the Hamiltonians of v6 break time reversal (complex conjugation in the bit basis)?
- **(b) A quench:** the 12 central pairs of the level-3 patch start in their frozen classical state, a product state in which every pair has a definite bump, and evolve unitarily. The entanglement entropy of half of them is the entropy that can grow. The initial condition is put in and named as the input.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.22 | **No time-odd term comes from the tile.** Every Hamiltonian of v6 is real in the bit basis: the tile Hamiltonian with the ledger, the pair qubits, the patch couplings, the polaron, the hopping. The gauge form uses X and Z only. The patch couplings are mirror invariant, so the Spectre's chirality is a parity, not a time reversal. A real family of Hamiltonians has Berry phases 0 or pi only (checked on a loop: 0), so moving the tile's parameters cannot supply a continuous time-odd flux | Exact, **Negative** |
| E7.23 | **The arrow points away from the initial moment, both ways.** For a real Hamiltonian and a real initial state, psi(-t) = psi(t)*, so S(-t) = S(t) exactly (0 to machine precision at every tau tested). Entropy grows into the past as into the future from the frozen state: the Janus-point structure the paper cites | Exact |
| E7.24 | **The growth depends on the flow.** From the frozen product state S rises from 0 at every tau. Its late-time mean is 0.17, 2.44, 2.90, 1.90 bits at tau = 0, 0.1, 0.3, 0.6 (Page value 5.28), peaking at 3.45 near tau = 0.2. It is low at both ends for different reasons: at the frozen end the eigenstates are nearly classical; at the plastic end the coupling between pairs vanishes and growth is slow (half the late value only at t = 14.5 for tau = 0.6, 0.04 bits by t = 40 at tau = 0.9) | Numeric (exact evolution, 4096 states) |

## 3. What this says

1. **The tile supplies no arrow.** Every term the model has built is time-reversal even, and the Spectre's handedness acts as a parity. The only time-odd term on record is the paper's signed-area term, which needs a flux that nothing in the tile provides. The scoreboard's arrow row stays open, now as a Negative.
2. **An entropy that grows does exist, and it is the initial condition's.** Starting from the frozen (Spectre-end) product state, unitary dynamics builds entanglement, and it builds it symmetrically in time. So the model has a thermodynamic arrow of the ordinary kind: it points away from a low-entropy state, in both directions, and which way is "forward" is decided by where that state is put. This replaces v4's counting entropy, which never grew along the flow, with one that grows in time.
3. **The frozen end keeps its records.** At tau = 0 the bits barely entangle (0.17 bits), because their eigenstates are nearly classical. That is the v2 and v3 statement that records sit at the frozen end, now from a unitary dynamics.
4. **What is put in.** The initial product state, the choice of half, and v6's forms for the tunnelling and the coupling.

## 4. Not done

Larger clusters (the 12-pair cluster is 4096 states, exact); a time-odd term added by hand (the area term as a Peierls phase) and its effect on S(t) - S(-t); thermal rather than pure initial states.
