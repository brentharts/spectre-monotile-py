# Stage 0 of toymodel_v8: reversal audit

**Result (Exact by construction, Counted on data): the flip relation is symmetric, so the topology-change statistic cannot be an arrow of time. Negative for "the flips of Section RECONNECTION are an arrow".**

Definition D8.1. A flip is an ordered pair of internal phases (z, z') whose present tile sets differ. The reverse flip is (z', z).

Exact. The present tile set at a phase is a function of the phase only, and "the region is tiled without overlap or hole" and "the contact graphs are (non-)isomorphic" are symmetric in the two phases. So (z, z') is valid iff (z', z) is, and a topology change forward is one backward. Every count of flips by type is therefore identical in the two orientations, and a quantity such as "fraction of flips that change the contact graph" has no direction.

Counted check. `spectre_flip_contacts.py` run on the same 1500 samples with the step reversed (`SWAP=1`), `NS=1500 DELTA=0.05 R2=7 DIAM=5.5`, class 0:

| | forward | reversed |
|---|---|---|
| unambiguous samples | 927 | 927 |
| samples with a change | 347 | 347 |
| valid tilings before and after | 149 | 149 |
| tile set meeting the footprint unchanged | 134 | 134 |
| changed, graph not isomorphic | 13 | 13 |
| changed, isomorphic, classes differ | 2 | 2 |
| (tiles meeting the footprint) before, after | e.g. (2,1) x 2, (1,3) x 1 | (1,2) x 2, (3,1) x 1 |

All type counts agree; the before/after tile counts swap order, as they must. This sample is half the size of the 343-pair run of `RECONNECTION.md` and has the same ratio (13 of 15 changed pairs against 20 of 30; no error bars were computed).

What an arrow would need (fixed now, before Stage 4):
1. An initial condition: a non-stationary distribution of local patterns relaxing toward the stationary one (v7 Stage 7 found the same for the quench).
2. A direction in the substitution: the inflation contracts internal space by 1/lambda, which distinguishes coarse from fine. Not tested.
3. A non-reversible rate: the transition rates between pattern classes violating detailed balance. Not found, not yet looked for (Stage 4).

Not shown: any of these three. Nothing in this stage supports a connection with quantum gravity.

Files: `spectre_flip_contacts_pattern_c0_ns1500.json` (forward), `spectre_flip_contacts_pattern_c0_swap.json` (reversed).
