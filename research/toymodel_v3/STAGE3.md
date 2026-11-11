# Stage 3: results (records as the compatibility rule)

Labels as in `PLAN.md`. File: `t3_records.py` (4 tests, about 10 s).

```
python3 research/toymodel_v3/t3_records.py --tests-only --story
```

## What was built

Stage 3 of v2 found a conflict: in the chain the register holds the better record at the frozen end, while the flow runs toward the Hat. There the register was its own memory, and one qubit holds at most one bit.

Here the records are kept outside the register, in a store:
- At each tick a record of the bit is written with the fidelity the bit has at that moment, `p_j = (1 + √(1 − c_j²))/2`, and is never erased. This is "the past is still here".
- A rewritten history must agree with the records: each tick it changes costs `(1 − p_j)/p_j`.
- This replaces the chain's ad hoc ledger tolerance δ. **Definition.**

## Results

| id | statement | label |
|---|---|---|
| E3.1 | **Records alone give an arrow.** With the amplitudes made flat (amplitude asymmetry exactly 0), going back is the expensive direction on the chain's flow (asym < 0 at every grid point) and the cheap one on the reversed flow. Rewriting a tick costs ≈ 0 where the bit is frozen and 1 where it is fluid. At k = 6, m = 1: −0.93 | Exact |
| E3.2 | **Records and amplitudes agree.** Together, the sign of the asymmetry is the amplitude sign at every grid point on both flows, and its size grows: −2.03 (amplitudes) → −2.96 (both) at k = 6, m = 1; −4.07 → −7.80 at m = 2. Records never reverse the amplitude arrow | Exact |
| E3.3 | The store grows on both flows: 1 bit per tick for the frozen ticks, then 0.98, 0.91, 0.73, 0.43, 0.13 and 0 toward the Hat on the chain's flow; the reverse order on the reversed flow; 10.17 bits in all on both | Exact |
| E3.4 | **The conflict of v2 is gone.** "Earlier = determined" (hard to rewrite) and "the past is well recorded" pick the same orientation, the chain's own (Spectre → Hat). "Later holds more records" holds on both flows, because the store only grows, so it does not choose. The reversed flow would make the past fluid and badly recorded | Exact |

## What it says

With an external store the toy's three notions of the past agree: determined, recorded and hard to change. They pick Spectre → Hat, with the frozen end as the past and the fluid end as the future, which is your picture in `ARROW_OF_TIME.md` §7.

The assumption doing the work is that records are written with the fidelity of the moment and never erased. That the store is physical (an environment), and what writing costs, are **Hypotheses**.

Records are a fourth arrow input, next to boundary data, damping and the amplitude schedule. Unlike the others, it points the same way as the schedule rather than being independent of it.

The plan's hope that records would *choose* the flow is met in this sense: given "the past is the determined part", records select one orientation, and the store's growth adds no conflict.
