# Stage 5: results (how fast a measurement acts on a history)

Labels as in `PLAN.md`. File: `t3_relax.py` (3 tests, about 1.5 minutes; Monte Carlo against exact marginals, so **Numeric**).

```
python3 research/toymodel_v3/t3_relax.py --tests-only
python3 research/toymodel_v3/t3_relax.py --story      # n = 21, about 4 minutes
```

## What was built

The history sum of v2 is sampled by local moves: Metropolis on a window of three consecutive letters, proposing a word with the same sum, so the timeline stays closed. These moves reach every closed timeline (checked for n = 5, 6, 7), whereas windows of two do not.

The chains start in exact equilibrium. A soft pin is switched on at the middle node at time 0. The response of each node is its shift toward the exact pinned marginal, as a fraction of the final shift.

## Results

| id | statement | label |
|---|---|---|
| E5.1 | The moves are ergodic on closed timelines (windows of 3: one class for n = 5, 6, 7; windows of 2: 2, 3, 2 classes). The dynamics keep the exact ensemble (unpinned chains stay within 0.03 of it) and end at the exact pinned marginals (within 0.15 at every node with a final shift ≥ 0.04) | Exact (ergodicity), Numeric |
| E5.2 | **Without the closure constraint there is a front.** The time to reach half the final shift grows with distance: 10, 14, 18, 18, 24 sweeps for d = 1 to 5 (n = 13), about 3 sweeps per site. That is a ripple, and the influence it carries decays geometrically | Numeric |
| E5.3 | **With closure the far end does not wait for a front.** n = 13: d = 4, 5, 6 reach half at 91, 85, 72 sweeps, against 54 at d = 1. n = 17: d = 6, 7, 8 at 181, 176, 138, against 92 at d = 1. The far time is about the same for every far node and grows with the size of the history: the far mean goes from 82 to 165, a ratio of 2.0 against (17/13)² = 1.71. That is consistent with one collective, diffusive mode of the constraint | Numeric |

## What it says

Both readings of the proposal's "ripple" fail for a closed history:
- **In the ensemble,** the influence of a measurement is global (Stage 1 of v2).
- **In time,** it builds up locally near the pin, and far from the pin on one time scale set by the length of the history, not by the distance.

A front exists only when the constraint is dropped, and then it carries almost no influence to the far end. With closure, the history responds as one object.

Not shown: the scaling beyond n = 17, and dynamics other than local Metropolis moves.
