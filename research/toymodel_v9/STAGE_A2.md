# Stage A2: step scan of the flip cluster size (and the stats port of A0)

Code: `t9_engine.py` (`Engine.stats`, regression flag `--stats`), `run_step_scan.py DELTA [NSA]`; data `step_scan_d{0.06,0.015,0.0075}.json`. Class 0, level 3, box 20, `IN_T/OUT_T` = 0.0015/0.0035, region radius 7 (3569 placements), 3500 random phases per step. All **Counted**.

## Port check (stage A0, second part)

`python3 t9_engine.py --stats` (about 11 minutes with the setup) reproduces the v8 disc-count and erasure run exactly: 2831 usable phases, log-log slope 2.250093, 163 area-conserving events at rho = 7 with P(Delta n_G != 0) = 0.319, and the 184 erasure rows identical. The contacts test (stage A0, first part) was already identical. So `t9_engine.py` now carries both v8 measurements.

## Result

| step delta | usable phases | changed fraction in disc rho = 7 | divided by delta | tiles changed per changed phase, rho = 1.5 | rho = 7 | slope | changed phases (rho = 7) |
|---|---|---|---|---|---|---|---|
| 0.06 | 2804 | 0.271 | 4.52 | 1.88 | 8.0 | 2.19 | 761 |
| 0.03 | 2831 (v8) | 0.130 | 4.34 | 1.81 | 7.5 | 2.25 | 369 |
| 0.015 | 2851 | 0.061 | 4.09 | 1.76 | 7.0 | 2.33 | 175 |
| 0.0075 | 2858 | 0.020 | 2.66 | 1.67 | 5.4 | 2.22 | 57 |

(0.03 row recomputed from the v8 JSON: mean changed / fraction changed.)

- **The step is resolved down to 0.015, not to 0.0075.** For a step small against the window the changed fraction should be proportional to delta. It is, to within 6% from 0.06 to 0.015 (4.52, 4.34, 4.09 per unit delta). At 0.0075 it drops to 2.66: a flip needs the phase to lie within delta of a boundary, and phases inside the ambiguity band (0.0015 to 0.0035 from a boundary) are discarded, so at 0.0075 the band is a large part of delta. The last row is therefore a resolution limit of the tolerance and of the depth-7 cloud (spacing 0.0012), and its 57 changed phases are few. Not evidence about flips.
- **The growth exponent does not depend on the step.** Changed tiles in a disc against its radius: slope 2.19, 2.25, 2.33, 2.22 (rho 2 to 7), consistent with the v8 value 2.25 at every resolved step.
- **A smaller step gives smaller clusters, but not down to one tile.** Per changed phase, the number of tiles changed within radius 1.5 falls slowly (1.88, 1.81, 1.76, 1.67) and within radius 7 from 8.0 to 7.0 across 0.06 to 0.015. So the elementary flip is a cluster of about two tiles at the origin and does not shrink to a single swap as the step halves; the radius-7 count includes tiles of more than one cluster at larger steps, which I did not separate.
- **Answers the plan's stop condition for A2:** cluster size does depend on delta, weakly; "single flips" are not reached at the resolved steps. Separating one flip from a combination needs the connected-cluster analysis (stage A1), not a disc count.

## Limits

One class, one region, one random stream (seed 11), no error bars on the table; the 0.0075 row is unresolved. The radius-26 closed-cluster run (stage A1) is not run: with the setup alone taking 35 s and 3569 placements at radius 7 taking 5 minutes per 3500 phases, the placement count at radius 26 is about 14 times larger, so I expect hours; this is for your machine.
