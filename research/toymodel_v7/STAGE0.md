# Stage 0: the scoreboard

File: `t7_targets.py` (1 test; the table in seconds, `--full` recomputes the recorded entries in minutes). Each missing feature of `PLAN.md` is one number with a target and the stage that works on it. Every later stage updates its row.

```
python3 research/toymodel_v7/t7_targets.py
python3 research/toymodel_v7/t7_targets.py --tests-only
```

| entry | value now | target | status | stage | source |
|---|---|---|---|---|---|
| light-bending coefficient (r_s / b) | 1.00 | 2 | open | 4 | v3 `t3_optics`, clock only |
| seam-rate clock vs sqrt(1 - r_s/r) at 2 r_s | 0.101 | 0 | open | 6 | v6 E6.26 (first order only) |
| overlap at r = r_s (horizon) | 0.607 | 0 | open | 6 | v6 E6.26 |
| CHSH, two pairs, tile settings, visibility, no discards | 2.454 | > 2 | **met** | 1 | `t7_bell` E7.2 |
| CHSH, patch clusters, tile settings, visibility | 1.988 | > 2 | open | 1 | `t7_bell` E7.4 |
| largest CHSH of any state | 2.828 | <= 2 sqrt2 | met (automatic) | 1 | v6 E6.3 |
| quadratic subfields shared by the inflation field and Q(zeta_30) | 0 | >= 1 | open | 2 | paper `res:field` |
| layouts of the 192 tips passing the rules | - | exactly 1 | pending | 3 | |
| fourth-moment index on the Chair44 (isotropic 0.6) | 0.477 | 0.6 | open | 5 | v6 E6.14 |
| spreading exponent on the Spectre (ballistic 1) | 0.559 | 1 | open | 5 | v6 E6.13 |
| grid cells where the bits' entropy grows | 0 | > 0 | open | 7 | v4 `res:flow-entropy` |
| T-odd terms not put in by hand | 0 | >= 1 | open | 7 | v6 terms are T-even |
| inflation block map closes | - | yes | pending | 8 | |

The "met" for Tsirelson is not progress: it holds for every quantum state, and says nothing about the model.
