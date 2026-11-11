# Stage 3: results (the arrow audit)

Labels as in `PLAN.md`. Nothing here connects to gravity, and nothing here derives an arrow of time. File: `tm2_arrow.py` (11 tests, about 4 s), figure `figures/tm2_stage3_arrow.png`.

```
python3 research/toymodel_v2/tm2_arrow.py --tests-only
python3 research/toymodel_v2/tm2_arrow.py --story
python3 research/toymodel_v2/tm2_arrow.py --no-show --save research/toymodel_v2/figures/tm2_stage3_arrow.png
```

## 1. What was built

An audit of every place where one direction of a timeline is treated differently from the other. Each input is switched off and reversed, and the observable that should follow it is checked. Added to the list of `PLAN.md` C5: input (i'), the base point of the prefix flux.

## 2. Results

| id | statement | label |
|---|---|---|
| E3.1 | The paper's action is reversal symmetric on every closed hexagonal timeline of 3 to 7 steps (proof in the docstring). On open timelines it is not: 6624 of the 7776 timelines of 5 steps differ, by a closed formula that depends on where the flux is counted from | Exact |
| E3.2 | At the level of the measure: neighbour agreement q(t) is mirror symmetric when the timeline is closed or the flux term is off; open with the flux term it is not (n = 9, λ = 0.5: q(0) − q(7) = −0.246). **The base point of Φ is an orientation input the plan had missed** | Exact |
| E3.3 | With closure, a pin at node m read backwards is a pin at n−1−m. The influence centroid lies on the far side from the pin (+0.147 for a pin at 2, −0.147 at 10, n = 13), and is 0 for a central pin | Exact |
| E3.4 | Damping: the centred scheme for the paper's wave equation retraces itself backwards only for γ = 0, or if γ is reversed too. Energy end/start: 0.142, 1.000, 7.450 for γ = +0.1, 0, −0.1 | Exact (finite scheme) |
| E3.5 | τ: mean flip tick minus centre is +4.846 on the chain, −4.846 on the reversed flow, 0 with flat angles; the present amplitudes are the same in all three | Exact |
| E3.6 | Handedness: mirroring the tile flips the sign of the turn at P₈ (−15° to +15° at τ = 0.5) and leaves the jump, area share and every cost unchanged. The chain's arrow does not know the handedness | Exact |
| E3.7 | Census of closed hexagonal timelines by whether reversal is a symmetry (table below). The Spectre family's outline word is oriented; its reversed turning word is the mirror image's, traversed backwards | Exact |
| E3.8 | The laws extend beyond the Hat: h odd, c even about τ = 1 | Exact (Definition D3.5) |
| E3.9 | **Records sit at the frozen end.** Reading at tick j recovers the bit of tick i with probability (1 + √(1−c_j²) cos Θ_ij)/2. Of two ticks, the more readable one holds the better record of the other. The Hat holds no record (0 bits); the Spectre end holds 10.67 bits about the other 12 ticks, at least 0.777 bit about each tick before the Hat. Nothing crosses the Hat on the extended flow | Exact (Definition D3.6) |
| E3.10 | Two radial readings of the overlap are exchanged by overlap ↔ Helstrom bias; under reading B, d² = −4σ² ln(1 − r_s/r), which tends to 4σ² r_s/r | Exact algebra of a Hypothesis |

Census (cyclic sense: the closed curve, forgetting where it starts):

| n | closed | rotation-reversible | mirror-reversible | oriented | oriented fraction |
|---|---|---|---|---|---|
| 2 | 6 | 6 | 6 | 0 | 0 |
| 3 | 12 | 0 | 12 | 0 | 0 |
| 4 | 90 | 66 | 90 | 0 | 0 |
| 5 | 360 | 0 | 120 | 240 | 0.667 |
| 6 | 2040 | 708 | 672 | 864 | 0.424 |
| 7 | 10080 | 0 | 1512 | 8568 | 0.850 |
| 8 | 54810 | 5922 | 6618 | 43488 | 0.793 |

## 3. Which input carries the arrow

1. **Three inputs carry an arrow, all put in by hand:** boundary data (a pin, or the base point of the flux), the sign of γ, the orientation of τ. Each observable follows its own input and mirrors exactly when the input is reversed. They live in three different models, so they are independent by construction; nothing couples them.
2. **Two inputs are binary data without an arrow:** handedness and scale ordering. Coupling either to a cost is a new postulate.
3. **An oriented closed timeline is generic, not special.** From 5 steps on, most closed timelines are not symmetric under reversal. The paper's problem 11 asked for an object with reversal not a symmetry; such objects are the majority. What is missing is a weight that distinguishes a timeline from its reverse, and the paper's action does not (E3.1).
4. **The record test disagrees with the chain's drawn orientation.** The chain runs Spectre (frozen, readable) to Hat (plastic, unreadable). Records sit at the frozen end. Two conventions are possible and they point opposite ways:
   - "later = the end that holds records": the flow runs Hat to Spectre;
   - "earlier = the frozen, determined end" (the chain, and `ARROW_OF_TIME.md` section 7): then the present cannot hold a record of the past in the register (0.11 to 0.13 bit per earlier tick at tick 11, 0 at the Hat), and the record has to be the past itself.
   The plan's condition "more than one input is needed and they disagree on direction" is met in this sense. **Negative** for a single arrow in the toy.

## 4. Consequences for the plan

- C5's table gains row (i'). The paper's formal section should say where its flux is counted from, because for open histories that is its arrow.
- C7 has a second reading (B: c = clock). A and B give opposite answers to "is the dense region frozen or plastic". Stage 5 has to derive τ(r) to choose; neither is supported yet.
- C9: the extension is implemented for the laws only. The dynamics through the Hat (what state comes out of rank 1) is not defined.

## 5. Not done

No external record store or environment; no coupling of the three models; the damping test is a 1D ring, not the paper's grid with ρ(x, y, t).
