# Stage 2: results (a weight odd under reversal; the pass rate and the violation)

Labels as in `PLAN.md`. Files: `t3_area.py` (4 tests, about 2 s) and `t3_passrate.py` (3 tests, under a second).

```
python3 research/toymodel_v3/t3_area.py --tests-only --story
python3 research/toymodel_v3/t3_passrate.py --tests-only --story
```

## 1. A weight that tells a timeline from its reverse (plan 3.1, 3.2)

The signed area K of a closed hexagonal timeline (an integer: twice the area in units of √3/4) is added to the action as θK.

| id | statement | label |
|---|---|---|
| E2a.1 | K is odd under reversal and under the mirror, even under the six rotations and under a shift of the start, on all closed timelines of 3 to 7 steps. `A_θ(Rs) − A_θ(s) = −2θK(s)` exactly: **the area term is a weight that tells a timeline from its reverse** | Exact |
| E2a.2 | Timelines with K ≠ 0: 12 of 12, 24 of 90, 360 of 360, 1020 of 2040, 10080 of 10080, 34920 of 54810 (n = 3 to 8). Every odd closed timeline encloses area. Of Stage 3's oriented timelines, the term misses 216 of 864 (n = 6) and 12960 of 43488 (n = 8), which enclose zero area. It also separates timelines whose reverse is their mirror image (120, 372, 1512, 4392 at n = 5 to 8) | Exact |
| E2a.3 | ⟨K⟩ is odd in θ. `P_θ(Ms) = P_θ(Rs) = P_{−θ}(s)` for every s: **the arrow flips with the mirror image of the whole world.** With θ = θ₀ε, ε the handedness of the tiling, one binary datum fixes both; the sign of θ₀ is a constant of the law. At θ = 1 (n = 6) 99% of the weight is on one orientation | Exact |
| E2a.4 | Counting the flux symmetrically from both ends removes the orientation of open histories: the symmetric action is reversal symmetric on all 7776 open 5-step timelines, against 6624 asymmetric ones for the paper's action | Exact |

**What it says.** Open problem 6 asks for a weight that tells a timeline from its reverse. One exists, and it is the most natural odd term on a planar walk: the enclosed area, the form of a magnetic flux through the history.

- **It does not come for free.** θ is chosen, and the term ties the order of events to the orientation of the plane. A world and its mirror image have opposite arrows.
- **What the arrow becomes.** It is no longer "put in by hand, independently three times". It becomes the chirality of the substrate, given one constant of the law.
- **What is still missing.** A reason for the term, and a coupling of the chain's bit to K.
- **The paper's own arrow on open histories (from the base point of Φ) is a convention.** It goes away when the flux is counted from both ends.

## 2. The pass rate is not what makes the violation (plan 4.1)

The two wings are read out at each end in one of three ways:
- **pin:** accept only the two letters on the setting's axis;
- **sign:** accept everything and read sgn(a·s);
- **soft:** accept with probability cosh(μ a·s)/cosh μ.

| read-out (Alice, Bob) | post-selected CHSH (β = 1, 2, 5; n = 1) | pass-rate contrast | max CH (local bound 0) | Alice accepts |
|---|---|---|---|---|
| pin, pin (v2) | 2.373, 3.520, 3.980 | 0.35 to 73 | −0.0088 at most | 1/3 |
| **pin, sign** | 1.662, **2.847**, **3.921** | **0** (1e-15) | −0.173 at most | 1/3 |
| soft (μ = 3), sign | 1.357, 2.245, **2.951** | **0** | −0.139 at most | 0.489 |
| sign, sign | 1.085, 1.677, 1.982 | 0 | −0.004 at most | 1 |

- **E2b.1 (Exact). A constant pass rate does not remove the violation.** With Bob accepting everything the pass rate is exactly independent of the settings, and the post-selected CHSH still exceeds 2√2. The problem note's "pass-rate leak" is not the cause.
- **E2b.2 (Exact). The Clauser–Horne combination never exceeds 0, in every mode.** The tables come from a local stochastic process with a local acceptance at each end. The violation is the one any local model produces when it may discard runs (the detection loophole). Alice keeps one history in three under pins, and 0.489 under the soft read-out.
- **E2b.3 (Exact).** No marginal depends on the far setting.

**What it says.** The two-ended conditioning of v2 has two readings, and the tables cannot tell them apart:

1. **The rejected histories are runs that happened and were not recorded.** Then the toy is a local model with inefficient detection, CH holds, and nothing is retrocausal. **Negative** for "the conditioning is retrocausal" as a property of the data.
2. **The rejected histories do not exist.** The ensemble is the set of histories that meet both ends. Then CHSH > 2 is a genuine two-ended effect, and Z(a, b) is not a frequency of anything, so the pass rate cannot leak.

What separates the two readings in an experiment is a test without discarded runs, or the CH inequality. This makes the paper's "retrocausal" precise: it is reading 2, and it is an assumption about which histories exist.

## 3. Consequences for the plan

- 4.1 is answered: a settings-independent pass rate is possible and does not bound CHSH. Under reading 1 the toy is local.
- 4.2 (a read-out in which the setting acts on the state) is now the only route to a quantum bound. Under reading 2 it should be built with no post-selection at all, so that CH and CHSH coincide.
- 3.1: the area term exists and is exact; coupling it to the chain, and to the tiling's handedness, is Stage 3's business together with the records.
