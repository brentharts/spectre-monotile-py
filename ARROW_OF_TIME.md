# The arrow of time in the Spectre–Hat toy models

Status: working note. Labels follow `QUANTGRAV_PLAN.md`: **Exact** = checked by a self test in this repository, **Definition** = a choice made in the toy, **Hypothesis** = a physical reading that nothing here tests, **Not explained** = something the toy needs but does not provide. Nothing here is a derivation of anything in physics; "gravity", "quantum" and "time" are borrowed words for parts of a small model (one bit, one vertex, one flow parameter).

## 1. The question and what this note does

Time moving forward is usually taken for granted from where we sit, as matter. Explaining it by entropy or phase-space volume needs a special initial condition and, many would say, is not a complete theory. The question here is narrower and can be asked of a model: *if there are two connected structures, one smooth and nearly static and one discrete and dynamic, what asymmetry between forward and backward does that give?*

The short answer from the toy models:

* **Exact:** given a smooth flow with a direction, going forward is free and going backward is possible but costly in three separate ways (amplitude, reach, search). Section 3.
* **Not explained:** why the smooth flow has that direction. The toy puts the direction in by hand. Section 4.

So the toy does not say why the arrow points forward for us. It replaces "why forward?" by a more specific question: *why does the smooth structure have an orientation relative to the discrete one?* Section 6 lists what could be tried next.

## 2. The picture and where it lives in the code

```
 structure 1 (smooth surface)    tau: 0 ----------------------------------------> 1
   stretches and bends slowly,           Spectre (14 corners, curved)        Hat (13 corners, straight)
   bits cannot bend it

 structure 2 (discrete bit at P_8, the 14th vertex)
   tick:  0   1   2  ...  j  ...  now              each tick: tunnel (flip) with a small angle
          b0  b1  b2     bj      bnow    <-------- may be rewritten, if compatible with structure 1
```

| idea | in the toy | file |
|---|---|---|
| smooth surface, flow `tau` | the tile of the Spectre → Hat flow; every edge except 7 and 8 depends on `tau` only | `spectre_hat_collinearity_flow_model.py` |
| the 14th vertex as the only place where both meet | P₈, flat in the Hat; its position along the chord normal carries one bit | same, `spectre_hat_time_model.py` |
| two arrows | tangent arriving at and leaving P₈, measured on the outline; angle = the turn `ω(τ)` | `spectre_hat_time_model.py` |
| "time can snap" | classical: flip the present bit or rewrite an earlier tick under rules R1–R5 | `spectre_hat_time_model.py` |
| two amplitudes | the two bit values are Gaussian packets of width σ at ±h(τ); overlap `c = exp(−d²/8σ²)`, `d = 2h` | `spectre_hat_quantum_bit_model.py` |
| quantum backward snap | a sum over *histories* (at which ticks the bit tunnelled), with a compatibility condition | `spectre_hat_backward_snap_model.py` |

## 3. What the toy establishes (all Exact)

Run `python3 <file> --tests-only` for each file; 43 tests in total, all passing.

### 3.1 Forward is free, backward costs three ways

Take the default quantum run (12 ticks, ω₀ = 30°, σ = 0.05) and a reference history in which the bit tunnelled once, at tick 10. Figure: `figures/spectre_hat_backward_snap.png`.

1. **Amplitude falls with depth.** A flip is a history in which one tick toggles; its weight goes as `tan²(θⱼ/2)` with `θⱼ = Ω c(τⱼ)/N`, so it falls with the overlap `c(τⱼ)` squared, and the overlap is exponentially small early on. Given that exactly one flip happened, the probability that it was at tick 11, 10, 9, 8 is 0.54, 0.30, 0.12, 0.03, and at tick 6 it is 6·10⁻⁴. Relative to the reference history (flip at tick 10), a compatible history with the flip first at tick 9 has relative probability 0.38, at tick 8 about 0.1 or less, and at tick 1 about 6·10⁻²³ (table in `--story`; values depend on δ because compatible histories include bubbles).
2. **Reach is limited by how well the surface remembers.** The surface has integrated the area the bit added (`Q = Σ bⱼ wⱼ`, `wⱼ = h(τⱼ)`). A compatible history must keep `Q` within a tolerance `δ`. The earliest tick a compatible history can start at moves back as `δ` grows: tick 10 for δ = 0.03, 9 for 0.06, 8 for 0.10, 7 for 0.15, 5 for 0.25, 1 for 0.40. With a tight memory the past is frozen.
3. **Search cost.** Following the flow forward takes n steps. Listing the compatible histories examines 2ⁿ patterns (4096 against 12 here).

### 3.2 The present does not store when the snap happened

The tunnelling operators commute, so the final amplitudes depend on the total angle `Θ = Σ θⱼ` only: any reordering or redistribution of the angles over the ticks gives the same present state. In this model the past tick at which a flip occurred is not encoded in the present, which is the exact sense in which "the past can be rewritten without the future noticing". Compatibility conditions are what limit it (3.1).

### 3.3 Coherent histories beat independent flips

The quantum flip probability `sin²(Θ/2) = 0.093` is 3.6 times the value for independent toggles (0.026). The difference is the cross terms between different ticks. A history with toggles at two ticks (a *bubble*, the bit reversed between them) has relative amplitude `−tan(θⱼ/2) tan(θₖ/2)`: second order, and with the sign that interferes destructively with the history without toggles.

### 3.4 Early ticks are frozen, late ticks are plastic

The tunnelling angle is proportional to the overlap, which is exponentially small while the two tiles are far apart. The bit is read almost perfectly there (readout bias 0.9 at τ = 0.75) and almost never snaps; near the Hat the overlap goes to 1, the bit becomes unreadable (probabilities ½, ½) and snaps are order one. The classical model has the same shape as a hard cutoff (`snap_scale`); the quantum model smooths it. Figures: `figures/spectre_hat_time.png`, `figures/spectre_hat_quantum_bit.png`.

### 3.5 Reversing the flow reverses the asymmetry

`BackwardSnap.reversed_flow()` runs the same bits on the flow the other way. The present state is unchanged (3.2) and the flip-time distribution turns from late-peaked to early-peaked. **The asymmetry belongs to the orientation of the smooth flow, not to the bits.**

## 4. What the toy does not establish

1. **The direction is an input.** The laws `ω(τ) = ω₀(1 − τ)`, `h(τ)`, `k(τ)`, `b(τ)` run one way because they were written that way. Rule D6 of `spectre_hat_time_model.py` forbids stepping back. Section 3.5 shows that choosing the other orientation moves the cheap ticks to the other end. The arrow in the toy is therefore assumed, and the cost asymmetry is a consequence of it, not an explanation of it.
2. **General relativity does not supply the direction.** The field equations are invariant under reversing time, and a time-reversed gravitational wave is also a solution. A smooth surface that "stretches and bends" does not by itself prefer a direction. In the toy the surface's stiffness is also a rule: no bit can change any edge except 7 and 8 (rule R4), and the surface's response to a bit is only the area `a·h` it adds. A finite, energy-dependent stiffness is not modelled.
3. **The toy's direction points toward forgetting.** Along `τ` the two bit states merge (register dimension 2 → 1, readable fraction 1 → 0). Read as entropy that is a *decrease* of the register's distinguishable states, the opposite of the usual thermodynamic arrow, unless the erased information goes somewhere. The flow is non-unitary (the odd sector, norm² `1 − c`, is annihilated at the Hat) and the model does not say where the information goes. A full account would have to; this is the place where the toy and the entropy arrow would meet.
4. **Superposition and backward snaps are separate features.** Two amplitudes come from Definition Q1 (two overlapping packets). They do not come from time reversal. The toy does not show that "in many places at once" is a consequence of backward steps.
5. **The scale story is one parameter.** What separates the frozen regime from the plastic one is the ratio `d(τ)/σ`. That is a single dimensionless number, not a derivation of why the universe has different scales, and the toy has one bit, not a macroscopic system.
6. **"Needs computation" is a statement about a search.** The 2ⁿ count is exhaustive search over histories. For generic weights, finding a set of flips that keeps a sum fixed is a subset-sum problem, hard in general; the weights here are smooth and structured, so cheaper methods exist. Whether nature does anything like this search is outside the model.

## 5. Your picture, claim by claim

| claim | what the toy shows | status |
|---|---|---|
| Most of the flow is fixed by a smooth structure that pushes things forward | Smooth sector independent of the bit history; all histories end in the same Hat (time model E4) | **Exact** for the toy; direction **Not explained** |
| A second, dynamic structure can rewrite bits in the past if compatible with what continues | Backward snaps under rules R1–R5 (classical); compatible histories with fixed final bit and bounded `Q` (quantum) | **Exact** (definitions B2, B3 are **Definition**) |
| It knows what is compatible because it is connected to the smooth surface | Compatibility is a function of surface quantities (the outline's validity, the accumulated area) | **Definition**; that nature works this way is **Hypothesis** |
| Backward steps are allowed but much harder than going forward | Amplitude, reach and search cost, section 3.1 | **Exact** |
| Particles can look as if they move backward in time | Bubbles: bit histories reversed between two toggles, second-order amplitude | Analogy only; no particles, no antiparticles in the toy (**Hypothesis**) |
| Fuzzy "many places at once" comes from time falling apart | Superposition is put in by the two amplitudes, independent of time reversal | **Not explained** |
| Different scales exist because they connect to two structures | Frozen vs plastic regimes set by `d/σ` | **Hypothesis** |
| GR spacetime can stretch and carry waves because the past is connected to the future | The toy has no wave dynamics; GR's equations are time-symmetric | **Not explained** |
| We live at the macro scale and see the smooth structure | Macro = many bits, large `d/σ`: bits frozen and readable | **Hypothesis** (the toy has one bit) |

## 6. Where an answer to "why forward for us" could come from

Each item below is something the toy could be extended to test; none is done.

1. **Make the orientation emerge.** Replace the one-way law by a time-symmetric one with boundary conditions, and ask which end the data pick. The Hat is a natural candidate for a special end state (all histories merge there), and a "past hypothesis" would be the statement that the other end is the one with distinguishable bits. This moves the assumption, which is honest, but does not remove it.
2. **Put the observer in.** An observer is a device that reads bits and writes records. Records can only be written where the bit is readable (`τ < 1`) and only for ticks that already exist (rule R1: no record of the future). Ask whether a record-writing process picks the orientation, i.e. whether an arrow of memory follows from the same structure.
3. **Track the erased information.** Couple the register to an environment so that the odd sector's norm goes somewhere, then compare the register's entropy with the environment's along the flow. That is where the standard entropy arrow and the toy's arrow would be compared directly.
4. **Give the surface finite stiffness.** Let bits bend the surface at a cost that depends on density, then see whether "bending is hard, rewriting is cheap at small scale" holds as a quantitative statement.
5. **Non-commuting dynamics.** Section 3.2 relies on commuting tunnelling operators. With non-commuting steps the time of a snap would be encoded in the present, and the rewrite would no longer be free.

Related ideas in the literature, by name only (sources not opened here, so not cited): time-symmetric and two-boundary formulations of quantum mechanics, the Feynman–Stückelberg reading of antiparticles, consistent histories, retrocausal interpretations, the "past hypothesis" for the thermodynamic arrow. Entries enter `quantgrav.bib` only after the source is checked.

## 7. The vacuum, the past and the future (added 2026-11-10)

Your picture, as given:

> The Vacuum connects the "Past" to the "Future". The Past: increasing order, solidifying, greater interconnections, becoming finite, determined, measured, eventually compressed to bit-level that is "future" compatible. The Future: decreasing order, fluid, any connections, infinite potential, unknown, un-measured, massless, never fully defined. Therefore each "particle" is surrounded by this compatible set, of past and future, with the "past" nearest the "particle" and the "future" far away.

Everything in this section is a **Hypothesis** about how to read the toy, except the rows marked Exact, which are statements about the toy that hold whatever the reading. The tests are in `research/toymodel_v2/tm2_arrow.py` (Stage 3) unless another file is named.

### 7.1 Where each word sits in the toy

| your word | in the toy | what is Exact |
|---|---|---|
| Past: solid, determined, measured | the frozen regime, overlap `c → 0` (Spectre end) | a reading there gives 1 bit (`W = 1.0000` up to tick 4); a flip there has relative weight down to 10⁻²³ (section 3.1) |
| Past: compressed to bit-level, future compatible | a history reduced to one bit that passes the compatibility rules (same final bit; ledger within δ) | the compatible sets and their counts (`spectre_hat_backward_snap_model.py`, `tm2_directed.py`); the rules themselves are **Definition** |
| Past: greater interconnections | not modelled by the one-bit chain. Nearest thing: on the tiling a bit can only open as one of a bump-and-dent pair (`STAGE0.md`) | the pair rule is Exact on finite patches; reading it as "solid means paired" is Hypothesis |
| Future: fluid, unknown, un-measured, never fully defined | the Hat end, `c → 1` | readable bits fall 0.91, 0.73, 0.43, 0.13 at ticks 8 to 11 and are exactly 0 at the Hat; the register has rank 1 there, so every state reads (½, ½) |
| Future: any connections, infinite potential | all bit histories end in the same Hat; 54% of the weight of a single flip is in the last tick, 30% in the one before | Exact (time model E4, section 3.1) |
| Future: massless | nothing | **Not explained**; the toy has no mass |
| The Vacuum connects them | structure 1, the smooth flow between the two ends, shared by every history | that it is shared is Exact (time model E4); calling it the vacuum is Hypothesis |
| the compatible set around a particle | the set of histories compatible with the present, arranged by how deep they reach | Exact as a set; "around" needs a map from `τ` to distance, see 7.3 |

So the picture fits the chain as it is drawn: time runs from the solid end to the fluid end, events get harder to rewrite as they recede (section 3.1), and the future end is never fully defined in an exact sense (0 readable bits).

### 7.2 What the audit adds, and one thing that cuts against the picture

Stage 3 of `toymodel_v2` listed every place an orientation enters and switched each off and on (`research/toymodel_v2/STAGE3.md`).

* **Exact:** three inputs carry an arrow and all three are put in by hand: boundary data of a history (a fixed node, or the point the flux is counted from), the sign of the damping in the wave equation, and the orientation of `τ`. The tile's handedness and the scale ordering carry no arrow in any model here. Section 4, item 1 stands: the direction is an input.
* **Exact:** the paper's action does not tell a closed timeline from its reverse, although most closed timelines of 5 or more steps are different from their reverse (240 of 360 at 5 steps, 43488 of 54810 at 8). Oriented objects are common; a weight that prefers one orientation is what is missing.
* **Exact, and against a single arrow:** records sit at the solid end. A reading at tick j recovers the bit of tick i with probability `(1 + √(1−c_j²) cos Θ_ij)/2`, so of any two ticks the more solid one holds the better record of the other. The solid end holds 10.67 bits about the other 12 ticks; tick 11 holds 1.24; the Hat holds 0.

That last result agrees with "the past is measured" and has a price. If the past is the solid end, the present, which is further toward the fluid end, holds almost no record of it in the register (0.11 to 0.13 bit about each earlier tick at tick 11). In your picture the record of the past therefore cannot be something the present carries; it has to be the solid past itself, still there and still connected. That is consistent with "the vacuum connects the past to the future", and it is a commitment: the past exists as structure, not as memory. The other convention, in which "later" means "holds records", would run time from the Hat to the Spectre and would make the future the solid end, which contradicts your second sentence. The toy cannot choose between the two; your text chooses the first.

### 7.3 "Past nearest the particle, future far away"

This needs a map from the overlap `c` to a distance `r`, and none is derived. Two readings were written down, using `σ_t² = r_s/r` and the clock factor `√(1 − σ_t²)` (verified in the companion ledger as a property of that model, not of nature):

| | reading A (`PLAN.md` C7) | reading B (added in Stage 3) |
|---|---|---|
| identification | `c = σ_t` | `c = √(1 − σ_t²)`, the clock factor |
| solid end (`c = 0`) | at infinity | at the horizon, `r = r_s` |
| fluid end (`c = 1`, the Hat) | at the horizon | at infinity |
| the bit's jump | `d² = 4σ² ln(r/r_s)` | `d² = −4σ² ln(1 − r_s/r)`, which tends to `4σ² r_s/r` |

The two are exchanged by swapping the overlap with the readable fraction `√(1−c²)` (Exact algebra). **Your sentence is reading B**, and so was v1's "dense regions are frozen"; the plan's C7 was reading A and says the opposite. Under reading B the chain's ticks sit at:

| tick | overlap c | r / r_s | readable bits | share of a single flip |
|---|---|---|---|---|
| 0 to 6 | 0.03 or less | 1.000 to 1.001 | 1.00 | below 0.001 |
| 8 | 0.22 | 1.05 | 0.91 | 0.03 |
| 9 | 0.42 | 1.22 | 0.73 | 0.12 |
| 10 | 0.68 | 1.87 | 0.43 | 0.30 |
| 11 | 0.91 | 5.77 | 0.13 | 0.54 |
| 12 (Hat) | 1 | infinite | 0 | (not defined) |

So under B the determined shells hug the particle, the rewritable ones are far, and the jump of the bit falls off like a 1/r potential. Cautions: (1) this is algebra on an identification, and reading A is the same algebra with the opposite conclusion; (2) under B the jump diverges at `r = r_s`, which no tile can do; (3) the scale ladder of Stage 2 (small scales fluid, large scales solid, under its own assumption) is about scale, not distance, and would disagree with B if "near the particle" meant "small scale". Stage 5 of the plan has to derive `τ(r)` from dynamics before either reading is more than a choice.

*Update after Stage 5* (`research/toymodel_v2/STAGE5.md`). The choice was reduced, not made. If the squared jump of the bit follows the static field of a source on the sheet, reading A comes out exactly on a two-dimensional sheet (logarithmic field, the readable fraction equals the clock factor, the Hat at `r_s`), and reading B comes out to first order on a three-dimensional one (`c² = exp(−r_s/r)`, a 1/r profile, 10% off the clock at `2r_s`, no horizon, so caution (2) above goes away: the jump diverges only at `r = 0`). So "the past nearest the particle" needs the sheet to be three-dimensional, which is the one place where your "3D space" does work in the models; the tile itself is two-dimensional and would give reading A.

Where a two-ended check lands, on the shared past or only on the ends, is worked out in `PAST_AND_POTENTIAL.md`.

### 7.4 What would test the picture

1. Derive `τ(r)` for a static source and see which reading, if either, comes out (plan Stage 5).
2. Give the past somewhere to be: an environment or record store coupled to the register, so that "the solid past is still there" is a statement with a number (section 6, item 3).
3. Put more than one bit in: "greater interconnections" needs the flat-pair rule of the tiling with weights (plan Stage 4 and Stage 6).
4. Section 6, items 1 and 2 were run in Stage 3 in a first form: the time-symmetric law exists (the extension beyond the Hat, with two mirror branches), and the record test gives the answer in 7.2.

## 8. Reproduce

```
python3 spectre_hat_collinearity_flow_model.py --tests-only      # 14 tests
python3 spectre_hat_time_model.py --tests-only                   # 9 tests
python3 spectre_hat_quantum_bit_model.py --tests-only            # 10 tests
python3 spectre_hat_backward_snap_model.py --tests-only          # 10 tests
python3 spectre_hat_backward_snap_model.py --story               # the numbers of section 3.1
python3 spectre_hat_backward_snap_model.py --no-show --save figures/spectre_hat_backward_snap.png
python3 research/toymodel_v2/tm2_arrow.py --tests-only              # 11 tests (section 7)
python3 research/toymodel_v2/tm2_arrow.py --story                   # the numbers of section 7
```

The symbolic version of the flow is `symspectre_hat_flow.py`; it agrees with the numeric flow to about 10⁻¹⁴ (`--verify`).
