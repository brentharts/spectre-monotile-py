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

## 7. Reproduce

```
python3 spectre_hat_collinearity_flow_model.py --tests-only      # 14 tests
python3 spectre_hat_time_model.py --tests-only                   # 9 tests
python3 spectre_hat_quantum_bit_model.py --tests-only            # 10 tests
python3 spectre_hat_backward_snap_model.py --tests-only          # 10 tests
python3 spectre_hat_backward_snap_model.py --story               # the numbers of section 3.1
python3 spectre_hat_backward_snap_model.py --no-show --save figures/spectre_hat_backward_snap.png
```

The symbolic version of the flow is `symspectre_hat_flow.py`; it agrees with the numeric flow to about 10⁻¹⁴ (`--verify`).
