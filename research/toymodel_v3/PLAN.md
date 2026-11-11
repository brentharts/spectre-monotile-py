# toymodel_v3: a plan for the problems listed against quantgrav.tex

Status: Stage 1 done (`STAGE1.md`: the factor 2 of light bending needs a space factor the model lacks; rule (C) gives 1/r, the clock and a horizon on Z^3 and none of them on the Spectre tiling). Stage 2 done (`STAGE2.md`: the signed area is a weight odd under reversal, which flips with the mirror image of the world; a constant pass rate keeps the violation, and the Clauser-Horne inequality never fails, so read as data the two-ended tables are local). Stages 3 and 4 done (`STAGE3.md`: with an external record store, records alone make rewriting the frozen end expensive and agree with the amplitudes, so the conflict of v2 is gone; `STAGE4.md`: with spin read-outs nothing is discarded, and the singlet follows exactly from an origin odd under inversion, for any common wing; a flux cannot make the origin odd, and a spinor read-out of scalar amplitudes is branch-dependent, so the sign needs spinor-valued amplitudes). Stage 6 done (`STAGE6.md`: the Chair44 is transient; its decoration carries a fixed-sign handedness psi = -3/14 per tile, odd under P and C; its relaxed walk tensor is isotropic in the bulk (2e-4), so the clock of rule (C) and long waves are, but its hop count is cubic, spread 0.24, so light must be a long wave, not a hop; B is still missing). Stage 5 done (`STAGE5.md`: with closure the far end responds on one global time ~ n^2, not by a front; without it there is a front). Stage 7 done: `quantgrav.tex` Section 'A second pass: the problems of the first' (Results res:bend to res:relax), numbers in `paper_numbers.py` (toymodel_v3_numbers). Labels as in `quantgrav.tex`: **Exact**, **Numeric**, **Negative**, **Hypothesis**, **Definition**. Nothing here connects to gravity until a test says so, and the paper keeps the rule that every number comes from code.

Input: the problem notes of 2026-11-10 (14 items in four groups, plus the paper's own open problems), read against the 16-page `quantgrav.tex` and the code of `research/toymodel_v2`.

## 0. First, sort the problems

Not every item is a defect. Each one gets one of four verdicts:

| verdict | meaning | what to do |
|---|---|---|
| **bug** | a model contradicts its own text, or a better-posed version is obvious | fix it, with a test that can fail |
| **open** | a genuine gap with a concrete next step | build the step, with a stated failure condition |
| **result** | a negative result that is correct as stated | keep it; tighten the wording; do not "fix" it |
| **misread** | the note says more than the paper does | correct the note; possibly clarify the paper |

| # | note | verdict | section |
|---|---|---|---|
| 1.1 | inflation incompatible with E8 | **result** + **misread** | 6.1 |
| 1.2 | Chair44 shape vs structure | **result** | 6.2 |
| 1.3 | Wilson placement dependence | **result** (not ours to fix) | 6.3 |
| 1.4 | curvature does not give point masses | **result** | 6.4 |
| 2.1 | orientations put in by hand | **open** | 3.1, 3.2 |
| 2.2 | action symmetric on closed histories | **open** (same root as 2.1) | 3.1 |
| 2.3 | "later holds records" against "earlier = determined" | **open**, and the most interesting one | 3.3 |
| 3.1 | no Tsirelson bound | **open** | 4.1, 4.2 |
| 3.2 | the pass rate depends on both settings | **open** (same root as 3.1) | 4.1 |
| 3.3 | e^{κAB} depends on the naming of an outcome | **bug**, already fixed by the derived rule | 4.3 |
| 4.1 | waves faster in dense regions | **bug** | 2.1 |
| 4.2 | 1/r and a horizon come apart | **open**, with a candidate fix | 2.2 |
| 4.3 | a pin acts globally, not as a ripple | **result** for the ensemble; **open** for the dynamics | 5.1 |

### Corrections to the notes (misreads)

- **1.1.** The paper shows that the inflation is not an *isometry* of E8. It does not show that combining the Eisenstein and icosian structures fails; that combination is an untested Hypothesis. A cut-and-project set also does not need the inflation to be an isometry, only a lattice (module) map. See 6.1.
- **1.3.** "Fits fail for features smaller than two blocks" holds at the aligned placement only. Averaged over placements, the thin loops lie *above* the fit.
- **3.2.** "Leaking information through acceptance rates" is a leak only if the pass rate can be observed at one end. In the toy it is a normalisation and nothing observes it. Whether it could be observed (for example as the duration of a check) is a modelling choice; see 4.1.
- **4.1.** The immediate problem is that equation (W) contradicts the paper's own text ("dense regions lag"). The comparison with GR, where the coordinate speed of light is lower near a mass (the Shapiro delay), points the same way.

## 1. Order of work

Cheap and decisive first. Each stage ends with a note `STAGEn.md` and a test module, as in v2.

| stage | content | cost | decides |
|---|---|---|---|
| 1 | 2.1 wave equation and light bending; 2.2 the clock from a hitting probability | S | 4.1, 4.2 |
| 2 | 3.1 a weight odd under reversal; 4.1 settings-independent pass rate | S | 2.1, 2.2, 3.2 |
| 3 | 3.3 records as the compatibility rule, and the direction of the flow | M | 2.3 |
| 4 | 4.2 a read-out in which the setting acts before reading | M | 3.1 |
| 5 | 5.1 local relaxation dynamics: does influence spread at a finite speed? | M | 4.3 |
| 6 | 2.2 on a three-dimensional substrate (the Chair44 cell graph) | L | 4.2 on the tilings |
| 7 | paper: rewrite the affected Results and the open problems; tighten 6.1–6.4 | S | — |

## 2. Gravity group

### 2.1 The wave equation (bug), and the factor 2

Equation (W) is `ψ_tt/(c²ρ) − ∇²ψ = S`. Its speed is `c√ρ`, so dense regions are fast (Result `res:waves`). The candidate fix puts the density where a refractive index goes: `n² ψ_tt / c² − ∇²ψ = S` with `n = n(ρ) > 1`, so the speed is `c/n`.

The clock (`Δτ = Δt₀/ρ`) and the index `n(ρ)` are then two couplings of one density. The test that can fail is light bending:

- in GR's weak field, the clock factor is `1 − r_s/2r` and the index is `n = 1 + r_s/r`, so a ray with impact parameter `b` is deflected by `2r_s/b`;
- a clock-only (purely temporal) coupling gives `r_s/b`, half the GR value;
- Nordström's scalar theory gives none.

**Build.** A ray (eikonal) and a wave-packet run on the lattice through `n(r)`. Measure the deflection and the Shapiro delay against `b`.

**Tests.**
- With `n = 1 + r_s/r` the deflection is `2r_s/b` to the lattice tolerance.
- With `n = 1 + r_s/2r` it is `r_s/b`.
- The delay is logarithmic in `b`.

**Fails if** no rule *derived* from the model links the index to the clock as `n ≈ clock⁻²`. Choosing the ratio by hand is not a fix; record it as a Definition. A Negative here would repeat history: a single scalar density cannot give the factor 2 without a second, spatial coupling.

**Paper.** Result `res:waves` stays as the test of (W) as written. A new Result gives the corrected equation and the deflection.

### 2.2 The clock from a hitting probability (open, candidate fix)

Stage 5 of v2 found that a coupling linear in `d²` gives either 1/r (on a 3D sheet) or an exact clock with a horizon (on a 2D sheet), never both. There is a coupling that gives both:

    the squared overlap of the bit equals 1 − (the field):   c² = 1 − r_s / r.

This is the 3D static field itself, and it gives the following.
- **The clock exactly.** `c = √(1 − r_s/r)`.
- **A horizon.** `c = 0` at `r = r_s`.
- **1/r far away.** `1 − c² = r_s/r`.
- **The jump.** `d² = −4σ² ln(1 − r_s/r)`, which tends to `4σ² r_s/r` far away.

It is reading B of Stage 3, and it says the dense region is frozen (the past next to the particle).

A rule that is only chosen is not a fix, so the question is why `c²` should be linear in a harmonic field. One candidate derivation:

- `1 − c²` is a probability (one minus the squared overlap of two states).
- The probability that a random walk started at distance `r` ever reaches a ball of radius `r_s` is exactly `r_s/r` in three dimensions, and is harmonic. In two dimensions it is 1 for every r, because walks there are recurrent.
- **Hypothesis.** The bit's two states are distinguishable exactly when the substrate's random walk from the bit does *not* reach the source: `c² = P(escape)`. The horizon is then the surface the walk cannot escape from.
- This ties the 2D/3D result of Stage 5 to a single fact (recurrence in 2D, transience in 3D). It predicts that a 2D substrate, such as the Spectre tiling, is "all horizon": every bit is unreadable at long times.

**Build.**
1. Escape probabilities of the simple random walk on Z³ from a ball, against `r_s/r` (exact on the lattice via the Green function).
2. The same on Z² and on the Spectre vertex graph, expecting escape probability 0, with a finite-size scaling of how it goes to 0.
3. Couple: `c²(r) = P_escape(r)`, then compare the readable fraction with the clock; check the horizon at the ball.
4. Stage 6: the Chair44 cell graph as a 3D aperiodic substrate.

**Tests.**
- Lattice escape probability matches `r_s/r` within the expected lattice correction.
- 2D escape tends to 0 (as `1/ln` of the size).
- The clock identity is exact under the coupling.

**Fails if** the escape probability on the Chair44 graph is not 1/r-like, or if the coupling needs a free function to match. **Hypothesis** throughout: nothing here says why distinguishability should be an escape probability.

**Paper.** Open problem 8 ("1/r and a horizon") gets a candidate and a test. Result `res:clock` gains a third row.

## 3. Arrow group

### 3.1 A weight that tells a timeline from its reverse (open)

Stage 3 found that the action `A` is reversal symmetric on closed timelines, while most closed timelines are oriented (240 of 360 at 5 steps). Missing is a term that is odd under reversal.

One exists on the hexagonal alphabet: the **signed area** enclosed by a closed timeline. The area changes sign when the order of steps is reversed and also under a mirror, and does not change under a rotation. A term `θ · Area` therefore distinguishes a timeline from its reverse:
- as a real weight, it prefers one orientation;
- as a phase `e^{iθ Area}` (an Aharonov–Bohm or magnetic term), it breaks time reversal the way a magnetic field does.

The sign of θ is an input, but it can be tied to one existing datum: the handedness of the Spectre tiling, a single global binary choice. That moves the arrow from "put in by hand" to "fixed by the chirality of the substrate", which is a statement the toy can test and the paper can state honestly.

**Build.** Add `θ·Area` to the transfer matrix of `tm2_history_sum` (the state needs the running area, which is still finite).

**Tests.**
- `A(Rs) − A(s) = −2θ Area(s)` on all closed timelines up to `n = 7`.
- The census splits by the sign of the area.
- Under the mirror together with `θ → −θ` everything is invariant (a CPT-like check).
- The chain's costs follow the sign of θ once the chain is coupled to the area.

**Fails if** coupling θ to the tiling's handedness needs a second free sign. That would be a Negative: the arrow would not be the chirality.

### 3.2 The base point of the flux (open, small)

The open-history arrow of Result `res:audit` (i) comes from counting `Φ_net` from `t = 0`. Write the symmetric version, with the flux counted from both ends and averaged, and check that it removes the asymmetry. Then the paper can say plainly that the formal section's arrow is a convention.

### 3.3 Records and the direction of the flow (open)

The conflict: records sit at the frozen (Spectre) end, while the chain runs toward the Hat. There are two ways out, and the toy can decide between them.

- **(a) Reverse the flow.** Let the chain run from the Hat (fluid) to the Spectre (frozen), as in "the potential falling into the past". Then:
  - later ticks are more determined and hold records of earlier ones;
  - "later holds records" and "the determined part is what has already happened" agree.

  The price: the earlier ticks are now the fluid ones, so rewriting them is cheap in amplitude (Stage 2, reversed flow).
- **(b) Records as the compatibility rule.** Replace the ad hoc ledger tolerance δ (B3 of the chain) by the records themselves. A rewrite of tick i is compatible only if it agrees with what the present holds about tick i, weighted by `K(i, now)`.

  On flow (a) the present is frozen and holds about 0.78 bit or more about every earlier tick, so a rewrite of the past is cheap in amplitude but expensive in compatibility. Backward is then hard because the present remembers, which is the standard thermodynamic reason, now inside the toy.

**Build.** `BackwardSnap` with a record constraint, on both flows.

**Tests.**
- On flow (a) with the record constraint, the cost of a backward snap rises with depth (the v1/v2 asymmetry is recovered).
- On the original flow it does not, because the present (near the Hat) holds about 0.11 to 0.13 bit per earlier tick and constrains almost nothing.

**Fails if** neither flow gives backward-is-hard with records alone. Then the asymmetry needs the ledger, which is an input.

**What it would settle.** Which orientation the toy's own record-keeping picks. `ARROW_OF_TIME.md` §7.2 left this as "the toy cannot choose"; with (b) it can.

## 4. Quantum group

### 4.1 A settings-independent pass rate (open)

Both the violation and the pass-rate dependence come from conditioning on both ends (Results `res:derived` (d) and `res:space`). Constrain the family so that `Z(a, b)` does not depend on the settings, then maximise CHSH. Ways to impose the constraint:
- boundary weights rescaled per setting;
- an extra "idle" outcome that absorbs the difference;
- wing lengths long enough that the contrast vanishes (Stage `res:space` shows it vanishes first, before the correlation).

**Tests.**
- An exact CHSH maximum under the constraint, at every β.
- Its relation to 2, to 2√2 and to the past shift δ.

**Possible outcomes, all informative.**
- `CHSH ≤ 2` under the constraint: no violation without a settings-dependent pass rate. That would be the cleanest statement of what "retrocausal" costs in this toy.
- `2 < CHSH ≤ 2√2`: a bound appears.
- `CHSH > 2√2`: the pass rate is not the whole story.

### 4.2 The setting acts before the read-out (open)

Tsirelson's theorem gives `|CHSH| ≤ 2√2` for any quantum state when each party measures a ±1 observable on its own system. Stage 4b had no such observable: the read-out was a sign of the end letter, or a pin.

**Build.**
- Map each end letter to a two-level system: the six hexagonal directions as six spin-½ states in a plane, 60° apart. This is the plane the Spectre lives in.
- Read the setting `a` as the observable `a·σ`.
- Let the two-ended history produce the joint state of the two ends (amplitudes from the wings, summed over the origin).

**Tests.**
- `CHSH ≤ 2√2` exactly for every parameter, by construction. A test of the code, not a result.
- Whether some (β, n) makes the joint state the singlet, so that `E = −cos θ` and the best value on the six directions is 5/2 (the value given in Result `res:chsh`).
- The overlap between the produced state and the singlet, against β and n.

**Fails if** no parameter gives a state close to the singlet. Then the toy reaches the quantum range only trivially (by the theorem), not by producing quantum correlations.

### 4.3 The naming dependence (bug, fixed)

Result `res:rule-label` and the derived table of `res:derived` already settle it. In the paper, retire `e^{κAB}` from anything but history; nothing to build.

## 5. Dynamics group

### 5.1 How fast does a measurement act on a history? (open)

Result `res:influence` is a statement about the exact *ensemble*: a pin's influence is global. The ripple picture is about *dynamics*, how a history relaxes after a pin is imposed.

**Build.** Local heat-bath (Glauber) updates of single letters that leave the exact ensemble invariant (detailed balance against `e^{−βA}` with the flux constraint enforced by pair moves).

**Measure.** Influence against node and relaxation time after a pin is switched on.

**Tests.**
- The stationary distribution equals the transfer-matrix marginals (exact check).
- The influence front moves at a finite speed in relaxation time.
- The long-time profile is the global one of `res:influence`.

**Outcome.** The paper can then say both things correctly: global in the ensemble, local in time, with a measured front speed. **Fails if** pair moves that preserve the flux are non-local enough to make the front jump.

## 6. Static group: results to keep, wording to tighten

### 6.1 The inflation (result; reframe)

Keep Result `res:inflation`. It is correct that no isometry of E8 restricts to the inflation. Add the reframing, and check it before writing it in: a cut-and-project description needs a lattice automorphism (Z-linear, not an isometry), and the Spectre already has one on Z[ζ₁₂] with a fractal window (Baake, Gähler, Mazáč and Sadun, cited in `res:bragg`).

The question worth asking is whether the lift's Z[ω]-module structure, which the embedding into E8 uses, is compatible with the inflation. Exact check: whether the 4 × 4 inflation block commutes with multiplication by ω on Z⁴. If it does, the inflation extends to E8 as a Z[ω]-module map that is not an isometry, and the Hypothesis of `res:field` gets a concrete form. If it does not, say so.

Number fields: λ² = 4 + √15 lies in Q(ζ₆₀), which contains both Q(ζ₁₂) and the golden ratio; λ itself needs √2 (it lies in Q(ζ₁₂₀)). That is a precise version of open problem 4.

### 6.2 Chair44 (result)

Keep it. One test could still change it: a tip-to-root rule that preserves adjacency (open problem 3). Optional, low priority. Consider moving Results `res:chair` and `res:chair-neg` to an appendix if the paper must shrink further.

### 6.3 Wilson placement (result; not ours)

Keep it. The open question belongs to the Wilson-loop authors (which placement their blocks correspond to). The one thing we can add: corner constants from placement-averaged energies at two block sizes, if the companion repository has the runs.

### 6.4 Curvature (result)

Keep it. If point masses are wanted from the tiling, they should come from *sources* in a field, as in 2.2, not from graph curvature. A candidate: Mystic tiles as unit sources (they are where the X-bonds are). On the 2D tiling this gives a logarithm, by Stage 5; on the Chair44 it could give 1/r. That joins 6.4 to Stage 6.

## 7. What the paper would say after this

| problem | if the stage succeeds | if it fails |
|---|---|---|
| waves | corrected equation; light bending factor 2 derived or shown to need a second coupling | Negative for a scalar density, stated as Nordström-like |
| 1/r and horizon | both from `c² = P(escape)` on a 3D substrate | the coupling stays a Definition |
| arrow | the arrow is the chirality of the tiling (one datum), or records pick the flow | every orientation stays an input; records do not decide |
| Tsirelson | a bound with a stated reason (constant pass rate, or setting-before-read-out) | the toy reaches the quantum range only by construction |
| ripple | global in the ensemble, finite front speed in time | — |

## 8. Rules (as in v2)

1. Every claim has a test that can fail; numbers in the paper come from `paper_numbers.py`.
2. Do not match what should be derived. A coupling chosen to reproduce √(1 − r_s/r) or 2r_s/b is a Definition, and says so.
3. Sources enter the bibliography only after they are opened. Nordström's theory, the GR light-bending factor and Tsirelson's theorem are standard and would be cited from a text that has been opened; the random-walk facts likewise.
4. Hold new files until "present the files"; never commit.

## 9. Proposed layout

```
research/toymodel_v3/
  PLAN.md               this file
  t3_optics.py          2.1   wave equation with an index, ray and packet deflection, delay
  t3_escape.py          2.2   escape probabilities on Z^3, Z^2, the Spectre graph; the clock coupling
  t3_area.py            3.1, 3.2   signed-area term, symmetric base point, census by area
  t3_records.py         3.3   backward snaps with a record constraint, both flows
  t3_passrate.py        4.1   CHSH under a settings-independent pass rate
  t3_qubit.py           4.2   end letters as spin-1/2, singlet overlap
  t3_chair_metric.py    6b    Chair44 walk tensor (bulk), hop-count anisotropy, clock isotropy
  t3_relax.py           5.1   heat-bath dynamics and the influence front
  t3_chair_substrate.py Stage 6   the Chair44 cell graph as a 3D substrate
```
