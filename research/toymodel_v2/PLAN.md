# toymodel_v2: plan (revision 3, after Stage 0)

Status: Stages 0 to 5 done (`STAGE0.md` to `STAGE5.md`); Stage 6 (optional) not started. Stage 3 added input (i') to C5 and reading B to C7 (see `STAGE3.md`, section 4). Labels as in `quantgrav.tex`: **Exact** (checked by a test that can fail), **Numeric**, **Definition** (a choice made in the toy), **Hypothesis** (a physical reading nothing here tests), **Negative** (a test was run and the idea failed), **Not explained**.

Framing, as in the paper: nothing here connects to gravity. "Gravity", "clock", "wave", "entanglement" and "arrow" name parts of small models and are used for a result only when a test shows the part behaves that way.

## 0. What changed

**Revision 3 (after Stage 0):** C1 corrected (the Hat is the same lift, not a different lattice object), C2 strengthened, and a new candidate compatibility rule from the tiling added to Stage 1 (see `STAGE0.md`).

**Revision 2 (from revision 1):**

Revision 1 was built from the four v1 scripts and the Spectre–Hat chain. It did not use three bodies of work that already exist, and it was wrong in places because of that:

- **`quantgrav.tex`** already contains a formal action (the section on the formal model), the hexagonal direction set inside E₈, the CHSH price, a one-parameter rule, a density and clock analysis, a chirality theorem, and exact density results on the tiling. Its open problems 11, 12 and 13 are what v2 should be aimed at.
- **The Quantum-Gravity-Dynamics fork, `directional/` (LEDGER.md)** has already tested v1 row by row. Several things revision 1 proposed to test are already verified there, and four rows are marked UNTESTED. Those four are the open front.
- **The chain's own numbers** connect to the paper more directly than I had seen (section 4).

Specific corrections: the node alphabet should be the **six hexagonal roots of E₈**, not the cubic six (the cubic set is not in E₈, is bipartite, and its zero outcomes set its CHSH threshold). Stage 3 of revision 1 (a first no-signalling and CHSH test) is already done; the open problem is to *derive* the rule. The density stage must start from the verified clock results and the exact tiling baseline. The arrow stage must start from the paper's chirality result and from the reversal symmetry of its own action.

## 1. Where v2 sits

| layer | where | what it holds |
|---|---|---|
| exact geometry | `quantgrav.tex`, the sections up to the open problems, and the companion repositories | E₈ and Hopf fibres; the Spectre lift lattice A₂⊕A₂ ⊂ E₈; the inflation does not extend (negative); Wilson-loop probe; tiling densities, chirality, curvature |
| dynamic toy | v1, then the dynamic sections of `quantgrav.tex` (axes, retrocausal contracts, density, formal model), then the QGD fork `directional/` with its ledger | six directions, contracts, CHSH price, clocks, rule, repair |
| the chain | `spectre_hat_*.py`, `symspectre_hat_flow.py`, `ARROW_OF_TIME.md` | flow Spectre → Hat, the 14th vertex bit, overlap `c(τ)`, quantum bit, backward snap, cost asymmetry |

**The open items v2 is for** (verbatim from the sources):
- Paper, problem 12: a consistency rule *derived* from a path action with boundary terms at both ends, which fixes κ and keeps no signalling when zero outcomes are allowed.
- Paper, problem 13: a density or clock with the 1/r fall-off and a horizon, using the exact tiling baseline ρ₀.
- Paper, problem 11 and the Introduction: an object on which reversal of the order of events is not a symmetry and which has the exact structure found.
- Ledger, UNTESTED: the grid produces travelling waves with a definite speed; a link between the six-direction timelines and the phase field σ_μ; a density that emerges from the dynamics.

## 2. Baseline already established (v2 must not redo it, only reproduce it)

| result | label | source |
|---|---|---|
| Closed timelines (zero net flux) of n = 0 to 8 steps: hexagonal 1, 0, 6, 12, 90, 360, 2040, 10080, 54810; cubic 1, 0, 6, 0, 90, 0, 1860, 0, 44730. The cubic set is bipartite, the hexagonal set has 12 closed 3-step timelines | Exact | ledger |
| The six hexagonal directions are six roots of an A₂ in E₈ (inner product 2× the planar one); a green root is orthogonal to every red root, so the planar "12 directions" is not an E₈ read-out | Exact | paper `res:hexdirs`, ledger |
| The copy contract signals the measured axis (log₂3 bits); global conservation signals; local conservation has no correlation (CHSH 0); revelation of a pre-existing axis is local (4/3) | Exact | ledger |
| With settings-dependent history and no signalling, the best CHSH is min(4, 2+12δ) (hexagonal) and min(4, 4/3+8δ) (cubic, threshold 1/12); 2√2 at δ = 0.069 (hexagonal) | Exact / Numeric (LP) | ledger, paper `res:price` |
| The rule P(v\|a,b) ∝ exp(κAB) pays it: CHSH 2.26, 2.49, 3.07, 3.62, 3.95 at κ = 0.1, 0.2, 0.5, 1, 2; passes 2√2 at κ = 0.362. Signalling is exactly zero only because the weight is even under v → −v; with the twelve read-outs it signals. The weight is chosen, not derived | Exact | ledger, paper `res:rule` |
| Repair after one shock: hexagonal n = 5 needs 1 edit in 7560 of 9000 cases and 2 in 1440; the greedy contract as written has a sign error (converges in 66% against 89% with it corrected, n = 12, 2000 seeds) | Exact / Recorded | ledger |
| The clock is 1/ρ by definition; update probability 1/ρ gives tick rate 1/ρ and a redshift equal to the density ratio (exact, given ρ); ρ = (1−r_s/r)^(−1/2) reproduces the QGD clock √(1−σ_t²) and diverges at r_s; the Lorentzian well falls as d⁻² where r⁻¹ is needed; the toy density is bounded, so the clock has a floor and there is no horizon | Exact / Recorded | ledger, paper `sec:density-limits` |
| Tiling density baseline ρ₀ = (5−√15)/5, deviations of supertile counts decay as (4−√15)^j per level (Pisot); ρ₀ is twice the Mystic frequency; graph curvature gives no gravitational potential | Exact† / Negative† | paper `res:mystic`, `res:curv` |
| The Spectre outline is one cyclic turning word; its mirror is its **reversal** and not a rotation of it, so the tile is chiral; all energies computed are blind to chirality; flux closure is symmetric under reversal. "Chirality, not order" | Exact† | paper `res:chirality` |
| No arrow found in: line sums, flux closure, density, curvature | Negative | paper, end of section 8 |
| The Hat, Spectre and Turtle tilings have the same bond graph | Exact | paper, appendix B |
| The Spectre inflation does not extend to E₈; λ² = 4+√15 | Negative | paper `res:inflation` |

† snapshot numbers from companion repositories.

v1 itself, run unmodified, shows the behaviours the ledger already records (`audit_v1.py`, seeds 0 to 499): the relaxation reaches |flux|² ≤ 3 in 63% of runs (ledger: 66% for 2000 seeds), the forced node survives in 17.6% of runs (that is 1/6: it survives only when it already equalled step 0), and B ends as one repeated axis in every run.

## 3. The proposal, restated

**Definition (draft postulates).**

- **P1 Smooth sheet S.** A field τ(x, t) (flow state) with a density ρ(x, t) on a lattice. Its dynamics are reversible and hyperbolic (Stage 5). The tiling baseline ρ₀ is the background.
- **P2 Discrete sheet D.** At each site a timeline of nodes in the **six hexagonal roots** (cubic six as a control). Optional: independent times per axis, as in the paper's section on dynamic axis trajectories.
- **P3 Coupling at the vertex.** A node's flip amplitude at tick t is θ = Ω·c(τ)/N (the quantum bit, unchanged).
- **P4 Constraint.** A history H is admissible if its flux J(H) is within δ of the reference. This replaces the greedy contract by a definition; the paper's action A[γ] is the candidate form.
- **P5 History sum.** W(H) = (product of local amplitudes) × (boundary terms at both ends) × [H admissible]. A backward snap is a change between admissible histories with relative weight W(H′)/W(H).
- **P6 Observation.** A measurement, and a setting, is a constraint on H, not an update rule. What the ledger calls "the settings as boundary conditions" is the same thing.

**Hypothesis (kept apart).** The two structures of `ARROW_OF_TIME.md` are S and D; "hard to move to the past" is the cost asymmetry; scales are a ratio of a jump to a width.

## 4. Connections the first plan missed

Each has a source, a status, and the test that will decide it in v2.

**C1. (corrected after Stage 0) The flat vertex is already in the Spectre; the family is one lift.** The paper's `res:corners` says every Spectre outline has 60° turns, 90° turns and **one straight pass** (inner product 2): two consecutive equal roots. That is the flat vertex of the chain, and it is flat for every (a, b) of the family. In the Cheritat–Ma lift every tile of the family, Hat included, is the same walk of E₈ roots, and the flow only scales the red plane by a and the green plane by b. (An earlier draft of this plan said the Hat's long edges are not roots and the lattice sees only the endpoints. That was wrong: it mixed the lift with the plane positions read as elements of Z[w], which exist only at b = √3 and record a rank collapse 4 → 2.) The opening of the flat vertex by h(τ) is the chain's addition, outside the family. *Exact (Stage 0, E1 to E4); the reading of b/a as the ratio of two orthogonal sectors, and as the paper's "time per axis", is a Hypothesis.*

**C2. (confirmed, stronger) The discrete structure is invariant along the flow.** Lifted vertices, bond graph (406 seams at level 2) and E₈ image are identical for all b in [1, 2] (Stage 0, E2, E3). The chain has the matching statement the other way round: bit flips leave the smooth sector unchanged (rule R4). Together these are the exact version of "two structures of one object": the combinatorics is fixed while the geometry flows, and the bit sits where the two meet. **New (Stage 0, E5 to E7):** the only thing the tiling does not allow is the bit itself. Opening every flat vertex alike breaks 88 of 406 seams; the tiling allows the opening only as antiparallel pairs on tiles whose flat vertices coincide (about 40% of interior tiles at level 4), conserving the area ledger Q; all other tiles are frozen. This is a compatibility rule that the substrate supplies and nobody chose (candidate for P4, Stage 1).

**C3. Hexagonal alphabet and the red/green split.** Red roots (D₄ᵃ) are mutually non-orthogonal; every green root (D₄ᵇ) is orthogonal to every red one. So the two colours are decoupled sectors and a natural place for the paper's "each axis carries its own time". v2 default: the red six as the alphabet; per-colour times as an option.

**C4. Order reversal is a mirror: the two arrows of the chain are the two traversal directions.** For the outline, reversing the order of the turning word gives the mirror word (paper). The "two arrows" at P₈ in `spectre_hat_time_model.py` (tangents arriving and leaving) swap under the reversal of the traversal. So on a *closed* timeline, "reverse the order of events" and "reflect" are the same operation for the Spectre. Two consequences to test:
 1. *Chirality census.* Among the closed hexagonal timelines of n steps (counts in section 2), count exactly those whose reversal is not a rotation of them. A chiral closed timeline has an intrinsic orientation. This is a finite, exact count (n ≤ 8, 54810 at n = 8), and it asks the paper's problem 11 in its sharpest form: is there a combinatorial object with reversal not a symmetry *and* the exact structure? The Spectre outline is one; the census says how special it is.
 2. *Independence of handedness.* The chain's cost laws use a, b, h and c only, so they should be unchanged under reflection of the tile: the chain's arrow does **not** know the tile's handedness. Test on the code. If one wants it to (parity locked to time orientation), the coupling must use a signed, parity-odd observable, as the paper says the handedness theorem is, not an energy. A Spectre tiling is strictly chiral, so handedness is a global binary datum of a tiling, like a time orientation. That is the analogy, not a result; real P and T are independent, and the plan should treat the locking as a toy hypothesis with its cost stated.

**C5. The paper's own action is time-reversal symmetric, and the orientation sits elsewhere.** The first term of A[γ] is symmetric. The flux term is a prefix sum from t = 0. For a closed timeline with zero target, reversal permutes the prefix sums and **A is symmetric** (proof: the prefix sums of the reversed timeline are minus the prefix sums of the original, shifted by one, and Φ(T) = 0; checked on 8646 sampled closed 6-step timelines, 0 asymmetric). For open timelines it is not (91.5% of 2000 random 8-step timelines). So an arrow in the paper's formalism can only come from: (i) **pinned ends** (s(0) = s_init, a fixed measured node); (ii) the **damping** γ∂ψ/∂t in the wave equation (which is not reversible); or something added by v2. The orientation inputs available in total are therefore a short list:

| input | where it comes from | status |
|---|---|---|
| (i) pinned end | boundary data of the history | present in the paper |
| (ii) damping γ | wave equation of the formal section | present in the paper |
| (iii) orientation of τ | the chain's flow law | present in the chain (`ARROW_OF_TIME.md`, E7) |
| (iv) handedness ε | strictly chiral tiles | C4, not coupled |
| (v) scale ordering | Pisot decay along inflation levels | C8, not coupled |

Stage 3 below audits each one separately and asks which are independent.

**C6. Deriving the rule (open problem 12): the target is specific.** P(v|a, b) ∝ exp(κAB) is a Boltzmann weight on the agreement of the two ends, normalised per pair of settings. The ledger notes that it signals once zero outcomes are allowed and that it has no derivation. P5 and P6 say where a derivation would come from: a history weight with a boundary term at *each* end, the settings entering as constraints. Requirements, all testable: (a) the AB form, or another form, comes out; (b) κ is fixed by model parameters (a candidate: a function of the overlap c, since κ = 0 is a frozen readable bit and large κ is a plastic one; this is a Hypothesis); (c) exact no-signalling for the twelve read-outs, not only for the six; (d) the normalisation is stated: the per-pair normalisation is exactly what allows signalling, so the derived weight must say what is normalised and why. Above 2√2 the table is a no-signalling box and not quantum; a complex-amplitude version (Stage 4b) is the only route to a Tsirelson bound, and a plausible outcome is that it does not reach it.

**C7. A formal identity between the quantum bit and the QGD clock.** In the quantum bit model the optimal readout success of two overlapping packets is governed by the Helstrom quantity √(1 − c²) (verified numerically: `helstrom(τ)` equals √(1−c²) at τ = 0, ½, 0.9, 1). The QGD clock is √(1 − σ_t²) (VERIFIED in the ledger), with σ_t² = r_s/r. If c is identified with σ_t, the readable fraction of the bit equals the static-observer clock, and the horizon (σ_t = 1) is exactly the Hat end (c = 1, bit unreadable, odd sector annihilated). With Gaussian packets this needs c² = r_s/r = exp(−d²/4σ²), i.e. d² = 4σ² ln(r/r_s), a logarithmic profile. **This is a coincidence of form until τ(r) comes out of dynamics.** It is the same fault as v1's "clock = 1/ρ": any monotone c can be matched by choosing τ(r). So the only valid test is Stage 5: a reversible wave on S produces τ(r), and then the identity either holds or it does not. Failing conditions: the profile is not logarithmic; the flow range [0, 1] puts a floor on c (c(0) = e^(−d₀²/8σ²) > 0), so the identity cannot hold at large r unless τ is extended below 0. Related ledger row: the link between the timelines and σ_μ is UNTESTED, and this would be its first definition.

**C8. Scales: a ladder from the Pisot number.** The tiling's density fluctuations decay as g^j with g = 4 − √15 = 1/λ² per inflation level j: smooth at large scale, fluctuating at small scale, exactly. That is the user's "smooth structure at large scale, discrete dynamic one at small scale" in the one place where it is exact. For the bit: if the packet width σ is an absolute quantum width and the jump d scales with the tile size, then d_j = λʲ d₀ and **c_j = c₀^((4+√15)^j)** (a consequence of the assumption, and exact given it). From c₀ = 0.872 (τ = 0.9): c₁ = 0.34, c₂ = 2·10⁻⁴. So the same number controls both how fast the tiling smooths out (g^j) and how fast the bit freezes (c₀^(g⁻ʲ)). Small tiles are plastic, large tiles are classical. This is a Hypothesis with an explicit assumption (d ∝ size, σ fixed); it is not time order, it is scale order, and the plan keeps the two apart. Test: the ladder is reproduced by code, and one checks whether the level ordering and the flow ordering (iii) can be related without a choice.

**C9. The Hat as a Janus-like point of the flow.** The laws extend analytically past τ = 1: h(τ) = a·tan(ω₀(1−τ)/2) is odd about τ = 1 and c(τ) is even, with the bit's two values swapping between convex and reflex at P₈ (the extension is not implemented; the code restricts τ to [0, 1]). The Hat is then a point where the bit is unreadable and both sides are mirror images. Barbour and collaborators study arrows that come from a time-symmetric law with a special central configuration and two arrows pointing away from it (named in the paper's introduction; only abstracts were read there and nothing from them is used here). Two orientations are available in the toy and they point **opposite ways**: the chain's flow runs *toward* the Hat (merging; bits become unreadable), while records accumulate *away* from it (bits become readable). Which is "forward" is decided in Stage 3 by the record test (rule R1 of the time model: a record can only be written where the bit is readable), not by choice.

**C10. Orientation at tiling level.** Every tile outline is traversed clockwise, so an edge arrow is antiparallel across each seam unless one side is flipped (`models/README.md`, model 5): a consistent patch-wide flow is a cycle of the dual graph. This belongs to Stage 6 (tiling), gated.

## 5. Rules for v2

1. **Define, then compute.** Every contract is an objective or a constraint, stated before it is coded.
2. **Pin what is forced.** A forced node is a constraint and cannot be overwritten.
3. **Exact where size allows.** Enumerate or use a transfer matrix over (state, flux). Counts must reproduce the ledger's.
4. **A test that can fail in every stage**, and a Negative result is a deliverable.
5. **A word needs its test.** "Wave" needs a speed and conserved energy; "clock" needs a ratio; "entanglement" needs a stated correlation and a no-signalling check; "arrow" needs one of the five inputs of C5 identified.
6. **Do not match what should be derived.** The fault of v1, and the risk in C7.
7. **Rely only on verified QGD rows.** The ledger has FAILS rows (the two-body far field, the Ricci residual). v2 uses the clock √(1−σ_t²) and nothing from the rows marked FAILS or UNTESTED.
8. **One core, many scripts.** Single-file models with `--tests-only`, docstring with definitions and exact claims, small shared `core.py`.
9. **v1 and the fork stay as they are.** Ports are new files.

## 6. Stages

Size: S a few hours, M a day, L several days.

### Stage 0 (M): baseline and the lattice checks — DONE (see `STAGE0.md`)
- `core.py`: counts equal the ledger for both alphabets, n ≤ 8 (7 tests).
- `tm2_lattice.py`: C1 corrected, C2 confirmed for the whole family, and the flat-pair result (9 tests).
- The planned failure condition (bond graph changes along the flow) did not occur; a different one did: the opened tile is not a member of the family (E6).

### Stage 1 (M): the constraint model on the hexagonal alphabet — DONE (see `STAGE1.md`)
- Exact transfer-matrix solver for the paper's action with a flux constraint and a pin (`tm2_history_sum.py`, 8 tests).
- **Result:** the failure condition was met: with the flux constraint on, a pin's influence does not decay with distance (it reaches the far end), so the "ripple" is a property of the greedy solver and not of the constraint. Repair counts equal the ledger; the corrected greedy contract is close per node but collapses in 14% of runs.
- **Result:** the flux constraint is blind to the bit (outlines stay closed under opening); the seam constraint is a strict subset of the area ledger Q = 0 (64 of 924 of 4096 at level 2; 2^82 against C(164, 82) at level 3). P4 should be stated with Q.
- **Open, carried to Stage 2:** the letter timeline and the bit are different objects; how they sit in one history is not yet defined.

### Stage 2 (M): directed weights from the chain, and the scale ladder
- Weights from the quantum bit (P3). **Regression:** the two-state, scalar-flux reduction reproduces `spectre_hat_backward_snap_model.py` (flip-time distribution, cost table, E7) to 1e-12.
- Then the six-state generalisation with a vector constraint: do the three costs (amplitude, reach, search) survive?
- **C8:** c_j = c₀^((4+√15)^j) in code, with the assumption stated in the docstring.
- **Fails if:** the vector constraint removes the asymmetry.

### Stage 3 (M): the arrow audit (the cheap and central stage)
- Prove C5 and test it: A symmetric on closed zero-target timelines, asymmetric otherwise; separate the five inputs by switching each off in turn.
- **C4:** the chirality census of closed hexagonal timelines (n ≤ 8); the mirror test on the chain.
- **C9:** implement the extension of the flow beyond τ = 1; run the record test (where can records be written) and decide which of the two orientations records follow.
- Upgrade of E7: swap the boundary data and see whether the asymmetry follows. **Result, whichever way it goes, is a statement of which of (i) to (v) carries the arrow.**
- **Fails if:** more than one input is needed to get the asymmetry and they disagree on direction. That is a result, not a failure of the stage.

### Stage 4 (L): derive the rule (open problem 12)
- Build the history weight with boundary terms at both ends; compute P(v | a, b) exactly; compare with exp(κAB) (C6 a to d).
- **4b:** complex amplitudes; compute CHSH exactly and compare with 2√2.
- **Fails if:** no form with exact no-signalling for the twelve read-outs exists, or κ cannot be fixed by model parameters. Both would be Negative results for the weight, with the cause identified.

### Stage 5 (L): dynamics on S, clock, waves (open problem 13)
- S as a scalar field with a leapfrog wave equation with γ = 0. **Tests:** energy conserved to 1e-12; pulse speed as stated; reflection and transmission across a density step match the discrete theory; reversal retraces to 1e-10. The damping γ is added afterwards, as a separate input, and shown to be an arrow input (C5 ii).
- Couple D to S through P3 and the constraint J; ask whether snaps radiate and whether the radiation falls off with distance.
- **C7:** read τ(r) off the dynamics for a static source and test the identity with √(1−σ_t²); test the fall-off (r⁻¹) and whether a horizon-like end (c = 1) occurs. The per-tick and per-coordinate-time plasticity must be separated: slower flow gives a smaller rate per coordinate time but a larger overlap per tick, so the direction of the net effect is not obvious (and v1's claim that dense regions are frozen is exactly what is tested).
- **Fails if:** the profile is not logarithmic, or the floor on c prevents the identity, or dense regions come out plastic.

### Stage 6 (optional, L): D on the Spectre tiling
- Place the bit on tiles of the family, consistent orientation as in C10, τ varying with density. Gated: only if an earlier stage shows that geometry beyond `c(τ)` and the C1, C2, C8 facts matters.

## 7. Layout

```
research/toymodel_v2/
  PLAN.md                  this file
  audit_v1.py              done
  core.py                  done (Stage 0): alphabets, flux, DP counts
  tm2_lattice.py           done (Stage 0): lift, family, flat pairs
  STAGE0.md                done: Stage 0 results
  tm2_history_sum.py       done (Stage 1)
  STAGE1.md                done: Stage 1 results
  tm2_directed.py          Stage 2   (imports spectre_hat_quantum_bit_model.py, spectre_hat_backward_snap_model.py)
  tm2_arrow.py             Stage 3
  tm2_rule.py              Stage 4
  tm2_waves.py             Stage 5
  TOYMODEL_V2.md           results note, written as stages finish
  figures/
```
The QGD fork is imported (not copied) through `models/_qgd.py` for regression, and the code falls back to the definitions if the fork is absent.

## 8. Decisions, with defaults

1. **Alphabet.** Six hexagonal roots (E₈); cubic as a control. Per-colour times as an option.
2. **Weights first, amplitudes in Stage 4b.**
3. **S.** A reversible scalar wave first. The σ_μ phase field of QGD only through its verified clock, and only in Stage 5.
4. **Observer.** A measurement or a setting is a constraint (P6).
5. **Orientation.** No orientation is put in by hand without being listed as one of the five inputs of C5.
6. **Spectre.** Enters through c(τ) and, from Stage 0, through C1, C2 and C8. The tiling only at Stage 6.

## 9. What would count as failure

- Stage 0: the bond graph changes along the flow (did not occur; the opened tile is outside the family instead, see `STAGE0.md`).
- Stage 1: influence does not decay.
- Stage 2: the reduction does not match the chain, or the vector constraint removes the asymmetry.
- Stage 3: no single input carries the arrow, or the record test and the flow orientation disagree and nothing breaks the tie.
- Stage 4: no derivation with exact no-signalling on the twelve; κ not fixed.
- Stage 5: the C7 identity fails; the profile is not logarithmic; dense regions come out plastic; coupling breaks reversibility or energy conservation without a stated term.
- Any stage: the result depends on update order or seed.

A failure is reported and labelled Negative with the postulate that caused it.

## 10. Related ideas, by name only (sources not opened, not cited)

Janus-point arrows of time; two-state-vector and multiple-time quantum mechanics; relational quantum mechanics; retrocausal formulations; consistent histories; Bell and CHSH, no-signalling, Tsirelson's bound; analogue gravity; causal sets, causal dynamical triangulations, Regge calculus; Helstrom discrimination; lattice wave equations. Entries enter `quantgrav.bib` only after the source is opened.

## 11. Order

Stage 0, then 1, 2. Then **3 (the arrow audit)**, which is cheap, exact and the thing you asked about, before the two long derivations: **4 (rule)** and **5 (dynamics and clock)**. Stage 6 is gated. If time is short, Stages 0, 2 and 3 alone give a clean statement of which input carries the arrow.

## 12. Reproduce so far

```
python3 research/toymodel_v2/audit_v1.py          # v1 behaviours, 500 seeds
```
