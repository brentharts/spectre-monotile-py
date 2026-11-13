# toymodel_v7: plan (revision 9, all stages done)

Status: Stages 0 to 8 done (`STAGE0.md` to `STAGE8.md`); summary in `README.md`.

Aim: take the list of missing features (relativity, quantum structure, thermodynamics, algebra) and turn each one into a test that can fail, ordered so that cheap exact questions come first. Labels as in `quantgrav.tex`: **Exact**, **Numeric**, **Definition**, **Hypothesis**, **Negative**, **Not explained**. Nothing here is a derivation of physics until a stage says what was derived and from what.

## 1. Where each item stands now (checked against the paper and toymodels v2 to v6)

| item | what exists | what is missing | status of the claim in the list |
|---|---|---|---|
| wave speed in dense regions | v2: with the paper's equation waves are faster where rho is high (Result `res:waves`); v3: written as a refractive index they slow down (Section `sec:second-gravity`) | a reason for the index form | partly out of date: the second pass already slows them |
| Lorentz invariance, the factor gamma | nothing; v6 Stage 2: a hopping wave is anisotropic at fourth order on the Chair44 and sub-ballistic on the Spectre (t^0.56) | an isotropic, linear dispersion at long wavelength and a moving clock that runs slow by gamma | correct, and v6 makes it harder |
| space factor B (light bending x2) | Result `res:bend`: a clock alone gives r_s / b, half the GR value; v6 Stage 4: the clock is the Franck-Condon rate c per crossed seam | a spatial factor from the model, not put in | correct |
| horizon | v3: random-walk escape on Z^3 gives 1/r and a horizon (Result `res:escape`); v5 Stage 5: the chair alone gives no clock; v6 Stage 4: the seam rate passes C7 to first order only | a self-consistent field: the bits source the field that sets their own overlap | correct |
| quantum Hamiltonian | v6 Stages 0 to 4: ledger sector exact, pair qubits, dipolar frustrated coupling, gauge form, polaron clock | the second read-out of the pair qubit | mostly resolved, as the list says |
| Bell without post-selection | v3: the two-ended tables are local with the detection loophole (Result `res:ch`); v6: Tsirelson is automatic, two-pair reductions reach 2.33 | outcomes and settings that are both defined on the tile, with nothing discarded | correct |
| RG for the monotile | Wilson-loop probe on hypercubic blocks, placement-dominated (Result `res:wl-place`) | a block map that uses the Spectre's own substitution | correct |
| arrow of time | v2: records at the frozen end, flow toward the Hat; v3: an external record store makes "determined", "recorded" and "hard to change" agree (Result `res:recordstore`); signed-area term odd under reversal (Result `res:areaterm`) | a microscopic reason for the sign of the area term; an initial condition | partly out of date: the records conflict is resolved, the sign is not |
| entropy | v4: counting and resolved entropies of the bits fall along the flow in 62 of 85 cells (Result `res:flow-entropy`) | an entropy that grows in a dynamics | correct |
| number fields | Result `res:field`: the inflation field is Q(sqrt6, sqrt10); the icosian E8 lives in Z[phi]; the lift is Eisenstein | a structure carrying both | correct; note the inflation field is not Q(zeta_12) |
| the 192 layout | v5: one layout among several, labelled a Hypothesis; it forces an odd number of live bits (v5 E5.17) | a rule that selects a layout | correct |

## 2. Stages (cheap and exact first)

| stage | content | size | what would count as progress | stop if |
|---|---|---|---|---|
| 0 | (**done**) **scoreboard** `t7_targets.py`: each missing feature as one computed number from the existing modules with its target (deflection coefficient 1 -> 2; largest CHSH without discards -> above 2; fourth-moment index -> isotropic; entropy production -> positive; ...). Every later stage moves one entry | S | a single place where the gaps are numbers | a target cannot be stated as a number |
| 1 | (**done**: two pairs pass with tunnelling pulses, 2.454 with visibility; patch clusters do not, 1.988) **Bell without post-selection.** The pair qubit has a second read-out the tile already supplies: its own dynamics. With `H_pair = -Gamma tau^x`, reading "which tile is the bump" after a delay t reads `cos(2 Gamma t) tau^z + sin(2 Gamma t) tau^y`. So a setting is a waiting time, the outcome is the bump, and no run is discarded. Test on two coupled pairs and on the Stage 1 clusters of v6: largest CHSH over delays, its window along the flow, and whether the delays needed are compatible with locality (no signalling across the pair graph, Lieb-Robinson of v6 Stage 2) | M | CHSH above 2 with settings and outcomes both defined on the tile, nothing discarded | the delay family reaches only one plane of the Bloch sphere and that plane holds no violation; or no-signalling fails at the needed delays |
| 2 | (**done**: E8's Coxeter element carries the Eisenstein and golden structures together; the inflation field meets them only in Q; lambda^2 = 4 + sqrt3 sqrt5; both rotations need rank 16) **number fields.** The Coxeter element of E8 has order 30; Q(zeta_30) has quadratic subfields Q(sqrt5), Q(sqrt-3), Q(sqrt-15), so E8 with its Coxeter element carries the icosian and the Eisenstein structures at once. Exact checks: build it, find the A2 + A2 and H4 sublattices it preserves, and test whether the inflation field Q(sqrt6, sqrt10) (which contains sqrt15) meets Q(zeta_30) beyond Q | S | an explicit object with both structures, or a proof that the inflation cannot be one of its automorphisms | (none; either answer is a result) |
| 3 | (**done**: the chair's three-fold axis selects one layout, 12 x 13 + 6 x 6, not v5's; it is neutral, so no 1/r from the layout) **the layout.** Enumerate the decompositions of 192 into the point counts of the tile (42 curved, 14 straight, 6 group, and others the tile supplies) and filter by stated rules: the chair's C3, one group per side, and an even number of live bits (the pair rule of v4 and v6). Does any layout survive, and is it unique? | S | a rule that selects the layout, or a proof that none does | the rules are chosen to pick an answer |
| 4 | (**done**: the bit lengthens its own edges as 1/c, the opposite of GR's need; coefficient 0.950 when the field sets the opening, wrong order b^-1/2 when it sets the flow time) **the space factor.** The same bit changes the area of its tile by +-a h and, as a seam bit, the rate of a hop by c (v6 Stage 4). Read the area change as a change in the number of hops per unit length (a spatial factor B) and the rate as the clock factor A. Measure the deflection of a packet passing a source on a 2D lattice with both effects, and compare with (1 + gamma) r_s / b | M | the coefficient 2 with A and B from one coupling, or a stated reason why AB is not 1 | B needs a second free constant |
| 5 | (**done**: a second-order wave on the Chair44 has an isotropic long-wave cone to 5.6e-5, corrections falling as lambda^-2 with the axes slowest; a moving clock follows 1/gamma at long wavelength; the first-order hopping of v6 is the Galilean form) **Lorentz.** (a) A second-order-in-time wave (the leapfrog of v2) and a Dirac-type hopping on the Chair44 and the Spectre: front speed against direction, and the fourth-moment anisotropy against the inflation level (does it shrink with scale?). (b) A moving clock: a packet carrying an internal two-level system, internal phase rate against group velocity, compared with 1/gamma for the measured dispersion | L | anisotropy that falls with scale; time dilation that matches the dispersion | anisotropy does not fall; the Spectre stays sub-ballistic at every scale |
| 6 | (**done**: rule I has no horizon at any back-reaction (Gauss); the tile cannot represent c below 0.0609 (critical angle); with the escape coupling the horizon becomes a shell of maximally open bits at 1.0037 R_eff) **a self-consistent horizon.** Iterate: bits source the field (v5), the field sets the overlap (rule I), the overlap sets the bits' strength (h(c)). Is there a fixed point with c -> 0 at finite r? | M | a horizon that comes out of the iteration | no fixed point with c = 0, or one only for a tuned coupling |
| 7 | (**done**: every v6 term is time-reversal even and chirality is a parity, so no arrow from the tile; a quench from the frozen state grows entanglement symmetrically in time, an arrow set by the initial condition; low entropy kept at the frozen end) **entropy and the arrow in a unitary dynamics.** (a) Symmetry audit: the Spectre is chiral, which breaks parity, not time reversal; list every term of the v6 Hamiltonians and check which are T-odd (the area term needs a flux). (b) A quench: start the Stage 1 clusters in the frozen (product) state, evolve unitarily, and measure the entanglement entropy of a half; does it grow, and is the growth along the flow direction? | M | an entropy that grows from a stated initial condition, with the initial condition named as the input | the only T-odd term is one put in by hand (then say so) |
| 8 | (**done**: no pair lies inside a supertile; every pair of level L-1 is inherited by exactly one at level L; the pair set closes by a local rule; inherited couplings scale as lambda^-3, so they are irrelevant) **RG by substitution.** Block the pair TFIM of v6 by the Spectre's own inflation (level j -> j + 1): effective tunnelling and coupling per block, and whether the map closes (the couplings at j + 1 are a function of those at j). Abelian first; a non-abelian (SU(2)) version only if the abelian one closes | L | a closed flow with a fixed point | the map does not close at levels 2 to 4 |

Order: 0, then 1, 2 and 3 (exact or small), then 4 and 6 (the gravity side), then 5, 7, 8.

## 3. Hypotheses kept apart

- H7.1. The delay read-out of Stage 1 is a physical setting. In the tile model a delay is a time; that a party can choose it is the usual assumption of a Bell test, not something the tile supplies.
- H7.2. The area change of a tile is a change of spatial length (Stage 4). It could equally be a change of a density; the stage has to say which and why.
- H7.3. (revised after Stage 2) The relevant object is a rank-16 lattice with a Z[zeta_60] structure, where lambda^2 = 4 + sqrt15 is available; E8 with its Coxeter element carries the 3-fold and 5-fold structures but not the Spectre's 12-fold rotation together with the 5-fold one.
- H7.4. The Stage 3 rules (C3, one group per side, even live bits) are the right rules. They are stated before the count is run.

## 4. Rules (as in v2 to v6)

Define then compute; a test that can fail in every stage; a Negative result is a deliverable; single-file modules with `--tests-only`; numbers in the paper only as macros of `models/paper_numbers.py`; earlier toymodels stay as they are. Every stage updates the scoreboard of Stage 0.

```
research/toymodel_v7/
  PLAN.md
  t7_targets.py       Stage 0
  t7_bell.py          Stage 1
  t7_fields.py        Stage 2
  t7_layout.py        Stage 3
  t7_space.py         Stage 4
  t7_lorentz.py       Stage 5
  t7_horizon.py       Stage 6
  t7_entropy.py       Stage 7
  t7_rg.py            Stage 8
  STAGE0.md ...
  figures/
```
