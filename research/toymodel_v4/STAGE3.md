# Stage 3: results (a spin structure from the tangent: does the outline supply the sign of the fusion?)

Labels as in `PLAN.md`. File: `t4_spin.py` (7 tests, about 2 s).

```
python3 research/toymodel_v4/t4_spin.py --tests-only --story
```

## What was built

toymodel_v3 left the sign of the fusion as an input: the two-wing state is the singlet exactly when the shared origin amplitude is odd under inversion (`t3_qubit.py` E4.4); a flux cannot make it odd (E4.5); and a half-angle read-out of scalar amplitudes depends on an arbitrary sign per letter (E4.6). The outline offers one candidate for a spin structure: the tangent angle has a unique continuous lift to the reals with total change -2 pi, so `exp(i theta/2)` is well defined along it and returns to minus itself after a circuit.

- Each of the six hexagonal directions sits on exactly one unopened edge of the outline (edges 0, 1, 4, 5, 10, 11), so a letter is an edge.
- Each edge gets the half-angle ket `(e^{-i Theta/2}, e^{i Theta/2}) / sqrt 2` with `Theta` the lifted chord angle from a chosen base edge.
- The state is `chi = sum_v a_v kappa_E(v) (x) kappa_E(v+3)`, the no-wing state of v3 with these kets; the origin amplitude `a_v` is the free input being tested.

## Results

| id | statement | label |
|---|---|---|
| E3.1 | The lift is antiperiodic (total change -2 pi, both bit values, several tau, omega_0 and base edges), so a seam cannot be avoided; the six directions sit on six different edges | Exact |
| E3.2 | Where the seam goes matters only through the six letter signs: over the 14 base edges there are 6 distinct sign patterns (up to an overall sign), one per gap between letters | Exact |
| E3.3 | **The outline does not make the origin odd.** A uniform origin and every even winding (w = 0, +-2, 4) have no singlet component for all 14 base edges and every tau tested | Exact, Negative |
| E3.4 | **The sign pattern is not a gauge.** For an odd winding the state is the singlet exactly or it vanishes, never anything else, but which depends on the base edge: for w = +-1 it vanishes for 1 of 14 base edges (seam between edges 10 and 11); for w = +-3 it is the singlet for all 14 | Exact, Negative |
| E3.5 | A seam-independent version (origin amplitude taken from the same lift, so its sign changes cancel those of the kets) gives one state up to a phase for all 14 base edges, and it is a **product** state: the covariant construction is v3's periodic equatorial construction with a uniform origin | Exact, Negative |
| E3.6 | The two arrows at P_8: overlap `cos(turn/2)`; the antisymmetrised pair has squared norm `2 sin^2(omega/2)`, is exactly the singlet for every tau < 1 and both bit values, and vanishes at the Hat (aligned arrows) | Exact (a definition of the pairing) |
| E3.7 | The relative half-angle phase of the two bit values is `exp(i omega)`, equal to -1 only at omega = pi. The s = -1 outline at tau = 0 is simple only up to omega_c = 133.0, 132.1, 120.5 degrees (k_0 = 0.3, 0.5, 0.8), so the relative phase stays at least 47, 48, 59 degrees from -1 | Exact (bisection), Negative |

## What it says

1. **A spin structure is available and it does not help.** The outline has a canonical antiperiodic lift. It fixes a sign per letter once a base edge is chosen, and the choice is not innocent: it decides whether an odd origin gives the singlet or cancels (E3.4). Making it seam-independent removes the effect and gives the v3 product state (E3.5). So the sign of the fusion stays an input, as in v3. The statement is sharper: the obstruction is not a missing spin structure but that every gauge-invariant combination reduces to periodic data on directions.
2. **The one place the arrows show an antisymmetry is the pair at P_8 (E3.6).** Any two distinct spin states antisymmetrised give the singlet, so E3.6 is algebra; what is new is the identification of the pair (past arrow, future arrow) with the fusion, the weight `sin^2(omega/2)`, and its vanishing at the Hat. That identification is a **Hypothesis**, untested here.
3. **E3.7 ties Stage 3 to Stage 0.** A relative sign -1 between the two bit values would need `omega = pi`, the antiparallel-arrow point. The geometry cannot reach it: the s = -1 outline stops being simple well before (E0.4 is the same fact seen from the other side). The failure condition set in `PLAN.md` was met: the outline supplies a mechanism, not a value.

## What is not done

- The letters use the six unopened edges at hexagonal directions; the odd-direction (green) edges and the two opened edges are not letters.
- No wing was added (v3's wings are unitaries on the six letters); with a common wing, E4.4 of v3 says an odd origin keeps the singlet, and E3.3 says no origin the outline supplies is odd.
- The pair of arrows was not turned into a full two-wing model with settings and a CHSH value.
