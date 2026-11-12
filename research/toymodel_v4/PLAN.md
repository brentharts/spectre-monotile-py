# toymodel_v4: use the outline, not the chain

Status: **Stage 0 done** (`t4_geometry.py`, 4 tests: what a flip does to the tile's geometry). **Stage 1 done** (`STAGE1.md`, `t4_arrow.py`, 5 tests: the robust arrows are the flow's own parameters in disguise; the direction stays an input; Negative). **Stage 4 done** (`STAGE4.md`, `t4_open.py` 4 tests, `t4_couple.py` 3 tests: the two open questions of the paper's Section 2, and coupled bits on a patch; Negative). **Stage 3 done** (`STAGE3.md`, `t4_spin.py`, 7 tests: the outline has a canonical spin structure, and it does not supply the sign of the fusion; Negative). **Stage 2 done** (`STAGE2.md`, `t4_patch.py`, 6 tests: the pairing rule holds on real Bezier outlines; one bit per flat pair; area is a transfer; counting entropy decreases toward the Hat). Stages 4 and 5 are a plan, not results. Labels as in `quantgrav.tex`: **Exact**, **Numeric**, **Negative**, **Hypothesis**, **Definition**. Nothing here connects to gravity or quantum mechanics until a test says so, and every number in the paper must come from code.

## 0. Why a v4, and what it is built on

toymodel_v2 and v3 tested the dynamic proposal of `quantgrav.tex` on two abstract objects: a hexagonal history sum, and a *chain* (a bit with a tunnelling overlap `c(tau)` from `spectre_hat_quantum_bit_model.py`). Neither contains a tile outline. `spectre_hat_time_model.py` (the figure `figures/spectre_hat_time.png`) does: for each `tau` it builds the actual Bezier outline with the 14th vertex P_8 pushed out or in by `h(tau)`, measures the two arrows on that outline, and tests snaps against rules R1-R5 on the outline (validity R3, smooth sector R4). v4 asks what that outline says that the chain could not.

Where v3 left the open problems of `quantgrav.tex` Section 'Open problems' (items 6 to 8):

| open problem | v3 verdict | what the outline can add |
|---|---|---|
| 6 an arrow of time | an area term `theta K` and records give an orientation, each put in | the outline has a native monotone area `A(tau)`, a curvature budget, and an asymmetry between the two bit values (Stage 0, 1) |
| 7 the sign of the fusion | a flux cannot supply it; a spin structure on the letters is needed | a closed outline has a canonical half-angle (spinor) lift of its tangent (Stage 3) |
| 8 clock and space | the clock needs an escape coupling; light bending needs a space factor `B` | one flow gives both an area (space) and a jump (clock) (Stage 5) |

The toy keeps its rules: the bit is classical, the direction of `tau` is imposed (D6), and the physics words are borrowed.

## 1. Stage 0 (done): what a flip does to the outline

`python3 research/toymodel_v4/t4_geometry.py --tests-only --story` (4 tests, about 3 s). Measured on the Bezier outline, `a = 1`:

| id | statement | label |
|---|---|---|
| E0.1 | `A(tau,+1) - A(tau,-1) = d(tau) = 2a tan(omega/2)`, for `omega_0` = 30, 60, 120 degrees and every `tau` tested; the two areas meet at the Hat | Exact |
| E0.2 | Total turning of both outlines is `-2 pi` (Gauss-Bonnet). Each curved edge is an S-curve with zero net turning, so all turning is in the 14 corners | Exact |
| E0.3 | A flip changes the corner turns at P_7, P_8, P_9 by `(-omega, +2 omega, -omega)` and nowhere else: zero net charge, zero dipole, a pure second difference | Exact |
| E0.4 | `s = +1` (convex corner) is a simple closed curve for every `tau` at `omega_0` up to 165 degrees. `s = -1` (reflex) is not near `tau = 0` once `omega_0 >= 135` degrees, for `k_0` = 0.3, 0.5, 0.8: the excluded stretch is `tau <= 0` at 135, about `<= 0.1` at 150, about `<= 0.15` at 165 (grid of 21 values) | Exact (grid) |

**What it says.**

- The bit is an *area* degree of freedom: a flip changes the area by exactly the jump `d`. In `spectre_hat_quantum_bit_model.py` the weight `w_j = h(tau_j)` (the area the bit adds) was introduced as a definition; here it is measured.
- A flip moves curvature without changing the total: it is a second difference `(-1, 2, -1) * omega` of the corner turns. The smooth sector cannot see a net charge, only this quadrupole.
- At large `omega_0` the geometry itself forbids `s = -1` at the Spectre end, so the early bit is forced to `+1` without any rule R2. That is the first thing in this project that looks like a *derived* boundary condition at the frozen end, and it is exactly where v3 found the records to sit (Result `res:records`).

**What it does not say.** It holds only for `omega_0` large; `omega_0` is a free parameter of the toy (the default is 30 degrees, the figure uses 120, the notes' arrow angle `pi(1-tau)` is `omega_0 = pi`, which the geometry cannot reach because `h = a tan(omega/2)` diverges). The exclusion is a statement about one family of Bezier outlines, not about the Spectre as a tile.

## 2. Order of work

Cheap and decisive first. Each stage ends with a note `STAGEn.md` and a test module `t4_*.py` with `--tests-only`, as in v2 and v3.

| stage | content | cost | decides |
|---|---|---|---|
| 1 | **done** (Negative): an arrow from the outline; which quantity is monotone along every admissible path | S | open problem 6 |
| 2 | **done**: many bits on a patch of real outlines: pairing rule, area budget, bits along the flow | M | whether the toy has a second law of its own (it has a counting entropy, and it points away from D6) |
| 3 | **done** (Negative): a spin structure from the tangent; does the outline supply the sign of the fusion? | M | open problem 7 |
| 4 | **done** (Negative, `STAGE4.md`, `t4_open.py`, `t4_couple.py`): the two open questions of the paper's Section 2 (entropy survives only where the reflex outline is valid at the Spectre end; no displacement of P_8 gives the relative sign -1) and coupled bits on a patch (pairs are geometrically independent; no ordering emerges) | M | whether coupling gives an ordering |
| 5 | one flow, two couplings: area as space factor, jump as clock | M | open problem 8 |
| 6 | paper: a section 'A third pass' and the figure | S | - |

### Stage 1: an arrow from the outline (open problem 6)

**Result: done, see `STAGE1.md`.** Reparameterising `tau` cannot matter, so the test used was stronger: each of the three flow parameters gets its own monotone law. Robust (monotone on every such path): the bit-averaged area `A0(b)`, the reflex tile's area and the jump `d`, which are the flow's own parameters in disguise. Law-dependent: the convex tile's area, the bending energy, the length. The direction of the flow stays an input (the failure condition below was met). The allowed bit values go 1, 2, 1 along the flow, and validity is one-way only outside a band of `omega` (E1.4). The text below is the plan as written before the stage was run.

Rule D6 forbids running `tau` backwards. Is there an arrow that does not depend on the parameter?

**Candidates, all measured on the tile.**

- The area `A(tau)`: `8.46` (Spectre, `omega_0 = 30`) to `13.86` (Hat), increasing (Stage 0 numbers).
- The jump `d(tau)`, decreasing; the relative area fluctuation `d/A`, decreasing.
- Structural asymmetry that needs no parameter: at the Spectre end the bit has two distinguishable values, at the Hat one (the map 'forget the bit' is two-to-one at one end and one-to-one at the other).
- The validity asymmetry of E0.4, when `omega_0 >= 135`.

**Tests.**
- Reparameterise `tau -> g(tau)` for monotone `g`: which of the above keep their ordering? (All monotone functions do, trivially; the point is which ones are *intrinsic*, i.e. defined without `tau`: area, perimeter, bending energy from `spectre_curvature.py`, distance to the Hat in the turning function.)
- Is the flow a gradient flow of one of them (steepest descent of a bending energy or of the turning-function distance to the Hat)? If the one-parameter family is a descent path, the direction is derived from the functional, with a sign convention left.

**Fails if** every intrinsic quantity is monotone only because the laws `b(tau)`, `k(tau)`, `omega(tau)` were written linear in `tau` (`test_laws_are_linear_in_tau` in the flow module says they are). Then the arrow stays an input and v4 says so (a Negative, like v2's audit).

### Stage 2: many bits on a patch (the pairing rule)

**Result: done, see `STAGE2.md`.** Edge matching on real Bezier outlines gives exactly the v2 rule (the failure condition below was not met): 6, 82, 804 flat pairs at levels 2, 3, 4, one bit per pair, area transferred between the two tiles. The number of open configurations is maximal at the Spectre end and 0 at the Hat, so a counting entropy decreases along the flow, the opposite orientation to 'later = more entropy' (and not even monotone for large `omega_0`). The text below is the plan as written before the stage was run.

v2 Result `res:pairs`: only flat vertices that meet pairwise can open, as bump and dent, with every edge still matching and the area unchanged. Now use the real outlines.

**Build.** Place `SpectreHatTime` tiles on the level-2 patch of `spectre_patch.py`; one bit per tile, `s_i` at its flat vertex; the compatibility rule is edge matching, checked on the outline with the R3/R4 machinery rather than assumed.

**Tests.**
- The number of compatible bit configurations as a function of `tau` (it must collapse at the Hat, where `d = 0` and all configurations coincide).
- The total area change: zero for a pair by E0.1 (`+d` and `-d`); non-zero for an unpaired flip, which therefore must be refused. Check this is what edge matching says, not only what the area says.
- The entropy `log(#configurations)` against `tau`: does the toy have a second law pointing the same way as D6, and is it independent of D6? (It will likely be the same orientation; if so, record that as a consistency check, not a derivation.)

**Fails if** edge matching on real outlines allows more or fewer configurations than the v2 combinatorial rule.

### Stage 3: a spin structure from the tangent (open problem 7)

**Result: done, see `STAGE3.md`.** The outline's tangent lift is a canonical antiperiodic spin structure, but it does not make the origin odd (E3.3), the sign pattern it fixes changes the outcome with the base edge (E3.4), and the seam-independent version is a product state (E3.5). The relative sign -1 between the bit values needs `omega = pi`, which the valid outlines never reach (E3.7: at least 47 degrees away). The failure condition below was met: a mechanism, not a value. The text below is the plan as written before the stage was run.

v3 Results `res:singlet` and `res:noflux`: the singlet follows exactly if the shared origin is odd under inversion, a flux cannot make it so, and a half-angle read-out of scalar amplitudes is branch-dependent. The missing thing is 'a spin structure on the letters'.

**Observation.** The tangent angle `theta(u)` along a closed outline has a unique continuous lift to the reals (up to a global constant), with total change `-2 pi` (E0.2). So the half-angle phase `exp(i theta/2)` is a well-defined function along the outline, with no branch choice, and it returns to minus itself after one circuit: the outline carries a canonical spin structure. At P_8 the two bit values give the phases `exp(-/+ i omega/2)`, whose ratio is `exp(-i omega)`: it equals `-1` exactly at `omega = pi`, the antiparallel-arrow point (a_out against a_in).

**Tests.**
- Build amplitudes for the two-wing model from the half-angle phases of the measured outline, and compute the singlet fidelity and the CHSH value. `res:noflux` gives the baseline to beat (fidelity 0 for rotation-invariant amplitudes).
- Check that the result does not depend on any sign choice (v3's `64` choices): that is the whole point of the lift.
- Vary `omega_0` and `tau`: where, if anywhere, is the relative sign `-1`?

**Fails if** the sign `-1` appears only at `omega = pi`, which the construction cannot reach (E0.4's companion: `h` diverges). Then the outline supplies the *mechanism* (a canonical spin structure) but not the *value*, and the sign stays an input; that is still a sharper statement than v3's.

### Stage 5 (was 4): one flow, two couplings (open problem 8)

v3: the clock needs an escape coupling `c^2 = 1 - h` with `h` the hitting probability of a random walk (transient substrate only); light bending needs a space factor `B = 1/A` that the model does not contain (a clock alone gives half the bending).

**Idea.** The flow already carries two scales: the area `A(tau)` (a space-like quantity) and the jump `d(tau)` (the clock-like readable fraction). With one source, ask whether a single static field fixes both, i.e. whether the pair `(A, d)` can play the roles of `(B, A_clock)` in `res:bend`.

**Tests.**
- Deflection `(1+gamma) r_s / b` with `gamma` derived from `dA/dd = 1` (E0.1) rather than chosen: what value results?
- The relation `d(tau)` against `A(tau)` along the flow (it is not linear): which of `gamma = 0`, `1` it is closer to, if either.

**Fails if** `gamma` is not a pure number from the construction (it depends on `omega_0`, `k_0`, the parameterisation). Then no space factor comes from the toy, and v3's Negative stands. This is the least likely stage to give a positive result; it is cheap, so it is included.

### Stage 5: paper

Add a short section after 'A second pass' with the Stage 0 to 4 results and numbers computed by `paper_numbers.py`; add the figure `figures/spectre_hat_time.png` (corner 120, snap scale 0.5) to the staged-tests discussion, with the E0 facts as a second panel; update the open-problem list.

## 3. Risks and honest limits

- **`omega_0` is a free parameter.** E0.4 needs `omega_0 >= 135` degrees; the default of 30 gives no exclusion. Any result that holds only above a threshold must say so.
- **One tile, one vertex, one bit.** Stage 2 is the first step towards many bits; none of this is a field theory.
- **The direction of `tau` is imposed.** Stage 1 may confirm that; a Negative there is a result.
- **Borrowed words.** 'Quadrupole', 'curvature charge', 'spin structure' and 'area' are geometry of a planar outline. They are analogies to Regge calculus and spinors, not derivations; the code tests only the geometry.
