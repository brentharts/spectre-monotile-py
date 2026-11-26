# toymodel_v12: the Hat Trick sphere as a closed substrate

Plan: `PLAN.md`. Needs a clone of `brentharts/hexasphere` (`export HEXASPHERE_DIR=...`). Each script runs its tests first; `--tests-only` stops there. Labels as in `quantgravalt.tex`.

| script | stage | content | checks | result file |
|---|---|---|---|---|
| `t12_pairs_sphere.py` | S0, S1 | flat vertex of a hat on the kite complex of `GP(3,0)`; pairs, hole and split cases; dipole couplings on the sphere | 6 | `results_pairs_sphere.json` |
| `t12_flip_graph.py` | S2 | all tilings by exact cover, rotor regions, bit strings, every flip with `(dV, dE, dF)` and contact changes | 6 | `results_flip_graph.json` |
| `t12_hamiltonian.py` | S3 | the fixed 30 x 30 dipole matrix on the icosahedron edges; exact classical ground state of the all-0 tiling; exact diagonalisation at fixed tiling | 4 | `results_hamiltonian.json` |
| `t12_quantum_geometry.py` | S3, S4 | quantum geometry bits with flip tunnelling, edge qubits with the dormant rule; flip channel and flip energies | 5 | `results_quantum_geometry.json` |
| `t12_evolution.py` | S3, S4 in time | exact evolution of the 12 geometry qubits at `kappa = 0`: signal of a flip, OTOC, entropy from a product state, back-reaction of an excited matter slot | 5 | `results_evolution.json`, `log_evolution.txt` |
| `t12_curvature.py` | S5(a) | geodesics on the flat hat metric (the icosahedron), deflection at a cone point | 6 | `results_curvature.json` |

## Decisions (2026-11-25)
* **D1.** On the sphere `h`, `J`, `kappa` are free parameters, not functions of the flow time (at the Hat end of the planar flow `h = 0` and the bit is closed). **Definition.**
* **D3.** Deflection at a cone point is reported as a lensing signature of a conical defect in 2+1 dimensions, not as a Newtonian or 3+1 result. **Definition.**

## Results

**The tiling space is a 12-bit register at the cone points (Exact, exhaustive).**
* `GP(3,0)` with its 12 pentagons as holes has exactly 4096 = 2^12 hat tilings (exact cover with no symmetry imposed, search exhausted at 40,951 nodes). The earlier count "at least 1000, unexhausted" in qwla `gp_hat_scan.py` is superseded.
* The 480 kites split into 12 rotor regions of 40 kites, one around each pentagon, the same in every tiling. A region is tiled by 5 hats in exactly two ways, independently of the others. A tiling is a string `sigma` in `{0,1}^12`.
* `sigma_p = 1`: the flat vertices of the 5 hats lie on the pentagon (the hat's side of length 2 is a pentagon side). `sigma_p = 0`: they lie at the midpoints of the 5 icosahedron edges at `p` (the 2-fold axes).
* The two icosahedral (and tetrahedral) tilings are the constant strings. The counts with symmetry `C5`, `C3`, `C2` (16, 16, 64) are `2^(number of pentagon orbits)` = `2^4`, `2^4`, `2^6`.

**Flips are closed clusters (Exact, all 24,576 edges of the 12-cube).**
* A flip changes one bit: 5 hats out, 5 hats in, on the same 40 kites. Every flip is closed. This is the first closed flip cluster of the project (the planar engine reached none).
* For a flip 0 to 1 at `p` with `k` of its 5 neighbours in state 1: `(dV, dE, dF) = (-k, -k, 0)`. So `V = 140 - n11`, `E = 210 - n11`, `F = 72`, with `n11` the number of icosahedron edges whose two ends are both 1 (a 4-valent vertex sits on such an edge). The all-0 tiling is trivalent.
* `(0, 0, 0)`, the count of a Pachner 2-2 move, occurs only when no neighbour is in state 1 (768 flips); the others are not Pachner counts. The labelled contact graph changes in every flip.

**Pairs (Exact, all 4096 tilings).**
* A bump-and-dent pair (two flat vertices at one point) exists on the icosahedron edge `pq` iff `sigma_p = sigma_q = 0`. At most 30 pairs (the all-0 tiling: every hat is in a pair), none in the all-1 tiling (all 60 flat vertices face a hole).
* Edge types: 00 a pair qubit, 11 a 4-valent vertex, 01 neither. A flat vertex facing a hole has no partner: its bump would change the area of the hole (**not modelled**).

**Light at a cone point (Numeric 1e-7; exact by unfolding).**
* The flat hat metric with the pentagons filled is the intrinsic metric of the regular icosahedron (27 kites per face), cone points of deficit 60 degrees at its vertices.
* Two parallel geodesics passing a cone point on opposite sides cross at 60 degrees for every impact parameter `b` (0.02 to 0.4 of half an edge), at distance `2b = b / sin 30`. With no cone point between them they stay parallel.
* No 1/b fall-off, no focal length, no clock effect: the bending is holonomy of a flat metric. In the 2+1 dictionary each cone point is a mass of 1/12 of the total that closes the sphere.

## The model (Definition) and S3, S4

Geometry: one qubit per pentagon, basis the 4096 tilings. Matter: 30 qubit slots on the icosahedron edges, slot `pq` present iff `sigma_p = sigma_q = 0`, `n_e = (1 - sigma_p)(1 - sigma_q)`.

    H = -Gamma sum_p X_p + g sum_p (1 - sigma_p) - J sum_e n_e X_e - kappa sum_{e<e'} n_e n_e' W_ee' Z_e Z_e'

`X_p` is the phason flip of rotor `p`; a slot keeps its state while it is switched off (the dormant rule). `Gamma`, `g`, `J`, `kappa` are free (D1). `W` is one fixed matrix, because the two hats of a pair do not depend on the other rotors; its largest entry is scaled to 1.

**S3, classical (Exact, all 2^29 states).** The all-0 tiling (30 pairs): 45% of the couplings are antiferromagnetic; the ground state leaves `f = 0.363` of the bond weight unsatisfied (0.368 with great-circle distance) and is 60-fold degenerate up to the global flip (30-fold with great-circle distance). **Negative** for H12.3: closing the surface does not lower the frustration (plane: 0.209 at 28 pairs, bound 0.294 at 804).

**S3, quantum at fixed tiling (Exact diagonalisation, 5 to 15 pairs).** Caps of 4, 6, 7, 8 rotors in state 0. At `J = 1`, `kappa = 0.3` the gap is 0.90 to 1.53 and the half-system entropy 0.19 to 0.82 bit; at `kappa = 1` the entropy reaches 2.48 bits for 15 pairs; for `kappa/J >= 3` the ground state is the cat doublet (1 bit, splitting below 1e-4), as in the plane.

**S3, joint model.**
* `kappa = 0` (**Exact**): every slot's `X_e` is conserved, and with all slots in `|+>` the model is a 12-qubit model on the icosahedron, `H_geo = -Gamma sum X_p + g sum n_p - J sum_<pq> n_p n_q`. Matter and geometry are in a product state. At `g = 5J/2` this is the ferromagnetic transverse-field Ising model on the icosahedron with coupling `J/4`: the two icosahedral tilings are its two ordered states, and at `Gamma = 0.2` the ground state is their cat (probability 0.46 each, 1.00 bit across a 6|6 cut of the cone points, 14.8 +- 14.8 pairs).
* `g = 0`: the all-0 tiling is favoured (each pair gains `J`). Its probability is 0.98, 0.63, 0.05 at `Gamma = 0.2, 1, 3`; the mean pair number 29.9, 27.7, 17.8; the mean vertex number 140.0, 139.9, 138.2. So the number of matter qubits and the number of vertices of the tiling are quantum observables with a spread.
* `kappa = 0.3` (**Numeric, variational**: slots restricted to at most 2 in `|->`, 1.79 million states): geometry and matter become entangled, 0.003, 0.039, 0.068 bit at `Gamma = 0.2, 1, 3` (`g = 0`). The truncation is checked at fixed tiling against exact diagonalisation: error per pair 2e-6 (5 pairs), 0.004 (10), 0.011 (18); the weight outside the no-excitation sector is 0.18 to 0.28, so at 30 pairs these numbers are bounds, not converged values. At `g = 5J/2` the coupling breaks the symmetry between the two icosahedral tilings in favour of the paired one (`Gamma = 1`: probability 0.20 against 0.0003, 21.6 pairs on average, 0.20 bit between geometry and matter).

**S4, the flip channel (Exact for the rule; Numeric for the energies).**
* `X_p` is a permutation of basis states: unitary, its own inverse, flip then reversed flip returns the state (overlap 1). No pair-count conservation is needed. On a closed space there is no boundary term.
* Energy of a flip at `Gamma = 0`, `kappa = 0.3`, all 49,152 ordered flips, by the number `d` of pairs destroyed: relaxed cost 1.005, 2.05, 3.15, 4.32, 5.55 for `d` = 1..5 (exactly `d J` at `kappa = 0`); the sudden cost is higher by at most 0.25. Created pairs enter in `|+>` and gain exactly `J` each at once; relaxation gains up to 0.58 more. 1,536 flips switch no pair and cost nothing.
* The geometry change of the same flip is `(dV, dE, dF) = (-k, -k, 0)` (S2). So one flip has a matter cost set by its neighbours in state 0 and a vertex change set by its neighbours in state 1.

## Time evolution at `kappa = 0` (Exact; `t12_evolution.py`)

In a fixed matter sector `x_e = +-1` the model is `H_geo[x] = -Gamma sum X_p + g sum n_p - J sum_e x_e n_p n_q` on 4096 states. `J = 1`, times in `1/J`. The icosahedron has diameter 3, so every "front" below is three or four points.

* **A flip spreads in order of distance.** OTOC `C_d(t) = 1 - Re tr(Z_q(t) Z_0 Z_q(t) Z_0)/4096` at `Gamma = 1`, `g = 2.5`: it reaches half of its maximum at `t` = 2.75, 4.0, 4.5 for graph distance 1, 2, 3 (time step 0.25), about 1.1 edges per unit time between distance 1 and 3. At `t = 1` it is 0.088, 0.0006, 0.0000; at `t = 6` it is 0.94, 0.80, 0.73. There is no sharp cone: the early growth is a power of `t` that rises with distance.
* **Response of the pair presence to the flip `X_0` of the ground state.** The first rise arrives later on farther edges for every parameter set tried (classes (1,1), (1,2), (2,2), (2,3): 0.30, 0.50, 0.80, 1.00 at `Gamma = 1`, `g = 2.5`; 1.00, 1.20, 1.85, 2.05 at `Gamma = 0.3`; 0.10, 0.15, 0.25, 0.30 at `Gamma = 3`). The rate of this first rise scales with `Gamma`; it is a cruder measure than the OTOC (time step 0.05, half of a first small maximum).
* **At `g = 5J/2` the rotor occupations do not respond at all** (the symmetry 0 <-> 1 keeps `<n_q> = 1/2`); only the pair presence does. At `g = 0` both respond (rotors: 0.35, 0.65, 0.90 at distance 1, 2, 3, `Gamma = 1`).
* **Entropy from the product state `|all-0>`** (6 cone points against 6, maximum possible 6 bits): at `Gamma = 1`, `g = 2.5` it is 1.04, 2.21, 2.64 bits at `t` = 1, 3, 6, peaks at 2.73 and has long-time mean 2.47 (minimum 1.96 over `t` in 50..400). Other sets: long-time mean 0.52 (`Gamma = 0.3`, `g = 2.5`), 2.01 (3, 2.5), 0.48 (1, 0), 1.70 (3, 0). **Negative** for an arrow: `S(t) = S(-t)` to machine precision (real `H`, real initial state), and the entropy stays far below the 6-bit maximum.
* **Back-reaction of matter on geometry.** Put one slot in `|->` (`x_e = -1`): the term on that edge changes sign and pairing there costs energy. Change of `<n_p>` at distance 0, 1, 2 from the edge: `-0.078, -0.011, -0.0025` (`Gamma = 3`, `g = 2.5`), `-0.050, -0.0012, -0.00005` (`Gamma = 1`, `g = 0`), `-0.086, -0.0068, -0.0009` (`Gamma = 3`, `g = 0`): local, falling by a factor of 5 to 40 per edge. At the symmetric point with small tunnelling (`Gamma = 0.3`, `g = 2.5`) it is `-0.485` at every distance: one matter excitation selects the all-1 tiling out of the cat of the two icosahedral tilings, at an energy cost of 0.0005 (`-14.6` pairs, `-14.6` vertices). At `Gamma = 1`, `g = 2.5` it is in between (`-0.29, -0.22, -0.20`).

Reading (**Definition**): matter that is excited changes the local geometry (fewer pairs, more 4-valent vertices), and the change is short-ranged away from the symmetric point. It is not a 1/r law and the graph is too small to fit one.

## Not done
Convergence of the joint model at `kappa != 0` (needs more than 2 excitations or an icosahedral symmetry basis); the point `g = 2.5`, `kappa = 0.3`, `Gamma = 0.2` (near-degenerate doublet, Lanczos did not converge); time evolution at `kappa != 0`; S5(b) (moving holes); S6. A flat vertex facing a hole has no partner and no bit here. The couplings use centroids of the unrelaxed kite complex. `Gamma` and `g` are not derived from anything.
