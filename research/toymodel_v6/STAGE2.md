# Stage 2: results (one hopping particle on the Spectre and Chair44 graphs)

Labels as in `PLAN.md`. Nothing here is light. File: `t6_hop.py` (5 tests, about 75 s), figure `figures/t6_stage2_hop.png`.

```
python3 research/toymodel_v6/t6_hop.py --tests-only
python3 research/toymodel_v6/t6_hop.py --no-show --save research/toymodel_v6/figures/t6_stage2_hop.png
```

## 1. The question

The paper's open problem 10 ("Clock and space"): on the Chair44 the walk tensor is isotropic and the hop count is cubic, so which of the two does a wave follow? The leapfrog wave of v2 was not run on the Chair44. Here the simplest quantum wave is: one particle with `H = L = D - A` (the graph Laplacian; `-A` as the control) on the graphs of `toymodel_v3/t3_escape.py`. The level-4 Spectre vertex graph has 28006 sites; the depth-5 Chair44 tile graph has 32768 tiles. Each run averages `|psi(t)|^2` over 60 bulk starting sites, two independent seeds.

## 2. Results

| id | statement | label |
|---|---|---|
| E6.12 | Long plane waves see the walk tensor: `<k\|L\|k> = k^T M k + O(k^4)` exactly. In the bulk M per site is isotropic to 0.998 (Spectre) and 0.995 (Chair44) | Exact |
| E6.13 | **The Spectre graph is isotropic and sub-ballistic.** Fourth-moment index 0.750 (the 2D isotropic 3/4; six-fold statistics force it), second moment isotropic to 0.95, rms ~ t^0.559 and t^0.558 on t = 4 to 24 for the two seeds | Numeric |
| E6.14 | **On the Chair44 the wave is cubic, and lags where the hop count leads.** Fourth-moment index 0.477 for both seeds, below the isotropic 3/5: weight along the diagonals. Mean radius at t = 3 is 9.0 to 9.5 in the axis cones, 12.3 to 12.8 in the face and body cones. The hop count per unit length (BFS) is 0.59 on the axes, 0.70 and 0.69 on the diagonals, so a hop-by-hop front would lead along the axes. The second moment is uniaxial, short axis 4 to 7 degrees from the chair's three-fold axis (1,1,1). The adjacency form gives the same index to 0.02 | Numeric |
| E6.15 | On the Chair44 the spreading is close to ballistic over the times the patch allows: rms ~ t^0.88 on t = 1 to 2.5 | Numeric (small patch) |
| E6.16 | No exact light cone: the amplitude at every graph distance is non-zero at any t > 0, and obeys the Lieb-Robinson bound `(\|\|H\|\| t)^D / D!` (\|\|H\|\| = 13.16; checked D = 1 to 12 at t = 0.3, largest ratio to the bound 0.063) | Exact |

## 3. What this says

1. **The answer to open problem 10, for a quantum particle: neither.** The walk tensor (isotropic) governs only long plane waves, which are not eigenstates on an aperiodic graph. The hop count (cubic, axes fastest) predicts the wrong sign. The actual packet is cubic, with the axes slowest, and uniaxial about the chair's own three-fold axis. Second moments cannot see a cubic anisotropy (any cubic-symmetric rank-2 tensor is isotropic), so the effect sits in the fourth moments, and the uniaxial part comes from the chair having only a three-fold axis.
2. **The planar tiling is isotropic but slow.** The Spectre's six-fold statistics force isotropy up to fourth order, and the spreading exponent 0.56 is between diffusion and a crystal: anomalous transport, typical of quasiperiodic structures. A clock or a light cone built on the Spectre graph would therefore not have a speed; distance would grow as a power of time.
3. **A Hamiltonian loses the exact cone of the leapfrog wave.** v2 had an exact light cone (E5.5). With hopping, causality is only the Lieb-Robinson bound, which is tight at short times and very loose at t = 1 (panel d).
4. **Consequences for the paper.** The paper's line "on the Chair44 the clock is isotropic and the hop count is not, so the question includes which of the two light follows" can now be answered for the toy: a hopping wave follows neither, and is anisotropic in the opposite sense to the hop count.

## 4. Not done

Larger Chair44 supertiles (depth 6: 8 times the tiles) to separate ballistic from anomalous; the spreading exponent on the Chair44 at longer times; the leapfrog wave of v2 on the same graphs (a classical wave may behave differently from the quantum one); a hopping particle coupled to the pair qubits (Stage 4).
