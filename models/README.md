# Models

Extended models of the curved Spectre monotile, built on the editable object model in `../spectremodel.py`
(`Tile`, `Vertex`, `BezHandle`). `spectre.py` stays as the untouched reference; nothing here modifies it.

**Status:** one model is written, `chair_fibers.py` (section 6), with `_tb.py` (section 7), `twist_hopping.py` (section 8) `hopf_shells.py` (section 9), `icosian_e8.py` (section 10) `e8_shells.py` (section 11) and `fibre_window.py` (section 11b below); the others, including the `quantum-lattice` plan, are a plan. Sources and results of the quantum-lattice / Hopf thread: `../QUANTUM_GRAVITY.md`. This page fixes the
conventions, records the facts the models rest on (each checked, see below), and orders the work.

## Conventions

Every model is one file, `models/<name>.py`, and has four parts:

1. **A SymPy derivation**, exact and symbolic in `a, b, k` and the twists.
2. **A numeric bridge** (`lambdify`) that is cross-checked against `spectremodel.Tile`, so the symbolic model and
   the object model cannot drift apart.
3. **Self tests** (`test_*` functions and `run_tests`, as in the top-level modules).
4. **One or two matplotlib figures**, saved to `figures/`.

Running `python3 models/<name>.py` runs the tests and then shows the figures; `--tests-only` and
`--no-show --save PNG` work as in `spectre_operator.py`. Results are labelled *exact* (derived and tested),
*numeric* or *hypothesis*, as in the top-level README. Shared helpers (symbols, the `lambdify` bridge, drawing
code, one colour scheme: red = out handle, blue = in handle) live in `models/_common.py`. Custom data goes in the
`data` dicts of `spectremodel`; models define the keys they use at the top of their file.

Not wired into the Makefile yet.

## What the models rest on

Checked on 2026-11-07. Fact 1 is covered by the self tests of `spectremodel.py`. Facts 2 and 3 were checked
against `Tile` with random twists (SymPy 1.14 for the formula; agreement to 1e-12 or better). Fact 4 was checked
symbolically for the untwisted offsets only; its consequence for twists follows from it and is not yet tested
on a real patch (that is model 4).

**1. Only the total twist of a handle matters.** A handle's geometry depends on `vertex.twist + handle.twist`
alone, so there are 28 effective twist parameters, not 42. The vertex twist is a convenient common-mode
coordinate for the two handles at a vertex.

**2. Corner turns.** Relative to the untwisted tile, the turn at a vertex changes by
`theta_out - theta_in` (totals). A pure vertex twist turns both tangents together, so it does not change any
corner angle; it rotates the corner rigidly. Handle twists bend it.

**3. Area is a sum over edges.** For an edge of chord length `L`, strength `k`, and total twists `t1` (out handle)
and `t2` (in handle), the signed area between the curve and its chord (positive when the curve bulges to the
right of its direction of travel, which is into the interior of the clockwise tile) is

    lens(t1, t2) = 3 L^2 / 80 * [ (4 k^2 + 1) sin(t1 - t2) - 4 (sin t1 - sin t2) - 8 k (cos t1 - cos t2) ]

and `|polygon area + sum of lens over the 14 edges|` equals `Tile.area()`. It vanishes for `t1 = t2` (the
point-symmetric S of `spectre_operator`), which is why the untwisted tile keeps the area
`2 sqrt3 a^2 + 3ab + sqrt3 b^2` for every `k`.

**4. Gluing rule.** If tile B runs a shared edge backwards (both tiles are traversed clockwise, so it does) and
both use the same `k`, then B's untwisted out handle at Q equals A's untwisted in handle at Q, and B's in handle
at P equals A's out handle at P. So the two curves coincide exactly when the total twists agree handle by handle
(A.out with B.in at P, A.in with B.out at Q). This extends the README finding that per-edge amounts tile only
when they are equal. It is shown here for the local pair; the placement of real tiles comes from `spectre_patch`
(all placements there are proper rotations, so no mirrored copies enter).

## The models

| # | File | Question | Part |
|---|------|----------|------|
| 0 | `_common.py` | shared symbols, bridge, drawing | support, no figures |
| 1 | `twist_edge.py` | what does one twisted edge do, exactly? | foundation |
| 2 | `fiber_colors.py` | vertex colours and fibre connections as data | your first example |
| 3 | `flow_tube.py` | handles as arrows and pinched tubes | your second example |
| 4 | `twist_gluing.py` | which twist patterns still tile? | tiling |
| 5 | `patch_flow.py` | direction and flow across several tiles | stretch |
| 6 | `chair_fibers.py` | fibres from the tile's vertices and handles to the Chair44 pyramid tips | **written** |

### 1. `twist_edge.py`: one twisted edge, exactly

*SymPy (exact):* the edge as a cubic in the two total twists; end tangent angles; the end curvatures; the
`lens(t1, t2)` formula above; point symmetry (it follows from the handle offsets that the S stays point-symmetric
exactly when `t1 = t2`; to be tested); the corner-turn rule as a symbolic identity.

*Figures:* (a) a grid of one edge over `(t1, t2)` with both lobes shaded by sign and the handles drawn;
(b) a heat map of `lens/L^2` over `(t1, t2)` with the zero line `t1 = t2`.

*Tests:* the symbolic edge equals `Tile.sample_edges`; `lens` equals the quadrature in `Tile.area`; the corner
rule equals the measured turns.

### 2. `fiber_colors.py`: custom data on vertices

*Data (no geometry):* `vertex.data["color"]` is an RGB triple; `vertex.data["fibers"]` is a list of
`{"to": j, "weight": w}` entries, a graph on the 14 vertices.

*SymPy (exact):* a fibre from vertex `i` to vertex `j` is a cubic with end directions taken from the handles
there (out handle at `i`, in handle at `j`), so vertex twists steer the fibres. Its smoothness measure
`integral |B''|^2 dt` is a polynomial integral and has a closed form in the endpoints, directions and speed
parameters; the minimiser over the speed parameters is explicit. Colour along each Bezier edge blends the two
vertex colours with a symbolic ease function.

*Numeric:* the fill inside the tile is the harmonic extension of the boundary colours, solved on the mesh of
`spectre_fem.TileMesh`. It stays inside the range of the boundary colours (maximum principle, to be tested).

*Figures:* (a) the tile filled by vertex colours with the fibres drawn and weighted; (b) the same tile before and
after vertex twists, showing the fibres re-route while the outline stays on its vertices.

### 3. `flow_tube.py`: arrows that flip, pinched tubes

The idea: a line cannot show direction or a second kind of twist, a tube can. There are two different twists
here, and the plan keeps them apart.

* **In-plane twist** (already implemented): the handle rotates about its vertex. It changes the outline.
* **Roll** (new option, `roll` plus `roll_deg`, per vertex and per handle, adding like `twist`): the handle
  rotates about its own axis. It does **not** change the outline, so it is a display and data option. A flat
  ribbon with roll `phi` seen from above has width `r |cos phi|`: it **pinches** at `phi = pi/2 + m pi`, and the
  visible face (front or back) **flips** at each pinch.

*SymPy (exact):* roll as a Bezier of its own: the four values `(roll at P, out, in, at Q)` blend with the same
Bernstein weights as the edge, `phi(t)`. Pinch positions are the roots of `cos phi(t)`; the number of pinches
along an edge is exact for linear `phi` (`floor` of the half-turns) and the roots of a cubic in general. The
tube outline is `x(t) +- r |cos phi(t)| n(t)` with `n` the unit normal of the edge.

*Flip:* a separate per-handle sign `data["flow"] = +1 or -1` that reverses the arrow. It carries no geometry.

*Figures:* (a) one edge drawn as a tube for total roll 0, 90, 180, 360 and 540 degrees, with the pinches and the
front/back colours; (b) the whole tile with a tube along every edge and arrow heads at the 28 handles, coloured
by flow sign.

*Needs:* `roll` added to `spectremodel.Vertex` and `BezHandle` (about 20 lines, validated like `twist`, not used by
`outline`). The tube geometry lives in `_common.py` so other models can reuse it.

*Hypothesis to test in model 4:* roll obeys the same gluing rule as the in-plane twist, so pinch patterns should
line up across a seam.

### 4. `twist_gluing.py`: which twist patterns tile

By the gluing rule, the tiling condition is linear in the total twists: two handles must have equal totals.

*Identical copies* (every tile has the same twists): build the graph on the 28 handles from
`SpectrePatch.edge_pairs()`. The number of connected components is the dimension of the compatible twist space,
the twist counterpart of `compatible_classes()` (which gave the uniform line for the amounts `k_i`). The answer is
answered while writing model 8: for the levels 1 to 3 the 28 handles fall into exactly
**two** components, which are the vertex twists alpha (even vertices) and beta (odd vertices); see section 8.

*Free twists per tile on a finite patch:* the same equalities give a sparse integer matrix, and SymPy's exact
nullspace gives the number of free parameters per level (1, 9, 71, 559 tiles).

*Figures:* (a) the 28-node handle graph with its components coloured; (b) a patch with a compatible twist pattern
that fits, next to an incompatible one with its overlap and gap lobes, in the style of `spectre_patch.png`.

*Tests:* the mismatch area of an incompatible pattern against `lens`; a rasterised patch as in `spectre_patch`.

### 5. `patch_flow.py` (stretch): direction and flow across tiles

Every tile outline is traversed clockwise, so on every shared edge the two tiles run in opposite directions: an
arrow along the boundary is antiparallel across each seam unless one side is flipped. A patch-wide flow therefore
needs the flip sign of model 3.

*Model:* a signed flux on every shared edge, with each tile's net flux equal to zero. This is a linear system over
the rationals from `edge_pairs()`; its solution space is the cycle space of the dual graph (tiles as nodes, shared
edges as links), whose dimension SymPy gives exactly per level. Boundary edges of the patch carry flux in or out.

*Figures:* (a) a level-1 patch (9 tiles) with a tube and arrow on every seam, coloured by flux; (b) the flow loops
as closed paths through the tiles.

### 6. `chair_fibers.py` (written): fibres to the Chair44 pyramid tips

`python3 models/chair_fibers.py` runs the tests and shows two 3D figures; `--no-show --save-dir figures`
writes them. The figures are `figures/chair_fibers_scene.png` (the tile as a sheet above Chair44 with one fibre
per vertex and handle) and `chair_fibers_twist.png` (what a vertex twist does to a fibre).

**What the tips are.** The actual Chair44 solid carries 192 small square pyramids, 8 on each of its 24 exposed unit
squares: height `j/10000` with `j = 1..12`, a bump (`+j`) or a dent (`-j`) of the same depth. A tip is a pyramid's
apex. The table (`models/data/r44_features.csv`, exact fractions) is an extract of Tsiokos' `six-birds-tiles` data
(CC BY 4.0, see `models/data/README.md`). The tests check it against the statements of the paper (each magnitude
8 bumps and 8 dents, marks more than 1/5 from every panel edge, apex at `j/10000` along the outward normal) and the
24 panels against `chair44_core.py`.

**The fibre (exact, SymPy).** From a base point `X` (a vertex, or the end of a handle) along the unit direction `u` of
that handle (a vertex uses its out handle, so twists steer it) to the tip `T`, arriving along the pyramid axis against the
panel normal `n`: the cubic with control points `X, X + s0 u, T + s1 n, T`. With `D = T - X`, `alpha = u.D`,
`beta = n.D`, `gamma = u.n`, `dd = D.D` the energy `integral |B''|^2 dt` is exactly

    E = 12 [ dd - 3 alpha s0 + 3 beta s1 + 3 (s0^2 - gamma s0 s1 + s1^2) ]

(checked against quadrature to 1e-12), a strictly convex quadratic, so the smoothest fibre with `s0, s1 >= 0` is
explicit (interior point, a boundary point, or the origin). For a tip on an up-facing panel and a sheet above it this
becomes the **twist law**

    E(phi) = 12 dd - 9 beta^2 - 9 max(0, alpha)^2,    alpha = |D_h| cos(phi - bearing),    E_min = 3 |D|^2,

so the best direction of a vertex is the bearing of its tip, and a vertex pointing away from its tip (`alpha <= 0`) does
not steer its fibre at all (`s0 = 0`, one fibre for all such directions). A vertex twist does not move the vertex, so
the law is exact in `theta`; a handle twist also moves the handle's end (the base point), so for handles it holds at
the actual point. The right panel of the twist figure shows the law (curves) against the full pipeline after setting
the twist on the `Tile` (dots).

**The pairing is a rule, not a fact.** A tile vertex has no natural partner among the tips. `assign_fibers` pairs the 42
bases (14 vertices, 28 handles) one to one with distinct tips so that the total energy is minimal (Hungarian algorithm
on the exact 42 x 192 matrix) and stores it as `base.data["fibers"] = [tip role]`. Any other pairing can be written
into the same data, several fibres per base included. The placement of the sheet (default: scale 0.5, centred over the
chair, height 3.2) is a free choice too. For `a = b = 1`, `k = 0.5` the total energy is 490.0, and all 32 up-facing
tips are used by the 42 bases while the other 10 bases wrap round to side faces; this count depends on the placement and
is only an observation. Nothing here tests the thesis of the top-level README.

*Not yet explored:* tips are drawn at their true positions, so their heights are invisible and only the sign and
magnitude `j` show (colour and marker size); whether those relate to anything on the tile side (the 12 magnitudes
against the 12 orientations of the Spectre, say) is open and untested.

## Plan: what to take from `quantum-lattice`

Read-only survey of `extras/` (`ql_tilings.py`, `ql_tiling_spectra.py`, `ql_chair44_chains.py`, `ql_common.py`, and the
two docs `TILINGS.md`, `HOPF_INSULATOR.md`). Nothing was copied or run from that repo; the one check below was run here.

### What is there

| Piece | What it does | Needs |
|---|---|---|
| `ql_tilings.planar_patch` | vertex graph of a level-n Hat / Spectre / Turtle patch | the separate `spectre` repo (`spectre_energy.Patch`) |
| `tight_binding`, `hopping_values`, `adjacency` | sparse H on the graph: `unit`, `inverse` (t0/length), `gauss` hopping, optional onsite | numpy, scipy |
| `to_pyqula` | wraps a point set and a sparse matrix as a pyqula `Hamiltonian` | pyqula (vendored there) |
| `ql_tiling_spectra` | dense spectrum, DOS, IPR, bipartite test, trace identity | numpy, matplotlib |
| `chair44_cell_model`, `chair44_labels_at` | face-adjacent cells of a depth-n Chair44 supertile, labels by digit inflation, onsite `lam * pyramid / 36` | `symchair44.py`, `chair44_twoadic.py` of the `spectre` repo |
| `ql_chair44_chains` | 1D chain with the pyramid height read along a line; Lyapunov exponent, N*IPR | the cell model above, scipy |
| `ql_common.multihopping_from_hk` | exact real-space hoppings from any periodic Bloch H(k) by FFT | numpy |
| `ql_berry_bundle`, `ql_hopf_insulator` | Chern number by Wilson loop / projector transport / FHS; Hopf invariant of a 3D two-band model | pyqula for the cross-check only |

### Licence and weight (decide before copying)

- `quantum-lattice` carries GPL-3.0 (`LICENSE.md`), and it vendors `pyqula`, which is Joaquin Lado's. The `extras/`
  files are in one commit by you; the pyqula tree is not yours.
- This repo has no `LICENSE` file in its working tree. Copying GPL code into it would need that settled first. I did
  not check what the upstream `spectre-monotile-py` says.
- The tight-binding pieces are about 30 lines each over numpy and scipy. **Recommendation: rewrite them here as
  `models/_tb.py` (no pyqula, no numba, no jax, no PySide6), and keep `to_pyqula` as an optional export.** Rewriting
  also removes the dependency on the separate `spectre` repo, see the check below.

### Checked here: `spectre_patch.SpectrePatch` replaces the `spectre` dependency for the planar part

Vertex graph built from `SpectrePatch(level, a, b).straight_polygon(i)` with vertices merged at 5 decimals:

| level | V | E | F | V - E + F | largest eigenvalue (unit hopping) |
|---|---|---|---|---|---|
| 1 | 78 | 86 | 9 | 1 | 2.3942 |
| 2 | 518 | 588 | 71 | 1 | 2.4288 |
| 3 | 3734 | 4292 | 559 | 1 | 2.4340 |

These match `TILINGS.md` (V=3734, E=4292, F=559 and E in [-2.434, 2.434] at level 3). Hat `(1, sqrt3)`, Spectre `(1, 1)`
and Turtle `(sqrt3, 1)` give the same V, E, F at every level, and their unit-hopping spectra agree exactly (difference
0.0, as the adjacency is the same). The merge tolerance is a choice; a different one was not tried.
**Not available here:** the Chair44 cell model. `symchair44` and `chair44_twoadic` are not in this repo, and
`chair44_core.py` is a different construction. Whether `chair44_core.py` can produce the same labels is untested.

### 7. `_tb.py` (written): tight-binding helpers

`patch_graph(level, a, b)` (one site per tiling vertex, one bond per tile edge, from `SpectrePatch`), `tight_binding`
(`unit`, `inverse`, `gauss` hopping, per-bond `values`, `onsite`), `spectrum`, `ipr`, `dos`, `summary`, `to_pyqula` (optional,
needs pyqula). Written from the definitions, no code taken from `quantum-lattice`. 22 self tests (`python3 models/_tb.py
--tests-only`, about 20 s); five of them were mutation-tested (wrong hopping rule, asymmetric adjacency, wrong IPR power,
no bipartite check) and each mutation fails a test. `python3 models/_tb.py --no-show --save-dir figures` writes
`figures/tb_tilings.png` (DOS of the three tilings for both hoppings, and N * IPR of the Spectre states), and prints:

| level 3, 3734 sites | hopping | E range | mean N * IPR |
|---|---|---|---|
| Hat, Spectre, Turtle | unit | [-2.434, 2.434] | 14.7 (all three) |
| Hat | 1/length | [-2.133, 2.133] | 84.4 |
| Turtle | 1/length | [-1.965, 1.965] | 58.7 |

The unit-hopping row and the energy ranges equal `TILINGS.md`. For 1/length hopping `TILINGS.md` gives N * IPR 82.4
(Hat) and 57.7 (Turtle) against 84.4 and 58.7 here. A mean over eigenvectors is not basis independent where eigenvalues are
degenerate, and `eigh` may mix a degenerate space differently in two programs, so this small difference is not
alarming, but I did not show that this is the cause. The merge tolerance for vertices (5 decimals) is a choice; the test
checks that 4 and 7 give the same graph at level 2.

### 8. `twist_hopping.py` (written): hopping that follows edge length

An edge is a Bézier arc, so `t = t0 / arclength` depends on `k` and on the twists of its two handles. `arclength(k, t_out,
t_in)`, `edge_lengths(tile)` (adaptive quadrature split at the slowest point; a fixed 40-point Gauss rule was 7e-8 off for
an edge whose speed fell to 1e-3), `glued_tile(alpha, beta, k)`, `gluing_classes(patch)`, `meeting_mismatch(patch, tile)`,
`patch_model(level, tile, hopping)`, `tile_ring(tile)`. 29 self tests (`python3 models/twist_hopping.py --tests-only`, 2 s); seven mutations of the code were
tried and six are caught by the tests; the seventh (a shifted owner edge) was missed until the owner test was
strengthened, and is caught now. `python3 models/twist_hopping.py --no-show --save-dir figures` writes
`figures/twist_hopping_tile.png` and `figures/twist_hopping_patch.png`.

Results (details and figures in `../QUANTUM_GRAVITY.md`):

- **Exact (tested, levels 1 to 3):** the twists that tile are exactly vertex twists, alpha on even and beta on odd vertices
  (two components of the 28 handles). Placed control points of all meeting pairs agree to 3e-14; a single extra handle twist
  breaks it.
- **Exact:** `length(k, t_out, t_in) = length(k, t_in, t_out)` (reversing an edge swaps the handles; checked on 120 random
  cases to 1e-12). So on a patch that tiles all 14 edges have the same length `L(alpha, beta)`, and the spectrum with
  `t = 1/length` is the unit-hopping spectrum divided by `L` (1e-12). With `L >= 1` the largest eigenvalue cannot exceed
  the unit one (Perron-Frobenius). The hoped-for effect, twist changing the *shape* of the spectrum on a tiling, does
  not happen through arc length.
- **Numeric:** for `k = 0.5` the length ranges from 1.012 to 1.709 over (alpha, beta), 1.185 untwisted.
- **Where shape can change:** a single tile that does not tile (`tile_ring`: only the edges whose handles are twisted change
  hopping), or a hopping rule that sees more than an edge length, such as a corner angle. Under the glued family no corner
  angle changes either, so a rule on the turning angle would need non-glued twists too. This is the next question.

### 9. `hopf_shells.py` (written): 24-cells and Hopf fibres of the 600-cell and E8

Quaternion code (arrays `(w, x, y, z)`), the 120 icosians, the 24 Hurwitz units, the 240 E8 roots as Hurwitz pairs, recognisers
for a 24-cell and a 600-cell shell, Hopf circle fibres of 2I, the fibres of the E8 roots (right orbits of 2T), the Hopf map `S^7
-> S^4`, the Coxeter H4 projection and the fibre-compatible projection `a + q b`. 33 self tests (`python3 models/hopf_shells.py
--tests-only`, 1 s); seven mutations were tried and each is caught (some by a crash rather than a failed check). Figures:
`figures/hopf_shells_600cell.png`, `figures/hopf_shells_e8.png`. Results are in `../QUANTUM_GRAVITY.md`; in short: 2I is 5
24-cells; the 600-cell's Hopf circles are 12 decagons, 20 hexagons, 30 squares over an icosahedron, dodecahedron and
icosidodecahedron; the E8 roots are 10 fibres of 24 (24-cells) whose Hopf base points on S^4 are the 5-cross-polytope; the Coxeter H4
plane gives twin 600-cell shells that cut every fibre, while `f = a + q b` (`q` = phi^{+-1} times one of the 16 half-units) gives
twin 600-cells that are 5 + 5 whole fibres. Not shown: that this is the Elser-Sloane construction, or any link to the tile.

### 10. `icosian_e8.py` (written): E8 from the icosians, compared with the Hurwitz construction

`sigma` (Galois conjugation on icosian coordinates), `icosian_roots()` (the 240 vectors `sqrt(2/5)(sqrt(3-phi) x, sqrt(2+phi) x^sigma)`
for `x` in 2I and phi 2I), `is_e8_root_system`, `right_fibres`, `find_isometry` (backtracking over inner products and the
same-fibre relation, vectorised, about 5 s) and `plane_cosines` / `matching_q`. 17 self tests (`python3 models/icosian_e8.py
--tests-only`, 7 s); six mutations were run and are caught, and a seventh (an isometry search that ignores fibres) is not a clean
failure, since it only makes the search run for many minutes. Result: the icosian and Hurwitz constructions are one fibred
root system; the icosian parallel and perpendicular planes are the `a + q b` planes with `|q| = 1/phi` and `|q| = phi`.
Figure `figures/icosian_e8.png`; details in `../QUANTUM_GRAVITY.md`. Not built: shells of E8 beyond the roots and the papers'
selection of whole fibres.

### 11. `e8_shells.py` (written): shells of E8, whole fibres, arithmetic counts

Exact arithmetic in Z[phi] (`zmul`, `zconj`, `znorm`, `zdivide`), the ideal-divisor sum `ideal_sum`, `shell_classes(n)`, the E8
lattice shells `e8_shell(n)` (Hurwitz pairs), `fibre_labels`, `alpha_of` (via the isometry of `icosian_e8`), `model_set_gaps`.
45 checks (`python3 models/e8_shells.py --tests-only`, 6 s); seven mutations (wrong ideal norms, wrong shell sum, dropped lattice
condition, one-sided fibre action, wrong cone, a single split prime) are all caught. Results are in `../QUANTUM_GRAVITY.md`: shells are
`240 sigma_3(n)` vectors in `10 sigma_3(n)` fibres; the classes `alpha = n + b phi` are the integers in a cone; class sizes
`120 S(alpha)`; radii constant on a fibre, so ball windows select whole fibres; a window `sigma(alpha) < phi` gives the
Fibonacci gaps `{1, phi}`. Not built: the 4D point set with the papers' own (polytope) window.

### Candidate models (hypotheses until a test says otherwise)

| # | File | Idea | Status |
|---|---|---|---|
| 7 | `_tb.py` | graph, hopping, onsite, spectrum, IPR in numpy/scipy; plus pyqula export | **written**, see below |
| 8 | `twist_hopping.py` | Hopping that follows the arc length of the curved, twisted edge | **written**, see below; the result is a rescaling, not new spectra |
| 9 | `fiber_onsite.py` | Use model 6 as the onsite term: each base has an assigned tip (`data["fibers"]`), so take `V = fibre energy` or the tip's signed height `sign * j / 10000` as its onsite energy, on the 14-cycle of one tile and on a patch where the tile's vertices are shared | Hypothesis. A patch vertex is shared by several tiles, each with its own fibre; the rule for combining them (sum, mean, first) is a free choice. The normalisation `pyramid / 36` in `quantum-lattice` is a per-cell sum I have not re-derived, so heights are not comparable yet |
| 10 | `twist_phase.py` | Read twist as a Peierls-like phase on a bond, `t * exp(i phi_e)`, with `phi_e` the total twist difference along the edge; then compute the flux through each tile and the spectrum | Hypothesis, and the weakest link: twist rotates a handle, it is not a gauge field on the graph. Test first whether the flux per tile is zero for every gluing-compatible pattern (model 4); if it always is, the phase is pure gauge and the spectrum cannot change |
| 11 | `chair_chain_tiles.py` | Read pyramid heights along lines of the Chair44 and compare with the 192 tip heights of `r44_features.csv` | Blocked on the Chair44 labels (see above). The finding to reproduce is `TILINGS.md`: N*IPR about 4-13 on tiling lines against 300+ shuffled, Lyapunov exponents not separating |
| 12 | `holonomy.py` | Treat the twist as a frame carried along the outline and compute its holonomy round the 14-cycle with the Wilson-loop code of `ql_berry_bundle` (a product of rotations, no pyqula needed) | Exact expectation to test: the holonomy of a pure-rotation frame is the sum of the twists mod 2 pi, so it ties directly to the "only totals matter" fact. This is the one place the bundle idea of open question 1 gets a concrete form |

### Connections looked for and not found

- **Hopf fibration and the "fibres" of model 6.** Hopf fibres are circles in S^3 over points of S^2 and come from a 3D
  periodic two-band Hamiltonian `H(k)`. The tile has no `k`, and the Chair44 is aperiodic. I found no map between them;
  the word "fibre" is shared, nothing more. Chern and Hopf numbers need a periodic lattice, so they apply to a Spectre
  approximant only if one is defined, which this repo does not do.
- **`multihopping_from_hk`.** Periodic input only. Skip.
- **`ql_hopf_insulator`.** No tile data goes in. Skip unless a periodic tile model appears.

### Questions for you

1. Licence: should models stay free of GPL code (rewrite, as recommended), or do you want to relicense this repo?
2. Is the `spectre` repo (`symchair44`, `chair44_twoadic`) one you want this repo to depend on, or should the Chair44
   labels be brought in as data (like `r44_features.csv`)?
3. Order: I would do 7 and 8 first (small, and 8 is new), then 12, then 9. 10 only after 4 has been written.

## Order of work

1. `_common.py` and model 1 (small, and every other model uses its pieces).
2. `roll` in `spectremodel`, then models 2 and 3 (your two examples; independent of each other).
3. Model 4, then model 5 if the flow picture looks worth it.

A step is done when its tests pass, its figures are saved, and the table above says so.

## Later ideas

`spectre_diffract`, `spectre_fem` and `spectre_curvature` take per-edge amounts, not control points. Giving them
a control-point entry would let the twisted tile go through the same exact checks (Porod perimeter, heat
content, bending energy). The total turning of the twisted outline should stay `-2 pi` while it stays simple,
because the twists cancel in the telescoping sum; this is an expectation, not yet tested.

## Open questions

1. **"Fibre".** The plan reads it as threads between vertices (a graph inside the tile). If you mean a fibre in
   the bundle sense (a value attached to each point, like the fibre clock in `spectre_cosmology.py`), the data
   schema in model 2 changes and the SymPy part becomes a connection and holonomy model instead.
2. **Roll.** Is a display-only axial twist the pinched tube you had in mind, or should the pinch come from
   something that also changes the outline?


### 11b. `fibre_window.py` (written): which E8 windows keep the Hopf fibres whole
E8 cut-and-project sets in 4D with a ball window, for the icosian splitting and the Coxeter-plane splitting of Quantum_Gravity_Lab.
Icosian ball windows never split a fibre, Coxeter ones do; the two splittings have the same radial statistics; density matches the volume
prediction. 11 self tests (`python3 models/fibre_window.py --tests-only`, about 7 s). Figure `figures/fibre_window.png`.
Results in `../QUANTUM_GRAVITY.md`. Not built: the Elser-Sloane polytope window.

### 11c. `voronoi_window.py` (written): the Voronoi-cell (Elser-Sloane-style) window of E8
Builds the 19440-vertex Voronoi cell of E8, projects it to the 4D perpendicular space (720-vertex, nearly spherical polytope) and tests which
fibres it keeps whole: all, in the icosian splitting, because the window is invariant under the fibre group; not in the Coxeter splitting.
10 self tests (`python3 models/voronoi_window.py --tests-only`, about 35 s). Figure `figures/voronoi_window.png`. Attribution to Elser and
Sloane is a hypothesis; their paper was not read.
