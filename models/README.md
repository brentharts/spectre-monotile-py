# Models

Extended models of the curved Spectre monotile, built on the editable object model in `../spectremodel.py`
(`Tile`, `Vertex`, `BezHandle`). `spectre.py` stays as the untouched reference; nothing here modifies it.

**Status:** one model is written, `chair_fibers.py` (section 6); the others are a plan. This page fixes the
conventions, records the facts the models rest on (each checked, see below), and orders the work.

## Conventions

Every model is one file, `models/<name>.py`, and has four parts:

1. **A SymPy derivation**, exact and symbolic in `a, b, k` and the twists.
2. **A numeric bridge** (`lambdify`) that is cross-checked against `spectremodel.Tile`, so the symbolic model and
   the object model cannot drift apart.
3. **Self tests** (`test_*` functions and `run_tests`, as in the top-level modules).
4. **One or two matplotlib figures**, saved to `models/figures/`.

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
not known yet.

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

`python3 models/chair_fibers.py` runs the tests and shows two 3D figures; `--no-show --save-dir models/figures`
writes them. The figures are `models/figures/chair_fibers_scene.png` (the tile as a sheet above Chair44 with one fibre
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
