# The Spectre–Hat Collinearity Flow

`spectre_hat_collinearity_flow_model.py` moves one editable tile (`spectremodel.Tile`) between the
curved Spectre and the straight Hat with a single parameter `tau` in `[0, 1]`. This note says what
the flow is, what it needs from the geometry of `spectremodel.py`, and where it differs from the
first sketch of the idea.

![The flow at tau = 0, 0.2, ..., 1 (corner_deg = 40 so the opening is easy to see)](figures/spectre_hat_flow.png)

## Idea

The Hat is `Tile(1, √3)` and has 13 vertices. The Spectre family starts at `Tile(1, 1)` and has a
14th vertex. Going from one to the other should not be a snap: the 14th vertex should fade out
because its turning angle goes continuously to zero, at which point it is an ordinary point of a
straight side and the polygon has 13 corners. The curved edges of the Spectre should go straight at
the same time.

(That the Hat is `Tile(1, √3)` and the Spectre is `Tile(1, 1)` is taken from the notes; this
document and the code check the geometry of the model, not that identification.)

## The flow

| quantity | law | tau = 0 (Spectre) | tau = 1 (Hat) |
|---|---|---|---|
| edge lengths `a`, `b` | `a = 1`, `b = 1 + (√3 − 1)·tau` | `b = 1` | `b = √3` |
| curve strength `k` | `k0·(1 − tau)` | `k0` | `0` (straight edges) |
| turn of the 14th vertex `w` | `w0·(1 − tau)` | `w0` (default 30°) | `0` (flat) |
| arrow angle | `π·(1 − tau)` | `π` | `0` |

The arrow angle is a parameterisation from the notes; nothing in the outline measures it.

### The 14th vertex and its edge directions

In the standard layout of `spectremodel.py` the 14 edge *directions* are fixed; `a` and `b` only
change lengths. Consequently every vertex turns by the same angle at every `tau` (all multiples of
30°), and exactly one vertex is flat from the start: **P₈**, where edges 7 and 8 both point along
`(0, −1)`. With only `a` and `b` changing, the polygon has 13 corners for every `tau`, so nothing
collapses.

To get a real 14 → 13 collapse the flow *opens* P₈. The chord `P₇ → P₉` stays where it is and P₈ is
pushed outward, perpendicular to it, by

    h(tau) = a · tan(w(tau) / 2).

Then, for every `tau`:

* edge 7 points along `−90° + w/2` and edge 8 along `−90° − w/2` (their directions change),
* both have length `a / cos(w/2)`,
* the turn at P₈ is exactly `−w` (clockwise, like every convex corner of the tile),
* P₇ and P₉ turn by `−60° + w/2` instead of `−60°`; the three changes cancel, so the total turn
  stays `−360°` and the outline stays closed,
* no other vertex moves.

At `tau = 1`, `w = 0`, P₈ is back on the chord and the tile is the straight `Tile(1, √3)`.

With `corner_deg=0` the opening is switched off and the flow is just the plain `Tile(a, b)` family
with P₈ flat all along.

### Values for the default flow (`k0 = 0.5`, `w0 = 30°`)

| tau | b | k | w (deg) | h | length of edges 7, 8 | corners | area |
|---|---|---|---|---|---|---|---|
| 0.00 | 1.0000 | 0.500 | 30.0 | 0.2679 | 1.0353 | 14 | 8.4641 |
| 0.25 | 1.1830 | 0.375 | 22.5 | 0.1989 | 1.0196 | 14 | 9.6361 |
| 0.50 | 1.3660 | 0.250 | 15.0 | 0.1317 | 1.0086 | 14 | 10.9259 |
| 0.75 | 1.5490 | 0.125 | 7.5 | 0.0655 | 1.0021 | 14 | 12.3328 |
| 1.00 | 1.7321 | 0.000 | 0.0 | 0.0000 | 1.0000 | 13 | 13.8564 |

The area is `2√3·a² + 3ab + √3·b² + a·h`; the last term is the triangle added by opening P₈.

## Facts about `spectremodel.py` that the flow relies on

* **Curvature does not change corner angles.** The handle offset `d/2 − k·n` is the chord `d`
  rotated by the fixed angle `atan(2k)`, the same for both edges at a vertex. So at every vertex the
  curve turns by exactly the polygon's turning angle, for every `k`. `k` only turns each edge into
  an S-shaped curve about its chord. The curve therefore has a corner at P₈ exactly when the polygon does.
* **The untwisted area ignores `k`**: each S-curve encloses zero net area against its chord, so the
  area equals the polygon area (`Tile.area()` is exact for the Bézier edges).
* The vertex count is read from turning angles (`effective_vertex_count`, polygon or curve tangents);
  it is 14 for `tau < 1` and 13 at `tau = 1` when `w0 > 0`.

## Differences from the first sketch

* The sketch's coordinate table `V_k(tau)` and its cross product `C(tau) = (√3/4)(1 − tau)` do not
  match the edge vectors of `spectre.py` / `spectremodel.py`, which have only multiples of 30° as
  turns; no vertex there has a 150° interior angle. The model's own version of the same quantity,
  the cross product of the two edges at P₈, is `C = −(a² + h²)·sin(w)`. It is non-zero for `tau < 1`,
  `0` at `tau = 1`, and shrinks monotonically, but it is not linear in `tau`.
* The default `w0 = 30°` reproduces a 150° interior angle at the 14th vertex as in the sketch, but it
  is a choice: the opened P₈ is a deformation added by the flow, not a corner of the original Spectre
  polygon of `spectremodel.py` (where P₈ is flat). At `tau = 0` the tile is therefore the plain
  `Tile(1, 1, k0)` only if `corner_deg=0`.
* The direction in which P₈ opens (outward) and the choice to keep `P₇ → P₉` fixed are design
  decisions. Other closure-preserving openings exist.

## Usage

```python
from spectre_hat_collinearity_flow_model import SpectreHatFlow

flow = SpectreHatFlow(k0=0.5, corner_deg=30.0)   # tau = 0
flow.tau = 0.4                                   # moves b, k and the opening of P_8 together
x, y = flow.tile.outline(60)                     # flow.tile is a live spectremodel.Tile
flow.effective_vertex_count()                    # 14 here, 13 at tau = 1
flow.vertex_report()                             # turn of each vertex (polygon and curve)
tiles = flow.frames(8)                           # independent tiles for tau = 0 ... 1
```

Setting `tau`, `k0` or `corner_deg` re-lays the tile out, so vertices moved by hand go back to the
layout; twists and `data` are kept.

Command line:

```
python3 spectre_hat_collinearity_flow_model.py --tests-only
python3 spectre_hat_collinearity_flow_model.py --frames 6 --corner 40 --no-show --save figures/spectre_hat_flow.png
python3 spectre_hat_collinearity_flow_model.py --tau 0.5 --report --no-show --save /tmp/t.png
```

## Tests

`--tests-only` runs 14 self tests (all pass). They check, among other things:

* the end points: `Tile(1, 1, k0)` with `corner_deg=0`, and the straight `Tile(1, √3)` at `tau = 1`;
* the linear laws, and that `tile` is live while `tile_at` and `frames` are independent;
* edge directions `−90° ∓ w/2`, lengths `a / cos(w/2)`, closure, and that only P₈ moves;
* turns at P₇ and P₉, and that the total turn is `−360°`;
* 14 corners for `tau < 1` and 13 at `tau = 1`, for polygon and curve tangents;
* curve corners equal polygon turns for every `tau` and `k`;
* area `= polygon formula + a·h`, for several `k`;
* input validation (failed sets leave the state unchanged).

## Open questions

* Is the opened 14th vertex the right model of the Spectre's extra vertex, or should the flow start
  from the polygon of `spectre.py` and use another vertex or direction change?
* The arrow angle is not tied to the geometry yet. A measurable version (for example the tangent
  direction at the two ends of the collapsing edge pair) would turn the notes' singularity picture
  into something the tests can check.

## What later models established about the flow (added 2026-11-10)

Labels as in `QUANTUM_GRAVITY.md`. Tests are in `research/toymodel_v2/` (`STAGE0.md` to `STAGE4.md`).

* **Exact:** the tiles `Tile(1, b)` for `b` from 1 to √3 (Spectre to Hat) are one integer walk in the 4D lift; the bond graph of a patch (406 seams at level 2) does not change along the way, only the relative scale of the two planes of the lift. At the Hat the positions fall onto one hexagonal lattice (rank 4 to rank 2). `tm2_lattice.py`.
* **Exact:** the opening of the 14th vertex by `h(tau)` used in this note is *not* a member of that family: on a tiling, opening every tile alike breaks 88 of 406 seams. It closes every seam only when two tiles that share their flat vertex open as a bump and a dent. So the first open question below has a partial answer: the opened vertex is consistent on a tiling only pairwise.
* **Exact:** the laws continue past the Hat: `h` is odd and the overlap of the two bit states is even about `tau = 1`. The turn at P₈ changes sign under a mirror image while the jump and every cost of the chain do not. This is the measurable version of the arrow angle asked for in the second open question: the signed turn at P₈, `−ω(τ)` for bit +1. `tm2_arrow.py`.

## A reading of the flow: past, potential and checking (added 2026-11-10)

Your idea, as given:

> The Past is still right here next to you and far away is the future: past vs potential, greater space as greater potential checking, less space as more past checking. What if just checking if the past is consistent with "now" is itself a process, part of the forward flow of time? "Now" is enclosed by "3D space", the potential falling into past.

Everything below is a **Hypothesis** about how to read the models unless a row says Exact. The companion note is `ARROW_OF_TIME.md`, section 7.

| your statement | nearest thing in the models | what is Exact |
|---|---|---|
| the past is right here, the future far away | the frozen end of the flow (Spectre side, bit readable) placed at the particle, the Hat end at infinity; this is "reading B" of the overlap | only the algebra of reading B; the opposite placement (reading A) is the same algebra (`tm2_arrow.py`, E3.10) |
| less space, more past checking | two wings of n letters from a shared origin, with a condition at each end. Short wings: the two-ended check changes the distribution of the shared origin by δ = 0.315 (n = 1), 0.264 (2), 0.141 (4) | Exact (`tm2_rule.py`, E4.8, β = 2) |
| greater space, greater potential checking | long wings: δ = 0.030 at n = 8 and falling; the check passes at the chance rate and constrains only the ends, not the origin. The correlation between the ends goes to zero with it | Exact (same test) |
| checking the past is a process in the forward flow | (i) the bit histories behind one present carry 0.216 bit in all, 95% of it in the last three ticks; the frozen ticks cost nothing to check. (ii) the weight of histories that pass a two-ended check depends on both end settings | Exact (`tm2_rule.py`, E4.11, E4.7) |
| "now" enclosed by 3D space | the flow is one tile in the plane and the alphabet is six directions in a plane. Stage 5 adds one fact: the static field of a source falls like 1/r only on a three-dimensional sheet, and only then does the frozen end sit at the particle (reading B, to first order) | the dimension dependence is Exact (`tm2_waves.py`, E5.7, E5.8); that the sheet is 3D is **Not explained** |

Two consequences if "checking is the flow of time" is taken at face value, both computable and neither derived:

1. *A clock.* If a tick lasts as long as its check costs, the rate is the check entropy per tick: 0.0003, 0.002, 0.009, 0.030, 0.067, 0.107 bit at ticks 6 to 11, zero before. It rises with the overlap `c` (0.03 to 0.91 over the same ticks). Under reading B the overlap is the clock factor, so this clock also stops at the particle and runs far away: the same direction, not the same function (the ratio to `c` goes from 0.01 to 0.12). This is the one place where your two sentences support each other in the toy.
2. *A cost.* If the time a two-ended check takes is the inverse of its pass rate, that time depends on the settings at both ends (contrast 1.21 where the correlation first beats a shared-past model). The outcomes carry no signal, but a locally visible duration would. So the idea needs the duration of a check to be invisible at either end alone, or it allows signalling. That is a constraint to state, not a refutation.

A closer look at "less space, more past checking", with the closed form of the past shift and the three rates at which things fade, is in `PAST_AND_POTENTIAL.md`.

What would test it: a derived map from `tau` to distance (plan Stage 5); a model in which a tick's duration is computed, not assigned; a 3D version (the Chair44 side of the repository) before anything is said about "3D space".

