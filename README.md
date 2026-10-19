# Spectre Monotile

The Spectre Monotile project provides a simple Python tool to visualize a geometric tile pattern based on given parameters a, b, and curve_value.

## Requirements

The dependencies can usually be installed using `pip`:

```pip install numpy matplotlib opencv-python bezier```

The `bezier` package is optional: if it is not installed (it needs a compiler on some platforms), `spectre.py`
automatically falls back to `bezierpy.py`, a small pure-Python implementation with no dependencies.
`python3 bezierpy.py` runs its self test.

## Usage

To use the Spectre Monotile tool, navigate to the project directory and run:

```python3 spectre.py <a> <b> <curve_value>```  
Replace `<a>`, `<b>`, and `<curve_value>` with your desired values.

For example, to draw a spectre tile with parameters a=1, b=1, and curve_value=0.5, use:

```python3 spectre.py 1 1 0.5```

![spectre_monotile](https://github.com/Jan-Piotraschke/spectre-monotile-py/assets/78916218/706cfb54-81a1-43da-b587-6c6afe15ee08)

## Getting Help

To view a help message and learn about the available parameters and options, use:

```python3 spectre.py --help```

### Hidden Nerd Stuff

You can draw a shape or something like our spectre monotile using Fourier series.

To use an image:  
```python fourier_drawing.py your_image.png --num_components 100```

To use the Spectre Monotile shape (default parameters):  
```python fourier_drawing.py```

To specify parameters for the Spectre Monotile:  
```python fourier_drawing.py --a 1 --b 1 --curve_strength 0.5 --num_components 50```

![fourier_drawing](https://github.com/user-attachments/assets/849d0357-0caa-4a70-b838-bb69ca33907a)

### Symbolic version (SymPy → LaTeX/PDF)

`symspectre.py` computes the curved monotile exactly and symbolically in `a`, `b` and the curve strength `k`
(vertices, Bézier control points, closed-form curves, area, tangent angles) and writes a LaTeX document, compiled
to PDF if `pdflatex` is installed. The numeric values only affect the TikZ figure.

```python3 symspectre.py --a 1 --b 1 --curve_strength 0.5 --outdir out```

Use `--no-pdf` to only write the `.tex` file, and `--verify` to cross-check the symbolic curves against `spectre.py`
(needs `numpy`).

### Per-edge curve control

`spectre_operator.py` builds the curved monotile with a separate curve amount for each of the 14 edges, plus one
`scale` that multiplies them all (the effective strength of edge `i` is `scale * curves[i]`):

```python
from spectre_operator import curved_spectre
x, y = curved_spectre(a=1, b=1, curves=1.0, scale=0.5)          # same as spectre.py with k=0.5
x, y = curved_spectre(curves=[1, -1] * 7, scale=0.4)            # alternating edges
x, y = curved_spectre(curves={3: 1.0, 7: -1.0}, scale=0.6)      # only edges 3 and 7
```

Run `python3 spectre_operator.py` to execute its self tests and show a figure with the edge numbering and a set of
variations (`--no-show --save out.png` to write the figure instead of opening a window).
The area stays `2√3·a² + 3ab + √3·b²` for any amounts; very different amounts on neighbouring edges can make the
outline cross itself (`is_simple` checks this).

![spectre_operator variations](figures/spectre_operator_variations.png)

The figure shows the edge numbering (top left) and per-edge variations; edge colour is red for positive and blue for
negative curve amounts. Rebuild it with `make figures`.

### Distance fields: inner and outer space

`spectre_fields.py` computes the signed distance to the tile boundary (negative inside, positive outside). Its level
sets are the inner and outer offset curves, a one-parameter family sweeping through the original shape.

```python
from spectre_fields import DistanceField
f = DistanceField.tile(a=1, b=1, curves=1.0, scale=0.5)   # same arguments as spectre_operator
f.level_curves(-0.1)         # inner offset curve(s)
f.region_area(0.2)           # area of the tile dilated by 0.2
f.inradius, f.incenter       # largest inscribed circle
segments, radius = f.medial_axis()
```

![distance fields](figures/spectre_distance_fields.png)

What the self tests check:

- the first-order growth of the offset areas is the **perimeter** (coarea formula), and the perimeter has the exact
  closed form `spectre_operator.perimeter` = `sum |d_i| F(k_i)`;
- the second-order term depends only on the **corner turns**: `A(+r) = A + P r + c_out r²`, `A(-r) = A - P r + c_in r²`
  with `c_out + c_in = π + 2 + √3` for equal amounts on all edges (the area itself never changes with `k`);
- the **medial axis** peaks at the inscribed circle, and the distance field agrees with the exact point-to-segment
  distance of the straight tile.

Very different amounts on neighbouring edges make sharp corners and shrink the range of `r` where the small-`r`
formulas hold.

### Fields on the filled tile: torsion, drum modes, heat

`spectre_fem.py` meshes the solid tile with triangles and solves three classical problems with linear finite
elements (numpy + `scipy.sparse` only, no mesh library). The area is the same for every curve amount, so every
difference below is due to the shape alone.

```python
from spectre_fem import TileMesh
m = TileMesh.tile(a=1, b=1, curves=1.0, scale=0.5, h=0.05)   # same arguments as spectre_operator
tor = m.torsion()              # phi, J = 2*integral(phi), shear stress |grad phi| per triangle
lam, modes = m.eigenmodes(6)   # Dirichlet drum modes: -lap u = lambda u, u = 0 on the outline
Q = m.heat_content([0.01, 0.1, 1.0])   # heat left after cooling from u = 1 with the outline at 0
```

![finite element fields](figures/spectre_fem_fields.png)

What the self tests check:

- the solver reproduces the exact **disc** and **square** results (torsional rigidity, eigenvalues, heat content);
- the tile obeys the **Saint-Venant** bound `J <= A^2 / (2 pi)` and **Faber-Krahn** `lambda_1 >= pi j01^2 / A`
  (the disc of equal area is the extremal shape) and the Polya bound `lambda_1 J / (4 A) < 1`;
- the early **heat content** is `Q(t) = A - 2 P sqrt(t / pi) + O(t)`: cooling feels the perimeter from step 1
  (`spectre_operator.perimeter`), while late cooling decays like `exp(-lambda_1 t)`;
- the torsion shear stress is **singular at the reflex corners** (it keeps growing as the mesh is refined, so
  its maximum is shown, not trusted to high accuracy), and is zero at the centre;
- the numbers converge with the mesh size, are unchanged by rotating the tile, scale exactly with `a, b`
  (`lambda ~ 1/s^2`, `J ~ s^4`), and the two chiralities `k` and `-k` have lowest tones that differ by about 1 %.

### Curvature, turning function and bending energy

`spectre_curvature.py` works out the curvature of the curved outline in closed form. For an edge with chord `d` and
strength `k` the numerator of the curvature collapses to a line, `kappa(t) = -18 k (1 - 2t) / (|d| (s'^2 + k^2 w'^2)^(3/2))`,
so every edge is an S with exactly one inflection point (at its middle) and the curvature is odd about it.

```python
from spectre_curvature import curvature_profile, total_absolute_curvature, bending_energy, max_curvature
prof = curvature_profile(a=1, b=1, curves=1.0, scale=0.5)   # kappa, tangent angle theta, corner turns along the outline
total_absolute_curvature(1, 1, 1.0, 0.5)    # exact: sum |corner turns| + sum 4 arctan(2|k_i|)
bending_energy(1, 1, 1.0, 0.5)              # integral of kappa^2 ds over the smooth edges
```

![curvature](figures/spectre_curvature.png)

What the self tests check:

- the closed form agrees with finite differences of the Bezier edges; every edge has **zero net curvature** and its
  absolute curvature integrates to exactly **`4 arctan(2|k|)`**;
- the corner turns obtained from the Bezier control points agree with the step-1 formula, and the **Umlaufsatz** holds:
  the total turning of the clockwise tile is exactly `-2 pi`;
- the total absolute curvature of the straight tile is `16 pi / 3` (8/3 of the convex value `2 pi`, because of the
  reflex corners), and curving adds `4 arctan(2|k|)` per edge, independent of `a` and `b`;
- the bending energy agrees between the closed-form quadrature and a sum along the outline, has the right small-`k` limit,
  and scales like `1/s` when lengths are scaled by `s` (while the turning angles do not change at all);
- the sharpest bend is on the shortest edge, and `k` and `-k` are exact mirror images (`kappa` changes sign, nothing else).

### Makefile

```
make test        # self tests: bezierpy.py, spectre_operator.py, spectre_fields.py, spectre_fem.py, spectre_curvature.py
make verify      # symbolic (SymPy) curves vs the numeric spectre.py
make check       # test + verify
make figures     # rebuild every figure in figures/
make pdf         # build/symspectre.pdf (needs pdflatex)
make show        # run the tests, then open the per-edge variations window
make show-fields # run the tests, then open the distance-field window
make show-fem    # run the tests, then open the finite-element window
make show-curvature # run the tests, then open the curvature window
make install-min # pip install numpy scipy matplotlib sympy (the bezier package is optional)
make clean       # remove build/ and Python caches
```

`make` on its own lists all targets.
