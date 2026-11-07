"""
chair_fibers.py - fibres from the Spectre tile to the pyramid tips of Chair44.

Chair44 (Tsiokos) is the seven-cube chair whose 24 exposed unit squares each carry eight small square pyramids,
192 in all: height j/10000 with j = 1..12, a bump (+j) or a dent (-j) of the same depth.  A pyramid's *tip* is its
apex.  This model joins the 14 vertices and 28 Bezier handles of a ``spectremodel.Tile`` (a flat sheet) to such
tips with *fibres*, so that the twist options of the tile can be seen in 3D.

    from spectremodel import Tile
    from chair_fibers import ChairTips, Placement, assign_fibers, fibre_curves

    tile = Tile(1, 1, 0.5)
    tips = ChairTips()                              # the 192 tips (models/data/r44_features.csv)
    place = Placement.for_tile(tile)                # where the sheet hangs above the chair
    assign_fibers(tile, tips, place)                # writes base.data["fibers"] = [tip role] on every vertex / handle
    tile.vertices[3].twist = 0.4                    # the tile stays dynamic ...
    for f in fibre_curves(tile, tips, place):       # ... and the fibres are recomputed from its current state
        print(f.base, f.tip, f.energy)

Nothing here is a canonical correspondence: a tile vertex has no natural partner among the tips, so the pairing is a
*rule* (below) and the placement of the sheet is a free choice.  The code works out exactly what the fibres do; it does not
test the thesis of the top-level README.

Fibre
-----
A fibre leaves a base point X (a vertex or the end of a handle) along the unit direction u of that handle (a vertex
uses its out handle), so twists steer it, and arrives at the tip T along the axis of the pyramid, travelling against
the panel's outward normal n.  It is the cubic Bezier with control points

    X,  X + s0 u,  T + s1 n,  T            (speeds s0, s1 >= 0).

With D = T - X, alpha = u.D, beta = n.D, gamma = u.n, dd = D.D the energy E = integral_0^1 |B''|^2 dt is exactly (SymPy,
``formulas()``; checked against quadrature)

    E = 12 [ dd - 3 alpha s0 + 3 beta s1 + 3 (s0^2 - gamma s0 s1 + s1^2) ],

a convex quadratic, so the smoothest fibre is the minimiser over s0, s1 >= 0: interior point
s0 = (2 alpha - gamma beta)/(4 - gamma^2), s1 = (gamma alpha - 2 beta)/(4 - gamma^2) when both are >= 0, otherwise a
boundary point (s0 = 0, s1 = -beta/2 or s1 = 0, s0 = alpha/2) or the origin.

For a tip on an up-facing panel and a sheet above it (u horizontal, n = +z, gamma = 0, beta <= 0) this collapses to

    E(phi) = 12 dd - 9 beta^2 - 9 max(0, alpha)^2,     alpha = |D_h| cos(phi - bearing),

so the energy-optimal start direction is the bearing of the tip, and E_min = 3 |D|^2.  A vertex twist theta turns phi by
theta and leaves X where it is, so E(theta) = top_face_energy(phi0 + theta, D) exactly (the twist law).  A handle twist
also moves X (the control point turns about its vertex), so for handles the law holds at the actual X.  A direction that
points away from the tip (alpha <= 0) does not steer the fibre at all: s0 = 0, and every such direction gives the same
fibre.

Pairing rule
------------
``assign_fibers`` pairs the 42 bases one to one with distinct tips so that the total energy is minimal (Hungarian
algorithm on the exact 42 x 192 energy matrix).  Any other pairing can be written straight into the data:
``base.data["fibers"]`` is a list of tip roles (several fibres per base are allowed).

Tips are drawn at their true positions; their heights (at most 12/10000) are far below the resolution of a plot, so the
sign and size of a tip are shown by colour and marker size.  Data: models/data (Tsiokos, CC BY 4.0).

Needs NumPy, SymPy and (for the pairing) SciPy; matplotlib is imported lazily for the figures.
"""

import argparse
import csv
import functools
import math
import operator
import os
import sys
from fractions import Fraction

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import chair44_core  # noqa: E402
from spectremodel import BezHandle, Tile, Vertex, layout_vertices  # noqa: E402

__all__ = [
    "TIP_CSV",
    "ChairTips",
    "Placement",
    "Fibre",
    "formulas",
    "fibre_optimum",
    "fibre_costs",
    "bases",
    "base_arrays",
    "assign_fibers",
    "make_fibre",
    "fibre_curves",
    "top_face_energy",
    "run_tests",
]

TIP_CSV = os.path.join(HERE, "data", "r44_features.csv")


# --------------------------------------------------------------------------
# The pyramid tips of Chair44
# --------------------------------------------------------------------------
class ChairTips:
    """The 192 pyramid features of the Chair44 solid.

    Arrays of length 192, indexed by ``role`` (= 8 * panel + mark):
    panel, axis (normal axis of the panel), sign (outward direction), coeff (signed height in
    units of 1/10000: + bump, - dent), apex (tip, shape (192, 3)), normal (outward unit normal of
    the panel, shape (192, 3)).  ``exact`` and ``panel_center`` hold the same tips and panel centres
    as tuples of Fractions.
    """

    def __init__(self, path=TIP_CSV):
        with open(path, newline="") as f:
            rows = sorted(csv.DictReader(f), key=lambda r: int(r["role"]))
        if [int(r["role"]) for r in rows] != list(range(len(rows))):
            raise ValueError("the roles of %s must be 0..n-1" % path)
        self.role = np.array([int(r["role"]) for r in rows])
        self.panel = np.array([int(r["panel"]) for r in rows])
        self.axis = np.array([int(r["normal_axis"]) for r in rows])
        self.sign = np.array([int(r["outward_sign"]) for r in rows])
        self.coeff = np.array([int(r["coefficient"]) for r in rows])
        self.exact = [tuple(Fraction(r[k]) for k in ("apex_x", "apex_y", "apex_z")) for r in rows]
        self.panel_center = [tuple(Fraction(r[k]) for k in ("panel_cx", "panel_cy", "panel_cz")) for r in rows]
        self.apex = np.array([[float(c) for c in e] for e in self.exact])
        self.normal = np.zeros((len(rows), 3))
        self.normal[np.arange(len(rows)), self.axis] = self.sign

    def __len__(self):
        return len(self.role)

    @property
    def height(self):
        """Signed height of every pyramid (+ out of the solid, - into it)."""
        return self.coeff / 10000.0

    def faces_up(self):
        """Boolean mask of the tips on panels that face +z."""
        return (self.axis == 2) & (self.sign == 1)


def _tip_index(role, n):
    role = operator.index(role)
    if not 0 <= role < n:
        raise IndexError("tip role %d out of range 0..%d" % (role, n - 1))
    return role


# --------------------------------------------------------------------------
# Exact fibre energy (SymPy)
# --------------------------------------------------------------------------
@functools.lru_cache(maxsize=None)
def formulas():
    """The SymPy derivation, done once.  Returns {"sym": expressions, "num": numpy functions}.

    B(t) is the cubic with control points X, X + s0 u, T + s1 n, T and B'' = 6 [(1 - t) A + t C] with A = D - 2 s0 u + s1 n,
    C = -D + s0 u - 2 s1 n; all inner products come from the Gram matrix of (D, u, n), so only
    dd, alpha, beta, gamma enter.
    """
    import sympy as sp

    s0, s1, dd, al, be, ga = sp.symbols("s0 s1 dd alpha beta gamma", real=True)
    gram = sp.Matrix([[dd, al, be], [al, 1, ga], [be, ga, 1]])
    A = sp.Matrix([1, -2 * s0, s1])
    C = sp.Matrix([-1, s0, -2 * s1])

    def ip(x, y):
        return (x.T * gram * y)[0]

    E = sp.expand(12 * (ip(A, A) + ip(A, C) + ip(C, C)))
    interior = sp.solve([E.diff(s0), E.diff(s1)], [s0, s1], dict=True)[0]
    s0_int, s1_int = sp.simplify(interior[s0]), sp.simplify(interior[s1])
    s1_b = sp.solve(E.subs(s0, 0).diff(s1), s1)[0]  # boundary s0 = 0
    s0_b = sp.solve(E.subs(s1, 0).diff(s0), s0)[0]  # boundary s1 = 0
    sym = {
        "E": E,
        "s0_int": s0_int,
        "s1_int": s1_int,
        "E_int": sp.simplify(E.subs(interior)),
        "s1_b0": s1_b,
        "E_b0": sp.simplify(E.subs({s0: 0, s1: s1_b})),
        "s0_b1": s0_b,
        "E_b1": sp.simplify(E.subs({s1: 0, s0: s0_b})),
        "E_0": sp.simplify(E.subs({s0: 0, s1: 0})),
    }
    num = {
        "energy": sp.lambdify((s0, s1, dd, al, be, ga), E, "numpy"),
        "interior": sp.lambdify((al, be, ga), (s0_int, s1_int), "numpy"),
        "E_int": sp.lambdify((dd, al, be, ga), sym["E_int"], "numpy"),
        "E_b0": sp.lambdify((dd, be), sym["E_b0"], "numpy"),
        "E_b1": sp.lambdify((dd, al), sym["E_b1"], "numpy"),
        "E_0": sp.lambdify((dd,), sym["E_0"], "numpy"),
    }
    return {"sym": sym, "num": num}


def fibre_optimum(dd, al, be, ga):
    """Smoothest fibre: speeds (s0, s1 >= 0) minimising the energy, and that energy.

    Inputs may be arrays (broadcast together); returns (s0, s1, E) of the same shape.  Because the energy is a
    strictly convex quadratic, the constrained minimiser is the best of four feasible candidates: the interior
    stationary point, the two boundary points, and the origin.
    """
    F = formulas()["num"]
    dd, al, be, ga = np.broadcast_arrays(*(np.asarray(v, dtype=float) for v in (dd, al, be, ga)))
    zero = np.zeros_like(dd)
    s0i, s1i = (np.broadcast_to(np.asarray(v, dtype=float), dd.shape) for v in F["interior"](al, be, ga))
    inf = np.full_like(dd, np.inf)
    cands = [
        (s0i, s1i, np.where((s0i >= 0) & (s1i >= 0), F["E_int"](dd, al, be, ga), inf)),
        (zero, -be / 2.0, np.where(be <= 0, F["E_b0"](dd, be), inf)),
        (al / 2.0, zero, np.where(al >= 0, F["E_b1"](dd, al), inf)),
        (zero, zero, np.broadcast_to(F["E_0"](dd), dd.shape)),
    ]
    idx = np.argmin(np.stack([c[2] for c in cands]), axis=0)
    return tuple(np.choose(idx, [c[k] for c in cands]) for k in range(3))


def top_face_energy(phi, D):
    """Exact energy of the smoothest fibre from X to a tip T = X + D on an up-facing panel (n = +z) when the start
    direction lies in the sheet plane at angle phi.  Valid for D_z <= 0 (the sheet is above the tip):
        E = 12 |D|^2 - 9 D_z^2 - 9 max(0, |D_h| cos(phi - bearing))^2,   bearing = atan2(D_y, D_x).
    Minimal, 3 |D|^2, at phi = bearing."""
    D = np.asarray(D, dtype=float)
    dh = math.hypot(D[0], D[1])
    alpha = dh * np.cos(np.asarray(phi, dtype=float) - math.atan2(D[1], D[0]))
    return 12.0 * (dh**2 + D[2] ** 2) - 9.0 * D[2] ** 2 - 9.0 * np.maximum(0.0, alpha) ** 2


# --------------------------------------------------------------------------
# Placing the tile and reading its bases
# --------------------------------------------------------------------------
class Placement:
    """Where the flat tile hangs in the frame of the chair (the chair occupies [0, 2]^3 minus the cube [1, 2]^3):

        x3 = (cx, cy, z) + scale * R(angle) (xy - ref).

    ``ref`` is fixed when the placement is made, so editing the tile (moving a vertex, say) never shifts the sheet.
    """

    def __init__(self, scale=0.5, center=(1.0, 1.0), z=3.2, angle=0.0, ref=(0.0, 0.0)):
        vals = [scale, z, angle, *center, *ref]
        if not np.all(np.isfinite(vals)) or len(center) != 2 or len(ref) != 2:
            raise ValueError("Placement needs finite numbers: scale, center (2), z, angle, ref (2)")
        if scale == 0:
            raise ValueError("scale must not be 0")
        self.scale, self.z, self.angle = float(scale), float(z), float(angle)
        self.center, self.ref = np.array(center, dtype=float), np.array(ref, dtype=float)

    @classmethod
    def for_tile(cls, tile, **kwargs):
        """Placement whose reference point is the centroid of the standard layout of ``tile`` (lengths a, b)."""
        return cls(ref=layout_vertices(tile.a, tile.b).mean(axis=0), **kwargs)

    def _rotate(self, v):
        c, s = math.cos(self.angle), math.sin(self.angle)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    def point(self, xy):
        """3D position of a point (x, y) of the tile."""
        v = self.scale * self._rotate(np.asarray(xy, dtype=float) - self.ref)
        return np.array([self.center[0] + v[0], self.center[1] + v[1], self.z])

    def direction(self, uv):
        """3D direction of a planar direction (rotated like the tile, not scaled)."""
        v = self._rotate(np.asarray(uv, dtype=float))
        return np.array([v[0], v[1], 0.0])


def bases(tile):
    """The 42 attachment points of a tile: its 14 vertices, then its 28 handles."""
    return list(tile.vertices) + list(tile.handles)


def base_label(base):
    if isinstance(base, Vertex):
        return "v%d" % base.index
    return "v%d.%s" % (base.vertex.index, base.kind)


def _base_geometry(base, placement):
    """3D start point and unit start direction of the fibre of a base (a vertex uses its out handle)."""
    if isinstance(base, Vertex):
        xy, d = base.position, base.out_handle.offset
    elif isinstance(base, BezHandle):
        xy, d = base.position, base.offset
    else:
        raise TypeError("a base is a Vertex or a BezHandle, got %r" % (base,))
    n = math.hypot(d[0], d[1])
    if n == 0:
        raise ValueError("%s has a zero-length handle, so it has no direction" % base_label(base))
    return placement.point(xy), placement.direction(d / n)


def base_arrays(tile, placement):
    """Start points X and unit start directions U of all 42 bases, each of shape (42, 3)."""
    geo = [_base_geometry(b, placement) for b in bases(tile)]
    return np.array([g[0] for g in geo]), np.array([g[1] for g in geo])


# --------------------------------------------------------------------------
# Fibres
# --------------------------------------------------------------------------
def fibre_costs(X, U, tips):
    """Energy of the smoothest fibre from every base to every tip: (E, s0, s1), each of shape (len(X), len(tips))."""
    D = tips.apex[None, :, :] - np.asarray(X)[:, None, :]
    dd = np.einsum("nmk,nmk->nm", D, D)
    al = np.einsum("nmk,nk->nm", D, np.asarray(U))
    be = np.einsum("nmk,mk->nm", D, tips.normal)
    ga = np.asarray(U) @ tips.normal.T
    s0, s1, E = fibre_optimum(dd, al, be, ga)
    return E, s0, s1


class Fibre:
    """One fibre: ``base`` (Vertex or BezHandle), ``tip`` (role), ``controls`` (4, 3) Bezier control points,
    ``points`` (samples + 1, 3) along the curve, ``energy`` (integral of |B''|^2) and ``speeds`` (s0, s1)."""

    def __init__(self, base, tip, controls, points, energy, speeds):
        self.base, self.tip, self.controls, self.points = base, tip, controls, points
        self.energy, self.speeds = float(energy), (float(speeds[0]), float(speeds[1]))

    def __repr__(self):
        return "<Fibre %s -> tip %d, energy %.4g>" % (base_label(self.base), self.tip, self.energy)


def _bernstein(t):
    t = np.asarray(t, dtype=float)
    u = 1.0 - t
    return np.stack([u**3, 3 * u**2 * t, 3 * u * t**2, t**3], axis=1)


def make_fibre(base, role, tips, placement, samples=60):
    """The smoothest fibre from ``base`` to the tip with this role, for the current state of the tile."""
    if int(samples) != samples or samples < 1:
        raise ValueError("samples must be a positive integer")
    role = _tip_index(role, len(tips))
    X, u = _base_geometry(base, placement)
    T, n = tips.apex[role], tips.normal[role]
    D = T - X
    s0, s1, E = fibre_optimum(D @ D, u @ D, n @ D, u @ n)
    controls = np.array([X, X + s0 * u, T + s1 * n, T])
    points = _bernstein(np.linspace(0.0, 1.0, int(samples) + 1)) @ controls
    return Fibre(base, role, controls, points, E, (s0, s1))


def fibre_curves(tile, tips, placement, samples=60):
    """All fibres listed in the ``data["fibers"]`` of the tile's vertices and handles, recomputed from the
    current state of the tile (nothing is cached)."""
    out = []
    for base in bases(tile):
        for role in base.data.get("fibers", ()):
            out.append(make_fibre(base, role, tips, placement, samples))
    return out


def assign_fibers(tile, tips, placement):
    """Pair the 42 bases one to one with distinct tips so that the total fibre energy is minimal, and store the
    pairing as ``base.data["fibers"] = [role]``.  Replaces any fibres already stored.  Returns the total energy."""
    from scipy.optimize import linear_sum_assignment

    B = bases(tile)
    X, U = base_arrays(tile, placement)
    E, _, _ = fibre_costs(X, U, tips)
    rows, cols = linear_sum_assignment(E)
    for b in B:
        b.data["fibers"] = []
    for r, c in zip(rows, cols):
        B[r].data["fibers"] = [int(tips.role[c])]
    return float(E[rows, cols].sum())


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 models/chair_fibers.py --tests-only)
# --------------------------------------------------------------------------
class _Skip(Exception):
    pass


def _need_scipy():
    try:
        import scipy.optimize  # noqa: F401
    except ImportError as exc:
        raise _Skip("scipy not installed: %s" % exc)


def _unit(rng):
    v = rng.normal(size=3)
    return v / np.linalg.norm(v)


def _scene(k=0.5):
    tile = Tile(1, 1, k)
    return tile, ChairTips(), Placement.for_tile(tile)


def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def test_vendored_tip_table_matches_the_paper():
    tips = ChairTips()
    assert len(tips) == 192 and list(tips.role) == list(range(192))
    assert all((tips.panel == p).sum() == 8 for p in range(24)) and np.array_equal(tips.role // 8, tips.panel)
    for j in range(1, 13):  # twelve magnitudes, each 8 bumps and 8 dents
        assert (tips.coeff == j).sum() == 8 and (tips.coeff == -j).sum() == 8
    assert tips.coeff.sum() == 0 and set(np.abs(tips.coeff)) == set(range(1, 13))
    margin = Fraction(1, 2) - Fraction(1, 5)  # marks lie more than 1/5 from every panel edge
    for i in range(192):
        ax, sg = int(tips.axis[i]), int(tips.sign[i])
        assert tips.exact[i][ax] == tips.panel_center[i][ax] + sg * Fraction(int(tips.coeff[i]), 10000)
        for t in range(3):
            if t != ax:
                assert abs(tips.exact[i][t] - tips.panel_center[i][t]) < margin, (i, t)
    assert len(set(tips.exact)) == 192
    assert np.allclose(np.linalg.norm(tips.normal, axis=1), 1.0)


def test_panels_agree_with_chair44_core():
    tips = ChairTips()
    from_csv = {(int(tips.axis[i]), int(tips.sign[i]), tips.panel_center[i]) for i in range(192)}
    expect = set()
    for ax, sd, cell in chair44_core.exposed_faces(chair44_core.CHAIR_CELLS):
        c = [Fraction(cell[k]) + Fraction(1, 2) for k in range(3)]
        c[ax] = Fraction(cell[ax] + (1 if sd > 0 else 0))
        expect.add((ax, sd, tuple(c)))
    assert len(from_csv) == 24 and from_csv == expect


def test_closed_form_energy_matches_quadrature():
    import sympy as sp

    f = formulas()
    s0, s1, dd, al, be, ga = sp.symbols("s0 s1 dd alpha beta gamma", real=True)
    hand = 12 * (dd - 3 * al * s0 + 3 * be * s1 + 3 * (s0**2 - ga * s0 * s1 + s1**2))
    assert sp.simplify(f["sym"]["E"] - hand) == 0
    rng = np.random.default_rng(1)
    nodes, w = np.polynomial.legendre.leggauss(8)
    t, w = 0.5 * (nodes + 1.0), 0.5 * w
    for _ in range(100):
        X, T, u, n = rng.normal(size=3), rng.normal(size=3), _unit(rng), _unit(rng)
        a, b = rng.uniform(0, 2, 2)
        P = np.array([X, X + a * u, T + b * n, T])
        B2 = 6 * ((P[0] - 2 * P[1] + P[2])[None] * (1 - t)[:, None] + (P[1] - 2 * P[2] + P[3])[None] * t[:, None])
        quad = float((np.einsum("ij,ij->i", B2, B2) * w).sum())
        D = T - X
        assert math.isclose(f["num"]["energy"](a, b, D @ D, u @ D, n @ D, u @ n), quad, rel_tol=1e-12, abs_tol=1e-12)


def test_optimal_speeds_are_the_constrained_minimum():
    _need_scipy()
    from scipy.optimize import minimize

    F = formulas()["num"]
    rng = np.random.default_rng(4)
    boundary = 0
    for _ in range(120):
        X, T, u, n = rng.normal(size=3), rng.normal(size=3), _unit(rng), _unit(rng)
        D = T - X
        args = (D @ D, u @ D, n @ D, u @ n)
        s0, s1, E = fibre_optimum(*args)
        boundary += int(s0 == 0 or s1 == 0)
        assert s0 >= 0 and s1 >= 0 and math.isclose(E, F["energy"](s0, s1, *args), rel_tol=1e-12, abs_tol=1e-12)
        probe = np.vstack([rng.uniform(0, 4, (300, 2)), np.maximum(0, [s0, s1] + 0.05 * rng.normal(size=(100, 2)))])
        assert np.all(F["energy"](probe[:, 0], probe[:, 1], *args) >= E - 1e-9)
        ref = minimize(lambda s: F["energy"](s[0], s[1], *args), [1.0, 1.0], bounds=[(0, None)] * 2, tol=1e-13)
        assert abs(ref.fun - E) < 1e-6 * max(1.0, abs(E))
    assert boundary >= 10, "the boundary cases of the constrained minimum were not exercised"
    # arrays and scalars agree
    a = rng.normal(size=(3, 4))
    s0, s1, E = fibre_optimum(a * a + 5.0, a, -np.abs(a), 0.3 * np.ones_like(a))
    assert s0.shape == s1.shape == E.shape == (3, 4)
    assert math.isclose(fibre_optimum(a[1, 2] ** 2 + 5.0, a[1, 2], -abs(a[1, 2]), 0.3)[2], E[1, 2])


def test_fibre_ends_and_tangents():
    tile, tips, place = _scene()
    assign_fibers(tile, tips, place)
    fibres = fibre_curves(tile, tips, place)
    nodes, w = np.polynomial.legendre.leggauss(8)
    t, w = 0.5 * (nodes + 1.0), 0.5 * w
    assert len(fibres) == 42
    for f in fibres:
        X, u = _base_geometry(f.base, place)
        T, n = tips.apex[f.tip], tips.normal[f.tip]
        assert np.allclose(f.points[0], X, atol=1e-13) and np.array_equal(f.controls[3], T)
        assert np.allclose(f.points[-1], T, atol=1e-13)
        s0, s1 = f.speeds
        assert s0 >= 0 and s1 >= 0
        assert np.allclose(f.controls[1] - f.controls[0], s0 * u) and np.allclose(f.controls[3] - f.controls[2], -s1 * n)
        if s0 > 1e-9:  # leaves along the handle direction
            assert np.allclose((f.controls[1] - X) / s0, u)
        P = f.controls
        B2 = 6 * ((P[0] - 2 * P[1] + P[2])[None] * (1 - t)[:, None] + (P[1] - 2 * P[2] + P[3])[None] * t[:, None])
        assert math.isclose(f.energy, float((np.einsum("ij,ij->i", B2, B2) * w).sum()), rel_tol=1e-10)


def test_twist_law_for_tips_on_up_facing_panels():
    rng = np.random.default_rng(6)
    n = np.array([0.0, 0.0, 1.0])
    for _ in range(40):
        D = np.array([*rng.uniform(-2, 2, 2), -rng.uniform(0.1, 2.0)])
        dd, bearing = D @ D, math.atan2(D[1], D[0])
        phis = np.linspace(-math.pi, math.pi, 73)
        U = np.stack([np.cos(phis), np.sin(phis), np.zeros_like(phis)], axis=1)
        _, _, E = fibre_optimum(dd, U @ D, n @ D, U @ n)
        assert np.allclose(E, [top_face_energy(p, D) for p in phis], atol=1e-10)
        assert math.isclose(top_face_energy(bearing, D), 3 * dd, rel_tol=1e-12)
        assert np.all(E >= 3 * dd - 1e-10), "no direction beats pointing at the tip"
        # a direction pointing away from the tip (alpha <= 0) does not steer the fibre: same fibre for all of them
        back = [i for i, p in enumerate(phis) if U[i] @ D <= 0]
        s0, s1, _ = fibre_optimum(dd, U[back] @ D, n @ D, U[back] @ n)
        assert np.all(s0 == 0.0) and np.allclose(s1, -(n @ D) / 2.0)
    # the same law through the Tile API.  A vertex twist turns u and leaves X alone: E(theta) = law(phi0 + theta, D).
    # A handle twist also moves X, so there the law is checked at the actual X and u.
    tile, tips, place = _scene()
    assign_fibers(tile, tips, place)
    n_vertex = n_handle = 0
    for b in bases(tile):
        role = b.data["fibers"][0]
        if not tips.faces_up()[role]:
            continue
        f0 = make_fibre(b, role, tips, place)
        X0, u0 = _base_geometry(b, place)
        phi0 = math.atan2(u0[1], u0[0])
        for theta in (-2.5, -1.0, 0.0, 0.7, 2.0):
            b.twist = theta
            X, u = _base_geometry(b, place)
            D = tips.apex[role] - X
            energy = make_fibre(b, role, tips, place).energy
            assert math.isclose(energy, top_face_energy(math.atan2(u[1], u[0]), D), rel_tol=1e-10)
            if isinstance(b, Vertex):
                assert np.allclose(X, X0) and math.isclose(energy, top_face_energy(phi0 + theta, D), rel_tol=1e-10)
        b.twist = 0.0
        assert math.isclose(make_fibre(b, role, tips, place).energy, f0.energy, rel_tol=1e-14)
        n_vertex, n_handle = n_vertex + isinstance(b, Vertex), n_handle + isinstance(b, BezHandle)
    assert n_vertex >= 5 and n_handle >= 10, (n_vertex, n_handle)


def test_assignment_is_one_to_one_optimal_and_lives_in_data():
    _need_scipy()
    tile, tips, place = _scene()
    total = assign_fibers(tile, tips, place)
    B = bases(tile)
    assert len(B) == 42 and all(len(b.data["fibers"]) == 1 for b in B)
    roles = [b.data["fibers"][0] for b in B]
    assert len(set(roles)) == 42
    X, U = base_arrays(tile, place)
    E, _, _ = fibre_costs(X, U, tips)
    assert math.isclose(total, sum(E[i, r] for i, r in enumerate(roles)), rel_tol=1e-12)
    rng = np.random.default_rng(8)
    assert all(E[np.arange(42), rng.choice(192, 42, replace=False)].sum() >= total - 1e-9 for _ in range(300))
    # editing the tile keeps the pairing and moves the fibres: a vertex twist turns its start direction exactly.
    # (Pick a vertex whose fibre uses its direction, s0 > 0; a vertex pointing away from its tip is not steered.)
    v = next(w for w in tile.vertices if make_fibre(w, w.data["fibers"][0], tips, place).speeds[0] > 1e-3)
    u_before = _base_geometry(v, place)[1]
    p_before = make_fibre(v, v.data["fibers"][0], tips, place).points
    v.twist = 0.4
    u_after = _base_geometry(v, place)[1]
    assert [b.data["fibers"] for b in B] == [[r] for r in roles]
    assert math.isclose(math.atan2(*u_after[1::-1]) - math.atan2(*u_before[1::-1]), 0.4, abs_tol=1e-12)
    assert not np.allclose(make_fibre(v, v.data["fibers"][0], tips, place).points, p_before)
    # re-assigning replaces the data
    assign_fibers(tile, tips, place)
    assert all(len(b.data["fibers"]) == 1 for b in B)


def test_custom_fibre_data_is_honoured():
    tile, tips, place = _scene()
    v, h = tile.vertices[0], tile.vertices[1].out_handle
    v.data["fibers"] = [3, 100]
    h.data["fibers"] = [3]
    fibres = fibre_curves(tile, tips, place, samples=10)
    assert [(f.base, f.tip) for f in fibres] == [(v, 3), (v, 100), (h, 3)]
    assert all(f.points.shape == (11, 3) for f in fibres)
    assert fibre_curves(Tile(), tips, place) == [], "no data, no fibres"
    for bad in ([192], [-1]):
        v.data["fibers"] = bad
        assert _raises(IndexError, fibre_curves, tile, tips, place)
    v.data["fibers"] = [2.5]
    assert _raises(TypeError, fibre_curves, tile, tips, place)
    v.data["fibers"] = [0]
    assert _raises(ValueError, make_fibre, v, 0, tips, place, 0)
    assert _raises(TypeError, make_fibre, "not a base", 0, tips, place)


def test_placement_is_fixed_and_edits_stay_local():
    tile, tips, place = _scene()
    X0, U0 = base_arrays(tile, place)
    assert np.allclose(X0[:14, 2], place.z) and np.allclose(U0[:, 2], 0.0)
    assert np.allclose(place.point(layout_vertices(1, 1).mean(axis=0)), [1.0, 1.0, place.z])
    tile.vertices[5].position = tile.vertices[5].position + np.array([0.2, -0.1])
    X1, U1 = base_arrays(tile, place)
    changed = {i for i in range(42) if not (np.allclose(X0[i], X1[i]) and np.allclose(U0[i], U1[i]))}
    names = {base_label(b) for i, b in enumerate(bases(tile)) if i in changed}
    # vertex 5 moved; its two chords (4->5 and 5->6) reshape four handles, and vertex 4 (whose direction is its out
    # handle, shaped by the chord 4->5) turns with them
    assert names == {"v5", "v4", "v4.out", "v5.in", "v5.out", "v6.in"}, names
    turned = Placement.for_tile(tile, angle=math.pi / 2, scale=2.0)
    d = tile.vertices[2].out_handle.offset
    assert np.allclose(turned.direction(d), [-d[1], d[0], 0.0])
    for bad in (dict(scale=0.0), dict(scale=float("nan")), dict(z=float("inf")), dict(center=(1.0,))):
        assert _raises(ValueError, Placement, **bad)


def test_figures_build():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise _Skip("matplotlib not installed: %s" % exc)
    tile, tips, place = _scene()
    assign_fibers(tile, tips, place)
    for fig in (figure_scene(tile, tips, place), figure_twist(tile, tips, place)):
        fig.canvas.draw()
        plt.close(fig)
    assert all(b.twist == 0.0 for b in bases(tile)), "figure_twist must restore the twists it sweeps"


def run_tests(verbose=True):
    """Run every ``test_*`` function in this module; return True if none failed."""
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except _Skip as exc:
            status = "SKIP (%s)" % exc
        except Exception as exc:  # noqa: BLE001 - report and keep going
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-66s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figures (matplotlib, 3D)
# --------------------------------------------------------------------------
_KIND_COLOUR = {"out": "tab:red", "in": "tab:blue"}
_BUMP, _DENT = "#e66101", "#5e3c99"


def _base_colour(base):
    return "0.15" if isinstance(base, Vertex) else _KIND_COLOUR[base.kind]


def _setup3d(ax, elev, azim, limits=((-0.2, 2.2), (-0.2, 2.2), (0.0, 3.4)), plan=False):
    """Equal-scale 3D axes over ``limits`` ((x0, x1), (y0, y1), (z0, z1)); ``plan`` hides the z axis."""
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    ax.set_box_aspect([hi - lo for lo, hi in limits])
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x", fontsize=8)
    ax.set_ylabel("y", fontsize=8)
    if plan:
        ax.set_zticks([])
    else:
        ax.set_zlabel("z", fontsize=8)
    ax.tick_params(labelsize=7)


def _draw_chair(ax, tips):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    quads = []
    for axis, sign, cell in chair44_core.exposed_faces(chair44_core.CHAIR_CELLS):
        t1, t2 = [k for k in range(3) if k != axis]
        quad = []
        for i, j in ((0, 0), (1, 0), (1, 1), (0, 1)):
            v = [0, 0, 0]
            v[axis], v[t1], v[t2] = cell[axis] + (1 if sign > 0 else 0), cell[t1] + i, cell[t2] + j
            quad.append(v)
        quads.append(quad)
    ax.add_collection3d(Poly3DCollection(quads, facecolor="#c9d6ea", edgecolor="0.55", linewidths=0.5, alpha=0.20))
    bump = tips.coeff > 0
    for mask, col in ((bump, _BUMP), (~bump, _DENT)):
        p, s = tips.apex[mask], 6 + 5 * np.abs(tips.coeff[mask])
        ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=col, s=s, depthshade=False, linewidths=0)


def _draw_tile(ax, tile, place, limits=None):
    """The sheet: outline, vertices, handle sticks.  With ``limits`` only the part inside that box is drawn."""
    def inside(p):
        return limits is None or all(lo <= p[k] <= hi for k, (lo, hi) in enumerate(limits))

    x, y = tile.outline(60)
    pts = np.array([place.point((a, b)) for a, b in zip(x, y)])
    pts[[not inside(p) for p in pts]] = np.nan  # breaks the line where it leaves the box
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color="black", lw=1.2)
    for h in tile.handles:
        p, q = place.point(h.vertex.position), place.point(h.position)
        if inside(p) and inside(q):
            ax.plot(*zip(p, q), color=_KIND_COLOUR[h.kind], lw=0.7)
    verts = np.array([p for p in (place.point(v.position) for v in tile.vertices) if inside(p)]).reshape(-1, 3)
    ax.scatter(verts[:, 0], verts[:, 1], verts[:, 2], color="0.2", s=10, depthshade=False)
    hp = [(place.point(h.position), _KIND_COLOUR[h.kind]) for h in tile.handles]
    hp = [(p, c) for p, c in hp if inside(p)]
    if hp:
        ax.scatter([p[0] for p, _ in hp], [p[1] for p, _ in hp], [p[2] for p, _ in hp],
                   c=[c for _, c in hp], s=8, depthshade=False)


def _legend(ax):
    from matplotlib.lines import Line2D

    items = [
        Line2D([], [], color="0.15", lw=1.4, label="fibre from a vertex"),
        Line2D([], [], color="tab:red", lw=1.4, label="fibre from an out handle"),
        Line2D([], [], color="tab:blue", lw=1.4, label="fibre from an in handle"),
        Line2D([], [], marker="o", ls="", color=_BUMP, ms=5, label="tip of a bump (+j)"),
        Line2D([], [], marker="o", ls="", color=_DENT, ms=5, label="tip of a dent (-j)"),
    ]
    ax.legend(handles=items, loc="upper left", fontsize=7, frameon=False)


def figure_scene(tile, tips, place):
    """The tile as a flat sheet above Chair44 with one fibre from every vertex and handle to its tip:
    a perspective view and the plan view."""
    import matplotlib.pyplot as plt

    fibres = fibre_curves(tile, tips, place)
    fig = plt.figure(figsize=(13, 6.5))
    for k, (elev, azim, title) in enumerate(((24, -58, "perspective"), (90, -90, "plan view (looking down)"))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        _draw_chair(ax, tips)
        _draw_tile(ax, tile, place)
        for f in fibres:
            ax.plot(f.points[:, 0], f.points[:, 1], f.points[:, 2], color=_base_colour(f.base), lw=0.9)
        _setup3d(ax, elev, azim, plan=(k == 1))
        ax.set_title(title, fontsize=9)
        if k == 0:
            _legend(ax)
    fig.suptitle(
        "Spectre tile (%d vertices + %d handles) joined to Chair44 pyramid tips by %d smoothest fibres; total energy %.1f\n"
        "pairing: one to one, minimal total energy; tips at true positions (heights <= 12/10000), colour/size = sign/|j|"
        % (len(tile.vertices), len(tile.handles), len(fibres), sum(f.energy for f in fibres)),
        fontsize=10,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    return fig


def figure_twist(tile, tips, place, n_pick=3):
    """What a vertex twist does to a fibre (a vertex does not move when twisted, so the closed form is exact).
    Left: the fibre of one vertex for a sweep of twists (colour = energy), the best one thick.  Right: energy against
    twist for three vertices (curves: ``top_face_energy``; dots: the full pipeline after setting the twist on the
    Tile), with the minimum 3 |D|^2 at the bearing of the tip and the flat part where the vertex points away from
    its tip.  Only vertices paired with a tip on an up-facing panel are used; their twists are restored afterwards."""
    import matplotlib.pyplot as plt

    cand = [v for v in tile.vertices if v.data.get("fibers") and tips.faces_up()[v.data["fibers"][0]]]
    if not cand:
        raise ValueError("no vertex is paired with a tip on an up-facing panel; run assign_fibers first")
    picks = [cand[i] for i in np.linspace(0, len(cand) - 1, min(n_pick, len(cand))).round().astype(int)]
    thetas = np.linspace(-math.pi, math.pi, 25)
    fig = plt.figure(figsize=(13, 6))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2)
    _draw_chair(ax3, tips)
    cmap = plt.get_cmap("viridis_r")
    fan_pts = []  # points of the fan in the left panel, to zoom onto it
    for idx, b in enumerate(picks):
        role = b.data["fibers"][0]
        base_twist = b.twist
        b.twist = 0.0  # the sweep is measured from the vertex's own twist = 0
        X, u0 = _base_geometry(b, place)
        D, phi0 = tips.apex[role] - X, math.atan2(u0[1], u0[0])
        theta_star = (math.atan2(D[1], D[0]) - phi0 + math.pi) % (2 * math.pi) - math.pi
        pipeline = []
        for th in thetas:
            b.twist = th
            pipeline.append(make_fibre(b, role, tips, place).energy)
        fine = np.linspace(-math.pi, math.pi, 400)
        (line,) = ax2.plot(np.degrees(fine), [top_face_energy(phi0 + t, D) for t in fine], lw=1.4,
                           label="%s -> tip %d   min 3|D|^2 = %.2f" % (base_label(b), role, 3 * (D @ D)))
        ax2.plot(np.degrees(thetas), pipeline, "o", ms=3.5, color=line.get_color())
        ax2.axvline(math.degrees(theta_star), color=line.get_color(), ls=":", lw=0.9)
        if idx == 0:
            lo, hi = min(pipeline), max(pipeline)
            for th in np.radians(np.arange(-165, 166, 15)):
                b.twist = th
                f = make_fibre(b, role, tips, place)
                fan_pts.append(f.points)
                ax3.plot(f.points[:, 0], f.points[:, 1], f.points[:, 2], color=cmap((f.energy - lo) / (hi - lo)), lw=1.0)
            b.twist = theta_star
            f = make_fibre(b, role, tips, place)
            ax3.plot(f.points[:, 0], f.points[:, 1], f.points[:, 2], color="black", lw=2.6)
            ax3.scatter(*tips.apex[role], marker="*", s=140, color="black", depthshade=False)
            ax3.scatter(*X, marker="o", s=40, color="black", depthshade=False)
            ax3.set_title("fibre of %s for vertex twists -150 .. +150 degrees (black: best, %.0f deg)"
                          % (base_label(b), math.degrees(theta_star)), fontsize=9)
        b.twist = base_twist
    pts = np.vstack(fan_pts)
    pad = 0.15
    box = tuple((float(pts[:, k].min()) - pad, float(pts[:, k].max()) + pad) for k in range(3))
    _draw_tile(ax3, tile, place, limits=box)
    _setup3d(ax3, 24, -58, limits=box)  # zoomed onto the fan, equal scale
    ax2.set_xlabel("twist of the vertex (degrees)")
    ax2.set_ylabel("fibre energy  integral |B''|^2 dt")
    ax2.set_title("E(twist) = 12|D|^2 - 9 D_z^2 - 9 max(0, |D_h| cos(twist + phi0 - bearing))^2   (dotted: best twist)", fontsize=8)
    ax2.legend(fontsize=7, frameon=False)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    return fig


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Join the Spectre tile of spectremodel.py to the pyramid tips of Chair44 with fibres: run the self "
        "tests, then show two 3D figures."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--curve-strength", type=float, default=0.5, help="curve strength k (default 0.5)")
    p.add_argument("--scale", type=float, default=0.5, help="size of the sheet in chair units (default 0.5)")
    p.add_argument("--height", type=float, default=3.2, help="height of the sheet above the chair base (default 3.2)")
    p.add_argument("--skip-tests", action="store_true", help="only draw the figures")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open windows (use with --save-dir)")
    p.add_argument("--save-dir", metavar="DIR", help="also save the figures (PNG) into this folder")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("chair_fibers self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0

    import matplotlib

    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tile = Tile(args.a, args.b, args.curve_strength)
    tips = ChairTips()
    place = Placement.for_tile(tile, scale=args.scale, z=args.height)
    total = assign_fibers(tile, tips, place)
    print("one-to-one pairing of 42 bases with tips, total fibre energy %.3f" % total)
    figs = {"chair_fibers_scene.png": figure_scene(tile, tips, place),
            "chair_fibers_twist.png": figure_twist(tile, tips, place)}
    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)
        for name, fig in figs.items():
            fig.savefig(os.path.join(args.save_dir, name), dpi=130)
            print("saved", os.path.join(args.save_dir, name))
    if not args.no_show:
        plt.show()
    for fig in figs.values():
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
