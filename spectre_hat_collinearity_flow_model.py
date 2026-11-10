"""
spectre_hat_collinearity_flow_model.py - the Spectre -> Hat flow Phi_tau on top of spectremodel.py.

The Hat is Tile(1, sqrt(3)) (13 vertices) and the Spectre family starts at Tile(1, 1)
(14 edges, curved).  ``SpectreHatFlow`` is a live object that moves one tile between
them with a single parameter tau in [0, 1]:

    a(tau) = 1,   b(tau) = 1 + (sqrt(3) - 1) * tau        (edge lengths, linear in tau)
    k(tau) = k0 * (1 - tau)                               (curve_strength: lambda = 1 - tau)
    w(tau) = w0 * (1 - tau)                               (turning angle of the 14th vertex)
    arrow(tau) = pi * (1 - tau)                           (orientation-arrow angle of the notes)

    tau = 0 : Spectre, Tile(1, 1) with curve_strength k0 and the 14th vertex open by w0
    tau = 1 : Hat,     Tile(1, sqrt(3)) with straight edges (k = 0) and the 14th vertex flat

The 14th vertex is P_8 of spectremodel.py.  In the standard layout its two edges (7 and 8)
both point along (0, -1).  The flow opens it: P_8 is pushed outward, perpendicular to the
chord P_7 -> P_9 (which does not move), by h = a * tan(w / 2), so that edge 7 and edge 8
tilt away from each other by w / 2 each.  The edge *directions* of those two edges
therefore change with tau, their lengths become a / cos(w / 2), and the turning angle at
P_8 is exactly -w(tau) (clockwise, like all convex corners of the tile).  The rest of the
polygon does not move, so the outline stays closed for every tau.  w0 = ``corner_deg``
(default 30 degrees: an interior angle of 150 degrees, as in the notes); corner_deg = 0
gives back the plain Tile(a, b) family with P_8 flat all along.

    from spectre_hat_collinearity_flow_model import SpectreHatFlow

    flow = SpectreHatFlow(k0=0.5)
    flow.tau = 0.4                      # moves a, b and the curvature of flow.tile together
    x, y = flow.tile.outline(60)        # the current outline (a plain spectremodel.Tile)
    flow.vertex_report()                # turning angle of every vertex, polygon and curve
    flow.effective_vertex_count()       # vertices that are really corners

What the geometry of spectremodel.py says (all of it is checked by the self tests)
-----------------------------------------------------------------------------------
* The 14 edge *directions* of the plain Tile(a, b) do not depend on a and b, only the
  lengths do.  So with corner_deg = 0 the turning angle of every polygon vertex is the
  same for every tau, and P_8 is flat all along (13 corners, never 14).  That is why the
  opening of P_8 above is needed for a 14 -> 13 collapse.
* With corner_deg = w0 > 0 the polygon has 14 corners for tau < 1 and 13 at tau = 1.
  Opening P_8 by w also changes the turn at its neighbours: P_7 and P_9 turn by
  -60 + w/2 degrees instead of -60 (the three changes add up to 0, the total turn stays
  -360).  All other vertices are untouched.
* The curved edges turn by exactly the polygon angle at every vertex, for every
  curve_strength: the handle offset d/2 - k*n is the chord d rotated by the fixed angle
  atan(2k), the same for both edges at a vertex.  So the curve has a corner at the same
  vertices as the polygon, whatever k is.  The curvature k turns each edge into an S
  shaped curve about its chord (it moves the outline, not the corner angles), and the
  straight Hat is the k = 0 end.
* The area of the untwisted tile is the polygon area for every k:
  2*sqrt(3)*a^2 + 3*a*b + sqrt(3)*b^2 + a*h, where a*h = a^2 * tan(w / 2) is the triangle
  added by opening P_8.

The notes' coordinate table V_k(tau) and the cross product C(tau) = sqrt(3)/4 * (1 - tau)
at V_14 do not agree with these edge vectors, which are the ones of spectre.py.  In this
model the cross product of the two edges at P_8 is C = -(a^2 + h^2) * sin(w) with
w = w0 * (1 - tau): it is non-zero for tau < 1 and 0 at tau = 1, but it is not linear in
tau.  ``arrow_angle`` is a parameterisation taken from the notes, not something measured
on the outline.

Only NumPy and spectremodel.py are needed; matplotlib is imported lazily by the
command line demo (python3 spectre_hat_collinearity_flow_model.py --help).
"""

import argparse
import math
import sys

import numpy as np

from spectremodel import N_EDGES, Tile, edge_vectors

__all__ = [
    "SQRT3",
    "FLAT_VERTEX",
    "SpectreHatFlow",
    "polygon_turning_angles",
    "curve_corner_angles",
    "notes_cross_product",
    "run_tests",
]

SQRT3 = math.sqrt(3.0)
FLAT_VERTEX = 8  # edges 7 and 8 of spectremodel.py are collinear: the 14th vertex of the Spectre


# --------------------------------------------------------------------------
# Measurements on a spectremodel.Tile
# --------------------------------------------------------------------------
def _signed_angle(u, v):
    """Counter-clockwise angle from u to v in radians, in (-pi, pi]."""
    return math.atan2(u[0] * v[1] - u[1] * v[0], float(u @ v))


def polygon_turning_angles(tile):
    """Turning angle (degrees, counter-clockwise positive) at every vertex of the straight
    14-gon through the current vertex positions, shape (14,).  0 means a flat vertex."""
    chords = np.array([tile.chord(i) for i in range(N_EDGES)])
    return np.array([math.degrees(_signed_angle(chords[i - 1], chords[i])) for i in range(N_EDGES)])


def curve_corner_angles(tile):
    """Angle (degrees) between the tangent arriving at each vertex along the curve and the
    tangent leaving it, shape (14,).  0 means the curve is smooth (G1) at that vertex.
    The tangents point along the handles; a handle of length 0 is an error."""
    out = np.empty(N_EDGES)
    for i, v in enumerate(tile.vertices):
        t_in = v.position - v.in_handle.position
        t_out = v.out_handle.position - v.position
        if min(np.hypot(*t_in), np.hypot(*t_out)) == 0.0:
            raise ValueError("vertex %d has a zero length handle, its tangent is undefined" % i)
        out[i] = math.degrees(_signed_angle(t_in, t_out))
    return out


def notes_cross_product(tile, i=FLAT_VERTEX):
    """z component of E_A x E_B for the two edges meeting at vertex i (the quantity C of
    the notes, taken at the flat vertex): |E_A||E_B| sin(turn).  0 means collinear."""
    ea, eb = tile.chord((i - 1) % N_EDGES), tile.chord(i)
    return float(ea[0] * eb[1] - ea[1] * eb[0])


# --------------------------------------------------------------------------
# The flow
# --------------------------------------------------------------------------
class SpectreHatFlow:
    """One Tile moved between the Spectre (tau = 0) and the Hat (tau = 1).

    Parameters
    ----------
    tau : flow parameter in [0, 1] (settable at any time).
    k0  : curve_strength of the Spectre end (tau = 0); the tile has k0 * (1 - tau).
    a   : the fixed edge length a(tau) = a (default 1).  b runs from a to a * sqrt(3).
    corner_deg : w0, turning angle of the 14th vertex P_8 at tau = 0, in degrees, in [0, 180)
                 (default 30).  The tile has w0 * (1 - tau).  0 keeps P_8 flat all along.

    Attributes
    ----------
    tile : the live ``spectremodel.Tile``.  Setting ``tau``, ``k0`` or ``corner_deg`` calls
           ``tile.layout``, opens P_8 and sets ``tile.curve_strength``, so vertices that were
           moved by hand go back to the layout; twists and ``data`` of the tile are kept.
    """

    def __init__(self, tau=0.0, k0=0.5, a=1.0, corner_deg=30.0):
        self._a = float(a)
        if not (math.isfinite(self._a) and self._a > 0.0):
            raise ValueError("a must be positive and finite, got %r" % (a,))
        self._k0 = self._check_k0(k0)
        self._corner_deg = self._check_corner(corner_deg)
        self._tau = self._check_tau(tau)
        self.tile = self.tile_at(self._tau)
        self.data = {}

    # -- validation ---------------------------------------------------------
    @staticmethod
    def _check_tau(tau):
        t = float(tau)
        if not (math.isfinite(t) and 0.0 <= t <= 1.0):
            raise ValueError("tau must be a number in [0, 1], got %r" % (tau,))
        return t

    @staticmethod
    def _check_k0(k0):
        k = float(k0)
        if not math.isfinite(k):
            raise ValueError("k0 must be finite, got %r" % (k0,))
        return k

    @staticmethod
    def _check_corner(deg):
        w = float(deg)
        if not (math.isfinite(w) and 0.0 <= w < 180.0):
            raise ValueError("corner_deg must be a number in [0, 180), got %r" % (deg,))
        return w

    # -- the laws of the flow -----------------------------------------------
    def b_of(self, tau):
        """b(tau) = a * (1 + (sqrt(3) - 1) * tau)."""
        return self._a * (1.0 + (SQRT3 - 1.0) * self._check_tau(tau))

    def k_of(self, tau):
        """curve_strength(tau) = k0 * (1 - tau)  (lambda = 1 - tau of the notes)."""
        return self._k0 * (1.0 - self._check_tau(tau))

    def corner_of(self, tau):
        """w(tau) = w0 * (1 - tau): turning angle of the 14th vertex P_8, in degrees
        (the vertex turns clockwise by this much, polygon_turning_angles()[8] = -w)."""
        return self._corner_deg * (1.0 - self._check_tau(tau))

    def opening_of(self, tau):
        """h(tau) = a * tan(w(tau) / 2): how far P_8 is pushed out of the chord P_7 -> P_9."""
        return self._a * math.tan(math.radians(self.corner_of(tau)) / 2.0)

    @staticmethod
    def arrow_angle(tau):
        """Angle between the two orientation arrows, pi * (1 - tau) (parameterisation of the
        notes: pi for the Spectre, 0 for the Hat)."""
        return math.pi * (1.0 - SpectreHatFlow._check_tau(tau))

    # -- state --------------------------------------------------------------
    @property
    def tau(self):
        return self._tau

    @tau.setter
    def tau(self, value):
        value = self._check_tau(value)  # validate first: a failed set changes nothing
        self._tau = value
        self._apply()

    @property
    def k0(self):
        return self._k0

    @k0.setter
    def k0(self, value):
        self._k0 = self._check_k0(value)
        self._apply()

    @property
    def corner_deg(self):
        return self._corner_deg

    @corner_deg.setter
    def corner_deg(self, value):
        self._corner_deg = self._check_corner(value)
        self._apply()

    @property
    def corner(self):
        """Current turning angle w of the 14th vertex in degrees."""
        return self.corner_of(self._tau)

    @property
    def a(self):
        return self._a

    @property
    def b(self):
        return self.tile.b

    @property
    def curve_strength(self):
        return self.tile.curve_strength

    @property
    def arrow(self):
        return self.arrow_angle(self._tau)

    def _open_corner(self, tile, tau):
        """Push P_8 of ``tile`` (on the standard layout) out by h(tau), perpendicular to the
        chord P_7 -> P_9, which is vertical and does not move."""
        v = tile.vertices[FLAT_VERTEX]
        v.position = v.position + np.array([self.opening_of(tau), 0.0])

    def _apply(self):
        self.tile.layout(a=self._a, b=self.b_of(self._tau))
        self._open_corner(self.tile, self._tau)
        self.tile.curve_strength = self.k_of(self._tau)

    def tile_at(self, tau):
        """A new, independent Tile for the given tau (this flow is not changed)."""
        tile = Tile(self._a, self.b_of(tau), self.k_of(tau))
        self._open_corner(tile, tau)
        return tile

    def frames(self, n):
        """``n`` independent tiles with tau = 0 ... 1 in equal steps (n >= 2)."""
        if int(n) != n or n < 2:
            raise ValueError("n must be an integer >= 2")
        return [self.tile_at(t) for t in np.linspace(0.0, 1.0, int(n))]

    # -- diagnostics --------------------------------------------------------
    def vertex_report(self):
        """List of 14 dicts {index, polygon_turn, curve_turn} (degrees) for the current tile."""
        poly, curve = polygon_turning_angles(self.tile), curve_corner_angles(self.tile)
        return [
            {"index": i, "polygon_turn": float(poly[i]), "curve_turn": float(curve[i])}
            for i in range(N_EDGES)
        ]

    def flat_vertices(self, tol_deg=1e-9, kind="polygon"):
        """Indices of the vertices whose turning angle is 0 (within tol_deg degrees).
        kind = "polygon" (straight chords) or "curve" (tangents of the Bezier edges)."""
        if kind not in ("polygon", "curve"):
            raise ValueError("kind must be 'polygon' or 'curve', got %r" % (kind,))
        angles = polygon_turning_angles(self.tile) if kind == "polygon" else curve_corner_angles(self.tile)
        return [i for i in range(N_EDGES) if abs(angles[i]) <= tol_deg]

    def effective_vertex_count(self, tol_deg=1e-9, kind="polygon"):
        """Number of vertices that are real corners (turning angle not 0)."""
        return N_EDGES - len(self.flat_vertices(tol_deg, kind))

    def __repr__(self):
        return "<SpectreHatFlow tau=%g a=%g b=%.6g k=%.4g (k0=%g) w=%.4g deg (w0=%g)>" % (
            self._tau,
            self._a,
            self.b,
            self.curve_strength,
            self._k0,
            self.corner,
            self._corner_deg,
        )


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectre_hat_collinearity_flow_model.py --tests-only)
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def _spectre_area(a, b):
    return 2 * SQRT3 * a * a + 3 * a * b + SQRT3 * b * b


def test_end_points_are_spectre_and_hat():
    f = SpectreHatFlow(0.0, k0=0.5, corner_deg=0.0)
    ref = Tile(1.0, 1.0, 0.5)
    assert np.array_equal(f.tile.control_points(), ref.control_points())
    f.tau = 1.0
    hat = Tile(1.0, SQRT3, 0.0)
    assert math.isclose(f.b, SQRT3, rel_tol=1e-15) and f.curve_strength == 0.0
    assert np.allclose(f.tile.control_points(), hat.control_points(), atol=1e-14)
    # a straight Hat: every Bezier edge is its chord
    ctrl = f.tile.control_points()
    assert np.allclose(ctrl[:, 1], ctrl[:, 0] + (ctrl[:, 3] - ctrl[:, 0]) / 2, atol=1e-14)


def test_laws_are_linear_in_tau():
    f = SpectreHatFlow(k0=0.4, a=2.0)
    for t in (0.0, 0.25, 0.5, 1.0):
        assert math.isclose(f.b_of(t), 2.0 * (1 + (SQRT3 - 1) * t), rel_tol=1e-15)
        assert math.isclose(f.k_of(t), 0.4 * (1 - t), abs_tol=1e-15)
        assert math.isclose(f.arrow_angle(t), math.pi * (1 - t), abs_tol=1e-15)
    f.tau = 0.5
    assert math.isclose(f.b, f.tile.b) and math.isclose(f.curve_strength, 0.2)
    assert math.isclose(f.arrow, math.pi / 2)


def test_tile_is_live_and_tile_at_is_independent():
    f = SpectreHatFlow(k0=0.5, corner_deg=0.0)
    tile = f.tile
    f.tau = 0.3
    assert f.tile is tile and tile.b == f.b_of(0.3)
    other = f.tile_at(0.9)
    assert other is not tile and f.tau == 0.3 and tile.b == f.b_of(0.3)
    assert np.array_equal(other.control_points(), Tile(1.0, f.b_of(0.9), f.k_of(0.9)).control_points())
    assert len(f.frames(5)) == 5 and np.allclose(f.frames(5)[2].b, f.b_of(0.5))
    f.k0 = 0.8
    assert math.isclose(f.curve_strength, 0.8 * 0.7)
    # twists and data survive the flow, hand-moved vertices go back to the layout
    tile.vertices[2].twist = 0.3
    tile.vertices[3].position = (9.0, 9.0)
    tile.data["x"] = 1
    f.tau = 0.6
    assert tile.vertices[2].twist == 0.3 and tile.data == {"x": 1}
    assert not np.allclose(tile.vertices[3].position, (9.0, 9.0))


def test_polygon_turning_angles_do_not_depend_on_tau():
    ref = polygon_turning_angles(Tile(1.0, 1.0, 0.5))
    assert np.allclose(ref / 30.0, np.round(ref / 30.0), atol=1e-9), "all turns are multiples of 30 degrees"
    f = SpectreHatFlow(k0=0.5, corner_deg=0.0)
    for t in np.linspace(0.0, 1.0, 11):
        f.tau = t
        assert np.allclose(polygon_turning_angles(f.tile), ref, atol=1e-9), t


def test_without_opening_the_14th_vertex_is_flat_all_along():
    f = SpectreHatFlow(k0=0.5, corner_deg=0.0)
    for t in np.linspace(0.0, 1.0, 11):
        f.tau = t
        assert f.flat_vertices() == [FLAT_VERTEX], t
        assert f.effective_vertex_count() == 13, t
        assert abs(notes_cross_product(f.tile)) < 1e-12, t
        assert f.flat_vertices(kind="curve") == [FLAT_VERTEX], "curved edges meet smoothly at P_8, t=%g" % t
    # no other pair of neighbouring edges is parallel at any tau
    f.tau = 0.0
    others = [abs(notes_cross_product(f.tile, i)) for i in range(N_EDGES) if i != FLAT_VERTEX]
    assert min(others) > 0.1


def test_curve_corners_equal_polygon_turns_for_every_tau_and_k():
    for k0 in (-0.7, 0.0, 0.5, 1.3):
        f = SpectreHatFlow(k0=k0)
        for t in (0.0, 0.5, 1.0):
            f.tau = t
            assert np.allclose(curve_corner_angles(f.tile), polygon_turning_angles(f.tile), atol=1e-9), (k0, t)
    # but the curve itself does move with k: tau = 0 is not the straight 14-gon
    f = SpectreHatFlow(0.0, k0=0.5)
    q = f.tile.sample_edges(4)[0][1]  # t = 1/4: off the chord (the edge is an S curve, t = 1/2 is on it)
    p0, d = f.tile.vertices[0].position, f.tile.chord(0)
    assert abs(d[0] * (q - p0)[1] - d[1] * (q - p0)[0]) / np.hypot(*d) > 0.05


def test_area_follows_the_closed_form_along_the_flow():
    f = SpectreHatFlow(k0=0.7, corner_deg=0.0)
    for t in np.linspace(0.0, 1.0, 9):
        f.tau = t
        assert math.isclose(f.tile.area(), _spectre_area(1.0, f.b), rel_tol=1e-12), t
    f.tau = 1.0
    assert math.isclose(f.tile.area(), _spectre_area(1.0, SQRT3), rel_tol=1e-12)


def test_closure_and_edge_lengths_at_the_ends():
    for t in (0.0, 1.0):
        tile = SpectreHatFlow(t, corner_deg=0.0).tile
        assert np.allclose(edge_vectors(tile.a, tile.b).sum(axis=0), 0.0, atol=1e-12)
        lengths = np.hypot(*np.array([tile.chord(i) for i in range(N_EDGES)]).T)
        expected = [tile.b if c == "b" else tile.a for c in "bbaabbaaaabbaa"]
        assert np.allclose(lengths, expected, atol=1e-12)


def test_validation():
    f = SpectreHatFlow(0.2, k0=0.5)
    for bad in (-0.1, 1.1, float("nan"), float("inf"), "x"):
        assert _raises(ValueError, setattr, f, "tau", bad), bad
        assert _raises(ValueError, SpectreHatFlow, bad)
    assert f.tau == 0.2 and f.b == f.b_of(0.2), "failed sets must not change state"
    for bad in (float("nan"), float("inf")):
        assert _raises(ValueError, setattr, f, "k0", bad)
    for bad in (0.0, -1.0, float("nan")):
        assert _raises(ValueError, SpectreHatFlow, 0.0, 0.5, bad)
    for bad in (1, 2.5, 0):
        assert _raises(ValueError, f.frames, bad)
    assert _raises(ValueError, f.flat_vertices, kind="nope")
    for bad in (-1.0, 180.0, 200.0, float("nan"), float("inf")):
        assert _raises(ValueError, setattr, f, "corner_deg", bad), bad
        assert _raises(ValueError, SpectreHatFlow, 0.0, 0.5, 1.0, bad), bad
    assert f.corner_deg == 30.0 and f.tau == 0.2
    assert _raises(AttributeError, setattr, f, "a", 2.0)
    assert _raises(AttributeError, setattr, f, "b", 2.0)


def test_opening_the_14th_vertex_changes_two_edge_directions_and_keeps_the_outline_closed():
    for a in (1.0, 1.7):
        f = SpectreHatFlow(k0=0.5, a=a, corner_deg=30.0)
        plain = Tile(a, 1.0, 0.0)
        for t in np.linspace(0.0, 1.0, 11):
            f.tau = t
            w, h = f.corner_of(t), f.opening_of(t)
            assert math.isclose(w, 30.0 * (1 - t), abs_tol=1e-12)
            assert math.isclose(h, a * math.tan(math.radians(w) / 2), abs_tol=1e-14)
            tile = f.tile
            chords = np.array([tile.chord(i) for i in range(N_EDGES)])
            assert np.allclose(chords.sum(axis=0), 0.0, atol=1e-12), "closed"
            assert math.isclose(polygon_turning_angles(tile)[FLAT_VERTEX], -w, abs_tol=1e-9)
            # the chord P_7 -> P_9 is fixed, only P_8 moves
            plain.layout(a=a, b=f.b)
            moved = [i for i in range(N_EDGES) if not np.allclose(tile.vertices[i].position, plain.vertices[i].position)]
            assert moved == ([FLAT_VERTEX] if t < 1.0 else []), (t, moved)
            # directions of edges 7 and 8: -90 deg -/+ w/2, lengths a / cos(w/2)
            ang = [math.degrees(math.atan2(chords[i][1], chords[i][0])) for i in (7, 8)]
            assert math.isclose(ang[0], -90.0 + w / 2, abs_tol=1e-9) and math.isclose(ang[1], -90.0 - w / 2, abs_tol=1e-9)
            lens = np.hypot(*chords[[7, 8]].T)
            assert np.allclose(lens, a / math.cos(math.radians(w) / 2), atol=1e-12)


def test_turns_of_the_neighbours_of_the_opened_vertex():
    f = SpectreHatFlow(k0=0.5, corner_deg=30.0)
    plain = polygon_turning_angles(Tile(1.0, 1.0, 0.5))
    for t in (0.0, 0.3, 0.8, 1.0):
        f.tau = t
        turns, w = polygon_turning_angles(f.tile), f.corner_of(t)
        assert math.isclose(turns[7], -60.0 + w / 2, abs_tol=1e-9) and math.isclose(turns[9], -60.0 + w / 2, abs_tol=1e-9)
        assert math.isclose(turns.sum(), -360.0, abs_tol=1e-9)
        rest = [i for i in range(N_EDGES) if i not in (7, 8, 9)]
        assert np.allclose(turns[rest], plain[rest], atol=1e-9)


def test_vertex_count_goes_from_14_to_13_and_the_cross_product_closes():
    f = SpectreHatFlow(k0=0.5, corner_deg=30.0)
    prev = None
    for t in np.linspace(0.0, 1.0, 11):
        f.tau = t
        w, h = f.corner_of(t), f.opening_of(t)
        c = notes_cross_product(f.tile)
        assert math.isclose(c, -(1.0 + h * h) * math.sin(math.radians(w)), abs_tol=1e-12), t
        if t < 1.0:
            assert f.effective_vertex_count() == 14 and f.effective_vertex_count(kind="curve") == 14, t
            assert f.flat_vertices() == [] and c < 0.0
        else:
            assert f.effective_vertex_count() == 13 and f.flat_vertices() == [FLAT_VERTEX] and abs(c) < 1e-12
            assert f.flat_vertices(kind="curve") == [FLAT_VERTEX]
        assert prev is None or abs(c) < prev, "the cross product shrinks monotonically"
        prev = abs(c)


def test_default_flow_ends_in_the_straight_hat_and_starts_open():
    f = SpectreHatFlow(k0=0.5)
    assert f.corner_deg == 30.0 and math.isclose(f.corner, 30.0)
    assert f.effective_vertex_count() == 14
    f.tau = 1.0
    assert np.allclose(f.tile.control_points(), Tile(1.0, SQRT3, 0.0).control_points(), atol=1e-14)
    f.corner_deg = 0.0
    f.tau = 0.0
    assert np.array_equal(f.tile.control_points(), Tile(1.0, 1.0, 0.5).control_points())


def test_area_of_the_opened_tile_adds_one_triangle():
    for k0 in (-0.6, 0.0, 0.5):
        f = SpectreHatFlow(k0=k0, a=1.3, corner_deg=40.0)
        for t in np.linspace(0.0, 1.0, 6):
            f.tau = t
            want = _spectre_area(1.3, f.b) + 1.3 * f.opening_of(t)
            assert math.isclose(f.tile.area(), want, rel_tol=1e-12), (k0, t)


def run_tests(verbose=True):
    """Run every ``test_*`` function in this module; return True if none failed."""
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001 - report and keep going
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-68s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Command line demo (matplotlib)
# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Draw the Spectre -> Hat flow (tau = 0 Spectre, tau = 1 Hat) on spectremodel.Tile."
    )
    p.add_argument("--tau", type=float, default=None, help="draw a single tile at this tau in [0, 1]")
    p.add_argument("--frames", type=int, default=6, help="without --tau: number of tiles tau = 0..1 in a row (default 6)")
    p.add_argument("--k0", type=float, default=0.5, help="curve_strength at tau = 0 (default 0.5)")
    p.add_argument("--corner", type=float, default=30.0, metavar="DEG",
                   help="turning angle of the 14th vertex at tau = 0 in degrees, 0 keeps it flat (default 30)")
    p.add_argument("--handles", action="store_true", help="draw the control handles")
    p.add_argument("--report", action="store_true", help="print the vertex report for each drawn tau")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="save the figure to this file")
    args = p.parse_args(argv)

    if args.tests_only:
        print("spectre_hat_collinearity_flow_model self tests:")
        return 0 if run_tests() else 1

    try:
        flow = SpectreHatFlow(0.0, k0=args.k0, corner_deg=args.corner)
    except ValueError as exc:
        p.error(str(exc))
    if args.tau is not None:
        try:
            taus = [flow._check_tau(args.tau)]
        except ValueError as exc:
            p.error(str(exc))
    else:
        if args.frames < 2:
            p.error("--frames must be at least 2")
        taus = list(np.linspace(0.0, 1.0, args.frames))

    import matplotlib

    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from spectremodel import draw

    fig, axes = plt.subplots(1, len(taus), figsize=(3.2 * len(taus), 3.6), squeeze=False)
    for ax, t in zip(axes[0], taus):
        flow.tau = t
        draw(flow.tile, ax, handles=args.handles)
        ax.set_title("tau %.2f   b %.3f   k %.3f\n%d corners, w %.1f deg, arrow %.0f deg" % (
            t, flow.b, flow.curve_strength, flow.effective_vertex_count(), flow.corner, math.degrees(flow.arrow)),
            fontsize=8)
        ax.axis("off")
        if args.report:
            print(flow)
            for r in flow.vertex_report():
                print("   P_%-2d polygon %8.3f   curve %8.3f" % (r["index"], r["polygon_turn"], r["curve_turn"]))
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
