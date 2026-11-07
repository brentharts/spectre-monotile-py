"""
spectremodel.py - the curved Spectre monotile as an editable object model.

spectre.py draws the curved tile from three numbers (a, b, curve_strength) and
hands back two arrays.  Here the tile is a live object that other code can use as
an API: it has vertices, every vertex has two Bezier control handles, and each of
them carries its own options (so far a *twist*) and a free ``data`` dict.  Nothing
is cached, so the curve you ask for always reflects the current state.

    from spectremodel import Tile

    tile = Tile(a=1, b=1, curve_strength=0.5)   # untwisted: the same curve as spectre.py
    v = tile.vertices[3]
    v.twist = 0.3                                # turns both handles of vertex 3 (radians)
    v.out_handle.twist = -0.2                    # extra turn of one handle only
    v.data["label"] = "tip"                      # anything, on the tile, a vertex or a handle
    x, y = tile.outline(samples=50)              # recomputed on every call
    tile.vertices[5].position = (2.0, 1.0)       # vertices can be moved as well

Model
-----
* 14 vertices P_0..P_13 (the default layout has P_0 = (0, 0) and uses the edge
  vectors of spectre.py) and 14 edges.  Edge i is the cubic Bezier from P_i to
  P_(i+1); edge 13 closes the tile back to P_0.
* Every vertex owns two ``BezHandle`` objects.  ``out_handle`` is the first inner
  control point of the edge that leaves the vertex (edge i for P_i); ``in_handle``
  is the second inner control point of the edge that arrives (edge i-1 for P_i).
  Edge i therefore has the control points
      [P_i, P_i.out_handle, P_(i+1).in_handle, P_(i+1)].
* Untwisted, the handles are those of spectre.py.  With the chord d = P_(i+1) - P_i,
  n = (d_y, -d_x) and k = curve_strength:
      out handle of P_i      =  P_i      + d/2 - k n
      in handle of P_(i+1)   =  P_(i+1)  - d/2 + k n
* Twist.  The offset of a handle from its vertex is rotated about that vertex by
  the angle ``vertex.twist + handle.twist`` (radians, counter-clockwise positive).
  So ``vertex.twist`` turns both handles of a vertex together, like rotating a node
  in a vector editor, and ``handle.twist`` turns a single handle.  A twist never
  changes the length of a handle and never moves a vertex.  With every twist at 0
  the curve is the one of spectre.py (checked by the self tests).
* Vertex positions are stored, not derived: moving a vertex moves the ends of its
  two edges and their handles follow the new chords.  ``Tile.layout(a, b)`` puts
  all vertices back on the standard layout (twists and data are kept).

The area 2*sqrt(3)*a^2 + 3*a*b + sqrt(3)*b^2 of the spectre.py tile does not depend
on curve_strength.  That is not true any more once handles are twisted or vertices
are moved, so ``Tile.area()`` is the exact area of the current outline.

Only NumPy is needed; the `bezier` package is not used.  matplotlib is imported
lazily by the command line demo (python3 spectremodel.py --help).
"""

import argparse
import copy
import math
import operator
import sys

import numpy as np

__all__ = [
    "N_EDGES",
    "EDGE_KIND",
    "BezHandle",
    "Vertex",
    "Tile",
    "edge_vectors",
    "layout_vertices",
    "run_tests",
]

N_EDGES = 14
# Length type of each edge (edge i runs P_i -> P_(i+1)); same order as spectre.py.
EDGE_KIND = "bbaabbaaaabbaa"


# --------------------------------------------------------------------------
# Standard layout
# --------------------------------------------------------------------------
def edge_vectors(a=1.0, b=1.0):
    """The 14 edge vectors of the standard tile, shape (14, 2) (order of spectre.py)."""
    c, s = 0.5, math.sqrt(3.0) / 2.0  # cos / sin of 60 degrees
    return np.array(
        [
            (c * b, s * b),
            (b, 0.0),
            (0.0, a),
            (s * a, c * a),
            (c * b, -s * b),
            (-c * b, -s * b),
            (s * a, -c * a),
            (0.0, -a),
            (0.0, -a),
            (-s * a, -c * a),
            (-c * b, s * b),
            (-b, 0.0),
            (0.0, a),
            (-s * a, c * a),
        ],
        dtype=float,
    )


def layout_vertices(a=1.0, b=1.0):
    """The 14 vertices of the standard tile, shape (14, 2); P_0 = (0, 0)."""
    return np.vstack([[0.0, 0.0], np.cumsum(edge_vectors(a, b), axis=0)[:-1]])


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------
def _finite(value, what):
    v = float(value)
    if not math.isfinite(v):
        raise ValueError("%s must be finite, got %r" % (what, value))
    return v


def _point(value, what):
    p = np.array(value, dtype=float)
    if p.shape != (2,) or not np.all(np.isfinite(p)):
        raise ValueError("%s must be two finite numbers (x, y), got %r" % (what, value))
    return p


def _readonly(arr):
    arr.flags.writeable = False
    return arr


def _rotate(vec, angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c * vec[0] - s * vec[1], s * vec[0] + c * vec[1]])


def _bernstein(t):
    """Cubic Bernstein basis at parameters t, shape (len(t), 4)."""
    t = np.asarray(t, dtype=float)
    u = 1.0 - t
    return np.stack([u**3, 3 * u**2 * t, 3 * u * t**2, t**3], axis=1)


def _check_samples(samples):
    if int(samples) != samples or samples < 1:
        raise ValueError("samples must be a positive integer")
    return int(samples)


# --------------------------------------------------------------------------
# Objects
# --------------------------------------------------------------------------
class BezHandle:
    """One Bezier control handle, attached to a vertex.

    Attributes
    ----------
    vertex : the Vertex this handle hangs on (the rotation centre of its twist).
    kind   : "out" (first inner control point of the edge leaving the vertex) or
             "in" (second inner control point of the edge arriving at it).
    twist  : own rotation about the vertex in radians (counter-clockwise positive);
             ``twist_deg`` is the same in degrees.
    data   : free dict for whatever the caller wants to attach.

    Read-only: ``edge`` (index of the edge this handle shapes), ``total_twist``
    (vertex twist + own twist), ``base_offset`` (untwisted offset from the vertex),
    ``offset`` (twisted offset from the vertex) and ``position`` (absolute point).
    """

    def __init__(self, vertex, kind):
        if kind not in ("out", "in"):
            raise ValueError("kind must be 'out' or 'in', got %r" % (kind,))
        self.vertex = vertex
        self.kind = kind
        self._twist = 0.0
        self.data = {}

    @property
    def tile(self):
        return self.vertex.tile

    @property
    def edge(self):
        i = self.vertex.index
        return i if self.kind == "out" else (i - 1) % N_EDGES

    @property
    def twist(self):
        return self._twist

    @twist.setter
    def twist(self, value):
        self._twist = _finite(value, "twist")

    @property
    def twist_deg(self):
        return math.degrees(self._twist)

    @twist_deg.setter
    def twist_deg(self, value):
        self.twist = math.radians(_finite(value, "twist_deg"))

    @property
    def total_twist(self):
        return self.vertex.twist + self._twist

    @property
    def base_offset(self):
        """Offset from the vertex before any twist (the spectre.py handle)."""
        d = self.tile.chord(self.edge)
        n = np.array([d[1], -d[0]])
        k = self.tile.curve_strength
        return _readonly(d / 2.0 - k * n if self.kind == "out" else -d / 2.0 + k * n)

    @property
    def offset(self):
        """Offset from the vertex after the twist."""
        # Future per-handle options (length, angle, ...) are applied here, on top of base_offset.
        return _readonly(_rotate(self.base_offset, self.total_twist))

    @property
    def position(self):
        """Absolute position of the control point."""
        return _readonly(self.vertex.position + self.offset)

    def __repr__(self):
        return "<BezHandle %s of vertex %d (edge %d), twist=%.4g>" % (
            self.kind,
            self.vertex.index,
            self.edge,
            self._twist,
        )


class Vertex:
    """A corner of the tile with its two handles.

    Attributes
    ----------
    tile       : the Tile it belongs to.
    index      : 0..13.
    position   : (x, y); assign a pair to move the vertex (read as a read-only array).
    twist      : rotation of both handles about the vertex in radians (counter-clockwise
                 positive), added to each handle's own twist; ``twist_deg`` in degrees.
    out_handle : BezHandle of the edge leaving this vertex.
    in_handle  : BezHandle of the edge arriving at this vertex.
    data       : free dict for whatever the caller wants to attach.
    """

    def __init__(self, tile, index, position):
        self.tile = tile
        self.index = index
        self._position = _point(position, "position")
        self._twist = 0.0
        self.data = {}
        self.out_handle = BezHandle(self, "out")
        self.in_handle = BezHandle(self, "in")

    @property
    def handles(self):
        return (self.out_handle, self.in_handle)

    @property
    def position(self):
        view = self._position.view()
        view.flags.writeable = False
        return view

    @position.setter
    def position(self, value):
        self._position = _point(value, "position")

    @property
    def twist(self):
        return self._twist

    @twist.setter
    def twist(self, value):
        self._twist = _finite(value, "twist")

    @property
    def twist_deg(self):
        return math.degrees(self._twist)

    @twist_deg.setter
    def twist_deg(self, value):
        self.twist = math.radians(_finite(value, "twist_deg"))

    def __repr__(self):
        return "<Vertex %d at (%.4g, %.4g), twist=%.4g>" % (
            self.index,
            self._position[0],
            self._position[1],
            self._twist,
        )


class Tile:
    """The curved Spectre monotile as an editable object.

    Parameters
    ----------
    a, b : edge lengths of the standard layout (as in spectre.py).
    curve_strength : k of spectre.py, shared by every edge (settable at any time).

    Attributes
    ----------
    vertices       : tuple of the 14 Vertex objects.
    handles        : tuple of the 28 BezHandle objects (out, in of vertex 0, then vertex 1, ...).
    curve_strength : k, validated on assignment.
    a, b           : lengths of the last ``layout`` (read-only; call ``layout`` to change them).
    data           : free dict for whatever the caller wants to attach.

    Nothing is cached: every query (``outline``, ``control_points``, ``area``, ...)
    is computed from the current vertices, twists and curve_strength.
    """

    def __init__(self, a=1.0, b=1.0, curve_strength=0.5):
        self.data = {}
        self.curve_strength = curve_strength
        self._a, self._b = _finite(a, "a"), _finite(b, "b")
        self._vertices = tuple(
            Vertex(self, i, p) for i, p in enumerate(layout_vertices(self._a, self._b))
        )

    # -- parameters ---------------------------------------------------------
    @property
    def curve_strength(self):
        return self._curve_strength

    @curve_strength.setter
    def curve_strength(self, value):
        self._curve_strength = _finite(value, "curve_strength")

    @property
    def a(self):
        return self._a

    @property
    def b(self):
        return self._b

    def layout(self, a=None, b=None):
        """Put every vertex on the standard layout for lengths a, b (None keeps the
        current value).  Twists, curve_strength and data are kept; moved vertices are reset."""
        a = self._a if a is None else _finite(a, "a")
        b = self._b if b is None else _finite(b, "b")
        for v, p in zip(self._vertices, layout_vertices(a, b)):
            v._position = p.copy()
        self._a, self._b = a, b

    # -- parts --------------------------------------------------------------
    @property
    def vertices(self):
        return self._vertices

    @property
    def handles(self):
        return tuple(h for v in self._vertices for h in v.handles)

    @staticmethod
    def _edge_index(i):
        i = operator.index(i)
        if not 0 <= i < N_EDGES:
            raise IndexError("edge index %d out of range 0..%d" % (i, N_EDGES - 1))
        return i

    def chord(self, i):
        """Vector P_(i+1) - P_i of edge i."""
        i = self._edge_index(i)
        return self._vertices[(i + 1) % N_EDGES].position - self._vertices[i].position

    def edge_handles(self, i):
        """The two handles that shape edge i: (out handle of P_i, in handle of P_(i+1))."""
        i = self._edge_index(i)
        return self._vertices[i].out_handle, self._vertices[(i + 1) % N_EDGES].in_handle

    # -- the curve (computed on demand) --------------------------------------
    def edge_control_points(self, i):
        """Bezier control points of edge i, shape (4, 2): [P_i, out handle, in handle, P_(i+1)]."""
        i = self._edge_index(i)
        v0, v1 = self._vertices[i], self._vertices[(i + 1) % N_EDGES]
        return np.array([v0.position, v0.out_handle.position, v1.in_handle.position, v1.position])

    def control_points(self):
        """Control points of all edges, shape (14, 4, 2)."""
        return np.array([self.edge_control_points(i) for i in range(N_EDGES)])

    def sample_edges(self, samples=50):
        """Every edge sampled from its start to its end vertex, both included:
        shape (14, samples + 1, 2)."""
        samples = _check_samples(samples)
        return np.einsum("mj,ejd->emd", _bernstein(np.linspace(0.0, 1.0, samples + 1)), self.control_points())

    def outline(self, samples=50, closed=True):
        """Outline of the tile as two arrays (x, y).

        samples : points per edge (the vertices are always included).
        closed  : repeat the first point at the end so that the outline closes.
        """
        samples = _check_samples(samples)
        pts = np.einsum("mj,ejd->emd", _bernstein(np.arange(samples) / samples), self.control_points())
        pts = pts.reshape(-1, 2)
        if closed:
            pts = np.vstack([pts, pts[:1]])
        return pts[:, 0].copy(), pts[:, 1].copy()

    def area(self):
        """Area enclosed by the current outline (absolute value of the signed area;
        the standard tile runs clockwise), exact for the Bezier edges."""
        ctrl = self.control_points()
        nodes, weights = np.polynomial.legendre.leggauss(6)  # exact: integrand has degree 5
        t, w = 0.5 * (nodes + 1.0), 0.5 * weights
        u = 1.0 - t
        pts = np.einsum("mj,ejd->emd", _bernstein(t), ctrl)
        quad = np.stack([u**2, 2 * u * t, t**2], axis=1)
        vel = np.einsum("mj,ejd->emd", quad, 3.0 * (ctrl[:, 1:] - ctrl[:, :-1]))
        cross = pts[..., 0] * vel[..., 1] - pts[..., 1] * vel[..., 0]
        return abs(0.5 * float((cross @ w).sum()))

    def copy(self):
        """Independent copy with the same vertices, twists and curve_strength.
        ``data`` dicts are deep-copied, so their contents must be copyable."""
        return copy.deepcopy(self)

    def __repr__(self):
        return "<Tile a=%g b=%g curve_strength=%g, %d vertices>" % (
            self._a,
            self._b,
            self._curve_strength,
            N_EDGES,
        )


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectremodel.py --tests-only)
# --------------------------------------------------------------------------
class _Skip(Exception):
    pass


def _spectre_area(a, b):
    r3 = math.sqrt(3.0)
    return 2 * r3 * a * a + 3 * a * b + r3 * b * b


def _shoelace(x, y):
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def _scrambled_tile(seed=0, a=1.2, b=0.8, k=0.45):
    rng = np.random.default_rng(seed)
    tile = Tile(a, b, k)
    for v in tile.vertices:
        v.twist = rng.uniform(-1, 1)
        for h in v.handles:
            h.twist = rng.uniform(-1, 1)
    return tile


def _changed_edges(t1, t2, samples=12):
    e1, e2 = t1.sample_edges(samples), t2.sample_edges(samples)
    return [i for i in range(N_EDGES) if not np.allclose(e1[i], e2[i], atol=1e-13)]


def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def test_untwisted_tile_is_spectre_py():
    try:
        import spectre
    except ImportError as exc:
        raise _Skip("spectre.py not importable: %s" % exc)
    for a, b, k in [(1.0, 1.0, 0.5), (1.3, 0.8, 0.45), (0.9, 1.6, -0.7), (1.0, 1.0, 0.0)]:
        xs, ys = spectre.generate_monotile(a, b, k)
        pts = Tile(a, b, k).sample_edges(samples=49)  # 50 points per edge, ends included
        for i in range(N_EDGES):
            lo = 1 + 50 * i
            assert np.allclose(pts[i, :, 0], xs[lo : lo + 50], atol=1e-12), "x, edge %d" % i
            assert np.allclose(pts[i, :, 1], ys[lo : lo + 50], atol=1e-12), "y, edge %d" % i


def test_structure_and_closure():
    tile = Tile(1.3, 0.8, 0.4)
    assert len(tile.vertices) == N_EDGES == len(EDGE_KIND) and len(tile.handles) == 2 * N_EDGES
    assert np.allclose(edge_vectors(1.3, 0.8).sum(axis=0), 0.0, atol=1e-12)
    for i, v in enumerate(tile.vertices):
        assert v.index == i and v.tile is tile
        assert v.out_handle.vertex is v and v.in_handle.vertex is v
        assert v.out_handle.edge == i and v.in_handle.edge == (i - 1) % N_EDGES
        o, n = tile.edge_handles(i)
        assert o is v.out_handle and n is tile.vertices[(i + 1) % N_EDGES].in_handle
    assert tile.handles[0] is tile.vertices[0].out_handle and tile.handles[1] is tile.vertices[0].in_handle
    ctrl = tile.control_points()
    assert ctrl.shape == (N_EDGES, 4, 2)
    assert np.array_equal(ctrl[13, 3], ctrl[0, 0]), "edge 13 must end exactly at vertex 0"
    x, y = tile.outline(samples=25)
    assert x[0] == x[-1] and y[0] == y[-1] and len(x) == 25 * N_EDGES + 1
    assert np.allclose(np.column_stack([x, y])[:-1:25], layout_vertices(1.3, 0.8), atol=1e-12)
    assert len(tile.outline(samples=5, closed=False)[0]) == 5 * N_EDGES


def test_twist_rotates_the_handle_about_its_vertex():
    tile = Tile(1.1, 0.7, 0.4)
    v = tile.vertices[4]
    h = v.out_handle
    base = h.offset.copy()
    P = v.position.copy()
    for angle in (0.5, -1.2, 2.0 * math.pi, 3.0):
        h.twist = angle
        off = h.offset
        assert math.isclose(np.hypot(*off), np.hypot(*base), rel_tol=1e-12), "length must not change"
        got = math.atan2(base[0] * off[1] - base[1] * off[0], base @ off)
        want = (angle + math.pi) % (2 * math.pi) - math.pi
        assert math.isclose(got, want, abs_tol=1e-12), "counter-clockwise positive"
        assert np.allclose(h.position, P + off) and np.array_equal(v.position, P)
    # the twist reaches the evaluated curve: the tangent leaving P_4 points along the handle
    h.twist = 0.5
    pts = tile.sample_edges(samples=20000)[4]
    tang = pts[1] - pts[0]
    off = h.offset
    assert abs(math.atan2(tang[0] * off[1] - tang[1] * off[0], tang @ off)) < 1e-3
    # and the other handle of the same edge is untouched
    assert np.allclose(tile.edge_handles(4)[1].offset, Tile(1.1, 0.7, 0.4).edge_handles(4)[1].offset)


def test_vertex_twist_turns_both_handles_and_adds_to_handle_twist():
    t1, t2 = Tile(1, 1, 0.5), Tile(1, 1, 0.5)
    t1.vertices[6].twist = 0.3
    for h in t2.vertices[6].handles:
        h.twist = 0.3
    assert np.allclose(t1.control_points(), t2.control_points(), atol=1e-14)
    t3 = Tile(1, 1, 0.5)
    v = t3.vertices[6]
    v.twist = 0.1
    v.out_handle.twist = 0.2
    assert math.isclose(v.out_handle.total_twist, 0.3) and math.isclose(v.in_handle.total_twist, 0.1)
    ref = Tile(1, 1, 0.5)
    ref.vertices[6].out_handle.twist = 0.3
    ref.vertices[6].in_handle.twist = 0.1
    assert np.allclose(t3.control_points(), ref.control_points(), atol=1e-14)
    t4 = Tile(1, 1, 0.5)
    t4.vertices[2].twist_deg = 90.0
    t4.vertices[2].out_handle.twist_deg = -45.0
    assert math.isclose(t4.vertices[2].twist, math.pi / 2) and math.isclose(t4.vertices[2].out_handle.twist_deg, -45.0)


def test_a_twist_changes_only_the_edges_that_own_the_handle():
    base = Tile(1, 1, 0.5)
    for i in (0, 6, 13):
        t = base.copy()
        t.vertices[i].twist = 0.4
        assert sorted(_changed_edges(base, t)) == sorted({(i - 1) % N_EDGES, i}), "vertex %d" % i
        t = base.copy()
        t.vertices[i].out_handle.twist = 0.4
        assert _changed_edges(base, t) == [i], "out handle %d" % i
        t = base.copy()
        t.vertices[i].in_handle.twist = 0.4
        assert _changed_edges(base, t) == [(i - 1) % N_EDGES], "in handle %d" % i


def test_twists_never_move_vertices_and_can_be_undone_exactly():
    base = Tile(1.2, 0.8, 0.45)
    ref = base.control_points()
    tile = _scrambled_tile(3)
    x, y = tile.outline(samples=20)
    assert np.allclose(np.column_stack([x, y])[:-1:20], layout_vertices(1.2, 0.8), atol=1e-12)
    for v in tile.vertices:
        v.twist = 0.0
        for h in v.handles:
            h.twist = 0.0
    assert np.array_equal(tile.control_points(), ref)


def test_area_is_exact_and_depends_on_twist():
    for a, b in [(1.0, 1.0), (1.4, 0.9)]:
        for k in (-0.8, 0.0, 0.5, 1.3):
            assert math.isclose(Tile(a, b, k).area(), _spectre_area(a, b), abs_tol=1e-12), (a, b, k)
    tile = _scrambled_tile(5)
    x, y = tile.outline(samples=4000)
    assert math.isclose(tile.area(), abs(_shoelace(x, y)), abs_tol=1e-5)
    assert abs(tile.area() - _spectre_area(1.2, 0.8)) > 1e-2, "twisted handles must change the area"


def test_moving_a_vertex_changes_only_its_two_edges():
    base = Tile(1, 1, 0.5)
    t = base.copy()
    t.vertices[5].position = t.vertices[5].position + np.array([0.2, -0.1])
    assert _changed_edges(base, t) == [4, 5]
    x, y = t.outline(samples=10)
    assert np.allclose(np.column_stack([x, y])[50], t.vertices[5].position, atol=1e-12)
    # handles follow the new chords (not the old ones)
    d = t.chord(5)
    assert np.allclose(t.vertices[5].out_handle.base_offset, d / 2 - 0.5 * np.array([d[1], -d[0]]))
    # layout() puts vertices back and keeps twists and data
    t.vertices[2].twist = 0.3
    t.vertices[5].data["keep"] = 1
    t.layout()
    assert np.allclose(t.vertices[5].position, base.vertices[5].position)
    assert t.vertices[2].twist == 0.3 and t.vertices[5].data == {"keep": 1}
    t.layout(a=2.0)
    assert t.a == 2.0 and t.b == 1.0 and np.allclose(t.vertices[2].position, layout_vertices(2.0, 1.0)[2])


def test_curve_strength_is_live():
    tile = Tile(1, 1, 0.5)
    before = tile.outline()
    tile.curve_strength = 0.1
    assert not np.allclose(tile.outline()[0], before[0])
    assert np.allclose(np.column_stack(tile.outline()), np.column_stack(Tile(1, 1, 0.1).outline()), atol=1e-15)
    tile.curve_strength = -0.1  # mirrored curves are allowed, as in spectre.py
    assert tile.curve_strength == -0.1


def test_data_is_free_survives_edits_and_copies_independently():
    tile = Tile(1, 1, 0.5)
    v = tile.vertices[2]
    tile.data["name"] = "demo"
    v.data["tags"] = ["x"]
    v.out_handle.data["weight"] = 3
    tile.curve_strength = 0.2
    v.twist = 0.1
    tile.layout(a=2.0)
    assert tile.data == {"name": "demo"} and v.data == {"tags": ["x"]} and v.out_handle.data == {"weight": 3}
    assert Tile().data == {} and Tile().vertices[0].data == {} and Tile().vertices[0].in_handle.data == {}
    c = tile.copy()
    cv = c.vertices[2]
    assert cv is not v and cv.tile is c and cv.out_handle.vertex is cv and c.handles[5].tile is c
    assert cv.twist == 0.1 and c.curve_strength == 0.2 and cv.data == {"tags": ["x"]}
    cv.twist = 0.9
    cv.data["tags"].append("y")
    c.vertices[3].position = (9.0, 9.0)
    assert v.twist == 0.1 and v.data["tags"] == ["x"]
    assert np.allclose(tile.vertices[3].position, layout_vertices(2.0, 1.0)[3])


def test_validation():
    tile = Tile()
    v, h = tile.vertices[1], tile.vertices[1].out_handle
    for bad in (float("nan"), float("inf"), -float("inf")):
        assert _raises(ValueError, setattr, v, "twist", bad)
        assert _raises(ValueError, setattr, h, "twist", bad)
        assert _raises(ValueError, setattr, h, "twist_deg", bad)
        assert _raises(ValueError, setattr, tile, "curve_strength", bad)
        assert _raises(ValueError, Tile, bad, 1.0, 0.5)
        assert _raises(ValueError, tile.layout, b=bad)
    for bad in ((1.0,), (1.0, 2.0, 3.0), [[1.0, 2.0]], (float("nan"), 0.0), "xy"):
        assert _raises(ValueError, setattr, v, "position", bad), bad
    before = v.position.copy()
    assert np.array_equal(v.position, before) and h.twist == 0.0 and v.twist == 0.0, "failed sets must not change state"
    assert _raises(ValueError, v.position.__setitem__, 0, 5.0), "position array is read-only"
    assert _raises(ValueError, h.position.__setitem__, 0, 5.0)
    assert _raises(AttributeError, setattr, tile, "a", 2.0), "a and b change only through layout()"
    assert _raises(AttributeError, setattr, tile, "vertices", ())
    for bad in (0, -3, 2.5):
        assert _raises(ValueError, tile.outline, samples=bad)
        assert _raises(ValueError, tile.sample_edges, samples=bad)
    for bad in (14, -1):
        assert _raises(IndexError, tile.edge_control_points, bad)
        assert _raises(IndexError, tile.edge_handles, bad)
        assert _raises(IndexError, tile.chord, bad)
    assert _raises(ValueError, BezHandle, v, "sideways")


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
            print("  %-62s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Command line demo (matplotlib)
# --------------------------------------------------------------------------
def _vertex_from(tile, text):
    i = int(text)
    if not 0 <= i < N_EDGES:
        raise ValueError("vertex index %d out of range 0..%d" % (i, N_EDGES - 1))
    return tile.vertices[i]


def _apply_twists(tile, vertex_specs, handle_specs, parser):
    for spec in vertex_specs:
        try:
            idx, deg = spec.split("=")
            _vertex_from(tile, idx).twist_deg = float(deg)
        except ValueError:
            parser.error("--vertex-twist expects I=DEG with I in 0..%d, got %r" % (N_EDGES - 1, spec))
    for spec in handle_specs:
        try:
            lhs, deg = spec.split("=")
            idx, kind = lhs.split(".")
            v = _vertex_from(tile, idx)
            {"out": v.out_handle, "in": v.in_handle}[kind].twist_deg = float(deg)
        except (ValueError, KeyError):
            parser.error("--handle-twist expects I.out=DEG or I.in=DEG with I in 0..%d, got %r" % (N_EDGES - 1, spec))


def draw(tile, ax, handles=False, samples=60):
    """Draw the outline of ``tile`` on a matplotlib axes (optionally with its handles)."""
    x, y = tile.outline(samples)
    ax.plot(x, y, color="black", linewidth=1)
    P = np.array([v.position for v in tile.vertices])
    ax.plot(P[:, 0], P[:, 1], "o", ms=3, color="0.3")
    if handles:
        for h in tile.handles:
            p, q = h.vertex.position, h.position
            col = "tab:red" if h.kind == "out" else "tab:blue"
            ax.plot([p[0], q[0]], [p[1], q[1]], "-", color=col, lw=0.7)
            ax.plot(q[0], q[1], ".", color=col, ms=5)
    ax.set_aspect("equal")


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Draw a Spectre monotile with the editable model of spectremodel.py "
        "(without twists it is the same curve as spectre.py)."
    )
    p.add_argument("a", type=float, nargs="?", default=1.0, help="edge length a (default 1)")
    p.add_argument("b", type=float, nargs="?", default=1.0, help="edge length b (default 1)")
    p.add_argument("curve_strength", type=float, nargs="?", default=0.5, help="strength of the curve (default 0.5)")
    p.add_argument("--vertex-twist", action="append", default=[], metavar="I=DEG",
                   help="twist both handles of vertex I by DEG degrees (repeatable)")
    p.add_argument("--handle-twist", action="append", default=[], metavar="I.out=DEG|I.in=DEG",
                   help="twist the out or in handle of vertex I by DEG degrees (repeatable)")
    p.add_argument("--handles", action="store_true", help="draw the control handles (red = out, blue = in)")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="save the figure to this file")
    args = p.parse_args(argv)

    if args.tests_only:
        print("spectremodel self tests:")
        return 0 if run_tests() else 1

    tile = Tile(args.a, args.b, args.curve_strength)
    _apply_twists(tile, args.vertex_twist, args.handle_twist, p)

    import matplotlib

    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 6))
    draw(tile, ax, handles=args.handles)
    ax.set_title("%r  area %.4f" % (tile, tile.area()), fontsize=9)
    if args.save:
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
    if not args.no_show:
        plt.show()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
