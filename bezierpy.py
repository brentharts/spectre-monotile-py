"""
bezierpy.py - a small, pure-Python stand-in for the parts of the `bezier`
package that this project uses.

It needs nothing but the standard library, so it works everywhere (no compiler,
no wheels, no NumPy).  It is API-compatible with the subset of `bezier` used by
spectre.py:

    curve = bezierpy.Curve(nodes, degree=3)
    x, y  = curve.evaluate_multi(s_values)     # rows, each has .tolist()

`nodes` has the same layout as in `bezier`: one *row per coordinate* and one
*column per control point*, i.e. shape (dimension, degree + 1).  Nested lists,
tuples or NumPy arrays are all accepted.

Curves of any degree and dimension are evaluated with de Casteljau's algorithm,
which is numerically stable (only convex combinations of the control points).

Use it as a drop-in replacement:

    try:
        import bezier
    except ImportError:
        import bezierpy as bezier

Differences from `bezier`:
  * `evaluate_multi` returns a list of rows (plain Python floats) instead of a
    NumPy array; each row has a `tolist()` method so code written for NumPy
    keeps working.
  * Only the evaluation API (plus a few helpers) is implemented.
"""

__all__ = ["Curve"]


class _Row(list):
    """A list of floats with a NumPy-like ``tolist()``."""

    def tolist(self):
        return list(self)


class Curve:
    """A Bezier curve defined by its control points.

    Parameters
    ----------
    nodes : sequence of sequences
        Control points, shape (dimension, degree + 1): ``nodes[r][c]`` is
        coordinate ``r`` of control point ``c``.
    degree : int, optional
        Degree of the curve; if given it must equal ``len(nodes[0]) - 1``.
    """

    def __init__(self, nodes, degree=None, *, copy=True, verify=True):
        rows = [[float(v) for v in row] for row in nodes]
        if not rows or not rows[0]:
            raise ValueError("nodes must be a non-empty (dimension, degree + 1) array")
        width = len(rows[0])
        if any(len(row) != width for row in rows):
            raise ValueError("every coordinate row of nodes must have the same length")
        if degree is not None and degree != width - 1:
            raise ValueError(
                "degree %r does not match the %d control points given" % (degree, width)
            )
        self._rows = rows
        self._degree = width - 1

    # ------------------------------------------------------------------
    @classmethod
    def from_nodes(cls, nodes, copy=True, verify=True):
        """Create a curve from nodes, inferring the degree (as in `bezier`)."""
        return cls(nodes, copy=copy, verify=verify)

    @property
    def degree(self):
        return self._degree

    @property
    def dimension(self):
        return len(self._rows)

    @property
    def nodes(self):
        """Control points as a list of coordinate rows (copy)."""
        return [list(row) for row in self._rows]

    # ------------------------------------------------------------------
    def _de_casteljau(self, row, s):
        pts = list(row)
        u = 1.0 - s
        for _ in range(self._degree):
            pts = [u * pts[i] + s * pts[i + 1] for i in range(len(pts) - 1)]
        return pts[0]

    def evaluate(self, s):
        """Point on the curve at parameter ``s``, as a list of coordinates."""
        s = float(s)
        return [self._de_casteljau(row, s) for row in self._rows]

    def evaluate_multi(self, s_vals):
        """Evaluate at many parameters.

        Returns one row per coordinate (``dimension`` rows), each a list with one
        value per parameter - the same layout as `bezier`, so
        ``x, y = curve.evaluate_multi(s)`` works for planar curves.
        """
        s_list = [float(s) for s in s_vals]
        return [
            _Row(self._de_casteljau(row, s) for s in s_list) for row in self._rows
        ]

    def hodograph(self):
        """The derivative curve (degree - 1), scaled as in `bezier`."""
        if self._degree == 0:
            raise ValueError("a point (degree 0) has no hodograph")
        n = self._degree
        rows = [[n * (row[i + 1] - row[i]) for i in range(n)] for row in self._rows]
        return Curve(rows)

    def __repr__(self):
        return "<Curve (degree=%d, dimension=%d)>" % (self._degree, self.dimension)


# ----------------------------------------------------------------------
# Self test:  python3 bezierpy.py
# ----------------------------------------------------------------------
def _bernstein_cubic(nodes, s):
    u = 1.0 - s
    w = (u**3, 3 * u**2 * s, 3 * u * s**2, s**3)
    return [sum(w[i] * row[i] for i in range(4)) for row in nodes]


def _selftest():
    import math

    nodes = [[0.0, 1.0, 2.5, 3.0], [0.0, -2.0, 4.0, 1.0]]
    c = Curve(nodes, degree=3)

    # end points and agreement with the explicit Bernstein form
    assert c.evaluate(0.0) == [0.0, 0.0]
    assert c.evaluate(1.0) == [3.0, 1.0]
    s_vals = [i / 49 for i in range(50)]
    xs, ys = c.evaluate_multi(s_vals)
    for s, x, y in zip(s_vals, xs, ys):
        bx, by = _bernstein_cubic(nodes, s)
        assert math.isclose(x, bx, abs_tol=1e-12) and math.isclose(y, by, abs_tol=1e-12)
    assert xs.tolist() == list(xs) and isinstance(xs.tolist()[0], float)

    # other degrees: a line, a quadratic, and a 3-D curve
    assert Curve([[0, 4], [0, 2]]).evaluate(0.25) == [1.0, 0.5]
    q = Curve([[0, 1, 2], [0, 2, 0]])  # parabola, midpoint (1, 1)
    assert q.degree == 2 and q.evaluate(0.5) == [1.0, 1.0]
    assert Curve([[0, 1], [0, 1], [0, 1]]).evaluate(0.5) == [0.5, 0.5, 0.5]

    # hodograph = derivative: compare with a central finite difference
    d = c.hodograph()
    h = 1e-6
    for s in (0.1, 0.5, 0.9):
        fd = [
            (a - b) / (2 * h)
            for a, b in zip(c.evaluate(s + h), c.evaluate(s - h))
        ]
        assert all(math.isclose(p, q_, rel_tol=1e-6, abs_tol=1e-6) for p, q_ in zip(d.evaluate(s), fd))

    # bad input is rejected
    for bad in ([[0, 1], [0]], [], [[]]):
        try:
            Curve(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for %r" % (bad,))
    try:
        Curve(nodes, degree=2)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for wrong degree")

    # optional: compare with the real `bezier` package if it is installed
    try:
        import bezier  # noqa: F401
        import numpy as np

        real = bezier.Curve(np.asfortranarray(nodes), degree=3)
        rx, ry = real.evaluate_multi(np.asarray(s_vals))
        assert max(abs(a - b) for a, b in zip(rx, xs)) < 1e-12
        assert max(abs(a - b) for a, b in zip(ry, ys)) < 1e-12
        print("bezierpy self test passed (also matches the real `bezier` package)")
    except ImportError:
        print("bezierpy self test passed (`bezier` not installed, so no cross-check)")


if __name__ == "__main__":
    _selftest()
