"""
spectre_fields.py - fields on and around the curved Spectre monotile.

Step 1: signed distance fields.  The signed distance d(p) to the boundary of the
tile is NEGATIVE inside and POSITIVE outside.  Its level sets are the "inner"
(d < 0) and "outer" (d > 0) offset curves, i.e. a one-parameter family that sweeps
through the original shape, much like the sign of a Cast factor in Blender.

    from spectre_fields import DistanceField
    f = DistanceField.tile(a=1, b=1, curves=1.0, scale=0.5)   # same arguments as spectre_operator
    f.sdf                      # grid values (shape (len(f.ys), len(f.xs)))
    f.level_curves(-0.1)       # inner offset curve(s), list of (n, 2) arrays
    f.region_area(+0.2)        # area of {d < 0.2}: the tile dilated by 0.2
    f.inradius, f.incenter     # largest inscribed circle
    segments, radius = f.medial_axis()   # skeleton and its local half-thickness

What the numbers mean (checked by the self tests):
  * d/d(level) of region_area is the length of the level curve (coarea formula),
    so at level 0 the first-order growth of the offset areas is the PERIMETER.
  * The second-order term depends only on the corner turns (convex corners add a
    circular sector, reflex corners lose an overlap):
        A(+r) = A + P r + c_out r^2,      A(-r) = A - P r + c_in r^2,
    see parallel_area_coefficients.  For equal amounts on all edges the corner turns
    are those of the straight polygon, and c_out + c_in = pi + 2 + sqrt(3).
  * The medial axis (points with two nearest boundary points) peaks at the inradius.

Run ``python3 spectre_fields.py`` to execute the self tests and show the figure
(--help for options).  Needs numpy, scipy and matplotlib.
"""

import argparse
import math
import os
import sys

import numpy as np
from scipy.spatial import Voronoi, cKDTree

import spectre_operator as so

__all__ = [
    "grid_level_curves",
    "grid_region_area",
    "grid_level_length",
    "DistanceField",
    "corner_turns",
    "parallel_area_coefficients",
    "predicted_region_area",
    "run_tests",
    "show_distance_fields",
]


# --------------------------------------------------------------------------
# Level sets of a gridded field
# --------------------------------------------------------------------------
def grid_level_curves(xs, ys, z, level):
    """Contour loops of z == level (marching squares): a list of (n, 2) arrays.
    ``z`` has shape (len(ys), len(xs))."""
    import contourpy

    return contourpy.contour_generator(xs, ys, z).lines(level)


def grid_region_area(xs, ys, z, level):
    """Area of {z < level} from the contour loops (accurate to a fraction of a cell).

    The loops run with higher values on their left, so loops around the region are
    clockwise and loops around holes counterclockwise; the signed shoelace sum is
    therefore minus the area (checked on a disc and an annulus in the tests).
    """
    total = 0.0
    for loop in grid_level_curves(xs, ys, z, level):
        if not np.allclose(loop[0], loop[-1]):
            raise ValueError("a contour leaves the grid; increase the padding")
        x, y = loop[:, 0], loop[:, 1]
        total += 0.5 * float(np.sum(x[:-1] * y[1:] - x[1:] * y[:-1]))
    return -total


def grid_level_length(xs, ys, z, level):
    """Total length of the contour z == level."""
    return float(
        sum(np.hypot(*np.diff(loop, axis=0).T).sum() for loop in grid_level_curves(xs, ys, z, level))
    )


# --------------------------------------------------------------------------
# The distance field
# --------------------------------------------------------------------------
class DistanceField:
    """Signed distance to a closed outline on a regular grid (negative inside).

    Parameters
    ----------
    boundary : (N, 2) array
        Densely sampled simple closed outline (the first point need not be repeated).
        Distances are measured to the sample points, so sample finely: the error is
        about spacing^2 / (8 d) at distance d.
    h : grid spacing.
    pad : the grid extends this far beyond the bounding box; level curves further out
        than ``pad`` leave the grid.
    coarse : optional coarser copy of the outline used only for the inside test
        (default: a thinned copy of ``boundary`` with about 1500 points).
    """

    def __init__(self, boundary, h=0.01, pad=0.5, coarse=None):
        from matplotlib.path import Path

        B = np.asarray(boundary, dtype=float)
        if B.ndim != 2 or B.shape[1] != 2 or len(B) < 8:
            raise ValueError("boundary must be an (N, 2) array with N >= 8")
        if h <= 0 or pad < 0:
            raise ValueError("h must be positive and pad non-negative")
        if np.allclose(B[0], B[-1]):
            B = B[:-1]
        self.boundary = B
        self.h = float(h)
        self.pad = float(pad)
        if coarse is None:
            coarse = B[:: max(1, len(B) // 1500)]
        self._path = Path(np.asarray(coarse, dtype=float))
        self.tree = cKDTree(B)

        lo, hi = B.min(axis=0) - pad, B.max(axis=0) + pad
        self.xs = np.arange(lo[0], hi[0] + h / 2, h)
        self.ys = np.arange(lo[1], hi[1] + h / 2, h)
        X, Y = np.meshgrid(self.xs, self.ys)
        self.sdf = self.signed_distance(np.column_stack([X.ravel(), Y.ravel()])).reshape(X.shape)
        self.params = {}
        self._incircle = None

    @classmethod
    def tile(cls, a=1.0, b=1.0, curves=1.0, scale=0.5, h=0.01, pad=0.5, samples=400):
        """Field of the curved tile; (a, b, curves, scale) as in spectre_operator.curved_spectre."""
        x, y = so.curved_spectre(a, b, curves, scale, samples=samples, closed=False)
        xc, yc = so.curved_spectre(a, b, curves, scale, samples=100, closed=False)
        field = cls(np.column_stack([x, y]), h=h, pad=pad, coarse=np.column_stack([xc, yc]))
        field.params = dict(a=a, b=b, curves=curves, scale=scale)
        return field

    # -- evaluation ----------------------------------------------------------
    def signed_distance(self, points):
        """Signed distance at arbitrary points, shape (..., 2) -> (...,)."""
        P = np.asarray(points, dtype=float)
        flat = P.reshape(-1, 2)
        dist, _ = self.tree.query(flat)
        inside = self._path.contains_points(flat)
        return np.where(inside, -dist, dist).reshape(P.shape[:-1])

    @property
    def extent(self):
        return (self.xs[0], self.xs[-1], self.ys[0], self.ys[-1])

    # -- offset curves and areas -----------------------------------------------
    def level_curves(self, level):
        """Offset curve(s) at signed distance ``level`` (negative: inside)."""
        return grid_level_curves(self.xs, self.ys, self.sdf, level)

    def region_area(self, level):
        """Area of {d < level}: the tile eroded (level < 0) or dilated (level > 0)."""
        return grid_region_area(self.xs, self.ys, self.sdf, level)

    def level_length(self, level):
        return grid_level_length(self.xs, self.ys, self.sdf, level)

    # -- inscribed circle ------------------------------------------------------
    def _inscribed(self):
        if self._incircle is None:
            from scipy.optimize import minimize

            i, j = np.unravel_index(np.argmin(self.sdf), self.sdf.shape)
            x0 = np.array([self.xs[j], self.ys[i]])
            step = self.h
            simplex = np.array([x0, x0 + [step, 0.0], x0 + [0.0, step]])
            res = minimize(
                lambda p: float(self.signed_distance(p)),
                x0,
                method="Nelder-Mead",
                options=dict(initial_simplex=simplex, xatol=1e-10, fatol=1e-12, maxiter=2000),
            )
            self._incircle = (res.x, -float(res.fun))
        return self._incircle

    @property
    def incenter(self):
        """Centre of the largest inscribed circle."""
        return self._inscribed()[0]

    @property
    def inradius(self):
        """Radius of the largest inscribed circle."""
        return self._inscribed()[1]

    # -- medial axis -----------------------------------------------------------
    def medial_axis(self, angle_deg=20.0):
        """Medial axis from the Voronoi diagram of the boundary samples.

        Returns ``(segments, radius)``: segments of shape (M, 2, 2) and the distance
        to the boundary (local half-thickness) at the middle of each one.  A Voronoi
        edge is kept if it lies inside and the two boundary points it separates are
        seen from it under an angle of at least ``angle_deg``; this removes the many
        short edges between neighbouring samples.  Corners with an exterior turn
        below ``angle_deg`` lose their branch.
        """
        vor = Voronoi(self.boundary)
        inside = self._path.contains_points(vor.vertices)
        rv = np.asarray(vor.ridge_vertices)
        rp = np.asarray(vor.ridge_points)
        ok = (rv >= 0).all(axis=1)
        rv, rp = rv[ok], rp[ok]
        ok = inside[rv[:, 0]] & inside[rv[:, 1]]
        rv, rp = rv[ok], rp[ok]
        seg = vor.vertices[rv]
        mid = seg.mean(axis=1)
        u = self.boundary[rp[:, 0]] - mid
        w = self.boundary[rp[:, 1]] - mid
        nu, nw = np.linalg.norm(u, axis=1), np.linalg.norm(w, axis=1)
        ang = np.degrees(np.arccos(np.clip((u * w).sum(axis=1) / (nu * nw), -1.0, 1.0)))
        keep = ang >= angle_deg
        return seg[keep], nu[keep]


# --------------------------------------------------------------------------
# Corner terms of the parallel areas
# --------------------------------------------------------------------------
def corner_turns(a=1.0, b=1.0, curves=1.0, scale=0.0):
    """Turn (radians, counterclockwise positive) of the outline at each of the 14
    vertices: the polygon's turn plus arctan(2 k_i) - arctan(2 k_(i-1))."""
    d = so.edge_vectors(a, b)
    prev = np.roll(d, 1, axis=0)
    turn = np.arctan2(prev[:, 0] * d[:, 1] - prev[:, 1] * d[:, 0], (prev * d).sum(axis=1))
    phi = np.arctan(2.0 * so.edge_strengths(curves, scale))
    return turn + phi - np.roll(phi, 1)


def parallel_area_coefficients(a=1.0, b=1.0, curves=1.0, scale=0.0):
    """Second-order coefficients (c_out, c_in) of the offset areas
        A(+r) = A + P r + c_out r^2,    A(-r) = A - P r + c_in r^2   (small r).
    With turns tau_i = |corner turns|, convex corners adding a circular sector and
    reflex corners losing an overlap:
        c_out = sum_convex tau/2 - sum_reflex tan(tau/2)
        c_in  = sum_convex tan(tau/2) - sum_reflex tau/2.
    The smooth S-shaped edges turn by zero in total and do not contribute.  They are
    dimensionless and do not depend on a and b.

    Valid for small r only, below the first focal / medial-axis event.  Neighbouring
    edges with very different amounts make sharp corners and shrink that range: for
    mild random amounts the prediction is within a few percent of the second-order
    term at r = 0.08, for alternating +/- amounts it is already off at that radius."""
    tau = corner_turns(a, b, curves, scale)
    sgn = np.sign(tau.sum())  # -1 for the clockwise tile
    convex, reflex = tau * sgn > 1e-12, tau * sgn < -1e-12
    T = np.abs(tau)
    c_out = (T[convex] / 2).sum() - np.tan(T[reflex] / 2).sum()
    c_in = np.tan(T[convex] / 2).sum() - (T[reflex] / 2).sum()
    return float(c_out), float(c_in)


def predicted_region_area(level, a=1.0, b=1.0, curves=1.0, scale=0.0):
    """Small-|level| prediction of the area of {d < level}."""
    c_out, c_in = parallel_area_coefficients(a, b, curves, scale)
    A, P = so.spectre_area(a, b), so.perimeter(a, b, curves, scale)
    return A + P * level + (c_out if level >= 0 else c_in) * level**2


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectre_fields.py)
# --------------------------------------------------------------------------
def _segment_distance(P, poly):
    """Exact distance from points P (n, 2) to the closed polygon ``poly`` (m, 2)."""
    A, B = poly, np.roll(poly, -1, axis=0)
    D = B - A
    best = np.full(len(P), np.inf)
    for a_, d_ in zip(A, D):
        t = np.clip(((P - a_) @ d_) / (d_ @ d_), 0.0, 1.0)
        best = np.minimum(best, np.hypot(*(P - (a_ + t[:, None] * d_)).T))
    return best


def test_grid_area_helper_on_disc_and_annulus():
    xs = np.arange(-3, 3, 0.02)
    ys = np.arange(-3, 3, 0.02)
    X, Y = np.meshgrid(xs, ys)
    r = np.hypot(X, Y)
    assert math.isclose(grid_region_area(xs, ys, r, 2.0), math.pi * 4.0, rel_tol=1e-4)
    ring = np.abs(r - 1.5)  # {ring < 0.5} is the annulus 1 < r < 2: the inner loop is a hole
    assert math.isclose(grid_region_area(xs, ys, ring, 0.5), math.pi * (4.0 - 1.0), rel_tol=1e-4)
    assert grid_region_area(xs, ys, r, -1.0) == 0.0  # empty region
    assert math.isclose(grid_level_length(xs, ys, r, 2.0), 4 * math.pi, rel_tol=1e-4)


def test_distance_field_of_a_circle():
    n, R = 3000, 1.3
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    f = DistanceField(R * np.column_stack([np.cos(th), np.sin(th)]), h=0.05, pad=0.8)
    X, Y = np.meshgrid(f.xs, f.ys)
    exact = np.hypot(X, Y) - R
    err = np.abs(f.sdf - exact)
    half_spacing = math.pi * R / n
    # distances are to the samples: error <= half the sample spacing on the boundary, about spacing^2/(8 d) away from it
    assert err.max() <= 1.01 * half_spacing
    assert err[np.abs(exact) > 0.02].max() < 1e-4
    assert math.isclose(f.inradius, 1.3, abs_tol=1e-3) and np.allclose(f.incenter, 0.0, atol=2e-2)
    assert math.isclose(f.region_area(0.5), math.pi * 1.8**2, rel_tol=1e-3)  # dilation: bigger circle
    assert math.isclose(f.region_area(-0.5), math.pi * 0.8**2, rel_tol=1e-3)  # erosion: smaller circle


def test_distance_field_matches_exact_polygon_distance():
    f = DistanceField.tile(1.0, 1.0, curves=0.0, scale=0.0, h=0.02, pad=0.3)  # the straight tile
    X, Y = np.meshgrid(f.xs, f.ys)
    P = np.column_stack([X.ravel(), Y.ravel()])
    exact = _segment_distance(P, so.vertices(1.0, 1.0))
    assert np.abs(np.abs(f.sdf.ravel()) - exact).max() < 1.5e-3
    assert np.all(np.sign(f.sdf.ravel())[exact > 1e-2] == np.sign(f.signed_distance(P))[exact > 1e-2])
    inside = f.sdf < 0
    assert abs(inside.sum() * f.h**2 - so.spectre_area(1, 1)) < 0.05  # pixel count, coarse


def test_zero_level_encloses_the_tile_area():
    rng = np.random.default_rng(2)
    for curves, scale in [(1.0, 0.0), (1.0, 0.25), (1.0, 0.5), (rng.uniform(-1, 1, 14), 0.3)]:
        f = DistanceField.tile(1.0, 1.0, curves, scale, h=0.01, pad=0.3)
        assert math.isclose(f.region_area(0.0), so.spectre_area(1, 1), rel_tol=5e-4), scale


def test_first_order_growth_of_offset_areas_is_the_perimeter():
    rng = np.random.default_rng(2)
    for curves, scale in [(1.0, 0.25), (1.0, 0.5), (rng.uniform(-1, 1, 14), 0.3)]:
        f = DistanceField.tile(1.0, 1.0, curves, scale, h=0.01, pad=0.3)
        d = 0.04
        slope = (f.region_area(d) - f.region_area(-d)) / (2 * d)
        assert math.isclose(slope, so.perimeter(1, 1, curves, scale), rel_tol=6e-3), (scale, slope)
        # coarea: the level curve at 0 has the perimeter as its length (marching squares cuts corners)
        assert math.isclose(f.level_length(0.0), so.perimeter(1, 1, curves, scale), rel_tol=6e-3)


def test_second_order_term_is_set_by_the_corner_turns():
    c_out, c_in = parallel_area_coefficients(1.0, 1.0, 1.0, 0.0)
    assert math.isclose(c_out + c_in, math.pi + 2 + math.sqrt(3), rel_tol=1e-12)
    rho = 0.08
    mild = np.random.default_rng(2).uniform(-1, 1, 14)  # per-edge amounts change the corner turns
    for curves, scale in [(1.0, 0.25), (1.0, 0.5), (mild, 0.3)]:
        f = DistanceField.tile(1.0, 1.0, curves, scale, h=0.01, pad=0.3)
        co, ci = parallel_area_coefficients(1.0, 1.0, curves, scale)
        if np.isscalar(curves):  # equal amounts on every edge: corner turns, hence c_out and c_in, are k independent
            assert np.allclose((co, ci), (c_out, c_in))
        else:
            assert not np.allclose((co, ci), (c_out, c_in))  # the prediction really uses the new corners
        for r, c in ((rho, co), (-rho, ci)):
            num, pred = f.region_area(r), predicted_region_area(r, 1.0, 1.0, curves, scale)
            # tolerance: contour/grid floor (~1e-3 in area) plus 5% of the second-order term for higher orders
            assert abs(num - pred) < 1e-3 + 0.05 * abs(c) * rho**2, (scale, r, num - pred)
        if np.isscalar(curves):
            total = f.region_area(rho) + f.region_area(-rho) - 2 * so.spectre_area(1, 1)
            assert math.isclose(total / rho**2, c_out + c_in, rel_tol=0.03), (scale, total / rho**2)


def test_inner_offsets_disappear_at_the_inradius():
    f = DistanceField.tile(1.0, 1.0, 1.0, 0.5, h=0.01, pad=0.3)
    r = f.inradius
    assert 0.9 < r < 1.2
    assert f.region_area(-(r - 0.02)) > 0.0
    assert f.region_area(-(r + 0.02)) == 0.0
    assert abs(float(f.signed_distance(f.incenter)) + r) < 1e-9  # the optimum is a real field value
    assert f.sdf.min() >= -r - 1e-9 and f.sdf.min() < -r + 0.01  # grid min is at most h below


def test_medial_axis_is_consistent_with_the_field():
    f = DistanceField.tile(1.0, 1.0, 1.0, 0.5, h=0.02, pad=0.3)
    seg, rad = f.medial_axis()
    assert len(seg) > 100 and seg.shape[1:] == (2, 2)
    assert abs(rad.max() - f.inradius) < 1.5e-3  # the axis peaks at the inscribed circle
    mid = seg.mean(axis=1)
    assert np.abs(f.signed_distance(mid) + rad).max() < 3e-3  # radius = distance to the boundary
    assert np.all(f.signed_distance(mid) < 0)  # inside the tile
    thin, _ = f.medial_axis(angle_deg=60.0)
    assert len(thin) < len(seg)  # a larger angle keeps fewer edges


def test_constructor_validation():
    for bad in (np.zeros((5, 2)), np.zeros((10, 3)), np.zeros(10)):
        try:
            DistanceField(bad)
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for boundary shape %s" % (bad.shape,))
    th = np.linspace(0, 2 * np.pi, 50, endpoint=False)
    circle = np.column_stack([np.cos(th), np.sin(th)])
    for kw in (dict(h=0.0), dict(h=-0.1), dict(pad=-1.0)):
        try:
            DistanceField(circle, **kw)
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for %r" % (kw,))
    thin = DistanceField(circle, h=0.1, pad=0.1)
    try:
        thin.region_area(0.5)  # the contour leaves the 0.1 padding
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for a level outside the grid")


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
            print("  %-58s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------
def show_distance_fields(a=1.0, b=1.0, k=0.5, seed=3, save=None, show=True):
    """Six panels: the signed distance field, inner space (erosion + medial axis), outer
    space (dilation wavefronts), the offset areas for several k, inradius and perimeter
    against k, and a field for random per-edge amounts."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle

    fld = DistanceField.tile(a, b, 1.0, k, h=0.012, pad=1.15)
    r_in = fld.inradius
    lim = 1.15

    def frame(ax, title, tight=False):
        m = 0.15 if tight else lim
        ax.set_aspect("equal")
        ax.set_xlim(fld.boundary[:, 0].min() - m, fld.boundary[:, 0].max() + m)
        ax.set_ylim(fld.boundary[:, 1].min() - m, fld.boundary[:, 1].max() + m)
        ax.set_title(title, fontsize=9)
        ax.axis("off")

    def tile_fill(ax, f):
        ax.contourf(f.xs, f.ys, f.sdf, levels=[-9.0, 0.0], colors=["0.9"])
        ax.contour(f.xs, f.ys, f.sdf, levels=[0.0], colors="k", linewidths=1.6)

    fig, axes = plt.subplots(2, 3, figsize=(17, 11.5))

    # 1. the signed distance field
    ax = axes[0, 0]
    vm = 1.1
    cf = ax.contourf(fld.xs, fld.ys, fld.sdf, levels=np.linspace(-vm, vm, 45), cmap="RdBu_r", extend="both")
    ax.contour(fld.xs, fld.ys, fld.sdf, levels=np.arange(-1.0, 1.01, 0.1), colors="k", linewidths=0.3, alpha=0.5)
    ax.contour(fld.xs, fld.ys, fld.sdf, levels=[0.0], colors="k", linewidths=2.0)
    fig.colorbar(cf, ax=ax, shrink=0.6, label="signed distance (negative inside)")
    frame(ax, "signed distance field, level lines every 0.1")

    # 2. inner space
    ax = axes[0, 1]
    tile_fill(ax, fld)
    depths = np.arange(0.05, r_in, 0.05)
    cm = plt.get_cmap("Blues")
    for dpt in depths:
        for loop in fld.level_curves(-dpt):
            ax.plot(loop[:, 0], loop[:, 1], color=cm(0.3 + 0.7 * dpt / r_in), lw=0.9)
    seg, rad = fld.medial_axis()
    lc = LineCollection(seg, cmap="magma_r", linewidths=1.5)
    lc.set_array(rad)
    ax.add_collection(lc)
    ax.add_patch(Circle(fld.incenter, r_in, fill=False, ls="--", color="crimson", lw=1.2))
    ax.plot(*fld.incenter, "o", color="crimson", ms=3)
    frame(
        ax,
        "inner space: erosion every 0.05, medial axis (colour = half-thickness),\ninscribed circle r = %.3f" % r_in,
        tight=True,
    )

    # 3. outer space
    ax = axes[0, 2]
    tile_fill(ax, fld)
    cm = plt.get_cmap("Reds")
    for lev in np.arange(0.05, 1.11, 0.05):
        for loop in fld.level_curves(lev):
            ax.plot(loop[:, 0], loop[:, 1], color=cm(0.25 + 0.75 * lev / 1.1), lw=0.9)
    frame(ax, "outer space: dilation wavefronts every 0.05")

    # 4. offset areas for several k
    ax = axes[1, 0]
    A0 = so.spectre_area(a, b)
    for kk, col in zip((0.0, 0.25, k), ("0.35", "tab:green", "tab:purple")):
        f = DistanceField.tile(a, b, 1.0, kk, h=0.02, pad=1.15)
        P = so.perimeter(a, b, 1.0, kk)
        lv = np.concatenate([np.arange(-f.inradius + 0.03, -0.03, 0.02), np.arange(0.03, 1.0 + 1e-9, 0.02)])
        ax.plot(lv, [(f.region_area(x) - A0) / x for x in lv], color=col, lw=1.6, label="k = %g,  P = %.2f" % (kk, P))
        rr = np.concatenate([np.linspace(-0.3, -0.01, 30), np.linspace(0.01, 0.3, 30)])
        ax.plot(rr, [(predicted_region_area(x, a, b, 1.0, kk) - A0) / x for x in rr], "--", color=col, lw=0.9)
        ax.plot(0.0, P, "o", color=col, ms=5)
    ax.plot([], [], "k--", lw=0.9, label="small-r prediction  P + c r")
    ax.axvline(0, color="0.7", lw=0.6)
    ax.set_xlabel("offset level r  (negative: inner)")
    ax.set_ylabel("(A(r) - A) / r")
    ax.set_title("growth rate of the offset area: tends to the perimeter P as r -> 0 (dots)", fontsize=9)
    ax.legend(fontsize=8, loc="upper left")

    # 5. k sweep
    ax = axes[1, 1]
    ks = np.arange(0.0, 0.601, 0.05)
    inr = [DistanceField.tile(a, b, 1.0, kk, h=0.02, pad=0.3).inradius for kk in ks]
    ax.plot(ks, inr, "o-", color="tab:blue", ms=3)
    ax.set_xlabel("curve strength k")
    ax.set_ylabel("inradius", color="tab:blue")
    ax2 = ax.twinx()
    ax2.plot(ks, [so.perimeter(a, b, 1.0, kk) for kk in ks], "s-", color="tab:red", ms=3)
    ax2.set_ylabel("perimeter (area is fixed at %.3f)" % A0, color="tab:red")
    ax.set_title("same area, different shape: inradius and perimeter against k", fontsize=9)

    # 6. random per-edge amounts
    ax = axes[1, 2]
    curves = np.random.default_rng(seed).uniform(-1, 1, so.N_EDGES)
    f6 = DistanceField.tile(a, b, curves, 0.4, h=0.012, pad=1.15)
    simple = so.is_simple(*so.curved_spectre(a, b, curves, 0.4, samples=30))
    cf = ax.contourf(f6.xs, f6.ys, f6.sdf, levels=np.linspace(-vm, vm, 45), cmap="RdBu_r", extend="both")
    ax.contour(f6.xs, f6.ys, f6.sdf, levels=np.arange(-1.0, 1.01, 0.1), colors="k", linewidths=0.3, alpha=0.5)
    ax.contour(f6.xs, f6.ys, f6.sdf, levels=[0.0], colors="k", linewidths=2.0)
    seg6, rad6 = f6.medial_axis()
    lc = LineCollection(seg6, colors="k", linewidths=0.8, alpha=0.8)
    ax.add_collection(lc)
    frame(ax, "random per-edge amounts (seed %d), scale 0.4%s" % (seed, "" if simple else "  (outline crosses itself)"))

    fig.suptitle(
        "Distance fields of the curved Spectre monotile (a=%g, b=%g): the signed distance is the proper "
        "'inner / outer field'; its level sets are the offset curves" % (a, b),
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=110)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Signed distance fields of the curved Spectre monotile: self tests and a figure."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--curve_strength", type=float, default=0.5, help="uniform curve strength k (default 0.5)")
    p.add_argument("--seed", type=int, default=3, help="seed of the random per-edge panel")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_fields self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_distance_fields(args.a, args.b, args.curve_strength, args.seed, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
