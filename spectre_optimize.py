"""
spectre_optimize.py - sensitivity and optimisation over the 14 per-edge curve amounts (step 4).

The curved tile has the same area for every choice of its 14 amounts k_i, so the lowest drum
tone lambda_1 and the torsional rigidity J (spectre_fem) are functions of the SHAPE alone.  Two
questions follow: how sensitive are they to each edge, and which pattern of amounts is best?

  * MorphingTile meshes the tile once and then DEFORMS that mesh (boundary nodes follow the
    Bezier edges exactly, interior nodes move harmonically) instead of re-meshing.  The topology
    never changes, so lambda_1(k) and J(k) are smooth functions that optimisers and finite
    differences can use.
  * shape_gradients gives d lambda_1 / d k_i and d J / d k_i from ONE solve with the Hadamard
    formulas (boundary velocity of edge i is w(t) n_i, see spectre_operator):

        d lambda_1 / d k_i = - int_edge i (du/dnu)^2  V_nu ds            (u: unit-norm ground state)
        d J        / d k_i = + int_edge i (dphi/dnu)^2 V_nu ds           (-lap phi = 2)

    with V_nu = w(t) n_i . nu the normal speed.  The normal derivatives come from the residual
    K u - lambda M u at the boundary nodes (the discrete flux).
  * optimize_amounts runs L-BFGS-B (scipy.optimize) with these gradients inside the box |k_i| <= B.

    from spectre_optimize import MorphingTile, shape_gradients, optimize_amounts
    mt = MorphingTile(h=0.05)
    g = shape_gradients(mt, np.full(14, 0.3))   # g["dlambda1"], g["dJ"]  (14 numbers each)
    best = optimize_amounts(mt, "lambda1", "min", bound=0.4)

What the self tests check: the morphed mesh reproduces a freshly built one and keeps the area;
the Hadamard gradients agree with central finite differences of the solver (per edge and in sum);
the optimiser finds the same optimum from different starts and satisfies the KKT conditions;
random search never beats it; the optimum agrees with an independent fine mesh (so it is not a mesh
artefact); and the chirality gap lambda_1(k) - lambda_1(-k) is first order: 2 k . grad lambda_1.

Run ``python3 spectre_optimize.py`` to run the self tests and show the figure (--help for options).
Needs numpy, scipy and matplotlib.
"""

import argparse
import math
import os
import sys

import numpy as np
import scipy.sparse.linalg as spla
from scipy.optimize import minimize

import spectre_fem as fem
import spectre_operator as so

__all__ = [
    "MorphingTile",
    "shape_gradients",
    "optimize_amounts",
    "chirality_gap",
    "run_tests",
    "show_optimization",
]


def _w(t):
    """The odd bump of the Bezier edge, w(t) = 3 t (1 - t) (2t - 1) (spectre_operator)."""
    return 3.0 * t * (1.0 - t) * (2.0 * t - 1.0)


def _check_amounts(k):
    k = np.asarray(k, dtype=float)
    if k.shape != (so.N_EDGES,) or not np.all(np.isfinite(k)):
        raise ValueError("amounts must be %d finite numbers" % so.N_EDGES)
    return k


class _MorphedMesh(fem.TileMesh):
    """A TileMesh whose nodes and triangles are given (no re-meshing)."""

    def __init__(self, points, triangles, n_boundary, h, params):
        self.h = float(h)
        self.points, self.triangles = points, triangles
        self.n_boundary = int(n_boundary)
        self.interior = np.ones(len(points), bool)
        self.interior[: self.n_boundary] = False
        self.params = params
        self._assemble()


class MorphingTile:
    """One triangle mesh of the tile, deformed to any set of per-edge amounts.

    Parameters
    ----------
    a, b : edge lengths.   h : mesh size.
    k_ref : the (scalar) amount the reference mesh is built for; deformations stay good for
        amounts within about 0.4-0.5 of it (the mesh quality is checked).
    min_angle : smallest triangle angle (degrees) accepted by ``mesh``.

    ``mesh(k)`` returns a spectre_fem.TileMesh for the 14 amounts ``k`` (scale 1).
    """

    def __init__(self, a=1.0, b=1.0, h=0.05, k_ref=0.0, min_angle=3.0):
        if not (np.isfinite(h) and h > 0 and np.isfinite(k_ref) and min_angle >= 0):
            raise ValueError("h must be positive, k_ref finite, min_angle non-negative")
        self.a, self.b, self.h, self.k_ref, self.min_angle = float(a), float(b), float(h), float(k_ref), float(min_angle)
        # boundary nodes: fixed Bezier parameters t_j on each edge, equally spaced in arclength
        # on the reference outline
        edges = so.curved_spectre_edges(a, b, k_ref, 1.0, samples=400)
        ts = []
        for e in edges:
            seg = np.hypot(*np.diff(e, axis=0).T)
            s = np.concatenate([[0.0], np.cumsum(seg)])
            n = max(1, int(round(s[-1] / h)))
            ts.append(np.interp(np.linspace(0.0, s[-1], n + 1)[:-1], s, np.linspace(0.0, 1.0, len(s))))
        self.node_edge = np.concatenate([np.full(len(t), i) for i, t in enumerate(ts)])
        self.node_t = np.concatenate(ts)
        self.n_boundary = len(self.node_t)
        self.reference = fem.TileMesh(self.boundary(np.full(so.N_EDGES, k_ref)), h)
        # harmonic extension of boundary displacements into the interior
        K = self.reference.K.tocsc()
        inside = self.reference.interior
        self._lu = spla.splu(K[inside][:, inside].tocsc())
        self._Kib = K[inside][:, ~inside].tocsr()
        self._ref_points = self.reference.points

    def boundary(self, k):
        """Boundary node positions for the amounts ``k``: shape (n_boundary, 2)."""
        k = _check_amounts(k)
        ctrl = so.edge_beziers(self.a, self.b, k, 1.0)[self.node_edge]  # (n, 4, 2)
        t = self.node_t
        u = 1.0 - t
        basis = np.stack([u**3, 3 * u**2 * t, 3 * u * t**2, t**3], axis=1)
        return np.einsum("mj,mjd->md", basis, ctrl)

    def mesh(self, k):
        k = _check_amounts(k)
        bd = self.boundary(k)
        disp = bd - self._ref_points[: self.n_boundary]
        inside = self.reference.interior
        rhs = -(self._Kib @ disp)
        moved = np.column_stack([self._lu.solve(rhs[:, c]) for c in range(2)])
        pts = self._ref_points.copy()
        pts[: self.n_boundary] = bd
        pts[inside] += moved
        m = _MorphedMesh(pts, self.reference.triangles, self.n_boundary, self.h,
                         dict(a=self.a, b=self.b, curves=k.copy(), scale=1.0))
        if (m.triangle_area <= 0).any() or m.min_angle_degrees() < self.min_angle:
            raise ValueError(
                "mesh too distorted for these amounts (min angle %.1f deg): build the MorphingTile "
                "with k_ref nearer to them" % (m.min_angle_degrees() if (m.triangle_area > 0).all() else 0.0)
            )
        return m


def shape_gradients(morph, k):
    """Values and Hadamard gradients at the amounts ``k``.

    Returns a dict with lambda1, J (floats) and dlambda1, dJ (arrays of 14: the derivative with
    respect to each edge's amount, all other amounts fixed)."""
    k = _check_amounts(k)
    m = morph.mesh(k)
    n = morph.n_boundary
    bd = m.points[:n]
    nxt, prv = np.roll(bd, -1, axis=0), np.roll(bd, 1, axis=0)
    ell = 0.5 * (np.hypot(*(nxt - bd).T) + np.hypot(*(bd - prv).T))  # lumped boundary length
    tangent = nxt - prv
    tangent /= np.hypot(*tangent.T)[:, None]
    signed_area = 0.5 * np.sum(bd[:, 0] * nxt[:, 1] - nxt[:, 0] * bd[:, 1])
    nu = np.column_stack([tangent[:, 1], -tangent[:, 0]]) if signed_area > 0 else np.column_stack([-tangent[:, 1], tangent[:, 0]])
    d = so.edge_vectors(morph.a, morph.b)
    n_edge = np.column_stack([d[:, 1], -d[:, 0]])
    velocity = _w(morph.node_t)[:, None] * n_edge[morph.node_edge]  # d(node)/d(k_i) for its own edge
    normal_speed = (velocity * nu).sum(axis=1)

    lam, modes = m.eigenmodes(1)
    u = modes[:, 0]
    flux = (m.K @ u - lam[0] * (m.M @ u))[:n] / ell  # = du/dnu (up to sign) at the boundary nodes
    dlam = -np.bincount(morph.node_edge, weights=flux**2 * normal_speed * ell, minlength=so.N_EDGES)

    tor = m.torsion()
    load = 2.0 * (m.M @ np.ones(len(m.points)))
    flux = (m.K @ tor["phi"] - load)[:n] / ell
    dJ = np.bincount(morph.node_edge, weights=flux**2 * normal_speed * ell, minlength=so.N_EDGES)
    return dict(lambda1=float(lam[0]), J=float(tor["J"]), dlambda1=dlam, dJ=dJ)


def optimize_amounts(morph, objective="lambda1", sense="min", bound=0.4, starts=None, n_random=3,
                     seed=0, maxiter=200):
    """Best pattern of the 14 amounts inside the box |k_i| <= bound.

    objective : "lambda1" (lowest drum tone) or "J" (torsional rigidity);  sense : "min" or "max".
    starts : optional list of start vectors; by default the straight tile plus ``n_random``
        random points in the box.  Uses L-BFGS-B with the Hadamard gradients.

    Returns a dict: k (best amounts), value, values (final value of every start), nit, simple
    (the outline does not cross itself), min_angle (mesh quality at the optimum), objective, sense,
    bound."""
    if objective not in ("lambda1", "J") or sense not in ("min", "max"):
        raise ValueError("objective must be 'lambda1' or 'J' and sense 'min' or 'max'")
    if not (np.isfinite(bound) and bound > 0):
        raise ValueError("bound must be positive")
    sign = 1.0 if sense == "min" else -1.0
    key, gkey = ("lambda1", "dlambda1") if objective == "lambda1" else ("J", "dJ")

    def fun(k):
        g = shape_gradients(morph, k)
        return sign * g[key], sign * g[gkey]

    if starts is None:
        rng = np.random.default_rng(seed)
        starts = [np.full(so.N_EDGES, 1e-3)] + [rng.uniform(-bound, bound, so.N_EDGES) for _ in range(int(n_random))]
    best, values = None, []
    for x0 in starts:
        res = minimize(fun, _check_amounts(x0), jac=True, method="L-BFGS-B",
                       bounds=[(-bound, bound)] * so.N_EDGES, options=dict(maxiter=maxiter))
        values.append(sign * float(res.fun))
        if best is None or res.fun < best.fun:
            best = res
    k = np.clip(best.x, -bound, bound)
    m = morph.mesh(k)
    simple = bool(so.is_simple(*so.curved_spectre(morph.a, morph.b, k, 1.0, samples=40)))
    return dict(k=k, value=sign * float(best.fun), values=values, nit=int(best.nit), simple=simple,
                min_angle=m.min_angle_degrees(), objective=objective, sense=sense, bound=float(bound))


def chirality_gap(morph, k):
    """lambda_1 and J for the amounts k and for -k (opposite curvature on every edge), and the
    first-order prediction of the gap  lambda_1(k) - lambda_1(-k) ~ 2 k . grad lambda_1(0)."""
    k = _check_amounts(k)
    plus, minus = shape_gradients(morph, k), shape_gradients(morph, -k)
    zero = shape_gradients(morph, np.zeros(so.N_EDGES))
    return dict(
        lambda1_plus=plus["lambda1"], lambda1_minus=minus["lambda1"],
        J_plus=plus["J"], J_minus=minus["J"],
        lambda1_first_order=float(2.0 * k @ zero["dlambda1"]),
        J_first_order=float(2.0 * k @ zero["dJ"]),
    )


# --------------------------------------------------------------------------
# Self tests
# --------------------------------------------------------------------------
_MT = {}


def _morph(h=0.05):
    """Shared MorphingTile (building it is the slow part of the tests)."""
    if h not in _MT:
        _MT[h] = MorphingTile(h=h, k_ref=0.0)
    return _MT[h]


def _rel(x, ref):
    return abs(x - ref) / abs(ref)


def _fd_gradients(morph, k, dk=0.05):
    fd_l, fd_j = np.zeros(so.N_EDGES), np.zeros(so.N_EDGES)
    for i in range(so.N_EDGES):
        kp, km = k.copy(), k.copy()
        kp[i] += dk
        km[i] -= dk
        mp, mm = morph.mesh(kp), morph.mesh(km)
        fd_l[i] = (mp.eigenmodes(1)[0][0] - mm.eigenmodes(1)[0][0]) / (2 * dk)
        fd_j[i] = (mp.torsion()["J"] - mm.torsion()["J"]) / (2 * dk)
    return fd_l, fd_j


def test_morphed_mesh_is_the_reference_at_k_ref_and_keeps_the_area():
    mt = _morph()
    ref = mt.mesh(np.zeros(so.N_EDGES))
    assert np.allclose(ref.points, mt.reference.points, atol=1e-12)
    A = so.spectre_area()
    rng = np.random.default_rng(1)
    for k in (np.full(so.N_EDGES, 0.3), rng.uniform(-0.4, 0.4, so.N_EDGES)):
        m = mt.mesh(k)
        assert (m.triangle_area > 0).all()
        assert _rel(m.area, A) < 2e-3, m.area  # polygon through the boundary nodes: O(h^2) short of A
        # the boundary nodes lie exactly on the Bezier edges
        n = mt.n_boundary
        assert np.allclose(m.points[:n], mt.boundary(k), atol=1e-14)
        # same physics as a freshly built mesh of the same tile (different, finer mesh)
        fresh = fem.TileMesh.tile(1, 1, k, 1.0, h=0.03)
        assert _rel(m.eigenmodes(1)[0][0], fresh.eigenmodes(1)[0][0]) < 6e-3
        assert _rel(m.torsion()["J"], fresh.torsion()["J"]) < 6e-3


def test_hadamard_gradients_match_finite_differences():
    mt = _morph()
    for k0 in (np.full(so.N_EDGES, 0.3), np.random.default_rng(3).uniform(-0.3, 0.3, so.N_EDGES)):
        g = shape_gradients(mt, k0)
        fd_l, fd_j = _fd_gradients(mt, k0)
        for exact, fd in ((g["dlambda1"], fd_l), (g["dJ"], fd_j)):
            assert np.abs(exact - fd).max() < 0.01 * max(1.0, np.abs(fd).max()) + 0.005, (exact, fd)
            assert np.linalg.norm(exact - fd) < 0.04 * np.linalg.norm(fd)
        assert abs(g["dlambda1"].sum() - fd_l.sum()) < 0.01 and abs(g["dJ"].sum() - fd_j.sum()) < 0.02


def test_gradient_sum_is_the_derivative_along_uniform_curving():
    # d/ds lambda_1(s * 1) = sum_i d lambda_1 / d k_i ; use a central difference of the solver
    mt = _morph()
    s, ds = 0.3, 0.05
    g = shape_gradients(mt, np.full(so.N_EDGES, s))
    fd = {key: (shape_gradients(mt, np.full(so.N_EDGES, s + ds))[key] - shape_gradients(mt, np.full(so.N_EDGES, s - ds))[key]) / (2 * ds)
          for key in ("lambda1", "J")}
    assert _rel(g["dlambda1"].sum(), fd["lambda1"]) < 0.03, (g["dlambda1"].sum(), fd["lambda1"])
    assert _rel(g["dJ"].sum(), fd["J"]) < 0.03, (g["dJ"].sum(), fd["J"])


def test_gradients_do_not_depend_much_on_the_mesh_size():
    k = np.full(so.N_EDGES, 0.3)
    coarse, fine = shape_gradients(_morph(0.05), k), shape_gradients(_morph(0.035), k)
    for key in ("dlambda1", "dJ"):
        assert np.linalg.norm(coarse[key] - fine[key]) < 0.06 * np.linalg.norm(fine[key]), key


def test_chirality_gap_is_first_order():
    # lambda_1(k) - lambda_1(-k) = 2 k . grad lambda_1(0) + O(k^3)
    mt = _morph()
    for amount in (0.2, 0.4):
        gap = chirality_gap(mt, np.full(so.N_EDGES, amount))
        actual = gap["lambda1_plus"] - gap["lambda1_minus"]
        assert abs(actual) > 1e-3
        assert abs(actual - gap["lambda1_first_order"]) < 0.25 * abs(actual), (actual, gap)
        actual_j = gap["J_plus"] - gap["J_minus"]
        assert abs(actual_j - gap["J_first_order"]) < 0.25 * abs(actual_j) + 5e-3, (actual_j, gap)
    # the opposite curving of a pattern is NOT the mirror image of the tile: the gap is nonzero
    rng_k = np.random.default_rng(8).uniform(-0.4, 0.4, so.N_EDGES)
    g = chirality_gap(mt, rng_k)
    assert abs(g["lambda1_plus"] - g["lambda1_minus"]) > 1e-3


def test_optimiser_finds_the_same_optimum_from_every_start_and_satisfies_kkt():
    mt = _morph()
    bound = 0.4
    straight = shape_gradients(mt, np.zeros(so.N_EDGES))
    cases = (("lambda1", "min", straight["lambda1"], -1), ("J", "max", straight["J"], +1))
    for objective, sense, base, direction in cases:
        res = optimize_amounts(mt, objective, sense, bound=bound, n_random=2, seed=5)
        assert direction * (res["value"] - base) > 0.05, (objective, res["value"], base)  # clearly better than straight
        assert max(res["values"]) - min(res["values"]) < 2e-3, res["values"]  # same optimum from every start
        k = res["k"]
        assert np.abs(k).max() <= bound + 1e-12 and res["simple"] and res["min_angle"] > 3.0
        # KKT of the box problem: free components have zero gradient, the rest point out of the box
        g = shape_gradients(mt, k)["dlambda1" if objective == "lambda1" else "dJ"] * (1 if sense == "min" else -1)
        free = np.abs(k) < bound - 1e-6
        assert np.all(np.abs(g[free]) < 2e-3), (objective, g[free])
        assert np.all(g[~free] * np.sign(k[~free]) < 2e-3), (objective, g[~free], k[~free])
        # the optimum does not exploit the distorted mesh: an independent fine mesh agrees
        fine = fem.TileMesh.tile(1, 1, k, 1.0, h=0.03)
        value = fine.eigenmodes(1)[0][0] if objective == "lambda1" else fine.torsion()["J"]
        assert _rel(value, res["value"]) < 6e-3, (objective, value, res["value"])
        # and the bounds hold: the area is fixed, so the optimum can never reach the disc
        A = so.spectre_area()
        if objective == "lambda1":
            assert res["value"] > 1.1 * fem.faber_krahn_bound(A)
        else:
            assert res["value"] < 0.9 * fem.saint_venant_bound(A)


def test_random_search_never_beats_the_optimum():
    mt = _morph()
    rng = np.random.default_rng(11)
    best_l = optimize_amounts(mt, "lambda1", "min", bound=0.4, n_random=1, seed=2)["value"]
    best_j = optimize_amounts(mt, "J", "max", bound=0.4, n_random=1, seed=2)["value"]
    for _ in range(150):
        k = rng.uniform(-0.4, 0.4, so.N_EDGES)
        m = mt.mesh(k)
        assert m.eigenmodes(1)[0][0] > best_l - 2e-3
        assert m.torsion()["J"] < best_j + 2e-3


def test_first_order_theory_predicts_the_optimal_signs():
    # at the straight tile lambda_1 ~ lambda_1(0) + g . k, so the best small pattern is k_i = -B sign(g_i);
    # the optimum found by L-BFGS-B has exactly those signs on the edges that matter
    mt = _morph()
    g = shape_gradients(mt, np.zeros(so.N_EDGES))["dlambda1"]
    res = optimize_amounts(mt, "lambda1", "min", bound=0.4, n_random=0)
    important = np.abs(g) > 0.02
    assert important.sum() >= 6
    assert np.all(np.sign(res["k"][important]) == -np.sign(g[important])), (g, res["k"])
    # first-order prediction brackets reality: the quadratic term raises the optimum above it
    predicted = shape_gradients(mt, np.zeros(so.N_EDGES))["lambda1"] - 0.4 * np.abs(g).sum()
    assert predicted < res["value"]


def test_validation():
    mt = _morph()
    for call in (
        lambda: MorphingTile(h=-0.1),
        lambda: MorphingTile(k_ref=float("nan")),
        lambda: mt.mesh(np.zeros(13)),
        lambda: mt.mesh(np.full(so.N_EDGES, np.nan)),
        lambda: mt.mesh(np.full(so.N_EDGES, 1.5)),  # far too distorted for a straight reference
        lambda: shape_gradients(mt, [0.0] * 3),
        lambda: optimize_amounts(mt, "volume", "min"),
        lambda: optimize_amounts(mt, "J", "sideways"),
        lambda: optimize_amounts(mt, "J", "max", bound=-1.0),
        lambda: chirality_gap(mt, np.zeros(5)),
    ):
        try:
            call()
        except ValueError:
            continue
        raise AssertionError("expected ValueError")


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
            print("  %-72s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------
def show_optimization(a=1.0, b=1.0, bound=0.4, h=0.05, save=None, show=True):
    """Two rows.  Top: sensitivity of lambda_1 and J to each edge at the straight tile (edges
    coloured by the gradient) and a check of the Hadamard gradient against finite differences.
    Bottom: the optimal patterns inside |k_i| <= bound (min lambda_1, max J and the opposite
    extremes), each edge coloured by its amount, with the straight tile dashed."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import TwoSlopeNorm

    mt = MorphingTile(a, b, h=h, k_ref=0.0)
    A = so.spectre_area(a, b)
    zero = np.zeros(so.N_EDGES)
    base = shape_gradients(mt, zero)
    edges0 = so.curved_spectre_edges(a, b, zero, 1.0, samples=2)
    cmap = plt.get_cmap("RdBu_r")

    def draw_edges(ax, k, values, vmax, title, lw=3.0, label_edges=False, dashed=True):
        edges = so.curved_spectre_edges(a, b, k, 1.0, samples=60)
        segs = [e for e in edges]
        lc = LineCollection(segs, colors=cmap(TwoSlopeNorm(0.0, -vmax, vmax)(values)), linewidths=lw, capstyle="round")
        ax.add_collection(lc)
        if dashed:
            outline = np.vstack([edges0[:, 0], edges0[:1, 0]])
            ax.plot(outline[:, 0], outline[:, 1], "k--", lw=0.7, alpha=0.6)
        if label_edges:
            mid = edges[:, edges.shape[1] // 2]
            for i, (x, y) in enumerate(mid):
                ax.annotate(str(i), (x, y), fontsize=7, ha="center", va="center",
                            bbox=dict(boxstyle="round,pad=0.12", fc="white", ec="none", alpha=0.8))
        allp = edges.reshape(-1, 2)
        ax.set_xlim(allp[:, 0].min() - 0.15, allp[:, 0].max() + 0.15)
        ax.set_ylim(allp[:, 1].min() - 0.15, allp[:, 1].max() + 0.15)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=9)

    fig, axes = plt.subplots(2, 4, figsize=(17, 9.6))

    for ax, key, name in ((axes[0, 0], "dlambda1", "lambda_1"), (axes[0, 1], "dJ", "J")):
        v = base[key]
        vmax = np.abs(v).max()
        draw_edges(ax, zero, v, vmax, "d %s / d k_i at the straight tile\n(red: increasing k_i raises it, blue: lowers it)" % name,
                   label_edges=True, dashed=False)

    # Hadamard against finite differences, at a curved tile
    k0 = np.full(so.N_EDGES, 0.3)
    g0 = shape_gradients(mt, k0)
    fd_l, fd_j = _fd_gradients(mt, k0)
    idx = np.arange(so.N_EDGES)
    for ax, exact, fd, name in ((axes[0, 2], g0["dlambda1"], fd_l, "lambda_1"), (axes[0, 3], g0["dJ"], fd_j, "J")):
        ax.bar(idx - 0.2, exact, 0.4, label="Hadamard (one solve)", color="tab:blue")
        ax.bar(idx + 0.2, fd, 0.4, label="finite differences (28 solves)", color="tab:orange")
        ax.axhline(0, color="0.5", lw=0.6)
        ax.set_xticks(idx)
        ax.set_xlabel("edge i")
        ax.set_ylabel("d %s / d k_i" % name)
        ax.set_title("gradient check at k_i = 0.3: max difference %.1e" % np.abs(exact - fd).max(), fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)

    jobs = (("lambda1", "min", "lowest tone"), ("lambda1", "max", "highest tone"),
            ("J", "max", "stiffest"), ("J", "min", "most flexible"))
    for ax, (obj, sense, label) in zip(axes[1], jobs):
        res = optimize_amounts(mt, obj, sense, bound=bound, n_random=3, seed=1)
        straight = base["lambda1"] if obj == "lambda1" else base["J"]
        name = "lambda_1" if obj == "lambda1" else "J"
        mirrored = shape_gradients(mt, -res["k"])["lambda1" if obj == "lambda1" else "J"]
        draw_edges(ax, res["k"], res["k"], bound,
                   "%s %s = %.3f  (%+.1f %% vs straight %.3f)\n-k gives %.3f;  %s"
                   % (sense, name, res["value"], 100 * (res["value"] / straight - 1), straight, mirrored,
                      "outline simple" if res["simple"] else "OUTLINE CROSSES"),
                   label_edges=False)
    sm = plt.cm.ScalarMappable(norm=TwoSlopeNorm(0.0, -bound, bound), cmap=cmap)
    fig.subplots_adjust(left=0.03, right=0.925, top=0.9, bottom=0.04, hspace=0.3)
    cax = fig.add_axes([0.945, 0.1, 0.012, 0.32])
    fig.colorbar(sm, cax=cax, label="edge amount k_i (red +, blue -)")
    fig.suptitle(
        "Per-edge sensitivity and optimal curve patterns of the Spectre monotile (a=%g, b=%g, area %.3f fixed; "
        "box |k_i| <= %g; Faber-Krahn %.3f, Saint-Venant %.3f)"
        % (a, b, A, bound, fem.faber_krahn_bound(A), fem.saint_venant_bound(A)),
        fontsize=11,
    )
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=100)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Sensitivity and optimisation of the 14 per-edge curve amounts of the Spectre monotile: "
        "self tests and a figure."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--bound", type=float, default=0.4, help="box |k_i| <= bound of the optimisation (default 0.4)")
    p.add_argument("--h", type=float, default=0.05, help="mesh size (default 0.05)")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_optimize self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_optimization(args.a, args.b, args.bound, args.h, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
