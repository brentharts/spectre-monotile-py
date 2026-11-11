#!/usr/bin/env python3
"""t3_escape.py - Stage 1b of research/toymodel_v3/PLAN.md: the clock from a hitting probability, on Z^3, on Z^2 and on the Spectre tiling.

Stage 5 of toymodel_v2 found that a coupling linear in the squared jump d^2 gives either 1/r (on a 3D sheet) or an exact clock with a horizon (on a 2D sheet), never both.
The coupling proposed here is

        c^2 = 1 - h(x),     h(x) = P(a random walk of the substrate started at x ever reaches the source),                                (C)

so the readable fraction of the bit is sqrt(h) and the overlap c is the escape amplitude.  h is harmonic off the source and equal to 1 on it, so
c is the static-observer clock factor sqrt(1 - r_s / r) wherever h = r_s / r, and c = 0 (a horizon) on the source.  Whether h = r_s / r depends only on
whether the substrate's walk is transient: on Z^3 it is, on Z^2 and on the planar Spectre tiling it is recurrent and h = 1 everywhere.
(C) is a **Definition**; that distinguishability should be an escape probability is a **Hypothesis**.

Definitions
  D2.1  Source: the vertices within distance R of a centre.  Absorbing sphere: the vertices at distance >= L (the walk is stopped there, so h_L -> h as L -> infinity).
        h_L solves the discrete Dirichlet problem (Laplacian of the graph) with h = 1 on the source and 0 on the sphere.
  D2.2  Continuum comparison: in d = 3, h_L(r) = (1/r - 1/L) / (1/R_eff - 1/L); in d = 2, h_L(r) = ln(L / r) / ln(L / R_eff); R_eff is the source's effective radius,
        fitted once per source.
  D2.3  The Spectre vertex graph: vertices and edges of the straight tiles of spectre_patch.SpectrePatch(level) (unit edges), centred at the vertex nearest the patch centroid.

Exact claims (each a test; ``python3 t3_escape.py --tests-only``)
  E2.1  Z^3 (L = 40, R = 3): with one fitted R_eff = 2.81, h_L follows the continuum form of D2.2 within 0.8% for r = 4..20 along an axis (worst at r = 4, next to the
        source); equivalently r h is flat to 1% once the finite sphere is corrected for.  h = 1 on the source and falls monotonically.  This is the 1/r law of a transient walk.
  E2.2  Z^2 (R = 3): at r = 6 the hitting probability grows with the absorbing radius, 0.61, 0.71, 0.77, 0.81 for L = 20, 40, 80, 160, tracking the logarithmic
        form of D2.2 within 0.03, which tends to 1.  A recurrent walk always reaches the source: no 1/r, and every bit is eventually unreadable ("all horizon").
  E2.3  The Spectre tiling (level 4, R = 3, ring at r = 9): the same as Z^2, h = 0.31, 0.51, 0.63 for L = 15, 30, 60, within 0.03 of the logarithmic form
        (level 5: 0.33, 0.53, 0.64, 0.71 for L = 15 to 120).
        The planar aperiodic substrate behaves as a two-dimensional one for this question.
  E2.4  The coupling (C) on Z^3: the overlap c = sqrt(1 - h) equals sqrt(1 - R_eff / r), the clock factor with r_s = R_eff, to 1% once the finite sphere is corrected for; c = 0 on the source (a horizon at r_s = R_eff); and 1 - c^2 falls as 1/r.  With the space factor of t3_optics (A B = 1) the index
        1 / (1 - h) bends light by 2 r_s / b to first order.  On Z^2 and on the Spectre tiling (C) gives c -> 0 everywhere as L grows.

  E2.5  The Chair44 (the 3D aperiodic monotile of Tsiokos, substitution of chair44_core.py), tile-adjacency graph of the depth-5 supertile (32768 tiles,
        mean degree 7.06, source = tiles with centroid within 3 of the centre): the walk is transient.  With the absorbing sphere at L = 28,
        h (1/r - 1/L)^(-1) = 2.10, 1.96, 2.01, 1.97, 2.01, 2.05 at r = 5, 7, 9, 11, 14, 18: flat to 5%, so h = A / r with A = 2.0, as on Z^3, and rule (C) gives the
        clock and a horizon on it.  But the Chair44 hull is achiral (proved in the Hat and Spectre repository: the substitution commutes with the x <-> y mirror,
        research/chair44_mirror_proof.py), so it has no handedness to fix the sign of the area term of t3_area.py; the Spectre has one (Im Z_k < 0) and is
        recurrent.  No substrate in either repository has both.

What it says: the 2D/3D split found in Stage 5 of v2 is the recurrence/transience split of the substrate's random walk, and the rule (C) turns it into both halves of
open problem 8 at once (1/r and a horizon) on a transient substrate, and into neither on a planar one.  The Chair44 (a 3D aperiodic solid) is the substrate to try next
(Stage 6 of the plan).  Not shown: why (C); the walk's own dynamics; any back-reaction of the bit on the substrate.

    python3 t3_escape.py --tests-only
    python3 t3_escape.py --story
    python3 t3_escape.py --no-show --save figures/t3_escape.png
"""
import math
import os
import sys

import numpy as np
import scipy.sparse as sps
import scipy.sparse.linalg as spl

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

_CACHE = {}


# -- Z^d ----------------------------------------------------------------------------------------------------------------
def hitting_lattice(dim, L, R):
    """(coordinates, distances, h_L) on the ball |x| < L of Z^dim, source |x| <= R (D2.1)."""
    key = ("Z", dim, L, R)
    if key in _CACHE:
        return _CACHE[key]
    n = 2 * L + 1
    idx = np.indices((n,) * dim).reshape(dim, -1).T - L
    r = np.sqrt((idx ** 2).sum(axis=1))
    src, out = r <= R, r >= L
    free = ~src & ~out
    pos = -np.ones(len(r), dtype=int)
    pos[free] = np.arange(free.sum())
    strides = np.array([n ** (dim - 1 - k) for k in range(dim)])
    fi = np.flatnonzero(free)
    m = len(fi)
    rows, cols, vals = [np.arange(m)], [np.arange(m)], [np.full(m, 2.0 * dim)]
    rhs = np.zeros(m)
    for k in range(dim):
        for s in (1, -1):
            nb = fi + s * strides[k]
            ok = pos[nb] >= 0
            rows.append(np.arange(m)[ok])
            cols.append(pos[nb][ok])
            vals.append(-np.ones(ok.sum()))
            rhs += src[nb]
    A = sps.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(m, m))
    sol, info = spl.cg(A, rhs, rtol=1e-13, maxiter=50000)
    if info != 0:
        raise RuntimeError("CG did not converge")
    h = np.zeros(len(r))
    h[src] = 1.0
    h[free] = sol
    _CACHE[key] = (idx, r, h)
    return idx, r, h


def axis_profile(dim, L, R):
    idx, r, h = hitting_lattice(dim, L, R)
    on = (idx[:, 0] > 0) & np.all(idx[:, 1:] == 0, axis=1)
    o = np.argsort(r[on])
    return r[on][o], h[on][o]


def continuum(r, L, R_eff, dim):
    r = np.asarray(r, dtype=float)
    if dim == 3:
        return (1.0 / r - 1.0 / L) / (1.0 / R_eff - 1.0 / L)
    if dim == 2:
        return np.log(L / r) / np.log(L / R_eff)
    raise ValueError("dim must be 2 or 3")


def fit_reff(dim, L, R, rmin, rmax):
    rr, hh = axis_profile(dim, L, R)
    m = (rr >= rmin) & (rr <= rmax)
    if dim == 3:
        # h (1/R - 1/L) = 1/r - 1/L  ->  1/R_eff = 1/L + (1/r - 1/L) / h, averaged
        inv = 1.0 / L + (1.0 / rr[m] - 1.0 / L) / hh[m]
        return float(1.0 / inv.mean())
    inv = np.log(L / rr[m]) / hh[m]
    return float(L / np.exp(inv.mean()))


# -- the Spectre vertex graph ------------------------------------------------------------------------------------------------
def spectre_graph(level=4):
    key = ("S", level)
    if key in _CACHE:
        return _CACHE[key]
    import spectre_patch as sp
    P = sp.SpectrePatch(level, 1.0, 1.0)
    index, pts, edges = {}, [], set()
    for i in range(P.n_tiles):
        ids = []
        for q in P.straight_polygon(i):
            k = (round(float(q[0]), 6), round(float(q[1]), 6))
            if k not in index:
                index[k] = len(pts)
                pts.append((float(q[0]), float(q[1])))
            ids.append(index[k])
        for a, b in zip(ids, ids[1:] + ids[:1]):
            edges.add((min(a, b), max(a, b)))
    X = np.array(pts)
    E = np.array(sorted(edges))
    c = X.mean(axis=0)
    centre = X[np.argmin(np.hypot(*(X - c).T))]
    _CACHE[key] = (X, E, np.hypot(*(X - centre).T))
    return _CACHE[key]


def chair44_graph(depth=5):
    """(tile centroids, edges, distance to the central tile) of the tile-adjacency graph of the depth-th Chair44 supertile (tiles adjacent when they share a unit face)."""
    key = ("C", depth)
    if key in _CACHE:
        return _CACHE[key]
    import chair44_core as C
    cells = C.patch_cells(depth)
    tiles = {}
    for c, ti in cells.items():
        tiles.setdefault(ti, []).append(c)
    ids = sorted(tiles)
    pos = {ti: i for i, ti in enumerate(ids)}
    cent = np.array([np.mean(tiles[ti], axis=0) for ti in ids]) + 0.5
    edges = set()
    for c, ti in cells.items():
        for dd in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            tj = cells.get((c[0] + dd[0], c[1] + dd[1], c[2] + dd[2]))
            if tj is not None and tj != ti:
                a, b = pos[ti], pos[tj]
                edges.add((min(a, b), max(a, b)))
    E = np.array(sorted(edges))
    side = 2 ** depth * 2
    c0 = cent[np.argmin(np.linalg.norm(cent - side / 2.0, axis=1))]
    _CACHE[key] = (cent, E, np.linalg.norm(cent - c0, axis=1))
    return _CACHE[key]


def chair44_rings(depth, R, L, radii):
    X, E, d = chair44_graph(depth)
    h = hitting_graph(X, E, d, R, L)
    return [float(h[np.abs(d - r) < 0.75].mean()) for r in radii]


def hitting_graph(X, E, d, R, L):
    src, out = d <= R, d >= L
    free = ~src & ~out
    W = sps.csr_matrix((np.ones(2 * len(E)), (np.r_[E[:, 0], E[:, 1]], np.r_[E[:, 1], E[:, 0]])), shape=(len(X),) * 2)
    deg = np.asarray(W.sum(axis=1)).ravel()
    Lap = (sps.diags(deg) - W).tocsr()
    A = Lap[free][:, free].tocsc()
    rhs = -(Lap[free][:, src] @ np.ones(src.sum()))
    h = np.zeros(len(X))
    h[src] = 1.0
    h[free] = spl.spsolve(A, rhs)
    return h


def spectre_ring(level, R, L, r_ring):
    X, E, d = spectre_graph(level)
    if L > 0.95 * d.max():
        raise ValueError("L exceeds the patch")
    h = hitting_graph(X, E, d, R, L)
    ring = np.abs(d - r_ring) < 0.5
    return float(h[ring].mean())


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e21_three_dimensions_give_one_over_r():
    L, R = 40, 3.0
    reff = fit_reff(3, L, R, 4.0, 20.0)
    assert math.isclose(reff, 2.81, abs_tol=0.01), reff
    rr, hh = axis_profile(3, L, R)
    m = (rr >= 4) & (rr <= 20)
    assert np.max(np.abs(hh[m] / continuum(rr[m], L, reff, 3) - 1.0)) < 9e-3
    assert np.all(np.diff(hh[rr >= R]) < 0) and np.all(hitting_lattice(3, L, R)[2][hitting_lattice(3, L, R)[1] <= R] == 1.0)
    # r h, corrected for the finite sphere, is flat
    h_inf = hh[m] * (1.0 - reff / L) / (1.0 - rr[m] / L)   # the finite sphere removed
    assert np.max(np.abs(rr[m] * h_inf / reff - 1.0)) < 1e-2


def test_e22_two_dimensions_always_hit():
    vals = []
    for L in (20, 40, 80, 160):
        rr, hh = axis_profile(2, L, 3.0)
        v = float(hh[rr == 6.0][0])
        vals.append(v)
        assert abs(v - math.log(L / 6.0) / math.log(L / 3.0)) < 0.03, (L, v)
    assert np.all(np.diff(vals) > 0) and [round(v, 2) for v in vals] == [0.61, 0.71, 0.77, 0.81]


def test_e23_the_spectre_tiling_is_two_dimensional():
    vals = []
    for L in (15, 30, 60):
        v = spectre_ring(4, 3.0, L, 9.0)
        vals.append(v)
        assert abs(v - math.log(L / 9.0) / math.log(L / 3.0)) < 0.03, (L, v)
    assert np.all(np.diff(vals) > 0) and [round(v, 2) for v in vals] == [0.31, 0.51, 0.63]


def test_e24_the_coupling_gives_the_clock_and_a_horizon_in_three_dimensions():
    L, R = 40, 3.0
    reff = fit_reff(3, L, R, 4.0, 20.0)
    rr, hh = axis_profile(3, L, R)
    m = (rr >= 4) & (rr <= 20)
    h_inf = hh[m] * (1.0 - reff / L) / (1.0 - rr[m] / L)   # the finite sphere removed
    c = np.sqrt(1.0 - h_inf)
    assert np.max(np.abs(c / np.sqrt(1.0 - reff / rr[m]) - 1.0)) < 1e-2
    idx, r, h = hitting_lattice(3, L, R)
    assert np.all(np.sqrt(1.0 - h[r <= R]) == 0.0), "c = 0 on the source: a horizon"
    import t3_optics as to
    n = lambda x: 1.0 / (1.0 - reff / x) if x > reff * (1 + 1e-12) else 1e12 / x  # noqa: E731
    c1, _ = to.coefficients(n, bs=tuple(reff * f for f in (400.0, 800.0, 1600.0, 3200.0)))
    assert math.isclose(c1 / reff, 2.0, abs_tol=2e-3), "with A B = 1: 2 r_s / b"
    # in 2D the overlap goes to 0 everywhere as the walk's room grows
    assert math.sqrt(1.0 - axis_profile(2, 160, 3.0)[1][5]) < math.sqrt(1.0 - axis_profile(2, 20, 3.0)[1][5])


def test_e25_the_chair44_is_transient_but_achiral():
    X, E, d = chair44_graph(5)
    assert len(X) == 32768 and math.isclose(2.0 * len(E) / len(X), 7.06, abs_tol=0.01)
    radii = (5, 7, 9, 11, 14, 18)
    h28 = chair44_rings(5, 3.0, 28, radii)
    A = [v / (1.0 / r - 1.0 / 28.0) for r, v in zip(radii, h28)]
    assert max(A) / min(A) < 1.08 and math.isclose(np.mean(A), 2.0, abs_tol=0.05), A
    h20 = chair44_rings(5, 3.0, 20, (5,))[0]
    assert h20 < h28[0] < 0.42, "converges as L grows (3D), to about A / r = 0.40 at r = 5, not to 1 (2D)"


def test_validation():
    try:
        continuum(5.0, 10, 3.0, 4)
        raise AssertionError("no error")
    except ValueError:
        pass
    try:
        spectre_ring(4, 3.0, 10_000, 9.0)
        raise AssertionError("no error")
    except ValueError:
        pass


def run_tests(verbose=True):
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-80s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    reff = fit_reff(3, 40, 3.0, 4.0, 20.0)
    rr, hh = axis_profile(3, 40, 3.0)
    print("Z^3, R = 3, L = 40: R_eff = %.4f;  r: h, continuum" % reff)
    for x in (4, 6, 10, 15, 20, 30):
        print("  %2d: %.5f %.5f" % (x, hh[rr == x][0], continuum(float(x), 40, reff, 3)))
    print("Z^2, R = 3, r = 6: h for L = 20, 40, 80, 160:", " ".join("%.3f" % axis_profile(2, L, 3.0)[1][axis_profile(2, L, 3.0)[0] == 6.0][0] for L in (20, 40, 80, 160)))
    print("Spectre level 4, R = 3, r = 9: h for L = 15, 30, 60:", " ".join("%.3f" % spectre_ring(4, 3.0, L, 9.0) for L in (15, 30, 60)))
    radii = (5, 7, 9, 11, 14, 18)
    h = chair44_rings(5, 3.0, 28, radii)
    print("Chair44 depth 5 tile graph, R = 3, L = 28: h (1/r - 1/L)^-1 at r =", radii, ":", " ".join("%.3f" % (v / (1 / r - 1 / 28)) for r, v in zip(radii, h)))
    X, E, d = spectre_graph(5)
    print("Spectre level 5 (%d vertices): h at r = 9 for L = 30, 60, 120, 240:" % len(X),
          " ".join("%.3f" % spectre_ring(5, 3.0, L, 9.0) for L in (30, 60, 120, 240)))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    reff = fit_reff(3, 40, 3.0, 4.0, 20.0)
    rr, hh = axis_profile(3, 40, 3.0)
    m = (rr > 3) & (rr < 36)
    hinf = hh[m] * (1 - reff / 40) / (1 - rr[m] / 40)
    ax[0].plot(rr[m], np.sqrt(1 - np.minimum(hinf, 1)), "o", ms=3, label="Z^3: c = sqrt(1 - h)")
    xs = np.linspace(reff, 36, 300)
    ax[0].plot(xs, np.sqrt(1 - reff / xs), "k", lw=1, label="clock sqrt(1 - r_s/r), r_s = R_eff")
    for L in (20, 40, 80, 160):
        r2, h2 = axis_profile(2, L, 3.0)
        mm = (r2 > 3) & (r2 < 36)
        ax[0].plot(r2[mm], np.sqrt(1 - h2[mm]), "-", lw=0.8, label="Z^2, L = %d" % L)
    ax[0].set_xlabel("r")
    ax[0].set_ylabel("overlap c (readability of the bit: sqrt(1 - c^2))")
    ax[0].set_title("rule C: clock and horizon on Z^3, none on Z^2")
    ax[0].legend(fontsize=7)
    Ls = (15, 30, 60, 120)
    ax[1].plot(Ls, [spectre_ring(5, 3.0, L, 9.0) for L in Ls], "o-", label="Spectre tiling, r = 9")
    ax[1].plot(Ls, [math.log(L / 9) / math.log(L / 3) for L in Ls], "k--", label="planar log form")
    ax[1].set_xscale("log")
    ax[1].set_xlabel("absorbing radius L")
    ax[1].set_ylabel("probability of reaching the source")
    ax[1].set_title("the tiling is recurrent: h -> 1")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
