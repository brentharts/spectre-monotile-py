"""
twist_hopping.py - model 8: hopping that follows the length of a curved, twisted edge.

In a tight-binding model on the Spectre family the only thing that makes Hat, Spectre and Turtle differ is the length
of a bond (they share one graph).  Here a bond is a Bezier edge of ``spectremodel.Tile``, so its length is the arc length
of the cubic, and the arc length changes with the curve strength k and with the twist of the two handles that shape it.

    from twist_hopping import arclength, glued_tile, patch_model, tile_ring

    tile = glued_tile(alpha=0.3, beta=-0.2, k=0.5)     # a tile whose twists tile the plane (see below)
    g, H, info = patch_model(2, tile, hopping="inverse")
    w, v = _tb.spectrum(H)

What is exact (every item is a test):

  * The bond graph does not change when anything is twisted, so with *unit* hopping the spectrum does not depend on
    k or on any twist.  Twist can only act through the length.
  * Twists that tile.  A patch built from copies of one tile needs, for every pair of meeting edges (A, i) and (B, j),
    that the two curves coincide: out(i) = in(j) and in(i) = out(j) in total handle twist (the gluing rule of
    models/README.md, fact 4).  On the real patch these conditions leave exactly TWO free numbers: alpha, shared by
    out(e) and in(e - 1) at every EVEN vertex e, and beta, the same at every ODD vertex.  So the twists that tile are
    exactly vertex twists, alpha at the even vertices and beta at the odd ones, and no corner angle changes (both handles
    at a vertex turn together).  ``glued_tile`` builds that tile; the test compares the placed control points of every
    meeting pair (levels 1 to 3, random alpha, beta, k) to 1e-12.  Any other twist leaves gaps and overlaps.
  * Reversing an edge swaps out and in with the same k, and arc length does not change under reversal, so
    length(k, t_out, t_in) = length(k, t_in, t_out).  Hence on a patch that tiles ALL 14 edges have one length
    L(alpha, beta), and with t = t0 / length the Hamiltonian is H_unit / L: the spectrum is the unit-hopping spectrum
    divided by L (tested to 1e-13).  Twisting a tiling in the only way it can be twisted gives a rescaling, nothing else.
    Since L >= 1 (the chord) every hopping is <= 1, so by Perron-Frobenius the largest eigenvalue never exceeds the
    unit-hopping one.  Shape changes of the spectrum need a tile that does not tile (see ``tile_ring``, one tile) or a rule
    that sees more than the edge length (a corner angle, say), which is the next question.
  * Arc length of a cubic is the integral of |B'|, evaluated here by adaptive quadrature split at the slowest
    point (it is not a closed form; the test compares it to a 512-panel Gauss quadrature and to the exact length |d| of the
    k = 0 edge).

What is numeric: the size of L(alpha, beta), and spectra of a single twisted tile.  Where the
hopping rule comes from is a choice (``t = t0 / length``, ``'gauss'`` also available); the model does not say that a
real material behaves like this.  Only a = b = 1 is built (the Mystic tiles of a != b would need their own twists).

    python3 models/twist_hopping.py                       tests, then two figures
    python3 models/twist_hopping.py --tests-only
    python3 models/twist_hopping.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np
from scipy.integrate import quad

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.realpath(os.path.join(_HERE, ".."))
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import _tb  # noqa: E402
from spectremodel import N_EDGES, Tile  # noqa: E402

_GL_NODES, _GL_WEIGHTS = np.polynomial.legendre.leggauss(40)       # used by the test's reference quadrature


# ------------------------------------------------------------------------------------------------ arc length

def bezier_length(ctrl):
    """Arc length of the cubic Bezier with control points ``ctrl`` (4, 2): integral of |B'(t)| over [0, 1].

    |B'|^2 is a quartic in t; the interval is split at the real stationary points of that quartic (where the speed is
    smallest, so where a strongly twisted edge nearly has a cusp) and each piece is integrated by adaptive quadrature (scipy.integrate.quad, tolerance 1e-13). A fixed Gauss rule was
    tried first and was 7e-8 off for an edge whose speed drops to 1e-3.
    """
    ctrl = np.asarray(ctrl, float)
    d = 3.0 * (ctrl[1:] - ctrl[:-1])                         # B'(t) = d0 (1-t)^2 + 2 d1 (1-t) t + d2 t^2
    c0 = d[0]
    c1 = 2.0 * (d[1] - d[0])
    c2 = d[0] - 2.0 * d[1] + d[2]                            # B'(t) = c0 + c1 t + c2 t^2
    q = np.polynomial.Polynomial([c0 @ c0, 2 * c0 @ c1, c1 @ c1 + 2 * c0 @ c2, 2 * c1 @ c2, c2 @ c2])
    cuts = [r.real for r in q.deriv().roots() if abs(r.imag) < 1e-12 and 1e-9 < r.real < 1 - 1e-9]
    edges = [0.0] + sorted(cuts) + [1.0]
    total = 0.0
    for a, b in zip(edges[:-1], edges[1:]):
        total += quad(lambda t: math.sqrt(float(np.dot(c0 + c1 * t + c2 * t * t, c0 + c1 * t + c2 * t * t))),
                      a, b, epsabs=1e-13, epsrel=1e-13, limit=200)[0]
    return total


def edge_lengths(tile):
    """Arc length of each of the 14 edges of the tile, shape (14,)."""
    return np.array([bezier_length(tile.edge_control_points(e)) for e in range(N_EDGES)])


def arclength(k, t_out, t_in, chord=1.0):
    """Arc length of one edge of chord length ``chord``, strength k, total twists t_out (start handle) and t_in (end
    handle), without building a tile (the edge is rotated so that the chord lies along x)."""
    d = np.array([chord, 0.0])
    n = np.array([d[1], -d[0]])

    def rot(v, a):
        c, s = math.cos(a), math.sin(a)
        return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])

    p = np.array([[0.0, 0.0], rot(d / 2 - k * n, t_out), d + rot(-d / 2 + k * n, t_in), d])
    return bezier_length(p)


# ------------------------------------------------------------------------------------------------ twists that tile

def glued_twists(alpha, beta):
    """Total handle twists (out[e], in[e]), each of shape (14,), of the family that tiles the plane."""
    out = np.array([alpha if e % 2 == 0 else beta for e in range(N_EDGES)])
    inn = np.array([beta if e % 2 == 0 else alpha for e in range(N_EDGES)])
    return out, inn


def set_edge_twists(tile, out, inn):
    """Give the handles of ``tile`` these total twists: out[e] to the out handle of vertex e, in[e] to the in handle of
    vertex e + 1 (the two handles that shape edge e).  Vertex twists are set to zero."""
    for v in tile.vertices:
        v.twist = 0.0
    for e in range(N_EDGES):
        tile.vertices[e].out_handle.twist = float(out[e])
        tile.vertices[(e + 1) % N_EDGES].in_handle.twist = float(inn[e])
    return tile


def glued_tile(alpha=0.0, beta=0.0, k=0.5):
    """The tile (a = b = 1, curve strength k) with the two-parameter family of twists that tiles the plane."""
    return set_edge_twists(Tile(1.0, 1.0, k), *glued_twists(alpha, beta))


def gluing_classes(patch):
    """Classes of the 28 effective twists (('o', e) out handle of edge e, ('i', e) in handle of edge e) that the meeting
    edges of ``patch`` force to be equal: out(i) = in(j) and in(i) = out(j) for every meeting pair (i, j)."""
    par = {}

    def find(x):
        par.setdefault(x, x)
        while par[x] != x:
            par[x] = par[par[x]]
            x = par[x]
        return x

    for e in range(N_EDGES):
        find(("o", e))
        find(("i", e))
    for _, i, _, j in patch.edge_pairs():
        i, j = int(i), int(j)
        par[find(("o", i))] = find(("i", j))
        par[find(("i", i))] = find(("o", j))
    groups = {}
    for x in list(par):
        groups.setdefault(find(x), []).append(x)
    return sorted(sorted(g) for g in groups.values())


def meeting_mismatch(patch, tile):
    """Largest distance between the control points of the two curves of every meeting pair (0 when the tile tiles)."""
    worst = 0.0
    ctrl = tile.control_points()                              # (14, 4, 2) in the tile's own frame
    homo = np.concatenate([ctrl, np.ones(ctrl.shape[:2] + (1,))], axis=2)
    for a, i, b, j in patch.edge_pairs():
        ca = homo[i] @ patch.placements[a].T
        cb = homo[j] @ patch.placements[b].T
        worst = max(worst, float(np.abs(ca[:, :2] - cb[::-1, :2]).max()))
    return worst


# ------------------------------------------------------------------------------------------------ the models

def patch_model(levels, tile, hopping="inverse", t0=1.0, kappa=1.0, onsite=None):
    """(graph, H, info) for the level-n patch whose tiles are all copies of ``tile`` (a = b = 1).

    Bond length = arc length of the tile edge that owns the bond; ``info['mismatch']`` is the largest difference between
    the two owners of a shared bond (0 for a tile that tiles)."""
    if abs(tile.a - 1.0) > 1e-12 or abs(tile.b - 1.0) > 1e-12:
        raise ValueError("only a = b = 1 is built")
    g = _tb.patch_graph(levels, 1.0, 1.0)
    L = edge_lengths(tile)
    owner = {}
    for f in g.faces:
        for e in range(N_EDGES):
            u, v = f[e], f[(e + 1) % N_EDGES]
            if u != v:
                owner.setdefault((min(u, v), max(u, v)), []).append(e)
    length = np.array([L[owner[(int(u), int(v))][0]] for u, v in g.bonds])
    other = np.array([L[owner[(int(u), int(v))][-1]] for u, v in g.bonds])
    H = _tb.tight_binding(g, values=_tb.hopping_values(length, hopping, t0, kappa), onsite=onsite)
    return g, H, dict(length=length, mismatch=float(abs(length - other).max()), edge_length=L)


def tile_ring(tile, hopping="inverse", t0=1.0, kappa=1.0):
    """(graph, H) of one tile: its 14 vertices on a ring, bond e = edge e with its arc length (twists included)."""
    pts = np.array([v.position for v in tile.vertices])
    bonds = [(e, e + 1) for e in range(N_EDGES - 1)] + [(0, N_EDGES - 1)]
    g = _tb.Graph(pts, bonds)
    # bond k of ``bonds`` is edge k for k < 13 and edge 13 (vertex 13 -> 0) for the last one
    L = edge_lengths(tile)
    return g, _tb.tight_binding(g, values=_tb.hopping_values(L, hopping, t0, kappa))


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    rng = np.random.default_rng(7)
    from spectre_patch import SpectrePatch
    import spectre_operator as so

    # arc length
    t0 = Tile(1, 1, 0.0)
    check("k = 0: every edge has the length of its chord", abs(edge_lengths(t0) - 1.0).max() < 1e-12)
    worst = 0.0
    for _ in range(6):
        tile = Tile(1.2, 0.8, 0.45)
        for v in tile.vertices:
            v.twist = rng.uniform(-3, 3)
            v.out_handle.twist = rng.uniform(-3, 3)
            v.in_handle.twist = rng.uniform(-3, 3)
        for e in range(N_EDGES):
            P = tile.edge_control_points(e)

            worst = max(worst, abs(bezier_length(P) - _reference_length(P)))
    check("arc length agrees with a 512-panel quadrature on random strongly twisted edges (near-cusp cases included)", worst < 1e-9, "(%.1e)" % worst)
    check("arc length >= chord, and = chord only for k = 0", all(
        arclength(k, a, b) >= 1.0 - 1e-12 for k in (0.1, 0.5) for a in (-2, 0, 1.3) for b in (-1, 0.4, 2.5)))
    tile = Tile(1, 1, 0.5)
    for v in tile.vertices:
        v.out_handle.twist = rng.uniform(-3, 3)
        v.in_handle.twist = rng.uniform(-3, 3)
    mine = np.array([arclength(0.5, tile.edge_handles(e)[0].total_twist, tile.edge_handles(e)[1].total_twist) for e in range(N_EDGES)])
    check("arclength(k, t_out, t_in) equals the Tile edge length (a = b = 1)", abs(mine - edge_lengths(tile)).max() < 1e-12)
    # total twist only (fact 1)
    a, b = Tile(1, 1, 0.5), Tile(1, 1, 0.5)
    a.vertices[3].twist = 0.7
    b.vertices[3].out_handle.twist = 0.7
    b.vertices[3].in_handle.twist = 0.7
    check("only total handle twists change a length (vertex twist = both handle twists)", abs(edge_lengths(a) - edge_lengths(b)).max() < 1e-13)

    # index correspondence between the tile and the patch
    P1 = SpectrePatch(2)
    V = np.array([v.position for v in Tile(1, 1, 0.5).vertices])
    Vh = np.c_[V, np.ones(14)]
    check("tile vertices are the vertices of spectre_operator, in the same order", abs(V - so.vertices(1, 1)).max() < 1e-12)
    check("every placement of the patch maps tile vertex e to polygon vertex e",
          max(abs((Vh @ P1.placements[i].T)[:, :2] - P1.straight_polygon(i)).max() for i in range(P1.n_tiles)) < 1e-12)

    # gluing classes and the two-parameter family
    expect = [sorted([("i", e) for e in range(1, 14, 2)] + [("o", e) for e in range(0, 14, 2)]),
              sorted([("i", e) for e in range(0, 14, 2)] + [("o", e) for e in range(1, 14, 2)])]
    expect = sorted(expect)
    classes = {lv: gluing_classes(SpectrePatch(lv)) for lv in (1, 2, 3)}
    check("meeting edges force exactly two classes of 14 effective twists (alpha, beta), levels 1 to 3",
          all(c == expect for c in classes.values()))
    worst = 0.0
    for lv in (1, 2, 3):
        Pl = SpectrePatch(lv)
        for _ in range(4):
            worst = max(worst, meeting_mismatch(Pl, glued_tile(rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(0.1, 0.7))))
    check("glued tile: every meeting pair has identical curves (control points), random alpha, beta, k, levels 1-3",
          worst < 1e-12, "(%.1e)" % worst)
    bad = glued_tile(0.4, -0.3, 0.5)
    bad.vertices[4].out_handle.twist += 0.5
    check("negative control: one extra handle twist breaks the gluing", meeting_mismatch(SpectrePatch(2), bad) > 1e-2)
    # alpha = beta = 0 is the plain Spectre with edge curve k
    check("alpha = beta = 0 is the untwisted tile", abs(glued_tile(0, 0, 0.5).control_points() - Tile(1, 1, 0.5).control_points()).max() == 0)

    # unit hopping does not see twists (exact); inverse does
    g0, H0, i0 = patch_model(2, glued_tile(0, 0, 0.5), "unit")
    g1, H1, i1 = patch_model(2, glued_tile(1.1, -0.6, 0.5), "unit")
    check("unit hopping: the Hamiltonian is the same for any twist and k", abs(H0 - H1).max() == 0)
    w_unit = _tb.spectrum(H0)[0]
    gS = _tb.patch_graph(2, 1, 1)
    check("... and equals the plain Spectre patch of _tb", abs(w_unit - _tb.spectrum(_tb.tight_binding(gS))[0]).max() < 1e-12)
    gi, Hi, ii = patch_model(2, glued_tile(0, 0, 0.5), "inverse")
    gj, Hj, ij = patch_model(2, glued_tile(1.1, -0.6, 0.5), "inverse")
    check("glued tile: the two owners of every shared bond give the same length", ii["mismatch"] < 1e-12 and ij["mismatch"] < 1e-12,
          "(%.1e, %.1e)" % (ii["mismatch"], ij["mismatch"]))
    wi, wj = _tb.spectrum(Hi)[0], _tb.spectrum(Hj)[0]
    check("1/length hopping: the spectrum depends on the twist", abs(wi - wj).max() > 1e-2, "(max difference %.3f)" % abs(wi - wj).max())
    check("... and stays symmetric about 0 (the graph is bipartite)", abs(wj + wj[::-1]).max() < 1e-9)
    check("Hamiltonian is symmetric", abs(Hj - Hj.T).max() < 1e-14)
    wp = _tb.spectrum(patch_model(2, glued_tile(1.1 + 2 * math.pi, -0.6 - 2 * math.pi, 0.5), "inverse")[1])[0]
    check("twists are periodic in 2 pi", abs(wp - wj).max() < 1e-10)
    nonglued = glued_tile(0.0, 0.0, 0.5)
    nonglued.vertices[4].out_handle.twist = 0.9
    check("a tile that does not tile shows a bond-length mismatch", patch_model(2, nonglued, "inverse")[2]["mismatch"] > 1e-3)
    check("only a = b = 1 is accepted", _raises(patch_model, 2, Tile(1.0, 1.5, 0.5)))

    # reversal symmetry, uniform lengths, rescaling, vertex-twist form, Perron bound
    worst = max(abs(arclength(k, a_, b_) - arclength(k, b_, a_)) for k in (0.1, 0.5, 0.9) for a_, b_ in rng.uniform(-3.2, 3.2, (40, 2)))
    check("length(k, t_out, t_in) = length(k, t_in, t_out) (edge reversal), 120 random cases", worst < 1e-12, "(%.1e)" % worst)
    Lg = edge_lengths(glued_tile(1.1, -0.6, 0.5))
    check("glued tile: all 14 edges have the same length", np.ptp(Lg) < 1e-12)
    wu = _tb.spectrum(H0)[0]
    check("glued tile: spectrum with t = 1/length is the unit-hopping spectrum divided by L",
          abs(wj - wu / Lg[0]).max() < 1e-12, "(%.1e)" % abs(wj - wu / Lg[0]).max())
    vt = Tile(1, 1, 0.5)
    for i, v in enumerate(vt.vertices):
        v.twist = 1.1 if i % 2 == 0 else -0.6
    check("glued tile = vertex twists alpha (even vertices) and beta (odd vertices), no handle twists",
          abs(vt.control_points() - glued_tile(1.1, -0.6, 0.5).control_points()).max() < 1e-13)
    check("glued tile: no corner angle changes (out and in handle of each vertex carry the same total twist)",
          all(abs(v.out_handle.total_twist - v.in_handle.total_twist) < 1e-15 for v in glued_tile(1.1, -0.6, 0.5).vertices))
    check("Perron bound: the largest eigenvalue with t = 1/length never exceeds the unit-hopping one",
          all(_tb.spectrum(patch_model(2, glued_tile(*rng.uniform(-3.2, 3.2, 2), 0.5), "inverse")[1])[0][-1] <= w_unit[-1] + 1e-12
              for _ in range(6)))
    ng = Tile(1, 1, 0.5)
    ng.vertices[3].out_handle.twist = 1.0
    gn, Hn, inf = patch_model(2, ng, "inverse")
    L0 = edge_lengths(Tile(1, 1, 0.5))[0]
    nchanged = int((abs(inf["length"] - L0) > 1e-9).sum())
    # independent count from the placed polygons: a bond takes the length of the FIRST tile (in order) that has it, so it
    # changes exactly when that tile meets it with its edge 3
    seen, expect_changed = set(), 0
    for t in range(P1.n_tiles):
        poly = np.round(P1.straight_polygon(t), 5) + 0.0
        for e in range(N_EDGES):
            key = frozenset((tuple(poly[e]), tuple(poly[(e + 1) % N_EDGES])))
            if key not in seen:
                seen.add(key)
                expect_changed += (e == 3)
    check("a tile that does not tile: the bonds whose length changes are exactly those first met by an edge 3",
          nchanged == expect_changed and 0 < nchanged < len(gn.bonds) // 2, "(%d of %d bonds)" % (nchanged, len(gn.bonds)))

    # single tile ring
    gr, Hr = tile_ring(Tile(1, 1, 0.5), "unit")
    wr = np.sort(_tb.spectrum(Hr)[0])
    check("one tile, unit hopping: 14-ring eigenvalues 2 cos(2 pi m / 14)", abs(wr - np.sort(2 * np.cos(2 * np.pi * np.arange(14) / 14))).max() < 1e-12)
    ring_tile = Tile(1, 1, 0.5)
    ring_tile.vertices[3].out_handle.twist = 1.0
    gr2, Hr2 = tile_ring(ring_tile, "inverse")
    L = edge_lengths(ring_tile)
    check("one tile: bond e of the ring has the length of edge e (including the closing edge 13)",
          abs(np.array([Hr2[e, (e + 1) % 14] for e in range(14)]) - 1.0 / L).max() < 1e-14)
    return ok


def _reference_length(P, panels=512):
    """Independent check: 40-point Gauss-Legendre on 512 equal panels (no use of the stationary-point split)."""
    d = 3.0 * (P[1:] - P[:-1])
    t = ((np.arange(panels)[:, None] + 0.5 * (_GL_NODES[None, :] + 1.0)) / panels).ravel()
    u = 1.0 - t
    vel = np.outer(u * u, d[0]) + np.outer(2 * u * t, d[1]) + np.outer(t * t, d[2])
    return float((np.sqrt((vel ** 2).sum(axis=1)) * np.tile(_GL_WEIGHTS, panels)).sum() * 0.5 / panels)


def _raises(fn, *a):
    try:
        fn(*a)
    except ValueError:
        return True
    return False


# ------------------------------------------------------------------------------------------------ figures

def figure_one_tile(save=None, show=True):
    """One tile (does not tile the plane): ring spectrum against the twist of one handle, and the edge that grows."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    th = np.linspace(-math.pi, math.pi, 241)
    spec = np.empty((len(th), N_EDGES))
    Lmax = np.empty(len(th))
    for n, a in enumerate(th):
        tile = Tile(1, 1, 0.5)
        tile.vertices[3].out_handle.twist = a
        spec[n] = np.sort(_tb.spectrum(tile_ring(tile, "inverse")[1])[0])
        Lmax[n] = edge_lengths(tile)[3]
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    for a_, col in ((0.0, "tab:gray"), (1.2, "tab:red"), (-1.2, "tab:blue")):
        tile = Tile(1, 1, 0.5)
        tile.vertices[3].out_handle.twist = a_
        x, y = tile.outline(80)
        ax[0].plot(x, y, color=col, lw=1.6, label="twist %+.1f" % a_)
    ax[0].set_aspect("equal")
    ax[0].legend(fontsize=8)
    ax[0].set_title("one handle twisted (out handle of vertex 3)")
    ax[1].plot(th, Lmax, color="k")
    ax[1].set_xlabel("twist of the handle (rad)")
    ax[1].set_ylabel("length of edge 3")
    ax[1].set_title("only edge 3 changes length")
    for m in range(N_EDGES):
        ax[2].plot(th, spec[:, m], color="tab:purple", lw=1.0)
    ax[2].set_xlabel("twist of the handle (rad)")
    ax[2].set_ylabel("E")
    ax[2].set_title("spectrum of the 14-ring, t = 1/length\n(unit hopping: fixed at 2 cos(2 pi m / 14))")
    fig.tight_layout()
    _finish(fig, save, show, plt)


def figure_patch(level=2, n=20, save=None, show=True):
    """The family that tiles: edge length L(alpha, beta) (one number for all 14 edges), the collapse of the spectra, and
    the reversal symmetry."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    grid = np.linspace(-math.pi, math.pi, n, endpoint=False)
    Lg = np.array([[edge_lengths(glued_tile(al, be, 0.5))[0] for be in grid] for al in grid])
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6), constrained_layout=True)
    h = grid[1] - grid[0]
    lim = [grid[0] - h / 2, grid[-1] + h / 2]
    im = ax[0].imshow(Lg.T, origin="lower", extent=lim + lim, cmap="viridis", aspect="equal")
    ax[0].set_xlabel("alpha: twist of the even vertices (rad)")
    ax[0].set_ylabel("beta: twist of the odd vertices (rad)")
    ax[0].set_title("length of every edge, k = 0.5\n(untwisted: %.3f)" % Lg[n // 2, n // 2])
    fig.colorbar(im, ax=ax[0], shrink=0.85)
    cols = ("tab:gray", "tab:red", "tab:blue")
    wu = _tb.spectrum(patch_model(level, glued_tile(0, 0, 0.5), "unit")[1])[0]
    x, d = _tb.dos(wu, 100, (-2.6, 2.6))
    ax[1].plot(x, d, color="k", lw=4, alpha=0.25, label="unit hopping")
    for (al, be), c in zip(((0.0, 0.0), (1.2, 1.2), (1.2, -1.2)), cols):
        Lc = edge_lengths(glued_tile(al, be, 0.5))[0]
        w = _tb.spectrum(patch_model(level, glued_tile(al, be, 0.5), "inverse")[1])[0]
        x, d = _tb.dos(w * Lc, 100, (-2.6, 2.6))
        ax[1].plot(x, d, color=c, lw=1.3, ls="--", label="alpha %+.1f, beta %+.1f  (L = %.3f)" % (al, be, Lc))
    ax[1].legend(fontsize=8)
    ax[1].set_xlabel("E x L")
    ax[1].set_ylabel("density of states")
    ax[1].set_title("level-%d patch, t = 1/length: E x L collapses\nonto the unit-hopping spectrum" % level)
    rs = np.random.default_rng(1)
    pts = rs.uniform(-math.pi, math.pi, (300, 2))
    ax[2].scatter([arclength(0.5, a, b) for a, b in pts], [arclength(0.5, b, a) for a, b in pts], s=6, color="tab:purple")
    ax[2].plot([1, 1.8], [1, 1.8], color="k", lw=0.8)
    ax[2].set_aspect("equal")
    ax[2].set_xlabel("length(k, t_out = a, t_in = b)")
    ax[2].set_ylabel("length(k, t_out = b, t_in = a)")
    ax[2].set_title("swapping the two handle twists:\nthe length does not change")
    _finish(fig, save, show, plt)
    return grid, Lg


def _finish(fig, save, show, plt):
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=110)
        print("wrote", save)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    if "--skip-tests" not in argv:
        print("tests:")
        if not run_tests():
            print("TESTS FAILED")
            return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    show = "--no-show" not in argv
    figure_one_tile(os.path.join(d, "twist_hopping_tile.png") if d else None, show)
    grid, Lg = figure_patch(save=os.path.join(d, "twist_hopping_patch.png") if d else None, show=show)
    n = len(grid)
    print("edge length over the (alpha, beta) grid, k = 0.5: min %.3f max %.3f (untwisted %.3f)" % (Lg.min(), Lg.max(), Lg[n // 2, n // 2]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
