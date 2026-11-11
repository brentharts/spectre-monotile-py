#!/usr/bin/env python3
"""t3_chair_metric.py - Stage 6b of research/toymodel_v3/PLAN.md: is the Chair44 tile graph isotropic, as the Spectre's is?

The Spectre tile graph has a graph distance close to 1.50 x the Euclidean one at large r, nearly isotropic (anisotropy 0.06, the Hat and Spectre repository,
research/graph_dilation.py).  Rule (C) of t3_escape.py uses Euclidean radii on the Chair44 graph; this module asks which of its notions of distance are isotropic.

The level-k supertile is itself a chair: 7 of the 8 octants of its bounding cube, the octant (1, 1, 1) missing.  The "central tile" of t3_escape.chair44_graph sits at
the inner corner of that notch.  All measurements here are made in the BULK: around p, the centre of the full octant (0, 0, 0) of the level-6 supertile (a cube of side
64, so every region used is at least 2 cells from any free surface), with each signed direction kept apart.

Definitions
  D6b.1  Tile graph as in t3_escape.chair44_graph: one node per tile (7 unit cells) at its centroid, an edge where two tiles share a unit face.
  D6b.2  Walk tensor, unrelaxed: M = sum over edges of e e^T, e the vector between the two centroids (the step covariance of the simple random walk, up to normalisation).
  D6b.3  Walk tensor, relaxed (homogenised): in a cube of half-width w around p, fix u = x.v on a shell of thickness 3, solve the graph Laplace equation inside, and take
         the Dirichlet energy per tile in the inner half-cube, sigma(v) = v.S.v.  S is the tensor that governs diffusion and long waves of the graph Laplacian at
         long wavelength; M is its value before relaxation.
  D6b.4  Hop count: d_G / r for pairs from 40 sources near p, r = 16 to 26, direction within 10 degrees of one of the 13 lines through the origin of a cube's
         axes, face diagonals and body diagonals (each line separately).

Claims (each a test; ``python3 t3_chair_metric.py --tests-only``, about 40 s)
  E6b.1  [Exact] The diagonal entries of M are equal at every level k = 1..5 (exact rationals); the off-diagonal entries are all -22/49 for k = 2..5, while tr M grows
         with the number of tiles.  So M / tr M -> I / 3.
  E6b.2  [Numeric] The relaxed tensor is isotropic in the bulk: level 6, w = 30, eigenvalues 4.0024, 4.0024, 4.0033 (relative spread 2e-4); relaxation lowers it from
         the unrelaxed 4.184 by 4%, the same in every direction.
         [Negative, a warning] Measured instead around the supertile's centre, at the notch, with the outer faces fixed and the notch's faces free, the same
         procedure gives a tensor uniaxial along (1, 1, 1) with a ratio of about 9 between its eigenvalues (level 5): a boundary artefact, not a property of the tiling.
  E6b.3  [Numeric; Negative for an isotropic hop count] The hop count is cubic, the same on every line of a class: 0.554 on the 3 axes, 0.707 to 0.708 on the 6 face
         diagonals, 0.660 to 0.667 on the 4 body diagonals; spread (max - min) / mean 0.24.  The taxicab metric of Z^3 at the same spacing (exact) gives 0.5, 0.707,
         0.866, spread 0.53; the Spectre's tile graph 0.06 (over 24 angular bins, a coarser measure).  The ball of the hop count is cube-like, rounder than Z^3's.
  E6b.4  [Numeric] The hitting probability of rule (C) is isotropic at large r: source radius 3 around p, absorbing radius 30, the excess of h along the body
         diagonals over h along the axes is 2.2% at r = 6 to 10 and below 0.3% at r = 15 to 20 (a short-range effect of the discrete source, as an isotropic S requires).

What it says
  The Chair44 has two notions of distance that disagree.  Diffusion, and so the clock c^2 = 1 - h of rule (C), and long waves of the graph Laplacian see an isotropic
  medium (S proportional to I to 2e-4).  A signal that hops one tile per tick sees a cubic one: about 25% slower along the axes than along the diagonals, in hops per
  unit length.  So on the Chair44 light has to be a long wave, not a hop count, or the substrate has a preferred frame.  Neither notion supplies a space factor B:
  rule (C) and the optics together give the clock-only half of light bending (E1.2 of t3_optics.py), on the Chair44 as on Z^3.

    python3 t3_chair_metric.py --tests-only
    python3 t3_chair_metric.py --story
"""
import itertools
import os
import sys
from fractions import Fraction

import numpy as np
import scipy.sparse as sps
import scipy.sparse.linalg as spl
from scipy.sparse.csgraph import shortest_path

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t3_escape as ve  # noqa: E402

CLASS_OF = {1: "axis", 2: "face", 3: "body"}


def lines():
    """{integer direction: unit vector} for the 13 lines (3 axes, 6 face diagonals, 4 body diagonals), one representative per line."""
    out = {}
    for v in itertools.product((-1, 0, 1), repeat=3):
        if any(v) and tuple(-x for x in v) not in out:
            out[v] = np.array(v, float) / np.linalg.norm(v)
    return out


def _tiles(depth):
    import chair44_core as C
    cells = C.patch_cells(depth)
    tiles = {}
    for c, ti in cells.items():
        tiles.setdefault(ti, []).append(c)
    return cells, tiles


def walk_tensor(depth):
    """(M as 3x3 Fractions, number of tiles) of D6b.2, centroids exact."""
    cells, tiles = _tiles(depth)
    ids = sorted(tiles)
    pos = {t: i for i, t in enumerate(ids)}
    cen = [tuple(Fraction(sum(c[i] for c in tiles[t]), len(tiles[t])) for i in range(3)) for t in ids]
    edges = set()
    for c, ti in cells.items():
        for dd in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            tj = cells.get((c[0] + dd[0], c[1] + dd[1], c[2] + dd[2]))
            if tj is not None and tj != ti:
                a, b = pos[ti], pos[tj]
                edges.add((min(a, b), max(a, b)))
    M = [[Fraction(0)] * 3 for _ in range(3)]
    for a, b in edges:
        e = [cen[b][i] - cen[a][i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                M[i][j] += e[i] * e[j]
    return M, len(ids)


def bulk_centre(X):
    """Centre of the full octant (0, 0, 0) of the supertile's bounding cube."""
    return (X.max(axis=0) + 0.9) / 4.0


def _laplacian(n, E):
    W = sps.csr_matrix((np.ones(2 * len(E)), (np.r_[E[:, 0], E[:, 1]], np.r_[E[:, 1], E[:, 0]])), shape=(n, n))
    return (sps.diags(np.asarray(W.sum(axis=1)).ravel()) - W).tocsr()


def relaxed_tensor(depth=6, w=30.0, at_notch=False):
    """(S, M_unrelaxed) of D6b.3, both 3x3 floats.  at_notch: the cube is the whole supertile around its centre (the artefact of E6b.2)."""
    X, E, _ = ve.chair44_graph(depth)
    if at_notch:
        p = X.mean(axis=0)
        keep = np.ones(len(X), bool)
        R = np.abs(X - p).max(axis=1)
        w = R.max()
    else:
        p = bulk_centre(X)
        keep = np.abs(X - p).max(axis=1) <= w
    idx = -np.ones(len(X), int)
    idx[keep] = np.arange(keep.sum())
    Y = X[keep] - p
    Ek = idx[E[keep[E[:, 0]] & keep[E[:, 1]]]]
    Lap = _laplacian(len(Y), Ek)
    Rm = np.abs(Y).max(axis=1)
    bd = Rm > w - 3
    fr = ~bd
    lu = spl.splu(Lap[fr][:, fr].tocsc())
    inner = Rm < w / 2
    ein = inner[Ek[:, 0]] & inner[Ek[:, 1]]

    def energy(v):
        u0 = Y @ v
        u = u0.copy()
        u[fr] = lu.solve(-(Lap[fr][:, bd] @ u0[bd]))
        du, du0 = u[Ek[:, 1]] - u[Ek[:, 0]], u0[Ek[:, 1]] - u0[Ek[:, 0]]
        return (du[ein] ** 2).sum() / inner.sum(), (du0[ein] ** 2).sum() / inner.sum()

    S, M = np.zeros((3, 3)), np.zeros((3, 3))
    I3 = np.eye(3)
    for i in range(3):
        S[i, i], M[i, i] = energy(I3[i])
    for i, j in ((0, 1), (0, 2), (1, 2)):
        e, e0 = energy(I3[i] + I3[j])
        S[i, j] = S[j, i] = (e - S[i, i] - S[j, j]) / 2
        M[i, j] = M[j, i] = (e0 - M[i, i] - M[j, j]) / 2
    return S, M


def hop_metric(depth=6, nsrc=40, rmin=16.0, rmax=26.0, seed=3):
    """{line: mean d_G / r} of D6b.4."""
    X, E, _ = ve.chair44_graph(depth)
    p = bulk_centre(X)
    n = len(X)
    A = sps.csr_matrix((np.ones(2 * len(E)), (np.r_[E[:, 0], E[:, 1]], np.r_[E[:, 1], E[:, 0]])), shape=(n, n))
    cand = np.nonzero(np.abs(X - p).max(axis=1) < 4)[0]
    src = np.random.default_rng(seed).choice(cand, nsrc, replace=False)
    D = shortest_path(A, unweighted=True, indices=src)
    L = lines()
    acc = {k: [] for k in L}
    cos = np.cos(np.radians(10.0))
    for s, d in zip(src, D):
        dx = X - X[s]
        r = np.linalg.norm(dx, axis=1)
        ok = (r >= rmin) & (r < rmax) & (np.abs(X - p).max(axis=1) < 31)
        u = dx[ok] / r[ok, None]
        q = d[ok] / r[ok]
        for k, v in L.items():
            acc[k].append(q[np.abs(u @ v) > cos])
    return {k: float(np.concatenate(a).mean()) for k, a in acc.items()}


def by_class(m):
    """{axis/face/body: (min, max)} over the lines of each class."""
    out = {}
    for k, v in m.items():
        c = CLASS_OF[int(np.count_nonzero(k))]
        lo, hi = out.get(c, (np.inf, -np.inf))
        out[c] = (min(lo, v), max(hi, v))
    return out


def taxicab_metric(spacing=2.0):
    """d_G / r of Z^3 with the given spacing (exact, large r), by class."""
    return {"axis": 1 / spacing, "face": 2 / (spacing * np.sqrt(2)), "body": 3 / (spacing * np.sqrt(3))}


def spread(values):
    v = np.array(list(values))
    return float((v.max() - v.min()) / v.mean())


def body_excess(depth=6, R=3.0, L=30.0, bands=((6, 10), (10, 15), (15, 20))):
    """[(band, h along body diagonals / h along axes - 1)] of E6b.4."""
    X, E, _ = ve.chair44_graph(depth)
    p = bulk_centre(X)
    d = np.linalg.norm(X - p, axis=1)
    h = ve.hitting_graph(X, E, d, R, L)
    dx = X - p
    ax = np.eye(3)
    bo = np.array([[1, 1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]]) / np.sqrt(3)
    cos = np.cos(np.radians(12.0))

    def mean(V, lo, hi):
        c = np.max(np.abs(dx @ V.T), axis=1) / np.maximum(d, 1e-9)
        m = (c > cos) & (d >= lo) & (d < hi)
        return (h[m] / (1 / d[m] - 1 / L)).mean()

    return [((lo, hi), float(mean(bo, lo, hi) / mean(ax, lo, hi) - 1)) for lo, hi in bands]


# ----------------------------------------------------------------------------------------------------------------------------------- tests
def test_e6b1_the_unrelaxed_walk_tensor_is_isotropic():
    for k in range(1, 6):
        M, ntiles = walk_tensor(k)
        assert ntiles == 8 ** k
        assert M[0][0] == M[1][1] == M[2][2], k
        off = {M[i][j] for i in range(3) for j in range(3) if i != j}
        assert len(off) == 1, (k, off)
        if k >= 2:
            assert off == {Fraction(-22, 49)}, (k, off)
    assert abs(float(Fraction(-22, 49) / M[0][0])) < 1e-4, "relative off-diagonal at level 5"


def test_e6b2_the_relaxed_tensor_is_isotropic_in_the_bulk():
    S, M = relaxed_tensor(6, 30.0)
    ev = np.linalg.eigvalsh(S)
    assert (ev.max() - ev.min()) / ev.mean() < 5e-4, ev
    assert 0.94 < ev.mean() / np.trace(M) * 3 < 0.97, (ev, M)
    for v in ((1, 1, 1), (1, 1, -1), (1, -1, 1), (-1, 1, 1)):
        v = np.array(v) / np.sqrt(3)
        assert abs(v @ S @ v / ev.mean() - 1) < 5e-4
    Sn, _ = relaxed_tensor(5, at_notch=True)
    en, vn = np.linalg.eigh(Sn)
    assert en[-1] / en[0] > 5, en
    assert abs(abs(vn[:, 0] @ np.ones(3)) / np.sqrt(3) - 1) < 1e-3, "the soft axis at the notch is (1, 1, 1)"


def test_e6b3_the_hop_count_is_cubic():
    c = by_class(hop_metric(6))
    for name, (lo, hi) in c.items():
        assert hi - lo < 0.01, (name, lo, hi)
    assert 0.55 < c["axis"][0] < 0.56 and 0.70 < c["face"][0] < 0.71 and 0.65 < c["body"][0] < 0.67, c
    g = [np.mean(v) for v in c.values()]
    assert 0.22 < spread(g) < 0.26, spread(g)
    t = taxicab_metric()
    assert np.allclose([t["axis"], t["face"], t["body"]], [0.5, 1 / np.sqrt(2), np.sqrt(3) / 2])
    assert spread(t.values()) > 2 * spread(g), "rounder than Z^3"


def test_e6b4_the_hitting_probability_is_isotropic_at_large_r():
    ex = body_excess(6)
    assert 0.015 < ex[0][1] < 0.03, ex
    assert abs(ex[-1][1]) < 0.003, ex
    assert abs(ex[-1][1]) < abs(ex[1][1]) < abs(ex[0][1]), ex


TESTS = [test_e6b1_the_unrelaxed_walk_tensor_is_isotropic, test_e6b2_the_relaxed_tensor_is_isotropic_in_the_bulk,
         test_e6b3_the_hop_count_is_cubic, test_e6b4_the_hitting_probability_is_isotropic_at_large_r]


def run_tests(verbose=True):
    failed = 0
    for t in TESTS:
        try:
            t()
            status = "PASS"
        except AssertionError as e:
            status = "FAIL %s" % (e,)
            failed += 1
        if verbose:
            print("  %-80s %s" % (t.__name__, status))
    print("%d test(s), %d failed" % (len(TESTS), failed))
    return failed


def story():
    for k in (1, 2, 5):
        M, n = walk_tensor(k)
        print("level %d (%d tiles): M_xx = M_yy = M_zz = %s, off-diagonal %s" % (k, n, M[0][0], M[0][1]))
    S, M = relaxed_tensor(6, 30.0)
    print("relaxed tensor in the bulk (level 6, w = 30): eigenvalues", np.round(np.linalg.eigvalsh(S), 4), "; unrelaxed diagonal %.4f" % M[0, 0])
    Sn, _ = relaxed_tensor(5, at_notch=True)
    print("same at the notch (level 5, whole supertile): eigenvalues", np.round(np.linalg.eigvalsh(Sn), 3), "(artefact)")
    c = by_class(hop_metric(6))
    print("hop count / r, r = 16..26, by class (min, max over its lines):", {k: (round(a, 3), round(b, 3)) for k, (a, b) in c.items()})
    print("taxicab Z^3:", {k: round(v, 3) for k, v in taxicab_metric().items()})
    print("h along body diagonals / along axes - 1:", [(b, "%.2f%%" % (100 * e)) for b, e in body_excess(6)])


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if "--story" in argv:
        story()
    if "--tests-only" in argv or not argv:
        return run_tests()
    return 0


if __name__ == "__main__":
    sys.exit(main())
