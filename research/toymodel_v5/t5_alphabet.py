#!/usr/bin/env python3
"""t5_alphabet.py - Stage 0 of toymodel_v5: the six points around the 14th vertex as an alphabet, and the cube.

Why.  toymodel_v2 argued that the alphabet of the timelines must be the six roots of an A2 (hexagonal), not the cubic six.  Here the six points that the bit of
the 14th vertex touches (the vertices P7, P9 and the four handles of the edges 7 and 8; models/chair_fibers_14th.py) are the candidate letters, and the cube is the
chair's own six sides.  Nothing here is physics: it is the algebra of one tile and one cube.

Setting (a = 1, convex value of the bit).  Relative to P8 the six points are  (0, +-a),  (+k, +-a/2),  (-k, +-a/2)  for the Spectre family at any b, tau (the edges 7 and 8 are
vertical chords of length a; k is the curvature strength).  They form three antipodal pairs about P8 (u1 = (0, a), u2 = (k, a/2), u3 = (k, -a/2)).

Definitions
  D5.1  Frame operator  S(k) = sum_{i=1..3} u_i u_i^T = diag(2 k^2, 3 a^2 / 2); anisotropy alpha(k) = min / max of its eigenvalues.
  D5.2  A *cube view* is a 2 x 3 matrix P whose columns are the images of the cube's three axes e_x, e_y, e_z, one from each antipodal pair, with a sign each:
        the six points are then P(+-e_i).  The view is *orthographic* when P P^T is a multiple of the identity (the rows are orthogonal and of equal length): then P is a
        scaled projection of the cube along the unit vector n = (row1 x row2)/|.|.
  D5.3  The *120 degree choice*: the plus ends of the three pairs are taken so that the three plus points are 120 degrees apart (sum zero).  There are two such triples,
        exchanged by inversion about P8.

Exact claims (each a test; ``python3 t5_alphabet.py --tests-only``)
  E5.1  The following are equivalent: (i) the six points have equal length; (ii) they are a regular hexagon (the six roots of an A2, up to scale); (iii) S(k) is a multiple of the
        identity (alpha = 1, a *tight frame*); (iv) k = sqrt(3) a / 2.  For k < sqrt3 a/2 the frame is elongated along the chords, alpha = 4 k^2 / (3 a^2).  At k = a/2 the six
        points are at 45 degree steps with lengths a and a/sqrt2 (a square-lattice look, alpha = 1/3).
  E5.2  At k = sqrt3 a / 2 the six points are exactly the orthographic image of the cube's six sides, scaled by sqrt(3/2) a.  With the 120 degree choice the viewing direction is
        n = +-(1, 1, 1)/sqrt3 under the assignment of the pairs to the axes of models/chair_fibers_14th.py (z: the two handles of P8, x: P7 and P9, y: the handles of the neighbours),
        in whichever cyclic order the pairs are given to x, y, z.  The boundary-order rule of chair_fibers_14th.py (plus = later along the boundary) gives n = +-(1, -1, -1)/sqrt3
        instead.  Both are body diagonals; they are not equivalent for the chair.
  E5.3  The seven-cube chair of Chair44 (the cube {0,1}^3 without the cell (1,1,1)) is invariant under the three-fold rotation about (1,1,1) and under no three-fold rotation about
        any of the other three body diagonals.  So the 120 degree choice puts the plane of the tile perpendicular to the chair's own three-fold axis, and the boundary-order rule does not.
  E5.4  Along the flow with k(tau) = k0 (1 - tau): alpha = (k(tau) / k_hex)^2 = (1 - tau)^2 when k0 = k_hex: the alphabet is a regular hexagon at the Spectre end and a line (rank one)
        at the Hat, where the six points lie on the chords.  The outline of the tile with k0 = sqrt3/2 is a simple closed curve along the whole flow (grid of 21 values).
  E5.5  The Gram matrix of the six unit directions is that of the A2 roots (entries 1, 1/2, -1/2, -1) at k = k_hex and not for any other k; its largest deviation is
        |cos(30 deg) - cos(atan(1/(2k/a)))| along the nearest pairs.

Status.  Exact algebra of the tile model.  That the letters of the dynamics *are* these points, and that the chair's axis plays any role, are Hypotheses of the toy; k = sqrt3/2 lies
outside the range (k0 <= 0.8) of research/toymodel_v4.

    python3 t5_alphabet.py --tests-only
    python3 t5_alphabet.py --no-show --save figures/t5_stage0_alphabet.png
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for p in (_ROOT, os.path.join(_ROOT, "models"), os.path.join(os.path.dirname(_HERE), "toymodel_v4")):
    if p not in sys.path:
        sys.path.insert(0, p)

import chair44_core  # noqa: E402
from spectremodel import Tile  # noqa: E402
import chair_fibers_14th as c14  # noqa: E402
from t4_critical import _intersects  # noqa: E402

SQRT3 = math.sqrt(3.0)
K_HEX = SQRT3 / 2.0
ROLES = c14.GROUP_ROLE          # +x +y +z -x -y -z of the boundary-order rule
PAIRS = (("v7", "v9"), ("v7.out", "v9.in"), ("v8.in", "v8.out"))   # x, y, z


def points(k, a=1.0):
    """The six points relative to P8, from the formula, by role name."""
    return {"v7": np.array([0.0, a]), "v9": np.array([0.0, -a]),
            "v7.out": np.array([k, a / 2]), "v9.in": np.array([-k, -a / 2]),
            "v8.in": np.array([-k, a / 2]), "v8.out": np.array([k, -a / 2])}


def points_from_tile(k, tau=0.0, a=1.0):
    """The same, read off a Tile (a, b(tau), k)."""
    b = a * (1.0 + (SQRT3 - 1.0) * tau)
    t = Tile(a, b, k)
    gp = c14.group_points(t)
    p8 = np.asarray(t.vertices[c14.FLAT].position, dtype=float)
    return {name: np.asarray(v.position, dtype=float) - p8 for name, v in gp.items()}


def frame_operator(pts):
    return sum(np.outer(pts[p[1]], pts[p[1]]) for p in PAIRS)


def anisotropy(k, a=1.0):
    ev = np.sort(np.linalg.eigvalsh(frame_operator(points(k, a))))
    return float(ev[0] / ev[1])


def triple_120(pts):
    """The two triples of plus points that are 120 degrees apart (sum zero); each as {pair index: name}."""
    out = []
    for signs in itertools.product((0, 1), repeat=3):
        names = [PAIRS[i][signs[i]] for i in range(3)]
        if np.linalg.norm(sum(pts[n] for n in names)) < 1e-9 * (1 + max(np.linalg.norm(pts[n]) for n in names)):
            out.append(names)
    return out


def view_direction(columns):
    """Viewing direction of the cube view with the given three columns (images of e_x, e_y, e_z), or None if not orthographic."""
    P = np.array(columns, dtype=float).T
    G = P @ P.T
    if abs(G[0, 1]) > 1e-9 or abs(G[0, 0] - G[1, 1]) > 1e-9:
        return None
    n = np.cross(P[0], P[1])
    return n / np.linalg.norm(n)


def chair_cells():
    return set(chair44_core.CHAIR_CELLS)


def rotates_chair_into_itself(axis):
    """Is the three-fold rotation about the body diagonal ``axis`` (a +-1 vector) a symmetry of the chair?"""
    d = np.array(axis, dtype=float)
    R = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float)      # (x, y, z) -> (z, x, y) about (1, 1, 1)
    D = np.diag(d)                                                       # +-1 signs: maps (1,1,1) to the diagonal d
    M = D @ R @ D
    cells = chair_cells()
    centred = {tuple(2 * np.array(c) - 1) for c in cells}
    mapped = {tuple(np.rint(M @ np.array(c)).astype(int)) for c in centred}
    return mapped == centred


def outline_is_simple(tile, samples=120):
    t = np.linspace(0.0, 1.0, samples + 1)
    B = np.stack([(1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t ** 2 * (1 - t), t ** 3], axis=1)
    S = np.einsum("mj,ejd->emd", B, tile.control_points())
    for i, j in itertools.combinations(range(len(S)), 2):
        if _intersects(S[i], S[j]):
            return False
    return True


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e51_four_conditions_are_one():
    for a in (1.0, 2.0):
        kh = SQRT3 * a / 2
        for k in (0.3 * a, 0.5 * a, 0.7 * a, kh, 1.0 * a, 1.3 * a):
            pts = points(k, a)
            L = [np.linalg.norm(v) for v in pts.values()]
            equal = max(L) - min(L) < 1e-9
            ang = np.sort(np.degrees(np.arctan2([v[1] for v in pts.values()], [v[0] for v in pts.values()])))
            gaps = np.diff(np.concatenate([ang, [ang[0] + 360.0]]))
            regular = equal and np.allclose(gaps, 60.0, atol=1e-7)
            tight = abs(anisotropy(k, a) - 1.0) < 1e-12
            at_hex = abs(k - kh) < 1e-12
            assert equal == regular == tight == at_hex, (a, k, equal, regular, tight, at_hex)
        assert abs(anisotropy(0.5 * a, a) - 1.0 / 3.0) < 1e-12
    # the tile itself, not only the formula, gives these points
    for k, tau in ((0.5, 0.0), (K_HEX, 0.0), (0.4, 0.6)):
        f, t = points(k * (1 - tau) if False else k), points_from_tile(k, tau)
        for n in f:
            assert np.allclose(f[n], t[n], atol=1e-9), (n, f[n], t[n])


def test_e52_the_hexagon_is_the_shadow_of_the_cube():
    pts = points(K_HEX)
    assert np.allclose(frame_operator(pts), 1.5 * np.eye(2))
    triples = triple_120(pts)
    assert len(triples) == 2
    assert np.allclose(sorted(map(tuple, (pts[n] for n in triples[0]))), sorted(map(tuple, (-pts[n] for n in triples[1]))))
    ones = np.ones(3) / SQRT3
    for names in triples:
        by_pair = {PAIRS.index(next(p for p in PAIRS if n in p)): n for n in names}
        for order in ((0, 1, 2), (1, 2, 0), (2, 0, 1)):        # even orders of the pairs onto x, y, z: looks along +(1, 1, 1)
            assert np.allclose(view_direction([pts[by_pair[i]] for i in order]), ones)
        for order in ((0, 2, 1), (2, 1, 0), (1, 0, 2)):        # odd orders: the opposite sense, -(1, 1, 1)
            assert np.allclose(view_direction([pts[by_pair[i]] for i in order]), -ones)
    # the boundary-order rule (plus = later along the boundary) gives (1, -1, -1), a diagonal that is not the chair's axis
    n = view_direction([pts[r] for r in ROLES[:3]])
    assert np.allclose(n, np.array([1.0, -1.0, -1.0]) / SQRT3), n
    # no orthographic view away from k_hex
    assert view_direction([points(0.5)[n] for n in ("v9", "v9.in", "v8.out")]) is None


def test_e53_the_chair_has_one_three_fold_axis():
    assert len(chair_cells()) == 7
    assert rotates_chair_into_itself((1, 1, 1))
    assert not any(rotates_chair_into_itself(d) for d in ((1, -1, -1), (-1, 1, -1), (-1, -1, 1)))


def test_e54_the_alphabet_flows_to_a_line():
    for tau in np.linspace(0.0, 1.0, 21):
        k = K_HEX * (1.0 - tau)
        assert abs(anisotropy(k) - (1.0 - tau) ** 2) < 1e-12
    assert anisotropy(0.0) == 0.0
    for tau in np.linspace(0.0, 0.95, 20):
        t = Tile(1.0, 1.0 + (SQRT3 - 1.0) * tau, K_HEX * (1.0 - tau))
        assert outline_is_simple(t), tau


def test_e55_the_gram_matrix_is_a2_only_at_k_hex():
    A2 = np.array([[math.cos(math.radians(60 * ((i - j) % 6))) for j in range(6)] for i in range(6)])

    def sorted_gram(k):
        v = np.array(list(points(k).values()))
        v = v[np.argsort(np.arctan2(v[:, 1], v[:, 0]))]
        v = v / np.linalg.norm(v, axis=1)[:, None]
        return v @ v.T

    assert np.abs(sorted_gram(K_HEX) - A2).max() < 1e-12
    for k in (0.3, 0.5, 0.8, 0.9, 1.2):
        assert np.abs(sorted_gram(k) - A2).max() > 1e-3


def run_tests():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        print("  %-72s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def figure():
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure(figsize=(15, 9.5))
    gs = fig.add_gridspec(2, 3)

    ax = fig.add_subplot(gs[0, 0])
    for k, col in ((0.5, "tab:blue"), (0.7, "tab:green"), (K_HEX, "crimson")):
        pts = points(k)
        v = np.array(list(pts.values()))
        v = v[np.argsort(np.arctan2(v[:, 1], v[:, 0]))]
        ax.plot(*np.vstack([v, v[:1]]).T, color=col, marker="o", lw=1.2, label="k=%.3f" % k)
    ax.set_aspect("equal"); ax.legend(fontsize=7)
    ax.set_title("(a) the six points around P8: regular hexagon (the A2 roots) only at k = sqrt3/2", fontsize=8)

    ax = fig.add_subplot(gs[0, 1])
    ks = np.linspace(0.0, 1.3, 131)
    ev = np.array([np.sort(np.linalg.eigvalsh(frame_operator(points(k)))) for k in ks])
    ax.plot(ks, ev[:, 0], label="small eigenvalue of S(k)"); ax.plot(ks, ev[:, 1], label="large eigenvalue of S(k)")
    ax.plot(ks, [anisotropy(k) for k in ks], "k", label="anisotropy (min/max)")
    ax.axvline(K_HEX, color="crimson", ls="--", lw=0.8); ax.axvline(0.5, color="tab:blue", ls=":", lw=0.8)
    ax.set_xlabel("k"); ax.legend(fontsize=7)
    ax.set_title("(b) frame operator of the three antipodal pairs: a tight frame iff k = sqrt3/2 (a = 1)", fontsize=8)

    ax = fig.add_subplot(gs[0, 2])
    taus = np.linspace(0, 1, 101)
    ax.plot(taus, (1 - taus) ** 2, "crimson", label="k0 = sqrt3/2: (1 - tau)^2")
    ax.plot(taus, [anisotropy(0.8 * (1 - t)) for t in taus], "tab:blue", label="k0 = 0.8")
    ax.plot(taus, [anisotropy(0.5 * (1 - t)) for t in taus], "tab:green", label="k0 = 0.5")
    ax.set_xlabel("tau (0 Spectre, 1 Hat)"); ax.set_ylabel("anisotropy of the alphabet"); ax.legend(fontsize=7)
    ax.set_title("(c) along the flow the alphabet loses a direction: a line at the Hat", fontsize=8)

    ax = fig.add_subplot(gs[1, 0], projection="3d")
    pts = points(K_HEX)
    names = triple_120(pts)[0]
    by_pair = {PAIRS.index(next(p for p in PAIRS if n in p)): n for n in names}
    cols = [pts[by_pair[i]] for i in range(3)]
    P = np.array(cols).T
    n = view_direction(cols)
    for i, (c, lab) in enumerate(zip("rgb", "xyz")):
        e = np.zeros(3); e[i] = 1.0
        ax.plot([-e[0], e[0]], [-e[1], e[1]], [-e[2], e[2]], color=c, lw=2)
        ax.text(*(1.12 * e), "+" + lab, fontsize=8)
    for s_ in itertools.product((-1, 1), repeat=3):
        pass
    cube = [[(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]]
    for a_, b_ in itertools.combinations(cube[0], 2):
        if sum(abs(u - v) for u, v in zip(a_, b_)) == 2:
            ax.plot(*zip(a_, b_), color="0.6", lw=0.5)
    ax.plot(*zip(-1.7 * n, 1.7 * n), color="k", ls="--", lw=1.2)
    ax.text(*(1.8 * n), "view (1,1,1)", fontsize=8)
    # the plane of the tile: perpendicular to n through the origin
    u1 = P[0] / np.linalg.norm(P[0]); u2 = P[1] / np.linalg.norm(P[1])
    g = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])
    ax.add_collection3d(Poly3DCollection([[a_ * u1 + b_ * u2 for a_, b_ in g]], alpha=0.12, facecolor="tab:orange"))
    ax.set_box_aspect([1, 1, 1]); ax.set_xlim(-1.8, 1.8); ax.set_ylim(-1.8, 1.8); ax.set_zlim(-1.8, 1.8)
    ax.view_init(20, 35)
    ax.set_title("(d) the six sides of a cube; the tile plane is perpendicular to (1,1,1)", fontsize=8)

    ax = fig.add_subplot(gs[1, 1], projection="3d")
    cells = sorted(chair_cells())
    quads = []
    for axis, sign, cell in chair44_core.exposed_faces(chair44_core.CHAIR_CELLS):
        t1, t2 = [k for k in range(3) if k != axis]
        q = []
        for i, j in ((0, 0), (1, 0), (1, 1), (0, 1)):
            v = [0, 0, 0]
            v[axis], v[t1], v[t2] = cell[axis] + (1 if sign > 0 else 0), cell[t1] + i, cell[t2] + j
            q.append(v)
        quads.append(q)
    ax.add_collection3d(Poly3DCollection(quads, facecolor="#c9d6ea", edgecolor="0.4", alpha=0.35, linewidths=0.5))
    c0 = np.array([1.0, 1.0, 1.0])
    tile = Tile(1.0, 1.0, K_HEX)
    ox, oy = tile.outline(80)
    p8 = np.asarray(tile.vertices[c14.FLAT].position, dtype=float)
    scale = 0.5
    q = np.column_stack([ox, oy]) - p8
    X = c0 + 2.2 * n + scale * (2.0 / 3.0) * (q @ P)       # points of the tile in the plane perpendicular to (1,1,1)
    ax.plot(*X.T, color="crimson", lw=1.2)
    for name, v in pts.items():
        ax.scatter(*(c0 + 2.2 * n + scale * (2.0 / 3.0) * (v @ P)), c="tab:orange", s=18)
    ax.plot(*zip(c0 - 1.2 * n, c0 + 2.6 * n), color="k", ls="--", lw=1)
    ax.text(*(c0 + 2.7 * n), "C3 axis (1,1,1)", fontsize=8)
    ax.set_box_aspect([1, 1, 1]); ax.set_xlim(-0.5, 3.5); ax.set_ylim(-0.5, 3.5); ax.set_zlim(-0.5, 3.5)
    ax.view_init(25, 35)
    ax.set_title("(e) Chair44 (7 cubes) and the k = sqrt3/2 tile perpendicular to its three-fold axis", fontsize=8)

    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    rows = [
        ("rule", "plus ends", "viewing direction", "chair C3 axis?"),
        ("boundary order", "later on boundary", "(1,-1,-1)/sqrt3", "no"),
        ("120 degree choice", "three at 120 deg", "(1,1,1)/sqrt3", "yes"),
    ]
    for i, r in enumerate(rows):
        for j, t in enumerate(r):
            ax.text((0.0, 0.42, 0.72, 0.9)[j] if False else (0.0, 0.40, 0.66, 0.90)[j], 0.85 - 0.12 * i, t, fontsize=7, weight="bold" if i == 0 else None, transform=ax.transAxes)
    ax.text(0.0, 0.35, "Exact algebra of the tile model (E5.1 - E5.5).\nThat the letters of the dynamics are these\nsix points is a Hypothesis; k = sqrt3/2 is outside\nthe range k0 <= 0.8 of toymodel_v4.", fontsize=8, transform=ax.transAxes)
    fig.tight_layout()
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", metavar="PNG")
    args = ap.parse_args(argv)
    print("t5_alphabet self tests:")
    if not run_tests():
        return 1
    if args.tests_only:
        return 0
    import matplotlib

    if args.no_show or args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = figure()
    if args.save:
        fig.savefig(args.save, dpi=140)
        print("saved", args.save)
    elif not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
