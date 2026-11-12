#!/usr/bin/env python3
"""t5_source.py - Stage 2 of toymodel_v5: Chair44 as a neutral source.

The 192 pyramid tips of Chair44 carry the signed depth j (+j a bump, -j a dent, j = 1..12, 8 of each sign and depth; Tsiokos, models/data).  Read the depth as a charge q = j and the apex as a
position: a source in the sense of toymodel_v2 (Stage 5): a snap leaves a static field, a pair snap (+q, -q) is a dipole with no net source.  Nothing here is gravity; "charge", "field" and "quadrupole" are
used for the lattice-free Coulomb potential  phi(r) = sum_i q_i / |r - r_i|  (the three-dimensional Green function of toymodel_v2, E5.7) and nothing else.

Definitions
  D5.8   Moments about the centre c = (1, 1, 1) of the chair: M0 = sum q, M1_a = sum q d_a, Q_ab = sum q d_a d_b (d = r - c), computed with the exact apexes (Fractions); the traceless quadrupole
         Qt = Q - tr(Q)/3 (the trace does not contribute to a harmonic far field).
  D5.9   The chair's symmetry group G: signed permutations of the axes about c that map the seven-cube chair onto itself (order 6, the permutations of the axes).
  D5.10  The far-field residual R(r) = phi - (3/2) n.Qt.n / r^3 (the quadrupole term), n = r / |r|.

Exact claims (each a test; ``python3 t5_source.py --tests-only``)
  E5.11  Monopole 0 and dipole 0, exactly (Fractions), about any point; 8 bumps and 8 dents of each depth.  tr Q = 91/50 (not zero), the traceless quadrupole has eigenvalues -47.817, 0.104, 47.713: a
         saddle (two large, equal and opposite values), not an axial quadrupole about the chair's axis (1, 1, 1).
  E5.12  The set of tip positions is invariant under all six elements of G.  The charges are not: the exchange of two axes (x <-> y) changes the sign of all 192 charges (exact), the other two exchanges
         change the sign of 144 of 192 and the two three-fold rotations leave 144 of 192 unchanged.  So the charge pattern is odd under a reflection of the chair and only 3/4 invariant under its three-fold axis.
  E5.13  Far field: the potential of the 192 charges tends to the quadrupole term and the residual falls one power faster (r^-4, the octupole): r^3 phi -> 39.4 (mean absolute value over eight fixed random
         directions, r = 64) while a single charge falls as r^-1 and a pair as r^-2 (fitted slopes -1.00, -2.00).
  E5.14  Neutrality of the ensemble.  A bit of a 14th-vertex group changes the area of its tile by 2 a h, a net source +-a h about the mean; the four groups have zero net source for exactly 6 of the 16 value
         patterns of their bits (two and two), which is the bump-and-dent pairing of toymodel_v4 (only antiparallel pairs open on a patch).  Then the ensemble is a neutral source at leading order like the chair.

Status.  Exact for the moments and the symmetry table; the far-field laws are fits (Numeric).  Reading the depth as a charge, and the apex as its position, is a Definition of this toy: nothing says the depth couples
to a field.  The odd parity under x <-> y is a fact about the data, not an explanation.

    python3 t5_source.py --tests-only
    python3 t5_source.py --no-show --save figures/t5_stage2_source.png
"""
import argparse
import itertools
import math
import os
import sys
from fractions import Fraction

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, os.path.join(_ROOT, "models"), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import chair44_core  # noqa: E402
import chair_fibers as cf  # noqa: E402

CENTRE = (Fraction(1), Fraction(1), Fraction(1))


def tips():
    return cf.ChairTips()


def charges(t=None):
    t = t or tips()
    return [int(c) for c in t.coeff]


def displacements(t=None):
    t = t or tips()
    return [tuple(x[i] - CENTRE[i] for i in range(3)) for x in t.exact]


def moments(t=None):
    t = t or tips()
    q, d = charges(t), displacements(t)
    m0 = sum(q)
    m1 = tuple(sum(qi * di[a] for qi, di in zip(q, d)) for a in range(3))
    Q = [[sum(qi * di[a] * di[b] for qi, di in zip(q, d)) for b in range(3)] for a in range(3)]
    return m0, m1, Q


def traceless(Q):
    tr = sum(Q[i][i] for i in range(3))
    return tr, np.array([[float(Q[a][b] - (tr / 3 if a == b else 0)) for b in range(3)] for a in range(3)])


def phi(points, q, r):
    """Coulomb potential of charges q at points (N, 3) at the position r (3,)."""
    return float((np.asarray(q, float) / np.linalg.norm(np.asarray(points, float) - np.asarray(r, float), axis=1)).sum())


def quad_term(Qt, r):
    r = np.asarray(r, float)
    rr = np.linalg.norm(r)
    n = r / rr
    return 1.5 * float(n @ Qt @ n) / rr ** 3


def symmetry_elements():
    """The signed permutation matrices that map the seven-cube chair onto itself (about its centre)."""
    cc = {tuple(2 * np.array(c) - 1) for c in chair44_core.CHAIR_CELLS}
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for i, p in enumerate(perm):
                M[i, p] = sg[i]
            if {tuple(np.rint(M @ np.array(c)).astype(int)) for c in cc} == cc:
                out.append((perm, M))
    return out


def symmetry_table(t=None):
    """For every element of the chair's group: (tips mapped onto tips, same charge, opposite charge)."""
    t = t or tips()
    r = np.array([[float(x) for x in d] for d in displacements(t)])
    q = np.array(charges(t))
    key = lambda p: tuple(np.round(2 * p, 2))  # noqa: E731
    table = {key(r[i]): q[i] for i in range(len(q))}
    rows = []
    for perm, M in symmetry_elements():
        onto = same = opp = 0
        for i in range(len(q)):
            j = table.get(key(M @ r[i]))
            if j is None:
                continue
            onto += 1
            same += int(j == q[i])
            opp += int(j == -q[i])
        rows.append((perm, onto, same, opp))
    return rows


def loglog_slope(fn, radii, dirs):
    v = np.array([np.mean([abs(fn(r * d)) for d in dirs]) for r in radii])
    return float(np.polyfit(np.log(radii), np.log(v), 1)[0])


def random_dirs(n=8, seed=1):
    rng = np.random.default_rng(seed)
    d = rng.normal(size=(n, 3))
    return d / np.linalg.norm(d, axis=1)[:, None]


def net_source_patterns(n=4):
    """Bit patterns s in {+1, -1}^n of the groups and their net source sum(s) a h (the bit moves the area by +-a h)."""
    return [(s, sum(s)) for s in itertools.product((1, -1), repeat=n)]


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e511_neutral_with_no_dipole_and_a_saddle_quadrupole():
    t = tips()
    m0, m1, Q = moments(t)
    assert m0 == 0 and m1 == (0, 0, 0)
    q = np.array(charges(t))
    for j in range(1, 13):
        assert (q == j).sum() == 8 and (q == -j).sum() == 8
    tr, Qt = traceless(Q)
    assert tr == Fraction(91, 50), tr
    w = np.sort(np.linalg.eigvalsh(Qt))
    assert np.allclose(w, [-47.8174, 0.1040, 47.7134], atol=1e-3), w
    # not axial about (1,1,1): the axial part (n.Qt.n)(3 n n^T - 1)/2 is 0.4% of the norm of Qt, and the eigenvector of the small eigenvalue is perpendicular to the axis
    n = np.ones(3) / math.sqrt(3)
    axial = float(n @ Qt @ n) * (3 * np.outer(n, n) - np.eye(3)) / 2
    assert np.linalg.norm(axial) / np.linalg.norm(Qt) < 0.005
    v = np.linalg.eigh(Qt)[1][:, 1]
    assert abs(float(v @ n)) < 1e-9


def test_e512_positions_have_the_chair_symmetry_and_charges_do_not():
    rows = symmetry_table()
    assert len(rows) == 6 and all(onto == 192 for _, onto, _, _ in rows)
    d = {perm: (same, opp) for perm, _, same, opp in rows}
    assert d[(0, 1, 2)] == (192, 0)
    assert d[(1, 0, 2)] == (0, 192)
    assert d[(0, 2, 1)] == (0, 144) and d[(2, 1, 0)] == (0, 144)
    assert d[(1, 2, 0)] == (144, 0) and d[(2, 0, 1)] == (144, 0)


def test_e513_far_field_is_quadrupole_with_an_octupole_correction():
    t = tips()
    P = np.array([[float(x) for x in d] for d in displacements(t)])
    q = np.array(charges(t), float)
    _, Qt = traceless(moments(t)[2])
    dirs = random_dirs()
    f = lambda r: phi(P, q, r)  # noqa: E731
    r3 = [np.mean([abs(f(r * d)) for d in dirs]) * r ** 3 for r in (16.0, 32.0, 64.0)]
    assert abs(r3[2] - 39.4) < 0.5 and r3[0] > r3[1] > r3[2] - 1e-9
    res = [np.mean([abs(f(r * d) - quad_term(Qt, r * d)) for d in dirs]) * r ** 4 for r in (32.0, 64.0, 128.0)]
    assert max(res) / min(res) < 1.3, res
    one = loglog_slope(lambda r: phi(np.zeros((1, 3)), [1.0], r), (8.0, 16.0, 32.0), dirs)
    pair = loglog_slope(lambda r: phi(np.array([[0, 0, 0.1], [0, 0, -0.1]]), [1.0, -1.0], r), (8.0, 16.0, 32.0), dirs)
    assert abs(one + 1.0) < 1e-6 and abs(pair + 2.0) < 0.05, (one, pair)


def test_e514_six_of_sixteen_bit_patterns_have_no_net_source():
    pats = net_source_patterns(4)
    assert len(pats) == 16 and sum(1 for _, s in pats if s == 0) == 6 == math.comb(4, 2)
    assert max(abs(s) for _, s in pats) == 4


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

    t = tips()
    q = np.array(charges(t), float)
    P = np.array([[float(x) for x in d] for d in displacements(t)])
    _, Qt = traceless(moments(t)[2])
    fig = plt.figure(figsize=(15, 9.5))

    ax = fig.add_subplot(2, 3, 1, projection="3d")
    quads = []
    for axis, sign, cell in chair44_core.exposed_faces(chair44_core.CHAIR_CELLS):
        t1, t2 = [k for k in range(3) if k != axis]
        qd = []
        for i, j in ((0, 0), (1, 0), (1, 1), (0, 1)):
            v = [0, 0, 0]
            v[axis], v[t1], v[t2] = cell[axis] + (1 if sign > 0 else 0), cell[t1] + i, cell[t2] + j
            qd.append(v)
        quads.append(qd)
    ax.add_collection3d(Poly3DCollection(quads, facecolor="#dde4f0", edgecolor="0.5", alpha=0.12, linewidths=0.4))
    sc = ax.scatter(*(P + 1).T, c=q, cmap="coolwarm", vmin=-12, vmax=12, s=10 + 4 * np.abs(q), depthshade=False)
    fig.colorbar(sc, ax=ax, shrink=0.6, label="charge j")
    ax.set_box_aspect([1, 1, 1]); ax.view_init(22, 40)
    ax.set_title("(a) the 192 tips as charges: monopole 0, dipole 0", fontsize=8)

    ax = fig.add_subplot(2, 3, 2)
    xs = np.linspace(-1.5, 3.5, 90)
    G = np.array([[phi(P + 1, q, (x, y, 2.6)) for x in xs] for y in xs])
    m = np.abs(G).max()
    im = ax.pcolormesh(xs, xs, G, cmap="coolwarm", vmin=-m, vmax=m, shading="auto")
    fig.colorbar(im, ax=ax, label="phi")
    ax.set_aspect("equal"); ax.set_title("(b) potential on the plane z = 2.6 above the chair", fontsize=8)

    ax = fig.add_subplot(2, 3, 3)
    radii = np.array([3, 4, 6, 8, 12, 16, 24, 32, 48, 64.0])
    dirs = random_dirs()
    chair = [np.mean([abs(phi(P, q, r * d)) for d in dirs]) for r in radii]
    one = [1.0 / r for r in radii]
    pair = [np.mean([abs(phi(np.array([[0, 0, 0.1], [0, 0, -0.1]]), [1, -1], r * d)) for d in dirs]) for r in radii]
    ax.loglog(radii, one, "o-", label="one charge: r^-1")
    ax.loglog(radii, pair, "s-", label="a +-1 pair: r^-2")
    ax.loglog(radii, chair, "^-", color="crimson", label="Chair44: r^-3 (quadrupole)")
    ax.loglog(radii, 39.4 * radii ** -3.0, "k:", lw=0.8)
    ax.set_xticks([4, 8, 16, 32, 64]); ax.set_xticklabels(["4", "8", "16", "32", "64"]); ax.minorticks_off()
    ax.set_xlabel("distance"); ax.set_ylabel("mean |phi| over 8 directions"); ax.legend(fontsize=7)
    ax.set_title("(c) far field of one charge, a pair and the chair", fontsize=8)

    ax = fig.add_subplot(2, 3, 4)
    th, ph = np.meshgrid(np.linspace(0, math.pi, 90), np.linspace(0, 2 * math.pi, 180), indexing="ij")
    N = np.stack([np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)], axis=-1)
    A = 1.5 * np.einsum("ija,ab,ijb->ij", N, Qt, N)
    im = ax.pcolormesh(np.degrees(ph), 90 - np.degrees(th), A, cmap="coolwarm", vmin=-70, vmax=70, shading="auto")
    fig.colorbar(im, ax=ax, label="r^3 phi")
    ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("latitude (deg)")
    ax.set_title("(d) the quadrupole pattern r^3 phi on the sphere", fontsize=8)

    ax = fig.add_subplot(2, 3, 5)
    rows = symmetry_table()
    labels = ["".join("xyz"[p] for p in perm) for perm, *_ in rows]
    same = [r_[2] for r_ in rows]
    opp = [r_[3] for r_ in rows]
    ax.bar(range(6), same, color="tab:blue", label="charge kept")
    ax.bar(range(6), opp, bottom=same, color="tab:red", label="charge reversed")
    ax.set_xticks(range(6)); ax.set_xticklabels(labels); ax.set_ylim(0, 215)
    ax.set_ylabel("tips (of 192)"); ax.legend(fontsize=7)
    ax.set_title("(e) the chair's six symmetries on the charges; positions are always kept", fontsize=8)

    ax = fig.add_subplot(2, 3, 6)
    ax.axis("off")
    w = np.sort(np.linalg.eigvalsh(Qt))
    lines = [
        "monopole       0   (exact)",
        "dipole         0   (exact)",
        "tr Q           91/50",
        "Qt eigenvalues %.3f, %.3f, %.3f" % tuple(w),
        "",
        "net source of 4 bits: zero for 6 of 16 patterns (two and two)",
        "= the bump-and-dent pairs of toymodel_v4",
        "",
        "Reading the depth as a charge is a Definition of the toy.",
    ]
    for i, ln in enumerate(lines):
        ax.text(0.0, 0.92 - 0.09 * i, ln, fontsize=9, family="monospace", transform=ax.transAxes)
    fig.tight_layout()
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", metavar="PNG")
    args = ap.parse_args(argv)
    print("t5_source self tests:")
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
