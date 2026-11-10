"""
fig_chair_axis.py - where the Chair44 label is not fixed by its 2-adic address, and where the notch axis is not (Result 'chair-axis' of quantgrav.tex).

Left: the number of addresses of the 2^M cube at which the label is undetermined, c(M) = 28 * 2^M - 48 (circles), the label excess E(M) = 140 * 2^M + 48 (squares)
and the number at which even the notch AXIS (the notch vector up to sign) is undetermined (diamonds): 16 for every M >= 2, on a log axis against the closed forms.
Right: those 16 addresses for M = 3, the 8 corner cells (orange) and the 8 central cells (blue) of the 8 x 8 x 8 cube.  The counts come from the subset automaton of
chair44_notch_axis.py of the Hat and Spectre repository ($SPECTRE_DIR), imported; it is an independent implementation (chair44_twoadic.py) that agrees with CICY's
chair44_nested.py (c, E, and its notch_axis_counts).  Tests: closed forms for M = 1..7, 16 axes for M = 2..7, and that the 16 positions at M = 3 are exactly the 8
corners and the central 2 x 2 x 2 block.  Evidence: exact (finite automaton; the all-M statement is the recurrence certificate in the repositories).

    python3 models/fig_chair_axis.py                 tests, then the figure
    python3 models/fig_chair_axis.py --tests-only
    python3 models/fig_chair_axis.py --no-show --save-dir figures
"""
import itertools
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _spectre  # noqa: E402

MMAX = 7
POS_M = 3


def data():
    if _spectre.load() is None:
        return None
    import chair44_notch_axis as K
    rows = K.table(MMAX)
    pos, _ = K.exceptional_positions(POS_M)
    return dict(rows=rows, pos=[tuple(int(x) for x in p) for p in pos.tolist()])


def run_tests(D=None, verbose=True):
    ok = True

    def check(name, good, info=""):
        nonlocal ok
        ok &= bool(good)
        if verbose:
            print("  %-4s %-72s %s" % ("ok" if good else "FAIL", name, info))

    if D is None:
        check("Hat and Spectre repository not found", False, "set SPECTRE_DIR")
        return False
    rows = D["rows"]
    check("c(M) = 28 * 2^M - 48 and E(M) = 140 * 2^M + 48, M = 1..7", all(c == 28 * 2 ** M - 48 and e == 140 * 2 ** M + 48 for M, c, e, _, _ in rows[1:]),
          ", ".join(str(r[1]) for r in rows[1:]))
    check("undetermined notch axis at exactly 16 addresses, M = 2..7", all(r[3] == 16 for r in rows[2:]))
    check("... and at 1 and 8 addresses for M = 0, 1", (rows[0][3], rows[1][3]) == (1, 8))
    n = 2 ** POS_M
    corners = set(itertools.product((0, n - 1), repeat=3))
    centre = set(itertools.product((n // 2 - 1, n // 2), repeat=3))
    check("the 16 positions at M = 3 are the 8 corners and the central 2 x 2 x 2 block", set(D["pos"]) == corners | centre and len(D["pos"]) == 16)
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig = plt.figure(figsize=(10.6, 4.6))
    ax = fig.add_subplot(1, 2, 1)
    bx = fig.add_subplot(1, 2, 2, projection="3d")
    Ms = np.arange(1, MMAX + 1)
    rows = D["rows"]
    c = [r[1] for r in rows[1:]]
    e = [r[2] for r in rows[1:]]
    a = [r[3] for r in rows[1:]]
    ax.semilogy(Ms, 28 * 2.0 ** Ms - 48, color="#0072B2", lw=1, alpha=0.6)
    ax.semilogy(Ms, 140 * 2.0 ** Ms + 48, color="#D55E00", lw=1, alpha=0.6)
    ax.semilogy(Ms, c, "o", color="#0072B2", ms=6, label=r"undetermined label $c(M)=28\cdot2^M-48$")
    ax.semilogy(Ms, e, "s", color="#D55E00", ms=6, label=r"label excess $E(M)=140\cdot2^M+48$")
    ax.semilogy(Ms, a, "D", color="#009E73", ms=6, label="undetermined notch axis: 16 for $M\\geq2$")
    ax.set_xlabel("$M$ (binary digits of the address)")
    ax.set_ylabel("number of addresses")
    ax.set_xticks(Ms)
    ax.set_ylim(5, 4e4)
    ax.legend(loc="upper left", fontsize=8.5, frameon=False)
    ax.set_title("Labels grow with the cube; the axis does not", fontsize=10.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    n = 2 ** POS_M
    corners = set(itertools.product((0, n - 1), repeat=3))
    P = D["pos"]
    for kind, col, nm in ((True, "#D55E00", "8 corner cells"), (False, "#0072B2", "8 central cells")):
        pts = np.array([p for p in P if (tuple(p) in corners) == kind])
        bx.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=col, s=48, depthshade=False, label=nm)
    g = np.array([0, n - 1])
    for x, y, z in itertools.product(g, repeat=3):
        for dx, dy, dz in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            q = np.array([x, y, z]) + (n - 1) * np.array([dx, dy, dz])
            if q.max() <= n - 1 and np.array([x, y, z])[np.argmax([dx, dy, dz])] == 0:
                bx.plot([x, q[0]], [y, q[1]], [z, q[2]], color="0.75", lw=0.8)
    bx.set_xlim(-0.5, n - 0.5)
    bx.set_ylim(-0.5, n - 0.5)
    bx.set_zlim(-0.5, n - 0.5)
    bx.set_xticks([0, n - 1])
    bx.set_yticks([0, n - 1])
    bx.set_zticks([0, n - 1])
    bx.view_init(elev=22, azim=35)
    bx.legend(loc="upper left", fontsize=8.5, frameon=False)
    bx.set_title("The 16 addresses at $M=3$ ($8\\times8\\times8$ cells)", fontsize=10.5)
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    D = data()
    if not run_tests(D):
        return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(D, os.path.join(d, "chair_axis.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
