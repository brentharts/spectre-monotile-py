"""
chair_e8_192.py - the 192 Chair44 pyramid tips against the 192 equatorial roots of E8: what matches, and what cannot.

The numerical coincidence (QUANTUM_GRAVITY.md, section 3, item 2): the Chair44 has 192 pyramid tips (24 panels x 8), and E8 minus
the two pole fibres D4a, D4b has 192 roots (8 fibres x 24; models/e6_fibres.py).  This file records, with tests, the structure of
each side and compares them.  It is a test of a *proposal* (that the tips and the equatorial roots can be identified compatibly with
their structure); it is not evidence for it.

Chair44 data: models/data/r44_features.csv (Tsiokos, CC BY 4.0).  E8 data: the Hurwitz-pair roots of hopf_shells.py.

Exact results (tests):
  Chair44 side.
    C1. 24 panels with 8 pyramids each.  The signed heights are the 24 values +-1, ..., +-12 (units of 1e-4), each occurring exactly 8 times.
    C2. The panels fall into three classes of 8 panels by their set of heights: P = {1..8} (sum 36), N = {-8..-1} (sum -36),
        Z = {+-9, ..., +-12} (sum 0).  Every panel carries 8 distinct heights, and all panels of a class carry the same set.
    C3. Among the 48 symmetries of the cube about the centroid of the panel centres, only the identity maps the tip set to itself with
        heights preserved: the labelled tip set has trivial symmetry.
  E8 side.
    E1. 192 equatorial roots = 8 fibres x 3 blocks, 8 roots in each of the 24 (fibre, block) cells.
    E2. Every block, and every fibre, is closed under r -> -r.  The fibre group 2T (24 elements) acts freely on the 192 roots, with 8 orbits.
  Comparison.
    X1. Shape: both are 3 x 8 x 8 (three classes, 8 panels or fibres each, 8 tips or roots per cell).  This is a numerical match of shape only.
    X2. Negation: on E8 each block is closed under r -> -r; on the Chair the height flip h -> -h (bump <-> dent) exchanges P and N and fixes only Z.
        So the three-class structure is not the same structure with its involution.
    X3. Symmetry: E8's equatorial set carries a free action of a group of order 24; the Chair's labelled tip set has no nontrivial cube symmetry (C3),
        and the cube symmetries are the natural isometries for axis-aligned panels (that no other isometry preserves the set is argued, not tested).
        So no identification of the two sets can be equivariant for an isometric action on the Chair.
Hypothesis, not tested here: that some non-geometric labelling (of heights by elements of 2T, say) makes the 3 x 8 x 8 shapes correspond.
Nothing in this file shows it, and X2 and X3 rule out the two most natural versions.

    python3 models/chair_e8_192.py                  tests, then a figure
    python3 models/chair_e8_192.py --tests-only
    python3 models/chair_e8_192.py --no-show --save-dir figures
"""
import collections
import csv
import itertools
import os
import sys
from fractions import Fraction as F

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import e6_fibres as ef  # noqa: E402

DATA = os.path.join(_HERE, "data", "r44_features.csv")


def load_tips():
    """List of dicts: panel, axis, sign, height (signed integer, units of 1e-4), apex (3 Fractions), centre (3 Fractions)."""
    out = []
    with open(DATA) as fh:
        for r in csv.DictReader(fh):
            out.append(dict(panel=int(r["panel"]), axis=int(r["normal_axis"]), sign=int(r["outward_sign"]),
                            height=int(r["coefficient"]),
                            apex=tuple(F(r[k]) for k in ("apex_x", "apex_y", "apex_z")),
                            centre=tuple(F(r[k]) for k in ("panel_cx", "panel_cy", "panel_cz"))))
    return out


def panel_classes(tips):
    """{sorted height tuple: [panels]}."""
    P = collections.defaultdict(list)
    for t in tips:
        P[t["panel"]].append(t["height"])
    cls = collections.defaultdict(list)
    for p, h in P.items():
        cls[tuple(sorted(h))].append(p)
    return dict(cls)


def cube_symmetries_of(points, labels):
    """The signed permutations (about the centroid of ``points``) that map the set to itself with labels preserved."""
    cs = [sum(p[i] for p in points) / len(points) for i in range(3)]
    S = dict(zip(points, labels))
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            def g(x):
                return tuple(cs[i] + signs[i] * (x[perm[i]] - cs[perm[i]]) for i in range(3))
            if all(g(x) in S and S[g(x)] == S[x] for x in S):
                out.append((perm, signs))
    return out


def chair_panel_centres(tips):
    return sorted({t["centre"] for t in tips})


def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    tips = load_tips()
    check("C1  192 tips, 24 panels of 8; the signed heights +-1..+-12 each occur exactly 8 times",
          len(tips) == 192 and set(collections.Counter(t["panel"] for t in tips).values()) == {8}
          and collections.Counter(t["height"] for t in tips) == collections.Counter({h: 8 for h in list(range(-12, 0)) + list(range(1, 13))}))
    cls = panel_classes(tips)
    sums = sorted(sum(k) for k in cls)
    check("C2  three panel classes of 8 panels: heights {1..8} (sum 36), {-8..-1} (sum -36), {+-9..+-12} (sum 0); 8 distinct heights per panel",
          len(cls) == 3 and sorted(len(v) for v in cls.values()) == [8, 8, 8] and sums == [-36, 0, 36]
          and all(len(set(k)) == 8 for k in cls)
          and (1, 2, 3, 4, 5, 6, 7, 8) in cls and (-8, -7, -6, -5, -4, -3, -2, -1) in cls and (-12, -11, -10, -9, 9, 10, 11, 12) in cls)
    pts = [t["apex"] for t in tips]
    disp = [t["height"] for t in tips]
    check("C3  the labelled tip set has only the identity among the 48 cube symmetries about its centroid",
          len(cube_symmetries_of(pts, disp)) == 1)
    cube = [tuple(F(x) for x in v) for v in itertools.product((-1, 1), repeat=3)]
    check("control: the symmetry finder returns all 48 for the 8 cube vertices with equal labels",
          len(cube_symmetries_of(cube, [0] * 8)) == 48)
    check("control: it returns fewer when one vertex gets a different label", len(cube_symmetries_of(cube, [1] + [0] * 7)) < 48)
    R, lab, idx = ef.setup()
    pa, pb = ef.poles()
    blk = ef.blocks()
    eq = np.where(blk >= 0)[0]
    cells = collections.Counter((int(lab[i]), int(blk[i])) for i in eq)
    check("E1  192 equatorial roots = 8 fibres x 3 blocks with 8 roots in each of the 24 cells",
          len(eq) == 192 and len(cells) == 24 and set(cells.values()) == {8})
    neg = np.array([ef._find(-R[i]) for i in range(240)])
    check("E2  every block and every fibre is closed under r -> -r",
          all(blk[neg[i]] == blk[i] and lab[neg[i]] == lab[i] for i in eq))
    free = True
    orbits = set()
    for l in ef.hurwitz_units():
        p = ef.right_perm(l)
        if not np.allclose(l, [1, 0, 0, 0]):
            free &= all(p[i] != i for i in eq)
    for i in eq:
        orbits.add(int(lab[i]))
    check("E2' the fibre group 2T (24 elements) acts freely on the 192 equatorial roots, with 8 orbits", free and len(orbits) == 8)
    check("X1  both sides have the shape 3 x 8 x 8", len(cls) * 8 * 8 == 192 and 3 * len(orbits) * 8 == 192)
    flip = {t["panel"]: tuple(sorted(-h for h in [u["height"] for u in tips if u["panel"] == t["panel"]])) for t in tips}
    swaps = collections.Counter((k, flip[v[0]]) for k, v in cls.items())
    moved = [k for k in cls if flip[cls[k][0]] != k]
    check("X2  on the Chair the flip h -> -h exchanges the classes P and N and fixes Z; on E8 negation fixes every block",
          len(moved) == 2 and sum(1 for k in cls if flip[cls[k][0]] == k) == 1)
    check("X3  E8 side has a free action of order 24, Chair side has no nontrivial cube symmetry: no equivariant identification by cube symmetries",
          free and len(cube_symmetries_of(pts, disp)) == 1)
    return ok


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tips = load_tips()
    cls = panel_classes(tips)
    order = []
    for k in sorted(cls, key=lambda k: -sum(k)):
        order += sorted(cls[k])
    M = np.zeros((24, 8))
    for r, p in enumerate(order):
        M[r] = sorted(t["height"] for t in tips if t["panel"] == p)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.4))
    im = ax[0].imshow(M, cmap="coolwarm", vmin=-12, vmax=12, aspect="auto")
    for y in (7.5, 15.5):
        ax[0].axhline(y, color="k", lw=1.5)
    ax[0].set_ylabel("panel (3 classes of 8: P, Z, N)"); ax[0].set_xlabel("pyramid slot (heights sorted)")
    ax[0].set_title("Chair44: 24 panels x 8 signed heights")
    fig.colorbar(im, ax=ax[0], label="signed height (1e-4)")
    R, lab, _ = ef.setup()
    pa, pb = ef.poles()
    blk = ef.blocks()
    eqf = [f for f in range(10) if f not in (pa, pb)]
    G = np.array([[int(((lab == f) & (blk == k)).sum()) for k in range(3)] for f in eqf])
    ax[1].imshow(G, cmap="Blues", vmin=0, vmax=10, aspect="auto")
    for r in range(8):
        for c in range(3):
            ax[1].text(c, r, G[r, c], ha="center", va="center")
    ax[1].set_xticks(range(3)); ax[1].set_xticklabels(["block 0", "block 1", "block 2"])
    ax[1].set_yticks(range(8)); ax[1].set_yticklabels(["fibre %d" % (k + 1) for k in range(8)])
    ax[1].set_title("E8 equatorial: 8 fibres x 3 blocks x 8 roots\n(same shape 3 x 8 x 8; not the same structure)")
    fig.tight_layout()
    if save:
        os.makedirs(os.path.dirname(save) or ".", exist_ok=True)
        fig.savefig(save, dpi=150)
        print("saved", save)
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
    figure(os.path.join(d, "chair_e8_192.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
