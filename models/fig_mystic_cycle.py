"""
fig_mystic_cycle.py - every X-bond of the Spectre tiling is a Mystic bond (Result 'mystic' of quantgrav.tex): a depth-3 patch, and one Mystic with its 14-cycle.

X-bond: a shared edge on which the two tiles disagree about the edge type (spectreatlas).  Left: the whole depth-3 patch, Mystic tiles shaded, every X-bond drawn
red, one interior cycle in black.  Right: that Mystic and its neighbours, the 14 X-bonds of the cycle, and its two interior vertices of valence 4 (blue dots).
The geometry and the counts come from spectre_mystic_xbond.py and spectreatlas.py of the Hat and Spectre repository ($SPECTRE_DIR), imported.  Tests (the
Counted statements M1, M2 of that repository, at depth 3, plus the chosen cycle): a shared edge is an X-bond exactly when one of its tiles is the Mystic and no
two Mystics share an edge; the X-bonds form disjoint 14-cycles with two valence-4 vertices each, one per Mystic; X-bonds = 14 M = 7 x valence-4 vertices.
Evidence label: counted (a finite check at one depth, the paper's (S) snapshot covers depths 2 to 4).

    python3 models/fig_mystic_cycle.py                 tests, then the figure
    python3 models/fig_mystic_cycle.py --tests-only
    python3 models/fig_mystic_cycle.py --no-show --save-dir figures
"""
import collections
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _spectre  # noqa: E402

DEPTH = 3


def data(depth=DEPTH):
    """-> dict: patch, table, valence, boundary vertices, X edges, cycles (list of dicts), chosen cycle."""
    if _spectre.load() is None:
        return None
    import spectre_mystic_xbond as M
    patch = M.patch_for(depth)
    val, bv, tab = M.vertex_data(patch)
    xedges = [k for k, ow in tab.items() if len(ow) == 2 and ow[0][2] != ow[1][2]]
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for k in xedges:
        parent[find(k[0])] = find(k[1])
    comps = collections.defaultdict(list)
    for k in xedges:
        comps[find(k[0])].append(k)
    cyc = []
    for r, es in comps.items():
        vs = {v for e in es for v in e}
        owners = {o[0] for e in es for o in tab[e]}
        mystic = [i for i in owners if patch[i][0] == "Gamma2"]
        cyc.append(dict(edges=es, verts=vs, v4=[v for v in vs if val[v] == 4 and v not in bv], interior=not (vs & bv), mystic=mystic))
    allc = np.mean([p for _, w in patch for p in w], axis=0)
    inner = [c for c in cyc if c["interior"]]
    chosen = min(inner, key=lambda c: np.linalg.norm(np.mean(list(c["verts"]), axis=0) - allc))
    return dict(M=M, patch=patch, tab=tab, val=val, bv=bv, xedges=xedges, cycles=cyc, chosen=chosen)


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
    M, P = D["M"], D["patch"]
    n, x, wrong, mm, bad = M.mystic_check(P)
    check("M1: X-bond iff exactly one Mystic; no Mystic-Mystic bond", n > 0 and wrong == 0 and mm == 0 and bad == 0, "%d bonds, %d X-bonds" % (n, x))
    nc, shape, nv4, on, nm = M.cycles(P)
    check("M2: X-bonds are disjoint 14-cycles with two valence-4 vertices each, one per Mystic", set(shape) == {(14, 14, 2)} and nc == nm, "%d cycles, %d Mystics" % (nc, nm))
    check("M2: X-bonds = 14 M = 7 x (interior valence-4 vertices), v4 = 2 M", len(D["xedges"]) == 14 * nm and nv4 == 2 * nm and len(D["xedges"]) == 7 * nv4,
          "X = %d, M = %d, v4 = %d" % (len(D["xedges"]), nm, nv4))
    c = D["chosen"]
    check("chosen cycle: 14 edges, 14 vertices, 2 interior valence-4 vertices, one Mystic", len(c["edges"]) == 14 and len(c["verts"]) == 14 and len(c["v4"]) == 2 and len(c["mystic"]) == 1)
    own = [sum(1 for o in D["tab"][e] if P[o[0]][0] == "Gamma2") for e in c["edges"]]
    check("chosen cycle: every edge has exactly one Mystic owner (the same Mystic)", own == [1] * 14 and len({o[0] for e in c["edges"] for o in D["tab"][e] if P[o[0]][0] == "Gamma2"}) == 1)
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    P, c = D["patch"], D["chosen"]
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11.5, 5.6), gridspec_kw=dict(width_ratios=[1.15, 1]))

    def draw(a, tiles, lw=0.4, xlw=1.6):
        for lab, w in tiles:
            a.add_patch(Polygon(w, closed=True, fc="#F3C98B" if lab == "Gamma2" else "#EEF3F8", ec="0.55", lw=lw))
        for e in D["xedges"]:
            if a is bx and not any(e in cc["edges"] for cc in [c]) and not near(e):
                continue
            (x0, y0), (x1, y1) = e
            a.plot([x0, x1], [y0, y1], color="#D55E00", lw=xlw)
        a.set_aspect("equal")
        a.axis("off")

    cv = np.mean(list(c["verts"]), axis=0)
    rad = max(np.linalg.norm(np.array(v) - cv) for v in c["verts"]) + 1.6

    def near(e):
        return all(np.linalg.norm(np.array(p) - cv) < rad for p in e)

    draw(ax, P, 0.3, 0.9)
    for (x0, y0), (x1, y1) in c["edges"]:
        ax.plot([x0, x1], [y0, y1], color="k", lw=2.0)
    allp = np.array([p for _, w in P for p in w])
    ax.set_xlim(allp[:, 0].min() - 1, allp[:, 0].max() + 1)
    ax.set_ylim(allp[:, 1].min() - 1, allp[:, 1].max() + 1)
    nm = sum(1 for lab, _ in P if lab == "Gamma2")
    ax.set_title("Depth-3 patch: %d tiles, %d Mystics (orange), %d X-bonds (red)" % (len(P), nm, len(D["xedges"])), fontsize=10)
    near_tiles = [(lab, w) for lab, w in P if np.linalg.norm(np.mean(w, axis=0) - cv) < rad]
    draw(bx, near_tiles, 0.8, 2.0)
    for (x0, y0), (x1, y1) in c["edges"]:
        bx.plot([x0, x1], [y0, y1], color="k", lw=2.6)
    for v in c["v4"]:
        bx.plot(*v, "o", color="#0072B2", ms=10, mec="white", mew=1.2, zorder=5)
        u = (np.array(v) - cv) / np.linalg.norm(np.array(v) - cv)
        bx.annotate("valence 4", v, xytext=tuple(26 * u * np.array([1, 1])), textcoords="offset points", fontsize=8.5, color="#0072B2",
                    ha="left" if u[0] > 0 else "right", va="bottom" if u[1] > 0 else "top")
    bx.set_xlim(cv[0] - rad, cv[0] + rad)
    bx.set_ylim(cv[1] - rad, cv[1] + rad)
    bx.set_title("One Mystic: its 14 X-bonds form a cycle through 2 valence-4 vertices", fontsize=10)
    fig.text(0.5, 0.02, "X-bonds $=14\\times$ Mystics $=7\\times$ valence-4 vertices; Mystic frequency $(5-\\sqrt{15})/10$ (counted at depths 2 to 4, exact frequencies)", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
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
    figure(D, os.path.join(d, "mystic_cycle.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
