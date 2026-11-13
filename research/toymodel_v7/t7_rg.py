#!/usr/bin/env python3
"""t7_rg.py - Stage 8 of toymodel_v7: renormalisation by the Spectre's own substitution.  Does the pair model of toymodel_v6 at level j map to the one at level j - 1 under the inflation, and does the map close?

The patch of level L is built by L substitutions; its tiles come out of `spectre_patch._build` in depth-first order.  Here the same construction is repeated with each tile's path of slot indices (and labels) recorded,
so that a level-1 supertile of the level-L patch is the set of tiles whose paths agree except for the last slot, and it corresponds to one tile of the level-(L-1) patch (same path).  The pairs are the flat
pairs of v4 (`t4_patch.flat_pairs`), the couplings those of v6 Stage 1 (bits as charges at the tile centroids, dipoles of length 3 at b = 1).

Exact / Numeric claims (each a test; ``python3 t7_rg.py --tests-only``)
  E7.25  The hierarchy is recovered exactly (the placements equal `SpectrePatch`'s at levels 2, 3, 4), and no flat pair lies inside a level-1 supertile (0 of 6, 82, 804): every pair joins two supertiles.  A block map by
         supertiles cuts every pair qubit; the pairs live on the boundaries of the blocks, as the gauge links of Stage 3 live on the boundaries of tiles.
  E7.26  Pairs are inherited.  Every flat pair of the level-(L-1) patch reappears as exactly one flat pair of the level-L patch, between the two supertiles that correspond to its tiles (6 of 6 at L = 3, 82 of 82 at L = 4).
         The other level-L pairs (76, 722) join adjacent supertiles whose tiles are not a pair; no pair joins supertiles that are not adjacent.  So N_L = N_(L-1) + (new pairs): 82 = 6 + 76, 804 = 82 + 722.
  E7.27  The pair set closes by a local rule.  The number of level-L pairs on the boundary of two adjacent supertiles is a function of the two tiles' labels and the edges they share: 194 types at L = 4 (96 at L = 3), no type with
         two different counts, never more than one pair per boundary; the 74 types seen at both levels give the same count.
  E7.28  The couplings of inherited pairs close too, and are irrelevant.  All inherited dipoles at a level transform by one fixed orthogonal map (the angle to the inflated parent is the same for all to 1e-9; it includes the
         substitution's reflection), and their midpoints inflate by 2.803 (lambda = 2.806).  For inherited pairs more than 60 apart at level 4 the coupling equals lambda^-3 times the dipolar coupling of the transformed parents
         (median ratio 1.002, quartiles within 3%).  The tunnelling of a pair does not change (the pair is the same object), so on the inherited pairs K / Gamma falls by lambda^-3 = 0.045 per level: the couplings
         flow to zero and the inherited pairs to the paramagnet.  The new pairs of each level carry tile-scale couplings: the ensemble is self-similar, with a trivial fixed point for the inherited part.

Status.  E7.25 to E7.27: Exact (combinatorics of the stated patches).  E7.28: Exact for the dipole map, Numeric for the coupling ratio.  The couplings are v6's Definition.  Closure is shown for the abelian (Ising) pair model only.

    python3 t7_rg.py --tests-only
    python3 t7_rg.py --no-show --save figures/t7_stage8_rg.png
"""
import argparse
import collections
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_RES)
for _p in (_HERE, os.path.join(_RES, "toymodel_v6"), os.path.join(_RES, "toymodel_v4"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import spectre_patch as sp  # noqa: E402
import t4_patch as tp  # noqa: E402
import t6_patch as tp6  # noqa: E402

LAMBDA = math.sqrt(4 + math.sqrt(15))
_C = {}


def build_paths(levels, a=1.0, b=1.0, rotation=30):
    """`spectre_patch._build` for the label Delta, with each tile's (shape, motion, path of slots, labels from the top)."""
    key = (levels, a, b)
    if key in _C:
        return _C[key]
    pts = sp._reference_points(a, b)
    quad = pts[[3, 5, 7, 11]]
    tiles = {lab: [(0, np.eye(3), (), (lab,))] for lab in sp.TILE_NAMES if lab != "Gamma"}
    tiles["Gamma"] = [(0, np.eye(3), (), ("Gamma",)), (1, sp._shift(pts[8]) @ sp._rot(rotation), (), ("Gamma",))]
    for _ in range(levels):
        base = quad
        total, transformations = 0, [np.eye(3)]
        transformed = base
        for angle, src, dst in ((60, 3, 1), (0, 2, 0), (60, 3, 1), (60, 3, 1), (0, 2, 0), (60, 3, 1), (-120, 3, 3)):
            if angle != 0:
                total += angle
                rm = sp._rot(total)
                transformed = sp._apply(rm, base)
            else:
                rm = sp._rot(total)
            trans = sp._shift(sp._apply(transformations[-1], base[src][None])[0] - transformed[dst])
            transformations.append(trans @ rm)
        mirror = np.diag([-1.0, 1.0, 1.0])
        transformations = [mirror @ t for t in transformations]
        quad = np.array([sp._apply(transformations[6], base[2][None])[0], sp._apply(transformations[5], base[1][None])[0],
                         sp._apply(transformations[3], base[2][None])[0], sp._apply(transformations[0], base[1][None])[0]])
        new = {}
        for lab, subst in sp.SUBSTITUTIONS.items():
            parts = []
            for slot, (name, trsf) in enumerate(zip(subst, transformations)):
                if name:
                    parts.extend((shape, trsf @ m, (slot,) + path, (name,) + labs) for shape, m, path, labs in tiles[name])
            new[lab] = parts
        tiles = new
    _C[key] = tiles["Delta"]
    return _C[key]


def placements_from_paths(levels):
    parts = build_paths(levels)
    t_ab = sp._reference_to_tile(1.0, 1.0)
    place = np.array([g @ np.linalg.inv(t_ab) for _, g, _, _ in parts])
    if np.all(np.linalg.det(place[:, :2, :2]) < 0):
        place = np.diag([-1.0, 1.0, 1.0]) @ place
    return place


def correspondence(L):
    """(supertile id per level-L tile, {path: [level-(L-1) tiles]}, labels of the level-(L-1) tiles, seams of level L-1 with edge pairs)."""
    key = ("corr", L)
    if key in _C:
        return _C[key]
    sup = [p[2][:-1] for p in build_paths(L)]
    prev = build_paths(L - 1)
    idx = collections.defaultdict(list)
    for i, p in enumerate(prev):
        idx[p[2]].append(i)
    labs = [p[3][0] for p in prev]
    seam = collections.defaultdict(set)
    for r in tp.patch(L - 1, 1.0).edge_pairs():
        a, ea, b, eb = map(int, r)
        seam[(min(a, b), max(a, b))].add((ea, eb) if a < b else (eb, ea))
    _C[key] = (sup, idx, labs, seam)
    return _C[key]


def classify(L):
    """For every level-L pair: ('inherited', parent pair) or ('new', adjacency) or ('apart', None)."""
    sup, idx, _, seam = correspondence(L)
    parents = set(tp.flat_pairs(L - 1, 1.0))
    out = []
    for a, b in tp.flat_pairs(L, 1.0):
        cands = [(min(x, y), max(x, y)) for x in idx[sup[a]] for y in idx[sup[b]]]
        par = [c for c in cands if c in parents]
        adj = [c for c in cands if c in seam]
        if par:
            out.append(("inherited", par[0], (a, b)))
        elif adj:
            out.append(("new", adj[0], (a, b)))
        else:
            out.append(("apart", None, (a, b)))
    return out


def rule_table(L):
    sup, idx, labs, seam = correspondence(L)
    cnt = collections.Counter(c[1] for c in classify(L) if c[1] is not None)
    table = collections.defaultdict(set)
    for (a, b), edges in seam.items():
        table[(labs[a], labs[b], tuple(sorted(edges)))].add(cnt.get((a, b), 0))
    return table, cnt


def inherited_geometry(L):
    inh = [(c[1], c[2]) for c in classify(L) if c[0] == "inherited"]
    _, Cp, _ = tp6.geometry(L - 1, 1.0)
    _, C, _ = tp6.geometry(L, 1.0)
    d_par = np.array([Cp[y] - Cp[x] for (x, y), _ in inh])
    d_ch = np.array([C[b] - C[a] for _, (a, b) in inh])
    m_par = np.array([(Cp[x] + Cp[y]) / 2 for (x, y), _ in inh])
    m_ch = np.array([(C[a] + C[b]) / 2 for _, (a, b) in inh])
    X, Y = m_par - m_par.mean(0), m_ch - m_ch.mean(0)
    U, S, Vt = np.linalg.svd(X.T @ Y)
    R = (U @ Vt).T
    s = S.sum() / (X ** 2).sum()
    return inh, C, d_par, d_ch, m_par, m_ch, R, s


def coupling_ratios(L=4, rmin=60.0):
    inh, C, d_par, d_ch, m_par, m_ch, R, s = inherited_geometry(L)
    cr = lambda u, v: u[0] * v[1] - u[1] * v[0]  # noqa: E731
    cos = np.array([(R @ x) @ y / np.linalg.norm(x) / np.linalg.norm(y) for x, y in zip(d_par, d_ch)])
    sin = np.array([cr(R @ x, y) / np.linalg.norm(x) / np.linalg.norm(y) for x, y in zip(d_par, d_ch)])
    c0, s0 = cos.mean(), sin.mean()
    P = np.array([[c0 * (R @ x)[0] - s0 * (R @ x)[1], s0 * (R @ x)[0] + c0 * (R @ x)[1]] for x in d_par])
    Mp = s * (m_par @ R.T)
    out = []
    for i in range(len(inh)):
        for j in range(i + 1, len(inh)):
            (a1, b1), (a2, b2) = inh[i][1], inh[j][1]
            if np.linalg.norm(m_ch[i] - m_ch[j]) < rmin:
                continue
            W = sum(sg / np.linalg.norm(C[p] - C[q]) for p, q, sg in ((a1, a2, 1), (a1, b2, -1), (b1, a2, -1), (b1, b2, 1)))
            n = Mp[i] - Mp[j]
            rr = np.linalg.norm(n)
            n = n / rr
            out.append(W / ((P[i] @ P[j] - 3 * (P[i] @ n) * (P[j] @ n)) / rr ** 3))
    return np.array(out), cos, sin, s


# --------------------------------------------------------------------------
def test_e725_no_pair_inside_a_supertile():
    for L, n in ((2, 6), (3, 82), (4, 804)):
        assert np.allclose(placements_from_paths(L), tp.patch(L, 1.0).placements)
        sup = [p[2][:-1] for p in build_paths(L)]
        pr = tp.flat_pairs(L, 1.0)
        assert len(pr) == n and sum(sup[a] == sup[b] for a, b in pr) == 0
    return "0 of 6, 82, 804"


def test_e726_pairs_are_inherited():
    out = {}
    for L, (n_par, n_new) in ((3, (6, 76)), (4, (82, 722))):
        c = classify(L)
        kinds = collections.Counter(k for k, _, _ in c)
        assert kinds == {"inherited": n_par, "new": n_new}, kinds
        per_parent = collections.Counter(x[1] for x in c if x[0] == "inherited")
        assert len(per_parent) == len(tp.flat_pairs(L - 1, 1.0)) and set(per_parent.values()) == {1}
        out[L] = dict(kinds)
    return out


def test_e727_the_pair_set_closes_by_a_local_rule():
    t3, _ = rule_table(3)
    t4, cnt4 = rule_table(4)
    assert len(t3) == 96 and len(t4) == 194
    assert all(len(v) == 1 for v in t3.values()) and all(len(v) == 1 for v in t4.values())
    assert max(cnt4.values()) == 1
    common = set(t3) & set(t4)
    assert len(common) == 74 and all(t3[k] == t4[k] for k in common)
    return len(t3), len(t4), len(common)


def test_e728_inherited_couplings_close_and_are_irrelevant():
    rat, cos, sin, s = coupling_ratios(4)
    assert np.ptp(cos) < 1e-9 and np.ptp(sin) < 1e-9
    assert abs(s - LAMBDA) / LAMBDA < 0.002
    q = np.percentile(rat, [25, 50, 75])
    assert abs(q[1] - 1) < 0.005 and q[0] > 0.97 and q[2] < 1.03, q
    return round(s, 4), [round(x, 4) for x in q], round(LAMBDA ** -3, 4)


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e7")]
    failed = 0
    for n, f in tests:
        try:
            r = f()
            if verbose:
                print("ok   %s  %s" % (n, "" if r is None else r))
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    fig, ax = plt.subplots(1, 3, figsize=(16, 5.2), gridspec_kw={"width_ratios": [1.3, 1, 1]})
    L = 3
    P = tp.patch(L, 1.0)
    sup = [p[2][:-1] for p in build_paths(L)]
    ids = {k: i for i, k in enumerate(sorted(set(sup)))}
    rng = np.random.default_rng(3)
    pal = rng.uniform(0.75, 0.97, size=(len(ids), 3))
    polys = [np.asarray(P.straight_polygon(i)) for i in range(P.n_tiles)]
    ax[0].add_collection(PolyCollection(polys, facecolors=[pal[ids[sup[i]]] for i in range(P.n_tiles)], edgecolors="0.6", linewidths=0.2))
    _, C, _ = tp6.geometry(L, 1.0)
    for kind, _, (a, b) in classify(L):
        col = "tab:red" if kind == "inherited" else "tab:blue"
        ax[0].plot(*np.array([C[a], C[b]]).T, color=col, lw=2.2 if kind == "inherited" else 1.0)
    ax[0].plot([], [], color="tab:red", lw=2.2, label="pair inherited from level 2 (6)")
    ax[0].plot([], [], color="tab:blue", lw=1.0, label="new pair on a supertile boundary (76)")
    ax[0].set_aspect("equal"); ax[0].set_xticks([]); ax[0].set_yticks([]); ax[0].autoscale_view(); ax[0].legend(fontsize=7, loc="lower left")
    ax[0].set_title("(a) level 3: supertiles (colours); every pair straddles two of them", fontsize=9)
    levels = [2, 3, 4]
    inh = [0, 6, 82]
    new = [6, 76, 722]
    ax[1].bar(levels, inh, color="tab:red", label="inherited")
    ax[1].bar(levels, new, bottom=inh, color="tab:blue", label="new")
    for x, a, b in zip(levels, inh, new):
        ax[1].text(x, a + b + 15, "%d" % (a + b), ha="center", fontsize=8)
    ax[1].set_xticks(levels); ax[1].set_xlabel("level"); ax[1].set_ylabel("flat pairs"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) N_L = N_(L-1) + new: 6, 82 = 6 + 76, 804 = 82 + 722", fontsize=9)
    rat, _, _, _ = coupling_ratios(4)
    ax[2].hist(rat, bins=np.linspace(0.85, 1.15, 61), color="0.4")
    ax[2].axvline(1, color="tab:red", lw=1)
    ax[2].set_xlabel("coupling / (lambda$^{-3}$ x inflated parent dipoles)"); ax[2].set_ylabel("pairs of inherited pairs, distance > 60")
    ax[2].set_title("(c) inherited couplings scale as lambda$^{-3}$ = %.3f per level" % LAMBDA ** -3, fontsize=9)
    fig.suptitle("toymodel_v7 Stage 8: the pair model under the Spectre's own substitution")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=140)
    if show:
        plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save")
    a = ap.parse_args(argv)
    failed = run_tests()
    if not a.tests_only and not failed:
        figure(a.save, not a.no_show)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
