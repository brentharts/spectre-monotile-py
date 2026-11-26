#!/usr/bin/env python3
"""
toymodel_v12, stage S1b tightened: local moves and all gluings of the closed Spectre folds.

`t12_folds.py` had two weak points: a swap could exchange tiles far apart, and each patch was folded by the one
gluing the DP returns first.  Here

  * a **local swap** removes a tile and adds one that is a neighbour of the removed tile in the planar patch
    (the tile hops to an adjacent position); adding or removing one tile is local already;
  * **all gluings** of each patch are enumerated: every non-crossing perfect matching of the boundary edges in
    which every glued vertex has angle <= 360 degrees (the conditions of the `ballgen` DP, counted instead of
    decided).  A gluing is summarised by its *outcome*: the multiset of positive deficits and the number of pairs.

Definition-free measure: a move between patches P and P' **must** change the curvature if no gluing of P and no
gluing of P' have the same cone multiset.

Model (Definition): a state is (patch, outcome); H = -Gamma A - J pairs, where A joins the outcomes of two patches
related by a local move, and the different outcomes of one patch (a *re-gluing*: same tiles, another seam).

Run:  BALLGEN_DIR=... python3 t12_folds_local.py      (needs results_folds.json; `--tests-only` for the checks)
"""
from __future__ import annotations

import json
import os
import sys
from collections import Counter
from functools import lru_cache

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_folds as TF      # noqa: E402

FULL = 360


def gluings(A, flat):
    """Counter {(deficits tuple, seam pairs): number of gluings} over all legal closed gluings of a boundary loop with
    start-vertex angles A and flags flat[k] (the vertex is the flat corner of a single tile)."""
    m = len(A)
    A = tuple(int(a) for a in A)
    flat = tuple(bool(f) for f in flat)

    @lru_cache(maxsize=None)
    def bub(l, r):
        """dict key (S, ntop (capped 3), nflat among tops (capped 3), cones, pairs) -> count, for the run l..r."""
        if r < l:
            return {(0, 0, 0, (), 0): 1}
        out = Counter()
        for j in range(l + 1, r + 1, 2):
            inner = bub(l + 1, j - 1)
            tail = bub(j + 1, r)
            for (si, ni, fi, ci, pi), wi in inner.items():
                ang = si + A[j]
                if ang > FULL:
                    continue
                is_pair = (ni == 1 and fi == 1 and flat[j] and ang == FULL)
                c1 = ci + ((FULL - ang,) if ang < FULL else ())
                for (st, nt, ft, ct, pt), wt in tail.items():
                    S = A[l] + st
                    if S > FULL:
                        continue
                    key = (S, min(3, nt + 1), min(3, ft + int(flat[l])), tuple(sorted(c1 + ct)), pi + pt + int(is_pair))
                    out[key] += wi * wt
        return dict(out)
    res = Counter()
    for (S, n, f, c, p), w in bub(0, m - 1).items():
        if n == 0:
            continue
        pair = (n == 2 and f == 2 and S == FULL)
        cones = tuple(sorted(c + ((FULL - S,) if S < FULL else ())))
        res[(cones, p + int(pair))] += w
    return res


def patch_outcomes(tiles):
    """(Counter of outcomes with total pairs = planar + seam, number of gluings) for a tile set, or None."""
    g = TF.pool().graph(list(tiles))
    if g is None:
        return None
    edges, ang = g.loops[0]
    A = [int(a) for a in ang]
    flat = [(h % 14 == TF.FLAT and a == 180) for h, a in zip(edges, A)]
    planar = 0
    seen = {}
    for lt in range(g.n_tiles):
        v = int(g.tile_vertex[lt, TF.FLAT])
        seen.setdefault(v, []).append(lt)
    for v, lst in seen.items():
        if len(lst) == 2 and int(g.vertex_angle[v]) == 360:
            planar += 1
    out = Counter()
    for (cones, p), w in gluings(A, flat).items():
        out[(cones, p + planar)] += w
    return out


def local_edges(folds):
    """(i, j, kind): add (one tile more) or local swap (a tile replaced by a planar neighbour of it)."""
    nb = TF.pool().nbrs
    idx = {tuple(f["tiles"]): i for i, f in enumerate(folds)}
    universe = sorted({t for f in folds for t in f["tiles"]})
    U = set(universe)
    edges = []
    for i, f in enumerate(folds):
        s = set(f["tiles"])
        for t in universe:
            if t in s:
                continue
            j = idx.get(tuple(sorted(s | {t})))
            if j is not None:
                edges.append((i, j, "add"))
        for u in s:
            for t in nb[u]:
                if t in s or t not in U:
                    continue
                j = idx.get(tuple(sorted((s - {u}) | {t})))
                if j is not None and i < j:
                    edges.append((i, j, "swap"))
    return edges


def must_change(outs, edges):
    rows = {}
    for kind in ("add", "swap"):
        es = [(i, j) for i, j, k in edges if k == kind]
        ca = [({c for c, _ in outs[i]}, {c for c, _ in outs[j]}) for i, j in es]
        pa = [({p for _, p in outs[i]}, {p for _, p in outs[j]}) for i, j in es]
        rows[kind] = {"moves": len(es),
                      "curvature_must_change": sum(1 for a, b in ca if not a & b),
                      "curvature_can_stay": sum(1 for a, b in ca if a & b),
                      "cone_count_must_change": sum(1 for a, b in ca if not {len(x) for x in a} & {len(x) for x in b}),
                      "pair_count_must_change": sum(1 for a, b in pa if not a & b)}
    return rows


def model(folds, outs, edges, Gamma, J=1.0, kinds=("add", "swap"), regluing=True, restrict=None):
    nodes = [(i, o) for i in range(len(folds)) if restrict is None or i in restrict for o in sorted(outs[i])]
    pos = {n: a for a, n in enumerate(nodes)}
    by = {}
    for a, (i, o) in enumerate(nodes):
        by.setdefault(i, []).append(a)
    n = len(nodes)
    Aadj = np.zeros((n, n), dtype=np.float32)
    for i, j, k in edges:
        if k in kinds and i in by and j in by:
            for a in by[i]:
                for b in by[j]:
                    Aadj[a, b] = Aadj[b, a] = 1
    if regluing:
        for i, lst in by.items():
            for a in lst:
                for b in lst:
                    if a != b:
                        Aadj[a, b] = 1
    # largest connected component
    comp = -np.ones(n, int)
    c = 0
    for s in range(n):
        if comp[s] >= 0:
            continue
        stack = [s]
        comp[s] = c
        while stack:
            x = stack.pop()
            for y in np.where(Aadj[x] > 0)[0]:
                if comp[y] < 0:
                    comp[y] = c
                    stack.append(y)
        c += 1
    big = np.argmax(np.bincount(comp))
    keep = np.where(comp == big)[0]
    npairs = np.array([nodes[a][1][1] for a in keep], float)
    ncone = np.array([len(nodes[a][1][0]) for a in keep], float)
    H = -Gamma * Aadj[np.ix_(keep, keep)].astype(np.float64) - J * np.diag(npairs)
    w, v = np.linalg.eigh(H)
    p = v[:, 0] ** 2
    multi = Counter()
    cone = Counter()
    for q, a in zip(p, keep):
        multi[nodes[a][1][0]] += q
        cone[len(nodes[a][1][0])] += q
    return {"Gamma": Gamma, "states": int(n), "component": int(len(keep)), "components": int(c),
            "mean_cone_points": float(p @ ncone), "var_cone_points": float(p @ ncone ** 2 - (p @ ncone) ** 2),
            "mean_pairs": float(p @ npairs), "P_cone_points": {str(k): round(float(x), 5) for k, x in sorted(cone.items())},
            "cone_multisets_above_1pct": sum(1 for x in multi.values() if x > 0.01), "gap": float(w[1] - w[0])}


# ---- tests ------------------------------------------------------------------------------------
def test_square_pillow():
    # four edges, all corners 90: the two gluings (0-1,2-3) and (0-3,1-2) each give cone angles 90, 90 and 180
    r = gluings([90, 90, 90, 90], [False] * 4)
    assert sum(r.values()) == 2 and set(r) == {((180, 270, 270), 0)}


def test_two_flat_corners_make_a_pair():
    # a digon-like loop: edges 0,1 with start angles 180,180 both flat: gluing 0-1 gives classes {v1} alone (180)
    r = gluings([180, 180], [True, True])
    assert sum(r.values()) == 1


def test_enumeration_agrees_with_the_dp_on_a_window():
    from spectre.dp import can_close
    W = TF.window(10)
    seen = 0
    for mask in range(3, 1 << 10):
        tiles = [W[i] for i in range(10) if (mask >> i) & 1]
        g = TF.pool().graph(tiles)
        if g is None:
            continue
        o = patch_outcomes(tiles)
        assert bool(o) == can_close([int(a) for a in g.loops[0][1]])
        if o:
            f = TF.fold(tiles)
            assert (f["deficits"], len(f["pairs"])) in o, (tiles, f["deficits"], len(f["pairs"]), list(o)[:3])
            assert all(sum(c) == 720 for c, _ in o)
            seen += 1
    assert seen > 50


def run_tests():
    TF._ballgen()
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main():
    d = json.load(open(os.path.join(HERE, "results_folds.json")))
    folds = [{"tiles": tuple(f["tiles"]), "deficits": tuple(f["deficits"]), "pairs": f["pairs"]} for f in d["fold_list"]]
    outs = [patch_outcomes(f["tiles"]) for f in folds]
    ng = [sum(o.values()) for o in outs]
    out = {"folds": len(folds), "gluings_total": int(sum(ng)), "gluings_per_patch": {"min": int(min(ng)), "median": float(np.median(ng)), "max": int(max(ng))},
           "patches_with_one_gluing": sum(1 for x in ng if x == 1),
           "patches_with_one_cone_multiset": sum(1 for o in outs if len({c for c, _ in o}) == 1),
           "cone_multisets_per_patch": {str(k): v for k, v in sorted(Counter(len({c for c, _ in o}) for o in outs).items())},
           "cone_multisets_overall": len({c for o in outs for c, _ in o}),
           "deficit_values": sorted({x for o in outs for c, _ in o for x in c}),
           "cone_points_range": [min(len(c) for o in outs for c, _ in o), max(len(c) for o in outs for c, _ in o)],
           "max_pairs": max(p for o in outs for _, p in o)}
    edges = local_edges(folds)
    comps = TF.components(len(folds), edges)
    out["local_edges"] = dict(Counter(k for _, _, k in edges))
    out["local_components"] = [len(c) for c in comps[:8]]
    out["local_first_gluing_table"] = TF.move_table(folds, edges)
    out["must_change"] = must_change(outs, edges)
    sw = [e for e in edges if e[2] == "swap"]
    eight = {i for i, f in enumerate(folds) if len(f["tiles"]) == 8}
    c8 = TF.components(len(folds), [e for e in sw if e[0] in eight and e[1] in eight])
    c8 = [c for c in c8 if c[0] in eight]
    out["local_swap_components_at_8_tiles"] = [len(c) for c in c8[:8]]
    print(json.dumps(out, indent=1), flush=True)
    out["model_all_gluings"] = [model(folds, outs, edges, G) for G in (0.2, 1.0, 3.0)]
    out["model_all_gluings_swaps_at_8_tiles"] = [model(folds, outs, sw, G, kinds=("swap",), restrict=eight) for G in (0.2, 1.0, 3.0)]
    out["model_first_gluing_local"] = [TF.quantum(folds, edges, comps[0], G) for G in (0.2, 1.0, 3.0)]
    for k in ("model_all_gluings", "model_all_gluings_swaps_at_8_tiles", "model_first_gluing_local"):
        for r in out[k]:
            print(k, r, flush=True)
    json.dump(out, open(os.path.join(HERE, "results_folds_local.json"), "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
