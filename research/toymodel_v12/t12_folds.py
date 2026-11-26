#!/usr/bin/env python3
"""
toymodel_v12, stage S1b: closed Spectre folds, a substrate on which the curvature moves.

On the Hat Trick sphere the cone points are frozen (`t12_holes.py`).  The other closed substrate in hand is the
*fold*: a disc patch of Spectre tiles whose boundary is glued to itself edge to edge so that every vertex has
angle <= 360 degrees (the exact interval DP of the `ballgen` repository, `BALLGEN_DIR`).  The result is a closed
flat surface with cone points; deficits add to 720 degrees.

**Window (Definition).**  The `n` tiles of the level-3 Spectre patch nearest to its centre.  *All* subsets of the
window are tested: a state is a subset that is a topological disc and can be glued shut (**Exact**, exhaustive in
the window).  The gluing is the one the DP returns first; a patch may have others, which are not enumerated.

For each fold: the multiset of positive deficits, and the bump-and-dent *pairs*: classes of glued vertices that
consist of exactly two tile corners, both the flat corner (index 8, 180 degrees) of their tiles.  This is the
planar rule "two flat vertices meet alone", applied after gluing, so pairs can also form across the seam.

**Moves.**  Two folds are joined when their tile sets differ by one tile (add / remove) or by one tile exchanged
for another (swap).  For each move: the change of the cone multiset, of the number of cone points and of the
pairs.  **Model (Definition).**  On a connected component of the move graph, H = -Gamma A - J * (number of pairs),
A the adjacency matrix: geometry states that differ in curvature are superposed.  kappa = 0; no dipoles here.

Run:  BALLGEN_DIR=/path/to/ballgen python3 t12_folds.py [window=18]       (`--tests-only` for the checks)
"""
from __future__ import annotations

import itertools
import json
import os
import sys
import time
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FLAT = 8
_B = {}


def _ballgen():
    d = os.environ.get("BALLGEN_DIR")
    for c in ([d] if d else []) + [os.path.join(HERE, "..", "..", "..", "ballgen")]:
        if c and os.path.exists(os.path.join(c, "spectre", "dp.py")):
            if c not in sys.path:
                sys.path.insert(0, c)
            return os.path.abspath(c)
    raise SystemExit("set BALLGEN_DIR to a clone of brentharts/ballgen")


def pool(levels=3):
    if "pool" not in _B:
        _ballgen()
        from spectre.patch import SpectrePatch
        from spectre.search import PatchPool
        _B["pool"] = PatchPool(SpectrePatch(levels=levels))
    return _B["pool"]


def window(n):
    p = pool()
    d0 = np.linalg.norm(p.cents - p.c0, axis=1)
    return [int(t) for t in np.argsort(d0, kind="stable")[:n]]


def fold(tiles):
    """None, or dict(tiles, deficits (positive, sorted), ledger, pairs = sorted list of (tile, tile)) for the closed
    fold of this tile set."""
    from spectre.dp import can_close, fold_with_holes
    g = pool().graph(list(tiles))
    if g is None:
        return None
    if not can_close([int(a) for a in g.loops[0][1]]):
        return None
    r = fold_with_holes(g, [])
    if r is None or r.holes:
        return None
    cls = {}
    for lt in range(g.n_tiles):
        for k in range(14):
            cls.setdefault(r.find(int(g.tile_vertex[lt, k])), []).append((lt, k, int(g.angle[lt, k])))
    pairs = []
    for v, lst in cls.items():
        if len(lst) == 2 and all(k == FLAT and a == 180 for _, k, a in lst) and lst[0][0] != lst[1][0]:
            pairs.append(tuple(sorted(g.tile_ids[lt] for lt, _, _ in lst)))
    ds = [c.deficit for c in r.cones]
    return {"tiles": tuple(sorted(int(t) for t in tiles)), "deficits": tuple(sorted(d for d in ds if d > 0)), "ledger": int(sum(ds)),
            "negative": any(d < 0 for d in ds), "pairs": tuple(sorted(pairs)),
            "planar_pairs": sum(1 for v, lst in cls.items() if len(lst) == 2 and all(k == FLAT for _, k, _ in lst)
                                and int(g.vertex_angle[g.tile_vertex[lst[0][0], lst[0][1]]]) == 360)}


def enumerate_folds(W, kmin=2, verbose=False):
    out = []
    n = len(W)
    t0 = time.time()
    for mask in range(1, 1 << n):
        if bin(mask).count("1") < kmin:
            continue
        f = fold([W[i] for i in range(n) if (mask >> i) & 1])
        if f is not None:
            out.append(f)
        if verbose and mask % 32768 == 0:
            print("  ", mask, len(out), round(time.time() - t0), "s", flush=True)
    return out


def move_graph(folds):
    """Edges (i, j, kind) between folds whose tile sets differ by one tile (add) or by one exchanged tile (swap)."""
    idx = {f["tiles"]: i for i, f in enumerate(folds)}
    universe = sorted({t for f in folds for t in f["tiles"]})
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
                j = idx.get(tuple(sorted((s - {u}) | {t})))
                if j is not None and i < j:
                    edges.append((i, j, "swap"))
    return edges


def components(n, edges):
    parent = list(range(n))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for i, j, _ in edges:
        parent[find(i)] = find(j)
    comp = {}
    for i in range(n):
        comp.setdefault(find(i), []).append(i)
    return sorted(comp.values(), key=len, reverse=True)


def move_table(folds, edges):
    rows = {}
    for kind in ("add", "swap"):
        es = [(i, j) for i, j, k in edges if k == kind]
        dn = Counter(len(folds[j]["deficits"]) - len(folds[i]["deficits"]) for i, j in es)
        dp = Counter(len(folds[j]["pairs"]) - len(folds[i]["pairs"]) for i, j in es)
        rows[kind] = {"moves": len(es),
                      "cone_multiset_changed": sum(1 for i, j in es if folds[i]["deficits"] != folds[j]["deficits"]),
                      "cone_count_changed": sum(1 for i, j in es if len(folds[i]["deficits"]) != len(folds[j]["deficits"])),
                      "pair_set_changed": sum(1 for i, j in es if folds[i]["pairs"] != folds[j]["pairs"]),
                      "pair_count_changed": sum(1 for i, j in es if len(folds[i]["pairs"]) != len(folds[j]["pairs"])),
                      "both_cones_and_pairs_changed": sum(1 for i, j in es if folds[i]["deficits"] != folds[j]["deficits"] and folds[i]["pairs"] != folds[j]["pairs"]),
                      "change_in_cone_count": {str(k): v for k, v in sorted(dn.items())},
                      "change_in_pair_count": {str(k): v for k, v in sorted(dp.items())}}
    return rows


def quantum(folds, edges, comp, Gamma, J=1.0, kinds=("add", "swap")):
    """Ground state of H = -Gamma A - J pairs on one component; distribution of the number of cone points."""
    pos = {i: a for a, i in enumerate(comp)}
    n = len(comp)
    H = np.zeros((n, n))
    for i, j, k in edges:
        if k in kinds and i in pos and j in pos:
            H[pos[i], pos[j]] -= Gamma
            H[pos[j], pos[i]] -= Gamma
    npairs = np.array([len(folds[i]["pairs"]) for i in comp], float)
    ncone = np.array([len(folds[i]["deficits"]) for i in comp], float)
    ntile = np.array([len(folds[i]["tiles"]) for i in comp], float)
    H -= J * np.diag(npairs)
    w, v = np.linalg.eigh(H)
    p = v[:, 0] ** 2
    by_cone = Counter()
    by_multi = Counter()
    for a, i in enumerate(comp):
        by_cone[len(folds[i]["deficits"])] += p[a]
        by_multi[folds[i]["deficits"]] += p[a]
    ent = float(-(p[p > 1e-15] * np.log2(p[p > 1e-15])).sum())
    return {"Gamma": Gamma, "E0": float(w[0]), "gap": float(w[1] - w[0]) if n > 1 else None,
            "mean_cone_points": float(p @ ncone), "var_cone_points": float(p @ ncone ** 2 - (p @ ncone) ** 2),
            "mean_pairs": float(p @ npairs), "var_pairs": float(p @ npairs ** 2 - (p @ npairs) ** 2),
            "mean_tiles": float(p @ ntile), "var_tiles": float(p @ ntile ** 2 - (p @ ntile) ** 2),
            "participation_bits": ent, "max_probability": float(p.max()),
            "P_cone_points": {str(k): round(float(x), 6) for k, x in sorted(by_cone.items())},
            "distinct_cone_multisets_with_P_above_1pct": sum(1 for x in by_multi.values() if x > 0.01)}


# ---- tests ---------------------------------------------------------------------------------------
def test_flat_corner_is_index_eight():
    _ballgen()
    from spectre.fold import PatchGraph
    g = PatchGraph(pool().patch, tiles=[0])
    assert int(g.angle[0, FLAT]) == 180 and list(g.angle[0]).count(180) == 1


def test_every_fold_in_a_small_window_has_ledger_720():
    fs = enumerate_folds(window(10))
    assert fs and all(f["ledger"] == 720 and not f["negative"] for f in fs)
    assert all(len(set(t for p in f["pairs"] for t in p)) == 2 * len(f["pairs"]) for f in fs)      # pairs are disjoint


def test_move_graph_edges_are_one_tile_apart():
    fs = enumerate_folds(window(10))
    for i, j, k in move_graph(fs):
        a, b = set(fs[i]["tiles"]), set(fs[j]["tiles"])
        assert (k == "add" and len(b - a) == 1 and not a - b) or (k == "swap" and len(a - b) == 1 and len(b - a) == 1)


def test_quantum_limits():
    fs = enumerate_folds(window(10))
    es = move_graph(fs)
    c = components(len(fs), es)[0]
    q = quantum(fs, es, c, 0.0)
    assert abs(q["E0"] + max(len(fs[i]["pairs"]) for i in c)) < 1e-12
    q = quantum(fs, es, c, 1.0)
    assert abs(sum(q["P_cone_points"].values()) - 1) < 1e-4


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main(n=18):
    W = window(n)
    folds = enumerate_folds(W, verbose=True)
    edges = move_graph(folds)
    comps = components(len(folds), edges)
    out = {"window": W, "subsets_tested": (1 << n) - 1 - n, "folds": len(folds),
           "ledger_all_720": all(f["ledger"] == 720 for f in folds), "negative_deficit_seen": any(f["negative"] for f in folds),
           "by_size": {str(k): v for k, v in sorted(Counter(len(f["tiles"]) for f in folds).items())},
           "deficit_values": {str(k): v for k, v in sorted(Counter(d for f in folds for d in f["deficits"]).items())},
           "cone_points": {str(k): v for k, v in sorted(Counter(len(f["deficits"]) for f in folds).items())},
           "cone_multisets": len({f["deficits"] for f in folds}),
           "pairs": {str(k): v for k, v in sorted(Counter(len(f["pairs"]) for f in folds).items())},
           "pairs_across_the_seam": sum(len(f["pairs"]) - f["planar_pairs"] for f in folds), "pairs_total": sum(len(f["pairs"]) for f in folds),
           "edges": dict(Counter(k for _, _, k in edges)), "components": [len(c) for c in comps[:10]], "n_components": len(comps),
           "moves": move_table(folds, edges)}
    print(json.dumps({k: v for k, v in out.items() if k != "window"}, indent=1), flush=True)
    c = comps[0]
    out["largest_component"] = {"folds": len(c), "sizes": {str(k): v for k, v in sorted(Counter(len(folds[i]["tiles"]) for i in c).items())},
                                "cone_multisets": len({folds[i]["deficits"] for i in c})}
    out["quantum"] = [quantum(folds, edges, c, G) for G in (0.0, 0.2, 1.0, 3.0)] if len(c) <= 6000 else []
    out["quantum_swaps_only"] = []
    sw = [e for e in edges if e[2] == "swap"]
    cs = components(len(folds), sw)
    if cs and 1 < len(cs[0]) <= 6000:
        out["quantum_swaps_only"] = [dict(quantum(folds, sw, cs[0], G, kinds=("swap",)), folds=len(cs[0]), tiles=len(folds[cs[0][0]]["tiles"])) for G in (0.2, 1.0, 3.0)]
    for q in out["quantum"] + out["quantum_swaps_only"]:
        print(q, flush=True)
    out["fold_list"] = [{"tiles": f["tiles"], "deficits": f["deficits"], "pairs": f["pairs"]} for f in folds]
    json.dump(out, open(os.path.join(HERE, "results_folds.json"), "w"))


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main(int(next((a for a in sys.argv[1:] if a.isdigit()), 18)))
