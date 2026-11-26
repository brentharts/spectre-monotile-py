#!/usr/bin/env python3
"""
toymodel_v12, stage S5(b): can curvature move?

**Exact (angle sums).** In the flat-kite metric a tile centre of degree d of the triangulation has cone angle
60 d degrees, deficit 60 (6 - d): the cone points are the tiles that are not hexagons.  They belong to the kite
complex, not to the hats.  So no re-tiling of one kite complex moves curvature; two separate questions remain.

  B1  Same complex GP(3,0), other holes.  Are the holes tied to the cone points?  Exact cover of the 540 kites
      (pentagon kites included) by hats and by *hole tiles* (a whole pentagon, 5 kites, or a whole hexagon, 6
      kites).  Counting: 5 a + 6 b = 4 mod 8 for a pentagon and b hexagon holes.
  B2  Another complex.  Flip one edge of the triangulation (the elementary move that changes tile degrees: the
      two ends lose one, the two opposite vertices gain one).  Away from the pentagons this creates two pentagons
      and two heptagons (deficits +60, +60, -60, -60: the ledger stays 720).  Does the new complex have a hat
      tiling, with the non-hexagons as holes, or with any choice of hole tiles?

A hat that covers kites of a pentagon or heptagon is still a flat hat (every kite is flat, and the cone point is
at a corner of the hat, never inside: a hat has at most 4 kites at a tile centre).

Run:  HEXASPHERE_DIR=... python3 t12_holes.py        (`--tests-only` for the checks)
"""
from __future__ import annotations

import json
import os
import sys
import time
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_pairs_sphere as P      # noqa: E402


def gp30():
    P._hexasphere()
    from hexasphere.hattrick.kites import geodesic_sphere
    V, F = geodesic_sphere(3)
    return np.asarray(V, float), [tuple(int(x) for x in f) for f in F]


def degrees(nv, F):
    d = [0] * nv
    for f in F:
        for v in f:
            d[v] += 1
    return d


def flip_edge(F, a, b):
    """Flip the edge ab of a triangulation given by counter-clockwise triangles.  Returns the new list."""
    F = [tuple(f) for f in F]
    t1 = next(i for i, f in enumerate(F) if any(f[s] == a and f[(s + 1) % 3] == b for s in range(3)))
    t2 = next(i for i, f in enumerate(F) if any(f[s] == b and f[(s + 1) % 3] == a for s in range(3)))
    c = next(v for v in F[t1] if v not in (a, b))
    d = next(v for v in F[t2] if v not in (a, b))
    assert not any(c in f and d in f for f in F), "the flipped edge already exists"
    F[t1], F[t2] = (a, d, c), (d, b, c)
    return F


def is_sphere(nv, F):
    E = {frozenset((f[s], f[(s + 1) % 3])) for f in F for s in range(3)}
    directed = Counter((f[s], f[(s + 1) % 3]) for f in F for s in range(3))
    return nv - len(E) + len(F) == 2 and all(v == 1 for v in directed.values()) and all((b, a) in directed for a, b in directed)


class Cover:
    """Exact cover of all kites of a triangulation by hats and optional hole tiles."""

    def __init__(self, nv, F, hole_tiles, both_hands=False):
        P._hexasphere()
        from hexasphere.hattrick.kites import KiteComplex, embed_hat
        self.F = F
        self.S = KiteComplex(np.zeros((nv, 3)), F, skip=())
        self.kites = list(self.S.kites)
        hats = set()
        for mirror in ((False, True) if both_hands else (False,)):
            for k in self.kites:
                p = embed_hat(self.S, k, mirror)
                if p:
                    hats.add(frozenset(p))
        self.hats = sorted(hats, key=sorted)
        self.tile_kites = {}
        for k in self.kites:
            self.tile_kites.setdefault(F[k // 3][k % 3], []).append(k)
        self.options = [("hat", None, tuple(sorted(h))) for h in self.hats] + \
                       [("hole", v, tuple(sorted(self.tile_kites[v]))) for v in sorted(hole_tiles)]

    def solve(self, max_solutions=10000, max_nodes=3_000_000, max_holes=None, deg=None, stop_at=None):
        """Count solutions.  Returns dict(solutions, exhausted, nodes, by_holes = Counter of sorted hole-degree tuples,
        first = one solution per hole pattern)."""
        Y = {i: o[2] for i, o in enumerate(self.options)}
        X = {k: set() for k in self.kites}
        for i, ks in Y.items():
            for k in ks:
                X[k].add(i)
        nodes, sols = [0], [0]
        by, first = Counter(), {}
        stop = [False]

        def select(r):
            cols = []
            for j in Y[r]:
                for i in X[j]:
                    for k in Y[i]:
                        if k != j:
                            X[k].remove(i)
                cols.append(X.pop(j))
            return cols

        def deselect(r, cols):
            for j in reversed(Y[r]):
                X[j] = cols.pop()
                for i in X[j]:
                    for k in Y[i]:
                        if k != j:
                            X[k].add(i)

        def rec(chosen, nh):
            nodes[0] += 1
            if stop[0] or nodes[0] > max_nodes or sols[0] >= max_solutions:
                stop[0] = True
                return
            if not X:
                sols[0] += 1
                holes = tuple(sorted((deg[self.options[i][1]] if deg else 0) for i in chosen if self.options[i][0] == "hole"))
                by[holes] += 1
                first.setdefault(holes, [self.options[i] for i in chosen])
                if stop_at and sols[0] >= stop_at:
                    stop[0] = True
                return
            c = min(X, key=lambda o: len(X[o]))
            for r in sorted(X[c]):
                h = self.options[r][0] == "hole"
                if h and max_holes is not None and nh >= max_holes:
                    continue
                cols = select(r)
                rec(chosen + [r], nh + h)
                deselect(r, cols)
                if stop[0]:
                    return
        old = sys.getrecursionlimit()
        sys.setrecursionlimit(max(old, 10000))
        try:
            rec([], 0)
        finally:
            sys.setrecursionlimit(old)
        return {"solutions": sols[0], "exhausted": not stop[0], "nodes": nodes[0], "by_holes": {str(k): v for k, v in sorted(by.items())}, "first": first}


def edge_orbits(V, F):
    """One edge of the triangulation per class of (degrees of the 4 vertices of its flip, distance to the nearest pentagon)."""
    nv = len(V)
    deg = degrees(nv, F)
    pent = [v for v in range(nv) if deg[v] == 5]
    X = V / np.linalg.norm(V, axis=1)[:, None]
    seen = {}
    for f in F:
        for s in range(3):
            a, b = f[s], f[(s + 1) % 3]
            if a > b:
                continue
            g = next(t for t in F if any(t[i] == b and t[(i + 1) % 3] == a for i in range(3)))
            c = next(v for v in f if v not in (a, b))
            d = next(v for v in g if v not in (a, b))
            mid = X[a] + X[b]
            mid /= np.linalg.norm(mid)
            key = (tuple(sorted((deg[a], deg[b]))), tuple(sorted((deg[c], deg[d]))), round(float(min(np.linalg.norm(X[p] - mid) for p in pent)), 4))
            seen.setdefault(key, (a, b))
    return seen


# ---- tests -------------------------------------------------------------------------------------
def test_gp30_and_flip_keep_the_sphere():
    V, F = gp30()
    assert is_sphere(len(V), F) and Counter(degrees(len(V), F)) == {6: 80, 5: 12}
    (a, b) = next(iter(edge_orbits(V, F).values()))
    F2 = flip_edge(F, a, b)
    assert is_sphere(len(V), F2) and len(F2) == 180
    d = degrees(len(V), F2)
    assert sum(60 * (6 - x) for x in d) == 720                     # the curvature ledger


def test_pentagon_holes_only_reproduces_4096():
    V, F = gp30()
    deg = degrees(len(V), F)
    pent = [v for v in range(len(V)) if deg[v] == 5]
    # forbid hats on pentagon kites by making the pentagons mandatory holes: the kites of a pentagon are covered by
    # the hole option or by hats, so here only count solutions with 12 holes
    r = Cover(len(V), F, pent).solve(max_solutions=100000, max_nodes=5_000_000, deg=deg)
    assert r["exhausted"]
    assert r["by_holes"].get(str(tuple([5] * 12))) == 4096, r["by_holes"]


def test_cover_solver_on_a_tiny_case():
    class C(Cover):
        def __init__(self):
            self.kites = [0, 1, 2, 3]
            self.options = [("hat", None, (0, 1)), ("hat", None, (2, 3)), ("hat", None, (1, 2)), ("hole", 9, (0,)), ("hole", 8, (3,))]
    r = C().solve()
    assert r["solutions"] == 2 and r["exhausted"]


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main():
    V, F = gp30()
    nv = len(V)
    deg = degrees(nv, F)
    pent = [v for v in range(nv) if deg[v] == 5]
    out = {}
    # B1a: hats may cover pentagon kites; only pentagons may be holes
    c = Cover(nv, F, pent)
    onp = sum(1 for h in c.hats if any(F[k // 3][k % 3] in pent for k in h))
    r = c.solve(max_solutions=100000, max_nodes=5_000_000, deg=deg)
    out["B1a_pentagon_holes_optional"] = {"hat_placements": len(c.hats), "placements_on_pentagon_kites": onp, **{k: v for k, v in r.items() if k != "first"}}
    print("B1a", out["B1a_pentagon_holes_optional"], flush=True)
    # B1b: any tile may be a hole, at most m holes
    for m in (2, 4, 6):
        t = time.time()
        r = Cover(nv, F, range(nv)).solve(max_solutions=20000, max_nodes=3_000_000, max_holes=m, deg=deg)
        out["B1b_any_tile_max_%d_holes" % m] = {k: v for k, v in r.items() if k != "first"}
        print("B1b", m, out["B1b_any_tile_max_%d_holes" % m], round(time.time() - t), "s", flush=True)
    # B1c: both hands
    r = Cover(nv, F, pent, both_hands=True).solve(max_solutions=100000, max_nodes=3_000_000, deg=deg)
    out["B1c_both_hands_pentagon_holes"] = {k: v for k, v in r.items() if k != "first"}
    print("B1c", out["B1c_both_hands_pentagon_holes"], flush=True)
    # B2: one edge flip
    out["B2_edge_flips"] = []
    for key, (a, b) in sorted(edge_orbits(V, F).items()):
        F2 = flip_edge(F, a, b)
        d2 = degrees(nv, F2)
        odd = [v for v in range(nv) if d2[v] != 6]
        row = {"edge_class": str(key), "degrees_after": dict(sorted(Counter(d2).items())), "ledger": sum(60 * (6 - x) for x in d2),
               "free_kites_if_all_non_hexagons_are_holes": 540 - sum(d2[v] for v in odd)}
        c2 = Cover(nv, F2, odd)
        r = c2.solve(max_solutions=100000, max_nodes=3_000_000, deg=d2)
        row["non_hexagon_holes_optional"] = {k: v for k, v in r.items() if k != "first"}
        r = Cover(nv, F2, range(nv)).solve(max_solutions=1, max_nodes=3_000_000, max_holes=16, deg=d2, stop_at=1)
        row["any_holes_up_to_16"] = {k: v for k, v in r.items() if k != "first"}
        r = Cover(nv, F2, odd, both_hands=True).solve(max_solutions=1, max_nodes=3_000_000, deg=d2, stop_at=1)
        row["both_hands"] = {k: v for k, v in r.items() if k != "first"}
        out["B2_edge_flips"].append(row)
        print("B2", row, flush=True)
    # B1d: any tile may be a hole, up to 16 holes (12 pentagons and up to 4 hexagons would fit the count)
    r = Cover(nv, F, range(nv)).solve(max_solutions=100000, max_nodes=5_000_000, max_holes=16, deg=deg)
    out["B1d_any_tile_max_16_holes"] = {k: v for k, v in r.items() if k != "first"}
    print("B1d", out["B1d_any_tile_max_16_holes"], flush=True)
    # B2 all: every one of the 270 edges, one hand, any holes up to 16
    E = sorted({(min(f[s], f[(s + 1) % 3]), max(f[s], f[(s + 1) % 3])) for f in F for s in range(3)})
    found, exhausted, nodes = 0, True, 0
    for a, b in E:
        F2 = flip_edge(F, a, b)
        r = Cover(nv, F2, range(nv)).solve(max_solutions=1, max_nodes=1_000_000, max_holes=16, deg=degrees(nv, F2), stop_at=1)
        found += r["solutions"]
        exhausted &= r["exhausted"] or r["solutions"] > 0
        nodes = max(nodes, r["nodes"])
    out["B2_all_edges"] = {"edges": len(E), "flips_with_a_tiling": found, "all_searches_exhausted": exhausted, "max_nodes": nodes}
    print("B2 all", out["B2_all_edges"], flush=True)
    # B3: random sequences of 2 to 6 flips (a sample of other triangulations with the same 92 tiles)
    import random
    rng = random.Random(12)
    rows = Counter()
    hit = []
    for trial in range(600):
        k = rng.choice([2, 3, 4, 6])
        F2 = list(F)
        ok = True
        for _ in range(k):
            E2 = sorted({(f[s], f[(s + 1) % 3]) for f in F2 for s in range(3)})
            a, b = rng.choice(E2)
            try:
                F2 = flip_edge(F2, a, b)
            except AssertionError:
                ok = False
                break
        d2 = degrees(nv, F2)
        if not ok or min(d2) < 3 or not is_sphere(nv, F2):
            rows[(k, "skipped")] += 1
            continue
        r = Cover(nv, F2, range(nv)).solve(max_solutions=1, max_nodes=300_000, max_holes=20, deg=d2, stop_at=1)
        rows[(k, "tiling" if r["solutions"] else ("none" if r["exhausted"] else "unexhausted"))] += 1
        if r["solutions"]:
            hit.append({"flips": k, "degrees": dict(sorted(Counter(d2).items()))})
    out["B3_random_flip_sequences"] = {"counts": {str(k): v for k, v in sorted(rows.items())}, "hits": hit[:20]}
    print("B3", out["B3_random_flip_sequences"], flush=True)
    json.dump(out, open(os.path.join(HERE, "results_holes.json"), "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
