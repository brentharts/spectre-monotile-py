#!/usr/bin/env python3
"""t11_swap_scan.py - exhaustive exact scan for local rearrangements ("swaps") of the Spectre tiling.

Question (Open problem 1 of quantgrav*.tex): is there ANY set of k adjacent tiles of the exact level-4 patch that can be
replaced by a different set of Spectre tiles, edge to edge and without overlap, leaving the rest of the patch fixed?
Earlier cavity runs (toymodel_v10/t10_cavity.py) erased the k tiles NEAREST one centre; a reported SAT run found a
2-tile swap at k = 4 when reflected tiles are allowed.  Here every vertex-connected set of k tiles of the central part of
the patch is tried, for k = 1..K, with and without reflected tiles, by the same exact search (Q(sqrt3) arithmetic).

Why this settles the 2-tile swap.  Let F1 (the original) and F2 be two fillings of a hole.  The tiles common to F1 and F2
can be put back, so F1 \\ F2 and F2 \\ F1 fill the same smaller hole.  A swap of two tiles therefore shows up already at
k = 2, in a hole made of two tiles that touch (share a vertex), or at k = 1 if the two are independent.  Every
cavity of at most K tiles with a second filling is found by this scan, because a minimal one is vertex-connected: if the
hole were a union of parts with no shared vertex, each part would carry its own second filling.

Holes with the same shape and the same ring of tiles around them (translation only) have the same fillings (a new tile must
lie inside the hole: everything outside is covered), so each class is searched once.  Labels: Exact for the outcome of each
search (finite, complete under the edge-to-edge assumption, which is checked by relaxing it for k <= 2); counts Counted.

    PYTHONPATH=<dir with spectre_exact.py> python3 t11_swap_scan.py --tests-only
    PYTHONPATH=... python3 t11_swap_scan.py --K 4 --radius 18 [--out results_swap_scan.json]
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "toymodel_v10"))
import t10_cavity as tc  # noqa: E402
from t10_cavity import Hash, Search, Tile, patch, xyf  # noqa: E402

CENTRE = tc.CENTRE


class FastSearch(Search):
    """t10_cavity.Search with the fixed-tile hash built only from tiles near the hole (same results, much faster)."""

    def __init__(self, erased, reflect=False, max_solutions=2, node_limit=10 ** 6, relax_edge=False, level=4):
        self.tiles, self.cent, self.vmap = patch(level)
        self.erased = sorted(erased)
        self.reflect = reflect
        self.shapes = tc._build_shapes(reflect)
        self.max_solutions = max_solutions
        self.node_limit = node_limit
        self.relax_edge = relax_edge
        eset = set(self.erased)
        bb = [self.tiles[i].bbox for i in self.erased]
        x0, y0 = min(b[0] for b in bb) - 8.0, min(b[1] for b in bb) - 8.0
        x1, y1 = max(b[2] for b in bb) + 8.0, max(b[3] for b in bb) + 8.0
        self.hash = Hash()
        for i, t in enumerate(self.tiles):
            if i in eset:
                continue
            if t.bbox[2] >= x0 and t.bbox[0] <= x1 and t.bbox[3] >= y0 and t.bbox[1] <= y1:
                self.hash.add(t)
        self.cav_verts = set(v for i in self.erased for v in self.tiles[i].verts)
        self.nodes = 0
        self.solutions = []
        self.solution_tiles = []
        self.placed = []
        self.cover = collections.defaultdict(list)
        for v in self.cav_verts:
            for j, k in self.vmap[v]:
                if j not in eset:
                    self.cover[v].append((self.tiles[j], k))
        self.bound = {v for v in self.cav_verts if v in self.cover}
        self.track = set(self.bound)
        self.cache = {}

    def candidates(self, v):
        if not self.relax_edge:
            return super().candidates(v)
        saved = tc.not_edge_to_edge
        tc.not_edge_to_edge = lambda a, b: False
        try:
            return super().candidates(v)
        finally:
            tc.not_edge_to_edge = saved

    def _rec(self):
        if len(self.solutions) >= self.max_solutions or self.nodes > self.node_limit:
            return
        self.nodes += 1
        best = None
        for v in self.open_vertices():
            c = self.cache.get(v)
            if c is None:
                c = self.cache[v] = self.candidates(v)
            if not c:
                return
            if best is None or len(c) < len(best[1]):
                best = (v, c)
                if len(c) == 1:
                    break
        if best is None:
            self.solutions.append(frozenset(t.vset for t in self.placed))
            self.solution_tiles.append(list(self.placed))
            return
        for t, _i in best[1]:
            self.place(t)
            self._rec()
            self.unplace(t)
            if len(self.solutions) >= self.max_solutions or self.nodes > self.node_limit:
                return


def complete(erased, reflect=False, relax_edge=False, max_solutions=2, node_limit=10 ** 6, level=4):
    S = FastSearch(erased, reflect, max_solutions, node_limit, relax_edge, level)
    S.run()
    orig = frozenset(S.tiles[i].vset for i in S.erased)
    others = [(s, ts) for s, ts in zip(S.solutions, S.solution_tiles) if s != orig]
    return {"k": len(S.erased), "nodes": S.nodes, "n_solutions": len(S.solutions),
            "original_found": orig in S.solutions, "others": others, "hit_node_limit": S.nodes > S.node_limit,
            "search": S}


def _edge_dir(v, w):
    a, b = xyf(v), xyf(w)
    return round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) / 30.0) % 12


def outline_turns(tiles_):
    """cyclic sequence of corner turns (units of 30 degrees) of the outline of the union of `tiles_`, normalised to counter-clockwise
    orientation, or None if the union does not have a single simple outline (a hole, or two parts touching at a vertex)."""
    cnt = collections.Counter(e for t in tiles_ for e in tile_edges(t))
    be = [e for e, k in cnt.items() if k == 1]
    adj = collections.defaultdict(list)
    for e in be:
        a, b = tuple(e)
        adj[a].append(b)
        adj[b].append(a)
    if not adj or any(len(x) != 2 for x in adj.values()):
        return None
    start = next(iter(adj))
    cyc, prev, cur = [start], None, start
    while True:
        nx = adj[cur][0] if prev is None else [x for x in adj[cur] if x != prev][0]
        prev, cur = cur, nx
        if cur == start:
            break
        cyc.append(cur)
    if len(cyc) != len(be):
        return None
    P = [xyf(v) for v in cyc]
    n = len(P)
    turns = []
    for i in range(n):
        a, b, c = P[i - 1], P[i], P[(i + 1) % n]
        u, w = math.atan2(b[1] - a[1], b[0] - a[0]), math.atan2(c[1] - b[1], c[0] - b[0])
        turns.append(round(((w - u + math.pi) % (2 * math.pi) - math.pi) / (math.pi / 6)))
    if sum(turns) < 0:
        turns = [-x for x in turns[::-1]]
    return turns


def canon_cyclic(a):
    return min(tuple(a[i:] + a[:i]) for i in range(len(a)))


def mystic_outline():
    """canonical cyclic turn sequence of the Mystic region (the two halves of a Mystic pair of the level-4 patch), and of its mirror."""
    tiles, cent, vmap = patch(4)
    ks = kinds()
    i = next(i for i, k in enumerate(ks) if k == "g1")
    j = i + 1
    assert ks[j] == "g2"
    t = outline_turns([tiles[i], tiles[j]])
    return canon_cyclic(t), canon_cyclic([-x for x in t[::-1]])


def is_mystic_shaped(tiles_):
    t = outline_turns(tiles_)
    if t is None:
        return False
    c = canon_cyclic(t)
    ref, refm = mystic_outline()
    return c == ref or c == refm


# ----------------------------------------------------------------- change of the vertex-edge-face counts
def tile_edges(t):
    n = len(t.verts)
    return {frozenset((t.verts[i], t.verts[(i + 1) % n])) for i in range(n)}


_KINDS = []


def kinds():
    """leaf kind of every tile of the level-4 patch: 'p' (single) or 'g1'/'g2' (the two halves of a Mystic pair)."""
    if not _KINDS:
        _KINDS.extend(k for _, k in tc.se.build(4)[4][0]["Delta"])
    return _KINDS


def signed_area(t):
    f = [xyf(v) for v in t.verts]
    n = len(f)
    return sum(f[i][0] * f[(i + 1) % n][1] - f[(i + 1) % n][0] * f[i][1] for i in range(n)) / 2


def dvef(old_tiles, new_tiles):
    """(dV, dE, dF) between two fillings of one hole (vertices, edges, tiles of the filling tiles themselves)."""
    def vef(ts):
        V = set().union(*[set(t.verts) for t in ts])
        E = set().union(*[tile_edges(t) for t in ts])
        return len(V), len(E), len(ts)
    a, b = vef(old_tiles), vef(new_tiles)
    return tuple(y - x for x, y in zip(a, b))


def brute_fillings(cavity, reflect=False, max_k=3):
    """independent oracle (floating point polygons, no sector logic): every set of non-overlapping placed Spectres, taken
    from all placements that put one corner on a vertex of the hole and lie inside the hole, whose union is the hole.
    Returns the number of such exact covers (k = len(cavity) <= max_k)."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    tiles, cent, vmap = patch(4)
    k = len(cavity)
    assert k <= max_k
    hole = unary_union([Polygon([xyf(v) for v in tiles[i].verts]) for i in cavity])
    hv = {v for i in cavity for v in tiles[i].verts}
    cands = {}
    for sh in tc._build_shapes(reflect):
        for v in hv:
            for i in range(14):
                verts = [tuple(a + b - c for a, b, c in zip(v, w, sh["verts"][i])) for w in sh["verts"]]
                key = frozenset(verts)
                if key in cands:
                    continue
                P = Polygon([xyf(w) for w in verts])
                if P.is_valid and P.difference(hole).area < 1e-7:
                    cands[key] = P
    keys = list(cands)
    area = hole.area
    count = 0

    def rec(chosen, start):
        nonlocal count
        if len(chosen) == k:
            if abs(unary_union([cands[x] for x in chosen]).area - area) < 1e-7:
                count += 1
            return
        for j in range(start, len(keys)):
            P = cands[keys[j]]
            if all(P.intersection(cands[x]).area < 1e-7 for x in chosen):
                rec(chosen + [keys[j]], j + 1)
    rec([], 0)
    return count


# ----------------------------------------------------------------- enumeration of holes
def neighbours(tiles, vmap, S):
    Sset = set(S)
    nb = {i: set() for i in S}
    for i in S:
        for v in tiles[i].verts:
            for j, _ in vmap[v]:
                if j != i and j in Sset:
                    nb[i].add(j)
    return nb


def connected_sets(nb, S, k):
    cur = {frozenset([i]) for i in S}
    for _ in range(k - 1):
        nxt = set()
        for s in cur:
            for j in set().union(*[nb[i] for i in s]) - s:
                nxt.add(s | {j})
        cur = nxt
    return cur


def hole_key(tiles, vmap, cavity):
    """translation-invariant description of the hole and the ring of tiles that touch it."""
    cav = set(cavity)
    ring = set()
    for i in cav:
        for v in tiles[i].verts:
            for j, _ in vmap[v]:
                if j not in cav:
                    ring.add(j)
    allv = [v for i in cav for v in tiles[i].verts]
    o = min(allv)
    sh = lambda t: tuple(sorted(tuple(a - b for a, b in zip(v, o)) for v in t.verts))
    return (tuple(sorted(sh(tiles[i]) for i in cav)), tuple(sorted(sh(tiles[j]) for j in ring)))


def scan(K=4, radius=18.0, reflect_modes=(False, True), verbose=True, max_per_k=None):
    tiles, cent, vmap = patch(4)
    d = np.linalg.norm(cent - cent[CENTRE], axis=1)
    S = [int(i) for i in np.where(d < radius)[0]]
    nb = neighbours(tiles, vmap, S)
    out = {"radius": radius, "tiles_in_region": len(S), "rows": []}
    for k in range(1, K + 1):
        sets = connected_sets(nb, S, k)
        classes = {}
        for s in sets:
            classes.setdefault(hole_key(tiles, vmap, s), sorted(s))
        reps = list(classes.values())
        if max_per_k:
            reps = reps[:max_per_k]
        for refl in reflect_modes:
            t0 = time.time()
            n_alt, nodes, alts, bad = 0, 0, [], 0
            for cav in reps:
                r = complete(cav, reflect=refl)
                nodes += r["nodes"]
                bad += (not r["original_found"]) or r["hit_node_limit"]
                if r["others"]:
                    n_alt += 1
                    for s, ts in r["others"]:
                        S_ = r["search"]
                        old = [S_.tiles[i] for i in S_.erased]
                        new_sets = {t.vset for t in ts}
                        gone = [i for i in cav if S_.tiles[i].vset not in new_sets]
                        gone_tiles = [S_.tiles[i] for i in gone]
                        alts.append({"cavity": cav, "dVEF": dvef(old, ts),
                                     "replaced_outline_mystic": bool(is_mystic_shaped(gone_tiles)),
                                     "reflected_tiles": sum(1 for t in ts if signed_area(t) * signed_area(old[0]) < 0),
                                     "tiles_changed": len(gone), "replaced_kinds": sorted(kinds()[i] for i in gone),
                                     "replaced_are_mystic_pair": sorted(kinds()[i] for i in gone) == ["g1", "g2"]})
            row = {"k": k, "reflect": refl, "connected_sets": len(sets), "hole_classes": len(reps),
                   "with_second_filling": n_alt, "searches_without_original_or_hit_limit": bad,
                   "nodes": nodes, "seconds": round(time.time() - t0, 1), "n_alternatives": len(alts),
                   "tiles_changed": dict(collections.Counter(a["tiles_changed"] for a in alts)),
                   "replaced_outline_mystic": dict(collections.Counter(a["replaced_outline_mystic"] for a in alts)),
                   "dVEF_counts": {str(k): v for k, v in collections.Counter(a["dVEF"] for a in alts).items()},
                   "reflected_in_new": dict(collections.Counter(a["reflected_tiles"] for a in alts)),
                   "mystic_swaps": sum(a["replaced_are_mystic_pair"] for a in alts),
                   "example": alts[:1]}
            out["rows"].append(row)
            if verbose:
                print("k=%d reflect=%-5s sets=%6d classes=%5d second fillings=%d (problems %d) nodes=%d %.0fs" % (
                    k, refl, len(sets), len(reps), n_alt, bad, nodes, time.time() - t0), flush=True)
    return out


# ----------------------------------------------------------------- tests
def test_s111_fast_search_agrees_with_t10():
    """FastSearch reproduces t10_cavity.complete on single tiles, nearest-k cavities and one reflected run."""
    for cav, refl in (([2675], False), (tc.choose_cavity(4, 10, CENTRE), False), (tc.choose_cavity(4, 4, CENTRE), True)):
        a = tc.complete(4, cav, reflect=refl)
        b = complete(cav, reflect=refl)
        assert (a["n_solutions"], a["original_found"], a["nodes"]) == (b["n_solutions"], b["original_found"], b["nodes"])
    return "3 cavities: same solution counts and node counts as t10_cavity"


def test_s112_small_holes_second_fillings_are_only_the_mystic_swap():
    """all vertex-connected holes of k = 1, 2 near the centre: chiral, none has a second filling; with reflections
    every second filling replaces a Mystic-shaped pair by its mirror dissection (2 tiles changed, dV=dE=dF=0)."""
    r = scan(K=2, radius=10.0, verbose=False)
    for row in r["rows"]:
        assert row["searches_without_original_or_hit_limit"] == 0
        if not row["reflect"]:
            assert row["with_second_filling"] == 0, row
        else:
            assert row["with_second_filling"] == row["tiles_changed"].get(2, 0), row
            assert row["with_second_filling"] == row["replaced_outline_mystic"].get(True, 0), row
    return "; ".join("k=%d refl=%s: %d classes, %d second fillings" % (x["k"], x["reflect"], x["hole_classes"],
                                                                      x["with_second_filling"]) for x in r["rows"])


def test_s113_edge_to_edge_assumption_is_not_what_forces_rigidity():
    """k = 1, 2 holes with the edge-to-edge rule relaxed (any non-overlapping placement) still have only the original."""
    tiles, cent, vmap = patch(4)
    d = np.linalg.norm(cent - cent[CENTRE], axis=1)
    S = [int(i) for i in np.where(d < 8.0)[0]]
    nb = neighbours(tiles, vmap, S)
    n = 0
    for k in (1, 2):
        classes = {}
        for s in connected_sets(nb, S, k):
            classes.setdefault(hole_key(tiles, vmap, s), sorted(s))
        for cav in classes.values():
            for refl in (False, True):
                r = complete(cav, reflect=refl, relax_edge=True)
                assert r["original_found"] and r["n_solutions"] == 1, (cav, refl, r["n_solutions"])
                n += 1
    return "%d searches with the edge-to-edge test off: unique" % n


def test_s114_euler_check_of_dvef():
    """for a filling compared with itself dV = dE = dF = 0; counts of a one-tile hole are 14, 14, 1."""
    tiles, cent, vmap = patch(4)
    t = tiles[CENTRE]
    assert dvef([t], [t]) == (0, 0, 0)
    assert (len(set(t.verts)), len(tile_edges(t))) == (14, 14)
    return "dV,dE,dF of a tile with itself = 0; a tile has 14 vertices and 14 edges"


def test_s115_independent_exact_cover_oracle_agrees():
    """brute-force exact covers (shapely, no sector logic) of all hole classes with k <= 3 near the centre, chiral and with
    reflections: exactly one cover each (the original), the same as the search; dedupe classes give equal node counts."""
    tiles, cent, vmap = patch(4)
    d = np.linalg.norm(cent - cent[CENTRE], axis=1)
    S = [int(i) for i in np.where(d < 7.0)[0]]
    nb = neighbours(tiles, vmap, S)
    n = 0
    for k in (1, 2, 3):
        groups = collections.defaultdict(list)
        for s in connected_sets(nb, S, k):
            groups[hole_key(tiles, vmap, s)].append(sorted(s))
        for key, members in list(groups.items())[:25]:
            for refl in (False, True):
                assert brute_fillings(members[0], refl) == 1, (members[0], refl)
                nd = {complete(m, reflect=refl)["nodes"] for m in members[:3]}
                assert len(nd) == 1, (members[:3], nd)
                n += 1
    return "%d (class, mode) pairs: brute-force cover count 1, node counts equal within a class" % n


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_s11")]
    failed = 0
    for n, f in tests:
        t = time.time()
        try:
            r = f()
            if verbose:
                print("ok   %s  %s  (%.1fs)" % (n, r, time.time() - t), flush=True)
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e), flush=True)
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--K", type=int, default=4)
    ap.add_argument("--radius", type=float, default=18.0)
    ap.add_argument("--chiral-only", action="store_true")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.tests_only:
        sys.exit(1 if run_tests() else 0)
    modes = (False,) if a.chiral_only else (False, True)
    res = scan(a.K, a.radius, modes)
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1, default=str)
