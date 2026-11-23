#!/usr/bin/env python3
"""t11_flip_exact.py - the point-cloud flips of toymodel_v8, rebuilt as exact tilings and refilled exactly.

Open problem 1 asks whether the flips reported in Section "Phason flips" are closed rearrangements, i.e. tilings.  The six flips
exported in toymodel_v8/flip_examples.json (class 0, level 3, delta = 0.03, every tile that meets the disc of radius 4.5 about
the origin tile) are taken here as data and treated exactly:

  1. every vertex is rounded to Z[zeta_12] (an integer 4-vector; all 454 vertices of the first flip round exactly, residual below
     1e-7), every 14-gon is identified with a rotation of the Spectre and every 20-gon (a Mystic pair) is cut into its two Spectres;
  2. each tiling (before = unchanged + removed, after = unchanged + added) is tested exactly for overlap and edge-to-edge contact,
     and its union is compared with the disc for holes (floating point, area);
  3. removed and added tiles that overlap are linked into clusters; a cluster is CLOSED iff the boundary edges of its removed
     tiles equal those of its added tiles (exact: sets of pairs of integer vertices);  the change (dV, dE, dF) of a closed cluster
     is reported;
  4. the before and the after configuration are looked up in the exact level-4 patch (12 rotations, all translations, vertex sets
     compared exactly): a configuration that is absent is not a patch of this tiling class;
  5. the removed tiles of each flip are erased in the patch and the hole refilled by the exact search, with and without reflected
     tiles.

Everything here is exact for what is in the file: the six flips are a random sample of 416 valid events, and each lists only the
tiles that touch the disc, so a cluster that leaves the disc is open by construction (no verdict about its outside).

    PYTHONPATH=<dir with spectre_exact.py> python3 t11_flip_exact.py --tests-only
    PYTHONPATH=... python3 t11_flip_exact.py [--out results_flip_exact.json]
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
sys.path.insert(0, _HERE)
import t10_cavity as tc  # noqa: E402
import t11_swap_scan as sw  # noqa: E402
from t10_cavity import Tile, xyf  # noqa: E402

se = tc.se
S3 = math.sqrt(3.0)
EXAMPLES = os.path.join(_HERE, "..", "toymodel_v8", "flip_examples.json")

# ----------------------------------------------------------------- exact coordinates
_SHAPES = tc._build_shapes(True)


def _norm(vs):
    m = min(vs)
    return tuple(sorted(tuple(a - b for a, b in zip(v, m)) for v in vs)), m


_NORM2SHAPE = {}
for _sh in _SHAPES:
    _n, _m = _norm(_sh["verts"])
    _NORM2SHAPE[_n] = (_sh, _m)


def to4(x, y, tol=1e-7):
    """(x, y) in (1/2) Z[sqrt3] x (1/2) Z[sqrt3]  ->  integer 4-vector of Z[zeta_12], or None."""
    def rec(v):
        for B in range(-400, 401):
            A = round(2 * v - B * S3)
            if abs(A + B * S3 - 2 * v) < tol:
                return A, B
        return None
    rx, ry = rec(x), rec(y)
    if rx is None or ry is None:
        return None
    (Ax, Bx), (Ay, By) = rx, ry
    q, r = By, Bx
    if (Ax - q) % 2 or (Ay - r) % 2:
        return None
    return ((Ax - q) // 2, q, r, (Ay - r) // 2)


def tile_from_vset(vs):
    """Tile (shape and translation) whose vertex set is vs, or None if vs is not a Spectre or a reflected Spectre."""
    n, m = _norm(vs)
    hit = _NORM2SHAPE.get(n)
    if hit is None:
        return None
    sh, msh = hit
    d = tuple(a - b for a, b in zip(m, msh))
    return Tile([tuple(a + b for a, b in zip(v, d)) for v in sh["verts"]], sh), sh["refl"]


def split_mystic(poly_pts):
    """a 20-gon (union of two Spectres, polygon given as exact 4-vectors) -> its two Spectres, or None; unique cover required."""
    from shapely.geometry import Polygon
    P = Polygon([xyf(v) for v in poly_pts])
    cands = {}
    for sh in tc._build_shapes(False):
        for v in poly_pts:
            for i in range(14):
                verts = [tuple(a + b - c for a, b, c in zip(v, w, sh["verts"][i])) for w in sh["verts"]]
                key = frozenset(verts)
                if key in cands:
                    continue
                Q = Polygon([xyf(w) for w in verts])
                if Q.is_valid and Q.difference(P).area < 1e-7:
                    cands[key] = (Q, Tile(verts, sh))
    keys = list(cands)
    out = []
    for a in range(len(keys)):
        for b in range(a + 1, len(keys)):
            Qa, Qb = cands[keys[a]][0], cands[keys[b]][0]
            if Qa.intersection(Qb).area < 1e-7 and abs(Qa.area + Qb.area - P.area) < 1e-7:
                out.append((cands[keys[a]][1], cands[keys[b]][1]))
    return out


def mirror4(p):
    """the reflection of the lift (spectre_exact.R) on an integer 4-vector; None stays None."""
    return None if p is None else tuple(int(x) for x in se.apply(se.R, np.array(p, dtype=object)))


def load_flips(path=EXAMPLES, mirror=True):
    """The engine of toymodel_v8 works on a level-3 patch, whose tiles are the mirror images of the tiles of the level-4 patch
    (odd levels have the opposite handedness).  mirror=True reflects every vertex (exactly) so that the flips have the
    handedness of tc.patch(4); the reflection changes no incidence, overlap or area."""
    d = json.load(open(path))
    flips = []
    mp = mirror4 if mirror else (lambda p: p)
    for e in d["examples"]:
        rec = {"sample": e["sample"], "n_removed_global": e["n_removed"], "n_added_global": e["n_added"],
               "origin": [mp(to4(x, y)) for x, y in e["origin_xy"][:-1]], "R": d["R"], "tiles": []}
        for t in e["tiles"]:
            pts = [mp(to4(x, y)) for x, y in t["xy"][:-1]]
            rec["tiles"].append({"status": t["status"], "G": t["G"], "pts": pts, "xy": t["xy"]})
        flips.append(rec)
    return flips


def exactify(flip):
    """-> dict with exact Spectre tiles per status ('both', 'removed', 'added'), a log of problems."""
    out = {"both": [], "removed": [], "added": []}
    log = {"unrounded_vertices": 0, "reflected": 0, "unmatched": 0, "mystic_split_ambiguous": 0, "mystic": 0}
    for t in flip["tiles"]:
        if any(p is None for p in t["pts"]):
            log["unrounded_vertices"] += sum(p is None for p in t["pts"])
            continue
        if len(t["pts"]) == 14:
            r = tile_from_vset(t["pts"])
            if r is None:
                log["unmatched"] += 1
                continue
            tile, refl = r
            log["reflected"] += int(refl)
            out[t["status"]].append(tile)
        else:
            sp = split_mystic(t["pts"])
            if len(sp) != 1:
                log["mystic_split_ambiguous"] += 1
                if not sp:
                    log["unmatched"] += 1
                    continue
            log["mystic"] += 1
            out[t["status"]].extend(sp[0])
    return out, log


# ----------------------------------------------------------------- validity, clusters, closure
def exact_valid(tiles):
    """no interior overlap and edge to edge, exactly, for every pair of nearby tiles."""
    h = tc.Hash()
    for t in tiles:
        h.add(t)
    for t in tiles:
        for o in h.near(t.bbox):
            if o is not t and (tc.tiles_overlap(t, o) or tc.not_edge_to_edge(t, o)):
                return False
    return True


def edges_of(t):
    n = len(t.verts)
    return [frozenset((t.verts[i], t.verts[(i + 1) % n])) for i in range(n)]


def boundary_edges(tiles):
    c = collections.Counter(e for t in tiles for e in edges_of(t))
    return {e for e, k in c.items() if k == 1}


def vef(tiles):
    V = set().union(*[set(t.verts) for t in tiles]) if tiles else set()
    E = {e for t in tiles for e in edges_of(t)}
    return len(V), len(E), len(tiles)


def clusters(rem, add):
    """connected components of the graph that links a removed and an added tile when their interiors overlap."""
    from shapely.geometry import Polygon
    P = {("r", i): Polygon([xyf(v) for v in t.verts]) for i, t in enumerate(rem)}
    P.update({("a", i): Polygon([xyf(v) for v in t.verts]) for i, t in enumerate(add)})
    nodes = list(P)
    adj = {n: set() for n in nodes}
    for a in nodes:
        if a[0] != "r":
            continue
        for b in nodes:
            if b[0] == "a" and P[a].intersection(P[b]).area > 1e-7:
                adj[a].add(b)
                adj[b].add(a)
    seen, comps = set(), []
    for n in nodes:
        if n in seen:
            continue
        comp, stack = [], [n]
        seen.add(n)
        while stack:
            x = stack.pop()
            comp.append(x)
            for y in adj[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        comps.append(comp)
    return [([rem[i] for k, i in c if k == "r"], [add[i] for k, i in c if k == "a"]) for c in comps]


def cluster_report(rem, add):
    closed = boundary_edges(rem) == boundary_edges(add) and len(rem) == len(add)
    r = {"removed": len(rem), "added": len(add), "closed": bool(closed)}
    if closed:
        a, b = vef(rem), vef(add)
        r["dVEF"] = tuple(y - x for x, y in zip(a, b))
    return r


def disc_hole_area(tiles, flip):
    """area of the disc (origin tile enlarged by R) not covered by the tiles, and of overlaps (floating point)."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    O = Polygon([xyf(v) for v in flip["origin"]])
    reg = O.buffer(flip["R"])
    pol = [Polygon([xyf(v) for v in t.verts]) for t in tiles]
    U = unary_union(pol)
    return reg.difference(U).area, reg.area


# ----------------------------------------------------------------- lookup in the exact patch
class PatchIndex:
    def __init__(self, level=4):
        self.tiles, self.cent, self.vmap = tc.patch(level)
        self.vs2idx = {t.vset: i for i, t in enumerate(self.tiles)}
        self.norm2idx = collections.defaultdict(list)
        for i, t in enumerate(self.tiles):
            self.norm2idx[_norm(t.vset)[0]].append(i)

    def closed_vertices(self):
        """vertices of the patch around which the incident tiles cover the full angle (12 units of 30 degrees)."""
        if not hasattr(self, "_closed"):
            self._closed = {v for v, lst in self.vmap.items()
                            if sum(self.tiles[i].ang[j] for i, j in lst) == 12}
        return self._closed

    def interior(self, idx):
        """True if every vertex of every tile in idx is surrounded by the patch on all sides (so a hole made of them is enclosed
        and the search is complete there)."""
        cl = self.closed_vertices()
        return all(v in cl for i in idx for v in self.tiles[i].verts)

    def find(self, config, first=0, max_hits=3):
        """all placements (rotation r, translation) of the configuration (list of Tile) onto tiles of the patch:
        returns [(r, [patch index of each tile])]."""
        hits = []
        for r in range(12):
            T = se.trot(r)
            rot = [frozenset(tuple(int(x) for x in se.apply(T, v)) for v in t.verts) for t in config]
            n, m = _norm(rot[first])
            for pi in self.norm2idx.get(n, ()):
                d = tuple(a - b for a, b in zip(min(self.tiles[pi].vset), m))
                idx = []
                for vs in rot:
                    key = frozenset(tuple(a + b for a, b in zip(v, d)) for v in vs)
                    j = self.vs2idx.get(key)
                    if j is None:
                        break
                    idx.append(j)
                else:
                    hits.append((r, idx))
                    if len(hits) >= max_hits:
                        return hits
        return hits


def origin_index(flip, tiles):
    O = frozenset(flip["origin"])
    for i, t in enumerate(tiles):
        if t.vset == O:
            return i
    return 0


# ----------------------------------------------------------------- the analysis of one flip
def analyse(flip, pidx, refill=True, verbose=False):
    ex, log = exactify(flip)
    before, after = ex["both"] + ex["removed"], ex["both"] + ex["added"]
    res = {"sample": flip["sample"], "exactify": log,
           "tiles": {k: len(v) for k, v in ex.items()},
           "valid_before": exact_valid(before), "valid_after": exact_valid(after)}
    hb, area = disc_hole_area(before, flip)
    ha, _ = disc_hole_area(after, flip)
    res["disc_uncovered_before"], res["disc_uncovered_after"] = round(hb, 9), round(ha, 9)
    cl = clusters(ex["removed"], ex["added"])
    res["clusters"] = [cluster_report(r, a) for r, a in cl]
    res["n_closed"] = sum(c["closed"] for c in res["clusters"])
    o = origin_index(flip, before)
    hb_ = pidx.find(before, first=o, max_hits=400)
    ha_ = pidx.find(after, first=origin_index(flip, after), max_hits=400)
    res["before_occurrences"], res["after_occurrences"] = len(hb_), len(ha_)
    res["before_in_patch"], res["after_in_patch"] = bool(hb_), bool(ha_)
    inner = [h for h in hb_ if pidx.interior(h[1])]
    res["before_interior_occurrences"] = len(inner)
    if refill and inner:
        removed_ids = {id(t) for t in ex["removed"]}
        rem_pos = [i for i in range(len(before)) if id(before[i]) in removed_ids]
        for refl in (False, True):
            runs = []
            for r, idx in inner[:3]:
                rem_idx = [idx[i] for i in rem_pos]
                c = sw.complete(rem_idx, reflect=refl)
                runs.append({"erased": len(rem_idx), "fillings": c["n_solutions"], "original_found": c["original_found"],
                             "nodes": c["nodes"]})
            res["refill_%s" % ("reflect" if refl else "chiral")] = runs
    return res


# ----------------------------------------------------------------- tests
_PIDX = []


def pidx():
    if not _PIDX:
        _PIDX.append(PatchIndex(4))
    return _PIDX[0]


def test_f111_all_vertices_round_exactly():
    flips = load_flips()
    n = sum(len(t["pts"]) for f in flips for t in f["tiles"])
    bad = sum(p is None for f in flips for t in f["tiles"] for p in t["pts"])
    assert bad == 0, bad
    for f in load_flips(mirror=False):
        for t in f["tiles"]:
            for p, (x, y) in zip(t["pts"], t["xy"]):
                fx, fy = xyf(p)
                assert abs(fx - x) < 1e-7 and abs(fy - y) < 1e-7
    return "%d vertices of %d tiles (6 flips) round to Z[zeta_12] with residual below 1e-7" % (n, sum(len(f["tiles"]) for f in flips))


def test_f112_tiles_are_unreflected_spectres_and_mystic_pairs_split_uniquely():
    flips = load_flips()
    tot = collections.Counter()
    for f in flips:
        ex, log = exactify(f)
        assert log["unmatched"] == 0 and log["reflected"] == 0 and log["unrounded_vertices"] == 0, log
        assert log["mystic_split_ambiguous"] == 0, log
        tot.update(log)
        tot["tiles"] += sum(len(v) for v in ex.values())
    return "all tiles are unreflected Spectres (%d Spectres from %d Mystic 20-gons, each split uniquely)" % (tot["tiles"], tot["mystic"])


def test_f113_before_and_after_are_exact_tilings_of_the_disc():
    flips = load_flips()
    for f in flips:
        ex, _ = exactify(f)
        before, after = ex["both"] + ex["removed"], ex["both"] + ex["added"]
        assert exact_valid(before) and exact_valid(after), f["sample"]
        for T in (before, after):
            hole, area = disc_hole_area(T, f)
            assert hole < 1e-6, (f["sample"], hole)
    return "6 flips: before and after have no overlap, are edge to edge (exact) and cover the disc of radius %.1f" % flips[0]["R"]


def test_f114_closure_criterion_detects_a_closed_swap():
    """positive control: two tiles of the exact patch against their mirror-image fillings (the Mystic swap of t11_swap_scan)
    are a CLOSED cluster with (dV, dE, dF) = (0, 0, 0); a single tile against nothing is not."""
    tiles, cent, vmap = tc.patch(4)
    cav = [2678, 2679]
    c = sw.complete(cav, reflect=True)
    assert c["n_solutions"] == 2
    (s, ts), = c["others"]
    old = [tiles[i] for i in cav]
    rep = cluster_report(old, ts)
    assert rep["closed"] and rep["dVEF"] == (0, 0, 0), rep
    assert not cluster_report(old[:1], ts[:1])["closed"]
    return "Mystic mirror swap: closed, dV,dE,dF = 0,0,0; one tile against one tile: not closed"


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_f11")]
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
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.tests_only:
        sys.exit(1 if run_tests() else 0)
    P = PatchIndex(4)
    out = []
    for f in load_flips():
        t = time.time()
        r = analyse(f, P)
        out.append(r)
        print(json.dumps(r, default=str), "(%.0fs)" % (time.time() - t), flush=True)
    if a.out:
        json.dump(out, open(a.out, "w"), indent=1, default=str)
