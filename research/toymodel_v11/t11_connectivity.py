#!/usr/bin/env python3
"""t11_connectivity.py - a connectivity of the Spectre tiling for which a flip is a reconnection (Open problem 4, first half).

Definition (worms).  The 14 edges of a Spectre have directions that are multiples of 30 degrees.  Ten of them have a direction that
occurs once in the tile, and each has exactly one antiparallel edge in the tile; the other four are two edges in a direction d and
two in the direction d + 180 degrees (the pair at the flat vertex P_8 is one of the two couples of equal directions).  Those four
can be paired in two ways with antiparallel edges, and exactly one of the two ways does not cross in the cyclic order of the
outline.  So every tile has a unique non-crossing pairing of its 14 edges into 7 antiparallel pairs.  A WORM is the chain of tiles
obtained by entering a tile through an edge, leaving through its partner, and entering the next tile through that edge: each tile
is crossed by seven worms, and a worm is a strip bounded by antiparallel edges, like a de Bruijn strip of a rhombus tiling.  The worms
are the sheets of this tiling.

Reconnection.  Take a hole H of the tiling, made of tiles, with the tiles outside fixed.  The worms that cross the boundary of H
enter and leave through boundary edges; the PAIRING of those edges (which entry connects to which exit through H) is a perfect
matching.  Two fillings of H are a RECONNECTION if their matchings differ: the outside is the same, the connection of the worm ends
through the hole is not.  A cycle of the symmetric difference of the matchings with four edges is a simple X-reconnection (two worms
exchange partners).

What is tested (``python3 t11_connectivity.py --tests-only``; results by running without argument):
  C1 (Exact)  the non-crossing antiparallel pairing of the 14 edges exists and is unique for all 24 shapes (12 rotations, both handednesses).
  C2 (Counted)  the worm structure of the level-4 patch: number, lengths, how many close or reach the boundary of the patch.
  C3 (Exact)  every filling of every hole class found by t11_swap_scan (k = 2, reflected tiles allowed) is compared with the original:
        pairing of worm ends and the contact graph of the ring.
No claim is made that this is the connectivity of a gauge field: it is a combinatorial definition, the only one tried, chosen because
it is well defined for every tile of both handedness and because the tetrad flux of the lift (Result lift-flux) is zero for every
phase, so no flux-based notion can distinguish two fillings.
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
from t10_cavity import xyf  # noqa: E402


# ----------------------------------------------------------------- edges and their partners
def edge_dir(v, w):
    a, b = xyf(v), xyf(w)
    return round(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) / 30.0) % 12


def tile_edges_info(t):
    """list of (edge key, direction) for the 14 edges of tile t in the order of t.verts."""
    n = len(t.verts)
    return [(frozenset((t.verts[i], t.verts[(i + 1) % n])), edge_dir(t.verts[i], t.verts[(i + 1) % n])) for i in range(n)]


def _cross(p, q, r, s_):
    """do the chords (p, q) and (r, s) of the cyclic order cross?"""
    a, b = sorted((p, q))
    return (a < r < b) != (a < s_ < b)


def partners(t):
    """{edge index: partner edge index}: the unique non-crossing pairing of the 14 edges of t into antiparallel pairs."""
    dirs = [d for _, d in tile_edges_info(t)]
    n = len(dirs)
    by = collections.defaultdict(list)
    for i, d in enumerate(dirs):
        by[d].append(i)
    out, ambiguous = {}, []
    for d in range(12):
        A, B = by.get(d, []), by.get((d + 6) % 12, [])
        assert len(A) == len(B), (d, A, B)
        if d >= 6:
            continue
        if len(A) == 1:
            out[A[0]], out[B[0]] = B[0], A[0]
        else:
            assert len(A) == 2
            ambiguous.append((A, B))
    for A, B in ambiguous:
        opts = [((A[0], B[0]), (A[1], B[1])), ((A[0], B[1]), (A[1], B[0]))]
        good = [o for o in opts if not _cross(o[0][0], o[0][1], o[1][0], o[1][1])]
        assert len(good) == 1, (A, B)
        for p, q in good[0]:
            out[p], out[q] = q, p
    assert len(out) == n
    return out


# ----------------------------------------------------------------- worms through a hole
def pairing(tiles):
    """the matching of boundary edges through the hole made of `tiles` (list of Tile).  Returns (matching, interior_steps):
    matching = set of frozenset({edge, edge}); interior_steps = number of tile crossings."""
    infos = [tile_edges_info(t) for t in tiles]
    parts = [partners(t) for t in tiles]
    emap = collections.defaultdict(list)
    for ti, info in enumerate(infos):
        for ei, (key, d) in enumerate(info):
            emap[key].append((ti, ei))
    matching, steps, done = set(), 0, set()
    for key, lst in emap.items():
        if len(lst) != 1:
            continue
        ti, ei = lst[0]
        if key in done:
            continue
        cur = (ti, ei)
        while True:
            steps += 1
            t_i, e_i = cur
            pe = parts[t_i][e_i]
            k2 = infos[t_i][pe][0]
            others = [x for x in emap[k2] if x != (t_i, pe)]
            if not others:
                matching.add(frozenset((key, k2)))
                done.update((key, k2))
                break
            cur = others[0]
            if steps > 10 ** 4:
                raise RuntimeError("worm does not end")
    return matching, steps


def boundary_edges(tiles):
    c = collections.Counter(key for t in tiles for key, _ in tile_edges_info(t))
    return {k for k, n in c.items() if n == 1}


def line_class_sizes(boundary):
    """sizes of the classes of boundary edges that share a line direction (direction mod 180 degrees): a worm joins two edges of one class."""
    c = collections.Counter()
    for key in boundary:
        a, b = tuple(key)
        c[edge_dir(a, b) % 6] += 1
    return dict(sorted(c.items()))


def n_possible_pairings(sizes):
    """number of perfect matchings that join only edges of one class (product of (n - 1)!!); 0 if a class has odd size."""
    out = 1
    for n in sizes.values():
        if n % 2:
            return 0
        for k in range(n - 1, 0, -2):
            out *= k
    return out


def cycles_of_difference(m1, m2):
    """cycles of the symmetric difference of two perfect matchings (as lists of edges); length counted in matching edges."""
    g = collections.defaultdict(list)
    for m, tag in ((m1, 1), (m2, 2)):
        for pr in m:
            a, b = tuple(pr)
            g[a].append((b, tag))
            g[b].append((a, tag))
    seen, cyc = set(), []
    for v in g:
        if v in seen:
            continue
        comp, stack = [], [v]
        seen.add(v)
        while stack:
            x = stack.pop()
            comp.append(x)
            for y, _ in g[x]:
                if y not in seen:
                    seen.add(y)
                    stack.append(y)
        n_diff = sum(1 for pr in m1 ^ m2 if next(iter(pr)) in comp)
        if n_diff:
            cyc.append(len(comp))
    return sorted(cyc)


def contact_signature(tiles, ring_edges):
    """for each tile of the hole filling: the multiset of (ring tile, number of shared edges); plus hole-hole edge counts."""
    infos = [tile_edges_info(t) for t in tiles]
    sig = []
    for i, info in enumerate(infos):
        c = collections.Counter()
        for key, d in info:
            if key in ring_edges:
                c[ring_edges[key]] += 1
            else:
                for j, info2 in enumerate(infos):
                    if j != i and any(key == k2 for k2, _ in info2):
                        c[("hole",)] += 1
        sig.append(tuple(sorted(c.items(), key=str)))
    return tuple(sorted(sig, key=str))


# ----------------------------------------------------------------- the analysis of one hole
def analyse_hole(cavity, level=4, reflect=True):
    tiles, cent, vmap = tc.patch(level)
    c = sw.complete(cavity, reflect=reflect, level=level)
    old = [tiles[i] for i in cavity]
    cav = set(cavity)
    ring_edges = {}
    ring = set()
    for i in cavity:
        for v in tiles[i].verts:
            for j, _ in vmap[v]:
                if j not in cav:
                    ring.add(j)
    for j in ring:
        for key, d in tile_edges_info(tiles[j]):
            ring_edges[key] = j
    m0, steps0 = pairing(old)
    out = []
    for s, ts in c["others"]:
        m1, steps1 = pairing(ts)
        assert boundary_edges(old) == boundary_edges(ts)
        out.append({
            "boundary_edges": len(boundary_edges(old)),
            "line_class_sizes": line_class_sizes(boundary_edges(old)),
            "possible_pairings": n_possible_pairings(line_class_sizes(boundary_edges(old))),
            "matching_changed": m0 != m1,
            "pairs_changed": len(m0 - m1),
            "difference_cycles": cycles_of_difference(m0, m1),
            "contact_changed": contact_signature(old, ring_edges) != contact_signature(ts, ring_edges),
            "dVEF": sw.dvef(old, ts),
            "replaced_kinds": sorted(sw.kinds()[i] for i in cavity if tiles[i].vset not in {t.vset for t in ts})})
    return c["n_solutions"], out


# ----------------------------------------------------------------- the worms of the patch
def patch_worms(level=4):
    """trace every worm of the patch: returns list of (length in tiles, closed?, reaches_boundary?)."""
    tiles, cent, vmap = tc.patch(level)
    info = [tile_edges_info(t) for t in tiles]
    emap = collections.defaultdict(list)
    for ti, inf in enumerate(info):
        for ei, (key, d) in enumerate(inf):
            emap[key].append((ti, ei))
    parts = [partners(t) for t in tiles]
    visited = set()
    res = []
    for ti in range(len(tiles)):
        for e0 in parts[ti]:
            if (ti, e0) in visited:
                continue
            # follow the worm in both directions from the crossing (ti, e0 -> partner)
            seq = []
            ends = 0
            closed = False
            for direction in (0, 1):
                cur = (ti, e0) if direction == 0 else (ti, parts[ti][e0])
                path = []
                while True:
                    t_i, e_i = cur
                    visited.add((t_i, e_i))
                    pe = parts[t_i][e_i]
                    visited.add((t_i, pe))
                    path.append(t_i)
                    k2 = info[t_i][pe][0]
                    others = [x for x in emap[k2] if x != (t_i, pe)]
                    if not others:
                        ends += 1
                        break
                    cur = others[0]
                    if cur == (ti, e0) or cur == (ti, parts[ti][e0]):
                        closed = True
                        break
                    if len(path) > 10 ** 5:
                        break
                if closed:
                    break
                if direction == 0:
                    seq = path
                else:
                    seq = path[::-1] + seq
            res.append((len(seq), closed, ends))
    return res


def worm_end_classes(level=4, limit=None):
    """for every worm of the patch that ends on both sides at the patch boundary: (line direction class of its first edge, of its last edge)."""
    tiles, cent, vmap = tc.patch(level)
    info = [tile_edges_info(t) for t in tiles]
    emap = collections.defaultdict(list)
    for ti, inf in enumerate(info):
        for ei, (key, d) in enumerate(inf):
            emap[key].append((ti, ei))
    parts = [partners(t) for t in tiles]
    bad, checked = 0, 0
    for key, lst in emap.items():
        if len(lst) != 1:
            continue
        ti, ei = lst[0]
        d0 = info[ti][ei][1] % 6
        cur = (ti, ei)
        steps = 0
        while True:
            t_i, e_i = cur
            pe = parts[t_i][e_i]
            if info[t_i][pe][1] % 6 != d0:
                bad += 1
            k2 = info[t_i][pe][0]
            others = [x for x in emap[k2] if x != (t_i, pe)]
            if not others:
                break
            cur = others[0]
            steps += 1
            if steps > 10 ** 5:
                break
        checked += 1
        if limit and checked >= limit:
            break
    return checked, bad


# ----------------------------------------------------------------- tests
def test_c111_every_shape_has_a_unique_noncrossing_antiparallel_pairing():
    for sh in tc._build_shapes(True):
        t = tc.Tile(sh["verts"], sh)
        p = partners(t)
        dirs = [d for _, d in tile_edges_info(t)]
        assert len(p) == 14 and all(p[p[i]] == i for i in p)
        assert all((dirs[i] - dirs[p[i]]) % 12 == 6 for i in p)
    return "24 shapes (12 rotations, both handednesses): 7 antiparallel pairs per tile, the non-crossing choice is unique"


def test_c112_matching_is_a_perfect_matching_and_unchanged_for_the_original():
    tiles, cent, vmap = tc.patch(4)
    for cav in ([2675], [2677, 2678, 2679], tc.choose_cavity(4, 10, tc.CENTRE)):
        old = [tiles[i] for i in cav]
        m, steps = pairing(old)
        b = boundary_edges(old)
        assert set().union(*m) == b and 2 * len(m) == len(b)
        assert pairing(old)[0] == m
    return "pairing of boundary edges is a perfect matching for holes of 1, 3, 10 tiles"


def test_c113_mystic_mirror_swap_reconnects_or_not_is_computed_and_recorded():
    n, alts = analyse_hole([2678, 2679])
    assert n == 2 and len(alts) == 1
    a = alts[0]
    assert a["dVEF"] == (0, 0, 0) and a["replaced_kinds"] == ["g1", "g2"]
    return "swap of the Mystic pair 2678/2679: boundary edges %d, matching changed=%s (pairs changed %d, cycles %s), contact graph changed=%s" % (
        a["boundary_edges"], a["matching_changed"], a["pairs_changed"], a["difference_cycles"], a["contact_changed"])


def test_c114_a_worm_keeps_its_line_direction():
    """exact lemma: every edge a worm passes has the same direction mod 180 degrees, so a worm joins boundary edges of one class;
    checked on every worm that starts at the boundary of the level-4 patch.  The Mystic outline has classes of 4, 4, 2, 2, 4, 4 edges, so
    81 pairings are compatible with the lemma: the pairing is not forced by the outline."""
    checked, bad = worm_end_classes(4)
    assert checked > 100 and bad == 0, (checked, bad)
    tiles, cent, vmap = tc.patch(4)
    sizes = line_class_sizes(boundary_edges([tiles[2678], tiles[2679]]))
    assert sorted(sizes.values()) == [2, 2, 4, 4, 4, 4] and n_possible_pairings(sizes) == 81, sizes
    return "%d boundary-started worms: all edges on a worm share a line direction; Mystic outline: class sizes %s, %d pairings compatible" % (
        checked, sorted(sizes.values()), n_possible_pairings(sizes))


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_c11")]
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


def run_all(out=None, radius=18.0):
    tiles, cent, vmap = tc.patch(4)
    # C2: worm structure
    t = time.time()
    w = patch_worms(4)
    L = np.array([x[0] for x in w])
    closed = sum(1 for x in w if x[1])
    two_ends = sum(1 for x in w if x[2] == 2)
    print("patch worms: %d worms, %d closed loops, %d end at the patch boundary on both sides, length min/median/max %d/%d/%d (%.0fs)" % (
        len(w), closed, two_ends, L.min(), np.median(L), L.max(), time.time() - t), flush=True)
    # C3: all hole classes (k = 2, 3) with a second filling when reflected tiles are allowed
    d = np.linalg.norm(cent - cent[tc.CENTRE], axis=1)
    S = [int(i) for i in np.where(d < radius)[0]]
    nb = sw.neighbours(tiles, vmap, S)
    rows = []
    for k in (2, 3):
        classes = {}
        for s in sw.connected_sets(nb, S, k):
            classes.setdefault(sw.hole_key(tiles, vmap, s), sorted(s))
        t = time.time()
        sub = []
        for cav in classes.values():
            n, alts = analyse_hole(cav)
            for a in alts:
                a["cavity"], a["k"] = cav, k
                sub.append(a)
        rows += sub
        print("k=%d hole classes: %d, with a second filling: %d (%.0fs)" % (k, len(classes), len(sub), time.time() - t), flush=True)
        for key in ("matching_changed", "contact_changed"):
            print("  %s in %d of %d" % (key, sum(r[key] for r in sub), len(sub)))
        print("  replaced kinds:", dict(collections.Counter(tuple(r["replaced_kinds"]) for r in sub)))
        print("  (dV,dE,dF):", dict(collections.Counter(r["dVEF"] for r in sub)))
        print("  boundary edges:", dict(collections.Counter(r["boundary_edges"] for r in sub)))
        print("  possible pairings:", dict(collections.Counter(r["possible_pairings"] for r in sub)), flush=True)
    if out:
        json.dump({"worms": {"n": len(w), "closed": closed, "two_ends": two_ends,
                             "length": [int(L.min()), float(np.median(L)), int(L.max())]}, "swaps": rows},
                  open(out, "w"), indent=1, default=str)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--radius", type=float, default=18.0)
    a = ap.parse_args()
    if a.tests_only:
        sys.exit(1 if run_tests() else 0)
    run_all(a.out, a.radius)
