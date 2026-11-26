#!/usr/bin/env python3
"""
toymodel_v12, stage S2: the closed flip graph of the Hat Trick sphere.

All hat tilings of GP(3,0) with the 12 pentagons as holes are enumerated by exact cover (no symmetry imposed) and
compared.  What the tests below establish (**Exact**, finite enumeration):

  * there are exactly 4096 = 2^12 tilings;
  * the 480 kites split into 12 *rotor regions* of 40 kites, one around each pentagon, the same in every tiling;
    a region is tiled by 5 hats in exactly two ways, independently of the other regions.  So a tiling is a point
    sigma of {0,1}^12, one bit at each cone point of the sphere (sigma_p = 1: the flat vertices of the 5 hats lie
    on the pentagon; sigma_p = 0: they point away, to the midpoints of the 5 icosahedron edges at p);
  * a *flip* is the change of one bit: 5 hats out, 5 hats in, same 40 kites.  It is a closed cluster.  The flip
    graph is the 12-cube;
  * the bump-and-dent pairs of `t12_pairs_sphere.py` are the icosahedron edges pq with sigma_p = sigma_q = 0
    (at most 30, one at each 2-fold axis).

For each flip the change (dV, dE, dF) of the cell complex (hats and pentagons as faces; a vertex is a point where
three or more faces meet) and of the contact graph is recorded.

Run:  HEXASPHERE_DIR=/path/to/hexasphere python3 t12_flip_graph.py     (`--tests-only` for the checks)
"""
from __future__ import annotations

import itertools
import json
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_pairs_sphere as P       # noqa: E402

_ALL = {}


def all_tilings():
    """(ball, list of tilings as sorted lists of sorted kite lists), every tiling, no symmetry imposed."""
    if not _ALL:
        b = P.ball("C1", max_tilings=6000, max_nodes=20_000_000)
        assert b.search["exhausted"]
        _ALL["b"] = b
        _ALL["T"] = [sorted(map(sorted, T)) for T in b.tilings]
        _ALL["at"] = P.kites_at(b)
    return _ALL["b"], _ALL["T"]


def bits(b, tiling):
    """sigma: for each pentagon (in the order b.pent) 1 if the 5 flat vertices around it lie on it, else 0."""
    cnt = Counter()
    for h in tiling:
        f = P.flat_vertex(b, h)
        for v in f[1]:
            if v in b.pent:
                cnt[v] += 1
    assert all(c == 5 for c in cnt.values()), cnt
    return tuple(1 if cnt.get(p, 0) == 5 else 0 for p in b.pent)


def icosa_edges(b):
    """Pairs (i, j) of indices into b.pent that are neighbours on the icosahedron (nearest pentagons)."""
    X = np.array([b.V[p] for p in b.pent])
    D = np.linalg.norm(X[:, None] - X[None], axis=2)
    d0 = np.sort(D[0])[1]
    return [(i, j) for i in range(12) for j in range(i + 1, 12) if abs(D[i, j] - d0) < 1e-6]


def regions(b, tilings):
    """The rotor regions: connected components of 'two kites are in one hat in some tiling', as kite sets."""
    parent = {k: k for k in b.S.kites}

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for T in tilings:
        for h in T:
            r = find(h[0])
            for k in h[1:]:
                parent[find(k)] = r
    comp = {}
    for k in b.S.kites:
        comp.setdefault(find(k), []).append(k)
    return sorted(map(sorted, comp.values()))


def tile_of(b, tiling):
    return {k: i for i, h in enumerate(tiling) for k in h}


def complex_counts(b, tiling):
    """(V, E, F) of the cell complex of a tiling: faces are the 60 hats and 12 pentagons, an edge is a maximal chain of
    kite sides between two different faces, a vertex a point where three or more faces meet.  Also the contact graph
    (set of unordered face pairs sharing a kite side; a pentagon is ('P', p))."""
    hat = tile_of(b, tiling)
    S = b.S
    deg = Counter()
    contact = set()
    seen = set()
    for k in S.kites:
        t, i = divmod(k, 3)
        f = S.F[t]
        c = S.corners(k)
        for s in range(4):
            nb = S.nb[k][s]
            other = hat[nb] if nb is not None else ("P", f[(i + 1) % 3] if s == 1 else f[(i + 2) % 3])
            me = hat[k]
            if other == me:
                continue
            e = frozenset((c[s], c[(s + 1) % 4]))
            contact.add(frozenset((me, other)))
            if e in seen:
                continue
            seen.add(e)
            for x in e:
                deg[x] += 1
    V = sum(1 for d in deg.values() if d >= 3)
    E = sum(d for d in deg.values() if d >= 3) // 2
    return V, E, len(tiling) + len(b.pent), contact


def flip_record(b, T0, T1):
    """Compare two tilings: hats removed and added, whether the cluster is closed, (dV, dE, dF), contact changes."""
    A, B = {tuple(h) for h in T0}, {tuple(h) for h in T1}
    out_, in_ = A - B, B - A
    k0, k1 = sorted(k for h in out_ for k in h), sorted(k for h in in_ for k in h)
    V0, E0, F0, C0 = complex_counts(b, T0)
    V1, E1, F1, C1 = complex_counts(b, T1)
    # contact graph by kite sets, so that hats are compared by identity
    name0 = {i: tuple(h) for i, h in enumerate(T0)}
    name1 = {i: tuple(h) for i, h in enumerate(T1)}
    lab = lambda C, nm: {frozenset(nm.get(x, x) for x in e) for e in C}
    L0, L1 = lab(C0, name0), lab(C1, name1)
    return {"out": len(out_), "in": len(in_), "closed": k0 == k1, "kites": len(k0),
            "dV": V1 - V0, "dE": E1 - E0, "dF": F1 - F0, "euler0": V0 - E0 + F0, "euler1": V1 - E1 + F1,
            "contacts_removed": len(L0 - L1), "contacts_added": len(L1 - L0),
            "degree_sequence_changed": sorted(Counter(x for e in C0 for x in e).values()) != sorted(Counter(x for e in C1 for x in e).values())}


def survey():
    b, Ts = all_tilings()
    sig = [bits(b, T) for T in Ts]
    index = {s: i for i, s in enumerate(sig)}
    edges = icosa_edges(b)
    nbrs = {i: [j for e in edges for j in e if i in e and j != i] for i in range(12)}
    reg = regions(b, Ts)
    at = _ALL["at"]
    # pairs against the edge rule, all tilings
    pair_rule = all(len(P.classify(b, T, at)["pairs"]) == sum(1 for i, j in edges if s[i] == 0 and s[j] == 0) for T, s in zip(Ts, sig))
    # flips: every edge of the 12-cube
    table = Counter()
    closed = True
    contact_changed = 0
    n = 0
    for s in sig:
        for p in range(12):
            if s[p] == 1:
                continue                       # each cube edge once, from sigma_p = 0 to 1
            t = tuple(1 if q == p else x for q, x in enumerate(s))
            r = flip_record(b, Ts[index[s]], Ts[index[t]])
            n += 1
            closed &= r["closed"] and r["out"] == 5 and r["in"] == 5 and r["kites"] == 40 and r["euler0"] == 2 and r["euler1"] == 2
            contact_changed += int(r["contacts_removed"] + r["contacts_added"] > 0)
            k = sum(s[q] for q in nbrs[p])
            table[(k, r["dV"], r["dE"], r["dF"], r["contacts_removed"], r["contacts_added"], r["degree_sequence_changed"])] += 1
    return {"tilings": len(Ts), "distinct_bit_strings": len(index), "regions": len(reg), "region_sizes": sorted(set(map(len, reg))),
            "icosahedron_edges": len(edges), "pairs_are_edges_with_both_bits_zero": pair_rule,
            "flips": n, "all_flips_closed_5_for_5_on_40_kites": closed, "flips_changing_contact_graph": contact_changed,
            "flip_table": [{"neighbours_in_state_1": k[0], "dV": k[1], "dE": k[2], "dF": k[3], "contacts_removed": k[4],
                            "contacts_added": k[5], "degree_sequence_changed": k[6], "count": v} for k, v in sorted(table.items())]}


# ---- tests ---------------------------------------------------------------------------------------
def test_there_are_4096_tilings_and_the_search_is_exhausted():
    b, Ts = all_tilings()
    assert len(Ts) == 4096 and b.search["exhausted"]


def test_tilings_are_the_12_cube():
    b, Ts = all_tilings()
    assert len({bits(b, T) for T in Ts}) == 4096


def test_twelve_rotor_regions_of_forty_kites():
    b, Ts = all_tilings()
    reg = regions(b, Ts)
    assert len(reg) == 12 and all(len(r) == 40 for r in reg)
    # each region is tiled in exactly two ways over all tilings
    for r in reg:
        rs = set(r)
        ways = {frozenset(tuple(h) for h in T if h[0] in rs) for T in Ts}
        assert len(ways) == 2 and all(len(w) == 5 for w in ways)


def test_one_bit_flip_is_a_closed_cluster():
    b, Ts = all_tilings()
    sig = {bits(b, T): T for T in Ts}
    z = tuple([0] * 12)
    for p in range(12):
        r = flip_record(b, sig[z], sig[tuple(1 if q == p else 0 for q in range(12))])
        assert r["closed"] and r["out"] == r["in"] == 5 and r["dF"] == 0 and r["dV"] == r["dE"] and r["euler0"] == r["euler1"] == 2


def test_icosahedron_has_thirty_edges():
    b, _ = all_tilings()
    assert len(icosa_edges(b)) == 30


def test_symmetric_tilings_are_the_two_constant_strings():
    b, Ts = all_tilings()
    bi = P.ball("I")
    s = sorted(bits(b, sorted(map(sorted, T))) for T in bi.tilings)
    assert s == [tuple([0] * 12), tuple([1] * 12)]


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        s = survey()
        json.dump(s, open(os.path.join(HERE, "results_flip_graph.json"), "w"), indent=1)
        print(json.dumps(s, indent=1))
