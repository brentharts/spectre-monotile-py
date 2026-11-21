#!/usr/bin/env python3
"""t10_rules.py - the local pair rule of Result `ham-rg` (Result 20 of the Nov 2026 notes) as a finite table.

Question (toymodel_v7 E7.27): the number of flat pairs on the boundary of two adjacent level-1 supertiles, is it a function
of a finite label of the two blocks and of the edges they share, and does the table of such labels close (no new
type from some level on)?

Definitions.  A level-L patch is built by L substitutions; each tile keeps its path of slots and the species c of the
leaf it was created as (its *leaf label*: the tile of species c is inflated into the supertile of species c, so this is the
type of the block the tile becomes), and kind in {p, g1, g2}.  A level-L tile with path (s_L, ..., s_1) belongs to the
level-(L-1) tile with path (s_L, ..., s_2) (drop the innermost slot).  Two level-(L-1) tiles are adjacent if they share a
directed edge reversed; the *seam* is the set of pairs of edge indices (e_a, e_b) so shared.  A flat pair of level L is a pair
of tiles whose flat vertices (vertex 10) meet alone; its two tiles lie in two adjacent blocks (v7 E7.25), and the pair is
counted on that seam.  The *type* of a seam is (leaf label, kind) of the two tiles and the sorted edge pairs; the
canonical type is the minimum over the two orders of the tiles.  Its *count* is the number of pairs on the seam (0 or 1).

Result (Exact; checked for levels 2 to 6): the count is a function of the canonical type; no pair lies inside a block and none
joins non-adjacent blocks; the table closes (no new type at levels 5 and 6).  The v7 table (key: the outermost species of the
tile, not its leaf label) does not close (194 types at level 4, 120 of them new), which is why the two tables differ.

    SPECTRE_DIR=/path/to/spectre python3 t10_rules.py --tests-only    # about 1 minute
    SPECTRE_DIR=... python3 t10_rules.py --levels 6                    # table, written to results_rules.json
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
for _p in (os.path.join(os.environ.get("SPECTRE_DIR", ""), "boundary_certificate"),
           os.path.join(_HERE, "..", "..", "..", "spectre", "boundary_certificate")):
    if _p and os.path.isfile(os.path.join(_p, "spectre_exact.py")):
        sys.path.insert(0, _p)
        break
import spectre_exact as se  # noqa: E402

FLAT = 10
_CACHE = {}


def build_hier(levels):
    """per level, the Delta patch as a list of (T, kind, path, leaf_species); path[0] is the outermost slot."""
    if levels in _CACHE:
        return _CACHE[levels]
    quad = [se.SPECTRE[3], se.SPECTRE[5], se.SPECTRE[7], se.SPECTRE[11]]
    g2 = se.mul(se.ttrans(se.SPECTRE[8]), se.trot(1))
    sys_ = {s: ([(se.IDENT, "p", (), s)] if s != "Gamma" else [(se.IDENT, "g1", (), "Gamma"), (g2, "g2", (), "Gamma")])
            for s in se.SPECIES}
    out = [sys_["Delta"]]
    for _ in range(levels):
        Ts, quad = se.slot_transforms(quad)
        new = {}
        for s in se.SPECIES:
            lst = []
            for slot, c in enumerate(se.SUPER_RULES[s]):
                if c is None:
                    continue
                for (T, kind, path, leaf) in sys_[c]:
                    lst.append((se.mul(Ts[slot], T), kind, (slot,) + path, leaf))
            new[s] = lst
        sys_ = new
        out.append(sys_["Delta"])
    _CACHE[levels] = out
    return out


def _key(v):
    return tuple(int(x) for x in v)


def flat_pairs(tiles):
    at = collections.defaultdict(list)
    for i, (T, _k, _p, _l) in enumerate(tiles):
        poly = [_key(v) for v in se.outline(T)]
        at[poly[FLAT]].append((i, "flat"))
        for j, v in enumerate(poly):
            if j != FLAT:
                at[v].append((i, j))
    pairs = []
    for v, lst in at.items():
        if len(lst) == 2 and all(x[1] == "flat" for x in lst):
            a, b = lst[0][0], lst[1][0]
            pairs.append((min(a, b), max(a, b)))
    return sorted(pairs)


def seams(tiles):
    own = {}
    for i, (T, _k, _p, _l) in enumerate(tiles):
        poly = [_key(v) for v in se.outline(T)]
        for e in range(14):
            own[(poly[e], poly[(e + 1) % 14])] = (i, e)
    out = collections.defaultdict(set)
    for (u, v), (a, ea) in own.items():
        o = own.get((v, u))
        if o and a < o[0]:
            out[(a, o[0])].add((ea, o[1]))
    return out


def table(L, levels_data=None):
    """rule table of level L (needs levels L and L-1).  Returns a dict of counts and the maps."""
    lv = levels_data or build_hier(L)
    cur, prev = lv[L], lv[L - 1]
    by_path = collections.defaultdict(list)
    for i, (_T, _k, p, _l) in enumerate(prev):
        by_path[p].append(i)
    sm = seams(prev)
    pairs = flat_pairs(cur)
    cnt = collections.Counter()
    inside = apart = 0
    for a, b in pairs:
        pa, pb = cur[a][2][:-1], cur[b][2][:-1]
        if pa == pb:
            inside += 1
            continue
        hit = [(min(x, y), max(x, y)) for x in by_path[pa] for y in by_path[pb] if (min(x, y), max(x, y)) in sm]
        if not hit:
            apart += 1
        else:
            cnt[hit[0]] += 1
    canon = collections.defaultdict(set)
    directed = collections.defaultdict(set)       # his convention: tile order by index, kind dropped
    for (a, b), edges in sm.items():
        la, ka, lb, kb = prev[a][3], prev[a][1], prev[b][3], prev[b][1]
        c = cnt.get((a, b), 0)
        e1 = tuple(sorted(edges))
        e2 = tuple(sorted((y, x) for x, y in edges))
        canon[min((la, ka, lb, kb, e1), (lb, kb, la, ka, e2))].add(c)
        directed[(la, lb, e1)].add(c)
    return {"L": L, "pairs": len(pairs), "inside": inside, "apart": apart, "max_per_seam": max(cnt.values()),
            "canon": canon, "directed": directed, "n_seams": len(sm)}


# ------------------------------------------------------------------ tests
def test_e120_pair_counts_and_adjacency():
    """flat pairs 6, 82, 804, 7012 at levels 2..5, none inside a block, none between non-adjacent blocks, at most one per seam."""
    lv = build_hier(5)
    ref = {2: 6, 3: 82, 4: 804, 5: 7012}
    for L, n in ref.items():
        t = table(L, lv)
        assert t["pairs"] == n and t["inside"] == 0 and t["apart"] == 0 and t["max_per_seam"] == 1, (L, t["pairs"])
    return "pairs 6, 82, 804, 7012; inside 0, apart 0, at most 1 per seam"


def test_e121_rule_is_a_function():
    lv = build_hier(5)
    out = []
    for L in (2, 3, 4, 5):
        t = table(L, lv)
        assert all(len(v) == 1 for v in t["canon"].values()) and all(len(v) == 1 for v in t["directed"].values()), L
        out.append(len(t["canon"]))
    return "count is a function of the canonical type at L=2..5 (types %s)" % out


def test_e122_closure():
    """canonical types at L = 2, 3, 4, 5, 6: no new type at 5 and 6."""
    lv = build_hier(6)
    keys, rows = {}, []
    for L in (2, 3, 4, 5, 6):
        t = table(L, lv)
        keys[L] = set(t["canon"])
        pos = sum(1 for v in t["canon"].values() if 1 in v)
        rows.append((L, t["pairs"], len(t["directed"]), len(t["canon"]), pos))
    assert keys[4] == keys[5] == keys[6] and not (keys[3] - keys[4])
    assert [r[3] for r in rows] == [14, 46, 53, 53, 53] and [r[4] for r in rows] == [6, 28, 35, 35, 35]
    assert [r[2] for r in rows] == [14, 51, 66, 66, 66]
    return "; ".join("L%d: %d pairs, %d index-ordered, %d canonical, %d positive" % r for r in rows)


def run_tests(verbose=True):
    import time
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e12")]
    failed = 0
    for n, f in tests:
        t = time.time()
        try:
            r = f()
            if verbose:
                print("ok   %s  %s  (%.0fs)" % (n, r, time.time() - t), flush=True)
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e), flush=True)
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--levels", type=int, default=0)
    a = ap.parse_args()
    if a.levels:
        lv = build_hier(a.levels)
        res = {}
        for L in range(2, a.levels + 1):
            t = table(L, lv)
            res[L] = {"pairs": t["pairs"], "seams": t["n_seams"], "index_ordered_types": len(t["directed"]),
                      "canonical_types": len(t["canon"]),
                      "canonical_positive": sum(1 for v in t["canon"].values() if 1 in v)}
            print(L, res[L], flush=True)
        json.dump(res, open(os.path.join(_HERE, "results_rules.json"), "w"), indent=1)
    else:
        sys.exit(1 if run_tests() else 0)
