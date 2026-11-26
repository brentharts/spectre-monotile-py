#!/usr/bin/env python3
"""
toymodel_v12, stages S0 and S1: the bit and its pairs on the Hat Trick sphere.

Substrate: the kite complex of the Goldberg sphere GP(3,0) (80 hexagons, 12 pentagon holes, 480 kites) and its
hat tilings (60 hats), from the `hexasphere` repository (`HEXASPHERE_DIR`).  Nothing is copied from there.

**Definition (flat vertex of a hat).** A hat is a 13-gon with one side of length 2 (two unit sides of the
14-gon Tile(1, sqrt 3) in line).  The middle of that side is the 14th vertex P8 of the Spectre-Hat family.  On
the kite complex it is the one outline point of the hat with interior angle 180 degrees; it is always an edge
midpoint `m{a,b}` of the grid, where four kites meet: two of tile a and two of tile b.

**Definition (pair).** Two hats whose flat vertices are the same point.  Then the two hats are the only tiles at
that point (each fills 180 degrees), which is the rule "flat vertices meet alone" of `quantgravalt.tex`.
Other cases for a flat vertex: `hole` (the other side is a pentagon hole) and `split` (the other side is two
hats with a 90 degree corner each).

**Decision D1 (2026-11-25).** On the sphere the bit height h, the tunnelling J and the coupling kappa are free
parameters, not functions of the flow time: at the Hat end of the planar flow h = 0 and the bit is closed.
The dipole couplings W_AB below use the hat centroids on the unit sphere, with chordal or great-circle distance.

Run:  HEXASPHERE_DIR=/path/to/hexasphere python3 t12_pairs_sphere.py      (`--tests-only` for the checks)
"""
from __future__ import annotations

import json
import math
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _hexasphere():
    d = os.environ.get("HEXASPHERE_DIR")
    for c in ([d] if d else []) + [os.path.join(HERE, "..", "..", "..", "hexasphere")]:
        if c and os.path.exists(os.path.join(c, "src", "hexasphere", "hattrick", "ball.py")):
            p = os.path.join(c, "src")
            if p not in sys.path:
                sys.path.insert(0, p)
            return os.path.abspath(c)
    raise SystemExit("set HEXASPHERE_DIR to a clone of brentharts/hexasphere")


_BALLS = {}


def ball(sym="I", max_tilings=200, max_nodes=2_000_000):
    """HatBall on GP(3,0) with all tilings invariant under `sym` (cached)."""
    key = (sym, max_tilings, max_nodes)
    if key not in _BALLS:
        _hexasphere()
        from hexasphere.hattrick import HatBall
        _BALLS[key] = HatBall.from_frequency(3, symmetry=sym, max_tilings=max_tilings, max_nodes=max_nodes)
    return _BALLS[key]


# ---- the flat vertex -------------------------------------------------------------------------
def flat_vertex(b, kites):
    """The outline point of a hat (a set of 8 kites) with interior angle 180 degrees: a corner id ('m', {a, b})."""
    loop, true, ang = b._outline(sorted(kites))
    flat = [loop[i] for i in range(len(loop)) if ang[i] == 180]
    assert len(flat) == 1 and len(loop) == 14 and len(true) == 13, (len(flat), len(loop), len(true))
    assert flat[0][0] == "m"
    return flat[0]


def kites_at(b):
    """corner id -> list of kites that have it."""
    out = {}
    for k in b.S.kites:
        for cid in b.S.corners(k):
            out.setdefault(cid, []).append(k)
    return out


def classify(b, tiling, at=None):
    """Flat vertices of a tiling.  Returns dict(pairs=[(i, j)], hole=[i], split=[i], flat=[cid per hat]) with hats
    indexed in the sorted order of the tiling."""
    at = at or kites_at(b)
    hats = sorted(map(sorted, tiling))
    hat_of = {k: i for i, h in enumerate(hats) for k in h}
    flat = [flat_vertex(b, h) for h in hats]
    by = {}
    for i, f in enumerate(flat):
        by.setdefault(f, []).append(i)
    pairs, hole, split = [], [], []
    for f, lst in by.items():
        if len(lst) == 2:
            pairs.append(tuple(sorted(lst)))
            continue
        i = lst[0]
        others = {hat_of[k] for k in at[f]} - {i}
        if len(at[f]) == 2:
            hole.append(i)             # only this hat's two kites exist at the point: the rest is a pentagon
        else:
            assert len(others) == 2, others
            split.append(i)
    return {"pairs": sorted(pairs), "hole": sorted(hole), "split": sorted(split), "flat": flat, "hats": hats}


# ---- geometry on the unit sphere -----------------------------------------------------------------
def hat_centroid(b, kites):
    P = np.array([b.S.corner_position(c) for k in kites for c in b.S.corners(k)])
    q = P.mean(0)
    return q / np.linalg.norm(q)


def dist(p, q, kind="chord"):
    c = float(np.linalg.norm(p - q))
    return c if kind == "chord" else 2.0 * math.asin(min(1.0, c / 2.0))


def pair_W(cents, kind="chord"):
    """W_AB = 1/r_AA' - 1/r_AB' - 1/r_BA' + 1/r_BB' for pairs given by the centroids (A, B) of their two hats: the
    Coulomb energy of two dipoles of charges +-1 (the formula of `quantgravalt.tex`, Eq. overview row IV)."""
    n = len(cents)
    W = np.zeros((n, n))
    for p in range(n):
        for q in range(p + 1, n):
            (A, B), (A2, B2) = cents[p], cents[q]
            W[p, q] = W[q, p] = 1 / dist(A, A2, kind) - 1 / dist(A, B2, kind) - 1 / dist(B, A2, kind) + 1 / dist(B, B2, kind)
    return W


def classical_ground(W):
    """Minimum of E(s) = -sum_{p<q} W_pq s_p s_q over s in {+-1}^n (s_0 = +1), exhaustive; returns (E, s, degeneracy,
    f_unsat) with f_unsat the unsatisfied share of sum |W|."""
    n = len(W)
    if n == 0:
        return 0.0, np.zeros(0), 1, 0.0
    assert n <= 26
    idx = np.arange(1 << (n - 1), dtype=np.int64)
    S = np.ones((len(idx), n))
    for p in range(1, n):
        S[:, p] = 1 - 2 * ((idx >> (p - 1)) & 1)
    E = -0.5 * np.einsum("ip,pq,iq->i", S, W, S)
    e0 = E.min()
    deg = int((E < e0 + 1e-9).sum())
    tot = np.abs(np.triu(W, 1)).sum()
    return float(e0), S[int(E.argmin())], deg, float((tot + e0) / (2 * tot)) if tot else 0.0


def orbits_of_pairs(b, cl, G_kperm):
    """Orbits of the pairs of a tiling under the kite permutations of a group (those that keep the tiling)."""
    hats = cl["hats"]
    index = {frozenset(h): i for i, h in enumerate(hats)}
    keep = [kp for kp in G_kperm if all(frozenset(kp[k] for k in h) in index for h in hats)]
    seen, orbs = set(), []
    for pr in cl["pairs"]:
        if pr in seen:
            continue
        o = set()
        for kp in keep:
            img = tuple(sorted(index[frozenset(kp[k] for k in hats[i])] for i in pr))
            o.add(img)
        seen |= o
        orbs.append(sorted(o))
    return len(keep), orbs


def survey(sym, **kw):
    b = ball(sym, **kw)
    at = kites_at(b)
    rows = []
    for T in b.tilings:
        cl = classify(b, T, at)
        cents = [(hat_centroid(b, cl["hats"][i]), hat_centroid(b, cl["hats"][j])) for i, j in cl["pairs"]]
        row = {"pairs": len(cl["pairs"]), "hole": len(cl["hole"]), "split": len(cl["split"])}
        if 0 < len(cents) <= 24:
            for kind in ("chord", "arc"):
                W = pair_W(cents, kind)
                e0, s, deg, f = classical_ground(W)
                row["f_unsat_" + kind] = round(f, 6)
                row["deg_" + kind] = deg
                row["antiferro_share_" + kind] = round(float((np.triu(W, 1) < 0).sum() / max(1, len(cents) * (len(cents) - 1) / 2)), 4)
        rows.append(row)
    return {"symmetry": sym, "tilings": len(b.tilings), "exhausted": b.search["exhausted"], "rows": rows,
            "pair_counts": dict(sorted(Counter(r["pairs"] for r in rows).items())),
            "hole_counts": dict(sorted(Counter(r["hole"] for r in rows).items()))}


# ---- tests -------------------------------------------------------------------------------------
def test_substrate_is_gp30():
    b = ball("I")
    assert len(b.V) == 92 and len(b.F) == 180 and len(b.S.kites) == 480 and len(b.pent) == 12 and len(b.tilings) == 2


def test_every_hat_has_one_flat_vertex_at_an_edge_midpoint():
    b = ball("I")
    for T in b.tilings:
        for h in T:
            f = flat_vertex(b, h)
            assert f[0] == "m" and len(f[1]) == 2


def test_flat_vertices_partition_into_pairs_holes_and_splits():
    b = ball("I")
    for T in b.tilings:
        cl = classify(b, T)
        assert 2 * len(cl["pairs"]) + len(cl["hole"]) + len(cl["split"]) == 60
        used = [i for p in cl["pairs"] for i in p]
        assert len(used) == len(set(used))                        # pairs are disjoint: one qubit per pair


def test_pair_counts_are_invariant_under_the_symmetry():
    b = ball("I")
    for T in b.tilings:
        cl = classify(b, T)
        n, orbs = orbits_of_pairs(b, cl, b.kperm)
        assert n == 60 and sum(len(o) for o in orbs) == len(cl["pairs"])
        assert all(len(o) in (30, 60) for o in orbs) or not orbs    # an orbit of pairs: free (60) or fixed by a 2-fold (30)


def test_dipole_coupling_is_symmetric_and_falls_off():
    A, B = np.array([1.0, 0, 0]), np.array([math.cos(0.1), math.sin(0.1), 0])
    def at(phi):
        R = np.array([[math.cos(phi), -math.sin(phi), 0], [math.sin(phi), math.cos(phi), 0], [0, 0, 1]])
        return (R @ A, R @ B)
    w1 = pair_W([at(0), at(0.6)])[0, 1]
    w2 = pair_W([at(0), at(1.2)])[0, 1]
    assert abs(w2) < abs(w1) and np.allclose(pair_W([at(0), at(0.6)]), pair_W([at(0), at(0.6)]).T)


def test_classical_ground_of_a_ferromagnet():
    W = np.ones((4, 4)) - np.eye(4)
    e0, s, deg, f = classical_ground(W)
    assert e0 == -6 and deg == 1 and f == 0 and abs(s.sum()) == 4


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__)
    print("%d checks pass" % len(fs))


def main():
    out = {}
    for sym, kw in (("I", {}), ("T", {}), ("C5", {}), ("C3", {}), ("C2", {}), ("C1", {"max_tilings": 1000, "max_nodes": 300000})):
        s = survey(sym, **kw)
        out[sym] = s
        print(sym, "tilings", s["tilings"], "exhausted", s["exhausted"], "pairs", s["pair_counts"], "holes", s["hole_counts"], flush=True)
    json.dump(out, open(os.path.join(HERE, "results_pairs_sphere.json"), "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
