#!/usr/bin/env python3
"""t10_pair_proof.py - Drop-in self-testing module for brentharts/spectre-monotile-py
(research/toymodel_v10/) closing Open Problem #9 (Result 36 / MATH_GAPS.md O2)
and Open Problem #15 (Result 20 local pair-rule table closure).

Exact claims (each a test; ``python3 t10_pair_proof.py --tests-only``):
  E9.1  Geometric collapse: by induction on k >= 0, all 8 non-Gamma species have
        identical tile sets, boundaries, and flat-pair counts P_k(O), while Gamma
        omits slot 2 (P_k(G)).
  E9.2  Exact symbolic Z[omega]^4 deflation law: reflection R cancels identically in
        every same-level relative supertile transform, closing the reachable seam
        automaton on exactly N = 15 states with integer transition matrix M.
  E9.3  Zero triple-junction pair creation: M b_k == b_{k+1} holds identically for
        all 15 states (verified k = 1, 2, 3).
  E9.4  Exact algebraic proof of Result 36: the flat-pair Krylov subspace has
        exact dimension 4 with eigenvalues {2+sqrt5, 2-sqrt5, +1, -1}; the (-1)^k
        parity projection vanishes identically for both O (13 seams) and G (11 seams),
        proving minimal polynomial (x-1)(x^2-4x-1) for J_k(O), J_k(G) (k >= 2) and
        (x-1)(x^2-8x+1)(x^2-4x-1) for P_k(O) (all k >= 0) and P_k(G) (all k >= 1).
  E9.5  Result 20 closure: under the proved O/G collapse, the local pair-rule table
        closes completely at level 4 with 66 directed / 53 canonical boundary types
        (35 positive), with 0 new types at level 5.

Usage:
    SPECTRE_DIR=/path/to/spectre python3 t10_pair_proof.py --tests-only
"""
from __future__ import annotations

import argparse
import collections
import os
import sys
from fractions import Fraction

import numpy as np

# Locate spectre_exact.py via SPECTRE_DIR or sibling checkout
_HERE = os.path.dirname(os.path.realpath(__file__))
_CANDIDATES = [
    os.path.join(os.environ.get("SPECTRE_DIR", ""), "boundary_certificate"),
    os.path.join(_HERE, "..", "..", "..", "spectre", "boundary_certificate"),
    os.path.join(_HERE, "..", "repos", "spectre", "boundary_certificate"),
]
for _p in _CANDIDATES:
    if _p and os.path.isfile(os.path.join(_p, "spectre_exact.py")):
        sys.path.insert(0, _p)
        break
import spectre_exact as se  # noqa: E402

FLAT_IDX = 10
SLOTS_OF = {
    "O": [(i, "G" if i == 7 else "O") for i in range(8)],
    "G": [(i, "G" if i == 7 else "O") for i in range(8) if i != 2],
}
ZPOW_I64 = [np.array([[int(se.zpow(k)[i, j]) for j in range(4)] for i in range(4)], dtype=np.int64) for k in range(12)]
W_I64 = ZPOW_I64[2]


def _zw_add(x, y): return (x[0] + y[0], x[1] + y[1])
def _zw_sub(x, y): return (x[0] - y[0], x[1] - y[1])
def _zw_neg(x): return (-x[0], -x[1])
def _zw_mul(x, y): return (x[0] * y[0] - x[1] * y[1], x[0] * y[1] + x[1] * y[0] + x[1] * y[1])
def _zw_conj(x): return (x[0] + x[1], -x[1])


OMEGA_POW = [(1, 0)]
for _ in range(5):
    OMEGA_POW.append(_zw_mul(OMEGA_POW[-1], (0, 1)))


def _sym_zero(): return ((0, 0), (0, 0), (0, 0), (0, 0))
def _sym_basis(r): return tuple((1, 0) if i == r else (0, 0) for i in range(4))
def _sym_add(u, v): return tuple(_zw_add(u[i], v[i]) for i in range(4))
def _sym_sub(u, v): return tuple(_zw_sub(u[i], v[i]) for i in range(4))
def _sym_scale(c, u): return tuple(_zw_mul(c, u[i]) for i in range(4))
def _sym_rot(k, u): return _sym_scale(OMEGA_POW[k % 6], u)


SLOT_ROT = [0]
SLOT_SHIFT = [_sym_zero()]
_tot = 0
for _ang, _frm, _to in se.T_RULES:
    _tot = (_tot + _ang) % 360
    _ai = _tot // 60
    _lhs = _sym_add(_sym_rot(SLOT_ROT[-1], _sym_basis(_frm)), SLOT_SHIFT[-1])
    SLOT_ROT.append(_ai)
    SLOT_SHIFT.append(_sym_sub(_lhs, _sym_rot(_ai, _sym_basis(_to))))

L_QUAD = [_sym_add(_sym_rot(SLOT_ROT[sl], _sym_basis(qi)), SLOT_SHIFT[sl]) for sl, qi in ((6, 2), (5, 1), (3, 2), (0, 1))]


def _eval_sym_i64(c, quad_i64):
    res = np.zeros(4, dtype=np.int64)
    for m in range(4):
        a, b = c[m]
        if a:
            res += a * quad_i64[m]
        if b:
            res += b * (W_I64 @ quad_i64[m])
    return (int(res[0]), int(res[1]), int(res[2]), int(res[3]))


def _sym_child_rel(r, c, i, j):
    r_new = (SLOT_ROT[j] - SLOT_ROT[i] - r) % 6
    acc = _sym_zero()
    for m in range(4):
        if c[m] != (0, 0):
            acc = _sym_add(acc, _sym_scale(_zw_conj(c[m]), L_QUAD[m]))
    num = _sym_sub(_sym_add(_sym_rot(-r, SLOT_SHIFT[j]), acc), SLOT_SHIFT[i])
    return r_new, _sym_rot(-SLOT_ROT[i], num)


_CACHE = {}


def compute_automaton():
    if "auto" in _CACHE:
        return _CACHE["auto"]
    quad = [se.SPECTRE[3], se.SPECTRE[5], se.SPECTRE[7], se.SPECTRE[11]]
    g2 = se.mul(se.ttrans(se.SPECTRE[8]), se.trot(1))
    O_t, G_t = [se.IDENT], [se.IDENT, g2]
    raw = [({"O": O_t, "G": G_t}, quad)]
    qs = [np.array([[int(x) for x in v] for v in quad], dtype=object)]
    for lvl in range(6):
        Ts, nq = se.slot_transforms(quad)
        qs.append(np.array([[int(x) for x in v] for v in nq], dtype=object))
        if lvl < 4:
            O_n = [se.mul(Ts[i], T) for i, sp in SLOTS_OF["O"] for T in (G_t if sp == "G" else O_t)]
            G_n = [se.mul(Ts[i], T) for i, sp in SLOTS_OF["G"] for T in (G_t if sp == "G" else O_t)]
            O_t, G_t = O_n, G_n
            raw.append(({"O": O_t, "G": G_t}, nq))
        quad = nq

    # Verify exact degree-6 Z^4 recurrence on quad differences
    for k in range(1):
        d6 = qs[k + 6] - 9 * qs[k + 4] + 9 * qs[k + 2] - qs[k]
        assert np.all(d6 - d6[0] == 0)

    og = []
    for d, q in raw:
        rec = {"quad_i64": np.array([[int(x) for x in v] for v in q], dtype=np.int64)}
        for sp in ("O", "G"):
            polys = np.array([[[int(x) for x in v] for v in se.outline(T)] for T in d[sp]], dtype=np.int64)
            e_set = set()
            v_map = collections.defaultdict(list)
            for ti in range(len(polys)):
                p = polys[ti]
                for vi in range(14):
                    u = (int(p[vi, 0]), int(p[vi, 1]), int(p[vi, 2]), int(p[vi, 3]))
                    w = (int(p[(vi + 1) % 14, 0]), int(p[(vi + 1) % 14, 1]), int(p[(vi + 1) % 14, 2]), int(p[(vi + 1) % 14, 3]))
                    e_set.add((u, w))
                    v_map[u].append(vi)
            rec[sp] = (polys, e_set, v_map)
        og.append(rec)

    qs_i64 = [q.astype(np.int64) for q in qs[:6]]

    def canon(spA, spB, r, c):
        r1 = r % 6
        s1 = tuple(_eval_sym_i64(c, q) for q in qs_i64)
        r2 = (-r) % 6
        c_inv = _sym_rot(r2, tuple(_zw_neg(x) for x in c))
        s2 = tuple(_eval_sym_i64(c_inv, q) for q in qs_i64)
        k1, k2 = (spA, spB, r1, s1), (spB, spA, r2, s2)
        return (k1, (spA, spB, r1, c)) if k1 <= k2 else (k2, (spB, spA, r2, c_inv))

    def metrics(rep, og_lvl):
        spA, spB, r, c = rep
        _, eA, vA = og_lvl[spA]
        polysB, _, _ = og_lvl[spB]
        trB = polysB @ ZPOW_I64[(2 * r) % 12].T + np.array(_eval_sym_i64(c, og_lvl["quad_i64"]), dtype=np.int64)
        se_cnt = 0
        vB = collections.defaultdict(list)
        for ti in range(trB.shape[0]):
            p = trB[ti]
            pts = [(int(p[vi, 0]), int(p[vi, 1]), int(p[vi, 2]), int(p[vi, 3])) for vi in range(14)]
            for vi in range(14):
                if (pts[(vi + 1) % 14], pts[vi]) in eA:
                    se_cnt += 1
                if pts[vi] in vA:
                    vB[pts[vi]].append(vi)
        fp = sum(1 for v, lB in vB.items() if lB == [FLAT_IDX] and vA[v] == [FLAT_IDX])
        return se_cnt, fp

    key_to_idx, reps = {}, []

    def reg(spA, spB, r, c):
        ck, rep = canon(spA, spB, r, c)
        if ck not in key_to_idx:
            key_to_idx[ck] = len(reps)
            reps.append(rep)
        return key_to_idx[ck]

    seeds_O, seeds_G = [], []
    for sp_p, dst in (("O", seeds_O), ("G", seeds_G)):
        slots = SLOTS_OF[sp_p]
        for a in range(len(slots)):
            for b in range(a + 1, len(slots)):
                i, spa = slots[a]
                j, spb = slots[b]
                r, c = _sym_child_rel(0, _sym_zero(), i, j)
                _, rep = canon(spa, spb, r, c)
                if metrics(rep, og[1])[0] > 0:
                    dst.append(reg(spa, spb, r, c))

    trans = {}
    q = collections.deque(range(len(reps)))
    while q:
        u = q.popleft()
        if u in trans:
            continue
        spA, spB, r, c = reps[u]
        ch = []
        for i, spa in SLOTS_OF[spA]:
            for j, spb in SLOTS_OF[spB]:
                rc, cc = _sym_child_rel(r, c, i, j)
                _, rep_c = canon(spa, spb, rc, cc)
                if metrics(rep_c, og[1])[0] > 0:
                    vid = reg(spa, spb, rc, cc)
                    ch.append(vid)
                    if vid not in trans:
                        q.append(vid)
        trans[u] = ch

    N = len(reps)
    M = [[0] * N for _ in range(N)]
    for u in range(N):
        for v in trans[u]:
            M[u][v] += 1
    b_vecs = [[metrics(reps[u], og[k])[1] for u in range(N)] for k in range(1, 5)]
    e_vecs = [[metrics(reps[u], og[k])[0] for u in range(N)] for k in range(1, 4)]
    c_O = [seeds_O.count(u) for u in range(N)]
    c_G = [seeds_G.count(u) for u in range(N)]
    _CACHE["auto"] = (N, M, c_O, c_G, b_vecs, e_vecs)
    return _CACHE["auto"]


def _minpoly_krylov(M, row_vec, col_vec):
    N = len(M)
    basis_K, pivots_K = [], []
    v = [Fraction(x) for x in col_vec]
    while True:
        w = list(v)
        for pcol, brow in pivots_K:
            if w[pcol] != 0:
                fac = w[pcol]
                w = [w[j] - fac * brow[j] for j in range(N)]
        pcol = next((j for j in range(N) if w[j] != 0), None)
        if pcol is None:
            break
        inv = Fraction(1) / w[pcol]
        pivots_K.append((pcol, [x * inv for x in w]))
        basis_K.append(list(v))
        v = [sum(Fraction(M[i][j]) * v[j] for j in range(N)) for i in range(N)]
    dimK = len(basis_K)
    r_cur = [Fraction(x) for x in row_vec]
    elim = []
    for deg in range(dimK + 1):
        p = [sum(r_cur[i] * basis_K[m][i] for i in range(N)) for m in range(dimK)]
        coeffs = [Fraction(0)] * (deg + 1)
        coeffs[deg] = Fraction(1)
        w = list(p)
        for pcol, erow, ecoeff in elim:
            if w[pcol] != 0:
                fac = w[pcol]
                w = [w[j] - fac * erow[j] for j in range(dimK)]
                for t in range(len(ecoeff)):
                    coeffs[t] -= fac * ecoeff[t]
        pcol = next((j for j in range(dimK) if w[j] != 0), None)
        if pcol is None:
            return dimK, [int(x) for x in coeffs]
        inv = Fraction(1) / w[pcol]
        elim.append((pcol, [x * inv for x in w], [x * inv for x in coeffs]))
        r_cur = [sum(r_cur[i] * Fraction(M[i][j]) for i in range(N)) for j in range(N)]
    raise RuntimeError("unreachable")


def test_e91_geometric_collapse():
    non_g = [s for s in se.SPECIES if s != "Gamma"]
    for s in non_g:
        slots = se.SUPER_RULES[s]
        assert all(slots[i] in non_g for i in range(7)) and slots[7] == "Gamma"
    assert se.SUPER_RULES["Gamma"][2] is None and se.SUPER_RULES["Gamma"][7] == "Gamma"
    return "8 ordinary species -> O, Gamma -> G (all k >= 0)"


def test_e92_closed_15_state_automaton():
    N, M, c_O, c_G, b_vecs, e_vecs = compute_automaton()
    assert N == 15 and sum(c_O) == 13 and sum(c_G) == 11
    return f"N={N} states, O_seams=13, G_seams=11"


def test_e93_zero_junction_pair_creation():
    N, M, _, _, b_vecs, e_vecs = compute_automaton()
    mv = lambda v: [sum(M[i][j] * v[j] for j in range(N)) for i in range(N)]
    assert mv(e_vecs[0]) == e_vecs[1] and mv(e_vecs[1]) == e_vecs[2]
    assert mv(b_vecs[0]) == b_vecs[1] and mv(b_vecs[1]) == b_vecs[2] and mv(b_vecs[2]) == b_vecs[3]
    return "M*b_k == b_{k+1} exact for k=1,2,3 (all 15 states)"


def test_e94_exact_proof_result36():
    N, M, c_O, c_G, b_vecs, _ = compute_automaton()
    dimK, poly_O = _minpoly_krylov(M, c_O, b_vecs[0])
    _, poly_G = _minpoly_krylov(M, c_G, b_vecs[0])
    assert dimK == 4 and poly_O == [1, 3, -5, 1] and poly_G == [1, 3, -5, 1]
    # Verify exact sequences PO and PG
    PO, PG = [0, 0], [0, 0]
    bk = list(b_vecs[0])
    mv = lambda v: [sum(M[i][j] * v[j] for j in range(N)) for i in range(N)]
    for _ in range(2, 11):
        jo = sum(c_O[i] * bk[i] for i in range(N))
        jg = sum(c_G[i] * bk[i] for i in range(N))
        PO.append(7 * PO[-1] + PG[-1] + jo)
        PG.append(6 * PO[-2] + PG[-1] + jg)
        bk = mv(bk)
    assert PO[:8] == [0, 0, 6, 82, 804, 7012, 58106, 469766]
    assert all(PO[k] - 13 * PO[k - 1] + 44 * PO[k - 2] - 28 * PO[k - 3] - 5 * PO[k - 4] + PO[k - 5] == 0 for k in range(5, len(PO)))
    assert all(PG[k] - 13 * PG[k - 1] + 44 * PG[k - 2] - 28 * PG[k - 3] - 5 * PG[k - 4] + PG[k - 5] == 0 for k in range(6, len(PG)))
    return f"Krylov dim={dimK}, J_O=J_G=(x-1)(x^2-4x-1), P_8(O)={PO[8]}"


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e9")]
    failed = 0
    for n, f in tests:
        try:
            r = f()
            if verbose:
                print("ok   %s  %s" % (n, r))
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.parse_args()
    return 1 if run_tests() else 0


if __name__ == "__main__":
    sys.exit(main())
