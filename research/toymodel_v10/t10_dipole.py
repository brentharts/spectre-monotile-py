#!/usr/bin/env python3
"""t10_dipole.py - frustrated dipolar pair-qubit Ising model on Spectre flat pairs,
beyond N = 14 (Result 17).  Pure Python/numpy rebuild of the C++20 Gray-code solver
of J. Lockwood (toymodel_v9 pull request, not merged); no compiled code.

Model (as in Result 17 and in Lockwood's solver):
  W_AB = 1/|A1-A2| - 1/|A1-B2| - 1/|B1-A2| + 1/|B1-B2|  (dipole-dipole, pair = two flat-vertex tiles),
  E(s) = sum_{i<j} W_ij s_i s_j,  f_unsat = (E_min + sum_{i<j}|W_ij|) / (2 sum_{i<j}|W_ij|).
Pairs are ordered by distance of the pair midpoint from the patch centroid; "cluster N" = the N innermost.

Exact method: meet in the middle.  Fix the last spin (Z2), split the rest into a low and a high
block, E = E_low(a) + E_high(b) + a^T W_lh b, evaluated for all 2^(N-1) states by blocked
matrix products (no Gray code, no C++).  Heuristic method: vectorised simulated annealing over
replicas plus single-flip descent, validated against the exact result.

Labels: Exact (N <= 28, floating point tolerance 1e-9 for degeneracy), Numeric (SA upper bounds).

    SPECTRE_DIR=/path/to/spectre python3 t10_dipole.py --tests-only   # about 1 min
    SPECTRE_DIR=... python3 t10_dipole.py --full                      # N=28 exact + SA to L4 (minutes)
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
for _p in (os.path.join(os.environ.get("SPECTRE_DIR", ""), "boundary_certificate"),
           os.path.join(_HERE, "..", "..", "..", "spectre", "boundary_certificate"),
           os.path.join(_HERE, "..", "repos", "spectre", "boundary_certificate")):
    if _p and os.path.isfile(os.path.join(_p, "spectre_exact.py")):
        sys.path.insert(0, _p)
        break
import spectre_exact as se  # noqa: E402

FLAT_IDX = 10
SQRT3 = math.sqrt(3.0)
_GEOM = {}


def z4_to_xy(v):
    p, q, r, s = (int(x) for x in v)
    return (p + 0.5 * q + 0.5 * SQRT3 * r, 0.5 * SQRT3 * q + 0.5 * r + s)


def _centroid(pts):
    pts = np.asarray(pts, float)
    x, y = pts[:, 0], pts[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    return np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (3.0 * cr.sum())


def pair_geometry(level):
    """Flat-vertex pairs of the level-`level` patch: (pairs, centroids C, order by midpoint radius)."""
    if level in _GEOM:
        return _GEOM[level]
    raw = se.build(level)[level][0]["Delta"]
    v_at = collections.defaultdict(list)
    cents, flat_v = [], []
    for i, (T, _k) in enumerate(raw):
        poly = [tuple(int(x) for x in v) for v in se.outline(T)]
        flat_v.append(poly[FLAT_IDX])
        for vi, v in enumerate(poly):
            v_at[v].append((i, vi))
        cents.append(_centroid([z4_to_xy(v) for v in poly]))
    C = np.array(cents)
    pairs = []
    for i, fv in enumerate(flat_v):
        oth = [x for x in v_at[fv] if x[0] != i]
        if len(oth) == 1 and oth[0][1] == FLAT_IDX and i < oth[0][0]:
            pairs.append((i, oth[0][0]))
    mids = np.array([0.5 * (C[a] + C[b]) for a, b in pairs])
    order = np.argsort(np.linalg.norm(mids - C.mean(axis=0), axis=1), kind="stable")
    _GEOM[level] = (pairs, C, order)
    return _GEOM[level]


def dipole_W(level, n):
    pairs, C, order = pair_geometry(level)
    sub = [pairs[i] for i in order[:n]]
    A = C[[p[0] for p in sub]]
    B = C[[p[1] for p in sub]]

    def inv(X, Y):
        d = np.linalg.norm(X[:, None] - Y[None], axis=2)
        with np.errstate(divide="ignore"):
            w = 1.0 / d
        w[~np.isfinite(w)] = 0.0
        return w

    W = inv(A, A) - inv(A, B) - inv(B, A) + inv(B, B)
    np.fill_diagonal(W, 0.0)
    return W


def _all_states(m):
    """All 2^m spin vectors (rows, +-1 as float32 would lose exactness: use float64)."""
    idx = np.arange(1 << m, dtype=np.int64)
    return 1.0 - 2.0 * ((idx[:, None] >> np.arange(m)) & 1)


def exact_ground_state(W, block=512):
    """Exact minimum of E = sum_{i<j} W_ij s_i s_j with s_{N-1} = +1 fixed.
    Returns (E_min, degeneracy modulo the global flip, spins of one minimiser)."""
    N = W.shape[0]
    M = N - 1
    L = M // 2
    H = M - L
    Wll, Whh, Wlh = W[:L, :L], W[L:M, L:M], W[:L, L:M]
    wf = W[L:M, M]                       # coupling of the high block to the fixed spin (low block: below)
    a = _all_states(L)
    b = _all_states(H)
    El = 0.5 * np.einsum("ni,ij,nj->n", a, Wll, a) + a @ W[:L, M]   # includes coupling to the fixed spin
    Eh = 0.5 * np.einsum("ni,ij,nj->n", b, Whh, b) + b @ wf
    G = a @ Wlh                          # (2^L, H)
    best, deg, arg = np.inf, 0, None
    tol = 1e-9
    for j0 in range(0, len(b), block):
        E = El[:, None] + (Eh[j0:j0 + block] + 0.0)[None, :] + G @ b[j0:j0 + block].T
        m = E.min()
        if m < best - tol:
            best, deg = m, 0
        if abs(m - best) <= tol:
            hit = np.argwhere(np.abs(E - best) <= tol)
            deg += len(hit)
            if arg is None or m < best - tol or arg[0] != best:
                i, jj = hit[0]
                arg = (best, i, j0 + jj)
    s = np.concatenate([a[arg[1]], b[arg[2]], [1.0]])
    return float(best), int(deg), s


def energy(W, s):
    return 0.5 * float(s @ W @ s)


def frac_unsat(W, E):
    tot = float(np.abs(np.triu(W, 1)).sum())
    return (E + tot) / (2.0 * tot)


def sa_ground_state(W, restarts=64, sweeps=300, seed=1):
    """Vectorised multi-start simulated annealing + single-flip descent (upper bound on E_min)."""
    N = W.shape[0]
    rng = np.random.default_rng(seed)
    s = rng.choice([-1.0, 1.0], size=(restarts, N))
    h = s @ W                            # local fields, W symmetric with zero diagonal
    scale = float(np.abs(W).sum(axis=1).mean())
    T0, T1 = 0.6 * scale, 0.01 * scale
    ratio = (T1 / T0) ** (1.0 / max(sweeps - 1, 1))
    T = T0
    for _ in range(sweeps):
        for i in range(N):
            dE = -2.0 * s[:, i] * h[:, i]
            acc = (dE <= 0) | (rng.random(restarts) < np.exp(-np.maximum(dE, 0) / T))
            ds = np.where(acc, -2.0 * s[:, i], 0.0)
            s[:, i] += ds
            h += ds[:, None] * W[i][None, :]
        T *= ratio
    for _ in range(50):                  # greedy descent
        moved = False
        for i in range(N):
            dE = -2.0 * s[:, i] * h[:, i]
            acc = dE < -1e-12
            if acc.any():
                moved = True
                ds = np.where(acc, -2.0 * s[:, i], 0.0)
                s[:, i] += ds
                h += ds[:, None] * W[i][None, :]
        if not moved:
            break
    E = 0.5 * np.einsum("ri,ri->r", s, h)
    k = int(np.argmin(E))
    hits = int((np.abs(E - E[k]) < 1e-9).sum())
    return float(E[k]), hits, s[k].copy()


# ---------------------------------------------------------------- tests
def test_e106_matches_result17():
    """Level 2 (N=6) and Level 3 central clusters N=6..14 against Result 17, deg = 1 (Exact)."""
    W2 = dipole_W(2, 6)
    E, deg, _ = exact_ground_state(W2)
    assert abs(frac_unsat(W2, E) - 0.060198) < 1e-4 and deg == 1
    assert len(pair_geometry(3)[0]) == 82
    out = []
    for n, ref in ((6, 0.128), (8, 0.136), (10, 0.150), (12, 0.156), (14, 0.165)):
        W = dipole_W(3, n)
        E, deg, s = exact_ground_state(W)
        f = frac_unsat(W, E)
        assert abs(f - ref) < 1e-3 and deg == 1, (n, f, deg)
        assert abs(energy(W, s) - E) < 1e-9          # minimiser really attains E
        out.append("%d:%.4f" % (n, f))
    return "L2 0.0602; L3 " + " ".join(out) + ", all deg 1"


def test_e107_exact_to_N22():
    """Exact meet-in-the-middle at N = 16..22 agrees with brute-force enumeration of a smaller case and
    with Lockwood's reported values (as listed in lockwood_reported/, not recomputed there)."""
    # internal cross-check: N=12 brute force over all 2^11 states
    W = dipole_W(3, 12)
    S = np.concatenate([_all_states(11), np.ones((1 << 11, 1))], axis=1)
    Eb = 0.5 * np.einsum("ni,ij,nj->n", S, W, S)
    E, deg, _ = exact_ground_state(W)
    assert abs(Eb.min() - E) < 1e-9 and deg == int((np.abs(Eb - Eb.min()) < 1e-9).sum())
    rep = json.load(open(os.path.join(_HERE, "lockwood_reported", "results_dipole_scaling.json")))
    ref = {r["N"]: r["frac_unsat"] for r in rep["level3_exact"]}
    out = []
    for n in (16, 18, 20, 22):
        W = dipole_W(3, n)
        E, deg, _ = exact_ground_state(W)
        f = frac_unsat(W, E)
        assert deg == 1 and abs(f - ref[n]) < 1e-5, (n, f, ref[n], deg)
        out.append("%d:%.6f" % (n, f))
    return " ".join(out) + " (deg 1; equal to the reported values)"


def test_e108_sa_matches_exact():
    """SA reaches the exact minimum at N = 14 and 20 (validation of the heuristic)."""
    out = []
    for n in (14, 20):
        W = dipole_W(3, n)
        E, _, _ = exact_ground_state(W)
        Es, hits, _ = sa_ground_state(W, restarts=64, sweeps=200)
        assert abs(Es - E) < 1e-9, (n, Es, E)
        out.append("%d hits %d/64" % (n, hits))
    return "SA = exact at N=" + ", ".join(out)


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e10")]
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


def full():
    res = {"level3_exact": [], "level3_sa": [], "level4_sa": []}
    for n in (24, 26, 28):
        W = dipole_W(3, n)
        t = time.time()
        E, deg, _ = exact_ground_state(W)
        dt = time.time() - t
        print("exact N=%d f=%.6f E=%+.6f deg=%d (%.1fs)" % (n, frac_unsat(W, E), E, deg, dt), flush=True)
        res["level3_exact"].append({"N": n, "E_min": E, "frac_unsat": frac_unsat(W, E), "count": deg, "t_s": round(dt, 2)})
    for lvl, ns, rst, sw, key in ((3, (32, 50, 82), 128, 300, "level3_sa"), (4, (150, 400, 804), 32, 150, "level4_sa")):
        for n in ns:
            W = dipole_W(lvl, n)
            t = time.time()
            E, hits, _ = sa_ground_state(W, restarts=rst, sweeps=sw)
            dt = time.time() - t
            print("SA L%d N=%d f<=%.6f hits %d/%d (%.1fs)" % (lvl, n, frac_unsat(W, E), hits, rst, dt), flush=True)
            res[key].append({"N": n, "E_min": E, "frac_unsat": frac_unsat(W, E), "count": hits, "restarts": rst, "t_s": round(dt, 2)})
    json.dump(res, open(os.path.join(_HERE, "results_dipole_py.json"), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--full", action="store_true")
    a = ap.parse_args()
    if a.full:
        full()
    else:
        sys.exit(1 if run_tests() else 0)
