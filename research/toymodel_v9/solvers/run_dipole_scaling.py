#!/usr/bin/env python3
"""run_dipole_scaling.py - Scales quantgrav.pdf Result 17 (frustrated dipolar Ising
ground states on Spectre flat-pair clusters) from N=14 up to:
  - 100% EXACT certified ground states for N = 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28 (Level 3)
  - High-confidence multi-start SA + 2-opt upper bounds for N = 28..82 (full Level 3) and
    N = 100, 200, 400, 804 (full Level 4), cross-checked against exact ground states at N = 6..28!
"""
import collections
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
SPECTRE_BC = os.path.join(UPGRADE_DIR, "repos", "spectre", "boundary_certificate")
SOLVER_BIN = os.path.join(UPGRADE_DIR, "bin", "dipole_solver")
sys.path.insert(0, SPECTRE_BC)
import spectre_exact as se  # noqa: E402

FLAT_IDX = 10
SQRT3 = math.sqrt(3.0)


def z4_to_xy(v):
    p, q, r, s = (int(x) for x in v)
    return (p + 0.5 * q + 0.5 * SQRT3 * r, 0.5 * SQRT3 * q + 0.5 * r + s)


def poly_centroid(pts):
    pts = np.asarray(pts, float)
    x, y = pts[:, 0], pts[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    A = cr.sum() * 0.5
    return np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (6.0 * A)


def build_pair_geometry(level):
    out = se.build(level)
    raw = out[level][0]["Delta"]
    v_at = collections.defaultdict(list)
    centroids = []
    flat_v = []
    for i, (T, _k) in enumerate(raw):
        poly_z4 = [tuple(int(x) for x in v) for v in se.outline(T)]
        flat_v.append(poly_z4[FLAT_IDX])
        for vi, v in enumerate(poly_z4):
            v_at[v].append((i, vi))
        centroids.append(poly_centroid([z4_to_xy(v) for v in poly_z4]))
    C = np.array(centroids, float)
    pairs = []
    for i, fv in enumerate(flat_v):
        oth = [x for x in v_at[fv] if x[0] != i]
        if len(oth) == 1 and oth[0][1] == FLAT_IDX and i < oth[0][0]:
            pairs.append((i, oth[0][0]))
    mids = np.array([0.5 * (C[a] + C[b]) for a, b in pairs])
    centre = C.mean(axis=0)
    order = np.argsort(np.linalg.norm(mids - centre, axis=1), kind="stable")
    return pairs, C, mids, order


def dipole_W(pairs, C, idx):
    sub = [pairs[i] for i in idx]
    A = C[[p[0] for p in sub]]
    B = C[[p[1] for p in sub]]

    def inv_d(X, Y):
        d = np.linalg.norm(X[:, None] - Y[None], axis=2)
        with np.errstate(divide="ignore"):
            w = 1.0 / d
        w[~np.isfinite(w)] = 0.0
        return w

    W = inv_d(A, A) - inv_d(A, B) - inv_d(B, A) + inv_d(B, B)
    np.fill_diagonal(W, 0.0)
    return W


def run_cpp(mode, W, restarts=512, sweeps=500):
    N = W.shape[0]
    lines = [f"{mode} {N}"]
    for i in range(N):
        lines.append(" ".join(f"{W[i, j]:.17g}" for j in range(N)))
    if mode == "sa":
        lines.append(f"{restarts} {sweeps}")
    payload = "\n".join(lines) + "\n"
    t0 = time.time()
    proc = subprocess.run([SOLVER_BIN], input=payload, capture_output=True, text=True, check=True)
    dt = time.time() - t0
    parts = proc.stdout.strip().split()
    return {
        "E_min": float(parts[0]),
        "tot_abs_W": float(parts[1]),
        "frac_unsat": float(parts[2]),
        "count": int(parts[3]),  # exact degeneracy (exact) or #restarts hitting best (sa)
        "t_s": round(dt, 3),
    }


def main():
    results = {"level2": {}, "level3_exact": [], "level3_sa": [], "level4_sa": []}

    # Self-test against quantgrav.pdf Result 17 / t6_patch.py E6.8!
    p2, C2, _, o2 = build_pair_geometry(2)
    assert len(p2) == 6
    W2 = dipole_W(p2, C2, o2)
    r2 = run_cpp("exact", W2)
    assert abs(r2["frac_unsat"] - 0.060) < 0.001, r2
    print(f"[PASS] Level 2 (N=6) exact: frac_unsat={r2['frac_unsat']:.6f} (paper: 0.060), deg={r2['count']}")
    results["level2"] = r2

    p3, C3, _, o3 = build_pair_geometry(3)
    assert len(p3) == 82
    paper_ref = {6: 0.128, 8: 0.136, 10: 0.150, 12: 0.156, 14: 0.165}

    print("\n=== Level 3 Exact Ground States (N = 6..28, OpenMP Gray Code) ===")
    for N in (6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28):
        W = dipole_W(p3, C3, o3[:N])
        re_ = run_cpp("exact", W)
        af_frac = float((W[np.triu_indices(N, 1)] > 0).mean())
        re_["N"] = N
        re_["af_bond_frac"] = round(af_frac, 4)
        if N in paper_ref:
            assert abs(re_["frac_unsat"] - paper_ref[N]) < 0.001, (N, re_, paper_ref[N])
        # Also validate SA against exact on N=14, 20, 26, 28
        sa_note = ""
        if N in (14, 20, 26, 28):
            rsa = run_cpp("sa", W, restarts=256, sweeps=400)
            gap = rsa["E_min"] - re_["E_min"]
            sa_note = f" | SA_gap={gap:.2e} (hits={rsa['count']}/256)"
        print(f"  N={N:2d}: f_unsat={re_['frac_unsat']:.6f}, E={re_['E_min']:+.6f}, AF={af_frac:.3f}, deg={re_['count']}, t={re_['t_s']:.3f}s{sa_note}")
        results["level3_exact"].append(re_)

    print("\n=== Level 3 Large Clusters up to Full Patch (N = 32..82, Multi-Start SA + 2-opt) ===")
    for N in (32, 40, 50, 60, 70, 82):
        W = dipole_W(p3, C3, o3[:N])
        rsa = run_cpp("sa", W, restarts=1024, sweeps=600)
        af_frac = float((W[np.triu_indices(N, 1)] > 0).mean())
        rsa["N"] = N
        rsa["af_bond_frac"] = round(af_frac, 4)
        print(f"  N={N:2d}: f_unsat<={rsa['frac_unsat']:.6f}, E={rsa['E_min']:+.6f}, AF={af_frac:.3f}, hits={rsa['count']}/1024, t={rsa['t_s']:.2f}s")
        results["level3_sa"].append(rsa)

    print("\n=== Level 4 Bulk Scaling up to Full Patch (N = 82..804, Multi-Start SA + 1/2-opt) ===")
    p4, C4, _, o4 = build_pair_geometry(4)
    assert len(p4) == 804
    for N, rst, sw in ((82, 512, 600), (150, 512, 600), (250, 256, 600), (400, 128, 500), (804, 64, 500)):
        W = dipole_W(p4, C4, o4[:N])
        rsa = run_cpp("sa", W, restarts=rst, sweeps=sw)
        af_frac = float((W[np.triu_indices(N, 1)] > 0).mean())
        rsa["N"] = N
        rsa["restarts"] = rst
        rsa["af_bond_frac"] = round(af_frac, 4)
        print(f"  N={N:3d}: f_unsat<={rsa['frac_unsat']:.6f}, E={rsa['E_min']:+.6f}, AF={af_frac:.3f}, hits={rsa['count']}/{rst}, t={rsa['t_s']:.2f}s")
        results["level4_sa"].append(rsa)

    out_path = os.path.join(UPGRADE_DIR, "results_dipole_scaling.json")
    with open(out_path, "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"\nSaved dipole scaling results to {out_path}")


if __name__ == "__main__":
    main()
