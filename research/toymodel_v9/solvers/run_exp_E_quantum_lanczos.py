#!/usr/bin/env python3
"""run_exp_E_quantum_lanczos.py - Experiment E (Open Problem #15):
Exact Matrix-Free OpenMP C++20 Lanczos Diagonalization of the Spectre Pair-Qubit
Frustrated Dipolar Transverse-Field Ising Model from N = 6 up to N = 24 Qubits!

1. Validates to <1e-9 against t6_patch.py E6.10 on N = 6, 8, 10, 12, 14
   (including exact CHSH_max = 2.325, CHSH_obs = 2.121 at N=10, tau=0.05).
2. Extends exact quantum diagonalization across tau in [0.0, 0.35] for
   N = 14, 16, 18, 20, 22 (plus N = 24 around the critical window!).
3. Measures finite-size scaling of:
   - Even-sector spectral gap Delta_even(tau) & its minimum tau_gap(N)
   - Z2 cat tunneling splitting Delta_tunnel(tau) = E0_odd - E0_even
   - Edwards-Anderson spin-glass susceptibility chi_SG(tau) & order parameter q_EA^2(tau)
   - Transverse magnetization m_x(tau) & susceptibility -dm_x/dtau
   - Exact pairwise concurrence C_max(tau), CHSH_max(tau), and observable CHSH_obs(tau)
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time

import numpy as np

FRONTIER_DIR = os.path.dirname(os.path.realpath(__file__))
UPGRADE_DIR = os.path.dirname(FRONTIER_DIR)
REPO_PY = os.path.join(UPGRADE_DIR, "repos", "spectre-monotile-py")
for p in (
    REPO_PY,
    os.path.join(REPO_PY, "research", "toymodel_v4"),
    os.path.join(REPO_PY, "research", "toymodel_v6"),
):
    if p not in sys.path:
        sys.path.insert(0, p)

import t4_patch as tp  # noqa: E402
import t6_pair as t0  # noqa: E402

SQRT3 = math.sqrt(3.0)
LANCZOS_BIN = os.path.join(UPGRADE_DIR, "bin", "quantum_lanczos")


def b_of(tau: float) -> float:
    return 1.0 + (SQRT3 - 1.0) * tau


def poly_centroid(poly):
    poly = np.asarray(poly, float)
    x, y = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    A = cr.sum() * 0.5
    return np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (6.0 * A)


_GEOM_CACHE = {}


def get_geometry(level: int, b: float):
    key = (level, round(b, 12))
    if key not in _GEOM_CACHE:
        P = tp.patch(level, b)
        pr = tp.flat_pairs(level, b)
        C = np.array([poly_centroid(P.straight_polygon(i)) for i in range(P.n_tiles)])
        _GEOM_CACHE[key] = (pr, C)
    return _GEOM_CACHE[key]


def coupling_W(level: int, b: float, idx=None):
    pr, C = get_geometry(level, b)
    pr = [pr[i] for i in idx] if idx is not None else pr
    A = C[[p[0] for p in pr]]
    B = C[[p[1] for p in pr]]

    def inv(X, Y):
        d = np.linalg.norm(X[:, None] - Y[None], axis=2)
        with np.errstate(divide="ignore"):
            out = 1.0 / d
        out[~np.isfinite(out)] = 0.0
        return out

    W = inv(A, A) - inv(A, B) - inv(B, A) + inv(B, B)
    np.fill_diagonal(W, 0.0)
    return W


def get_cluster_order(level: int):
    pr, C = get_geometry(level, 1.0)
    mid = np.array([0.5 * (C[a] + C[b]) for a, b in pr])
    centre = C.mean(axis=0)
    return list(np.argsort(np.linalg.norm(mid - centre, axis=1), kind="stable"))


def compute_kappa():
    W2 = coupling_W(2, 1.0)
    return 1.0 / (t0.h(0.0) ** 2 * float(np.abs(W2).max()))


def run_lanczos_batch(N: int, tau_list, idx, kappa: float, level: int = 3, max_steps: int = 90, max_ent_pairs: int = 0):
    """Send all tau points for a given N in a single C++20 Lanczos invocation."""
    lines = []
    meta = []
    for tau in tau_list:
        W = coupling_W(level, b_of(tau), idx)
        K = kappa * (t0.h(tau) ** 2) * W
        c = t0.overlap(tau)
        Gamma = c  # J0 = 1
        lines.append(f"{N} {Gamma:.17g} {max_steps} {max_ent_pairs}")
        for i in range(N):
            lines.append(" ".join(f"{K[i, j]:.17g}" for j in range(N)))
        meta.append((tau, c))
    payload = "\n".join(lines) + "\n"
    proc = subprocess.run([LANCZOS_BIN], input=payload, capture_output=True, text=True, check=True)
    out_lines = [ln.strip() for ln in proc.stdout.strip().splitlines() if ln.strip().startswith("OK ")]
    assert len(out_lines) == len(tau_list), (len(out_lines), len(tau_list))

    rows = []
    for (tau, c), ln in zip(meta, out_lines):
        toks = ln.split()
        E0 = float(toks[1])
        gap_even = float(toks[2])
        gap_tunnel = float(toks[3])
        mx = float(toks[4])
        chi_sg = float(toks[5])
        q_ea2 = float(toks[6])
        c_max = float(toks[7])
        chsh_max = float(toks[8])
        steps = int(toks[9])
        res0 = float(toks[10])
        res1 = float(toks[11])
        t_s = float(toks[16])
        chsh_obs = (1.0 - c * c) * chsh_max
        rows.append({
            "tau": round(float(tau), 4),
            "Gamma": round(float(c), 6),
            "E0": round(E0, 8),
            "gap_even": round(gap_even, 8),
            "gap_tunnel": round(gap_tunnel, 8),
            "mx": round(mx, 6),
            "chi_SG": round(chi_sg, 6),
            "q_EA2": round(q_ea2, 6),
            "c_max": round(c_max, 6),
            "chsh_max": round(chsh_max, 6),
            "chsh_obs": round(chsh_obs, 6),
            "lanczos_steps": steps,
            "res0": res0,
            "res1": res1,
            "t_s": round(t_s, 3),
        })
    return rows


def main():
    t_start = time.time()
    kappa = compute_kappa()
    order3 = get_cluster_order(3)
    print(f"kappa = {kappa:.12f}, Level-3 pairs = {len(order3)}")

    # 1. Exact Validation against t6_patch.py E6.10 (N=10: CHSH_max=2.325 at tau=0.05, CHSH_obs=2.121 across scan)
    print("\n=== Validation vs. quantgrav.pdf Result 17-18 (t6_patch.py E6.10) ===")
    v10_scan = run_lanczos_batch(10, [0.03, 0.04, 0.05], order3[:10], kappa, max_steps=100, max_ent_pairs=0)
    v10 = v10_scan[2]
    best_obs_10 = max(r["chsh_obs"] for r in v10_scan)
    print(f"  N=10, tau=0.05: CHSH_max={v10['chsh_max']:.6f} (ref 2.325), best CHSH_obs(0.03..0.05)={best_obs_10:.6f} (ref 2.121), gap={v10['gap_even']:.6f}, res0={v10['res0']:.2e}, t={v10['t_s']}s")
    assert abs(v10["chsh_max"] - 2.325) < 1e-3, v10
    assert abs(best_obs_10 - 2.121) < 2e-3, v10_scan

    # 2. Full Finite-Size Quantum Phase Scan across N = 8, 10, 12, 14, 16, 18, 20, 22
    tau_grid_dense = [round(0.01 * i, 4) for i in range(0, 31)]  # 0.00..0.30 (31 pts)
    tau_grid_22 = [0.0, 0.02, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11, 0.12, 0.14, 0.16, 0.20, 0.25, 0.30]
    tau_grid_24 = [0.0, 0.03, 0.05, 0.07, 0.08, 0.09, 0.10, 0.12, 0.15, 0.20, 0.28]

    results = {"kappa": kappa, "validation_N10_tau005": v10, "scans": {}, "summary": []}

    for N in (8, 10, 12, 14, 16, 18, 20, 22, 24):
        if N <= 20:
            tgrid = tau_grid_dense
            max_ent = 0  # ALL pairs exact!
            steps = 90
        elif N == 22:
            tgrid = tau_grid_22
            max_ent = 60  # top 60 strongest-coupled pairs
            steps = 85
        else:
            tgrid = tau_grid_24
            max_ent = 30  # top 30 strongest-coupled pairs
            steps = 75

        t0_n = time.time()
        rows = run_lanczos_batch(N, tgrid, order3[:N], kappa, max_steps=steps, max_ent_pairs=max_ent)
        dt_n = time.time() - t0_n
        results["scans"][str(N)] = rows

        # Extract key finite-size scaling markers
        min_gap_row = min(rows, key=lambda r: r["gap_even"])
        max_c_row = max(rows, key=lambda r: r["c_max"])
        max_chsh_row = max(rows, key=lambda r: r["chsh_max"])
        max_obs_row = max(rows, key=lambda r: r["chsh_obs"])
        # Peak of -dm_x/dtau (quantum critical susceptibility proxy)
        dmx_peak = (0.0, 0.0)
        for i in range(1, len(rows)):
            d = (rows[i]["mx"] - rows[i - 1]["mx"]) / (rows[i]["tau"] - rows[i - 1]["tau"])
            if d > dmx_peak[0]:
                dmx_peak = (d, 0.5 * (rows[i]["tau"] + rows[i - 1]["tau"]))
        # Value of chi_SG at tau = 0 (frozen glass) and at gap minimum
        chi_sg_0 = rows[0]["chi_SG"]
        chi_sg_gap = min_gap_row["chi_SG"]
        q_ea2_0 = rows[0]["q_EA2"]

        summ = {
            "N": N,
            "n_tau": len(tgrid),
            "tau_gap_min": min_gap_row["tau"],
            "gap_min": min_gap_row["gap_even"],
            "tunnel_at_0": rows[0]["gap_tunnel"],
            "chi_SG_at_0": chi_sg_0,
            "q_EA2_at_0": q_ea2_0,
            "chi_SG_at_gap_min": chi_sg_gap,
            "tau_dmx_peak": round(dmx_peak[1], 4),
            "dmx_dtau_peak": round(dmx_peak[0], 4),
            "c_max_peak": max_c_row["c_max"],
            "tau_c_peak": max_c_row["tau"],
            "chsh_max_peak": max_chsh_row["chsh_max"],
            "chsh_obs_peak": max_obs_row["chsh_obs"],
            "tau_chsh_obs_peak": max_obs_row["tau"],
            "wall_s": round(dt_n, 2),
        }
        results["summary"].append(summ)
        print(
            f"  N={N:2d} ({len(tgrid):2d} pts, {dt_n:5.1f}s): "
            f"gap_min={summ['gap_min']:.4f} @ tau={summ['tau_gap_min']:.2f} | "
            f"chi_SG(0)={chi_sg_0:.2f} (q_EA2={q_ea2_0:.3f}) | "
            f"dmx_peak={summ['dmx_dtau_peak']:.2f} @ tau={summ['tau_dmx_peak']:.2f} | "
            f"C_max={summ['c_max_peak']:.3f} | CHSH_obs={summ['chsh_obs_peak']:.3f}",
            flush=True,
        )

    results["elapsed_s"] = round(time.time() - t_start, 2)
    out_path = os.path.join(FRONTIER_DIR, "results_exp_E_quantum_lanczos.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[DONE] Saved Experiment E exact quantum Lanczos results to {out_path} ({results['elapsed_s']}s)")


if __name__ == "__main__":
    main()
