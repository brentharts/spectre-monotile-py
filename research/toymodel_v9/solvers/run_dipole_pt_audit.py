#!/usr/bin/env python3
"""run_dipole_pt_audit.py - Rigorous Parallel Tempering + Houdayer ICM + 2-opt audit
of the frustrated dipolar Ising upper bounds at Level 3 (N=28, 82) and Level 4
(N=82, 150, 250, 400, 804), responding directly to review critique #1:
  'At N >= 82, SA hit best energy in only 1/32..1/128 restarts -> only an upper bound.'

Runs 32 independent Parallel Tempering ensembles (each 2 paired ladders of R=24 replicas
with Houdayer isoenergetic cluster moves + exact 1-opt & 2-opt polish) and reports:
  - f_best, f_median, f_worst across 32 INDEPENDENT ensembles
  - independent hit count (out of 32)
  - exact delta vs. previous SA upper bound!
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time

import numpy as np

FRONTIER_DIR = os.path.dirname(os.path.realpath(__file__))
UPGRADE_DIR = os.path.dirname(FRONTIER_DIR)
PT_BIN = os.path.join(UPGRADE_DIR, "bin", "dipole_pt")

sys.path.insert(0, UPGRADE_DIR)
import run_dipole_scaling as rds  # noqa: E402


def run_pt(W, n_ens=32, R=24, n_sweeps=400):
    N = W.shape[0]
    lines = [f"{N} {n_ens} {R} {n_sweeps}"]
    for i in range(N):
        lines.append(" ".join(f"{W[i, j]:.17g}" for j in range(N)))
    env = dict(os.environ, OMP_NUM_THREADS="32")
    proc = subprocess.run([PT_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True, env=env)
    toks = proc.stdout.strip().split()
    assert toks[0] == "PT_RES"
    return {
        "N": int(toks[1]),
        "n_ens": int(toks[2]),
        "hits": int(toks[3]),
        "E_best": float(toks[4]),
        "f_best": float(toks[5]),
        "f_median": float(toks[6]),
        "f_worst": float(toks[7]),
        "t_s": round(float(toks[8]), 2),
    }


def main():
    old = json.load(open(os.path.join(UPGRADE_DIR, "results_dipole_scaling.json")))
    old_l3 = {r["N"]: r["frac_unsat"] for r in old["level3_exact"] + old["level3_sa"]}
    old_l4 = {r["N"]: r["frac_unsat"] for r in old["level4_sa"]}

    p3, C3, _, o3 = rds.build_pair_geometry(3)
    p4, C4, _, o4 = rds.build_pair_geometry(4)

    out = {"level3_pt": [], "level4_pt": []}
    print("=== Parallel Tempering + Houdayer ICM + 2-opt Audit (32 independent ensembles) ===")
    for lvl, pairs, C, order, cases, ref_map in [
        (3, p3, C3, o3, [(28, 32, 20, 400), (82, 32, 24, 600)], old_l3),
        (4, p4, C4, o4, [(82, 32, 24, 600), (150, 32, 24, 800), (250, 32, 28, 2000), (400, 32, 28, 2000), (804, 32, 28, 2000)], old_l4),
    ]:
        key = f"level{lvl}_pt"
        for N, n_ens, R, sw in cases:
            W = rds.dipole_W(pairs, C, order[:N])
            r = run_pt(W, n_ens=n_ens, R=R, n_sweeps=sw)
            r["prev_sa_f"] = round(ref_map[N], 6)
            r["delta_f"] = round(r["f_best"] - ref_map[N], 6)
            out[key].append(r)
            print(
                f"  L{lvl} N={N:3d}: f_best={r['f_best']:.6f} (prev SA={r['prev_sa_f']:.6f}, Δ={r['delta_f']:+.6f}) | "
                f"hits={r['hits']:2d}/{n_ens} | med={r['f_median']:.6f} worst={r['f_worst']:.6f} | t={r['t_s']}s",
                flush=True,
            )

    out_path = os.path.join(FRONTIER_DIR, "results_dipole_pt_audit.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"[DONE] Saved {out_path}")


if __name__ == "__main__":
    main()
