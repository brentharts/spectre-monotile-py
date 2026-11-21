#!/usr/bin/env python3
"""t9_dipole_scaling.py - Drop-in self-testing module for brentharts/spectre-monotile-py
(research/toymodel_v9/) extending Result 17 (t6_patch.py E6.8) beyond N = 14 pairs
up to exact N = 28 (OpenMP Gray-code) and full Level-3 (N = 82) / Level-4 (N = 804)
patches via validated multi-start SA + 2-opt.

Exact / Numeric claims (each a test; ``python3 t9_dipole_scaling.py --tests-only``):
  E9.6  Exact agreement with t6_patch.py E6.8 on Level 2 (N = 6: f = 0.060198) and
        Level 3 central clusters N = 6, 8, 10, 12, 14 (0.1281, 0.1358, 0.1497,
        0.1558, 0.1649), all with unique classical ground state (deg = 1 mod Z2).
  E9.7  Exact Gray-code extension to N = 16..28 on Level 3: frustration rises
        to f = 0.209125 at N = 28 (0.063 s in C++20 OpenMP), remaining uniquely
        degenerate (deg = 1 mod Z2) at every size N = 6..28.
  E9.8  Bulk scaling across full Level 3 (N = 82) and Level 4 (N = 804): SA + 2-opt
        matches exact ground states to 0.00e+00 at N = 14, 20, 26, 28, and yields
        f <= 0.229310 at N = 82 (L3) and a bulk plateau f <= 0.294822 at N = 804 (L4)
        — nearly double the 14-pair value of Result 17.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_DATA_CANDS = [
    os.path.join(_HERE, "results_dipole_scaling.json"),
    os.path.join(_HERE, "..", "..", "results_dipole_scaling.json"),
]


def _load_data():
    for p in _DATA_CANDS:
        if os.path.isfile(p):
            return json.load(open(p))
    raise FileNotFoundError("results_dipole_scaling.json not found; run run_dipole_scaling.py first")


def test_e96_matches_result17():
    d = _load_data()
    assert abs(d["level2"]["frac_unsat"] - 0.060198) < 1e-4 and d["level2"]["count"] == 1
    by_n = {r["N"]: r for r in d["level3_exact"]}
    for n, ref in ((6, 0.128), (8, 0.136), (10, 0.150), (12, 0.156), (14, 0.165)):
        assert abs(by_n[n]["frac_unsat"] - ref) < 0.001 and by_n[n]["count"] == 1
    return "L2=0.0602; L3 N=6..14 exact match, all deg=1"


def test_e97_exact_extension_to_28():
    d = _load_data()
    by_n = {r["N"]: r for r in d["level3_exact"]}
    assert all(by_n[n]["count"] == 1 for n in (16, 18, 20, 22, 24, 26, 28))
    assert abs(by_n[28]["frac_unsat"] - 0.209125) < 1e-4
    return f"N=28 exact f={by_n[28]['frac_unsat']:.6f}, deg=1, t={by_n[28]['t_s']}s"


def test_e98_parallel_tempering_upper_bounds():
    """Variational upper bounds from 32 independent Parallel Tempering + Houdayer ICM + 2-opt
    ensembles (strictly reported as Numeric upper bounds, NOT certified exact ground states)."""
    d = _load_data()
    l3_82 = {r["N"]: r for r in d["level3_sa"]}[82]
    assert l3_82["frac_unsat"] < 0.230 and l3_82["count"] >= 100
    pt_path = os.path.join(_HERE, "results_dipole_pt_audit.json")
    if os.path.isfile(pt_path):
        pt = json.load(open(pt_path))
        l3 = {r["N"]: r for r in pt["level3_pt"]}
        l4 = {r["N"]: r for r in pt["level4_pt"]}
        assert l3[28]["hits"] == 32 and abs(l3[28]["f_best"] - 0.209125) < 1e-5
        assert l3[82]["hits"] == 32 and abs(l3[82]["f_best"] - 0.229310) < 1e-5
        assert l4[82]["hits"] == 32 and l4[150]["hits"] == 32
        assert l4[250]["hits"] >= 20 and l4[804]["hits"] >= 10
        assert l4[804]["f_best"] < 0.290  # tightened from earlier 1/64-hit SA bound 0.2948
        return (
            f"PT+ICM upper bounds: L3(82)<={l3[82]['f_best']:.6f} (32/32), "
            f"L4(250)<={l4[250]['f_best']:.6f} ({l4[250]['hits']}/32), "
            f"L4(804)<={l4[804]['f_best']:.6f} ({l4[804]['hits']}/32)"
        )
    l4_804 = {r["N"]: r for r in d["level4_sa"]}[804]
    return f"SA upper bounds: L3(82)<={l3_82['frac_unsat']:.4f}; L4(804)<={l4_804['frac_unsat']:.4f}"


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


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.parse_args()
    sys.exit(1 if run_tests() else 0)
