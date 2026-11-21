#!/usr/bin/env python3
"""t9_cavity_rigidity.py - Drop-in self-testing module for brentharts/spectre-monotile-py
(research/toymodel_v9/) checking all exact SAT/DRAT cavity rigidity, erasure code,
relaxed-SAT defect, and 3D Chair44 results (Upgrades 1 + Experiments A, B, D).

Exact claims (each a test; ``python3 t9_cavity_rigidity.py --tests-only``):
  E9.9   All 12 Spectre orientations on Level 4 are 1-tile erasure unique under BOTH
         unrestricted exact (1/2)Z[sqrt3] MRV search and Result-66 DRAT-certified SAT.
  E9.10  Zero chiral closed flip clusters exist up to 300 erased tiles (r = 27.96,
         zero hull assumption) and 180 erased tiles DRAT-certified (12.84M clauses).
  E9.11  Breaking chirality (allow_reflect=True) immediately destroys rigidity: a
         2-tile closed swap (rem=2, add=2, dV=dE=dF=0) appears inside a 4-tile cavity.
  E9.12  Relaxed-SAT minimal non-trivial chiral rearrangement defect is constant
         D_min = 20 across k = 6, 10, 16, 20 (DRAT-certified UNSAT at D <= 19),
         realized by a localized two-pocket vacancy-dipole (rem=2, add=1, comps=[12,8]).
  E9.13  Random i.i.d. tile erasures inside a 160-tile pinned Level-4 disc achieve
         100% unique classical recovery (121/121 trials across p = 0.05..1.00).
  E9.14  3D Chair44 (Depth 3, 512 chairs): erasing all 225 strictly interior chairs
         (1,575 unit cubes, 5.62M clauses) is DRAT-verified UNSAT; even the bare
         undecorated 7-cube chair locks the 225-chair interior cavity rigidly.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_SEARCH = [_HERE, os.path.join(_HERE, "..", "..")]


def _load(name):
    for d in _SEARCH:
        p = os.path.join(d, name)
        if os.path.isfile(p):
            return json.load(open(p))
    raise FileNotFoundError(name)


def test_e99_all_12_orientations_single_erasure():
    d = _load("results_cavity_L4.json")
    arr = d["exp1a_single_all12"]
    assert len(arr) == 12
    assert all(x["A"]["status"] == "UNSAT" and x["B"]["status"] == "UNSAT" and x["B"]["drat_verified"] for x in arr)
    return "12/12 orientations UNSAT (unrestricted) + DRAT-verified"


def test_e910_macroscopic_chiral_rigidity():
    d = _load("results_cavity_L4.json")
    u = d["exp1b_unrestricted_chiral_L4"]
    m = d["exp1d_modelset_drat_L4"]
    max_u = max(u, key=lambda r: r["n_erased"])
    max_m = max(m, key=lambda r: r["n_erased"])
    assert all(r["status"] == "UNSAT" for r in u) and max_u["n_erased"] == 300 and max_u["radius"] > 27.9
    assert all(r["status"] == "UNSAT" and r["drat_verified"] for r in m) and max_m["n_erased"] == 180
    return f"Unrestricted k={max_u['n_erased']} (r={max_u['radius']}); DRAT k={max_m['n_erased']} ({max_m['n_clauses']} clauses)"


def test_e911_chirality_control_2tile_swap():
    d = _load("results_cavity_L4.json")
    hits = [r for r in d["exp1c_reflect_control_L4"] if r["status"] == "SAT"]
    assert len(hits) == 1 and hits[0]["n_erased"] == 4 and hits[0]["n_removed"] == 2 and hits[0]["n_added"] == 2
    assert (hits[0]["dV"], hits[0]["dE"], hits[0]["dF"]) == (0, 0, 0)
    return "reflect=True breaks rigidity at k=4: 2-tile swap (dV,dE,dF)=(0,0,0)"


def test_e912_relaxed_sat_defect_dipole():
    d = _load("results_exp_A_worm.json")
    arr = d["exp_a_worm_defect"]
    assert [r["k"] for r in arr] == [6, 10, 16, 20]
    for r in arr:
        assert r["D_min"] == 20 and r["drat_cert_D_minus_1"] is True
        w = r["witness"]
        assert w["exact_D"] == 20 and w["n_defect_components"] == 2 and w["defect_comp_verts"] == [12, 8]
        assert w["n_removed"] == 2 and w["n_added"] == 1
    return "D_min=20 exact (D=19 DRAT-UNSAT) for k=6..20, 2-pocket dipole [12,8]"


def test_e913_random_erasure_100pct_recovery():
    d = _load("results_exp_B_erasure.json")
    arr = d["exp_b_erasure_threshold"]
    tot = sum(r["n_trials"] for r in arr)
    uniq = sum(r["n_unique"] for r in arr)
    assert tot == 121 and uniq == 121
    return f"{uniq}/{tot} trials (100.0%) uniquely recovered across p=0.05..1.00"


def test_e914_chair44_3d_rigidity():
    d = _load("results_exp_D_chair44.json")
    assert all(r["status"] == "UNSAT" and r["drat_verified"] for r in d["decorated_chair44"])
    all225 = d["all_225_interior"]
    assert all225["decorated"]["status"] == "UNSAT" and all225["decorated"]["drat_verified"] is True
    assert all225["bare"]["status"] == "UNSAT"
    assert all225["decorated"]["n_erased_chairs"] == 225 and all225["decorated"]["n_erased_cubes"] == 1575
    return "3D Chair44 all 225 interior chairs (1,575 cubes) DRAT-UNSAT (bare also UNSAT)"


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
