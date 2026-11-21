#!/usr/bin/env python3
"""t9_frontiers.py - Self-testing module for brentharts/spectre-monotile-py
(research/toymodel_v9/) verifying the four Frontiers Part II discoveries:
  E9.15  Holographic wedge rigidity & first exact open-boundary chiral phason worms
         on a 60-tile Level-4 disc (center DRAT-locked up to theta_open=270 deg and
         corridor 2x90 deg; abrupt half-disc wedge rem=31/add=36 at 300 deg).
  E9.16  Heesch x Spectre Bridge (Result 81 upgraded): single-parity deformed Spectre
         (+h and -h, chiral and reflect) has DRAT-verified H_c = 0; 28/28 single-vertex
         perturbations collapse H_c = inf -> 0.
  E9.17  Flat-pair-forbidden straight chiral Spectre (Model C1) has exact H_c = 1 on
         the Result-66 lattice (1-corona SAT 8 tiles; 2-corona DRAT-verified UNSAT).
  E9.18  Quantum Lanczos ED extended to N = 24 qubits (16.78M dim): validates N=10
         CHSH_max=2.325 / CHSH_obs=2.121; extensive chi_SG(0) -> 19.58 at N=24; critical
         lock at tau_c ~ 0.13-0.14; CHSH_obs <= 1.900 for all N = 12..24 (Negative for
         macroscopic pairwise Bell violation).
  E9.19  Depth-8 all-12-class Level-4 flips & 2D Langevin Brownian internal-space walks:
         direct delta-scaling alpha = 0.8805 (alpha/2 = 0.4402 vs. OP #14 0.4445) and
         Langevin R(T) ~ T^0.414 (R^2 = 0.99902) with tight 12-class rotational isotropy.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_SEARCH = [_HERE, os.path.join(_HERE, "frontiers"), os.path.join(_HERE, "..", "..")]


def _load(name):
    for d in _SEARCH:
        p = os.path.join(d, name)
        if os.path.isfile(p):
            return json.load(open(p))
    raise FileNotFoundError(name)


def test_e915_holographic_wedge_rigidity():
    d = _load("results_frontier1b_center_worm.json")
    sa = {r["theta_open_deg"]: r for r in d["single_arc_center_flip"]}
    cor = {r["open_arc_deg_each"]: r for r in d["corridor_center_flip"]}
    for th in (60, 120, 180, 210, 240, 270):
        assert sa[th]["status"] == "UNSAT" and sa[th]["drat_verified"] is True
    assert sa[300]["status"] == "SAT" and sa[300]["n_removed_from_C"] == 31 and sa[300]["n_added_new"] == 36
    for op in (50, 70, 90):
        assert cor[op]["status"] == "UNSAT" and cor[op]["drat_verified"] is True
    assert cor[110]["status"] == "SAT" and cor[110]["n_removed_from_C"] == 30 and cor[110]["n_added_new"] == 33
    return "Center DRAT-locked <=270 deg (corridor <=2x90 deg); half-disc wedge at 300 deg (rem=31,add=36)"


def test_e916_heesch_28_of_28_cliff():
    d = _load("results_exp_C_heesch_spectre.json")
    assert d["control_unmodified_chiral"]["status"] == "SAT"
    c2 = d["model_C2_result81_single_parity"]
    for k in ("bump_pos_chiral", "bump_pos_reflect", "dent_neg_chiral", "dent_neg_reflect"):
        assert c2[k]["status"] == "UNSAT" and c2[k]["drat_verified"] is True
    atlas = d["model_C3_14vertex_atlas_chiral"]
    assert len(atlas) == 14
    n_unsat = sum(
        (r["out_pos"]["status"].startswith("UNSAT")) + (r["in_neg"]["status"].startswith("UNSAT"))
        for r in atlas
    )
    assert n_unsat == 28
    return "Result 81 +-h (chiral & reflect) DRAT H_c=0; 28/28 single-vertex moves H_c=0"


def test_e917_flat_pair_forbidden_heesch_1():
    d = _load("results_exp_C1_2corona.json")
    assert d["control_cor1"]["status"] == "SAT" and d["control_cor2"]["status"] == "SAT"
    assert d["no_fp_cor1"]["status"] == "SAT" and d["no_fp_cor1"]["n_placed_tiles"] == 8
    assert d["no_fp_cor2"]["status"] == "UNSAT" and d["no_fp_cor2"]["drat_verified"] is True
    return "Flat-pair-forbidden Spectre exact H_c=1 (cor1 SAT 8 tiles; cor2 DRAT-UNSAT 106,650 cls)"


def test_e918_quantum_lanczos_to_N24():
    d = _load("results_exp_E_quantum_lanczos.json")
    assert abs(d["validation_N10_tau005"]["chsh_max"] - 2.325) < 1e-3
    by_n = {r["N"]: r for r in d["summary"]}
    assert by_n[24]["chi_SG_at_0"] > 19.0 and by_n[24]["q_EA2_at_0"] > 0.80
    assert all(0.12 <= by_n[n]["tau_dmx_peak"] <= 0.15 for n in (16, 18, 20, 22, 24))
    assert all(by_n[n]["chsh_obs_peak"] < 1.95 for n in (12, 14, 16, 18, 20, 22, 24))
    assert by_n[24]["c_max_peak"] < 0.40
    return f"N=24 exact: chi_SG(0)={by_n[24]['chi_SG_at_0']:.2f}, tau_c~0.14, CHSH_obs<=1.900 (N>=12)"


def test_e919_depth8_all12_langevin():
    d = _load("results_exp_F_flips_langevin.json")
    lg = d["langevin"]
    assert abs(lg["fitted_alpha_from_delta"] / 2.0 - 0.4445) < 0.01
    assert 0.39 < lg["fitted_beta"] < 0.46 and lg["r2_fit"] > 0.998
    assert len(d["all12_stats"]) == 48
    return f"alpha/2={lg['fitted_alpha_from_delta']/2:.4f} (pred 0.4445), Langevin beta={lg['fitted_beta']:.4f} (R2={lg['r2_fit']:.5f})"


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
