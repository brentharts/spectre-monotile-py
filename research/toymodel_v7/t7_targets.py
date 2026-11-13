#!/usr/bin/env python3
"""t7_targets.py - Stage 0 of toymodel_v7: the scoreboard.  Each missing feature of the plan as one number, its target, and the stage of v7 that works on it.

Cheap entries are recomputed from the modules that produced them; expensive ones (minutes) are recorded with the test that produced them and recomputed only with --full.  Every later stage of v7 updates its entry.
A row is 'met' only when its target is reached by the stated computation; 'open' otherwise.  Nothing here is new physics: it is bookkeeping that makes the gaps numbers.

    python3 t7_targets.py            # the table
    python3 t7_targets.py --tests-only
    python3 t7_targets.py --full     # also recompute the recorded entries (several minutes)
"""
import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_RES)
for _p in (_HERE, os.path.join(_RES, "toymodel_v6"), os.path.join(_RES, "toymodel_v4"), os.path.join(_RES, "toymodel_v3"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _deflection():
    import t3_optics as vo
    return float(vo.coefficients(vo.index_from(0.0))[0])


def _bell_two():
    import t7_bell as b
    import t6_pair as t0
    psi, _, _ = b.two_pair_state(0.0)
    return (1 - t0.overlap(0.0) ** 2) * b.chsh_plane(b.corr(psi[:, None] * psi.conj()[None]))


def _bell_patch():
    import t7_bell as b
    return max(b.cluster_scan(n)[1] for n in b.CLUSTERS)


def _horizon():
    return math.exp(-0.5)        # seam rate exp(-r_s / 2r) at r = r_s (v6 E6.26): the overlap does not reach 0


def _c7_second_order():
    r = 2.0
    return math.exp(-1 / (2 * r)) / math.sqrt(1 - 1 / r) - 1      # 0.101: first order passes, second does not


def _fields():
    inflation = {"6", "10", "15"}          # quadratic subfields of Q(sqrt6, sqrt10) (paper, Result res:field)
    coxeter = {"5", "-3", "-15"}           # quadratic subfields of Q(zeta_30): conductors dividing 15
    return len(inflation & coxeter)


def _chair_a4():
    import t6_hop as h
    return float(h.run("chair44", 1, h.CHAIR_T)["A4"][-1])


def _spectre_beta():
    import numpy as np
    import t6_hop as h
    r = h.run("spectre", 1, h.SPEC_T)
    return float(np.polyfit(np.log(h.SPEC_T[1:]), np.log(h.rms(r["S"])[1:]), 1)[0])


# name: (description, target text, test of the target, stage, function or None, recorded value, source)
ROWS = [
    ("deflection", "light-bending coefficient (units r_s / b)", "2", lambda v: abs(v - 2) < 0.05, 4, _deflection, None, "t3_optics, clock only"),
    ("space_s1", "bending coefficient with the tile's own space factor, field sets the opening (level 4)", "2", lambda v: abs(v - 2) < 0.05, 4, None, 0.9504, "t7_space E7.13 (Negative: below the clock alone)"),
    ("space_s2", "fall-off exponent of the bending, field sets the flow time", "-1", lambda v: abs(v + 1) < 0.05, 4, None, -0.527, "t7_space E7.14 (Negative: wrong order)"),
    ("c7", "seam-rate clock against sqrt(1 - r_s/r) at r = 2 r_s (relative)", "0", lambda v: abs(v) < 0.01, 6, _c7_second_order, None, "v6 E6.26: first order only"),
    ("horizon", "overlap c at r = r_s (rule I, seam bits)", "0", lambda v: v < 1e-3, 6, _horizon, None, "v6 E6.26"),
    ("c_floor", "least overlap any tile can represent (reflex valid up to the critical angle)", "0", lambda v: v < 1e-3, 6, None, 0.0609, "t7_horizon E7.15: exp(-h*^2/2), h* = (3 + sqrt3)/2"),
    ("bell_two", "CHSH, two pairs, tile settings, visibility, nothing discarded", "> 2", lambda v: v > 2, 1, _bell_two, None, "t7_bell E7.2"),
    ("bell_patch", "CHSH, patch clusters, tile settings, visibility", "> 2", lambda v: v > 2, 1, _bell_patch, None, "t7_bell E7.4"),
    ("tsirelson", "largest CHSH of any state of the v6 Hamiltonians", "<= 2 sqrt2", lambda v: v <= 2 * math.sqrt(2) + 1e-12, 1, None, 2 * math.sqrt(2), "automatic (v6 E6.3)"),
    ("fields", "quadratic subfields shared by the inflation field and Q(zeta_30)", ">= 1", lambda v: v >= 1, 2, _fields, None, "t7_fields E7.6 (Negative)"),
    ("e8_both", "E8 carries the Eisenstein and golden structures through one commuting family", "yes", lambda v: bool(v), 2, None, 1, "t7_fields E7.5 (Coxeter element)"),
    ("lambda_sq", "lambda^2 lies in Q(sqrt3, sqrt5), the compositum of the two sides", "yes", lambda v: bool(v), 2, None, 1, "t7_fields E7.7: lambda^2 = 4 + sqrt3 sqrt5"),
    ("rank_both", "least rank carrying the 12-fold and 5-fold rotations together", "8 (E8)", lambda v: v == 8, 2, None, 16, "t7_fields E7.8: phi(60) = 16"),
    ("layout", "layouts of the 192 tips that pass the stated rules", "exactly 1", lambda v: v == 1, 3, None, 1, "t7_layout E7.10: 12 x 13 + 6 x 6"),
    ("layout_mono", "net bit source of the selected layout (the 1/r clock needs a nonzero one)", "!= 0", lambda v: v != 0, 3, None, 0, "t7_layout E7.11 (Negative for C7)"),
    ("isotropy", "fourth-moment index of a hopping packet started on one site, Chair44 (isotropic 0.6)", "0.6", lambda v: abs(v - 0.6) < 0.02, 5, _chair_a4, 0.4774, "v6 E6.14 (lattice-scale: does not shrink, t7_lorentz E7.19)"),
    ("longwave", "anisotropy of the long-wave speed of u_tt = -L u on the Chair44 (ball R = 40)", "< 1e-3", lambda v: v < 1e-3, 5, None, 5.6e-5, "t7_lorentz E7.18"),
    ("gamma", "moving clock against 1/gamma, long wavelength (k = 0.2)", "< 1e-3", lambda v: abs(v) < 1e-3, 5, None, 3e-4, "t7_lorentz E7.21 (second-order form only)"),
    ("transport", "spreading exponent on the Spectre graph (ballistic 1)", "1", lambda v: abs(v - 1) < 0.05, 5, _spectre_beta, 0.559, "v6 E6.13"),
    ("entropy", "cells of the (omega0, k0) grid where the bits' entropy grows along the flow", "> 0", lambda v: v > 0, 7, None, 0, "v4 res:flow-entropy (non-increasing or rise-then-fall)"),
    ("quench", "entanglement entropy grown by unitary dynamics from the frozen state (12 pairs, tau = 0.3, bits)", "> 0", lambda v: v > 0, 7, None, 2.899, "t7_entropy E7.24 (initial condition put in)"),
    ("janus", "|S(t) - S(-t)| in the quench: the arrow points away from the initial moment both ways", "> 0 for an arrow", lambda v: v > 0, 7, None, 0.0, "t7_entropy E7.23 (exactly symmetric)"),
    ("arrow", "T-odd terms of the v6 Hamiltonians not put in by hand", ">= 1", lambda v: v >= 1, 7, None, 0, "t7_entropy E7.22 (Negative: all real)"),
    ("rg", "the pair model closes under the substitution (pair set by a local rule; inherited couplings x lambda^-3)", "yes", lambda v: bool(v), 8, None, 1, "t7_rg E7.26 to E7.28 (trivial fixed point for the inherited part)"),
    ("rg_table", "boundary types of the local pair rule first seen at the last level computed (level 4)", "0 (table complete)", lambda v: v == 0, 8, None, 120, "t7_rg E7.27: 120 of 194 new at level 4"),
]
EXPECTED = {"deflection": 1.00, "c7": 0.1014, "horizon": 0.6065, "bell_two": 2.4543, "bell_patch": 1.9880, "fields": 0, "isotropy": 0.4774, "transport": 0.559}


def table(full=False):
    out = []
    for name, desc, target, ok, stage, fn, rec, src in ROWS:
        if fn is not None and (rec is None or full):
            v = fn()
        else:
            v = rec
        status = "pending" if v is None else ("met" if ok(v) else "open")
        out.append((name, desc, v, target, status, stage, src))
    return out


def test_e70_the_scoreboard_reproduces_its_numbers():
    rows = {r[0]: r for r in table()}
    for k, want in EXPECTED.items():
        got = rows[k][2]
        assert abs(got - want) < (0.02 if k == "deflection" else 1e-3), (k, got, want)
    assert rows["bell_two"][4] == "met" and rows["bell_patch"][4] == "open" and rows["tsirelson"][4] == "met"
    assert rows["deflection"][4] == "open" and rows["horizon"][4] == "open" and rows["fields"][4] == "open"
    assert rows["layout"][4] == "met" and rows["layout_mono"][4] == "open"
    assert rows["e8_both"][4] == "met" and rows["lambda_sq"][4] == "met" and rows["rank_both"][4] == "open"
    return {k: v[4] for k, v in rows.items()}


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e7")]
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


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--full", action="store_true")
    a = ap.parse_args(argv)
    if a.tests_only:
        return 1 if run_tests() else 0
    print("%-11s %-8s %-11s %-7s %s" % ("entry", "value", "target", "status", "stage / source"))
    for name, desc, v, target, status, stage, src in table(a.full):
        vs = "-" if v is None else ("%.4g" % v if isinstance(v, float) else str(v))
        print("%-11s %-8s %-11s %-7s %d  %s  [%s]" % (name, vs, target, status, stage, desc, src))
    return 0


if __name__ == "__main__":
    sys.exit(main())
