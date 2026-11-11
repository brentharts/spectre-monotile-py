#!/usr/bin/env python3
"""t4_geometry.py - Stage 0 of research/toymodel_v4/PLAN.md: what a flip of the bit does to the *geometry* of the tile.

toymodel_v2 and v3 used the chain of spectre_hat_quantum_bit_model.py (a bit with an overlap c(tau)) and a hexagonal history sum.  Neither looks at the outline.
spectre_hat_time_model.py has the outline: the tile at tau with P_8 at base + s*h*(1, 0).  This module measures what a flip changes there.

Definitions
  D0.1  Tile(tau, s) = SpectreHatTime.tile(tau, s), a Bezier outline with 14 vertices; omega(tau) = omega_0 (1 - tau); d(tau) = 2 a tan(omega/2) (jump of P_8).
  D0.2  Corner turn at vertex i: the signed angle between the tangent leaving edge i-1 and the tangent entering edge i, measured on the sampled outline.

Exact claims (each a test; ``python3 t4_geometry.py --tests-only``)
  E0.1  Area: A(tau, +1) - A(tau, -1) = d(tau) (a = 1), for several omega_0 and every tau tested; both equal the flow tile's area at tau = 1 (the Hat).
  E0.2  Gauss-Bonnet: the total turning of both outlines is -2 pi, and the curved edges have zero net turning (each is an S-curve), so all of it sits in the corners.
  E0.3  A flip changes the corner turns at P_7, P_8, P_9 by (-omega, +2 omega, -omega) and nothing else: zero net charge, zero dipole, a pure second difference
        (a 'quadrupole' of curvature).  The bit moves curvature without changing the total.
  E0.4  Validity is not symmetric: the s = +1 outline (convex corner) is a simple closed curve for every tau at omega_0 up to 165 degrees, the s = -1 outline (reflex corner)
        is not near tau = 0 once omega_0 >= 135 degrees (k_0 = 0.3, 0.5, 0.8).  The geometry alone forbids one value of the bit at the Spectre end.

Hypothesis / not done: that the second difference (-1, 2, -1) is a source in any field equation; that the early-end exclusion is a boundary condition of anything.

    python3 t4_geometry.py --tests-only
    python3 t4_geometry.py --story
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from spectre_hat_time_model import SpectreHatTime, is_simple_tile  # noqa: E402

P8 = 7  # corner at the END of edge 7 is the 14th vertex (FLAT_VERTEX = 8 in 1-based labels)


def _wrap(x):
    return (np.asarray(x) + np.pi) % (2 * np.pi) - np.pi


def turning(tile, n=200):
    """(edge_turning, corner_turns): net tangent turning inside each edge, and the turn at the corner ending each edge."""
    S = np.asarray(tile.sample_edges(n))  # (14, n + 1, 2)
    tang = [np.arctan2(*np.diff(S[e], axis=0).T[::-1]) for e in range(S.shape[0])]
    edge = np.array([_wrap(np.diff(t)).sum() for t in tang])
    corner = np.array([_wrap(tang[(e + 1) % len(tang)][0] - tang[e][-1]) for e in range(len(tang))])
    return edge, corner


def test_e01_area_difference_is_the_jump():
    for om in (30.0, 60.0, 120.0):
        m = SpectreHatTime(corner_deg=om)
        for tau in (0.0, 0.25, 0.5, 0.75, 0.9):
            dA = m.tile(tau, 1).area() - m.tile(tau, -1).area()
            assert abs(dA - m.jump(tau)) < 1e-9, (om, tau, dA, m.jump(tau))
        assert abs(m.tile(1.0, 1).area() - m.tile(1.0, -1).area()) < 1e-12


def test_e02_gauss_bonnet_all_turning_is_in_the_corners():
    m = SpectreHatTime(corner_deg=120.0)
    for tau in (0.0, 0.3, 0.6, 0.9):
        for s in (1, -1):
            edge, corner = turning(m.tile(tau, s))
            assert abs(edge.sum() + corner.sum() + 2 * math.pi) < 1e-6, (tau, s)
            assert np.abs(edge).max() < 1e-6, "an edge turns on net"


def test_e03_a_flip_is_a_second_difference_of_corner_turns():
    m = SpectreHatTime(corner_deg=120.0)
    for tau in (0.0, 0.3, 0.6, 0.9):
        w = m.omega(tau)
        _, cp = turning(m.tile(tau, 1))
        _, cn = turning(m.tile(tau, -1))
        diff = cn - cp  # not wrapped: at omega > 90 degrees the change 2 omega exceeds pi
        want = np.zeros(14)
        want[P8 - 1], want[P8], want[P8 + 1] = -w, 2 * w, -w
        assert np.allclose(diff, want, atol=1e-6), (tau, np.degrees(diff))
        assert abs(diff.sum()) < 1e-6 and abs(diff[P8 + 1] - diff[P8 - 1]) < 1e-6  # no charge, no dipole


def test_e04_the_reflex_value_fails_first_and_only_at_large_omega():
    taus = np.linspace(0.0, 1.0, 21)
    for k0 in (0.3, 0.5, 0.8):
        m = SpectreHatTime(corner_deg=120.0, k0=k0)
        assert all(is_simple_tile(m.tile(t, -1)) for t in taus), k0
        m = SpectreHatTime(corner_deg=150.0, k0=k0)
        assert all(is_simple_tile(m.tile(t, 1)) for t in taus), k0
        bad = [t for t in taus if not is_simple_tile(m.tile(t, -1))]
        assert bad and bad[0] == 0.0 and max(bad) <= 0.1 + 1e-9, (k0, bad)


def run_tests():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        print("  %-75s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    m = SpectreHatTime(corner_deg=120.0)
    print("omega_0 = 120 deg. Corner turns (deg) at P_7, P_8, P_9 for s = +1 / s = -1, and the difference:")
    for tau in (0.0, 0.5, 0.9):
        _, cp = turning(m.tile(tau, 1))
        _, cn = turning(m.tile(tau, -1))
        f = lambda c: " ".join("%7.1f" % math.degrees(c[i]) for i in (P8 - 1, P8, P8 + 1))
        print(" tau %.1f  s+: %s   s-: %s   diff: %s   area change %.4f" % (tau, f(cp), f(cn), f(cn - cp), m.jump(tau)))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ok = run_tests()
    if not ok or "--tests-only" in argv:
        return 0 if ok else 1
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
