#!/usr/bin/env python3
"""t4_open.py - the two open questions raised by Section 2 of quantgrav.tex (the flow of the 14th vertex), tested.

  Q1  Does the fall of the bit count toward the Hat survive other choices of omega_0 and k_0, and what does 'entropy' mean here?
  Q2  Is there any deformation of the outline that reaches a relative sign of -1 between the two bit values?

Definitions
  D5.1  Counting entropy.  A flat pair of the level-3 patch (N = 82 pairs, t4_patch.py) has two distinct open states iff 0 < omega and the reflex outline is a simple closed curve.
        S_count(tau) = N * 1[that holds at (b(tau), k(tau), omega(tau))], in bits (log2 of 2 states per pair).
  D5.2  Resolved entropy.  The two states count as distinct only if they differ by more than the resolution eps of the model (the snap scale, 0.5):
        S_res(tau) = N * log2(1 + d(tau) / eps) * 1[reflex outline simple].  It falls continuously to 0 at the Hat; D5.1 falls by a step at tau = 1 and not before.
  D5.3  Relative sign.  With Delta the difference between the corner turns at P_8 of the s = -1 and the s = +1 outline (each wrapped to (-pi, pi]), the relative spinor sign of the two
        values is exp(i Delta / 2) (for the chord-normal displacement Delta = 2 omega, as in t4_spin.py E3.7).  It is -1 iff Delta = 2 pi (mod 4 pi).
        A deformation is: P_8 displaced by s h (cos phi, sin phi) in the tile frame (phi = 0 is the chord normal of the flow), any h > 0 and phi, any b in [1, sqrt 3], k in [0, 0.8].

Claims (each a test; ``python3 t4_open.py --tests-only``)
  E5.1  Both entropies are monotone non-increasing along the linear flow iff the reflex outline is simple at tau = 0, i.e. omega_0 < omega_c(k_0) of t4_spin.py.  Above it S rises from 0 and then falls:
        no monotone arrow at all.  So the fall survives every omega_0 and k_0 of the grid below omega_c and fails above it, for both definitions.
  E5.2  S_count at tau -> 1^- equals its maximum: the fall to 0 is a step at the Hat itself (omega = 0 is the only point of zero), the counting entropy is not decreasing 'toward' the Hat but at it.
  E5.3  On 40 random admissible paths per omega_0 (one exponent per parameter, k_0 random) the resolved entropy is monotone on all of them for omega_0 = 30 and 100 degrees and on none for 136 and 165.
        (Stage 1 found the validity band near 136 degrees depends on the path; in this sample it did not show.)
  E5.4  Relative sign: for every deformation of the scan with BOTH outlines simple, |Delta| < 2 pi.  The supremum found is 313 degrees on the coarse scan of the test and 315 on a finer one (b = sqrt 3, k = 0.3 or 0, phi = -40 degrees, h = 3.9), more than the
        chord-normal 2 omega_c = 264 degrees (k_0 = 0.5), so tilting the displacement helps, and still about 45 degrees short of 360: the sign -1 is not reached by displacing P_8 in any direction.
        The corner turn is a wrapped angle, so Delta = 2 pi is the limit of a vertex at infinity.  Not tested: moving other vertices, or the S-curves' strength per edge.

    python3 t4_open.py --tests-only
    python3 t4_open.py --story
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from spectre_hat_collinearity_flow_model import FLAT_VERTEX, SQRT3  # noqa: E402
from spectre_hat_time_model import is_simple_tile  # noqa: E402
from spectremodel import Tile  # noqa: E402
import t4_patch as tp  # noqa: E402
import t4_spin as ts  # noqa: E402

N_PAIRS = 82
EPS = 0.5
OM0S = list(range(10, 171, 10))
K0S = (0.1, 0.3, 0.5, 0.7, 0.9)
TAUS = np.linspace(0.0, 1.0, 101)


def entropies(b_of, k_of, w_of, taus=TAUS):
    """(S_count, S_res) along a path; b_of, k_of, w_of map tau to b, k, omega (degrees)."""
    sc, sr = [], []
    for t in taus:
        om = w_of(t)
        ok = om > 1e-12 and tp.reflex_valid(b_of(t), k_of(t), om)
        h = math.tan(math.radians(om) / 2.0)
        sc.append(N_PAIRS * 1.0 if ok else 0.0)
        sr.append(N_PAIRS * math.log2(1.0 + 2.0 * h / EPS) if ok else 0.0)
    return np.array(sc), np.array(sr)


def linear_path(om0, k0):
    return (lambda t: 1.0 + (SQRT3 - 1.0) * t), (lambda t: k0 * (1.0 - t)), (lambda t: om0 * (1.0 - t))


def non_increasing(x):
    return bool(np.all(np.diff(x) <= 1e-9))


def grid_classes():
    """{(om0, k0): (monotone S_count, monotone S_res, valid at tau = 0)}"""
    out = {}
    for om0 in OM0S:
        for k0 in K0S:
            sc, sr = entropies(*linear_path(om0, k0))
            out[(om0, k0)] = (non_increasing(sc), non_increasing(sr), sc[0] > 0)
    return out


def corner_difference(b, k, h, phi_deg):
    """D5.3: (Delta, both outlines simple) for the displacement s h (cos phi, sin phi) of P_8, s = +-1."""
    u = np.array([math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))])
    turns, simple = [], []
    for s in (1, -1):
        t = Tile(1.0, b, k)
        v = t.vertices[FLAT_VERTEX]
        v.position = v.position + s * h * u
        simple.append(bool(is_simple_tile(t)))
        _, corner = ts_turning(t)
        turns.append(corner[FLAT_VERTEX - 1])
    d = turns[1] - turns[0]
    return float(d), all(simple)


def ts_turning(tile):
    import t4_geometry as tg
    return tg.turning(tile)


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e51_the_fall_survives_exactly_below_omega_c():
    cls = grid_classes()
    oc = {k0: ts.omega_c(k0)[0] for k0 in K0S}
    for (om0, k0), (mc, mr, v0) in cls.items():
        below = om0 < oc[k0]
        assert v0 == below, (om0, k0)
        assert mc == below and mr == below, (om0, k0, mc, mr, below)
    assert sum(1 for v in cls.values() if v[0]) == sum(1 for (om0, k0) in cls if om0 < oc[k0])


def test_e52_the_counting_entropy_falls_by_a_step_at_the_hat():
    sc, sr = entropies(*linear_path(30.0, 0.5))
    assert sc[-1] == 0.0 and np.all(sc[:-1] == N_PAIRS), sc
    assert sr[-1] == 0.0 and np.all(np.diff(sr) < 0) and sr[-2] > 0 and sr[-2] < 0.2 * sr[0]


def test_e53_random_paths():
    rng = np.random.default_rng(3)
    frac = {}
    for om0 in (30.0, 100.0, 136.0, 165.0):
        mono = 0
        for _ in range(40):
            pb, pk, pw = rng.uniform(0.3, 6.0, 3)
            k0 = rng.uniform(0.1, 0.9)
            _, sr = entropies(lambda t: 1.0 + (SQRT3 - 1.0) * t ** pb, lambda t: k0 * (1.0 - t ** pk), lambda t: om0 * (1.0 - t ** pw), np.linspace(0.0, 1.0, 41))
            mono += non_increasing(sr)
        frac[om0] = mono / 40.0
    assert frac == {30.0: 1.0, 100.0: 1.0, 136.0: 0.0, 165.0: 0.0}, frac


def sup_delta(b, k, phis, hs):
    best = 0.0
    for phi in phis:
        for h in hs:
            d, ok = corner_difference(b, k, h, phi)
            if ok:
                assert abs(d) < 2.0 * math.pi - 0.5, (b, k, phi, h, d)
                best = max(best, abs(d))
    return best


def test_e54_no_valid_deformation_reaches_relative_sign_minus_one():
    hs = np.linspace(0.3, 6.0, 20)
    phis = (-60.0, -45.0, -30.0, 0.0, 30.0, 60.0)
    best = max(sup_delta(b, k, phis, hs) for b in (1.0, SQRT3) for k in (0.0, 0.3))
    assert math.radians(290.0) < best < math.radians(320.0), math.degrees(best)
    d0, ok0 = corner_difference(1.0, 0.5, math.tan(math.radians(60.0)), 0.0)
    assert ok0 and abs(abs(d0) - math.radians(240.0)) < 1e-6, d0  # chord normal: Delta = 2 omega


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
        print("  %-82s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    cls = grid_classes()
    print("S monotone (count, resolved, valid at tau=0) on the (omega_0, k_0) grid:")
    for om0 in OM0S:
        print("  %3d  " % om0 + " ".join("%d%d%d" % tuple(int(x) for x in cls[(om0, k0)]) for k0 in K0S))


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
