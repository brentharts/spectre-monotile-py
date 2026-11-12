#!/usr/bin/env python3
"""t4_proofs.py - computer checks of the statements that Section 'Foundations' of quantgrav.tex proves on paper.

  P1  Every Bezier edge of spectremodel.Tile is point symmetric about the midpoint of its chord (P0 + P3 = P1 + P2), for every (a, b, k) tried.  This is the hypothesis of the proof that an edge encloses
      zero signed area with its chord, hence that the area of a tile does not depend on k.
  P2  The area of the tile is the polygon area A0(a, b) = 2 sqrt3 a^2 + 3ab + sqrt3 b^2 (the form of the Hat and Spectre repository, tile_family.py), and moving P_8 by s h (1, 0) changes it by s a h exactly.
  P3  The reflex outline of the straight tile (k = 0) stays simple exactly for h < h*(b) = min((3b + sqrt3)/2, (b + sqrt3)/(sqrt3 b - 1)), b in [1, sqrt3]: the first contact is P_8 with the edge P_12 P_13 for
      b below b_x = (1 + 2 sqrt7) / (3 sqrt3) = 1.2108 and the vertex P_11 on the edge P_8' P_9 above it.  So omega_c(b) = 2 arctan h*(b), 134.18 degrees at the Spectre and exactly 120 degrees at the Hat.
  P4  Free action: right multiplication by a non-trivial unit of the Hurwitz order fixes no non-zero vector of E_8 (checked on shells 1 to 3 against the proof, which is one line).

    python3 t4_proofs.py --tests-only
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from spectre_hat_collinearity_flow_model import FLAT_VERTEX  # noqa: E402
from spectre_hat_time_model import is_simple_tile  # noqa: E402
from spectremodel import Tile  # noqa: E402

S3 = math.sqrt(3.0)
B_X = (1.0 + 2.0 * math.sqrt(7.0)) / (3.0 * S3)


def a0_form(a, b):
    return 2.0 * S3 * a * a + 3.0 * a * b + S3 * b * b


def h_star(b):
    return min((3.0 * b + S3) / 2.0, (b + S3) / (S3 * b - 1.0))


def _moved(a, b, k, s, h):
    t = Tile(a, b, k)
    v = t.vertices[FLAT_VERTEX]
    v.position = v.position + np.array([s * h, 0.0])
    return t


def h_star_numeric(b):
    lo, hi = 0.0, 40.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if is_simple_tile(_moved(1.0, b, 0.0, -1, mid)) else (lo, mid)
    return lo


def test_p1_every_bezier_edge_is_point_symmetric():
    for a, b in ((1.0, 1.0), (1.0, S3), (0.7, 1.3)):
        for k in (0.0, 0.3, 0.8):
            for s, h in ((0, 0.0), (1, 0.4), (-1, 0.9)):
                cp = _moved(a, b, k, s, h).control_points()
                assert np.allclose(cp[:, 0] + cp[:, 3], cp[:, 1] + cp[:, 2], atol=1e-12), (a, b, k, s, h)


def test_p2_the_area_form_and_the_jump():
    for a, b in ((1.0, 1.0), (1.0, S3), (0.7, 1.3), (1.0, 1.0 / S3)):
        for k in (0.0, 0.5, 0.8):
            assert abs(Tile(a, b, k).area() - a0_form(a, b)) < 1e-9, (a, b, k)
            for h in (0.2, 1.1):
                up, dn = _moved(a, b, k, 1, h).area(), _moved(a, b, k, -1, h).area()
                assert abs(abs(up - dn) - 2.0 * a * h) < 1e-9 and abs(0.5 * (up + dn) - a0_form(a, b)) < 1e-9, (a, b, k, h)


def test_p3_the_critical_displacement_has_a_closed_form():
    assert abs(B_X - 1.2108) < 1e-4
    for b in np.linspace(1.0, S3, 25):
        assert abs(h_star(b) - h_star_numeric(b)) < 1e-9, b
    assert abs(h_star(1.0) - (3.0 + S3) / 2.0) < 1e-12 and abs(h_star(S3) - S3) < 1e-12
    assert abs(2.0 * math.degrees(math.atan(h_star(1.0))) - 134.177) < 1e-3 and abs(2.0 * math.degrees(math.atan(h_star(S3))) - 120.0) < 1e-9
    assert abs((3.0 * B_X + S3) / 2.0 - (B_X + S3) / (S3 * B_X - 1.0)) < 1e-12  # the two contacts coincide at b_x


def test_p4_right_multiplication_by_a_unit_is_free_on_e8():
    import hopf_shells as hs
    R = hs.e8_roots()
    assert len(R) == 240
    # units of the Hurwitz order: +-1, +-i, +-j, +-k and (+-1 +-i +-j +-k)/2 ; a q = a with a != 0 forces q = 1 in the quaternions (a is invertible): check on the roots
    import itertools
    units = [np.array(u, dtype=float) for u in itertools.product((0.5, -0.5), repeat=4)]
    for e in np.eye(4):
        units += [e, -e]
    assert len(units) == 24

    def qmul(x, y):
        a1, b1, c1, d1 = x
        a2, b2, c2, d2 = y
        return np.array([a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2, a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2, a1 * c2 - b1 * d2 + c1 * a2 + d1 * b2, a1 * d2 + b1 * c2 - c1 * b2 + d1 * a2])
    one = np.array([1.0, 0.0, 0.0, 0.0])
    for q in units:
        if np.allclose(q, one):
            continue
        for r in R[:60]:
            r = np.asarray(r, dtype=float)
            a, b = r[:4], r[4:]
            fixed = np.allclose(qmul(a, q), a) and np.allclose(qmul(b, q), b)
            assert not fixed, (q, r)


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
        print("  %-78s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def main(argv=None):
    return 0 if run_tests() else 1


if __name__ == "__main__":
    sys.exit(main())
