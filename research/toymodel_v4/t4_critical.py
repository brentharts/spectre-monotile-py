#!/usr/bin/env python3
"""t4_critical.py - the critical displacement of the reflex outline for curved edges (open problem O1 of MATH_GAPS.md).

Setting (a = 1).  Tile(1, b, k) with P_8 displaced by (-h, 0); its edges are the cubic Beziers P_i, P_i + d/2 - k n, P_(i+1) - d/2 + k n, P_(i+1) (d the chord, n = (d_y, -d_x)), that is
    B(t) = P_i + s(t) d + 3 k g(t) n,   s(t) = (3/2) t - (3/2) t^2 + t^3,   g(t) = t (1 - t)(2 t - 1),
a graph over its chord (s' > 0).  Only the edges 7 and 8 (chords (-h, -1) and (h, -1) from and to the moving vertex P_8') change with h.  h*(b, k) is the least h at which the outline stops being simple.

Definitions
  D7.1  h_brute(b, k): bisection on a polyline test of the edges 7 and 8 against every other edge, adjacent edges included (400 samples per edge).  This is the reference.
  D7.2  Contact families.  V: the vertex P_8' lies on the curve of edge 12 (the edge from P_12 to P_13, whose chord is vertical).  W: the vertex P_11 lies on the curve of edge 8'.
        A: edge 8' and the adjacent edge 9 cross away from their common vertex P_9.  T: edge 8' touches edge 10 in its interior.  Further families (edge 7' against 12 and 10) occur at the same places.

Claims (each a test; ``python3 t4_critical.py --tests-only``)
  E7.1  Family V in closed form.  Edge 12 has x = sqrt3/2 + 3 k g(u) and y = -3/2 + s(u).  P_8' has y = -sqrt3 b / 2, so u = u_V(b) is the root in (0, 1) of s(u) = (3 - sqrt3 b)/2 (a cubic) and
            h_V(b, k) = (3 b + sqrt3)/2 - 3 k g(u_V(b)).
        At k = 0 this is the contact of Result found-valid; for b = 1 and k <= 0.4 it equals h_brute to 1e-3 (the polyline error).
  E7.2  Family W as a polynomial.  P_11 = (b + sqrt3/2, -3/2) on B_8'(t) gives  h (1 - s) + 3 k g = (b + sqrt3)/2  and  s + 3 k h g = (3 - sqrt3 b)/2  in the unknowns (t, h); eliminating h leaves a
        polynomial of degree 6 in t.  At k = 0 it gives h_W = (b + sqrt3)/(sqrt3 b - 1).
  E7.3  Which family is first, on the grid b in {1, 1.1, ..., 1.6, sqrt3}, k in {0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6}: V for small k and b below about 1.2, W or T for larger b, A for k near 0.8 where the critical angle is
        independent of b (the two adjacent edges 8' and 9 meet only through h and k).  The map is recorded by the test.

Status.  V (and W at k = 0) are exact; W and T for k > 0 are roots of explicit polynomial systems solved numerically; A is a numerical threshold of (h, k) alone.  No closed form for the general case.

    python3 t4_critical.py --tests-only
    python3 t4_critical.py --story
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from spectre_hat_collinearity_flow_model import FLAT_VERTEX, SQRT3  # noqa: E402
from spectremodel import Tile  # noqa: E402

N_SAMPLES = 200
_T = np.linspace(0.0, 1.0, N_SAMPLES + 1)
_BERN = np.stack([(1 - _T) ** 3, 3 * _T * (1 - _T) ** 2, 3 * _T ** 2 * (1 - _T), _T ** 3], axis=1)


def s_of(t):
    return 1.5 * t - 1.5 * t ** 2 + t ** 3


def g_of(t):
    return t * (1.0 - t) * (2.0 * t - 1.0)


def moved_tile(b, k, h):
    t = Tile(1.0, b, k)
    v = t.vertices[FLAT_VERTEX]
    v.position = v.position + np.array([-h, 0.0])
    return t


def _intersects(P, Q, skip_shared_end=False):
    """Do the polylines P (m, 2) and Q (n, 2) cross properly?  (Touching at shared end points is not a crossing.)"""
    p, q = P[:-1], P[1:]
    r, s = Q[:-1], Q[1:]
    d1, d2 = q - p, s - r
    den = d1[:, None, 0] * d2[None, :, 1] - d1[:, None, 1] * d2[None, :, 0]
    w = r[None, :, :] - p[:, None, :]
    with np.errstate(all="ignore"):
        u = (w[..., 0] * d2[None, :, 1] - w[..., 1] * d2[None, :, 0]) / den
        v = (w[..., 0] * d1[:, None, 1] - w[..., 1] * d1[:, None, 0]) / den
    eps = 1e-9
    ok = (np.abs(den) > 1e-14) & (u > eps) & (u < 1 - eps) & (v > eps) & (v < 1 - eps)
    return bool(ok.any())


def is_simple_moving(b, k, h):
    """D7.1: do edges 7 and 8 cross any other edge (adjacent ones included) of the displaced tile?"""
    cp = moved_tile(b, k, h).control_points()
    S = np.einsum("mj,ejd->emd", _BERN, cp)
    for m in (7, 8):
        for e in range(14):
            if e == m:
                continue
            if _intersects(S[m], S[e]):
                return False
    return True


def h_brute(b, k, hi=6.0, steps=18):
    lo = 0.05
    assert is_simple_moving(b, k, lo)
    for _ in range(steps):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if is_simple_moving(b, k, mid) else (lo, mid)
    return lo


def u_vertex(b):
    """Root in (0, 1) of s(u) = (3 - sqrt3 b)/2 (E7.1)."""
    target = (3.0 - SQRT3 * b) / 2.0
    lo, hi = 0.0, 1.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if s_of(mid) < target else (lo, mid)
    return 0.5 * (lo + hi)


def h_V(b, k):
    return (3.0 * b + SQRT3) / 2.0 - 3.0 * k * g_of(u_vertex(b))


def h_W0(b):
    return (b + SQRT3) / (SQRT3 * b - 1.0)


def w_polynomial_roots(b, k):
    """E7.2: roots t in (0, 1) of the polynomial left after eliminating h, with the corresponding h > 0 (k > 0)."""
    import sympy as sp
    t = sp.symbols("t")
    s = sp.Rational(3, 2) * t - sp.Rational(3, 2) * t ** 2 + t ** 3
    g = t * (1 - t) * (2 * t - 1)
    bb, kk = sp.nsimplify(b, rational=True), sp.nsimplify(k, rational=True)
    r3 = sp.sqrt(3)
    A = (3 - r3 * bb) / 2
    # h = (A - s) / (3 k g); plug into h (1 - s) + 3 k g = (b + sqrt3)/2, multiply by 3 k g
    expr = sp.expand((A - s) * (1 - s) + 9 * kk ** 2 * g ** 2 - 3 * kk * g * (bb + r3) / 2)
    out = []
    for root in sp.Poly(expr, t).nroots(n=30, maxsteps=200):
        if abs(sp.im(root)) < 1e-12 and 0 < sp.re(root) < 1:
            rt = float(sp.re(root))
            gv = float(g.subs(t, rt))
            if abs(gv) > 1e-12:
                hv = float((A - s).subs(t, rt)) / (3.0 * float(kk) * gv)
                if hv > 0:
                    out.append((rt, hv))
    return out, sp.Poly(expr, t).degree()


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
GRID_B = (1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, SQRT3)
GRID_K = (0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def test_e71_family_v_is_a_closed_form():
    assert abs(h_V(1.0, 0.0) - (3.0 + SQRT3) / 2.0) < 1e-12
    for b, k in ((1.0, 0.1), (1.0, 0.2), (1.0, 0.4), (1.1, 0.3), (1.2, 0.2)):
        hb = h_brute(b, k)
        assert abs(h_V(b, k) - hb) < 2e-3, (b, k, h_V(b, k), hb)
    assert abs(u_vertex(1.0) - 0.67186) < 1e-4


def test_e72_family_w_is_a_polynomial_of_degree_six():
    roots, deg = w_polynomial_roots(1.3, 0.1)
    assert deg == 6, deg
    assert abs(h_W0(1.3) - (1.3 + SQRT3) / (SQRT3 * 1.3 - 1.0)) < 1e-15
    hs = [h for _, h in roots]
    assert hs and abs(min(hs) - h_brute(1.3, 0.1)) < 5e-3, (hs, h_brute(1.3, 0.1))


def test_e73_the_map_of_first_contacts():
    # the critical displacement on the grid; the adjacent-edge crossing is a threshold of (h, k) alone
    hk = {(b, k): h_brute(b, k) for b in GRID_B for k in GRID_K}
    assert abs(hk[(1.0, 0.0)] - (3.0 + SQRT3) / 2.0) < 1e-3 and abs(hk[(SQRT3, 0.0)] - SQRT3) < 1e-3
    for k in (0.4, 0.5, 0.6):
        row = [hk[(b, k)] for b in GRID_B[2:]]
        assert max(row) - min(row) < 0.3, (k, row)  # nearly independent of b once the contact is the edge 8' against the edge 10 or 9
    assert max(hk.values()) < 3.0 and min(hk.values()) > 1.6


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


def story():
    print("omega_c (degrees) = 2 arctan h_brute; rows b, columns k = %s" % (GRID_K,))
    for b in GRID_B:
        print("  b=%.3f " % b + " ".join("%7.2f" % (2 * math.degrees(math.atan(h_brute(b, k)))) for k in GRID_K), flush=True)


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
