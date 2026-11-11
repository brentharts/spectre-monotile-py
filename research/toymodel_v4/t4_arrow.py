#!/usr/bin/env python3
"""t4_arrow.py - Stage 1 of research/toymodel_v4/PLAN.md: is there an arrow of the flow that does not depend on how tau is written? (open problem 6 of quantgrav.tex)

The Spectre -> Hat flow (spectre_hat_collinearity_flow_model.py) moves three parameters of the tile together, linearly in tau: the length b from a to a sqrt3, the curve strength k from k_0
to 0, and the corner angle omega at P_8 from omega_0 to 0.  Reparameterising tau (tau -> g(tau)) cannot change which functional is monotone, so the useful test is stronger: give each of the
three parameters its OWN monotone law (b(tau), k(tau), omega(tau) each running from its Spectre value to its Hat value, with any shape).  Call such a path admissible.  A quantity
is a *robust* arrow if it is monotone along every admissible path.  It is *law-dependent* if it is monotone along the linear flow but not along every admissible path.

Definitions
  D1.1  Tile(b, k, omega, s): the Bezier tile of spectremodel.Tile(a = 1, b, k) with P_8 pushed along the chord normal by s h, h = a tan(omega / 2) (the tile of SpectreHatTime for any b, k, omega).
  D1.2  Functionals of the outline alone: A (area), L (length of the outline), B (bending energy: integral of curvature^2 ds over the 14 Bezier edges, corners not included).
  D1.3  Admissible path: b = a (1 + (sqrt3 - 1) u_b(tau)), k = k_0 (1 - u_k(tau)), omega = omega_0 (1 - u_omega(tau)), with u_b, u_k, u_omega increasing from 0 to 1.  Here u = tau^p, p > 0, one exponent
        per parameter; p = 1 for all three is the flow of the module.

Exact claims (each a test; ``python3 t4_arrow.py --tests-only``)
  E1.1  The area splits exactly.  A(b, k, omega, s) = A0(b) + s a tan(omega / 2), with A0(b) = 2 sqrt3 a^2 + 3 a b + sqrt3 b^2 (the area of the Spectre family, from the flow module) independent
        of k and omega; so the two bit values differ by d = 2 a tan(omega / 2) (E0.1) and A0 is the bit-averaged area.  A0 is strictly increasing in b.
  E1.2  Robust arrows.  A0 and the area of the reflex tile A_- = A0 - a tan(omega / 2) increase along every one of 300 random admissible paths (exponents p in [0.2, 6]), because
        dA0/db > 0, dA_-/db > 0, dA_-/domega < 0 and A is independent of k.  So does d = 2 a tan(omega / 2) decrease (it is a function of omega only).
  E1.3  Law-dependent arrows.  Along the linear flow (omega_0 = 120 degrees, k_0 = 0.5, s = +1) A_+ rises, B falls monotonically and L is not monotone.  But dA_+/domega > 0 and dB/domega < 0 (opposite to
        the flow), so a path with (p_b, p_k, p_omega) = (6, 1, 1) makes A_+ fall somewhere and one with (6, 6, 1) makes B rise: monotonicity of A_+ and B belongs to the linear laws, not to the end points.
  E1.4  Validity is one-way only outside a band.  On the grid b in 5 values from 1 to sqrt3, k in 5 values from 0.8 to 0, omega in 11 values from 170 to 0 degrees: the s = +1 outline is a simple closed
        curve at all 275 points; the s = -1 outline is simple at every point with omega <= 119 and at none with omega >= 153; at omega = 136 it depends on (b, k), and of the 690 steps toward the Hat
        on the grid, 5 go from valid to invalid, all at omega = 136 (the band is where the threshold omega_c(b, k) lies).  So an admissible path may leave the valid set and re-enter it, but only
        while omega is in that band.
  E1.5  The two bit values give the same tile iff omega = 0, for every b and k tested: the end at which the bit cannot be read is fixed by omega alone.

Status.  The robust quantities are A0 (a function of b), A_- and d (functions of omega): each is a parameter of the flow in disguise, and each increases (A0, A_-) or decreases (d) toward the Hat.
Nothing else tested has this property.  So the arrow is not *derived* by an intrinsic functional that the laws do not already contain: it is the orientation 'from the end with a readable
bit and a small area to the end where the bit merges and the area is largest', and the three robust quantities agree on it.  That is a consistency, not an explanation, and D6 stays an input.
Valid bit values are not monotone either: at large omega_0 the s = -1 value is excluded early (E1.4), so the number of allowed, distinguishable bit values goes 1 -> 2 -> 1 along the flow.

    python3 t4_arrow.py --tests-only
    python3 t4_arrow.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
from spectre_hat_collinearity_flow_model import FLAT_VERTEX, SQRT3  # noqa: E402
from spectre_hat_time_model import is_simple_tile  # noqa: E402
from spectremodel import Tile  # noqa: E402

A_EDGE = 1.0


def tile_p(b, k, omega_deg, s):
    """D1.1."""
    t = Tile(A_EDGE, b, k)
    h = A_EDGE * math.tan(math.radians(omega_deg) / 2.0)
    v = t.vertices[FLAT_VERTEX]
    v.position = v.position + np.array([s * h, 0.0])
    return t


def A0(b):
    return 2.0 * SQRT3 * A_EDGE ** 2 + 3.0 * A_EDGE * b + SQRT3 * b * b


def functionals(t, n=400):
    """D1.2: dict A, L, B of a tile."""
    cp = np.asarray(t.control_points())
    u = np.linspace(0.0, 1.0, n)
    L = bend = 0.0
    for e in range(cp.shape[0]):
        P = cp[e]
        d1 = 3 * ((1 - u) ** 2)[:, None] * (P[1] - P[0]) + 6 * (u * (1 - u))[:, None] * (P[2] - P[1]) + 3 * (u ** 2)[:, None] * (P[3] - P[2])
        d2 = 6 * (1 - u)[:, None] * (P[2] - 2 * P[1] + P[0]) + 6 * u[:, None] * (P[3] - 2 * P[2] + P[1])
        sp = np.hypot(d1[:, 0], d1[:, 1])
        kap = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(sp, 1e-12) ** 3
        L += np.trapezoid(sp, u)
        bend += np.trapezoid(kap ** 2 * sp, u)
    return {"A": t.area(), "L": L, "B": bend}


def path_params(tau, p_b, p_k, p_w, k0=0.5, om0=120.0):
    """D1.3: (b, k, omega_deg) at tau for an admissible path."""
    return (A_EDGE * (1.0 + (SQRT3 - 1.0) * tau ** p_b), k0 * (1.0 - tau ** p_k), om0 * (1.0 - tau ** p_w))


def along(p_b, p_k, p_w, s, key, n=25, k0=0.5, om0=120.0):
    taus = np.linspace(0.0, 1.0, n)
    return np.array([functionals(tile_p(*path_params(t, p_b, p_k, p_w, k0, om0), s))[key] for t in taus])


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e11_the_area_splits_into_a_function_of_b_and_a_function_of_omega():
    bs = np.linspace(1.0, SQRT3, 4)
    for b, k, om in itertools.product(bs, (0.0, 0.4, 0.8), (0.0, 30.0, 90.0, 150.0)):
        for s in (1, -1):
            want = A0(b) + s * A_EDGE * math.tan(math.radians(om) / 2.0)
            assert abs(tile_p(b, k, om, s).area() - want) < 1e-9, (b, k, om, s)
    vals = [A0(b) for b in np.linspace(1.0, SQRT3, 50)]
    assert all(y > x for x, y in zip(vals, vals[1:]))
    assert abs(A0(1.0) - (2 * SQRT3 + 3 + SQRT3)) < 1e-12 and abs(A0(SQRT3) - 8 * SQRT3) < 1e-12  # the Spectre and the Hat


def test_e12_robust_arrows_are_monotone_on_every_admissible_path():
    rng = np.random.default_rng(1)
    taus = np.linspace(0.0, 1.0, 40)
    for _ in range(300):
        p_b, p_k, p_w = np.exp(rng.uniform(math.log(0.2), math.log(6.0), 3))
        om0, k0 = rng.uniform(20.0, 160.0), rng.uniform(0.1, 0.9)
        b, k, om = np.array([path_params(t, p_b, p_k, p_w, k0, om0) for t in taus]).T
        a0 = A0(b)
        am = a0 - A_EDGE * np.tan(np.radians(om) / 2.0)
        d = 2.0 * A_EDGE * np.tan(np.radians(om) / 2.0)
        assert np.all(np.diff(a0) > 0) and np.all(np.diff(am) > 0) and np.all(np.diff(d) < 0), (p_b, p_k, p_w, om0, k0)
    # the same on real tiles for a handful of paths (the closed form of E1.1 is what the 300 paths use)
    for pp in ((1, 1, 1), (4, 0.5, 2), (0.3, 5, 0.7)):
        for s_sign, key in ((-1, "A"),):
            a = along(*pp, s_sign, key)
            assert np.all(np.diff(a) > 0), pp


def test_e13_area_of_the_convex_tile_and_bending_energy_belong_to_the_linear_laws():
    ap, bp, lp = (along(1, 1, 1, 1, key) for key in ("A", "B", "L"))
    assert np.all(np.diff(ap) > 0) and np.all(np.diff(bp) < 0), "monotone along the linear flow"
    assert not (np.all(np.diff(lp) > 0) or np.all(np.diff(lp) < 0)), "perimeter is not monotone along the flow"
    a_b_late = along(6, 1, 1, 1, "A", n=60)  # exponents (p_b, p_k, p_omega) = (6, 1, 1): b late, omega early
    assert np.any(np.diff(a_b_late) < 0), "A_+ falls where b moves late and omega moves early"
    b_path = along(6, 6, 1, 1, "B", n=60)  # (6, 6, 1): b and k late, omega early
    assert np.any(np.diff(b_path) > 0), "B rises where omega moves first"


def _validity_grid(s):
    bs = np.linspace(1.0, SQRT3, 5)
    ks = np.linspace(0.8, 0.0, 5)
    oms = np.linspace(170.0, 0.0, 11)
    V = np.zeros((5, 5, 11), dtype=bool)
    for i, j, l in itertools.product(range(5), range(5), range(11)):
        V[i, j, l] = is_simple_tile(tile_p(bs[i], ks[j], oms[l], s))
    return V, oms


def test_e14_validity_is_one_way_only_outside_a_band():
    Vp, oms = _validity_grid(+1)
    assert Vp.all() and Vp.size == 275
    Vm, oms = _validity_grid(-1)
    assert Vm[:, :, oms <= 119.5].all(), "s = -1 valid for omega <= 119"
    assert not Vm[:, :, oms >= 152.5].any(), "s = -1 invalid for omega >= 153"
    band = (oms > 119.5) & (oms < 152.5)
    assert band.sum() == 1 and Vm[:, :, band].any() and not Vm[:, :, band].all(), "omega = 136 depends on (b, k)"
    viol, total, where = 0, 0, set()
    for i, j, l in itertools.product(range(5), range(5), range(11)):
        for di, dj, dl in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):  # b up, k down, omega down: toward the Hat
            i2, j2, l2 = i + di, j + dj, l + dl
            if i2 < 5 and j2 < 5 and l2 < 11:
                total += 1
                if Vm[i, j, l] and not Vm[i2, j2, l2]:
                    viol += 1
                    where.add(float(oms[l]))
    assert (viol, total) == (5, 690), (viol, total)
    assert where == {136.0}, where


def test_e15_the_bit_merges_iff_omega_is_zero():
    for b, k in itertools.product((1.0, 1.4, SQRT3), (0.0, 0.3, 0.8)):
        same = np.allclose(tile_p(b, k, 0.0, 1).control_points(), tile_p(b, k, 0.0, -1).control_points(), atol=1e-12)
        assert same, (b, k)
        for om in (1.0, 10.0, 60.0):
            diff = np.abs(tile_p(b, k, om, 1).control_points() - tile_p(b, k, om, -1).control_points()).max()
            assert diff > 1e-3, (b, k, om, diff)


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
        print("  %-86s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    print("Linear flow (omega_0 = 120, k_0 = 0.5), s = +1: A, L, B at tau = 0, .25, .5, .75, 1")
    for key in ("A", "L", "B"):
        print("  %s:" % key, np.round(along(1, 1, 1, 1, key, n=5), 3))
    print("A_- (robust) along the linear flow:", np.round(along(1, 1, 1, -1, "A", n=5), 3))
    print("Path with b and k late (p = 6, 6, 1): A_+ and B at five tau")
    print("  A_+:", np.round(along(6, 6, 1, 1, "A", n=5), 3))
    print("  B  :", np.round(along(6, 6, 1, 1, "B", n=5), 3))


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
