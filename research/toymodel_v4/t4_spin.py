#!/usr/bin/env python3
"""t4_spin.py - Stage 3 of research/toymodel_v4/PLAN.md: does the tile outline supply the sign of the fusion (open problem 7 of quantgrav.tex)?

toymodel_v3 (t3_qubit.py): the two-wing state is the singlet exactly when the shared origin amplitude is odd under inversion, a_{v+3} = -a_v; a flux cannot make it odd (E4.5);
and reading scalar amplitudes with half-angle (spinor) kets needs a sign for each of the six letters, so the answer depends on a choice (E4.6).  The remedy named there: a spin structure.
The tile outline has one natural candidate: its tangent angle has a unique continuous lift to the reals, with total change -2 pi, so exp(i theta / 2) is a well defined function along
the outline and returns to minus itself after one circuit.  This module uses it and asks what it buys.

Definitions
  D3.1  Lift.  Theta_e = the chord direction of edge e of the outline of SpectreHatTime.tile(tau, s), continued from a base edge e0 by adding the signed corner turn at each corner
        (the principal value in (-pi, pi]).  The seam is the point where the lift is cut open: after a full circuit the lift has changed by -2 pi.
  D3.2  Spinor ket of edge e: kappa_e = (exp(-i Theta_e / 2), exp(+i Theta_e / 2)) / sqrt 2 (the SU(2)-covariant half-angle ket of t3_qubit.py E4.6, with the sign now fixed by the lift).
  D3.3  Letters.  The six hexagonal directions phi_v = 60 v degrees are the six edges of the outline whose direction is a multiple of 60 degrees among the edges not shaped by the bit (edges 0, 1, 4, 5, 10, 11; edges 7 and 8
        can also point along a hexagonal direction at particular tau and are left out); each direction occurs on exactly one of them (E3.1), so a letter is an edge: E(v).
  D3.4  State.  chi = sum_v a_v kappa_{E(v)} (x) kappa_{E(v+3)}  for an origin amplitude a_v (the two-wing state of t3_qubit.py with no wing, kets replaced by D3.2).
  D3.5  The two arrows at P_8 (spectre_hat_time_model.Arrows) give two spinors kappa_in, kappa_out with the measured angle between them, the turn -s omega.

Exact claims (each a test; ``python3 t4_spin.py --tests-only``)
  E3.1  The lift is antiperiodic: the total change of Theta around the outline is -2 pi for both bit values at every tau tested, so kappa returns to -kappa after a circuit and a seam
        cannot be avoided; and the six hexagonal directions sit on six different edges, one each.
  E3.2  Where the seam is put matters only through the signs of the six letters.  Over the 14 possible base edges there are 6 distinct sign patterns on the letters (up to an overall sign):
        the seam can sit in any of the 6 gaps between the six letters on the outline.
  E3.3  [Negative] The outline does not supply the sign.  For a uniform origin (a_v = 1 / sqrt 6) and for every even winding a_v = exp(i w phi_v) / sqrt 6, w = 0, 2, 4, the state has no
        singlet component for every base edge and every tau tested: the outline's spin structure does not make the origin odd.
  E3.4  [Negative for gauge independence] For an odd winding w the state is the singlet exactly or it vanishes, never anything else, and which one happens depends on the base edge:
        for w = +-1 it vanishes for 1 of the 14 base edges (the seam between edges 10 and 11, where the six signs make the six terms cancel) and is the singlet for the other 13; for
        w = +-3 it is the singlet for all 14.  So the sign pattern fixed by the lift is not a gauge: it changes the physics.
  E3.5  A seam-independent version, with the origin amplitude taken from the same lift (a_v = exp(i (Theta_{E(v)} + Theta_{E(v+3)}) / 2) / sqrt 6, whose sign changes cancel those of the kets),
        gives the same state up to a global phase for all 14 base edges, and that state is a product state (singlet fidelity 0): the covariant construction reduces to the periodic
        equatorial construction of t3_qubit.py with a uniform origin, so a spin structure alone adds nothing.
  E3.6  The two arrows at P_8: kappa_in and kappa_out have overlap cos(turn/2), the antisymmetrised pair kappa_in (x) kappa_out - kappa_out (x) kappa_in has squared norm 2 sin^2(omega/2)
        and is exactly the singlet for every tau < 1 and both bit values; at tau = 1 (aligned arrows) it vanishes.
  E3.7  The relative sign of the two bit values.  The half-angle phases of the turn at P_8 are exp(-+ i omega / 2) for s = +-1, so the relative phase is exp(i omega), equal to -1 only at
        omega = pi.  The s = -1 outline at tau = 0 is a simple closed curve only up to omega_c = 133.0, 132.1, 120.5 degrees for k_0 = 0.3, 0.5, 0.8 (bisection; every larger integer
        angle tested up to 179 fails), so within the valid outlines the relative phase stays at least 47, 48, 59 degrees away from -1.

Status.  The outline provides a mechanism (a canonical, antiperiodic lift) and not a value: it fixes a sign per letter only after a base edge is chosen (E3.2), and the sign pattern
that results either leaves the origin even (E3.3), or changes the outcome with the base edge (E3.4), or cancels against a covariant amplitude (E3.5).  The only place where the arrows
show an antisymmetry is the pair at P_8 (E3.6), with weight sin^2(omega/2) that vanishes at the Hat; and a relative sign -1 between the bit values would need omega = pi, which the geometry
excludes (E3.7).  The sign of the fusion therefore stays an input, as in toymodel_v3.  **Hypothesis**, not tested: that the pair of arrows at P_8, antisymmetrised, is the fusion (E3.6).

    python3 t4_spin.py --tests-only
    python3 t4_spin.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
_V3 = os.path.join(os.path.dirname(_HERE), "toymodel_v3")
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
for _p in (_ROOT, _V3, _V2):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t3_qubit as Q  # noqa: E402
from spectre_hat_time_model import SpectreHatTime, is_simple_tile  # noqa: E402

N_EDGES = 14
OPENED = (7, 8)  # the two edges that P_8 shapes (spectre_hat_time_model.OPENED_EDGES)
TWO_PI = 2.0 * math.pi


def chords(tile):
    """Principal chord angles of the 14 edges (radians), from vertex to next vertex."""
    P = np.array([v.position for v in tile.vertices])
    d = np.roll(P, -1, axis=0) - P
    return np.arctan2(d[:, 1], d[:, 0])


def _wrap(x):
    return (x + math.pi) % TWO_PI - math.pi


def lift(tile, e0=0):
    """D3.1: (Theta, closing) with Theta[e] the continued chord angle of edge e starting at base edge e0, and closing = the change of the
    lift after a full circuit (back at e0)."""
    ang = chords(tile)
    th = np.zeros(N_EDGES)
    th[e0] = ang[e0]
    for k in range(1, N_EDGES):
        i, j = (e0 + k - 1) % N_EDGES, (e0 + k) % N_EDGES
        th[j] = th[i] + _wrap(ang[j] - ang[i])
    closing = th[(e0 + N_EDGES - 1) % N_EDGES] + _wrap(ang[e0] - ang[(e0 + N_EDGES - 1) % N_EDGES]) - th[e0]
    return th, closing


def kappa(theta):
    """D3.2."""
    return np.array([np.exp(-0.5j * theta), np.exp(0.5j * theta)]) / math.sqrt(2.0)


def letter_edges(tile):
    """D3.3: dict v -> edge index for the edges whose chord direction is 60 v degrees; raises if a direction is missing or repeated."""
    ang = chords(tile)
    out = {}
    for e in range(N_EDGES):
        if e in OPENED:
            continue  # edges 7 and 8 are shaped by the bit and may point along a hexagonal direction at some tau
        k = (math.degrees(ang[e]) % 360.0) / 60.0
        if abs(k - round(k)) < 1e-6:
            v = int(round(k)) % 6
            if v in out:
                raise ValueError("direction %d on two edges" % v)
            out[v] = e
    if sorted(out) != list(range(6)):
        raise ValueError("not every hexagonal direction is on an edge: %r" % (sorted(out),))
    return out


def chi(tile, e0, a_v, amplitude_from_lift=False):
    """D3.4 (and the covariant variant of E3.5 when ``amplitude_from_lift``: a_v is multiplied by exp(i (Theta_E(v) + Theta_E(v+3)) / 2))."""
    th, _ = lift(tile, e0)
    E = letter_edges(tile)
    s = np.zeros(4, dtype=complex)
    for v in range(6):
        ea, eb = E[v], E[(v + 3) % 6]
        a = a_v[v] * (np.exp(0.5j * (th[ea] + th[eb])) if amplitude_from_lift else 1.0)
        s += a * np.kron(kappa(th[ea]), kappa(th[eb]))
    return s


def sign_pattern(tile, e0):
    """Parity of the number of 2 pi by which the lift of each letter's edge differs from its direction, normalised so letter 0 has 0 (an overall sign is no pattern)."""
    th, _ = lift(tile, e0)
    E = letter_edges(tile)
    n = [int(round((th[E[v]] - math.radians(60.0 * v)) / TWO_PI)) % 2 for v in range(6)]
    return tuple((x - n[0]) % 2 for x in n)


def concurrence(c):
    """|det| of the 2 x 2 coefficient matrix of a normalised two-qubit pure state times 2: 0 for a product state, 1 for a maximally entangled one."""
    c = c / np.linalg.norm(c)
    return float(2.0 * abs(c[0] * c[3] - c[1] * c[2]))


def odd_wind(w):
    return Q.winding(w)


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e31_the_lift_is_antiperiodic_and_letters_are_edges():
    for om in (30.0, 120.0):
        m = SpectreHatTime(corner_deg=om)
        for tau in (0.0, 0.5, 0.9):
            for s in (1, -1):
                tile = m.tile(tau, s)
                for e0 in (0, 5, 11):
                    th, closing = lift(tile, e0)
                    assert abs(closing + TWO_PI) < 1e-9, (om, tau, s, e0, closing)
                    assert np.allclose(kappa(th[e0] + closing), -kappa(th[e0]))
                E = letter_edges(tile)
                assert len(set(E.values())) == 6
    assert letter_edges(SpectreHatTime().tile(0.0, 1)) == {1: 0, 0: 1, 5: 4, 4: 5, 2: 10, 3: 11}


def test_e32_six_sign_patterns_one_per_gap():
    tile = SpectreHatTime(corner_deg=30.0).tile(0.0, 1)
    pats = [sign_pattern(tile, e0) for e0 in range(N_EDGES)]
    assert len(set(pats)) == 6, sorted(set(pats))


def test_e33_even_origins_never_give_the_singlet():
    m = SpectreHatTime(corner_deg=30.0)
    for tau in (0.0, 0.5, 0.9):
        tile = m.tile(tau, 1)
        for w in (0, 2, -2, 4):
            for e0 in range(N_EDGES):
                assert Q.fidelity(chi(tile, e0, odd_wind(w))) < 1e-9, (tau, w, e0)


def test_e34_odd_origins_give_the_singlet_or_nothing_depending_on_the_base_edge():
    tile = SpectreHatTime(corner_deg=30.0).tile(0.0, 1)
    zero_counts = {}
    for w in (1, -1, 3, -3, 5):
        zero = 0
        for e0 in range(N_EDGES):
            c = chi(tile, e0, odd_wind(w))
            if np.linalg.norm(c) < 1e-9:
                zero += 1
            else:
                assert abs(Q.fidelity(c) - 1.0) < 1e-9, (w, e0, Q.fidelity(c))
        zero_counts[w] = zero
    assert zero_counts[1] == 1 and zero_counts[-1] == 1, zero_counts
    assert zero_counts[3] == 0 and zero_counts[-3] == 0, zero_counts


def test_e35_the_covariant_version_is_seam_independent_and_a_product_state():
    tile = SpectreHatTime(corner_deg=30.0).tile(0.0, 1)
    uni = np.ones(6) / math.sqrt(6.0)
    ref = chi(tile, 0, uni, amplitude_from_lift=True)
    assert np.linalg.norm(ref) > 1e-6
    for e0 in range(N_EDGES):
        c = chi(tile, e0, uni, amplitude_from_lift=True)
        assert abs(abs(np.vdot(ref, c)) - np.linalg.norm(ref) * np.linalg.norm(c)) < 1e-9, e0  # parallel: same state up to a phase
        assert Q.fidelity(c) < 1e-9, e0
        assert concurrence(c) < 1e-9, e0  # a product state, not merely one without a singlet part


def test_e36_the_antisymmetrised_arrows_are_the_singlet_and_vanish_at_the_hat():
    m = SpectreHatTime(corner_deg=120.0)
    for tau in (0.0, 0.25, 0.5, 0.75, 0.95, 1.0):
        for s in (1, -1):
            ar = m.arrows(tau, s)
            a_in, a_out = math.atan2(ar.a_in[1], ar.a_in[0]), math.atan2(ar.a_out[1], ar.a_out[0])
            k1, k2 = kappa(a_in), kappa(a_out)
            assert abs(np.vdot(k1, k2) - math.cos(0.5 * ar.turn)) < 1e-9
            A = np.kron(k1, k2) - np.kron(k2, k1)
            want = 2.0 * math.sin(0.5 * m.omega(tau)) ** 2
            assert abs(np.linalg.norm(A) ** 2 - want) < 1e-9, (tau, s)
            if tau < 1.0:
                assert abs(Q.fidelity(A) - 1.0) < 1e-9, (tau, s)
            else:
                assert np.linalg.norm(A) < 1e-9


def _s_minus_valid(om, k0):
    return is_simple_tile(SpectreHatTime(corner_deg=om, k0=k0).tile(0.0, -1))


def omega_c(k0):
    """Largest omega_0 (degrees) at which the s = -1 outline at tau = 0 is a simple closed curve (bisection; first failure found on a 1 degree scan)."""
    first_bad = next(om for om in range(100, 180) if not _s_minus_valid(float(om), k0))
    lo, hi = 100.0, float(first_bad)
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if _s_minus_valid(mid, k0) else (lo, mid)
    return lo, first_bad


def test_e37_the_relative_sign_minus_one_needs_omega_pi_which_the_geometry_excludes():
    for om in (30.0, 90.0, 120.0):
        m = SpectreHatTime(corner_deg=om)
        a_p, a_m = m.arrows(0.0, 1), m.arrows(0.0, -1)
        rel = np.exp(0.5j * (a_m.turn - a_p.turn))  # exp(i omega)
        assert abs(rel - np.exp(1j * math.radians(om))) < 1e-9
    want = {0.3: (132.97, 133), 0.5: (132.13, 133), 0.8: (120.47, 121)}
    for k0, (oc, first_bad) in want.items():
        got, fb = omega_c(k0)
        assert abs(got - oc) < 0.02 and fb == first_bad, (k0, got, fb)
        assert all(not _s_minus_valid(float(om), k0) for om in range(fb, 180, 4)), k0  # larger angles stay invalid
        assert 180.0 - got > 47.0, (k0, got)


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
        print("  %-92s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    m = SpectreHatTime(corner_deg=30.0)
    tile = m.tile(0.0, 1)
    print("letters (direction v -> edge):", letter_edges(tile))
    print("sign patterns of the 6 letters, by base edge:")
    for e0 in range(N_EDGES):
        print("  e0 = %2d  %s" % (e0, "".join(map(str, sign_pattern(tile, e0)))))
    uni = np.ones(6) / math.sqrt(6.0)
    print("singlet fidelity by base edge, uniform origin:", [round(Q.fidelity(chi(tile, e, uni)), 3) for e in range(N_EDGES)])
    for w in (1, 3):
        print("singlet fidelity by base edge, winding %+d:  " % w, [round(Q.fidelity(chi(tile, e, odd_wind(w))), 3) for e in range(N_EDGES)])
    print("omega_c (largest omega_0 with a valid s = -1 outline at tau = 0):")
    for k0 in (0.3, 0.5, 0.8):
        oc, _ = omega_c(k0)
        print("  k_0 = %.1f: %.2f deg, gap to pi %.2f deg" % (k0, oc, 180.0 - oc))


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
