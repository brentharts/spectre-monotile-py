#!/usr/bin/env python3
"""t3_area.py - Stage 2a of research/toymodel_v3/PLAN.md: a weight that tells a closed timeline from its reverse (plan 3.1), and the base point of the flux (3.2).

Stage 3 of toymodel_v2: the action A = sum N(s(t) - s(t-1)) + lam sum N(Phi(t)) is the same on a closed timeline and on its reverse R s (s(t) -> s(n-1-t)), although most
closed timelines are oriented.  Here a term odd under R is added: the signed area enclosed by the closed timeline.

Definitions
  D2a.1  Planar cross product of letters p + q w (w = e^{i pi/3}): u x v = (sqrt3 / 2) K(u, v), K(u, v) = p_u q_v - q_u p_v (an integer).
         Signed area of a closed timeline: Area(s) = (1/2) sum_t Phi(t) x s(t+1) = (sqrt3 / 4) K(s), K(s) = sum_t K(Phi(t), s(t+1)) (an integer).
  D2a.2  Weight with an area term: W_theta(s) = exp(-beta A(s) - theta K(s)) on the closed timelines of n steps.  theta = 0 is Stage 1 of v2.
  D2a.3  A symmetric base point: A_sym = sum N(s(t) - s(t-1)) + (lam / 2) sum_t [N(Phi_fwd(t)) + N(Phi_bwd(t))], with Phi_bwd the flux counted from the other end.
  D2a.4  Mirror M: the reflection (p, q) -> (q, p) of the hexagonal lattice applied to every letter (it maps the six roots to themselves).

Exact claims (each a test; ``python3 t3_area.py --tests-only``)
  E2a.1  K is odd under reversal and under the mirror, even under the six rotations and under a cyclic shift of the start: K(R s) = K(M s) = -K(s), K(g s) = K(s),
         on all closed hexagonal timelines of 3 to 7 steps.  A_theta(R s) - A_theta(s) = -2 theta K(s) exactly, so the area term is a weight that tells s from R s
         whenever K(s) != 0.
  E2a.2  It does so for many of them: the closed timelines with K != 0 are 12 of 12 (n = 3), 24 of 90, 360 of 360, 1020 of 2040, 10080 of 10080 and 34920 of 54810 (n = 8).
         Every closed timeline with an odd number of steps encloses area.  Of the 'oriented' ones of Stage 3 (reverse not a symmetry image), all have K != 0 at
         n = 5 and 7; at n = 6 and 8, 216 of 864 and 12960 of 43488 have K = 0: the area term does not see those.  Conversely the term also separates
         timelines whose reverse is their mirror image (120, 372, 1512, 4392 with K != 0 that are not oriented at n = 5..8): it is an orientation of the plane
         as well as of the order.
  E2a.3  The measure: <K>_theta is odd in theta and 0 at theta = 0; P_theta(M s) = P_{-theta}(s) and P_theta(R s) = P_{-theta}(s) for every s (n = 6, several beta, lam).
         The arrow therefore flips with the mirror image of the whole world: if theta = theta_0 eps with eps = +-1 the handedness of the tiling, a tiling and its mirror
         image carry opposite arrows, and one binary datum (the handedness) fixes both.  The sign of theta_0 is a constant of the law, not a property of the state.
  E2a.4  The symmetric base point removes the orientation of open histories: A_sym(R s) = A_sym(s) on all 7776 open timelines of 5 steps, where the action of the paper
         differs on 6624 of them (Stage 3 of v2).

Hypothesis / not done: that a term like theta K belongs in the action (it is the form of a magnetic flux through the history; here it is chosen); its value; a coupling
of the chain's bit to K; whether the tiling's handedness can actually set eps (the outline of the Spectre has a definite sign of K, but which traversal is 'forward'
is the same convention as the handedness label).

    python3 t3_area.py --tests-only
    python3 t3_area.py --story
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
for _p in (_HERE, _V2):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import core  # noqa: E402
import tm2_arrow as ta  # noqa: E402
import tm2_history_sum as th  # noqa: E402

HEX = np.array(core.HEX)
MIRROR_INDEX = np.array([int(np.flatnonzero(np.all(HEX == HEX[k][::-1], axis=1))[0]) for k in range(6)])  # (p, q) -> (q, p)


def K_of(T):
    """K of D2a.1 for timelines T (N, n) of letter indices (closed or not: the area of the path closed by a straight return to the origin)."""
    S = HEX[T]
    P = np.cumsum(S, axis=1)
    return (P[:, :-1, 0] * S[:, 1:, 1] - P[:, :-1, 1] * S[:, 1:, 0]).sum(axis=1)


def action(T, lam=0.5):
    S = HEX[T]
    P = np.cumsum(S, axis=1)
    return th.norm2(S[:, 1:] - S[:, :-1]).sum(axis=1) + lam * th.norm2(P[:, 1:]).sum(axis=1)


def action_sym(T, lam=0.5):
    S = HEX[T]
    Pf = np.cumsum(S, axis=1)
    Pb = np.cumsum(S[:, ::-1], axis=1)
    return th.norm2(S[:, 1:] - S[:, :-1]).sum(axis=1) + 0.5 * lam * (th.norm2(Pf[:, 1:]).sum(axis=1) + th.norm2(Pb[:, 1:]).sum(axis=1))


def weights(n, beta, lam, theta):
    T = core.closed_timelines("hex", n).astype(int)
    w = np.exp(-beta * action(T, lam) - theta * K_of(T))
    return T, w / w.sum()


def index_of(T):
    """Integer code of each timeline (base 6), for looking up images."""
    return (T * (6 ** np.arange(T.shape[1])[::-1])).sum(axis=1)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e2a1_the_area_is_odd_under_reversal_and_mirror():
    for n in range(3, 8):
        T = core.closed_timelines("hex", n).astype(int)
        K = K_of(T)
        assert np.array_equal(K_of(T[:, ::-1]), -K), n
        assert np.array_equal(K_of(MIRROR_INDEX[T]), -K), n
        for r in range(6):
            assert np.array_equal(K_of((T + r) % 6), K), (n, r)
        for sh in range(1, n):
            assert np.array_equal(K_of(np.roll(T, sh, axis=1)), K), (n, sh)
        for theta in (0.3, -1.1):
            a = action(T) + theta * K
            ar = action(T[:, ::-1]) + theta * K_of(T[:, ::-1])
            assert np.allclose(ar - a, -2.0 * theta * K, atol=1e-12)
    assert np.all(HEX[MIRROR_INDEX] == HEX[:, ::-1])


def test_e2a2_how_many_are_told_apart():
    counts = [int((K_of(core.closed_timelines("hex", n).astype(int)) != 0).sum()) for n in range(3, 9)]
    assert counts == [12, 24, 360, 1020, 10080, 34920], counts
    for n in (3, 5, 7):
        T = core.closed_timelines("hex", n).astype(int)
        assert np.all(K_of(T) != 0), "odd closed timelines always enclose area"
    missed, extra = {}, {}
    for n in (5, 6, 7, 8):
        T = core.closed_timelines("hex", n).astype(int)
        R = T[:, ::-1]
        rots, refl = ta._images(T, 6)
        a, b = ta._equal_any(R, rots, True), ta._equal_any(R, refl, True)
        oriented = ~a & ~b
        missed[n] = int((oriented & (K_of(T) == 0)).sum())
        extra[n] = int((~oriented & (K_of(T) != 0)).sum())
        assert int(oriented.sum()) == ta.CENSUS[n][6]
    assert missed == {5: 0, 6: 216, 7: 0, 8: 12960} and extra == {5: 120, 6: 372, 7: 1512, 8: 4392}, (missed, extra)


def test_e2a3_the_measure_and_the_mirror():
    for beta, lam in ((1.0, 0.0), (0.7, 0.5)):
        T, p0 = weights(6, beta, lam, 0.0)
        K = K_of(T)
        assert abs((p0 * K).sum()) < 1e-13
        code = {c: i for i, c in enumerate(index_of(T))}
        mir = np.array([code[c] for c in index_of(MIRROR_INDEX[T])])
        rev = np.array([code[c] for c in index_of(T[:, ::-1])])
        means = []
        for theta in (0.2, -0.2, 0.5):
            _, p = weights(6, beta, lam, theta)
            _, q = weights(6, beta, lam, -theta)
            assert np.allclose(p[mir], q, atol=1e-15) and np.allclose(p[rev], q, atol=1e-15)
            means.append((p * K).sum())
        assert means[0] < 0 < means[1] and math.isclose(means[0], -means[1], rel_tol=1e-12) and means[2] < means[0]
        _, p = weights(6, beta, lam, 0.5)
        assert np.abs(p - p[rev]).max() > 1e-3, "with theta != 0 a timeline and its reverse have different weights"


def test_e2a4_a_symmetric_base_point_removes_the_open_arrow():
    U = core.all_timelines("hex", 5).astype(int)
    assert np.allclose(action_sym(U[:, ::-1], 1.0), action_sym(U, 1.0), atol=1e-12)
    assert int((np.abs(action(U[:, ::-1], 1.0) - action(U, 1.0)) > 1e-12).sum()) == 6624


def run_tests(verbose=True):
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-80s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    print("closed timelines with nonzero area, n = 3..8:", [int((K_of(core.closed_timelines("hex", n).astype(int)) != 0).sum()) for n in range(3, 9)])
    for theta in (0.0, 0.1, 0.3, 1.0):
        T, p = weights(6, 1.0, 0.5, theta)
        K = K_of(T)
        print("theta = %.1f: <K> = %+.4f, P(K > 0) = %.4f, P(K < 0) = %.4f" % (theta, (p * K).sum(), p[K > 0].sum(), p[K < 0].sum()))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
