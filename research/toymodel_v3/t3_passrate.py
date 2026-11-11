#!/usr/bin/env python3
"""t3_passrate.py - Stage 2b of research/toymodel_v3/PLAN.md: is it the settings-dependent pass rate that makes the two-ended history violate CHSH? (plan 4.1)

The two-wing history of tm2_rule.py (origin v, a wing of n letters to each party, stiffness weight exp(-beta N(step))) is read out at each end in one of three ways:
  'pin'   the end must be one of the two letters on the setting's axis (+-a); outcome = which one.  Histories whose end is elsewhere are not accepted.
  'sign'  every history is accepted; outcome = sgn(a . s_end) (never 0 for a root setting).
  'soft'  every end is accepted with probability cosh(mu a.s) / cosh(mu) <= 1, with outcome +-1 in proportion exp(+-mu a.s).
'pin' at both ends is Stage 4 of v2.  The pass rate is Z(a, b), the probability that a history is accepted at both ends.

Definitions
  D2b.1  P(alpha, beta | a, b, accepted) = W / Z: the post-selected table, from which CHSH is computed (as in v2).
  D2b.2  CH(a, a', b, b'; alpha0, beta0) = p(a,b) + p(a,b') + p(a',b) - p(a',b') - p_A(a) - p_B(b), with p the UNNORMALISED probabilities over all histories
         (accepted and with the given outcomes), p_A, p_B the single-end ones.  The Clauser-Horne combination: every local model with local acceptance has CH <= 0,
         whatever the acceptance rate (no fair-sampling assumption).

Exact claims (each a test; ``python3 t3_passrate.py --tests-only``)
  E2b.1  A constant pass rate does not remove the violation.  With Alice pinned and Bob reading the sign of every history ('pin'/'sign'), Z(a, b) does not depend
         on the settings (to 1e-15), and the post-selected CHSH is 2.847 at beta = 2 and 3.921 at beta = 5 (n = 1), above 2 sqrt 2.  With 'soft'/'sign'
         (mu = 3) it is 2.951 at beta = 5.  Only when both ends accept everything ('sign'/'sign') is CHSH <= 2 (1.982 at beta = 5).
  E2b.2  CH <= 0 in every case (all four read-out pairs, beta in {1, 2, 3, 5}, n in {1, 3}, every choice of settings and outcomes): the largest value found is
         -0.0088 (pin/pin, beta = 5).  The tables are those of a local stochastic process with a local acceptance at each end, so the post-selected violation is
         the one any local model can produce when it is allowed to discard runs: Alice accepts 1/3 of the histories under 'pin'.
  E2b.3  No marginal depends on the far setting in any mode (1e-14), and with Bob complete Bob's marginal is exactly 1/2.

What it says.  The violation of v2 is post-selection, not the pass-rate dependence: a pass rate can be made constant and the violation stays.  Two readings remain:
  (i)  the rejected histories are runs that happened and were not recorded: then the toy is a local model with the detection loophole, and CH never fails;
  (ii) the rejected histories do not exist (the ensemble IS the set of histories that meet both ends, 'nature normalises'): then CHSH > 2 is a genuine
       two-ended effect, Z(a, b) is not a frequency of anything, and the 'pass-rate leak' of the problem notes is not observable.
The toy cannot choose between them; a Bell test with CH (or with no discarded runs) is what separates them in an experiment.  [Negative for 'the conditioning is
retrocausal' as a property of the tables alone: read as data, they are local.]

    python3 t3_passrate.py --tests-only
    python3 t3_passrate.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
for _p in (_HERE, _V2):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import tm2_rule as R  # noqa: E402

MODES = ("pin", "sign", "soft")
SETTINGS = R.HEX_SETTINGS


def readout(setting, mode, mu=3.0):
    """Acceptance-and-outcome weights, shape (2, 6): row 0 outcome +1, row 1 outcome -1; each column sums to the acceptance probability (<= 1)."""
    x = R.D6 @ R.unit(setting)
    if mode == "pin":
        return np.array([(np.abs(x - 1.0) < 1e-9) * 1.0, (np.abs(x + 1.0) < 1e-9) * 1.0])
    if mode == "sign":
        return np.array([(x > 1e-9) * 1.0, (x < -1e-9) * 1.0])
    if mode == "soft":
        m = 2.0 * math.cosh(mu)
        return np.array([np.exp(mu * x) / m, np.exp(-mu * x) / m])
    raise ValueError("mode must be one of %r" % (MODES,))


class Box:
    """The two-wing tables for read-out modes (mode_A, mode_B)."""

    def __init__(self, beta, n, mode_a, mode_b, mu=3.0):
        self.beta, self.n, self.modes, self.mu = beta, n, (mode_a, mode_b), mu
        self.G = np.linalg.matrix_power(R.transfer(beta), n)
        self.lam0 = float(self.G.sum(axis=1)[0])    # every row sums to lambda_0^n (the matrix is circulant)
        self.z_free = 6.0 * self.lam0 ** 2

    def joint(self, a, b):
        """Unnormalised probabilities p[alpha, beta] over all histories (D2b.2)."""
        A = readout(a, self.modes[0], self.mu) @ self.G.T
        B = np.roll(readout(b, self.modes[1], self.mu) @ self.G.T, -3, axis=1)
        return A @ B.T / self.z_free

    def single(self, side, setting):
        w = readout(setting, self.modes[side], self.mu) @ self.G.T
        return w.sum(axis=1) / (6.0 * self.lam0)

    def pass_rate(self, a, b):
        return float(self.joint(a, b).sum())

    def E(self, a, b):
        P = self.joint(a, b)
        return float((P[0, 0] + P[1, 1] - P[0, 1] - P[1, 0]) / P.sum())

    def chsh(self):
        return R.chsh(np.array([[self.E(a, b) for b in SETTINGS] for a in SETTINGS]))

    def ch_max(self):
        J = {(a, b): self.joint(a, b) for a in SETTINGS for b in SETTINGS}
        sa = {a: self.single(0, a) for a in SETTINGS}
        sb = {b: self.single(1, b) for b in SETTINGS}
        best = -np.inf
        for a, a2 in itertools.permutations(SETTINGS, 2):
            for b, b2 in itertools.permutations(SETTINGS, 2):
                for oa in (0, 1):
                    for ob in (0, 1):
                        v = J[(a, b)][oa, ob] + J[(a, b2)][oa, ob] + J[(a2, b)][oa, ob] - J[(a2, b2)][oa, ob] - sa[a][oa] - sb[b][ob]
                        best = max(best, v)
        return float(best)

    def contrast(self):
        z = np.array([[self.pass_rate(a, b) for b in SETTINGS] for a in SETTINGS])
        return float(z.max() / z.min() - 1.0)

    def marginal_dependence(self):
        dev = 0.0
        for a in SETTINGS:
            for b in SETTINGS:
                P = self.joint(a, b)
                dev = max(dev, abs(P[0].sum() / P.sum() - 0.5), abs(P[:, 0].sum() / P.sum() - 0.5))
        return dev


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e2b1_a_constant_pass_rate_keeps_the_violation():
    for beta, n, want in ((2.0, 1, 2.847), (5.0, 1, 3.921)):
        b = Box(beta, n, "pin", "sign")
        assert b.contrast() < 1e-14
        assert math.isclose(b.chsh(), want, abs_tol=1e-3), (beta, b.chsh())
    assert Box(5.0, 1, "pin", "sign").chsh() > R.TSIRELSON
    s = Box(5.0, 1, "soft", "sign")
    assert s.contrast() < 1e-14 and math.isclose(s.chsh(), 2.951, abs_tol=1e-3), s.chsh()
    for beta in (1.0, 2.0, 3.0, 5.0):
        c = Box(beta, 1, "sign", "sign")
        assert c.chsh() <= 2.0 + 1e-12 and c.contrast() < 1e-14
    assert math.isclose(Box(5.0, 1, "sign", "sign").chsh(), 1.982, abs_tol=1e-3)
    assert Box(2.0, 1, "pin", "pin").contrast() > 1.0, "pins at both ends: the pass rate depends on the settings (v2)"


def test_e2b2_the_clauser_horne_combination_never_exceeds_zero():
    worst = -np.inf
    for ma, mb in (("pin", "pin"), ("pin", "sign"), ("soft", "sign"), ("soft", "soft")):
        for beta in (1.0, 2.0, 3.0, 5.0):
            for n in (1, 3):
                v = Box(beta, n, ma, mb).ch_max()
                assert v <= 1e-12, (ma, mb, beta, n, v)
                worst = max(worst, v)
    assert math.isclose(worst, -0.0088, abs_tol=2e-4), worst
    assert math.isclose(Box(2.0, 1, "pin", "pin").single(0, 0).sum(), 1.0 / 3.0, rel_tol=1e-12), "Alice accepts one history in three"
    assert math.isclose(Box(2.0, 1, "sign", "pin").single(0, 0).sum(), 1.0, rel_tol=1e-12)


def test_e2b3_no_signalling():
    for ma, mb in (("pin", "pin"), ("pin", "sign"), ("soft", "sign"), ("soft", "soft"), ("sign", "sign")):
        assert Box(2.0, 2, ma, mb).marginal_dependence() < 1e-14
    b = Box(3.0, 1, "pin", "sign")
    for a in SETTINGS:
        for bb in SETTINGS:
            P = b.joint(a, bb)
            assert math.isclose(P[:, 0].sum() / P.sum(), 0.5, abs_tol=1e-14)
    try:
        readout(0, "x")
        raise AssertionError("no error")
    except ValueError:
        pass


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
    print("read-out (Alice, Bob)  beta n | post-selected CHSH  pass-rate contrast  max CH (local bound 0)  Alice's acceptance")
    for ma, mb in (("pin", "pin"), ("pin", "sign"), ("soft", "sign"), ("sign", "sign")):
        for beta in (1.0, 2.0, 5.0):
            b = Box(beta, 1, ma, mb)
            print("  %-5s %-5s %.0f 1 | %.3f  %.2e  %+.4f  %.3f" % (ma, mb, beta, b.chsh(), b.contrast(), b.ch_max(), b.single(0, 0).sum()))


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
