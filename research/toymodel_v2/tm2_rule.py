#!/usr/bin/env python3
"""tm2_rule.py - Stage 4 of PLAN.md: a correlation rule from a history weight with a boundary term at each end (the paper's open problem 12), and what 'checking' costs.

The paper's rule (Result 'rule'): weight each origin direction v by exp(kappa A B), A and B the sign read-outs of the two settings at v, normalised per pair of settings.
It was chosen, not derived.  Here a rule is computed from a history weight and compared with it.  Nothing here is a model of the singlet state or of gravity.

Definitions
  D4.1  Two-wing history.  An origin letter v (one of the six hexagonal roots); Alice's wing s_A(0) = v, ..., s_A(n); Bob's wing s_B(0) = -v, ..., s_B(n) (the shared axis
        of the paper's fusion at t = 0).  Each wing has the first term of the paper's action, weight exp(-beta sum_t N(s(t) - s(t-1))), N the planar norm^2 (0, 1, 3, 4 for
        letters 0, 60, 120, 180 degrees apart).  The flux term is off (lam = 0).
  D4.2  Boundary term.  A setting is a direction a (an angle in units of 30 degrees: even = a root, odd = between two roots; the 'twelve read-outs').  The outcome alpha = +-1
        has weight exp(mu alpha a.s_A(n)); the same for Bob with (b, beta_out).  ``mu = None`` is the hard pin s_A(n) = alpha a (a must be a root), the limit mu -> infinity.
  D4.3  P(alpha, beta | a, b) = W(alpha, beta; a, b) / Z(a, b), with Z(a, b) the sum of W over the outcomes: the ensemble is the histories that satisfy both end
        conditions.  This normalisation per pair of settings is a conditioning (post-selection) and is where the dependence on both settings enters.
  D4.4  Check weight Z(a, b) and its contrast max_b Z / min_b Z.  For hard pins Z(a, b) / Z_free is the fraction of free histories that pass the two-end check.
  D4.5  The past ensemble P(v | a, b) = g(v, a) g(-v, b) / Z(a, b) and delta = max |P(v | a, b) - 1/6|, the paper's price variable.
  D4.6  The paper's rule: P(v | a, b) proportional to exp(kappa A B), A = sgn(a.v), B = -sgn(b.v).
  D4.7  Stage 4b.  Amplitudes: each wing is a unitary U on the six letters; Psi(s_A, s_B) = sum_v U[s_A, v] U[s_B, -v] / sqrt 6.  Complete read-out: outcome = sgn(a.s_end)
        (0 allowed for the odd settings).  Post-selected read-out: the hard pins, P proportional to |Psi(alpha a, beta b)|^2.
  D4.8  Check entropy of the chain: h_j = H2(sin^2(theta_j / 2)) bits per tick, H = sum_j h_j; 2^H is the effective number of bit histories behind one present.

Exact claims (each a test; ``python3 tm2_rule.py --tests-only``)
  E4.1  The transfer-matrix table equals brute-force enumeration of both wings (n <= 3, hard and soft, 6 and 12 settings) to 1e-12.
  E4.2  Closed form for hard pins.  With lambda_k = 1 + 2 e^(-beta) cos(pi k/3) + 2 e^(-3 beta) cos(2 pi k/3) + (-1)^k e^(-4 beta) and r_k = (lambda_k / lambda_0)^(2n),
        E(m) = -[2 r_1 cos(pi m/3) + (-1)^m r_3] / [1 + 2 r_2 cos(2 pi m/3)] for settings m x 60 degrees apart.  E(180 - theta) = -E(theta).  The best CHSH value on the six
        roots is E_0 + 3 E_1 (E_m = |E(m)|).  Long or soft wings (r_2, r_3 << r_1): E = -2 r_1 cos(theta), a cosine law with visibility 2 r_1 < 4/5, CHSH < 2.
        Stiff wings (beta n -> infinity at fixed n): E -> -sgn(a.b), the paper's sign read-out, CHSH -> 4.
  E4.3  No-signalling is exact (1e-14) for every beta, n, mu and for the six and the twelve settings: each marginal is 1/2.  Reason: the weight is invariant under the
        inversion (v, wings, alpha, beta) -> (-v, -wings, -alpha, -beta).  There are no zero outcomes: a.s = 0 gives equal weights.  Control: a bias exp(eta u.s) on the wings
        breaks the inversion and Alice's marginal then depends on Bob's setting (0.535 to 0.571 over b for eta = 0.3).
  E4.4  The AB form does not come out.  (a) Derived: for each v the unnormalised weight of the past is a product g(v, a) g(-v, b) (rank 1 over the pairs of settings);
        the rule's exp(kappa A B) has rank 2 for kappa != 0.  (b) The rule depends on the labelling of Bob's outcome: renaming b -> -b with the outcome flipped is the same
        measurement and maps kappa -> -kappa, so E_rule(180 - theta) != -E_rule(theta) (kappa = 0.5: E(60) = +0.152, E(120) = +0.689); the derived table is covariant.
        (c) The rule's CHSH values are reproduced here: 2.257, 2.494, 3.068, 3.620, 3.946 at kappa = 0.1, 0.2, 0.5, 1, 2.
  E4.5  The strength is fixed by the model parameters.  Hard pins, n = 1: CHSH rises strictly with beta, passes 2 at beta = 0.841 and 2 sqrt 2 at beta = 1.255; at fixed beta it
        falls strictly with the wing length n.  There is no free kappa: the role is played by (beta, n) through r_1, r_2, r_3, and by mu.
  E4.6  The price is paid, and overpaid.  On the grid of the story every derived table has CHSH <= min(4, 2 + 12 delta) (the paper's bound, here only checked, since the
        derived read-outs are not deterministic), and CHSH > 2 only with delta >= 0.14: e.g. beta = 1, n = 1: CHSH 2.373 at delta = 0.207, where the bound allows 4.
  E4.7  The check weight depends on both settings: hard pins give Z(m) proportional to 1 + 2 rho cos(2 pi m/3), rho = r_2, so the contrast is (1 + 2 rho) / (1 - rho).
        It is 1 only for rho = 0, where CHSH -> 0; at the two crossings of E4.5 it is 1.21 and 1.68 (delta = 0.169 and 0.254).  Outcomes carry no signal; the rate at which histories pass the check does.
  E4.8  Wing length against the past: delta, the contrast and CHSH all fall strictly with n (beta = 2: delta = 0.315, 0.264, 0.141, 0.030 at n = 1, 2, 4, 8).
        Short wings: the check revises the past; long wings: it passes at the chance rate and leaves the past uniform.
  E4.9  Stage 4b, complete read-outs: for any unitary wings (rotation invariant or not) the table is the classical distribution |Psi(s_A, s_B)|^2 with local read-outs of the
        ends, so CHSH <= 2 (200 random unitaries, six and twelve settings; 2 is reached at U = 1) and no-signalling is exact with the zero outcomes included.  Complex amplitudes
        alone do not help: the settings never act before the read-out.
  E4.10 Stage 4b, post-selected read-outs: for the quantum walk U = exp(-i t H) on the hexagon of letters CHSH tends to 4 as t -> 0 (the stiff limit again), lies above
        2 sqrt 2 for 36% of t in (0, 3) and goes down to 1.01.  No bound at 2 sqrt 2 appears.  [Negative for a Tsirelson bound from this construction]
  E4.11 Check entropy (D4.8): the probabilities |amplitude|^2 of the 2^n bit histories are a product distribution, so their entropy is exactly H = sum_j h_j (1e-12 against
        enumeration).  Default chain: H = 0.216 bit of a possible 12, 2^H = 1.16 effective histories against 4096 enumerated; h_j rises strictly along the flow and 95% of H
        is in the last three ticks.  Checking the frozen past costs nothing; the cost is where the bit is plastic.

Hypothesis / not done
  * That nature conditions on both ends.  D4.3 is the assumption that does all the work; with it any stiff shared history violates CHSH, without it none does (E4.9).
  * kappa as a function of the overlap c (PLAN.md C6 b): not derived.  The strength here is set by beta, n and mu, which are not tied to the chain.
  * A read-out in which the setting acts on the state before it is read (needed for a quantum bound) is not built.
  * 'Checking is a process in time': if the time a check takes is the inverse of the pass rate, E4.7 says that time depends on both settings.  Not modelled.

    python3 tm2_rule.py --tests-only
    python3 tm2_rule.py --story
    python3 tm2_rule.py --no-show --save figures/tm2_stage4_rule.png
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import core  # noqa: E402

NORMS = (0, 1, 3, 4, 3, 1)  # planar norm^2 of the difference of two roots k x 60 degrees apart
D6 = np.array([[math.cos(math.pi * k / 3.0), math.sin(math.pi * k / 3.0)] for k in range(6)])
HEX_SETTINGS = (0, 2, 4, 6, 8, 10)
TWELVE = tuple(range(12))
TSIRELSON = 2.0 * math.sqrt(2.0)


def unit(a):
    """Unit vector of the setting a (units of 30 degrees)."""
    return np.array([math.cos(math.pi * a / 6.0), math.sin(math.pi * a / 6.0)])


def transfer(beta, eta=0.0, u=0):
    """T[k, l] = exp(-beta N(k - l)), times the control bias exp(eta u.s_l) if eta != 0."""
    if not (math.isfinite(beta) and beta >= 0.0):
        raise ValueError("beta must be finite and >= 0")
    T = np.array([[math.exp(-beta * NORMS[(k - l) % 6]) for l in range(6)] for k in range(6)])
    if eta:
        T = T * np.exp(eta * (D6 @ unit(u)))[None, :]
    return T


def boundary(a, mu):
    """Boundary factors, shape (2, 6): row 0 for outcome +1, row 1 for -1 (D4.2)."""
    x = D6 @ unit(a)
    if mu is None:
        if a % 2:
            raise ValueError("a hard pin needs a root (an even setting), got %r" % (a,))
        return np.array([(np.abs(x - 1.0) < 1e-9).astype(float), (np.abs(x + 1.0) < 1e-9).astype(float)])
    if not (math.isfinite(mu) and mu >= 0.0):
        raise ValueError("mu must be None or finite and >= 0")
    return np.array([np.exp(mu * x), np.exp(-mu * x)])


class TwoWing:
    """Exact tables of D4.1 to D4.5 for wings of n steps."""

    def __init__(self, beta=1.0, n=1, mu=None, eta=0.0, u=0):
        if int(n) != n or n < 0:
            raise ValueError("n must be an integer >= 0")
        self.beta, self.n, self.mu, self.eta, self.u = float(beta), int(n), mu, float(eta), u
        self.G = np.linalg.matrix_power(transfer(beta, eta, u), self.n)

    def _ends(self, a, b):
        A = boundary(a, self.mu) @ self.G.T                       # A[alpha, v]
        B = np.roll(boundary(b, self.mu) @ self.G.T, -3, axis=1)   # B[beta, v]: Bob's wing starts at -v
        return A, B

    def weights(self, a, b):
        """W[alpha, beta] (index 0 = +1, 1 = -1), unnormalised."""
        A, B = self._ends(a, b)
        return A @ B.T

    def Z(self, a, b):
        return float(self.weights(a, b).sum())

    def table(self, a, b):
        W = self.weights(a, b)
        z = W.sum()
        if z <= 0.0:
            raise ValueError("no history satisfies the end conditions for settings %r, %r" % (a, b))
        return W / z

    def E(self, a, b):
        P = self.table(a, b)
        return float(P[0, 0] + P[1, 1] - P[0, 1] - P[1, 0])

    def past(self, a, b):
        """P(v | a, b) (D4.5)."""
        A, B = self._ends(a, b)
        h = A.sum(axis=0) * B.sum(axis=0)
        return h / h.sum()

    def past_factors(self, a, b):
        """g(v, a) and g(-v, b) of D4.5, shape (6,) each."""
        A, B = self._ends(a, b)
        return A.sum(axis=0), B.sum(axis=0)

    def delta(self, settings=HEX_SETTINGS):
        return max(float(np.abs(self.past(a, b) - 1.0 / 6.0).max()) for a in settings for b in settings)

    def E_table(self, settings=HEX_SETTINGS):
        return np.array([[self.E(a, b) for b in settings] for a in settings])

    def contrast(self, settings=HEX_SETTINGS):
        z = np.array([[self.Z(a, b) for b in settings] for a in settings])
        return float((z.max(axis=1) / z.min(axis=1)).max())

    def marginal_deviation(self, settings=HEX_SETTINGS):
        """Largest |P(alpha = +1 | a, b) - 1/2| and |P(beta = +1 | a, b) - 1/2|."""
        dev = 0.0
        for a in settings:
            for b in settings:
                P = self.table(a, b)
                dev = max(dev, abs(P[0].sum() - 0.5), abs(P[:, 0].sum() - 0.5))
        return dev

    def chsh(self, settings=HEX_SETTINGS):
        return chsh(self.E_table(settings))


def chsh(E):
    """Largest |E(a,b) + E(a,b') + E(a',b) - E(a',b')| over the settings of the square table E."""
    n = len(E)
    best = 0.0
    for a, a2 in itertools.permutations(range(n), 2):
        for b, b2 in itertools.permutations(range(n), 2):
            best = max(best, abs(E[a, b] + E[a, b2] + E[a2, b] - E[a2, b2]))
    return float(best)


def brute_table(a, b, beta, n, mu):
    """D4.3 by enumeration of the origin and both wings."""
    W = np.zeros((2, 2))
    bA, bB = boundary(a, mu), boundary(b, mu)
    for v in range(6):
        for wa in itertools.product(range(6), repeat=n):
            sa = (v,) + wa
            wA = math.exp(-beta * sum(NORMS[(sa[t] - sa[t - 1]) % 6] for t in range(1, n + 1)))
            for wb in itertools.product(range(6), repeat=n):
                sb = ((v + 3) % 6,) + wb
                wB = math.exp(-beta * sum(NORMS[(sb[t] - sb[t - 1]) % 6] for t in range(1, n + 1)))
                W += wA * wB * np.outer(bA[:, sa[-1]], bB[:, sb[-1]])
    return W / W.sum()


def eigen(beta):
    """lambda_0..lambda_3 of the transfer matrix (E4.2)."""
    e1, e3, e4 = math.exp(-beta), math.exp(-3.0 * beta), math.exp(-4.0 * beta)
    return [1.0 + 2.0 * e1 * math.cos(math.pi * k / 3.0) + 2.0 * e3 * math.cos(2.0 * math.pi * k / 3.0) + (-1) ** k * e4 for k in range(4)]


def closed_form_E(m, beta, n):
    lam = eigen(beta)
    r1, r2, r3 = [(lam[k] / lam[0]) ** (2 * n) for k in (1, 2, 3)]
    return -(2.0 * r1 * math.cos(math.pi * m / 3.0) + (-1) ** m * r3) / (1.0 + 2.0 * r2 * math.cos(2.0 * math.pi * m / 3.0))


def crossing(target, n=1, lo=0.0, hi=6.0):
    """beta at which the hard-pin CHSH on the six roots equals ``target`` (bisection; CHSH rises with beta)."""
    f = lambda x: TwoWing(x, n).chsh() - target  # noqa: E731
    if f(lo) > 0 or f(hi) < 0:
        raise ValueError("target not bracketed")
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) < 0 else (lo, mid)
    return 0.5 * (lo + hi)


# -- the paper's rule ---------------------------------------------------------------------------------------
def _sgn(x):
    return np.where(np.abs(x) < 1e-9, 0.0, np.sign(x))


def rule_E(a, b, kappa, flip_b=False):
    """E(a, b) of the rule (D4.6); ``flip_b`` describes Bob's measurement by -b with the outcome renamed (the same measurement)."""
    A = _sgn(D6 @ unit(a))
    B = -_sgn(D6 @ unit(b))
    w = np.exp(kappa * A * (B if not flip_b else -B))   # the rule applied to the renamed read-out
    w = w / w.sum()
    return float((w * A * B).sum())


def rule_table(kappa, settings=HEX_SETTINGS):
    return np.array([[rule_E(a, b, kappa) for b in settings] for a in settings])


# -- Stage 4b -------------------------------------------------------------------------------------------------
HOP = np.array([[1.0 if (k - l) % 6 in (1, 5) else 0.0 for l in range(6)] for k in range(6)])


def walk_unitary(t):
    """exp(-i t H), H the hopping between neighbouring letters."""
    w, V = np.linalg.eigh(HOP)
    return (V * np.exp(-1j * t * w)) @ V.conj().T


def random_unitary(rng, circulant=False):
    if circulant:
        ph = rng.uniform(0, 2 * math.pi, 6)
        F = np.exp(2j * math.pi * np.outer(np.arange(6), np.arange(6)) / 6.0) / math.sqrt(6.0)
        return (F * np.exp(1j * ph)) @ F.conj().T
    q, r = np.linalg.qr(rng.normal(size=(6, 6)) + 1j * rng.normal(size=(6, 6)))
    return q * (np.diag(r) / np.abs(np.diag(r)))


def pair_state(U):
    """Psi[s_A, s_B] of D4.7."""
    return (U @ np.roll(U, 3, axis=1).T) / math.sqrt(6.0)   # sum_v U[sA, v] U[sB, v + 3]


def complete_tables(U, settings):
    """E table and the largest dependence of a marginal on the far setting, for the complete sign read-out (zero outcomes kept)."""
    p = np.abs(pair_state(U)) ** 2
    sg = {a: _sgn(D6 @ unit(a)) for a in settings}
    E = np.array([[float(sg[a] @ p @ sg[b]) for b in settings] for a in settings])
    pa, pb = p.sum(axis=1), p.sum(axis=0)
    # marginals of Alice: sum over s_B of p, independent of b by construction; report the numerical spread
    spread = 0.0
    for a in settings:
        m = [float(((sg[a] == o) * pa).sum()) for o in (1.0, 0.0, -1.0)]
        for b in settings:
            joint = [float(sum(p[i, j] for i in range(6) for j in range(6) if sg[a][i] == o)) for o in (1.0, 0.0, -1.0)]
            spread = max(spread, max(abs(x - y) for x, y in zip(m, joint)))
    assert abs(pb.sum() - 1.0) < 1e-12
    return E, spread


def postselected_E(U, settings=HEX_SETTINGS):
    psi = pair_state(U)
    E = np.zeros((len(settings), len(settings)))
    for i, a in enumerate(settings):
        for j, b in enumerate(settings):
            ka, kb = a // 2, b // 2
            W = np.array([[abs(psi[(ka + 3 * x) % 6, (kb + 3 * y) % 6]) ** 2 for y in (0, 1)] for x in (0, 1)])
            z = W.sum()
            E[i, j] = (W[0, 0] + W[1, 1] - W[0, 1] - W[1, 0]) / z if z > 1e-300 else 0.0
    return E


# -- D4.8 ---------------------------------------------------------------------------------------------------------
def H2(p):
    p = min(max(float(p), 0.0), 1.0)
    return 0.0 if p in (0.0, 1.0) else -(p * math.log2(p) + (1.0 - p) * math.log2(1.0 - p))


def check_entropy(angles):
    """h_j = H2(sin^2(theta_j / 2)) for each tick."""
    return np.array([H2(math.sin(th / 2.0) ** 2) for th in angles])


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    except Exception:  # noqa: BLE001
        return False
    return False


def test_e41_transfer_matrix_equals_enumeration():
    for beta, n, mu, a, b in ((1.0, 1, None, 0, 2), (0.7, 2, None, 4, 10), (1.3, 3, None, 2, 2), (1.0, 2, 1.5, 1, 6), (0.4, 3, 0.8, 5, 11), (2.0, 1, 3.0, 3, 3)):
        assert np.allclose(TwoWing(beta, n, mu).table(a, b), brute_table(a, b, beta, n, mu), atol=1e-12), (beta, n, mu, a, b)
    assert NORMS == tuple(int(round(core.planar_norm2_hex(tuple(np.subtract(core.HEX[k], core.HEX[0]))))) for k in range(6)), "N is the norm of core.HEX"


def test_e42_closed_form_and_limits():
    for beta in (0.5, 1.0, 2.0):
        for n in (1, 2, 5):
            tw = TwoWing(beta, n)
            for m in range(6):
                assert math.isclose(tw.E(0, 2 * m), closed_form_E(m, beta, n), rel_tol=1e-10, abs_tol=1e-13)
                assert math.isclose(tw.E(4, (4 + 2 * m) % 12), tw.E(0, 2 * m), abs_tol=1e-13), "depends on the angle only"
                assert math.isclose(tw.E(0, (2 * m + 6) % 12), -tw.E(0, 2 * m), abs_tol=1e-13), "b -> -b flips the sign"
            e0, e1 = abs(tw.E(0, 0)), abs(tw.E(0, 2))
            assert math.isclose(tw.chsh(), e0 + 3.0 * e1, rel_tol=1e-12)
    soft = TwoWing(0.5, 6)
    lam = eigen(0.5)
    r1 = (lam[1] / lam[0]) ** 12
    for m in range(4):
        assert math.isclose(soft.E(0, 2 * m), -2.0 * r1 * math.cos(math.pi * m / 3.0), abs_tol=2e-5), "cosine law for soft wings"
    assert 2.0 * r1 < 0.8 and soft.chsh() < 2.0
    stiff = TwoWing(8.0, 1)
    assert np.allclose([stiff.E(0, 2 * m) for m in range(4)], [-1.0, -1.0, 1.0, 1.0], atol=2e-3) and stiff.chsh() > 3.99
    assert _raises(ValueError, TwoWing(1.0, 0).table, 0, 2), "n = 0: the ends are v and -v, most pins cannot be met"
    assert math.isclose(TwoWing(1.0, 0).E(0, 0), -1.0)


def test_e43_no_signalling_exact_and_the_control():
    for beta, n, mu in ((1.0, 1, None), (2.0, 3, None), (0.3, 2, 0.7), (2.5, 1, 6.0), (1.0, 4, 2.0)):
        tw = TwoWing(beta, n, mu)
        assert tw.marginal_deviation(HEX_SETTINGS) < 1e-14
        if mu is not None:
            assert tw.marginal_deviation(TWELVE) < 1e-14
    assert _raises(ValueError, boundary, 3, None)
    ctrl = TwoWing(2.0, 2, 2.0, eta=0.3, u=1)
    marg = [ctrl.table(0, b)[0].sum() for b in TWELVE]
    assert math.isclose(min(marg), 0.535, abs_tol=1e-3) and math.isclose(max(marg), 0.571, abs_tol=1e-3), "broken inversion symmetry signals"
    assert TwoWing(2.0, 2, 2.0, eta=0.0, u=1).marginal_deviation(TWELVE) < 1e-14


def test_e44_the_ab_form_does_not_come_out():
    tw = TwoWing(1.5, 2, 2.0)
    for v in range(6):
        M = np.array([[tw.past_factors(a, b)[0][v] * tw.past_factors(a, b)[1][v] for b in TWELVE] for a in TWELVE])
        sv = np.linalg.svd(M, compute_uv=False)
        assert sv[1] < 1e-12 * sv[0], "derived: rank 1 over the pairs of settings"
        A, B = _sgn(D6[v] @ np.array([unit(a) for a in HEX_SETTINGS]).T), -_sgn(D6[v] @ np.array([unit(b) for b in HEX_SETTINGS]).T)
        R = np.exp(0.5 * np.outer(A, B))
        sr = np.linalg.svd(R, compute_uv=False)
        assert sr[1] > 0.1 * sr[0] and sr[2] < 1e-12 * sr[0], "rule: rank 2"
        assert np.linalg.svd(np.exp(0.0 * np.outer(A, B)), compute_uv=False)[1] < 1e-12
    assert [round(chsh(rule_table(k)), 3) for k in (0.0, 0.1, 0.2, 0.5, 1.0, 2.0)] == [2.0, 2.257, 2.494, 3.068, 3.62, 3.946]
    assert math.isclose(rule_E(0, 2, 0.5), 0.152, abs_tol=1e-3) and math.isclose(rule_E(0, 4, 0.5), 0.689, abs_tol=1e-3)
    assert abs(rule_E(0, 2, 0.5) + rule_E(0, 8, 0.5)) > 0.5, "the rule is not covariant under b -> -b"
    for k in (0.3, 1.0):
        for b in HEX_SETTINGS:
            assert math.isclose(rule_E(0, b, k, flip_b=True), rule_E(0, b, -k), abs_tol=1e-14), "renaming Bob's outcome is kappa -> -kappa"
    assert math.isclose(rule_E(0, 2, 0.0) + rule_E(0, 8, 0.0), 0.0, abs_tol=1e-14), "only kappa = 0 is covariant"


def test_e45_the_strength_is_fixed_by_the_parameters():
    betas = np.linspace(0.05, 5.0, 40)
    s = [TwoWing(x, 1).chsh() for x in betas]
    assert np.all(np.diff(s) > 0) and s[0] < 0.1 and s[-1] > 3.97
    b2, bt = crossing(2.0), crossing(TSIRELSON)
    assert math.isclose(b2, 0.841, abs_tol=1e-3) and math.isclose(bt, 1.255, abs_tol=1e-3), (b2, bt)
    for beta in (1.0, 2.0, 3.0):
        sn = [TwoWing(beta, n).chsh() for n in range(1, 10)]
        assert np.all(np.diff(sn) < 0)
    for mu_lo, mu_hi in ((0.5, 1.0), (1.0, 2.0), (2.0, 4.0)):
        assert TwoWing(2.0, 2, mu_lo).chsh(TWELVE) < TwoWing(2.0, 2, mu_hi).chsh(TWELVE)
    assert math.isclose(TwoWing(3.0, 1, 40.0).chsh(), TwoWing(3.0, 1, None).chsh(), rel_tol=1e-7), "the hard pin is the limit of the soft one"


def test_e46_the_price_is_paid_and_overpaid():
    worst = 0.0
    for beta in (0.5, 1.0, 2.0, 3.0):
        for n in (1, 2, 4, 8):
            for mu, st in ((None, HEX_SETTINGS), (1.0, TWELVE), (4.0, TWELVE)):
                tw = TwoWing(beta, n, mu)
                s, d = tw.chsh(st), tw.delta(st)
                assert s <= min(4.0, 2.0 + 12.0 * d) + 1e-12, (beta, n, mu, s, d)
                if s > 2.0:
                    assert d >= 0.14, (beta, n, mu, s, d)
                worst = max(worst, s - 2.0 - 12.0 * d)
    tw = TwoWing(1.0, 1)
    assert math.isclose(tw.chsh(), 2.373, abs_tol=1e-3) and math.isclose(tw.delta(), 0.207, abs_tol=1e-3)
    assert TwoWing(0.0, 3, 2.0).delta(TWELVE) < 1e-15 and TwoWing(0.0, 3, 2.0).chsh(TWELVE) < 1e-12, "no stiffness: the past is uniform and nothing is correlated"


def test_e47_the_check_weight_depends_on_both_settings():
    for beta, n in ((1.0, 1), (2.0, 2), (3.0, 4)):
        tw = TwoWing(beta, n)
        lam = eigen(beta)
        rho = (lam[2] / lam[0]) ** (2 * n)
        z = np.array([tw.Z(0, 2 * m) for m in range(6)])
        assert np.allclose(z / z.mean(), [1.0 + 2.0 * rho * math.cos(2.0 * math.pi * m / 3.0) for m in range(6)], atol=1e-12)
        assert math.isclose(tw.contrast(), (1.0 + 2.0 * rho) / (1.0 - rho), rel_tol=1e-12)
        free = 6.0 * lam[0] ** (2 * n)   # all two-wing histories
        assert math.isclose(sum(tw.Z(a, b) for a in (0, 2, 4) for b in (0, 2, 4)), free, rel_tol=1e-12), "the nine axis pairs partition the free histories"
    c2, ct = TwoWing(crossing(2.0), 1).contrast(), TwoWing(crossing(TSIRELSON), 1).contrast()
    assert math.isclose(c2, 1.207, abs_tol=2e-3) and math.isclose(ct, 1.682, abs_tol=2e-3), (c2, ct)
    assert TwoWing(1.0, 30).contrast() < 1.0 + 1e-9 and TwoWing(1.0, 30).chsh() < 1e-6


def test_e48_wing_length_against_the_past():
    for beta in (1.0, 2.0, 3.0):
        d = [TwoWing(beta, n).delta() for n in range(1, 9)]
        c = [TwoWing(beta, n).contrast() for n in range(1, 9)]
        assert np.all(np.diff(d) < 0) and np.all(np.diff(c) < 0)
    assert [round(TwoWing(2.0, n).delta(), 3) for n in (1, 2, 4, 8)] == [0.315, 0.264, 0.141, 0.03]


def test_e49_complex_amplitudes_with_complete_readouts_stay_local():
    rng = np.random.default_rng(4)
    best = 0.0
    for i in range(200):
        U = random_unitary(rng, circulant=(i % 2 == 0))
        assert np.allclose(U @ U.conj().T, np.eye(6), atol=1e-12)
        psi = pair_state(U)
        assert math.isclose(float((np.abs(psi) ** 2).sum()), 1.0, rel_tol=1e-12)
        for st in (HEX_SETTINGS, TWELVE) if i < 20 else (HEX_SETTINGS,):
            E, spread = complete_tables(U, st)
            s = chsh(E)
            assert s <= 2.0 + 1e-12 and spread < 1e-14
            best = max(best, s)
    E, _ = complete_tables(np.eye(6, dtype=complex), HEX_SETTINGS)
    assert math.isclose(chsh(E), 2.0, rel_tol=1e-12) and best <= 2.0 + 1e-12
    assert np.allclose(E[0, :4], [-1.0, -1.0 / 3.0, 1.0 / 3.0, 1.0], atol=1e-12), "U = 1 is the shared-past model: the saw-tooth"


def test_e410_postselected_amplitudes_have_no_tsirelson_bound():
    ts = np.linspace(0.01, 3.0, 600)
    s = np.array([chsh(postselected_E(walk_unitary(t))) for t in ts])
    assert s.max() > 3.99 and math.isclose((s > TSIRELSON).mean(), 0.36, abs_tol=0.02) and math.isclose(s.min(), 1.015, abs_tol=5e-3)
    assert chsh(postselected_E(walk_unitary(1e-3))) > 3.9, "a short walk is the stiff limit"
    # the walk is a unitary and its post-selected marginals are uniform (inversion symmetry again)
    psi = pair_state(walk_unitary(0.7))
    for ka in range(3):
        for kb in range(3):
            W = np.array([[abs(psi[(ka + 3 * x) % 6, (kb + 3 * y) % 6]) ** 2 for y in (0, 1)] for x in (0, 1)])
            assert math.isclose(W[0].sum(), W[1].sum(), rel_tol=1e-10) and math.isclose(W[:, 0].sum(), W[:, 1].sum(), rel_tol=1e-10)


def test_e411_check_entropy_of_the_chain():
    from spectre_hat_backward_snap_model import BackwardSnap
    m = BackwardSnap(ticks=12)
    h = check_entropy(m.angles())
    p = np.abs(m.paths["amp"]) ** 2
    assert math.isclose(float(p.sum()), 1.0, rel_tol=1e-12)
    ent = float(-(p[p > 0] * np.log2(p[p > 0])).sum())
    assert math.isclose(ent, float(h.sum()), rel_tol=1e-12)
    assert math.isclose(h.sum(), 0.2157, abs_tol=2e-4) and math.isclose(2.0 ** h.sum(), 1.161, abs_tol=1e-3), h.sum()
    assert np.all(np.diff(h) >= 0) and np.all(np.diff(h[3:]) > 0) and h[:3].max() < 1e-6
    assert math.isclose(h[-3:].sum() / h.sum(), 0.946, abs_tol=2e-3), h[-3:].sum() / h.sum()
    assert H2(0.5) == 1.0 and H2(0.0) == 0.0


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


# --------------------------------------------------------------------------
# Story and figure
# --------------------------------------------------------------------------
def story():
    print("derived table, hard pins, six roots:  beta  n | CHSH   delta  bound 2+12 delta  contrast | E(0), E(60)")
    for beta in (0.5, 1.0, 2.0, 3.0):
        for n in (1, 2, 4, 8):
            tw = TwoWing(beta, n)
            d = tw.delta()
            print("  %.1f %2d | %.3f  %.3f  %.3f  %7.3f | %+.3f %+.3f" % (beta, n, tw.chsh(), d, min(4.0, 2.0 + 12.0 * d), tw.contrast(), tw.E(0, 0), tw.E(0, 2)))
    print("soft boundary, twelve settings (beta = 2, n = 2):  mu | CHSH  delta  contrast  largest marginal deviation")
    for mu in (0.5, 1.0, 2.0, 4.0, 8.0):
        tw = TwoWing(2.0, 2, mu)
        print("  %.1f | %.3f  %.3f  %.3f  %.1e" % (mu, tw.chsh(TWELVE), tw.delta(TWELVE), tw.contrast(TWELVE), tw.marginal_deviation(TWELVE)))
    b2, bt = crossing(2.0), crossing(TSIRELSON)
    print("crossings (hard pins, n = 1): CHSH = 2 at beta = %.4f (contrast %.3f, delta %.3f); 2 sqrt 2 at beta = %.4f (contrast %.3f, delta %.3f)"
          % (b2, TwoWing(b2, 1).contrast(), TwoWing(b2, 1).delta(), bt, TwoWing(bt, 1).contrast(), TwoWing(bt, 1).delta()))
    print("the paper's rule: CHSH at kappa 0.1, 0.2, 0.5, 1, 2:", " ".join("%.3f" % chsh(rule_table(k)) for k in (0.1, 0.2, 0.5, 1.0, 2.0)),
          "| E(60), E(120) at kappa 0.5: %+.3f %+.3f (derived at beta = 1, n = 1: %+.3f %+.3f)" % (rule_E(0, 2, 0.5), rule_E(0, 4, 0.5), TwoWing(1.0, 1).E(0, 2), TwoWing(1.0, 1).E(0, 4)))
    ts = np.linspace(0.01, 3.0, 600)
    s = np.array([chsh(postselected_E(walk_unitary(t))) for t in ts])
    print("4b: complete read-outs CHSH <= 2 (see tests); post-selected quantum walk: max CHSH %.3f at t = %.3f, above 2 sqrt 2 for %.0f%% of t in (0, 3)" % (s.max(), ts[int(s.argmax())], 100 * (s > TSIRELSON).mean()))
    from spectre_hat_backward_snap_model import BackwardSnap
    h = check_entropy(BackwardSnap(ticks=12).angles())
    print("check entropy of the chain, bits per tick:", " ".join("%.4f" % x for x in h), "| total %.4f, 2^H = %.3f of 4096" % (h.sum(), 2.0 ** h.sum()))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(18, 4.2))
    th = np.arange(0, 181, 60)
    for beta, n in ((0.5, 6), (1.0, 1), (2.0, 1), (8.0, 1)):
        ax[0].plot(th, [TwoWing(beta, n).E(0, 2 * m) for m in range(4)], "o-", label="derived beta=%g n=%d" % (beta, n))
    ax[0].plot(th, [rule_E(0, 2 * m, 0.5) for m in range(4)], "k--s", label="rule kappa=0.5")
    ax[0].plot(th, -np.cos(np.radians(th)), "k:", label="-cos")
    ax[0].set_xlabel("angle between the settings (deg)")
    ax[0].set_ylabel("E")
    ax[0].set_title("correlation")
    ax[0].legend(fontsize=7)
    betas = np.linspace(0.05, 4.0, 60)
    for n in (1, 2, 4):
        ax[1].plot(betas, [TwoWing(x, n).chsh() for x in betas], label="n = %d" % n)
    for y in (2.0, TSIRELSON):
        ax[1].axhline(y, color="k", lw=0.6)
    ax[1].set_xlabel("stiffness beta")
    ax[1].set_ylabel("CHSH (six roots, hard pins)")
    ax[1].set_title("strength from the parameters")
    ax[1].legend(fontsize=8)
    d = [TwoWing(x, 1).delta() for x in betas]
    s = [TwoWing(x, 1).chsh() for x in betas]
    ax[2].plot(d, s, label="derived (n = 1)")
    dd = np.linspace(0, 0.34, 50)
    ax[2].plot(dd, np.minimum(4.0, 2.0 + 12.0 * dd), "k--", label="bound min(4, 2 + 12 delta)")
    ax[2].axhline(2.0, color="k", lw=0.6)
    ax[2].set_xlabel("delta: how far the check moves the past")
    ax[2].set_ylabel("CHSH")
    ax[2].set_title("the price")
    ax[2].legend(fontsize=8)
    ns = range(1, 13)
    for beta in (1.0, 2.0, 3.0):
        ax[3].semilogy(ns, [TwoWing(beta, n).contrast() - 1.0 for n in ns], "o-", label="beta = %g" % beta)
    ax[3].set_xlabel("wing length n")
    ax[3].set_ylabel("contrast of the check weight - 1")
    ax[3].set_title("the check sees both settings")
    ax[3].legend(fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
