#!/usr/bin/env python3
"""tm2_checking.py - where a two-ended check lands: on the shared past, or only on the ends.  Companion to PAST_AND_POTENTIAL.md (repository root).

Model: the two-wing history of tm2_rule.py (D4.1 to D4.3) with hard pins: an origin letter v, a wing of n hexagonal letters from v to Alice and one from -v to Bob,
stiffness weight exp(-beta N(step)) per step, and at each end the pin s_end = +-a (Alice) or +-b (Bob).  "The check" is the condition that both ends meet their pins;
the past is the origin v; the wing length n is the "space" between the past and each end.  Nothing here is gravity or a model of 3D space.

Definitions
  D6.1  Transfer eigenvalues lambda_k (k = 0..5, lambda_k = lambda_{6-k}) as in tm2_rule.eigen; q_k = lambda_k / lambda_0.  For the hexagonal alphabet the eigenvector k is
        exp(i pi k v / 3): k = 1 and 5 are odd under v -> -v (they carry a sign), k = 2 and 4 are even (they carry an axis only), k = 3 is odd.
  D6.2  Past shift delta(a, b) = max_v |P(v | a, b) - 1/6| (the paper's price variable); its maximum over the settings is tm2_rule's delta.
  D6.3  Profile: TV(t) = total-variation distance from uniform of the letter at node t of Alice's wing (t = 0 the origin, t = n the end), given both pins, outcomes summed.
  D6.4  Bits on the past: I_past(a, b) = log2 6 - H(P(v | a, b)).  Bits of the check: I_check(a, b) = log2(Z_free / Z(a, b)), the information that the two ends met
        their pins (Z as in tm2_rule, outcomes summed; Z_free the weight of all two-wing histories).

Exact claims (each a test; ``python3 tm2_checking.py --tests-only``)
  E6.1  Only the axis of a setting reaches the past.  Summed over the outcome, the weight an end sends back to the origin is
            g(v, a) = (lambda_0^n + 2 lambda_2^n cos(2 pi (v - a) / 3)) / 3          (v, a in units of 60 degrees)
        so the odd modes, which carry the outcome's sign, cancel, and P(v | a, b) is proportional to (1 + 2 q cos(2 pi (v - a) / 3)) (1 + 2 q cos(2 pi (v - b) / 3)), q = q_2^n.
  E6.2  Closed form of the past shift.  Its maximum over the settings is at a = b (same axis) and equals
            delta = q (2 + q) / (3 (1 + 2 q^2)),   q = (lambda_2 / lambda_0)^n,
        for every beta and n (it depends on q only), and tends to 2 q / 3.  For a = b the origin's TV distance from uniform is 2 delta.
  E6.3  Three decay rates per wing step:   past shift   ln(lambda_0 / lambda_2)       (one wing: exponent n)
                                           pass-rate contrast  2 ln(lambda_0 / lambda_2)  (both wings must reach the origin and return: exponent 2n)
                                           correlation  2 ln(lambda_0 / lambda_1)     (end to end through the origin, odd mode: exponent 2n)
        and the order is contrast > past shift > correlation for every beta > 0 on the grid tested (0.05 to 8), because lambda_1^2 > lambda_0 lambda_2.
        So with growing space the setting-dependence of the pass rate goes first, the past stops being revised next, and the correlation between the ends goes last.
  E6.4  The profile rises monotonically from the origin to the end, where it is 2/3 (the pinned letter is one of two of six); the end of each wing is always fully
        checked, whatever n.  The origin's share falls with n.
  E6.5  Bits.  Averaged over the nine axis pairs, I_check = log2 9 - (a term that vanishes with n), and for every n the nine pass weights sum to Z_free.
        The bits that land on the past fall from 1.322 at n = 1 to 0.240 at n = 4 and 0.011 at n = 8 (beta = 2, a = b).
  E6.6  Short space and long space.  Where the past is revised by delta < 0.14 the CHSH value on the six roots is below 2 (grid of tm2_rule, E4.6), while |E| at
        zero angle can still be large: at beta = 3, n = 16, delta = 0.066 and |E(0)| = 0.442 but CHSH = 1.117.  Correlated ends with an almost unrevised past is the
        long-space regime; a violation needs both stiffness (the correlation) and short space (the revision of the past).

Not shown: anything about distance in space (n counts letters of a history); any reason the ends should be conditioned on (the assumption of tm2_rule, D4.3).

    python3 tm2_checking.py --tests-only
    python3 tm2_checking.py --story
    python3 tm2_checking.py --no-show --save figures/tm2_checking.png
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import tm2_rule as R  # noqa: E402

ROOTS = (0, 2, 4, 6, 8, 10)  # settings in units of 30 degrees


def q2(beta, n):
    lam = R.eigen(beta)
    return (lam[2] / lam[0]) ** n


def g_closed(v, a, beta, n):
    """E6.1, v and a in units of 60 degrees."""
    lam = R.eigen(beta)
    return (lam[0] ** n + 2.0 * lam[2] ** n * math.cos(2.0 * math.pi * (v - a) / 3.0)) / 3.0


def delta_closed(beta, n):
    q = q2(beta, n)
    return q * (2.0 + q) / (3.0 * (1.0 + 2.0 * q * q))


def rates(beta):
    """(past shift, pass-rate contrast, correlation) decay rates per wing step (E6.3)."""
    lam = R.eigen(beta)
    return math.log(lam[0] / lam[2]), 2.0 * math.log(lam[0] / lam[2]), 2.0 * math.log(lam[0] / lam[1])


def profile(beta, n, a=0, b=0):
    """TV(t), t = 0..n (D6.3); a, b settings in units of 30 degrees (roots)."""
    T = R.transfer(beta)
    pa = R.boundary(a, None).sum(axis=0)
    pb = R.boundary(b, None).sum(axis=0)
    from_bob = np.roll(np.linalg.matrix_power(T, n) @ pb, -3)  # weight of Bob's wing, indexed by the origin v
    out = []
    for t in range(n + 1):
        fwd = from_bob @ np.linalg.matrix_power(T, t)            # sum_v from_bob[v] T^t[v, x]
        bwd = np.linalg.matrix_power(T, n - t) @ pa
        p = fwd * bwd
        p = p / p.sum()
        out.append(0.5 * float(np.abs(p - 1.0 / 6.0).sum()))
    return np.array(out)


def entropy2(p):
    p = np.asarray(p, dtype=float)
    p = p[p > 0]
    return float(-(p * np.log2(p)).sum())


def bits_on_past(beta, n, a=0, b=0):
    return math.log2(6.0) - entropy2(R.TwoWing(beta, n).past(a, b))


def bits_of_check(beta, n, a=0, b=0):
    z_free = 6.0 * R.eigen(beta)[0] ** (2 * n)
    return math.log2(z_free / R.TwoWing(beta, n).Z(a, b))


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e61_only_the_axis_reaches_the_past():
    for beta, n in ((0.7, 1), (1.0, 3), (2.0, 5), (3.0, 2)):
        tw = R.TwoWing(beta, n)
        for a in ROOTS:
            for b in ROOTS:
                gA, gB = tw.past_factors(a, b)
                for v in range(6):
                    assert math.isclose(gA[v], g_closed(v, a // 2, beta, n), rel_tol=1e-12)
                    assert math.isclose(gB[v], g_closed((v + 3) % 6, b // 2, beta, n), rel_tol=1e-12)
                # a and a + 180 degrees are the same axis: the past cannot tell them apart
                assert np.allclose(tw.past(a, b), tw.past((a + 6) % 12, b), atol=1e-15)
    # the odd mode is present at the ends: the outcome is not uniform given the far outcome
    assert abs(R.TwoWing(2.0, 2).E(0, 0)) > 0.9


def test_e62_closed_form_of_the_past_shift():
    for beta in (0.3, 1.0, 2.0, 3.0, 5.0):
        for n in (1, 2, 3, 5, 8, 13):
            tw = R.TwoWing(beta, n)
            d = {(a, b): float(np.abs(tw.past(a, b) - 1.0 / 6.0).max()) for a in ROOTS for b in ROOTS}
            assert math.isclose(max(d.values()), delta_closed(beta, n), rel_tol=1e-10, abs_tol=1e-15), (beta, n)
            assert math.isclose(d[(0, 0)], max(d.values()), rel_tol=1e-9, abs_tol=1e-15), "largest for the same axis"
            assert math.isclose(0.5 * float(np.abs(tw.past(0, 0) - 1 / 6).sum()), 2.0 * d[(0, 0)], rel_tol=1e-10, abs_tol=1e-15)
    assert math.isclose(delta_closed(2.0, 1), 0.31505, abs_tol=1e-5) and math.isclose(delta_closed(2.0, 8), 0.02959, abs_tol=1e-5)
    for beta in (1.0, 2.0, 3.0):
        n = 120
        assert math.isclose(delta_closed(beta, n) / q2(beta, n), 2.0 / 3.0, rel_tol=1e-3)
    # depends on q only: two (beta, n) with the same q give the same delta
    b1 = 2.0
    target = q2(b1, 2)
    lo, hi = 0.01, 10.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if q2(mid, 1) < target else (lo, mid)
    assert math.isclose(R.TwoWing(0.5 * (lo + hi), 1).delta(), R.TwoWing(b1, 2).delta(), rel_tol=1e-8)


def test_e63_three_rates_and_their_order():
    for beta in np.geomspace(0.05, 8.0, 60):
        past, contrast, corr = rates(beta)
        lam = R.eigen(beta)
        assert lam[1] ** 2 > lam[0] * lam[2] and contrast > past > corr > 0.0, beta
    for beta in (1.0, 2.0, 3.0):
        past, contrast, corr = rates(beta)
        ns = np.arange(4, 120)
        d = [R.TwoWing(beta, int(n)).delta() for n in ns]
        c = [R.TwoWing(beta, int(n)).contrast() - 1.0 for n in ns]
        e = [abs(R.TwoWing(beta, int(n)).E(0, 0)) for n in ns]
        for series, rate in ((d, past), (c, contrast), (e, corr)):
            series = np.array(series)
            keep = (series > 1e-9) & (series < 0.05)   # asymptotic and above rounding
            assert keep.sum() >= 3, (beta, rate)
            slope = -np.polyfit(ns[keep], np.log(series[keep]), 1)[0]
            assert math.isclose(slope, rate, rel_tol=0.02), (beta, slope, rate)
    assert [round(x, 3) for x in rates(2.0)] == [0.392, 0.783, 0.239]


def test_e64_the_ends_are_always_checked():
    for beta in (1.0, 2.0, 3.0):
        for n in (1, 2, 4, 8):
            for a, b in ((0, 0), (0, 2), (0, 4)):
                p = profile(beta, n, a, b)
                assert math.isclose(p[-1], 2.0 / 3.0, rel_tol=1e-12) and np.all(np.diff(p) > 0), (beta, n, a, b, p)
            tw = R.TwoWing(beta, n)
            assert math.isclose(profile(beta, n)[0], 0.5 * float(np.abs(tw.past(0, 0) - 1 / 6).sum()), rel_tol=1e-10)
    assert [round(profile(2.0, n)[0], 4) for n in (1, 2, 4, 8)] == [0.6301, 0.528, 0.2828, 0.0592]


def test_e65_bits():
    for beta in (1.0, 2.0):
        for n in (1, 3, 8):
            tw = R.TwoWing(beta, n)
            z = [tw.Z(a, b) for a in (0, 2, 4) for b in (0, 2, 4)]
            assert math.isclose(sum(z), 6.0 * R.eigen(beta)[0] ** (2 * n), rel_tol=1e-12)
            avg = np.mean([bits_of_check(beta, n, a, b) for a in (0, 2, 4) for b in (0, 2, 4)])
            assert avg >= math.log2(9.0) - 1e-12
    assert abs(np.mean([bits_of_check(2.0, 20, a, b) for a in (0, 2, 4) for b in (0, 2, 4)]) - math.log2(9.0)) < 1e-6
    got = [round(bits_on_past(2.0, n), 3) for n in (1, 4, 8)]
    assert got == [1.322, 0.24, 0.011], got
    for n in (1, 2, 4, 8):
        assert bits_on_past(2.0, n) < bits_of_check(2.0, n), "the past gets a part of what the check removes"


def test_e66_long_space_correlated_ends_unrevised_past():
    tw = R.TwoWing(3.0, 16)
    assert math.isclose(tw.delta(), 0.066, abs_tol=1e-3) and math.isclose(abs(tw.E(0, 0)), 0.442, abs_tol=1e-3) and math.isclose(tw.chsh(), 1.117, abs_tol=1e-3)
    for beta in (1.0, 2.0, 3.0):
        for n in range(1, 25):
            t = R.TwoWing(beta, n)
            if t.delta() < 0.14:
                assert t.chsh() < 2.0, (beta, n)


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
    print("decay rates per wing step (past shift, pass-rate contrast, correlation) and the lengths 1/rate")
    for beta in (1.0, 2.0, 3.0):
        r = rates(beta)
        print("  beta = %.0f: %.3f %.3f %.3f   lengths %.2f %.2f %.2f" % ((beta,) + r + tuple(1.0 / x for x in r)))
    print("beta = 2, same axis at both ends:  n | delta  |E(0)|  CHSH  contrast-1 | bits on past / bits of check | profile from origin to end")
    for n in (1, 2, 4, 8, 16):
        tw = R.TwoWing(2.0, n)
        print("  %2d | %.4f %.4f %.3f %.2e | %.3f / %.3f | %s" % (n, tw.delta(), abs(tw.E(0, 0)), tw.chsh(), tw.contrast() - 1.0, bits_on_past(2.0, n), bits_of_check(2.0, n),
                                                              " ".join("%.3f" % x for x in profile(2.0, n))))
    t = R.TwoWing(3.0, 16)
    print("long space, stiff: beta = 3, n = 16: delta %.3f, |E(0)| %.3f, CHSH %.3f" % (t.delta(), abs(t.E(0, 0)), t.chsh()))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(18, 4.2))
    for n in (1, 2, 4, 8):
        p = profile(2.0, n)
        ax[0].plot(np.arange(n + 1) / n, p, "o-", label="n = %d" % n)
    ax[0].set_xlabel("position along the wing (0 = shared past, 1 = checked end)")
    ax[0].set_ylabel("how far the check moves the letter (TV)")
    ax[0].set_title("where the check lands (beta = 2)")
    ax[0].legend(fontsize=8)
    qs = np.linspace(0, 1, 200)
    ax[1].plot(qs, qs * (2 + qs) / (3 * (1 + 2 * qs * qs)), "k", label="q(2+q) / 3(1+2q^2)")
    for beta, m in ((1.0, "o"), (2.0, "s"), (3.0, "^")):
        ns = range(1, 21)
        ax[1].plot([q2(beta, n) for n in ns], [R.TwoWing(beta, n).delta() for n in ns], m, label="beta = %g, n = 1..20" % beta)
    ax[1].set_xlabel("q = (lambda_2 / lambda_0)^n")
    ax[1].set_ylabel("past shift delta")
    ax[1].set_title("one curve for all stiffness and length")
    ax[1].legend(fontsize=7)
    ns = np.arange(1, 25)
    for lab, fn in (("pass-rate contrast - 1", lambda t: t.contrast() - 1.0), ("past shift delta", lambda t: t.delta()), ("|E(0)| end to end", lambda t: abs(t.E(0, 0)))):
        ax[2].semilogy(ns, [fn(R.TwoWing(2.0, int(n))) for n in ns], "o-", ms=3, label=lab)
    ax[2].axhline(0.14, color="k", lw=0.6, ls="--")
    ax[2].text(14, 0.16, "CHSH > 2 needs delta above this", fontsize=7)
    ax[2].set_xlabel("wing length n (beta = 2)")
    ax[2].set_title("what goes first as space grows")
    ax[2].legend(fontsize=7)
    for beta in (1.0, 2.0, 3.0):
        ax[3].plot(ns, [bits_on_past(beta, int(n)) for n in ns], "o-", ms=3, label="beta = %g" % beta)
    ax[3].axhline(math.log2(9), color="k", lw=0.6, ls="--")
    ax[3].text(10, math.log2(9) - 0.25, "bits a check removes (about log2 9)", fontsize=7)
    ax[3].set_xlabel("wing length n")
    ax[3].set_ylabel("bits that land on the shared past")
    ax[3].set_title("past checking vs potential checking")
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
