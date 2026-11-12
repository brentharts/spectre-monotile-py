#!/usr/bin/env python3
"""t5_rule.py - Stage 1 of toymodel_v5: the history-weight rule of toymodel_v2 (Stage 4) on the six points of the 14th vertex.

toymodel_v2 gave the letters the six roots of an A2 and the step cost N = squared planar distance (0, 1, 3, 4).  Here the letters are the six points around P8 of the tile (a = 1,
curvature k): V = (0, +-1), (+-k, +-1/2), sorted by angle so that V[i + 3] = -V[i] (Stage 0), and the step cost is the squared distance of the points, exp(-beta |V_i - V_j|^2).  The two-wing
history, the hard-pin read-outs and the normalisation per pair of settings are exactly those of tm2_rule.py (D4.1 to D4.5, hard pins); a setting m is the letter V[m] (outcome +1) and its
antipode V[m+3] (outcome -1).  The cubic six (+-e_x, +-e_y, +-e_z) is the control.  Nothing here is physics.

Definitions
  D5.6  TwoWingV(V, beta, n): P(alpha, beta' | m, m') of the two-wing ensemble with letters V (antipodal order), wings of n steps, hard pins; Bob's wing starts at -v.
  D5.7  CHSH(V, beta, n): the largest |E + E + E - E| over the six settings; E the correlation of the table.  beta_2(V), beta_22(V): the stiffness at which it passes 2 and 2 sqrt 2.

Exact claims (each a test; ``python3 t5_rule.py --tests-only``)
  E5.6  Regression: for V = the hexagon of k = sqrt3/2 the tables, the CHSH value and the settings agree with toymodel_v2/tm2_rule.py to 1e-12 (n = 1, 2).  The enumeration of both wings
        (n = 1) equals the transfer-matrix table to 1e-12 for every k tried.
  E5.7  No-signalling is exact for every k and for the cubic six (marginals 1/2 to 1e-14), because the alphabet is antipodal: the weight is invariant under V -> -V.  Control: a bias exp(eta u.V)
        on the wing letters breaks the inversion and the marginal of Alice then depends on Bob's setting.
  E5.8  The geometry sets the strength.  At beta = 1.255 and n = 1 CHSH rises strictly with k (1.670 at k = 0, 1.943 at 1/2, 2.828 at sqrt3/2, 3.058 at 1, 3.236 at 1.3).  The stiffness at which
        CHSH passes 2 falls from 1.308 at k = 1/2 to 0.841 at k = sqrt3/2 and 0.718 at k = 1; the one for 2 sqrt 2 is 1.255 at k = sqrt3/2 (v2's value), 1.094 at k = 1, and does not exist (below 4) for k = 1/2 up to beta = 8.
  E5.9  Along the flow, k = k0 (1 - tau): CHSH falls strictly with tau (beta = 1.255, k0 = sqrt3/2: 2.828 at the Spectre end, below 2 after tau = 0.4, 1.670 at the Hat) and stays above the value of two letters
        sharing a point at k = 0.  The Bell-violating window ends at a flow time tau_2(beta, k0), computed here.
  E5.10 The cubic six is local: CHSH <= 2 for every beta tried and tends to 2 from below (1.992 at beta = 2, 2.000 at beta >= 4), while the planar six reach 4 (3.9993 at beta = 8, k = 1).  The shadow of the cube
        changes the correlations, not only the picture.

Status.  Exact (transfer matrix, enumeration) for the tables; the thresholds are numerical roots.  That the step cost is the squared distance of the tile points, and that the wing weight is the paper's first term, are
Definitions of toymodel_v2; the 2 sqrt 2 of v2 is a point the family passes through, not a bound (v2 E4.10).

    python3 t5_rule.py --tests-only
    python3 t5_rule.py --no-show --save figures/t5_stage1_rule.png
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, os.path.join(os.path.dirname(_HERE), "toymodel_v2"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t5_alphabet as ta  # noqa: E402

K_HEX = ta.K_HEX
TSIRELSON = 2.0 * math.sqrt(2.0)


def alphabet(k):
    """The six points (a = 1) sorted by angle, V[i + 3] = -V[i]."""
    pts = ta.points(k)
    v = np.array(list(pts.values()))
    v = v[np.argsort(np.arctan2(v[:, 1], v[:, 0]), kind="stable")]
    if not np.allclose(v[3:], -v[:3]):
        raise ValueError("the six points are not in antipodal order at k = %r" % k)
    return v


def cubic():
    return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float)


class TwoWingV:
    """Tables of D5.6 for wings of n steps (hard pins).  ``eta`` and ``u`` add the control bias exp(eta u.V) on the wing letters."""

    def __init__(self, V, beta=1.0, n=1, eta=0.0, u=None):
        V = np.asarray(V, dtype=float)
        if V.shape[0] != 6 or not np.allclose(V[3:], -V[:3]):
            raise ValueError("V must have six rows in antipodal order (V[i + 3] = -V[i])")
        if not (math.isfinite(beta) and beta >= 0) or int(n) != n or n < 0:
            raise ValueError("beta >= 0 and an integer n >= 0 are needed")
        self.V, self.beta, self.n = V, float(beta), int(n)
        D = ((V[:, None, :] - V[None, :, :]) ** 2).sum(-1)
        T = np.exp(-beta * D)
        if eta:
            T = T * np.exp(eta * (V @ np.asarray(u, dtype=float)))[None, :]
        self.G = np.linalg.matrix_power(T, self.n)

    def weights(self, a, b):
        WA = np.array([self.G[:, a], self.G[:, (a + 3) % 6]])                       # [alpha, v]
        WB = np.roll(np.array([self.G[:, b], self.G[:, (b + 3) % 6]]), -3, axis=1)   # Bob's wing starts at -v
        return WA @ WB.T

    def table(self, a, b):
        W = self.weights(a, b)
        return W / W.sum()

    def Z(self, a, b):
        return float(self.weights(a, b).sum())

    def E(self, a, b):
        P = self.table(a, b)
        return float(P[0, 0] + P[1, 1] - P[0, 1] - P[1, 0])

    def E_table(self):
        return np.array([[self.E(a, b) for b in range(6)] for a in range(6)])

    def chsh(self):
        E = self.E_table()
        best = 0.0
        for a, a2 in itertools.permutations(range(6), 2):
            for b, b2 in itertools.permutations(range(6), 2):
                best = max(best, abs(E[a, b] + E[a, b2] + E[a2, b] - E[a2, b2]))
        return float(best)

    def marginal_deviation(self):
        dev = 0.0
        for a in range(6):
            for b in range(6):
                P = self.table(a, b)
                dev = max(dev, abs(P[0].sum() - 0.5), abs(P[:, 0].sum() - 0.5))
        return dev

    def brute(self, a, b):
        """The table by enumeration of the origin and both wings."""
        D = ((self.V[:, None, :] - self.V[None, :, :]) ** 2).sum(-1)
        W = np.zeros((2, 2))
        for v in range(6):
            for wa in itertools.product(range(6), repeat=self.n):
                sa = (v,) + wa
                wA = math.exp(-self.beta * sum(D[sa[t], sa[t - 1]] for t in range(1, self.n + 1)))
                for wb in itertools.product(range(6), repeat=self.n):
                    sb = ((v + 3) % 6,) + wb
                    wB = math.exp(-self.beta * sum(D[sb[t], sb[t - 1]] for t in range(1, self.n + 1)))
                    for al, end_a in ((0, a), (1, (a + 3) % 6)):
                        for be, end_b in ((0, b), (1, (b + 3) % 6)):
                            if sa[-1] == end_a and sb[-1] == end_b:
                                W[al, be] += wA * wB
        return W / W.sum()


def chsh_of(V, beta, n=1):
    return TwoWingV(V, beta, n).chsh()


def crossing(V, target, n=1, lo=0.0, hi=8.0):
    """Smallest stiffness beta (bisection) at which CHSH exceeds target, or None if it does not within [lo, hi]."""
    if chsh_of(V, hi, n) <= target:
        return None
    for _ in range(45):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if chsh_of(V, mid, n) > target else (mid, hi)
    return 0.5 * (lo + hi)


def window_end(beta, k0, target=2.0, n=1):
    """Flow time tau at which CHSH (k = k0 (1 - tau)) falls to target, by bisection; None if it never exceeds it or never falls to it."""
    f = lambda t: chsh_of(alphabet(k0 * (1.0 - t)), beta, n) - target  # noqa: E731
    if f(0.0) <= 0 or f(0.999) >= 0:
        return None
    lo, hi = 0.0, 0.999
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e56_regression_against_toymodel_v2_and_enumeration():
    import tm2_rule as v2

    V = alphabet(K_HEX)
    for beta in (0.5, 1.0, 1.255, 2.0):
        for n in (1, 2):
            mine, ref = TwoWingV(V, beta, n), v2.TwoWing(beta=beta, n=n, mu=None)
            for a in range(6):
                for b in range(6):
                    assert np.abs(mine.table(a, b) - ref.table(2 * a, 2 * b)).max() < 1e-12, (beta, n, a, b)
            assert abs(mine.chsh() - ref.chsh()) < 1e-12
    for k in (0.0, 0.5, K_HEX, 1.0):
        w = TwoWingV(alphabet(k), 1.0, 1)
        for a, b in ((0, 0), (0, 1), (1, 4), (2, 5)):
            assert np.abs(w.table(a, b) - w.brute(a, b)).max() < 1e-12, (k, a, b)


def test_e57_no_signalling_is_exact_and_the_bias_breaks_it():
    for V in [alphabet(k) for k in (0.0, 0.3, 0.5, K_HEX, 1.0, 1.3)] + [cubic()]:
        for beta in (0.3, 1.0, 2.5):
            assert TwoWingV(V, beta, 1).marginal_deviation() < 1e-14
            assert TwoWingV(V, beta, 2).marginal_deviation() < 1e-14
    u = np.array([1.0, 0.0])
    assert TwoWingV(alphabet(K_HEX), 1.0, 1, eta=0.3, u=u).marginal_deviation() > 1e-3


def test_e58_the_geometry_sets_the_strength():
    ks = np.linspace(0.0, 1.3, 27)
    c = np.array([chsh_of(alphabet(k), 1.255) for k in ks])
    assert np.all(np.diff(c) > 0)
    assert abs(c[0] - 1.6697) < 1e-3 and abs(chsh_of(alphabet(K_HEX), 1.255) - 2.8281) < 2e-4
    assert abs(crossing(alphabet(K_HEX), 2.0) - 0.8410) < 1e-3 and abs(crossing(alphabet(K_HEX), TSIRELSON) - 1.2552) < 1e-3
    assert abs(crossing(alphabet(0.5), 2.0) - 1.3083) < 1e-3 and crossing(alphabet(0.5), TSIRELSON) is None
    assert abs(crossing(alphabet(1.0), 2.0) - 0.7180) < 1e-3 and abs(crossing(alphabet(1.0), TSIRELSON) - 1.0937) < 1e-3


def test_e59_the_violating_window_closes_along_the_flow():
    c = np.array([chsh_of(alphabet(K_HEX * (1.0 - t)), 1.255) for t in np.linspace(0, 1, 21)])
    assert np.all(np.diff(c) < 0) and c[0] > TSIRELSON - 1e-3 and c[-1] < 1.7
    t2 = window_end(1.255, K_HEX)
    assert t2 is not None and 0.35 < t2 < 0.45, t2
    assert window_end(1.255, 0.5) is None                      # k0 = 1/2 never exceeds 2 at beta = 1.255
    assert window_end(2.0, 0.8) is not None


def test_e510_the_cubic_six_is_local():
    for beta in (0.3, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0):
        assert chsh_of(cubic(), beta) <= 2.0 + 1e-12
    assert abs(chsh_of(cubic(), 4.0) - 2.0) < 1e-3 and chsh_of(cubic(), 2.0) < 2.0
    assert chsh_of(alphabet(1.0), 8.0) > 3.99


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
        print("  %-72s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def figure():
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(2, 3, figsize=(15, 8.5))
    ks = np.linspace(0.0, 1.3, 53)
    ax = axs[0, 0]
    for beta, col in ((0.5, "C0"), (1.0, "C1"), (1.255, "C2"), (2.0, "C3")):
        ax.plot(ks, [chsh_of(alphabet(k), beta) for k in ks], color=col, label="beta=%.3g" % beta)
    ax.axhline(2, color="k", lw=0.6); ax.axhline(TSIRELSON, color="k", lw=0.6, ls="--"); ax.axvline(K_HEX, color="crimson", ls=":", lw=0.8)
    ax.set_xlabel("k"); ax.set_ylabel("CHSH"); ax.legend(fontsize=7)
    ax.set_title("(a) the curvature sets the strength: CHSH of the two-wing rule on the six points", fontsize=8)

    ax = axs[0, 1]
    kk = np.linspace(0.3, 1.3, 26)
    b2 = [crossing(alphabet(k), 2.0) for k in kk]
    b22 = [crossing(alphabet(k), TSIRELSON) for k in kk]
    ax.plot(kk, [np.nan if v is None else v for v in b2], label="CHSH = 2")
    ax.plot(kk, [np.nan if v is None else v for v in b22], label="CHSH = 2 sqrt2")
    ax.axvline(K_HEX, color="crimson", ls=":", lw=0.8); ax.set_ylim(0, 8)
    ax.set_xlabel("k"); ax.set_ylabel("stiffness beta needed"); ax.legend(fontsize=7)
    ax.set_title("(b) stiffness at which the table passes 2 and 2 sqrt2 (gap: not within beta <= 8)", fontsize=8)

    ax = axs[0, 2]
    taus = np.linspace(0, 1, 41)
    for k0, beta, col in ((K_HEX, 1.255, "C2"), (K_HEX, 2.0, "C3"), (0.8, 2.0, "C4"), (0.5, 2.0, "C5")):
        ax.plot(taus, [chsh_of(alphabet(k0 * (1 - t)), beta) for t in taus], color=col, label="k0=%.2f, beta=%.3g" % (k0, beta))
    ax.axhline(2, color="k", lw=0.6); ax.axhline(TSIRELSON, color="k", lw=0.6, ls="--")
    ax.set_xlabel("tau (0 Spectre, 1 Hat)"); ax.set_ylabel("CHSH"); ax.legend(fontsize=7)
    ax.set_title("(c) along the flow the violating window closes", fontsize=8)

    ax = axs[1, 0]
    bs = np.linspace(0.1, 6.0, 40)
    ax.plot(bs, [chsh_of(alphabet(K_HEX), b) for b in bs], label="planar six, k = sqrt3/2 (A2)")
    ax.plot(bs, [chsh_of(alphabet(1.0), b) for b in bs], label="planar six, k = 1")
    ax.plot(bs, [chsh_of(cubic(), b) for b in bs], label="cubic six (3D)")
    ax.axhline(2, color="k", lw=0.6); ax.axhline(TSIRELSON, color="k", lw=0.6, ls="--"); ax.axhline(4, color="k", lw=0.6, ls=":")
    ax.set_xlabel("beta"); ax.set_ylabel("CHSH"); ax.legend(fontsize=7)
    ax.set_title("(d) the cube's own six are local (-> 2); its shadow reaches 4", fontsize=8)

    ax = axs[1, 1]
    th = np.arange(6) * 60
    for k, col in ((0.5, "C0"), (K_HEX, "crimson"), (1.0, "C2")):
        w = TwoWingV(alphabet(k), 1.255)
        ax.plot(th, [w.E(0, b) for b in range(6)], "o-", color=col, label="k=%.3f" % k)
    ax.set_xlabel("Bob's setting (index x 60 deg, letters sorted by angle; letter 0 for Alice)"); ax.set_ylabel("E(0, b)"); ax.legend(fontsize=7)
    ax.set_title("(e) correlation of letter 0 with the six settings (beta = 1.255)", fontsize=8)

    ax = axs[1, 2]
    devs = [TwoWingV(alphabet(k), 1.0).marginal_deviation() for k in ks]
    ax.semilogy(ks, np.maximum(devs, 1e-17), label="antipodal alphabet")
    ax.semilogy(ks, [TwoWingV(alphabet(k), 1.0, 1, eta=0.3, u=np.array([1.0, 0.0])).marginal_deviation() for k in ks], label="with a bias on the wings (control)")
    ax.set_xlabel("k"); ax.set_ylabel("largest marginal deviation"); ax.legend(fontsize=7)
    ax.set_title("(f) no-signalling is exact for every k; a bias breaks it", fontsize=8)
    fig.tight_layout()
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", metavar="PNG")
    args = ap.parse_args(argv)
    print("t5_rule self tests:")
    if not run_tests():
        return 1
    if args.tests_only:
        return 0
    import matplotlib

    if args.no_show or args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = figure()
    if args.save:
        fig.savefig(args.save, dpi=140)
        print("saved", args.save)
    elif not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
