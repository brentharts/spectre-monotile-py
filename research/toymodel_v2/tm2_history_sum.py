#!/usr/bin/env python3
"""tm2_history_sum.py - Stage 1 of PLAN.md: the exact history-sum on the hexagonal alphabet, with the flux constraint and the seam constraint.

Setting.  A history H is a timeline of n nodes s(0..n-1), each one of the six hexagonal roots (core.HEX, planar coordinates (p, q) in the basis (1, w)).
Planar norm^2 N(p, q) = p^2 + p q + q^2 (the E8 norm is twice it).

Definitions
  D1.1  Action (the paper's A[gamma], literal form, target 0):  A(H) = sum_{t=1}^{n-1} [ N(s(t) - s(t-1)) + lam N(Phi(t)) ],  Phi(t) = s(0) + ... + s(t).
  D1.2  Weight W(H) = exp(-beta A(H)).  beta = 0, lam = 0 gives the uniform (counting) measure.
  D1.3  Flux constraint ('admissible'):  N(Phi(n-1)) <= d2.   d2 = 0 means closed, d2 = 1 allows the one unit of flux the odd lengths force, d2 huge removes it.
  D1.4  A pin is a constraint s(m) = k.  It is part of the history, not an update rule (P6 of PLAN.md).
  D1.5  Exact marginals P(s(t) = k | admissible, pin) by a transfer matrix over the state (s(t), Phi(t)); cost O(n^3 * 36 * ...) , exact up to rounding for any n <= 40.
  D1.6  Influence of a pin at m on node t: total-variation distance between the marginal with the pin and the marginal without it (same constraint).
  D1.7  Seam constraint (from Stage 0): on a flat pair the two signs are antiparallel, every other tile is frozen.  Area ledger Q = sum of the signs (times a h).

Exact claims (each a test; ``python3 tm2_history_sum.py --tests-only``)
  E1.1  The transfer-matrix marginals equal brute-force enumeration (n <= 7) to 1e-12, with and without a pin and a flux constraint.
  E1.2  The action is reversal symmetric on every closed timeline (n = 6, all 2040) and not on open ones; with d2 = 0 the marginals with a pin at the centre
        of an odd timeline are mirror symmetric about the pin.
  E1.3  The pinned node stays pinned (P = 1); the marginals are covariant under the sixfold rotation of the alphabet (pin letter k -> k + 1 shifts every marginal).
  E1.4  **Influence does not decay along the timeline once the flux constraint is on.**  Without it (d2 huge) the influence at distance 1..6 falls strictly
        (n = 13: 0.436, 0.322, 0.229, 0.161, 0.113, 0.079).  With closure (d2 = 0, n = 21) it is not monotone and is larger at distance 8 (0.199) than at distance 3
        (0.069).  A pinned node acts through the global constraint on the whole timeline, not as a ripple fading with distance.   [Negative for the ripple picture]
  E1.5  Repair after one forced step (the ledger's numbers): hexagonal n = 5: 1 further edit in 7560 of 9000 shocks and 2 in 1440; n = 6: 57960 of 61200 and 3240.
  E1.6  The v1 greedy contract, with its sign error corrected, on the hexagonal alphabet (n = 13, pin at 6, 4000 seeded runs) reaches the admissible set in
        about 85% of runs; its single-node marginals are within total variation 0.12 of the exact uniform-over-admissible ones, but in about 13% of runs it
        collapses to one repeated letter, a history of exact probability 0 (a constant timeline is not admissible).  Its joint structure is not typical.
  E1.7  The flux constraint is blind to the bit: opening the flat vertex of a tile leaves its outline closed for every h, and moves the two edges at the flat vertex
        off the alphabet (their planar norm^2 is no longer 1).  The seam constraint implies Q = 0 on every flat pair and is strictly stronger than 'total Q = 0':
        on the 6 flat pairs of the level-2 patch, of 4096 sign assignments of the 12 tiles, 64 close all seams, 924 have Q = 0, and the 64 are a subset of the 924.
        At level 3 (82 pairs) the counts are 2^82 against C(164, 82).

Hypothesis (not tested here): that the seam constraint is the compatibility rule of the substrate and the flux constraint of the paper's action is the wrong
quantity for the bit; a combined model needs the bit to be a letter flip AND a seam opening, which Stage 1 does not construct.

    python3 tm2_history_sum.py --tests-only
    python3 tm2_history_sum.py --story
    python3 tm2_history_sum.py --no-show --save figures/tm2_stage1_influence.png
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import core  # noqa: E402

HEX = np.array(core.HEX)
BIG = 10 ** 9


def norm2(v):
    v = np.asarray(v)
    return v[..., 0] ** 2 + v[..., 0] * v[..., 1] + v[..., 1] ** 2


DIFF = np.array([[int(norm2(HEX[a] - HEX[b])) for b in range(6)] for a in range(6)], float)


def check_params(n, beta, lam, d2):
    if int(n) != n or not 1 <= n <= 40:
        raise ValueError("n must be an integer in [1, 40]")
    for name, v in (("beta", beta), ("lam", lam)):
        if not (np.isfinite(v) and v >= 0):
            raise ValueError("%s must be finite and >= 0" % name)
    if not (d2 >= 0):
        raise ValueError("d2 must be >= 0")


def action(seq, lam=0.5):
    """D1.1 for a sequence of alphabet indices."""
    S = HEX[np.asarray(seq)]
    Phi = np.cumsum(S, axis=0)
    return float(norm2(S[1:] - S[:-1]).sum() + lam * norm2(Phi[1:]).sum())


class HistorySum:
    """Exact marginals of W(H) = exp(-beta A(H)) restricted to N(flux) <= d2 and to a pin.  D1.5."""

    def __init__(self, n=13, beta=1.0, lam=0.5, d2=1):
        check_params(n, beta, lam, d2)
        self.n, self.beta, self.lam, self.d2 = int(n), float(beta), float(lam), d2
        G = 2 * self.n + 1
        self.G, self.off = G, self.n
        P = np.arange(-self.n, self.n + 1)
        PP, QQ = np.meshgrid(P, P, indexing="ij")
        self._coord = np.stack([PP, QQ], -1)
        self._pen = np.exp(-self.beta * self.lam * norm2(self._coord))
        self._T = np.exp(-self.beta * DIFF)
        self._fin = (norm2(self._coord) <= d2).astype(float)

    def _shift(self, arr, k):
        p, q = HEX[k]
        G = self.G
        out = np.zeros_like(arr)
        out[max(p, 0):G + min(p, 0), max(q, 0):G + min(q, 0)] = arr[max(-p, 0):G + min(-p, 0), max(-q, 0):G + min(-q, 0)]
        return out

    def _unshift(self, arr, k):
        p, q = HEX[k]
        G = self.G
        out = np.zeros_like(arr)
        out[max(-p, 0):G + min(-p, 0), max(-q, 0):G + min(-q, 0)] = arr[max(p, 0):G + min(p, 0), max(q, 0):G + min(q, 0)]
        return out

    def marginals(self, pin=None):
        """(P, Z): P[t, k] = Prob(s(t) = k), Z = total weight of the admissible histories (with the pin)."""
        n, G, off = self.n, self.G, self.off
        if pin is not None and not (0 <= pin[0] < n and 0 <= pin[1] < 6):
            raise ValueError("pin must be (node in [0, n), letter in [0, 6))")

        def pinned(a, t):
            if pin is not None and pin[0] == t:
                mask = np.zeros(6)
                mask[pin[1]] = 1.0
                return a * mask[:, None, None]
            return a

        alpha = [None] * n
        a0 = np.zeros((6, G, G))
        for k in range(6):
            a0[k, off + HEX[k][0], off + HEX[k][1]] = 1.0
        alpha[0] = pinned(a0, 0)
        for t in range(1, n):
            new = np.zeros((6, G, G))
            for k2 in range(6):
                acc = sum(self._T[k1, k2] * alpha[t - 1][k1] for k1 in range(6))
                new[k2] = self._shift(acc, k2) * self._pen
            alpha[t] = pinned(new, t)
        beta_ = [None] * n
        beta_[n - 1] = pinned(np.broadcast_to(self._fin, (6, G, G)).copy(), n - 1)
        for t in range(n - 2, -1, -1):
            nb = np.zeros((6, G, G))
            for k1 in range(6):
                acc = 0.0
                for k2 in range(6):
                    acc = acc + self._T[k1, k2] * self._unshift(beta_[t + 1][k2] * self._pen, k2)
                nb[k1] = acc
            beta_[t] = pinned(nb, t)
        Z = float((alpha[n - 1] * beta_[n - 1]).sum())
        if Z <= 0:
            raise ValueError("no admissible history")
        P = np.array([(alpha[t] * beta_[t]).sum(axis=(1, 2)) for t in range(n)]) / Z
        return P, Z

    def influence(self, m, letter=0):
        """D1.6: TV distance at every node between the marginal with the pin (m, letter) and without it."""
        free, _ = self.marginals()
        pinned_, _ = self.marginals((m, letter))
        return 0.5 * np.abs(pinned_ - free).sum(axis=1)


def brute_marginals(n, beta, lam, d2, pin=None):
    """Reference by enumeration of all 6^n histories (n <= 8)."""
    check_params(n, beta, lam, d2)
    T = core.all_timelines("hex", n).astype(int)
    S = HEX[T]
    Phi = np.cumsum(S, axis=1)
    A = norm2(S[:, 1:] - S[:, :-1]).sum(1) + lam * norm2(Phi[:, 1:]).sum(1)
    ok = norm2(Phi[:, -1]) <= d2
    if pin is not None:
        ok &= T[:, pin[0]] == pin[1]
    w = np.exp(-beta * A) * ok
    Z = w.sum()
    return np.array([[w[T[:, t] == k].sum() / Z for k in range(6)] for t in range(n)]), float(Z)


# ------------------------------------------------------------------------------------------------ repair (E1.5)

def repair_counts(n):
    """After one forced change of one step of a closed timeline: (shocks, with a 1-edit repair, needing 2).  Ledger numbers for the hexagonal set."""
    cl = core.closed_timelines("hex", n).astype(int)
    one = two = 0
    for t in cl:
        for pos in range(n):
            for x in range(6):
                if x == t[pos]:
                    continue
                s = t.copy()
                s[pos] = x
                f = HEX[s].sum(0)
                found = False
                for j in range(n):
                    if j == pos:
                        continue
                    for y in range(6):
                        if y != s[j] and not (f - HEX[s[j]] + HEX[y]).any():
                            found = True
                            break
                    if found:
                        break
                one += found
                two += not found
    return len(cl) * n * 5, one, two


# ------------------------------------------------------------------------------------------------ greedy (E1.6)

def greedy_contract(seq, shock, obs, stop=1, max_iter=30):
    """v1's AxisNetwork contract on the hexagonal alphabet with the sign error corrected: a node is rewritten to the letter minimising N(flux - old + new)."""
    s = [int(x) for x in seq]
    s[shock] = obs
    for it in range(max_iter):
        f = HEX[s].sum(0)
        if int(norm2(f)) <= stop:
            return s, True
        spread = it // 3 + 1
        for off in range(-spread, spread + 1):
            t = shock + off
            if 0 <= t < len(s) and t != shock:
                s[t] = min(range(6), key=lambda a: int(norm2(f - HEX[s[t]] + HEX[a])))
    return s, int(norm2(HEX[s].sum(0))) <= stop


def greedy_statistics(n=13, shock=6, runs=4000, seed=5):
    rng = np.random.default_rng(seed)
    marg = np.zeros((n, 6))
    ok = collapsed = 0
    for _ in range(runs):
        seq = rng.integers(0, 6, size=n)
        s, good = greedy_contract(seq, shock, 0)
        ok += good
        for t, k in enumerate(s):
            marg[t, k] += 1
        collapsed += len(set(s[:shock] + s[shock + 1:])) == 1
    marg /= runs
    exact, _ = HistorySum(n, 0.0, 0.0, 1).marginals((shock, 0))
    return ok / runs, collapsed / runs, 0.5 * np.abs(marg - exact).sum(axis=1)


# ------------------------------------------------------------------------------------------------ seam vs flux (E1.7)

def seam_versus_ledger(levels=2, h=0.2):
    """Brute force over the sign assignments of the tiles in flat pairs (needs Stage 0's tm2_lattice): (assignments, seam-closed, Q = 0, seam subset of Q = 0)."""
    import tm2_lattice as lat
    _, pairs = lat.flat_vertex_types(levels)
    tiles = [t for pr in pairs for t in pr]
    closed = qzero = 0
    subset = True
    for signs in itertools.product((1, -1), repeat=len(tiles)):
        n_broken = len(lat.broken_seams(levels, 1.0, {t: s * h for t, s in zip(tiles, signs)})[1])
        q0 = sum(signs) == 0
        closed += n_broken == 0
        qzero += q0
        if n_broken == 0 and not q0:
            subset = False
    return 2 ** len(tiles), closed, qzero, subset


def outline_closure_and_alphabet(levels=2, h=0.2):
    """For every tile of the patch opened by +h: (largest |sum of edge vectors|, number of tiles with an edge off the alphabet)."""
    import tm2_lattice as lat
    P, polys = lat.opened_polygons(levels, 1.0, {t: h for t in range(lat.patch(levels).n_tiles)})
    worst, off = 0.0, 0
    for p in polys:
        e = np.roll(p, -1, axis=0) - p
        worst = max(worst, float(np.abs(e.sum(0)).max()))
    P0, polys0 = lat.opened_polygons(levels, 1.0, {})
    for p0, p in zip(polys0, polys):
        e = np.roll(p, -1, axis=0) - p
        e0 = np.roll(p0, -1, axis=0) - p0
        off += int(np.any(np.abs(np.hypot(*e.T) - np.hypot(*e0.T)) > 1e-9))
    return worst, off


# ------------------------------------------------------------------------------------------------ tests

def test_e1_transfer_matrix_equals_brute_force():
    for (n, beta, lam, d2, pin) in ((6, 1.0, 0.5, 1, (2, 0)), (5, 0.7, 0.3, 0, None), (7, 0.4, 0.0, 4, (3, 2)), (4, 0.0, 0.0, 10 ** 6, (1, 5)), (7, 1.2, 0.2, 1, None)):
        m, Z = HistorySum(n, beta, lam, d2).marginals(pin)
        mb, Zb = brute_marginals(n, beta, lam, d2, pin)
        assert np.abs(m - mb).max() < 1e-12 and abs(Z - Zb) < 1e-12 * max(1.0, Zb), (n, beta, lam, d2, pin)
        assert np.allclose(m.sum(axis=1), 1.0)


def test_e2_action_is_reversal_symmetric_exactly_on_closed_timelines():
    cl = core.closed_timelines("hex", 6).astype(int)
    assert len(cl) == 2040
    assert max(abs(action(t, 0.7) - action(t[::-1], 0.7)) for t in cl) < 1e-9
    T = core.all_timelines("hex", 6).astype(int)
    rng = np.random.default_rng(0)
    idx = rng.choice(len(T), 3000, replace=False)
    opens = [t for t in T[idx] if (HEX[t].sum(0) != 0).any()]
    asym = np.mean([abs(action(t, 0.7) - action(t[::-1], 0.7)) > 1e-9 for t in opens])
    assert asym > 0.5, asym
    # with closure the marginals about a central pin are mirror symmetric
    n = 11
    hs = HistorySum(n, 0.8, 0.5, 0)
    m, _ = hs.marginals((5, 0))
    for d in range(1, 6):
        assert np.abs(m[5 - d] - m[5 + d]).max() < 1e-12


def test_e3_pin_and_rotation_covariance():
    hs = HistorySum(9, 0.9, 0.4, 1)
    m0, _ = hs.marginals((4, 0))
    assert np.allclose(m0[4], np.eye(6)[0])
    for k in range(1, 6):
        mk, _ = hs.marginals((4, k))
        assert np.abs(np.roll(m0, k, axis=1) - mk).max() < 1e-12, k
    # mutation: a pin on a different node is not equivalent
    mo, _ = hs.marginals((3, 0))
    assert np.abs(mo - m0).max() > 1e-3


def test_e4_influence_without_the_flux_constraint_decays_and_with_closure_does_not():
    iu = HistorySum(13, 1.0, 0.0, BIG).influence(6)
    tv = iu[6:]
    assert abs(tv[0] - 5.0 / 6.0) < 1e-12
    assert all(tv[d + 1] < tv[d] for d in range(len(tv) - 1)), tv
    assert abs(tv[1] - 0.4364) < 5e-4 and abs(tv[6] - 0.0792) < 5e-4
    ic = HistorySum(21, 1.0, 0.0, 0).influence(10)
    assert ic[10 + 8] > ic[10 + 3] + 0.1, (ic[13], ic[18])
    assert abs(ic[13] - 0.0689) < 5e-4 and abs(ic[18] - 0.1993) < 5e-4


def test_e5_repair_counts_equal_the_ledger():
    assert repair_counts(5) == (9000, 7560, 1440)
    assert repair_counts(6) == (61200, 57960, 3240)


def test_e6_greedy_contract_is_not_typical_of_the_exact_distribution():
    success, collapse, tv = greedy_statistics(13, 6, 4000, 5)
    assert 0.80 < success < 0.90, success
    assert tv.max() < 0.12, tv.max()
    assert collapse > 0.10, collapse
    # a constant timeline has flux 12 * root: not admissible, so the exact probability of collapse is 0
    assert core.flux([0] * 13, "hex") == (13, 0) and int(norm2(np.array([13, 0]))) > 1
    ex, _ = HistorySum(13, 0.0, 0.0, 1).marginals((6, 0))
    assert ex[7].max() < 0.2 and ex[3].max() < 0.2          # in the exact uniform measure no node concentrates on one letter


def test_e7_flux_is_blind_to_the_bit_and_the_seam_constraint_is_inside_the_area_ledger():
    worst, off = outline_closure_and_alphabet(2, 0.2)
    assert worst < 1e-12 and off == 71
    worst0, off0 = outline_closure_and_alphabet(2, 0.0)
    assert off0 == 0
    total, closed, qzero, subset = seam_versus_ledger(2)
    assert (total, closed, qzero, subset) == (4096, 64, 924, True)
    assert 924 == math.comb(12, 6) and 64 == 2 ** 6
    assert 2 ** 82 < math.comb(164, 82)


def test_validation():
    for args in ((0, 1, 0, 1), (41, 1, 0, 1), (5.5, 1, 0, 1), (5, -1, 0, 1), (5, 1, float("nan"), 1), (5, 1, 0, -1)):
        try:
            HistorySum(*args)
        except ValueError:
            continue
        raise AssertionError("accepted %r" % (args,))
    try:
        HistorySum(5).marginals((5, 0))
    except ValueError:
        pass
    else:
        raise AssertionError("pin out of range accepted")
    try:
        HistorySum(1, 1.0, 0.0, 0).marginals()
    except ValueError:
        pass
    else:
        raise AssertionError("an empty admissible set (one node, closed) must raise")


def run_tests(verbose=True):
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            if verbose:
                print("  %-90s PASS" % name)
        except AssertionError as e:
            failed += 1
            print("  %-90s FAIL %s" % (name, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def story():
    print("influence of a pin (TV distance) at distance d from the pin")
    for label, hs, m in (("no flux constraint, n = 13", HistorySum(13, 1.0, 0.0, BIG), 6), ("closed, n = 21", HistorySum(21, 1.0, 0.0, 0), 10),
                         ("closed, n = 21, lam = 0.5", HistorySum(21, 1.0, 0.5, 0), 10)):
        iu = hs.influence(m)
        print("  %-28s" % label, " ".join("%.3f" % iu[m + d] for d in range(0, min(9, hs.n - m))))
    print("repair counts n = 5:", repair_counts(5), " n = 6:", repair_counts(6))
    s, c, tv = greedy_statistics()
    print("greedy: success %.3f collapse %.3f max node TV %.3f" % (s, c, tv.max()))
    print("seam vs ledger, level 2 (assignments, seam-closed, Q = 0, subset):", seam_versus_ledger(2))
    print("outline closure / off-alphabet tiles under opening:", outline_closure_and_alphabet(2, 0.2))
    print("level 3: seam-closed 2^82 = %.3e, Q = 0: C(164, 82) = %.3e" % (2.0 ** 82, float(math.comb(164, 82))))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
    for label, hs, m, c in (("no flux constraint (n = 13)", HistorySum(13, 1.0, 0.0, BIG), 6, "#2a9d8f"), ("closed, n = 13", HistorySum(13, 1.0, 0.0, 0), 6, "#e76f51"),
                            ("closed, n = 21", HistorySum(21, 1.0, 0.0, 0), 10, "#264653"), ("closed, n = 21, lam = 0.5", HistorySum(21, 1.0, 0.5, 0), 10, "#e9c46a")):
        iu = hs.influence(m)
        d = np.arange(0, hs.n - m)
        ax[0].plot(d, iu[m:], "o-", color=c, label=label, ms=4)
    ax[0].set_xlabel("distance from the pin")
    ax[0].set_ylabel("influence (TV)")
    ax[0].set_title("A pin acts through the global constraint", fontsize=10)
    ax[0].legend(fontsize=8)
    s, c_, tv = greedy_statistics()
    ax[1].bar(range(13), tv, color="#2a9d8f")
    ax[1].set_xlabel("node")
    ax[1].set_ylabel("TV greedy vs exact")
    ax[1].set_title("v1 greedy (sign fixed): success %.0f%%, collapse %.0f%%" % (100 * s, 100 * c_), fontsize=10)
    ks = np.arange(1, 83)
    ax[2].semilogy(ks, [math.comb(2 * k, k) for k in ks], "-", color="#e76f51", label="total Q = 0: C(2k, k)")
    ax[2].semilogy(ks, [2.0 ** k for k in ks], "-", color="#2a9d8f", label="seam closed: 2^k")
    ax[2].set_xlabel("flat pairs k")
    ax[2].set_ylabel("allowed sign assignments")
    ax[2].set_title("Seam constraint inside the area ledger", fontsize=10)
    ax[2].legend(fontsize=8)
    fig.suptitle("Stage 1: exact history-sum on the hexagonal alphabet (tm2_history_sum.py)", fontsize=11)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
        print("wrote", save)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
