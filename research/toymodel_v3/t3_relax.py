#!/usr/bin/env python3
"""t3_relax.py - Stage 5 of research/toymodel_v3/PLAN.md: how fast does a measurement act on a history?  Local relaxation dynamics on the history sum.

Stage 1 of toymodel_v2 (Result res:influence of the paper): in the exact ensemble a pinned node influences the far end of a closed timeline as much as its neighbours.
That is a statement about the equilibrium ensemble.  Here the ensemble is sampled by LOCAL moves, the pin is switched on at time 0, and the time each node takes to
respond is measured.

Definitions
  D5.1  Ensemble: open timelines of n hexagonal letters, weight exp(-beta sum N(s(t) - s(t-1))), and either closed (total flux 0) or free (no constraint); lam = 0.
        A soft pin: weight exp(mu [s(m) = k]) at the middle node m (mu = 3, k = 0).  The exact marginals with and without the pin come from the transfer matrix.
  D5.2  Moves: Metropolis on a window of 3 consecutive letters, proposing a uniformly random word with the SAME sum (closed ensemble; this keeps the flux and the
        moves are ergodic on closed timelines, checked by connectivity for n = 5, 6, 7, where windows of 2 are not) or any word (free ensemble).  One sweep = n - 2
        window updates in random order.  The chains start in exact equilibrium without the pin (forward filtering, backward sampling).
  D5.3  Response of node t at sweep tau: the shift of P(s(t) = k_t) from its unpinned value, divided by the exact final shift, with k_t the letter of largest exact
        shift at t (a signed, unbiased estimate; averaged over the two nodes at distance d).  t_half(d): first sweep at which it reaches 1/2 (log interpolation).

Claims (each a test; ``python3 t3_relax.py --tests-only``, about 2.5 minutes; Monte Carlo, so Numeric, with the exact marginals as the reference)
  E5.1  The dynamics leave the right ensemble invariant: chains started in exact equilibrium stay there (unpinned), and with the pin they end at the exact pinned
        marginals (response within 0.15 of 1 at every node with final shift >= 0.04 after 1024 sweeps, n = 13).  Windows of 3 are ergodic on closed timelines
        of 5, 6 and 7 steps; windows of 2 split them into 2, 3 and 2 classes.
  E5.2  Free ensemble (no constraint): t_half grows with the distance, 10, 14, 18, 18, 24 sweeps for d = 1..5 (n = 13), about 3 sweeps per site; at n = 21 the same
        slope continues to d = 10 (story): a front at about 0.3 sites per sweep, a 'ripple', and the influence it carries decays geometrically.
  E5.3  Closed ensemble: the far nodes do not wait for a front.  At n = 13 the nodes at d = 4, 5, 6 reach 1/2 at 91, 85, 72 sweeps, against 54 at d = 1; at n = 17 those at
        d = 6, 7, 8 at 181, 176, 138, against 92 at d = 1.  The far sweep is about the same for every far node, and it grows with the size of the timeline:
        far mean 82 (n = 13) and 165 (n = 17), a ratio 2.0 against (17/13)^2 = 1.71 (the test allows 30%), consistent with a diffusive time ~ n^2.  The far response is a collective mode of the constraint
        (the redistribution of the flux over the whole timeline), with a time set by the size of the history, not by the distance.

What it says.  Both readings of the proposal's 'ripple' fail for the closed history.  In the ensemble the influence is global (v2); in time it is not a front either:
near the pin it builds up locally, far from it on one global time scale.  A front exists only when the closure constraint is dropped, and then the influence it
carries decays geometrically.  With closure the history behaves as one object.  Not shown: the scaling for larger n; a dynamics other than local Metropolis.

    python3 t3_relax.py --tests-only
    python3 t3_relax.py --story          (n = 21 runs, about 4 minutes)
"""
import collections
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
import core  # noqa: E402
import tm2_history_sum as th  # noqa: E402

HEX = np.array(core.HEX)
NORMS = np.array([[th.norm2(HEX[a] - HEX[b]) for b in range(6)] for a in range(6)])
WORDS = np.array(list(itertools.product(range(6), repeat=3)))
_KEY = HEX[WORDS].sum(axis=1)
_KEY = _KEY[:, 0] * 100 + _KEY[:, 1]
_GROUPS = collections.defaultdict(list)
for _i, _k in enumerate(_KEY):
    _GROUPS[int(_k)].append(_i)
_GID = {k: j for j, k in enumerate(_GROUPS)}
TABLE = np.full((len(_GROUPS), max(len(v) for v in _GROUPS.values())), -1)
GSIZE = np.zeros(len(_GROUPS), dtype=int)
for _k, _v in _GROUPS.items():
    TABLE[_GID[_k], :len(_v)] = _v
    GSIZE[_GID[_k]] = len(_v)
WORD_G = np.array([_GID[int(k)] for k in _KEY])


# -- exact marginals ---------------------------------------------------------------------------------------------------------
def _alphas(n, beta, wvec, m):
    hs = th.HistorySum(n, beta, 0.0, 0)
    G, off = hs.G, hs.off

    def W(a, t):
        return a * wvec[:, None, None] if t == m else a

    alpha = [None] * n
    a0 = np.zeros((6, G, G))
    for k in range(6):
        a0[k, off + HEX[k][0], off + HEX[k][1]] = 1.0
    alpha[0] = W(a0, 0)
    for t in range(1, n):
        new = np.zeros((6, G, G))
        for k2 in range(6):
            acc = sum(hs._T[k1, k2] * alpha[t - 1][k1] for k1 in range(6))
            new[k2] = hs._shift(acc, k2) * hs._pen
        alpha[t] = W(new, t)
    return hs, alpha, W


def exact_closed(n, beta, m=None, k=0, mu=0.0):
    wvec = np.ones(6)
    if m is not None:
        wvec[k] = math.exp(mu)
    hs, alpha, W = _alphas(n, beta, wvec, m)
    G = hs.G
    beta_ = [None] * n
    beta_[n - 1] = W(np.broadcast_to(hs._fin, (6, G, G)).copy(), n - 1)
    for t in range(n - 2, -1, -1):
        nb = np.zeros((6, G, G))
        for k1 in range(6):
            acc = 0.0
            for k2 in range(6):
                acc = acc + hs._T[k1, k2] * hs._unshift(beta_[t + 1][k2] * hs._pen, k2)
            nb[k1] = acc
        beta_[t] = W(nb, t)
    P = np.array([(alpha[t] * beta_[t]).sum(axis=(1, 2)) for t in range(n)])
    if m is not None:
        P[m] = P[m] / wvec   # the pin weight sits in both alpha[m] and beta[m]
    return P / P.sum(axis=1, keepdims=True)


def exact_free(n, beta, m=None, k=0, mu=0.0):
    T = np.exp(-beta * NORMS)
    w = [np.ones(6) for _ in range(n)]
    if m is not None:
        w[m] = np.ones(6)
        w[m][k] = math.exp(mu)
    a = [w[0]]
    for t in range(1, n):
        a.append((a[-1] @ T) * w[t])
    b = [None] * n
    b[n - 1] = np.ones(6)
    for t in range(n - 2, -1, -1):
        b[t] = T @ (b[t + 1] * w[t + 1])
    P = np.array([a[t] * b[t] for t in range(n)])
    return P / P.sum(axis=1, keepdims=True)


def sample_closed(n, beta, C, rng):
    """C exact samples of the unpinned closed ensemble (forward filtering, backward sampling)."""
    hs, alpha, _ = _alphas(n, beta, np.ones(6), None)
    G = hs.G
    last = (alpha[n - 1] * hs._fin[None]).ravel()
    last = last / last.sum()
    k, p, q = np.unravel_index(rng.choice(len(last), size=C, p=last), (6, G, G))
    S = np.zeros((C, n), dtype=int)
    S[:, n - 1] = k
    for t in range(n - 2, -1, -1):
        p, q = p - HEX[k, 0], q - HEX[k, 1]
        w = alpha[t][:, p, q].T * hs._T[:, k].T
        w = w / w.sum(axis=1, keepdims=True)
        k = (np.cumsum(w, axis=1) < rng.random(C)[:, None]).sum(axis=1)
        S[:, t] = k
    assert np.all(HEX[S].sum(axis=1) == 0)
    return S


def sample_free(n, beta, C, rng):
    T = np.exp(-beta * NORMS)
    Pt = T / T.sum(axis=1, keepdims=True)
    S = np.zeros((C, n), dtype=int)
    S[:, 0] = rng.integers(0, 6, C)
    for t in range(1, n):
        S[:, t] = (np.cumsum(Pt[S[:, t - 1]], axis=1) < rng.random(C)[:, None]).sum(axis=1)
    return S


# -- dynamics ------------------------------------------------------------------------------------------------------------------------
def _local_energy(S, i, beta, m, k, mu):
    n = S.shape[1]
    E = np.zeros(S.shape[0])
    for a in range(max(i - 1, 0), min(i + 3, n - 1)):
        E += beta * NORMS[S[:, a], S[:, a + 1]]
    if m is not None and i <= m <= i + 2:
        E -= mu * (S[:, m] == k)
    return E


def sweep(S, beta, rng, constrained=True, m=None, k=0, mu=0.0):
    n = S.shape[1]
    for i in rng.permutation(n - 2):
        if constrained:
            code = S[:, i] * 36 + S[:, i + 1] * 6 + S[:, i + 2]
            g = WORD_G[code]
            new = WORDS[TABLE[g, rng.integers(0, 1 << 30, size=len(S)) % GSIZE[g]]]
        else:
            new = WORDS[rng.integers(0, 216, size=len(S))]
        E0 = _local_energy(S, i, beta, m, k, mu)
        old = S[:, i:i + 3].copy()
        S[:, i:i + 3] = new
        E1 = _local_energy(S, i, beta, m, k, mu)
        reject = rng.random(len(S)) >= np.exp(-(E1 - E0))
        S[reject, i:i + 3] = old[reject]
    return S


def marginals(S):
    return np.array([np.bincount(S[:, t], minlength=6) / len(S) for t in range(S.shape[1])])


def window_classes(n, w):
    """Number of classes of closed timelines of n steps connected by same-sum window moves of width w."""
    T = [tuple(t) for t in core.closed_timelines("hex", n).astype(int)]
    idx = {t: i for i, t in enumerate(T)}
    bysum = collections.defaultdict(list)
    for wd in itertools.product(range(6), repeat=w):
        bysum[tuple(HEX[list(wd)].sum(axis=0))].append(wd)
    parent = list(range(len(T)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for t in T:
        for i in range(n - w + 1):
            for wd in bysum[tuple(HEX[list(t[i:i + w])].sum(axis=0))]:
                a, b = find(idx[t]), find(idx[t[:i] + wd + t[i + w:]])
                if a != b:
                    parent[a] = b
    return len({find(i) for i in range(len(T))})


def experiment(n, constrained, C, times, beta=1.0, mu=3.0, seed=0):
    """(m, final exact shift per node, response array [len(times), n])."""
    m = n // 2
    rng = np.random.default_rng(seed)
    if constrained:
        S, P0, Pf = sample_closed(n, beta, C, rng), exact_closed(n, beta), exact_closed(n, beta, m, 0, mu)
    else:
        S, P0, Pf = sample_free(n, beta, C, rng), exact_free(n, beta), exact_free(n, beta, m, 0, mu)
    D = Pf - P0
    ks = np.argmax(np.abs(D), axis=1)
    sg = np.sign(D[np.arange(n), ks])
    fin = np.abs(D[np.arange(n), ks])
    out, tt = [], 0
    for T in times:
        while tt < T:
            sweep(S, beta, rng, constrained, m, 0, mu)
            tt += 1
        Pm = marginals(S)
        out.append(sg * (Pm[np.arange(n), ks] - P0[np.arange(n), ks]) / fin)
    return m, fin, np.array(out)


def t_half(times, vals):
    for i in range(1, len(vals)):
        if vals[i] >= 0.5 > vals[i - 1]:
            l0, l1 = math.log(max(times[i - 1], 0.5)), math.log(times[i])
            f = (0.5 - vals[i - 1]) / (vals[i] - vals[i - 1])
            return math.exp(l0 + f * (l1 - l0))
    return float("nan")


def profile(n, constrained, C, times, seed=0):
    m, fin, out = experiment(n, constrained, C, times, seed=seed)
    rows = []
    for d in range(1, n - m):
        v = 0.5 * (out[:, m + d] + out[:, m - d])
        rows.append((d, float(fin[m + d]), t_half(times, v), float(v[-1])))
    return rows


TIMES = (0, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e51_the_dynamics_keep_the_ensemble_and_are_ergodic():
    assert [window_classes(n, 3) for n in (5, 6, 7)] == [1, 1, 1]
    assert [window_classes(n, 2) for n in (5, 6, 7)] == [2, 3, 2]
    rng = np.random.default_rng(2)
    S = sample_closed(13, 1.0, 20000, rng)
    P0 = exact_closed(13, 1.0)
    for _ in range(50):
        sweep(S, 1.0, rng, True)
    assert np.all(HEX[S].sum(axis=1) == 0)
    assert 0.5 * np.abs(marginals(S) - P0).sum(axis=1).max() < 0.03, "unpinned chains stay in equilibrium"
    rows = profile(13, True, 12000, TIMES, seed=4)
    for d, fin, th_, last in rows:
        if fin >= 0.04:
            assert abs(last - 1.0) < 0.15, (d, fin, last)


def test_e52_without_the_constraint_there_is_a_front():
    rows = profile(13, False, 12000, TIMES, seed=5)
    th_ = [r[2] for r in rows[:5]]
    assert all(np.isfinite(th_)) and th_[4] > 1.8 * th_[0] and th_[2] > th_[0], th_
    slope = np.polyfit(np.arange(1, 6), th_, 1)[0]
    assert 2.0 < slope < 6.0, (th_, slope)


def test_e53_with_the_constraint_the_far_end_answers_on_a_global_time():
    far = {}
    for n in (13, 17):
        rows = profile(n, True, 16000, TIMES, seed=6)
        sel = [r[2] for r in rows if r[0] >= n // 2 - 2 and r[1] >= 0.04]
        assert len(sel) >= 2 and all(np.isfinite(sel)), (n, rows)
        assert max(sel) / min(sel) < 2.0, (n, sel)
        far[n] = float(np.mean(sel))
    ratio = far[17] / far[13]
    assert abs(ratio / (17.0 / 13.0) ** 2 - 1.0) < 0.3, (far, ratio)


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
    times = TIMES + (2048,)
    for constrained in (False, True):
        rows = profile(21, constrained, 20000, times, seed=0)
        print("%s, n = 21: d, final shift, t_half (sweeps), response at 2048" % ("closed" if constrained else "free"))
        for d, fin, th_, last in rows:
            print("  %2d  %.3f  %7.1f  %.2f" % (d, fin, th_, last))
    for n in (13, 17):
        rows = profile(n, True, 16000, TIMES, seed=6)
        sel = [r[2] for r in rows if r[0] >= n // 2 - 2 and r[1] >= 0.04]
        print("closed, n = %d: far t_half %s, mean %.1f" % (n, ["%.0f" % x for x in sel], np.mean(sel)))


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
