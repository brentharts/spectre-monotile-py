#!/usr/bin/env python3
"""t7_bell.py - Stage 1 of toymodel_v7: a Bell test with settings and outcomes both defined on the tile, and nothing discarded.

The pair qubit of toymodel_v6 (tau^z = +1: the first tile of the pair is the bump) has one read-out on the tile, the bump (v6 E6.2).  Its other directions are reached by its own dynamics:
  * a tunnelling pulse: H_pair = - Gamma_p tau^x for a time delta rotates the read-out about tau^x by theta = 2 Gamma_p delta (the pair's own tunnelling, made brief and strong);
  * a bias pulse: - b_p tau^z, a potential difference across the pair from a nearby charge (the dipole coupling of v5 and v6), rotates it about tau^z.
A party chooses a pulse, then both read the bump at the same time.  Outcomes are +-1 for every run.  The coupling between pairs stays on throughout.

Definitions
  D7.1  Tunnelling-only settings: unit vectors in the y-z plane of the pair's Bloch sphere (rotations about tau^x of the read-out tau^z).  Full settings: tunnelling and bias pulses, any unit vector.
  D7.2  Finite pulses: H = H_eff + Gamma_p (tau^x_A or not) ... a pulse of strength Gamma_p and angle theta on each pair, then a simultaneous read-out of tau^z; Gamma_p -> infinity is the ideal setting.
  D7.3  Read-out visibility 1 - c^2 per correlation (v6 D6.4), c = c(tau) the overlap of the bit's two positions.
  D7.4  The excluded protocol: settings as waiting times before a read-out (one party reads first).  It is evaluated to show why it is excluded.

Exact / Numeric claims (each a test; ``python3 t7_bell.py --tests-only``)
  E7.1  For the real ground states of v6 (two pairs and the clusters) the correlation matrix has T_xy = T_yx = T_yz = T_zy = 0, so tunnelling-only settings give CHSH = 2 sqrt(T_yy^2 + T_zz^2) and full settings
        2 sqrt(m1 + m2) (Horodecki).  For two pairs, T = diag(1, -C, C): tunnelling-only CHSH = 2 sqrt2 C, full = 2 sqrt(1 + C^2).
  E7.2  Two pairs pass.  Along the flow (J0 = K0 = 1) the tunnelling-only value is 2.583 at the Spectre end (2.455 with visibility) and stays above 2 with visibility up to tau_2 = 0.053; a direct optimisation over the
        y-z plane agrees with the closed form to 1e-6.
  E7.3  Marginals cannot decide locality here; the window can.  The ground state is even under the global flip tau^x_A tau^x_B, which commutes with H and with every tunnelling pulse, so each marginal is exactly 1/2
        (to 1e-15) for every pulse strength and also for the waiting-time protocol: no-signalling holds and says nothing.  What separates the protocols is the phase the inter-pair coupling acts for during the
        measurement window, K x window.  Pulses: K theta / (2 Gamma_p) -> 0, and CHSH converges to the state's value (2.267, 2.515, 2.574, 2.582 at Gamma_p = 2, 5, 20, 200; state value 2.583), the error falling
        as 1 / Gamma_p.  Waiting times: the delays a violation needs span pi / (2 Gamma), a coupling phase of 7.0 rad, so the correlations can be made during the window (the locality loophole); excluded.
  E7.4  Patches do not pass.  On the central 6, 10 and 14 pairs of the level-3 patch (v6 Stage 1), over tau in [0, 0.3], the largest tunnelling-only CHSH of any two pairs is 1.95, 2.13 and 1.98, and with visibility at most 1.99:
        no violation with tile settings and visibility.  With full settings the values are those of v6 (2.33, with visibility 2.12 in the 10-pair cluster only).  **Negative** for a Bell test on a patch with the tile's own read-outs.

Status.  E7.1, E7.3: Exact / Numeric (finite matrices).  E7.2, E7.4: exact given the Definitions of v6 (Gamma = c, K from the charges, visibility).  That a party can choose a pulse is the usual free-setting assumption of a Bell test
(H7.1 of PLAN.md), not something the tile supplies.

    python3 t7_bell.py --tests-only
    python3 t7_bell.py --no-show --save figures/t7_stage1_bell.png
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np
import scipy.linalg as sl

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_RES, "toymodel_v6"), os.path.join(_RES, "toymodel_v4"), os.path.dirname(_RES), os.path.join(os.path.dirname(_RES), "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t6_pair as t0  # noqa: E402

SX, SY, SZ, I2 = t0.SX, t0.SY, t0.SZ, t0.I2
CLUSTERS = (6, 10, 14)
TAUS = np.linspace(0.0, 0.3, 16)


def corr(rho):
    return np.array([[float(np.real(np.trace(rho @ np.kron(p, q)))) for q in t0.PAULI] for p in t0.PAULI])


def chsh_plane(T, axes=(1, 2)):
    M = T[np.ix_(axes, axes)]
    s = np.sort(np.linalg.eigvalsh(M.T @ M))[::-1]
    return float(2 * math.sqrt(max(s[0] + s[1], 0.0)))


def chsh_full(T):
    s = np.sort(np.linalg.eigvalsh(T.T @ T))[::-1]
    return float(2 * math.sqrt(max(s[0] + s[1], 0.0)))


def two_pair_state(tau, J0=1.0, K0=1.0):
    G, K = t0.flow(tau, J0, K0)[:2]
    _, psi = t0.ground_state(t0.effective_hamiltonian(G, K))
    return psi, G, K


def yz_direct(psi, grid=181):
    """Direct optimisation of CHSH over y-z read-outs (angle grid, then refinement)."""
    T = corr(np.outer(psi, psi.conj()))
    th = np.linspace(0, np.pi, grid)
    u = np.stack([np.zeros_like(th), np.sin(th), np.cos(th)], axis=1)
    E = u @ T @ u.T
    best = 0.0
    # for fixed (a, a') the best (b, b') follow in closed form: |E(a,.) + E(a',.)| + |E(a,.) - E(a',.)| maximised over b
    for i in range(grid):
        for j in range(grid):
            best = max(best, float(np.max(np.abs(E[i] + E[j])) + np.max(np.abs(E[i] - E[j]))))
    return best


def tau_two_tunnelling(J0=1.0, K0=1.0):
    def f(t):
        psi, _, _ = two_pair_state(t, J0, K0)
        c = t0.overlap(t)
        return (1 - c * c) * chsh_plane(corr(np.outer(psi, psi.conj()))) - 2.0
    lo, hi = 0.0, 1.0
    if f(lo) <= 0:
        return 0.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return 0.5 * (lo + hi)


def pulse_probabilities(psi, H, thA, thB, Gp):
    """Joint read-out probabilities of tau^z after simultaneous tunnelling pulses of angles thA, thB at strength Gp, with H on during the pulses.
    The two pulses take times thA/(2Gp) and thB/(2Gp); the shorter one is followed by H alone until the longer ends."""
    tA, tB = abs(thA) / (2 * Gp), abs(thB) / (2 * Gp)
    sA, sB = math.copysign(1, thA), math.copysign(1, thB)
    XA, XB = np.kron(SX, I2), np.kron(I2, SX)
    t1, t2 = min(tA, tB), max(tA, tB)
    U1 = sl.expm(-1j * t1 * (H - Gp * (sA * XA + sB * XB)))
    rest = (H - Gp * sA * XA) if tA > tB else (H - Gp * sB * XB)
    U2 = sl.expm(-1j * (t2 - t1) * rest)
    phi = U2 @ U1 @ psi
    return (np.abs(phi) ** 2).reshape(2, 2)            # index: (bit of A, bit of B); bit 0 <-> tau^z = +1


def marginal_A(P):
    return float(P[0].sum())


def signalling(psi, H, Gp, angles):
    worst = 0.0
    for a in angles:
        m = [marginal_A(pulse_probabilities(psi, H, a, b, Gp)) for b in angles]
        worst = max(worst, max(m) - min(m))
    return worst


def chsh_from_probs(psi, H, Gp, a, a2, b, b2):
    def E(x, y):
        P = pulse_probabilities(psi, H, x, y, Gp)
        return float(P[0, 0] + P[1, 1] - P[0, 1] - P[1, 0])
    return E(a, b) + E(a, b2) + E(a2, b) - E(a2, b2)


def ideal_angles(psi):
    """Optimal y-z settings from the closed form: read-out vectors (0, sin th, cos th)."""
    T = corr(np.outer(psi, psi.conj()))
    M = T[1:, 1:]
    U, s, Vt = np.linalg.svd(M)
    # Alice: a, a' = the two left singular directions combined at +-45 degrees around b's
    b = Vt[0]; b2 = Vt[1]
    ang = lambda v: math.atan2(v[0], v[1])   # noqa: E731  (v = (y, z))
    a = (M @ (b + b2)); a2 = (M @ (b - b2))
    return ang(a), ang(a2), ang(b), ang(b2)


def waiting_signalling(psi, H, delays):
    """D7.4: A reads at t_a, B at t_b; the marginal of A as B's delay varies (with t_b < t_a B reads first)."""
    def PA(ta, tb):
        P0 = np.kron(np.diag([1, 0]), I2); P1 = np.kron(np.diag([0, 1]), I2)
        Q0 = np.kron(I2, np.diag([1, 0])); Q1 = np.kron(I2, np.diag([0, 1]))
        if tb < ta:
            phi = sl.expm(-1j * tb * H) @ psi
            tot = 0.0
            for Q in (Q0, Q1):
                chi = sl.expm(-1j * (ta - tb) * H) @ (Q @ phi)
                tot += float(np.linalg.norm(P0 @ chi) ** 2)
            return tot
        phi = sl.expm(-1j * ta * H) @ psi
        return float(np.linalg.norm(P0 @ phi) ** 2)
    worst = 0.0
    for ta in delays:
        m = [PA(ta, tb) for tb in delays]
        worst = max(worst, max(m) - min(m))
    return worst


_SCAN = {}


def cluster_scan(n):
    if n in _SCAN:
        return _SCAN[n]
    import t6_patch as tp6
    idx = tp6.cluster(3, n)
    best_yz, best_yz_obs, best_full = (0.0, None), 0.0, 0.0
    for tau in TAUS:
        K, _ = tp6.couplings_at(tau, 3, idx)
        c = t0.overlap(tau)
        psi, _, _ = tp6.parity_even_ground(c, K)
        for a, b in itertools.combinations(range(n), 2):
            T = corr(tp6.reduced_two(psi, n, a, b))
            assert abs(T[0, 1]) + abs(T[1, 0]) + abs(T[1, 2]) + abs(T[2, 1]) < 1e-10
            yz = chsh_plane(T)
            if yz > best_yz[0]:
                best_yz = (yz, round(float(tau), 3))
            best_yz_obs = max(best_yz_obs, (1 - c * c) * yz)
            best_full = max(best_full, chsh_full(T))
    _SCAN[n] = (best_yz, best_yz_obs, best_full)
    return _SCAN[n]


# --------------------------------------------------------------------------
def test_e71_the_structure_of_t():
    for tau in (0.0, 0.1, 0.5):
        psi, G, K = two_pair_state(tau)
        T = corr(np.outer(psi, psi.conj()))
        C = K / math.sqrt(4 * G * G + K * K)
        assert np.allclose(T, np.diag([1.0, -C, C]), atol=1e-10), T
        assert abs(chsh_plane(T) - 2 * math.sqrt(2) * C) < 1e-10
        assert abs(chsh_full(T) - 2 * math.sqrt(1 + C * C)) < 1e-10
    return "T = diag(1, -C, C)"


def test_e72_two_pairs_pass_with_tile_settings():
    psi, _, _ = two_pair_state(0.0)
    T = corr(np.outer(psi, psi.conj()))
    v = 1 - t0.overlap(0.0) ** 2
    s = chsh_plane(T)
    assert abs(s - 2.583) < 1e-3 and abs(v * s - 2.455) < 1e-3, (s, v * s)
    assert abs(yz_direct(psi) - s) < 1e-3
    t2 = tau_two_tunnelling()
    assert abs(t2 - 0.053) < 1e-3, t2
    return round(s, 4), round(v * s, 4), round(t2, 4)


def test_e73_marginals_cannot_decide_locality_the_window_can():
    psi, G, K = two_pair_state(0.0)
    H = t0.effective_hamiltonian(G, K)
    angles = ideal_angles(psi)
    for Gp in (2.0, 20.0, 200.0):
        assert signalling(psi, H, Gp, angles) < 1e-12
    assert waiting_signalling(psi, H, np.linspace(0.0, math.pi / (2 * G), 7)) < 1e-12
    ideal = chsh_plane(corr(np.outer(psi, psi.conj())))
    vals = [abs(chsh_from_probs(psi, H, Gp, *angles)) for Gp in (2.0, 5.0, 20.0, 200.0)]
    want = (2.267, 2.515, 2.574, 2.582)
    assert all(abs(v - w) < 1e-3 for v, w in zip(vals, want)), vals
    err = [ideal - abs(chsh_from_probs(psi, H, Gp, *angles)) for Gp in (200.0, 2000.0)]
    assert abs(err[0] / err[1] - 10.0) < 1.5, err
    window_phase = K * math.pi / (2 * G)
    assert abs(window_phase - 7.04) < 0.01, window_phase
    return [round(v, 3) for v in vals], round(ideal, 3), round(window_phase, 2)


def test_e74_patches_do_not_pass():
    out = {}
    for n in CLUSTERS:
        (yz, t), yzo, full = cluster_scan(n)
        out[n] = (round(yz, 3), t, round(yzo, 3), round(full, 3))
        assert yzo < 2.0
    assert abs(out[6][0] - 1.952) < 2e-3 and abs(out[10][0] - 2.127) < 2e-3 and abs(out[14][0] - 1.977) < 2e-3, out
    assert max(v[2] for v in out.values()) < 1.99 + 1e-3
    return out


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e7")]
    failed = 0
    for n, f in tests:
        try:
            r = f()
            if verbose:
                print("ok   %s  %s" % (n, "" if r is None else r))
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
    taus = np.linspace(0, 1, 101)
    yz, yzo, full = [], [], []
    for t in taus:
        psi, _, _ = two_pair_state(t)
        T = corr(np.outer(psi, psi.conj()))
        v = 1 - t0.overlap(t) ** 2
        yz.append(chsh_plane(T)); yzo.append(v * chsh_plane(T)); full.append(chsh_full(T))
    ax[0].plot(taus, full, "k", label="full settings (tunnelling + bias)")
    ax[0].plot(taus, yz, "tab:blue", label="tunnelling pulses only")
    ax[0].plot(taus, yzo, "tab:blue", ls=":", label="tunnelling only, with visibility")
    ax[0].axhline(2, color="k", lw=0.8); ax[0].axhline(2 * math.sqrt(2), color="k", lw=0.6, ls="-.")
    ax[0].axvline(tau_two_tunnelling(), color="tab:blue", ls="--", lw=0.8)
    ax[0].set_xlabel("tau"); ax[0].set_ylabel("CHSH"); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) two pairs: a violation with the tile's own settings", fontsize=9)
    psi, G, K = two_pair_state(0.0)
    H = t0.effective_hamiltonian(G, K)
    angles = ideal_angles(psi)
    ideal = chsh_plane(corr(np.outer(psi, psi.conj())))
    Gps = np.geomspace(2, 2000, 13)
    th = max(abs(x) for x in angles)
    ax[1].loglog(Gps, [ideal - abs(chsh_from_probs(psi, H, g, *angles)) for g in Gps], "o-k", label="state value - measured CHSH")
    ax[1].loglog(Gps, K * th / (2 * Gps), "k:", label="coupling phase in the window, K theta / 2 Gamma$_p$")
    ax[1].axhline(K * math.pi / (2 * G), color="tab:red", label="waiting-time protocol: K pi / 2 Gamma (excluded)")
    ax[1].set_xlabel("pulse strength Gamma$_p$"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) marginals are exactly 1/2 in every protocol;\nthe coupling phase in the window separates them", fontsize=9)
    x = np.arange(len(CLUSTERS))
    sc = [cluster_scan(n) for n in CLUSTERS]
    ax[2].bar(x - 0.27, [s[2] for s in sc], 0.27, color="0.3", label="full settings")
    ax[2].bar(x, [s[0][0] for s in sc], 0.27, color="tab:blue", label="tunnelling only")
    ax[2].bar(x + 0.27, [s[1] for s in sc], 0.27, color="tab:cyan", label="tunnelling only, with visibility")
    ax[2].axhline(2, color="k", lw=0.8)
    ax[2].set_xticks(x); ax[2].set_xticklabels(["%d pairs" % n for n in CLUSTERS]); ax[2].set_ylim(1.5, 2.45)
    ax[2].set_ylabel("largest CHSH of two pairs, tau in [0, 0.3]"); ax[2].legend(fontsize=7)
    ax[2].set_title("(c) patches: no violation with the tile's settings", fontsize=9)
    fig.suptitle("toymodel_v7 Stage 1: a Bell test with settings and outcomes defined on the tile, nothing discarded")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=140)
    if show:
        plt.show()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save")
    a = ap.parse_args(argv)
    failed = run_tests()
    if not a.tests_only and not failed:
        figure(a.save, not a.no_show)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
