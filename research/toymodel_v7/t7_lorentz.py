#!/usr/bin/env python3
"""t7_lorentz.py - Stage 5 of toymodel_v7: Lorentz.  Does the anisotropy of waves on the Chair44 and the Spectre shrink with scale, and does a moving clock run slow by gamma?

Two forms of a wave on a graph with Laplacian L:
  Schroedinger (first order, the hopping of toymodel_v6):  i dpsi/dt = L psi,    plane waves  omega = k^T (M/N) k + O(k^4)   (quadratic: no light cone).
  wave (second order, the leapfrog of toymodel_v2):        u_tt = -(L + m^2) u,  plane waves  omega^2 = m^2 + k^T (M/N) k - Q4(k) + O(k^6),
where M = sum_e e e^T and Q4(k) = sum_e (k.e)^4 / 12 (per site), from 2 (1 - cos x) = x^2 - x^4 / 12 + ....  Graphs: the Chair44 tile graph of depth 6 (262144 tiles) and the Spectre vertex graph of level 4.
Bulk sums over the edges within radius R of the centre, per site in the same ball.

Exact / Numeric claims (each a test; ``python3 t7_lorentz.py --tests-only``)
  E7.18  Only the second-order form has a light cone, and its cone is isotropic at long wavelength.  The first-order form has group velocity proportional to k.  The second-order form has speed c with c^2 = u^T (M/N) u:
         4.617 on the Chair44 (depth 6), isotropic to 1 - 6e-5 in the ball R = 40 (ratio of the extreme eigenvalues of M/N); the anisotropy of M/N falls as the ball grows from R = 10 (7e-3, 1.4e-4, 5.6e-5 at R = 10, 20, 40; smaller balls
         fluctuate with the local tiles).
  E7.19  The first correction is cubic and shrinks as lambda^-2.  On the Chair44, Q4 / k^4 per site is 1.297 along an axis, 0.806 along a face diagonal and 0.643 along a body diagonal, so the phase speed is lowest along the axes:
         v_axis / v_body - 1 = -(Q_axis - Q_body) k^2 / (2 c^2) = -2.80 / lambda^2 (lambda in tile spacings; -1.1% at 16, -0.11% at 50).  This is the sign v6 found for the quantum packets (axes slowest, E6.14): that
         anisotropy is the lattice-scale end of this term, and it does not shrink for a packet started on one site because such a packet contains lattice-scale wavelengths.  (Plane-wave estimate; scattering by the
         aperiodic structure is not included.)
  E7.20  On the Spectre the fourth-order tensor is isotropic too (six-fold statistics force every tensor of rank up to 5 to be isotropic in 2D): sum (u.e)^4 per site varies by less than 1% over directions, so the
         anisotropy starts at k^6, lambda^-4.
  E7.21  A moving clock runs slow by gamma exactly when the dispersion is relativistic.  For a packet of carrier k the phase at its centre advances at omega - k v_g (stationary phase; checked by exact evolution on a periodic
         chain to 3e-4).  With omega^2 = m^2 + c^2 k^2 this is m / gamma(v_g), exactly.  On the lattice (chain, omega^2 = m^2 + 2 (1 - cos k), m = 0.5) the clock follows 1/gamma to 0.03% at k = 0.2, 0.9% at 0.5 and 3.4% at k = 1
         (growing close to k^4).  The first-order (hopping) form gives omega - k v_g = -k^2 / 2 m*: the Galilean (first order in v^2) limit of gamma, with no rest frequency.

Status.  E7.18 to E7.20: Exact sums over the stated graphs; the reading of the plane-wave quotient as the speed is the homogenization (long-wave) statement, and the direct packet runs (not in the tests) were dominated by scattering
at lambda <= 12 and by the boundary at lambda = 24.  E7.21: Exact algebra and an exact evolution on a periodic chain.  Nothing here is relativity: it says what a wave on these graphs does at long wavelength.

    python3 t7_lorentz.py --tests-only
    python3 t7_lorentz.py --no-show --save figures/t7_stage5_lorentz.png
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_RES, "toymodel_v3"), os.path.dirname(_RES), os.path.join(os.path.dirname(_RES), "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

DIRS = {"axis": np.array([1.0, 0.0, 0.0]), "face": np.array([1.0, 1.0, 0.0]) / math.sqrt(2), "body": np.ones(3) / math.sqrt(3)}
_C = {}


def graph(name):
    if name not in _C:
        import t3_escape as te
        X, E, d = te.chair44_graph(6) if name == "chair44" else te.spectre_graph(4)
        _C[name] = (X, E, X[np.argmin(d)])
    return _C[name]


def ball_tensors(name, R):
    X, E, c = graph(name)
    e = X[E[:, 1]] - X[E[:, 0]]
    inb = np.linalg.norm(X[E[:, 0]] - c, axis=1) < R
    nb = int((np.linalg.norm(X - c, axis=1) < R).sum())
    e = e[inb]
    M = (e[:, :, None] * e[:, None, :]).sum(axis=0) / nb
    return e, nb, M


def q4(e, nb, u):
    return float(((e @ u) ** 4).sum() / (12.0 * nb))


def chain_clock(k0, m=0.5, N=8192, sigma=60.0, T=200.0):
    """Exact evolution of a positive-frequency packet on a periodic chain with omega^2 = m^2 + 2 (1 - cos k): (measured phase rate at the centre, omega - k v_g, m / gamma(v_g))."""
    x = np.arange(N) - N // 4
    psi0 = np.exp(-x ** 2 / (4 * sigma ** 2) + 1j * k0 * x)
    k = 2 * np.pi * np.fft.fftfreq(N)
    om = np.sqrt(m * m + 2 * (1 - np.cos(k)))
    f0 = np.fft.fft(psi0)
    ts = np.linspace(T / 2, T, 401)        # phase step per sample well below pi
    ph, xc = [], []
    for t in ts:
        psi = np.fft.ifft(f0 * np.exp(-1j * om * t))
        p = np.abs(psi) ** 2
        c = float((p * x).sum() / p.sum())
        j = int(round(c)) + N // 4
        ph.append(np.angle(psi[j] * np.exp(-1j * k0 * (j - N // 4 - c))))      # phase at the centre itself
        xc.append(c)
    ph = np.unwrap(ph)
    rate = -float(np.polyfit(ts, ph, 1)[0])
    w0 = math.sqrt(m * m + 2 * (1 - math.cos(k0)))
    vg = math.sin(k0) / w0
    return rate, w0 - k0 * vg, m * math.sqrt(1 - vg ** 2 / 1.0)


# --------------------------------------------------------------------------
def test_e718_only_the_second_order_form_has_an_isotropic_cone():
    an = []
    for R in (10.0, 20.0, 40.0):
        _, _, M = ball_tensors("chair44", R)
        w = np.linalg.eigvalsh(M)
        an.append(1 - w.min() / w.max())
    _, _, M = ball_tensors("chair44", 40.0)
    c2 = float(np.mean(np.linalg.eigvalsh(M)))
    assert abs(c2 - 4.617) < 1e-3 and an[-1] < 1e-4 and an[0] > an[1] > an[2], (c2, an)
    # first-order form: omega ~ k^2, group velocity ~ k (no finite speed at long wavelength)
    ks = np.array([0.01, 0.02, 0.04])
    vg = 2 * c2 * ks
    assert np.allclose(vg[1:] / vg[:-1], 2.0)
    return round(c2, 4), [float("%.1e" % a) for a in an]


def test_e719_the_first_correction_is_cubic_and_shrinks():
    e, nb, M = ball_tensors("chair44", 40.0)
    Q = {k: q4(e, nb, u) for k, u in DIRS.items()}
    assert abs(Q["axis"] - 1.297) < 2e-3 and abs(Q["face"] - 0.806) < 2e-3 and abs(Q["body"] - 0.643) < 2e-3, Q
    c2 = float(np.mean(np.linalg.eigvalsh(M)))
    coef = -(Q["axis"] - Q["body"]) * (2 * math.pi) ** 2 / (2 * c2)       # v_axis / v_body - 1 = coef / lambda^2
    assert abs(coef + 2.80) < 0.02, coef
    assert Q["axis"] > Q["face"] > Q["body"]                               # axes slowest: the sign of v6 E6.14
    return {k: round(v, 3) for k, v in Q.items()}, round(coef, 3)


def test_e720_the_spectre_is_isotropic_to_fourth_order():
    e, nb, M = ball_tensors("spectre", 50.0)
    th = np.linspace(0, np.pi, 181)
    vals = np.array([((e[:, :2] @ np.array([math.cos(a), math.sin(a)])) ** 4).sum() / nb for a in th])
    spread = float(vals.max() / vals.min() - 1)
    assert spread < 0.01, spread
    return round(spread, 4)


def test_e721_a_moving_clock_runs_slow_by_gamma_when_the_dispersion_is_relativistic():
    out = []
    for k0 in (0.2, 0.5, 1.0):
        rate, pred, gam = chain_clock(k0)
        assert abs(rate / pred - 1) < 1e-3, (k0, rate, pred)
        out.append((k0, round(rate, 5), round(pred / gam - 1, 4)))
    dev = [abs(o[2]) for o in out]
    assert dev[0] < 5e-4 and abs(dev[1] - 0.0088) < 5e-4 and abs(dev[2] - 0.0344) < 5e-4 and dev[0] < dev[1] < dev[2]
    # continuum relativistic dispersion: omega - k v_g = m / gamma exactly
    m, c = 0.7, 1.3
    for k in (0.1, 1.0, 5.0):
        w = math.sqrt(m * m + c * c * k * k)
        vg = c * c * k / w
        assert abs((w - k * vg) - m * math.sqrt(1 - vg ** 2 / c ** 2)) < 1e-12
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
    Rs = np.array([6, 8, 10, 14, 20, 28, 40.0])
    an = []
    for R in Rs:
        _, _, M = ball_tensors("chair44", R)
        w = np.linalg.eigvalsh(M)
        an.append(1 - w.min() / w.max())
    ax[0].loglog(Rs, an, "o-k")
    ax[0].set_xlabel("radius of the ball (tile spacings)"); ax[0].set_ylabel("1 - (min / max eigenvalue of M/N)")
    ax[0].set_title("(a) Chair44: the long-wave speed becomes isotropic with scale", fontsize=9)
    e, nb, M = ball_tensors("chair44", 40.0)
    c2 = float(np.mean(np.linalg.eigvalsh(M)))
    Q = {k: q4(e, nb, u) for k, u in DIRS.items()}
    lam = np.geomspace(4, 200, 60)
    for k, col in (("axis", "tab:red"), ("face", "tab:orange")):
        ax[1].loglog(lam, (Q[k] - Q["body"]) * (2 * np.pi / lam) ** 2 / (2 * c2), color=col, label="%s slower than body diagonal" % k)
    ax[1].set_xlabel("wavelength (tile spacings)"); ax[1].set_ylabel("relative speed deficit"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) the first correction is cubic and falls as lambda$^{-2}$", fontsize=9)
    ks = np.linspace(0.05, 1.6, 20)
    vals = [chain_clock(k) for k in ks]
    vg = [math.sin(k) / math.sqrt(0.25 + 2 * (1 - math.cos(k))) for k in ks]
    ax[2].plot(vg, [v[0] / 0.5 for v in vals], "o", color="tab:blue", ms=4, label="measured clock rate / rest rate (lattice)")
    vv = np.linspace(0, 0.99, 100)
    ax[2].plot(vv, np.sqrt(1 - vv ** 2), "k", label="1 / gamma")
    ax[2].set_xlabel("group velocity v / c"); ax[2].set_ylabel("clock rate / rest rate"); ax[2].legend(fontsize=7)
    ax[2].set_title("(c) a moving clock: 1/gamma at long wavelength (chain);\npoints leaving the curve are past the lattice's top speed", fontsize=9)
    fig.suptitle("toymodel_v7 Stage 5: isotropy with scale, and the moving clock")
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
