#!/usr/bin/env python3
"""t6_clock.py - Stage 4 of toymodel_v6: a hopping particle coupled to the bits, read as a clock.

The question of toymodel_v2 (C7) and v5 Stage 5 in Hamiltonian form: is the overlap c of the bit's two states a rate, and if so, is it the clock factor sqrt(1 - r_s / r)?  Here a particle hops on a chain, and its
presence changes the ground state of a fast bit (gap Delta >> t) to one with overlap c with the old.  Then every hop carries the Franck-Condon factor of the bits it changes.  Two placements of the bit:

  site bits (D6.15): one bit per tile, rotated while the particle sits on the tile.  A hop changes two bits: t_eff = t c^2.
  link bits (D6.16): one bit per seam, its state set by which side of the seam the particle is on (the bit as the flux at the seam's vertex, Stage 3).  A hop changes one bit: t_eff = t c.

Definitions
  D6.15  H = - t sum_j (|j+1><j| + h.c.) - Delta sum_k m_k(j) . sigma_k,  m = (0, 0, 1) for an unoccupied site k, m = (sin theta, 0, cos theta) for the occupied one; cos(theta / 2) = c.
  D6.16  Same with one qubit per link k, field m0 if the particle is left of link k (j <= k), m1 if it is right.
  D6.17  Clock rate = t_eff / t (the hopping rate relative to the bare one; a Definition of what the particle's "clock" is).  Along the flow c = c(tau) of Stage 0.  Under rule I of toymodel_v2 (d^2 proportional to the field
         1 / r), c^2 = exp(-r_s / r).

Exact / Numeric claims (each a test; ``python3 t6_clock.py --tests-only``)
  E6.22  Site bits narrow the band by c^2.  On 6 sites (6 bits, 384 states) the 6 lowest levels are those of an open chain with t_eff; t_eff / t -> c^2 for c = 0.3, 0.6, 0.9 as Delta / t = 10, 40, 160; the error of t_eff falls as
         1 / Delta^2 and the residual of the open-chain fit as 1 / Delta (ratio of residuals at Delta = 40 and 160 within 10% of 4).
  E6.23  Link bits narrow it by c.  Same with 5 link bits: t_eff / t -> c, residual proportional to 1 / Delta.
  E6.24  Along the flow the clock runs slow at the frozen end: link rate c(tau) from 0.223 at the Spectre to 1 at the Hat (site rate c^2 from 0.050), strictly increasing.  Frozen (readable) = slow; plastic = free.
  E6.25  A slow region delays a packet by the semiclassical amount.  On a chain of 2400 sites with bond hopping t c(x), a slab of c = 0.5 (smooth edges, width 300), a packet at the band centre (k = pi/2, where k does not change
         across the slab) arrives late by the time of flight integral of 1 / (2 t c(x)) minus the free one (136.2 against 136.1, test within 2%); the reflected weight is below 1e-3.
  E6.26  C7: the link placement passes to first order, the site placement does not.  Under rule I, link rate = exp(-r_s / 2r): equal to sqrt(1 - r_s / r) to first order in r_s / r (10.1% above it at r = 2 r_s, below 1%
         from r = 7.5 r_s, the numbers of toymodel_v2).  Site rate = exp(-r_s / r): its first-order term is twice the clock's, and it differs from it by r_s / 2r at large r (relative), so it is the square of the clock.

Status.  E6.22, E6.23: exact diagonalisation (Numeric in the limit Delta -> infinity, Exact as a perturbative statement at first order in t / Delta).  E6.24, E6.26: Exact algebra.  E6.25: Numeric.  That a bit is fast, that the particle
rotates it, and that the hopping rate is a clock are Definitions.  Nothing here is gravity.

    python3 t6_clock.py --tests-only
    python3 t6_clock.py --no-show --save figures/t6_stage4_clock.png
"""
import argparse
import math
import os
import sys

import numpy as np
import scipy.sparse as sps
import scipy.sparse.linalg as sla

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import t6_pair as t0  # noqa: E402


def fields(c):
    th = 2.0 * math.acos(c)
    return np.array([0.0, 0.0, 1.0]), np.array([math.sin(th), 0.0, math.cos(th)])


def polaron_hamiltonian(N, t, Delta, c, mode):
    m0, m1 = fields(c)
    ne = N if mode == "site" else N - 1
    E = 1 << ne
    dim = N * E
    H = np.zeros((dim, dim))
    e = np.arange(E)
    for j in range(N):
        base = j * E
        for k in range(ne):
            m = (m1 if k == j else m0) if mode == "site" else (m0 if j <= k else m1)
            b = (e >> k) & 1
            H[base + e, base + e] += -Delta * m[2] * (1 - 2 * b)
            H[base + (e ^ (1 << k)), base + e] += -Delta * m[0]
        if j < N - 1:
            H[(j + 1) * E + e, base + e] -= t
            H[base + e, (j + 1) * E + e] -= t
    return H


def effective_hopping(N, t, Delta, c, mode):
    w = np.linalg.eigvalsh(polaron_hamiltonian(N, t, Delta, c, mode))[:N]
    w = w - w.mean()
    ref = -2.0 * np.cos(np.pi * np.arange(1, N + 1) / (N + 1))
    ref = ref - ref.mean()
    te = float(w @ ref / (ref @ ref))
    return te, float(np.abs(w - te * ref).max())


def slab_profile(n, c_slab=0.5, start=900, width=300, edge=40.0):
    x = np.arange(n - 1) + 0.5
    s = 0.5 * (np.tanh((x - start) / edge) - np.tanh((x - start - width) / edge))
    return 1.0 - (1.0 - c_slab) * s


def chain(cs, t=1.0):
    n = len(cs) + 1
    return sps.diags([-t * cs, -t * cs], [1, -1], shape=(n, n), format="csc")


def packet(n, x0=400, sigma=40.0, k0=math.pi / 2):
    x = np.arange(n)
    psi = np.exp(-((x - x0) ** 2) / (4 * sigma ** 2) + 1j * k0 * x)
    return psi / np.linalg.norm(psi)


def delay_run(T=850.0, n=2400, c_slab=0.5):
    cs = slab_profile(n, c_slab)
    psi0 = packet(n)
    out = {}
    for name, prof in (("slab", cs), ("free", np.ones(n - 1))):
        psi = sla.expm_multiply(-1j * T * chain(prof), psi0)
        p = np.abs(psi) ** 2
        out[name] = p
    x = np.arange(n)
    beyond = x > 1400
    xs = (out["slab"][beyond] * x[beyond]).sum() / out["slab"][beyond].sum()
    xf = (out["free"][beyond] * x[beyond]).sum() / out["free"][beyond].sum()
    measured = (xf - xs) / 2.0                       # free speed 2 t at k = pi/2
    predicted = float(np.sum(1.0 / (2.0 * cs) - 0.5))
    reflected = float(out["slab"][x < 700].sum())
    return measured, predicted, reflected, out, cs


# --------------------------------------------------------------------------
def _narrowing(mode, power, n):
    res = {}
    for c in (0.3, 0.6, 0.9):
        r = [effective_hopping(n, 1.0, D, c, mode) for D in (10.0, 40.0, 160.0)]
        assert abs(r[-1][0] - c ** power) < 2e-4, (mode, c, r)
        assert abs(r[1][1] / r[2][1] - 4.0) < 0.4, (mode, c, r)
        res[c] = round(r[-1][0], 5)
    return res


def test_e622_site_bits_narrow_by_c_squared():
    return _narrowing("site", 2, 6)


def test_e623_link_bits_narrow_by_c():
    return _narrowing("link", 1, 6)


def test_e624_the_frozen_end_is_slow():
    taus = np.linspace(0, 1, 101)
    c = np.array([t0.overlap(t) for t in taus])
    assert np.all(np.diff(c) > 0) and abs(c[0] - 0.2231) < 1e-4 and abs(c[-1] - 1) < 1e-15
    assert abs(c[0] ** 2 - 0.0498) < 1e-4
    return round(float(c[0]), 4), round(float(c[0] ** 2), 4)


def test_e625_a_slow_region_delays_a_packet_by_the_time_of_flight():
    m, p, refl, _, _ = delay_run()
    assert abs(m / p - 1) < 0.02, (m, p)
    assert refl < 1e-3, refl
    return round(m, 2), round(p, 2), refl


def test_e626_c7_link_passes_first_order_site_does_not():
    r = np.array([2.0, 7.5, 100.0, 1e4])
    link = np.exp(-1 / (2 * r))
    site = np.exp(-1 / r)
    clock = np.sqrt(1 - 1 / r)
    rel_link = link / clock - 1
    assert abs(rel_link[0] - 0.101) < 1e-3 and abs(rel_link[1]) < 0.01
    big = r[-1]
    assert abs((link[-1] - clock[-1]) * big) < 1e-3             # first-order terms agree
    assert abs((site[-1] - clock[-1]) * big + 0.5) < 1e-3        # site: first-order term off by r_s / 2r
    assert np.allclose(site, link ** 2)
    return [round(x, 4) for x in rel_link]


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e6")]
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

    fig, ax = plt.subplots(1, 4, figsize=(17, 4.4))
    cs = np.linspace(0.1, 0.95, 9)
    for mode, mk, col in (("site", "o", "tab:blue"), ("link", "s", "tab:red")):
        ax[0].plot(cs, [effective_hopping(6, 1.0, 160.0, c, mode)[0] for c in cs], mk, color=col, label="%s bits (exact, Delta = 160)" % mode)
    cc = np.linspace(0, 1, 50)
    ax[0].plot(cc, cc ** 2, "--", color="tab:blue", label="c$^2$"); ax[0].plot(cc, cc, "--", color="tab:red", label="c")
    ax[0].set_xlabel("overlap c of the bit's two ground states"); ax[0].set_ylabel("t$_{eff}$ / t"); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) each hop carries the overlap of the bits it changes", fontsize=9)
    Ds = np.geomspace(5, 320, 10)
    for c, ls in ((0.3, "-"), (0.6, "--")):
        for mode, col in (("site", "tab:blue"), ("link", "tab:red")):
            ax[1].loglog(Ds, [abs(effective_hopping(6, 1.0, D, c, mode)[0] - (c ** 2 if mode == "site" else c)) + 1e-12 for D in Ds], ls, color=col, label="%s, c = %.1f" % (mode, c))
    ax[1].loglog(Ds, 5.0 / Ds ** 2, "k:", label="1 / Delta$^2$")
    ax[1].set_xlabel("Delta / t"); ax[1].set_ylabel("|t$_{eff}$ - limit|"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) the limit is reached as the bit becomes fast", fontsize=9)
    m, p, refl, out, prof = delay_run()
    x = np.arange(len(out["free"]))
    ax[2].plot(x, out["free"], color="0.5", label="no slab")
    ax[2].plot(x, out["slab"], color="tab:red", label="slab of c = 0.5")
    ax2 = ax[2].twinx(); ax2.plot(np.arange(len(prof)) + 0.5, prof, "k:", lw=0.8); ax2.set_ylabel("c(x)")
    ax[2].set_xlabel("site"); ax[2].set_ylabel("|psi|$^2$ at t = 850"); ax[2].legend(fontsize=7, loc="upper left")
    ax[2].set_title("(c) delay %.1f against time of flight %.1f" % (m, p), fontsize=9)
    r = np.geomspace(1.2, 100, 200)
    ax[3].semilogx(r, np.sqrt(1 - 1 / r), "k", label="clock sqrt(1 - r$_s$/r)")
    ax[3].semilogx(r, np.exp(-1 / (2 * r)), "tab:red", label="link rate c = exp(-r$_s$/2r)")
    ax[3].semilogx(r, np.exp(-1 / r), "tab:blue", label="site rate c$^2$ = exp(-r$_s$/r)")
    ax[3].set_xlabel("r / r$_s$"); ax[3].set_ylabel("rate"); ax[3].legend(fontsize=7)
    ax[3].set_title("(d) C7 under rule I: the link rate is the clock to first order", fontsize=9)
    fig.suptitle("toymodel_v6 Stage 4: a particle that changes fast bits as it hops, read as a clock")
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
