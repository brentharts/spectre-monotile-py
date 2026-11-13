#!/usr/bin/env python3
"""t7_horizon.py - Stage 6 of toymodel_v7: can the bits make a horizon by themselves?

A horizon in the toy is a radius where the overlap of the bit's two states reaches c = 0 (the bit fully readable, the clock stopped).  Rule I of toymodel_v2 ties the overlap to the static field, c^2 = exp(-g phi); the escape
coupling of toymodel_v3 ties it to the probability h that the substrate's walk reaches the source, c^2 = 1 - h (Result res:escape; on Z^3 c^2 = 1 - R_eff / r).  The bits are themselves sources of strength proportional to
their opening h (v5, v6), so the field and the overlap could feed back on each other.  This stage asks whether that feedback, or the tile, can make c = 0 at a finite radius.

Definitions
  D7.8  Opening and overlap: c = exp(-h^2 / 2) (v6, sigma = 1), so h^2 = -2 ln c.  Under rule I, h^2 = g phi.
  D7.9  Back-reaction: a point source M at the origin and a cloud of bits with source density rho = alpha h(r) = alpha sqrt(g phi(r)) for r < R_cut (bits aligned with the source: the sign that deepens the well).
        Radial solution phi(r) = (1/r) int_0^r 4 pi rho r'^2 dr' + int_r^R 4 pi rho r' dr' + M / r, solved by iteration on a grid.
  D7.10 Validity: the reflex value of the bit is a valid outline only for corner angles below omega_c (straight Spectre: 2 arctan h*, h* = t4_proofs.h_star(1)); beyond it there is no tile.

Exact / Numeric claims (each a test; ``python3 t7_horizon.py --tests-only``)
  E7.15  The tile has a floor on the overlap.  On the flow range c >= c(0) = 0.2231 (omega0 = 120).  Extended past the Spectre the opening h = tan(omega / 2) diverges only at omega = 180 degrees, but the reflex value stops being a tile at
         omega_c = 134.18 degrees (h* = (3 + sqrt3) / 2 = 2.366), so c >= c* = exp(-h*^2 / 2) = 0.0609 for every tile.  The tile cannot represent c = 0.
  E7.16  Rule I has no horizon at any back-reaction.  With a nonnegative source density, |phi'(r)| <= M_enc(R) / r^2, so phi is bounded at every r > 0 (Gauss), and c^2 = exp(-g phi) > 0 at every finite radius, for any coupling and
         any power of h.  Numerically (g = 1, M = 1, alpha = 1e-4 and 5e-4, R_cut = 40): the iteration converges, the Gauss bound holds on the grid, the cloud deepens the well (r phi at r = 0.5 rises above 1) and ln c stays finite everywhere.  With
         alpha = 5e-4 rule I asks for c below the tile's floor c* on the whole cloud (r_sat = R_cut = 40; with alpha = 1e-4 nowhere): the bits saturate there, a frozen
         region, still not c = 0.
         **Negative** for a horizon from the bits under rule I.
  E7.17  The escape coupling meets the floor.  With c^2 = 1 - R_eff / r the overlap reaches 0 at R_eff; with the tile's floor, the bits saturate at r* = R_eff / (1 - c*^2) = 1.0037 R_eff, inside which c stays at c* = 0.0609.
         The horizon of v3 becomes a shell of maximally open bits just outside R_eff: a frozen surface, not a point of zero overlap.

Status.  E7.15, E7.17: Exact (algebra and the closed form of h*).  E7.16: Exact (the Gauss bound) with a Numeric illustration.  Rule I, the escape coupling and the bits as aligned sources are Definitions.

    python3 t7_horizon.py --tests-only
    python3 t7_horizon.py --no-show --save figures/t7_stage6_horizon.png
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_RES, "toymodel_v4"), os.path.join(_RES, "toymodel_v6"), os.path.dirname(_RES), os.path.join(os.path.dirname(_RES), "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t6_pair as t0  # noqa: E402


def h_star():
    import t4_proofs as tpf
    return float(tpf.h_star(1.0))


def c_floor():
    return math.exp(-h_star() ** 2 / 2)


def self_consistent(alpha, g=1.0, M=1.0, R=40.0, n=4000, rmin=0.5, iters=400):
    r = np.linspace(rmin, R, n)
    dr = r[1] - r[0]
    phi = M / r
    for _ in range(iters):
        rho = alpha * np.sqrt(g * np.maximum(phi, 0.0))
        inner = np.cumsum(4 * np.pi * rho * r ** 2) * dr
        outer = np.cumsum((4 * np.pi * rho * r)[::-1])[::-1] * dr
        new = M / r + inner / r + outer
        if np.max(np.abs(new - phi)) < 1e-12:
            phi = new
            break
        phi = 0.5 * phi + 0.5 * new
    return r, phi


# --------------------------------------------------------------------------
def test_e715_the_tile_has_a_floor_on_the_overlap():
    taus = np.linspace(0, 1, 201)
    c = np.array([t0.overlap(t) for t in taus])
    assert abs(c.min() - 0.2231) < 1e-4 and abs(c[0] - c.min()) < 1e-15
    hs = h_star()
    assert abs(2 * math.degrees(math.atan(hs)) - 134.2) < 0.05
    assert abs(hs - (3 + math.sqrt(3)) / 2) < 1e-12 and abs(c_floor() - 0.0609) < 1e-4
    return round(hs, 4), round(c_floor(), 4)


def test_e716_rule_one_has_no_horizon():
    out = []
    for alpha in (1e-4, 5e-4):
        r, phi = self_consistent(alpha)
        assert np.all(np.isfinite(phi))
        # the Gauss bound: |phi'| <= M_enc(R) / r^2
        rho = alpha * np.sqrt(np.maximum(phi, 0.0))
        Menc = 1.0 + np.sum(4 * np.pi * rho * r ** 2) * (r[1] - r[0])
        dphi = np.abs(np.gradient(phi, r))
        assert np.all(dphi <= Menc / r ** 2 * (1 + 1e-2) + 1e-9)
        lnc = -phi / 2                                   # ln c, finite where phi is
        assert np.all(np.isfinite(lnc))
        out.append((alpha, round(float(phi[0] * r[0]), 3), round(float(lnc.min()), 2)))
    assert out[1][1] > out[0][1] > 1.0                  # the cloud deepens the well
    # with the stronger cloud rule I asks for c below the tile's floor: the bits saturate inside r_sat
    r, phi = self_consistent(5e-4)
    below = r[-phi / 2 < math.log(c_floor())]
    r_sat = float(below.max()) if len(below) else 0.0
    _, phi1 = self_consistent(1e-4)
    assert abs(r_sat - 40.0) < 0.05 and np.all(-phi1 / 2 > math.log(c_floor()))   # 5e-4: the whole cloud is below the floor; 1e-4: none of it
    return out, round(r_sat, 2)


def test_e717_the_escape_coupling_meets_the_floor():
    cs = c_floor()
    rstar = 1.0 / (1 - cs ** 2)
    assert abs(rstar - 1.0037) < 2e-4
    r = np.linspace(1.0, 3.0, 2001)
    c = np.sqrt(np.clip(1 - 1 / r, 0, None))
    c_tile = np.maximum(c, cs)
    assert c.min() == 0 and c_tile.min() == cs
    assert np.all(c_tile[r > rstar + 1e-3] == c[r > rstar + 1e-3])
    return round(rstar, 4)


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
    om = np.linspace(0, 179, 400)
    hh = np.tan(np.radians(om) / 2)
    ax[0].semilogy(om, np.exp(-hh ** 2 / 2), "k")
    oc = 2 * math.degrees(math.atan(h_star()))
    ax[0].axvline(120, color="tab:blue", ls=":", label="flow range ends (omega0 = 120)")
    ax[0].axvline(oc, color="tab:red", ls="--", label="critical angle %.1f: no tile beyond" % oc)
    ax[0].axhline(c_floor(), color="tab:red", lw=0.8)
    ax[0].set_xlabel("corner angle omega at P8 (degrees)"); ax[0].set_ylabel("overlap c = exp(-h$^2$/2)"); ax[0].set_ylim(1e-12, 2); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) the tile cannot represent c below %.4f" % c_floor(), fontsize=9)
    for alpha, col in ((0.0, "0.5"), (1e-4, "tab:blue"), (5e-4, "tab:purple")):
        r, phi = self_consistent(alpha) if alpha else (np.linspace(0.5, 40, 4000), 1 / np.linspace(0.5, 40, 4000))
        ax[1].plot(r, np.exp(-phi / 2), color=col, label="alpha = %g" % alpha)
    ax[1].set_xscale("log"); ax[1].set_yscale("log"); ax[1].set_xlabel("r"); ax[1].set_ylabel("overlap c (rule I)"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) rule I with back-reaction: c > 0 at every r", fontsize=9)
    r = np.linspace(0.9, 3.0, 800)
    c = np.sqrt(np.clip(1 - 1 / r, 0, None))
    ax[2].plot(r, c, "k--", label="escape coupling, c$^2$ = 1 - R$_{eff}$/r")
    ax[2].plot(r, np.maximum(c, c_floor()), "tab:red", label="with the tile's floor")
    ax[2].axvline(1 / (1 - c_floor() ** 2), color="tab:red", ls=":", lw=0.8)
    ax[2].set_xlabel("r / R$_{eff}$"); ax[2].set_ylabel("overlap c"); ax[2].legend(fontsize=7)
    ax[2].set_title("(c) the horizon becomes a shell of maximally open bits", fontsize=9)
    fig.suptitle("toymodel_v7 Stage 6: can the bits make a horizon?")
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
