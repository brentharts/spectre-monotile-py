#!/usr/bin/env python3
"""t7_space.py - Stage 4 of toymodel_v7: the space factor.  Can the bit that gives the clock (v6 Stage 4: the seam rate c) also give the spatial factor that doubles the bending of light?

For a wave in a massless band the coordinate speed is (hops per unit time) x (coordinate length per hop), and the ray optics depend only on the index n = v_inf / v (Definition D7.7, the eikonal of H = c(x) H0).  v6 Stage 4
fixed the first factor: the seam rate is c, with c = exp(-r_s / 2r) under rule I.  So n = 1 / (c l / l_inf), and the deflection of a ray of impact parameter b is computed by the quadrature of toymodel_v3
(`t3_optics.deflection`, r_s = 1).  General relativity's 2 r_s / b needs n = 1 / A = 1 / c^2: a coordinate length per hop proportional to c, shrinking near the source.

Two readings of how the field reaches the tile (H7.2 of PLAN.md):
  S1  the field sets only the opening h of the bit (h^2 = -2 ln c = r_s / r); the tile's shape (b, k) stays.  Only the bonds at an open P8 change.
  S2  the field sets the whole flow time tau (c(tau) = c), so the tile's edge b(tau) = 1 + (sqrt3 - 1) tau changes too.

Exact / Numeric claims (each a test; ``python3 t7_space.py --tests-only``)
  E7.12  The bit lengthens its own edges by the inverse of its overlap.  The two edges at P8 have length l = sqrt(1 + h^2) (= 1 / cos(omega / 2)) and the overlap is c = exp(-h^2 / 2), so l c = 1 - h^4 / 4 + O(h^6):
         l = 1 / c to first order.  The area ledger keeps the mean tile area of a pair fixed (A+ + A- = 2 A0), so no space factor comes from area.
  E7.13  Reading S1.  Clock only gives coefficient 1.00; the needed l = c gives 2.00; every bond behaving like a P8 edge gives 0.00 (the clock cancelled).  On the patch the open bonds are a fraction f = 2 (pairs) / (bonds)
         = 0.0204, 0.0382, 0.0496 at levels 2, 3, 4, and the coefficient is 1 - f (0.950 at level 4): the tile's geometry works against the bending.  **Negative.**
  E7.14  Reading S2.  The mean bond length of the patch is 1 + 0.4416 (b - 1) (level 4); near the Hat 1 - c ~ (1 - tau)^2 while b changes linearly in 1 - tau, so n - 1 ~ sqrt(r_s / r) and the deflection falls as
         b^-1/2 (local slope -0.53 between b = 3200 and 6400), not as r_s / b.  The wrong order.  **Negative.**

Status.  E7.12: Exact (series).  E7.13, E7.14: Numeric (quadrature, fitted coefficients) on Exact bond counts.  D7.7 and the readings S1, S2 are Definitions.  Nothing here is light.

    python3 t7_space.py --tests-only
    python3 t7_space.py --no-show --save figures/t7_stage4_space.png
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_RES, "toymodel_v3"), os.path.join(_RES, "toymodel_v4"), os.path.join(_RES, "toymodel_v6"), os.path.dirname(_RES), os.path.join(os.path.dirname(_RES), "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t3_optics as vo  # noqa: E402
import t6_pair as t0  # noqa: E402

SQRT3 = math.sqrt(3.0)
_CACHE = {}


def _exp(x):
    return math.exp(min(x, 700.0))


def bond_stats(level):
    """(bonds, pairs, f = 2 pairs / bonds, fraction of b-edges) of the level's vertex graph."""
    if level not in _CACHE:
        import _tb
        import t4_patch as tp
        G1 = _tb.patch_graph(level, 1.0, 1.0)
        Gb = _tb.patch_graph(level, 1.0, SQRT3)
        nb = len(G1.bonds)
        pairs = len(tp.flat_pairs(level, 1.0))
        fb = float(np.mean(np.abs(np.asarray(Gb.length) - SQRT3) < 1e-3))     # vertices are rounded to 5 decimals
        _CACHE[level] = (nb, pairs, 2.0 * pairs / nb, fb)
    return _CACHE[level]


def n_clock():
    return lambda r: _exp(1.0 / (2 * r))


def n_needed():
    return lambda r: _exp(1.0 / r)


def n_all_p8():
    return lambda r: _exp(1.0 / (2 * r)) / math.sqrt(1 + 1.0 / r)


def n_s1(f):
    return lambda r: _exp(1.0 / (2 * r)) / (1 + f * (math.sqrt(1 + 1.0 / r) - 1))


def tau_of_c(c):
    lo, hi = 0.0, 1.0
    for _ in range(80):
        m = 0.5 * (lo + hi)
        lo, hi = (m, hi) if t0.overlap(m) < c else (lo, m)
    return 0.5 * (lo + hi)


def n_s2(f, fb):
    def ell(tau):
        b = 1 + (SQRT3 - 1) * tau
        h = t0.h(tau)
        return 1 + fb * (b - 1) + f * (math.sqrt(1 + h * h) - 1)
    inf = ell(1.0)

    def n(r):
        if r < 0.05:
            return 1e6
        c = math.exp(-1.0 / (2 * r))
        return inf / (c * ell(tau_of_c(c)))
    return n


# --------------------------------------------------------------------------
def test_e712_the_bit_lengthens_its_edges_by_the_inverse_overlap():
    import sympy as sp
    h = sp.symbols("h")
    ser = sp.series(sp.sqrt(1 + h ** 2) * sp.exp(-h ** 2 / 2), h, 0, 6).removeO()
    assert sp.simplify(ser - (1 - h ** 4 / 4)) == 0
    for tau in (0.0, 0.5, 0.9):
        om = math.radians(120.0 * (1 - tau))
        assert abs(math.sqrt(1 + t0.h(tau) ** 2) - 1 / math.cos(om / 2)) < 1e-12
    return "l c = 1 - h^4/4 + O(h^6)"


def test_e713_reading_s1_reduces_the_bending():
    c1 = vo.coefficients(n_clock())[0]
    c2 = vo.coefficients(n_needed())[0]
    c0 = vo.coefficients(n_all_p8())[0]
    assert abs(c1 - 1) < 2e-3 and abs(c2 - 2) < 2e-3 and abs(c0) < 2e-3, (c1, c2, c0)
    fs = []
    for lv, want in ((2, 0.0204), (3, 0.0382), (4, 0.0496)):
        f = bond_stats(lv)[2]
        assert abs(f - want) < 1e-4, (lv, f)
        fs.append(f)
    cs = vo.coefficients(n_s1(fs[-1]))[0]
    assert abs(cs - (1 - fs[-1])) < 2e-3 and cs < 1, cs
    return round(c1, 4), round(c2, 4), round(c0, 4), [round(x, 4) for x in fs], round(cs, 4)


def test_e714_reading_s2_has_the_wrong_order():
    _, _, f, fb = bond_stats(4)
    assert abs(fb - 0.4416) < 1e-3, fb
    n = n_s2(f, fb)
    al = [vo.deflection(n, b) for b in (3200.0, 6400.0)]
    slope = math.log(al[1] / al[0]) / math.log(2.0)
    assert abs(slope + 0.5) < 0.05, slope
    return round(slope, 3)


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
    hh = np.linspace(0, 1.8, 200)
    ax[0].plot(hh, np.sqrt(1 + hh ** 2), "tab:red", label="edge at P8, l = sqrt(1 + h$^2$)")
    ax[0].plot(hh, np.exp(hh ** 2 / 2), "k--", label="1 / c = exp(h$^2$/2)")
    ax[0].plot(hh, np.exp(-hh ** 2 / 2), "tab:green", label="needed for GR: l = c")
    ax[0].set_xlabel("opening h of the bit"); ax[0].set_ylabel("length per hop"); ax[0].set_ylim(0, 3); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) the bit lengthens its edges as 1/c, GR needs c", fontsize=9)
    _, _, f4, fb = bond_stats(4)
    names = ["clock only", "S1: patch\n(f = %.3f)" % f4, "every bond\na P8 edge", "needed\n(l = c)"]
    vals = [vo.coefficients(n)[0] for n in (n_clock(), n_s1(f4), n_all_p8(), n_needed())]
    ax[1].bar(range(4), vals, color=["0.5", "tab:red", "tab:red", "tab:green"])
    for i, v in enumerate(vals):
        ax[1].text(i, v + 0.04, "%.3f" % v, ha="center", fontsize=8)
    ax[1].axhline(2, color="k", ls=":", lw=0.8)
    ax[1].set_xticks(range(4)); ax[1].set_xticklabels(names, fontsize=8); ax[1].set_ylabel("deflection coefficient (units r$_s$/b)")
    ax[1].set_title("(b) reading S1: the tile reduces the bending", fontsize=9)
    bs = np.geomspace(100, 6400, 8)
    ax[2].loglog(bs, [vo.deflection(n_s2(f4, fb), b) for b in bs], "o-", color="tab:purple", label="S2: the field sets tau")
    ax[2].loglog(bs, [vo.deflection(n_clock(), b) for b in bs], "s-", color="0.5", label="clock only")
    ax[2].loglog(bs, 2.0 / bs, "k:", label="GR: 2 r$_s$/b")
    ax[2].loglog(bs, 1.3 / np.sqrt(bs), "k-.", lw=0.8, label="b$^{-1/2}$")
    ax[2].set_xlabel("impact parameter b (units r$_s$)"); ax[2].set_ylabel("deflection (rad)"); ax[2].legend(fontsize=7)
    ax[2].set_title("(c) reading S2: the wrong order", fontsize=9)
    fig.suptitle("toymodel_v7 Stage 4: the space factor from the tile, in two readings")
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
