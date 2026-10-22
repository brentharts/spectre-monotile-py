#!/usr/bin/env python3
"""spectre_cosmology.py - the Spectre's growth rates against two cosmological singularities (pancake and BKL).

Ported from the spectre repository (spectre_pancake.py, spectre_bkl.py), where every statement has a ledger label.
Nothing here needs the tile geometry: only the inflation lambda^2 = 4 + sqrt(15), the perimeter growth
nu = 2 + sqrt(5) used by spectre_fractal.py, and the hexagonal rotation of the cusp torus.  Three parts.

  A. the BKL (Mixmaster) map as a Gauss map  (``bkl_step``, ``closed_orbit``)
       u -> u - 1 for u >= 2 and u -> 1/(u - 1) for 1 < u < 2.  The orbit of u* = 3 + sqrt(15) = [6;1,6,1,...] closes
       after SEVEN epochs, in eras of 6 and 1 epochs, with multiplier (4 + sqrt15)^2 = lambda^4 exactly.  The orbit of
       nu = 2 + sqrt5 = [4;4,...] is one era of 4 epochs with multiplier -nu^2 (det [[4,1],[1,0]] = -1).  Hence the
       fractal dimension of the supertile outline of spectre_fractal.py,
            D = ln(nu) / ln(lambda) = 2 ln|m_nu| / ln m_lambda  with  m_nu = -nu^2,  m_lambda = lambda^4,
       is a ratio of the log-multipliers of two closed Mixmaster orbits (``dimension_from_orbits``).
       NOT specific to the Spectre: every periodic continued fraction gives a closed orbit whose multiplier is the
       square of its unit (``test_every_period_closes``).  There is no dynamics on the tiling for it to describe.

  B. the pancake's fibre clock  (``turns_per_level``, ``levels_per_turn``)
       A null ray along the collapsing fibre of the pancake of Dechant, Lasenby and Hobson winds the comoving angle
       (2/a0) ln(t/t_in), and the Euler angle has period 4 pi on S^3 (that identification is not checked).  Taking
       one substitution level as a rescaling by lambda, a level is ln(lambda)/(2 pi a0) fibre turns.  At a0 = 1 that
       is 6.090 levels per turn: close to 6 and not 6; the a0 that would give 6 is 0.985, a tuned value.  This is
       numerology and the self test asserts the miss.  (The coefficient 2/a0 was checked against a numerical
       pancake solution in pyCICY; it is quoted here.)

  C. the orbifold of the cusp torus  (``stabilisers``)
       The 60-degree rotation R = [[0,-1],[1,1]] (order 6, in the basis E1, E2 of the hexagonal lattice) has
       stabilisers 6, 2, 3 on the six site classes (the centre, three edge midpoints, two vertices), so the torus
       modulo Z/6 is the sphere S^2(2,3,6) and 2 - (1-1/2) - (1-1/3) - (1-1/6) = 0 = chi(T)/6.  The same Z/6 in the
       fibre circle gives the lens space L(6,1) over the pancake's S^2 base; sharing the group is the entire overlap,
       no map between the two is constructed.

What the self tests check (run ``python3 spectre_cosmology.py --tests-only``):
  * the BKL orbits above (periods, eras, multipliers) in exact arithmetic, and a grid of other periods;
  * D = ln nu / ln lambda = 1.3993 equals 2 ln|m_nu| / ln m_lambda;
  * the fibre clock: N(1) is not 6, and a0 = 3 ln(lambda)/pi is the value that would make it 6;
  * the stabilisers, orbits and the orbifold Euler characteristic.
"""
import argparse
import sys
from fractions import Fraction as Fr

import numpy as np
import sympy as sp

S15, S5 = sp.sqrt(15), sp.sqrt(5)
LAMBDA2 = 4 + S15           # inflation (area) factor; linear factor is sqrt of it
LAMBDA = sp.sqrt(LAMBDA2)
NU = 2 + S5                 # outline-edge growth per level
R = ((0, -1), (1, 1))       # 60-degree rotation of the hexagonal lattice in the basis (E1, E2)


# --------------------------------------------------------------------------
# A. the BKL map
# --------------------------------------------------------------------------
def bkl_step(u):
    """One epoch of the BKL map, exact: (u', du'/du)."""
    if sp.N(u, 60) >= 2:
        return sp.radsimp(sp.simplify(u - 1)), sp.Integer(1)
    return sp.radsimp(sp.simplify(1 / (u - 1))), sp.simplify(-1 / (u - 1) ** 2)


def closed_orbit(u0, max_epochs=40):
    """(epochs of u0 until it returns, era lengths, multiplier).  An inversion ends an era."""
    us, ders, u = [u0], [], u0
    for _ in range(max_epochs):
        u, d = bkl_step(u)
        ders.append(d)
        if sp.simplify(u - u0) == 0:
            eras, run = [], 0
            for d_ in ders:
                run += 1
                if d_ != 1:
                    eras.append(run)
                    run = 0
            assert run == 0
            return us, eras, sp.radsimp(sp.simplify(sp.prod(ders)))
        us.append(u)
    raise ValueError("no closed orbit within %d epochs" % max_epochs)


def periodic_u(a, b):
    """[a; b, a, b, ...] as an exact surd."""
    x = sp.Symbol("x", positive=True)
    return [s for s in sp.solve(sp.Eq(x, a + 1 / (b + 1 / x)), x) if sp.N(s) > 0][0]


def unit_of(a, b):
    M = sp.Matrix([[a, 1], [1, 0]]) * sp.Matrix([[b, 1], [1, 0]])
    return max(M.eigenvals(), key=lambda e: sp.N(e))


def kasner_exponents(u):
    D = 1 + u + u ** 2
    return (-u / D, (1 + u) / D, u * (1 + u) / D)


def dimension_from_orbits():
    """D = 2 ln|m_nu| / ln m_lambda from the two closed orbits (exact multipliers), as a float."""
    _, _, m_nu = closed_orbit(NU)
    _, _, m_lam = closed_orbit(3 + S15)
    return float(2 * sp.log(abs(m_nu)) / sp.log(m_lam))


# --------------------------------------------------------------------------
# B. the pancake fibre clock
# --------------------------------------------------------------------------
def turns_per_level(a0):
    """Fibre turns per substitution level: ln(lambda) / (2 pi a0)."""
    return sp.log(LAMBDA) / (2 * sp.pi * sp.sympify(a0))


def levels_per_turn(a0):
    return 1 / turns_per_level(a0)


def a0_for_levels_per_turn(N):
    """The a0 that would make N levels one turn: tuning, not a derivation."""
    return sp.sympify(N) * sp.log(LAMBDA) / (2 * sp.pi)


# --------------------------------------------------------------------------
# C. the orbifold
# --------------------------------------------------------------------------
def _mat_pow(A, n):
    out = ((1, 0), (0, 1))
    for _ in range(n):
        out = tuple(tuple(sum(out[i][k] * A[k][j] for k in range(2)) for j in range(2)) for i in range(2))
    return out


def _act(A, p):
    q = tuple(sum(A[i][k] * p[k] for k in range(2)) for i in range(2))
    return tuple(c - (c.numerator // c.denominator) for c in q)


SITES = [(Fr(0), Fr(0)), (Fr(1, 2), Fr(0)), (Fr(0), Fr(1, 2)), (Fr(1, 2), Fr(1, 2)),
         (Fr(1, 3), Fr(1, 3)), (Fr(2, 3), Fr(2, 3))]


def stabilisers():
    """{site: order of its stabiliser in <R>} and {orbit: stabiliser order}."""
    st = {p: sum(1 for k in range(6) if _act(_mat_pow(R, k), p) == p) for p in SITES}
    orbits = {frozenset(_act(_mat_pow(R, k), p) for k in range(6)): st[p] for p in SITES}
    return st, orbits


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_spectre_orbit():
    us, eras, mult = closed_orbit(3 + S15)
    assert len(us) == 7 and eras == [6, 1]
    assert sp.simplify(mult - LAMBDA2 ** 2) == 0
    M = sp.Matrix([[6, 1], [1, 0]]) * sp.Matrix([[1, 1], [1, 0]])
    x = sp.Symbol("x")
    assert M == sp.Matrix([[7, 6], [1, 1]]) and sp.expand(M.charpoly(x).as_expr() - (x ** 2 - 8 * x + 1)) == 0
    assert sp.simplify(1 + us[0] + us[0] ** 2 - 7 * LAMBDA2) == 0      # a one-off, see the docstring


def test_nu_orbit():
    us, eras, mult = closed_orbit(NU)
    assert len(us) == 4 and eras == [4] and sp.simplify(mult + NU ** 2) == 0
    assert sp.simplify(sp.GoldenRatio ** 3 - NU) == 0


def test_dimension_is_a_ratio_of_multipliers():
    d = dimension_from_orbits()
    assert abs(d - float(sp.log(NU) / sp.log(LAMBDA))) < 1e-12 and abs(d - 1.3993) < 5e-5


def test_every_period_closes():
    for a, b in ((1, 6), (2, 3), (3, 1), (1, 4), (2, 5), (4, 3)):
        _, eras, mult = closed_orbit(periodic_u(a, b))
        assert sorted(eras) == sorted([a, b])
        assert sp.simplify(abs(mult) - unit_of(a, b) ** 2) == 0


def test_kasner_exponents_of_the_orbit():
    us, _, _ = closed_orbit(3 + S15)
    for u in us:
        p = kasner_exponents(u)
        assert sp.simplify(sum(p) - 1) == 0 and sp.simplify(sum(q * q for q in p) - 1) == 0


def test_fibre_clock_is_numerology():
    N1 = sp.N(levels_per_turn(1), 30)
    assert 6.08 < N1 < 6.10 and abs(N1 - 6) > 0.08
    assert sp.simplify(levels_per_turn(a0_for_levels_per_turn(6)) - 6) == 0
    assert abs(sp.N(a0_for_levels_per_turn(6)) - 1) > 0.01


def test_orbifold():
    st, orbits = stabilisers()
    assert sorted(st.values()) == [2, 2, 2, 3, 3, 6] and sorted(orbits.values()) == [2, 3, 6]
    assert all(len(o) * m == 6 for o, m in orbits.items())
    assert 2 - sum(1 - Fr(1, m) for m in orbits.values()) == 0
    assert _mat_pow(R, 6) == ((1, 0), (0, 1)) and _mat_pow(R, 3) == ((-1, 0), (0, -1))


def run_tests(verbose=True):
    """Run every ``test_*`` function in this module; return True if none failed."""
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001 - report and keep going
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-74s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# figure
# --------------------------------------------------------------------------
def show_cosmology(save=None, show=True):
    """Three panels: the fibre clock; the BKL orbit of u* with its Kasner exponents; D from two closed orbits."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 3, figsize=(14, 4))
    n = np.linspace(0, 14, 200)
    for a0 in (0.5, 1.0, 2.0):
        ax[0].plot(n, float(turns_per_level(a0)) * n, label="a0 = %g" % a0)
    ax[0].axhline(1.0, ls=":", c="gray")
    ax[0].axvline(6, ls="--", c="gray")
    ax[0].plot([float(levels_per_turn(1))], [1.0], "ko", ms=4)
    ax[0].set_xlabel("substitution levels n"); ax[0].set_ylabel("fibre turns")
    ax[0].set_title("fibre clock: N(1) = %.3f, not 6" % float(levels_per_turn(1)), fontsize=9)
    ax[0].legend(frameon=False, fontsize=8)

    us, eras, mult = closed_orbit(3 + S15)
    uf = [float(u) for u in us]
    pe = np.array([[float(q) for q in kasner_exponents(u)] for u in us])
    k = np.arange(1, len(us) + 1)
    for i in range(3):
        ax[1].plot(k, pe[:, i], "o-", label="p%d" % (i + 1))
    ax[1].axhline(0, c="gray", lw=0.5)
    ax[1].set_xlabel("epoch (period 7, eras %s)" % eras); ax[1].set_ylabel("Kasner exponent")
    ax[1].set_title("BKL orbit of 3 + sqrt15, multiplier lambda^4", fontsize=9)
    ax[1].legend(frameon=False, fontsize=8)

    _, _, m_nu = closed_orbit(NU)
    lm = [float(sp.log(abs(m_nu))), float(sp.log(mult))]
    ax[2].bar(["ln|m_nu|\n(era 4)", "ln m_lambda\n(eras 6,1)"], lm, color=["C0", "C1"])
    ax[2].set_title("D = 2 ln|m_nu| / ln m_lambda = %.4f" % dimension_from_orbits(), fontsize=9)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=110)
        print("wrote", save)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description="The Spectre's growth rates against the pancake and BKL singularities: self tests and a figure.")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)
    if not args.skip_tests:
        print("spectre_cosmology self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_cosmology(save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
