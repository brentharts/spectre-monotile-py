#!/usr/bin/env python3
"""t7_fields.py - Stage 2 of toymodel_v7: the number fields.  Does one structure carry the Eisenstein side (the Spectre's lift, sqrt -3, 12-fold), the icosian side (E8 in Z[phi], sqrt 5) and the inflation?

The paper (Result res:field): the inflation factor lambda generates Q(lambda) = Q(sqrt6, sqrt10), which contains sqrt15 but not sqrt3, sqrt5 or phi; the icosian E8 lives in Z[phi]; the lift is Eisenstein.  Exact checks
in SymPy (rational matrices, factorisation over number fields).

Definitions
  D7.5  E8 with the simple roots of Bourbaki in R^8; w = s_1 ... s_8 the Coxeter element (product of the simple reflections, rational 8 x 8 matrix).
  D7.6  The Eisenstein structure: omega = w^10.  The golden structure: g = w^6 + w^-6.

Exact claims (each a test; ``python3 t7_fields.py --tests-only``)
  E7.5  One automorphism carries both.  The Cartan matrix has determinant 1 (E8 unimodular).  w has order 30 and characteristic polynomial Phi_30, so E8 is a module over Z[zeta_30] of rank one, with no fixed vector.
        omega = w^10 satisfies omega^2 + omega + 1 = 0 and has no fixed vector (an Eisenstein structure, E8 = Z[omega]^4); g = w^6 + w^-6 satisfies g^2 + g - 1 = 0 (a golden structure, E8 a Z[phi]-module of rank 4, phi = -g or
        1 + g).  They commute.  So E8 is at the same time an Eisenstein lattice and an icosian-type lattice, through one element of its Weyl group.
  E7.6  The inflation does not join them.  Phi_30 stays irreducible over Q(sqrt6, sqrt10) (and over Q(sqrt15)), so Q(lambda) and Q(zeta_30) meet only in Q: no element of Z[zeta_30] acting on E8 is the inflation.
        **Negative**.
  E7.7  The inflation squared is made of the two sides.  lambda = (sqrt6 + sqrt10) / 2 has minimal polynomial x^4 - 8x^2 + 1; lambda sqrt2 = sqrt3 + sqrt5, so lambda^2 = 4 + sqrt3 sqrt5: the Spectre's 12-fold irrationality times the
        icosian 5-fold one.  4 + sqrt15 is the fundamental unit of Z[sqrt15] (the least solution of x^2 - 15 y^2 = 1 with y > 0 is (4, 1)).
  E7.8  Rank.  A lattice on which a fixed-point-free 12-fold rotation (Spectre, eigenvalues primitive 12th roots) and a fixed-point-free 5-fold one (icosian) act and commute is a module over Z[zeta_60]; its rank is a multiple of
        phi(60) = 16.  E8 (rank 8) cannot carry both at once; the smallest candidate rank is 16, and Q(zeta_60) contains sqrt15 (Phi_60 splits over Q(sqrt15)), so lambda^2 acts there.  **Exact**, and a pointer: rank 16, not 8.

Status.  All Exact.  Which rank-16 lattice (E8 + E8, Barnes-Wall, the trace form of Z[zeta_60]) is the relevant one is not decided here (Hypothesis H7.3 of PLAN.md, revised).

    python3 t7_fields.py --tests-only
    python3 t7_fields.py --no-show --save figures/t7_stage2_fields.png
"""
import argparse
import sys

import sympy as sp

X = sp.symbols("x")
_C = {}


def simple_roots():
    h = sp.Rational(1, 2)
    e = lambda i: sp.Matrix([1 if j == i else 0 for j in range(8)])  # noqa: E731
    return [sp.Matrix([h, -h, -h, -h, -h, -h, -h, h]), e(0) + e(1), e(1) - e(0), e(2) - e(1), e(3) - e(2), e(4) - e(3), e(5) - e(4), e(6) - e(5)]


def coxeter():
    if "w" not in _C:
        I = sp.eye(8)
        w = I
        for a in simple_roots():
            w = w * (I - 2 * a * a.T / (a.T * a)[0])
        _C["w"] = w
    return _C["w"]


def order(M, cap=200):
    P = sp.eye(M.shape[0])
    for k in range(1, cap + 1):
        P = P * M
        if P == sp.eye(M.shape[0]):
            return k
    return None


LAMBDA = (sp.sqrt(6) + sp.sqrt(10)) / 2


# --------------------------------------------------------------------------
def test_e75_one_automorphism_carries_both():
    S = simple_roots()
    C = sp.Matrix(8, 8, lambda i, j: (S[i].T * S[j])[0])
    assert C.det() == 1
    w = coxeter()
    I = sp.eye(8)
    assert order(w) == 30
    assert sp.expand(w.charpoly(X).as_expr() - sp.cyclotomic_poly(30, X)) == 0
    assert (w - I).det() != 0
    om = w ** 10
    assert om ** 2 + om + I == sp.zeros(8) and (om - I).det() != 0
    g = w ** 6 + (w ** 6).inv()
    assert g ** 2 + g - I == sp.zeros(8)
    assert om * g == g * om
    return "order 30, Phi_30; w^10 Eisenstein, w^6 + w^-6 golden, commuting"


def test_e76_the_inflation_does_not_join_them():
    for ext in ([sp.sqrt(6), sp.sqrt(10)], [sp.sqrt(15)]):
        _, fac = sp.factor_list(sp.cyclotomic_poly(30, X), extension=ext)
        assert len(fac) == 1 and sp.degree(fac[0][0], X) == 8
    return "Phi_30 irreducible over Q(sqrt6, sqrt10)"


def test_e77_the_inflation_squared_is_made_of_the_two_sides():
    assert sp.expand(sp.minimal_polynomial(LAMBDA, X) - (X ** 4 - 8 * X ** 2 + 1)) == 0
    assert sp.simplify(LAMBDA * sp.sqrt(2) - (sp.sqrt(3) + sp.sqrt(5))) == 0
    assert sp.simplify(LAMBDA ** 2 - (4 + sp.sqrt(3) * sp.sqrt(5))) == 0
    sols = [(x, y) for y in range(1, 10) for x in range(1, 100) if x * x - 15 * y * y == 1]
    assert min(sols, key=lambda s: s[1]) == (4, 1)
    return "lambda sqrt2 = sqrt3 + sqrt5; 4 + sqrt15 fundamental unit"


def test_e78_both_rotations_need_rank_sixteen():
    assert sp.totient(60) == 16 and sp.totient(12) == 4 and sp.totient(5) == 4
    _, fac = sp.factor_list(sp.cyclotomic_poly(60, X), extension=sp.sqrt(15))
    assert len(fac) == 2                                  # sqrt15 lies in Q(zeta_60)
    _, fac30 = sp.factor_list(sp.cyclotomic_poly(30, X), extension=sp.sqrt(15))
    assert len(fac30) == 1                                # but not in Q(zeta_30)
    return 16


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
    import numpy as np

    w = np.array(coxeter().evalf(), dtype=float)
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.6))
    t = np.linspace(0, 2 * np.pi, 400)
    for a, (M, name, col) in zip(ax[:2], ((w, "w (order 30)", "k"), (np.linalg.matrix_power(w, 10), "w$^{10}$ (Eisenstein, circles) and w$^6$ (5-fold, squares)", "tab:blue"))):
        ev = np.linalg.eigvals(M)
        a.plot(np.cos(t), np.sin(t), color="0.8")
        a.scatter(ev.real, ev.imag, color=col, s=40, zorder=3)
        a.set_aspect("equal"); a.set_title("eigenvalues of " + name, fontsize=9)
    ev6 = np.linalg.eigvals(np.linalg.matrix_power(w, 6))
    ax[1].scatter(ev6.real, ev6.imag, color="tab:orange", s=25, marker="s", zorder=3, label="w$^6$ (5-fold: golden)")
    ax[1].legend(fontsize=7, loc="lower left")
    ax[2].axis("off")
    txt = ("E8 with its Coxeter element w:\n"
           "  order 30, charpoly Phi_30\n"
           "  w^10: omega^2 + omega + 1 = 0   (Eisenstein, sqrt -3)\n"
           "  w^6 + w^-6: g^2 + g - 1 = 0     (golden, sqrt 5)\n"
           "  -> both structures on one lattice, commuting\n\n"
           "the inflation lambda = (sqrt6 + sqrt10)/2:\n"
           "  Q(lambda) meets Q(zeta_30) only in Q   (Negative)\n"
           "  lambda sqrt2 = sqrt3 + sqrt5\n"
           "  lambda^2 = 4 + sqrt3 sqrt5, the unit of Z[sqrt15]\n\n"
           "12-fold (Spectre) and 5-fold (icosian), commuting\n"
           "and fixed-point free: rank a multiple of\n"
           "phi(60) = 16; sqrt15 lies in Q(zeta_60).")
    ax[2].text(0.0, 1.0, txt, va="top", family="monospace", fontsize=8.5)
    fig.suptitle("toymodel_v7 Stage 2: one lattice for the Eisenstein and icosian structures, and where the inflation sits")
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
