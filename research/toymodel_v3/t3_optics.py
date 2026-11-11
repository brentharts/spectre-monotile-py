#!/usr/bin/env python3
"""t3_optics.py - Stage 1a of research/toymodel_v3/PLAN.md: the wave equation with a refractive index, and what light bending asks of it.

Stage 5 of toymodel_v2 found that the paper's wave equation  psi_tt / (c^2 rho) - Lap psi = S  has speed c sqrt(rho): waves are faster where the density is higher.
The corrected form puts an index n > 1 where the density goes,

        n^2 psi_tt / c^2 - Lap psi = S,      speed c / n,                                           (W')

and the question becomes which n.  A static weak field is described by two factors, a clock factor A (rate of a static clock, squared) and a space factor B
(proper length per coordinate length, squared):  ds^2 = -A dt^2 + B dx^2, so that light has the coordinate speed sqrt(A / B) and the index is n = sqrt(B / A).
The paper's model has a clock (Delta tau = Delta t_0 / rho) and nothing for space, i.e. B = 1.

Definitions
  D1.1  Clock factor A(r) = 1 - r_s / r (the static-observer clock of the companion model, squared).  Space factor B(r) = A(r)^(-gamma); gamma = 0 is the model as
        stated (clock only), gamma = 1 the case A B = 1, gamma = -1 the case "space and time treated alike" (B = A).
  D1.2  Deflection of a ray of impact parameter b in a radial index n(r):  alpha(b) = 2 int_{r0}^inf b dr / (r sqrt(n^2 r^2 - b^2)) - pi,  n(r0) r0 = b  (Bouguer's
        invariant n r sin psi = b of geometric optics, the eikonal limit of W').
  D1.3  Delay of a pulse through an index profile, relative to n = 1:  int (n - 1) dx / c  (1D; the line-of-sight version is the Shapiro delay).

Exact claims (each a test; ``python3 t3_optics.py --tests-only``)
  E1.1  The quadrature of D1.2 is right: n = 1 + a / r gives alpha = 2 a / b + O(1/b^2), and the Schwarzschild index in isotropic coordinates,
        n = (1 + r_s/4r)^3 / (1 - r_s/4r), gives alpha = 2 r_s / b + (15 pi / 16)(r_s / b)^2 + ..., the known second-order value (both coefficients to 1e-3 by
        extrapolation in 1/b).
  E1.2  To first order the deflection is (1 + gamma) r_s / b.  Clock only (gamma = 0, the model as stated): r_s / b, half the value of general relativity.  Space and
        time alike (gamma = -1): zero.  A B = 1 (gamma = 1): 2 r_s / b.  [Negative for the model as stated; the factor 2 needs a space factor B = 1 / A, which the
        model does not contain.]
  E1.3  The index n = 1 / (1 - r_s/r) (gamma = 1 with the clock of D1.1) gives 2 r_s / b at first order and the coefficient 3 pi / 2 at second order, against 15 pi / 16
        for Schwarzschild in isotropic coordinates.  The clock of D1.1 is Schwarzschild's in areal coordinates, so the two second-order numbers belong to different
        radial coordinates and are not compared further.
  E1.4  The corrected equation W' on a lattice (the leapfrog sheet of tm2_waves.py with c^2 rho -> 1 / n^2): energy conserved to 1e-12, reversal retraces to 1e-10,
        group speed equal to the discrete dispersion relation's with speed 1/n (2e-3), and a pulse that crosses a slab of index n arrives LATER than one that does not,
        by the slab's optical excess (n - 1) x width within 3%.  Dense = slow, as the paper's text says.
  E1.5  The delay through the 1/r profile n = 1 + a / |x| (cut off at |x| < x_c) grows like 2 a ln(X / x_c) with the length X of the path: logarithmic, the Shapiro form.

Hypothesis / not done: a reason for B = 1 / A inside the model (the clock-only model gives half the bending, and treating space like time gives none); the second order;
a two-dimensional wave run (beam centroids on the lattice agree with the ray to 10-20% only, because of diffraction and the walls, so it is not used as a test).

    python3 t3_optics.py --tests-only
    python3 t3_optics.py --story
    python3 t3_optics.py --no-show --save figures/t3_optics.png
"""
import math
import os
import sys

import numpy as np
from scipy.integrate import quad
from scipy.optimize import brentq

_HERE = os.path.dirname(os.path.realpath(__file__))
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _V2, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import tm2_waves as tw  # noqa: E402


# -- geometric optics ---------------------------------------------------------------------------------------------
def deflection(nfun, b):
    """alpha(b) of D1.2 for a radial index nfun(r); quadrature with the substitution r = r0 / (1 - t^2)."""
    if b <= 0:
        raise ValueError("b must be positive")
    f = lambda r: nfun(r) * r - b  # noqa: E731
    grid = np.geomspace(1e-6 * b, 10.0 * b + 10.0, 4000)
    vals = np.array([f(r) for r in grid])
    sign = np.flatnonzero((vals[:-1] < 0) & (vals[1:] >= 0))
    if len(sign) == 0:
        raise ValueError("no turning point: the ray is captured or n r never reaches b")
    i = sign[-1]  # the outermost turning point
    r0 = brentq(f, grid[i], grid[i + 1], xtol=1e-14, rtol=1e-15)

    def g(t):
        u = 1.0 - t * t
        if u <= 0.0:
            return 0.0
        r = r0 / u
        val = (nfun(r) * r) ** 2 - b * b
        if val <= 0.0:
            return 0.0
        return 2.0 * t * b * r0 / (u * u * r * math.sqrt(val))

    integral, _ = quad(g, 0.0, 1.0, limit=400, epsabs=1e-13, epsrel=1e-12)
    return 2.0 * integral - math.pi


def index_from(gamma, r_s=1.0):
    """n(r) = sqrt(B / A) with A = 1 - r_s/r and B = A^(-gamma) (D1.1)."""
    def n(r):
        if r <= r_s * (1.0 + 1e-12):   # inside the horizon of A: the ray never gets there; a large value keeps the root search inside r > r_s
            return 1e12 / r
        return (1.0 - r_s / r) ** (-(1.0 + gamma) / 2.0)
    return n


def n_isotropic(r_s=1.0):
    return lambda r: (1.0 + r_s / (4.0 * r)) ** 3 / (1.0 - r_s / (4.0 * r))


def coefficients(nfun, bs=(400.0, 800.0, 1600.0, 3200.0)):
    """(first, second) order coefficients of alpha(b) = c1 / b + c2 / b^2 + c3 / b^3, by a least-squares fit in 1/b (r_s = 1)."""
    x = np.array([1.0 / b for b in bs])
    y = np.array([deflection(nfun, b) for b in bs])
    M = np.vstack([x, x ** 2, x ** 3]).T
    c = np.linalg.lstsq(M, y, rcond=None)[0]
    return float(c[0]), float(c[1])


# -- the corrected wave equation on a lattice -----------------------------------------------------------------------
def sheet(psi0, psi1, n, dt=0.4):
    """W' as the leapfrog sheet of tm2_waves with c^2 rho = 1 / n^2."""
    return tw.Sheet(psi0, psi1, 1.0 / np.asarray(n, dtype=float) ** 2, dt)


def slab_delay(n_slab=1.5, width=200, k=0.3, dt=0.4, length=4096):
    """(measured lag of the centroid of a pulse that crossed the slab, predicted optical excess (n - 1) width)."""
    nn = np.ones(length)
    nn[1200:1200 + width] = n_slab
    a, b = tw.pulse(length, 700.0, k, 50.0, dt, 1.0)
    free = sheet(a, b, np.ones(length), dt)
    slab = sheet(a, b, nn, dt)
    steps = int(1500 / dt)
    free.step(steps)
    slab.step(steps)
    ef, es = tw.energy_density(free), tw.energy_density(slab)
    right = np.arange(length) >= 1200 + width
    lag = tw.centroid(ef) - tw.centroid(np.where(right, es, 0.0))
    # the group index of the lattice inside the slab: the pulse's group speed there over the free one
    om = tw.dispersion(k, dt, 1.0)
    k2 = tw.wavenumber(om, dt, 1.0 / n_slab)
    vg_free, vg_slab = tw.group_speed(k, dt, 1.0), tw.group_speed(k2, dt, 1.0 / n_slab)
    predicted = width * (vg_free / vg_slab - 1.0)
    return lag, predicted, (n_slab - 1.0) * width


def optical_excess(a, X, x_c):
    """int_{-X}^{X} (n - 1) dx for n = 1 + a / max(|x|, x_c)."""
    return 2.0 * (a + a * math.log(X / x_c))


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    except Exception:  # noqa: BLE001
        return False
    return False


def test_e11_the_quadrature_reproduces_the_known_coefficients():
    c1, c2 = coefficients(lambda r: 1.0 + 1.0 / r)
    assert math.isclose(c1, 2.0, abs_tol=1e-3), c1
    c1, c2 = coefficients(n_isotropic())
    assert math.isclose(c1, 2.0, abs_tol=1e-3) and math.isclose(c2, 15.0 * math.pi / 16.0, rel_tol=1e-3), (c1, c2)
    assert abs(deflection(lambda r: 1.0, 10.0)) < 1e-10, "no index, no bending"
    assert _raises(ValueError, deflection, lambda r: 1.0, -1.0)


def test_e12_first_order_is_one_plus_gamma():
    for gamma, want in ((0.0, 1.0), (1.0, 2.0), (-1.0, 0.0), (0.5, 1.5)):
        c1, _ = coefficients(index_from(gamma))
        assert math.isclose(c1, want, abs_tol=2e-3), (gamma, c1)
    # the model as stated: clock only -> half of 2 r_s / b at large b
    for b in (200.0, 1000.0):
        assert math.isclose(deflection(index_from(0.0), b) * b, 1.0, rel_tol=0.02)
        assert math.isclose(deflection(index_from(1.0), b) * b, 2.0, rel_tol=0.03)
        assert abs(deflection(index_from(-1.0), b)) < 1e-10


def test_e13_second_order_of_the_chain_index():
    c1, c2 = coefficients(index_from(1.0))
    assert math.isclose(c1, 2.0, abs_tol=2e-3) and math.isclose(c2, 1.5 * math.pi, rel_tol=2e-3), (c1, c2)
    assert not math.isclose(c2, 15.0 * math.pi / 16.0, rel_tol=0.1)


def test_e14_the_corrected_equation_is_slow_where_the_index_is_high():
    rng = np.random.default_rng(1)
    for shape in ((128,), (24, 24), (10, 10, 10)):
        n = 1.0 + rng.uniform(0.0, 0.8, size=shape)
        p0 = rng.normal(size=shape)
        s = sheet(p0, p0 + 0.1 * rng.normal(size=shape), n, 0.4)
        e0 = s.energy()
        es = [s.step(40).energy() for _ in range(5)]
        assert max(abs(e - e0) for e in es) < 1e-12 * max(1.0, abs(e0)) * 50
        q0, q1 = s.prev.copy(), s.cur.copy()
        s.step(150).reverse().step(150)
        assert np.abs(s.prev - q0).max() < 1e-10 or np.abs(s.cur - q0).max() < 1e-10
    lag, predicted, excess = slab_delay()
    assert lag > 0.0 and math.isclose(lag, predicted, rel_tol=0.03), (lag, predicted)
    assert math.isclose(lag, excess, rel_tol=0.06), (lag, excess)
    for n_ in (1.0, 2.0):
        assert math.isclose(tw.group_speed(0.4, 0.4, 1.0 / n_), math.cos(0.2) / n_ / math.sqrt(1 - (0.4 / n_ * math.sin(0.2)) ** 2), rel_tol=1e-12)


def test_e15_the_delay_through_a_one_over_r_profile_is_logarithmic():
    a, x_c = 0.5, 4.0
    xs = np.linspace(-4000.0, 4000.0, 800001)
    for X in (500.0, 1000.0, 2000.0, 4000.0):
        m = np.abs(xs) <= X
        num = np.trapezoid(a / np.maximum(np.abs(xs[m]), x_c), xs[m])
        assert math.isclose(num, optical_excess(a, X, x_c), rel_tol=1e-6)
    d = [optical_excess(a, X, x_c) for X in (500.0, 1000.0, 2000.0)]
    assert math.isclose(d[1] - d[0], 2.0 * a * math.log(2.0), rel_tol=1e-12) and math.isclose(d[2] - d[1], d[1] - d[0], rel_tol=1e-12)


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
    print("deflection coefficients alpha = c1 r_s/b + c2 (r_s/b)^2 (A = 1 - r_s/r, B = A^-gamma)")
    for lab, nf in (("clock only (gamma = 0, the model as stated)", index_from(0.0)), ("space like time (gamma = -1)", index_from(-1.0)),
                    ("A B = 1 (gamma = 1)", index_from(1.0)), ("Schwarzschild, isotropic coordinates", n_isotropic())):
        c1, c2 = coefficients(nf)
        print("  %-45s c1 = %.4f  c2 = %.4f" % (lab, c1, c2))
    print("  (15 pi / 16 = %.4f, 3 pi / 2 = %.4f)" % (15 * math.pi / 16, 1.5 * math.pi))
    lag, pred, exc = slab_delay()
    print("slab n = 1.5, width 200: pulse lag %.2f, lattice group-index prediction %.2f, optical excess (n - 1) width %.1f" % (lag, pred, exc))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    bs = np.geomspace(20, 2000, 30)
    for lab, nf in (("clock only (model as stated)", index_from(0.0)), ("A B = 1", index_from(1.0)), ("space like time", index_from(-1.0)),
                    ("Schwarzschild (isotropic)", n_isotropic())):
        ax[0].semilogx(bs, [deflection(nf, b) * b for b in bs], label=lab)
    ax[0].axhline(2.0, color="k", lw=0.6, ls="--")
    ax[0].set_xlabel("impact parameter b / r_s")
    ax[0].set_ylabel("deflection x b / r_s")
    ax[0].set_title("light bending needs a space factor")
    ax[0].legend(fontsize=8)
    n, dt, k = 4096, 0.4, 0.3
    nn = np.ones(n)
    nn[1200:1400] = 1.5
    a, b = tw.pulse(n, 700.0, k, 50.0, dt, 1.0)
    f = sheet(a, b, np.ones(n), dt).step(int(1500 / dt))
    s = sheet(a, b, nn, dt).step(int(1500 / dt))
    ax[1].plot(tw.energy_density(f), label="n = 1")
    ax[1].plot(tw.energy_density(s), label="slab n = 1.5 at 1200..1400")
    ax[1].axvspan(1200, 1400, color="0.9")
    ax[1].set_xlim(1800, 2600)
    ax[1].set_xlabel("x")
    ax[1].set_title("corrected equation: dense = slow")
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
