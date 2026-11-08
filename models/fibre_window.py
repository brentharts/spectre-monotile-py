"""
fibre_window.py - E8 cut-and-project point sets in 4D, and which windows respect the Hopf fibres.

Question (QUANTUM_GRAVITY.md, sections 1 and 3).  The papers build a 4D quasicrystal from E8 and select sites fibre by fibre.
Quantum_Gravity_Lab uses a ball in the perpendicular space of the Coxeter-element H4 plane and says that this is not the
Elser-Sloane window.  Which windows accept or reject a whole fibre (a right-2T orbit of 24 lattice vectors) at a time?

Setting.  E8 = Hurwitz pairs (a, b), a - b in (1 + i)H, covolume 1.  Two splittings R^8 = parallel + perpendicular, each into
two orthogonal 4D spaces:
  'icosian'  the splitting of icosian_e8.py: Phi(x) = sqrt(2/5)(sqrt(3 - phi) x, sqrt(2 + phi) x^sigma), reached from the Hurwitz
             coordinates by the isometry found there.  Perpendicular radius^2 = (2/5)(2 + phi)|x^sigma|^2.
  'coxeter'  the splitting used by Quantum_Gravity_Lab: parallel = Coxeter-element eigenspaces m in {1, 11, 19, 29} (of 30),
             perpendicular = m in {7, 13, 17, 23}.
The point set is  L(R) = { parallel part of v : v in E8, |perpendicular part of v| <= R }  (a ball window).

Exact (tests):
  1. Icosian ball window: every fibre of shells 1..N is accepted or rejected whole (the perpendicular radius is constant on a fibre,
     because right multiplication by a unit quaternion is an isometry of each coordinate quaternion), for every R tested.
  2. The same fibres are *split* by a ball window in the Coxeter perpendicular space for some R (negative control: the property is
     about the splitting, not about any ball).
  3. The parallel image is injective on the lattice (no two lattice vectors share a parallel part): the point set has no repeats.
Numeric (tests, with stated tolerance):
  4. The number of points with |parallel| <= rho and |perpendicular| <= R is close to the volume prediction
     vol(B4)^2 rho^4 R^4 (E8 has covolume 1; vol(B4) = pi^2/2), for both splittings, within 6 per cent at the sizes used (small boxes are off by up to 15 per cent; discreteness).
  5. The two splittings have the same joint distribution of (|par|^2, |perp|^2) over the lattice, so any radial window gives the same
     number of points in either; they differ only in which vectors are chosen, hence in whether fibres stay whole.
Not claimed: that a ball window is the Elser-Sloane window (it is not; theirs is a polytope), that the point set is a model of any
Spectre or Chair44 structure, or anything about spin networks.

    python3 models/fibre_window.py                  tests, then a figure (about 40 s)
    python3 models/fibre_window.py --tests-only
    python3 models/fibre_window.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import e8_shells as es  # noqa: E402
import hopf_shells as hs  # noqa: E402

BALL4 = math.pi ** 2 / 2.0          # volume of the unit 4-ball
_CACHE = {}


def lattice(nmax):
    """All E8 vectors of norm^2 2n, n = 1..nmax (Hurwitz coordinates), with the fibre label of each (labels are global)."""
    if _CACHE.get("nmax", 0) < nmax:
        shells = {n: es.e8_shell(n) for n in range(nmax, 0, -1)}       # largest first: the shell cache is built once
        V = np.vstack([shells[n] for n in range(1, nmax + 1)])
        lab = np.concatenate([es.fibre_labels(shells[n])[0] + 10 ** 6 * n for n in range(1, nmax + 1)])
        _CACHE.update(nmax=nmax, V=V, lab=lab, shell=np.concatenate([[n] * len(shells[n]) for n in range(1, nmax + 1)]))
    keep = _CACHE["shell"] <= nmax
    return _CACHE["V"][keep], _CACHE["lab"][keep]


def coxeter_basis():
    """(par, perp): orthonormal 8x4 bases of the two Coxeter H4 spaces, in the Hurwitz coordinates of hopf_shells."""
    if "cox" not in _CACHE:
        R = hs.e8_roots()
        par, _, _ = hs.coxeter_h4_basis(R, ms=(1, 11, 19, 29))
        perp, _, _ = hs.coxeter_h4_basis(R, ms=(7, 13, 17, 23))
        _CACHE["cox"] = (par, perp)
    return _CACHE["cox"]


def split(V, kind):
    """(parallel, perpendicular) parts of the rows of V, each (N, 4), for kind 'icosian' or 'coxeter'."""
    if kind == "icosian":
        W = V @ es.icosian_isometry()
        return W[:, :4], W[:, 4:]
    if kind == "coxeter":
        par, perp = coxeter_basis()
        return V @ par, V @ perp
    raise ValueError(kind)


def window_set(nmax, R, kind):
    """(points (M, 4), fibre labels (M,)) of L(R) built from the shells 1..nmax (the origin is not included)."""
    V, lab = lattice(nmax)
    par, perp = split(V, kind)
    keep = np.linalg.norm(perp, axis=1) <= R + 1e-9
    return par[keep], lab[keep]


def split_fibres(nmax, R, kind):
    """Number of fibres of shells 1..nmax that have some, but not all, of their 24 vectors inside the window, and the total."""
    V, lab = lattice(nmax)
    _, perp = split(V, kind)
    inside = np.linalg.norm(perp, axis=1) <= R + 1e-9
    u, inv = np.unique(lab, return_inverse=True)
    n_in = np.bincount(inv, weights=inside.astype(float))
    n_all = np.bincount(inv)
    return int(np.sum((n_in > 0) & (n_in < n_all))), len(u)


def count_box(nmax, rho, R, kind):
    """Number of lattice points (origin excluded) with |parallel| <= rho and |perpendicular| <= R, from shells 1..nmax.
    Complete only if (rho^2 + R^2) / 2 <= nmax."""
    V, _ = lattice(nmax)
    par, perp = split(V, kind)
    return int(np.sum((np.linalg.norm(par, axis=1) <= rho + 1e-9) & (np.linalg.norm(perp, axis=1) <= R + 1e-9)))


def predicted(rho, R):
    return BALL4 ** 2 * rho ** 4 * R ** 4


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True, nmax=8):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    V, lab = lattice(nmax)
    check("shells 1..%d: 240 sigma_3(n) vectors in total, 10 sigma_3(n) fibres per shell" % nmax,
          len(V) == sum(240 * es.sigma3(n) for n in range(1, nmax + 1))
          and len(np.unique(lab)) == sum(10 * es.sigma3(n) for n in range(1, nmax + 1)),
          "(%d vectors, %d fibres)" % (len(V), len(np.unique(lab))))
    Rs = [0.5, 0.8, 1.0, 1.2, 1.5, 1.8, 2.0]
    ico = [split_fibres(nmax, r, "icosian")[0] for r in Rs]
    check("icosian ball window: no fibre is split, for R = %s" % Rs, all(c == 0 for c in ico))
    cox = [split_fibres(nmax, r, "coxeter")[0] for r in Rs]
    check("negative control: the Coxeter ball window splits fibres for some R", max(cox) > 0,
          "(split fibres per R: %s)" % cox)
    par, _ = split(V, "icosian")
    check("injective: the parallel images of the lattice vectors are all distinct",
          len(np.unique(np.round(par, 7), axis=0)) == len(V))
    pc, _ = split(V, "coxeter")
    check("... also for the Coxeter splitting", len(np.unique(np.round(pc, 7), axis=0)) == len(V))
    check("the Coxeter bases are orthonormal and mutually orthogonal",
          np.allclose(coxeter_basis()[0].T @ coxeter_basis()[0], np.eye(4), atol=1e-8)
          and np.allclose(coxeter_basis()[1].T @ coxeter_basis()[1], np.eye(4), atol=1e-8)
          and np.allclose(coxeter_basis()[0].T @ coxeter_basis()[1], 0, atol=1e-8))
    for kind in ("icosian", "coxeter"):
        got, exp = 0, 0.0
        for rho, R in [(2.5, 2.5), (2.8, 2.8), (3.4, 1.5), (3.0, 2.0)]:
            assert (rho ** 2 + R ** 2) / 2 <= nmax
            got += count_box(nmax, rho, R, kind) + 1          # +1 for the origin, which is in the lattice
            exp += predicted(rho, R)
        check("%s: points in the box |par| <= rho, |perp| <= R match vol(B4)^2 rho^4 R^4 within 6 per cent" % kind,
              abs(got / exp - 1) < 0.06, "(ratio %.3f)" % (got / exp))
    check("negative control: the prediction with rho^4 R^3 is off by more than 20 per cent",
          abs((count_box(nmax, 2.5, 2.5, "icosian") + 1) / (BALL4 ** 2 * 2.5 ** 4 * 2.5 ** 3) - 1) > 0.2)
    ja = sorted(map(tuple, np.round(np.c_[(split(V, "icosian")[0] ** 2).sum(1), (split(V, "icosian")[1] ** 2).sum(1)], 5)))
    jb = sorted(map(tuple, np.round(np.c_[(split(V, "coxeter")[0] ** 2).sum(1), (split(V, "coxeter")[1] ** 2).sum(1)], 5)))
    check("the two splittings give the same joint distribution of (|par|^2, |perp|^2) over the lattice vectors",
          ja == jb)
    check("negative control: that distribution is not the trivial one (|par|^2 takes many values)",
          len({x for x, _ in ja}) > 50)
    return ok


# ------------------------------------------------------------------------------------------------ figure

def figure(save=None, show=True, nmax=8):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    Rs = np.linspace(0.3, 2.4, 22)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
    for kind, col in (("icosian", "tab:blue"), ("coxeter", "tab:red")):
        frac = [split_fibres(nmax, r, kind)[0] / split_fibres(nmax, r, kind)[1] for r in Rs]
        ax[0].plot(Rs, frac, "o-", color=col, label=kind)
    ax[0].set_xlabel("window radius R (perpendicular ball)")
    ax[0].set_ylabel("fraction of fibres split by the window")
    ax[0].set_title("Shells 1..%d: which ball windows cut a fibre" % nmax)
    ax[0].legend()
    for kind, col in (("icosian", "tab:blue"), ("coxeter", "tab:red")):
        rr = np.linspace(1.0, 3.0, 9)
        rho = 2.6
        ratio = [(count_box(nmax, rho, r, kind) + 1) / predicted(rho, r) for r in rr
                 if (rho ** 2 + r ** 2) / 2 <= nmax]
        ax[1].plot(rr[:len(ratio)], ratio, "o-", color=col, label=kind)
    ax[1].axhline(1, color="k", lw=0.8)
    ax[1].set_xlabel("R"); ax[1].set_ylabel("count / (vol B4)^2 rho^4 R^4")
    ax[1].set_title("Density, rho = 2.6 (curves coincide)")
    ax[1].legend()
    pts, lab = window_set(nmax, 1.0, "icosian")
    near = pts[np.linalg.norm(pts, axis=1) <= 3.5]
    ax[2].scatter(near[:, 0], near[:, 1], s=6, color="tab:blue")
    ax[2].set_aspect("equal")
    ax[2].set_title("L(R), icosian window R = 1: coords 1, 2")
    fig.tight_layout()
    if save:
        os.makedirs(os.path.dirname(save) or ".", exist_ok=True)
        fig.savefig(save, dpi=150)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    if "--skip-tests" not in argv:
        print("tests:")
        if not run_tests():
            print("TESTS FAILED")
            return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(os.path.join(d, "fibre_window.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
