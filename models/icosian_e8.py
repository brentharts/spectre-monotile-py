"""
icosian_e8.py - E8 from the icosians, and how it compares with the Hurwitz construction of hopf_shells.py.

hopf_shells.py built the 240 roots of E8 as pairs of Hurwitz quaternions and found a 4D projection that respects the
24-point Hopf fibres.  Elser and Sloane (and the Sadoc-Mosseri paper behind QUANTUM_GRAVITY.md) use another description of
E8: the icosian ring, a Z[phi]-module of quaternions, mapped into R^4 + R^4 by x -> (x, x^sigma) where sigma is the
Galois conjugation phi -> 1 - phi = -1/phi.  This file builds that E8 from scratch and compares the two.

Construction (derived here and checked by the tests below, not quoted):

  With the form N(x) = Tr_{Q(sqrt5)/Q}( (3 - phi) |x|^2 ) = (3 - phi) |x|^2 + (2 + phi) |x^sigma|^2 the icosians
  u in 2I (|u|^2 = 1) and phi u (|phi u|^2 = phi^2) both have N = 5, because Tr((3 - phi)(phi^2 - 1)) = 0.  The 240 vectors

        Phi(x) = sqrt(2/5) ( sqrt(3 - phi) x , sqrt(2 + phi) x^sigma ),   x in 2I  or  x in phi 2I,

  have norm 2, integer inner products and are closed under their own reflections with 8 simple roots of Cartan
  determinant 1: they are the roots of E8.  The *other* natural guess, phi u' with u' the conjugate group sigma(2I), is
  not a root system (tested as a negative control).

Results (each is a test):
  1. The icosian roots are an E8 root system.  Right multiplication by 2T acts as (a, b) -> (a l, b l) on them (the
     Hurwitz units are fixed by sigma, l^sigma = l), so the 2T-orbits are Hopf fibres of S^7 -> S^4 exactly as in the
     Hurwitz construction: 10 fibres of 24, each a 24-cell, with the cross-polytope as base points.
  2. The orthogonal projection to the first factor R^4 (the Elser-Sloane-type "parallel space") puts the roots on two
     shells of 120, radii in ratio phi, each a rotated copy of 2I, and each shell is 5 whole fibres.  The second factor
     ("perpendicular space") does the same with the shells swapped (copies of the conjugate group sigma(2I), tested by their
     inner products only): parallel and perpendicular radii lie on the circle r_par^2 + r_perp^2 = 2, in two clusters.
  3. The two constructions are the same fibred root system: an isometry of R^8 exists that maps the Hurwitz roots onto the
     icosian roots and every Hurwitz fibre onto an icosian fibre (found by backtracking and then checked on all 240
     roots).  Under it the icosian parallel plane is exactly one of the 32 fibre-compatible planes a + q b of
     hopf_shells.py (all principal angles 0), with |q| = 1/phi, and no other of the 32 coincides with it; the perpendicular plane is
     the one with |q| = phi.
  4. So for E8 the icosian projection and the Hurwitz fibre-compatible projection do not differ: the "fibre-respecting"
     projection of hopf_shells.py is the icosian one in other coordinates.

What this does not show: that the papers' quasicrystal (the shell-by-shell selection of whole fibres) was reproduced; the
selection rule itself is not built here.  The isometry is one found by search; others differ from it by automorphisms of
the fibred root system, which is why the plane is one of the 32 and not all of them.

    python3 models/icosian_e8.py                 tests, then a figure
    python3 models/icosian_e8.py --tests-only
    python3 models/icosian_e8.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hopf_shells as hs  # noqa: E402
from hopf_shells import PHI, _key, hurwitz_units, icosians, qmul  # noqa: E402

A_PAR = math.sqrt(3.0 - PHI)       # weight of the parallel factor
A_PERP = math.sqrt(2.0 + PHI)      # weight of the perpendicular factor
PHI_CONJ = 1.0 - PHI               # sigma(phi) = -1 / phi


# ------------------------------------------------------------------------------------------------ the conjugation

def sigma_value(v):
    """sigma on one icosian coordinate: the values 0, +/-1/2, +/-1 are fixed, phi/2 <-> -1/(2 phi) (sign kept on the
    magnitude), 1/(2 phi) <-> -phi/2."""
    m, s = abs(v), (1.0 if v >= 0 else -1.0)
    if abs(m - PHI / 2) < 1e-9:
        return s * (-1.0 / (2 * PHI))
    if abs(m - 1 / (2 * PHI)) < 1e-9:
        return s * (-PHI / 2)
    if min(abs(m), abs(m - 0.5), abs(m - 1.0)) < 1e-9:
        return v
    raise ValueError("not an icosian coordinate: %r" % (v,))


def sigma(x):
    """Galois conjugate of icosian(s) x (arrays (..., 4))."""
    x = np.asarray(x, float)
    return np.vectorize(sigma_value)(x)


# ------------------------------------------------------------------------------------------------ E8 from the icosians

def phi_embedding(x, xs):
    """Phi(x) = sqrt(2/5) (sqrt(3 - phi) x, sqrt(2 + phi) x^sigma): rows (n, 8)."""
    return math.sqrt(2.0 / 5.0) * np.hstack([A_PAR * np.asarray(x), A_PERP * np.asarray(xs)])


def icosian_roots(variant="phi"):
    """The 240 vectors Phi(u), Phi(phi u), u in 2I (shape (240, 8), the first 120 are the units, the rest are phi times them).

    variant 'conjugate' builds the wrong guess Phi(phi w^sigma) instead, for the negative control."""
    I = icosians()
    Is = sigma(I)
    units = phi_embedding(I, Is)
    if variant == "phi":
        scaled = phi_embedding(PHI * I, PHI_CONJ * Is)            # (phi u)^sigma = phi^sigma u^sigma
    elif variant == "conjugate":
        scaled = phi_embedding(PHI * Is, PHI_CONJ * I)            # phi w^sigma, with (phi w^sigma)^sigma = phi' w
    else:
        raise ValueError(variant)
    return np.vstack([units, scaled])


def is_e8_root_system(R, tol=1e-6):
    """240 distinct norm-2 vectors with integer inner products, closed under reflection in each other, with 8 simple roots
    of Cartan determinant 1."""
    R = np.asarray(R, float)
    if R.shape != (240, 8) or len({_key(r) for r in R}) != 240 or np.abs((R ** 2).sum(1) - 2).max() > tol:
        return False
    G = R @ R.T
    if np.abs(G - np.round(G)).max() > tol:
        return False
    keys = {_key(r) for r in R}
    if not all(_key(b - (a @ b) * a) in keys for a in R for b in R[::3]):
        return False
    S = hs.e8_simple_roots(R)
    C = np.array([[2 * a @ b / (a @ a) for b in S] for a in S])
    return len(S) == 8 and round(np.linalg.det(C)) == 1


def right_fibres(R):
    """Labels of the right 2T-orbits (a, b) -> (a l, b l) of the roots, and their number."""
    T = hurwitz_units()
    idx = {_key(r): i for i, r in enumerate(R)}
    lab = -np.ones(len(R), int)
    n = 0
    for i, r in enumerate(R):
        if lab[i] >= 0:
            continue
        for t in T:
            lab[idx[_key(np.r_[qmul(r[:4], t), qmul(r[4:], t)])]] = n
        n += 1
    return lab, n


# ------------------------------------------------------------------------------------------------ comparison

def find_isometry(Ra, la, Rb, lb, seed=1, extra=10):
    """An orthogonal g (8, 8), acting as x -> x @ g, that maps the roots Ra onto the roots Rb and sends fibres to fibres
    (la, lb: fibre labels), or None.  Backtracking over images of 8 independent roots plus ``extra`` more, matching all inner
    products and the 'same fibre' relation, then checked on every root."""
    Ga, Gb = Ra @ Ra.T, Rb @ Rb.T
    basis = []
    for i in range(len(Ra)):
        if np.linalg.matrix_rank(Ra[basis + [i]]) == len(basis) + 1:
            basis.append(i)
        if len(basis) == 8:
            break
    rest = [i for i in range(len(Ra)) if i not in basis]
    order = basis + list(np.random.default_rng(seed).choice(rest, extra, replace=False))
    kb = {_key(r): i for i, r in enumerate(Rb)}
    norm_ok = np.abs(np.diag(Gb) - 2) < 1e-6

    def verify(assign):
        g = np.linalg.solve(Ra[order[:8]], Rb[assign[:8]])
        if np.abs(g @ g.T - np.eye(8)).max() > 1e-8:
            return None
        try:
            mapped = np.array([kb[_key(r)] for r in Ra @ g])
        except KeyError:
            return None
        if all(len(set(lb[mapped[la == f]])) == 1 for f in set(la)):
            return g
        return None

    def dfs(k, assign):
        if k == len(order):
            return verify(assign)
        mask = norm_ok.copy()
        for j in range(k):
            same = la[order[k]] == la[order[j]]
            mask &= (np.abs(Gb[:, assign[j]] - Ga[order[k], order[j]]) < 1e-6) & ((lb == lb[assign[j]]) == same)
        for c in np.flatnonzero(mask):
            g = dfs(k + 1, assign + [int(c)])
            if g is not None:
                return g
        return None

    return dfs(0, [])


def plane_cosines(g, cols=slice(0, 4)):
    """Smallest and all cosines of the principal angles between (a) the plane spanned by the columns ``cols`` of the isometry g
    (cols 0:4 = the icosian parallel factor pulled back to the Hurwitz coordinates, 4:8 = the perpendicular factor) and
    (b) each of the 32 fibre-compatible planes a + q b of hopf_shells.  Returns (qs, cosines (32, 4))."""
    Q, _ = np.linalg.qr(g[:, cols])
    out, qs = [], []
    for q, _ in hs.fibre_compatible_q():
        Fq, _ = np.linalg.qr(hs._f_matrix(q).T)
        out.append(np.linalg.svd(Q.T @ Fq, compute_uv=False))
        qs.append(q)
    return qs, np.array(out)


def matching_q(g, cols=slice(0, 4)):
    """The q's (among the 32) whose plane coincides with the plane of g[:, cols]."""
    qs, cs = plane_cosines(g, cols)
    return [q for q, c in zip(qs, cs) if c.min() > 1 - 1e-9]


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    I = icosians()
    Is = sigma(I)
    ico = {_key(v) for v in I}
    check("sigma maps 2I to a group of unit quaternions and is multiplicative (all 120 x 120 products)",
          np.allclose((Is ** 2).sum(1), 1) and all(np.allclose(qmul(Is[i], Is[j]), sigma(qmul(I[i], I[j]))) for i in range(0, 120, 3) for j in range(120)))
    T = hurwitz_units()
    check("sigma fixes the 24 Hurwitz units (so right multiplication by 2T acts the same way on both factors)",
          np.allclose(sigma(T), T) and all(_key(t) in ico for t in T))
    check("sigma is not the identity on 2I (the conjugate copy differs)", not np.allclose(Is, I))
    check("(3 - phi) phi^2 + (2 + phi) phi'^2 = 5 and (3 - phi) + (2 + phi) = 5: both classes have the same norm",
          abs(A_PAR ** 2 * PHI ** 2 + A_PERP ** 2 * PHI_CONJ ** 2 - 5) < 1e-12 and abs(A_PAR ** 2 + A_PERP ** 2 - 5) < 1e-12)

    R = icosian_roots()
    check("the 240 vectors {Phi(u), Phi(phi u)} are an E8 root system (norm 2, integral, closed, Cartan det 1)", is_e8_root_system(R))
    check("negative control: Phi(phi w^sigma) in place of Phi(phi u) is not a root system", not is_e8_root_system(icosian_roots("conjugate")))
    lab, nf = right_fibres(R)
    check("right 2T orbits: 10 fibres of 24, each a 24-cell", nf == 10 and list(np.bincount(lab)) == [24] * 10 and all(hs.is_24cell(R[lab == f]) for f in range(nf)))
    Hs = hs.hopf_s4(R)
    spread = max(np.ptp(Hs[lab == f], axis=0).max() for f in range(nf))
    B = np.array([Hs[lab == f][0] for f in range(nf)])
    Bn = B / np.linalg.norm(B, axis=1)[:, None]
    Gb = np.round(Bn @ Bn.T, 6) + 0.0
    check("the Hopf map S^7 -> S^4 is constant on each fibre and the base points are the 5-cross-polytope",
          spread < 1e-9 and set(Gb.ravel()) == {-1.0, 0.0, 1.0} and all(int((r == -1).sum()) == 1 for r in Gb))

    par, perp = R[:, :4], R[:, 4:]
    rp = np.round((par ** 2).sum(1), 6)
    lv = sorted(set(rp))
    check("parallel factor: two shells of 120, radii in ratio phi, each a rotated copy of 2I",
          len(lv) == 2 and [int((rp == x).sum()) for x in lv] == [120, 120] and abs(math.sqrt(lv[1] / lv[0]) - PHI) < 1e-5
          and all(hs.is_600cell_shell(par[rp == x]) for x in lv))
    check("each parallel shell is 5 whole fibres (units: 5 fibres, phi-multiples: the other 5)",
          all(len(set(rp[lab == f])) == 1 for f in range(nf)) and [len(set(lab[rp == x])) for x in lv] == [5, 5])
    rq = np.round((perp ** 2).sum(1), 6)
    lq = sorted(set(rq))
    check("perpendicular factor: two shells of 120 in ratio phi with the 600-cell's inner products (they are copies of sigma(2I), "
          "which is not a rotated copy of 2I), the inner/outer roles swapped",
          len(lq) == 2 and abs(math.sqrt(lq[1] / lq[0]) - PHI) < 1e-5 and all(hs._spectrum_only(perp[rq == x]) for x in lq)
          and all(((rp == lv[0]) == (rq == lq[1]))[i] for i in range(240)))
    check("parallel and perpendicular radii lie on r_par^2 + r_perp^2 = 2", np.allclose((par ** 2).sum(1) + (perp ** 2).sum(1), 2))

    Rh = hs.e8_roots()
    lh, nh = hs.e8_fibres(Rh)
    g = find_isometry(Rh, lh, R, lab)
    check("an isometry of R^8 maps the Hurwitz roots onto the icosian roots and every Hurwitz fibre onto an icosian fibre",
          g is not None and np.abs(g @ g.T - np.eye(8)).max() < 1e-8)
    if g is not None:
        qs_, cs = plane_cosines(g)
        mins = np.sort(cs.min(axis=1))
        check("... under it the icosian parallel plane is exactly one of the 32 fibre-compatible planes (all four principal cosines 1); "
              "no other of the 32 coincides with it",
              abs(mins[-1] - 1) < 1e-9 and int((mins > 1 - 1e-9).sum()) == 1,
              "(%s)" % np.round(sorted(set(np.round(mins, 4))), 4))
        qp, qq = matching_q(g, slice(0, 4)), matching_q(g, slice(4, 8))
        check("... and the perpendicular plane is also exactly one of the 32: |q| = 1/phi for the parallel plane, |q| = phi for the "
              "perpendicular one",
              len(qp) == 1 and len(qq) == 1 and abs(np.linalg.norm(qp[0]) - 1 / PHI) < 1e-9 and abs(np.linalg.norm(qq[0]) - PHI) < 1e-9)
        T_dev = max(np.abs(_rmul(t) @ _proj(g) @ _rmul(t).T - _proj(g)).max() for t in T)
        check("... and that plane is invariant under right multiplication by 2T (it is a quaternionic line)", T_dev < 1e-9, "(%.0e)" % T_dev)
    check("negative control: an orthogonal map that is not an isometry of the root systems is refused by the search",
          find_isometry(Rh, lh, R * np.r_[np.ones(4), 1.1 * np.ones(4)] , lab) is None)
    return ok


def _rmul(l):
    A = np.zeros((8, 8))
    for k in range(4):
        e = np.zeros(4)
        e[k] = 1.0
        A[k, :4] = qmul(e, l)
        A[4 + k, 4:] = qmul(e, l)
    return A


def _proj(g):
    Q, _ = np.linalg.qr(g[:, :4])
    return Q @ Q.T


# ------------------------------------------------------------------------------------------------ figure

def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    R = icosian_roots()
    lab, nf = right_fibres(R)
    rpar = np.sqrt((R[:, :4] ** 2).sum(1))
    rperp = np.sqrt((R[:, 4:] ** 2).sum(1))
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    th = np.linspace(0, math.pi / 2, 100)
    ax[0].plot(math.sqrt(2) * np.cos(th), math.sqrt(2) * np.sin(th), color="0.7")
    for f in range(nf):
        m = lab == f
        ax[0].scatter(rpar[m], rperp[m], s=30 + 10 * f, alpha=0.5, color=plt.cm.tab10(f))
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("radius in the parallel space R^4")
    ax[0].set_ylabel("radius in the perpendicular space R^4")
    ax[0].set_title("240 roots: two clusters on\nr_par^2 + r_perp^2 = 2 (colour = fibre)")
    inner = rpar < 1.0
    for f in range(nf):
        n_in = int((inner & (lab == f)).sum())
        ax[1].bar(f, n_in, color="tab:blue", label="inner parallel shell" if f == 0 else None)
        ax[1].bar(f, 24 - n_in, bottom=n_in, color="tab:orange", label="outer parallel shell" if f == 0 else None)
    ax[1].set_xticks(range(nf))
    ax[1].set_xlabel("fibre (right 2T orbit)")
    ax[1].set_ylabel("points of the fibre")
    ax[1].set_title("icosian projection: each fibre in one shell\n(5 fibres in each)")
    ax[1].legend(fontsize=8, loc="lower right")
    Rh = hs.e8_roots()
    lh, nh = hs.e8_fibres(Rh)
    g = find_isometry(Rh, lh, R, lab)
    cs = np.sort(plane_cosines(g)[1].min(axis=1))[::-1]
    ax[2].bar(range(32), cs, color=["tab:green"] + ["tab:gray"] * 31)
    ax[2].set_xlabel("the 32 fibre-compatible planes a + q b (sorted)")
    ax[2].set_ylabel("smallest cosine of a principal angle")
    ax[2].set_title("icosian plane against the 32 planes\nof hopf_shells: exactly one coincides")
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=110)
        print("wrote", save)
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
    figure(os.path.join(d, "icosian_e8.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
