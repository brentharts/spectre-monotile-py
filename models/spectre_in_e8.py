"""
spectre_in_e8.py - the 4D lift of the Spectre (Cheritat-Ma, lift4d.py) placed inside E8, and what that does and does not say.

Known from lift4d.py.  Every Spectre edge is a unit vector at a multiple of 30 degrees.  Even multiples are hexagonal vectors
p + q w (w = e^{i pi/3}) and are called *red*; odd multiples are z (r + s w), z = e^{i pi/6}, and are called *green*.  A vertex gets
integer coordinates (p, q, r, s) and the plane position is  u + z v  with  u = p + q w,  v = r + s w.  So the lift lattice is
Z[zeta_12] = Z[w] + z Z[w], the red A2 plus the green A2.

New here.  E8 has an order-3 fibre rotation R_w (models/e6_fibres.py) that makes it a rank-4 module over Z[omega] (omega = e^{2 pi i/3}).
Take two roots r1 = (1 + i, 0) in the pole fibre D4a and r2 = (0, 1 + i) in the pole fibre D4b; they are Hermitian-orthogonal
(<r1, omega^k r2> = 0 for k = 0, 1, 2).  Then
        embed(p, q, r, s) = (p + q (1 + R_w)) r1 + (r + s (1 + R_w)) r2          (1 + omega = w)
is a Z-linear map from the lift lattice into E8 with  <embed(x), embed(y)> = 2 Re(u_x conj u_y + v_x conj v_y).

Exact results (tested on the 71 tiles / 518 vertices of a level-2 patch):
  1. The embedding is an isometry (up to the factor 2) from the lift lattice with the red-orthogonal-green form, so the lift lattice is A2 + A2
     inside E8, and it is injective on the vertices of the patch.
  2. Every Spectre edge becomes a root of E8, each red edge a root of the pole fibre D4a and each green edge a root of D4b.  Each tile's
     14-edge outline is a closed 14-step walk on those roots (8 red + 6 green for 63 of the 71 tiles, 6 red + 8 green for the other 8, whose
     orientation differs by an odd multiple of 30 degrees; the corner types are the same for all 71).
  3. For consecutive edges of the same colour the E8 inner product is 2 cos(turning angle); for a red-green pair it is 0.
  4. Negative control: a root orthogonal to r1 but not Hermitian-orthogonal breaks the isometry.
  5. The inflation does NOT extend to E8.  Its 4D block has characteristic polynomial x^4 - 8x^2 + 1 with roots +-lambda, +-1/lambda,
     lambda = (sqrt 3 + sqrt 5)/sqrt 2, lambda^2 = 4 + sqrt 15, so it has eigenvalues of modulus not 1, while every automorphism of E8 has finite
     order.  The field Q(lambda) = Q(sqrt 6, sqrt 10) contains sqrt 15 = sqrt 3 sqrt 5 but neither sqrt 3 nor sqrt 5, so it does not contain the golden ratio.
Hypothesis, not tested: that the product structure sqrt 15 = sqrt 3 x sqrt 5 (hexagonal field times icosahedral field) points to a construction
that combines the Eisenstein structure of E8 with its icosian structure.  Nothing here shows that it does.
What this does not give: a cut-and-project scheme from E8 (the Spectre window is a fractal in the contracting plane of the inflation, see
lift4d.py), the Chair44, or any selection of sites by fibres.

    python3 models/spectre_in_e8.py                 tests, then a figure
    python3 models/spectre_in_e8.py --tests-only
    python3 models/spectre_in_e8.py --no-show --save-dir figures
"""
import collections
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_HERE, _ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import e6_fibres as ef  # noqa: E402
import lift4d as L  # noqa: E402

W = np.exp(1j * math.pi / 3)
R1 = np.r_[np.array([1.0, 1, 0, 0]), np.zeros(4)]
R2 = np.r_[np.zeros(4), np.array([1.0, 1, 0, 0])]
_C = {}


def rw():
    if "rw" not in _C:
        _C["rw"] = ef.right_matrix(ef.OMEGA)
    return _C["rw"]


def mul_omega(x, k=1):
    for _ in range(k):
        x = x @ rw()
    return x


def mul_w(x):
    """Multiplication by w = 1 + omega on E8 viewed as a Z[omega]-module."""
    return x + mul_omega(x)


def embed(v, r1=R1, r2=R2):
    """Lift coordinates (p, q, r, s) -> a vector of E8 (array of shape (8,))."""
    p, q, r, s = (float(t) for t in v)
    return p * r1 + q * mul_w(r1) + r * r2 + s * mul_w(r2)


def uv(v):
    return v[0] + v[1] * W, v[2] + v[3] * W


def hermitian_ip(a, b):
    """2 Re(u_a conj u_b + v_a conj v_b) for lift coordinates a, b."""
    ua, va = uv(a)
    ub, vb = uv(b)
    return 2 * ((ua * np.conj(ub)).real + (va * np.conj(vb)).real)


def patch(levels=2):
    if ("patch", levels) not in _C:
        _C[("patch", levels)] = L.lift_patch(levels)
    return _C[("patch", levels)]


def edge_vectors(tile):
    """E8 vectors of the 14 edges of a lifted tile (list of 14 four-vectors)."""
    n = len(tile)
    return [embed(np.array(tile[(i + 1) % n]) - np.array(tile[i])) for i in range(n)]


def is_red(step):
    return step[2] == 0 and step[3] == 0


def inflation_poly_facts():
    """(irreducible over Q, roots of x^4 - 8x^2 + 1, [does Q(lambda) contain sqrt3, sqrt5, phi])."""
    import sympy as sp
    x = sp.symbols("x")
    lam = (sp.sqrt(3) + sp.sqrt(5)) / sp.sqrt(2)
    P = x ** 4 - 8 * x ** 2 + 1
    irreducible = sp.Poly(P, x).is_irreducible
    lam_ok = sp.simplify(lam ** 2 - (4 + sp.sqrt(15))) == 0 and sp.simplify(P.subs(x, lam)) == 0
    degs = []
    for extra in (sp.sqrt(3), sp.sqrt(5), (1 + sp.sqrt(5)) / 2, sp.sqrt(15)):
        d = sp.degree(sp.minimal_polynomial(lam + extra, x), x)
        degs.append(d)
    contains = [d == 4 for d in degs]              # Q(lambda, c) has degree 4 iff c is in Q(lambda); 8 otherwise (lambda + c generates it)
    return irreducible, lam_ok, np.roots([1, 0, -8, 0, 1]), contains


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    R, lab, idx = ef.setup()
    pa, pb = ef.poles()
    tiles, lifted, _ = patch(2)
    check("r1 = (1 + i, 0) and r2 = (0, 1 + i) are roots in the two pole fibres, Hermitian-orthogonal (ip with omega^k r2 = 0, k = 0, 1, 2)",
          lab[idx[tuple(np.round(R1, 6) + 0)]] == pa and lab[idx[tuple(np.round(R2, 6) + 0)]] == pb
          and all(abs(R1 @ mul_omega(R2, k)) < 1e-9 for k in range(3)))
    check("level-2 patch: 71 tiles, 518 vertices", len(tiles) == 71 and len(lifted) == 518)
    keys = list(lifted)
    V = [np.array(lifted[k]) for k in keys]
    E = np.array([embed(v) for v in V])
    bad = 0
    rng = np.random.default_rng(0)
    for _ in range(4000):
        i, j = rng.integers(len(V), size=2)
        if abs(E[i] @ E[j] - hermitian_ip(V[i], V[j])) > 1e-9:
            bad += 1
    check("the embedding is an isometry for the red-orthogonal-green form (4000 random vertex pairs of the patch)", bad == 0)
    check("the embedding is injective on the 518 vertices", len(np.unique(np.round(E, 6), axis=0)) == len(V))
    all_roots = all_red = all_green = True
    closed = True
    for t in tiles:
        n = len(t)
        steps = [np.array(t[(i + 1) % n]) - np.array(t[i]) for i in range(n)]
        d = edge_vectors(t)
        closed &= bool(np.allclose(sum(d), 0))
        for s, e in zip(steps, d):
            k = tuple(np.round(e, 6) + 0)
            if k not in idx:
                all_roots = False
                continue
            f = lab[idx[k]]
            if is_red(s):
                all_red &= f == pa
            else:
                all_green &= f == pb
    check("every edge of all 71 tiles is a root of E8, and every outline is a closed walk of 14 roots", all_roots and closed)
    check("red edges are roots of the pole fibre D4a, green edges roots of D4b", all_red and all_green)
    t0 = tiles[0]
    steps0 = [np.array(t0[(i + 1) % 14]) - np.array(t0[i]) for i in range(14)]
    check("a tile outline has 8 red and 6 green edges", sum(is_red(s) for s in steps0) == 8)
    split = collections.Counter()
    for t in tiles:
        n = len(t)
        split[sum(is_red(np.array(t[(i + 1) % n]) - np.array(t[i])) for i in range(n))] += 1
    check("across the 71 tiles the red-green split is 8+6 for 63 tiles and 6+8 for 8 (a tile rotated by an odd multiple of 30 degrees swaps the colours)",
          dict(split) == {8: 63, 6: 8}, "(%s)" % {int(k): v for k, v in split.items()})
    ok3 = True
    for t in tiles:
        n = len(t)
        steps = [np.array(t[(i + 1) % n]) - np.array(t[i]) for i in range(n)]
        d = edge_vectors(t)
        for i in range(n):
            j = (i + 1) % n
            ip = d[i] @ d[j]
            ci, cj = is_red(steps[i]), is_red(steps[j])
            if ci == cj:
                a, b = uv(steps[i]), uv(steps[j])
                pi_, pj = (a[0], b[0]) if ci else (a[1], b[1])
                want = 2 * (pi_ * np.conj(pj)).real
                ok3 &= abs(ip - want) < 1e-9
            else:
                ok3 &= abs(ip) < 1e-9
    check("same-colour consecutive edges: inner product = 2 cos(turn) of the in-colour angle; red-green pairs: 0", ok3)
    # negative control: a root orthogonal to r1 but not Hermitian-orthogonal
    wrong = None
    for r in R:
        if abs(r @ R1) < 1e-9 and any(abs(r @ mul_omega(R1, k)) > 1e-9 for k in range(3)) and abs(r @ r - 2) < 1e-9:
            wrong = r
            break
    bad2 = sum(abs(embed(v, R1, wrong) @ embed(w, R1, wrong) - hermitian_ip(v, w)) > 1e-9 for v, w in zip(V[:200], V[1:201]))
    check("negative control: a second root that is not Hermitian-orthogonal to r1 breaks the isometry", wrong is not None and bad2 > 0, "(%d of 200 pairs off)" % bad2)
    irr, lam_ok, roots_, contains = inflation_poly_facts()
    real = sorted(roots_.real)
    check("inflation block: x^4 - 8x^2 + 1 is irreducible, lambda = (sqrt3 + sqrt5)/sqrt2 is a root with lambda^2 = 4 + sqrt15",
          irr and lam_ok)
    check("its roots are +-2.8059 and +-0.3564, so it has eigenvalues of modulus != 1 and cannot be (the restriction of) an isometry of E8",
          np.allclose(real, [-2.8059, -0.3564, 0.3564, 2.8059], atol=1e-3))
    check("lambda generates a field of degree 4 that contains sqrt15 but not sqrt3, sqrt5 or the golden ratio",
          contains == [False, False, False, True], "(degrees of lambda + c for c = sqrt3, sqrt5, phi, sqrt15: %s)" % contains)
    return ok


# ------------------------------------------------------------------------------------------------ figure

def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tiles, lifted, _ = patch(2)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5.6))
    for t in tiles:
        n = len(t)
        P = np.array([L.project(np.array(v)) for v in t])
        for i in range(n):
            s = np.array(t[(i + 1) % n]) - np.array(t[i])
            a, b = P[i], P[(i + 1) % n]
            ax[0].plot([a.real, b.real], [a.imag, b.imag], color="tab:red" if is_red(s) else "tab:green", lw=0.8)
    ax[0].set_aspect("equal"); ax[0].set_title("Spectre patch: red edges = roots of pole D4a, green = D4b")
    t = tiles[0]
    n = len(t)
    P = np.array([L.project(np.array(v)) for v in t])
    d = edge_vectors(t)
    for i in range(n):
        s = np.array(t[(i + 1) % n]) - np.array(t[i])
        a, b = P[i], P[(i + 1) % n]
        ax[1].plot([a.real, b.real], [a.imag, b.imag], color="tab:red" if is_red(s) else "tab:green", lw=3)
        j = (i + 1) % n
        ax[1].text((a + b).real / 2, (a + b).imag / 2, "%d" % round(d[i] @ d[j]), fontsize=9, ha="center", va="center",
                   bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.8))
    ax[1].set_aspect("equal")
    ax[1].set_title("one tile: label on edge i = inner product of root i with root i+1\n(2 straight, 1 turn of 60 deg, 0 red-green)")
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
    figure(os.path.join(d, "spectre_in_e8.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
