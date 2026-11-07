"""
e8_shells.py - the shells of the E8 lattice beyond the roots: counts, whole fibres, and the arithmetic of the icosian form.

Sadoc and Mosseri (QUANTUM_GRAVITY.md, section 1) cut E8 into shells round a vertex, group the shell sites into
3-sphere fibres of 24, accept or reject whole fibres by arithmetic criteria, give an arithmetic formula for the number of
points on each shell, and reduce the selection to a 2D-1D construction like the Fibonacci chain.  icosian_e8.py built the 240
roots (shell 1) in two ways; this file goes up the shells and tests each of those sentences for the construction used here.

Setting.  E8 = the Hurwitz pairs (a, b) with a - b in (1 + i)H, norm |a|^2 + |b|^2 = 2n on shell n (roots: n = 1).  A *fibre* is a
right 2T orbit (a, b) -> (a l, b l).  In the icosian coordinates of icosian_e8.py (reached by the isometry found there) a lattice
vector is Phi(x) = sqrt(2/5) (sqrt(3 - phi) x, sqrt(2 + phi) x^sigma) with x an icosian of reduced norm alpha = |x|^2 in Z[phi].

Exact results (tests):
  1. Shell n has 240 sigma_3(n) vectors (n = 1..6 counted from the lattice; 1..60 from the formula below), and it splits into
     exactly 10 sigma_3(n) fibres of 24 (no vector is fixed by a non-trivial l).
  2. Shell n consists of the vectors with alpha = n + b phi, b an integer with alpha and sigma(alpha) both > 0: so b runs over
     the integers in (-n / phi, n phi).  The shell index n is the integer part of alpha, and (n, b) is a 2D lattice point:
     the position inside a shell is an integer b in a window, which is the 2D-1D structure of the papers.
  3. The number of icosians of norm alpha is 120 times the number of ideals-with-multiplicity weight
     S(alpha) = sum over ideals d | (alpha) of N(d) (the icosian ring is a maximal order of class number one, ramified only
     at the two infinite places).  Counted on the enumerated shells n = 1..6 and, as an identity of integers,
     sum_b 120 S(n + b phi) = 240 sigma_3(n) for n = 1..60.  Each class (n, b) is therefore 5 S(alpha) fibres.
  4. On a fibre the pair of radii (|a|, |b|) is constant (right multiplication by a unit quaternion preserves both), so any
     rule that depends on the radii, such as a spherical window in the perpendicular space, accepts or rejects whole fibres.
     A rule that looks at one coordinate does not (negative control).
  5. A perpendicular window sigma(alpha) < w selects the alpha's of Z[phi] in a model set; for w = phi its consecutive gaps take
     exactly the two values 1 and phi (the Fibonacci chain), for w = 1 the values phi and phi^2.

Not built: the specific 4D point set of Elser and Sloane (their choice of window shape in the 4D perpendicular space, which is
a polytope and not a ball, so its acceptance need not be constant on a fibre); the claim 3 for ideals is verified, not
proved, by the counts here.  Nothing relates to the Spectre or the Chair44.

    python3 models/e8_shells.py                 tests, then a figure (about 25 s)
    python3 models/e8_shells.py --tests-only
    python3 models/e8_shells.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hopf_shells as hs  # noqa: E402
import icosian_e8 as ie  # noqa: E402
from hopf_shells import PHI, hurwitz_units, qmul  # noqa: E402


# ------------------------------------------------------------------------------------------------ exact Z[phi]

def zmul(x, y):
    """(a + b phi)(c + d phi) with phi^2 = phi + 1, as integer pairs."""
    a, b = x
    c, d = y
    return (a * c + b * d, a * d + b * c + b * d)


def zconj(x):
    """sigma(a + b phi) = a + b (1 - phi) = (a + b) - b phi."""
    a, b = x
    return (a + b, -b)


def znorm(x):
    """Field norm a^2 + a b - b^2 (an integer, can be negative)."""
    a, b = x
    return a * a + a * b - b * b


def zdivide(x, y):
    """(True, y / x) if x divides y in Z[phi], else (False, None)."""
    n = znorm(x)
    q = zmul(y, zconj(x))
    if q[0] % n == 0 and q[1] % n == 0:
        return True, (q[0] // n, q[1] // n)
    return False, None


def _rational_factors(n):
    f, d = {}, 2
    while d * d <= n:
        while n % d == 0:
            f[d] = f.get(d, 0) + 1
            n //= d
        d += 1
    if n > 1:
        f[n] = f.get(n, 0) + 1
    return f


def _valuation(x, pi):
    k = 0
    while True:
        ok, q = zdivide(pi, x)
        if not ok:
            return k
        x, k = q, k + 1


def _split_primes(p):
    """Two non-associate prime elements of norm +-p, for a prime p = +-1 mod 5.

    a^2 + a b - b^2 = s p (s = +-1) gives a = (-b +- sqrt(5 b^2 + 4 s p)) / 2, so one pass over b finds them."""
    reps = []
    for b in range(0, 2 * p + 2):
        for sgn in (1, -1):
            disc = 5 * b * b + 4 * sgn * p
            if disc < 0:
                continue
            r = math.isqrt(disc)
            if r * r != disc:
                continue
            for a2 in (-b + r, -b - r):
                if a2 % 2:
                    continue
                c = (a2 // 2, b)
                if abs(znorm(c)) == p and not any(zdivide(c, q)[0] and zdivide(q, c)[0] for q in reps):
                    reps.append(c)
        if len(reps) == 2:
            return reps
    raise AssertionError((p, reps))


_SPLIT = {}


def ideal_sum(alpha):
    """S(alpha) = sum of N(d) over the ideals d dividing (alpha), alpha = (a, b) = a + b phi nonzero.  Multiplicative over
    prime ideals: sqrt5 (norm 5, ramified), the two primes over p = +-1 mod 5 (norm p each), and p itself for the other p
    (inert, norm p^2); a prime power with valuation v contributes 1 + Np + ... + Np^v."""
    n = abs(znorm(alpha))
    out = 1
    for p, _ in _rational_factors(n).items():
        if p == 5:
            v = _valuation(alpha, (-1, 2))                        # sqrt5 = 2 phi - 1
            out *= sum(5 ** k for k in range(v + 1))
        elif p % 5 in (1, 4):
            if p not in _SPLIT:
                _SPLIT[p] = _split_primes(p)
            for pi in _SPLIT[p]:
                out *= sum(p ** k for k in range(_valuation(alpha, pi) + 1))
        else:
            out *= sum((p * p) ** k for k in range(_valuation(alpha, (p, 0)) + 1))
    return out


def sigma3(n):
    return sum(d ** 3 for d in range(1, n + 1) if n % d == 0)


def shell_classes(n):
    """The classes of shell n: list of (b, S(alpha), points, fibres) for alpha = n + b phi totally positive."""
    out = []
    for b in range(-n - 1, 3 * n + 2):
        if n + b * PHI > 1e-12 and n + b * (1 - PHI) > 1e-12:
            s = ideal_sum((n, b))
            out.append((b, s, 120 * s, 5 * s))
    return out


# ------------------------------------------------------------------------------------------------ the lattice

def _hurwitz_by_norm(maxnorm):
    """Dict norm -> array (k, 4) of the Hurwitz quaternions of that norm (integer coordinates, or all four half-odd)."""
    out = {m: [] for m in range(maxnorm + 1)}
    r = int(math.isqrt(maxnorm))
    rng = range(-r, r + 1)
    for w in rng:
        for x in rng:
            for y in rng:
                for z in rng:
                    m = w * w + x * x + y * y + z * z
                    if m <= maxnorm:
                        out[m].append((w, x, y, z))
    ro = int(math.isqrt(4 * maxnorm)) + 1
    odd = [o for o in range(-ro, ro + 1) if o % 2]
    for w in odd:
        for x in odd:
            for y in odd:
                for z in odd:
                    m4 = w * w + x * x + y * y + z * z
                    if m4 % 4 == 0 and m4 // 4 <= maxnorm:
                        out[m4 // 4].append((w / 2, x / 2, y / 2, z / 2))
    return {m: np.array(v, float).reshape(-1, 4) for m, v in out.items()}


def e8_shell(n, _cache={}):
    """All vectors of norm 2n of E8 = {(a, b) Hurwitz, a - b in (1 + i)H}, array (N, 8)."""
    key = max(n, _cache.get("max", 0))
    if _cache.get("max", 0) < n:
        _cache.clear()
        _cache["max"] = key
        _cache["H"] = _hurwitz_by_norm(2 * key)
    H = _cache["H"]
    one_minus_i = np.array([1.0, -1.0, 0.0, 0.0])
    rows = []
    for na in range(2 * n + 1):
        A, B = H[na], H[2 * n - na]
        if len(A) == 0 or len(B) == 0:
            continue
        D = A[:, None, :] - B[None, :, :]
        y = qmul(one_minus_i, D) / 2.0
        integral = np.all(np.abs(y - np.round(y)) < 1e-9, axis=-1)
        half = np.all(np.abs(y - 0.5 - np.round(y - 0.5)) < 1e-9, axis=-1)
        ia, ib = np.nonzero(integral | half)
        rows.append(np.hstack([A[ia], B[ib]]))
    return np.vstack(rows)


def fibre_labels(V):
    """Labels of the right 2T orbits (a, b) -> (a l, b l) of the vectors V (N, 8), and the number of orbits."""
    T = hurwitz_units()
    codes = np.empty((len(V), len(T)), dtype=np.int64)
    for k, t in enumerate(T):
        img = np.hstack([qmul(V[:, :4], t), qmul(V[:, 4:], t)])
        q = np.round(2 * img).astype(np.int64) + 40
        c = np.zeros(len(V), dtype=np.int64)
        for j in range(8):
            c = c * 128 + q[:, j]
        codes[:, k] = c
    canon = codes.min(axis=1)
    _, lab = np.unique(canon, return_inverse=True)
    return lab, int(lab.max()) + 1


_ISO = {}


def icosian_isometry():
    """The isometry g (x -> x @ g) of icosian_e8.find_isometry from the Hurwitz coordinates to the icosian ones (cached)."""
    if "g" not in _ISO:
        Rh = hs.e8_roots()
        lh, _ = hs.e8_fibres(Rh)
        Ri = ie.icosian_roots()
        li, _ = ie.right_fibres(Ri)
        _ISO["g"] = ie.find_isometry(Rh, lh, Ri, li)
    return _ISO["g"]


def alpha_of(V, g):
    """(n, b) of each vector, from the icosian coordinates: alpha = |parallel|^2 (5/2) / (3 - phi) = n + b phi, and
    sigma(alpha) = |perp|^2 (5/2) / (2 + phi).  Returns floats (a, b) and the residual from integers."""
    W = V @ g
    al = (W[:, :4] ** 2).sum(1) * 2.5 / (3 - PHI)
    als = (W[:, 4:] ** 2).sum(1) * 2.5 / (2 + PHI)
    b = (al - als) / math.sqrt(5)
    a = al - b * PHI
    res = max(np.abs(a - np.round(a)).max(), np.abs(b - np.round(b)).max())
    return a, b, res


def model_set_gaps(w, limit=60.0):
    """Sorted distinct gaps of {alpha in Z[phi] : 0 < alpha < limit, 0 <= sigma(alpha) < w}."""
    pts = []
    for n in range(-200, 200):
        for b in range(-200, 200):
            a1, a2 = n + b * PHI, n + b * (1 - PHI)
            if 0 < a1 < limit and 0 <= a2 < w:
                pts.append(a1)
    return sorted(set(np.round(np.diff(sorted(pts)), 5)))


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True, shells=6):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    check("Z[phi]: norm is multiplicative, sigma is a ring map, phi^2 = phi + 1",
          znorm(zmul((3, 2), (-1, 4))) == znorm((3, 2)) * znorm((-1, 4)) and zconj(zmul((3, 2), (-1, 4))) == zmul(zconj((3, 2)), zconj((-1, 4)))
          and zmul((0, 1), (0, 1)) == (1, 1))
    check("ideal sums: S(1) = 1, S(2) = 1 + 4 (2 is inert), S(3) = 1 + 9, S(sqrt5) = 1 + 5, S(5) = 1 + 5 + 25",
          [ideal_sum(x) for x in ((1, 0), (2, 0), (3, 0), (-1, 2), (5, 0))] == [1, 5, 10, 6, 31])
    check("S is multiplicative on coprime elements and S(11) counts the two primes over 11 (1 + 11)^2 = 144",
          ideal_sum(zmul((2, 0), (3, 0))) == ideal_sum((2, 0)) * ideal_sum((3, 0)) and ideal_sum((11, 0)) == 144)
    ok_id = all(sum(c[2] for c in shell_classes(n)) == 240 * sigma3(n) for n in range(1, 61))
    check("sum over the classes of 120 S(n + b phi) = 240 sigma_3(n), exactly, for n = 1..60", ok_id)
    check("shell n has the classes b in (-n/phi, n phi): the integers in that interval",
          all([c[0] for c in shell_classes(n)] == [b for b in range(-n - 1, 3 * n + 2) if -n / PHI < b < n * PHI] for n in range(1, 40)))
    check("shell 1 = the 240 roots: classes b = 0 (the 120 units, alpha = 1) and b = 1 (phi times them, alpha = phi^2)",
          [(c[0], c[2]) for c in shell_classes(1)] == [(0, 120), (1, 120)])

    g = icosian_isometry()
    for n in range(1, shells + 1):
        V = e8_shell(n)
        check("shell %d: %d vectors = 240 sigma_3(%d), all distinct, norm %d" % (n, len(V), n, 2 * n),
              len(V) == 240 * sigma3(n) and len({tuple(v) for v in V}) == len(V) and np.allclose((V ** 2).sum(1), 2 * n))
        a, b, res = alpha_of(V, g)
        check("  alpha = n + b phi with integer (n, b) for every vector (residual %.0e) and a = n" % res,
              res < 1e-8 and np.all(np.round(a) == n))
        bs = np.round(b).astype(int)
        cls = {c[0]: c for c in shell_classes(n)}
        counts = {int(k): int((bs == k).sum()) for k in set(bs)}
        check("  points per class = 120 S(alpha) for all %d classes" % len(cls), set(counts) == set(cls) and all(counts[k] == cls[k][2] for k in cls))
        if n <= 4:
            lab, nf = fibre_labels(V)
            check("  %d fibres = 10 sigma_3(%d), all of size 24" % (nf, n), nf == 10 * sigma3(n) and set(np.bincount(lab)) == {24})
            W = V @ g
            rp = np.round((W[:, :4] ** 2).sum(1), 8)
            rq = np.round((W[:, 4:] ** 2).sum(1), 8)
            const = all(len(set(rp[lab == f])) == 1 and len(set(rq[lab == f])) == 1 for f in range(nf))
            check("  both radii are constant on every fibre", const)
            fib_per = {int(k): len(set(lab[bs == k])) for k in set(bs)}
            check("  fibres per class = 5 S(alpha)", all(fib_per[k] == cls[k][3] for k in cls))
            whole = True
            for w in (0.5, 1.0, PHI, 2.5):
                acc = (np.round(((W[:, 4:] ** 2).sum(1)) * 2.5 / (2 + PHI), 8) < w)
                whole &= all(len(set(acc[lab == f])) == 1 for f in range(nf))
            check("  windows sigma(alpha) < w (a ball in the perpendicular space) accept or reject whole fibres", whole)
            split = sum(1 for f in range(nf) if len(set(W[lab == f][:, 4] > 0)) > 1)
            check("  negative control: the rule 'first perpendicular coordinate > 0' splits fibres", split > 0, "(%d of %d)" % (split, nf))
    check("model set {alpha in Z[phi] : 0 <= sigma(alpha) < w}: gaps {1, phi} for w = phi (Fibonacci), {phi, phi^2} for w = 1",
          model_set_gaps(PHI) == [1.0, round(PHI, 5)] and model_set_gaps(1.0) == [round(PHI, 5), round(PHI ** 2, 5)])
    return ok


# ------------------------------------------------------------------------------------------------ figure

def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8), constrained_layout=True)
    N = 14
    xs, ys, ss = [], [], []
    for n in range(1, N + 1):
        for b, s, pts, fib in shell_classes(n):
            xs.append(n)
            ys.append(b)
            ss.append(fib)
    sc = ax[0].scatter(xs, ys, s=[8 + 0.35 * v for v in ss], c=ss, cmap="viridis")
    nn = np.linspace(0.5, N + 0.5, 50)
    ax[0].plot(nn, nn * PHI, color="0.5", lw=0.8)
    ax[0].plot(nn, -nn / PHI, color="0.5", lw=0.8)
    ax[0].set_xlabel("shell n (norm 2n)")
    ax[0].set_ylabel("b, with alpha = n + b phi")
    ax[0].set_title("classes (n, b) of the E8 shells: integers inside\nthe cone b in (-n/phi, n phi); size = fibres 5 S(alpha)")
    fig.colorbar(sc, ax=ax[0], shrink=0.8, label="fibres in the class")
    nmax = 12
    ns = np.arange(1, nmax + 1)
    ax[1].bar(ns, [10 * sigma3(n) for n in ns], color=["tab:blue" if n <= 6 else "tab:gray" for n in ns])
    ax[1].set_yscale("log")
    ax[1].set_xlabel("shell n")
    ax[1].set_ylabel("fibres of 24 in the shell = 10 sigma_3(n)")
    ax[1].set_title("fibres per shell (blue: counted from the lattice,\ngrey: formula; points = 24 x fibres)")
    ax[1].set_ylim(5, 10 * sigma3(12) * 12)
    for n in (1, 2, 3, 6, 12):
        ax[1].text(n, 10 * sigma3(n) * 1.15, "%d" % (240 * sigma3(n)), ha="center", fontsize=7, rotation=90, va="bottom")
    for w, c in ((1.0, "tab:red"), (PHI, "tab:green")):
        pts = sorted(n + b * PHI for n in range(-120, 120) for b in range(-120, 120)
                     if 0 < n + b * PHI < 30 and 0 <= n + b * (1 - PHI) < w)
        ax[2].eventplot([pts], lineoffsets=0.4 if w < 1.5 and c == "tab:red" else 1.2, linelengths=0.6, colors=c)
    ax[2].set_ylim(-0.2, 1.9)
    ax[2].set_yticks([0.4, 1.2])
    ax[2].set_yticklabels(["window w = 1\ngaps phi, phi^2", "window w = phi\ngaps 1, phi"])
    ax[2].set_xlabel("alpha")
    ax[2].set_title("fibres accepted by sigma(alpha) < w: alpha forms\na Fibonacci-type chain (gaps in the labels)")
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
    figure(os.path.join(d, "e8_shells.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
