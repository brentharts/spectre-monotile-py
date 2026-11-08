"""
e6_fibres.py - E8 ⊃ E6 + A2 and the triality blocks, read off the Hopf fibres of the Hurwitz-pair model.

Setting.  E8 = Hurwitz pairs (a, b) with a - b in (1 + i)H, 240 roots of norm^2 2 (hopf_shells.py).  A *fibre* is a right 2T orbit
(a, b) -> (a l, b l), l in 2T; there are 10 of 24 roots.  The chirality-standard-model fork (QUANTUM_GRAVITY.md, section 4) works with
E8 ⊃ E6 x SU(3) (240 = 72 + 6 + 6 x 27) and with E8 ⊃ D4 x D4 (240 = 24 + 24 + 3 x 64).  Here both are located inside the fibres.
Written from scratch; nothing was copied from that repository (which is not freely licensed).

Notation.  D4a = roots (a, 0), D4b = roots (0, b); w = (-1 + i + j + k)/2 (order 3); R_l = the fibre rotation (a, b) -> (a l, b l).
The three *blocks* of the 192 roots with a, b both nonzero are the classes of a modulo (1 + i)H (the Lipschitz units; the two
classes of half-integer units); they are the 64-blocks 8v x 8v, 8s x 8s, 8c x 8c of the D4 x D4 picture.

Exact results (finite enumerations, all tested):
  1. D4a and D4b are each a whole fibre (the poles of the 5-cross-polytope of base points); the other 192 roots are the 8 equatorial fibres.
  2. Every equatorial fibre has exactly 8 roots in each of the three 64-blocks.  So the fibres cut across the blocks evenly.
  3. R_w (a fibre rotation of order 3, with no fixed vector) permutes the three blocks cyclically: the order-3 triality of D4 x D4 is
     a fibre rotation.
  4. A2 = the R_w orbit of (1 + i, 0) and its negatives is an A2 root system inside the pole D4a, preserved by R_w, which acts on it
     as a rotation of order 3.  Its orthogonal complement in the roots is an E6 (72 roots).
  5. Fibres vs the split 240 = 6 + 72 + 162: pole D4a holds the A2 (6) and 18 of the 162; pole D4b is entirely E6 (24); each of the
     8 equatorial fibres holds 6 of E6 and 18 of the 162.  E6 = 24 + 3 x 16: its 48 equatorial roots are 16 in each block.
  6. The 162 projects to 6 weights of A2 (27 roots each) and to 54 weights of E6 (27 and its negative, 3 roots each); within one of the
     27's the inner products are 1 x 4/3, 16 x 1/3, 10 x -2/3 (the 1 + 16 + 10 of SO(10)).  Each 27-class has exactly 3 roots in each
     of the 9 fibres other than D4b: 27 = 9 x 3.
  7. E8 has 1120 A2 subsystems.  They have exactly two fibre profiles: 160 lie inside one fibre (10 fibres x 16 A2 in a D4), 960 meet
     three fibres in two roots each.  The fibre group acts with orbits of size 4 (40 orbits, stabiliser Z6 = <-w>) and 12 (80 orbits,
     stabiliser +-1).  Our A2 is of the first kind.
Hypothesis, not tested: that the Chair44 count 192 = 8 x 24 is the equatorial part (E8 minus the two D4 poles); that the Eisenstein
structure of E8 (R_w) connects to the hexagonal geometry of the Spectre.  Nothing here uses either tile.

    python3 models/e6_fibres.py                 tests, then a figure
    python3 models/e6_fibres.py --tests-only
    python3 models/e6_fibres.py --no-show --save-dir figures
"""
import collections
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import hopf_shells as hs  # noqa: E402
from hopf_shells import hurwitz_units, qmul  # noqa: E402

OMEGA = np.array([-0.5, 0.5, 0.5, 0.5])
ONE_MINUS_I = np.array([1.0, -1.0, 0.0, 0.0])
_C = {}


def setup():
    """(roots (240, 8), fibre labels (240,), index dict) of the Hurwitz-pair E8 (cached)."""
    if "R" not in _C:
        R = hs.e8_roots()
        lab, _ = hs.e8_fibres(R)
        _C.update(R=R, lab=lab, idx={tuple(np.round(r, 6) + 0): i for i, r in enumerate(R)})
    return _C["R"], _C["lab"], _C["idx"]


def _find(x):
    return setup()[2][tuple(np.round(x, 6) + 0)]


def right_perm(l):
    """Permutation of the roots induced by the fibre rotation (a, b) -> (a l, b l)."""
    R, _, _ = setup()
    return np.array([_find(np.r_[qmul(r[:4], l), qmul(r[4:], l)]) for r in R])


def right_matrix(l):
    """The 8 x 8 matrix of (a, b) -> (a l, b l) acting on row vectors."""
    M = np.zeros((8, 8))
    for k in range(4):
        e = np.zeros(4)
        e[k] = 1
        col = qmul(e, l)
        M[k, :4] = col
        M[4 + k, 4:] = col
    return M


def poles():
    """(label of the fibre D4a = {(a, 0)}, label of D4b = {(0, b)})."""
    R, lab, _ = setup()
    a = {int(lab[i]) for i in range(240) if np.allclose(R[i][4:], 0)}
    b = {int(lab[i]) for i in range(240) if np.allclose(R[i][:4], 0)}
    assert len(a) == 1 and len(b) == 1
    return a.pop(), b.pop()


def blocks():
    """Array (240,): block 0, 1, 2 (the class of a mod (1 + i)H) for the 192 equatorial roots, -1 for the 48 in D4a, D4b."""
    R, _, _ = setup()
    reps = [np.array([1.0, 0, 0, 0]), OMEGA, qmul(OMEGA, OMEGA)]
    out = np.full(240, -1)
    for i, r in enumerate(R):
        if np.allclose(r[4:], 0) or np.allclose(r[:4], 0):
            continue
        for k, rep in enumerate(reps):
            if hs.in_hurwitz(qmul(ONE_MINUS_I, r[:4] - rep) / 2.0) or hs.in_hurwitz(qmul(ONE_MINUS_I, r[:4] + rep) / 2.0):
                out[i] = k
    return out


def omega_a2():
    """Sorted indices of the A2 = {+-(1 + i, 0) R_w^k}, a root subsystem inside D4a."""
    x = np.r_[np.array([1.0, 1, 0, 0]), np.zeros(4)]
    out = set()
    for _ in range(3):
        out |= {_find(x), _find(-x)}
        x = np.r_[qmul(x[:4], OMEGA), np.zeros(4)]
    return sorted(out)


def complement(A):
    """Indices of the roots orthogonal to every root in A, and of the remaining roots."""
    R, _, _ = setup()
    G = np.round(R @ R.T).astype(int)
    orth = np.where(np.all(G[:, A] == 0, axis=1))[0]
    rest = np.array([i for i in range(240) if i not in set(A) | set(orth.tolist())])
    return orth, rest


def a2_subsystems():
    """All 1120 A2 root subsystems of E8, as sorted index tuples."""
    R, _, _ = setup()
    G = np.round(R @ R.T).astype(int)
    out = set()
    for i, j in zip(*np.where(G == -1)):
        k = _find(R[i] + R[j])
        s = {i, j, k}
        out.add(tuple(sorted(s | {_find(-R[x]) for x in s})))
    return sorted(out)


def fibre_counts(idx_set):
    """Number of roots of ``idx_set`` in each of the 10 fibres."""
    _, lab, _ = setup()
    return np.bincount(lab[list(idx_set)], minlength=10)


def is_e6_root_system(idx_set):
    """72 roots of rank 6 whose inner products with each root are 1 x 2, 20 x 1, 30 x 0, 20 x -1, 1 x -2, closed under reflections."""
    R, _, _ = setup()
    P = R[list(idx_set)]
    if len(P) != 72 or np.linalg.matrix_rank(P, tol=1e-8) != 6:
        return False
    G = np.round(P @ P.T).astype(int)
    want = {2: 1, 1: 20, 0: 30, -1: 20, -2: 1}
    if any(dict(collections.Counter(row)) != want for row in G):
        return False
    S = {tuple(np.round(p, 6) + 0) for p in P}
    return all(tuple(np.round(q - (q @ p) * p, 6) + 0) in S for p in P for q in P)


def weights_27(A):
    """(A2 parts (162, 8), E6 parts (162, 8), indices of the 162) of the roots outside A2 and its orthogonal complement."""
    R, _, _ = setup()
    _, rest = complement(A)
    B = R[A].T
    P = B @ np.linalg.pinv(B)
    pa = R[rest] @ P.T
    return pa, R[rest] - pa, rest


# ------------------------------------------------------------------------------------------------ tests

def run_tests(verbose=True):
    ok = True

    def check(name, cond, extra=""):
        nonlocal ok
        ok &= bool(cond)
        if verbose:
            print("  %-4s %s %s" % ("ok" if cond else "FAIL", name, extra))

    R, lab, _ = setup()
    pa, pb = poles()
    blk = blocks()
    check("240 roots of norm^2 2 in 10 fibres of 24", len(R) == 240 and np.allclose((R ** 2).sum(1), 2)
          and sorted(np.bincount(lab)) == [24] * 10)
    check("D4a = {(a, 0)} and D4b = {(0, b)} are each a whole fibre; the other 192 roots are the 8 equatorial fibres",
          np.sum(lab == pa) == 24 and np.sum(lab == pb) == 24 and pa != pb
          and all(np.allclose(R[i][4:], 0) for i in np.where(lab == pa)[0])
          and all(np.allclose(R[i][:4], 0) for i in np.where(lab == pb)[0]))
    check("the 192 equatorial roots fall into three blocks of 64", [int((blk == k).sum()) for k in range(3)] == [64, 64, 64]
          and int((blk == -1).sum()) == 48)
    eqf = [f for f in range(10) if f not in (pa, pb)]
    check("every equatorial fibre has exactly 8 roots in each block",
          all([int(((lab == f) & (blk == k)).sum()) for k in range(3)] == [8, 8, 8] for f in eqf))
    rng = np.random.default_rng(1)
    fake = np.where(blk >= 0, rng.permutation(np.where(blk >= 0, blk, 0)), -1)
    check("negative control: a random relabelling of the blocks does not give 8 + 8 + 8 in every fibre",
          not all([int(((lab == f) & (fake == k)).sum()) for k in range(3)] == [8, 8, 8] for f in eqf))
    perm = right_perm(OMEGA)
    b0 = np.where(blk == 0)[0]
    shift = int(blk[perm[b0[0]]])
    check("R_w permutes the roots with order 3 and cycles the three blocks (every block k goes to block k + %d mod 3)" % shift,
          len(set(perm)) == 240 and np.array_equal(perm[perm[perm]], np.arange(240)) and shift in (1, 2)
          and all(blk[perm[i]] == (blk[i] + shift) % 3 for i in range(240) if blk[i] >= 0))
    ev = np.linalg.eigvals(right_matrix(OMEGA))
    check("R_w has no fixed vector: all eigenvalues are primitive cube roots of 1", np.allclose(ev ** 3, 1) and not np.any(np.isclose(ev, 1)))
    i_perm = right_perm(np.array([0.0, 1, 0, 0]))
    check("negative control: R_i has order 4 and does not cycle three blocks",
          not np.array_equal(i_perm[i_perm[i_perm]], np.arange(240)) and not all(blk[i_perm[i]] != blk[i] for i in range(240) if blk[i] >= 0))
    A = omega_a2()
    GA = np.round(R[A] @ R[A].T).astype(int)
    check("A2 = R_w orbit of (1 + i, 0): 6 roots of rank 2, Gram entries {2, 1, -1, -2}, inside the pole fibre D4a",
          len(A) == 6 and np.linalg.matrix_rank(R[A]) == 2 and set(GA.ravel()) == {2, 1, -1, -2} and all(lab[A] == pa))
    check("R_w preserves A2 and moves each of its roots to a neighbour at inner product -1 (a rotation by 120 degrees)",
          set(perm[A]) == set(A) and all(R[i] @ R[perm[i]] == -1 for i in A))
    E6, rest = complement(A)
    check("the roots orthogonal to A2 form an E6: 72 roots, rank 6, E6 inner-product pattern, closed under reflections",
          is_e6_root_system(E6))
    check("negative control: the roots orthogonal to a single root are 126 (an E7), not an E6", len(complement([0])[0]) == 126
          and not is_e6_root_system(complement([0])[0]))
    check("240 = 6 + 72 + 162", len(A) + len(E6) + len(rest) == 240 and len(rest) == 162)
    check("R_w preserves E6 and the 162", set(perm[E6]) == set(E6) and set(perm[rest]) == set(rest))
    fa, fe, fr = fibre_counts(A), fibre_counts(E6), fibre_counts(rest)
    exp_a = np.zeros(10, int); exp_a[pa] = 6
    exp_e = np.full(10, 6); exp_e[pa] = 0; exp_e[pb] = 24
    exp_r = np.full(10, 18); exp_r[pb] = 0
    check("fibre profile: A2 in D4a only; E6 = all of D4b + 6 per equatorial fibre; the 162 = 18 in each of 9 fibres",
          np.array_equal(fa, exp_a) and np.array_equal(fe, exp_e) and np.array_equal(fr, exp_r))
    eq_e6 = [i for i in E6 if blk[i] >= 0]
    eq_rest = [i for i in rest if blk[i] >= 0]
    check("E6 = 24 + 3 x 16: its 48 equatorial roots are 16 per block; the equatorial part of the 162 is 48 per block",
          sorted(collections.Counter(blk[eq_e6]).values()) == [16, 16, 16] and sorted(collections.Counter(blk[eq_rest]).values()) == [48, 48, 48])
    pa_, pe_, ridx = weights_27(A)
    ka = collections.Counter(map(tuple, np.round(pa_, 5) + 0))
    ke = collections.Counter(map(tuple, np.round(pe_, 5) + 0))
    check("the 162 give 6 A2 weights (27 roots each) and 54 E6 weights (3 roots each) of norm^2 4/3",
          sorted(ka.values()) == [27] * 6 and sorted(set(ke.values())) == [3] and len(ke) == 54
          and np.allclose((np.array(list(ke.keys())) ** 2).sum(1), 4 / 3, atol=1e-4))
    one27 = None
    for a_w in ka:
        m = np.all(np.abs(pa_ - np.array(a_w)) < 1e-5, axis=1)
        one27 = np.unique(np.round(pe_[m], 5), axis=0)
        break
    gram = np.round(one27 @ one27.T * 3).astype(int)           # inner products times 3
    pattern = sorted(collections.Counter(gram[0]).items())
    check("one 27: 27 distinct E6 weights with inner products 1 x 4/3, 16 x 1/3, 10 x -2/3 from any weight (1 + 16 + 10)",
          len(one27) == 27 and all(sorted(collections.Counter(row).items()) == [(-2, 10), (1, 16), (4, 1)] for row in gram),
          "(pattern %s)" % pattern)
    ok27 = True
    for a_w in ka:
        m = np.all(np.abs(pa_ - np.array(a_w)) < 1e-5, axis=1)
        c = np.bincount(lab[ridx[m]], minlength=10)
        want = np.full(10, 3); want[pb] = 0
        ok27 &= bool(np.array_equal(c, want))
    check("each of the 6 weight classes has exactly 3 roots in each of the 9 fibres other than D4b (27 = 9 x 3)", ok27)
    subs = a2_subsystems()
    check("E8 has 1120 A2 subsystems", len(subs) == 1120)
    prof = collections.Counter()
    inside = 0
    for S in subs:
        c = np.bincount(lab[list(S)], minlength=10)
        prof[tuple(sorted(c[c > 0]))] += 1
        inside += int(len(set(lab[list(S)])) == 1)
    check("two fibre profiles for the A2's: 160 inside one fibre (6), 960 meeting three fibres in 2 + 2 + 2",
          dict(prof) == {(6,): 160, (2, 2, 2): 960} and inside == 160, str(dict(prof)))
    perms = [right_perm(l) for l in hurwitz_units()]
    sset = {frozenset(s) for s in subs}
    seen, sizes = set(), []
    for s in sset:
        if s in seen:
            continue
        orb = {frozenset(int(p[i]) for i in s) for p in perms}
        seen |= orb
        sizes.append(len(orb))
    check("fibre group on the 1120 A2's: 40 orbits of size 4 and 80 of size 12",
          dict(collections.Counter(sizes)) == {4: 40, 12: 80}, str(dict(collections.Counter(sizes))))
    stab = [k for k, p in enumerate(perms) if set(p[A]) == set(A)]
    T = hurwitz_units()
    minus_w = -OMEGA
    pw = right_perm(minus_w)
    cyc = [np.arange(240)]
    for _ in range(5):
        cyc.append(pw[cyc[-1]])
    check("stabiliser of our A2 in the fibre group is the cyclic group Z6 generated by -w",
          len(stab) == 6 and len({tuple(c) for c in cyc}) == 6 and np.array_equal(pw[cyc[-1]], np.arange(240)))
    check("our A2 lies in an orbit of size 4 and inside one fibre (the aligned kind)",
          len({frozenset(int(p[i]) for i in A) for p in perms}) == 4 and len(set(lab[A])) == 1)
    ke_neg = collections.Counter(map(tuple, np.round(pe_, 5) + 0))
    check("27bar = -27: the 54 E6 weights are closed under negation, and no weight equals its negative",
          all(tuple(np.round(-np.array(w), 5) + 0) in ke_neg and tuple(np.round(-np.array(w), 5) + 0) != w for w in ke_neg))
    return ok


# ------------------------------------------------------------------------------------------------ figure

def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    R, lab, _ = setup()
    pa, pb = poles()
    A = omega_a2()
    E6, rest = complement(A)
    order = [pa, pb] + [f for f in range(10) if f not in (pa, pb)]
    names = ["pole a\n(has A2)", "pole b\n(D4b)"] + ["eq %d" % k for k in range(1, 9)]
    fa, fe, fr = fibre_counts(A), fibre_counts(E6), fibre_counts(rest)
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    x = np.arange(10)
    ax[0].bar(x, fa[order], color="tab:green", label="A2 (6)")
    ax[0].bar(x, fe[order], bottom=fa[order], color="tab:blue", label="E6 (72)")
    ax[0].bar(x, fr[order], bottom=(fa + fe)[order], color="tab:orange", label="the 162 = 6 x 27")
    ax[0].set_xticks(x); ax[0].set_xticklabels(names, fontsize=7); ax[0].set_ylabel("roots in the fibre (24 each)")
    ax[0].set_ylim(0, 32); ax[0].set_title("E8 ⊃ E6 + A2 across the 10 fibres"); ax[0].legend(fontsize=8, ncol=3, loc="upper center")
    pa_, pe_, ridx = weights_27(A)
    classes = sorted({tuple(np.round(v, 5) + 0) for v in pa_})
    M = np.zeros((len(classes), 10), int)
    for r, c in enumerate(classes):
        m = np.all(np.abs(pa_ - np.array(c)) < 1e-5, axis=1)
        M[r] = np.bincount(lab[ridx[m]], minlength=10)[order]
    im = ax[1].imshow(M, cmap="Blues", vmin=0, vmax=4)
    for r in range(M.shape[0]):
        for c in range(10):
            ax[1].text(c, r, M[r, c], ha="center", va="center", fontsize=8)
    ax[1].set_xticks(x); ax[1].set_xticklabels(names, fontsize=7); ax[1].set_ylabel("A2 weight (3 and 3bar)")
    ax[1].set_yticks(range(len(classes))); ax[1].set_title("each 27: 3 roots in each of 9 fibres")
    blk = blocks()
    eq = order[2:]
    bottom = np.zeros(8)
    for k, col in enumerate(("tab:red", "tab:purple", "tab:cyan")):
        v = np.array([((lab == f) & (blk == k)).sum() for f in eq])
        ax[2].bar(range(8), v, bottom=bottom, color=col, label="block %d (8v / 8s / 8c)" % k)
        bottom += v
    ax[2].set_xticks(range(8)); ax[2].set_xticklabels(["eq %d" % k for k in range(1, 9)], fontsize=8)
    ax[2].set_ylabel("roots"); ax[2].set_ylim(0, 32); ax[2].set_title("triality blocks: 8 + 8 + 8 in every equatorial fibre"); ax[2].legend(fontsize=8, ncol=3, loc="upper center")
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
    figure(os.path.join(d, "e6_fibres.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
