#!/usr/bin/env python3
"""core.py - alphabets, flux and exact counts of closed timelines for toymodel_v2 (Stage 0).

A timeline is a list of steps, each a letter of an alphabet of six signed directions.  Two alphabets are used:

  HEX    the six hexagonal directions p + q w (w = e^{i pi/3}) written as integer pairs (p, q).  These are the red A2 roots of the Spectre lift,
         the six directions that lie in E8 (quantgrav.tex, Result 'hexdirs').  Planar norm^2 of (p, q) is p^2 + p q + q^2; the E8 inner product
         is 2 x the planar one.
  CUBIC  +-X, +-Y, +-Z as integer triples.  The cubic set is not in E8; it is kept as a control (it is bipartite: no closed timeline of odd length).

The flux of a timeline is the vector sum of its steps; a timeline is *closed* when its flux is zero.

Definitions D0.1  alphabet = tuple of integer vectors; flux = their sum; closed = zero flux.
Exact claims (each is a test below, ``python3 core.py --tests-only``)
  E0.1  The dynamic-programming count of closed timelines equals brute-force enumeration for n <= 8, for both alphabets.
  E0.2  The counts for n = 0..8 are  hex   1, 0, 6, 12, 90, 360, 2040, 10080, 54810
                                       cubic 1, 0, 6, 0, 90, 0, 1860, 0, 44730
        These are the numbers of closed walks on the triangular and the simple cubic lattice; the same numbers are in the ledger of the
        Quantum-Gravity-Dynamics fork (directional/LEDGER.md).  They were typed in here from that ledger, not computed from it.
  E0.3  Cubic: no closed timeline of odd length, |flux|^2 has the parity of n.  Hex: 12 closed timelines of length 3.
  E0.4  The hexagonal steps are the red steps of lift4d.step(), e0 - e1 + e2 = 0 (the red A2), and consecutive roots at angle 60 degrees have
        planar inner product 1/2 (E8 inner product 1).
  E0.5  The tests detect a wrong alphabet (mutation checks).
"""
import itertools
import math
import os
import sys

import numpy as np

HEX = ((1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1))
CUBIC = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
ALPHABETS = {"hex": HEX, "cubic": CUBIC}

LEDGER_HEX = (1, 0, 6, 12, 90, 360, 2040, 10080, 54810)
LEDGER_CUBIC = (1, 0, 6, 0, 90, 0, 1860, 0, 44730)


def alphabet(name_or_tuple):
    if isinstance(name_or_tuple, str):
        if name_or_tuple not in ALPHABETS:
            raise ValueError("alphabet must be one of %s" % sorted(ALPHABETS))
        return ALPHABETS[name_or_tuple]
    return tuple(tuple(int(c) for c in v) for v in name_or_tuple)


def flux(seq, alph="hex"):
    """Vector sum of the steps of a timeline given as a sequence of indices into the alphabet."""
    A = alphabet(alph)
    dim = len(A[0])
    tot = [0] * dim
    for k in seq:
        for i in range(dim):
            tot[i] += A[k][i]
    return tuple(tot)


def planar_norm2_hex(v):
    """Planar norm^2 of p + q w."""
    p, q = v
    return p * p + p * q + q * q


def check_n(n, nmax=12):
    if int(n) != n or n < 0 or n > nmax:
        raise ValueError("n must be an integer in [0, %d]" % nmax)
    return int(n)


def closed_counts(alph="hex", n_max=8):
    """Exact numbers of closed timelines of n = 0..n_max steps, by dynamic programming over the flux."""
    A = alphabet(alph)
    n_max = check_n(n_max, 40)
    dim = len(A[0])
    table = {tuple([0] * dim): 1}
    out = [1]
    for _ in range(n_max):
        nxt = {}
        for f, c in table.items():
            for s in A:
                g = tuple(f[i] + s[i] for i in range(dim))
                nxt[g] = nxt.get(g, 0) + c
        table = nxt
        out.append(table.get(tuple([0] * dim), 0))
    return out


def flux_distribution(alph, n):
    """dict flux -> number of timelines of n steps with that flux (dynamic programming)."""
    A = alphabet(alph)
    n = check_n(n, 40)
    dim = len(A[0])
    table = {tuple([0] * dim): 1}
    for _ in range(n):
        nxt = {}
        for f, c in table.items():
            for s in A:
                g = tuple(f[i] + s[i] for i in range(dim))
                nxt[g] = nxt.get(g, 0) + c
        table = nxt
    return table


def all_timelines(alph, n):
    """All 6^n timelines of n steps as an (6^n, n) int8 array of alphabet indices (n <= 8)."""
    n = check_n(n, 8)
    K = len(alphabet(alph))
    if n == 0:
        return np.zeros((1, 0), dtype=np.int8)
    grids = np.indices((K,) * n, dtype=np.int8).reshape(n, -1).T
    return grids


def closed_timelines(alph, n):
    """The closed timelines of n steps as an (N, n) int8 array (n <= 8), in lexicographic order."""
    A = np.array(alphabet(alph))
    T = all_timelines(alph, n)
    if n == 0:
        return T
    f = A[T].sum(axis=1)
    return T[np.all(f == 0, axis=1)]


def closed_count_brute(alph, n):
    return int(len(closed_timelines(alph, n)))


# ------------------------------------------------------------------------------------------------ tests

def _import_lift4d():
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    if root not in sys.path:
        sys.path.insert(0, root)
    import lift4d
    return lift4d


def test_dp_equals_brute_force_for_both_alphabets():
    for name in ("hex", "cubic"):
        dp = closed_counts(name, 8)
        for n in range(0, 9):
            assert dp[n] == closed_count_brute(name, n), (name, n)


def test_counts_equal_the_ledger():
    assert tuple(closed_counts("hex", 8)) == LEDGER_HEX
    assert tuple(closed_counts("cubic", 8)) == LEDGER_CUBIC


def test_parity_and_odd_closed_timelines():
    c = closed_counts("cubic", 9)
    assert all(c[n] == 0 for n in range(1, 10, 2))
    assert closed_counts("hex", 3)[3] == 12
    T = all_timelines("cubic", 6)
    f = np.array(CUBIC)[T].sum(axis=1)
    assert np.all((f * f).sum(axis=1) % 2 == 0)            # n = 6 even: |flux|^2 even
    T = all_timelines("cubic", 5)
    f = np.array(CUBIC)[T].sum(axis=1)
    assert np.all((f * f).sum(axis=1) % 2 == 1)            # n = 5 odd: |flux|^2 odd
    # the twelve closed hexagonal 3-step timelines are the two triangles (counter- and clockwise) started at any of the 3 corners x ... = 12
    tl = closed_timelines("hex", 3)
    assert len(tl) == 12 and len({tuple(r) for r in tl}) == 12


def test_hex_alphabet_is_the_red_a2_of_the_lift():
    L = _import_lift4d()
    for k in range(6):
        p, q, r, s = L.step(2 * k)
        assert (r, s) == (0, 0) and (p, q) == HEX[k]
    assert L.HEX == list(HEX)
    e = HEX
    assert (e[0][0] - e[1][0] + e[2][0], e[0][1] - e[1][1] + e[2][1]) == (0, 0)
    assert all(planar_norm2_hex(v) == 1 for v in HEX)
    for k in range(6):                                      # neighbours at 60 degrees: planar inner product 1/2, so E8 inner product 1
        a, b = HEX[k], HEX[(k + 1) % 6]
        assert planar_norm2_hex((a[0] + b[0], a[1] + b[1])) == 3          # |a + b|^2 = 1 + 1 + 2 <a, b>  = 3  ->  <a, b> = 1/2
        assert planar_norm2_hex((a[0] + HEX[(k + 3) % 6][0], a[1] + HEX[(k + 3) % 6][1])) == 0   # opposite roots cancel


def test_closed_counts_follow_the_known_recurrence_signs():
    # independent check of the first nontrivial terms: closed walks of 2 steps = number of directions
    assert closed_counts("hex", 2)[2] == 6 and closed_counts("cubic", 2)[2] == 6
    # flux distribution is symmetric under negation and sums to 6^n
    for name in ("hex", "cubic"):
        d = flux_distribution(name, 5)
        assert sum(d.values()) == 6 ** 5
        assert all(d[tuple(-x for x in f)] == c for f, c in d.items())


def test_mutations_are_detected():
    bad_hex = ((1, 0), (0, 1), (1, -1), (-1, 0), (0, -1), (-1, 1))         # same set in another order: same counts (must NOT be detected)
    assert closed_counts(bad_hex, 6) == closed_counts("hex", 6)
    sheared = ((1, 0), (0, 1), (1, 1), (-1, 0), (0, -1), (-1, -1))         # the same triangular lattice in a sheared basis: same counts (must NOT be detected)
    assert closed_counts(sheared, 6) == closed_counts("hex", 6)
    bad = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1))              # a different step set (square lattice plus two diagonals)
    assert closed_counts(bad, 4) != closed_counts("hex", 4)
    five = HEX[:5]
    assert closed_counts(five, 6) != closed_counts("hex", 6)
    assert closed_counts(CUBIC, 3) != closed_counts("hex", 3)      # n = 4 is not enough: both give 90


def test_validation():
    for bad in (-1, 2.5, 41):
        try:
            closed_counts("hex", bad)
        except ValueError:
            continue
        raise AssertionError("closed_counts accepted %r" % (bad,))
    try:
        closed_counts("nope", 3)
    except ValueError:
        pass
    else:
        raise AssertionError("unknown alphabet accepted")
    try:
        all_timelines("hex", 9)
    except ValueError:
        pass
    else:
        raise AssertionError("all_timelines accepted n = 9")


def run_tests(verbose=True):
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            if verbose:
                print("  %-70s PASS" % name)
        except AssertionError as e:
            failed += 1
            print("  %-70s FAIL %s" % (name, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = run_tests()
    if "--tests-only" in argv or failed:
        return 1 if failed else 0
    for name in ("hex", "cubic"):
        print("closed timelines, %-5s n=0..8: %s" % (name, closed_counts(name, 8)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
