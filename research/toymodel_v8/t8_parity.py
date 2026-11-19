#!/usr/bin/env python3
"""t8_parity.py - Stage 7 of toymodel_v8: the orientation-parity classes of the Spectre (qwla / Batle and Bednorz) against our patches and windows.

Source read: github.com/panjea/qwla (README.md, SPECTRE_QEC_CODES.md, SPECTRE_QUANTUM_UNIVERSE.md).  The claims it quotes from Batle and Bednorz (arXiv:2607.15326) were not read in full by us
(only a search snippet of its Section 4.4, which agrees); here they are tested on our own patches.

Definitions
  D8.2  Orientation index of a tile: m = round(angle / 30 degrees) mod 12 of its placement; parity = m mod 2.
Exact claims (each a test; ``python3 t8_parity.py --tests-only``)
  E8.1  Patch sizes 1, 9, 71, 559, 4401, 34649 (levels 0 to 5) obey N_k = 8 N_{k-1} - N_{k-2}, as the qwla README says.
  E8.2  In our patches (reference orientation of spectre_patch) the minority parity is exactly the Mystic-shaped tiles: even count = number of shape-1 tiles at every level 1 to 5 (1, 8, 63, 496, 3905).
  E8.3  even(L) = odd(L-1) for L >= 1 (the minority at level L is the majority one level down), so odd/even -> 4 + sqrt15 and f_minority -> (5 - sqrt15)/10 = 0.11270; at level 5 within 1e-4.
  E8.5  (H8.7) No integer functional with |l_i| <= 3 on the lift has a bounded range as the patch grows (range grows by > 1.3 per level, > 8 at level 4), while the real bounded plane of lift4d grows by < 10% per level.\n  E8.4  The qwla note's gap Delta = |K| sqrt15/5 is quoted as 0.7483 |K|; sqrt15/5 = 0.7746.  The formula f_even - f_odd = sqrt15/5 is right, the decimal in the note is not.
"""
import math
import sys
import numpy as np

sys.path.insert(0, __file__.rsplit("/research/", 1)[0] if "/research/" in __file__ else ".")
import spectre_patch as sp

S15 = math.sqrt(15.0)


def parity_counts(L):
    P = sp.SpectrePatch(L)
    m = P.placements
    ang = np.degrees(np.arctan2(m[:, 1, 0], m[:, 0, 0]))
    assert np.allclose(ang / 30.0, np.round(ang / 30.0), atol=1e-6)
    idx = np.round(ang / 30.0).astype(int) % 12
    par = idx % 2
    return dict(N=P.n_tiles, even=int((par == 0).sum()), odd=int((par == 1).sum()), mystic=int((P.shape == 1).sum()),
                even_is_mystic=bool(np.array_equal(par == 0, P.shape == 1)))


def test_e81_patch_sizes_follow_the_recurrence():
    N = [parity_counts(L)["N"] for L in range(6)]
    assert N == [1, 9, 71, 559, 4401, 34649], N
    assert all(N[k] == 8 * N[k - 1] - N[k - 2] for k in range(2, 6))
    return N


def test_e82_minority_parity_is_the_mystic_shaped_tiles():
    out = []
    for L in range(1, 6):
        c = parity_counts(L)
        assert c["even_is_mystic"] and c["even"] == c["mystic"], c
        out.append(c["even"])
    assert out == [1, 8, 63, 496, 3905], out
    return out


def test_e83_minority_count_is_the_previous_majority_and_the_frequency_converges():
    c = [parity_counts(L) for L in range(6)]
    for L in range(1, 6):
        assert c[L]["even"] == c[L - 1]["odd"], L          # even(L) = odd(L-1)
    f = c[5]["even"] / c[5]["N"]
    assert abs(f - (5 - S15) / 10) < 1e-4, f
    assert abs(c[5]["odd"] / c[5]["even"] - (4 + S15)) < 5e-3
    return f


def test_e84_the_gap_decimal_in_the_note_is_wrong():
    assert abs(S15 / 5 - 0.7746) < 1e-4 and abs(S15 / 5 - 0.7483) > 0.02
    return S15 / 5


def test_e85_no_integer_coordinate_functional_has_a_bounded_primitive_but_a_real_plane_does():
    """H8.7: the closed 1-forms d(l) for l in (Z^4)*; l is a candidate for an exact (bounded-primitive) class iff its range on the lifted vertices stays bounded as the patch grows."""
    import itertools
    import lift4d as L
    V = {}
    for lev in (3, 4):
        _, lifted, _ = L.lift_patch(lev)
        V[lev] = np.array(list(lifted.values()), float)
        V[lev] -= V[lev].mean(0)
    S, _ = L.bounded_plane(4)
    plane = [np.abs(V[lev] @ S.T).max() for lev in (3, 4)]
    assert plane[1] / plane[0] < 1.1, plane                         # the real internal plane: range grows by < 10% per level
    worst = []
    for l in itertools.product(range(-3, 4), repeat=4):
        if any(l):
            r = [np.abs(V[lev] @ np.array(l)).max() for lev in (3, 4)]
            worst.append((r[1] / r[0], r[1], l))
    # every integer functional with |l_i| <= 3 has a range that grows by at least 1.3 per level and exceeds 8 at level 4
    assert min(w[0] for w in worst) > 1.3, min(worst)
    assert min(w[1] for w in worst) > 8, min(worst, key=lambda w: w[1])
    return plane, min(worst)


def run_tests(verbose=True):
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    bad = 0
    for t in tests:
        try:
            r = t()
            if verbose: print("  ok  ", t.__name__, r)
        except AssertionError as e:
            bad += 1; print("  FAIL", t.__name__, e)
    print("%d tests, %d failed" % (len(tests), bad))
    return bad


if __name__ == "__main__":
    sys.exit(1 if run_tests() else 0)
