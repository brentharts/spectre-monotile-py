#!/usr/bin/env python3
"""t7_layout.py - Stage 3 of toymodel_v7: which layouts of the Chair44's 192 tips pass rules stated in advance.

v5 used 192 = 3 x 42 + 3 x 14 + 4 x 6 (three curved Spectre tiles, three straight tiles, four groups of the six points the bit moves) and labelled it a Hypothesis.  Here the point counts and the rules are fixed from the
tile model and the chair before the count is run, and every solution is enumerated.

Point counts a tile supplies (fixed in advance)
  Spectre end (tau = 0, curved, P8 present):  42 (14 vertices + 28 handles), 28 (handles only), 14 (vertices only), 6 (the group around P8).
  Hat end (tau = 1, straight, P8 gone: 13 corners, SPECTRE_HAT_FLOW.md):   13.
  The 14-vertex straight tile is the one v5 used for its Hat layer (b = sqrt3, P8 flat, h = 0).
Live bits: a tile carries a live bit if it is at the Spectre end (types 42, 28) or is a group (6); types 14 and 13 are silent (h = 0, v5 E5.16).

Rules (fixed in advance)
  R1  The counts add to 192.
  R2  The chair's three-fold axis (1,1,1), its only rotation (v5 E5.3).  The Spectre has no rotational symmetry, so a tile is never fixed by it: each tile type's count is a multiple of 3.  At most one group per side of the cube,
      and the rotation permutes the six sides in two orbits of three (+x +y +z, -x -y -z), so the number of groups is 0, 3 or 6.
  R3  The pair rule (v4, v6): the number of live bits is even.

Exact claims (each a test; ``python3 t7_layout.py --tests-only``)
  E7.9   R1 has 126 solutions in the five types.  v5's layout (3, 0, 3, 0, 4) is one of them and fails R2 (four groups) and R3 (seven live bits).  With only v5's three types (42, 14, 6) no solution passes R2.
  E7.10  R2 alone selects one layout: 12 Hat tiles of 13 corners and 6 groups, one on every side (0, 0, 0, 12, 6).  It passes R3 automatically (6 live bits).  Without the group clause of R2, 7 solutions remain.
  E7.11  The layout is neutral.  Pairing the six groups into bump-and-dent pairs compatibly with the rotation can be done in 3 ways, one of them antipodal (+x with -x, ...).  For every such pairing the net bit source is 0;
         with the bump on every + side the antipodal one has bit dipole 2 h R (1, 1, 1), exactly along the chair's own axis.  So the layout that the rules select has no monopole, and the 1/r clock of v5 Stage 5 (which came from an
         odd, unpaired live bit) is not available in it.  **Negative** for C7 with the selected layout.

Status.  All Exact (enumeration).  That the point counts and rules are the right ones is Hypothesis H7.4 of PLAN.md; they were written down before the enumeration, and the 13-corner type is the one that makes a solution exist.

    python3 t7_layout.py --tests-only
    python3 t7_layout.py --no-show --save figures/t7_stage3_layout.png
"""
import argparse
import itertools
import sys

import numpy as np

TYPES = ("curved42", "handles28", "straight14", "hat13", "group6")
POINTS = (42, 28, 14, 13, 6)
LIVE = (1, 1, 0, 0, 1)
SIDES = ("+x", "+y", "+z", "-x", "-y", "-z")
NORMAL = {"+x": (1, 0, 0), "+y": (0, 1, 0), "+z": (0, 0, 1), "-x": (-1, 0, 0), "-y": (0, -1, 0), "-z": (0, 0, -1)}
ROT = {"+x": "+y", "+y": "+z", "+z": "+x", "-x": "-y", "-y": "-z", "-z": "-x"}       # the three-fold rotation about (1,1,1)
V5 = (3, 0, 3, 0, 4)


def solutions(total=192):
    out = []
    ranges = [range(total // p + 1) for p in POINTS]
    for s in itertools.product(*ranges):
        if sum(n * p for n, p in zip(s, POINTS)) == total:
            out.append(s)
    return out


def r2(s, group_clause=True):
    tiles_ok = all(n % 3 == 0 for n in s[:4])
    return tiles_ok and (s[4] in (0, 3, 6) if group_clause else s[4] <= 6)


def r3(s):
    return sum(n * l for n, l in zip(s, LIVE)) % 2 == 0


def invariant_pairings():
    """Perfect matchings of the six sides into (bump, dent) pairs that the rotation maps to themselves."""
    out = []
    sides = list(SIDES)

    def matchings(rest):
        if not rest:
            yield []
            return
        a = rest[0]
        for b in rest[1:]:
            r = [x for x in rest if x not in (a, b)]
            for m in matchings(r):
                yield [(a, b)] + m

    for m in matchings(sides):
        ms = {frozenset(p) for p in m}
        if {frozenset(ROT[x] for x in p) for p in ms} == ms:
            out.append(m)
    return out


def bit_moments(pairing, h=1.0, R=1.0):
    """Net source and dipole when the first side of each pair carries +h and the second -h, at distance R."""
    q = {}
    for a, b in pairing:
        q[a], q[b] = +h, -h
    mono = sum(q.values())
    dip = sum(q[s] * R * np.array(NORMAL[s], float) for s in SIDES)
    return mono, dip


# --------------------------------------------------------------------------
def test_e79_v5_layout_fails_the_rules():
    sols = solutions()
    assert len(sols) == 126 and V5 in sols
    assert not r2(V5) and not r3(V5)
    v5types = [s for s in sols if s[1] == 0 and s[3] == 0]
    assert not [s for s in v5types if r2(s)]
    return len(sols), len(v5types)


def test_e710_the_rotation_selects_one_layout():
    sols = solutions()
    sel = [s for s in sols if r2(s)]
    assert sel == [(0, 0, 0, 12, 6)] and r3(sel[0])
    assert len([s for s in sols if r2(s, group_clause=False)]) == 7
    assert sum(r3(s) for s in sols) == 64
    return sel[0]


def test_e711_the_selected_layout_is_neutral():
    P = invariant_pairings()
    assert len(P) == 3
    anti = [m for m in P if all(NORMAL[a] == tuple(-x for x in NORMAL[b]) for a, b in m)]
    assert len(anti) == 1
    for m in P:
        mono, dip = bit_moments(m)
        assert mono == 0
    # orient every pair of the antipodal matching with the bump on the + side
    m = [(a, b) if a.startswith("+") else (b, a) for a, b in anti[0]]
    _, dip = bit_moments(m)
    assert np.allclose(dip, 2 * np.ones(3))
    return [sorted(map(sorted, x)) for x in P]


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

    sols = solutions()
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.6), gridspec_kw={"width_ratios": [1, 1.3]})
    labels = ["R1: total 192", "R1 + R3: even live bits", "R1 + R2 without group clause", "R1 + R2", "R1 + R2 + R3"]
    counts = [len(sols), sum(r3(s) for s in sols), sum(r2(s, False) for s in sols), sum(r2(s) for s in sols), sum(r2(s) and r3(s) for s in sols)]
    ax[0].barh(range(len(labels))[::-1], counts, color=["0.6", "0.6", "tab:blue", "tab:green", "tab:green"])
    for i, c in enumerate(counts):
        ax[0].text(c + 1, len(labels) - 1 - i, str(c), va="center", fontsize=8)
    ax[0].set_yticks(range(len(labels))[::-1]); ax[0].set_yticklabels(labels, fontsize=8)
    ax[0].set_xscale("log"); ax[0].set_xlabel("layouts that pass")
    ax[0].set_title("(a) the chair's three-fold axis leaves one layout", fontsize=9)
    ax[1].axis("off")
    txt = ("point counts (fixed in advance):  42 curved, 28 handles, 14 straight,\n"
           "  13 Hat corners (P8 gone at tau = 1), 6 group around P8\n\n"
           "v5's layout 3 x 42 + 3 x 14 + 4 x 6:\n"
           "  fails R2 (4 groups) and R3 (7 live bits)\n"
           "  with only the types 42, 14, 6: no layout passes R2\n\n"
           "the selected layout  12 x 13 + 6 x 6 = 192:\n"
           "  12 Hat tiles (four orbits of the rotation)\n"
           "  6 groups, one on every side of the cube\n"
           "  6 live bits: even, passes R3 by itself\n\n"
           "pairings of the six groups the rotation respects: 3,\n"
           "  one antipodal; net bit source 0 for each;\n"
           "  antipodal dipole along (1,1,1), the chair's axis\n"
           "  -> no monopole: the 1/r clock of v5 is not available")
    ax[1].text(0.0, 1.0, txt, va="top", family="monospace", fontsize=8.5)
    fig.suptitle("toymodel_v7 Stage 3: layouts of the Chair44's 192 tips under rules stated in advance")
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
