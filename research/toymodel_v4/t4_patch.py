#!/usr/bin/env python3
"""t4_patch.py - Stage 2 of research/toymodel_v4/PLAN.md: many bits on a patch of real outlines (the pairing rule on Bezier curves, and what the flow does to the number of bit states).

toymodel_v2 (tm2_lattice.py E6, E7) found the pairing rule on the STRAIGHT 14-gons of a patch: the flat vertex P_8 of a tile can open only together with the flat vertex of exactly one other tile
('a flat pair'), as a bump and a dent.  Here the tiles are the real outlines of spectremodel.Tile (Bezier edges, curve strength k, corner opening h = a tan(omega / 2)) placed by the rigid
motions of spectre_patch.SpectrePatch, and seams are compared as Bezier curves (four control points, one edge against the other reversed).

Definitions
  D2.1  Patch: SpectrePatch(levels, 1, b).  Tile i is spectremodel.Tile(*patch.tile_params(i), k) (the Mystic tiles have (a, b) swapped), placed by patch.placements[i].  Both tile codes
        have the same 14 vertices in the same order, and the flat vertex is index 8 in both (checked in E2.0).
  D2.2  Opening a tile with bit s: its vertex 8 is moved by s h (1, 0) in the tile's own frame, h = tan(omega / 2) (a = 1); s = +1 is the convex corner of SpectreHatTime, s = -1 the reflex one.
        The curves of edges 7 and 8 follow their new chords, every other edge is untouched.
  D2.3  Seam: a pair of meeting edges (patch.edge_pairs).  It is closed if the four control points of one edge, in order, equal those of the other reversed (tolerance 1e-9).
  D2.4  Flat pair: two tiles whose vertex 8 coincide in the patch and with no other tile at that point.

Exact claims (each a test; ``python3 t4_patch.py --tests-only``)
  E2.0  The tile code of the patch (spectre_operator) and of the flow (spectremodel) agree vertex by vertex for b in {1, sqrt3} and have the flat vertex at index 8; moving that vertex changes
        the Bezier control points of edges 7 and 8 only.
  E2.1  Flat pairs on the real patch: level 2 has 6, level 3 has 82 (as in tm2_lattice) and level 4 has 804.  For every flat pair of the level-3 patch, for (b, k, omega) in a small grid, opening the pair as
        (s_A, s_B) = (+1, -1) or (-1, +1) leaves every seam closed, and opening it as (+1, +1) or (-1, -1) breaks exactly two seams: the pair rule holds on the curves.
  E2.2  A tile outside a flat pair cannot open alone: for 60 tiles of the level-3 patch that have other tiles at their vertex 8 and are not in a pair, a single opening (either sign) breaks at
        least two seams.  So the bits live on the 164 tiles of the 82 flat pairs, 29.3 per cent of the 559 tiles (16.9, 29.3, 36.5 per cent at levels 2, 3, 4).  The 89 tiles of the level-3
        boundary with nothing at vertex 8 open without breaking any seam: that is the edge of the patch, not a bit, and is left out.
  E2.3  The area is a transfer: opening a pair as (+1, -1) changes the area of A by +h and of B by -h, and of the patch by 0 (E1.1 of t4_arrow.py, now on the patch); opening both alike changes
        the patch area by +-2 h.
  E2.4  What a pair can do, as a table of valid states: of the four sign assignments (s_A, s_B) exactly two close the seams, (+1, -1) and (-1, +1), the two-dimensional S_z = 0 sector of two
        two-level systems, which contains the singlet and the triplet state with m = 0; each of the two states has one convex and one reflex tile, so it is valid (both outlines simple) iff omega is
        below the threshold omega_c(b, k) of Stage 3, and the two states coincide with each other (and with the closed pair) iff omega = 0.
  E2.5  The number of bits along the flow.  Let B(tau) be the number of flat pairs that have two valid, distinct open states (0 < omega < omega_c).  For omega_0 = 30 degrees B = 82 for every tau < 1 and 0
        at tau = 1 (level 3).  For omega_0 = 150 degrees (k_0 = 0.5) B = 0 until the flow enters the valid region, is 82 after, and is 0 at tau = 1.  So the log2 of the number of open configurations,
        B(tau), is not monotone in the second case, and decreases to 0 at the Hat in both.

  E2.6  The density of flat pairs per tile is 0.1467, 0.1827, 0.2024, 0.2130 at levels 3 to 6; the boundary correction falls by the perimeter-to-area ratio nu/lambda^2 = 0.538 per level, and
        extrapolating from levels 4 and 5 gives 0.2253, the density rho_0 = (5 - sqrt 15)/5 of valence-4 vertices per tile, to within 1e-4.
  E2.7  (Exact given the recurrence; the recurrence is Checked) The pair counts of the level-k patch, 0, 0, 6, 82, 804, 7012, 58106, 469766 for k = 0..7 (level 7 by t4_pairs_level7.py, outside the tests),
        satisfy the order-5 recurrence with characteristic polynomial (x - 1)(x^2 - 8x + 1)(x^2 - 4x - 1): the roots are those of the tile counts (mu = 4 + sqrt 15 and 1/mu) and of the perimeter law
        (nu = 2 + sqrt 5 = phi^3 and 2 - sqrt 5) and 1, as the supertile structure predicts (pairs inside the tiles of a supertile grow like mu^k, pairs at its boundary like nu^k).  It was fitted to levels
        0 to 4 and holds at levels 5, 6 and 7.  Its solution is
            P_k = (mu^k - mu'^k)/sqrt15 - (1/4 + 3 sqrt5/20) nu^k - (1/4 - 3 sqrt5/20) nu'^k + 1/2,   T_k = A mu^k + B mu'^k, A = (5 + sqrt15)/(2 sqrt15),
        so P_k / T_k -> 2 / (5 + sqrt 15) = (5 - sqrt 15)/5 = rho_0 exactly, with correction of order (nu/mu)^k.  A proof of the recurrence needs the boundary of a supertile as a finite-state operator
        (spectre_boundary_operator.py of the Hat and Spectre repository): not done.
        No local bijection with valence-4 vertices exists in the simplest form: each valence-4 vertex carries exactly one corner of index 12, but only 697 of the 992 such tiles at level 4 lie in a pair.

Status.  The pairing rule is not an artefact of straight polygons: it holds on the curves, with no change.  The constraint leaves a two-state space per pair, so the tiling fixes which
configurations are allowed and not their amplitudes (the sign of the fusion stays an input, Stage 3).  The number of open configurations is maximal at the Spectre end and collapses at the Hat,
so a counting entropy decreases along the flow, opposite to the reading 'later = more entropy'; the other reading (information about the bit is lost toward the Hat) is the D6 direction.
Whether the toy's arrow is the first or the second is the conflict of toymodel_v2 (Result records) and of Stage 1; counting does not decide it.  **Hypothesis**, not tested: that a bit per flat pair,
with the area transfer of E2.3 as its conserved charge, is a unit of anything physical.

    python3 t4_patch.py --tests-only
    python3 t4_patch.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import spectre_operator as so  # noqa: E402
import spectre_patch as sp  # noqa: E402
from spectre_hat_time_model import is_simple_tile  # noqa: E402
from spectremodel import Tile  # noqa: E402

FLAT = 8
SQRT3 = math.sqrt(3.0)
TOL = 1e-9
_CACHE = {}


def patch(levels, b):
    key = ("patch", levels, round(b, 12))
    if key not in _CACHE:
        _CACHE[key] = sp.SpectrePatch(levels, 1.0, b)
    return _CACHE[key]


def _place(M, pts):
    p = np.asarray(pts, dtype=float).reshape(-1, 2)
    return (p @ M[:2, :2].T + M[:2, 2]).reshape(np.shape(pts))


def tile_cp(P, i, k, s=0, h=0.0):
    """D2.1, D2.2: global control points (14, 4, 2) of tile i of patch P with curve strength k; s = +-1 opens vertex 8 by s h in the tile's frame."""
    t = Tile(*P.tile_params(i), k)
    if s:
        v = t.vertices[FLAT]
        v.position = v.position + np.array([s * h, 0.0])
    return _place(P.placements[i], np.asarray(t.control_points()))


def base_cps(levels, b, k):
    key = ("cp", levels, round(b, 12), round(k, 12))
    if key not in _CACHE:
        P = patch(levels, b)
        _CACHE[key] = [tile_cp(P, i, k) for i in range(P.n_tiles)]
    return _CACHE[key]


def seam_rows_by_tile(levels, b):
    key = ("rows", levels, round(b, 12))
    if key not in _CACHE:
        P = patch(levels, b)
        by = {}
        for r in P.edge_pairs():
            by.setdefault(int(r[0]), []).append(tuple(int(x) for x in r))
            by.setdefault(int(r[2]), []).append(tuple(int(x) for x in r))
        _CACHE[key] = by
    return _CACHE[key]


def broken_seams(levels, b, k, opened):
    """D2.3: list of broken seams (A, i, B, j) after opening the tiles in ``opened`` = {tile: (s, h)}; only seams that touch an opened tile are examined (the others are untouched)."""
    P = patch(levels, b)
    base = base_cps(levels, b, k)
    cp = {t: tile_cp(P, t, k, s, h) for t, (s, h) in opened.items()}
    get = lambda t: cp[t] if t in cp else base[t]
    seen, bad = set(), []
    by = seam_rows_by_tile(levels, b)
    for t in opened:
        for (A, i, B, j) in by.get(t, []):
            if (A, i, B, j) in seen:
                continue
            seen.add((A, i, B, j))
            if not np.allclose(get(A)[i], get(B)[j][::-1], atol=TOL):
                bad.append((A, i, B, j))
    return bad


def free_flat_tiles(levels, b):
    """Tiles with no other tile at their vertex 8: on the boundary of the patch, where opening breaks no seam (a boundary artefact, not a bit)."""
    P = patch(levels, b)
    polys = [P.straight_polygon(i) for i in range(P.n_tiles)]
    at = {}
    for t, poly in enumerate(polys):
        for v in range(14):
            at.setdefault((round(poly[v][0], 6) + 0.0, round(poly[v][1], 6) + 0.0), []).append(t)
    return [t for t, poly in enumerate(polys) if at[(round(poly[FLAT][0], 6) + 0.0, round(poly[FLAT][1], 6) + 0.0)] == [t]]


def flat_pairs(levels, b):
    """D2.4: list of (A, B), A < B: tiles whose vertex 8 meets another tile's vertex 8 alone."""
    key = ("pairs", levels, round(b, 12))
    if key not in _CACHE:
        P = patch(levels, b)
        polys = [P.straight_polygon(i) for i in range(P.n_tiles)]
        at = {}
        for t, poly in enumerate(polys):
            for v in range(14):
                at.setdefault((round(poly[v][0], 6) + 0.0, round(poly[v][1], 6) + 0.0), []).append((t, v))
        pairs = []
        for t, poly in enumerate(polys):
            q = (round(poly[FLAT][0], 6) + 0.0, round(poly[FLAT][1], 6) + 0.0)
            oth = [x for x in at[q] if x[0] != t]
            if len(oth) == 1 and oth[0][1] == FLAT and t < oth[0][0]:
                pairs.append((t, oth[0][0]))
        _CACHE[key] = pairs
    return _CACHE[key]


def tile_area(P, i, k, s=0, h=0.0):
    t = Tile(*P.tile_params(i), k)
    if s:
        v = t.vertices[FLAT]
        v.position = v.position + np.array([s * h, 0.0])
    return t.area()


def reflex_valid(b, k, omega_deg):
    """Is the s = -1 outline of Tile(1, b, k) at corner angle omega a simple closed curve (Stage 3 / E0.4)?"""
    h = math.tan(math.radians(omega_deg) / 2.0)
    t = Tile(1.0, b, k)
    v = t.vertices[FLAT]
    v.position = v.position + np.array([-h, 0.0])
    return is_simple_tile(t)


def open_states(b, k, omega_deg):
    """E2.4: the valid distinct open states of a flat pair at (b, k, omega): [] (not valid), or the list of (s_A, s_B) in {(+1, -1), (-1, +1)} (empty at omega = 0: the pair is closed)."""
    if omega_deg <= 0.0:
        return []
    return [(1, -1), (-1, 1)] if reflex_valid(b, k, omega_deg) else []


def bits_along_flow(levels, omega0, k0, n=21, b0=1.0):
    """E2.5: (taus, B(tau)): number of flat pairs of the patch that carry two valid distinct open states, with the linear laws of the flow."""
    taus = np.linspace(0.0, 1.0, n)
    npairs = len(flat_pairs(levels, b0))
    out = []
    for t in taus:
        b, k, om = 1.0 + (SQRT3 - 1.0) * t, k0 * (1.0 - t), omega0 * (1.0 - t)
        out.append(npairs if open_states(b, k, om) else 0)
    return taus, np.array(out)


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e20_the_two_tile_codes_agree_and_only_edges_7_and_8_follow_vertex_8():
    cr = lambda u, v: u[0] * v[1] - u[1] * v[0]
    for b in (1.0, SQRT3):
        V = np.array(so.vertices(1.0, b))
        T = np.array([v.position for v in Tile(1.0, b, 0.5).vertices])
        assert np.allclose(V, T, atol=1e-12)
        flat = [j for j in range(14) if abs(cr(V[j] - V[j - 1], V[(j + 1) % 14] - V[j])) < 1e-9 and np.dot(V[j] - V[j - 1], V[(j + 1) % 14] - V[j]) > 0]
        assert flat == [FLAT], flat
    t1, t2 = Tile(1.0, 1.0, 0.5), Tile(1.0, 1.0, 0.5)
    v = t2.vertices[FLAT]
    v.position = v.position + np.array([0.2, 0.0])
    c1, c2 = np.asarray(t1.control_points()), np.asarray(t2.control_points())
    assert [e for e in range(14) if not np.allclose(c1[e], c2[e])] == [7, 8]


def test_e21_the_pairing_rule_holds_on_the_curves():
    assert len(flat_pairs(2, 1.0)) == 6 and len(flat_pairs(3, 1.0)) == 82 and len(flat_pairs(3, SQRT3)) == 82 and len(flat_pairs(4, 1.0)) == 804
    for (b, k, om) in ((1.0, 0.5, 30.0), (SQRT3, 0.3, 60.0), (1.4, 0.8, 110.0)):
        h = math.tan(math.radians(om) / 2.0)
        for (A, B) in flat_pairs(3, b):
            for sA, sB in ((1, -1), (-1, 1)):
                assert broken_seams(3, b, k, {A: (sA, h), B: (sB, h)}) == [], (b, k, om, A, B, sA, sB)
            for sA in (1, -1):
                bad = broken_seams(3, b, k, {A: (sA, h), B: (sA, h)})
                assert len(bad) == 2, (b, k, om, A, B, sA, len(bad))


def test_e22_a_tile_outside_a_flat_pair_cannot_open_alone():
    levels, b, k, h = 3, 1.0, 0.5, 0.3
    P = patch(levels, b)
    in_pair = {t for pr in flat_pairs(levels, b) for t in pr}
    free = set(free_flat_tiles(levels, b))
    assert len(in_pair) == 164 and P.n_tiles == 559 and abs(100.0 * len(in_pair) / P.n_tiles - 29.3) < 0.05
    assert len(free) == 89 and not (free & in_pair), "boundary tiles with nothing at vertex 8"
    for t in sorted(free)[:10]:
        assert broken_seams(levels, b, k, {t: (1, h)}) == [], t  # at the free boundary a tile opens without breaking a seam
    tested = 0
    for t in range(P.n_tiles):
        if t in in_pair or t in free:
            continue
        for s in (1, -1):
            assert len(broken_seams(levels, b, k, {t: (s, h)})) >= 2, (t, s)
        tested += 1
        if tested >= 60:
            break
    assert tested == 60


def test_e23_the_area_is_a_transfer():
    levels = 3
    for b, k, om in ((1.0, 0.5, 30.0), (SQRT3, 0.4, 100.0)):
        P, h = patch(levels, b), math.tan(math.radians(om) / 2.0)
        for (A, B) in flat_pairs(levels, b)[:20]:
            a0, b0 = tile_area(P, A, k), tile_area(P, B, k)
            assert abs(tile_area(P, A, k, +1, h) - a0 - h) < 1e-9 and abs(tile_area(P, B, k, -1, h) - b0 + h) < 1e-9
            assert abs((tile_area(P, A, k, +1, h) + tile_area(P, B, k, -1, h)) - (a0 + b0)) < 1e-9
            assert abs((tile_area(P, A, k, +1, h) + tile_area(P, B, k, +1, h)) - (a0 + b0) - 2 * h) < 1e-9
            assert abs((tile_area(P, A, k, -1, h) + tile_area(P, B, k, -1, h)) - (a0 + b0) + 2 * h) < 1e-9


def test_e24_two_valid_states_per_pair_only_below_the_threshold():
    sector = {(+1, -1), (-1, +1)}
    P, b, k, h = patch(3, 1.0), 1.0, 0.5, 0.2
    A, B = flat_pairs(3, b)[0]
    closing = {(sA, sB) for sA, sB in itertools.product((1, -1), repeat=2) if not broken_seams(3, b, k, {A: (sA, h), B: (sB, h)})}
    assert closing == sector
    assert set(open_states(b, k, 30.0)) == sector and set(open_states(b, k, 120.0)) == sector
    assert open_states(b, k, 140.0) == [] and open_states(b, k, 170.0) == []
    assert open_states(b, k, 0.0) == []  # omega = 0: the states coincide with the closed pair
    for om in (1.0, 10.0):  # and for omega > 0 the two open outlines differ
        assert not np.allclose(tile_cp(P, A, k, +1, math.tan(math.radians(om) / 2)), tile_cp(P, A, k, -1, math.tan(math.radians(om) / 2)))


def test_e25_the_number_of_bits_along_the_flow():
    taus, b30 = bits_along_flow(3, 30.0, 0.5)
    assert np.all(b30[:-1] == 82) and b30[-1] == 0
    taus, b150 = bits_along_flow(3, 150.0, 0.5)
    assert b150[0] == 0 and b150[-1] == 0 and b150.max() == 82
    on = np.flatnonzero(b150 == 82)
    assert on.size and on[0] > 0 and np.all(b150[on[0]:-1] == 82), b150  # closed early, then a single block of 82, then 0 at the Hat
    assert not (np.all(np.diff(b150) <= 0) or np.all(np.diff(b150) >= 0)), "not monotone"


def count_pairs_fast(levels):
    """(tiles, flat pairs) of the level-``levels`` patch at b = 1 by sorting vertex keys (vectorised; same count as len(flat_pairs); the patch class is limited to 6 levels)."""
    P = patch(levels, 1.0)
    V0 = so.vertices(1.0, 1.0)  # at a = b = 1 the straight polygons of the Spectre and of the Mystic coincide
    M = P.placements
    X = np.einsum("nij,vj->nvi", M[:, :2, :2], V0) + M[:, None, :2, 2]
    K = np.rint(X * 1e5).astype(np.int64)
    key = (K[:, :, 0] + 10 ** 9) * (2 ** 31) + (K[:, :, 1] + 10 ** 9)
    flat = key.ravel()
    order = np.argsort(flat, kind="stable")
    fs = flat[order]
    vid = order % 14
    starts = np.flatnonzero(np.r_[True, fs[1:] != fs[:-1]])
    sizes = np.diff(np.r_[starts, len(fs)])
    two = starts[sizes == 2]
    return P.n_tiles, int(((vid[two] == FLAT) & (vid[two + 1] == FLAT)).sum())


# the recurrence of the pair counts (E2.7): characteristic polynomial (x - 1)(x^2 - 8x + 1)(x^2 - 4x - 1) = x^5 - 13 x^4 + 44 x^3 - 28 x^2 - 5 x + 1
PAIR_RECURRENCE = (1, -13, 44, -28, -5, 1)


def pair_closed_form(k):
    """Exact value (float) of P_k = (mu^k - mu'^k)/sqrt15 - (1/4 + 3 sqrt5/20) nu^k - (1/4 - 3 sqrt5/20) nu'^k + 1/2, mu = 4 + sqrt15, nu = 2 + sqrt5 (E2.7)."""
    r15, r5 = math.sqrt(15.0), math.sqrt(5.0)
    mu, mu2, nu, nu2 = 4.0 + r15, 4.0 - r15, 2.0 + r5, 2.0 - r5
    return (mu ** k - mu2 ** k) / r15 - (0.25 + 3.0 * r5 / 20.0) * nu ** k - (0.25 - 3.0 * r5 / 20.0) * nu2 ** k + 0.5


def pair_density_limit(levels=(4, 5)):
    """E2.6: estimate of the limit of (flat pairs per tile) from two consecutive levels, assuming the boundary correction falls by the perimeter-to-area ratio r = nu / lambda^2 per level,
    nu = phi^3 (perimeter law of the Spectre supertiles) and lambda^2 = 4 + sqrt 15 (area): f_inf = f_n + (f_n - f_(n-1)) r / (1 - r)."""
    r = (2.0 + math.sqrt(5.0)) / (4.0 + math.sqrt(15.0))
    f = [len(flat_pairs(lv, 1.0)) / patch(lv, 1.0).n_tiles for lv in levels]
    return f[-1] + (f[-1] - f[-2]) * r / (1.0 - r), f


def test_e26_the_density_of_flat_pairs_tends_to_the_density_of_valence_four_vertices():
    est, f = pair_density_limit()
    rho0 = (5.0 - math.sqrt(15.0)) / 5.0
    assert abs(f[0] - 804 / 4401) < 1e-12 and abs(f[1] - 7012 / 34649) < 1e-12
    assert abs(est - rho0) < 2e-4, (est, rho0)


def test_e27_the_pair_counts_obey_a_linear_recurrence_with_the_exact_limit_rho_0():
    series = [count_pairs_fast(lv) for lv in range(0, 7)]
    assert [p for _, p in series] == [0, 0, 6, 82, 804, 7012, 58106], series
    assert [len(flat_pairs(lv, 1.0)) for lv in (2, 3, 4)] == [6, 82, 804]
    c, T, Pn = PAIR_RECURRENCE, [t for t, _ in series], [p for _, p in series]
    for n in range(0, 2):  # order 5: five initial values, two independent checks (levels 5 and 6)
        assert sum(c[i] * Pn[n + 5 - i] for i in range(6)) == 0, n
    assert all(T[k + 2] == 8 * T[k + 1] - T[k] for k in range(5))
    for k in range(7):
        assert abs(pair_closed_form(k) - Pn[k]) < 1e-6, (k, pair_closed_form(k), Pn[k])
    mu = 4.0 + math.sqrt(15.0)
    A = (9.0 - (4.0 - math.sqrt(15.0))) / (mu - (4.0 - math.sqrt(15.0)))  # T_k = A mu^k + B mu'^k from T_0 = 1, T_1 = 9
    assert abs((1.0 / math.sqrt(15.0)) / A - (1.0 - math.sqrt(15.0) / 5.0)) < 1e-12, "limit P_k / T_k = 1 - sqrt15/5 = rho_0"


def run_tests():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        print("  %-86s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    for lv in (2, 3):
        P = patch(lv, 1.0)
        pr = flat_pairs(lv, 1.0)
        print("level %d: %d tiles, %d flat pairs, %.1f per cent of tiles can carry a bit" % (lv, P.n_tiles, len(pr), 200.0 * len(pr) / P.n_tiles))
    for om0 in (30.0, 150.0):
        taus, B = bits_along_flow(3, om0, 0.5)
        print("omega_0 = %3.0f: B(tau) at tau = %s -> %s" % (om0, np.round(taus[::4], 2), B[::4]))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ok = run_tests()
    if not ok or "--tests-only" in argv:
        return 0 if ok else 1
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
