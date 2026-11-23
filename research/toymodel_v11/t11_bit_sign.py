#!/usr/bin/env python3
"""t11_bit_sign.py - the relative sign of the bit at vertices other than P_8 (Open problem 4, second half).

Definition (toymodel_v4, t4_open.py D5.3).  A bit at vertex j displaces P_j by s h (cos phi, sin phi), s = +-1.  With turn_s(j)
the corner turn at P_j of the outline with value s, wrapped to (-pi, pi], Delta_j = turn_{-1}(j) - turn_{+1}(j) and the relative
spinor sign of the two values is exp(i Delta_j / 2).  It is -1 iff Delta_j = 2 pi (mod 4 pi).  t4_open.py showed for j = 8 that
every deformation with both outlines simple has |Delta| < 2 pi (largest 313 to 315 degrees).  Here: every other vertex.

Statements (each a test; ``python3 t11_bit_sign.py --tests-only``)
  B1 (Exact; elementary)  The corner turn of a simple outline at a vertex lies strictly inside (-pi, pi): +-pi is a cusp, a
        spike of zero opening, which is not simple.  So |Delta_j| < 2 pi for EVERY vertex of EVERY simple outline pair, and the
        relative sign -1 is never reached by moving one vertex.  The bound is sharp for arbitrary polygons (a spike and a notch
        reach Delta -> 2 pi) and not sharp inside the Spectre family (B2).
  B2 (Numeric)  Largest |Delta_j| over the family (b = 1, sqrt3; k = 0, 0.3; 18 directions; 20 openings h) with both outlines
        simple, for each of the 14 vertices (table printed by --table).
  B3 (Exact)  Move a vertex of a TILING so that every tile meeting there follows (the only deformations that keep a tiling).  The
        corner turns of the tiles meeting at a vertex add up to n pi - 2 pi, whatever the position of the vertex, so the sum of the
        Delta over those tiles is 0 and the product of the signs is +1.  For the flat pair at P_8 (two tiles, flat corners) the
        two tiles have Delta_A = -Delta_B, again total 0.  Numeric check: the sum of the exterior angles of the tiles around a
        displaced vertex of a patch is unchanged.
  B4 (Exact)  The two tiles of a pair are congruent copies.  The relative phase of |+-> and |-+> is exp(i (Delta_A - Delta_B) / 2),
        and Delta_A = Delta_B for congruent tiles (the corner turn is rotation invariant), so it is +1 for every deformation of every
        vertex.  A singlet sign -1 does not come from the corner turns of a pair of congruent tiles.
Not tested: the strength of the S-curves per edge; tiles with reflected (mirror) partners, for which B4 does not apply.
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, os.path.join(_HERE, "..", "toymodel_v4")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from spectre_hat_collinearity_flow_model import FLAT_VERTEX, SQRT3  # noqa: E402
from spectre_hat_time_model import is_simple_tile  # noqa: E402
from spectremodel import Tile  # noqa: E402
import t4_geometry as tg  # noqa: E402

TWO_PI = 2.0 * math.pi
N_VERT = 14


def corner_turn(tile, j):
    """wrapped turn at vertex j (the corner that ends edge j - 1)."""
    _, corner = tg.turning(tile)
    return float(corner[(j - 1) % N_VERT])


def delta_at(j, b, k, h, phi_deg):
    """(Delta_j, both outlines simple) for the displacement s h (cos phi, sin phi) of P_j."""
    u = np.array([math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg))])
    turns, simple = [], []
    for s in (1, -1):
        t = Tile(1.0, b, k)
        v = t.vertices[j]
        v.position = v.position + s * h * u
        simple.append(bool(is_simple_tile(t)))
        turns.append(corner_turn(t, j))
    return float(turns[1] - turns[0]), all(simple)


def sup_delta(j, bs=(1.0, SQRT3), ks=(0.0, 0.3), phis=None, hs=None):
    phis = np.arange(0.0, 180.0, 10.0) if phis is None else phis
    hs = np.linspace(0.3, 6.0, 20) if hs is None else hs
    best, arg, n_ok = 0.0, None, 0
    for b in bs:
        for k in ks:
            for phi in phis:
                for h in hs:
                    d, ok = delta_at(j, b, k, h, phi)
                    if ok:
                        n_ok += 1
                        assert abs(d) < TWO_PI, (j, b, k, phi, h, d)
                        if abs(d) > best:
                            best, arg = abs(d), (b, k, float(phi), float(h))
    return best, arg, n_ok


def delta_multi(moves, b=1.0, k=0.3, t=1.0):
    """(sum of Delta over the moved vertices, both outlines simple) for the displacement s t h (cos phi, sin phi) of each moved
    vertex (j, h, phi), s = +-1: the bit of a deformation that moves several vertices of ONE tile."""
    turns = {1: {}, -1: {}}
    simple = []
    for s in (1, -1):
        tile = Tile(1.0, b, k)
        for j, h, phi in moves:
            u = np.array([math.cos(math.radians(phi)), math.sin(math.radians(phi))])
            tile.vertices[j].position = tile.vertices[j].position + s * t * h * u
        simple.append(bool(is_simple_tile(tile)))
        for j, _, _ in moves:
            turns[s][j] = corner_turn(tile, j)
    return float(sum(turns[-1][j] - turns[1][j] for j, _, _ in moves)), all(simple)


# ------------------------------------------------------------------ tests
def test_b111_p8_reproduces_the_v4_result():
    """j = 8 (P_8): chord-normal opening gives Delta = 2 omega = 240 degrees at omega = 120; coarse sup 290..320 degrees."""
    d0, ok0 = delta_at(FLAT_VERTEX, 1.0, 0.5, math.tan(math.radians(60.0)), 0.0)
    assert ok0 and abs(abs(d0) - math.radians(240.0)) < 1e-6, d0
    best, arg, n = sup_delta(FLAT_VERTEX, ks=(0.3,), phis=np.array([-60.0, -45.0, -30.0, 0.0, 30.0, 60.0]),
                             hs=np.linspace(0.3, 6.0, 12))
    assert math.radians(285.0) < best < math.radians(320.0), math.degrees(best)
    return "Delta(chord normal, omega=120) = %.1f; coarse sup %.1f degrees at %s (%d valid)" % (
        math.degrees(abs(d0)), math.degrees(best), arg, n)


def test_b112_no_vertex_reaches_two_pi():
    """every vertex, a coarse grid: |Delta_j| < 2 pi whenever both outlines are simple (and never equals 2 pi - 0.05)."""
    rows = []
    for j in range(N_VERT):
        best, arg, n = sup_delta(j, bs=(1.0,), ks=(0.3,), phis=np.arange(0.0, 180.0, 30.0), hs=np.linspace(0.3, 6.0, 8))
        assert best < TWO_PI - 0.05
        rows.append((j, math.degrees(best), n))
    return "sup |Delta| per vertex (b=1,k=0.3, coarse), degrees: " + " ".join("%d:%.0f" % (j, d) for j, d, _ in rows)


def test_b113_the_bound_two_pi_is_sharp_for_arbitrary_simple_polygons():
    """one vertex moved between two simple outlines: a thin outward spike (turn -> +pi) and the tip of a thin slit cut into a
    square (turn -> -pi).  Delta -> -2 pi as the opening shrinks, and is never equal to it."""
    def turn(P, i):
        a, b, c = P[i - 1], P[i], P[(i + 1) % len(P)]
        u, v = b - a, c - b
        return math.atan2(u[0] * v[1] - u[1] * v[0], u @ v)

    def simple(P):
        n = len(P)

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        for i in range(n):
            for j in range(i + 1, n):
                if j == i + 1 or (i == 0 and j == n - 1):
                    continue
                a, b, c, d = P[i], P[(i + 1) % n], P[j], P[(j + 1) % n]
                if cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0:
                    return False
        return True

    def outline(eps, v):
        return np.array([[-5, -5], [5, -5], [5, 5], [-5, 5], [-5, eps], v, [-5, -eps]], dtype=float)
    gaps = []
    for eps in (0.2, 0.02, 0.002):
        slit = outline(eps, [3.0, 0.0])
        spike = outline(eps, [-10.0, 0.0])
        assert simple(slit) and simple(spike)
        d = turn(slit, 5) - turn(spike, 5)
        assert abs(d) < TWO_PI
        gaps.append(TWO_PI - abs(d))
    assert gaps[0] > gaps[1] > gaps[2] > 0 and gaps[2] < 0.05
    return "2 pi - |Delta| = %s as the opening shrinks (0.2, 0.02, 0.002): sup 2 pi, never attained" % ", ".join("%.4f" % g for g in gaps)


def test_b114_coherent_vertex_move_keeps_total_turn():
    """B3: the corner turns of the tiles meeting at a vertex of a patch sum to n pi - 2 pi (interior angles sum to 2 pi), checked
    on a patch of exact tiles by displacing the vertex coherently and recomputing the angles in floating point."""
    sys.path.insert(0, os.path.join(_HERE, "..", "toymodel_v10"))
    import t10_cavity as tc
    tiles, cent, vmap = tc.patch(4)
    c = tc.CENTRE
    checked = 0
    for v in tiles[c].verts[:14]:
        inc = vmap[v]
        if len(inc) < 3:
            continue
        for sgn_h in ((0.0, 0.0), (0.2, -0.1), (-0.3, 0.25)):
            tot = 0.0
            for i, jj in inc:
                P = [np.array(tc.xyf(w)) for w in tiles[i].verts]
                P[jj] = P[jj] + np.array(sgn_h)
                a, b, cc = P[jj - 1], P[jj], P[(jj + 1) % 14]
                u, w = b - a, cc - b
                tot += math.atan2(u[0] * w[1] - u[1] * w[0], u @ w)
            ref = len(inc) * math.pi - 2 * math.pi
            # orientation of the stored vertex order may be clockwise: compare up to sign
            assert abs(abs(tot) - abs(ref)) < 1e-9 or abs(tot + ref) < 1e-9, (v, sgn_h, tot, ref)
            checked += 1
    assert checked >= 6
    return "%d (vertex, displacement) cases: turns around the vertex sum to (n-2) pi, so sum of Delta = 0" % checked


def test_b115_congruent_pair_has_relative_sign_plus_one():
    """B4: rotating the displaced outlines (the two tiles of a pair are rotated copies) leaves every corner turn unchanged, so
    Delta_A = Delta_B and the relative phase exp(i (Delta_A - Delta_B) / 2) of |+-> and |-+> is 1; checked for several vertices."""
    class _Pts:
        def __init__(self, S):
            self.S = S

        def sample_edges(self, n):
            return self.S
    worst, cases, skipped = 0.0, 0, 0
    for j in (0, 3, 5, 8, 11):
        for (h, phi) in ((0.5, 0.0), (1.0, 0.0), (1.0, 40.0), (1.5, -30.0), (2.0, 90.0)):
            u = np.array([math.cos(math.radians(phi)), math.sin(math.radians(phi))])
            samples, simple = {}, []
            for s in (1, -1):
                t = Tile(1.0, 1.0, 0.3)
                t.vertices[j].position = t.vertices[j].position + s * h * u
                simple.append(bool(is_simple_tile(t)))
                samples[s] = np.asarray(t.sample_edges(40))
            if not all(simple):          # the sign is defined for simple outlines only
                skipped += 1
                continue
            for rot in (60.0, 150.0, 210.0):
                th = math.radians(rot)
                Rm = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
                turns = {}
                for s in (1, -1):
                    for tag, S in (("A", samples[s]), ("B", samples[s] @ Rm.T)):
                        _, c = tg.turning(_Pts(S), n=40)
                        turns[(tag, s)] = c[(j - 1) % N_VERT]
                dA = turns[("A", -1)] - turns[("A", 1)]
                dB = turns[("B", -1)] - turns[("B", 1)]
                worst = max(worst, abs(dA - dB))
                cases += 1
    assert worst < 1e-9, worst
    assert cases >= 12, cases
    return "%d (vertex, displacement, rotation) cases with simple outlines (%d displacements skipped as not simple): Delta_A - Delta_B = 0 to %.1e" % (cases, skipped, worst)


def test_b116_two_moved_vertices_of_one_tile_can_reach_minus_one():
    """B5 (Numeric): moving P_0 and P_4 of one isolated tile (parameters from the single-vertex scans) makes the summed Delta pass
    360 degrees with both outlines simple; bisection finds the opening at which it is 2 pi exactly.  This is a deformation of one
    tile, not of a tiling: by B3 the neighbours that share these vertices compensate."""
    mv = [(0, 3.557142857142857, 150.0), (4, 1.1142857142857143, 90.0)]
    ts = np.linspace(0.2, 0.6, 41)
    vals = [delta_multi(mv, t=t) for t in ts]
    i = next(i for i in range(len(ts)) if vals[i][1] and vals[i][0] >= TWO_PI)
    assert i > 0 and vals[i - 1][1] and vals[i - 1][0] < TWO_PI
    lo, hi = ts[i - 1], ts[i]
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        v, ok = delta_multi(mv, t=mid)
        assert ok
        lo, hi = (mid, hi) if v < TWO_PI else (lo, mid)
    v, ok = delta_multi(mv, t=0.5 * (lo + hi))
    assert ok and abs(v - TWO_PI) < 1e-6, v
    return "sum Delta = %.6f degrees at opening factor t = %.4f, both outlines simple" % (math.degrees(v), 0.5 * (lo + hi))


def run_tests(verbose=True):
    import time
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_b11")]
    failed = 0
    for n, f in tests:
        t = time.time()
        try:
            r = f()
            if verbose:
                print("ok   %s  %s  (%.1fs)" % (n, r, time.time() - t), flush=True)
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e), flush=True)
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def table():
    print("vertex  sup|Delta| (deg)  at (b, k, phi, h)   n_valid")
    for j in range(N_VERT):
        best, arg, n = sup_delta(j)
        print("%5d  %8.1f   %s   %d" % (j, math.degrees(best), arg, n), flush=True)


if __name__ == "__main__":
    if "--table" in sys.argv:
        table()
    else:
        sys.exit(1 if run_tests() else 0)
