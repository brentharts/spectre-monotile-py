#!/usr/bin/env python3
"""tm2_lattice.py - Stage 0 checks C1 and C2: what is discrete and what is smooth along the Spectre -> Hat flow, and which bits the tiling allows.

Source of the objects: lift4d.py (the Cheritat-Ma lift) and models/spectre_in_e8.py (its embedding in E8).  Every edge of the Spectre tiling is a unit
step of the lift lattice Z^4 = A2 + A2 (red + green).  The family Tile(a, b) is the projection  P_{a,b}(p,q,r,s) = a (p + q w) + b z (r + s w)  of ONE
integer lift: red steps get length a, green steps length b.  So along the flow b = 1 -> sqrt(3):

  * the lift (integer vertices, bond graph, E8 image) does not change at all           -> the discrete structure D is flow invariant;
  * only the relative scale of the red plane and the green plane changes                 -> the smooth parameter is the ratio b/a of two orthogonal sectors.

Definitions
  D1  FLAT: index of the one vertex of the 14-gon whose turn is 0 (the 'straight pass', the vertex called P_8 in the flow model).  It is flat for every (a, b).
  D2  A tile 'opens' at its flat vertex by displacing it by s h along the outward normal of the chord through its two neighbours, s = +1 (bump) or -1 (dent).
  D3  A seam is a pair of meeting edges of the patch (spectre_patch.edge_pairs).  It is *closed* if the two edges still coincide after the displacements.

Exact claims (finite checks on level-2 and level-3 patches; tolerance 1e-9; each is a test, ``python3 tm2_lattice.py --tests-only``)
  E1  Every tile edge is a single unit step of the lift and an E8 root (norm^2 2); 63 tiles have 8 red + 6 green edges and 8 tiles 6 red + 8 green (level 2).
  E2  The projection P_{1,b} of the lifted patch equals SpectrePatch(levels, 1, b) edge by edge, for b between 1 and 2.
  E3  The bond graph (edge_pairs) is identical for all those b; edge lengths on both sides of every seam agree.
  E4  At the Hat (b = sqrt 3) the projected vertices all lie on the hexagonal lattice Z[w] (rank 2), although the lift has rank 4: the map Z^4 -> Z[w]
      has a kernel of rank 2.  Read through that physical lattice the long edges (length sqrt 3) have E8 norm^2 6 and are not roots; read through the lift
      they are roots.  Both readings are exact and differ.  Distinct lifted vertices stay distinct (518 of 518 at level 2).
  E5  The flat vertex of a tile coincides with: the flat vertex of exactly one other tile (a 'flat pair', always two Spectre-shaped tiles), or with non-flat
      vertices of other tiles (types (0,12), (10,10), (4,12), and for every Mystic tile (10,12)), or with nothing (patch boundary).
  E6  Opening every tile alike (any h > 0, either sign, b = 1 or sqrt 3) breaks 88 of the 406 seams of the level-2 patch, all of them at edges touching the flat vertex.
  E7  Opening a flat pair as one bump and one dent (s_A = -s_B in the outward-normal convention) closes every seam and leaves the total area unchanged; as two
      bumps (or two dents) it breaks exactly 2 seams and changes the total area by 2 a h.  Opening a tile that is not in a flat pair breaks at least 2 seams.

Hypothesis (not tested here): that the antiparallel pair rule is the 'compatibility' of the bit (P4 of PLAN.md) that the tiling itself supplies, with the
area ledger Q = sum of signed openings = 0 per pair.  Nothing here says that nature uses it.

    python3 tm2_lattice.py --tests-only
    python3 tm2_lattice.py --story [--levels 3]
    python3 tm2_lattice.py --no-show --save figures/tm2_stage0_flat_pairs.png
"""
import collections
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, os.path.join(_ROOT, "models"), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import lift4d as L  # noqa: E402
import spectre_operator as so  # noqa: E402
import spectre_patch as sp  # noqa: E402
import spectre_in_e8 as E8  # noqa: E402

TOL = 1e-9
SQRT3 = math.sqrt(3.0)
B_GRID = (1.0, 1.2, 1.5, SQRT3, 2.0)
_C = {}


def flat_index(a=1.0, b=1.0):
    """Index of the vertex of Tile(a, b) whose turn is 0 (D1); asserts there is exactly one."""
    v = np.array(so.vertices(a, b))
    n = len(v)
    idx = []
    for j in range(n):
        e1, e2 = v[j] - v[j - 1], v[(j + 1) % n] - v[j]
        if abs(e1[0] * e2[1] - e1[1] * e2[0]) < TOL and float(e1 @ e2) > 0:
            idx.append(j)
    assert len(idx) == 1, "expected one flat vertex, found %r" % (idx,)
    return idx[0]


FLAT = flat_index()


def lifted(levels):
    if ("lift", levels) not in _C:
        _C[("lift", levels)] = L.lift_patch(levels)
    return _C[("lift", levels)]


def patch(levels, b=1.0):
    key = ("patch", levels, round(b, 12))
    if key not in _C:
        _C[key] = sp.SpectrePatch(levels, 1.0, b)
    return _C[key]


# ------------------------------------------------------------------------------------------------ E1, E2, E3

def edge_steps(levels):
    """Per tile: the 14 integer 4D steps of its edges."""
    tiles, _, _ = lifted(levels)
    return [[tuple(np.array(t[(i + 1) % 14]) - np.array(t[i])) for i in range(14)] for t in tiles]


def e8_edge_norms(levels):
    c = collections.Counter()
    for t in lifted(levels)[0]:
        for v in E8.edge_vectors(t):
            c[round(float(v @ v), 6)] += 1
    return c


def colour_counts(levels):
    """Counter of (n_red, n_green) over the tiles."""
    c = collections.Counter()
    for st in edge_steps(levels):
        red = sum(1 for s in st if s[2] == 0 and s[3] == 0)
        c[(red, 14 - red)] += 1
    return c


def projection_error(levels, b):
    """Largest difference between the edge vectors of the projected lift and those of SpectrePatch(levels, 1, b)."""
    tiles = lifted(levels)[0]
    P = patch(levels, b)
    worst = 0.0
    for i, t in enumerate(tiles):
        pr = L.project(np.array(t), 1.0, b)
        ev = np.roll(pr, -1) - pr
        poly = P.straight_polygon(i)
        pc = poly[:, 0] + 1j * poly[:, 1]
        worst = max(worst, float(np.max(np.abs(ev - (np.roll(pc, -1) - pc)))))
    return worst


def bond_graph(levels, b):
    return patch(levels, b).edge_pairs()


def seam_length_mismatch(levels, b):
    """Largest difference in length between the two edges of any seam."""
    P = patch(levels, b)
    polys = [P.straight_polygon(i) for i in range(P.n_tiles)]
    worst = 0.0
    for A, i, B, j in P.edge_pairs():
        la = np.hypot(*(polys[A][(i + 1) % 14] - polys[A][i]))
        lb = np.hypot(*(polys[B][(j + 1) % 14] - polys[B][j]))
        worst = max(worst, float(abs(la - lb)))
    return worst


# ------------------------------------------------------------------------------------------------ E4

def hat_lattice_coords(levels):
    """Integer coordinates (x, y) in the basis (1, w) of every projected vertex at b = sqrt 3, and the number that are off the lattice."""
    tiles, lift, _ = lifted(levels)
    V = np.array(sorted({tuple(v) for v in lift.values()}))
    z = L.project(V, 1.0, SQRT3)
    y = z.imag / (math.sqrt(3) / 2)
    x = z.real - y / 2
    off = int(np.sum((np.abs(x - np.round(x)) > 1e-7) | (np.abs(y - np.round(y)) > 1e-7)))
    return V, np.round(np.c_[x, y]).astype(int), off


def hat_physical_edge_norms(levels):
    """E8 norm^2 of the edges of the projected tiles at b = sqrt 3, read through the physical lattice Z[w] (embed of (x, y, 0, 0))."""
    tiles, _, _ = lifted(levels)
    c = collections.Counter()
    for t in tiles:
        pr = L.project(np.array(t), 1.0, SQRT3)
        for i in range(14):
            d = pr[(i + 1) % 14] - pr[i]
            y = d.imag / (math.sqrt(3) / 2)
            x = d.real - y / 2
            xi, yi = int(round(x)), int(round(y))
            assert abs(x - xi) < 1e-7 and abs(y - yi) < 1e-7
            e = E8.embed((xi, yi, 0, 0))
            c[round(float(e @ e), 6)] += 1
    return c


def kernel_rank_at(b2):
    """Rank over Q of the kernel of Z^4 -> Z[w] at b = sqrt 3 (b2 = 3) via the integer matrix of (p,q,r,s) -> (p + r - s, q + r + 2 s)."""
    if b2 != 3:
        raise ValueError("only b^2 = 3 is a map into Z[w]")
    M = np.array([[1, 0, 1, -1], [0, 1, 1, 2]])
    return 4 - int(np.linalg.matrix_rank(M))


def distinct_projected_vertices(levels, b):
    V = np.array(sorted({tuple(v) for v in lifted(levels)[1].values()}))
    z = L.project(V, 1.0, b)
    return len({(round(p.real, 7), round(p.imag, 7)) for p in z}), len(V)


# ------------------------------------------------------------------------------------------------ E5 to E7

def _vertex_index(P):
    polys = [P.straight_polygon(i) for i in range(P.n_tiles)]
    idx = collections.defaultdict(list)
    for t, p in enumerate(polys):
        for k, q in enumerate(p):
            idx[(round(q[0], 6), round(q[1], 6))].append((t, k))
    return polys, idx


def flat_vertex_types(levels, b=1.0):
    """Counter of (shape, sorted vertex indices of the OTHER tiles at the flat vertex) and the list of flat pairs (A, B), A < B."""
    P = patch(levels, b)
    polys, idx = _vertex_index(P)
    kinds, pairs = collections.Counter(), []
    for t, p in enumerate(polys):
        q = p[FLAT]
        oth = [(tt, k) for tt, k in idx[(round(q[0], 6), round(q[1], 6))] if tt != t]
        kinds[(int(P.shape[t]), tuple(sorted(k for _, k in oth)))] += 1
        if len(oth) == 1 and oth[0][1] == FLAT and t < oth[0][0]:
            pairs.append((t, oth[0][0]))
    return kinds, pairs


def _outward_normal(poly, j):
    n = len(poly)
    c = poly[(j + 1) % n] - poly[j - 1]
    nrm = np.array([c[1], -c[0]]) / np.hypot(*c)                 # outward for a counter-clockwise polygon
    area2 = np.dot(poly[:, 0], np.roll(poly[:, 1], -1)) - np.dot(poly[:, 1], np.roll(poly[:, 0], -1))
    return nrm if area2 > 0 else -nrm


def polygon_area(poly):
    return 0.5 * abs(np.dot(poly[:, 0], np.roll(poly[:, 1], -1)) - np.dot(poly[:, 1], np.roll(poly[:, 0], -1)))


def opened_polygons(levels, b, openings):
    """Polygons of the patch with the flat vertex of tile t displaced by openings[t] (signed height) along its outward normal (D2)."""
    P = patch(levels, b)
    polys = [P.straight_polygon(i).copy() for i in range(P.n_tiles)]
    for t, hh in openings.items():
        polys[t][FLAT] = polys[t][FLAT] + hh * _outward_normal(polys[t], FLAT)
    return P, polys


def broken_seams(levels, b, openings):
    """(number of seams, list of broken seams (A, i, B, j)) after the displacements (D3)."""
    P, polys = opened_polygons(levels, b, openings)
    bad = []
    pairs = P.edge_pairs()
    for A, i, B, j in pairs:
        a0, a1 = polys[A][i], polys[A][(i + 1) % 14]
        b0, b1 = polys[B][j], polys[B][(j + 1) % 14]
        if not (np.hypot(*(a0 - b1)) < TOL and np.hypot(*(a1 - b0)) < TOL):
            bad.append((int(A), int(i), int(B), int(j)))
    return len(pairs), bad


def total_area(levels, b, openings):
    P, polys = opened_polygons(levels, b, openings)
    return float(sum(polygon_area(p) for p in polys))


# ------------------------------------------------------------------------------------------------ tests

def test_e1_lifted_edges_are_unit_steps_and_e8_roots():
    steps = edge_steps(2)
    assert len(steps) == 71
    for st in steps:
        for s in st:
            assert sorted(abs(c) for c in s)[:2] == [0, 0] and sum(abs(c) for c in s) in (1, 2)
            assert (s[2] == 0 and s[3] == 0) or (s[0] == 0 and s[1] == 0)
    assert dict(e8_edge_norms(2)) == {2.0: 71 * 14}
    assert dict(colour_counts(2)) == {(8, 6): 63, (6, 8): 8}


def test_e2_projection_of_one_lift_is_the_whole_family():
    for b in B_GRID:
        assert projection_error(2, b) < TOL, b
    # mutation: the projection with one b against the patch built with another must disagree
    tiles = lifted(2)[0]
    pr = L.project(np.array(tiles[3]), 1.0, 1.2)
    poly = patch(2, 1.5).straight_polygon(3)
    pc = poly[:, 0] + 1j * poly[:, 1]
    assert np.max(np.abs((np.roll(pr, -1) - pr) - (np.roll(pc, -1) - pc))) > 1e-2


def test_e3_bond_graph_is_the_same_along_the_family():
    ref = bond_graph(2, 1.0)
    assert ref.shape == (406, 4)
    for b in B_GRID:
        assert np.array_equal(bond_graph(2, b), ref), b
        assert patch(2, b).n_tiles == 71 and len(patch(2, b).boundary_edges()) == len(patch(2, 1.0).boundary_edges())
        assert seam_length_mismatch(2, b) < TOL, b
    assert int(patch(2, SQRT3).shape.sum()) == 8                    # 8 Mystic tiles (shape 1) carry the swapped lengths (b, a)


def test_e4_the_hat_projection_collapses_rank_four_to_rank_two():
    V, xy, off = hat_lattice_coords(2)
    assert len(V) == 518 and off == 0
    assert kernel_rank_at(3) == 2
    assert distinct_projected_vertices(2, 1.0) == (518, 518)
    assert distinct_projected_vertices(2, SQRT3) == (518, 518)
    norms = hat_physical_edge_norms(2)
    assert set(norms) == {2.0, 6.0}
    n_mys = int(patch(2, SQRT3).shape.sum())
    n_spec = 71 - n_mys
    assert norms[2.0] == 8 * n_spec + 6 * n_mys and norms[6.0] == 6 * n_spec + 8 * n_mys
    # at b = 1 the physical edges are the roots themselves (norm^2 2), by E1
    assert dict(e8_edge_norms(2)) == {2.0: 994}
    # a mutation: at b = 1.5 the vertices are NOT on Z[w]
    tiles, lift, _ = lifted(2)
    V1 = np.array(sorted({tuple(v) for v in lift.values()}))
    z = L.project(V1, 1.0, 1.5)
    y = z.imag / (math.sqrt(3) / 2)
    x = z.real - y / 2
    assert np.sum((np.abs(x - np.round(x)) > 1e-7) | (np.abs(y - np.round(y)) > 1e-7)) > 0


def test_e5_flat_vertex_types():
    for levels in (2, 3):
        for b in (1.0, SQRT3):
            kinds, pairs = flat_vertex_types(levels, b)
            allowed = {(), (8,), (0, 12), (10, 10), (10, 12), (4, 12)}
            assert {k[1] for k in kinds} <= allowed, kinds
            assert {k[1] for k in kinds if k[0] == 1} <= {(10, 12), ()}       # every Mystic tile: (10, 12)
            P = patch(levels, b)
            assert all(P.shape[a] == 0 and P.shape[c] == 0 for a, c in pairs)
    kinds, pairs = flat_vertex_types(3)
    assert len(pairs) == 82 and kinds[(0, (8,))] == 164
    assert kinds[(0, ())] + kinds[(1, ())] == 89                                # boundary tiles at level 3


def test_e6_uniform_opening_breaks_seams_at_the_flat_vertex():
    for b in (1.0, SQRT3):
        assert broken_seams(2, b, {})[1] == []
        for hh in (0.1, 0.268, -0.1):
            n, bad = broken_seams(2, b, {t: hh for t in range(71)})
            assert n == 406 and len(bad) == 88, (b, hh, len(bad))
            assert all(i in (FLAT - 1, FLAT) or j in (FLAT - 1, FLAT) for _, i, _, j in bad)


def test_e7_antiparallel_pairs_close_every_seam_and_conserve_area():
    levels, hh = 3, 0.2
    for b in (1.0, SQRT3):
        _, pairs = flat_vertex_types(levels, b)
        assert len(pairs) == 82
        base = total_area(levels, b, {})
        for (A, B) in pairs:
            n, bad = broken_seams(levels, b, {A: hh, B: -hh})
            assert bad == [], (b, A, B)
            assert abs(total_area(levels, b, {A: hh, B: -hh}) - base) < 1e-9
            n, bad = broken_seams(levels, b, {A: hh, B: hh})
            assert len(bad) == 2, (b, A, B, len(bad))
            assert abs(total_area(levels, b, {A: hh, B: hh}) - (base + 2 * hh)) < 1e-9     # each bump adds a h with a = 1
            assert abs(total_area(levels, b, {A: -hh, B: -hh}) - (base - 2 * hh)) < 1e-9


def test_e7b_a_tile_outside_a_flat_pair_cannot_open_alone():
    levels = 3
    kinds, pairs = flat_vertex_types(levels)
    P = patch(levels, 1.0)
    polys, idx = _vertex_index(P)
    in_pair = {t for pr in pairs for t in pr}
    tested = 0
    for t in range(P.n_tiles):
        q = polys[t][FLAT]
        oth = [x for x in idx[(round(q[0], 6), round(q[1], 6))] if x[0] != t]
        if t in in_pair or not oth:
            continue
        n, bad = broken_seams(levels, 1.0, {t: 0.2})
        assert len(bad) >= 2, t
        tested += 1
        if tested >= 60:
            break
    assert tested >= 60


def test_validation():
    for bad in (-1, 0, float("nan")):
        try:
            sp.SpectrePatch(2, 1.0, bad)
        except ValueError:
            continue
        raise AssertionError("SpectrePatch accepted b = %r" % (bad,))
    try:
        kernel_rank_at(2)
    except ValueError:
        pass
    else:
        raise AssertionError("kernel_rank_at accepted b^2 = 2")


def run_tests(verbose=True):
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            if verbose:
                print("  %-75s PASS" % name)
        except AssertionError as e:
            failed += 1
            print("  %-75s FAIL %s" % (name, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def story(levels=3):
    print("flat vertex index %d; levels %d" % (FLAT, levels))
    print("lifted edges, E8 norm^2:", dict(e8_edge_norms(2)), " colours (red, green):", dict(colour_counts(2)))
    for b in B_GRID:
        print("  b = %.4f  projection error %.1e   bond graph equal to b = 1: %s" % (b, projection_error(2, b), np.array_equal(bond_graph(2, b), bond_graph(2, 1.0))))
    print("Hat view: edge E8 norm^2 through the physical lattice Z[w]:", dict(hat_physical_edge_norms(2)), " kernel rank", kernel_rank_at(3))
    for lv in range(2, levels + 1):
        kinds, pairs = flat_vertex_types(lv)
        n = patch(lv).n_tiles
        inter = n - sum(c for k, c in kinds.items() if k[1] == ())
        print("level %d: %d tiles, flat pairs %d (%.1f%% of interior tiles), vertex types %s" % (lv, n, len(pairs), 100.0 * 2 * len(pairs) / inter, dict(kinds)))
    n, bad = broken_seams(2, 1.0, {t: 0.2 for t in range(71)})
    print("uniform opening h = 0.2, level 2: %d of %d seams broken" % (len(bad), n))
    _, pairs = flat_vertex_types(3)
    A, B = pairs[0]
    print("flat pair", (A, B), " bump+dent broken seams:", len(broken_seams(3, 1.0, {A: 0.2, B: -0.2})[1]), " bump+bump:", len(broken_seams(3, 1.0, {A: 0.2, B: 0.2})[1]))


def figure(save=None, show=True, levels=3):
    """Left: the level-`levels` patch coloured by what meets each tile's flat vertex.  Right: one flat pair opened as bump + dent (seams closed) and as bump + bump (two seams open)."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    P = patch(levels)
    kinds, pairs = flat_vertex_types(levels)
    polys, idx = _vertex_index(P)
    in_pair = {t for pr in pairs for t in pr}
    colour = {}
    for t, p in enumerate(polys):
        q = p[FLAT]
        oth = [x for x in idx[(round(q[0], 6), round(q[1], 6))] if x[0] != t]
        colour[t] = "#2a9d8f" if t in in_pair else ("#e9c46a" if P.shape[t] == 1 else ("#cccccc" if not oth else "#e76f51"))
    fig, ax = plt.subplots(1, 3, figsize=(15, 5.2))
    for t, p in enumerate(polys):
        ax[0].add_patch(Polygon(p, closed=True, fc=colour[t], ec="k", lw=0.3))
    ax[0].autoscale_view()
    ax[0].set_aspect("equal")
    ax[0].set_title("level %d: flat vertex meets a flat vertex (teal, %d tiles)\nnon-flat vertices (orange), Mystic (yellow), nothing (grey)" % (levels, 2 * len(pairs)), fontsize=9)
    A, B = pairs[len(pairs) // 2]
    near = {A, B}
    qc = polys[A][FLAT]
    for pa in range(P.n_tiles):
        if np.min(np.hypot(*(polys[pa] - qc).T)) < 2.6:
            near.add(pa)
    for k, (sa, sb, title) in enumerate(((1, -1, "bump + dent: every seam closes, area unchanged"), (1, 1, "bump + bump: 2 seams open, area + 2 a h"))):
        _, op = opened_polygons(levels, 1.0, {A: 0.35 * sa, B: 0.35 * sb})
        for t in near:
            ax[k + 1].add_patch(Polygon(op[t], closed=True, fc="#2a9d8f" if t in (A, B) else "#dddddd", ec="k", lw=0.6, alpha=0.85))
        pts = np.vstack([op[t] for t in near])
        ax[k + 1].set_xlim(qc[0] - 3.2, qc[0] + 3.2)
        ax[k + 1].set_ylim(qc[1] - 3.2, qc[1] + 3.2)
        ax[k + 1].plot(*qc, "r.", ms=6)
        ax[k + 1].set_aspect("equal")
        ax[k + 1].set_title(title, fontsize=9)
    fig.suptitle("Stage 0: the flat vertex can open only in antiparallel pairs (tm2_lattice.py)", fontsize=11)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
        print("wrote", save)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    lv = 3
    if "--levels" in argv:
        lv = int(argv[argv.index("--levels") + 1])
    if "--story" in argv:
        story(lv)
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv, levels=lv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
