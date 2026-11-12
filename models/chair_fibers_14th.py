"""
chair_fibers_14th.py - what the bit of the 14th vertex does to the fibres of chair_fibers.py.

The 14th vertex P_8 of the Spectre carries the bit s = +-1 of the flow (research/toymodel_v4): the reflex value s = -1
moves P_8 by -h (1, 0), h = a tan(omega/2), and the two outlines differ in area by exactly a * 2h.  Here the sheet hangs
over Chair44 as in chair_fibers.py, and we ask what the *chair* sees of the bit: the fibres of the three bases at P_8
(the vertex and its two handles) are recomputed for s = +1 and s = -1 along the flow tau in [0, 1]
(b = a (1 + (sqrt3 - 1) tau), k = k0 (1 - tau), omega = omega0 (1 - tau)), the sheet keeps the placement it has at the Spectre.

Quantities (all exact for the fibre model, no new fitting)
  gap(tau)     = sum over the three bases at P_8 of min_tip E(base, tip) at s = -1, minus the same at s = +1.
                 Assignment free: every base takes its cheapest tip, tips may repeat.
  ripple(tau)  = number of the 42 bases whose tip changes in the one-to-one minimal-energy pairing when the bit flips.
  best tips    = the cheapest tip of the vertex P_8 for the two values (its sign +j / -j is the bump or dent it reads).

Results (each is a test below)
  E8.1  Both quantities vanish at the Hat (tau = 1): there the two values of the bit are the same outline, so the chair
        cannot tell them apart.  Exact (the gap is a sum of identical energies).
  E8.2  The gap is *not* an arrow: its sign at fixed tau depends on the placement of the sheet (angle, scale, height); on a
        grid of placements both signs occur at the Spectre end.  Negative result: the chair does not choose a value of the bit
        without a rule for the placement.
  E8.3  The one-to-one pairing is not stable under the bit: at some tau flipping the bit changes the partner tips of several
        bases, not only P_8, so the bit is not local for the fibres.
  E8.4  Chair44 has bump (+j) and dent (-j) tips of every depth j = 1..12, 8 of each: the same counting as the bump-and-dent
        pairs of the Spectre patch (Result flow-patch), but a bump and the dent of the same depth are at distances from 0.177 to
        1.061 and never at a distance fixed by the tile.  A Hypothesis only: the dictionary bit <-> sign of the tip is by rule.

    python3 models/chair_fibers_14th.py --tests-only
    python3 models/chair_fibers_14th.py --save figures/chair_fibers_14th.png
    python3 models/chair_fibers_14th.py --ensemble figures/chair_fibers_ensemble.png --ensemble3d figures/chair_fibers_ensemble_3d.png
"""
import argparse
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (HERE, ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

import chair_fibers as cf  # noqa: E402
from spectremodel import Tile  # noqa: E402

SQRT3 = math.sqrt(3.0)
FLAT = 8  # the 14th vertex


def params(tau, omega0=120.0, k0=0.5):
    """(b, k, h) at flow time tau for a = 1."""
    return 1.0 + (SQRT3 - 1.0) * tau, k0 * (1.0 - tau), math.tan(math.radians(omega0 * (1.0 - tau)) / 2.0)


def bit_tile(tau, s, omega0=120.0, k0=0.5):
    """The tile at flow time tau with the bit s = +1 (convex) or -1 (reflex)."""
    b, k, h = params(tau, omega0, k0)
    t = Tile(1.0, b, k)
    if s < 0:
        v = t.vertices[FLAT]
        v.position = v.position + np.array([-h, 0.0])
    return t


def default_placement(**kw):
    return cf.Placement.for_tile(Tile(1.0, 1.0, 0.5), **kw)


def _p8_rows(tile):
    return [FLAT] + [len(tile.vertices) + i for i, hd in enumerate(tile.handles) if hd.vertex.index == FLAT]


def cheapest(tile, tips, place):
    """For each base of P_8 the (cheapest tip, energy)."""
    X, U = cf.base_arrays(tile, place)
    E, _, _ = cf.fibre_costs(X, U, tips)
    rows = _p8_rows(tile)
    return [(int(E[r].argmin()), float(E[r].min())) for r in rows]


def gap(tau, tips, place, **kw):
    lo = sum(e for _, e in cheapest(bit_tile(tau, -1, **kw), tips, place))
    hi = sum(e for _, e in cheapest(bit_tile(tau, +1, **kw), tips, place))
    return lo - hi


def pairing(tau, s, tips, place, **kw):
    t = bit_tile(tau, s, **kw)
    cf.assign_fibers(t, tips, place)
    return [b.data["fibers"][0] for b in cf.bases(t)]


def ripple(tau, tips, place, **kw):
    a, b = pairing(tau, +1, tips, place, **kw), pairing(tau, -1, tips, place, **kw)
    return sum(x != y for x, y in zip(a, b))


def placement_grid(angles, scales, z=3.2):
    return [[cf.Placement.for_tile(Tile(1.0, 1.0, 0.5), angle=an, scale=sc, z=z) for sc in scales] for an in angles]


# --------------------------------------------------------------------------
# The ensemble: 3 full tiles + 3 vertex tiles + 4 groups of the 14th vertex = 192 points
# --------------------------------------------------------------------------
N_FULL, N_VERT, N_GROUP = 3, 3, 4
FULL_POINTS, VERT_POINTS, GROUP_POINTS = 42, 14, 6        # 14 vertices + 28 handles, the 14 vertices, the six around P_8
SIDES = ((0, +1), (1, +1), (2, +1), (0, -1), (1, -1), (2, -1))   # the six sides of a cube (axis, sign): +x +y +z -x -y -z
SIDE_NAMES = ("+x", "+y", "+z", "-x", "-y", "-z")
TRIPLE = (6, 7, 8)   # three mutually edge-adjacent tiles of the 9-tile patch, containing the Gamma pair (7, 8)
# order of the six points along the boundary of the tile (the 14th vertex P_8 sits between v8.in and v8.out)
BOUNDARY_ORDER = ("v7", "v7.out", "v8.in", "v8.out", "v9.in", "v9")


def counting_solutions(total=192):
    """All (x, y, z) >= 0 with 42 x + 14 y + 6 z = total (x full tiles, y vertex tiles, z groups of the 14th vertex)."""
    return [(x, y, z) for x in range(total // 42 + 1) for y in range(total // 14 + 1) for z in range(total // 6 + 1)
            if 42 * x + 14 * y + 6 * z == total]


def group_points(tile):
    """The six points of a tile that the bit of P_8 touches: the vertices P_7 and P_9 and the four handles of the
    edges 7 and 8 (the out handle of P_7, the in and out handles of P_8, the in handle of P_9)."""
    v, hd = tile.vertices, {(h.vertex.index, h.kind): h for h in tile.handles}
    return {"v7": v[7], "v7.out": hd[(7, "out")], "v8.in": hd[(8, "in")], "v8.out": hd[(8, "out")],
            "v9.in": hd[(9, "in")], "v9": v[9]}


def antipodes(tile):
    """Pairs of the six points that are exchanged by the point reflection through P_8 (valid for the convex value of the
    bit: the S-shaped edges 7 and 8 are translates of each other and each is point symmetric)."""
    gp = group_points(tile)
    o = {k: np.asarray(v.position, dtype=float) - tile.vertices[FLAT].position for k, v in gp.items()}
    out = {}
    for k in o:
        mates = [j for j in o if j != k and np.linalg.norm(o[j] + o[k]) < 1e-6]
        out[k] = mates[0] if len(mates) == 1 else None
    return out


def canonical_group_roles(tile):
    """The six points in the order of SIDES (+x, +y, +z, -x, -y, -z).  Rule: the three axes are the three antipodal pairs
    about P_8 (derived: ``antipodes``); z is the pair of the two handles of P_8 itself, x the pair of the vertices P_7,
    P_9, y the remaining pair (the handles of the neighbours); on each axis the plus end is the point that comes later
    along the boundary.  The assignment of the pairs to x, y, z is a choice (z: the axis of the bit, after the user's
    suggestion); the pairs and the signs are not."""
    mate = antipodes(tile)
    if any(m is None for m in mate.values()):
        raise ValueError("the six points are not antipodal about P_8")
    axis_of = {}
    for pair, ax in ((("v8.in", "v8.out"), 2), (("v7", "v9"), 0)):
        axis_of[pair[0]] = axis_of[pair[1]] = ax
    for k in mate:
        axis_of.setdefault(k, 1)
    plus = {}
    for k, m in mate.items():
        plus[k] = BOUNDARY_ORDER.index(k) > BOUNDARY_ORDER.index(m)
    roles = [None] * 6
    for k in mate:
        roles[SIDES.index((axis_of[k], +1 if plus[k] else -1))] = k
    return roles


def _tile_frame():
    return Tile(1.0, 1.0, 0.5)


GROUP_ROLE = tuple(canonical_group_roles(_tile_frame()))
GROUP_RADIUS = 0.8
_PATCH_CACHE = {}


def triple_placements(b=1.0):
    """3 x 3 placements of the three tiles of TRIPLE in the 9-tile patch of tile (1, b), and the patch."""
    from spectre_patch import SpectrePatch

    if b not in _PATCH_CACHE:
        _PATCH_CACHE[b] = SpectrePatch(levels=1, a=1.0, b=b)
    p = _PATCH_CACHE[b]
    return [p.placements[i] for i in TRIPLE], p


def _apply2(m, pts):
    pts = np.asarray(pts, dtype=float)
    return pts @ m[:2, :2].T + m[:2, 2]


def _cluster_map(scale, centre_xy, z):
    """Function taking planar points of the patch to 3D: shrink about the centroid of the Spectre triple and put it
    over ``centre_xy`` at height z (the same map for the Spectre and the Hat triple, so they are directly comparable)."""
    pl, _ = triple_placements(1.0)
    verts = np.concatenate([_apply2(m, np.array([v.position for v in _tile_frame().vertices])) for m in pl])
    ref = verts.mean(axis=0)

    def f(q):
        q = np.asarray(q, dtype=float)
        xy = np.asarray(centre_xy) + scale * (q - ref)
        return np.column_stack([xy, np.full(len(xy), z)])

    return f


def full_tiles(scale=0.3, z=3.0, centre_xy=(1.0, 1.0)):
    """The three Spectre tiles (curved, 14 vertices + 28 handles each): lists of (outline 3D, base points (42, 3),
    base unit directions (42, 3)).  They are the placements of TRIPLE in the patch, so they fit edge to edge."""
    f = _cluster_map(scale, centre_xy, z)
    pl, _ = triple_placements(1.0)
    out = []
    for m in pl:
        t = _tile_frame()
        ox, oy = t.outline(60)
        outline = f(_apply2(m, np.column_stack([ox, oy])))
        B = cf.bases(t)
        P = f(_apply2(m, np.array([b.position for b in B])))
        offs = np.array([b.out_handle.offset if isinstance(b, cf.Vertex) else b.offset for b in B], dtype=float)
        D = offs @ m[:2, :2].T
        D = D / np.linalg.norm(D, axis=1)[:, None]
        out.append((outline, P, np.column_stack([D, np.zeros(len(D))])))
    return out


def vertex_tiles(scale=0.3, z=-1.2, centre_xy=(1.0, 1.0)):
    """The three Hat tiles of the same triple (straight edges, 14 vertices): (outline 3D, points (14, 3), directions)."""
    f = _cluster_map(scale, centre_xy, z)
    _, patch = triple_placements(SQRT3)
    out = []
    for i in TRIPLE:
        poly = f(patch.straight_polygon(i))
        D = np.roll(poly, -1, axis=0) - poly
        D = D / np.linalg.norm(D, axis=1)[:, None]
        out.append((np.vstack([poly, poly[:1]]), poly, D))
    return out


def group_tips_panels(tips):
    """Panels of each side, in index order: group g takes the g-th panel of every side."""
    return {sd: sorted(set(tips.panel[(tips.axis == sd[0]) & (tips.sign == sd[1])])) for sd in SIDES}


def panel_centres(tips):
    return {int(p): np.array([float(c) for c in tips.panel_center[int(np.where(tips.panel == p)[0][0])]]) for p in set(tips.panel)}


FACE_SIDES = ((0, +1), (1, +1), (0, -1), (1, -1))   # the four lateral sides, one group each (g0 +x, g1 +y, g2 -x, g3 -y)
LAYOUT = "faces"


def _face_frame(axis, sign):
    """Right-handed in-plane axes (t1, t2) of a side with outward normal e = sign * axis: t1 x t2 = e, so a sheet laid
    on the side is seen unmirrored from outside."""
    e = np.zeros(3); e[axis] = sign
    t1, t2 = np.zeros(3), np.zeros(3)
    t1[(axis + 1) % 3], t2[(axis + 2) % 3] = 1.0, 1.0
    if np.dot(np.cross(t1, t2), e) < 0:
        t1, t2 = t2, t1
    return e, t1, t2


def group_geometry(tips, tau=0.0, bits=(1, 1, 1, 1), scale=0.3, layout=None, radius=0.9):
    """Start points and directions of the 24 group points, and the 3D outline of each group's sheet.

    layout "faces" (default): group g is a whole curved tile standing on the side FACE_SIDES[g] of the chair, turned so
    that its plane is parallel to that side and its front looks at the chair; its six points (P7, P9 and the four
    handles at P8) are where they are on the sheet, and the sheet moves with the bit.
    layout "spread": the six points of group g are put in front of six panels, one per side, by the canonical rule
    GROUP_ROLE (no sheet is drawn)."""
    layout = layout or LAYOUT
    if layout == "spread":
        centres, panels = panel_centres(tips), group_tips_panels(tips)
        ref = bit_tile(tau, +1).vertices[FLAT].position
        X, U, tid, role = [], [], [], []
        for g in range(N_GROUP):
            gp = group_points(bit_tile(tau, bits[g]))
            for k, (axis, sign) in enumerate(SIDES):
                o = np.asarray(gp[GROUP_ROLE[k]].position, dtype=float) - ref
                e = np.zeros(3); e[axis] = sign
                t1, t2 = np.zeros(3), np.zeros(3)
                t1[(axis + 1) % 3], t2[(axis + 2) % 3] = 1.0, 1.0
                X.append(centres[panels[(axis, sign)][g]] + radius * e + scale * (o[0] * t1 + o[1] * t2))
                U.append(-e); tid.append(g); role.append(GROUP_ROLE[k])
        return np.array(X), np.array(U), np.array(tid), role, []
    X, U, tid, role, outlines = [], [], [], [], []
    ref_tile = _tile_frame()
    ref_xy = np.array([v.position for v in ref_tile.vertices]).mean(axis=0)
    for g, (axis, sign) in enumerate(FACE_SIDES):
        e, t1, t2 = _face_frame(axis, sign)
        side_tips = tips.apex[(tips.axis == axis) & (tips.sign == sign)]
        centre = side_tips.mean(axis=0)
        centre[axis] = side_tips[:, axis].max() if sign > 0 else side_tips[:, axis].min()
        centre = centre + radius * e

        def world(xy):
            q = (np.asarray(xy, dtype=float) - ref_xy) * scale
            return centre + np.outer(q[..., 0], t1) + np.outer(q[..., 1], t2)

        tile = bit_tile(tau, bits[g])
        ox, oy = tile.outline(60)
        outlines.append(world(np.column_stack([ox, oy])))
        gp = group_points(tile)
        for k, key in enumerate(GROUP_ROLE):
            b = gp[key]
            off = b.out_handle.offset if isinstance(b, cf.Vertex) else b.offset
            d = np.asarray(off, dtype=float)
            d = d / np.linalg.norm(d)
            X.append(world(np.asarray(b.position)[None])[0])
            U.append(d[0] * t1 + d[1] * t2); tid.append(g); role.append(key)
    return np.array(X), np.array(U), np.array(tid), role, outlines


def group_partners(tips, tau=0.0, bits=(1, 1, 1, 1), scale=0.3, layout=None):
    """The tip and energy of each of the 24 group points (index g * 6 + k).  "spread": the cheapest tip of the point's own
    panel.  "faces": the six points of a sheet take six distinct tips of its side by minimal total energy."""
    from scipy.optimize import linear_sum_assignment

    layout = layout or LAYOUT
    X, U, tid, _, _ = group_geometry(tips, tau, bits, scale, layout)
    E, _, _ = cf.fibre_costs(X, U, tips)
    out = [None] * len(X)
    if layout == "spread":
        panels = group_tips_panels(tips)
        for i in range(len(X)):
            g, k = divmod(i, GROUP_POINTS)
            cand = np.where(tips.panel == panels[SIDES[k]][g])[0]
            t = int(cand[np.argmin(E[i, cand])])
            out[i] = (t, float(E[i, t]))
        return out
    for g, (axis, sign) in enumerate(FACE_SIDES):
        cand = np.where((tips.axis == axis) & (tips.sign == sign))[0]
        rows = np.arange(g * GROUP_POINTS, (g + 1) * GROUP_POINTS)
        r, c = linear_sum_assignment(E[np.ix_(rows, cand)])
        for ri, ci in zip(r, c):
            out[rows[ri]] = (int(cand[ci]), float(E[rows[ri], cand[ci]]))
    return out


def bit_patterns(tips, tau, scale=0.3, layout=None):
    """For every group the signs (+1 bump, -1 dent) of the tips read by its six points, for the bit at +1 and at -1."""
    pat = {}
    for s in (+1, -1):
        part = group_partners(tips, tau, (s,) * N_GROUP, scale, layout)
        pat[s] = ([[int(np.sign(tips.coeff[part[g * 6 + k][0]])) for k in range(6)] for g in range(N_GROUP)],
                  [[part[g * 6 + k][0] for k in range(6)] for g in range(N_GROUP)])
    return pat


def ensemble(tau=0.0, bits=(1, 1, 1, 1), swap=False, full_z=3.0, vert_z=-1.2, scale=0.3):
    """The 192 bases and their start points / directions.  classes: 0 full (Spectre), 1 vertex only (Hat), 2 group."""
    tips_unused = None  # (group geometry needs the panels; supplied in assign_ensemble)
    zf, zv = (vert_z, full_z) if swap else (full_z, vert_z)
    X, U, cls, tid, label = [], [], [], [], []
    for i, (_, P, D) in enumerate(full_tiles(scale, zf)):
        for r in range(len(P)):
            X.append(P[r]); U.append(D[r]); cls.append(0); tid.append(i); label.append("t%d:%s" % (i, cf.base_label(cf.bases(_tile_frame())[r])))
    for i, (_, P, D) in enumerate(vertex_tiles(scale, zv)):
        for r in range(len(P)):
            X.append(P[r]); U.append(D[r]); cls.append(1); tid.append(i); label.append("h%d:v%d" % (i, r))
    return np.array(X), np.array(U), np.array(cls), np.array(tid), label


def assign_ensemble(tips, tau=0.0, bits=(1, 1, 1, 1), swap=False, scale=0.3, layout=None):
    """Group points take the cheapest tip of their own panel (one panel per side and group, 24 in all); the 168 bases of
    the tiles are paired one to one with the 168 other tips by minimal total energy."""
    from scipy.optimize import linear_sum_assignment

    X0, U0, cls0, tid0, label0 = ensemble(tau, bits, swap, scale=scale)
    gX, gU, gtid, grole, outlines = group_geometry(tips, tau, bits, scale, layout)
    X, U = np.vstack([X0, gX]), np.vstack([U0, gU])
    cls = np.concatenate([cls0, np.full(len(gX), 2)])
    tid = np.concatenate([tid0, gtid])
    label = label0 + ["g%d:%s" % (g, r) for g, r in zip(gtid, grole)]
    E, _, _ = cf.fibre_costs(X, U, tips)
    partner = -np.ones(len(X), dtype=int)
    used = np.zeros(len(tips), dtype=bool)
    gp = group_partners(tips, tau, bits, scale, layout)
    for i, (t, _) in enumerate(gp):
        partner[len(X0) + i], used[t] = t, True
    rest, free = np.arange(len(X0)), np.where(~used)[0]
    rows, cols = linear_sum_assignment(E[np.ix_(rest, free)])
    partner[rest[rows]] = free[cols]
    return dict(X=X, U=U, cls=cls, tile=tid, label=label, partner=partner, E=E, sheets=outlines,
                energy=float(E[np.arange(len(X)), partner].sum()))


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e81_both_values_coincide_at_the_hat():
    tips, place = cf.ChairTips(), default_placement()
    assert abs(gap(1.0, tips, place)) < 1e-12
    assert ripple(1.0, tips, place) == 0
    assert abs(gap(0.0, tips, place)) > 1e-3  # and they do not at the Spectre


def test_e82_the_sign_of_the_gap_depends_on_the_placement():
    tips = cf.ChairTips()
    signs = set()
    for row in placement_grid(np.linspace(0, 2 * math.pi, 7)[:-1], (0.3, 0.5, 0.8)):
        for pl in row:
            signs.add(np.sign(gap(0.0, tips, pl)))
    assert signs == {-1.0, 1.0}, signs


def test_e83_the_pairing_is_not_local_to_the_bit():
    tips, place = cf.ChairTips(), default_placement()
    r = [ripple(t, tips, place) for t in np.linspace(0, 0.9, 10)]
    assert max(r) > 1, r  # more than one base changes partner for some tau


def test_e84_chair_counts_match_bump_dent_pairs():
    tips = cf.ChairTips()
    for j in range(1, 13):
        assert (tips.coeff == j).sum() == 8 and (tips.coeff == -j).sum() == 8
    d = []
    for j in range(1, 13):
        a, b = np.where(tips.coeff == j)[0], np.where(tips.coeff == -j)[0]
        d.append(np.linalg.norm(tips.apex[a][:, None] - tips.apex[b][None], axis=2).min(axis=1))
    d = np.concatenate(d)
    assert abs(d.min() - 0.1768) < 1e-3 and abs(d.max() - 1.0607) < 1e-3


def test_gap_matches_make_fibre():
    tips, place = cf.ChairTips(), default_placement()
    t = bit_tile(0.3, -1)
    for base, (tip, e) in zip([cf.bases(t)[i] for i in _p8_rows(t)], cheapest(t, tips, place)):
        assert abs(cf.make_fibre(base, tip, tips, place).energy - e) < 1e-9


def test_e85_the_count_192_is_3_3_4():
    assert FULL_POINTS * N_FULL + VERT_POINTS * N_VERT + GROUP_POINTS * N_GROUP == 192 == len(cf.ChairTips())
    sol = counting_solutions()
    assert len(sol) == 15 and (3, 3, 4) in sol
    # with all three kinds present and as many full as vertex tiles, (3, 3, 4) is the only way
    assert [s for s in sol if s[0] == s[1] >= 1 and s[2] >= 1] == [(3, 3, 4)]
    # 4 groups of 6 points are 24 = the exposed panels, and the other 168 = 24 * 7
    assert N_GROUP * GROUP_POINTS == 24 and 192 - 24 == 168 == 24 * 7
    tips = cf.ChairTips()
    assert len(set(tips.panel)) == 24 and all(((tips.axis == a) & (tips.sign == s)).sum() == 32 for a, s in SIDES)


def test_e86_the_group_is_the_six_points_the_bit_moves():
    t1, t2 = bit_tile(0.3, +1), bit_tile(0.3, -1)
    assert len(group_points(t1)) == 6
    changed = [i for i, (a, b) in enumerate(zip(cf.bases(t1), cf.bases(t2))) if np.linalg.norm(a.position - b.position) > 1e-12]
    names = {cf.base_label(cf.bases(t1)[i]) for i in changed}
    assert names == {"v8", "v8.in", "v8.out", "v7.out", "v9.in"}, names  # P_7 and P_9 themselves stay


def test_e88_the_six_points_are_three_antipodal_pairs_and_the_rule_is_canonical():
    for tau in (0.0, 0.3, 0.7, 0.95):  # at the Hat (k = 0) the handles lie on the chords and the pairing degenerates
        m = antipodes(bit_tile(tau, +1))
        assert {frozenset(kv) for kv in m.items()} == {frozenset(("v7", "v9")), frozenset(("v7.out", "v9.in")), frozenset(("v8.in", "v8.out"))}, m
        assert canonical_group_roles(bit_tile(tau, +1)) == list(GROUP_ROLE)
    assert GROUP_ROLE == ("v9", "v9.in", "v8.out", "v7", "v7.out", "v8.in"), GROUP_ROLE
    # opposite sides carry opposite points: the second half is the point reflection of the first
    assert [BOUNDARY_ORDER.index(a) > BOUNDARY_ORDER.index(b) for a, b in zip(GROUP_ROLE[:3], GROUP_ROLE[3:])] == [True] * 3


def test_e89_the_triple_fits_edge_to_edge():
    import itertools
    pl, patch = triple_placements(1.0)
    pairs = {(int(t), int(u)) for t, _, u, _ in patch.edge_pairs()}
    assert all((min(a, b), max(a, b)) in pairs for a, b in itertools.combinations(TRIPLE, 2)), pairs
    assert all(np.linalg.det(m[:2, :2]) > 0 and abs(np.linalg.det(m[:2, :2]) - 1) < 1e-9 for m in pl)
    angles = np.degrees(np.arctan2([m[1, 0] for m in pl], [m[0, 0] for m in pl]))
    assert len({round(a) % 360 for a in angles}) == 3 and all(abs(a / 30 - round(a / 30)) < 1e-6 for a in angles)
    assert np.abs(np.array([v.position for v in _tile_frame().vertices]) - __import__("spectre_operator").vertices(1, 1)).max() < 1e-12


def test_e87_the_ensemble_pairing_is_a_bijection_with_one_tip_per_panel():
    tips = cf.ChairTips()
    r = assign_ensemble(tips, layout="spread")
    assert sorted(r["partner"]) == list(range(192))
    g = np.where(r["cls"] == 2)[0]
    assert sorted(tips.panel[r["partner"][g]]) == list(range(24))
    assert np.isfinite(r["energy"])


def test_e810_the_four_sheets_face_their_sides():
    tips = cf.ChairTips()
    X, U, tid, role, sheets = group_geometry(tips, layout="faces")
    assert len(sheets) == 4
    for g, (axis, sign) in enumerate(FACE_SIDES):
        e, t1, t2 = _face_frame(axis, sign)
        assert np.allclose(np.cross(t1, t2), e)            # unmirrored when seen from outside
        o = sheets[g]
        assert np.ptp(o @ e) < 1e-9                          # the sheet is flat and parallel to the side
        side = tips.apex[(tips.axis == axis) & (tips.sign == sign)]
        assert (o @ e).mean() > (side @ e).max() - 1e-9   # in front of the chair
    r = assign_ensemble(tips)
    assert sorted(r["partner"]) == list(range(192))
    for g, (axis, sign) in enumerate(FACE_SIDES):
        tp = r["partner"][np.where((r["cls"] == 2) & (r["tile"] == g))[0]]
        assert len(set(tp)) == 6 and all(tips.axis[t] == axis and tips.sign[t] == sign for t in tp)


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
        print("  %-70s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# figure
# --------------------------------------------------------------------------
def figure(tau_scene=0.3):
    import matplotlib.pyplot as plt

    tips, place = cf.ChairTips(), default_placement()
    taus = np.linspace(0.0, 1.0, 41)
    fig = plt.figure(figsize=(14, 9))
    gs = fig.add_gridspec(2, 3)

    # (a) the two values over the chair
    ax = fig.add_subplot(gs[0, 0], projection="3d")
    cf._draw_chair(ax, tips)
    for s, col in ((+1, "tab:blue"), (-1, "crimson")):
        t = bit_tile(tau_scene, s)
        x, y = t.outline(60)
        pts = np.array([place.point(p) for p in zip(x, y)])
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], color=col, lw=1.0)
        for i in _p8_rows(t):
            base = cf.bases(t)[i]
            tip, _ = cheapest(t, tips, place)[_p8_rows(t).index(i)]
            f = cf.make_fibre(base, tip, tips, place)
            ax.plot(f.points[:, 0], f.points[:, 1], f.points[:, 2], color=col, lw=1.4)
    cf._setup3d(ax, 24, -58)
    ax.set_title("(a) tau=%.1f: P8 convex (blue) and reflex (red), cheapest fibres of its three bases" % tau_scene, fontsize=8)

    # (b) gap against tau for several placements
    ax = fig.add_subplot(gs[0, 1])
    for (an, sc, z), col in zip(((0, 0.5, 3.2), (0.5, 0.5, 3.2), (0, 0.8, 3.2), (1.0, 0.3, 4.0), (2.0, 0.5, 3.2), (3.5, 0.5, 3.2)),
                                 ("C0", "C1", "C2", "C3", "C4", "C5")):
        pl = cf.Placement.for_tile(Tile(1.0, 1.0, 0.5), angle=an, scale=sc, z=z)
        ax.plot(taus, [gap(t, tips, pl) for t in taus], color=col, label="angle %.1f, scale %.1f, z %.1f" % (an, sc, z))
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("tau (0 Spectre, 1 Hat)")
    ax.set_ylabel("gap  E(reflex) - E(convex)")
    ax.set_title("(b) the chair's preference for a value of the bit: zero at the Hat, sign set by the placement", fontsize=8)
    ax.legend(fontsize=6)

    # (c) ripple
    ax = fig.add_subplot(gs[0, 2])
    tt = np.linspace(0.0, 1.0, 21)
    ax.step(tt, [ripple(t, tips, place) for t in tt], where="mid", color="k")
    ax.set_xlabel("tau")
    ax.set_ylabel("bases whose tip changes")
    ax.set_title("(c) one-to-one pairing: bases (of 42) that change tip when the bit flips", fontsize=8)

    # (d) sign map at the Spectre end over placements
    ax = fig.add_subplot(gs[1, 0])
    angles, scales = np.linspace(0, 2 * math.pi, 25)[:-1], np.linspace(0.3, 0.9, 13)
    G = np.array([[gap(0.0, tips, pl) for pl in row] for row in placement_grid(angles, scales)])
    im = ax.pcolormesh(scales, np.degrees(angles), np.sign(G) * np.log10(1 + abs(G)), cmap="coolwarm", shading="nearest")
    fig.colorbar(im, ax=ax, label="sign(gap) log10(1+|gap|)")
    ax.set_xlabel("scale of the sheet")
    ax.set_ylabel("angle of the sheet (deg)")
    ax.set_title("(d) gap at the Spectre end: both signs occur", fontsize=8)

    # (e) bump/dent depth pairs of the chair
    ax = fig.add_subplot(gs[1, 1])
    for j in range(1, 13):
        a, b = np.where(tips.coeff == j)[0], np.where(tips.coeff == -j)[0]
        d = np.linalg.norm(tips.apex[a][:, None] - tips.apex[b][None], axis=2).min(axis=1)
        ax.scatter([j] * len(d), d, c="0.2", s=14)
    ax.set_xlabel("depth j")
    ax.set_ylabel("distance of a bump to the nearest dent of the same depth")
    ax.set_title("(e) Chair44: 8 bumps and 8 dents of each depth, at no fixed distance", fontsize=8)

    # (f) which tip (bump/dent) the vertex reads
    ax = fig.add_subplot(gs[1, 2])
    for s, col in ((+1, "tab:blue"), (-1, "crimson")):
        c = [tips.coeff[cheapest(bit_tile(t, s), tips, place)[0][0]] for t in taus]
        ax.step(taus, c, where="mid", color=col, label="s=%+d" % s)
    ax.axhline(0, color="k", lw=0.6)
    ax.set_xlabel("tau")
    ax.set_ylabel("signed depth of the cheapest tip of P8")
    ax.set_title("(f) the tip P8 reads: sign and depth of the cheapest fibre", fontsize=8)
    ax.legend(fontsize=7)
    fig.tight_layout()
    return fig


def _chair(ax, tips, alpha, tip_alpha=1.0):
    """Chair44: exposed faces with the given opacity and the 192 tips (bump orange, dent purple)."""
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    import chair44_core

    quads = []
    for axis, sign, cell in chair44_core.exposed_faces(chair44_core.CHAIR_CELLS):
        t1, t2 = [k for k in range(3) if k != axis]
        quad = []
        for i, j in ((0, 0), (1, 0), (1, 1), (0, 1)):
            v = [0, 0, 0]
            v[axis], v[t1], v[t2] = cell[axis] + (1 if sign > 0 else 0), cell[t1] + i, cell[t2] + j
            quad.append(v)
        quads.append(quad)
    ax.add_collection3d(Poly3DCollection(quads, facecolor="#c9d6ea", edgecolor="0.45", linewidths=0.5, alpha=alpha))
    if tip_alpha > 0:
        bump = tips.coeff > 0
        for mask, col in ((bump, cf._BUMP), (~bump, cf._DENT)):
            p, sz = tips.apex[mask], 6 + 4 * np.abs(tips.coeff[mask])
            ax.scatter(p[:, 0], p[:, 1], p[:, 2], c=col, s=sz, alpha=tip_alpha, depthshade=False, linewidths=0)


def _fibre_points(X, u, tip, tips, n=40):
    D = tips.apex[tip] - X
    nrm = tips.normal[tip]
    s0, s1, _ = cf.fibre_optimum(D @ D, u @ D, nrm @ D, u @ nrm)
    return cf._bernstein(np.linspace(0, 1, n)) @ np.array([X, X + s0 * u, tips.apex[tip] + s1 * nrm, tips.apex[tip]])


_CLS_COLOUR = {0: "0.2", 1: "tab:green", 2: "tab:orange"}
_CLS_NAME = {0: "full tile (42)", 1: "Hat vertices (14)", 2: "14th group (6)"}


def _scene3d(ax, tips, res, classes=(0, 1, 2), chair_alpha=0.15, tip_alpha=1.0, fibre_alpha=0.6, elev=18, azim=-60,
             tiles=True, limits=((-1.8, 3.8), (-1.8, 3.8), (-1.8, 3.6))):
    _chair(ax, tips, chair_alpha, tip_alpha)
    for i in range(len(res["X"])):
        c = res["cls"][i]
        if c in classes:
            B = _fibre_points(res["X"][i], res["U"][i], res["partner"][i], tips)
            ax.plot(B[:, 0], B[:, 1], B[:, 2], color=_CLS_COLOUR[c], lw=1.2 if c == 2 else 0.4, alpha=fibre_alpha)
    for c in classes:
        m = res["cls"] == c
        ax.scatter(*res["X"][m].T, c=_CLS_COLOUR[c], s=22 if c == 2 else 6, depthshade=False)
    if tiles and 2 in classes:
        for o in res.get("sheets", []):
            ax.plot(*o.T, color="tab:orange", lw=1.8)
    if tiles:
        for (o, _, _) in full_tiles():
            ax.plot(*o.T, color="black", lw=1.8) if 0 in classes else None
        for (o, _, _) in vertex_tiles():
            ax.plot(*o.T, color="tab:green", lw=1.8) if 1 in classes else None
    ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1]); ax.set_zlim(*limits[2])
    ax.set_box_aspect([hi - lo for lo, hi in limits])
    ax.view_init(elev=elev, azim=azim)
    ax.tick_params(labelsize=6)


def figure_ensemble_3d():
    """Six views of the 192 fibres: opaque and transparent chair, from the side, above and below, one class at a time."""
    import matplotlib.pyplot as plt

    tips = cf.ChairTips()
    res = assign_ensemble(tips)
    fig = plt.figure(figsize=(16, 10.5))
    views = [
        ("(a) everything, chair opaque", dict(chair_alpha=0.55, fibre_alpha=0.5, elev=20, azim=-60)),
        ("(b) everything, chair transparent, tips shown", dict(chair_alpha=0.06, fibre_alpha=0.6, elev=20, azim=-60)),
        ("(c) side view (looking along y), chair transparent", dict(chair_alpha=0.08, fibre_alpha=0.5, elev=0, azim=-90)),
        ("(d) from above: the three smooth Spectre tiles (a metatile triple) and their fibres", dict(classes=(0,), chair_alpha=0.05, tip_alpha=0.0, fibre_alpha=0.35, elev=90, azim=-90)),
        ("(e) from below: the three Hat vertex sets and their fibres", dict(classes=(1,), chair_alpha=0.05, tip_alpha=0.0, fibre_alpha=0.5, elev=-90, azim=-90)),
        ("(f) the four 14th-vertex sheets only: each faces one lateral side and takes 6 of its 32 tips", dict(classes=(2,), chair_alpha=0.25, tip_alpha=0.35, fibre_alpha=0.9, elev=22, azim=-35)),
    ]
    for i, (title, kw) in enumerate(views):
        ax = fig.add_subplot(2, 3, i + 1, projection="3d")
        _scene3d(ax, tips, res, **kw)
        ax.set_title(title, fontsize=9)
        if i == 0:
            from matplotlib.lines import Line2D
            ax.legend(handles=[Line2D([], [], color=_CLS_COLOUR[c], marker="o", ls="-", ms=4, label=_CLS_NAME[c]) for c in (0, 1, 2)],
                      fontsize=7, loc="upper left")
    fig.suptitle("Chair44 with three Spectre tiles above (curved), three Hat tiles below (vertices only) and four curved 14th-vertex sheets standing on the four lateral sides: 192 fibres", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def figure_ensemble():
    """Counting, layout energy, the canonical group rule and the bump/dent patterns of the groups as the bit flips."""
    import matplotlib.pyplot as plt
    from collections import Counter

    tips = cf.ChairTips()
    r, r2 = assign_ensemble(tips), assign_ensemble(tips, swap=True)
    fig = plt.figure(figsize=(15, 9.5))
    gs = fig.add_gridspec(2, 3)

    ax = fig.add_subplot(gs[0, 0])
    sol = counting_solutions()
    ax.scatter([x for x, y, z in sol], [y for x, y, z in sol], c="0.6", s=60)
    for x, y, z in sol:
        ax.annotate(str(z), (x, y), textcoords="offset points", xytext=(6, 4), fontsize=8)
    ax.scatter([3], [3], c="crimson", s=140, zorder=3)
    ax.set_xlabel("x full tiles (42)"); ax.set_ylabel("y vertex tiles (14)")
    ax.set_title("(a) the 15 ways 42x + 14y + 6z = 192 (label z); red: 3, 3, 4", fontsize=9)

    ax = fig.add_subplot(gs[0, 1])
    w = 0.4
    for off, (res, alpha) in zip((-w / 2, w / 2), ((r, 1.0), (r2, 0.4))):
        bottom = np.zeros(6)
        for c in (0, 1, 2):
            tp = res["partner"][res["cls"] == c]
            d = Counter((int(tips.axis[t]), int(tips.sign[t])) for t in tp)
            h = np.array([d.get(sd, 0) for sd in SIDES])
            ax.bar(np.arange(6) + off, h, w, bottom=bottom, color=_CLS_COLOUR[c], alpha=alpha)
            bottom += h
    ax.set_xticks(range(6)); ax.set_xticklabels(SIDE_NAMES)
    ax.set_ylabel("tips taken")
    ax.set_title("(b) sides reached: left full above (dark), right swapped (pale); total energy %.0f vs %.0f" % (r["energy"], r2["energy"]), fontsize=8)

    ax = fig.add_subplot(gs[0, 2])
    t1, t2 = bit_tile(0.0, +1), bit_tile(0.0, -1)
    x, y = t1.outline(80); ax.plot(x, y, color="0.3", lw=1)
    x2, y2 = t2.outline(80); ax.plot(x2, y2, color="crimson", lw=0.8)
    gp = group_points(t1)
    p8 = np.asarray(t1.vertices[FLAT].position)
    for k, side in zip(GROUP_ROLE, SIDE_NAMES):
        pos = np.asarray(gp[k].position)
        ax.scatter(*pos, c="tab:orange", s=40, zorder=3)
        ax.plot([p8[0], pos[0]], [p8[1], pos[1]], color="tab:orange", lw=0.5)
        ax.annotate("%s: %s" % (side, k), pos, textcoords="offset points", xytext=(4, 4), fontsize=7)
    ax.scatter(*p8, c="tab:blue", s=50, zorder=3); ax.scatter(*t2.vertices[FLAT].position, c="crimson", s=50, zorder=3)
    ax.set_aspect("equal")
    ax.set_title("(c) rule of the alternative layout 'spread' (6 points to 6 sides): antipodal pairs about P8 = axes", fontsize=8)

    # bump/dent patterns of the four groups
    ax = fig.add_subplot(gs[1, 0:2])
    taus = np.linspace(0.0, 1.0, 21)
    rows = []
    for tau in taus:
        pt = bit_patterns(tips, tau)
        rows.append((sum(pt[+1][1][g][k] != pt[-1][1][g][k] for g in range(4) for k in range(6)),
                     sum(pt[+1][0][g][k] != pt[-1][0][g][k] for g in range(4) for k in range(6)),
                     len({tuple(p) for p in pt[+1][0]}), len({tuple(p) for p in pt[-1][0]})))
    rows = np.array(rows)
    ax.step(taus, rows[:, 0], where="mid", color="tab:blue", label="group points whose tip changes (of 24)")
    ax.step(taus, rows[:, 1], where="mid", color="crimson", label="of which bump <-> dent")
    ax.step(taus, rows[:, 2], where="mid", color="0.4", ls="--", label="distinct patterns of the 4 groups (bit +1)")
    ax.step(taus, rows[:, 3], where="mid", color="0.7", ls=":", label="distinct patterns of the 4 groups (bit -1)")
    ax.set_xlabel("tau (0 Spectre, 1 Hat)"); ax.set_ylabel("count")
    ax.set_title("(d) all four sheets flip their bit together: how the tips read by the 24 group points react", fontsize=9)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[1, 2])
    tau = 0.2
    pt = bit_patterns(tips, tau)
    img = np.array([pt[s][0][g] for s in (+1, -1) for g in range(4)] , dtype=float)
    ax.imshow(img, cmap="PuOr_r", vmin=-1, vmax=1, aspect="auto")
    for i in range(8):
        for k in range(6):
            g, s = i % 4, (+1, -1)[i // 4]
            if pt[+1][1][g][k] != pt[-1][1][g][k]:
                ax.text(k, i, "*", ha="center", va="center", fontsize=12)
    ax.axhline(3.5, color="k", lw=1)
    ax.set_xticks(range(6)); ax.set_xticklabels(GROUP_ROLE, fontsize=7)
    ax.set_yticks(range(8)); ax.set_yticklabels(["g%d s=+1" % g for g in range(4)] + ["g%d s=-1" % g for g in range(4)], fontsize=7)
    ax.set_title("(e) tau=%.1f: tip read by each group point, orange bump, purple dent, * tip changed" % tau, fontsize=8)
    fig.tight_layout()
    return fig



def main(argv=None):
    ap = argparse.ArgumentParser(description="The bit of the 14th vertex seen through the fibres of Chair44.")
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--save", metavar="PNG")
    ap.add_argument("--ensemble", metavar="PNG", help="save the analysis figure of the 3 + 3 + 4 ensemble")
    ap.add_argument("--ensemble3d", metavar="PNG", help="save the six-view 3D figure of the ensemble")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args(argv)
    print("chair_fibers_14th self tests:")
    if not run_tests():
        return 1
    if args.tests_only:
        return 0
    import matplotlib

    if args.no_show or args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if args.ensemble3d:
        figure_ensemble_3d().savefig(args.ensemble3d, dpi=130)
        print("saved", args.ensemble3d)
        if not (args.ensemble or args.save):
            return 0
    if args.ensemble:
        figure_ensemble().savefig(args.ensemble, dpi=140)
        print("saved", args.ensemble)
        if not args.save:
            return 0
    fig = figure()
    if args.save:
        fig.savefig(args.save, dpi=140)
        print("saved", args.save)
    if not (args.no_show or args.save):
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
