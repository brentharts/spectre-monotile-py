#!/usr/bin/env python3
"""spectre_cavity_sat.py - Dual-Engine Exact Z[sqrt(3)] Solver for Spectre Cavity Completion,
Multi-Tile Erasure Certification (upgrading quantgrav.pdf Result 8), and
Closed Flip-Cluster / Pachner-Move Discovery (Open Problem #14).

Engine A (Unrestricted Combinatorial Exact MRV Search):
  No model-set hull assumption and no index-36 coset assumption.  Searches the
  COMPLETE space of all edge-to-edge chiral Spectre tilings of a k-tile cavity C
  using exact (1/2) Z[sqrt(3)] polygon geometry, exact area conservation (exactly
  k tiles), and Minimum-Remaining-Values (MRV) boundary-edge unit propagation.
  Avoids irrational Z[zeta_12] BFS micro-shift blowup because every tile placed
  is flush-anchored to an active boundary edge of the remaining unfilled cavity.

Engine B (Result-66 Index-36 Model-Set Lattice SAT + DRAT Certification):
  Restricts candidate placements to the exact index-36 Z^4 model-set sublattice
  (quantgrav.pdf Result 66) with bounded internal coordinate (uniformly discrete!).
  Encodes exact directed-edge Stokes flux conservation + pairwise interior-overlap
  exclusion + blocking clause into DIMACS CNF, solves with CaDiCaL 2.1.3, and
  formally certifies every UNSAT verdict with drat-trim.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
SPECTRE_BC = os.path.join(UPGRADE_DIR, "repos", "spectre", "boundary_certificate")
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")

sys.path.insert(0, SPECTRE_BC)
import spectre_exact as se  # noqa: E402

SQRT3 = math.sqrt(3.0)
Z3Num = Tuple[int, int]
Z3Pt = Tuple[Z3Num, Z3Num]
Z4Vec = Tuple[int, int, int, int]


# ============================================================================
# Exact arithmetic in (1/2) Z[sqrt(3)]
# ============================================================================
def z3_sign(val: Z3Num) -> int:
    a, b = val
    if a == 0 and b == 0:
        return 0
    if a >= 0 and b >= 0:
        return 1
    if a <= 0 and b <= 0:
        return -1
    aa = a * a
    bb3 = 3 * b * b
    if a > 0:
        return 1 if aa > bb3 else (-1 if aa < bb3 else 0)
    return 1 if bb3 > aa else (-1 if bb3 < aa else 0)


def z3_sub(u: Z3Num, v: Z3Num) -> Z3Num:
    return (u[0] - v[0], u[1] - v[1])


def z3_add(u: Z3Num, v: Z3Num) -> Z3Num:
    return (u[0] + v[0], u[1] + v[1])


def z3_mul_raw(u: Z3Num, v: Z3Num) -> Z3Num:
    return (u[0] * v[0] + 3 * u[1] * v[1], u[0] * v[1] + u[1] * v[0])


def z4_to_z3pt(v: Z4Vec) -> Z3Pt:
    p, q, r, s = v
    return ((2 * p + q, r), (r + 2 * s, q))


def z4_to_f2(v: Z4Vec) -> Tuple[float, float]:
    p, q, r, s = v
    return (p + 0.5 * q + 0.5 * SQRT3 * r, 0.5 * SQRT3 * q + 0.5 * r + s)


def z3_cross(o: Z3Pt, a: Z3Pt, b: Z3Pt) -> Z3Num:
    ax, ay = z3_sub(a[0], o[0]), z3_sub(a[1], o[1])
    bx, by = z3_sub(b[0], o[0]), z3_sub(b[1], o[1])
    return z3_sub(z3_mul_raw(ax, by), z3_mul_raw(ay, bx))


def z3_orient(o: Z3Pt, a: Z3Pt, b: Z3Pt) -> int:
    return z3_sign(z3_cross(o, a, b))


def z3_cmp(u: Z3Num, v: Z3Num) -> int:
    return z3_sign(z3_sub(u, v))


def z3_on_segment_strict(p: Z3Pt, a: Z3Pt, b: Z3Pt) -> bool:
    if p == a or p == b:
        return False
    if z3_orient(a, b, p) != 0:
        return False
    return (z3_cmp(p[0], a[0]) * z3_cmp(p[0], b[0]) <= 0) and (z3_cmp(p[1], a[1]) * z3_cmp(p[1], b[1]) <= 0)


def z3_segments_properly_cross(a: Z3Pt, b: Z3Pt, c: Z3Pt, d: Z3Pt) -> bool:
    o1 = z3_orient(a, b, c)
    o2 = z3_orient(a, b, d)
    if o1 == 0 or o2 == 0 or o1 == o2:
        return False
    o3 = z3_orient(c, d, a)
    o4 = z3_orient(c, d, b)
    return o3 != 0 and o4 != 0 and o3 != o4


def z3_point_in_poly_status(p: Z3Pt, poly: Sequence[Z3Pt]) -> int:
    n = len(poly)
    inside = False
    px, py = p
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        if p == a or z3_on_segment_strict(p, a, b):
            return 0
        ca = z3_cmp(a[1], py)
        cb = z3_cmp(b[1], py)
        if (ca > 0) != (cb > 0):
            if cb > 0:
                if z3_orient(a, b, p) > 0:
                    inside = not inside
            else:
                if z3_orient(b, a, p) > 0:
                    inside = not inside
    return 1 if inside else -1


def z3_scale2_poly(poly: Sequence[Z3Pt]) -> List[Z3Pt]:
    return [((2 * x[0], 2 * x[1]), (2 * y[0], 2 * y[1])) for x, y in poly]


def z3_polys_overlap_interior(polyA: Sequence[Z3Pt], polyB: Sequence[Z3Pt],
                              setA: Set[Z3Pt], setB: Set[Z3Pt]) -> bool:
    if setA == setB:
        return True
    nA, nB = len(polyA), len(polyB)
    for i in range(nA):
        a1, a2 = polyA[i], polyA[(i + 1) % nA]
        for j in range(nB):
            b1, b2 = polyB[j], polyB[(j + 1) % nB]
            if z3_segments_properly_cross(a1, a2, b1, b2):
                return True
    for p in polyA:
        if p not in setB and z3_point_in_poly_status(p, polyB) == 1:
            return True
    for p in polyB:
        if p not in setA and z3_point_in_poly_status(p, polyA) == 1:
            return True
    polyB2 = z3_scale2_poly(polyB)
    for i in range(nA):
        a1, a2 = polyA[i], polyA[(i + 1) % nA]
        splits = [a1, a2]
        for q in polyB:
            if z3_on_segment_strict(q, a1, a2):
                splits.append(q)
        if len(splits) > 2:
            dx = z3_cmp(a2[0], a1[0])
            dy = z3_cmp(a2[1], a1[1])
            splits.sort(key=lambda pt: (pt[0][0] + pt[0][1] * SQRT3) * dx + (pt[1][0] + pt[1][1] * SQRT3) * dy)
        for s_idx in range(len(splits) - 1):
            mid2 = (z3_add(splits[s_idx][0], splits[s_idx + 1][0]), z3_add(splits[s_idx][1], splits[s_idx + 1][1]))
            if z3_point_in_poly_status(mid2, polyB2) == 1:
                return True
    polyA2 = z3_scale2_poly(polyA)
    for j in range(nB):
        b1, b2 = polyB[j], polyB[(j + 1) % nB]
        splits = [b1, b2]
        for q in polyA:
            if z3_on_segment_strict(q, b1, b2):
                splits.append(q)
        if len(splits) > 2:
            dx = z3_cmp(b2[0], b1[0])
            dy = z3_cmp(b2[1], b1[1])
            splits.sort(key=lambda pt: (pt[0][0] + pt[0][1] * SQRT3) * dx + (pt[1][0] + pt[1][1] * SQRT3) * dy)
        for s_idx in range(len(splits) - 1):
            mid2 = (z3_add(splits[s_idx][0], splits[s_idx + 1][0]), z3_add(splits[s_idx][1], splits[s_idx + 1][1]))
            if z3_point_in_poly_status(mid2, polyA2) == 1:
                return True
    return False


# ============================================================================
# Exact Z^4 Spectre geometry
# ============================================================================
def _to_int_mat(M) -> np.ndarray:
    return np.array([[int(M[i, j]) for j in range(4)] for i in range(4)], dtype=np.int64)


def _to_int_vec(v) -> Z4Vec:
    return tuple(int(x) for x in v)


ZPOW_MATS = [_to_int_mat(se.zpow(k)) for k in range(12)]
REFLECT_MAT = _to_int_mat(se.R[0])
SPECTRE_Z4 = np.array([_to_int_vec(v) for v in se.SPECTRE], dtype=np.int64)

UNIT_DIRS_Z4: Dict[Z4Vec, int] = {}
for _k in range(12):
    UNIT_DIRS_Z4[tuple(int(x) for x in ZPOW_MATS[_k][:, 0])] = _k

ORIENT_VERTS: List[List[Z4Vec]] = []
ORIENT_EDGE_DIRS: List[List[int]] = []
for _ref in (0, 1):
    for _k in range(12):
        M = ZPOW_MATS[_k] if _ref == 0 else (ZPOW_MATS[_k] @ REFLECT_MAT)
        pts = [tuple(int(x) for x in (M @ SPECTRE_Z4[i])) for i in range(14)]
        if _ref == 1:
            pts = list(reversed(pts))
        fpts = [z4_to_f2(v) for v in pts]
        area2 = sum(fpts[i][0] * fpts[(i + 1) % 14][1] - fpts[(i + 1) % 14][0] * fpts[i][1] for i in range(14))
        assert area2 > 1.0
        edirs = [UNIT_DIRS_Z4[tuple(pts[(j + 1) % 14][c] - pts[j][c] for c in range(4))] for j in range(14)]
        ORIENT_VERTS.append(pts)
        ORIENT_EDGE_DIRS.append(edirs)

EDGES_BY_DIR = {
    False: {d: [(o, j) for o in range(12) for j in range(14) if ORIENT_EDGE_DIRS[o][j] == d] for d in range(12)},
    True:  {d: [(o, j) for o in range(24) for j in range(14) if ORIENT_EDGE_DIRS[o][j] == d] for d in range(12)},
}


class TileGeom:
    __slots__ = ("key", "orient", "shift", "verts_z4", "edges_z4", "edge_set", "poly_z3", "set_z3", "bbox", "centroid")

    def __init__(self, orient: int, shift: Z4Vec):
        self.key = (orient, shift)
        self.orient = orient
        self.shift = shift
        base = ORIENT_VERTS[orient]
        s0, s1, s2, s3 = shift
        verts = [(v[0] + s0, v[1] + s1, v[2] + s2, v[3] + s3) for v in base]
        self.verts_z4 = verts
        self.edges_z4 = [(verts[j], verts[(j + 1) % 14]) for j in range(14)]
        self.edge_set = frozenset(self.edges_z4)
        self.poly_z3 = [z4_to_z3pt(v) for v in verts]
        self.set_z3 = set(self.poly_z3)
        fpts = [z4_to_f2(v) for v in verts]
        xs = [p[0] for p in fpts]
        ys = [p[1] for p in fpts]
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        self.centroid = (sum(xs) / 14.0, sum(ys) / 14.0)


def bbox_may_overlap(b1, b2, pad=1e-6) -> bool:
    return not (b1[2] < b2[0] - pad or b2[2] < b1[0] - pad or b1[3] < b2[1] - pad or b2[3] < b1[1] - pad)


def tiles_overlap(t1: TileGeom, t2: TileGeom) -> bool:
    if not bbox_may_overlap(t1.bbox, t2.bbox):
        return False
    return z3_polys_overlap_interior(t1.poly_z3, t2.poly_z3, t1.set_z3, t2.set_z3)


def build_chiral_patch(level: int, species: str = "Delta") -> Tuple[List[TileGeom], List[str]]:
    """Build exact chiral level-L Spectre patch (det_sign = +1 for all tiles) + kinds ('p', 'g1', 'g2')."""
    out = se.build(level)
    raw = out[level][0][species]
    need_mirror = (se.det_sign(raw[0][0]) < 0)
    mat_to_orient = {tuple(ZPOW_MATS[k].flatten()): k for k in range(12)}
    tiles: List[TileGeom] = []
    kinds: List[str] = []
    for T, kind in raw:
        if need_mirror:
            T = se.mul(se.R, T)
        assert se.det_sign(T) == 1
        A = _to_int_mat(T[0])
        t = _to_int_vec(T[1])
        tiles.append(TileGeom(mat_to_orient[tuple(A.flatten())], t))
        kinds.append(kind)
    return tiles, kinds


def patch_adjacency(tiles: Sequence[TileGeom]) -> Dict[int, Set[int]]:
    edge_owner = {}
    for idx, tg in enumerate(tiles):
        for e in tg.edges_z4:
            edge_owner[e] = idx
    adj = {i: set() for i in range(len(tiles))}
    for (u, v), i in edge_owner.items():
        j = edge_owner.get((v, u))
        if j is not None:
            adj[i].add(j)
    return adj


def extract_cavity_boundary(erased_tiles: Sequence[TileGeom]) -> Tuple[Set[Tuple[Z4Vec, Z4Vec]], List[List[Z3Pt]]]:
    counts = collections.Counter()
    for tg in erased_tiles:
        for e in tg.edges_z4:
            counts[e] += 1
    bd_edges = {e for e in counts if (e[1], e[0]) not in counts}
    nxt = collections.defaultdict(list)
    for u, v in bd_edges:
        nxt[u].append(v)
    cycles_z4 = []
    used = set()
    for e in sorted(bd_edges):
        if e in used:
            continue
        if len(nxt[e[0]]) != 1:
            return bd_edges, []
        cyc = [e[0]]
        cur = e
        while cur not in used:
            used.add(cur)
            v = cur[1]
            if len(nxt[v]) != 1:
                return bd_edges, []
            if v == cyc[0]:
                break
            cyc.append(v)
            cur = (v, nxt[v][0])
        cycles_z4.append(cyc)
    return bd_edges, [[z4_to_z3pt(v) for v in cyc] for cyc in cycles_z4]


def poly_inside_simple_cavity(tg: TileGeom, cav_poly_z3: Sequence[Z3Pt], cav_set_z3: Set[Z3Pt],
                              cav_bbox: Tuple[float, float, float, float],
                              cav2_z3: Sequence[Z3Pt], rev_bd: Set[Tuple[Z4Vec, Z4Vec]]) -> bool:
    if (tg.bbox[0] < cav_bbox[0] - 1e-6 or tg.bbox[1] < cav_bbox[1] - 1e-6 or
            tg.bbox[2] > cav_bbox[2] + 1e-6 or tg.bbox[3] > cav_bbox[3] + 1e-6):
        return False
    if any(e in rev_bd for e in tg.edges_z4):
        return False
    for p in tg.poly_z3:
        if p not in cav_set_z3 and z3_point_in_poly_status(p, cav_poly_z3) == -1:
            return False
    nC = len(cav_poly_z3)
    for i in range(14):
        a1, a2 = tg.poly_z3[i], tg.poly_z3[(i + 1) % 14]
        for j in range(nC):
            b1, b2 = cav_poly_z3[j], cav_poly_z3[(j + 1) % nC]
            if z3_segments_properly_cross(a1, a2, b1, b2):
                return False
    for i in range(14):
        a1, a2 = tg.poly_z3[i], tg.poly_z3[(i + 1) % 14]
        splits = [a1, a2]
        for q in cav_poly_z3:
            if z3_on_segment_strict(q, a1, a2):
                splits.append(q)
        if len(splits) > 2:
            dx = z3_cmp(a2[0], a1[0])
            dy = z3_cmp(a2[1], a1[1])
            splits.sort(key=lambda pt: (pt[0][0] + pt[0][1] * SQRT3) * dx + (pt[1][0] + pt[1][1] * SQRT3) * dy)
        for s_idx in range(len(splits) - 1):
            mid2 = (z3_add(splits[s_idx][0], splits[s_idx + 1][0]), z3_add(splits[s_idx][1], splits[s_idx + 1][1]))
            if z3_point_in_poly_status(mid2, cav2_z3) == -1:
                return False
    return True


def verify_exact_partition(chosen: Sequence[TileGeom], bd_edges: Set[Tuple[Z4Vec, Z4Vec]]) -> None:
    counts = collections.Counter()
    for tg in chosen:
        for e in tg.edges_z4:
            counts[e] += 1
            assert counts[e] == 1, f"Duplicate directed edge {e}!"
    unmatched = {e for e in counts if (e[1], e[0]) not in counts}
    assert unmatched == bd_edges, f"Boundary mismatch: {len(unmatched)} vs {len(bd_edges)}"
    for i in range(len(chosen)):
        for j in range(i + 1, len(chosen)):
            assert not tiles_overlap(chosen[i], chosen[j]), "Overlap in decoded solution!"


def cell_complex_counts(tiles: Sequence[TileGeom]) -> Tuple[int, int, int]:
    V = set()
    E = set()
    for tg in tiles:
        for u, v in tg.edges_z4:
            V.add(u)
            E.add((u, v) if u < v else (v, u))
    return len(V), len(E), len(tiles)


def grow_simple_cavity(tiles: Sequence[TileGeom], adj: Dict[int, Set[int]], seed: int, target_size: int) -> List[int]:
    all_edges = collections.Counter()
    for tg in tiles:
        for e in tg.edges_z4:
            all_edges[e] += 1
    interior = {i for i, tg in enumerate(tiles) if all((e[1], e[0]) in all_edges for e in tg.edges_z4)}
    cx, cy = tiles[seed].centroid
    cav = [seed]
    cav_set = {seed}
    while len(cav) < target_size:
        frontier = set()
        for u in cav:
            for nb in adj[u]:
                if nb not in cav_set and nb in interior:
                    frontier.add(nb)
        if not frontier:
            break
        ranked = sorted(frontier, key=lambda i: (tiles[i].centroid[0] - cx) ** 2 + (tiles[i].centroid[1] - cy) ** 2)
        added = False
        for cand in ranked:
            trial = cav + [cand]
            _, cycs = extract_cavity_boundary([tiles[i] for i in trial])
            if len(cycs) == 1:
                cav = trial
                cav_set.add(cand)
                added = True
                break
        if not added:
            break
    return cav


# ============================================================================
# ENGINE A: Exact Unrestricted Combinatorial MRV-Edge Cavity Solver (ZERO hull assumption!)
# ============================================================================
def solve_cavity_unrestricted_mrv(
    erased_tiles: Sequence[TileGeom],
    allow_reflect: bool = False,
    forbid_original: bool = True,
    max_nodes: int = 200000,
) -> dict:
    """Exhaustive exact MRV boundary-edge search over ALL edge-to-edge Spectre fillings of C!"""
    t0 = time.time()
    bd_edges, cycles_z3 = extract_cavity_boundary(erased_tiles)
    if len(cycles_z3) != 1:
        return {"status": "NON_SIMPLE_CAVITY", "n_erased": len(erased_tiles)}
    cav_poly_z3 = cycles_z3[0]
    cav_set_z3 = set(cav_poly_z3)
    cav2_z3 = z3_scale2_poly(cav_poly_z3)
    fpts = [((p[0][0] + p[0][1] * SQRT3) * 0.5, (p[1][0] + p[1][1] * SQRT3) * 0.5) for p in cav_poly_z3]
    cav_bbox = (min(p[0] for p in fpts), min(p[1] for p in fpts), max(p[0] for p in fpts), max(p[1] for p in fpts))
    rev_bd = {(v, u) for (u, v) in bd_edges}
    orig_keys = frozenset(tg.key for tg in erased_tiles)
    target_k = len(erased_tiles)

    # Memoize TileGeom and cavity containment test per (orient, shift)
    inside_cache: Dict[Tuple[int, Z4Vec], Optional[TileGeom]] = {}
    edge_cands_cache: Dict[Tuple[Z4Vec, Z4Vec], List[TileGeom]] = {}

    def cands_for_edge(e: Tuple[Z4Vec, Z4Vec]) -> List[TileGeom]:
        res = edge_cands_cache.get(e)
        if res is not None:
            return res
        u, v = e
        d = UNIT_DIRS_Z4[(v[0] - u[0], v[1] - u[1], v[2] - u[2], v[3] - u[3])]
        out = []
        for o, j in EDGES_BY_DIR[allow_reflect][d]:
            bv = ORIENT_VERTS[o][j]
            shift = (u[0] - bv[0], u[1] - bv[1], u[2] - bv[2], u[3] - bv[3])
            key = (o, shift)
            if key not in inside_cache:
                tg = TileGeom(o, shift)
                inside_cache[key] = tg if poly_inside_simple_cavity(tg, cav_poly_z3, cav_set_z3, cav_bbox, cav2_z3, rev_bd) else None
            tg = inside_cache[key]
            if tg is not None:
                out.append(tg)
        edge_cands_cache[e] = out
        return out

    # Also cache pairwise overlaps between TileGeom keys
    ov_cache: Dict[Tuple[Tuple[int, Z4Vec], Tuple[int, Z4Vec]], bool] = {}

    def overlaps_any(tg: TileGeom, placed: Sequence[TileGeom]) -> bool:
        k1 = tg.key
        for p in placed:
            k2 = p.key
            pk = (k1, k2) if k1 < k2 else (k2, k1)
            v = ov_cache.get(pk)
            if v is None:
                v = tiles_overlap(tg, p)
                ov_cache[pk] = v
            if v:
                return True
        return False

    nodes = 0
    aborted = False
    solution: Optional[List[TileGeom]] = None

    def dfs(active: Set[Tuple[Z4Vec, Z4Vec]], placed: List[TileGeom]) -> bool:
        nonlocal nodes, aborted, solution
        nodes += 1
        if nodes > max_nodes:
            aborted = True
            return False
        if not active:
            if len(placed) != target_k:
                return False
            pk = frozenset(t.key for t in placed)
            if forbid_original and pk == orig_keys:
                return False
            solution = list(placed)
            return True
        if len(placed) >= target_k:
            return False

        # MRV: choose active edge with fewest valid non-overlapping candidates!
        # Valid candidate t for active edge e must NOT have any edge (x,y) whose reverse (y,x) is in active
        # (Wait: if (y,x) is in active, then placing t cancels (y,x)? NO! Active edges have UNFILLED cavity on LEFT.
        #  Tile t has its interior on the LEFT of its directed edges!
        #  So matching edge e=(u,v) in active consumes e (same direction!).
        #  If t has an edge (x,y) whose REVERSE (y,x) is in active, that means the cavity boundary at (y,x) has
        #  FILLED exterior on the right of (y,x), i.e. on the LEFT of (x,y) -> tile t would lie INSIDE the filled exterior!
        #  So NO edge of t may have its reverse in `active`!)
        best_e = None
        best_cands = None
        for e in active:
            valid = []
            for tg in cands_for_edge(e):
                # Quick check: no reversed active edge in tg
                bad = False
                for ex, ey in tg.edges_z4:
                    if (ey, ex) in active:
                        bad = True
                        break
                if bad:
                    continue
                if overlaps_any(tg, placed):
                    continue
                valid.append(tg)
                if best_cands is not None and len(valid) >= len(best_cands):
                    break
            if len(valid) == 0:
                return False  # Dead edge -> instant prune!
            if best_cands is None or len(valid) < len(best_cands):
                best_e = e
                best_cands = valid
                if len(best_cands) == 1:
                    break  # Forced unit propagation!

        for tg in best_cands:
            # Update active boundary: edges of tg already in active are consumed;
            # other edges of tg become new active edges with REVERSED direction (remaining cavity is on their right -> reverse puts cavity on left)!
            new_active = set(active)
            for ex, ey in tg.edges_z4:
                if (ex, ey) in new_active:
                    new_active.remove((ex, ey))
                else:
                    new_active.add((ey, ex))
            placed.append(tg)
            if dfs(new_active, placed):
                return True
            placed.pop()
            if aborted:
                return False
        return False

    found = dfs(set(bd_edges), [])
    dt = time.time() - t0
    if found and solution is not None:
        verify_exact_partition(solution, bd_edges)
        chosen_keys = {t.key for t in solution}
        v0, e0, f0 = cell_complex_counts(erased_tiles)
        v1, e1, f1 = cell_complex_counts(solution)
        return {
            "engine": "A_unrestricted_MRV",
            "status": "SAT",
            "allow_reflect": allow_reflect,
            "n_erased": target_k,
            "n_bd_edges": len(bd_edges),
            "n_removed": len(orig_keys - chosen_keys),
            "n_added": len(chosen_keys - orig_keys),
            "dV": v1 - v0,
            "dE": e1 - e0,
            "dF": f1 - f0,
            "same_as_orig": (chosen_keys == orig_keys),
            "nodes": nodes,
            "n_cached_placements": len(inside_cache),
            "t_s": round(dt, 4),
        }
    return {
        "engine": "A_unrestricted_MRV",
        "status": "ABORTED" if aborted else "UNSAT_EXHAUSTIVE",
        "allow_reflect": allow_reflect,
        "n_erased": target_k,
        "n_bd_edges": len(bd_edges),
        "nodes": nodes,
        "n_cached_placements": len(inside_cache),
        "t_s": round(dt, 4),
    }


# ============================================================================
# ENGINE B: Exact Index-36 Model-Set Lattice SAT + DRAT Certification!
# ============================================================================
def integer_hnf_4d(vectors: Sequence[Z4Vec]) -> Tuple[np.ndarray, int, np.ndarray]:
    """Compute exact 4x4 column Hermite Normal Form of a list of integer 4-vectors (pure Python ints)."""
    # Use Euclidean column operations on a 4 x M integer matrix
    cols = [list(v) for v in vectors if any(x != 0 for x in v)]
    M = len(cols)
    # Build 4 x M matrix A[row][col]
    A = [[cols[c][r] for c in range(M)] for r in range(4)]
    piv_c = 0
    for r in range(4):
        while True:
            nz = [c for c in range(piv_c, M) if A[r][c] != 0]
            if not nz:
                break
            # Pick column with smallest |A[r][c]|
            best = min(nz, key=lambda c: abs(A[r][c]))
            if best != piv_c:
                for rr in range(4):
                    A[rr][piv_c], A[rr][best] = A[rr][best], A[rr][piv_c]
            if A[r][piv_c] < 0:
                for rr in range(4):
                    A[rr][piv_c] = -A[rr][piv_c]
            changed = False
            piv = A[r][piv_c]
            for c in range(piv_c + 1, M):
                if A[r][c] != 0:
                    q = A[r][c] // piv
                    for rr in range(4):
                        A[rr][c] -= q * A[rr][piv_c]
                    if A[r][c] != 0:
                        changed = True
            if not changed:
                # Reduce earlier columns mod piv
                for c in range(piv_c):
                    q = A[r][c] // piv
                    if q != 0:
                        for rr in range(4):
                            A[rr][c] -= q * A[rr][piv_c]
                piv_c += 1
                break
    assert piv_c == 4, f"Rank < 4: {piv_c}"
    H = np.array([[A[r][c] for c in range(4)] for r in range(4)], dtype=np.int64)
    detH = abs(int(round(np.linalg.det(H.astype(float)))))
    # Exact integer adjugate = detH * H^{-1}
    Hi = np.linalg.inv(H.astype(float))
    ADJ = np.rint(Hi * detH).astype(np.int64)
    # Verify exact integer identity: ADJ @ H == detH * I_4
    assert np.array_equal(ADJ @ H, detH * np.eye(4, dtype=np.int64))
    return H, detH, ADJ


def extract_model_set_lattice(tiles: Sequence[TileGeom], kinds: Sequence[str]):
    """Extract the exact index-36 Z^4 sublattice, per-class coset representatives, and
    contracting internal-space projection STAR directly from a real Spectre patch!
    """
    # Group tile shifts by (orient, is_g2) — wait: in chiral single-Spectre representation,
    # each (orient, kind in {'p','g1','g2'}) has a coset! Note 'p' and 'g1' share the O/G base!
    # Let's check exact cosets for each (orient) (or (orient, kind=='g2'))!
    by_class = collections.defaultdict(list)
    for tg, kd in zip(tiles, kinds):
        by_class[(tg.orient, kd == "g2")].append(tg.shift)

    # Check common difference lattice across all classes!
    first_cls = next(iter(by_class))
    u0 = by_class[first_cls][0]
    diffs = [tuple(v[c] - u0[c] for c in range(4)) for v in by_class[first_cls][1:]]
    H, detH, ADJ = integer_hnf_4d(diffs)

    # Verify EVERY class has differences in the exact same index-detH sublattice!
    cosets: Dict[int, Set[Z4Vec]] = collections.defaultdict(set)
    for (o, is_g2), vecs in by_class.items():
        ref = vecs[0]
        arr = np.array([[v[c] - ref[c] for c in range(4)] for v in vecs], dtype=np.int64)
        ok = np.all((arr @ ADJ.T) % detH == 0, axis=1)
        assert ok.all(), f"Class {(o, is_g2)} not in common sublattice!"
        cosets[o].add(ref)

    # Compute contracting internal-space projection STAR (2x4) via SVD of all tile shifts
    all_shifts = np.array([tg.shift for tg in tiles], dtype=float)
    _, sv, vt = np.linalg.svd(all_shifts - all_shifts.mean(0), full_matrices=False)
    STAR = vt[2:]  # (2, 4) orthonormal basis of the bounded internal plane!
    # Measure exact internal-space bounding radius per class
    int_coords = all_shifts @ STAR.T
    int_center = int_coords.mean(axis=0)
    int_rad = float(np.linalg.norm(int_coords - int_center, axis=1).max())
    return {
        "H": H,
        "detH": detH,
        "ADJ": ADJ,
        "cosets": {o: list(refs) for o, refs in cosets.items()},
        "STAR": STAR,
        "int_center": int_center,
        "int_rad": int_rad,
        "sv": sv.tolist(),
    }


def solve_cavity_modelset_sat(
    erased_tiles: Sequence[TileGeom],
    lattice_info: dict,
    internal_margin: float = 1.5,
    forbid_original: bool = True,
    certify_drat: bool = True,
) -> dict:
    """Engine B: Exact SAT + DRAT over ALL Result-66 index-36 model-set placements inside cavity C!"""
    t0 = time.time()
    bd_edges, cycles_z3 = extract_cavity_boundary(erased_tiles)
    if len(cycles_z3) != 1:
        return {"engine": "B_modelset_SAT", "status": "NON_SIMPLE_CAVITY", "n_erased": len(erased_tiles)}
    cav_poly_z3 = cycles_z3[0]
    cav_set_z3 = set(cav_poly_z3)
    cav2_z3 = z3_scale2_poly(cav_poly_z3)
    fpts = [((p[0][0] + p[0][1] * SQRT3) * 0.5, (p[1][0] + p[1][1] * SQRT3) * 0.5) for p in cav_poly_z3]
    cav_bbox = (min(p[0] for p in fpts), min(p[1] for p in fpts), max(p[0] for p in fpts), max(p[1] for p in fpts))
    rev_bd = {(v, u) for (u, v) in bd_edges}

    detH = lattice_info["detH"]
    ADJ = lattice_info["ADJ"]
    STAR = lattice_info["STAR"]
    int_center = lattice_info["int_center"]
    max_int_r = lattice_info["int_rad"] * internal_margin + 1.0
    cosets = lattice_info["cosets"]

    # Enumerate ALL Z^4 vectors whose physical image is near cav_bbox AND internal image is within max_int_r!
    # Map (phys_x, phys_y, int_x, int_y) = M4 @ v -> invert M4 to get exact bounding box in Z^4!
    PH = np.array([
        [1.0, 0.5, 0.5 * SQRT3, 0.0],
        [0.0, 0.5 * SQRT3, 0.5, 1.0],
    ], dtype=float)
    M4 = np.vstack([PH, STAR])
    Mi = np.linalg.inv(M4)

    # Physical center & radius of cavity (+ tile diameter 5.0 margin)
    pcx = 0.5 * (cav_bbox[0] + cav_bbox[2])
    pcy = 0.5 * (cav_bbox[1] + cav_bbox[3])
    prad = 0.5 * math.hypot(cav_bbox[2] - cav_bbox[0], cav_bbox[3] - cav_bbox[1]) + 5.5
    # Center 4D target
    center_4 = np.array([pcx, pcy, int_center[0], int_center[1]])
    v_center = Mi @ center_4
    # Exact box half-widths along each Z^4 coordinate from Cauchy-Schwarz on rows of Mi
    half_w = np.ceil(
        np.linalg.norm(Mi[:, :2], axis=1) * prad + np.linalg.norm(Mi[:, 2:], axis=1) * max_int_r
    ).astype(int) + 1
    lo = np.floor(v_center).astype(int) - half_w
    hi = np.ceil(v_center).astype(int) + half_w

    # Generate all Z^4 vectors in [lo, hi] that satisfy physical+internal bounds AND index-36 coset!
    # Vectorized over (x1, x2, x3) for each x0!
    r1 = np.arange(lo[1], hi[1] + 1, dtype=np.int64)
    r2 = np.arange(lo[2], hi[2] + 1, dtype=np.int64)
    r3 = np.arange(lo[3], hi[3] + 1, dtype=np.int64)
    g1, g2, g3 = np.meshgrid(r1, r2, r3, indexing="ij")
    tail = np.column_stack([g1.ravel(), g2.ravel(), g3.ravel()])  # (M, 3)

    # Precompute allowed coset residues: (v @ ADJ.T) % detH for each orient!
    orient_residues: Dict[int, Set[Tuple[int, int, int, int]]] = {}
    all_residues: Set[Tuple[int, int, int, int]] = set()
    for o, refs in cosets.items():
        rset = set()
        for ref in refs:
            rv = tuple(int(x) for x in ((np.array(ref, dtype=np.int64) @ ADJ.T) % detH))
            rset.add(rv)
            all_residues.add(rv)
        orient_residues[o] = rset

    # Fast residue lookup via packed int key
    def pack_res(arr):
        return ((arr[:, 0] * detH + arr[:, 1]) * detH + arr[:, 2]) * detH + arr[:, 3]

    orient_packed = {
        o: {((r[0] * detH + r[1]) * detH + r[2]) * detH + r[3] for r in rset}
        for o, rset in orient_residues.items()
    }
    all_packed = np.array(sorted({p for s in orient_packed.values() for p in s}), dtype=np.int64)

    cands: List[TileGeom] = []
    seen_keys: Set[Tuple[int, Z4Vec]] = set()
    for x0 in range(int(lo[0]), int(hi[0]) + 1):
        V = np.empty((len(tail), 4), dtype=np.int64)
        V[:, 0] = x0
        V[:, 1:] = tail
        # Filter physical radius
        px = V @ PH[0] - pcx
        py = V @ PH[1] - pcy
        m_ph = (px * px + py * py) <= prad * prad
        if not m_ph.any():
            continue
        V = V[m_ph]
        # Filter internal radius
        ix = V @ STAR[0] - int_center[0]
        iy = V @ STAR[1] - int_center[1]
        m_in = (ix * ix + iy * iy) <= max_int_r * max_int_r
        if not m_in.any():
            continue
        V = V[m_in]
        # Filter index-36 cosets
        res = (V @ ADJ.T) % detH
        pk = pack_res(res)
        m_cos = np.isin(pk, all_packed)
        if not m_cos.any():
            continue
        V = V[m_cos]
        pk = pk[m_cos]
        for v_row, pval in zip(V, pk):
            shift = (int(v_row[0]), int(v_row[1]), int(v_row[2]), int(v_row[3]))
            pv_i = int(pval)
            for o, pset in orient_packed.items():
                if pv_i in pset:
                    key = (o, shift)
                    if key in seen_keys:
                        continue
                    tg = TileGeom(o, shift)
                    if poly_inside_simple_cavity(tg, cav_poly_z3, cav_set_z3, cav_bbox, cav2_z3, rev_bd):
                        seen_keys.add(key)
                        cands.append(tg)

    orig_keys = {tg.key for tg in erased_tiles}
    assert orig_keys.issubset(seen_keys), f"Original tiles missing from Result-66 candidates! Missing {len(orig_keys - seen_keys)}"

    # Build exact Stokes-flux + overlap-exclusion DIMACS CNF
    key_to_var = {tg.key: i + 1 for i, tg in enumerate(cands)}
    edge_vars: Dict[Tuple[Z4Vec, Z4Vec], List[int]] = collections.defaultdict(list)
    for tg in cands:
        vid = key_to_var[tg.key]
        for e in tg.edges_z4:
            edge_vars[e].append(vid)

    clauses: List[List[int]] = []
    for e in bd_edges:
        vs = edge_vars.get(e, [])
        assert len(vs) >= 1
        clauses.append(vs)
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                clauses.append([-vs[i], -vs[j]])

    for e, vs in edge_vars.items():
        if e in bd_edges:
            continue
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                clauses.append([-vs[i], -vs[j]])
        opp = edge_vars.get((e[1], e[0]), [])
        for v in vs:
            clauses.append([-v] + opp)

    # Spatial bucketed exact overlap exclusion
    buckets = collections.defaultdict(list)
    for i, tg in enumerate(cands):
        x0, y0, x1, y1 = tg.bbox
        for bx in range(int(math.floor(x0 / 4.0)), int(math.floor(x1 / 4.0)) + 1):
            for by in range(int(math.floor(y0 / 4.0)), int(math.floor(y1 / 4.0)) + 1):
                buckets[(bx, by)].append(i)
    checked = set()
    n_ov = 0
    for ids in buckets.values():
        m = len(ids)
        for a in range(m):
            ia = ids[a]
            for b in range(a + 1, m):
                ib = ids[b]
                pair = (ia, ib) if ia < ib else (ib, ia)
                if pair in checked:
                    continue
                checked.add(pair)
                if tiles_overlap(cands[ia], cands[ib]):
                    clauses.append([-(ia + 1), -(ib + 1)])
                    n_ov += 1

    if forbid_original:
        clauses.append([-key_to_var[k] for k in orig_keys])

    t_enc = time.time() - t0
    with tempfile.TemporaryDirectory(prefix="spectre_sat_") as tmp:
        cnf_path = os.path.join(tmp, "cavity.cnf")
        drat_path = os.path.join(tmp, "proof.drat")
        with open(cnf_path, "w") as fh:
            fh.write(f"p cnf {len(cands)} {len(clauses)}\n")
            for cl in clauses:
                fh.write(" ".join(map(str, cl)) + " 0\n")
        cmd = [CADICAL_BIN, "--no-binary", cnf_path]
        if certify_drat:
            cmd.append(drat_path)
        t1 = time.time()
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        t_solve = time.time() - t1

        if proc.returncode == 20:
            drat_ok = None
            t_drat = 0.0
            if certify_drat:
                t2 = time.time()
                chk = subprocess.run([DRAT_TRIM_BIN, cnf_path, drat_path], capture_output=True, text=True, check=False)
                t_drat = time.time() - t2
                drat_ok = ("s VERIFIED" in chk.stdout)
                assert drat_ok, f"drat-trim failed: {chk.stdout}"
            return {
                "engine": "B_modelset_SAT",
                "status": "UNSAT",
                "drat_verified": drat_ok,
                "n_erased": len(erased_tiles),
                "n_bd_edges": len(bd_edges),
                "n_candidates": len(cands),
                "n_clauses": len(clauses),
                "n_overlap_clauses": n_ov,
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 4),
                "t_drat": round(t_drat, 4),
            }
        elif proc.returncode == 10:
            chosen = []
            for line in proc.stdout.splitlines():
                if line.startswith("v "):
                    for tok in line.split()[1:]:
                        v = int(tok)
                        if v > 0:
                            chosen.append(cands[v - 1])
            verify_exact_partition(chosen, bd_edges)
            chosen_keys = {tg.key for tg in chosen}
            v0, e0, f0 = cell_complex_counts(erased_tiles)
            v1, e1, f1 = cell_complex_counts(chosen)
            return {
                "engine": "B_modelset_SAT",
                "status": "SAT",
                "n_erased": len(erased_tiles),
                "n_chosen": len(chosen),
                "n_removed": len(orig_keys - chosen_keys),
                "n_added": len(chosen_keys - orig_keys),
                "dV": v1 - v0,
                "dE": e1 - e0,
                "dF": f1 - f0,
                "same_as_orig": (chosen_keys == orig_keys),
                "n_candidates": len(cands),
                "n_clauses": len(clauses),
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 4),
            }
        else:
            raise RuntimeError(f"CaDiCaL rc={proc.returncode}: {proc.stderr}")


# ============================================================================
# Self-tests & systematic experiments
# ============================================================================
def run_all(level: int = 3, out_json: Optional[str] = None):
    print(f"=== Building Exact Level-{level} Chiral Spectre Patch ===")
    tiles, kinds = build_chiral_patch(level)
    adj = patch_adjacency(tiles)
    lat = extract_model_set_lattice(tiles, kinds)
    print(f"  Tiles: {len(tiles)}, exact index-{lat['detH']} Z^4 sublattice verified across all {len(lat['cosets'])} orientations!")
    print(f"  Internal window radius: {lat['int_rad']:.4f}, singular values: {[round(x, 2) for x in lat['sv']]}")

    all_edges = collections.Counter()
    for tg in tiles:
        for e in tg.edges_z4:
            all_edges[e] += 1
    interior = [i for i, tg in enumerate(tiles) if all((e[1], e[0]) in all_edges for e in tg.edges_z4)]
    cx = sum(t.centroid[0] for t in tiles) / len(tiles)
    cy = sum(t.centroid[1] for t in tiles) / len(tiles)
    interior.sort(key=lambda i: (tiles[i].centroid[0] - cx) ** 2 + (tiles[i].centroid[1] - cy) ** 2)
    center = interior[0]

    # Self-test 1: Unblocked recovery on BOTH Engine A and Engine B (k=6 cavity)
    cav6 = grow_simple_cavity(tiles, adj, center, 6)
    ra_rec = solve_cavity_unrestricted_mrv([tiles[i] for i in cav6], allow_reflect=False, forbid_original=False)
    assert ra_rec["status"] == "SAT" and ra_rec["same_as_orig"], ra_rec
    rb_rec = solve_cavity_modelset_sat([tiles[i] for i in cav6], lat, forbid_original=False, certify_drat=False)
    assert rb_rec["status"] == "SAT" and rb_rec["same_as_orig"], rb_rec
    print(f"  [PASS] Self-test unblocked 6-tile recovery: Engine A ({ra_rec['nodes']} nodes, {ra_rec['t_s']}s) & Engine B ({rb_rec['n_candidates']} cands, {rb_rec['t_solve']}s)")

    results = {
        "level": level,
        "n_tiles": len(tiles),
        "n_interior": len(interior),
        "sublattice_index": lat["detH"],
        "exp1a_single_tile_all_orients": [],
        "exp1b_unrestricted_chiral": [],
        "exp1c_unrestricted_reflect_control": [],
        "exp1d_modelset_large_cavities": [],
    }

    # Exp 1A: ALL 12 orientations — 1-tile erasure certified by BOTH Engine A (unrestricted) AND Engine B (DRAT)!
    print("\n=== [Exp 1A] Exact 1-Tile Erasure Across All Interior Orientations (Engine A Unrestricted + Engine B DRAT) ===")
    by_orient = collections.defaultdict(list)
    for idx in interior:
        by_orient[tiles[idx].orient].append(idx)
    for o in sorted(by_orient.keys()):
        idx = by_orient[o][0]
        ra = solve_cavity_unrestricted_mrv([tiles[idx]], allow_reflect=False, forbid_original=True)
        rb = solve_cavity_modelset_sat([tiles[idx]], lat, forbid_original=True, certify_drat=True)
        assert ra["status"] == "UNSAT_EXHAUSTIVE", ra
        assert rb["status"] == "UNSAT" and rb["drat_verified"], rb
        results["exp1a_single_tile_all_orients"].append({"orient": o, "tile_idx": idx, "A": ra, "B": rb})
    print(f"  [PROVED] {len(by_orient)}/{len(by_orient)} orientations: 100% UNSAT_EXHAUSTIVE (Engine A) AND 100% DRAT-VERIFIED UNSAT (Engine B)!")

    # Exp 1B: Engine A — Unrestricted Combinatorial Chiral Multi-Tile Erasure Horizon (NO hull assumption!)
    print("\n=== [Exp 1B] Engine A: Unrestricted Chiral Spectre Cavity Rigidity (Zero Hull Assumption) ===")
    for sz in (2, 4, 6, 8, 10, 12, 16, 20, 25, 30, 40, 50):
        cav = grow_simple_cavity(tiles, adj, center, sz)
        if len(cav) < sz:
            break
        sx, sy = tiles[center].centroid
        rad = max(math.hypot(tiles[i].centroid[0] - sx, tiles[i].centroid[1] - sy) for i in cav)
        ra = solve_cavity_unrestricted_mrv([tiles[i] for i in cav], allow_reflect=False, forbid_original=True, max_nodes=50000)
        ra["radius"] = round(rad, 3)
        results["exp1b_unrestricted_chiral"].append(ra)
        print(f"  k={len(cav):2d} (r={rad:5.2f}, dC={ra['n_bd_edges']:3d}): {ra['status']:17s} nodes={ra['nodes']:6d} t={ra['t_s']:.3f}s"
              + (f" rem={ra['n_removed']} add={ra['n_added']} (dV,dE,dF)=({ra['dV']},{ra['dE']},{ra['dF']})" if ra["status"] == "SAT" else ""))
        if ra["status"] in ("SAT", "ABORTED"):
            break

    # Exp 1C: Engine A — Reflection-Allowed Control (proving chirality is essential & solver is NOT over-constrained!)
    print("\n=== [Exp 1C] Engine A: Reflection-Allowed Control (Testing Non-Chiral Re-Tiling) ===")
    for sz in (1, 2, 3, 4, 6, 8, 10, 12, 16):
        cav = grow_simple_cavity(tiles, adj, center, sz)
        if len(cav) < sz:
            break
        ra = solve_cavity_unrestricted_mrv([tiles[i] for i in cav], allow_reflect=True, forbid_original=True, max_nodes=50000)
        results["exp1c_unrestricted_reflect_control"].append(ra)
        print(f"  k={len(cav):2d} (reflect=True): {ra['status']:17s} nodes={ra['nodes']:6d} t={ra['t_s']:.3f}s"
              + (f" rem={ra['n_removed']} add={ra['n_added']} (dV,dE,dF)=({ra['dV']},{ra['dE']},{ra['dF']})" if ra["status"] == "SAT" else ""))
        if ra["status"] in ("SAT", "ABORTED"):
            break

    # Exp 1D: Engine B — Result-66 Model-Set Lattice SAT + DRAT up to HUGE cavities (radius 10..26+!)
    print("\n=== [Exp 1D] Engine B: Exact Result-66 Model-Set SAT + DRAT Up To Radius 26+ (Open Problem #14!) ===")
    # Also test around a Mystic pair (g1+g2) seed and a flat-pair seed!
    mystic_seed = next((i for i in interior if kinds[i] == "g1"), interior[1])
    for seed_name, seed_idx, sz_list in (
        ("center", center, [4, 8, 12, 16, 20, 30, 40, 60, 80, 100, 130]),
        ("mystic", mystic_seed, [2, 6, 12, 20, 40, 70]),
    ):
        print(f"  Seed '{seed_name}' (tile #{seed_idx}, orient={tiles[seed_idx].orient}, kind={kinds[seed_idx]}):")
        for sz in sz_list:
            cav = grow_simple_cavity(tiles, adj, seed_idx, sz)
            if len(cav) < sz:
                break
            sx, sy = tiles[seed_idx].centroid
            rad = max(math.hypot(tiles[i].centroid[0] - sx, tiles[i].centroid[1] - sy) for i in cav)
            rb = solve_cavity_modelset_sat([tiles[i] for i in cav], lat, forbid_original=True, certify_drat=True)
            rb["seed_name"] = seed_name
            rb["seed_idx"] = seed_idx
            rb["radius"] = round(rad, 3)
            results["exp1d_modelset_large_cavities"].append(rb)
            if rb["status"] == "UNSAT":
                print(f"    k={len(cav):3d} (r={rad:5.2f}, dC={rb['n_bd_edges']:3d}): UNSAT [DRAT={rb['drat_verified']}] "
                      f"cands={rb['n_candidates']:5d} clauses={rb['n_clauses']:7d} enc={rb['t_encode']:.2f}s solve={rb['t_solve']:.3f}s drat={rb['t_drat']:.3f}s")
            elif rb["status"] == "SAT":
                print(f"    k={len(cav):3d} (r={rad:5.2f}): *** CLOSED FLIP CLUSTER FOUND! *** "
                      f"rem={rb['n_removed']} add={rb['n_added']} (dV,dE,dF)=({rb['dV']},{rb['dE']},{rb['dF']}) "
                      f"cands={rb['n_candidates']} solve={rb['t_solve']:.3f}s")
                break
            else:
                print(f"    k={len(cav):3d}: {rb['status']}")

    if out_json:
        with open(out_json, "w") as fh:
            json.dump(results, fh, indent=2)
        print(f"\nSaved results to {out_json}")
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--level", type=int, default=3)
    ap.add_argument("--out", default=os.path.join(UPGRADE_DIR, "results_cavity_sat.json"))
    a = ap.parse_args()
    run_all(level=a.level, out_json=a.out)


if __name__ == "__main__":
    main()
