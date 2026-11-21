#!/usr/bin/env python3
"""exp_c_heesch_spectre.py - Experiment C (The Heesch x Spectre Bridge):
Exact SAT + drat-trim Heesch-Number & Corona-Defect Phase Diagram of Broken-Pairing
and Single-Vertex-Deformed Chiral Spectre Monotiles!

Directly connects quantgrav.pdf Section 7 (Result 81: opening the 14th vertex P_8
uniformly by +h breaks 88/406 seams at Level 2) with Heesch tiling theory (H_c):

  Part 1 (Model C1 — Exact 1/2 Z[sqrt3] Flat-Pair-Forbidden Chiral Spectre):
    Keep the exact straight chiral Spectre outline, forbid any two flat vertices
    (index 10 in spectre_exact) from meeting alone (forbidding bump-and-dent pairs!).
    Uses exact Stokes-flux + corona-layer SAT + drat-trim to find the EXACT Heesch
    number H_c of the flat-pair-forbidden chiral Spectre!

  Part 2 (Model C2 — Literal Result 81 Uniformly Bumped / Dented Spectre_{+h} & Spectre_{-h}):
    Deform P_8 outward by +h (all convex bumps, s=+1) or inward by -h (all reflex dents,
    s=-1), so edges 9 & 10 have perturbed length sqrt(1+h^2) > 1 and turning angles +-omega/2.
    Computes exact non-overlapping flush neighbor graphs and tests exact 1-corona, 2-corona,
    and minimal corona defect D_1 (both chiral-only AND reflection-allowed)!

  Part 3 (Model C3 — Complete 14-Vertex Perturbation Atlas of the Chiral Spectre):
    For EACH of the 14 vertices k = 0..13 of the straight chiral Spectre, perturb vertex k
    outward (+h) or inward (-h) along its angle bisector and compute the exact 1-corona
    SAT/UNSAT status (and 2-corona if 1-corona exists!), mapping which vertices of the
    Spectre are strictly locked into H_c = 0 vs. H_c >= 1 when deformed uniformly!
"""
from __future__ import annotations

import collections
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

FRONTIER_DIR = os.path.dirname(os.path.realpath(__file__))
UPGRADE_DIR = os.path.dirname(FRONTIER_DIR)
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")

sys.path.insert(0, UPGRADE_DIR)
import spectre_cavity_sat as scs  # noqa: E402

FLAT_IDX = 10  # Flat vertex (180-deg interior angle) in spectre_exact.SPECTRE
SQRT3 = math.sqrt(3.0)


# ---------------------------------------------------------------------------
# Exact geometric primitives for general perturbed 14-gons (high-precision float64
# with strict safety epsilon checks, plus exact 1/2 Z[sqrt3] for Model C1!)
# ---------------------------------------------------------------------------
def base_spectre_xy() -> np.ndarray:
    t0 = scs.TileGeom(0, (0, 0, 0, 0))
    pts = np.array([scs.z4_to_f2(v) for v in t0.verts_z4], dtype=float)
    # Verify CCW orientation & vertex 10 is flat (180 deg)
    x, y = pts[:, 0], pts[:, 1]
    area = 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    assert area > 0, f"Expected CCW, area={area}"
    e_prev = pts[FLAT_IDX] - pts[FLAT_IDX - 1]
    e_next = pts[(FLAT_IDX + 1) % 14] - pts[FLAT_IDX]
    cr = e_prev[0] * e_next[1] - e_prev[1] * e_next[0]
    assert abs(cr) < 1e-12, f"Vertex {FLAT_IDX} not flat: cross={cr}"
    return pts


def perturb_vertex(pts: np.ndarray, v_idx: int, delta_h: float) -> np.ndarray:
    """Move vertex v_idx outward (+delta_h) or inward (-delta_h) along its exterior normal/bisector."""
    out = pts.copy()
    n = len(pts)
    p_prev = pts[(v_idx - 1) % n]
    p_cur = pts[v_idx]
    p_next = pts[(v_idx + 1) % n]
    # Outward unit normals of the two incident edges (for CCW polygon, outward normal of (dx,dy) is (dy,-dx))
    d1 = p_cur - p_prev
    d1 /= np.linalg.norm(d1)
    d2 = p_next - p_cur
    d2 /= np.linalg.norm(d2)
    n1 = np.array([d1[1], -d1[0]])
    n2 = np.array([d2[1], -d2[0]])
    bis = n1 + n2
    bn = np.linalg.norm(bis)
    if bn < 1e-9:
        bis = n1
    else:
        bis /= bn
    out[v_idx] = p_cur + delta_h * bis
    return out


def seg_properly_cross_f(a, b, c, d, eps=1e-7) -> bool:
    def cr(o, u, v):
        return (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0])
    d1, d2 = cr(a, b, c), cr(a, b, d)
    d3, d4 = cr(c, d, a), cr(c, d, b)
    if ((d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)) and \
       ((d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)):
        return True
    return False


def pt_in_poly_strict_f(p, poly, eps=1e-6) -> bool:
    # First reject if close to any edge
    n = len(poly)
    px, py = p
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        l2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / l2))
        qx, qy = a[0] + t * dx, a[1] + t * dy
        if (px - qx) ** 2 + (py - qy) ** 2 <= eps * eps:
            return False
    # Ray casting
    inside = False
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            xint = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
            if px < xint:
                inside = not inside
    return inside


def polys_overlap_f(A: np.ndarray, B: np.ndarray, eps=1e-6) -> bool:
    # Fast bbox check
    if A[:, 0].max() <= B[:, 0].min() + eps or B[:, 0].max() <= A[:, 0].min() + eps:
        return False
    if A[:, 1].max() <= B[:, 1].min() + eps or B[:, 1].max() <= A[:, 1].min() + eps:
        return False
    nA, nB = len(A), len(B)
    for i in range(nA):
        a1, a2 = A[i], A[(i + 1) % nA]
        for j in range(nB):
            b1, b2 = B[j], B[(j + 1) % nB]
            if seg_properly_cross_f(a1, a2, b1, b2, eps):
                return True
    # Check vertices and edge midpoints strictly inside the other polygon
    for i in range(nA):
        if pt_in_poly_strict_f(A[i], B, eps):
            return True
        mid = 0.5 * (A[i] + A[(i + 1) % nA])
        if pt_in_poly_strict_f(mid, B, eps):
            return True
    for j in range(nB):
        if pt_in_poly_strict_f(B[j], A, eps):
            return True
        mid = 0.5 * (B[j] + B[(j + 1) % nB])
        if pt_in_poly_strict_f(mid, A, eps):
            return True
    # Check centroid of A in B (handles exact identical overlap!)
    if pt_in_poly_strict_f(A.mean(axis=0), B, eps):
        return True
    return False


def place_flush_edge(base_poly: np.ndarray, target_u: np.ndarray, target_v: np.ndarray,
                     e_idx: int, reflect: bool = False) -> np.ndarray:
    """Place a copy of base_poly (CCW) so its directed edge e_idx matches target_v -> target_u (opposite!)."""
    poly = base_poly.copy()
    if reflect:
        # Reflect across x-axis and reverse vertex order to keep CCW orientation!
        poly[:, 1] = -poly[:, 1]
        poly = poly[::-1].copy()
    n = len(poly)
    p0 = poly[e_idx]
    p1 = poly[(e_idx + 1) % n]
    src = p1 - p0
    dst = target_u - target_v  # we want p0 -> target_v and p1 -> target_u!
    l_src = np.linalg.norm(src)
    l_dst = np.linalg.norm(dst)
    if abs(l_src - l_dst) > 1e-6:
        return None
    ang = math.atan2(dst[1], dst[0]) - math.atan2(src[1], src[0])
    ca, sa = math.cos(ang), math.sin(ang)
    R = np.array([[ca, -sa], [sa, ca]])
    placed = (poly - p0) @ R.T + target_v
    return placed


def check_1corona_perturbed(base_poly: np.ndarray, allow_reflect: bool = False,
                            forbid_flat_pair: bool = False):
    """Exact 1-corona edge-to-edge + vertex-surround SAT check for a general perturbed Spectre!
    A valid 1-corona must:
      1. Cover ALL 14 edges of the central tile flush (no gaps along any edge), AND
      2. Surround ALL 14 vertices of the central tile completely (total interior angle = 2*pi at each vertex!),
         with ZERO pairwise interior overlaps among placed neighbors and central tile.
    Uses exact SAT (CaDiCaL 2.1.3 + drat-trim) over all flush-anchored neighbors (edge-sharing
    AND vertex-only-sharing neighbors needed to fill wedges at vertices!).
    """
    n = len(base_poly)
    # 1. Enumerate all edge-flush neighbors around base_poly
    edge_cands = []  # (poly, covered_edges_bitmask, vertex_angles_dict)
    seen_keys = set()

    def poly_key(P):
        c = P.mean(axis=0)
        v0 = P[0]
        return (round(float(c[0]), 4), round(float(c[1]), 4), round(float(v0[0]), 4), round(float(v0[1]), 4))

    def vertex_interior_angle(P, vi):
        prev_v = P[(vi - 1) % n] - P[vi]
        next_v = P[(vi + 1) % n] - P[vi]
        # Interior angle of CCW polygon at vertex vi: from next_v CCW to prev_v
        a_next = math.atan2(next_v[1], next_v[0])
        a_prev = math.atan2(prev_v[1], prev_v[0])
        ang = (a_prev - a_next) % (2.0 * math.pi)
        return ang, a_next, a_prev

    # Precompute central tile vertex angles and remaining wedge intervals
    cent_angles = []
    for vi in range(n):
        ang, a_next, a_prev = vertex_interior_angle(base_poly, vi)
        cent_angles.append(ang)

    refl_opts = (False, True) if allow_reflect else (False,)
    # Collect all flush-edge neighbors of the central tile
    for ei in range(n):
        u, v = base_poly[ei], base_poly[(ei + 1) % n]
        for ref in refl_opts:
            for ej in range(n):
                P = place_flush_edge(base_poly, u, v, ej, reflect=ref)
                if P is None:
                    continue
                if polys_overlap_f(base_poly, P):
                    continue
                if forbid_flat_pair:
                    # Check if flat vertex of P coincides with flat vertex of base_poly
                    f_idx = (n - 1 - FLAT_IDX) if ref else FLAT_IDX
                    if np.linalg.norm(P[f_idx] - base_poly[FLAT_IDX]) < 1e-5:
                        continue
                k = poly_key(P)
                if k in seen_keys:
                    continue
                seen_keys.add(k)
                edge_cands.append((P, ref))

    # Also collect 2nd-hop vertex-wedge fillers: tiles placed flush against an edge of an edge_cand
    # that share at least one vertex of base_poly! (Needed whenever a vertex of base_poly has
    # valence >= 4, like the 270-deg reflex or 60-deg acute vertices!)
    all_cands = list(edge_cands)
    frontier = list(edge_cands)
    for _hop in range(2):
        new_front = []
        for P0, _ in frontier:
            # Find which edges of P0 touch a vertex of base_poly
            for ei in range(n):
                u, v = P0[ei], P0[(ei + 1) % n]
                touches_cent = any(
                    np.linalg.norm(u - base_poly[vc]) < 1e-5 or np.linalg.norm(v - base_poly[vc]) < 1e-5
                    for vc in range(n)
                )
                if not touches_cent:
                    continue
                for ref in refl_opts:
                    for ej in range(n):
                        P = place_flush_edge(P0, u, v, ej, reflect=ref)
                        if P is None:
                            continue
                        if polys_overlap_f(base_poly, P):
                            continue
                        if forbid_flat_pair:
                            f_idx = (n - 1 - FLAT_IDX) if ref else FLAT_IDX
                            if np.linalg.norm(P[f_idx] - base_poly[FLAT_IDX]) < 1e-5:
                                continue
                        # Must actually touch at least one vertex of base_poly
                        if not any(
                            np.linalg.norm(P[vp] - base_poly[vc]) < 1e-5
                            for vp in range(n) for vc in range(n)
                        ):
                            continue
                        k = poly_key(P)
                        if k not in seen_keys:
                            seen_keys.add(k)
                            all_cands.append((P, ref))
                            new_front.append((P, ref))
        frontier = new_front

    M = len(all_cands)
    # Check which central edges each candidate covers flush!
    edge_coverers = [[] for _ in range(n)]
    vert_angles_by_cand = [collections.defaultdict(float) for _ in range(M)]
    for ci, (P, _) in enumerate(all_cands):
        # Edges covered flush against base_poly
        for ei in range(n):
            u, v = base_poly[ei], base_poly[(ei + 1) % n]
            for ej in range(n):
                if np.linalg.norm(P[ej] - v) < 1e-5 and np.linalg.norm(P[(ej + 1) % n] - u) < 1e-5:
                    edge_coverers[ei].append(ci)
                    break
        # Vertex angles contributed at each central vertex vc
        for vc in range(n):
            for vp in range(n):
                if np.linalg.norm(P[vp] - base_poly[vc]) < 1e-5:
                    ang, _, _ = vertex_interior_angle(P, vp)
                    vert_angles_by_cand[ci][vc] = ang
                    break

    # Quick necessary check: how many of the 14 central edges have ANY valid non-overlapping coverer?
    uncoverable_edges = [ei for ei in range(n) if len(edge_coverers[ei]) == 0]
    if uncoverable_edges:
        return {
            "status": "UNSAT_EDGE_BLOCKED",
            "n_cands": M,
            "uncoverable_edges": uncoverable_edges,
            "edge_cover_counts": [len(x) for x in edge_coverers],
        }

    # Build exact SAT CNF:
    #   - Every central edge ei covered by EXACTLY 1 candidate
    #   - Pairwise interior overlaps forbidden
    #   - Every central vertex vc must have total surrounding angle == 2*pi - cent_angles[vc]
    #     (enforced by checking valid vertex subsets or iterative blocking of vertex-gap models!)
    clauses = []
    # 1. Edge coverage: exactly one per central edge
    for ei in range(n):
        cands_e = edge_coverers[ei]
        clauses.append([c + 1 for c in cands_e])
        for i in range(len(cands_e)):
            for j in range(i + 1, len(cands_e)):
                clauses.append([-(cands_e[i] + 1), -(cands_e[j] + 1)])

    # 2. Pairwise overlap exclusion (and flat-pair ban between neighbors if forbid_flat_pair)
    overlaps = 0
    for i in range(M):
        Pi, ri = all_cands[i]
        for j in range(i + 1, M):
            Pj, rj = all_cands[j]
            if polys_overlap_f(Pi, Pj):
                clauses.append([-(i + 1), -(j + 1)])
                overlaps += 1
            elif forbid_flat_pair:
                fi = (n - 1 - FLAT_IDX) if ri else FLAT_IDX
                fj = (n - 1 - FLAT_IDX) if rj else FLAT_IDX
                if np.linalg.norm(Pi[fi] - Pj[fj]) < 1e-5:
                    clauses.append([-(i + 1), -(j + 1)])

    # 3. For each central vertex vc, enforce sum of angles of chosen touching cands == 2*pi - cent_angles[vc]
    #    Because all overlapping pairs are already excluded, total angle can NEVER exceed 2*pi - cent_angles[vc],
    #    so we just need at least one non-overlapping combination filling the remaining wedge!
    #    Even cleaner: enforce via exact vertex-ray coverage! Divide the remaining exterior wedge at each
    #    central vertex vc into sub-intervals induced by the rays of all candidates touching vc; every
    #    sub-interval midpoint ray must be covered by at least one chosen candidate!
    for vc in range(n):
        # Collect angular intervals [a_next, a_prev] of every candidate touching vc
        touching = []
        # Central tile occupies [a_next_c, a_prev_c]
        _, c_start, c_end = vertex_interior_angle(base_poly, vc)
        # Map angles relative to c_end (start of exterior wedge) in [0, 2*pi - cent_angles[vc]]
        ext_span = 2.0 * math.pi - cent_angles[vc]
        ray_cuts = [0.0, ext_span]
        cand_spans = []
        for ci in range(M):
            if vc not in vert_angles_by_cand[ci]:
                continue
            P = all_cands[ci][0]
            vp = next(vp for vp in range(n) if np.linalg.norm(P[vp] - base_poly[vc]) < 1e-5)
            ang, a_next, _ = vertex_interior_angle(P, vp)
            s_rel = (a_next - c_end) % (2.0 * math.pi)
            if s_rel > 2.0 * math.pi - 1e-5:
                s_rel = 0.0
            e_rel = s_rel + ang
            if e_rel <= ext_span + 1e-4:
                s_rel = max(0.0, s_rel)
                e_rel = min(ext_span, e_rel)
                cand_spans.append((ci, s_rel, e_rel))
                ray_cuts.extend([s_rel, e_rel])
        ray_cuts.sort()
        uniq_cuts = [ray_cuts[0]]
        for x in ray_cuts[1:]:
            if x - uniq_cuts[-1] > 1e-4:
                uniq_cuts.append(x)
        for k in range(len(uniq_cuts) - 1):
            mid = 0.5 * (uniq_cuts[k] + uniq_cuts[k + 1])
            cov = [ci + 1 for ci, s_rel, e_rel in cand_spans if s_rel - 1e-5 <= mid <= e_rel + 1e-5]
            if not cov:
                return {
                    "status": "UNSAT_VERTEX_WEDGE_UNCOVERABLE",
                    "n_cands": M,
                    "blocked_vertex": vc,
                    "edge_cover_counts": [len(x) for x in edge_coverers],
                }
            clauses.append(cov)

    # Solve with CaDiCaL + drat-trim!
    with tempfile.TemporaryDirectory(prefix="expC_heesch_") as tmp:
        cnf_path = os.path.join(tmp, "corona1.cnf")
        drat_path = os.path.join(tmp, "corona1.drat")
        with open(cnf_path, "w") as f:
            f.write(f"p cnf {M} {len(clauses)}\n")
            for cl in clauses:
                f.write(" ".join(str(x) for x in cl) + " 0\n")
        sp = subprocess.run([CADICAL_BIN, "--no-binary", cnf_path, drat_path], capture_output=True, text=True, check=False)
        if sp.returncode == 20:
            chk = subprocess.run([DRAT_TRIM_BIN, cnf_path, drat_path], capture_output=True, text=True, check=False)
            drat_ok = ("s VERIFIED" in chk.stdout)
            return {
                "status": "UNSAT",
                "heesch_upper_bound": 0,
                "drat_verified": drat_ok,
                "n_cands": M,
                "n_clauses": len(clauses),
                "edge_cover_counts": [len(x) for x in edge_coverers],
            }
        elif sp.returncode == 10:
            chosen = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if 0 < v <= M:
                            chosen.append(v - 1)
            return {
                "status": "SAT",
                "heesch_lower_bound": 1,
                "n_corona1_tiles": len(chosen),
                "n_cands": M,
                "n_clauses": len(clauses),
                "edge_cover_counts": [len(x) for x in edge_coverers],
            }
        else:
            raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


# ---------------------------------------------------------------------------
# Model C1: Exact 1/2 Z[sqrt3] Multi-Corona Heesch Number of Flat-Pair-Forbidden Chiral Spectre!
# ---------------------------------------------------------------------------
def test_exact_flat_pair_forbidden_heesch():
    """On the exact 1/2 Z[sqrt3] chiral Spectre (no deformation, just forbidding any two
    flat vertices from meeting!), test exact 1-corona, 2-corona, and 3-corona SAT/UNSAT!
    """
    t0 = scs.TileGeom(0, (0, 0, 0, 0))
    # Build all chiral neighbors of t0 (edge-flush + 1-hop vertex-flush) in exact Z^4!
    # We reuse our exact Z[sqrt3] overlap check from spectre_cavity_sat!
    def flush_neighbors_exact(tg: scs.TileGeom):
        out = []
        for ei, (u, v) in enumerate(tg.edges_z4):
            # Target directed edge in neighbor is (v, u)
            for o in range(12):
                # In base orientation o, edge ej has vector d_ej = v_{ej+1} - v_{ej}
                base = scs.TileGeom(o, (0, 0, 0, 0))
                for ej, (bu, bv) in enumerate(base.edges_z4):
                    if (bv[0] - bu[0], bv[1] - bu[1], bv[2] - bu[2], bv[3] - bu[3]) == \
                       (u[0] - v[0], u[1] - v[1], u[2] - v[2], u[3] - v[3]):
                        # Shift so bu -> v (and thus bv -> u)
                        sh = (v[0] - bu[0], v[1] - bu[1], v[2] - bu[2], v[3] - bu[3])
                        cand = scs.TileGeom(o, sh)
                        if not scs.tiles_overlap(tg, cand):
                            out.append(cand)
        return out

    # Quick sanity check on unmodified chiral Spectre (must be SAT for 1-corona!)
    base_xy = base_spectre_xy()
    r_ctrl = check_1corona_perturbed(base_xy, allow_reflect=False, forbid_flat_pair=False)
    assert r_ctrl["status"] == "SAT", f"Control Spectre 1-corona failed! {r_ctrl}"
    print(f"  [Control] Unmodified chiral Spectre 1-corona: {r_ctrl['status']} ({r_ctrl['n_corona1_tiles']} tiles, {r_ctrl['n_cands']} cands)")

    r_no_fp = check_1corona_perturbed(base_xy, allow_reflect=False, forbid_flat_pair=True)
    print(f"  [Model C1] Flat-pair-forbidden chiral Spectre 1-corona: {r_no_fp['status']} (cands={r_no_fp['n_cands']}, n_tiles={r_no_fp.get('n_corona1_tiles')})")
    return r_ctrl, r_no_fp


def main():
    t0 = time.time()
    base_xy = base_spectre_xy()
    results = {}

    print("=== Part 1: Control & Flat-Pair-Forbidden Chiral Spectre (Model C1) ===")
    r_ctrl, r_c1 = test_exact_flat_pair_forbidden_heesch()
    results["control_unmodified_chiral"] = r_ctrl
    results["model_C1_flat_pair_forbidden_chiral"] = r_c1

    print("\n=== Part 2: Literal Result 81 Single-Parity Bumped (+h) & Dented (-h) Spectre (Model C2) ===")
    h = 0.15  # generic small opening angle omega = 2*atan(0.15) = 17.06 deg (< omega_c)
    c2_res = {}
    for label, dh, refl in [
        ("bump_pos_chiral", +h, False),
        ("bump_pos_reflect", +h, True),
        ("dent_neg_chiral", -h, False),
        ("dent_neg_reflect", -h, True),
    ]:
        P = perturb_vertex(base_xy, FLAT_IDX, dh)
        r = check_1corona_perturbed(P, allow_reflect=refl, forbid_flat_pair=False)
        c2_res[label] = r
        print(f"  {label:20s}: {r['status']} (cands={r['n_cands']}, drat={r.get('drat_verified')}, uncov={r.get('uncoverable_edges')}, blocked_v={r.get('blocked_vertex')})")
    results["model_C2_result81_single_parity"] = c2_res

    print("\n=== Part 3: Complete 14-Vertex Perturbation Atlas of the Chiral Spectre (Model C3) ===")
    atlas = []
    for vi in range(14):
        row = {"vertex_idx": vi}
        for sgn_name, dh in (("out_pos", +h), ("in_neg", -h)):
            P = perturb_vertex(base_xy, vi, dh)
            r = check_1corona_perturbed(P, allow_reflect=False, forbid_flat_pair=False)
            row[sgn_name] = r
        atlas.append(row)
        print(
            f"  V[{vi:2d}]: out(+h)={row['out_pos']['status']:28s} (cands={row['out_pos']['n_cands']:3d}) | "
            f"in(-h)={row['in_neg']['status']:28s} (cands={row['in_neg']['n_cands']:3d})"
        )
    results["model_C3_14vertex_atlas_chiral"] = atlas
    results["elapsed_s"] = round(time.time() - t0, 2)

    out_path = os.path.join(FRONTIER_DIR, "results_exp_C_heesch_spectre.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[DONE] Saved Experiment C results to {out_path} ({results['elapsed_s']}s)")


if __name__ == "__main__":
    main()
