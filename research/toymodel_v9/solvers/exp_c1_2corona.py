#!/usr/bin/env python3
"""exp_c1_2corona.py - Exact 1/2 Z[sqrt3] SAT + drat-trim 2-corona test for Model C1:
the chiral straight Spectre with the single combinatorial rule 'no two flat vertices may meet'
(forbidding bump-and-dent pairs!).

We already proved:
  - Literal single-parity deformed Spectre (+h or -h): H_c = 0 (DRAT-UNSAT on 1-corona)
  - All 28/28 single-vertex perturbations of the chiral Spectre: H_c = 0
  - Flat-pair-forbidden straight chiral Spectre (Model C1): admits a 6-tile 1-corona (H_c >= 1)!

Here we test exact 1-corona and 2-corona on the Result-66 index-36 sublattice (and unrestricted
flush-anchored neighbor cloud!) for Model C1 to determine whether forbidding flat pairs produces
a finite Heesch number H_c = 1 or >= 2!
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

FRONTIER_DIR = os.path.dirname(os.path.realpath(__file__))
UPGRADE_DIR = os.path.dirname(FRONTIER_DIR)
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")

sys.path.insert(0, UPGRADE_DIR)
import spectre_cavity_sat as scs  # noqa: E402
import run_exp_A_B as expab  # noqa: E402

FLAT_IDX = 10


def solve_corona_k_no_flat_pair(k_corona: int, forbid_fp: bool = True, rad_box: float = 11.5):
    """Exact 1/2 Z[sqrt3] corona-k SAT solver on the Result-66 index-36 chiral Spectre lattice!
    Every tile t in layer L (0..k-1) must have ALL 14 of its directed edges matched by opposite
    edges of tiles in layers <= L+1 (exact Stokes flux around layers 0..k-1!), with zero overlaps
    and zero flat-vertex-to-flat-vertex contacts if forbid_fp=True!
    """
    t0 = time.time()
    tiles, kinds = scs.build_chiral_patch(3, "Delta")
    lat = scs.extract_model_set_lattice(tiles, kinds)

    # Pick a central interior tile t0 of Level 3 as the seed (layer 0)
    cx = sum(0.5 * (t.bbox[0] + t.bbox[2]) for t in tiles) / len(tiles)
    cy = sum(0.5 * (t.bbox[1] + t.bbox[3]) for t in tiles) / len(tiles)
    seed_tg = min(tiles, key=lambda t: math.hypot(0.5 * (t.bbox[0] + t.bbox[2]) - cx, 0.5 * (t.bbox[1] + t.bbox[3]) - cy))
    sx = 0.5 * (seed_tg.bbox[0] + seed_tg.bbox[2])
    sy = 0.5 * (seed_tg.bbox[1] + seed_tg.bbox[3])

    raw_cands = expab.enumerate_raw_modelset_for_bbox((sx - rad_box, sy - rad_box, sx + rad_box, sy + rad_box), lat, internal_margin=1.5)
    cands = [seed_tg]
    seen = {seed_tg.key}
    for o, sh in raw_cands:
        tg = scs.TileGeom(o, sh)
        if tg.key in seen:
            continue
        mx = 0.5 * (tg.bbox[0] + tg.bbox[2])
        my = 0.5 * (tg.bbox[1] + tg.bbox[3])
        if math.hypot(mx - sx, my - sy) <= rad_box:
            if not scs.tiles_overlap(seed_tg, tg):
                seen.add(tg.key)
                cands.append(tg)
    M = len(cands)

    # Index directed edges and flat vertices
    edge_to_cands = collections.defaultdict(list)
    flat_v_to_cands = collections.defaultdict(list)
    for i, tg in enumerate(cands):
        for e in tg.edges_z4:
            edge_to_cands[e].append(i)
        flat_v_to_cands[tg.verts_z4[FLAT_IDX]].append(i)

    # Precompute neighbor adjacency (shared vertex or shared edge!) and overlaps
    # For corona definition: tile j is adjacent to tile i if they share ANY vertex (and don't overlap)
    vert_to_cands = collections.defaultdict(list)
    for i, tg in enumerate(cands):
        for v in tg.verts_z4:
            vert_to_cands[v].append(i)

    # Build layer variables: x[L, i] means tile i is placed in layer L (for L = 1..k_corona)
    # Seed tile 0 is FIXED in layer 0!
    # A tile i can only be in layer 1 if it shares a vertex with seed 0 (and not overlap seed 0).
    layer_cands = [{0}]
    for L in range(1, k_corona + 1):
        prev = layer_cands[-1]
        cur = set()
        for i in prev:
            for v in cands[i].verts_z4:
                for j in vert_to_cands[v]:
                    if j != 0:
                        cur.add(j)
        layer_cands.append(cur)

    # Assign SAT variable IDs:
    #   occ[i] = 1..M-1 (whether candidate i >= 1 is placed in ANY layer 1..k_corona)
    #   lay[L, i] (for L = 1..k_corona-1, i in layer_cands[L]): whether tile i is in layer <= L
    # Actually even cleaner for k_corona = 1 or 2:
    #   - If k_corona == 1: every edge of seed 0 is covered by occ[j], AND every vertex wedge of seed 0 is full
    #     (Wait: since tiles are non-overlapping on exact Z[sqrt3], if we require ALL edges of seed 0 covered
    #     AND every vertex of seed 0 has zero open interior edges of layer-1 tiles touching it -> exact 1-corona!)
    #   - For general k_corona (1 or 2):
    #     Variables: var_L[(L, i)] for L in 1..k_corona, i in layer_cands[L] (tile i placed in layer L).
    #     Constraints:
    #       (1) Each candidate i placed in at most one layer L.
    #       (2) Overlap exclusion: if i, j overlap, cannot place both in any layers.
    #       (3) If forbid_fp: if i, j share their flat vertex V[10], cannot place both! (Also ban any j sharing V[10] with seed 0!)
    #       (4) Exact surrounding of layer 0 (seed 0): every edge e of seed 0 must be matched by rev(e) of some layer-1 tile,
    #           AND every vertex v of seed 0 must have total angle 2*pi (equivalently: for every tile i in layer 1 touching v,
    #           any directed edge of i incident to v MUST be matched by rev(e) of either seed 0 or another layer-1 tile!).
    #       (5) Exact surrounding of layer L (1 <= L <= k_corona - 1): if tile i is in layer L, then
    #           (a) every directed edge e of tile i MUST be matched by rev(e) of some tile in layers {L-1, L, L+1}, AND
    #           (b) every tile j sharing a vertex with i that is placed in layer L+1... wait: for every vertex v of tile i,
    #               every directed edge incident to v of any tile touching v must cancel!
    #           Even simpler & 100% sufficient/necessary for exact edge-to-edge tiling:
    #           If tile i is in layer L (0 <= L < k_corona), then for EVERY edge e of tile i, rev(e) is covered by
    #           a tile in layers <= L+1; AND for EVERY vertex v of tile i, any placed tile j in layer <= L+1 that touches v
    #           has BOTH of its edges incident to v matched by tiles in layers <= L+1!
    #           (Think about that! At any vertex v of a layer-L tile, the union of non-overlapping polygons around v
    #           has NO boundary rays at v iff every edge incident to v cancels its reverse — which guarantees the
    #           total angle around v is EXACTLY 2*pi with ZERO gaps!! That's 100% exact Stokes-vertex closure!)
    var_id = {}
    nxt_v = 1
    for L in range(1, k_corona + 1):
        for i in sorted(layer_cands[L]):
            var_id[(L, i)] = nxt_v
            nxt_v += 1
    # Helper var occ[i] = OR_L var_id[(L, i)] for each active candidate i >= 1
    active_cands = sorted({i for L in range(1, k_corona + 1) for i in layer_cands[L]})
    occ_var = {}
    for i in active_cands:
        occ_var[i] = nxt_v
        nxt_v += 1

    clauses = []
    # Link occ_var[i] <-> OR_L var_id[(L, i)] and at-most-one layer per tile
    for i in active_cands:
        lvars = [var_id[(L, i)] for L in range(1, k_corona + 1) if (L, i) in var_id]
        for lv in lvars:
            clauses.append([-lv, occ_var[i]])
        clauses.append([-occ_var[i]] + lvars)
        for a in range(len(lvars)):
            for b in range(a + 1, len(lvars)):
                clauses.append([-lvars[a], -lvars[b]])

    # Flat-pair ban (seed 0 and between any two active tiles)
    if forbid_fp:
        fv0 = seed_tg.verts_z4[FLAT_IDX]
        for i in active_cands:
            if cands[i].verts_z4[FLAT_IDX] == fv0:
                clauses.append([-occ_var[i]])
        for fv, idxs in flat_v_to_cands.items():
            act = [i for i in idxs if i in occ_var]
            for a in range(len(act)):
                for b in range(a + 1, len(act)):
                    clauses.append([-occ_var[act[a]], -occ_var[act[b]]])

    # Pairwise overlap exclusion on occ_var
    n_ov = 0
    for ai in range(len(active_cands)):
        i = active_cands[ai]
        bi, ti = cands[i].bbox, cands[i]
        for aj in range(ai + 1, len(active_cands)):
            j = active_cands[aj]
            if scs.bbox_may_overlap(bi, cands[j].bbox) and scs.tiles_overlap(ti, cands[j]):
                clauses.append([-occ_var[i], -occ_var[j]])
                n_ov += 1

    # Map directed edge -> list of layer vars in layers <= L_max
    def vars_for_edge_upto(e, L_max):
        res = []
        for i in edge_to_cands.get(e, []):
            if i == 0:
                return None  # covered by fixed seed 0!
            for L in range(1, min(L_max, k_corona) + 1):
                if (L, i) in var_id:
                    res.append(var_id[(L, i)])
        return res

    # Exact Stokes edge & vertex-ring closure around every layer L in 0..k_corona-1:
    # Layer 0 (fixed seed 0):
    for e in seed_tg.edges_z4:
        rev_vs = vars_for_edge_upto((e[1], e[0]), 1)
        assert rev_vs is not None
        clauses.append(rev_vs)
    for v in seed_tg.verts_z4:
        # Every layer-1 tile j touching v must have BOTH its directed edges incident to v matched by layer <= 1 (or seed 0)
        for j in vert_to_cands[v]:
            if j == 0 or (1, j) not in var_id:
                continue
            vj = var_id[(1, j)]
            for e in cands[j].edges_z4:
                if e[0] == v or e[1] == v:
                    rev_vs = vars_for_edge_upto((e[1], e[0]), 1)
                    if rev_vs is not None:  # not matched by seed 0
                        clauses.append([-vj] + rev_vs)

    # Layers 1..k_corona-1:
    for L in range(1, k_corona):
        for i in sorted(layer_cands[L]):
            vi = var_id[(L, i)]
            # All 14 edges of tile i must be matched by layer <= L+1 (or seed 0)
            for e in cands[i].edges_z4:
                rev_vs = vars_for_edge_upto((e[1], e[0]), L + 1)
                if rev_vs is not None:
                    clauses.append([-vi] + rev_vs)
            # Vertex-ring closure at all 14 vertices of tile i:
            # If tile i is in layer L AND tile j (in layer <= L+1) touches vertex v of tile i,
            # then j's two edges incident to v must be matched by layer <= L+1 (or seed 0)!
            for v in cands[i].verts_z4:
                for j in vert_to_cands[v]:
                    if j == 0 or j == i or j not in occ_var:
                        continue
                    # Only restrict j in layers <= L+1
                    j_lvars = [var_id[(Lj, j)] for Lj in range(1, L + 2) if (Lj, j) in var_id]
                    if not j_lvars:
                        continue
                    for e in cands[j].edges_z4:
                        if e[0] == v or e[1] == v:
                            rev_vs = vars_for_edge_upto((e[1], e[0]), L + 1)
                            if rev_vs is not None:
                                for vj in j_lvars:
                                    clauses.append([-vi, -vj] + rev_vs)

    t_enc = time.time() - t0
    with tempfile.TemporaryDirectory(prefix="c1_cor_") as tmp:
        cnf_path = os.path.join(tmp, "cor.cnf")
        drat_path = os.path.join(tmp, "cor.drat")
        with open(cnf_path, "w") as f:
            f.write(f"p cnf {nxt_v - 1} {len(clauses)}\n")
            for cl in clauses:
                f.write(" ".join(str(x) for x in cl) + " 0\n")
        t1 = time.time()
        sp = subprocess.run([CADICAL_BIN, "--no-binary", cnf_path, drat_path], capture_output=True, text=True, check=False)
        t_solve = time.time() - t1
        if sp.returncode == 20:
            chk = subprocess.run([DRAT_TRIM_BIN, cnf_path, drat_path], capture_output=True, text=True, check=False)
            drat_ok = ("s VERIFIED" in chk.stdout)
            return {
                "k_corona": k_corona, "forbid_fp": forbid_fp, "status": "UNSAT",
                "drat_verified": drat_ok, "n_cands": len(active_cands),
                "n_clauses": len(clauses), "t_encode": round(t_enc, 2), "t_solve": round(t_solve, 3),
            }
        elif sp.returncode == 10:
            chosen = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if v > 0:
                            chosen.append(v)
            ch_set = set(chosen)
            placed = [i for i in active_cands if occ_var[i] in ch_set]
            return {
                "k_corona": k_corona, "forbid_fp": forbid_fp, "status": "SAT",
                "n_placed_tiles": len(placed) + 1, "n_cands": len(active_cands),
                "n_clauses": len(clauses), "t_encode": round(t_enc, 2), "t_solve": round(t_solve, 3),
            }
        else:
            raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


def main():
    res = {}
    for label, k, ffp, rb in [
        ("control_cor1", 1, False, 8.0),
        ("no_fp_cor1", 1, True, 8.0),
        ("control_cor2", 2, False, 12.5),
        ("no_fp_cor2", 2, True, 12.5),
    ]:
        r = solve_corona_k_no_flat_pair(k, ffp, rb)
        res[label] = r
        print(f"  {label:16s}: {r['status']:5s} (drat={r.get('drat_verified')}, tiles={r.get('n_placed_tiles')}, cands={r['n_cands']}, clauses={r['n_clauses']}, t={r['t_solve']}s)", flush=True)
    out_path = os.path.join(FRONTIER_DIR, "results_exp_C1_2corona.json")
    with open(out_path, "w") as f:
        json.dump(res, f, indent=2)
    print(f"[DONE] Saved {out_path}")


if __name__ == "__main__":
    main()
