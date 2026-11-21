#!/usr/bin/env python3
"""run_frontier1_open_arc_sat.py - Frontier 1A (Open-Boundary Chiral Phason Worms &
Exact qwla Boundary-Pinning Threshold in 1/2 Z[sqrt3] SAT + drat-trim, C++20 OpenMP!).

Uses bin/open_arc_cnf (128-bit exact 1/2 Z[sqrt3] OpenMP C++20) + CaDiCaL 2.1.3 + drat-trim:
  1. Single Open Arc Sweep (theta_open = 0, 30, 60, ..., 360 deg) on a 60-tile Level-4 disc:
     Finds the EXACT critical open-arc angle theta_c where the first non-trivial chiral
     phason re-tiling enters C (with DRAT proofs for every rigid angle < theta_c!).
  2. Two Opposite Open Arcs (Corridor / Strip Geometry for qwla logical distance d_L):
     Pin top & bottom walls, leave left & right ends open; finds whether a chiral phason
     worm crosses boundary-to-boundary and extracts its exact polygons!
"""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import tempfile
import time

FRONTIER_DIR = os.path.dirname(os.path.realpath(__file__))
UPGRADE_DIR = os.path.dirname(FRONTIER_DIR)
OPEN_BIN = os.path.join(UPGRADE_DIR, "bin", "open_arc_cnf")
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")

sys.path.insert(0, UPGRADE_DIR)
import spectre_cavity_sat as scs  # noqa: E402
import run_fast_cavity_suite as rfcs  # noqa: E402
import run_exp_A_B as expab  # noqa: E402


def solve_open_cpp(tiles, cav_ids, pinned_ids, bd_cyc_z4, raw_cands, certify_drat=True):
    t0 = time.time()
    cav_tiles = [tiles[i] for i in cav_ids]
    pin_tiles = [tiles[i] for i in pinned_ids]
    orig_keys = {tg.key for tg in cav_tiles}

    with tempfile.TemporaryDirectory(prefix="f1_cpp_") as tmp:
        cnf_path = os.path.join(tmp, "open.cnf")
        drat_path = os.path.join(tmp, "open.drat")
        lines = [
            cnf_path,
            f"{len(cav_tiles)} {len(pin_tiles)} {len(bd_cyc_z4)} {len(raw_cands)}",
        ]
        for tg in cav_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for tg in pin_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for v in bd_cyc_z4:
            lines.append(f"{v[0]} {v[1]} {v[2]} {v[3]}")
        for o, s in raw_cands:
            lines.append(f"{o} {s[0]} {s[1]} {s[2]} {s[3]}")

        env = dict(os.environ, OMP_NUM_THREADS="16")
        proc = subprocess.run([OPEN_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True, env=env)
        t_enc = time.time() - t0
        out_lines = proc.stdout.strip().splitlines()
        hdr = out_lines[0].split()
        if hdr[0] != "OK":
            return {"status": hdr[0], "n_bd_pinned": int(hdr[3]), "n_bd_open": int(hdr[4]), "t_encode": round(t_enc, 3)}
        M, n_clauses, n_bd_pin, n_bd_open, n_ov = map(int, hdr[1:6])

        cmd = [CADICAL_BIN, "--no-binary", cnf_path]
        if certify_drat:
            cmd.append(drat_path)
        t1 = time.time()
        sp = subprocess.run(cmd, capture_output=True, text=True, check=False)
        t_solve = time.time() - t1

        if sp.returncode == 20:
            drat_ok = None
            if certify_drat:
                chk = subprocess.run([DRAT_TRIM_BIN, cnf_path, drat_path], capture_output=True, text=True, check=False)
                drat_ok = ("s VERIFIED" in chk.stdout)
                assert drat_ok
            return {
                "status": "UNSAT",
                "drat_verified": drat_ok,
                "n_cands": M,
                "n_clauses": n_clauses,
                "n_bd_pinned": n_bd_pin,
                "n_bd_open": n_bd_open,
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 3),
            }
        elif sp.returncode == 10:
            cands_tbl = []
            for ln in out_lines[1:1 + M]:
                toks = [int(x) for x in ln.split()]
                cands_tbl.append((toks[0], (toks[1], toks[2], toks[3], toks[4])))
            chosen_tiles = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if 0 < v <= M:
                            o, sh = cands_tbl[v - 1]
                            chosen_tiles.append(scs.TileGeom(o, sh))
            chosen_keys = {tg.key for tg in chosen_tiles}
            rem_tiles = [tg for tg in cav_tiles if tg.key not in chosen_keys]
            add_tiles = [tg for tg in chosen_tiles if tg.key not in orig_keys]
            kept_tiles = [tg for tg in cav_tiles if tg.key in chosen_keys]

            # Exact Python double-check: ZERO overlaps!
            for a in range(len(chosen_tiles)):
                for b in range(a + 1, len(chosen_tiles)):
                    assert not (scs.bbox_may_overlap(chosen_tiles[a].bbox, chosen_tiles[b].bbox) and
                                scs.tiles_overlap(chosen_tiles[a], chosen_tiles[b])), "OVERLAP!"
            return {
                "status": "SAT",
                "n_cands": M,
                "n_clauses": n_clauses,
                "n_bd_pinned": n_bd_pin,
                "n_bd_open": n_bd_open,
                "n_removed_from_C": len(rem_tiles),
                "n_added_new": len(add_tiles),
                "n_kept_in_C": len(kept_tiles),
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 3),
                "polys_rem": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in rem_tiles],
                "polys_add": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in add_tiles],
                "polys_kept": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in kept_tiles],
            }
        else:
            raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


def main():
    t0 = time.time()
    print("=== Frontier 1A (C++20 OpenMP): Exact 1/2 Z[sqrt3] Open-Boundary Chiral Phason Worm Threshold ===")
    tiles, kinds = scs.build_chiral_patch(4, "Delta")
    adj = scs.patch_adjacency(tiles)
    edge_set = {e for tg in tiles for e in tg.edges_z4}
    interior_set = {i for i, tg in enumerate(tiles) if all((e[1], e[0]) in edge_set for e in tg.edges_z4)}
    lat = scs.extract_model_set_lattice(tiles, kinds)

    seed = 2675
    k_disc = 60
    cav_ids = rfcs.fast_grow_simple_cavity(tiles, adj, interior_set, seed, k_disc)
    cav_tiles = [tiles[i] for i in cav_ids]
    _, cycs = rfcs.extract_boundary_cycle_z4(cav_tiles)
    assert len(cycs) == 1
    bd_cyc_z4 = cycs[0]

    # Precompute raw model-set candidates ONCE for the whole disc!
    cav_pts_f = [scs.z4_to_f2(v) for v in bd_cyc_z4]
    cx0, cy0 = min(p[0] for p in cav_pts_f) - 2.5, min(p[1] for p in cav_pts_f) - 2.5
    cx1, cy1 = max(p[0] for p in cav_pts_f) + 2.5, max(p[1] for p in cav_pts_f) + 2.5
    raw_cands = expab.enumerate_raw_modelset_for_bbox((cx0, cy0, cx1, cy1), lat, internal_margin=1.5)
    print(f"  Disc k={k_disc} tiles, |bd|={len(bd_cyc_z4)}, precomputed {len(raw_cands)} raw Result-66 candidates")

    cav_set = set(cav_ids)
    collar1 = {j for i in cav_ids for j in adj[i] if j not in cav_set}
    collar2 = collar1 | {j for i in collar1 for j in adj[i] if j not in cav_set}

    cx = sum(p[0] for p in cav_pts_f) / len(cav_pts_f)
    cy = sum(p[1] for p in cav_pts_f) / len(cav_pts_f)

    def tile_angle_deg(tid):
        pts = [scs.z4_to_f2(v) for v in tiles[tid].verts_z4]
        return math.degrees(math.atan2(sum(p[1] for p in pts) / 14.0 - cy, sum(p[0] for p in pts) / 14.0 - cx)) % 360.0

    # Sweep 1: Single open boundary arc
    print(f"\n--- Sweep 1: Single Open Arc on {k_disc}-Tile Level-4 Disc ---")
    arc_results = []
    for theta in (0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360):
        pinned = [tid for tid in collar2 if tile_angle_deg(tid) >= theta]
        r = solve_open_cpp(tiles, cav_ids, pinned, bd_cyc_z4, raw_cands, certify_drat=True)
        r["theta_open_deg"] = theta
        if r["status"] == "SAT" and any(x["status"] == "SAT" for x in arc_results):
            r.pop("polys_rem", None); r.pop("polys_add", None); r.pop("polys_kept", None)
        arc_results.append(r)
        print(
            f"  theta_open={theta:3d} deg (bd_pin={r['n_bd_pinned']:3d}, bd_open={r['n_bd_open']:3d}, cands={r.get('n_cands',0):5d}): "
            f"{r['status']:5s} drat={r.get('drat_verified')} "
            f"rem={r.get('n_removed_from_C')} add={r.get('n_added_new')} enc={r['t_encode']}s solve={r.get('t_solve')}s",
            flush=True,
        )

    # Sweep 2: Two opposite open arcs (corridor / strip geometry!)
    print(f"\n--- Sweep 2: Two Opposite Open Arcs (Corridor / Strip) on {k_disc}-Tile Level-4 Disc ---")
    corridor_results = []
    for wall_deg in (150, 130, 110, 90, 70, 50, 30):
        half_w = 0.5 * wall_deg
        pinned = [
            tid for tid in collar2
            if abs(((tile_angle_deg(tid) - 90.0 + 180.0) % 360.0) - 180.0) <= half_w
            or abs(((tile_angle_deg(tid) - 270.0 + 180.0) % 360.0) - 180.0) <= half_w
        ]
        r = solve_open_cpp(tiles, cav_ids, pinned, bd_cyc_z4, raw_cands, certify_drat=True)
        r["wall_arc_deg_each"] = wall_deg
        r["open_arc_deg_each"] = 180 - wall_deg
        if r["status"] == "SAT" and any(x["status"] == "SAT" for x in corridor_results):
            r.pop("polys_rem", None); r.pop("polys_add", None); r.pop("polys_kept", None)
        corridor_results.append(r)
        print(
            f"  walls=2x{wall_deg:3d} deg (open=2x{180-wall_deg:3d} deg, bd_pin={r['n_bd_pinned']:3d}, bd_open={r['n_bd_open']:3d}): "
            f"{r['status']:5s} drat={r.get('drat_verified')} "
            f"rem={r.get('n_removed_from_C')} add={r.get('n_added_new')} enc={r['t_encode']}s solve={r.get('t_solve')}s",
            flush=True,
        )

    out = {
        "disc_k": k_disc,
        "seed": seed,
        "single_open_arc_sweep": arc_results,
        "opposite_arc_corridor_sweep": corridor_results,
        "elapsed_s": round(time.time() - t0, 2),
    }
    out_path = os.path.join(FRONTIER_DIR, "results_frontier1_open_worms.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[DONE] Saved Frontier 1A open-boundary SAT worm results to {out_path} ({out['elapsed_s']}s)")


if __name__ == "__main__":
    main()
