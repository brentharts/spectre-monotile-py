#!/usr/bin/env python3
"""run_frontier1b_center_worm.py - Exact 1/2 Z[sqrt3] SAT + drat-trim Deep-Bulk Chiral Phason Worm!
Forces the CENTRAL SEED TILE (#2675, depth r = 12.66 inside the 60-tile Level-4 disc) to flip
(unit clause -x_seed), separating shallow boundary nibbles from TRUE macroscopic chiral phason worms
that propagate all the way from an open boundary to the deep interior!
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


def solve_center_worm(tiles, cav_ids, pinned_ids, bd_cyc_z4, raw_cands, seed_key, certify_drat=True):
    t0 = time.time()
    cav_tiles = [tiles[i] for i in cav_ids]
    pin_tiles = [tiles[i] for i in pinned_ids]
    orig_keys = {tg.key for tg in cav_tiles}

    with tempfile.TemporaryDirectory(prefix="f1b_worm_") as tmp:
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
        out_lines = proc.stdout.strip().splitlines()
        hdr = out_lines[0].split()
        M, n_clauses, n_bd_pin, n_bd_open = map(int, hdr[1:5])

        cands_tbl = []
        seed_var = None
        for idx, ln in enumerate(out_lines[1:1 + M]):
            toks = [int(x) for x in ln.split()]
            key = (toks[0], (toks[1], toks[2], toks[3], toks[4]))
            cands_tbl.append(key)
            if key == seed_key:
                seed_var = idx + 1
        assert seed_var is not None

        # Append unit clause forcing the CENTRAL SEED TILE to be flipped (-seed_var 0)!
        with open(cnf_path, "r") as f:
            txt = f.read()
        first_nl = txt.index("\n")
        txt = f"p cnf {M} {n_clauses + 1}\n" + txt[first_nl + 1:] + f"-{seed_var} 0\n"
        with open(cnf_path, "w") as f:
            f.write(txt)

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
                "status": "UNSAT", "drat_verified": drat_ok,
                "n_cands": M, "n_clauses": n_clauses + 1,
                "n_bd_pinned": n_bd_pin, "n_bd_open": n_bd_open,
                "t_solve": round(t_solve, 3),
            }
        elif sp.returncode == 10:
            chosen_tiles = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if 0 < v <= M:
                            o, sh = cands_tbl[v - 1]
                            chosen_tiles.append(scs.TileGeom(o, sh))
            chosen_keys = {tg.key for tg in chosen_tiles}
            assert seed_key not in chosen_keys
            rem_tiles = [tg for tg in cav_tiles if tg.key not in chosen_keys]
            add_tiles = [tg for tg in chosen_tiles if tg.key not in orig_keys]
            kept_tiles = [tg for tg in cav_tiles if tg.key in chosen_keys]
            # Exact zero-overlap check
            for a in range(len(chosen_tiles)):
                for b in range(a + 1, len(chosen_tiles)):
                    assert not (scs.bbox_may_overlap(chosen_tiles[a].bbox, chosen_tiles[b].bbox) and
                                scs.tiles_overlap(chosen_tiles[a], chosen_tiles[b])), "OVERLAP!"
            return {
                "status": "SAT",
                "n_cands": M, "n_clauses": n_clauses + 1,
                "n_bd_pinned": n_bd_pin, "n_bd_open": n_bd_open,
                "n_removed_from_C": len(rem_tiles),
                "n_added_new": len(add_tiles),
                "n_kept_in_C": len(kept_tiles),
                "t_solve": round(t_solve, 3),
                "polys_rem": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in rem_tiles],
                "polys_add": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in add_tiles],
                "polys_kept": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in kept_tiles],
                "polys_pin": [[[round(p[0], 4), round(p[1], 4)] for p in [scs.z4_to_f2(v) for v in t.verts_z4]] for t in pin_tiles],
            }
        raise RuntimeError(f"rc={sp.returncode}")


def main():
    t0 = time.time()
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
    bd_cyc_z4 = cycs[0]
    cav_pts_f = [scs.z4_to_f2(v) for v in bd_cyc_z4]
    cx0, cy0 = min(p[0] for p in cav_pts_f) - 2.5, min(p[1] for p in cav_pts_f) - 2.5
    cx1, cy1 = max(p[0] for p in cav_pts_f) + 2.5, max(p[1] for p in cav_pts_f) + 2.5
    raw_cands = expab.enumerate_raw_modelset_for_bbox((cx0, cy0, cx1, cy1), lat, internal_margin=1.5)

    cav_set = set(cav_ids)
    collar1 = {j for i in cav_ids for j in adj[i] if j not in cav_set}
    collar2 = collar1 | {j for i in collar1 for j in adj[i] if j not in cav_set}
    cx = sum(p[0] for p in cav_pts_f) / len(cav_pts_f)
    cy = sum(p[1] for p in cav_pts_f) / len(cav_pts_f)
    seed_key = tiles[seed].key

    def tile_angle_deg(tid):
        pts = [scs.z4_to_f2(v) for v in tiles[tid].verts_z4]
        return math.degrees(math.atan2(sum(p[1] for p in pts) / 14.0 - cy, sum(p[0] for p in pts) / 14.0 - cx)) % 360.0

    print("=== Frontier 1B: Deep-Bulk Chiral Phason Worm (FORCED CENTER-TILE #2675 FLIP) ===")
    single_arc = []
    for theta in (60, 120, 180, 210, 240, 270, 300, 330, 360):
        pinned = [tid for tid in collar2 if tile_angle_deg(tid) >= theta]
        r = solve_center_worm(tiles, cav_ids, pinned, bd_cyc_z4, raw_cands, seed_key, certify_drat=True)
        r["theta_open_deg"] = theta
        # Keep polygons for first SAT witness only
        if r["status"] == "SAT" and any(x["status"] == "SAT" for x in single_arc):
            for k in ("polys_rem", "polys_add", "polys_kept", "polys_pin"):
                r.pop(k, None)
        single_arc.append(r)
        print(
            f"  [Single Arc] theta_open={theta:3d} deg (bd_pin={r['n_bd_pinned']:3d}, bd_open={r['n_bd_open']:3d}): "
            f"{r['status']:5s} drat={r.get('drat_verified')} rem={r.get('n_removed_from_C')} add={r.get('n_added_new')} t={r['t_solve']}s",
            flush=True,
        )

    corridor = []
    for wall_deg in (130, 110, 90, 70, 50, 30):
        half_w = 0.5 * wall_deg
        pinned = [
            tid for tid in collar2
            if abs(((tile_angle_deg(tid) - 90.0 + 180.0) % 360.0) - 180.0) <= half_w
            or abs(((tile_angle_deg(tid) - 270.0 + 180.0) % 360.0) - 180.0) <= half_w
        ]
        r = solve_center_worm(tiles, cav_ids, pinned, bd_cyc_z4, raw_cands, seed_key, certify_drat=True)
        r["wall_arc_deg_each"] = wall_deg
        r["open_arc_deg_each"] = 180 - wall_deg
        if r["status"] == "SAT" and any(x["status"] == "SAT" for x in corridor):
            for k in ("polys_rem", "polys_add", "polys_kept", "polys_pin"):
                r.pop(k, None)
        corridor.append(r)
        print(
            f"  [Corridor]   walls=2x{wall_deg:3d} deg (open=2x{180-wall_deg:3d} deg, bd_pin={r['n_bd_pinned']:3d}): "
            f"{r['status']:5s} drat={r.get('drat_verified')} rem={r.get('n_removed_from_C')} add={r.get('n_added_new')} t={r['t_solve']}s",
            flush=True,
        )

    out = {
        "disc_k": k_disc,
        "seed": seed,
        "single_arc_center_flip": single_arc,
        "corridor_center_flip": corridor,
        "elapsed_s": round(time.time() - t0, 2),
    }
    out_path = os.path.join(FRONTIER_DIR, "results_frontier1b_center_worm.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[DONE] Saved {out_path} ({out['elapsed_s']}s)")


if __name__ == "__main__":
    main()
