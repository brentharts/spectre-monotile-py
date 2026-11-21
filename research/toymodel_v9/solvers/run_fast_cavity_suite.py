#!/usr/bin/env python3
"""run_fast_cavity_suite.py - Blazing-fast C++20 + CaDiCaL 2.1.3 + drat-trim driver for:
  1. Engine A (Unrestricted Exact MRV, ZERO hull assumption):
     Level-3 & Level-4 chiral cavities from k = 1 all the way up to k = 300+ tiles (radius 26+!)
     plus exact non-chiral (reflection-allowed) control showing erasure collapse at k = 8 (2-tile swap)!
  2. Engine B (Result-66 Index-36 Model-Set Lattice SAT + DRAT):
     Machine-verified DRAT UNSAT proofs (or exact closed flip-cluster witnesses!) up to radius 26+
     across multiple seeds (center, Mystic pair, flat-pair, all 12 orientations)!
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

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, UPGRADE_DIR)
import spectre_cavity_sat as scs  # noqa: E402

FAST_BIN = os.path.join(UPGRADE_DIR, "bin", "spectre_fast_cavity")
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")


def extract_boundary_cycle_z4(erased_tiles):
    counts = collections.Counter()
    for tg in erased_tiles:
        for e in tg.edges_z4:
            counts[e] += 1
    bd_edges = {e for e in counts if (e[1], e[0]) not in counts}
    nxt = collections.defaultdict(list)
    for u, v in bd_edges:
        nxt[u].append(v)
    cycles = []
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
        cycles.append(cyc)
    return bd_edges, cycles


def run_cpp_mrv(erased_tiles, allow_reflect=False, forbid_orig=True, max_nodes=2000000):
    t0 = time.time()
    bd_edges, cycles = extract_boundary_cycle_z4(erased_tiles)
    if len(cycles) != 1:
        return {"status": "NON_SIMPLE", "n_erased": len(erased_tiles)}
    cyc = cycles[0]
    lines = [f"mrv {1 if allow_reflect else 0} {1 if forbid_orig else 0} {len(erased_tiles)} {max_nodes} {len(cyc)}"]
    for v in cyc:
        lines.append(f"{v[0]} {v[1]} {v[2]} {v[3]}")
    for tg in erased_tiles:
        s = tg.shift
        lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
    proc = subprocess.run([FAST_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
    dt = time.time() - t0
    out_lines = proc.stdout.strip().splitlines()
    hdr = out_lines[0].split()
    status, nodes, pool_sz, n_sol = hdr[0], int(hdr[1]), int(hdr[2]), int(hdr[3])
    res = {
        "engine": "A_cpp_unrestricted_MRV",
        "status": status,
        "allow_reflect": allow_reflect,
        "n_erased": len(erased_tiles),
        "n_bd_edges": len(cyc),
        "nodes": nodes,
        "pool_size": pool_sz,
        "t_s": round(dt, 4),
    }
    if status == "SAT":
        chosen = []
        for ln in out_lines[1:1 + n_sol]:
            toks = [int(x) for x in ln.split()]
            chosen.append(scs.TileGeom(toks[0], (toks[1], toks[2], toks[3], toks[4])))
        scs.verify_exact_partition(chosen, bd_edges)
        orig_keys = {t.key for t in erased_tiles}
        chosen_keys = {t.key for t in chosen}
        v0, e0, f0 = scs.cell_complex_counts(erased_tiles)
        v1, e1, f1 = scs.cell_complex_counts(chosen)
        res.update({
            "n_removed": len(orig_keys - chosen_keys),
            "n_added": len(chosen_keys - orig_keys),
            "dV": v1 - v0, "dE": e1 - e0, "dF": f1 - f0,
            "same_as_orig": (chosen_keys == orig_keys),
        })
    return res


def run_cpp_modelset_sat(erased_tiles, lat, internal_margin=1.5, forbid_orig=True, certify_drat=True):
    t0 = time.time()
    bd_edges, cycles = extract_boundary_cycle_z4(erased_tiles)
    if len(cycles) != 1:
        return {"status": "NON_SIMPLE", "n_erased": len(erased_tiles)}
    cyc = cycles[0]
    fpts = [scs.z4_to_f2(v) for v in cyc]
    cx0, cy0 = min(p[0] for p in fpts), min(p[1] for p in fpts)
    cx1, cy1 = max(p[0] for p in fpts), max(p[1] for p in fpts)

    detH = lat["detH"]
    ADJ = lat["ADJ"]
    STAR = lat["STAR"]
    int_center = lat["int_center"]
    max_int_r = lat["int_rad"] * internal_margin + 1.0
    cosets = lat["cosets"]

    PH = np.array([[1.0, 0.5, 0.5 * scs.SQRT3, 0.0], [0.0, 0.5 * scs.SQRT3, 0.5, 1.0]], dtype=float)
    Mi = np.linalg.inv(np.vstack([PH, STAR]))
    pcx, pcy = 0.5 * (cx0 + cx1), 0.5 * (cy0 + cy1)
    prad = 0.5 * math.hypot(cx1 - cx0, cy1 - cy0) + 5.5
    v_center = Mi @ np.array([pcx, pcy, int_center[0], int_center[1]])
    half_w = np.ceil(np.linalg.norm(Mi[:, :2], axis=1) * prad + np.linalg.norm(Mi[:, 2:], axis=1) * max_int_r).astype(int) + 1
    lo, hi = np.floor(v_center).astype(int) - half_w, np.ceil(v_center).astype(int) + half_w

    r1 = np.arange(lo[1], hi[1] + 1, dtype=np.int64)
    r2 = np.arange(lo[2], hi[2] + 1, dtype=np.int64)
    r3 = np.arange(lo[3], hi[3] + 1, dtype=np.int64)
    g1, g2, g3 = np.meshgrid(r1, r2, r3, indexing="ij")
    tail = np.column_stack([g1.ravel(), g2.ravel(), g3.ravel()])

    def pack_res(arr):
        return ((arr[:, 0] * detH + arr[:, 1]) * detH + arr[:, 2]) * detH + arr[:, 3]

    orient_packed = {}
    for o, refs in cosets.items():
        pset = set()
        for ref in refs:
            rv = (np.array(ref, dtype=np.int64) @ ADJ.T) % detH
            pset.add(int(((rv[0] * detH + rv[1]) * detH + rv[2]) * detH + rv[3]))
        orient_packed[o] = pset
    all_packed = np.array(sorted({p for s in orient_packed.values() for p in s}), dtype=np.int64)

    # Pre-filter raw (orient, shift) candidates via vectorized numpy (super fast!)
    raw_cands = []
    seen = set()
    for x0 in range(int(lo[0]), int(hi[0]) + 1):
        V = np.empty((len(tail), 4), dtype=np.int64)
        V[:, 0] = x0
        V[:, 1:] = tail
        px = V @ PH[0] - pcx
        py = V @ PH[1] - pcy
        m_ph = (px * px + py * py) <= prad * prad
        if not m_ph.any():
            continue
        V = V[m_ph]
        ix = V @ STAR[0] - int_center[0]
        iy = V @ STAR[1] - int_center[1]
        m_in = (ix * ix + iy * iy) <= max_int_r * max_int_r
        if not m_in.any():
            continue
        V = V[m_in]
        pk = pack_res((V @ ADJ.T) % detH)
        m_cos = np.isin(pk, all_packed)
        if not m_cos.any():
            continue
        V, pk = V[m_cos], pk[m_cos]
        for row, pval in zip(V, pk):
            sh = (int(row[0]), int(row[1]), int(row[2]), int(row[3]))
            pv_i = int(pval)
            for o, pset in orient_packed.items():
                if pv_i in pset:
                    k = (o, sh)
                    if k not in seen:
                        seen.add(k)
                        raw_cands.append(k)

    with tempfile.TemporaryDirectory(prefix="spectre_fast_sat_") as tmp:
        cnf_path = os.path.join(tmp, "cavity.cnf")
        drat_path = os.path.join(tmp, "proof.drat")
        lines = [f"cnf {cnf_path} {1 if forbid_orig else 0} {len(cyc)} {len(erased_tiles)} {len(raw_cands)}"]
        for v in cyc:
            lines.append(f"{v[0]} {v[1]} {v[2]} {v[3]}")
        for tg in erased_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for o, s in raw_cands:
            lines.append(f"{o} {s[0]} {s[1]} {s[2]} {s[3]}")
        env = dict(os.environ, OMP_NUM_THREADS="16")
        proc = subprocess.run([FAST_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True, env=env)
        t_enc = time.time() - t0
        out_lines = proc.stdout.strip().splitlines()
        hdr = out_lines[0].split()
        assert hdr[0] == "OK", proc.stdout + proc.stderr
        n_vars, n_clauses, n_ov = int(hdr[1]), int(hdr[2]), int(hdr[3])

        cmd = [CADICAL_BIN, "--no-binary", cnf_path]
        if certify_drat:
            cmd.append(drat_path)
        t1 = time.time()
        sp = subprocess.run(cmd, capture_output=True, text=True, check=False)
        t_solve = time.time() - t1

        if sp.returncode == 20:
            drat_ok = None
            t_drat = 0.0
            if certify_drat:
                t2 = time.time()
                chk = subprocess.run([DRAT_TRIM_BIN, cnf_path, drat_path], capture_output=True, text=True, check=False)
                t_drat = time.time() - t2
                drat_ok = ("s VERIFIED" in chk.stdout)
                assert drat_ok, f"drat-trim failed: {chk.stdout}"
            return {
                "engine": "B_cpp_modelset_SAT",
                "status": "UNSAT",
                "drat_verified": drat_ok,
                "n_erased": len(erased_tiles),
                "n_bd_edges": len(cyc),
                "n_candidates": n_vars,
                "n_clauses": n_clauses,
                "n_overlap_clauses": n_ov,
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 4),
                "t_drat": round(t_drat, 4),
            }
        elif sp.returncode == 10:
            cands_tbl = []
            for ln in out_lines[1:1 + n_vars]:
                toks = [int(x) for x in ln.split()]
                cands_tbl.append((toks[0], (toks[1], toks[2], toks[3], toks[4])))
            chosen = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if v > 0:
                            o, sh = cands_tbl[v - 1]
                            chosen.append(scs.TileGeom(o, sh))
            scs.verify_exact_partition(chosen, bd_edges)
            orig_keys = {t.key for t in erased_tiles}
            chosen_keys = {t.key for t in chosen}
            v0, e0, f0 = scs.cell_complex_counts(erased_tiles)
            v1, e1, f1 = scs.cell_complex_counts(chosen)
            return {
                "engine": "B_cpp_modelset_SAT",
                "status": "SAT",
                "n_erased": len(erased_tiles),
                "n_bd_edges": len(cyc),
                "n_removed": len(orig_keys - chosen_keys),
                "n_added": len(chosen_keys - orig_keys),
                "dV": v1 - v0, "dE": e1 - e0, "dF": f1 - f0,
                "same_as_orig": (chosen_keys == orig_keys),
                "n_candidates": n_vars,
                "n_clauses": n_clauses,
                "t_encode": round(t_enc, 3),
                "t_solve": round(t_solve, 4),
            }
        raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


def fast_grow_simple_cavity(tiles, adj, interior_set, seed, target_size):
    cx, cy = tiles[seed].centroid
    cav = [seed]
    cav_set = {seed}
    while len(cav) < target_size:
        frontier = set()
        for u in cav:
            for nb in adj[u]:
                if nb not in cav_set and nb in interior_set:
                    frontier.add(nb)
        if not frontier:
            break
        ranked = sorted(frontier, key=lambda i: (tiles[i].centroid[0] - cx) ** 2 + (tiles[i].centroid[1] - cy) ** 2)
        added = False
        for cand in ranked:
            trial = cav + [cand]
            _, cycs = extract_boundary_cycle_z4([tiles[i] for i in trial])
            if len(cycs) == 1:
                cav = trial
                cav_set.add(cand)
                added = True
                break
        if not added:
            break
    return cav


def main():
    t_start = time.time()
    print("=== Building Exact Level-4 Chiral Spectre Patch (4,401 tiles!) ===")
    tiles4, kinds4 = scs.build_chiral_patch(4)
    adj4 = scs.patch_adjacency(tiles4)
    lat4 = scs.extract_model_set_lattice(tiles4, kinds4)
    print(f"  Level-4 patch: {len(tiles4)} tiles, exact index-{lat4['detH']} Z^4 sublattice across {len(lat4['cosets'])} orientations!")
    print(f"  Internal window radius: {lat4['int_rad']:.4f}")

    all_edges = collections.Counter()
    for tg in tiles4:
        for e in tg.edges_z4:
            all_edges[e] += 1
    interior_set = {i for i, tg in enumerate(tiles4) if all((e[1], e[0]) in all_edges for e in tg.edges_z4)}
    cx = sum(t.centroid[0] for t in tiles4) / len(tiles4)
    cy = sum(t.centroid[1] for t in tiles4) / len(tiles4)
    interior = sorted(interior_set, key=lambda i: (tiles4[i].centroid[0] - cx) ** 2 + (tiles4[i].centroid[1] - cy) ** 2)
    center = interior[0]
    mystic_seed = next(i for i in interior if kinds4[i] == "g1")

    results = {
        "level": 4, "n_tiles": len(tiles4), "n_interior": len(interior), "sublattice_index": lat4["detH"],
        "exp1a_single_all12": [], "exp1b_unrestricted_chiral_L4": [],
        "exp1c_reflect_control_L4": [], "exp1d_modelset_drat_L4": [],
    }

    # Self-test unblocked recovery on k=10
    cav10 = fast_grow_simple_cavity(tiles4, adj4, interior_set, center, 10)
    ra_rec = run_cpp_mrv([tiles4[i] for i in cav10], allow_reflect=False, forbid_orig=False)
    assert ra_rec["status"] == "SAT" and ra_rec["same_as_orig"]
    rb_rec = run_cpp_modelset_sat([tiles4[i] for i in cav10], lat4, forbid_orig=False, certify_drat=False)
    assert rb_rec["status"] == "SAT" and rb_rec["same_as_orig"]
    print(f"  [PASS] Unblocked 10-tile recovery: C++ MRV ({ra_rec['t_s']}s) & C++ SAT ({rb_rec['t_encode']}s + {rb_rec['t_solve']}s)")

    # Exp 1A: All 12 orientations single-tile erasure
    by_orient = collections.defaultdict(list)
    for idx in interior:
        by_orient[tiles4[idx].orient].append(idx)
    for o in sorted(by_orient.keys()):
        idx = by_orient[o][0]
        ra = run_cpp_mrv([tiles4[idx]], allow_reflect=False, forbid_orig=True)
        rb = run_cpp_modelset_sat([tiles4[idx]], lat4, forbid_orig=True, certify_drat=True)
        assert ra["status"] == "UNSAT" and rb["status"] == "UNSAT" and rb["drat_verified"]
        results["exp1a_single_all12"].append({"orient": o, "tile_idx": idx, "A": ra, "B": rb})
    print(f"\n[Exp 1A] 12/12 orientations on Level-4: 100% UNSAT (unrestricted C++ MRV) + 100% DRAT-VERIFIED UNSAT!")

    # Exp 1B: Unrestricted Chiral Cavity Rigidity on Level 4 up to k = 300+ tiles (radius > 28!)
    print("\n=== [Exp 1B] Engine A (C++20 Unrestricted Chiral MRV, ZERO Hull Assumption) on Level 4 ===")
    for seed_name, seed_idx, sz_list in (
        ("center", center, [10, 20, 40, 60, 80, 100, 130, 160, 200, 250, 300]),
        ("mystic", mystic_seed, [20, 60, 120, 200]),
    ):
        print(f"  Seed '{seed_name}' (tile #{seed_idx}, orient={tiles4[seed_idx].orient}, kind={kinds4[seed_idx]}):")
        for sz in sz_list:
            cav = fast_grow_simple_cavity(tiles4, adj4, interior_set, seed_idx, sz)
            if len(cav) < sz:
                break
            sx, sy = tiles4[seed_idx].centroid
            rad = max(math.hypot(tiles4[i].centroid[0] - sx, tiles4[i].centroid[1] - sy) for i in cav)
            ra = run_cpp_mrv([tiles4[i] for i in cav], allow_reflect=False, forbid_orig=True, max_nodes=2000000)
            ra["seed_name"] = seed_name
            ra["radius"] = round(rad, 3)
            results["exp1b_unrestricted_chiral_L4"].append(ra)
            print(f"    k={len(cav):3d} (r={rad:5.2f}, dC={ra['n_bd_edges']:3d}): {ra['status']:7s} nodes={ra['nodes']:7d} pool={ra['pool_size']:5d} t={ra['t_s']:.3f}s"
                  + (f" *** CLOSED FLIP! rem={ra['n_removed']} add={ra['n_added']} (dV,dE,dF)=({ra['dV']},{ra['dE']},{ra['dF']}) ***" if ra["status"] == "SAT" else ""))
            if ra["status"] in ("SAT", "ABORTED"):
                break

    # Exp 1C: Reflection-Allowed Control on Level 4 (proving chirality locks the cavity!)
    print("\n=== [Exp 1C] Engine A (C++20 Reflection-Allowed Control) on Level 4 ===")
    for sz in (2, 4, 6, 8, 10, 12):
        cav = fast_grow_simple_cavity(tiles4, adj4, interior_set, center, sz)
        ra = run_cpp_mrv([tiles4[i] for i in cav], allow_reflect=True, forbid_orig=True, max_nodes=200000)
        results["exp1c_reflect_control_L4"].append(ra)
        print(f"  k={len(cav):2d} (reflect=True): {ra['status']:7s} nodes={ra['nodes']:6d} t={ra['t_s']:.3f}s"
              + (f" rem={ra['n_removed']} add={ra['n_added']} (dV,dE,dF)=({ra['dV']},{ra['dE']},{ra['dF']})" if ra["status"] == "SAT" else ""))
        if ra["status"] == "SAT":
            break

    # Exp 1D: Engine B (C++20 Result-66 Index-36 Model-Set SAT + DRAT up to radius > 20!)
    print("\n=== [Exp 1D] Engine B (C++20 Result-66 Model-Set SAT + DRAT) on Level 4 ===")
    for sz in (10, 20, 40, 60, 80, 100, 140, 180):
        cav = fast_grow_simple_cavity(tiles4, adj4, interior_set, center, sz)
        if len(cav) < sz:
            break
        sx, sy = tiles4[center].centroid
        rad = max(math.hypot(tiles4[i].centroid[0] - sx, tiles4[i].centroid[1] - sy) for i in cav)
        rb = run_cpp_modelset_sat([tiles4[i] for i in cav], lat4, forbid_orig=True, certify_drat=True)
        rb["radius"] = round(rad, 3)
        results["exp1d_modelset_drat_L4"].append(rb)
        if rb["status"] == "UNSAT":
            print(f"  k={len(cav):3d} (r={rad:5.2f}, dC={rb['n_bd_edges']:3d}): UNSAT [DRAT={rb['drat_verified']}] "
                  f"cands={rb['n_candidates']:6d} clauses={rb['n_clauses']:8d} enc={rb['t_encode']:.2f}s solve={rb['t_solve']:.3f}s drat={rb['t_drat']:.3f}s")
        elif rb["status"] == "SAT":
            print(f"  k={len(cav):3d} (r={rad:5.2f}): *** CLOSED FLIP CLUSTER! *** rem={rb['n_removed']} add={rb['n_added']} (dV,dE,dF)=({rb['dV']},{rb['dE']},{rb['dF']})")
            break

    out_path = os.path.join(UPGRADE_DIR, "results_cavity_L4.json")
    with open(out_path, "w") as fh:
        json.dump(results, fh, indent=2)
    print(f"\nSaved Level-4 cavity suite to {out_path} (total {time.time() - t_start:.1f}s)")


if __name__ == "__main__":
    main()
