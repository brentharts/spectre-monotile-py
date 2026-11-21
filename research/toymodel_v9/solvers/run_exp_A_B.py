#!/usr/bin/env python3
"""run_exp_A_B.py - Runs Experiment A (Relaxed-SAT Chiral Phason-Worm Termination Cost D_min(r))
and Experiment B (Random i.i.d. Tile-Erasure Recovery Threshold p_c for the Spectre qwla code).

Uses our C++20 OpenMP exact Z[sqrt(3)] engine `bin/spectre_exp_ab` + CaDiCaL 2.1.3 + drat-trim.
"""
from __future__ import annotations

import collections
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time

import numpy as np

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, UPGRADE_DIR)
import spectre_cavity_sat as scs  # noqa: E402
import run_fast_cavity_suite as rfcs  # noqa: E402

AB_BIN = os.path.join(UPGRADE_DIR, "bin", "spectre_exp_ab")
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")


def enumerate_raw_modelset_for_bbox(bbox, lat, internal_margin=1.5):
    cx0, cy0, cx1, cy1 = bbox
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
    return raw_cands


def build_pinned_collar(tiles, erased_set, pad=6.0):
    ex0 = min(tiles[i].bbox[0] for i in erased_set)
    ey0 = min(tiles[i].bbox[1] for i in erased_set)
    ex1 = max(tiles[i].bbox[2] for i in erased_set)
    ey1 = max(tiles[i].bbox[3] for i in erased_set)
    pinned = []
    for i, tg in enumerate(tiles):
        if i in erased_set:
            continue
        if tg.bbox[2] >= ex0 - pad and tg.bbox[0] <= ex1 + pad and tg.bbox[3] >= ey0 - pad and tg.bbox[1] <= ey1 + pad:
            pinned.append(tg)
    return pinned, (ex0, ey0, ex1, ey1)


def solve_relax_step(erased_tiles, pinned_tiles, raw_cands, max_D, certify_drat=False):
    with tempfile.TemporaryDirectory(prefix="expA_relax_") as tmp:
        cnf_path = os.path.join(tmp, "relax.cnf")
        drat_path = os.path.join(tmp, "proof.drat")
        lines = [f"relax_cnf {cnf_path} {max_D} 0", f"{len(erased_tiles)} {len(pinned_tiles)} {len(raw_cands)}"]
        for tg in erased_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for tg in pinned_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for o, s in raw_cands:
            lines.append(f"{o} {s[0]} {s[1]} {s[2]} {s[3]}")
        env = dict(os.environ, OMP_NUM_THREADS="16")
        proc = subprocess.run([AB_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True, env=env)
        out_lines = proc.stdout.strip().splitlines()
        hdr = out_lines[0].split()
        assert hdr[0] == "OK"
        M, n_vars, n_clauses, n_def, n_bd = map(int, hdr[1:6])

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
            return {"status": "UNSAT", "D": max_D, "drat_verified": drat_ok, "n_cands": M, "n_clauses": n_clauses, "t_solve": round(t_solve, 4)}
        elif sp.returncode == 10:
            cands_tbl = []
            for ln in out_lines[1:1 + M]:
                toks = [int(x) for x in ln.split()]
                cands_tbl.append((toks[0], (toks[1], toks[2], toks[3], toks[4]), toks[5]))
            chosen = []
            n_new = 0
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if 0 < v <= M:
                            o, sh, is_orig = cands_tbl[v - 1]
                            chosen.append(scs.TileGeom(o, sh))
                            if is_orig == 0:
                                n_new += 1
            # Independently verify ZERO overlaps among chosen and with pinned, AND exact unmatched edge count!
            for i in range(len(chosen)):
                for j in range(i + 1, len(chosen)):
                    assert not scs.tiles_overlap(chosen[i], chosen[j]), "Overlap in relaxed SAT solution!"
            ecnt = collections.Counter()
            for tg in erased_tiles:
                for e in tg.edges_z4:
                    ecnt[e] += 1
            bd_edges = {e for e in ecnt if (e[1], e[0]) not in ecnt}
            ch_edges = set()
            for tg in chosen:
                for e in tg.edges_z4:
                    assert e not in ch_edges
                    ch_edges.add(e)
            uncov_bd = [e for e in bd_edges if e not in ch_edges]
            unmat_int = [e for e in ch_edges if (e not in bd_edges) and ((e[1], e[0]) not in ch_edges)]
            exact_D = len(uncov_bd) + len(unmat_int)
            assert exact_D <= max_D and n_new >= 1
            # Analyze connected components of defect edges (how many localized defect pockets/endpoints!)
            v_adj = collections.defaultdict(set)
            for u, v in uncov_bd + unmat_int:
                v_adj[u].add(v)
                v_adj[v].add(u)
            seen_v = set()
            comp_sizes = []
            for v in v_adj:
                if v in seen_v:
                    continue
                q = collections.deque([v])
                seen_v.add(v)
                cnt_v = 0
                while q:
                    cur = q.popleft()
                    cnt_v += 1
                    for nb in v_adj[cur]:
                        if nb not in seen_v:
                            seen_v.add(nb)
                            q.append(nb)
                comp_sizes.append(cnt_v)
            orig_keys = {t.key for t in erased_tiles}
            chosen_keys = {t.key for t in chosen}
            return {
                "status": "SAT",
                "D_bound": max_D,
                "exact_D": exact_D,
                "uncovered_bd_edges": len(uncov_bd),
                "unmatched_int_edges": len(unmat_int),
                "n_defect_components": len(comp_sizes),
                "defect_comp_verts": sorted(comp_sizes, reverse=True),
                "n_erased": len(erased_tiles),
                "n_chosen": len(chosen),
                "delta_tiles": len(chosen) - len(erased_tiles),
                "n_removed": len(orig_keys - chosen_keys),
                "n_added": len(chosen_keys - orig_keys),
                "n_cands": M,
                "t_solve": round(t_solve, 4),
            }
        raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


def solve_multi_erasure(erased_tiles, pinned_tiles, raw_cands):
    with tempfile.TemporaryDirectory(prefix="expB_multi_") as tmp:
        cnf_path = os.path.join(tmp, "multi.cnf")
        lines = [f"multi_cnf {cnf_path} 1", f"{len(erased_tiles)} {len(pinned_tiles)} {len(raw_cands)}"]
        for tg in erased_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for tg in pinned_tiles:
            s = tg.shift
            lines.append(f"{tg.orient} {s[0]} {s[1]} {s[2]} {s[3]}")
        for o, s in raw_cands:
            lines.append(f"{o} {s[0]} {s[1]} {s[2]} {s[3]}")
        env = dict(os.environ, OMP_NUM_THREADS="16")
        proc = subprocess.run([AB_BIN], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True, env=env)
        hdr = proc.stdout.strip().splitlines()[0].split()
        assert hdr[0] == "OK"
        sp = subprocess.run([CADICAL_BIN, cnf_path], capture_output=True, text=True, check=False)
        assert sp.returncode in (10, 20)
        return (sp.returncode == 20), int(hdr[1]), int(hdr[2])


def main():
    t0 = time.time()
    print("=== Building Level-4 Chiral Spectre Patch for Experiments A & B ===")
    tiles4, kinds4 = scs.build_chiral_patch(4)
    adj4 = scs.patch_adjacency(tiles4)
    lat4 = scs.extract_model_set_lattice(tiles4, kinds4)

    all_edges = collections.Counter()
    for tg in tiles4:
        for e in tg.edges_z4:
            all_edges[e] += 1
    interior_set = {i for i, tg in enumerate(tiles4) if all((e[1], e[0]) in all_edges for e in tg.edges_z4)}
    cx = sum(t.centroid[0] for t in tiles4) / len(tiles4)
    cy = sum(t.centroid[1] for t in tiles4) / len(tiles4)
    interior = sorted(interior_set, key=lambda i: (tiles4[i].centroid[0] - cx) ** 2 + (tiles4[i].centroid[1] - cy) ** 2)
    center = interior[0]

    # =========================================================================
    # EXPERIMENT A: Relaxed-SAT Minimum Defect D_min(r) for Non-Trivial Chiral Rearrangement!
    # =========================================================================
    print("\n=== [Experiment A] Exact Relaxed-SAT Chiral Phason-Worm Defect D_min(r) ===")
    exp_a = []
    for sz in (6, 10, 16, 20, 30, 40):
        cav = rfcs.fast_grow_simple_cavity(tiles4, adj4, interior_set, center, sz)
        erased_tiles = [tiles4[i] for i in cav]
        pinned, bbox = build_pinned_collar(tiles4, set(cav))
        raw_cands = enumerate_raw_modelset_for_bbox(bbox, lat4)
        sx, sy = tiles4[center].centroid
        rad = max(math.hypot(tiles4[i].centroid[0] - sx, tiles4[i].centroid[1] - sy) for i in cav)

        # Binary / stepped search for exact D_min!
        # Note: trivial 1-tile deletion is forbidden (>=1 NEW tile required!).
        # Start stepping D = 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 24, 28... then narrow exact D_min!
        lo_unsat = 0
        hi_sat = None
        best_sat = None
        for D_try in (4, 8, 12, 16, 20, 26, 32):
            r = solve_relax_step(erased_tiles, pinned, raw_cands, D_try, certify_drat=False)
            if r["status"] == "SAT":
                hi_sat = r["exact_D"]
                best_sat = r
                break
            else:
                lo_unsat = D_try
        # Exact binary search in (lo_unsat, hi_sat) to pinpoint exact D_min!
        while hi_sat is not None and lo_unsat + 1 < hi_sat:
            mid = (lo_unsat + hi_sat) // 2
            r = solve_relax_step(erased_tiles, pinned, raw_cands, mid, certify_drat=False)
            if r["status"] == "SAT":
                hi_sat = r["exact_D"]
                best_sat = r
            else:
                lo_unsat = mid
        # Certify global minimality at D_min - 1 with DRAT!
        cert_r = None
        if hi_sat is not None and hi_sat > 0:
            cert_r = solve_relax_step(erased_tiles, pinned, raw_cands, hi_sat - 1, certify_drat=True)
            assert cert_r["status"] == "UNSAT" and cert_r["drat_verified"]

        rec = {
            "k": len(cav), "radius": round(rad, 3),
            "D_min": hi_sat,
            "drat_cert_D_minus_1": cert_r["drat_verified"] if cert_r else False,
            "witness": best_sat,
        }
        exp_a.append(rec)
        if best_sat:
            print(f"  k={len(cav):2d} (r={rad:5.2f}): EXACT D_min={hi_sat:2d} [D-1 DRAT={rec['drat_cert_D_minus_1']}] "
                  f"(bd_uncov={best_sat['uncovered_bd_edges']}, int_unmat={best_sat['unmatched_int_edges']}, "
                  f"comps={best_sat['n_defect_components']} {best_sat['defect_comp_verts']}, "
                  f"rem={best_sat['n_removed']}, add={best_sat['n_added']}, dTiles={best_sat['delta_tiles']})")

    # =========================================================================
    # EXPERIMENT B: Random i.i.d. Tile-Erasure Recovery Threshold p_c (qwla code!)
    # =========================================================================
    print("\n=== [Experiment B] Random i.i.d. Tile-Erasure Recovery Rate vs. Erasure Fraction p (160-tile interior disc) ===")
    disc_ids = rfcs.fast_grow_simple_cavity(tiles4, adj4, interior_set, center, 160)
    # Precompute raw candidates ONCE for the whole 160-tile disc bounding box!
    _, disc_bbox = build_pinned_collar(tiles4, set(disc_ids))
    disc_raw_cands = enumerate_raw_modelset_for_bbox(disc_bbox, lat4)
    print(f"  Interior 160-tile disc: {len(disc_raw_cands)} Result-66 candidate placements in disc bbox")

    rng = random.Random(20261120)
    exp_b = []
    for p in (0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.65, 0.80, 1.00):
        n_trials = 1 if p == 1.00 else 15
        n_unique = 0
        max_comp_list = []
        n_erased_list = []
        t_p = time.time()
        for tr in range(n_trials):
            if p == 1.00:
                erased_ids = list(disc_ids)
            else:
                erased_ids = [i for i in disc_ids if rng.random() < p]
                if not erased_ids:
                    erased_ids = [rng.choice(disc_ids)]
            eset = set(erased_ids)
            n_erased_list.append(len(eset))
            # Largest connected component of erased tiles (percolation diagnostic!)
            seen_e = set()
            max_c = 0
            for u in eset:
                if u in seen_e:
                    continue
                q = collections.deque([u])
                seen_e.add(u)
                csz = 0
                while q:
                    cur = q.popleft()
                    csz += 1
                    for nb in adj4[cur]:
                        if nb in eset and nb not in seen_e:
                            seen_e.add(nb)
                            q.append(nb)
                max_c = max(max_c, csz)
            max_comp_list.append(max_c)

            erased_tiles = [tiles4[i] for i in eset]
            pinned, _ = build_pinned_collar(tiles4, eset)
            is_unique, nc, ncl = solve_multi_erasure(erased_tiles, pinned, disc_raw_cands)
            if is_unique:
                n_unique += 1
        rec_b = {
            "p": p,
            "n_trials": n_trials,
            "n_unique": n_unique,
            "recovery_rate": round(n_unique / n_trials, 4),
            "mean_erased": round(sum(n_erased_list) / n_trials, 1),
            "mean_largest_comp": round(sum(max_comp_list) / n_trials, 1),
            "t_s": round(time.time() - t_p, 2),
        }
        exp_b.append(rec_b)
        print(f"  p={p:4.2f}: unique={n_unique:2d}/{n_trials:2d} ({100*n_unique/n_trials:5.1f}%), "
              f"mean_erased={rec_b['mean_erased']:5.1f}, mean_max_cluster={rec_b['mean_largest_comp']:5.1f}, t={rec_b['t_s']:.2f}s")

    out = {"exp_a_worm_defect": exp_a, "exp_b_erasure_threshold": exp_b, "elapsed_s": round(time.time() - t0, 2)}
    out_path = os.path.join(UPGRADE_DIR, "results_exp_A_B.json")
    with open(out_path, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"\nSaved Experiments A & B to {out_path} ({time.time()-t0:.1f}s)")


if __name__ == "__main__":
    main()
