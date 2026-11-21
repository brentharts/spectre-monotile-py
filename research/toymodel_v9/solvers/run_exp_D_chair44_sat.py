#!/usr/bin/env python3
"""run_exp_D_chair44_sat.py - Experiment D: Exact 3D SAT + drat-trim Cavity Rigidity
and Local-Move Discovery on the 3D Aperiodic Chair44 Monotile (Tsiokos / Flicker).

Directly compares:
  1. Bare 7-cube Chair (undecorated Z^3 polycube): tests how quickly finite local
     3D swaps / closed clusters appear.
  2. Decorated Aperiodic Chair44 (exact 44-contact atlas from chair44_data/r44_atlas.json):
     tests whether 3D Chair44 admits finite closed 3D phason flip clusters (3D Pachner moves)
     or exhibits macroscopic DRAT-certified cavity rigidity like the 2D chiral Spectre!
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

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
SPECTRE_REPO = os.path.join(UPGRADE_DIR, "repos", "spectre")
CADICAL_BIN = os.path.join(UPGRADE_DIR, "bin", "cadical")
DRAT_TRIM_BIN = os.path.join(UPGRADE_DIR, "bin", "drat-trim")

sys.path.insert(0, SPECTRE_REPO)
import chair44 as c44  # noqa: E402


def build_chair44_supertile(depth: int):
    children, legal, raw_cnt = c44.load_atlas()
    assert len(legal) == 44 and raw_cnt == 2388
    poses = [c44.ROOT_POSE]
    for _ in range(depth):
        nxt = []
        for G, t in poses:
            for H, u in children:
                nxt.append((c44.frame_mul(G, H), c44.vadd(tuple(2 * x for x in t), c44.frame_apply(G, u))))
        poses = nxt
    # Verify exact partition & face-contact legality of the supertile!
    cell_owner = {}
    tile_cells = []
    for idx, p in enumerate(poses):
        cs = c44.pose_cells(p)
        tile_cells.append(cs)
        for c in cs:
            assert c not in cell_owner
            cell_owner[c] = idx
    # Adjacency across shared unit faces
    adj = {i: set() for i in range(len(poses))}
    for c, i in cell_owner.items():
        for ax in range(3):
            for sd in (-1, 1):
                nb = list(c)
                nb[ax] += sd
                j = cell_owner.get(tuple(nb))
                if j is not None and j != i:
                    adj[i].add(j)
    # Sanity check: every adjacent pair in the supertile is in the 44-contact legal atlas!
    for i, nbs in adj.items():
        for j in nbs:
            if i < j:
                rp = c44.relative_pose(poses[i], poses[j])
                assert rp in legal, f"Supertile contact not in legal atlas: {i},{j}"
    return poses, tile_cells, cell_owner, adj, legal


def solve_chair44_cavity(
    erased_ids,
    poses,
    tile_cells,
    cell_owner,
    adj,
    legal,
    decorated: bool = True,
    forbid_orig: bool = True,
    certify_drat: bool = True,
):
    t0 = time.time()
    erased_set = set(erased_ids)
    cavity_cells = set()
    for i in erased_ids:
        cavity_cells |= tile_cells[i]

    # Identify all pinned exterior tiles face-adjacent to cavity_cells
    ext_nbs = set()
    for i in erased_ids:
        for j in adj[i]:
            if j not in erased_set:
                ext_nbs.add(j)

    # Map each cavity cell -> set of adjacent pinned exterior tile indices
    cell_ext_nbs = collections.defaultdict(set)
    for c in cavity_cells:
        for ax in range(3):
            for sd in (-1, 1):
                nb = list(c)
                nb[ax] += sd
                owner = cell_owner.get(tuple(nb))
                if owner is not None and owner not in erased_set:
                    cell_ext_nbs[c].add(owner)

    # Enumerate ALL valid discrete poses (frame in 24 proper? wait: Chair44 is chiral/proper? Let's test BOTH 24 proper & all 48!)
    # Note: chair44_core.py says all 8 children have frame_det == 1, and chair44_rigidity tests mirror too;
    # in Chair44's 44-contact atlas, let's check det of legal poses!
    legal_dets = {c44.frame_det(p[0]) for p in legal}
    frames = [f for f in c44.all_frames() if (not decorated) or (c44.frame_det(f) in legal_dets)]
    # Precompute relative cell offsets for each frame f at shift (0,0,0)
    frame_base_cells = []
    for f in frames:
        cs = c44.pose_cells((f, (0, 0, 0)))
        frame_base_cells.append((f, cs))

    # Candidate poses whose 7 unit cells are ALL inside cavity_cells
    # (anchor any of the 7 base cells onto a cavity cell c -> unique shift t!)
    seen_poses = set()
    cands = []  # list of (pose, frozenset_cells)
    for f, bcs in frame_base_cells:
        b0 = next(iter(bcs))
        for c in cavity_cells:
            t = (c[0] - b0[0], c[1] - b0[1], c[2] - b0[2])
            pose = (f, t)
            if pose in seen_poses:
                continue
            seen_poses.add(pose)
            cs = frozenset((x[0] + t[0], x[1] + t[1], x[2] + t[2]) for x in bcs)
            if not cs.issubset(cavity_cells):
                continue
            # If decorated: immediately prune if face-adjacent to ANY pinned exterior tile with illegal contact!
            if decorated:
                touched_ext = set()
                for cell in cs:
                    touched_ext |= cell_ext_nbs.get(cell, set())
                bad = False
                for ex in touched_ext:
                    if c44.relative_pose(poses[ex], pose) not in legal:
                        bad = True
                        break
                if bad:
                    continue
            cands.append((pose, cs))

    # Note: for bare chair, multiple orientations of the symmetric 7-cube chair occupy the exact same 7 cells!
    # Deduplicate by cell-set ONLY when undecorated (bare chair has 6-fold symmetry of its cell set).
    if not decorated:
        by_cs = {}
        for pose, cs in cands:
            if cs not in by_cs:
                by_cs[cs] = pose
        # Make sure original poses are kept as representatives for their cell sets
        for i in erased_ids:
            by_cs[tile_cells[i]] = poses[i]
        cands = [(p, cs) for cs, p in by_cs.items()]

    pose_to_var = {p: idx + 1 for idx, (p, _) in enumerate(cands)}
    orig_vars = []
    for i in erased_ids:
        assert poses[i] in pose_to_var, f"Original chair {i} missing from candidates!"
        orig_vars.append(pose_to_var[poses[i]])

    # Build exact cover + 44-contact matching CNF!
    cell_vars = collections.defaultdict(list)
    for idx, (_, cs) in enumerate(cands):
        vid = idx + 1
        for c in cs:
            cell_vars[c].append(vid)

    clauses = []
    # 1. Exact cover of every unit cell c in cavity_cells: sum_{p ni c} x_p == 1
    for c in cavity_cells:
        vs = cell_vars.get(c, [])
        assert len(vs) >= 1
        clauses.append(vs)
        for a in range(len(vs)):
            for b in range(a + 1, len(vs)):
                clauses.append([-vs[a], -vs[b]])

    # 2. If decorated: pairwise face-adjacent interior candidates must satisfy 44-contact atlas!
    n_match_clauses = 0
    if decorated:
        # Find all pairs of non-overlapping candidates that share a unit face
        face_pairs = set()
        for idx, (_, cs) in enumerate(cands):
            v1 = idx + 1
            for c in cs:
                for ax in range(3):
                    for sd in (-1, 1):
                        nb = (c[0] + (sd if ax == 0 else 0),
                              c[1] + (sd if ax == 1 else 0),
                              c[2] + (sd if ax == 2 else 0))
                        if nb in cavity_cells and nb not in cs:
                            for v2 in cell_vars[nb]:
                                if v1 < v2:
                                    face_pairs.add((v1, v2))
        for v1, v2 in face_pairs:
            p1, cs1 = cands[v1 - 1]
            p2, cs2 = cands[v2 - 1]
            if not cs1.isdisjoint(cs2):
                continue  # already excluded by cell exact-cover
            if c44.relative_pose(p1, p2) not in legal:
                clauses.append([-v1, -v2])
                n_match_clauses += 1

    # 3. Blocking clause forbidding original interior configuration
    if forbid_orig:
        clauses.append([-v for v in orig_vars])

    t_enc = time.time() - t0
    with tempfile.TemporaryDirectory(prefix="chair44_sat_") as tmp:
        cnf_path = os.path.join(tmp, "chair.cnf")
        drat_path = os.path.join(tmp, "proof.drat")
        with open(cnf_path, "w") as fh:
            fh.write(f"p cnf {len(cands)} {len(clauses)}\n")
            for cl in clauses:
                fh.write(" ".join(map(str, cl)) + " 0\n")
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
                assert drat_ok, f"drat-trim failed on Chair44! {chk.stdout}"
            return {
                "status": "UNSAT",
                "drat_verified": drat_ok,
                "decorated": decorated,
                "n_erased_chairs": len(erased_ids),
                "n_erased_cubes": len(cavity_cells),
                "n_candidates": len(cands),
                "n_clauses": len(clauses),
                "n_match_clauses": n_match_clauses,
                "t_encode": round(t_enc, 4),
                "t_solve": round(t_solve, 4),
                "t_drat": round(t_drat, 4),
            }
        elif sp.returncode == 10:
            chosen = []
            for ln in sp.stdout.splitlines():
                if ln.startswith("v "):
                    for tok in ln.split()[1:]:
                        v = int(tok)
                        if v > 0:
                            chosen.append(cands[v - 1])
            # Independently verify exact 3D partition and (if decorated) 100% 44-contact legality!
            cov = set()
            for p, cs in chosen:
                assert cov.isdisjoint(cs)
                cov |= cs
            assert cov == cavity_cells
            if decorated:
                # Check all interior-interior and interior-exterior contacts!
                chosen_owner = {}
                for ci, (p, cs) in enumerate(chosen):
                    for c in cs:
                        chosen_owner[c] = ci
                for ci, (p, cs) in enumerate(chosen):
                    for c in cs:
                        for ax in range(3):
                            for sd in (-1, 1):
                                nb = (c[0] + (sd if ax == 0 else 0),
                                      c[1] + (sd if ax == 1 else 0),
                                      c[2] + (sd if ax == 2 else 0))
                                if nb in chosen_owner and chosen_owner[nb] != ci:
                                    assert c44.relative_pose(p, chosen[chosen_owner[nb]][0]) in legal
                                elif nb in cell_owner and cell_owner[nb] not in erased_set:
                                    assert c44.relative_pose(poses[cell_owner[nb]], p) in legal
            orig_poses = {poses[i] for i in erased_ids}
            chosen_poses = {p for p, _ in chosen}
            return {
                "status": "SAT",
                "decorated": decorated,
                "n_erased_chairs": len(erased_ids),
                "n_erased_cubes": len(cavity_cells),
                "n_removed": len(orig_poses - chosen_poses),
                "n_added": len(chosen_poses - orig_poses),
                "same_as_orig": (chosen_poses == orig_poses),
                "n_candidates": len(cands),
                "n_clauses": len(clauses),
                "t_encode": round(t_enc, 4),
                "t_solve": round(t_solve, 4),
            }
        raise RuntimeError(f"CaDiCaL rc={sp.returncode}")


def main():
    t0 = time.time()
    print("=== Experiment D: Exact 3D SAT + DRAT Cavity Rigidity on Chair44 (Depth 3 = 512 chairs, 3,584 cubes) ===")
    poses, tile_cells, cell_owner, adj, legal = build_chair44_supertile(3)
    # Identify strictly interior chairs (all 24 exposed unit faces touch another chair!)
    interior = []
    centroids = []
    for i, cs in enumerate(tile_cells):
        cx = sum(c[0] for c in cs) / 7.0
        cy = sum(c[1] for c in cs) / 7.0
        cz = sum(c[2] for c in cs) / 7.0
        centroids.append((cx, cy, cz))
        is_int = True
        for c in cs:
            for ax in range(3):
                for sd in (-1, 1):
                    nb = (c[0] + (sd if ax == 0 else 0),
                          c[1] + (sd if ax == 1 else 0),
                          c[2] + (sd if ax == 2 else 0))
                    if nb not in cell_owner:
                        is_int = False
                        break
                if not is_int:
                    break
            if not is_int:
                break
        if is_int:
            interior.append(i)

    # Pick central-most interior chair (center of a full octant, as in Result 97)
    mx = sum(centroids[i][0] for i in interior) / len(interior)
    my = sum(centroids[i][1] for i in interior) / len(interior)
    mz = sum(centroids[i][2] for i in interior) / len(interior)
    interior.sort(key=lambda i: (centroids[i][0] - mx) ** 2 + (centroids[i][1] - my) ** 2 + (centroids[i][2] - mz) ** 2)
    seed = interior[0]
    print(f"  Depth-3 Chair44: {len(poses)} chairs, {len(interior)} strictly interior chairs; seed #{seed}")

    def grow_3d_ball(seed_id, k):
        sx, sy, sz = centroids[seed_id]
        cav = [seed_id]
        cav_set = {seed_id}
        int_set = set(interior)
        while len(cav) < k:
            fr = set()
            for u in cav:
                for nb in adj[u]:
                    if nb not in cav_set and nb in int_set:
                        fr.add(nb)
            if not fr:
                break
            nxt = min(fr, key=lambda i: (centroids[i][0] - sx) ** 2 + (centroids[i][1] - sy) ** 2 + (centroids[i][2] - sz) ** 2)
            cav.append(nxt)
            cav_set.add(nxt)
        return cav

    # Self-test: unblocked recovery on k=8 decorated Chair44
    c8 = grow_3d_ball(seed, 8)
    r_rec = solve_chair44_cavity(c8, poses, tile_cells, cell_owner, adj, legal, decorated=True, forbid_orig=False, certify_drat=False)
    assert r_rec["status"] == "SAT" and r_rec["same_as_orig"], r_rec
    print(f"  [PASS] Unblocked 8-chair 3D recovery: {r_rec['n_candidates']} cands, {r_rec['t_solve']}s")

    res = {"depth": 3, "n_chairs": len(poses), "n_interior": len(interior), "bare_control": [], "decorated_chair44": []}

    print("\n--- [Exp D1] Bare 7-Cube 3D Chair Control (Undecorated) ---")
    for k in (1, 2, 4, 6, 8, 12, 16, 20):
        cav = grow_3d_ball(seed, k)
        r = solve_chair44_cavity(cav, poses, tile_cells, cell_owner, adj, legal, decorated=False, forbid_orig=True, certify_drat=False)
        res["bare_control"].append(r)
        print(f"  k={len(cav):2d} chairs ({r['n_erased_cubes']:3d} cubes): {r['status']:5s} cands={r['n_candidates']:5d} "
              + (f"rem={r['n_removed']} add={r['n_added']} t={r['t_solve']:.4f}s" if r["status"] == "SAT" else f"t={r['t_solve']:.4f}s"))
        if r["status"] == "SAT":
            break

    print("\n--- [Exp D2] Decorated Aperiodic Chair44 (Exact 44-Contact Atlas + DRAT Proofs) ---")
    for k in (1, 2, 4, 8, 16, 24, 32, 48, 64, 96, 128, 160):
        cav = grow_3d_ball(seed, k)
        if len(cav) < k:
            break
        sx, sy, sz = centroids[seed]
        rad = max(math.sqrt((centroids[i][0] - sx) ** 2 + (centroids[i][1] - sy) ** 2 + (centroids[i][2] - sz) ** 2) for i in cav)
        r = solve_chair44_cavity(cav, poses, tile_cells, cell_owner, adj, legal, decorated=True, forbid_orig=True, certify_drat=True)
        r["radius_3d"] = round(rad, 3)
        res["decorated_chair44"].append(r)
        if r["status"] == "UNSAT":
            print(f"  k={len(cav):3d} chairs ({r['n_erased_cubes']:4d} cubes, r={rad:5.2f}): UNSAT [DRAT={r['drat_verified']}] "
                  f"cands={r['n_candidates']:5d} clauses={r['n_clauses']:7d} enc={r['t_encode']:.2f}s solve={r['t_solve']:.4f}s drat={r['t_drat']:.4f}s")
        else:
            print(f"  k={len(cav):3d} chairs ({r['n_erased_cubes']:4d} cubes, r={rad:5.2f}): *** 3D CLOSED FLIP FOUND! *** "
                  f"rem={r['n_removed']} add={r['n_added']} cands={r['n_candidates']} solve={r['t_solve']:.4f}s")
            break

    out_path = os.path.join(UPGRADE_DIR, "results_exp_D_chair44.json")
    with open(out_path, "w") as fh:
        json.dump(res, fh, indent=2)
    print(f"\nSaved Experiment D results to {out_path} ({time.time()-t0:.2f}s)")


if __name__ == "__main__":
    main()
