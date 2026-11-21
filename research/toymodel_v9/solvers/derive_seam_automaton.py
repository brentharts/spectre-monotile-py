#!/usr/bin/env python3
"""derive_seam_automaton.py - Extract the exact child-seam substitution rules
for all 13 internal slot-pair seams of O_k and G_k (k=1..5) and build the exact
closed finite-state matrix proof of quantgrav.pdf Result 36 (Open Problem #9 / Gap O2).
"""
import collections
import os
import sys
from fractions import Fraction

import numpy as np

UPGRADE_DIR = os.path.dirname(os.path.realpath(__file__))
SPECTRE_BC = os.path.join(UPGRADE_DIR, "repos", "spectre", "boundary_certificate")
sys.path.insert(0, SPECTRE_BC)
import spectre_exact as se  # noqa: E402

FLAT_IDX = 10


def _vec4(v):
    return tuple(int(x) for x in v)


def _mul_T(T1, T2):
    return (T1[0].dot(T2[0]), T1[0].dot(T2[1]) + T1[1])


def build_OG(max_level):
    """Exact 2-species build: O (all 8 non-Gamma species are identical by induction!) and G (Gamma)."""
    quad = [se.SPECTRE[3], se.SPECTRE[5], se.SPECTRE[7], se.SPECTRE[11]]
    g2 = se.mul(se.ttrans(se.SPECTRE[8]), se.trot(1))
    O_tiles = [(se.IDENT, ())]
    G_tiles = [(se.IDENT, ()), (g2, ())]
    levels = [({"O": O_tiles, "G": G_tiles}, quad, None)]
    O_SLOTS = [(i, "G" if i == 7 else "O") for i in range(8)]
    G_SLOTS = [(i, "G" if i == 7 else "O") for i in range(8) if i != 2]
    for _ in range(max_level):
        Ts, nq = se.slot_transforms(quad)
        new_O = [(_mul_T(Ts[i], T), (i,) + p) for i, sp in O_SLOTS for T, p in (G_tiles if sp == "G" else O_tiles)]
        new_G = [(_mul_T(Ts[i], T), (i,) + p) for i, sp in G_SLOTS for T, p in (G_tiles if sp == "G" else O_tiles)]
        O_tiles, G_tiles, quad = new_O, new_G, nq
        levels.append(({"O": O_tiles, "G": G_tiles}, quad, Ts))
    return levels


def slot_pair_subseams(tiles_k):
    """For a level-k supertile (k >= 2), decompose EVERY cross-top-slot flat pair AND
    every cross-top-slot shared edge by its (top_slot_A, sub_slot_A) vs (top_slot_B, sub_slot_B)!
    """
    # Map every vertex -> list of (tile_idx, v_idx, top_slot, sub_slot)
    v_at = collections.defaultdict(list)
    e_at = {}
    for i, (T, path) in enumerate(tiles_k):
        poly = [_vec4(v) for v in se.outline(T)]
        for vi, v in enumerate(poly):
            v_at[v].append((i, vi, path[0], path[1]))
            e_at[(v, poly[(vi + 1) % 14])] = (i, path[0], path[1])

    # Shared edges between different top-level slots, grouped by (top_A, top_B) -> Counter of ((sub_A, sub_B))
    edge_decomp = collections.defaultdict(collections.Counter)
    for (u, v), (ia, ta, sa) in e_at.items():
        opp = e_at.get((v, u))
        if opp is None:
            continue
        ib, tb, sb = opp
        if ta < tb:
            edge_decomp[(ta, tb)][(sa, sb)] += 1

    # Flat pairs between different top-level slots, grouped by (top_A, top_B) -> Counter of ((sub_A, sub_B))
    pair_decomp = collections.defaultdict(collections.Counter)
    for v, lst in v_at.items():
        if len(lst) == 2 and lst[0][1] == FLAT_IDX and lst[1][1] == FLAT_IDX:
            (_, _, ta, sa), (_, _, tb, sb) = lst
            if ta != tb:
                if ta < tb:
                    pair_decomp[(ta, tb)][(sa, sb)] += 1
                else:
                    pair_decomp[(tb, ta)][(sb, sa)] += 1
    return edge_decomp, pair_decomp


def main():
    levels = build_OG(5)
    print("Checking edge and flat-pair child-slot decompositions of the 13 seams of O_k across k=2..5:")
    # For each top-slot pair (i, j), see which child pairs (sa, sb) share edges at k=2,3,4,5!
    for k in (2, 3, 4, 5):
        ed, pd = slot_pair_subseams(levels[k][0]["O"])
        print(f"\n--- k={k} (O_{k}, {len(levels[k][0]['O'])} tiles) ---")
        for (i, j) in sorted(ed.keys()):
            sub_e = sorted(ed[(i, j)].keys())
            sub_p = dict(sorted(pd[(i, j)].items()))
            print(f"  seam ({i},{j}): total_edges={sum(ed[(i,j)].values()):3d}, total_pairs={sum(pd[(i,j)].values()):2d}, "
                  f"child_seams={sub_e}  pairs={sub_p}")


if __name__ == "__main__":
    main()
