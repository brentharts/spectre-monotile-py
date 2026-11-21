"""B2 input: grow the level-11 patch and save the tiles whose centroid is within RT of the patch centre (x4 vertex coordinates, tile type) -> dflip_patch_L11.npz.
usage: python3 t9_dflip_patch.py [LEVEL=11] [RT=22]"""
import sys; sys.path.insert(0, '.')
import numpy as np
import t9_dissect as d, t9_patch as P
BEST = {'K': 0, 'B': 1, 'C': 1, 'A': 8}
L = int(sys.argv[1]) if len(sys.argv) > 1 else 11; RT = float(sys.argv[2]) if len(sys.argv) > 2 else 22.0
tiles = P.grow(BEST, L)
V4 = np.rint(np.array([np.linalg.matrix_power(d.S, L) @ v for v in d.T4["K"]])).astype(int)
cen = np.array([d.phys4(v) for v in V4]).mean(0)
types = []; verts = []
for T, g, x in tiles:
    W4 = np.array([x + d.RX[g] @ d.T4[T][j] for j in range(4)]).astype(int)
    c = np.array([d.phys4(w) for w in W4]).mean(0)
    if np.linalg.norm(c - cen) <= RT: types.append("KBCA".index(T)); verts.append(W4)
types = np.array(types); verts = np.array(verts)
np.savez_compressed("dflip_patch_L%d.npz" % L, types=types, verts=verts, centre=cen, RT=RT, level=L)
print("tiles kept", len(types), "of", len(tiles), "; type counts", np.bincount(types), "; centre", cen.round(3))
