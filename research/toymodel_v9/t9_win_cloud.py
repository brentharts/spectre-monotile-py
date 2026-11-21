"""Save the star-frame point clouds of the three D6 vertex classes of the selected dissection at LEVEL (default 10) to win_cloud_L<LEVEL>.npz; classes are named I, II, III by window volume."""
import sys; sys.path.insert(0, '.')
import numpy as np, collections
import t9_dissect as d, t9_patch as P, t9_patch_windows as W
BEST = {'K': 0, 'B': 1, 'C': 1, 'A': 8}
L = int(sys.argv[1]) if len(sys.argv) > 1 else 10
tiles = P.grow(BEST, L); verts = set()
for T, g, x in tiles:
    for j in range(4): verts.add(tuple(int(z) for z in (x + d.RX[g] @ d.T4[T][j])))
cls = collections.defaultdict(list)
for v in verts: cls[P.class_key(v)].append(v)
vol = {k: len(v) for k, v in cls.items()}
big = sorted([(k, v) for k, v in cls.items() if len(v) > 5000 and k not in ()], key=lambda kv: -len(kv[1]))
out = {}
for k, vs in cls.items():
    st = np.array([d.star4(v) for v in vs])
    out[str(k)] = st; print(k, len(vs))
np.savez_compressed("win_cloud_L%d.npz" % L, **out)
