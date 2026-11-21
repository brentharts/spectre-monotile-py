"""Stage B1c: grow a patch of the ABCK tiling by repeated substitution (a dissection choice from t9_dissect_faces), and measure
vertex densities by coset class and tile counts. Usage: python3 t9_patch.py [levels]"""
import sys, time, json, collections
sys.path.insert(0, '.')
import numpy as np
import t9_dissect as d
import t9_dissect_faces as F
RXk = [tuple(map(tuple, m)) for m in d.RX]; IDX = {m: i for i, m in enumerate(RXk)}
MUL = np.zeros((120, 120), int)
for a in range(120):
    for b in range(120): MUL[a, b] = IDX[tuple(map(tuple, d.RX[a] @ d.RX[b]))]
S4i = d.S
def children(choice):
    ch = {k: [] for k in "KBCA"}
    for T in "KBCA":
        for t, g0, p0 in F.raw[T][choice[T]][1]: ch[T].append((t, g0, np.array(p0)))
    return ch
def grow(choice, levels, start="K"):
    ch = children(choice); tiles = [(start, 0, np.zeros(6, int))]    # (type, group index of the orientation, x4 of the origin)
    for lv in range(levels):
        new = []
        for T, g, x in tiles:
            Sx = S4i @ x; assert np.allclose(Sx, np.rint(Sx), atol=1e-9)
            Sx = np.rint(Sx).astype(int); Rg = d.RX[g]
            for t, g0, p0 in ch[T]: new.append((t, MUL[g, g0], Sx + Rg @ p0))
        tiles = new
    return tiles
def class_key(x4):
    r = np.array(x4) % 4; m = tuple(int(z) for z in sorted(np.minimum(r, 4 - r)))
    s = int(np.sum(x4)); return (m, (s // 4) % 2 if s % 4 == 0 else None)
def analyse(choice, levels):
    t0 = time.time(); tiles = grow(choice, levels)
    cnt = collections.Counter(t for t, g, x in tiles)
    # outer tetrahedron: levels-fold inflation of the start tile
    V4 = np.array([np.linalg.matrix_power(S4i, levels) @ v for v in d.T4["K"]]); V4 = np.rint(V4).astype(int)
    V = np.array([d.phys4(v) for v in V4]); cen = V.mean(0)
    # inscribed ball: min distance from centroid to the four planes
    dist = []
    for i in range(4):
        f = [V[j] for j in range(4) if j != i]; n = np.cross(f[1] - f[0], f[2] - f[0]); n /= np.linalg.norm(n); dist.append(abs(n @ (cen - f[0])))
    rad = 0.7 * min(dist); vol_ball = 4 / 3 * np.pi * rad ** 3
    verts = {}
    for T, g, x in tiles:
        for j in range(4):
            xv = tuple(int(z) for z in (x + d.RX[g] @ d.T4[T][j]))
            if xv not in verts: verts[xv] = None
    inb = [v for v in verts if np.linalg.norm(d.phys4(v) - cen) <= rad]
    kc = collections.Counter(class_key(v) for v in inb)
    return dict(tiles=len(tiles), counts=dict(cnt), rad=float(rad), nverts_ball=len(inb), dens={str(k): c / vol_ball for k, c in kc.items()}, total=len(inb) / vol_ball, secs=time.time() - t0)
if __name__ == "__main__":
    L = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    fr = {"K": 0.5189, "B": 0.2829, "C": 0.1604, "A": 0.0379}
    NC = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    for nf, ch in F.FAILS[:NC]:
        r = analyse(ch, L)
        tt = sum(r["counts"].values())
        print(nf, ch, "tiles", r["tiles"], {k: round(v / tt, 4) for k, v in r["counts"].items()}, "ball radius %.2f vertices %d total density %.4f" % (r["rad"], r["nverts_ball"], r["total"]), "%.0fs" % r["secs"])
        for k, v in sorted(r["dens"].items(), key=lambda x: -x[1]): print("    ", k, round(v, 4))
