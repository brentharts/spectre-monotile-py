"""Covering of internal space by the windows of the tiles that overlap a given Spectre tile, and the Wada-type test across window boundaries.

Setting.  A tile of window class a (shape S_a, frame A_a) placed at lattice vector u is present in a tiling iff the internal point y = STAR u lies in
its window Omega_a = c_a + sc R_a Z_0(S_a) (Z_0 from lakes_of_wada.spectre_window; the affine fit of the classes against a level-N patch is that of
brentharts/spectre boundary_certificate/window_boundary_closure.py).  A tile (b, u + w) overlapping the placed tile (a, u) is present iff
y in Omega_b - STAR w, i.e. in the frame z of Omega_a:   z_b = R_b^T R_a z + R_b^T (c_a - c_b + STAR w)/sc  must lie in Z_0(S_b).
Lattice constraint (the point that the first attempt, spectre_window_neighbours.py, missed): the tile vectors of one class lie in a coset of a sublattice
of Z^4 of index 36 (Smith form 1,1,6,6), so w must be congruent to (v_b - u_a) modulo that sublattice, v_b and u_a being tile vectors of the patch.
Outputs: pixel statistics in the z frame of Z_0(S_a): inside Z_0 the number of overlapping tiles present must be 0 (a tiling has no overlaps); outside,
the number is >= 1 (the plane is covered).  Multiplicity of distinct neighbour windows at the boundary is the Wada-type statistic.
Usage: SPECTRE_DIR=/path/to/spectre python3 figures/spectre_window_covering.py N class [npix]    -> figures/spectre_window_covering_N{N}_c{class}.csv
"""
import io, contextlib, os, sys, itertools, time
SP = os.environ.get("SPECTRE_DIR", "../spectre")
sys.path.insert(0, os.path.join(SP, "boundary_certificate")); sys.path.insert(0, SP)
import numpy as np
import mpmath as mp
t00 = time.time()
with contextlib.redirect_stdout(io.StringIO()):
    import spectre_window as sw
from spectre_exact import slot_transforms, SPECTRE, mul, apply, IDENT, SUPER_RULES, ttrans, trot, to_c
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
print("imports %.0fs" % (time.time() - t00), flush=True)
lam = float(sw.LAM); RAD = float(sw.RAD)
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
A_CLASS = int(sys.argv[2]) if len(sys.argv) > 2 else 0
NPIX = int(sys.argv[3]) if len(sys.argv) > 3 else 700
quads = [[SPECTRE[3], SPECTRE[5], SPECTRE[7], SPECTRE[11]]]; TS = [None]
for j in range(N):
    Ts, nq = slot_transforms(quads[-1]); TS.append(Ts); quads.append(nq)
Bm = mp.matrix(4, 4)
for c, v in enumerate(sw.E[0] + sw.F[0]):
    for r in range(4): Bm[r, c] = v[r]
Bim = mp.inverse(Bm)
STARm = sw.T[0] * mp.matrix([[Bim[r, c] for c in range(4)] for r in (2, 3)])
STAR = np.array(STARm.tolist(), dtype=float)
groups = {}
VEC = {}
def rec(s, j, T, z):
    if j == 0:
        g = groups.setdefault((tuple(T[0].flatten()), 'G' if s == 'Gamma' else 'O'), [T[0], [], []])
        g[1].append(STARm * mp.matrix([mp.mpf(int(x)) for x in T[1]])); g[2].append(z)
        VEC.setdefault((tuple(T[0].flatten()), 'G' if s == 'Gamma' else 'O'), []).append(tuple(int(x) for x in T[1])); return
    p = (j - 1) % 2
    for i, c in enumerate(SUPER_RULES[s]):
        if c is not None:
            rec(c, j - 1, mul(T, TS[j][i]), sw.O[(i, p)].T * (sw.GA[(i, p)] + z / sw.LAM))
rec('Delta', N, IDENT, mp.matrix(2, 1))
FR, FRm = [], []
for (fk, S), (A, Y, Z) in groups.items():
    X = mp.matrix([[z[0], z[1], 1] for z in Z]); Ym = mp.matrix([[y[0], y[1]] for y in Y])
    NX, B2 = X.T * X, X.T * Ym
    cols = [mp.lu_solve(NX, B2[:, k]) for k in range(2)]
    sol = mp.matrix([[cols[k][i] for k in range(2)] for i in range(3)])
    R, c = sol[0:2, 0:2].T, sol[2, 0:2].T
    res = max(abs(x) for x in (X * sol - Ym)); G = R.T * R; s2 = G[0, 0]
    assert res < mp.mpf(10) ** -40 and max(abs(x) for x in (G - s2 * mp.eye(2))) < mp.mpf(10) ** -40, (res, G)
    sc = mp.sqrt(s2); Rm = R / sc
    FRm.append((Rm, c, sc))
    FR.append((S, A, np.array(Rm.tolist(), dtype=float), np.array(c.tolist(), dtype=float).ravel(), float(sc), len(Z)))
scale = FR[0][4]
print('window classes (shape, frame):', len(FR), ' tiles per class', [f[5] for f in FR], ' shapes', [f[0] for f in FR], ' similarity ratio', scale, ' %.0fs' % (time.time() - t00), flush=True)
LEAVES = {'O': [IDENT], 'G': [IDENT, mul(ttrans(SPECTRE[8]), trot(1))]}
def shape(S, A, t):
    return unary_union([Polygon([(lambda w: (w.real, w.imag))(to_c(apply(mul((A, t), L), v))) for v in SPECTRE])
                        for L in LEAVES[S]])

PH = np.array([[to_c(np.eye(4, dtype=object)[k]).real for k in range(4)], [to_c(np.eye(4, dtype=object)[k]).imag for k in range(4)]])
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from lakes_of_wada import spectre_window as SWR
from scipy.spatial import cKDTree
import sympy
from sympy.matrices.normalforms import hermite_normal_form
keys = list(groups.keys())
# lattice of tile-vector differences (index 36 in Z^4), common to all classes
D = [tuple(a - b for a, b in zip(v, VEC[keys[0]][0])) for v in VEC[keys[0]][1:]]
H = hermite_normal_form(sympy.Matrix(D).T)
assert H.shape == (4, 4)
detH = abs(int(H.det())); ADJ = np.array((H.inv() * detH).tolist(), dtype=object).astype(np.int64)
print("difference lattice index in Z^4:", detH, flush=True)
def in_lattice(W):                                   # rows w; w in lattice iff H^-1 w integral
    return np.all((W @ ADJ.T) % detH == 0, axis=1)
for k in keys:                                       # every class has differences in the same lattice
    d = np.array([[a - b for a, b in zip(v, VEC[k][0])] for v in VEC[k]], dtype=np.int64)
    assert in_lattice(d).all()
trees = {S: cKDTree(SWR.points(S, 0, 7)[0]) for S in "OG"}
TOL = float(os.environ.get("TOL", "0.008"))
SUF = "" if NPIX == 700 else "_n%d" % NPIX
a = A_CLASS
S1, A1, R1, c1, _, _ = FR[a]; u_a = np.array(VEC[keys[a]][0], dtype=np.int64)
P1 = shape(S1, A1, np.zeros(4, dtype=object))
# candidate relative vectors: small physical part, bounded internal part, in the right coset for each class b
ymax = 2 * RAD * scale + 2 * max(np.linalg.norm(f[3]) for f in FR) + 1
M4 = np.vstack([PH, STAR]); Mi = np.linalg.inv(M4)
bnd = int(np.ceil(np.linalg.norm(Mi, axis=1).max() * np.hypot(12.0, ymax))) + 1
rng = np.arange(-bnd, bnd + 1)
cands = []
for x0 in rng:
    Vc = np.array(list(itertools.product([x0], rng, rng, rng)))
    ok = np.linalg.norm(Vc @ PH.T, axis=1) < 12.0
    Vc = Vc[ok]
    if len(Vc): cands.append(Vc[np.linalg.norm(Vc @ STAR.T, axis=1) < ymax])
cands = np.concatenate(cands); print("lattice candidates", len(cands), "box edge", bnd, flush=True)
# ======================= phason flips with the contact graph =======================
# Placements (b, w) whose footprint lies within R2 of the footprint of the tile a at the origin; a (b = a, w = 0) is included.
# At internal point z (frame of Z_0(S_a)) the tile (b, w) is present iff z_b = B z + d lies in Z_0(S_b).
R2 = float(os.environ.get("R2", "1.0")); DIAM = float(os.environ.get("DIAM", "0")); DELTA = float(os.environ.get("DELTA", "0.03")); NS = int(os.environ.get("NS", "3000"))
IN_T, OUT_T = float(os.environ.get("IN_T", "0.008")), float(os.environ.get("OUT_T", "0.016"))                           # nearest-point distance: in if < IN_T, out if > OUT_T, else ambiguous
PL = []
for b in range(len(FR)):
    Sb, Ab, Rb, cb, _, _ = FR[b]
    v_b = np.array(VEC[keys[b]][0], dtype=np.int64)
    W = cands[in_lattice(cands - (v_b - u_a))]
    dd = ((c1 - cb)[None, :] + W @ STAR.T) @ Rb / scale
    keep = np.linalg.norm(dd, axis=1) <= 2 * RAD + 0.6      # z may lie up to 0.3 beyond the window radius
    for w, d in zip(W[keep], dd[keep]):
        poly = shape(Sb, Ab, np.array([int(x) for x in w], dtype=object))
        if poly.distance(P1) < R2:
            PL.append((b, Sb, Rb.T @ R1, d, w, poly))
print("tile bounds", P1.bounds, "max class diam", max(shape(f[0],f[1],np.zeros(4,dtype=object)).minimum_rotated_rectangle.length for f in FR));print("placements within %.2f of the tile: %d" % (R2, len(PL)), flush=True)
own_idx = [k for k, p in enumerate(PL) if p[0] == a and (p[4] == 0).all()][0]
rng_ = np.random.default_rng(7)
Zc = trees[S1].data; lo, hi = Zc.min(0) - 0.3, Zc.max(0) + 0.3
Z0 = rng_.uniform(lo, hi, (NS, 2)); ph = rng_.uniform(0, 2 * np.pi, NS)
Z1 = Z0 + DELTA * np.stack([np.cos(ph), np.sin(ph)], 1)
if os.environ.get('SWAP'): Z0, Z1 = Z1, Z0                  # evaluate the reversed step z' -> z
def state(Zs):
    pres = np.zeros((len(Zs), len(PL)), bool); amb = np.zeros((len(Zs), len(PL)), bool)
    for k, (b, Sb, B, d, w, poly) in enumerate(PL):
        dist = trees[Sb].query(Zs @ B.T + d)[0]
        pres[:, k] = dist < IN_T; amb[:, k] = (dist >= IN_T) & (dist <= OUT_T)
    return pres, amb
P0_, A0_ = state(Z0); P1_, A1_ = state(Z1)
usable = ~(A0_.any(1) | A1_.any(1))
changed = usable & (P0_ != P1_).any(1)
print("samples %d, unambiguous %d, with a change of the present set %d" % (NS, usable.sum(), changed.sum()), flush=True)
polys = [p[5] for p in PL]
np.savez('/tmp/flip_states.npz', P0=P0_, P1=P1_, usable=usable, changed=changed, Z0=Z0, Z1=Z1)
def touches(p, q):
    i = p.intersection(q)
    return i.area < 1e-9 and i.length > 1e-6
def overlap(p, q): return p.intersection(q).area
def analyse(pres0, pres1):
    gone = [k for k in np.where(pres0 & ~pres1)[0]]; new = [k for k in np.where(~pres0 & pres1)[0]]
    # components: tiles of gone/new linked when their polygons overlap (an old tile and a new one covering common area)
    nodes = [("g", k) for k in gone] + [("n", k) for k in new]
    parent = list(range(len(nodes)))
    def f(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if overlap(polys[nodes[i][1]], polys[nodes[j][1]]) > 1e-6:
                parent[f(i)] = f(j)
    comps = {}
    for i, n in enumerate(nodes): comps.setdefault(f(i), []).append(n)
    out = []
    from shapely.ops import unary_union as UU
    for comp in comps.values():
        rem = [k for t, k in comp if t == "g"]; add = [k for t, k in comp if t == "n"]
        Fo = UU([polys[k] for k in rem]) if rem else Polygon(); Fn = UU([polys[k] for k in add]) if add else Polygon()
        feq = Fo.symmetric_difference(Fn).area < 1e-6
        # outside tiles (present in both) touching the footprint
        keep = [k for k in np.where(pres0 & pres1)[0]]
        env = (Fo.union(Fn)).buffer(1e-4)
        nb = [k for k in keep if polys[k].intersects(env)]
        def link(cl):
            tiles_ = rem if cl == 0 else add
            inc = {t: frozenset(c for c in nb if touches(polys[t], polys[c])) for t in tiles_}
            pairs = set()
            for t, s in inc.items():
                for c1_ in s:
                    for c2_ in s:
                        if c1_ < c2_: pairs.add((c1_, c2_))
            return inc, pairs
        inc0, pr0 = link(0); inc1, pr1 = link(1)
        ctc0 = set(c for s in inc0.values() for c in s); ctc1 = set(c for s in inc1.values() for c in s)
        Fu = Fo.union(Fn)
        def valid(pres):
            tl = [polys[k] for k in np.where(pres)[0] if polys[k].intersects(Fu) and polys[k].intersection(Fu).area > 1e-9]
            ov = sum(tl[x].intersection(tl[y]).area for x in range(len(tl)) for y in range(x + 1, len(tl)) if tl[x].intersects(tl[y]))
            return (Fu.area - UU(tl).intersection(Fu).area < 1e-6 if tl else False) and ov < 1e-6
        v0, v1 = valid(pres0), valid(pres1)
        out.append(dict(nrem=len(rem), nadd=len(add), feq=feq, rem=rem, add=add, link_changed=pr0 != pr1, nb_changed=ctc0 != ctc1,
                        valid=bool(v0 and v1), npairs0=len(pr0), npairs1=len(pr1), areaerr=Fo.area - Fn.area))
    return out

import collections, json
from shapely.geometry import LineString
from shapely.ops import unary_union as UU
def complex_counts(cells, Fb):
    """V, E, F of the cell complex of the given polygons (T-junctions split), and the numbers on the boundary of the footprint Fb."""
    R = lambda p: (round(p[0], 6), round(p[1], 6))
    rings = [[R(p) for p in c.exterior.coords[:-1]] for c in cells]
    V = set(v for r in rings for v in r)
    E = set()
    for r in rings:
        n = len(r)
        for i in range(n):
            a_, b_ = r[i], r[(i + 1) % n]
            ls = LineString([a_, b_]); pts = [a_, b_] + [v for v in V if v != a_ and v != b_ and ls.distance(Point(v)) < 1e-6]
            pts.sort(key=lambda v: (v[0] - a_[0]) ** 2 + (v[1] - a_[1]) ** 2)
            for k in range(len(pts) - 1): E.add(frozenset((pts[k], pts[k + 1])))
    bd = Fb.boundary
    Vb = set(v for v in V if bd.distance(Point(v)) < 1e-6)
    Eb = set(e for e in E if all(bd.distance(Point(v)) < 1e-6 for v in e) and bd.distance(LineString(list(e)).interpolate(0.5, normalized=True)) < 1e-6)
    return len(V), len(E), len(cells), len(Vb), len(Eb)
INNER = P1.buffer(R2 - DIAM)
def region_valid(pres):
    pol = [polys[k] for k in np.where(pres)[0] if polys[k].intersects(INNER) and polys[k].intersection(INNER).area > 1e-9]
    ov = sum(pol[x].intersection(pol[y]).area for x in range(len(pol)) for y in range(x + 1, len(pol)) if pol[x].intersects(pol[y]))
    return ov < 1e-6 and INNER.area - UU(pol).intersection(INNER).area < 1e-6
nvalid = [i for i in np.where(changed)[0] if region_valid(P0_[i]) and region_valid(P1_[i])]
print("changed samples with a valid tiling of the region before and after: %d of %d" % (len(nvalid), changed.sum()), flush=True)
DIAM_TILE = float(os.environ.get('DIAM_TILE', '5.5'))
rows = []; nclu = 0; ninv = 0
for i in nvalid:
    for e in analyse(P0_[i], P1_[i]):
        rem, add = e["rem"], e["add"]
        if not rem or not add: continue
        Fo = UU([polys[k] for k in rem]); Fn = UU([polys[k] for k in add]); Fu = Fo.union(Fn)
        if min(P1.distance(polys[k]) for k in rem + add) > R2 - DIAM_TILE: continue   # all overlapping tiles are in the placement list
        nclu += 1
        if nclu <= 12: print('DBG', len(rem), len(add), round(Fo.area, 3), round(Fn.area, 3), round(Fo.symmetric_difference(Fn).area, 3), e['valid'], round(float(INNER.intersection(Fu).area / Fu.area), 2), flush=True)
        if not e["valid"] or Fo.symmetric_difference(Fn).area > 1e-6: ninv += 1; continue
        c0 = complex_counts([polys[k] for k in rem], Fo); c1 = complex_counts([polys[k] for k in add], Fn)
        rows.append(dict(nrem=len(rem), nadd=len(add), cls_rem=sorted(PL[k][0] for k in rem), cls_add=sorted(PL[k][0] for k in add),
                         dV=c1[0] - c0[0], dE=c1[1] - c0[1], dF=c1[2] - c0[2], bV=(c0[3], c1[3]), bE=(c0[4], c1[4]),
                         link_changed=bool(e["link_changed"]), diam=float(Fu.bounds[2] - Fu.bounds[0])))
print("clusters inside the valid region: %d; closed (valid, equal footprints): %d; not closed: %d" % (nclu, len(rows), ninv))
cnt = collections.Counter((r["nrem"], r["nadd"]) for r in rows); print("(removed, added):", sorted(cnt.items(), key=lambda x: -x[1])[:12])
print("boundary vertices/edges equal before and after: %d / %d" % (sum(r["bV"][0] == r["bV"][1] and r["bE"][0] == r["bE"][1] for r in rows), len(rows)))
print("Euler: dV - dE + dF == boundary change in (V - E):", sum((r["dV"] - r["dE"] + r["dF"]) == 0 for r in rows), "of", len(rows))
mv = collections.Counter((r["dV"], r["dE"], r["dF"]) for r in rows); print("(dV, dE, dF) counts:", sorted(mv.items(), key=lambda x: -x[1]))
json.dump(dict(NS=NS, DELTA=DELTA, R2=R2, DIAM=DIAM, clusters=nclu, closed=len(rows), rows=rows), open("figures/spectre_flip_moves_c%d.json" % a, "w"), indent=1)
