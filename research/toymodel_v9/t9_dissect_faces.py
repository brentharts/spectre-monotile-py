"""Select among the dissections by face compatibility: two faces of the inflated tiles that are congruent (by an element of the group and a translation)
must be cut into triangles in the same way, because the two tiles meeting there are cut independently."""
import sys, itertools, json
sys.path.insert(0, '.')
import numpy as np, t9_dissect as d
def tile_vs(name, g, p):
    return [tuple(int(z) for z in (np.array(p) + d.RX[g] @ d.T4[name][j])) for j in range(4)]
sols, targ, raw = {}, {}, {}
for name in "KBCA":
    V4, out, nodes = d.solve(name, cap=200, Qmax=float(__import__("os").environ.get("QM","9")))
    targ[name] = [tuple(int(z) for z in v) for v in V4]
    raw[name] = [o for o in out if o[0] == d.MROW[name]]
    sols[name] = [[(t, tile_vs(t, g, p)) for t, g, p in o[1]] for o in raw[name]]
def plane_of(tri):
    P = np.array([d.phys4(v) for v in tri]); n = np.cross(P[1] - P[0], P[2] - P[0]); n /= np.linalg.norm(n); return n, n @ P[0]
def face_triangulation(sol, tface):
    n, off = plane_of(tface); res = set()
    for ty, vs in sol:
        for tri in itertools.combinations(vs, 3):
            P = np.array([d.phys4(v) for v in tri])
            if np.allclose(P @ n, off, atol=1e-9):
                # lies in the face plane; keep if inside the face triangle
                Pf = np.array([d.phys4(v) for v in tface]); Mx = (Pf[1:] - Pf[0]).T
                lam = np.linalg.lstsq(Mx, (P - Pf[0]).T, rcond=None)[0]
                if (lam > -1e-9).all() and (lam.sum(0) < 1 + 1e-9).all(): res.add(frozenset(tri))
    return res
faces = {}
for name in "KBCA":
    for fi in range(4):
        faces[(name, fi)] = [targ[name][j] for j in range(4) if j != fi]
def congruences(f1, f2):
    """all (gi, t, perm) with g f1[i] + t = f2[perm[i]]"""
    out = []
    for gi in range(120):
        for perm in itertools.permutations(range(3)):
            t = np.array(f2[perm[0]]) - d.RX[gi] @ np.array(f1[0])
            if all(tuple(int(z) for z in (d.RX[gi] @ np.array(f1[i]) + t)) == f2[perm[i]] for i in range(3)): out.append((gi, t))
    return out
keys = list(faces); pairs = []
for a in range(len(keys)):
    for b in range(a, len(keys)):
        cg = congruences(faces[keys[a]], faces[keys[b]])
        if cg: pairs.append((keys[a], keys[b], cg))
print("face pairs that are congruent:", len(pairs), flush=True)
def img(tris, gi, t): return {frozenset(tuple(int(z) for z in (d.RX[gi] @ np.array(v) + t)) for v in tr) for tr in tris}
ok_choice = {}
for name in "KBCA":
    ok_choice[name] = list(range(len(sols[name])))
tri_cache = {(n, i, fi): face_triangulation(sols[n][i], faces[(n, fi)]) for n in "KBCA" for i in range(len(sols[n])) for fi in range(4)}
# consistency: for every pair of congruent faces, (some) congruence maps the triangulation of one to that of the other
def compatible(c1, c2, k1, k2, cg):
    T1, T2 = tri_cache[(k1[0], c1, k1[1])], tri_cache[(k2[0], c2, k2[1])]
    return any(img(T1, gi, t) == T2 for gi, t in cg)
FAILS = []
for combo in itertools.product(*[range(len(sols[n])) for n in 'KBCA']):
    ch = dict(zip('KBCA', combo)); FAILS.append((sum(1 for k1, k2, cg in pairs if not compatible(ch[k1[0]], ch[k2[0]], k1, k2, cg)), ch))
FAILS.sort(key=lambda x: x[0])
good = []
for combo in itertools.product(*[range(len(sols[n])) for n in "KBCA"]):
    ch = dict(zip("KBCA", combo)); ok = True
    for k1, k2, cg in pairs:
        if not compatible(ch[k1[0]], ch[k2[0]], k1, k2, cg): ok = False; break
    if ok: good.append(ch)
print("combinations", np.prod([len(sols[n]) for n in "KBCA"]), "face-compatible:", len(good), good[:10], flush=True)
json.dump(dict(good=good, fails=[(f, c) for f, c in FAILS]), open("dissection_choices.json", "w"))
