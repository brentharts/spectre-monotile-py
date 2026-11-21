"""Export a few flip events (before/after tilings around the origin tile) from the engine cache to flip_examples.json.
Needs the cache /tmp/PL_c0_r14_p20.pkl and /tmp/flip_states.npz written by spectre_flip_moves.py (IN_T=0.0015 OUT_T=0.0035 DELTA=0.03 R2=14)."""
import pickle, json, sys, numpy as np
from shapely.ops import unary_union
PL = pickle.load(open('/tmp/PL_c0_r14_p20.pkl', 'rb')); d = np.load('/tmp/flip_states.npz')
P0, P1, ch, Z0, Z1 = d['P0'], d['P1'], d['changed'], d['Z0'], d['Z1']
polys = [p[5] for p in PL]; cls = [int(p[0]) for p in PL]
own = [k for k, p in enumerate(PL) if p[0] == 0 and (p[4] == 0).all()][0]; P = polys[own]
R = 4.5; reg = P.buffer(R); inner = P.buffer(R - 0.0)
def valid(pres):
    pol = [polys[k] for k in np.where(pres)[0] if polys[k].intersects(reg)]
    ov = sum(pol[x].intersection(pol[y]).area for x in range(len(pol)) for y in range(x + 1, len(pol)) if pol[x].intersects(pol[y]))
    return ov < 1e-6 and reg.area - unary_union(pol).intersection(reg).area < 1e-6
foot = P.buffer(1.0)
out = []
for i in np.where(ch)[0]:
    rem = np.where(P0[i] & ~P1[i])[0]; add = np.where(~P0[i] & P1[i])[0]
    near = [k for k in list(rem) + list(add) if polys[k].intersects(foot)]
    if not near: continue
    if not (valid(P0[i]) and valid(P1[i])): continue
    out.append((i, len(rem), len(add)))
print('candidate events', len(out))
rng = np.random.default_rng(1); pick = [out[j] for j in rng.choice(len(out), min(6, len(out)), replace=False)]
ex = []
for i, nr, na in pick:
    tiles = []
    for k in np.where(P0[i] | P1[i])[0]:
        if not polys[k].intersects(reg): continue
        st = 'both' if (P0[i][k] and P1[i][k]) else ('removed' if P0[i][k] else 'added')
        tiles.append(dict(status=st, cls=cls[k], G=bool(PL[k][1] == 'G'), xy=[list(map(float, c)) for c in polys[k].exterior.coords]))
    ex.append(dict(sample=int(i), z0=Z0[i].tolist(), z1=Z1[i].tolist(), n_removed=nr, n_added=na, origin_xy=[list(map(float, c)) for c in P.exterior.coords], tiles=tiles))
json.dump(dict(R=R, examples=ex), open('flip_examples.json', 'w'))
print('written', len(ex))
