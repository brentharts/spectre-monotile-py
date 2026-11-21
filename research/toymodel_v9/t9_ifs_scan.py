"""Disjointness defect of the IFS pieces for every dissection combination: for a correct substitution the pieces of W_T' are disjoint,
so  vol(W_T') = sum over roles vol(W_T)/tau^3  exactly. Returns the relative defect per tile type."""
import sys, time, json
sys.path.insert(0, '.')
import numpy as np
import t9_ifs as I
d = I.d
def defect(ch, n=80000, Rb=7.0):
    R = I.roles(ch)
    v8 = I.volumes(ch, n=n, Rb=Rb, D=8, seed=1); v9 = I.volumes(ch, n=n, Rb=Rb, D=9, seed=1)
    out = {}
    for Tp in "KBCA":
        pieces = sum(v8[T] / d.tau ** 3 for (T, Rp, c) in R[Tp])
        out[Tp] = pieces / v9[Tp] - 1 if v9[Tp] > 0 else None
    return out, v9
res = []
t0 = time.time()
for nf, ch in I.F.FAILS:
    df, v = defect(ch); res.append((nf, ch, df, v))
    print(nf, ch, {k: round(x, 3) for k, x in df.items()}, "max|defect| %.3f" % max(abs(x) for x in df.values()), flush=True)
res.sort(key=lambda r: max(abs(x) for x in r[2].values()))
print("\nbest by disjointness:")
for nf, ch, df, v in res[:6]: print(nf, ch, {k: round(x, 3) for k, x in df.items()}, {k: round(x, 2) for k, x in v.items()})
json.dump([(nf, ch, df, v) for nf, ch, df, v in res], open("ifs_scan.json", "w"))
print("%.0fs" % (time.time() - t0))
