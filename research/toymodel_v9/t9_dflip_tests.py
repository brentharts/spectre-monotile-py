"""B2 tests (need dflip_patch_L11.npz, win_I_table.npz, dflip_stats.json): python3 t9_dflip_tests.py"""
import sys, json; sys.path.insert(0, '.')
import numpy as np
import t9_dflip as E, t9_patch as P
z = np.load("dflip_patch_L11.npz"); cen = z["centre"]; res = []
def check(name, ok, info=""): res.append(ok); print(("PASS " if ok else "FAIL ") + name + ("  " + info if info else ""), flush=True)
vs = E.VertexSet(cen, 19.0); m0 = vs.mask(); pv = set(tuple(int(q) for q in w) for W4 in z["verts"] for w in W4)
for c in E.CLS:
    sel = (vs.cls == c) & (np.linalg.norm(vs.phys - cen, axis=1) < 18); mod = {tuple(x) for x in vs.x4[sel & m0]}
    pat = {w for w in pv if P.class_key(np.array(w)) == E.CLS[c] and np.linalg.norm(E.d.phys4(np.array(w)) - cen) < 18}
    check("B2 class %s: model-set vertices equal the substitution patch vertices in r<18: %d vs %d, model-only %d, patch-only %d" % (c, len(mod), len(pat), len(mod - pat), len(pat - mod)), mod == pat)
S = json.load(open("dflip_stats.json")); T = json.load(open("dflip_tiles.json")); r = S["rows"][-1]
check("B2 flips at |gamma| = 0.4 agree with the uniform-window prediction within 10%% (II %.0f vs %.0f, III %.0f vs %.0f)" % (r["removed"]["II"], r["predicted_removed"]["II"], r["removed"]["III"], r["predicted_removed"]["III"]), all(abs(r["removed"][c] - r["predicted_removed"][c]) / r["predicted_removed"][c] < 0.1 for c in ("II", "III")))
check("B2 flips are local hops: median distance removed -> nearest added %.3f = 1.176 (= 2 sin 36 degrees) within 0.01" % T["hop_hist_percentiles"][1], abs(T["hop_hist_percentiles"][1] - 2 * np.sin(np.pi / 5)) < 0.01)
check("B2 tile destruction grows with |gamma|: %s" % np.round(T["tile_destroyed"], 3), all(np.diff(T["tile_destroyed"]) > 0))
check("B2 no class II or III vertex flips at |gamma| = 0.01 (finite-ball gap)", S["rows"][0]["removed"]["II"] == 0 and S["rows"][0]["removed"]["III"] == 0)
print("%d/%d passed" % (sum(res), len(res))); sys.exit(0 if all(res) else 1)
