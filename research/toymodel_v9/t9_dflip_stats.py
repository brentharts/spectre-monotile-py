"""B2 statistics: vertex phason flips of the Danzer model set. usage: python3 t9_dflip_stats.py  -> dflip_stats.json
For internal shifts gamma of length delta (12 generic directions each): removed / added vertices per class in a physical ball, the prediction
vol(W minus (W-gamma))/vol(W) x N_c, the distance from each removed vertex to the nearest added vertex (locality of the flip), and tile survival."""
import sys, json, time; sys.path.insert(0, '.')
import numpy as np
from scipy.spatial import cKDTree
import t9_dflip as E, t9_win_def as W
z = np.load("dflip_patch_L11.npz"); cen = z["centre"]; RB = 18.0
vs = E.VertexSet(cen, RB + 1.0); inb = np.linalg.norm(vs.phys - cen, axis=1) <= RB; m0 = vs.mask()
rng = np.random.default_rng(4); dirs = rng.normal(size=(12, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
out = dict(ball=RB, cen=list(cen), N={c: int((m0 & inb & (vs.cls == c)).sum()) for c in E.CLS}, rows=[])
X = rng.uniform(-3.2, 3.2, size=(600000, 3)); box = 6.4 ** 3
VOL = {c: E.window(c, X).mean() * box for c in E.CLS}
print("ball radius", RB, "vertices", out["N"], "window volumes (MC)", {c: round(v, 2) for c, v in VOL.items()})
for delta in (0.01, 0.02, 0.05, 0.1, 0.2, 0.4):
    rem = {c: [] for c in E.CLS}; add = {c: [] for c in E.CLS}; pred = {c: [] for c in E.CLS}; near = []; surv = []
    for u in dirs:
        g = delta * u; m = vs.mask(g)
        for c in E.CLS:
            s = inb & (vs.cls == c); rem[c].append(int((m0 & ~m & s).sum())); add[c].append(int((~m0 & m & s).sum()))
            pred[c].append(E.window(c, X).__and__(~E.window(c, X + g)).mean() * box / VOL[c] * out["N"][c])
        R_ = np.flatnonzero(m0 & ~m & inb); A_ = np.flatnonzero(~m0 & m & inb)
        if len(R_) and len(A_): near += list(cKDTree(vs.phys[A_]).query(vs.phys[R_])[0])
    row = dict(delta=delta, removed={c: float(np.mean(v)) for c, v in rem.items()}, added={c: float(np.mean(v)) for c, v in add.items()}, predicted_removed={c: float(np.mean(v)) for c, v in pred.items()},
               nearest_added_median=float(np.median(near)) if near else None, nearest_added_pct=[float(x) for x in np.percentile(near, [10, 50, 90])] if near else None)
    out["rows"].append(row)
    print("delta %.2f removed %s (pred %s) added %s ; nearest added distance 10/50/90%% %s" % (delta, {c: round(v, 1) for c, v in row["removed"].items()}, {c: round(v, 1) for c, v in row["predicted_removed"].items()}, {c: round(v, 1) for c, v in row["added"].items()}, None if not near else np.round(row["nearest_added_pct"], 2)), flush=True)
json.dump(out, open("dflip_stats.json", "w"), indent=1)
