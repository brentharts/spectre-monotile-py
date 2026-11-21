"""B2 figures: dflip_windows.png (window cross-sections), dflip_slab.png (vertex slab with flips), dflip_stats.png (flip counts, hop length, tile survival).  python3 t9_dflip_figs.py"""
import sys, json; sys.path.insert(0, '.')
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import t9_dflip as E, t9_win_def as W
tau = E.tau; COL = {"I": "#1f77b4", "II": "#d62728", "III": "#2ca02c"}
# ---------- figure 1: window cross-sections
def unit(a): a = np.array(a, float); return a / np.linalg.norm(a)
planes = [("plane of a 5-fold and a 3-fold axis", unit((0, tau, 1)), unit((1, 1, 1))), ("plane z = 0 (two 2-fold axes)", np.array([1., 0, 0]), np.array([0, 1., 0])), ("plane through 5-fold, 2-fold axes", unit((0, tau, 1)), np.array([1., 0, 0]))]
fig, ax = plt.subplots(3, 3, figsize=(10, 10)); g = np.linspace(-3, 3, 360); U, V = np.meshgrid(g, g)
for i, (name, a, b) in enumerate(planes):
    b = b - (b @ a) * a; b /= np.linalg.norm(b)
    X = U[..., None] * a + V[..., None] * b; X = X.reshape(-1, 3)
    for j, c in enumerate(("I", "II", "III")):
        m = E.window(c, X).reshape(U.shape); ax[i, j].imshow(m, extent=[-3, 3, -3, 3], origin="lower", cmap="Greys", alpha=0.85)
        ax[i, j].set_title("class %s   %s" % (c, name if j == 0 else ""), fontsize=8); ax[i, j].set_aspect("equal"); ax[i, j].tick_params(labelsize=6)
fig.suptitle("Danzer ABCK windows in the internal (star) frame: I (dodecahedron with 12 star pits), II (dodecahedron), III (great dodecahedron)", fontsize=9); fig.tight_layout(); fig.savefig("dflip_windows.png", dpi=130); plt.close(fig)
# ---------- shared data
z = np.load("dflip_patch_L11.npz"); cen = z["centre"]; RB = 18.0
vs = E.VertexSet(cen, RB + 1.0); inb = np.linalg.norm(vs.phys - cen, axis=1) <= RB; m0 = vs.mask()
# ---------- figure 2: slab
delta = 0.2; rng = np.random.default_rng(4); u = rng.normal(size=3); u /= np.linalg.norm(u); gam = delta * u; m1 = vs.mask(gam)
# slab: project onto the plane spanned by two physical directions; slab thickness 1.2 along third
P0 = vs.phys - cen; sl = (np.abs(P0[:, 2]) < 0.9) & (np.linalg.norm(P0[:, :2], axis=1) < 11)
fig, ax = plt.subplots(1, 2, figsize=(13, 6.5))
for c in E.CLS:
    s = sl & m0 & (vs.cls == c); ax[0].scatter(P0[s, 0], P0[s, 1], s=14, c=COL[c], label="class %s (%d)" % (c, s.sum()))
ax[0].set_title("ABCK vertices in a slab |z|<0.9 (phase 0)", fontsize=10); ax[0].legend(fontsize=8)
kept = sl & m0 & m1; rem = sl & m0 & ~m1; add = sl & ~m0 & m1
ax[1].scatter(P0[kept, 0], P0[kept, 1], s=8, c="0.75", label="kept"); ax[1].scatter(P0[rem, 0], P0[rem, 1], s=30, c="red", label="removed (%d)" % rem.sum()); ax[1].scatter(P0[add, 0], P0[add, 1], s=30, c="blue", marker="s", label="added (%d)" % add.sum())
Aall = np.flatnonzero(~m0 & m1 & inb); T = cKDTree(vs.phys[Aall])
for i in np.flatnonzero(rem):
    dd, k = T.query(vs.phys[i])
    if dd < 2.0: j = Aall[k]; ax[1].plot([P0[i, 0], vs.phys[j, 0] - cen[0]], [P0[i, 1], vs.phys[j, 1] - cen[1]], c="k", lw=0.5)
ax[1].set_title("shift |gamma|=%.2f: removed to nearest added vertex" % delta, fontsize=10); ax[1].legend(fontsize=8)
for a_ in ax: a_.set_aspect("equal"); a_.set_xlabel("x"); a_.set_ylabel("y")
fig.tight_layout(); fig.savefig("dflip_slab.png", dpi=130); plt.close(fig)
# ---------- figure 3: statistics and tile survival
S = json.load(open("dflip_stats.json")); deltas = [r["delta"] for r in S["rows"]]
key = {tuple(x): i for i, x in enumerate(vs.x4)}; tv = []; cls_known = []
for W4 in z["verts"]:
    if np.linalg.norm(np.mean([E.d.phys4(w) for w in W4], 0) - cen) > 12: continue
    idx = [key.get(tuple(int(q) for q in w), -1) for w in W4]; tv.append(idx)
tv = np.array(tv); known = tv >= 0; print("tiles in ball 12:", len(tv), "; fraction of tile vertices of class IV/unmodelled:", 1 - known.mean())
surv = []
dirs = rng.normal(size=(12, 3)); dirs /= np.linalg.norm(dirs, axis=1)[:, None]
for dl in deltas:
    f = []
    for uu in dirs:
        m = vs.mask(dl * uu); ok = np.where(known, m[np.maximum(tv, 0)], True).all(1); f.append(ok.mean())
    surv.append(np.mean(f))
fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))
for c in E.CLS:
    ax[0].loglog(deltas, [max(r["removed"][c], 0.5) for r in S["rows"]], "o-", c=COL[c], label="class %s measured" % c); ax[0].loglog(deltas, [r["predicted_removed"][c] for r in S["rows"]], "--", c=COL[c], label="class %s uniform-window prediction" % c)
ax[0].set_xlabel("|gamma|"); ax[0].set_ylabel("vertices removed in the ball r<18 (mean of 12 directions)"); ax[0].legend(fontsize=7); ax[0].set_title("flip count vs phase shift")
ax[1].plot(deltas, [1 - s for s in surv], "ko-"); ax[1].set_xscale("log"); ax[1].set_xlabel("|gamma|"); ax[1].set_ylabel("fraction of tiles destroyed"); ax[1].set_title("tiles with at least one removed vertex (class IV vertices unmodelled)")
dl = 0.2; hist = []
for uu in dirs:
    m = vs.mask(dl * uu); R_ = np.flatnonzero(m0 & ~m & inb); A_ = np.flatnonzero(~m0 & m & inb)
    if len(R_) and len(A_): hist += list(cKDTree(vs.phys[A_]).query(vs.phys[R_])[0])
ax[2].hist(hist, bins=np.linspace(0, 5, 60), color="0.4"); ax[2].set_xlabel("distance removed -> nearest added vertex"); ax[2].set_title("locality of flips, |gamma| = 0.2"); 
fig.tight_layout(); fig.savefig("dflip_stats.png", dpi=130); plt.close(fig)
json.dump(dict(deltas=deltas, tile_destroyed=[1 - s for s in surv], hop_hist_percentiles=[float(x) for x in np.percentile(hist, [10, 50, 90])]), open("dflip_tiles.json", "w"), indent=1)
print("tile destroyed", dict(zip(deltas, np.round([1 - s for s in surv], 4)))); print("hop length percentiles", np.percentile(hist, [10, 50, 90]).round(3), "; distinct hop lengths (rounded 0.01):", sorted(set(np.round(hist, 2)))[:12])
