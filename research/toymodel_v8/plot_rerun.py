"""Figures for RERUN.md: fig_rerun_contact.png, fig_rerun_stats.png, fig_flip_gallery.png.  Reads the *_tight_* JSON files and flip_examples.json (export_flip_examples.py).
Usage: python3 plot_rerun.py   (from research/toymodel_v8)"""
import json, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False, "figure.dpi": 130})
C = dict(same="#9aa5b1", noniso="#d1495b", iso="#edae49", new="#00798c", old="#8d99ae", rem="#e07a2f", add="#2a6fbb")

# ---------------- Figure 1: contact graph ----------------
pt = json.load(open("spectre_flip_contacts_pattern_c0_tight_d0.03.json")); ps = json.load(open("spectre_flip_contacts_pattern_c0_tight_d0.03_swap.json"))
po = json.load(open("spectre_flip_contacts_pattern_c0_oldtol_box12.json"))
def cats(p):
    q = p["patterns"]; return [q.get("(True, True, True)", 0), q.get("(False, True, False)", 0), q.get("(False, False, False)", 0)]
fig, ax = plt.subplots(1, 3, figsize=(14, 4.2), gridspec_kw=dict(width_ratios=[1.25, 1, 1.15]))
lab = ["same tile set\n(flip elsewhere)", "tile set changed,\ngraph isomorphic\n(classes differ)", "tile set changed,\ngraph NOT isomorphic"]
cols = [C["same"], C["iso"], C["noniso"]]
x = np.arange(3); w = 0.38
a, b = cats(pt), cats(ps)
ax[0].bar(x - w / 2, a, w, color=cols, edgecolor="k", label="step z -> z'"); ax[0].bar(x + w / 2, b, w, color=cols, edgecolor="k", hatch="//", label="reversed step z' -> z")
for xi, (u, v) in enumerate(zip(a, b)): ax[0].text(xi - w / 2, u + 4, str(u), ha="center", fontsize=9); ax[0].text(xi + w / 2, v + 4, str(v), ha="center", fontsize=9)
ax[0].set_xticks(x); ax[0].set_xticklabels(lab, fontsize=8); ax[0].set_ylabel("valid pairs (of %d)" % pt["valid_pairs"])
ax[0].set_title("(a) outcome per flip; solid = forward, hatched = reversed\n(identical: the relation is symmetric, no arrow)", fontsize=9)
# (b) fractions tight vs old
changed_t = a[1] + a[2]; changed_o = cats(po)[1] + cats(po)[2]
fr = [("tight tol.,\nbox 20\n(n=%d)" % pt["valid_pairs"], changed_t / pt["valid_pairs"], a[2] / max(changed_t, 1)),
      ("old tol., box 12\n(n=%d)" % po["valid_pairs"], changed_o / po["valid_pairs"], cats(po)[2] / max(changed_o, 1))]
xx = np.arange(2)
ax[1].bar(xx - 0.2, [f[1] for f in fr], 0.4, color=C["new"], label="tile set changed / valid")
ax[1].bar(xx + 0.2, [f[2] for f in fr], 0.4, color=C["noniso"], label="not isomorphic / changed")
for i, f in enumerate(fr): ax[1].text(i - 0.2, f[1] + 0.02, "%.2f" % f[1], ha="center", fontsize=9); ax[1].text(i + 0.2, f[2] + 0.02, "%.2f" % f[2], ha="center", fontsize=9)
ax[1].set_xticks(xx); ax[1].set_xticklabels([f[0] for f in fr], fontsize=8); ax[1].set_ylim(0, 1.1); ax[1].legend(fontsize=8, frameon=False, loc="upper right")
ax[1].set_title("(b) fractions: stable across settings", fontsize=9)
# (c) heat map sizes
S = np.zeros((6, 6), int)
for k, v in pt["sizes"].items():
    i, j = eval(k); S[i, j] = v
im = ax[2].imshow(S[1:, 1:], cmap="Blues", origin="lower", extent=(0.5, 5.5, 0.5, 5.5))
for i in range(1, 6):
    for j in range(1, 6):
        if S[i, j]: ax[2].text(j, i, S[i, j], ha="center", va="center", fontsize=8, color="w" if S[i, j] > 60 else "k")
ax[2].plot([0.5, 5.5], [0.5, 5.5], "k:", lw=0.8)
ax[2].set_xlabel("tiles meeting the footprint, after"); ax[2].set_ylabel("tiles meeting the footprint, before")
ax[2].set_title("(c) tile count before vs after (valid pairs)\noff-diagonal = a flip changes the count", fontsize=9)
fig.suptitle("Contact graph of the tiles meeting a fixed footprint under a phason step of 0.03 (class 0, level 3; IN_T 0.0015, OUT_T 0.0035)", fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.94)); fig.savefig("fig_rerun_contact.png"); plt.close(fig)

# ---------------- Figure 2: statistics ----------------
st = json.load(open("spectre_flip_stats_tight_d0.03.json")); A = st["partA"]; rhos = sorted(A, key=float); r = np.array([float(k) for k in rhos])
ch = np.array([A[k]["mean_changed"] for k in rhos]); pr = np.array([A[k]["mean_present"] for k in rhos])
old_r = np.array([3, 4, 5, 6, 7.0]); old_ch = np.array([0.060, 0.169, 0.212, 0.348, 0.488]); old_pr = np.array([2.2, 6.2, 7.3, 12.6, 17.3])
fig, ax = plt.subplots(1, 4, figsize=(17, 4.0))
ax[0].loglog(r, ch, "o-", color=C["new"], label="tight tolerance (slope %.2f)" % st["slope"]); ax[0].loglog(old_r, old_ch, "s--", color=C["old"], label="old tolerance (slope 2.22)")
rr = np.linspace(2, 7, 20); ax[0].loglog(rr, ch[2] * (rr / 3.0) ** 2, "k:", label=r"$\rho^2$")
ax[0].set_xlabel(r"disc radius $\rho$"); ax[0].set_ylabel("mean changed tiles"); ax[0].legend(fontsize=8, frameon=False); ax[0].set_title("(a) H8.5: changed tiles grow as $\\rho^2$", fontsize=9)
ax[1].plot(r, ch / pr, "o-", color=C["new"], label="tight"); ax[1].plot(old_r, old_ch / old_pr, "s--", color=C["old"], label="old")
ax[1].set_ylim(0, 0.12); ax[1].set_xlabel(r"$\rho$"); ax[1].set_ylabel("changed per present tile"); ax[1].legend(fontsize=8, frameon=False)
ax[1].set_title("(b) constant density; doubled by\nthe selection effect of dropped samples", fontsize=9)
n = np.array([A[k]["n_area_conserving"] for k in rhos]); p = np.array([A[k]["P_dG_nonzero_given_area_conserved"] for k in rhos])
se = np.sqrt(np.clip(p * (1 - p), 1e-9, None) / np.maximum(n, 1))
ax[2].errorbar(r, p, yerr=se, fmt="o-", color=C["noniso"], capsize=3)
for xi, yi, ni in zip(r, p, n): ax[2].text(xi, yi + 0.07, "n=%d" % ni, ha="center", fontsize=7)
ax[2].set_ylim(0, 1.1); ax[2].set_xlabel(r"$\rho$"); ax[2].set_ylabel("P($n_G$ changed | area conserved)")
ax[2].set_title("(c) H8.4: Mystic-pair count is not\nlocally conserved (not smooth in $\\rho$)", fontsize=9)
rows = np.array(st["partB"]); J = rows[:, 0]; I = rows[:, 1]
labs = ["1", "2", ">=3"]; sels = [J == 1, J == 2, J >= 3]
fr = [(rows[s, 4] / rows[s, 5]).mean() for s in sels]; nn = [int(s.sum()) for s in sels]; pnd = [(I[s] >= 2).mean() if s.any() else 0 for s in sels]
ax[3].bar(labs, fr, color=C["new"], label="mean consistent fraction"); ax[3].plot(labs, pnd, "ko", ms=8, label="P(inside not determined by outside)")
for i, (f, k) in enumerate(zip(fr, nn)): ax[3].text(i, f + 0.03, "n=%d" % k, ha="center", fontsize=8)
ax[3].set_ylim(0, 1.1); ax[3].set_xlabel("multiplicity of outcomes (window pieces near the phase)"); ax[3].legend(fontsize=7, frameon=False, loc="upper right")
ax[3].set_title("(d) H8.6 negative: erased tile always\ndetermined by its surroundings", fontsize=9)
fig.suptitle("Flip statistics, step 0.03, %d usable phases (class 0, level 3, IN_T 0.0015, OUT_T 0.0035)" % st["usable"], fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.93)); fig.savefig("fig_rerun_stats.png"); plt.close(fig)

# ---------------- Figure 3: gallery ----------------
ex = json.load(open("flip_examples.json")); R = ex["R"]
NE = 4                                              # four events: before (top row) and after (bottom row)
fig, axs = plt.subplots(2, NE, figsize=(14, 7.6))
for j, e in enumerate(ex["examples"][:NE]):
    c0 = np.mean(np.array(e["origin_xy"]), axis=0)
    e["n_removed"] = sum(t["status"] == "removed" for t in e["tiles"]); e["n_added"] = sum(t["status"] == "added" for t in e["tiles"])   # tiles shown in the window only
    for side in (0, 1):
        axx = axs[side, j]
        for t in e["tiles"]:
            st_ = t["status"]
            if side == 0 and st_ == "added": continue
            if side == 1 and st_ == "removed": continue
            fc = {"both": "#e8ecef", "removed": C["rem"], "added": C["add"]}[st_]
            axx.add_patch(MPoly(np.array(t["xy"]), closed=True, fc=fc, ec="#3d4852", lw=0.6, hatch="////" if t["G"] else None, alpha=0.95))
        axx.add_patch(MPoly(np.array(e["origin_xy"]), closed=True, fc="none", ec="k", lw=2.2))
        axx.set_xlim(c0[0] - R - 1.5, c0[0] + R + 1.5); axx.set_ylim(c0[1] - R - 1.5, c0[1] + R + 1.5); axx.set_aspect("equal"); axx.axis("off")
        axx.set_title(("before z: %d tiles removed in view" % e["n_removed"]) if side == 0 else ("after z + 0.03: %d tiles added in view" % e["n_added"]), fontsize=9)
fig.suptitle("Flip examples: tiles within 4.5 of the origin tile (black outline). Orange = removed, blue = added, grey = unchanged, hatched = Mystic-pair tile. Region validated: no overlaps, no holes.", fontsize=10)
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig("fig_flip_gallery.png"); plt.close(fig)
print("ok")
