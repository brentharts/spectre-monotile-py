#!/usr/bin/env python3
"""
toymodel_v12: the figure of the Hat Trick section of `quantgravalt.tex` (figures/fig_hattrick_register.png).

(a), (b) one rotor (the 40 kites around a pentagon) in its two states, drawn in the tangent plane at the pentagon;
the dot on each hat is its flat vertex.  (c) the OTOC of `results_evolution.json`.

Run:  HEXASPHERE_DIR=... python3 t12_figure.py
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_pairs_sphere as P       # noqa: E402
import t12_flip_graph as G         # noqa: E402

INK, MUTED, HOLE = "#1f1f1f", "#6b6b6b", "#d9d9d9"
FILL = "#c6dbef"
SERIES = ["#0072B2", "#E69F00", "#009E73"]          # Okabe-Ito blue, orange, green


def rotor_polys(b, tiling, p):
    """Outlines (tangent-plane coordinates at pentagon p) of the 5 hats of rotor p, their flat vertices, the pentagon."""
    n = b.V[b.pent[p]] / np.linalg.norm(b.V[b.pent[p]])
    e1 = np.cross(n, [0.3, 0.5, 0.8])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    pr = lambda q: np.array([(q / (q @ n)) @ e1, (q / (q @ n)) @ e2])
    touch = set()
    for k in b.S.kites:
        t, i = divmod(k, 3)
        if b.pent[p] in (b.F[t][(i + 1) % 3], b.F[t][(i + 2) % 3]):
            touch.add(k)
    hats = [h for h in tiling if touch & set(h)]
    polys, flats = [], []
    for h in hats:
        loop, _, _ = b._outline(sorted(h))
        polys.append(np.array([pr(b.S.corner_position(c)) for c in loop]))
        flats.append(pr(b.S.corner_position(P.flat_vertex(b, h))))
    pent = np.array([pr(b.S.corner_position(c)) for c in b.pentagon_loop(b.pent[p])])
    return polys, flats, pent


def main():
    b, Ts = G.all_tilings()
    sig = {G.bits(b, T): T for T in Ts}
    z = tuple([0] * 12)
    one = tuple(1 if q == 0 else 0 for q in range(12))
    fig, ax = plt.subplots(1, 3, figsize=(10.5, 3.6), gridspec_kw={"width_ratios": [1, 1, 1.25]})
    for a, s, title in ((ax[0], z, r"(a) rotor in state 0"), (ax[1], one, r"(b) rotor in state 1")):
        polys, flats, pent = rotor_polys(b, sig[s], 0)
        a.add_patch(Polygon(pent, closed=True, facecolor=HOLE, edgecolor=MUTED, lw=0.8))
        for q in polys:
            a.add_patch(Polygon(q, closed=True, facecolor=FILL, edgecolor=INK, lw=1.1, joinstyle="round"))
        F = np.array(flats)
        a.plot(F[:, 0], F[:, 1], "o", ms=5.5, mfc=SERIES[1], mec="white", mew=1.0, zorder=5)
        allp = np.vstack(polys)
        m = np.abs(allp).max() * 1.06
        a.set_xlim(-m, m)
        a.set_ylim(-m, m)
        a.set_aspect("equal")
        a.axis("off")
        a.set_title(title, fontsize=10, color=INK, loc="left")
    ax[0].text(0.5, -0.04, "flat vertices on the five edges at the cone point", transform=ax[0].transAxes, ha="center", va="top", fontsize=8, color=MUTED)
    ax[1].text(0.5, -0.04, "flat vertices on the pentagon (the hole)", transform=ax[1].transAxes, ha="center", va="top", fontsize=8, color=MUTED)
    d = json.load(open(os.path.join(HERE, "results_evolution.json")))["otoc"][0]
    t, C = np.array(d["times"]), np.array(d["C_d1_d2_d3"])
    a = ax[2]
    for k in range(3):
        a.plot(t, C[:, k], "-o", color=SERIES[k], lw=2, ms=3.5)
        a.text(t[-1] + 0.12, C[-1, k], "distance %d" % (k + 1), color=INK, fontsize=8.5, va="center")
    a.set_xlim(0, 7.6)
    a.set_ylim(0, 1.02)
    a.set_xlabel(r"time ($1/J$)", fontsize=9, color=INK)
    a.set_ylabel(r"$C_d(t)$", fontsize=9, color=INK)
    a.set_title(r"(c) spreading of a flip ($\Gamma=1$, $g=2.5$, $\kappa=0$)", fontsize=10, color=INK, loc="left")
    for sp_ in ("top", "right"):
        a.spines[sp_].set_visible(False)
    a.grid(axis="y", color="#e6e6e6", lw=0.6)
    a.tick_params(labelsize=8, colors=MUTED)
    fig.tight_layout()
    out = os.path.join(HERE, "..", "..", "figures", "fig_hattrick_register.png")
    fig.savefig(out, dpi=200)
    print("wrote", os.path.abspath(out))


if __name__ == "__main__":
    main()
