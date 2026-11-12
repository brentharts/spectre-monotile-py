#!/usr/bin/env python3
"""spectre_hat_critical.py - figure for the critical displacement of the reflex outline (MATH_GAPS O1).

Writes figures/spectre_hat_critical.png (and caches the grid in /tmp).  Needs research/toymodel_v4/t4_critical.py.
  (a) h*(b, k) as omega_c = 2 arctan h* (degrees) on a grid, with the closed-form family V overlaid as lines
  (b) h* against k for fixed b: the vertex-contact closed form h_V against the brute-force value
  (c) the outline just below and just above h* for three (b, k) cases, the contact marked
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(HERE, "research", "toymodel_v4"))
sys.path.insert(0, HERE)
import t4_critical as c  # noqa: E402

c.N_SAMPLES = 120
c._T = np.linspace(0.0, 1.0, c.N_SAMPLES + 1)
c._BERN = np.stack([(1 - c._T) ** 3, 3 * c._T * (1 - c._T) ** 2, 3 * c._T ** 2 * (1 - c._T), c._T ** 3], axis=1)

BS = np.linspace(1.0, c.SQRT3, 11)
KS = np.linspace(0.0, 0.8, 9)
CACHE = "/tmp/hcrit_grid.npy"


def grid():
    if os.path.exists(CACHE):
        return np.load(CACHE)
    H = np.array([[c.h_brute(b, k, steps=16) for b in BS] for k in KS])
    np.save(CACHE, H)
    return H


def outline(b, k, h, ax, color, lw=1.2, fixed=False):
    cp = c.moved_tile(b, k, h).control_points()
    S = np.einsum("mj,ejd->emd", c._BERN, cp)
    for e in range(14):
        if e in (7, 8):
            ax.plot(S[e][:, 0], S[e][:, 1], color=color, lw=1.6)
        elif fixed:
            ax.plot(S[e][:, 0], S[e][:, 1], color="0.5", lw=1.0)
    ax.set_aspect("equal")
    ax.axis("off")


def main():
    H = grid()
    omega = 2 * np.degrees(np.arctan(H))
    fig = plt.figure(figsize=(13, 8.5))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.1, 1])
    ax = fig.add_subplot(gs[0, 0:2])
    im = ax.pcolormesh(BS, KS, omega, shading="nearest", cmap="viridis")
    fig.colorbar(im, ax=ax, label=r"$\omega_c=2\arctan h^*$ (degrees)")
    ax.set_xlabel("b"); ax.set_ylabel("k")
    ax.set_title(r"(a) critical angle of the reflex outline, curved edges")
    ax2 = fig.add_subplot(gs[0, 2])
    for b, col in zip((1.0, 1.2, 1.4, c.SQRT3), ("C0", "C1", "C2", "C3")):
        j = int(np.argmin(abs(BS - b)))
        ax2.plot(KS, H[:, j], "o-", color=col, ms=4, label="b=%.2f brute" % BS[j])
        ax2.plot(KS, [c.h_V(BS[j], k) for k in KS], "--", color=col, lw=1)
    ax2.set_ylim(1.5, 3.0)
    ax2.set_xlabel("k"); ax2.set_ylabel(r"$h^*$")
    ax2.set_title(r"(b) dashed: closed form $h_V(b,k)$")
    ax2.legend(fontsize=7)
    cases = [(1.0, 0.2), (1.4, 0.4), (1.3, 0.8)]
    for i, (b, k) in enumerate(cases):
        a = fig.add_subplot(gs[1, i])
        hs = c.h_brute(b, k, steps=22)
        outline(b, k, hs * 0.93, a, "tab:green", fixed=True)
        outline(b, k, hs * 1.07, a, "crimson")
        a.set_title("(c%d) b=%.1f, k=%.1f, $h^*$=%.3f  (edges 7, 8: green below, red above)" % (i + 1, b, k, hs), fontsize=9)
    fig.tight_layout()
    out = os.path.join(HERE, "figures", "spectre_hat_critical.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
