"""
fig_density_clock.py - the Spectre's exact baseline density and how fast a finite region forgets it (Pisot decay; Section 'density-limits').

The density of valence-4 vertices per spectre is rho_0 = (5 - sqrt15)/5 = 0.2254 (exact, twice the Mystic frequency).  For every complete level-j supertile of a K = 6 patch
(242047 tiles) spectre_density_clock.py of the Hat and Spectre repository ($SPECTRE_DIR, imported) gives the weighted count of valence-4 vertices.  Left: the area-weighted mean of
rho_j against rho_0, as a relative error (it falls below 1e-4 by j = 3).  Right: the deviation of a supertile's weighted count from rho_0 times its number of spectres, for the Delta
(label 0) and the other eight labels, in absolute value on a log axis, against the Pisot guide (4 - sqrt15)^j = 0.127^j: it shrinks by a factor of about 8 per level, with no
perimeter term.  The Chair44 contrast (integer inflation 2, notch sum 7 * 2^n) is in the text, not drawn.  Evidence: counted on a finite patch; rho_0 exact in the repository.
The tiling makes no gravitational potential of its own; this is the baseline 1 of the clock rate, nothing more.

    python3 models/fig_density_clock.py                 tests, then the figure (about 45 s)
    python3 models/fig_density_clock.py --tests-only
    python3 models/fig_density_clock.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _spectre  # noqa: E402

JS = (1, 2, 3, 4)
RHO0 = (5 - math.sqrt(15)) / 5
PISOT = 4 - math.sqrt(15)


def data():
    if _spectre.load() is None:
        return None
    import spectre_density_clock as C
    tab = {r["j"]: r for r in C.table(6, (0,) + JS)}
    dev = {}
    for j in JS:
        rho, area, lab = C.supertile_densities(j, 6)
        d = (rho - RHO0) * area
        dev[j] = dict(delta=float(d[lab == 0].mean()), other=float(d[lab != 0].mean()), n=len(rho))
    return dict(C=C, tab=tab, dev=dev)


def run_tests(D=None, verbose=True):
    ok = True

    def check(name, good, info=""):
        nonlocal ok
        ok &= bool(good)
        if verbose:
            print("  %-4s %-72s %s" % ("ok" if good else "FAIL", name, info))

    if D is None:
        check("Hat and Spectre repository not found", False, "set SPECTRE_DIR")
        return False
    rel = {j: abs(D["tab"][j]["mean"] - RHO0) / RHO0 for j in JS}
    check("area-weighted mean of rho_j equals rho_0 to 0.3% at j = 1, 0.1% at 2, 1e-4 at 3 and 4", rel[1] < 3e-3 and rel[2] < 1e-3 and rel[3] < 1e-4 and rel[4] < 1e-4,
          ", ".join("%.1e" % rel[j] for j in JS))
    dv = D["dev"]
    check("Delta deviation is positive, the other labels' negative (j = 1, 2, 3)", all(dv[j]["delta"] > 0 > dv[j]["other"] for j in (1, 2, 3)),
          "Delta %s" % ", ".join("%+.3f" % dv[j]["delta"] for j in (1, 2, 3)))
    ratios = [abs(dv[j + 1]["delta"] / dv[j]["delta"]) for j in (1, 2)] + [abs(dv[j + 1]["other"] / dv[j]["other"]) for j in (1, 2)]
    check("deviations shrink by about 1/8 per level (ratio between 0.09 and 0.18)", all(0.09 < r < 0.18 for r in ratios), ", ".join("%.3f" % r for r in ratios))
    check("the Pisot unit 4 - sqrt15 = 0.127", abs(PISOT - 0.1270167) < 1e-6)
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.2, 4.4))
    rel = [abs(D["tab"][j]["mean"] - RHO0) / RHO0 for j in JS]
    ax.semilogy(JS, rel, "o-", color="#0072B2", lw=1.8)
    ax.set_xticks(JS)
    ax.set_xlabel("supertile level $j$")
    ax.set_ylabel(r"$|\bar\rho_j-\rho_0|/\rho_0$")
    ax.set_title(r"Mean density equals $\rho_0=(5-\sqrt{15})/5$", fontsize=10.5)
    ax.text(0.97, 0.95, "$\\rho_0=%.4f$ per spectre" % RHO0, transform=ax.transAxes, ha="right", va="top", fontsize=9.5)
    dv = D["dev"]
    bx.semilogy(JS, [abs(dv[j]["delta"]) for j in JS], "o-", color="#D55E00", lw=1.8, label=r"$\Delta$ supertiles ($+$)")
    bx.semilogy(JS, [abs(dv[j]["other"]) for j in JS], "s-", color="#009E73", lw=1.8, label="the other eight labels ($-$)")
    guide = [abs(dv[1]["delta"]) * PISOT ** (j - 1) for j in JS]
    bx.semilogy(JS, guide, "--", color="0.35", lw=1.2, label=r"guide $(4-\sqrt{15})^{\,j}=0.127^{\,j}$")
    bx.set_xticks(JS)
    bx.set_xlabel("supertile level $j$")
    bx.set_ylabel(r"$|\,\mathrm{count}-\rho_0\cdot\mathrm{area}\,|$ (vertices)")
    bx.set_title("Deviation shrinks geometrically, no perimeter term", fontsize=10.5)
    bx.legend(loc="upper right", fontsize=8.5, frameon=False)
    for a in (ax, bx):
        for s in ("top", "right"):
            a.spines[s].set_visible(False)
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    D = data()
    if not run_tests(D):
        return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(D, os.path.join(d, "density_clock.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
