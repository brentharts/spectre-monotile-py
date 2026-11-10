"""
fig_wilson_corners.py - Wilson action on the A2 + A2 lattice: what a corner costs, and why the Spectre outline is not corner-local (Wilson part of Section 'companion').

Model H of spectre_wilson.py (Hat and Spectre repository, $SPECTRE_DIR, imported): the first-step fluctuation energy of a closed loop on the lattice of the lift,
fitted as E = alpha P + sum_turns b_turn c_turn with P the number of links and c the number of turns of 60, 120 and 90 degrees, on structured loops with sides 3 and 4.
Left: the fitted coefficients (alpha per link; b for the three turn angles; negative b: a corner costs less than the straight links it replaces).  Right: the Spectre
outline (all 71 outlines of levels 1 and 2 have the same energy) and its copies scaled by 2 and 3, as energy against the additive prediction alpha P + sum b c; the gap
0.30, 0.18, 0.19 does not shrink with scale, so the outline is not corner-local at these sizes.  Honest limits (WILSON.md): unconstrained energy only, blunt (60) and right (90)
corners are not separated cleanly by the data, and no constrained first-step energy with block averaging is computed.  Evidence: numeric (a fit; Maxwell-limit check 1.3e-6 in the repository).

    python3 models/fig_wilson_corners.py                 tests, then the figure (about 25 s)
    python3 models/fig_wilson_corners.py --tests-only
    python3 models/fig_wilson_corners.py --no-show --save-dir figures
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _spectre  # noqa: E402

N = 20
SCALES = (1, 2, 3)


def data():
    if _spectre.load() is None:
        return None
    import spectre_wilson as W
    coef, A, y, names = W.fit(W.structured_loops(), N)
    o = W.spectre_outlines(2)[0]
    rows = []
    for k in SCALES:
        scaled = [d for d in o for _ in range(k)]
        P, c = W.corner_counts(scaled)
        pred = float(np.array([P, c[60], c[120], c[90]]) @ coef)
        rows.append(dict(k=k, P=P, corners=dict(c), E=float(W.energy(scaled, N)), pred=pred))
    return dict(W=W, coef=coef, resid=float(np.abs(y - A @ coef).max()), rows=rows)


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
    a, b60, b120, b90 = D["coef"]
    check("alpha = 0.300, b(60) = -0.070, b(120) = -0.315, b(90) = -0.139 (to 0.002)", abs(a - 0.300) < 2e-3 and abs(b60 + 0.070) < 2e-3 and abs(b120 + 0.315) < 2e-3 and abs(b90 + 0.139) < 2e-3,
          "%.4f %.4f %.4f %.4f" % (a, b60, b120, b90))
    check("residual on the training loops below 0.07", D["resid"] < 0.07, "%.4f" % D["resid"])
    r = D["rows"]
    gaps = [x["E"] - x["pred"] for x in r]
    check("outline energy 3.183 at scale 1, prediction 2.884", abs(r[0]["E"] - 3.183) < 2e-3 and abs(r[0]["pred"] - 2.884) < 2e-3, "%.3f / %.3f" % (r[0]["E"], r[0]["pred"]))
    check("gap above the prediction at every scale (0.30, 0.18, 0.19 to 0.01)", all(abs(g - t) < 0.01 for g, t in zip(gaps, (0.30, 0.18, 0.19))), ", ".join("%.3f" % g for g in gaps))
    check("the gap does not shrink to zero with scale: gap(3) >= 0.9 gap(2) > 0.1", gaps[2] >= 0.9 * gaps[1] > 0.1)
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.2, 4.4), gridspec_kw=dict(width_ratios=[1, 1.1]))
    names = [r"$\alpha$ (per link)", r"$b$, turn $60^\circ$", r"$b$, turn $120^\circ$", r"$b$, turn $90^\circ$"]
    vals = list(D["coef"])
    cols = ["#0072B2", "#009E73", "#D55E00", "#E69F00"]
    ax.bar(range(4), vals, color=cols)
    for i, v in enumerate(vals):
        ax.text(i, v + (0.012 if v > 0 else -0.03), "%.3f" % v, ha="center", fontsize=9)
    ax.axhline(0, color="0.3", lw=0.7)
    ax.set_xticks(range(4))
    ax.set_xticklabels(names, fontsize=8.5)
    ax.set_ylim(-0.4, 0.38)
    ax.set_title("Fitted perimeter and corner terms", fontsize=10.5)
    ax.set_ylabel("coefficient of $E=\\alpha P+\\sum b\\,c$")
    ks = [r["k"] for r in D["rows"]]
    gaps = [r["E"] - r["pred"] for r in D["rows"]]
    bx.bar(ks, gaps, 0.55, color="#D55E00", label="Spectre outline: computed minus additive prediction")
    for k, g, r in zip(ks, gaps, D["rows"]):
        bx.text(k, g + 0.008, "%.2f\n(E=%.2f, P=%d)" % (g, r["E"], r["P"]), ha="center", fontsize=8.5)
    bx.axhspan(-D["resid"], D["resid"], color="#0072B2", alpha=0.18, label="largest miss on the fitted training loops (%.3f)" % D["resid"])
    bx.axhline(0, color="0.3", lw=0.7)
    bx.set_xticks(ks)
    bx.set_xticklabels(["outline $\\times%d$" % k for k in ks])
    bx.set_xlim(0.4, 3.6)
    bx.set_ylim(-0.12, 0.45)
    bx.set_ylabel("energy above the additive prediction")
    bx.set_title("The outline is not corner-local: the gap stays", fontsize=10.5)
    bx.legend(loc="upper right", fontsize=8, frameon=False)
    bx.text(0.98, 0.02, "unconstrained energy only", transform=bx.transAxes, ha="right", fontsize=8, color="0.4")
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
    figure(D, os.path.join(d, "wilson_corners.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
