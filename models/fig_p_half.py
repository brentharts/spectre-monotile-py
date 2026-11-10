"""
fig_p_half.py - the flavour frequency p = Pr(Phi^2) is 1/2: the measured values fall toward it, and charge conservation balances only at p = 1/2.

Left: p measured on the Spectre patches at depths 3, 4, 5 (spectre_mystic_xbond.measured_p, from the contact atlas of spectreatlas.py), with the line p = 1/2 and
the Aitken extrapolation of the three depths.  Right: X-charge per tile, in units of the Mystic frequency g/(1+g): the Mystic carries 14 and the normal tiles carry
4 (Gamma_1, Delta, Sigma, one each), 2 (Lambda) and 2 p (Phi); the stack on the right equals the Mystic's bar only at p = 1/2 (a second stack at p = 0.58, the
depth-4 value, is drawn for contrast).  Everything is taken from spectre_mystic_xbond.py of the Hat and Spectre repository ($SPECTRE_DIR), imported.  Evidence:
the balance is exact given the atlas of charges; the three p values are counted; p = 1/2 is conditional on the atlas, as in Result 'mystic'.

    python3 models/fig_p_half.py                 tests, then the figure
    python3 models/fig_p_half.py --tests-only
    python3 models/fig_p_half.py --no-show --save-dir figures
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _spectre  # noqa: E402

DEPTHS = (3, 4, 5)
CHARGES = {"Gamma1": 4, "Delta": 4, "Sigma": 4, "Lambda": 2}


def data():
    if _spectre.load() is None:
        return None
    import sympy as sp
    import spectre_mystic_xbond as M
    E = M.exact_p()
    ps = M.measured_p(DEPTHS)
    t = {k: float(v) for k, v in E["t"].items()}
    return dict(M=M, E=E, p=ps, aitken=M.aitken(*ps), t=t, sp=sp)


def stack(D, p):
    """X-charge per unit of Mystic frequency carried by each normal species at flavour frequency p."""
    t, tg = D["t"], D["t"]["Gamma"]
    parts = [("$\\Gamma_1$", 4 * t["Gamma"] / tg), ("$\\Delta$", 4 * t["Delta"] / tg), ("$\\Sigma$", 4 * t["Sigma"] / tg),
             ("$\\Lambda$", 2 * t["Lambda"] / tg), ("$\\Phi^2$", 2 * p * t["Phi"] / tg)]
    return parts


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
    sp = D["sp"]
    check("conservation of X-charge gives p = 1/2 exactly", D["E"]["p"] == [sp.Rational(1, 2)], str(D["E"]["p"]))
    check("measured p falls toward 1/2 (depths 3, 4, 5)", all(a > b > 0.5 for a, b in zip(D["p"], D["p"][1:])), ", ".join("%.3f" % x for x in D["p"]))
    check("Aitken extrapolation within 0.02 of 1/2", abs(D["aitken"] - 0.5) < 0.02, "%.3f" % D["aitken"])
    mystic = 14.0
    bal = sum(v for _, v in stack(D, 0.5))
    check("the normal-tile stack at p = 1/2 equals the Mystic's 14 (to 1e-12)", abs(bal - mystic) < 1e-12, "%.12f" % bal)
    check("control: at p = 0.58 it does not (it exceeds 14)", sum(v for _, v in stack(D, 0.58)) > mystic + 0.1, "%.3f" % sum(v for _, v in stack(D, 0.58)))
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.4, 4.5), gridspec_kw=dict(width_ratios=[1, 1.1]))
    ax.plot(DEPTHS, D["p"], "o-", color="#0072B2", lw=1.8, label="measured $p$")
    ax.axhline(0.5, color="#009E73", lw=1.3, ls="--", label=r"$p=1/2$ (charge conservation)")
    ax.plot([6.3], [D["aitken"]], "D", color="#D55E00", ms=7, label="Aitken limit %.3f" % D["aitken"])
    for k, p in zip(DEPTHS, D["p"]):
        ax.text(k + 0.12, p + 0.012, "%.3f" % p, ha="center", fontsize=8.5)
    ax.set_xticks(list(DEPTHS) + [6.3])
    ax.set_xticklabels([str(k) for k in DEPTHS] + ["$\\infty$"])
    ax.set_xlim(2.6, 6.7)
    ax.set_ylim(0.47, 0.72)
    ax.set_xlabel("depth of the Spectre patch")
    ax.set_ylabel(r"$p=\Pr(\Phi^2)$")
    ax.set_title("The flavour frequency falls toward 1/2", fontsize=10.5)
    ax.legend(loc="upper right", fontsize=8.5, frameon=False)
    cols = ["#0072B2", "#56B4E9", "#009E73", "#E69F00", "#CC79A7"]
    bars = [("Mystic\n(14)", [("Mystic", 14.0)], 0), ("normal tiles\n$p=1/2$", stack(D, 0.5), 1), ("normal tiles\n$p=0.58$", stack(D, 0.58), 2)]
    for label, parts, x in bars:
        bottom = 0.0
        for i, (nm, v) in enumerate(parts):
            c = "#D55E00" if nm == "Mystic" else cols[i]
            bx.bar(x, v, 0.6, bottom=bottom, color=c, edgecolor="white", label=nm if x == 1 else None)
            bottom += v
        bx.text(x, bottom + 0.25, "%.2f" % bottom, ha="center", fontsize=9)
    bx.axhline(14, color="0.4", lw=0.9, ls=":")
    bx.set_xticks([0, 1, 2])
    bx.set_xticklabels([b[0] for b in bars], fontsize=9)
    bx.set_ylim(0, 18.5)
    bx.set_ylabel("X-charge per Mystic frequency")
    bx.set_title("Charge balance holds only at $p=1/2$ (given the atlas)", fontsize=10.5)
    bx.legend(loc="upper center", fontsize=8.5, frameon=False, ncol=5, columnspacing=1.0, handlelength=1.2)
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
    figure(D, os.path.join(d, "p_half.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
