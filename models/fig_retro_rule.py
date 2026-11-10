"""
fig_retro_rule.py - the one-parameter rule that pays the price of retro-dependence (Result 'rule' of quantgrav.tex), and when it signals.

Rule (directional/retro_rule.py of the Quantum-Gravity-Dynamics fork, imported, not copied; $QGD_DIR): P(v | a, b) proportional to exp(kappa A B), A and B
the sign read-outs of the two settings at the origin direction v among the six hexagonal directions.  Left: the best CHSH value over the settings among the six
against kappa, with the linear-programme bound min(4, 2 + 12 delta(kappa)), delta(kappa) the largest probability shift the rule uses, and the local, quantum and
no-signalling values.  Right: the largest change of one party's outcome distribution when the other party's setting changes ('signalling') at kappa = 1 for four
cases: the rule on the six read-outs (zero), the rule on the twelve (zero outcomes possible), the 'align' weight on the twelve, and a control that breaks the
symmetry v -> -v.  The weight is chosen, not derived, and no-signalling on the six is a symmetry property.  Evidence label: exact checks, numeric curve.

    python3 models/fig_retro_rule.py                 tests, then the figure
    python3 models/fig_retro_rule.py --tests-only
    python3 models/fig_retro_rule.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _qgd  # noqa: E402

TSIRELSON = 2 * math.sqrt(2)
KAPPAS = np.concatenate([np.linspace(0, 1, 21), np.linspace(1.25, 3, 8)])


def compute():
    """-> dict with the curve and the signalling bars, or None if the fork is not found."""
    if _qgd.load() is None:
        return None
    import retro_rule as RR
    six, twelve = list(RR.V), list(RR.TWELVE)
    S, dl = [], []
    for k in KAPPAS:
        P = RR.rule("AB", float(k))
        S.append(abs(RR.best_chsh(P, six)[0]))
        dl.append(RR.delta_used(P, six))
    sig = {
        "AB, six": RR.signalling(RR.rule("AB", 1.0), six),
        "AB, twelve": RR.signalling(RR.rule("AB", 1.0), twelve),
        "align, twelve": RR.signalling(RR.rule("align", 1.0), twelve),
        "odd control, six": RR.signalling(RR.rule("oddcontrol", 1.0), six),
    }
    return dict(S=np.array(S), delta=np.array(dl), sig=sig, kappa_q=RR.kappa_for(TSIRELSON), RR=RR)


def run_tests(D=None, verbose=True):
    ok = True

    def check(name, good, info=""):
        nonlocal ok
        ok &= bool(good)
        if verbose:
            print("  %-4s %-72s %s" % ("ok" if good else "FAIL", name, info))

    if D is None:
        check("fork not found: nothing to test", False, "set QGD_DIR")
        return False
    S, k = D["S"], list(KAPPAS)
    at = lambda x: S[k.index(min(k, key=lambda y: abs(y - x)))]
    check("kappa = 0 is the local model, S = 2", abs(S[0] - 2) < 1e-9, "%.6f" % S[0])
    check("S rises with kappa and stays below 4", all(b >= a - 1e-9 for a, b in zip(S, S[1:])) and S.max() < 4, "S(2) = %.3f" % at(2.0))
    check("S(1) = 3.62 to 0.01 (ledger value)", abs(at(1.0) - 3.62) < 0.01, "%.3f" % at(1.0))
    check("2 sqrt2 reached at kappa = 0.362 to 0.001", abs(D["kappa_q"] - 0.362) < 1e-3, "%.4f" % D["kappa_q"])
    check("every S lies below the LP bound min(4, 2 + 12 delta)", all(s <= min(4, 2 + 12 * d) + 1e-9 for s, d in zip(S, D["delta"])))
    sg = D["sig"]
    check("six read-outs: no signalling (symmetry v -> -v)", sg["AB, six"] < 1e-9, "%.2e" % sg["AB, six"])
    check("control that breaks v -> -v signals", sg["odd control, six"] > 0.01, "%.3f" % sg["odd control, six"])
    check("twelve read-outs signal (zero outcome)", sg["AB, twelve"] > 0.01 and sg["align, twelve"] > 0.01, "%.3f, %.3f" % (sg["AB, twelve"], sg["align, twelve"]))
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(D, path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.5, 4.3), gridspec_kw=dict(width_ratios=[1.55, 1]))
    ax.plot(KAPPAS, D["S"], "o-", color="#0072B2", ms=3.5, lw=1.8, label="best $S$ on the six directions (rule)")
    ax.plot(KAPPAS, np.minimum(4, 2 + 12 * D["delta"]), color="#009E73", lw=1.4, ls="--", label=r"LP bound $\min(4,\,2+12\delta(\kappa))$")
    for y, t in ((2, r"local $2$"), (TSIRELSON, r"quantum $2\sqrt{2}$"), (4, r"no-signalling $4$")):
        ax.axhline(y, color="0.4", lw=0.9, ls=":")
        ax.text(3.0, y + 0.03, t, ha="right", va="bottom", fontsize=8.5, color="0.25")
    ax.axvline(D["kappa_q"], color="#D55E00", lw=0.9, ls=":")
    ax.annotate(r"$\kappa=%.3f$: $S=2\sqrt{2}$" % D["kappa_q"], (D["kappa_q"], TSIRELSON), xytext=(0.62, 2.35), fontsize=8.5, color="#D55E00",
                arrowprops=dict(arrowstyle="-", color="#D55E00", lw=0.8))
    ax.set_xlim(0, 3.02)
    ax.set_ylim(1.8, 4.2)
    ax.set_xlabel(r"$\kappa$ (coupling between the two ends of the history)")
    ax.set_ylabel("CHSH value $S$")
    ax.set_title("A chosen rule pays the price", fontsize=10.5)
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 0.2), fontsize=8.5, frameon=False)
    names = list(D["sig"])
    vals = [D["sig"][n] for n in names]
    cols = ["#0072B2", "#E69F00", "#E69F00", "#CC79A7"]
    bx.barh(range(len(names)), vals, color=cols)
    bx.set_yticks(range(len(names)))
    bx.set_yticklabels(names, fontsize=9)
    bx.invert_yaxis()
    for i, v in enumerate(vals):
        bx.text(v + 0.005, i, "0" if v < 1e-9 else "%.3f" % v, va="center", fontsize=8.5)
    bx.set_xlim(0, max(vals) * 1.25)
    bx.set_xlabel(r"largest change of one party's outcome law ($\kappa=1$)")
    bx.set_title("Signalling: zero only through $v\\to-v$", fontsize=10.5)
    for a_ in (ax, bx):
        for s in ("top", "right"):
            a_.spines[s].set_visible(False)
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    D = compute()
    if not run_tests(D):
        return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(D, os.path.join(d, "retro_rule.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
