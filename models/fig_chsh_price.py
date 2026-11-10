"""
fig_chsh_price.py - the price of a Bell violation: largest CHSH value under no signalling against the retro-dependence delta.

Closed forms (Result 'price' of quantgrav.tex): hexagonal set  S_max = min(4, 2 + 12 delta);  cubic set  S_max = min(4, 4/3 + 8 delta),
threshold delta > 1/12 for the cubic set to pass 2.  Where the Quantum-Gravity-Dynamics fork is found ($QGD_DIR), the linear programmes of
`hex_directions.scan_hex` (six read-outs) and `retro_boundary_model.scan` (26 cubic read-outs) are solved at a grid of delta and drawn as points
on the lines; the tests demand agreement to 1e-6.  Evidence label: numeric (a linear programme over the chosen setting families), dagger-free
here because the closed forms are recomputed, not copied.

    python3 models/fig_chsh_price.py                 tests, then the figure
    python3 models/fig_chsh_price.py --tests-only
    python3 models/fig_chsh_price.py --no-show --save-dir figures
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _qgd  # noqa: E402

TSIRELSON = 2 * math.sqrt(2)
DELTAS_LP = [0.0, 0.03, 0.06, 1 / 12, 0.1, 0.12, 0.15, 0.2, 0.25]


def hex_closed(d):
    return np.minimum(4.0, 2.0 + 12.0 * np.asarray(d))


def cubic_closed(d):
    return np.minimum(4.0, 4.0 / 3.0 + 8.0 * np.asarray(d))


def lp_points():
    """{'hex': [...], 'cubic': [...]} solved by the fork's linear programmes, or None."""
    if _qgd.load() is None:
        return None
    import hex_directions as H
    import retro_boundary_model as R
    h = H.scan_hex(DELTAS_LP, readouts=H.SIX_V)
    c = R.scan(DELTAS_LP)
    return {"hex": [h[d] for d in DELTAS_LP], "cubic": [c[d] for d in DELTAS_LP]}


def run_tests(pts=None, verbose=True):
    ok = True

    def check(name, good, info=""):
        nonlocal ok
        ok &= bool(good)
        if verbose:
            print("  %-4s %-72s %s" % ("ok" if good else "FAIL", name, info))

    check("hex: local value 2 at delta = 0, quantum value 2 sqrt2 at delta = (2 sqrt2 - 2)/12", abs(hex_closed(0) - 2) < 1e-12 and abs(hex_closed((TSIRELSON - 2) / 12) - TSIRELSON) < 1e-12,
          "delta = %.4f" % ((TSIRELSON - 2) / 12))
    check("hex: reaches 4 at delta = 1/6", abs(hex_closed(1 / 6) - 4) < 1e-12)
    check("cubic: local value 4/3 below the threshold; passes 2 exactly at delta = 1/12", abs(cubic_closed(0) - 4 / 3) < 1e-12 and abs(cubic_closed(1 / 12) - 2) < 1e-12)
    check("cubic: 2 sqrt2 at delta = (2 sqrt2 - 4/3)/8, 4 at delta = 1/3", abs(cubic_closed((TSIRELSON - 4 / 3) / 8) - TSIRELSON) < 1e-12 and abs(cubic_closed(1 / 3) - 4) < 1e-12,
          "delta = %.4f" % ((TSIRELSON - 4 / 3) / 8))
    if pts is None:
        check("fork not found: linear-programme points skipped", True, "set QGD_DIR")
    else:
        eh = max(abs(a - b) for a, b in zip(pts["hex"], hex_closed(DELTAS_LP)))
        ec = max(abs(a - b) for a, b in zip(pts["cubic"], cubic_closed(DELTAS_LP)))
        check("linear programme (six hexagonal read-outs) equals min(4, 2 + 12 delta)", eh < 1e-6, "max |difference| = %.1e" % eh)
        check("linear programme (26 cubic read-outs) equals min(4, 4/3 + 8 delta)", ec < 1e-6, "max |difference| = %.1e" % ec)
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(path=None, show=True, pts=None):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    d = np.linspace(0, 0.36, 400)
    ax.plot(d, hex_closed(d), color="#0072B2", lw=2, label=r"hexagonal: $\min(4,\,2+12\delta)$")
    ax.plot(d, cubic_closed(d), color="#D55E00", lw=2, ls="--", label=r"cubic: $\min(4,\,4/3+8\delta)$")
    if pts is not None:
        ax.plot(DELTAS_LP, pts["hex"], "o", color="#0072B2", ms=5, mfc="white", label="linear programme (hexagonal)")
        ax.plot(DELTAS_LP, pts["cubic"], "s", color="#D55E00", ms=5, mfc="white", label="linear programme (cubic)")
    for y, t, c in ((2, r"local bound $2$", "0.35"), (TSIRELSON, r"quantum $2\sqrt{2}$", "0.35"), (4, r"no-signalling $4$", "0.35")):
        ax.axhline(y, color=c, lw=0.9, ls=":")
        ax.text(0.362, y + 0.04, t, ha="right", va="bottom", fontsize=8.5, color="0.25")
    ax.axvline(1 / 12, color="#D55E00", lw=0.8, ls=":")
    ax.text(1 / 12 + 0.003, 3.45, r"cubic threshold $\delta=1/12$", fontsize=8.5, color="#D55E00", rotation=90, va="bottom")
    ax.set_xlim(0, 0.365)
    ax.set_ylim(1.2, 4.3)
    ax.set_xlabel(r"$\delta$: how far a hidden-state probability may move from uniform when it depends on both settings")
    ax.set_ylabel("largest CHSH value $S$, no signalling")
    ax.set_title("The price of a Bell violation (numeric)", fontsize=10.5)
    ax.legend(loc="lower right", ncol=2, fontsize=8, frameon=False, columnspacing=1.0)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    pts = lp_points() if "--skip-lp" not in argv else None
    if not run_tests(pts):
        return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(os.path.join(d, "chsh_price.png") if d else None, "--no-show" not in argv, pts)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
