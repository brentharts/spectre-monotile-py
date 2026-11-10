"""
fig_hex_settings.py - why the six directions of E8 are hexagonal and what that does to CHSH: four settings on the six hexagonal directions.

Left: the six hexagonal directions (the roots of the A2 plane, 60 degrees apart) and one choice of the four settings a, a', b, b' that gives |S| = 4 under the sign
read-out E(a, b) = -sgn(a . b) of the copy-contract toy (found by search over all 6^4 choices, the first in lexicographic order of HDIRS).  Right: the four
correlations E that enter S = E(a,b) + E(a,b') + E(a',b) - E(a',b'), for the toy (sign read-out) and for the singlet, E = -a . b, on the same settings.  The toy
sums to 4 and the singlet to at most 5/2 on the six hexagonal directions; on the six cubic directions both give 2 (Result 'chsh').  The toy is a no-signalling
table, not a quantum model.  Closed form, no external repository.  Evidence label: exact.

    python3 models/fig_hex_settings.py                 tests, then the figure
    python3 models/fig_hex_settings.py --tests-only
    python3 models/fig_hex_settings.py --no-show --save-dir figures
"""
import itertools
import math
import os
import sys

import numpy as np

HDIRS = ("+A", "-A", "+B", "-B", "+C", "-C")
ANG = {"+A": 0, "-A": 180, "+B": 60, "-B": 240, "+C": 120, "-C": 300}
CUBIC = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


def vec(k):
    return np.array([math.cos(math.radians(ANG[k])), math.sin(math.radians(ANG[k]))])


def sgn(x):
    return int((x > 1e-9) - (x < -1e-9))


def toy_E(a, b):
    return -sgn(float(vec(a) @ vec(b)))


def singlet_E(a, b):
    return -float(vec(a) @ vec(b))


def S(E, a, a2, b, b2):
    return E(a, b) + E(a, b2) + E(a2, b) - E(a2, b2)


def best_quadruple():
    for q in itertools.product(HDIRS, repeat=4):
        if abs(S(toy_E, *q)) == 4:
            return q
    return None


def cubic_max():
    best_t = best_s = 0.0
    for q in itertools.product(CUBIC, repeat=4):
        e = lambda a, b: -float(np.sign(np.dot(CUBIC[a], CUBIC[b])))
        f = lambda a, b: -float(np.dot(CUBIC[a], CUBIC[b]))
        best_t = max(best_t, abs(S(e, *q)))
        best_s = max(best_s, abs(S(f, *q)))
    return best_t, best_s


def run_tests(verbose=True):
    ok = True

    def check(name, good, info=""):
        nonlocal ok
        ok &= bool(good)
        if verbose:
            print("  %-4s %-72s %s" % ("ok" if good else "FAIL", name, info))

    q = best_quadruple()
    check("hexagonal toy reaches |S| = 4 on the six directions", q is not None, str(q))
    hexs = max(abs(S(singlet_E, *p)) for p in itertools.product(HDIRS, repeat=4))
    check("hexagonal singlet: largest |S| on the six is 5/2", abs(hexs - 2.5) < 1e-12, "%.6f" % hexs)
    ct, cs = cubic_max()
    check("cubic six: toy and singlet both give 2 (the toy's zero outcomes sit at sgn(0) = 0)", abs(ct - 2) < 1e-12 and abs(cs - 2) < 1e-12, "%.3f, %.3f" % (ct, cs))
    check("the inner products among the six are only +-1/2 and +-1 (no zeros)",
          all(min(abs(abs(float(vec(a) @ vec(b))) - x) for x in (0.5, 1.0)) < 1e-12 for a in HDIRS for b in HDIRS))
    if verbose:
        print("OK" if ok else "FAILED")
    return ok


def figure(path=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    q = best_quadruple()
    names = ("a", "a'", "b", "b'")
    col = {"a": "#0072B2", "a'": "#56B4E9", "b": "#D55E00", "b'": "#E69F00"}
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(10.2, 4.5), gridspec_kw=dict(width_ratios=[1, 1.05]))
    ax.set_aspect("equal")
    for k in HDIRS:
        v = vec(k)
        ax.annotate("", xy=v, xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color="0.65", lw=1.2))
        ax.text(*(1.17 * v), k, ha="center", va="center", fontsize=9, color="0.35")
    used = {}
    for n, k in zip(names, q):
        used.setdefault(k, []).append(n)
    for k, ns in used.items():
        v = vec(k)
        ax.annotate("", xy=v, xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=col[ns[0]], lw=2.6))
        ax.text(*(0.58 * v + 0.10 * np.array([-v[1], v[0]])), ", ".join(ns), ha="center", va="center", fontsize=10.5, fontweight="bold", color=col[ns[0]])
    t = np.linspace(0, 2 * np.pi, 200)
    ax.plot(np.cos(t), np.sin(t), color="0.88", lw=0.8)
    ax.set_xlim(-1.4, 1.4)
    ax.set_ylim(-1.4, 1.4)
    ax.axis("off")
    ax.set_title("Six hexagonal directions, four settings", fontsize=10.5)
    E = [("E(a,b)", q[0], q[2], +1), ("E(a,b')", q[0], q[3], +1), ("E(a',b)", q[1], q[2], +1), ("-E(a',b')", q[1], q[3], -1)]
    x = np.arange(4)
    tv = [s * toy_E(a, b) for _, a, b, s in E]
    sv = [s * singlet_E(a, b) for _, a, b, s in E]
    bx.bar(x - 0.19, tv, 0.36, color="#0072B2", label="toy (sign read-out)")
    bx.bar(x + 0.19, sv, 0.36, color="#999999", label=r"singlet $-\,a\cdot b$")
    bx.set_xticks(x)
    bx.set_xticklabels([e[0] for e in E])
    bx.axhline(0, color="0.3", lw=0.7)
    bx.set_ylim(-1.1, 1.45)
    bx.text(0.02, 0.97, "toy: $S=%.0f$     singlet: $S=%.1f$  (signs aside, $|S|=4$ and $2.5$)" % (sum(tv), sum(sv)), transform=bx.transAxes, va="top", fontsize=10.5)
    bx.set_ylabel("signed term in $S$")
    bx.legend(loc="upper right", bbox_to_anchor=(1.0, 0.86), fontsize=8.5, frameon=False)
    bx.set_title("The four terms of $S$ (exact)", fontsize=10.5)
    for s in ("top", "right"):
        bx.spines[s].set_visible(False)
    fig.tight_layout()
    if path:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        fig.savefig(path, dpi=150)
    if show:
        plt.show()
    plt.close(fig)


def main(argv):
    if not run_tests():
        return 1
    if "--tests-only" in argv:
        return 0
    d = argv[argv.index("--save-dir") + 1] if "--save-dir" in argv else None
    figure(os.path.join(d, "hex_settings.png") if d else None, "--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
