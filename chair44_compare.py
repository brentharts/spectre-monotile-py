#!/usr/bin/env python3
"""chair44_compare.py - the Spectre and the Chair44 hierarchy on the same footing.

Both are substitution tilings with a linear inflation factor: the Spectre sqrt(4+sqrt(15)) = 2.8062
(area factor 4+sqrt(15) = 7.873 per level, d = 2), the Chair44 exactly 2 (volume factor 8, d = 3).
Each tile is reduced to a point (its centroid; for the chair the centre of its bounding cube) and an
orientation vector (e^{i theta} for the Spectre, the notch direction g(1,1,1) for the chair).  Then
the same three questions are asked of both:

  1. twist and mirror of the inflation (exact; Fact 11 and chair44_twist in the paper repo)
  2. how fast does the orientation sum S_n grow?  Both give |S_{n+1}|/|S_n| -> the linear inflation
     factor: exactly 2 for the chair (S_n = 2^n (1,1,1)), 2.8062 for the Spectre (numerically, S_n = 1.4489 lambda^n to 4 digits).
     Hence I(0)/N = |S|^2/N ~ lambda^((2-d) n): constant (diffuse-like) for d = 2, 2^-n for d = 3.
  3. where does the power go?  Median of I(k)/N over fixed random wave vectors, with k scaled by the
     tile size l = (tile volume)^(1/d), against N.  A falling median means sparse peaks.

Honest limits: this compares point sets from two different dimensions; the random-k median is a loose
statistic with sampling noise; none of this proves that either diffraction measure is pure point.

CLI: python3 chair44_compare.py [--tests-only] [--png figures/chair44_compare.png]
"""
import argparse
import math
import sys

import numpy as np

import chair44_slice as CS
import spectre_operator as so
import spectre_patch as sp

LAMBDA = math.sqrt(4.0 + math.sqrt(15.0))        # Spectre linear inflation factor
SPECTRE_AREA = so.spectre_area(1.0, 1.0)
ELL_SPECTRE = math.sqrt(SPECTRE_AREA)
ELL_CHAIR = 7.0 ** (1.0 / 3.0)


def tile_centroid(a=1.0, b=1.0):
    """Area centroid of the straight Spectre polygon (a, b)."""
    v = so.vertices(a, b)
    x, y = v[:, 0], v[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    A = cr.sum() / 2.0
    return np.array([((x + x1) * cr).sum() / (6 * A), ((y + y1) * cr).sum() / (6 * A)])


def spectre_points(level):
    """(centroids (N,2), angles in radians (N,)) of the level-n Spectre patch (all proper rotations)."""
    p = sp.SpectrePatch(level)
    c = tile_centroid()
    P = (p.placements @ np.array([c[0], c[1], 1.0]))[:, :2]
    return P, np.radians(p.angles())


def orientation_sums(levels):
    """Rows (level, N, |S_n|, |S_n|/lambda^n, |S_n|^2/N) for both tilings.
    Spectre: S = sum e^{i theta}.  Chair: S = z component of the sum of the notch vectors."""
    rows = {"spectre": [], "chair": []}
    for n in levels:
        _, th = spectre_points(n)
        S = abs(np.exp(1j * th).sum())
        rows["spectre"].append((n, len(th), S, S / LAMBDA ** n, S ** 2 / len(th)))
        N = CS.notches(n)
        Sc = float(N.sum(axis=0)[2])
        rows["chair"].append((n, len(N), Sc, Sc / 2.0 ** n, Sc ** 2 / len(N)))
    return rows


def random_k_median(points, weights, ell, n_k=300, seed=1):
    """(median, mean, max) of I(k)/N at n_k random k in the box [0, 2 pi / ell]^d."""
    d = points.shape[1]
    K = np.random.default_rng(seed).uniform(0, 2 * np.pi / ell, (n_k, d))
    v = np.abs(np.exp(1j * (K @ points.T)) @ weights) ** 2 / len(points)
    return float(np.median(v)), float(v.mean()), float(v.max())


def median_exponent(ns, med):
    """slope of log median against log N."""
    return float(np.polyfit(np.log(np.asarray(ns, float)), np.log(np.asarray(med, float)), 1)[0])


def median_table(levels, n_k=300, seed=1):
    out = {}
    for name in ("spectre", "spectre_weighted", "chair", "chair_weighted"):
        ns, med = [], []
        for n in levels:
            if name.startswith("spectre"):
                P, th = spectre_points(n)
                w = np.exp(1j * th) if name.endswith("weighted") else np.ones(len(P))
                ell = ELL_SPECTRE
            else:
                P = CS.centres(n).astype(float)
                w = CS.notches(n)[:, 2].astype(float) if name.endswith("weighted") else np.ones(len(P))
                ell = ELL_CHAIR
            ns.append(len(P))
            med.append(random_k_median(P, w, ell, n_k, seed)[0])
        out[name] = (ns, med, median_exponent(ns, med))
    return out


# ------------------------------------------------------------------ tests
def test_spectre_centroid_is_the_area_centroid_and_lies_inside_the_tile():
    c = tile_centroid()
    v = so.vertices(1.0, 1.0)
    h = 0.01
    xs = np.arange(v[:, 0].min(), v[:, 0].max(), h) + h / 2
    ys = np.arange(v[:, 1].min(), v[:, 1].max(), h) + h / 2
    X, Y = np.meshgrid(xs, ys)
    from matplotlib.path import Path
    inside = Path(v).contains_points(np.column_stack([X.ravel(), Y.ravel()]))
    num = np.array([X.ravel()[inside].mean(), Y.ravel()[inside].mean()])
    assert np.allclose(c, num, atol=0.01), (c, num)
    assert Path(v).contains_point(c)


def test_tile_counts_grow_by_the_area_factor_and_by_eight():
    ns = [sp.SpectrePatch(n).n_tiles for n in (3, 4, 5)]
    assert abs(ns[2] / ns[1] - (4 + math.sqrt(15))) < 0.01
    assert [len(CS.centres(n)) for n in (2, 3)] == [64, 512]
    assert abs(LAMBDA ** 2 - (4 + math.sqrt(15))) < 1e-12


def test_orientation_sum_grows_by_the_linear_inflation_factor_in_both():
    r = orientation_sums((3, 4, 5))
    assert [abs(x[3] - 1.0) < 1e-12 for x in r["chair"]] == [True] * 3          # S_n = 2^n exactly (norm)
    s = [x[2] for x in r["spectre"]]
    assert abs(s[2] / s[1] - LAMBDA) < 0.005 and abs(s[1] / s[0] - LAMBDA) < 0.03
    # chair |S|^2 / N falls 2x a level, the Spectre value is constant (d = 2 vs d = 3)
    c2 = [x[4] for x in r["chair"]]
    s2 = [x[4] for x in r["spectre"]]
    assert abs(c2[1] / c2[0] - 0.5) < 1e-12
    assert abs(s2[2] / s2[1] - 1.0) < 0.01


def test_random_k_median_falls_for_both_and_faster_for_the_chair_unweighted():
    t = median_table((2, 3, 4), n_k=200)
    for name in t:
        assert t[name][1][0] > t[name][1][-1], name
    assert t["chair"][2] < t["spectre"][2] < 0


def test_orientation_weighting_changes_the_median():
    t = median_table((3, 4), n_k=200)
    assert t["spectre_weighted"][1][-1] > t["spectre"][1][-1]
    assert t["chair_weighted"][1][-1] > t["chair"][1][-1]


def run_tests(verbose=True):
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-74s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# ------------------------------------------------------------------ figure
def _map_spectre(P, M=161, kmax=2.0):
    ks = np.linspace(-kmax, kmax, M)
    KX, KY = np.meshgrid(ks, ks)
    K = np.column_stack([KX.ravel(), KY.ravel()])
    I = np.zeros(len(K))
    for i in range(0, len(K), 4000):
        I[i:i + 4000] = np.abs(np.exp(1j * (K[i:i + 4000] @ P.T)).sum(axis=1)) ** 2 / len(P)
    return ks, I.reshape(M, M)


def show_compare(levels=(2, 3, 4, 5), save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm
    fig, ax = plt.subplots(2, 3, figsize=(16, 10))
    rows = orientation_sums(levels)
    n = np.array(levels)
    ax[0, 0].plot(n, [r[3] for r in rows["spectre"]], "o-", label="Spectre  |sum e^(i theta)| / 2.8062^n")
    ax[0, 0].plot(n, [r[3] for r in rows["chair"]], "s-", label="Chair44  |sum notch| / 2^n (= 1 exactly)")
    ax[0, 0].set_ylim(0, 2)
    ax[0, 0].set_xlabel("level n")
    ax[0, 0].set_title("orientation sum grows like the linear inflation factor", fontsize=9)
    ax[0, 0].legend(fontsize=8)
    ax[0, 1].semilogy(n, [r[4] for r in rows["spectre"]], "o-", label="Spectre (d = 2): constant")
    ax[0, 1].semilogy(n, [r[4] for r in rows["chair"]], "s-", label="Chair44 (d = 3): 2^-n")
    ax[0, 1].set_xlabel("level n")
    ax[0, 1].set_title("|S|^2 / N: I(0)/N for the orientation weights", fontsize=9)
    ax[0, 1].legend(fontsize=8)
    t = median_table(levels)
    for name, st in (("spectre", "C0-"), ("spectre_weighted", "C0--"), ("chair", "C1-"), ("chair_weighted", "C1--")):
        ns, med, ex = t[name]
        ax[0, 2].loglog(ns, med, st, marker="o", label="%s  slope %.2f" % (name, ex))
    ax[0, 2].set_xlabel("number of tiles N")
    ax[0, 2].set_ylabel("median I(k)/N, 300 random k")
    ax[0, 2].set_title("power concentrates in sparse peaks in both", fontsize=9)
    ax[0, 2].legend(fontsize=7)
    P, _ = spectre_points(4)
    ks, I = _map_spectre(P)
    ax[1, 0].imshow(np.maximum(I, 1e-2), origin="lower", cmap="magma",
                    norm=LogNorm(1e-2, I.max()), extent=(-2, 2, -2, 2))
    ax[1, 0].set_title("Spectre centroids, level 4 (%d): log I/N" % len(P), fontsize=9)
    Pc = CS.centres(5)
    Ic = np.fft.fftshift(CS.intensity(Pc, None, 512))
    ax[1, 1].imshow(np.maximum(Ic, 1e-3).T, origin="lower", cmap="magma", norm=LogNorm(1e-3, Ic.max()),
                    extent=(-np.pi, np.pi, -np.pi, np.pi))
    ax[1, 1].set_title("Chair44 centres, level 5 projected along z (%d): log I/N" % len(Pc), fontsize=9)
    ax[1, 2].axis("off")
    txt = ("                       Spectre              Chair44\n"
           "dimension d            2                    3\n"
           "linear inflation       2.8062 (irr.)        2 (exact)\n"
           "tile-count factor      7.873                8\n"
           "twist of supertile     11.565 deg, mirror   0, no mirror\n"
           "orientations           12 (30 deg steps)    24 (rotations)\n"
           "supertile outline D    1.399                2 (flat)\n"
           "orientation sum        1.4489 * 2.8062^n    2^n exactly\n"
           "|S|^2/N                constant             2^-n\n"
           "pure point?            not decided          not decided")
    ax[1, 2].text(0, 0.95, txt, family="monospace", fontsize=9, va="top")
    fig.suptitle("Spectre against Chair44, same questions asked of both", fontsize=12)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=120)
    if show:
        plt.show()
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--png")
    a = ap.parse_args(argv)
    if a.tests_only:
        return 0 if run_tests() else 1
    lv = (2, 3, 4, 5)
    r = orientation_sums(lv)
    for k in r:
        print(k)
        for row in r[k]:
            print("  n=%d N=%-6d |S|=%-9.4g |S|/lam^n=%-7.4f |S|^2/N=%.4f" % row)
    for k, (ns, med, ex) in median_table(lv).items():
        print("%-18s median %s slope %.2f" % (k, ["%.3g" % m for m in med], ex))
    if a.png:
        show_compare(lv, a.png, show=False)
        print("wrote", a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
