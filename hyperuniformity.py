#!/usr/bin/env python3
"""hyperuniformity.py - fluctuations of the orientation field of the Chair44 and the Spectre in windows.

For a weight w_j attached to every tile (an orientation observable with zero mean) the number variance
sigma^2(R) = < (sum of w_j over the tiles in a window of radius R)^2 > over window centres inside the patch
is measured and fitted as R^beta.  For uncorrelated weights beta = d (the volume); a hyperuniform field has
beta < d (beta = d-1 is the 'surface' class).  The exact growth of the sum over a whole supertile in each
channel (orientation_spectra.py) predicts what to expect: |S|^2 ~ R^(2h) with h the channel exponent, so
beta = 2h for windows that are supertiles; this module measures round windows, which need not agree.

Windows are balls (discs) around random tiles that lie at least R_max inside the patch, found from a mask
of the occupied region.  Weights: the Chair44 channels are matrix entries of the irreps (A2, E, T1, T2) of
the tile's frame; for the Spectre cos(m theta) of the tile's orientation, m = 1, 2, 3.

CLI: python3 hyperuniformity.py [--tests-only] [--png figures/chair44_hyperuniformity.png]
"""
import argparse
import math
import sys

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

import chair44_core as C
import chair44_slice as CS
import chair44_compare as CC
import orientation_spectra as OS


# ------------------------------------------------------------------ the data
def chair_data(depth=5):
    """(points (N,3), {channel: weights (N,)}, mask function for the interior)."""
    poses = C.sigma_depth(depth)
    P = np.array([C.vadd(C.frame_apply(f, (1, 1, 1)), t) for f, t in poses], float)
    irr = OS.chair_irreps()
    W = {
        "A2": np.array([irr["A2"](f)[0, 0] for f, _ in poses]),
        "E": np.array([irr["E"](f)[0, 0] for f, _ in poses]),
        "T1": np.array([irr["T1"](f)[0, 2] for f, _ in poses]),
        "T2": np.array([irr["T2"](f)[0, 0] for f, _ in poses]),
    }
    occ, _ = CS.occupancy(depth)
    return P, W, occ


def spectre_data(level=5):
    P, th = CC.spectre_points(level)
    W = {"m=%d" % m: np.cos(m * th) for m in (1, 2, 3)}
    return P, W


# ------------------------------------------------------------------ interior mask
def inner_centres_chair(P, occ, rmax, n, rng):
    dist = ndimage.distance_transform_edt(np.pad(occ, 1))[1:-1, 1:-1, 1:-1]
    ok = np.array([dist[tuple(np.clip(p.astype(int), 0, occ.shape[0] - 1))] >= rmax for p in P])
    idx = np.flatnonzero(ok)
    return P[rng.choice(idx, size=min(n, len(idx)), replace=False)] if len(idx) else P[:0]


def inner_centres_spectre(P, rmax, n, rng, pix=3.0):
    lo = P.min(axis=0) - 2 * pix
    hi = P.max(axis=0) + 2 * pix
    shape = np.ceil((hi - lo) / pix).astype(int)
    grid = np.zeros(shape, bool)
    ij = ((P - lo) / pix).astype(int)
    grid[ij[:, 0], ij[:, 1]] = True
    grid = ndimage.binary_closing(ndimage.binary_dilation(grid, iterations=2), iterations=2)
    dist = ndimage.distance_transform_edt(grid) * pix
    ok = dist[ij[:, 0], ij[:, 1]] >= rmax
    idx = np.flatnonzero(ok)
    return P[rng.choice(idx, size=min(n, len(idx)), replace=False)] if len(idx) else P[:0]


# ------------------------------------------------------------------ variance
def number_variance(P, w, centres, radii):
    """sigma^2(R) = mean over centres of (sum of w in the ball)^2, for each radius (weights de-meaned)."""
    tree = cKDTree(P)
    w = np.asarray(w, float) - float(np.mean(w))
    out = []
    for r in radii:
        lists = tree.query_ball_point(centres, r)
        s = np.array([w[l].sum() for l in lists])
        out.append(float(np.mean(s ** 2)))
    return np.array(out)


def fit_exponent(radii, var, lo=None, hi=None):
    r = np.asarray(radii, float)
    m = np.ones(len(r), bool)
    if lo is not None:
        m &= r >= lo
    if hi is not None:
        m &= r <= hi
    return float(np.polyfit(np.log(r[m]), np.log(np.asarray(var)[m]), 1)[0])


def bootstrap_slope(P, w, centres, radii, n_boot=60, seed=5, lo=None, hi=None):
    """Slope of log sigma^2 against log R and its bootstrap standard error over windows."""
    tree = cKDTree(P)
    w = np.asarray(w, float) - float(np.mean(w))
    sums = np.array([[w[l].sum() for l in tree.query_ball_point(centres, r)] for r in radii])   # (R, windows)
    rng = np.random.default_rng(seed)
    def slope(idx):
        return fit_exponent(radii, np.mean(sums[:, idx] ** 2, axis=1), lo, hi)
    full = slope(np.arange(sums.shape[1]))
    boots = [slope(rng.integers(0, sums.shape[1], sums.shape[1])) for _ in range(n_boot)]
    return full, float(np.std(boots))


def measure_chair(depth=5, n=600, seed=1, radii=(2, 3, 4, 6, 8, 11, 14)):
    rng = np.random.default_rng(seed)
    P, W, occ = chair_data(depth)
    cen = inner_centres_chair(P, occ, max(radii), n, rng)
    return {k: (np.array(radii, float), number_variance(P, w, cen, radii)) for k, w in W.items()}, len(cen), P


def measure_spectre(level=5, n=600, seed=1, radii=(6, 9, 13, 20, 30, 45, 65)):
    rng = np.random.default_rng(seed)
    P, W = spectre_data(level)
    cen = inner_centres_spectre(P, max(radii), n, rng)
    return {k: (np.array(radii, float), number_variance(P, w, cen, radii)) for k, w in W.items()}, len(cen), P


def measure_all(chair_depth=6, spectre_level=6, n=1500, seed=1):
    """Variance curves and bootstrap slopes for every channel of both tilings.
    Returns {'chair': {ch: (radii, var, slope, err)}, 'spectre': {...}, 'windows': (nc, ns)}."""
    rng = np.random.default_rng(seed)
    out = {"chair": {}, "spectre": {}}
    rc = (3, 5, 8, 12, 18, 26)
    P, W, occ = chair_data(chair_depth)
    cen = inner_centres_chair(P, occ, max(rc), n, rng)
    for k, w in W.items():
        sl, er = bootstrap_slope(P, w, cen, rc)
        out["chair"][k] = (np.array(rc, float), number_variance(P, w, cen, rc), sl, er)
    rs = (6, 9, 13, 20, 30, 45, 65, 95)
    Ps, Ws = spectre_data(spectre_level)
    cens = inner_centres_spectre(Ps, max(rs), n, rng)
    for k, w in Ws.items():
        sl, er = bootstrap_slope(Ps, w, cens, rs)
        out["spectre"][k] = (np.array(rs, float), number_variance(Ps, w, cens, rs), sl, er)
    out["windows"] = (len(cen), len(cens))
    return out


def show_hyperuniformity(res=None, save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    res = res or measure_all()
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
    for a, key, d, ell in ((ax[0], "chair", 3, "Chair44 ($d=3$): $R$ in cells"), (ax[1], "spectre", 2, "Spectre ($d=2$): $R$ in edge lengths")):
        for k, (r, v, sl, er) in res[key].items():
            a.loglog(r, v, "o-", label="%s: slope %.2f" % (k, sl))
        r0 = res[key][next(iter(res[key]))][0]
        v0 = res[key][next(iter(res[key]))][1][0]
        a.loglog(r0, v0 * (r0 / r0[0]) ** d, "k:", label="volume $R^{%d}$" % d)
        a.loglog(r0, v0 * (r0 / r0[0]) ** (d - 1), "k--", label="surface $R^{%d}$" % (d - 1))
        a.set_xlabel(ell)
        a.set_ylabel(r"$\sigma^{2}(R)$ of the orientation weights")
        a.legend(fontsize=8)
    ax[0].set_title("Chair44 channels (irrep matrix entries)", fontsize=10)
    ax[1].set_title("Spectre channels ($\\cos m\\theta$)", fontsize=10)
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=120)
    if show:
        plt.show()
    return fig


# ------------------------------------------------------------------ tests
def test_number_variance_of_independent_weights_scales_as_the_volume():
    rng = np.random.default_rng(2)
    P = rng.uniform(0, 60, (60000, 3))
    w = rng.choice([-1.0, 1.0], len(P))
    cen = rng.uniform(15, 45, (300, 3))
    r = (3.0, 4.5, 6.0, 9.0)
    v = number_variance(P, w, cen, r)
    assert abs(fit_exponent(r, v) - 3.0) < 0.25


def test_count_variance_of_a_square_lattice_is_surface_like_for_continuous_centres():
    g = np.array([(x, y) for x in range(140) for y in range(140)], float)
    rng = np.random.default_rng(1)
    cen = rng.uniform(30, 110, (800, 2))
    r = (4.0, 6.0, 9.0, 13.0, 18.0)
    tree = cKDTree(g)
    v = []
    for rr in r:
        n = np.array([len(l) for l in tree.query_ball_point(cen, rr)], float)
        v.append(float(np.var(n)))
    assert abs(fit_exponent(r, v) - 1.0) < 0.35


def test_chair_weights_have_zero_mean_and_unit_scale():
    P, W, _ = chair_data(3)
    for k, w in W.items():
        assert abs(np.mean(w)) < 0.05 and np.max(np.abs(w)) <= 1.0 + 1e-12, k


def test_interior_centres_are_inside_by_the_margin():
    P, W, occ = chair_data(4)
    cen = inner_centres_chair(P, occ, 4.0, 50, np.random.default_rng(1))
    assert len(cen) == 50
    d = ndimage.distance_transform_edt(np.pad(occ, 1))[1:-1, 1:-1, 1:-1]
    assert all(d[tuple(c.astype(int))] >= 4.0 for c in cen)


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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--png")
    a = ap.parse_args(argv)
    if a.tests_only:
        return 0 if run_tests() else 1
    res = measure_all()
    print("windows (chair, spectre):", res["windows"])
    for key in ("chair", "spectre"):
        for k, (r, v, sl, er) in res[key].items():
            print("  %-7s %-3s slope %.2f +- %.2f over R = %g..%g" % (key, k, sl, er, r[0], r[-1]))
    if a.png:
        show_hyperuniformity(res, a.png, show=False)
        print("wrote", a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
