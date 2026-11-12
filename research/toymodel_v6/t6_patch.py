#!/usr/bin/env python3
"""t6_patch.py - Stage 1 of toymodel_v6: the bump-and-dent pairs of a real Spectre patch as a transverse-field Ising model, with the coupling taken from the geometry.

Stage 0 showed that in the ledger sector each pair is one qubit tau (tau^z = +1: the first tile of the pair is the bump).  Here the pairs are those of toymodel_v4 (`t4_patch.flat_pairs`) and the coupling
between pairs is not chosen: it is the Coulomb energy of the bits read as charges, as in toymodel_v5 Stage 3 (a bit is a source of strength h at its tile).  A pair in state tau carries +h tau at the
centroid of its first tile and -h tau at the centroid of its second: a dipole.  Tiles outside pairs cannot open (v4) and sit at the flat state, charge 0.

Definitions
  D6.5  Charges.  Tile t of pair A carries q = +-h(tau) tau_A at its centroid c_t (straight outline, tile parameters (1, b(tau)), b = 1 + (sqrt3 - 1) tau).  Unpaired tiles carry 0.
  D6.6  Coupling.  H_K = kappa sum_{A<B} U_AB with U the Coulomb energy of the four charges; then H_K = kappa h^2 sum W_AB tau^z_A tau^z_B with
        W_AB = 1/r(A1,B1) - 1/r(A1,B2) - 1/r(A2,B1) + 1/r(A2,B2).  kappa is fixed once, so that the largest |kappa h(0)^2 W| at tau = 0 is K0 = 1.
  D6.7  H = - Gamma(tau) sum_A tau^x_A + H_K,  Gamma = J0 c(tau) as in Stage 0 (D6.3), J0 = 1.  Visibility of a read-out 1 - c^2 (D6.4).
  D6.8  Cluster.  The 14 pairs of the level-3 patch nearest its centre (2^14 states), and the 6 pairs of the level-2 patch, each with open boundary.

Exact / Numeric claims (each a test; ``python3 t6_patch.py --tests-only``)
  E6.6  The pair qubit is defined on the patch.  The flat pairs are disjoint (6, 82, 804 pairs at levels 2, 3, 4, no tile in two) and the same list for every b in [1, sqrt3]; at each b the centroid
        separation inside a pair is the same for every pair (to 1e-9 on levels 2 and 3): 3 at the Spectre, sqrt 21 at the Hat.
  E6.7  The coupling is an exact Ising term with both signs, falling as r^-3.  For each pair of pairs the four sign combinations of the charges give U = h^2 W tau_A tau_B to 1e-14.  At level 4 the
        fitted slope of |W| against the distance of the pair midpoints (beyond 15) is -2.99 (dipole-dipole); the fraction of positive (antiferromagnetic) couplings is 1/3 at level 2 and 0.49 / 0.50 at levels 3 / 4.
  E6.8  The patch is frustrated.  The classical ground state (Gamma = 0, brute force) leaves a fraction f of the bond weight unsatisfied: 0.060 for the 6 pairs of level 2; 0.128, 0.136, 0.150, 0.156, 0.165
        for the central 6, 8, 10, 12, 14 pairs of level 3 (f = 0 would mean a ferromagnet up to a gauge).
  E6.9  The frozen end is a cat, not an entangled pair.  As Gamma -> 0 (Gamma = 1e-6, couplings of tau = 0) the even ground state of each cluster is the cat (|s> + |-s>)/sqrt2 of its classical ground
        state s (overlap 1 - 1e-10) and every pairwise concurrence is below 1e-10.  For two pairs the cat is a Bell state, which is why Stage 0 found the most entanglement at the Spectre end: that is a
        property of n = 2.  **Negative** for E6.5 of Stage 0 as a statement about a patch.
  E6.10 Along the flow (tau in [0, 0.4], clusters of 6 to 14 pairs): the largest pairwise concurrence peaks at 0.38 to 0.65 at an intermediate flow time (0.05 to 0.13, depending on the cluster), below the
        two-pair value 0.913 of the Spectre end; the largest CHSH of a reduced two-pair state is 2.33 (10 pairs, tau = 0.05), and with the visibility 1 - c^2 of Stage 0 at most 2.12, above 2 only for the
        10-pair cluster.  The location of the gap minimum depends on the cluster (tau = 0 to 0.11): no transition is claimed.
  E6.11 The Hat end is a paramagnet: the even-sector gap tends to 4 Gamma = 4, the concurrence to 0 and the CHSH of every reduced state to 2 from above.

Status.  E6.6, E6.7: Exact (geometry, algebra).  E6.8 to E6.11: exact diagonalisation of the stated finite clusters (Numeric only in the choice of cluster).
That a bit is a charge at the tile centroid is v5's Definition; the forms of Gamma and the visibility are Stage 0's.  Nothing here is physics.

    python3 t6_patch.py --tests-only
    python3 t6_patch.py --no-show --save figures/t6_stage1_patch.png
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np
import scipy.sparse as spm
import scipy.sparse.linalg as sla

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_RES)
for _p in (_HERE, os.path.join(_RES, "toymodel_v4"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t4_patch as tp  # noqa: E402
import t6_pair as t0  # noqa: E402

SQRT3 = math.sqrt(3.0)
N_CLUSTER = 14
_CACHE = {}


def b_of(tau):
    return 1.0 + (SQRT3 - 1.0) * tau


def centroid(poly):
    poly = np.asarray(poly, float)
    x, y = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    A = cr.sum() / 2.0
    return np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (6.0 * A)


def geometry(level, b):
    key = (level, round(b, 12))
    if key not in _CACHE:
        P = tp.patch(level, b)
        pr = tp.flat_pairs(level, b)
        C = np.array([centroid(P.straight_polygon(i)) for i in range(P.n_tiles)])
        _CACHE[key] = (pr, C, P)
    return _CACHE[key]


def coupling(level, b, idx=None):
    """W (n x n, zero diagonal) and the pair midpoints, for the pairs idx (default all)."""
    pr, C, _ = geometry(level, b)
    pr = [pr[i] for i in idx] if idx is not None else pr
    A = C[[p[0] for p in pr]]
    B = C[[p[1] for p in pr]]

    def inv(X, Y):
        d = np.linalg.norm(X[:, None] - Y[None], axis=2)
        with np.errstate(divide="ignore"):
            out = 1.0 / d
        out[~np.isfinite(out)] = 0.0
        return out

    W = inv(A, A) - inv(A, B) - inv(B, A) + inv(B, B)
    np.fill_diagonal(W, 0.0)
    return W, 0.5 * (A + B)


def cluster(level=3, n=N_CLUSTER):
    """Indices of the n pairs nearest the centre of the patch (fixed at b = 1; the list is the same for every b)."""
    pr, C, _ = geometry(level, 1.0)
    mid = np.array([0.5 * (C[a] + C[b]) for a, b in pr])
    centre = C.mean(axis=0)
    return list(np.argsort(np.linalg.norm(mid - centre, axis=1), kind="stable")[:n])


KAPPA = None


def kappa():
    global KAPPA
    if KAPPA is None:
        W, _ = coupling(2, 1.0)
        KAPPA = 1.0 / (t0.h(0.0) ** 2 * np.abs(W).max())
    return KAPPA


def couplings_at(tau, level, idx=None):
    W, mid = coupling(level, b_of(tau), idx)
    return kappa() * t0.h(tau) ** 2 * W, mid


def classical_ground(K):
    n = K.shape[0]
    iu = np.triu_indices(n, 1)
    best, arg = None, None
    for s in itertools.product((1, -1), repeat=n - 1):
        s = np.array((1,) + s, float)
        e = float((K[iu] * s[iu[0]] * s[iu[1]]).sum())
        if best is None or e < best:
            best, arg = e, s
    tot = float(np.abs(K[iu]).sum())
    return best, arg, (best + tot) / (2.0 * tot)        # unsatisfied fraction f: E = S - 2U, S = total


def hamiltonian(Gamma, K):
    """Sparse H = -Gamma sum tau^x + sum_{A<B} K_AB tau^z tau^z on n qubits (bit A of the index = qubit A, 0 <-> tau^z = +1)."""
    n = K.shape[0]
    dim = 1 << n
    idx = np.arange(dim)
    bits = ((idx[:, None] >> np.arange(n)[None]) & 1).astype(float)
    s = 1.0 - 2.0 * bits
    iu = np.triu_indices(n, 1)
    diag = ((s[:, iu[0]] * s[:, iu[1]]) * K[iu]).sum(axis=1)
    rows, cols = [idx], [idx]
    vals = [diag]
    for a in range(n):
        rows.append(idx); cols.append(idx ^ (1 << a)); vals.append(np.full(dim, -Gamma))
    return spm.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(dim, dim))


def parity_even_ground(Gamma, K):
    """Ground state and gap within the sector of even global tau^x parity (the Z2 of H)."""
    n = K.shape[0]
    H = hamiltonian(Gamma, K)
    dim = 1 << n
    # basis of the even sector: (|x> + |x ^ all>)/sqrt2 for x with bit n-1 = 0
    full = dim - 1
    xs = np.arange(dim // 2)
    Pm = spm.csr_matrix((np.full(dim, 1 / math.sqrt(2)), (np.concatenate([xs, xs ^ full]), np.concatenate([xs, xs]))), shape=(dim, dim // 2))
    He = (Pm.T @ H @ Pm).tocsc()
    if He.shape[0] <= 256:
        w, v = np.linalg.eigh(He.toarray())
    else:
        w, v = sla.eigsh(He, k=3, which="SA", tol=1e-12)
        o = np.argsort(w); w, v = w[o], v[:, o]
    psi = Pm @ v[:, 0]
    return psi / np.linalg.norm(psi), float(w[1] - w[0]), H


def reduced_two(psi, n, a, b):
    t = psi.reshape([2] * n)       # axis k <-> bit n-1-k of the index
    ax_a, ax_b = n - 1 - a, n - 1 - b
    rest = [k for k in range(n) if k not in (ax_a, ax_b)]
    m = np.transpose(t, [ax_a, ax_b] + rest).reshape(4, -1)
    return m @ m.conj().T


def concurrence_rho(rho):
    yy = np.kron(t0.SY, t0.SY)
    R = rho @ yy @ rho.conj() @ yy
    ev = np.sqrt(np.clip(np.sort(np.real(np.linalg.eigvals(R)))[::-1], 0, None))
    return float(max(0.0, ev[0] - ev[1] - ev[2] - ev[3]))


def chsh_rho(rho):
    T = np.array([[float(np.real(np.trace(rho @ np.kron(a, b)))) for b in t0.PAULI] for a in t0.PAULI])
    m = np.sort(np.linalg.eigvalsh(T.T @ T))[::-1]
    return float(2 * math.sqrt(max(m[0] + m[1], 0.0)))


def flow_point(tau, level=3, idx=None):
    idx = cluster(level) if idx is None and level == 3 else idx
    K, mid = couplings_at(tau, level, idx)
    c = t0.overlap(tau)
    psi, gap, _ = parity_even_ground(c, K)
    n = K.shape[0]
    best = (0.0, None, 0.0)
    conc = {}
    for a, b in itertools.combinations(range(n), 2):
        rho = reduced_two(psi, n, a, b)
        C = concurrence_rho(rho)
        conc[(a, b)] = (C, float(np.linalg.norm(mid[a] - mid[b])))
        s = chsh_rho(rho)
        if s > best[2]:
            best = (C, (a, b), s)
    Cmax = max(v[0] for v in conc.values())
    return dict(gap=gap, cmax=Cmax, chsh=best[2], chsh_obs=(1 - c * c) * best[2], conc=conc)


def window_end(level=3, lo=0.0, hi=0.2):
    """(Not used by the tests.)"""
    f = lambda t: flow_point(t, level)["chsh_obs"] - 2.0  # noqa: E731
    if f(lo) <= 0:
        return 0.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) > 0 else (lo, mid)
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
CLUSTERS = (6, 8, 10, 12, 14)
SCAN = np.linspace(0.0, 0.4, 41)
_SCAN = {}


def scan(n):
    if n not in _SCAN:
        idx = cluster(3, n)
        _SCAN[n] = [flow_point(t, 3, idx) for t in SCAN]
    return _SCAN[n]


def test_e66_the_pair_qubit_is_defined_on_the_patch():
    for lv, n in ((2, 6), (3, 82), (4, 804)):
        pr = tp.flat_pairs(lv, 1.0)
        tiles = [t for p in pr for t in p]
        assert len(pr) == n and len(set(tiles)) == len(tiles)
    seps = {}
    for b in (1.0, 1.2, 1.5, SQRT3):
        for lv in (2, 3):
            pr, C, _ = geometry(lv, b)
            assert pr == tp.flat_pairs(lv, 1.0)
            sep = np.array([np.linalg.norm(C[x] - C[y]) for x, y in pr])
            assert sep.max() - sep.min() < 1e-9
            seps[b] = float(sep[0])
    assert abs(seps[1.0] - 3.0) < 1e-9 and abs(seps[SQRT3] - math.sqrt(21.0)) < 1e-9
    return seps


def test_e67_the_coupling_is_ising_with_both_signs_and_dipolar():
    pr, C, _ = geometry(2, 1.0)
    W, _ = coupling(2, 1.0)
    for A, B in itertools.combinations(range(len(pr)), 2):
        for sA, sB in itertools.product((1, -1), repeat=2):
            q = {pr[A][0]: sA, pr[A][1]: -sA, pr[B][0]: sB, pr[B][1]: -sB}
            U = sum(q[i] * q[j] / np.linalg.norm(C[i] - C[j]) for i in (pr[A][0], pr[A][1]) for j in (pr[B][0], pr[B][1]))
            assert abs(U - W[A, B] * sA * sB) < 1e-14
    fr = {}
    for lv in (2, 3, 4):
        W, mid = coupling(lv, 1.0)
        iu = np.triu_indices(W.shape[0], 1)
        fr[lv] = float(np.mean(W[iu] > 0))
        if lv == 4:
            d = np.linalg.norm(mid[:, None] - mid[None], axis=2)[iu]
            far = d > 15
            sl = float(np.polyfit(np.log(d[far]), np.log(np.abs(W[iu][far])), 1)[0])
            assert abs(sl + 3.0) < 0.05, sl
    assert abs(fr[2] - 1 / 3) < 1e-12 and abs(fr[3] - 0.49) < 0.01 and abs(fr[4] - 0.50) < 0.01, fr
    return round(sl, 3), {k: round(v, 3) for k, v in fr.items()}


def test_e68_the_patch_is_frustrated():
    K2, _ = couplings_at(0.0, 2)
    f = [classical_ground(K2)[2]]
    for n in CLUSTERS:
        f.append(classical_ground(couplings_at(0.0, 3, cluster(3, n))[0])[2])
    want = (0.060, 0.128, 0.136, 0.150, 0.156, 0.165)
    assert all(abs(x - y) < 1e-3 for x, y in zip(f, want)), f
    return [round(x, 3) for x in f]


def test_e69_the_frozen_end_is_a_cat():
    worst = 0.0
    for n in CLUSTERS:
        K, _ = couplings_at(0.0, 3, cluster(3, n))
        _, s, _ = classical_ground(K)
        psi, _, _ = parity_even_ground(1e-6, K)
        cat = np.zeros(1 << n)
        x = int(sum(1 << a for a in range(n) if s[a] < 0))
        cat[x] = cat[x ^ ((1 << n) - 1)] = 1 / math.sqrt(2)
        assert abs(abs(cat @ psi) - 1) < 1e-9
        worst = max(worst, max(concurrence_rho(reduced_two(psi, n, a, b)) for a, b in itertools.combinations(range(n), 2)))
    assert worst < 1e-10
    # n = 2: the cat is a Bell state, concurrence 1
    K2 = np.array([[0.0, 1.0], [1.0, 0.0]])
    psi, _, _ = parity_even_ground(1e-6, K2)
    assert abs(concurrence_rho(reduced_two(psi, 2, 0, 1)) - 1) < 1e-9
    return worst


def test_e610_along_the_flow():
    peaks, best, best_obs, over = [], (0.0, None), (0.0, None), []
    for n in CLUSTERS:
        pts = scan(n)
        c = [p["cmax"] for p in pts]
        j = int(np.argmax(c))
        peaks.append((round(c[j], 3), round(float(SCAN[j]), 3)))
        assert 0.37 < c[j] < 0.66 and 0.04 <= SCAN[j] <= 0.14 and c[j] < t0.flow(0.0)[2]
        for t, p in zip(SCAN, pts):
            assert p["chsh"] <= t0.TSIRELSON + 1e-12
            if p["chsh"] > best[0]:
                best = (p["chsh"], (n, round(float(t), 3)))
            if p["chsh_obs"] > best_obs[0]:
                best_obs = (p["chsh_obs"], (n, round(float(t), 3)))
        if max(p["chsh_obs"] for p in pts) > 2:
            over.append(n)
    assert abs(best[0] - 2.325) < 1e-3 and best[1] == (10, 0.05), best
    assert abs(best_obs[0] - 2.121) < 1e-3 and over == [10], (best_obs, over)
    return peaks, (round(best[0], 3), best[1]), (round(best_obs[0], 3), best_obs[1])


def test_e611_the_hat_end_is_a_paramagnet():
    p = flow_point(0.999)
    assert abs(p["gap"] - 4.0 * t0.overlap(0.999)) < 1e-3 and p["cmax"] < 1e-5 and 2.0 <= p["chsh"] < 2.0 + 1e-6
    return round(p["gap"], 4)


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e6")]
    failed = 0
    for n, f in tests:
        try:
            r = f()
            if verbose:
                print("ok   %s  %s" % (n, "" if r is None else r))
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save")
    a = ap.parse_args(argv)
    failed = run_tests()
    if not a.tests_only and not failed:
        figure(a.save, not a.no_show)
    return 1 if failed else 0


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection, PolyCollection

    fig = plt.figure(figsize=(15, 8.6))
    ax0 = fig.add_subplot(2, 3, 1)
    pr, C, P = geometry(3, 1.0)
    polys = [np.asarray(P.straight_polygon(i)) for i in range(P.n_tiles)]
    paired = {t for p in pr for t in p}
    idx = cluster(3, 14)
    incl = {t for i in idx for t in pr[i]}
    col = ["#c9d6ea" if t in incl else ("#efe3c7" if t in paired else "0.93") for t in range(P.n_tiles)]
    ax0.add_collection(PolyCollection(polys, facecolors=col, edgecolors="0.6", linewidths=0.3))
    K, mid = couplings_at(0.0, 3, idx)
    segs, cols, lws = [], [], []
    for a, b in itertools.combinations(range(len(idx)), 2):
        if abs(K[a, b]) > 0.05:
            segs.append([mid[a], mid[b]]); cols.append("tab:red" if K[a, b] > 0 else "tab:blue"); lws.append(3 * abs(K[a, b]))
    ax0.add_collection(LineCollection(segs, colors=cols, linewidths=lws))
    for i in idx:
        a, b = pr[i]
        ax0.annotate("", xy=C[b], xytext=C[a], arrowprops=dict(arrowstyle="->", color="k", lw=0.8))
    ax0.set_xlim(mid[:, 0].min() - 8, mid[:, 0].max() + 8); ax0.set_ylim(mid[:, 1].min() - 8, mid[:, 1].max() + 8)
    ax0.set_aspect("equal"); ax0.set_xticks([]); ax0.set_yticks([])
    ax0.set_title("(a) level 3: the 14 central pairs (blue), other pairs (tan);\narrow bump to dent; red antiferro, blue ferro coupling", fontsize=8)

    ax1 = fig.add_subplot(2, 3, 2)
    W, m4 = coupling(4, 1.0)
    iu = np.triu_indices(W.shape[0], 1)
    d = np.linalg.norm(m4[:, None] - m4[None], axis=2)[iu]
    w = W[iu]
    rng = np.random.default_rng(0)
    pick = rng.choice(len(d), 20000, replace=False)
    ax1.scatter(d[pick], np.abs(w[pick]), s=1, c=np.where(w[pick] > 0, "tab:red", "tab:blue"), linewidths=0)
    ax1.set_xscale("log"); ax1.set_yscale("log")
    ax1.plot([], [], "o", ms=4, color="tab:red", label="W > 0 (antiferro)"); ax1.plot([], [], "o", ms=4, color="tab:blue", label="W < 0 (ferro)")
    dd = np.geomspace(5, 150, 10)
    ax1.loglog(dd, 2 * dd ** -3.0, "k--", label="r$^{-3}$")
    ax1.set_xlabel("distance of the pair midpoints"); ax1.set_ylabel("|W|"); ax1.legend(fontsize=7)
    ax1.set_title("(b) level 4: the bit charges couple pairs as dipoles", fontsize=8)

    ax2 = fig.add_subplot(2, 3, 3)
    two = [t0.flow(t) for t in SCAN]
    for n in CLUSTERS:
        ax2.plot(SCAN, [p["gap"] for p in scan(n)], label="%d pairs" % n)
    ax2.plot(SCAN, [x[5] for x in two], "k--", label="two pairs (Stage 0)")
    ax2.set_xlabel("tau"); ax2.set_ylabel("gap (even sector)"); ax2.legend(fontsize=7)
    ax2.set_title("(c) gap: the minimum moves with the cluster", fontsize=8)

    ax3 = fig.add_subplot(2, 3, 4)
    for n in CLUSTERS:
        ax3.plot(SCAN, [p["cmax"] for p in scan(n)], label="%d pairs" % n)
    ax3.plot(SCAN, [x[2] for x in two], "k--", label="two pairs (Stage 0)")
    ax3.set_xlabel("tau"); ax3.set_ylabel("largest pairwise concurrence"); ax3.legend(fontsize=7)
    ax3.set_title("(d) the frozen end is a cat: no pairwise entanglement", fontsize=8)

    ax4 = fig.add_subplot(2, 3, 5)
    for n in CLUSTERS:
        l, = ax4.plot(SCAN, [p["chsh"] for p in scan(n)], label="%d pairs" % n)
        ax4.plot(SCAN, [p["chsh_obs"] for p in scan(n)], ":", color=l.get_color())
    ax4.plot(SCAN, [x[3] for x in two], "k--", label="two pairs")
    ax4.axhline(2, color="k", lw=0.8); ax4.axhline(t0.TSIRELSON, color="k", ls="-.", lw=0.6)
    ax4.set_xlabel("tau"); ax4.set_ylabel("largest CHSH of a reduced two-pair state"); ax4.legend(fontsize=7)
    ax4.set_title("(e) solid: state; dotted: with visibility 1 - c$^2$", fontsize=8)

    ax5 = fig.add_subplot(2, 3, 6)
    ax5.axis("off")
    f = test_e68_the_patch_is_frustrated()
    txt = ("unsatisfied bond weight f (classical)\n  level 2, 6 pairs:  %.3f\n  level 3, 6..14 pairs: %s\n\n"
           "frozen end (Gamma -> 0): cat of the classical\n  ground state, pairwise concurrence < 1e-10\n  (two pairs: a Bell state, Stage 0)\n\n"
           "largest reduced CHSH 2.33; with visibility 2.12\n  (10 pairs only)\n\nHat end: paramagnet, gap 4 Gamma\n\n"
           "Charges at centroids (v5) and the forms of\nGamma and the visibility (Stage 0) are Definitions.") % (f[0], ", ".join("%.3f" % x for x in f[1:]))
    ax5.text(0.0, 1.0, txt, va="top", family="monospace", fontsize=8)
    fig.suptitle("toymodel_v6 Stage 1: the pairs of a Spectre patch as a transverse-field Ising model")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=140)
    if show:
        plt.show()



if __name__ == "__main__":
    sys.exit(main())
