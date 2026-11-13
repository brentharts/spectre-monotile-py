#!/usr/bin/env python3
"""t6_hop.py - Stage 2 of toymodel_v6: the hopping sector.  A single particle hopping on the Spectre vertex graph and on the Chair44 tile graph, against the walk tensor and the hop count of toymodel_v3.

The paper's open problem (item 10, 'Clock and space') says: on the Chair44 the walk tensor is isotropic and the hop count is cubic, so which of the two does a wave follow?  A hopping Hamiltonian answers
that for the simplest quantum wave.  H = t L with L = D - A the graph Laplacian (D6.9; the adjacency form -A is the control).  Graphs from `toymodel_v3/t3_escape.py`: the vertex graph of the level-4 Spectre
patch (28006 sites) and the face-adjacency graph of the depth-5 Chair44 supertile (32768 tiles), positions the vertices and the tile centroids.

Definitions
  D6.9   H = L (t = 1).  A run starts on one bulk site (60 random sites within radius 15 (Spectre) / 8 (Chair44) of the centre, two seeds) and averages |psi(t)|^2 over the starts, with r measured from each start.
  D6.10  Second moment S(t) = <r r^T>; fourth-moment index A4 = <sum_i r_i^4> / <|r|^4>: isotropic 3/5 in 3D, 3/4 in 2D; all weight on the cube axes 1, on face diagonals 1/2, on body diagonals 1/3.
  D6.11  Cone radii: the mean |r| of the probability inside the cones of half-angle 18 degrees about the 3 axes, 6 face and 4 body diagonals.  Hop count: graph distance / Euclidean distance in the same cones (BFS).

Exact / Numeric claims (each a test; ``python3 t6_hop.py --tests-only``)
  E6.12  Long waves see the walk tensor.  For a plane wave on the sites, <k|L|k> = sum_edges 2 (1 - cos k.e) = k^T M k + O(k^4) exactly (checked: ratio -> 1 as k -> 0).  In the bulk the walk tensor M per site is isotropic
         to 0.998 (Spectre, 2D) and 0.995 (Chair44, 3D) (ratio of its smallest to largest eigenvalue).
  E6.13  The Spectre graph is isotropic and sub-ballistic.  A4 = 0.750 +- 0.01 (the 2D isotropic value; six-fold statistics force it), S isotropic to 0.95; rms(t) ~ t^beta with beta between 0.54 and 0.60 on t in [4, 24]
         for both seeds: anomalous transport, slower than a crystal (1) and faster than diffusion (1/2).  **Numeric**; finite times.
  E6.14  The Chair44 packet is cubic, and opposite to the hop count.  A4 = 0.477 (both seeds, to 0.005), below the isotropic 3/5: the probability goes along the diagonals.  The cone radii at t = 3 are 9.0 and 9.5 along the
         axes and 12.3 to 12.8 along face and body diagonals (ratio below 0.8).  The hop count per unit length is smallest along the axes (BFS, computed here: 0.59, 0.70, 0.69 hops per unit length on axes, face and body diagonals), so a hop-by-hop front would lead along the axes; the quantum
         wave lags there.  S is uniaxial with its short axis within 8 degrees of the chair's three-fold axis (1,1,1) (6.6 and 4.1 for the two seeds).  The adjacency form -A gives A4 within 0.02.
  E6.15  On the Chair44 the spreading is close to ballistic over the times the patch allows: beta = 0.88 on t in [1, 2.5], before the probability beyond radius 25 exceeds 2e-3.  **Numeric**; the patch is small.
  E6.16  No exact light cone.  The amplitude at graph distance D after time t is non-zero for every D and obeys the Lieb-Robinson bound |<j| e^{-iHt} |i>| <= (||H|| t)^D / D! (checked for D = 1..12, t = 0.3); the leapfrog wave of
         toymodel_v2 (E5.5) has an exact cone.  A Hamiltonian buys reversibility and unitarity and loses the exact cone; the effective speed is set by the bound, not by a lattice rule.

Status.  E6.12 and E6.16 Exact; E6.13 to E6.15 Numeric (finite patches and times, 60 starts).  Nothing here is light: 'which of the two a wave follows' is answered for a single hopping particle only.

    python3 t6_hop.py --tests-only
    python3 t6_hop.py --no-show --save figures/t6_stage2_hop.png
"""
import argparse
import math
import os
import sys

import numpy as np
import scipy.sparse as sps
import scipy.sparse.csgraph as csg
import scipy.sparse.linalg as sla

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_RES)
for _p in (_HERE, os.path.join(_RES, "toymodel_v3"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t3_escape as te  # noqa: E402

N_STARTS = 60
CONE = math.cos(math.radians(18.0))
DIRS = {"axis": np.eye(3),
        "face": np.array([[1, 1, 0], [1, 0, 1], [0, 1, 1], [1, -1, 0], [1, 0, -1], [0, 1, -1]]) / math.sqrt(2),
        "body": np.array([[1, 1, 1], [1, 1, -1], [1, -1, 1], [-1, 1, 1]]) / math.sqrt(3)}
_CACHE = {}


def graph(name):
    if name not in _CACHE:
        X, E, d = te.spectre_graph(4) if name == "spectre" else te.chair44_graph(5)
        n = len(X)
        A = sps.csr_matrix((np.ones(2 * len(E)), (np.r_[E[:, 0], E[:, 1]], np.r_[E[:, 1], E[:, 0]])), shape=(n, n)).tocsc()
        deg = np.asarray(A.sum(axis=1)).ravel()
        _CACHE[name] = dict(X=X, E=E, d=d, A=A, L=(sps.diags(deg) - A).tocsc(), centre=X[np.argmin(d)])
    return _CACHE[name]


def walk_tensor(name, frac=0.35):
    g = graph(name)
    X, E, c = g["X"], g["E"], g["centre"]
    R = frac * g["d"].max()
    inb = (np.linalg.norm(X[E[:, 0]] - c, axis=1) < R) & (np.linalg.norm(X[E[:, 1]] - c, axis=1) < R)
    e = X[E[inb, 1]] - X[E[inb, 0]]
    return (e[:, :, None] * e[:, None, :]).sum(axis=0) / (np.linalg.norm(X - c, axis=1) < R).sum()


def starts(name, seed):
    g = graph(name)
    rad = 15.0 if name == "spectre" else 8.0
    bulk = np.where(np.linalg.norm(g["X"] - g["centre"], axis=1) < rad)[0]
    return np.random.default_rng(seed).choice(bulk, N_STARTS, replace=False)


def run(name, seed, times, form="L"):
    """Averages over the starts at the given times: S (dim x dim), A4, the probability beyond radius 25 of the centre, cone radii (3D)."""
    key = (name, seed, tuple(times), form)
    if key in _CACHE:
        return _CACHE[key]
    g = graph(name)
    X = g["X"]
    H = g["L"] if form == "L" else -g["A"]
    dim = X.shape[1]
    nt = len(times)
    S = np.zeros((nt, dim, dim)); M4 = np.zeros(nt); R4 = np.zeros(nt); edge = np.zeros(nt)
    cone = {k: np.zeros((nt, 2)) for k in DIRS} if dim == 3 else None
    far = np.linalg.norm(X - g["centre"], axis=1) > 25.0
    for s in starts(name, seed):
        r = X - X[s]
        rr = np.linalg.norm(r, axis=1)
        masks = {k: np.max(np.abs(r @ D.T), axis=1) / np.maximum(rr, 1e-12) > CONE for k, D in DIRS.items()} if dim == 3 else None
        psi = np.zeros(len(X), complex); psi[s] = 1.0
        tprev = 0.0
        for k, t in enumerate(times):
            if t > tprev:
                psi = sla.expm_multiply(-1j * (t - tprev) * H, psi)
                tprev = t
            p = np.abs(psi) ** 2
            S[k] += (p[:, None, None] * r[:, :, None] * r[:, None, :]).sum(axis=0)
            M4[k] += (p * (r ** 4).sum(axis=1)).sum(); R4[k] += (p * rr ** 4).sum()
            edge[k] += p[far].sum()
            if masks:
                for c, m in masks.items():
                    cone[c][k] += ((p[m] * rr[m]).sum(), p[m].sum())
    out = dict(S=S / N_STARTS, A4=M4 / np.maximum(R4, 1e-300), edge=edge / N_STARTS,
               cone={c: v[:, 0] / np.maximum(v[:, 1], 1e-300) for c, v in cone.items()} if cone else None)
    _CACHE[key] = out
    return out


def rms(S):
    return np.sqrt(np.trace(S, axis1=1, axis2=2))


def hops_per_length(name="chair44", seed=1, rmin=10.0, rmax=22.0):
    """Mean graph distance / Euclidean distance in the three cones, from the start sites (BFS)."""
    g = graph(name)
    X = g["X"]
    out = {k: [] for k in DIRS}
    for s in starts(name, seed)[:20]:
        dist = csg.shortest_path(g["A"], unweighted=True, indices=[s])[0]
        r = X - X[s]
        rr = np.linalg.norm(r, axis=1)
        sel = (rr > rmin) & (rr < rmax)
        for k, D in DIRS.items():
            m = sel & (np.max(np.abs(r @ D.T), axis=1) / np.maximum(rr, 1e-12) > CONE)
            out[k].append(float(np.mean(dist[m] / rr[m])))
    return {k: float(np.mean(v)) for k, v in out.items()}


# --------------------------------------------------------------------------
def test_e612_long_waves_see_the_walk_tensor():
    for name in ("spectre", "chair44"):
        g = graph(name)
        e = g["X"][g["E"][:, 1]] - g["X"][g["E"][:, 0]]
        M = (e[:, :, None] * e[:, None, :]).sum(axis=0)
        rng = np.random.default_rng(0)
        u = rng.standard_normal(M.shape[0]); u /= np.linalg.norm(u)
        rat = []
        for kk in (0.1, 0.03, 0.01):
            k = kk * u
            rat.append(float((2 * (1 - np.cos(e @ k))).sum() / (k @ M @ k)))
        assert abs(rat[-1] - 1) < 1e-4 and abs(rat[0] - 1) > abs(rat[-1] - 1)
    an = {}
    for name in ("spectre", "chair44"):
        w = np.linalg.eigvalsh(walk_tensor(name))
        an[name] = float(w.min() / w.max())
    assert an["spectre"] > 0.997 and an["chair44"] > 0.994, an
    return {k: round(v, 4) for k, v in an.items()}


SPEC_T = (2.0, 4.0, 8.0, 16.0, 24.0)
CHAIR_T = (1.0, 1.5, 2.0, 2.5, 3.0)


def test_e613_the_spectre_graph_is_isotropic_and_sub_ballistic():
    out = []
    for seed in (1, 2):
        r = run("spectre", seed, SPEC_T)
        rm = rms(r["S"])
        beta = float(np.polyfit(np.log(SPEC_T[1:]), np.log(rm[1:]), 1)[0])
        assert 0.54 < beta < 0.60, beta
        assert np.all(np.abs(r["A4"][1:] - 0.75) < 0.01), r["A4"]
        w = np.linalg.eigvalsh(r["S"][-1])
        assert w.min() / w.max() > 0.95
        out.append(round(beta, 3))
    return out


def test_e614_the_chair_packet_is_cubic_and_opposite_to_the_hop_count():
    res = []
    for seed in (1, 2):
        r = run("chair44", seed, CHAIR_T)
        a4 = float(r["A4"][-1])
        assert abs(a4 - 0.477) < 0.005, a4
        c = {k: float(v[-1]) for k, v in r["cone"].items()}
        assert 9.0 < c["axis"] < 9.7 and all(12.2 < c[k] < 12.9 for k in ("face", "body")), c
        assert c["axis"] / min(c["face"], c["body"]) < 0.8
        w, v = np.linalg.eigh(r["S"][-1])
        ang = math.degrees(math.acos(abs(v[:, 0] @ np.ones(3)) / math.sqrt(3)))
        assert ang < 8.0, ang
        res.append((round(a4, 4), {k: round(x, 2) for k, x in c.items()}, round(ang, 1)))
    hp = hops_per_length()
    assert hp["axis"] < hp["face"] and hp["axis"] < hp["body"], hp
    ra = run("chair44", 1, CHAIR_T, form="A")
    assert abs(ra["A4"][-1] - res[0][0]) < 0.02
    return res, {k: round(v, 3) for k, v in hp.items()}


def test_e615_the_chair_spreading_is_close_to_ballistic():
    r = run("chair44", 1, CHAIR_T)
    keep = [i for i, e in enumerate(r["edge"]) if e < 2e-3]
    assert len(keep) >= 3 and CHAIR_T[keep[-1]] >= 2.5
    t = np.array(CHAIR_T)[keep]
    beta = float(np.polyfit(np.log(t), np.log(rms(r["S"])[keep]), 1)[0])
    assert 0.85 < beta < 0.95, beta
    return round(beta, 3)


def test_e616_no_exact_light_cone():
    g = graph("chair44")
    H = g["L"]
    s = int(np.argmin(g["d"]))
    dist = csg.shortest_path(g["A"], unweighted=True, indices=[s])[0]
    norm = float(abs(sla.eigsh(H.astype(float), k=1, which="LM", return_eigenvectors=False)[0]))
    t = 0.3
    psi = np.zeros(H.shape[0], complex); psi[s] = 1
    psi = sla.expm_multiply(-1j * t * H, psi)
    worst = 0.0
    for D in range(1, 13):
        amp = float(np.abs(psi[dist == D]).max())
        bound = (norm * t) ** D / math.factorial(D)
        assert 0 < amp <= bound, (D, amp, bound)
        worst = max(worst, amp / bound)
    return round(norm, 3), worst


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


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 4, figsize=(17, 4.8))
    for seed, ls in ((1, "-"), (2, "--")):
        rs = run("spectre", seed, SPEC_T)
        rc = run("chair44", seed, CHAIR_T)
        ax[0].loglog(SPEC_T, rms(rs["S"]), "o" + ls, color="tab:orange", label="Spectre vertex graph" if seed == 1 else None)
        ax[0].loglog(CHAIR_T, rms(rc["S"]), "s" + ls, color="tab:blue", label="Chair44 tile graph" if seed == 1 else None)
        ax[1].plot(SPEC_T, rs["A4"], "o" + ls, color="tab:orange")
        ax[1].plot(CHAIR_T, rc["A4"], "s" + ls, color="tab:blue")
    tt = np.geomspace(1, 24, 10)
    ax[0].loglog(tt, 4.5 * tt, "k:", lw=0.8, label="ballistic t")
    ax[0].loglog(tt, 2.9 * tt ** 0.5, "k-.", lw=0.8, label="diffusive t$^{1/2}$")
    ax[0].set_xlabel("t"); ax[0].set_ylabel("rms distance (averaged over 60 starts)"); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) Spectre sub-ballistic (t$^{0.56}$), Chair44 near ballistic", fontsize=9)
    ax[1].axhline(0.75, color="tab:orange", lw=0.8, ls=":"); ax[1].axhline(0.6, color="tab:blue", lw=0.8, ls=":")
    ax[1].text(20, 0.735, "isotropic 2D: 3/4", fontsize=7, ha="right"); ax[1].text(20, 0.585, "isotropic 3D: 3/5", fontsize=7, ha="right")
    ax[1].set_xscale("log"); ax[1].set_xlabel("t"); ax[1].set_ylabel("A4 = <sum r_i^4> / <r^4>")
    ax[1].set_title("(b) fourth moments: Spectre isotropic, Chair44 diagonal-heavy", fontsize=9)
    rc = run("chair44", 1, CHAIR_T)
    hp = hops_per_length()
    ks = ["axis", "face", "body"]
    x = np.arange(3)
    ax[2].bar(x - 0.2, [rc["cone"][k][-1] for k in ks], 0.4, label="quantum: mean radius at t = 3")
    ax2 = ax[2].twinx()
    ax2.bar(x + 0.2, [1.0 / hp[k] for k in ks], 0.4, color="tab:red", label="hop front: length per hop")
    ax[2].set_xticks(x); ax[2].set_xticklabels(["cube axes", "face diagonals", "body diagonals"])
    ax[2].set_ylabel("mean radius in the cone"); ax2.set_ylabel("Euclidean length per hop", color="tab:red")
    h1, l1 = ax[2].get_legend_handles_labels(); h2, l2 = ax2.get_legend_handles_labels()
    ax[2].legend(h1 + h2, l1 + l2, fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=1)
    ax[2].set_title("(c) Chair44: the wave lags where the hop count leads", fontsize=9)
    g = graph("chair44")
    s = int(np.argmin(g["d"]))
    dist = csg.shortest_path(g["A"], unweighted=True, indices=[s])[0]
    norm = float(abs(sla.eigsh(g["L"].astype(float), k=1, which="LM", return_eigenvectors=False)[0]))
    for t, col in ((0.3, "k"), (1.0, "tab:green")):
        psi = np.zeros(g["L"].shape[0], complex); psi[s] = 1
        psi = sla.expm_multiply(-1j * t * g["L"], psi)
        D = np.arange(1, 16)
        ax[3].semilogy(D, [np.abs(psi[dist == k]).max() for k in D], "o-", color=col, ms=3, label="max |amplitude|, t = %.1f" % t)
        ax[3].semilogy(D, [(norm * t) ** k / math.factorial(k) for k in D], ":", color=col, label="Lieb-Robinson bound, t = %.1f" % t)
    ax[3].set_ylim(1e-30, 1e8)
    ax[3].set_xlabel("graph distance D"); ax[3].legend(fontsize=7)
    ax[3].set_title("(d) no exact cone, only the Lieb-Robinson bound", fontsize=9)
    fig.suptitle("toymodel_v6 Stage 2: one hopping particle on the Spectre and Chair44 graphs (H = graph Laplacian)")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=140)
    if show:
        plt.show()


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


if __name__ == "__main__":
    sys.exit(main())
