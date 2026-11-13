#!/usr/bin/env python3
"""t6_gauge.py - Stage 3 of toymodel_v6: a Z2 gauge form of the constraint on the Spectre tiling, and whether its Gauss law is the area ledger.

The proposal promotes the flux constraint to a plaquette (Wilson-loop) term, H = g sum_p (W_p + W_p^dag - 2), and says that g -> infinity "generates genuine entanglement and Tsirelson-bounded correlations".
Here the gauge structure is built on the patch itself and both claims are tested exactly (GF(2) stabilizer algebra, no large matrices).

Definitions
  D6.12  Cell complex of a patch: sites = tiles, links = tile edges shared by two tiles, plaquettes = interior vertices of the tiling (the links incident to the vertex).  Z2 link qubits.
  D6.13  Pure gauge theory: Gauss law G_t = prod_{l at t} X_l; plaquette B_p = prod_{l at p} Z_l; H = - g sum_p B_p - h sum_l X_l in the sector G_t = +1.
  D6.14  With matter: a qubit on each tile (the bit sigma of Stage 0), G_t = X^m_t prod_{l at t} X_l.  The ledger of a pair A = (A1, A2) is Z^m_{A1} Z^m_{A2} = -1 (Stage 0).

Exact claims (each a test; ``python3 t6_gauge.py --tests-only``)
  E6.17  The complex is a disc.  T - L + P = 1 at levels 1, 2, 3 (9 - 40 + 32, 71 - 406 + 336, 559 - 3534 + 2976); plaquettes have 2, 3 or 4 links (level 2: 254, 66, 16).  Every link has at least one interior endpoint.
         The stop condition of the plan (no plaquette structure) is not met.
  E6.18  The pure theory is consistent and its plaquette limit is unique.  Every G_t commutes with every B_p; over GF(2) the Gauss laws have rank T - 1 = 70 and the plaquettes rank P = 336 (level 2), together L = 406,
         so at h = 0 the ground state is a unique stabilizer state (no degeneracy on a disc).  Wegner duality: every tile meets every interior vertex in 0 or 2 of its links, so X_l -> mu^x_p mu^x_p' (or mu^x_p for a link
         with one interior end) and B_p -> mu^z_p preserves all relations: on a disc the plaquette theory is a transverse-field Ising model on the tiling's interior vertices.
  E6.19  The ledger is not the Gauss law.  At every flat pair (6 at level 2, 82 at level 3) the shared vertex P8 is a plaquette with exactly two links, and these are the only two links between the pair.  With matter,
         Z^m_{A1} Z^m_{A2} anticommutes with G_{A1} and G_{A2}: it is not gauge invariant.  Its gauge-invariant forms are the two Wilson lines W = Z^m_{A1} Z_e Z^m_{A2} along the two shared links, and their product is B_{P8}.
         So the ledger is a Z-type (Wilson-line) condition, not an X-type Gauss law; fixing it along both links fixes the flux at P8.  **Negative** for "the Gauss law is the area ledger".
  E6.20  In the other frame it is a Gauss law with no gauge field.  P_A = -Z_{A1} Z_{A2} commutes with every term of the tile Hamiltonian of Stages 0 and 1 (a local Z2 symmetry, one per pair).  Under a Hadamard on the tiles it is
         a two-link Gauss law at P8 whose links are the two tiles.  The pairs are disjoint, so the graph with sites P8 and links the paired tiles is a perfect matching: zero independent cycles, no Wilson loop, no gauge dynamics.
  E6.21  The g -> infinity state is entangled but has no two-link Bell pair.  In the unique h = 0 ground state (level 2) no single link and no two links carry both an X-type and a Z-type stabilizer: 394 pairs of links have a ZZ
         stabilizer (from the two-link plaquettes and their products) and none has an XX one.  Every two-link reduced state is therefore separable and its CHSH value is at most 2.  The state is entangled across a cut: for the links
         left of the patch's centre line (215 links) the entanglement entropy is 12 bits (S_A = |A| - dim S_A).  **Negative** for "g -> infinity generates Tsirelson-bounded correlations".

Status.  All Exact (finite stabilizer algebra on stated patches).  The gauge structure is a Definition; the tiling supplies the complex and the position of the ledger in it, not the Gauss law.

    python3 t6_gauge.py --tests-only
    python3 t6_gauge.py --no-show --save figures/t6_stage3_gauge.png
"""
import argparse
import collections
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_RES)
for _p in (_HERE, os.path.join(_RES, "toymodel_v4"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import _tb  # noqa: E402
import t4_patch as tp  # noqa: E402
import t6_pair as t0  # noqa: E402

_CACHE = {}


def complex_(level):
    if level in _CACHE:
        return _CACHE[level]
    G = _tb.patch_graph(level)
    faces = G.faces
    ef = collections.defaultdict(list)
    for t, f in enumerate(faces):
        for j in range(len(f)):
            u, v = f[j], f[(j + 1) % len(f)]
            ef[(min(u, v), max(u, v))].append(t)
    links = [e for e, fs in ef.items() if len(fs) == 2]
    bverts = {u for e, fs in ef.items() if len(fs) == 1 for u in e}
    inc = collections.defaultdict(list)
    for k, e in enumerate(links):
        for u in e:
            inc[u].append(k)
    plaq = [v for v in range(G.n) if v not in bverts]
    c = dict(G=G, faces=faces, ef=ef, links=links, bverts=bverts, inc=inc, plaq=plaq, T=len(faces), L=len(links), P=len(plaq))
    _CACHE[level] = c
    return c


def gauss_matrix(c):
    M = np.zeros((c["T"], c["L"]), np.uint8)
    for k, e in enumerate(c["links"]):
        for t in c["ef"][e]:
            M[t, k] = 1
    return M


def plaquette_matrix(c):
    M = np.zeros((c["P"], c["L"]), np.uint8)
    for i, v in enumerate(c["plaq"]):
        M[i, c["inc"][v]] = 1
    return M


def gf2_rank_null(M):
    """(rank, basis of the null space {w : M w = 0} as rows) over GF(2)."""
    A = (M.copy() % 2).astype(np.uint8)
    r, cols = A.shape
    piv, row = [], 0
    for j in range(cols):
        if row == r:
            break
        nz = np.nonzero(A[row:, j])[0]
        if len(nz) == 0:
            continue
        p = row + nz[0]
        if p != row:
            A[[row, p]] = A[[p, row]]
        hit = np.nonzero(A[:, j])[0]
        hit = hit[hit != row]
        A[hit] ^= A[row]
        piv.append(j)
        row += 1
    free = [j for j in range(cols) if j not in set(piv)]
    N = np.zeros((len(free), cols), np.uint8)
    for k, f in enumerate(free):
        N[k, f] = 1
        for i, pj in enumerate(piv):
            if A[i, f]:
                N[k, pj] = 1
    return row, N


def gf2_rank(M):
    return gf2_rank_null(M)[0]


def two_body_stabilizers(c):
    """Single-link and two-link elements of the X-type (Gauss) and Z-type (plaquette) groups of the h = 0 ground state."""
    _, NG = gf2_rank_null(gauss_matrix(c))      # v in rowspace(G) iff NG v = 0
    _, NB = gf2_rank_null(plaquette_matrix(c))
    out = {}
    for name, N in (("X", NG), ("Z", NB)):
        cols = [N[:, a].tobytes() for a in range(c["L"])]
        zero = [a for a in range(c["L"]) if not N[:, a].any()]
        grp = collections.defaultdict(list)
        for a, key in enumerate(cols):
            grp[key].append(a)
        pairs = {(a, b) for g in grp.values() for i, a in enumerate(g) for b in g[i + 1:]}
        out[name] = (zero, pairs)
    return out


def entropy_of_region(c, A):
    """Entanglement entropy (bits) of the link set A in the h = 0 stabilizer state: |A| - dim(S_A), CSS: S_A splits into X and Z parts."""
    A = np.asarray(sorted(A))
    rest = np.setdiff1d(np.arange(c["L"]), A)
    dim = 0
    for M in (gauss_matrix(c), plaquette_matrix(c)):
        dim += gf2_rank(M) - gf2_rank(M[:, rest])
    return int(len(A) - dim)


def link_midpoints(c):
    pts = c["G"].points
    return np.array([(pts[u] + pts[v]) / 2 for u, v in c["links"]])


# --------------------------------------------------------------------------
def test_e617_the_complex_is_a_disc():
    want = {1: (9, 40, 32), 2: (71, 406, 336), 3: (559, 3534, 2976)}
    for lv, (T, L, P) in want.items():
        c = complex_(lv)
        assert (c["T"], c["L"], c["P"]) == (T, L, P), (lv, c["T"], c["L"], c["P"])
        assert T - L + P == 1
        assert all(u not in c["bverts"] or v not in c["bverts"] for u, v in c["links"])
    sizes = collections.Counter(len(complex_(2)["inc"][v]) for v in complex_(2)["plaq"])
    assert dict(sizes) == {2: 254, 3: 66, 4: 16}, sizes
    return dict(sizes)


def test_e618_pure_theory_consistent_unique_and_dual_to_ising():
    c = complex_(2)
    Gm, Bm = gauss_matrix(c), plaquette_matrix(c)
    assert not ((Gm.astype(int) @ Bm.T.astype(int)) % 2).any()
    rg, rb = gf2_rank(Gm), gf2_rank(Bm)
    assert rg == c["T"] - 1 and rb == c["P"] and rg + rb == c["L"]
    # Wegner: a tile meets each interior vertex in 0 or 2 of its links (so the product of a tile's link maps is 1)
    plaq_index = {v: i for i, v in enumerate(c["plaq"])}
    for t in range(c["T"]):
        cnt = collections.Counter()
        for k in np.nonzero(Gm[t])[0]:
            for u in c["links"][k]:
                if u in plaq_index:
                    cnt[u] += 1
        assert all(n in (0, 2) for n in cnt.values()), (t, cnt)
    return rg, rb


def test_e619_the_ledger_is_a_wilson_line_not_a_gauss_law():
    for lv, npairs in ((2, 6), (3, 82)):
        c = complex_(lv)
        pr = tp.flat_pairs(lv, 1.0)
        assert len(pr) == npairs
        for a, b in pr:
            v = c["faces"][a][tp.FLAT]
            assert c["faces"][b][tp.FLAT] == v and v not in c["bverts"]
            lk = c["inc"][v]
            between = [k for k, e in enumerate(c["links"]) if sorted(c["ef"][e]) == sorted([a, b])]
            assert len(lk) == 2 and sorted(lk) == sorted(between)
    # symplectic check with matter on level 2: columns (matter T, links L), X block then Z block
    c = complex_(2)
    T, L = c["T"], c["L"]
    Gm = gauss_matrix(c)

    def pauli(xm=(), xl=(), zm=(), zl=()):
        v = np.zeros(2 * (T + L), np.uint8)
        for i in xm: v[i] = 1
        for i in xl: v[T + i] = 1
        for i in zm: v[T + L + i] = 1
        for i in zl: v[2 * T + L + i] = 1
        return v

    def anticommute(p, q):
        n = T + L
        return int((p[:n].astype(int) @ q[n:] + p[n:].astype(int) @ q[:n]) % 2)

    gauss = [pauli(xm=(t,), xl=tuple(np.nonzero(Gm[t])[0])) for t in range(T)]
    for a, b in tp.flat_pairs(2, 1.0):
        led = pauli(zm=(a, b))
        assert anticommute(led, gauss[a]) and anticommute(led, gauss[b])
        v = c["faces"][a][tp.FLAT]
        e1, e2 = c["inc"][v]
        w1, w2 = pauli(zm=(a, b), zl=(e1,)), pauli(zm=(a, b), zl=(e2,))
        assert all(not anticommute(w, g) for w in (w1, w2) for g in gauss)
        prod = w1 ^ w2
        assert np.array_equal(prod, pauli(zl=(e1, e2)))          # = B_{P8}
    return "ledger = Wilson line; W_e W_e' = B_P8"


def test_e620_in_the_other_frame_a_gauss_law_without_a_gauge_field():
    H = t0.tile_hamiltonian(1.0, 3.0, 0.7, eps=0.4)
    n = 4
    for i, j in ((0, 1), (2, 3)):
        P = -t0.op(t0.SZ, i, n) @ t0.op(t0.SZ, j, n)
        assert np.abs(H @ P - P @ H).max() < 1e-12
    # the graph with sites = P8 points and links = the paired tiles: a cycle needs a link (tile) at two sites, i.e. a tile in two pairs
    for lv in (2, 3):
        pr = tp.flat_pairs(lv, 1.0)
        at = collections.Counter(t for p in pr for t in p)
        assert max(at.values()) == 1           # every tile-link has one site: components are single sites, zero cycles
    return "matching: no cycles"


def test_e621_the_plaquette_limit_has_no_two_link_bell_pair():
    c = complex_(2)
    st = two_body_stabilizers(c)
    zx, px = st["X"]
    zz, pz = st["Z"]
    assert not zx and not zz
    assert len(px) == 0 and len(pz) == 394, (len(px), len(pz))
    assert not (px & pz)
    mid = link_midpoints(c)
    centre = c["G"].points.mean(axis=0)
    A = np.nonzero(mid[:, 0] < centre[0])[0]
    S = entropy_of_region(c, A)
    assert len(A) == 215 and S == 12, (len(A), S)
    return len(pz), int(len(A)), S


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
    from matplotlib.collections import LineCollection, PolyCollection

    c = complex_(2)
    pts = c["G"].points
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.8))
    polys = [pts[f] for f in c["faces"]]
    pr = tp.flat_pairs(2, 1.0)
    paired = {t for p in pr for t in p}
    ax[0].add_collection(PolyCollection(polys, facecolors=["#efe3c7" if t in paired else "0.94" for t in range(c["T"])], edgecolors="none"))
    ax[0].add_collection(LineCollection([pts[list(e)] for e in c["links"]], colors="0.45", linewidths=0.6))
    bnd = [e for e, fs in c["ef"].items() if len(fs) == 1]
    ax[0].add_collection(LineCollection([pts[list(e)] for e in bnd], colors="k", linewidths=1.2))
    size = {v: len(c["inc"][v]) for v in c["plaq"]}
    for s_, col, ms in ((2, "tab:blue", 6), (3, "tab:green", 18), (4, "tab:purple", 26)):
        vv = [v for v in c["plaq"] if size[v] == s_]
        ax[0].scatter(pts[vv, 0], pts[vv, 1], s=ms, color=col, label="plaquette with %d links (%d)" % (s_, len(vv)), zorder=3)
    p8 = [c["faces"][a][tp.FLAT] for a, b in pr]
    ax[0].scatter(pts[p8, 0], pts[p8, 1], s=120, marker="*", color="tab:red", label="P8 of a flat pair (6)", zorder=4)
    ax[0].set_aspect("equal"); ax[0].set_xticks([]); ax[0].set_yticks([]); ax[0].autoscale_view()
    ax[0].legend(fontsize=7, loc="lower left")
    ax[0].set_title("(a) level 2: links (grey), plaquettes at interior vertices;\nT - L + P = 71 - 406 + 336 = 1", fontsize=9)

    # (b) schematic of one pair
    a, b = pr[0]
    v = c["faces"][a][tp.FLAT]
    e1, e2 = c["inc"][v]
    sub = [a, b]
    lo = np.min(np.vstack([pts[c["faces"][t]] for t in sub]), axis=0) - 0.5
    hi = np.max(np.vstack([pts[c["faces"][t]] for t in sub]), axis=0) + 0.5
    ax[1].add_collection(PolyCollection([pts[c["faces"][t]] for t in sub], facecolors=["#c9d6ea", "#f3c9c9"], edgecolors="0.4"))
    for k, col in ((e1, "tab:orange"), (e2, "tab:green")):
        ax[1].plot(*pts[list(c["links"][k])].T, color=col, lw=4)
    ca, cb = pts[c["faces"][a]].mean(axis=0), pts[c["faces"][b]].mean(axis=0)
    ax[1].scatter(*pts[v], s=200, marker="*", color="tab:red", zorder=4)
    ax[1].text(*ca, "A1\nZ$^m_{A1}$", ha="center", fontsize=9); ax[1].text(*cb, "A2\nZ$^m_{A2}$", ha="center", fontsize=9)
    ax[1].set_xlim(lo[0], hi[0]); ax[1].set_ylim(lo[1], hi[1]); ax[1].set_aspect("equal"); ax[1].set_xticks([]); ax[1].set_yticks([])
    ax[1].set_title("(b) one flat pair: P8 (star) is a plaquette with the two shared links.\nLedger Z$_{A1}$Z$_{A2}$ is not gauge invariant; W$_e$ = Z$_{A1}$Z$_e$Z$_{A2}$ (orange, green)\nare, and W$_e$ W$_{e'}$ = B$_{P8}$", fontsize=8)

    # (c) entanglement entropy of a half-plane cut, and two-link stabilizers
    mid = link_midpoints(c)
    xs = np.linspace(mid[:, 0].min() + 0.5, mid[:, 0].max() - 0.5, 21)
    S, nA = [], []
    for x in xs:
        A = np.nonzero(mid[:, 0] < x)[0]
        S.append(entropy_of_region(c, A)); nA.append(len(A))
    ax[2].plot(xs, S, "o-k", label="entropy of the links left of x (bits)")
    ax[2].set_ylabel("bits")
    st = two_body_stabilizers(c)
    ax[2].set_xlabel("position of the cut x\n\nh = 0 ground state (unique): two-link ZZ stabilizers %d, two-link XX stabilizers %d\n-> no two-link Bell pair, every two-link CHSH <= 2" % (len(st["Z"][1]), len(st["X"][1])), fontsize=8)
    ax[2].legend(fontsize=7, loc="upper right")
    ax[2].set_title("(c) the plaquette limit is entangled across a cut,\nbut not pairwise", fontsize=9)
    fig.suptitle("toymodel_v6 Stage 3: a Z2 gauge form on the Spectre patch, and where the area ledger sits in it")
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
