#!/usr/bin/env python3
"""
toymodel_v12, stage S3: the pair-qubit Hamiltonian on the Hat Trick sphere.

From S1 and S2: a tiling is sigma in {0,1}^12 (one bit per pentagon = icosahedron vertex), and a pair qubit sits on
the icosahedron edge pq iff sigma_p = sigma_q = 0.  The two hats of a pair do not depend on the rest of sigma, so
there is one fixed 30 x 30 coupling matrix W (dipole sums over the hat centroids, as in `quantgravalt.tex`), and

    H_sigma = -J sum_{e present} X_e  -  kappa sum_{e<e' present} W_ee' Z_e Z_e'          (D1: J, kappa free).

W is normalised so that its largest entry has modulus 1 (kappa_norm of the paper).  Contents:
  * `couplings()`            W, the edges, and the presence mask of every sigma;
  * `classical_30()`         exact classical ground state of the all-0 tiling (30 pairs, all 2^29 states);
  * `ground(sigma, J, kappa)` exact diagonalisation for one tiling (up to 20 pairs), parity blocks.

Run:  HEXASPHERE_DIR=... python3 t12_hamiltonian.py        (`--tests-only` for the checks)
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_pairs_sphere as P     # noqa: E402
import t12_flip_graph as G       # noqa: E402

_C = {}


def couplings(kind="chord"):
    """dict(edges=[(i, j)] over pentagon indices, W=30x30 normalised, scale, mid=edge midpoints on the sphere)."""
    if kind in _C:
        return _C[kind]
    b, Ts = G.all_tilings()
    sig = {G.bits(b, T): T for T in Ts}
    T0 = sig[tuple([0] * 12)]
    cl = P.classify(b, T0, G._ALL["at"])
    assert len(cl["pairs"]) == 30
    edges = G.icosa_edges(b)
    X = np.array([b.V[p] for p in b.pent])
    mid = np.array([(X[i] + X[j]) / np.linalg.norm(X[i] + X[j]) for i, j in edges])
    cents = [None] * 30
    for (i, j) in cl["pairs"]:
        f = b.S.corner_position(cl["flat"][i])
        e = int(np.argmin(np.linalg.norm(mid - f, axis=1)))
        assert np.linalg.norm(mid[e] - f) < 1e-6 and cents[e] is None
        cents[e] = (P.hat_centroid(b, cl["hats"][i]), P.hat_centroid(b, cl["hats"][j]))
    W = P.pair_W(cents, kind)
    scale = float(np.abs(W).max())
    _C[kind] = {"edges": edges, "W": W / scale, "scale": scale, "mid": mid, "cents": cents}
    return _C[kind]


def presence(edges):
    """P[s, e] = 1 if edge e is present in the tiling with bit mask s (bit p of s = sigma_p)."""
    s = np.arange(4096)
    out = np.zeros((4096, len(edges)), dtype=np.int8)
    for e, (i, j) in enumerate(edges):
        out[:, e] = (((s >> i) & 1) == 0) & (((s >> j) & 1) == 0)
    return out


def classical_30(W):
    """Exact minimum of E(s) = -sum_{e<e'} W s s over the 2^29 states (s_0 = +1) by splitting into 15 + 15 spins."""
    n = len(W)
    assert n == 30
    A, B = list(range(15)), list(range(15, 30))
    idxb = np.arange(1 << 15)
    Sb = (1 - 2 * ((idxb[:, None] >> np.arange(15)[None]) & 1)).astype(np.float64)
    Eb = -0.5 * np.einsum("ip,pq,iq->i", Sb, W[np.ix_(B, B)], Sb)
    idxa = np.arange(1 << 14)
    Sa = np.ones((len(idxa), 15))
    Sa[:, 1:] = 1 - 2 * ((idxa[:, None] >> np.arange(14)[None]) & 1)
    Ea = -0.5 * np.einsum("ip,pq,iq->i", Sa, W[np.ix_(A, A)], Sa)
    Hf = Sa @ W[np.ix_(A, B)]                      # field of a on b
    best, deg, arg = np.inf, 0, None
    for lo in range(0, len(idxa), 256):
        E = Ea[lo:lo + 256, None] + Eb[None, :] - Hf[lo:lo + 256] @ Sb.T
        m = E.min()
        if m < best - 1e-9:
            best, deg = m, int((E < m + 1e-9).sum())
            ia, ib = np.unravel_index(int(E.argmin()), E.shape)
            arg = np.concatenate([Sa[lo + ia], Sb[ib]])
        elif abs(m - best) <= 1e-9:
            deg += int((E < m + 1e-9).sum())
    tot = np.abs(np.triu(W, 1)).sum()
    return {"E0": float(best), "degeneracy_mod_global_flip": deg, "f_unsat": float((tot + best) / (2 * tot)), "state": arg.astype(int).tolist(),
            "antiferro_share": float((np.triu(W, 1) < 0).sum() / 435)}


def ground(present, W, J, kappa, k=2):
    """Lowest `k` eigenvalues and the ground vector of H_sigma on the present edges (list of edge indices), in the
    Z basis (no symmetry sector is separated: at small J the two lowest states are the cat doublet of the global
    flip, so `gap` is then the doublet splitting and the returned vector may be any state of the doublet)."""
    es = list(present)
    n = len(es)
    if n == 0:
        return np.array([0.0]), np.ones(1)
    assert n <= 20
    import scipy.sparse as sp
    import scipy.sparse.linalg as sla
    N = 1 << n
    idx = np.arange(N)
    S = 1 - 2 * ((idx[:, None] >> np.arange(n)[None]) & 1)
    Wp = W[np.ix_(es, es)]
    diag = -kappa * 0.5 * np.einsum("ip,pq,iq->i", S.astype(float), Wp, S.astype(float))
    H = sp.diags(diag)
    for p in range(n):
        H = H - J * sp.csr_matrix((np.ones(N), (idx, idx ^ (1 << p))), shape=(N, N))
    if N <= 512:
        w, v = np.linalg.eigh(H.toarray())
    else:
        w, v = sla.eigsh(H.tocsr(), k=max(k, 2), which="SA")
        o = np.argsort(w)
        w, v = w[o], v[:, o]
    return w[:k], v[:, 0]


def entropy_half(psi, n):
    """Entanglement entropy (bits) of the first n//2 qubits."""
    a = n // 2
    s = np.linalg.svd(psi.reshape(1 << (n - a), 1 << a), compute_uv=False) ** 2
    s = s[s > 1e-14]
    return float(-(s * np.log2(s)).sum())


# ---- tests ---------------------------------------------------------------------------------
def test_couplings_have_icosahedral_symmetry():
    c = couplings()
    W = c["W"]
    assert W.shape == (30, 30) and np.allclose(W, W.T) and abs(np.abs(W).max() - 1) < 1e-12
    # the spectrum of |W| rows is the same for every edge (one orbit of edges)
    rows = np.sort(np.abs(W), axis=1)
    assert np.allclose(rows, rows[0], atol=1e-9)


def test_presence_counts():
    c = couplings()
    Pm = presence(c["edges"])
    assert Pm[0].sum() == 30 and Pm[4095].sum() == 0 and Pm[1].sum() == 25
    assert sorted(set(Pm.sum(1).tolist()))[-3:] == [21, 25, 30]


def test_exact_ground_matches_dense_and_limits():
    c = couplings()
    W = c["W"]
    w, v = ground([0, 1, 2, 3], W, 1.0, 0.0)
    assert abs(w[0] + 4) < 1e-9                                      # kappa = 0: every pair in |+>, energy -J each
    w2, _ = ground([0, 1, 2, 3, 4, 5], W, 0.0001, 1.0)
    e0, s, deg, f = P.classical_ground(W[:6, :6])
    assert abs(w2[0] - e0) < 1e-2                                    # J -> 0: the classical ground energy


def test_classical_split_agrees_with_brute_force_on_a_small_case():
    rng = np.random.default_rng(1)
    c = couplings()
    r = classical_30(c["W"])
    s = np.array(r["state"])
    assert abs(-0.5 * s @ c["W"] @ s - r["E0"]) < 1e-9
    for _ in range(2000):                                            # no single or double flip lowers the energy
        t = s.copy()
        for q in rng.choice(30, size=rng.integers(1, 3), replace=False):
            t[q] *= -1
        assert -0.5 * t @ c["W"] @ t >= r["E0"] - 1e-9


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main():
    out = {}
    for kind in ("chord", "arc"):
        c = couplings(kind)
        r = classical_30(c["W"])
        r["scale"] = c["scale"]
        out["classical_all0_" + kind] = r
        print(kind, {k: v for k, v in r.items() if k != "state"}, flush=True)
    c = couplings()
    W, edges = c["W"], c["edges"]
    Pm = presence(edges)
    # quantum: tilings with one orbit of structure: k pentagons in state 0 forming a connected cap
    rows = []
    b, _ = G.all_tilings()
    X = np.array([b.V[p] for p in b.pent])
    order = np.argsort(-X @ X[0])                                    # pentagons by distance from pentagon 0
    for m in (4, 6, 7, 8):                                           # the m nearest pentagons in state 0, others 1
        s = sum(1 << int(p) for p in order[m:])
        pres = np.where(Pm[s])[0]
        for J, kappa in ((1.0, 0.3), (1.0, 1.0), (1.0, 3.0), (0.3, 1.0), (0.1, 1.0)):
            w, v = ground(pres, W, J, kappa)
            e0, _, deg, f = P.classical_ground(W[np.ix_(pres, pres)]) if len(pres) <= 24 else (None, None, None, None)
            rows.append({"zeros": m, "pairs": int(len(pres)), "J": J, "kappa": kappa, "E0": float(w[0]), "gap": float(w[1] - w[0]),
                         "S_half_bits": entropy_half(v, len(pres)), "f_unsat_classical": f, "classical_degeneracy": deg})
            print(rows[-1], flush=True)
    out["quantum_caps"] = rows
    json.dump(out, open(os.path.join(HERE, "results_hamiltonian.json"), "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
