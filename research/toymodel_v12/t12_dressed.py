#!/usr/bin/env python3
"""
toymodel_v12: the joint model at kappa != 0 with matter-dressed tilings (a size check on `t12_quantum_geometry.py`).

Why not a symmetry basis: the joint space is 2^42; the icosahedral group divides it by 60 at most, and the matter
of the all-0 tiling alone is 2^30.  Neither is within reach of numpy here.  What is done instead:

**Step 1, matter at a fixed tiling (configuration interaction in the X basis).**  F = set of slots in |->.  At
fixed sigma only present slots can be in F (absent slots stay in |+>, the dormant convention), parity of |F| is
conserved, and `H_sigma` restricted to |F| <= m is a finite matrix: CI(m).  CI(2) is what `t12_quantum_geometry`
used; here CI(4), with CI(6) and exact diagonalisation as checks where they fit.

**Step 2, dressed tilings (Definition).**  |sigma~> = |sigma> (x) |psi0(sigma)>, the CI(4) matter ground state of
tiling sigma.  These 4096 states are orthonormal.  The joint Hamiltonian restricted to their span is exactly

    H_BO[sigma, sigma'] = (E0(sigma) + g n0(sigma)) delta  -  Gamma <psi0(sigma)|psi0(sigma')>   (sigma' = sigma ^ p),

because a flip X_p does not act on the slots.  Its ground state is **variational** for the joint model (an upper
bound within CI(4)), and it is size-consistent in the matter in a way the CI(2) joint run is not.  The reduced state
of the geometry is rho[s, s'] = c_s c_s' G[s, s'] with G the Gram matrix of the matter states, which gives the
geometry-matter entanglement.

Run:  HEXASPHERE_DIR=... python3 t12_dressed.py        (`--tests-only` for the checks)
"""
from __future__ import annotations

import itertools
import json
import os
import sys

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as sla

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_hamiltonian as Hm      # noqa: E402

NS = 4096


def basis_masks(slots, m):
    """Sorted bit masks of the subsets F of `slots` with |F| even and <= m."""
    out = [0]
    for k in range(2, m + 1, 2):
        out += [sum(1 << s for s in c) for c in itertools.combinations(slots, k)]
    return np.array(sorted(out), dtype=np.int64)


def popcount(a):
    a = a.copy()
    c = np.zeros(len(a), dtype=np.int64)
    while a.any():
        c += a & 1
        a >>= 1
    return c


def ci_matrix(slots, W, J, kappa, m):
    """H_sigma in the CI(m) space of the present `slots`: (masks, sparse matrix)."""
    M = basis_masks(slots, m)
    n = len(slots)
    diag = -J * (n - 2 * popcount(M))
    rows, cols, vals = [np.arange(len(M))], [np.arange(len(M))], [diag.astype(float)]
    for a, b in itertools.combinations(slots, 2):
        if W[a, b] == 0:
            continue
        T = M ^ ((1 << a) | (1 << b))
        j = np.searchsorted(M, T)
        j[j == len(M)] = 0
        ok = M[j] == T
        rows.append(np.where(ok)[0])
        cols.append(j[ok])
        vals.append(np.full(int(ok.sum()), -kappa * W[a, b]))
    H = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(len(M), len(M)))
    return M, H


def ci_ground(slots, W, J, kappa, m):
    """(E0, masks, vector) of the CI(m) ground state; the amplitude on F = {} is made positive."""
    if len(slots) < 2 or kappa == 0:
        return -J * len(slots), np.array([0], dtype=np.int64), np.ones(1)
    M, H = ci_matrix(slots, W, J, kappa, m)
    if len(M) <= 700:
        w, v = np.linalg.eigh(H.toarray())
        e, x = w[0], v[:, 0]
    else:
        w, v = sla.eigsh(H, k=1, which="SA", tol=1e-10)
        e, x = w[0], v[:, 0]
    return float(e), M, x * np.sign(x[int(np.searchsorted(M, 0))])


class Dressed:
    """All 4096 dressed tilings at given J, kappa (CI(m))."""

    def __init__(self, J=1.0, kappa=0.3, m=4, verbose=False):
        c = Hm.couplings()
        self.W, self.edges, self.J, self.kappa, self.m = c["W"], c["edges"], J, kappa, m
        self.Pm = Hm.presence(self.edges)
        self.GM = basis_masks(list(range(30)), m)                      # global CI(m) basis of the 30 slots
        self.A = np.zeros((NS, len(self.GM)), dtype=np.float32)        # matter states, rows = tilings
        self.E0 = np.zeros(NS)
        for s in range(NS):
            slots = [int(e) for e in np.where(self.Pm[s])[0]]
            e, M, x = ci_ground(slots, self.W, J, kappa, m)
            self.E0[s] = e
            self.A[s, np.searchsorted(self.GM, M)] = x
            if verbose and s % 512 == 0:
                print("  dressed", s, len(slots), flush=True)
        self.n0 = np.array([12 - bin(s).count("1") for s in range(NS)], float)
        self.n00 = self.Pm.sum(1).astype(float)
        self.n11 = np.array([sum(1 for i, j in self.edges if (s >> i) & 1 and (s >> j) & 1) for s in range(NS)], float)
        self.G = (self.A @ self.A.T).astype(np.float64)                # Gram matrix of the matter states

    def H(self, Gamma, g):
        H = np.diag(self.E0 + g * self.n0)
        idx = np.arange(NS)
        for p in range(12):
            H[idx, idx ^ (1 << p)] -= Gamma * self.G[idx, idx ^ (1 << p)]
        return H

    def solve(self, Gamma, g):
        w, v = np.linalg.eigh(self.H(Gamma, g))
        return w, v

    def observables(self, c):
        p = np.abs(c) ** 2
        rho = (c[:, None] * np.conj(c)[None, :]) * self.G
        ev = np.linalg.eigvalsh(rho)
        ev = ev[ev > 1e-13]
        # mean number of excited slots in the dressed state
        nF = Hm_pop(self.GM)
        exc = float(p @ ((self.A.astype(np.float64) ** 2) @ nF))
        return {"P_all0": float(p[0]), "P_all1": float(p[NS - 1]), "mean_rotors_in_0": float(p @ self.n0), "mean_pairs": float(p @ self.n00),
                "var_pairs": float(p @ self.n00 ** 2 - (p @ self.n00) ** 2), "mean_vertices": float(140 - p @ self.n11),
                "mean_excited_slots": exc, "S_geometry_matter_bits": float(-(ev * np.log2(ev)).sum())}


def Hm_pop(M):
    return popcount(M).astype(np.float64)


def convergence(J=1.0, kappa=0.3):
    """CI(2), CI(4), CI(6) and exact ground energies at fixed tilings of growing size."""
    c = Hm.couplings()
    W, Pm = c["W"], Hm.presence(c["edges"])
    rows = []
    E = set(map(tuple, c["edges"]))
    far = next((1 << i) | (1 << j) for i in range(12) for j in range(i + 1, 12) if (i, j) not in E)   # two 1s, not adjacent
    for s in (0b111111000000, 0b111000000000, far, 0b100000000000, 0):
        slots = [int(e) for e in np.where(Pm[s])[0]]
        n = len(slots)
        r = {"sigma": s, "pairs": n}
        for m in (2, 4, 6):
            if m == 6 and n > 21:
                continue
            r["CI%d" % m] = ci_ground(slots, W, J, kappa, m)[0]
        if n <= 20:
            r["exact"] = float(Hm.ground(slots, W, J, kappa)[0][0])
        # second-order perturbation theory, size-consistent: -kappa^2 sum W^2 / (4J)
        r["PT2"] = -J * n - kappa ** 2 * float((np.triu(W[np.ix_(slots, slots)], 1) ** 2).sum()) / (4 * J)
        rows.append(r)
    return rows


# ---- tests ----------------------------------------------------------------------------------
def test_basis_sizes():
    assert len(basis_masks(list(range(30)), 2)) == 436 and len(basis_masks(list(range(30)), 4)) == 1 + 435 + 27405


def test_ci_is_variational_and_ordered():
    c = Hm.couplings()
    Pm = Hm.presence(c["edges"])
    slots = [int(e) for e in np.where(Pm[0b111111000000])[0]]
    e2, e4, e6 = (ci_ground(slots, c["W"], 1.0, 0.3, m)[0] for m in (2, 4, 6))
    ex = float(Hm.ground(slots, c["W"], 1.0, 0.3)[0][0])
    assert e2 >= e4 >= e6 >= ex - 1e-9 and e4 - ex < 2e-3 and e6 - ex < 1e-4


def test_ci2_matches_the_block_of_the_joint_module():
    import t12_quantum_geometry as Q
    c = Hm.couplings()
    m = Q.Model(c["W"], c["edges"], kappa=0.3)
    s = 0b110011001100
    slots = [int(e) for e in np.where(m.Pm[s] > 0)[0]]
    assert abs(ci_ground(slots, c["W"], 1.0, 0.3, 2)[0] - np.linalg.eigvalsh(m.matter_block(s))[0]) < 1e-9


def test_kappa_zero_gives_the_geometry_model():
    d = Dressed(kappa=0.0)
    assert np.allclose(d.G, 1.0) and np.allclose(d.E0, -d.n00)                 # all matter states are |+...+>


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main():
    out = {"convergence": convergence()}
    for r in out["convergence"]:
        print(r, flush=True)
    d = Dressed(kappa=0.3, verbose=True)
    idx = np.arange(NS)
    nb = np.concatenate([d.G[idx, idx ^ (1 << p)] for p in range(12)])
    out["overlap_of_neighbouring_tilings"] = {"min": float(nb.min()), "mean": float(nb.mean()), "max": float(nb.max())}
    out["E0_all0_CI4"] = float(d.E0[0])
    print(out["overlap_of_neighbouring_tilings"], flush=True)
    old = {}
    p = os.path.join(HERE, "results_quantum_geometry.json")
    if os.path.exists(p):
        old = {(r["g"], r["Gamma"]): r for r in json.load(open(p))["scan"] if r["kappa"] == 0.3}
    out["scan"] = []
    for g in (0.0, 2.5):
        for Gamma in (0.2, 1.0, 3.0):
            w, v = d.solve(Gamma, g)
            r = {"g": g, "kappa": 0.3, "Gamma": Gamma, "E0": float(w[0]), "gap": float(w[1] - w[0])}
            r.update(d.observables(v[:, 0]))
            if (g, Gamma) in old:
                r["E0_joint_CI2"] = old[(g, Gamma)]["E0"]
                r["S_joint_CI2"] = old[(g, Gamma)]["S_geometry_matter_bits"]
            out["scan"].append(r)
            print({k: (round(x, 4) if isinstance(x, float) else x) for k, x in r.items()}, flush=True)
    # time evolution in the dressed space from the all-0 tiling: geometry-matter entropy and its reversal
    w, v = d.solve(1.0, 2.5)
    c0 = np.zeros(NS)
    c0[0] = 1.0
    a = v.T @ c0
    ts = [0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0]
    ev = []
    for t in ts:
        c = v @ (np.exp(-1j * w * t) * a)
        o = d.observables(c)
        cm = v @ (np.exp(1j * w * t) * a)
        ev.append({"t": t, "S_geometry_matter_bits": o["S_geometry_matter_bits"], "mean_pairs": o["mean_pairs"],
                   "S_reversed": d.observables(cm)["S_geometry_matter_bits"]})
        print(ev[-1], flush=True)
    out["evolution_Gamma1_g2.5_from_all0"] = ev
    json.dump(out, open(os.path.join(HERE, "results_dressed.json"), "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
