#!/usr/bin/env python3
"""
toymodel_v12, stages S3 (joint) and S4: quantum geometry bits, edge qubits and the flip channel.

**Definition (the joint model).**  Geometry: one qubit per pentagon, basis |sigma>, sigma in {0,1}^12 (the 4096 hat
tilings of the Hat Trick sphere, S2).  Matter: one qubit slot per icosahedron edge (30 slots); slot e = pq is
*present* iff sigma_p = sigma_q = 0, n_e = (1 - sigma_p)(1 - sigma_q).

    H = -Gamma sum_p X_p  +  g sum_p (1 - sigma_p)  -  J sum_e n_e X_e  -  kappa sum_{e<e'} n_e n_e' W_ee' Z_e Z_e'

X_p is a phason flip of the rotor at p (5 hats for 5).  It acts on the geometry qubit only: the matter slots keep
their state when they are switched off and on (the *dormant* rule of qwla Stage 3).  Gamma (flip tunnelling) and g
(a cost per rotor in the pair-bearing state) are free parameters, like J and kappa (D1).

**Exact structure.**  In the X basis of the slots, write F for the set of slots in |->.  Then -J n_e X_e is
diagonal, Z_e Z_e' toggles e and e' in F, and X_p does not touch F.
  * kappa = 0: every F is conserved; in the sector F = {} the model is exactly a 12-qubit model on the icosahedron,
        H_geo = -Gamma sum X_p + g sum n_p - J sum_<pq> n_p n_q ,   n_p = 1 - sigma_p        (4096 states).
  * kappa != 0: solved here in the space |F| in {0, 2} (4096 x 436 states).  This is **variational** (an upper bound
    for the ground energy) and correct to second order in kappa; the weight outside F = {} is reported, and the
    truncation is checked against exact diagonalisation at fixed sigma.

**The flip channel (S4).**  U_p = X_p on this space: a permutation of basis states, unitary, its own inverse.
Energy of a flip: sudden (old matter state, new H) and relaxed (difference of matter ground energies).

Run:  HEXASPHERE_DIR=... python3 t12_quantum_geometry.py        (`--tests-only` for the checks)
"""
from __future__ import annotations

import itertools
import json
import os
import sys

import numpy as np
import scipy.sparse.linalg as sla

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import t12_hamiltonian as Hm     # noqa: E402

NS, NE = 4096, 30
PAIRS = list(itertools.combinations(range(NE), 2))
IU = (np.array([a for a, _ in PAIRS]), np.array([b for _, b in PAIRS]))
NF = 1 + len(PAIRS)


class Model:
    def __init__(self, W, edges, J=1.0, kappa=0.0, Gamma=0.3, g=0.0):
        self.W, self.edges, self.J, self.kappa, self.Gamma, self.g = W, edges, J, kappa, Gamma, g
        self.Pm = Hm.presence(edges).astype(float)                       # [sigma, e]
        s = np.arange(NS)
        self.n0 = np.array([12 - bin(x).count("1") for x in s], float)   # rotors in state 0
        self.n00 = self.Pm.sum(1)
        self.M = self.W[None] * self.Pm[:, :, None] * self.Pm[:, None, :]  # [sigma, e, e'] present couplings
        base = g * self.n0 - J * self.n00
        self.d0 = base                                                   # F = {}
        self.d2 = base[:, None] + 2 * J * (self.Pm[:, IU[0]] + self.Pm[:, IU[1]])   # F = {a, b}
        self.Mu = self.M[:, IU[0], IU[1]]                                # [sigma, pair]

    # geometry-only model (kappa = 0, F = {})
    def H_geo(self):
        H = np.diag(self.d0)
        idx = np.arange(NS)
        for p in range(12):
            H[idx, idx ^ (1 << p)] -= self.Gamma
        return H

    def _flip(self, psi):
        """sum_p X_p on the sigma axis of psi[sigma, ...]."""
        out = np.zeros_like(psi)
        idx = np.arange(NS)
        for p in range(12):
            out += psi[idx ^ (1 << p)]
        return out

    def matvec(self, x):
        psi = x.reshape(NS, NF)
        a0, a2 = psi[:, 0], psi[:, 1:]
        out = -self.Gamma * self._flip(psi)
        out[:, 0] += self.d0 * a0
        out[:, 1:] += self.d2 * a2
        if self.kappa:
            k = self.kappa
            out[:, 0] -= k * (self.Mu * a2).sum(1)
            out[:, 1:] -= k * self.Mu * a0[:, None]
            A = np.zeros((NS, NE, NE))
            A[:, IU[0], IU[1]] = a2
            A = A + A.transpose(0, 2, 1)
            B = A @ self.M
            B = B + B.transpose(0, 2, 1)
            out[:, 1:] -= k * B[:, IU[0], IU[1]]
        return out.reshape(-1)

    def ground(self, k=2):
        if self.kappa == 0:
            w, v = np.linalg.eigh(self.H_geo())
            psi = np.zeros((NS, NF))
            psi[:, 0] = v[:, 0]
            return w[:k], psi
        op = sla.LinearOperator((NS * NF, NS * NF), matvec=self.matvec, dtype=float)
        w, v = sla.eigsh(op, k=k, which="SA", tol=1e-7, ncv=12)
        o = np.argsort(w)
        return w[o], v[:, o[0]].reshape(NS, NF)

    # fixed sigma (Gamma = 0): matter ground state in the truncated space
    def matter_block(self, s):
        H = np.zeros((NF, NF))
        H[0, 0] = self.d0[s]
        H[np.arange(1, NF), np.arange(1, NF)] = self.d2[s]
        H[0, 1:] = H[1:, 0] = -self.kappa * self.Mu[s]
        A = np.zeros((NF - 1, NE, NE))
        A[np.arange(NF - 1), IU[0], IU[1]] = 1
        A = A + A.transpose(0, 2, 1)
        B = A @ self.M[s]
        B = B + B.transpose(0, 2, 1)
        H[1:, 1:] -= self.kappa * B[:, IU[0], IU[1]].T
        return H


def observables(m, psi):
    prob = (psi ** 2).sum(1)                                              # P(sigma)
    s = np.arange(NS)
    n11 = np.array([sum(1 for i, j in m.edges if (x >> i) & 1 and (x >> j) & 1) for x in s], float)
    rho = psi @ psi.T                                                     # geometry reduced state
    ev = np.linalg.eigvalsh(rho)
    ev = ev[ev > 1e-14]
    # hemisphere cut of the geometry qubits (kappa = 0 only meaningful as a pure-state entropy): bits 0..5 | 6..11
    geo = psi[:, 0] / max(np.linalg.norm(psi[:, 0]), 1e-300)
    sv = np.linalg.svd(geo.reshape(64, 64), compute_uv=False) ** 2
    sv = sv[sv > 1e-14]
    return {"P_all0": float(prob[0]), "P_all1": float(prob[NS - 1]), "mean_rotors_in_0": float(prob @ m.n0),
            "mean_pairs": float(prob @ m.n00), "var_pairs": float(prob @ m.n00 ** 2 - (prob @ m.n00) ** 2),
            "mean_vertices": float(140 - prob @ n11), "weight_F2": float((psi[:, 1:] ** 2).sum()),
            "S_geometry_matter_bits": float(-(ev * np.log2(ev)).sum()), "S_geometry_6_6_bits": float(-(sv * np.log2(sv)).sum())}


def flip_energies(m, sample=None):
    """Gamma = 0. For every flip sigma -> sigma^p: relaxed dE = E0(sigma') - E0(sigma) and sudden dE = <psi0(sigma)| H_sigma'
    |psi0(sigma)> - E0(sigma) (dormant rule: the slot amplitudes are kept).  Tabulated by direction and by the number
    of edges switched."""
    ss = list(range(NS)) if sample is None else list(sample)
    need = sorted(set(ss) | {s ^ (1 << p) for s in ss for p in range(12)})
    E0, V0 = {}, {}
    for s in need:                                   # one block at a time: 4096 blocks do not fit in memory together
        w, v = np.linalg.eigh(m.matter_block(s))
        E0[s], V0[s] = w[0], v[:, 0]
    want = set(ss)
    rows = {}
    for t in need:
        Ht = None
        for p in range(12):
            s = t ^ (1 << p)
            if s not in want:
                continue
            if Ht is None:
                Ht = m.matter_block(t)
            v = V0[s]
            sudden = v @ Ht @ v - E0[s]
            dn = int(m.n00[t] - m.n00[s])
            rows.setdefault(dn, []).append((E0[t] - E0[s], sudden))
    return {str(k): {"count": len(v), "relaxed_mean": float(np.mean([a for a, _ in v])), "sudden_mean": float(np.mean([b for _, b in v])),
                     "relaxed_minmax": [float(min(a for a, _ in v)), float(max(a for a, _ in v))],
                     "sudden_minus_relaxed_max": float(max(b - a for a, b in v))} for k, v in sorted(rows.items())}


# ---- tests -------------------------------------------------------------------------------------
def _model(**kw):
    c = Hm.couplings()
    return Model(c["W"], c["edges"], **kw)


def test_matvec_is_symmetric():
    m = _model(kappa=0.4, Gamma=0.3, g=0.2)
    rng = np.random.default_rng(0)
    x, y = rng.normal(size=NS * NF), rng.normal(size=NS * NF)
    assert abs(x @ m.matvec(y) - y @ m.matvec(x)) < 1e-8 * np.linalg.norm(x) * np.linalg.norm(y)


def test_kappa_zero_classical_limit():
    m = _model(kappa=0.0, Gamma=0.0)
    w, psi = m.ground()
    assert abs(w[0] + 30) < 1e-12 and abs(psi[0, 0] ** 2 - 1) < 1e-12          # all-0 tiling, 30 pairs in |+>


def test_flip_is_a_unitary_involution_and_the_dormant_rule_returns_the_state():
    m = _model(kappa=0.4)
    rng = np.random.default_rng(2)
    psi = rng.normal(size=(NS, NF))
    idx = np.arange(NS)
    for p in (0, 5, 11):
        phi = psi[idx ^ (1 << p)]
        assert abs(np.linalg.norm(phi) - np.linalg.norm(psi)) < 1e-12           # a permutation: unitary
        assert np.array_equal(phi[idx ^ (1 << p)], psi)                           # flip then reversed flip: overlap 1


def test_truncation_against_exact_diagonalisation_at_fixed_sigma():
    m = _model(kappa=0.3)
    b_order = [0b111111000000, 0b111110100000, 0b110011001100]
    for s in b_order:
        pres = np.where(m.Pm[s] > 0)[0]
        if not 2 <= len(pres) <= 16:
            continue
        ex, _ = Hm.ground(pres, m.W, 1.0, 0.3)
        tr = np.linalg.eigvalsh(m.matter_block(s))[0]
        assert tr >= ex[0] - 1e-9                                                 # variational
        assert tr - ex[0] < 0.02 * len(pres), (s, tr, ex[0])                       # and close at this kappa


def test_flip_energy_at_kappa_zero_is_J_per_edge():
    m = _model(kappa=0.0)
    t = flip_energies(m, sample=[0, 1, 77, 4095])
    for dn, r in t.items():
        assert abs(r["relaxed_mean"] + int(dn)) < 1e-12 and abs(r["sudden_mean"] + int(dn)) < 1e-12


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main(part="all"):
    """part: 'scan' (ground states), 'flips' (truncation check and flip energies) or 'all'.  The point g = 2.5,
    kappa = 0.3, Gamma = 0.2 is left out: the two lowest states are nearly degenerate there (the cat of the two
    icosahedral tilings) and the Lanczos run did not converge in 20 minutes."""
    c = Hm.couplings()
    path = os.path.join(HERE, "results_quantum_geometry.json")
    out = json.load(open(path)) if os.path.exists(path) else {}
    if part in ("scan", "all"):
        out["scan"] = []
        for g in (0.0, 2.5):
            for kappa in (0.0, 0.3):
                for Gamma in (0.2, 1.0, 3.0):
                    if g == 2.5 and kappa and Gamma < 1:
                        continue
                    m = Model(c["W"], c["edges"], J=1.0, kappa=kappa, Gamma=Gamma, g=g)
                    w, psi = m.ground()
                    r = {"g": g, "kappa": kappa, "Gamma": Gamma, "E0": float(w[0]), "gap": float(w[1] - w[0])}
                    r.update(observables(m, psi))
                    out["scan"].append(r)
                    print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
                    json.dump(out, open(path, "w"), indent=1)
    if part in ("flips", "all"):
        m = Model(c["W"], c["edges"], J=1.0, kappa=0.3)
        out["truncation"] = []
        for s in (0b111111000000, 0b111110100000, 0b110011001100, 0b101010101010, 0b111000000000):
            pres = np.where(m.Pm[s] > 0)[0]
            if 2 <= len(pres) <= 18:
                ex, _ = Hm.ground(pres, m.W, 1.0, 0.3)
                tr = float(np.linalg.eigvalsh(m.matter_block(s))[0])
                out["truncation"].append({"sigma": s, "pairs": int(len(pres)), "exact": float(ex[0]), "truncated": tr, "error_per_pair": (tr - float(ex[0])) / len(pres)})
                print(out["truncation"][-1], flush=True)
        out["flip_energies_kappa0.3"] = flip_energies(m)
        print(json.dumps(out["flip_energies_kappa0.3"], indent=1))
        json.dump(out, open(path, "w"), indent=1)


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main(next((a for a in sys.argv[1:] if a in ("scan", "flips")), "all"))
