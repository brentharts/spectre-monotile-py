#!/usr/bin/env python3
"""
toymodel_v12: time evolution of the geometry qubits at kappa = 0 (exact).

At kappa = 0 every matter slot keeps its X eigenvalue x_e = +-1 (README, S3), so in a fixed matter sector the joint
model is a 12-qubit model on the icosahedron (n_p = 1 - sigma_p):

    H_geo[x] = -Gamma sum_p X_p + g sum_p n_p - J sum_{e = pq} x_e n_p n_q            (4096 states, dense, exact).

Experiments (all **Exact** up to floating point; times in units of 1/J):
  A  signal    flip rotor 0 in the ground state (the unitary X_0, a phason flip) and follow the pair presence
               <n_p n_q>(t) on the edges, by the distances of their two ends from rotor 0 (classes (1,1), (1,2),
               (2,2), (2,3)), and the rotor occupations <n_q>(t).  Arrival time = first time the response reaches
               half of its first maximum.  The icosahedron has diameter 3, so a "cone" is four points at most.
  A' otoc      C_d(t) = 1 - Re tr(Z_q(t) Z_0 Z_q(t) Z_0) / 4096, state independent.
  B  entropy   from the product state |all-0>: entropy of 6 cone points against the other 6, for t and -t.
  C  back-reaction   one matter slot in |-> (x_e = -1): the change of the ground-state occupations <n_p> with the
               distance of p from that edge, and of the mean numbers of pairs and vertices.

Run:  HEXASPHERE_DIR=... python3 t12_evolution.py       (`--tests-only` for the checks)
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
NS = 4096
IDX = np.arange(NS)
BIT = [((IDX >> p) & 1) for p in range(12)]          # sigma_p of every basis state
_E = {}


def edges():
    if "e" not in _E:
        import t12_hamiltonian as Hm
        _E["e"] = [tuple(e) for e in Hm.couplings()["edges"]]
    return _E["e"]


def graph_distance(E, src):
    d = {s: 0 for s in src}
    front = list(src)
    while front:
        nxt = []
        for u in front:
            for a, b in E:
                for x, y in ((a, b), (b, a)):
                    if x == u and y not in d:
                        d[y] = d[u] + 1
                        nxt.append(y)
        front = nxt
    return [d[p] for p in range(12)]


def diag(E, g, J, x=None):
    x = np.ones(len(E)) if x is None else x
    n = [1 - b for b in BIT]
    out = g * sum(n).astype(float)
    for xe, (a, b) in zip(x, E):
        out = out - J * xe * (n[a] * n[b])
    return out


def H_geo(E, Gamma, g, J=1.0, x=None):
    H = np.diag(diag(E, g, J, x))
    for p in range(12):
        H[IDX, IDX ^ (1 << p)] -= Gamma
    return H


class Evolver:
    def __init__(self, E, Gamma, g, J=1.0, x=None):
        self.E = E
        self.w, self.V = np.linalg.eigh(H_geo(E, Gamma, g, J, x))

    def ground(self):
        return self.V[:, 0].copy()

    def evolve(self, psi, t):
        return self.V @ (np.exp(-1j * self.w * t) * (self.V.T @ psi))


def occupations(psi):
    p = np.abs(psi) ** 2
    return np.array([p @ (1 - b) for b in BIT])


def pair_and_vertex_numbers(E, psi):
    p = np.abs(psi) ** 2
    n00 = sum((1 - BIT[a]) * (1 - BIT[b]) for a, b in E)
    n11 = sum(BIT[a] * BIT[b] for a, b in E)
    return float(p @ n00), float(140 - p @ n11)


def entropy_6_6(psi):
    s = np.linalg.svd(psi.reshape(64, 64), compute_uv=False) ** 2
    s = s[s > 1e-15]
    return float(-(s * np.log2(s)).sum())


EDGE_CLASSES = [(0, 1), (1, 1), (1, 2), (2, 2), (2, 3)]        # (distances of the two ends from the flipped rotor)
EDGE_MEAN_DISTANCE = [0.5, 1.0, 1.5, 2.0, 2.5]


def signal(ev, ts, src=0):
    """Response to the flip X_src of the ground state: (rotor, edge) with rotor[t, d] the change of <n_q> averaged over
    the rotors at distance d = 0..3 and edge[t, c] the change of the pair presence <n_p n_q> averaged over the edges
    of class c (EDGE_CLASSES).  At g = 5J/2 the model has the symmetry 0 <-> 1 and the rotor response is zero."""
    d = np.array(graph_distance(ev.E, [src]))
    cls = [EDGE_CLASSES.index(tuple(sorted((d[a], d[b])))) for a, b in ev.E]
    nn = [(1 - BIT[a]) * (1 - BIT[b]) for a, b in ev.E]
    psi0 = ev.ground()
    p0 = psi0 ** 2
    base_r, base_e = occupations(psi0), np.array([p0 @ x for x in nn])
    psi = psi0[IDX ^ (1 << src)]
    rot, edg = np.zeros((len(ts), 4)), np.zeros((len(ts), 5))
    for i, t in enumerate(ts):
        phi = ev.evolve(psi, t)
        p = np.abs(phi) ** 2
        dn = occupations(phi) - base_r
        de = np.array([p @ x for x in nn]) - base_e
        for k in range(4):
            rot[i, k] = dn[d == k].mean()
        for c in range(5):
            edg[i, c] = de[[j for j in range(len(cls)) if cls[j] == c]].mean()
    return rot, edg


def arrival(ts, y):
    """First time |y| reaches half of its first local maximum (None if it never rises above 1e-6)."""
    a = np.abs(y)
    peak = next((i for i in range(1, len(a) - 1) if a[i] >= a[i - 1] and a[i] > a[i + 1] and a[i] > 1e-6), None)
    if peak is None:
        return None
    i = int(np.argmax(a[:peak + 1] >= 0.5 * a[peak]))
    return float(ts[i])


def otoc(ev, ts, src=0):
    """C_d(t) = 1 - Re tr(Z_q(t) Z_src Z_q(t) Z_src) / dim for one rotor q at each distance 1, 2, 3."""
    d = graph_distance(ev.E, [src])
    Z = lambda p: (1 - 2 * BIT[p]).astype(float)
    qs = [d.index(k) for k in (1, 2, 3)]
    Vt = ev.V.T
    Aq = [Vt @ (Z(q)[:, None] * ev.V) for q in qs]              # Z_q in the eigenbasis
    B = Vt @ (Z(src)[:, None] * ev.V)
    out = np.zeros((len(ts), 3))
    for i, t in enumerate(ts):
        ph = np.exp(1j * ev.w * t)
        for k, A in enumerate(Aq):
            M = ((ph[:, None] * A) * ph.conj()[None, :]) @ B     # Z_q(t) Z_src
            out[i, k] = 1 - float(np.real(np.sum(M * M.T))) / NS
    return out


def back_reaction(E, Gamma, g, J=1.0, e0=0):
    """Ground state with slot e0 in |-> against the one with all slots in |+>."""
    x = np.ones(len(E))
    x[e0] = -1
    a, b = Evolver(E, Gamma, g, J), Evolver(E, Gamma, g, J, x)
    d = np.array(graph_distance(E, list(E[e0])))
    na, nb = occupations(a.ground()), occupations(b.ground())
    pa, va = pair_and_vertex_numbers(E, a.ground())
    pb, vb = pair_and_vertex_numbers(E, b.ground())
    return {"dn_by_distance": {str(k): float((nb - na)[d == k].mean()) for k in sorted(set(d.tolist()))},
            "rotors_at_distance": {str(k): int((d == k).sum()) for k in sorted(set(d.tolist()))},
            "excitation_energy": float(b.w[0] - a.w[0]), "d_pairs": pb - pa, "d_vertices": vb - va}


# ---- tests -----------------------------------------------------------------------------------
def test_icosahedron_distances():
    d = graph_distance(edges(), [0])
    assert sorted(d) == [0] + [1] * 5 + [2] * 5 + [3]


def test_evolution_is_unitary_and_time_reversal_symmetric():
    ev = Evolver(edges(), 1.0, 0.0)
    psi = np.zeros(NS)
    psi[0] = 1.0
    a, b = ev.evolve(psi, 1.7), ev.evolve(psi, -1.7)
    assert abs(np.linalg.norm(a) - 1) < 1e-12 and np.allclose(a, b.conj())          # real H, real state
    assert abs(entropy_6_6(a) - entropy_6_6(b)) < 1e-10
    assert np.allclose(ev.evolve(a, -1.7), psi, atol=1e-10)


def test_no_tunnelling_no_signal():
    ev = Evolver(edges(), 0.0, 0.0)
    s, e = signal(ev, np.array([0.0, 1.0, 3.0]))
    assert np.allclose(s[:, 1:], 0) and np.allclose(s[:, 0], -1) and np.allclose(e[:, 1:], 0)                    # the flipped rotor stays flipped


def test_signal_reaches_neighbours_before_the_antipode():
    ev = Evolver(edges(), 1.0, 2.5)
    ts = np.linspace(0, 6, 121)
    s, e = signal(ev, ts)
    t1, t3 = arrival(ts, e[:, 1]), arrival(ts, e[:, 4])
    assert t1 is not None and t3 is not None and t1 < t3
    assert np.allclose(e[0, 1:], 0, atol=1e-12) and np.allclose(s, 0, atol=1e-9)     # symmetric point: rotors silent


def test_excited_slot_repels_pairing_at_its_ends():
    r = back_reaction(edges(), 1.0, 0.0)
    assert r["dn_by_distance"]["0"] < 0 and r["excitation_energy"] > 0 and r["d_pairs"] < 0


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__, flush=True)
    print("%d checks pass" % len(fs))


def main():
    E = edges()
    ts = np.linspace(0, 12, 241)
    out = {"times": ts.tolist(), "signal": [], "otoc": [], "entropy": [], "back_reaction": []}
    for Gamma, g in ((0.3, 2.5), (1.0, 2.5), (3.0, 2.5), (1.0, 0.0), (3.0, 0.0)):
        ev = Evolver(E, Gamma, g)
        s, e = signal(ev, ts)
        arr = [arrival(ts, e[:, k]) for k in range(1, 5)]
        ok = all(a is not None for a in arr)
        r = {"Gamma": Gamma, "g": g, "edge_mean_distance": EDGE_MEAN_DISTANCE[1:], "edge_arrival": arr,
             "edge_max_response": [float(np.abs(e[:, k]).max()) for k in range(1, 5)],
             "speed_fit": float(np.polyfit(arr, EDGE_MEAN_DISTANCE[1:], 1)[0]) if ok and len(set(arr)) > 1 else None,
             "rotor_arrival_d1_d2_d3": [arrival(ts, s[:, k]) for k in (1, 2, 3)],
             "rotor_response": s.tolist(), "edge_response": e.tolist()}
        out["signal"].append(r)
        print({k: v for k, v in r.items() if not k.endswith("response")}, flush=True)
        psi = np.zeros(NS)
        psi[0] = 1.0
        S = [entropy_6_6(ev.evolve(psi, t)) for t in ts]
        Sm = [entropy_6_6(ev.evolve(psi, -t)) for t in ts[::20]]
        long = [entropy_6_6(ev.evolve(psi, t)) for t in np.linspace(50, 400, 60)]
        e = {"Gamma": Gamma, "g": g, "S_at_1_3_6_12": [S[20], S[60], S[120], S[240]], "max_S": max(S), "time_of_max": float(ts[int(np.argmax(S))]),
             "long_time_mean": float(np.mean(long)), "long_time_min": float(np.min(long)), "reversal_asymmetry": float(max(abs(a - b) for a, b in zip(S[::20], Sm))), "S": S}
        out["entropy"].append(e)
        print({k: v for k, v in e.items() if k != "S"}, flush=True)
        b = back_reaction(E, Gamma, g)
        b.update({"Gamma": Gamma, "g": g})
        out["back_reaction"].append(b)
        print(b, flush=True)
    to = np.linspace(0, 6, 25)
    for Gamma, g in ((1.0, 2.5),):
        C = otoc(Evolver(E, Gamma, g), to)
        r = {"Gamma": Gamma, "g": g, "times": to.tolist(), "C_d1_d2_d3": C.tolist(), "arrival_half_of_max": [float(to[int(np.argmax(C[:, k] >= 0.5 * C[:, k].max()))]) for k in range(3)]}
        out["otoc"].append(r)
        print({k: v for k, v in r.items() if k not in ("C_d1_d2_d3", "times")}, flush=True)
    json.dump(out, open(os.path.join(HERE, "results_evolution.json"), "w"))


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        main()
