#!/usr/bin/env python3
"""t7_entropy.py - Stage 7 of toymodel_v7: entropy and the arrow in a unitary dynamics.

(a) A time-reversal audit of the Hamiltonians of toymodel_v6.  (b) A quench: the pairs of a patch cluster start in their frozen classical state (a product state: every pair has a definite bump) and evolve unitarily;
the entanglement entropy of half of them is the entropy that can grow.  The initial condition is the input, named as such: no arrow is derived from the dynamics.

Definitions
  D7.11  Time reversal is complex conjugation K in the bit basis (spinless bits).  A term is T-odd if it is not invariant under K; a real Hamiltonian is T-even.
  D7.12  Quench: the central 12 pairs of the level-3 patch (v6 Stage 1), H = -c(tau) sum tau^x + sum K_AB tau^z tau^z at flow time tau, initial state the classical ground state |s> of the couplings at that tau
         (Gamma = 0), half = the 6 pairs with the smallest x of the pair midpoint.  S(t) = entanglement entropy of the half, in bits.

Exact / Numeric claims (each a test; ``python3 t7_entropy.py --tests-only``)
  E7.22  No T-odd term comes from the tile.  Every Hamiltonian of v6 is real in the bit basis: the tile Hamiltonian with the ledger, the pair qubits, the patch couplings, the polaron, the hopping Laplacian; the gauge form uses
         X and Z only.  The patch couplings are mirror invariant (reflecting the centroids leaves W unchanged): the Spectre's chirality is a parity, not a time reversal.  A real family of Hamiltonians has Berry phases 0 or pi
         only, so no continuous T-odd flux can come from moving the tile's parameters (checked on a loop in the (Gamma, K) plane of two pairs: phase 0 to 1e-10).  **Negative**: a T-odd term must be put in (the area
         term theta K of the paper is one, and it needs a flux).
  E7.23  The arrow points away from the initial moment, both ways.  For a real H and a real initial state psi(-t) = psi(t)*, so S(-t) = S(t) exactly (to 1e-12 at every tau tested): entropy grows into the past as into the
         future from the frozen state.  This is the Janus-point structure the paper cites; the direction is set by where the low-entropy state is put.
  E7.24  The growth depends on the flow.  From the frozen product state S rises from 0 at every tau; its late-time mean (t in [20, 40]) is 0.17, 2.44, 2.90 and 1.90 bits at tau = 0, 0.1, 0.3, 0.6, below the Page value 5.28:
         smallest at the frozen (Spectre) end, largest mid-flow (3.45 at tau = 0.2 in the figure's scan), slow at the plastic end (half of the late value only at t = 14.5 for tau = 0.6; 0.38 and 0.04 bits by t = 40
         at tau = 0.8, 0.9, where the coupling between pairs vanishes).  The frozen end keeps its low entropy because its eigenstates are nearly classical: records survive there.

Status.  E7.22, E7.23: Exact.  E7.24: Numeric (exact evolution of a 4096-state cluster).  The initial condition is put in; nothing here derives it.

    python3 t7_entropy.py --tests-only
    python3 t7_entropy.py --no-show --save figures/t7_stage7_entropy.png
"""
import argparse
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_RES = os.path.dirname(_HERE)
for _p in (_HERE, os.path.join(_RES, "toymodel_v6"), os.path.join(_RES, "toymodel_v4"), os.path.join(_RES, "toymodel_v3"), os.path.dirname(_RES), os.path.join(os.path.dirname(_RES), "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t6_pair as t0  # noqa: E402
import t6_patch as tp6  # noqa: E402

N = 12
TAUS = (0.0, 0.1, 0.3, 0.6)
PAGE = 6 - 1 / (2 * math.log(2))
_C = {}


def setup(tau):
    if tau in _C:
        return _C[tau]
    idx = tp6.cluster(3, N)
    K, mid = tp6.couplings_at(tau, 3, idx)
    H = tp6.hamiltonian(t0.overlap(tau), K).toarray()
    E, V = np.linalg.eigh(H)
    _, s, _ = tp6.classical_ground(K)
    psi0 = np.zeros(1 << N)
    psi0[int(sum(1 << a for a in range(N) if s[a] < 0))] = 1.0
    half = sorted(np.argsort(mid[:, 0], kind="stable")[: N // 2].tolist())
    _C[tau] = (E, V, V.T @ psi0, half, H)
    return _C[tau]


def entropy(psi, half):
    t = psi.reshape([2] * N)
    axA = [N - 1 - a for a in half]
    axB = [N - 1 - b for b in range(N) if b not in half]
    m = np.transpose(t, axA + axB).reshape(1 << len(half), -1)
    p = np.linalg.svd(m, compute_uv=False) ** 2
    p = p[p > 1e-15]
    return float(-(p * np.log2(p)).sum())


def S_of(tau, ts):
    E, V, a, half, _ = setup(tau)
    return np.array([entropy(V @ (np.exp(-1j * E * t) * a), half) for t in ts])


# --------------------------------------------------------------------------
def test_e722_no_t_odd_term_comes_from_the_tile():
    import t6_clock as tc
    import t6_gauge as tg
    mats = [t0.tile_hamiltonian(1.0, 3.0, 0.7, 0.4), t0.effective_hamiltonian(0.3, 0.8), setup(0.1)[4], tc.polaron_hamiltonian(4, 1.0, 10.0, 0.6, "link")]
    for M in mats:
        assert np.abs(np.imag(np.asarray(M))).max() == 0.0 and np.allclose(M, np.asarray(M).T)
    # the gauge form uses X and Z only: its stabilizer matrices are 0/1 and there is no Y
    c = tg.complex_(2)
    assert set(np.unique(tg.gauss_matrix(c))) <= {0, 1} and set(np.unique(tg.plaquette_matrix(c))) <= {0, 1}
    # mirror invariance of the patch couplings
    pr, C, _ = tp6.geometry(3, 1.0)
    W, _ = tp6.coupling(3, 1.0)
    Cm = C * np.array([-1.0, 1.0])
    A, B = Cm[[p[0] for p in pr]], Cm[[p[1] for p in pr]]
    inv = lambda X, Y: np.where(np.eye(len(X)) > 0, 0.0, 1.0 / np.maximum(np.linalg.norm(X[:, None] - Y[None], axis=2), 1e-300))  # noqa: E731
    Wm = inv(A, A) - inv(A, B) - inv(B, A) + inv(B, B)
    np.fill_diagonal(Wm, 0.0)
    assert np.allclose(W, Wm, atol=1e-12)
    # Berry phase of the two-pair ground state around a loop in the (Gamma, K) plane
    th = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    states = []
    for x in th:
        G, K = 1.0 + 0.5 * math.cos(x), 1.0 + 0.5 * math.sin(x)
        _, psi = t0.ground_state(t0.effective_hamiltonian(G, K))
        states.append(psi)
    prod = 1.0 + 0j
    for i in range(len(states)):
        prod *= np.vdot(states[i], states[(i + 1) % len(states)])
    phase = abs(np.angle(prod))
    assert phase < 1e-10 or abs(phase - math.pi) < 1e-10, phase
    return "all real; mirror-invariant W; Berry phase %.1e" % phase


def test_e723_the_arrow_points_away_from_the_initial_moment():
    worst = 0.0
    for tau in TAUS:
        for t in (2.0, 5.0, 17.0):
            a, b = S_of(tau, [t])[0], S_of(tau, [-t])[0]
            worst = max(worst, abs(a - b))
    assert worst < 1e-12, worst
    return float("%.1e" % worst)


def test_e724_the_growth_depends_on_the_flow():
    ts = np.linspace(0, 40, 81)
    late, half_t = {}, {}
    for tau in TAUS:
        S = S_of(tau, ts)
        assert S[0] < 1e-12 and S[1] > 0
        late[tau] = float(np.mean(S[40:]))
        half_t[tau] = float(ts[np.argmax(S > 0.5 * late[tau])])
    want = {0.0: 0.173, 0.1: 2.444, 0.3: 2.899, 0.6: 1.904}
    assert all(abs(late[k] - v) < 2e-3 for k, v in want.items()), late
    assert late[0.0] < late[0.6] < late[0.1] < late[0.3] < PAGE
    assert abs(half_t[0.6] - 14.5) < 0.6
    return {k: round(v, 3) for k, v in late.items()}, {k: round(v, 1) for k, v in half_t.items()}


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e7")]
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

    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4), gridspec_kw={"width_ratios": [1.5, 1, 1]})
    ts = np.linspace(-40, 40, 161)
    for tau, col in zip(TAUS, ("tab:blue", "tab:green", "tab:orange", "tab:red")):
        ax[0].plot(ts, S_of(tau, ts), color=col, label="tau = %.1f (c = %.2f)" % (tau, t0.overlap(tau)))
    ax[0].axhline(PAGE, color="k", ls=":", lw=0.8); ax[0].text(39, PAGE - 0.3, "Page value", ha="right", fontsize=7)
    ax[0].axvline(0, color="k", lw=0.6)
    ax[0].set_xlabel("time t (the frozen product state at t = 0)"); ax[0].set_ylabel("entropy of half the cluster (bits)"); ax[0].legend(fontsize=7)
    ax[0].set_title("(a) entropy grows away from the initial state, into the future and the past", fontsize=9)
    tt = np.linspace(20, 40, 41)
    taus = np.linspace(0, 0.9, 10)
    ax[1].plot(taus, [np.mean(S_of(t, tt)) for t in taus], "o-k")
    ax[1].set_xlabel("flow time tau (0 Spectre, 1 Hat)"); ax[1].set_ylabel("late-time entropy (bits)")
    ax[1].set_title("(b) low at both ends (frozen: nearly classical;\nplastic: the coupling vanishes), largest mid-flow", fontsize=9)
    ax[2].axis("off")
    ax[2].text(0.0, 1.0, "time-reversal audit (bit basis):\n\n  tile Hamiltonian + ledger   real\n  pair qubits                 real\n  patch couplings (dipoles)   real,\n                              mirror-invariant\n  polaron clock               real\n  hopping Laplacian           real\n  gauge form                  X and Z only\n\n  Berry phases of real families: 0 or pi\n\n  -> no T-odd term from the tile;\n     the arrow is the initial condition",
               va="top", family="monospace", fontsize=8.5)
    fig.suptitle("toymodel_v7 Stage 7: entropy and the arrow in a unitary dynamics")
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
