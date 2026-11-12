#!/usr/bin/env python3
"""t6_pair.py - Stage 0 of toymodel_v6: a quantum Hamiltonian for the bits of the 14th vertex, on one and on two bump-and-dent pairs.

Each tile carries a qubit, sigma^z = +1 (convex) / -1 (reflex).  toymodel_v4 found that on a tiling the bit can open only on a pair of tiles that share the flat vertex, as a bump and a dent:
the area ledger of the pair, Q = sigma^z_i + sigma^z_j, is 0.  The proposed bit Hamiltonian (the pasted proposal, section 2B and 2C) is, on the tiles,

    H = - J sum_pairs sigma^x_i sigma^x_j  +  g sum_pairs (sigma^z_i + sigma^z_j)^2  -  K sum_<A,B> sigma^z_{A1} sigma^z_{B1}  -  eps sum_i sigma^z_i          (H)

with the penalty g for the ledger (the constraint term, here in its simplest form: the ledger itself and not a Wilson loop), a coupling K between the first tiles of neighbouring pairs
and a uniform field eps (the area term a h of the proposal).  All of (H) is a **Definition**.  Nothing here is physics.

Definitions
  D6.1  Pair qubit.  In the sector Q = 0 a pair has two states, |+-> and |-+>; tau^z = sigma^z of its first tile, tau^x exchanges them.
  D6.2  Effective Hamiltonian (exact in the sector Q = 0):  H_eff = - Gamma sum_A tau^x_A - K sum_<A,B> tau^z_A tau^z_B, Gamma = J.
  D6.3  Flow.  Gamma(tau) = J0 c(tau), c = exp(-d^2 / 8 sigma^2), d = 2 h(tau) (the overlap of the two positions of the shared vertex, toymodel_v2's quantum bit);
        K(tau) = K0 (h(tau) / h(0))^2 (two sources of strength h, toymodel_v5 Stage 3).  h(tau) = tan(omega0 (1 - tau) / 2), omega0 = 120 degrees, sigma = 1, a = 1.
  D6.4  Read-out visibility of a pair qubit: v = 1 - c^2 per correlation (two unsharp read-outs of Helstrom bias sqrt(1 - c^2) each).  CHSH_obs = v * CHSH_state.

Exact claims (each a test; ``python3 t6_pair.py --tests-only``)
  E6.1  The constraint turns the Ising term into a tunnelling term, exactly.  sigma^x sigma^x flips both tiles of a pair, so it commutes with Q^2 and maps the sector Q = 0 to itself:
        for two pairs (four tiles, 16 states) the spectrum of H_eff is contained in that of (H) to 1e-12 for every g, and is its four lowest levels once g is of order J (g = 5, J = 1).
        No limit g -> infinity is needed.  The field eps vanishes identically in the sector: the area term cannot bias a pair.
  E6.2  One pair cannot show a Bell violation.  The only one-tile observables that commute with the ledger are functions of sigma^z; with them the correlations of the pair are those of
        a shared classical bit (E = -1, CHSH = 2 exactly, never more).  A violation needs two pairs.
  E6.3  Two pairs.  The ground state of H_eff is cos(t)|00> + sin(t)|11> in the tau^x-parity-even sector with concurrence C = K / sqrt(4 Gamma^2 + K^2) (closed form, checked to 1e-12), and its
        largest CHSH value over all read-outs is 2 sqrt(1 + C^2) <= 2 sqrt 2 (Horodecki's formula, checked against a direct optimisation).  Tsirelson's bound holds for every state of every
        such Hamiltonian (2000 random states): it is a property of the Hilbert space, not of the constraint.  The gap above the ground state is sqrt(4 Gamma^2 + K^2) - K.
  E6.4  Read-outs on six directions.  For the maximally entangled state and read-outs restricted to six directions 60 degrees apart in a plane the largest CHSH value is 5/2 (the paper's
        hexagonal value for the singlet), not 2 sqrt 2, which needs 45 degrees.
  E6.5  Along the flow (D6.3, D6.4; J0 = K0 = 1).  C falls strictly from 0.913 at the Spectre to 0 at the Hat; CHSH_state from 2.708 to 2; the observable value from 2.574 to 0, passing 2
        at tau_2 = 0.102.  The window of violation is at the Spectre end and closes toward the Hat, as for the classical rule of toymodel_v5 Stage 1, now below 2 sqrt 2 throughout.
        The gap grows along the flow (0.095 at the Spectre, 2 at the Hat): the entangled end is the fragile one.

Status.  Exact (finite matrices) for E6.1 to E6.4; E6.5 is exact given D6.3 and D6.4, which are Definitions (the forms of Gamma, K and v are choices).

    python3 t6_pair.py --tests-only
    python3 t6_pair.py --no-show --save figures/t6_stage0_pair.png
"""
import argparse
import itertools
import math
import sys

import numpy as np

SX = np.array([[0, 1], [1, 0]], dtype=complex)
SY = np.array([[0, -1j], [1j, 0]])
SZ = np.array([[1, 0], [0, -1]], dtype=complex)
I2 = np.eye(2, dtype=complex)
PAULI = (SX, SY, SZ)
OMEGA0, SIGMA = 120.0, 1.0
TSIRELSON = 2 * math.sqrt(2)


def op(single, site, n):
    out = np.array([[1.0 + 0j]])
    for k in range(n):
        out = np.kron(out, single if k == site else I2)
    return out


def tile_hamiltonian(J=1.0, g=10.0, K=0.5, eps=0.0):
    """(H) on two pairs: tiles (0, 1) and (2, 3); K couples tiles 0 and 2."""
    n = 4
    H = np.zeros((16, 16), dtype=complex)
    for i, j in ((0, 1), (2, 3)):
        H -= J * op(SX, i, n) @ op(SX, j, n)
        q = op(SZ, i, n) + op(SZ, j, n)
        H += g * q @ q
    H -= K * op(SZ, 0, n) @ op(SZ, 2, n)
    for i in range(n):
        H -= eps * op(SZ, i, n)
    return H


def effective_hamiltonian(Gamma=1.0, K=0.5):
    return -Gamma * (np.kron(SX, I2) + np.kron(I2, SX)) - K * np.kron(SZ, SZ)


def ground_state(H):
    w, v = np.linalg.eigh(H)
    return w, v[:, 0]


def concurrence(psi):
    a = psi.reshape(2, 2)
    return float(2 * abs(a[0, 0] * a[1, 1] - a[0, 1] * a[1, 0]))


def corr_matrix(psi):
    rho = np.outer(psi, psi.conj())
    return np.array([[float(np.real(np.trace(rho @ np.kron(a, b)))) for b in PAULI] for a in PAULI])


def chsh_max(psi):
    """Horodecki: 2 sqrt(m1 + m2), the two largest eigenvalues of T^t T."""
    T = corr_matrix(psi)
    m = np.sort(np.linalg.eigvalsh(T.T @ T))[::-1]
    return float(2 * math.sqrt(max(m[0] + m[1], 0.0)))


def chsh_direct(psi, trials=200, seed=0):
    """Direct optimisation over read-out directions (alternating maximisation from random starts)."""
    T = corr_matrix(psi)
    rng = np.random.default_rng(seed)
    best = 0.0
    for _ in range(trials):
        a, a2 = rng.standard_normal(3), rng.standard_normal(3)
        a, a2 = a / np.linalg.norm(a), a2 / np.linalg.norm(a2)
        for _ in range(60):
            b = T.T @ (a + a2); b2 = T.T @ (a - a2)
            b, b2 = b / max(np.linalg.norm(b), 1e-300), b2 / max(np.linalg.norm(b2), 1e-300)
            a = T @ (b + b2); a2 = T @ (b - b2)
            a, a2 = a / max(np.linalg.norm(a), 1e-300), a2 / max(np.linalg.norm(a2), 1e-300)
        best = max(best, float(a @ T @ b + a @ T @ b2 + a2 @ T @ b - a2 @ T @ b2))
    return best


def chsh_hexagonal(psi):
    """Largest CHSH with all four read-outs among six directions 60 degrees apart in the x-z plane."""
    T = corr_matrix(psi)
    dirs = [np.array([math.sin(k * math.pi / 3), 0.0, math.cos(k * math.pi / 3)]) for k in range(6)]
    E = np.array([[u @ T @ v for v in dirs] for u in dirs])
    return max(abs(E[a, b] + E[a, b2] + E[a2, b] - E[a2, b2]) for a, a2, b, b2 in itertools.product(range(6), repeat=4))


def h(tau):
    return math.tan(math.radians(OMEGA0 * (1.0 - tau)) / 2.0)


def overlap(tau):
    return math.exp(-(2 * h(tau)) ** 2 / (8 * SIGMA ** 2))


def flow(tau, J0=1.0, K0=1.0):
    """(Gamma, K, C, CHSH_state, CHSH_obs, gap) at flow time tau."""
    c = overlap(tau)
    G, K = J0 * c, K0 * (h(tau) / h(0.0)) ** 2
    C = K / math.sqrt(4 * G * G + K * K)
    s = 2 * math.sqrt(1 + C * C)
    return G, K, C, s, (1 - c * c) * s, math.sqrt(4 * G * G + K * K) - K


def tau_two(J0=1.0, K0=1.0):
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if flow(mid, J0, K0)[4] > 2 else (lo, mid)
    return 0.5 * (lo + hi)


# --------------------------------------------------------------------------
def test_e61_the_constraint_turns_ising_into_tunnelling():
    J, K = 1.0, 0.5
    target = np.linalg.eigvalsh(effective_hamiltonian(J, K))
    n = 4
    q1, q2 = op(SZ, 0, n) + op(SZ, 1, n), op(SZ, 2, n) + op(SZ, 3, n)
    errs = []
    for g in (0.0, 0.3, 5.0, 100.0):
        H = tile_hamiltonian(J, g, K)
        assert np.abs(H @ (q1 @ q1) - (q1 @ q1) @ H).max() < 1e-12 and np.abs(H @ (q2 @ q2) - (q2 @ q2) @ H).max() < 1e-12   # the sector is invariant
        w = np.linalg.eigvalsh(H)
        errs.append(max(float(np.abs(w - t).min()) for t in target))          # the spectrum of H_eff is contained in that of (H), exactly
        assert errs[-1] < 1e-12
    w = np.linalg.eigvalsh(tile_hamiltonian(J, 5.0, K))
    assert np.abs(w[:4] - target).max() < 1e-12 and w[4] > w[3] + 1.0       # and it is the low-energy sector once g is of order J
    # the uniform field vanishes on every state with Q = 0 on both pairs, and the low levels do not move
    w1 = np.linalg.eigvalsh(tile_hamiltonian(J, 5.0, K, eps=0.7))
    assert np.abs(w1[:4] - target).max() < 1e-12
    return max(errs)


def test_e62_one_pair_cannot_violate():
    # one-tile observables commuting with Q = sz1 + sz2 are diagonal
    Q = np.kron(SZ, I2) + np.kron(I2, SZ)
    for P in (SX, SY):
        assert np.abs(np.kron(P, I2) @ Q - Q @ np.kron(P, I2)).max() > 1
    assert np.abs(np.kron(SZ, I2) @ Q - Q @ np.kron(SZ, I2)).max() == 0
    psi = np.array([0, 1, 1, 0], dtype=complex) / math.sqrt(2)                  # the tunnelling ground state of a pair
    E = float(np.real(psi.conj() @ np.kron(SZ, SZ) @ psi))
    assert abs(E + 1) < 1e-15
    # with read-outs +-sz or +-1 on each side, CHSH is at most 2
    outs = [SZ, -SZ, I2, -I2]
    best = max(abs(sum(sg * float(np.real(psi.conj() @ np.kron(a, b) @ psi)) for (a, b), sg in zip(((A, B), (A, B2), (A2, B), (A2, B2)), (1, 1, 1, -1))))
               for A, A2, B, B2 in itertools.product(outs, repeat=4))
    assert abs(best - 2.0) < 1e-12
    return best


def test_e63_two_pairs_closed_form_and_tsirelson():
    for G, K in ((1.0, 0.5), (0.2, 1.0), (1.0, 0.0), (0.05, 2.0)):
        w, psi = ground_state(effective_hamiltonian(G, K))
        C = K / math.sqrt(4 * G * G + K * K)
        assert abs(concurrence(psi) - C) < 1e-10
        assert abs(chsh_max(psi) - 2 * math.sqrt(1 + C * C)) < 1e-10
        assert abs(chsh_direct(psi, trials=20) - chsh_max(psi)) < 1e-5
        assert abs((w[1] - w[0]) - (math.sqrt(4 * G * G + K * K) - K)) < 1e-10
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(2000):
        v = rng.standard_normal(4) + 1j * rng.standard_normal(4)
        worst = max(worst, chsh_max(v / np.linalg.norm(v)))
    assert worst <= TSIRELSON + 1e-12
    return worst


def test_e64_six_directions_give_five_halves():
    bell = np.array([1, 0, 0, 1], dtype=complex) / math.sqrt(2)
    assert abs(chsh_max(bell) - TSIRELSON) < 1e-12
    assert abs(chsh_hexagonal(bell) - 2.5) < 1e-12
    return chsh_hexagonal(bell)


def test_e65_the_window_is_at_the_spectre_end():
    taus = np.linspace(0, 1, 201)
    F = np.array([flow(t) for t in taus])
    assert np.all(np.diff(F[:, 2]) < 0) and abs(F[0, 2] - 0.913) < 1e-3 and F[-1, 2] == 0
    assert abs(F[0, 3] - 2.708) < 1e-3 and abs(F[-1, 3] - 2.0) < 1e-12
    assert abs(F[0, 4] - 2.574) < 1e-3 and abs(F[-1, 4]) < 1e-12
    assert np.all(F[:, 3] <= TSIRELSON)
    t2 = tau_two()
    assert abs(t2 - 0.102) < 1e-3 and flow(t2 - 1e-3)[4] > 2 > flow(t2 + 1e-3)[4]
    assert np.all(np.diff(F[:, 5]) > 0) and abs(F[0, 5] - 0.095) < 1e-3 and abs(F[-1, 5] - 2.0) < 1e-12
    return t2


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

    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    gs = np.linspace(0, 3, 61)
    target = np.linalg.eigvalsh(effective_hamiltonian(1.0, 0.5))
    W = np.array([np.linalg.eigvalsh(tile_hamiltonian(1.0, g, 0.5)) for g in gs])
    for k in range(16):
        ax[0].plot(gs, W[:, k], color="0.7", lw=0.8)
    for t in target:
        ax[0].axhline(t, color="r", lw=1.2)
    ax[0].set_xlabel("penalty g of the ledger"); ax[0].set_ylabel("levels of four tiles (grey); two pair qubits (red)")
    ax[0].set_title("(a) the ledger sector is exact for every g", fontsize=9)
    taus = np.linspace(0, 1, 201)
    F = np.array([flow(t) for t in taus])
    ax[1].plot(taus, F[:, 3], "b", label="CHSH of the ground state")
    ax[1].plot(taus, F[:, 4], "r", label="with read-out visibility 1 - c$^2$")
    ax[1].axhline(2, color="k", lw=0.8); ax[1].axhline(TSIRELSON, color="k", ls="--", lw=0.8)
    ax[1].axvline(tau_two(), color="r", ls=":")
    ax[1].set_xlabel("tau (0 Spectre, 1 Hat)"); ax[1].set_ylabel("CHSH"); ax[1].legend(fontsize=7)
    ax[1].set_title("(b) two pairs along the flow: the window closes toward the Hat", fontsize=9)
    ax[2].plot(taus, F[:, 0], label="tunnelling Gamma = J0 c")
    ax[2].plot(taus, F[:, 1], label="coupling K = K0 (h/h0)$^2$")
    ax[2].plot(taus, F[:, 2], "k", label="concurrence C")
    ax[2].plot(taus, F[:, 5], "g--", label="gap")
    ax[2].set_xlabel("tau"); ax[2].legend(fontsize=7); ax[2].set_title("(c) the entangled end is the fragile one (small gap)", fontsize=9)
    fig.suptitle("toymodel_v6 Stage 0: a Hamiltonian for bump-and-dent pairs")
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
    if a.tests_only:
        return 1 if run_tests() else 0
    failed = run_tests()
    figure(a.save, not a.no_show)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
