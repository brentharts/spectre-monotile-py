#!/usr/bin/env python3
"""t3_qubit.py - Stage 4 of research/toymodel_v3/PLAN.md (item 4.2): a read-out in which the setting acts on the state, with nothing discarded.

Stage 4b of toymodel_v2 and Stage 2 here: with a sign or pin read-out of the end letter, CHSH is <= 2 when nothing is discarded and anything up to 4 when runs are
discarded; Clauser-Horne never fails.  Tsirelson's bound needs a party to measure a +-1 observable on its own system.  Here each end letter is mapped to a spin-1/2
state and each setting is an observable, and every run is kept.

Definitions
  D4.1  Letter k (direction phi_k = k x 60 degrees) -> equatorial spin state |phi_k> = (|0> + e^{i phi_k} |1>) / sqrt 2.  Setting a (a x 30 degrees) -> the
        observable sigma(a) = cos(a) sigma_x + sin(a) sigma_y (outcomes +-1).
  D4.2  The two-ended history with a COHERENT origin: amplitude a_v on the origin letter v, wings by unitaries U_A, U_B on the six letters (U = 1 for no wing), the
        second wing starting at -v:  Psi(s_A, s_B) = sum_v a_v U_A[s_A, v] U_B[s_B, v + 3].  The joint spin state is chi = sum Psi(s_A, s_B) |phi_{s_A}> |phi_{s_B}>, normalised.
  D4.3  Winding: a_v = e^{i w phi_v} / sqrt 6.  A classical origin is a probability vector p_v: the state is the mixture sum p_v |phi_v><phi_v| x |phi_v+pi><phi_v+pi|.

Exact claims (each a test; ``python3 t3_qubit.py --tests-only``)
  E4.1  Nothing is discarded, the outcome probabilities sum to 1 for every pair of settings, neither marginal depends on the far setting, and CHSH <= 2 sqrt 2 for every
        state (500 random two-qubit states, twelve settings): Tsirelson's bound holds by construction (Tsirelson's theorem), and Clauser-Horne is CHSH here.
  E4.2  With U = 1 the four coefficients of chi are Fourier components of the origin amplitude: chi = (1/2) (A_0 |00> - A_1 |01> + A_1 |10> - A_2 |11>),
        A_m = sum_v a_v e^{i m phi_v}.  So a winding w gives: w = 0 -> |00> (a product state), w = -1 -> the singlet (fidelity 1), w = -2 -> |11>, and w = 1, 2, 3
        -> chi = 0 (the history cancels).  A uniform coherent origin, the shared axis of the paper, gives a product state, not an entangled one.
  E4.3  For w = -1, E(a, b) = -cos(a - b) exactly: the singlet's correlation.  Best CHSH over the six hexagonal settings 5/2, over the twelve settings 1 + sqrt 3 =
        2.732, and over 24 settings (15 degrees) 2 sqrt 2; never more.
  E4.4  The symmetry behind it: if the origin amplitude is odd under inversion, a_{v+3} = -a_v, then chi is antisymmetric under exchange of the two spins for ANY
        common wing U_A = U_B, so it is the singlet whenever it is not zero (200 random odd origins with random common unitaries: fidelity 1 to 1e-12); an even origin
        (a_{v+3} = a_v) gives a state with no singlet component at all.  Different wings break it (mean fidelity 0.26 over 200 random pairs).  The quantum walk
        exp(-i t H) keeps the w = -1 singlet at every t.  A classical origin (any probability vector) gives a separable mixture, CHSH <= 2.

What it says.  The quantum range is reached without post-selection, and the singlet is reached exactly, by one condition: the shared origin must be a coherent
superposition that is ODD under v -> -v.  That is the paper's anti-correlated fusion S_B(0) = -S_A(0), made coherent with a relative sign -1 between an origin
and its opposite: an antisymmetry of the kind fermions have.  With it, any wing the two parties share keeps the singlet; without it (an even, e.g. uniform, origin)
there is no singlet component.  With no wing, among the windings only w = -1 survives the read-out (w = +1 cancels), so the sense of the winding has to be opposite
to that of the read-out map: a relative handedness.  So the entanglement here is not made by the two-ended history; it is put into the
origin as a phase vortex.  **Hypothesis**: that such a vortex is what a magnetic-flux term (the area term of t3_area.py) or a holonomy vortex of the Spectre would
supply at the fusion point; nothing here derives it.

    python3 t3_qubit.py --tests-only
    python3 t3_qubit.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
for _p in (_HERE, _V2):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import tm2_rule as R  # noqa: E402

PHI = np.pi * np.arange(6) / 3.0
KET = np.array([[1.0, np.exp(1j * f)] for f in PHI]) / math.sqrt(2.0)
SX = np.array([[0, 1], [1, 0]], dtype=complex)
SY = np.array([[0, -1j], [1j, 0]], dtype=complex)
SINGLET = np.array([0, 1, -1, 0], dtype=complex) / math.sqrt(2.0)
TWELVE = tuple(range(12))


def sigma(a, step_deg=30.0):
    t = math.radians(step_deg * a)
    return math.cos(t) * SX + math.sin(t) * SY


def winding(w):
    return np.exp(1j * w * PHI) / math.sqrt(6.0)


def spin_state(a_v, U=None, UB=None):
    """chi of D4.2 (not normalised); UB defaults to U (a common wing)."""
    U = np.eye(6, dtype=complex) if U is None else U
    UB = U if UB is None else UB
    Psi = sum(a_v[v] * np.outer(U[:, v], UB[:, (v + 3) % 6]) for v in range(6))
    return np.einsum("ab,ai,bj->ij", Psi, KET, KET).reshape(4)


def normalised(chi):
    n = np.linalg.norm(chi)
    return None if n < 1e-12 else chi / n


def correlation(rho_or_chi, a, b, step_deg=30.0):
    O = np.kron(sigma(a, step_deg), sigma(b, step_deg))
    if rho_or_chi.ndim == 1:
        return float(np.vdot(rho_or_chi, O @ rho_or_chi).real)
    return float(np.trace(rho_or_chi @ O).real)


def chsh_of(state, settings, step_deg=30.0):
    E = np.array([[correlation(state, a, b, step_deg) for b in settings] for a in settings])
    return R.chsh(E)


def fidelity(chi):
    c = normalised(chi)
    return 0.0 if c is None else float(abs(np.vdot(SINGLET, c)) ** 2)


def classical_mixture(p):
    rho = np.zeros((4, 4), dtype=complex)
    for v in range(6):
        k = np.kron(KET[v], KET[(v + 3) % 6])
        rho += p[v] * np.outer(k, k.conj())
    return rho


def random_unitary(rng):
    q, r = np.linalg.qr(rng.normal(size=(6, 6)) + 1j * rng.normal(size=(6, 6)))
    return q * (np.diag(r) / np.abs(np.diag(r)))


def random_circulant(rng):
    F = np.exp(2j * np.pi * np.outer(np.arange(6), np.arange(6)) / 6.0) / math.sqrt(6.0)
    return (F * np.exp(1j * rng.uniform(0, 2 * np.pi, 6))) @ F.conj().T


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e41_nothing_is_discarded_and_tsirelson_holds():
    rng = np.random.default_rng(3)
    for _ in range(500):
        chi = rng.normal(size=4) + 1j * rng.normal(size=4)
        chi /= np.linalg.norm(chi)
        assert chsh_of(chi, TWELVE[:6]) <= 2 * math.sqrt(2) + 1e-9
    chi = normalised(spin_state(winding(-1)))
    for a in TWELVE:
        for b in TWELVE:
            P = [0.25 * (1 + sa * 0 + sb * 0 + sa * sb * correlation(chi, a, b)) for sa in (1, -1) for sb in (1, -1)]   # marginals are 0 (below)
            assert math.isclose(sum(P), 1.0) and min(P) >= -1e-12
            # marginal of Alice: <sigma(a) x 1> independent of b
            m = float(np.vdot(chi, np.kron(sigma(a), np.eye(2)) @ chi).real)
            assert abs(m) < 1e-12
    assert chsh_of(chi, TWELVE[:6]) <= 2 * math.sqrt(2) + 1e-12


def test_e42_the_coefficients_are_fourier_components_of_the_origin():
    rng = np.random.default_rng(5)
    for _ in range(20):
        a = rng.normal(size=6) + 1j * rng.normal(size=6)
        A = [np.sum(a * np.exp(1j * m * PHI)) for m in range(3)]
        want = 0.5 * np.array([A[0], -A[1], A[1], -A[2]])
        assert np.allclose(spin_state(a), want, atol=1e-12)
    assert fidelity(spin_state(winding(-1))) > 1 - 1e-12
    assert np.allclose(np.abs(normalised(spin_state(winding(0)))), [1, 0, 0, 0], atol=1e-12)
    assert np.allclose(np.abs(normalised(spin_state(winding(-2)))), [0, 0, 0, 1], atol=1e-12)
    for w in (1, 2, 3):
        assert np.linalg.norm(spin_state(winding(w))) < 1e-12, w


def test_e43_the_singlet_correlation_and_its_chsh():
    chi = normalised(spin_state(winding(-1)))
    for a in TWELVE:
        for b in TWELVE:
            assert math.isclose(correlation(chi, a, b), -math.cos(math.radians(30 * (a - b))), abs_tol=1e-12)
    assert math.isclose(chsh_of(chi, TWELVE[::2]), 2.5, abs_tol=1e-12)
    assert math.isclose(chsh_of(chi, TWELVE), 1 + math.sqrt(3), abs_tol=1e-12)
    assert math.isclose(chsh_of(chi, tuple(range(24)), 15.0), 2 * math.sqrt(2), abs_tol=1e-12)


def test_e44_an_odd_origin_gives_the_singlet_for_any_common_wing():
    a = winding(-1)
    for t in (0.1, 0.7, 1.3, 2.9):
        assert fidelity(spin_state(a, R.walk_unitary(t))) > 1 - 1e-12
    rng = np.random.default_rng(7)
    for _ in range(50):
        assert fidelity(spin_state(a, random_circulant(rng))) > 1 - 1e-12
    rng = np.random.default_rng(11)
    fo, fe, fd = [], [], []
    for _ in range(200):
        x = rng.normal(size=3) + 1j * rng.normal(size=3)
        U = random_unitary(rng)
        fo.append(fidelity(spin_state(np.r_[x, -x], U)))
        fe.append(fidelity(spin_state(np.r_[x, x], U)))
        fd.append(fidelity(spin_state(a, U, random_unitary(rng))))
    assert min(fo) > 1 - 1e-12 and max(fe) < 1e-20, "odd origin -> singlet, even origin -> none, for any common wing"
    assert math.isclose(np.mean(fd), 0.26, abs_tol=0.03) and max(fd) < 0.95, np.mean(fd)
    for _ in range(100):
        p = rng.dirichlet(np.ones(6))
        assert chsh_of(classical_mixture(p), TWELVE) <= 2.0 + 1e-12


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
            print("  %-80s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    for w in (0, -1, -2, 1):
        chi = spin_state(winding(w))
        c = normalised(chi)
        if c is None:
            print("winding %+d: the history cancels (chi = 0)" % w)
            continue
        print("winding %+d: |chi| = %.4f, singlet fidelity %.3f, CHSH over 6 / 12 settings %.3f / %.3f" % (w, np.linalg.norm(chi), fidelity(chi), chsh_of(c, TWELVE[::2]), chsh_of(c, TWELVE)))
    rng = np.random.default_rng(7)
    print("w = -1 with a common random wing: fidelity %.3f; with two different random wings, mean of 200: %.3f" % (
        fidelity(spin_state(winding(-1), random_unitary(rng))), np.mean([fidelity(spin_state(winding(-1), random_unitary(rng), random_unitary(rng))) for _ in range(200)])))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
