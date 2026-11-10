"""
spectre_hat_quantum_bit_model.py - a quantum bit at the 14th vertex of the Spectre-Hat flow.

spectre_hat_time_model.py stores a classical bit at P_8 (s = +1 pushes P_8 out of the chord
P_7 -> P_9 by h(tau), s = -1 pushes it in) and the two tiles differ by the jump d(tau) = 2 h(tau).
Here the bit has two *amplitudes* and the overlap of its two states is set by that jump.

Status labels (as in QUANTUM_GRAVITY.md): **Exact** = checked by the self tests of this file
(python3 spectre_hat_quantum_bit_model.py --tests-only); **Definition** = a choice made here;
**Hypothesis** = a physical reading nothing here tests.  Nothing here derives physics.

    from spectre_hat_quantum_bit_model import SpectreHatQubit

    q = SpectreHatQubit(ticks=20, corner_deg=30, snap_scale=0.1)   # starts in |+>
    q.overlap(0.5)                      # c(tau) = <+|-> at tau = 0.5
    q.helstrom(0.5)                     # best possible reading of the bit there, sqrt(1 - c^2)
    q.step(20)                          # smooth flow plus tunnelling, tick by tick
    q.readout_probs(q.tau)              # (1/2, 1/2): the bit is unreadable at the Hat
    q.measure()                         # reads the bit; the register collapses to that tile

Definitions
-----------
Q1  The quantum degree of freedom.  x is the position of P_8 along the normal of the chord
    P_7 -> P_9, measured from the chord (outward positive); the rest of the tile is classical.
    The two bit states are Gaussian wave packets of standard deviation sigma centred at
    x = +h(tau) and x = -h(tau):  |+> = G(x - h),  |-> = G(x + h),  G(x) = (2 pi sigma^2)^(-1/4)
    exp(-x^2 / 4 sigma^2).  Their centres are d(tau) = 2h(tau) apart.  As tau runs, the double
    well with minima +-h becomes a single well (h -> 0): the 14th vertex collapse.
Q2  The register.  psi = alpha |+> + beta |->.  The *coefficients* (alpha, beta) are the discrete
    information stored at the vertex; the smooth flow carries them along unchanged (they ride
    on the surface) while the two states they multiply move together.
Q3  Tunnelling (snaps).  A partial flip U(theta) = cos(theta/2) 1 + i sin(theta/2) X, X swapping
    alpha and beta, acts once per tick with theta_j = Omega c(tau_j) / N (Omega = ``tunnel_rate``).
    It is switched off at ticks where either tile self-intersects (the validity rule R3 of the
    classical model).  This replaces the classical cost rule R2 (jump <= snap_scale) by a smooth
    suppression: a snap is exponentially rare while the tiles are far apart and order one when
    they merge.  ``sigma`` defaults to snap_scale / 2, so c = exp(-1/2) where the classical
    window opens (d = snap_scale).
Q4  Reading.  The bit is read by the best projective measurement for telling |+> from |->
    (Helstrom).  Reading writes a classical bit: after the outcome s the register is |s>.

What is exact (tests)
---------------------
E1  Overlap: <+|-> = c(tau) = exp(-d(tau)^2 / 8 sigma^2), by numerical integration of the packets,
    and the norm of alpha|+> + beta|-> is N = |alpha|^2 + |beta|^2 + 2 c Re(conj(alpha) beta).
E2  Gram matrix [[1, c], [c, 1]]: eigenvalues 1 +- c; the register space has dimension 2 for
    tau < 1 and 1 at tau = 1 (the Hat), where |+> = |->.
E3  Even and odd sectors a_e = (alpha + beta)/sqrt2, a_o = (alpha - beta)/sqrt2 (parity
    x -> -x swaps the two tiles): N = (1 + c)|a_e|^2 + (1 - c)|a_o|^2.  The odd state
    (|+> - |->)/sqrt2 has norm^2 1 - c, which goes to 0 at the Hat: it is annihilated there,
    like the sink/source pair of arrows in the notes.
E4  Partial flips U(theta) preserve N exactly for every c, add their angles, and put the phases
    e^(+-i theta/2) on the even and odd sectors.  The smooth flow with fixed coefficients does
    not preserve N: it is the non-unitary step that erases the bit.
E5  Reading: the best success probability for |+> versus |-> is (1 + sqrt(1 - c^2))/2 (checked by
    brute force over all measurement bases); for |+> the probability bias P(+) - P(-) equals
    sqrt(1 - c^2) and decreases strictly with tau to 0; at the Hat every state reads (1/2, 1/2).
E6  Classical limit: as sigma -> 0 the overlap is 0 before the Hat, reading is perfect,
    P(+) = |alpha|^2, and tunnelling is frozen.
E7  The total angle of the run is the integral of Omega c(tau) dtau, and the final state is
    cos(Theta/2)|+> + i sin(Theta/2)|-> when starting in |+>.

Physics reading (all **Hypothesis**; none of it is tested here)
---------------------------------------------------------------
* P_8 as one quantum coordinate in a double well that merges into a single well: a toy for a
  discrete degree of freedom that is only distinguishable while its two states are separated
  by the smooth geometry.  At the Hat the discrete information is not stored anywhere any more.
* The overlap c(tau) as a measure of how distinguishable two classical geometries are, in the
  spirit of the overlap of two coherent states.  Here it is put in by Q1, not derived.
* Odd-sector annihilation at the Hat <-> the notes' picture of two arrows annihilating at the
  14th vertex.  The classical model measured the arrows on the outline; the quantum model does
  not use them.
* The flow as the irreversible step: with the coefficients carried along, the bit is erased in
  the direction of increasing tau.  The model does not derive why tau increases.
What does not carry over: no unitary evolution of the whole tile, no Hamiltonian is given for
tau-evolution (the tunnelling angle is a rule), one coordinate and one vertex only, the smooth
surface stays classical, there is no gravity and no field.

Only NumPy and the two modules spectre_hat_time_model.py, spectre_hat_collinearity_flow_model.py
(and spectremodel.py) are needed; matplotlib is imported lazily by the command line.
"""

import argparse
import math
import sys

import numpy as np

from spectre_hat_time_model import ArrowError, SpectreHatTime, is_simple_tile

__all__ = ["Annihilated", "SpectreHatQubit", "gaussian", "run_tests"]


class Annihilated(RuntimeError):
    """The register has norm 0 (it is the odd state at the Hat): there is nothing to read."""


def gaussian(x, centre, sigma):
    """The packet G(x - centre) of Q1, normalised to integral 1 of its square."""
    return (2.0 * math.pi * sigma**2) ** -0.25 * np.exp(-((np.asarray(x, dtype=float) - centre) ** 2) / (4.0 * sigma**2))


def _state(alpha, beta):
    psi = np.array([alpha, beta], dtype=complex)
    if not np.all(np.isfinite(psi)) or not np.any(psi):
        raise ValueError("alpha and beta must be finite and not both 0, got %r" % ((alpha, beta),))
    return psi


class SpectreHatQubit:
    """Two amplitudes at P_8 whose overlap is set by the jump d(tau) of the time model.

    Parameters
    ----------
    ticks, k0, a, corner_deg, snap_scale : as in SpectreHatTime (the clock and the geometry).
    sigma       : width of the packets (Q1); default snap_scale / 2.
    tunnel_rate : Omega of Q3, radians of flip per unit tau at overlap 1 (default pi).
    alpha, beta : the register at tick 0 (not normalised; default |+>).
    seed        : seed of the random generator used by ``measure`` and ``sample_readout``.
    """

    def __init__(self, ticks=20, k0=0.5, a=1.0, corner_deg=30.0, snap_scale=0.1, sigma=None,
                 tunnel_rate=math.pi, alpha=1.0, beta=0.0, seed=None):
        self.time = SpectreHatTime(ticks=ticks, k0=k0, a=a, corner_deg=corner_deg, snap_scale=snap_scale)
        self.sigma = float(snap_scale) / 2.0 if sigma is None else float(sigma)
        if not (math.isfinite(self.sigma) and self.sigma > 0.0):
            raise ValueError("sigma must be positive and finite (give sigma or a positive snap_scale), got %r" % (sigma,))
        self.tunnel_rate = float(tunnel_rate)
        if not (math.isfinite(self.tunnel_rate) and self.tunnel_rate >= 0.0):
            raise ValueError("tunnel_rate must be finite and >= 0, got %r" % (tunnel_rate,))
        self.psi = _state(alpha, beta)
        self.rng = np.random.default_rng(seed)
        self.angle_total = 0.0
        self.outcomes = []
        self.rows = []
        self.data = {}
        self._record_row(0.0)

    # -- clock -------------------------------------------------------------------
    @property
    def N(self):
        return self.time.N

    @property
    def now(self):
        return self.time.now

    @property
    def tau(self):
        return self.time.tau

    @property
    def alpha(self):
        return complex(self.psi[0])

    @property
    def beta(self):
        return complex(self.psi[1])

    # -- the overlap (Q1) and what follows from it ------------------------------------------
    def jump(self, tau):
        return self.time.jump(tau)

    def overlap(self, tau):
        """c(tau) = <+|-> = exp(-d^2 / 8 sigma^2) (E1)."""
        d = self.jump(tau)
        return math.exp(-d * d / (8.0 * self.sigma**2))

    def one_minus_overlap(self, tau):
        """1 - c(tau), accurate for small d (exp(x) - 1 done with expm1)."""
        d = self.jump(tau)
        return -math.expm1(-d * d / (8.0 * self.sigma**2))

    def gram(self, tau):
        c = self.overlap(tau)
        return np.array([[1.0, c], [c, 1.0]])

    def rank(self, tau):
        """Dimension of the register space: 2 while the tiles differ, 1 at the Hat (E2)."""
        return 2 if self.jump(tau) > 0.0 else 1

    def helstrom(self, tau):
        """sqrt(1 - c^2): the trace distance of |+> and |->, the bias of the best reading (E5)."""
        return math.sqrt(self.one_minus_overlap(tau) * (1.0 + self.overlap(tau)))

    def helstrom_success(self, tau):
        """(1 + sqrt(1 - c^2)) / 2: best probability of telling |+> from |-> (E5)."""
        return 0.5 * (1.0 + self.helstrom(tau))

    # -- the register ----------------------------------------------------------------------
    def _psi(self, psi):
        return self.psi if psi is None else _state(*psi)

    def norm2(self, tau, psi=None):
        """N = |alpha|^2 + |beta|^2 + 2 c Re(conj(alpha) beta)  (E1)."""
        al, be = self._psi(psi)
        return float(abs(al) ** 2 + abs(be) ** 2 + 2.0 * self.overlap(tau) * (np.conj(al) * be).real)

    def sectors(self, tau, psi=None):
        """Even and odd coefficients and their weights (E3)."""
        al, be = self._psi(psi)
        ae, ao = (al + be) / math.sqrt(2.0), (al - be) / math.sqrt(2.0)
        c = self.overlap(tau)
        return {"a_even": ae, "a_odd": ao, "w_even": (1.0 + c) * abs(ae) ** 2,
                "w_odd": self.one_minus_overlap(tau) * abs(ao) ** 2}

    def ortho(self, tau, psi=None):
        """Amplitudes (psi_s, psi_a) in the orthonormal basis of the symmetric and antisymmetric
        combinations; |psi_s|^2 + |psi_a|^2 = N."""
        s = self.sectors(tau, psi)
        return math.sqrt(1.0 + self.overlap(tau)) * s["a_even"], math.sqrt(self.one_minus_overlap(tau)) * s["a_odd"]

    def readout_probs(self, tau, psi=None, tol=1e-14):
        """(P(+), P(-)) of the Helstrom reading of the normalised register at tau: the
        projectors onto (e_s +- e_a)/sqrt2.  None if the register is annihilated (N <= tol)."""
        ps, pa = self.ortho(tau, psi)
        n = abs(ps) ** 2 + abs(pa) ** 2
        if n <= tol:
            return None
        return float(0.5 * abs(ps + pa) ** 2 / n), float(0.5 * abs(ps - pa) ** 2 / n)

    def readout_bias(self, tau, psi=None):
        """P(+) - P(-), or None when annihilated."""
        p = self.readout_probs(tau, psi)
        return None if p is None else p[0] - p[1]

    def wavefunction(self, x, tau, psi=None):
        """alpha G(x - h) + beta G(x + h) on the points x (Q1); its squared integral is N."""
        al, be = self._psi(psi)
        h = self.jump(tau) / 2.0
        return al * gaussian(x, h, self.sigma) + be * gaussian(x, -h, self.sigma)

    # -- tunnelling (Q3) --------------------------------------------------------------------
    def tunnelling_allowed(self, tau):
        """False where either tile self-intersects (rule R3 of the classical model)."""
        return all(is_simple_tile(self.time.tile(tau, s)) for s in (1, -1))

    def snap_angle(self, j):
        """theta_j = Omega c(tau_j) / N, zero where tunnelling is not allowed."""
        tj = self.time.tau_of(j)
        if not self.tunnelling_allowed(tj):
            return 0.0
        return self.tunnel_rate * self.overlap(tj) / self.N

    def theta_density(self, tau):
        """Omega c(tau): the angle per unit tau (before the validity switch)."""
        return self.tunnel_rate * self.overlap(tau)

    @staticmethod
    def flip(psi, theta):
        """U(theta) psi = cos(theta/2) psi + i sin(theta/2) X psi (E4)."""
        return math.cos(theta / 2.0) * psi + 1j * math.sin(theta / 2.0) * psi[::-1]

    def step(self, n=1):
        """Run n ticks: at each, tunnel by theta_j, then the smooth flow moves to the next tick
        (the coefficients stay as they are, Q2).  Negative n raises ArrowError."""
        if int(n) != n:
            raise ValueError("n must be an integer, got %r" % (n,))
        if n < 0:
            raise ArrowError("the smooth flow only runs forward: step(%d)" % n)
        if self.now + n > self.N:
            raise ValueError("cannot step past tick %d (tau = 1): now=%d, n=%d" % (self.N, self.now, n))
        for _ in range(int(n)):
            theta = self.snap_angle(self.now)
            self.psi = self.flip(self.psi, theta)
            self.angle_total += theta
            self.time.step(1)
            self._record_row(theta)

    # -- reading (Q4) ---------------------------------------------------------------------------
    def sample_readout(self, n, rng=None):
        """n independent readings of the present register (it is not changed); array of +-1."""
        p = self.readout_probs(self.tau)
        if p is None:
            raise Annihilated("the register has norm 0 at tau = %g" % self.tau)
        rng = self.rng if rng is None else rng
        return np.where(rng.random(int(n)) < p[0], 1, -1)

    def measure(self):
        """Read the bit.  The register becomes the basis state of the outcome (Q4)."""
        s = int(self.sample_readout(1)[0])
        self.psi = _state(1.0, 0.0) if s > 0 else _state(0.0, 1.0)
        self.outcomes.append({"tick": self.now, "tau": self.tau, "bit": s})
        self._record_row(0.0)
        return s

    # -- bookkeeping ----------------------------------------------------------------------------
    def _record_row(self, theta):
        tau = self.tau
        p = self.readout_probs(tau)
        self.rows.append({"tick": self.now, "tau": tau, "overlap": self.overlap(tau), "theta": theta,
                          "norm2": self.norm2(tau), "p_plus": None if p is None else p[0],
                          "bias": None if p is None else p[0] - p[1], "rank": self.rank(tau),
                          "helstrom": self.helstrom(tau)})

    def __repr__(self):
        return "<SpectreHatQubit tick %d/%d tau=%.4g alpha=%.4g%+.4gj beta=%.4g%+.4gj sigma=%g Omega=%g>" % (
            self.now, self.N, self.tau, self.psi[0].real, self.psi[0].imag, self.psi[1].real, self.psi[1].imag,
            self.sigma, self.tunnel_rate)


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectre_hat_quantum_bit_model.py --tests-only)
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def _random_states(n, seed=0):
    rng = np.random.default_rng(seed)
    return [tuple(rng.normal(size=2) + 1j * rng.normal(size=2)) for _ in range(n)]


def test_e1_overlap_is_the_gaussian_overlap_and_norm_is_the_gram_form():
    q = SpectreHatQubit(corner_deg=30.0, sigma=0.2)
    x = np.linspace(-6.0, 6.0, 200001)
    for tau in (0.0, 0.3, 0.8, 0.95):
        h = q.jump(tau) / 2
        num = np.trapezoid(gaussian(x, h, q.sigma) * gaussian(x, -h, q.sigma), x)
        assert math.isclose(num, q.overlap(tau), rel_tol=1e-9), tau
        assert math.isclose(np.trapezoid(gaussian(x, h, q.sigma) ** 2, x), 1.0, rel_tol=1e-9)
        for psi in _random_states(3, seed=int(tau * 100)):
            n_wave = np.trapezoid(np.abs(q.wavefunction(x, tau, psi)) ** 2, x)
            assert math.isclose(n_wave, q.norm2(tau, psi), rel_tol=1e-8), (tau, psi)
    assert math.isclose(q.overlap(1.0), 1.0, abs_tol=0.0) and q.overlap(0.0) < q.overlap(0.5) < q.overlap(1.0)
    q2 = SpectreHatQubit(snap_scale=0.3)
    assert math.isclose(q2.sigma, 0.15) and math.isclose(math.exp(-0.3**2 / (8 * q2.sigma**2)), math.exp(-0.5))


def test_e2_gram_matrix_and_the_rank_drop_at_the_hat():
    q = SpectreHatQubit(corner_deg=60.0, sigma=0.3)
    for tau in (0.0, 0.4, 0.9, 0.999):
        c = q.overlap(tau)
        assert np.allclose(sorted(np.linalg.eigvalsh(q.gram(tau))), sorted([1 - c, 1 + c]), atol=1e-14)
        assert q.rank(tau) == 2 and q.one_minus_overlap(tau) > 0.0
    assert q.rank(1.0) == 1 and q.one_minus_overlap(1.0) == 0.0 and abs(np.linalg.det(q.gram(1.0))) == 0.0
    assert np.allclose(q.gram(1.0), 1.0), "|+> = |-> at the Hat"
    tiny = SpectreHatQubit(corner_deg=30.0, sigma=0.2)  # expm1 keeps 1 - c accurate near the Hat
    d = tiny.jump(0.999999)
    assert math.isclose(tiny.one_minus_overlap(0.999999), d * d / (8 * 0.2**2), rel_tol=1e-6)


def test_e3_even_odd_sectors_and_annihilation_of_the_odd_state():
    q = SpectreHatQubit(corner_deg=30.0, sigma=0.2)
    for tau in (0.0, 0.5, 1.0):
        for psi in _random_states(4, seed=7):
            s = q.sectors(tau, psi)
            assert math.isclose(s["w_even"] + s["w_odd"], q.norm2(tau, psi), rel_tol=1e-12, abs_tol=1e-12)
            ps, pa = q.ortho(tau, psi)
            assert math.isclose(abs(ps) ** 2 + abs(pa) ** 2, q.norm2(tau, psi), rel_tol=1e-12, abs_tol=1e-12)
            assert math.isclose(q.norm2(tau, psi), q.norm2(tau, psi[::-1]), rel_tol=1e-12, abs_tol=1e-12), "parity"
    r = 1 / math.sqrt(2)
    odd, even = (r, -r), (r, r)
    norms = [q.norm2(t, odd) for t in np.linspace(0, 1, 11)]
    assert np.all(np.diff(norms) < 0) and norms[-1] == 0.0
    assert math.isclose(q.norm2(0.3, odd), 1 - q.overlap(0.3)) and math.isclose(q.norm2(0.3, even), 1 + q.overlap(0.3))
    assert q.readout_probs(1.0, odd) is None and q.readout_probs(0.99, odd) is not None
    assert _raises(Annihilated, SpectreHatQubit(ticks=2, alpha=1.0, beta=-1.0, tunnel_rate=0.0).__class__.sample_readout,
                   _at_hat(SpectreHatQubit(ticks=2, alpha=1.0, beta=-1.0, tunnel_rate=0.0)), 1)


def _at_hat(q):
    q.step(q.N)
    return q


def test_e4_partial_flips_are_gram_unitary_and_the_flow_is_not():
    q = SpectreHatQubit(corner_deg=30.0, sigma=0.2)
    for tau in (0.0, 0.6, 0.99):
        for psi in _random_states(4, seed=3):
            for theta in (0.3, 1.7, math.pi):
                new = q.flip(np.array(psi), theta)
                assert math.isclose(q.norm2(tau, new), q.norm2(tau, psi), rel_tol=1e-12), (tau, theta)
                s0, s1 = q.sectors(tau, psi), q.sectors(tau, new)
                assert np.isclose(s1["a_even"], np.exp(1j * theta / 2) * s0["a_even"])
                assert np.isclose(s1["a_odd"], np.exp(-1j * theta / 2) * s0["a_odd"])
            two = q.flip(q.flip(np.array(psi), 0.4), 0.9)
            assert np.allclose(two, q.flip(np.array(psi), 1.3)), "angles add"
    # smooth flow with fixed coefficients: |+> keeps norm 1, the even state grows, the odd one dies
    assert math.isclose(q.norm2(0.0, (1, 0)), 1.0) and math.isclose(q.norm2(1.0, (1, 0)), 1.0)
    assert q.norm2(1.0, (1, 1)) == 4.0 and q.norm2(0.0, (1, 1)) < 4.0 and q.norm2(1.0, (1, -1)) == 0.0


def test_e5_helstrom_reading():
    q = SpectreHatQubit(corner_deg=30.0, sigma=0.2)
    thetas = np.linspace(0.0, math.pi, 20001)
    for tau in (0.0, 0.5, 0.9, 1.0):
        c = q.overlap(tau)
        plus, minus = q.ortho(tau, (1, 0)), q.ortho(tau, (0, 1))
        best = 0.0
        for th in thetas[::4]:
            mp, mm = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
            best = max(best, 0.5 * (abs(mp @ np.array(plus)) ** 2 + abs(mm @ np.array(minus)) ** 2))
        assert math.isclose(best, 0.5 * (1 + math.sqrt(1 - c * c)), abs_tol=1e-7), tau
        assert math.isclose(q.helstrom_success(tau), 0.5 * (1 + math.sqrt(1 - c * c)), abs_tol=1e-12)
        pp, pm = q.readout_probs(tau, (1, 0))
        assert math.isclose(pp - pm, q.helstrom(tau), abs_tol=1e-12) and math.isclose(pp + pm, 1.0)
        assert math.isclose(q.helstrom_success(tau), pp, abs_tol=1e-12)
    taus = np.linspace(0, 1, 101)
    bias = np.array([q.helstrom(t) for t in taus])
    assert np.all(np.diff(bias) < 0) and bias[-1] == 0.0
    for psi in _random_states(5, seed=11):
        p = q.readout_probs(1.0, psi)
        assert math.isclose(p[0], 0.5, abs_tol=1e-12) and math.isclose(p[1], 0.5, abs_tol=1e-12), "unreadable at the Hat"


def test_e6_classical_limit_sigma_to_zero():
    q = SpectreHatQubit(corner_deg=30.0, sigma=1e-3, alpha=0.6, beta=0.8)
    for tau in (0.0, 0.5, 0.9):  # d(tau) >> sigma here; very close to tau = 1 even this sigma is not small
        assert q.overlap(tau) < 1e-100 and math.isclose(q.helstrom(tau), 1.0, abs_tol=1e-100)
        p = q.readout_probs(tau)
        assert math.isclose(p[0], 0.36, abs_tol=1e-12) and math.isclose(p[1], 0.64, abs_tol=1e-12)
    q.step(q.N - 1)
    assert q.angle_total < 1e-100 and np.allclose(q.psi, [0.6, 0.8], atol=1e-100), "tunnelling frozen"
    big = SpectreHatQubit(corner_deg=30.0, sigma=50.0)  # far quantum limit: tiles overlap almost fully
    assert big.overlap(0.0) > 0.9999 and big.helstrom(0.0) < 0.01


def test_e7_total_angle_and_the_final_state():
    q = SpectreHatQubit(ticks=200, corner_deg=30.0, snap_scale=0.1)
    q.step(q.N)
    theta = sum(r["theta"] for r in q.rows)
    assert math.isclose(theta, q.angle_total, rel_tol=1e-12)
    taus = np.linspace(0, 1, 20001)
    integral = np.trapezoid([q.theta_density(t) for t in taus], taus)
    assert abs(q.angle_total - integral) / integral < 0.03
    assert np.allclose(q.psi, [math.cos(q.angle_total / 2), 1j * math.sin(q.angle_total / 2)], atol=1e-12)
    # tunnelling angles grow along the flow: frozen early, active late
    ang = [r["theta"] for r in q.rows[1:]]
    assert np.all(np.diff(ang) > 0) and ang[0] < 1e-3 * ang[-1] + 1e-12
    # the flip probability of a measured outcome after the run
    p = q.readout_probs(1.0)
    assert math.isclose(p[0], 0.5, abs_tol=1e-12)


def test_validity_switches_tunnelling_off_where_a_tile_self_intersects():
    q = SpectreHatQubit(ticks=20, corner_deg=150.0, k0=0.5, sigma=0.5)
    assert q.snap_angle(0) == 0.0 and not q.tunnelling_allowed(0.0)
    assert q.snap_angle(19) > 0.0 and q.tunnelling_allowed(0.95)
    ok = SpectreHatQubit(corner_deg=30.0)
    assert all(ok.snap_angle(j) > 0.0 for j in range(ok.N))


def test_measurement_statistics_and_collapse():
    q = SpectreHatQubit(ticks=20, corner_deg=30.0, sigma=0.2, alpha=1.0, beta=0.0, seed=5)
    q.step(15)
    p = q.readout_probs(q.tau)
    s = q.sample_readout(40000, rng=np.random.default_rng(1))
    assert abs(np.mean(s == 1) - p[0]) < 0.01
    before = q.psi.copy()
    q.sample_readout(10)
    assert np.array_equal(before, q.psi), "sampling does not collapse"
    out = q.measure()
    assert out in (1, -1) and np.allclose(q.psi, [1, 0] if out > 0 else [0, 1])
    assert q.outcomes == [{"tick": 15, "tau": 0.75, "bit": out}]
    assert math.isclose(q.norm2(q.tau), 1.0)
    a = SpectreHatQubit(seed=9, alpha=1, beta=1)
    b = SpectreHatQubit(seed=9, alpha=1, beta=1)
    assert a.measure() == b.measure(), "seeded runs repeat"


def test_arrow_and_validation():
    q = SpectreHatQubit()
    assert _raises(ArrowError, q.step, -1) and _raises(ValueError, q.step, q.N + 1) and _raises(ValueError, q.step, 0.5)
    for kw in ({"sigma": 0.0}, {"sigma": -1.0}, {"sigma": float("nan")}, {"snap_scale": 0.0}, {"tunnel_rate": -1.0},
               {"tunnel_rate": float("inf")}, {"alpha": 0.0, "beta": 0.0}, {"alpha": float("nan")}):
        assert _raises(ValueError, SpectreHatQubit, **kw), kw
    assert q.now == 0 and np.allclose(q.psi, [1, 0]), "failed steps change nothing"


def run_tests(verbose=True):
    """Run every ``test_*`` function in this module; return True if none failed."""
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001 - report and keep going
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        if verbose:
            print("  %-78s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Command line: a story and a figure
# --------------------------------------------------------------------------
def story(q, out=print):
    """Prepare |+>, run the whole flow with tunnelling, print the table, then read the bit."""
    out(repr(q))
    out("%4s %6s %9s %9s %9s %9s %9s %5s" % ("tick", "tau", "overlap", "theta", "norm2", "P(+)", "bias", "rank"))
    q.step(q.N)
    for r in q.rows:
        fmt = lambda v: "   -   " if v is None else "%9.5f" % v
        out("%4d %6.3f %9.5f %9.5f %9.5f %s %s %5d" % (r["tick"], r["tau"], r["overlap"], r["theta"], r["norm2"],
                                                    fmt(r["p_plus"]), fmt(r["bias"]), r["rank"]))
    out("total tunnelling angle %.5f rad; state %s" % (q.angle_total, np.round(q.psi, 5)))
    out("best reading of the bit at the end: %.4f (a coin: the Hat has erased it)" % q.helstrom_success(1.0))


def plot(q, axes_psi, ax_curve):
    x = np.linspace(-2.6 * q.jump(0.0) / 2 - 4 * q.sigma, 2.6 * q.jump(0.0) / 2 + 4 * q.sigma, 800)
    r = 1 / math.sqrt(2)
    states = [("|+>", (1, 0), "tab:blue"), ("even", (r, r), "tab:green"), ("odd", (r, -r), "tab:red")]
    taus = np.linspace(0.0, 1.0, len(axes_psi))
    ymax = 1.05 * max(np.max(np.abs(q.wavefunction(x, t, p)) ** 2) for t in taus for _, p, _ in states)
    for ax, tv in zip(axes_psi, taus):
        for name, psi, col in states:
            ax.plot(x, np.abs(q.wavefunction(x, tv, psi)) ** 2, color=col, lw=1.2, label=name if tv == 0.0 else None)
        ax.set_ylim(0, ymax)
        ax.set_yticks([])
        ax.set_title("tau %.2f  d %.2f  c %.2f" % (tv, q.jump(tv), q.overlap(tv)), fontsize=8)
        ax.set_xlabel("x (P_8 from the chord)", fontsize=7)
        if tv == 0.0:
            ax.legend(fontsize=7, frameon=False)
    t = np.linspace(0.0, 1.0, 400)
    ax_curve.plot(t, [q.overlap(v) for v in t], color="tab:orange", lw=2.0,
                  label="overlap c(tau) = tunnelling angle / Omega")
    ax_curve.plot(t, [q.helstrom(v) for v in t], color="tab:purple", label="readout bias sqrt(1-c^2)")
    ax_curve.plot(t, [q.time.readable(v) for v in t], color="0.5", ls="--", label="classical readable d/d(0)")
    ax_curve.axvspan(q.time.tau_star(), 1.0, color="tab:orange", alpha=0.12, label="classical snap window")
    ax_curve.set_xlim(0, 1)
    ax_curve.set_xlabel("tau")
    ax_curve.legend(fontsize=7, loc="center left")


def main(argv=None):
    p = argparse.ArgumentParser(description="Quantum bit at the 14th vertex of the Spectre-Hat flow (toy model).")
    p.add_argument("--ticks", type=int, default=20, help="number of ticks N (default 20)")
    p.add_argument("--corner", type=float, default=30.0, metavar="DEG", help="omega_0 in degrees (default 30)")
    p.add_argument("--k0", type=float, default=0.5, help="curve strength at tau = 0 (default 0.5)")
    p.add_argument("--snap-scale", type=float, default=0.1, help="classical snap scale; sigma defaults to half of it")
    p.add_argument("--sigma", type=float, default=None, help="width of the packets (default snap_scale / 2)")
    p.add_argument("--tunnel-rate", type=float, default=math.pi, help="Omega, radians of flip per unit tau (default pi)")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--story", action="store_true", help="print a run from |+> through the whole flow")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="save the figure to this file")
    args = p.parse_args(argv)

    if args.tests_only:
        print("spectre_hat_quantum_bit_model self tests:")
        return 0 if run_tests() else 1
    try:
        q = SpectreHatQubit(ticks=args.ticks, k0=args.k0, corner_deg=args.corner, snap_scale=args.snap_scale,
                            sigma=args.sigma, tunnel_rate=args.tunnel_rate)
    except ValueError as exc:
        p.error(str(exc))
    if not (args.story or args.save):
        print(q)
        print("use --story, --save PNG or --tests-only")
        return 0
    fig_q = SpectreHatQubit(ticks=args.ticks, k0=args.k0, corner_deg=args.corner, snap_scale=args.snap_scale,
                            sigma=args.sigma, tunnel_rate=args.tunnel_rate)
    if args.story:
        story(q)
    if args.save:
        import matplotlib

        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(13, 6.4))
        gs = fig.add_gridspec(2, 5, height_ratios=[1.0, 1.1])
        axes = [fig.add_subplot(gs[0, i]) for i in range(5)]
        ax = fig.add_subplot(gs[1, :])
        plot(fig_q, axes, ax)
        fig.tight_layout()
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
        if not args.no_show:
            plt.show()
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
