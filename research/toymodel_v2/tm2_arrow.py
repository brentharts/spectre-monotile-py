#!/usr/bin/env python3
"""tm2_arrow.py - Stage 3 of PLAN.md: the arrow audit.  Where does an orientation enter, which inputs are independent, and where can records sit.

Nothing here derives an arrow of time.  The audit lists every place where one direction of a timeline is treated differently from the other, switches each
off and reverses each, and records what follows it.  "Arrow" below always means: an observable that changes sign (or mirrors) when a named input is reversed.

Definitions
  D3.1  Reversal R of a timeline: s(t) -> s(n-1-t).  Backward traversal T: s(t) -> -s(n-1-t).  R is T followed by the half turn (rotation by pi).
  D3.2  The inputs audited (C5 of PLAN.md, with one added by this stage):
          (i)   a pin (boundary data of the history-sum);         (i') the base point of the prefix flux Phi (the action's flux term counts from t = 0);
          (ii)  the damping gamma of the wave equation;            (iii) the orientation of tau (the chain's amplitude schedule theta_j);
          (iv)  the handedness of the tile;                         (v)  the scale ordering of the ladder c_j (D2.5 of tm2_directed.py).
  D3.3  Observables.  pin: centroid of the influence profile minus the centre of the timeline.  base point: q(t) = P(s(t+1) = s(t)), compared with q(n-2-t).
        damping: energy at the end over energy at the start.  tau: mean of the flip-time distribution minus the centre.  handedness: signed turn at P_8.
        scale: c_(j+1) - c_j.
  D3.4  Census classes of a closed hexagonal timeline s.  'rotation-reversible': R(s) = g(s) for a rotation g of the alphabet;  'mirror-reversible': R(s) = g(s)
        for a reflection g;  'oriented': neither.  Strict = as sequences with their start;  cyclic = up to a shift of the start (the closed curve, forgetting where it starts).
  D3.5  Extension of the flow beyond the Hat (C9):  h(tau) = a tan(omega_0 (1 - tau) / 2) for tau in [0, 2] (odd about 1), c(tau) = exp(-h^2 / 2 sigma^2) (even about 1).
        This is the analytic continuation of the laws; whether the flow continues is not decided here.
  D3.6  Records.  W(j) = 1 - H2((1 + sqrt(1 - c_j^2)) / 2): bits one optimal reading at tick j gives about the bit at tick j.
        K(i, j) = 1 - H2(p), p = probability that an optimal reading at tick j gives the bit that held at tick i (tunnelling between the ticks, coherent).
        H2 = binary entropy in bits.  A 'record of tick i held at tick j' is K(i, j).  This is the register as its own memory; no external store is modelled.

  D3.7  Two radial readings of the overlap (both **Hypothesis**; only their algebra is tested).  With sigma_t^2 = r_s / r (the QGD clock is sqrt(1 - sigma_t^2)):
          reading A (C7 of PLAN.md)  c = sigma_t:            r / r_s = 1 / c^2;        the Hat (c = 1, fluid) is at the horizon, the frozen end far away.
          reading B (added here)     c = sqrt(1 - sigma_t^2): r / r_s = 1 / (1 - c^2);  the frozen end (c = 0) is at the horizon, the Hat (fluid) is at infinity,
                                     and the bit's jump obeys d^2 = -4 sigma^2 ln(1 - r_s / r), which is 4 sigma^2 r_s / r far away (it falls like the Newtonian potential).
        Reading B is the one in which 'the solid, determined end is nearest the particle and the fluid end far away' (ARROW_OF_TIME.md, section 7).

Exact claims (each a test; ``python3 tm2_arrow.py --tests-only``)
  E3.1  C5.  A(R s) = A(s) for every closed hexagonal timeline of 3 to 7 steps (12, 90, 360, 2040, 10080 of them) and any lam.  Proof: the first term is symmetric; the
        prefix sums of R s are F - Phi(n-2-t) with F = Phi(n-1) the total flux, which for F = 0 are minus the original ones in reverse order; N(-v) = N(v), and the two
        terms that do not pair, N(Phi(0)) and N(Phi(n-2)), are both 1 (single letters).  For open timelines the same identity gives
        A(R s) - A(s) = lam * [N(F) + sum_{t=0}^{n-3} N(F - Phi(t)) - sum_{t=1}^{n-1} N(Phi(t))], non-zero for 6624 of the 7776
        timelines of 5 steps.  So in the open case the action itself has an orientation: the base point of Phi (input i').
  E3.2  Measure level.  q(t) is mirror symmetric (to 1e-12) for closed timelines with any lam, and for open ones with lam = 0; for open ones with lam > 0 it is not
        (n = 9, lam = 0.5: q(0) - q(7) = -0.246: neighbours agree more often at the far end from the base point).  Counting the flux from the other end mirrors q exactly (it is the same measure read backwards).
  E3.3  Pin.  With closure, the marginals with a pin at node m, read backwards, are the marginals with the pin at n-1-m (1e-12).  The influence centroid is
        on the far side from the pin (the constraint carries the influence to the other end, E1.4), changes sign when the pin is moved to the mirror node (n = 13:
        +0.147 for a pin at 2, -0.147 at 10) and is 0 for a central pin.  The asymmetry follows the boundary data and nothing else.
  E3.4  Damping.  The centred scheme for psi_tt / (c^2 rho) - Lap psi + gamma psi_t = 0 on a ring retraces its steps when run backwards if and only if gamma = 0
        (error 1e-12 against O(1)); with gamma -> -gamma on the way back it retraces for every gamma (1e-9).  The energy falls for gamma > 0, is constant to 1e-3 for gamma = 0
        and grows for gamma < 0.  The sign of gamma is an orientation, independent of every other input (nothing else in this file enters the wave equation).
  E3.5  tau.  The mean flip time is late on the chain's flow, early on the reversed flow (mirror image, 1e-12), and central when the angles are replaced by their mean
        (orientation switched off); the final amplitudes are the same in all three cases (same total angle).
  E3.6  Handedness.  Mirroring the tile changes the sign of the turn at P_8 (a parity-odd observable exists) and leaves the jump d, the area share, the validity and
        therefore every angle theta_j and every cost of the chain unchanged.  The chain's arrow does not know the handedness.
  E3.7  Census (D3.4), closed hexagonal timelines of n = 2..8 steps: counts in ``CENSUS`` (strict and cyclic).  Rotation-reversible timelines have signed area 0.
        Cyclic: the oriented ones are 0 of 6, 0 of 12, 0 of 90, 240 of 360, 864 of 2040, 8568 of 10080 and 43488 of 54810: an intrinsic orientation first appears at
        5 steps and is the majority at 5, 7 and 8 steps (0.67, 0.42, 0.85, 0.79 for n = 5..8).  So an oriented closed timeline is the generic case, not a special one.
        The outline of the Spectre family (14 steps of the 12-direction alphabet; the direction word is the same for every a, b) is oriented in the same sense; reversing
        the order of its turning word gives the turning word of the mirror image traversed backwards, and not the word itself (C4).
  E3.8  Extension (D3.5): h is odd and c even about tau = 1 to 1e-12 and both agree with the chain's on [0, 1].
  E3.9  Records.  p = (1 + sqrt(1 - c_j^2) cos Theta_ij) / 2, Theta_ij the tunnelling angle between the ticks (closed form = the register's own readout, 1e-12).
        Hence K(i, j) >= K(j, i) exactly when tick j is the more readable one (c_j <= c_i): of two ticks, the one nearer the Spectre end holds the better record of the other.
        W falls strictly along tau and W = 0, K(i, Hat) = 0 at the Hat: the Hat end holds no record of anything.  The Spectre end holds at least 0.77 bit about every tick before the Hat
        (default run).  On the extended flow no record crosses the Hat (the register has rank 1 there), and on both sides the bits held are flat over the frozen ticks
        (10.669 within 1e-4) and fall strictly from tick 4 to 0 at the Hat: the record-holding region is the frozen one, away from the Hat.

  E3.10 Algebra of D3.7: for reading B, c = exp(-d^2 / 8 sigma^2) with d^2 = -4 sigma^2 ln(1 - r_s / r) gives c^2 = 1 - r_s / r exactly, and d^2 r / (4 sigma^2 r_s) -> 1
        for large r (1.005 at r = 100 r_s); the two readings are exchanged by c^2 <-> 1 - c^2, i.e. by overlap <-> Helstrom bias, and agree at c^2 = 1/2 (r = 2 r_s).

What the audit says (the plan's 'which of (i) to (v) carries the arrow')
  * Three inputs carry an arrow and are put in by hand: boundary data ((i) and (i')), the sign of gamma (ii), the orientation of tau (iii).  Each is switched off and
    reversed above; each observable follows its own input.  They sit in three different models, so their independence is by construction, and no model here couples them.
  * Two inputs are binary data with no arrow attached: handedness (iv) and scale ordering (v).  Coupling either to a cost would be a new postulate.
  * The record test does not agree with the chain's drawn orientation.  The chain runs from the Spectre (readable, frozen) to the Hat (unreadable, plastic); records sit
    at the Spectre end.  If 'later' is defined as the end that holds records, the flow runs from the Hat to the Spectre.  If 'earlier' is defined as the frozen, determined end
    (the chain's reading and ARROW_OF_TIME.md), then the present does not hold a record of the past in the register and the record must be the past itself.
    The plan's condition 'more than one input is needed and they disagree on direction' is met in this sense.  [Negative for 'one arrow'; a result of the stage]

Hypothesis / not done: no external record store, no environment (where the annihilated odd sector goes), no coupling between the three models, no continuation of the
dynamics through the Hat (only of the laws).

    python3 tm2_arrow.py --tests-only
    python3 tm2_arrow.py --story
    python3 tm2_arrow.py --no-show --save figures/tm2_stage3_arrow.png
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import core  # noqa: E402
import spectre_operator as so  # noqa: E402
import tm2_history_sum as hsum  # noqa: E402
from spectre_hat_backward_snap_model import BackwardSnap  # noqa: E402
from spectre_hat_quantum_bit_model import SpectreHatQubit  # noqa: E402

HEX = np.array(core.HEX)
BIG = hsum.BIG

# E3.7: n -> (closed, strict rotation-reversible, strict mirror-reversible, strict oriented, cyclic rotation-reversible, cyclic mirror-reversible, cyclic oriented)
CENSUS = {
    2: (6, 6, 6, 0, 6, 6, 0),
    3: (12, 0, 12, 0, 0, 12, 0),
    4: (90, 42, 66, 0, 66, 90, 0),
    5: (360, 0, 24, 336, 0, 120, 240),
    6: (2040, 228, 240, 1596, 708, 672, 864),
    7: (10080, 0, 216, 9864, 0, 1512, 8568),
    8: (54810, 1386, 1506, 51984, 5922, 6618, 43488),
}


# -- (i), (i'): the action and the history-sum ---------------------------------------------------------
def reverse_action_difference(seq, lam):
    """A(R s) - A(s)."""
    seq = np.asarray(seq)
    return hsum.action(seq[::-1], lam) - hsum.action(seq, lam)


def agreement_profile(n, beta, lam, d2):
    """q(t) = P(s(t+1) = s(t)), t = 0..n-2, from the exact marginals with a pin (the measure is invariant under the sixfold rotation)."""
    hs = hsum.HistorySum(n, beta, lam, d2)
    return np.array([hs.marginals((t, 0))[0][t + 1][0] for t in range(n - 1)])


def influence_centroid(n, m, beta=1.0, lam=0.0, d2=0):
    """Centroid of the influence profile of a pin at m (the pin itself excluded) minus the centre (n-1)/2."""
    iu = hsum.HistorySum(n, beta, lam, d2).influence(m).copy()
    iu[m] = 0.0
    return float((np.arange(n) * iu).sum() / iu.sum() - (n - 1) / 2.0)


# -- (ii): the damped wave equation on a ring -----------------------------------------------------------
def wave_run(psi0, psi1, gamma, steps, dt=0.05, c2rho=1.0):
    """Centred scheme for psi_tt / c2rho - Lap psi + gamma psi_t = 0 on a ring; returns the last two time levels (prev, cur)."""
    prev, cur = np.array(psi0, dtype=float), np.array(psi1, dtype=float)
    a, b = 1.0 / (c2rho * dt * dt), gamma / (2.0 * dt)
    for _ in range(int(steps)):
        lap = np.roll(cur, 1) + np.roll(cur, -1) - 2.0 * cur
        nxt = (lap + a * (2.0 * cur - prev) + b * prev) / (a + b)
        prev, cur = cur, nxt
    return prev, cur


def wave_energy(prev, cur, dt=0.05, c2rho=1.0):
    v = (cur - prev) / dt
    g = np.roll(cur, -1) - cur
    gp = np.roll(prev, -1) - prev
    return float(0.5 * (v * v).sum() / c2rho + 0.5 * (g * gp).sum())


def wave_initial(n=32, dt=0.05):
    x = np.arange(n)
    p0 = np.exp(-0.5 * ((x - n / 2.0) / 2.5) ** 2)
    p1 = np.exp(-0.5 * ((x - n / 2.0 - 0.6 * dt) / 2.5) ** 2)
    return p0, p1


def retrace_error(gamma, gamma_back=None, steps=400):
    """Run forward ``steps`` with gamma, swap the two last levels, run ``steps`` with gamma_back (default gamma): distance to the start."""
    p0, p1 = wave_initial()
    a, b = wave_run(p0, p1, gamma, steps)
    c, d = wave_run(b, a, gamma if gamma_back is None else gamma_back, steps)
    return float(max(np.abs(d - p0).max(), np.abs(c - p1).max()))


def energy_ratio(gamma, steps=400):
    p0, p1 = wave_initial()
    a, b = wave_run(p0, p1, gamma, steps)
    return wave_energy(a, b) / wave_energy(p0, p1)


# -- (iii): the orientation of tau ------------------------------------------------------------------------
def flat_schedule(m):
    """The chain with every angle replaced by the mean angle (orientation of tau switched off; same total angle)."""
    f = BackwardSnap.__new__(BackwardSnap)
    f.qubit, f.n = m.qubit, m.n
    f._angles = np.full(m.n, m.angles().mean())
    f._weights, f._paths = m.weights(), None
    return f


def mean_flip_time(m):
    """Mean tick of the single flip minus the centre (n-1)/2."""
    d = m.flip_time_distribution()
    return float((np.arange(m.n) * d).sum() - (m.n - 1) / 2.0)


# -- (iv): handedness ----------------------------------------------------------------------------------------
def tile_points(q, tau, s, mirror=False):
    """Vertex positions of the chain's tile (bit s) at tau; mirrored in the x axis if asked."""
    pts = np.array([v.position for v in q.time.tile(tau, s).vertices], dtype=float)
    if mirror:
        pts = pts * np.array([1.0, -1.0])
    return pts


def signed_turn(pts, i):
    """Signed turn (radians) of the polygon at vertex i."""
    a, b = pts[i] - pts[i - 1], pts[(i + 1) % len(pts)] - pts[i]
    return math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1])


def signed_area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return 0.5 * float((x * np.roll(y, -1) - np.roll(x, -1) * y).sum())


def flat_vertex_of(q):
    """Index of P_8 in the chain's tile: the vertex the bit moves."""
    a, b = tile_points(q, 0.5, 1), tile_points(q, 0.5, -1)
    moved = np.flatnonzero(np.abs(a - b).sum(axis=1) > 1e-12)
    assert len(moved) == 1
    return int(moved[0])


# -- C4: the census ----------------------------------------------------------------------------------------------
def _images(T, order):
    """The images of the timelines T (N, n) under the rotations (det +1) and reflections (det -1) of an alphabet of ``order`` directions."""
    rots = [(T + r) % order for r in range(order)]
    refl = [(-T + r) % order for r in range(order)]
    return rots, refl


def _equal_any(R, imgs, cyclic):
    n = R.shape[1]
    hit = np.zeros(len(R), dtype=bool)
    for g in imgs:
        for sh in (range(n) if cyclic else (0,)):
            hit |= np.all(R == np.roll(g, sh, axis=1), axis=1)
    return hit


def census(n):
    """D3.4 for the closed hexagonal timelines of n steps: the tuple of ``CENSUS``."""
    T = core.closed_timelines("hex", n).astype(int)
    R = T[:, ::-1]
    rots, refl = _images(T, 6)
    out = [len(T)]
    for cyc in (False, True):
        a, b = _equal_any(R, rots, cyc), _equal_any(R, refl, cyc)
        out += [int(a.sum()), int(b.sum()), int((~a & ~b).sum())]
    return tuple(out)


def timeline_signed_area(T):
    """Twice the signed area of the closed polygons of the timelines T (integer, units of the lattice cell)."""
    S = HEX[T]
    P = np.cumsum(S, axis=1)
    return (P[:, :-1, 0] * S[:, 1:, 1] - P[:, :-1, 1] * S[:, 1:, 0]).sum(axis=1)


def outline_word(a=1.0, b=1.0, mirror=False):
    """Edge directions of Tile(a, b) as integers mod 12 (multiples of 30 degrees)."""
    e = np.array(so.edge_vectors(a, b), dtype=float)
    if mirror:
        e = e * np.array([1.0, -1.0])
    ang = np.degrees(np.arctan2(e[:, 1], e[:, 0])) / 30.0
    w = np.rint(ang).astype(int)
    assert np.allclose(ang, w, atol=1e-9)
    return w % 12


def turning_word(w, order=12):
    """Turns between consecutive edges (cyclic), in units of 2 pi / order, in (-order/2, order/2]."""
    t = (np.roll(w, -1) - w) % order
    return np.where(t > order // 2, t - order, t)


def word_class(w, order=12):
    """(rotation-reversible, mirror-reversible) of one closed word, cyclic sense."""
    T = np.asarray(w)[None, :]
    rots, refl = _images(T, order)
    R = T[:, ::-1]
    return bool(_equal_any(R, rots, True)[0]), bool(_equal_any(R, refl, True)[0])


# -- C9: the extension and the records -----------------------------------------------------------------------------
def h_ext(q, tau):
    """h(tau) = a tan(omega_0 (1 - tau) / 2) on [0, 2] (D3.5)."""
    if not 0.0 <= tau <= 2.0:
        raise ValueError("tau must be in [0, 2]")
    return q.time.a * math.tan(q.time.omega0 * (1.0 - tau) / 2.0)


def c_ext(q, tau):
    h = h_ext(q, tau)
    return math.exp(-h * h / (2.0 * q.sigma ** 2))


def H2(p):
    p = min(max(p, 0.0), 1.0)
    if p in (0.0, 1.0):
        return 0.0
    return -(p * math.log2(p) + (1.0 - p) * math.log2(1.0 - p))


def radius(c, reading):
    """r / r_s for overlap c under reading 'A' (c = sigma_t) or 'B' (c = clock), D3.7; inf where the reading puts the tick at infinity."""
    if reading not in ("A", "B"):
        raise ValueError("reading must be 'A' or 'B'")
    x = c * c if reading == "A" else -math.expm1(2.0 * math.log(c)) if c > 0.0 else 1.0
    return math.inf if x <= 0.0 else 1.0 / x


def jump2_of_radius(r, sigma, reading="B"):
    """d^2 at r (units of r_s) under a reading of D3.7."""
    if r < 1.0:
        raise ValueError("r must be >= 1 (units of r_s)")
    c2 = 1.0 / r if reading == "A" else 1.0 - 1.0 / r
    return math.inf if c2 <= 0.0 else -4.0 * sigma ** 2 * math.log(c2)


class Records:
    """D3.6 on the chain's ticks 0..N (N is the Hat), or on the extended flow 0..2N with ``extended=True``."""

    def __init__(self, q=None, extended=False):
        self.q = SpectreHatQubit(ticks=12) if q is None else q
        self.N = self.q.N
        self.extended = bool(extended)
        self.last = 2 * self.N if extended else self.N
        self.c = np.array([c_ext(self.q, j / self.N) for j in range(self.last + 1)])
        # theta_j: the tunnelling between tick j and j + 1 (the chain's theta_j before the Hat, its mirror image after)
        th = [self.q.snap_angle(j) if j < self.N else (self.q.snap_angle(2 * self.N - 1 - j) if j < 2 * self.N else 0.0) for j in range(self.last + 1)]
        self.theta = np.array(th)

    def helstrom(self, j):
        return math.sqrt(max(0.0, 1.0 - self.c[j] ** 2)) if j != self.N else 0.0

    def angle(self, i, j):
        lo, hi = min(i, j), max(i, j)
        return float(self.theta[lo:hi].sum())

    def crosses_hat(self, i, j):
        return min(i, j) < self.N < max(i, j)

    def p_correct(self, i, j):
        """Probability that the optimal reading at tick j returns the bit of tick i."""
        if self.crosses_hat(i, j) or self.N in (i, j) and i != j:
            return 0.5  # rank 1 at the Hat: nothing passes, and the Hat's own 'bit' is not defined
        return 0.5 * (1.0 + self.helstrom(j) * math.cos(self.angle(i, j)))

    def W(self, j):
        return 1.0 - H2(0.5 * (1.0 + self.helstrom(j)))

    def K(self, i, j):
        return 1.0 - H2(self.p_correct(i, j))

    def held_at(self, j):
        """Sum over the other ticks i of K(i, j): bits the register at tick j holds about the rest of the timeline."""
        return float(sum(self.K(i, j) for i in range(self.last + 1) if i != j))


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    except Exception:  # noqa: BLE001
        return False
    return False


def test_e31_the_action_is_reversal_symmetric_exactly_on_closed_timelines():
    for n, count in ((3, 12), (4, 90), (5, 360), (6, 2040), (7, 10080)):
        T = core.closed_timelines("hex", n).astype(int)
        assert len(T) == count
        for lam in (0.0, 0.5, 2.0):
            S = HEX[T]
            Sr = S[:, ::-1]
            A = lambda X: (hsum.norm2(X[:, 1:] - X[:, :-1]).sum(axis=1) + lam * hsum.norm2(np.cumsum(X, axis=1)[:, 1:]).sum(axis=1))  # noqa: E731
            assert np.array_equal(A(S), A(Sr)), (n, lam)
    assert reverse_action_difference(T[17], 0.5) == 0.0  # the vectorised form agrees with hsum.action
    U = core.all_timelines("hex", 5).astype(int)
    diff = np.array([reverse_action_difference(u, 1.0) for u in U])
    assert int((np.abs(diff) > 1e-12).sum()) == 6624 and len(U) == 7776
    # the identity for open timelines
    for u in U[::97]:
        Phi = np.cumsum(HEX[u], axis=0)
        F = Phi[-1]
        rhs = hsum.norm2(F) + hsum.norm2(F - Phi[:-2]).sum() - hsum.norm2(Phi[1:]).sum()
        assert math.isclose(reverse_action_difference(u, 1.0), float(rhs), abs_tol=1e-12)
    assert all(reverse_action_difference(u, 0.0) == 0.0 for u in U[::97]), "without the flux term there is no orientation"


def test_e32_the_base_point_of_the_flux_is_an_orientation_of_the_open_measure():
    for lam in (0.0, 0.5):
        q = agreement_profile(9, 1.0, lam, 0)
        assert np.allclose(q, q[::-1], atol=1e-12), "closed: symmetric"
    q0 = agreement_profile(9, 1.0, 0.0, BIG)
    assert np.allclose(q0, q0[::-1], atol=1e-12) and np.allclose(q0, q0[0], atol=1e-12), "open, no flux term: symmetric (and flat)"
    q = agreement_profile(9, 1.0, 0.5, BIG)
    assert not np.allclose(q, q[::-1], atol=1e-3)
    assert math.isclose(q[0] - q[-1], -0.246, abs_tol=5e-4), q[0] - q[-1]
    # mutation: a wrong (symmetrised) observable would hide it
    assert abs((q + q[::-1])[0] - (q + q[::-1])[-1]) < 1e-12


def test_e33_a_pin_carries_the_asymmetry_and_it_follows_the_pin():
    n = 13
    hs = hsum.HistorySum(n, 1.0, 0.3, 0)
    for m in (0, 2, 5):
        P, _ = hs.marginals((m, 1))
        Pm, _ = hs.marginals((n - 1 - m, 1))
        assert np.allclose(P[::-1], Pm, atol=1e-12), m
    a, b, c = influence_centroid(n, 2), influence_centroid(n, n - 3), influence_centroid(n, 6)
    assert math.isclose(a, 0.147, abs_tol=5e-4) and math.isclose(a, -b, abs_tol=1e-10) and abs(c) < 1e-10, (a, b, c)
    free, _ = hs.marginals()
    assert np.allclose(free, 1.0 / 6.0, atol=1e-12), "no pin: every node uniform, nothing to orient"


def test_e34_damping_is_an_orientation():
    assert retrace_error(0.0) < 1e-10
    for g in (0.05, 0.3):
        assert retrace_error(g) > 1e-2, "with damping the reversed run does not retrace"
        assert retrace_error(g, -g) < 1e-8, "it retraces only if the sign of gamma is reversed too"
    e0, ep, em = energy_ratio(0.0), energy_ratio(0.1), energy_ratio(-0.1)
    assert abs(e0 - 1.0) < 1e-3 and ep < 0.5 and em > 2.0, (e0, ep, em)
    assert math.isclose(math.log(ep), -math.log(em), rel_tol=0.05), "reversing gamma reverses the energy flow"


def test_e35_the_orientation_of_tau():
    m = BackwardSnap(ticks=12)
    r, f = m.reversed_flow(), flat_schedule(m)
    a, b, c = mean_flip_time(m), mean_flip_time(r), mean_flip_time(f)
    assert a > 4.0 and math.isclose(a, -b, abs_tol=1e-12) and abs(c) < 1e-12, (a, b, c)
    for x, y, z in zip(m.final_amplitudes(), r.final_amplitudes(), f.final_amplitudes()):
        assert abs(x - y) < 1e-13 and abs(x - z) < 1e-13
    assert math.isclose(f.total_angle(), m.total_angle(), rel_tol=1e-14)


def test_e36_the_chain_does_not_know_the_handedness():
    q = SpectreHatQubit(ticks=12)
    i8 = flat_vertex_of(q)
    for tau in (0.0, 0.3, 0.75):
        turns, jumps, areas = [], [], []
        for mirror in (False, True):
            p, mns = tile_points(q, tau, 1, mirror), tile_points(q, tau, -1, mirror)
            turns.append((signed_turn(p, i8), signed_turn(mns, i8)))
            jumps.append(float(np.hypot(*(p[i8] - mns[i8]))))
            areas.append(abs(signed_area(p)) - abs(signed_area(mns)))
            assert signed_area(p) * (-1 if mirror else 1) * np.sign(signed_area(tile_points(q, tau, 1))) > 0
        assert math.isclose(turns[0][0], -turns[1][0], abs_tol=1e-12) and math.isclose(turns[0][1], -turns[1][1], abs_tol=1e-12)
        assert abs(turns[0][0]) > 1e-3, "the parity-odd observable is not zero"
        assert math.isclose(jumps[0], jumps[1], rel_tol=1e-13) and math.isclose(jumps[0], q.jump(tau), rel_tol=1e-9)
        assert math.isclose(areas[0], areas[1], rel_tol=1e-12)
        assert math.isclose(turns[0][0], -turns[0][1], abs_tol=1e-9), "the bit is the sign of the turn"
    # the angles are functions of the jump only
    assert math.isclose(q.overlap(0.75), math.exp(-q.jump(0.75) ** 2 / (8 * q.sigma ** 2)), rel_tol=1e-14)


def test_e37_census_and_the_spectre():
    for n in range(2, 9):
        got = census(n)
        assert got == CENSUS[n], (n, got)
        closed, sr, sm, so_, cr, cm, co = got
        assert closed == core.LEDGER_HEX[n] and sr + sm - (closed - so_) >= 0 and co <= so_
        T = core.closed_timelines("hex", n).astype(int)
        rots, _ = _images(T, 6)
        rr = _equal_any(T[:, ::-1], rots, True)
        assert np.all(timeline_signed_area(T)[rr] == 0), "a rotation-reversible closed timeline encloses zero signed area"
    assert [CENSUS[n][6] for n in range(2, 9)] == [0, 0, 0, 240, 864, 8568, 43488]
    for a, b in ((1.0, 1.0), (1.0, math.sqrt(3.0)), (math.sqrt(3.0), 1.0)):
        w = outline_word(a, b)
        assert len(w) == 14 and word_class(w) == (False, False), "the outline of the family is oriented"
        assert np.array_equal(w, outline_word()), "the direction word does not depend on a, b"
    # C4: reversed turning word = turning word of the mirror image traversed backwards; and = minus the turning word of R(w)
    w = outline_word()
    tw = turning_word(w)
    mirror_back = (outline_word(mirror=True)[::-1] + 6) % 12
    tmb = turning_word(mirror_back)
    rev = tw[::-1]
    assert any(np.array_equal(np.roll(rev, k), tmb) for k in range(14))
    assert not any(np.array_equal(np.roll(rev, k), tw) for k in range(14)), "and it is not the word itself: the tile is chiral"
    assert int(tw.sum()) in (12, -12) and int((tw == 0).sum()) == 1, "one full turn, exactly one flat vertex"
    # a mirror-symmetric control: the regular hexagon
    assert word_class(np.arange(6), 6) == (False, True)


def test_e38_extension_beyond_the_hat():
    q = SpectreHatQubit(ticks=12)
    for tau in (0.0, 0.25, 0.5, 0.9, 1.0):
        assert math.isclose(2.0 * h_ext(q, tau), q.jump(tau), rel_tol=1e-12, abs_tol=1e-15)
        assert math.isclose(c_ext(q, tau), q.overlap(tau), rel_tol=1e-12)
        assert math.isclose(h_ext(q, 2.0 - tau), -h_ext(q, tau), rel_tol=1e-12, abs_tol=1e-15)
        assert math.isclose(c_ext(q, 2.0 - tau), c_ext(q, tau), rel_tol=1e-12)
    assert h_ext(q, 1.0) == 0.0 and c_ext(q, 1.0) == 1.0
    assert _raises(ValueError, h_ext, q, 2.1) and _raises(ValueError, h_ext, q, -0.1)


def test_e39_records_sit_at_the_readable_end():
    q = SpectreHatQubit(ticks=12)
    rec = Records(q)
    N = rec.N
    # closed form against the register's own readout
    for i, j in ((2, 9), (9, 2), (5, 11), (11, 5), (10, 11), (0, 11)):
        psi = np.array([1.0 + 0j, 0j])
        lo, hi = min(i, j), max(i, j)
        for k in range(lo, hi):
            psi = q.flip(psi, q.snap_angle(k) * (1 if i < j else -1))
        assert math.isclose(q.readout_probs(j / N, tuple(psi))[0], rec.p_correct(i, j), rel_tol=1e-12), (i, j)
    for i in range(N):
        for j in range(i + 1, N):
            assert rec.c[i] < rec.c[j] and rec.K(j, i) >= rec.K(i, j), "the more readable tick holds the better record"
            if j >= 6:
                assert rec.K(j, i) > rec.K(i, j)
    W = [rec.W(j) for j in range(N + 1)]
    assert np.all(np.diff(W) <= 0) and np.all(np.diff(W[5:]) < 0)
    assert W[N] == 0.0 and W[0] > 0.999999
    assert all(rec.K(i, N) == 0.0 for i in range(N)) and rec.held_at(N) == 0.0, "the Hat end holds no record"
    assert math.isclose(min(rec.K(i, 0) for i in range(1, N)), 0.777, abs_tol=1e-3) and rec.held_at(0) > 10.0
    assert q.readout_probs(1.0, (0.3, 0.8j)) == (0.5, 0.5), "rank 1 at the Hat: every state reads the same"
    # extended flow: nothing crosses, both sides mirror, the record-holding ends are the two ends away from the Hat
    ex = Records(q, extended=True)
    assert np.allclose(ex.c, ex.c[::-1], atol=1e-12) and np.allclose(ex.theta[:2 * N], ex.theta[:2 * N][::-1], atol=0)
    for i in range(N):
        for j in range(N + 1, 2 * N + 1):
            assert ex.K(i, j) == 0.0 and ex.K(j, i) == 0.0
    held = [ex.held_at(j) for j in range(2 * N + 1)]
    assert np.allclose(held, held[::-1], atol=1e-9) and int(np.argmin(held)) == N and max(held) - held[0] < 1e-4
    assert np.all(np.abs(np.diff(held[:5])) < 1e-4) and np.all(np.diff(held[4: N + 1]) < 0), "flat over the frozen ticks, then falling to 0 at the Hat"


def test_e310_two_radial_readings():
    sigma = 0.05
    for r in (1.5, 2.0, 7.0, 100.0):
        d2 = jump2_of_radius(r, sigma, "B")
        c = math.exp(-d2 / (8.0 * sigma ** 2))
        assert math.isclose(c * c, 1.0 - 1.0 / r, rel_tol=1e-12) and math.isclose(radius(c, "B"), r, rel_tol=1e-9)
        cA = math.exp(-jump2_of_radius(r, sigma, "A") / (8.0 * sigma ** 2))
        assert math.isclose(cA * cA, 1.0 / r, rel_tol=1e-12) and math.isclose(radius(cA, "A"), r, rel_tol=1e-9)
        assert math.isclose(radius(math.sqrt(1.0 - c * c), "A"), radius(c, "B"), rel_tol=1e-9), "overlap <-> Helstrom bias exchanges the readings"
    assert math.isclose(jump2_of_radius(100.0, sigma) * 100.0 / (4.0 * sigma ** 2), 1.005, abs_tol=1e-3)
    assert math.isclose(radius(math.sqrt(0.5), "A"), 2.0, rel_tol=1e-12) and math.isclose(radius(math.sqrt(0.5), "B"), 2.0, rel_tol=1e-12)
    assert radius(1.0, "B") == math.inf and radius(1.0, "A") == 1.0 and radius(0.0, "A") == math.inf and radius(0.0, "B") == 1.0
    assert jump2_of_radius(1.0, sigma, "B") == math.inf and jump2_of_radius(1.0, sigma, "A") == 0.0
    assert _raises(ValueError, radius, 0.5, "C") and _raises(ValueError, jump2_of_radius, 0.5, sigma)
    # on the chain: under B the frozen ticks hug the horizon and the plastic ones are far; under A the other way round
    rec = Records()
    rB = [radius(rec.c[j], "B") for j in range(rec.N + 1)]
    rA = [radius(rec.c[j], "A") for j in range(rec.N + 1)]
    assert np.all(np.diff(rB) >= 0) and np.all(np.diff(rA) < 0) and rB[0] < 1.0 + 1e-9 and rB[-1] == math.inf and rA[-1] == 1.0


def test_validation_and_helpers():
    assert H2(0.5) == 1.0 and H2(0.0) == 0.0 and H2(1.0) == 0.0
    assert _raises(ValueError, hsum.HistorySum(5, 1.0, 0.0, 0).marginals, (9, 0))
    assert signed_turn(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]), 1) > 0
    assert signed_area(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])) == 1.0


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


# --------------------------------------------------------------------------
# Story and figure
# --------------------------------------------------------------------------
def story():
    print("audit: observable with the input on / off / reversed")
    print("  (i)   pin, influence centroid (n = 13)        : %+.3f / %+.3f / %+.3f   (pin at 2 / centre / 10)" % (influence_centroid(13, 2), influence_centroid(13, 6), influence_centroid(13, 10)))
    q = agreement_profile(9, 1.0, 0.5, BIG)
    q0 = agreement_profile(9, 1.0, 0.0, BIG)
    print("  (i')  base point of Phi, q(0) - q(n-2) (n = 9) : %+.3f / %+.3f / %+.3f   (lam 0.5 / lam 0 / counted from the other end)" % (q[0] - q[-1], q0[0] - q0[-1], q[-1] - q[0]))
    print("  (ii)  damping, energy end / start              : %.3f / %.3f / %.3f   (gamma +0.1 / 0 / -0.1)" % (energy_ratio(0.1), energy_ratio(0.0), energy_ratio(-0.1)))
    m = BackwardSnap(ticks=12)
    print("  (iii) tau, mean flip tick minus centre         : %+.3f / %+.3f / %+.3f   (chain / flat angles / reversed flow)" % (mean_flip_time(m), mean_flip_time(flat_schedule(m)), mean_flip_time(m.reversed_flow())))
    qb = SpectreHatQubit(ticks=12)
    i8 = flat_vertex_of(qb)
    t0, t1 = signed_turn(tile_points(qb, 0.5, 1), i8), signed_turn(tile_points(qb, 0.5, 1, True), i8)
    print("  (iv)  handedness, turn at P_8 (deg), jump       : %+.2f, %.6f | mirror %+.2f, %.6f   (costs unchanged)" % (math.degrees(t0), qb.jump(0.5), math.degrees(t1), qb.jump(0.5)))
    print("  (v)   scale ordering                           : not coupled to any cost (tm2_directed.ladder)")
    print("census of closed hexagonal timelines (n: closed | strict rot, mirror, oriented | cyclic rot, mirror, oriented)")
    for n in range(2, 9):
        c = CENSUS[n]
        print("  %d: %6d | %5d %5d %6d | %6d %6d %6d   oriented fraction (cyclic) %.3f" % ((n,) + c + (c[6] / c[0],)))
    rec = Records(qb)
    print("records on the chain (tick: c, W = bits readable, K(tick -> Spectre end), K(tick -> last tick before the Hat), bits held at the tick)")
    for j in (0, 3, 6, 8, 9, 10, 11, 12):
        print("  %2d: c = %.3e  W = %.4f  K(j,0) = %.4f  K(j,11) = %.4f  held = %.3f" % (j, rec.c[j], rec.W(j), rec.K(j, 0) if j else 1.0, rec.K(j, 11) if j != 11 else rec.W(11), rec.held_at(j)))


    d = BackwardSnap(ticks=12).flip_time_distribution()
    print("two radial readings (Hypothesis): tick, r/r_s under A (c = sigma_t), under B (c = clock), share of the single flip")
    for j in (0, 6, 8, 9, 10, 11, 12):
        print("  %2d: A %10.4g   B %10.6g   flip share %s" % (j, radius(rec.c[j], "A"), radius(rec.c[j], "B"), "%.4f" % d[j] if j < 12 else "  -"))


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(18, 4.2))
    q = agreement_profile(9, 1.0, 0.5, BIG)
    ax[0].plot(range(8), q, "o-", label="open, lam = 0.5")
    ax[0].plot(range(8), agreement_profile(9, 1.0, 0.5, 0), "s-", label="closed, lam = 0.5")
    ax[0].plot(range(8), agreement_profile(9, 1.0, 0.0, BIG), "^-", label="open, lam = 0")
    ax[0].set_xlabel("t")
    ax[0].set_ylabel("P(s(t+1) = s(t))")
    ax[0].set_title("(i') base point of the flux")
    ax[0].legend(fontsize=8)
    hs = hsum.HistorySum(13, 1.0, 0.0, 0)
    for m_, st in ((2, "o-"), (6, "s-"), (10, "^-")):
        iu = hs.influence(m_)
        ax[1].plot(range(13), iu, st, label="pin at %d" % m_)
    ax[1].set_xlabel("node")
    ax[1].set_ylabel("influence (TV)")
    ax[1].set_title("(i) the asymmetry follows the pin")
    ax[1].legend(fontsize=8)
    ns = list(range(2, 9))
    ax[2].plot(ns, [CENSUS[n][3] / CENSUS[n][0] for n in ns], "o-", label="strict")
    ax[2].plot(ns, [CENSUS[n][6] / CENSUS[n][0] for n in ns], "s-", label="cyclic")
    ax[2].set_xlabel("steps n")
    ax[2].set_ylabel("oriented fraction of closed timelines")
    ax[2].set_title("census (C4)")
    ax[2].legend(fontsize=8)
    ex = Records(extended=True)
    x = np.arange(2 * ex.N + 1) / ex.N
    ax[3].plot(x, [ex.W(j) for j in range(2 * ex.N + 1)], "o-", label="W: bits readable at the tick")
    ax[3].plot(x, np.array([ex.held_at(j) for j in range(2 * ex.N + 1)]) / ex.N, "s-", label="bits held about the others / N")
    ax[3].axvline(1.0, color="k", lw=0.6)
    ax[3].set_xlabel("tau (Hat at 1, extended flow)")
    ax[3].set_title("records sit away from the Hat")
    ax[3].legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=130)
    if show:
        plt.show()
    plt.close(fig)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    if "--save" in argv:
        figure(save=argv[argv.index("--save") + 1], show="--no-show" not in argv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
