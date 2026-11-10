"""
spectre_hat_time_model.py - time, two arrows and one stored bit at the 14th vertex.

A toy model on top of spectre_hat_collinearity_flow_model.py (the Spectre -> Hat flow).  It
makes precise an idea about time: the *smooth* tile flows forward in tau and that direction
is fixed; the only place where something *discrete* can happen is the 14th vertex P_8, which
stores one bit, and that bit may "snap" (flip, or be rewritten in the past) as long as the
result stays consistent with the smooth flow that continues.

Status labels (as in QUANTUM_GRAVITY.md): **Exact** = checked by the self tests of this file
(python3 spectre_hat_time_model.py --tests-only); **Definition** = a choice made here, not
derived from anything; **Hypothesis** = a physical reading that nothing in this file tests.
Nothing here is a derivation of anything in physics: the physics section at the end says
which words are borrowed and what does not carry over.

    from spectre_hat_time_model import SpectreHatTime

    m = SpectreHatTime(ticks=20, corner_deg=30, snap_scale=0.1)
    m.step(18)                         # smooth time runs forward; the bit persists
    m.can_snap(17)                     # (True, [])   rewrite the record of tick 17
    m.snap(17)                         # a snap back by one tick; present and future unchanged
    m.snap()                           # flip the present bit; the future outline changes
    m.can_snap(3)                      # (False, ['cost: ...'])  early ticks are set in stone
    m.arrows(m.tau)                    # the two arrows at P_8
    m.step(-1)                         # ArrowError: the smooth flow only runs forward

Objects
-------
D1  Smooth time.  tau in [0, 1], the flow of SpectreHatFlow: b(tau), k(tau), omega(tau) and
    h(tau) = a tan(omega/2) as in that module.  Everything except P_8 depends on tau only.
D2  The bit.  P_8 stores s = +1 or -1 per tick.  s = +1 pushes P_8 *out* of the chord
    P_7 -> P_9 by h(tau) (a convex corner, turn -omega), s = -1 pushes it *in* by h(tau)
    (a reflex corner, turn +omega).  The tile is the flow tile with P_8 at base + s*h*(1, 0).
D3  Ticks.  The bit lives on ticks 0..N, tick j at tau_j = j/N; its value holds on
    [tau_j, tau_(j+1)).  ``now`` is a tick; the record is the bits of ticks 0..now.  Stepping
    to a new tick copies the last bit (persistence).  The smooth outline is continuous in tau;
    only the bit is on a clock.
D4  The two arrows.  At P_8 the outline has an *incoming* arrow a_in (unit tangent of the
    curve arriving along edge 7) and an *outgoing* arrow a_out (unit tangent leaving along
    edge 8).  They are measured on the Bezier outline, not assumed.  The signed angle from a_in
    to a_out is the turn -s*omega.  This replaces the notes' arrow angle pi*(1 - tau), which
    equals omega only for omega_0 = pi; the geometry cannot reach that (h = a tan(omega/2)
    grows without bound), so omega_0 < pi here.  Two derived numbers: ``alignment`` =
    cos(turn) (1: past and future arrows aligned, a smooth worldline; 0: perpendicular;
    -1: a_out points back along a_in) and ``backward`` = max(0, -cos(turn)), the part of a_out
    that runs *against* the incoming direction.  It is non-zero only while omega > pi/2.
D5  Readability.  The two bit values differ only in the position of P_8, by the jump
    d(tau) = 2 h(tau).  ``readable(tau) = d(tau)/d(0)``.  At tau = 1 the two tiles are the
    same Hat and the bit cannot be read.
D6  The smooth arrow.  Time runs in the direction in which the bit becomes unreadable, i.e.
    of decreasing d(tau).  The flow could be run backwards as mathematics; the model forbids
    it (``step`` with a negative number raises ArrowError).

Dynamics: a snap, and the rules it must obey
--------------------------------------------
``snap()`` flips the present bit (a *forward* snap); ``snap(j)`` with j < now rewrites the
record of an earlier tick (a *backward* snap, ``depth`` = (now - j)/N).  Only the bit of P_8
can snap.  A snap is allowed when every rule holds (``can_snap`` lists the broken ones):
R1  record: the tick exists, 0 <= j <= now (the future has no record).
R2  cost: the jump d(tau_j) is at most ``snap_scale``.  d decreases, so early ticks are
    frozen ("set in stone") and the window of ticks that can still snap opens at
    tau* = 1 - 2 atan(snap_scale / (2a)) / omega_0 and stays open.
R3  validity: the outline with the new bit is a simple closed curve over the stretch of
    smooth flow it affects: [tau_j, tau_(j+1)] for a rewrite, [tau_now, 1] for a present flip
    (the new bit persists).
R4  smooth sector: every edge except 7 and 8 is the same with the old and the new bit.
R5  present and future (rewrites only): the tile at tau_now is the same before and after.
The future smooth flow is therefore *the same whatever the bits do* (R4, and the fact that all
histories end in the same Hat); the bits are constrained by it, not the other way round.

What is exact (tests)
---------------------
E1  The s = +1 tile is the SpectreHatFlow tile; the s = -1 tile differs from it only in
    edges 7 and 8, and not at all at tau = 1.
E2  The measured arrows turn by -s*omega(tau); at tau = 1 they are aligned.
E3  d(tau) = 2 a tan(omega_0 (1 - tau)/2) is strictly decreasing, also at tau = 1.
E4  The smooth sector and the final Hat do not depend on the bit history.
E5  The window of snappable ticks, its closed-form start tau*, and the frozen ticks.
E6  A present flip changes the future only at edges 7 and 8; a rewrite leaves present and
    future untouched.
E7  R3 is a real constraint for large omega_0 (at omega_0 = 150 degrees, k_0 = 0.5, the
    s = -1 tile self-intersects early in the flow; checked by the tests).

Physics reading (all **Hypothesis**; none of it is tested here)
---------------------------------------------------------------
* Smooth surface <-> a classical continuum geometry with a fixed time direction; the stored
  bit <-> a discrete degree of freedom living at one special point of that geometry.  The 14th
  vertex is the only place where both exist, which is the (loose) sense in which it "blends
  discrete and smooth".  Nothing here is a quantum field, a metric or an action.
* Forward snaps and backward snaps <-> a jump event, and a rewrite of a record, at the scale
  ``snap_scale``.  The rule that early snaps cost too much and late ones little is a
  stand-in for "classical early, quantum late"; ``snap_scale`` is a free parameter, not
  an hbar.
* The backward component of the arrows (omega > pi/2) <-> a worldline that doubles back in
  time, as in the Stueckelberg-Feynman reading of antiparticles.  The model has a geometric
  version of this (a_out against a_in) and no particles.
* Unreadability at tau = 1 <-> information loss in the direction of time (a Landauer-style
  arrow).  Here it is a property of the construction: the two tiles merge.
* R3-R5 <-> consistency of the record with what continues, in the spirit of consistent
  histories or self-consistency conditions.  These are definitions of a toy, not those theories.
What does not carry over: the bit is classical (no superposition, no amplitude, no unitarity);
the arrow of time is imposed by rule D6, not derived from dynamics; one bit per tick at one
vertex is far from any field theory; and no gravity is involved.

Only NumPy and the two modules of this repo are needed; matplotlib is imported lazily by the
command line (python3 spectre_hat_time_model.py --help).
"""

import argparse
import math
import sys

import numpy as np

from spectre_hat_collinearity_flow_model import FLAT_VERTEX, SpectreHatFlow
from spectremodel import N_EDGES

__all__ = [
    "ArrowError",
    "SnapRefused",
    "Arrows",
    "SpectreHatTime",
    "is_simple_tile",
    "run_tests",
]

OPENED_EDGES = (FLAT_VERTEX - 1, FLAT_VERTEX)  # edges 7 and 8 are the ones P_8 shapes


class ArrowError(RuntimeError):
    """The smooth flow was asked to run backwards."""


class SnapRefused(RuntimeError):
    """A snap broke at least one rule; ``reasons`` lists them."""

    def __init__(self, reasons):
        super().__init__("; ".join(reasons))
        self.reasons = list(reasons)


class Arrows:
    """The two arrows at P_8 (D4): unit vectors and what follows from them."""

    def __init__(self, a_in, a_out):
        self.a_in, self.a_out = a_in, a_out
        self.turn = math.atan2(a_in[0] * a_out[1] - a_in[1] * a_out[0], float(a_in @ a_out))
        self.alignment = float(a_in @ a_out)
        self.backward = max(0.0, -self.alignment)

    def __repr__(self):
        return "<Arrows turn=%.4g deg alignment=%.4f backward=%.4f>" % (
            math.degrees(self.turn), self.alignment, self.backward)


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def is_simple_tile(tile, samples=12):
    """True if the sampled outline of ``tile`` (14 * samples points) is a simple closed curve:
    no two edges of the polyline cross (neighbours are skipped)."""
    pts = tile.sample_edges(samples)[:, :-1].reshape(-1, 2)
    a, b = pts, np.roll(pts, -1, axis=0)
    n = len(pts)

    def side(p, q, r):
        return (r[..., 1] - p[..., 1]) * (q[..., 0] - p[..., 0]) - (q[..., 1] - p[..., 1]) * (r[..., 0] - p[..., 0])

    for i in range(n):
        js = np.array([j for j in range(n) if j not in (i, (i - 1) % n, (i + 1) % n)])
        c, d = a[js], b[js]
        s1, s2 = side(a[i], b[i], c), side(a[i], b[i], d)
        s3, s4 = side(c, d, a[i]), side(c, d, b[i])
        if np.any((s1 * s2 < 0) & (s3 * s4 < 0)):
            return False
    return True


def _smooth_part(tile):
    """Control points of every edge except 7 and 8: the smooth sector (R4)."""
    ctrl = tile.control_points()
    return ctrl[[i for i in range(N_EDGES) if i not in OPENED_EDGES]]


# --------------------------------------------------------------------------
# The model
# --------------------------------------------------------------------------
class SpectreHatTime:
    """Smooth time tau, a bit at the 14th vertex, and the snaps allowed between them.

    Parameters
    ----------
    ticks      : N >= 1, the bit lives on ticks 0..N at tau = j/N.
    k0, a, corner_deg : as in SpectreHatFlow (corner_deg = omega_0 in degrees, in (0, 180)).
    snap_scale : largest jump d = 2h that a snap may cause (rule R2), >= 0.
    bit        : the bit of tick 0, +1 or -1.
    grid       : number of tau samples used to check R3 and R4 (>= 2).
    """

    def __init__(self, ticks=20, k0=0.5, a=1.0, corner_deg=30.0, snap_scale=0.1, bit=1, grid=21):
        if int(ticks) != ticks or ticks < 1:
            raise ValueError("ticks must be an integer >= 1, got %r" % (ticks,))
        if int(grid) != grid or grid < 2:
            raise ValueError("grid must be an integer >= 2, got %r" % (grid,))
        if not (math.isfinite(float(snap_scale)) and snap_scale >= 0.0):
            raise ValueError("snap_scale must be a finite number >= 0, got %r" % (snap_scale,))
        if bit not in (1, -1):
            raise ValueError("bit must be +1 or -1, got %r" % (bit,))
        if not (0.0 < float(corner_deg) < 180.0):
            raise ValueError("corner_deg must be in (0, 180), got %r" % (corner_deg,))
        self.flow = SpectreHatFlow(0.0, k0=k0, a=a, corner_deg=corner_deg)
        self.N = int(ticks)
        self.snap_scale = float(snap_scale)
        self.grid = int(grid)
        self.now = 0
        self._record = [int(bit)]
        self.log = []
        self.data = {}

    # -- parameters and the laws --------------------------------------------
    @property
    def a(self):
        return self.flow.a

    @property
    def omega0(self):
        """omega_0 in radians."""
        return math.radians(self.flow.corner_deg)

    def omega(self, tau):
        return math.radians(self.flow.corner_of(tau))

    def jump(self, tau):
        """d(tau) = 2 h(tau): distance between P_8 for s = +1 and s = -1 (D5)."""
        return 2.0 * self.flow.opening_of(tau)

    def readable(self, tau):
        """d(tau) / d(0): 1 at the Spectre, 0 at the Hat."""
        return self.jump(tau) / self.jump(0.0)

    def readability_rate(self, tau):
        """d'(tau) = -a omega_0 / cos^2(omega/2): negative everywhere (D6)."""
        return -self.a * self.omega0 / math.cos(self.omega(tau) / 2.0) ** 2

    def tau_of(self, tick):
        return tick / self.N

    @property
    def tau(self):
        return self.tau_of(self.now)

    # -- the record -----------------------------------------------------------
    @property
    def record(self):
        """Tuple of the bits of ticks 0..now."""
        return tuple(self._record)

    @property
    def bit(self):
        return self._record[self.now]

    def tick_of(self, tau):
        """Index of the tick whose bit holds at tau (clamped to 0..N)."""
        return min(self.N, int(math.floor(tau * self.N + 1e-12)))

    def bit_at(self, tau):
        """Bit that holds at tau: recorded up to ``now``, the present bit projected beyond."""
        return self._record[min(self.tick_of(tau), self.now)]

    def read_bit(self, tau, eps=1e-9):
        """bit_at(tau) if it can be read there (jump > eps), else None."""
        return self.bit_at(tau) if self.jump(tau) > eps else None

    # -- tiles and arrows -------------------------------------------------------
    def tile(self, tau, s):
        """The tile at tau with bit s (D2): the flow tile, P_8 moved by -2h if s = -1."""
        if s not in (1, -1):
            raise ValueError("s must be +1 or -1, got %r" % (s,))
        tile = self.flow.tile_at(tau)
        if s < 0:
            v = tile.vertices[FLAT_VERTEX]
            v.position = v.position + np.array([-self.jump(tau), 0.0])
        return tile

    def outline_at(self, tau):
        """The tile at tau with the bit that holds there."""
        return self.tile(tau, self.bit_at(tau))

    def arrows(self, tau, s=None):
        """The two arrows at P_8 (D4), measured on the Bezier outline.  s defaults to the
        bit that holds at tau."""
        s = self.bit_at(tau) if s is None else s
        v = self.tile(tau, s).vertices[FLAT_VERTEX]
        t_in, t_out = v.position - v.in_handle.position, v.out_handle.position - v.position
        return Arrows(t_in / np.hypot(*t_in), t_out / np.hypot(*t_out))

    def backward_epoch(self):
        """tau below which the arrows have a backward component (omega > pi/2), or None
        when omega_0 <= pi/2."""
        return None if self.omega0 <= math.pi / 2 else 1.0 - (math.pi / 2) / self.omega0

    # -- the smooth arrow ---------------------------------------------------------
    def step(self, n=1):
        """Let smooth time run n ticks forward.  New ticks copy the last bit (D3).  A negative
        n raises ArrowError (D6); running past tick N raises ValueError."""
        if int(n) != n:
            raise ValueError("n must be an integer, got %r" % (n,))
        if n < 0:
            raise ArrowError("the smooth flow only runs forward: step(%d)" % n)
        if self.now + n > self.N:
            raise ValueError("cannot step past tick %d (tau = 1): now=%d, n=%d" % (self.N, self.now, n))
        for _ in range(int(n)):
            self._record.append(self._record[-1])
            self.now += 1

    # -- snaps ---------------------------------------------------------------------
    def snap_window(self):
        """Ticks whose bit may snap, by the cost rule R2 alone: a tuple (j_min, N)
        (j_min = N + 1 when no tick qualifies, i.e. the window is empty).  Closed form of
        the start: tau* = 1 - 2 atan(snap_scale / (2a)) / omega_0, clamped to [0, 1]."""
        ok = [j for j in range(self.N + 1) if self.jump(self.tau_of(j)) <= self.snap_scale + 1e-12]
        return (min(ok) if ok else self.N + 1, self.N)

    def tau_star(self):
        """tau* of R2 (closed form, clamped to [0, 1])."""
        return min(1.0, max(0.0, 1.0 - 2.0 * math.atan(self.snap_scale / (2.0 * self.a)) / self.omega0))

    def frozen_ticks(self):
        """Ticks (up to now) that R2 freezes: set in stone."""
        j_min = self.snap_window()[0]
        return list(range(0, min(j_min, self.now + 1)))

    def _range(self, j):
        """tau range of smooth flow a snap of tick j affects (for R3)."""
        lo = self.tau_of(j)
        hi = 1.0 if j == self.now else self.tau_of(j + 1)
        return np.linspace(lo, hi, self.grid)

    def can_snap(self, j=None):
        """(allowed, reasons) for a snap of tick j (None = the present tick); nothing changes."""
        j = self.now if j is None else j
        if int(j) != j:
            raise ValueError("j must be an integer, got %r" % (j,))
        j = int(j)
        if not 0 <= j <= self.now:
            return False, ["record: tick %d has no record yet (now = %d)" % (j, self.now)]
        reasons = []
        tj = self.tau_of(j)
        if self.jump(tj) > self.snap_scale + 1e-12:
            reasons.append("cost: jump %.4g at tau=%.4g exceeds snap_scale %.4g (tick %d is frozen)"
                           % (self.jump(tj), tj, self.snap_scale, j))
        old, new = self._record[j], -self._record[j]
        bad_valid, bad_smooth = [], []
        for tv in self._range(j):
            if not is_simple_tile(self.tile(tv, new)):
                bad_valid.append(float(tv))
            if not np.array_equal(_smooth_part(self.tile(tv, old)), _smooth_part(self.tile(tv, new))):
                bad_smooth.append(float(tv))
        if bad_valid:
            reasons.append("valid: the outline with bit %+d self-intersects at tau=%.4g (%d of %d samples)"
                           % (new, bad_valid[0], len(bad_valid), self.grid))
        if bad_smooth:
            reasons.append("smooth: edges other than 7 and 8 would change at tau=%.4g" % bad_smooth[0])
        # R5 (a rewrite leaves the tile at now alone) holds by construction: the tile at now
        # uses the bit of tick now, which a rewrite of tick j < now does not touch; snap()
        # asserts it on the real state change.
        return (not reasons), reasons

    def snap(self, j=None):
        """Snap tick j (None = present).  Returns the logged event or raises SnapRefused."""
        j = self.now if j is None else int(j)
        ok, reasons = self.can_snap(j)
        if not ok:
            raise SnapRefused(reasons)
        before_present = self.tile(self.tau, self.bit).control_points()
        old = self._record[j]
        self._record[j] = -old
        if j < self.now:  # R5, checked on the real state change
            assert np.array_equal(before_present, self.tile(self.tau, self.bit).control_points())
        event = {
            "kind": "flip_now" if j == self.now else "snap_back",
            "tick": j,
            "tau": self.tau_of(j),
            "now": self.tau,
            "depth": (self.now - j) / self.N,
            "jump": self.jump(self.tau_of(j)),
            "old": old,
            "new": -old,
        }
        self.log.append(event)
        return event

    # -- summaries -------------------------------------------------------------------
    def history(self):
        """List of dicts per recorded tick: tau, bit, jump, readable, frozen."""
        j_min = self.snap_window()[0]
        return [
            {"tick": j, "tau": self.tau_of(j), "bit": self._record[j], "jump": self.jump(self.tau_of(j)),
             "readable": self.readable(self.tau_of(j)), "frozen": j < j_min}
            for j in range(self.now + 1)
        ]

    def __repr__(self):
        return "<SpectreHatTime tick %d/%d tau=%.4g bit=%+d omega_0=%g deg snap_scale=%g, %d snaps>" % (
            self.now, self.N, self.tau, self.bit, self.flow.corner_deg, self.snap_scale, len(self.log))


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectre_hat_time_model.py --tests-only)
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def _changed_edges(t1, t2):
    c1, c2 = t1.control_points(), t2.control_points()
    return [i for i in range(N_EDGES) if not np.allclose(c1[i], c2[i], atol=1e-13)]


def test_e1_plus_tile_is_the_flow_tile_and_minus_differs_in_edges_7_and_8_only():
    m = SpectreHatTime(corner_deg=40.0, k0=0.6)
    for tv in (0.0, 0.3, 0.7, 1.0):
        plus, minus = m.tile(tv, +1), m.tile(tv, -1)
        assert np.array_equal(plus.control_points(), m.flow.tile_at(tv).control_points())
        assert _changed_edges(plus, minus) == ([7, 8] if tv < 1.0 else []), tv
    assert math.isclose(np.hypot(*(m.tile(0.0, 1).vertices[8].position - m.tile(0.0, -1).vertices[8].position)),
                        m.jump(0.0), rel_tol=1e-12)


def test_e2_measured_arrows_turn_by_minus_s_omega_and_align_at_the_hat():
    for corner, k0 in ((30.0, 0.5), (100.0, 0.8), (150.0, -0.4)):
        m = SpectreHatTime(corner_deg=corner, k0=k0)
        for tv in (0.0, 0.25, 0.6, 0.95):
            for s in (1, -1):
                arr = m.arrows(tv, s)
                assert math.isclose(arr.turn, -s * m.omega(tv), abs_tol=1e-9), (corner, tv, s)
                assert math.isclose(np.hypot(*arr.a_in), 1.0) and math.isclose(np.hypot(*arr.a_out), 1.0)
                assert math.isclose(arr.alignment, math.cos(m.omega(tv)), abs_tol=1e-9)
                assert (arr.backward > 1e-12) == (m.omega(tv) > math.pi / 2 + 1e-12)
        hat = m.arrows(1.0, 1)
        assert math.isclose(hat.alignment, 1.0, abs_tol=1e-12) and hat.backward == 0.0
    assert SpectreHatTime(corner_deg=30.0).backward_epoch() is None
    m = SpectreHatTime(corner_deg=120.0)
    assert math.isclose(m.backward_epoch(), 0.25)
    assert m.arrows(0.2, 1).backward > 0 and m.arrows(0.3, 1).backward == 0.0


def test_e3_readability_decreases_strictly_and_the_smooth_flow_only_runs_forward():
    m = SpectreHatTime(corner_deg=60.0)
    taus = np.linspace(0.0, 1.0, 101)
    d = np.array([m.jump(t) for t in taus])
    assert np.all(np.diff(d) < 0) and d[-1] == 0.0 and math.isclose(m.readable(0.0), 1.0)
    for t in (0.0, 0.5, 1.0):  # analytic rate equals the numerical derivative and is negative
        e = 1e-6
        lo, hi = max(0.0, t - e), min(1.0, t + e)
        assert m.readability_rate(t) < 0
        assert math.isclose(m.readability_rate(t), (m.jump(hi) - m.jump(lo)) / (hi - lo), rel_tol=1e-4)
    assert _raises(ArrowError, m.step, -1)
    assert _raises(ValueError, m.step, m.N + 1) and _raises(ValueError, m.step, 1.5)
    m.step(5)
    assert m.now == 5 and m.record == (1,) * 6, "persistence"
    assert m.read_bit(0.2) == 1 and m.read_bit(1.0) is None, "unreadable at the Hat"


def test_e4_smooth_sector_and_final_hat_do_not_depend_on_the_bit_history():
    grid = np.linspace(0.0, 1.0, 11)
    plain = SpectreHatTime(ticks=10, corner_deg=30.0, snap_scale=10.0)
    plain.step(10)
    fancy = SpectreHatTime(ticks=10, corner_deg=30.0, snap_scale=10.0)
    fancy.step(4)
    fancy.snap()
    fancy.step(3)
    fancy.snap(2)
    fancy.step(3)
    fancy.snap()
    assert plain.record != fancy.record
    for tv in grid:
        assert np.array_equal(_smooth_part(plain.outline_at(tv)), _smooth_part(fancy.outline_at(tv)))
    for m in (plain, fancy):
        for s in (1, -1):
            assert np.array_equal(m.tile(1.0, s).control_points(), plain.tile(1.0, 1).control_points())
    assert not np.array_equal(plain.outline_at(0.45).control_points(), fancy.outline_at(0.45).control_points())


def test_e5_snap_window_and_frozen_ticks():
    m = SpectreHatTime(ticks=20, corner_deg=30.0, snap_scale=0.1)
    ts = m.tau_star()
    assert math.isclose(m.jump(ts), 0.1, rel_tol=1e-12)
    j_min = m.snap_window()[0]
    assert m.jump(m.tau_of(j_min)) <= 0.1 + 1e-12 < m.jump(m.tau_of(j_min - 1))
    assert j_min == math.ceil(ts * 20 - 1e-9) == 17
    m.step(10)
    assert m.frozen_ticks() == list(range(0, 11))
    ok, why = m.can_snap(3)
    assert not ok and why[0].startswith("cost") and _raises(SnapRefused, m.snap, 3)
    assert m.record == (1,) * 11 and m.log == [], "a refused snap changes nothing"
    ok, why = m.can_snap(m.now + 1)
    assert not ok and why[0].startswith("record")
    m.step(8)  # now = 18
    assert m.frozen_ticks() == list(range(0, 17))
    assert m.can_snap(16)[0] is False and m.can_snap(17) == (True, []) and m.can_snap() == (True, [])
    assert SpectreHatTime(snap_scale=0.0).snap_window()[0] == 20, "only the Hat itself is free"
    assert SpectreHatTime(snap_scale=100.0).snap_window()[0] == 0


def test_e6_present_flip_changes_the_future_only_at_edges_7_8_and_a_rewrite_leaves_it_alone():
    m = SpectreHatTime(ticks=20, corner_deg=30.0, snap_scale=1.0)
    m.step(18)
    before = {tv: m.outline_at(tv) for tv in (0.5, 0.85, 0.9, 1.0)}
    ev = m.snap(17)  # backward snap by one tick
    assert ev["kind"] == "snap_back" and math.isclose(ev["depth"], 1 / 20) and m.record[17] == -1
    for tv in (0.9, 1.0):  # present and projected future unchanged
        assert np.array_equal(before[tv].control_points(), m.outline_at(tv).control_points()), tv
    assert _changed_edges(before[0.85], m.outline_at(0.85)) == [7, 8], "the rewritten interval changed"
    assert np.array_equal(before[0.5].control_points(), m.outline_at(0.5).control_points())
    ev = m.snap()  # forward snap
    assert ev["kind"] == "flip_now" and ev["depth"] == 0.0 and m.bit == -1
    assert _changed_edges(before[0.9], m.outline_at(0.9)) == [7, 8]
    assert np.array_equal(before[1.0].control_points(), m.outline_at(1.0).control_points()), "the Hat is the same"
    assert [e["kind"] for e in m.log] == ["snap_back", "flip_now"]
    h = m.history()
    assert len(h) == 19 and h[17]["bit"] == -1 and h[18]["bit"] == -1 and not any(r["frozen"] for r in h)
    m2 = SpectreHatTime(ticks=20, corner_deg=30.0, snap_scale=0.1)
    m2.step(18)
    assert [r["frozen"] for r in m2.history()] == [True] * 17 + [False] * 2


def test_e7_validity_is_a_real_constraint_for_large_corner_angles():
    # omega_0 = 150 degrees: the reflex tile (s = -1) crosses itself early in the flow
    m = SpectreHatTime(ticks=20, corner_deg=150.0, k0=0.5, snap_scale=100.0, grid=21)
    assert is_simple_tile(m.tile(0.0, +1)) and not is_simple_tile(m.tile(0.0, -1))
    bad = [t for t in np.linspace(0.0, 1.0, 201) if not is_simple_tile(m.tile(t, -1))]
    assert bad and max(bad) < 0.5 and is_simple_tile(m.tile(1.0, -1))
    m.step(2)
    ok, why = m.can_snap()
    assert not ok and len(why) == 1 and why[0].startswith("valid"), why
    assert _raises(SnapRefused, m.snap) and m.bit == 1
    m.step(18)  # now at the Hat: both bits give the same simple tile
    assert m.can_snap() == (True, [])
    # with the cost rule on as well, both reasons are reported
    m2 = SpectreHatTime(ticks=20, corner_deg=150.0, k0=0.5, snap_scale=0.1)
    ok, why = m2.can_snap()
    assert not ok and [w.split(":")[0] for w in why] == ["cost", "valid"]
    # at 30 degrees every outline is simple: R3 never fires
    ok30 = SpectreHatTime(corner_deg=30.0)
    assert all(is_simple_tile(ok30.tile(t, s)) for t in np.linspace(0, 1, 41) for s in (1, -1))


def test_free_flips_at_the_hat_and_bit_at():
    m = SpectreHatTime(ticks=5, corner_deg=30.0, snap_scale=0.0)
    m.step(5)
    assert m.tau == 1.0 and m.jump(1.0) == 0.0
    ev = m.snap()  # at the Hat a flip costs nothing and changes nothing
    assert ev["jump"] == 0.0 and m.bit == -1 and m.can_snap(4)[0] is False
    assert m.bit_at(1.0) == -1 and m.bit_at(0.0) == 1 and m.bit_at(0.39) == 1
    assert np.array_equal(m.outline_at(1.0).control_points(), m.tile(1.0, 1).control_points())


def test_validation():
    for kw in ({"ticks": 0}, {"ticks": 2.5}, {"grid": 1}, {"snap_scale": -1.0}, {"snap_scale": float("nan")},
               {"bit": 0}, {"corner_deg": 0.0}, {"corner_deg": 180.0}):
        assert _raises(ValueError, SpectreHatTime, **kw), kw
    m = SpectreHatTime()
    assert _raises(ValueError, m.tile, 0.5, 0) and _raises(ValueError, m.can_snap, 1.5)
    assert _raises(ValueError, m.tile, 1.5, 1), "tau outside [0, 1]"


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
            print("  %-82s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Command line: a scripted story and a figure
# --------------------------------------------------------------------------
def story(m, out=print):
    """A scripted run: frozen early epoch, then forward and backward snaps in the window."""
    out(repr(m))
    j_min = m.snap_window()[0]
    out("snap window: ticks %d..%d (tau* = %.4f); ticks before %d are set in stone" % (j_min, m.N, m.tau_star(), j_min))
    steps = [("step", max(1, j_min // 2)), ("snap", None), ("step", j_min - max(1, j_min // 2) + 1),
             ("snap", j_min - 1), ("snap", j_min), ("snap", None), ("step", m.N - j_min - 1), ("snap", None)]
    for what, arg in steps:
        if what == "step":
            m.step(arg)
            out("step %d -> tick %d (tau=%.3f), bit %+d, readable %.3f" % (arg, m.now, m.tau, m.bit, m.readable(m.tau)))
        else:
            j = m.now if arg is None else arg
            ok, why = m.can_snap(j)
            if ok:
                ev = m.snap(j)
                out("snap tick %d (%s): %+d -> %+d, depth %.3f, jump %.4f" % (j, ev["kind"], ev["old"], ev["new"],
                                                                         ev["depth"], ev["jump"]))
            else:
                out("snap tick %d refused: %s" % (j, "; ".join(why)))
    out("record: %s" % "".join("+" if s > 0 else "-" for s in m.record))


def plot(m, axes_tiles, ax_curve):
    """Both branches over each other (blue s=+1, red s=-1) and the curves d, omega, window."""
    taus = np.linspace(0.0, 1.0, len(axes_tiles))
    for ax, tv in zip(axes_tiles, taus):
        for s, col in ((1, "tab:blue"), (-1, "tab:red")):
            x, y = m.tile(tv, s).outline(60)
            ax.plot(x, y, color=col, lw=1.1, label="bit %+d" % s if tv == 0.0 else None)
        p = m.tile(tv, 1).vertices[FLAT_VERTEX].position
        ax.plot(p[0], p[1], "o", color="k", ms=3)
        ax.set_aspect("equal")
        ax.axis("off")
        if tv == 0.0:
            ax.legend(fontsize=7, loc="lower left", frameon=False)
        ax.set_title("tau %.2f  jump %.2f\nomega %.0f deg" % (tv, m.jump(tv), math.degrees(m.omega(tv))), fontsize=8)
    t = np.linspace(0.0, 1.0, 201)
    ax_curve.plot(t, [m.readable(x) for x in t], color="k", label="readable d(tau)/d(0)")
    ax_curve.plot(t, [m.omega(x) / math.pi for x in t], color="tab:green", label="omega / pi (reversal)")
    ax_curve.axhline(0.5, color="tab:green", lw=0.5, ls=":")
    ts = m.tau_star()
    ax_curve.axvspan(0.0, ts, color="0.85", label="set in stone (R2)")
    ax_curve.axvspan(ts, 1.0, color="tab:orange", alpha=0.2, label="snap window")
    ep = m.backward_epoch()
    if ep is not None:
        ax_curve.axvspan(0.0, ep, ymin=0.0, ymax=0.06, color="tab:green", alpha=0.6, label="arrows run backward")
    for e in m.log:
        ax_curve.plot([e["tau"]], [m.readable(e["tau"])], "v" if e["kind"] == "snap_back" else "o",
                      color="tab:red", ms=6)
    ax_curve.set_xlabel("tau")
    ax_curve.set_xlim(0, 1)
    ax_curve.legend(fontsize=7, loc="upper right")


def main(argv=None):
    p = argparse.ArgumentParser(
        description="Time, two arrows and one bit at the 14th vertex of the Spectre-Hat flow (toy model)."
    )
    p.add_argument("--ticks", type=int, default=20, help="number of ticks N (default 20)")
    p.add_argument("--corner", type=float, default=30.0, metavar="DEG", help="omega_0 in degrees (default 30)")
    p.add_argument("--k0", type=float, default=0.5, help="curve strength at tau = 0 (default 0.5)")
    p.add_argument("--snap-scale", type=float, default=0.1, help="largest jump a snap may cause (default 0.1)")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--story", action="store_true", help="print a scripted run with forward and backward snaps")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="save the figure (runs the story first) to this file")
    args = p.parse_args(argv)

    if args.tests_only:
        print("spectre_hat_time_model self tests:")
        return 0 if run_tests() else 1

    try:
        m = SpectreHatTime(ticks=args.ticks, k0=args.k0, corner_deg=args.corner, snap_scale=args.snap_scale)
    except ValueError as exc:
        p.error(str(exc))
    if args.story or args.save:
        story(m)
    else:
        print(m)
        print("use --story, --save PNG or --tests-only")
        return 0
    if args.save:
        import matplotlib

        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(13, 6.2))
        gs = fig.add_gridspec(2, 5, height_ratios=[1.3, 1.0])
        axes = [fig.add_subplot(gs[0, i]) for i in range(5)]
        ax = fig.add_subplot(gs[1, :])
        plot(m, axes, ax)
        fig.tight_layout()
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
        if not args.no_show:
            plt.show()
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
