"""
spectre_hat_backward_snap_model.py - the quantum version of the backward snap.

In spectre_hat_time_model.py a *backward snap* rewrites the record of an earlier tick.  Here the
bit at P_8 is the quantum bit of spectre_hat_quantum_bit_model.py and a "record" is replaced by a
*history*: a sum over the ticks at which the bit tunnelled.  A backward snap is then a history in
which the tunnelling happened earlier than in a reference history, and the question becomes
how much amplitude such a history has, how far back it can reach, and what it costs to find.

Status labels (as in QUANTUM_GRAVITY.md): **Exact** = checked by the self tests of this file
(python3 spectre_hat_backward_snap_model.py --tests-only); **Definition** = a choice made here;
**Hypothesis** = a physical reading nothing here tests.

    from spectre_hat_backward_snap_model import BackwardSnap

    m = BackwardSnap(ticks=12)
    m.final_amplitudes()                # (cos(Theta/2), i sin(Theta/2)) as a sum over 4096 histories
    m.flip_time_distribution()          # when did the flip happen, given that exactly one did
    ref = m.reference(10)               # history: the bit tunnelled once, at tick 10
    m.backward_snap(ref, 8, delta=0.15) # the best compatible history whose flip is at tick 8
    m.reach(ref, delta=0.15)            # earliest tick a compatible history can start at

Definitions
-----------
B1  Histories.  The register of the quantum model tunnels at tick j by U(theta_j) = cos(theta_j/2)
    + i sin(theta_j/2) X, theta_j = Omega c(tau_j)/N.  Expanding the product over ticks
    0..n-1 gives a sum over toggle patterns t in {0,1}^n: tick j *stays* (amplitude cos) or
    *toggles* (amplitude i sin).  A history is one pattern; its bits b_0 = +, b_(j+1) = b_j
    (-1)^(t_j) are the bit entering each tick.  Its amplitude is the product of the factors.
B2  Present compatibility.  The smooth flow continues from the *present* bit, so a history is
    compatible with the future only if it ends in the same final bit as the reference history.
B3  Holonomy compatibility (optional, ``delta``).  The smooth surface has integrated the area the
    bit added, Q(history) = sum_j b_j w_j with w_j = h(tau_j) (the area of the tile is
    A0 + b a h, so b_j w_j is the bit's share).  It cannot forget it: a compatible history has
    |Q - Q_reference| <= delta.  ``delta = None`` switches B3 off.  This is a design choice, not
    derived; delta plays the role of how fuzzy the surface's memory is.
B4  The reference history is a single toggle at a late tick k (``reference(k)``): the bit
    tunnelled once, at the tick where tunnelling is cheapest to explain.
B5  A *backward snap to tick j* is a compatible history whose first toggle is at tick j < k.
    Of those, the one with the largest amplitude is reported, with its probability relative to
    the reference.  Finding it takes a search over the patterns (``work``); following the
    smooth flow forward takes n steps.

What is exact (tests)
---------------------
E1  The sum over all histories with an even number of toggles is cos(Theta/2), with an odd number
    i sin(Theta/2), Theta = sum theta_j (the product of the U's, as in the quantum model).
    Equivalently cos(Theta/2) = prod cos(theta_j/2) * sum over even m of (-1)^(m/2) e_m(t),
    t_j = tan(theta_j/2) and e_m the elementary symmetric polynomial.
E2  The order of the snaps is not observable: any permutation or redistribution of the angles with
    the same sum gives the same final amplitudes (the U's commute).  The present state knows
    Theta, not the tick at which the tunnelling happened.
E3  Given exactly one toggle, it happened at tick j with probability proportional to
    tan^2(theta_j/2), which is close to (c_j)^2: the "when" of the flip is exponentially late.
    Going back m ticks costs the ratio tan^2(theta_j/2) / tan^2(theta_k/2).
E4  A bubble (toggles at j < k, none elsewhere) has the amplitude -tan(theta_j/2) tan(theta_k/2)
    relative to the history without toggles: second order, with the sign that interferes
    destructively with it.  Quantum flip probability sin^2(Theta/2) exceeds the classical
    Markov value (1 - prod cos(theta_j))/2 for small angles: the difference is the cross terms
    between different ticks.
E5  With B3, the set of compatible histories grows with delta, the earliest reachable tick moves
    back, and the best probability of a backward snap falls steeply with depth (checked for
    strict decrease).  The cheapest backward snap of a single flip by m ticks changes Q by
    -2 sum w over the m ticks, which delta has to cover or compensate with further toggles.
E6  Work: the forward path takes n steps, the search over compatible histories examines 2^n
    patterns (exhaustive).
E7  Reversing the smooth flow (``reversed_flow``) leaves the final amplitudes unchanged (E2) and
    turns the late-peaked distribution of the flip time into an early-peaked one: the asymmetry
    between cheap and expensive ticks belongs to the orientation of the smooth flow, not to the
    bits.

Physics reading (all **Hypothesis**; none of it is tested here)
---------------------------------------------------------------
* A backward snap as a history in which the discrete event happened earlier, compatible with the
  continuation: a toy of retrodiction with a future boundary condition (two-state-vector and
  consistent-histories language is borrowed, nothing of those theories is derived).
* A bubble (two toggles) as a zig-zag in the bit's history, in the spirit of Stueckelberg-Feynman
  worldlines that run backward between two vertices.  Here it is a bit history, not a worldline.
* The smooth surface as the thing that knows what is compatible: the constraint B3 is a function
  of the surface's accumulated area.  That the surface does carry such a memory is an assumption.
* The cost asymmetry: forward = n steps and amplitude 1, backward = a search and an amplitude
  that falls with depth.  The search is exponential only for generic weights (a subset-sum
  problem); the weights here are structured and cheaper algorithms exist.  Whether a physical
  constraint has such a hard structure is not addressed.
What does not carry over: the forward direction is put in by the flow law (see ARROW_OF_TIME.md);
there is one bit; the angles theta_j are a rule, not a Hamiltonian; nothing here is a field.

Needs NumPy and spectre_hat_quantum_bit_model.py (and what it needs); matplotlib is imported
lazily by the command line.
"""

import argparse
import math
import sys

import numpy as np

from spectre_hat_quantum_bit_model import SpectreHatQubit

__all__ = ["BackwardSnap", "run_tests"]

MAX_TICKS = 20  # 2^20 histories


def _first_toggle(patterns, n):
    """Index of the lowest set bit of each pattern (n if the pattern is 0)."""
    low = patterns & -patterns
    out = np.full(len(patterns), n, dtype=int)
    nz = low > 0
    out[nz] = np.log2(low[nz]).astype(int)
    return out


class BackwardSnap:
    """All histories of the quantum bit up to tick n and the backward snaps between them.

    Parameters
    ----------
    ticks, k0, a, corner_deg, snap_scale, sigma, tunnel_rate : as in SpectreHatQubit.
    n : number of ticks to sum over (default: all ``ticks``, i.e. up to the Hat); at most 20.
    """

    def __init__(self, ticks=12, k0=0.5, a=1.0, corner_deg=30.0, snap_scale=0.1, sigma=None,
                 tunnel_rate=math.pi, n=None):
        self.qubit = SpectreHatQubit(ticks=ticks, k0=k0, a=a, corner_deg=corner_deg, snap_scale=snap_scale,
                                     sigma=sigma, tunnel_rate=tunnel_rate)
        n = self.qubit.N if n is None else n
        if int(n) != n or not 1 <= n <= min(MAX_TICKS, self.qubit.N):
            raise ValueError("n must be an integer in 1..min(%d, ticks), got %r" % (MAX_TICKS, n))
        self.n = int(n)
        self._angles = np.array([self.qubit.snap_angle(j) for j in range(self.n)])
        self._weights = np.array([self.qubit.jump(self.qubit.time.tau_of(j)) / 2.0 for j in range(self.n + 1)])
        self._paths = None

    # -- the ingredients ------------------------------------------------------------------
    def angles(self):
        """theta_0 .. theta_(n-1) of B1."""
        return self._angles.copy()

    def weights(self):
        """w_0 .. w_n = h(tau_j), the bit's share of the surface area at each tick (B3)."""
        return self._weights.copy()

    def tan_half(self):
        return np.tan(self._angles / 2.0)

    @staticmethod
    def build_paths(angles, weights=None):
        """Enumerate all 2^n histories: dict of arrays amp, toggles, parity (final bit: 0 = same,
        1 = flipped), first (tick of the first toggle, n if none) and Q (if weights given).
        Pattern p has toggle t_j = bit j of p."""
        n = len(angles)
        amp = np.ones(1, dtype=complex)
        for th in angles:
            amp = np.concatenate([amp * math.cos(th / 2.0), amp * 1j * math.sin(th / 2.0)])
        p = np.arange(len(amp))
        cum = np.zeros(len(amp), dtype=int)
        out = {"pattern": p, "amp": amp, "first": _first_toggle(p, n)}
        if weights is not None:
            q = np.full(len(amp), float(weights[0]))
        for j in range(n):
            cum ^= (p >> j) & 1
            if weights is not None:
                q += weights[j + 1] * (1 - 2 * cum)
        out["parity"] = cum
        out["toggles"] = np.array([bin(x).count("1") for x in p]) if n <= 16 else _popcount(p)
        if weights is not None:
            out["Q"] = q
        return out

    @property
    def paths(self):
        if self._paths is None:
            self._paths = self.build_paths(self._angles, self._weights)
        return self._paths

    # -- E1, E2: the sum over histories ------------------------------------------------------
    def final_amplitudes(self):
        """(sum over even-toggle histories, sum over odd-toggle histories)."""
        p = self.paths
        return complex(p["amp"][p["parity"] == 0].sum()), complex(p["amp"][p["parity"] == 1].sum())

    def total_angle(self):
        return float(self._angles.sum())

    def flip_probability_quantum(self):
        """sin^2(Theta/2)."""
        return math.sin(self.total_angle() / 2.0) ** 2

    def flip_probability_markov(self):
        """Probability of an odd number of toggles if every tick toggled independently with
        probability sin^2(theta_j/2): (1 - prod cos theta_j)/2."""
        return 0.5 * (1.0 - float(np.prod(np.cos(self._angles))))

    # -- E3, E4: when did it flip, and bubbles -------------------------------------------------------
    def flip_time_distribution(self):
        """P(the single toggle was at tick j | exactly one toggle), proportional to tan^2(theta_j/2)."""
        w = self.tan_half() ** 2
        return w / w.sum()

    def depth_weight(self, j, k):
        """tan^2(theta_j/2) / tan^2(theta_k/2): probability of the flip at tick j relative to tick k."""
        t = self.tan_half()
        return float((t[j] / t[k]) ** 2)

    def bubble_ratio(self, j, k):
        """Amplitude of toggles at j and k (and nowhere else) relative to no toggles: -tan tan."""
        t = self.tan_half()
        return float(-t[j] * t[k])

    def pattern_of(self, ticks):
        """Pattern index with toggles at the given ticks."""
        p = 0
        for j in ticks:
            if not 0 <= j < self.n:
                raise ValueError("tick %r is outside 0..%d" % (j, self.n - 1))
            p |= 1 << int(j)
        return p

    def ticks_of(self, pattern):
        return [j for j in range(self.n) if (pattern >> j) & 1]

    def reference(self, k):
        """Pattern of B4: a single toggle at tick k."""
        return self.pattern_of([k])

    # -- B2, B3, B5: compatibility and the backward snap -------------------------------------------------
    def compatible(self, ref, delta=None):
        """Boolean mask over all patterns: same final bit as ``ref`` (B2) and, if delta is not
        None, |Q - Q_ref| <= delta (B3)."""
        p = self.paths
        mask = p["parity"] == p["parity"][ref]
        if delta is not None:
            if not (math.isfinite(float(delta)) and delta >= 0.0):
                raise ValueError("delta must be a finite number >= 0 or None, got %r" % (delta,))
            mask &= np.abs(p["Q"] - p["Q"][ref]) <= delta
        return mask

    def reach(self, ref, delta=None):
        """Earliest tick at which a compatible history has its first toggle."""
        mask = self.compatible(ref, delta)
        return int(self.paths["first"][mask].min())

    def backward_snap(self, ref, j, delta=None):
        """The compatible history with first toggle at tick j and the largest amplitude (B5):
        dict with its toggle ticks, its probability relative to the reference, the number of
        compatible histories with that first toggle and the work of the search.  None if there is
        none."""
        if not 0 <= j < self.n:
            raise ValueError("tick %r is outside 0..%d" % (j, self.n - 1))
        p = self.paths
        mask = self.compatible(ref, delta) & (p["first"] == j)
        if not mask.any():
            return None
        idx = np.flatnonzero(mask)
        best = idx[np.argmax(np.abs(p["amp"][idx]))]
        ref_prob = abs(p["amp"][ref]) ** 2
        return {"toggles": self.ticks_of(int(best)), "relative_probability": float(abs(p["amp"][best]) ** 2 / ref_prob),
                "relative_amplitude": complex(p["amp"][best] / p["amp"][ref]), "candidates": int(mask.sum()),
                "work": self.work_backward(), "depth": self.ticks_of(ref)[0] - j if len(self.ticks_of(ref)) else 0}

    def table(self, ref, delta=None):
        """For each tick j before the reference's first toggle: backward_snap(ref, j, delta) or None."""
        k = self.ticks_of(ref)[0]
        return {j: self.backward_snap(ref, j, delta) for j in range(k)}

    def constrained_amplitudes(self, ref, delta=None):
        """Sum of the amplitudes of the compatible histories, by final bit: (same, flipped), and the
        probability that a history is compatible (sum of |amp|^2 over them)."""
        p = self.paths
        mask = self.compatible(ref, delta)
        # compatible histories share the final bit of ref, so one of the two sums is empty
        s = complex(p["amp"][mask].sum())
        same = p["parity"][ref] == 0
        return (s, 0j) if same else (0j, s), float((np.abs(p["amp"][mask]) ** 2).sum())

    # -- E7: the same bits on the reversed flow -------------------------------------------------------------
    def reversed_flow(self):
        """The same model with the smooth flow run the other way: the angles and the weights are
        taken in reverse order (the Hat comes first).  Everything else is shared."""
        r = BackwardSnap.__new__(BackwardSnap)
        r.qubit, r.n = self.qubit, self.n
        r._angles, r._weights, r._paths = self._angles[::-1].copy(), self._weights[::-1].copy(), None
        return r

    # -- E6: the cost of finding it --------------------------------------------------------------------------
    def work_forward(self):
        """Steps to follow the smooth flow forward through the n ticks."""
        return self.n

    def work_backward(self):
        """Patterns examined by the exhaustive search for compatible histories."""
        return 2 ** self.n

    def __repr__(self):
        return "<BackwardSnap n=%d ticks, Theta=%.5g rad, sigma=%g>" % (self.n, self.total_angle(), self.qubit.sigma)


def _popcount(p):
    out = np.zeros(len(p), dtype=int)
    q = p.copy()
    while q.any():
        out += q & 1
        q >>= 1
    return out


# --------------------------------------------------------------------------
# Self tests  (run with:  python3 spectre_hat_backward_snap_model.py --tests-only)
# --------------------------------------------------------------------------
def _raises(exc, fn, *args, **kwargs):
    try:
        fn(*args, **kwargs)
    except exc:
        return True
    return False


def _elementary(vals, m):
    e = np.zeros(m + 1)
    e[0] = 1.0
    for v in vals:
        for k in range(m, 0, -1):
            e[k] += v * e[k - 1]
    return e


def test_e1_sum_over_histories_is_the_product_of_the_tunnelling_unitaries():
    m = BackwardSnap(ticks=12)
    even, odd = m.final_amplitudes()
    th = m.total_angle()
    assert np.isclose(even, math.cos(th / 2), atol=1e-13) and np.isclose(odd, 1j * math.sin(th / 2), atol=1e-13)
    rng = np.random.default_rng(2)
    angles = rng.uniform(0.0, 1.5, size=10)
    p = BackwardSnap.build_paths(angles)
    assert np.isclose(p["amp"][p["parity"] == 0].sum(), math.cos(angles.sum() / 2), atol=1e-13)
    assert np.isclose(p["amp"][p["parity"] == 1].sum(), 1j * math.sin(angles.sum() / 2), atol=1e-13)
    assert math.isclose(float((np.abs(p["amp"]) ** 2).sum()), 1.0, abs_tol=1e-13), "histories are a probability distribution"
    # the elementary symmetric polynomial form
    t = np.tan(angles / 2)
    e = _elementary(t, len(t))
    pref = np.prod(np.cos(angles / 2))
    cos_form = pref * sum((-1) ** (k // 2) * e[k] for k in range(0, len(t) + 1, 2))
    sin_form = pref * sum((-1) ** ((k - 1) // 2) * e[k] for k in range(1, len(t) + 1, 2))
    assert math.isclose(cos_form, math.cos(angles.sum() / 2), abs_tol=1e-12)
    assert math.isclose(sin_form, math.sin(angles.sum() / 2), abs_tol=1e-12)
    assert np.array_equal(p["toggles"], _popcount(p["pattern"]))


def test_e2_the_order_and_distribution_of_snaps_is_not_observable():
    rng = np.random.default_rng(4)
    a = rng.uniform(0.0, 1.0, size=9)
    base = BackwardSnap.build_paths(a)
    for perm in (rng.permutation(9), np.arange(9)[::-1]):
        pp = BackwardSnap.build_paths(a[perm])
        assert np.isclose(pp["amp"][pp["parity"] == 0].sum(), base["amp"][base["parity"] == 0].sum(), atol=1e-13)
        assert np.isclose(pp["amp"][pp["parity"] == 1].sum(), base["amp"][base["parity"] == 1].sum(), atol=1e-13)
    b = a.copy()
    b[0] += 0.3  # move angle from the last tick to the first: same sum, same present
    b[-1] -= 0.3
    pb = BackwardSnap.build_paths(b)
    assert np.isclose(pb["amp"][pb["parity"] == 1].sum(), base["amp"][base["parity"] == 1].sum(), atol=1e-13)
    assert not np.allclose(pb["amp"], base["amp"]), "the histories themselves do differ"


def test_e3_when_did_it_flip():
    m = BackwardSnap(ticks=12)
    dist = m.flip_time_distribution()
    p = m.paths
    single = np.array([abs(p["amp"][m.reference(j)]) ** 2 for j in range(m.n)])
    assert np.allclose(dist, single / single.sum(), rtol=1e-9, atol=1e-15)
    assert math.isclose(dist.sum(), 1.0) and np.all(np.diff(dist) > 0), "later ticks are likelier"
    assert dist[-1] > 0.5 and dist[:6].sum() < 1e-4
    for j, k in ((9, 11), (8, 10), (10, 11)):
        c = m.qubit.overlap(j / 12) / m.qubit.overlap(k / 12)
        assert math.isclose(m.depth_weight(j, k), single[j] / single[k], rel_tol=1e-9)
        assert math.isclose(m.depth_weight(j, k), c**2, rel_tol=0.03), (j, k)


def test_e4_bubbles_interfere_destructively_and_beat_the_markov_value():
    m = BackwardSnap(ticks=12)
    p = m.paths
    zero = p["amp"][0]
    for j, k in ((9, 11), (8, 10), (10, 11)):
        pat = m.pattern_of([j, k])
        assert math.isclose((p["amp"][pat] / zero).real, m.bubble_ratio(j, k), rel_tol=1e-12) and abs((p["amp"][pat] / zero).imag) < 1e-15
        assert m.bubble_ratio(j, k) < 0
    assert m.flip_probability_quantum() > m.flip_probability_markov()
    one = BackwardSnap.build_paths(np.array([0.0, 0.0, 0.7, 0.0]))
    assert math.isclose(float((np.abs(one["amp"][one["parity"] == 1]) ** 2).sum()), math.sin(0.35) ** 2)
    th = np.array([0.0, 0.0, 0.7, 0.0])
    assert math.isclose(0.5 * (1 - np.prod(np.cos(th))), math.sin(0.35) ** 2), "one active tick: no cross terms"


def test_e5_compatibility_grows_with_delta_and_deeper_snaps_cost_more():
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    deltas = (0.03, 0.06, 0.1, 0.15, 0.25, 0.4)
    counts = [int(m.compatible(ref, d).sum()) for d in deltas]
    reaches = [m.reach(ref, d) for d in deltas]
    assert counts == sorted(counts) and counts[0] == 1 and counts[-1] > counts[0]
    assert reaches == sorted(reaches, reverse=True) and reaches[0] == 10 and reaches[-1] < 5
    assert m.compatible(ref, None).sum() == 2 ** (m.n - 1), "B3 off: only the final bit matters"
    for d in deltas:
        assert m.compatible(ref, d)[ref]
    tab = m.table(ref, 0.4)
    probs = [tab[j]["relative_probability"] for j in sorted(tab, reverse=True) if tab[j] is not None]
    assert len(probs) >= 6 and np.all(np.diff(probs) < 0) and probs[0] < 1.0, "deeper = less likely"
    assert m.table(ref, 0.03) == {j: None for j in range(10)}, "a tight surface memory freezes the past"
    # a single toggle moved back by m ticks changes Q by -2 sum of the weights it passes
    w = m.weights()
    p = m.paths
    for j in (9, 8, 7):  # bits j+1..10 are - instead of +
        pat = m.reference(j)
        assert math.isclose(p["Q"][pat] - p["Q"][ref], -2.0 * sum(w[i] for i in range(j + 1, 11)), rel_tol=1e-9)
    best = m.backward_snap(ref, 9, 0.1)
    assert best["toggles"] == [9] and math.isclose(best["relative_probability"], m.depth_weight(9, 10), rel_tol=1e-9)
    assert best["depth"] == 1 and best["relative_amplitude"].real > 0, "a moved flip interferes constructively"
    assert m.backward_snap(ref, 5, 0.1) is None


def test_constrained_amplitudes_and_post_selection():
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    (same, flip), acc = m.constrained_amplitudes(ref, None)
    even, odd = m.final_amplitudes()
    assert same == 0j and np.isclose(flip, odd, atol=1e-13) and 0.0 < acc <= 1.0
    (_, f_tight), acc_tight = m.constrained_amplitudes(ref, 0.03)
    assert np.isclose(f_tight, m.paths["amp"][ref]) and acc_tight < acc
    (_, f_mid), acc_mid = m.constrained_amplitudes(ref, 0.15)
    assert acc_tight < acc_mid < acc
    assert abs(f_mid) > abs(f_tight), "compatible moved flips add constructively to the present amplitude"
    (s0, _), _ = m.constrained_amplitudes(0, None)
    assert np.isclose(s0, even, atol=1e-13)


def test_e6_work_of_the_forward_path_and_of_the_search():
    for n in (6, 9, 12):
        m = BackwardSnap(ticks=n)
        assert m.work_forward() == n and m.work_backward() == 2**n
        assert len(m.paths["amp"]) == 2**n
    assert BackwardSnap(ticks=12).backward_snap(BackwardSnap(ticks=12).reference(10), 9, 0.1)["work"] == 4096


def test_frozen_early_ticks_and_validity_gate():
    m = BackwardSnap(ticks=12)
    th = m.angles()
    assert th[0] < 1e-6 and np.all(np.diff(th) > 0), "tunnelling is frozen early and grows"
    ref = m.reference(10)
    far = m.backward_snap(ref, 1, 10.0)  # huge tolerance: compatible, but the amplitude is negligible
    assert far["toggles"] == [1] and far["relative_probability"] < 1e-9
    assert m.backward_snap(ref, 0, 10.0)["relative_probability"] < 1e-11
    g = BackwardSnap(ticks=12, corner_deg=150.0, k0=0.5, sigma=0.5)
    assert g.angles()[0] == 0.0 and g.angles()[-1] > 0.0
    assert g.backward_snap(g.reference(10), 0, 100.0)["relative_probability"] == 0.0, "no tunnelling where a tile self-intersects"


def test_e7_reversing_the_flow_reverses_the_asymmetry_and_nothing_else():
    m = BackwardSnap(ticks=12)
    r = m.reversed_flow()
    assert np.allclose(r.angles(), m.angles()[::-1]) and np.allclose(r.weights(), m.weights()[::-1])
    for a, b in zip(m.final_amplitudes(), r.final_amplitudes()):
        assert np.isclose(a, b, atol=1e-13), "the present state does not know the orientation"
    d, dr = m.flip_time_distribution(), r.flip_time_distribution()
    assert np.allclose(dr, d[::-1]) and d[-1] > 0.5 and dr[0] > 0.5
    assert d[m.n // 2:].sum() > 0.99 and dr[: m.n // 2].sum() > 0.99
    assert m.flip_probability_quantum() == r.flip_probability_quantum()
    assert m.angles()[0] < m.angles()[-1] and r.angles()[0] > r.angles()[-1]


def test_validation():
    for kw in ({"n": 0}, {"n": 13}, {"n": 2.5}):
        assert _raises(ValueError, BackwardSnap, **dict(ticks=12, **kw)), kw
    assert _raises(ValueError, BackwardSnap, ticks=21)
    m = BackwardSnap(ticks=8)
    assert _raises(ValueError, m.compatible, 0, -1.0) and _raises(ValueError, m.compatible, 0, float("nan"))
    assert _raises(ValueError, m.pattern_of, [8]) and _raises(ValueError, m.backward_snap, m.reference(5), 8)


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
            print("  %-80s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Command line: a story and a figure
# --------------------------------------------------------------------------
def story(m, k=10, out=print):
    out(repr(m))
    even, odd = m.final_amplitudes()
    out("sum over %d histories: stay %.5f%+.5fj, flip %.5f%+.5fj  (cos/sin of Theta/2 = %.5f, %.5f)"
        % (m.work_backward(), even.real, even.imag, odd.real, odd.imag, math.cos(m.total_angle() / 2),
           math.sin(m.total_angle() / 2)))
    out("quantum flip probability %.5f, independent-toggle (Markov) value %.5f" % (m.flip_probability_quantum(),
                                                                                  m.flip_probability_markov()))
    dist = m.flip_time_distribution()
    out("given exactly one flip, when? " + "  ".join("t%d:%.4f" % (j, dist[j]) for j in range(m.n) if dist[j] >= 1e-4))
    ref = m.reference(k)
    out("reference: one flip at tick %d.  Backward snaps compatible with the surface (work %d vs %d forward):"
        % (k, m.work_backward(), m.work_forward()))
    out("%6s %6s %7s %12s %s" % ("delta", "reach", "compat", "", "relative probability of a flip first at tick j"))
    for delta in (0.03, 0.06, 0.1, 0.15, 0.25, 0.4):
        tab = m.table(ref, delta)
        cells = "  ".join("t%d:%.1e" % (j, tab[j]["relative_probability"]) for j in sorted(tab, reverse=True)
                          if tab[j] is not None)
        out("%6.2f %6d %7d %12s %s" % (delta, m.reach(ref, delta), int(m.compatible(ref, delta).sum()), "", cells))


def plot(m, k, axes):
    ax1, ax2, ax3 = axes
    dist = m.flip_time_distribution()
    ax1.bar(range(m.n), np.maximum(dist, 1e-30), color="tab:blue")
    ax1.set_yscale("log")
    ax1.set_ylim(1e-12, 2)
    ax1.set_xlabel("tick j of the flip")
    ax1.set_ylabel("P(flip at j | one flip)")
    ax1.set_title("when did it flip? (E3)", fontsize=9)
    ref = m.reference(k)
    for delta, col in ((0.1, "tab:green"), (0.25, "tab:orange"), (0.4, "tab:red")):
        tab = m.table(ref, delta)
        js = sorted(j for j in tab if tab[j] is not None)
        ax2.plot([k - j for j in js], [tab[j]["relative_probability"] for j in js], "o-", color=col, label="delta %.2f" % delta)
    ax2.set_yscale("log")
    ax2.set_xlabel("depth: ticks back from the reference flip (tick %d)" % k)
    ax2.set_ylabel("relative probability")
    ax2.set_title("backward snaps (B5, E5)", fontsize=9)
    ax2.legend(fontsize=7)
    ns = np.arange(4, 21)
    ax3.semilogy(ns, ns, label="forward: n steps")
    ax3.semilogy(ns, 2.0**ns, label="search: 2^n histories")
    ax3.set_xlabel("ticks n")
    ax3.set_title("work (E6)", fontsize=9)
    ax3.legend(fontsize=7)


def main(argv=None):
    p = argparse.ArgumentParser(description="Quantum backward snap: histories of the bit at the 14th vertex (toy model).")
    p.add_argument("--ticks", type=int, default=12, help="ticks N, at most 20 (default 12)")
    p.add_argument("--corner", type=float, default=30.0, metavar="DEG", help="omega_0 in degrees (default 30)")
    p.add_argument("--snap-scale", type=float, default=0.1, help="classical snap scale; sigma defaults to half of it")
    p.add_argument("--ref", type=int, default=None, help="tick of the reference flip (default ticks - 2)")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit")
    p.add_argument("--story", action="store_true", help="print the sum over histories and the backward snap table")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="save the figure to this file")
    args = p.parse_args(argv)
    if args.tests_only:
        print("spectre_hat_backward_snap_model self tests:")
        return 0 if run_tests() else 1
    try:
        m = BackwardSnap(ticks=args.ticks, corner_deg=args.corner, snap_scale=args.snap_scale)
    except ValueError as exc:
        p.error(str(exc))
    k = m.n - 2 if args.ref is None else args.ref
    if not 1 <= k < m.n:
        p.error("--ref must be in 1..%d" % (m.n - 1))
    if not (args.story or args.save):
        print(m)
        print("use --story, --save PNG or --tests-only")
        return 0
    if args.story:
        story(m, k)
    if args.save:
        import matplotlib

        if args.no_show:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 3, figsize=(13, 3.8))
        plot(m, k, axes)
        fig.tight_layout()
        fig.savefig(args.save, dpi=130)
        print("saved", args.save)
        if not args.no_show:
            plt.show()
        plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
