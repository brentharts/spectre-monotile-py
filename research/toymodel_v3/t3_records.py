#!/usr/bin/env python3
"""t3_records.py - Stage 3 of research/toymodel_v3/PLAN.md: records as the compatibility rule, and which direction of the flow they pick.

Stage 3 of toymodel_v2 found a conflict: in the chain the register holds the better record at the frozen end, while the flow runs toward the Hat.  There the register was
its own memory, and one qubit holds at most one bit.  Here the records are kept outside the register, in a store that keeps every record once written ("the past is
still here"), and they replace the ad hoc ledger tolerance delta of the chain as the rule for which rewritten histories are compatible.

Definitions
  D3.1  Record store.  At tick j (j = 0..n) a record of the bit b_j entering tick j is written by an optimal reading, with fidelity p_j = (1 + sqrt(1 - c_j^2)) / 2
        (the Helstrom value; c_j the overlap at that tick).  Records are kept and are independent.  The records of the reference history agree with its bits.
  D3.2  Weight of a history H given the records: |amp(H)|^2 L(H), with L(H) relative to the reference = prod over the ticks where H and the reference differ of
        (1 - p_j) / p_j.  (The chain's ledger constraint is not used.)
  D3.3  Best backward (forward) snap to tick j: the history whose first toggle is at j with the largest weight; asymmetry asym(k, m) = log10(P_back / P_fwd) for the
        reference with one toggle at k and depth m, as in tm2_directed.py, here with weights D3.2.
  D3.4  Information in the store after tick t: I(t) = sum_{j <= t} (1 - H2(p_j)) bits.

Exact claims (each a test; ``python3 t3_records.py --tests-only``)
  E3.1  Records alone give an arrow.  With the amplitudes made flat (every angle the mean angle, so the amplitude asymmetry is exactly zero), the record weight makes
        the snap toward the frozen end the expensive one: on the chain's flow (Spectre -> Hat) asym(k, m) < 0 (going back costs more) for every k in 3..8 and
        m in 1..3 where both exist, and on the reversed flow (Hat -> Spectre) asym > 0.  In both cases rewriting a tick costs (1 - p_j)/p_j, which is about 0
        where the bit is frozen (p_j = 1) and 1 where it is fluid (p_j = 1/2).
  E3.2  Records and amplitudes agree.  With the chain's amplitudes and the records together, the sign of asym is the amplitude sign of tm2_directed.py at every grid
        point, on both flows, and its size grows: at k = 6, m = 1 on the chain's flow from -2.03 (amplitudes only) to -2.96; with flat amplitudes the records alone give -0.93 there.  Records never reverse the amplitude
        arrow on this grid.
  E3.3  The store grows on both flows (I(t) is non-decreasing by construction), but differently: on the chain's flow it gains 1.000 bit per tick for the frozen ticks
        and less and less toward the Hat (0.13 at tick 11, 0 at the Hat); on the reversed flow the gains come last.  I(n) is the same on both (10.17 bits of 13 ticks).
  E3.4  With the store, the Stage 3 conflict is gone.  "Earlier = determined" (frozen, hard to rewrite: E3.1) and "the past is well recorded" (the store's fidelity
        for a tick is highest where the tick is frozen) pick the same orientation: the chain's own, Spectre -> Hat.  "Later holds more records" holds on both
        flows, because the store only grows, so it does not choose.  The reversed flow would make the past fluid and badly recorded.

What it says.  Records are an arrow input of their own: they make rewriting the frozen end expensive even when the amplitudes are symmetric.  They point the same way
as the amplitude schedule (the frozen end is the past), so with an external store the toy's notions of past (determined, recorded, hard to change) agree.  The one
assumption doing the work is D3.1: records are written with the fidelity of the moment and never erased.  **Hypothesis**: that the store is physical (an environment);
its cost (writing, Landauer) is not modelled.

    python3 t3_records.py --tests-only
    python3 t3_records.py --story
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_V2 = os.path.join(os.path.dirname(_HERE), "toymodel_v2")
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, _V2, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import tm2_arrow as ta  # noqa: E402
import tm2_directed as td  # noqa: E402
from spectre_hat_backward_snap_model import BackwardSnap  # noqa: E402


def H2(p):
    p = min(max(p, 0.0), 1.0)
    return 0.0 if p in (0.0, 1.0) else -(p * math.log2(p) + (1 - p) * math.log2(1 - p))


class RecordedChain:
    """The chain's histories with the record weight D3.2.  ``flow`` 'chain' (Spectre -> Hat) or 'reversed'; ``flat`` replaces the angles by their mean."""

    def __init__(self, ticks=12, flow="chain", flat=False):
        base = BackwardSnap(ticks=ticks)
        q = base.qubit
        c = np.array([q.overlap(q.time.tau_of(j)) for j in range(base.n + 1)])
        if flow == "chain":
            m = base
        elif flow == "reversed":
            m = base.reversed_flow()
            c = c[::-1].copy()
        else:
            raise ValueError("flow must be 'chain' or 'reversed'")
        if flat:
            m = ta.flat_schedule(m)
        self.m, self.n, self.c = m, m.n, c
        self.p = 0.5 * (1.0 + np.sqrt(np.maximum(0.0, 1.0 - c * c)))
        p = m.paths["pattern"]
        bits = np.zeros((len(p), self.n + 1), dtype=np.int8)   # 0 = +, 1 = -: the bit entering tick j (bits[:, 0] = +)
        cum = np.zeros(len(p), dtype=np.int8)
        for j in range(self.n):
            cum ^= ((p >> j) & 1).astype(np.int8)
            bits[:, j + 1] = cum
        self.bits = bits
        with np.errstate(divide="ignore"):
            self._logodds = np.log(np.where(self.p < 1.0, (1.0 - self.p) / self.p, 0.0))   # -inf where the record is perfect

    def weight(self, ref):
        """log of |amp|^2 L relative to the reference, for all patterns."""
        p = self.m.paths
        amp2 = np.abs(p["amp"]) ** 2
        with np.errstate(divide="ignore"):
            la = np.log(amp2) - math.log(amp2[ref])
        diff = self.bits != self.bits[ref]
        lr = np.where(diff, self._logodds[None, :], 0.0).sum(axis=1)
        return la + lr

    def best(self, ref, j):
        p = self.m.paths
        mask = (p["parity"] == p["parity"][ref]) & (p["first"] == j)
        if not mask.any():
            return None
        w = self.weight(ref)[mask]
        return float(w.max())

    def asymmetry(self, k, depth):
        ref = self.m.reference(k)
        lo, hi = k - depth, k + depth
        if lo < 0 or hi >= self.n:
            return None
        b, f = self.best(ref, lo), self.best(ref, hi)
        if b is None or f is None or not (math.isfinite(b) and math.isfinite(f)):
            return None
        return (b - f) / math.log(10.0)

    def store(self):
        """I(t), t = 0..n (D3.4)."""
        return np.cumsum([1.0 - H2(x) for x in self.p])


def amplitude_only(flow, k, depth):
    m = BackwardSnap(ticks=12)
    m = m if flow == "chain" else m.reversed_flow()
    return td.Directed(m).asymmetry(k, depth, None)


GRID = [(k, d) for k in range(3, 9) for d in (1, 2, 3)]


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e31_records_alone_give_an_arrow():
    for flow, sign in (("chain", -1), ("reversed", +1)):
        r = RecordedChain(flow=flow, flat=True)
        seen = 0
        for k, d in GRID:
            a = r.asymmetry(k, d)
            if a is None:
                continue
            seen += 1
            assert a * sign > 0, (flow, k, d, a)
            # the amplitudes are flat: without records the asymmetry is zero
        assert seen >= 10, (flow, seen)
    flat = ta.flat_schedule(BackwardSnap(ticks=12))
    assert td.Directed(flat).asymmetry(6, 1, None) == 0.0 or abs(td.Directed(flat).asymmetry(6, 1, None)) < 1e-12
    r = RecordedChain()
    assert r.p[0] > 1 - 1e-9 and math.isclose(r.p[-1], 0.5, abs_tol=1e-12)


def test_e32_records_and_amplitudes_agree():
    for flow, sign in (("chain", -1), ("reversed", +1)):
        r = RecordedChain(flow=flow)
        for k, d in GRID:
            a, a0 = r.asymmetry(k, d), amplitude_only(flow, k, d)
            if a is None or a0 is None:
                continue
            assert a * sign > 0 and a0 * sign > 0 and abs(a) >= abs(a0) - 1e-9, (flow, k, d, a, a0)
    r = RecordedChain()
    assert math.isclose(amplitude_only("chain", 6, 1), -2.03, abs_tol=0.01)
    assert math.isclose(r.asymmetry(6, 1), -2.96, abs_tol=0.01), r.asymmetry(6, 1)
    assert math.isclose(RecordedChain(flat=True).asymmetry(6, 1), -0.93, abs_tol=0.01)


def test_e33_the_store_grows_on_both_flows():
    a, b = RecordedChain().store(), RecordedChain(flow="reversed").store()
    assert np.all(np.diff(a) >= 0) and np.all(np.diff(b) >= 0)
    assert math.isclose(a[-1], b[-1], rel_tol=1e-12) and math.isclose(a[-1], 10.17, abs_tol=0.01), a[-1]
    gains = np.diff(np.r_[0.0, a])
    assert gains[0] > 0.999999 and math.isclose(gains[11], 0.129, abs_tol=1e-3) and gains[12] == 0.0
    assert np.all(np.diff(gains) <= 1e-12), "on the chain's flow the gains fall toward the Hat"
    assert np.all(np.diff(np.diff(np.r_[0.0, b])) >= -1e-12), "on the reversed flow they rise"


def test_e34_determined_and_recorded_pick_the_same_orientation():
    r = RecordedChain()
    # the frozen end is the hard-to-rewrite end (E3.1) and the best-recorded end
    assert r.p[0] > r.p[6] > r.p[11]
    cost = (1 - r.p) / r.p
    assert cost[0] < 1e-6 < cost[11] < 1.0
    try:
        RecordedChain(flow="x")
        raise AssertionError("no error")
    except ValueError:
        pass


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
    print("asym(k, m) = log10(P_back / P_fwd): amplitudes only | flat amplitudes + records | amplitudes + records")
    for flow in ("chain", "reversed"):
        r, rf = RecordedChain(flow=flow), RecordedChain(flow=flow, flat=True)
        for k, d in ((4, 1), (6, 1), (6, 2), (8, 1)):
            f = lambda v: "  --  " if v is None else "%+7.2f" % v  # noqa: E731
            print("  %-8s k=%d m=%d: %s | %s | %s" % (flow, k, d, f(amplitude_only(flow, k, d)), f(rf.asymmetry(k, d)), f(r.asymmetry(k, d))))
    a = RecordedChain().store()
    print("store I(t) on the chain's flow:", " ".join("%.2f" % x for x in a))
    print("record fidelity p_j:", " ".join("%.3f" % x for x in RecordedChain().p))


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
