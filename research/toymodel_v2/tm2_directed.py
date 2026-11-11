#!/usr/bin/env python3
"""tm2_directed.py - Stage 2 of PLAN.md: the quantum bit and the hexagonal letter in one history, with a vector constraint, and the scale ladder.

Setting.  Stage 1 had a timeline of letters (six roots, vector flux) and the chain (`spectre_hat_backward_snap_model.py`) had a timeline of bits (toggle pattern,
scalar area ledger Q).  They are different objects.  Here they are put in one history in the simplest way that keeps both exact:

  the letter at tick j is  b_j u_j,  with b_j = +-1 the bit entering tick j (the chain's bit) and u_j one of the six hexagonal roots (a prescribed direction).

So the sign of a letter is the bit and its axis is fixed by the tick.  The bit's amplitudes are the chain's (nothing is changed there); what changes is the constraint:
the scalar ledger Q = sum_j b_j w_j (w_j = h(tau_j)) becomes the planar flux  Phi = sum_j b_j w_j u_j  and compatibility becomes |Phi - Phi_ref| <= delta.
With every u_j equal this is the chain's constraint again.

Definitions
  D2.1  Axes u_0..u_n (one per tick, n + 1 of them, unit complex numbers; default all 1).  ``HEXC`` are the six roots as complex numbers p + q e^{i pi/3}.
  D2.2  Phi(pattern) = sum_{j=0}^{n} b_j w_j u_j  (same order of summation as the chain's Q).
  D2.3  Compatible with the reference: same final bit (B2 of the chain) and |Phi - Phi_ref| <= delta (the vector version of B3).  delta = None switches it off.
  D2.4  Depth-m asymmetry at reference tick k:  asym(k, m) = log10( P_back / P_fwd ), P_back (P_fwd) the largest relative probability of a compatible history whose first
        toggle is at k - m (k + m).  Negative = going back is the expensive direction.  Defined only when both exist.
  D2.5  Scale ladder (C8).  Assumption: the packet width sigma is an absolute width and the jump d of the bit scales with the tile, d_j = lam^j d_0 with lam^2 = 4 + sqrt(15),
        the Pisot number of the Spectre inflation.  Then c_j = exp(-d_j^2 / 8 sigma^2) = c_0^((4 + sqrt15)^j).

Exact claims (each a test; ``python3 tm2_directed.py --tests-only``)
  E2.1  Regression.  With all axes equal, Phi is the chain's Q (real, to 1e-12 and in fact bit for bit), and the compatible sets, the reach, the table of backward snaps
        (toggles, relative probability, candidates), the constrained amplitudes and the reversed flow are those of ``BackwardSnap`` to 1e-12, at n = 12 and n = 8.
  E2.2  Rotating every axis by one root (or negating all) changes nothing: the constraint depends on |Phi - Phi_ref| only.
  E2.3  The amplitude cost cannot be lowered by the vector constraint: the best compatible backward snap to tick j has relative probability <= the single toggle's
        tan^2(theta_j/2) / tan^2(theta_k/2), with equality whenever the single toggle is compatible.  With the constraint off the compatible set has 2^(n-1) histories.
  E2.4  The vector constraint is a different constraint, not a re-labelled scalar one: for the axes cycling through the six roots there are histories compatible in
        the scalar sense and not in the vector sense and the other way round (n = 12, delta = 0.4: counts in the story).
  E2.5  The search cost: the exhaustive work is 2^n for every axes sequence; the number of compatible histories and the reach are monotone in delta (more room, more histories,
        earlier reach); at delta = 0.4 the counts are 43 (scalar), 70 (axes cycling through 3 roots), 711 (6 roots).
  E2.6  The scale ladder: c_j = c_0^((4+sqrt15)^j) equals exp(-(lam^j d_0)^2 / 8 sigma^2) to 1e-12; 4 + sqrt15 is the largest eigenvalue of ``spectre_patch.substitution_matrix()`` and
        (4+sqrt15)(4-sqrt15) = 1; from c_0 = c(tau = 0.9) = 0.872 the ladder is 0.872, 0.340, 2.0e-4 (the numbers of PLAN.md C8).

  E2.7  When no compensation is needed (the best backward and forward histories are both the single toggle) the asymmetry is the amplitude ratio
        log10(tan^2(theta_(k-m)/2) / tan^2(theta_(k+m)/2)), negative on the forward flow and positive on the reversed flow, for every axes sequence and every delta;
        with the constraint off the reversed flow gives exactly the mirrored value (reference k <-> n-1-k).

Numeric claim (a sample on a fixed grid, not a proof)
  N2.1  **The vector constraint can reverse the sign of the net asymmetry, but only where it binds.**  Grid: k in {4, 6, 8}, m in {1, 2, 3},
        delta in {0.3, 0.6, 1.0}, both flows.  Structured axes (scalar, periods of 2, 3 and 6 roots): 108 cases per flow, none reversed on this grid;
        below the grid the six-root axes do reverse it on the reversed flow (k = 6, m = 1: -16.1 at delta = 0.2 and -14.6 at 0.25, +24.9 from 0.28 on).  200 random axes sequences:
        in the cases where the best histories are single toggles the sign is the amplitude sign (E2.7); where at least one needs compensating toggles it is reversed in
        18 of 994 forward cases and 145 of 572 reversed cases, always at delta = 0.3 or 0.6, never at delta = 1.0.  So the orientation carried by the amplitudes is not
        removed by the constraint in general, but a tight, generic vector constraint can override it: the asymmetry then belongs to which axes the tiling supplies, which
        Stage 2 does not know.  (PLAN.md 'fails if the vector constraint removes the asymmetry': partly met, see STAGE2.md.)

Hypothesis / not done
  * The letter's axis is prescribed, not summed over: a history with six free letters per tick and bit amplitudes on top (6^n histories) has no definition of its amplitude here.
  * The reference is the chain's single late toggle; a reference that is itself a history of the substrate has not been built.
  * Which axes the Spectre supplies (the direction of the flat-vertex opening on each tile, Stage 0) is not used; periodic and random sequences stand in for it.
  * The ladder uses an assumption (D2.5) about how d and sigma scale; nothing here derives it.

    python3 tm2_directed.py --tests-only
    python3 tm2_directed.py --story
    python3 tm2_directed.py --no-show --save figures/tm2_stage2_directed.png

Needs NumPy, ``core.py``, the chain (``spectre_hat_backward_snap_model.py``) and ``spectre_patch.py``; matplotlib only for the figure.
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
from spectre_hat_backward_snap_model import BackwardSnap  # noqa: E402

W6 = complex(math.cos(math.pi / 3), math.sin(math.pi / 3))
HEXC = np.array([p + q * W6 for p, q in core.HEX])
PI_OVER_3 = math.pi / 3.0


def axes_from(spec, n):
    """n + 1 unit complex axes from a spec: None (all 1), a period (list of root indices, repeated) or a full array."""
    if spec is None:
        return np.ones(n + 1, dtype=complex)
    if isinstance(spec, str):
        raise ValueError("axes spec must be None or a sequence")
    a = np.asarray(spec)
    if a.dtype.kind in "iu":  # root indices
        a = HEXC[np.resize(a % 6, n + 1)]
    a = np.asarray(a, dtype=complex)
    if a.shape != (n + 1,):
        raise ValueError("need %d axes, got shape %r" % (n + 1, a.shape))
    if not np.allclose(np.abs(a), 1.0, atol=1e-12):
        raise ValueError("axes must have modulus 1")
    return a


class Directed:
    """The chain's histories with the vector constraint Phi = sum b_j w_j u_j.

    Parameters: ``m`` a BackwardSnap (default ``BackwardSnap(ticks=12)``), ``axes`` as in ``axes_from``.
    """

    def __init__(self, m=None, axes=None):
        self.m = BackwardSnap(ticks=12) if m is None else m
        self.n = self.m.n
        self.axes = axes_from(axes, self.n)
        self._phi = None

    # -- the flux ----------------------------------------------------------------------
    def flux(self):
        """Phi over all 2^n patterns (D2.2), same summation order as the chain's Q."""
        if self._phi is None:
            p = self.m.paths["pattern"]
            w = self.m.weights()
            u = self.axes
            cum = np.zeros(len(p), dtype=int)
            ph = np.full(len(p), w[0] * u[0], dtype=complex)
            for j in range(self.n):
                cum ^= (p >> j) & 1
                ph += w[j + 1] * (1 - 2 * cum) * u[j + 1]
            self._phi = ph
        return self._phi

    # -- compatibility, reach, backward snaps (as in the chain, with Phi) ----------------------
    def compatible(self, ref, delta=None):
        p = self.m.paths
        mask = p["parity"] == p["parity"][ref]
        if delta is not None:
            if not (math.isfinite(float(delta)) and delta >= 0.0):
                raise ValueError("delta must be a finite number >= 0 or None, got %r" % (delta,))
            mask &= np.abs(self.flux() - self.flux()[ref]) <= delta
        return mask

    def candidates(self, ref, delta=None):
        return int(self.compatible(ref, delta).sum())

    def reach(self, ref, delta=None):
        mask = self.compatible(ref, delta)
        return int(self.m.paths["first"][mask].min())

    def backward_snap(self, ref, j, delta=None):
        """As ``BackwardSnap.backward_snap`` with the vector constraint; None if there is no compatible history with first toggle j."""
        if not 0 <= j < self.n:
            raise ValueError("tick %r is outside 0..%d" % (j, self.n - 1))
        p = self.m.paths
        mask = self.compatible(ref, delta) & (p["first"] == j)
        if not mask.any():
            return None
        idx = np.flatnonzero(mask)
        best = idx[np.argmax(np.abs(p["amp"][idx]))]
        ref_prob = abs(p["amp"][ref]) ** 2
        return {"toggles": self.m.ticks_of(int(best)), "relative_probability": float(abs(p["amp"][best]) ** 2 / ref_prob),
                "relative_amplitude": complex(p["amp"][best] / p["amp"][ref]), "candidates": int(mask.sum()),
                "work": self.m.work_backward(), "depth": self.m.ticks_of(ref)[0] - j if len(self.m.ticks_of(ref)) else 0}

    def table(self, ref, delta=None):
        k = self.m.ticks_of(ref)[0]
        return {j: self.backward_snap(ref, j, delta) for j in range(k)}

    def constrained_amplitudes(self, ref, delta=None):
        p = self.m.paths
        mask = self.compatible(ref, delta)
        s = complex(p["amp"][mask].sum())
        same = p["parity"][ref] == 0
        return (s, 0j) if same else (0j, s), float((np.abs(p["amp"][mask]) ** 2).sum())

    def reversed_flow(self):
        """The flow run the other way: the chain's reversed flow, the axes reversed with their ticks."""
        return Directed(self.m.reversed_flow(), self.axes[::-1].copy())

    # -- D2.4 -------------------------------------------------------------------------------
    def asymmetry(self, k, depth, delta=None):
        """log10(P_back / P_fwd) at reference tick k and depth m, or None if either direction has no compatible history."""
        ref = self.m.reference(k)
        lo, hi = k - depth, k + depth
        if lo < 0 or hi >= self.n:
            return None
        b = self.backward_snap(ref, lo, delta)
        f = self.backward_snap(ref, hi, delta)
        if b is None or f is None or b["relative_probability"] <= 0.0 or f["relative_probability"] <= 0.0:
            return None
        return math.log10(b["relative_probability"] / f["relative_probability"])


# -- the scale ladder (C8) ----------------------------------------------------------------
PISOT = 4.0 + math.sqrt(15.0)


def ladder(c0, levels):
    """c_j = c0^((4 + sqrt15)^j) for j = 0..levels (D2.5)."""
    if not 0.0 < c0 <= 1.0:
        raise ValueError("c0 must be in (0, 1]")
    return [c0 ** (PISOT ** j) for j in range(levels + 1)]


def ladder_from_jump(d0, sigma, levels):
    """The same ladder from the jump: d_j = lam^j d_0, c_j = exp(-d_j^2 / 8 sigma^2), lam = sqrt(4 + sqrt15)."""
    lam = math.sqrt(PISOT)
    return [math.exp(-((lam ** j) * d0) ** 2 / (8.0 * sigma ** 2)) for j in range(levels + 1)]


# -- sample sets -------------------------------------------------------------------------------
def named_axes(n=12):
    return {"scalar": None, "3 roots": [0, 1, 2], "2 roots": [0, 1], "6 roots": [0, 1, 2, 3, 4, 5]}


def random_axes(n, seed):
    rng = np.random.default_rng(seed)
    return HEXC[rng.integers(0, 6, n + 1)]


def sweep(n_random=200, deltas=(0.3, 0.6, 1.0), ks=(4, 6, 8), depths=(1, 2, 3)):
    """Direction asymmetry over the grid for the structured axes (scalar, periods 3, 2, 6) and ``n_random`` random sequences.

    Returns a dict keyed by (group, flow, regime) with counts, where group is 'structured' or 'random', flow 'fwd' or 'rev' and regime
    'undefined' (one direction has no compatible history), 'single' (both best histories are the single toggle: no compensation needed),
    'comp_ok' or 'comp_bad' (at least one best history needs compensating toggles; ok = the amplitude sign, bad = the opposite or zero);
    plus 'bad_deltas', the set of deltas at which a bad case occurred."""
    m = BackwardSnap(ticks=12)
    r = m.reversed_flow()
    seqs = [None, [0, 1, 2], [0, 1], [0, 1, 2, 3, 4, 5]] + [random_axes(m.n, s) for s in range(n_random)]
    out = {"bad_deltas": set()}

    def bump(key):
        out[key] = out.get(key, 0) + 1

    for i, s in enumerate(seqs):
        group = "structured" if i < 4 else "random"
        for name, flow, sign in (("fwd", m, -1), ("rev", r, +1)):
            d = Directed(flow, s)
            for delta in deltas:
                for k in ks:
                    ref = flow.reference(k)
                    for depth in depths:
                        a = d.asymmetry(k, depth, delta)
                        if a is None:
                            bump((group, name, "undefined"))
                            continue
                        single = (d.backward_snap(ref, k - depth, delta)["toggles"] == [k - depth]
                                  and d.backward_snap(ref, k + depth, delta)["toggles"] == [k + depth])
                        if single:
                            bump((group, name, "single"))
                        elif a * sign > 0.0:
                            bump((group, name, "comp_ok"))
                        else:
                            bump((group, name, "comp_bad"))
                            out["bad_deltas"].add(delta)
    return out


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


def test_e21_regression_to_the_chain():
    for ticks in (12, 8):
        m = BackwardSnap(ticks=ticks)
        d = Directed(m)
        assert np.array_equal(d.flux().real, m.paths["Q"]) and np.all(d.flux().imag == 0.0), "Phi is Q bit for bit"
        for k in (ticks - 2, ticks // 2):
            ref = m.reference(k)
            for delta in (None, 0.03, 0.15, 0.4, 1.5):
                assert np.array_equal(d.compatible(ref, delta), m.compatible(ref, delta))
                assert d.reach(ref, delta) == m.reach(ref, delta)
                ta, tb = d.table(ref, delta), m.table(ref, delta)
                assert ta.keys() == tb.keys()
                for j in ta:
                    if tb[j] is None:
                        assert ta[j] is None
                        continue
                    assert ta[j]["toggles"] == tb[j]["toggles"] and ta[j]["candidates"] == tb[j]["candidates"] and ta[j]["work"] == tb[j]["work"]
                    assert math.isclose(ta[j]["relative_probability"], tb[j]["relative_probability"], rel_tol=1e-12, abs_tol=1e-300)
                    assert abs(ta[j]["relative_amplitude"] - tb[j]["relative_amplitude"]) <= 1e-12 * max(1.0, abs(tb[j]["relative_amplitude"]))
                (sa, fa), pa = d.constrained_amplitudes(ref, delta)
                (sb, fb), pb = m.constrained_amplitudes(ref, delta)
                assert abs(sa - sb) <= 1e-12 and abs(fa - fb) <= 1e-12 and math.isclose(pa, pb, rel_tol=1e-12, abs_tol=1e-300)
        dr, mr = d.reversed_flow(), m.reversed_flow()
        assert np.array_equal(dr.flux().real, mr.paths["Q"])
        ref = mr.reference(ticks - 2)
        assert np.array_equal(dr.compatible(ref, 0.4), mr.compatible(ref, 0.4))


def test_e22_rotation_and_sign_do_not_matter():
    m = BackwardSnap(ticks=12)
    axes = random_axes(m.n, 3)
    base = Directed(m, axes)
    ref = m.reference(8)
    for rot in range(6):
        other = Directed(m, axes * HEXC[rot])
        assert np.array_equal(base.compatible(ref, 0.4), other.compatible(ref, 0.4)), rot
        assert np.allclose(np.abs(base.flux() - base.flux()[ref]), np.abs(other.flux() - other.flux()[ref]), atol=1e-12)
    assert np.array_equal(base.compatible(ref, 0.4), Directed(m, -axes).compatible(ref, 0.4))


def test_e23_the_constraint_cannot_lower_the_amplitude_cost():
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    assert (m.angles()[:12] / 2).max() < math.pi / 4  # every tan(theta/2) < 1, so extra toggles only lower the amplitude
    for spec in (None, [0, 1, 2], [0, 1, 2, 3, 4, 5], random_axes(m.n, 5)):
        d = Directed(m, spec)
        assert d.candidates(ref, None) == 2 ** (m.n - 1)
        for delta in (0.15, 0.4, 1.0, None):
            assert d.compatible(ref, delta)[ref]
            for j in range(3, 10):
                bs = d.backward_snap(ref, j, delta)
                if bs is None:
                    continue
                single = m.depth_weight(j, 10)
                assert bs["relative_probability"] <= single * (1 + 1e-12)
                if bs["toggles"] == [j]:
                    assert math.isclose(bs["relative_probability"], single, rel_tol=1e-12)
        # with no constraint beyond the final bit the single toggle is always the best
        for j in range(3, 10):
            assert d.backward_snap(ref, j, None)["toggles"] == [j]


def test_e24_the_vector_constraint_is_a_different_constraint():
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    sc = Directed(m).compatible(ref, 0.4)
    ve = Directed(m, [0, 1, 2, 3, 4, 5]).compatible(ref, 0.4)
    assert (sc & ~ve).any() and (ve & ~sc).any()
    assert int(sc.sum()) == 43 and int(ve.sum()) == 711
    # for an all-equal axis (any root) it is the scalar one
    for r in range(6):
        assert np.array_equal(Directed(m, np.full(m.n + 1, HEXC[r])).compatible(ref, 0.4), sc)


def test_e25_search_cost():
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    deltas = (0.03, 0.06, 0.1, 0.15, 0.25, 0.4, 0.8)
    for spec in (None, [0, 1, 2], [0, 1, 2, 3, 4, 5], random_axes(m.n, 7), random_axes(m.n, 8)):
        d = Directed(m, spec)
        counts = [d.candidates(ref, x) for x in deltas]
        reaches = [d.reach(ref, x) for x in deltas]
        assert counts == sorted(counts) and reaches == sorted(reaches, reverse=True)
        assert counts[0] >= 1 and d.m.work_backward() == 2 ** m.n and d.m.work_forward() == m.n
    assert [Directed(m, s).candidates(ref, 0.4) for s in (None, [0, 1, 2], [0, 1, 2, 3, 4, 5])] == [43, 70, 711]


def test_e26_scale_ladder():
    from spectre_hat_quantum_bit_model import SpectreHatQubit
    import spectre_patch
    q = SpectreHatQubit(ticks=12)
    c0 = q.overlap(0.9)
    lad = ladder(c0, 3)
    d0 = q.jump(0.9)
    lj = ladder_from_jump(d0, q.sigma, 3)
    assert all(math.isclose(a, b, rel_tol=1e-12) for a, b in zip(lad, lj))
    assert math.isclose(PISOT * (4.0 - math.sqrt(15.0)), 1.0, rel_tol=1e-14)
    ev = np.linalg.eigvals(np.array(spectre_patch.substitution_matrix(), dtype=float))
    assert math.isclose(max(ev.real), PISOT, rel_tol=1e-9)
    assert math.isclose(c0, 0.872, abs_tol=5e-4) and math.isclose(lad[1], 0.340, abs_tol=5e-4) and math.isclose(lad[2], 2.0e-4, rel_tol=0.05)
    assert lad[0] > lad[1] > lad[2] > lad[3] > 0.0
    assert _raises(ValueError, ladder, 0.0, 2) and _raises(ValueError, ladder, 1.5, 2)
    assert ladder(1.0, 3) == [1.0] * 4, "at the Hat (c = 1) the ladder is flat"


def test_e27_when_no_compensation_is_needed_the_sign_is_the_amplitude_sign():
    m = BackwardSnap(ticks=12)
    r = m.reversed_flow()
    t = m.tan_half()
    seen = 0
    for spec in (None, [0, 1, 2], [0, 1, 2, 3, 4, 5], random_axes(m.n, 1), random_axes(m.n, 2), random_axes(m.n, 3)):
        for flow, sign in ((m, -1), (r, +1)):
            d = Directed(flow, spec)
            tt = flow.tan_half()
            for delta in (0.3, 0.6, 1.0, None):
                for k in (4, 6, 8):
                    ref = flow.reference(k)
                    for depth in (1, 2, 3):
                        b, f = d.backward_snap(ref, k - depth, delta), d.backward_snap(ref, k + depth, delta)
                        if b is None or f is None or b["toggles"] != [k - depth] or f["toggles"] != [k + depth]:
                            continue
                        seen += 1
                        a = d.asymmetry(k, depth, delta)
                        assert math.isclose(a, math.log10((tt[k - depth] / tt[k + depth]) ** 2), rel_tol=1e-9) and a * sign > 0.0
    assert seen > 200
    # the constraint off: the reversed flow gives exactly the mirrored value
    for k in (4, 6, 8):
        for depth in (1, 2, 3):
            a = Directed(m).asymmetry(k, depth, None)
            b = Directed(r).asymmetry(m.n - 1 - k, depth, None)
            assert a is not None and b is not None and math.isclose(a, -b, rel_tol=1e-9), (k, depth, a, b)
            assert math.isclose(a, math.log10((t[k - depth] / t[k + depth]) ** 2), rel_tol=1e-9)


def test_n21_the_binding_constraint_can_reverse_the_sign():
    s = sweep(n_random=200)
    g = lambda grp, flow, reg: s.get((grp, flow, reg), 0)  # noqa: E731
    # structured axes: never reversed on the grid (108 cases per flow)
    for flow in ("fwd", "rev"):
        assert g("structured", flow, "comp_bad") == 0
        assert sum(g("structured", flow, x) for x in ("undefined", "single", "comp_ok", "comp_bad")) == 4 * 3 * 3 * 3
    # random axes: reversed only where compensating toggles are needed, only at delta < 1
    assert g("random", "fwd", "single") + g("random", "rev", "single") > 7000
    assert (g("random", "fwd", "comp_bad"), g("random", "rev", "comp_bad")) == (18, 145)
    assert s["bad_deltas"] == {0.3, 0.6}
    d6 = Directed(BackwardSnap(ticks=12).reversed_flow(), [0, 1, 2, 3, 4, 5])  # below the grid the structured six-root axes reverse the sign too
    assert [round(d6.asymmetry(6, 1, x), 1) for x in (0.2, 0.25, 0.28)] == [-16.1, -14.6, 24.9]
    assert g("random", "fwd", "comp_ok") == 976 and g("random", "rev", "comp_ok") == 427


def test_validation():
    m = BackwardSnap(ticks=8)
    assert _raises(ValueError, Directed, m, np.ones(5, dtype=complex))
    assert _raises(ValueError, Directed, m, np.full(9, 2.0, dtype=complex))
    assert _raises(ValueError, Directed, m, "x")
    d = Directed(m)
    assert _raises(ValueError, d.compatible, 0, -0.1) and _raises(ValueError, d.compatible, 0, float("nan"))
    assert _raises(ValueError, d.backward_snap, m.reference(5), 8)
    assert d.asymmetry(0, 1, None) is None and d.asymmetry(7, 1, None) is None


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
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    print("compatible histories (n = 12, reference: one toggle at tick 10)")
    print("  delta   " + "  ".join("%9s" % k for k in ("scalar", "3 roots", "6 roots")) + "   reach (scalar / 3 / 6)")
    for dl in (0.05, 0.15, 0.4, 0.8):
        ds = [Directed(m, s) for s in (None, [0, 1, 2], [0, 1, 2, 3, 4, 5])]
        print("  %5.2f   %s   %s" % (dl, "  ".join("%9d" % d.candidates(ref, dl) for d in ds), " / ".join(str(d.reach(ref, dl)) for d in ds)))
    print("direction asymmetry log10(P_back/P_fwd), k = 6, delta = 0.6 (forward flow | reversed flow), depths 1,2,3")
    r = m.reversed_flow()
    for name, s in (("scalar", None), ("3 roots", [0, 1, 2]), ("6 roots", [0, 1, 2, 3, 4, 5])):
        f = [Directed(m, s).asymmetry(6, x, 0.6) for x in (1, 2, 3)]
        b = [Directed(r, s).asymmetry(6, x, 0.6) for x in (1, 2, 3)]
        fmt = lambda v: "  --  " if v is None else "%6.2f" % v  # noqa: E731
        print("  %-8s %s | %s" % (name, " ".join(fmt(v) for v in f), " ".join(fmt(v) for v in b)))
    s = sweep(200)
    for grp in ("structured", "random"):
        for flow in ("fwd", "rev"):
            print("  sweep %-10s %s: " % (grp, flow) + ", ".join("%s %d" % (x, s.get((grp, flow, x), 0)) for x in ("single", "comp_ok", "comp_bad", "undefined")))
    print("  deltas at which the sign was reversed:", sorted(s["bad_deltas"]))
    c0 = 0.8718
    print("scale ladder from c0 = %.4f:" % c0, " ".join("%.3g" % c for c in ladder(c0, 3)), "(lam^2 = 4 + sqrt15 = %.4f)" % PISOT)


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    m = BackwardSnap(ticks=12)
    r = m.reversed_flow()
    ref = m.reference(10)
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    deltas = np.geomspace(0.03, 1.5, 30)
    for name, s in (("scalar", None), ("3 roots", [0, 1, 2]), ("6 roots", [0, 1, 2, 3, 4, 5])):
        d = Directed(m, s)
        ax[0].semilogy(deltas, [d.candidates(ref, x) for x in deltas], label=name)
    ax[0].set_xscale("log")
    ax[0].set_xlabel("delta")
    ax[0].set_ylabel("compatible histories (of 2048)")
    ax[0].legend()
    ax[0].set_title("search: the room the constraint leaves")
    for name, s in (("scalar", None), ("3 roots", [0, 1, 2]), ("6 roots", [0, 1, 2, 3, 4, 5])):
        for flow, ls in ((m, "-"), (r, "--")):
            d = Directed(flow, s)
            ys = [d.asymmetry(6, 1, x) for x in deltas]
            xs = [x for x, y in zip(deltas, ys) if y is not None]
            ax[1].plot(xs, [y for y in ys if y is not None], ls, label="%s %s" % (name, "fwd" if flow is m else "rev"))
    ax[1].axhline(0.0, color="k", lw=0.6)
    ax[1].set_xscale("log")
    ax[1].set_xlabel("delta")
    ax[1].set_ylabel("log10(P_back / P_fwd), k = 6, m = 1")
    ax[1].legend(fontsize=7, ncol=2)
    ax[1].set_title("the sign of the asymmetry")
    lad = ladder(0.8718, 3)
    ax[2].semilogy(range(4), lad, "o-")
    ax[2].set_xlabel("inflation level j")
    ax[2].set_ylabel("c_j = c_0^((4+sqrt15)^j)")
    ax[2].set_title("scale ladder (assumption D2.5)")
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
