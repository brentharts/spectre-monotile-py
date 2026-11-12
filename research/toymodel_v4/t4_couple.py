#!/usr/bin/env python3
"""t4_couple.py - Stage 4 of research/toymodel_v4/PLAN.md (as asked): couple the bits of a patch, so that neighbouring flat pairs interact, and see whether an ordering of tau emerges that does not
depend on how tau is written.

Two parts: what the geometry itself couples (Exact), and what a stated interaction couples (a model choice, Numeric / Hypothesis).

Definitions
  D6.1  State of a flat pair (A, B) of the level-3 patch: sigma = +1 is (s_A, s_B) = (+1, -1), sigma = -1 is (-1, +1), the two seam-closing states of t4_patch.py E2.4.  Each pair has its own h = a tan(omega_p / 2).
  D6.2  Area charge: in state sigma the tile A holds the area change +sigma h and B holds -sigma h (t4_patch.py E2.3), placed at the centroids c_A, c_B of the straight polygons.
  D6.3  Interaction (a choice, not derived): E(sigma) = sum_{i<j} J_ij sigma_i sigma_j, J_ij = h_i h_j sum_{a in i, c in j} q_a q_c V(|c_a - c_c|), q_A = +1, q_B = -1, V one of -log r (a 2D Coulomb
        law for an area source), 1/r, 1/r^2.  There is no free coefficient: h enters as a common factor when all pairs share omega.
  D6.4  Cluster: the 16 flat pairs of the level-3 patch whose midpoints are nearest the patch centre at b = 1; ground state by exact enumeration of the 2^16 states, taken up to the global flip.

Claims (each a test; ``python3 t4_couple.py --tests-only``)
  E4.1  Geometric decoupling.  In the level-3 patch the edges 7 and 8 of each of the 164 tiles of flat pairs meet only the partner (edge 7 with edge 8 and edge 8 with edge 7); so the union of a pair has the same
        outer boundary in all four sign assignments that close its seams, and nothing outside the pair sees the opening.  Opening every pair at once, each with its own h and its own state, closes every seam.
  E4.2  What does couple: (b, k) are one clock for the whole patch.  Changing b or k of a single tile breaks the seams around it; opening a pair by any h does not.  The corner angle omega is the only
        parameter that can differ from pair to pair.
  E4.3  Negative: no ordering emerges.  Under D6.3 the ground state does not depend on omega (a common h scales the energy by h^2).  For the central 16-pair cluster it does not depend on b either:
        it is the same state at all 21 values of b in [1, sqrt 3], and the same for the three kernels; its energy changes monotonically with b (a smooth function, not an event) and the gap to the first
        excited state stays positive.  On 8 random 12-pair subsets (seed 1, 15 values of b) the ground state changes 0, 1 and 1 times in total for -log r, 1/r, 1/r^2: a few level crossings, at b values
        that depend on the kernel and the subset.  So the interaction adds no sequence of events along the flow of its own; the only dependence on the flow is the scale h^2 (a function of omega, falling to 0 at
        the Hat) and a slow drift of the energy with b.

Status.  Geometry couples the pairs through (b, k) only.  An interaction that couples them in omega has to be put in; the one of D6.3 is an assumption, and with it no ordering of tau emerges beyond the
parameters themselves (E4.3).
Hypothesis, not tested: that area is a source of anything.  Not done: the whole patch (82 spins, no exact solution; annealing is not done), validity (the pair cannot open above omega_c, which would remove spins
along the flow) and per-pair omega_p in the interaction.

    python3 t4_couple.py --tests-only
    python3 t4_couple.py --story
"""
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_ROOT, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from spectre_hat_collinearity_flow_model import SQRT3  # noqa: E402
from spectremodel import Tile  # noqa: E402
import t4_patch as tp  # noqa: E402

LEVEL = 3
N_CLUSTER = 16
KERNELS = {"log": lambda r: -np.log(r), "inv": lambda r: 1.0 / r, "inv2": lambda r: 1.0 / r ** 2}
BS = np.linspace(1.0, SQRT3, 21)


def pair_data(b):
    """(pairs, centroids): the flat pairs of the level-3 patch at b and the centroid of each tile."""
    P = tp.patch(LEVEL, b)
    pairs = tp.flat_pairs(LEVEL, b)
    cen = np.array([np.mean(P.straight_polygon(i), axis=0) for i in range(P.n_tiles)])
    return pairs, cen


def cluster(n=N_CLUSTER):
    """Indices (into the pair list) of the n pairs nearest the centre of the patch at b = 1."""
    pairs, cen = pair_data(1.0)
    mid = np.array([0.5 * (cen[a] + cen[c]) for a, c in pairs])
    ctr = np.mean(cen, axis=0)
    return list(np.argsort(np.linalg.norm(mid - ctr, axis=1))[:n])


def couplings(b, sel, kernel, h=None):
    """J_ij of D6.3 for the pairs ``sel`` at b (h = None: all h equal to 1)."""
    pairs, cen = pair_data(b)
    V = KERNELS[kernel]
    h = np.ones(len(sel)) if h is None else np.asarray(h, dtype=float)
    pts = [(cen[pairs[i][0]], cen[pairs[i][1]]) for i in sel]
    n = len(sel)
    J = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            e = 0.0
            for qa, ca in zip((1.0, -1.0), pts[i]):
                for qc, cc in zip((1.0, -1.0), pts[j]):
                    e += qa * qc * V(np.linalg.norm(ca - cc))
            J[i, j] = J[j, i] = h[i] * h[j] * e
    return J


_CONF = {}


def configs(n):
    if n not in _CONF:
        _CONF[n] = (1.0 - 2.0 * ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1)).astype(float)
    return _CONF[n]


def ground_state(J):
    """(energy, state up to the global flip as a tuple with sigma_0 = +1, gap to the next distinct state)."""
    S = configs(J.shape[0])
    E = 0.5 * np.einsum("ci,ij,cj->c", S, J, S)
    order = np.argsort(E)
    g = S[order[0]] * S[order[0]][0]
    gap = None
    for k in order[1:]:
        if not np.allclose(S[k] * S[k][0], g):
            gap = float(E[k] - E[order[0]])
            break
    return float(E[order[0]]), tuple(int(x) for x in g), gap


def sequence(kernel, bs=BS, sel=None):
    sel = cluster() if sel is None else sel
    return [ground_state(couplings(b, sel, kernel))[1] for b in bs]


def changes(seq):
    return sum(1 for a, b in zip(seq, seq[1:]) if a != b)


def cp_with(P, i, b=None, k=0.5):
    """Control points of tile i placed in the patch with its own (b, k), other tiles unchanged (the parameters are not those of the patch)."""
    a0, b0 = P.tile_params(i)
    t = Tile(a0, b0 if b is None else b, k)
    return tp._place(P.placements[i], np.asarray(t.control_points()))


def seams_broken_by(levels, b, k, i, cp):
    base = tp.base_cps(levels, b, k)
    P = tp.patch(levels, b)
    bad = 0
    for (A, ia, B, ib) in tp.seam_rows_by_tile(levels, b).get(i, []):
        ca = cp if A == i else base[A]
        cb = cp if B == i else base[B]
        if not np.allclose(ca[ia], cb[ib][::-1], atol=tp.TOL):
            bad += 1
    return bad


# --------------------------------------------------------------------------
# tests
# --------------------------------------------------------------------------
def test_e41_geometric_decoupling():
    pairs = tp.flat_pairs(LEVEL, 1.0)
    by = tp.seam_rows_by_tile(LEVEL, 1.0)
    partner = {}
    for A, B in pairs:
        partner[A], partner[B] = B, A
    n_rows = 0
    for t, o in partner.items():
        for (a, i, b, j) in by[t]:
            me, ie, other, je = (a, i, b, j) if a == t else (b, j, a, i)
            if ie in (7, 8):
                assert other == o and je == (15 - ie), (t, ie, other, je)  # edge 7 meets edge 8 of the partner and vice versa
                n_rows += 1
    assert n_rows == 2 * len(partner) == 328 and len(partner) == 164
    rng = np.random.default_rng(11)
    for b, k in ((1.0, 0.5), (SQRT3, 0.3)):
        opened = {}
        for A, B in tp.flat_pairs(LEVEL, b):
            h = math.tan(math.radians(rng.uniform(5.0, 120.0)) / 2.0)
            sg = rng.choice((1, -1))
            opened[A], opened[B] = (sg, h), (-sg, h)
        assert tp.broken_seams(LEVEL, b, k, opened) == [], (b, k)


def test_e42_b_and_k_are_one_clock():
    b, k = 1.0, 0.5
    P = tp.patch(LEVEL, b)
    in_pair = {t for pr in tp.flat_pairs(LEVEL, b) for t in pr}
    for i in sorted(in_pair)[:12]:
        assert seams_broken_by(LEVEL, b, k, i, tp.base_cps(LEVEL, b, k)[i]) == 0
        assert seams_broken_by(LEVEL, b, k, i, cp_with(P, i, b=b + 0.05, k=k)) > 0, ("b", i)
        assert seams_broken_by(LEVEL, b, k, i, cp_with(P, i, b=b, k=k + 0.1)) > 0, ("k", i)


def test_e43_no_ordering_emerges():
    sel = cluster()
    assert all(tc_pairs == pair_data(1.0)[0] for tc_pairs in (pair_data(b)[0] for b in BS)), "pair list and tile indices are the same for every b"
    states, energy = set(), {}
    for kern in KERNELS:
        es = []
        for b in BS:
            e1, g1, gap = ground_state(couplings(b, sel, kern))
            e2, g2, _ = ground_state(couplings(b, sel, kern, h=np.full(len(sel), 0.5)))
            assert g1 == g2 and abs(e2 - 0.25 * e1) < 1e-9 * max(1.0, abs(e1)), (kern, b)
            assert gap is not None and gap > 1e-3, (kern, b, gap)
            states.add(g1)
            es.append(e1)
        d = np.diff(es)
        assert np.all(d < 0) or np.all(d > 0), kern
        energy[kern] = es
    assert len(states) == 1, states
    rng = np.random.default_rng(1)
    pairs = pair_data(1.0)[0]
    tot = {}
    for kern in KERNELS:
        tot[kern] = 0
        for _ in range(8):
            sub = list(rng.choice(len(pairs), 12, replace=False))
            tot[kern] += changes(sequence(kern, np.linspace(1.0, SQRT3, 15), sub))
    assert tot == {"log": 0, "inv": 1, "inv2": 1}, tot


def run_tests():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    failed = 0
    for n, f in tests:
        try:
            f()
            status = "PASS"
        except Exception as exc:  # noqa: BLE001
            failed += 1
            status = "FAIL: %s: %s" % (type(exc).__name__, exc)
        print("  %-82s %s" % (n, status))
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


def story():
    sel = cluster()
    for kern in KERNELS:
        seq = sequence(kern, sel=sel)
        distinct = []
        for s in seq:
            if s not in distinct:
                distinct.append(s)
        print("%-5s ground-state changes along b: %d, distinct states %d" % (kern, changes(seq), len(distinct)))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ok = run_tests()
    if not ok or "--tests-only" in argv:
        return 0 if ok else 1
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
