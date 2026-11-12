#!/usr/bin/env python3
"""t5_arrow.py - Stage 4 of toymodel_v5: the arrow audit of toymodel_v2 (Stage 3) on the ensemble.

v2 found five inputs with an orientation or a binary datum: (i) pinned end / flux base, (ii) damping gamma, (iii) orientation of tau, (iv) handedness, (v) scale ordering; three carry an arrow, two do not.
The ensemble of Stage 3 adds binary data: the order of the layers (which side the live tiles are on), the direction of the flow (which end of tau is silent: h(1) = 0), the chirality of the chair
(its charges are odd under x <-> y, Stage 2) and the cyclic order of the six points (Stage 0).  Each is audited here by switching it and asking which observable follows it.

Exact claims (each a test; ``python3 t5_arrow.py --tests-only``)
  E5.20  Chair with groups has an exact "mirror times charge conjugation" symmetry.  P = exchange of x and y about the chair's centre maps the chair's charges to their negatives (Stage 2) and the groups
         (+x, +y, -x, -y) to (+y, +x, -y, -x).  Hence C P S(chair, s) = S(chair, -sigma s) with sigma the permutation (0 1)(2 3) of the groups: the potential of the configuration is mapped to itself
         with all bits reversed and exchanged.  Mirroring the tile (y -> -y, cyclic order of the six points reversed) leaves the CHSH value of Stage 1 unchanged to 1e-12 for every k and beta tried:
         the six points' cyclic order is not an orientation of the rule.
  E5.21  The live side is a dipole.  The three Hat tiles are silent, so the dipole of the bit sources is p = sum_i h_i s_i z_i over the full tiles only: for the three full tiles p_z = Z h sum s,
         an odd multiple of sqrt3 Z at least: it points to the live side and never vanishes (odd count; E5.17).  Exchanging the layers reverses p exactly and changes nothing else about the
         monopole; the Coulomb interaction energy of the three live bits (all +) with the chair's charges is -0.187 for tiles up and +0.076 for tiles mirrored down (the chair is not symmetric
         under z -> -z): the layer order is seen by the chair, with a sign change.
  E5.22  Layer order and flow direction are one datum in the field.  Four ensembles: (up, tau_full = 0 live), (down, tau_full = 0), (up, tau flow reversed: the tiles on the up side are the silent
         ones), (down, reversed).  The dipole of the live bits is +, -, -, +: only the product of the two choices is observable in the source.  The field cannot tell "Spectre above, flow
         downward" from "Hat above, flow upward".
  E5.23  Damping on the ensemble.  A damped leapfrog step with a stationary source returns the data after time reversal only if gamma is reversed with the time (error < 1e-9), and not otherwise
         (error > 1e-3 at gamma = 0.2)  (3.9 at gamma = 0.2).  Damping and the flip of a bit are independent inputs: with gamma = 0 the flip is
         reversible (Stage 3 E5.15); gamma is a third orientation, not a consequence of the layers.

Status.  All Exact (counting, algebra, finite schemes).  Which of these inputs the physics would use is not decided; the Stage 4 result is the list and its dependencies.

    python3 t5_arrow.py --tests-only
    python3 t5_arrow.py --no-show --save figures/t5_stage4_arrow.png
"""
import argparse
import itertools
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
for _p in (_HERE, os.path.join(os.path.dirname(_HERE), "toymodel_v2"), _ROOT, os.path.join(_ROOT, "models")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import t5_ensemble as te  # noqa: E402
import t5_rule as tr  # noqa: E402
import tm2_waves as w  # noqa: E402


def _centre():
    return te.centre()


def swap_xy(p):
    c = _centre()
    d = np.asarray(p) - c
    return c + np.array([d[1], d[0], d[2]])


def sums_by_site(items):
    out = {}
    for p, q in items:
        k = tuple(int(x) for x in p)
        out[k] = out.get(k, 0.0) + float(q)
    return {k: v for k, v in out.items() if abs(v) > 1e-12}


def chair_plus_groups(s, chair_sign=1.0):
    sites, q = te.chair_sites()
    items = [(p, chair_sign * qq) for p, qq in zip(sites, q)]
    pos, taus = te.bit_sites()
    st = te.strengths(taus)
    for i in range(6, 10):
        items.append((pos[i], st[i] * s[i - 6]))
    return items


def mirror_tile_alphabet(V):
    M = np.asarray(V) * np.array([1.0, -1.0])
    return M[np.argsort(np.arctan2(M[:, 1], M[:, 0]), kind="stable")]


def live_dipole(layer_up=True, flow_reversed=False, bits=(1, 1, 1)):
    """Dipole z-component of the bit sources of the three tiles above/below: the live ones are the tiles whose flow time is 0."""
    Z = te.Z
    up_tau = 1.0 if flow_reversed else 0.0
    down_tau = 0.0 if flow_reversed else 1.0
    if not layer_up:
        up_tau, down_tau = down_tau, up_tau
    p = 0.0
    for zs, tau in ((+Z, up_tau), (-Z, down_tau)):
        p += sum(te.h(tau) * b * zs for b in bits)
    return p


def damped_run(psi0, psi1, src, steps, gamma, dt, rho):
    prev, cur = psi0.copy(), psi1.copy()
    g = gamma * dt / 2.0
    for _ in range(steps):
        nxt = (2.0 * cur - (1.0 - g) * prev + dt ** 2 * rho * (w.laplacian(cur) + src)) / (1.0 + g)
        prev, cur = cur, nxt
    return prev, cur


def damped_roundtrip(gamma, gamma_back, steps=30, seed=0):
    rng = np.random.default_rng(seed)
    n = te.N
    p0 = 1e-2 * rng.standard_normal((n, n, n))
    p1 = p0 + 1e-2 * rng.standard_normal((n, n, n))
    src = te.source([1] * 10)
    rho = te.density()
    a, b = damped_run(p0, p1, src, steps, gamma, te.DT, rho)
    c, d = damped_run(b, a, src, steps, gamma_back, te.DT, rho)
    return float(max(np.abs(c - p1).max(), np.abs(d - p0).max()))


# --------------------------------------------------------------------------
def test_e520_mirror_times_conjugation_and_tile_mirror():
    sigma = (1, 0, 3, 2)
    for s in itertools.product((1, -1), repeat=4):
        left = sums_by_site([(swap_xy(p), q) for p, q in chair_plus_groups(s)])          # P S(chair, s)
        s2 = tuple(s[sigma[i]] for i in range(4))
        right = sums_by_site(chair_plus_groups(s2, chair_sign=-1.0))                      # S(-chair, sigma s)
        assert left.keys() == right.keys() and all(abs(left[k] - right[k]) < 1e-12 for k in left), s
    # the chair alone is odd, so the groups' bits are what restores the symmetry
    only = sums_by_site([(swap_xy(p), q) for p, q in chair_plus_groups((0, 0, 0, 0))])
    assert only != sums_by_site(chair_plus_groups((0, 0, 0, 0)))
    worst = 0.0
    for k in (0.0, 0.5, math.sqrt(3) / 2, 1.0):
        V = tr.alphabet(k)
        M = mirror_tile_alphabet(V)
        assert np.allclose(M[3:], -M[:3])
        for beta in (0.5, 1.255, 3.0):
            worst = max(worst, abs(tr.chsh_of(V, beta) - tr.chsh_of(M, beta)))
    assert worst < 1e-12, worst
    return worst


def test_e521_the_live_side_is_a_dipole():
    p_up = live_dipole(True)
    p_dn = live_dipole(False)
    assert abs(p_up + p_dn) < 1e-12 and abs(p_up - 3 * te.Z * math.sqrt(3)) < 1e-12
    for bits in itertools.product((1, -1), repeat=3):                 # an odd count: never zero
        assert abs(live_dipole(True, bits=bits)) >= te.Z * math.sqrt(3) - 1e-12
    # interaction energy with the chair, up against down
    sites, q = te.chair_sites()
    c = _centre()
    pos, taus = te.bit_sites()
    def U(up):
        u = 0.0
        for i in range(3):
            pz = pos[i].copy()
            if not up:
                pz[2] = 2 * c[2] - pz[2]
            u += te.h(0.0) * float((q / np.linalg.norm(sites - pz, axis=1)).sum())
        return u
    uu, ud = U(True), U(False)
    assert abs(uu - ud) > 1e-3, (uu, ud)
    return uu, ud


def test_e522_layer_order_and_flow_direction_are_one_datum():
    d = {(u, r): live_dipole(u, r) for u in (True, False) for r in (False, True)}
    assert d[(True, False)] > 0 and d[(False, False)] < 0 and d[(True, True)] < 0 and d[(False, True)] > 0
    assert abs(d[(True, False)] - d[(False, True)]) < 1e-12 and abs(d[(False, False)] - d[(True, True)]) < 1e-12
    return d


def test_e523_damping_on_the_ensemble():
    e0 = damped_roundtrip(0.0, 0.0)
    assert e0 < 1e-9, e0
    e_back = damped_roundtrip(0.2, -0.2)
    assert e_back < 1e-9, e_back
    e_bad = damped_roundtrip(0.2, 0.2)
    assert e_bad > 1e-3, e_bad
    return e0, e_back, e_bad


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e5")]
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

    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    names = ["up, flow 0->1", "down, flow 0->1", "up, flow reversed", "down, flow reversed"]
    vals = [live_dipole(u, r) for u in (True, False) for r in (False, True)]
    vals = [vals[0], vals[2], vals[1], vals[3]]
    ax[0].bar(range(4), vals, color=["tab:blue", "tab:orange", "tab:orange", "tab:blue"])
    ax[0].set_xticks(range(4)); ax[0].set_xticklabels(names, rotation=25, ha="right", fontsize=7)
    ax[0].set_ylabel("dipole p_z of the live bits"); ax[0].set_title("layer order x flow = one datum")
    gs = np.linspace(0, 0.4, 9)
    ax[1].semilogy(gs, [max(damped_roundtrip(g, g, steps=20), 1e-16) for g in gs], "r", label="gamma not reversed")
    ax[1].semilogy(gs, [max(damped_roundtrip(g, -g, steps=20), 1e-16) for g in gs], "g", label="gamma reversed")
    ax[1].set_xlabel("gamma"); ax[1].set_ylabel("retrace error"); ax[1].legend(fontsize=7); ax[1].set_title("damping is a third input")
    ks = np.linspace(0, 1.3, 27)
    ax[2].plot(ks, [tr.chsh_of(tr.alphabet(k), 1.255) for k in ks], "k", label="tile")
    ax[2].plot(ks, [tr.chsh_of(mirror_tile_alphabet(tr.alphabet(k)), 1.255) for k in ks], "r--", label="mirrored tile")
    ax[2].set_xlabel("k"); ax[2].set_ylabel("CHSH (beta 1.255)"); ax[2].legend(fontsize=7); ax[2].set_title("handedness is invisible")
    fig.suptitle("toymodel_v5 Stage 4: arrow audit on the ensemble")
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
