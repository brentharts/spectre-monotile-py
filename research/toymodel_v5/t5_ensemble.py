#!/usr/bin/env python3
"""t5_ensemble.py - Stage 3 of toymodel_v5: the ensemble (Chair44 + 3 full tiles + 3 Hat tiles + 4 groups) as a source on the reversible sheet S of toymodel_v2.

Layout (lattice units; the chair is scaled by L = 3 sites per chair unit, centre at the middle of an n^3 periodic lattice):
  * chair: the 192 signed depths j of Stage 2 as static sources q = j at the nearest site of the tip (several tips may share a site; they add).
  * 3 full tiles (flow time TAU_FULL, default 0 = Spectre) at z = +Z, 3 Hat tiles (tau = 1) at z = -Z, on the corners of a triangle of radius 4 about the axis;
    4 groups at (+-R, 0, 0), (0, +-R, 0) (the "faces" layout of chair_fibers_14th), flow time TAU_GROUP (default 0).  Ten bits s_i = +-1 in all.
  * Tiles as density: c^2 rho = 1 + EPS_RHO on the sites of the tiles (a Definition: the tile is a place where the sheet is stiffer / denser; EPS_RHO is a free parameter).
  * Bits as sources: a bit flip changes the tile's area by 2 a h(tau) (v4: A+- = A0 +- a h, h = a tan(omega/2), omega = omega0 (1 - tau)), so the bit i is the source
    q_i = kappa a h(tau_i) s_i at the tile's site; kappa = 1 (a Definition: one unit of area is one unit of charge).  At tau = 1, h = 0: the Hat tiles are frozen.

Definitions
  D5.15  h(tau) = tan(omega0 (1 - tau) / 2), omega0 = 120 degrees (a = 1);  q_i(s) = kappa h(tau_i) s_i.
  D5.16  The sheet is toymodel_v2's Sheet (leapfrog of (W), D5.1); a flip of bit i at step f replaces its source q_i s_i by -q_i s_i from step f on.

Exact / Numeric claims (each a test; ``python3 t5_ensemble.py --tests-only``)
  E5.15  Reversibility with a flip.  Forward n steps with a bit flipped at step f, then the time-reversed run with the reversed source schedule, returns the initial data to 1e-9
         (with the tile density).  The flip is a stated term: with the flip omitted from the reversed run the data do not return (> 1e-3).  Energy (D5.1) is continuous across the flip
         (psi(n+1) differs from the unflipped run by exactly dt^2 c^2 rho (B - A), 1e-13): the flip enters only through the source term.
  E5.16  The Hat layer is frozen.  A flip of a Hat bit changes the source by exactly 0 and the field by exactly 0; a flip of a full tile changes the source by 2 h(0) = 2 sqrt3.
         Along the flow the strength of a flip, 2 h(tau), falls strictly from 2 sqrt3 at tau = 0 to 0 at tau = 1 (the plastic end of v4 is the silent end of the field).
  E5.17  The ensemble cannot be neutral at tau = 0.  Seven bits of equal strength h (3 full + 4 groups; the Hat's three have h = 0) give a net source sum s_i h that is an odd multiple of h,
         never 0: among the 2^10 patterns the minimum |monopole| is h(0) = sqrt3, reached by 560 patterns (the C(7,3) + C(7,4) = 70 patterns of the seven live bits, times 2^3 for the free Hat bits).
         Neutrality needs the groups at another flow time: it is exact at isolated tau_group (h = sqrt3/2 at 0.3184, sqrt3/4 at 0.6098; 3 sqrt3 = ... with the full tiles at tau = 0), nowhere else on a grid of 20.  The pair rule of v4 (antiparallel pairs) is
         impossible for an odd number of live bits.
  E5.18  Bits against the chair's quadrupole.  The chair's traceless quadrupole (Stage 2) has an xx - yy part (+-39) and an xz, yz part (+-19.5); the four group bits (on +-x, +-y) make
         a quadrupole with xx, yy entries only.  The largest overlap over the 16 patterns is cos = 0.8165 = sqrt(2/3) (to 1e-4), reached by (+,-,+,-) and its negative: the bits can cancel the
         xx - yy part (2/3 of the squared norm) but never the xz, yz part, whatever the coupling; after the best scale the residual is 0.577 = 1/sqrt3 of the chair's norm.  The cancelling
         pattern has no group monopole and no group dipole (opposite sides carry equal bits).  The scale it needs, 1.58 (kappa = 1.58 at R = 8 sites, tau_group = 0), is of order one;
         that is not derived and is not claimed as a coincidence of meaning.  The xz, yz part would need groups on the +-z sides (the "spread" layout), which the faces layout leaves empty.
  E5.19  The far field is set by the lowest nonzero multipole of the whole: r^-1 if the net bit source is nonzero (which E5.17 says it always is), and the settled field of a flip on the
         lattice at r = 8 agrees with q G_3(r) within 3% (regression of v2 E5.6 with the ensemble's own geometry).

Status.  Algebra and counting: Exact.  The field regression is Numeric.  That a bit is a source of strength h and the tile a density are Definitions; nothing here is gravity.

    python3 t5_ensemble.py --tests-only
    python3 t5_ensemble.py --no-show --save figures/t5_stage3_ensemble.png
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
import t5_source as ts  # noqa: E402
import tm2_waves as w  # noqa: E402

OMEGA0 = 120.0
N = 48
L = 3                 # lattice sites per chair unit
Z = 8                 # height of the tile layers
R = 8                 # distance of the groups
EPS_RHO = 0.5
KAPPA = 1.0
TAU_FULL = 0.0
TAU_GROUP = 0.0
DT = 0.4


def h(tau):
    return math.tan(math.radians(OMEGA0 * (1.0 - tau)) / 2.0)


def centre():
    return np.array([N // 2] * 3)


def chair_sites():
    d = np.array([[float(x) for x in v] for v in ts.displacements()])
    q = np.array(ts.charges(), float)
    return centre() + np.rint(L * d).astype(int), q


def bit_sites():
    """Ten bit sites in the order: 3 full, 3 Hat, 4 groups, and their flow times."""
    c = centre()
    tri = [np.array([round(4 * math.cos(2 * math.pi * k / 3)), round(4 * math.sin(2 * math.pi * k / 3)), 0]) for k in range(3)]
    pos = [c + t + np.array([0, 0, Z]) for t in tri] + [c + t + np.array([0, 0, -Z]) for t in tri]
    pos += [c + np.array(v) for v in ((R, 0, 0), (0, R, 0), (-R, 0, 0), (0, -R, 0))]
    taus = [TAU_FULL] * 3 + [1.0] * 3 + [TAU_GROUP] * 4
    return np.array(pos), np.array(taus)


def strengths(taus=None):
    taus = bit_sites()[1] if taus is None else np.asarray(taus, float)
    return KAPPA * np.array([h(t) for t in taus])


def source(bits, taus=None, chair=True):
    """The static source array for the ten bits (each +-1)."""
    src = np.zeros((N, N, N))
    if chair:
        s, q = chair_sites()
        for (i, j, k), qq in zip(s, q):
            src[i, j, k] += qq
    pos, tt = bit_sites()
    st = strengths(tt if taus is None else taus)
    for p, a, b in zip(pos, st, bits):
        src[tuple(p)] += a * b
    return src


def density():
    pos, _ = bit_sites()
    rho = np.ones((N, N, N))
    for p in pos[:6]:
        rho[tuple(p)] = 1.0 + EPS_RHO
    return rho


def run_with_flip(n_steps, flip_step, bits, flip_index, reverse_schedule=True, density_on=True, seed=0):
    """Forward run with a flip, then the reversed run; returns (error of the returned data, the sheet)."""
    rng = np.random.default_rng(seed)
    psi0 = 1e-2 * rng.standard_normal((N, N, N))
    psi1 = psi0 + 1e-2 * rng.standard_normal((N, N, N))
    bits2 = list(bits)
    bits2[flip_index] *= -1
    A, B = source(bits), source(bits2)
    rho = density() if density_on else 1.0
    s = w.Sheet(psi0, psi1, rho, DT, A)
    for k in range(1, n_steps + 1):
        s.source = A if k < flip_step else B
        s.step(1)
    s.reverse()
    for k in range(n_steps, 0, -1):
        s.source = (A if k < flip_step else B) if reverse_schedule else B
        s.step(1)
    s.reverse()
    return float(max(np.abs(s.prev - psi0).max(), np.abs(s.cur - psi1).max())), s


# --------------------------------------------------------------------------
def monopoles(taus):
    st = strengths(taus)
    return np.array([float(np.dot(st, np.array(b))) for b in itertools.product((1, -1), repeat=10)])


def group_quadrupole(s):
    """Traceless quadrupole sum q (3 r r - r^2 delta)/... of the four group bits, in the units of the chair's traceless Q (sum q d d - tr/3 delta), at the group distance R / L chair units."""
    d = R / L
    Q = np.zeros((3, 3))
    for (ax, sg), b in zip(((0, 1), (1, 1), (0, -1), (1, -1)), s):
        r = np.zeros(3)
        r[ax] = sg * d
        Q += KAPPA * h(TAU_GROUP) * b * np.outer(r, r)
    return Q - np.trace(Q) / 3 * np.eye(3)


def overlaps():
    _, Qc = ts.traceless(ts.moments()[2])
    out = []
    for s in itertools.product((1, -1), repeat=4):
        Qb = group_quadrupole(s)
        nb = np.linalg.norm(Qb)
        out.append((s, float((Qc * Qb).sum() / (np.linalg.norm(Qc) * nb)) if nb > 0 else 0.0))
    return Qc, out


# --------------------------------------------------------------------------
# tests
def test_e515_reversibility_with_a_flip():
    bits = [1, -1, 1, 1, 1, -1, 1, -1, 1, 1]
    err, s = run_with_flip(24, 10, bits, 0)
    assert err < 1e-9, err
    err_bad, _ = run_with_flip(24, 10, bits, 0, reverse_schedule=False)
    assert err_bad > 1e-3, err_bad
    # the flip is felt only through the next step: psi(n+1) differs by dt^2 c^2 rho (B - A) exactly, and psi(n) not at all
    A, B = source(bits), source([-bits[0]] + bits[1:])
    rho = density()
    sa = w.Sheet(np.zeros((N, N, N)), np.zeros((N, N, N)), rho, DT, A).step(12)
    sb = w.Sheet(sa.prev.copy(), sa.cur.copy(), rho, DT, B)
    sa.step(1); sb.step(1)
    assert np.array_equal(sa.prev, sb.prev)
    assert np.abs((sb.cur - sa.cur) - DT ** 2 * rho * (B - A)).max() < 1e-13
    return err, err_bad


def test_e516_hat_frozen_and_flow_strength():
    pos, tt = bit_sites()
    bits = [1] * 10
    for i in (3, 4, 5):                       # Hat tiles: the flip changes nothing
        b2 = list(bits)
        b2[i] = -1
        assert np.array_equal(source(bits), source(b2))
    b2 = list(bits)
    b2[0] = -1
    d = source(bits) - source(b2)
    assert abs(d[tuple(pos[0])] - 2 * math.sqrt(3)) < 1e-12 and abs(d.sum() - 2 * math.sqrt(3)) < 1e-12
    taus = np.linspace(0, 1, 21)
    hs = np.array([h(t) for t in taus])
    assert np.all(np.diff(hs) < 0) and abs(hs[-1]) < 1e-12
    return 2 * hs[0]


def test_e517_the_ensemble_is_not_neutral():
    m = monopoles([TAU_FULL] * 3 + [1.0] * 3 + [TAU_GROUP] * 4)
    assert abs(np.abs(m).min() - math.sqrt(3)) < 1e-12
    assert int((np.abs(np.abs(m) - math.sqrt(3)) < 1e-9).sum()) == 560
    assert np.all(np.abs(m) > 1.7)
    # the odd count: seven live bits of equal strength
    # a grid of group flow times: neutrality needs the groups to differ from the full tiles
    best = []
    for tg in np.linspace(0, 0.95, 20):
        mm = monopoles([TAU_FULL] * 3 + [1.0] * 3 + [tg] * 4)
        best.append(float(np.abs(mm).min()))
    assert min(best) < math.sqrt(3) - 1e-3        # some group flow time lowers the minimum
    assert abs(best[0] - math.sqrt(3)) < 1e-12
    # exact neutrality at isolated flow times: h_group = sqrt3/2 (tau = 0.3184) and sqrt3/4 (tau = 0.6098)
    for hg in (math.sqrt(3) / 2, math.sqrt(3) / 4):
        tg = 1.0 - 2.0 * math.degrees(math.atan(hg)) / OMEGA0
        assert abs(h(tg) - hg) < 1e-12
        assert np.abs(monopoles([TAU_FULL] * 3 + [1.0] * 3 + [tg] * 4)).min() < 1e-12
    return best[:1]


def test_e518_bits_against_the_chair_quadrupole():
    Qc, ov = overlaps()
    cmax = max(abs(c) for _, c in ov)
    assert abs(cmax - math.sqrt(2.0 / 3.0)) < 1e-4, cmax
    best = [s for s, c in ov if abs(abs(c) - cmax) < 1e-12]
    assert sorted(best) == [(-1, 1, -1, 1), (1, -1, 1, -1)]
    for s in best:                                   # no monopole, no dipole from the groups
        assert sum(s) == 0 and s[0] == s[2] and s[1] == s[3]
    for s in itertools.product((1, -1), repeat=4):   # no xz, yz, xy entries
        Qb = group_quadrupole(s)
        assert abs(Qb[0, 2]) + abs(Qb[1, 2]) + abs(Qb[0, 1]) < 1e-12
    Qb = group_quadrupole(best[0])
    k = -float((Qc * Qb).sum() / (Qb * Qb).sum())
    resid = float(np.linalg.norm(Qc + k * Qb) / np.linalg.norm(Qc))
    assert abs(abs(k) - 1.58) < 0.01 and abs(resid - 1.0 / math.sqrt(3.0)) < 2e-3, (k, resid)
    # the unreachable part is exactly the xz, yz entries of the chair's Q (a Qc-only residual, independent of kappa)
    assert abs(Qc[0, 2]) > 19 and abs(Qc[1, 2]) > 19
    return cmax, k, resid


def test_e519_far_field_of_a_flip():
    n = 64
    c = n // 2
    shape = (n, n, n)
    src = np.zeros(shape)
    q = 2 * math.sqrt(3)
    src[c, c, c] = q
    s = w.Sheet(np.zeros(shape), np.zeros(shape), 1.0, DT, src).step(40)
    acc = np.zeros(shape)
    for _ in range(60):
        acc += s.step().cur
    acc /= 60
    r = 8
    g = q / (4 * math.pi * r)
    got = float(acc[c + r, c, c])
    assert abs(got / g - 1.0) < 0.03, got / g
    return got / g


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

    fig, ax = plt.subplots(2, 2, figsize=(11, 8))
    taus = np.linspace(0, 1, 101)
    ax[0, 0].plot(taus, [2 * h(t) for t in taus], "k")
    ax[0, 0].axvline(1, color="tab:green", ls=":")
    ax[0, 0].set_xlabel("flow time tau"); ax[0, 0].set_ylabel("strength of a flip, 2 h(tau)")
    ax[0, 0].set_title("Hat end is silent")

    m = monopoles([TAU_FULL] * 3 + [1.0] * 3 + [TAU_GROUP] * 4)
    ax[0, 1].hist(m, bins=np.linspace(-14, 14, 57), color="0.4")
    ax[0, 1].axvline(-math.sqrt(3), color="r", ls=":"); ax[0, 1].axvline(math.sqrt(3), color="r", ls=":")
    ax[0, 1].set_xlabel("net bit monopole"); ax[0, 1].set_ylabel("patterns of 1024")
    ax[0, 1].set_title("no pattern is neutral at tau_group = 0")

    tg = np.linspace(0, 0.95, 96)
    mins = [np.abs(monopoles([TAU_FULL] * 3 + [1.0] * 3 + [t] * 4)).min() for t in tg]
    ax[1, 0].plot(tg, mins, "k")
    ax[1, 0].set_xlabel("flow time of the groups"); ax[1, 0].set_ylabel("min |net monopole| over 1024 patterns")

    Qc, ov = overlaps()
    ax[1, 1].bar(range(16), [c for _, c in ov], color="tab:blue")
    ax[1, 1].set_xlabel("pattern of the four group bits"); ax[1, 1].set_ylabel("cos(chair Q, bit Q)")
    ax[1, 1].set_title("overlap of the bit quadrupole with the chair's")
    fig.suptitle("toymodel_v5 Stage 3: ensemble as sources on the sheet")
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
