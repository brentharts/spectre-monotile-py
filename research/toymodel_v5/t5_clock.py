#!/usr/bin/env python3
"""t5_clock.py - Stage 5 of toymodel_v5: the dynamics and the clock of toymodel_v2 (Stage 5, C7) with the chair, and then the ensemble, as the source.

v2 Stage 5 (rule I, reading B): the squared jump of the bit follows the static field, d^2 = 4 sigma^2 g |phi|, the overlap is c^2 = exp(-d^2 / 4 sigma^2) = exp(-g |phi|), and the identity C7 asks
that c be the clock sqrt(1 - r_s / r) with r_s = g * (monopole): needs |phi| = r_s / r to first order.  Here phi is the Coulomb potential sum q / r (chair units; lattice units for the dynamics).
The modulus is a Definition (a squared jump cannot be negative; the chair's potential changes sign).  Sources: (A) the chair alone, (B) chair plus the ten bits of Stage 3 with the cancelling pattern
(full tiles +,+,-; groups +,-,+,-: net bit monopole sqrt3, no group dipole, Stage 3 E5.18), the Hat bits silent.

Exact / Numeric claims (each a test; ``python3 t5_clock.py --tests-only``)
  E5.24  The dynamics settle to the static field.  The chair switched on at t = 0 on the 64^3 sheet (L = 3 sites per chair unit) has a time-averaged field behind the front equal to the static
         solution of the periodic Poisson equation (FFT) within 5% (rms over the shell r = 7..14 sites).  Energy including the source work is conserved and the run retraces (v2 E5.1, E5.2 with this source).
  E5.25  C7 fails for the chair alone.  The angular rms of |phi| falls as r^-3 (slope -3.00 on r = 20..200 chair units, to 0.05), not r^-1: the profile -ln c^2 = g |phi| is the quadrupole's, so
         c^2 = exp(-g |phi|) cannot be exp(-r_s/r), and the monopole r_s = g sum q = 0.  Along the nodal cones of the saddle |phi| = 0 and c = 1 at every radius.  The identity needs 1/r.  **Negative**.
  E5.26  The ensemble restores the 1/r, from the bits and not from the chair.  With pattern B (monopole sqrt3 = r_s / g) the rms slope is -1.00000 far away (r = 1500); the angular mean of r |phi| is within 1%
         of sqrt3 from r_1 = 8.4 chair units outward (computed in the test).  There c agrees with the clock sqrt(1 - r_s / r) to 0.9% at r = 10, 0.2% at 20, 0.05% at 40: first order, as in v2.  Close in
         (r = 3 to 5) the chair's quadrupole and the bits' dipole dominate and c is 31% / 1.6% below the clock.  The 1/r that C7 needs is carried by the unpaired live bit (E5.17), a Hypothesis-level coupling.
  E5.27  Causality and speed on the ensemble: with the ten bits and the chair switched on at step 0, the field is exactly zero beyond lattice distance n of every source after n steps (n = 12), and
         nonzero within it; a probe 6 sites above a full tile sees the front at step 8 through a dense line (rho = 1.5) and at step 9 in vacuum (v2 E5.3: dense = faster, on this geometry).

Status.  Algebra of the profile: Exact.  The settled-field and front checks: Numeric / Exact (support).  No horizon: |phi| is finite everywhere away from the sources, so c < 1 only on a set of directions and has a floor
nowhere; the readings A and B of v2 are not decided by this stage.

    python3 t5_clock.py --tests-only
    python3 t5_clock.py --no-show --save figures/t5_stage5_clock.png
"""
import argparse
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
import t5_source as ts  # noqa: E402
import tm2_waves as w  # noqa: E402

PATTERN_B = [1, 1, -1, 1, 1, 1, 1, -1, 1, -1]     # 3 full, 3 Hat (silent), 4 groups
M_B = math.sqrt(3.0)


def directions(n=300):
    i = np.arange(n) + 0.5
    phi = math.pi * (1 + 5 ** 0.5) * i
    z = 1 - 2 * i / n
    r = np.sqrt(1 - z * z)
    return np.column_stack([r * np.cos(phi), r * np.sin(phi), z])


def sources_chair_units(bits=None):
    """(positions, charges) in chair units about the chair's centre: chair tips, and (if bits) the live bits."""
    d = np.array([[float(x) for x in v] for v in ts.displacements()])
    q = np.array(ts.charges(), float)
    if bits is None:
        return d, q
    pos, taus = te.bit_sites()
    st = te.strengths(taus)
    P = (pos - te.centre()) / te.L
    return np.vstack([d, P]), np.concatenate([q, st * np.array(bits)])


def phi_at(P, q, X):
    """Potential sum q / |x - p| at points X (n, 3), summed over sources."""
    out = np.zeros(len(X))
    for p, qq in zip(P, q):
        out += qq / np.linalg.norm(X - p, axis=1)
    return out


def rms_profile(P, q, radii, n=300):
    D = directions(n)
    return np.array([math.sqrt(float(np.mean(phi_at(P, q, r * D) ** 2))) for r in radii])


def slope(radii, y):
    return float(np.polyfit(np.log(radii), np.log(y), 1)[0])


def static_field(src):
    """Periodic Poisson solution of -Lap phi = src - mean(src), by FFT."""
    n = src.shape[0]
    k = 2 * np.pi * np.fft.fftfreq(n)
    k2 = sum(np.meshgrid(*[(2 - 2 * np.cos(k))] * 3, indexing="ij"))
    f = np.fft.fftn(src)
    k2[0, 0, 0] = 1.0
    f = f / k2
    f[0, 0, 0] = 0.0
    return np.real(np.fft.ifftn(f))


def chair_lattice_source(n):
    s, q = te.chair_sites()
    src = np.zeros((n, n, n))
    c = n // 2
    for (i, j, k), qq in zip(s - te.centre() + c, q):
        src[i, j, k] += qq
    return src


def shell_mask(n, r0, r1):
    c = n // 2
    g = np.indices((n, n, n)) - c
    r = np.sqrt((g ** 2).sum(axis=0))
    return (r >= r0) & (r <= r1)


# --------------------------------------------------------------------------
def test_e524_the_dynamics_settle_to_the_static_field():
    n = 64
    src = chair_lattice_source(n)
    s = w.Sheet(np.zeros((n, n, n)), np.zeros((n, n, n)), 1.0, 0.4, src)
    s.step(50)
    e0 = s.energy()
    acc = np.zeros((n, n, n))
    m = 60
    for _ in range(m):
        s.step(1)
        acc += s.cur
        assert abs(s.energy() - e0) <= 1e-10 * max(1.0, abs(e0))
    acc /= m
    stat = static_field(src)
    mask = shell_mask(n, 7, 14)
    err = float(np.sqrt(np.mean((acc[mask] - stat[mask]) ** 2)) / np.sqrt(np.mean(stat[mask] ** 2)))
    assert err < 0.05, err
    # retrace with this source
    p = w.Sheet(np.zeros((n, n, n)), np.zeros((n, n, n)), 1.0, 0.4, src).step(20)
    start0, start1 = np.zeros((n, n, n)), np.zeros((n, n, n))
    p.reverse().step(20)
    assert max(np.abs(p.cur - start0).max(), np.abs(p.prev - start1).max()) < 1e-10
    return err


def test_e525_c7_fails_for_the_chair_alone():
    P, q = sources_chair_units()
    assert abs(q.sum()) < 1e-12
    radii = np.geomspace(20, 200, 10)
    y = rms_profile(P, q, radii)
    sl = slope(radii, y)
    assert abs(sl + 3.0) < 0.05, sl
    # nodal cones: the potential changes sign over the directions at a fixed large radius
    v = phi_at(P, q, 60 * directions(300))
    assert v.min() < 0 < v.max()
    return sl


def test_e526_the_ensemble_restores_one_over_r_from_the_bits():
    P, q = sources_chair_units(PATTERN_B)
    assert abs(q.sum() - M_B) < 1e-9
    radii = np.geomspace(1.5, 1500, 40)
    y = rms_profile(P, q, radii, n=200)
    local = np.diff(np.log(y)) / np.diff(np.log(radii))
    assert local[0] < -2.0 and abs(local[-1] + 1.0) < 1e-3, (local[0], local[-1])
    D = directions(200)
    grid = np.geomspace(3, 200, 200)
    ratio = np.array([float(np.mean(np.abs(phi_at(P, q, r * D))) * r) for r in grid])
    off = np.where(np.abs(ratio / M_B - 1) > 0.01)[0]
    r1 = float(grid[off[-1] + 1])
    assert 8 < r1 < 15, r1

    def rel(r):
        c = float(np.mean(np.sqrt(np.exp(-np.abs(phi_at(P, q, r * D))))))
        return c / math.sqrt(1 - M_B / r) - 1

    d = {r: rel(r) for r in (3, 5, 10, 20, 40, 100)}
    assert d[3] < -0.2 and -0.03 < d[5] < -0.005
    assert abs(d[10]) < 0.012 and abs(d[20]) < 0.003 and abs(d[40]) < 0.001
    assert abs(d[100]) < abs(d[40]) < abs(d[20])
    return r1, {k: round(v, 5) for k, v in d.items()}


def test_e527_causality_and_speed_on_the_ensemble():
    n = te.N
    src = te.source(PATTERN_B)
    rho = te.density()
    s = w.Sheet(np.zeros((n, n, n)), np.zeros((n, n, n)), rho, te.DT, src)
    steps = 12
    s.step(steps)
    g = np.indices((n, n, n))
    c = te.centre()
    pos, _ = te.bit_sites()
    sites, _ = te.chair_sites()
    allsrc = np.vstack([sites, pos])
    man = np.min([np.abs(g[0] - p[0]) + np.abs(g[1] - p[1]) + np.abs(g[2] - p[2]) for p in allsrc], axis=0)
    # changes beyond Manhattan distance steps (of the nearest source): exactly zero  (periodic: use the wrapped distance)
    nn = np.min([sum(np.minimum(np.abs(g[a] - p[a]), n - np.abs(g[a] - p[a])) for a in range(3)) for p in allsrc], axis=0)
    assert np.all(s.cur[nn > steps] == 0.0) and np.any(s.cur[nn <= steps] != 0.0)
    # speed: arrival step of a probe vs distance, in a dense and a vacuum run
    def arrival(rho_arr, probe):
        sh = w.Sheet(np.zeros((n, n, n)), np.zeros((n, n, n)), rho_arr, te.DT, src)
        for t in range(1, 40):
            sh.step(1)
            if abs(sh.cur[tuple(probe)]) > 1e-4:
                return t
        return 40
    probe = pos[0] + np.array([0, 0, 6])
    # dense slab through the probe's line: rho = 1.5 along z above the first full tile
    dense = np.ones((n, n, n))
    dense[tuple(pos[0][:2])] = 1.5
    dense[pos[0][0], pos[0][1], pos[0][2]:probe[2] + 1] = 1.5
    a_d, a_v = arrival(dense, probe), arrival(np.ones((n, n, n)), probe)
    assert a_d <= a_v, (a_d, a_v)
    return a_d, a_v


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

    radii = np.geomspace(1.5, 1500, 40)
    Pc, qc = sources_chair_units()
    Pb, qb = sources_chair_units(PATTERN_B)
    yc, yb = rms_profile(Pc, qc, radii, 150), rms_profile(Pb, qb, radii, 150)
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    ax[0].loglog(radii, yc, "b", label="chair alone"); ax[0].loglog(radii, yb, "r", label="chair + bits (B)")
    ax[0].loglog(radii, M_B / radii, "k:", label="r$^{-1}$"); ax[0].loglog(radii, 60 * radii ** -3.0, "k--", label="r$^{-3}$")
    ax[0].set_xlabel("r (chair units)"); ax[0].set_ylabel("rms |phi|"); ax[0].legend(fontsize=7); ax[0].set_title("profile: 1/r only from the bits")
    r2 = np.geomspace(3, 400, 60)
    D = directions(150)
    c_b = [math.sqrt(np.mean(np.exp(-np.abs(phi_at(Pb, qb, r * D))))) for r in r2]
    c_c = [math.sqrt(np.mean(np.exp(-np.abs(phi_at(Pc, qc, r * D))))) for r in r2]
    ax[1].semilogx(r2, c_b, "r", label="c, ensemble B"); ax[1].semilogx(r2, c_c, "b", label="c, chair alone")
    ax[1].semilogx(r2, np.sqrt(np.clip(1 - M_B / r2, 0, None)), "k--", label="clock sqrt(1 - r_s/r)")
    ax[1].set_xlabel("r (chair units)"); ax[1].set_ylabel("c (angular mean)"); ax[1].legend(fontsize=7); ax[1].set_title("C7: only first order")
    v = phi_at(Pc, qc, 40 * directions(2000))
    ax[2].hist(v, bins=60, color="0.4"); ax[2].set_xlabel("phi on the sphere r = 40"); ax[2].set_title("chair: sign-changing (nodal cones)")
    fig.suptitle("toymodel_v5 Stage 5: the clock with the chair and the ensemble as source")
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
