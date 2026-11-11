#!/usr/bin/env python3
"""tm2_waves.py - Stage 5 of PLAN.md: the smooth sheet S as a reversible wave field, what a snap radiates, and what a static source does to the bit's clock.

The field is the paper's 'coupled timeline wave equation' (quantgrav.tex, formal section) with the damping off:

        psi_tt / (c^2 rho(x)) - Lap psi = source(x, t)                       (W)

on a periodic lattice in 1, 2 or 3 dimensions (spacing 1), integrated by the leapfrog scheme.  The bit of the chain enters in two ways: a snap changes the area of
a tile by 2 a h, which is taken as a step in the source at one site; and the static field of a source sets the separation d of the bit's two states.
Nothing here is gravity.  'Clock' means the factor sqrt(1 - r_s / r) of the companion model (a verified property of that model, not of nature).

Definitions
  D5.1  Leapfrog: psi(n+1) = 2 psi(n) - psi(n-1) + dt^2 c^2 rho (Lap psi(n) + source).  Discrete energy
        E(n+1/2) = sum (psi(n+1) - psi(n))^2 / (2 dt^2 c^2 rho) + sum over bonds grad psi(n) . grad psi(n+1) / 2 - sum source (psi(n) + psi(n+1)) / 2  (static source).
  D5.2  A snap at site x0 is source(x, t) = q for t >= 0 at x = x0 (a step: the bit's area share changed and stays changed).  A pair snap is +q at x0 and -q at
        a neighbouring site (the bump-and-dent pair of Stage 0, which conserves area).
  D5.3  Static field G_D of a unit source in D dimensions: -Lap G = delta (minus the uniform background a periodic lattice needs).
        Continuum: G_1 = -|x| / 2, G_2 = -ln(r) / (2 pi), G_3 = 1 / (4 pi r).
  D5.4  Coupling rules (both **Definition**; choosing between them is the open question):
          rule I    d^2 = 4 sigma^2 r_s [4 pi G_3(r)]            = 4 sigma^2 r_s / r        (the squared jump is the field; 3 dimensions; zero far away)
          rule II   d^2 = 4 sigma^2 2 pi [G_2(r_s) - G_2(r)]     = 4 sigma^2 ln(r / r_s)    (the squared jump is the drop of the field from r_s; 2 dimensions)
          linear    d   = 2 sigma sqrt(r_s) [4 pi G_3(r)] ... d proportional to the field itself: d^2 = 4 sigma^2 (r_0 / r)^2.
        The overlap is c = exp(-d^2 / 8 sigma^2) as in the chain, so c^2 = exp(-d^2 / 4 sigma^2).

Exact claims (each a test; ``python3 tm2_waves.py --tests-only``)
  E5.1  The discrete energy D5.1 is conserved to 1e-12 (relative) for any rho(x) and in 1, 2 and 3 dimensions, without and with a static source.
  E5.2  Reversal: swapping the two last time levels and running the same number of steps returns the initial data to 1e-10, with a density step and with a static source.
  E5.3  Speed.  Equation (W) has wave speed c sqrt(rho): the wave is FASTER where the density is higher.  The measured group speed of a narrow pulse equals the
        discrete dispersion relation's (2e-3) for rho = 1 and rho = 4, and a pulse crossing a slab of rho = 4 arrives earlier than one that does not.
        [Negative for the sentence 'high-density regions induce phase lag' of the paper: with (W) and the clock 1 / rho, dense regions have slow clocks and fast waves.]
  E5.4  Reflection and transmission at a density step equal the discrete theory R = (e^{-i k2} - e^{-i k1}) / (e^{i k1} - e^{-i k2}), |R|^2 + (sin k2 / sin k1)(c2rho_1/c2rho_2)^0 ... see
        ``step_theory``: measured energy fractions agree to 2e-3 and sum to 1 to 1e-9.
  E5.5  Causality: after n steps a snap has changed nothing at lattice (Manhattan) distance greater than n; exactly zero.
  E5.6  A snap radiates and leaves the static field behind: in 3 dimensions the field behind the front settles to q G_3 (time average over the lattice's ringing: within 0.5% of the lattice Green function at r = 2..8), and the
        height of the front falls like 1 / r.  A pair snap has zero net source: its front is lower and falls faster, and its settled field relative to the single snap's falls from 0.16 at r = 2 to 0.006 at r = 8 (a dipole).
        Energy D5.1 (with the source term) is conserved to 1e-12 throughout, and the run retraces backwards: radiation is reversible here; the damping is what would make it one-way.
  E5.7  The static field decides the fall-off, and the dimension decides the static field: on the lattice, differences of G follow -|x| / 2 (1D, exact), -ln(r) / 2 pi
        (2D, 2%) and 1 / 4 pi r (3D, 2%; r G_3 flat to 1%).  **A 1 / r profile needs three dimensions.**
  E5.8  Clock identities (algebra of D5.4, each exact):
          rule II (2D):  c^2 = r_s / r exactly, so sqrt(1 - c^2) = sqrt(1 - r_s / r): the readable fraction of the bit IS the clock factor (reading A of Stage 3; C7 of
                         PLAN.md holds exactly).  The Hat (c = 1) is at r = r_s and the frozen end at infinity: the dense region is plastic.
          rule I (3D):   c^2 = exp(-r_s / r) = 1 - r_s / r + (r_s / r)^2 / 2 - ...: the overlap is the clock factor to first order (reading B), relative error of c
                         below 1% for r > 7.5 r_s and 10% at r = 2 r_s.  The frozen end is at r = 0, the Hat at infinity: the dense region is frozen.  There is no horizon:
                         c > 0 for every r > 0.
          linear:        1 - c^2 = (r_0 / r)^2 + ...: the deficit falls like 1 / r^2, the fall-off of the paper's density, not 1 / r.          [Negative]
          rule II in 3D: d^2 is bounded, c^2 -> exp(-const) at infinity: a floor on c.                                                              [Negative]
  E5.9  Plasticity per tick and per coordinate time (PLAN.md: 'must be separated').  Per tick the tunnelling angle is proportional to c.  Per coordinate time it is c times
        the clock factor: reading B gives c^2 = 1 - r_s / r (monotone: frozen near, plastic far); reading A gives c sqrt(1 - c^2), which vanishes at both ends and is largest
        at r = 2 r_s (value 1/2).  So under A the dense region is plastic per tick but frozen per coordinate time.

What Stage 5 says about the plan's failure conditions
  * 'the profile is not logarithmic': it is logarithmic in 2 dimensions (rule II) and then C7 is exact; in 3 dimensions it is not, and the identity is first order only.
  * 'the floor on c prevents the identity': it does for rule II in 3 dimensions.
  * 'dense regions come out plastic': yes under rule II / reading A (per tick), no under rule I / reading B.  The two rules differ by the dimension of the sheet and by
    what the squared jump is tied to (the field, or its drop).  Nothing here selects one.  Both need d^2, not d, to be linear in the field, which is an assumption.

Hypothesis / not done: a derivation of D5.4 (why the squared jump); the back-reaction of the field on the bit's dynamics (the snap is prescribed, not triggered);
rho depending on psi; the tiling as the lattice (a cubic lattice is used); the paper's damping and its update probability 1 / rho.

    python3 tm2_waves.py --tests-only
    python3 tm2_waves.py --story
    python3 tm2_waves.py --no-show --save figures/tm2_stage5_waves.png
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


# -- the field ------------------------------------------------------------------------------------------------
def laplacian(psi):
    out = -2.0 * psi.ndim * psi
    for ax in range(psi.ndim):
        out = out + np.roll(psi, 1, axis=ax) + np.roll(psi, -1, axis=ax)
    return out


class Sheet:
    """Leapfrog for (W) on a periodic lattice.  ``c2rho`` is c^2 rho(x) (array or number), ``source`` a static source array or None."""

    def __init__(self, psi0, psi1, c2rho=1.0, dt=0.5, source=None):
        self.prev, self.cur = np.array(psi0, dtype=float), np.array(psi1, dtype=float)
        if self.prev.shape != self.cur.shape:
            raise ValueError("psi0 and psi1 must have the same shape")
        self.c2rho = np.broadcast_to(np.asarray(c2rho, dtype=float), self.cur.shape).copy()
        if np.any(self.c2rho <= 0.0):
            raise ValueError("c^2 rho must be positive")
        if dt <= 0.0 or dt * dt * self.c2rho.max() * self.cur.ndim > 1.0 + 1e-12:
            raise ValueError("dt violates the stability bound dt^2 c^2 rho D <= 1")
        self.dt = float(dt)
        self.source = None if source is None else np.array(source, dtype=float)
        self.steps = 0

    def step(self, n=1):
        for _ in range(int(n)):
            rhs = laplacian(self.cur)
            if self.source is not None:
                rhs = rhs + self.source
            nxt = 2.0 * self.cur - self.prev + self.dt ** 2 * self.c2rho * rhs
            self.prev, self.cur = self.cur, nxt
            self.steps += 1
        return self

    def energy(self):
        v = (self.cur - self.prev) / self.dt
        e = 0.5 * float((v * v / self.c2rho).sum())
        for ax in range(self.cur.ndim):
            e += 0.5 * float(((np.roll(self.cur, -1, axis=ax) - self.cur) * (np.roll(self.prev, -1, axis=ax) - self.prev)).sum())
        if self.source is not None:
            e -= 0.5 * float((self.source * (self.cur + self.prev)).sum())
        return e

    def reverse(self):
        self.prev, self.cur = self.cur, self.prev
        return self


def pulse(n, x0, k, width, dt, speed):
    """Two time levels of a right-moving Gaussian wave packet of wavenumber k on a ring of n sites (the second level advanced by the discrete phase and group speed)."""
    x = np.arange(n, dtype=float)
    om = dispersion(k, dt, speed)
    vg = group_speed(k, dt, speed)

    def level(t):
        return np.exp(-0.5 * ((x - x0 - vg * t) / width) ** 2) * np.cos(k * (x - x0) - om * t)

    return level(0.0), level(dt)


def dispersion(k, dt, speed):
    """omega(k) of the leapfrog scheme: sin(omega dt / 2) = speed dt sin(k / 2)."""
    return 2.0 / dt * math.asin(speed * dt * math.sin(k / 2.0))


def group_speed(k, dt, speed):
    return speed * math.cos(k / 2.0) / math.sqrt(1.0 - (speed * dt * math.sin(k / 2.0)) ** 2)


def wavenumber(omega, dt, speed):
    """Inverse of ``dispersion``."""
    return 2.0 * math.asin(math.sin(omega * dt / 2.0) / (speed * dt))


def step_theory(k1, dt, speed1, speed2):
    """Reflected and transmitted energy fractions for a wave of wavenumber k1 going from speed1 to speed2 (uniform bonds, the step in c^2 rho)."""
    om = dispersion(k1, dt, speed1)
    k2 = wavenumber(om, dt, speed2)
    R = (np.exp(-1j * k2) - np.exp(-1j * k1)) / (np.exp(1j * k1) - np.exp(-1j * k2))
    r2 = abs(R) ** 2
    return r2, 1.0 - r2, k2


def centroid(e):
    x = np.arange(len(e), dtype=float)
    return float((x * e).sum() / e.sum())


def energy_density(sheet):
    v = (sheet.cur - sheet.prev) / sheet.dt
    g = 0.5 * (np.roll(sheet.cur, -1) - sheet.cur) * (np.roll(sheet.prev, -1) - sheet.prev)
    g = 0.5 * (g + np.roll(g, 1))
    return 0.5 * v * v / sheet.c2rho + g


# -- static fields ----------------------------------------------------------------------------------------------
def lattice_green(shape):
    """Solution of -Lap G = delta_0 - 1/V on the periodic lattice of the given shape (zero mean)."""
    src = np.zeros(shape)
    src[(0,) * len(shape)] = 1.0
    src -= 1.0 / src.size
    ks = np.meshgrid(*[2.0 * np.pi * np.fft.fftfreq(n) for n in shape], indexing="ij")
    lam = sum(4.0 * np.sin(k / 2.0) ** 2 for k in ks)
    lam[(0,) * len(shape)] = 1.0
    G = np.fft.ifftn(np.fft.fftn(src) / lam).real
    return G - G.mean()


def green_continuum(r, dim):
    if dim == 1:
        return -0.5 * abs(r)
    if dim == 2:
        return -math.log(r) / (2.0 * math.pi)
    if dim == 3:
        return 1.0 / (4.0 * math.pi * r)
    raise ValueError("dim must be 1, 2 or 3")


def background(r, shape):
    """The quadratic term the uniform compensating background adds to a periodic Green function: r^2 / (2 D V)."""
    return r * r / (2.0 * len(shape) * float(np.prod(shape)))


# -- D5.4: the coupling rules --------------------------------------------------------------------------------------
def c2_rule(r, rule, r_s=1.0, kappa=1.0):
    """c^2 at radius r (units free) under a coupling rule of D5.4."""
    if r <= 0.0:
        raise ValueError("r must be positive")
    if rule == "I":
        return math.exp(-r_s / r)
    if rule == "II":
        if r < r_s:
            raise ValueError("rule II is defined for r >= r_s")
        return r_s / r
    if rule == "linear":
        return math.exp(-(r_s / r) ** 2)
    if rule == "II-3D":
        if r < r_s:
            raise ValueError("rule II is defined for r >= r_s")
        return math.exp(-kappa * (1.0 - r_s / r))
    raise ValueError("unknown rule %r" % (rule,))


def clock(r, r_s=1.0):
    return math.sqrt(1.0 - r_s / r)


def plasticity(r, reading, r_s=1.0):
    """(per tick, per coordinate time) in units of the far or near maximum: per tick = c; per coordinate time = c x clock factor."""
    if reading == "A":
        c = math.sqrt(r_s / r)
    elif reading == "B":
        c = math.sqrt(1.0 - r_s / r)
    else:
        raise ValueError("reading must be 'A' or 'B'")
    return c, c * clock(r, r_s)


# -- snaps -----------------------------------------------------------------------------------------------------------
def snap_run(n=48, steps=40, dt=0.5, pair=False, q=1.0):
    """A snap (or a pair snap) at the centre of an n^3 lattice, from rest: the sheet after ``steps`` steps."""
    shape = (n, n, n)
    src = np.zeros(shape)
    c = n // 2
    src[c, c, c] = q
    if pair:
        src[c + 1, c, c] = -q
    return Sheet(np.zeros(shape), np.zeros(shape), 1.0, dt, src).step(steps)


def settled_average(n=64, t0=40, t1=100, dt=0.5, pair=False):
    """Time average of psi over steps t0..t1 after a snap (the lattice rings behind the front; the average is the settled field).
    The window must end before the front has gone round the periodic box (t1 dt < n - 8)."""
    shape = (n, n, n)
    src = np.zeros(shape)
    c = n // 2
    src[c, c, c] = 1.0
    if pair:
        src[c + 1, c, c] = -1.0
    s = Sheet(np.zeros(shape), np.zeros(shape), 1.0, dt, src).step(t0)
    acc = np.zeros(shape)
    for _ in range(t1 - t0):
        acc += s.step().cur
    return acc / (t1 - t0)


def green_infinite(r_list, n=96):
    """Lattice Green function of the infinite cubic lattice on the axis, from the periodic one: background removed, constant fixed at r = 30."""
    shape = (n, n, n)
    G = lattice_green(shape)
    g = lambda r: G[r, 0, 0] - background(r, shape)  # noqa: E731
    off = g(30) - green_continuum(30, 3)
    return [g(r) - off for r in r_list]


def front_height(n=64, dt=0.5, radii=(6, 9, 12, 15), pair=False):
    """Largest |psi| seen at distance r along an axis while the front of a snap passes (3D)."""
    shape = (n, n, n)
    src = np.zeros(shape)
    c = n // 2
    src[c, c, c] = 1.0
    if pair:
        src[c + 1, c, c] = -1.0
    s = Sheet(np.zeros(shape), np.zeros(shape), 1.0, dt, src)
    best = {r: 0.0 for r in radii}
    for _ in range(int((max(radii) + 6) / dt)):
        s.step()
        for r in radii:
            best[r] = max(best[r], abs(s.cur[c, c + r, c]))
    return best


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


def _random_sheet(shape, seed, dt, source=False):
    rng = np.random.default_rng(seed)
    p0 = rng.normal(size=shape)
    p1 = p0 + dt * rng.normal(size=shape) * 0.3
    rho = 1.0 + rng.uniform(0.0, 0.6 / len(shape), size=shape)
    return Sheet(p0, p1, rho, dt, rng.normal(size=shape) * 0.2 if source else None)


def test_e51_energy_is_conserved():
    for shape, dt in (((64,), 0.6), ((24, 24), 0.5), ((12, 12, 12), 0.4)):
        for source in (False, True):
            s = _random_sheet(shape, 3, dt, source)
            e0 = s.energy()
            es = [s.step(25).energy() for _ in range(8)]
            assert max(abs(e - e0) for e in es) <= 1e-12 * max(1.0, abs(e0)) * 50, (shape, source, e0, es)
    assert _raises(ValueError, Sheet, np.zeros(8), np.zeros(8), 4.0, 0.6) and _raises(ValueError, Sheet, np.zeros(8), np.zeros(9))
    assert _raises(ValueError, Sheet, np.zeros(8), np.zeros(8), -1.0, 0.1)


def test_e52_reversal_retraces():
    for shape, dt in (((64,), 0.6), ((16, 16), 0.5), ((10, 10, 10), 0.4)):
        for source in (False, True):
            s = _random_sheet(shape, 5, dt, source)
            p0, p1 = s.prev.copy(), s.cur.copy()
            s.step(200).reverse().step(200)
            assert np.abs(s.cur - p0).max() < 1e-10 and np.abs(s.prev - p1).max() < 1e-10, (shape, source)


def test_e53_the_wave_is_faster_where_the_density_is_higher():
    n, dt, k = 2048, 0.2, 0.5
    for c2rho in (1.0, 4.0):
        sp = math.sqrt(c2rho)
        p0, p1 = pulse(n, 300.0, k, 40.0, dt, sp)
        s = Sheet(p0, p1, c2rho, dt)
        x0 = centroid(energy_density(s))
        steps = int(600 / (sp * dt))
        s.step(steps)
        v = (centroid(energy_density(s)) - x0) / (steps * dt)
        assert math.isclose(v, group_speed(k, dt, sp), rel_tol=2e-3), (c2rho, v, group_speed(k, dt, sp))
        assert math.isclose(v, sp * math.cos(k / 2.0), rel_tol=2e-2), "close to c sqrt(rho) cos(k/2)"
    # a slab of rho = 4 between 500 and 700: the pulse that crosses it is ahead
    rho = np.ones(n)
    rho[500:700] = 4.0
    a, b = pulse(n, 300.0, 0.3, 40.0, dt, 1.0)
    free = Sheet(a, b, 1.0, dt).step(3000)
    slab = Sheet(a, b, rho, dt).step(3000)
    ef, es = energy_density(free), energy_density(slab)
    right = slice(700, n)
    lead = centroid(np.where(np.arange(n) >= 700, es, 0.0)) - centroid(ef)
    assert es[right].sum() > 0.3 * es.sum() and lead > 80.0, lead
    assert math.isclose(lead, 200.0 * (1.0 - group_speed(0.3, dt, 1.0) / group_speed(wavenumber(dispersion(0.3, dt, 1.0), dt, 2.0), dt, 2.0)), rel_tol=0.05)


def test_e54_reflection_and_transmission_at_a_density_step():
    n, dt = 4096, 0.2
    for k, c2 in ((0.5, 4.0), (0.8, 2.0), (0.4, 0.5)):
        rho = np.ones(n)
        rho[2000:] = c2
        p0, p1 = pulse(n, 1500.0, k, 60.0, dt, 1.0)
        s = Sheet(p0, p1, rho, dt)
        e0 = s.energy()
        s.step(int(1100 / dt * (1.0 if c2 >= 1 else 1.3)))
        e = energy_density(s)
        left, right = e[:2000].sum(), e[2000:].sum()
        r2, t2, k2 = step_theory(k, dt, 1.0, math.sqrt(c2))
        assert abs(left + right - e0) < 1e-9 * e0
        assert abs(left / e0 - r2) < 2e-3 and abs(right / e0 - t2) < 2e-3, (k, c2, left / e0, r2)
    r2, t2, k2 = step_theory(0.5, dt, 1.0, 1.0)
    assert r2 < 1e-28 and math.isclose(k2, 0.5, rel_tol=1e-12), "no step, no reflection"


def test_e55_causality_is_exact():
    n, steps = 40, 9
    s = snap_run(n, steps)
    c = n // 2
    idx = np.indices((n, n, n)) - c
    man = np.abs(idx).sum(axis=0)
    assert np.all(s.cur[man > steps] == 0.0) and np.any(s.cur[man == steps - 1] != 0.0)


def test_e56_a_snap_radiates_and_leaves_the_static_field():
    n = 64
    c = n // 2
    rs = (2, 3, 4, 5, 6, 8)
    avg = settled_average(n)
    Ginf = green_infinite(rs)
    for r, g in zip(rs, Ginf):
        assert math.isclose(avg[c, c + r, c], g, rel_tol=5e-3), (r, avg[c, c + r, c], g)
    src = np.zeros((48, 48, 48))
    src[24, 24, 24] = 1.0
    t = Sheet(np.zeros((48, 48, 48)), np.zeros((48, 48, 48)), 1.0, 0.5, src)
    e = [t.step(5).energy() for _ in range(8)]
    assert max(abs(x) for x in e) < 1e-12, "from rest with a static source the conserved energy stays 0: the field energy equals the work of the source"
    t.reverse().step(40)
    assert np.abs(t.cur).max() < 1e-10 and np.abs(t.prev).max() < 1e-10, "the radiation retraces: reversible"
    mono, di = front_height(), front_height(pair=True)
    rr = sorted(mono)
    slope_m = np.polyfit(np.log(rr), np.log([mono[r] for r in rr]), 1)[0]
    assert -1.25 < slope_m < -0.9, slope_m
    assert all(di[r] < 0.4 * mono[r] for r in rr) and np.all(np.diff([di[r] / mono[r] for r in rr]) < 0), "the pair's front is lower and falls faster"
    pavg = settled_average(n, pair=True)
    ratio = [pavg[c, c + r, c] / avg[c, c + r, c] for r in (2, 4, 6, 8)]
    assert np.all(np.diff(ratio) < 0) and ratio[0] < 0.2 and ratio[-1] < 0.01, "the pair's settled field falls faster: a dipole's"


def test_e57_the_dimension_decides_the_fall_off():
    G1 = lattice_green((512,))
    for r in (3, 10, 40):
        assert math.isclose(G1[r] - G1[1], -0.5 * (r - 1) + (r * r - 1) / (2.0 * 512), rel_tol=1e-10)
    shape2 = (256, 256)
    G2 = lattice_green(shape2)
    for r in (12, 24, 48):
        want = green_continuum(r, 2) - green_continuum(6, 2) + background(r, shape2) - background(6, shape2)
        assert math.isclose(G2[r, 0] - G2[6, 0], want, rel_tol=0.02), (r, G2[r, 0] - G2[6, 0], want)
    g8, g12, g16 = green_infinite((8, 12, 16))
    assert math.isclose(g12 - g8, green_continuum(12, 3) - green_continuum(8, 3), rel_tol=0.02)
    assert math.isclose(g16 - g8, green_continuum(16, 3) - green_continuum(8, 3), rel_tol=0.02)
    prod3 = [8 * g8, 12 * g12, 16 * g16]
    assert max(prod3) / min(prod3) < 1.01 and math.isclose(prod3[1], 1.0 / (4.0 * math.pi), rel_tol=0.01), "r G_3 is flat: 1/r"
    prod2 = [r * (G2[r, 0] - G2[6, 0]) for r in (12, 24, 48)]
    assert abs(prod2[2] / prod2[0]) > 5.0, "r G_2 is not"
    assert _raises(ValueError, green_continuum, 1.0, 4)


def test_e58_clock_identities():
    for r in (1.0, 1.5, 2.0, 7.0, 100.0):
        c2 = c2_rule(r, "II")
        assert math.isclose(math.sqrt(1.0 - c2), clock(r), rel_tol=1e-14, abs_tol=1e-15), "rule II: Helstrom = clock, exactly"
    assert c2_rule(1.0, "II") == 1.0 and c2_rule(1e12, "II") < 1e-11, "Hat at r_s, frozen end far away"
    for r, tol in ((2.0, 0.11), (7.5, 0.01), (50.0, 2.1e-4)):
        c = math.sqrt(c2_rule(r, "I"))
        assert abs(c / clock(r) - 1.0) < tol and c > clock(r), (r, c / clock(r) - 1.0)
    assert math.isclose(math.sqrt(c2_rule(2.0, "I")) / clock(2.0) - 1.0, 0.1014, abs_tol=5e-4)
    for x in (0.01, 0.05):
        assert math.isclose(c2_rule(1.0 / x, "I"), 1.0 - x + x * x / 2.0, abs_tol=x ** 3)
    assert c2_rule(1.0, "I") > 0.36 and c2_rule(0.05, "I") > 0.0 and c2_rule(1e9, "I") > 1.0 - 1e-8, "no horizon: c > 0 for every r > 0"
    assert np.all(np.diff([c2_rule(r, "I") for r in (0.1, 0.5, 1.0, 3.0, 30.0)]) > 0), "rule I: frozen near, plastic far"
    assert np.all(np.diff([c2_rule(r, "II") for r in (1.0, 2.0, 3.0, 30.0)]) < 0), "rule II: plastic near, frozen far"
    for r in (10.0, 100.0):
        assert math.isclose((1.0 - c2_rule(r, "linear")) * r * r, 1.0, rel_tol=1e-2), "linear coupling: deficit ~ 1/r^2"
        assert not math.isclose((1.0 - c2_rule(r, "linear")) * r, 1.0, rel_tol=0.5)
    assert math.isclose(c2_rule(1e9, "II-3D", kappa=2.0), math.exp(-2.0), rel_tol=1e-6), "rule II in 3D: a floor"
    assert _raises(ValueError, c2_rule, 0.5, "II") and _raises(ValueError, c2_rule, 1.0, "x") and _raises(ValueError, c2_rule, 0.0, "I")
    # the rules are what D5.4 says, on the lattice Green functions (2D: rule II against sqrt(r_s / r))
    shape2 = (256, 256)
    G2 = lattice_green(shape2)
    rs = 6
    for r in (12, 24, 48):
        d2_over_4s2 = 2.0 * math.pi * ((G2[rs, 0] - background(rs, shape2)) - (G2[r, 0] - background(r, shape2)))
        assert math.isclose(math.exp(-d2_over_4s2), rs / r, rel_tol=0.02), (r, math.exp(-d2_over_4s2), rs / r)


def test_e59_plasticity_per_tick_and_per_coordinate_time():
    rs = np.linspace(1.001, 60.0, 4000)
    a = np.array([plasticity(r, "A") for r in rs])
    b = np.array([plasticity(r, "B") for r in rs])
    assert np.all(np.diff(a[:, 0]) < 0) and np.all(np.diff(b[:, 0]) > 0) and np.all(np.diff(b[:, 1]) > 0)
    i = int(np.argmax(a[:, 1]))
    assert math.isclose(rs[i], 2.0, abs_tol=0.02) and math.isclose(a[i, 1], 0.5, abs_tol=1e-4)
    assert a[0, 1] < 0.04 and a[-1, 1] < 0.13, "reading A: frozen per coordinate time at both ends"
    for r in (1.5, 4.0, 30.0):
        assert math.isclose(plasticity(r, "B")[1], 1.0 - 1.0 / r, rel_tol=1e-12)
        assert math.isclose(plasticity(r, "A")[1], math.sqrt((1.0 / r) * (1.0 - 1.0 / r)), rel_tol=1e-12)
    assert _raises(ValueError, plasticity, 2.0, "C")


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
    dt = 0.2
    print("speed (group, k = 0.5): rho = 1: %.4f   rho = 4: %.4f   (c sqrt(rho) cos(k/2) = %.4f, %.4f)" % (group_speed(0.5, dt, 1.0), group_speed(0.5, dt, 2.0), math.cos(0.25), 2 * math.cos(0.25)))
    for k, c2 in ((0.5, 4.0), (0.8, 2.0), (0.4, 0.5)):
        r2, t2, k2 = step_theory(k, dt, 1.0, math.sqrt(c2))
        print("density step 1 -> %.1f, k = %.1f: reflected %.4f transmitted %.4f (k2 = %.4f)" % (c2, k, r2, t2, k2))
    avg, pavg = settled_average(), settled_average(pair=True)
    c = 32
    rs_ = (2, 3, 4, 5, 6, 8)
    print("snap, settled field / lattice Green function at r =", rs_, ":", " ".join("%.3f" % (avg[c, c + r, c] / g) for r, g in zip(rs_, green_infinite(rs_))))
    print("pair snap, settled field / single snap's:", " ".join("%.4f" % (pavg[c, c + r, c] / avg[c, c + r, c]) for r in rs_))
    mono, di = front_height(), front_height(pair=True)
    rs = sorted(mono)
    print("front height at r =", rs, ": snap", " ".join("%.4f" % mono[r] for r in rs), "| pair", " ".join("%.4f" % di[r] for r in rs))
    print("  log-log slopes: snap %.2f, pair %.2f" % (np.polyfit(np.log(rs), np.log([mono[r] for r in rs]), 1)[0], np.polyfit(np.log(rs), np.log([di[r] for r in rs]), 1)[0]))
    print("clock: r/r_s | clock sqrt(1 - r_s/r) | rule II (2D): sqrt(1 - c^2) | rule I (3D): c | linear: c")
    for r in (1.0, 1.5, 2.0, 4.0, 7.5, 20.0, 100.0):
        print("  %6.1f | %.4f | %.4f | %.4f | %.4f" % (r, clock(r), math.sqrt(1 - c2_rule(r, "II")), math.sqrt(c2_rule(r, "I")), math.sqrt(c2_rule(r, "linear"))))
    print("plasticity per coordinate time: reading A max at r = 2 r_s (0.5); reading B = 1 - r_s/r")


def figure(save=None, show=True):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 4, figsize=(18, 4.2))
    n, dt = 2048, 0.2
    rho = np.ones(n)
    rho[500:700] = 4.0
    a, b = pulse(n, 300.0, 0.3, 40.0, dt, 1.0)
    free = Sheet(a, b, 1.0, dt).step(3000)
    slab = Sheet(a, b, rho, dt).step(3000)
    ax[0].plot(energy_density(free), label="rho = 1")
    ax[0].plot(energy_density(slab), label="slab rho = 4 at 500..700")
    ax[0].axvspan(500, 700, color="0.9")
    ax[0].set_xlim(0, 1400)
    ax[0].set_xlabel("x")
    ax[0].set_ylabel("energy density at t = 600")
    ax[0].set_title("dense = faster (equation W)")
    ax[0].legend(fontsize=8)
    s = settled_average()
    c = 32
    r = np.arange(1, 20)
    ax[1].loglog(r, [abs(s[c, c + x, c]) for x in r], "o-", label="snap, time average")
    p = settled_average(pair=True)
    ax[1].loglog(r, [abs(p[c, c + x, c]) + 1e-12 for x in r], "s-", label="pair snap")
    ax[1].loglog(r, 1.0 / (4 * np.pi * r), "k--", label="1 / 4 pi r")
    ax[1].set_xlabel("r")
    ax[1].set_ylabel("|psi|")
    ax[1].set_title("a snap leaves the static field behind")
    ax[1].legend(fontsize=8)
    rr = np.linspace(1.0, 20.0, 400)
    ax[2].plot(rr, [clock(x) for x in rr], "k", lw=2, label="clock sqrt(1 - r_s/r)")
    ax[2].plot(rr, [math.sqrt(1 - c2_rule(x, "II")) for x in rr], "--", label="rule II (2D): sqrt(1 - c^2)")
    ax[2].plot(rr, [math.sqrt(c2_rule(x, "I")) for x in rr], label="rule I (3D): c")
    ax[2].plot(rr, [math.sqrt(c2_rule(x, "linear")) for x in rr], ":", label="linear coupling: c")
    ax[2].set_xlabel("r / r_s")
    ax[2].set_title("the clock from a static source")
    ax[2].legend(fontsize=8)
    ax[3].plot(rr[1:], [plasticity(x, "A")[0] for x in rr[1:]], "C0--", label="A per tick")
    ax[3].plot(rr[1:], [plasticity(x, "A")[1] for x in rr[1:]], "C0", label="A per coordinate time")
    ax[3].plot(rr[1:], [plasticity(x, "B")[0] for x in rr[1:]], "C1--", label="B per tick")
    ax[3].plot(rr[1:], [plasticity(x, "B")[1] for x in rr[1:]], "C1", label="B per coordinate time")
    ax[3].set_xlabel("r / r_s")
    ax[3].set_title("plasticity")
    ax[3].legend(fontsize=8)
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
