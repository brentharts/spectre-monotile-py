"""
spectre_curvature.py - curvature, turning function and bending energy of the curved
Spectre monotile (step 3).

Each edge is the cubic Bezier B(t) = P + s(t) d + k w(t) n with d the edge vector, n = (d_y, -d_x)
(same length, perpendicular), s = t (3 - 3t + 2 t^2) / 2 and w = 3 t (1 - t) (2t - 1), see
spectre_operator.  Everything below follows in closed form:

    B' = s' d + k w' n,   s' = 3/2 - 3t + 3t^2,   w' = -18 t^2 + 18 t - 3,
    s' w'' - w' s'' = 18 (1 - 2t)                    (the cubic terms cancel!)

    signed curvature   kappa(t) = -18 k (1 - 2t) / (|d| (s'^2 + k^2 w'^2)^(3/2))

(counterclockwise turning is positive; the tile itself is traversed clockwise).  Consequences,
all checked by the self tests:

  * kappa is odd about the middle of the edge: one inflection point per edge, at t = 1/2, and
    the integral of kappa ds over an edge is ZERO (an S bends back as much as it bends forward);
  * the absolute curvature integrates to exactly  int |kappa| ds = 4 arctan(2 |k|)  per edge, since
    the tangent turns by 2 arctan(2|k|) from either end to the middle;
  * the corners carry the rest: turn_i = polygon turn + arctan(2 k_i) - arctan(2 k_(i-1)), and the
    Umlaufsatz (Hopf) holds:  sum of all turns + sum of int kappa ds = -2 pi;
  * total absolute curvature  sum |turn_i| + sum 4 arctan(2|k_i|)  is 2 pi only for a convex curve;
    the tile is far from convex (it has reflex corners), so it is much bigger;
  * lengths scale as s, so kappa as 1/s and the bending energy  int kappa^2 ds  as 1/s, while the
    turning angles do not change at all.

    from spectre_curvature import curvature_profile, total_absolute_curvature, bending_energy
    prof = curvature_profile(a=1, b=1, curves=1.0, scale=0.5)   # arrays along the whole outline
    prof["kappa"], prof["theta"], prof["corner_turn"]
    bending_energy(1, 1, 1.0, 0.5)

Run ``python3 spectre_curvature.py`` to execute the self tests and show the figure
(--help for options).  Needs numpy, scipy and matplotlib.
"""

import argparse
import math
import os
import sys

import numpy as np

import spectre_operator as so

__all__ = [
    "edge_speed",
    "edge_curvature",
    "curvature_profile",
    "corner_turn_angles",
    "total_absolute_curvature",
    "bending_energy",
    "max_curvature",
    "curvature_comb",
    "run_tests",
    "show_curvature",
]

_NODES, _WEIGHTS = np.polynomial.legendre.leggauss(64)
_T_GL, _W_GL = 0.5 * (_NODES + 1.0), 0.5 * _WEIGHTS


def _check_t(t):
    t = np.asarray(t, dtype=float)
    if t.ndim != 1 or t.size == 0 or not np.all(np.isfinite(t)) or t.min() < 0.0 or t.max() > 1.0:
        raise ValueError("t must be a non-empty 1-D array of numbers in [0, 1]")
    return t


def _sw(t):
    """s', w' and the quantity s' w'' - w' s'' = 18 (1 - 2t) of the edge parametrisation."""
    sp = 1.5 - 3.0 * t + 3.0 * t**2
    wp = -18.0 * t**2 + 18.0 * t - 3.0
    return sp, wp


# --------------------------------------------------------------------------
# Curvature of the edges
# --------------------------------------------------------------------------
def edge_speed(t, a=1.0, b=1.0, curves=1.0, scale=1.0):
    """|B'(t)| of every edge: shape (14, len(t))."""
    t = _check_t(t)
    k = so.edge_strengths(curves, scale)[:, None]
    length = np.hypot(*so.edge_vectors(a, b).T)[:, None]
    sp, wp = _sw(t)
    return length * np.sqrt(sp[None] ** 2 + (k * wp[None]) ** 2)


def edge_curvature(t, a=1.0, b=1.0, curves=1.0, scale=1.0):
    """Signed curvature kappa(t) of every edge: shape (14, len(t)).

    Counterclockwise turning is positive.  Straight edges (k = 0) have kappa = 0."""
    t = _check_t(t)
    k = so.edge_strengths(curves, scale)[:, None]
    length = np.hypot(*so.edge_vectors(a, b).T)[:, None]
    sp, wp = _sw(t)
    return -18.0 * k * (1.0 - 2.0 * t)[None] / (length * (sp[None] ** 2 + (k * wp[None]) ** 2) ** 1.5)


def corner_turn_angles(a=1.0, b=1.0, curves=1.0, scale=1.0):
    """Turn of the outline at each of the 14 vertices, measured from the tangent directions of
    the two edges that meet there (counterclockwise positive, in (-pi, pi]).  Independent of the
    closed formula in spectre_fields.corner_turns, which it must reproduce."""
    d = so.edge_vectors(a, b)
    k = so.edge_strengths(curves, scale)
    n = np.stack([d[:, 1], -d[:, 0]], axis=1)
    # B'(0) = B'(1) = 3/2 d - 3 k n: both ends of an edge have the same tangent direction
    tan_start = 1.5 * d - 3.0 * k[:, None] * n
    tan_end = tan_start
    prev = np.roll(tan_end, 1, axis=0)  # leaving edge i - 1
    cross = prev[:, 0] * tan_start[:, 1] - prev[:, 1] * tan_start[:, 0]
    return np.arctan2(cross, (prev * tan_start).sum(axis=1))


def curvature_profile(a=1.0, b=1.0, curves=1.0, scale=1.0, per_edge=400):
    """Curvature and tangent angle along the whole outline.

    Returns a dict with
      s            arclength, shape (14 * per_edge + 1,), from vertex 0 around to vertex 0 again
      kappa        signed curvature at those points (the corners are NOT included: they carry the
                   turns below as Dirac masses)
      theta        tangent angle, continuous along the edges and jumping by the corner turn at
                   each vertex (the jump is applied to the first sample of the next edge)
      edge         edge index of each sample
      point        (n, 2) points of the outline
      corner_s     arclength of each of the 14 vertices
      corner_turn  turn at each vertex (see corner_turn_angles)
      length       total length (the perimeter)
    The samples are an equally spaced grid in t (meant for plotting and simple sums; the integral
    quantities below are computed in closed form or by Gauss-Legendre)."""
    n = int(per_edge)
    if n != per_edge or n < 8:
        raise ValueError("per_edge must be an integer >= 8")
    t = np.linspace(0.0, 1.0, n + 1)
    k = so.edge_strengths(curves, scale)
    d = so.edge_vectors(a, b)
    kappa = edge_curvature(t, a, b, curves, scale)
    ctrl = so.edge_beziers(a, b, curves, scale)
    pts = so.evaluate_edges(ctrl, t)  # (14, n+1, 2)
    # arclength of each edge: cumulative trapezoid of the speed on a fine Gauss grid is overkill;
    # integrate exactly per sub-interval with 8-point Gauss-Legendre
    g, gw = np.polynomial.legendre.leggauss(8)
    tl, tr = t[:-1], t[1:]
    tm, half = 0.5 * (tl + tr), 0.5 * (tr - tl)
    tt = tm[:, None] + half[:, None] * g[None]
    sp, wp = _sw(tt.ravel())
    sp, wp = sp.reshape(tt.shape), wp.reshape(tt.shape)
    length = np.hypot(*d.T)
    v = length[:, None, None] * np.sqrt(sp[None] ** 2 + (k[:, None, None] * wp[None]) ** 2)  # (14, n, 8)
    seg = (v * gw[None, None] * half[None, :, None]).sum(axis=2)  # (14, n)
    edge_len = seg.sum(axis=1)
    turns = corner_turn_angles(a, b, curves, scale)

    s_edge = np.concatenate([np.zeros((so.N_EDGES, 1)), np.cumsum(seg, axis=1)], axis=1)
    offsets = np.concatenate([[0.0], np.cumsum(edge_len)])[:-1]
    s_all = (s_edge + offsets[:, None])

    # tangent angle: continuous within an edge, chained through the corner turns
    sp1, wp1 = _sw(t)
    nvec = np.stack([d[:, 1], -d[:, 0]], axis=1)
    vel = sp1[None, :, None] * d[:, None, :] + (k[:, None] * wp1[None, :])[:, :, None] * nvec[:, None, :]
    ang = np.arctan2(vel[..., 1], vel[..., 0])
    ang = np.unwrap(ang, axis=1)
    theta = np.empty_like(ang)
    theta[0] = ang[0]
    for i in range(1, so.N_EDGES):
        start = theta[i - 1, -1] + turns[i]  # start of edge i = end of edge i - 1 plus the turn
        theta[i] = ang[i] - ang[i, 0] + start
    theta_all, corner_s = theta, offsets

    keep = slice(0, n)  # drop the last sample of each edge (= first of the next), add the very end later
    s_out = np.concatenate([s_all[:, keep].ravel(), [s_all[-1, -1]]])
    kappa_out = np.concatenate([kappa[:, keep].ravel(), [kappa[-1, -1]]])
    theta_out = np.concatenate([theta_all[:, keep].ravel(), [theta_all[-1, -1]]])
    point_out = np.vstack([pts[:, keep].reshape(-1, 2), pts[-1, -1][None]])
    edge_out = np.concatenate([np.repeat(np.arange(so.N_EDGES), n), [so.N_EDGES - 1]])
    return dict(
        s=s_out,
        kappa=kappa_out,
        theta=theta_out,
        edge=edge_out,
        point=point_out,
        corner_s=corner_s,
        corner_turn=turns,
        length=float(edge_len.sum()),
        edge_length=edge_len,
    )


# --------------------------------------------------------------------------
# Integral quantities
# --------------------------------------------------------------------------
def total_absolute_curvature(a=1.0, b=1.0, curves=1.0, scale=1.0):
    """Total absolute curvature  sum_i |turn_i| + sum_i 4 arctan(2 |k_i|)  (closed form).

    It equals 2 pi for a convex outline and is larger otherwise (Fenchel).  It does not depend
    on a and b."""
    k = so.edge_strengths(curves, scale)
    return float(np.abs(corner_turn_angles(a, b, curves, scale)).sum() + 4.0 * np.arctan(2.0 * np.abs(k)).sum())


def bending_energy(a=1.0, b=1.0, curves=1.0, scale=1.0):
    """Bending energy  int kappa^2 ds  over the smooth parts (the corners have infinite energy).

    Per edge it is  (1 / |d|) * 324 k^2 int_0^1 (1 - 2t)^2 / (s'^2 + k^2 w'^2)^(5/2) dt,
    evaluated by Gauss-Legendre quadrature."""
    k = so.edge_strengths(curves, scale)
    length = np.hypot(*so.edge_vectors(a, b).T)
    sp, wp = _sw(_T_GL)
    integrand = 324.0 * k[:, None] ** 2 * (1.0 - 2.0 * _T_GL)[None] ** 2 / (sp[None] ** 2 + (k[:, None] * wp[None]) ** 2) ** 2.5
    return float(((integrand @ _W_GL) / length).sum())


def max_curvature(a=1.0, b=1.0, curves=1.0, scale=1.0):
    """Largest |kappa| over the smooth parts of the outline, and the edge and parameter where
    it occurs: (kappa_max, edge, t)."""
    t = np.linspace(0.0, 1.0, 20001)
    kap = np.abs(edge_curvature(t, a, b, curves, scale))
    i, j = np.unravel_index(np.argmax(kap), kap.shape)
    # refine with a parabola through the three neighbouring samples
    if 0 < j < len(t) - 1:
        y0, y1, y2 = kap[i, j - 1], kap[i, j], kap[i, j + 1]
        denom = y0 - 2.0 * y1 + y2
        shift = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
        t_star = float(t[j] + shift * (t[1] - t[0]))
        val = float(np.abs(edge_curvature(np.array([t_star]), a, b, curves, scale))[i, 0])
        return val, int(i), t_star
    return float(kap[i, j]), int(i), float(t[j])


def curvature_comb(a=1.0, b=1.0, curves=1.0, scale=1.0, factor=0.15, per_edge=60):
    """Curvature comb: for sample points p of the outline, the hair from p to p + factor * kappa * n
    with n the left normal of the tangent (so hairs point towards the centre of curvature and
    their length is proportional to |kappa|).  Returns (base, tip, kappa), each of shape
    (14, per_edge + 1, ...)."""
    if not np.isfinite(factor):
        raise ValueError("factor must be finite")
    t = np.linspace(0.0, 1.0, int(per_edge) + 1)
    ctrl = so.edge_beziers(a, b, curves, scale)
    base = so.evaluate_edges(ctrl, t)
    d = so.edge_vectors(a, b)
    k = so.edge_strengths(curves, scale)
    nvec = np.stack([d[:, 1], -d[:, 0]], axis=1)
    sp, wp = _sw(t)
    vel = sp[None, :, None] * d[:, None, :] + (k[:, None] * wp[None, :])[:, :, None] * nvec[:, None, :]
    speed = np.hypot(vel[..., 0], vel[..., 1])
    left = np.stack([-vel[..., 1], vel[..., 0]], axis=-1) / speed[..., None]
    kap = edge_curvature(t, a, b, curves, scale)
    tip = base + factor * kap[..., None] * left
    return base, tip, kap


# --------------------------------------------------------------------------
# Self tests
# --------------------------------------------------------------------------
def _rel(x, ref):
    return abs(x - ref) / abs(ref)


def _quad_edge(fun, lo=0.0, hi=1.0):
    from scipy.integrate import quad

    return quad(fun, lo, hi, epsabs=1e-13, epsrel=1e-13, limit=200)[0]


def test_curvature_matches_finite_differences_of_the_bezier_edges():
    curves, scale = np.random.default_rng(2).uniform(-1, 1, 14), 0.6
    t = np.linspace(0.0, 1.0, 20001)
    pts = so.evaluate_edges(so.edge_beziers(1, 1, curves, scale), t)  # (14, m, 2)
    dt = t[1] - t[0]
    v = np.gradient(pts, dt, axis=1)
    acc = np.gradient(v, dt, axis=1)
    kap_fd = (v[..., 0] * acc[..., 1] - v[..., 1] * acc[..., 0]) / np.hypot(v[..., 0], v[..., 1]) ** 3
    inner = slice(100, -100)
    kap = edge_curvature(t, 1, 1, curves, scale)
    assert np.allclose(kap[:, inner], kap_fd[:, inner], rtol=1e-5, atol=1e-6)
    # speed too
    assert np.allclose(edge_speed(t, 1, 1, curves, scale)[:, inner], np.hypot(v[..., 0], v[..., 1])[:, inner], rtol=1e-6)


def test_each_edge_has_one_inflection_and_zero_net_curvature():
    k = 0.5
    t = np.linspace(0.0, 1.0, 1001)
    kap = edge_curvature(t, 1, 1, 1.0, k)  # (14, 1001)
    assert np.allclose(kap, -kap[:, ::-1], atol=1e-12)  # odd about the middle of the edge
    assert np.all(kap[:, 500] == 0.0)
    first, second = kap[:, 1:500], kap[:, 501:-1]
    sign = np.sign(first[:, :1])
    assert np.all(sign * first > 0) and np.all(-sign * second > 0)  # one sign on each half
    for i in range(so.N_EDGES):
        net = _quad_edge(lambda x: float(edge_curvature(np.array([x]), 1, 1, 1.0, k)[i, 0] * edge_speed(np.array([x]), 1, 1, 1.0, k)[i, 0]))
        assert abs(net) < 1e-10, (i, net)
    assert np.allclose(edge_curvature(t, 1, 1, 1.0, 0.0), 0.0)  # straight edges


def test_absolute_curvature_of_an_edge_is_4_arctan_2k():
    for k in (0.05, 0.3, 0.5, 1.0, -0.7):
        for i in (0, 3, 8):  # edges of both lengths
            def f(x):
                xs = np.array([x])
                return abs(float(edge_curvature(xs, 1, 1, 1.0, k)[i, 0] * edge_speed(xs, 1, 1, 1.0, k)[i, 0]))

            half = _quad_edge(f, 0.0, 0.5)
            assert _rel(2.0 * half, 4.0 * math.atan(2.0 * abs(k))) < 1e-10, (k, i)


def test_corner_turns_from_the_bezier_control_points():
    # independent derivation: tangent leaving a vertex = C1 - P0 of the next edge,
    # tangent arriving = P3 - C2 of the previous edge
    import spectre_fields as sf

    for curves, scale in ((1.0, 0.0), (1.0, 0.5), (np.random.default_rng(4).uniform(-1, 1, 14), 0.5)):
        ctrl = so.edge_beziers(1, 1, curves, scale)
        out = ctrl[:, 1] - ctrl[:, 0]
        arrive = np.roll(ctrl[:, 3] - ctrl[:, 2], 1, axis=0)
        turn = np.arctan2(arrive[:, 0] * out[:, 1] - arrive[:, 1] * out[:, 0], (arrive * out).sum(axis=1))
        assert np.allclose(corner_turn_angles(1, 1, curves, scale), turn, atol=1e-12)
        assert np.allclose(sf.corner_turns(1, 1, curves, scale), turn, atol=1e-12)  # the step-1 formula
    # and both ends of an edge point the same way
    ctrl = so.edge_beziers(1, 1, 1.0, 0.8)
    start, end = ctrl[:, 1] - ctrl[:, 0], ctrl[:, 3] - ctrl[:, 2]
    assert np.allclose(start[:, 0] * end[:, 1] - start[:, 1] * end[:, 0], 0.0, atol=1e-12)
    assert np.all((start * end).sum(axis=1) > 0)


def test_umlaufsatz_total_turning_is_minus_two_pi():
    patterns = [(1.0, 0.0), (1.0, 0.5), (1.0, -0.5), (np.random.default_rng(5).uniform(-1, 1, 14), 0.4)]
    for curves, scale in patterns:
        prof = curvature_profile(1, 1, curves, scale, per_edge=50)
        theta = prof["theta"]
        # tangent angle at the very end plus the closing turn at vertex 0 is one full clockwise turn
        assert abs(theta[-1] - theta[0] + prof["corner_turn"][0] + 2.0 * math.pi) < 1e-10
        assert abs(prof["corner_turn"].sum() + 0.0 + 2.0 * math.pi) < 1e-10  # smooth parts turn by 0
        assert _rel(prof["length"], so.perimeter(1, 1, curves, scale)) < 1e-12
        assert np.all(np.diff(prof["s"]) > 0)


def test_total_absolute_curvature_formula_and_the_straight_tile():
    prof = curvature_profile(1, 1, 1.0, 0.5, per_edge=2000)
    numeric = np.abs(prof["corner_turn"]).sum() + np.trapezoid(np.abs(prof["kappa"]), prof["s"])
    assert _rel(total_absolute_curvature(1, 1, 1.0, 0.5), numeric) < 1e-3
    # the straight tile: sum of the absolute turns of the 14-gon, 16 pi / 3, far above 2 pi
    assert _rel(total_absolute_curvature(1, 1, 1.0, 0.0), 16.0 * math.pi / 3.0) < 1e-12
    assert total_absolute_curvature(1, 1, 1.0, 0.0) > 2.0 * math.pi  # not convex (Fenchel)
    # and the value does not depend on a, b (only lengths change, not directions), nor on the sign of k
    assert _rel(total_absolute_curvature(2, 3, 1.0, 0.5), total_absolute_curvature(1, 1, 1.0, 0.5)) < 1e-12
    assert _rel(total_absolute_curvature(1, 1, 1.0, -0.5), total_absolute_curvature(1, 1, 1.0, 0.5)) < 1e-12


def test_bending_energy_by_two_independent_routes():
    prof = curvature_profile(1, 1, 1.0, 0.5, per_edge=3000)
    numeric = np.trapezoid(prof["kappa"] ** 2, prof["s"])
    assert _rel(bending_energy(1, 1, 1.0, 0.5), numeric) < 1e-3
    # small-k limit: E ~ k^2 sum_i 324 / |d_i| int (1 - 2t)^2 / s'^5 dt
    length = np.hypot(*so.edge_vectors(1, 1).T)
    coeff = _quad_edge(lambda x: 324.0 * (1 - 2 * x) ** 2 / (1.5 - 3 * x + 3 * x * x) ** 5)
    ref = float((coeff / length).sum())
    for k in (1e-3, 1e-4):
        assert _rel(bending_energy(1, 1, 1.0, k) / k**2, ref) < 5e-5 + 5 * k * k
    # mirror curves have the same energy; zero for straight edges
    assert _rel(bending_energy(1, 1, 1.0, -0.5), bending_energy(1, 1, 1.0, 0.5)) < 1e-14
    assert bending_energy(1, 1, 1.0, 0.0) == 0.0


def test_scaling_lengths_changes_curvature_but_not_the_turns():
    for f in (0.5, 3.0):
        t = np.linspace(0, 1, 101)
        assert np.allclose(edge_curvature(t, f, f, 1.0, 0.5), edge_curvature(t, 1, 1, 1.0, 0.5) / f, rtol=1e-12, atol=1e-14)
        assert _rel(bending_energy(f, f, 1.0, 0.5), bending_energy(1, 1, 1.0, 0.5) / f) < 1e-12
        assert _rel(max_curvature(f, f, 1.0, 0.5)[0], max_curvature(1, 1, 1.0, 0.5)[0] / f) < 1e-9
        assert np.allclose(corner_turn_angles(f, f, 1.0, 0.5), corner_turn_angles(1, 1, 1.0, 0.5), atol=1e-12)
        assert _rel(total_absolute_curvature(f, f, 1.0, 0.5), total_absolute_curvature(1, 1, 1.0, 0.5)) < 1e-12


def test_max_curvature_is_the_sampled_maximum_and_symmetric_in_k():
    kmax, edge, t_star = max_curvature(1, 1, 1.0, 0.5)
    t = np.linspace(0, 1, 400001)
    dense = np.abs(edge_curvature(t, 1, 1, 1.0, 0.5)).max()
    assert _rel(kmax, dense) < 1e-9 and abs(edge_curvature(np.array([t_star]), 1, 1, 1.0, 0.5)[edge, 0]) == kmax
    assert _rel(max_curvature(1, 1, 1.0, -0.5)[0], kmax) < 1e-12
    # kappa_max * |d| depends on k only, so the sharpest bend is on the SHORTEST edge: 1 / min(a, b)
    assert _rel(max_curvature(1, 2.5, 1.0, 0.5)[0], kmax) < 1e-12
    assert _rel(max_curvature(2, 3, 1.0, 0.5)[0], kmax / 2.0) < 1e-9


def test_comb_hairs_are_normal_to_the_curve_and_proportional_to_curvature():
    base, tip, kap = curvature_comb(1, 1, 1.0, 0.5, factor=0.2, per_edge=40)
    hair = tip - base
    assert np.allclose(np.hypot(hair[..., 0], hair[..., 1]), 0.2 * np.abs(kap), atol=1e-12)
    # tangent by differences of the base points: hairs are orthogonal to it
    tang = np.gradient(base, axis=1)
    inner = slice(2, -2)
    dots = (hair * tang).sum(axis=-1)[:, inner]
    assert np.abs(dots).max() < 1e-2 * 0.2 * np.abs(kap).max() * np.hypot(*tang[:, inner].reshape(-1, 2).T).max()
    # hairs point towards the centre of curvature: for a positive kappa that is the left of travel
    cross = tang[..., 0] * hair[..., 1] - tang[..., 1] * hair[..., 0]
    assert np.all(cross[:, inner] * kap[:, inner] >= -1e-12)
    flat_base, flat_tip, _ = curvature_comb(1, 1, 1.0, 0.0, factor=1.0)  # straight edges: no hairs
    assert np.allclose(flat_tip, flat_base)


def test_validation():
    for call in (
        lambda: edge_curvature(np.array([-0.1, 0.5])),
        lambda: edge_curvature(np.array([0.5, 1.5])),
        lambda: edge_curvature(np.array([])),
        lambda: edge_curvature(np.array([np.nan])),
        lambda: edge_speed(np.zeros((2, 2))),
        lambda: curvature_profile(per_edge=3),
        lambda: curvature_profile(per_edge=20.5),
        lambda: curvature_comb(factor=float("nan")),
        lambda: bending_energy(1, 1, [1.0] * 13, 1.0),
    ):
        try:
            call()
        except ValueError:
            continue
        raise AssertionError("expected ValueError")


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
            print("  %-70s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------
def show_curvature(a=1.0, b=1.0, k=0.5, seed=3, save=None, show=True):
    """Six panels: the curvature comb; curvature and corner turns along the outline; the turning
    function; the tangent indicatrix on the unit circle; the integral quantities against k; a comb
    for random per-edge amounts."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    prof = curvature_profile(a, b, 1.0, k, per_edge=400)
    base, tip, kap = curvature_comb(a, b, 1.0, k, factor=0.08, per_edge=80)
    pos_c, neg_c, col_c = "tab:red", "tab:blue", "tab:green"

    def draw_comb(ax, base, tip, kap, title, lw=0.7):
        outline = base.reshape(-1, 2)
        ax.plot(outline[:, 0], outline[:, 1], "k", lw=1.4)
        hairs = np.stack([base.reshape(-1, 2), tip.reshape(-1, 2)], axis=1)
        colors = np.where(kap.reshape(-1) > 0, pos_c, neg_c)
        ax.add_collection(LineCollection(hairs, colors=colors, linewidths=lw, alpha=0.8))
        ax.plot(tip.reshape(-1, 2)[:, 0], tip.reshape(-1, 2)[:, 1], color="0.5", lw=0.5)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(title, fontsize=9)
        ax.autoscale()

    fig, axes = plt.subplots(2, 3, figsize=(17, 11.2))

    # 1. the comb
    ax = axes[0, 0]
    draw_comb(ax, base, tip, kap, "curvature comb: hairs point to the centre of curvature\n(red kappa > 0, blue kappa < 0; numbers are vertices)")
    vx = prof["point"][np.searchsorted(prof["s"], prof["corner_s"])]
    ax.scatter(vx[:, 0], vx[:, 1], s=14, color="k", zorder=3)
    for i, (x, y) in enumerate(vx):
        ax.annotate(str(i), (x, y), fontsize=7, xytext=(3, 3), textcoords="offset points")

    # 2. curvature and corner turns along the outline
    ax = axes[0, 1]
    ax.plot(prof["s"], prof["kappa"], color="k", lw=1.0)
    ax.fill_between(prof["s"], 0, prof["kappa"], where=prof["kappa"] > 0, color=pos_c, alpha=0.35)
    ax.fill_between(prof["s"], 0, prof["kappa"], where=prof["kappa"] < 0, color=neg_c, alpha=0.35)
    ax.axhline(0, color="0.5", lw=0.6)
    ax.set_xlabel("arclength s")
    ax.set_ylabel("curvature kappa of the smooth edges")
    ax2 = ax.twinx()
    ax2.vlines(prof["corner_s"], 0, np.degrees(prof["corner_turn"]), color=col_c, lw=2.2)
    ax2.set_ylabel("corner turn (degrees)", color=col_c)
    lim = 1.15 * np.abs(np.degrees(prof["corner_turn"])).max()
    ax2.set_ylim(-lim, lim)
    ax.set_title("every edge: odd S-shaped curvature, net zero; the corners carry the turns", fontsize=9)

    # 3. turning function
    ax = axes[0, 2]
    ax.plot(prof["s"], np.degrees(prof["theta"] - prof["theta"][0]), color="k", lw=1.2)
    ax.vlines(prof["corner_s"], np.degrees(prof["theta"][np.searchsorted(prof["s"], prof["corner_s"])] - prof["theta"][0]) - np.degrees(prof["corner_turn"]),
              np.degrees(prof["theta"][np.searchsorted(prof["s"], prof["corner_s"])] - prof["theta"][0]), color=col_c, lw=1.2, alpha=0.7)
    th_end = np.degrees(prof["theta"][-1] - prof["theta"][0])
    ax.vlines(prof["s"][-1], th_end, th_end + np.degrees(prof["corner_turn"][0]), color=col_c, lw=1.2, alpha=0.7)
    ax.axhline(-360, color="0.4", ls="--", lw=0.8)
    ax.text(prof["s"][0], -360, " one full clockwise turn (Umlaufsatz)", va="bottom", fontsize=8)
    ax.set_xlabel("arclength s")
    ax.set_ylabel("tangent angle (degrees), relative to the start")
    ax.set_title("turning function: the S-shaped edges wiggle, the corners step\n(green: corner turns; the last step closes the loop at -360 degrees)", fontsize=9)
    ax.grid(alpha=0.3)

    # 4. tangent angle relative to the chord
    ax = axes[1, 0]
    tt = np.linspace(0.0, 1.0, 400)
    sp_, wp_ = _sw(tt)
    for kv, col in zip((0.1, 0.25, 0.5, 1.0), plt.cm.viridis(np.linspace(0.1, 0.85, 4))):
        rel = np.degrees(np.arctan(kv * wp_ / sp_))
        ax.plot(tt, rel, color=col, lw=2, label="k = %g" % kv)
        ax.axhline(np.degrees(np.arctan(2 * kv)), color=col, ls=":", lw=0.9)
        ax.axhline(-np.degrees(np.arctan(2 * kv)), color=col, ls=":", lw=0.9)
    ax.axhline(0, color="0.5", lw=0.6)
    ax.set_xlabel("position along the edge, t")
    ax.set_ylabel("tangent angle minus chord angle (degrees)")
    ax.legend(fontsize=8, loc="upper left")
    ax.set_title("each edge: tangent flicks from -arctan(2k) to +arctan(2k) (middle) and back\n"
                 "so |turning| = 4 arctan(2|k|) per edge and the net is zero", fontsize=9)
    ax.grid(alpha=0.3)

    # 5. integral quantities against k
    ax = axes[1, 1]
    ks = np.linspace(0.0, 1.0, 41)
    tac = np.array([total_absolute_curvature(a, b, 1.0, kv) for kv in ks])
    kmx = [max_curvature(a, b, 1.0, kv)[0] * min(a, b) for kv in ks]
    ax.plot(ks, tac / (2 * np.pi), color="tab:purple", lw=2)
    ax.axhline(1.0, color="0.4", ls="--", lw=0.9)
    ax.text(1.0, 1.0, "convex curve: 1 ", va="bottom", ha="right", fontsize=8)
    ax.annotate("straight tile: 8/3", (0.0, tac[0] / (2 * np.pi)), xytext=(0.05, 4.2), fontsize=8,
                arrowprops=dict(arrowstyle="-", color="0.4"))
    ax.set_xlabel("curve strength k")
    ax.set_ylabel("total absolute curvature / 2 pi", color="tab:purple")
    ax2 = ax.twinx()
    ax2.plot(ks, kmx, color="tab:orange", lw=2)
    ax2.set_ylabel("sharpest bend x shortest edge  (kappa_max min(a, b))", color="tab:orange")
    ax.set_title("exact total curvature (purple) and sharpest bend (orange) against k:\n"
                 "curving adds 4 arctan(2k) per edge to the 16 pi / 3 of the corners", fontsize=9)
    ax.grid(alpha=0.3)

    # 6. random per-edge amounts
    ax = axes[1, 2]
    curves = np.random.default_rng(seed).uniform(-1, 1, so.N_EDGES)
    b6, t6, k6 = curvature_comb(a, b, curves, 0.4, factor=0.08, per_edge=80)
    draw_comb(ax, b6, t6, k6, "random per-edge amounts (seed %d), scale 0.4:\neach edge bends by its own k" % seed)

    fig.suptitle(
        "Curvature of the curved Spectre monotile (a=%g, b=%g, k=%g): closed form kappa = -18 k (1 - 2t) / "
        "(|d| (s'^2 + k^2 w'^2)^(3/2)),  int|kappa| ds = 4 arctan(2|k|) per edge" % (a, b, k),
        fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=100)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(
        description="Curvature, turning function and bending energy of the curved Spectre monotile: "
        "self tests and a figure."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--curve_strength", type=float, default=0.5, help="uniform curve strength k (default 0.5)")
    p.add_argument("--seed", type=int, default=3, help="seed of the random per-edge panel")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_curvature self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_curvature(args.a, args.b, args.curve_strength, args.seed, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
