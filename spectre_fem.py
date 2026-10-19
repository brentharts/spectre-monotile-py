"""
spectre_fem.py - fields on the FILLED curved Spectre monotile (step 2).

Step 1 (spectre_fields.py) looked at distance fields around the outline.  Here the tile is a
solid: it is meshed with triangles and three classical fields are solved on it with linear
(P1) finite elements, using nothing but numpy and scipy.sparse.

    from spectre_fem import TileMesh
    m = TileMesh.tile(a=1, b=1, curves=1.0, scale=0.5, h=0.05)   # arguments as in spectre_operator
    t = m.torsion()           # Saint-Venant torsion: phi, J (torsional rigidity), shear stress
    w = m.eigenmodes(6)       # Dirichlet drum modes: -lap u = lambda u, u = 0 on the outline
    Q = m.heat_content(times) # cooling from u = 1 with the outline held at 0

The three problems (all with u = 0 on the outline, K = stiffness, M = mass matrix):

  torsion   -lap(phi) = 2             J = 2 * integral(phi)      stress = (phi_y, -phi_x)
  modes     -lap(u)   = lambda * u    lambda_1 is the lowest tone of a drum shaped like the tile
  heat      u_t = lap(u),  u(0) = 1   Q(t) = integral(u) = heat left in the tile at time t

What the self tests check:
  * the solver reproduces the exact disc and square results (J, lambda_n, heat content);
  * for the tile, J obeys the Saint-Venant bound  J <= A^2 / (2 pi)  and lambda_1 obeys
    Faber-Krahn  lambda_1 >= pi j_{0,1}^2 / A,  where A is the area, which never depends on
    the curve amounts - so how far a curved tile sits from the bound measures its shape only;
  * Q(t) = A - (2/sqrt(pi)) P sqrt(t) + O(t) for small t: the heat content sees the PERIMETER
    that step 1 computed in closed form (spectre_operator.perimeter);
  * the numbers converge with the mesh size and do not depend on how the tile is rotated.

The three fields feel the corners: at a reflex (> 180 degree) corner the shear stress of the
torsion problem is singular, which P1 elements on a uniform mesh only resolve slowly, so the
values of the stress there are reported by the plots, not tested to high accuracy.

Run ``python3 spectre_fem.py`` to run the self tests and show the figure (--help for options).
"""

import argparse
import math
import os
import sys

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.optimize import brentq
from scipy.spatial import Delaunay, cKDTree
from scipy.special import jn_zeros
from matplotlib.path import Path

import spectre_operator as so

__all__ = [
    "TileMesh",
    "polygon_boundary",
    "disc_boundary",
    "faber_krahn_bound",
    "saint_venant_bound",
    "run_tests",
    "show_fem_fields",
]

J01 = float(jn_zeros(0, 1)[0])  # first zero of the Bessel function J_0: 2.404825...


# --------------------------------------------------------------------------
# Boundaries: closed loops of points, spaced by about h
# --------------------------------------------------------------------------
def _resample_chain(points, h):
    """Points along a polyline at equal arclength, start included and end excluded."""
    seg = np.hypot(*np.diff(points, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(1, int(round(s[-1] / h)))
    target = np.linspace(0.0, s[-1], n + 1)[:-1]
    return np.column_stack([np.interp(target, s, points[:, 0]), np.interp(target, s, points[:, 1])])


def polygon_boundary(corners, h):
    """Boundary loop along a straight-sided polygon (corners kept exactly)."""
    corners = np.asarray(corners, float)
    parts = []
    for p, q in zip(corners, np.roll(corners, -1, axis=0)):
        parts.append(_resample_chain(np.array([p, q]), h))
    return np.vstack(parts)


def disc_boundary(radius, h):
    n = max(8, int(round(2 * math.pi * radius / h)))
    th = 2 * math.pi * np.arange(n) / n
    return radius * np.column_stack([np.cos(th), np.sin(th)])


def _tile_boundary(a, b, curves, scale, h):
    edges = so.curved_spectre_edges(a, b, curves, scale, samples=400)  # (14, 401, 2)
    return np.vstack([_resample_chain(e, h) for e in edges])


# --------------------------------------------------------------------------
# Mesh
# --------------------------------------------------------------------------
class TileMesh:
    """Triangle mesh of a region plus P1 finite element matrices.

    Parameters
    ----------
    boundary : (n, 2) closed loop of points (first point not repeated), spaced by about ``h``.
    h : target edge length; interior points sit on a hexagonal lattice of that spacing.

    Attributes: ``points`` (N, 2), ``triangles`` (T, 3), ``boundary_index`` (n,) the first n
    points, ``interior`` boolean mask of free nodes, ``K`` stiffness, ``M`` mass (consistent).
    """

    def __init__(self, boundary, h):
        boundary = np.asarray(boundary, float)
        if boundary.ndim != 2 or boundary.shape[1] != 2 or len(boundary) < 3:
            raise ValueError("boundary must be an (n, 2) array with n >= 3")
        if not (np.isfinite(boundary).all() and h > 0 and np.isfinite(h)):
            raise ValueError("boundary must be finite and h positive")
        self.h = float(h)
        n = len(boundary)
        path = Path(np.vstack([boundary, boundary[:1]]))
        # dense copy of the loop for distances to the boundary
        nxt = np.roll(boundary, -1, axis=0)
        dense = np.vstack([boundary + (nxt - boundary) * f for f in np.linspace(0, 1, 8, endpoint=False)])
        tree = cKDTree(dense)

        lo, hi = boundary.min(axis=0) - h, boundary.max(axis=0) + h
        dy = h * math.sqrt(3.0) / 2.0
        rows = np.arange(lo[1], hi[1] + dy, dy)
        pts = []
        for r, y in enumerate(rows):
            xs = np.arange(lo[0] + (0.5 * h if r % 2 else 0.0), hi[0] + h, h)
            pts.append(np.column_stack([xs, np.full_like(xs, y)]))
        lattice = np.vstack(pts)
        keep = path.contains_points(lattice) & (tree.query(lattice)[0] > 0.7 * h)
        points = np.vstack([boundary, lattice[keep]])

        tri = Delaunay(points).simplices
        centroid = points[tri].mean(axis=1)
        tri = tri[path.contains_points(centroid)]
        p0, p1, p2 = (points[tri[:, i]] for i in range(3))
        cross = (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (p1[:, 1] - p0[:, 1]) * (p2[:, 0] - p0[:, 0])
        # Delaunay makes flat triangles out of collinear boundary points: drop them
        good = np.abs(cross) > 1e-9 * h * h
        tri, cross = tri[good], cross[good]
        flip = cross < 0  # keep the orientation counterclockwise
        tri[flip] = tri[flip][:, [0, 2, 1]]

        self.points, self.triangles = points, tri
        self.n_boundary = n
        self.interior = np.ones(len(points), bool)
        self.interior[:n] = False
        self._assemble()

    @classmethod
    def tile(cls, a=1.0, b=1.0, curves=1.0, scale=0.5, h=0.05):
        """Mesh of the curved tile; (a, b, curves, scale) as in spectre_operator.curved_spectre."""
        m = cls(_tile_boundary(a, b, curves, scale, h), h)
        m.params = dict(a=a, b=b, curves=curves, scale=scale)
        return m

    # -- finite element matrices ---------------------------------------------
    def _assemble(self):
        P, T = self.points, self.triangles
        p0, p1, p2 = P[T[:, 0]], P[T[:, 1]], P[T[:, 2]]
        # gradients of the hat functions: grad N_i = (y_j - y_k, x_k - x_j) / (2 area)
        bx = np.stack([p1[:, 1] - p2[:, 1], p2[:, 1] - p0[:, 1], p0[:, 1] - p1[:, 1]], axis=1)
        by = np.stack([p2[:, 0] - p1[:, 0], p0[:, 0] - p2[:, 0], p1[:, 0] - p0[:, 0]], axis=1)
        twice = (p1[:, 0] - p0[:, 0]) * (p2[:, 1] - p0[:, 1]) - (p2[:, 0] - p0[:, 0]) * (p1[:, 1] - p0[:, 1])
        area = 0.5 * twice
        self.triangle_area = area
        self._grad = (bx / twice[:, None], by / twice[:, None])  # (T, 3) each
        Ke = (bx[:, :, None] * bx[:, None, :] + by[:, :, None] * by[:, None, :]) / (4.0 * area[:, None, None])
        Me = area[:, None, None] / 12.0 * (1.0 + np.eye(3))[None]
        I = np.repeat(T[:, :, None], 3, axis=2).ravel()
        J = np.repeat(T[:, None, :], 3, axis=1).ravel()
        N = len(P)
        self.K = sp.csr_matrix((Ke.ravel(), (I, J)), shape=(N, N))
        self.M = sp.csr_matrix((Me.ravel(), (I, J)), shape=(N, N))

    # -- basic geometry --------------------------------------------------------
    @property
    def area(self):
        return float(self.triangle_area.sum())

    def boundary_edges(self):
        """Edges used by exactly one triangle (the outline of the mesh), as sorted pairs."""
        e = np.sort(np.concatenate([self.triangles[:, [0, 1]], self.triangles[:, [1, 2]],
                                    self.triangles[:, [2, 0]]]), axis=1)
        uniq, count = np.unique(e, axis=0, return_counts=True)
        return uniq[count == 1]

    def min_angle_degrees(self):
        P, T = self.points, self.triangles
        best = 180.0
        for i in range(3):
            u = P[T[:, (i + 1) % 3]] - P[T[:, i]]
            v = P[T[:, (i + 2) % 3]] - P[T[:, i]]
            cosang = (u * v).sum(1) / (np.hypot(*u.T) * np.hypot(*v.T))
            best = min(best, float(np.degrees(np.arccos(np.clip(cosang, -1, 1))).min()))
        return best

    def integral(self, u):
        """Integral of a P1 function given by its node values."""
        return float(np.sum(self.M @ np.ones(len(u)) * u))

    def gradient(self, u):
        """Gradient of a P1 function per triangle: two arrays of length T."""
        gx, gy = self._grad
        return (gx * u[self.triangles]).sum(1), (gy * u[self.triangles]).sum(1)

    # -- problems --------------------------------------------------------------
    def _free(self, A):
        return A[self.interior][:, self.interior].tocsc()

    def torsion(self):
        """Saint-Venant torsion: -lap(phi) = 2, phi = 0 on the outline.

        Returns dict(phi=node values, J=2*integral(phi), stress=|grad phi| per triangle,
        max_stress=its maximum).  J is the torsional rigidity (for G*theta = 1)."""
        free = self.interior
        load = 2.0 * (self.M @ np.ones(len(self.points)))  # lumped load for -lap(phi) = 2
        phi = np.zeros(len(self.points))
        phi[free] = spla.spsolve(self._free(self.K), load[free])
        gx, gy = self.gradient(phi)
        stress = np.hypot(gx, gy)
        return dict(phi=phi, J=2.0 * self.integral(phi), stress=stress, max_stress=float(stress.max()))

    def eigenmodes(self, count=6):
        """Lowest ``count`` Dirichlet eigenpairs of -lap: (lambda (count,), modes (N, count)).

        Modes are normalised to unit L2 norm and sign-fixed (largest entry positive)."""
        free = self.interior
        nfree = int(free.sum())
        if not 1 <= count < nfree - 1:
            raise ValueError("count must be between 1 and the number of free nodes - 2")
        lam, vec = spla.eigsh(self._free(self.K), k=count, M=self._free(self.M), sigma=0.0, which="LM")
        order = np.argsort(lam)
        lam, vec = lam[order], vec[:, order]
        modes = np.zeros((len(self.points), count))
        modes[free] = vec
        for j in range(count):
            if modes[np.argmax(np.abs(modes[:, j])), j] < 0:
                modes[:, j] *= -1.0
        return lam, modes

    def heat(self, times, u0=None, substeps=40):
        """Heat equation u_t = lap(u) with u = 0 on the outline; returns the node values at
        each time of ``times`` (sorted, > 0) as an array (len(times), N).

        u0 defaults to 1 in the interior.  Crank-Nicolson with a few implicit-Euler start-up
        steps (which damp the jump between u0 = 1 and the boundary value 0)."""
        times = np.asarray(times, float)
        if times.ndim != 1 or times.size == 0 or np.any(times <= 0) or np.any(np.diff(times) < 0):
            raise ValueError("times must be a non-empty increasing sequence of positive numbers")
        free = self.interior
        u = np.zeros(len(self.points))
        u[free] = 1.0 if u0 is None else np.asarray(u0, float)[free]
        K, M = self._free(self.K), self._free(self.M)
        v = u[free]
        out, t_now = [], 0.0
        for T in times:
            span = T - t_now
            if span > 0:
                v = self._advance(K, M, v, span, max(substeps, 4), start=(t_now == 0.0))
            t_now = T
            full = np.zeros(len(self.points))
            full[free] = v
            out.append(full)
        return np.array(out)

    @staticmethod
    def _advance(K, M, v, span, n, start):
        """Advance by ``span`` in ``n`` steps of Crank-Nicolson.  At the very start the first
        two steps are replaced by four backward-Euler steps of half the size (Rannacher
        start-up), which removes the oscillations caused by the jump at the outline."""
        dt = span / n
        if start:
            lu = spla.splu((M + 0.5 * dt * K).tocsc())
            for _ in range(4):
                v = lu.solve(M @ v)
            n -= 2
        lu = spla.splu((M + 0.5 * dt * K).tocsc())
        rhs = (M - 0.5 * dt * K).tocsr()
        for _ in range(n):
            v = lu.solve(rhs @ v)
        return v

    def heat_content(self, times, **kw):
        """Q(t) = integral of the temperature, starting from u = 1 inside."""
        u = self.heat(times, **kw)
        ones = self.M @ np.ones(len(self.points))
        return u @ ones


# --------------------------------------------------------------------------
# Reference values
# --------------------------------------------------------------------------
def faber_krahn_bound(area):
    """Smallest possible lambda_1 for a region of this area: the disc's, pi * j01^2 / area."""
    return math.pi * J01**2 / area


def saint_venant_bound(area):
    """Largest possible torsional rigidity J = 2 * integral(phi) for this area: the disc's,
    area^2 / (2 pi)."""
    return area * area / (2.0 * math.pi)


def _disc_heat_content(t, terms=400):
    """Exact heat content of the unit disc started from 1: 4 pi sum exp(-j_n^2 t) / j_n^2."""
    j = jn_zeros(0, terms)
    return 4.0 * math.pi * float(np.sum(np.exp(-(j**2) * t) / j**2))


def _square_heat_content(t):
    """Exact heat content of the unit square started from 1 (product of two 1-D series)."""
    n = np.arange(1, 400, 2)
    return float(np.sum(8.0 / (n**2 * math.pi**2) * np.exp(-(n**2) * math.pi**2 * t))) ** 2


UNIT_SQUARE = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
J_SQUARE = 0.140577  # torsional rigidity 2*int(phi) of the unit square (series value 0.1405770...)


# --------------------------------------------------------------------------
# Self tests
# --------------------------------------------------------------------------
def _rel(x, ref):
    return abs(x - ref) / abs(ref)


def test_mesh_is_a_valid_triangulation_of_the_tile():
    A = so.spectre_area()
    for k in (0.0, 0.5):  # straight edges are the hard case (collinear boundary nodes)
        m = TileMesh.tile(1, 1, 1.0, k, h=0.06)
        assert (m.triangle_area > 0).all()
        assert m.min_angle_degrees() > 15.0, m.min_angle_degrees()
        # the mesh outline is exactly the boundary loop
        n = m.n_boundary
        loop = {tuple(sorted((i, (i + 1) % n))) for i in range(n)}
        assert {tuple(e) for e in m.boundary_edges().tolist()} == loop
        # area: the polygon through the boundary nodes (error is O(h^2) at the curved edges)
        assert _rel(m.area, A) < 2e-3, (m.area, A)
        assert m.K.shape == m.M.shape == (len(m.points),) * 2
        assert abs(m.M.sum() - m.area) < 1e-12  # mass matrix integrates 1 * 1


def test_disc_reproduces_the_exact_solutions():
    h = 0.04
    m = TileMesh(disc_boundary(1.0, h), h)
    tor = m.torsion()
    assert _rel(tor["J"], math.pi / 2) < 3e-3, tor["J"]  # phi = (1 - r^2) / 2
    # the disc is the extremal shape of both bounds, so they must reproduce its own values
    assert _rel(saint_venant_bound(math.pi), math.pi / 2) < 1e-12
    assert _rel(faber_krahn_bound(math.pi), J01**2) < 1e-12
    assert abs(tor["phi"].max() - 0.5) < 3e-3
    lam, _ = m.eigenmodes(4)
    assert _rel(lam[0], J01**2) < 3e-3, lam
    j11 = float(jn_zeros(1, 1)[0])
    assert _rel(lam[1], j11**2) < 5e-3 and _rel(lam[2], j11**2) < 5e-3  # double eigenvalue
    ts = [0.03, 0.1, 0.3]
    Q = m.heat_content(ts)
    for t, q in zip(ts, Q):
        assert _rel(q, _disc_heat_content(t)) < 5e-3, (t, q)


def test_square_reproduces_the_exact_solutions():
    h = 0.025
    m = TileMesh(polygon_boundary(UNIT_SQUARE, h), h)
    assert _rel(m.torsion()["J"], J_SQUARE) < 3e-3
    lam, _ = m.eigenmodes(4)
    for got, ref in zip(lam, np.array([2, 5, 5, 8]) * math.pi**2):
        assert _rel(got, ref) < 5e-3, (got, ref)
    ts = [0.03, 0.06, 0.1]  # later the content is tiny and eigenvalue errors times t grow
    for t, q in zip(ts, m.heat_content(ts)):
        assert _rel(q, _square_heat_content(t)) < 6e-3, (t, q)


def test_torsion_energy_identity():
    # integral |grad phi|^2 = 2 integral phi = J: ties gradient(), the areas and K together
    m = TileMesh.tile(1, 1, 1.0, 0.4, h=0.06)
    tor = m.torsion()
    energy = float(np.sum(m.triangle_area * tor["stress"] ** 2))
    assert _rel(energy, tor["J"]) < 1e-9, (energy, tor["J"])
    assert tor["phi"][~m.interior].max() == 0.0 and tor["phi"].min() >= 0.0  # positive inside


def test_modes_are_orthonormal_and_rayleigh_quotients_match():
    m = TileMesh.tile(1, 1, 1.0, 0.5, h=0.06)
    lam, modes = m.eigenmodes(5)
    assert np.all(np.diff(lam) >= 0)
    assert np.allclose(modes.T @ (m.M @ modes), np.eye(5), atol=1e-9)
    for j in range(5):
        u = modes[:, j]
        assert abs(u @ (m.K @ u) - lam[j]) < 1e-8 * lam[j]
    assert (modes[:, 0] >= -1e-12).all()  # the ground state has no nodal line
    assert modes[:, 1].min() < -1e-3 and modes[:, 1].max() > 1e-3  # the next mode changes sign


def test_heat_follows_the_modes_and_obeys_the_maximum_principle():
    m = TileMesh.tile(1, 1, 1.0, 0.5, h=0.06)
    lam, modes = m.eigenmodes(2)
    # started from the ground state, the heat just decays like exp(-lambda_1 t)
    t = 0.1
    u = m.heat([t / 2, t], u0=modes[:, 0], substeps=200)
    ratio = (u[1] @ (m.M @ modes[:, 0])) / (modes[:, 0] @ (m.M @ modes[:, 0]))
    assert _rel(ratio, math.exp(-lam[0] * t)) < 2e-4, (ratio, math.exp(-lam[0] * t))
    assert np.allclose(u[1], math.exp(-lam[0] * t) * modes[:, 0], atol=2e-4)
    # from u = 1: stays between 0 and 1 (up to the small overshoot that the consistent mass
    # matrix allows when the time step is below h^2 / 6) and the content only decreases
    ts = np.array([0.002, 0.01, 0.05, 0.2, 2.5, 3.0])
    U = m.heat(ts)
    assert U.min() > -1e-2 and U.max() < 1.0 + 1e-2
    Q = m.heat_content(ts)
    assert np.all(np.diff(Q) < 0) and Q[0] < m.area
    # at late times it decays like exp(-lambda_1 t)
    rate = -math.log(Q[-1] / Q[-2]) / (ts[-1] - ts[-2])
    assert _rel(rate, lam[0]) < 5e-3, (rate, lam[0])


def test_heat_content_sees_the_perimeter():
    # Q(t) = A - 2 P sqrt(t / pi) + O(t): fit (A - Q) / sqrt(t) = alpha + beta sqrt(t) + gamma t
    # on small times and compare alpha with 2 P / sqrt(pi) (spectre_operator.perimeter).
    # The curved tile (k = 0.5) has perimeter 18.7 against 15.8 for the straight one, so the
    # 2.5 % tolerance cleanly separates the right perimeter from a wrong one.
    A = so.spectre_area()
    ts = np.array([0.004, 0.006, 0.009, 0.013, 0.02, 0.03, 0.045])
    m = TileMesh.tile(1, 1, 1.0, 0.5, h=0.0125)
    y = (A - m.heat_content(ts)) / np.sqrt(ts)
    alpha = np.polyfit(np.sqrt(ts), y, 2)[-1]
    ref = 2.0 * so.perimeter(1, 1, 1.0, 0.5) / math.sqrt(math.pi)
    assert _rel(alpha, ref) < 0.025, (alpha, ref)
    assert _rel(alpha, 2.0 * so.perimeter(1, 1, 1.0, 0.0) / math.sqrt(math.pi)) > 0.1


def test_bounds_hold_for_every_curve_pattern():
    # The area never changes, so the Faber-Krahn and Saint-Venant bounds are the same numbers
    # for all shapes; the tile sits strictly inside them and never touches the disc values.
    A = so.spectre_area()
    rng = np.random.default_rng(7)
    patterns = [(1.0, 0.0), (1.0, 0.3), (1.0, -0.3), (1.0, 0.7), (rng.uniform(-1, 1, 14), 0.3)]
    for curves, scale in patterns:
        x, y = so.curved_spectre(1, 1, curves, scale, samples=30)
        assert so.is_simple(x, y)
        m = TileMesh.tile(1, 1, curves, scale, h=0.06)
        J = m.torsion()["J"]
        lam = m.eigenmodes(1)[0][0]
        assert J < 0.9 * saint_venant_bound(A), (scale, J)
        assert lam > 1.1 * faber_krahn_bound(A), (scale, lam)
        # Polya: lambda_1 * P / A < 1 for the torsion function of -lap(u) = 1; here J = 4 P
        assert lam * J / (4.0 * A) < 1.0, (scale, lam * J / (4.0 * A))


def test_numbers_converge_with_the_mesh_size():
    vals = []
    for h in (0.1, 0.05, 0.025):
        m = TileMesh.tile(1, 1, 1.0, 0.5, h=h)
        vals.append((m.torsion()["J"], m.eigenmodes(1)[0][0]))
    vals = np.array(vals)
    d = np.abs(np.diff(vals, axis=0))  # successive changes shrink at least by a factor 2
    assert np.all(d[1] < 0.5 * d[0]), d


def test_rotation_and_scaling_do_not_change_the_physics():
    h = 0.04
    base = TileMesh.tile(1, 1, 1.0, 0.5, h=h)
    J0, lam0 = base.torsion()["J"], base.eigenmodes(1)[0][0]
    # rotate the outline by 37 degrees: new lattice, same answers up to mesh noise
    c, s = math.cos(math.radians(37)), math.sin(math.radians(37))
    rot = _tile_boundary(1, 1, 1.0, 0.5, h) @ np.array([[c, s], [-s, c]])
    m = TileMesh(rot, h)
    assert _rel(m.torsion()["J"], J0) < 3e-3 and _rel(m.eigenmodes(1)[0][0], lam0) < 3e-3
    # scale a and b (and h) by 2: the same mesh, lambda ~ 1/s^2 and J ~ s^4 exactly
    big = TileMesh.tile(2, 2, 1.0, 0.5, h=2 * h)
    assert _rel(big.eigenmodes(1)[0][0], lam0 / 4.0) < 1e-6
    assert _rel(big.torsion()["J"], 16.0 * J0) < 1e-6


def test_the_two_chiralities_differ_by_more_than_the_mesh_noise():
    # k and -k have the same area and, edge by edge, mirror-image curves, yet the tile is chiral:
    # the straight polygon is not symmetric, so bulging out versus in is not a mirror image.
    h = 0.025
    lam = {s: TileMesh.tile(1, 1, 1.0, s * 0.5, h=h).eigenmodes(1)[0][0] for s in (+1, -1)}
    lam_c = {s: TileMesh.tile(1, 1, 1.0, s * 0.5, h=0.04).eigenmodes(1)[0][0] for s in (+1, -1)}
    gap = abs(lam[+1] - lam[-1]) / lam[+1]
    gap_c = abs(lam_c[+1] - lam_c[-1]) / lam_c[+1]
    assert gap > 5e-3 and abs(gap - gap_c) < 0.5 * gap, (gap, gap_c)


def test_shear_stress_peaks_at_a_reflex_corner_and_is_singular_there():
    # The tile is traversed clockwise, so a counterclockwise turn of the outline is a reflex
    # (270 or 240 degree) corner, where the torsion stress behaves like r^(pi/angle - 1): it grows
    # without bound as the mesh is refined.
    d = so.edge_vectors(1, 1)
    cross = d[:, 0] * np.roll(d, 1, axis=0)[:, 1] - d[:, 1] * np.roll(d, 1, axis=0)[:, 0]
    reflex = cross < 0  # edge i-1 then edge i: counterclockwise turn
    V = so.vertices(1, 1)
    peaks = []
    for h in (0.06, 0.03):
        m = TileMesh.tile(1, 1, 1.0, 0.5, h=h)
        tor = m.torsion()
        c = m.points[m.triangles[int(np.argmax(tor["stress"]))]].mean(axis=0)
        dist = np.hypot(*(V - c).T)
        assert reflex[int(np.argmin(dist))] and dist.min() < 1.5 * h, (h, dist.min())
        peaks.append(tor["max_stress"])
    assert peaks[1] > 1.05 * peaks[0], peaks


def test_validation():
    for bad in (np.zeros((2, 2)), np.zeros((4, 3)), np.array([[0, 0], [1, 0], [np.nan, 1.0]])):
        try:
            TileMesh(bad, 0.1)
        except ValueError:
            pass
        else:
            raise AssertionError("expected ValueError for a bad boundary")
    try:
        TileMesh(UNIT_SQUARE, -1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for h <= 0")
    m = TileMesh(polygon_boundary(UNIT_SQUARE, 0.2), 0.2)
    for call in (lambda: m.heat([]), lambda: m.heat([0.2, 0.1]), lambda: m.heat([-1.0]),
                 lambda: m.eigenmodes(0), lambda: m.eigenmodes(10**6)):
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
            print("  %-66s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------
def show_fem_fields(a=1.0, b=1.0, k=0.5, h=0.03, save=None, show=True):
    """Twelve panels: torsion function and shear stress; torsional rigidity and lowest tone
    against k (with the disc bounds); the first four drum modes; three heat snapshots and the
    heat content against its perimeter asymptote."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm
    import matplotlib.tri as mtri

    A = so.spectre_area(a, b)
    m = TileMesh.tile(a, b, 1.0, k, h=h)
    tri = mtri.Triangulation(m.points[:, 0], m.points[:, 1], m.triangles)
    outline = np.vstack([m.points[: m.n_boundary], m.points[:1]])
    pad = 0.12

    def frame(ax, title):
        ax.set_aspect("equal")
        ax.set_xlim(outline[:, 0].min() - pad, outline[:, 0].max() + pad)
        ax.set_ylim(outline[:, 1].min() - pad, outline[:, 1].max() + pad)
        ax.plot(outline[:, 0], outline[:, 1], "k", lw=1.2)
        ax.set_title(title, fontsize=9)
        ax.axis("off")

    fig, axes = plt.subplots(3, 4, figsize=(17, 12.4))

    # -- torsion ---------------------------------------------------------------
    tor = m.torsion()
    ax = axes[0, 0]
    cs = ax.tricontourf(tri, tor["phi"], levels=18, cmap="viridis")
    ax.tricontour(tri, tor["phi"], levels=18, colors="k", linewidths=0.25, alpha=0.6)
    fig.colorbar(cs, ax=ax, shrink=0.6)
    frame(ax, "torsion function phi  (-lap phi = 2),  J = %.3f" % tor["J"])

    ax = axes[0, 1]
    st = np.maximum(tor["stress"], 1e-3)
    pc = ax.tripcolor(tri, facecolors=st, cmap="magma", norm=LogNorm(vmin=0.05, vmax=max(2.0, st.max())))
    fig.colorbar(pc, ax=ax, shrink=0.6)
    i = int(np.argmax(tor["stress"]))
    peak = m.points[m.triangles[i]].mean(axis=0)
    ax.plot(*peak, "o", mfc="none", mec="w", ms=11, mew=1.5)
    frame(ax, "shear stress |grad phi| (log); circled: a reflex corner, where it is singular")

    # -- J and lambda_1 against k ----------------------------------------------
    ks = np.linspace(-0.8, 0.8, 17)
    Js, lams, kk = [], [], []
    for kv in ks:
        x, y = so.curved_spectre(a, b, 1.0, kv, samples=30)
        if not so.is_simple(x, y):
            continue
        mk = TileMesh.tile(a, b, 1.0, kv, h=0.06)
        Js.append(mk.torsion()["J"])
        lams.append(mk.eigenmodes(1)[0][0])
        kk.append(kv)
    kk, Js, lams = np.array(kk), np.array(Js), np.array(lams)
    ax = axes[0, 2]
    ax.plot(kk, Js, "o-", color="tab:green", ms=4)
    ax.axhline(saint_venant_bound(A), color="k", ls="--", lw=1)
    ax.text(kk.min(), saint_venant_bound(A), " Saint-Venant bound (disc of equal area)", va="bottom", fontsize=8)
    ax.set_xlabel("curve strength k   (k < 0: curved inwards)")
    ax.set_ylabel("torsional rigidity J")
    ax.set_title("stiffer when straight; area is the same for every k", fontsize=9)
    ax.grid(alpha=0.3)
    ax = axes[0, 3]
    ax.plot(kk, lams, "o-", color="tab:purple", ms=4)
    ax.axhline(faber_krahn_bound(A), color="k", ls="--", lw=1)
    ax.text(kk.min(), faber_krahn_bound(A), " Faber-Krahn bound (disc of equal area)", va="bottom", fontsize=8)
    ax.set_xlabel("curve strength k")
    ax.set_ylabel("lowest tone  lambda_1")
    ax.set_title("lambda_1 rises with |k|; k and -k differ slightly (chiral tile)", fontsize=9)
    ax.grid(alpha=0.3)

    # -- Dirichlet modes -------------------------------------------------------
    lam, modes = m.eigenmodes(4)
    for j in range(4):
        ax = axes[1, j]
        u = modes[:, j]
        v = np.abs(u).max()
        ax.tricontourf(tri, u, levels=np.linspace(-v, v, 25), cmap="RdBu_r")
        ax.tricontour(tri, u, levels=[0.0], colors="k", linewidths=0.9, linestyles="--")
        frame(ax, "mode %d,  lambda = %.3f  (dashed: nodal line)" % (j + 1, lam[j]))

    # -- heat --------------------------------------------------------------------
    snaps = [0.005, 0.03, 0.15]
    U = m.heat(snaps)
    for j, t in enumerate(snaps):
        ax = axes[2, j]
        Uj = np.clip(U[j], 0.0, 1.0 - 1e-9)  # the consistent mass matrix overshoots by ~0.5 % early on
        cs = ax.tricontourf(tri, Uj, levels=np.linspace(0, 1, 21), cmap="inferno")
        ax.tricontour(tri, Uj, levels=[0.5], colors="w", linewidths=0.8)
        if j == 2:
            fig.colorbar(cs, ax=ax, shrink=0.6, label="temperature")
        frame(ax, "heat, t = %g  (starts at 1, outline held at 0)" % t)

    ax = axes[2, 3]
    ts = np.geomspace(0.002, 1.0, 40)
    Q = m.heat_content(ts)
    P = so.perimeter(a, b, 1.0, k)
    ax.plot(ts, Q, "-", color="tab:red", lw=2, label="heat content Q(t) (FEM)")
    tt = ts[ts < 0.06]
    ax.plot(tt, A - 2 * P * np.sqrt(tt / math.pi), "k--", lw=1.2, label="A - 2P sqrt(t/pi)")
    t_ref = 0.5
    q_ref = float(np.interp(t_ref, ts, Q))
    ax.plot(ts, q_ref * np.exp(-lam[0] * (ts - t_ref)), ":", color="tab:blue", lw=1.4, label="c exp(-lambda_1 t) tail")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_ylim(0.02, A * 1.2)
    ax.set_xlabel("time t")
    ax.set_ylabel("heat left in the tile")
    ax.set_title("early cooling sees the perimeter, late cooling the lowest tone", fontsize=9)
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    fig.suptitle(
        "Fields on the filled curved Spectre monotile (a=%g, b=%g, k=%g; P1 finite elements, h=%g, %d nodes): "
        "torsion, drum modes, heat" % (a, b, k, h, len(m.points)),
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
        description="Torsion, Dirichlet drum modes and heat on the filled curved Spectre monotile: "
        "self tests and a figure."
    )
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--curve_strength", type=float, default=0.5, help="uniform curve strength k (default 0.5)")
    p.add_argument("--h", type=float, default=0.03, help="mesh size of the figure (default 0.03)")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_fem self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_fem_fields(args.a, args.b, args.curve_strength, args.h, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
