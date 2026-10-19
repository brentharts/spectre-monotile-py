"""
spectre_diffract.py - form factor and diffraction of the curved Spectre monotile (step 5b).

The Fourier transform of the indicator function of the tile,

        F(k) = integral over the tile of exp(i k.x) dA,

is computed by Green's theorem along the 14 Bezier edges (the divergence of k exp(i k.x) is
i |k|^2 exp(i k.x)):

        F(k) = (1 / (i |k|^2)) * oriented boundary integral of exp(i k.x) (k_x dy - k_y dx),

evaluated by Gauss-Legendre quadrature on every edge.  A tile placed by x -> L x + t has the form
factor  |det L| exp(i k.t) F(L^T k), so the amplitude of a whole patch of disjoint tiles is the exact
sum over its tiles (spectre_patch.SpectrePatch), grouped by orientation:

    from spectre_diffract import form_factor, patch_amplitude, diffraction_intensity
    F = form_factor(k, a=1, b=1, curves=1.0, scale=0.5)            # k: array (..., 2)
    amp = patch_amplitude(patch, k, curves=1.0, scale=0.5)         # FT of the patch
    I = diffraction_intensity(patch, k)                            # |amp|^2 / number of tiles

What the self tests check:
  * F(0) = area, the small-k moments are the centroid and the second moments of the tile, the
    straight tile agrees with the exact polygon formula and the curved tile with direct rasterisation;
  * the placed form factor equals the direct integral over a rotated, mirrored, translated outline;
  * Porod's law for a sharp boundary: the angular and radial mean of k^3 |F|^2 tends to 2 P, with P the
    exact perimeter of spectre_operator - for the curved AND the straight tile (P differs by 18 %), and
    Plancherel's identity  integral |F|^2 d^2k / (2 pi)^2 = area  holds with that Porod tail;
  * the amplitude of a patch equals the FT of its rasterised union.

So the same perimeter turns up in the heat content (spectre_fem), the offset areas (spectre_fields)
and the high-k tail of the diffraction.

Run ``python3 spectre_diffract.py`` to run the self tests and show the figure (--help for options).
Needs numpy and matplotlib.
"""

import argparse
import math
import os
import sys

import numpy as np

import spectre_operator as so
import spectre_patch as sp

__all__ = [
    "form_factor",
    "tile_moments",
    "placed_form_factor",
    "patch_amplitude",
    "diffraction_intensity",
    "porod_constant",
    "run_tests",
    "show_diffraction",
]

K_SMALL = 1e-4  # below this |k| the form factor uses its second-order Taylor expansion


def _check_k(k):
    k = np.asarray(k, dtype=float)
    if k.ndim < 1 or k.shape[-1] != 2 or not np.all(np.isfinite(k)):
        raise ValueError("k must be a finite array of shape (..., 2)")
    return k


def _edge_data(a, b, curves, scale, nodes):
    """Quadrature data of the 14 edges: points X, tangents dX of shape (14, q, 2) and weights."""
    ctrl = so.edge_beziers(a, b, curves, scale)
    x, w = np.polynomial.legendre.leggauss(int(nodes))
    t, w = 0.5 * (x + 1.0), 0.5 * w
    u = 1.0 - t
    basis = np.stack([u**3, 3 * u**2 * t, 3 * u * t**2, t**3], axis=1)
    dbasis = np.stack([-3 * u**2, 3 * (u**2 - 2 * u * t), 3 * (2 * u * t - t**2), 3 * t**2], axis=1)
    return np.einsum("qj,ejd->eqd", basis, ctrl), np.einsum("qj,ejd->eqd", dbasis, ctrl), w


def _orientation_sign(a, b, curves, scale):
    x, y = so.curved_spectre(a, b, curves, scale, samples=20, closed=False)
    return 1.0 if so.polygon_area(x, y) > 0 else -1.0


def tile_moments(a=1.0, b=1.0, curves=1.0, scale=0.5):
    """Area, first moments and second moments of the tile by Green's theorem (exact: the integrands
    are polynomials): returns dict(area, mx, my, mxx, mxy, myy) with m.. = integral of x^i y^j dA."""
    X, dX, w = _edge_data(a, b, curves, scale, 24)
    x, y, dx, dy = X[..., 0], X[..., 1], dX[..., 0], dX[..., 1]
    sign = _orientation_sign(a, b, curves, scale)

    def line(f):  # oriented boundary integral of f, counterclockwise positive
        return sign * float(np.sum(f * w[None, :]))

    return dict(
        area=0.5 * line(x * dy - y * dx),
        mx=0.5 * line(x**2 * dy),
        my=-0.5 * line(y**2 * dx),
        mxx=line(x**3 * dy) / 3.0,
        mxy=0.5 * line(x**2 * y * dy),
        myy=-line(y**3 * dx) / 3.0,
    )


def _auto_nodes(kmax, a, b, scale):
    longest = max(a, b) * (1.0 + 2.0 * abs(scale))  # generous bound for an edge's arclength
    return int(min(512, max(32, 0.8 * kmax * longest + 32)))


def form_factor(k, a=1.0, b=1.0, curves=1.0, scale=0.5, nodes=None, chunk=2048):
    """Fourier transform F(k) of the indicator function of the tile; k has shape (..., 2).

    ``nodes`` is the number of Gauss-Legendre nodes per edge (default: chosen from the largest |k|)."""
    k = _check_k(k)
    flat = k.reshape(-1, 2)
    kmag = np.hypot(flat[:, 0], flat[:, 1])
    nodes = int(nodes) if nodes is not None else _auto_nodes(float(kmag.max(initial=0.0)), a, b, float(np.max(np.abs(so.edge_strengths(curves, scale)))))
    if nodes < 4:
        raise ValueError("nodes must be at least 4")
    X, dX, w = _edge_data(a, b, curves, scale, nodes)
    sign = _orientation_sign(a, b, curves, scale)
    out = np.empty(len(flat), dtype=complex)
    small = kmag < K_SMALL
    mom = tile_moments(a, b, curves, scale)
    ks = flat[small]
    out[small] = (mom["area"] + 1j * (ks[:, 0] * mom["mx"] + ks[:, 1] * mom["my"])
                  - 0.5 * (ks[:, 0] ** 2 * mom["mxx"] + 2 * ks[:, 0] * ks[:, 1] * mom["mxy"] + ks[:, 1] ** 2 * mom["myy"]))
    idx = np.where(~small)[0]
    for start in range(0, len(idx), chunk):
        sel = idx[start:start + chunk]
        kk = flat[sel]
        phase = np.exp(1j * np.einsum("md,eqd->meq", kk, X))
        cross = kk[:, 0, None, None] * dX[None, :, :, 1] - kk[:, 1, None, None] * dX[None, :, :, 0]
        out[sel] = sign * (phase * cross * w[None, None, :]).sum(axis=(1, 2)) / (1j * kmag[sel] ** 2)
    return out.reshape(k.shape[:-1])


def placed_form_factor(placement, k, a=1.0, b=1.0, curves=1.0, scale=0.5, **kw):
    """Form factor of the tile placed by the 3 x 3 affine motion ``placement``:
    |det L| exp(i k.t) F(L^T k)."""
    k = _check_k(k)
    m = np.asarray(placement, dtype=float)
    lin, shift = m[:2, :2], m[:2, 2]
    return abs(np.linalg.det(lin)) * np.exp(1j * (k @ shift)) * form_factor(k @ lin, a, b, curves, scale, **kw)


def patch_amplitude(patch, k, curves=1.0, scale=0.5, **kw):
    """Fourier transform of the union of the tiles of ``patch`` (exact: the tiles are disjoint),
    for a patch whose tiles all carry the same curve amounts.  Tiles are grouped by shape and
    orientation, so only one form factor per group is evaluated."""
    k = _check_k(k)
    flat = k.reshape(-1, 2)
    amp = np.zeros(len(flat), dtype=complex)
    ang = np.round(patch.angles() / 30.0).astype(int) % 12
    for shape in np.unique(patch.shape):
        a, b = (patch.b, patch.a) if shape else (patch.a, patch.b)
        for o in np.unique(ang[patch.shape == shape]):
            members = np.where((patch.shape == shape) & (ang == o))[0]
            lin = patch.placements[members[0], :2, :2]
            trans = patch.placements[members, :2, 2]  # (m, 2)
            structure = np.zeros(len(flat), dtype=complex)
            for start in range(0, len(flat), 4096):
                structure[start:start + 4096] = np.exp(1j * (flat[start:start + 4096] @ trans.T)).sum(axis=1)
            amp += structure * form_factor(flat @ lin, a, b, curves, scale, **kw)
    return amp.reshape(k.shape[:-1])


def diffraction_intensity(patch, k, curves=1.0, scale=0.5, **kw):
    """|FT of the patch|^2 per tile."""
    return np.abs(patch_amplitude(patch, k, curves, scale, **kw)) ** 2 / patch.n_tiles


def porod_constant(radii, a=1.0, b=1.0, curves=1.0, scale=0.5, n_angles=240):
    """Angular mean of k^3 |F(k)|^2 at each radius in ``radii``; it oscillates around 2 P (Porod's law
    for a sharp boundary of perimeter P) with a decaying amplitude."""
    radii = np.asarray(radii, dtype=float)
    if radii.ndim != 1 or np.any(radii <= 0) or n_angles < 8:
        raise ValueError("radii must be a 1-D array of positive numbers and n_angles at least 8")
    th = np.linspace(0.0, 2.0 * np.pi, int(n_angles), endpoint=False)
    out = np.empty(len(radii))
    for i, r in enumerate(radii):
        kk = r * np.column_stack([np.cos(th), np.sin(th)])
        out[i] = np.mean(np.abs(form_factor(kk, a, b, curves, scale)) ** 2) * r**3
    return out


# --------------------------------------------------------------------------
# Self tests
# --------------------------------------------------------------------------
def _rel(x, ref):
    return abs(x - ref) / abs(ref)


def _polygon_form_factor(poly, k):
    """Exact FT of a polygon (counterclockwise or clockwise), independent of the Bezier machinery:
    sum over edges of the closed form of  integral exp(i k.x) (k x dx)/(i k^2)  along a segment."""
    k = np.asarray(k, float)
    k2 = k @ k
    sign = 1.0 if so.polygon_area(*poly.T) > 0 else -1.0
    total = 0.0j
    for p, q in zip(poly, np.roll(poly, -1, axis=0)):
        d = q - p
        kd = k @ d
        seg = np.exp(1j * (k @ p)) * ((np.exp(1j * kd) - 1.0) / (1j * kd) if abs(kd) > 1e-12 else 1.0)
        total += seg * (k[0] * d[1] - k[1] * d[0])
    return sign * total / (1j * k2)


def _raster_ft(polys, ks, h=0.01):
    count, xs, ys = sp._coverage_grid(polys, h=h, margin=0.1)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack([gx.ravel(), gy.ravel()])[count.ravel() >= 1]
    return np.array([np.sum(np.exp(1j * pts @ np.asarray(k))) * h * h for k in ks])


def test_small_k_limit_gives_the_area_and_the_moments_of_the_tile():
    A = so.spectre_area()
    for scale in (0.0, 0.5):
        mom = tile_moments(1, 1, 1.0, scale)
        assert _rel(mom["area"], A) < 1e-12
        # independent: moments by fine shoelace-type sums of the sampled outline
        x, y = so.curved_spectre(1, 1, 1.0, scale, samples=4000, closed=False)
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cross = x * y1 - x1 * y
        sign = np.sign(so.polygon_area(x, y))
        mx = sign * np.sum((x + x1) * cross) / 6.0
        my = sign * np.sum((y + y1) * cross) / 6.0
        assert abs(mom["mx"] - mx) < 2e-6 and abs(mom["my"] - my) < 2e-6
        # F(k) = A + i k.M1 - k^T M2 k / 2 + O(k^3)
        for k in ([1e-3, 0.0], [0.0, 2e-3], [1e-3, -1e-3]):
            k = np.array(k)
            f = form_factor(k, 1, 1, 1.0, scale, nodes=64)
            taylor = (mom["area"] + 1j * (k[0] * mom["mx"] + k[1] * mom["my"])
                      - 0.5 * (k[0] ** 2 * mom["mxx"] + 2 * k[0] * k[1] * mom["mxy"] + k[1] ** 2 * mom["myy"]))
            assert abs(f - taylor) < 1e-7, (k, f, taylor)  # boundary formula, just above K_SMALL scale
        assert abs(form_factor(np.zeros(2), 1, 1, 1.0, scale) - A) < 1e-12
        # continuity across the switch between the Taylor series and the boundary integral
        lo = form_factor(np.array([(1.0 - 1e-6) * K_SMALL, 0.0]), 1, 1, 1.0, scale)  # series
        hi = form_factor(np.array([(1.0 + 1e-6) * K_SMALL, 0.0]), 1, 1, 1.0, scale)  # boundary integral
        assert abs(lo - hi) < 1e-6 * A


def test_straight_tile_matches_the_exact_polygon_formula():
    poly = so.vertices(1, 1)
    rng = np.random.default_rng(3)
    for k in rng.uniform(-8, 8, (6, 2)):
        assert abs(form_factor(k, 1, 1, 1.0, 0.0) - _polygon_form_factor(poly, k)) < 1e-9
    poly2 = so.vertices(1, 2.3)
    for k in rng.uniform(-5, 5, (3, 2)):
        assert abs(form_factor(k, 1, 2.3, 1.0, 0.0) - _polygon_form_factor(poly2, k)) < 1e-9


def test_curved_tile_matches_rasterisation_and_quadrature_converges():
    x, y = so.curved_spectre(1, 1, 1.0, 0.5, samples=200, closed=False)
    poly = np.column_stack([x, y])
    ks = [[1.0, 0.5], [3.0, -2.0], [6.0, 4.0]]
    raster = _raster_ft([poly], ks, h=0.01)
    for k, r in zip(ks, raster):
        assert abs(form_factor(np.array(k), 1, 1, 1.0, 0.5) - r) < 3e-3 * so.spectre_area() / 2, (k, r)
    # more quadrature nodes change nothing: Gauss-Legendre has converged
    big = np.array([[25.0, 7.0], [-12.0, 30.0]])
    assert np.abs(form_factor(big, nodes=96) - form_factor(big, nodes=300)).max() < 1e-8
    # symmetries: F(-k) = conj F(k), |F| <= area
    k = np.random.default_rng(4).uniform(-10, 10, (20, 2))
    f = form_factor(k)
    assert np.allclose(form_factor(-k), np.conj(f), atol=1e-10)
    assert np.all(np.abs(f) <= so.spectre_area() + 1e-9)


def test_placed_form_factor_includes_rotation_mirror_and_translation():
    k = np.array([[1.3, -0.4], [4.0, 2.5], [-7.0, 3.0]])
    for ang, mirror, shift in ((37.0, False, (1.5, -2.0)), (200.0, True, (-3.0, 0.7))):
        c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        lin = np.array([[c, -s], [s, c]]) @ (np.diag([-1.0, 1.0]) if mirror else np.eye(2))
        m = np.eye(3)
        m[:2, :2], m[:2, 2] = lin, shift
        # direct: transform the outline's Bezier control points and integrate the new outline
        ctrl = so.edge_beziers(1, 1, 1.0, 0.5) @ lin.T + np.array(shift)
        x, dx, w = [], [], None
        t, w = np.polynomial.legendre.leggauss(80)
        t, w = 0.5 * (t + 1.0), 0.5 * w
        u = 1.0 - t
        basis = np.stack([u**3, 3 * u**2 * t, 3 * u * t**2, t**3], axis=1)
        dbasis = np.stack([-3 * u**2, 3 * (u**2 - 2 * u * t), 3 * (2 * u * t - t**2), 3 * t**2], axis=1)
        X, dX = np.einsum("qj,ejd->eqd", basis, ctrl), np.einsum("qj,ejd->eqd", dbasis, ctrl)
        # orientation of the new outline follows from the signed area of its polygon
        verts = ctrl[:, 0]
        sign = 1.0 if so.polygon_area(*verts.T) > 0 else -1.0
        for kv in k:
            phase = np.exp(1j * (X @ kv))
            direct = sign * np.sum(phase * (kv[0] * dX[..., 1] - kv[1] * dX[..., 0]) * w[None, :]) / (1j * (kv @ kv))
            assert abs(placed_form_factor(m, kv) - direct) < 1e-8, (ang, mirror, kv)


def test_porod_law_the_tail_of_the_form_factor_is_the_perimeter():
    radii = np.linspace(18.0, 45.0, 28)
    for scale in (0.5, 0.0):
        P = so.perimeter(1, 1, 1.0, scale)
        mean = porod_constant(radii, 1, 1, 1.0, scale).mean()
        assert _rel(mean, 2.0 * P) < 0.02, (scale, mean, 2.0 * P)
    # the perimeters of the two tiles differ by far more than the tolerance
    assert so.perimeter(1, 1, 1.0, 0.5) > 1.15 * so.perimeter(1, 1, 1.0, 0.0)


def test_plancherel_identity_with_the_porod_tail():
    # integral |F|^2 d^2k / (2 pi)^2 = area; beyond |k| = K the Porod law adds P / (pi K)
    A, P = so.spectre_area(), so.perimeter(1, 1, 1.0, 0.5)
    K = 20.0
    x, w = np.polynomial.legendre.leggauss(160)
    r, wr = 0.5 * K * (x + 1.0), 0.5 * K * w
    th = np.linspace(0.0, 2.0 * np.pi, 120, endpoint=False)
    total = 0.0
    for ri, wi in zip(r, wr):
        kk = ri * np.column_stack([np.cos(th), np.sin(th)])
        total += wi * ri * np.mean(np.abs(form_factor(kk)) ** 2) * 2.0 * np.pi
    value = total / (2.0 * np.pi) ** 2 + P / (np.pi * K)
    assert _rel(value, A) < 0.01, (value, A)


def test_patch_amplitude_is_the_fourier_transform_of_the_union():
    patch = sp.SpectrePatch(1)
    polys = patch.polygons(1.0, 0.5, samples=50)
    ks = [[0.0, 0.0], [0.4, 0.3], [1.2, -0.8], [2.0, 1.5], [-3.0, 1.0]]
    raster = _raster_ft(polys, ks, h=0.01)
    amp = patch_amplitude(patch, np.array(ks), 1.0, 0.5)
    assert abs(amp[0] - patch.area) < 1e-9  # curving keeps the area
    assert np.abs(amp - raster).max() < 0.01 * patch.area, (amp, raster)
    # a != b: Mystic tiles with their own form factor
    p2 = sp.SpectrePatch(1, 1.0, 1.7)
    a2 = patch_amplitude(p2, np.array([[0.0, 0.0], [0.7, -0.4]]), 1.0, 0.4)
    assert abs(a2[0] - p2.area) < 1e-8
    r2 = _raster_ft(p2.polygons(1.0, 0.4, samples=50), [[0.7, -0.4]], h=0.01)
    assert abs(a2[1] - r2[0]) < 0.01 * p2.area
    # intensity is |amp|^2 per tile
    assert abs(diffraction_intensity(patch, np.array([[0.4, 0.3]]), 1.0, 0.5)[0] - abs(amp[1]) ** 2 / patch.n_tiles) < 1e-9


def test_validation():
    for call in (
        lambda: form_factor(np.array([1.0, 2.0, 3.0])),
        lambda: form_factor(np.array([float("nan"), 0.0])),
        lambda: form_factor(np.array([1.0, 1.0]), nodes=2),
        lambda: porod_constant(np.array([-1.0])),
        lambda: porod_constant(np.array([[10.0]])),
        lambda: porod_constant(np.array([10.0]), n_angles=3),
        lambda: patch_amplitude(sp.SpectrePatch(0), np.array([1.0])),
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
            print("  %-74s %s" % (name, status))
    if verbose:
        print("%d test(s), %d failed" % (len(tests), failed))
    return failed == 0


# --------------------------------------------------------------------------
# Figure
# --------------------------------------------------------------------------
def show_diffraction(a=1.0, b=1.0, k=0.5, levels=3, save=None, show=True):
    """Six panels: |F|^2 of the curved and of the straight tile; the diffraction of a curved and of
    a straight patch; Porod's law (k^3 <|F|^2> -> 2 P); Plancherel's identity with the Porod tail."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm

    A = so.spectre_area(a, b)
    patch = sp.SpectrePatch(levels, a, b)
    fig, axes = plt.subplots(2, 3, figsize=(17, 10.6))

    def grid(extent, n):
        ax_ = np.linspace(-extent, extent, n)
        gx, gy = np.meshgrid(ax_, ax_)
        return np.stack([gx, gy], axis=-1), ax_

    # 1-2. tile form factors
    kk, axis = grid(10.0, 281)
    for ax, scale, name in ((axes[0, 0], k, "curved tile (k = %g)" % k), (axes[0, 1], 0.0, "straight tile")):
        f2 = np.abs(form_factor(kk, a, b, 1.0, scale)) ** 2
        im = ax.imshow(f2, extent=(-10, 10, -10, 10), origin="lower", cmap="magma",
                       norm=LogNorm(vmin=1e-6 * A * A, vmax=A * A))
        ax.set_title("|F(k)|^2 of the %s (log scale, area^2 at the centre)" % name, fontsize=9)
        ax.set_xlabel("k_x")
        ax.set_ylabel("k_y")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)

    # 3-4. patch diffraction
    kk2, axis2 = grid(5.0, 221)
    for ax, scale, name in ((axes[0, 2], k, "curved"), (axes[1, 0], 0.0, "straight")):
        inten = diffraction_intensity(patch, kk2, 1.0, scale)
        im2 = ax.imshow(inten, extent=(-5, 5, -5, 5), origin="lower", cmap="inferno",
                        norm=LogNorm(vmin=max(inten.max() * 1e-6, 1e-3), vmax=inten.max()))
        ax.set_title("diffraction of %d %s tiles:\n|FT of the patch|^2 per tile (log scale)" % (patch.n_tiles, name), fontsize=9)
        ax.set_xlabel("k_x")
        ax.set_ylabel("k_y")
        fig.colorbar(im2, ax=ax, fraction=0.046, pad=0.03)

    # 5. Porod
    ax = axes[1, 1]
    radii = np.linspace(6.0, 45.0, 79)
    for scale, col, name in ((k, "tab:red", "curved"), (0.0, "tab:blue", "straight")):
        P = so.perimeter(a, b, 1.0, scale)
        raw = porod_constant(radii, a, b, 1.0, scale)
        ax.plot(radii, raw, color=col, lw=0.7, alpha=0.45)
        smooth = np.convolve(raw, np.ones(9) / 9.0, mode="valid")
        ax.plot(radii[4:-4], smooth, color=col, lw=2.0, label="%s: k^3 <|F|^2> (running mean)" % name)
        ax.axhline(2 * P, color=col, ls="--", lw=1.0, label="%s: 2 P = %.2f" % (name, 2 * P))
    ax.set_xlabel("|k|")
    ax.set_ylabel("angular mean of k^3 |F|^2")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_title("Porod's law: the high-k tail measures the exact perimeter,\n"
                 "so the diffraction tail tells the curved tile from the straight one", fontsize=9)

    # 6. Plancherel
    ax = axes[1, 2]
    rr = np.linspace(0.05, 30.0, 300)
    th = np.linspace(0.0, 2.0 * np.pi, 96, endpoint=False)
    for scale, col, name in ((k, "tab:red", "curved"), (0.0, "tab:blue", "straight")):
        radial = np.array([np.mean(np.abs(form_factor(r * np.column_stack([np.cos(th), np.sin(th)]), a, b, 1.0, scale)) ** 2) for r in rr])
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (radial[1:] * rr[1:] + radial[:-1] * rr[:-1]) * np.diff(rr))]) / (2 * math.pi)
        tail = so.perimeter(a, b, 1.0, scale) / (math.pi * rr)
        ax.plot(rr, cum, color=col, lw=1.4, label="%s: integral up to K" % name)
        late = rr >= 6.0
        ax.plot(rr[late], (cum + tail)[late], color=col, lw=1.4, ls=":", label="%s: + Porod tail P/(pi K)" % name)
    ax.axhline(A, color="k", ls="--", lw=0.9)
    ax.text(0.5, A, " area (Plancherel)", va="bottom", fontsize=8)
    ax.set_xlabel("cutoff K")
    ax.set_ylabel("integral |F|^2 d^2k / (2 pi)^2")
    ax.set_ylim(0, 1.15 * A)
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(alpha=0.3)
    ax.set_title("Plancherel: the energy in |k| < K converges to the area,\nfaster once the Porod tail is added", fontsize=9)

    fig.suptitle("Form factor and diffraction of the curved Spectre monotile (a=%g, b=%g): Green's theorem along the Bezier edges, "
                 "exact sums over a %d-tile patch" % (a, b, patch.n_tiles), fontsize=11)
    fig.subplots_adjust(left=0.05, right=0.97, top=0.9, bottom=0.07, wspace=0.28, hspace=0.3)
    if save:
        os.makedirs(os.path.dirname(os.path.abspath(save)), exist_ok=True)
        fig.savefig(save, dpi=100)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


# --------------------------------------------------------------------------
def main(argv=None):
    p = argparse.ArgumentParser(description="Form factor and diffraction of the curved Spectre monotile: self tests and a figure.")
    p.add_argument("--a", type=float, default=1.0, help="edge length a (default 1)")
    p.add_argument("--b", type=float, default=1.0, help="edge length b (default 1)")
    p.add_argument("--curve_strength", type=float, default=0.5, help="uniform curve strength k (default 0.5)")
    p.add_argument("--levels", type=int, default=3, help="substitution levels of the patch (default 3)")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_diffract self tests:")
        if not run_tests():
            return 1
    if args.tests_only:
        return 0
    show_diffraction(args.a, args.b, args.curve_strength, args.levels, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
