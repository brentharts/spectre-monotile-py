"""
spectre_fractal.py - what is (and is not) fractal in the diffraction of the Spectre (step 6).

The folded, brain-like ridges in the form-factor plots of spectre_diffract.py are the dark valleys of
|F(k)|^2 of ONE tile.  F is an entire (real-analytic) function of k, so those valleys are smooth curves:
their dimension is 1 and no scaling range with a non-integer dimension exists.  The module measures
this, and then measures the objects around the picture that really are fractal:

  A. the folds of one tile (``tile_field``, ``phase_vortices``, ``valley_box_counts``)
       F is complex, so |F|^2 vanishes only at isolated points (phase vortices, charge +1 or -1) - the
       dark lines are valleys that run between them and never touch zero.  Vortices are found by the
       winding of the phase around grid cells and refined by Newton's method; their density is measured.
       Box counting of the valley set gives the local dimension as a function of the box size: 2 below the
       valley width, a plateau at 1 (a curve), 2 again above the spacing of the folds - no fractal range.

  B. the outline of a supertile (``outline_segments``, ``boundary_counts``, ``integer_recurrence``)
       The union of the tiles of a level-n patch is a supertile, and its outline is a fractal curve (it is
       what shapes the diffraction pattern of panels 3-4).  The number of outline edges B_n obeys an exact
       integer recurrence  B_(n+3) = 5 B_(n+2) - 3 B_(n+1) - B_n  with characteristic polynomial
       (x - 1)(x^2 - 4x - 1), so B_n grows by r = 2 + sqrt(5) per level while the linear size grows by
       sqrt(4 + sqrt(15)) per level.  The outline is self-similar with an open set condition (graph
       directed), so its Hausdorff and box dimension both equal

            D = ln(2 + sqrt(5)) / ln sqrt(4 + sqrt(15)) = 1.3993 .

       (Fitting the recurrence is numerical evidence over levels 0..6, not a proof; the open set
       condition is assumed, not proved.)  Direct box counting of the outline approaches this from below
       and slowly (finite patches, crossovers): ``box_count_points``.

  C. what the diffraction pattern does with it (``radial_intensity``, ``window_slopes``, ``track_peak``)
       A fractal outline of dimension D makes the orientation-averaged intensity decay like k^-(4-D)
       between 1/R and the tile scale, instead of the k^-3 of a smooth boundary (Porod).  Measured slopes
       scatter strongly with the fit window (log periodic modulation with period ln sqrt(4+sqrt(15)) in
       ln k), so they are reported as a mean over windows and only compared loosely with the exact D.
       ``track_peak`` follows a local maximum of the per-tile intensity through the patch levels:
       the exponent alpha of I_max ~ N^alpha is 1 for a Bragg peak and 0 for diffuse scattering.

What the self tests check (run ``python3 spectre_fractal.py --tests-only``):
  * box counting reproduces the exact dimensions of a line (1), a filled square (2) and the Sierpinski
    triangle (ln 3 / ln 2), on a bitmap;
  * the vortex finder recovers the known zeros and charges of a synthetic complex function, and every
    vortex of the real tile refines to a true zero (|F| < 1e-8 area) inside the cell where it was found,
    with the same zeros at two grid resolutions;
  * the outline edge counts of levels 0..5 equal the stored regression values, the integer recurrence
    fitted to them predicts level 6 (stored, computed once with ``--deep``), its growth rate is 2+sqrt(5),
    and the outline of every level is one closed simple loop of unit edges;
  * the box counts of the geometric outline grow from level to level by about r, and D lies strictly
    between 1 and 2;
  * the valley set of a tile has no fractal range: the local box dimension is within 0.15 of 1 on a plateau
    and 2 at both ends;
  * the peak exponent is exactly 1 for a square array at a Bragg point, about 0 for the same squares at
    random positions, and exactly 1 at the origin of the Spectre patch (I(0) = N A^2).

Run ``python3 spectre_fractal.py`` to run the self tests and show the figure (--help for options,
--peaks also tracks the strongest diffraction maxima through the levels, which takes a few minutes).
Needs numpy, scipy and matplotlib.
"""

import argparse
import math
import os
import sys
from fractions import Fraction

import numpy as np

import spectre_diffract as sd
import spectre_operator as so
import spectre_patch as sp

LAMBDA = 4.0 + math.sqrt(15.0)  # tiles per tile and area ratio per level; linear ratio is sqrt(LAMBDA)
GROWTH = 2.0 + math.sqrt(5.0)  # growth of the number of outline edges per level
D_OUTLINE = 2.0 * math.log(GROWTH) / math.log(LAMBDA)  # ln(GROWTH) / ln(sqrt(LAMBDA))

# number of outline edges (edges without partner) of the level 0..6 patch; level 6 takes 80 s (--deep)
BOUNDARY_COUNTS = (14, 46, 182, 758, 3198, 13534, 57318)


# --------------------------------------------------------------------------
# A. the folds of one tile: phase vortices and valley sets
# --------------------------------------------------------------------------
def tile_field(a=1.0, b=1.0, curves=1.0, scale=0.5, extent=10.0, n=401):
    """Form factor of the tile on the square grid [-extent, extent]^2 with n points per side.
    Returns (axis, F) with F[iy, ix] = F(axis[ix], axis[iy])."""
    if n < 8 or extent <= 0:
        raise ValueError("need n >= 8 and extent > 0")
    axis = np.linspace(-extent, extent, int(n))
    gx, gy = np.meshgrid(axis, axis)
    return axis, sd.form_factor(np.stack([gx, gy], axis=-1), a, b, curves, scale)


def _wrap(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def phase_vortices(field, axis):
    """Zeros of a complex field on a grid, found by the winding of its phase around every cell.

    Returns (positions, charges): the cell centres (m, 2) and the integer winding numbers (+1 or -1)."""
    field = np.asarray(field)
    if field.ndim != 2 or field.shape[0] != field.shape[1] or len(axis) != field.shape[0]:
        raise ValueError("field must be square and match the axis")
    ph = np.angle(field)
    w = (_wrap(ph[:-1, 1:] - ph[:-1, :-1]) + _wrap(ph[1:, 1:] - ph[:-1, 1:])
         + _wrap(ph[1:, :-1] - ph[1:, 1:]) + _wrap(ph[:-1, :-1] - ph[1:, :-1]))
    q = np.rint(w / (2.0 * np.pi)).astype(int)
    iy, ix = np.nonzero(q)
    h = axis[1] - axis[0]
    pos = np.column_stack([axis[ix] + 0.5 * h, axis[iy] + 0.5 * h])
    return pos, q[iy, ix]


def refine_zero(func, k0, step=1e-5, iterations=40, max_move=None):
    """Newton's method for a zero of a complex function of k = (kx, ky) (``func`` takes an array (..., 2)).

    Returns (k, |func(k)|)."""
    k = np.array(k0, dtype=float)
    for _ in range(iterations):
        f0 = func(k[None, :])[0]
        if abs(f0) < 1e-13:
            break
        jac = np.empty((2, 2))
        for d in range(2):
            e = np.zeros(2)
            e[d] = step
            diff = (func((k + e)[None, :])[0] - func((k - e)[None, :])[0]) / (2.0 * step)
            jac[:, d] = (diff.real, diff.imag)
        try:
            move = np.linalg.solve(jac, -np.array([f0.real, f0.imag]))
        except np.linalg.LinAlgError:
            break
        if max_move is not None and np.hypot(*move) > max_move:
            move *= max_move / np.hypot(*move)
        k = k + move
    return k, abs(func(k[None, :])[0])


def normalised_intensity(field, axis, n_bins=60):
    """|F|^2 divided by its mean over the circle of the same radius (a smooth profile), so that
    'dark' means dark relative to the neighbourhood.  Returns (Z, radius grid)."""
    gx, gy = np.meshgrid(axis, axis)
    r = np.hypot(gx, gy)
    inten = np.abs(field) ** 2
    edges = np.linspace(0.0, r.max() * (1 + 1e-9), n_bins + 1)
    idx = np.clip(np.digitize(r, edges) - 1, 0, n_bins - 1)
    sums = np.bincount(idx.ravel(), weights=inten.ravel(), minlength=n_bins)
    cnt = np.bincount(idx.ravel(), minlength=n_bins)
    prof = sums / np.maximum(cnt, 1)
    return inten / prof[idx], r


def box_count_image(mask, sizes):
    """Number of boxes of side s (in pixels) that contain a True pixel, for every s in ``sizes``
    (boxes are aligned with the image corner; the partial boxes at the edge are included)."""
    mask = np.asarray(mask, dtype=bool)
    out = []
    for s in sizes:
        s = int(s)
        if s < 1:
            raise ValueError("box sizes must be at least 1 pixel")
        ny, nx = (-(-mask.shape[0] // s)) * s, (-(-mask.shape[1] // s)) * s
        padded = np.zeros((ny, nx), dtype=bool)
        padded[:mask.shape[0], :mask.shape[1]] = mask
        out.append(int(padded.reshape(ny // s, s, nx // s, s).any(axis=(1, 3)).sum()))
    return np.array(out)


def local_dimension(sizes, counts):
    """Local box dimension  -d ln N / d ln s  from consecutive box sizes."""
    return -np.gradient(np.log(np.asarray(counts, float)), np.log(np.asarray(sizes, float)))


def valley_box_counts(field, axis, eps=0.1, r_range=(3.0, 10.0), sizes=(1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48)):
    """Box counting of the valley set {|F|^2 < eps * (mean |F|^2 at that radius)} in an annulus of k.

    Returns (box side in k units, counts, local dimension, area fraction of the valley set)."""
    z, r = normalised_intensity(field, axis)
    ann = (r >= r_range[0]) & (r <= r_range[1])
    valley = (z < eps) & ann
    counts = box_count_image(valley, sizes)
    side = np.asarray(sizes, float) * (axis[1] - axis[0])
    return side, counts, local_dimension(side, counts), valley.sum() / max(ann.sum(), 1)


def vortex_density(positions, radii):
    """Number of vortices inside |k| < K divided by the disc area, for every K in ``radii``."""
    r = np.hypot(positions[:, 0], positions[:, 1])
    return np.array([(r < K).sum() / (math.pi * K * K) for K in radii])


# --------------------------------------------------------------------------
# B. the outline of a supertile
# --------------------------------------------------------------------------
def outline_segments(patch):
    """Segments (m, 2, 2) of the straight outline of the patch: the edges without a partner tile."""
    segs = []
    for t, e in patch.boundary_edges():
        v = patch.straight_polygon(t)
        segs.append((v[e], v[(e + 1) % so.N_EDGES]))
    return np.array(segs)


def outline_loops(segments, decimals=6):
    """Number of closed loops of the outline, and whether every vertex has exactly two outline edges."""
    pts = np.round(segments.reshape(-1, 2), decimals)
    key = {}
    for p in map(tuple, pts):
        key.setdefault(p, len(key))
    ends = np.array([key[tuple(p)] for p in pts]).reshape(-1, 2)
    deg = np.bincount(ends.ravel(), minlength=len(key))
    parent = list(range(len(key)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for u, v in ends:
        parent[find(u)] = find(v)
    return len({find(i) for i in range(len(key))}), bool(np.all(deg == 2))


def boundary_counts(max_level=5, a=1.0, b=1.0):
    """Number of outline edges of the patches of level 0 .. max_level."""
    return [len(sp.SpectrePatch(level, a, b).boundary_edges()) for level in range(max_level + 1)]


def integer_recurrence(seq, order=3):
    """Exact rational linear recurrence  s[i+order] = sum_j c[j] s[i+j]  fitted to the first ``order``
    equations of the sequence.  Returns (coefficients as Fractions, residuals of ALL equations)."""
    seq = [int(x) for x in seq]
    n_eq = len(seq) - order
    if n_eq < order:
        raise ValueError("need at least 2 * order terms")
    rows = [[Fraction(seq[i + j]) for j in range(order)] + [Fraction(seq[i + order])] for i in range(order)]
    for col in range(order):  # Gauss-Jordan elimination over the rationals
        piv = next((r for r in range(col, order) if rows[r][col] != 0), None)
        if piv is None:
            raise ValueError("singular system: the sequence satisfies a shorter recurrence")
        rows[col], rows[piv] = rows[piv], rows[col]
        rows[col] = [x / rows[col][col] for x in rows[col]]
        for r in range(order):
            if r != col and rows[r][col] != 0:
                f = rows[r][col]
                rows[r] = [x - f * y for x, y in zip(rows[r], rows[col])]
    coef = [rows[j][order] for j in range(order)]
    resid = [sum(coef[j] * seq[i + j] for j in range(order)) - seq[i + order] for i in range(n_eq)]
    return coef, resid


def growth_rate(coef):
    """Largest root of  x^order = sum_j c[j] x^j."""
    order = len(coef)
    poly = [1.0] + [-float(coef[order - 1 - i]) for i in range(order)]
    return float(max(abs(np.roots(poly))))


def box_count_points(segments, sizes, step=0.02):
    """Box counts of the outline (sampled every ``step``) for boxes of side s in ``sizes`` (grid aligned)."""
    segments = np.asarray(segments, float)
    length = np.linalg.norm(segments[:, 1] - segments[:, 0], axis=1)
    m = int(math.ceil(length.max() / step))
    u = (np.arange(m) + 0.5) / m
    pts = (segments[:, 0][:, None, :] * (1 - u)[None, :, None] + segments[:, 1][:, None, :] * u[None, :, None]).reshape(-1, 2)
    out = []
    for s in sizes:
        ij = np.floor(pts / s).astype(np.int64)
        out.append(len(np.unique(ij, axis=0)))
    return np.array(out)


# --------------------------------------------------------------------------
# C. diffraction of the fractal outline
# --------------------------------------------------------------------------
def radial_intensity(patch, radii, n_angles=96, scale=0.0, a=1.0, b=1.0):
    """Orientation average of |FT of the patch|^2 at every radius in ``radii`` (for a straight patch by
    default, so that the only boundary structure is the outline of the supertile)."""
    th = np.linspace(0.0, 2.0 * np.pi, int(n_angles), endpoint=False)
    out = []
    for r in radii:
        kk = r * np.column_stack([np.cos(th), np.sin(th)])
        out.append(np.mean(np.abs(sd.patch_amplitude(patch, kk, 1.0, scale)) ** 2))
    return np.array(out)


def window_slopes(radii, inten, starts=(0.01, 0.02, 0.03, 0.04), periods=2):
    """Log-log slope of ``inten`` in windows [k0, k0 sqrt(LAMBDA)^periods]: a whole number of periods of
    the log-periodic modulation.  Returns a list of (k0, k1, slope)."""
    ratio = math.exp(periods * 0.5 * math.log(LAMBDA))
    out = []
    for k0 in starts:
        k1 = k0 * ratio
        m = (radii >= k0) & (radii <= k1)
        if k1 <= radii.max() and m.sum() >= 5:
            out.append((k0, k1, float(np.polyfit(np.log(radii[m]), np.log(inten[m]), 1)[0])))
    return out


def peak_exponent(n_tiles, intensity_per_tile):
    """Exponent alpha of  I ~ N^alpha  (least squares in log-log); 1 for a Bragg peak, 0 for diffuse
    scattering."""
    return float(np.polyfit(np.log(np.asarray(n_tiles, float)), np.log(np.asarray(intensity_per_tile, float)), 1)[0])


def track_peak(patches, k0, width0=0.02):
    """Follow a local maximum of the per-tile intensity (scale 0) from the first patch to the last, by
    Nelder-Mead from the previous position.  Returns (k final, list of (n_tiles, I_max))."""
    from scipy.optimize import minimize

    k = np.array(k0, float)
    track = []
    for i, p in enumerate(patches):
        w = width0 / (3.0 ** i)

        def neg(q, p=p):
            return -float(sd.diffraction_intensity(p, np.asarray(q, float).reshape(1, 2), 1.0, 0.0)[0])

        res = minimize(neg, k, method="Nelder-Mead",
                       options=dict(xatol=1e-5, fatol=1e-9, initial_simplex=np.array([k, k + [w, 0.0], k + [0.0, w]])))
        k = res.x
        track.append((p.n_tiles, -res.fun))
    return k, track


def square_array_intensity(n, side, k, jitter=None, seed=0):
    """Per-tile intensity |amplitude|^2 / N at ``k`` of an n x n array of squares of the given side, at
    the lattice points (jitter=None) or at uniformly random positions in [0, n]^2."""
    k = np.asarray(k, float)
    if jitter is None:
        gx, gy = np.meshgrid(np.arange(n), np.arange(n))
        pos = np.column_stack([gx.ravel(), gy.ravel()]).astype(float)
    else:
        pos = np.random.default_rng(seed).uniform(0.0, n, size=(n * n, 2))

    def sq(q):
        return side if abs(q) < 1e-12 else (np.exp(1j * q * side) - 1.0) / (1j * q)

    amp = np.exp(1j * (pos @ k)).sum() * sq(k[0]) * sq(k[1])
    return float(abs(amp) ** 2 / len(pos))


# --------------------------------------------------------------------------
# Self tests
# --------------------------------------------------------------------------
def test_box_counting_gives_the_exact_dimensions_of_known_sets():
    n = 512
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    sizes = (1, 2, 4, 8, 16, 32)
    sierpinski = (i & j) == 0
    counts = box_count_image(sierpinski, sizes)
    assert list(counts) == [3 ** (9 - m) for m in range(6)], counts  # exactly 3^(9-m)
    d = local_dimension(sizes, counts)[1:-1]
    assert np.allclose(d, math.log(3) / math.log(2), atol=1e-12), d
    line = (i == j)
    assert np.allclose(local_dimension(sizes, box_count_image(line, sizes)), 1.0, atol=1e-12)
    filled = np.ones((n, n), bool)
    assert np.allclose(local_dimension(sizes, box_count_image(filled, sizes)), 2.0, atol=1e-12)
    assert box_count_image(np.zeros((9, 9), bool), (3,))[0] == 0
    # partial boxes at the edge are counted
    assert box_count_image(np.ones((5, 5), bool), (2,))[0] == 9


def test_vortex_finder_recovers_known_zeros_and_charges():
    axis = np.linspace(-3.0, 3.0, 121)
    gx, gy = np.meshgrid(axis, axis)
    z = gx + 1j * gy
    # zeros at 1+0.5i (charge +1) and -1.2-0.7i (charge -1, through conj), plus a smooth envelope
    f = (z - (1.0 + 0.5j)) * np.conj(z - (-1.2 - 0.7j)) * np.exp(-0.1 * np.abs(z) ** 2)
    pos, q = phase_vortices(f, axis)
    assert len(q) == 2 and sorted(q) == [-1, 1], (pos, q)
    h = axis[1] - axis[0]
    for p, c in zip(pos, q):
        target = np.array([1.0, 0.5]) if c > 0 else np.array([-1.2, -0.7])
        assert np.hypot(*(p - target)) < h, (p, target)

    def fun(kk):
        zz = kk[..., 0] + 1j * kk[..., 1]
        return (zz - (1.0 + 0.5j)) * np.conj(zz - (-1.2 - 0.7j))

    k, res = refine_zero(fun, pos[0])
    assert res < 1e-12 and min(np.hypot(*(k - np.array([1.0, 0.5]))), np.hypot(*(k - np.array([-1.2, -0.7])))) < 1e-9
    try:
        phase_vortices(np.ones((4, 5)), np.arange(4.0))
    except ValueError:
        pass
    else:
        raise AssertionError("non-square field must be rejected")


def test_every_vortex_of_the_tile_is_a_true_zero_found_at_both_resolutions():
    area = so.spectre_area(1.0, 1.0)
    found = []
    for n in (241, 361):
        axis, f = tile_field(1.0, 1.0, 1.0, 0.5, 6.0, n)
        pos, q = phase_vortices(f, axis)
        h = axis[1] - axis[0]
        inside = np.hypot(pos[:, 0], pos[:, 1]) < 5.5
        pos, q = pos[inside], q[inside]
        assert len(q) > 10 and q.min() == -1 and q.max() == 1
        zs = []
        for p in pos:
            k, res = refine_zero(lambda kk: sd.form_factor(kk, 1, 1, 1.0, 0.5), p, max_move=h)
            assert res < 1e-8 * area, (p, res)
            assert np.hypot(*(k - p)) < 1.5 * h, (p, k)
            zs.append(k)
        found.append(np.array(zs))
    assert len(found[0]) == len(found[1]), [len(x) for x in found]
    a, b = (x[np.lexsort((x[:, 1], x[:, 0]))] for x in found)
    assert np.max(np.abs(a - b)) < 1e-6, np.max(np.abs(a - b))  # the same zeros, to Newton accuracy
    # straight tile too, and its vortices are as dense (same order) as those of the curved tile
    axis, fs = tile_field(1.0, 1.0, 1.0, 0.0, 6.0, 241)
    ps, qs = phase_vortices(fs, axis)
    n_straight = int((np.hypot(ps[:, 0], ps[:, 1]) < 5.5).sum())
    assert 0.5 < n_straight / len(found[0]) < 2.0, (n_straight, len(found[0]))


def test_outline_is_one_simple_closed_loop_and_edge_counts_match_the_regression_values():
    counts = boundary_counts(4)
    assert counts == list(BOUNDARY_COUNTS[:5]), counts
    for level in (1, 2, 3):
        segs = outline_segments(sp.SpectrePatch(level))
        assert len(segs) == BOUNDARY_COUNTS[level]
        assert np.allclose(np.linalg.norm(segs[:, 1] - segs[:, 0], axis=1), 1.0)
        loops, simple = outline_loops(segs)
        assert loops == 1 and simple, (level, loops, simple)


def test_integer_recurrence_predicts_the_next_level_and_gives_the_growth_rate_and_dimension():
    coef, resid = integer_recurrence(BOUNDARY_COUNTS, 3)
    assert [int(c) for c in coef] == [-1, -3, 5], coef  # B[n+3] = 5 B[n+2] - 3 B[n+1] - B[n]
    assert all(r == 0 for r in resid), resid  # all four equations hold, the fourth is a check
    # level 5 recomputed live from levels 0..4 plus the recurrence
    live = boundary_counts(4)
    assert 5 * live[4] - 3 * live[3] - live[2] == BOUNDARY_COUNTS[5]
    assert abs(growth_rate(coef) - GROWTH) < 1e-12
    assert abs(D_OUTLINE - 1.39925) < 1e-5 and 1.0 < D_OUTLINE < 2.0
    # the ratio of the counts converges to the growth rate
    ratios = [BOUNDARY_COUNTS[i + 1] / BOUNDARY_COUNTS[i] for i in range(len(BOUNDARY_COUNTS) - 1)]
    assert abs(ratios[-1] - GROWTH) < 1e-3 and abs(ratios[-1] - GROWTH) < abs(ratios[-3] - GROWTH)
    # the substitution factors: the number of tiles per level is the Perron eigenvalue 4 + sqrt(15)
    n = [sp.SpectrePatch(i).n_tiles for i in (3, 4, 5)]
    assert abs(n[2] / n[1] - LAMBDA) < 2e-3
    try:
        integer_recurrence([1, 2, 3], 3)
    except ValueError:
        pass
    else:
        raise AssertionError("too short a sequence must be rejected")


def test_geometric_box_counts_of_the_outline_grow_by_the_growth_rate_per_level():
    sizes = (1.0, 2.0, 4.0)
    n4 = box_count_points(outline_segments(sp.SpectrePatch(4)), sizes)
    n5 = box_count_points(outline_segments(sp.SpectrePatch(5)), sizes)
    ratio = n5 / n4
    assert np.all(np.abs(ratio - GROWTH) < 0.15 * GROWTH), ratio  # fine boxes see the new level's detail
    dim = math.log(ratio[0]) / math.log(math.sqrt(LAMBDA))
    assert abs(dim - D_OUTLINE) < 0.1, (dim, D_OUTLINE)
    # a smooth control: a circle gives local dimension 1, ratio of counts is the ratio of radii
    th = np.linspace(0, 2 * np.pi, 4001)
    circ = np.stack([np.column_stack([np.cos(th[:-1]), np.sin(th[:-1])]), np.column_stack([np.cos(th[1:]), np.sin(th[1:])])], axis=1)
    c1 = box_count_points(100.0 * circ, (1.0, 2.0), step=0.1)
    assert abs(math.log(c1[0] / c1[1]) / math.log(2.0) - 1.0) < 0.05, c1


def test_valley_set_of_a_tile_has_a_dimension_one_plateau_and_no_fractal_range():
    for scale in (0.5, 0.0):
        axis, f = tile_field(1.0, 1.0, 1.0, scale, 10.0, 401)
        side, counts, dim, frac = valley_box_counts(f, axis, eps=0.1, sizes=(1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48))
        assert 0.02 < frac < 0.3, frac
        assert dim[0] > 1.5 and dim[-1] > 1.4, dim  # 2 at both ends (band width, fold spacing)
        plateau = dim[(side > 0.3) & (side < 1.2)]
        assert plateau.min() > 0.6 and plateau.min() < 1.2, dim  # dips to about 1, a curve
        assert not np.any((dim > 1.25) & (dim < 1.4) & (side > 0.4) & (side < 1.0)), dim  # nothing at 1.3-1.4
    # the minimum local dimension is the dimension of a curve for a synthetic line pattern of finite width
    n = 400
    yy, xx = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    stripes = (np.abs(((xx + 0.3 * yy) % 40) - 20) < 3)
    sz = (1, 2, 4, 6, 8, 10, 12, 16)
    d = local_dimension(sz, box_count_image(stripes, sz))
    assert d[0] > 1.4 and 0.8 < d[3:7].min() < 1.25, d  # 2 inside a stripe, 1 between its width and the spacing


def test_window_slopes_use_whole_periods_and_peak_exponent_separates_bragg_from_diffuse():
    r = np.geomspace(0.005, 0.5, 80)
    pure = r ** -2.5
    wins = window_slopes(r, pure)
    assert len(wins) >= 3
    for k0, k1, s in wins:
        assert abs(s + 2.5) < 1e-9 and abs(k1 / k0 - LAMBDA) < 1e-9, (k0, k1)  # two periods = a factor LAMBDA
    wob = r ** -2.5 * (1.0 + 0.3 * np.cos(2.0 * math.pi * np.log(r) / (0.5 * math.log(LAMBDA))))
    # a log periodic modulation over whole periods barely moves the slope
    ww = window_slopes(r, wob, starts=(0.01, 0.02), periods=2)
    assert len(ww) == 2 and all(abs(s + 2.5) < 0.15 for _, _, s in ww)
    k = (2.0 * math.pi, 2.0 * math.pi)  # both square factors matter here
    ns = [4, 8, 16]
    bragg = [square_array_intensity(n, 0.5, k) for n in ns]
    assert abs(peak_exponent([n * n for n in ns], bragg) - 1.0) < 1e-9, bragg
    diffuse = [square_array_intensity(n, 0.5, k, jitter=True, seed=3) for n in ns]
    assert abs(peak_exponent([n * n for n in ns], diffuse)) < 0.35, diffuse
    p = [sp.SpectrePatch(level) for level in (2, 3)]
    area = so.spectre_area(1.0, 1.0)
    i0 = [float(sd.diffraction_intensity(q, np.zeros((1, 2)), 1.0, 0.0)[0]) for q in p]
    assert np.allclose(i0, [q.n_tiles * area ** 2 for q in p], rtol=1e-9), i0  # the origin is Bragg
    kf, tr = track_peak(p, (0.3, 0.1))
    assert len(tr) == 2 and tr[0][1] > 0 and np.all(np.isfinite(kf))
    start = float(sd.diffraction_intensity(p[0], np.array([[0.3, 0.1]]), 1.0, 0.0)[0])
    assert tr[0][1] >= start - 1e-9  # the tracker never loses height at the first level


def test_validation():
    for bad in (lambda: tile_field(extent=-1.0), lambda: tile_field(n=4),
                lambda: box_count_image(np.ones((4, 4), bool), (0,))):
        try:
            bad()
        except ValueError:
            continue
        raise AssertionError("invalid input must raise")


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
def show_fractal(k=0.5, save=None, show=True):
    """Six panels: the folds of the tile with their vortices; the local dimension of the valley set
    (a plateau at 1); the density of vortices; the fractal outline of a supertile; how the outline
    counts grow (dimension 1.399); the orientation-averaged intensity of the supertile."""
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm

    area = so.spectre_area(1.0, 1.0)
    fig, axes = plt.subplots(2, 3, figsize=(17, 10.6))

    # 1. folds of the curved tile with the vortices
    ext, n = 7.0, 351
    axis, f = tile_field(1.0, 1.0, 1.0, k, ext, n)
    pos, q = phase_vortices(f, axis)
    ax = axes[0, 0]
    im = ax.imshow(np.abs(f) ** 2, extent=(-ext, ext, -ext, ext), origin="lower", cmap="magma",
                   norm=LogNorm(vmin=1e-6 * area ** 2, vmax=area ** 2))
    ax.scatter(*pos[q > 0].T, s=22, facecolor="none", edgecolor="cyan", lw=1.2, label="vortex, charge +1 (%d)" % (q > 0).sum())
    ax.scatter(*pos[q < 0].T, s=22, facecolor="none", edgecolor="lime", lw=1.2, label="vortex, charge -1 (%d)" % (q < 0).sum())
    ax.legend(fontsize=8, loc="upper right")
    ax.set_xlabel("k_x")
    ax.set_ylabel("k_y")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
    ax.set_title("the folds are valleys of |F|^2 (curved tile): |F|^2 vanishes only at the\n"
                 "isolated phase vortices, so the dark lines are smooth curves between them", fontsize=9)

    # 2. local dimension of the valley set
    ax = axes[0, 1]
    axis2, f2 = tile_field(1.0, 1.0, 1.0, k, 10.0, 521)
    axis3, f3 = tile_field(1.0, 1.0, 1.0, 0.0, 10.0, 521)
    cols = {0.05: "tab:green", 0.1: "tab:orange", 0.2: "tab:purple"}
    for eps, col in cols.items():
        for fld, ax_, ls, name in ((f2, axis2, "-", "curved"), (f3, axis3, "--", "straight")):
            side, _, dim, _ = valley_box_counts(fld, ax_, eps=eps)
            ax.plot(side, dim, ls, color=col, lw=1.4, label="eps=%g %s" % (eps, name))
    ax.axhline(1.0, color="k", lw=0.8)
    ax.axhline(2.0, color="k", lw=0.8)
    ax.axhline(D_OUTLINE, color="tab:red", lw=1.0, ls=":", label="D of the supertile outline = %.3f" % D_OUTLINE)
    ax.set_xscale("log")
    ax.set_ylim(0.2, 2.1)
    ax.set_xlabel("box side in k")
    ax.set_ylabel("local box dimension  -d ln N / d ln s")
    ax.legend(fontsize=7, ncol=2, loc="lower right")
    ax.grid(alpha=0.3)
    ax.set_title("box counting of the dark valleys: 2 at the width of a valley, a dip to ~1 (a curve),\n"
                 "2 again above the spacing of the folds - no scaling range with a fractional value", fontsize=9)

    # 3. vortex density
    ax = axes[0, 2]
    radii = np.linspace(2.0, 9.5, 40)
    for scale, col, name in ((k, "tab:red", "curved"), (0.0, "tab:blue", "straight")):
        axv, fv = tile_field(1.0, 1.0, 1.0, scale, 10.0, 401)
        pv, qv = phase_vortices(fv, axv)
        dens = vortex_density(pv, radii)
        ax.plot(radii, dens, color=col, lw=1.6, label="%s: %d vortices in the square, net charge %d" % (name, len(qv), qv.sum()))
    ax.set_ylim(0, 0.6)
    ax.set_xlabel("K")
    ax.set_ylabel("vortices in |k| < K per unit area of k")
    ax.legend(fontsize=8, loc="lower right")
    ax.grid(alpha=0.3)
    ax.set_title("density of the zeros of F: about 0.3 per unit k-area (pairs of opposite charge)", fontsize=9)

    # 4. outline of a supertile
    ax = axes[1, 0]
    patch4 = sp.SpectrePatch(4)
    segs = outline_segments(patch4)
    from matplotlib.collections import LineCollection

    ax.add_collection(LineCollection(segs, colors="tab:red", linewidths=0.6))
    ax.autoscale()
    ax.set_aspect("equal")
    ax.set_title("outline of the level-4 supertile (%d tiles): %d unit edges in one closed loop\n"
                 "of box dimension D = %.3f at the limit" % (patch4.n_tiles, len(segs), D_OUTLINE), fontsize=9)
    ax.set_xlabel("x")
    ax.set_ylabel("y")

    # 5. box counts of the outline, with the growth of the edge counts as an inset
    ax = axes[1, 1]
    anchor = None
    for level, col in ((3, "tab:blue"), (4, "tab:green"), (5, "tab:purple")):
        sg = outline_segments(sp.SpectrePatch(level))
        radius = float(np.max(np.hypot(*sg.reshape(-1, 2).T)))
        sizes = np.geomspace(1.0, radius / 2.0, 22)
        cnt = box_count_points(sg, sizes)
        ax.loglog(sizes, cnt, "o-", ms=3, color=col, lw=1.1, label="level %d outline" % level)
        if level == 5:
            anchor = (sizes[0], cnt[0])
    xs = np.array([1.0, 60.0])
    ax.loglog(xs, anchor[1] * (xs / anchor[0]) ** -D_OUTLINE, "r--", lw=1.0, label="slope -D = -%.3f" % D_OUTLINE)
    ax.loglog(xs, anchor[1] * (xs / anchor[0]) ** -1.0, "k:", lw=1.0, label="slope -1 (smooth curve)")
    ax.set_xlabel("box side")
    ax.set_ylabel("number of boxes N")
    ax.legend(fontsize=8, loc="lower left")
    ax.grid(alpha=0.3, which="both")
    ax.set_title("box counts of the finite outlines: the fitted slope rises with the level (about -1.2 at level 3,\n"
                 "-1.3 at level 5) towards -D; inset: the exact growth of the edge counts", fontsize=9)
    ins = ax.inset_axes([0.58, 0.58, 0.38, 0.34])
    ratios = [BOUNDARY_COUNTS[i + 1] / BOUNDARY_COUNTS[i] for i in range(len(BOUNDARY_COUNTS) - 1)]
    ins.plot(np.arange(1, len(ratios) + 1), ratios, "o-", color="tab:red", ms=3)
    ins.axhline(GROWTH, color="k", ls="--", lw=0.8)
    ins.set_title("B(n)/B(n-1) -> 2+sqrt5", fontsize=7)
    ins.tick_params(labelsize=6)

    # 6. radial intensity of the supertile
    ax = axes[1, 2]
    radii = np.geomspace(0.006, 0.6, 70)
    for level, col in ((4, "tab:green"), (5, "tab:purple")):
        pat = sp.SpectrePatch(level)
        inten = radial_intensity(pat, radii, n_angles=72)
        ax.loglog(radii, inten / inten[-1], color=col, lw=1.2, label="level %d (%d tiles)" % (level, pat.n_tiles))
        sl = window_slopes(radii, inten)
        mean = np.mean([x for _, _, x in sl])
        spread = np.std([x for _, _, x in sl])
        ax.text(0.02, 0.05 + 0.09 * (level - 4), "level %d: mean slope %.2f +- %.2f over %d windows" % (level, mean, spread, len(sl)),
                transform=ax.transAxes, fontsize=8, color=col, bbox=dict(fc="white", ec="none", alpha=0.8))
        if level == 5:
            ref = radii[(radii > 0.02) & (radii < 0.3)]
            k0 = 0.03
            y0 = float(np.interp(k0, radii, inten / inten[-1]))
            ax.loglog(ref, y0 * (ref / k0) ** -3.0, "k--", lw=1.0, label="k^-3 (smooth boundary, Porod)")
            ax.loglog(ref, y0 * (ref / k0) ** -(4 - D_OUTLINE), "r--", lw=1.0, label="k^-(4-D) = k^-%.2f" % (4 - D_OUTLINE))
    ax.set_xlabel("|k|")
    ax.set_ylabel("orientation-averaged |FT|^2 (relative)")
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(alpha=0.3, which="both")
    ax.set_title("orientation-averaged intensity of the supertile: mean window slopes -2.6 to -2.7\n"
                 "(smooth boundary -3, 4 - D = -2.60), windows scatter by about 0.2: a loose check only", fontsize=9)

    fig.suptitle("What is fractal around the folds? Not the folds (smooth, dimension 1) but the outline of the supertile: "
                 "D = ln(2+sqrt5) / ln sqrt(4+sqrt15) = %.4f" % D_OUTLINE, fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if save:
        fig.savefig(save, dpi=110)
        print("saved", save)
    if show:
        plt.show()
    plt.close(fig)


def report_peaks(levels=(2, 3, 4, 5), n_peaks=6, ext=1.2, n=121):
    """Track the strongest per-tile intensity maxima of the level-3 patch away from the origin through the
    levels and print their exponent alpha (I_max ~ N^alpha); returns the list of exponents."""
    from scipy.ndimage import maximum_filter

    patches = [sp.SpectrePatch(lv) for lv in levels]
    base = patches[levels.index(3)] if 3 in levels else patches[0]
    ax_ = np.linspace(-ext, ext, n)
    gx, gy = np.meshgrid(ax_, ax_)
    g = sd.diffraction_intensity(base, np.stack([gx, gy], axis=-1), 1.0, 0.0)
    mx = (g == maximum_filter(g, size=5)) & (np.hypot(gx, gy) > 0.25)
    iy, ix = np.nonzero(mx)
    order = np.argsort(-g[iy, ix])
    seen, picks = [], []
    for o in order:  # skip the point reflection k -> -k (the intensity is symmetric)
        p = np.array([ax_[ix[o]], ax_[iy[o]]])
        if any(np.hypot(*(p + s)) < 0.05 for s in seen):
            continue
        seen.append(p)
        picks.append(p)
        if len(picks) == n_peaks:
            break
    alphas, finals = [], []
    for p in picks:
        kf, track = track_peak(patches, p)
        if np.hypot(*kf) < 0.05:
            print("  start k=(%.2f, %.2f): slid into the origin (the trivial Bragg peak I = N A^2), skipped" % (p[0], p[1]))
            continue
        if any(np.hypot(*(kf - q)) < 0.01 or np.hypot(*(kf + q)) < 0.01 for q in finals):
            print("  start k=(%.2f, %.2f): converged to a maximum already tracked, skipped" % (p[0], p[1]))
            continue
        finals.append(kf)
        ns, ii = zip(*track)
        alpha = peak_exponent(ns[1:], ii[1:])
        alphas.append(alpha)
        print("  start k=(%.2f, %.2f) -> k=(%.4f, %.4f)  I_max/tile by level: %s  alpha = %.2f" %
              (p[0], p[1], kf[0], kf[1], ", ".join("%.4g" % x for x in ii), alpha))
    return alphas


def main(argv=None):
    p = argparse.ArgumentParser(description="Fractal analysis around the diffraction of the Spectre: self tests, a figure, optional checks.")
    p.add_argument("--curve_strength", type=float, default=0.5, help="uniform curve strength k of the tile (default 0.5)")
    p.add_argument("--skip-tests", action="store_true", help="only make the figure")
    p.add_argument("--tests-only", action="store_true", help="run the self tests and exit (no plot)")
    p.add_argument("--no-show", action="store_true", help="do not open a window (use with --save)")
    p.add_argument("--save", metavar="PNG", help="also save the figure to this file")
    p.add_argument("--deep", action="store_true", help="recompute the level-6 outline count (about 80 s) and check it")
    p.add_argument("--peaks", action="store_true", help="track the strongest diffraction maxima through the levels (a few minutes)")
    args = p.parse_args(argv)

    if not args.skip_tests:
        print("spectre_fractal self tests:")
        if not run_tests():
            return 1
    if args.deep:
        b6 = len(sp.SpectrePatch(6).boundary_edges())
        print("level 6 outline edges: %d (stored %d) -> %s" % (b6, BOUNDARY_COUNTS[6], "OK" if b6 == BOUNDARY_COUNTS[6] else "MISMATCH"))
        if b6 != BOUNDARY_COUNTS[6]:
            return 1
    if args.peaks:
        print("peak exponents (1 = Bragg, 0 = diffuse):")
        report_peaks()
    if args.tests_only:
        return 0
    show_fractal(args.curve_strength, save=args.save, show=not args.no_show)
    return 0


if __name__ == "__main__":
    sys.exit(main())
