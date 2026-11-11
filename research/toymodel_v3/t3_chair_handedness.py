#!/usr/bin/env python3
"""t3_chair_handedness.py - item 6a of research/toymodel_v3/PLAN.md: does the decorated Chair44 carry a handedness that could fix the sign of the area term?

Background (CONNECTIONS_SPECTRE.md, section 1.1).  The Chair44 is the transient substrate (t3_escape.py E2.5: 1/r), but its hull is achiral: the substitution
commutes with the x <-> y mirror P (proved in the Hat and Spectre repository, research/chair44_mirror_proof.py), so every pseudoscalar of the bare geometry vanishes.
The bump/dent decoration (192 pyramid features, Tsiokos, CC BY 4.0, models/data/r44_features.csv) is invariant under PC, C = flip every height, but not under P.
A pseudoscalar that is odd under C as well as under P is therefore not forced to vanish.

Definitions (everything is exact rational arithmetic)
  D6a.1  Reference chair: the 7 unit cells [0,2]^3 minus [1,2]^3 (chair44_core.CHAIR_CELLS), centroid c0 = (13/14)(1,1,1).  Feature f: base point b_f on its panel
         (the apex with its normal coordinate set to the panel's), outward unit normal n_f, signed height h_f in {+-1, ..., +-12}.
  D6a.2  The chair's own polar vector: g = c0 - (1,1,1), the centroid minus the centre of the bounding cube [0,2]^3 (= -(1/14)(1,1,1)).
  D6a.3  Decoration moments, u_f = b_f - c0:  dipole D = sum h_f u_f (polar, C-odd);  axial moment A = sum h_f u_f x n_f (axial, C-odd).
  D6a.4  psi = A . g: rotation invariant, odd under a mirror (A axial, g polar) and odd under C (A linear in h).  For a supertile, chi_k = sum over tiles of
         (R_t A) . (R_t g), each tile in its pose (R_t, t_t), computed from the placed features, not from the formula.

Exact claims (each a test; ``python3 t3_chair_handedness.py --tests-only``)
  E6a.1  The bare geometry is mirror symmetric: the level-k identity-frame supertile (k = 0..3) is, as a set of unit cells partitioned into tiles, equal to its
         image under P = (x <-> y).  So every pseudoscalar of the cells vanishes (the result of the Hat and Spectre repository, re-checked here on the cells).
  E6a.2  On the reference chair P maps the decorated feature set to itself with every height negated (192 of 192): P(decoration) = C(decoration).
  E6a.3  D = 0 exactly (which is why the simplest C-odd pseudoscalars, built from D, vanish), A = (-93/14, -93/14, 114/7), and psi = A . g = -3/14.
         psi changes sign under P and under C separately, is unchanged under PC and under all 24 proper rotations of the cube (and under translations).
  E6a.4  On the level-k supertile (k = 1..3; 8, 64, 512 tiles; every placement a proper rotation) chi_k = N_k psi exactly, computed feature by feature:
         -12/7, -96/7, -768/7.  The sign is the same at every level, unlike the Spectre's supertile chirality, which alternates.

What it says.  The decorated Chair44 is chiral in the PC sense: its mirror image is its bump/dent-swapped image, and psi = -3/14 per tile tells the two apart.
So the transient substrate CAN supply a fixed-sign handedness eps = sign(psi) for the area term theta = theta_0 eps of t3_area.py, but only through the decoration:
swapping every bump and dent reverses the arrow, and a mirror image of the world reverses it too, so the combined PC image keeps it.
The obstruction of CONNECTIONS_SPECTRE.md section 1.1 is lifted for the decorated tile; it stands for the bare geometry.  **Hypothesis**: that the decoration is
physical (a charge-like label) and that the arrow couples to psi; nothing here couples them.

    python3 t3_chair_handedness.py --tests-only
    python3 t3_chair_handedness.py --story
"""
import csv
import itertools
import os
import sys
from fractions import Fraction as Fr

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
import chair44_core as C  # noqa: E402

DATA = os.path.join(_ROOT, "models", "data", "r44_features.csv")


def _vec(a):
    return tuple(Fr(x) for x in a)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(s, a):
    return tuple(s * x for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def features():
    """[(base point, outward normal, height)] of the reference chair (D6a.1)."""
    out = []
    with open(DATA) as fh:
        for r in csv.DictReader(fh):
            apex = [Fr(r["apex_x"]), Fr(r["apex_y"]), Fr(r["apex_z"])]
            pc = [Fr(r["panel_cx"]), Fr(r["panel_cy"]), Fr(r["panel_cz"])]
            k = int(r["normal_axis"])
            apex[k] = pc[k]
            n = [0, 0, 0]
            n[k] = int(r["outward_sign"])
            out.append((tuple(apex), tuple(Fr(x) for x in n), int(r["coefficient"])))
    return out


CENTROID = tuple(Fr(sum(c[i] for c in C.CHAIR_CELLS), 7) + Fr(1, 2) for i in range(3))
G = sub(CENTROID, (1, 1, 1))


# -- maps: proper rotations (chair44 frames), the mirror P, the decoration flip C ---------------------------------
def frame_point(f, v, t=(0, 0, 0)):
    p, s = f
    return tuple(s[i] * v[p[i]] + t[i] for i in range(3))


def frame_vector(f, v):
    return frame_point(f, v)


def mirror(v):
    return (v[1], v[0], v[2])


def moments(feats, centre):
    D = (0, 0, 0)
    A = (0, 0, 0)
    for b, n, h in feats:
        u = sub(b, centre)
        D = add(D, scale(h, u))
        A = add(A, scale(h, cross(u, n)))
    return D, A


def psi(feats=None, centre=None, g=None):
    feats = features() if feats is None else feats
    centre = CENTROID if centre is None else centre
    g = G if g is None else g
    return dot(moments(feats, centre)[1], g)


def transformed(feats, point_map, vec_map, flip=False):
    return [(point_map(b), vec_map(n), -h if flip else h) for b, n, h in feats]


def rotations():
    """The 24 proper rotations of the cube as chair44 frames (permutation, signs) with determinant +1."""
    out = []
    for p in itertools.permutations(range(3)):
        for s in itertools.product((1, -1), repeat=3):
            f = (p, s)
            if C.frame_det(f) == 1:
                out.append(f)
    return out


def chi(depth):
    """chi_k of D6a.4, feature by feature on the level-depth supertile."""
    feats = features()
    total = Fr(0)
    n_tiles = 0
    for f, t in C.sigma_depth(depth):
        assert C.frame_det(f) == 1
        n_tiles += 1
        placed = [(frame_point(f, b, t), frame_vector(f, n), h) for b, n, h in feats]
        centre = frame_point(f, CENTROID, t)
        g = frame_vector(f, G)
        total += psi(placed, centre, g)
    return n_tiles, total


def supertile_cells(depth):
    """{cell: tile index} of the level-depth identity-frame supertile."""
    return C.patch_cells(depth)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------
def test_e6a1_the_bare_supertile_is_mirror_symmetric():
    for depth in range(0, 4):
        cells = supertile_cells(depth)
        tiles = {}
        for c, ti in cells.items():
            tiles.setdefault(ti, set()).add(c)
        parts = {frozenset(v) for v in tiles.values()}
        mparts = {frozenset(mirror(c) for c in v) for v in tiles.values()}
        assert parts == mparts, depth


def test_e6a2_the_mirror_of_the_decoration_is_its_flip():
    feats = features()
    assert len(feats) == 192
    key = {(b, n): h for b, n, h in feats}
    hits = 0
    for b, n, h in feats:
        mb, mn = mirror(b), mirror(n)
        assert (mb, mn) in key and key[(mb, mn)] == -h
        hits += 1
    assert hits == 192


def test_e6a3_the_decorated_chair_has_a_c_odd_handedness():
    feats = features()
    D, A = moments(feats, CENTROID)
    assert D == (0, 0, 0)
    assert A == (Fr(-93, 14), Fr(-93, 14), Fr(114, 7))
    assert G == (Fr(-1, 14),) * 3
    p = psi(feats)
    assert p == Fr(-3, 14)
    # P alone: mirror the features, the centre and g
    pm = psi(transformed(feats, mirror, mirror), mirror(CENTROID), mirror(G))
    pc = psi(transformed(feats, lambda x: x, lambda x: x, flip=True))
    ppc = psi(transformed(feats, mirror, mirror, flip=True), mirror(CENTROID), mirror(G))
    assert pm == -p and pc == -p and ppc == p
    for f in rotations():
        rf = transformed(feats, lambda x: frame_point(f, x), lambda x: frame_vector(f, x))
        assert psi(rf, frame_point(f, CENTROID), frame_vector(f, G)) == p
    shift = (Fr(5), Fr(-3, 2), Fr(7, 3))
    tf = transformed(feats, lambda x: add(x, shift), lambda x: x)
    assert psi(tf, add(CENTROID, shift), G) == p
    assert len(rotations()) == 24


def test_e6a4_the_supertile_handedness_has_one_sign_at_every_level():
    got = [chi(k) for k in (1, 2, 3)]
    assert got == [(8, Fr(-12, 7)), (64, Fr(-96, 7)), (512, Fr(-768, 7))], got
    assert all(c == n * Fr(-3, 14) for n, c in got)


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


def story():
    D, A = moments(features(), CENTROID)
    print("decorated reference chair: D =", D, " A =", A, " g =", G, " psi = A.g =", psi())
    for k in (1, 2, 3):
        n, c = chi(k)
        print("level %d supertile: %d tiles, chi = %s = N psi" % (k, n, c))


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    failed = not run_tests()
    if failed or "--tests-only" in argv:
        return 1 if failed else 0
    if "--story" in argv:
        story()
    return 0


if __name__ == "__main__":
    sys.exit(main())
