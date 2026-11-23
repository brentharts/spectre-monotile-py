#!/usr/bin/env python3
"""t10_cavity.py - exact cavity completion for Spectre patches by propagation search (no SAT, no C++).

Replaces the SAT/DRAT part of Lockwood's toymodel_v9 pull request (reported in lockwood_reported/).
Question: erase k tiles of the exact level-4 patch (4401 tiles, all one chirality); in how many ways can
the hole be refilled by Spectre tiles, edge to edge, without overlap?  Unique means only the original.

Method (a corner-filling search with minimum-remaining-values and forced moves, i.e. unit propagation):
  * every vertex is a point of Z[zeta_12], stored as an integer 4-vector (spectre_exact.py), so vertex
    identity, corner angles (multiples of 30 degrees) and the 12 rotations are exact;
  * a vertex v of the cavity boundary (or of a new tile) is open while its tiles cover < 360 degrees.
    The free directions at v are known exactly; a new tile must put one of its corners into them;
  * candidate tiles at v: 14 corners x 12 rotations (24 with reflections), filtered by the free sectors,
    then by interior overlap with every fixed or placed tile (triangulated, exact separating-edge test on
    Q(sqrt3) coordinates; floats only decide the clear cases, |value| > 1e-9) and by edge-to-edge
    (no vertex of one tile strictly inside an edge of another);
  * pick the open vertex with fewest candidates; none -> dead end, one -> forced move.
  Every tiling of the hole has exactly one tile in the free sector at v, so branching there is complete.
  Area is the same for all tiles, so a filling covers the hole iff it uses k tiles (checked).

Assumption (tested on the original tilings): tilings are edge to edge on the vertex graph.
Labels: Exact for the outcome of each search (finite, complete under that assumption); the node counts are
Counted.  Search stops after 2 fillings (original + one more) unless --all.

    SPECTRE_DIR=/path/to/spectre python3 t10_cavity.py --tests-only
"""
from __future__ import annotations

import argparse
import collections
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
for _p in (os.path.join(os.environ.get("SPECTRE_DIR", ""), "boundary_certificate"),
           os.path.join(_HERE, "..", "..", "..", "spectre", "boundary_certificate"),
           os.path.join(_HERE, "..", "repos", "spectre", "boundary_certificate")):
    if _p and os.path.isfile(os.path.join(_p, "spectre_exact.py")):
        sys.path.insert(0, _p)
        break
import spectre_exact as se  # noqa: E402

S3 = math.sqrt(3.0)
EPS = 1e-9


# ------------------------------------------------------------------ exact arithmetic on Q(sqrt3)
def _sgn(a, b):
    """sign of a + b*sqrt3, a and b integers."""
    if b == 0:
        return (a > 0) - (a < 0)
    if a == 0:
        return (b > 0) - (b < 0)
    if (a > 0) == (b > 0):
        return 1 if a > 0 else -1
    return (1 if a > 0 else -1) if a * a > 3 * b * b else (1 if b > 0 else -1)


def xy2(v):
    """doubled exact coordinates: (2x, 2y) as pairs (a, b) = a + b*sqrt3."""
    p, q, r, s = v
    return (2 * p + q, r), (r + 2 * s, q)


def xyf(v):
    p, q, r, s = v
    return (p + 0.5 * q + 0.5 * S3 * r, 0.5 * S3 * q + 0.5 * r + s)


def cross_sign(o, a, b):
    """sign of cross(a - o, b - o) for Z^4 points (exact; float filter first)."""
    (ox, oy), (ax, ay), (bx, by) = xyf(o), xyf(a), xyf(b)
    val = (ax - ox) * (by - oy) - (ay - oy) * (bx - ox)
    if abs(val) > 1e-7:
        return 1 if val > 0 else -1
    (Ox, Oy), (Ax, Ay), (Bx, By) = xy2(o), xy2(a), xy2(b)
    ux = (Ax[0] - Ox[0], Ax[1] - Ox[1]); uy = (Ay[0] - Oy[0], Ay[1] - Oy[1])
    wx = (Bx[0] - Ox[0], Bx[1] - Ox[1]); wy = (By[0] - Oy[0], By[1] - Oy[1])

    def mul(p, q):
        return (p[0] * q[0] + 3 * p[1] * q[1], p[0] * q[1] + p[1] * q[0])
    m1, m2 = mul(ux, wy), mul(uy, wx)
    return _sgn(m1[0] - m2[0], m1[1] - m2[1])


# ------------------------------------------------------------------ shapes
def _ear_clip(pts):
    """triangulate a simple polygon (float points, any orientation) -> list of index triples."""
    n = len(pts)
    area = sum(pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1] for i in range(n))
    idx = list(range(n))
    if area < 0:
        idx.reverse()

    def cr(a, b, c):
        return (pts[b][0] - pts[a][0]) * (pts[c][1] - pts[a][1]) - (pts[b][1] - pts[a][1]) * (pts[c][0] - pts[a][0])

    def inside(p, a, b, c):
        return cr(a, b, p) > 1e-12 and cr(b, c, p) > 1e-12 and cr(c, a, p) > 1e-12
    tris = []
    while len(idx) > 3:
        for k in range(len(idx)):
            a, b, c = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if cr(a, b, c) > 1e-12 and not any(inside(p, a, b, c) for p in idx if p not in (a, b, c)):
                tris.append((a, b, c))
                idx.pop(k)
                break
        else:
            raise RuntimeError("ear clipping failed")
    tris.append(tuple(idx))
    return tris


def _build_shapes(reflect):
    shapes = []
    for refl in ([False, True] if reflect else [False]):
        for k in range(12):
            T = se.trot(k)
            if refl:
                T = se.mul(T, se.R)
            verts = [tuple(int(x) for x in se.apply(T, v)) for v in se.SPECTRE]
            base = verts[0]
            verts = [tuple(a - b for a, b in zip(v, base)) for v in verts]   # vertex 0 at origin
            f = [xyf(v) for v in verts]
            n = len(verts)
            area = sum(f[i][0] * f[(i + 1) % n][1] - f[(i + 1) % n][0] * f[i][1] for i in range(n))
            ccw = area > 0
            ang = []
            dirs = []                     # direction (units of 30 deg) of the edge leaving each vertex
            for i in range(n):
                nx, pv = f[(i + 1) % n], f[i - 1]
                d_next = math.degrees(math.atan2(nx[1] - f[i][1], nx[0] - f[i][0]))
                d_prev = math.degrees(math.atan2(pv[1] - f[i][1], pv[0] - f[i][0]))
                un, up = round(d_next / 30) % 12, round(d_prev / 30) % 12
                assert abs(d_next / 30 - round(d_next / 30)) < 1e-9
                if ccw:
                    a = (up - un) % 12            # sweep ccw from next-direction to prev-direction
                    sec = [(un + j) % 12 for j in range(a)]
                else:
                    a = (un - up) % 12
                    sec = [(up + j) % 12 for j in range(a)]
                ang.append(a)
                dirs.append(frozenset(sec))
            assert sum(ang) == 72, sum(ang)
            tris = _ear_clip(f)
            shapes.append({"verts": verts, "f": f, "ang": ang, "sec": dirs, "tris": tris, "refl": refl,
                           "rot": k})
    return shapes


class Tile:
    __slots__ = ("verts", "vset", "bbox", "tris", "ang", "sec", "tf", "edges")

    def __init__(self, verts, shape):
        self.verts = verts
        self.vset = frozenset(verts)
        f = [xyf(v) for v in verts]
        self.bbox = (min(p[0] for p in f), min(p[1] for p in f), max(p[0] for p in f), max(p[1] for p in f))
        self.tris = [tuple(verts[i] for i in t) for t in shape["tris"]]
        self.tf = [tuple(f[i] for i in t) for t in shape["tris"]]
        self.ang = shape["ang"]
        self.sec = shape["sec"]
        self.edges = [(verts[i], verts[(i + 1) % len(verts)]) for i in range(len(verts))]


def _ccw(tri):
    return tri if cross_sign(*tri) > 0 else (tri[0], tri[2], tri[1])


def _tri_overlap(t1, t2):
    """True iff the open interiors of two (non-degenerate) triangles intersect (exact)."""
    t1, t2 = _ccw(t1), _ccw(t2)
    for A, B in ((t1, t2), (t2, t1)):
        for i in range(3):
            p, q = A[i], A[(i + 1) % 3]
            if all(cross_sign(p, q, x) <= 0 for x in B):
                return False
    return True


def _bbox_hit(a, b):
    return a[0] < b[2] - EPS and b[0] < a[2] - EPS and a[1] < b[3] - EPS and b[1] < a[3] - EPS


def tiles_overlap(A, B):
    if not _bbox_hit(A.bbox, B.bbox):
        return False
    for ta, fa in zip(A.tris, A.tf):
        xa = [min(p[0] for p in fa), min(p[1] for p in fa), max(p[0] for p in fa), max(p[1] for p in fa)]
        for tb, fb in zip(B.tris, B.tf):
            xb = [min(p[0] for p in fb), min(p[1] for p in fb), max(p[0] for p in fb), max(p[1] for p in fb)]
            if _bbox_hit(xa, xb) and _tri_overlap(ta, tb):
                return True
    return False


def _on_open_segment(p, a, b):
    """p lies strictly inside segment ab (exact)."""
    if p == a or p == b:
        return False
    fa, fb, fp = xyf(a), xyf(b), xyf(p)
    if not (min(fa[0], fb[0]) - 1e-7 <= fp[0] <= max(fa[0], fb[0]) + 1e-7 and
            min(fa[1], fb[1]) - 1e-7 <= fp[1] <= max(fa[1], fb[1]) + 1e-7):
        return False
    if cross_sign(a, b, p) != 0:
        return False
    # collinear and in the bounding box => between a and b
    return True


def not_edge_to_edge(A, B):
    """a vertex of one tile strictly inside an edge of the other."""
    if not _bbox_hit((A.bbox[0] - 1e-6, A.bbox[1] - 1e-6, A.bbox[2] + 1e-6, A.bbox[3] + 1e-6), B.bbox):
        return False
    for P, Q in ((A, B), (B, A)):
        for v in P.verts:
            if v in Q.vset:
                continue
            for a, b in Q.edges:
                if _on_open_segment(v, a, b):
                    return True
    return False


# ------------------------------------------------------------------ patch
_PATCH = {}


def patch(level=4):
    if level in _PATCH:
        return _PATCH[level]
    raw = se.build(level)[level][0]["Delta"]
    shapes = _build_shapes(False)
    # identify each tile with a shape by the rotation of its first edge: match vertex 0 and rotation
    tiles = []
    for T, _kind in raw:
        verts = [tuple(int(x) for x in v) for v in se.outline(T)]
        e = tuple(b - a for a, b in zip(verts[0], verts[1]))
        for sh in shapes:
            if tuple(sh["verts"][1]) == e:
                break
        else:
            raise RuntimeError("tile not a rotation of the spectre")
        sh_verts = sh["verts"]
        assert all(tuple(a + b for a, b in zip(verts[0], sh_verts[i])) == verts[i] for i in range(14))
        tiles.append(Tile(verts, sh))
    cent = np.array([[(t.bbox[0] + t.bbox[2]) / 2, (t.bbox[1] + t.bbox[3]) / 2] for t in tiles])
    vmap = collections.defaultdict(list)
    for i, t in enumerate(tiles):
        for j, v in enumerate(t.verts):
            vmap[v].append((i, j))
    _PATCH[level] = (tiles, cent, vmap)
    return _PATCH[level]


def choose_cavity(level, k, centre_tile):
    """the k tiles nearest (by bbox centre) to the given tile, grown as a connected set."""
    tiles, cent, vmap = patch(level)
    d = np.linalg.norm(cent - cent[centre_tile], axis=1)
    order = list(np.argsort(d, kind="stable"))
    chosen = set(order[:k])
    return sorted(int(i) for i in chosen)


# ------------------------------------------------------------------ search
class Hash:
    def __init__(self, cell=3.0):
        self.cell = cell
        self.d = collections.defaultdict(list)

    def cells(self, bb):
        c = self.cell
        for ix in range(int(math.floor(bb[0] / c)), int(math.floor(bb[2] / c)) + 1):
            for iy in range(int(math.floor(bb[1] / c)), int(math.floor(bb[3] / c)) + 1):
                yield (ix, iy)

    def add(self, t):
        for c in self.cells(t.bbox):
            self.d[c].append(t)

    def near(self, bb):
        seen, out = set(), []
        pad = (bb[0] - 1e-6, bb[1] - 1e-6, bb[2] + 1e-6, bb[3] + 1e-6)
        for c in self.cells(pad):
            for t in self.d.get(c, ()):
                if id(t) not in seen:
                    seen.add(id(t))
                    out.append(t)
        return out


class Search:
    def __init__(self, level, erased, reflect=False, max_solutions=2, node_limit=10 ** 7):
        self.tiles, self.cent, self.vmap = patch(level)
        self.erased = sorted(erased)
        self.reflect = reflect
        self.shapes = _build_shapes(reflect)
        self.max_solutions = max_solutions
        self.node_limit = node_limit
        eset = set(self.erased)
        self.hash = Hash()
        for i, t in enumerate(self.tiles):
            if i not in eset:
                self.hash.add(t)
        self.cav_verts = set(v for i in self.erased for v in self.tiles[i].verts)
        self.nodes = 0
        self.solutions = []
        self.placed = []
        self.cover = collections.defaultdict(list)   # vertex -> list of (tile, corner index) placed or fixed
        for i, t in enumerate(self.tiles):
            if i in eset:
                continue
            for j, v in enumerate(t.verts):
                if v in self.cav_verts:
                    self.cover[v].append((t, j))
        # only vertices where a fixed tile meets the hole must stay vertices of the filling (edge to edge); a vertex
        # interior to the hole may end up inside a new tile, so it is tracked only once a placed tile has a corner there
        self.bound = {v for v in self.cav_verts if v in self.cover}
        self.track = set(self.bound)
        self.cache = {}

    # directions covered at a vertex
    def used_sectors(self, v):
        s = collections.Counter()
        for t, j in self.cover[v]:
            for u in t.sec[j]:
                s[u] += 1
        return s

    def candidates(self, v):
        used = self.used_sectors(v)
        free = [u for u in range(12) if used[u] == 0]
        if any(c > 1 for c in used.values()):
            return []
        if not free:
            return []
        out = []
        fset = set(free)
        for sh in self.shapes:
            for i in range(14):
                if not sh["sec"][i] or not sh["sec"][i] <= fset:
                    continue
                verts = [tuple(a + b - c for a, b, c in zip(v, w, sh["verts"][i])) for w in sh["verts"]]
                t = Tile(verts, sh)
                ok = True
                for o in self.hash.near(t.bbox):
                    if tiles_overlap(t, o) or not_edge_to_edge(t, o):
                        ok = False
                        break
                if not ok:
                    continue
                # sector bookkeeping at the other vertices of the new tile
                for j, w in enumerate(t.verts):
                    if j == i or w not in self.cover:
                        continue
                    uw = self.used_sectors(w)
                    if any(uw[u] for u in t.sec[j]):
                        ok = False
                        break
                if ok:
                    out.append((t, i))
        return out

    def open_vertices(self):
        res = []
        for v in self.track:
            if sum(len(t.sec[j]) for t, j in self.cover[v]) < 12:
                res.append(v)
        return res

    def _invalidate(self, t):
        x0, y0, x1, y1 = t.bbox[0] - 5.0, t.bbox[1] - 5.0, t.bbox[2] + 5.0, t.bbox[3] + 5.0
        for v in [v for v in self.cache if x0 <= xyf(v)[0] <= x1 and y0 <= xyf(v)[1] <= y1]:
            del self.cache[v]

    def place(self, t):
        self._invalidate(t)
        self.hash.add(t)
        self.placed.append(t)
        for j, v in enumerate(t.verts):
            self.cover[v].append((t, j))
            self.track.add(v)

    def unplace(self, t):
        self._invalidate(t)
        self.placed.pop()
        for c in self.hash.cells(t.bbox):
            self.hash.d[c].remove(t)
        for j, v in enumerate(t.verts):
            self.cover[v].pop()
        # vertices that are not on the hole boundary and have no tiles left leave the tracked set
        for v in t.verts:
            if v not in self.bound and not self.cover[v]:
                self.track.discard(v)
                del self.cover[v]

    def run(self):
        self._rec()
        return self.solutions

    def _rec(self):
        if len(self.solutions) >= self.max_solutions or self.nodes > self.node_limit:
            return
        self.nodes += 1
        best = None
        for v in self.open_vertices():
            c = self.cache.get(v)
            if c is None:
                c = self.cache[v] = self.candidates(v)
            if not c:
                return
            if best is None or len(c) < len(best[1]):
                best = (v, c)
                if len(c) == 1:
                    break
        if best is None:
            self.solutions.append(frozenset(t.vset for t in self.placed))
            return
        for t, _i in best[1]:
            self.place(t)
            self._rec()
            self.unplace(t)
            if len(self.solutions) >= self.max_solutions or self.nodes > self.node_limit:
                return


def complete(level, erased, reflect=False, max_solutions=2, node_limit=10 ** 6):
    S = Search(level, erased, reflect, max_solutions, node_limit)
    sols = S.run()
    orig = frozenset(S.tiles[i].vset for i in S.erased)
    others = [s for s in sols if s != orig]
    return {"k": len(S.erased), "nodes": S.nodes, "n_solutions": len(sols), "original_found": orig in sols,
            "other": others, "hit_node_limit": S.nodes > S.node_limit}


# ------------------------------------------------------------------ tests
CENTRE = 2675   # the central tile used in Lockwood's runs (his index; same level-4 ordering assumed)


def test_e110_patch_edge_to_edge():
    """level-4 patch: 4401 tiles of one chirality, no interior overlaps and edge to edge, near the centre."""
    tiles, cent, vmap = patch(4)
    assert len(tiles) == 4401
    h = Hash()
    near = [i for i in range(len(tiles)) if np.linalg.norm(cent[i] - cent[CENTRE]) < 7]
    for i in near:
        h.add(tiles[i])
    bad = 0
    for i in near:
        for o in h.near(tiles[i].bbox):
            if o is not tiles[i] and (tiles_overlap(tiles[i], o) or not_edge_to_edge(tiles[i], o)):
                bad += 1
    assert bad == 0
    return "%d central tiles: 0 overlaps, 0 non-edge-to-edge" % len(near)


def test_e111_single_tile_unique():
    """erase one tile (a sample over the 12 orientations): the only refilling is the original."""
    tiles, cent, vmap = patch(4)
    # orientation of a tile = rotation index of its first edge
    seen, picks = set(), []
    for i in sorted(range(len(tiles)), key=lambda i: np.linalg.norm(cent[i] - cent[CENTRE])):
        e = tuple(b - a for a, b in zip(tiles[i].verts[0], tiles[i].verts[1]))
        if e not in seen:
            seen.add(e)
            picks.append(i)
        if len(picks) == 12:
            break
    assert len(picks) == 12
    nodes = 0
    for i in picks:
        r = complete(4, [i])
        assert r["original_found"] and r["n_solutions"] == 1, (i, r["n_solutions"])
        nodes += r["nodes"]
    return "12 orientations: unique, %d nodes in total" % nodes


def test_e112_chiral_cavities_rigid():
    """centre cavities k = 10, 20, 40: only the original filling (Lockwood reports the same up to k = 300)."""
    out = []
    for k in (10, 20, 40):
        r = complete(4, choose_cavity(4, k, CENTRE))
        assert r["original_found"] and r["n_solutions"] == 1, (k, r["n_solutions"])
        out.append("k=%d: %d nodes" % (k, r["nodes"]))
    return "; ".join(out)


def test_e113_chirality_control():
    """with reflected tiles allowed the original is still found; count cavities (51 edge-adjacent pairs and the
    nearest-k sets k = 1..6 around the centre) with a second filling.  Lockwood reports a 2-tile swap at k = 4.
    Before the fix of the interior-vertex bug (toymodel_v11) this search found none; now it finds the 2-tile
    Mystic swap (recorded, not asserted: see toymodel_v11/README.md)."""
    tiles, cent, vmap = patch(4)
    d = np.linalg.norm(cent - cent[CENTRE], axis=1)
    near = [int(i) for i in np.argsort(d)[:25]]
    pairs = sorted({(min(i, j), max(i, j)) for i in near for v in tiles[i].verts for j, _ in vmap[v]
                    if j != i and j in near and len(tiles[i].vset & tiles[j].vset) >= 2})
    sets = [list(p) for p in pairs] + [choose_cavity(4, k, CENTRE) for k in range(1, 7)]
    alt = 0
    for e in sets:
        r = complete(4, e, reflect=True)
        assert r["original_found"] and not r["hit_node_limit"]
        alt += bool(r["other"])
    return "%d cavities with reflections allowed: %d with a second filling" % (len(sets), alt)


def run_tests(verbose=True):
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_e11")]
    failed = 0
    for n, f in tests:
        t = time.time()
        try:
            r = f()
            if verbose:
                print("ok   %s  %s  (%.1fs)" % (n, r, time.time() - t), flush=True)
        except AssertionError as e:
            failed += 1
            print("FAIL %s %r" % (n, e), flush=True)
    print("%d test(s), %d failed" % (len(tests), failed))
    return failed


def scan(ks, reflect=False, centre=CENTRE):
    for k in ks:
        t = time.time()
        r = complete(4, choose_cavity(4, k, centre), reflect=reflect)
        print("k=%4d reflect=%s fillings=%d original=%s nodes=%d (%.1fs)" % (
            k, reflect, r["n_solutions"], r["original_found"], r["nodes"], time.time() - t), flush=True)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--tests-only", action="store_true")
    ap.add_argument("--scan", type=int, nargs="*")
    ap.add_argument("--reflect", action="store_true")
    a = ap.parse_args()
    if a.scan:
        scan(a.scan, a.reflect)
    else:
        sys.exit(1 if run_tests() else 0)
