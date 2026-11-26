#!/usr/bin/env python3
"""
toymodel_v12, stage S5(a) (decision D3): light at a cone point of the Hat Trick sphere.

**Exact (angle sums).** With every kite flat, the Hat Trick surface (pentagons filled by 5 kites each) is flat
except at the 12 pentagon centres, where 5 kite corners of 60 degrees meet: cone angle 300, deficit 60 degrees.
Its 540 kites are 27 per face of a regular icosahedron, so the flat hat metric *is* the intrinsic metric of the
icosahedron, with the cone points at its vertices.  Geodesics ("light") can therefore be traced exactly by
unfolding across the icosahedron's edges.

What is measured (**Numeric**, tolerance 1e-9; the values are exact by the unfolding argument):
  * two parallel geodesics that pass a cone point on opposite sides at impact parameter b cross behind it at the
    angle delta = 60 degrees, for every b (no 1/b fall-off);
  * they cross at distance b / sin(delta/2) = 2b from the cone point (linear in b: a cone has no focal length);
  * two parallel geodesics with no cone point between them stay parallel.

**Reading (Definition, 2+1 only).** In 2+1 gravity a point mass m is a cone point of deficit 8 pi G m (Deser,
Jackiw, 't Hooft 1984); the same metric is the cross-section of a straight cosmic string.  The deflection is the
holonomy of a flat metric: it is topological, there is no Newtonian potential and no clock effect (g_tt = 1), so
all of the bending is the "space factor" and none is the clock factor of toymodel_v3.  It is a lensing signature
of a conical defect, not of a 1/r field.

Run:  python3 t12_curvature.py      (`--tests-only` for the checks)
"""
from __future__ import annotations

import itertools
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PHI = (1 + 5 ** 0.5) / 2
DEFICIT = 60.0


def icosahedron():
    """Vertices (edge length 2) and outward-oriented faces."""
    V = []
    for a in (1, -1):
        for b in (PHI, -PHI):
            V += [(0, a, b), (a, b, 0), (b, 0, a)]
    V = np.array(V, float)
    F = []
    for i, j, k in itertools.combinations(range(12), 3):
        if all(abs(np.linalg.norm(V[x] - V[y]) - 2) < 1e-9 for x, y in ((i, j), (j, k), (i, k))):
            n = np.cross(V[j] - V[i], V[k] - V[i])
            F.append((i, j, k) if n @ (V[i] + V[j] + V[k]) > 0 else (i, k, j))
    return V, F


class Surface:
    def __init__(self):
        self.V, self.F = icosahedron()
        self.N = [self._unit(np.cross(self.V[j] - self.V[i], self.V[k] - self.V[i])) for i, j, k in self.F]
        self.e2f = {}
        for f, t in enumerate(self.F):
            for s in range(3):
                self.e2f[(t[s], t[(s + 1) % 3])] = f

    @staticmethod
    def _unit(x):
        return x / np.linalg.norm(x)

    def trace(self, f, p, d, length):
        """Geodesic from point p of face f in the tangent direction d, for a given length.  Returns the list of
        straight pieces (face, start, end, direction).  Crossing an edge keeps the angle to the edge (unfolding)."""
        d = self._unit(d - (d @ self.N[f]) * self.N[f])
        out = []
        left = length
        while left > 1e-12:
            t = self.F[f]
            best = None
            for s in range(3):
                a, b = self.V[t[s]], self.V[t[(s + 1) % 3]]
                e = b - a
                inward = np.cross(self.N[f], e)                  # points into the face
                den = d @ inward
                if den >= -1e-13:
                    continue                                     # not heading out through this edge
                lam = -((p - a) @ inward) / den
                if lam > 1e-11 and (best is None or lam < best[0]):
                    best = (lam, s)
            lam, s = best
            if lam >= left:
                out.append((f, p, p + left * d, d))
                break
            q = p + lam * d
            a, b = t[s], t[(s + 1) % 3]
            u = (self.V[b] - self.V[a]) / 2.0
            x = (q - self.V[a]) @ u / (u @ u)
            if x < 1e-9 or x > 2 - 1e-9:
                raise ValueError("geodesic hits a cone point")
            g = self.e2f[(b, a)]
            # unfold: keep the components along the edge and across it
            e = self._unit(self.V[b] - self.V[a])
            in_f, in_g = np.cross(self.N[f], e), np.cross(self.N[g], -e)
            d2 = (d @ e) * e - (d @ in_f) * in_g
            out.append((f, p, q, d))
            p, d, f = q, self._unit(d2), g
            left -= lam
        return out


def crossing(pa, pb):
    """First crossing of two traced geodesics: (point, angle in degrees, face) or None."""
    for f, p0, p1, d in pa:
        for g, q0, q1, e in pb:
            if f != g:
                continue
            # solve p0 + s (p1-p0) = q0 + t (q1-q0) in the plane of the face
            A = np.array([p1 - p0, -(q1 - q0)]).T
            sol, res, rk, _ = np.linalg.lstsq(A, q0 - p0, rcond=None)
            if rk < 2:
                continue
            s, t = sol
            x = p0 + s * (p1 - p0)
            if -1e-12 <= s <= 1 + 1e-12 and -1e-12 <= t <= 1 + 1e-12 and np.linalg.norm(x - (q0 + t * (q1 - q0))) < 1e-9:
                return x, math.degrees(math.acos(max(-1.0, min(1.0, float(d @ e))))), f
    return None


def two_rays(S, b, L=1.0, length=4.0, vertex=0, shift=0.0):
    """Two parallel geodesics in a face at `vertex`, running along the face's bisector toward the vertex, offset by
    +-b (and both shifted sideways by `shift`).  With shift = 0 they pass the cone point on opposite sides."""
    f = next(i for i, t in enumerate(S.F) if vertex in t)
    t = S.F[f]
    c = sum(S.V[x] for x in t) / 3.0
    u = S._unit(S.V[vertex] - c)
    w = np.cross(S.N[f], u)
    base = S.V[vertex] - L * u + shift * w
    return S.trace(f, base + b * w, u, length), S.trace(f, base - b * w, u, length), S.V[vertex]


def lens(b, **kw):
    S = Surface()
    ra, rb, apex = two_rays(S, b, **kw)
    x = crossing(ra, rb)
    if x is None:
        return None
    return {"b": b, "angle_deg": x[1], "distance": float(np.linalg.norm(x[0] - apex)), "predicted_distance": b / math.sin(math.radians(DEFICIT / 2))}


def cone_table(deficits=(60, 90, 120, 180, 240, 270)):
    """Flat cone of a given deficit: crossing angle of two parallel geodesics around the apex (the deficit, when it is
    below 180), crossing distance over b, and the number of self-crossings of one geodesic, #{k >= 1 : k theta < 180}."""
    rows = []
    for d in deficits:
        th = 360 - d
        rows.append({"deficit": d, "cone_angle": th,
                     "crossing_angle": d if d < 180 else None,
                     "crossing_distance_over_b": round(1 / math.sin(math.radians(d / 2)), 6) if d < 180 else None,
                     "self_crossings": sum(1 for k in range(1, 10) if k * th < 180),
                     "mass_8piG_units": round(d / 720, 6)})      # share of the total 720 degrees of a sphere
    return rows


# ---- tests --------------------------------------------------------------------------------------
def test_icosahedron():
    V, F = icosahedron()
    assert len(V) == 12 and len(F) == 20


def test_geodesic_length_and_tangency():
    S = Surface()
    ra, _, _ = two_rays(S, 0.2)
    assert abs(sum(np.linalg.norm(q - p) for _, p, q, _ in ra) - 4.0) < 1e-9
    assert all(abs(d @ S.N[f]) < 1e-9 and abs(np.linalg.norm(d) - 1) < 1e-9 for f, _, _, d in ra)


def test_deflection_is_the_deficit_for_every_impact_parameter():
    for b in (0.02, 0.05, 0.1, 0.2, 0.3, 0.4):
        r = lens(b)
        assert r is not None and abs(r["angle_deg"] - DEFICIT) < 1e-7, r


def test_crossing_distance_is_linear_in_b():
    for b in (0.05, 0.1, 0.2, 0.4):
        r = lens(b)
        assert abs(r["distance"] - 2 * b) < 1e-8, r


def test_no_cone_point_between_no_deflection():
    S = Surface()
    ra, rb, _ = two_rays(S, 0.05, shift=0.3, length=3.0)         # both rays pass the vertex on the same side
    assert crossing(ra, rb) is None
    # where both are in one face they are still parallel
    same = [(d, e) for f, _, _, d in ra for g, _, _, e in rb if f == g]
    assert same and all(abs(d @ e - 1) < 1e-9 for d, e in same)


def test_cone_table():
    t = {r["deficit"]: r for r in cone_table()}
    assert t[60]["crossing_distance_over_b"] == 2.0 and t[60]["self_crossings"] == 0
    assert t[240]["self_crossings"] == 1 and t[270]["self_crossings"] == 1 and t[180]["self_crossings"] == 0
    assert abs(12 * t[60]["mass_8piG_units"] - 1) < 1e-5


def run_tests():
    fs = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in fs:
        f()
        print("ok", f.__name__)
    print("%d checks pass" % len(fs))


if __name__ == "__main__":
    run_tests()
    if "--tests-only" not in sys.argv:
        out = {"deficit": DEFICIT, "lens": [lens(b) for b in (0.02, 0.05, 0.1, 0.2, 0.3, 0.4)], "cones": cone_table()}
        json.dump(out, open(os.path.join(HERE, "results_curvature.json"), "w"), indent=1)
        print(json.dumps(out, indent=1))
