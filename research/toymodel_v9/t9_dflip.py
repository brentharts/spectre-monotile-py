"""Stage B2: phason-flip engine for the Danzer ABCK model set (selected dissection of STAGE_B1c, windows of STAGE_B1d).
Vertices of class c at phase gamma:  { x in coset_c of the weight lattice D6* : star(x) + gamma in W_c },  star frame, internal shift gamma (3-vector).
Classes (level-11 labelling, key = P.class_key): I = half-integer coset ((2,..,2),1), II = the other half-integer coset ((2,..,2),0), III = the root lattice D6 itself ((0,..,0),0); class IV (quarter lattice, window volume about 1) is NOT modelled.
Window I: tabulated radial oracle (win_I_table.npz, about 1%); windows II and III: exact (t9_win_def)."""
import sys; sys.path.insert(0, '.')
import numpy as np
import t9_dissect as d, t9_patch as P, t9_win_def as W, t9_win_oracle as O
tau = d.tau; D = 4
CLS = {"I": ((2,) * 6, 1), "II": ((2,) * 6, 0), "III": ((0,) * 6, 0)}
MAT = np.vstack([d.Ep, tau * d.Eq]) / D          # x4 -> (phys, star)
BASIS = np.array([4 * np.eye(6, dtype=int)[i] for i in range(5)] + [[2] * 6])   # generates D6* (x4 units)
_oracle = None
def window(cls, X):
    global _oracle
    if cls == "II": return W.inII(X)
    if cls == "III": return W.inIII(X)
    if _oracle is None: _oracle = O.RadialOracle.load("win_I_table.npz"); _oracle.margin = 1.004   # radial margin fitted on the level-11 patch (patch points have r/R <= 1.002, non-patch lattice points >= 1.006)
    return _oracle(X)
def enumerate_points(cen, R, Rs):
    """all x4 in D6* with |phys(x)-cen| <= R and |star(x)| <= Rs (ellipsoid; Fincke-Pohst on the QR of the weighted embedding)."""
    Wd = np.diag([1 / R] * 3 + [1 / Rs] * 3); F = Wd @ MAT @ BASIS.T; w = Wd @ np.concatenate([cen, np.zeros(3)])
    Q, Rm = np.linalg.qr(F); b = Q.T @ w; out = []; c = np.zeros(6)
    def rec(k, rem):
        if k < 0: out.append(c.copy()); return
        s = b[k] - sum(Rm[k, j] * c[j] for j in range(k + 1, 6)); r = Rm[k, k]
        lo = (s - np.sqrt(max(rem, 0))) / r; hi = (s + np.sqrt(max(rem, 0))) / r
        if lo > hi: lo, hi = hi, lo
        for v in range(int(np.ceil(lo - 1e-9)), int(np.floor(hi + 1e-9)) + 1):
            c[k] = v; rec(k - 1, rem - (r * v - s) ** 2)
    rec(5, 1.0); C = np.array(out); return np.rint(C @ BASIS).astype(int)
def points_box(cen, R, Rs):
    """all x4 with |phys-cen| <= R AND |star| <= Rs (the ellipsoid is enlarged by sqrt2 in both factors, then filtered)"""
    X = enumerate_points(cen, R * 2 ** .5, Rs * 2 ** .5); ph = X @ MAT[:3].T; st = X @ MAT[3:].T
    k = (np.linalg.norm(ph - cen, axis=1) <= R) & (np.linalg.norm(st, axis=1) <= Rs); return X[k]
class VertexSet:
    def __init__(self, cen, R, Rs=3.3):
        X = points_box(np.asarray(cen, float), R, Rs); keys = [P.class_key(x) for x in X]; self.cls = np.array([next((c for c, k in CLS.items() if k == kk), "") for kk in keys])
        keep = self.cls != ""; self.x4 = X[keep]; self.cls = self.cls[keep]; self.phys = self.x4 @ (MAT[:3].T); self.star = self.x4 @ (MAT[3:].T)
    def mask(self, gamma=(0, 0, 0)):
        m = np.zeros(len(self.x4), bool); g = np.asarray(gamma, float)
        for c in CLS:
            s = self.cls == c; m[s] = window(c, self.star[s] + g)
        return m
def tile_survival(vs, patch, gamma, cen, rin):
    """fraction of patch tiles (centroid within rin of cen) all of whose class I-III vertices are present at phase gamma"""
    key = {tuple(x): i for i, x in enumerate(vs.x4)}; m = vs.mask(gamma); ok = 0; n = 0; skip = 0
    for W4 in patch["verts"]:
        if np.linalg.norm(np.mean([d.phys4(w) for w in W4], 0) - cen) > rin: continue
        n += 1; good = True
        for w in W4:
            i = key.get(tuple(int(z) for z in w))
            if i is None: skip += 1; continue          # class IV (or outside enumeration)
            if not m[i]: good = False
        ok += good
    return ok / n, n, skip
