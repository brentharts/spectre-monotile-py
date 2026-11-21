"""Fit planes n.x=h to the boundary of the folded window clouds (RANSAC on outermost points per direction cell)."""
import sys; sys.path.insert(0, '.')
import numpy as np, t9_win_fold as Fo
from scipy.spatial import cKDTree
def surface(F, N=70, cap=0.012):
    r = np.linalg.norm(F, axis=1); D = F / r[:, None]; T = cKDTree(D); pts = []
    for i in range(N + 1):
        for j in range(N + 1 - i):
            k = N - i - j; v = i * Fo.a5 + j * Fo.a3 + k * Fo.a2; v /= np.linalg.norm(v)
            idx = T.query_ball_point(v, cap)
            if len(idx) > 8:
                s = np.sort(r[idx]); pts.append(v * s[-3])
    return np.array(pts)
def ransac(S, tol=0.015, minin=25, rounds=4000, seed=0):
    rng = np.random.default_rng(seed); rem = np.ones(len(S), bool); planes = []
    while rem.sum() > minin:
        idx = np.flatnonzero(rem); best = None
        for _ in range(rounds):
            s = S[rng.choice(idx, 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
            if nn < 1e-9: continue
            n /= nn; h = n @ s[0]
            if h < 0: n, h = -n, -h
            inl = rem & (np.abs(S @ n - h) < tol)
            if best is None or inl.sum() > best[0]: best = (inl.sum(), n, h, inl)
        if best[0] < minin: break
        # refine by least squares
        inl = best[3]; A = np.c_[S[inl], -np.ones(inl.sum())]; _, _, Vt = np.linalg.svd(A - A.mean(0) * 0, full_matrices=False)
        planes.append((best[0], best[1], best[2])); rem &= ~inl
    return planes, rem.sum()
if __name__ == "__main__":
    for n in ("I", "II", "III"):
        F = np.load("win_fold_%s.npy" % n); S = surface(F); planes, left = ransac(S)
        print(n, "surface pts", len(S), "planes", len(planes), "unassigned", left)
        for c, nv, h in planes: print("   inliers %4d  n=(%.3f %.3f %.3f) h=%.3f  angle to a5/a3/a2 = %.1f/%.1f/%.1f deg" % (c, *nv, h, *np.degrees(np.arccos([nv @ Fo.a5, nv @ Fo.a3, nv @ Fo.a2]))))
