"""Fold a window cloud into the fundamental cone of the icosahedral group B (x120 points), give the radial function on the cone and fit planes 1/R(u)=m.u"""
import sys; sys.path.insert(0, '.')
import numpy as np, t9_win_radial as R, t9_win_sym as S
from scipy.spatial import cKDTree
tau = R.tau
B = S.group([S.rot((0, tau, 1), 2 * np.pi / 5), S.rot((1, 1, 1), 2 * np.pi / 3)]); B = B + [-m for m in B]
a5 = R.unit((0, tau, 1))
# 3-fold axis nearest to a5 among B-orbit of (1,1,1); 2-fold nearest among orbit of (1,0,0)... use orbit search
def nearest(seed, ref, excl=None):
    O = np.array([m @ R.unit(seed) for m in B]); c = O @ ref; c[c > 0.9999999] = -2; return O[np.argmax(c)]
a3 = nearest((1, 1, 1), a5); a2 = None
O2 = np.array([m @ R.unit((1, 0, 0)) for m in B]); cand = [v for v in O2 if v @ a5 > 0.8 and v @ a3 > 0.8]; a2 = cand[0]
def to_cone(P):
    """fold every point to the cone spanned by a5,a3,a2 using the 120 group elements"""
    Mi = np.linalg.inv(np.array([a5, a3, a2]).T)    # barycentric (positive) test
    best = None; out = np.zeros_like(P); done = np.zeros(len(P), bool)
    for m in B:
        Q = P @ m.T; l = Q @ Mi.T; ok = (l >= -1e-9).all(1) & ~done; out[ok] = Q[ok]; done |= ok
    assert done.all(); return out
if __name__ == "__main__":
    C = R.load(); print("angles a5-a3 %.2f a5-a2 %.2f a3-a2 %.2f deg" % tuple(np.degrees(np.arccos([a5 @ a3, a5 @ a2, a3 @ a2]))))
    res = {}
    for n, P in C.items():
        F = to_cone(P); print(n, "folded points", len(F))
        r = np.linalg.norm(F, axis=1); D = F / r[:, None]; T = cKDTree(D)
        # grid on the cone: barycentric weights
        gs = []
        for i in range(0, 13):
            for j in range(0, 13 - i):
                k = 12 - i - j; v = (i * a5 + j * a3 + k * a2); v /= np.linalg.norm(v); gs.append(((i, j, k), v))
        rows = []
        for w, v in gs:
            idx = T.query_ball_point(v, 0.03)
            rows.append((w, v, np.sort(r[idx])[-3] if len(idx) > 5 else np.nan, len(idx)))
        res[n] = rows
        print("   weights(a5,a3,a2)/12   R    1/R   (n)")
        for w, v, Rv, nn in rows[::3]: print("   %s  %.3f  %.3f  (%d)" % (w, Rv, 1 / Rv, nn))
        np.save("win_fold_%s.npy" % n, F)
