"""Approximate membership oracle for window I from the folded level-11 cloud: radial function R(u) tabulated on the fundamental cone of group B (nearest-grid lookup).
 Exact definitions are in t9_win_def.py (II, III). usage: python3 t9_win_oracle.py  (self-test)"""
import sys; sys.path.insert(0, '.')
import numpy as np, t9_win_fold as Fo, t9_win_radial as R, t9_win_def as W
from scipy.spatial import cKDTree
class RadialOracle:
    def __init__(self, F, N=200, cap=0.008, rank=8):
        r = np.linalg.norm(F, axis=1); T = cKDTree(F / r[:, None]); self.D = []; self.R = []
        for i in range(N + 1):
            for j in range(N + 1 - i):
                v = i * Fo.a5 + j * Fo.a3 + (N - i - j) * Fo.a2; v /= np.linalg.norm(v); idx = T.query_ball_point(v, cap)
                if len(idx) > rank: self.D.append(v); self.R.append(np.sort(r[idx])[-rank])
        self.T = cKDTree(np.array(self.D)); self.R = np.array(self.R)
    def save(self, fn): np.savez_compressed(fn, D=np.array(self.D), R=self.R)
    @classmethod
    def load(cls, fn):
        z = np.load(fn); o = cls.__new__(cls); o.D = z["D"]; o.R = z["R"]; o.T = cKDTree(o.D); return o
    def __call__(self, X):
        Y = Fo.to_cone(np.asarray(X, float)); r = np.linalg.norm(Y, axis=1); _, k = self.T.query(Y / np.maximum(r, 1e-12)[:, None]); return r <= getattr(self, 'margin', 1.0) * self.R[k]
if __name__ == "__main__" and "--save" in sys.argv:
    C = R.load(); O_ = RadialOracle(Fo.to_cone(C["I"])); O_.save("win_I_table.npz"); print("saved win_I_table.npz,", len(O_.R), "cells")
elif __name__ == "__main__":
    C = R.load(); F = np.load("win_fold_I.npy"); O = RadialOracle(F)
    rng = np.random.default_rng(3); X = rng.uniform(-3, 3, size=(400000, 3)); m = O(X)
    print("oracle volume %.2f (expected 47.64)" % (m.mean() * 216))
    P = C['I'][::7]; print("cloud points inside oracle: %.4f" % O(P * 0.995).mean())
    d, _ = cKDTree(C['I']).query(X); print("random points with no cloud point within 0.12 that the oracle accepts: %.4f of accepted" % ((d >= 0.12) & m).sum() / max(m.sum(), 1) if False else "accepted-but-cloud-empty fraction %.4f" % (((d >= 0.12) & m).sum() / m.sum()))
    print("same for definitions: II inside-def-but-empty %.4f, III %.4f" % (((cKDTree(C['II']).query(X)[0] >= 0.12) & W.inII(X)).sum() / W.inII(X).sum(), ((cKDTree(C['III']).query(X)[0] >= 0.12) & W.inIII(X)).sum() / W.inIII(X).sum()))
