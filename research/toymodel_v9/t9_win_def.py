"""Explicit class windows (star frame; icosahedral group B, five-fold axes of type (0,tau,1)); all lengths for the Danzer scale where the dodecahedron of class I has edge 2.
 I   = D(edge 2)  minus the 12 'pits'  P_j = { x : n_k.x > 1/rho for all 5 neighbours k of five-fold axis j }
 II  = D(edge 2/tau)
 III = Ico(edge 2) minus the 20 pits   Q_m = { x : n_k.x > tau/rho for the 3 five-fold axes k nearest to three-fold axis m }
 rho = sqrt(tau*sqrt5) = circumradius of the icosahedron of edge 2; n_k are unit five-fold axes."""
import sys; sys.path.insert(0, '.')
import numpy as np
tau = (1 + 5 ** .5) / 2; rho = (tau * 5 ** .5) ** .5
def unit(a): a = np.array(a, float); return a / np.linalg.norm(a)
# five-fold axes (0,tau,1) type: 12 unit vectors; three-fold axes: 20 unit vectors (centres of icosahedron faces)
AX5 = []
for s1 in (1, -1):
    for s2 in (1, -1):
        for p in ((0, tau, 1), (tau, 1, 0), (1, 0, tau)): AX5.append(unit((p[0] * (s1 if p[0] else 1), p[1] * (s1 if p[0] == 0 else s2 if p[1] else 1), p[2] * s2)))
AX5 = np.unique(np.round(np.array([unit(v) for v in {tuple(np.round(s * np.array(p), 9)) for p in ((0, tau, 1), (tau, 1, 0), (1, 0, tau)) for s in np.array([(a, b, c) for a in (1, -1) for b in (1, -1) for c in (1, -1)])}]), 9), axis=0)
assert len(AX5) == 12, len(AX5)
G = AX5 @ AX5.T
NBR5 = [np.flatnonzero(np.abs(G[j] - 1 / 5 ** .5) < 1e-6) for j in range(12)]       # five neighbours at 63.43 deg
assert all(len(n) == 5 for n in NBR5)
AX3 = []
for a in range(12):
    for b in NBR5[a]:
        for c in NBR5[a]:
            if b < c and abs(G[b, c] - 1 / 5 ** .5) < 1e-6 and a < b: AX3.append(unit(AX5[a] + AX5[b] + AX5[c]))
AX3 = np.array(AX3); assert len(AX3) == 20, len(AX3)
NBR3 = [np.argsort(-(AX5 @ m))[:3] for m in AX3]
def inD(X, edge):
    """dodecahedron of the given edge, face normals = the 12 five-fold axes (this orientation: faces on five-fold axes), inradius = edge*tau^2/(2 sqrt(3-tau))"""
    h = edge * tau ** 2 / (2 * np.sqrt(3 - tau)); return (X @ AX5.T <= h + 1e-12).all(1)
def inI(X):
    ok = inD(X, 2.0)
    for j in range(12): ok &= ~((X @ AX5[NBR5[j]].T > 1 / rho).all(1))
    return ok
def inII(X): return inD(X, 2 / tau)
def inIco(X, edge=2.0):
    h = edge * (tau ** 2 / (2 * 3 ** .5)) ; return (X @ AX3.T <= h + 1e-12).all(1)      # inradius of icosahedron edge a: a*tau^2/(2 sqrt3)
def inIII(X):
    ok = inIco(X)
    for m in range(20): ok &= ~((X @ AX5[NBR3[m]].T > tau / rho).all(1))
    return ok
if __name__ == "__main__":
    rng = np.random.default_rng(5); X = rng.uniform(-3, 3, size=(4000000, 3)); box = 216.0
    exp = {"I": 20 * (4 - tau), "II": 4 * (tau + 2), "III": 20 * (tau - 1)}
    for n, f in (("I", inI), ("II", inII), ("III", inIII)):
        print(n, "volume %.3f +- %.3f expected %.3f" % (f(X).mean() * box, np.sqrt(f(X).mean() * (1 - f(X).mean()) / len(X)) * box, exp[n]))
