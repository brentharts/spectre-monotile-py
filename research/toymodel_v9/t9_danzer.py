"""toymodel_v9 track B: Danzer ABCK from D6 (stage B1) and the 7-fold inflation (stage B3).
Run: python3 t9_danzer.py --tests-only
Each test prints PASS/FAIL. Labels: Exact = exact algebra (sympy) ; Numeric = floating point with stated tolerance."""
import sys, itertools
import numpy as np
import sympy as sp
from scipy.spatial import ConvexHull

tau = (1 + 5 ** .5) / 2
T = sp.Rational(1, 2) + sp.sqrt(5) / 2
results = []
def check(name, ok, info=""):
    results.append(ok); print(("PASS " if ok else "FAIL ") + name + ("  " + info if info else ""))

# ---------- B1.1 icosahedral projection of Z^6 (Exact) ----------
# columns: six 5-fold axes (0,1,tau) cyclic, the standard E_par; E_perp = Galois conjugate tau -> -1/tau
def axes(t):
    return sp.Matrix([[0, 1, t], [0, -1, t], [1, t, 0], [-1, t, 0], [t, 0, 1], [t, 0, -1]]).T
Epar = axes(T)                       # 3x6
Eperp = axes(sp.Rational(1, 2) - sp.sqrt(5) / 2)   # tau' = -1/tau
n_par = sp.simplify((Epar * Epar.T)[0, 0]); n_perp = sp.simplify((Eperp * Eperp.T)[0, 0])
check("B1.1 rows of Epar (and of Eperp) are orthogonal with equal norms", sp.simplify(Epar * Epar.T - n_par * sp.eye(3)) == sp.zeros(3, 3) and sp.simplify(Eperp * Eperp.T - n_perp * sp.eye(3)) == sp.zeros(3, 3), "norms^2 = %s, %s" % (n_par, n_perp))
P = sp.simplify(Epar.T * Epar / n_par + Eperp.T * Eperp / n_perp)
check("B1.1 the two projectors sum to the identity on R^6 (orthogonal 6D split)", sp.simplify(P - sp.eye(6)) == sp.zeros(6, 6))
check("B1.1 Epar Eperp^T = 0 (E_par orthogonal to E_perp)", sp.simplify(Epar * Eperp.T) == sp.zeros(3, 3))

# ---------- B1.2 tau-inflation as an integer 6x6 matrix on D6 (Exact) ----------
# find integer S with Epar S = tau Epar and Eperp S = -1/tau Eperp
A = sp.Matrix.vstack(Epar, Eperp)
Tm = sp.diag(T, T, T, sp.Rational(1, 2) - sp.sqrt(5) / 2, sp.Rational(1, 2) - sp.sqrt(5) / 2, sp.Rational(1, 2) - sp.sqrt(5) / 2)
S = sp.simplify(A.inv() * Tm * A)
S = S.applyfunc(sp.nsimplify)
print("S =", S.tolist())
check("B1.2 every entry of S is +-1/2 (S is NOT an integer matrix: it does not preserve Z^6)", all(abs(z) == sp.Rational(1, 2) for z in S))
check("B1.2 S maps e_1 outside Z^6 (so the P-type lattice Z^6 is not S-invariant)", not all(z == sp.floor(z) for z in S[:, 0]))
D6basis = [sp.Matrix([1 if k == i else (1 if k == j else 0) for k in range(6)]) for i in range(6) for j in range(i + 1, 6)]
check("B1.2 S maps every D6 root e_i+e_j into D6 (exact)", all(sum(S * b) % 2 == 0 and all(sp.simplify(z - sp.floor(z)) == 0 for z in S * b) for b in D6basis))
check("B1.2 S^2 = S + I and |det S| = 1", sp.simplify(S * S - S - sp.eye(6)) == sp.zeros(6, 6) and abs(S.det()) == 1)
Sn = np.array(S.tolist(), dtype=float)
rng = np.random.default_rng(1)
V = rng.integers(-5, 6, size=(2000, 6)); V = V[V.sum(1) % 2 == 0]
W = V @ Sn.T
check("B1.2 S maps random D6 vectors to integer vectors of even sum", bool(np.all(np.abs(W - np.rint(W)) < 1e-9) and np.all(np.rint(W).sum(1) % 2 == 0)))
Wi = V @ np.linalg.inv(Sn).T
check("B1.2 S^-1 maps random D6 vectors to integer vectors of even sum (D6 is S-invariant)", bool(np.all(np.abs(Wi - np.rint(Wi)) < 1e-9) and np.all(np.rint(Wi).sum(1) % 2 == 0)))

# ---------- B1.3 rhombic triacontahedron and the tile K (Numeric) ----------
pts = []
for s1, s2 in itertools.product([1, -1], repeat=2):
    pts += [(0, s1, s2 * tau), (s1, s2 * tau, 0), (s2 * tau, 0, s1)]       # 12 five-fold points
for s in itertools.product([1, -1], repeat=3):
    pts.append(s)                                                         # 8 cube corners
    pts.append((0, s[1] * tau, s[2] / tau)); pts.append((s[0] / tau, 0, s[2] * tau)); pts.append((s[0] * tau, s[1] / tau, 0))
pts = np.unique(np.round(np.array(pts), 9), axis=0)
r = np.linalg.norm(pts, axis=1)
five = pts[np.isclose(r, np.sqrt(1 + tau ** 2))]; three = pts[np.isclose(r, np.sqrt(3))]
check("B1.3 triacontahedron has 12 five-fold and 20 three-fold vertices", len(five) == 12 and len(three) == 20, "%d, %d" % (len(five), len(three)))
H = ConvexHull(np.vstack([five, three]))
vol_tri = H.volume
# group triangles into planar faces by equation
eqs = np.round(H.equations, 6)
faces = {}
for simp, eq in zip(H.simplices, eqs):
    faces.setdefault(tuple(eq), set()).update(simp.tolist())
allp = np.vstack([five, three])
check("B1.3 hull has 30 rhombic faces of 4 vertices", len(faces) == 30 and all(len(v) == 4 for v in faces.values()))
face = list(faces.values())[0]; idx = sorted(face)
P5 = [allp[i] for i in idx if np.isclose(np.linalg.norm(allp[i]), np.sqrt(1 + tau ** 2))]
P3 = [allp[i] for i in idx if np.isclose(np.linalg.norm(allp[i]), np.sqrt(3))]
centre = np.mean([allp[i] for i in idx], axis=0)
K = np.array([[0, 0, 0], P5[0], P3[0], centre])
vK = abs(np.linalg.det(K[1:] - K[0])) / 6
check("B1.3 120 copies of K = triacontahedron volume", abs(120 * vK - vol_tri) < 1e-6, "vol K = %.6f, 120 vol K = %.6f" % (vK, vol_tri))
# edge lengths of K at the scale used by the paper (triacontahedron scaled by 1/2)
e = sorted(np.linalg.norm(K[i] - K[j]) / 2 for i in range(4) for j in range(i + 1, 4))
check("B1.3 K contains edges a=sqrt(2+tau)/2 and b=sqrt(3)/2 (scale 1/2)", any(abs(x - np.sqrt(2 + tau) / 2) < 1e-9 for x in e) and any(abs(x - np.sqrt(3) / 2) < 1e-9 for x in e), "edges " + ", ".join("%.4f" % x for x in e))

# ---------- B1.4 inflation matrix of ABCK (Exact, from arXiv:2003.13449 as copied) ----------
M = sp.Matrix([[1, 1, 0, 0], [4, 2, 1, 0], [2, 0, 2, 1], [6, 3, 2, 0]])   # rows tauK,tauB,tauC,tauA ; cols K,B,C,A
x = sp.symbols('x')
cp = sp.factor(M.charpoly(x).as_expr(), extension=sp.sqrt(5))
check("B1.4 charpoly of the inflation matrix has the root tau^3", sp.simplify(M.charpoly(x).as_expr().subs(x, T ** 3)) == 0)
# volumes: right eigenvector for tau^3 normalised to K = 1
ns = (M - T ** 3 * sp.eye(4)).nullspace()
check("B1.4 volume eigenvector exists for tau^3", len(ns) == 1)
v = sp.simplify(ns[0] / ns[0][0])
exp = [1, 2 * T, 2 * T, 2 * T + 2]
check("B1.4 volumes K:B:C:A = 1 : 2tau : 2tau : 2tau+2", all(sp.simplify(v[i] - exp[i]) == 0 for i in range(4)), str([sp.nsimplify(sp.simplify(z)) for z in v]))
# tile frequencies: left eigenvector
nl = (M.T - T ** 3 * sp.eye(4)).nullspace()[0]
nl = sp.simplify(nl / sum(nl))
check("B1.4 tile frequencies (left eigenvector) positive", all(float(z) > 0 for z in nl), "K,B,C,A = " + ", ".join("%.5f" % float(z) for z in nl))
# the triacontahedron volume in units of K is 120; the rhombic triacontahedron has volume 4*sqrt(5+2 sqrt5)... check independent formula
check("B1.3 triacontahedron volume formula 4 sqrt(5+2 sqrt5) a^3 with a = rhombus edge", abs(vol_tri - 4 * np.sqrt(5 + 2 * np.sqrt(5)) * np.linalg.norm(P5[0] - P3[0]) ** 3) < 1e-6, "vol=%.6f" % vol_tri)

# ---------- B1.5 cross-check against Frettloh's note (Table 1 and inflation matrix) ----------
Mf = sp.Matrix([[0, 0, 1, 0], [3, 2, 0, 1], [2, 1, 2, 0], [6, 4, 2, 1]])         # as copied; order A,B,C,K
perm = [3, 1, 2, 0]                                                             # (K,B,C,A) -> index in (A,B,C,K)
Mk_in_ABCK = sp.Matrix(4, 4, lambda i, j: M[[3, 1, 2, 0][i], [3, 1, 2, 0][j]])  # Koca matrix reordered to rows/cols A,B,C,K
check("B1.5 Frettloh's inflation matrix is the transpose of Koca's, reordered to (A,B,C,K)", Mf == Mk_in_ABCK.T)
check("B1.5 its eigenvalues are tau^3, tau, -1/tau, -tau^-3", sorted(float(sp.N(r)) for r in Mf.eigenvals()) == sorted([float(sp.N(t)) for t in (T ** 3, T, -1 / T, -T ** -3)]) or all(abs(sorted(np.linalg.eigvals(np.array(Mf.tolist(), float)).real)[i] - sorted([tau ** 3, tau, -1 / tau, -tau ** -3])[i]) < 1e-9 for i in range(4)))
t = tau
tiles = {"A": [(0, 0, 0), (t ** 3, 0, t ** 2), (t ** 2, t ** 2, t ** 2), (t ** 2, 1, 0)],
         "B": [(0, 0, 0), (t ** 3, 0, t ** 2), (t ** 2, t ** 2, t ** 2), (t ** 2, t, 1)],
         "C": [(0, 0, 0), (-t, 0, 1), (t ** 2, t ** 2, t ** 2), (0, t ** 2, 1)],
         "K": [(0, 0, 0), (-1, t, 0), (t, t, t), (-.5, .5 / t, .5 * t)]}
vol = {k: abs(np.linalg.det(np.array(v[1:]) - np.array(v[0]))) / 6 for k, v in tiles.items()}
rat = [vol[k] / vol["K"] for k in "KBCA"]
check("B1.5 volumes of A,B,C,K from Table 1 coordinates give K:B:C:A = 1 : 2tau : 2tau : 2tau+2", all(abs(a - b) < 1e-9 for a, b in zip(rat, [1, 2 * tau, 2 * tau, 2 * tau + 2])), "ratios " + ", ".join("%.4f" % r for r in rat))
def edge_ratios(P):
    P = np.array(P); e = sorted(np.linalg.norm(P[i] - P[j]) for i in range(4) for j in range(i + 1, 4)); return np.array(e) / e[0]
ek = edge_ratios(tiles["K"])
check("B1.5 K of Table 1 has edge ratios of a similar tetrahedron to our K", np.allclose(ek, edge_ratios(K), atol=1e-9) or np.allclose(ek, edge_ratios(K) , atol=1e-6), "Table-1 K %s ; ours %s" % (np.round(ek, 4), np.round(edge_ratios(K), 4)))
# tile frequencies from the matrix agree with Frettloh's numbers
f_ABCK = [0.0379, 0.2829, 0.1604, 0.5189]
fl = [float(nl[3]), float(nl[1]), float(nl[2]), float(nl[0])]       # nl was (K,B,C,A) -> (A,B,C,K)
check("B1.5 frequencies from the matrix agree with the published A,B,C,K values to 4 digits", all(abs(a - b) < 6e-5 for a, b in zip(fl, f_ABCK)), "ours " + ", ".join("%.5f" % x for x in fl))
# vertex densities: class windows over det(D6)
dII = T / 20
detD6 = 4 * (7 * T + 4) / dII
check("B1.5 densities: window volumes / det(D6) with det fixed by class II reproduce 0.26631, 0.06910 (class I, III)", abs(float(20 * (5 * T + 2) / detD6) - 0.26631) < 1e-4 and abs(float(20 * T ** 2 / detD6) - 0.06910) < 1e-4, "det(D6) in these units = %.3f ; total %.5f" % (float(detD6), float((20 * (5 * T + 2) + 4 * (7 * T + 4) + 20 * T ** 2) / detD6)))

# ---------- B1.6 Table 1 vertices as exact lattice vectors in our frame (Exact) ----------
Tc = sp.Rational(1, 2) - sp.sqrt(5) / 2
Afull = sp.Matrix.vstack(axes(T), axes(Tc)); Ainv = Afull.inv()
conjf = lambda e: sp.simplify(sp.expand(e).subs(sp.sqrt(5), -sp.sqrt(5)))
tT = T
Tab = {"A": [(0, 0, 0), (tT ** 3, 0, tT ** 2), (tT ** 2, tT ** 2, tT ** 2), (tT ** 2, 1, 0)],
       "B": [(0, 0, 0), (tT ** 3, 0, tT ** 2), (tT ** 2, tT ** 2, tT ** 2), (tT ** 2, tT, 1)],
       "C": [(0, 0, 0), (-tT, 0, 1), (tT ** 2, tT ** 2, tT ** 2), (0, tT ** 2, 1)],
       "K": [(0, 0, 0), (-1, tT, 0), (tT, tT, tT), (-sp.Rational(1, 2), sp.Rational(1, 2) / tT, sp.Rational(1, 2) * tT)]}
def lattice_vec(p):
    v = sp.Matrix(p); vc = v.applyfunc(conjf)
    return (Ainv * sp.Matrix.vstack(v, vc)).applyfunc(lambda z: sp.nsimplify(sp.simplify(z)))
def coset(x):
    h = sp.Rational(1, 2)
    if all(z == sp.floor(z) for z in x): return "D6" if sum(x) % 2 == 0 else "v"
    if all((z - h) == sp.floor(z - h) for z in x): return "s" if sum(x) % 2 == 1 else "c"
    return "other"
cos = {k: [coset(lattice_vec(p)) for p in v[1:]] for k, v in Tab.items()}
alln = all(all(z.is_rational for z in lattice_vec(p)) for v in Tab.values() for p in v)
check("B1.6 every Table 1 vertex of A,B,C,K is a rational vector in our 6D coordinates (E_perp = Galois conjugate)", alln)
check("B1.6 vertex cosets relative to the first vertex: A: s,v,s ; B: s,v,D6 ; C: v,v,s", cos["A"] == ["s", "v", "s"] and cos["B"] == ["s", "v", "D6"] and cos["C"] == ["v", "v", "s"], str(cos))
check("B1.6 K has a vertex in none of D6, v, s, c (its fourth vertex, 'class IV')", cos["K"] == ["v", "s", "other"], str(cos["K"]))
cov = 2 * abs(Afull.det())
check("B1.6 covolume of D6 in our frame is 80 sqrt(5); the note's det(D6) is tau^3 times larger (757.77)", sp.simplify(cov - 80 * sp.sqrt(5)) == 0 and abs(757.771 / float(cov) - tau ** 3) < 1e-5, "ratio %.6f vs tau^3 %.6f" % (757.771 / float(cov), tau ** 3))

# ---------- B1.7 windows from the Delone cells of D6 (Numeric + Exact) ----------
star = lambda x: tau * (np.array([[0, 1, 1 - tau], [0, -1, 1 - tau], [1, 1 - tau, 0], [-1, 1 - tau, 0], [1 - tau, 0, 1], [1 - tau, 0, -1]]).T @ x)   # tau * conjugation
cross = [sg * np.eye(6)[j] for j in range(6) for sg in (1, -1)]                 # Delone cell of the shallow hole e_1: 6-cross-polytope, centre removed
Hc = ConvexHull(np.array([star(v) for v in cross]))
cp = np.array([star(v) for v in cross]); ico_edge = min(np.linalg.norm(p - q) for p in cp for q in cp if np.linalg.norm(p - q) > 1e-9)
check("B1.7 cross-polytope cell projects (star frame) to an icosahedron of edge 2 and volume 5/12(3+sqrt5)*8", len(Hc.vertices) == 12 and abs(ico_edge - 2) < 1e-9 and abs(Hc.volume - 5 / 12 * (3 + 5 ** .5) * 8) < 1e-9, "volume %.6f" % Hc.volume)
dem = {par: np.array([star(np.array(b) - 0.5) for b in itertools.product([0, 1], repeat=6) if sum(b) % 2 == par]) for par in (0, 1)}
Hd = ConvexHull(dem[1])
a_d = min(np.linalg.norm(p - q) for p in dem[1][Hd.vertices] for q in dem[1][Hd.vertices] if np.linalg.norm(p - q) > 1e-9)
check("B1.7 odd demicube cell (deep hole) projects to a regular dodecahedron: 20 vertices, 12 pentagonal faces' worth (36 triangles), edge 2", len(Hd.vertices) == 20 and len(Hd.simplices) == 36 and abs(a_d - 2) < 1e-9, "edge %.6f" % a_d)
check("B1.7 its volume is 4(7tau+4) = 61.305 (the note's class II window; dodecahedron of edge 2 has volume (15+7sqrt5)/4 * 8)", abs(Hd.volume - 4 * (7 * tau + 4)) < 1e-9 and abs(Hd.volume - (15 + 7 * 5 ** .5) / 4 * 8) < 1e-9, "%.6f" % Hd.volume)
detstar = 2 * abs(float(Afull.det())) * tau ** 3
check("B1.7 covolume in the star frame (tau * conjugation) is 757.77 and class II density = tau/20", abs(detstar - 757.77) < 0.01 and abs(Hd.volume / detstar - tau / 20) < 1e-6, "det %.3f density %.6f" % (detstar, Hd.volume / detstar))
check("B1.7 the Baake-Grimm volume 4(tau+2) over the plain-conjugation covolume 80 sqrt5 gives the same density tau/20 (window and covolume both scale by tau^3)", abs(4 * (tau + 2) / (80 * 5 ** .5) - tau / 20) < 1e-12, "%.6f" % (4 * (tau + 2) / (80 * 5 ** .5)))
check("B1.7 class III volume 20 tau^2 = 3 x icosahedron(edge 2); class I 20(5tau+2) is not a Delone-cell projection (even demicube hull has volume %.4f)" % ConvexHull(dem[0]).volume, abs(20 * tau ** 2 - 3 * Hc.volume) < 1e-9 and abs(ConvexHull(dem[0]).volume - 20 * (5 * tau + 2)) > 1)

# ---------- B3 the 7-fold inflation factor (Exact + Numeric) ----------
l7 = 1 + sp.sin(2 * sp.pi / 7) / sp.sin(sp.pi / 7)
mp = sp.minimal_polynomial(1 + 2 * sp.cos(sp.pi / 7), x)
lv = float(l7)
check("B3 1 + sin(2pi/7)/sin(pi/7) = 1 + 2cos(pi/7)", abs(lv - (1 + 2 * np.cos(np.pi / 7))) < 1e-12, "= %.9f" % lv)
check("B3 its minimal polynomial is x^3 - 4x^2 + 3x + 1", sp.expand(mp - (x ** 3 - 4 * x ** 2 + 3 * x + 1)) == 0, str(mp))
check("B3 that polynomial is irreducible over Q (degree 3)", sp.Poly(mp, x).is_irreducible)
conj = sorted(np.roots([1, -4, 3, 1]), key=lambda z: -abs(z))
check("B3 not a PV number: a conjugate has modulus > 1", abs(conj[1]) > 1, "roots %s" % ", ".join("%.5f" % z.real for z in conj))
check("B3 the Spectre inflation lambda^2 = 4+sqrt(15) is Pisot (conjugate 4-sqrt(15) < 1)", abs(4 - 15 ** .5) < 1)

print("\n%d/%d passed" % (sum(results), len(results)))
sys.exit(0 if all(results) else 1)
