"""toymodel_v9 track A, stage 0: the Spectre phason-flip engine as a module, with tests.

Refactor of wada-coudene/figures/spectre_flip_contacts.py (same algorithm, no module-level side effects, no sys.argv).
Needs two clones: SPECTRE_DIR (brentharts/spectre) and WADA_DIR (wada-coudene, for lakes_of_wada and figures/spectre_window_maps.npz).

  python3 t9_engine.py --tests-only          fast tests (setup invariants, reversal audit on 300 samples)  ~ a few minutes
  python3 t9_engine.py --stats                adds the v8 disc-count and erasure regression (r2=7, 3500 samples)
  python3 t9_engine.py --full                 regression against v8: 6000 samples at delta 0.03 must give 539 / 468 / 244 / 55 / 169  (long)

Defaults are the corrected v8 settings: box PBOX=20, IN_T/OUT_T = 0.0015/0.0035, delta 0.03, footprint radius R2=1.
"""
import io, contextlib, os, sys, itertools, time, collections, json
SP = os.environ.get("SPECTRE_DIR", "/home/claude/repos/spectre")
WD = os.environ.get("WADA_DIR", "/home/claude/repos/wada-coudene")
sys.path.insert(0, os.path.join(SP, "boundary_certificate")); sys.path.insert(0, SP); sys.path.insert(0, WD)
import numpy as np, mpmath as mp, sympy, networkx as nx
from sympy.matrices.normalforms import hermite_normal_form
from scipy.spatial import cKDTree
from shapely.geometry import Polygon
from shapely.ops import unary_union as UU
with contextlib.redirect_stdout(io.StringIO()):
    import spectre_window as sw
from spectre_exact import slot_transforms, SPECTRE, mul, apply, IDENT, SUPER_RULES, ttrans, trot, to_c
from lakes_of_wada import spectre_window as SWR


class Engine:
    def __init__(self, N=3, a_class=0, pbox=20.0, r2=1.0, in_t=0.0015, out_t=0.0035, verbose=True):
        t0 = time.time(); self.N, self.a, self.PBOX, self.R2, self.IN_T, self.OUT_T = N, a_class, pbox, r2, in_t, out_t
        lam = float(sw.LAM); self.RAD = float(sw.RAD)
        quads = [[SPECTRE[3], SPECTRE[5], SPECTRE[7], SPECTRE[11]]]; TS = [None]
        for j in range(N):
            Ts, nq = slot_transforms(quads[-1]); TS.append(Ts); quads.append(nq)
        Bm = mp.matrix(4, 4)
        for c, v in enumerate(sw.E[0] + sw.F[0]):
            for r in range(4): Bm[r, c] = v[r]
        Bim = mp.inverse(Bm)
        STARm = sw.T[0] * mp.matrix([[Bim[r, c] for c in range(4)] for r in (2, 3)])
        self.STAR = STAR = np.array(STARm.tolist(), dtype=float)
        groups, VEC = {}, {}
        def rec(s, j, T, z):
            if j == 0:
                key = (tuple(T[0].flatten()), 'G' if s == 'Gamma' else 'O')
                g = groups.setdefault(key, [T[0], [], []])
                g[1].append(STARm * mp.matrix([mp.mpf(int(x)) for x in T[1]])); g[2].append(z)
                VEC.setdefault(key, []).append(tuple(int(x) for x in T[1])); return
            p = (j - 1) % 2
            for i, c in enumerate(SUPER_RULES[s]):
                if c is not None:
                    rec(c, j - 1, mul(T, TS[j][i]), sw.O[(i, p)].T * (sw.GA[(i, p)] + z / sw.LAM))
        rec('Delta', N, IDENT, mp.matrix(2, 1))
        FR = []
        for (fk, S), (A, Y, Z) in groups.items():
            X = mp.matrix([[z[0], z[1], 1] for z in Z]); Ym = mp.matrix([[y[0], y[1]] for y in Y])
            NX, B2 = X.T * X, X.T * Ym
            cols = [mp.lu_solve(NX, B2[:, k]) for k in range(2)]
            sol = mp.matrix([[cols[k][i] for k in range(2)] for i in range(3)])
            R, c = sol[0:2, 0:2].T, sol[2, 0:2].T
            res = max(abs(x) for x in (X * sol - Ym)); G = R.T * R; s2 = G[0, 0]
            assert res < mp.mpf(10) ** -40 and max(abs(x) for x in (G - s2 * mp.eye(2))) < mp.mpf(10) ** -40
            sc = mp.sqrt(s2); Rm = R / sc
            FR.append((S, A, np.array(Rm.tolist(), dtype=float), np.array(c.tolist(), dtype=float).ravel(), float(sc), len(Z)))
        self.FR, self.VEC, self.keys = FR, VEC, list(groups.keys()); self.scale = FR[0][4]
        self.LEAVES = {'O': [IDENT], 'G': [IDENT, mul(ttrans(SPECTRE[8]), trot(1))]}
        self.PH = PH = np.array([[to_c(np.eye(4, dtype=object)[k]).real for k in range(4)], [to_c(np.eye(4, dtype=object)[k]).imag for k in range(4)]])
        keys = self.keys
        D = [tuple(x - y for x, y in zip(v, VEC[keys[0]][0])) for v in VEC[keys[0]][1:]]
        H = hermite_normal_form(sympy.Matrix(D).T); assert H.shape == (4, 4)
        self.detH = detH = abs(int(H.det())); self.ADJ = np.array((H.inv() * detH).tolist(), dtype=object).astype(np.int64)
        for k in keys:
            d = np.array([[x - y for x, y in zip(v, VEC[k][0])] for v in VEC[k]], dtype=np.int64); assert self.in_lattice(d).all()
        self.trees = {S: cKDTree(SWR.points(S, 0, 7)[0]) for S in "OG"}
        S1, A1, R1, c1, _, _ = FR[a_class]; self.S1, self.R1, self.c1 = S1, R1, c1
        u_a = np.array(VEC[keys[a_class]][0], dtype=np.int64)
        self.P1 = P1 = self.shape(S1, A1, np.zeros(4, dtype=object))
        ymax = 2 * self.RAD * self.scale + 2 * max(np.linalg.norm(f[3]) for f in FR) + 1
        Mi = np.linalg.inv(np.vstack([PH, STAR]))
        bnd = int(np.ceil(np.linalg.norm(Mi, axis=1).max() * np.hypot(pbox, ymax))) + 1
        rng = np.arange(-bnd, bnd + 1); cands = []
        for x0 in rng:
            Vc = np.array(list(itertools.product([x0], rng, rng, rng)))
            Vc = Vc[np.linalg.norm(Vc @ PH.T, axis=1) < pbox]
            if len(Vc): cands.append(Vc[np.linalg.norm(Vc @ STAR.T, axis=1) < ymax])
        cands = np.concatenate(cands); self.ncands = len(cands)
        PL = []
        for b in range(len(FR)):
            Sb, Ab, Rb, cb, _, _ = FR[b]; v_b = np.array(VEC[keys[b]][0], dtype=np.int64)
            W = cands[self.in_lattice(cands - (v_b - u_a))]
            dd = ((c1 - cb)[None, :] + W @ STAR.T) @ Rb / self.scale
            keep = np.linalg.norm(dd, axis=1) <= 2 * self.RAD + 0.6
            for w, d in zip(W[keep], dd[keep]):
                poly = self.shape(Sb, Ab, np.array([int(x) for x in w], dtype=object))
                if poly.distance(P1) < r2: PL.append((b, Sb, Rb.T @ R1, d, w, poly))
        self.PL = PL; self.polys = [p[5] for p in PL]
        if verbose: print("engine ready: %d classes, index %d, %d lattice candidates, %d placements, %.0fs" % (len(FR), detH, self.ncands, len(PL), time.time() - t0), flush=True)

    def in_lattice(self, W): return np.all((W @ self.ADJ.T) % self.detH == 0, axis=1)
    def shape(self, S, A, t):
        return unary_union_shape(self.LEAVES[S], A, t)

    def phases(self, NS, delta, seed=7):
        rng = np.random.default_rng(seed); Zc = self.trees[self.S1].data; lo, hi = Zc.min(0) - 0.3, Zc.max(0) + 0.3
        Z0 = rng.uniform(lo, hi, (NS, 2)); ph = rng.uniform(0, 2 * np.pi, NS)
        return Z0, Z0 + delta * np.stack([np.cos(ph), np.sin(ph)], 1)

    def state(self, Zs):
        pres = np.zeros((len(Zs), len(self.PL)), bool); amb = np.zeros((len(Zs), len(self.PL)), bool)
        for k, (b, Sb, B, d, w, poly) in enumerate(self.PL):
            dist = self.trees[Sb].query(Zs @ B.T + d)[0]
            pres[:, k] = dist < self.IN_T; amb[:, k] = (dist >= self.IN_T) & (dist <= self.OUT_T)
        return pres, amb

    def region_valid(self, pres):
        INNER = self.P1.buffer(self.R2)      # DIAM = 0 as in the v8 runs
        pol = [self.polys[k] for k in np.where(pres)[0] if self.polys[k].intersects(INNER) and self.polys[k].intersection(INNER).area > 1e-9]
        ov = sum(pol[x].intersection(pol[y]).area for x in range(len(pol)) for y in range(x + 1, len(pol)) if pol[x].intersects(pol[y]))
        return ov < 1e-6 and INNER.area - UU(pol).intersection(INNER).area < 1e-6

    @staticmethod
    def touches(p, q):
        i = p.intersection(q); return i.area < 1e-9 and i.length > 1e-6

    def pattern(self, pres):
        ks = [k for k in np.where(pres)[0] if self.polys[k].intersection(self.P1).area > 1e-9]
        G = nx.Graph(); G.add_nodes_from((k, {"c": self.PL[k][0]}) for k in ks)
        for x in range(len(ks)):
            for y in range(x + 1, len(ks)):
                if self.touches(self.polys[ks[x]], self.polys[ks[y]]): G.add_edge(ks[x], ks[y])
        return ks, G

    def stats(self, NSA=3500, delta=0.03, NB=200, M=50, EPS=0.05, seed=11, rhos=(1.5, 2, 3, 4, 5, 6, 7), partB=True):
        """Part A: changed tiles in a disc of radius rho about the origin tile, and the Mystic-pair count (v8 H8.5, H8.4).
        Part B: erasure of the origin tile, ambiguity of the inside given the outside (v8 H8.6). Port of spectre_flip_stats.py; same random stream."""
        rng = np.random.default_rng(seed); polys, PL = self.polys, self.PL
        isG = np.array([p[1] == "G" for p in PL]); cent = np.array([[q.centroid.x, q.centroid.y] for q in polys])
        c0 = np.array([self.P1.centroid.x, self.P1.centroid.y]); rad = np.linalg.norm(cent - c0, axis=1)
        Zc = self.trees[self.S1].data; lo, hi = Zc.min(0) - 0.3, Zc.max(0) + 0.3
        Z0 = rng.uniform(lo, hi, (NSA, 2)); ph = rng.uniform(0, 2 * np.pi, NSA); Z1 = Z0 + delta * np.stack([np.cos(ph), np.sin(ph)], 1)
        Pa, Aa = self.state(Z0); Pb, Ab = self.state(Z1); ok = ~(Aa.any(1) | Ab.any(1)); A = {}
        for rho in rhos:
            m = rad <= rho
            rem = Pa[ok][:, m] & ~Pb[ok][:, m]; add = ~Pa[ok][:, m] & Pb[ok][:, m]; nch = rem.sum(1) + add.sum(1); gm = isG[m]
            dG = add[:, gm].sum(1) - rem[:, gm].sum(1); dN = add.sum(1) - rem.sum(1); dA = dN + dG
            anych = nch > 0; ac = anych & (dA == 0)
            A[rho] = dict(mean_present=float(Pa[ok][:, m].sum(1).mean()), mean_changed=float(nch.mean()), frac_any=float(anych.mean()),
                          mean_changed_given_change=float(nch[anych].mean()) if anych.any() else None, n_area_conserving=int(ac.sum()),
                          P_dG_nonzero_given_area_conserved=float((dG[ac] != 0).mean()) if ac.any() else None)
        out = dict(delta=delta, NSA=NSA, usable=int(ok.sum()), partA=A)
        lr = np.log(list(rhos)); lm = np.log([max(A[r]["mean_changed"], 1e-9) for r in rhos]); out["slope"] = float(np.polyfit(lr[1:], lm[1:], 1)[0])
        if not partB: return out
        inside = np.array([polys[k].intersection(self.P1).area > 1e-9 for k in range(len(PL))]); outside = ~inside; rows = []
        for z in rng.uniform(lo, hi, (NB, 2)):
            u = rng.uniform(-1, 1, (4 * M, 2)); u = u[np.linalg.norm(u, axis=1) <= 1][:M]
            Zb = np.vstack([z[None, :], z[None, :] + EPS * u]); P, Am = self.state(Zb); good = ~Am.any(1)
            if not good[0]: continue
            P, Zb = P[good], Zb[good]; out0 = P[0][outside]
            cons = np.array([np.array_equal(p[outside], out0) for p in P])
            pats_all = set(tuple(np.where(p[inside])[0]) for p in P); pats_cons = set(tuple(np.where(p[inside])[0]) for p, c in zip(P, cons) if c)
            half = np.linalg.norm(Zb - z, axis=1) <= EPS / 2
            pats_all_h = set(tuple(np.where(p[inside])[0]) for p, h in zip(P, half) if h)
            pats_cons_h = set(tuple(np.where(p[inside])[0]) for p, c, h in zip(P, cons, half) if c and h)
            rows.append((len(pats_all), len(pats_cons), len(pats_all_h), len(pats_cons_h), int(cons.sum()), len(P)))
        out["partB"] = rows; return out

    def run(self, Z0, Z1):
        """Contact-pattern test of v8: returns counts and the per-sample record."""
        P0, A0 = self.state(Z0); P1, A1 = self.state(Z1)
        usable = ~(A0.any(1) | A1.any(1)); changed = usable & (P0 != P1).any(1)
        nvalid = [i for i in np.where(changed)[0] if self.region_valid(P0[i]) and self.region_valid(P1[i])]
        nm = lambda x, y: x["c"] == y["c"]; res = collections.Counter(); rec = {}
        for i in nvalid:
            k0, G0 = self.pattern(P0[i]); k1, G1 = self.pattern(P1[i])
            key = (set(k0) == set(k1), nx.is_isomorphic(G0, G1), nx.is_isomorphic(G0, G1, node_match=nm))
            res[key] += 1; rec[int(i)] = (key, len(k0), len(k1))
        return dict(usable=int(usable.sum()), changed=int(changed.sum()), valid=len(nvalid), patterns=res, changed_idx=sorted(int(i) for i in np.where(changed)[0]),
                    present0=P0, present1=P1, rec=rec)


def unary_union_shape(leaves, A, t):
    return UU([Polygon([(lambda w: (w.real, w.imag))(to_c(apply(mul((A, t), L), v))) for v in SPECTRE]) for L in leaves])


# ----------------------------------------------------------------------------------------------- tests
def main():
    full = "--full" in sys.argv
    results = []
    def check(name, ok, info=""):
        results.append(ok); print(("PASS " if ok else "FAIL ") + name + ("  " + info if info else ""), flush=True)
    E = Engine()
    check("A0.1 twelve window classes, six single (O) and six Mystic-pair (G)", len(E.FR) == 12 and sorted(f[0] for f in E.FR).count('G') == 6)
    check("A0.1 tiles per class at level 3 as in v8", [f[5] for f in E.FR] == [70, 75, 76, 76, 70, 14, 66, 12, 9, 8, 9, 11], str([f[5] for f in E.FR]))
    check("A0.1 similarity ratio sqrt(10)", abs(E.scale - 10 ** .5) < 1e-12)
    check("A0.1 difference lattice has index 36 in Z^4", E.detH == 36)
    check("A0.2 placement list within radius 1 of the class-0 tile has 767 entries at box 20 (v8)", len(E.PL) == 767, str(len(E.PL)))
    Z0, Z1 = E.phases(300, 0.03)
    fwd = E.run(Z0, Z1); rev = E.run(Z1, Z0)
    check("A0.3 reversal audit: the same samples change under the step and its reverse", fwd["changed_idx"] == rev["changed_idx"], "%d changed of %d usable" % (fwd["changed"], fwd["usable"]))
    check("A0.3 reversal audit: the same valid pairs and the same outcome counts", fwd["valid"] == rev["valid"] and dict(fwd["patterns"]) == dict(rev["patterns"]), str(dict(fwd["patterns"])))
    same = all(fwd["rec"][i][0] == rev["rec"][i][0] and fwd["rec"][i][1:] == rev["rec"][i][2:0:-1] for i in fwd["rec"])
    check("A0.3 reversal audit: per sample, outcome equal and before/after sizes exchanged", same)
    if full:
        Z0, Z1 = E.phases(6000, 0.03); t0 = time.time(); r = E.run(Z0, Z1); p = r["patterns"]
        check("A0.5 v8 regression (6000 samples): 539 changed, 468 valid, 244 same tile set, 169 non-isomorphic, 55 isomorphic",
              r["changed"] == 539 and r["valid"] == 468 and p[(True, True, True)] == 244 and p[(False, False, False)] == 169 and p[(False, True, False)] == 55,
              "%s  (%.0fs)" % (dict(p), time.time() - t0))

    if "--stats" in sys.argv or full:
        import json as _j
        ref = _j.load(open(os.environ.get("V8_STATS", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "toymodel_v8", "spectre_flip_stats_tight_d0.03.json"))))
        E7 = Engine(r2=7.0); t0 = time.time(); st = E7.stats(); 
        check("A0.6 v8 stats regression: usable phases %d, slope %.6f" % (ref["usable"], ref["slope"]), st["usable"] == ref["usable"] and abs(st["slope"] - ref["slope"]) < 1e-9, "got %d, %.6f (%.0fs)" % (st["usable"], st["slope"], time.time() - t0))
        check("A0.6 area-conserving events at rho=7 and P(dG!=0) match v8 (163, 0.319)", st["partA"][7]["n_area_conserving"] == ref["partA"]["7"]["n_area_conserving"] == 163 and abs(st["partA"][7]["P_dG_nonzero_given_area_conserved"] - ref["partA"]["7"]["P_dG_nonzero_given_area_conserved"]) < 1e-12)
        check("A0.6 erasure rows identical to v8 (%d z used)" % len(ref["partB"]), st["partB"] == [tuple(r) for r in ref["partB"]] or [list(r) for r in st["partB"]] == ref["partB"])
    print("\n%d/%d passed" % (sum(results), len(results))); sys.exit(0 if all(results) else 1)

if __name__ == "__main__":
    if any(a in sys.argv for a in ("--tests-only", "--full", "--stats")): main()
    else: print(__doc__)
