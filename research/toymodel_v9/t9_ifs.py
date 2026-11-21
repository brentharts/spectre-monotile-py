"""Windows of the ABCK tile origins as the attractor of the graph-directed IFS of a dissection choice (stage B1b)."""
import sys, time, json
sys.path.insert(0, '.')
import numpy as np
import t9_dissect as d
import t9_dissect_faces as F
Rint = [np.vectorize(d.conj_entry)(d.G[gi]) for gi in range(120)]
INV = {gi: next(j for j in range(120) if np.allclose(d.G[j] @ d.G[gi], np.eye(3), atol=1e-9)) for gi in range(120)}
def roles(choice):
    R = {k: [] for k in "KBCA"}
    for T in "KBCA":
        for t, g0, p0 in F.raw[T][choice[T]][1]:
            h = INV[g0]; R[t].append((T, Rint[h], d.star4(d.RX[h] @ np.array(p0))))
    return R
def member(R, T, S, depth, Rb, D):
    n = len(S); out = np.zeros(n, bool)
    if n == 0: return out
    if depth == D: return np.linalg.norm(S, axis=1) <= Rb
    for (Tp, Rp, c) in R[T]:
        W = -d.tau * (S - c) @ np.linalg.inv(Rp).T
        idx = np.where((np.linalg.norm(W, axis=1) <= Rb) & ~out)[0]
        if len(idx): out[idx[member(R, Tp, W[idx], depth + 1, Rb, D)]] = True
    return out
def volumes(choice, n=60000, Rb=7.0, D=8, seed=0):
    R = roles(choice); rng = np.random.default_rng(seed)
    S = rng.uniform(-Rb, Rb, (n, 3)); S = S[np.linalg.norm(S, axis=1) <= Rb]; ball = 4 / 3 * np.pi * Rb ** 3
    return {T: float(member(R, T, S, 0, Rb, D).mean() * ball) for T in "KBCA"}
if __name__ == "__main__":
    f = {"K": 0.5189, "B": 0.2829, "C": 0.1604, "A": 0.0379}
    print("fails distribution:", [x[0] for x in F.FAILS][:12])
    for nf, ch in F.FAILS[:6]:
        t0 = time.time(); v = volumes(ch)
        print(nf, ch, {k: round(x, 2) for k, x in v.items()}, "ratios", {k: round(v[k] / v["K"], 3) if v["K"] else None for k in v}, "target", {k: round(f[k] / f["K"], 3) for k in f}, "%.0fs" % (time.time() - t0), flush=True)
