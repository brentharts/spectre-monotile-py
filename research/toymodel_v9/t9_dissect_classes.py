import sys, itertools, json
sys.path.insert(0, '.')
import numpy as np, t9_dissect as d
def tile_vs(name, g, p):
    return frozenset(tuple(int(z) for z in (np.array(p) + d.RX[g] @ d.T4[name][j])) for j in range(4))
for name in "KBCA":
    V4, out, nodes = d.solve(name, cap=100)
    sols = [o for o in out if o[0] == d.MROW[name]]
    sets = [frozenset((t, tile_vs(t, g, p)) for t, g, p in o[1]) for o in sols]
    Vt = [tuple(int(z) for z in v) for v in V4]
    syms = []
    for gi in range(120):
        for perm in itertools.permutations(range(4)):
            t = np.array(Vt[perm[0]]) - d.RX[gi] @ np.array(Vt[0])
            if all(tuple(int(z) for z in (d.RX[gi] @ np.array(Vt[i]) + t)) == Vt[perm[i]] for i in range(4)):
                syms.append((gi, tuple(t)))
    def act(s, sol):
        gi, t = s
        return frozenset((ty, frozenset(tuple(int(z) for z in (d.RX[gi] @ np.array(v) + np.array(t))) for v in vs)) for ty, vs in sol)
    classes = []
    for i, so in enumerate(sets):
        orb = {act(s, so) for s in syms}
        for c in classes:
            if so in c[1]: c[2].append(i); break
        else: classes.append([i, orb, [i]])
    print(name, "symmetries of the target:", len(syms), " solutions with the matrix counts:", len(sets), " classes under those symmetries:", len(classes), [len(c[2]) for c in classes], flush=True)
