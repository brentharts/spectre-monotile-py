"""B1c tests: the dissection {K:0,B:1,C:1,A:8} (index into the exact-cover solution lists) grown to level 9/10; run: python3 t9_patch_tests.py"""
import sys, collections
sys.path.insert(0, '.')
import numpy as np
import t9_dissect as d, t9_patch as P, t9_patch_windows as W
BEST = {'K': 0, 'B': 1, 'C': 1, 'A': 8}
res = []
def check(name, ok, info=""): res.append(ok); print(("PASS " if ok else "FAIL ") + name + ("  " + info if info else ""), flush=True)
tau = d.tau
bg = [20 * (4 - tau), 4 * (tau + 2), 20 * (tau - 1)]
tiles = P.grow(BEST, 9); cnt = collections.Counter(t for t, g, x in tiles); tt = sum(cnt.values())
fr = {"K": 0.5189, "B": 0.2829, "C": 0.1604, "A": 0.0379}
check("B1c tile frequencies match the published ones to 1.2e-3", all(abs(cnt[k] / tt - fr[k]) < 1.2e-3 for k in fr), str({k: round(cnt[k] / tt, 4) for k in fr}))
r = W.run(BEST, levels=10, minpts=30)
big = sorted([x for x in r if x[3] > 500], key=lambda x: -x[4])
check("B1c exactly three D6-lattice vertex classes", len(big) == 3, "%d" % len(big))
vols = [x[4] for x in big]
check("B1c window volumes d*covol %s match Baake-Grimm %s within 2%%" % (np.round(vols, 2), np.round(bg, 2)), len(vols) == 3 and all(abs(a - b) / b < 0.02 for a, b in zip(vols, bg)))
two = big[1]
check("B1c class II (vol 4(tau+2)) fills its convex hull: fill %.2f, hull %.2f" % (two[4] / two[5], two[5]), abs(two[4] / two[5] - 1) < 0.08)
check("B1c class I hull %.1f is a dodecahedron-sized hull (>vol %.1f: non-convex window)" % (big[0][5], big[0][4]), big[0][5] > 1.2 * big[0][4])
check("B1c class III hull %.1f exceeds its volume %.1f (star-shaped, non-convex)" % (big[2][5], big[2][4]), big[2][5] > 1.2 * big[2][4])
print("%d/%d passed" % (sum(res), len(res))); sys.exit(0 if all(res) else 1)
