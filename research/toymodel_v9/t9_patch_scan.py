"""Scan all dissection combinations: fill ratio d*covol/hull of each D6-lattice vertex class (1 = the cloud fills its convex hull)."""
import sys, json, time
sys.path.insert(0, '.')
import t9_patch_windows as W, t9_dissect_faces as F
res = []; t0 = time.time()
for nf, ch in F.FAILS:
    r = W.run(ch, levels=9, minpts=30)
    big = [(k, n, wv / hull, wv, hull) for k, n, dens, cv, wv, hull in r if cv > 500]
    res.append(dict(fails=nf, choice=ch, classes=[dict(key=str(k), n=n, fill=f, dcov=wv, hull=h) for k, n, f, wv, h in big]))
    print(nf, ch, ["%.2f" % f for k, n, f, wv, h in sorted(big, key=lambda x: -x[2])], "total d*covol(D6 classes) %.1f" % sum(wv for k, n, f, wv, h in big), flush=True)
json.dump(res, open("patch_scan.json", "w"), indent=1)
print("%.0fs" % (time.time() - t0))
