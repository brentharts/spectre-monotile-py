"""Long run: window volumes of the selected Danzer dissection at higher levels. usage: python3 t9_patch_long.py LEVEL [MINPTS]; writes patch_long_L<LEVEL>.json"""
import sys, json
sys.path.insert(0, '.')
import t9_patch_windows as W
BEST = {'K': 0, 'B': 1, 'C': 1, 'A': 8}
L = int(sys.argv[1]); mp = int(sys.argv[2]) if len(sys.argv) > 2 else 100
out = []
for k, n, dens, cv, wv, hull in sorted(W.run(BEST, levels=L, minpts=mp), key=lambda r: -r[4]):
    print("class", k, "n=%d density %.5f covol %.2f d*covol %.2f hull %.2f fill %.3f" % (n, dens, cv, wv, hull, wv / hull), flush=True)
    out.append(dict(key=str(k), n=n, density=dens, covol=cv, window_volume=wv, hull=hull, fill=wv / hull))
json.dump(dict(choice=BEST, level=L, classes=out), open("patch_long_L%d.json" % L, "w"), indent=1)
