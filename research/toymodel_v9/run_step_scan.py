"""Stage A2: step scan of the flip cluster size. Usage: python3 run_step_scan.py DELTA [NSA]   -> step_scan_d{DELTA}.json
Part A of the stats only (no erasure test), r2 = 7, tight tolerance, box 20, as in v8."""
import sys, json, time
from t9_engine import Engine
d = float(sys.argv[1]); n = int(sys.argv[2]) if len(sys.argv) > 2 else 3500
E = Engine(r2=7.0); t0 = time.time(); st = E.stats(NSA=n, delta=d, partB=False); st["seconds"] = time.time() - t0
json.dump(st, open("step_scan_d%s.json" % d, "w"), indent=1)
print(d, n, "usable", st["usable"], "slope %.3f" % st["slope"], {r: (round(v["frac_any"], 4), round(v["mean_changed_given_change"] or 0, 2)) for r, v in st["partA"].items()}, "%.0fs" % st["seconds"])
