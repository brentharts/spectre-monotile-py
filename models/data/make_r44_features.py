"""Rebuild r44_features.csv: the 192 pyramid features of Tsiokos' Chair44 solid.

    python3 make_r44_features.py PATH/TO/six-birds-tiles/solid  [out.csv]

Reads r44_solid.json and native_panels.csv from the upstream `solid/` folder (Ioannis Tsiokos,
https://github.com/ioannist/six-birds-tiles, CC BY 4.0) and writes one row per feature:

    role         0..191, = 8 * panel + mark
    panel        0..23, the exposed unit square carrying the feature
    normal_axis  0, 1 or 2: the axis normal to the panel
    outward_sign +1 or -1: which way the panel faces
    coefficient  signed height in units of 1/10000 (+ bump out of the solid, - dent of equal depth)
    apex_x, apex_y, apex_z   exact tip of the pyramid, as fractions
    panel_cx, panel_cy, panel_cz   exact centre of the panel, as fractions
"""
import csv
import json
import os
import sys
from fractions import Fraction


def main(solid_dir, out):
    with open(os.path.join(solid_dir, "r44_solid.json")) as f:
        patches = json.load(f)["patches"]
    with open(os.path.join(solid_dir, "native_panels.csv")) as f:
        panels = list(csv.DictReader(f))
    rows = []
    for p in sorted(patches, key=lambda p: p["role"]):
        pan = panels[p["face"]]
        assert p["role"] // 8 == p["face"], "role must be 8 * panel + mark"
        rows.append(
            [p["role"], p["face"], pan["normal_axis"], pan["outward_sign"], p["coefficient"]]
            + [str(Fraction(c)) for c in p["apex"]]
            + [str(Fraction(pan[k])) for k in ("center_x", "center_y", "center_z")]
        )
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["role", "panel", "normal_axis", "outward_sign", "coefficient",
                    "apex_x", "apex_y", "apex_z", "panel_cx", "panel_cy", "panel_cz"])
        w.writerows(rows)
    print("wrote %d features to %s" % (len(rows), out))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "r44_features.csv"))
