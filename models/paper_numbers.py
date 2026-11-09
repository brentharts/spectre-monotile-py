"""paper_numbers.py - every number quoted in quantgrav.tex, computed from the models (or copied from a labelled snapshot) into LaTeX macros.

The paper must not drift from the code.  This script computes the numbers that the text quotes from this repository's own models, writes them
as \\newcommand macros to build/quantgrav_numbers.tex (the paper \\input's that file), and checks each against the value the text and the model
tests assert.  Numbers of the Wilson-loop probe come from `data/wilson_snapshot.json` (copied from the printed output of another repository, with
its commit and date in the file); they are not recomputed here and are flagged as a snapshot in the paper.

Macro names are letters only (LaTeX forbids digits): \\qgInscribed, \\qgPartitions, ...

    python3 models/paper_numbers.py                 compute, check, print the table and write build/quantgrav_numbers.tex (about 45 s)
    python3 models/paper_numbers.py --tests-only    compute and check only
"""
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(_HERE)
for p in (_HERE, _ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

SNAPSHOT = os.path.join(_HERE, "data", "wilson_snapshot.json")
COMPANION = os.path.join(_HERE, "data", "companion_snapshot.json")
OUT = os.path.join(_ROOT, "build", "quantgrav_numbers.tex")


def corner_types(levels=2):
    """(number of tiles, number of distinct vertices, {|turn| in degrees: corners per tile} if the same for all tiles, {(red, green): tiles}).

    A corner is a place where consecutive edges of a tile outline differ; |turn| is the geometric turn in the plane of the tiling."""
    import collections
    import lift4d
    tiles, lifted = lift4d.lift_patch(levels)[:2]
    split = collections.Counter()
    types = set()
    for t in tiles:
        n = len(t)
        pts = [complex(lift4d.project(np.array(v))) for v in t]
        dirs = []
        for i in range(n):
            d = pts[(i + 1) % n] - pts[i]
            dirs.append(int(round(math.degrees(math.atan2(d.imag, d.real)) / 30)) % 12)
        turns = tuple(sorted(collections.Counter(abs(((dirs[i] - dirs[i - 1] + 6) % 12 - 6) * 30) for i in range(n)).items()))
        types.add(turns)
        split[(sum(1 for d in dirs if d % 2 == 0), sum(1 for d in dirs if d % 2 == 1))] += 1
    return len(tiles), len(lifted), (dict(types.pop()) if len(types) == 1 else None), dict(split)


def compute():
    """{macro name: (value as text, how it was obtained)}"""
    import chair_e8_192 as ch
    import e6_fibres as ef
    import e8_shells as es
    import hopf_shells as hs
    import spectre_in_e8 as se
    import voronoi_window as vw
    N = {}

    def put(name, value, src):
        N[name] = (str(value), src)

    put("Inscribed", len(hs.inscribed_24cells()), "hopf_shells.inscribed_24cells()")
    put("Partitions", len(hs.coset_partitions()), "hopf_shells.coset_partitions()")
    R = hs.e8_roots()
    lab, nf = hs.e8_fibres(R)
    put("Roots", len(R), "hopf_shells.e8_roots()")
    put("Fibres", nf, "hopf_shells.e8_fibres()")
    put("FibreSize", int(np.bincount(lab).min()), "hopf_shells.e8_fibres(): smallest fibre (all equal, tested)")
    for k, w in ((1, "One"), (2, "Two"), (3, "Three")):
        assert len(es.e8_shell(k)) == 240 * es.sigma3(k)
        put("Shell" + w, len(es.e8_shell(k)), "e8_shells.e8_shell(%d) = 240 sigma_3(%d)" % (k, k))
    put("ASubsystems", len(ef.a2_subsystems()), "e6_fibres.a2_subsystems()")
    orth, rest = ef.complement(ef.omega_a2())
    put("ESixRoots", len(orth), "e6_fibres.complement(omega_a2())")
    put("Rest", len(rest), "e6_fibres.complement(omega_a2()): the roots in neither A2 nor E6")
    tips = ch.load_tips()
    put("ChairTips", len(tips), "chair_e8_192.load_tips()")
    put("ChairPanels", len({t["panel"] for t in tips}), "chair_e8_192.load_tips()")
    put("ChairClasses", len(ch.panel_classes(tips)), "chair_e8_192.panel_classes()")
    nt, nv, types, split = corner_types(2)
    put("Tiles", nt, "lift4d.lift_patch(2)")
    put("Vertices", nv, "lift4d.lift_patch(2)")
    assert types is not None, "corner types differ between tiles"
    put("CornersSixty", types[60], "corners of a tile outline, turn 60 degrees (same for all tiles)")
    put("CornersNinety", types[90], "corners of a tile outline, turn 90 degrees (same for all tiles)")
    put("CornersStraight", types[0], "straight passes of a tile outline")
    put("TilesMajor", split[(8, 6)], "tiles with 8 red and 6 green edges")
    put("TilesMinor", split[(6, 8)], "tiles with 6 red and 8 green edges")
    irr, lam_ok, roots, contains = se.inflation_poly_facts()
    assert irr and lam_ok and contains == [False, False, False, True]
    put("LambdaSquared", "%.4f" % (4 + math.sqrt(15)), "4 + sqrt 15 (checked symbolically in spectre_in_e8.inflation_poly_facts())")
    inr, circ = vw.radii("icosian")
    h, _ = vw.window_vertices_and_hull("icosian")
    put("VoronoiVertices", len(h.vertices), "voronoi_window.window_vertices_and_hull()")
    put("VoronoiVolume", "%.3f" % h.volume, "voronoi_window.window_vertices_and_hull()")
    put("VoronoiIn", "%.4f" % inr, "voronoi_window.radii()")
    put("VoronoiCirc", "%.4f" % circ, "voronoi_window.radii()")
    put("VoronoiFacets", len(np.unique(np.round(h.equations, 5), axis=0)), "distinct facet hyperplanes of the hull (not the number of triangulated simplices)")
    put("VoronoiCellVertices", len(vw.voronoi_vertices()), "voronoi_window.voronoi_vertices()")
    import fibre_window as fw
    V8, lab8 = fw.lattice(8)
    put("VecEight", len(V8), "fibre_window.lattice(8): shells 1 to 8")
    put("FibreEight", len(np.unique(lab8)), "fibre_window.lattice(8)")
    put("SplitIcosian", fw.split_fibres(8, 1.4, "icosian")[0], "fibre_window.split_fibres(8, 1.4, 'icosian')")
    put("SplitCoxOne", fw.split_fibres(8, 1.4, "coxeter")[0], "fibre_window.split_fibres(8, 1.4, 'coxeter'), R = 1.4")
    put("SplitCoxTwo", fw.split_fibres(8, 2.0, "coxeter")[0], "fibre_window.split_fibres(8, 2.0, 'coxeter'), R = 2.0")
    put("VorAccepted", int(vw.accepted(8, "icosian")[2].sum()), "voronoi_window.accepted(8): vectors inside W")
    put("VorCoxSplit", vw.split_count(8, "coxeter")[0], "voronoi_window.split_count(8, 'coxeter')")
    snap = json.load(open(SNAPSHOT))
    for k, v in snap.items():
        if not k.startswith("_"):
            put("W" + "".join(c for c in k.title() if c.isalpha()), v, "snapshot data/wilson_snapshot.json (not recomputed here)")
    # computed here from closed forms quoted by the dynamic sections (the companion LPs give the same values at the delta checked there)
    put("BitsCopy", "%.3f" % math.log2(3), "log2 3: bits about the measured axis that the copy contract signals")
    put("Tsirelson", "%.3f" % (2 * math.sqrt(2)), "2 sqrt 2")
    put("DeltaCubic", "1/12", "cubic threshold of min(4, 4/3 + 8 delta) = 2 (companion LP); 1/12 exactly")
    assert abs(4 / 3 + 8 / 12 - 2) < 1e-12
    put("DeltaCubicTsirelson", "%.3f" % ((2 * math.sqrt(2) - 4 / 3) / 8), "(2 sqrt 2 - 4/3)/8")
    put("DeltaHexTsirelson", "%.3f" % ((2 * math.sqrt(2) - 2) / 12), "(2 sqrt 2 - 2)/12")
    put("DeltaHexFull", "1/6", "min(4, 2 + 12 delta) = 4 at delta = 1/6")
    put("RhoZero", "%.4f" % ((5 - math.sqrt(15)) / 5), "(5 - sqrt 15)/5, baseline density per spectre (Exact in the spectre repository)")
    put("PisotRatio", "%.3f" % (4 - math.sqrt(15)), "4 - sqrt 15, the Pisot conjugate 1/lambda^2")
    put("MysticFreq", "%.4f" % ((5 - math.sqrt(15)) / 10), "(5 - sqrt 15)/10, frequency of the Mystic tile = rho_0/2 (Exact in the spectre repository)")
    comp = json.load(open(COMPANION))
    for k, v in comp.items():
        if not k.startswith("_"):
            put("C" + "".join(c for c in k.title() if c.isalpha()), v, "snapshot data/companion_snapshot.json (not recomputed here)")
    return N


EXPECTED = {"Inscribed": "25", "Partitions": "10", "Roots": "240", "Fibres": "10", "FibreSize": "24", "ShellOne": "240", "ShellTwo": "2160",
            "ShellThree": "6720", "ASubsystems": "1120", "ESixRoots": "72", "Rest": "162", "ChairTips": "192", "ChairPanels": "24",
            "ChairClasses": "3", "Tiles": "71", "Vertices": "518", "CornersSixty": "7", "CornersNinety": "6", "CornersStraight": "1",
            "TilesMajor": "63", "TilesMinor": "8", "LambdaSquared": "7.8730", "VoronoiVertices": "720", "VoronoiVolume": "3.530",
            "VoronoiIn": "0.9092", "VoronoiCirc": "0.9732",
            "VoronoiFacets": "1200", "VoronoiCellVertices": "19440", "VecEight": "340320", "FibreEight": "14180", "SplitIcosian": "0",
            "BitsCopy": "1.585", "Tsirelson": "2.828", "DeltaCubic": "1/12", "DeltaCubicTsirelson": "0.187", "DeltaHexTsirelson": "0.069",
            "DeltaHexFull": "1/6", "RhoZero": "0.2254", "MysticFreq": "0.1127", "PisotRatio": "0.127", "CChairfraction": "13/24",
            "SplitCoxOne": "7376", "SplitCoxTwo": "13251", "VorAccepted": "6120", "VorCoxSplit": "2540"}


def write(N, path=OUT):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write("%% generated by models/paper_numbers.py; do not edit\n")
        for k, (v, src) in N.items():
            fh.write("\\newcommand{\\qg%s}{%s}%% %s\n" % (k, v, src))


def run_tests(verbose=True):
    ok = True
    N = compute()
    for k, want in EXPECTED.items():
        got = N[k][0]
        good = got == want
        ok &= good
        if verbose:
            print("  %-4s %-16s %s%s" % ("ok" if good else "FAIL", k, got, "" if good else "   (expected %s)" % want))
    return ok, N


def main(argv):
    ok, N = run_tests()
    if not ok:
        print("MISMATCH")
        return 1
    if "--tests-only" not in argv:
        write(N)
        print("wrote %s (%d macros)" % (OUT, len(N)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
