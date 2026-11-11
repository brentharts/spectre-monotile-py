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


def toymodel_numbers():
    """Numbers of the section 'A staged test of the dynamic half' (research/toymodel_v2), recomputed.  {macro: (text, source)}; macro names start with T."""
    tdir = os.path.join(_ROOT, "research", "toymodel_v2")
    if tdir not in sys.path:
        sys.path.insert(0, tdir)
    import tm2_arrow as ta
    import tm2_directed as td
    import tm2_history_sum as th
    import tm2_lattice as tl
    import tm2_rule as tr
    from spectre_hat_backward_snap_model import BackwardSnap
    T = {}

    def put(name, value, src):
        T["T" + name] = (str(value), src)

    # Stage 0
    put("Seams", len(tl.bond_graph(2, 1.0)), "tm2_lattice.bond_graph(2, 1)")
    n, bad = tl.broken_seams(2, 1.0, {t: 0.2 for t in range(71)})
    put("Broken", len(bad), "tm2_lattice.broken_seams, uniform opening, level 2")
    for lv, nm in ((2, "Two"), (3, "Three"), (4, "Four")):
        kinds, pairs = tl.flat_vertex_types(lv)
        inter = tl.patch(lv).n_tiles - sum(c for k, c in kinds.items() if k[1] == ())
        put("PairFrac" + nm, "%.1f" % (100.0 * 2 * len(pairs) / inter), "tm2_lattice.flat_vertex_types(%d): tiles in a flat pair / interior tiles, percent" % lv)
    sv = th.seam_versus_ledger(2)
    put("SeamAll", sv[0], "tm2_history_sum.seam_versus_ledger(2): sign assignments")
    put("SeamClosed", sv[1], "same: assignments that close every seam")
    put("SeamLedger", sv[2], "same: assignments with Q = 0")
    put("HatNormSq", int(round(sorted(dict(tl.hat_physical_edge_norms(2)))[-1])), "tm2_lattice.hat_physical_edge_norms(2): largest E8 norm^2 of a Hat edge read through Z[w]")
    # Stage 1
    iu = th.HistorySum(13, 1.0, 0.0, th.BIG).influence(6)
    put("InflOpenNear", "%.3f" % iu[7], "tm2_history_sum: influence at distance 1, n = 13, no flux constraint")
    put("InflOpenFar", "%.3f" % iu[12], "same, distance 6")
    iu = th.HistorySum(21, 1.0, 0.0, 0).influence(10)
    put("InflClosedThree", "%.3f" % iu[13], "tm2_history_sum: influence at distance 3, n = 21, closed")
    put("InflClosedEight", "%.3f" % iu[18], "same, distance 8")
    s_, c_, tv = th.greedy_statistics()
    put("GreedySuccess", "%.2f" % s_, "tm2_history_sum.greedy_statistics(): success rate")
    put("GreedyCollapse", "%.2f" % c_, "same: collapse rate")
    # Stage 2
    m = BackwardSnap(ticks=12)
    ref = m.reference(10)
    for nm, spec in (("One", None), ("Three", [0, 1, 2]), ("Six", [0, 1, 2, 3, 4, 5])):
        put("Compat" + nm, td.Directed(m, spec).candidates(ref, 0.4), "tm2_directed: compatible histories at delta = 0.4, %s-root axes" % nm.lower())
    sw = td.sweep(200)
    put("RevFwdBad", sw[("random", "fwd", "comp_bad")], "tm2_directed.sweep(200): reversed sign, forward flow")
    put("RevFwdAll", sw[("random", "fwd", "comp_bad")] + sw[("random", "fwd", "comp_ok")], "same: cases needing compensation, forward flow")
    put("RevRevBad", sw[("random", "rev", "comp_bad")], "same: reversed sign, reversed flow")
    put("RevRevAll", sw[("random", "rev", "comp_bad")] + sw[("random", "rev", "comp_ok")], "same: cases needing compensation, reversed flow")
    lad = td.ladder(ta.Records().q.overlap(0.9), 2)
    put("LadderZero", "%.3f" % lad[0], "tm2_directed.ladder from c(tau = 0.9)")
    put("LadderOne", "%.3f" % lad[1], "same, level 1")
    put("LadderTwo", "%.1f\\times10^{-4}" % (lad[2] * 1e4), "same, level 2")
    # Stage 3
    U = ta.core.all_timelines("hex", 5).astype(int)
    put("OpenAsym", int(sum(abs(ta.reverse_action_difference(u, 1.0)) > 1e-12 for u in U)), "tm2_arrow: open 5-step timelines with A(R s) != A(s)")
    put("OpenAll", len(U), "6^5")
    q = ta.agreement_profile(9, 1.0, 0.5, ta.BIG)
    put("BaseAsym", "%.3f" % (q[-1] - q[0]), "tm2_arrow.agreement_profile(9, 1, 0.5, open): q(n-2) - q(0)")
    put("DampPlus", "%.3f" % ta.energy_ratio(0.1), "tm2_arrow.energy_ratio(+0.1)")
    put("DampMinus", "%.2f" % ta.energy_ratio(-0.1), "tm2_arrow.energy_ratio(-0.1)")
    put("MeanFlip", "%.2f" % ta.mean_flip_time(m), "tm2_arrow.mean_flip_time(chain)")
    for nn, nm in ((5, "Five"), (8, "Eight")):
        c = ta.census(nn)
        put("Closed" + nm, c[0], "tm2_arrow.census(%d): closed timelines" % nn)
        put("Oriented" + nm, c[6], "same: oriented (cyclic sense)")
    rec = ta.Records()
    put("HeldSpectre", "%.2f" % rec.held_at(0), "tm2_arrow.Records().held_at(0), bits")
    put("HeldLast", "%.2f" % rec.held_at(11), "same, tick 11")
    put("HeldMin", "%.3f" % min(rec.K(i, 0) for i in range(1, rec.N)), "smallest K(i, 0), i before the Hat")
    put("Ticks", rec.N, "ticks of the default chain")
    # Stage 4
    b2, bt = tr.crossing(2.0), tr.crossing(tr.TSIRELSON)
    put("BetaTwo", "%.3f" % b2, "tm2_rule.crossing(2)")
    put("BetaTsirelson", "%.3f" % bt, "tm2_rule.crossing(2 sqrt 2)")
    put("ContrastTwo", "%.2f" % tr.TwoWing(b2, 1).contrast(), "contrast of the check weight at CHSH = 2")
    put("ContrastTsirelson", "%.2f" % tr.TwoWing(bt, 1).contrast(), "same at 2 sqrt 2")
    put("DeltaTwo", "%.3f" % tr.TwoWing(b2, 1).delta(), "delta at CHSH = 2")
    put("RuleSixty", "%+.3f" % tr.rule_E(0, 2, 0.5), "tm2_rule.rule_E at 60 degrees, kappa = 0.5")
    put("RuleOneTwenty", "%+.3f" % tr.rule_E(0, 4, 0.5), "same at 120 degrees")
    put("RuleChshOne", "%.3f" % tr.chsh(tr.rule_table(1.0)), "CHSH of the rule at kappa = 1, recomputed")
    ctrl = tr.TwoWing(2.0, 2, 2.0, eta=0.3, u=1)
    marg = [ctrl.table(0, b)[0].sum() for b in tr.TWELVE]
    put("BiasLo", "%.3f" % min(marg), "tm2_rule: biased control, smallest marginal")
    put("BiasHi", "%.3f" % max(marg), "same, largest")
    for nn, nm in ((1, "One"), (8, "Eight")):
        put("PastShift" + nm, "%.3f" % tr.TwoWing(2.0, nn).delta(), "tm2_rule: delta at beta = 2, n = %d" % nn)
    ts = np.linspace(0.01, 3.0, 600)
    sw_ = np.array([tr.chsh(tr.postselected_E(tr.walk_unitary(t))) for t in ts])
    put("WalkAbove", "%.0f" % (100.0 * (sw_ > tr.TSIRELSON).mean()), "tm2_rule: percent of walk times in (0, 3) with CHSH > 2 sqrt 2")
    h = tr.check_entropy(m.angles())
    put("CheckEntropy", "%.3f" % h.sum(), "tm2_rule.check_entropy(chain), total bits")
    put("CheckLast", "%.0f" % (100.0 * h[-3:].sum() / h.sum()), "same: percent in the last three ticks")
    # Stage 5
    import tm2_waves as tw
    put("SpeedDense", "%.3f" % tw.group_speed(0.5, 0.2, 2.0), "tm2_waves.group_speed, rho = 4, k = 0.5")
    put("SpeedOne", "%.3f" % tw.group_speed(0.5, 0.2, 1.0), "same, rho = 1")
    put("StepRefl", "%.3f" % tw.step_theory(0.5, 0.2, 1.0, 2.0)[0], "tm2_waves.step_theory: reflected fraction, rho 1 -> 4, k = 0.5")
    fh = tw.front_height()
    rr = sorted(fh)
    put("FrontSlope", "%.2f" % -np.polyfit(np.log(rr), np.log([fh[r] for r in rr]), 1)[0], "tm2_waves.front_height: minus the log-log slope")
    avg, pavg = tw.settled_average(), tw.settled_average(pair=True)
    put("PairNear", "%.2f" % (pavg[32, 34, 32] / avg[32, 34, 32]), "tm2_waves.settled_average: pair / single at r = 2")
    put("PairFar", "%.3f" % (pavg[32, 40, 32] / avg[32, 40, 32]), "same at r = 8")
    put("RuleIErr", "%.0f" % (100.0 * (math.sqrt(tw.c2_rule(2.0, "I")) / tw.clock(2.0) - 1.0)), "tm2_waves: percent by which c exceeds the clock at 2 r_s, rule I")
    return T


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
    put("ShearSV", "%.4f" % math.sqrt(2 - math.sqrt(3)), "sqrt(2 - sqrt 3) = |1 - zeta_12|, both singular values of the fitted shear (spectre repository deform_fit_v1: 0.5176)")
    for k, v in toymodel_numbers().items():
        put(k, v[0], v[1])
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
            "DeltaHexFull": "1/6", "RhoZero": "0.2254", "MysticFreq": "0.1127", "ShearSV": "0.5176", "PisotRatio": "0.127", "CChairfraction": "13/24",
            "SplitCoxOne": "7376", "SplitCoxTwo": "13251", "VorAccepted": "6120", "VorCoxSplit": "2540"}
# research/toymodel_v2 (section 'A staged test of the dynamic half'); recomputed by toymodel_numbers()
EXPECTED.update({'TSeams': '406', 'TBroken': '88', 'TPairFracTwo': '24.0', 'TPairFracThree': '34.9', 'TPairFracFour': '40.0', 'TSeamAll': '4096', 'TSeamClosed': '64', 'TSeamLedger': '924', 'THatNormSq': '6', 'TInflOpenNear': '0.436', 'TInflOpenFar': '0.079', 'TInflClosedThree': '0.069', 'TInflClosedEight': '0.199', 'TGreedySuccess': '0.85', 'TGreedyCollapse': '0.14', 'TCompatOne': '43', 'TCompatThree': '70', 'TCompatSix': '711', 'TRevFwdBad': '18', 'TRevFwdAll': '994', 'TRevRevBad': '145', 'TRevRevAll': '572', 'TLadderZero': '0.872', 'TLadderOne': '0.340', 'TOpenAsym': '6624', 'TOpenAll': '7776', 'TBaseAsym': '0.246', 'TDampPlus': '0.142', 'TDampMinus': '7.45', 'TMeanFlip': '4.85', 'TClosedFive': '360', 'TOrientedFive': '240', 'TClosedEight': '54810', 'TOrientedEight': '43488', 'THeldSpectre': '10.67', 'THeldLast': '1.24', 'THeldMin': '0.777', 'TTicks': '12', 'TBetaTwo': '0.841', 'TBetaTsirelson': '1.255', 'TContrastTwo': '1.21', 'TContrastTsirelson': '1.68', 'TDeltaTwo': '0.169', 'TRuleSixty': '+0.152', 'TRuleOneTwenty': '+0.689', 'TRuleChshOne': '3.620', 'TBiasLo': '0.535', 'TBiasHi': '0.571', 'TPastShiftOne': '0.315', 'TPastShiftEight': '0.030', 'TWalkAbove': '36', 'TCheckEntropy': '0.216', 'TCheckLast': '95'})
EXPECTED.update({'TSpeedDense': '1.947', 'TSpeedOne': '0.970', 'TStepRefl': '0.118', 'TFrontSlope': '1.09', 'TPairNear': '0.16', 'TPairFar': '0.006', 'TRuleIErr': '10'})


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
