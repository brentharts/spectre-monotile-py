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


def toymodel_v3_numbers():
    """Numbers of the section 'A second pass' (research/toymodel_v3), recomputed.  Macro names start with V."""
    for sub in ("toymodel_v2", "toymodel_v3"):
        d = os.path.join(_ROOT, "research", sub)
        if d not in sys.path:
            sys.path.insert(0, d)
    import core
    import t3_area as va
    import t3_chair_metric as vm
    import t3_chair_handedness as vc
    import t3_escape as ve
    import t3_optics as vo
    import t3_passrate as vp
    import t3_qubit as vq
    import t3_records as vr
    import t3_relax as vx
    V = {}

    def put(name, value, src):
        V["V" + name] = (str(value), src)

    put("ClockOnly", "%.2f" % vo.coefficients(vo.index_from(0.0))[0], "t3_optics: first-order deflection coefficient, clock only")
    put("FullBend", "%.2f" % vo.coefficients(vo.index_from(1.0))[0], "same, A B = 1")
    lag, pred, exc = vo.slab_delay()
    put("SlabLag", "%.1f" % lag, "t3_optics.slab_delay: lag of a pulse through a slab n = 1.5, width 200")
    reff = ve.fit_reff(3, 40, 3.0, 4.0, 20.0)
    put("Reff", "%.2f" % reff, "t3_escape.fit_reff: effective radius of the source on Z^3")
    put("SpecHitOne", "%.2f" % ve.spectre_ring(4, 3.0, 15, 9.0), "t3_escape.spectre_ring(4, 3, L = 15, r = 9)")
    put("SpecHitTwo", "%.2f" % ve.spectre_ring(4, 3.0, 60, 9.0), "same, L = 60")
    radii = (5, 7, 9, 11, 14, 18)
    h = ve.chair44_rings(5, 3.0, 28, radii)
    A = [v / (1.0 / r - 1.0 / 28.0) for r, v in zip(radii, h)]
    put("ChairA", "%.2f" % np.mean(A), "t3_escape.chair44_rings: mean of h (1/r - 1/L)^-1, r = 5..18")
    put("ChairFlat", "%.0f" % (100.0 * (max(A) / min(A) - 1.0)), "same: spread max/min - 1, percent")
    c = vm.by_class(vm.hop_metric(6))
    put("HopAxis", "%.2f" % np.mean(c["axis"]), "t3_chair_metric: hop count / r on the cube axes, level 6 bulk, r = 16..26")
    put("HopFace", "%.2f" % np.mean(c["face"]), "t3_chair_metric: hop count / r on the face diagonals")
    put("HopBody", "%.2f" % np.mean(c["body"]), "t3_chair_metric: hop count / r on the body diagonals")
    put("HopSpread", "%.2f" % vm.spread([np.mean(v) for v in c.values()]), "t3_chair_metric: spread (max - min) / mean of the hop count over the three classes")
    S, M = vm.relaxed_tensor(6, 30.0)
    ev = np.linalg.eigvalsh(S)
    m_, e_ = ("%.0e" % ((ev.max() - ev.min()) / ev.mean())).split("e")
    put("TensorSpread", "%s\\times10^{%d}" % (m_, int(e_)), "t3_chair_metric: relative eigenvalue spread of the relaxed walk tensor, level 6 bulk")
    put("RelaxDrop", "%.0f" % (100 * (1 - ev.mean() / M[0, 0])), "t3_chair_metric: percent by which relaxation lowers the walk tensor")
    en = np.linalg.eigvalsh(vm.relaxed_tensor(5, at_notch=True)[0])
    put("NotchRatio", "%.0f" % (en[-1] / en[0]), "t3_chair_metric: eigenvalue ratio of the same tensor measured at the notch (artefact)")
    ex = vm.body_excess(6)
    put("BodyNear", "%.1f" % (100 * ex[0][1]), "t3_chair_metric: percent excess of h on body diagonals over axes, r = 6..10")
    put("BodyFar", "%.1f" % (100 * abs(ex[-1][1])), "t3_chair_metric: same, r = 15..20, absolute")
    for n, nm in ((6, "Six"), (8, "Eight")):
        T = core.closed_timelines("hex", n).astype(int)
        put("Area" + nm, int((va.K_of(T) != 0).sum()), "t3_area: closed timelines of %d steps with nonzero signed area" % n)
    put("Psi", "-3/14" if vc.psi() == vc.Fr(-3, 14) else str(vc.psi()), "t3_chair_handedness.psi(): handedness of the decorated chair")
    put("PinSignChsh", "%.2f" % vp.Box(5.0, 1, "pin", "sign").chsh(), "t3_passrate: post-selected CHSH, pin/sign, beta = 5, constant pass rate")
    worst = max(vp.Box(b, n, ma, mb).ch_max() for ma, mb in (("pin", "pin"), ("pin", "sign"), ("soft", "sign"), ("soft", "soft")) for b in (1.0, 2.0, 3.0, 5.0) for n in (1, 3))
    put("ChWorst", "%.4f" % worst, "t3_passrate: largest Clauser-Horne combination found (local bound 0)")
    r = vr.RecordedChain()
    put("RecBoth", "%.2f" % r.asymmetry(6, 1), "t3_records: asymmetry at k = 6, m = 1, amplitudes and records")
    put("RecFlat", "%.2f" % vr.RecordedChain(flat=True).asymmetry(6, 1), "same, flat amplitudes, records only")
    put("RecAmp", "%.2f" % vr.amplitude_only("chain", 6, 1), "same, amplitudes only")
    rng = np.random.default_rng(11)
    fd = []
    for _ in range(200):
        U = vq.random_unitary(rng)
        rng.normal(size=6)  # keep the draw order of the test irrelevant: a separate stream
        fd.append(vq.fidelity(vq.spin_state(vq.winding(-1), U, vq.random_unitary(rng))))
    put("DiffWing", "%.2f" % np.mean(fd), "t3_qubit: mean singlet fidelity with two different random wings")
    lo, hi = vq.flux_branch_range()
    put("FluxLo", "%.3f" % lo, "t3_qubit: least singlet fidelity of the flux amplitude over the 64 spinor branches")
    put("FluxHi", "%.2f" % hi, "t3_qubit: greatest singlet fidelity of the flux amplitude over the 64 spinor branches")
    free = vx.profile(13, False, 8000, vx.TIMES, seed=5)
    put("FreeOne", "%.0f" % free[0][2], "t3_relax: t_half at d = 1, free, n = 13")
    put("FreeFive", "%.0f" % free[4][2], "same, d = 5")
    far = {}
    for n in (13, 17):
        rows = vx.profile(n, True, 8000, vx.TIMES, seed=6)
        sel = [x[2] for x in rows if x[0] >= n // 2 - 2 and x[1] >= 0.04]
        far[n] = float(np.mean(sel))
        put("Far" + ("Thirteen" if n == 13 else "Seventeen"), "%.0f" % far[n], "t3_relax: mean t_half of the far nodes, closed, n = %d" % n)
    put("FarRatio", "%.1f" % (far[17] / far[13]), "ratio of the two")
    return V


def toymodel_numbers():
    """Numbers of the section 'A staged test of the dynamic half' (research/toymodel_v2), recomputed.  {macro: (text, source)}; macro names start with T."""
    tdir = os.path.join(_ROOT, "research", "toymodel_v2")
    if tdir not in sys.path:
        sys.path.insert(0, tdir)
    import tm2_arrow as ta
    import tm2_checking as tk
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
    # Stage 6 (tm2_checking)
    pr, ct, co = tk.rates(2.0)
    put("RatePast", "%.3f" % pr, "tm2_checking.rates(2): past shift, per wing step")
    put("RateContrast", "%.3f" % ct, "same: pass-rate contrast")
    put("RateCorr", "%.3f" % co, "same: end-to-end correlation")
    for nn, nm in ((1, "One"), (4, "Four"), (8, "Eight")):
        put("PastBits" + nm, "%.3f" % tk.bits_on_past(2.0, nn), "tm2_checking.bits_on_past, beta = 2, n = %d, a = b" % nn)
    lw = tr.TwoWing(3.0, 16)
    put("LongDelta", "%.3f" % lw.delta(), "tm2_rule.TwoWing(3, 16).delta()")
    put("LongE", "%.3f" % abs(lw.E(0, 0)), "same: |E| at zero angle")
    put("LongChsh", "%.3f" % lw.chsh(), "same: CHSH")
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


def toymodel_v4_numbers():
    """Numbers of the section on the flow of the 14th vertex (research/toymodel_v4 and spectre_hat_time_3d), recomputed.  {macro: (text, source)}; macro names start with W."""
    import itertools
    sys.path.insert(0, _ROOT)
    vdir = os.path.join(_ROOT, "research", "toymodel_v4")
    if vdir not in sys.path:
        sys.path.insert(0, vdir)
    import spectre_hat_time_3d as t3
    import t4_arrow as ta4
    import t4_patch as tp
    import t4_spin as ts
    from spectre_hat_time_model import SpectreHatTime
    W = {}

    def put(name, value, src):
        W["W" + name] = (str(value), src)

    m = t3.run_story()
    put("TauStar", "%.3f" % m.tau_star(), "spectre_hat_time_model.tau_star(), omega_0 = 120, snap scale 0.5 (the 2D figure)")
    put("JumpZero", "%.3f" % m.jump(0.0), "jump d(0) = 2 a tan(omega_0 / 2), a = 1, omega_0 = 120")
    put("JumpHalf", "%.3f" % m.jump(0.5), "d(1/2) at the same parameters")
    put("ReadHalf", "%.3f" % (m.jump(0.5) / m.jump(0.0)), "readability d(1/2)/d(0)")
    for k0, nm in ((0.3, "Three"), (0.5, "Five"), (0.8, "Eight")):
        oc, fb = ts.omega_c(k0)
        put("OmegaC" + nm, "%.1f" % oc, "t4_spin.omega_c(%.1f): largest omega_0 (degrees) with a simple s = -1 outline at tau = 0" % k0)
        put("FirstBad" + nm, fb, "same: first failing whole degree")
    # a flow at omega_0 = 160: where the reflex value becomes valid
    lo, hi = 0.0, 1.0
    from spectre_hat_time_model import is_simple_tile
    ok = lambda t: is_simple_tile(SpectreHatTime(corner_deg=160.0, k0=0.5).tile(t, -1))
    assert not ok(0.0) and ok(0.99)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    put("ForbidEnd", "%.3f" % hi, "end of the tau interval in which s = -1 is not a simple curve, omega_0 = 160, k0 = 0.5 (bisection)")
    # patch of real outlines
    for lv, nm in ((2, "Two"), (3, "Three"), (4, "Four")):
        P = tp.patch(lv, 1.0)
        pr = tp.flat_pairs(lv, 1.0)
        put("Pairs" + nm, len(pr), "t4_patch.flat_pairs(%d, 1): flat pairs" % lv)
        put("PatchTiles" + nm, P.n_tiles, "t4_patch.patch(%d, 1).n_tiles" % lv)
        put("PairPct" + nm, "%.1f" % (200.0 * len(pr) / P.n_tiles), "tiles in a flat pair / all tiles, percent")
        put("FreeTiles" + nm, len(tp.free_flat_tiles(lv, 1.0)), "t4_patch.free_flat_tiles(%d, 1): boundary tiles with nothing at vertex 8" % lv)
    # validity of s = -1 on the (b, k, omega) grid
    Vm, oms = ta4._validity_grid(-1)
    viol = total = 0
    where = set()
    for i, j, l in itertools.product(range(5), range(5), range(11)):
        for di, dj, dl in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            i2, j2, l2 = i + di, j + dj, l + dl
            if i2 < 5 and j2 < 5 and l2 < 11:
                total += 1
                if Vm[i, j, l] and not Vm[i2, j2, l2]:
                    viol += 1
                    where.add(float(oms[l]))
    put("ValidViol", viol, "t4_arrow: steps toward the Hat (b up, k down, omega down) that lose validity of s = -1, grid 5 x 5 x 11")
    put("ValidSteps", total, "same: number of such steps examined")
    assert len(where) == 1
    put("ValidBand", "%.0f" % where.pop(), "same: the one grid value of omega (degrees) at which those steps lose validity")
    put("ValidPoints", Vm.size, "same: grid points")
    # spin structure
    tile = SpectreHatTime(corner_deg=30.0).tile(0.0, 1)
    put("SignPatterns", len({ts.sign_pattern(tile, e0) for e0 in range(ts.N_EDGES)}), "t4_spin.sign_pattern: distinct patterns over the 14 base edges")
    put("SpinEdges", ts.N_EDGES, "edges of the outline")
    # open questions (t4_open.py) and the coupling of pairs (t4_couple.py)
    import t4_couple as tc4
    import t4_open as to4
    cls = to4.grid_classes()
    put("GridCells", len(cls), "t4_open: (omega_0, k_0) grid, 17 x 5 cells")
    put("GridMono", sum(1 for v in cls.values() if v[0]), "same: cells where the counting entropy is non-increasing along the linear flow")
    put("GridMonoRes", sum(1 for v in cls.values() if v[1]), "same: cells where the resolved entropy is non-increasing")
    put("GridValidStart", sum(1 for v in cls.values() if v[2]), "same: cells where the reflex outline is simple at tau = 0")
    sc, sr = to4.entropies(*to4.linear_path(30.0, 0.5))
    put("ResBeforeHat", "%.0f" % (100.0 * sr[-2] / sr[0]), "t4_open: resolved entropy at tau = 0.99 as a percent of its value at tau = 0, omega_0 = 30, k_0 = 0.5")
    hs = np.linspace(0.3, 6.0, 20)
    best = max(to4.sup_delta(bb, kk, (-60.0, -45.0, -30.0, 0.0, 30.0, 60.0), hs) for bb in (1.0, math.sqrt(3.0)) for kk in (0.0, 0.3))
    put("DeltaSup", "%.0f" % math.degrees(best), "t4_open.sup_delta: largest corner-turn difference (degrees) over valid displacements of P_8, coarse scan")
    put("DeltaNormal", "%.0f" % (2.0 * ts.omega_c(0.5)[0]), "2 omega_c at k_0 = 0.5: the largest difference for the chord-normal displacement")
    import t4_proofs as tpf
    put("Bx", "%.3f" % tpf.B_X, "t4_proofs.B_X = (1 + 2 sqrt 7) / (3 sqrt 3)")
    put("OmegaCStraight", "%.1f" % (2.0 * math.degrees(math.atan(tpf.h_star(1.0)))), "t4_proofs.h_star(1): omega_c of the straight Spectre")
    put("OmegaCHat", "%.0f" % (2.0 * math.degrees(math.atan(tpf.h_star(math.sqrt(3.0))))), "t4_proofs.h_star(sqrt 3): omega_c of the straight tile at the Hat")
    est, fr = tp.pair_density_limit()
    put("PairLimit", "%.4f" % est, "t4_patch.pair_density_limit(): extrapolated density of flat pairs per tile from levels 4 and 5")
    put("PairFive", "%.4f" % fr[-1], "t4_patch: flat pairs per tile at level 5")
    put("BoundaryRatio", "%.3f" % ((2.0 + math.sqrt(5.0)) / (4.0 + math.sqrt(15.0))), "nu / lambda^2 = phi^3 / (4 + sqrt 15)")
    put("PairSix", tp.count_pairs_fast(6)[1], "t4_patch.count_pairs_fast(6): flat pairs of the level-6 patch")
    put("PairSeven", int(round(tp.pair_closed_form(7))), "closed form of t4_patch.pair_closed_form(7); counted directly by t4_pairs_level7.py (2147679 tiles)")
    put("ClusterSize", tc4.N_CLUSTER, "t4_couple: pairs in the exactly solved cluster")
    put("CoupledTiles", len({t for pr in tp.flat_pairs(3, 1.0) for t in pr}), "t4_patch: tiles in flat pairs, level 3")
    rng = np.random.default_rng(1)
    pairs = tc4.pair_data(1.0)[0]
    for kern, nm in (("log", "Log"), ("inv", "Inv"), ("inv2", "InvTwo")):
        tot = 0
        for _ in range(8):
            sub = list(rng.choice(len(pairs), 12, replace=False))
            tot += tc4.changes(tc4.sequence(kern, np.linspace(1.0, math.sqrt(3.0), 15), sub))
        put("Changes" + nm, tot, "t4_couple: ground-state changes along b, 8 random 12-pair subsets, kernel %s" % kern)
    return W


def toymodel_v5_numbers():
    """Numbers of the section 'A third pass: several tiles and the Chair44' (research/toymodel_v5), recomputed.  {macro: (text, source)}; macro names start with X."""
    for d in (os.path.join(_ROOT, "research", "toymodel_v5"), os.path.join(_ROOT, "research", "toymodel_v2")):
        if d not in sys.path:
            sys.path.insert(0, d)
    import t5_alphabet as xa
    import t5_arrow as xw
    import t5_clock as xc
    import t5_ensemble as xe
    import t5_rule as xr
    import t5_source as xs
    X = {}

    def put(name, value, src):
        X["X" + name] = (str(value), src)

    put("AnisoHalf", "%.3f" % xa.anisotropy(0.5), "t5_alphabet.anisotropy(1/2): ratio of the eigenvalues of the frame operator")
    hexk = math.sqrt(3) / 2
    for nm, k in (("Zero", 0.0), ("Half", 0.5), ("Hex", hexk), ("One", 1.0)):
        put("Chsh" + nm, "%.3f" % xr.chsh_of(xr.alphabet(k), 1.255), "t5_rule.chsh_of(alphabet(k), 1.255), k = %.3f" % k)
    put("BetaTwoHalf", "%.3f" % xr.crossing(xr.alphabet(0.5), 2.0), "t5_rule.crossing: stiffness at which CHSH passes 2, k = 1/2")
    put("WindowEnd", "%.3f" % xr.window_end(1.255, hexk), "t5_rule.window_end(1.255, sqrt3/2): flow time at which CHSH falls to 2")
    put("WindowEndTwo", "%.3f" % xr.window_end(2.0, hexk), "t5_rule.window_end(2, sqrt3/2)")
    put("CubicTwo", "%.3f" % xr.chsh_of(xr.cubic(), 2.0), "t5_rule.chsh_of(cubic(), 2): the cubic six")
    put("PlanarEight", "%.4f" % xr.chsh_of(xr.alphabet(1.0), 8.0), "t5_rule.chsh_of(alphabet(1), 8)")
    m0, m1, Q = xs.moments()
    tr, Qt = xs.traceless(Q)
    ev = np.linalg.eigvalsh(Qt)
    put("Monopole", m0, "t5_source.moments(): sum of the 192 signed depths")
    put("TraceQ", "%d/%d" % (tr.numerator, tr.denominator), "t5_source: trace of the second moment (exact)")
    put("QLow", "%.1f" % ev[0], "t5_source: smallest eigenvalue of the traceless quadrupole")
    put("QMid", "%.2f" % ev[1], "t5_source: middle eigenvalue")
    put("QHigh", "%.1f" % ev[2], "t5_source: largest eigenvalue")
    n = np.ones(3) / math.sqrt(3)
    ax = 1.5 * float(n @ Qt @ n) * (np.outer(n, n) - np.eye(3) / 3)
    put("AxialPct", "%.1f" % (100 * np.linalg.norm(ax) / np.linalg.norm(Qt)), "t5_source: norm of the part of Qt axial about (1,1,1), percent of |Qt|")
    put("ChairSlope", "%.2f" % xc.slope(np.geomspace(20, 200, 10), xc.rms_profile(*xc.sources_chair_units(), np.geomspace(20, 200, 10))), "t5_clock: log-log slope of rms |phi| of the chair, r = 20..200")
    mono = xe.monopoles([0.0] * 3 + [1.0] * 3 + [0.0] * 4)
    put("MinMonopole", "%.3f" % np.abs(mono).min(), "t5_ensemble.monopoles: least |net bit source| over the 1024 patterns, all live bits at tau = 0")
    put("MinPatterns", int((np.abs(np.abs(mono) - math.sqrt(3)) < 1e-9).sum()), "t5_ensemble: patterns that reach it")
    Qc, ov = xe.overlaps()
    cmax = max(abs(c) for _, c in ov)
    put("Overlap", "%.3f" % cmax, "t5_ensemble.overlaps(): largest |cos| between the group-bit quadrupole and the chair's")
    put("Residual", "%.3f" % math.sqrt(1 - cmax ** 2), "sqrt(1 - cos^2): part of the chair's quadrupole the four groups cannot cancel")
    put("LiveDipole", "%.1f" % xw.live_dipole(True), "t5_arrow.live_dipole(): dipole of the three live bits, Z = 8")
    P, q = xc.sources_chair_units(xc.PATTERN_B)
    D = xc.directions(200)
    for nm, r in (("Three", 3), ("Ten", 10), ("Forty", 40)):
        c = float(np.mean(np.sqrt(np.exp(-np.abs(xc.phi_at(P, q, r * D))))))
        put("ClockErr" + nm, ("%.2f" if r > 20 else "%.1f") % (100 * abs(c / math.sqrt(1 - xc.M_B / r) - 1)), "t5_clock: |c / clock - 1| in percent at r = %d chair units, pattern B" % r)
    return X


def toymodel_v6_numbers():
    """Numbers of the section 'A Hamiltonian for the bits' (research/toymodel_v6), recomputed.  {macro: (text, source)}; macro names start with Y."""
    for d in (os.path.join(_ROOT, "research", "toymodel_v6"), os.path.join(_ROOT, "research", "toymodel_v4")):
        if d not in sys.path:
            sys.path.insert(0, d)
    import t6_pair as y0
    import t6_patch as y1
    Y = {}

    def put(name, value, src):
        Y["Y" + name] = (str(value), src)

    f0 = y0.flow(0.0)
    put("TwoC", "%.3f" % f0[2], "t6_pair.flow(0): concurrence of two pairs at the Spectre end")
    put("TwoChsh", "%.3f" % f0[3], "t6_pair.flow(0): CHSH of the two-pair ground state")
    put("TwoGap", "%.3f" % f0[5], "t6_pair.flow(0): gap")
    put("TwoEnd", "%.3f" % y0.tau_two(), "t6_pair.tau_two(): flow time at which the two-pair CHSH with visibility falls to 2")
    W, mid = y1.coupling(4, 1.0)
    iu = np.triu_indices(W.shape[0], 1)
    d = np.linalg.norm(mid[:, None] - mid[None], axis=2)[iu]
    far = d > 15
    put("Slope", "%.2f" % np.polyfit(np.log(d[far]), np.log(np.abs(W[iu][far])), 1)[0], "t6_patch.coupling(4, 1): slope of |W| against distance beyond 15")
    put("AntiFour", "%.2f" % np.mean(W[iu] > 0), "t6_patch: fraction of antiferromagnetic couplings at level 4")
    f = y1.test_e68_the_patch_is_frustrated()
    put("FrusTwo", "%.3f" % f[0], "t6_patch.classical_ground: unsatisfied bond weight, 6 pairs of level 2")
    put("FrusLo", "%.3f" % f[1], "t6_patch: same, central 6 pairs of level 3")
    put("FrusHi", "%.3f" % f[-1], "t6_patch: same, central 14 pairs of level 3")
    peaks, best, best_obs = y1.test_e610_along_the_flow()
    cs = [c for c, _ in peaks]
    ts = [t for _, t in peaks]
    put("PeakLo", "%.2f" % min(cs), "t6_patch.scan: smallest of the peak pairwise concurrences over the clusters of 6 to 14 pairs")
    put("PeakHi", "%.2f" % max(cs), "t6_patch.scan: largest")
    put("PeakTauLo", "%.2f" % min(ts), "t6_patch.scan: earliest flow time of a peak")
    put("PeakTauHi", "%.2f" % max(ts), "t6_patch.scan: latest")
    put("ChshMax", "%.2f" % best[0], "t6_patch.scan: largest CHSH of a reduced two-pair state")
    put("ChshObs", "%.2f" % best_obs[0], "t6_patch.scan: same with the read-out visibility")
    import collections
    import t6_gauge as y3
    c = y3.complex_(2)
    put("GTiles", c["T"], "t6_gauge.complex_(2): tiles")
    put("GLinks", c["L"], "t6_gauge.complex_(2): links (shared edges)")
    put("GPlaq", c["P"], "t6_gauge.complex_(2): plaquettes (interior vertices)")
    put("GTwo", collections.Counter(len(c["inc"][v]) for v in c["plaq"])[2], "t6_gauge: plaquettes with two links, level 2")
    c3 = y3.complex_(3)
    put("GPairsThree", len(y3.tp.flat_pairs(3, 1.0)), "t4_patch.flat_pairs(3): every P8 is a two-link plaquette (t6_gauge E6.19)")
    st = y3.two_body_stabilizers(c)
    put("GZZ", len(st["Z"][1]), "t6_gauge.two_body_stabilizers: two-link ZZ stabilizers of the plaquette limit")
    put("GXX", len(st["X"][1]), "t6_gauge.two_body_stabilizers: two-link XX stabilizers")
    _, n_left, S = y3.test_e621_the_plaquette_limit_has_no_two_link_bell_pair()
    put("GCutLinks", n_left, "t6_gauge: links left of the centre line")
    put("GCutBits", S, "t6_gauge.entropy_of_region: entanglement entropy of those links (bits)")
    return Y


def toymodel_v7_numbers():
    """Numbers of the subsection 'Renormalisation by the substitution' (research/toymodel_v7/t7_rg.py), recomputed.  {macro: (text, source)}; macro names start with Z."""
    import collections
    d = os.path.join(_ROOT, "research", "toymodel_v7")
    if d not in sys.path:
        sys.path.insert(0, d)
    import numpy as np
    import t7_rg as z
    Z = {}

    def put(name, value, src):
        Z["Z" + name] = (str(value), src)

    for L, nm in ((2, "Two"), (3, "Three"), (4, "Four")):
        sup = [p[2][:-1] for p in z.build_paths(L)]
        pr = z.tp.flat_pairs(L, 1.0)
        put("Pairs" + nm, len(pr), "t4_patch.flat_pairs(%d)" % L)
        put("Inside" + nm, sum(sup[a] == sup[b] for a, b in pr), "t7_rg: pairs with both tiles in one level-1 supertile, level %d" % L)
    for L, nm in ((3, "Three"), (4, "Four")):
        k = collections.Counter(c[0] for c in z.classify(L))
        put("New" + nm, k["new"], "t7_rg.classify(%d): new pairs" % L)
        put("Inh" + nm, k["inherited"], "t7_rg.classify(%d): inherited pairs" % L)
    t3, _ = z.rule_table(3)
    t4, _ = z.rule_table(4)
    put("TypesThree", len(t3), "t7_rg.rule_table(3): boundary types")
    put("TypesFour", len(t4), "t7_rg.rule_table(4): boundary types")
    put("TypesCommon", len(set(t3) & set(t4)), "t7_rg: types seen at both levels (all agree)")
    put("TypesNew", len(set(t4) - set(t3)), "t7_rg: types first seen at level 4")
    rat, _, _, s = z.coupling_ratios(4)
    q = np.percentile(rat, [25, 50, 75])
    put("Scale", "%.3f" % s, "t7_rg: similarity scale of the inherited pair midpoints, level 3 to 4")
    put("Lambda", "%.3f" % z.LAMBDA, "sqrt(4 + sqrt 15)")
    put("RatioMed", "%.3f" % q[1], "t7_rg.coupling_ratios(4): median of coupling / (lambda^-3 x transformed parents), distance > 60")
    put("RatioLo", "%.3f" % q[0], "same, lower quartile")
    put("RatioHi", "%.3f" % q[2], "same, upper quartile")
    put("LambdaMinusThree", "%.3f" % z.LAMBDA ** -3, "lambda^-3")
    return Z


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
    for k, v in toymodel_v3_numbers().items():
        put(k, v[0], v[1])
    for k, v in toymodel_v4_numbers().items():
        put(k, v[0], v[1])
    for k, v in toymodel_v5_numbers().items():
        put(k, v[0], v[1])
    for k, v in toymodel_v6_numbers().items():
        put(k, v[0], v[1])
    for k, v in toymodel_v7_numbers().items():
        put(k, v[0], v[1])
    put("TNumbersChecked", len(EXPECTED), "number of quoted values checked by run_tests (len(EXPECTED))")
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
EXPECTED.update({'TSeams': '406', 'TBroken': '88', 'TPairFracTwo': '24.0', 'TPairFracThree': '34.9', 'TPairFracFour': '40.0', 'TSeamAll': '4096', 'TSeamClosed': '64', 'TSeamLedger': '924', 'THatNormSq': '6', 'TInflOpenNear': '0.436', 'TInflOpenFar': '0.079', 'TInflClosedThree': '0.069', 'TInflClosedEight': '0.199', 'TGreedySuccess': '0.85', 'TGreedyCollapse': '0.14', 'TCompatOne': '43', 'TCompatThree': '70', 'TCompatSix': '711', 'TRevFwdBad': '18', 'TRevFwdAll': '994', 'TRevRevBad': '145', 'TRevRevAll': '572', 'TLadderZero': '0.872', 'TLadderOne': '0.340', 'TOpenAsym': '6624', 'TOpenAll': '7776', 'TBaseAsym': '0.246', 'TDampPlus': '0.142', 'TDampMinus': '7.45', 'TMeanFlip': '4.85', 'TClosedFive': '360', 'TOrientedFive': '240', 'TClosedEight': '54810', 'TOrientedEight': '43488', 'THeldSpectre': '10.67', 'THeldLast': '1.24', 'THeldMin': '0.777', 'TTicks': '12', 'TBetaTwo': '0.841', 'TBetaTsirelson': '1.255', 'TContrastTwo': '1.21', 'TContrastTsirelson': '1.68', 'TDeltaTwo': '0.169', 'TRuleSixty': '+0.152', 'TRuleOneTwenty': '+0.689', 'TRuleChshOne': '3.620', 'TBiasLo': '0.535', 'TBiasHi': '0.571', 'TPastShiftOne': '0.315', 'TPastShiftEight': '0.030', 'TWalkAbove': '36', 'TCheckEntropy': '0.216', 'TCheckLast': '95', 'TRatePast': '0.392', 'TRateContrast': '0.783', 'TRateCorr': '0.239', 'TPastBitsOne': '1.322', 'TPastBitsFour': '0.240', 'TPastBitsEight': '0.011', 'TLongDelta': '0.066', 'TLongE': '0.442', 'TLongChsh': '1.117'})
# research/toymodel_v3 (section 'A second pass'); recomputed by toymodel_v3_numbers()
EXPECTED.update({'VClockOnly': '1.00', 'VFullBend': '2.00', 'VSlabLag': '105.5', 'VReff': '2.81', 'VSpecHitOne': '0.31', 'VSpecHitTwo': '0.63', 'VChairA': '2.02', 'VChairFlat': '7', 'VHopAxis': '0.55', 'VHopFace': '0.71', 'VHopBody': '0.66', 'VHopSpread': '0.24', 'VTensorSpread': '2\\times10^{-4}', 'VRelaxDrop': '4', 'VNotchRatio': '9', 'VBodyNear': '2.2', 'VBodyFar': '0.1', 'VAreaSix': '1020', 'VAreaEight': '34920', 'VPsi': '-3/14', 'VPinSignChsh': '3.92', 'VChWorst': '-0.0088', 'VRecBoth': '-2.96', 'VRecFlat': '-0.93', 'VRecAmp': '-2.03', 'VDiffWing': '0.26', 'VFluxLo': '0.017', 'VFluxHi': '0.93', 'VFreeOne': '9', 'VFreeFive': '23', 'VFarThirteen': '88', 'VFarSeventeen': '139', 'VFarRatio': '1.6'})
EXPECTED.update({'TSpeedDense': '1.947', 'TSpeedOne': '0.970', 'TStepRefl': '0.118', 'TFrontSlope': '1.09', 'TPairNear': '0.16', 'TPairFar': '0.006', 'TRuleIErr': '10'})

# research/toymodel_v4 and spectre_hat_time_3d (section on the flow of the 14th vertex); recomputed by toymodel_v4_numbers()
EXPECTED.update({'WTauStar': '0.766', 'WJumpZero': '3.464', 'WJumpHalf': '1.155', 'WReadHalf': '0.333', 'WOmegaCThree': '133.0', 'WFirstBadThree': '133', 'WOmegaCFive': '132.1', 'WFirstBadFive': '133', 'WOmegaCEight': '120.5', 'WFirstBadEight': '121', 'WForbidEnd': '0.148', 'WPairsTwo': '6', 'WPatchTilesTwo': '71', 'WPairPctTwo': '16.9', 'WFreeTilesTwo': '21', 'WPairsThree': '82', 'WPatchTilesThree': '559', 'WPairPctThree': '29.3', 'WFreeTilesThree': '89', 'WPairsFour': '804', 'WPatchTilesFour': '4401', 'WPairPctFour': '36.5', 'WFreeTilesFour': '377', 'WValidViol': '5', 'WValidBand': '136', 'WValidSteps': '690', 'WValidPoints': '275', 'WSignPatterns': '6', 'WGridCells': '85', 'WGridMono': '62', 'WGridMonoRes': '62', 'WGridValidStart': '62', 'WResBeforeHat': '1', 'WDeltaSup': '313', 'WDeltaNormal': '264', 'WClusterSize': '16', 'WPairSix': '58106', 'WPairSeven': '469766', 'WPairLimit': '0.2253', 'WPairFive': '0.2024', 'WBoundaryRatio': '0.538', 'WBx': '1.211', 'WOmegaCStraight': '134.2', 'WOmegaCHat': '120', 'WCoupledTiles': '164', 'WChangesLog': '0', 'WChangesInv': '1', 'WChangesInvTwo': '1', 'WSpinEdges': '14'})

# research/toymodel_v5 (section 'A third pass'); recomputed by toymodel_v5_numbers()
EXPECTED.update({'XAnisoHalf': '0.333', 'XChshZero': '1.670', 'XChshHalf': '1.943', 'XChshHex': '2.828', 'XChshOne': '3.057', 'XBetaTwoHalf': '1.308', 'XWindowEnd': '0.391', 'XWindowEndTwo': '0.718', 'XCubicTwo': '1.992', 'XPlanarEight': '3.9993', 'XMonopole': '0', 'XTraceQ': '91/50', 'XQLow': '-47.8', 'XQMid': '0.10', 'XQHigh': '47.7', 'XAxialPct': '0.4', 'XChairSlope': '-3.00', 'XMinMonopole': '1.732', 'XMinPatterns': '560', 'XOverlap': '0.816', 'XResidual': '0.577', 'XLiveDipole': '41.6', 'XClockErrThree': '30.9', 'XClockErrTen': '0.9', 'XClockErrForty': '0.05'})
# research/toymodel_v6 (section 'A Hamiltonian for the bits'); recomputed by toymodel_v6_numbers()
EXPECTED.update({'YTwoC': '0.913', 'YTwoChsh': '2.708', 'YTwoGap': '0.095', 'YTwoEnd': '0.102', 'YSlope': '-2.99', 'YAntiFour': '0.50', 'YFrusTwo': '0.060', 'YFrusLo': '0.128', 'YFrusHi': '0.165', 'YPeakLo': '0.38', 'YPeakHi': '0.65', 'YPeakTauLo': '0.05', 'YPeakTauHi': '0.13', 'YChshMax': '2.33', 'YChshObs': '2.12', 'YGTiles': '71', 'YGLinks': '406', 'YGPlaq': '336', 'YGTwo': '254', 'YGPairsThree': '82', 'YGZZ': '394', 'YGXX': '0', 'YGCutLinks': '215', 'YGCutBits': '12'})
# research/toymodel_v7/t7_rg.py (subsection 'Renormalisation by the substitution'); recomputed by toymodel_v7_numbers()
EXPECTED.update({'ZPairsTwo': '6', 'ZInsideTwo': '0', 'ZPairsThree': '82', 'ZInsideThree': '0', 'ZPairsFour': '804', 'ZInsideFour': '0', 'ZNewThree': '76', 'ZInhThree': '6', 'ZNewFour': '722', 'ZInhFour': '82', 'ZTypesThree': '96', 'ZTypesFour': '194', 'ZTypesCommon': '74', 'ZTypesNew': '120', 'ZScale': '2.803', 'ZLambda': '2.806', 'ZRatioMed': '1.002', 'ZRatioLo': '0.975', 'ZRatioHi': '1.025', 'ZLambdaMinusThree': '0.045'})

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
