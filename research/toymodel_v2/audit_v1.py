#!/usr/bin/env python3
"""Reproduce the v1 audit numbers quoted in PLAN.md section 1 (Stage 0).

Runs the unmodified toymodel_v1 classes over seeds 0..N-1 and counts:
  AxisNetwork          fraction of runs whose relaxation ends with |flux|^2 <= 3,
                       and fraction where |flux|^2 ended larger than it started
  GravitationalNetwork fraction of runs where the forced shock node still holds
                       the forced axis after relaxation (no well, density 1)
  CoupledParticle      fraction of runs where B's whole timeline is one axis
                       after A is measured

Usage:  python3 audit_v1.py [N]        (default N = 500)
v1 is imported, never modified.
"""
import contextlib
import io
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "toymodel_v1"))
from AxisNetwork import AxisNetwork                                   # noqa: E402
from CoupledParticleNetwork import CoupledParticleNetwork             # noqa: E402
from GravitationalTimelineNetwork import GravitationalTimelineNetwork  # noqa: E402


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def audit(n_runs=500):
    conv = worse = kept = single = 0
    for s in range(n_runs):
        random.seed(s)
        net = AxisNetwork(12)
        f0 = sum(x * x for x in net.compute_net_flux())
        quiet(net.inject_shockwave, 6, "-X")
        f1 = sum(x * x for x in net.compute_net_flux())
        conv += f1 <= 3
        worse += f1 > f0

        random.seed(s)
        g = GravitationalTimelineNetwork(10)
        quiet(g.apply_gravitational_relaxation, 2, "+Z")
        kept += g.timeline[2] == "+Z"

        random.seed(s)
        c = CoupledParticleNetwork(8, "X")
        quiet(c.measure_particle_a, 5, "+Z")
        single += len(set(c.timeline_b)) == 1
    return {
        "axis_converged": conv / n_runs,
        "axis_flux_increased": worse / n_runs,
        "gravitational_shock_kept": kept / n_runs,
        "coupled_B_single_axis": single / n_runs,
    }


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 500
    for k, v in audit(n).items():
        print(f"{k:28s} {v:.3f}")
