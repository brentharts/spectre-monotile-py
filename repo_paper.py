#!/usr/bin/env python3
"""repo_paper.py - rebuild the data, the figures and the PDF of quantgrav.tex from scratch.

The paper needs more than this repository: some modules import other repositories (Hat and Spectre, Wada, the
Quantum-Gravity-Dynamics fork, the Wilson-loop repository).  This script clones them, checks the Python packages, runs
every self test, redraws the figures, writes the numbers and builds the PDF.  Nothing is hard-coded about what a module
does: self tests are found by looking for `--tests-only` in each script, and figure commands are read from the
`python3 X.py ... --save ...` lines of the module docstrings.  Notes, times and the list of manual steps: REPRODUCE.md.

    python3 repo_paper.py --list                  show the steps and their estimated times, run nothing
    python3 repo_paper.py deps                    clone the other repositories (into --deps, default: next to this one)
    python3 repo_paper.py all                     deps + packages + tests + figures + numbers + paper  (about 30 minutes)
    python3 repo_paper.py tests -j 4              only the self tests, four at a time
    python3 repo_paper.py figures numbers paper   a subset, in the order given
    python3 repo_paper.py all --long              also the runs that take more than ten minutes (hours in total)
    python3 repo_paper.py all --dry-run           print the commands only
    python3 repo_paper.py tests --only t7_        only steps whose name matches the regular expression

Steps: deps, packages, tests, figures, data, numbers, paper, all (= deps packages tests figures data numbers paper).
`data` regenerates the files that the paper quotes from other runs (toymodel_v9 and v10 statistics); with `--long` it
includes the runs of more than ten minutes.  Logs go to build/repo_paper/NAME.log, the summary to
build/repo_paper/summary.json.  Exit status 1 if any step failed.  Python 3.8 or later, standard library only.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.realpath(__file__))
BUILD = os.path.join(ROOT, "build")
LOGDIR = os.path.join(BUILD, "repo_paper")

# other repositories: (directory name, url, environment variable read by the modules, pinned commit or None)
DEPS = [
    ("spectre", "https://github.com/brentharts/spectre.git", "SPECTRE_DIR", None),
    ("wada-coudene", "https://github.com/panjea/wada-coudene.git", "WADA_DIR", None),
    ("Quantum-Gravity-Dynamics", "https://github.com/brentharts/Quantum-Gravity-Dynamics.git", "QGD_DIR", None),
    ("4D_Wilson_Loop_RG", "https://github.com/brentharts/4D_Wilson_Loop_RG.git", "WILSON_RG_DIR", None),
    ("HopfFibration", "https://github.com/brentharts/HopfFibration.git", "HOPF_DIR", None),
    ("CICY", "https://github.com/brentharts/CICY.git", "CICY_DIR", None),
    ("qwla", "https://github.com/panjea/qwla.git", "QWLA_DIR", None),
]
# The snapshots of quantgrav.tex were taken at these commits (see REPRODUCE.md); set --pin to check them out.
PINS = {"4D_Wilson_Loop_RG": "52a3972"}

PACKAGES = ["numpy", "scipy", "sympy", "matplotlib", "mpmath", "networkx", "shapely"]

# estimated seconds of the slow self tests (from the paper and the stage notes); everything else is "seconds"
SLOW = {"t3_relax.py": 100, "t4_open.py": 70, "t4_couple.py": 70, "t10_cavity.py": 70, "t9_engine.py": 150,
        "wilson_corner_locality.py": 60, "fig_chsh_price.py": 70}
# scripts that have `--tests-only` but are run as part of something else, or need a long run to be meaningful
SKIP_TESTS = {"research/toymodel_v9/t9_engine.py"}      # run by the `data` step
# toymodel_v9 is driven by its own Makefile, not script by script
V9_TESTS = [("v9-quick", "make quick", 300, False), ("v9-patch-tests", "make patch-tests", 300, False)]
V9_LONG = [("v9-patch-long", "make patch-long LEVEL=11", 1200, True), ("v9-flip-all", "make flip-all", 1200, True),
           ("v9-engine-stats", "make engine-stats", 700, True)]

FIG_FIXED = [   # figures drawn by commands that are written in the paper rather than in a docstring
    ("fig-spectre_hat_time_3d", "python3 spectre_hat_time_3d.py --no-show --save figures/spectre_hat_time_3d.png", 60),
    ("fig-spectre_hat_time_3d-160", "python3 spectre_hat_time_3d.py --no-show --corner 160 --save figures/spectre_hat_time_3d_omega160.png", 60),
    ("fig-spectre_hat_critical", "python3 spectre_hat_critical.py", 60),
    ("fig-v8-plot_rerun", "python3 plot_rerun.py", 30, "research/toymodel_v8"),
]
DATA_STEPS = [   # (name, cwd, command, seconds, long)
    ("data-v10-dipole", "research/toymodel_v10", "python3 t10_dipole.py --full", 30, False),
    ("data-v10-rules", "research/toymodel_v10", "python3 t10_rules.py --levels 6", 60, False),
    ("data-v10-cavity-k80", "research/toymodel_v10", "python3 t10_cavity.py --scan 80", 120, False),
    ("data-v10-cavity-k300", "research/toymodel_v10", "python3 t10_cavity.py --scan 80 150 300", 700, True),
    ("data-v9-engine-tests", "research/toymodel_v9", "python3 t9_engine.py --tests-only", 150, False),
    ("data-v9-engine-stats", "research/toymodel_v9", "python3 t9_engine.py --stats", 700, True),
]


class Step:
    def __init__(self, name, cmd, cwd=".", secs=10, long=False, group="tests", env=None, parallel=True):
        self.name, self.cmd, self.cwd, self.secs, self.long, self.group = name, cmd, cwd, secs, long, group
        self.env, self.parallel = env or {}, parallel
        self.status, self.elapsed = "pending", 0.0

    def log(self):
        return os.path.join(LOGDIR, re.sub(r"[^A-Za-z0-9_.-]", "_", self.name) + ".log")


def dep_dir(args, name):
    return os.path.join(os.path.abspath(args.deps), name)


def environment(args):
    env = dict(os.environ)
    env["MPLBACKEND"] = "Agg"
    for name, _url, var, _pin in DEPS:
        d = dep_dir(args, name)
        if os.path.isdir(d):
            env.setdefault(var, d)
    return env


# ------------------------------------------------------------------ discovery
def scripts_with(flag, folders):
    out = []
    for folder in folders:
        full = os.path.join(ROOT, folder)
        if not os.path.isdir(full):
            continue
        for f in sorted(os.listdir(full)):
            if f.endswith(".py"):
                try:
                    txt = open(os.path.join(full, f), encoding="utf-8").read()
                except OSError:
                    continue
                if re.search(r"[\"']" + re.escape(flag) + r"[\"']", txt) and f != os.path.basename(__file__):
                    out.append((folder, f, txt))
    return out


def research_dirs():
    base = os.path.join(ROOT, "research")
    return sorted("research/" + d for d in os.listdir(base) if d.startswith("toymodel_v")) if os.path.isdir(base) else []


def test_steps():
    steps = []
    for folder, f, _txt in scripts_with("--tests-only", [".", "models"] + research_dirs()):
        rel = os.path.normpath(os.path.join(folder, f))
        if rel in SKIP_TESTS or folder == "research/toymodel_v9":
            continue
        steps.append(Step("test-" + rel.replace("/", "."), "python3 %s --tests-only" % f, folder, SLOW.get(f, 15), group="tests"))
    for name, cmd, secs, long in V9_TESTS + V9_LONG:
        steps.append(Step("test-" + name, cmd, "research/toymodel_v9", secs, long, "tests", parallel=False))
    steps.append(Step("test-wada-coudene", "python3 -m pytest -q tests", "$wada-coudene", 120, group="tests"))
    steps.append(Step("test-hopf_wada", "python3 hopf_wada.py --tests", "$HopfFibration", 60, group="tests"))
    return steps


FIG_RE = re.compile(r"python3\s+([A-Za-z0-9_/.]+\.py)\s+([^`\"'\\\n]*--(?:save|png|save-dir)[^`\"'\\\n]*)")


def figure_steps():
    steps = [Step("fig-root-make", "make figures", ".", 400, group="figures", parallel=False)]
    seen = set()
    for folder, f, txt in scripts_with("--no-show", ["models"] + research_dirs()) + scripts_with("--save", ["models"] + research_dirs()):
        for m in FIG_RE.finditer(txt):
            script, rest = m.group(1), m.group(2).strip()
            # commands are written either from the repository root (models/X.py) or from the module folder (X.py)
            if script.startswith("models/") or script.startswith("research/"):
                cwd, script = ".", script
            else:
                if os.path.basename(script) != f:
                    continue
                cwd, script = folder, os.path.basename(script)
            cmd = "python3 %s %s" % (script, rest)
            key = (cwd, cmd)
            if key in seen or not os.path.isfile(os.path.join(ROOT, cwd, script)):
                continue
            seen.add(key)
            steps.append(Step("fig-%s" % os.path.normpath(os.path.join(cwd, script)).replace("/", "."), cmd, cwd,
                              SLOW.get(os.path.basename(script), 30), group="figures"))
    for item in FIG_FIXED:
        name, cmd, secs = item[:3]
        cwd = item[3] if len(item) > 3 else "."
        steps.append(Step(name, cmd, cwd, secs, group="figures"))
    return steps


def data_steps():
    return [Step(n, c, d, s, long, "data", parallel=False) for n, d, c, s, long in DATA_STEPS]


def paper_steps():
    return [Step("numbers", "python3 models/paper_numbers.py", ".", 60, group="numbers", parallel=False),
            Step("paper", "latexmk -pdf -f -interaction=nonstopmode -outdir=build quantgrav.tex", ".", 120, group="paper", parallel=False)]


# ------------------------------------------------------------------ running
def run_cmd(step, args, env):
    cwd = step.cwd
    if cwd.startswith("$"):
        cwd = dep_dir(args, cwd[1:])
        if not os.path.isdir(cwd):
            step.status = "skipped (repository %s not cloned: run `deps`)" % os.path.basename(cwd)
            return step
    else:
        cwd = os.path.join(ROOT, cwd)
    cmd = step.cmd.replace("python3", shlex.quote(sys.executable), 1) if step.cmd.startswith("python3") else step.cmd
    if step.cmd.startswith("make ") and not shutil.which("make"):
        step.status = "skipped (make not installed)"
        return step
    if step.cmd.startswith("latexmk") and not shutil.which("latexmk"):
        step.status = "skipped (latexmk not installed)"
        return step
    if args.dry_run:
        print("  [dry-run] (cd %s && %s)" % (os.path.relpath(cwd, ROOT), step.cmd))
        step.status = "dry-run"
        return step
    os.makedirs(LOGDIR, exist_ok=True)
    e = dict(env)
    e.update(step.env)
    t0 = time.time()
    try:
        with open(step.log(), "w") as lf:
            lf.write("$ (cd %s && %s)\n" % (cwd, step.cmd))
            lf.flush()
            p = subprocess.run(cmd, shell=True, cwd=cwd, env=e, stdout=lf, stderr=subprocess.STDOUT,
                               timeout=max(600, 5 * step.secs) if not args.no_timeout else None)
        step.status = "ok" if p.returncode == 0 else "FAILED (exit %d)" % p.returncode
    except subprocess.TimeoutExpired:
        step.status = "FAILED (timeout)"
    step.elapsed = time.time() - t0
    # a test script that prints SKIPPED because a repository is missing counts as skipped, not as passed
    if step.status == "ok" and "SKIPPED" in open(step.log(), errors="replace").read():
        step.status = "ok (some checks SKIPPED, see log)"
    return step


def run_steps(steps, args, env):
    todo = [s for s in steps if (args.long or not s.long) and (not args.only or re.search(args.only, s.name))]
    skipped_long = [s for s in steps if s.long and not args.long and (not args.only or re.search(args.only, s.name))]
    for s in skipped_long:
        s.status = "not run (--long)"
    par = [s for s in todo if s.parallel]
    ser = [s for s in todo if not s.parallel]

    def report(s):
        print("  %-52s %s%s" % (s.name, s.status, "  (%.0fs)" % s.elapsed if s.elapsed else ""), flush=True)
    if par and args.jobs > 1 and not args.dry_run:
        with cf.ThreadPoolExecutor(args.jobs) as ex:
            for s in ex.map(lambda x: run_cmd(x, args, env), par):
                report(s)
    else:
        for s in par:
            report(run_cmd(s, args, env))
    for s in ser:
        report(run_cmd(s, args, env))
        if s.status.startswith("FAILED") and not args.keep_going:
            break
    return steps


# ------------------------------------------------------------------ deps and packages
def step_deps(args):
    results = []
    os.makedirs(args.deps, exist_ok=True)
    for name, url, var, _pin in DEPS:
        d = dep_dir(args, name)
        pin = PINS.get(name) if args.pin else None
        if args.dry_run:
            print("  [dry-run] git clone %s %s%s" % (url, d, " && git checkout " + pin if pin else ""))
            continue
        if os.path.isdir(os.path.join(d, ".git")):
            print("  %-26s present (%s)" % (name, d))
        else:
            r = subprocess.run(["git", "clone", "--quiet", url, d])
            if r.returncode:
                print("  %-26s CLONE FAILED" % name)
                results.append(name)
                continue
            print("  %-26s cloned" % name)
        if pin:
            r = subprocess.run(["git", "-C", d, "checkout", "--quiet", pin])
            print("  %-26s pinned to %s%s" % (name, pin, "" if r.returncode == 0 else "  FAILED"))
        head = subprocess.run(["git", "-C", d, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
        print("  %-26s %s at %s" % ("", var, head))
    return results


def step_packages(args):
    missing = []
    for p in PACKAGES:
        r = subprocess.run([sys.executable, "-c", "import " + p], capture_output=True)
        if r.returncode:
            missing.append(p)
    if missing and args.install and not args.dry_run:
        subprocess.run([sys.executable, "-m", "pip", "install", "--break-system-packages", "-q"] + missing)
        return step_packages(argparse.Namespace(**{**vars(args), "install": False}))
    print("  packages: %s" % ("all present (%s)" % ", ".join(PACKAGES) if not missing else "MISSING " + ", ".join(missing)
                                + "  (run with --install, or: pip install " + " ".join(missing) + ")"))
    for tool in ("make", "latexmk", "pdflatex", "git"):
        print("  tool %-9s %s" % (tool, "found" if shutil.which(tool) else "NOT FOUND"))
    return missing


# ------------------------------------------------------------------ main
GROUPS = {"tests": test_steps, "figures": figure_steps, "data": data_steps}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("steps", nargs="*", default=["all"], help="deps packages tests figures data numbers paper all")
    ap.add_argument("--deps", default=os.path.join(os.path.dirname(ROOT), ""), help="where the other repositories live (default: next to this one)")
    ap.add_argument("--long", action="store_true", help="include the runs of more than ten minutes")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--only", help="regular expression: run only steps whose name matches")
    ap.add_argument("-j", "--jobs", type=int, default=1, help="run independent tests and figures this many at a time")
    ap.add_argument("--keep-going", action="store_true", help="continue after a failed serial step")
    ap.add_argument("--install", action="store_true", help="pip install missing packages")
    ap.add_argument("--pin", action="store_true", help="check out the commits the paper's snapshots were taken at (PINS)")
    ap.add_argument("--no-timeout", action="store_true")
    args = ap.parse_args(argv)
    args.deps = os.path.abspath(args.deps)
    names = args.steps
    if "all" in names:
        names = ["deps", "packages", "tests", "figures", "data", "numbers", "paper"]
    for n in names:
        if n not in ("deps", "packages", "tests", "figures", "data", "numbers", "paper"):
            ap.error("unknown step %r" % n)

    if args.list:
        total = 0
        for n in names:
            if n in GROUPS:
                for s in GROUPS[n]():
                    mark = "  [--long]" if s.long else ""
                    print("  %-52s ~%4ds  (cd %s && %s)%s" % (s.name, s.secs, s.cwd, s.cmd, mark))
                    total += 0 if s.long else s.secs
            elif n == "numbers" or n == "paper":
                for s in paper_steps():
                    if s.group == n:
                        print("  %-52s ~%4ds  %s" % (s.name, s.secs, s.cmd))
                        total += s.secs
            else:
                print("  %s" % n)
        print("  estimated total without --long: about %d minutes" % (total // 60 + 1))
        return 0

    os.makedirs(LOGDIR, exist_ok=True)
    all_steps, failed = [], 0
    for n in names:
        print("== %s" % n, flush=True)
        if n == "deps":
            failed += len(step_deps(args))
            continue
        if n == "packages":
            if step_packages(args) and not args.install:
                print("  (continuing: steps that need a missing package will fail)")
            continue
        env = environment(args)
        steps = GROUPS[n]() if n in GROUPS else [s for s in paper_steps() if s.group == n]
        run_steps(steps, args, env)
        all_steps += steps
        failed += sum(s.status.startswith("FAILED") for s in steps)
        if n == "tests" and not args.dry_run:
            ok = sum(s.status.startswith("ok") for s in steps)
            print("  tests: %d ok, %d failed, %d skipped or not run" % (ok, sum(s.status.startswith("FAILED") for s in steps),
                                                                     len(steps) - ok - sum(s.status.startswith("FAILED") for s in steps)))
        if n == "numbers" and failed:
            print("  (numbers failed: the paper needs build/quantgrav_numbers.tex)")
    if not args.dry_run:
        json.dump([{"name": s.name, "status": s.status, "seconds": round(s.elapsed, 1), "log": os.path.relpath(s.log(), ROOT)}
                   for s in all_steps], open(os.path.join(LOGDIR, "summary.json"), "w"), indent=1)
        bad = [s for s in all_steps if s.status.startswith("FAILED")]
        print("== summary: %d steps, %d failed%s" % (len(all_steps), len(bad), "" if not bad else ": " + ", ".join(s.name for s in bad)))
        if os.path.isfile(os.path.join(BUILD, "quantgrav.pdf")):
            print("   paper: build/quantgrav.pdf")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
