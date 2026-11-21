# Reproducing `quantgrav.tex`

Everything the paper quotes can be rebuilt from this repository and six other repositories with one script:

```
python3 repo_paper.py --list          # what it would run, with estimated times
python3 repo_paper.py all             # clone the other repositories, run every self test, redraw the figures,
                                      # write the numbers, build build/quantgrav.pdf   (about 45 minutes)
python3 repo_paper.py all --long      # also the runs of more than ten minutes (a few hours)
```

`repo_paper.py` uses only the standard library; logs go to `build/repo_paper/NAME.log` and a summary to `build/repo_paper/summary.json`. Self tests are found by looking for `--tests-only` in each script, and figure commands are read from the `python3 X.py ... --save ...` lines of the module docstrings, so a new module is picked up without editing the script. Options: `--deps DIR` (where the other repositories live, default: next to this one), `-j N` (run independent tests and figures N at a time), `--only REGEX`, `--dry-run`, `--keep-going`, `--install` (pip install missing packages), `--pin` (check out the commits of the snapshots, below).

Needs: Python 3.8 or later, `numpy`, `scipy`, `sympy`, `matplotlib`, `mpmath`, `networkx`, `shapely` (the last three only for `toymodel_v9`), `make`, `git`, and `latexmk` with `pdflatex` for the paper. `python3 repo_paper.py packages` checks them.

## Steps

| step | what it does | time |
|---|---|---|
| `deps` | `git clone` the repositories below (skips those present) | a minute |
| `packages` | checks the Python packages and tools | seconds |
| `tests` | every `--tests-only` self test (about 100 scripts), `toymodel_v9` by its Makefile, and the tests of `wada-coudene` and `HopfFibration` | about 15 minutes |
| `figures` | `make figures`, every figure command found in a docstring, and the three figures whose command is only in the paper | about 15 minutes |
| `data` | statistics that the paper quotes from `toymodel_v9` and `toymodel_v10` (dipole, pair rule, cavity k = 80, flip engine tests) | about 6 minutes |
| `numbers` | `models/paper_numbers.py` writes `build/quantgrav_numbers.tex` (the `\qg...` macros) | about a minute |
| `paper` | `latexmk` builds `build/quantgrav.pdf` (needs `numbers`) | 2 minutes |
| `--long` | the cavity of 300 tiles (about 12 minutes), `make patch-long LEVEL=11` (about 20 minutes, 2 GB), `make flip-all` (about 20 minutes), the v8 flip statistics by `t9_engine.py --stats` (about 12 minutes) | hours |

A test that prints `SKIPPED` (for instance because a repository is missing) is reported as "ok (some checks SKIPPED)", not as passed.

## The other repositories

| directory | URL | variable read by the modules | used by |
|---|---|---|---|
| `spectre` | <https://github.com/brentharts/spectre> | `SPECTRE_DIR` | exact Spectre construction (`boundary_certificate/spectre_exact.py`), `models/_spectre.py`, `toymodel_v9`, `toymodel_v10` |
| `wada-coudene` | <https://github.com/panjea/wada-coudene> | `WADA_DIR` | Appendices on windows and Yang--Mills, `t9_engine.py` (window rasters), `python3 -m pytest tests` |
| `Quantum-Gravity-Dynamics` | <https://github.com/brentharts/Quantum-Gravity-Dynamics> | `QGD_DIR` | `models/fig_chsh_price.py`, `models/fig_retro_rule.py`, Appendix on the dynamic proposal (`pytest tests/test_directional_hex*.py`) |
| `4D_Wilson_Loop_RG` | <https://github.com/brentharts/4D_Wilson_Loop_RG> | `WILSON_RG_DIR` | `models/wilson_corner_locality.py`; recomputing the snapshot is `make test` there (about 10 minutes) |
| `HopfFibration` | <https://github.com/brentharts/HopfFibration> | `HOPF_DIR` | Hopf lift of the Wada result (`python3 hopf_wada.py --tests`) |
| `CICY` | <https://github.com/brentharts/CICY> | `CICY_DIR` | cross-checks of the companion results |
| `qwla` | <https://github.com/panjea/qwla> | `QWLA_DIR` | the erasure-code notes quoted in Section 4 |

The modules look for these as `$VARIABLE`, then `../<directory>`, then `/home/claude/<directory>`; `repo_paper.py` sets the variables itself.

## Where each module is and how it is run by hand

All commands run from the root of this repository. Each module of the table below has a self test with the number of checks in brackets.

- `python3 models/X.py --tests-only` for the modules of the static half (Appendices on $E_8$ to the Wilson-loop probe) and the tight-binding appendix; `python3 research/toymodel_v2/X.py --tests-only` for the staged test of the dynamic half, which also uses `spectre_hat_quantum_bit_model.py` and `spectre_hat_backward_snap_model.py` in the root.
- Modules named `t3_`, `t4_`, `t5_`, `t6_`, `t7_` are in `research/toymodel_v3` to `v7` and run the same way (`python3 research/toymodel_v7/t7_targets.py` prints the scoreboard of the section "toward physics"). `t3_relax` takes about 1.5 minutes, `t4_open` and `t4_couple` about a minute, the others seconds.
- `wilson_corner_locality` needs `WILSON_RG_DIR`.
- The 14th-vertex flow (Section "Spectre to Hat"): `python3 spectre_hat_time_3d.py --no-show --save figures/spectre_hat_time_3d.png` is the first figure of that section; the same with `--corner 160 --save figures/spectre_hat_time_3d_omega160.png` is the second; `python3 spectre_hat_critical.py` writes the critical-angle figure; `--gif FILE` writes the turntable.
- Figures of the Chair44 ensemble are written by `research/toymodel_v5/t5_alphabet.py` and `t5_source.py` (`--no-show --save PATH`), the pair-qubit patch figure by `research/toymodel_v6/t6_patch.py` (15 seconds), and the renormalisation figure by `research/toymodel_v7/t7_rg.py`.
- The flip sections: `toymodel_v8` (stage notes, JSON and logs; `plot_rerun.py` draws the figures), `toymodel_v9` (`make quick`, `make patch-tests`, `make flip-all`; see its `Makefile`), `toymodel_v10` (`make quick cavity rules` in `research/toymodel_v10`, needs `SPECTRE_DIR`; the cavity of 300 tiles is `python3 t10_cavity.py --scan 300`, about 8 minutes).

| Section | Test modules (number of checks) |
|---|---|
| Foundations | `t4_proofs` (4), `hopf_shells` |
| 14th-vertex flow | `t4_geometry` (4), `t4_arrow` (5), `t4_patch` (6), `t4_spin` (7), `spectre_hat_time_3d` (3) |
| flow results, coupling | `t4_open` (4), `t4_couple` (3) |
| $E_8$ | `hopf_shells` (33), `icosian_e8` (17), `e8_shells` (45) |
| windows | `fibre_window` (11), `voronoi_window` (10) |
| $E_6$ | `e6_fibres` (25) |
| Spectre in $E_8$ | `spectre_in_e8` (13) |
| Chair44 count | `chair_e8_192` (11) |
| Wilson-loop probe | `wilson_corner_locality` (9) |
| companion repositories | Hat and Spectre repository |
| windows, Yang--Mills | `tests/test_spectre_window.py`, `tests/test_yangmills.py` (`wada-coudene`), `hopf_wada` (`HopfFibration`) |
| dynamic proposal tests | Quantum-Gravity-Dynamics fork |
| staged test: lattice | `tm2_lattice` (9) |
| staged test: history | `tm2_history_sum` (8), `core` (7) |
| staged test: arrow | `tm2_arrow` (11), `tm2_directed` (9) |
| staged test: rule | `tm2_rule` (11), `tm2_checking` (6) |
| staged test: waves | `tm2_waves` (9) |
| second pass: gravity | `t3_optics` (5), `t3_escape` (6), `t3_chair_metric` (4) |
| second pass: arrow | `t3_area` (4), `t3_chair_handedness` (4), `t3_records` (4) |
| second pass: quantum, relaxation | `t3_passrate` (3), `t3_qubit` (6), `t3_relax` (3) |
| third pass: alphabet | `t5_alphabet` (5), `t5_rule` (5), `chair_fibers_14th` (11) |
| third pass: source, ensemble | `t5_source` (4), `t5_ensemble` (5), `t5_arrow` (4), `t5_clock` (4) |
| Hamiltonian | `t6_pair` (5), `t6_patch` (6), `t6_gauge` (6) |
| renormalisation | `t7_rg` (4) |
| toward physics | `t7_targets` (1), `t7_bell` (4), `t7_fields` (4), `t7_layout` (3), `t7_space` (3), `t7_lorentz` (4), `t7_horizon` (3), `t7_entropy` (3), `t6_hop` (5), `t6_clock` (5) |
| flips, pair rule, pair recurrence | `research/toymodel_v10`: `t10_cavity`, `t10_dipole`, `t10_pair_proof` (4), `t10_rules` (3); `research/toymodel_v8/t8_parity` (5); the flip engines `spectre_flip_contacts`, `spectre_flip_stats`, `spectre_flip_moves` are scripts with logs, not self tests (`t9_engine.py` carries their regression) |
| Danzer appendix | `research/toymodel_v9` (`make quick patch-tests win-tests flip-tests`; about 40 minutes; `make flip-all` regenerates the clouds, patch, statistics and figures) |
| tight binding | `_tb` (22), `twist_hopping` (29), `chair_fibers` (10) |

(The numbers in brackets are those printed by the paper at the time of writing; the scripts are the authority.)

## Numbers marked with a dagger

A dagger in the paper marks a number that is copied from a snapshot or from a log, not recomputed by `models/paper_numbers.py`. Where they come from:

- **Section on phason flips.** The logs and JSON files in `research/toymodel_v8` (`RERUN.md`, `STAGE1.md`, `STAGE3.md`, `STAGE7.md`, `STAGE7B.md`), run with `figures/spectre_flip_*.py` of the `wada-coudene` repository; the figures are drawn by `plot_rerun.py`. `research/toymodel_v9/t9_engine.py` reproduces the counts 539 / 468 / 244 / 55 / 169 and the disc statistics (`--tests-only`, `--stats`). The exact-refilling, dipole and pair-rule results are the logs and JSON of `research/toymodel_v10` (`log_*.txt`, `results_*.json`).
- **Wilson-loop appendix.** Printed strings copied into `models/data/wilson_snapshot.json` from the Wilson-loop repository at commit `52a3972` (7 and 8 November 2026); recomputing them is `make test` there (about 10 minutes). `python3 repo_paper.py deps --pin` checks out that commit.
- **Companion repositories and the dynamic proposal.** `models/data/companion_snapshot.json`, with the commit of each source, from the self tests of the Hat and Spectre repository (`--tests`) and the Quantum-Gravity-Dynamics fork (`pytest tests/test_directional_hex*.py`).
- **Windows and Yang--Mills appendices.** `models/data/wada_snapshot.json`, from the `wada-coudene` repository (`YANGMILLS.md` sections 6 and 8 to 13 and the CSV files of `figures/`, 18 November 2026) and from `hopf_wada.py`; a few inline numbers of the Yang--Mills appendix (the Hessian interval, the three-variable fractions) are quoted from the same document without a macro.
- Closed forms ($\log_2 3$, $2\sqrt2$, the $\delta$ thresholds, $\rho_0$, $4-\sqrt{15}$) are computed by `paper_numbers.py`.

A mismatch with a snapshot is a reason to doubt the paper, not its source.

## What has and has not been run through `repo_paper.py`

Run on 2026-11-21 in a fresh cloud workspace (2 cores), with the other repositories at their current heads:

| step | result |
|---|---|
| `tests` (without `--long`) | 102 steps: 98 ok, 1 failed, 3 not run (`--long`). The failure is in another repository: `wada-coudene` `tests/test_spectre_window.py::test_neighbour_window_partition_tables_are_not_wada` needs `figures/spectre_window_partition_N3_c5.csv`, which is not in that repository (the class-5 covering was not committed; compare the note in `research/toymodel_v8/RERUN.md` that classes 2 and 5 were not rerun). The other 70 tests there pass. |
| `figures`, `data` (run on a scratch copy so that the committed figures stay untouched) | 58 steps, 0 failed; `make figures` 3 minutes |
| `numbers`, `paper` | ok; `build/quantgrav.pdf` has no undefined references (the numbers step took 5 minutes while other jobs ran; it takes about a minute alone) |
| `--long` steps | **not run**: `v9-patch-long`, `v9-flip-all`, `v9-engine-stats`, `data-v10-cavity-k300`. Their commands are the ones given in the stage notes and Makefiles; the cavity of 300 tiles was run by hand (459 s). |
| `deps --pin` | not run (the Wilson-loop repository was used at its current head, 17a3d25, not at 52a3972, so its snapshot is not rechecked here) |

`make figures` and the figure commands overwrite the files of `figures/` and `research/toymodel_v*/figures/`; run `python3 repo_paper.py figures` on a clean checkout (or a copy) and compare with `git diff` before committing redrawn figures.
