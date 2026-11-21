# `toymodel_v9` — Exact SAT/DRAT Cavity Rigidity, Symbolic Proof of Result 36, and Dipolar Frustration Scaling

Drop-in verification suite and LaTeX additions (`LATEX_ADDITIONS.tex`) addressing **Open Problems #9, #14, and #15** and `MATH_GAPS.md` **Gap O2** of `quantgrav.tex`, following the paper's `Exact` / `Numeric` / `Negative` / `Hypothesis` evidence conventions and Table 3's `--tests-only` self-test contract.

## Summary of Results

| Module | Checks | Scope & Verdict | Label |
|---|---|---|---|
| `t9_pair_proof.py` | 4 | Exact 9→2 (`O`/`G`) geometric collapse; closed **15-state** symbolic $\mathbb{Z}[\omega]^4$ seam automaton (reflection $R$ cancels identically, $M\mathbf{b}_k = \mathbf{b}_{k+1}$ exact); rational Krylov proof (`dim = 4`) of Result 36's order-5 recurrence $(x-1)(x^2-8x+1)(x^2-4x-1)=0$ uncovering exact $(-1)^k$ parity cancellation; Result 20 boundary table closes at Level 4 (**66 directed / 53 canonical, 35 positive; 0 new at Level 5**). | `Exact` |
| `t9_cavity_rigidity.py` | 6 | Exact $\tfrac{1}{2}\mathbb{Z}[\sqrt{3}]$ Stokes-flux SAT + `drat-trim` certification: 12/12 orientations 1-tile UNSAT; **zero chiral closed flip clusters up to 300 erased tiles ($r = 27.96$, zero hull assumption)** and **180 tiles DRAT-verified** (12.84M clauses); chirality control (`reflect=True`) breaks immediately at $k=4$ via an exact 2-tile closed swap (`dV=dE=dF=0`); **121/121 (100%)** random i.i.d. erasure recovery across $p \in [0.05, 1.00]$ on a 160-tile Level-4 disc; minimal chiral rearrangement defect $D_{\min}=20$ (`drat-trim` UNSAT at 19, two-pocket dipole `[12, 8]`); **3D Chair44** Depth-3 all 225 interior chairs (1,575 cubes, 5.62M clauses) DRAT-verified UNSAT (bare 7-cube chair also UNSAT). | `Exact`; `Negative` (local chiral Pachner moves) |
| `t9_dipole_scaling.py` | 3 | Exact OpenMP C++20 Gray-code extension of Result 17 from $N=14$ to **$N=28$ exact in 0.063 s** ($f_{\text{unsat}} = 0.209125$, unique ground state `deg = 1 mod Z2` at every size); validated multi-start SA + 2-opt (`0.00e+00` error vs. exact at $N=14,20,26,28$) gives $f \le 0.2293$ on full Level 3 ($N=82$) and a bulk plateau **$f \le 0.2948$** on full Level 4 ($N=804$). | `Exact` ($N \le 28$), `Numeric` ($N \le 804$) |

## Running All 13 Self-Tests

```bash
SPECTRE_DIR=/path/to/spectre python3 research/toymodel_v9/t9_pair_proof.py --tests-only
python3 research/toymodel_v9/t9_cavity_rigidity.py --tests-only
python3 research/toymodel_v9/t9_dipole_scaling.py --tests-only
```

## Directory Contents

- `t9_pair_proof.py`, `t9_cavity_rigidity.py`, `t9_dipole_scaling.py` — self-testing modules (`13/13` checks passing)
- `results_*.json` — verified output certificates from the exact C++20 / CaDiCaL 2.1.3 / `drat-trim` pipeline
- `LATEX_ADDITIONS.tex` — drop-in LaTeX `Result` blocks and proof matching `quantgrav.tex` macros
- `figures/` — exact vector SVG figures generated directly from $\tfrac{1}{2}\mathbb{Z}[\sqrt{3}]$, $\mathbb{Z}[\omega]^4$, Chair44, and dipolar ground-state data
- `solvers/` — standalone C++20 (`spectre_fast_cavity.cpp`, `spectre_exp_ab.cpp`, `dipole_solver.cpp`) and Python runners for full end-to-end regeneration
