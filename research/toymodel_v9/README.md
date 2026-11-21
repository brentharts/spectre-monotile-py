# `toymodel_v9` — Exact SAT/DRAT Cavity Rigidity, Symbolic Proof of Result 36, and Dipolar Frustration Scaling

Drop-in verification suite and LaTeX additions (`LATEX_ADDITIONS.tex`) addressing **Open Problems #9, #14, and #15** and `MATH_GAPS.md` **Gap O2** of `quantgrav.tex`, following the paper's `Exact` / `Numeric` / `Negative` / `Hypothesis` evidence conventions and Table 3's `--tests-only` self-test contract.

## Summary of Results

| Module | Checks | Scope & Verdict | Label |
|---|---|---|---|
| `t9_pair_proof.py` | 4 | Exact 9→2 (`O`/`G`) geometric collapse; closed **15-state** symbolic $\mathbb{Z}[\omega]^4$ seam automaton (reflection $R$ cancels identically, $M\mathbf{b}_k = \mathbf{b}_{k+1}$ exact); rational Krylov proof (`dim = 4`) of Result 36's order-5 recurrence $(x-1)(x^2-8x+1)(x^2-4x-1)=0$ uncovering exact $(-1)^k$ parity cancellation; Result 20 boundary table closes at Level 4 (**66 directed / 53 canonical, 35 positive; 0 new at Level 5**). | `Exact` |
| `t9_cavity_rigidity.py` | 6 | Exact $\tfrac{1}{2}\mathbb{Z}[\sqrt{3}]$ Stokes-flux SAT + `drat-trim` certification: 12/12 orientations 1-tile UNSAT; **zero chiral closed flip clusters up to 300 erased tiles ($r = 27.96$, zero hull assumption)** and **180 tiles DRAT-verified** (12.84M clauses); chirality control (`reflect=True`) breaks immediately at $k=4$ via an exact 2-tile closed swap (`dV=dE=dF=0`); **121/121 (100%)** random i.i.d. erasure recovery across $p \in [0.05, 1.00]$ on a 160-tile Level-4 disc; minimal chiral rearrangement defect $D_{\min}=20$ (`drat-trim` UNSAT at 19, two-pocket dipole `[12, 8]`); **3D Chair44** Depth-3 all 225 interior chairs (1,575 cubes, 5.62M clauses) DRAT-verified UNSAT (bare 7-cube chair also UNSAT). | `Exact`; `Negative` (local chiral Pachner moves) |
| `t9_dipole_scaling.py` | 3 | Exact OpenMP C++20 Gray-code extension of Result 17 from $N=14$ to **$N=28$ exact in 0.063 s** ($f_{\text{unsat}} = 0.209125$, unique ground state `deg = 1 mod Z2` at every size); **32 independent Parallel Tempering + Houdayer ICM + 2-opt ensembles** (`dipole_pt.cpp`) tighten the larger-patch **variational upper bounds** to $f \le 0.229310$ on full Level 3 ($N=82$, `32/32` hits) and on Level 4: $0.275472$ ($N=82$, `32/32`), $0.281228$ ($N=150$, `32/32`), $0.291321$ ($N=250$, `27/32`), $0.293813$ ($N=400$, `9/32`), and **$f \le 0.289026$** on full Level 4 ($N=804$, `13/32` hits, median `0.289047` — strictly reported as `Numeric` upper bounds, not an established thermodynamic plateau). | `Exact` ($N \le 28$), `Numeric` upper bounds ($N \le 804$) |
| `t9_frontiers.py` | 5 | Holographic 270° wedge rigidity & first exact open-boundary chiral phason wedges (`rem=31, add=36` at 300°); Heesch × Spectre DRAT proofs ($H_c = 0$ for 28/28 single-vertex $\pm h$ moves & Result 81 $\pm h$ chiral/reflect; exact $H_c = 1$ for flat-pair-forbidden straight Spectre); Quantum Lanczos ED to **$N = 24$ qubits**; depth-8 all-12-class flips ($\alpha/2 = 0.4402$) & Langevin $R(T) \propto T^{0.4137}$ ($R^2 = 0.99902$). | `Exact` / `Numeric` / `Negative` |

## Reconciliation with `toymodel_v8` (`STAGE3.md`)

There is **zero conflict** between `toymodel_v8` and `toymodel_v9` on chiral phason flips — they measure the two complementary halves of the same phenomenon:
- **`toymodel_v8` (`STAGE3.md`)** shifted the global internal phase $z \to z + \delta$ inside a finite placement box (`R2 = 14`, complete only to radius `8.5`) and found 818 overlap-linked clusters of `8–17` tiles per side (`1–13` in the first run), of which **0 of 818 closed** inside the validated radius `8.5`, leaving open whether closure would occur around radius `~26`.
- **`toymodel_v9`** resolves that open question on both sides: (1) **Part I (`t9_cavity_rigidity.py`)** proves exhaustively (zero hull assumption) that **zero closed chiral flip clusters exist up to 300 single tiles ($r = 27.96$, diameter $> 55$)**, ruling out radius-`~26` closure; (2) **Part II (`t9_frontiers.py`, `E9.15`)** explains what `v8`'s unclosed `8–17`-tile events are — finite-window cross-sections of **non-local open-boundary chiral phason wedges** (which on a 60-tile disc stay DRAT-locked at the center up to $\theta_{\text{open}} = 270^\circ$ and nucleate a half-disc wedge `rem=31, add=36` at $300^\circ$).
- *Accounting note:* `v8` counts Mystic pairs `G` as one placement of area $2 A_0$ (so area-conserving flips can have $\Delta N_{\text{placements}} = \pm 1$), whereas `v9` works directly in individual chiral Spectre tiles of equal area $A_0$ (where any closed cavity filling strictly satisfies $n_{\text{rem}} = n_{\text{add}}$).

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
