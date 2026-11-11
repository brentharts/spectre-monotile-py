# Stage 1: results (waves and light bending; the clock from a hitting probability)

Labels as in `PLAN.md`. Nothing here is gravity; "clock" means the factor √(1 − r_s/r) of the companion model, and "light bending" means rays of the model's own wave equation. Files: `t3_optics.py` (5 tests, about 1 s) and `t3_escape.py` (5 tests, about 2 s); figures `figures/t3_optics.png` and `figures/t3_escape.png`.

```
python3 research/toymodel_v3/t3_optics.py --tests-only --story
python3 research/toymodel_v3/t3_escape.py --tests-only --story
```

## 1. The wave equation (plan 2.1)

The corrected equation puts the density in a refractive index, `n² ψ_tt / c² − ∇²ψ = S`, with speed `c/n`.

| id | statement | label |
|---|---|---|
| E1.4 | On the lattice it conserves energy (1e-12) and retraces (1e-10). A pulse through a slab of n = 1.5 and width 200 arrives later than a free one by 105.5 cells. The lattice group index predicts 104.4 and the optical excess (n − 1) × width is 100. **Dense = slow**, as the paper's text says | Exact / Numeric |
| E1.5 | The delay through a 1/r index grows like 2a ln X with the path length: the Shapiro form | Exact |

**Which n?** A static field has a clock factor A and a space factor B (`ds² = −A dt² + B dx²`), so the index is `n = √(B/A)`. With `A = 1 − r_s/r` and `B = A^−γ`:

| model | γ | deflection, first order | second-order coefficient |
|---|---|---|---|
| clock only (**the paper's model as stated**: Δτ = Δt₀/ρ, nothing for space) | 0 | **r_s/b**, half of GR | π/2 |
| space treated like time (B = A) | −1 | **0** | 0 |
| A B = 1 | 1 | **2 r_s/b** | 3π/2 |
| Schwarzschild, isotropic coordinates (the check of the method) | — | 2 r_s/b | 15π/16, as known |

- **Exact (E1.1–E1.3).** The quadrature reproduces 15π/16 for Schwarzschild, so the method is right. The first-order deflection is (1 + γ) r_s/b.
- **Negative for the model as stated.** A clock alone gives half the bending. Two further points:
  - "Treating space and time alike" gives none. The paper's directions-not-dimensions idea, if read that way, removes light bending altogether.
  - The factor 2 needs B = 1/A, a second, spatial coupling that the model does not contain. This repeats the history of scalar theories of gravity.
- At second order the AB = 1 index gives 3π/2 against 15π/16. The radial coordinates are different (the clock √(1 − r_s/r) is Schwarzschild's in areal coordinates), so this is not compared further.

## 2. The clock from a hitting probability (plan 2.2)

Rule (C): `c² = 1 − h`, where h is the probability that the substrate's random walk, started at the bit, ever reaches the source.

| id | substrate | result | label |
|---|---|---|---|
| E2.1 | Z³ (source radius 3, absorbing sphere 40) | `h = R_eff/r` with R_eff = 2.81: the continuum form within 0.8% for r = 4 to 20, and r·h flat to 1% | Exact / Numeric |
| E2.2 | Z² | at r = 6, h = 0.61, 0.71, 0.77, 0.81 as the absorbing radius grows from 20 to 160, following ln(L/r)/ln(L/R) → 1 | Exact / Numeric |
| E2.3 | **Spectre tiling** (vertex graph, levels 4 and 5) | the same as Z²: at r = 9, h = 0.31, 0.51, 0.63 (level 4, L = 15 to 60) and up to 0.78 at L = 240 (level 5, 214 662 vertices), within 0.03 of the planar form | Exact / Numeric |
| E2.4 | rule (C) on Z³ | `c = √(1 − R_eff/r)` to 1%: **the clock factor, a horizon (c = 0 on the source) and 1/r, all from one rule**. With AB = 1 the index 1/(1 − h) bends light by 2r_s/b | Exact algebra on the numbers above |

What this says:

1. **The 2D/3D split of Stage 5 (v2) is recurrence against transience.** A walk on a 3D substrate escapes with probability 1 − r_s/r; on a 2D substrate it never escapes.
2. **Rule (C) closes open problem 8 on a transient substrate.** It gives 1/r, the exact clock and a horizon together, which no rule linear in d² did.
3. **On the planar Spectre tiling it gives none of them.** Every bit becomes unreadable as the walk's room grows: the tiling is "all horizon". A 1/r field needs a 3D substrate, and the Chair44 is the one this repository has (Stage 6).
4. **(C) is a Definition, and why it should hold is a Hypothesis.** The open question is why distinguishability should be an escape probability.

## 3. Light bending and the clock together

Rule (C) on a transient substrate gives A = c² = 1 − r_s/r. Light bending then needs B = 1/A, and nothing yet supplies B. So after Stage 1 the clock has a candidate origin (escape of a walk) and the space factor has none.

The next thing to try, in the spirit of (C), is a walk conditioned to escape. Doob's h-transform of the walk by (1 − h) changes its local speed in exactly the regions where A is small; whether it gives B = 1/A is a computation, not yet done.

## 4. Not done

- 2D wave runs: beam centroids agree with rays only to 10–20%, because of diffraction and the walls, so they are not used as tests.
- The Chair44 substrate (Stage 6).
- A space factor from the model.
