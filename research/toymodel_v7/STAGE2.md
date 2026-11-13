# Stage 2: results (the number fields)

Labels as in `PLAN.md`. File: `t7_fields.py` (4 tests, about 3 s, SymPy), figure `figures/t7_stage2_fields.png`.

```
python3 research/toymodel_v7/t7_fields.py --tests-only
python3 research/toymodel_v7/t7_fields.py --no-show --save research/toymodel_v7/figures/t7_stage2_fields.png
```

## 1. The question

The paper's Result `res:field`: the Spectre's lift is Eisenstein (sqrt -3), the icosian E8 lives in Z[phi] (sqrt 5), and the inflation factor lambda generates Q(sqrt6, sqrt10). The open problem asks for one structure that carries both number fields, with lambda as a unit or an automorphism.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.5 | **One automorphism carries both sides.** The Coxeter element w of E8 has order 30 and characteristic polynomial Phi_30, with no fixed vector. Its power w^10 satisfies omega^2 + omega + 1 = 0 with no fixed vector (an Eisenstein structure: E8 = Z[omega]^4). The combination w^6 + w^-6 satisfies g^2 + g - 1 = 0 (a golden structure: E8 a Z[phi]-module). They commute | Exact |
| E7.6 | **The inflation does not join them.** Phi_30 stays irreducible over Q(sqrt6, sqrt10), so Q(lambda) and Q(zeta_30) meet only in Q. No element of the Z[zeta_30] action on E8 is the inflation | Exact, **Negative** |
| E7.7 | **The inflation squared is made of the two sides.** lambda sqrt2 = sqrt3 + sqrt5, so lambda^2 = 4 + sqrt3 sqrt5: the Spectre's 12-fold irrationality times the icosian 5-fold one. 4 + sqrt15 is the fundamental unit of Z[sqrt15] | Exact |
| E7.8 | **Both rotations need rank 16.** A lattice on which a fixed-point-free 12-fold rotation and a fixed-point-free 5-fold rotation act and commute is a Z[zeta_60]-module, so its rank is a multiple of phi(60) = 16. E8 (rank 8) cannot carry both. Q(zeta_60) contains sqrt15 (Phi_60 splits over Q(sqrt15); Phi_30 does not), so lambda^2 is available there | Exact |

## 3. What this says

1. **E8 already unifies the two rotation fields of the paper's two sides**, sqrt -3 and sqrt 5, through one element of its Weyl group. The paper's statement that the two structures use different number fields stays true for the inflation, but not for the 3-fold and 5-fold symmetries: those share a home in Q(zeta_30) acting on E8.
2. **The inflation is the obstruction.** It lives in Q(sqrt6, sqrt10), which shares nothing with Q(zeta_30) except Q. Its square, though, is 4 + sqrt3 sqrt5, a unit built from exactly the two irrationalities that E8 carries separately.
3. **Rank 16, not 8.** The Spectre's own rotation is 12-fold, not 3-fold, and 12-fold with 5-fold together needs rank 16, where sqrt15 and therefore lambda^2 are available. The candidate structures are E8 + E8, the Barnes-Wall lattice, or the trace form of Z[zeta_60]. Which one, if any, makes lambda^2 act as an automorphism that respects the Spectre's lift is the next question (H7.3 revised).

## 4. Not done

A rank-16 lattice with a Z[zeta_60] structure in which lambda^2 = 4 + sqrt15 acts, and the Spectre's lift (A2 + A2, rank 4) inside it; whether the embedding of the paper's Section `sec:embed` extends.
