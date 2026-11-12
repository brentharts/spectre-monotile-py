# Stage 5: results (dynamics and the clock, with the chair and the ensemble as source)

Labels as in `PLAN.md`. Nothing here is gravity. File: `t5_clock.py` (4 tests, about 2 s), figure `figures/t5_stage5_clock.png`.

```
python3 research/toymodel_v5/t5_clock.py --tests-only
python3 research/toymodel_v5/t5_clock.py --no-show --save research/toymodel_v5/figures/t5_stage5_clock.png
```

## 1. What was tested

The identity C7 of `toymodel_v2` (rule I, reading B): the squared jump of the bit follows the static field, c^2 = exp(-g |phi|), and the overlap c should be the clock sqrt(1 - r_s / r) with r_s = g times the monopole. Two sources: (A) the chair alone; (B) the chair plus the ten bits of Stage 3 with the pattern that cancels the chair's xx - yy quadrupole (net bit monopole sqrt3, Hat bits silent). The modulus |phi| is a Definition (the chair's potential changes sign, a squared jump cannot).

## 2. Results

| id | statement | label |
|---|---|---|
| E5.24 | The chair switched on at t = 0 on the 64^3 sheet leaves a time-averaged field equal to the static periodic Poisson solution within 3.7% (shell r = 7 to 14 sites). Energy including the source work is conserved and the run retraces | Numeric / Exact |
| E5.25 | **C7 fails for the chair alone.** The angular rms of |phi| falls with slope -3.004 (r = 20 to 200 chair units), not -1; the monopole is 0, so r_s = 0; the potential changes sign over the sphere, so c = 1 on the nodal cones at every radius | Exact, **Negative** |
| E5.26 | With the bits, the rms slope is -1.00000 far away and r |phi| is within 1% of sqrt3 from r = 8.4 chair units outward. There c agrees with the clock to 0.9% at r = 10, 0.2% at 20, 0.05% at 40, 8e-5 at 100: first order only, as in v2. Close in, c is 31% below the clock at r = 3 and 1.6% below at 5. The 1/r comes from the unpaired live bit, not from the chair | Numeric |
| E5.27 | The field is exactly zero beyond lattice distance n of every source after n steps (n = 12); a probe 6 sites above a full tile sees the front at step 8 through a dense line, 9 in vacuum | Exact / Numeric |

## 3. What this says

1. **Stop condition met for the chair as the source.** The identity C7 needs a 1/r profile; the chair, neutral at two orders, gives r^-3 and a sign-changing field. Its clock would be 1 along the nodal cones at every radius.
2. **The identity is rescued by the bits.** The unpaired live bit (the odd count of Stage 3, E5.17) supplies a monopole, and beyond about 8 chair units the clock agrees with c to first order. The identity is then a statement about the bit's strength sqrt3 g, and the chair's 192 charges play no part in it beyond r about 8.
3. **The two arrows are consistent.** The chair needs the bits to give a clock; the bits alone would give the same far field. So in this toy the chair is a decoration of the near field (r below about 8), where the quadrupole and the dipole of the live layer (Stage 4) break the sphericity.
4. **No horizon and no decision between readings.** |phi| is finite away from the sources, so c does not reach 0; readings A and B of v2 are not decided by this stage (same conclusion as v2 Stage 5).

## 4. Not done

The coupling of g to a model quantity (g = 1 here, only the profile is tested); the clock from the dynamics rather than the settled field; the bits as dynamical variables; the lattice version of the 1/r fit (only the settled-field agreement is done).
