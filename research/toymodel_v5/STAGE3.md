# Stage 3: results (the ensemble as sources on the sheet)

Labels as in `PLAN.md`. Nothing here is gravity. File: `t5_ensemble.py` (5 tests, about 1 s), figure `figures/t5_stage3_ensemble.png`.

```
python3 research/toymodel_v5/t5_ensemble.py --tests-only
python3 research/toymodel_v5/t5_ensemble.py --no-show --save research/toymodel_v5/figures/t5_stage3_ensemble.png
```

## 1. What was built (all Definitions)

On the reversible sheet of `toymodel_v2` (48^3 periodic lattice, leapfrog, dt = 0.4):
- the chair's 192 signed depths as static sources, 3 lattice sites per chair unit;
- 3 full tiles above (flow time 0) and 3 Hat tiles below (flow time 1), on a triangle of radius 4; 4 groups on +-x, +-y at distance 8 (the faces layout);
- tiles as density: c^2 rho = 1.5 on the six tile sites;
- ten bits, bit i a source kappa h(tau_i) s_i with h = tan(omega0 (1 - tau) / 2), kappa = 1 (an area unit is a charge unit).

## 2. Results

| id | statement | label |
|---|---|---|
| E5.15 | A bit flip at step 10 of 24, then the time-reversed run with the reversed source schedule, returns the data to 3e-15. Reversing without the flip fails (error 0.8). The flip enters only through the next step: psi(n+1) differs by exactly dt^2 c^2 rho (B - A) | Exact |
| E5.16 | The Hat layer is silent: a Hat flip changes the source and the field by exactly 0; a full-tile flip changes the source by 2 sqrt3 = 3.464. The strength 2 h(tau) falls strictly along the flow to 0 | Exact |
| E5.17 | **No bit pattern of the ensemble is neutral at tau_group = 0.** Seven live bits of equal strength (3 full + 4 groups; the Hat's three are silent) give an odd multiple of sqrt3: the minimum |monopole| is sqrt3, reached by 560 of 1024 patterns. Exact neutrality appears only at isolated group flow times (h = sqrt3/2 at 0.3184, sqrt3/4 at 0.6098) | Exact (counting) |
| E5.18 | The four group bits make a quadrupole with xx and yy entries only. Largest overlap with the chair's: cos = 0.8165 = sqrt(2/3) (to 1e-4), by (+,-,+,-) and its negative, which have no group monopole and no group dipole. They can cancel the chair's xx - yy part (2/3 of its squared norm) at kappa = 1.58, never the xz, yz part (residual 0.577 = 1/sqrt3 of the norm) | Exact / Numeric |
| E5.19 | Settled field of a full-tile flip at r = 8: 1.0015 of q G_3(r) (regression of v2 E5.6) | Numeric |

## 3. What this says

1. **v4's pair rule cannot hold with the ensemble as counted.** The pair rule makes the net source of the bits zero; with a Hat layer that is silent the live bits are seven of equal strength, an odd number. At tau_group = 0 the leading far field of the whole is therefore a 1/r monopole from the bits, not the chair's r^-3 quadrupole (a **Negative** for "the ensemble is neutral like the chair"). It is neutral only if the groups sit at a different flow time from the full tiles, at isolated values.
2. **The Hat layer is a clean frozen end.** Its bits are readable by nothing: they do not change the field. This is the v2 reading (C7: Hat = unreadable) in a form that can be checked, and it costs the Hat layer its role as a source.
3. **The bits can compensate part of the chair, and only part.** A pattern with +x, -x up and +y, -y down reproduces the chair's xx - yy quadrupole; the xz, yz part needs groups on the +-z sides, which the faces layout leaves empty (the spread layout has them). The needed strength, kappa = 1.58, is of order one; that is not derived.
4. **Reversibility survives the coupling** once the flip is a stated term (E5.15).

## 4. Not done

The dynamics of the bits (they are set by hand, not driven by the field); the readability c(tau) of the bits at the ensemble's own field (v2 Stage 5 rules I and II); the arrow audit (Stage 4); a coupling of the chair's charges to the bits other than through the common field.
