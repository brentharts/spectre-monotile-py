# Stage 2: results (Chair44 as a neutral source)

Labels as in `PLAN.md`. Nothing here is gravity. File: `t5_source.py` (4 tests, about 2 s), figure `figures/t5_stage2_source.png`.

```
python3 research/toymodel_v5/t5_source.py --tests-only
python3 research/toymodel_v5/t5_source.py --no-show --save research/toymodel_v5/figures/t5_stage2_source.png
```

## 1. What was built

The 192 pyramid tips as point charges q = j (bump +j, dent -j) at their apexes, about the chair's centre (1, 1, 1). The field is the lattice-free Coulomb potential, the three-dimensional Green function of `toymodel_v2` (E5.7). Reading the depth as a charge is a **Definition** of the toy.

## 2. Results

| id | statement | label |
|---|---|---|
| E5.11 | Monopole 0 and dipole 0, exactly (Fractions); 8 bumps and 8 dents of each depth. tr Q = 91/50; the traceless quadrupole has eigenvalues -47.817, 0.104, 47.713 (a saddle). Its part axial about the chair's axis (1,1,1) is 0.4% of its norm, and the eigenvector of the small eigenvalue is exactly perpendicular to the axis | Exact |
| E5.12 | The tip positions are invariant under all six symmetries of the chair. The charges are not: the exchange x <-> y reverses all 192; the other two exchanges reverse 144 of 192; the two three-fold rotations keep 144 of 192. The charge pattern is odd under one reflection of the chair and only 3/4 invariant under its three-fold axis | Exact |
| E5.13 | Far field: the potential tends to the quadrupole term (r^3 phi = 39.4 at r = 64) with an r^-4 residual; one charge falls as r^-1 and a pair as r^-2 (fitted slopes -1.00, -2.00) | Numeric |
| E5.14 | Four bits of the 14th-vertex groups have zero net source for exactly 6 of 16 patterns (two and two): the bump-and-dent pairing of `toymodel_v4`. Then the ensemble is neutral at leading order like the chair | Exact (counting) |

## 3. What this says

1. **The chair emits no 1/r and no 1/r^2 field.** Its leading field is a quadrupole falling as r^-3. In the language of v2 Stage 5 ("a snap leaves the static field of a source; a pair snap is a dipole with almost no net field") the chair is neutral at two orders.
2. **The leading field is not axial.** A chair with a three-fold axis would give an axial quadrupole; the depth assignment gives a saddle that sits almost entirely off the axis. The tile plane of Stage 0 is perpendicular to that axis, so the tile sees a quadrupole that has no component along its normal. The odd parity under x <-> y is a fact about the data, not an explanation.
3. **A net 1/r source can only come from the bits.** Each tile bit moves the area by +-a h; a charge-like 1/r field needs an unbalanced bit pattern, and the pair rule of v4 forbids it. So in this toy the ensemble has no leading monopole unless the pair rule is broken; whether that matches anything is not tested.

## 4. Not done

The field on the v2 wave lattice (here the continuum Coulomb potential only); a coupling between the bits and the chair charges; the dynamics (Stage 3 and later).
