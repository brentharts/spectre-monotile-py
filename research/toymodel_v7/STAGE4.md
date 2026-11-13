# Stage 4: results (the space factor from the tile)

Labels as in `PLAN.md`. File: `t7_space.py` (3 tests, about 3 s), figure `figures/t7_stage4_space.png`.

```
python3 research/toymodel_v7/t7_space.py --tests-only
python3 research/toymodel_v7/t7_space.py --no-show --save research/toymodel_v7/figures/t7_stage4_space.png
```

## 1. The question

A clock alone bends light by r_s/b, half of general relativity (paper, Result `res:bend`). For a wave in a massless band the coordinate speed is (hops per unit time) x (coordinate length per hop), so the index is n = 1 / (c l / l_inf) (Definition D7.7). v6 Stage 4 fixed the rate: the seam rate is c, with c = exp(-r_s/2r) under rule I. The factor 2 then needs l = c: a hop that gets shorter near the source in proportion to the overlap. Two readings of how the field reaches the tile:
- **S1:** the field sets only the bit's opening h;
- **S2:** the field sets the whole flow time tau, so the tile's edge b changes too.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.12 | **The bit lengthens its own edges by the inverse of its overlap.** The two edges at P8 have length sqrt(1 + h^2), and c = exp(-h^2/2), so l c = 1 - h^4/4 + O(h^6): l = 1/c to first order. The area ledger keeps a pair's mean area fixed, so no space factor comes from area | Exact |
| E7.13 | **Reading S1 reduces the bending.** Clock only: 1.00. The needed l = c: 2.00. Every bond behaving like a P8 edge: 0.00 (the clock cancelled exactly). On the patch the open bonds are f = 2.0%, 3.8%, 5.0% of all bonds at levels 2 to 4, and the coefficient is 1 - f = 0.950 at level 4 | Numeric, **Negative** |
| E7.14 | **Reading S2 has the wrong order.** Near the Hat 1 - c goes as (1 - tau)^2 while b changes linearly in 1 - tau, so n - 1 goes as sqrt(r_s/r) and the deflection falls as b^-1/2 (local slope -0.53), above the GR value at every b tried | Numeric, **Negative** |

## 3. What this says

1. **The tile's geometry points the wrong way.** The only length the bit changes is the length of its own two edges, and it changes as the inverse of the overlap. So the space factor the tile supplies, if any, undoes the clock instead of doubling it. Read naively, a world made only of open P8 edges would not bend light at all.
2. **The light-bending row of the scoreboard stays at 1.** Neither reading gives 2; S1 gives slightly less than the clock alone, S2 gives a different power law. The factor 2 still needs a spatial coupling the model does not contain (as the paper says), and Stage 4 adds that the tile's own geometry has the opposite sign.
3. **What would change the answer.** A length per hop that shrinks with the separation of the bit's two states. In the toy that would need either a different reading of "coordinate length" (hops as proper length instead of the embedding), which turns the P8 result into B = A and zero bending, or a new term. Both are choices; neither is in the tile.

## 4. Not done

A direct ray simulation on a honeycomb (massless) lattice with H = c(x) H0 (here the eikonal is a Definition and the quadrature of v3 does the optics); a reading in which proper length is counted in hops.
