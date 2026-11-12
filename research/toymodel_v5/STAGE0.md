# Stage 0: results (the alphabet, the cube and the chair axis)

Labels as in `PLAN.md`. File: `t5_alphabet.py` (5 tests, about 1 s), figure `figures/t5_stage0_alphabet.png`.

## 1. What was built

The six points that the bit of the 14th vertex touches are read as an alphabet. Relative to P8 they are (0, +-a) and (+-k, +-a/2): three antipodal pairs, whose frame operator is diag(2k^2, 3a^2/2). The cube is the chair's six sides. All results are algebra of the tile model and a cube; none is physics.

## 2. Results

| id | statement | label |
|---|---|---|
| E5.1 | Equal length, regular hexagon (the six roots of an A2), tight frame (frame operator a multiple of the identity) and k = sqrt3 a/2 are the same condition. For smaller k the alphabet is elongated along the chords with anisotropy 4k^2/(3a^2); at k = a/2 it is 1/3 (45 degree steps, lengths a and a/sqrt2) | Exact |
| E5.2 | At k = sqrt3 a/2 the six points are the orthographic image of the cube's six sides. With the 120 degree choice of plus ends the viewing direction is +-(1,1,1)/sqrt3 for every assignment of the pairs to x, y, z (even orders +, odd orders -). The boundary-order rule of `chair_fibers_14th.py` gives (1,-1,-1)/sqrt3. No orthographic view exists for other k | Exact |
| E5.3 | The seven-cube chair is invariant under the three-fold rotation about (1,1,1) and under none of the three-fold rotations about the other body diagonals. So the 120 degree choice puts the tile plane perpendicular to the chair's own axis; the boundary-order rule does not | Exact |
| E5.4 | Along the flow with k = k0(1 - tau) the anisotropy is (k/k_hex)^2, which is (1 - tau)^2 for k0 = sqrt3/2: a regular hexagon at the Spectre end, a line at the Hat. The outline with k0 = sqrt3/2 stays a simple closed curve along the flow (20 values of tau) | Exact; Numeric (simplicity) |
| E5.5 | The Gram matrix of the six directions is that of the A2 roots only at k = sqrt3/2 | Exact |

## 3. What this says

1. **The hexagonal alphabet of v2 is the shadow of the cubic one** at one value of the curvature. v2's dichotomy (hexagonal or cubic) is a choice of k, and the planar projection along the chair's axis connects them. The old statement "cubic-like at k = 1/2" is wrong: at k = 1/2 the points are at 45 degree steps with unequal lengths, not a cubic set.
2. **The orientation rule changes the answer.** The rule used so far for the group points (plus = later along the boundary) puts the tile plane perpendicular to a body diagonal that is not the chair's axis; the 120 degree choice puts it perpendicular to the axis. `chair_fibers_14th.py` is unchanged (it is already pushed); the 120 degree choice is defined here as an alternative.
3. **The alphabet degrades along the flow** like the square of the remaining curvature. So the six letters carry information only where the bit is readable, which matches the picture of the chain without being a derivation of it.

## 4. Not done

The dynamics on these letters (Stage 1); whether k = sqrt3/2 is reachable in the flow of toymodel_v4 (its range is k0 <= 0.8, and the outline is only checked to be simple here); the 3D tile placement of the full ensemble with the 120 degree choice.
