# Stage 5: results (Lorentz: isotropy with scale, and the moving clock)

Labels as in `PLAN.md`. File: `t7_lorentz.py` (4 tests, about 13 s), figure `figures/t7_stage5_lorentz.png`.

```
python3 research/toymodel_v7/t7_lorentz.py --tests-only
python3 research/toymodel_v7/t7_lorentz.py --no-show --save research/toymodel_v7/figures/t7_stage5_lorentz.png
```

## 1. Two forms of a wave

On a graph with Laplacian L there are two natural waves:
- **first order in time** (the hopping of v6): `i dpsi/dt = L psi`, with long-wave dispersion omega = k^T (M/N) k;
- **second order in time** (the leapfrog of v2): `u_tt = -(L + m^2) u`, with omega^2 = m^2 + k^T (M/N) k - Q4(k) + ...

Here M = sum_e e e^T and Q4(k) = sum_e (k.e)^4 / 12, per site, from the expansion of 2(1 - cos k.e). The graphs are the Chair44 tile graph of depth 6 (262144 tiles) and the Spectre vertex graph of level 4.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.18 | **Only the second-order form has a light cone, and the cone is isotropic at long wavelength.** The first-order form has group velocity proportional to k, so no finite speed. The second-order form has c^2 = 4.617 on the Chair44, isotropic to 5.6e-5 in a ball of radius 40; the anisotropy falls as the ball grows (7e-3, 1.4e-4, 5.6e-5 at R = 10, 20, 40) | Exact (sums over the graph) |
| E7.19 | **The first correction is cubic and shrinks as lambda^-2.** Q4/k^4 per site is 1.297 along an axis, 0.806 along a face diagonal and 0.643 along a body diagonal, so the axes are slowest: v_axis/v_body - 1 = -2.80/lambda^2 (-1.1% at 16 tile spacings, -0.11% at 50). This is the sign v6 found for quantum packets (axes slowest). That anisotropy did not shrink because a packet started on one site contains lattice-scale wavelengths | Exact sums; plane-wave estimate |
| E7.20 | **On the Spectre the fourth-order tensor is isotropic too:** sum (u.e)^4 varies by 0.37% over directions, so anisotropy starts at k^6 (lambda^-4). Six-fold statistics force it | Exact sums |
| E7.21 | **A moving clock runs slow by gamma exactly when the dispersion is relativistic.** The phase at a packet's centre advances at omega - k v_g, checked to 3e-4 by exact evolution on a periodic chain. For omega^2 = m^2 + c^2 k^2 this equals m/gamma exactly. On the chain lattice (m = 0.5) the clock follows 1/gamma to 0.03% at k = 0.2, 0.9% at 0.5, 3.4% at 1. The first-order form gives only the Galilean limit, with no rest frequency | Exact algebra; exact evolution |

## 3. What this says

1. **Lorentz invariance emerges at long wavelength, for the right kind of wave.** On the Chair44, a second-order wave has an isotropic light cone to 5.6e-5, and its corrections fall as (tile size / wavelength)^2. A moving clock built from such a wave (a massive packet's phase) runs slow by gamma to the same order. This holds on any graph whose walk tensor is isotropic; nothing here is special to the Spectre or the Chair44 beyond that.
2. **The quantum hopping of v6 is the non-relativistic form.** It has no light cone and only the Galilean limit of time dilation. Its anisotropy (v6 Stage 2) is the lattice-scale end of the same k^4 term, and it does not shrink because a packet started on one site is not a long wave. So the scoreboard's isotropy row for localized packets stays open, but it is now explained, and two new rows (the long-wave cone and the clock) are met.
3. **What would make it a Lorentz-invariant quantum theory.** A Hamiltonian whose single-particle dispersion is the second-order one: a two-component field (a Dirac or Klein-Gordon form), not a one-component hopping. That is a construction, not a result of the tile, and it is the next step on this line.
4. **What is not settled.** The direct packet runs on the depth-6 Chair44 were dominated by scattering off the tile structure at lambda <= 12 and by the boundary at lambda = 24, so the lambda^-2 law is a plane-wave estimate, not a measurement on the graph. The Spectre's sub-ballistic quantum transport (v6) is not addressed: whether a second-order wave on the Spectre graph is ballistic was not run.

## 4. Not done

A Dirac-type (two-component) hopping on the Spectre or Chair44 and its cone; a measured long-wave speed on a larger Chair44 supertile (depth 7) to test the lambda^-2 law with scattering included; the second-order wave on the Spectre graph.
