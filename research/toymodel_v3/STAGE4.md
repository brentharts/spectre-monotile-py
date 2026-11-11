# Stage 4: results (a read-out in which the setting acts on the state)

Labels as in `PLAN.md`. File: `t3_qubit.py` (4 tests, about 1 s).

```
python3 research/toymodel_v3/t3_qubit.py --tests-only --story
```

## What was built

- Each end letter (a direction 60k°) becomes an equatorial spin-½ state `(|0⟩ + e^{iφ_k}|1⟩)/√2`.
- Each setting becomes the observable `cos a σ_x + sin a σ_y`.
- The shared origin is a *coherent* superposition a_v; the two wings are unitaries on the letters.
- No run is discarded, so the Clauser–Horne inequality and CHSH coincide, and Tsirelson's bound holds by construction (Tsirelson's theorem).

## Results

| id | statement | label |
|---|---|---|
| E4.1 | Nothing discarded; no marginal depends on the far setting; CHSH ≤ 2√2 for 500 random states | Exact (a check of the code) |
| E4.2 | With no wing, the state's coefficients are Fourier components of the origin: `χ = ½(A₀|00⟩ − A₁|01⟩ + A₁|10⟩ − A₂|11⟩)`, `A_m = Σ a_v e^{imφ_v}`. A uniform origin (the paper's shared axis made coherent) gives the **product** state \|00⟩. A phase winding of −1 gives the **singlet** exactly. A winding of +1 cancels | Exact |
| E4.3 | For the singlet, E = −cos(a − b) exactly. The best CHSH is 5/2 on the six hexagonal settings, 1 + √3 = 2.732 on twelve settings, and 2√2 on 24 settings (15°) | Exact |
| E4.4 | **The symmetry behind it.** If the origin amplitude is odd under v → −v (`a_{v+3} = −a_v`), the state is antisymmetric under exchange for *any* wing the two parties share, hence the singlet (200 random cases, fidelity 1). An even origin has no singlet component at all. Different wings break it (mean fidelity 0.26). A classical (probability) origin gives a separable state, CHSH ≤ 2 | Exact |

## What it says

1. **The quantum range is reached without discarding anything.** The route of Stage 2 (post-selection) is replaced by observables, as the plan asked.
2. **The singlet comes from one condition on the shared origin.** It must be odd under inversion. This is the paper's anti-correlated fusion `S_B(0) = −S_A(0)`, made coherent with a relative sign −1 between an origin and its opposite: an antisymmetry of the kind fermions have. Stiffness, wing length and the two-ended conditioning play no role.
3. **Without a wing, the sense of the winding has to be opposite to the read-out map's.** This is a relative handedness; the mirror image of both is equivalent.
4. **The toy does not produce the entanglement; it is put in as the sign.** "Fails if no parameter gives a state close to the singlet" is passed only in this sense: the parameter is the parity of the origin, chosen, not derived. **Hypothesis:** that a phase vortex at the fusion point supplies it, from the area (flux) term of `t3_area.py` or a holonomy vortex of the Spectre.
