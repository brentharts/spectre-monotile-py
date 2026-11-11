# Stage 4: results (a read-out in which the setting acts on the state)

Labels as in `PLAN.md`. File: `t3_qubit.py` (6 tests, a few seconds).

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
| E4.5 | **A flux cannot supply the sign.** Build the two-wing amplitude Ψ[s_A, s_B] from stiffness and a complex area phase e^{iθK} (K the signed area of the joined path). Any amplitude invariant under a common rotation of both ends gives exactly \|00⟩ under this read-out, with singlet fidelity 0 (n = 1, 2; β = 0.5, 2; seven values of θ) | Exact (Negative for the hypothesis below) |
| E4.6 | **A spinor read-out of scalar amplitudes is not well defined.** Using half-angle kets (e^{−iφ/2}, e^{iφ/2}) needs a sign for each of the six letters. Over the 64 choices, an odd winding gives fidelity 1 (48 choices) or a zero state (16); the flux amplitude at n = 1, β = 0.5, θ = 2.75 gives 0.017 to 0.93. Flipping letter u's sign equals flipping a_u and a_{u+3}, which keeps an odd origin odd. The principal branch happens to give the singlet for every odd winding; that is a gauge choice, not a result. A consistent version needs spinor-valued amplitudes (a spin structure, e.g. the Dic₁₂ holonomy of the Hat and Spectre repository); not built | Negative |

## What it says

1. **The quantum range is reached without discarding anything.** The route of Stage 2 (post-selection) is replaced by observables, as the plan asked.
2. **The singlet comes from one condition on the shared origin.** It must be odd under inversion. This is the paper's anti-correlated fusion `S_B(0) = −S_A(0)`, made coherent with a relative sign −1 between an origin and its opposite: an antisymmetry of the kind fermions have. Stiffness, wing length and the two-ended conditioning play no role.
3. **Without a wing, the sense of the winding has to be opposite to the read-out map's.** This is a relative handedness; the mirror image of both is equivalent.
4. **The toy does not produce the entanglement; it is put in as the sign.** "Fails if no parameter gives a state close to the singlet" is passed only in this sense: the parameter is the parity of the origin, chosen, not derived. The hypothesis that a phase vortex supplies it was tested. A flux (the area term of `t3_area.py` as a complex phase) cannot, under this read-out: it is rotation invariant and gives a product state (E4.5). Reading scalar amplitudes as spinors makes the answer depend on an arbitrary sign per letter (E4.6).
5. **What is left.** **Hypothesis:** the sign comes from amplitudes that are spinor-valued from the start, so that a 2π turn of the end letter gives −1 (a spin structure on the letters, such as the Dic₁₂ holonomy of the Spectre). That is a change of model, not a parameter, and it is not built.
