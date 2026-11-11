# Stage 4: results (a rule from a history weight; the paper's open problem 12)

Labels as in `PLAN.md`. Nothing here connects to gravity or models the singlet state. File: `tm2_rule.py` (11 tests, about 3 s), figure `figures/tm2_stage4_rule.png`.

```
python3 research/toymodel_v2/tm2_rule.py --tests-only
python3 research/toymodel_v2/tm2_rule.py --story
python3 research/toymodel_v2/tm2_rule.py --no-show --save research/toymodel_v2/figures/tm2_stage4_rule.png
```

## 1. What was built

A two-wing history: an origin letter v (one of the six hexagonal roots), a wing of n letters to Alice starting at v and one to Bob starting at −v, each weighted by the first term of the paper's action, `exp(−β Σ N(s(t) − s(t−1)))`. A setting is a direction; the outcome ±1 at an end has the boundary weight `exp(μ α a·s_end)` (hard pin `s_end = α a` in the limit). The table is `P(α, β | a, b) = W / Z(a, b)`: the ensemble of histories that satisfy both end conditions. All of this is **Definition**; the tables are then exact (transfer matrix, checked against enumeration).

## 2. Results against the four requirements of C6

| | requirement | result | label |
|---|---|---|---|
| (a) | the AB form, or another, comes out | **Another form.** The past is weighted by a product `g(v, a) g(−v, b)` (rank 1 over the pairs of settings) and the read-outs are soft; the rule's `exp(κAB)` has rank 2 and cannot be written that way. For hard pins `E(m) = −[2r₁cos(πm/3) + (−1)^m r₃] / [1 + 2r₂cos(2πm/3)]`, `r_k = (λ_k/λ₀)^(2n)` | Exact (E4.2, E4.4) |
| (b) | κ fixed by model parameters | **Yes, there is no free κ**: the strength is set by stiffness β, wing length n and boundary coupling μ. Hard pins, n = 1: CHSH passes 2 at β = 0.841 and 2√2 at β = 1.255, tends to 4 for stiff wings (E → −sgn(a·b), the paper's sign read-out) and to a cosine law of visibility 2r₁ < 0.8 for soft ones. Not tied to the overlap c of the chain | Exact (E4.5); the link to c is still a Hypothesis |
| (c) | exact no-signalling for the twelve read-outs | **Yes**, for every β, n, μ, to 1e-14, because the weight is invariant under inverting everything; there are no zero outcomes. A bias that breaks the inversion signals (Alice's marginal 0.535 to 0.571 over Bob's settings) | Exact (E4.3) |
| (d) | the normalisation is stated | It is a **conditioning on both end conditions**. The pass weight `Z(a, b)` depends on both settings: contrast `(1 + 2ρ)/(1 − ρ)`, 1.21 where CHSH = 2 and 1.68 where CHSH = 2√2. The outcomes carry no signal; the pass rate does | Exact (E4.7) |

Further results:

| id | statement | label |
|---|---|---|
| E4.4b | **The paper's rule depends on how Bob's outcome is named.** Describing the same measurement by −b with the outcome flipped maps κ → −κ; so `E_rule(180° − θ) ≠ −E_rule(θ)` (κ = 0.5: +0.152 at 60°, +0.689 at 120°). The derived table is covariant. The rule's CHSH values 2.257, 2.494, 3.068, 3.620, 3.946 are reproduced | Exact |
| E4.6 | The price `CHSH ≤ min(4, 2 + 12δ)` holds on the grid and is overpaid: CHSH exceeds 2 only for δ ≥ 0.14 (the bound would allow it for any δ > 0) | Numeric (grid) |
| E4.8 | δ, the contrast and CHSH fall strictly with wing length (β = 2: δ = 0.315, 0.264, 0.141, 0.030 at n = 1, 2, 4, 8) | Exact |
| E4.9 | **4b, complete read-outs:** with any unitary wings the table is a classical distribution of the two ends with local read-outs, so CHSH ≤ 2. Complex amplitudes alone do nothing | Exact |
| E4.10 | **4b, post-selected read-outs:** a quantum walk on the letters gives CHSH up to 4 (above 2√2 for 36% of walk times). No bound at 2√2 | Exact; Negative for a Tsirelson bound |
| E4.11 | Check entropy of the chain: the 4096 bit histories carry 0.216 bit (1.16 effective histories), 95% of it in the last three ticks | Exact |

## 3. What this says

1. **A rule does come out of a history weight, and it is not the paper's.** Requirements (b), (c), (d) are met; (a) is met with a different form. The plan's failure conditions are not triggered.
2. **Everything rests on conditioning on both ends.** With it, any stiff shared history violates CHSH, up to 4; without it (complete read-outs, real or complex), none does. So the model's "retrocausality" is exactly the statement that the ensemble is selected by a two-ended check, and its measurable footprint is the pass rate, which depends on both settings.
3. **There is no quantum bound here.** 2√2 is a point the family passes through. A bound would need the setting to act on the state before the read-out; no such read-out is built.
4. **The paper's Result on the rule needs a correction**: the rule as stated is not covariant under renaming an outcome (E4.4b), so κ has a sign convention hidden in it.

## 4. Not done

The link between (β, n, μ) and the chain's overlap; a flux term on the wings; wings on the tiling; a read-out with non-commuting settings; any model of the pass rate as a duration.
