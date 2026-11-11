# Stage 5: results (the sheet as a wave field, snaps, and the clock; the paper's open problem 13)

Labels as in `PLAN.md`. Nothing here is gravity; "clock" means the factor √(1 − r_s/r) of the companion model. File: `tm2_waves.py` (9 tests, a few seconds), figure `figures/tm2_stage5_waves.png`.

```
python3 research/toymodel_v2/tm2_waves.py --tests-only
python3 research/toymodel_v2/tm2_waves.py --story
python3 research/toymodel_v2/tm2_waves.py --no-show --save research/toymodel_v2/figures/tm2_stage5_waves.png
```

## 1. What was built

The paper's wave equation with the damping off, `ψ_tt/(c²ρ) − ∇²ψ = source`, on a periodic lattice in 1, 2 and 3 dimensions with a leapfrog scheme. A snap of the bit is a step in the source at one site (the tile's area changed and stays changed); a pair snap is +q and −q on neighbouring sites (the bump-and-dent pair of Stage 0). The static field of a source is then tied to the bit's separation d by a coupling rule, and the overlap `c = exp(−d²/8σ²)` is compared with the clock.

## 2. Results

| id | statement | label |
|---|---|---|
| E5.1, E5.2 | The scheme conserves its discrete energy to 1e-12 and retraces backwards to 1e-10, in 1, 2, 3 dimensions, with a density profile and a static source | Exact |
| E5.3 | **The equation's wave speed is c√ρ: waves are faster where the density is higher.** Measured group speeds match the discrete dispersion relation (0.970 and 1.947 for ρ = 1 and 4); a pulse that crosses a slab of ρ = 4 comes out ahead | Exact; **Negative** for the paper's "high-density regions induce phase lag" |
| E5.4 | Reflection and transmission at a density step equal the discrete theory (for ρ 1 → 4 at k = 0.5: 0.118 reflected) to 2e-3, summing to 1 | Exact / Numeric (finite pulse) |
| E5.5 | Causality: after n steps nothing has changed beyond lattice distance n; exactly zero | Exact |
| E5.6 | **A snap radiates a front and leaves the static field behind it.** In 3D the settled field equals the lattice Green function within 0.5% (r = 2 to 8) and the front's height falls like r^−1.09. A pair snap has no net source: its settled field is 0.16 of a single snap's at r = 2 and 0.006 at r = 8. Energy including the source's work is conserved and the run retraces: the radiation is reversible | Exact / Numeric |
| E5.7 | The static field of a point source is linear in 1D, logarithmic in 2D and 1/r in 3D, on the lattice. **A 1/r profile needs three dimensions** | Exact (1D), Numeric 2% (2D, 3D) |
| E5.8 | Clock identities, by coupling rule (table below) | Exact algebra of a Definition |
| E5.9 | Plasticity per coordinate time is the per-tick value times the clock factor. Reading B: 1 − r_s/r. Reading A: √((r_s/r)(1 − r_s/r)), zero at both ends, largest (½) at r = 2r_s | Exact algebra |

Coupling rules (all assume the *squared* jump is linear in the field):

| rule | dimension | d² | overlap | against the clock | dense region |
|---|---|---|---|---|---|
| II: d² ∝ drop of the field from r_s | 2 | 4σ² ln(r/r_s) | c² = r_s/r | **exact**: √(1 − c²) = √(1 − r_s/r). The plan's C7, reading A | plastic per tick (the Hat is at r_s) |
| I: d² ∝ the field | 3 | 4σ² r_s/r | c² = exp(−r_s/r) | first order only: c exceeds the clock by 10% at 2r_s, under 1% beyond 7.5r_s. Reading B. No horizon | frozen |
| linear: d ∝ the field | 3 | 4σ²(r₀/r)² | 1 − c² ≈ (r₀/r)² | **fails**: 1/r² fall-off | frozen |
| II in 3D | 3 | bounded | c² → e^(−const) | **fails**: a floor on c | |

## 3. Against the plan's failure conditions

- *"The profile is not logarithmic."* It is logarithmic on a 2D sheet, and then C7 holds exactly. In 3D it is not, and the identity is first order.
- *"The floor on c prevents the identity."* True for rule II in 3D.
- *"Dense regions come out plastic."* True under rule II (per tick; per coordinate time they are frozen at both ends), false under rule I.

So Stage 5 does not select a reading. It reduces the choice between A and B to two stated things: the dimension of the sheet, and whether the squared jump follows the field or its drop from a reference radius. The tile and its lift are 2D objects, which favours rule II and reading A; "past nearest the particle" (`ARROW_OF_TIME.md` section 7), the paper's "dense regions resist reordering" and a 1/r profile all need rule I, which needs three dimensions.

## 4. Consequences

1. The paper's wave equation and its text disagree about density (E5.3). Either the equation should have ρ multiplying the time derivative term (`ρ ψ_tt/c² − ∇²ψ`), or the text should say dense regions speed waves up.
2. Open problem 13 (a 1/r profile and a horizon): 1/r is available from a 3D sheet with rule I, to first order, without a horizon; an exact clock with a horizon is available from a 2D sheet with rule II, without 1/r. Not both.
3. A snap is a source of radiation and of a static field; a pair snap is a dipole. With the pair rule of Stage 0, isolated bits on the tiling would radiate as dipoles only.

## 5. Not done

Why d², not d, should be linear in the field; back-reaction (snaps are prescribed, not triggered by the field); ρ depending on ψ; the tiling instead of a cubic lattice; the damping and the paper's update probability 1/ρ; Stage 6.
