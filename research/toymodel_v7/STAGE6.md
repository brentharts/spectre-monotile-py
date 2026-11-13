# Stage 6: results (can the bits make a horizon?)

Labels as in `PLAN.md`. File: `t7_horizon.py` (3 tests, under 1 s), figure `figures/t7_stage6_horizon.png`. Stage 5 (Lorentz) is not run yet; the plan's order puts the two gravity stages, 4 and 6, first.

```
python3 research/toymodel_v7/t7_horizon.py --tests-only
python3 research/toymodel_v7/t7_horizon.py --no-show --save research/toymodel_v7/figures/t7_stage6_horizon.png
```

## 1. The question

A horizon in the toy is a radius where the overlap c reaches 0: the bit fully readable, the clock stopped. Two couplings tie c to a field:
- **rule I** (v2): c^2 = exp(-g phi), with phi the static field;
- **the escape coupling** (v3): c^2 = 1 - (probability that the walk reaches the source), which is 1 - R_eff/r on Z^3.

The bits are themselves sources of strength proportional to their opening, so the question is whether that feedback, or the tile, can give c = 0 at a finite radius.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.15 | **The tile has a floor.** On the flow range c >= c(0) = 0.2231. Extended past the Spectre, the opening tan(omega/2) diverges only at 180 degrees, but the reflex value stops being a tile at omega_c = 134.18 degrees (h* = (3 + sqrt3)/2). So c >= exp(-h*^2/2) = 0.0609 for every tile. The tile cannot represent c = 0 | Exact |
| E7.16 | **Rule I has no horizon at any back-reaction.** With a nonnegative source density, Gauss's law gives \|phi'(r)\| <= M_enc(R)/r^2, so phi is bounded at every r > 0 and c > 0 everywhere, for any coupling and any power of h. Numerically: alpha = 1e-4 and 5e-4 converge, the Gauss bound holds on the grid, and the cloud deepens the well (r phi at r = 0.5 of 1.44 and 11.4 against 0.5 without it). With alpha = 5e-4 rule I asks for c below the tile's floor on the whole cloud: the bits would saturate there | Exact (bound), Numeric (illustration); **Negative** |
| E7.17 | **The escape coupling meets the floor.** c^2 = 1 - R_eff/r reaches 0 at R_eff. With the tile's floor the bits saturate at r* = R_eff / (1 - c*^2) = 1.0037 R_eff, and inside it c stays at 0.0609. The horizon of v3 becomes a shell of maximally open bits just outside R_eff | Exact |

## 3. What this says

1. **The bits cannot make a horizon by themselves.** Under rule I the field stays finite at every r > 0 whatever the bits do (Gauss), so the overlap never reaches 0. The back-reaction can deepen the well a great deal, but not make it infinite at a finite radius.
2. **The tile cannot be at a horizon anyway.** Its reflex value stops being an outline at the critical angle, which puts a floor of 0.0609 on the overlap. This is a property of the Spectre's geometry, the same critical angle that the paper proves in closed form for straight edges.
3. **What a horizon becomes in the toy.** With v3's escape coupling, the place where c would reach 0 becomes a thin shell, 0.37% outside R_eff, where every bit is open as far as a tile allows. Inside it nothing changes: a frozen surface. That is a concrete, testable replacement for "horizon" in this model, and it is set by the tile, not by a parameter.
4. **The scoreboard's horizon row stays open**, and gains the floor 0.0609 as a second open row: a horizon would need either a coupling that is not rule I (the escape one, with c = 0 at the source surface) or a tile that can open past the critical angle, which the Spectre cannot.

## 4. Not done

The saturated shell as a boundary condition for waves (reflection, absorption); the field of the saturated region (bits at maximum opening are sources of fixed strength, so the cloud's mass stops growing with phi there).
