# Stage 0: results

Labels as in `PLAN.md`: **Exact** (finite check with a test that can fail), **Definition**, **Hypothesis**, **Negative**. Nothing here connects to gravity.

Files: `core.py` (7 tests), `tm2_lattice.py` (9 tests), `figures/tm2_stage0_flat_pairs.png`, `audit_v1.py`. Reproduce:

```
python3 research/toymodel_v2/core.py --tests-only
python3 research/toymodel_v2/tm2_lattice.py --tests-only          # about 14 s
python3 research/toymodel_v2/tm2_lattice.py --story --levels 4
python3 research/toymodel_v2/tm2_lattice.py --no-show --save research/toymodel_v2/figures/tm2_stage0_flat_pairs.png
```

## 1. What was done and the outcome in one paragraph

The baseline (`core.py`) reproduces the ledger's closed-timeline counts for both alphabets and agrees with brute force. The two lattice checks of the plan turned out differently from what the plan said. **C2 holds, in a stronger form than stated:** the Spectre, the Hat and everything between them are projections of *one* integer lift, so the discrete structure (lifted vertices, bond graph, E₈ image) does not change at all along the flow, and what changes is the relative scale of the red and green planes. **C1 was wrong as written** (section 3). **A new result:** the one thing the tiling does not allow is what the chain calls the bit. Opening the flat vertex of every tile alike breaks 22% of the seams of a level-2 patch, and the tiling permits the opening only in antiparallel *pairs* of tiles whose flat vertices coincide.

## 2. Baseline (`core.py`)

| result | label |
|---|---|
| closed timelines, n = 0..8, hexagonal: 1, 0, 6, 12, 90, 360, 2040, 10080, 54810; cubic: 1, 0, 6, 0, 90, 0, 1860, 0, 44730; dynamic programming equals brute force for n ≤ 8 | Exact (counts typed in from the QGD ledger, then checked against computation) |
| the hexagonal alphabet is the red A₂ of `lift4d.step`; e₀ − e₁ + e₂ = 0; neighbours at 60° have planar inner product ½ | Exact |
| the cubic set has no odd closed timeline and \|flux\|² has the parity of n; the hexagonal set has 12 closed 3-step timelines | Exact |
| the tests detect a wrong step set (a different alphabet changes the counts); a sheared basis of the same lattice does not, as it should not | Exact (mutation checks) |

## 3. The lattice, the flow and the corrected C1 (`tm2_lattice.py`)

| id | statement | label |
|---|---|---|
| E1 | Each of the 994 edges of the 71 tiles of the level-2 patch is one unit step of the lift lattice and an E₈ root (norm² 2); 63 tiles have 8 red + 6 green edges, 8 have 6 red + 8 green | Exact |
| E2 | The projection P_{1,b} of the lifted patch equals `SpectrePatch(2, 1, b)` edge by edge (error ≤ 1e-14) for b = 1, 1.2, 1.5, √3, 2 | Exact (numerical, tolerance 1e-9) |
| E3 | The bond graph (406 seams) and the number of boundary edges are identical for all those b; the two edges of every seam have equal length | Exact |
| E4 | At the Hat (b = √3) all 518 vertices lie on the hexagonal lattice Z[w] (rank 2) though the lift has rank 4: the map Z⁴ → Z[w] has a kernel of rank 2. Through that physical lattice the long edges have E₈ norm² 6 (not roots); through the lift all edges are roots. Distinct lifted vertices stay distinct | Exact |

**Correction to C1 of `PLAN.md`.** The plan said "the Hat's long edges are not roots and intermediate tiles are not in the lattice, so the lattice sees only the endpoints". That mixes two readings. In the lift (Cheritat–Ma), which is the reading in which the paper embeds the Spectre in E₈, *every tile of the family, Hat included, is the same lattice walk of roots*; the deformation acts on the projection to the plane, scaling the red plane by a and the green plane by b. The "norm² 6" statement is true only for the other reading (positions in the plane read as elements of Z[w]), which exists only at b = √3, and there it records a rank collapse 4 → 2. This is better than what the plan had: D (the lift) is flow invariant, and the flow parameter is the ratio of the scales of two E₈-orthogonal sectors, the colours that C3 identified. *Hypothesis:* those two scales are the "independent time per axis" of the paper's section on dynamic axes; nothing here tests that.

**The straight pass is already in the Spectre.** The 14-gon has exactly one flat vertex for every (a, b) (index 8 in `spectre_operator`, `FLAT` in the code), as the paper's "one straight pass" says. The flow model's *opening* of that vertex by h(τ) is an addition of the chain, not part of the family. That is where the next section comes from.

## 4. What the tiling says about the bit (E5 to E7)

| id | statement | label |
|---|---|---|
| E5 | The flat vertex of a tile coincides with (i) the flat vertex of exactly one other tile (a **flat pair**; always two Spectre-shaped tiles), or (ii) non-flat vertices of other tiles, of types (0,12), (10,10), (4,12), and for every Mystic tile (10,12), or (iii) nothing (patch boundary). Level 3: 82 flat pairs (164 tiles of 559). Level 4: 804 pairs (1608 of 4401). Fraction of interior tiles in a pair: 24.0%, 34.9%, 40.0% at levels 2, 3, 4 (42.4% at level 5, from an exploratory run not in the tests) | Exact (finite) |
| E6 | Opening the flat vertex of every tile alike (any h > 0, either sign, b = 1 or √3) breaks 88 of the 406 seams of the level-2 patch, all at edges touching the flat vertex | Exact (finite) |
| E7 | Opening a flat pair as one bump and one dent (opposite signs in the outward-normal convention) closes every seam and leaves the total area unchanged, for all 82 pairs of level 3, at b = 1 and b = √3. Two bumps (or two dents) break exactly 2 seams and change the area by ±2 a h. Opening a tile outside a flat pair breaks at least 2 seams (checked on 60 tiles) | Exact (finite) |

**Reading, as Hypothesis.** The opening h at the flat vertex is the chain's bit (convex or reflex at P₈). The tiling then supplies a compatibility rule that nobody chose: a bit can flip only on a flat pair, the two bits of the pair are antiparallel, and the net area `Q = Σ s h a` of each pair is zero. That is the same quantity as the chain's holonomy ledger `Q = Σ bⱼ wⱼ`, here forced to zero by seam closure. About 40% of interior tiles are in such pairs, one free bit per pair (about 20% of tiles); the remaining tiles are frozen. Two caveats: the check is on finite patches (levels 2 to 4); and a flat vertex that meets non-flat vertices could open if its neighbours changed shape too, which the identical-copy assumption excludes.

Figure: `figures/tm2_stage0_flat_pairs.png` (left: the level-3 patch coloured by type; middle and right: one pair opened as bump + dent, closed, and as bump + bump, with the overlap).

## 5. What this changes in the plan

- **C1** is replaced by the corrected statement above; C2 is stronger than the plan had it; the plan text is updated.
- **Stage 1 gets a second candidate for P4.** Besides the flux constraint J of the paper's action, the tiling gives a *seam* constraint: signs on flat pairs are antiparallel, other tiles are frozen. Stage 1 should compute both and compare them (is the seam constraint a special case of a flux constraint, an independent one, or incompatible?).
- **Number of free bits is now a quantity.** The plan treated a node at every site. On the tiling it is about 20% of tiles, in pairs. This is a structural fact about the aperiodic substrate, and it should be kept in view when the plan talks about "scales".
- **Not done in Stage 0:** whether the pairs form a larger structure (chains or clusters of flat vertices across levels), and the limit of the pair fraction as the level grows. The sequence 24.0, 34.9, 40.0, 42.4% is rising, with no limit claimed.

## 6. Status of the plan's Stage 0 list

| item | result |
|---|---|
| `core.py`, counts equal the ledger for both alphabets, n ≤ 8 | done, passes |
| C1: edge roots and E₈ norms at τ = 0 and τ = 1 | done; the plan's claim was wrong and is corrected |
| C2: bond graph along the flow | done; invariant for the whole family b ∈ [1, 2]; fails only for the opened tile (E6), which is outside the family |
| "fails if the bond graph changes along the flow" | did not fail; a different failure was found (E6) |
