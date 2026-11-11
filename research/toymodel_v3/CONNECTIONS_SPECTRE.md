# Connections to the Hat and Spectre repository (`brentharts/spectre`, `research/`)

Read on 2026-11-10 from a clone at commit 9938874. I read the 78 files of `research/` by their docstrings, and the results recorded in `NEXT_STEPS.md`, `CLOCK.md` and `PROGRESS_SINCE_PR14.md`. Nothing was copied: the two repositories have different licences, and this one shares results and cross-references only, as `Readme.md` there says. One new computation was made here, in `t3_escape.py` E2.5. Labels as in `PLAN.md`; numbers from that repository are quoted as it records them, and are marked †.

## 1. The two connections that matter most

### 1.1 The chiral substrate is recurrent; the transient one is achiral

| | Spectre tiling | Chair44 |
|---|---|---|
| random walk (this repo, `t3_escape.py`) | **recurrent**: the probability of reaching the source tends to 1 like a planar lattice (E2.3); no 1/r | **transient**: h ∝ 1/r on the tile graph of the depth-5 supertile, flat to 5% for r = 5 to 18 (E2.5, new); rule (C) gives the clock and a horizon |
| handedness (that repo) | **chiral**: Im Z_k < 0 exactly for k ≤ 900, with a closed form whose leading term has Im A < 0 (`research/handedness.py`, `handedness_closed.py`)† | **achiral**: the substitution commutes with the x ↔ y mirror, a finite proof on the 168 labels (`research/chair44_mirror_proof.py`), so every pseudoscalar vanishes (X_k = 0 exactly for k ≤ 7, `chair44_handedness.py`)† |

This is an obstruction for the plan:

- Stage 1 found that 1/r and a horizon need a transient (3D) substrate.
- Stage 2 found that the arrow can be the substrate's handedness, through the area term θ = θ₀ε.
- In these two repositories, the substrate with a handedness is recurrent, and the transient one has no handedness. **Exact** for both facts; that no aperiodic monotile has both is not claimed.

There is a way through, and it can be tested. The *decorated* Chair44 (bump and dent pyramids) is invariant under PC but not under P alone (`chair44_pc_proof.py`: P maps the decoration D to −D, which is C(D), for all 192 features)†. So a pseudoscalar that is also odd under C, such as one weighted by the signed heights, is not forced to vanish. If one is nonzero, the 3D substrate can fix the sign of θ, but only together with the bump/dent labels: flipping every bump into a dent would flip the arrow.

**Result (item 6a, `t3_chair_handedness.py`, after this note was first written).** One is nonzero. The axial moment of the decoration, dotted with the chair's own polar vector, is ψ = −3/14 per tile, exactly. It is odd under P and under C, and has the same sign at every supertile level (χ_k = Nψ). So the obstruction holds for the bare geometry and is lifted for the decorated tile. That the decoration is physical is a **Hypothesis**.

### 1.2 Records need space, and the past needs proximity to change

`records_recognizability.py` (top level of that repository) asks how large a patch a tile must see to know its level-k ancestor†:
- about 2.4, 7.7 and 23 tile scales for 10% error at levels 1, 2 and 3;
- so each level costs about 3 times more radius, close to λ = 2.81;
- an observer of radius R can name its ancestors up to k_max ≈ log_λ(R/c·ℓ);
- going to finer scales is free, and going to coarser ones costs space.

`lookback_lca.py`† adds the dictionary "depth ≈ look-back": two tiles a distance d apart share their first common ancestor at level h ≈ 0.92 log_λ d + 1.72. It is generic for self-similar hierarchies (a quadtree does the same).

Together with `PAST_AND_POTENTIAL.md` of this repository, a consistent picture appears, though it is not tested as one model:

| | more space between the observer and the past | source |
|---|---|---|
| **knowing** the past | grows: more levels of ancestors become recognisable, about one level per factor λ of radius | records_recognizability† |
| **revising** the past | falls: the shift of the shared origin under a two-ended check decays as (λ₂/λ₀)ⁿ, and the bits that land on it go 1.32 → 0.011 from n = 1 to 8 | `tm2_checking.py` |

"Less space, more past checking; more space, more potential checking" then has a sharper form: space is what it costs to *read* the past, and proximity is what it takes to *change* it. **Hypothesis** as a statement about the world; each half is a computed fact about its own model. For Stage 3 (records as the compatibility rule), the recognizability radius is a ready-made record capacity for the tiling, to set against the chain's K(i, j).

## 2. Other connections, by plan item

| plan item (`PLAN.md`) | script in `brentharts/spectre/research` | what it gives | use here |
|---|---|---|---|
| 2.1 space factor B | `graph_dilation.py` (H1)† | graph distance ≈ 1.50 × Euclidean at large r, nearly isotropic (anisotropy 0.06, against 0.078 for the honeycomb and 0.22 for the square) | the tiling's own metric is a constant multiple of the Euclidean one: **the substrate supplies no B(r) by itself**, so the space factor needed for light bending has to come from the source, not from the tiling. It also justifies the Euclidean radii used in `t3_escape.py` at large r |
| 3.1 area term, sign ε | `handedness_closed.py`, `parity_clock.py` (top level), `outline_chirality.py`, `sheared_mirror_v1.json`† | two kinds of pseudoscalar. **Fixed sign:** Im Z_k < 0 at every level; the swirl ⟨sin(θ − φ)⟩ about a supertile is negative at every level (−0.427 … −0.030); tile orientation statistics do not alternate. **Alternating:** supertile outlines, the fitted shear M (off-diagonal ±0.49 with the parity of K) and the diffraction pattern mirror at each level | ε for θ = θ₀ε should be a fixed-sign one (Im Z_k), otherwise **the arrow would flip from one scale to the next**. A test: the relative sign of the outline's signed area K and Im Z_k on the same tiles; that product is mirror invariant, so it is a statement of the law, not a convention |
| 3.1 the form of the term | `flux_complex.py`, `flux_hamiltonian.py` (R8)† | U(1) flux π/2 at the Z₂ vortices breaks time reversal in the tile hopping model; every Bott index stays 0 at K = 3, 4 | θK is the same kind of term (a magnetic flux through the history), used in a different place. Their negative says a T-breaking flux has no topological spectral signature on the Spectre; the history-sum version is untested |
| 3.3 records | `records_recognizability.py`, `lookback_lca.py` (top level)† | §1.2 above | a record capacity for the tiling |
| 4.2 quantum read-out | `entanglement_h2.py` (H2)† | free fermions on the Spectre vertex graph: area law, mutual information decays the same against graph and Euclidean distance | a ready quantum state on the substrate; nothing Spectre-specific yet |
| 6.4 sources | `vortex_density.py`, `vortex_density2.py`, `diffraction.py`† | exact vortex density; the vortex set has Bragg peaks | vortices are a candidate set of point sources; on the planar tiling their field would be logarithmic (Stage 5 of v2), on the Chair44 it would need another defect |
| background density | `spin_densities.py`† | ρ₀ = (5 − √15)/5, the valence-4 density | already in `quantgrav.tex` |
| arrow audit | `barbour_complexity.py` (top level)† | Barbour's shape complexity of the tile centroids shows no clustering arrow beyond the footprint | agrees with Stage 3 of v2 |

Not connected to the v3 plan: the window and model-set studies, gap labelling, cohomology, doublers, Z₂ deconfinement, genus and ribbon studies. They concern the tiling's structure, not dynamics. The particle-mass scripts at the top level (`toymodel_v1.py`, `toymodel_v2.py` there, and the "TIS" plots) are not tested computations and are not used.

## 3. Additions to the plan

- **6a (new, small).** A height-weighted pseudoscalar of the decorated Chair44 supertile, odd under P and under C. If it is nonzero, the transient substrate can carry the arrow through its decoration (§1.1); if it vanishes for all levels, the obstruction of §1.1 stands. It needs `chair44_core.py` (present here) and the feature table `models/data/r44_features.csv` (present here).
- **3.1b (new, small).** The relative sign of the outline's signed area and Im Z_k: whether the law θ₀ can be fixed by the tile itself, and whether the chosen ε is a fixed-sign pseudoscalar.
- **3.3 (amended).** Use the recognizability radius as the record capacity of the tiling, next to the chain's K(i, j).
- **Stage 6 (started).** The Chair44 walk is transient (E2.5). Rule (C) on it gives 1/r, the clock and a horizon; whether its graph metric is isotropic, as the Spectre's is, is the next check.
