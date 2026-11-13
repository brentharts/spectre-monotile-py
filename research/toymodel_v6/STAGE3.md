# Stage 3: results (a Z2 gauge form on the Spectre patch)

Labels as in `PLAN.md`. Nothing here is physics. File: `t6_gauge.py` (5 tests, under 1 s), figure `figures/t6_stage3_gauge.png`.

```
python3 research/toymodel_v6/t6_gauge.py --tests-only
python3 research/toymodel_v6/t6_gauge.py --no-show --save research/toymodel_v6/figures/t6_stage3_gauge.png
```

## 1. What was built

The proposal turns the constraint into a plaquette term `g sum_p (W_p + W_p^dag - 2)` and says that `g -> infinity` gives entanglement with Tsirelson-bounded correlations. Here the gauge structure is the patch's own cell complex:
- **sites:** tiles;
- **links:** tile edges shared by two tiles;
- **plaquettes:** interior vertices of the tiling.

Z2 link qubits, Gauss law `G_t = prod X_l` at each tile, plaquette `B_p = prod Z_l` at each vertex, `H = -g sum B_p - h sum X_l`. With matter, a bit qubit on each tile enters the Gauss law. Everything is computed exactly in the GF(2) stabilizer algebra.

## 2. Results

| id | statement | label |
|---|---|---|
| E6.17 | The complex is a disc: T - L + P = 1 at levels 1, 2, 3 (9 - 40 + 32, 71 - 406 + 336, 559 - 3534 + 2976). Plaquettes have 2, 3 or 4 links (level 2: 254, 66, 16). The plan's stop condition (no plaquette structure) is not met | Exact |
| E6.18 | The pure theory is consistent: every Gauss law commutes with every plaquette. Ranks T - 1 = 70 and P = 336 add to L = 406, so the plaquette limit has a unique ground state (no degeneracy on a disc). Every tile meets every interior vertex in 0 or 2 links, so Wegner duality holds: on a disc the plaquette theory is a transverse-field Ising model on the interior vertices of the tiling | Exact |
| E6.19 | **The ledger is not the Gauss law.** At every flat pair (6 of 6 at level 2, 82 of 82 at level 3) the shared vertex P8 is a plaquette with exactly two links, and these are the only links the pair shares. The ledger `Z_A1 Z_A2` anticommutes with the Gauss laws of both tiles. Its gauge-invariant forms are the two Wilson lines `Z_A1 Z_e Z_A2` along the shared links, and their product is the plaquette `B_P8` | Exact, **Negative** |
| E6.20 | In the other frame it is a Gauss law without a gauge field. `-Z_A1 Z_A2` commutes with every term of the Stage 0 and Stage 1 Hamiltonian: one local Z2 symmetry per pair. After a Hadamard on the tiles it is a two-link Gauss law at P8. The pairs are disjoint, so these Gauss laws share no links: no cycles, no Wilson loops, no gauge dynamics | Exact |
| E6.21 | **The plaquette limit is entangled, but not pairwise.** In the unique ground state no link and no two links carry both an X and a Z stabilizer (394 two-link ZZ stabilizers, no XX). So every two-link reduced state is separable and its CHSH value is at most 2. The links left of the centre line (215) still carry 12 bits of entanglement entropy | Exact, **Negative** |

## 3. What this says

1. **The tiling supplies the complex, not the Gauss law.** Tiles, seams and vertices form an honest disc, and a Z2 gauge theory on it is consistent. But the Gauss law (X-type, at tiles) is a choice; nothing in the tile model fixes it.
2. **The area ledger sits in the gauge theory as flux, not charge.** The ledger is a Z-type condition. Expressed gauge-invariantly it becomes two Wilson lines across the two shared seams, whose product is the flux through the plaquette at the flat vertex P8. So the bit can be read as a flux at P8: one vertex, one bit, and the two-tile ledger is then an identity rather than a constraint. That is a cleaner bookkeeping, and the only structural link found between the bit and a gauge field.
3. **The proposal's g -> infinity claim fails here.** On a disc the plaquette limit is a single stabilizer state with many-body entanglement (12 bits across a cut) and no two-body Bell violation. The quantum bound is automatic for any state (Stage 0), and the projection does not produce correlations that violate CHSH between two links.
4. **The plaquette theory is an Ising model in disguise.** By Wegner duality, on a disc it carries no degrees of freedom beyond an Ising model on the tiling's interior vertices. 254 of the 336 vertices at level 2 are two-link plaquettes, so this Ising model has many weakly connected sites.

## 4. Not done

Matter dynamics with the gauge field (tile bits hopping with link phases, the "gauge-Higgs" form of the proposal); a patch with a hole, where the plaquette limit would have a degeneracy; the bit-as-flux reading coupled to the pair Hamiltonian of Stage 1.
