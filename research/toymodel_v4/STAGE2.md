# Stage 2: results (many bits on a patch of real outlines)

Labels as in `PLAN.md`. File: `t4_patch.py` (6 tests, about 4 s).

```
python3 research/toymodel_v4/t4_patch.py --tests-only --story
```

## What was built

toymodel_v2 found the pairing rule on the straight 14-gons of a patch. Here every tile is the real outline of `spectremodel.Tile` (Bezier edges, curve strength `k`, opening `h = tan(omega/2)`), placed by the rigid motions of `spectre_patch.SpectrePatch`, and a seam is closed only if the four Bezier control points of one edge equal those of the other reversed (tolerance 1e-9). A bit is the opening `s = +-1` of vertex 8 in the tile's own frame.

## Results

| id | statement | label |
|---|---|---|
| E2.0 | The tile code of the patch and the tile code of the flow have the same 14 vertices and the same flat vertex (index 8); moving it changes the control points of edges 7 and 8 only | Exact |
| E2.1 | **The pairing rule holds on the curves.** Flat pairs: 6, 82, 804 at levels 2, 3, 4 (same at `b = 1` and `sqrt3`). For all 82 pairs of level 3 and three `(b, k, omega)`, opening as `(+1, -1)` or `(-1, +1)` leaves every seam closed; opening as `(+1, +1)` or `(-1, -1)` breaks exactly 2 seams | Exact |
| E2.2 | A tile with other tiles at its vertex 8 and not in a pair cannot open alone (60 tested, either sign, at least 2 seams break). The bits sit on the 164 tiles of the 82 pairs: 16.9, 29.3, 36.5 per cent of the tiles at levels 2, 3, 4. 89 boundary tiles of level 3 with nothing at vertex 8 open freely: the edge of the patch, not a bit | Exact |
| E2.3 | **The area is a transfer.** Opening a pair as `(+1, -1)` changes the tile areas by `+h` and `-h` and the patch area by 0; both alike changes it by `+-2h` | Exact |
| E2.4 | Of the four sign assignments exactly two close the seams, the `S_z = 0` sector. Each has a reflex tile, so it is valid iff `omega` is below the Stage 3 threshold (valid at 30 and 120 degrees, not at 140 and 170); at `omega = 0` the states merge with the closed pair | Exact |
| E2.5 | **Bits along the flow** (level 3, linear laws): for `omega_0 = 30` degrees the number of pairs with two valid open states is `B = 82` for every `tau < 1` and 0 at the Hat; for `omega_0 = 150` degrees `B = 0` at `tau = 0`, 82 for the rest of the flow, 0 at the Hat. Not monotone in the second case | Exact |

## What it says

1. **The pairing rule is not an artefact of straight polygons.** The curves follow their chords, so a pair opened as a bump and a dent closes every seam, and any other opening breaks two. The conclusion of v2 stands on the real outlines: a bit is the displacement of the *shared* vertex of a flat pair along its common normal, one bit per pair, never per tile.
2. **The conserved quantity is area, transferred between the two tiles** (E2.3). A flip of a pair moves area `h` from one tile to the other; the patch's area does not change. This is the patch version of E1.1, and it is why no bit-dependent area ledger can appear.
3. **The tiling constrains which states are allowed, not their amplitudes.** The allowed space of a pair is the two-dimensional `S_z = 0` sector of two two-level systems, which contains both the singlet and the `m = 0` triplet; nothing in the tiling picks the relative sign. That is the Stage 3 Negative, seen from the patch side.
4. **Counting does not give a second law of its own that points the D6 way.** The log2 of the number of open configurations is `B(tau)`: maximal at the Spectre end and 0 at the Hat. A counting entropy therefore *decreases* along the flow, opposite to the reading 'later = more entropy'. The other reading, that the bit becomes unreadable toward the Hat (information about it is lost), is the direction of D6. Which of the two is 'the arrow' is the conflict of toymodel_v2 (Result `res:records`), resolved in v3 only by adding an external record store. For large `omega_0` the count is not even monotone (0, then 82, then 0).

## What is not done

- Overlaps between non-neighbouring tiles of an opened patch were not checked (only seams, and the validity of each opened outline). Seam closure and area transfer make a gap or overlap at a seam impossible; a collision elsewhere is not excluded and not tested.
- Only the 82 pairs of level 3 were opened one pair at a time; opening several pairs at once is a product of independent pairs for the seams (each pair's seams involve only its own two tiles), but a joint check on the full patch was not run.
- Bits are classical states of the opening, with no amplitude and no dynamics; the weights `c(tau)` of the quantum-bit model were not used.
- The counting entropy is of the allowed states only; no coarse-graining or measure on them is chosen.
