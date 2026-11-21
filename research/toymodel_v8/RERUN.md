# Rerun of the Stage 0, 3 and 7b statistics with the corrected engine (2026-11-20)

Corrections (see `STAGE1.md`, `STAGE3.md`): (1) lattice box `PBOX` 20 instead of 12, (2) tolerance IN_T = 0.0015, OUT_T = 0.0035 instead of 0.003 / 0.010, because a flip happens only where the phase is within the step of a window boundary, so a loose IN_T makes the old and the new tile both "present" exactly in the changed samples. Class 0, level 3, step 0.03. All **Counted**. Logs and JSON are named `*_tight_*`.

## Contact graph under flips (`spectre_flip_contacts.py`; footprint = tile at the origin plus margin 1, 767 placements)
| run | valid pairs | tile set meeting the footprint changed | graph not isomorphic (tile set changed) | tile set changed, isomorphic as unlabelled graph |
|---|---|---|---|---|
| tight tolerance, box 20, 6000 phases | 468 of 539 changed | 224 (48%) | 169 (36% of valid, 75% of changed) | 55 |
| reversed step (SWAP), same | 468 | 224 | 169 | 55 (identical: reversal symmetry, Stage 0) |
| old tolerance 0.003/0.010, box 12, 3000 phases | 60 of 192 | 32 (53%) | 27 | 5 |
| default tolerance 0.008/0.016 | 0 of 356 | contaminated | | |
The earlier numbers (30 changed, 20 non-isomorphic among 343 valid pairs) came from a contaminated and incomplete setting; with the old settings re-run today the proportions among valid pairs are similar (27 of 32 non-isomorphic) but the sample is small and selected. With the corrected setting the qualitative result stands and gets better support: **a flip that changes the tile set meeting the footprint changes the contact graph's isomorphism class in 75% of 224 cases (169)**. The absolute fraction of changed tile sets (48% of valid changed samples) is not comparable with the old 30/343, which was diluted by samples with changes only far from the footprint.
Hypothesis as originally worded still holds only for this definition of connectivity (tiles meeting a fixed footprint); no F, no R, no transport velocity: not reconnection in the sense of the paper. Not an arrow: the reverse step gives the identical counts.
Note: "tile set meeting the footprint changed" includes tile-for-tile swaps; the graph changes in 75% of these, so many are not swaps.

## Part A of the statistics (`spectre_flip_stats.py`, disc counts, 2831 usable of 3500 phases, step 0.03, R2 = 7)
| rho | present | changed | changed per present tile (old / new) | area-conserving events | P(n_G changes) among them (old / new) |
|---|---|---|---|---|---|
| 3 | 2.2 | 0.128 | 0.027 / 0.058 | 43 (old 5) | 0.80 / 0.77 |
| 5 | 7.3 | 0.406 | 0.029 / 0.056 | 103 (old 18) | 0.06 / 0.010 |
| 7 | 17.2 | 0.971 | 0.028 / 0.056 | 163 (old 27) | 0.33 / 0.32 |
- H8.5 (changed tiles grow as rho^2): slope 2.25 (old 2.22): **supported again**. The rate per tile doubles (0.056 against 0.028) with the tight tolerance, as expected from the selection effect named in STAGE7B.md (fewer samples dropped); it is still constant across rho.
- H8.4 (Mystic-pair count not locally conserved): at rho = 7, 163 area-conserving events (old 27), n_G changed in 0.32 (old 9/27 = 0.33): **supported again, now with a larger sample**. Non-monotone in rho (0.77, 0.25, 0.01, 0.40, 0.32 at rho 3 to 7), so the fraction is not a smooth function; the sign of the effect is the stable part.
- H8.6 (multiplicity against erasure): P(inside not determined by outside) = 0.000 for every multiplicity at both radii: **negative again**.

## Parity (`t8_parity.py`)
Uses level-N patches and window areas only (no raster tolerance, no placement list): not affected by either correction. Not rerun; 5 tests passed earlier.

## Still not done
Stage 3 closure (needs R2 of about 26; run locally, see STAGE3.md), cluster size against step, the step scan of the contact test (a run at step 0.015 was killed for memory when run in parallel).

## Figures (`plot_rerun.py`, data from the `*_tight_*` JSON files and `flip_examples.json` written by `export_flip_examples.py`)
- `fig_rerun_contact.png`: (a) outcomes of the 468 valid pairs, forward and reversed (identical); (b) fractions with tight and old settings; (c) tile count meeting the footprint before against after.
- `fig_rerun_stats.png`: (a) changed tiles against rho, tight and old, with rho^2; (b) changed per present tile; (c) P(n_G changed) for area-conserving events with counts; (d) erasure test, multiplicity against consistency.
- `fig_flip_gallery.png`: six flips, before and after, within 4.5 of the origin tile: removed tiles orange, added blue, Mystic pairs hatched. Each flip moves 5 to 10 tiles in view, so it is a cluster, not a one-tile swap (as in STAGE3.md).
Reading notes: panel (c) of the stats figure is not smooth in rho and the old series starts at rho = 3. The gallery events are a random sample of 416 valid events that touch the footprint, not a selection of the clearest ones.

## Wada covering script with the wider box (2026-11-20)
`spectre_window_covering_pbox.py` (the covering script of `wada-coudene` with `PBOX`): class 0, level 3, 700 pixels: 368 overlapping placements and 92 containing the probe point at both PBOX = 12 and 20. The covering results for class 0 are not affected by the box. Classes 2 and 5 not rerun (class 5 is the one most exposed).

## Paper
`quantgrav.tex` Section "Phason flips: clusters, contact graphs and what they could mean for physics" (`sec:flips`), Results `res:flip-engine` to `res:flip-erasure`, Table `tab:flips-physics`, Figures from `research/toymodel_v8/figures/`; bib entries `socolar1986`, `pachner1991`, `landauer1961`, `qwla2026`. Builds at 50 pages.
