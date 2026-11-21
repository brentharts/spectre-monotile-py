# Stage A0: the flip engine as a module, with a regression against v8

Code: `t9_engine.py` (`--tests-only`, about 2 minutes; `--full`, about 4 minutes in total). A refactor of `wada-coudene/figures/spectre_flip_contacts.py`: same algorithm, no module-level side effects and no `sys.argv`; class `Engine(N, a_class, pbox, r2, in_t, out_t)` with `phases`, `state`, `region_valid`, `pattern`, `run`. Defaults are the v8 corrected settings (box 20, `IN_T`/`OUT_T` = 0.0015/0.0035, delta 0.03, footprint radius 1). It needs two clones, `SPECTRE_DIR` (brentharts/spectre) and `WADA_DIR` (wada-coudene, for `lakes_of_wada` and `figures/spectre_window_maps.npz`); both default to `/home/claude/repos/...`.

## Result (9 of 9 tests pass)

- Setup (Exact / Counted): 12 window classes, 6 single and 6 Mystic-pair; tiles per class at level 3 = 70, 75, 76, 76, 70, 14, 66, 12, 9, 8, 9, 11; similarity ratio sqrt(10); difference lattice of index 36 in Z^4; 767 placements within radius 1 of the class-0 tile at box 20.
- Reversal audit (Exact by construction, confirmed on 300 samples): the step and its reverse change the same samples, give the same valid pairs and outcome counts, and per sample the same outcome with the before and after sizes exchanged.
- **Regression against v8 (Counted): reproduced exactly.** 6000 samples, delta 0.03: 539 change the present set, 468 have a valid tiling before and after, 244 keep the tile set meeting the footprint, 169 change it with a non-isomorphic graph, 55 change it with an isomorphic graph. The run took 107 s, so the v8 contact-graph numbers do not depend on anything outside this module and the two clones.

## Update (stats ported)

`Engine.stats` ports the disc-count and erasure code of `spectre_flip_stats.py`; `--stats` reproduces the v8 run exactly (see `STAGE_A2.md`). Not ported: the cluster-closure code (`spectre_flip_moves.py`).

## Not ported (first version) A test I drafted for "the origin tile is present in unchanged samples" was vacuous (it ended in `or True`) and was removed rather than kept.

## Consequence for the plan

Stage A1 (closed clusters, region radius about 26) and A2 (step scan) can now call this module. Because a 6000-sample run takes under two minutes here, the step scan at delta 0.015 and 0.0075 with the tight tolerance is cheap and I can run it without waiting for you; the radius-26 closed-cluster run is the one that needs your machine, and its time is not yet measured.
