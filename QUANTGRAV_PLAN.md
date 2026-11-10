# Plan for `quantgrav.tex`

Status (updated 2026-11-10): the outline below is the original skeleton. `quantgrav.tex` now also has the companion-repository results, a dynamic half (directional axes, retrocausal contracts, density, formal model) and a conclusion; see "Update 2026-11-10" at the end. One paper (decided 2026-11-08); the Wilson-loop work is Section 6 of the skeleton.

## Framing
Nothing computed here connects to gravity. The paper reports exact structure on tilings and $E_8$, negative results, and an abelian Wilson-loop probe, and says so. "Quantum gravity" stays out of the claims and out of the title; the file name is only a file name.

## Outline
1. Scope and conventions (labels Exact / Numeric / Negative / Hypothesis; LLM-assistance disclosure; summary table)
2. Background: Spectre and its lift; Chair44; $E_8$, icosians, Hopf fibres
3. $E_8$ and its Hopf fibres: 25 inscribed 24-cells and 10 partitions; shells $240\sigma_3(n)$; windows; $E_6$ and the 27
4. The Spectre in $E_8$: $A_2\oplus A_2$ embedding, corner types, inflation does not extend (negative)
5. The Chair44 count 192 (negative)
6. An abelian Wilson-loop probe: corner locality, placement, gauge, block size; what it cannot see
7. Open problems and the four tests that could fail (from `TILE_E8_BRIDGE.md` section 3)
Appendices: reproduction; tight-binding and twist-hopping models (appendix by default).

## Phases
1. Decisions: done (one paper; tight-binding in an appendix unless changed).
2. Skeleton and build: done (`quantgrav.tex`, `quantgrav.bib`, `make quantgrav`, `make numbers`).
3. Numbers from code: `models/paper_numbers.py` writes `build/quantgrav_numbers.tex` (26 numbers recomputed from the models and checked, 34 from `models/data/wilson_snapshot.json`, flagged with a dagger). To do: recompute the snapshot numbers automatically when the Wilson repository is present.
4. Draft sections 3 to 6 from the existing write-ups, with the existing figures.
5. References, as a gate: an entry enters `quantgrav.bib` only when its data was checked against a source that was opened. Checked so far: Elser-Sloane (via Sloane's list), Sadoc-Mosseri (publisher page, abstract only), Smith et al., Chéritat's page, Moore-Ran-Wen and Deng et al. (arXiv records, no journal data), the Wilson and QGL repositories. Not yet checked, not cited: Gluck-Warner-Ziller, Moody-Patera, Conway-Sloane, Conway-Smith, the HAL record.
6. Review by a fresh reader who sees only the text and the tests; then check every claim against its test.

## Gaps the paper will show openly
- No hexagonal action: the 60-degree corners of the Spectre are not tested (problem P1 of `GEOMETRY_BRIDGE.md`).
- Placement and block size: the Wilson probe is at L = 3 (one run at L = 5), and which placement matches the programme's blocks is unchecked.
- The four evidence tests of `TILE_E8_BRIDGE.md` section 3 have not been run.
- The Elser-Sloane attribution of the Voronoi window is a hypothesis until their paper is read.

## Update 2026-11-10: the dynamic half, and what `research/toymodel_v2` adds

**State of the paper.** Beyond the skeleton, `quantgrav.tex` has: the companion results (Chair44 line sums, Spectre line sums, chirality, hexagonal Wilson action, Mystic X-bonds, curvature, Bragg peaks); the dynamic half (six directions, retrocausal contracts, interaction density, a formal action and wave equation); the CHSH ruler; and a conclusion. The numbers for the dynamic half come from the Quantum-Gravity-Dynamics fork (`directional/`, ledger in `directional/LEDGER.md`) as snapshots (dagger).

**What the new work is.** `research/toymodel_v2/` (`PLAN.md`, `STAGE0.md`, `STAGE1.md`) is a staged programme aimed at the paper's open problems 11 (arrow), 12 (derive the rule) and 13 (density fall-off and horizon). Stages 0 to 5 are done; the chain of models `spectre_hat_*_model.py` and `ARROW_OF_TIME.md` come before it. Unlike the snapshot numbers, every Stage 0 and Stage 1 number is recomputed by tests in this repository (`core.py` 7, `tm2_lattice.py` 9, `tm2_history_sum.py` 8, `tm2_directed.py` 9, `tm2_arrow.py` 11, `tm2_rule.py` 11, `tm2_waves.py` 9, about 50 s together).

**Results that could enter the paper** (each is a test, none is a claim about physics):
1. *The Spectre, the Hat and everything between are one lift* (Stage 0, E1 to E3): every edge of every tile of the family is an $E_8$ root; the flow rescales the green plane against the red; the bond graph (406 seams at level 2) is identical for $b\in[1,2]$. This generalises the appendix statement that Hat, Spectre and Turtle share a bond graph from three tilings to the whole family. Exact.
2. *Rank collapse at the Hat* (E4): the lift has rank 4 but its projection at $b=\sqrt3$ lies on the hexagonal lattice (rank 2); read through that lattice the long edges have $E_8$ norm$^2$ 6 and are not roots. Two exact readings that differ; the paper's embedding is the first. Exact.
3. *Flat pairs* (E5 to E7): the flat vertex of a tile meets the flat vertex of exactly one other tile (about 40% of interior tiles at level 4), or non-flat vertices, or nothing. Opening every flat vertex alike breaks 88 of 406 seams at level 2; opening a flat pair as a bump and a dent closes every seam and conserves area. Exact (finite patches). A candidate for the first compatibility rule that the tiling itself supplies; its reading as a rule of nature is a Hypothesis.
4. *Influence of a forced node* (Stage 1, E1.4): with the flux constraint on, a forced node influences the far end of the timeline as much as its neighbours; without it influence decays geometrically. Negative for the "ripple" wording of the retrocausal section. Exact (finite).
5. *The paper's action* (E1.2): $\mathcal A[\gamma]$ is reversal symmetric on every closed zero-target timeline (all 2040 of length 6, proved) and not on open ones, so an arrow in the formal section can only come from pinned ends or the damping term. Exact.
6. *The flux constraint cannot see the bit* (E1.7): outlines stay closed under opening; the seam rule is a strict subset of $Q=0$ (64 of 924 of 4096 at level 2). Exact (finite).
7. *Stage 2* (`STAGE2.md`): the chain is the special case of the six-root history-sum with equal axes (regression to 1e-12); the orientation sits in the amplitude schedule and a tight generic vector constraint can override it (Numeric); the scale ladder follows from an assumption, not from the tiling.
8. *Stage 3, the arrow audit* (`STAGE3.md`): the orientation inputs are boundary data (including the base point of the flux, which the formal section does not state), the sign of the damping, and the orientation of tau; all are put in by hand. Oriented closed timelines are the majority from 5 steps (census), so problem 11 is about the weight, not about the existence of oriented objects. Records sit at the frozen end, against the chain's drawn direction (Negative for a single arrow).
9. *Stage 4, a derived rule* (`STAGE4.md`): a two-ended history weight gives a correlation rule with no free parameter kappa, exact no-signalling for six and twelve settings, and CHSH from 0 to 4 set by stiffness and wing length. It is not the exp(kappa AB) rule, it rests entirely on conditioning on both ends (the pass rate depends on both settings), and no quantum bound appears, with or without complex amplitudes.
10. *Stage 5, waves and the clock* (`STAGE5.md`): the wave equation of the formal section is reversible and causal on a lattice with the damping off, and its speed is c sqrt(rho): faster in dense regions, against the paper's text. A snap radiates and leaves a static field; 1/r needs a 3D sheet. If the squared jump of the bit follows the static field, a 2D sheet gives the clock factor exactly with a horizon and no 1/r; a 3D sheet gives 1/r, the clock to first order, and no horizon.
11. *From the chain* (`spectre_hat_*`): cost asymmetry between forward and backward histories, and its reversal with the flow (`ARROW_OF_TIME.md`); the Helstrom quantity $\sqrt{1-c^2}$ of the quantum bit has the form of the QGD clock $\sqrt{1-\sigma_t^2}$ (a formal coincidence until $\tau(r)$ is derived).

**Claims in `quantgrav.tex` to revise when these enter** (nothing edited yet):
- Retrocausal section and formal section: "ripple outwards" and "past nodes adjust cooperatively" describe the greedy solver; with the flux constraint the influence of a forced node is global (result 4).
- Formal section: the action carries no arrow on closed histories (result 5); say where the orientation sits (pinned ends, damping).
- Appendix on tight binding: "Hat, Spectre and Turtle have the same bond graph" becomes the statement for the whole family (result 1).
- Formal section: state where the flux is counted from; for open histories that choice is the action's arrow (result 8).
- Result on the rule exp(kappa AB): it depends on the naming of Bob's outcome (renaming is kappa -> -kappa); state the convention or replace it by the derived, covariant table (result 9). Open problem 12 is then partly answered: a rule is derived, the AB form is not, and the assumption doing the work is the two-ended conditioning.
- Wave equation of the formal section and the sentence "high-density regions induce phase lag": they disagree (speed is c sqrt(rho)); fix one (result 10).
- Open problem 13: restate as the choice found in result 10 (2D sheet: exact clock, horizon, no 1/r; 3D sheet: 1/r, first-order clock, no horizon), with the assumption that the squared jump is linear in the field.
- Open problem 11 should be restated: oriented closed timelines are generic; what is missing is a weight that tells a timeline from its reverse.
- Open problems 11 to 13: partly addressed as above; problem 12 has a second candidate constraint (the seam rule).

**New phases.**
7. Decide whether the chain and `toymodel_v2` enter this paper or a second one (they share the bibliography gate and the labels). Default: results 1 to 3 into the geometry half (they are about the Spectre family), results 4 to 10 into the dynamic half as tested corrections of its own claims, result 11 as background of `toymodel_v2`.
8. `paper_numbers.py`: recompute the Stage 0 to Stage 2 numbers (74%-style rates are not needed: use the exact ones) into macros; Appendix A gets the three modules in the checks table.
9. Re-run the review of phase 6 on the dynamic half, since it was written before these tests.

## Gaps added by this update
- The letter timeline and the bit are put in one history only by a Definition (letter = bit times a prescribed root); the axes are not taken from the tiling and six free letters per tick have no amplitude (`toymodel_v2` Stage 2).
- Flat-pair statistics are on levels 2 to 5 only; the limit of the pair fraction (24.0, 34.9, 40.0, 42.4% at levels 2 to 5) and the clustering of pairs are unknown.
- The C7 identity (Helstrom $\sqrt{1-c^2}$ against the QGD clock) and the scale ladder $c_j=c_0^{(4+\sqrt{15})^j}$ are not tested; both need a stated assumption (see `toymodel_v2/PLAN.md`, C7 and C8).
- The map from the overlap to distance has two readings with opposite conclusions (dense = plastic or dense = frozen); neither is derived (`toymodel_v2` Stage 5). `ARROW_OF_TIME.md` section 7 records which one the past/future picture needs.
- The derived rule's parameters (stiffness, wing length, boundary coupling) are not tied to the chain's overlap or to the tiling; a read-out with non-commuting settings is not built, so nothing here can show a bound at 2 sqrt 2.
- No derivation of why the squared jump of the bit should be linear in the sheet's field; no back-reaction of the field on the bit; the sheet is a cubic lattice, not the tiling.
- No record store or environment in any model, so the record test uses the register as its own memory.
- No source on Janus-point arrows, two-state-vector quantum mechanics or Helstrom discrimination has been opened (names only).
