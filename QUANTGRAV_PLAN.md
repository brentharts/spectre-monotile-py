# Plan for `quantgrav.tex`

Status: skeleton builds (`make quantgrav` -> `build/quantgrav.pdf`); every section is a stub with `\todo{}` markers. One paper (decided 2026-11-08); the Wilson-loop work is Section 6.

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
