# Stage 7 of toymodel_v8: connections from qwla (orientation parity, erasure codes)

Read: `github.com/panjea/qwla` (`README.md`, `SPECTRE_QEC_CODES.md`, `SPECTRE_QUANTUM_UNIVERSE.md`; the code was only listed, not run). The notes quote Li and Boyle (arXiv:2311.13040) and Batle and Bednorz (arXiv:2607.15326). We did not read the Batle and Bednorz paper in full: a search snippet of its Section 4.4 (the exchanging isometry of the two classes is the rotation R30, so the Spectre sectors merge under SE(2)) agrees with the note. Everything numerical below was recomputed on our own patches and windows.

## Results

**E8.1 (Exact, `t8_parity.py`).** Our patches have 1, 9, 71, 559, 4401, 34649 tiles at levels 0 to 5, obeying N_k = 8 N_{k-1} - N_{k-2}, as the qwla README says. The 559 is the same patch whose cell complex has 3534 links in `t6_gauge.py`.

**E8.2 (Exact).** Orientation parity (index m = angle/30 degrees mod 12, parity m mod 2) of our patches: the minority parity is exactly the Mystic-shaped tiles at every level (1, 8, 63, 496, 3905).

**E8.3 (Exact).** even(L) = odd(L-1), so the minority frequency tends to (5 - sqrt15)/10 = 0.11270 (0.112702 at level 5) and odd/even to 4 + sqrt15 = lambda^2. This reproduces the Batle and Bednorz frequencies in the qwla note; in our reference frame the roles of "even" and "odd" are swapped, which is the LI1/LI2 relabelling by R30 that the note describes.

**E8.4 (Exact; a correction to the note).** The note gives the sector gap as |K| sqrt15/5 = 0.7483 |K|. sqrt15/5 = 0.7746; the formula f_even - f_odd = sqrt15/5 is right and the decimal is not.

**N8.1 (Counted, `spectre_window_area_ratio.py`).** The window of the Mystic-pair class G against the window of the single class O: area ratio G/O at tolerance 0.012, 0.006, 0.003, 0.0015: 0.1578, 0.1515, 0.1479, 0.1460, against the prediction 1/(3 + sqrt15) = 0.14550 (0.3% at the finest tolerance, falling with the tolerance as the boundary dimension predicts). The prediction follows from E8.3: pairs per single = minority / (majority - minority) = 1/(3 + sqrt15). So the internal-space windows carry the same ratio as the orientation-parity frequencies: the two LI-class frequencies are window areas.

## Connections, each stated as a Hypothesis with a test (run in `STAGE7B.md`)

- **H8.4 (parity as the sector label of a flip).** Under the Batle and Bednorz reading, the parity N_K^even is the sector observable. A flip of the tiling must keep the LI class of the whole hull, but locally it can change the count of even tiles. Test (not run): for closed flip clusters, the change in the number of G-class (Mystic pair) tiles. Area conservation fixes n_O + 2 n_G only, so n_G is free. If n_G changes by flips, the local parity is not conserved and only its density is.
- **H8.5 (local recoverability against flips).** H2 of the notes (outside a bounded region K, the tiling determines the inside) says two distinct tilings of the hull cannot differ only inside a bounded K. A global phase shift changes tiles with positive density (about delta^0.89, Result `res:phason`), spread over the plane, which is consistent with H2. The flip clusters of Stage 3 (1 to 13 tiles) are pieces of that pattern, not isolated changes. Test (not run): the number of changed tiles inside a disc of radius rho against rho^2 at fixed delta.
- **H8.6 (erasure and the flip multiplicity).** The Wada-type multiplicity (3 or more window pieces within r of a boundary point, about 10% at r = 0.064) is where more than two tiles are candidates for the same place; whether those are the places where an erased region K is least recoverable is a question for a simulation of erasure on `QC-71` of qwla, not done.
- **H8.7 (cohomology).** The note's rank-4 first cohomology H^1 = C^4 of the hull is the rank of our lift Z^4 (Result `res:lift-flux`: the tetrad 1-forms are exact). Both counts are 4; a map between them is not defined here and is not claimed.

## Not taken over from qwla

- `TriSpectreNetwork` couples qubit expectations to Bezier twists: a feedback loop with no conservation law stated; it is a simulation design, not a result, and it has no counterpart among our tests.
- "Entanglement across scales" (`CX`/`CZ` between QC-9, QC-71, QC-559) is a construction; no statement of ours depends on it. The scale ratios 9, 71, 559 are the patch sizes of E8.1.
- The date given for arXiv:2607.15326 in the note ("October 30, 2026") does not match an identifier of July 2026; we did not check which is right.

Files: `t8_parity.py`, `spectre_window_area_ratio.py` (also in `wada-coudene/figures/`).
