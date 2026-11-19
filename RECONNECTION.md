# Gravitational reconnection (Comisso & Asenjo) and this repository

Source: Comisso & Asenjo, "Reconnection of Gravitational Fields", arXiv:2609.25241v1 / Phys. Rev. D 114, 084095 (2026). Read in full from the arXiv PDF; derivations not checked; published version not compared. Paper section: `sec:toward-reconnect` in `quantgrav.tex`.

| Idea | Status | Statement |
|---|---|---|
| Flux and connectivity | **Exact** (flux), no analogue (connectivity) | In the Z2 gauge form, the flux through a plaquette set S equals the product of Z_l over the cut of S, so X_l changes it iff l is in the cut (discrete Eq. 29). Test `test_e622_...` in `research/toymodel_v6/t6_gauge.py`: level 3 has 3534 links, 186 change one plaquette flux, 3348 change two. Sheet connectivity is not defined: flux is pointlike on a 2D complex. |
| Phason flip = reconnection | **Negative** as stated | Lift edge vectors are differences of lattice points, so the tetrad 1-forms are exact and F = 0 for every phase. Flips are ideal. A different object (tile contact graph) is needed; the Wada-type multiplicity (about 10% at r = 0.064) is a candidate statistic, untested. The identification tetrad index = lift coordinate is ours. |
| Helicity = handedness | **Idea** | Helicity needs A.B in 3+1; the tiling is planar. Only a parity-odd analogy (Im Z_k, chiral outline). |
| Sparling exchange = missing source | **Negative** | X is proportional to F.T, vanishing in vacuum, so it is no monopole source. Only a structural analogy to the pair ledger (sum conserved, parts exchange). |
