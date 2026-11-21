# Stage B0: what the unread sources say (2026-11-20)

Read through WebFetch (text of the arXiv pages; the Tsiokos page only the first 100 000 of 159 589 characters). Nothing is cited in `quantgrav.bib` until the notes there say what was read.

## Tsiokos, arXiv:2609.19214, "A Strongly Aperiodic Monotile in Three Dimensions" (submitted 16 Sept 2026)

This is the Chair44 paper that `quantgrav.tex` already cites as Tsiokos26. So the 3D einstein idea (B3) is **not an open problem in the form proposed**: it is claimed solved, and the repo already works with the solid.
- Claim: a seven-cube chair with 24 exposed unit panels carrying square-pyramid bumps or dents; tilings exist (reflections allowed); no tiling has a translation; every tiling has a symmetry group of order at most 24; every tiling is homochiral; a unique nested hierarchy of supertiles.
- Strong in the sense of Mozes and the hat paper; only **mildly** aperiodic in the stricter taxonomy of Coulbois, Gajardo, Guillon and Lutfalla (an explicit tiling has a symmetry group of order 8).
- Status: not refereed (the author calls it a proof submission); finite checks stated as Lean 4 theorems, the main theorem kernel-checked modulo 21 `native_decide` hooks; two independent Python replays of the census.
- **B3 becomes:** relate the Chair44 contact rule to what v8 measured for the Spectre (is the unique-hierarchy statement a model-set statement, what are its windows), and keep the screw-axis question as a comparison with the SCD biprism. Claims about Chair44 stay "as stated by the author, unrefereed".

## Koca, Ozdes Koca and Al-Siyabi, arXiv:2003.13449 (Danzer ABCK from D6)

- Inflation factor tau = (1+sqrt 5)/2. Tile K has the vertex at the origin and three vertices that are projections of D6 vectors built from an integer pair (m1, m2), m1 + m2 even.
- Inflation matrix as copied (rows tau K, tau B, tau C, tau A; columns K, B, C, A): (1,1,0,0), (4,2,1,0), (2,0,2,1), (6,3,2,0).
- **Exact check (ours):** its characteristic polynomial is x^4 - 5x^3 + 2x^2 + 5x + 1, with Perron root tau^3 = 4.2361, as it must be since volumes scale by tau^3. The volume eigenvector gives K : B : C : A = 1 : 2tau : 2tau : 2tau+2 (3.236, 3.236, 5.236). The matrix is therefore consistent; the volumes are our computation, the paper does not state them.
- Not read: the full tiling procedure (rotations and translations in D6).

## Still to read

Gahler, Hunton, Kellendonk (arXiv:1202.2240, math-ph/0505048) beyond the snippets; Baake and Frettloh on SCD tilings; Nischke and Danzer 1996 (the 7-fold construction; only the Tilings Encyclopedia summary was read).
