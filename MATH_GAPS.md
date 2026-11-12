# Math gaps in quantgrav.tex: ledger

Status words: **Proved** (a proof is in Section "Foundations" or the section named), **Checked** (computation only), **Open** (no proof; route given), **Quoted** (classical theorem used without a source opened here).
Resources in the Hat and Spectre repository (`brentharts/spectre`, cloned next to this one): `spectre_substrate.tex` (Lean-checked closed statements), `CONNECTIONS.md` (ledger of links), `tile_family.py` (area form of Tile(a,b)), `spectre_bkl.py`, `spectre_e8.py`, `spectre_operator.py`, `spectre_mystic_fibres.py`, `spectre_torsion.py`, `NEXT_STEPS.md` (directions A to K).

## Filled in this round

| id | statement | where | status |
|---|---|---|---|
| F1 | shell n of E8 has 240 sigma_3(n) vectors and 10 sigma_3(n) fibres of 24, for all n | Result found-shell | **Proved** (theta = E4, free action) + **Quoted** (modularity, dim M_4 = 1); previously Checked n <= 8 |
| F2 | the inflation cannot extend to an automorphism of E8 preserving the lift lattice | Result found-inflation | **Proved** (finite order); previously asserted |
| F3 | area of the tile = chord-polygon area A0(a,b), independent of k; A+- = A0 +- a h; corner turns change by (-w, 2w, -w) | Result found-area | **Proved** (point symmetry of the Bezier edges, rotation index); A0 agrees with `tile_family.py` |
| F4 | reflex outline of the straight tile is simple iff h < h*(b) = min((3b+sqrt3)/2, (b+sqrt3)/(sqrt3 b - 1)); omega_c = 134.18 deg at the Spectre, exactly 120 deg at the Hat; crossover b_x = (1+2 sqrt7)/(3 sqrt3) | Result found-valid | **Proved** for the two contacts; minimality **Checked** on 25 values of b |
| F5 | vertex graph of a tiling with bounded edge lengths and degrees is recurrent (Spectre) | Result found-recur | **Proved** (Nash-Williams, **Quoted**) |
| F6 | canonical antiperiodic spin structure of a simple outline | Result found-spin | **Proved** (Umlaufsatz, **Quoted**) |
| F7 | the flat-pair counts 0, 0, 6, 82, 804, 7012, 58106, 469766 (levels 0 to 7) satisfy the order-5 recurrence with roots mu^+-1, nu, 2 - sqrt5, 1; closed form P_k; density -> rho_0 = (5 - sqrt15)/5 exactly | Result found-pairs, `t4_patch.py` E2.7, `t4_pairs_level7.py` | **Exact given the recurrence**; the recurrence is **Checked** (fitted on levels 0 to 4, holds on 5, 6, 7); proof of the recurrence Open (see O2) |

## Open, with a route

| id | gap | route | uses |
|---|---|---|---|
| O1 | omega_c(k) for Bezier edges (k > 0): closed form or algebraic characterisation | PARTLY FILLED by `research/toymodel_v4/t4_critical.py`: contact family V (vertex on edge 12) is closed form h_V = (3b+sqrt3)/2 - 3k g(u_V(b)), u_V a root of a cubic; family W is a degree-6 polynomial after eliminating h; h*(b,k) is the minimum over families and is non-monotone in k; for k near 0.8 an adjacent-edge crossing (8',9) gives h = 1.729 independent of b. Open: closed forms for W, T and the adjacent-edge family, and a proof that no other contact occurs | F4, `t4_spin.omega_c` |
| O2 | prove the recurrence of F7 | the shape is forced by the structure (pairs inside tiles ~ mu^k, along the boundary ~ nu^k, constant from the 1 of the perimeter law), so a proof needs the boundary of a supertile as a finite-state operator: the repository has it for the perimeter (`spectre_boundary_operator.py`, `boundary_certificate/`, recurrence P(k) = 5P(k-1) - 3P(k-2) - P(k-3)); extend its states by the corner types that decide whether two flat vertices meet alone, and show the junction count J_k satisfies the same recurrence. Then P_k = sum_j M^(k-j) J_j gives the closed form. A direct local bijection pair <-> valence-4 vertex was tried and does not exist (each valence-4 vertex has exactly one corner of index 12, but 295 of the 992 such tiles at level 4 are in no pair) | `spectre_boundary_operator.py`, `boundary_certificate/`, CONNECTIONS rows 21, 28, 29 |
| O3 | the linear flow laws b, k, omega are choices; why a -> sqrt3 a and omega -> 0 | b: the Hat is Tile(a, sqrt3 a), the Turtle Tile(a, a/sqrt3) in one family with area form A(a,b) (`tile_family.py`); the flow is a path in that family. omega, k have no derivation | `tile_family.py`, Readme "One family, three tiles" |
| O4 | S_max(delta) = min(4, 2 + 12 delta) | exhibit the LP dual: a combination of the four CHSH terms and the no-signalling equalities whose bound is 2 + 12 delta, using the 6-fold symmetry (the counts of Result price are an LP optimum only) | Quantum-Gravity-Dynamics fork |
| O5 | transience of the Chair44 graph; constant R_eff = 2.81 in h = R_eff / r | Z^3 hitting probability = G(x)/G(0) with the lattice Green function (Watson integral G(0) = 1.516...); identify R_eff from G(0) and the absorbing set; transience from quasi-isometry to Z^3 (Quoted) | `chair44_lines.py` |
| O6 | Spectre and Hat as members of one family and the fibres on the cusp torus (six Mystic classes = one orbit of Z/6 on torsion points of the hexagonal torus) | the flow of Section 2 acts on the same family; whether the bit at P_8 is seen by the torsion dictionary is untested | `spectre_torsion.py`, `spectre_mystic_fibres.py` |
| O7 | the spin structure vs Dic_12 holonomy of the repository (CONNECTIONS row 30, Numerology) | compare the half-angle lift of F6 with the Dic_12 character on the 14-step walk; a map would turn a Numerology row into a statement | `spectre_holonomy.py` |
| O8 | the inflation lambda^2 = 4 + sqrt15 as a BKL orbit (CONNECTIONS rows 1 to 3) vs the E8 results | true for every quadratic unit; needs a property special to the Spectre before use | `spectre_bkl.py` |
| O9 | sqrt15 = sqrt3 sqrt5 joining the Eisenstein and icosian structures of E8 (CONNECTIONS row 17, Numerology) | exhibit a map; F2 shows the inflation is not an isometry of any such construction | `spectre_e8.py`, `E8.md` section 4 |
| O10 | Voronoi window: that W is the window of Elser and Sloane | read the sources (not opened); a Hypothesis meanwhile | |

## What each filled gap changes in the paper

F1 turns a finite check into a theorem. F3 and F4 make Section 2 rest on proofs instead of tests; F4 also shows the exclusion of the reflex value is an algebraic condition, which is what O1 asks to extend. F5 is half of the clock argument (recurrent planar substrate); the transient half is O5. F6 is the only structure of the project that is canonical, and it still does not supply the sign (Result flow-sign). F7 is the first link between the flow and the substitution arithmetic of the repository.
