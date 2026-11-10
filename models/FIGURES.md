# Figures for the CHSH and retro-rule results

Three matplotlib figures for results of `quantgrav.tex` (Results `chsh`, `price`, `rule`). They are **not** included in the paper; this page holds them with
the script that draws each, the numbers it checks, and the label of each statement. Every script recomputes its numbers and runs self tests; a figure is not
drawn if a test fails.

    python3 models/fig_chsh_price.py   --no-show --save-dir figures     (about 55 s: solves the linear programmes)
    python3 models/fig_retro_rule.py   --no-show --save-dir figures     (about 7 s)
    python3 models/fig_hex_settings.py --no-show --save-dir figures     (closed form)
    add --tests-only to run only the checks

`fig_chsh_price.py` and `fig_retro_rule.py` import the `directional/` modules of the Quantum-Gravity-Dynamics fork (imported, not copied) through
`models/_qgd.py`, which looks in `$QGD_DIR`, then `../Quantum-Gravity-Dynamics`, then `/home/claude/Quantum-Gravity-Dynamics`. `fig_hex_settings.py` needs nothing else.

## 1. The price of a Bell violation: `figures/chsh_price.png` (numeric)

![CHSH price](../figures/chsh_price.png)

Largest CHSH value S under no signalling against delta, how far a hidden-state probability may move from uniform when it depends on both settings. Hexagonal
set: min(4, 2 + 12 delta). Cubic set: min(4, 4/3 + 8 delta), which passes the local value 2 only for delta > 1/12 because the cubic read-out has zero outcomes.
The open markers are linear programmes solved by the fork (`hex_directions.scan_hex`, six read-outs; `retro_boundary_model.scan`, 26 cubic read-outs) and agree
with the lines to 1e-15. Tests: 6.

## 2. The rule that pays the price: `figures/retro_rule.png` (exact checks, numeric curve)

![Retro rule](../figures/retro_rule.png)

Left: best |S| on the six hexagonal directions for the weight P(v | a, b) proportional to exp(kappa A B): 2 at kappa = 0, 3.62 at kappa = 1, 2 sqrt 2 at
kappa = 0.362, always below the linear-programme bound min(4, 2 + 12 delta(kappa)). Right: signalling at kappa = 1. It is zero on the six directions and nonzero
for the twelve read-outs and for a control that breaks the symmetry v -> -v. So the zero is a symmetry property of the six, not a feature of the rule, and the
weight is a chosen family, not derived from a path action. Above 2 sqrt 2 the table is a no-signalling box, not a quantum model. Tests: 8.

## 3. Four settings on six hexagonal directions: `figures/hex_settings.png` (exact)

![Hexagonal settings](../figures/hex_settings.png)

Settings a = b = +A, a' = +B, b' = -C give |S| = 4 for the toy with the sign read-out E(a, b) = -sgn(a . b); the singlet, E = -a . b, gives 5/2 on the same
settings (its maximum on the six). On the six cubic directions both give 2. The toy is a no-signalling table, not a model of the singlet. Tests: 4.

## What these figures do not show

They show properties of a chosen toy and of linear programmes over it. No measured CHSH value is used, no claim is made about gravity, and the rule of figure 2
is not derived.
