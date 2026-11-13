# Stage 1: results (a Bell test with the tile's own settings, nothing discarded)

Labels as in `PLAN.md`. File: `t7_bell.py` (4 tests, about 5 s), figure `figures/t7_stage1_bell.png`.

```
python3 research/toymodel_v7/t7_bell.py --tests-only
python3 research/toymodel_v7/t7_bell.py --no-show --save research/toymodel_v7/figures/t7_stage1_bell.png
```

## 1. The settings

v6 left the pair qubit with one read-out on the tile: which tile is the bump. Its other directions come from the pair's own dynamics:
- **tunnelling pulse:** a brief, strong `-Gamma_p tau^x` rotates the read-out about tau^x;
- **bias pulse:** a potential difference across the pair, from a nearby charge through the dipole coupling of v5 and v6, rotates it about tau^z.

A party chooses a pulse, and both read the bump at the same time. Every run has an outcome +-1, nothing is discarded, and the coupling between the pairs stays on.

## 2. Results

| id | statement | label |
|---|---|---|
| E7.1 | The ground states are real, so their correlation matrices have no xy or yz entries. Tunnelling-only settings give CHSH = 2 sqrt(T_yy^2 + T_zz^2); full settings give Horodecki's value. For two pairs T = diag(1, -C, C): tunnelling only gives 2 sqrt2 C, full gives 2 sqrt(1 + C^2). The strong correlation lies along the tunnelling axis, which a tunnelling pulse cannot reach | Exact |
| E7.2 | **Two pairs pass.** With tunnelling pulses only: CHSH 2.583 at the Spectre end, 2.454 with the read-out visibility, above 2 with visibility up to tau = 0.053 | Exact given v6's Definitions |
| E7.3 | **Marginals cannot decide locality; the window can.** Every marginal is exactly 1/2 in every protocol, including waiting-time settings, because of the global flip symmetry. What separates the protocols is the phase the coupling acts for during the measurement window. Pulses: K theta / (2 Gamma_p) goes to 0, and CHSH converges to the state's value as 1/Gamma_p (2.267, 2.515, 2.574, 2.582 at Gamma_p = 2, 5, 20, 200; state 2.583). Waiting times: a coupling phase of 7.0 rad in the window, so the correlations can be made during it (the locality loophole); excluded | Exact / Numeric |
| E7.4 | **Patches do not pass.** On the 6-, 10- and 14-pair clusters of v6, tunnelling-only settings give at most 1.95, 2.13 and 1.98, and below 2 with visibility everywhere (largest 1.988). With full settings: 2.00, 2.32, 2.09 | Exact given v6's Definitions; **Negative** |

## 3. What this says

1. **A Bell test without post-selection exists in the toy, for two pairs.** Settings (pulses of the pair's own tunnelling), outcomes (which tile is the bump) and the violation are all defined on the tile, with every run counted. This closes the gap v3 found (detection loophole) and v6 left open (no second read-out), for the two-pair system. The pulses are local in the limit of short pulses, measured by the coupling phase in the window, not by marginals.
2. **It does not carry over to a patch.** On the clusters the frozen end is a cat (v6 E6.9), the tunnelling-only plane misses the axis where the remaining correlation sits, and the read-out visibility takes the rest. The scoreboard row for patches stays open.
3. **The violation lives where the bit is least readable.** Above 2 only up to tau = 0.053, at the Spectre end where visibility is high, and gone long before the Hat: the same direction as the classical rule of v5 Stage 1, now with Tsirelson's bound automatic.
4. **What is assumed.** That a party can choose a pulse is the free-setting assumption of every Bell test (H7.1). The forms of Gamma = c, of K from the charges and of the visibility are v6's Definitions.

## 4. Not done

Bias pulses from an explicit nearby charge (here an ideal rotation); pairs far apart on a patch, where K is small and the entanglement with it; thermal states.
