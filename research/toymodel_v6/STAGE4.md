# Stage 4: results (a particle that changes fast bits as it hops, read as a clock)

Labels as in `PLAN.md`. Nothing here is gravity. File: `t6_clock.py` (5 tests, about 1 s), figure `figures/t6_stage4_clock.png`.

```
python3 research/toymodel_v6/t6_clock.py --tests-only
python3 research/toymodel_v6/t6_clock.py --no-show --save research/toymodel_v6/figures/t6_stage4_clock.png
```

## 1. The question and the model

The C7 question of toymodel_v2 (and of v5 Stage 5) asks whether the overlap c of the bit's two states is the clock factor sqrt(1 - r_s/r). In v2 the overlap was matched to the clock by a reading; it was never a rate of anything.

Here a particle hops on a chain (`-t` per bond), and its presence changes the ground state of a fast bit (field strength Delta much larger than t) to a state with overlap c with the old one. That is the Franck-Condon (polaron) mechanism: every hop carries the overlap of the bits it changes. Two placements are compared:
- **site bits:** one bit per tile, set while the particle sits on it;
- **link bits:** one bit per seam, set by which side of the seam the particle is on. This is the bit as the flux at the seam's vertex, from Stage 3.

The clock is defined as the hopping rate relative to the bare one (Definition).

## 2. Results

| id | statement | label |
|---|---|---|
| E6.22 | Site bits narrow the band by c^2: on 6 sites with 6 bits, t_eff / t tends to 0.09, 0.36, 0.81 for c = 0.3, 0.6, 0.9; the error falls as 1/Delta^2 | Numeric (exact diagonalisation) |
| E6.23 | Link bits narrow it by c: t_eff / t tends to 0.3, 0.6, 0.9, error 1/Delta^2 | Numeric |
| E6.24 | Along the flow the clock is slow at the frozen end: link rate c(tau) rises strictly from 0.223 at the Spectre to 1 at the Hat (site rate c^2 from 0.050) | Exact |
| E6.25 | A slow region delays a packet by the time of flight. With a slab of c = 0.5 on a 2400-site chain, a packet at the band centre arrives late by 136.2 against the semiclassical 136.1; reflected weight below 1e-25 | Numeric |
| E6.26 | C7 under v2's rule I: the link rate exp(-r_s/2r) equals the clock to first order (10.1% above it at 2 r_s, below 1% from 7.5 r_s, v2's numbers). The site rate exp(-r_s/r) is the square of the link rate: its first-order term is twice the clock's | Exact |

## 3. What this says

1. **The overlap becomes a rate.** In v2 the identification "c = clock" (reading B) was a choice between two readings. Here it follows from a Hamiltonian: a particle that has to change a fast bit to move carries the Franck-Condon factor, so its rate is c per changed bit. The frozen end (bits readable, c small) is where the particle is slow.
2. **The placement decides the power, and only the seam placement passes C7.** With bits on the tiles a hop changes two bits and the rate is c^2, twice the redshift of the clock at first order. With the bit as the flux on the seam (Stage 3's reading), one hop crosses one seam and the rate is c, the clock to first order. This is the first place where Stage 3's gauge reading changes a number.
3. **What is still put in.** That rule I holds (squared jump proportional to the field), and that 1/r comes from a three-dimensional source (v5 Stage 5: the unpaired live bit). The identity holds to first order only, as in v2, and there is no horizon: c does not reach 0 at finite r.
4. **The delay is a clock effect, measurable as a time of flight.** A region of slow bits delays a packet exactly by the integral of 1/v, without reflection when the region is smooth. Within this toy, this is the Shapiro-type delay of a clock-only metric (the paper's light-bending result needs a space factor in addition).

## 4. Not done

Bits with their own dynamics (finite Delta, the pair tunnelling of Stage 1) and the slow-bit regime, where the polaron is not coherent; a 2D or 3D patch instead of a chain; the source of the field derived rather than taken from v5; a test of the space factor B = 1/A that light bending needs.
