# Data

`r44_features.csv` lists the 192 pyramid features of the Chair44 (R44) solid: panel, normal, signed height and the
exact apex ("tip") of each. It is derived from `solid/r44_solid.json` and `solid/native_panels.csv` of

> Ioannis Tsiokos, *six-birds-tiles*, <https://github.com/ioannist/six-birds-tiles>, commit
> `d53d2245bc91c01cf81b6490a1cd6732e7f73bd3` (2026-10-02), licensed under
> [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).

and describes the solid of I. Tsiokos, *A Strongly Aperiodic Monotile in Three Dimensions* and *Matching Rules for a
Three-Dimensional Strongly Aperiodic Monotile* (see reference 8 of the top-level README).

The table is a reformatted extract (columns selected, coordinates written as exact fractions); no values were
changed. `make_r44_features.py` rebuilds it from a clone of the upstream repository. The models that read it check
it against the statements of the paper and against `chair44_core.py` (see `models/chair_fibers.py`).
