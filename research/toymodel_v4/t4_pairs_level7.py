#!/usr/bin/env python3
"""t4_pairs_level7.py - count the flat pairs of the level-7 patch (2 147 679 tiles) by sorting vertex keys; needs about 3 GB and one minute.
SpectrePatch refuses more than 6 levels, so the check is lifted here by editing the source text; nothing else changes.

The recurrence of t4_patch.PAIR_RECURRENCE, fitted to levels 0 to 4 and checked on levels 5 and 6, predicts 469766 pairs at level 7; this script counts them.

    python3 t4_pairs_level7.py          # prints: 7 2147679 469766
"""
import os
import sys
import types

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _ROOT)
import spectre_operator as so  # noqa: E402

FLAT = 8


def load_patch_class(max_levels=8):
    path = os.path.join(_ROOT, "spectre_patch.py")
    src = open(path).read().replace("levels > 6", "levels > %d" % max_levels)
    mod = types.ModuleType("spectre_patch_unbounded")
    mod.__dict__["__file__"] = path
    exec(compile(src, path, "exec"), mod.__dict__)
    return mod.SpectrePatch


def count(levels):
    P = load_patch_class()(levels, 1.0, 1.0)
    V0 = so.vertices(1.0, 1.0)
    M = P.placements
    X = np.einsum("nij,vj->nvi", M[:, :2, :2], V0) + M[:, None, :2, 2]
    K = np.rint(X * 1e5).astype(np.int64)
    key = (K[:, :, 0] + 10 ** 9) * (2 ** 31) + (K[:, :, 1] + 10 ** 9)
    flat = key.ravel()
    order = np.argsort(flat, kind="stable")
    fs, vid = flat[order], order % 14
    starts = np.flatnonzero(np.r_[True, fs[1:] != fs[:-1]])
    sizes = np.diff(np.r_[starts, len(fs)])
    two = starts[sizes == 2]
    return P.n_tiles, int(((vid[two] == FLAT) & (vid[two + 1] == FLAT)).sum())


if __name__ == "__main__":
    for lv in map(int, sys.argv[1:] or [7]):
        print(lv, *count(lv))
