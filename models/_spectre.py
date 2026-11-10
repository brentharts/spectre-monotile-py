"""_spectre.py - find the Hat and Spectre repository (spectre.py, spectreatlas.py, spectre_mystic_xbond.py, ...) for the figure scripts.
$SPECTRE_DIR, ../spectre, or /home/claude/spectre.  Imported, not copied; load() returns None if it is absent."""
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(_HERE)
_CAND = [os.environ.get("SPECTRE_DIR", ""), os.path.join(_ROOT, "..", "spectre"), "/home/claude/spectre"]


def load():
    """-> directory of the repository (put first on sys.path), or None."""
    for base in _CAND:
        if base and os.path.isfile(os.path.join(base, "spectre_mystic_xbond.py")):
            base = os.path.realpath(base)
            if base in sys.path:
                sys.path.remove(base)
            sys.path.insert(0, base)
            res = os.path.join(base, "research")           # some modules do sys.path.insert(0, "research"), relative to the working directory
            if os.path.isdir(res) and res not in sys.path:
                sys.path.append(res)
            return base
    return None
