"""_qgd.py - find the Quantum-Gravity-Dynamics fork (directional/ modules) for the figure scripts.  $QGD_DIR, ../Quantum-Gravity-Dynamics, or /home/claude/Quantum-Gravity-Dynamics.
The modules are imported, not copied; if the fork is absent, load() returns None and the figure scripts draw only what is closed-form."""
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
_ROOT = os.path.dirname(_HERE)
_CAND = [os.environ.get("QGD_DIR", ""), os.path.join(_ROOT, "..", "Quantum-Gravity-Dynamics"), "/home/claude/Quantum-Gravity-Dynamics"]


def load():
    """-> directory of the fork's directional/ package on sys.path, or None."""
    for base in _CAND:
        d = os.path.join(base, "directional") if base else ""
        if d and os.path.isfile(os.path.join(d, "hex_directions.py")):
            if d not in sys.path:
                sys.path.insert(0, d)
            return d
    return None
