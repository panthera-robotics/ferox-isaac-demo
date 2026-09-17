"""Offline (CPU, no simulator) hand-fidelity checks for the Inspire hand assets of the G1 twin.

Everything here reads a URDF (and optionally its STL meshes) and the versioned embodiment manifest; nothing here
drives a simulator, a robot or writes into a campaign ledger. Values derived from the donor asset are labelled
DONOR; manufacturer figures are NOMINAL; installed measurements are MEASURED; anything else stays null.
"""
import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1]
_ROOT = _TOOLS.parent
for _p in (str(_TOOLS), str(_ROOT)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
