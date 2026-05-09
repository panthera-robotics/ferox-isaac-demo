"""Diagnose which 'utils' module gets imported when test runs."""
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True})
print("[D] SimApp up", flush=True)

import sys
import os

# Show what is already in sys.modules with name 'utils'
existing = [k for k in sys.modules.keys() if k == "utils" or k.endswith(".utils")]
print(f"[D] sys.modules already has these utils-like entries: {len(existing)}", flush=True)
for k in existing[:5]:
    mod = sys.modules.get(k)
    fpath = getattr(mod, "__file__", "<no file>")
    print(f"[D]   {k} -> {fpath}", flush=True)

# Now insert our path and import
sys.path.insert(0, "/workspace/ferox_isaac")
import utils
print(f"[D] utils.__file__ = {utils.__file__}", flush=True)

# Check what attributes utils has
all_attrs = [a for a in dir(utils) if not a.startswith("_")]
print(f"[D] utils has {len(all_attrs)} public attributes", flush=True)
print(f"[D] First 10: {all_attrs[:10]}", flush=True)

target = "setup_cmd_vel_graph"
print(f"[D] hasattr(utils, target): {hasattr(utils, target)}", flush=True)

sim_app.close()
print("[D] DONE", flush=True)
