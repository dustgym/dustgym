#!/usr/bin/env python3
"""Export the sysrev-sourced per-planet terramechanics (terrain_authority/bodies.py) to bodies.json,
so the planet browser LOADS the SAME constants the sim uses (single source of truth) when a body is
chosen -- no hardcoded/divergent copy. Re-run after editing bodies.py.
"""
import dataclasses
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))   # the monorepo root holds terrain_authority/

from terrain_authority import bodies as B  # noqa: E402
from terrain_authority import ipex_specs as S  # noqa: E402
from terrain_authority import constants as K  # noqa: E402

out = {}
for key, b in B.BODIES.items():
    d = dataclasses.asdict(b)
    d["bekker"] = ({"k_c": b.bekker[0], "k_phi": b.bekker[1], "n": b.bekker[2]} if b.bekker else None)
    out[key] = d

# IPEx/energy constants for the browser's build estimate. JS can't import .py, so mirror them here
# from the source of truth (ipex_specs.py + constants.py). Underscore key -> not a body (inert to
# body lookups in the browser + mission_planner). Re-run this script after editing those .py files.
out["_ipex"] = {
    "drum_kg": S.REGOLITH_PER_CYCLE_KG,
    "dig_j_per_kg": round(S.dig_energy_per_kg(), 1),
    "drive_j_per_m": round(S.drive_energy_per_m(), 2),
    "battery_j": round(S.battery_energy_j(), 1),
    "dig_rate_kg_hr": S.DIG_RATE_KG_PER_HR,
    "sinter_enabled": K.SINTER_ENABLED,
}

path = os.path.join(HERE, "bodies.json")
with open(path, "w") as f:
    json.dump(out, f, indent=2)
print(f"wrote {len(out)} bodies -> {path}: {', '.join(b.label for b in B.BODIES.values())}")
