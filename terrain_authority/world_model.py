"""terrain_authority.world_model: the dustgym package's coherent world-model surface.

Ties the five world-model layers (see docs/world_model.md) to their package modules, so one import
gives you the whole model for terrain *transformation*. The physics-side layers (Geometry, Material,
Physics, Task) ship in this package; the perception-side layers (the map-channel scorer and the
per-cell Uncertainty field) live in the app layer (scripts/ros2_bridge, planet_browser) because they
need the Godot render and the planner, and are referenced here rather than imported.

The design call (docs/world_model.md): conserved physics for the DYNAMICS (exact, sub-ms, unhackable;
model-based search beats model-free RL) + a thin learned model only for PERCEPTION. So the package
exposes the conserved model directly; learning is reserved for the observation branch.
"""
from __future__ import annotations

import numpy as np

from . import column_state, constants, drive, material, slip, terramechanics  # noqa: F401  (surface re-exports)

#: layer -> where it lives (package module, or the app layer that uses the package).
LAYERS = {
    "geometry": "column_state (heightmap + slope) + real LOLA DEM ingest (dem_import)",
    "material": "material (per-cell friction/cohesion + cut/slip maps; threaded into drive.drive_step)",
    "physics": "terramechanics (Bekker sinkage) + slip (traction + slip ladder); the conserved transition S(t+1)=f(S,a)",
    "task": "target heightmap + conserved cut/fill (app: planet_browser/mission_planner + structures)",
    "uncertainty": "autonomy Belief (pose/energy/drum sigma) + the map-channel per-cell sigma (app: scripts/ros2_bridge)",
}


def geometry(cs) -> tuple[np.ndarray, np.ndarray]:
    """Geometry layer: (heightmap [m], slope [deg]) for a ColumnState."""
    h = cs.derive_height()
    gy, gx = np.gradient(h, cs.cell_m)
    return h, np.degrees(np.arctan(np.hypot(gx, gy)))


def material_layer(cs) -> dict:
    """Material layer: per-cell strength + trafficability from the ColumnState's conserved density."""
    return material.material_fields(cs.density)


def earthwork(cs, target_height) -> dict:
    """Task layer: the cut/fill volume to reach a target surface (positive = cut, negative = fill) [m^3]."""
    h = cs.derive_height()
    diff = h - np.asarray(target_height, dtype=float)
    area = cs.cell_m ** 2
    return {"cut_m3": float(np.clip(diff, 0.0, None).sum() * area),
            "fill_m3": float(np.clip(-diff, 0.0, None).sum() * area)}


def describe() -> dict:
    """The five-layer map (layer -> package/app module). The world model at a glance."""
    return dict(LAYERS)
