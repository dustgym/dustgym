"""Material layer of the world model: per-cell regolith strength from the real density field.

Each cell's internal friction angle and cohesion are monotonic functions of its RELATIVE DENSITY
Dr = (rho - RHO_SURFACE) / (RHO_DEEP - RHO_SURFACE), interpolated across the sourced spec ranges
(constants.py / spec section 5.2). The loose endpoint is the repo's nominal surface value (so the field
reproduces the global constant at surface density); the dense endpoint is the spec's compacted value.
The field therefore stiffens with the CONSERVED per-cell density (compacted berms strong, loose spoil
weak) and is grounded in the real density.rf32, not fabricated.

From those it derives the two "what the robot learns" maps the world model calls for:
  - cut_difficulty: cohesion is the interlocking cut resistance (spec section 9, cohesion is "like
    Velcro"), so low-cohesion loose soil is easy to cut.
  - slip_susceptibility: the inverse of the per-cell traction capacity c*A + N*tan(phi) (the repo's own
    developed-thrust form, slip.py), so loose low-strength soil slips more under the same wheel load.

This is the Material layer as a queryable representation + trafficability maps. Threading per-cell
material back INTO the sinkage/slip solver (so it changes the dynamics, not just the prediction) is the
follow-on; the solver currently reads the global constants.
"""
from __future__ import annotations

import numpy as np

from . import constants as K

# Sourced strength endpoints (constants.py / spec section 5.2). LOOSE = the repo's nominal value at the
# surface density (Dr = 0); DENSE = the spec's compacted value (Dr = 1). Friction range 30-50 deg
# (->55 at depth); cohesion 0.1-1.0 kPa. Interpolating in real relative density between these.
PHI_LOOSE_DEG = float(np.rad2deg(K.PHI))   # 37 deg, spec mid-range loose-surface value
PHI_DENSE_DEG = 50.0                        # spec section 5.2 dense end of phi 30-50 deg
COHESION_LOOSE_PA = float(K.COHESION)       # 170 Pa, spec nominal 0.17 kPa
COHESION_DENSE_PA = 1000.0                  # spec section 5.2 dense end of c 0.1-1.0 kPa


def _norm01(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    lo, hi = float(a.min()), float(a.max())
    return (a - lo) / (hi - lo) if hi > lo else np.zeros_like(a)


def relative_density(density) -> np.ndarray:
    """Per-cell relative density Dr in [0,1] from bulk density [kg/m^3] (clamped to the loose-dense range)."""
    rho = np.asarray(density, dtype=float)
    return np.clip((rho - K.RHO_SURFACE) / (K.RHO_DEEP - K.RHO_SURFACE), 0.0, 1.0)


def cell_strength(density_value: float) -> tuple[float, float]:
    """Per-cell (phi_rad, cohesion_pa) for a single bulk density [kg/m^3] -- the scalar form of
    material_fields, for threading the LOCAL material into the slip solver per drive step (so loose
    cells slip more, compacted cells less). Same monotonic interpolation as material_fields."""
    dr = float(np.clip((float(density_value) - K.RHO_SURFACE) / (K.RHO_DEEP - K.RHO_SURFACE), 0.0, 1.0))
    phi_deg = PHI_LOOSE_DEG + dr * (PHI_DENSE_DEG - PHI_LOOSE_DEG)
    cohesion_pa = COHESION_LOOSE_PA + dr * (COHESION_DENSE_PA - COHESION_LOOSE_PA)
    return float(np.deg2rad(phi_deg)), float(cohesion_pa)


def material_fields(density, *, normal_load_n: float = 200.0, contact_area_m2: float = 0.05) -> dict:
    """Per-cell material from the density field. Returns a dict of HxW arrays.

    `normal_load_n` / `contact_area_m2` are a reference wheel normal load and contact patch used only for
    the traction-capacity proxy; they do not affect friction/cohesion. Absolute fields (friction_deg,
    cohesion_pa, traction_capacity_n) are physical; slip_susceptibility and cut_difficulty are normalized
    [0,1] decision/display indices over the field.
    """
    dr = relative_density(density)
    friction_deg = PHI_LOOSE_DEG + dr * (PHI_DENSE_DEG - PHI_LOOSE_DEG)
    cohesion_pa = COHESION_LOOSE_PA + dr * (COHESION_DENSE_PA - COHESION_LOOSE_PA)
    # developed-thrust traction capacity (slip.py form): c * A + N * tan(phi)
    traction_n = cohesion_pa * contact_area_m2 + normal_load_n * np.tan(np.deg2rad(friction_deg))
    return {
        "relative_density": dr,
        "friction_deg": friction_deg,
        "cohesion_pa": cohesion_pa,
        "traction_capacity_n": traction_n,
        "slip_susceptibility": 1.0 - _norm01(traction_n),  # loose -> low traction -> high slip
        "cut_difficulty": _norm01(cohesion_pa),            # cohesion is the interlocking cut resistance
    }


def load_density(scene_dir: str):
    """Read a scene's density.rf32 (row-major C float32) [kg/m^3]."""
    import json
    import os
    m = json.load(open(os.path.join(scene_dir, "metadata.json")))
    g = m["grid"]
    d = np.fromfile(os.path.join(scene_dir, "density.rf32"), dtype="<f4")
    return d.reshape(int(g["height"]), int(g["width"])).astype(float)
