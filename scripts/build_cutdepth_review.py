"""Build the CUT-DEPTH ALBEDO review scene (render_fidelity: exposed-sublayer brightening).

A focused REVIEW deliverable (not a canonical scenes.py sample) to judge the cut-depth /
exposed-sublayer albedo term in isolation before stacking the Hapke BRDF on top. Physical
story: freshly EXCAVATED regolith exposes immature sub-surface material that is BRIGHTER than
the space-weathered surface; the brightening should be CONTINUOUS with cut depth, not a flat
per-state tint (asce-es-2024-isru-pilot-excavator-wheel-testing.pdf / regolith maturity).

THE SIGNAL (compaction-immune, topography-immune): this scene uses the "uniform mantle" model
-- ``datum`` carries the macro topography and a UNIFORM regolith mantle of areal mass
M0 = Z_T * RHO_SURFACE sits on top. Then the excavated areal-mass deficit (M0 - mass_areal)
is exactly the removed mass, and (M0 - mass_areal)/RHO_SURFACE is the removed THICKNESS [m].
  * Excavation removes mass_areal      -> deficit > 0 -> brightens (graded by depth).
  * Compaction raises density, mass_areal UNCHANGED -> deficit 0 -> NOT brightened.
  * Natural topography lives in datum, mass_areal stays M0 -> VIRGIN never brightens.
Re-basing any surface to this model leaves derive_height() unchanged (datum + M0/rho = surface).

The scene is laid out to MAKE THE EFFECT LEGIBLE in one still:
  * a drum trench in a 4-step DEPTH STAIRCASE (2 -> 9.5 cm) -> a visible brightness gradient;
  * a parallel TREAD compaction band (same disturbance bump, mass untouched) -> the IMMUNITY
    proof: it darkens (compacted) but does NOT brighten;
  * a SPOIL heap (dumped excavate, mass ADDED) -> deficit clamped 0 -> not brightened either.

    python scripts/build_cutdepth_review.py
then render ON vs OFF (the cut_depth_enabled gate follows the regolith_model metadata):
    cd godot_sidecar && ./render_layers.sh -- --scene ../samples/cutdepth_review --layers terrain ...
"""

from __future__ import annotations

import json
import os
import sys

import numpy as np

# Run standalone (python scripts/build_cutdepth_review.py) by putting the repo root on the path.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # noqa: E402

from terrain_authority import constants as K  # noqa: E402
from terrain_authority import procgen  # noqa: E402
from terrain_authority.io_fields import save_scene, write_hillshade_png  # noqa: E402
from terrain_authority.rover import drum_pass, straight_path, wheel_pass  # noqa: E402

W = H = 256
CELL_M = 0.02                      # 2 cm -> 5.12 m patch (same extent as the canonical scenes)
NAME = "cutdepth_review"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(ROOT, "samples", NAME)

M0 = K.Z_T * K.RHO_SURFACE          # pristine mantle areal mass [kg/m^2] = 0.12 * 1300 = 156
CUT_DEPTH_FULL_M = 0.095            # deepest staircase step -> full fresh-albedo at this depth


def _rebase_to_uniform_mantle(cs) -> None:
    """In place: keep the surface, but split it into datum (topography) + a UNIFORM mantle.

    datum := surface - Z_T (bedrock follows the macro relief); density := RHO_SURFACE;
    mass_areal := Z_T * RHO_SURFACE everywhere. derive_height() is unchanged (datum + M0/rho).
    """
    surface = cs.derive_height()
    cs.density = np.full((H, W), K.RHO_SURFACE, dtype=np.float64)
    cs.datum = surface - K.Z_T
    cs.mass_areal = np.full((H, W), M0, dtype=np.float64)
    cs.state_label[:] = 0          # VIRGIN
    cs.disturbance[:] = 0.0


def main() -> int:
    os.makedirs(SCENE_DIR, exist_ok=True)

    # Gentle rolling base for some relief, then re-base to the uniform-mantle model.
    cs = procgen.rolling_hills(W, H, CELL_M, seed=7, amplitude_m=0.05, base_cells=5)
    _rebase_to_uniform_mantle(cs)
    mass_before = cs.total_mass()

    # --- excavation: FOUR separate pits at increasing depth (a depth staircase) ---
    # Separated by gaps so the disc-swept drum footprints DON'T overlap (overlapping
    # drum_pass calls double-cut the shared cells). Each pit then reads its own clean depth,
    # so the render shows four distinct fresh-albedo levels = the continuous depth->brightness
    # mapping. cols 48..208; swath length 24, drum half-width 6 cells -> ~8-12 col gaps.
    dig_r = 96
    c0, c1 = 48, 208
    width_m = 0.24                         # ~12-cell drum swath (half_w 6 cells)
    pits = [(48, 0.02), (92, 0.045), (136, 0.07), (180, 0.095)]  # (col_start, depth_m)
    for s0, d in pits:
        s1 = min(s0 + 24, c1)
        swath = straight_path(dig_r, s0, dig_r, s1, step_cells=1)
        drum_pass(cs, swath, depth_m=d, width_m=width_m, dump_rc=None)  # cut to drum inventory

    # --- spoil heap: dump the excavated inventory as a SPOIL mound (mass ADDED -> deficit<0) ---
    heap_r, heap_c, heap_rad = 150, 168, 18
    rr = np.arange(H)[:, None] - heap_r
    cc = np.arange(W)[None, :] - heap_c
    heap_mask = (rr ** 2 + cc ** 2) <= heap_rad ** 2
    cs.dump_from_inventory(heap_mask, cs.drum_inventory)

    # --- compaction band: a TREAD rut parallel to the trench (IMMUNITY proof) ---
    # density up, mass_areal UNTOUCHED -> cut-depth deficit 0 -> darkens but does NOT brighten.
    tread_r = 132
    wheel_pass(cs, straight_path(tread_r, c0, tread_r, c1, step_cells=1),
               wheel_width_m=0.24, compaction=0.16)

    mass_after = cs.total_mass()

    az = {"min_rc": [60, 30], "max_rc": [175, 222]}
    x1 = round(W * CELL_M, 4)
    h = cs.derive_height()
    meta = {
        "schema_version": "1.0",
        "scene_name": NAME,
        "producer": "scripts/build_cutdepth_review.py (cut-depth albedo review; uniform mantle)",
        "grid": {"width": W, "height": H, "cell_m": CELL_M, "order": "row-major-C"},
        "world_bounds_m": {"x0": 0.0, "y0": 0.0, "x1": x1, "y1": x1},
        "gravity_m_s2": K.g,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1", "enum": K.STATE_NAMES},
        },
        "ice_present": False,
        "height_range_m": [round(float(h.min()), 5), round(float(h.max()), 5)],
        "clasts": [],
        "active_zone": az,
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": W, "label": "ROOT"}],
        # ADDITIVE: declares the uniform-mantle model so the renderer enables the cut-depth term.
        "regolith_model": {
            "uniform_mantle": True,
            "mantle_areal_kg_m2": round(M0, 4),
            "surface_density": K.RHO_SURFACE,
            "regolith_thickness_m": K.Z_T,
            "cut_depth_full_m": CUT_DEPTH_FULL_M,
            "maturity_albedo_ratio": 1.4,   # sourced fresh/mature ratio (OMAT/Lucey; Hapke 2001)
            "note": "datum carries topography; uniform mantle M0 on top. Cut-depth signal = "
                    "(M0 - mass_areal)/surface_density [m]. Compaction- and topography-immune.",
        },
        "features": ["cut_depth"],
        "notes": "Cut-depth albedo REVIEW. Drum trench in a 2->9.5 cm depth staircase (row 96), "
                 "a parallel TREAD compaction band (row 132, immunity proof: darkens, does NOT "
                 "brighten), and a SPOIL heap (mass added, not brightened). Mass conserved.",
    }
    save_scene(SCENE_DIR, cs.fields_dict(), meta)
    write_hillshade_png(h, os.path.join(SCENE_DIR, "preview_hillshade.png"), CELL_M,
                        altdeg=K.SUN_ELEVATION_DEG_POLAR, title=f"{NAME} hillshade")

    drift = abs(mass_after - mass_before)
    # Report the deepest-cut deficit as a sanity check of the signal.
    ma = cs.mass_areal
    deepest = float((M0 - ma).max())
    print(f"  wrote {NAME}  M0={M0:.1f} kg/m^2  max_deficit={deepest:.1f} kg/m^2 "
          f"(= {deepest / K.RHO_SURFACE * 100:.1f} cm cut)  mass_drift={drift:.2e} kg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
