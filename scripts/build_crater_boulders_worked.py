"""Build crater_boulders_WORKED: the crater_boulders scene after the rover drives + digs.

The before/after showcase deliverable -- the EXACT crater_boulders base (same seeds, crater,
Golombek boulders) re-based to the uniform-mantle model and then WORKED by a single drive:
  * wheel TRACKS (compaction, four_wheel_pass) along the whole front-margin path; and
  * a drum EXCAVATION trench (drum_pass) over the middle leg -- the rover lowered its drum
    mid-drive -- with the spoil dumped as a parallel ridge (bulking, mass conserved).
The path threads among the boulders (we do NOT model rover/clast contact yet -- the rover just
passes through, spec note). Rendered side-by-side with the pristine crater_boulders, a busy
low-angle (grazing-sun) scene that reads original-vs-modified terrain at a glance.

Carries the additive §5.2 wheel_tracks (cleat detail) + drum_marks (teeth detail) and the
regolith_model flag that enables the cut-depth / exposed-sublayer albedo. The fresh-albedo
strength is the SOURCED maturity_albedo_ratio (not an eyeball): immature/fresh regolith is
brighter than the space-weathered surface (lunar soil maturity; OMAT, Lucey et al. 2000; Hapke
2001 space-weathering), measured ratios ~1.3-1.8; we use a CONSERVATIVE 1.4.

HONEST LIMITATIONS (documented, not modeled): the excavation OUTCOME is uncertain -- RASSOR
drum action may FLUFF/bulk the regolith (porosity change -> ambiguous reflectance), drop CLODS
(micro-shadowing/roughness, a BRDF effect, not albedo), and disturbed fines may RE-SETTLE and
re-mature the fresh cut quickly (shrinking the contrast). So the depth-graded brightening here
is a CONSERVATIVE first-order maturity model, deliberately subtle, flagged as a hypothesis.

    python scripts/build_crater_boulders_worked.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # noqa: E402

from terrain_authority import constants as K  # noqa: E402
from terrain_authority import procgen  # noqa: E402
from terrain_authority.io_fields import save_scene, write_hillshade_png  # noqa: E402
from terrain_authority.rover import (  # noqa: E402
    build_drum_marks_meta, build_wheel_tracks_meta, drum_pass, four_wheel_pass)

W = H = 256
CELL_M = 0.02
NAME = "crater_boulders_worked"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENE_DIR = os.path.join(ROOT, "samples", NAME)

M0 = K.Z_T * K.RHO_SURFACE          # uniform pristine mantle areal mass [kg/m^2]
MATURITY_ALBEDO_RATIO = 1.4         # sourced fresh/mature reflectance ratio (see docstring)
CUT_DEPTH_FULL_M = 0.06             # removed thickness mapping to full fresh-albedo


def _build_base():
    """EXACT replica of scenes.build_crater_boulders (seed 17 hills + D=2.2 crater + seed 71
    Golombek boulders, bowl-excluded, surface-snapped). Returns (cs, clasts)."""
    cs = procgen.rolling_hills(W, H, CELL_M, seed=17, amplitude_m=0.06, base_cells=3)
    diameter_m = 2.2
    cr, cc = H // 2, W // 2
    procgen.carve_crater(cs, (cr, cc), diameter_m)
    R = 0.5 * diameter_m
    cx, cz = cc * CELL_M, cr * CELL_M
    h = cs.derive_height()
    raw = procgen.sample_boulders(W, H, CELL_M, k=0.08, seed=71)
    clasts: list[dict] = []
    for c in raw:
        x, _y, z = c["center_m"]
        if np.hypot(x - cx, z - cz) < 0.95 * R:
            continue
        col = min(W - 1, max(0, int(round(x / CELL_M))))
        row = min(H - 1, max(0, int(round(z / CELL_M))))
        rad = c["radius_m"]
        buried = c["buried_frac"]
        c["center_m"] = [round(x, 4), round(float(h[row, col]) + rad * (1.0 - 2.0 * buried), 4),
                         round(z, 4)]
        c["id"] = len(clasts)
        clasts.append(c)
    return cs, clasts


def _rebase_uniform_mantle(cs) -> None:
    """datum carries topography, a uniform mantle M0 sits on top; derive_height() unchanged."""
    surface = cs.derive_height()
    cs.density = np.full((H, W), K.RHO_SURFACE, dtype=np.float64)
    cs.datum = surface - K.Z_T
    cs.mass_areal = np.full((H, W), M0, dtype=np.float64)
    cs.state_label[:] = 0
    cs.disturbance[:] = 0.0


def _seg(a, b):
    (ra, ca), (rb, cb) = a, b
    n = int(max(abs(rb - ra), abs(cb - ca))) + 1
    return list(zip(np.linspace(ra, rb, n), np.linspace(ca, cb, n)))


def _heading_at(path, i):
    j0 = max(0, i - 1)
    j1 = min(len(path) - 1, i + 1)
    drow = path[j1][0] - path[j0][0]
    dcol = path[j1][1] - path[j0][1]
    if abs(drow) < 1e-9 and abs(dcol) < 1e-9:
        return 0.0
    return float(np.arctan2(drow, dcol))


def main() -> int:
    os.makedirs(SCENE_DIR, exist_ok=True)
    cs, clasts = _build_base()
    _rebase_uniform_mantle(cs)
    mass_before = cs.total_mass()

    # --- drive path: a shallow arc across the FRONT margin (rows < bowl), among boulders. ---
    # Apex bows toward the front (low row) so the drive stays clear of the fresh bowl.
    path = _seg((64, 32), (48, 128)) + _seg((48, 128), (64, 214))[1:]
    poses = [(path[i], _heading_at(path, i)) for i in range(len(path))]

    # Wheel tracks along the WHOLE drive (compaction, mass-preserving).
    polylines = four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.14)
    apex_heading = _heading_at(path, len(path) // 2)
    wheel_tracks = build_wheel_tracks_meta(polylines, apex_heading, cell_m=CELL_M, width_m=0.18)

    # Drum EXCAVATION over the MIDDLE leg (cols ~92..164): the rover lowered its drum mid-drive.
    swath = [(r, c) for (r, c) in path if 92.0 <= c <= 164.0]
    # Spoil ridge dumped parallel to the trench, offset toward the bowl (+row).
    dump = [(r + 22.0, c) for (r, c) in swath]
    moved_kg = drum_pass(cs, swath, depth_m=0.05, width_m=0.24, dump_rc=dump)
    drum_heading = _heading_at(swath, len(swath) // 2) if len(swath) > 1 else 0.0
    drum_entry = build_drum_marks_meta(swath, drum_heading, drum="front",
                                       depth_m=0.05, width_m=0.24, cell_m=CELL_M)
    mass_after = cs.total_mass()

    az = {"min_rc": [18, 22], "max_rc": [110, 220]}
    x1 = round(W * CELL_M, 4)
    h = cs.derive_height()
    meta = {
        "schema_version": "1.0",
        "scene_name": NAME,
        "producer": "scripts/build_crater_boulders_worked.py (before/after showcase)",
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
        "clasts": clasts,
        "active_zone": az,
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": W, "label": "ROOT"}],
        "wheel_tracks": wheel_tracks,
        "drum_marks": [drum_entry],
        "regolith_model": {
            "uniform_mantle": True,
            "mantle_areal_kg_m2": round(M0, 4),
            "surface_density": K.RHO_SURFACE,
            "regolith_thickness_m": K.Z_T,
            "cut_depth_full_m": CUT_DEPTH_FULL_M,
            "maturity_albedo_ratio": MATURITY_ALBEDO_RATIO,
            "note": "Cut-depth albedo = (M0 - mass_areal)/surface_density. maturity_albedo_ratio "
                    "is a SOURCED fresh/mature reflectance ratio (OMAT/Lucey; Hapke 2001), "
                    "conservative 1.4. Outcome caveats (fluffing/clods/dust re-settle) NOT "
                    "modeled -- this is a first-order maturity hypothesis. See script docstring.",
        },
        "features": ["wheel_tracks", "drum_marks", "cut_depth"],
        "contract_revision": "1.0.2",
        "notes": f"crater_boulders WORKED: same base, driven path (tracks throughout) + a drum "
                 f"trench over the middle leg (spoil ridge). {len(clasts)} clasts (rover passes "
                 f"through; no contact modeled). Mass conserved through the drum inventory.",
    }
    save_scene(SCENE_DIR, cs.fields_dict(), meta)
    write_hillshade_png(h, os.path.join(SCENE_DIR, "preview_hillshade.png"), CELL_M,
                        altdeg=K.SUN_ELEVATION_DEG_POLAR, title=f"{NAME} hillshade")

    print(f"  wrote {NAME}  clasts={len(clasts)}  excavated={moved_kg:.3f} kg  "
          f"mass_drift={abs(mass_after - mass_before):.2e} kg  "
          f"max_cut={float((M0 - cs.mass_areal).max()) / K.RHO_SURFACE * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
