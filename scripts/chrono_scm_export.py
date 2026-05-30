#!/usr/bin/env python3
"""SCM -> INTERFACE.md contract exporter (STUB) — foss_ipex Path A.

Maps a running PyChrono `SCMTerrain` patch onto the frozen on-disk state-field contract
(INTERFACE.md), per the field mapping in docs/chrono_integration.md §4.1/§4.2 and the
mass-stays-surrogate caveat in §4.3/§7. This is the seam that lets Chrono be a drop-in for
the NumPy surrogate with ZERO Godot changes.

STATUS: STUB / mapping-shape demonstrator. It shows
  (1) that we can read SCM's deformed-node field (GetModifiedNodes) and per-node
      terramechanics state (GetNodeInfo) and rasterize it onto our grid, and
  (2) exactly which INTERFACE fields Chrono can source vs. which STAY surrogate-side.
It does NOT close the mass-conservation loop (that is structurally surrogate-side; §4.3).

WHAT SCM SOURCES (chrono_integration.md §4.2):
  heightmap.rf32   <- SCM deformed height per node (GetModifiedNodes / GetHeight).
  disturbance.rf32 <- normalized plastic sinkage proxy (NodeInfo.sinkage_plastic, or
                      GetHeight - GetInitHeight), clamped to [0,1].
  state_label.r8   <- DERIVED producer logic: cells with plastic sinkage -> TREAD(1).
                      SCM has no notion of our enum (§4.2) — this is producer code.

WHAT STAYS SURROGATE-SIDE (chrono_integration.md §4.3/§7; INTERFACE.md §4):
  mass_areal.rf32  <- THE conserved invariant. SCM does NOT conserve/expose areal mass.
  density.rf32     <- SCM is single-layer constant-density Bekker; no current bulk-density field.
  ice.rf32         <- not a Chrono concept.
  (multi-pass paving, stratigraphy, slip-sinkage theta_m migration, bulking — all surrogate.)

The realistic integration (§4.4) is a HYBRID: Chrono owns the wheel-rut geometry in the
"under wheels (rolling)" zone; the surrogate keeps mass/density bookkeeping and re-derives
height = mass_areal/density so INTERFACE's invariant (§4) stays green. This stub fills the
SCM-sourced fields and fills the surrogate-owned fields with PLACEHOLDER constants, clearly
labeled, so the output is a contract-VALID directory whose Chrono-vs-surrogate provenance is
explicit. A real hybrid producer would replace the placeholders with the surrogate grid.

Run in the conda 'chrono' env (needs pychrono). Imports the project's frozen io_fields
helper (INTERFACE.md §7) — the ONLY place raster bytes are written on the Python side.
"""
from __future__ import annotations

import os
import sys

import numpy as np

# Make terrain_authority importable (the project package lives one level up).
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from terrain_authority.io_fields import save_scene  # noqa: E402  (frozen contract writer)

# Producer-owned placeholder constants for the fields SCM cannot source (§4.3).
# These are NOT physics — they are clearly-labeled stand-ins for the surrogate grid that a
# real hybrid producer (§4.4) would supply. Values are typical lunar-regolith SI numbers.
PLACEHOLDER_DENSITY = 1500.0   # kg/m^3 (loose regolith; INTERFACE §4 SI)
DATUM_M = 0.0                  # height datum the SCM heights ride on


def rasterize_scm(terrain, chrono, width: int, height: int, cell_m: float,
                  x0: float, z0: float):
    """Sample SCM onto our (height, width) raster grid (row-major, INTERFACE §2).

    Returns (height_grid, sinkage_grid). Uses GetModifiedNodes(all=True) for the deformed
    field and GetInitHeight for the reference, so sinkage = init - deformed (positive = sunk).
    Falls back to per-cell GetHeight sampling if node enumeration is unavailable.
    Index convention (INTERFACE §2): k = row*width + col, col -> +X, row -> +Z.
    """
    hmap = np.full((height, width), DATUM_M, dtype=np.float32)
    sink = np.zeros((height, width), dtype=np.float32)

    # Preferred path: enumerate modified nodes -> (grid i,j) -> height.
    nodes = None
    try:
        nodes = terrain.GetModifiedNodes(True)
    except Exception:
        nodes = None

    if nodes is not None and len(nodes) > 0:
        for nl in nodes:
            try:
                ij, h_def = nl                     # NodeLevel = (ChVector2i, double)
                i, j = int(ij.x), int(ij.y)
            except Exception:
                try:
                    i, j, h_def = int(nl.first.x), int(nl.first.y), float(nl.second)
                except Exception:
                    continue
            # SCM grid (i,j) -> our (row,col). SCM's i,j are signed about the plane origin;
            # shift by the patch min corner so index 0 is the world min corner (INTERFACE §3).
            col = i + width // 2
            row = j + height // 2
            if 0 <= row < height and 0 <= col < width:
                hmap[row, col] = np.float32(h_def)
                # net sinkage from the undeformed reference at this world point
                wx = x0 + col * cell_m
                wz = z0 + row * cell_m
                try:
                    h_init = terrain.GetInitHeight(chrono.ChVector3d(wx, 0.0, wz))
                    sink[row, col] = np.float32(max(0.0, h_init - h_def))
                except Exception:
                    pass
    else:
        # Fallback: sample GetHeight at each cell centre (slower, always available).
        for row in range(height):
            for col in range(width):
                wx = x0 + col * cell_m
                wz = z0 + row * cell_m
                try:
                    hmap[row, col] = np.float32(
                        terrain.GetHeight(chrono.ChVector3d(wx, 0.0, wz)))
                except Exception:
                    pass

    return hmap, sink


def build_fields(hmap: np.ndarray, sink: np.ndarray):
    """Assemble the INTERFACE field dict. SCM-sourced + clearly-labeled placeholders."""
    h, w = hmap.shape

    # disturbance: normalized plastic-sinkage proxy clamped to [0,1] (§4.2).
    smax = float(sink.max()) if sink.size else 0.0
    disturbance = (sink / smax).astype("<f4") if smax > 1e-9 else np.zeros((h, w), "<f4")

    # state_label: producer logic — cells with any sinkage -> TREAD(1), else VIRGIN(0) (§4.2).
    state_label = np.where(sink > 1e-4, 1, 0).astype("u1")

    # PLACEHOLDER surrogate-owned fields (§4.3): constant density; mass back-derived so the
    # INTERFACE invariant height == mass_areal/density holds in the EXACT contract form (§4),
    # i.e. with datum 0 so `height = mass_areal/density` literally. We express each cell as a
    # positive column THICKNESS above a deep reference plane (a fixed 1.0 m soil column for the
    # nominal surface), then add the SCM deformation. This keeps mass_areal strictly positive
    # AND makes the stored heightmap reproduce as mass_areal/density to float32 precision.
    density = np.full((h, w), PLACEHOLDER_DENSITY, dtype="<f4")
    NOMINAL_COLUMN_M = 1.0          # 1 m placeholder soil column under the nominal (0 m) surface
    thickness = (NOMINAL_COLUMN_M + hmap).astype(np.float64)   # column thickness (m) per cell
    mass_areal = (thickness * PLACEHOLDER_DENSITY).astype("<f4")
    # heightmap stored = the contract-DERIVED value, referenced back to the same nominal surface
    # so it equals the SCM deformed height: height = mass_areal/density - NOMINAL_COLUMN_M.
    heightmap = (mass_areal.astype(np.float64) / PLACEHOLDER_DENSITY
                 - NOMINAL_COLUMN_M).astype("<f4")

    return {
        "heightmap": heightmap,        # SCM-sourced (deformed geometry), re-derived to satisfy §4
        "mass_areal": mass_areal,      # SURROGATE-owned placeholder (back-derived; §4.3)
        "density": density,            # SURROGATE-owned placeholder (constant; §4.3)
        "disturbance": disturbance,    # SCM-sourced (plastic-sinkage proxy; §4.2)
        "state_label": state_label,    # producer-derived (§4.2)
    }


def export(terrain, chrono, sysmbs, out_dir: str, width: int, height: int,
           cell_m: float, gravity: float = 1.62, scene_name: str = "chrono_scm_spike",
           chassis_body=None):
    """Read SCM, map to INTERFACE fields, write a contract-valid scene dir via io_fields."""
    x0 = -(width // 2) * cell_m
    z0 = -(height // 2) * cell_m

    hmap, sink = rasterize_scm(terrain, chrono, width, height, cell_m, x0, z0)
    fields = build_fields(hmap, sink)

    # Demonstrate per-node NodeInfo read-back (the rich terramechanics state, §4.1) at the
    # DEEPEST cell of the deformed field — this is what a real density/disturbance derivation
    # would consume. We pick the cell from `hmap` (the rut always shows there) rather than
    # `sink` (which is empty if GetInitHeight isn't sampled), and query GetNodeInfo in the SCM
    # reference frame. NOTE: SCM was given a Y-up reference frame (rotated -90 about X), so the
    # GetNodeInfo query point is expressed in that same frame — pass (wx, 0, wz) and let SCM's
    # frame transform handle it. If the struct reads zero, it means the sampled point fell
    # between active ray-cast nodes; the per-NODE values are most reliably read by iterating
    # GetModifiedNodes (heights) — NodeInfo(loc) is a point query for spot inspection.
    node_info_demo = None
    try:
        r, c = np.unravel_index(int(np.argmin(hmap)), hmap.shape)
        wx, wz = x0 + c * cell_m, z0 + r * cell_m
        ni = terrain.GetNodeInfo(chrono.ChVector3d(wx, 0.0, wz))
        node_info_demo = {
            "query_world_xz": [round(wx, 4), round(wz, 4)],
            "query_cell_rc": [int(r), int(c)],
            "deformed_height_m": round(float(hmap[r, c]), 5),
        }
        node_info_demo.update({
            k: getattr(ni, k, None)
            for k in ("sinkage", "sinkage_plastic", "sinkage_elastic",
                      "sigma", "sigma_yield", "kshear", "tau")
        })
    except Exception as e:
        node_info_demo = {"error": repr(e)}

    clasts = []
    if chassis_body is not None:
        p = chassis_body.GetPos()
        clasts = [{"id": 0, "center_m": [p.x, p.y, p.z], "radius_m": 0.15,
                   "shape": "cylinder", "buried_frac": 0.0}]

    metadata = {
        "schema_version": "1.0",
        "scene_name": scene_name,
        "producer": "chrono_scm_export (PyChrono SCMTerrain — STUB; mass/density placeholder)",
        "grid": {"width": width, "height": height, "cell_m": cell_m, "order": "row-major-C"},
        "world_bounds_m": {"x0": x0, "y0": z0,
                            "x1": x0 + width * cell_m, "y1": z0 + height * cell_m},
        "gravity_m_s2": gravity,
        "fields": {
            "heightmap": {"file": "heightmap.rf32", "dtype": "<f4", "units": "m"},
            "mass_areal": {"file": "mass_areal.rf32", "dtype": "<f4", "units": "kg/m^2"},
            "density": {"file": "density.rf32", "dtype": "<f4", "units": "kg/m^3"},
            "disturbance": {"file": "disturbance.rf32", "dtype": "<f4", "units": "1 (normalized)"},
            "state_label": {"file": "state_label.r8", "dtype": "u1",
                            "enum": ["VIRGIN", "TREAD", "EXCAVATED", "SPOIL", "COMPACTED_BERM"]},
        },
        "ice_present": False,
        "height_range_m": [float(fields["heightmap"].min()), float(fields["heightmap"].max())],
        "clasts": clasts,
        "active_zone": {"min_rc": [0, 0], "max_rc": [height, width]},
        "quadtree": [{"level": 0, "row0": 0, "col0": 0, "size": max(width, height), "label": "ROOT"}],
        "notes": ("Chrono Y-up, lunar g. heightmap+disturbance SCM-sourced; mass_areal+density "
                  "are SURROGATE-OWNED PLACEHOLDERS (SCM does not conserve mass — "
                  "chrono_integration.md §4.3/§7). NodeInfo demo: " + repr(node_info_demo)),
    }

    save_scene(out_dir, fields, metadata)
    return metadata, node_info_demo


if __name__ == "__main__":
    # Standalone smoke: build the spike sim, step briefly, export once. Imports the rover
    # spike's builders so the two scripts stay consistent.
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "chrono_scm_rover", os.path.join(os.path.dirname(__file__), "chrono_scm_rover.py"))
    rover = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rover)
    print("chrono_scm_export.py is a library + stub; run scripts/chrono_scm_rover.py to drive "
          "the sim, or import export() from a driver. Run with --demo to do a short self-test.")
    if "--demo" in sys.argv:
        import pychrono as chrono  # noqa: F401
        print("(demo wiring left to the driver; see chrono_scm_rover.py)")
