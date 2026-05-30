#!/usr/bin/env python3
"""Minimal PyChrono wheeled body on an SCM deformable-terrain patch — foss_ipex Path A.

This is the first *real physics authority* spike for the project (README §4 row #2;
docs/chrono_integration.md §2.3, §5, §7). It stands a single rigid wheel-like body on a
small `SCMTerrain` patch at **lunar gravity, Chrono run Y-up** so Chrono body coordinates
map 1:1 to Godot/field-space (INTERFACE.md §3, chrono_integration.md §5), rolls/sinks it a
few hundred steps headless, and proves we can read back the deformed wheel-rut node data
that the eventual contract exporter needs (chrono_integration.md §4.1).

It deliberately does NOT yet build a full Chrono::Vehicle (that needs an authored vehicle
JSON / kinematic tree — chrono_integration.md gotcha #3, no URDF import). A single rigid
body driven into the soil is enough to exercise SCM ray-casting, sinkage, bulldozing and
the GetModifiedNodes / NodeInfo read-back path that §4 maps onto INTERFACE.md.

Run (in the dedicated py3.12 conda env, NOT the project .venv — gotcha #2):
    eval "$(/home/john/text-generation-webui/installer_files/conda/bin/conda shell.bash hook)"
    conda activate chrono
    python scripts/chrono_scm_rover.py

Headless: this script is windowless (no Irrlicht), so no display / xvfb is needed
(gotcha #4). It prints step diagnostics and a summary of modified SCM nodes.

CLASS NAME (gotcha #1): pychrono 10.0.0 exposes `SCMTerrain`, not the older
`SCMDeformableTerrain`. We resolve whichever the installed build provides so the spike is
robust across pins.
"""
from __future__ import annotations

import math
import sys

import pychrono as chrono
import pychrono.vehicle as veh

# --- grid geometry: match INTERFACE.md / metadata.json so SCM nodes line up with our
#     raster 1:1 (chrono_integration.md §4.2). The frozen sample is 256x256 @ 2 cm = 5.12 m.
#     For a fast spike we use a SMALL patch (cell still 2 cm) to keep ray-cast cost down. ---
CELL_M = 0.02          # 2 cm cell — matches INTERFACE grid.cell_m
PATCH_M = 1.28         # 1.28 m square -> 64x64 nodes (small, fast; same cell size as the contract)
LUNAR_G = 1.62         # m/s^2, Moon (chrono_integration.md §5; INTERFACE metadata.gravity_m_s2)
DT = 1e-3              # 1 ms step
N_STEPS = 400          # a few hundred steps, per the task


def resolve_scm_terrain():
    """Return the SCM terrain class for the installed pychrono build (gotcha #1)."""
    if hasattr(veh, "SCMTerrain"):
        return veh.SCMTerrain, "SCMTerrain"
    if hasattr(veh, "SCMDeformableTerrain"):
        return veh.SCMDeformableTerrain, "SCMDeformableTerrain"
    raise RuntimeError(
        "Neither SCMTerrain nor SCMDeformableTerrain found in pychrono.vehicle — "
        f"available SCM-ish names: {[n for n in dir(veh) if 'SCM' in n]}"
    )


def main() -> int:
    # pychrono 10.0.0 does not expose __version__; report the build via the module file.
    print(f"pychrono module: {chrono.__file__}")

    # --- system: SMC (penalty) contact, Y-up, lunar gravity (chrono_integration.md §2.3/§5) ---
    sysmbs = chrono.ChSystemSMC()
    # Bullet collision system — matches the working stock demo_VEH_DeformableSoil.py.
    sysmbs.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sysmbs.SetGravitationalAcceleration(chrono.ChVector3d(0, -LUNAR_G, 0))  # Y-up, 1/6 g
    print(f"gravity set to (0, -{LUNAR_G}, 0)  [Y-up, lunar]")

    # --- a single rigid 'wheel' body resting just above the soil so it sinks under gravity.
    #     A cylinder is the cheapest stand-in for a wheel/drum to make SCM ray-cast & rut. ---
    wheel_radius = 0.15    # 15 cm
    wheel_width = 0.12     # 12 cm
    wheel_mass = 25.0      # kg (a chunky wheel/corner mass)

    wheel = chrono.ChBody()
    wheel.SetMass(wheel_mass)
    # Spin axis along Z (field-z), wheel rolls in +X; sits centered, hub just above surface.
    wheel.SetPos(chrono.ChVector3d(0.0, wheel_radius + 0.005, 0.0))
    # Orient the cylinder (default Chrono cylinder axis is Y) so its axis lies along Z:
    wheel.SetRot(chrono.QuatFromAngleX(math.pi / 2.0))
    sysmbs.AddBody(wheel)

    # Collision + visual cylinder. SCM ray-casts against the body's collision geometry.
    mat = chrono.ChContactMaterialSMC()
    mat.SetFriction(0.8)
    mat.SetYoungModulus(1e7)
    try:
        wheel.EnableCollision(True)
    except AttributeError:
        wheel.SetCollide(True)  # older API name
    ct_cyl = chrono.ChCollisionShapeCylinder(mat, wheel_radius, wheel_width)
    wheel.AddCollisionShape(ct_cyl)

    # Give it a small forward + spin kick so it actually carves a rut, not just sinks.
    wheel.SetPosDt(chrono.ChVector3d(0.25, 0.0, 0.0))     # 0.25 m/s in +X (field x)
    wheel.SetAngVelLocal(chrono.ChVector3d(0.0, 0.0, -1.6))  # spin about local axis -> rolling

    # --- SCM deformable terrain patch (chrono_integration.md §2.3/§4) ---
    SCMClass, scm_name = resolve_scm_terrain()
    print(f"using terrain class: {scm_name}")
    terrain = SCMClass(sysmbs)
    # VERIFIED against the shipped stock demo (demo_VEH_DeformableSoil.py, pychrono 10.0.0):
    # SCM defaults to a Z-up ISO reference frame. To run in our Y-up global frame we rotate
    # the TERRAIN frame -90 deg about X (chrono_integration.md §5; the demo's documented trick).
    # The 10.0.0 method is SetReferenceFrame(ChCoordsysd) — NOT the older SetPlane().
    terrain.SetReferenceFrame(
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(-math.pi / 2.0)))

    # Bekker / Janosi params — JSC-1A-ish lunar regolith analogue (chrono_integration.md §2.3).
    # SetSoilParameters(Kphi, Kc, n, Mohr_cohesion[Pa], Mohr_friction[deg], Janosi_K[m],
    #                    elastic_K[Pa/m], damping_R)
    terrain.SetSoilParameters(
        0.2e6,   # Bekker_Kphi
        0.0,     # Bekker_Kc
        1.0,     # Bekker_n
        170.0,   # Mohr_cohesion (Pa)  ~0.17 kPa
        35.0,    # Mohr_friction (deg)
        0.018,   # Janosi shear K (m)  ~1.8 cm
        2e8,     # elastic_K (Pa/m)
        3e4,     # damping_R
    )

    # Bulldozing -> accumulated displaced soil (berm/rut). Off by default (§2.3).
    try:
        terrain.EnableBulldozing(True)
    except AttributeError:
        pass

    # Regular grid: Initialize(sizeX, sizeY, delta). delta = cell size (m).
    terrain.Initialize(PATCH_M, PATCH_M, CELL_M)
    nodes_per_side = int(round(PATCH_M / CELL_M))
    print(f"SCM patch {PATCH_M} x {PATCH_M} m, cell {CELL_M} m -> ~{nodes_per_side}x{nodes_per_side} nodes")

    # --- step the sim headless ---
    # VERIFIED stepping pattern (stock demo_VEH_DeformableSoil.py): SCMTerrain is registered
    # as a system component, so a single sys.DoStepDynamics(dt) advances the terrain too — do
    # NOT also call terrain.Synchronize/Advance here or the soil double-steps. (The explicit
    # Synchronize/Advance pattern in chrono_integration.md §2.3 is the Chrono::Vehicle driver
    # loop, where the vehicle is advanced separately; for a bare SCMTerrain+body, one step.)
    print(f"stepping {N_STEPS} steps @ dt={DT}s (t_end={N_STEPS*DT:.3f}s)...")
    for i in range(N_STEPS):
        t = sysmbs.GetChTime()
        sysmbs.DoStepDynamics(DT)
        if i % 50 == 0 or i == N_STEPS - 1:
            p = wheel.GetPos()
            # GetHeight signature varies; try point overload then x,y.
            try:
                h = terrain.GetHeight(chrono.ChVector3d(p.x, p.y, p.z))
            except Exception:
                h = float("nan")
            print(f"  step {i:4d} t={t:6.3f}s  wheel=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f})  "
                  f"terrainH@wheel={h:+.5f}")

    # --- read back deformed-node data (the exporter's input; chrono_integration.md §4.1) ---
    try:
        modified = terrain.GetModifiedNodes(True)   # all_nodes=True -> full deformed field
        n_mod = len(modified)
    except Exception as e:  # signature / availability differs across builds
        modified = None
        n_mod = -1
        print(f"  GetModifiedNodes unavailable/failed: {e!r}")

    # Diagnostics counters if present.
    for getter in ("GetNumRayHits", "GetNumRayCasts", "GetNumContactPatches", "GetNumErosionNodes"):
        fn = getattr(terrain, getter, None)
        if fn:
            try:
                print(f"  {getter}() = {fn()}")
            except Exception as e:
                print(f"  {getter}() failed: {e!r}")

    if n_mod >= 0:
        print(f"GetModifiedNodes(all=True) returned {n_mod} nodes")
        # Show the deepest few (most-sunk) so we can SEE the rut in the read-back.
        sample = []
        for nl in modified:
            try:
                ij, height = nl
                sample.append((ij.x, ij.y, height))
            except Exception:
                # NodeLevel may be a struct; fall back to attrs.
                try:
                    sample.append((nl.first.x, nl.first.y, nl.second))
                except Exception:
                    break
        if sample:
            sample.sort(key=lambda s: s[2])  # lowest height first (deepest rut)
            print("  deepest 5 modified nodes (i, j, height_m):")
            for ij_i, ij_j, hgt in sample[:5]:
                print(f"    node (i={ij_i:4d}, j={ij_j:4d})  height={hgt:+.5f} m")

    print("OK: SCM rover spike stepped without fatal error.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
