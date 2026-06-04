#!/usr/bin/env python3
"""LIVE Chrono producer (rigid-body half of the §4.4 hybrid) — foss_ipex Path A, core-only.

The frozen architecture splits authority: **Chrono owns rigid-body dynamics (rover, clasts); the NumPy
surrogate owns the deformable-regolith MASS** (chrono_integration.md §4.3/§4.4). This module is the real,
RUNNABLE rigid-body producer: it settles rigid clasts (rocks) on a contact surface under the body's
gravity with a genuine Chrono multibody solve, and exports their rest poses — the clast field the
surrogate consumes. It REPLACES the placeholder "single rigid cylinder, PLACEHOLDER_DENSITY" demonstrator
with actual Chrono dynamics.

Scope (honest): this is Chrono CORE (`ChSystemSMC`, rigid bodies, penalty contact). The SCM
deformable-terrain SOIL oracle (`chrono_scm_export.py`, the Bekker pressure-sinkage ground-truth for the
FIX-1/FIX-2 K_PHI calibration) lives in `pychrono.vehicle`, which the conda-forge PyChrono build OMITS —
that still needs a source build of Chrono with the vehicle module. So: rigid-body authority = real and
live here; the soil-sinkage oracle = still gated on a vehicle-enabled Chrono. FIX-1 (K_PHI) is meanwhile
resolved by the literature-sourced NASA LTV lunar Bekker values (bodies sysrev).

Run in the Chrono env (NOT the runtime venv — that has no pychrono):
    MAMBA_ROOT_PREFIX=/tmp/mamba LD_LIBRARY_PATH=/tmp/chrono-env/lib \
        /tmp/chrono-env/bin/python scripts/chrono_clast_producer.py

CC0-1.0 (see ../LICENSE).
"""

from __future__ import annotations

import math

import pychrono as chrono

GRAIN_DENSITY_KG_M3 = 3100.0   # lunar regolith grain density (clasts are dense rock, not loose fill)
G_MOON = 1.62
G_EARTH = 9.81


def _system(gravity_z: float) -> "chrono.ChSystemSMC":
    sysm = chrono.ChSystemSMC()
    sysm.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, gravity_z))
    sysm.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # else primitive contacts don't engage
    return sysm


def _material() -> "chrono.ChContactMaterialSMC":
    mat = chrono.ChContactMaterialSMC()
    mat.SetYoungModulus(2.0e7)     # stiff enough that penalty penetration stays << clast radius
    mat.SetFriction(0.6)
    mat.SetRestitution(0.0)        # regolith clasts don't bounce
    return mat


def settle_clasts(clasts, *, gravity_z=-G_MOON, dt=2.0e-4, max_time_s=3.0, rest_ke_frac=1e-3):
    """Settle rigid clasts on a fixed ground plane (top at z=0) under `gravity_z` with a real Chrono solve.

    `clasts` = list of (x, y, radius_m) placed just above the surface. Returns a dict with the rest poses
    and settle diagnostics. Bodies come to rest at z ~= radius (resting ON the surface), with kinetic
    energy decaying to a small fraction of the initial drop energy (dissipative contact)."""
    mat = _material()
    sysm = _system(gravity_z)
    ground = chrono.ChBodyEasyBox(50.0, 50.0, 0.4, 2000.0, True, True, mat)
    ground.SetPos(chrono.ChVector3d(0.0, 0.0, -0.2))      # top face at z = 0
    ground.SetFixed(True)
    sysm.Add(ground)

    bodies = []
    for (x, y, r) in clasts:
        b = chrono.ChBodyEasySphere(r, GRAIN_DENSITY_KG_M3, True, True, mat)
        b.SetPos(chrono.ChVector3d(x, y, r + 0.05))       # released 5 cm above its resting height
        sysm.Add(b)
        bodies.append((b, r))

    def total_ke():
        ke = 0.0
        for b, _ in bodies:
            v = b.GetPosDt()
            ke += 0.5 * b.GetMass() * (v.x * v.x + v.y * v.y + v.z * v.z)
        return ke

    drop_pe = sum(b.GetMass() * abs(gravity_z) * 0.05 for b, _ in bodies)  # PE of the 5 cm drop
    steps = int(max_time_s / dt)
    settled_t = None
    for i in range(steps):
        sysm.DoStepDynamics(dt)
        if i > 100 and total_ke() < rest_ke_frac * max(drop_pe, 1e-9):
            settled_t = (i + 1) * dt
            break

    rest = [{"x": b.GetPos().x, "y": b.GetPos().y, "z": b.GetPos().z, "radius_m": r,
             "mass_kg": b.GetMass()} for b, r in bodies]
    return {"gravity_z": gravity_z, "n_clasts": len(bodies), "settled_time_s": settled_t,
            "final_ke_J": total_ke(), "drop_pe_J": drop_pe, "rest": rest}


def free_fall_time(height_m: float, gravity_z: float, *, dt=1.0e-4) -> float:
    """Drop a single rigid clast `height_m` in vacuum (no ground) and return the measured fall time [s].
    A clean exact-physics check of the Chrono solve against analytic t = sqrt(2h/|g|)."""
    sysm = _system(gravity_z)
    b = chrono.ChBodyEasySphere(0.1, GRAIN_DENSITY_KG_M3, True, False, _material())
    z0 = b.GetPos().z
    sysm.Add(b)
    t = 0.0
    while z0 - b.GetPos().z < height_m:
        sysm.DoStepDynamics(dt)
        t += dt
        if t > 100.0:
            break
    return t


def _selfcheck():
    print("=== free-fall (Chrono solve vs analytic t = sqrt(2h/|g|)) ===")
    h = 1.0
    for label, g in (("moon", G_MOON), ("earth", G_EARTH)):
        t = free_fall_time(h, -g)
        analytic = math.sqrt(2 * h / g)
        print(f"  {label:5s} g={g}: chrono {t:.4f} s vs analytic {analytic:.4f} s  "
              f"(err {100*abs(t-analytic)/analytic:.2f}%)")
        assert abs(t - analytic) / analytic < 0.02, "free-fall must match analytic within 2%"
    tm, te = free_fall_time(h, -G_MOON), free_fall_time(h, -G_EARTH)
    ratio, expect = tm / te, math.sqrt(G_EARTH / G_MOON)
    print(f"  lunar/earth fall-time ratio {ratio:.3f} vs sqrt(g_e/g_m) {expect:.3f}")
    assert abs(ratio - expect) / expect < 0.02

    print("=== settle 5 rigid clasts on the surface under lunar gravity ===")
    clasts = [(0.0, 0.0, 0.12), (0.4, 0.0, 0.10), (-0.3, 0.2, 0.08), (0.2, 0.4, 0.11), (-0.4, -0.3, 0.09)]
    r = settle_clasts(clasts, gravity_z=-G_MOON)
    print(f"  settled at t={r['settled_time_s']} s; final KE {r['final_ke_J']:.3e} J "
          f"(drop PE {r['drop_pe_J']:.3e} J)")
    for c in r["rest"]:
        print(f"    clast r={c['radius_m']:.2f} m  rest z={c['z']:.4f} m  (expect ~{c['radius_m']:.2f})")
        assert c["radius_m"] * 0.85 <= c["z"] <= c["radius_m"] * 1.10, "clast must rest ON the surface"
    assert r["final_ke_J"] < 1e-2 * r["drop_pe_J"], "kinetic energy must dissipate to rest"
    print("\nlive Chrono rigid-body producer: all physics checks passed.")


if __name__ == "__main__":
    _selfcheck()
