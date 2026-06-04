"""Tests for the world model's Material layer (material.py).

Per-cell strength is a monotonic function of the real per-cell density across the sourced spec ranges:
endpoints are anchored (surface density reproduces the repo's nominal friction/cohesion; deep density
hits the spec's dense values), the field is monotonic in density, and on a REAL worked scene's
density.rf32 it varies spatially and stays in range. No fabricated data.
"""
from __future__ import annotations

import os

import numpy as np

from . import constants as K
from . import material as mat

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, ".."))
_WORKED = os.path.join(_REPO, "samples", "crater_boulders_worked")


def test_endpoints_anchor_to_sourced_values():
    # surface (loose) density -> the repo's nominal friction/cohesion; deep density -> the spec dense end.
    loose = mat.material_fields(np.array([[K.RHO_SURFACE]]))
    dense = mat.material_fields(np.array([[K.RHO_DEEP]]))
    assert abs(float(loose["friction_deg"][0, 0]) - np.rad2deg(K.PHI)) < 1e-9
    assert abs(float(loose["cohesion_pa"][0, 0]) - K.COHESION) < 1e-9
    assert abs(float(dense["friction_deg"][0, 0]) - mat.PHI_DENSE_DEG) < 1e-9
    assert abs(float(dense["cohesion_pa"][0, 0]) - mat.COHESION_DENSE_PA) < 1e-9


def test_relative_density_clamps():
    dr = mat.relative_density(np.array([800.0, K.RHO_SURFACE, K.RHO_DEEP, 3000.0]))
    assert dr[0] == 0.0 and dr[1] == 0.0 and dr[2] == 1.0 and dr[3] == 1.0


def test_strength_is_monotonic_in_density():
    rho = np.array([[1300.0, 1500.0, 1700.0, 1920.0]])
    f = mat.material_fields(rho)
    assert np.all(np.diff(f["friction_deg"][0]) > 0)          # denser -> higher friction
    assert np.all(np.diff(f["cohesion_pa"][0]) > 0)           # denser -> more cohesion
    assert np.all(np.diff(f["traction_capacity_n"][0]) > 0)   # denser -> more grip
    assert np.all(np.diff(f["slip_susceptibility"][0]) < 0)   # denser -> slips less
    assert np.all(np.diff(f["cut_difficulty"][0]) > 0)        # denser -> harder to cut


def test_real_worked_scene_in_range_and_varies():
    if not os.path.isfile(os.path.join(_WORKED, "density.rf32")):
        import pytest
        pytest.skip("no crater_boulders_worked density bundle")
    rho = mat.load_density(_WORKED)
    f = mat.material_fields(rho)
    assert f["friction_deg"].shape == rho.shape
    assert float(f["friction_deg"].std()) > 0.0               # spatially varying
    assert mat.PHI_LOOSE_DEG - 1e-6 <= float(f["friction_deg"].min())
    assert float(f["friction_deg"].max()) <= mat.PHI_DENSE_DEG + 1e-6
    assert K.COHESION - 1e-6 <= float(f["cohesion_pa"].min())
    assert float(f["cohesion_pa"].max()) <= mat.COHESION_DENSE_PA + 1e-6


def test_material_changes_drive_slip():
    # threaded into the solver: on a forward grade, loose soil slips MORE than compacted soil, and
    # material=False is byte-identical to the global-params path. Material is load-bearing in the dynamics.
    from . import drive
    from .column_state import ColumnState
    H = W = 16
    cols = np.arange(W)[None, :] * np.ones((H, 1))
    datum = (cols * 0.20).astype(float)                       # forward grade along +col (~21.8 deg)
    mass = np.full((H, W), 50.0)

    def cs_with(rho):
        return ColumnState(W, H, 0.5, mass_areal=mass.copy(), density=np.full((H, W), float(rho)),
                           state_label=np.zeros((H, W), np.uint8), disturbance=np.zeros((H, W)),
                           datum=datum.copy())

    rc, yaw = (8.0, 8.0), 0.0                                 # facing +col (up the grade)
    _, _, loose = drive.drive_step(cs_with(K.RHO_SURFACE), rc, yaw, 0.2, 0.0, material=True)
    _, _, dense = drive.drive_step(cs_with(K.RHO_DEEP), rc, yaw, 0.2, 0.0, material=True)
    assert loose["slip"] > dense["slip"]                      # loose regolith slips more (Material -> dynamics)
    _, _, off = drive.drive_step(cs_with(1500.0), rc, yaw, 0.2, 0.0, material=False)
    _, _, glob = drive.drive_step(cs_with(1500.0), rc, yaw, 0.2, 0.0)
    assert off["slip"] == glob["slip"]                        # default path byte-identical (opt-in only)


if __name__ == "__main__":                                    # pure-python runner, no pytest needed
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"ok  {name}")
            except BaseException as e:                         # noqa: BLE001 -- report skips too
                if type(e).__name__ == "Skipped":
                    print(f"skip {name}: {e}")
                else:
                    raise
    print("material: all checks passed")
