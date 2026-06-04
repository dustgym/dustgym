"""Package smoke test for the world-model surface: the five layers are importable from the installed
dustgym package and compute on a real scene. No synthetic data (real crater_boulders bundle)."""
from __future__ import annotations

import os

import numpy as np

from . import world_model as wm
from .worksite import coarse_base_from_bundle

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, ".."))
_SCENE = os.path.join(_REPO, "samples", "crater_boulders")


def test_describe_has_five_layers():
    assert set(wm.describe()) == {"geometry", "material", "physics", "task", "uncertainty"}


def test_layers_compute_on_a_real_scene():
    cs, _ = coarse_base_from_bundle(_SCENE)
    h, slope = wm.geometry(cs)
    assert h.shape == cs.mass_areal.shape and float(slope.min()) >= 0.0
    mat = wm.material_layer(cs)
    assert "friction_deg" in mat and mat["friction_deg"].shape == cs.mass_areal.shape
    ew = wm.earthwork(cs, float(np.median(h)))                  # flatten to the median
    assert ew["cut_m3"] > 0.0 and ew["fill_m3"] > 0.0          # conserved cut + fill, both real


def test_package_modules_import():
    # the conserved physics-side layers all ship in the installed package
    from terrain_authority import column_state, drive, material, slip, terramechanics  # noqa: F401


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("world_model: all checks passed")
