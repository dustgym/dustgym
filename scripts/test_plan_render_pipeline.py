"""Conservation tests for the plan -> render pipeline's flatten earthwork (no Godot needed).

The flatten moves real regolith on the conserved ColumnState (cut above target -> drum -> fill below),
so total mass (grid + drum) is invariant, the drum bookkeeping balances, and the worked AFTER bundle
round-trips through save/load. Uses the real crater_boulders bundle; no synthetic data.
"""
from __future__ import annotations

import os
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, ".."))
sys.path.insert(0, _REPO)
sys.path.insert(0, _HERE)
import plan_render_pipeline as prp  # noqa: E402
from terrain_authority.worksite import coarse_base_from_bundle  # noqa: E402

_SCENE = os.path.join(_REPO, "samples", "crater_boulders")


def test_flatten_conserves_mass_and_drum_balances():
    cs, _ = coarse_base_from_bundle(_SCENE)
    before = cs.total_mass()                                   # grid + drum
    plan = prp.plan_flatten(cs, pad_frac=0.5)
    after = cs.total_mass()
    assert abs(after - before) < 1e-6 * before                 # cut -> drum -> fill conserves total mass
    assert plan["cut_kg"] > 0 and plan["fill_kg"] > 0
    assert plan["fill_kg"] <= plan["cut_kg"] + 1e-6            # cannot deposit more than was cut
    assert abs(plan["drum_kg"] - (plan["cut_kg"] - plan["fill_kg"])) < 1e-3   # drum balances
    assert plan["cut_vol_m3"] > 0 and plan["fill_vol_m3"] > 0


def test_after_bundle_roundtrips():
    cs, meta = coarse_base_from_bundle(_SCENE)
    prp.plan_flatten(cs, pad_frac=0.5)
    h_after = cs.derive_height().copy()
    d = tempfile.mkdtemp(prefix="plan_render_")
    prp.write_bundle(cs, meta, os.path.join(d, "after"))
    cs2, _ = coarse_base_from_bundle(os.path.join(d, "after"))
    assert np.allclose(cs2.derive_height(), h_after, atol=1e-2)   # worked bundle is renderable + faithful


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("plan_render_pipeline: all checks passed")
