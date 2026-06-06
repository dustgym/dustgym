"""Tests for gen_ipex_mesh.py — verify the self-authored IPEx primitive body is built to the
sourced IPEx dimensions (wheel 30.5 cm, medium bucket drum 295 mm) and exports valid glTF parts
the Godot sidecar can assemble. CC0: we author these primitives (no EZ-RASSOR / third-party mesh).

Real-data check: the dimensions come from terrain_authority.ipex_specs (Schuler/Zhang sourced) +
the generator's disclosed [CALIB] chassis numbers; no fabricated geometry passed off as flight CAD.
"""
from __future__ import annotations

import os

import trimesh

from scripts import gen_ipex_mesh as G
from terrain_authority import ipex_specs as ix


def test_part_dimensions_match_sourced_specs(tmp_path):
    parts = G.build_parts()
    # wheel: a cylinder of the sourced flight radius (0.1524 m) -> diameter 0.305 m across its spin plane
    wheel = parts["wheel"]
    ext = wheel.extents
    spin_dia = sorted(ext)[1] * 2 if False else max(ext[0], ext[1])   # spin-plane diameter
    assert abs(spin_dia - ix.WHEEL_DIAMETER_M) < 1e-3, (spin_dia, ix.WHEEL_DIAMETER_M)
    # drum: the IPEx medium bucket drum, 295 mm dia (BDSCALE)
    drum = parts["drum"]
    drum_dia = max(drum.extents[0], drum.extents[1])
    assert abs(drum_dia - G.DRUM_DIAMETER_M) < 1e-3
    assert abs(G.DRUM_DIAMETER_M - 0.295) < 1e-3            # sourced medium-drum diameter
    # all four assemble-parts present
    assert set(parts) == {"rover_body", "wheel", "drum", "drum_arm"}


def test_parts_centered_at_joint_origin():
    # convert_rover_mesh.py centers sub-parts at their joint/rotation center; ours must too so the
    # sidecar can place them at WHEEL_ORIGINS. Wheel + drum symmetric about (0,0) in the spin plane.
    for name in ("wheel", "drum"):
        c = G.build_parts()[name].centroid
        assert abs(c[0]) < 1e-6 and abs(c[1]) < 1e-6, (name, c)


def test_export_writes_loadable_glbs(tmp_path):
    out = G.export_all(str(tmp_path))
    for name in ("rover_body", "wheel", "drum", "drum_arm"):
        p = os.path.join(str(tmp_path), f"{name}.glb")
        assert os.path.exists(p) and os.path.getsize(p) > 0
        loaded = trimesh.load(p)                            # round-trips as valid glTF
        assert loaded is not None
    assert out == str(tmp_path)


def _run_all():
    import tempfile
    test_part_dimensions_match_sourced_specs(None)
    test_parts_centered_at_joint_origin()
    with tempfile.TemporaryDirectory() as d:
        test_export_writes_loadable_glbs(type("P", (), {"__fspath__": lambda s: d, "__str__": lambda s: d})())
    print("all gen_ipex_mesh checks passed")


if __name__ == "__main__":
    _run_all()
