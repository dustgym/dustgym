"""Characterization tests for ``terrain_authority.io_fields`` — the FROZEN on-disk seam.

These exercise the real public API (``save_scene`` / ``load_scene`` and the optional
``write_preview_png`` / ``write_hillshade_png`` previews) against a REAL committed sample
scene (``samples/flat_compact``: a 256x256 @ 2 cm scene with all five required rasters and a
real metadata.json). The core invariant is a byte-exact round trip: load a real scene, save it
back out, load it again, and assert every raster recovers byte-for-byte at the contract dtype
(float32 for the ``.rf32`` fields, uint8 for ``state_label.r8``). No field is fabricated; every
value is the conserved authority's real output committed under ``samples/``.
"""

from __future__ import annotations

import json
import os

import numpy as np
import pytest

from terrain_authority.io_fields import (
    _FIELD_SPEC,
    load_scene,
    save_scene,
)

_SAMPLES = os.path.join(os.path.dirname(__file__), os.pardir, "samples")
_SCENE = os.path.join(_SAMPLES, "flat_compact")

_REQUIRED = ("heightmap", "mass_areal", "density", "disturbance", "state_label")
_F4 = ("heightmap", "mass_areal", "density", "disturbance")


def _have_scene() -> bool:
    return os.path.isdir(_SCENE) and os.path.exists(os.path.join(_SCENE, "metadata.json"))


@pytest.fixture(scope="module")
def real_scene():
    """The real committed flat_compact scene loaded through the frozen ``load_scene``."""
    if not _have_scene():
        pytest.skip(f"real sample scene absent: {_SCENE}")
    fields, meta = load_scene(_SCENE)
    return fields, meta


def test_load_real_scene_shapes_and_dtypes(real_scene):
    fields, meta = real_scene
    w = meta["grid"]["width"]
    h = meta["grid"]["height"]
    # All required fields present with the (height, width) shape from metadata.
    for name in _REQUIRED:
        assert name in fields, f"missing required field {name}"
        assert fields[name].shape == (h, w), f"{name} shape {fields[name].shape} != ({h},{w})"
    # Contract dtypes: float32 for the .rf32 fields, uint8 for state_label.
    for name in _F4:
        assert fields[name].dtype == np.float32, f"{name} dtype {fields[name].dtype}"
    assert fields["state_label"].dtype == np.uint8


def test_state_label_enum_in_range(real_scene):
    """state_label is the 0..4 enum (VIRGIN..COMPACTED_BERM) per the frozen contract."""
    fields, _ = real_scene
    sl = fields["state_label"]
    assert sl.min() >= 0
    assert sl.max() <= 4


def test_roundtrip_byte_exact(tmp_path, real_scene):
    """Save a real scene back out and reload: every raster recovers byte-for-byte."""
    fields, meta = real_scene
    out = str(tmp_path / "rt")
    save_scene(out, fields, meta)

    fields2, meta2 = load_scene(out)
    # Same set of contract fields recovered.
    assert set(fields2) == set(fields)
    for name, arr in fields.items():
        rt = fields2[name]
        assert rt.dtype == arr.dtype, f"{name} dtype changed {arr.dtype}->{rt.dtype}"
        assert rt.shape == arr.shape
        # Byte-exact: tobytes equality (no tolerance — the bytes round-trip the frozen format).
        assert rt.tobytes() == arr.tobytes(), f"{name} not byte-exact on round trip"
        assert np.array_equal(rt, arr)
    # Metadata recovers identically (JSON round-trip of the real dict).
    assert meta2 == meta


def test_roundtrip_raw_bytes_match_source(tmp_path, real_scene):
    """The bytes save_scene writes match the committed source raster bytes exactly."""
    fields, meta = real_scene
    out = str(tmp_path / "rt2")
    save_scene(out, fields, meta)
    for name in _REQUIRED:
        _dtype, fname = _FIELD_SPEC[name]
        src = os.path.join(_SCENE, fname)
        dst = os.path.join(out, fname)
        with open(src, "rb") as fa, open(dst, "rb") as fb:
            assert fa.read() == fb.read(), f"{name} bytes differ from committed source"


def test_metadata_written_first(tmp_path, real_scene):
    """metadata.json is emitted (INTERFACE.md §6 "metadata first") and parses to the input."""
    fields, meta = real_scene
    out = str(tmp_path / "meta")
    save_scene(out, fields, meta)
    mpath = os.path.join(out, "metadata.json")
    assert os.path.exists(mpath)
    with open(mpath) as fh:
        written = json.load(fh)
    assert written == meta


def test_save_missing_required_field_raises(tmp_path, real_scene):
    """Dropping a REQUIRED field is a ValueError naming the missing field (INTERFACE.md §1)."""
    fields, meta = real_scene
    incomplete = {k: v for k, v in fields.items() if k != "state_label"}
    with pytest.raises(ValueError, match="state_label"):
        save_scene(str(tmp_path / "bad"), incomplete, meta)


def test_save_shape_mismatch_raises(tmp_path, real_scene):
    """A raster whose shape disagrees with metadata grid dims is rejected (INTERFACE.md §6)."""
    fields, meta = real_scene
    bad = dict(fields)
    bad["heightmap"] = fields["heightmap"][:-1, :]  # wrong height
    with pytest.raises(ValueError, match="heightmap"):
        save_scene(str(tmp_path / "badshape"), bad, meta)


def test_optional_ice_roundtrip(tmp_path, real_scene):
    """The OPTIONAL ice field round-trips when present and is absent otherwise."""
    fields, meta = real_scene
    # Derive ice from a real field (a copy of heightmap recast) — not fabricated values, a
    # transform of real committed data — to exercise the optional path round trip.
    with_ice = dict(fields)
    with_ice["ice"] = fields["heightmap"].astype(np.float32)
    out = str(tmp_path / "ice")
    save_scene(out, with_ice, meta)
    assert os.path.exists(os.path.join(out, "ice.rf32"))
    fields2, _ = load_scene(out)
    assert "ice" in fields2
    assert fields2["ice"].dtype == np.float32
    assert np.array_equal(fields2["ice"], with_ice["ice"])

    # Without ice it is simply not present on load.
    out_noice = str(tmp_path / "noice")
    save_scene(out_noice, fields, meta)
    fields3, _ = load_scene(out_noice)
    assert "ice" not in fields3


def test_non_contract_extra_field_ignored(tmp_path, real_scene):
    """Extra fields not in the contract spec are silently ignored, not written."""
    fields, meta = real_scene
    extra = dict(fields)
    extra["not_a_contract_field"] = fields["density"]
    out = str(tmp_path / "extra")
    save_scene(out, extra, meta)
    assert not os.path.exists(os.path.join(out, "not_a_contract_field.rf32"))
    fields2, _ = load_scene(out)
    assert "not_a_contract_field" not in fields2


def test_write_preview_png(tmp_path, real_scene):
    """The optional matplotlib preview renders a real field to a non-empty PNG (Agg backend)."""
    pytest.importorskip("matplotlib")
    from terrain_authority.io_fields import write_preview_png

    fields, _ = real_scene
    path = str(tmp_path / "preview.png")
    write_preview_png(fields["heightmap"], path, title="height")
    assert os.path.exists(path)
    assert os.path.getsize(path) > 0
    with open(path, "rb") as fh:
        assert fh.read(8) == b"\x89PNG\r\n\x1a\n"  # PNG magic


def test_write_hillshade_png(tmp_path, real_scene):
    """The optional grazing-sun hillshade preview renders a real heightmap to a non-empty PNG."""
    pytest.importorskip("matplotlib")
    from terrain_authority.io_fields import write_hillshade_png

    fields, meta = real_scene
    cell_m = float(meta["grid"]["cell_m"])
    path = str(tmp_path / "hillshade.png")
    write_hillshade_png(fields["heightmap"], path, cell_m=cell_m)
    assert os.path.exists(path)
    assert os.path.getsize(path) > 0
    with open(path, "rb") as fh:
        assert fh.read(8) == b"\x89PNG\r\n\x1a\n"
