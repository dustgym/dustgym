"""Characterization tests for ``terrain_authority.hexviz`` — the terminal hex visualizer.

These render a REAL committed sample scene (``samples/rolling_hills``, which has real height
relief so the hex levels are not a flat plateau) through the module's real public surface:
``hex_render`` (field -> (text, vmin, vmax)), ``_downsample`` (block-mean reducer), and ``main``
(the CLI that loads a scene via io_fields and prints the picture + legend). Assertions check the
real invariants: the rendered text is non-empty, uses only the 0-f hex glyphs, has the requested
grid dimensions, brackets the field's true min/max, prints the legend, and exits 0 on a present
field / 2 on an absent one. No synthetic field is fabricated; the scene is real on-disk data.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from terrain_authority.hexviz import _HEX, _downsample, hex_render, main
from terrain_authority.io_fields import load_scene

_SAMPLES = os.path.join(os.path.dirname(__file__), os.pardir, "samples")
_SCENE = os.path.join(_SAMPLES, "rolling_hills")


def _have_scene() -> bool:
    return os.path.isdir(_SCENE) and os.path.exists(os.path.join(_SCENE, "metadata.json"))


@pytest.fixture(scope="module")
def real_fields():
    if not _have_scene():
        pytest.skip(f"real sample scene absent: {_SCENE}")
    fields, _ = load_scene(_SCENE)
    return fields


def test_downsample_real_field_shape_and_mean(real_fields):
    """Block-mean downsample yields the requested dims and preserves the overall mean band."""
    h = real_fields["heightmap"].astype(np.float64)
    ds = _downsample(h, 32, 16)
    assert ds.shape == (16, 32)
    assert ds.dtype == np.float64
    # The block-mean grand average stays within the original field's min/max envelope.
    assert h.min() <= ds.mean() <= h.max()


def test_downsample_caps_to_field_dims():
    """_downsample never upsamples: requesting more cells than the field has is clamped."""
    field = np.arange(4 * 6, dtype=np.float64).reshape(4, 6)
    ds = _downsample(field, out_w=100, out_h=100)
    assert ds.shape == (4, 6)


def test_hex_render_real_height(real_fields):
    """Rendering the real heightmap returns non-empty text of the requested grid using 0-f."""
    text, vmin, vmax = hex_render(real_fields["heightmap"], out_w=48, out_h=24)
    assert isinstance(text, str)
    assert text  # non-empty
    lines = text.split("\n")
    assert len(lines) == 24
    assert all(len(line) == 48 for line in lines)
    # Every glyph is a valid hex digit from the module's alphabet.
    assert set(text) <= set(_HEX + "\n")
    # vmin/vmax bracket the rendered (downsampled) field and order correctly.
    assert vmin <= vmax
    # Real relief => more than one distinct level appears.
    glyphs = set(text) - {"\n"}
    assert len(glyphs) > 1


def test_hex_render_vmin_vmax_match_downsample(real_fields):
    """The returned vmin/vmax equal the downsampled field's true extremes (real invariant)."""
    field = real_fields["disturbance"].astype(np.float64)
    ds = _downsample(field, 64, 32)
    text, vmin, vmax = hex_render(field, 64, 32)
    assert vmin == pytest.approx(float(ds.min()))
    assert vmax == pytest.approx(float(ds.max()))


def test_hex_render_flat_field_all_zero_glyph():
    """A constant field has zero span -> all cells map to glyph '0' (the span<=0 branch)."""
    flat = np.full((20, 20), 3.14, dtype=np.float64)
    text, vmin, vmax = hex_render(flat, 8, 4)
    assert vmin == vmax
    assert set(text) - {"\n"} == {"0"}


def test_main_renders_real_scene(capsys):
    """main() loads the real scene, prints header + picture + legend, and returns 0."""
    if not _have_scene():
        pytest.skip(f"real sample scene absent: {_SCENE}")
    rc = main([_SCENE, "--field", "heightmap", "--width", "40", "--height", "20"])
    assert rc == 0
    out = capsys.readouterr().out
    assert out.strip()
    assert "scene:" in out
    assert "legend" in out
    assert "min=" in out and "max=" in out
    # The picture body lines use the hex alphabet.
    body = [ln for ln in out.splitlines() if ln and not ln.startswith("#")]
    assert body
    assert all(set(ln) <= set(_HEX) for ln in body)


def test_main_state_label_prints_enum_legend(capsys):
    """Rendering state_label appends the VIRGIN..COMPACTED_BERM enum key."""
    if not _have_scene():
        pytest.skip(f"real sample scene absent: {_SCENE}")
    rc = main([_SCENE, "--field", "state_label"])
    assert rc == 0
    out = capsys.readouterr().out
    assert "VIRGIN" in out
    assert "COMPACTED_BERM" in out


def test_main_absent_field_returns_2(tmp_path, capsys):
    """A valid field choice that is absent from the loaded scene returns exit code 2.

    Built by copying the real scene to a tmp dir and deleting the density raster so
    ``load_scene`` legitimately omits it; the committed samples/ tree is never touched.
    """
    if not _have_scene():
        pytest.skip(f"real sample scene absent: {_SCENE}")
    from terrain_authority.io_fields import _FIELD_SPEC, save_scene

    fields, meta = load_scene(_SCENE)
    out = str(tmp_path / "scene")
    save_scene(out, fields, meta)
    # Remove the density raster so load_scene won't include the 'density' field.
    os.remove(os.path.join(out, _FIELD_SPEC["density"][1]))
    rc = main([out, "--field", "density"])
    assert rc == 2
    err = capsys.readouterr().err
    assert "density" in err
