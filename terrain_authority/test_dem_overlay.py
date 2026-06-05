"""Characterization tests for ``terrain_authority.dem_overlay`` — the conservation-critical
smooth-interp + zero-mean-per-base-cell procgen overlay (L0 §4 / INTERFACE.md §5.3).

The base block driving these tests is built from the REAL LOLA Haworth backbone
(``samples/lunar_dem/haworth_10km_5m/heightmap.rf32``, real relief -96.6..+2842.2 m) via
``dem_import.dem_to_base`` — the same real-data idiom ``test_dem_io.py`` uses. No synthetic
field is fabricated; every base value is the conserved authority's real output over real lunar
relief.

The headline invariant under test is the resolution-bridge contract: refining a real base block
to fine through ``overlay_residual`` and coarsening it back recovers the base — the carried
density/datum/state_label/disturbance BIT-EXACT and mass_areal to the float64 noise floor
(~1e-12 relative), exactly as the module's docstring and self-test promise. Tests also cover the
smooth-interp mean restoration, the global-anchor determinism, and the Wave-2 crater feature_fn.
"""

from __future__ import annotations

import inspect
import json
import os

import numpy as np
import pytest

from terrain_authority import refinement
from terrain_authority.dem_import import Affine, crop_square, dem_to_base
from terrain_authority.dem_io import BASE_FIELD_NAMES
from terrain_authority.dem_overlay import (
    DEFAULT_OVERLAY_PARAMS,
    _block_means,
    _restore_block_means,
    _smooth_interp_height,
    make_crater_feature_fn,
    overlay_residual,
)

_DEM_DIR = os.path.join(
    os.path.dirname(__file__), os.pardir, "samples", "lunar_dem", "haworth_10km_5m",
)
_HEIGHTMAP = os.path.join(_DEM_DIR, "heightmap.rf32")
_METADATA = os.path.join(_DEM_DIR, "metadata.json")
_CELL_M = 5.0
_X0_CENTER = -52900.0
_Y0_CENTER = 105400.0


def _real_base_fields(crop_px: int = 24):
    """A REAL base field dict (5 BASE_FIELD_NAMES) from the committed LOLA surface."""
    if not (os.path.exists(_HEIGHTMAP) and os.path.exists(_METADATA)):
        pytest.skip(f"real LOLA backbone absent: {_DEM_DIR}")
    with open(_METADATA) as fh:
        meta = json.load(fh)
    h = int(meta["grid"]["height"])
    w = int(meta["grid"]["width"])
    Z = np.fromfile(_HEIGHTMAP, dtype="<f4").reshape(h, w)
    affine = Affine(x0=_X0_CENTER, y0=_Y0_CENTER, px=_CELL_M)
    cx, cy = affine.xy(h // 2, w // 2)
    extent = crop_px * _CELL_M
    Z_crop, aff_crop = crop_square(Z, affine, (float(cx), float(cy)), extent)
    cs = dem_to_base(Z_crop, aff_crop, _CELL_M)
    fields = {n: getattr(cs, n) for n in BASE_FIELD_NAMES}
    return fields, cs.cell_m


@pytest.fixture(scope="module")
def real_base():
    fields, base_cell_m = _real_base_fields(24)
    return fields, base_cell_m


# --- block-mean / mean-restore machinery (the conservation primitives) --------------------

def test_block_means_matches_coarsen_reduction():
    """_block_means uses the same reshape-mean coarsen uses (so the cancellation is exact)."""
    rng = np.random.default_rng(0)
    k = 4
    fine = rng.uniform(0, 10, (3 * k, 5 * k))
    bm = _block_means(fine, k)
    assert bm.shape == (3, 5)
    # Reference reduction: reshape (H,k,W,k), mean over the k-axes.
    ref = fine.reshape(3, k, 5, k).mean(axis=(1, 3))
    assert np.array_equal(bm, ref)


def test_restore_block_means_forces_target(real_base):
    """After restore, each k x k block mean equals the target base value exactly."""
    fields, _ = real_base
    base_thick = fields["mass_areal"] / fields["density"]
    H, W = base_thick.shape
    k = 5
    # Smooth-interp the real thickness then restore: block means must hit the base exactly.
    smooth = _smooth_interp_height(base_thick, k, "bicubic")
    restored = _restore_block_means(smooth, base_thick, k)
    assert restored.shape == (H * k, W * k)
    bm = _block_means(restored, k)
    assert np.allclose(bm, base_thick, rtol=0, atol=1e-9)


def test_smooth_interp_shape_and_modes(real_base):
    """All three smooth modes produce the (H*k, W*k) fine field over the real base."""
    fields, _ = real_base
    base_thick = fields["mass_areal"] / fields["density"]
    H, W = base_thick.shape
    k = 4
    for mode in ("bicubic", "bilinear", "none"):
        out = _smooth_interp_height(base_thick, k, mode)
        assert out.shape == (H * k, W * k)
        assert np.isfinite(out).all()
    # "none" is the piecewise-constant plateau == np.repeat block copy.
    plateau = _smooth_interp_height(base_thick, k, "none")
    assert np.array_equal(plateau, np.repeat(np.repeat(base_thick, k, 0), k, 1))


# --- overlay_residual: the resolution-bridge invariant on REAL relief ---------------------

@pytest.mark.parametrize("k", [2, 5, 8])
def test_overlay_coarsen_recovers_real_base(real_base, k):
    """coarsen(overlay_residual(real_base, k)) == real_base: carried bit-exact, mass to floor."""
    fields, base_cell_m = real_base
    fine = overlay_residual(fields, k, _X0_CENTER, _Y0_CENTER,
                            fine_cell_m=base_cell_m / k, world_seed=7)
    # Fine bundle has the carried fields at (H*k, W*k).
    H, W = fields["mass_areal"].shape
    for n in ("mass_areal", "density", "datum", "state_label", "disturbance"):
        assert fine[n].shape == (H * k, W * k), n

    back = refinement.coarsen_field(fine, k)
    # Carried intensive fields coarsen BIT-EXACT.
    assert np.array_equal(back["density"], fields["density"])
    assert np.array_equal(back["datum"], fields["datum"])
    assert np.array_equal(back["state_label"], fields["state_label"])
    assert np.array_equal(back["disturbance"], fields["disturbance"])
    # mass_areal recovers to the float64 noise floor (mass conservation).
    rel = np.abs((back["mass_areal"] - fields["mass_areal"])
                 / fields["mass_areal"]).max()
    assert rel < 1e-12, f"mass not conserved: worst_rel={rel:.2e}"


def test_overlay_adds_real_detail(real_base):
    """The overlay adds genuine sub-base detail: per-base-cell block variance is > 0."""
    fields, base_cell_m = real_base
    k = 8
    fine = overlay_residual(fields, k, _X0_CENTER, _Y0_CENTER,
                            fine_cell_m=base_cell_m / k, world_seed=3)
    H, W = fields["mass_areal"].shape
    thick = (fine["mass_areal"] / fine["density"]).reshape(H, k, W, k)
    assert float(thick.var(axis=(1, 3)).max()) > 0.0


def test_overlay_global_anchor_determinism(real_base):
    """Same (world_x0, world_y0, world_seed) -> byte-identical; a different origin differs."""
    fields, base_cell_m = real_base
    k = 5
    fc = base_cell_m / k
    a = overlay_residual(fields, k, 1000.0, 2000.0, fine_cell_m=fc, world_seed=11)
    b = overlay_residual(fields, k, 1000.0, 2000.0, fine_cell_m=fc, world_seed=11)
    c = overlay_residual(fields, k, 1050.0, 2000.0, fine_cell_m=fc, world_seed=11)
    assert np.array_equal(a["mass_areal"], b["mass_areal"])
    # Shifting the global origin changes the fbm sample -> the detail differs.
    assert not np.array_equal(a["mass_areal"], c["mass_areal"])


def test_overlay_no_fbm_is_plateau_safe(real_base):
    """With fbm_nu0=0 the residual is zero; coarsen still recovers the base exactly."""
    fields, base_cell_m = real_base
    k = 4
    params = dict(DEFAULT_OVERLAY_PARAMS)
    params["fbm_nu0"] = 0.0
    fine = overlay_residual(fields, k, _X0_CENTER, _Y0_CENTER,
                            fine_cell_m=base_cell_m / k, world_seed=0, params=params)
    back = refinement.coarsen_field(fine, k)
    rel = np.abs((back["mass_areal"] - fields["mass_areal"])
                 / fields["mass_areal"]).max()
    assert rel < 1e-12


def test_overlay_rejects_bad_k(real_base):
    fields, base_cell_m = real_base
    with pytest.raises(ValueError):
        overlay_residual(fields, 0, 0.0, 0.0, fine_cell_m=1.0)


def test_overlay_dict_requires_fine_cell_m(real_base):
    """A dict base (not a ColumnState) requires fine_cell_m to be supplied."""
    fields, _ = real_base
    with pytest.raises(ValueError, match="fine_cell_m"):
        overlay_residual(fields, 4, 0.0, 0.0)


def test_overlay_accepts_columnstate_input(real_base):
    """A ColumnState base derives fine_cell_m from cs.cell_m/k and conserves mass on coarsen."""
    fields, base_cell_m = real_base
    cs = refinement._column_state_from_fields(fields, base_cell_m)
    k = 4
    fine = overlay_residual(cs, k, _X0_CENTER, _Y0_CENTER, world_seed=5)
    H, W = fields["mass_areal"].shape
    assert fine["mass_areal"].shape == (H * k, W * k)
    back = refinement.coarsen_field(fine, k)
    rel = np.abs((back["mass_areal"] - fields["mass_areal"])
                 / fields["mass_areal"]).max()
    assert rel < 1e-12
    # The carried density still coarsens bit-exact through the ColumnState path.
    assert np.array_equal(back["density"], fields["density"])


# --- make_crater_feature_fn: the Wave-2 seam -----------------------------------------------

def test_crater_feature_fn_signature_and_shape():
    """The crater feature_fn matches the frozen hook signature and returns residual_h's shape."""
    feat = make_crater_feature_fn(dem_effres_m=15.0, d_min_m=1.0)
    sig = inspect.signature(feat)
    pos = [p.name for p in sig.parameters.values()
           if p.kind in (inspect.Parameter.POSITIONAL_ONLY,
                          inspect.Parameter.POSITIONAL_OR_KEYWORD)]
    kwo = [p.name for p in sig.parameters.values()
           if p.kind == inspect.Parameter.KEYWORD_ONLY]
    assert pos == ["residual_h", "world_x0", "world_y0", "cell_m"]
    assert kwo == ["params", "world_seed"]

    resid = np.zeros((120, 120), dtype=np.float64)
    out = feat(resid, 2000.0, -3000.0, 0.5,
               params=DEFAULT_OVERLAY_PARAMS, world_seed=0)
    assert isinstance(out, np.ndarray)
    assert out.shape == resid.shape


def test_crater_feature_fn_determinism_and_carving():
    """Same world point/seed -> byte-identical carved craters; a different origin differs."""
    feat = make_crater_feature_fn(dem_effres_m=15.0, d_min_m=1.0)
    resid = np.zeros((200, 200), dtype=np.float64)
    a = feat(resid, 2000.0, -3000.0, 0.5, params=DEFAULT_OVERLAY_PARAMS, world_seed=0)
    b = feat(resid, 2000.0, -3000.0, 0.5, params=DEFAULT_OVERLAY_PARAMS, world_seed=0)
    c = feat(resid, 2055.0, -3000.0, 0.5, params=DEFAULT_OVERLAY_PARAMS, world_seed=0)
    assert a.tobytes() == b.tobytes()
    assert np.any(a != 0.0), "no craters carved on this real-scale window"
    assert not np.array_equal(a, c)
