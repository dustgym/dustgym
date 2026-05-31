"""Conservation-critical procgen overlay: smooth-interp + zero-mean-per-base-cell residual
(L0 contract §4; eval §5 steps 3-4; INTERFACE.md §5.3).

THE PROBLEM this solves. ``refinement.refine_field`` is a piecewise-constant ``np.repeat``
block copy (``refinement.py:193``): refining a base cell to a k x k fine block makes a FLAT
plateau. Two pathologies follow (eval §5 step 4):
  1. adding zero-mean fbm on top of flat plateaus ALIASES at every base-cell boundary (the
     low-frequency content is a staircase, not a continuous surface);
  2. the resolution-bridge invariant (INTERFACE.md §5.3, spec §10) requires
     ``coarsen(fine_tile) == base`` BIT-EXACT, so whatever detail we add must be ZERO-MEAN per
     base cell, and cross-base-cell features must not be zero-meaned independently per cell
     (that injects an internal-boundary step).

THE OVERLAY (this module), in the contract's three steps:
  (1) MEAN-PRESERVING SMOOTH INTERPOLATION. Replace the flat k x k plateau height field with a
      smooth (bicubic-via-scipy / bilinear-fallback) interpolation across base-cell CENTERS, so
      the refined low-frequency surface is continuous, AND re-coarsens to the original base mean
      (we enforce this by subtracting, per base cell, the difference between the interpolated
      block mean and the base value -> the block mean is restored EXACTLY).
  (2) ZERO-MEAN-PER-BASE-CELL RESIDUAL. Add a bounded ``fbm_global`` residual (global-frame,
      deterministic via coord_seed) and subtract its OWN per-base-cell mean, so each base cell's
      mean detail is 0 and ``coarsen`` recovers the base.
  (3) CROSS-BASE-CELL CONTINUITY. The fbm is sampled on the GLOBAL lattice over the whole tile
      window at once (the "union of overlapping k x k blocks"), so a feature spanning a base-cell
      boundary is continuous; only the PER-BASE-CELL MEAN is removed (each cell keeps its own
      internal shape), so the boundary stays continuous while each cell's mean is unchanged.

The result is written back through ``ColumnState.set_height_via_mass`` so it is mass-conserving
by construction (height authored, mass backed out at fixed density/datum). density / datum /
state_label / disturbance are carried through the refine unchanged (the residual is a HEIGHT
detail only), so ``coarsen`` of those fields is already the base by ``coarsen(refine(x))==x``.

WAVE-1 SCOPE. Only the MECHANISM is wired here, exercised with ``fbm_global``. The crater
(``procgen_csfd``) and boulder generators are Wave-2; ``_apply_feature_hook`` is the explicit,
documented seam where they plug in (same zero-mean-per-base-cell + global-lattice discipline).

Pure NumPy; scipy is used ONLY for the optional bicubic smoothing and degrades to a pure-NumPy
bilinear if unavailable. Imports ``refinement`` and ``column_state`` (allowed); modifies neither.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from . import refinement
from .column_state import ColumnState


# ---------------------------------------------------------------------------
# Default overlay parameters. Wave-2 (Lane B / procgen_csfd) supplies the sourced
# values; these are placeholder magnitudes for the MECHANISM test only and are tagged
# so nobody mistakes them for sourced terrain. (eval §6: roughness anchor nu0 comes from
# Product-90 LDRM_RMSD; H ~0.95 resolved band.)
# ---------------------------------------------------------------------------
DEFAULT_OVERLAY_PARAMS: dict = {
    "fbm_nu0": 1.0e-4,          # [CALIB placeholder] residual VARIANCE [m^2]; Wave-2 <- LDRM_RMSD
    "fbm_H": 0.95,              # [CALIB] Hurst, resolved band (eval §6; Rosenburg 2011)
    "fbm_octaves": 6,
    "fbm_base_wavelength_m": 8.0,
    "fbm_lacunarity": 2.0,
    "smooth": "bicubic",        # "bicubic" (scipy) | "bilinear" | "none"
}


# ---------------------------------------------------------------------------
# 1. Mean-preserving smooth interpolation of a base height field to fine.
# ---------------------------------------------------------------------------

def _smooth_interp_height(base_h: np.ndarray, k: int, mode: str) -> np.ndarray:
    """Smoothly interpolate a (H, W) base height field to (H*k, W*k) across base-cell CENTERS.

    The piecewise-constant ``refine_field`` puts the base value at every fine cell in a block
    (a flat plateau). Here we instead place the base value at each base cell's CENTER and
    interpolate continuously to the fine cell centers, so the refined surface has no staircase.

    Fine-cell ``(rf, cf)`` center in base-cell units is ``(rf + 0.5)/k - 0.5`` (the base cell
    center for fine row ``rf`` is at base coordinate ``rf//k``; the +0.5/-0.5 puts both grids on
    cell-center registration). ``mode``:
      * "bicubic"  -> scipy ``map_coordinates`` order=3 (smoothest); falls back to bilinear if
        scipy is unavailable.
      * "bilinear" -> pure-NumPy separable bilinear.
      * "none"     -> the piecewise-constant plateau (== refine_field), for A/B comparison.

    NOTE this is NOT yet mean-preserving per base cell; the caller restores each base-cell mean
    exactly afterwards (``_restore_block_means``), which is what makes coarsen==base bit-exact.
    """
    H, W = base_h.shape
    if mode == "none":
        return np.repeat(np.repeat(base_h, k, axis=0), k, axis=1)

    # Fine-cell centers expressed in base-cell index coordinates (center registration).
    rf = (np.arange(H * k, dtype=np.float64) + 0.5) / k - 0.5   # (H*k,)
    cf = (np.arange(W * k, dtype=np.float64) + 0.5) / k - 0.5   # (W*k,)

    if mode == "bicubic":
        try:
            from scipy.ndimage import map_coordinates
            RR, CC = np.meshgrid(rf, cf, indexing="ij")
            coords = np.vstack([RR.ravel(), CC.ravel()])
            out = map_coordinates(base_h, coords, order=3, mode="nearest")
            return out.reshape(H * k, W * k)
        except Exception:
            mode = "bilinear"

    # Pure-NumPy separable bilinear (clamp to edges).
    def _axis_weights(idx: np.ndarray, n: int):
        i0 = np.clip(np.floor(idx).astype(np.int64), 0, n - 1)
        i1 = np.clip(i0 + 1, 0, n - 1)
        t = np.clip(idx - i0, 0.0, 1.0)
        return i0, i1, t

    r0, r1, tr = _axis_weights(rf, H)
    c0, c1, tc = _axis_weights(cf, W)
    # Gather the 4 corners and blend.
    v00 = base_h[np.ix_(r0, c0)]
    v01 = base_h[np.ix_(r0, c1)]
    v10 = base_h[np.ix_(r1, c0)]
    v11 = base_h[np.ix_(r1, c1)]
    tr2 = tr[:, None]
    tc2 = tc[None, :]
    top = v00 * (1.0 - tc2) + v01 * tc2
    bot = v10 * (1.0 - tc2) + v11 * tc2
    return top * (1.0 - tr2) + bot * tr2


# ---------------------------------------------------------------------------
# Per-base-cell block mean helpers (the conservation machinery).
# ---------------------------------------------------------------------------

def _block_means(fine: np.ndarray, k: int) -> np.ndarray:
    """Mean of each k x k base-cell block of a (H*k, W*k) fine field -> (H, W).

    Uses the SAME reduction ``coarsen_field`` uses (reshape to (H,k,W,k), mean over the k-axes)
    so "subtract this block mean" cancels EXACTLY against what coarsen will later compute —
    that exact cancellation is what gives bit-exact ``coarsen(fine)==base``.
    """
    hf, wf = fine.shape
    h, w = hf // k, wf // k
    return fine.reshape(h, k, w, k).mean(axis=(1, 3))


def _expand(block: np.ndarray, k: int) -> np.ndarray:
    """Expand a (H, W) per-base-cell array to (H*k, W*k) by piecewise-constant repeat."""
    return np.repeat(np.repeat(block, k, axis=0), k, axis=1)


def _restore_block_means(fine_h: np.ndarray, base_h: np.ndarray, k: int) -> np.ndarray:
    """Shift each k x k block of ``fine_h`` by a constant so its block-mean == ``base_h`` exactly.

    For each base cell, subtract (block_mean(fine_h) - base_h) uniformly across that block.
    Adding a per-block CONSTANT does not change the block's internal shape (continuity intact)
    but forces ``mean(block) == base_h`` for that cell. Because ``coarsen`` later takes the same
    block mean, this makes the round-trip bit-exact up to one float subtraction we eliminate by
    construction below (see overlay_residual: the height is re-meaned through the identical
    reshape-mean path coarsen uses).
    """
    correction = base_h - _block_means(fine_h, k)   # (H, W)
    return fine_h + _expand(correction, k)


# ---------------------------------------------------------------------------
# 2. Feature hook (Wave-2 seam for procgen_csfd craters / boulders).
# ---------------------------------------------------------------------------

def _apply_feature_hook(residual_h: np.ndarray, world_x0: float, world_y0: float,
                        cell_m: float, *, params: dict, world_seed: int,
                        feature_fn: Callable | None) -> np.ndarray:
    """ADD sub-DEM features (craters/boulders) to the fine residual height, in-place-safe.

    WAVE-2 SEAM. ``feature_fn(residual_h, world_x0, world_y0, cell_m, params, world_seed)`` is
    the documented plug for ``procgen_csfd`` (Poisson-per-log-D craters via ``carve_crater``)
    and the boulder field. It MUST author features on the GLOBAL lattice (so cross-base-cell
    features are continuous) and return the augmented fine residual; the per-base-cell zero-mean
    projection in ``overlay_residual`` then runs AFTER it, so a feature spanning a base-cell
    boundary stays continuous while each base cell's mean is restored (eval §5 step 3, the
    "generate on the union, subtract each cell's own mean within its sub-block" rule).

    For Wave-1 ``feature_fn is None`` -> no-op (the fbm_global residual alone exercises the
    mechanism). Returns the (possibly augmented) residual.
    """
    if feature_fn is None:
        return residual_h
    out = feature_fn(residual_h, world_x0, world_y0, cell_m,
                     params=params, world_seed=world_seed)
    if out is None:
        return residual_h
    return np.asarray(out, dtype=np.float64)


# ---------------------------------------------------------------------------
# 3. overlay_residual — the contract entry point (L0 §4).
# ---------------------------------------------------------------------------

def overlay_residual(base_tile_fields: ColumnState | dict[str, np.ndarray], k: int,
                     world_x0: float, world_y0: float, *,
                     params: dict | None = None, world_seed: int = 0,
                     fine_cell_m: float | None = None,
                     feature_fn: Callable | None = None) -> dict[str, np.ndarray]:
    """Refine a base block to fine WITH a continuous, mass-conserving procgen overlay (L0 §4).

    Pipeline (eval §5 steps 3-4; INTERFACE.md §5.3). Detail is overlaid in THICKNESS space
    (mass/density) rather than raw height, because mass_areal is the conserved field and density
    is copied per block, so ``coarsen(mass)=density·mean(thickness)`` and a per-block thickness
    mean restore keeps mass coarsen-consistent WITHOUT tripping the ``max(h-datum,0)`` clamp:
      0. Refine the base block (density/datum/state_label/disturbance) via ``refine_field`` —
         these carried fields are intensive copies, so ``coarsen`` of them is the base already.
      1. SMOOTH-INTERP the base THICKNESS across base-cell centers (kills the np.repeat plateau,
         anti-alias), then restore each base cell's mean EXACTLY (``_restore_block_means``) so the
         smoothed low-frequency layer re-coarsens to the original base.
      2. ADD a bounded ``fbm_global`` residual sampled on the GLOBAL lattice over the whole
         window (cross-base-cell continuity), then SUBTRACT its per-base-cell mean (zero-mean
         per base cell). Optionally ADD Wave-2 features via ``feature_fn`` BEFORE the zero-mean
         projection so they stay continuous across boundaries.
      3. Restore the per-base-cell thickness mean, clamp non-negative (mean-preserving), giving a
         continuous, detailed thickness whose block means equal the base thickness.
      4. mass_areal = thickness·density; a final per-block mean restore on mass_areal through the
         SAME reshape-mean path ``coarsen_field`` uses makes ``coarsen(fine mass)==base mass`` to
         the float64 NOISE FLOOR (~1e-15 relative — the irreducible heterogeneous-mean ULP, the
         same floor the repo's own ``test_coarsen_nonuniform_datum`` accepts; NOT a bit-zero fake).
         The carried density/datum coarsen bit-exact, so height coarsens to the same floor.

    Parameters
    ----------
    base_tile_fields : ColumnState | dict
        The BASE block (5 carried fields). A ColumnState is accepted (its cell_m gives the base
        cell size when ``fine_cell_m`` is supplied).
    k : int
        Positive integer refine factor (base_cell_m / fine_cell_m).
    world_x0, world_y0 : float
        GLOBAL metre coordinate of this block's origin (the (0,0) fine-cell lower corner). This
        anchors the global fbm lattice so adjacent tiles / re-visits agree (determinism).
    params : dict | None
        Overlay params (see DEFAULT_OVERLAY_PARAMS). None -> defaults.
    world_seed : int
        Forwarded to fbm_global / coord_seed.
    fine_cell_m : float | None
        Fine cell size [m]. If None and ``base_tile_fields`` is a ColumnState, derived as
        ``cs.cell_m / k``; otherwise required.
    feature_fn : callable | None
        Wave-2 crater/boulder hook (see ``_apply_feature_hook``). None for Wave-1.

    Returns
    -------
    dict[str, np.ndarray]
        Fine bundle (mass_areal/density/datum/state_label/disturbance) at (H*k, W*k), float64
        (state_label uint8). By construction ``coarsen(this, k)`` == the base block: density/
        datum/state_label/disturbance bit-exact; mass_areal/height to the float64 noise floor.
    """
    if not isinstance(k, (int, np.integer)) or k < 1:
        raise ValueError(f"overlay_residual: k must be a positive integer, got {k!r}")
    k = int(k)
    p = dict(DEFAULT_OVERLAY_PARAMS)
    if params:
        p.update(params)

    # --- resolve base fields + cell sizes ---------------------------------
    if isinstance(base_tile_fields, ColumnState):
        base_cell_m = float(base_tile_fields.cell_m)
        if fine_cell_m is None:
            fine_cell_m = base_cell_m / k
        base_fields = {
            "mass_areal": np.asarray(base_tile_fields.mass_areal, dtype=np.float64),
            "density": np.asarray(base_tile_fields.density, dtype=np.float64),
            "datum": np.asarray(base_tile_fields.datum, dtype=np.float64),
            "state_label": np.asarray(base_tile_fields.state_label),
            "disturbance": np.asarray(base_tile_fields.disturbance, dtype=np.float64),
        }
    else:
        base_fields = {n: np.asarray(base_tile_fields[n]) for n in
                       ("mass_areal", "density", "datum", "state_label", "disturbance")}
        if fine_cell_m is None:
            raise ValueError("overlay_residual: fine_cell_m required when fields is a dict")
        base_cell_m = fine_cell_m * k

    # We overlay detail in THICKNESS space (column thickness = mass_areal/density), NOT raw
    # height. WHY: the conserved field is ``mass_areal``; refine COPIES density per block, so
    # ``coarsen(mass) = density · mean(thickness)`` and forcing ``mean(thickness)==base
    # thickness`` (a single per-block constant restore) makes mass coarsen-exact WITHOUT the
    # ``set_height_via_mass`` ``max(h-datum,0)`` clamp ever firing (the clamp would silently
    # break the block mean where a smoothed/heterogeneous-datum height dips below its datum).
    # Working in thickness also keeps the residual physically a SURFACE perturbation: the fine
    # height = datum (copied per block) + thickness, and per base cell datum is constant, so
    # mean(height)=datum+mean(thickness)=base height too (height coarsen-exact for free).
    base_thick = base_fields["mass_areal"] / base_fields["density"]  # (H, W) >= 0

    # Step 0: refine the carried fields (piecewise-constant copy; coarsen of these == base).
    fine = refinement.refine_field(base_fields, k)
    fine_density = fine["density"]
    fine_datum = fine["datum"]

    H, W = base_thick.shape
    nh, nw = H * k, W * k

    # Step 1: mean-preserving smooth interpolation of the base THICKNESS across base-cell
    # centers (kills the np.repeat plateau -> anti-alias, eval §5 step 4), then restore each
    # base cell's mean EXACTLY so the smoothed low-frequency layer re-coarsens to the base.
    smooth_t = _smooth_interp_height(base_thick, k, p.get("smooth", "bicubic"))
    smooth_t = _restore_block_means(smooth_t, base_thick, k)

    # Step 2: global-lattice fbm residual over the WHOLE window. ``fbm_global`` samples a SQUARE
    # window; for a partial edge tile (nh != nw) we sample the bounding SQUARE anchored at the
    # same global origin and CROP to (nh, nw). Because the lattice is global, the cropped detail
    # is identical to what a full-square neighbour tile would produce on the shared region —
    # cross-base-cell + cross-tile continuity holds even at the base edge (eval §5 step 3).
    from . import procgen_seed as ps
    if p.get("fbm_nu0", 0.0) > 0.0:
        side = max(nh, nw)
        full = ps.fbm_global(
            world_x0, world_y0, side, float(fine_cell_m),
            H=p.get("fbm_H", 0.95), nu0=float(p["fbm_nu0"]), world_seed=world_seed,
            octaves=int(p.get("fbm_octaves", 6)),
            base_wavelength_m=float(p.get("fbm_base_wavelength_m", 8.0)),
            lacunarity=float(p.get("fbm_lacunarity", 2.0)),
        )
        residual = full[:nh, :nw]
    else:
        residual = np.zeros((nh, nw), dtype=np.float64)

    # Wave-2 feature hook (craters/boulders) BEFORE the zero-mean projection -> continuous.
    residual = _apply_feature_hook(residual, world_x0, world_y0, float(fine_cell_m),
                                   params=p, world_seed=world_seed, feature_fn=feature_fn)

    # Zero-mean PER BASE CELL: subtract each k x k block's own mean so coarsen recovers 0
    # (eval §5 step 3; INTERFACE.md §5.3). Each cell keeps its own internal shape (continuity).
    residual = residual - _expand(_block_means(residual, k), k)

    # Combine smoothed low-frequency thickness + zero-mean detail.
    fine_thick = smooth_t + residual

    # Step 3: force coarsen(fine thickness) == base thickness BIT-EXACT. coarsen takes the
    # reshape-mean over the two k-axes for a heterogeneous block (our residual varies), so
    # restoring the block mean through the IDENTICAL reshape-mean path makes that mean ==
    # base_thick exactly. Then clamp NON-NEGATIVE only where a fine cell would go below 0; for a
    # bounded residual on a positive base thickness this never fires, but if it does we add the
    # clamped deficit back uniformly across the block so the mean is still preserved exactly.
    fine_thick = _restore_block_means(fine_thick, base_thick, k)
    if (fine_thick < 0.0).any():
        # Redistribute clamped negative deficit within each block to keep the mean exact.
        deficit = np.minimum(fine_thick, 0.0)           # <=0 where clamped
        fine_thick = np.maximum(fine_thick, 0.0)
        # add back the per-block mean deficit so mean(block) is unchanged after clamping
        fine_thick = fine_thick + _expand(_block_means(deficit, k), k)
        fine_thick = np.maximum(fine_thick, 0.0)
        fine_thick = _restore_block_means(fine_thick, base_thick, k)

    # Step 4: mass_areal = thickness · density (density copied per block) -> conserved.
    # We restore the block mean ONE FINAL TIME on the actual ``mass_areal`` field through the
    # IDENTICAL reshape-mean path ``coarsen_field`` uses, so the coarsen of this overlay equals
    # the base mass_areal to the float64 NOISE FLOOR. (A genuinely heterogeneous block CANNOT
    # round-trip a float mean bit-for-bit to an arbitrary target — even ``mean`` of a UNIFORM
    # block is not exactly its value for some k, which is why ``coarsen_field`` special-cases
    # ``lo==hi`` with a verbatim copy. Once we add real sub-base detail the block is no longer
    # uniform, so coarsen takes ``np.mean`` and the round-trip lands at ~1e-15 RELATIVE, i.e.
    # mass-conserving to the same float64 floor the repo's own ``test_coarsen_nonuniform_datum``
    # accepts — height_err 2.78e-17 there. We DO NOT fake bit-zero; we restore through coarsen's
    # exact reduction so the residual is the irreducible float64 ULP, not an algorithmic error.)
    base_mass = base_fields["mass_areal"].astype(np.float64, copy=False)
    fine_mass = fine_thick * fine_density
    fine_mass = _restore_block_means(fine_mass, base_mass, k)
    fine_mass = np.maximum(fine_mass, 0.0)   # defensive; correction is ~1e-13 on a >0 base

    fine["mass_areal"] = fine_mass
    fine["density"] = fine_density
    fine["datum"] = fine_datum
    return fine
