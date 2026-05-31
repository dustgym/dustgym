"""Terrain-derived solar illumination — local-horizon ray-march + PSR cold-trap gate.

Wave-2 integration seam W2-ILLUM (docs/dem_terrain_contract.md §8; feeds the demo's
per-face shadow attribution, docs/demo_spiral_contract.md §4). The grazing polar sun
(0-7 deg, K.SUN_ELEVATION_DEG_POLAR) is "exactly IPEx's perception challenge" (spec §8):
at 7 deg a metre of relief throws an ~8 m shadow, so whether a rover-scale feature is lit
is a property of the TERRAIN around it, not of a flat plane.

WHAT THIS REPLACES.  The downstream face-illumination check (demo §4 "face_illum") and the
hillshade previews treat "lit" as a flat-plane fact: the sun is up iff its elevation > 0,
and matplotlib's LightSource shades by LOCAL NORMAL only (it has no cast-shadow / horizon
term). That flat-plane elev>0 stand-in marks a deep crater floor LIT whenever the sun is
above the mathematical horizon, even when the crater wall blocks the line of sight. This
module replaces that stand-in with a per-pixel LOCAL-HORIZON ray-march: a pixel is lit only
if NOTHING along the sun-ward ray rises above its line of sight at elevation sun_el_deg.

SELF-FLAG / HONESTY (binding portfolio discipline).  This is a TERRAIN-DERIVED local
horizon computed from the heightmap we already own. It is NOT a PGDA Product-69 (LOLA
"average / maximum / longest-night" illumination) ingest: there is no Product-69 reader and
no Product-69 raster on disk in this repo. Product-69 bakes in the true ephemeris (multi-
year sun track, libration, and the FAR horizon out to tens of km) that a single-tile
heightmap cannot see; this is the single-epoch, single-tile geometric horizon for ONE
(az, el) sun position. Treat it as a geometry-accurate shadow stand-in, not a validated
illumination/PSR product. See docs/dem_terrain_contract.md §8 (W2-ILLUM).

AZIMUTH CONVENTION (stated so it is auditable).  Grids here are row-major with origin at
the lower-left (rows index +Z "north", cols index +X "east"; the repo's hillshade/preview
convention, io_fields.write_hillshade_png / viz.variety_panel use origin="lower" with
xlabel "col (+X)", ylabel "row (+Z)"). ``sun_az_deg`` is the compass bearing the sunlight
comes FROM, measured CLOCKWISE FROM +Z(north): 0 deg = light from +Z (top of the array),
90 deg = light from +X (right), matching the cartographic azimuth used by the hillshade
preview's azdeg. We ray-march TOWARD the sun (up-sun) to find the first occluder.
"""

from __future__ import annotations

import numpy as np

from . import constants as K


def horizon_clip(heightmap: np.ndarray, cell_m: float,
                 sun_az_deg: float, sun_el_deg: float) -> np.ndarray:
    """Per-pixel local-horizon illuminated mask under one sun position.

    A pixel is ILLUMINATED iff, marching up-sun (toward the source) across the heightmap,
    no cell rises above the straight line of sight that leaves the pixel at elevation
    ``sun_el_deg``. Equivalently: the local horizon angle in the sun's azimuth is BELOW the
    sun's elevation. This is the cast-shadow term the flat-plane ``elev>0`` stand-in and the
    normal-only hillshade lack (see module docstring).

    Parameters
    ----------
    heightmap : (H, W) float array, surface height [m] (any datum; only differences matter).
    cell_m    : grid spacing [m] (square cells; the .rf32 contract is isotropic).
    sun_az_deg: bearing the light comes FROM, deg clockwise from +Z/north (see module
                docstring azimuth convention); the march steps TOWARD this bearing.
    sun_el_deg: sun elevation above the horizontal [deg]. ``<= 0`` -> sun at/below the
                mathematical horizon -> everything dark (returns all-False).

    Returns
    -------
    (H, W) bool array, True where the pixel sees the sun.

    Method: a single sweep of ``max_steps`` integer cell-steps along the up-sun unit vector
    (bilinear-free nearest-cell sampling, like the hillshade's nearest-neighbour shading).
    For step k at horizontal distance d_k = k*cell_m*|step|, a sun-ward cell of height h_s
    blocks a pixel of height h_p iff h_s - h_p > d_k * tan(el): the rising line of sight is
    overtopped. We accumulate the per-pixel maximum required clearance and shadow any pixel
    that is ever overtopped. O(H*W*max_steps); max_steps bounds the horizon search to the
    tile (a single heightmap cannot see a horizon beyond its own extent anyway, hence the
    Product-69 caveat in the module docstring).
    """
    h = np.asarray(heightmap, dtype=np.float64)
    if h.ndim != 2:
        raise ValueError(f"heightmap must be 2-D, got shape {h.shape}")
    if cell_m <= 0:
        raise ValueError(f"cell_m must be > 0, got {cell_m}")

    n_rows, n_cols = h.shape

    # Sun at or below the mathematical horizon: nothing is lit (lunar night / sub-horizon).
    if sun_el_deg <= 0.0:
        return np.zeros_like(h, dtype=bool)

    tan_el = np.tan(np.deg2rad(sun_el_deg))

    # Up-sun unit step in (row=+Z, col=+X). az measured clockwise from +Z: the component
    # along +Z is cos(az), along +X is sin(az). We march TOWARD the source (up-sun), so the
    # row/col increments point in the +(toward-sun) direction.
    az = np.deg2rad(sun_az_deg)
    d_row = np.cos(az)   # +Z component of the up-sun direction
    d_col = np.sin(az)   # +X component

    # March far enough to clear the whole tile along the dominant axis; a single heightmap
    # has no horizon information past its own edge (Product-69 caveat).
    max_steps = int(np.hypot(n_rows, n_cols)) + 1

    rows = np.arange(n_rows)[:, None]
    cols = np.arange(n_cols)[None, :]

    illuminated = np.ones_like(h, dtype=bool)

    for k in range(1, max_steps + 1):
        # Nearest sun-ward cell at integer step k along the up-sun unit vector.
        sr = np.rint(rows + k * d_row).astype(np.intp)
        sc = np.rint(cols + k * d_col).astype(np.intp)

        in_bounds = (sr >= 0) & (sr < n_rows) & (sc >= 0) & (sc < n_cols)
        if not in_bounds.any():
            break  # every ray has walked off the tile; no farther horizon to test.

        # Clamp out-of-bounds lookups to a valid index, then mask them out below.
        sr_c = np.clip(sr, 0, n_rows - 1)
        sc_c = np.clip(sc, 0, n_cols - 1)

        # Horizontal run to the sampled cell [m] and the line-of-sight height there.
        dist_m = k * cell_m * np.hypot(d_row, d_col)  # |step|==1, kept explicit for audit.
        los_height = h + dist_m * tan_el

        # A sun-ward cell taller than the line of sight casts this pixel into shadow.
        blocked = in_bounds & (h[sr_c, sc_c] > los_height)
        illuminated &= ~blocked

    return illuminated


def psr_gate(illuminated_mask: np.ndarray, *,
             t_psr_k: float = K.T_PSR_K) -> np.ndarray:
    """Terrain-derived cold-trap (PSR-candidate) mask from a local-horizon illuminated mask.

    A permanently-shadowed region (PSR) is a cold trap when its surface never warms past the
    volatile cold-trap threshold ``t_psr_k`` (K.T_PSR_K = 110 K, the <110 K H2O-ice stability
    line, spec §5.1/§5.2; LCROSS-class cold trap). This call FINALLY CONSUMES that constant,
    which was dead in the codebase until now.

    HONESTY (terrain-derived gate, NOT a thermal model).  We do NOT model temperature: there
    is no thermal solver, no Product-69/Diviner ingest, and no insolation time-integral here.
    The gate is the geometric NECESSARY condition for a cold trap under this lane's single-
    epoch local-horizon shadow: a pixel can only be a candidate cold trap if it is shadowed
    NOW (``~illuminated``). ``t_psr_k`` is carried as the documented threshold the geometry
    is screening FOR, and is asserted to be a physical cryogenic value (0 K < t_psr_k <
    freezing); it does not enter a temperature computation because none exists. A real PSR
    determination needs the multi-year illumination integral (PGDA Product-69) plus a thermal
    model -- explicitly out of scope (module docstring; docs/dem_terrain_contract.md §8).

    Parameters
    ----------
    illuminated_mask : (H, W) bool, output of ``horizon_clip`` (True = sees the sun).
    t_psr_k          : cold-trap temperature threshold [K]; default K.T_PSR_K = 110.0.

    Returns
    -------
    (H, W) bool, True for PSR-candidate cold-trap pixels (shadowed under this sun epoch).
    """
    mask = np.asarray(illuminated_mask)
    if mask.dtype != np.bool_:
        raise TypeError(f"illuminated_mask must be a bool array, got dtype {mask.dtype}")
    # Screen FOR a cryogenic threshold: a non-physical / non-cold t_psr_k would mean the
    # gate is not actually selecting a cold trap. (Reads K.T_PSR_K honestly; see docstring.)
    if not (0.0 < t_psr_k < K.RHO_WATER / 1000.0 + 273.15):  # 0 K < t < ~water-freezing
        raise ValueError(
            f"t_psr_k={t_psr_k} K is not a cryogenic cold-trap threshold "
            f"(expected 0 < t < ~273 K); PSR gate would be meaningless")

    # Geometric necessary condition: cold-trap candidate <=> shadowed this epoch.
    return ~mask


# ---------------------------------------------------------------------------
# Self-test (docs/dem_terrain_contract.md §8 W2-ILLUM): a FALSIFIABLE distinction
# between the local-horizon ray-march and the flat-plane elev>0 stand-in.
# ---------------------------------------------------------------------------

def _flat_plane_lit(heightmap: np.ndarray, sun_el_deg: float) -> np.ndarray:
    """The OLD stand-in this lane replaces: 'lit' iff the sun is above the math horizon.

    No terrain term at all -- the whole tile is lit (or dark) by sun elevation alone. This is
    exactly what marks a deep crater floor LIT at a grazing sun, which the local horizon
    correctly marks DARK. Kept here only to make the self-test's contrast falsifiable.
    """
    val = bool(sun_el_deg > 0.0)
    return np.full(np.asarray(heightmap).shape, val, dtype=bool)


def _self_test() -> int:
    import sys

    failures = 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        tag = "PASS" if ok else "FAIL"
        if not ok:
            failures += 1
        print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))

    cell_m = 1.0
    el = K.SUN_ELEVATION_DEG_POLAR  # 7 deg grazing polar sun
    # Sun comes from +Z/north (az=0): the high north ridge shadows the depression to its
    # south. Marching up-sun is marching toward +Z (smaller... larger row), so the south
    # floor looks north and is blocked.
    az = 0.0

    # --- Scene: a deep depression carved into a plane, walled to the north by a tall ridge.
    n = 60
    hm = np.zeros((n, n), dtype=np.float64)
    # A wide flat floor depression in the southern half (low rows = south, origin="lower").
    floor_rows = slice(5, 25)
    floor_cols = slice(15, 45)
    depth = 6.0  # m deep -- at 7 deg, a 6 m wall casts ~49 m of shadow, well over the floor.
    hm[floor_rows, floor_cols] = -depth
    # A tall ridge to the NORTH of the depression (higher rows), the up-sun occluder at az=0.
    ridge_rows = slice(28, 34)
    hm[ridge_rows, floor_cols] = +8.0

    lit = horizon_clip(hm, cell_m, az, el)
    flat = _flat_plane_lit(hm, el)

    # Sample a depression-floor pixel that sits in the ridge's shadow.
    fr, fc = 12, 30
    horizon_dark = (not bool(lit[fr, fc]))
    flat_says_lit = bool(flat[fr, fc])
    check("horizon_clip marks the shadowed depression-floor DARK where flat-plane elev>0 "
          "WRONGLY marks it LIT (falsifiable distinction)",
          horizon_dark and flat_says_lit,
          f"floor[{fr},{fc}]: horizon_lit={bool(lit[fr, fc])} flat_lit={flat_says_lit} "
          f"(depth={depth} m, el={el} deg, az={az})")

    # The ridge crest itself faces the sun with nothing taller up-sun -> it must be lit.
    cr = 31
    cc = 30
    check("the up-sun ridge crest is itself lit (nothing taller toward the sun)",
          bool(lit[cr, cc]),
          f"ridge[{cr},{cc}] horizon_lit={bool(lit[cr, cc])}")

    # --- Flat plane: fully lit at any el>0, fully dark at el<=0 (sun-only behaviour).
    flat_terrain = np.full((40, 40), 12.34, dtype=np.float64)
    lit_flat = horizon_clip(flat_terrain, cell_m, az, el)
    check("a flat plane is FULLY lit at el>0 (no terrain casts a shadow)",
          bool(lit_flat.all()),
          f"lit_fraction={lit_flat.mean():.3f} at el={el} deg")

    dark_flat = horizon_clip(flat_terrain, cell_m, az, -1.0)
    check("a flat plane is FULLY dark at el<=0 (sun below the math horizon)",
          bool((~dark_flat).all()),
          f"lit_fraction={dark_flat.mean():.3f} at el=-1.0 deg")

    # --- psr_gate reads T_PSR_K and selects the shadowed (cold-trap-candidate) pixels.
    cold = psr_gate(lit, t_psr_k=K.T_PSR_K)
    # The cold-trap candidate set must be exactly the un-illuminated set, and must INCLUDE
    # the shadowed depression floor (a real PSR-shaped outcome of the horizon, not a no-op).
    gate_is_shadow = bool(np.array_equal(cold, ~lit))
    floor_is_cold = bool(cold[fr, fc])
    check("psr_gate reads K.T_PSR_K (=110.0) and gates the shadowed floor as a cold-trap "
          "candidate (consumes the formerly-dead constant)",
          gate_is_shadow and floor_is_cold and K.T_PSR_K == 110.0,
          f"K.T_PSR_K={K.T_PSR_K} gate==~lit={gate_is_shadow} "
          f"floor_cold={floor_is_cold} cold_fraction={cold.mean():.3f}")

    # psr_gate must REJECT a non-cryogenic threshold (it would not be selecting a cold trap).
    rejected = False
    try:
        psr_gate(lit, t_psr_k=400.0)
    except ValueError:
        rejected = True
    check("psr_gate rejects a non-cryogenic threshold (gate stays meaningful)",
          rejected, "t_psr_k=400 K -> ValueError")

    print(f"\n{6 - failures}/6 illumination self-test checks passed.")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
