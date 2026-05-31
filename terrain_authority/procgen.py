"""Pure-NumPy procedural generators for lunar terrain state (spec §4, §5, §9).

NO external 'noise' package — fbm/value-noise is implemented here from scratch with a
seeded RNG so scenes are deterministic/replayable (spec §10 determinism).

Every generator that changes the SURFACE backs the change out to the conserved
``mass_areal`` field via ColumnState.set_height_via_mass (or by editing mass directly),
so derive_height() reproduces the authored surface and the conservation invariants hold
(spec §10). Generators author surface + density; mass is the source of truth thereafter.
"""

from __future__ import annotations

import numpy as np

from . import constants as K
from .column_state import ColumnState, StateLabel


# ---------------------------------------------------------------------------
# Value noise + fractional Brownian motion (fbm) — no 'noise' pkg.
# ---------------------------------------------------------------------------

def _value_noise(height: int, width: int, cells: int, rng: np.random.Generator) -> np.ndarray:
    """Smooth value noise in [0,1] at ~``cells`` features across the grid.

    Lay a coarse (cells+1)x(cells+1) lattice of random values, bilinearly upsample with a
    smoothstep fade to the full (height,width) grid. Deterministic given ``rng``.
    """
    cells = max(1, cells)
    lattice = rng.random((cells + 1, cells + 1))

    # Normalized sample coords in lattice space.
    gy = np.linspace(0.0, cells, height)
    gx = np.linspace(0.0, cells, width)
    y0 = np.floor(gy).astype(int)
    x0 = np.floor(gx).astype(int)
    y1 = np.minimum(y0 + 1, cells)
    x1 = np.minimum(x0 + 1, cells)
    ty = gy - y0
    tx = gx - x0
    # Smoothstep fade (Perlin's 3t^2 - 2t^3).
    fy = (ty * ty * (3 - 2 * ty))[:, None]
    fx = (tx * tx * (3 - 2 * tx))[None, :]

    v00 = lattice[np.ix_(y0, x0)]
    v01 = lattice[np.ix_(y0, x1)]
    v10 = lattice[np.ix_(y1, x0)]
    v11 = lattice[np.ix_(y1, x1)]
    top = v00 * (1 - fx) + v01 * fx
    bot = v10 * (1 - fx) + v11 * fx
    return top * (1 - fy) + bot * fy


def fbm(height: int, width: int, octaves: int = 5, base_cells: int = 4,
        lacunarity: float = 2.0, gain: float = 0.5,
        seed: int = 0, *, normalize: str = "minmax",
        target_rms: float | None = None) -> np.ndarray:
    """Fractional Brownian motion: sum of octaves of value noise.

    base_cells features at octave 0, multiplied by ``lacunarity`` each octave; amplitude
    multiplied by ``gain``. Pure NumPy (no 'noise' pkg).

    ``normalize`` selects the OUTPUT scaling (default reproduces the legacy behaviour
    EXACTLY, so existing scenes/tests are byte-for-byte unchanged):

      "minmax"   (DEFAULT, legacy): renormalize to [0, 1] via (x-min)/(max-min). This is
                 what every existing caller gets. NOTE (docs/lunar_dem_10km_eval.md §6):
                 this min-max renorm is a realization-dependent NONLINEAR rescale that
                 DESTROYS the PSD slope the Hurst-derived ``gain`` is meant to set — it is
                 fine for the cosmetic equatorial archetypes but wrong for the DEM overlay.

      "variance": OPT-IN, for the DEM path. Skip the [0,1] renorm entirely; instead make
                 the field zero-mean and scale it to a target ROOT-MEAN-SQUARE deviation
                 ``target_rms`` (a deviogram/variance anchor, e.g. from PGDA Product-90
                 LDRM_RMSD). This preserves the spectral slope set by ``gain`` (use
                 constants.hurst_to_fbm_gain(H) for a sourced Hurst). Returns a zero-mean
                 field with RMS == target_rms (in the SAME UNITS as target_rms, e.g. m).
                 If the raw field is degenerate (RMS 0), returns zeros.

    Variance-anchored mode is the spectrally-faithful path the DEM residual overlay needs;
    "minmax" stays the default so nothing existing changes.
    """
    rng = np.random.default_rng(seed)
    total = np.zeros((height, width), dtype=np.float64)
    amp = 1.0
    cells = base_cells
    amp_sum = 0.0
    for _ in range(octaves):
        total += amp * _value_noise(height, width, int(round(cells)), rng)
        amp_sum += amp
        amp *= gain
        cells *= lacunarity
    total /= amp_sum

    if normalize == "minmax":
        # LEGACY path — unchanged byte-for-byte from the original implementation.
        lo, hi = total.min(), total.max()
        if hi > lo:
            total = (total - lo) / (hi - lo)
        return total

    if normalize == "variance":
        if target_rms is None:
            raise ValueError("fbm(normalize='variance') requires target_rms")
        centered = total - total.mean()
        rms = float(np.sqrt(np.mean(centered ** 2)))
        if rms <= 0.0:
            return np.zeros_like(centered)
        return centered * (float(target_rms) / rms)

    raise ValueError(f"fbm: unknown normalize={normalize!r} (use 'minmax' or 'variance')")


# ---------------------------------------------------------------------------
# Terrain archetypes
# ---------------------------------------------------------------------------

def rolling_hills(width: int, height: int, cell_m: float, *, seed: int = 1,
                  amplitude_m: float = 0.18, base_cells: int = 3) -> ColumnState:
    """Rolling 'fluffy' hills: higher relief, LOW-density loose top layer (spec §9).

    The loose-over-dense gradient is "the hinge for the three terrain states and
    multi-pass paving" (spec §9). Here the top layer is fluffy: density biased toward
    RHO_SURFACE, with crests slightly looser than troughs. Disturbance ~0 (undriven).
    """
    cs = ColumnState(width=width, height=height, cell_m=cell_m)
    relief = fbm(height, width, octaves=5, base_cells=base_cells, seed=seed)
    surface = (relief - relief.mean()) * amplitude_m  # zero-mean rolling surface [m]

    # Fluffy: low density, looser on the crests (where relief is high).
    cs.density = K.RHO_SURFACE * (1.0 - 0.12 * relief)  # crests ~12% looser
    cs.density = np.clip(cs.density, 0.9 * K.RHO_SURFACE, K.RHO_SURFACE)

    # Author surface as thickness above a datum so mass stays > 0 everywhere.
    cs.datum = np.full((height, width), surface.min() - K.Z_T)
    cs.set_height_via_mass(surface)
    cs.state_label[:] = StateLabel.VIRGIN
    cs.disturbance[:] = 0.02 * relief
    return cs


def flat_compact(width: int, height: int, cell_m: float, *, seed: int = 2,
                 amplitude_m: float = 0.01) -> ColumnState:
    """Flat, dense, low-disturbance terrain — a low-albedo proxy (spec §9; §8 optics).

    Used as a low-albedo proxy via HIGH compaction + LOW disturbance: a dense, smooth
    plate that the shader (spec §8) would render darker/firmer. Tiny micro-relief only.
    """
    cs = ColumnState(width=width, height=height, cell_m=cell_m)
    micro = fbm(height, width, octaves=3, base_cells=8, seed=seed)
    surface = (micro - micro.mean()) * amplitude_m

    cs.density = np.full((height, width), K.RHO_DEEP)  # compacted plate
    cs.datum = np.full((height, width), surface.min() - K.Z_T)
    cs.set_height_via_mass(surface)
    cs.state_label[:] = StateLabel.VIRGIN
    cs.disturbance[:] = 0.0
    return cs


# ---------------------------------------------------------------------------
# Crater carving (Pike-class fresh simple crater).
# ---------------------------------------------------------------------------

def carve_crater(cs: ColumnState, center_rc: tuple[int, int], diameter_m: float, *,
                 depth_ratio: float = K.CRATER_DEPTH_DIAMETER_RATIO,
                 rim_height_frac: float = K.CRATER_RIM_HEIGHT_FRAC,
                 ejecta_extent_radii: float = K.CRATER_EJECTA_EXTENT_RADII,
                 size_dependent: bool = False,
                 ejecta_mode: str = "quadratic") -> ColumnState:
    """Carve a parameterized fresh simple (Pike-class) crater into the surface, in-place.

    Profile (radial r from center, R = radius):
        floor: bowl of depth = depth_ratio*diameter at center, parabolic up to the rim.
        rim:   a raised lip of height rim_height_frac*depth at r=R, decaying outward.
        ejecta: a thin positive blanket out to ejecta_extent_radii*R.

    MASS-CONSISTENT (spec §6, §10): we edit the SURFACE then back it out to mass_areal at
    the local density. Removing material lowers mass; raising the rim/ejecta adds mass at
    the local density. (This single analytical profile is a stand-in: no degradation
    state, no true excavation-ejecta mass balance — Pike-class fresh morphometry only.)

    Pike-class depth/diameter ~0.2 for fresh simple lunar craters (constants.py).

    OPT-IN refinements (both default to the LEGACY behaviour, so existing callers/tests
    are byte-for-byte unchanged; see docs/lunar_dem_10km_eval.md §6):

      size_dependent (default False): when True AND the caller did not override
        ``depth_ratio``, use constants.crater_depth_ratio(diameter_m) — d/D ~ 0.196 above
        400 m (Pike 1977 [FIXED]) dropping to ~0.13 below 400 m (Stopar 2017 [CALIB]).
        A flat 0.2 is too deep for the sub-400 m craters procgen_csfd actually adds. An
        explicit ``depth_ratio`` argument always wins (back-compat: the default sentinel
        is the legacy constant, so passing it is indistinguishable from not passing it —
        size_dependent simply gates whether we substitute the sourced size-dependent law).

      ejecta_mode (default "quadratic"): "quadratic" is the legacy edge-keyed ramp;
        "mcgetchin" uses the empirical radial thickness ~ (r/R)^CRATER_EJECTA_DECAY_EXP
        (=-3.0; McGetchin 1973 / Settle & Head 1977 / Melosh 1989), normalized to the
        same rim-edge amplitude so the rim height is unchanged and the blanket thins
        outward by the sourced power law instead of a quadratic.
    """
    h, w = cs.height, cs.width
    cm = cs.cell_m
    R = 0.5 * diameter_m
    # Size-dependent d/D is OPT-IN and only substitutes when depth_ratio was left at its
    # legacy default (an explicit caller value always wins — byte-exact back-compat).
    if size_dependent and depth_ratio == K.CRATER_DEPTH_DIAMETER_RATIO:
        depth_ratio = K.crater_depth_ratio(diameter_m)
    depth = depth_ratio * diameter_m
    rim_h = rim_height_frac * depth
    r0, c0 = center_rc

    rows = (np.arange(h)[:, None] - r0) * cm
    cols = (np.arange(w)[None, :] - c0) * cm
    r = np.sqrt(rows ** 2 + cols ** 2)

    surface = cs.derive_height()
    delta = np.zeros_like(surface)

    # Inside the bowl: parabolic floor, depressed.
    inside = r <= R
    rn = np.clip(r / R, 0.0, 1.0)
    delta[inside] = -depth * (1.0 - rn[inside] ** 2)

    # Rim lip: gaussian bump centered at r=R, width ~0.25R.
    rim_sigma = 0.25 * R
    delta += rim_h * np.exp(-((r - R) ** 2) / (2 * rim_sigma ** 2))

    # Ejecta blanket: thin positive skirt beyond the rim out to ejecta extent.
    ej_outer = ejecta_extent_radii * R
    ej_region = (r > R) & (r <= ej_outer)
    if ejecta_mode == "quadratic":
        # LEGACY edge-keyed quadratic ramp — unchanged.
        ej_t = np.clip((ej_outer - r) / (ej_outer - R + 1e-9), 0.0, 1.0)
        delta[ej_region] += 0.15 * rim_h * (ej_t[ej_region] ** 2)
    elif ejecta_mode == "mcgetchin":
        # Empirical (r/R)^-3 radial decay (McGetchin 1973 / Settle & Head 1977 / Melosh
        # 1989). Normalize so thickness at the rim (r=R) equals the legacy rim-edge
        # amplitude 0.15*rim_h, then decay by the power law and taper to 0 at ej_outer so
        # the blanket ends cleanly (no discontinuity at the continuous-ejecta edge).
        rn_ej = r[ej_region] / R                       # >= 1 over the ejecta region
        amp = 0.15 * rim_h * (rn_ej ** K.CRATER_EJECTA_DECAY_EXP)  # =0.15*rim_h at r=R
        taper = np.clip((ej_outer - r[ej_region]) / (ej_outer - R + 1e-9), 0.0, 1.0)
        delta[ej_region] += amp * taper
    else:
        raise ValueError(
            f"carve_crater: unknown ejecta_mode={ejecta_mode!r} (use 'quadratic' or 'mcgetchin')")

    new_surface = surface + delta
    cs.set_height_via_mass(new_surface)

    # Label: bowl interior is freshly excavated (dense sublayer exposed -> brighter,
    # spec §6); bump up density on the exposed floor and bump disturbance there.
    floor = r <= 0.8 * R
    cs.state_label[floor] = StateLabel.EXCAVATED
    cs.density[floor] = np.clip(cs.density[floor] * 1.15, None, K.RHO_DEEP)
    cs.disturbance[inside] = np.clip(cs.disturbance[inside] + 0.5 * (1 - rn[inside]), 0, 1)
    # Re-back-out mass after the density change so height stays consistent at new rho.
    cs.set_height_via_mass(new_surface)
    return cs


# ---------------------------------------------------------------------------
# Golombek boulder-field sampler (cumulative fractional AREA SFD).
# ---------------------------------------------------------------------------

def sample_boulders(width: int, height: int, cell_m: float, k: float, *,
                    d_min_m: float = 0.04, d_max_m: float = 0.6,
                    seed: int = 7) -> list[dict]:
    """Sample a Golombek rock field as a clast list (INTERFACE.md §5 clasts schema).

    rock-size-freq_abstract.txt (Golombek et al. 2003): F_k(D) = k*exp(-q(k)*D) is the
    cumulative FRACTIONAL AREA covered by rocks of diameter >= D, with k the total
    fractional area covered by all rocks and q(k) = 1.79 + 0.152/k. Family of
    non-crossing curves; total rock abundance 5-40%.

    We invert the area SFD to a count distribution. The number of rocks with diameter in
    [D, D+dD] per unit area follows from differentiating the area coverage and dividing
    by per-rock area (pi/4 D^2):

        dF/dD = -k*q*exp(-q*D)              (area coverage density)
        n(D) dD = (-dF/dD) / (pi/4 D^2) dD   (count density per unit area)

    We bin [d_min, d_max], compute expected counts over the patch area, Poisson-sample,
    place rocks at uniform-random positions, and assign a random buried fraction.

    Try k=0.05 (sparse) and k=0.2 (rocky) per the task.

    PAPERED OVER: clast positions/sizes are independently sampled (no spatial clustering,
    no overlap rejection, no slope/ejecta correlation); buried_frac is uniform random,
    not derived from local relief.
    """
    rng = np.random.default_rng(seed)
    q = K.golombek_q(k)
    patch_area = (width * cell_m) * (height * cell_m)  # m^2

    # Diameter bins (log-spaced to resolve the steep small end).
    edges = np.geomspace(d_min_m, d_max_m, 18)
    centers = np.sqrt(edges[:-1] * edges[1:])
    widths = np.diff(edges)

    # Area-coverage density dF/dD = k*q*exp(-q*D); per-rock area pi/4 D^2.
    area_density = k * q * np.exp(-q * centers)            # 1/m of fractional area
    per_rock_area = (np.pi / 4.0) * centers ** 2           # m^2
    count_density_per_area = area_density / per_rock_area  # rocks / m^2 / m
    expected_counts = count_density_per_area * widths * patch_area

    clasts: list[dict] = []
    cid = 0
    Wm = width * cell_m
    Hm = height * cell_m
    for D, lam in zip(centers, expected_counts):
        n = int(rng.poisson(lam))
        for _ in range(n):
            x = float(rng.uniform(0, Wm))
            z = float(rng.uniform(0, Hm))
            buried = float(rng.uniform(0.1, 0.7))
            radius = 0.5 * float(D)
            # center height (y-up): partially buried -> center sits below surface by
            # buried_frac*diameter. Use surface ~0 reference (metadata is descriptive).
            y_center = radius - buried * float(D)
            clasts.append({
                "id": cid,
                "center_m": [round(x, 4), round(y_center, 4), round(z, 4)],
                "radius_m": round(radius, 4),
                "shape": "sphere",
                "buried_frac": round(buried, 3),
            })
            cid += 1
    return clasts
