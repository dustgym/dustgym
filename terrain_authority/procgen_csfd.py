"""Crater-population generator — sub-DEM crater size-frequency synthesis (Lane B).

Companion to ``procgen.sample_boulders``: where that samples a Golombek rock field as a
clast list, this samples a CRATER field as a list of (center, diameter) stamps and carves
each one with ``procgen.carve_crater``. It is the population sampler the single-crater
``carve_crater`` lacked (docs/dem_terrain_contract.md §6, docs/lunar_dem_10km_eval.md §6/§7).

THE SOURCED MODEL (every parameter tagged in constants.py; see that file for citations):

  expected cumulative density   N(>=D) = min( production(D, T), equilibrium(D) )   [/m^2]
      production : Neukum/Ivanov/Hartmann 2001 production polynomial at the committed
                   surface age T (constants.neukum_production_cumulative)   [CALIB]
      equilibrium: Xiao & Werner 2015 steady-state cap n_eq(>=D)=0.084 D^-2  [CALIB]
                   (a surface in equilibrium has erased as many small craters as it gains)

  de-confliction: only synthesize craters BELOW the DEM's effective resolution
      D_max = dem_effective_resolution / LDEM_EFFRES_NYQUIST_MULT-style cut — craters the
      DEM already RESOLVES are not re-synthesized (they are in the base heightmap already).
      The caller passes ``dem_effective_resolution`` (e.g. from PGDA Product-90 LDEM_EFFRES).

Per log-D bin we difference the CAPPED cumulative curve -> expected count per bin over the
patch area, Poisson-sample (so the realized count is stochastic but the mean obeys the cap),
place at uniform-random positions, and stamp with carve_crater using the SOURCED size-
dependent depth/diameter (Stopar 2017, shallower below 400 m) and McGetchin (r/R)^-3 ejecta.

PURE stdlib + numpy; seeded -> bit-reproducible (spec §10). carve_crater is mass-conserving,
so the carved field stays height==datum+mass/density consistent (carve_crater backs every
edit out to mass_areal). PAPERED OVER: craters are placed independently (no spatial
clustering / overlap rejection / pre-existing-crater degradation); positions uniform.
"""

from __future__ import annotations

import numpy as np

from . import constants as K
from . import procgen
from .column_state import ColumnState


def expected_crater_counts(d_edges: np.ndarray, area_m2: float,
                           *, age_gyr: float = K.NEUKUM_SURFACE_AGE_GYR,
                           apply_equilibrium_cap: bool = True,
                           ) -> tuple[np.ndarray, np.ndarray]:
    """Expected crater count per log-D bin over ``area_m2``, capped at equilibrium.

    Returns (centers, expected_counts) for the bins defined by ``d_edges`` [m]:
      centers          geometric-mean diameter of each bin [m],
      expected_counts  mean number of craters with D in [edge_i, edge_{i+1}] over the area.

    The count in a bin is the difference of the CUMULATIVE capped density across the bin
    edges times the area:  count_i = (Ncap(>=edge_i) - Ncap(>=edge_{i+1})) * area, with
    Ncap(>=D) = min(production(D, age), eq_sfd(D)) when apply_equilibrium_cap (default),
    else the bare Neukum production. Capping the CUMULATIVE curve (not per-bin) keeps the
    cap monotone and exactly the Xiao & Werner steady-state ceiling.
    """
    d_edges = np.asarray(d_edges, dtype=np.float64)
    prod_cum = K.neukum_production_cumulative(d_edges, age_gyr=age_gyr)
    if apply_equilibrium_cap:
        cum = np.minimum(prod_cum, K.eq_sfd(d_edges))
    else:
        cum = prod_cum
    # Cumulative N(>=D) is non-increasing in D; per-bin density = drop across the bin.
    per_bin_density = np.maximum(cum[:-1] - cum[1:], 0.0)   # /m^2 in [edge_i, edge_i+1]
    centers = np.sqrt(d_edges[:-1] * d_edges[1:])
    return centers, per_bin_density * float(area_m2)


def populate_craters(cs: ColumnState, dem_effective_resolution_m: float, *,
                     d_min_m: float = 1.0,
                     nyquist_mult: float = K.LDEM_EFFRES_NYQUIST_MULT,
                     age_gyr: float = K.NEUKUM_SURFACE_AGE_GYR,
                     apply_equilibrium_cap: bool = True,
                     n_bins: int = 16,
                     size_dependent_depth: bool = True,
                     ejecta_mode: str = "mcgetchin",
                     seed: int = 0,
                     return_records: bool = False,
                     ) -> ColumnState | tuple[ColumnState, list[dict]]:
    """Synthesize a sub-DEM crater population into ``cs`` IN-PLACE and return it.

    Only craters in [d_min_m, D_max] are synthesized, where
        D_max = dem_effective_resolution_m / nyquist_mult
    (de-confliction: craters at/above the DEM effective resolution are ALREADY in the base
    heightmap, so we synthesize strictly BELOW it; nyquist_mult ~2-3 is the
    LDEM_EFFRES_NYQUIST_MULT engineering heuristic). If D_max <= d_min_m the band is empty
    and the grid is returned unchanged (a no-op — correct when the DEM already resolves
    everything down to d_min_m).

    Expected counts per log-D bin come from expected_crater_counts (Neukum production
    capped at Xiao & Werner equilibrium); each bin is Poisson-sampled, craters placed at
    uniform-random positions, and stamped with carve_crater using the sourced size-dependent
    depth/diameter (Stopar 2017) and McGetchin (r/R)^-3 ejecta by default.

    Deterministic given ``seed`` (numpy default_rng). If ``return_records`` also returns the
    list of placed-crater dicts {center_rc, diameter_m, depth_ratio}.
    """
    rng = np.random.default_rng(seed)
    d_max_m = dem_effective_resolution_m / nyquist_mult
    records: list[dict] = []

    if d_max_m <= d_min_m:
        # DEM resolves everything down to d_min_m -> nothing to synthesize.
        return (cs, records) if return_records else cs

    area_m2 = (cs.width * cs.cell_m) * (cs.height * cs.cell_m)
    d_edges = np.geomspace(d_min_m, d_max_m, n_bins + 1)
    centers, expected = expected_crater_counts(
        d_edges, area_m2, age_gyr=age_gyr, apply_equilibrium_cap=apply_equilibrium_cap)

    Wm = cs.width * cs.cell_m
    Hm = cs.height * cs.cell_m
    for D, lam in zip(centers, expected):
        n = int(rng.poisson(lam))
        for _ in range(n):
            # Uniform position in metres -> cell index (row=z, col=x; INTERFACE.md §2).
            x_m = float(rng.uniform(0.0, Wm))
            z_m = float(rng.uniform(0.0, Hm))
            c0 = int(round(x_m / cs.cell_m))
            r0 = int(round(z_m / cs.cell_m))
            c0 = min(max(c0, 0), cs.width - 1)
            r0 = min(max(r0, 0), cs.height - 1)
            procgen.carve_crater(cs, (r0, c0), float(D),
                                 size_dependent=size_dependent_depth,
                                 ejecta_mode=ejecta_mode)
            if return_records:
                dr = K.crater_depth_ratio(float(D)) if size_dependent_depth \
                    else K.CRATER_DEPTH_DIAMETER_RATIO
                records.append({
                    "center_rc": [r0, c0],
                    "diameter_m": round(float(D), 5),
                    "depth_ratio": round(float(dr), 4),
                })
    return (cs, records) if return_records else cs


# ---------------------------------------------------------------------------
# Self-test (spec §10 determinism + the §7 falsifiable acceptance properties).
#   python -m terrain_authority.procgen_csfd
# Checks, on a self-contained synthetic ColumnState:
#   1. reproducibility — same seed -> identical placed-crater records;
#   2. equilibrium cap — emplaced cumulative density per log-D bin <= eq_sfd cap;
#   3. DEM-resolution cutoff — every synthesized D is strictly below D_max, and a DEM
#      that resolves down to d_min yields ZERO craters (the no-op de-confliction);
#   4. mass/height consistency — carve_crater keeps height==datum+mass/density.
# Prints PASS/FAIL and exits nonzero on any failure.
# ---------------------------------------------------------------------------

def _make_patch(width: int = 200, height: int = 200, cell_m: float = 0.5) -> ColumnState:
    """A flat self-contained base patch (no dependency on scenes/dem_import)."""
    cs = ColumnState(width=width, height=height, cell_m=cell_m)
    cs.density[:] = K.RHO_SURFACE
    cs.datum[:] = -K.REGOLITH_THICKNESS_M     # deep datum so bowls never clamp to 0 mass
    cs.set_height_via_mass(np.zeros((height, width)))
    return cs


def _self_test() -> int:

    results: list[tuple[str, bool, str]] = []

    def check(name: str, ok: bool, detail: str = "") -> None:
        results.append((name, bool(ok), detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ({detail})" if detail else ""))

    # A 100 m patch (0.5 m cells) with a DEM effective resolution of 15 m (Haworth-group
    # median, Barker 2023). D_max = 15 / 2.5 = 6 m; synthesize craters in [1 m, 6 m].
    eff_res = 15.0
    d_min = 1.0
    nyq = K.LDEM_EFFRES_NYQUIST_MULT
    d_max = eff_res / nyq

    # 1. reproducibility ----------------------------------------------------
    cs_a = _make_patch()
    _, rec_a = populate_craters(cs_a, eff_res, d_min_m=d_min, seed=12345, return_records=True)
    cs_b = _make_patch()
    _, rec_b = populate_craters(cs_b, eff_res, d_min_m=d_min, seed=12345, return_records=True)
    cs_c = _make_patch()
    _, rec_c = populate_craters(cs_c, eff_res, d_min_m=d_min, seed=999, return_records=True)
    same_seed = rec_a == rec_b
    diff_seed = rec_a != rec_c
    check("reproducible: same seed -> identical craters; different seed differs",
          same_seed and diff_seed and len(rec_a) > 0,
          f"n={len(rec_a)} same_seed={same_seed} diff_seed={diff_seed}")

    # 2. equilibrium cap: emplaced cumulative density <= eq_sfd at each bin edge --------
    area = (cs_a.width * cs_a.cell_m) * (cs_a.height * cs_a.cell_m)
    diam = np.array([r["diameter_m"] for r in rec_a])
    cap_ok = True
    cap_detail = []
    for D in np.geomspace(d_min, d_max, 6):
        emplaced_cum = float(np.count_nonzero(diam >= D)) / area    # /m^2
        cap = float(K.eq_sfd(D))                                    # /m^2
        # Allow Poisson over-shoot tolerance: 1 extra crater over the area is the
        # smallest resolvable density step; cap is satisfied within that quantum + 25%.
        tol = cap * 0.25 + 1.0 / area
        ok = emplaced_cum <= cap + tol
        cap_ok = cap_ok and ok
        cap_detail.append(f"D={D:.2f}:{emplaced_cum:.3e}<= {cap:.3e}")
    check("equilibrium cap: emplaced cumulative density <= Xiao&Werner eq_sfd per bin",
          cap_ok, "  ".join(cap_detail))

    # 3a. every synthesized D strictly below D_max (and >= d_min) ----------------------
    band_ok = bool(diam.size and diam.max() < d_max + 1e-9 and diam.min() >= d_min - 1e-9)
    # 3b. a DEM resolving down to d_min (eff_res = d_min*nyq) yields ZERO craters --------
    cs_z = _make_patch()
    _, rec_z = populate_craters(cs_z, d_min * nyq, d_min_m=d_min, seed=7, return_records=True)
    noop_ok = (len(rec_z) == 0) and np.array_equal(cs_z.derive_height(),
                                                   _make_patch().derive_height())
    check("dem-resolution cutoff: all D in [d_min, D_max); DEM-resolves-all -> no-op",
          band_ok and noop_ok,
          f"D_max={d_max:.2f} maxD={diam.max():.2f} minD={diam.min():.2f} noop_craters={len(rec_z)}")

    # 4. mass/height consistency after carving the population --------------------------
    h = cs_a.derive_height()
    expect = cs_a.datum + cs_a.mass_areal / cs_a.density
    err = float(np.max(np.abs(h - expect)))
    check("mass-consistent: height == datum + mass/density after carving population",
          err <= 1e-9, f"max_err={err:.2e} m")

    n_fail = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{len(results) - n_fail}/{len(results)} checks passed.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
