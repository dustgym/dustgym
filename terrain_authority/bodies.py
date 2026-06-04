"""bodies.py — per-planet terramechanics constants for the dustgym environments.

Each ``Body`` carries SOURCED surface/regolith mechanics for a planetary body that is a real
habitat and/or ISRU-mining target (systematic review: docs/bodies_sysrev.md). Values are tagged
MEASURED (in-situ / returned-sample), ESTIMATED (simulant / model / analog), or UNKNOWN. Nothing is
fabricated: where the literature has no value, the field is None and the repo's lunar baseline stands
in as an explicit, flagged analog.

Bodies (top targets per agency roadmaps + peer-reviewed proposals; science-only bodies such as
Europa/Enceladus/Titan/Vesta are intentionally excluded):
    moon   - Artemis base + ISRU (water ice). Full Apollo/LTV data.            [MEASURED]
    mars   - human habitats + ISRU (MOXIE flown). Native Bekker UNKNOWN; GRC-3 simulant used. [EST]
    ceres  - water/brine mining + Janhunen habitat proposal. Dawn gravity/repose; Bekker UNKNOWN.
    bennu  - C-type asteroid water mining (OSIRIS-REx). MICROGRAVITY: Bekker model INVALID.
    phobos - Mars-system staging waypoint (JAXA MMX). MILLI-GRAVITY: Bekker analog only.
    earth  - validation/sanity body (Wong dry-sand reference).

bekker_regime flags where the gravity-loaded Bekker pressure-sinkage model applies:
    "gravity-loaded"  - model valid (Moon/Mars/Ceres/Earth).
    "microgravity"    - model OUT OF REGIME (Bennu/Phobos): ~no overburden, cohesion/granular-dynamics
                        dominate (granular Bond number / DEM), NOT Bekker. The drive env still runs but
                        emits a warning; treat results as a placeholder, not physics.
"""
from __future__ import annotations

import dataclasses
import math

from .terramechanics import TerramechanicsParams


@dataclasses.dataclass(frozen=True)
class Body:
    name: str                          # canonical key (lowercase)
    label: str                         # display name
    g: float                           # representative surface gravity [m/s^2]
    bekker_regime: str                 # "gravity-loaded" | "microgravity"
    bulk_density: float | None = None  # surface regolith bulk density [kg/m^3]
    cohesion_pa: float | None = None   # representative cohesion [Pa]
    friction_deg: float | None = None  # internal friction angle [deg]
    repose_deg: float | None = None    # angle of repose [deg]
    bekker: tuple | None = None        # (k_c [N/m^2], k_phi [N/m^3], n) in repo units, if sourced
    confidence: str = ""               # per-field MEASURED/ESTIMATED/UNKNOWN summary
    g_note: str = ""                   # gravity variation note (asteroids/Phobos vary strongly)
    role: str = ""                     # habitat/mining role + citation
    provenance: str = ""               # key citations for the soil constants


# ---- the registry (constants + citations: docs/bodies_sysrev.md) --------------------------------
BODIES = {
    "moon": Body(
        "moon", "Moon", 1.62, "gravity-loaded",
        bulk_density=1300.0, cohesion_pa=170.0, friction_deg=35.0, repose_deg=35.0,
        bekker=(1400.0, 820000.0, 1.0),
        confidence="g/density/cohesion/friction MEASURED (Apollo + ChaSTE); Bekker MEASURED (NASA LTV).",
        role="Artemis base camp + ISRU water ice (NASA Moon-to-Mars ADD Rev.B 2024).",
        provenance="NASA LTV terramechanics white paper NTRS 20220010732 (k_c=1400 N/m^2, k_phi=820000 "
                   "N/m^3, n=1.0, c=170 Pa); Carrier/Mitchell Lunar Sourcebook 1991; ChaSTE 2025 (rho).",
    ),
    "mars": Body(
        "mars", "Mars", 3.71, "gravity-loaded",
        bulk_density=1500.0, cohesion_pa=1000.0, friction_deg=35.0, repose_deg=33.0,
        bekker=(23200.0, 606700.0, 1.0),
        confidence="g MEASURED; density/cohesion/friction MEASURED in-situ (MER/InSight); Bekker "
                   "ESTIMATED (GRC-3 simulant, no native-Mars bevameter exists).",
        role="human habitats + ISRU; MOXIE produced O2 in-situ (Hecht 2022).",
        provenance="Oravec et al. 2020 NASA GRC (GRC-3 simulant: k_c=23.2 kN/m^2, k_phi=606.7 kN/m^3, "
                   "n=1.0); Sullivan 2011 (c 0-2 kPa, phi 30-37); Spohn 2022 InSight (duricrust 5.8 kPa).",
    ),
    "ceres": Body(
        "ceres", "Ceres", 0.284, "gravity-loaded",
        bulk_density=1300.0, cohesion_pa=None, friction_deg=34.5, repose_deg=34.5,
        bekker=None,
        confidence="g MEASURED (Dawn); repose 34.5 MEASURED; near-surface density ESTIMATED (crust "
                   "1200-1360, porosity 53-72%); cohesion + Bekker UNKNOWN (no in-situ data).",
        role="water/brine mining (Dawn brines) + Janhunen 2021 megasatellite-habitat proposal.",
        provenance="Park 2016 / Ermakov 2017 (g, density); Icarus 2024 (repose 34.5+/-2.8); cohesion "
                   "UNKNOWN -> lunar Bekker analog used (flagged); friction = repose proxy.",
    ),
    "bennu": Body(
        "bennu", "Bennu (C-type asteroid)", 4.0e-5, "microgravity",
        bulk_density=1190.0, cohesion_pa=2.0, friction_deg=33.0, repose_deg=33.0,
        bekker=None,
        confidence="g/density/cohesion MEASURED (OSIRIS-REx); friction ESTIMATED (boulder morphology). "
                   "Bekker N/A: microgravity rubble pile, near-zero cohesion, near-fluidized surface.",
        g_note="varies ~3e-5 (equatorial ridge) to ~8.5e-5 m/s^2 (poles); Ryugu analog ~1.1-1.5e-4.",
        role="C-type asteroid water/volatile mining (Bennu first proposed target, Jin 2021).",
        provenance="Scheeres 2019 (g); Lauretta 2019 / Watanabe 2019 (rho 1190); Walsh 2022 (cohesion "
                   "<=2 Pa, near-fluidized); Robin 2024 (phi ~33). Bekker INVALID -> use DEM/granular.",
    ),
    "phobos": Body(
        "phobos", "Phobos (Mars moon)", 0.0057, "microgravity",
        bulk_density=1850.0, cohesion_pa=500.0, friction_deg=38.0, repose_deg=38.0,
        bekker=None,
        confidence="g/density MEASURED; cohesion/friction ESTIMATED (tidal-fracture models, analogs). "
                   "Bekker UNKNOWN (milli-g); JAXA MMX/IDEFIX will return first in-situ data ~2027.",
        g_note="mean ~0.0057 m/s^2 but varies ~210% (shape) / ~450% (with Mars tides) across the body.",
        role="Mars-system staging waypoint + possible ISRU (NASA NTRS 20160006319; JAXA MMX).",
        provenance="Ernst 2023 (rho ~1850); Hurford 2016 (cohesion ~500 Pa surface model, phi ~40); "
                   "Murdoch 2025 IDEFIX (phi 33.5+/-6.1 analog). Bekker UNKNOWN -> lunar analog (flagged).",
    ),
    "earth": Body(
        "earth", "Earth", 9.81, "gravity-loaded",
        bulk_density=1600.0, cohesion_pa=1040.0, friction_deg=28.0, repose_deg=34.0,
        bekker=(990.0, 1528430.0, 1.1),
        confidence="reference/validation body: Wong dry-sand Bekker table (well-established).",
        role="validation/sanity body (terrestrial dry sand).",
        provenance="Wong, Theory of Ground Vehicles (dry sand: k_c=0.99 kN/m^(n+1), k_phi=1528.43 "
                   "kN/m^(n+2), n=1.1, c=1.04 kPa, phi=28).",
    ),
}

DEFAULT_BODY = "moon"


def get_body(name) -> Body:
    """Resolve a body by name (case-insensitive) or pass a Body through unchanged."""
    if isinstance(name, Body):
        return name
    key = str(name).strip().lower()
    if key not in BODIES:
        raise KeyError(f"unknown body {name!r}; known: {sorted(BODIES)}")
    return BODIES[key]


def params_for_body(name) -> TerramechanicsParams:
    """TerramechanicsParams for a body from its SOURCED constants (bodies_sysrev.md).

    Overrides the repo baseline with the body's sourced cohesion / friction / density / Bekker moduli
    where the literature provides them. For bodies whose Bekker moduli are UNKNOWN (Ceres) or the model
    is out of regime (Bennu/Phobos, microgravity), the lunar Bekker moduli stand in as an explicit,
    flagged analog (see Body.bekker_regime / .provenance); the body-sourced cohesion/friction/density
    are still applied. Gravity itself is carried separately into the load (see RoverSimEnv(body=...))."""
    b = get_body(name)
    base = TerramechanicsParams.from_constants()
    kw: dict = {}
    if b.bekker is not None:
        kc, kphi, n = b.bekker
        kw.update(k_c=float(kc), k_phi=float(kphi), n_sinkage=float(n))
    if b.cohesion_pa is not None:
        kw["cohesion"] = float(b.cohesion_pa)
    if b.friction_deg is not None:
        kw["phi_rad"] = math.radians(float(b.friction_deg))
    if b.bulk_density is not None:
        kw["rho_surface"] = float(b.bulk_density)
    return dataclasses.replace(base, **kw)
