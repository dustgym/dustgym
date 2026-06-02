"""Load-bearing Bekker pressure-sinkage for the Tier-2 rover (added 2026-06-01).

Makes the previously-DECORATIVE Bekker moduli in constants.py (K_PHI, K_C,
N_SINKAGE) load-bearing: a real pressure-sinkage solve that replaces the
hardcoded ``compaction=0.12`` density bump in rover.wheel_pass / four_wheel_pass
(wired in a follow-on step behind an opt-in flag — this module is the physics).

  Bekker:  p = (k_c / b + k_phi * s(rho)) * z^n
    p      = contact pressure [Pa] = normal load / contact area
    b      = smaller contact-patch dimension [m] (Bekker plate width)
    s(rho) = density-stiffening factor (>=1): denser soil bears better, so a
             repeated pass over compacted soil sinks LESS — the spec §6 "paving"
             effect, here EMERGENT from physics, not a hardcoded constant.
  inverted:  z = ( p / (k_c/b + k_phi*s) ) ** (1/n)

Sinkage maps to a MASS-CONSERVING density increase (sinkage_to_density_factor):
the loose column is compacted (density up, areal mass untouched), so the surface
drops by z and column mass is conserved exactly (spec §10 invariant 1).

CONFIG (TerramechanicsParams): the solver functions take the moduli as keyword
args, so they are config-driven. ``TerramechanicsParams`` is a JSON-serializable
view whose DEFAULTS come from constants.py (the provenance-tagged source of
truth). Override fields for domain randomization / per-scenario experiments, and
to_json()/from_json() to persist a config alongside a scene (the repo's
on-disk-seam convention). The honesty tags ([CALIB]/[UNKNOWN]) stay in
constants.py; a serialized config is a concrete CHOICE within those envelopes.

PARAMETER NOTE / honest gap: constants.py uses the spec §5.2 Apollo-era moduli
(K_PHI=820000, K_C=1400). The committed Chrono SCM oracle (scripts/chrono_scm_
rover.py:112) used a JSC-1A analogue (k_phi=0.2e6, k_c=0) — a ~4x lower k_phi, so
SCM predicts MORE sinkage. Which set to trust is exactly what the controlled
load-sweep oracle (Phase 0.3, deferred to a PyChrono host) must reconcile; both
are reachable via TerramechanicsParams.from_constants() / .scm_oracle(). The
1g->1/6g Lyasko-2010 reduced-gravity correction is NOT yet applied (needs the
sweep + the paper) and is the named next calibration step.
"""
from __future__ import annotations

import dataclasses
import json
from dataclasses import dataclass

import numpy as np

from . import constants as K


# ---------------------------------------------------------------------------
# Config: JSON-serializable params, defaulting from the tagged constants.py.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class TerramechanicsParams:
    """One scenario / RL-episode terramechanics config (JSON round-trippable).

    Defaults are the canonical lunar values from constants.py. Override fields
    for domain randomization or experiments; serialize with to_json/from_json.
    """

    k_c: float = K.K_C                       # Bekker cohesive modulus [Pa/m^(n-1)]
    k_phi: float = K.K_PHI                   # Bekker frictional modulus [Pa/m^n]
    n_sinkage: float = K.N_SINKAGE           # sinkage exponent
    cohesion: float = K.COHESION             # [Pa]
    phi_rad: float = float(K.PHI)            # internal friction [rad]
    k_shear: float = K.K_SHEAR               # Janosi-Hanamoto shear modulus [m]
    slip_c1: float = K.SLIP_C1               # slip-sinkage c1 (Phase 2)
    slip_c2: float = K.SLIP_C2               # slip-sinkage c2 (Phase 2)
    rho_surface: float = K.RHO_SURFACE       # loose surface density [kg/m^3]
    rho_deep: float = K.RHO_DEEP             # compacted ceiling [kg/m^3]
    rover_mass_dry_kg: float = K.ROVER_MASS_DRY_KG
    contact_len_m: float = 0.10              # nominal wheel contact patch length [m]
    contact_width_m: float = 0.18            # wheel contact width [m] (rover.py)

    @classmethod
    def from_constants(cls) -> "TerramechanicsParams":
        """The canonical lunar values from constants.py (all defaults)."""
        return cls()

    @classmethod
    def scm_oracle(cls) -> "TerramechanicsParams":
        """The committed Chrono SCM run's soil set (chrono_scm_rover.py:112) — a
        JSC-1A analogue (k_phi=0.2e6, k_c=0, n=1). For oracle cross-checks."""
        return cls(k_c=0.0, k_phi=0.2e6, n_sinkage=1.0)

    @classmethod
    def lunar(cls) -> "TerramechanicsParams":
        """from_constants() with the Lyasko 1g->1/6g reduced-gravity correction
        applied (sourced direction, [CALIB] magnitude). Use for lunar runs; the
        bare from_constants() stays Earth/Apollo-fit (spec §5.2 "not applied")."""
        return lyasko_reduce(cls.from_constants())

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "TerramechanicsParams":
        names = {f.name for f in dataclasses.fields(cls)}
        unknown = set(d) - names
        if unknown:
            raise ValueError(f"unknown terramechanics param(s): {sorted(unknown)}")
        return cls(**{k: v for k, v in d.items() if k in names})

    def to_json(self, path: str | None = None, *, indent: int = 2) -> str:
        s = json.dumps(self.to_dict(), indent=indent, sort_keys=True)
        if path is not None:
            with open(path, "w") as fh:
                fh.write(s + "\n")
        return s

    @classmethod
    def from_json(cls, path: str) -> "TerramechanicsParams":
        with open(path) as fh:
            return cls.from_dict(json.load(fh))


#: Module default (canonical constants.py values), used when no params are passed.
_DEFAULT_PARAMS = TerramechanicsParams.from_constants()


# ---------------------------------------------------------------------------
# Weight-on-wheels (sourced IPEx 30 kg-class mass; constants.py).
# ---------------------------------------------------------------------------

def static_wheel_load_n(payload_kg: float = 0.0, *,
                        rover_mass_dry_kg: float = K.ROVER_MASS_DRY_KG,
                        n_wheels: int = K.N_WHEELS) -> float:
    """Static per-wheel normal load [N] at lunar g for a given drum payload.

    (rover_mass_dry_kg + payload_kg) * g / n_wheels. ~12.15 N/wheel dry,
    ~24.3 N/wheel at the full 30 kg payload (ascend24 "30 kg-class"). Equal-split
    baseline; CG-based fore/aft load transfer is a refinement (CG height is not
    in the public TRL-5 overview, so not modelled).
    """
    payload_kg = max(0.0, float(payload_kg))
    return (rover_mass_dry_kg + payload_kg) * K.g / n_wheels


# ---------------------------------------------------------------------------
# Bekker pressure-sinkage.
# ---------------------------------------------------------------------------

def density_stiffening(density: float, rho_surface: float = K.RHO_SURFACE) -> float:
    """Bearing-stiffening factor s(rho) >= 1 (denser soil bears better).

    [CALIB] linear law s = rho / rho_surface (s=1 at the loose surface; ~1.48 at
    RHO_DEEP). Drives multi-pass paving — a repeated pass on compacted soil sinks
    less. Refit against the Chrono oracle sweep.
    """
    return max(1.0, float(density) / float(rho_surface))


def bekker_pressure_sinkage(pressure_pa: float, *, b_m: float,
                            k_c: float = K.K_C, k_phi: float = K.K_PHI,
                            n: float = K.N_SINKAGE, stiffening: float = 1.0) -> float:
    """Bekker static pressure-sinkage z [m] for contact pressure ``pressure_pa``.

    z = ( p / (k_c/b + k_phi*stiffening) ) ** (1/n). Reads the (previously
    decorative) constants.py moduli by default; override for the SCM oracle set.
    Returns 0 for non-positive pressure.
    """
    if pressure_pa <= 0.0:
        return 0.0
    if b_m <= 0.0:
        raise ValueError(f"b_m (contact width) must be > 0, got {b_m}")
    k = k_c / b_m + k_phi * max(1.0, stiffening)
    if k <= 0.0:
        raise ValueError("bearing modulus (k_c/b + k_phi*s) must be > 0")
    return (pressure_pa / k) ** (1.0 / n)


def wheel_static_sinkage(load_n: float, *, params: TerramechanicsParams | None = None,
                         density: float | None = None,
                         contact_len_m: float | None = None,
                         contact_width_m: float | None = None,
                         k_c: float | None = None, k_phi: float | None = None,
                         n: float | None = None) -> float:
    """Static bearing sinkage z [m] of one wheel under normal load ``load_n``.

    Pressure = load / (contact_len * contact_width); Bekker plate width b =
    min(contact_len, contact_width). ``density`` (current cell density) drives the
    paving stiffening; None -> loose surface (s=1). Params come from ``params``
    (or constants.py defaults); explicit kwargs override individual fields.
    Nominal rectangular contact patch (10-20 cm anchor, spec §4); a sinkage-
    coupled contact length is a refinement (avoids a fixed-point solve here).
    """
    if load_n <= 0.0:
        return 0.0
    p = params or _DEFAULT_PARAMS
    contact_len_m = p.contact_len_m if contact_len_m is None else contact_len_m
    contact_width_m = p.contact_width_m if contact_width_m is None else contact_width_m
    k_c = p.k_c if k_c is None else k_c
    k_phi = p.k_phi if k_phi is None else k_phi
    n = p.n_sinkage if n is None else n

    area = max(1e-9, contact_len_m * contact_width_m)
    pressure = load_n / area
    b = min(contact_len_m, contact_width_m)
    s = density_stiffening(density, p.rho_surface) if density is not None else 1.0
    return bekker_pressure_sinkage(pressure, b_m=b, k_c=k_c, k_phi=k_phi, n=n, stiffening=s)


def sinkage_to_density_factor(z_m: float, thickness_m: float) -> float:
    """Density-increase factor f so density *= (1+f) thins the column by z_m.

    MASS-CONSERVING: height = mass/density, so to drop thickness t -> t - z at
    fixed areal mass, density must rise by t/(t-z), i.e. f = z/(t-z). z is clamped
    just below the column thickness (cannot compact past zero thickness). The
    caller still caps the resulting density at RHO_DEEP (the physical ceiling).
    """
    if z_m <= 0.0 or thickness_m <= 0.0:
        return 0.0
    z = min(z_m, 0.999 * thickness_m)
    return z / (thickness_m - z)


# ---------------------------------------------------------------------------
# Vectorized field form — the seam rover.four_wheel_pass(physical=True) calls.
# ---------------------------------------------------------------------------

def physical_compaction_field(density, mass_areal, load_n: float, *,
                              params: TerramechanicsParams | None = None,
                              contact_len_m: float | None = None,
                              contact_width_m: float | None = None):
    """Per-cell density-increase factor field f from load-driven Bekker sinkage.

    Apply as ``density *= (1 + f)`` then cap at RHO_DEEP — MASS-CONSERVING (density
    -only edit; height re-derives). Vectorized numpy mirror of wheel_static_sinkage
    + sinkage_to_density_factor: per cell, stiffening s = density/rho_surface (denser
    soil bears better -> paving), z = (p/(k_c/b + k_phi*s))**(1/n), f = z/(t-z).
    ``density``/``mass_areal`` are the masked cells (kg/m^3, kg/m^2). Returns f (same
    shape). load_n <= 0 -> zeros.
    """
    p = params or _DEFAULT_PARAMS
    cl = p.contact_len_m if contact_len_m is None else contact_len_m
    cw = p.contact_width_m if contact_width_m is None else contact_width_m
    density = np.asarray(density, dtype=np.float64)
    if load_n <= 0.0 or density.size == 0:
        return np.zeros_like(density)
    area = max(1e-9, cl * cw)
    b = min(cl, cw)
    pressure = load_n / area
    s = np.maximum(1.0, density / p.rho_surface)          # per-cell stiffening (paving)
    k = p.k_c / b + p.k_phi * s
    z = (pressure / k) ** (1.0 / p.n_sinkage)             # per-cell sinkage [m]
    thickness = np.asarray(mass_areal, dtype=np.float64) / density
    z = np.minimum(z, 0.999 * thickness)                  # clamp below thickness
    return z / (thickness - z)


# ---------------------------------------------------------------------------
# Lyasko-2010 reduced-gravity correction (1g -> 1/6 g).
# ---------------------------------------------------------------------------

def lyasko_reduce(params: TerramechanicsParams, *, g: float = K.g, g_earth: float = 9.81,
                  kphi_frac: float = 0.30, c_frac: float = 0.30,
                  n_frac: float = 0.0) -> TerramechanicsParams:
    """Apply the Lyasko-2010 reduced-gravity correction -> a new TerramechanicsParams.

    DIRECTION is sourced (Lyasko 2010; spec §5.2; low-g bevameter/DEM studies —
    JANSS 38(4):237; J.Terramech. low-gravity-device & 2D-DEM studies): lowering
    gravity DECREASES the frictional modulus k_phi and cohesion c (and, per the
    literature, the exponent n), while k_c and phi show little change; the NET
    result is that sinkage INCREASES under the same load. Each reduced parameter =
    earth * (1 - frac * deficit), deficit = clip(1 - g/g_earth, 0, 1). At
    g = g_earth this is the identity (no reduction).

    MAGNITUDE is [CALIB] — to be fit against the Chrono::GPU DEM load-sweep oracle
    on euclid (Phase 0.3). HONEST MODELLING NOTE on n: the literature reports n
    also decreasing, but in the Bekker form p=(k_c/b + k_phi)z^n the n-exponent is
    dimensionally tied to k_phi's units, and naively lowering n at sub-metre sinkage
    (p/k < 1) DECREASES z — the opposite of the sourced net truth. So the net
    sinkage increase is carried by the (dimensionally clean) k_phi reduction, and
    ``n_frac`` defaults to 0.0; re-parameterising n consistently is deferred to the
    oracle fit, where k_phi is re-fit in the new units. c_frac feeds Phase-2 shear.
    """
    deficit = max(0.0, min(1.0, 1.0 - g / g_earth))

    def _scale(frac: float) -> float:
        return 1.0 - frac * deficit

    return dataclasses.replace(
        params,
        k_phi=params.k_phi * _scale(kphi_frac),
        cohesion=params.cohesion * _scale(c_frac),
        n_sinkage=params.n_sinkage * _scale(n_frac),   # n_frac default 0 -> n unchanged
    )   # k_c and phi_rad deliberately unchanged (Lyasko: little change)
