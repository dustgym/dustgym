"""Slip-sinkage ladder — the path-dependent "slippy dirt" (Phase 2, 2026-06-01).

Builds on the load-bearing Bekker layer (terramechanics.py) and the per-wheel normal
loads emitted by rover.conform_pose. Models the failure class a closed loop exists for
(spec §6 "two distinct sinkage modes"; the Spirit-rover slip-sinkage runaway):

  traction_budget       H_max = c*A + N*tan(phi)                  Coulomb-Mohr ceiling
  developed_thrust      H(s)  = H_max*(1 - (1-e^-x)/x), x=s*L/K    Janosi-Hanamoto
  slip_for_demand       invert H(s)=demand; demand >= H_max -> ENTRAPMENT
  compaction_resistance R_c grows with sinkage                    Bekker motion resistance
  slip_sinkage_equilibrium  fixed point: slope gravity + R_c -> demand -> slip ->
                            deeper sinkage (terramechanics.slip_sinkage_multiplier) ->
                            more R_c -> ...  converges (stable) or diverges (runaway).

Entrapment occurs (physically) when the demanded thrust exceeds the traction budget —
for this light 30 kg-class rover that happens on slopes past ~the friction angle, where
the cohesion term no longer covers the along-slope gravity. Recovery = back off the
commanded thrust (``demand_frac`` < 1) or take a gentler slope.

MAGNITUDES: SLIP_C1/SLIP_C2 are [UNKNOWN] (constants §5.2); the slip-sinkage law is a
defensible parameterized Tier-2 form validated on QUALITATIVE behaviour (monotone, the
runaway feedback, recovery). Quantitative fit is deferred to the oracle (DEFERRED_FIXES.md).
"""
from __future__ import annotations

import math

from . import constants as K
from . import terramechanics as tm


def traction_budget(normal_load_n: float, *, cohesion: float = K.COHESION,
                    phi_rad: float = K.PHI, contact_area_m2: float) -> float:
    """Max developable thrust [N] = c*A + N*tan(phi) (Coulomb-Mohr ceiling)."""
    return cohesion * contact_area_m2 + normal_load_n * math.tan(phi_rad)


def developed_thrust(slip: float, h_max: float, *, contact_len_m: float,
                     k_shear: float = K.K_SHEAR) -> float:
    """Janosi-Hanamoto thrust [N] developed at slip ratio s. 0 at s<=0; rises
    monotonically toward h_max as s -> inf. H(s) = h_max*(1 - (1-e^-x)/x), x=s*L/K."""
    if slip <= 0.0:
        return 0.0
    x = slip * contact_len_m / k_shear
    return h_max * (1.0 - (1.0 - math.exp(-x)) / x)


def slip_for_demand(demand_n: float, h_max: float, *, contact_len_m: float,
                    k_shear: float = K.K_SHEAR, s_max: float = 0.99) -> tuple[float, bool]:
    """Slip ratio s such that developed_thrust(s) == demand. Returns (s, entrapped).

    demand <= 0 -> (0, False). demand >= h_max -> the thrust ceiling is unreachable ->
    (s_max, True): the wheel spins, cannot develop the demand, and digs in (runaway).
    Otherwise bisection on the monotone developed_thrust curve.
    """
    if demand_n <= 0.0:
        return 0.0, False
    if demand_n >= h_max:
        return s_max, True
    lo, hi = 1e-6, s_max
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if developed_thrust(mid, h_max, contact_len_m=contact_len_m, k_shear=k_shear) < demand_n:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi), False


def compaction_resistance(sinkage_m: float, *, contact_width_m: float,
                          k_c: float = K.K_C, k_phi: float = K.K_PHI,
                          n: float = K.N_SINKAGE) -> float:
    """Bekker compaction (motion) resistance [N] for a wheel sunk by ``sinkage_m``:
    R_c = (b/(n+1)) * (k_c/b + k_phi) * z^(n+1). Grows with sinkage -> feeds the runaway."""
    if sinkage_m <= 0.0:
        return 0.0
    b = contact_width_m
    return (b / (n + 1.0)) * (k_c / b + k_phi) * sinkage_m ** (n + 1.0)


def slip_sinkage_equilibrium(total_weight_n: float, slope_rad: float, *,
                             n_wheels: int = K.N_WHEELS,
                             contact_len_m: float = 0.10, contact_width_m: float = 0.18,
                             params: "tm.TerramechanicsParams | None" = None,
                             demand_frac: float = 1.0,
                             s_entrap: float = 0.95, z_entrap_m: float | None = None,
                             max_iter: int = 200, tol: float = 1e-7) -> dict:
    """Per-wheel fixed-point slip-sinkage solve on a slope.

    Iterates: demand = demand_frac*(along-slope gravity) + compaction_resistance(sinkage)
    -> slip = slip_for_demand(demand, budget) -> sinkage = static * slip_multiplier(slip)
    -> recompute resistance ... until the sinkage converges, or the loop diverges
    (slip >= s_entrap, sinkage >= z_entrap, or demand >= budget) = the Spirit-mode runaway.

    ``demand_frac`` < 1 models the operator backing off the commanded climb thrust
    (the recovery lever). Returns {slip, sinkage_m, resistance_n, demand_n, budget_n,
    entrapped, iters, static_sinkage_m}.
    """
    p = params or tm.TerramechanicsParams.from_constants()
    normal = total_weight_n * math.cos(slope_rad) / n_wheels       # per-wheel normal load
    along = total_weight_n * math.sin(slope_rad) / n_wheels        # per-wheel along-slope gravity
    area = contact_len_m * contact_width_m
    z_entrap = z_entrap_m if z_entrap_m is not None else 2.0 * 0.18  # ~ wheel diameter
    h_max = traction_budget(normal, cohesion=p.cohesion, phi_rad=p.phi_rad,
                            contact_area_m2=area)
    z_static = tm.wheel_static_sinkage(normal, params=p, contact_len_m=contact_len_m,
                                       contact_width_m=contact_width_m)
    sink = z_static
    slip = 0.0
    resistance = 0.0
    demand = demand_frac * along
    entrapped = False
    iters = 0
    for i in range(max_iter):
        iters = i + 1
        resistance = compaction_resistance(sink, contact_width_m=contact_width_m,
                                           k_c=p.k_c, k_phi=p.k_phi, n=p.n_sinkage)
        demand = demand_frac * along + resistance
        slip, ent = slip_for_demand(demand, h_max, contact_len_m=contact_len_m,
                                    k_shear=p.k_shear)
        if ent:
            entrapped = True
            break
        new_sink = z_static * tm.slip_sinkage_multiplier(slip, c1=p.slip_c1, c2=p.slip_c2)
        if new_sink >= z_entrap or slip >= s_entrap:
            entrapped = True
            sink = new_sink
            break
        if abs(new_sink - sink) < tol:
            sink = new_sink
            break
        sink = new_sink
    return {
        "slip": slip,
        "sinkage_m": sink,
        "static_sinkage_m": z_static,
        "resistance_n": resistance,
        "demand_n": demand,
        "budget_n": h_max,
        "entrapped": entrapped,
        "iters": iters,
    }
