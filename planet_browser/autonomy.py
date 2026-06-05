"""Closed-loop autonomy (P12) — the recursive belief-state estimator (the AutoNav "OD" analog).

DS1 AutoNav's core was an orbit-determination FILTER producing state + covariance, which the maneuver
planner then replanned against. Our planner has been deterministic (assumed-perfect state); this module
is the missing ESTIMATE half: a belief state where every quantity carries a 1-sigma uncertainty.

  predict(...)   — process / dead-reckoning step: move + spend; uncertainty GROWS (odometry drift,
                   energy-model error from slip/terrain). The AutoNav "time update".
  update_*(...)  — measurement fusion from a sensor; uncertainty SHRINKS via a scalar Kalman update.
                   The AutoNav "measurement update".

Measurements come from the real drum-sensor model (rassor_mass_model) and the conserved authority, never
fabricated. The closed loop (autonomy/controller, next increment) runs plan -> execute leg -> sense ->
estimate -> replan over the conserved authority first (AutoNav's self-simulation), then real telemetry.
"""

from __future__ import annotations

import dataclasses
import math

from . import mission_planner as MP
from .mission_planner import BATTERY_J


@dataclasses.dataclass
class Belief:
    """Estimated mission state with 1-sigma uncertainty on each quantity (the AutoNav OD file analog)."""
    x: float
    y: float
    pos_sigma_m: float
    energy_J: float
    energy_sigma_J: float
    drum_kg: float
    drum_sigma_kg: float
    tasks_done: int
    tasks_total: int
    t_s: float = 0.0

    def soc_frac(self) -> float:
        """Battery state-of-charge estimate (fraction of the pack)."""
        return self.energy_J / BATTERY_J

    def to_dict(self) -> dict:
        return {**dataclasses.asdict(self), "soc_frac": self.soc_frac()}


def initial_belief(mission, tasks_total, *, pos_sigma_m=0.5):
    """A fresh belief at mission start: parked at the charger, full pack, empty drum — all well known."""
    cx, cy = mission.charger
    return Belief(x=float(cx), y=float(cy), pos_sigma_m=float(pos_sigma_m),
                  energy_J=float(BATTERY_J), energy_sigma_J=0.0,
                  drum_kg=0.0, drum_sigma_kg=0.0, tasks_done=0, tasks_total=int(tasks_total))


def _kf_update(mu, var, z, r):
    """Scalar Kalman / Bayesian fusion of a prior (mu, var) with a measurement (z, variance r).
    Returns (mu', var') with var' <= min(var, r). var=inf -> take the measurement; r=0 -> exact measurement."""
    if not math.isfinite(var):
        return z, r
    if r <= 0:
        return z, 0.0
    if var <= 0:
        return mu, 0.0
    k = var / (var + r)                                   # Kalman gain
    return mu + k * (z - mu), (1.0 - k) * var


def predict(b, *, moved_to=None, drive_m=0.0, odom_drift_frac=0.05,
            energy_spent_J=0.0, energy_model_sigma_frac=0.12,
            drum_delta_kg=0.0, drum_process_sigma_kg=0.0, dt_s=0.0):
    """Process step. `moved_to` sets the believed pose (the commanded destination); pose uncertainty grows
    by `odom_drift_frac * drive_m`. Energy drops by `energy_spent_J` with its uncertainty growing by
    `energy_model_sigma_frac` of the spend (the slip/terrain unknown -- exactly the AutoNav lesson that
    model error must be carried, not assumed away). The drum changes by `drum_delta_kg`."""
    x, y = (b.x, b.y) if moved_to is None else (float(moved_to[0]), float(moved_to[1]))
    pos_var = b.pos_sigma_m ** 2 + (odom_drift_frac * drive_m) ** 2
    e_var = b.energy_sigma_J ** 2 + (energy_model_sigma_frac * energy_spent_J) ** 2
    d_var = b.drum_sigma_kg ** 2 + drum_process_sigma_kg ** 2
    return dataclasses.replace(b, x=x, y=y, pos_sigma_m=math.sqrt(pos_var),
                               energy_J=b.energy_J - energy_spent_J, energy_sigma_J=math.sqrt(e_var),
                               drum_kg=b.drum_kg + drum_delta_kg, drum_sigma_kg=math.sqrt(d_var),
                               t_s=b.t_s + dt_s)


def update_drum(b, reading_kg, reading_sigma_kg):
    """Fuse a drum-mass measurement (motor-current inference, rassor_mass_model) into the belief."""
    mu, var = _kf_update(b.drum_kg, b.drum_sigma_kg ** 2, float(reading_kg), float(reading_sigma_kg) ** 2)
    return dataclasses.replace(b, drum_kg=mu, drum_sigma_kg=math.sqrt(max(0.0, var)))


def update_pose(b, fix_xy, fix_sigma_m):
    """Fuse a pose fix (landmark / map match) into the position belief."""
    vx, varx = _kf_update(b.x, b.pos_sigma_m ** 2, float(fix_xy[0]), float(fix_sigma_m) ** 2)
    vy, vary = _kf_update(b.y, b.pos_sigma_m ** 2, float(fix_xy[1]), float(fix_sigma_m) ** 2)
    return dataclasses.replace(b, x=vx, y=vy, pos_sigma_m=math.sqrt(max(0.0, max(varx, vary))))


def update_energy(b, reading_J, reading_sigma_J):
    """Fuse a battery state-of-charge measurement (coulomb count / voltage) into the energy belief."""
    mu, var = _kf_update(b.energy_J, b.energy_sigma_J ** 2, float(reading_J), float(reading_sigma_J) ** 2)
    return dataclasses.replace(b, energy_J=mu, energy_sigma_J=math.sqrt(max(0.0, var)))


# ---- EXECUTOR + CONTROLLER: the closed loop (plan -> execute -> sense -> estimate -> replan) -----
def nominal_leg_energy_J(pose, leg):
    """The planner's MODEL estimate for a leg: flat 135 J/m drive (pose->site) + the leg's dig/haul/lift.
    This is what the plan BUDGETED; `execute_leg` returns the slip-adjusted truth, and the gap is the model
    error the estimator carries and the controller replans against (the AutoNav model-vs-truth dynamic)."""
    drive = MP._d(pose, leg["site"])
    haul_e = leg.get("haul_e", leg.get("haul_m", 0.0) * MP.DRIVE_J_PER_M)   # #1 slip-aware haul (the plan's)
    return (drive * MP.DRIVE_J_PER_M + leg.get("dig_e", 0.0) + leg.get("sinter_e", 0.0)
            + haul_e + leg.get("lift_e", 0.0))


def execute_leg(belief, leg, *, dem=None, dem_origin=(0.0, 0.0), g=None, body="moon"):
    """Step the rover from its believed pose through one leg, returning the TRUE telemetry it experiences:
    the inter-leg drive costs `135/(1-slip) + rover_mass*g*Δh` (slope→slip from the real DEM + exact gravity
    climb), plus the leg's dig/haul/lift. This is the physical truth that diverges from the flat nominal plan."""
    g = MP.body_gravity(body) if g is None else g
    pose = (belief.x, belief.y)
    site = leg["site"]
    drive_m = MP._d(pose, site)
    dh = MP.haul_elevation_gain_m(dem, dem_origin, pose, site) if dem is not None else 0.0
    slope_deg = math.degrees(math.atan2(abs(dh), drive_m)) if drive_m > 1e-9 else 0.0
    slip = MP.slip_alpha_to_slip(slope_deg)
    true_drive_J = drive_m * MP.DRIVE_J_PER_M / (1.0 - slip) + MP.ROVER_MASS_KG * g * max(0.0, dh)
    haul_e = leg.get("haul_e", leg.get("haul_m", 0.0) * MP.DRIVE_J_PER_M)   # #1 slip-aware haul (the plan's)
    true_J = (true_drive_J + leg.get("dig_e", 0.0) + leg.get("sinter_e", 0.0)
              + haul_e + leg.get("lift_e", 0.0))
    return {"drive_m": drive_m, "true_energy_J": true_J, "new_pose": site,
            "slope_deg": slope_deg, "slip": slip, "drum_through_kg": leg.get("mass", 0.0)}


def run_closed_loop(mission, *, dem=None, dem_origin=(0.0, 0.0), algorithm="auto", objective="time",
                    max_traverse_slope_deg=25.0, perception_sigma_m=None, dig_sigma_gate_m=0.20):
    """Run the AutoNav-style loop over the conserved-model: plan -> execute leg (true telemetry) -> estimate
    (predict + measure) -> replan/recharge against the ESTIMATE. Drains the battery by the slip-adjusted TRUE
    energy with reserve-aware recharges, so on real terrain it recharges/replans more than the flat plan
    expected. Runs in simulation first (AutoNav's self-simulation); real telemetry swaps in later.

    Simplification (the precise energy sim is mission_planner._simulate): recharges return to the charger and
    set the pack full; the return-to-site drive is not re-accounted here. The point of this layer is the
    closed-loop estimate/replan dynamic, not a second energy simulator."""
    g = MP.body_gravity(mission.body)
    reserve = MP.RESERVE_FRAC * BATTERY_J
    trips, _flows, _surplus, _meta = MP._build_trips(mission, dem, dem_origin, max_traverse_slope_deg)
    prec = MP.trip_precedence(trips, mission)
    order = MP.optimize_sequence(trips, mission, algorithm=algorithm, objective=objective, precedence=prec)
    belief = initial_belief(mission, len(trips))
    remaining = list(order)
    recharges = replans = 0
    perception_fixes = observe_more = 0
    legs = []

    def _recharge(b):
        d_back = MP._d((b.x, b.y), mission.charger)
        b = predict(b, moved_to=mission.charger, drive_m=d_back, energy_spent_J=0.0)
        b = dataclasses.replace(b, energy_J=BATTERY_J, energy_sigma_J=0.0)
        if perception_sigma_m is not None:             # docking at the charger is a known-landmark pose fix
            b = update_pose(b, mission.charger, perception_sigma_m)
        return b

    while remaining:
        i = remaining.pop(0)
        leg = trips[i]
        # PERCEPTION-IN-THE-LOOP (Uncertainty-layer dig-ready gate): before committing to a dig, if the
        # pose estimate is too uncertain, dwell and take more observations until it is confident enough.
        if perception_sigma_m is not None and leg.get("dig_e", 0.0) > 0.0:
            while belief.pos_sigma_m > dig_sigma_gate_m:
                belief = update_pose(belief, (belief.x, belief.y), perception_sigma_m)
                observe_more += 1
        nominal_J = nominal_leg_energy_J((belief.x, belief.y), leg)
        telem = execute_leg(belief, leg, dem=dem, dem_origin=dem_origin, g=g, body=mission.body)
        # ESTIMATE: move (pose uncertainty grows with distance), and grow the energy uncertainty by the
        # leg's model error (a priori the plan can't see the slip truth -> carry it as 1-sigma).
        belief = predict(belief, moved_to=telem["new_pose"], drive_m=telem["drive_m"], energy_spent_J=0.0)
        # PERCEPTION MEASUREMENT: fuse a map/landmark pose fix (the map channel / AprilTag SLAM egress),
        # bounding the dead-reckoning drift. Without it pose sigma only grows; with it the loop is corrected.
        if perception_sigma_m is not None:
            belief = update_pose(belief, (telem["new_pose"][0], telem["new_pose"][1]), perception_sigma_m)
            perception_fixes += 1
        e_sig = math.sqrt(belief.energy_sigma_J ** 2 + (0.12 * nominal_J) ** 2)
        belief = dataclasses.replace(belief, energy_sigma_J=e_sig)
        # drain the TRUE energy with reserve-aware recharges (closed-loop battery management on the estimate)
        left = telem["true_energy_J"]
        while left > 1e-6:
            usable = belief.energy_J - reserve
            if usable <= 1e-3:
                belief = _recharge(belief); recharges += 1
                if remaining:                                  # REPLAN remaining order from the charger
                    sub = [trips[k] for k in remaining]
                    so = MP.optimize_sequence(sub, mission, algorithm="nearest", objective=objective)
                    remaining = [remaining[k] for k in so]
                    replans += 1
                usable = belief.energy_J - reserve
            chunk = min(left, usable)
            belief = dataclasses.replace(belief, energy_J=belief.energy_J - chunk)
            left -= chunk
        belief = dataclasses.replace(belief, tasks_done=belief.tasks_done + 1)
        legs.append({"leg": leg["label"], "nominal_J": nominal_J, "true_J": telem["true_energy_J"],
                     "dig_e": float(leg.get("dig_e", 0.0)),     # dig doesn't slip; only the drive portion inflates
                     "soc": belief.soc_frac(), "slope_deg": telem["slope_deg"], "slip": telem["slip"],
                     "energy_sigma_J": e_sig})
    return {"belief": belief, "completed": belief.tasks_done == len(trips), "n_trips": len(trips),
            "recharges": recharges, "replans": replans, "legs": legs,
            "perception_fixes": perception_fixes, "observe_more": observe_more}
