"""Closed-loop drive — ROS/policy drives the rover via cmd_vel (Phase 3, 2026-06-01).

Closes the loop the spec is built for: instead of replaying a precomputed path
(drive_spiral.py, left intact), a controller supplies a TWIST (forward speed +
yaw rate) each step. The producer integrates it (rover.step_pose), reads the local
slope (rover.conform_pose), computes SLIP from the terrain demand (slip.py), reduces
the achieved motion by that slip, and carves the slip-deepened ruts
(rover.four_wheel_pass(physical=True)). So commanded and achieved motion DIVERGE
under slip, and that divergence is path-dependent — the whole point of a closed loop.

Two entry points:
  * closed_loop_drive(...)  -- run a sequence of twists (deterministic; testable).
  * poll_cmd_vel(path)      -- the reverse seam: read the latest {v, omega} twist
                               from a JSON file (file-mediated, like the INTERFACE
                               state-field seam), so a ROS node / Nav2 can drive it.

Slip magnitudes are [UNKNOWN]/[CALIB] (DEFERRED_FIXES.md); the loop's STRUCTURE
(commanded-vs-achieved divergence, stall-on-slope) is what is validated here.
"""
from __future__ import annotations

import json
import os

from . import constants as K
from . import rover
from . import slip as slipmod
from . import terramechanics as tm
from .column_state import ColumnState


def poll_cmd_vel(path: str) -> tuple[float, float]:
    """Read the latest twist (v [m/s], omega [rad/s]) from a JSON file. The reverse
    command seam — a ROS node / policy writes ``{"v": .., "omega": ..}`` to ``path``;
    the producer polls it. Missing / unreadable / empty -> (0.0, 0.0) (safe stop).
    """
    if not os.path.exists(path):
        return 0.0, 0.0
    try:
        with open(path) as fh:
            d = json.load(fh)
        return float(d.get("v", 0.0)), float(d.get("omega", 0.0))
    except (ValueError, OSError):
        return 0.0, 0.0


def drive_step(cs: ColumnState, rc: tuple[float, float], yaw: float,
               v_cmd: float, omega_cmd: float, *, dt: float = 0.1,
               params: "tm.TerramechanicsParams | None" = None,
               payload_kg: float = 0.0, wheel_width_m: float = 0.18,
               contact_len_m: float = 0.10, g: float = K.g,
               clasts: "list[dict] | None" = None) -> tuple[tuple[float, float], float, dict]:
    """One closed-loop step: command twist in, (new_rc, new_yaw, telemetry) out.

    read local forward slope (conform_pose pitch, incl. clast ride-over if ``clasts``
    given) -> slip-sinkage equilibrium (slip.py) -> achieved v = (1-slip)*commanded v
    -> integrate pose (step_pose) -> carve slip-deepened ruts at the achieved pose
    (four_wheel_pass(physical=True)). MASS-CONSERVING. ``g`` sets the body gravity (default
    lunar; see bodies.py) -> weight = m*g drives the load. Telemetry dict: rc, yaw, v_cmd,
    omega_cmd, v_achieved, slip, entrapped, slope_rad, sinkage_m.
    """
    p = params or tm.TerramechanicsParams.from_constants()
    weight_n = (K.ROVER_MASS_DRY_KG + max(0.0, payload_kg)) * float(g)
    h = cs.derive_height()
    cf = rover.conform_pose(h, rc, yaw, cell_m=cs.cell_m, payload_kg=payload_kg, clasts=clasts, g=g)
    slope_rad = cf["pitch_rad"]                           # forward grade the wheels fight
    eq = slipmod.slip_sinkage_equilibrium(weight_n, slope_rad, params=p,
                                          contact_len_m=contact_len_m,
                                          contact_width_m=wheel_width_m)
    s = eq["slip"]
    v_ach = (1.0 - s) * v_cmd                             # slip robs forward progress
    new_rc, new_yaw = rover.step_pose(rc, yaw, v_ach, omega_cmd, dt, cell_m=cs.cell_m)
    rover.four_wheel_pass(cs, [(new_rc, new_yaw)], wheel_width_m=wheel_width_m,
                          physical=True, loads=cf["normal_loads"], params=p,
                          contact_len_m=contact_len_m, slip=s)
    telem = {
        "rc": [new_rc[0], new_rc[1]], "yaw": new_yaw,
        "v_cmd": float(v_cmd), "omega_cmd": float(omega_cmd),
        "v_achieved": float(v_ach), "slip": float(s), "entrapped": bool(eq["entrapped"]),
        "slope_rad": float(slope_rad), "sinkage_m": float(eq["sinkage_m"]),
    }
    return new_rc, new_yaw, telem


def closed_loop_drive(cs: ColumnState, start_rc: tuple[float, float], start_yaw: float,
                      twists, *, dt: float = 0.1,
                      params: "tm.TerramechanicsParams | None" = None,
                      payload_kg: float = 0.0, wheel_width_m: float = 0.18,
                      contact_len_m: float = 0.10, g: float = K.g,
                      clasts: "list[dict] | None" = None) -> dict:
    """Drive ``cs`` through a sequence of ``twists`` ((v_mps, omega_radps) pairs),
    one drive_step each. Deterministic. ``clasts`` (optional) enables boulder
    ride-over in the per-step conform. ``g`` sets body gravity (default lunar; bodies.py).
    Returns {steps, commanded_dist_m, achieved_dist_m, final_rc, final_yaw, any_entrapped}.
    """
    p = params or tm.TerramechanicsParams.from_constants()
    rc = (float(start_rc[0]), float(start_rc[1]))
    yaw = float(start_yaw)
    steps: list[dict] = []
    commanded_dist = achieved_dist = 0.0
    any_entrapped = False
    for i, (v_cmd, omega_cmd) in enumerate(twists):
        rc, yaw, telem = drive_step(cs, rc, yaw, v_cmd, omega_cmd, dt=dt, params=p,
                                    payload_kg=payload_kg, wheel_width_m=wheel_width_m,
                                    contact_len_m=contact_len_m, g=g, clasts=clasts)
        telem["frame"] = i
        commanded_dist += abs(v_cmd) * dt
        achieved_dist += abs(telem["v_achieved"]) * dt
        any_entrapped = any_entrapped or telem["entrapped"]
        steps.append(telem)
    return {
        "steps": steps,
        "commanded_dist_m": commanded_dist,
        "achieved_dist_m": achieved_dist,
        "final_rc": [rc[0], rc[1]],
        "final_yaw": yaw,
        "any_entrapped": any_entrapped,
    }
