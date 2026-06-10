"""Flight side: the onboard rover executive (no ROS).

Loads a crop of the real LOLA Haworth DEM into a conserved ``ColumnState``, then runs an onboard
pure-pursuit waypoint follower that drives the terramechanics authority one tick at a time via
``terrain_authority.drive.drive_step``. The executive *commands* (a twist per tick); only the authority
mutates terrain, so mass is conserved by construction (CONTRIBUTING.md). Onboard safing stops a leg on
slip-runaway entrapment or when the battery hits its reserve — the move-and-wait reflexes the
light-time budget demands.

Body gravity + soil thread through ``bodies`` (Moon first; Earth is the same call path with g=9.81).
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

# Run-from-source fallback (same pattern as planet_browser/mission_planner): when dustgym is not
# pip-installed, make the monorepo root importable so ``terrain_authority`` resolves.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from terrain_authority import bodies, constants as K, drive, io_fields, ipex_specs
from terrain_authority.column_state import ColumnState

import messages


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class CropContext:
    """A loaded window of the Haworth DEM + where it sits in the full grid (for world framing / plots)."""

    cell_m: float
    r0: int
    c0: int
    heightmap: np.ndarray          # (win_h, win_w) real LOLA surface [m]
    fields: dict                   # mass_areal/density/state_label/disturbance/datum crops (float64/uint8)
    world_x0: float
    world_y0: float


def load_crop(scene_dir: str, r0: int, c0: int, win_h: int, win_w: int) -> CropContext:
    """Load a (win_h x win_w) crop of a committed DEM scene as conserved fields.

    Mirrors scenes.build_from_dem's datum derivation locally (datum = heightmap - mass/density) so
    ``derive_height()`` reproduces the real DEM surface exactly, without the full-grid corridor build.
    """
    base, meta = io_fields.load_scene(scene_dir)
    cell_m = float(meta["grid"]["cell_m"])
    sl = (slice(r0, r0 + win_h), slice(c0, c0 + win_w))
    hm = np.ascontiguousarray(base["heightmap"][sl], dtype=np.float64)
    mass = np.ascontiguousarray(base["mass_areal"][sl], dtype=np.float64)
    dens = np.ascontiguousarray(base["density"][sl], dtype=np.float64)
    st = np.ascontiguousarray(base["state_label"][sl], dtype=np.uint8)
    dist = np.ascontiguousarray(base["disturbance"][sl], dtype=np.float64)
    datum = hm - mass / dens
    wb = meta.get("world_bounds_m", {})
    world_x0 = float(wb.get("x0", 0.0)) + c0 * cell_m
    world_y0 = float(wb.get("y0", 0.0)) - r0 * cell_m   # row increases -> world Y decreases
    fields = {"mass_areal": mass, "density": dens, "state_label": st, "disturbance": dist, "datum": datum}
    return CropContext(cell_m=cell_m, r0=r0, c0=c0, heightmap=hm, fields=fields,
                       world_x0=world_x0, world_y0=world_y0)


@dataclass
class FlightModel:
    """Onboard executive over a Haworth crop. Drives waypoints via pure pursuit + drive_step."""

    crop: CropContext
    start_rc: tuple[float, float]
    start_yaw: float = 0.0
    body: str = "moon"
    payload_kg: float = 0.0
    dt: float = 0.1
    v_max_default: float = 0.3
    omega_max: float = 0.6
    kp_yaw: float = 2.0
    max_steps_per_leg: int = 6000
    downlink_decim: int = 20         # send one Pose TM every N ticks (+ always the leg endpoint)

    cs: ColumnState = field(init=False)
    rc: tuple[float, float] = field(init=False)
    yaw: float = field(init=False)
    g: float = field(init=False)
    history: list[messages.Pose] = field(init=False, default_factory=list)

    def __post_init__(self) -> None:
        f = self.crop.fields
        self.cs = ColumnState(width=self.crop.heightmap.shape[1], height=self.crop.heightmap.shape[0],
                              cell_m=self.crop.cell_m, mass_areal=f["mass_areal"].copy(),
                              density=f["density"].copy(), state_label=f["state_label"].copy(),
                              disturbance=f["disturbance"].copy(), datum=f["datum"].copy())
        b = bodies.get_body(self.body)
        self.g = float(b.g)
        self._params = bodies.params_for_body(self.body)
        self.rc = (float(self.start_rc[0]), float(self.start_rc[1]))
        self.yaw = float(self.start_yaw)
        self._battery_j = ipex_specs.battery_energy_j()
        self._reserve_frac = ipex_specs.BATTERY_RESERVE_FRAC
        self._drive_j_per_m = ipex_specs.drive_energy_per_m()
        self._mass_kg = K.ROVER_MASS_DRY_KG + max(0.0, self.payload_kg)
        self.energy_j = 0.0          # cumulative consumed across the mission
        self.met = 0.0               # mission elapsed time [s]
        self._total_mass0 = self.cs.total_mass()

    @property
    def soc(self) -> float:
        return 1.0 - self.energy_j / self._battery_j

    def _height_at(self, rc: tuple[float, float]) -> float:
        ri = min(max(int(round(rc[0])), 0), self.cs.height - 1)
        ci = min(max(int(round(rc[1])), 0), self.cs.width - 1)
        return float(self.cs.datum[ri, ci] + self.cs.mass_areal[ri, ci] / self.cs.density[ri, ci])

    def step_toward(self, goal: tuple[float, float], v_max: float, radius: float, *, leg_id: int = 0
                    ) -> "tuple[messages.Pose | None, bool, int | None, float, float]":
        """One control tick of pure pursuit toward ``goal`` (row, col).

        Returns (pose, done, status, step_commanded_m, step_achieved_m). ``pose`` is None and done=True
        with status REACHED when the rover is already within ``radius``. This is the single per-tick
        primitive used by both :meth:`execute_goto` (loopback/CCSDS) and the rclpy executive timer.
        """
        drow = goal[0] - self.rc[0]
        dcol = goal[1] - self.rc[1]
        if math.hypot(drow, dcol) <= radius:
            return None, True, messages.LEG_REACHED, 0.0, 0.0
        desired = math.atan2(drow, dcol)                             # CONTRACT.md §4 heading convention
        err = _wrap(desired - self.yaw)
        omega = max(-self.omega_max, min(self.omega_max, self.kp_yaw * err))
        v = v_max if abs(err) < 0.2 else v_max * max(0.0, math.cos(err))  # creep into the turn

        h0 = self._height_at(self.rc)
        new_rc, new_yaw, telem = drive.drive_step(
            self.cs, self.rc, self.yaw, v, omega, dt=self.dt,
            params=self._params, payload_kg=self.payload_kg, g=self.g)
        self.rc, self.yaw = new_rc, new_yaw

        step_cmd = abs(v) * self.dt
        step_ach = abs(telem["v_achieved"]) * self.dt
        de = self._drive_j_per_m * step_cmd                          # wheel work ~ commanded distance
        dh = self._height_at(self.rc) - h0
        if dh > 0.0:
            de += self._mass_kg * self.g * dh                        # gravity lift on climbs
        self.energy_j += de
        self.met += self.dt

        p = messages.Pose(leg_id=leg_id, row=self.rc[0], col=self.rc[1], yaw_rad=self.yaw,
                          v_achieved_mps=telem["v_achieved"], slip=telem["slip"],
                          sinkage_m=telem["sinkage_m"], slope_rad=telem["slope_rad"],
                          soc=self.soc, entrapped=bool(telem["entrapped"]))
        self.history.append(p)
        if telem["entrapped"]:
            return p, True, messages.LEG_ENTRAPPED, step_cmd, step_ach
        if self.soc <= self._reserve_frac:
            return p, True, messages.LEG_LOW_BATTERY, step_cmd, step_ach
        return p, False, None, step_cmd, step_ach

    def execute_goto(self, cmd: messages.GoTo) -> tuple[messages.Leg, list[messages.Pose]]:
        """Run an onboard pure-pursuit leg to ``cmd``'s waypoint; return (leg summary, downlink samples)."""
        goal = (float(cmd.goal_row), float(cmd.goal_col))
        v_max = float(cmd.v_max_mps) if cmd.v_max_mps > 0 else self.v_max_default
        radius = float(cmd.goal_radius_cells)
        commanded = achieved = 0.0
        energy0 = self.energy_j
        downlink: list[messages.Pose] = []
        status = messages.LEG_MAX_STEPS
        last_pose: messages.Pose | None = None

        for step in range(self.max_steps_per_leg):
            p, done, st, step_cmd, step_ach = self.step_toward(goal, v_max, radius, leg_id=cmd.leg_id)
            if p is None:
                status = messages.LEG_REACHED
                break
            commanded += step_cmd
            achieved += step_ach
            last_pose = p
            if step % self.downlink_decim == 0:
                downlink.append(p)
            if done:
                status = st if st is not None else messages.LEG_MAX_STEPS
                break

        if last_pose is not None and (not downlink or downlink[-1] is not last_pose):
            downlink.append(last_pose)                               # always report the leg endpoint
        leg = messages.Leg(leg_id=cmd.leg_id, status=status, commanded_dist_m=commanded,
                           achieved_dist_m=achieved, energy_J=self.energy_j - energy0,
                           mass_kg=self.cs.total_mass(), final_row=self.rc[0], final_col=self.rc[1])
        return leg, downlink

    def mass_drift(self) -> float:
        """Relative change in conserved total mass since construction (should be ~0)."""
        return abs(self.cs.total_mass() - self._total_mass0) / max(self._total_mass0, 1.0)

    def serve(self, link, *, expect_legs: int | None = None, idle_timeout: float = 2.0) -> int:
        """Receive telecommands on ``link`` and answer with telemetry. Returns legs served.

        Blocks on each ``recv``; exits after ``expect_legs`` legs (if given) or after ``idle_timeout``
        seconds of silence once at least one leg has been served. Used by both the loopback demo
        (in a thread) and the rclpy bridge.
        """
        served = 0
        while True:
            pkt = link.recv(timeout=idle_timeout)
            if pkt is None:
                if expect_legs is None or served >= expect_legs:
                    return served
                continue
            msg = messages.decode(pkt)
            if isinstance(msg, messages.Safe):
                leg = messages.Leg(leg_id=0, status=messages.LEG_SAFED, commanded_dist_m=0.0,
                                   achieved_dist_m=0.0, energy_J=0.0, mass_kg=self.cs.total_mass(),
                                   final_row=self.rc[0], final_col=self.rc[1])
                link.send(messages.encode(leg, met=self.met))
                continue
            if isinstance(msg, messages.GoTo):
                leg, downlink = self.execute_goto(msg)
                for p in downlink:
                    link.send(messages.encode(p, seq_count=served, met=self.met))
                link.send(messages.encode(leg, seq_count=served, met=self.met))
                served += 1
                if expect_legs is not None and served >= expect_legs:
                    return served
