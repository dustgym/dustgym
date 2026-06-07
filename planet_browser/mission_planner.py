#!/usr/bin/env python3
"""mission_planner.py — SimCity-Space lunar build planner + mission-control report.

Takes a MISSION (build orders on a map), CUT-FILL BALANCES it (route excavated material to fills,
minimizing haul), OPTIMIZES the execution sequence (TSP + battery-aware mid-task recharge), and outputs
a 2-3 page mission-control REPORT (PDF + markdown): coordinates, actions, speed, battery-draw over the
project, cumulative mass/energy, the material balance, and metrics.

Order kinds:
  cut    — excavate a footprint to a depth -> PRODUCES regolith (energy: 4151 J/kg dig).
  fill   — berm/pad/road raise -> CONSUMES regolith (supplied from the nearest cut; hauled in drum loads).
  sinter — fuse a surface into hard pad/road (the lunar concrete analog) IN PLACE -> energy 0.92 MJ/kg
           (~220x dig; the energy bottleneck), no material moved.

Grounded: per-body density/gravity from bodies.json (sysrev terrain_authority/bodies.py); IPEx +
sinter constants from terrain_authority (ipex_specs, constants). The recharge power and sinter-head
power are [CALIB] (no IPEx solar/sinter spec).
Run:  python3 mission_planner.py
"""
from __future__ import annotations

import dataclasses
import hashlib
import heapq
import itertools
import json
import math
import os
import warnings

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- grounded constants: imported from the .py source of truth (terrain_authority), not duplicated.
# The monorepo root (planet_browser's parent) holds terrain_authority/, samples/, scripts/; ensure it is
# importable. _REPO_ROOT also anchors the sample/script paths. (When dustgym is pip-installed,
# terrain_authority imports directly; this insert is the run-from-source fallback.)
import sys
_REPO_ROOT = os.path.dirname(HERE)
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
import numpy as np                                      # for validate_plan (executes orders on the authority)
from terrain_authority import config                    # PO-02: configurable application-data (reports) dir
from terrain_authority import ipex_specs as S          # IPEx energy/battery (NTRS 20240008162) + planner knobs
from terrain_authority import constants as C            # materials + the SINTER_ENABLED gate
from terrain_authority import rassor_mass_model as RM   # ICE-RASSOR drum-fill sensing (NTRS 20210022781)
from terrain_authority import slip as TMS               # conserved slip ladder — weight-aware leg slip
from terrain_authority import terramechanics as TM      # TerramechanicsParams for the slip solve
from terrain_authority import validation as VAL         # RB-01: physical-domain validation at this input boundary
from terrain_authority import vehicles as V             # vehicle/tool capability registry (gate order kinds)
from terrain_authority.bodies import get_body as _get_body, params_for_body  # soil model (soil override)
from terrain_authority.column_state import ColumnState  # conserved authority — for I8 plan validation

DRIVE_SPEED_MS  = S.DRIVE_SPEED_MS                       # 0.30 m/s
DIG_RATE_KG_S   = S.DIG_RATE_KG_PER_HR / 3600.0         # 42 kg/hr
DIG_J_PER_KG    = S.dig_energy_per_kg()                  # ~4151 J/kg (derived)
DRIVE_J_PER_M   = S.drive_energy_per_m()                 # ~135 J/m (derived)
BATTERY_J       = S.battery_energy_j()                   # ~4.79 MJ (12S/30Ah)
DRUM_KG         = S.REGOLITH_PER_CYCLE_KG                # 30 kg/cycle
SINTER_J_PER_KG = C.SINTER_ENERGY_J_PER_KG              # 0.92 MJ/kg [CALIB]
SINTER_POWER_W  = S.SINTER_HEAD_POWER_W                  # 1000 W [CALIB]
CHARGE_W        = S.RECHARGE_POWER_W                     # 700 W [CALIB]
RESERVE_FRAC    = S.BATTERY_RESERVE_FRAC                 # 0.10
ROVER_MASS_KG   = S.ROVER_MASS_CLASS_KG                  # 30 kg-class (for gravity-climb drive energy)
DRIVE_POWER_W   = S.drive_power_w()                      # ~40 W (Table 3 driving cases)
IDLE_POWER_W    = S.IDLE_POWER_W                         # [ASSUMPTION] continuous survival draw (default 0 = off)
SLIP_ALPHA      = 2.0                                    # [CALIB] slip energy multiplier vs tan(slope) (I10 costmap)
_TM_PARAMS      = TM.TerramechanicsParams.from_constants()   # lunar defaults for the weight-aware leg-slip solve

# Per-body OPERATING TIMESCALE (astronomical solar-day lengths; Earth-hours) — so the endurance/report
# prints the correct day/night + sunlit work-window scale for the selected body. solar_day_h = synodic
# (sun-to-sun) day; daylight_h ~= half; op_window_h = the usable high-sun window for solar power.
#   Moon: synodic day 29.53 Earth-days (708.7 h); ~9-11-day high-sun window (per project lead) = 216-264 h.
#   Mars: sol 24.66 h; Earth: 24 h; Ceres: 9.07 h rotation. op_window for non-Moon ~ the midday good-sun hours.
BODY_TIMESCALE = {
    "moon":  {"solar_day_h": 708.7, "daylight_h": 354.4, "op_window_h": (216.0, 264.0), "day_label": "lunar day"},
    "mars":  {"solar_day_h": 24.66, "daylight_h": 12.33, "op_window_h": (6.0, 8.0),     "day_label": "sol"},
    "earth": {"solar_day_h": 24.0,  "daylight_h": 12.0,  "op_window_h": (6.0, 8.0),     "day_label": "day"},
    "ceres": {"solar_day_h": 9.07,  "daylight_h": 4.54,  "op_window_h": (2.0, 3.0),     "day_label": "Ceres day"},
}


def body_timescale(body):
    """Operating timescale for `body` (per BODY_TIMESCALE); a generic ~24 h fallback for unlisted bodies."""
    return dict(BODY_TIMESCALE.get(body, {"solar_day_h": 24.0, "daylight_h": 12.0,
                                          "op_window_h": (6.0, 8.0), "day_label": "day"}), body=body)
# Sinter gate is C.SINTER_ENABLED (single source, in terrain_authority.constants); read live below.


def body_density(body):
    with open(os.path.join(HERE, "bodies.json")) as f:
        return float(json.load(f)[body]["bulk_density"])


def body_gravity(body):
    """Surface gravity [m/s^2] for the body (bodies.json, sysrev MEASURED). Used for haul lift energy."""
    with open(os.path.join(HERE, "bodies.json")) as f:
        return float(json.load(f)[body]["g"])


def _bodies():
    """Known body keys from bodies.json (the py-generated single source); excludes the _ipex block."""
    with open(os.path.join(HERE, "bodies.json")) as f:
        return {k for k in json.load(f) if not k.startswith("_")}


@dataclasses.dataclass
class BuildOrder:
    action: str
    kind: str               # "cut" | "fill" | "sinter"
    x: float; y: float
    footprint_m2: float
    depth_m: float          # cut depth / fill height / sinter depth
    note: str = ""
    def mass_kg(self, rho): return self.footprint_m2 * self.depth_m * rho


@dataclasses.dataclass
class Mission:
    name: str; body: str; orders: list
    charger: tuple = (0.0, 0.0); date: str = "2026-06-03"
    #: precedence as (before_action, after_action) pairs by order action-name (I9): the trip(s) touching
    #: `after` must be sequenced after the trip(s) touching `before` (e.g. grade road before hauling on it).
    precedence: list = dataclasses.field(default_factory=list)
    vehicle: str = "ipex"                              # the platform doing the work (vehicles.VEHICLES)
    tools: tuple = ()                                  # tools mounted on it (vehicles.TOOLS) -> extra capabilities
    soil: str = ""                                     # regolith model override (a body name); "" -> the body's own
    #: discrete keep-out obstacles (boulders / no-go zones) in the LOCAL order frame, as circles
    #: {x, y, r} in metres. Hauls route AROUND them (cells inside become impassable on the costmap) and a
    #: build placed inside one is rejected. Single-vehicle; complements the slope/crater hazard costmap (I10).
    keepouts: tuple = ()
    @property
    def density(self): return body_density(self.body)


def mission_soil_params(mission):
    """The TerramechanicsParams (soil/Bekker model) a mission's drive physics uses: its `soil` override
    (any body's regolith, e.g. Earth dry-sand on a lunar map) or the body's own when no override is set.
    Gravity stays the body's (see body_gravity) -- soil and gravity are independent (terramechanics.py)."""
    return params_for_body(mission.soil or mission.body)


_ORDER_KINDS = ("cut", "fill", "sinter")
_ORDER_FIELDS = ("action", "kind", "x", "y", "footprint_m2", "depth_m")
#: order kind -> the vehicle capability it requires (vehicles.ACTIONS). The fleet (selected vehicle +
#: mounted tools) must have it or the order is refused -- e.g. sinter needs the separate sinter Tool.
KIND_CAPABILITY = {"cut": "excavate", "fill": "dump", "sinter": "sinter"}


def mission_from_dict(payload):
    """Build a Mission from a JSON-style dict (the browser's build-order queue: see index.html).

    Validates the body against bodies.json and every order's required fields + kind; raises ValueError
    on malformed input (NO silent defaults for the physics inputs). Sinter orders are accepted here but
    refused downstream in plan_and_simulate while the gate is off (see constants.SINTER_ENABLED)."""
    if not isinstance(payload, dict):
        raise ValueError("mission payload must be a JSON object")
    body = payload.get("body")
    if body not in _bodies():
        raise ValueError(f"unknown body {body!r}; known: {sorted(_bodies())}")
    # the fleet doing the work: a vehicle + mounted tools -> its capability set gates the order kinds.
    veh = str(payload.get("vehicle", V.DEFAULT_VEHICLE))
    tools = tuple(str(t) for t in (payload.get("tools") or ()))
    try:
        caps = V.capabilities_of(veh, tools=tools)
    except KeyError as e:
        raise ValueError(str(e))                       # unknown vehicle/tool -> 400, not 500
    soil = str(payload.get("soil") or "").strip()      # regolith model override (a body name); "" -> body's own
    if soil:
        try:
            _get_body(soil)                            # validate it is a known body (the soil source)
        except KeyError as e:
            raise ValueError(str(e))
        if _get_body(soil).name == _get_body(body).name:
            soil = ""                                  # same as the body -> no override stored
    raw = payload.get("orders")
    if not isinstance(raw, list) or not raw:
        raise ValueError("mission needs a non-empty 'orders' list")
    orders = []
    for i, o in enumerate(raw):
        if not isinstance(o, dict):
            raise ValueError(f"order {i} must be an object")
        missing = [k for k in _ORDER_FIELDS if k not in o]
        if missing:
            raise ValueError(f"order {i} missing field(s): {missing}")
        if o["kind"] not in _ORDER_KINDS:
            raise ValueError(f"order {i} kind {o['kind']!r} not in {_ORDER_KINDS}")
        need = KIND_CAPABILITY.get(o["kind"])
        if need and need not in caps:                  # capability gate: does THIS fleet have the verb?
            if o["kind"] == "sinter":
                raise ValueError(
                    f"order {i}: sinter is GATED OFF -- no vehicle in the fleet carries a sinter tool "
                    f"({veh!r} is a drum excavator; sinter is a separate Tool to mount).")
            raise ValueError(
                f"order {i}: kind {o['kind']!r} needs the {need!r} capability, which the fleet "
                f"({veh!r} + tools {list(tools)}) lacks.")
        # RB-01: reject NaN/Inf coords and non-positive footprint/depth at this public boundary
        # (float() alone accepts float("nan"); a negative depth or zero area is physically meaningless).
        orders.append(BuildOrder(
            action=str(o["action"]), kind=str(o["kind"]),
            x=VAL.ensure_finite_scalar(o["x"], f"order {i} x"),
            y=VAL.ensure_finite_scalar(o["y"], f"order {i} y"),
            footprint_m2=VAL.ensure_positive_scalar(o["footprint_m2"], f"order {i} footprint_m2"),
            depth_m=VAL.ensure_positive_scalar(o["depth_m"], f"order {i} depth_m"),
            note=str(o.get("note", ""))))
    c = payload.get("charger", (0.0, 0.0))
    kwargs = dict(name=str(payload.get("name", "Build Mission")), body=body, orders=orders,
                  charger=(VAL.ensure_finite_scalar(c[0], "charger x"),
                           VAL.ensure_finite_scalar(c[1], "charger y")),
                  vehicle=veh, tools=tools, soil=soil)
    if "date" in payload:
        kwargs["date"] = str(payload["date"])
    prec = payload.get("precedence")                       # I9: [[before_action, after_action], ...]
    if prec is not None:
        actions = {o.action for o in orders}
        pairs = []
        for p in prec:
            if not (isinstance(p, (list, tuple)) and len(p) == 2):
                raise ValueError(f"precedence entry {p!r} must be [before_action, after_action]")
            b, a = str(p[0]), str(p[1])
            if b not in actions or a not in actions:
                raise ValueError(f"precedence {b!r}->{a!r} references an unknown order action")
            pairs.append((b, a))
        kwargs["precedence"] = pairs
    kos = payload.get("keepouts")                          # discrete keep-out obstacles (circles, local m)
    if kos is not None:
        if not isinstance(kos, list):
            raise ValueError("'keepouts' must be a list of {x, y, r} circles")
        clean = []
        for j, k in enumerate(kos):
            if not isinstance(k, dict) or not all(f in k for f in ("x", "y", "r")):
                raise ValueError(f"keepout {j} must be an object with x, y, r")
            clean.append({"x": VAL.ensure_finite_scalar(k["x"], f"keepout {j} x"),
                          "y": VAL.ensure_finite_scalar(k["y"], f"keepout {j} y"),
                          "r": VAL.ensure_positive_scalar(k["r"], f"keepout {j} r")})
        kwargs["keepouts"] = tuple(clean)
    return Mission(**kwargs)


def _d(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])


# ---- cut-fill balance: route excavated material to fills, nearest-first ------------------------
# Bulking/swell (I7, planner side): a CUT excavates BANK (in-situ) material; a FILL places LOOSE spoil,
# which bulks. Mass is conserved: cut at rho_bank = bulk*SWELL, fill at rho_loose = bulk (bodies.json).
SWELL = C.RHO_DEEP / C.RHO_SPOIL


def balance(mission: Mission):
    rho_bank, rho_loose = mission.density * SWELL, mission.density
    cuts = [(o, o.mass_kg(rho_bank)) for o in mission.orders if o.kind == "cut"]
    fills = [(o, o.mass_kg(rho_loose)) for o in mission.orders if o.kind == "fill"]
    supply = {id(o): m for o, m in cuts}
    flows = []                                          # (cut, fill, mass, dist)
    for fo, need in fills:
        rem = need
        for co, _ in sorted(cuts, key=lambda cm: _d((cm[0].x, cm[0].y), (fo.x, fo.y))):
            if rem <= 1e-6: break
            avail = supply[id(co)]
            if avail <= 1e-6: continue
            take = min(rem, avail)
            flows.append((co, fo, take, _d((co.x, co.y), (fo.x, fo.y))))
            supply[id(co)] -= take; rem -= take
        if rem > 1e-6:
            flows.append((None, fo, rem, 0.0))          # deficit: imported material (flagged)
    for co, _ in cuts:                                  # un-routed cut mass: excavated spoil (dug, then piled)
        rem = supply[id(co)]
        if rem > 1e-6:
            flows.append((co, None, rem, 0.0))          # surplus: (cut, None) spoil flow, symmetric to import
    surplus_kg = sum(m for c, f, m, _ in flows if c is not None and f is None)
    return flows, surplus_kg


# ---- sequence + simulate (battery-aware, sinter, haul shuttles) --------------------------------
# ---- objectives: the metric the sequencer optimizes / the user sorts by ------------------------
# Each entry is (direction, totals -> scalar). "min" objectives are minimized, "max" maximized (the
# optimizer negates them). Every objective reads from the SIMULATED totals, so ANY algorithm can be
# scored against ANY objective -- overall duration/time, energy, average power, drive distance, recharge
# stops, or amount moved (constant for a full plan -> a sort key; the lever once plans are budgeted).
OBJECTIVES = {
    "time":     ("min", lambda T: T["time_s"]),
    "duration": ("min", lambda T: T["time_s"]),            # alias for "overall duration"
    "energy":   ("min", lambda T: T["energy_J"]),
    "power":    ("min", lambda T: T["avg_power_w"]),        # average power output
    "distance": ("min", lambda T: T["distance_m"]),
    "charges":  ("min", lambda T: T["charges"]),
    "mass":     ("max", lambda T: T["mass_kg"]),            # amount moved
}
# Sequencer algorithms. nearest/greedy/two_opt/or_opt/lk are heuristics (objective-scored by simulation);
# brute + held_karp are EXACT (brute over permutations <=7; Held-Karp DP exact-on-driving-distance <=16);
# auto dispatches to the strongest solver the problem size + precedence allow ("solved in sequence").
SEQUENCERS = ("auto", "nearest", "greedy", "two_opt", "or_opt", "lk", "brute", "held_karp")
BRUTE_MAX_TRIPS = 7          # exhaustive permutation search only up to 7! = 5040
HELD_KARP_MAX_TRIPS = 16     # Held-Karp DP is O(2^n * n^2); ~16 trips is the practical ceiling


def _build_trips(mission, dem, dem_origin, max_traverse_slope_deg):
    """Order-INDEPENDENT trip construction: cut->fill flows (I10-routed haul + exact gravity lift) and
    sinters. Returns (trips, flows, surplus_kg, meta). meta carries the routing summary; trips carry the
    per-trip dig/haul/lift energy so any visit order can be simulated/scored downstream."""
    rho = mission.density
    g = body_gravity(mission.body)                          # for haul lift energy (exact m*g*dh)
    _soil = mission_soil_params(mission)                    # soil model for the haul slip (soil override)
    flows, surplus_kg = balance(mission)
    sinters = [o for o in mission.orders if o.kind == "sinter"]
    if sinters and not C.SINTER_ENABLED:
        raise RuntimeError(
            f"{len(sinters)} sinter order(s) present but sinter is GATED OFF for the IPEx baseline "
            "(drum excavator, no sinter tool; sinter energy ~14-20x the pack per kg). Enable a "
            "sinter-equipped variant via terrain_authority.constants.SINTER_ENABLED.")
    trips = []
    straight_haul_m = 0.0; routed_haul_m = 0.0; blocked_legs = 0
    for co, fo, mass, dist in flows:
        if co is None:
            trips.append(dict(kind="import", site=(fo.x, fo.y), label=f"Import fill: {fo.action}",
                              mass=mass, dig_e=mass*DIG_J_PER_KG, dig_t=mass/DIG_RATE_KG_S,
                              haul_m=0.0, haul_e=0.0, lift_e=0.0, dest=(fo.x, fo.y),
                              actions=frozenset({fo.action})))
        elif fo is None:
            # surplus (un-routed) cut mass: it is still EXCAVATED -- the dominant dig cost (4151 J/kg) must
            # enter the plan. Dig in place; the spoil-disposal haul to a dump is a separate unmodeled term
            # (no spoil-site coordinate to fabricate one), so haul/lift = 0 here.
            trips.append(dict(kind="dig", site=(co.x, co.y), label=f"Excavate spoil: {co.action}",
                              mass=mass, dig_e=mass*DIG_J_PER_KG, dig_t=mass/DIG_RATE_KG_S,
                              haul_m=0.0, haul_e=0.0, lift_e=0.0, dest=(co.x, co.y),
                              actions=frozenset({co.action})))
        else:
            loads = max(1, math.ceil(mass / DRUM_KG))
            leg = base = dist                           # one-way cut<->fill distance (straight line)
            if dem is not None:
                leg, base, reached = routed_distance(dem, dem_origin, (co.x, co.y), (fo.x, fo.y),
                                                     max_slope_deg=max_traverse_slope_deg,
                                                     keepouts=mission.keepouts)
                if not reached:
                    blocked_legs += 1                   # no safe corridor -> fell back to the line; flagged
            straight_haul_m += base; routed_haul_m += leg
            haul_m = 2 * leg * loads                    # shuttle: cut<->fill, one round trip per drum load
            dh = haul_elevation_gain_m(dem, dem_origin, (co.x, co.y), (fo.x, fo.y))
            # #1 slip-loss: the wheel travels 1/(1-slip) per metre of ground on a slope, so the haul costs
            # more than flat 135 J/m. slip from the cut<->fill slope; no DEM/flat -> slip 0 -> haul_e = flat.
            slope_haul = math.degrees(math.atan2(abs(dh), leg)) if leg > 1e-9 else 0.0
            # weight-coupled: the loaded outbound leg (carrying ~DRUM_KG) slips more than the empty
            # return; each pays 1/(1-slip) per ground metre. (haul_m = out + back = 2*leg*loads.)
            out_m = back_m = leg * loads
            slip_loaded = slip_alpha_to_slip(slope_haul, payload_kg=DRUM_KG, g=g, params=_soil)
            slip_empty = slip_alpha_to_slip(slope_haul, payload_kg=0.0, g=g, params=_soil)
            haul_e = (out_m * DRIVE_J_PER_M / (1.0 - slip_loaded)
                      + back_m * DRIVE_J_PER_M / (1.0 - slip_empty))
            trips.append(dict(kind="cutfill", site=(co.x, co.y), label=f"{co.action} → {fo.action}",
                              mass=mass, dig_e=mass*DIG_J_PER_KG, dig_t=mass/DIG_RATE_KG_S,
                              haul_m=haul_m, haul_e=haul_e, lift_e=mass * g * max(0.0, dh), dest=(fo.x, fo.y),
                              actions=frozenset({co.action, fo.action})))
    for o in sinters:
        m = o.mass_kg(rho)
        trips.append(dict(kind="sinter", site=(o.x, o.y), label=o.action, mass=m, lift_e=0.0,
                          sinter_e=m*SINTER_J_PER_KG, sinter_t=m*SINTER_J_PER_KG/SINTER_POWER_W,
                          dest=(o.x, o.y), actions=frozenset({o.action})))
    meta = dict(straight_haul_m=straight_haul_m, routed_haul_m=routed_haul_m, blocked_legs=blocked_legs,
                routed=dem is not None, traverse_cap_deg=float(max_traverse_slope_deg))
    return trips, flows, surplus_kg, meta


def trip_precedence(trips, mission):
    """I9: lift the mission's order-level precedence (before_action -> after_action) to TRIP-index
    constraints (i, j): trip i must precede trip j. A trip 'touches' the actions of the orders it serves
    (a cut->fill trip touches both). Self-edges are dropped. Returns a list of (i, j)."""
    pairs = []
    for before, after in (mission.precedence or []):
        for i, ti in enumerate(trips):
            if before in ti["actions"]:
                for j, tj in enumerate(trips):
                    if i != j and after in tj["actions"]:
                        pairs.append((i, j))
    return sorted(set(pairs))


def _precedence_is_feasible(n, pairs):
    """AL2 guard: do the (i, j) 'trip i before trip j' constraints admit ANY valid ordering, or do they
    form a cycle (no build sequence can satisfy them)? Kahn topological sort over all n trips -- feasible
    iff every trip can be emitted. Returns True (acyclic / satisfiable) or False (cyclic / unsatisfiable)."""
    indeg = [0] * n
    succ = [[] for _ in range(n)]
    for i, j in pairs:
        succ[i].append(j)
        indeg[j] += 1
    queue = [k for k in range(n) if indeg[k] == 0]
    emitted = 0
    while queue:
        u = queue.pop()
        emitted += 1
        for v in succ[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                queue.append(v)
    return emitted == n


def _simulate(mission, trips):
    """Battery-aware simulation of an ORDERED trip list (phase-split recharging; drive between sites by
    straight line; intra-trip haul/lift baked into each trip). Pure in (mission, trips) so the optimizer
    can score any candidate order. Returns (tl, per_trip, core) -- core = the order-dependent metrics."""
    pos = list(mission.charger); batt = BATTERY_J; t = 0.0
    cum_mass = 0.0; cum_energy = 0.0; charges = 0; reserve = RESERVE_FRAC * BATTERY_J
    tl = []; per_trip = []

    def drive(to):
        nonlocal pos, batt, t, cum_energy
        d = _d(pos, to)
        if d <= 1e-9: return
        e = d * DRIVE_J_PER_M; dur = d / DRIVE_SPEED_MS
        tl.append(dict(t0=t, t1=t+dur, kind="drive", batt0=batt, batt1=batt-e, mass=0.0, speed=DRIVE_SPEED_MS,
                       x0=pos[0], y0=pos[1], x1=to[0], y1=to[1]))   # P5: rover moves pos -> to
        pos = list(to); batt -= e; t += dur; cum_energy += e

    def charge():
        nonlocal batt, t, charges
        drive(mission.charger); need = BATTERY_J - batt; dur = need / CHARGE_W
        tl.append(dict(t0=t, t1=t+dur, kind="charge", batt0=batt, batt1=BATTERY_J, mass=0.0, speed=0.0,
                       x0=pos[0], y0=pos[1], x1=pos[0], y1=pos[1]))  # parked at charger
        batt = BATTERY_J; t += dur; charges += 1

    def spend(kind, total_e, total_dur, work_pos, mass=0.0, speed=0.0, haul_m=0.0, haul_e=None, lift_e=0.0):
        # draw total_e at work_pos, splitting across recharges; haul_e is the haul drive ENERGY (#1
        # slip-adjusted; default flat 135 J/m), haul_m the haul distance (TIME); lift_e the uphill gravity work.
        nonlocal batt, t, cum_mass, cum_energy
        if haul_e is None:
            haul_e = haul_m * DRIVE_J_PER_M
        e = total_e + haul_e + lift_e
        dur = total_dur + (haul_m / DRIVE_SPEED_MS)
        spent = 0.0
        while spent < e - 1e-6:
            usable = batt - reserve
            if usable <= 1e-3:
                charge(); drive(work_pos); continue
            chunk = min(e - spent, usable)
            cd = dur * (chunk / e) if e > 0 else 0.0
            tl.append(dict(t0=t, t1=t+cd, kind=kind, batt0=batt, batt1=batt-chunk,
                           mass=mass*(chunk/e) if e > 0 else 0.0, speed=speed,
                           x0=work_pos[0], y0=work_pos[1], x1=work_pos[0], y1=work_pos[1]))  # working at site
            batt -= chunk; t += cd; spent += chunk
        cum_mass += mass; cum_energy += e

    for tr in trips:
        t0 = t; drive(tr["site"])
        if tr["kind"] == "sinter":
            spend("sinter", tr["sinter_e"], tr["sinter_t"], tr["site"], mass=0.0)
        else:
            spend("dig", tr["dig_e"], tr["dig_t"], tr["site"], mass=tr["mass"],
                  haul_m=tr.get("haul_m", 0.0), haul_e=tr.get("haul_e"), lift_e=tr.get("lift_e", 0.0))
        per_trip.append(dict(trip=tr, t_start=t0, t_end=t))
    drive(mission.charger)
    # distance_m = inter-site drive legs (timeline speed*dt) + the intra-trip haul shuttle (cut<->fill, baked
    # into each trip as haul_m but NOT a timeline drive leg). Omitting haul_m under-reported total driving
    # ~9x and made the `distance` objective optimize a quantity missing its largest term.
    drive_m = sum((p["t1"]-p["t0"])*p["speed"] for p in tl)
    haul_m = sum(tr.get("haul_m", 0.0) for tr in trips)
    distance_m = drive_m + haul_m
    core = dict(time_s=t, mass_kg=cum_mass, energy_J=cum_energy, charges=charges, distance_m=distance_m,
                avg_power_w=(cum_energy / t if t > 1e-9 else 0.0))
    return tl, per_trip, core


def _score(core, objective):
    """(sortable, raw) for a SINGLE objective: sortable is always MINIMIZED (max objectives negated)."""
    direction, fn = OBJECTIVES[objective]
    raw = fn(core)
    return (raw if direction == "min" else -raw), raw


def parse_objective(objective):
    """Normalize an objective spec to a weight dict. Accepts a single name ('time'), a dict
    ({'time': 0.6, 'energy': 0.4}), or a 'name:w,name:w' string. A single name -> {name: 1.0}. Every
    component must be a known objective. Weights are renormalized to sum to 1."""
    if isinstance(objective, str) and objective in OBJECTIVES:
        return {objective: 1.0}
    if isinstance(objective, str):                          # "time:0.6,energy:0.4"
        spec = {}
        for part in objective.split(","):
            name, _, w = part.partition(":")
            spec[name.strip()] = float(w) if w.strip() else 1.0
        objective = spec
    if not isinstance(objective, dict) or not objective:
        raise ValueError(f"unparseable objective {objective!r}")
    for k in objective:
        if k not in OBJECTIVES:
            raise ValueError(f"unknown objective {k!r}; known: {sorted(OBJECTIVES)}")
    tot = sum(objective.values()) or 1.0
    return {k: v / tot for k, v in objective.items()}


def _make_core_scorer(mission, trips, objective):
    """Return a function core -> sortable scalar (lower = better). For a single objective this is the raw
    metric (max objectives negated). For a WEIGHTED multi-objective it is the weighted sum of each metric
    normalized by a reference plan (the nearest-neighbour order), so differently-scaled metrics combine."""
    weights = parse_objective(objective)
    if len(weights) == 1:
        (name,) = weights
        return lambda core: _score(core, name)[0]
    ref = _simulate(mission, [trips[i] for i in _nn_order(trips, mission)])[2]   # reference scales

    def scorer(core):
        s = 0.0
        for name, w in weights.items():
            direction, fn = OBJECTIVES[name]
            raw, r = fn(core), fn(ref)
            norm = (raw / r) if direction == "min" else (r / max(raw, 1e-9))
            s += w * norm
        return s
    return scorer


def _nn_order(trips, mission, *, eligible_fn=None):
    """Nearest-neighbour order from the charger; if eligible_fn is given, only choose currently-eligible
    trips (precedence-aware)."""
    n = len(trips); seq = []; unv = list(range(n)); cur = mission.charger
    while unv:
        cands = [i for i in unv if eligible_fn(i, seq)] if eligible_fn else unv
        k = min(cands, key=lambda i: _d(cur, trips[i]["site"])); seq.append(k); unv.remove(k)
        cur = trips[k]["site"]
    return seq


def _prec_masks(n, precedence):
    """Per-trip predecessor bitmask: pred[j] has bit i set iff trip i must precede trip j."""
    pred = [0] * n
    for i, j in (precedence or []):
        pred[j] |= (1 << i)
    return pred


def _respects(order, pred):
    """True iff `order` honors every precedence constraint (each trip after all its predecessors)."""
    seen = 0
    for j in order:
        if pred[j] & ~seen:                                # a predecessor of j not yet visited
            return False
        seen |= (1 << j)
    return True


def _held_karp(trips, mission, pred):
    """Exact min-DRIVING-DISTANCE Hamiltonian tour (charger -> all sites -> charger) by Held-Karp DP,
    honoring precedence (a Sequential Ordering Problem). O(2^n * n^2). Returns the trip order; the planner
    then simulates it for the chosen objective's true battery-aware totals (distance is the exact lever;
    it is a near-perfect proxy for time/energy here because dig energy dominates and is order-independent)."""
    n = len(trips)
    pts = [tuple(mission.charger)] + [tuple(t["site"]) for t in trips]
    dmat = [[_d(pts[a], pts[b]) for b in range(n + 1)] for a in range(n + 1)]
    full = (1 << n) - 1
    dp = [[math.inf] * n for _ in range(1 << n)]
    par = [[-1] * n for _ in range(1 << n)]
    for j in range(n):
        if pred[j] == 0:                                   # may go first only if it has no predecessors
            dp[1 << j][j] = dmat[0][j + 1]
    for mask in range(1 << n):
        for j in range(n):
            base = dp[mask][j]
            if base == math.inf:
                continue
            for k in range(n):
                if mask & (1 << k):
                    continue
                if pred[k] & ~mask:                        # k's predecessors not all in `mask`
                    continue
                nm = mask | (1 << k); nd = base + dmat[j + 1][k + 1]
                if nd < dp[nm][k]:
                    dp[nm][k] = nd; par[nm][k] = j
    best, endj = math.inf, -1
    for j in range(n):
        v = dp[full][j] + dmat[j + 1][0]
        if v < best:
            best, endj = v, j
    if endj == -1:                                         # no complete tour honors the precedence DAG
        raise ValueError("precedence is infeasible (cyclic / unsatisfiable): no valid trip ordering exists")
    order = []; mask, j = full, endj
    while j != -1:
        order.append(j); pj = par[mask][j]; mask ^= (1 << j); j = pj
    order.reverse()
    return order


def optimize_sequence(trips, mission, *, algorithm="auto", objective="time", precedence=None):
    """Return a visit order (trip indices) chosen by `algorithm` to optimize `objective` (a name, a
    'name:w,...' string, or a weight dict), honoring `precedence` (list of (i, j): trip i before trip j).

      auto       -- dispatch to the strongest solver the size + precedence allow (brute<=7, held_karp<=16,
                    else lk); precedence routes to the SOP-aware variants.
      nearest    -- distance nearest-neighbour from the charger (no simulation; fast; objective-agnostic).
      greedy     -- append the eligible trip minimizing the objective of the prefix-so-far (sim-scored).
      two_opt    -- nearest seed + 2-opt segment reversals improving the objective (precedence-valid only).
      or_opt     -- nearest seed + Or-opt relocations of 1-3 consecutive trips (precedence-valid only).
      lk         -- 2-opt + Or-opt to convergence (a Lin-Kernighan-STYLE composite, not full variable-depth LK).
      brute      -- exhaustive over (precedence-valid) permutations, <= BRUTE_MAX_TRIPS. Optimal.
      held_karp  -- exact min-driving-distance DP (SOP-aware), <= HELD_KARP_MAX_TRIPS, then simulated."""
    parse_objective(objective)                             # validates the objective spec (raises if bad)
    n = len(trips)
    if n <= 1:
        return list(range(n))
    pred = _prec_masks(n, precedence)
    has_prec = any(pred)

    def eligible(i, placed):
        seen = 0
        for p in placed:
            seen |= (1 << p)
        return (pred[i] & ~seen) == 0

    score_core = _make_core_scorer(mission, trips, objective)

    def score(order):
        return score_core(_simulate(mission, [trips[i] for i in order])[2])

    if algorithm == "auto":
        if n <= BRUTE_MAX_TRIPS:
            return optimize_sequence(trips, mission, algorithm="brute", objective=objective, precedence=precedence)
        # 8..16: exact driving tour (Held-Karp) as a strong SEED, then LK-polish on the REAL (recharge-
        # coupled) objective -- "solved in sequence". >16: LK from the nearest seed.
        algorithm = "held_karp_lk" if n <= HELD_KARP_MAX_TRIPS else "lk"

    if algorithm == "nearest":
        return _nn_order(trips, mission, eligible_fn=eligible if has_prec else None)

    if algorithm == "held_karp" and n <= HELD_KARP_MAX_TRIPS:
        return _held_karp(trips, mission, pred)            # PURE exact driving tour (no real-objective polish)

    if algorithm == "greedy":
        order = []; unv = list(range(n))
        while unv:
            cands = [i for i in unv if eligible(i, order)] if has_prec else unv
            nxt = min(cands, key=lambda i: score(order + [i]))
            order.append(nxt); unv.remove(nxt)
        return order

    if algorithm == "brute" and n <= BRUTE_MAX_TRIPS:
        perms = (p for p in itertools.permutations(range(n)) if not has_prec or _respects(p, pred))
        return list(min(perms, key=score))

    # ---- local-search family (2-opt / Or-opt / LK-style), precedence-valid moves only ----
    def two_opt_moves(o):
        for i in range(n - 1):
            for j in range(i + 1, n):
                yield o[:i] + o[i:j + 1][::-1] + o[j + 1:]

    def or_opt_moves(o):                                   # relocate a run of 1-3 consecutive trips
        for seg in (1, 2, 3):
            for i in range(n - seg + 1):
                chunk = o[i:i + seg]; rest = o[:i] + o[i + seg:]
                for k in range(len(rest) + 1):
                    if k != i:
                        yield rest[:k] + chunk + rest[k:]

    def local_search(seed, use_two_opt=True, use_or_opt=True):
        order = list(seed); best = score(order); gens = []
        if use_two_opt: gens.append(two_opt_moves)
        if use_or_opt: gens.append(or_opt_moves)
        improving = True
        while improving:
            improving = False
            for gen in gens:
                for cand in gen(order):
                    if has_prec and not _respects(cand, pred):
                        continue
                    s = score(cand)
                    if s < best - 1e-9:
                        order, best, improving = list(cand), s, True
        return order

    nn_seed = _nn_order(trips, mission, eligible_fn=eligible if has_prec else None)
    if algorithm == "two_opt":
        return local_search(nn_seed, use_or_opt=False)
    if algorithm == "or_opt":
        return local_search(nn_seed, use_two_opt=False)
    if algorithm == "held_karp_lk":                        # auto's 8-16 path: HK seed + LK polish
        return local_search(_held_karp(trips, mission, pred))
    if algorithm in ("lk", "brute", "held_karp"):          # lk; also the >cap fallback for brute/held_karp
        return local_search(nn_seed)
    raise ValueError(f"unknown algorithm {algorithm!r}; known: {SEQUENCERS}")


# ---- sequence + simulate (battery-aware, sinter, haul shuttles) --------------------------------
def _mission_totals(mission, trips, flows, surplus_kg, meta, core):
    """The mission / material / routing / keep-out totals shared by the single- and multi-vehicle planners.
    `core` carries the simulated time/energy/distance/charges/mass; the caller applies survival + algorithm
    + vehicle fields. Kept DRY so the multi-vehicle aggregate reports the same fields as single-vehicle."""
    return dict(
        core,
        cut_kg=sum(o.mass_kg(mission.density * SWELL) for o in mission.orders if o.kind == "cut"),
        fill_kg=sum(o.mass_kg(mission.density) for o in mission.orders if o.kind == "fill"),
        sinter_kg=sum(o.mass_kg(mission.density) for o in mission.orders if o.kind == "sinter"),
        surplus_kg=surplus_kg,
        deficit_kg=sum(m for c, f, m, d in flows if c is None),
        drum_cycles=sum(max(1, math.ceil(tr["mass"] / DRUM_KG)) for tr in trips if tr["kind"] == "cutfill"),
        lift_energy_J=float(sum(tr.get("lift_e", 0.0) for tr in trips)),
        routed_haul=meta["routed"], blocked_legs=meta["blocked_legs"], traverse_cap_deg=meta["traverse_cap_deg"],
        haul_detour_frac=(meta["routed_haul_m"] / meta["straight_haul_m"] - 1.0)
        if meta["straight_haul_m"] > 1e-9 else 0.0,
        n_keepouts=len(mission.keepouts),
        keepout_conflicts=sum(1 for o in mission.orders for k in mission.keepouts
                              if (o.x - k["x"]) ** 2 + (o.y - k["y"]) ** 2 <= k["r"] ** 2))


def _trip_work_e(tr):
    """A trip's work energy (dig + sinter + haul) -- the load used to balance the fleet allocation."""
    return tr.get("dig_e", 0.0) + tr.get("sinter_e", 0.0) + tr.get("haul_e", 0.0)


def _allocate_trips(trips, vehicles):
    """MV2: SITE-EXCLUSIVE, load-balanced (LPT) allocation of trips to V vehicles. Trips are grouped by
    site so no two vehicles ever work the SAME site (zero co-occupation by construction); whole site-groups
    are then assigned greedily to the least-loaded vehicle by work energy (longest-processing-time first).
    Returns a list of V index-lists (some may be empty if V exceeds the number of sites)."""
    groups: dict = {}
    for idx, tr in enumerate(trips):
        groups.setdefault(tuple(tr["site"]), []).append(idx)

    def gcost(idxs):
        return sum(_trip_work_e(trips[i]) for i in idxs)

    loads = [0.0] * vehicles
    alloc: list = [[] for _ in range(vehicles)]
    for idxs in sorted(groups.values(), key=gcost, reverse=True):   # biggest site-group first (LPT)
        v = min(range(vehicles), key=lambda k: loads[k])
        alloc[v].extend(idxs)
        loads[v] += gcost(idxs)
    return alloc


def _vehicle_conflicts(per_vehicle):
    """MV5: count space-time conflicts -- two DIFFERENT vehicles whose per-trip time windows overlap at the
    SAME site. Site-exclusive allocation makes this 0 by construction; the detector verifies it (and would
    catch a future allocation that lets vehicles share a site). Continuous haul-PATH crossing avoidance is
    not modelled here (future MV work) -- this is site-level deconfliction."""
    spans = [(v, tuple(pt["trip"]["site"]), pt["t_start"], pt["t_end"])
             for v, pv in enumerate(per_vehicle) for pt in pv["per_trip"]]
    conflicts = 0
    for a in range(len(spans)):
        va, sa, s0, s1 = spans[a]
        for b in range(a + 1, len(spans)):
            vb, sb, t0, t1 = spans[b]
            if va != vb and sa == sb and s0 < t1 and t0 < s1:     # same site, overlapping windows
                conflicts += 1
    return conflicts


def plan_multi(mission: Mission, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
               algorithm="nearest", objective="time", vehicles=2):
    """MV1-7: plan a multi-vehicle build mission. Build trips once, allocate them site-exclusively across V
    vehicles (load-balanced), sequence + battery-simulate EACH vehicle independently (they work in parallel
    from the shared charger), and aggregate: makespan = max per-vehicle time (the wall-clock the fleet
    finishes in), energy/distance/charges = fleet sums. Returns the same (trips, flows, per_trip, tl, totals)
    shape as the single-vehicle planner, with per-trip `vehicle` tags + a vehicles_detail breakdown.

    v1 scope + honest gaps: site-exclusive allocation guarantees no two rovers co-occupy a site (verified by
    a space-time conflict detector); the SHARED CHARGER is not contention-modelled (each vehicle recharges
    independently -- a stated simplification); continuous haul-PATH collision avoidance and cross-vehicle
    PRECEDENCE are future MV work (precedence + vehicles>1 is refused, not silently mis-ordered)."""
    if vehicles < 1:
        raise ValueError(f"vehicles must be >= 1 (got {vehicles})")
    if mission.precedence:
        raise RuntimeError(
            "multi-vehicle + precedence is not yet coordinated (v1): cross-vehicle precedence ordering is "
            "future MV work. Plan single-vehicle, or remove the precedence constraints.")
    trips, flows, surplus_kg, meta = _build_trips(mission, dem, dem_origin, max_traverse_slope_deg)
    alloc = _allocate_trips(trips, vehicles)
    per_vehicle = []
    for v, idxs in enumerate(alloc):
        vtrips = [trips[i] for i in idxs]
        if vtrips:
            order = optimize_sequence(vtrips, mission, algorithm=algorithm, objective=objective)
            vtrips = [vtrips[k] for k in order]
        for tr in vtrips:
            tr["vehicle"] = v
        tl, per_trip, core = _simulate(mission, vtrips)
        per_vehicle.append({"vehicle": v, "trips": vtrips, "tl": tl, "per_trip": per_trip, "core": core})
    conflicts = _vehicle_conflicts(per_vehicle)
    makespan = max((pv["core"]["time_s"] for pv in per_vehicle), default=0.0)
    agg = dict(
        time_s=float(makespan),
        mass_kg=sum(pv["core"]["mass_kg"] for pv in per_vehicle),
        energy_J=sum(pv["core"]["energy_J"] for pv in per_vehicle),
        charges=sum(pv["core"]["charges"] for pv in per_vehicle),
        distance_m=sum(pv["core"]["distance_m"] for pv in per_vehicle),
        avg_power_w=0.0)
    agg["avg_power_w"] = agg["energy_J"] / makespan if makespan > 1e-9 else 0.0
    survival_J = IDLE_POWER_W * sum(pv["core"]["time_s"] for pv in per_vehicle)   # idle per vehicle * its time
    all_trips = [tr for pv in per_vehicle for tr in pv["trips"]]
    all_per_trip = [pt for pv in per_vehicle for pt in pv["per_trip"]]
    all_tl = [seg for pv in per_vehicle for seg in pv["tl"]]
    totals = _mission_totals(mission, all_trips, flows, surplus_kg, meta, agg)
    if survival_J > 0.0:
        totals["energy_J"] = agg["energy_J"] + survival_J
        totals["avg_power_w"] = totals["energy_J"] / makespan if makespan > 1e-9 else 0.0
    detail = [{"vehicle": pv["vehicle"], "n_trips": len(pv["trips"]), "time_s": pv["core"]["time_s"],
               "energy_J": pv["core"]["energy_J"], "distance_m": pv["core"]["distance_m"],
               "charges": pv["core"]["charges"]} for pv in per_vehicle]
    totals.update(survival_energy_J=float(survival_J), idle_power_w=float(IDLE_POWER_W),
                  algorithm=algorithm, resolved_algorithm=algorithm, optimality="heuristic",
                  n_precedence=0, objective=str(objective), vehicles=int(vehicles),
                  makespan_s=float(makespan), vehicle_conflicts=int(conflicts), vehicles_detail=detail)
    return all_trips, flows, all_per_trip, all_tl, totals


def plan_and_simulate(mission: Mission, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
                      algorithm="nearest", objective="time", vehicles=1):
    """Plan a single-vehicle build mission: build trips, choose a visit order (pluggable `algorithm` x
    `objective`), simulate it battery-aware, and return (trips, flows, per_trip, tl, totals).

    `vehicles` > 1 dispatches to the multi-vehicle planner (`plan_multi`: site-exclusive fleet allocation
    + per-vehicle battery sim + parallel makespan + space-time deconfliction). vehicles=1 is the default
    single-vehicle product planner."""
    if vehicles != 1:
        return plan_multi(mission, dem=dem, dem_origin=dem_origin,
                          max_traverse_slope_deg=max_traverse_slope_deg,
                          algorithm=algorithm, objective=objective, vehicles=vehicles)
    trips, flows, surplus_kg, meta = _build_trips(mission, dem, dem_origin, max_traverse_slope_deg)
    prec = trip_precedence(trips, mission)                  # I9: order-level precedence -> trip constraints
    if not _precedence_is_feasible(len(trips), prec):       # AL2: fail loud, never a silent 0-trip "success"
        raise RuntimeError(
            "infeasible precedence: the mission's precedence constraints form a cycle, so no build "
            "sequence can satisfy them. Check `mission.precedence` for a loop (e.g. A->B and B->A).")
    order = optimize_sequence(trips, mission, algorithm=algorithm, objective=objective, precedence=prec)
    trips = [trips[i] for i in order]
    tl, per_trip, core = _simulate(mission, trips)
    resolved = algorithm                                    # what 'auto' actually dispatched to
    if algorithm == "auto":
        resolved = "brute" if len(trips) <= BRUTE_MAX_TRIPS else (
            "held_karp_lk" if len(trips) <= HELD_KARP_MAX_TRIPS else "lk")
    # AL1: be explicit about optimality. brute = exact on the objective; held_karp(_lk) = exact on driving
    # distance then polished; anything else (lk past HELD_KARP_MAX_TRIPS) is unbounded local search -- warn.
    optimality = ("exact" if resolved == "brute"
                  else "distance-exact+polish" if resolved in ("held_karp", "held_karp_lk")
                  else "heuristic")
    if optimality == "heuristic" and len(trips) > HELD_KARP_MAX_TRIPS:
        warnings.warn(
            f"plan visit order is heuristic: {len(trips)} trips exceed the exact cap "
            f"(HELD_KARP_MAX_TRIPS={HELD_KARP_MAX_TRIPS}); algorithm '{resolved}' has no optimality bound.",
            stacklevel=2)
    # K11c: continuous idle/heater/survival draw over the WHOLE mission duration -- the likely-dominant
    # multi-day term the active-leg ledger omits. [ASSUMPTION] (IDLE_POWER_W, default 0 = not modelled);
    # folded into the headline energy/avg-power only when set, so a default plan is never silently inflated.
    survival_J = IDLE_POWER_W * core["time_s"]
    totals = _mission_totals(mission, trips, flows, surplus_kg, meta, core)
    if survival_J > 0.0:
        totals["energy_J"] = core["energy_J"] + survival_J
        totals["avg_power_w"] = totals["energy_J"] / core["time_s"] if core["time_s"] > 1e-9 else 0.0
    totals.update(
        survival_energy_J=float(survival_J), idle_power_w=float(IDLE_POWER_W),
        algorithm=algorithm, resolved_algorithm=resolved, optimality=optimality, n_precedence=len(prec),
        objective=str(objective), vehicles=1,
        makespan_s=float(core["time_s"]), vehicle_conflicts=0, vehicles_detail=[])   # uniform fleet schema
    return trips, flows, per_trip, tl, totals


# ---- RB-03: ONE immutable plan artifact that every output is a view of -----------------------------
PLAN_RESULT_VERSION = "1.0"


@dataclasses.dataclass(frozen=True)
class PlanResult:
    """The single source-of-truth plan (RB-03). Produced ONCE by ``plan()``; totals, report, Plan IR,
    timeline, and the browser are VIEWS over it, never independent recomputations of the planner.
    Frozen prevents field reassignment; the contained list/dicts are read-only by convention."""
    mission: "Mission"
    dem_origin: tuple
    trips: list
    flows: dict
    per_trip: list
    tl: list
    totals: dict
    provenance: dict

    def as_tuple(self):
        """The legacy (trips, flows, per_trip, tl, totals) shape older call sites consume."""
        return self.trips, self.flows, self.per_trip, self.tl, self.totals


def _plan_provenance(mission, *, algorithm, objective, vehicles, dem_origin):
    """CT-07: provenance for a PlanResult -- schema version, mode, the planning config, and a DETERMINISTIC
    content hash of the mission + origin + config, so a result is tied to exactly the inputs that made it."""
    canon = json.dumps({
        "mission": dataclasses.asdict(mission), "dem_origin": list(dem_origin),
        "algorithm": str(algorithm), "objective": str(objective), "vehicles": int(vehicles),
    }, sort_keys=True, default=str)
    return {
        "schema_version": PLAN_RESULT_VERSION, "mode": "PLAN",
        "config": {"algorithm": str(algorithm), "objective": str(objective), "vehicles": int(vehicles)},
        "input_sha256": hashlib.sha256(canon.encode()).hexdigest(),
    }


def plan(mission: Mission, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
         algorithm="nearest", objective="time", vehicles=1) -> PlanResult:
    """RB-03 keystone: compute the canonical plan ONCE and package it as an immutable PlanResult. Pass the
    result to ``run`` / ``build_timeline`` / ``plan_ir`` so they do NOT each re-run the planner (the server
    does this), guaranteeing totals/report/timeline/IR/playback describe one and the same plan. Wraps
    plan_and_simulate (single-vehicle or, for vehicles>1, the fleet planner)."""
    trips, flows, per_trip, tl, totals = plan_and_simulate(
        mission, dem=dem, dem_origin=dem_origin, max_traverse_slope_deg=max_traverse_slope_deg,
        algorithm=algorithm, objective=objective, vehicles=vehicles)
    prov = _plan_provenance(mission, algorithm=algorithm, objective=objective,
                            vehicles=vehicles, dem_origin=dem_origin)
    return PlanResult(mission=mission, dem_origin=tuple(dem_origin), trips=trips, flows=flows,
                      per_trip=per_trip, tl=tl, totals=totals, provenance=prov)


# ---- executable Plan IR: the machine-consumable plan a rover / ROS executive runs (vs the human PDF) ----
PLAN_IR_VERSION = "1.0"
_IR_OP = {"cutfill": "CutHaulFill", "dig": "Excavate", "import": "Import", "sinter": "Sinter"}
_IR_DIG_OPS = ("Excavate", "CutHaulFill")
_IR_MODEL_ERR_FRAC = 0.12   # per-action energy/time tolerance band (the plan's a-priori model error, 1-sigma)


def plan_ir(mission: Mission, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
            algorithm="nearest", objective="time", vehicles=1, plan_id=None, result=None):
    """Emit a versioned, machine-EXECUTABLE plan IR -- the artifact a rover / ROS executive consumes, as
    opposed to the human PDF. An ordered list of typed actions (GoTo / Excavate / CutHaulFill / Import /
    Sinter), each with expected duration/energy/distance, a model-error tolerance band, and preconditions
    (battery reserve; for digs the drum cap + the map-coverage gate); plus the precedence DAG over action
    ids, the headline expectations, and a DETERMINISTIC content-hash plan_id (no wall clock). Recharges are
    not positional actions -- they are precondition-driven (an executive recharges when `battery_J_min` is
    violated), so the IR stays valid under the real battery draw. Built by lowering the simulated plan."""
    from .map_channel import COVERAGE_DIG_GATE
    if result is None:                                  # RB-03: reuse the shared plan if given (no recompute)
        result = plan(mission, dem=dem, dem_origin=dem_origin,
                      max_traverse_slope_deg=max_traverse_slope_deg,
                      algorithm=algorithm, objective=objective, vehicles=vehicles)
    trips, _flows, _per_trip, _tl, totals = result.as_tuple()
    reserve_J = round(RESERVE_FRAC * BATTERY_J, 1)
    actions = []
    trip_work_aid = {}                                 # trip index -> its work-action id (precedence lowering)
    # RB-04: track the previous position PER VEHICLE. Each rover starts at the charger and advances along
    # its OWN trips; a single shared `prev` would make a fleet rover's first GoTo measure from the previous
    # rover's last position (a cross-vehicle position leak) and overstate its drive distance/energy.
    prev_by_vehicle: dict = {}
    charger = tuple(mission.charger)
    aid = 0
    for ti, tr in enumerate(trips):
        site = tuple(tr["site"])
        veh = int(tr.get("vehicle", 0))
        prev = prev_by_vehicle.get(veh, charger)
        d = _d(prev, site)
        actions.append({
            "id": aid, "op": "GoTo", "vehicle": veh, "to": [round(site[0], 3), round(site[1], 3)],
            "expect": {"distance_m": round(d, 2), "duration_s": round(d / DRIVE_SPEED_MS, 1),
                       "energy_J": round(d * DRIVE_J_PER_M, 1)},
            "tol": {"energy_frac": _IR_MODEL_ERR_FRAC}, "pre": {"battery_J_min": reserve_J}})
        aid += 1
        op = _IR_OP.get(tr["kind"], "Work")
        work_e = (tr.get("dig_e", 0.0) + tr.get("sinter_e", 0.0) + tr.get("haul_e", 0.0) + tr.get("lift_e", 0.0))
        work_t = (tr.get("dig_t", 0.0) + tr.get("sinter_t", 0.0) + tr.get("haul_m", 0.0) / DRIVE_SPEED_MS)
        pre = {"battery_J_min": reserve_J}
        if op in _IR_DIG_OPS:
            pre["drum_kg_max"] = round(DRUM_KG, 1)
            pre["map_coverage_min"] = COVERAGE_DIG_GATE      # the survey-before-dig gate, as a precondition
        act = {
            "id": aid, "op": op, "vehicle": veh,
            "site": [round(site[0], 3), round(site[1], 3)],
            "dest": [round(tr["dest"][0], 3), round(tr["dest"][1], 3)],
            "mass_kg": round(float(tr.get("mass", 0.0)), 1),
            "loads": (max(1, math.ceil(tr.get("mass", 0.0) / DRUM_KG)) if op == "CutHaulFill" else 0),
            "haul_m": round(tr.get("haul_m", 0.0), 1),
            "actions": sorted(tr.get("actions", [])),
            "expect": {"energy_J": round(work_e, 1), "duration_s": round(work_t, 1)},
            "tol": {"energy_frac": _IR_MODEL_ERR_FRAC}, "pre": pre}
        actions.append(act)
        trip_work_aid[ti] = aid
        aid += 1
        prev_by_vehicle[veh] = tuple(tr.get("dest", site))   # RB-04: advance only THIS vehicle's position
    precedence = sorted({(trip_work_aid[i], trip_work_aid[j])
                         for i, j in trip_precedence(trips, mission)
                         if i in trip_work_aid and j in trip_work_aid})
    if plan_id is None:                                # deterministic content hash (no wall clock)
        key = json.dumps({
            "body": mission.body, "algorithm": algorithm, "objective": str(objective), "vehicles": vehicles,
            "orders": [(o.action, o.kind, o.x, o.y, o.footprint_m2, o.depth_m) for o in mission.orders],
            "ops": [(a["op"], a.get("site"), a.get("to")) for a in actions]}, sort_keys=True)
        plan_id = hashlib.sha1(key.encode()).hexdigest()[:16]
    crs = "IAU_2015:30135" if (dem is not None and mission.body == "moon") else "local"
    return {
        "schema_version": PLAN_IR_VERSION, "plan_id": plan_id, "body": mission.body,
        "vehicles": int(totals.get("vehicles", 1)), "objective": str(objective),
        "algorithm": totals.get("resolved_algorithm", algorithm),
        "frame": {"origin_m": [round(dem_origin[0], 3), round(dem_origin[1], 3)],
                  "charger": [float(mission.charger[0]), float(mission.charger[1])], "crs": crs},
        "actions": actions, "precedence": [list(p) for p in precedence],
        "expect": {"duration_s": round(totals["time_s"], 1), "energy_J": round(totals["energy_J"], 1),
                   "distance_m": round(totals["distance_m"], 1), "charges": int(totals["charges"]),
                   "makespan_s": round(totals.get("makespan_s", totals["time_s"]), 1)},
        "acceptance": {"as_built_tol_m": 0.02, "recharge_is_precondition_driven": True},
        "provenance": result.provenance,                # CT-07: schema/mode/config + input hash of the one plan
    }


# the min-objective metric columns used for Pareto non-domination across algorithms
_PARETO_METRICS = ("time_s", "energy_J", "distance_m", "charges")


def compare_algorithms(mission: Mission, *, objective="time", algorithms=None, dem=None,
                       dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0):
    """Run every concrete sequencer and return their metrics sorted by `objective` (best first) -- 'run
    different algorithms for path planning / optimization, however I sort it'. Each row carries that plan's
    full metrics (re-sort by any column) + a `pareto` flag (non-dominated across time/energy/distance/charges)
    so the trade-off frontier is visible. `objective` may be a single name or a weighted spec."""
    algos = algorithms or [a for a in SEQUENCERS if a != "auto"]   # compare the concrete solvers
    weights = parse_objective(objective)
    rows = []
    for a in algos:
        try:
            _, _, _, _, T = plan_and_simulate(mission, dem=dem, dem_origin=dem_origin,
                                              max_traverse_slope_deg=max_traverse_slope_deg,
                                              algorithm=a, objective=objective)
            rows.append({"algorithm": a, "resolved_algorithm": T["resolved_algorithm"],
                         "time_s": T["time_s"], "energy_J": T["energy_J"], "avg_power_w": T["avg_power_w"],
                         "distance_m": T["distance_m"], "charges": T["charges"], "mass_kg": T["mass_kg"],
                         "lift_energy_J": T["lift_energy_J"]})
        except Exception as e:                              # e.g. sinter gated -> report, don't crash the table
            rows.append({"algorithm": a, "error": str(e)})
    ok = [r for r in rows if "error" not in r]
    # objective_value: single objective = the raw metric; weighted = sum of metrics normalized to the best
    # in this comparison set (min-objectives /best, max-objectives best/), lower = better.
    best = {n: min((OBJECTIVES[n][1](r) for r in ok), default=1.0) for n in weights}
    bestmax = {n: max((OBJECTIVES[n][1](r) for r in ok), default=1.0) for n in weights}
    for r in ok:
        if len(weights) == 1:
            (name,) = weights
            r["objective_value"] = OBJECTIVES[name][1](r)
        else:
            s = 0.0
            for n, w in weights.items():
                direction, fn = OBJECTIVES[n]
                v = fn(r)
                s += w * ((v / best[n]) if direction == "min" else (bestmax[n] / max(v, 1e-9)))
            r["objective_value"] = s
    # Pareto: a plan is non-dominated if no other plan is <= on all metrics and < on at least one
    for r in ok:
        r["pareto"] = not any(o is not r
                              and all(o[m] <= r[m] + 1e-9 for m in _PARETO_METRICS)
                              and any(o[m] < r[m] - 1e-9 for m in _PARETO_METRICS) for o in ok)
    direction = "min" if len(weights) > 1 else OBJECTIVES[next(iter(weights))][0]
    inf = float("inf")
    rows.sort(key=lambda r: (r["objective_value"] if direction == "min" else -r["objective_value"])
              if "objective_value" in r else inf)
    return {"objective": str(objective), "direction": direction, "rows": rows}


def build_timeline(mission: Mission, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
                   algorithm="nearest", objective="time", result=None):
    """P5 (execute + watch): turn the battery-aware simulation into a compact, animatable timeline. Each
    frame is a time-bounded segment carrying the rover's start/end position, battery fraction, phase, and
    cumulative mass moved; the browser interpolates the rover marker + telemetry HUD along it. Positions
    come from the actual sim moves (drive/charge/work), not a reconstruction. Honors the chosen sequencer
    so the animation matches the planned order."""
    if result is None:                                  # RB-03: reuse the shared plan if given (no recompute)
        result = plan(mission, dem=dem, dem_origin=dem_origin,
                      max_traverse_slope_deg=max_traverse_slope_deg,
                      algorithm=algorithm, objective=objective)
    _, _, _, tl, totals = result.as_tuple()
    frames = []
    cum = 0.0
    for s in tl:
        cum += s["mass"]
        frames.append(dict(
            t0=round(s["t0"], 3), t1=round(s["t1"], 3),
            x0=round(float(s["x0"]), 3), y0=round(float(s["y0"]), 3),
            x1=round(float(s["x1"]), 3), y1=round(float(s["y1"]), 3),
            phase=s["kind"], batt0_frac=s["batt0"] / BATTERY_J, batt1_frac=s["batt1"] / BATTERY_J,
            cum_mass_kg=round(cum, 1)))
    return dict(duration_s=round(totals["time_s"], 3), battery_J=float(BATTERY_J),
                charger=list(mission.charger), frames=frames,
                provenance=result.provenance)            # CT-07: ties this playback to the one plan


def _haworth_bundle(bundle_dir=None):
    return bundle_dir or os.path.join(_REPO_ROOT, "samples", "lunar_dem", "haworth_10km_5m")


def load_haworth_dem():
    """Load the real LOLA Haworth 5 m DEM from the sim bundle: returns (heightmap [m], cell_m)."""
    bundle = _haworth_bundle()
    g = json.load(open(os.path.join(bundle, "metadata.json")))["grid"]
    Z = np.fromfile(os.path.join(bundle, "heightmap.rf32"), dtype="<f4").reshape(g["height"], g["width"])
    return Z.astype(np.float64), float(g["cell_m"])


# ---- P4: stream a km-scale DEM by window, without holding the whole map in RAM ------------------
def dem_grid_info(bundle_dir=None):
    """Grid metadata (width/height/cell_m) for a DEM bundle WITHOUT loading the heightfield -- the basis
    for streaming a km-scale map a window at a time instead of holding the whole array in RAM."""
    g = json.load(open(os.path.join(_haworth_bundle(bundle_dir), "metadata.json")))["grid"]
    return {"width": int(g["width"]), "height": int(g["height"]), "cell_m": float(g["cell_m"])}


def read_dem_window(r0, c0, h, w, bundle_dir=None):
    """Read ONLY the [r0:r0+h, c0:c0+w] window of the DEM (seek per row -> exactly h*w*4 bytes of I/O),
    returning (window [m], cell_m). The full 2000x2000 array is never materialised, so this scales to
    km-scale maps with a fixed memory ceiling. The window is clamped to the grid bounds."""
    bundle = _haworth_bundle(bundle_dir)
    info = dem_grid_info(bundle)
    W, H, cell = info["width"], info["height"], info["cell_m"]
    r0 = max(0, min(int(r0), H)); c0 = max(0, min(int(c0), W))
    h = max(0, min(int(h), H - r0)); w = max(0, min(int(w), W - c0))
    out = np.empty((h, w), dtype=np.float64)
    with open(os.path.join(bundle, "heightmap.rf32"), "rb") as f:
        for i in range(h):
            f.seek(((r0 + i) * W + c0) * 4)
            out[i] = np.frombuffer(f.read(w * 4), dtype="<f4").astype(np.float64)
    return out, float(cell)


def flattest_anchor_streamed(window_m=20.0, tile=400, bundle_dir=None):
    """Streamed equivalent of `flattest_anchor`: find the flattest buildable region of a km-scale DEM by
    scanning it TILE BY TILE (each tile read with a halo so the slope + window-mean are correct at tile
    edges), never holding the whole map in RAM. Returns the (x, y) in DEM meters of the global min mean-slope."""
    bundle = _haworth_bundle(bundle_dir)
    info = dem_grid_info(bundle)
    W, H, cell = info["width"], info["height"], info["cell_m"]
    k = max(1, int(round(window_m / cell)))
    halo = k + 1
    try:
        from scipy.ndimage import uniform_filter
    except Exception:
        uniform_filter = None
    best = (math.inf, 0, 0)                                  # (mean_slope, row, col) in global indices
    for tr in range(0, H, tile):
        for tc in range(0, W, tile):
            r0, c0 = max(0, tr - halo), max(0, tc - halo)
            r1, c1 = min(H, tr + tile + halo), min(W, tc + tile + halo)
            Zt, _ = read_dem_window(r0, c0, r1 - r0, c1 - c0, bundle)
            smap = slope_deg_map(Zt, cell)
            sm = uniform_filter(smap, size=k, mode="nearest") if uniform_filter else smap
            ir0, ic0 = tr - r0, tc - c0                      # this tile's interior within the haloed read
            ir1, ic1 = min(r1, tr + tile) - r0, min(c1, tc + tile) - c0
            sub = sm[ir0:ir1, ic0:ic1]
            if sub.size == 0:
                continue
            lr, lc = np.unravel_index(int(np.argmin(sub)), sub.shape)
            val = float(sub[lr, lc])
            if val < best[0]:
                best = (val, tr + lr, tc + lc)
    return float(best[2] * cell), float(best[1] * cell)


def slope_deg_map(Z, cell_m):
    """Per-cell surface slope [deg] from a heightmap (gradient magnitude -> arctan)."""
    gy, gx = np.gradient(Z, cell_m)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


def flattest_anchor(dem, *, window_m=20.0):
    """M11: auto-find the flattest buildable region on a DEM. Returns the (x, y) in DEM meters of the
    cell that minimizes mean slope over a `window_m` box (a pad-sized patch, not a single lucky cell).
    Haworth is ~62% steeper than 15 deg, so an automatic flat-site finder is a real planning aid; this
    is the origin the local order frame anchors to so the slope gate fires on actual buildable ground."""
    Z, cell = dem
    smap = slope_deg_map(Z, cell)
    k = max(1, int(round(window_m / cell)))
    try:
        from scipy.ndimage import uniform_filter
        sm = uniform_filter(smap, size=k, mode="nearest")
    except Exception:
        sm = smap
    row, col = np.unravel_index(int(np.argmin(sm)), sm.shape)
    return float(col * cell), float(row * cell)


def latlon_to_dem_origin(lat, lon, *, bundle_dir=None):
    """M11: project a selenographic lat/lon (deg) to the Haworth DEM order-frame origin (x, y) [m] -- the
    SAME pixel-meter frame flattest_anchor returns -- so a globe site-pick anchors the plan where the user
    clicked instead of the auto flattest site. The DEM is south-polar stereographic on the R=1737400 m Moon
    sphere (IAU_2015:30135; see dem_import). Raises ValueError if the point falls outside the committed
    tile, ImportError if pyproj (the [planner] extra) is absent so the caller can fall back to the anchor."""
    from pyproj import CRS, Transformer
    meta = json.load(open(os.path.join(_haworth_bundle(bundle_dir), "metadata.json")))
    g, b = meta["grid"], meta["world_bounds_m"]
    cell, W, H = float(g["cell_m"]), int(g["width"]), int(g["height"])
    crs = CRS.from_user_input("IAU_2015:30135")
    fwd = Transformer.from_crs(crs.geodetic_crs, crs, always_xy=True)
    xs, ys = fwd.transform(float(lon), float(lat))                       # selenographic -> polar-stereographic m
    ax0, ay0 = float(b["x0"]) + cell / 2.0, float(b["y1"]) - cell / 2.0  # pixel(0,0) CENTER (north-up raster)
    col, row = (xs - ax0) / cell, (ay0 - ys) / cell
    if not (-0.5 <= col <= W - 0.5 and -0.5 <= row <= H - 0.5):
        raise ValueError(f"site lat/lon ({lat:.3f}, {lon:.3f}) is outside the mapped Haworth tile "
                         f"({W}x{H} @ {cell:g} m, IAU_2015:30135)")
    ci, ri = min(max(int(round(col)), 0), W - 1), min(max(int(round(row)), 0), H - 1)
    return float(ci * cell), float(ri * cell)                            # matches flattest_anchor's frame


# ---- I10: hazard + slope/slip-aware haul routing on a DEM costmap -------------------------------
# A straight cut<->fill line ignores craters and steep walls. I10 routes hauls over a slope costmap:
# steeper ground costs more (slip -> more energy/time per meter) and ground past the traverse limit is
# an impassable hazard, so the route bends around craters instead of plowing through them.
_ROUTE_NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


MAX_DROP_M = 2.0   # [ASSUMPTION] a per-cell downward step the rover must not drive off (cliff / crater-rim / pit edge)


def negative_obstacle_mask(Z, *, max_drop_m=MAX_DROP_M):
    """The "don't fall in a hole" hazard: cells overlooking a DROP-OFF — a 3x3-neighbourhood downward step
    greater than ``max_drop_m`` (a cliff / crater-rim / pit edge). OR'd into the costmap as impassable so
    routes keep OFF the edge. The distinct value over the slope cap is the **flat lip** at the top of a drop:
    that cell can be gentle (passable by slope) yet sit at the edge of a fall — this flags it. (On a coarse
    DEM a steep wall is also a drop, so the two overlap there; the sensor / sub-cell + enclosed-sink versions
    are PRD P16/P17.) Returns a boolean mask the size of ``Z``."""
    from scipy.ndimage import minimum_filter
    if max_drop_m is None or max_drop_m <= 0:
        return np.zeros(Z.shape, dtype=bool)
    nbr_min = minimum_filter(Z, size=3, mode="nearest")   # lowest height in the 3x3 neighbourhood
    return (Z - nbr_min) > float(max_drop_m)


def slope_costmap(Z, cell_m, *, max_slope_deg=25.0, slip_alpha=2.0, max_drop_m=None):
    """I10: per-cell traversal cost from terrain slope. cost = 1 + slip_alpha*tan(slope) (a slip-weighted
    per-meter multiplier — slope drives wheel slip, which costs energy/time); cells steeper than
    max_slope_deg are impassable hazards a rover can't safely traverse. When ``max_drop_m`` is set, cells
    overlooking a drop-off (negative_obstacle_mask) are ALSO impassable (the don't-fall-in-a-hole hazard,
    incl. the flat lip a slope cap misses). Returns (cost[H,W], passable bool)."""
    smap = slope_deg_map(Z, cell_m)
    passable = smap <= max_slope_deg
    if max_drop_m is not None:
        passable = passable & ~negative_obstacle_mask(Z, max_drop_m=max_drop_m)
    cost = 1.0 + slip_alpha * np.tan(np.radians(np.minimum(smap, 89.0)))
    return cost, passable


def _apply_keepouts(passable, cell_m, r0, c0, dem_origin, keepouts):
    """Mark cells inside any keep-out circle impassable, in-place, on a cropped costmap. keepouts are
    {x,y,r} in the LOCAL order frame (metres); dem_origin maps that frame to DEM world metres. The crop
    starts at row r0/col c0. Reuses route_least_cost's existing impassable-avoidance -> hauls bend around."""
    if not keepouts:
        return passable
    ox, oy = dem_origin
    H, W = passable.shape
    for k in keepouts:
        kc = (ox + k["x"]) / cell_m - c0                   # keep-out centre in crop-cell coords
        kr = (oy + k["y"]) / cell_m - r0
        rad = k["r"] / cell_m
        c_lo, c_hi = max(0, int(kc - rad)), min(W, int(kc + rad) + 1)
        r_lo, r_hi = max(0, int(kr - rad)), min(H, int(kr + rad) + 1)
        for r in range(r_lo, r_hi):
            for c in range(c_lo, c_hi):
                if (r - kr) ** 2 + (c - kc) ** 2 <= rad * rad:
                    passable[r, c] = False
    return passable


def route_least_cost(cost, passable, cell_m, start_rc, goal_rc):
    """I10: least-(slip-weighted-)cost 8-connected path over a costmap, avoiding impassable cells (Dijkstra).
    Returns (path[list of (r,c)], geometric_length_m, reached). The slip-weighted cost drives the routing
    CHOICE (detour around hazards); the returned length is the geometric path distance used for the haul."""
    H, W = cost.shape
    sr, sc = int(start_rc[0]), int(start_rc[1])
    gr, gc = int(goal_rc[0]), int(goal_rc[1])
    if not (0 <= sr < H and 0 <= sc < W and 0 <= gr < H and 0 <= gc < W):
        return [], math.inf, False
    if not (passable[sr, sc] and passable[gr, gc]):
        return [], math.inf, False
    dist = np.full((H, W), math.inf)
    glen = np.full((H, W), math.inf)
    dist[sr, sc] = 0.0
    glen[sr, sc] = 0.0
    prev = {}
    pq = [(0.0, sr, sc)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        if (r, c) == (gr, gc):
            break
        for dr, dc, seg in _ROUTE_NB:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and passable[nr, nc]:
                nd = d + seg * cell_m * 0.5 * (cost[r, c] + cost[nr, nc])
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    glen[nr, nc] = glen[r, c] + seg * cell_m
                    prev[(nr, nc)] = (r, c)
                    heapq.heappush(pq, (nd, nr, nc))
    if not math.isfinite(dist[gr, gc]):
        return [], math.inf, False
    path = [(gr, gc)]
    while path[-1] != (sr, sc):
        path.append(prev[path[-1]])
    path.reverse()
    return path, float(glen[gr, gc]), True


def routed_distance(dem, dem_origin, a_xy, b_xy, *, max_slope_deg=25.0, slip_alpha=2.0, margin_m=20.0,
                    keepouts=()):
    """I10: terrain-aware haul distance between two LOCAL sites on the real DEM (anchored via dem_origin,
    M11). Crops the DEM to the two sites' bounding box + margin, builds a slope costmap, and routes a
    least-cost hazard-avoiding path. Returns (routed_m, grid_straight_m, reached): routed_m is the path
    length and grid_straight_m the straight-line distance between the same DEM cells (so detour =
    routed/grid_straight - 1 >= 0, measured consistently on the grid, not vs the finer local coords).
    reached=False if no safe corridor exists -> caller falls back to the straight line and flags the leg."""
    Z, cell = dem
    ox, oy = dem_origin
    ax, ay = ox + a_xy[0], oy + a_xy[1]
    bx, by = ox + b_xy[0], oy + b_xy[1]
    H, W = Z.shape
    c0 = max(0, int((min(ax, bx) - margin_m) / cell))
    c1 = min(W, int((max(ax, bx) + margin_m) / cell) + 1)
    r0 = max(0, int((min(ay, by) - margin_m) / cell))
    r1 = min(H, int((max(ay, by) + margin_m) / cell) + 1)
    straight = math.hypot(bx - ax, by - ay)
    if c1 - c0 < 2 or r1 - r0 < 2:                       # sites off the DEM -> can't route
        return straight, straight, False
    crop = Z[r0:r1, c0:c1]
    cost, passable = slope_costmap(crop, cell, max_slope_deg=max_slope_deg, slip_alpha=slip_alpha,
                                   max_drop_m=MAX_DROP_M)   # routes also keep off drop-offs (don't fall in a hole)
    _apply_keepouts(passable, cell, r0, c0, dem_origin, keepouts)   # discrete obstacles -> impassable cells
    hc, wc = crop.shape
    start = (min(max(int(ay / cell) - r0, 0), hc - 1), min(max(int(ax / cell) - c0, 0), wc - 1))
    goal = (min(max(int(by / cell) - r0, 0), hc - 1), min(max(int(bx / cell) - c0, 0), wc - 1))
    grid_straight = math.hypot((goal[1] - start[1]) * cell, (goal[0] - start[0]) * cell)
    _, length_m, reached = route_least_cost(cost, passable, cell, start, goal)
    return (length_m, grid_straight, True) if reached else (straight, straight, False)


def haul_elevation_gain_m(dem, dem_origin, a_xy, b_xy):
    """Net elevation change z(b) - z(a) [m] along a haul, read from the real DEM (anchored via dem_origin,
    M11). Positive = hauling uphill, which costs exact gravity work m*g*dh; <= 0 = downhill (no positive
    lift, and the rover does not regenerate going down). Returns 0.0 with no DEM or if a site is off-grid."""
    if dem is None:
        return 0.0
    Z, cell = dem
    ox, oy = dem_origin
    H, W = Z.shape

    def _z(x, y):
        c, r = int(round((ox + x) / cell)), int(round((oy + y) / cell))
        return float(Z[r, c]) if (0 <= r < H and 0 <= c < W) else None

    za, zb = _z(*a_xy), _z(*b_xy)
    return 0.0 if (za is None or zb is None) else (zb - za)


# ---- endurance / single-charge range (the "true distance before recharge", grounded) ------------
def single_charge_range_m(g, *, slope_deg=0.0, slip=0.0, full_pack=False):
    """One-way driving distance on a single charge [m]. Usable energy / effective drive cost, where the
    effective cost = the flat 135 J/m amplified by wheel slip (1/(1-slip), the wheel travels further than
    the ground) plus the exact gravity-climb term rover_mass*g*sin(slope) on the uphill. `full_pack` uses
    the whole pack; otherwise it stops at the operational reserve."""
    usable = BATTERY_J * (1.0 if full_pack else (1.0 - RESERVE_FRAC))
    jpm = DRIVE_J_PER_M / max(1e-6, 1.0 - slip) + ROVER_MASS_KG * g * math.sin(math.radians(max(0.0, slope_deg)))
    return usable / jpm


def reachable_radius_on_dem(dem, dem_origin, usable_j, g, *, stride=10, slip_alpha=SLIP_ALPHA):
    """DEM-grounded one-charge reach: a Dijkstra DRIVE-ENERGY field from the anchor over a (coarsened)
    slope+slip costmap -- each edge costs seg*135*(1+slip_alpha*tan(theta)) + rover_mass*g*max(0, climb).
    Returns the iso-energy reachable set: radius_m (farthest reachable cell), area_m2, whether the whole
    tile is within one charge, and the worst-cell energy (the hardest point to reach)."""
    Z, cell = dem
    Zc = np.asarray(Z, dtype=np.float64)[::stride, ::stride]    # coarsen for a fast field; honest estimate
    cc = cell * stride
    H, W = Zc.shape
    ox, oy = dem_origin
    ar = min(max(int(round(oy / cell)) // stride, 0), H - 1)
    ac = min(max(int(round(ox / cell)) // stride, 0), W - 1)
    INF = math.inf
    energy = np.full((H, W), INF)
    energy[ar, ac] = 0.0
    pq = [(0.0, ar, ac)]
    while pq:
        e, r, c = heapq.heappop(pq)
        if e > energy[r, c]:
            continue
        for dr, dc, seg in _ROUTE_NB:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W:
                dh = Zc[nr, nc] - Zc[r, c]
                slope = math.atan2(abs(dh), seg * cc)
                step = seg * cc * DRIVE_J_PER_M * (1.0 + slip_alpha * math.tan(slope)) + ROVER_MASS_KG * g * max(0.0, dh)
                ne = e + step
                if ne < energy[nr, nc]:
                    energy[nr, nc] = ne
                    heapq.heappush(pq, (ne, nr, nc))
    reach = energy <= usable_j
    rr, cci = np.where(reach)
    dists = np.hypot((cci - ac) * cc, (rr - ar) * cc)
    finite = energy[np.isfinite(energy)]
    return {
        "radius_m": float(dists.max()) if dists.size else 0.0,
        "area_m2": float(reach.sum() * cc * cc),
        "tile_fully_reachable": bool(reach.all()),
        "worst_cell_J": float(finite.max()) if finite.size else 0.0,
        "worst_cell_pack_frac": float(finite.max() / BATTERY_J) if finite.size else 0.0,
        "grid_cell_m": float(cc),
    }


# ---- power model (P10/K8): per-site power source + thermal derating ------------------------------
POWER_KINDS = ("psr_tower", "sunlit_solar")


def thermal_derate(temp_c):
    """Usable Li-ion pack fraction vs temperature. [CALIB, general Li-ion cold behavior] ~1.0 at >=0 C,
    falling ~1%/C below 0 (a standard rough rule), floored at 0.5. `None` (no temp given) -> 1.0. The IPEx
    qual envelope is -35/+40 C (FIX-5); off-the-shelf cells can't meet -35 C without derating/heaters."""
    if temp_c is None or temp_c >= 0.0:
        return 1.0
    return max(0.5, 1.0 + 0.01 * float(temp_c))


def power_regime(mission, *, kind="psr_tower", charge_power_w=None, temp_c=None):
    """Per-site power model. A PSR (e.g. Haworth, the loaded DEM) has NO sun -> a lander/tower charging
    budget at the charger, available ANYTIME (this is what the planner's recharge actually models -- calling
    it solar was the error). A SUNLIT site recharges from solar, available only during the body's daylight
    fraction, so the EFFECTIVE recharge throughput is duty-limited (duty = daylight_h / solar_day_h).
    Optional cold thermal derating of the usable pack. Day/night from the grounded `body_timescale`."""
    if kind not in POWER_KINDS:
        raise ValueError(f"unknown power kind {kind!r}; known: {POWER_KINDS}")
    ts = body_timescale(mission.body)
    cw = CHARGE_W if charge_power_w is None else float(charge_power_w)
    if kind == "sunlit_solar":
        duty = ts["daylight_h"] / ts["solar_day_h"]
        avail = f"daylight only (~{ts['daylight_h']:.0f} h / {ts['solar_day_h']:.0f} h {ts['day_label']})"
    else:
        duty = 1.0
        avail = "anytime (lander/tower budget; a PSR has no sun)"
    derate = thermal_derate(temp_c)
    return {"kind": kind, "charge_power_w": cw, "duty_frac": duty, "effective_charge_w": cw * duty,
            "availability": avail, "thermal_derate": derate, "usable_pack_J": BATTERY_J * derate,
            "day_label": ts["day_label"], "daylight_h": ts["daylight_h"], "solar_day_h": ts["solar_day_h"]}


def endurance(mission, *, dem=None, dem_origin=(0.0, 0.0), power_site="psr_tower", temp_c=None):
    """Single-charge driving capability ("true distance before recharge"), grounded in the IPEx specs.
    Returns the flat range (full pack + to reserve), the slope+slip-adjusted range at the work-area's
    representative slope (if a DEM is given), and the DEM-grounded reachable radius from the charger."""
    g = body_gravity(mission.body)
    out = {
        "pack_energy_MJ": BATTERY_J / 1e6, "drive_power_w": DRIVE_POWER_W, "flat_j_per_m": DRIVE_J_PER_M,
        "speed_ms": DRIVE_SPEED_MS, "rover_mass_kg": ROVER_MASS_KG, "g": g, "reserve_frac": RESERVE_FRAC,
        "range_flat_full_km": single_charge_range_m(g, full_pack=True) / 1000.0,
        "range_flat_reserve_km": single_charge_range_m(g) / 1000.0,
        "duration_flat_h": single_charge_range_m(g) / DRIVE_SPEED_MS / 3600.0,
    }
    out["power"] = power_regime(mission, kind=power_site, temp_c=temp_c)   # #2 per-site power source
    # ConOps reconciliation [SCHULER24]: the per-charge range is a per-SORTIE bound, not a mission limit.
    # Over the 11-day mission the rover traverses ~70 km AND excavates 5-10 t -> the drums dominate the
    # energy budget, and the sunlit operating window (~9-11 Earth-days) dwarfs any single charge.
    # body-correct operating timescale: is a full-range sortie inside one sunlit window, or does it span days?
    ts = body_timescale(mission.body)
    dur_h = out["duration_flat_h"]
    win_lo, win_hi = ts["op_window_h"]
    ts["sortie_h"] = dur_h
    ts["sorties_per_window"] = win_lo / dur_h                  # how many full-range sorties fit one sun window
    ts["spans_days"] = dur_h / ts["daylight_h"]               # sols/lunar-days a continuous sortie would span
    ts["fits_in_window"] = dur_h <= win_hi
    out["timescale"] = ts
    drive_mj = S.TRAVERSE_KM * 1000.0 * DRIVE_J_PER_M / 1e6
    reg_lo, reg_hi = S.TOTAL_REGOLITH_KG
    out["conops"] = {
        "traverse_km": S.TRAVERSE_KM, "mission_days": S.MISSION_DAYS,
        "regolith_t": [reg_lo / 1000.0, reg_hi / 1000.0],
        "drive_energy_MJ": drive_mj, "drive_packs": drive_mj * 1e6 / BATTERY_J,
        "dig_energy_MJ": [reg_lo * DIG_J_PER_KG / 1e6, reg_hi * DIG_J_PER_KG / 1e6],
        "dig_packs": [reg_lo * DIG_J_PER_KG / BATTERY_J, reg_hi * DIG_J_PER_KG / BATTERY_J],
        "drums_dominate": (reg_lo * DIG_J_PER_KG) > (S.TRAVERSE_KM * 1000.0 * DRIVE_J_PER_M),
    }
    if dem is not None:
        Z, cell = dem
        H, W = np.asarray(Z).shape
        ox, oy = dem_origin
        rc = min(max(int(round(oy / cell)), 0), H - 1); cc0 = min(max(int(round(ox / cell)), 0), W - 1)
        r0 = min(max(0, rc - 200), max(0, H - 400)); c0 = min(max(0, cc0 - 200), max(0, W - 400))
        win = np.asarray(Z)[r0:r0 + 400, c0:c0 + 400]
        med_slope = float(np.median(slope_deg_map(win, cell))) if win.size else 0.0
        slip = min(0.95, slip_alpha_to_slip(med_slope, params=mission_soil_params(mission)))   # soil-aware
        out["work_area_median_slope_deg"] = med_slope
        out["range_slopeslip_km"] = single_charge_range_m(g, slope_deg=med_slope, slip=slip) / 1000.0
        out["reach"] = reachable_radius_on_dem(dem, dem_origin, BATTERY_J * (1 - RESERVE_FRAC), g)
    return out


def slip_alpha_to_slip(slope_deg, payload_kg=0.0, g=None, params=None):
    """Wheel slip from terrain slope AND the rover's laden weight, via the CONSERVED slip ladder
    (slip.slip_sinkage_equilibrium): a steeper grade OR a heavier rover (full drum) -> more slip,
    entrapping near ~45 deg. ``payload_kg`` is the regolith in the drum on this leg (0 = empty); ``g``
    defaults to lunar. This replaces the old slope-only [CALIB] curve so the planner's per-leg slip (and
    the 1/(1-slip) drive-energy inflation) is weight-coupled, consistent with the simulator authority.
    (The per-cell routing costmap keeps the cheap SLIP_ALPHA*tan(slope) ranking heuristic.)"""
    gg = C.g if g is None else float(g)
    p = params if params is not None else _TM_PARAMS     # soil model (params_for_body(soil)); default lunar
    weight_n = (ROVER_MASS_KG + max(0.0, payload_kg)) * gg
    eq = TMS.slip_sinkage_equilibrium(weight_n, math.radians(max(0.0, slope_deg)),
                                      params=p, contact_len_m=0.10, contact_width_m=0.18)
    return max(0.0, min(0.95, float(eq["slip"])))


def validate_plan(mission, *, cell_m=0.5, regolith_depth_m=10.0, max_cells=500, dem=None,
                  dem_origin=(0.0, 0.0), max_slope_deg=15.0, accept_flatness_tol_m=0.02):
    """I8: validate the plan on the CONSERVED authority. Rasterize each order's footprint onto a
    `ColumnState`, execute the cuts (into the drum) then the fills (from the drum), and report mass
    conservation + per-order feasibility + the executed (mass-exact) cut/fill vs the planner's abstract
    estimate. This is the physical-realizability check the footprint estimator can't give: a cut deeper
    than the regolith mantle floors at the datum (infeasible); a fill beyond the drum can't be placed."""
    rho_bank, rho_loose = mission.density * SWELL, mission.density
    cuts = [o for o in mission.orders if o.kind == "cut"]
    fills = [o for o in mission.orders if o.kind == "fill"]
    sides = [math.sqrt(o.footprint_m2) for o in mission.orders]
    margin = 2.0 + (max(sides) / 2 if sides else 0.0)
    x0 = min(o.x - s / 2 for o, s in zip(mission.orders, sides)) - margin
    y0 = min(o.y - s / 2 for o, s in zip(mission.orders, sides)) - margin
    x1 = max(o.x + s / 2 for o, s in zip(mission.orders, sides)) + margin
    y1 = max(o.y + s / 2 for o, s in zip(mission.orders, sides)) + margin
    if max(x1 - x0, y1 - y0) / cell_m > max_cells:          # cap grid for speed; coarsen the cell
        cell_m = max(x1 - x0, y1 - y0) / max_cells
    W = max(1, int(math.ceil((x1 - x0) / cell_m)))
    H = max(1, int(math.ceil((y1 - y0) / cell_m)))
    cs = ColumnState(width=W, height=H, cell_m=cell_m,
                     mass_areal=np.full((H, W), rho_bank * regolith_depth_m, dtype=np.float64))
    # P0 as-built acceptance: when a DEM is given, start the surface at the REAL terrain (datum = terrain
    # - mantle so derive_height == terrain), not a flat mantle. A uniform-depth cut/fill on a sloped surface
    # then leaves a sloped surface -- so the as-built flatness check below actually reveals whether the plan
    # achieves a level pad (it can't on a flat mantle, where everything is trivially flat).
    on_real_dem = dem is not None
    if on_real_dem:
        Z, _dem_cell = dem
        ox, oy = dem_origin
        ci = np.clip(((x0 + (np.arange(W) + 0.5) * cell_m + ox) / _dem_cell).astype(int), 0, Z.shape[1] - 1)
        ri = np.clip(((y0 + (np.arange(H) + 0.5) * cell_m + oy) / _dem_cell).astype(int), 0, Z.shape[0] - 1)
        cs.datum = Z[np.ix_(ri, ci)] - regolith_depth_m
    m0 = cs.total_mass()
    rr, cc = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")

    def _mask(o):
        s = math.sqrt(o.footprint_m2); half = (s / 2) / cell_m
        cx, cy = (o.x - x0) / cell_m, (o.y - y0) / cell_m
        return (np.abs(cc + 0.5 - cx) <= half) & (np.abs(rr + 0.5 - cy) <= half)

    cell_area = cell_m * cell_m
    feasible = True
    exec_cut = 0.0
    for o in cuts:                                          # cuts first -> load the global drum
        mask = _mask(o)
        if not mask.any():
            feasible = False; continue
        moved = cs.cut_to_inventory(mask, o.depth_m * rho_bank)
        exec_cut += moved
        # feasibility = did the authority move the asked-for depth over the RASTERIZED footprint? gate on
        # the on-grid area (mask cells x cell_area), not the analytic footprint, so a sub-grid footprint
        # under-covering the 0.5 m cells doesn't read as infeasible -- only a true datum-floor does.
        if moved < 0.99 * mask.sum() * cell_area * o.depth_m * rho_bank:   # floored at datum -> not enough material
            feasible = False
    exec_fill = 0.0
    for o in fills:                                         # fills from the drum
        mask = _mask(o)
        if not mask.any():
            feasible = False; continue
        target = cs.derive_height().copy(); target[mask] += o.depth_m
        placed = cs.fill_toward(mask, target, max_lift_m=o.depth_m, spoil_density=rho_loose)
        exec_fill += placed
    # fills draw from a SHARED drum -> fill feasibility is a global MATERIAL question, not per-order grid
    # coverage: the plan is fill-infeasible only when the drum ran dry while the executed fill fell short
    # of the analytic plan (a genuine under-supply), not when rasterization shifts the berm by a few cells.
    planned_fill_total = sum(o.footprint_m2 * o.depth_m * rho_loose for o in fills)
    if fills and exec_fill < 0.99 * planned_fill_total and cs.drum_inventory <= 1e-6 * max(1.0, planned_fill_total):
        feasible = False
    drift = abs(cs.total_mass() - m0)
    mass_conserved = drift <= 1e-6 * max(1.0, m0)
    # P0 as-built acceptance: measure the FLATNESS of the executed surface over each worked footprint
    # (RMSE of as-built height about the footprint mean) -- the "did we build a level pad to +/-tol" check
    # the flat-mantle path could never give. Reported per-order (worst + mean); on a flat mantle it is ~0.
    # NOT folded into `feasible` (a uniform-depth excavation of a slope is feasible but legitimately not flat).
    as_built = cs.derive_height()
    flat_rmses = []
    for o in mission.orders:
        mask = _mask(o)
        if int(mask.sum()) < 2:
            continue
        h = as_built[mask]
        flat_rmses.append(float(np.sqrt(np.mean((h - h.mean()) ** 2))))
    as_built_worst = max(flat_rmses) if flat_rmses else 0.0
    as_built_mean = (sum(flat_rmses) / len(flat_rmses)) if flat_rmses else 0.0
    # I6 + I11: terrain-aware siting against the real DEM. A pad on a crater wall fails even when material
    # is available. dem = (heightmap, cell_m). M11: the order's LOCAL x,y is anchored to a real DEM site via
    # dem_origin (DEM meters where local (0,0) sits). I11: gate the WHOLE footprint, not just the center cell
    # -- a pad whose centre is flat but whose edge straddles a steep rim must still fail (worst slope over the
    # footprint + the fraction of footprint cells over the threshold are reported as the acceptance check).
    slope_violations = []
    if dem is not None:
        Z, dem_cell = dem
        smap = slope_deg_map(Z, dem_cell)
        Hd, Wd = smap.shape
        ox, oy = dem_origin
        for o in mission.orders:
            half = (math.sqrt(o.footprint_m2) / 2.0) / dem_cell
            cx, cy = (ox + o.x) / dem_cell, (oy + o.y) / dem_cell
            r0, r1 = max(0, int(round(cy - half))), min(Hd, int(round(cy + half)) + 1)
            c0, c1 = max(0, int(round(cx - half))), min(Wd, int(round(cx + half)) + 1)
            if r1 <= r0 or c1 <= c0:
                continue
            patch = smap[r0:r1, c0:c1]
            worst = float(patch.max())
            if worst > max_slope_deg:                          # any cell in the footprint too steep -> reject
                slope_violations.append({"action": o.action, "slope_deg": round(worst, 1),
                                         "frac_over": round(float((patch > max_slope_deg).mean()), 2),
                                         "x": o.x, "y": o.y})
    return {
        "feasible": bool(feasible and mass_conserved and not slope_violations),
        "mass_conserved": bool(mass_conserved),
        "slope_violations": slope_violations,
        "max_slope_deg": float(max_slope_deg),
        "mass_drift_kg": float(drift),
        "planned_cut_kg": float(sum(o.footprint_m2 * o.depth_m * rho_bank for o in cuts)),
        "executed_cut_kg": float(exec_cut),
        "planned_fill_kg": float(sum(o.footprint_m2 * o.depth_m * rho_loose for o in fills)),
        "executed_fill_kg": float(exec_fill),
        "drum_remaining_kg": float(cs.drum_inventory),
        "executed_dig_J": float(exec_cut * DIG_J_PER_KG),
        "grid": {"rows": H, "cols": W, "cell_m": cell_m},
        # P0 as-built acceptance (level-surface check on the executed surface):
        "as_built_on_real_dem": bool(on_real_dem),         # False -> measured on a flat mantle (trivially flat)
        "as_built_flatness_rmse_m": float(as_built_worst),  # worst footprint flatness RMSE
        "as_built_flatness_mean_m": float(as_built_mean),
        "as_built_tol_m": float(accept_flatness_tol_m),
        "as_built_pass": bool(as_built_worst <= accept_flatness_tol_m),
    }


def _dur(s):
    h = s / 3600
    return f"{h:.1f} h" if h < 48 else f"{h/24:.1f} d"


def report(mission, trips, flows, per_trip, tl, totals, out_pdf, out_md, endu=None):
    th = totals["time_s"] / 3600
    with PdfPages(out_pdf) as pdf:
        # PAGE 1 — plan table + material balance + totals
        fig = plt.figure(figsize=(8.5, 11))
        fig.suptitle(f"LUNAR BUILD MISSION PLAN — {mission.name}\n{mission.body.title()} · {mission.date} · "
                     f"cut-fill balanced, optimized sequence", fontsize=13, fontweight="bold")
        ax = fig.add_axes([0.04, 0.46, 0.92, 0.40]); ax.axis("off")
        rows = [["#", "Trip", "kind", "Site (x,y)", "Mass t", "Duration", "Energy (chg)"]]
        for i, pt in enumerate(per_trip, 1):
            tr = pt["trip"]
            e = (tr.get("sinter_e", tr.get("dig_e", 0.0)) + tr.get("haul_m", 0.0)*DRIVE_J_PER_M
                 + tr.get("lift_e", 0.0))
            rows.append([str(i), tr["label"][:34], tr["kind"], f"({tr['site'][0]:.0f},{tr['site'][1]:.0f})",
                         f"{tr['mass']/1000:.2f}", _dur(pt["t_end"]-pt["t_start"]), f"{e/BATTERY_J:.1f}"])
        tab = ax.table(cellText=rows, loc="upper center", cellLoc="center",
                       colWidths=[0.04, 0.36, 0.10, 0.14, 0.10, 0.12, 0.12])
        tab.auto_set_font_size(False); tab.set_fontsize(8); tab.scale(1, 1.5)
        for c in range(len(rows[0])): tab[0, c].set_facecolor("#1c3a6e"); tab[0, c].set_text_props(color="w")
        bal = (f"MATERIAL BALANCE\n  cut {totals['cut_kg']/1000:.1f} t → fill {totals['fill_kg']/1000:.1f} t"
               f"  ·  surplus(spoil) {totals['surplus_kg']/1000:.1f} t  ·  deficit(import) {totals['deficit_kg']/1000:.1f} t"
               f"  ·  sinter {totals['sinter_kg']/1000:.2f} t")
        tot = (f"TOTALS   project {_dur(totals['time_s'])} ({th:.0f} h)   moved {totals['mass_kg']/1000:.1f} t"
               f" ({totals['mass_kg']/DRUM_KG:.0f} drum loads)\n"
               f"         energy {totals['energy_J']/1e6:.1f} MJ ({totals['energy_J']/BATTERY_J:.1f} charges,"
               f" {totals['charges']} recharge stops)   drive {totals['distance_m']/1000:.2f} km\n"
               f"         {totals['drum_cycles']} drum cycles; fill SENSED from motor current "
               f"+/-{RM.FDC_MPE_HALF_FULL*100:.1f}% (>half full), no load cell")
        if endu:
            rng = endu.get("range_slopeslip_km", endu["range_flat_reserve_km"])
            tot += (f"\n         per-sortie range {rng:.0f} km"
                    + (f" (slope+slip @ {endu['work_area_median_slope_deg']:.0f} deg; {endu['range_flat_reserve_km']:.0f} km flat)"
                       if "range_slopeslip_km" in endu else " (flat)"))
            ts = endu.get("timescale")
            if ts:
                d = ts["solar_day_h"]; scale = f"{d/24:.0f} Earth-days" if d >= 48 else f"{d:.0f} h"
                tot += f"\n         1 {ts['day_label']} ~ {scale} ({ts['daylight_h']:.0f} h light)"
            c = endu.get("conops")
            if c:
                tot += (f"\n         ConOps: {c['traverse_km']:.0f} km + {c['regolith_t'][0]:.0f}-{c['regolith_t'][1]:.0f} t / "
                        f"{c['mission_days']:.0f} d -> drive ~{c['drive_packs']:.1f} packs vs dig "
                        f"~{c['dig_packs'][0]:.0f}-{c['dig_packs'][1]:.0f} packs: drums dominate")
        fig.text(0.04, 0.40, bal, fontsize=8, family="monospace", wrap=True,
                 bbox=dict(boxstyle="round", fc="#fff4e6", ec="#cc8a33"))
        fig.text(0.04, 0.26, tot, fontsize=8, family="monospace", wrap=True,
                 bbox=dict(boxstyle="round", fc="#eef3ff", ec="#1c3a6e"))
        fig.text(0.04, 0.07,
                 "Cut-fill balanced (excavated material routed to nearest fill; surplus→spoil, deficit→import). "
                 "Grounded: per-body density/gravity (bodies.json); IPEx — 0.30 m/s, 42 kg/hr dig, 4151 J/kg, "
                 "135 J/m (slip-adjusted on a DEM), 4.79 MJ battery, 30 kg/drum; SINTER 0.92 MJ/kg [CALIB] "
                 "(~220× dig). Recharge 700 W + sinter-head 1000 W are [CALIB]. Pluggable sequencer × objective; "
                 "battery-aware mid-task recharge.", fontsize=7, color="#445", wrap=True)
        pdf.savefig(fig); plt.close(fig)

        # PAGE 2 — battery + speed
        fig, (axb, axs) = plt.subplots(2, 1, figsize=(11, 8.5))
        col = {"dig": "#e07b39", "drive": "#3b82c4", "charge": "#3fa34d", "sinter": "#b5179e"}
        for p in tl:
            axb.plot([p["t0"]/3600, p["t1"]/3600], [p["batt0"]/BATTERY_J*100, p["batt1"]/BATTERY_J*100], color=col[p["kind"]], lw=2)
            axs.plot([p["t0"]/3600, p["t1"]/3600], [p["speed"], p["speed"]], color=col[p["kind"]], lw=2)
        axb.axhline(RESERVE_FRAC*100, ls="--", color="#c33", lw=1)
        axb.set_ylabel("battery %"); axb.set_title("Battery draw over the planned project"); axb.set_ylim(0, 105); axb.grid(alpha=.3)
        axb.legend(handles=[plt.Line2D([], [], color=c, lw=3, label=k) for k, c in col.items()], loc="upper right", fontsize=8)
        axs.set_ylabel("speed m/s"); axs.set_xlabel("mission time (hours)"); axs.set_title("Speed profile"); axs.grid(alpha=.3)
        fig.tight_layout(); pdf.savefig(fig); plt.close(fig)

        # PAGE 3 — route + flows + per-task
        fig = plt.figure(figsize=(11, 8.5)); axm = fig.add_axes([0.06, 0.10, 0.42, 0.80])
        axm.plot(*mission.charger, "s", color="#3fa34d", ms=12, label="charger/base")
        for i, pt in enumerate(per_trip, 1):
            s = pt["trip"]["site"]; axm.plot(s[0], s[1], "o", color=col.get(pt["trip"]["kind"], "#e07b39"), ms=11)
            axm.annotate(str(i), s, fontsize=8, fontweight="bold", ha="center", va="center", color="w")
        for co, fo, mass, d in flows:                  # cut->fill material flows (skip spoil dig-in-place)
            if co is not None and fo is not None:
                axm.annotate("", xy=(fo.x, fo.y), xytext=(co.x, co.y),
                             arrowprops=dict(arrowstyle="->", color="#cc8a33", lw=1.4, alpha=.8))
        axm.set_title("Site route + material flows (cut→fill)"); axm.set_xlabel("x (m)"); axm.set_ylabel("y (m)")
        axm.legend(fontsize=8); axm.grid(alpha=.3); axm.set_aspect("equal", adjustable="datalim")
        axt = fig.add_axes([0.58, 0.10, 0.38, 0.34])
        labels = [str(i+1) for i in range(len(per_trip))]
        axt.bar(labels, [(p["trip"].get("sinter_e", p["trip"].get("dig_e", 0))) / BATTERY_J for p in per_trip],
                color=[col.get(p["trip"]["kind"], "#e07b39") for p in per_trip])
        axt.set_title("Energy per trip (battery charges)"); axt.set_xlabel("trip #"); axt.grid(alpha=.3, axis="y")
        axc = fig.add_axes([0.58, 0.56, 0.38, 0.34]); cm = ce = 0.0; tt = [0]; mm = [0]; ee = [0]
        for p in tl:
            cm += p["mass"]; ce += (p["batt0"]-p["batt1"]) if p["kind"] != "charge" else 0
            tt.append(p["t1"]/3600); mm.append(cm/1000); ee.append(ce/1e6)
        axc.plot(tt, mm, color="#e07b39", label="t moved"); axc2 = axc.twinx(); axc2.plot(tt, ee, color="#3b82c4")
        axc.set_title("Cumulative progress"); axc.set_xlabel("hours"); axc.set_ylabel("t", color="#e07b39")
        axc2.set_ylabel("MJ", color="#3b82c4"); axc.grid(alpha=.3)
        pdf.savefig(fig); plt.close(fig)

    # markdown
    md = [f"# Lunar Build Mission Plan — {mission.name}", "",
          f"**Body:** {mission.body.title()} · **Date:** {mission.date} · cut-fill balanced · "
          f"sequence **{totals.get('algorithm', 'nearest')}** optimizing **{totals.get('objective', 'time')}**", "",
          "## Sequence",
          "| # | Trip | kind | Site (x,y) | Mass t | Duration | Energy (chg) |",
          "|---|------|------|-----------|--------|----------|--------------|"]
    for i, pt in enumerate(per_trip, 1):
        tr = pt["trip"]
        e = tr.get("sinter_e", tr.get("dig_e", 0)) + tr.get("haul_m", 0)*DRIVE_J_PER_M + tr.get("lift_e", 0)
        md.append(f"| {i} | {tr['label']} | {tr['kind']} | ({tr['site'][0]:.0f},{tr['site'][1]:.0f}) | "
                  f"{tr['mass']/1000:.2f} | {_dur(pt['t_end']-pt['t_start'])} | {e/BATTERY_J:.1f} |")
    md += ["", "## Material balance",
           f"- cut **{totals['cut_kg']/1000:.1f} t** → fill **{totals['fill_kg']/1000:.1f} t** · "
           f"surplus(spoil) {totals['surplus_kg']/1000:.1f} t · deficit(import) {totals['deficit_kg']/1000:.1f} t · "
           f"sinter {totals['sinter_kg']/1000:.2f} t", "", "## Totals",
           f"- Project time **{_dur(totals['time_s'])}** ({th:.0f} h) · moved **{totals['mass_kg']/1000:.1f} t** "
           f"({totals['mass_kg']/DRUM_KG:.0f} drum loads)",
           f"- Energy **{totals['energy_J']/1e6:.1f} MJ** = {totals['energy_J']/BATTERY_J:.1f} charges "
           f"({totals['charges']} recharge stops) · drive {totals['distance_m']/1000:.2f} km"
           + (f" · incl. **{totals['lift_energy_J']/1e6:.2f} MJ** lifting regolith uphill (exact m·g·Δh, real DEM)"
              if totals.get("lift_energy_J", 0) > 0 else ""),
           (f"- Survival/idle power **{totals['survival_energy_J']/1e6:.1f} MJ** over the sortie "
            f"(@ {totals['idle_power_w']:.0f} W continuous) **[ASSUMPTION]** -- folded into the total above"
            if totals.get("survival_energy_J", 0) > 0 else
            "- Survival/idle power **not modelled** (active legs only; set IDLE_POWER_W to include the "
            "continuous heater/avionics load, the likely-dominant multi-day term) **[ASSUMPTION]**"),
           f"- **{totals['drum_cycles']} drum cycles** (offload events); drum fill SENSED from motor current "
           f"(no load cell, ICE-RASSOR NTRS 20210022781) -- known to ±{RM.FDC_MPE_HALF_FULL*100:.1f}% when "
           f">half full, ±{RM.FDC_MPE_ALL*100:.1f}% below; rover offloads at the upper confidence bound"]
    if totals.get("routed_haul"):                       # I10: hauls routed around real-DEM hazards
        md.append(
            f"- Hauls **routed around hazards** on the real Haworth slope costmap (traverse cap "
            f"{totals['traverse_cap_deg']:.0f}°): **+{totals['haul_detour_frac']*100:.1f}% detour** over straight "
            f"lines" + (f"; ⚠ **{totals['blocked_legs']} leg(s) had no safe corridor** (straight-line fallback, "
                        "flagged)" if totals['blocked_legs'] else ""))
    if endu:                                            # single-charge SORTIE range (not a mission limit)
        line = (f"- **Per-sortie range:** {endu['range_flat_reserve_km']:.1f} km flat to reserve "
                f"({endu['range_flat_full_km']:.1f} km full pack, {endu['duration_flat_h']:.0f} h driving at "
                f"{endu['speed_ms']:.2f} m/s)")
        if "range_slopeslip_km" in endu:
            line += (f"; **{endu['range_slopeslip_km']:.1f} km** slope+slip-adjusted at the work-area median "
                     f"{endu['work_area_median_slope_deg']:.0f}° slope")
        md.append(line)
        if "reach" in endu:
            r = endu["reach"]
            md.append("- One-charge reach on this DEM: " + (
                f"**entire {r['radius_m']/1000:.1f} km work area** within reach "
                f"(~{r['worst_cell_pack_frac']*100:.0f}% of the pack to the farthest point)"
                if r["tile_fully_reachable"] else f"radius **{r['radius_m']/1000:.1f} km** from the charger"))
        pw = endu.get("power")
        if pw:                                          # #2 per-site power source (PSR tower vs sunlit solar)
            md.append(f"- Power: **{pw['kind'].replace('_', ' ')}** — {pw['availability']}; charge "
                      f"{pw['charge_power_w']:.0f} W (effective {pw['effective_charge_w']:.0f} W @ duty "
                      f"{pw['duty_frac']:.2f})" + (f"; cold-derated pack ×{pw['thermal_derate']:.2f}"
                                                   if pw['thermal_derate'] < 1.0 else ""))
        ts = endu.get("timescale")
        if ts:                                          # body-correct operating timescale (Moon ≠ Mars ≠ ...)
            d = ts["solar_day_h"]
            scale = f"{d/24:.1f} Earth-days" if d >= 48 else f"{d:.1f} h"
            if ts["fits_in_window"]:
                rel = (f"a full-range {ts['sortie_h']:.0f} h sortie fits ~{ts['sorties_per_window']:.0f}× "
                       f"in the ~{ts['op_window_h'][0]:.0f}–{ts['op_window_h'][1]:.0f} h sunlit window")
            else:
                rel = (f"a full-range {ts['sortie_h']:.0f} h sortie spans ~{ts['spans_days']:.1f} {ts['day_label']}s "
                       f"(> the ~{ts['daylight_h']:.0f} h daylight → night pauses)")
            md.append(f"- Timescale ({mission.body.title()}): **1 {ts['day_label']} ≈ {scale}** "
                      f"(~{ts['daylight_h']:.0f} h daylight) — {rel}; range is not window-bound.")
        c = endu.get("conops")
        if c:                                           # [SCHULER24] ConOps reconciliation: drums dominate
            md.append(
                f"- ConOps [SCHULER24, lunar IPEx]: **{c['traverse_km']:.0f} km traverse + {c['regolith_t'][0]:.0f}–"
                f"{c['regolith_t'][1]:.0f} t excavated over {c['mission_days']:.0f} days** → driving "
                f"~{c['drive_energy_MJ']:.1f} MJ (~{c['drive_packs']:.1f} packs) vs digging "
                f"~{c['dig_energy_MJ'][0]:.0f}–{c['dig_energy_MJ'][1]:.0f} MJ (~{c['dig_packs'][0]:.0f}–"
                f"{c['dig_packs'][1]:.0f} packs): **the drums dominate the energy budget** (recharged daily).")
    md += ["",
           "_Grounded (bodies.json + ipex_specs + rassor_mass_model); sinter 0.92 MJ/kg; recharge 700 W + "
           "sinter-head 1000 W are [CALIB]._"]
    with open(out_md, "w") as f:
        f.write("\n".join(md))


def demo_mission():
    return Mission(name="South-Pole Site Development", body="moon", charger=(0, 0), orders=[
        BuildOrder("Level landing pad", "cut", 40, 30, 36, 0.04, "6x6 m"),
        BuildOrder("Grade access road", "cut", 15, 5, 30, 0.02, "15x2 m"),
        BuildOrder("Build blast berm", "fill", 44, 44, 14, 0.10, "from pad cut"),
        BuildOrder("Fill crater dip", "fill", -20, 30, 8, 0.08, "from road cut"),
        # Sinter is GATED OFF (energy/density [CALIB], not IPEx-grounded). Re-add once SINTER_ENABLED:
        #   BuildOrder("Sinter pad apron", "sinter", 40, 30, 9, 0.01, "fuse landing surface"),
    ])


def run(mission: Mission, stem=None, *, dem=None, dem_origin=(0.0, 0.0), max_traverse_slope_deg=25.0,
        algorithm="nearest", objective="time", vehicles=1, result=None):
    """Plan + simulate + render the report. ``stem`` names the output files (default = the date); the
    server passes a unique per-mission stem so concurrent plans don't overwrite each other. When ``dem``
    is supplied (server passes the real Haworth DEM for Moon), hauls are I10-routed around hazards.
    ``algorithm`` x ``objective`` select the pluggable sequencer + optimization metric. ``vehicles`` > 1
    plans a multi-vehicle fleet (plan_multi). ``result`` reuses a shared PlanResult (RB-03; no recompute)."""
    if result is None:
        result = plan(mission, dem=dem, dem_origin=dem_origin,
                      max_traverse_slope_deg=max_traverse_slope_deg,
                      algorithm=algorithm, objective=objective, vehicles=vehicles)
    trips, flows, per_trip, tl, totals = result.as_tuple()
    rdir = config.reports_dir()                         # PO-02: configurable app-data dir (where the server serves from)
    os.makedirs(rdir, exist_ok=True)
    stem = stem or f"{mission.date}_mission_plan"
    pdf = os.path.join(rdir, f"{stem}.pdf")
    md = os.path.join(rdir, f"{stem}.md")
    report(mission, trips, flows, per_trip, tl, totals, pdf, md,
           endu=endurance(mission, dem=dem, dem_origin=dem_origin))
    return pdf, md, totals


def main():
    m = demo_mission(); pdf, md, totals = run(m)
    print(f"trips: {[t['label'] for t in plan_and_simulate(m)[0]]}")
    print(f"balance: cut {totals['cut_kg']/1000:.1f} t -> fill {totals['fill_kg']/1000:.1f} t, "
          f"surplus {totals['surplus_kg']/1000:.1f} t, deficit {totals['deficit_kg']/1000:.1f} t, "
          f"sinter {totals['sinter_kg']/1000:.2f} t")
    print(f"totals: {_dur(totals['time_s'])}, {totals['energy_J']/1e6:.1f} MJ, "
          f"{totals['energy_J']/BATTERY_J:.1f} charges, {totals['charges']} recharges")
    print(f"report -> {pdf}")


if __name__ == "__main__":
    main()
