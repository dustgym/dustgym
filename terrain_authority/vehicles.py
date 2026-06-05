"""Vehicle / power-source / tool registries (extensibility — PRD O4 + MV6).

Mirrors the Body/BODIES pattern in bodies.py: a frozen spec + a registry, so adding a vehicle, a power
source, or a tool is ONE entry rather than a global edit. Three design rules baked in here:

  * **Power is a separate entity.** A ``PowerSource`` is its own spec; a vehicle does NOT own its power.
    The assignment is an N:N ``PowerGrid`` over fleet-instance names (a source can serve many vehicles;
    a vehicle can draw from many sources).
  * **Tools are separate entities.** The sinter head is a ``Tool`` that GRANTS the ``sinter`` capability;
    it is NOT a capability of the IPEx drum excavator. A bare ``ipex`` cannot sinter; an ``ipex`` with
    the sinter tool mounted can.
  * **The ``.py`` is the single source of truth** ("everything stays .py"): every default is here with a
    provenance tag, like bodies.py. JSON is only ever a GENERATED export for the browser (gen_bodies_json),
    never the source.

Phase A is purely additive (nothing imports this yet but the tests). Threading ``vehicle=`` / the grid
through the drive/env/planner chain (like ``body=``) and capability-gating actions are the next phases.
"""
from __future__ import annotations

import dataclasses

from . import bodies as B          # body registry: get_body / params_for_body (for body assignment)
from . import constants as C
from . import ipex_specs as S

# The action vocabulary (the verbs the controller seam / planner / envs speak). A vehicle's `capabilities`
# is a subset; a Tool grants one. Kept here so "what actions exist" has a single home.
ACTIONS = frozenset({"drive", "excavate", "haul", "dump", "compact", "grade", "fill", "sinter", "process"})


@dataclasses.dataclass(frozen=True)
class PowerSource:
    """An energy source, independent of any vehicle. ``capacity_j``=0 marks a continuous-only source
    (RTG / tether / surface tower); ``recharge_w``=0 marks a non-rechargeable source."""
    name: str
    label: str
    kind: str                          # "battery" | "rtg" | "fuel_cell" | "tether" | "solar" | "tower"
    capacity_j: float                  # stored energy [J] (0 = continuous-only)
    recharge_w: float = 0.0            # rechargeable input power [W] (0 = not rechargeable)
    continuous_w: float = 0.0          # always-on output [W] (RTG/tether/tower; also a home for idle/survival draw, K11)
    mass_kg: float | None = None
    provenance: str = ""


@dataclasses.dataclass(frozen=True)
class Tool:
    """A separate implement that GRANTS a capability when mounted on a vehicle (e.g., the sinter head).
    Carries the tool's own [CALIB] cost numbers so they are not baked into a vehicle that lacks it."""
    name: str
    label: str
    capability: str                    # the ACTION it grants (must be in ACTIONS)
    energy_j_per_kg: float = 0.0
    product_density_kg_m3: float | None = None
    provenance: str = ""


@dataclasses.dataclass(frozen=True)
class Vehicle:
    """A mobile platform spec. Power is NOT owned here (see PowerGrid); ``onboard_power`` only names the
    source(s) the vehicle carries by default. ``capabilities`` is the base action set (tools add more)."""
    name: str
    label: str
    dry_mass_kg: float
    n_wheels: int
    wheel_width_m: float
    contact_len_m: float
    drum_capacity_kg: float            # 0 for non-excavators (haulers, the sinter rig)
    drive_power_w: float
    dig_energy_j_per_kg: float         # 0 if it cannot dig
    capabilities: frozenset
    onboard_power: tuple = ()          # default PowerSource name(s) carried onboard
    provenance: str = ""


# ---- the registries (the .py source of truth) ---------------------------------------------------
POWER_SOURCES = {
    "ipex_battery": PowerSource(
        "ipex_battery", "IPEx 12S/30Ah Li-ion", "battery",
        capacity_j=S.battery_energy_j(), recharge_w=S.RECHARGE_POWER_W,
        provenance="ipex_specs.py (NTRS 20240008162) + 12S/30Ah pack; recharge_w [CALIB]."),
    "lander_tower": PowerSource(
        "lander_tower", "Surface power tower / lander", "tower",
        capacity_j=0.0, recharge_w=0.0, continuous_w=S.RECHARGE_POWER_W,
        provenance="[ASSUMPTION] a shared surface power station (K8 PSR tower) that can serve N vehicles; "
                   "continuous_w reuses the [CALIB] recharge power, not a new fabricated value."),
}

TOOLS = {
    # sinter is a SEPARATE entity, not a capability of the IPEx excavator (no microwave/laser head on IPEx).
    "sinter": Tool(
        "sinter", "Sinter head (regolith fuser)", "sinter",
        energy_j_per_kg=C.SINTER_ENERGY_J_PER_KG, product_density_kg_m3=C.RHO_SINTERED,
        provenance="[CALIB] energy = constants.SINTER_ENERGY_J_PER_KG (thermodynamic floor); product "
                   "density = constants.RHO_SINTERED (microwave-sinter measured). Its host platform's "
                   "mass/power are not sourced -> kept as a tool, not a fabricated vehicle."),
}

VEHICLES = {
    "ipex": Vehicle(
        "ipex", "ISRU Pilot Excavator (IPEx)",
        dry_mass_kg=S.ROVER_MASS_CLASS_KG, n_wheels=S.N_WHEELS,
        wheel_width_m=0.18, contact_len_m=0.10,             # the values the drive chain uses today
        drum_capacity_kg=S.REGOLITH_PER_CYCLE_KG,
        drive_power_w=S.drive_power_w(), dig_energy_j_per_kg=S.dig_energy_per_kg(),
        capabilities=frozenset({"drive", "excavate", "haul", "dump", "compact"}),   # NO sinter
        onboard_power=("ipex_battery",),
        provenance="ipex_specs.py (Schuler et al., IPEx TRL-5, NTRS 20240008162); a RASSOR-lineage drum "
                   "excavator -> no sinter tool on the baseline platform."),
}

DEFAULT_VEHICLE = "ipex"


# ---- lookups (KeyError on unknown, like bodies.get_body) -----------------------------------------
def _get(registry, name, what):
    if name in registry:
        return registry[name]
    key = str(name).strip().lower()
    if key in registry:
        return registry[key]
    raise KeyError(f"unknown {what} {name!r}; known: {sorted(registry)}")


def get_vehicle(name) -> Vehicle:
    return name if isinstance(name, Vehicle) else _get(VEHICLES, name, "vehicle")


def get_power(name) -> PowerSource:
    return name if isinstance(name, PowerSource) else _get(POWER_SOURCES, name, "power source")


def get_tool(name) -> Tool:
    return name if isinstance(name, Tool) else _get(TOOLS, name, "tool")


def capabilities_of(vehicle, tools=()) -> frozenset:
    """A vehicle's EFFECTIVE capabilities: its base set plus the capability each mounted tool grants."""
    v = get_vehicle(vehicle)
    caps = set(v.capabilities)
    for t in tools:
        caps.add(get_tool(t).capability if not isinstance(t, Tool) else t.capability)
    return frozenset(caps)


class PowerGrid:
    """N:N power-source <-> vehicle assignment. ``links`` are ``(power_source_name, vehicle_name)`` edges
    over fleet-INSTANCE names, so one source can serve many vehicles and a vehicle can draw from many
    sources. (Instance names are free-form; types live in the registries above.)"""

    def __init__(self, links):
        self.links = [tuple(e) for e in links]

    def vehicles_for(self, power) -> list:
        return [v for (p, v) in self.links if p == power]

    def sources_for(self, vehicle) -> list:
        return [p for (p, v) in self.links if v == vehicle]


def default_grid(vehicle_names) -> PowerGrid:
    """Wire each named vehicle to its onboard power source(s) — the single-vehicle/no-shared-power default."""
    links = []
    for name in vehicle_names:
        for ps in get_vehicle(name).onboard_power:
            links.append((ps, name))
    return PowerGrid(links)


# ---- deployment: bind vehicles / tools / power to BODIES ----------------------------------------
@dataclasses.dataclass(frozen=True)
class Placement:
    """A deployed vehicle INSTANCE: a vehicle type placed ON A BODY, with tools mounted and power
    assigned. This is how a vehicle / tool / power source gets bound to a particular body — different
    placements can sit on different bodies. ``power`` empty -> the vehicle's onboard source(s)."""
    instance: str                      # fleet-unique instance name (e.g. "rover_1")
    vehicle: str                       # VEHICLES key
    body: str                          # BODIES key (the world it operates on: terrain geometry + gravity)
    tools: tuple = ()                  # TOOLS keys mounted on it
    power: tuple = ()                  # POWER_SOURCES keys (empty -> the vehicle's onboard_power)
    soil: str = ""                     # regolith model override (a BODIES key); "" -> the body's own soil
    g: float | None = None             # gravity override [m/s^2]; None -> the body's own gravity

    def __post_init__(self):
        get_vehicle(self.vehicle)      # validate every reference against the registries
        B.get_body(self.body)
        if self.soil:
            B.get_body(self.soil)      # the soil source is also a body (its regolith)
        for t in self.tools:
            get_tool(t)
        for p in self.power:
            get_power(p)


class Deployment:
    """A fleet across one or more bodies: a set of Placements. Resolves, per instance, the body physics
    (params_for_body), the effective capabilities (base + mounted tools), and the power assignment
    (an N:N PowerGrid). The same vehicle/tool/power type can appear in placements on different bodies."""

    def __init__(self, placements):
        self.placements = list(placements)
        self._by_instance = {p.instance: p for p in self.placements}

    def placement(self, instance) -> Placement:
        if instance not in self._by_instance:
            raise KeyError(f"unknown instance {instance!r}; known: {sorted(self._by_instance)}")
        return self._by_instance[instance]

    def bodies(self) -> set:
        return {B.get_body(p.body).name for p in self.placements}

    def on_body(self, body) -> list:
        b = B.get_body(body).name
        return [p for p in self.placements if B.get_body(p.body).name == b]

    def params_for(self, instance):
        """The TerramechanicsParams (soil/Bekker) for this instance: its `soil` override (e.g. Earth soil
        on a lunar map) or the body's own regolith. Soil and gravity are independent (see gravity_for)."""
        p = self.placement(instance)
        return B.params_for_body(p.soil or p.body)

    def gravity_for(self, instance) -> float:
        """The gravity [m/s^2] for this instance: its `g` override or the body's own. Decoupled from soil,
        so e.g. Earth soil under lunar gravity (body=moon, soil=earth) or Earth gravity on lunar terrain."""
        p = self.placement(instance)
        return float(p.g) if p.g is not None else B.get_body(p.body).g

    def capabilities_for(self, instance) -> frozenset:
        p = self.placement(instance)
        return capabilities_of(p.vehicle, tools=p.tools)

    def power_for(self, instance) -> tuple:
        p = self.placement(instance)
        return tuple(p.power) if p.power else tuple(get_vehicle(p.vehicle).onboard_power)

    def grid(self) -> PowerGrid:
        """The N:N power grid implied by the placements (a source named in several placements serves
        several instances)."""
        links = [(ps, p.instance) for p in self.placements for ps in self.power_for(p.instance)]
        return PowerGrid(links)
