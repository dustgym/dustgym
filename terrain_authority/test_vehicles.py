"""Vehicle / power-source / tool registries (extensibility — PRD O4 + MV6).

Mirrors the Body/BODIES pattern. Three constraints this locks in:
  * power is a SEPARATE entity from the vehicle, wired by an N:N PowerGrid (a source serves many
    vehicles; a vehicle draws from many sources),
  * the sinter head is its OWN Tool, NOT a capability of the IPEx drum excavator,
  * the `ipex` entry reproduces the ipex_specs.py numbers exactly (single source of truth, no drift).
"""
import math

import pytest

from terrain_authority import constants as K
from terrain_authority import ipex_specs as S
from terrain_authority import rover
from terrain_authority import vehicles as V


def test_both_bodies_registered_with_geometry():
    # both selectable vehicles exist, each carrying its own geometry (gauge/wheelbase/wheel/CG)
    assert "ipex" in V.VEHICLES and "ez_rassor" in V.VEHICLES
    for name in ("ipex", "ez_rassor"):
        v = V.get_vehicle(name)
        for f in ("gauge_m", "wheelbase_m", "wheel_radius_m", "cg_height_m", "render_assets"):
            assert hasattr(v, f), (name, f)


def test_ez_rassor_geometry_equals_the_render_globals():
    # ez_rassor IS the EZ-RASSOR URDF the render + the rover.py globals describe (byte-identical default)
    v = V.get_vehicle("ez_rassor")
    assert math.isclose(v.gauge_m, rover.WHEEL_GAUGE_M)
    assert math.isclose(v.wheelbase_m, rover.WHEEL_BASE_M)
    assert math.isclose(v.wheel_radius_m, rover.WHEEL_RADIUS_M)
    assert math.isclose(v.cg_height_m, K.CG_HEIGHT_M)
    assert v.render_assets == ""                                    # "" -> the default godot_sidecar/assets/


def test_ipex_geometry_is_flight_scale():
    v = V.get_vehicle("ipex")
    assert math.isclose(v.wheel_radius_m, S.WHEEL_RADIUS_M)         # 0.1524 m, sourced (30.5 cm dia)
    assert math.isclose(v.gauge_m, round(0.7 * S.SKID_STEER_TRACK_M, 4))   # 0.7 x RASSOR-2 track
    # the flight IPEx body is narrower + shorter than the EZ-RASSOR demo robot
    ez = V.get_vehicle("ez_rassor")
    assert v.gauge_m < ez.gauge_m and v.wheelbase_m < ez.wheelbase_m and v.wheel_radius_m < ez.wheel_radius_m
    assert v.render_assets == "ipex"                               # the CC0 self-authored parts


def test_geometry_of_returns_stability_kwargs_and_differs_per_vehicle():
    from terrain_authority import stability as ST
    g_ipex = V.geometry_of("ipex")
    g_ez = V.geometry_of("ez_rassor")
    assert set(g_ipex) >= {"gauge_m", "wheelbase_m", "cg_height_m"}
    # the dict drops straight into stability.stability(); the two bodies give different tip limits
    lim_ipex = ST.tip_tilt_limit_deg(gauge_m=g_ipex["gauge_m"], wheelbase_m=g_ipex["wheelbase_m"],
                                     cg_height_m=g_ipex["cg_height_m"])
    lim_ez = ST.tip_tilt_limit_deg(gauge_m=g_ez["gauge_m"], wheelbase_m=g_ez["wheelbase_m"],
                                   cg_height_m=g_ez["cg_height_m"])
    assert lim_ipex != lim_ez                                       # distinct physics models, selectable


def test_ipex_vehicle_matches_the_ipex_specs_globals():
    v = V.get_vehicle("ipex")
    assert math.isclose(v.dry_mass_kg, S.ROVER_MASS_CLASS_KG)
    assert v.n_wheels == S.N_WHEELS
    assert math.isclose(v.drum_capacity_kg, S.REGOLITH_PER_CYCLE_KG)
    assert math.isclose(v.drive_power_w, S.drive_power_w())
    assert math.isclose(v.dig_energy_j_per_kg, S.dig_energy_per_kg())


def test_sinter_is_a_separate_tool_not_an_ipex_capability():
    ipex = V.get_vehicle("ipex")
    assert "sinter" not in ipex.capabilities                       # NOT on the current vehicle
    assert {"drive", "excavate", "haul", "dump", "compact"} <= ipex.capabilities
    sinter = V.get_tool("sinter")
    assert sinter.capability == "sinter"                            # sinter is its own entity
    # mounting the tool grants the capability; the bare vehicle still lacks it
    assert "sinter" in V.capabilities_of(ipex, tools=[sinter])
    assert "sinter" not in V.capabilities_of(ipex)


def test_power_sources_grounded():
    b = V.get_power("ipex_battery")
    assert b.kind == "battery" and math.isclose(b.capacity_j, S.battery_energy_j())
    tower = V.get_power("lander_tower")
    assert tower.continuous_w > 0.0 and tower.capacity_j == 0.0     # continuous-only shared source


def test_power_grid_is_n_to_n():
    # one source serves many vehicles (N:1), and a vehicle draws from many sources (1:N) -> N:N
    grid = V.PowerGrid([("lander_tower", "rover_a"), ("lander_tower", "rover_b"),
                        ("rover_c_batt", "rover_c"), ("lander_tower", "rover_c")])
    assert set(grid.vehicles_for("lander_tower")) == {"rover_a", "rover_b", "rover_c"}   # N:1
    assert set(grid.sources_for("rover_c")) == {"rover_c_batt", "lander_tower"}          # 1:N
    assert grid.sources_for("rover_a") == ["lander_tower"]


def test_default_grid_wires_onboard_power():
    grid = V.default_grid(["ipex"])                                 # a single ipex on its onboard battery
    assert "ipex_battery" in grid.sources_for("ipex")


def test_registries_extensible_and_guarded():
    assert "ipex" in V.VEHICLES and "ipex_battery" in V.POWER_SOURCES and "sinter" in V.TOOLS
    assert V.DEFAULT_VEHICLE == "ipex"
    for getter in (V.get_vehicle, V.get_power, V.get_tool):
        with pytest.raises(KeyError):
            getter("does-not-exist-9000")


# ---- deployment: vehicles / tools / power assigned to bodies -------------------------------------
def test_placement_binds_vehicle_tool_power_to_a_body():
    p = V.Placement("rover_1", "ipex", "mars", tools=("sinter",), power=("ipex_battery",))
    assert p.body == "mars"
    with pytest.raises(KeyError):
        V.Placement("bad", "ipex", "pluto")                        # unknown body rejected
    with pytest.raises(KeyError):
        V.Placement("bad", "spaceship", "moon")                    # unknown vehicle rejected


def test_deployment_spans_bodies_with_body_correct_physics():
    dep = V.Deployment([
        V.Placement("luna_1", "ipex", "moon"),
        V.Placement("ares_1", "ipex", "mars", tools=("sinter",)),
    ])
    assert dep.bodies() == {"moon", "mars"}                         # same vehicle type, different bodies
    assert [pl.instance for pl in dep.on_body("mars")] == ["ares_1"]
    # body-correct physics: per-placement params come from the body's sourced soil -> Mars cohesion
    # (~1000 Pa) differs from the Moon (~170 Pa), so the two placements get different terramechanics
    assert dep.params_for("ares_1").cohesion > dep.params_for("luna_1").cohesion
    # the Mars rover carries the sinter tool -> gains the capability there, the Moon one does not
    assert "sinter" in dep.capabilities_for("ares_1")
    assert "sinter" not in dep.capabilities_for("luna_1")


def test_deployment_grid_is_n_to_n_across_a_shared_tower():
    dep = V.Deployment([
        V.Placement("r1", "ipex", "moon", power=("lander_tower",)),
        V.Placement("r2", "ipex", "moon", power=("lander_tower", "ipex_battery")),
    ])
    g = dep.grid()
    assert set(g.vehicles_for("lander_tower")) == {"r1", "r2"}      # one tower serves both (N:1)
    assert set(g.sources_for("r2")) == {"lander_tower", "ipex_battery"}   # r2 draws from both (1:N)
    # a placement with no explicit power falls back to the vehicle's onboard source
    solo = V.Deployment([V.Placement("r3", "ipex", "moon")])
    assert solo.power_for("r3") == ("ipex_battery",)
