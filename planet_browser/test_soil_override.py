"""Soil override: assign one body's regolith (e.g. Earth dry-sand) to another body's map/gravity.

Map, soil, and gravity are independent. A `soil` override swaps the Bekker/cohesion model the drive
physics uses while gravity stays the body's own -- so you can run Earth terramechanics on a lunar map.
"""
import pytest

from planet_browser import mission_planner as MP


def _payload(soil=None):
    p = {"name": "t", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 60, "y": 0, "footprint_m2": 36, "depth_m": 0.05},
        {"action": "fill", "kind": "fill", "x": -60, "y": 0, "footprint_m2": 16, "depth_m": 0.08}]}
    if soil is not None:
        p["soil"] = soil
    return p


def test_soil_defaults_to_the_body_and_validates():
    m = MP.mission_from_dict(_payload())
    assert m.soil == "" and MP.mission_soil_params(m).k_c == MP.mission_soil_params(MP.mission_from_dict(_payload("moon"))).k_c
    with pytest.raises(ValueError):
        MP.mission_from_dict(_payload("atlantis"))                 # unknown soil body -> 400


def test_earth_soil_on_a_lunar_mission_changes_the_haul_energy():
    moon = MP.mission_from_dict(_payload())                        # lunar map + lunar soil
    earth = MP.mission_from_dict(_payload("earth"))                # lunar map + EARTH soil (gravity stays lunar)
    assert earth.soil == "earth"
    # the soil model feeds the haul slip -> different soil yields different planned haul energy
    e_moon = sum(t.get("haul_e", 0.0) for t in MP._build_trips(moon, None, (0.0, 0.0), 25.0)[0])
    e_earth = sum(t.get("haul_e", 0.0) for t in MP._build_trips(earth, None, (0.0, 0.0), 25.0)[0])
    assert e_moon > 0 and e_earth > 0 and e_moon != e_earth
    # gravity is unchanged by the soil swap (still the body's)
    assert MP.body_gravity(earth.body) == MP.body_gravity(moon.body)


def test_soil_override_flows_through_the_server():
    from fastapi.testclient import TestClient

    from planet_browser import server as SRV
    c = TestClient(SRV.app)
    assert c.post("/plan", json=_payload("earth")).status_code == 200      # Earth soil on the lunar Haworth plan
    bad = c.post("/plan", json=_payload("atlantis"))
    assert bad.status_code == 400


def test_roversim_env_soil_override_changes_sinkage():
    from terrain_authority import drive as D
    from terrain_authority.column_state import ColumnState
    from terrain_authority.rover_env import RoverSimEnv
    moon_env = RoverSimEnv(body="moon")
    earth_on_moon = RoverSimEnv(body="moon", soil="earth")         # Earth soil, lunar gravity
    assert moon_env.g == earth_on_moon.g                           # gravity unchanged by the soil swap
    assert moon_env.params_base.k_phi != earth_on_moon.params_base.k_phi   # the regolith model differs

    def sink(params, g):
        cs = ColumnState(width=48, height=48, cell_m=0.05)
        return D.drive_step(cs, (24.0, 24.0), 0.0, 0.2, 0.0, params=params, g=g)[2]["sinkage_m"]

    assert sink(moon_env.params_base, moon_env.g) != sink(earth_on_moon.params_base, earth_on_moon.g)
