"""M11: a globe lat/lon pick projects to the Haworth DEM order-frame origin (south-polar stereographic,
IAU_2015:30135), so the plan is sited where the user clicked instead of the auto flattest anchor."""
import json
import math
import os

import pytest

pytest.importorskip("pyproj")
from pyproj import CRS, Transformer  # noqa: E402

from planet_browser import mission_planner as MP  # noqa: E402

BUNDLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "samples", "lunar_dem", "haworth_10km_5m")
_have = os.path.isdir(BUNDLE)


def _cell_latlon(ri, ci):
    """The selenographic lat/lon of DEM cell (ri, ci) -- inverse of latlon_to_dem_origin's projection."""
    meta = json.load(open(os.path.join(BUNDLE, "metadata.json")))
    g, b = meta["grid"], meta["world_bounds_m"]
    cell = g["cell_m"]
    ax0, ay0 = b["x0"] + cell / 2.0, b["y1"] - cell / 2.0
    xs, ys = ax0 + ci * cell, ay0 - ri * cell
    crs = CRS.from_user_input("IAU_2015:30135")
    inv = Transformer.from_crs(crs, crs.geodetic_crs, always_xy=True)
    lon, lat = inv.transform(xs, ys)
    return lat, lon, cell


@pytest.mark.skipif(not _have, reason="Haworth bundle absent")
def test_latlon_to_dem_origin_round_trips_to_the_cell():
    ri, ci = 800, 1200                                     # a known interior cell
    lat, lon, cell = _cell_latlon(ri, ci)
    ox, oy = MP.latlon_to_dem_origin(lat, lon)
    assert math.isclose(ox, ci * cell, abs_tol=cell) and math.isclose(oy, ri * cell, abs_tol=cell)


@pytest.mark.skipif(not _have, reason="Haworth bundle absent")
def test_off_tile_latlon_raises():
    with pytest.raises(ValueError, match="outside"):
        MP.latlon_to_dem_origin(0.0, 0.0)                  # the equator is nowhere near the south-pole tile


@pytest.mark.skipif(not _have, reason="Haworth bundle absent")
def test_server_plan_accepts_in_tile_pick_and_rejects_off_tile():
    from fastapi.testclient import TestClient

    from planet_browser import server as SRV
    c = TestClient(SRV.app)
    lat, lon, _ = _cell_latlon(1000, 1000)                 # tile centre
    order = [{"action": "cut", "kind": "cut", "x": 0, "y": 0, "footprint_m2": 9, "depth_m": 0.04}]
    ok = c.post("/plan", json={"name": "sited", "body": "moon", "charger": [0, 0], "orders": order,
                               "lat": lat, "lon": lon})
    assert ok.status_code == 200 and ok.json()["ok"] is True
    off = c.post("/plan", json={"name": "off", "body": "moon", "charger": [0, 0], "orders": order,
                                "lat": 0.0, "lon": 0.0})
    assert off.status_code == 400 and "outside" in off.json()["error"]
