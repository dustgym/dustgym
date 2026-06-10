"""HITL console tests: latency model (unit) + a host-only point-and-click round-trip (no ROS, no GPU).

The round-trip runs a real flight executive over a UDP link as the 'rover' and drives the console via
its FastAPI app, exercising route-on-click -> CCSDS command -> drive -> delayed telemetry -> belief
update. Camera capture (GPU/Godot) is not exercised here.
"""
from __future__ import annotations

import os
import threading
import time

import pytest

import console_server as cs

_SCENE = os.path.join(os.path.dirname(__file__), "..", "..", "samples", "lunar_dem", "haworth_10km_5m")
pytestmark = pytest.mark.skipif(not os.path.isdir(_SCENE), reason="Haworth sample not present")


def test_latency_model_scales_with_time_factor():
    try:
        c = cs.Console(haworth=_SCENE, r0=720, c0=1800, win=40, bridge_host="127.0.0.1",
                       bridge_port=52041, local_port=52040, round_trip_s=10.0, time_factor=1.0,
                       sun_el=3.0, v_max=0.3, max_slope_deg=14.0)
    except OSError:
        pytest.skip("UDP unavailable")
    try:
        assert c._one_way_wall() == pytest.approx(5.0)        # rt/2 at 1x
        c.set_config(time_factor=10.0)
        assert c._one_way_wall() == pytest.approx(0.5)        # rt/2 / 10x
        assert c.link.light_time_s == pytest.approx(0.5)
        c.set_config(round_trip_s=0.0)                        # training mode: no delay
        assert c._one_way_wall() == pytest.approx(0.0)
    finally:
        c._stop.set()
        c.link.close()


def test_state_exposes_heading_and_coach_surface():
    """New operator-aid surface (no GPU): heading for the map arrow, cam-pitch config, coach-view
    state field + endpoint registration. The actual chase render needs Godot, so it is not invoked."""
    from fastapi.testclient import TestClient

    try:
        c = cs.Console(haworth=_SCENE, r0=720, c0=1800, win=40, bridge_host="127.0.0.1",
                       bridge_port=52045, local_port=52044, round_trip_s=8.0, time_factor=1.0,
                       sun_el=3.0, v_max=0.3, max_slope_deg=14.0, cam_pitch_deg=10.0)
    except OSError:
        pytest.skip("UDP unavailable")
    cs.CONSOLE = c
    try:
        client = TestClient(cs.app)
        st = client.get("/state").json()
        for k in ("belief_yaw", "chase_seq", "cam_pitch_deg"):
            assert k in st, f"state missing {k}"
        assert st["cam_pitch_deg"] == pytest.approx(10.0)
        assert st["chase_seq"] == 0                                   # nothing rendered yet
        # cam-pitch is live-tunable from the camera page slider.
        client.post("/config", json={"cam_pitch_deg": 18.0})
        assert c.cam_pitch_deg == pytest.approx(18.0)
        # the coach view + its capture endpoint are registered; un-rendered chase serves 404.
        assert client.get("/chase").status_code == 404
        routes = {r.path for r in cs.app.routes}
        assert {"/chase", "/capture_chase"} <= routes
        # the map page draws a heading arrow (not a bare dot).
        assert "function arrow(" in cs._MAP_HTML
        assert "third-person coach view" in cs._CAM_HTML
    finally:
        c._stop.set()
        c.link.close()


def test_point_and_click_roundtrip_host_only():
    from fastapi.testclient import TestClient

    from flight import FlightModel, load_crop
    from link import UdpLink
    from route import slope_deg, snap_to_navigable

    crop = load_crop(_SCENE, 720, 1800, 50, 50)
    sl = slope_deg(crop.heightmap, crop.cell_m)
    start = snap_to_navigable(sl, (10, 10), 14.0)
    fm = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="moon", dt=0.2)
    try:
        rover = UdpLink(("127.0.0.1", 52043), ("127.0.0.1", 52042))           # the 'rover' end
        console = cs.Console(haworth=_SCENE, r0=720, c0=1800, win=50, bridge_host="127.0.0.1",
                             bridge_port=52043, local_port=52042, round_trip_s=0.0, time_factor=1.0,
                             sun_el=3.0, v_max=0.3, max_slope_deg=14.0)
    except OSError:
        pytest.skip("UDP unavailable")
    cs.CONSOLE = console
    t = threading.Thread(target=fm.serve, args=(rover,), kwargs={"expect_legs": None, "idle_timeout": 4.0},
                         daemon=True)
    t.start()
    try:
        client = TestClient(cs.app)
        assert client.get("/state").json()["win"] == 50
        assert client.get("/map.png").content[:8] == b"\x89PNG\r\n\x1a\n"     # real PNG

        before = tuple(console._belief_rc)
        r = client.post("/goal", json={"u": 38 / 49, "v": 38 / 49}).json()
        assert r["ok"] and r["waypoints"]

        saw_leg = False
        for _ in range(120):
            time.sleep(0.1)
            st = client.get("/state").json()
            if st["last_leg"] is not None:
                saw_leg = True
            if saw_leg and st["queue_len"] == 0:
                break
        assert saw_leg, "no leg telemetry returned to the console"
        assert tuple(console._belief_rc) != before, "rover did not move"
        client.post("/stop")
    finally:
        console._stop.set()
        rover.close()
        console.link.close()
