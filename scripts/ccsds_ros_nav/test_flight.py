"""End-to-end flight tests on a small real Haworth crop (no synthetic terrain, no ROS)."""
from __future__ import annotations

import os
import threading

import pytest

import messages
from flight import FlightModel, load_crop
from link import loopback_pair
from route import slope_deg, snap_to_navigable

_SCENE = os.path.join(os.path.dirname(__file__), "..", "..", "samples", "lunar_dem", "haworth_10km_5m")
pytestmark = pytest.mark.skipif(not os.path.isdir(_SCENE), reason="Haworth sample not present")


def _gentle_crop(win=50):
    crop = load_crop(_SCENE, 720, 1800, win, win)
    sl = slope_deg(crop.heightmap, crop.cell_m)
    start = snap_to_navigable(sl, (8, 8), 12.0)
    goal = snap_to_navigable(sl, (24, 24), 12.0)
    return crop, start, goal


def test_execute_goto_reaches_and_conserves_mass():
    crop, start, goal = _gentle_crop()
    fm = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="moon", dt=0.2)
    leg, downlink = fm.execute_goto(messages.GoTo(leg_id=0, goal_row=float(goal[0]),
                                                  goal_col=float(goal[1]), v_max_mps=0.3,
                                                  goal_radius_cells=1.0))
    assert leg.status == messages.LEG_REACHED
    assert leg.achieved_dist_m > 0.0
    assert fm.mass_drift() < 1e-9                       # conserved by construction
    assert downlink and downlink[-1].leg_id == 0
    for p in downlink:
        assert 0.0 <= p.slip < 1.0
        assert p.soc <= 1.0
        assert not p.entrapped


def test_body_gravity_threads_through():
    crop, start, _ = _gentle_crop()
    moon = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="moon")
    earth = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="earth")
    assert moon.g < earth.g                             # 1.62 < 9.81 (bodies.py)


def test_serve_over_loopback_link():
    crop, start, goal = _gentle_crop()
    fm = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="moon", dt=0.2)
    ground, flight = loopback_pair()
    t = threading.Thread(target=fm.serve, args=(flight,), kwargs={"expect_legs": 1}, daemon=True)
    t.start()
    ground.send(messages.encode(messages.GoTo(leg_id=0, goal_row=float(goal[0]),
                goal_col=float(goal[1]), v_max_mps=0.3, goal_radius_cells=1.0), met=0.0))
    saw_leg = None
    for _ in range(5000):
        pkt = ground.recv(timeout=5.0)
        assert pkt is not None, "flight went silent"
        msg = messages.decode(pkt)
        if isinstance(msg, messages.Leg):
            saw_leg = msg
            break
    t.join(timeout=5.0)
    assert saw_leg is not None and saw_leg.status == messages.LEG_REACHED


def test_safe_command_acknowledged():
    crop, start, _ = _gentle_crop()
    fm = FlightModel(crop=crop, start_rc=(float(start[0]), float(start[1])), body="moon")
    ground, flight = loopback_pair()
    t = threading.Thread(target=fm.serve, args=(flight,), kwargs={"expect_legs": 1}, daemon=True)
    t.start()
    ground.send(messages.encode(messages.Safe(reason=1), met=0.0))
    pkt = ground.recv(timeout=5.0)
    assert pkt is not None
    leg = messages.decode(pkt)
    assert isinstance(leg, messages.Leg) and leg.status == messages.LEG_SAFED
    # release the server thread (still waiting for its 1 leg): send a tiny goto
    ground.send(messages.encode(messages.GoTo(leg_id=0, goal_row=float(start[0]),
                goal_col=float(start[1]), v_max_mps=0.3, goal_radius_cells=2.0), met=0.0))
    t.join(timeout=5.0)
