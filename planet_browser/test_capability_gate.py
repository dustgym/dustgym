"""Phase B: the planner capability-gates order kinds by the selected vehicle + mounted tools.

cut needs `excavate`, fill needs `dump`, sinter needs `sinter`. The default IPEx drum excavator has
excavate+dump (so cut/fill work) but NOT sinter -- sinter is a separate Tool. Mounting it satisfies
the capability gate.
"""
import pytest

from planet_browser import mission_planner as MP


def _payload(kind, vehicle=None, tools=None):
    o = {"action": f"{kind} job", "kind": kind, "x": 5, "y": 5, "footprint_m2": 9, "depth_m": 0.05}
    p = {"name": "t", "body": "moon", "charger": [0, 0], "orders": [o]}
    if vehicle is not None:
        p["vehicle"] = vehicle
    if tools is not None:
        p["tools"] = tools
    return p


def test_ipex_can_cut_and_fill():
    for k in ("cut", "fill"):
        m = MP.mission_from_dict(_payload(k))          # ipex has excavate + dump -> accepted
        assert m.orders[0].kind == k and m.vehicle == "ipex"


def test_ipex_cannot_sinter_without_the_tool():
    with pytest.raises(ValueError, match="GATED OFF"):
        MP.mission_from_dict(_payload("sinter"))       # drum excavator, no sinter tool


def test_mounting_the_sinter_tool_passes_the_capability_gate():
    # with the separate sinter Tool mounted, the capability gate is satisfied and the mission builds
    m = MP.mission_from_dict(_payload("sinter", tools=["sinter"]))
    assert m.orders[0].kind == "sinter" and "sinter" in m.tools


def test_unknown_vehicle_or_tool_is_a_value_error_not_a_crash():
    with pytest.raises(ValueError):
        MP.mission_from_dict(_payload("cut", vehicle="bulldozer9000"))
    with pytest.raises(ValueError):
        MP.mission_from_dict(_payload("cut", tools=["jackhammer"]))


def test_server_forwards_vehicle_and_tools_to_the_gate():
    # the ASGI server passes the browser's selected vehicle/tools through to the capability gate
    from fastapi.testclient import TestClient

    from planet_browser import server as SRV
    c = TestClient(SRV.app)
    ok = c.post("/plan", json=_payload("cut", vehicle="ipex", tools=[]))
    assert ok.status_code == 200 and ok.json()["ok"] is True
    bad = c.post("/plan", json=_payload("cut", vehicle="bulldozer9000"))
    assert bad.status_code == 400 and "bulldozer9000" in bad.json()["error"]
    sinter = c.post("/plan", json=_payload("sinter", vehicle="ipex"))     # no tool -> capability-refused
    assert sinter.status_code == 400 and "GATED OFF" in sinter.json()["error"]
