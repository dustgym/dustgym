"""server.py route coverage — real http.server on an ephemeral port, hit with urllib.

Complements planet_browser/test_mission_planner.py (which drives /plan, /sense, /compare, /structure
success paths). This file fills the route gaps with REAL HTTP round-trips against the stdlib server:

  * GET /, /index.html, /bodies.json, /reports/<name> (200 + real bytes), and the 404 paths.
  * GET /dem/hillshade.png + /dem/height.png -> the real committed Haworth preview PNGs (image/png),
    and /dem/<bogus> -> 404.
  * POST unknown route -> 404; POST with bad JSON -> 400.
  * POST /render -> 503 when the Godot pipeline is absent (PRP is None), and 400 on bad params.
  * /sense, /compare, /structure error paths (gated/invalid inputs return honest 4xx).
  * sinter order via /plan -> 400 with the GATED-OFF error (constants.SINTER_ENABLED is False).

The server is the REAL one (no synthetic constants); reports/PDFs are produced by the grounded
mission_planner. Run: cd .. && PYTHONPATH=. <venv>/bin/python -m pytest planet_browser/test_server.py -q
"""
from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

import pytest

from . import server as SRV


@pytest.fixture()
def base():
    srv = SRV.make_server(0)                                 # ephemeral port, real ThreadingHTTPServer
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        yield "http://127.0.0.1:%d" % srv.server_address[1]
    finally:
        srv.shutdown()


def _get(base, route):
    try:
        with urllib.request.urlopen(base + route, timeout=30) as r:
            return r.status, r.read(), r.headers.get("Content-Type")
    except urllib.error.HTTPError as e:
        return e.code, e.read(), e.headers.get("Content-Type")


def _post(base, route, obj, raw=None):
    data = raw if raw is not None else json.dumps(obj).encode()
    req = urllib.request.Request(base + route, data=data, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


# ---- GET static + content types ------------------------------------------------------------------
def test_get_index_root_and_alias(base):
    for route in ("/", "/index.html"):
        code, body, ctype = _get(base, route)
        assert code == 200 and b"<" in body[:2048]          # real HTML
        assert "text/html" in ctype


def test_get_bodies_json_is_real_json(base):
    code, body, ctype = _get(base, "/bodies.json")
    assert code == 200 and "application/json" in ctype
    d = json.loads(body)
    assert "moon" in d and "_ipex" in d                      # the py-generated bodies + ipex mirror


def test_get_unknown_route_404(base):
    code, body, _ = _get(base, "/does-not-exist")
    j = json.loads(body)
    assert code == 404 and j["ok"] is False and "no route" in j["error"]


def test_get_missing_report_404(base):
    code, body, _ = _get(base, "/reports/nonesuch-deadbeef.pdf")
    j = json.loads(body)
    assert code == 404 and j["ok"] is False and "not found" in j["error"]


# ---- GET /dem/ : the real committed Haworth preview PNGs -----------------------------------------
def test_get_dem_hillshade_png(base):
    code, body, ctype = _get(base, "/dem/hillshade.png")
    assert code == 200 and ctype == "image/png"
    assert body[:8] == b"\x89PNG\r\n\x1a\n"                  # a real PNG signature


def test_get_dem_height_png(base):
    code, body, ctype = _get(base, "/dem/height.png")
    assert code == 200 and ctype == "image/png" and body[:4] == b"\x89PNG"


def test_get_dem_unknown_404(base):
    code, body, _ = _get(base, "/dem/bogus.png")
    j = json.loads(body)
    assert code == 404 and "no dem" in j["error"]


# ---- a generated report is actually served back from /reports/ ----------------------------------
def test_report_pdf_round_trips_through_reports_route(base):
    payload = {"name": "Served Site", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 40, "y": 30, "footprint_m2": 36, "depth_m": 0.04},
        {"action": "fill", "kind": "fill", "x": 44, "y": 44, "footprint_m2": 14, "depth_m": 0.10}]}
    code, body = _post(base, "/plan", payload)
    assert code == 200 and body["pdf"].startswith("/reports/")
    pcode, pbody, pctype = _get(base, body["pdf"])
    assert pcode == 200 and pbody[:5] == b"%PDF-" and pctype == "application/pdf"
    mcode, mbody, mctype = _get(base, body["md"])
    assert mcode == 200 and "text/markdown" in mctype and len(mbody) > 0


# ---- POST error paths ---------------------------------------------------------------------------
def test_post_unknown_route_404(base):
    code, body = _post(base, "/nope", {})
    assert code == 404 and body["ok"] is False and "no route" in body["error"]


def test_post_bad_json_400(base):
    code, body = _post(base, "/plan", None, raw=b"{not valid json")
    assert code == 400 and body["ok"] is False and "bad JSON" in body["error"]


def test_plan_sinter_order_gated_off_400(base):
    code, body = _post(base, "/plan", {"name": "S", "body": "moon", "charger": [0, 0], "orders": [
        {"action": "Sinter apron", "kind": "sinter", "x": 10, "y": 10, "footprint_m2": 9, "depth_m": 0.01}]})
    assert code == 400 and body["ok"] is False and "GATED OFF" in body["error"]


def test_plan_unknown_body_400(base):
    code, body = _post(base, "/plan", {"name": "P", "body": "pluto", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 1, "y": 1, "footprint_m2": 9, "depth_m": 0.02}]})
    assert code == 400 and "body" in body["error"]


# ---- /sense (drum-fill sensing) error + success -------------------------------------------------
def test_sense_missing_true_mass_400(base):
    code, body = _post(base, "/sense", {"capacity_kg": 30.0})
    assert code == 400 and "true_mass_kg" in body["error"]


def test_sense_success_inferred_and_offload(base):
    code, body = _post(base, "/sense", {"true_mass_kg": 12.0})
    assert code == 200 and body["ok"] is True
    assert abs(body["inferred_kg"] - 12.0) < 1.0            # faithful inference, noise off
    assert body["offload"] is False and body["current_a"] > 0.0
    assert body["lower_kg"] <= body["inferred_kg"] <= body["upper_kg"]


# ---- /compare + /structure error paths ----------------------------------------------------------
def test_compare_unknown_body_400(base):
    code, body = _post(base, "/compare", {"name": "c", "body": "pluto", "charger": [0, 0], "orders": [
        {"action": "cut", "kind": "cut", "x": 1, "y": 1, "footprint_m2": 9, "depth_m": 0.02}]})
    assert code == 400 and body["ok"] is False


def test_structure_non_numeric_xy_400(base):
    code, body = _post(base, "/structure", {"name": "landing_pad", "x": "oops", "y": 0})
    assert code == 400 and body["ok"] is False and "numeric" in body["error"]


def test_structure_unknown_name_400(base):
    code, body = _post(base, "/structure", {"name": "death_star", "x": 0, "y": 0})
    assert code == 400 and body["ok"] is False and "death_star" in body["error"]


# ---- /render : 503 when the Godot pipeline is absent, 400 on bad params --------------------------
def test_render_503_when_pipeline_absent(base, monkeypatch):
    # the render pipeline degrades to a 503 when the Godot binary / plan_render_pipeline is unavailable.
    monkeypatch.setattr(SRV, "PRP", None)                   # real degrade path, not a stub of logic
    code, body = _post(base, "/render", {"u": 0.5, "v": 0.5})
    assert code == 503 and body["ok"] is False and "render pipeline unavailable" in body["error"]


def test_render_bad_params_400(base):
    if SRV.PRP is None:
        pytest.skip("render pipeline absent -> /render short-circuits to 503 before param parse")
    # PRP present: bad (non-numeric) params are rejected with a 400 BEFORE any Godot render runs.
    code, body = _post(base, "/render", {"u": "not-a-float", "v": 0.5})
    assert code == 400 and body["ok"] is False and "bad params" in body["error"]
