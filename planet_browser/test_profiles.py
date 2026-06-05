"""Profiles: save / list / load a planning config snapshot via the server (load/save for import/export)."""
from fastapi.testclient import TestClient

from planet_browser import server as SRV


def _client(tmp_path, monkeypatch):
    monkeypatch.setattr(SRV, "PROFILES", str(tmp_path))   # isolate from the real profiles/ dir
    return TestClient(SRV.app)


def test_save_list_load_round_trip(tmp_path, monkeypatch):
    c = _client(tmp_path, monkeypatch)
    cfg = {"body": "moon", "soil": "earth", "vehicle": "ipex", "tools": ["sinter"],
           "orders": [{"action": "cut", "kind": "cut", "x": 5, "y": 5, "footprint_m2": 9, "depth_m": 0.05}],
           "algorithm": "nearest", "objective": "energy"}
    saved = c.post("/profile", json={"name": "Haworth Pad #1", "profile": cfg})
    assert saved.status_code == 200 and saved.json()["name"] == "haworth-pad-1"        # slugified

    listing = c.get("/profiles").json()
    assert listing["ok"] is True and "haworth-pad-1" in listing["profiles"]

    loaded = c.get("/profile/haworth-pad-1").json()
    assert loaded["name"] == "Haworth Pad #1" and loaded["profile"] == cfg              # exact round-trip


def test_load_missing_profile_404(tmp_path, monkeypatch):
    c = _client(tmp_path, monkeypatch)
    r = c.get("/profile/does-not-exist")
    assert r.status_code == 404 and r.json()["ok"] is False and "no profile" in r.json()["error"]


def test_empty_name_rejected(tmp_path, monkeypatch):
    c = _client(tmp_path, monkeypatch)
    assert c.post("/profile", json={"name": "", "profile": {}}).status_code == 400   # min_length=1


def test_profiles_empty_when_none_saved(tmp_path, monkeypatch):
    c = _client(tmp_path / "empty", monkeypatch)
    assert c.get("/profiles").json() == {"ok": True, "profiles": []}
