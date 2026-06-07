"""WP0.6 (PO-02 / RB-06) — reports + profiles live in a CONFIGURABLE application-data directory.

A wheel install puts the package in (often read-only) site-packages, so reports/profiles must NOT be
written beside the source. $DUSTGYM_DATA_DIR (else ~/.local/share/dustgym) is the writable root; this
verifies the resolver honors the env var, that mission reports land there, and that profile writes are
atomic. Real planner run on a real moon mission; no synthetic data.
"""
from __future__ import annotations

import os

from fastapi.testclient import TestClient

from planet_browser import mission_planner as MP
from planet_browser import server as SRV
from terrain_authority import config


def test_data_dir_honors_env(monkeypatch, tmp_path):
    monkeypatch.setenv("DUSTGYM_DATA_DIR", str(tmp_path))
    assert config.data_dir() == str(tmp_path)
    assert config.reports_dir() == os.path.join(str(tmp_path), "reports")
    assert config.profiles_dir() == os.path.join(str(tmp_path), "profiles")


def test_data_dir_default_is_outside_the_package():
    # default (no env) must NOT be inside the installed package dir (read-only on a wheel install)
    monkeypatch_env = os.environ.pop("DUSTGYM_DATA_DIR", None)
    try:
        d = config.data_dir()
    finally:
        if monkeypatch_env is not None:
            os.environ["DUSTGYM_DATA_DIR"] = monkeypatch_env
    pkg = os.path.dirname(os.path.abspath(SRV.__file__))
    assert "dustgym" in d and not d.startswith(pkg)


def test_report_is_written_to_the_configured_data_dir(monkeypatch, tmp_path):
    monkeypatch.setenv("DUSTGYM_DATA_DIR", str(tmp_path))
    m = MP.mission_from_dict({
        "name": "appdata", "body": "moon", "charger": [0, 0],
        "orders": [{"action": "pad", "kind": "cut", "x": 30, "y": 20, "footprint_m2": 25, "depth_m": 0.04}],
    })
    pdf, md, _ = MP.run(m, stem="appdata_test")
    assert pdf.startswith(str(tmp_path)) and os.path.exists(pdf)     # written under the data dir, not the package
    assert os.path.exists(md)


def test_profile_save_is_atomic_and_uses_the_data_dir(monkeypatch, tmp_path):
    # point the server's module-level dirs at the scratch root (the route uses PROFILES)
    monkeypatch.setattr(SRV, "PROFILES", str(tmp_path / "profiles"))
    c = TestClient(SRV.app)
    r = c.post("/profile", json={"name": "Haworth Pad 1", "profile": {"body": "moon", "orders": []}})
    assert r.status_code == 200
    saved = os.path.join(str(tmp_path / "profiles"), "haworth-pad-1.json")
    assert os.path.exists(saved)
    leftovers = [f for f in os.listdir(str(tmp_path / "profiles")) if f.endswith(".tmp")]
    assert leftovers == [], f"profile write left temp files: {leftovers}"   # atomic


def test_dem_dir_env_locates_the_asset_explicitly(monkeypatch, tmp_path):
    # RB-06 explicit asset mode: the (unpackaged) DEM bundle location is configurable.
    monkeypatch.setenv("DUSTGYM_DEM_DIR", str(tmp_path / "haworth"))
    assert MP._haworth_bundle() == str(tmp_path / "haworth")
    monkeypatch.delenv("DUSTGYM_DEM_DIR", raising=False)
    assert MP._haworth_bundle().endswith(os.path.join("samples", "lunar_dem", "haworth_10km_5m"))


def test_moon_dem_degrades_cleanly_when_asset_absent(monkeypatch):
    # a fresh wheel has no DEM bundle: _moon_dem degrades to a flat slope-check, it does NOT crash.
    monkeypatch.setenv("DUSTGYM_DEM_DIR", "/nonexistent/dem/bundle")
    monkeypatch.setattr(SRV, "_MOON_DEM", None)          # reset the cache
    dem, origin = SRV._moon_dem()
    assert dem is None and origin == (0.0, 0.0)


def _run_all():
    print("run under pytest (uses fixtures)")


if __name__ == "__main__":
    _run_all()
