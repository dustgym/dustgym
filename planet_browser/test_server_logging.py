"""N10 structured logging / observability on the ASGI planner server.

The access-log middleware routes every request (method/path/status/duration) through the
`planet_browser.server` logger, configurable via $DUSTGYM_LOG_LEVEL.
"""
import logging

import pytest
from fastapi.testclient import TestClient

from . import server as SRV


@pytest.fixture()
def client():
    return TestClient(SRV.app)


def test_configure_logging_respects_env(monkeypatch):
    monkeypatch.setenv("DUSTGYM_LOG_LEVEL", "DEBUG")
    SRV._configure_logging()
    assert logging.getLogger().level == logging.DEBUG
    SRV._configure_logging("INFO")                                  # restore a sane default
    assert logging.getLogger().level == logging.INFO


def test_request_is_access_logged(client, caplog):
    with caplog.at_level(logging.INFO, logger="planet_browser.server"):
        r = client.get("/healthz")
        assert r.status_code == 200
    recs = [rec for rec in caplog.records if rec.name == "planet_browser.server"]
    assert recs, "the request produced no access-log record"
    blob = " ".join(rec.getMessage() for rec in recs)
    assert "GET /healthz" in blob and "200" in blob


def test_bad_route_is_logged_with_status(client, caplog):
    with caplog.at_level(logging.INFO, logger="planet_browser.server"):
        r = client.get("/no-such-route")
        assert r.status_code == 404
    blob = " ".join(rec.getMessage() for rec in caplog.records if rec.name == "planet_browser.server")
    assert "404" in blob
