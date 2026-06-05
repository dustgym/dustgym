"""N10 structured logging / observability on the planner server.

The server routes its access logs and previously-silent failure paths through a real logger
(`planet_browser.server`) instead of stdout/silent, configurable via $DUSTGYM_LOG_LEVEL.
"""
import logging
import threading
import urllib.request

import pytest

from planet_browser import server


def test_configure_logging_respects_env(monkeypatch):
    monkeypatch.setenv("DUSTGYM_LOG_LEVEL", "DEBUG")
    server._configure_logging()
    assert logging.getLogger().level == logging.DEBUG
    # restore a sane default so later tests are not left at DEBUG
    server._configure_logging("INFO")
    assert logging.getLogger().level == logging.INFO


@pytest.fixture
def live_server():
    srv = server.make_server(port=0, host="127.0.0.1")
    host, port = srv.server_address
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    try:
        yield f"http://{host}:{port}"
    finally:
        srv.shutdown()
        t.join(timeout=5)


def test_request_is_access_logged(live_server, caplog):
    with caplog.at_level(logging.INFO, logger="planet_browser.server"):
        with urllib.request.urlopen(live_server + "/", timeout=10) as r:
            assert r.status == 200
    recs = [rec for rec in caplog.records if rec.name == "planet_browser.server"]
    assert recs, "the request produced no access-log record"
    # the access log carries the request line (GET /) and the 200 status
    blob = " ".join(rec.getMessage() for rec in recs)
    assert "GET /" in blob and "200" in blob


def test_bad_route_is_logged_with_status(live_server, caplog):
    with caplog.at_level(logging.INFO, logger="planet_browser.server"):
        try:
            urllib.request.urlopen(live_server + "/no-such-route", timeout=10)
        except urllib.error.HTTPError as e:
            assert e.code == 404
    blob = " ".join(rec.getMessage() for rec in caplog.records if rec.name == "planet_browser.server")
    assert "404" in blob
