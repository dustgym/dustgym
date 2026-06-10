"""Tests for the CCSDS link transports (loopback always; UDP best-effort)."""
from __future__ import annotations

import ccsds
import messages
from link import UdpLink, loopback_pair


def test_loopback_pair_carries_real_octets_both_ways():
    ground, flight = loopback_pair()
    ground.send(messages.encode(messages.GoTo(leg_id=1, goal_row=5.0, goal_col=6.0), met=0.0))
    got = messages.decode(flight.recv(timeout=1.0))
    assert isinstance(got, messages.GoTo) and got.leg_id == 1

    flight.send(messages.encode(messages.Leg(leg_id=1, status=messages.LEG_REACHED,
                commanded_dist_m=1.0, achieved_dist_m=1.0, energy_J=1.0, mass_kg=1.0,
                final_row=5.0, final_col=6.0), met=1.0))
    back = messages.decode(ground.recv(timeout=1.0))
    assert isinstance(back, messages.Leg) and back.status == messages.LEG_REACHED


def test_loopback_recv_timeout_returns_none():
    ground, _flight = loopback_pair()
    assert ground.recv(timeout=0.05) is None


def test_udp_nonblocking_recv_returns_none_when_empty():
    # regression: timeout=0.0 -> non-blocking socket -> empty queue raises BlockingIOError,
    # which recv must swallow and return None (else a 50 Hz poll-drain crashes the bridge).
    try:
        a = UdpLink(("127.0.0.1", 0), ("127.0.0.1", 9))
    except OSError:
        import pytest
        pytest.skip("UDP sockets unavailable in this environment")
    try:
        assert a.recv(timeout=0.0) is None
    finally:
        a.close()


def test_udp_localhost_roundtrip():
    # localhost UDP; best-effort (skip if the sandbox forbids binding).
    try:
        a = UdpLink(("127.0.0.1", 0), ("127.0.0.1", 0))
        b_port = a._sock.getsockname()[1]
        b = UdpLink(("127.0.0.1", 0), ("127.0.0.1", b_port))
        a.remote_addr = ("127.0.0.1", b._sock.getsockname()[1])
    except OSError:
        import pytest
        pytest.skip("UDP sockets unavailable in this environment")
    try:
        a.send(messages.encode(messages.Safe(reason=3), met=0.0))
        pkt = b.recv(timeout=2.0)
        assert pkt is not None
        assert messages.decode(pkt) == messages.Safe(reason=3)
        assert isinstance(pkt, ccsds.SpacePacket)
    finally:
        a.close()
        b.close()
