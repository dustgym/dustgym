"""Tests for the command/telemetry payloads + APID registry."""
from __future__ import annotations

import pytest

import ccsds
import messages


def _roundtrip(msg):
    pkt = messages.encode(msg, seq_count=7, met=3.25)
    assert pkt.met == pytest.approx(3.25)
    assert pkt.seq_count == 7
    return messages.decode(ccsds.SpacePacket.unpack(pkt.pack()))


def test_goto_roundtrip_and_apid():
    g = messages.GoTo(leg_id=42, goal_row=10.25, goal_col=-3.5, v_max_mps=0.25, goal_radius_cells=2.0)
    assert g.APID == messages.APID_CMD_GOTO
    assert g.PTYPE == ccsds.TYPE_TC
    assert _roundtrip(g) == g


def test_safe_roundtrip():
    assert _roundtrip(messages.Safe(reason=9)) == messages.Safe(reason=9)


def test_setsim_roundtrip_and_apid():
    s = messages.SetSim(time_factor=120.0)
    assert s.APID == messages.APID_CMD_SETSIM and s.PTYPE == ccsds.TYPE_TC
    assert _roundtrip(s) == s


def test_pose_roundtrip_preserves_fields():
    p = messages.Pose(leg_id=1, row=12.5, col=7.25, yaw_rad=0.5, v_achieved_mps=0.27,
                      slip=0.13, sinkage_m=0.004, slope_rad=0.08, soc=0.91, entrapped=True)
    back = _roundtrip(p)
    assert back.APID == messages.APID_TLM_POSE and back.PTYPE == ccsds.TYPE_TM
    assert back == p


def test_leg_roundtrip():
    L = messages.Leg(leg_id=2, status=messages.LEG_REACHED, commanded_dist_m=50.0,
                     achieved_dist_m=48.0, energy_J=6500.0, mass_kg=1.0e6,
                     final_row=20.0, final_col=30.0)
    assert _roundtrip(L) == L


def test_img_roundtrip_variable_name():
    im = messages.Img(leg_id=3, frame_index=5, width=1024, height=768,
                      size_bytes=123456, name="leg03/front_left_005.png")
    assert _roundtrip(im) == im


def test_decode_unknown_apid_raises():
    pkt = ccsds.SpacePacket(apid=0x123, packet_type=ccsds.TYPE_TM, seq_count=0, user_data=b"x", met=0.0)
    with pytest.raises(ValueError):
        messages.decode(pkt)


def test_dataclass_dict_roundtrip_for_ros_bridge():
    # the rclpy bridge ships Pose/Leg as JSON dicts (vars(msg)) and rebuilds via Klass(**dict);
    # guard that path here without needing rclpy.
    p = messages.Pose(leg_id=4, row=1.5, col=2.5, yaw_rad=0.3, v_achieved_mps=0.2, slip=0.1,
                      sinkage_m=0.003, slope_rad=0.04, soc=0.88, entrapped=False)
    assert messages.Pose(**vars(p)) == p
    L = messages.Leg(leg_id=4, status=messages.LEG_REACHED, commanded_dist_m=10.0, achieved_dist_m=9.0,
                     energy_J=1500.0, mass_kg=1.0e6, final_row=1.5, final_col=2.5)
    assert messages.Leg(**vars(L)) == L
