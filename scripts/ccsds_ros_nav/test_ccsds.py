"""Tests for the CCSDS 133.0-B-2 Space Packet codec."""
from __future__ import annotations

import struct

import pytest

import ccsds


def test_primary_header_bit_layout():
    pkt = ccsds.SpacePacket(apid=0x0C8, packet_type=ccsds.TYPE_TC, seq_count=5,
                            user_data=b"\x01\x02", met=None)
    raw = pkt.pack()
    w0, w1, w2 = struct.unpack(">HHH", raw[:6])
    assert (w0 >> 13) & 0x7 == 0                 # version 0
    assert (w0 >> 12) & 0x1 == ccsds.TYPE_TC     # type bit
    assert (w0 >> 11) & 0x1 == 0                  # no secondary header (met is None)
    assert w0 & 0x7FF == 0x0C8                    # APID
    assert (w1 >> 14) & 0x3 == ccsds.SEQ_UNSEGMENTED
    assert w1 & 0x3FFF == 5                       # sequence count
    assert w2 == len(b"\x01\x02") - 1            # data length = octets - 1


def test_roundtrip_with_met_secondary_header():
    pkt = ccsds.SpacePacket(apid=0x064, packet_type=ccsds.TYPE_TM, seq_count=1234,
                            user_data=b"payload-bytes", met=12.5)
    back = ccsds.SpacePacket.unpack(pkt.pack())
    assert back.apid == 0x064
    assert back.packet_type == ccsds.TYPE_TM
    assert back.seq_count == 1234
    assert back.sec_hdr_flag == 1
    assert back.met == pytest.approx(12.5)
    assert back.user_data == b"payload-bytes"


def test_total_len_matches_packed():
    pkt = ccsds.SpacePacket(apid=1, packet_type=ccsds.TYPE_TM, seq_count=0, user_data=b"abc", met=0.0)
    assert pkt.total_len() == len(pkt.pack())


def test_seq_count_wraps_at_14_bits():
    assert ccsds.SpacePacket(apid=1, packet_type=0, seq_count=0x3FFF, user_data=b"x").seq_count == 0x3FFF
    with pytest.raises(ValueError):
        ccsds.SpacePacket(apid=1, packet_type=0, seq_count=0x4000, user_data=b"x")


def test_rejects_bad_apid_and_type():
    with pytest.raises(ValueError):
        ccsds.SpacePacket(apid=0x800, packet_type=0, seq_count=0, user_data=b"x")
    with pytest.raises(ValueError):
        ccsds.SpacePacket(apid=1, packet_type=2, seq_count=0, user_data=b"x")


def test_empty_data_field_rejected_on_pack():
    with pytest.raises(ValueError):
        ccsds.SpacePacket(apid=1, packet_type=0, seq_count=0, user_data=b"", met=None).pack()


def test_unpack_truncated_raises():
    with pytest.raises(ValueError):
        ccsds.SpacePacket.unpack(b"\x00\x00\x00")              # shorter than header
    good = ccsds.SpacePacket(apid=1, packet_type=0, seq_count=0, user_data=b"abcd").pack()
    with pytest.raises(ValueError):
        ccsds.SpacePacket.unpack(good[:-2])                    # data field truncated
