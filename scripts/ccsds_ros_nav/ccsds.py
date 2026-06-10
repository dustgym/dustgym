"""CCSDS Space Packet codec (CCSDS 133.0-B-2).

A small, dependency-free implementation of the CCSDS Space Packet Protocol primary header (the
6-octet header that frames every command and telemetry unit in this stack) plus this project's
secondary-header convention. This is a real, spec-faithful codec — not a placeholder — and is fully
exercised end to end (the loopback link packs and unpacks real bytes, see link.py).

Reference: CCSDS 133.0-B-2, "Space Packet Protocol" (Blue Book). The primary header is 48 bits:

    word0: packet version number (3) | packet type (1) | secondary header flag (1) | APID (11)
    word1: sequence flags (2) | packet sequence count (14)
    word2: packet data length = (octets in the packet data field) - 1

``packet type`` is 0 for telemetry (TM, space->ground) and 1 for telecommand (TC, ground->space).

Secondary-header convention (this mission, see CONTRACT.md §1): when the secondary header flag is set,
the first 8 octets of the data field are a big-endian IEEE-754 float64 Mission Elapsed Time [s] (a
simplified stand-in for a CCSDS 301.0-B time code). The rest of the data field is the user payload.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

PRIMARY_HEADER_LEN = 6

# packet type
TYPE_TM = 0  # telemetry: rover -> ground
TYPE_TC = 1  # telecommand: ground -> rover

# sequence flags (CCSDS 133.0-B Table 4-3); we only emit standalone, unsegmented packets.
SEQ_UNSEGMENTED = 0b11

APID_IDLE = 0x7FF  # reserved idle APID (133.0-B section 4.1.2.3.4)
_APID_MAX = 0x7FF
_SEQ_COUNT_MAX = 0x3FFF
_DATA_LEN_MAX = 0x10000  # data field may be 1..65536 octets (length field holds len-1)
_MET_LEN = 8  # float64 mission-elapsed-time secondary header


@dataclass(frozen=True)
class SpacePacket:
    """One CCSDS Space Packet (primary header + optional MET secondary header + user payload)."""

    apid: int
    packet_type: int          # TYPE_TM or TYPE_TC
    seq_count: int            # 14-bit packet sequence count, wraps at 0x3FFF
    user_data: bytes          # the payload (after the secondary header, if any)
    met: float | None = None  # mission elapsed time [s]; None -> no secondary header
    version: int = 0
    seq_flags: int = SEQ_UNSEGMENTED

    def __post_init__(self) -> None:
        if not (0 <= self.apid <= _APID_MAX):
            raise ValueError(f"APID {self.apid} out of 11-bit range")
        if self.packet_type not in (TYPE_TM, TYPE_TC):
            raise ValueError(f"packet_type must be 0 (TM) or 1 (TC), got {self.packet_type}")
        if not (0 <= self.seq_count <= _SEQ_COUNT_MAX):
            raise ValueError(f"seq_count {self.seq_count} out of 14-bit range")
        if self.version != 0:
            raise ValueError(f"CCSDS packet version number must be 0 (133.0-B-2), got {self.version}")

    @property
    def sec_hdr_flag(self) -> int:
        return 1 if self.met is not None else 0

    def _data_field(self) -> bytes:
        head = struct.pack(">d", float(self.met)) if self.met is not None else b""
        return head + self.user_data

    def pack(self) -> bytes:
        """Serialize to the on-the-wire octet string (primary header + data field)."""
        data = self._data_field()
        if not (1 <= len(data) <= _DATA_LEN_MAX):
            raise ValueError(f"data field must be 1..{_DATA_LEN_MAX} octets, got {len(data)}")
        w0 = ((self.version & 0x7) << 13) | ((self.packet_type & 0x1) << 12) \
            | ((self.sec_hdr_flag & 0x1) << 11) | (self.apid & _APID_MAX)
        w1 = ((self.seq_flags & 0x3) << 14) | (self.seq_count & _SEQ_COUNT_MAX)
        w2 = len(data) - 1
        return struct.pack(">HHH", w0, w1, w2) + data

    @classmethod
    def unpack(cls, buf: bytes) -> "SpacePacket":
        """Parse one Space Packet from ``buf`` (must contain exactly one packet)."""
        if len(buf) < PRIMARY_HEADER_LEN:
            raise ValueError(f"buffer too short for a Space Packet header ({len(buf)} < 6)")
        w0, w1, w2 = struct.unpack(">HHH", buf[:PRIMARY_HEADER_LEN])
        version = (w0 >> 13) & 0x7
        packet_type = (w0 >> 12) & 0x1
        sec_hdr_flag = (w0 >> 11) & 0x1
        apid = w0 & _APID_MAX
        seq_flags = (w1 >> 14) & 0x3
        seq_count = w1 & _SEQ_COUNT_MAX
        data_len = w2 + 1
        data = buf[PRIMARY_HEADER_LEN:PRIMARY_HEADER_LEN + data_len]
        if len(data) != data_len:
            raise ValueError(f"truncated data field: header says {data_len} octets, got {len(data)}")
        met: float | None = None
        if sec_hdr_flag:
            if len(data) < _MET_LEN:
                raise ValueError("secondary-header flag set but data field shorter than MET (8 octets)")
            (met,) = struct.unpack(">d", data[:_MET_LEN])
            user_data = data[_MET_LEN:]
        else:
            user_data = data
        return cls(apid=apid, packet_type=packet_type, seq_count=seq_count,
                   user_data=user_data, met=met, version=version, seq_flags=seq_flags)

    def total_len(self) -> int:
        """Octets this packet occupies on the wire."""
        return PRIMARY_HEADER_LEN + len(self._data_field())
