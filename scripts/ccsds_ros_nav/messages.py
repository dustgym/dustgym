"""Command + telemetry payloads and the APID registry (CONTRACT.md §2, §3).

Each message is a small frozen dataclass with a fixed big-endian ``struct`` layout. ``encode`` wraps a
message in a CCSDS Space Packet (correct APID + packet type + MET secondary header); ``decode`` does the
inverse, dispatching on APID. Pure stdlib — no third-party serializer — so the wire format is auditable
and the tests run on a bare CPU.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

import ccsds

# --- APIDs (CONTRACT.md §2) -------------------------------------------------------------------------
APID_CMD_GOTO = 0x0C8
APID_CMD_SAFE = 0x0C9
APID_CMD_SETSIM = 0x0CA
APID_TLM_POSE = 0x064
APID_TLM_LEG = 0x065
APID_TLM_IMG = 0x066

# --- leg-completion status codes (CONTRACT.md §3 Leg) ----------------------------------------------
LEG_REACHED = 0
LEG_ENTRAPPED = 1
LEG_LOW_BATTERY = 2
LEG_MAX_STEPS = 3
LEG_SAFED = 4

LEG_STATUS_NAME = {
    LEG_REACHED: "REACHED", LEG_ENTRAPPED: "ENTRAPPED", LEG_LOW_BATTERY: "LOW_BATTERY",
    LEG_MAX_STEPS: "MAX_STEPS", LEG_SAFED: "SAFED",
}


@dataclass(frozen=True)
class GoTo:
    """Telecommand: drive to a waypoint in grid (row, col) cells."""

    leg_id: int
    goal_row: float
    goal_col: float
    v_max_mps: float = 0.3
    goal_radius_cells: float = 1.0

    APID = APID_CMD_GOTO
    PTYPE = ccsds.TYPE_TC
    _FMT = ">Idddd"

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, int(self.leg_id), float(self.goal_row), float(self.goal_col),
                           float(self.v_max_mps), float(self.goal_radius_cells))

    @classmethod
    def from_bytes(cls, b: bytes) -> "GoTo":
        leg_id, gr, gc, vmax, rad = struct.unpack(cls._FMT, b)
        return cls(leg_id=leg_id, goal_row=gr, goal_col=gc, v_max_mps=vmax, goal_radius_cells=rad)


@dataclass(frozen=True)
class Safe:
    """Telecommand: all-stop / safe the rover."""

    reason: int = 0

    APID = APID_CMD_SAFE
    PTYPE = ccsds.TYPE_TC
    _FMT = ">H"

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, int(self.reason) & 0xFFFF)

    @classmethod
    def from_bytes(cls, b: bytes) -> "Safe":
        (reason,) = struct.unpack(cls._FMT, b)
        return cls(reason=reason)


@dataclass(frozen=True)
class SetSim:
    """Telecommand: set the simulation time-acceleration factor (sim seconds per wall second)."""

    time_factor: float = 1.0

    APID = APID_CMD_SETSIM
    PTYPE = ccsds.TYPE_TC
    _FMT = ">d"

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, float(self.time_factor))

    @classmethod
    def from_bytes(cls, b: bytes) -> "SetSim":
        (tf,) = struct.unpack(cls._FMT, b)
        return cls(time_factor=tf)


@dataclass(frozen=True)
class Pose:
    """Telemetry: a single drive-tick state sample."""

    leg_id: int
    row: float
    col: float
    yaw_rad: float
    v_achieved_mps: float
    slip: float
    sinkage_m: float
    slope_rad: float
    soc: float
    entrapped: bool

    APID = APID_TLM_POSE
    PTYPE = ccsds.TYPE_TM
    _FMT = ">H8dB"

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, int(self.leg_id) & 0xFFFF, float(self.row), float(self.col),
                           float(self.yaw_rad), float(self.v_achieved_mps), float(self.slip),
                           float(self.sinkage_m), float(self.slope_rad), float(self.soc),
                           1 if self.entrapped else 0)

    @classmethod
    def from_bytes(cls, b: bytes) -> "Pose":
        leg_id, row, col, yaw, v, slip, sink, slope, soc, entr = struct.unpack(cls._FMT, b)
        return cls(leg_id=leg_id, row=row, col=col, yaw_rad=yaw, v_achieved_mps=v, slip=slip,
                   sinkage_m=sink, slope_rad=slope, soc=soc, entrapped=bool(entr))


@dataclass(frozen=True)
class Leg:
    """Telemetry: leg-complete summary."""

    leg_id: int
    status: int
    commanded_dist_m: float
    achieved_dist_m: float
    energy_J: float
    mass_kg: float
    final_row: float
    final_col: float

    APID = APID_TLM_LEG
    PTYPE = ccsds.TYPE_TM
    _FMT = ">HH6d"

    def to_bytes(self) -> bytes:
        return struct.pack(self._FMT, int(self.leg_id) & 0xFFFF, int(self.status) & 0xFFFF,
                           float(self.commanded_dist_m), float(self.achieved_dist_m),
                           float(self.energy_J), float(self.mass_kg),
                           float(self.final_row), float(self.final_col))

    @classmethod
    def from_bytes(cls, b: bytes) -> "Leg":
        leg_id, status, cd, ad, e, m, fr, fc = struct.unpack(cls._FMT, b)
        return cls(leg_id=leg_id, status=status, commanded_dist_m=cd, achieved_dist_m=ad,
                   energy_J=e, mass_kg=m, final_row=fr, final_col=fc)


@dataclass(frozen=True)
class Img:
    """Telemetry: imagery file metadata — the CFDP-style 'a file is ready' downlink announce."""

    leg_id: int
    frame_index: int
    width: int
    height: int
    size_bytes: int
    name: str

    APID = APID_TLM_IMG
    PTYPE = ccsds.TYPE_TM
    _HEAD = ">HHHHIH"

    def to_bytes(self) -> bytes:
        raw = self.name.encode("utf-8")
        return struct.pack(self._HEAD, int(self.leg_id) & 0xFFFF, int(self.frame_index) & 0xFFFF,
                           int(self.width) & 0xFFFF, int(self.height) & 0xFFFF,
                           int(self.size_bytes) & 0xFFFFFFFF, len(raw)) + raw

    @classmethod
    def from_bytes(cls, b: bytes) -> "Img":
        n = struct.calcsize(cls._HEAD)
        leg_id, frame_index, w, h, size, name_len = struct.unpack(cls._HEAD, b[:n])
        name = b[n:n + name_len].decode("utf-8")
        return cls(leg_id=leg_id, frame_index=frame_index, width=w, height=h,
                   size_bytes=size, name=name)


_DECODERS = {
    APID_CMD_GOTO: GoTo, APID_CMD_SAFE: Safe, APID_CMD_SETSIM: SetSim,
    APID_TLM_POSE: Pose, APID_TLM_LEG: Leg, APID_TLM_IMG: Img,
}


def encode(msg, *, seq_count: int = 0, met: float | None = 0.0) -> ccsds.SpacePacket:
    """Wrap a message dataclass in a CCSDS Space Packet (its APID/type, MET secondary header)."""
    return ccsds.SpacePacket(apid=msg.APID, packet_type=msg.PTYPE, seq_count=seq_count,
                             user_data=msg.to_bytes(), met=met)


def decode(pkt: ccsds.SpacePacket):
    """Reconstruct the message dataclass from a Space Packet, dispatching on APID."""
    klass = _DECODERS.get(pkt.apid)
    if klass is None:
        raise ValueError(f"no decoder registered for APID 0x{pkt.apid:03X}")
    return klass.from_bytes(pkt.user_data)
