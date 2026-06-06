"""Static tip-over stability — the "don't tip" criterion (RL trains against it; the planner avoids it).

A 4-wheel rover on a tilted surface tips when its centre-of-mass projection crosses the support-polygon
edge. The static stability angle (SSA) about each body axis is ``atan(half_support / cg_height)``: ROLL
about the track (half = gauge/2), PITCH about the wheelbase (half = base/2). The rover is stable while
``|roll| < SSA_roll`` and ``|pitch| < SSA_pitch``; the margin is how many degrees remain on the binding axis.

RASSOR (KSC-TOPS-7 / US 9,027,265) makes tipping TERRAIN-driven, not DIG-driven: the counter-rotating bucket
drums cancel the horizontal dig reaction, so excavation adds ~no tipping moment (unlike a terrestrial
excavator that levers against its own weight). RASSOR is also symmetric and "recovers from overturning", so
a tip is recoverable, not fatal — but planning and RL still avoid it (recovery costs time + risk).

Geometry: the modeled rover's gauge/wheelbase live in ``rover.WHEEL_GAUGE_M`` / ``WHEEL_BASE_M`` (0.57 /
0.40 m); ``cg_height`` is an [ASSUMPTION] (``constants.CG_HEIGHT_M``) — the exact RASSOR/IPEx centre of mass
is not in the public spec, so it is a documented, env-overridable assumption, not a fabricated datum. With
the modeled gauge>wheelbase the PITCH axis binds (the rover is wider than it is long), so driving straight
up/down a steep grade tips before cross-slope does.
"""
from __future__ import annotations

import math


def ssa_deg(half_support_m: float, cg_height_m: float) -> float:
    """Static stability angle [deg] = atan(half_support / cg_height): the terrain tilt about an axis at
    which the CG crosses the support edge. Taller CG -> smaller SSA; wider support -> larger SSA."""
    if cg_height_m <= 0.0:
        return 90.0
    return math.degrees(math.atan2(half_support_m, cg_height_m))


def tip_tilt_limit_deg(*, gauge_m: float, wheelbase_m: float, cg_height_m: float) -> float:
    """The worst-case (binding) terrain tilt the rover can sit at before tipping — the physical ceiling on
    the planner's traverse-slope cap."""
    return min(ssa_deg(gauge_m / 2.0, cg_height_m), ssa_deg(wheelbase_m / 2.0, cg_height_m))


def stability(pitch_deg: float, roll_deg: float, *, gauge_m: float, wheelbase_m: float,
              cg_height_m: float, warn_frac: float = 0.7) -> dict:
    """Tip-over assessment from the rover's terrain attitude (pitch about the wheelbase, roll about the
    track; both from ``rover.conform_pose``). Returns the per-axis SSAs, the binding margin (degrees of
    tilt remaining before tip-over; <= 0 means tipping), the binding axis, and a risk band
    ('ok' / 'warn' within ``warn_frac`` of an SSA / 'tip')."""
    ssa_pitch = ssa_deg(wheelbase_m / 2.0, cg_height_m)
    ssa_roll = ssa_deg(gauge_m / 2.0, cg_height_m)
    m_pitch = ssa_pitch - abs(pitch_deg)
    m_roll = ssa_roll - abs(roll_deg)
    margin = min(m_pitch, m_roll)
    if margin <= 0.0:
        risk = "tip"
    elif abs(pitch_deg) >= warn_frac * ssa_pitch or abs(roll_deg) >= warn_frac * ssa_roll:
        risk = "warn"
    else:
        risk = "ok"
    return {
        "ssa_pitch_deg": ssa_pitch, "ssa_roll_deg": ssa_roll,
        "margin_deg": margin, "binding_axis": "pitch" if m_pitch <= m_roll else "roll", "risk": risk,
    }
