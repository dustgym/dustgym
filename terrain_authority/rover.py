"""Minimal single wheel-pass rut carving (spec §6 TREAD transition, §9).

GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE. Per spec §9 ("Robot design context"): IPEx
uses counter-rotating bucket drums (RASSOR heritage; 2021-ASCEND-Mass-Inference-RASSOR.pdf)
that cancel horizontal excavation reaction forces because in 1/6 g there is too little
weight-on-wheels to anchor digging — "because forces are engineered small, the Tier-2
analytical layer need not be force-accurate to high precision to be useful — it must be
geometry- and state-accurate." asce-es-2024-isru-pilot-excavator-wheel-testing.pdf
characterizes the IPEx wheel on simulant; we reproduce the OBSERVABLE outcome of a pass
(a compacted, slightly sunken rut with bumped disturbance), not the contact mechanics.

A pass does the spec §6 VIRGIN/TREAD -> TREAD transition:
    density UP (compaction), height DOWN slightly (MASS PRESERVED — denser column is
    thinner), state_label -> TREAD, disturbance bumped. Multi-pass "paving" emerges by
    re-applying: each pass meets denser soil and compacts a little less (spec §6).

We do NOT model slip-sinkage / runaway entrapment here (spec §6 two sinkage modes) — that
is the path-dependent failure a full Chrono::Vehicle + slip-sinkage solver would surface.

Beyond the single-track ``wheel_pass`` (kept intact; tests + the tread_track scene depend on
it), this module adds the IPEx 4-wheel footprint and the excavation drum (spec §5 producer
changes; INTERFACE.md §5.2 additive metadata):
    - ``wheel_contact_points`` / ``four_wheel_pass``: stamp FOUR separate compacting ruts
      (LF/RF/LB/RB) from a rover pose sequence, reusing wheel_pass's mass-conserving mechanism.
    - ``build_wheel_tracks_meta`` / ``build_drum_marks_meta``: shape the frozen §5.2
      ``wheel_tracks`` / ``drum_marks`` metadata so the shader can orient per-wheel cleat
      (§4.2.3) and drum-teeth (§4.2.4) detail WITHOUT resolving it in the heightfield.
    - ``drum_pass``: cut an EXCAVATED swath (and optionally DUMP it as SPOIL) via the existing
      column_state drum inventory — mass conserved through the inventory (spec §7 bulking).
All four-wheel / drum ops keep mass_areal conserved (spec §10 invariant 1); height re-derives.
"""

from __future__ import annotations

import numpy as np

from . import constants as K
from .column_state import ColumnState, StateLabel


def _wheel_mask(cs: ColumnState, center_rc: tuple[float, float], half_width_cells: float) -> np.ndarray:
    """Disc footprint of the contact patch around (row,col)."""
    r0, c0 = center_rc
    rows = np.arange(cs.height)[:, None] - r0
    cols = np.arange(cs.width)[None, :] - c0
    return (rows ** 2 + cols ** 2) <= half_width_cells ** 2


def wheel_pass(cs: ColumnState, path_rc: list[tuple[int, int]], *,
               wheel_width_m: float = 0.18, compaction: float = 0.12) -> ColumnState:
    """Carve a single rut along ``path_rc`` (list of (row,col)), in-place. MASS PRESERVED.

    wheel_width_m: contact-patch width (~10-20 cm, spec §4 resolution anchor; IPEx wheel,
        asce-es-2024). compaction: fractional density increase under the wheel per pass.

    Mechanism (spec §6): density *= (1+compaction) up to RHO_DEEP. Because mass_areal is
    untouched and height = mass/density, the column thins -> the rut sinks. Disturbance is
    bumped (drives fresh-cut albedo/roughness downstream, INTERFACE.md §4), and the cell
    is relabelled TREAD (or COMPACTED_BERM if it was SPOIL — driving over spoil compacts
    it into a real structure, spec §6).
    """
    half_w = max(0.5, 0.5 * wheel_width_m / cs.cell_m)  # half-width in cells

    touched = np.zeros((cs.height, cs.width), dtype=bool)
    for (r, c) in path_rc:
        m = _wheel_mask(cs, (r, c), half_w)
        touched |= m

    if not touched.any():
        return cs

    # Compaction: density up, capped at the deep/compacted ceiling. MASS UNCHANGED ->
    # height drops automatically via derive_height().
    cs.density[touched] = np.minimum(cs.density[touched] * (1.0 + compaction), K.RHO_DEEP)

    # State + disturbance. SPOIL -> COMPACTED_BERM (deliberate structure step, spec §6);
    # everything else -> TREAD.
    was_spoil = touched & (cs.state_label == StateLabel.SPOIL)
    cs.state_label[touched & ~was_spoil] = StateLabel.TREAD
    cs.state_label[was_spoil] = StateLabel.COMPACTED_BERM
    cs.disturbance[touched] = np.clip(cs.disturbance[touched] + 0.35, 0.0, 1.0)
    return cs


def straight_path(r0: int, c0: int, r1: int, c1: int, step_cells: int = 1) -> list[tuple[int, int]]:
    """Sample a straight (row,col) path between two cells (Bresenham-ish, dense)."""
    n = max(abs(r1 - r0), abs(c1 - c0)) // max(1, step_cells) + 1
    rs = np.linspace(r0, r1, n).round().astype(int)
    cs = np.linspace(c0, c1, n).round().astype(int)
    return list(zip(rs.tolist(), cs.tolist()))


# ---------------------------------------------------------------------------
# Per-wheel stamping — FOUR separate compacting ruts (spec §5 "rover.py ->
# 4-wheel stamping", §4.2.3 cleat marks; INTERFACE.md §5.2 wheel_tracks).
#
# wheel_pass (above) sweeps ONE disc footprint along a centerline — kept intact
# because tests + the tread_track scene depend on it. The functions below add the
# IPEx 4-wheel layout: from a rover pose (center + heading) we place the 4 ground
# contacts via the sidecar.gd WHEEL_ORIGINS body frame rotated into field space,
# then stamp EACH wheel's contact polyline as its own rut with the SAME
# mass-conserving compaction mechanism as wheel_pass (density up, mass untouched,
# height sinks). The four polylines feed wheel_tracks metadata so the shader can
# orient per-wheel cleat detail (§4.2.3) without resolving cleats in the grid.
# ---------------------------------------------------------------------------

#: IPEx wheel layout (sidecar.gd WHEEL_ORIGINS, body frame, metres). Track gauge
#: 0.57 m (wheels at lateral +/-0.285), wheelbase 0.40 m (wheels fore/aft +/-0.20).
#: asce-es-2024-isru-pilot-excavator-wheel-testing.pdf characterizes the IPEx wheel
#: on simulant; we reproduce only the OBSERVABLE 4-rut footprint, not contact mechanics.
WHEEL_GAUGE_M = 0.57
WHEEL_BASE_M = 0.40


def wheel_contact_points(center_rc: tuple[float, float], heading_rad: float, *,
                         cell_m: float, gauge_m: float = WHEEL_GAUGE_M,
                         wheelbase_m: float = WHEEL_BASE_M) -> dict[str, tuple[float, float]]:
    """Field-space (row,col) contact centers of the 4 wheels for one rover pose.

    LABELLING (DOCUMENTED, consistent with sidecar.gd WHEEL_ORIGINS keys):
        F/B = Front/Back along the body FORE axis (F = +wheelbase/2, B = -wheelbase/2);
        L/R = Left/Right along the body LATERAL axis (L = +gauge/2, R = -gauge/2).
      So LF=(+fore,+left), RF=(+fore,-left), LB=(-fore,+left), RB=(-fore,-left).

    HEADING (INTERFACE.md §5.2): heading_rad 0 = +col/+X, +pi/2 = +row/+Z. The
    field-space FORWARD unit (drow,dcol) = (sin h, cos h); LATERAL (left) =
    (cos h, -sin h) (a +90 deg / left-hand rotation of forward in (row,col)).

    A wheel at body offset (fore, lateral) lands at the rover center plus
    (fore/cell_m)*forward + (lateral/cell_m)*lateral, in FRACTIONAL cells (the
    caller rounds when rasterizing). Returns {"LF","RF","LB","RB"} -> (row,col).
    """
    r0, c0 = center_rc
    sh, ch = np.sin(heading_rad), np.cos(heading_rad)
    fwd = np.array([sh, ch])            # forward unit in (row,col)
    lat = np.array([ch, -sh])           # left unit in (row,col)
    half_base = 0.5 * wheelbase_m / cell_m
    half_gauge = 0.5 * gauge_m / cell_m
    out: dict[str, tuple[float, float]] = {}
    for key, (fore_sign, lat_sign) in (("LF", (+1, +1)), ("RF", (+1, -1)),
                                       ("LB", (-1, +1)), ("RB", (-1, -1))):
        off = fore_sign * half_base * fwd + lat_sign * half_gauge * lat
        out[key] = (r0 + float(off[0]), c0 + float(off[1]))
    return out


def four_wheel_pass(cs: ColumnState, poses: list[tuple[tuple[float, float], float]], *,
                    wheel_width_m: float = 0.18,
                    compaction: float = 0.12) -> dict[str, list[tuple[float, float]]]:
    """Stamp FOUR separate compacting ruts (LF/RF/LB/RB) along a pose sequence. MASS PRESERVED.

    ``poses`` is a list of (center_rc, heading_rad) — the rover-center track this drive.
    For each pose we compute the 4 wheel contact centers (wheel_contact_points) and
    accumulate, per wheel, a contact polyline. Each polyline is then stamped as its OWN
    rut using the SAME mechanism as wheel_pass (spec §6): under each wheel's footprint
    density goes UP capped at RHO_DEEP, mass_areal is untouched so the column thins and
    the rut SINKS, state -> TREAD (SPOIL -> COMPACTED_BERM), disturbance bumped.

    Mass is conserved exactly (density-only edit; height re-derived via derive_height()).
    Returns {"LF","RF","LB","RB"} -> list of (row,col) FLOAT contact centers, so a scene
    can build the INTERFACE.md §5.2 wheel_tracks metadata (build_wheel_tracks_meta).

    GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE (module docstring; spec §9): we lay the
    observable 4-rut footprint of the IPEx layout (asce-es-2024 wheel), not slip-sinkage.
    """
    half_w = max(0.5, 0.5 * wheel_width_m / cs.cell_m)  # half-width in cells

    # Per-wheel contact polylines (float centers, used both for stamping + metadata).
    polylines: dict[str, list[tuple[float, float]]] = {"LF": [], "RF": [], "LB": [], "RB": []}
    for (center_rc, heading_rad) in poses:
        pts = wheel_contact_points(center_rc, heading_rad, cell_m=cs.cell_m)
        for key in polylines:
            polylines[key].append(pts[key])

    # Stamp each wheel's rut independently (its own disc sweep), exactly as wheel_pass
    # does for a single track. Density-only -> mass conserved.
    for key in polylines:
        touched = np.zeros((cs.height, cs.width), dtype=bool)
        for (r, c) in polylines[key]:
            touched |= _wheel_mask(cs, (r, c), half_w)
        if not touched.any():
            continue
        cs.density[touched] = np.minimum(cs.density[touched] * (1.0 + compaction), K.RHO_DEEP)
        was_spoil = touched & (cs.state_label == StateLabel.SPOIL)
        cs.state_label[touched & ~was_spoil] = StateLabel.TREAD
        cs.state_label[was_spoil] = StateLabel.COMPACTED_BERM
        cs.disturbance[touched] = np.clip(cs.disturbance[touched] + 0.35, 0.0, 1.0)

    return polylines


def build_wheel_tracks_meta(polylines: dict[str, list[tuple[float, float]]],
                            headings: dict[str, float] | float, *,
                            cell_m: float, width_m: float = 0.18,
                            slip: dict[str, float] | float | None = None) -> dict[str, dict]:
    """Shape the §5.2 ``wheel_tracks`` metadata dict from four_wheel_pass output.

    Returns EXACTLY the INTERFACE.md §5.2 shape (consumers MAY ignore it; additive only):
        {"LF": {"points": [[r,c],...] BASE-cell ints,
                "heading_rad": float,           # travel dir, 0=+col/+X, +pi/2=+row/+Z
                "slip": float (OPTIONAL),        # [0,1], omitted if None
                "width_m": float SI metres},     # contact band width (NOT cells)
         "RF": ..., "LB": ..., "RB": ...}

    ``points`` are [row,col] BASE-cell ints (rounded float contacts, INTERFACE.md §5.2);
    ``width_m`` (and any *_m) stay SI metres. ``headings``/``slip`` may be a single value
    applied to all four wheels, or a per-wheel dict keyed LF/RF/LB/RB.
    """
    def _per_wheel(val, key):
        return val.get(key) if isinstance(val, dict) else val

    out: dict[str, dict] = {}
    for key in ("LF", "RF", "LB", "RB"):
        pts = polylines.get(key, [])
        ipts = [[int(round(r)), int(round(c))] for (r, c) in pts]
        entry: dict = {
            "points": ipts,
            "heading_rad": float(_per_wheel(headings, key)),
            "width_m": float(width_m),  # SI metres (§5.2), NOT cells
        }
        s = _per_wheel(slip, key)
        if s is not None:
            entry["slip"] = float(s)  # OPTIONAL [0,1] (§5.2), omitted when absent
        out[key] = entry
    return out


# ---------------------------------------------------------------------------
# Drum dig events — EXCAVATED/SPOIL swath + teeth marks (spec §5 "Drum dig
# events", §4.2.4 drum teeth marks; INTERFACE.md §5.2 drum_marks).
#
# The mass transfer already exists in column_state (cut_to_inventory /
# dump_from_inventory): a counter-rotating RASSOR drum (2021-ASCEND-Mass-
# Inference-RASSOR.pdf) cuts a band into the drum inventory and optionally dumps
# it as SPOIL elsewhere — mass conserved via the drum inventory. We add the
# convenience that relabels the cut band EXCAVATED (cut_to_inventory leaves the
# label untouched) and the §5.2 drum_marks metadata builder.
# ---------------------------------------------------------------------------

#: RASSOR drum teeth geometry, used only for the §5.2 drum_marks metadata (shader
#: detail, never grid geometry — spec §4.2.4). 2021-ASCEND-Mass-Inference-RASSOR.pdf:
#: counter-rotating bucket drums with a periodic scoop/teeth pattern; we expose the
#: teeth count/pitch so the shader can phase the teeth normals + POM (spec §4.2.4).
DRUM_TEETH_COUNT = 8
DRUM_TEETH_PITCH_M = 0.025  # ~2.5 cm scoop pitch (RASSOR drum; shader-only detail)


def _swath_mask(cs: ColumnState, swath_rc: list[tuple[float, float]], half_width_cells: float) -> np.ndarray:
    """Union of disc footprints along a swath centerline (same idiom as _wheel_mask)."""
    mask = np.zeros((cs.height, cs.width), dtype=bool)
    for (r, c) in swath_rc:
        mask |= _wheel_mask(cs, (r, c), half_width_cells)
    return mask


def drum_pass(cs: ColumnState, swath_rc: list[tuple[float, float]], *,
              depth_m: float, width_m: float = 0.20,
              dump_rc: list[tuple[float, float]] | None = None) -> float:
    """Dig a band to EXCAVATED (and optionally DUMP it as SPOIL), in-place. MASS PRESERVED.

    Cuts a swath ``width_m`` wide along ``swath_rc`` down to ``depth_m`` of column
    thickness, transferring the removed areal mass into the drum inventory via
    ``column_state.cut_to_inventory`` (mass leaves the grid into the drum — conserved).
    The cut cells are then relabelled EXCAVATED (cut_to_inventory leaves labels alone)
    and their disturbance bumped. If ``dump_rc`` is given, the freshly excavated mass is
    redeposited there as SPOIL via ``column_state.dump_from_inventory`` (bulking, spec §7:
    same mass at loose spoil density occupies more height). Returns the kg excavated.

    Per-cell cut mass = depth_m * local_density (areal kg/m^2), clamped so mass_areal>=0.
    Counter-rotating RASSOR drum cancels horizontal reaction (spec §9; 2021-ASCEND-Mass-
    Inference-RASSOR.pdf) — we model the OBSERVABLE excavated swath, not the cut mechanics.
    """
    half_w = max(0.5, 0.5 * width_m / cs.cell_m)
    cut = _swath_mask(cs, swath_rc, half_w)
    if not cut.any():
        return 0.0

    # Areal mass to remove per cell = depth * local bulk density (kg/m^2). cut_to_inventory
    # clamps to available mass and books the absolute kg into drum_inventory (conserved).
    mass_per_cell = depth_m * cs.density
    moved_kg = cs.cut_to_inventory(cut, mass_per_cell)

    # Relabel the dug band EXCAVATED (cut_to_inventory only moves mass) + bump disturbance.
    cs.state_label[cut] = StateLabel.EXCAVATED
    cs.disturbance[cut] = np.clip(cs.disturbance[cut] + 0.4, 0.0, 1.0)

    if dump_rc is not None:
        dump = _swath_mask(cs, dump_rc, half_w)
        if dump.any():
            cs.dump_from_inventory(dump, moved_kg)  # SPOIL at loose density (bulking)
    return moved_kg


def build_drum_marks_meta(swath_rc: list[tuple[float, float]], heading_rad: float, *,
                          drum: str, depth_m: float, width_m: float = 0.20,
                          teeth_count: int = DRUM_TEETH_COUNT,
                          teeth_pitch_m: float = DRUM_TEETH_PITCH_M,
                          phase: float = 0.0, cell_m: float) -> dict:
    """Shape a single INTERFACE.md §5.2 ``drum_marks`` ENTRY (the scene wraps a list).

    Returns one entry of the §5.2 drum_marks list (additive, consumers MAY ignore it):
        {"drum": "front"|"back",
         "swath": [[r,c],...] BASE-cell ints,    # dug-band centerline (row-major, §2/§3)
         "depth_m": float SI metres, "width_m": float SI metres,
         "teeth_count": int, "teeth_pitch_m": float SI metres, "phase": float}

    ``heading_rad`` (0=+col/+X, +pi/2=+row/+Z) orients the transverse teeth ridge; the
    teeth params are SHADER detail only (spec §4.2.4 teeth normals + POM), never grid
    geometry. teeth_count/teeth_pitch_m default to the RASSOR drum signature (2021-ASCEND-
    Mass-Inference-RASSOR.pdf: counter-rotating bucket drum, periodic scoop teeth). All
    *_m are SI metres; only ``swath`` is [row,col] base cells (convert via cell_m).
    """
    return {
        "drum": str(drum),
        "swath": [[int(round(r)), int(round(c))] for (r, c) in swath_rc],
        "depth_m": float(depth_m),
        "width_m": float(width_m),  # SI metres (§5.2), NOT cells
        "teeth_count": int(teeth_count),
        "teeth_pitch_m": float(teeth_pitch_m),  # SI metres
        "phase": float(phase),
    }
