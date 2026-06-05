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
from . import terramechanics as tm
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
                    compaction: float = 0.12,
                    physical: bool = False,
                    loads: "dict[str, float] | float | None" = None,
                    params: "tm.TerramechanicsParams | None" = None,
                    contact_len_m: float | None = None,
                    slip: "dict[str, float] | float | None" = None) -> dict[str, list[tuple[float, float]]]:
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

    LOAD-BEARING opt-in (``physical=True``, added 2026-06-01): the per-wheel compaction
    is computed from a real Bekker pressure-sinkage solve (terramechanics.py) instead of
    the constant ``compaction`` — making the previously-decorative moduli load-bearing.
    ``loads`` gives the per-wheel normal load [N] (dict keyed LF/RF/LB/RB, a scalar for
    all four, or None -> the sourced 30 kg-class static dry load); ``params`` selects the
    TerramechanicsParams set (None -> constants.py defaults; .lunar()/.scm_oracle() also
    valid). Still a density-only edit -> mass conserved exactly. ``physical=False``
    (default) is byte-identical to the prior constant-compaction behaviour.
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
    # Snapshot the SPOIL mask ONCE before any wheel stamps, and accumulate the union of all
    # four wheels' footprints. The state relabel is applied AFTER the loop against this pre-pass
    # snapshot — otherwise an earlier wheel's SPOIL->COMPACTED_BERM is seen as "not SPOIL" by a
    # later OVERLAPPING wheel and clobbered back to TREAD (front/rear wheels share a row line on
    # a straight crest sweep), yielding zero standing berm. Density is still edited per wheel
    # (monotone, capped — order-independent), so a cell under two wheels compacts twice.
    spoil0 = cs.state_label == StateLabel.SPOIL
    any_touched = np.zeros((cs.height, cs.width), dtype=bool)
    for key in polylines:
        touched = np.zeros((cs.height, cs.width), dtype=bool)
        for (r, c) in polylines[key]:
            touched |= _wheel_mask(cs, (r, c), half_w)
        if not touched.any():
            continue
        if physical:
            load_n = loads.get(key) if isinstance(loads, dict) else loads
            if load_n is None:
                load_n = tm.static_wheel_load_n()
            s_wheel = slip.get(key) if isinstance(slip, dict) else slip
            f = tm.physical_compaction_field(
                cs.density[touched], cs.mass_areal[touched], load_n,
                params=params, contact_len_m=contact_len_m, contact_width_m=wheel_width_m,
                slip=(s_wheel or 0.0))
            cs.density[touched] = np.minimum(cs.density[touched] * (1.0 + f), K.RHO_DEEP)
        else:
            cs.density[touched] = np.minimum(cs.density[touched] * (1.0 + compaction), K.RHO_DEEP)
        any_touched |= touched

    if any_touched.any():
        was_spoil = any_touched & spoil0
        cs.state_label[any_touched & ~was_spoil] = StateLabel.TREAD   # fresh rut over non-spoil
        cs.state_label[was_spoil] = StateLabel.COMPACTED_BERM         # driving over spoil firms it
        cs.disturbance[any_touched] = np.clip(cs.disturbance[any_touched] + 0.35, 0.0, 1.0)

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
# Kinematic terrain conform — rest pose (tilt + seat height) from 4 wheel contacts.
#
# GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE (module docstring; spec §9). We seat the
# rover ON its 4 wheel contacts (real DEM heights, riding OVER clasts) and least-squares
# fit a rigid plane -> the resting tilt (surface normal) + seat height. NO contact forces,
# NO settling, NO slip: that path-dependent dynamics is the deferred Chrono::Vehicle + SCM
# job (README §4 #2-3). This is the surrogate "rover pose producer" sitting behind the same
# INTERFACE seam a real Chrono::Vehicle would later emit (up-normal + height), zero consumer
# change. Clasts are Python-authored (procgen.sample_boulders -> metadata.clasts), so the
# wheel can ride a half-buried boulder instead of clipping it (README §4 "passes through
# clasts" fixed GEOMETRICALLY here, not as collision dynamics).
# ---------------------------------------------------------------------------


def _bilinear_height(heightmap: np.ndarray, row: float, col: float) -> float:
    """Bilinear DEM height at FRACTIONAL (row,col); clamps to grid bounds.

    The Python authority stores height as a [row,col] array (column_state.derive_height);
    Godot samples it bilinearly (state_fields.height_uv). We mirror that here so the 4 wheel
    contacts read a smooth gradient even when the wheelbase is sub-cell (coarse base grids).
    """
    h, w = heightmap.shape
    r = min(max(float(row), 0.0), h - 1.0)
    c = min(max(float(col), 0.0), w - 1.0)
    r0 = int(np.floor(r)); c0 = int(np.floor(c))
    r1 = min(r0 + 1, h - 1); c1 = min(c0 + 1, w - 1)
    tr = r - r0; tc = c - c0
    top = heightmap[r0, c0] * (1.0 - tc) + heightmap[r0, c1] * tc
    bot = heightmap[r1, c0] * (1.0 - tc) + heightmap[r1, c1] * tc
    return float(top * (1.0 - tr) + bot * tr)


def _clast_contact_height(clasts: list[dict], x: float, z: float, dem_h: float,
                          climb_limit_m: float) -> float:
    """Max of the DEM height and any clast SPHERE-CAP surface at world (x,z) — ride-over.

    A clast is a Python-authored sphere (metadata.clasts): center_m=[cx,cy,cz], radius_m=r.
    A wheel contact landing within a clast's horizontal footprint rests on the cap at
    cy + sqrt(r^2 - d_horiz^2) when that exceeds the DEM. Geometric contact, not collision.

    ``climb_limit_m`` bounds the rise above the DEM to ~one wheel radius: a rigid wheel
    climbs onto a boulder's SHOULDER, it cannot balance on the apex of a boulder taller than
    itself (boulders that large are obstacles a planner routes around — not modelled here;
    flagged in drive_spiral.py). Without this the cap of a fully-exposed 0.8 m boulder lifts a
    wheel ~1.6 m and the rover lurches to >35deg, which is an artefact, not terramechanics.
    """
    best = dem_h
    ceil = dem_h + max(climb_limit_m, 0.0)
    for cl in clasts:
        ctr = cl.get("center_m")
        if ctr is None:
            continue
        cx, cy, cz = float(ctr[0]), float(ctr[1]), float(ctr[2])
        r = float(cl.get("radius_m", 0.0))
        if r <= 0.0:
            continue
        d2 = (x - cx) ** 2 + (z - cz) ** 2
        if d2 >= r * r:
            continue
        cap = cy + float(np.sqrt(r * r - d2))   # sphere top surface at (x,z)
        cap = min(cap, ceil)                     # rigid-wheel climb limit (shoulder, not apex)
        if cap > best:
            best = cap
    return best


#: Rigid-wheel climb limit for clast ride-over (m). IPEx wheel radius ~0.18 m
#: (asce-es-2024; sidecar.gd WHEEL bottom y=-0.179): a wheel climbs onto a boulder's
#: shoulder at most ~one radius, never balances on the apex of a taller boulder.
WHEEL_RADIUS_M = 0.18


def conform_pose(heightmap: np.ndarray, center_rc: tuple[float, float], heading_rad: float, *,
                 cell_m: float, world_x0: float = 0.0, world_y0: float = 0.0,
                 clasts: list[dict] | None = None, climb_limit_m: float = WHEEL_RADIUS_M,
                 min_grad_cells: float = 2.5,
                 gauge_m: float = WHEEL_GAUGE_M, wheelbase_m: float = WHEEL_BASE_M,
                 payload_kg: float = 0.0,
                 rover_mass_dry_kg: float = K.ROVER_MASS_DRY_KG,
                 g: float = K.g) -> dict:
    """Kinematic rest pose of the 4-wheel rover on the terrain at one spiral pose.

    TWO terms, least-squares fit to a plane y = a*x + b*z + c in GODOT WORLD axes
    (x = world_x0 + col*cell_m, z = world_y0 + row*cell_m, y up):

      1. MACRO terrain slope over a RESOLVABLE stencil (radius = max(half-wheelbase,
         ``min_grad_cells``*cell_m)). On a coarse base grid the 0.4 m wheelbase is sub-cell,
         so a literal 4-wheel plane fit JITTERS as contacts cross cell boundaries (dead-flat
         one frame, >25deg the next, untethered from the visible relief). Fitting the slope
         over >= a few cells reports the tilt the rover actually follows at the DEM's
         resolvable scale -- honest under-resolution, not sub-cell noise. On a FINE grid the
         stencil collapses to the real wheelbase (half-wheelbase dominates).
      2. CLAST ride-over at the 4 real wheel contacts: a wheel on a half-buried boulder rises
         onto its shoulder (capped at ``climb_limit_m``), tilting the plane there. With no
         clast the 4 contacts lie exactly on the macro plane, so the fit recovers the smooth
         macro tilt unchanged.

    Returns:
        up      : surface normal in GODOT world axes, normalize(-a, 1, -b) -- the rover's
                  local +Y after conform; the sidecar tilts Basis(UP,yaw) onto it.
        z_m     : plane height at the rover center (seat height; informational).
        pitch_rad / roll_rad : slope along the body FORWARD / LEFT axis (display/debug;
                  the load-bearing tilt is ``up``, sign-convention-free).
        contacts: {LF,RF,LB,RB} -> [row,col] field cells.

    ``heading_rad`` is the §5.2 FIELD travel-heading (0=+col/+X, +pi/2=+row/+Z), the
    ``wheel_contact_points`` convention (NOT the negated Godot rover yaw).
    GEOMETRY/STATE-ACCURATE, NOT FORCE-ACCURATE (spec §9).
    """
    clasts = clasts or []
    r0, c0 = float(center_rc[0]), float(center_rc[1])
    xc = world_x0 + c0 * cell_m
    zc = world_y0 + r0 * cell_m
    sh, ch = np.sin(heading_rad), np.cos(heading_rad)
    fwd = (ch, sh)            # forward world (x,z)
    lat = (-sh, ch)           # left world (x,z)

    # --- (1) MACRO slope over a resolvable stencil (center +- grad_r along fwd/lat) -------
    grad_r = max(0.5 * wheelbase_m, max(min_grad_cells, 0.0) * cell_m)
    sten = np.empty((4, 3), dtype=np.float64)
    sten_h = np.empty(4, dtype=np.float64)
    for i, (ox, oz) in enumerate((fwd, (-fwd[0], -fwd[1]), lat, (-lat[0], -lat[1]))):
        x = xc + grad_r * ox
        z = zc + grad_r * oz
        sten[i] = (x, z, 1.0)
        sten_h[i] = _bilinear_height(heightmap, (z - world_y0) / cell_m, (x - world_x0) / cell_m)
    (a_dem, b_dem, c_dem), *_ = np.linalg.lstsq(sten, sten_h, rcond=None)

    # --- (2) wheel contacts: macro height + capped clast ride-over -----------------------
    pts = wheel_contact_points(center_rc, heading_rad, cell_m=cell_m,
                               gauge_m=gauge_m, wheelbase_m=wheelbase_m)
    rows = ("LF", "RF", "LB", "RB")
    A = np.empty((4, 3), dtype=np.float64)
    hv = np.empty(4, dtype=np.float64)
    for i, key in enumerate(rows):
        row, col = pts[key]
        x = world_x0 + col * cell_m
        z = world_y0 + row * cell_m
        dem_local = _bilinear_height(heightmap, row, col)
        rise = _clast_contact_height(clasts, x, z, dem_local, climb_limit_m) - dem_local
        A[i] = (x, z, 1.0)
        hv[i] = (a_dem * x + b_dem * z + c_dem) + max(0.0, rise)
    (a, b, c), *_ = np.linalg.lstsq(A, hv, rcond=None)

    z_center = float(a * xc + b * zc + c)
    nrm = np.array([-a, 1.0, -b], dtype=np.float64)
    nrm /= np.linalg.norm(nrm)
    slope_fwd = a * ch + b * sh           # forward world (x,z)=(cos h, sin h)
    slope_lat = a * (-sh) + b * ch        # left world=(-sin h, cos h)
    # Per-wheel NORMAL load (for load-bearing sinkage): the weight
    # component along the surface normal, split equally over the 4 contacts. nrm[1] is
    # cos(tilt) (flat -> full weight; steeper -> less normal load -> the slip driver).
    # Equal split; CG-based fore/aft transfer is a refinement (CG height not
    # in the public TRL-5 overview). Feeds four_wheel_pass(physical=True, loads=...).
    total_weight_n = (rover_mass_dry_kg + max(0.0, payload_kg)) * float(g)
    normal_total_n = total_weight_n * float(nrm[1])
    per_wheel_n = normal_total_n / 4.0
    return {
        "up": [float(nrm[0]), float(nrm[1]), float(nrm[2])],
        "z_m": z_center,
        "pitch_rad": float(np.arctan(slope_fwd)),
        "roll_rad": float(np.arctan(slope_lat)),
        "contacts": {k: [float(pts[k][0]), float(pts[k][1])] for k in rows},
        "normal_loads": {k: per_wheel_n for k in rows},
        "normal_load_total_n": normal_total_n,
    }


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


# ---------------------------------------------------------------------------
# Differential-drive kinematic step (closes the loop).
#
# The missing (state, command) -> next_pose primitive: today the rover replays a
# precomputed path (spiral_path/drive_spiral); this lets a controller (ROS cmd_vel
# or an RL policy) DRIVE it one twist at a time. Same FIELD heading convention as
# wheel_contact_points / conform_pose (heading 0 = +col/+X, +pi/2 = +row/+Z;
# forward unit in (row,col) = (sin yaw, cos yaw)) so the integrated pose feeds
# straight into conform_pose + four_wheel_pass with zero convention juggling.
# ---------------------------------------------------------------------------

def step_pose(center_rc: tuple[float, float], yaw_rad: float,
              v_mps: float, omega_radps: float, dt_s: float, *,
              cell_m: float) -> tuple[tuple[float, float], float]:
    """Advance a unicycle/differential-drive pose by one twist over ``dt_s``.

    ``v_mps`` forward speed, ``omega_radps`` yaw rate. Exact constant-twist arc
    integration (deterministic): straight line when |omega| ~ 0, else a circular
    arc of radius v/omega. Returns ((row, col), yaw_rad), yaw wrapped to (-pi, pi].

    Forward unit in (row,col) is (sin yaw, cos yaw) (the wheel_contact_points
    convention), so yaw=0 advances +col and yaw=pi/2 advances +row.
    """
    r0, c0 = float(center_rc[0]), float(center_rc[1])
    yaw0 = float(yaw_rad)
    new_yaw = yaw0 + omega_radps * dt_s
    dist_cells = (v_mps * dt_s) / cell_m
    if abs(omega_radps) < 1e-9:
        dr = dist_cells * np.sin(yaw0)
        dc = dist_cells * np.cos(yaw0)
    else:
        # ∫ v·(sin yaw, cos yaw) dt over the constant-omega arc:
        #   drow = (v/ω)(cos yaw0 - cos yaw1);  dcol = (v/ω)(sin yaw1 - sin yaw0)
        v_cells = dist_cells / dt_s
        k = v_cells / omega_radps
        dr = k * (np.cos(yaw0) - np.cos(new_yaw))
        dc = k * (np.sin(new_yaw) - np.sin(yaw0))
    new_yaw = (new_yaw + np.pi) % (2.0 * np.pi) - np.pi
    return (r0 + float(dr), c0 + float(dc)), float(new_yaw)
