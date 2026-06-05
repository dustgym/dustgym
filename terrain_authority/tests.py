"""Conservation-invariant assert-runner (spec §10). No pytest dependency.

    python -m terrain_authority.tests

Checks (spec §10):
  1. Total mass Σ(mass_areal·cell_area) + drum_inventory is constant across a full
     cut -> dump -> relax cycle (invariant 1).
  2. heightmap == datum + mass_areal/density everywhere after every op (invariant 2).
  3. Sandpile relaxation conserves mass AND leaves every loose slope <= theta_r.

Prints PASS/FAIL per check; exits nonzero if any check fails.
"""

from __future__ import annotations


import numpy as np

from . import constants as K
from . import procgen
from . import refinement
from .column_state import ColumnState, StateLabel
from .quadtree import build_quadtree, leaves_cover_field
from .rover import (WHEEL_GAUGE_M, conform_pose,
                    four_wheel_pass, straight_path, wheel_contact_points, wheel_pass)
from .sandpile import Sandpile

_results: list[tuple[str, bool, str]] = []


def check(name: str, passed: bool, detail: str = "") -> None:
    _results.append((name, bool(passed), detail))
    tag = "PASS" if passed else "FAIL"
    print(f"[{tag}] {name}" + (f"  ({detail})" if detail else ""))


def _height_consistent(cs: ColumnState, atol: float = 1e-9) -> tuple[bool, float]:
    """heightmap == datum + mass_areal/density everywhere (spec §10 invariant 2)."""
    h = cs.derive_height()
    expect = cs.datum + cs.mass_areal / cs.density
    err = float(np.max(np.abs(h - expect)))
    return err <= atol, err


# ---------------------------------------------------------------------------
# Check 1: cut -> dump -> relax conserves Σ(mass·area)+inventory (invariant 1).
# ---------------------------------------------------------------------------

def test_cut_dump_relax_conserves_mass() -> None:
    cs = procgen.rolling_hills(64, 64, 0.03, seed=99, amplitude_m=0.1)
    m0 = cs.total_mass()

    # CUT: excavate a disc into the drum (any -> EXCAVATED, mass -> inventory).
    rows = np.arange(64)[:, None]
    cols = np.arange(64)[None, :]
    cut_mask = ((rows - 20) ** 2 + (cols - 20) ** 2) <= 8 ** 2
    cut_areal = 0.5 * cs.mass_areal  # take half the column under the drum
    cs.cut_to_inventory(cut_mask, cut_areal[cut_mask] if False else cut_areal)
    cs.state_label[cut_mask] = StateLabel.EXCAVATED
    m1 = cs.total_mass()
    ok_h1, e1 = _height_consistent(cs)

    # DUMP: deposit all drum inventory as SPOIL onto a different disc (bulking -> looser).
    dump_mask = ((rows - 44) ** 2 + (cols - 44) ** 2) <= 8 ** 2
    cs.dump_from_inventory(dump_mask, total_kg=cs.drum_inventory)
    m2 = cs.total_mass()
    ok_h2, e2 = _height_consistent(cs)

    # RELAX: sandpile the dumped pile down to repose.
    sp = Sandpile(cs, theta_r=K.THETA_R, connectivity=8)
    sp.relax_to_rest(max_steps=300)
    m3 = cs.total_mass()
    ok_h3, e3 = _height_consistent(cs)

    drift = max(abs(m1 - m0), abs(m2 - m0), abs(m3 - m0))
    rel = drift / m0
    check("invariant-1: mass constant across cut->dump->relax",
          rel < 1e-9,
          f"m0={m0:.6f} m1={m1:.6f} m2={m2:.6f} m3={m3:.6f} kg  rel_drift={rel:.2e}")
    check("invariant-2: height==datum+mass/density after each op",
          ok_h1 and ok_h2 and ok_h3,
          f"max_err cut={e1:.2e} dump={e2:.2e} relax={e3:.2e} m")


# ---------------------------------------------------------------------------
# Check 2: height-density consistency holds after procgen + rover ops.
# ---------------------------------------------------------------------------

def test_height_consistency_all_ops() -> None:
    errs = {}
    cs = procgen.flat_compact(48, 48, 0.02, seed=1)
    errs["flat_compact"] = _height_consistent(cs)[1]

    cs = procgen.rolling_hills(48, 48, 0.02, seed=2)
    errs["rolling_hills"] = _height_consistent(cs)[1]

    procgen.carve_crater(cs, (24, 24), 0.6)
    errs["carve_crater"] = _height_consistent(cs)[1]

    wheel_pass(cs, straight_path(5, 5, 40, 40), wheel_width_m=0.06)
    errs["wheel_pass"] = _height_consistent(cs)[1]

    worst = max(errs.values())
    check("invariant-2: height consistent after procgen+crater+rover",
          worst <= 1e-9,
          "  ".join(f"{k}={v:.2e}" for k, v in errs.items()))


# ---------------------------------------------------------------------------
# Check 3: rover pass preserves mass (compaction is density-only).
# ---------------------------------------------------------------------------

def test_rover_pass_preserves_mass() -> None:
    cs = procgen.rolling_hills(64, 64, 0.02, seed=4, amplitude_m=0.08)
    m0 = cs.total_mass()
    h0 = cs.derive_height().copy()
    wheel_pass(cs, straight_path(10, 10, 50, 55), wheel_width_m=0.12, compaction=0.15)
    m1 = cs.total_mass()
    h1 = cs.derive_height()
    sank = bool(np.any(h1 < h0 - 1e-6))  # rut should sink somewhere
    check("rover: single pass preserves mass (density-only compaction)",
          abs(m1 - m0) / m0 < 1e-9 and sank,
          f"m0={m0:.6f} m1={m1:.6f} kg  rut_sank={sank}")


# ---------------------------------------------------------------------------
# Check 4: sandpile conserves mass AND reaches repose on all loose cells.
# ---------------------------------------------------------------------------

def test_sandpile_conserves_and_reposes() -> None:
    cs = procgen.flat_compact(80, 80, 0.02, seed=7, amplitude_m=0.0)
    # Make the whole surface loose so relaxation is allowed everywhere.
    cs.density[:] = K.RHO_SURFACE
    cs.state_label[:] = StateLabel.VIRGIN
    # Re-back-out mass at the loose density to keep height consistent.
    cs.set_height_via_mass(cs.derive_height())
    m0 = cs.total_mass()

    sp = Sandpile(cs, theta_r=K.THETA_R, connectivity=8, transfer_fraction=0.6)
    # Drop a tall narrow cone of loose mass in the center -> far over repose.
    sp.deposit(40, 40, mass_kg=60.0, radius_cells=3)
    m_after_deposit = cs.total_mass()

    steps, _ = sp.relax_to_rest(max_steps=600)
    m1 = cs.total_mass()

    # Mass conserved across the RELAXATION (deposit adds mass; relax must not change it).
    rel = abs(m1 - m_after_deposit) / m_after_deposit
    mass_ok = rel < 1e-9

    # Every loose slope <= theta_r (+ small tolerance).
    final_max_slope = sp._max_loose_slope()
    repose_ok = final_max_slope <= K.THETA_R + np.deg2rad(1.0)

    check("invariant-1: sandpile relaxation conserves mass",
          mass_ok,
          f"m_pre={m_after_deposit:.6f} m_post={m1:.6f} kg rel_drift={rel:.2e} "
          f"(deposit raised {m_after_deposit-m0:.3f} kg) steps={steps}")
    check("spec §7: all loose slopes <= theta_r after relaxation",
          repose_ok,
          f"max_loose_slope={np.rad2deg(final_max_slope):.2f}deg "
          f"theta_r={np.rad2deg(K.THETA_R):.2f}deg")


# ---------------------------------------------------------------------------
# Check 5: round-trip I/O fidelity (float32) — save_scene/load_scene.
# ---------------------------------------------------------------------------

def test_io_roundtrip() -> None:
    import os
    import tempfile

    from .io_fields import load_scene, save_scene

    cs = procgen.rolling_hills(32, 48, 0.02, seed=8)  # non-square: catch row/col swaps
    meta = {
        "schema_version": "1.0", "scene_name": "iotest",
        "grid": {"width": 32, "height": 48, "cell_m": 0.02, "order": "row-major-C"},
    }
    with tempfile.TemporaryDirectory() as d:
        sd = os.path.join(d, "iotest")
        save_scene(sd, cs.fields_dict(), meta)
        # metadata.json must exist (written first).
        meta_ok = os.path.exists(os.path.join(sd, "metadata.json"))
        fields, meta2 = load_scene(sd)
        shape_ok = fields["heightmap"].shape == (48, 32)
        # float32 round-trip on mass_areal within float32 precision.
        rt = np.allclose(fields["mass_areal"], cs.mass_areal.astype("<f4"), rtol=0, atol=0)
        label_ok = fields["state_label"].dtype == np.uint8
    check("io: save/load round-trip (dims, dtype, row-major)",
          meta_ok and shape_ok and rt and label_ok,
          f"meta={meta_ok} shape={shape_ok} mass_rt={rt} label_u8={label_ok}")


# ---------------------------------------------------------------------------
# Check 6: interaction-keyed quadtree (spec §4 "the tree manages space, keyed
# to interaction"). Three properties: (a) the leaf set tiles the field with no
# gaps/overlap, (b) promotion is monotone toward the rover — the leaf CONTAINING
# the rover footprint is at min_leaf (finest), a node FAR from the rover stays
# above min_leaf (coarse), and the box-distance-to-rover is non-decreasing with
# leaf size; (c) the active (fine) leaf count is bounded along the whole drive.
# ---------------------------------------------------------------------------

def test_quadtree_space_management() -> None:
    field_size = 256
    min_leaf = 8
    refine_factor = 0.5
    footprint_radius = 5.5  # cells (~22 cm wheel half-width at 0.02 m/cell)

    # --- (a) the leaf set tiles the field exactly once (no gaps/overlap) -----------
    rover = (128, 133)
    res = build_quadtree(field_size, rover, min_leaf=min_leaf,
                         refine_factor=refine_factor,
                         footprint_radius_cells=footprint_radius)
    cover_ok, n_once = leaves_cover_field(res)
    check("quadtree: leaves tile the field exactly once (no gaps/overlap)",
          cover_ok and n_once == field_size * field_size,
          f"covered_once={n_once}/{field_size * field_size} leaves={len(res.leaves)}")

    # --- (b) promotion is monotone toward the rover --------------------------------
    # Leaf containing the rover footprint center is at the finest (min_leaf) level.
    rr, rc = rover
    rover_leaf_size = next(
        (r1 - r0) for (r0, c0, r1, c1) in res.leaves if r0 <= rr < r1 and c0 <= rc < c1)
    rover_is_fine = rover_leaf_size == min_leaf
    # A node in the far corner (opposite the rover) stays coarse (> min_leaf).
    far_leaf_size = next(
        (r1 - r0) for (r0, c0, r1, c1) in res.leaves if r0 <= 4 < r1 and c0 <= 4 < c1)
    far_is_coarse = far_leaf_size > min_leaf
    # Monotonicity: bin leaves by size, and assert the MINIMUM box-distance-to-rover
    # is non-decreasing as leaf size grows (bigger leaves only live farther out).
    from .quadtree import _box_chebyshev_distance
    by_size: dict[int, float] = {}
    for (r0, c0, r1, c1) in res.leaves:
        d = _box_chebyshev_distance(r0, c0, r1, c1, rr, rc)
        sz = r1 - r0
        by_size[sz] = min(by_size.get(sz, np.inf), d)
    sizes_sorted = sorted(by_size)
    min_dists = [by_size[s] for s in sizes_sorted]
    monotone = all(min_dists[i] <= min_dists[i + 1] + 1e-9 for i in range(len(min_dists) - 1))
    check("quadtree: promotion monotone toward rover (rover leaf fine, far leaf coarse)",
          rover_is_fine and far_is_coarse and monotone,
          f"rover_leaf={rover_leaf_size} far_leaf={far_leaf_size} "
          f"min_dist_by_size={[f'{s}:{d:.0f}' for s, d in zip(sizes_sorted, min_dists)]}")

    # --- (c) active (fine) leaf count is bounded along the whole drive -------------
    # Replay the SAME tread-track rover positions and assert the active set never blows
    # up (LOD budget is finite) and that as the rover moves the active cluster MOVES
    # (its centroid tracks the rover, not stuck at the start).
    from .scenes import _tread_path_endpoints
    cr0, cc0, cr1, cc1, cr2, cc2 = _tread_path_endpoints()
    path = straight_path(cr0, cc0, cr1, cc1, 1) + straight_path(cr1, cc1, cr2, cc2, 1)[1:]
    chunks = np.array_split(np.arange(len(path)), 31)
    positions = [None] + [[path[k] for k in ch][-1] for ch in chunks if len(ch)]

    MAX_ACTIVE = 64  # generous LOD budget; observed peak is well under this
    max_active = 0
    centroids = []
    for pos in positions:
        r = build_quadtree(field_size, pos, min_leaf=min_leaf,
                           refine_factor=refine_factor,
                           footprint_radius_cells=footprint_radius)
        max_active = max(max_active, len(r.active_leaves))
        if r.active_leaves:
            arr = np.array([[(a + c) / 2, (b + d) / 2] for (a, b, c, d) in r.active_leaves])
            centroids.append((pos, arr.mean(axis=0)))
    bounded = max_active <= MAX_ACTIVE
    # The active-cluster centroid stays near the rover it is keyed to (within ~min_leaf*4).
    tracks = all(
        abs(cent[0] - pos[0]) <= min_leaf * 4 and abs(cent[1] - pos[1]) <= min_leaf * 4
        for pos, cent in centroids)
    check("quadtree: active-leaf count bounded + cluster tracks the rover along the drive",
          bounded and tracks,
          f"max_active={max_active} (budget {MAX_ACTIVE}); cluster_tracks_rover={tracks} "
          f"over {len(positions)} frames")


# ---------------------------------------------------------------------------
# Render-fidelity acceptance tests (render_fidelity_spec.md §6). The variable-
# resolution operators (refinement.py) and the 4-wheel / drum producer ops
# (rover.py) just landed; these assert the NORMATIVE §2.4 mass-conservation
# invariants and the §5.2/§5.3 producer geometry against them.
# ---------------------------------------------------------------------------

def _mixed_fine_bundle(h: int, w: int, k: int, *, seed: int = 0,
                       ) -> dict[str, np.ndarray]:
    """A genuinely heterogeneous (H*k, W*k) fine bundle (NOT a refine() of anything).

    Random per-cell mass/density/datum/disturbance and per-cell state labels, all in
    physical ranges (density>0 everywhere — the §2.4 zero-mass precondition), so coarsen()
    exercises the general harmonic-mean / priority-reduce paths rather than the bit-exact
    uniform-block shortcut. Deterministic via ``seed``.
    """
    rng = np.random.default_rng(seed)
    hf, wf = h * k, w * k
    labels = np.array([int(StateLabel.VIRGIN), int(StateLabel.TREAD),
                       int(StateLabel.EXCAVATED), int(StateLabel.SPOIL),
                       int(StateLabel.COMPACTED_BERM)], dtype=np.uint8)
    return {
        "mass_areal": rng.uniform(0.0, 250.0, size=(hf, wf)),
        "density": rng.uniform(K.RHO_SURFACE, K.RHO_DEEP, size=(hf, wf)),
        "datum": rng.uniform(-0.3, 0.3, size=(hf, wf)),
        "state_label": labels[rng.integers(0, labels.size, size=(hf, wf))],
        "disturbance": rng.uniform(0.0, 1.0, size=(hf, wf)),
    }


def _bundle_height(b: dict[str, np.ndarray]) -> np.ndarray:
    """height = datum + mass_areal/density on a raw field bundle (INTERFACE.md §4)."""
    return b["datum"] + b["mass_areal"] / b["density"]


# §6.1 refine/coarsen round-trip exact -------------------------------------------------

def test_refine_coarsen_roundtrip() -> None:
    """§6.1: coarsen(refine(cell)) == cell EXACTLY; mass drift 0; height max-err 0.

    Checked across SEVERAL refine factors k ∈ {2, 3, 5, 8} — deliberately including
    non-power-of-two k and k=8 (the spec §2.5/§8 mission config "base 8 cm + 1 cm touched
    band"). This matters: ``np.mean`` of k^2 identical float64s is only bit-exact when the
    k^2 sum/divide round-trips (k=2, 4); for k=3/5/6/7/8 a plain mean drifts ~1e-13. The
    operators copy homogeneous blocks VERBATIM (refinement._uniform_aware_mean + the density
    uniform branch), so the round-trip is drift-0 for EVERY integer k — this test locks that.
    """
    ks = [2, 3, 5, 8]
    worst = {"field": 0.0, "h_refine": 0.0, "h_back": 0.0, "mass_copy": 0.0}
    labels_ok = True
    for k in ks:
        base = _mixed_fine_bundle(7, 5, 1, seed=11)  # arbitrary (H, W) base bundle
        fine = refinement.refine_field(base, k)
        back = refinement.coarsen_field(fine, k)

        # Every carried field round-trips BIT-EXACT (refine copies -> uniform block -> the
        # coarsen reductions are exact identities for all k via the verbatim-copy shortcut).
        for name in ("mass_areal", "density", "datum", "disturbance"):
            worst["field"] = max(worst["field"], float(np.max(np.abs(back[name] - base[name]))))
        labels_ok = labels_ok and bool(np.array_equal(back["state_label"], base["state_label"]))

        # Mass conservation as the INTENSIVE per-cell invariant: refine copies mass_areal
        # (kg/m^2) verbatim into all k^2 children, so each child's areal mass equals its
        # parent's to drift 0 (total mass Σ mass_areal·cell_area is then conserved because the
        # k^2 children each have 1/k^2 of the parent's plan area). Asserted as a verbatim copy
        # (NOT a Σ over k^2 terms, whose float REDUCTION ORDER differs ~1e-16 even for equal
        # values — a summation artifact, not a conservation violation).
        worst["mass_copy"] = max(worst["mass_copy"], float(np.max(np.abs(
            fine["mass_areal"] - np.repeat(np.repeat(base["mass_areal"], k, 0), k, 1)))))

        # height invariant exact across both ops, for every k.
        worst["h_refine"] = max(worst["h_refine"], float(np.max(np.abs(
            _bundle_height(fine) - np.repeat(np.repeat(_bundle_height(base), k, 0), k, 1)))))
        worst["h_back"] = max(worst["h_back"], float(np.max(np.abs(
            _bundle_height(back) - _bundle_height(base)))))

    ok = (worst["field"] == 0.0 and labels_ok and worst["mass_copy"] == 0.0
          and worst["h_refine"] == 0.0 and worst["h_back"] == 0.0)
    check("§6.1: refine/coarsen round-trip exact for k in {2,3,5,8} (drift 0, height max-err 0)",
          ok,
          f"label_ok={labels_ok} mass_copy_err={worst['mass_copy']:.2e} "
          f"h_err(refine)={worst['h_refine']:.2e} h_err(back)={worst['h_back']:.2e} "
          f"field_max_err={worst['field']:.2e}")


# §6.2 base<->tile consistency ---------------------------------------------------------

def test_base_tile_consistency() -> None:
    """§6.2: for a refined scene, every base cell over a tile == coarsen(tile children).

    Builds a small base ColumnState, drives a 4-wheel pass + a drum dig so the corridor
    carries TREAD/EXCAVATED/SPOIL labels and non-uniform density (a genuine coarsen), then
    extracts tiles over base-cell-aligned leaf boxes. For each tile, coarsen its fine cells
    back and assert mass_areal AND area-mean height match the base block exactly (the §5.3
    NORMATIVE base<->tile invariant), plus state_label/datum/disturbance.
    """
    base_cell_m, fine_cell_m = 0.02, 0.01
    k = refinement.k_factor(base_cell_m, fine_cell_m)  # 2
    cs = procgen.rolling_hills(48, 48, base_cell_m, seed=23, amplitude_m=0.1)
    # Lay TREAD ruts + an EXCAVATED/SPOIL dig so the corridor is heterogeneous.
    poses = [((r, r), np.deg2rad(45.0)) for r in np.linspace(10, 38, 15)]
    four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.12)
    from .rover import drum_pass
    swath = [(8, c) for c in range(8, 24)]
    dump = [(40, c) for c in range(8, 24)]
    drum_pass(cs, swath, depth_m=0.03, width_m=0.20, dump_rc=dump)

    # Base-cell-aligned leaf boxes over the touched corridor (8x8 blocks; k=2 divides 8).
    leaf_boxes = [[8, 8, 16, 16], [16, 16, 24, 24], [24, 24, 32, 32],
                  [0, 8, 8, 16], [40, 8, 48, 16]]
    tiles = refinement.extract_tiles(cs, leaf_boxes, fine_cell_m)

    worst_mass = 0.0
    worst_height = 0.0
    worst_datum = 0.0
    worst_dist = 0.0
    labels_ok = True
    for t in tiles:
        r0, c0, r1, c1 = t.region_rc
        coarse = refinement.coarsen_field(t.fields, k)
        base_block = {
            "mass_areal": cs.mass_areal[r0:r1, c0:c1],
            "density": cs.density[r0:r1, c0:c1],
            "datum": cs.datum[r0:r1, c0:c1],
            "state_label": cs.state_label[r0:r1, c0:c1],
            "disturbance": cs.disturbance[r0:r1, c0:c1],
        }
        worst_mass = max(worst_mass,
                         float(np.max(np.abs(coarse["mass_areal"] - base_block["mass_areal"]))))
        worst_height = max(worst_height,
                           float(np.max(np.abs(_bundle_height(coarse) - _bundle_height(base_block)))))
        worst_datum = max(worst_datum,
                          float(np.max(np.abs(coarse["datum"] - base_block["datum"]))))
        worst_dist = max(worst_dist,
                         float(np.max(np.abs(coarse["disturbance"] - base_block["disturbance"]))))
        labels_ok = labels_ok and bool(
            np.array_equal(coarse["state_label"], base_block["state_label"]))

    ok = (worst_mass == 0.0 and worst_height == 0.0 and worst_datum == 0.0
          and worst_dist == 0.0 and labels_ok and len(tiles) == len(leaf_boxes))
    check("§6.2: base<->tile consistency (mass + area-mean height) over a refined scene",
          ok,
          f"tiles={len(tiles)} max_mass_err={worst_mass:.2e} max_height_err={worst_height:.2e} "
          f"max_datum_err={worst_datum:.2e} max_dist_err={worst_dist:.2e} labels_ok={labels_ok}")


# §6.2b zero-mass coarsen --------------------------------------------------------------

def test_coarsen_zero_mass() -> None:
    """§6.2b: an all-empty (mass_areal=0) block coarsens to finite density=mean(density_fine),
    height==datum, no NaN/inf (the zero-mass branch; density_fine>0 precondition)."""
    k = 2
    h, w = 3, 3
    rng = np.random.default_rng(7)
    rho_fine = rng.uniform(K.RHO_SURFACE, K.RHO_DEEP, size=(h * k, w * k))
    datum_fine = rng.uniform(-0.2, 0.2, size=(h * k, w * k))
    fine = {
        "mass_areal": np.zeros((h * k, w * k)),  # ALL empty
        "density": rho_fine,
        "datum": datum_fine,
        "state_label": np.full((h * k, w * k), int(StateLabel.VIRGIN), dtype=np.uint8),
        "disturbance": np.zeros((h * k, w * k)),
    }
    coarse = refinement.coarsen_field(fine, k)
    rho_mean = rho_fine.reshape(h, k, w, k).mean(axis=(1, 3))
    datum_mean = datum_fine.reshape(h, k, w, k).mean(axis=(1, 3))
    height = _bundle_height(coarse)

    finite = bool(np.all(np.isfinite(coarse["density"])) and np.all(np.isfinite(height)))
    rho_ok = float(np.max(np.abs(coarse["density"] - rho_mean))) == 0.0
    height_is_datum = float(np.max(np.abs(height - datum_mean))) == 0.0
    mass_zero = float(np.max(np.abs(coarse["mass_areal"]))) == 0.0
    check("§6.2b: zero-mass coarsen -> finite density=mean(rho_fine), height==datum, no nan/inf",
          finite and rho_ok and height_is_datum and mass_zero,
          f"finite={finite} rho==mean(rho_fine)={rho_ok} height==datum={height_is_datum} "
          f"mass_zero={mass_zero}")


# §6.2c non-uniform datum --------------------------------------------------------------

def test_coarsen_nonuniform_datum() -> None:
    """§6.2c: with varied per-cell datum, coarse height == area-mean(child heights) and
    datum_coarse == mean(datum_fine)."""
    k = 2
    h, w = 4, 4
    fine = _mixed_fine_bundle(h, w, k, seed=31)  # non-uniform datum + density + mass
    coarse = refinement.coarsen_field(fine, k)

    datum_mean = fine["datum"].reshape(h, k, w, k).mean(axis=(1, 3))
    height_fine_mean = _bundle_height(fine).reshape(h, k, w, k).mean(axis=(1, 3))
    height_coarse = _bundle_height(coarse)

    datum_err = float(np.max(np.abs(coarse["datum"] - datum_mean)))
    height_err = float(np.max(np.abs(height_coarse - height_fine_mean)))
    # area-mean height = mean of child heights for equal-area children; allow float round-off.
    ok = datum_err == 0.0 and height_err <= 1e-12
    check("§6.2c: non-uniform datum -> coarse height==area-mean(child h), datum==mean(datum_fine)",
          ok, f"datum_err={datum_err:.2e} height_err={height_err:.2e}")


# §6.2d non-integer k rejected ---------------------------------------------------------

def test_non_integer_k_rejected() -> None:
    """§6.2d: a base_cell_m/fine_cell_m that is not a positive integer is REJECTED."""
    # Genuine non-integer ratio (0.02/0.012 = 1.666...) must raise; a non-positive must raise.
    rejected_noninteger = False
    try:
        refinement.k_factor(0.02, 0.012)
    except ValueError:
        rejected_noninteger = True
    rejected_nonpositive = False
    try:
        refinement.k_factor(0.02, 0.0)
    except ValueError:
        rejected_nonpositive = True
    # A genuine integer ratio must still be accepted (and absorb float division noise).
    accepted_int = refinement.k_factor(0.02, 0.01) == 2
    check("§6.2d: non-integer / non-positive k rejected (ValueError), integer k accepted",
          rejected_noninteger and rejected_nonpositive and accepted_int,
          f"noninteger_raised={rejected_noninteger} nonpositive_raised={rejected_nonpositive} "
          f"int_accepted={accepted_int}")


# §6.3 toggle equivalence --------------------------------------------------------------

def test_refinement_toggle_equivalence() -> None:
    """§6.3: building a scene with refinement DISABLED yields BYTE-IDENTICAL base rasters to
    the plain uniform pipeline (the no-op-equivalent escape hatch).

    Self-contained: build the SAME base ColumnState two ways and save both into temp dirs —
    (A) the plain uniform pipeline (just the base fields), (B) a "refinement-disabled" build
    that attaches the refinement policy block (enabled=false) + the ignorable feature flags
    but emits NO tiles. Then compare the on-disk raster bytes for the 5 REQUIRED rasters.
    refinement.enabled=false must touch neither the rasters nor any existing key.
    """
    import os
    import tempfile

    from .io_fields import save_scene

    def _build_base() -> ColumnState:
        cs = procgen.rolling_hills(40, 40, 0.02, seed=44, amplitude_m=0.1)
        poses = [((r, r), np.deg2rad(45.0)) for r in np.linspace(8, 32, 12)]
        four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.12)
        return cs

    cs_a = _build_base()
    cs_b = _build_base()
    meta_common = {
        "schema_version": "1.0", "scene_name": "toggle",
        "grid": {"width": 40, "height": 40, "cell_m": 0.02, "order": "row-major-C"},
    }
    raster_files = ["heightmap.rf32", "mass_areal.rf32", "density.rf32",
                    "disturbance.rf32", "state_label.r8"]
    with tempfile.TemporaryDirectory() as d:
        sa = os.path.join(d, "plain")
        sb = os.path.join(d, "refine_off")
        save_scene(sa, cs_a.fields_dict(), dict(meta_common))
        # B: refinement DISABLED + ignorable discoverability flags; NO tiles emitted.
        meta_b = dict(meta_common)
        meta_b["contract_revision"] = "1.0.2"
        meta_b["features"] = ["refinement", "wheel_tracks", "drum_marks"]
        meta_b["refinement"] = {
            "enabled": False, "base_cell_m": 0.02, "fine_cell_m": 0.01,
            "refine_where": "none", "fine_min_leaf": 4,
        }
        save_scene(sb, cs_b.fields_dict(), meta_b)

        all_identical = True
        details = []
        for fn in raster_files:
            with open(os.path.join(sa, fn), "rb") as fh:
                ba = fh.read()
            with open(os.path.join(sb, fn), "rb") as fh:
                bb = fh.read()
            same = ba == bb
            all_identical = all_identical and same
            if not same:
                details.append(fn)
        no_tiles = not os.path.isdir(os.path.join(sb, "tiles"))
    check("§6.3: refinement-disabled build is byte-identical to plain uniform base rasters",
          all_identical and no_tiles,
          f"rasters_identical={all_identical} no_tiles_dir={no_tiles}"
          + (f" diff={details}" if details else ""))


# §6.4 4-wheel separability ------------------------------------------------------------

def test_four_wheel_separability() -> None:
    """§6.4: after a STRAIGHT drive, exactly two TREAD bands at ~0.57 m gauge; after a TURN,
    four distinct arcs/clusters."""
    cell_m = 0.02
    gauge_cells = WHEEL_GAUGE_M / cell_m  # 0.57 m / 0.02 = 28.5 cells L<->R separation

    # --- STRAIGHT drive along +row (heading +pi/2): L and R wheels share a row-band each;
    # the two fore/aft wheels on the same side overlap into ONE band -> exactly TWO bands. ---
    cs = procgen.flat_compact(96, 96, cell_m, seed=3)
    heading = np.deg2rad(90.0)  # +row/+Z travel; lateral axis is +col/-col
    poses = [((r, 48.0), heading) for r in np.linspace(20, 76, 40)]
    four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.12)
    tread = cs.state_label == int(StateLabel.TREAD)
    # Project onto the lateral (col) axis: count contiguous TREAD column-bands.
    col_hit = tread.any(axis=0)
    n_bands_straight, band_centers = _count_bands(col_hit)
    two_bands = n_bands_straight == 2
    gauge_ok = False
    if len(band_centers) == 2:
        sep = abs(band_centers[1] - band_centers[0])
        gauge_ok = abs(sep - gauge_cells) <= 4.0  # within ~8 cm of the 28.5-cell gauge

    # --- TURNING drive: heading sweeps so the 4 wheels trace 4 distinct arcs/clusters. ---
    cs2 = procgen.flat_compact(96, 96, cell_m, seed=3)
    poses2 = []
    for t in np.linspace(0.0, 1.0, 40):
        ang = np.deg2rad(20.0 + 120.0 * t)  # heading sweeps 100 deg -> a real turn
        rr = 48.0 + 16.0 * np.sin(ang)
        cc = 48.0 + 16.0 * (1.0 - np.cos(ang))
        poses2.append(((rr, cc), ang))
    poly = four_wheel_pass(cs2, poses2, wheel_width_m=0.12, compaction=0.12)
    # Four wheels -> four contact polylines whose endpoint clusters are pairwise separated.
    centroids = {key: np.mean(np.array(poly[key]), axis=0) for key in ("LF", "RF", "LB", "RB")}
    min_sep = min(
        float(np.hypot(*(centroids[a] - centroids[b])))
        for i, a in enumerate(("LF", "RF", "LB", "RB"))
        for b in ("LF", "RF", "LB", "RB")[i + 1:])
    four_distinct = min_sep > 0.5 * gauge_cells  # clusters clearly separated (> ~half gauge)

    check("§6.4: 4-wheel separability (straight=2 bands @ gauge; turn=4 distinct clusters)",
          two_bands and gauge_ok and four_distinct,
          f"straight_bands={n_bands_straight} band_sep_cells={band_centers} "
          f"gauge_ok={gauge_ok} turn_min_cluster_sep={min_sep:.1f}cells "
          f"(half_gauge={0.5 * gauge_cells:.1f})")


def _count_bands(hit: np.ndarray) -> tuple[int, list[float]]:
    """Count contiguous True runs in a 1-D mask; return (n_runs, run-center indices)."""
    n = 0
    centers: list[float] = []
    i = 0
    L = len(hit)
    while i < L:
        if hit[i]:
            j = i
            while j < L and hit[j]:
                j += 1
            centers.append(0.5 * (i + j - 1))
            n += 1
            i = j
        else:
            i += 1
    return n, centers


# §6.5 mass under 4-wheel pass ---------------------------------------------------------

def test_four_wheel_pass_preserves_mass() -> None:
    """§6.5: total mass unchanged under a 4-wheel pass (density-only compaction)."""
    cs = procgen.rolling_hills(64, 64, 0.02, seed=4, amplitude_m=0.08)
    m0 = cs.total_mass()
    h0 = cs.derive_height().copy()
    poses = [((r, r), np.deg2rad(45.0)) for r in np.linspace(12, 52, 30)]
    four_wheel_pass(cs, poses, wheel_width_m=0.18, compaction=0.15)
    m1 = cs.total_mass()
    h1 = cs.derive_height()
    sank = bool(np.any(h1 < h0 - 1e-6))  # ruts must sink somewhere
    ok_h, err_h = _height_consistent(cs)
    check("§6.5: 4-wheel pass preserves mass (density-only compaction)",
          abs(m1 - m0) / m0 < 1e-9 and sank and ok_h,
          f"m0={m0:.6f} m1={m1:.6f} kg rel_drift={abs(m1 - m0) / m0:.2e} "
          f"ruts_sank={sank} height_err={err_h:.2e}")


def test_conform_pose_flat_ramp_clast() -> None:
    """Kinematic conform: flat -> upright; planar ramp -> pitch==atan(slope); clast ride-over tilts.

    GEOMETRY-ACCURATE check (no forces): the rover sits on the plane through its 4 wheel
    contacts, and a half-buried boulder under one wheel lifts that contact (README §4 fix).
    """
    cell_m = 0.05
    H = W = 64
    center = (32.0, 32.0)

    # (a) FLAT -> perfectly upright, zero tilt.
    flat = np.zeros((H, W), dtype=np.float64)
    pf = conform_pose(flat, center, 0.0, cell_m=cell_m)
    flat_ok = (abs(pf["pitch_rad"]) < 1e-9 and abs(pf["roll_rad"]) < 1e-9
               and abs(pf["up"][0]) < 1e-9 and abs(pf["up"][2]) < 1e-9
               and abs(pf["up"][1] - 1.0) < 1e-9)

    # (b) planar RAMP rising in +x (world). heading 0 -> forward=+col/+X, so the fore/aft
    #     wheels straddle the slope -> pitch == atan(slope), roll == 0, up tilts toward -x.
    slope = 0.1
    ramp = np.tile(np.arange(W) * cell_m * slope, (H, 1)).astype(np.float64)  # height = slope*x
    pr = conform_pose(ramp, center, 0.0, cell_m=cell_m)
    ramp_ok = (abs(pr["pitch_rad"] - float(np.arctan(slope))) < 1e-6
               and abs(pr["roll_rad"]) < 1e-9
               and pr["up"][0] < 0.0 and abs(pr["up"][2]) < 1e-9)

    # (c) CLAST ride-over: a boulder centred under the LF wheel lifts that contact -> tilt.
    cpts = wheel_contact_points(center, 0.0, cell_m=cell_m)
    lf_r, lf_c = cpts["LF"]
    clast = {"center_m": [lf_c * cell_m, 0.0, lf_r * cell_m], "radius_m": 0.30}
    pc = conform_pose(flat, center, 0.0, cell_m=cell_m, clasts=[clast])
    tilt_flat = abs(pf["pitch_rad"]) + abs(pf["roll_rad"])
    tilt_clast = abs(pc["pitch_rad"]) + abs(pc["roll_rad"])
    clast_ok = tilt_clast > tilt_flat + 1e-3

    check("conform_pose: flat upright / ramp pitch=atan(slope) / clast ride-over tilts",
          flat_ok and ramp_ok and clast_ok,
          f"flat={flat_ok} ramp_pitch={np.degrees(pr['pitch_rad']):.3f}deg"
          f"(exp {np.degrees(np.arctan(slope)):.3f}) clast_tilt "
          f"{np.degrees(tilt_flat):.2f}->{np.degrees(tilt_clast):.2f}deg")


def main() -> int:
    test_cut_dump_relax_conserves_mass()
    test_height_consistency_all_ops()
    test_rover_pass_preserves_mass()
    test_sandpile_conserves_and_reposes()
    test_io_roundtrip()
    test_quadtree_space_management()
    # render_fidelity_spec.md §6 acceptance tests (refinement.py + rover.py producer ops).
    test_refine_coarsen_roundtrip()          # §6.1
    test_base_tile_consistency()             # §6.2
    test_coarsen_zero_mass()                 # §6.2b
    test_coarsen_nonuniform_datum()          # §6.2c
    test_non_integer_k_rejected()            # §6.2d
    test_refinement_toggle_equivalence()     # §6.3
    test_four_wheel_separability()           # §6.4
    test_four_wheel_pass_preserves_mass()    # §6.5
    test_conform_pose_flat_ramp_clast()      # kinematic terrain conform (rover-physics pass)

    n_fail = sum(1 for _, ok, _ in _results if not ok)
    n_pass = len(_results) - n_fail
    print(f"\n{n_pass}/{len(_results)} checks passed.")
    if n_fail:
        print(f"{n_fail} FAILED.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
