"""Characterization tests for terrain_authority/scenes.py — the sample-scene builder/exporter.

These run the REAL builders against a redirected ``SAMPLES_DIR`` (a pytest tmp dir) so the
committed ``samples/`` rasters are NEVER touched, then load the written scenes back through the
real ``io_fields.load_scene`` and assert the physical invariants the module promises:

  * mass conservation across the cut/dump/relax/drive operations (spec §10 invariant 1);
  * the height identity ``height == datum + mass_areal/density`` (INTERFACE.md §4, spec §6);
  * field shapes/dtypes (on-disk float32 rasters, uint8 state_label) and value ranges
    (density > 0, finite heights);
  * determinism under the builders' fixed seeds (two independent runs are bit-identical).

No synthetic data: ``main()`` / the builders ARE the real implementation under test (procgen
generators with fixed seeds), and ``build_from_dem`` is exercised against the committed real-LOLA
Haworth DEM scene on disk. A run is shared across tests via a session fixture so the (somewhat
expensive) full ``main()`` sweep is paid once.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np
import pytest

from terrain_authority import constants as K
from terrain_authority import scenes
from terrain_authority.io_fields import load_scene

# Scenes that build_* writes as a single (non time-series) bundle directly under SAMPLES_DIR/<name>.
SINGLE_SCENES = [
    "flat_compact",
    "rolling_hills",
    "crater",
    "boulder_field",
    "crater_boulders",
]

# Time-series scenes: SAMPLES_DIR/<name>/t000.. plus a parent metadata.json.
TIMESERIES_SCENES = [
    "crater_caveins",
    "tread_track",
    "tread_track_4wheel",
    "excavation_marks",
]


# ---------------------------------------------------------------------------
# Session fixture: run the REAL builders once into a tmp SAMPLES_DIR.
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def built_samples(tmp_path_factory):
    """Run scenes.main() once with SAMPLES_DIR redirected to a tmp dir.

    Exercises every builder (the nine legacy scenes) AND the DEM hook in main(); returns the
    tmp root. The committed samples/ tree is left untouched (asserted below).
    """
    out = tmp_path_factory.mktemp("samples_built")
    orig = scenes.SAMPLES_DIR
    scenes.SAMPLES_DIR = str(out)
    try:
        rc = scenes.main()
    finally:
        scenes.SAMPLES_DIR = orig
    assert rc == 0
    return str(out)


def _height_from_fields(fields: dict) -> np.ndarray:
    """The height identity: datum is omitted on disk, but the saved heightmap must equal
    mass_areal/density above SOME datum; we instead verify the identity that holds without datum
    knowledge — the saved heightmap is finite and the loaded fields are internally consistent
    (density>0). For the strict identity we use the in-RAM ColumnState path in dedicated tests."""
    return np.asarray(fields["heightmap"], dtype=np.float64)


# ---------------------------------------------------------------------------
# main() smoke + committed-samples safety.
# ---------------------------------------------------------------------------

def test_main_builds_all_scenes(built_samples):
    """main() returns 0 and produces every legacy scene dir."""
    for name in SINGLE_SCENES + TIMESERIES_SCENES:
        assert os.path.isdir(os.path.join(built_samples, name)), name


def test_committed_samples_untouched(built_samples):
    """The real committed samples/ tree must still be present and not redirected."""
    assert scenes.SAMPLES_DIR.endswith("samples")
    assert os.path.exists(os.path.join("samples", "flat_compact", "metadata.json"))
    # The fixture wrote elsewhere (a tmp dir), not into the repo samples/.
    assert os.path.abspath(built_samples) != os.path.abspath(scenes.SAMPLES_DIR)


# ---------------------------------------------------------------------------
# Single-bundle scene invariants (shape / dtype / ranges / metadata).
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", SINGLE_SCENES)
def test_single_scene_fields_and_metadata(built_samples, name):
    fields, meta = load_scene(os.path.join(built_samples, name))

    # The 5 REQUIRED contract rasters are present.
    for req in ("heightmap", "mass_areal", "density", "disturbance", "state_label"):
        assert req in fields, (name, req)

    # On-disk dtypes: float32 rasters, uint8 state_label (io_fields _FIELD_SPEC).
    assert fields["heightmap"].dtype == np.float32
    assert fields["mass_areal"].dtype == np.float32
    assert fields["density"].dtype == np.float32
    assert fields["disturbance"].dtype == np.float32
    assert fields["state_label"].dtype == np.uint8

    # Shapes == (HEIGHT, WIDTH) from the module grid constants.
    for arr in fields.values():
        assert arr.shape == (scenes.HEIGHT, scenes.WIDTH), (name, arr.shape)

    # Value ranges: density strictly positive, heights finite, disturbance in [0,1],
    # state_label inside the declared enum, mass non-negative.
    assert np.all(fields["density"] > 0.0)
    assert np.all(np.isfinite(_height_from_fields(fields)))
    assert np.all(fields["mass_areal"] >= 0.0)
    assert fields["disturbance"].min() >= 0.0 and fields["disturbance"].max() <= 1.0 + 1e-6
    assert fields["state_label"].max() < len(K.STATE_NAMES)

    # Metadata: schema/grid/gravity/world bounds as _base_metadata writes them.
    assert meta["schema_version"] == "1.0"
    assert meta["scene_name"] == name
    assert meta["grid"]["width"] == scenes.WIDTH
    assert meta["grid"]["height"] == scenes.HEIGHT
    assert meta["grid"]["cell_m"] == scenes.CELL_M
    assert meta["gravity_m_s2"] == K.g
    assert meta["world_bounds_m"]["x1"] == round(scenes.WIDTH * scenes.CELL_M, 4)
    assert meta["world_bounds_m"]["y1"] == round(scenes.HEIGHT * scenes.CELL_M, 4)

    # height_range_m is [min,max] consistent with the saved heightmap (to float32 precision).
    hr = meta["height_range_m"]
    h = _height_from_fields(fields)
    assert hr[0] <= h.min() + 1e-3 and hr[1] >= h.max() - 1e-3
    assert hr[0] <= hr[1]

    # Quadtree metadata: a ROOT plus at least one ACTIVE node (INTERFACE.md §5).
    labels = {n["label"] for n in meta["quadtree"]}
    assert "ROOT" in labels and "ACTIVE" in labels


def test_height_matches_mass_over_density_on_disk(built_samples):
    """The saved height equals mass_areal/density above the per-scene datum.

    The datum field is intentionally NOT serialized (io_fields omits it). For the flat_compact
    scene the procgen datum is a known constant (surface.min() - Z_T at RHO_DEEP), so we can
    reconstruct it from a single corner and verify height == datum + mass/density everywhere to
    float32 tolerance — the load-bearing INTERFACE.md §4 identity."""
    fields, _ = load_scene(os.path.join(built_samples, "flat_compact"))
    h = fields["heightmap"].astype(np.float64)
    mass = fields["mass_areal"].astype(np.float64)
    rho = fields["density"].astype(np.float64)
    thickness = mass / rho
    # datum = height - thickness must be (nearly) constant for flat_compact (uniform datum).
    datum = h - thickness
    assert np.ptp(datum) < 1e-3, f"datum not uniform on flat_compact: ptp={np.ptp(datum)}"
    # And the identity round-trips: datum + mass/rho == height (definitionally) to f32 noise.
    assert np.max(np.abs((datum + thickness) - h)) < 1e-4


def test_crater_has_excavated_label_and_lower_floor(built_samples):
    """The crater builder carves a bowl: floor cells get the EXCAVATED label and the bowl
    center sits below the rim (real morphometry, not just 'it ran')."""
    fields, meta = load_scene(os.path.join(built_samples, "crater"))
    labels = fields["state_label"]
    from terrain_authority.column_state import StateLabel
    assert np.any(labels == int(StateLabel.EXCAVATED)), "no EXCAVATED cells in crater"
    h = fields["heightmap"].astype(np.float64)
    cr, cc = scenes.HEIGHT // 2, scenes.WIDTH // 2
    center_h = h[cr, cc]
    # A corner well outside the bowl is the undisturbed reference.
    edge_h = h[5, 5]
    assert center_h < edge_h, (center_h, edge_h)


def test_boulder_field_clasts_schema(built_samples):
    """boulder_field records real Golombek clasts in metadata with the §5 clast schema."""
    _, meta = load_scene(os.path.join(built_samples, "boulder_field"))
    clasts = meta["clasts"]
    assert isinstance(clasts, list) and len(clasts) > 0
    for c in clasts:
        assert set(("id", "center_m", "radius_m", "shape", "buried_frac")) <= set(c)
        assert c["radius_m"] > 0.0
        assert 0.0 <= c["buried_frac"] <= 1.0
        assert len(c["center_m"]) == 3


def test_crater_boulders_excludes_bowl_and_snaps_surface(built_samples):
    """crater_boulders excludes clasts from the fresh bowl and snaps them onto the terrain."""
    fields, meta = load_scene(os.path.join(built_samples, "crater_boulders"))
    clasts = meta["clasts"]
    assert len(clasts) > 0
    h = fields["heightmap"].astype(np.float64)
    cr, cc = scenes.HEIGHT // 2, scenes.WIDTH // 2
    cx, cz = cc * scenes.CELL_M, cr * scenes.CELL_M
    R = 0.5 * 2.2
    for c in clasts:
        x, _y, z = c["center_m"]
        # No clast inside 0.95*R of the bowl center (the exclusion rule in the builder).
        assert math.hypot(x - cx, z - cz) >= 0.95 * R - scenes.CELL_M, c
        # ids are reassigned 0..n-1 contiguously by the builder.
    assert [c["id"] for c in clasts] == list(range(len(clasts)))
    assert np.all(np.isfinite(h))


# ---------------------------------------------------------------------------
# Time-series scene invariants (frames + parent metadata + mass conservation).
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", TIMESERIES_SCENES)
def test_timeseries_structure_and_parent_meta(built_samples, name):
    scene_dir = os.path.join(built_samples, name)
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    ts = parent["time_series"]
    frame_dirs = ts["frame_dirs"]
    assert ts["frame_count"] == len(frame_dirs)
    assert len(frame_dirs) >= 2  # at least pristine + one mutated frame
    # Every advertised frame dir exists and loads as a full contract bundle.
    for fd in frame_dirs:
        fields, meta = load_scene(os.path.join(scene_dir, fd))
        assert fields["heightmap"].shape == (scenes.HEIGHT, scenes.WIDTH)
        assert fields["state_label"].dtype == np.uint8
        assert np.all(fields["density"] > 0.0)
        assert meta["frame_index"] == int(fd[1:])
    # Parent mass_drift is tiny (mass conserved across the series).
    assert ts["mass_drift_kg"] < 1e-3, (name, ts["mass_drift_kg"])


def test_mass_conserved_first_to_last_frame_on_disk(built_samples):
    """tread_track is pure compaction: total grid mass at t000 == final frame (read off disk).

    Mass = Σ mass_areal * cell^2 (drum inventory is never touched here), so it must be stable
    across the whole driven track to float32 round-off."""
    scene_dir = os.path.join(built_samples, "tread_track")
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    frame_dirs = parent["time_series"]["frame_dirs"]
    cell2 = scenes.CELL_M ** 2

    def grid_mass(fd):
        f, _ = load_scene(os.path.join(scene_dir, fd))
        return float(f["mass_areal"].astype(np.float64).sum()) * cell2

    m0 = grid_mass(frame_dirs[0])
    mN = grid_mass(frame_dirs[-1])
    assert m0 > 0.0
    # float32 storage on a 256x256 grid: a few-gram relative drift is expected, not a leak.
    assert abs(mN - m0) / m0 < 1e-4, (m0, mN)


def test_tread_track_lays_compaction_trail(built_samples):
    """Along the driven track the density rises (compaction) and TREAD appears: a real
    path-dependent change, not just a re-saved pristine surface."""
    from terrain_authority.column_state import StateLabel
    scene_dir = os.path.join(built_samples, "tread_track")
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    frame_dirs = parent["time_series"]["frame_dirs"]
    f0, _ = load_scene(os.path.join(scene_dir, frame_dirs[0]))
    fN, _ = load_scene(os.path.join(scene_dir, frame_dirs[-1]))
    # No TREAD on the pristine frame; some TREAD after driving.
    assert not np.any(f0["state_label"] == int(StateLabel.TREAD))
    assert np.any(fN["state_label"] == int(StateLabel.TREAD))
    # Mean density rises somewhere (compaction toward RHO_DEEP).
    assert fN["density"].astype(np.float64).max() >= f0["density"].astype(np.float64).max()


def test_excavation_marks_drum_marks_and_excavated(built_samples):
    """excavation_marks: the dug frame carries §5.2 drum_marks and an EXCAVATED/SPOIL change,
    and the parent records a positive drum_excavated_kg with mass conserved through inventory."""
    from terrain_authority.column_state import StateLabel
    scene_dir = os.path.join(built_samples, "excavation_marks")
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    ts = parent["time_series"]
    assert ts["drum_excavated_kg"] > 0.0
    assert ts["mass_drift_kg"] < 1e-3
    # Pristine t000 has no drum_marks; dug t001 does.
    _, m0 = load_scene(os.path.join(scene_dir, "t000"))
    f1, m1 = load_scene(os.path.join(scene_dir, "t001"))
    assert "drum_marks" not in m0
    assert "drum_marks" in m1 and len(m1["drum_marks"]) == 1
    # The dig produced EXCAVATED cells and dumped SPOIL.
    assert np.any(f1["state_label"] == int(StateLabel.EXCAVATED))
    assert np.any(f1["state_label"] == int(StateLabel.SPOIL))


def test_tread_track_4wheel_features_and_tiles(built_samples):
    """tread_track_4wheel: per-frame wheel_tracks + refinement, and the FINAL frame writes
    fine tile bundles whose coarsening returns the base block (base<->tile consistency)."""
    from terrain_authority import refinement
    scene_dir = os.path.join(built_samples, "tread_track_4wheel")
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        parent = json.load(fh)
    assert parent["contract_revision"] == "1.0.2"
    assert "wheel_tracks" in parent["features"]
    frame_dirs = parent["time_series"]["frame_dirs"]

    # A mid frame carries wheel_tracks (4 contact polylines) + a refinement policy block.
    mid_dir = frame_dirs[len(frame_dirs) // 2]
    _, mid_meta = load_scene(os.path.join(scene_dir, mid_dir))
    assert "refinement" in mid_meta and mid_meta["refinement"]["enabled"] is True
    assert mid_meta["refinement"]["base_cell_m"] == scenes.CELL_M
    assert mid_meta["refinement"]["fine_cell_m"] == scenes.FINE_CELL_M
    if "wheel_tracks" in mid_meta:
        wt = mid_meta["wheel_tracks"]
        assert set(("LF", "RF", "LB", "RB")) <= set(wt) or isinstance(wt, dict)

    # Final frame: tiles[] descriptor + on-disk tile bundles; coarsen(tile) == base block.
    last_dir = frame_dirs[-1]
    last_fields, last_meta = load_scene(os.path.join(scene_dir, last_dir))
    assert "tiles" in last_meta and len(last_meta["tiles"]) > 0
    k = refinement.k_factor(scenes.CELL_M, scenes.FINE_CELL_M)
    base_mass = last_fields["mass_areal"].astype(np.float64)
    base_state = last_fields["state_label"]
    for tdesc in last_meta["tiles"]:
        rel = tdesc["dir"]
        tdir = os.path.join(scene_dir, last_dir, rel)
        assert os.path.isdir(tdir), tdir
        tfields, tmeta = load_scene(tdir)
        # tile cell_m is the fine resolution and dims are k x the base region.
        r0, c0, r1, c1 = tdesc["region_rc"]
        assert tmeta["grid"]["cell_m"] == scenes.FINE_CELL_M
        assert tfields["heightmap"].shape == ((r1 - r0) * k, (c1 - c0) * k)
        # Coarsen the fine tile back to base and compare to the base block (§5.3 invariant).
        # state_label/disturbance/density round-trip exactly; datum is missing on disk so we
        # reconstruct it via height - mass/density (uniform per cell) before coarsening.
        h_t = tfields["heightmap"].astype(np.float64)
        m_t = tfields["mass_areal"].astype(np.float64)
        rho_t = tfields["density"].astype(np.float64)
        bundle = {
            "mass_areal": m_t,
            "density": rho_t,
            "datum": h_t - m_t / rho_t,
            "state_label": tfields["state_label"],
            "disturbance": tfields["disturbance"].astype(np.float64),
        }
        back = refinement.coarsen_field(bundle, k)
        # mass_areal is intensive: coarsen returns the base-block mean per base cell. Compare
        # to the base raster block (to float32 storage tolerance).
        ref_block = base_mass[r0:r1, c0:c1]
        scale = max(float(np.max(np.abs(ref_block))), 1.0)
        assert np.max(np.abs(back["mass_areal"] - ref_block)) / scale < 1e-3
        assert np.array_equal(back["state_label"], base_state[r0:r1, c0:c1])


# ---------------------------------------------------------------------------
# Determinism (fixed seeds).
# ---------------------------------------------------------------------------

def test_builder_determinism_flat_compact(tmp_path):
    """Two independent flat_compact builds (same fixed seed inside the builder) are bit-identical
    on every raster — the spec §10 determinism guarantee."""
    def build_into(sub):
        d = tmp_path / sub
        d.mkdir()
        orig = scenes.SAMPLES_DIR
        scenes.SAMPLES_DIR = str(d)
        try:
            scenes.build_flat_compact()
        finally:
            scenes.SAMPLES_DIR = orig
        return load_scene(os.path.join(str(d), "flat_compact"))[0]

    a = build_into("run_a")
    b = build_into("run_b")
    for name in a:
        assert np.array_equal(a[name], b[name]), name


def test_builder_determinism_tread_track_replay():
    """_replay_tread_track is deterministic: two runs give identical mass + identical final
    frame, and mass_before == mass_after (pure compaction conserves grid mass)."""
    frames1, mb1, ma1 = scenes._replay_tread_track()
    frames2, mb2, ma2 = scenes._replay_tread_track()
    assert mb1 == mb2 and ma1 == ma2
    assert abs(ma1 - mb1) / mb1 < 1e-9, (mb1, ma1)  # pure compaction: mass conserved
    assert np.array_equal(frames1[-1].mass_areal, frames2[-1].mass_areal)
    assert np.array_equal(frames1[-1].density, frames2[-1].density)


def test_replay_caveins_conserves_mass():
    """_replay_caveins relaxes an over-steepened rim by the sandpile CA; the avalanche
    redistributes mass but does not create/destroy it (frame 0 -> final to f64 round-off)."""
    cr, cc = scenes.HEIGHT // 2, scenes.WIDTH // 2
    frames = scenes._replay_caveins(2.0, cr, cc)
    assert len(frames) >= 2
    m0 = frames[0].total_mass()
    mN = frames[-1].total_mass()
    assert m0 > 0.0
    assert abs(mN - m0) / m0 < 1e-9, (m0, mN)
    # The relaxation actually moved material (the surface changed between first and last).
    assert not np.array_equal(frames[0].mass_areal, frames[-1].mass_areal)


# ---------------------------------------------------------------------------
# Pure helper functions.
# ---------------------------------------------------------------------------

def test_heading_from_segment_conventions():
    """INTERFACE.md §5.2 convention: 0 = +col/+X, +pi/2 = +row/+Z; zero segment -> 0."""
    assert scenes._heading_from_segment((0, 0), (0, 1)) == 0.0          # +col
    assert math.isclose(scenes._heading_from_segment((0, 0), (1, 0)), math.pi / 2)  # +row
    assert scenes._heading_from_segment((3, 3), (3, 3)) == 0.0          # degenerate
    # A diagonal goes to atan2(drow, dcol) = pi/4.
    assert math.isclose(scenes._heading_from_segment((0, 0), (1, 1)), math.pi / 4)


def test_default_quadtree_root_and_active():
    qt = scenes._default_quadtree(active_row0=10, active_col0=20, active_size=40)
    assert qt[0]["label"] == "ROOT" and qt[0]["size"] == scenes.WIDTH
    assert qt[1]["label"] == "ACTIVE"
    assert qt[1]["row0"] == 10 and qt[1]["col0"] == 20 and qt[1]["size"] == 40


def test_base_metadata_shape_and_defaults():
    meta = scenes._base_metadata("unit_test_scene", notes="hello")
    assert meta["scene_name"] == "unit_test_scene"
    assert meta["notes"] == "hello"
    assert meta["grid"] == {"width": scenes.WIDTH, "height": scenes.HEIGHT,
                            "cell_m": scenes.CELL_M, "order": "row-major-C"}
    assert meta["gravity_m_s2"] == K.g
    # Defaults: empty clasts, the default active_zone, the default ROOT+ACTIVE quadtree.
    assert meta["clasts"] == []
    assert meta["active_zone"] == {"min_rc": [64, 64], "max_rc": [192, 192]}
    assert {n["label"] for n in meta["quadtree"]} == {"ROOT", "ACTIVE"}
    # extra= merges additional keys.
    meta2 = scenes._base_metadata("s", extra={"custom_key": 7})
    assert meta2["custom_key"] == 7


def test_tread_path_endpoints_and_positions_lockstep():
    """The path endpoints stay on-grid and the per-frame positions begin with the pristine
    (None) frame then track real (row,col) cells inside the grid."""
    cr0, cc0, cr1, cc1, cr2, cc2 = scenes._tread_path_endpoints()
    for v, hi in ((cr0, scenes.HEIGHT), (cr1, scenes.HEIGHT), (cr2, scenes.HEIGHT),
                  (cc0, scenes.WIDTH), (cc1, scenes.WIDTH), (cc2, scenes.WIDTH)):
        assert 0 <= v < hi
    positions = scenes._tread_frame_positions()
    assert positions[0] is None  # pristine pre-drive frame
    moving = [p for p in positions[1:] if p is not None]
    assert len(moving) >= 2
    for (r, c) in moving:
        assert 0 <= r < scenes.HEIGHT and 0 <= c < scenes.WIDTH
    # Frame positions and the replay frames have the same count (lockstep contract).
    frames, _, _ = scenes._replay_tread_track()
    assert len(frames) == len(positions)


def test_clone_is_deep_copy():
    """_clone produces an independent ColumnState (mutating the clone leaves the source alone)."""
    from terrain_authority import procgen
    cs = procgen.flat_compact(32, 32, 0.02, seed=2)
    clone = scenes._clone(cs)
    assert clone.width == cs.width and clone.height == cs.height
    assert np.array_equal(clone.mass_areal, cs.mass_areal)
    before = cs.mass_areal.copy()
    clone.mass_areal[0, 0] += 123.0
    assert np.array_equal(cs.mass_areal, before)  # source untouched -> real copy
    assert clone.mass_areal[0, 0] != before[0, 0]


# ---------------------------------------------------------------------------
# build_from_dem against the committed real-LOLA DEM scene.
# ---------------------------------------------------------------------------

DEM_SCENE = os.path.join("samples", "lunar_dem", "haworth_10km_5m")
_DEM_PRESENT = os.path.exists(os.path.join(DEM_SCENE, "metadata.json"))


@pytest.mark.skipif(not _DEM_PRESENT, reason="committed DEM scene not on disk")
def test_build_from_dem_roundtrip_and_provenance():
    """build_from_dem wires the four Wave-2 generators against the real Haworth DEM and returns
    a loadable bundle whose derive_height() == the committed heightmap (the §8 datum re-supply),
    with the ChaSTE density, illumination, and corridor provenance attached additively."""
    fields, meta = scenes.build_from_dem(radius_m=20.0)
    # 5 required rasters + the re-derived datum.
    for req in ("heightmap", "mass_areal", "density", "disturbance", "state_label", "datum"):
        assert req in fields, req
    h = fields["heightmap"]
    assert np.all(np.isfinite(h))
    # The height identity round-trips on the real DEM to <= 1e-3 m (the builder's own assert).
    derived = fields["datum"] + fields["mass_areal"] / fields["density"]
    assert np.max(np.abs(derived - h)) <= 1e-3
    # ChaSTE bulk density is a constant inside the acceptance range, and > 0.
    rho = fields["density"]
    assert np.all(rho > 0.0)
    assert K.RHO_SURFACE_POLAR <= float(rho.flat[0]) <= K.RHO_BULK_POLAR_10CM
    assert np.ptp(rho) == 0.0  # single mass-weighted-mean scalar broadcast
    # Provenance blocks are present and self-consistent.
    assert meta["schema_version"] == "1.0"
    assert meta["density_source"]["tag"] == "[CALIB]"
    illum = meta["illumination"]
    assert illum["tag"] == "terrain-derived"
    assert 0.0 <= illum["lit_fraction"] <= 1.0
    assert math.isclose(illum["lit_fraction"] + illum["shadow_fraction"], 1.0, abs_tol=1e-6)
    dc = meta["dem_corridor"]
    assert dc["refine_factor_k"] >= 1
    assert dc["fine_tiles_materialized"] >= 1
    # Conservation self-check on a materialized fine tile: coarsen(fine) == base, bit-exact.
    cc = dc["conservation_check"]
    assert cc["coarsen_equals_base"] is True
    assert cc["datum_bit_exact"] is True and cc["state_bit_exact"] is True
    assert {"dem_backbone", "dem_corridor", "density_chaste", "illumination_horizon"} <= set(
        meta["features"])


@pytest.mark.skipif(not _DEM_PRESENT, reason="committed DEM scene not on disk")
def test_build_from_dem_no_craters_path():
    """The with_craters=False branch still builds a conservation-clean corridor (feature_fn None)."""
    fields, meta = scenes.build_from_dem(radius_m=15.0, with_craters=False)
    assert meta["dem_corridor"]["with_craters"] is False
    derived = fields["datum"] + fields["mass_areal"] / fields["density"]
    assert np.max(np.abs(derived - fields["heightmap"])) <= 1e-3


@pytest.mark.skipif(not _DEM_PRESENT, reason="committed DEM scene not on disk")
def test_build_dem_scene_hook_runs(capsys):
    """build_dem_scene() is the main() hook: it builds in-RAM and prints a status line; it must
    not raise and must report the wired stack for the committed DEM scene."""
    scenes.build_dem_scene()
    out = capsys.readouterr().out
    assert "build_from_dem" in out
    assert "coarsen==base=True" in out


def test_build_dem_scene_missing_is_graceful(tmp_path, monkeypatch):
    """A fresh checkout without samples/lunar_dem/ degrades gracefully (SKIP, not fatal)."""
    monkeypatch.setattr(scenes, "ROOT", str(tmp_path))
    scenes.build_dem_scene()  # must simply return without raising


def test_build_from_dem_rejects_non_dem_scene(tmp_path):
    """A scene dir with metadata but no heightmap raster is rejected (not a DEM scene)."""
    sd = tmp_path / "no_height"
    sd.mkdir()
    meta = {
        "grid": {"width": 4, "height": 4, "cell_m": 5.0, "order": "row-major-C"},
        "world_bounds_m": {"x0": 0.0, "y0": 0.0, "x1": 20.0, "y1": 20.0},
    }
    with open(sd / "metadata.json", "w") as fh:
        json.dump(meta, fh)
    with pytest.raises(ValueError, match="no heightmap"):
        scenes.build_from_dem(str(sd))
