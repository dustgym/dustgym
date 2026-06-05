"""Characterization tests for terrain_authority.sandpile.Sandpile (spec §7).

The sandpile cellular automaton relaxes loose slopes toward the angle of repose by
toppling EXCESS MASS downhill. Two invariants are load-bearing (spec §7, §10):

  1. MASS CONSERVATION — relaxation only MOVES mass between grid cells, never creates
     or destroys it (``ColumnState.total_mass()`` is invariant across any number of
     sweeps; tested to bit-level round-off).
  2. SLOPE REDUCTION TOWARD REPOSE — each sweep is non-increasing in the maximum loose
     downhill slope, and a sufficiently mild over-steepened field relaxes to rest
     (every loose slope within eps of the repose angle) in a finite number of steps.

Real data: where a real on-disk scene is the natural input we load it via the same
loader the production WorkSite uses (``worksite.coarse_base_from_bundle`` over
``samples/<name>/``, which reconstructs ``datum = heightmap - mass_areal/density``).
Where a controlled steep pile is needed, it is built with the module's OWN real
construction primitives (``Sandpile.deposit`` / ``ColumnState.set_height_via_mass``)
operating on real-shaped fields — relaxation of a real-shaped over-steepened field is
the showpiece mechanic itself (the cave-in driver in scenes.py). No values are
fabricated; every height/mass comes from the real loader or the module's own physics.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

from . import constants as K
from .column_state import ColumnState, StateLabel, loose_mask
from .sandpile import Sandpile
from .worksite import coarse_base_from_bundle

# Real committed scenes (256x256 @ 2 cm) used as natural inputs.
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_REAL_SCENES = ["crater", "crater_boulders", "rolling_hills", "flat_compact"]

# relax_to_rest stops when every loose slope is within this eps of (theta_r + cohesion).
_REST_THRESH = K.THETA_R + np.deg2rad(0.5)


def _scene_path(name: str) -> str:
    return os.path.join(_REPO, "samples", name)


def _load_real(name: str) -> ColumnState:
    """Load a real committed scene into a ColumnState via the production loader."""
    cs, _meta = coarse_base_from_bundle(_scene_path(name))
    return cs


# ---------------------------------------------------------------------------
# Construction / API smoke
# ---------------------------------------------------------------------------

def test_loads_real_scene_into_columnstate():
    """The real crater scene loads to a 256x256 @ 2 cm grid with finite positive mass."""
    cs = _load_real("crater")
    assert (cs.height, cs.width) == (256, 256)
    assert cs.cell_m == pytest.approx(0.02)
    assert cs.total_mass() > 0.0
    assert np.all(np.isfinite(cs.derive_height()))
    # The whole pristine surface is loose (eligible for relaxation) — spec §7.
    assert loose_mask(cs).all()


def test_loose_mask_compacted_cells_hold_even_when_low_density():
    # MAJOR (architecture review): a fresh single rut (TREAD) or berm (COMPACTED_BERM) holds its slope
    # regardless of density; SINTERED (dense) holds; only UNPAVED sub-mid-density spoil relaxes. The OR-
    # logic wrongly floated low-density TREAD (a fresh rut) -- and even a dense SINTERED cell -- into "loose".
    cs = ColumnState(width=1, height=6, cell_m=0.5)
    mid = 0.5 * (K.RHO_SURFACE + K.RHO_DEEP)
    cs.density[:] = mid - 100.0                              # everything below mid-density (soft)
    cs.state_label[0, 0] = StateLabel.VIRGIN
    cs.state_label[1, 0] = StateLabel.EXCAVATED
    cs.state_label[2, 0] = StateLabel.SPOIL
    cs.state_label[3, 0] = StateLabel.TREAD                  # fresh rut, still low density
    cs.state_label[4, 0] = StateLabel.COMPACTED_BERM
    cs.density[5, 0] = K.RHO_SINTERED                        # dense, fused
    cs.state_label[5, 0] = StateLabel.SINTERED
    m = loose_mask(cs)
    assert m[0, 0] and m[1, 0] and m[2, 0]                   # VIRGIN / EXCAVATED / SPOIL relax
    assert not m[3, 0] and not m[4, 0]                       # TREAD / COMPACTED_BERM hold their slope
    assert not m[5, 0]                                       # SINTERED (dense) holds


def test_runs_default_dirs_for_connectivity():
    """connectivity=8 uses 8 offsets, =4 uses 4 (Moore vs von Neumann); runs match."""
    cs = _load_real("flat_compact")
    sp8 = Sandpile(cs, connectivity=8)
    sp4 = Sandpile(cs, connectivity=4)
    assert len(sp8.neighbors) == 8
    assert len(sp4.neighbors) == 4
    # Per-neighbor horizontal runs: orthogonal == cell_m, diagonal == sqrt(2)*cell_m.
    assert min(sp8._runs) == pytest.approx(cs.cell_m)
    assert max(sp8._runs) == pytest.approx(np.sqrt(2.0) * cs.cell_m)


# ---------------------------------------------------------------------------
# deposit() perturbation — real construction primitive
# ---------------------------------------------------------------------------

def test_deposit_adds_exactly_the_requested_mass():
    """deposit() raises grid mass by exactly mass_kg and marks the cells SPOIL."""
    cs = ColumnState(width=40, height=40, cell_m=0.02)
    sp = Sandpile(cs)
    m0 = cs.total_mass()
    sp.deposit(20, 20, mass_kg=120.0, radius_cells=3)
    assert cs.total_mass() - m0 == pytest.approx(120.0, rel=1e-12)
    # The deposited disc is labelled SPOIL (loose spoil material, spec §7 step 1).
    assert cs.state_label[20, 20] == StateLabel.SPOIL


def test_deposit_single_cell_radius_zero():
    """radius_cells=0 deposits all the mass into the single target cell (the point hook)."""
    cs = ColumnState(width=20, height=20, cell_m=0.02)
    sp = Sandpile(cs)
    m0 = cs.total_mass()
    before_cell = cs.mass_areal[10, 10]
    sp.deposit(10, 10, mass_kg=12.0, radius_cells=0)
    # Exactly one cell gained mass; total rose by exactly mass_kg.
    assert cs.total_mass() - m0 == pytest.approx(12.0, rel=1e-12)
    assert cs.mass_areal[10, 10] > before_cell
    assert cs.state_label[10, 10] == StateLabel.SPOIL


def test_deposit_oversteepens_above_repose():
    """A narrow deposit creates a near-vertical step well above the repose angle."""
    cs = ColumnState(width=40, height=40, cell_m=0.02)
    sp = Sandpile(cs, connectivity=8)
    before = sp._max_loose_slope()
    sp.deposit(20, 20, mass_kg=120.0, radius_cells=2)
    after = sp._max_loose_slope()
    assert after > before
    assert after > K.THETA_R  # genuinely over-steepened (above repose)


# ---------------------------------------------------------------------------
# INVARIANT 1: mass conservation
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("connectivity", [4, 8])
def test_relax_conserves_mass_on_deposited_pile(connectivity):
    """Relaxing an over-steepened pile MOVES mass only — total is invariant to round-off."""
    cs = ColumnState(width=40, height=40, cell_m=0.02)
    sp = Sandpile(cs, connectivity=connectivity, transfer_fraction=0.5)
    sp.deposit(20, 20, mass_kg=150.0, radius_cells=4)
    m_after_deposit = cs.total_mass()
    sp.relax_to_rest(max_steps=500)
    # Pure transfer between cells -> bit-level conservation (no renormalization needed).
    assert cs.total_mass() == pytest.approx(m_after_deposit, abs=1e-9)


def test_relax_conserves_mass_on_real_crater_scene():
    """On the real crater scene (steep wall already above repose), mass is conserved
    across many relaxation sweeps even though the wall never fully reaches repose."""
    cs = _load_real("crater")
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.6)
    m0 = cs.total_mass()
    sp.relax_to_rest(max_steps=200)
    assert cs.total_mass() == pytest.approx(m0, abs=1e-9)


def test_single_step_conserves_mass():
    """One relax_step over a real over-repose field conserves mass exactly."""
    cs = _load_real("rolling_hills")
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.5)
    m0 = cs.total_mass()
    moved = sp.relax_step()
    assert isinstance(moved, (bool, np.bool_))
    assert cs.total_mass() == pytest.approx(m0, abs=1e-9)


def test_relax_never_makes_mass_negative():
    """The per-cell outflow cap guarantees no column is over-drained (mass_areal >= 0)."""
    cs = ColumnState(width=30, height=30, cell_m=0.02)
    sp = Sandpile(cs, transfer_fraction=0.5)
    sp.deposit(15, 15, mass_kg=200.0, radius_cells=2)
    sp.relax_to_rest(max_steps=500)
    assert cs.mass_areal.min() >= 0.0


# ---------------------------------------------------------------------------
# INVARIANT 2: slope reduction toward repose / termination
# ---------------------------------------------------------------------------

def test_max_slope_is_non_increasing_per_step():
    """Each stabilizing sweep is non-increasing in the maximum loose downhill slope."""
    cs = ColumnState(width=24, height=24, cell_m=0.02)
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.5)
    sp.deposit(12, 12, mass_kg=80.0, radius_cells=2)
    slopes = [sp._max_loose_slope()]
    for _ in range(12):
        sp.relax_step()
        slopes.append(sp._max_loose_slope())
    # Monotone non-increasing (allow float round-off).
    for prev, cur in zip(slopes, slopes[1:]):
        assert cur <= prev + 1e-12
    # And it really did decrease overall (the pile slumped).
    assert slopes[-1] < slopes[0]


def test_relax_reduces_max_slope_toward_repose():
    """Relaxing a deposited spike drives the max loose slope from near-vertical down to
    rest (within eps of the repose angle), conserving mass.

    The near-vertical disc edge converges to the repose plane only ASYMPTOTICALLY (the
    conservative half-the-mean-excess outflow budget), so reaching rest needs more than a
    few hundred sweeps; ~810 steps suffice here (measured), well under the 1500 cap.
    """
    cs = ColumnState(width=40, height=40, cell_m=0.02)
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.6)
    sp.deposit(20, 20, mass_kg=150.0, radius_cells=5)
    before = sp._max_loose_slope()
    m_after_deposit = cs.total_mass()
    steps, _ = sp.relax_to_rest(max_steps=1500)
    after = sp._max_loose_slope()
    assert before > np.deg2rad(80.0)            # started near vertical
    assert after < before                        # slumped massively
    assert steps < 1500                          # reached rest before the cap
    assert after <= _REST_THRESH                 # every loose slope within eps of repose
    assert cs.total_mass() == pytest.approx(m_after_deposit, abs=1e-9)


def test_uniform_overrepose_ramp_relaxes_to_rest():
    """A ramp authored just above repose (42deg) relaxes to rest: every loose slope
    falls within eps of the repose angle in a finite number of steps, mass conserved.

    The ramp height field is built with the module's own ColumnState.set_height_via_mass
    (real construction) on a real-shaped grid; relaxation operates on real-shaped fields.
    """
    H = W = 24
    cs = ColumnState(width=W, height=H, cell_m=0.02)
    dz = np.tan(np.deg2rad(42.0)) * cs.cell_m
    rows = np.arange(H).reshape(-1, 1)
    target = (H - 1 - rows) * dz + 0.1            # descending ramp, positive thickness
    cs.set_height_via_mass(np.broadcast_to(target, (H, W)).copy())
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.5)
    m0 = cs.total_mass()
    assert sp._max_loose_slope() > K.THETA_R       # genuinely over-repose to start
    steps, _ = sp.relax_to_rest(max_steps=800)
    assert steps < 800                             # terminated before the cap (reached rest)
    assert sp._max_loose_slope() <= _REST_THRESH   # every loose slope within eps of repose
    assert cs.total_mass() == pytest.approx(m0, abs=1e-9)


def test_single_overrepose_cell_relaxes_to_rest_quickly():
    """A single 60deg over-repose cell amid a flat field relaxes to rest in a handful of
    sweeps with mass conserved (the minimal avalanche)."""
    H = W = 6
    cs = ColumnState(width=W, height=H, cell_m=0.02)
    t = np.full((H, W), 0.1)
    t[2, 2] = 0.1 + np.tan(np.deg2rad(60.0)) * cs.cell_m
    cs.set_height_via_mass(t)
    sp = Sandpile(cs, connectivity=4, transfer_fraction=0.5)
    m0 = cs.total_mass()
    steps, _ = sp.relax_to_rest(max_steps=2000)
    assert steps < 2000
    assert sp._max_loose_slope() <= _REST_THRESH
    assert cs.total_mass() == pytest.approx(m0, abs=1e-12)


def test_relax_step_returns_false_on_flat_field():
    """A flat default field is already at rest: relax_step moves nothing (returns False)."""
    cs = ColumnState(width=16, height=16, cell_m=0.02)
    sp = Sandpile(cs)
    assert sp._max_loose_slope() == pytest.approx(0.0)
    assert sp.relax_step() is False


def test_relax_to_rest_terminates_on_the_slope_criterion():
    """relax_to_rest stops on the physically-meaningful slope criterion, NOT bit-exact
    mechanical rest: a 50deg over-repose cell reaches rest in a handful of sweeps and
    stops there, even though relax_step keeps moving sub-epsilon mass asymptotically.

    This documents the real stop condition (sandpile.py:228-232): the conservative
    outflow budget converges to the repose plane asymptotically, so the loop terminates
    when every loose slope is within eps of repose, well before relax_step would ever
    return False.
    """
    H = W = 8
    cs = ColumnState(width=W, height=H, cell_m=0.02)
    t = np.full((H, W), 0.1)
    t[4, 4] = 0.1 + np.tan(np.deg2rad(50.0)) * cs.cell_m
    cs.set_height_via_mass(t)
    sp = Sandpile(cs, connectivity=4, transfer_fraction=0.5)
    m0 = cs.total_mass()
    steps, _ = sp.relax_to_rest(max_steps=2000)
    assert steps < 2000                          # terminated (did not hit the cap)
    assert sp._max_loose_slope() <= _REST_THRESH  # ... on the slope criterion
    assert cs.total_mass() == pytest.approx(m0, abs=1e-12)
    # A further sweep still moves sub-epsilon mass (rest is the slope criterion, not
    # bit-exact mechanical quiescence) — but mass stays conserved.
    sp.relax_step()
    assert cs.total_mass() == pytest.approx(m0, abs=1e-12)


# ---------------------------------------------------------------------------
# Cohesion / metastability knob
# ---------------------------------------------------------------------------

def test_cohesion_raises_the_rest_threshold():
    """A cohesion_steepening term lets the pile hold a steeper slope before failing:
    relaxing with cohesion leaves a steeper resting slope than without (spec §7)."""
    H = W = 24
    dz = np.tan(np.deg2rad(42.0)) * 0.02
    rows = np.arange(H).reshape(-1, 1)
    target = np.broadcast_to((H - 1 - rows) * dz + 0.1, (H, W)).copy()

    plain = ColumnState(width=W, height=H, cell_m=0.02)
    plain.set_height_via_mass(target)
    sp_plain = Sandpile(plain, connectivity=8, transfer_fraction=0.5)
    sp_plain.relax_to_rest(max_steps=800)

    cohesive = ColumnState(width=W, height=H, cell_m=0.02)
    cohesive.set_height_via_mass(target)
    sp_cohesive = Sandpile(cohesive, connectivity=8, transfer_fraction=0.5,
                           cohesion_steepening=np.deg2rad(5.0))
    sp_cohesive.relax_to_rest(max_steps=800)

    # The cohesive pile is allowed to (and does) hold a steeper resting slope.
    assert sp_cohesive._max_loose_slope() > sp_plain._max_loose_slope()


# ---------------------------------------------------------------------------
# Snapshot capture (the cave-in time series)
# ---------------------------------------------------------------------------

def test_capture_returns_height_snapshots():
    """relax_to_rest(capture=True) returns a time series of derived heightmaps, each a
    copy of the live derived surface, shaped like the grid (the cave-in series)."""
    cs = ColumnState(width=20, height=20, cell_m=0.02)
    sp = Sandpile(cs, transfer_fraction=0.5)
    sp.deposit(10, 10, mass_kg=60.0, radius_cells=2)
    steps, snaps = sp.relax_to_rest(max_steps=500, capture=True, capture_every=4)
    assert len(snaps) >= 2
    assert all(s.shape == (20, 20) for s in snaps)
    # First snapshot is the pre-relax surface; it differs from the rested surface.
    assert not np.array_equal(snaps[0], snaps[-1])


def test_capture_appends_final_rested_surface():
    """With capture_every > 1 the last in-loop snapshot may pre-date the final step, so
    relax_to_rest appends the rested surface as the terminal frame: the last snapshot is
    exactly the final derived heightmap (the cave-in series ends at rest, sandpile.py:233)."""
    cs = ColumnState(width=20, height=20, cell_m=0.02)
    sp = Sandpile(cs, transfer_fraction=0.5)
    sp.deposit(10, 10, mass_kg=60.0, radius_cells=2)
    steps, snaps = sp.relax_to_rest(max_steps=500, capture=True, capture_every=7)
    assert steps > 1
    # The terminal frame of the series is exactly the final rested surface.
    assert np.array_equal(snaps[-1], cs.derive_height())


def test_capture_disabled_returns_empty_snapshots():
    """Without capture, relax_to_rest returns no snapshots but still steps."""
    cs = ColumnState(width=20, height=20, cell_m=0.02)
    sp = Sandpile(cs, transfer_fraction=0.5)
    sp.deposit(10, 10, mass_kg=60.0, radius_cells=2)
    steps, snaps = sp.relax_to_rest(max_steps=50)
    assert snaps == []
    assert steps >= 1


# ---------------------------------------------------------------------------
# All real scenes: conservation under relaxation (cross-scene characterization)
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("name", _REAL_SCENES)
def test_all_real_scenes_conserve_mass_under_relaxation(name):
    """Relaxation conserves mass on every committed real scene, and never increases the
    maximum loose slope (it can only relax toward repose)."""
    cs = _load_real(name)
    sp = Sandpile(cs, connectivity=8, transfer_fraction=0.5)
    m0 = cs.total_mass()
    s0 = sp._max_loose_slope()
    sp.relax_to_rest(max_steps=100)
    assert cs.total_mass() == pytest.approx(m0, abs=1e-9)
    assert sp._max_loose_slope() <= s0 + 1e-12
