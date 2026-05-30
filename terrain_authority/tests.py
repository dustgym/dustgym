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

import sys

import numpy as np

from . import constants as K
from . import procgen
from .column_state import ColumnState, StateLabel, loose_mask
from .rover import straight_path, wheel_pass
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
    import json
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


def main() -> int:
    test_cut_dump_relax_conserves_mass()
    test_height_consistency_all_ops()
    test_rover_pass_preserves_mass()
    test_sandpile_conserves_and_reposes()
    test_io_roundtrip()

    n_fail = sum(1 for _, ok, _ in _results if not ok)
    n_pass = len(_results) - n_fail
    print(f"\n{n_pass}/{len(_results)} checks passed.")
    if n_fail:
        print(f"{n_fail} FAILED.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
