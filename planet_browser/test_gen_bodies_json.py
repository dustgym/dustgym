"""gen_bodies_json.py coverage — run the generator, verify bodies.json matches the .py source.

gen_bodies_json.py exports terrain_authority/bodies.py + ipex_specs.py to planet_browser/bodies.json so
the browser loads the SAME constants the sim uses (single source of truth). The script runs at module
import (no __main__ guard), so we execute it IN-PROCESS via runpy against a tmp copy (committed
bodies.json untouched, and coverage sees the lines) and assert:

  * every terrain_authority.bodies.BODIES body (incl. all ROVER_BODIES) is present, with its real
    gravity / label / bekker block carried through unchanged from the .py source;
  * the generated _ipex energy block matches ipex_specs.py exactly (drum kg, dig J/kg, drive J/m,
    battery J, dig rate, sinter gate).

No synthetic data: the comparison is the live .py constants vs the JSON the script writes.
Run from the repo root: PYTHONPATH=. <venv>/bin/python -m pytest planet_browser/test_gen_bodies_json.py -q
"""
from __future__ import annotations

import importlib
import json
import os

import pytest

from terrain_authority import bodies as B
from terrain_authority import constants as K
from terrain_authority import ipex_specs as S
from terrain_authority.registration import ROVER_BODIES

_HERE = os.path.dirname(os.path.abspath(__file__))
_COMMITTED = os.path.join(_HERE, "bodies.json")


@pytest.fixture(scope="module")
def generated():
    """IMPORT the REAL planet_browser.gen_bodies_json module, which (it has no __main__ guard) runs
    the generator at import time, regenerating planet_browser/bodies.json from the live .py source.
    Importing (vs runpy on a copy) is what lets coverage attribute the executed lines to the actual
    module. We snapshot the committed bodies.json and restore it byte-exact afterwards so the
    checked-in artifact is untouched. Returns the freshly generated dict for assertions."""
    backup = None
    if os.path.isfile(_COMMITTED):
        with open(_COMMITTED, "rb") as f:
            backup = f.read()
    try:
        import planet_browser.gen_bodies_json as gen        # noqa: F401  (import has the side effect)
        importlib.reload(gen)                                # re-run even if a prior test imported it
        with open(_COMMITTED) as f:
            out = json.load(f)
    finally:
        if backup is not None:
            with open(_COMMITTED, "wb") as f:
                f.write(backup)                              # restore the committed artifact byte-exact
    return out


def test_every_bodies_entry_present_with_real_constants(generated):
    out = generated
    for key, b in B.BODIES.items():
        assert key in out, f"{key} missing from bodies.json"
        d = out[key]
        assert d["name"] == b.name and d["label"] == b.label
        assert d["g"] == b.g                                 # real surface gravity carried through
        assert d["bekker_regime"] == b.bekker_regime


def test_all_rover_bodies_present(generated):
    out = generated
    for body in ROVER_BODIES:                                # moon/mars/ceres/earth -> the drive IDs
        assert body in out


def test_bekker_block_matches_py_source(generated):
    out = generated
    for key, b in B.BODIES.items():
        bk = out[key]["bekker"]
        if b.bekker is None:
            assert bk is None                                # unsourced (Ceres/Bennu/Phobos) -> null
        else:
            assert bk == {"k_c": b.bekker[0], "k_phi": b.bekker[1], "n": b.bekker[2]}


def test_ipex_energy_block_matches_ipex_specs(generated):
    out = generated
    ip = out["_ipex"]
    assert ip["drum_kg"] == S.REGOLITH_PER_CYCLE_KG
    assert ip["dig_j_per_kg"] == round(S.dig_energy_per_kg(), 1)
    assert ip["drive_j_per_m"] == round(S.drive_energy_per_m(), 2)
    assert ip["battery_j"] == round(S.battery_energy_j(), 1)
    assert ip["dig_rate_kg_hr"] == S.DIG_RATE_KG_PER_HR
    assert ip["sinter_enabled"] == K.SINTER_ENABLED          # gate mirrored from constants


def test_ipex_is_underscore_keyed_not_a_body(generated):
    out = generated
    assert "_ipex" in out and "_ipex" not in B.BODIES        # inert to body lookups in the browser


def test_committed_bodies_json_is_in_sync_with_source():
    # the checked-in planet_browser/bodies.json must already reflect the .py source (re-run after edits)
    with open(os.path.join(_HERE, "bodies.json")) as f:
        committed = json.load(f)
    for key, b in B.BODIES.items():
        assert key in committed and committed[key]["g"] == b.g
    assert committed["_ipex"]["sinter_enabled"] == K.SINTER_ENABLED
