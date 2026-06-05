"""N15 externalized config overlay (area O): env / DUSTGYM_CONFIG override module constants.

Every module-level scalar in constants.py / ipex_specs.py is overridable without editing
source, via DUSTGYM_<NAME> env vars or a DUSTGYM_CONFIG TOML file (env wins). Derived values
recompute from their (possibly overridden) inputs.
"""
import importlib
import os
import textwrap

import pytest

from terrain_authority import config, constants, ipex_specs


@pytest.fixture
def clean_reload():
    """After each test, strip DUSTGYM_* env and reload the constant modules to defaults
    so an override cannot leak into the rest of the suite."""
    yield
    for var in [v for v in os.environ if v.startswith("DUSTGYM_")]:
        del os.environ[var]
    importlib.reload(constants)
    importlib.reload(ipex_specs)


def test_no_env_is_identity(clean_reload):
    importlib.reload(constants)
    assert constants.RHO_SURFACE == 1300.0
    assert constants.RHO_SPOIL == 1300.0           # derived
    assert constants.RHO_GRAIN == 3100.0           # derived
    assert config.get_overrides() == {}


def test_env_override_scalar_recomputes_derived(clean_reload, monkeypatch):
    monkeypatch.setenv("DUSTGYM_RHO_SURFACE", "1250")
    importlib.reload(constants)
    assert constants.RHO_SURFACE == 1250.0
    assert constants.RHO_SPOIL == 1250.0           # RHO_SPOIL tracks the loose surface -> recomputed


def test_env_override_grain_inputs_recompute(clean_reload, monkeypatch):
    monkeypatch.setenv("DUSTGYM_G_s", "3.0")
    importlib.reload(constants)
    assert constants.G_s == 3.0
    assert constants.RHO_GRAIN == 3.0 * constants.RHO_WATER   # 3000, recomputed


def test_toml_file_override(clean_reload, monkeypatch, tmp_path):
    cfg = tmp_path / "dust.toml"
    cfg.write_text(textwrap.dedent("""
        RHO_SURFACE = 1200.0
        ROVER_MASS_DRY_KG = 25.0
    """))
    monkeypatch.setenv("DUSTGYM_CONFIG", str(cfg))
    importlib.reload(constants)
    assert constants.RHO_SURFACE == 1200.0
    assert constants.ROVER_MASS_DRY_KG == 25.0


def test_toml_section_table(clean_reload, monkeypatch, tmp_path):
    cfg = tmp_path / "dust.toml"
    cfg.write_text("[constants]\nCOHESION = 200.0\n")
    monkeypatch.setenv("DUSTGYM_CONFIG", str(cfg))
    importlib.reload(constants)
    assert constants.COHESION == 200.0


def test_env_wins_over_file(clean_reload, monkeypatch, tmp_path):
    cfg = tmp_path / "dust.toml"
    cfg.write_text("RHO_SURFACE = 1200.0\n")
    monkeypatch.setenv("DUSTGYM_CONFIG", str(cfg))
    monkeypatch.setenv("DUSTGYM_RHO_SURFACE", "1100")
    importlib.reload(constants)
    assert constants.RHO_SURFACE == 1100.0


def test_unknown_key_not_injected(clean_reload, monkeypatch):
    monkeypatch.setenv("DUSTGYM_NOT_A_REAL_CONSTANT", "5")
    importlib.reload(constants)
    assert not hasattr(constants, "NOT_A_REAL_CONSTANT")    # a typo cannot create a global


def test_ipex_specs_override_recomputes_functions(clean_reload, monkeypatch):
    base = ipex_specs.battery_energy_j()
    monkeypatch.setenv("DUSTGYM_BATTERY_SERIES_CELLS", "14")
    importlib.reload(ipex_specs)
    assert ipex_specs.BATTERY_SERIES_CELLS == 14
    # battery_energy_j reads the constant live -> a 14S pack stores more than 12S
    assert ipex_specs.battery_energy_j() > base
    assert ipex_specs.battery_energy_j() == 14 * ipex_specs.LIION_NOMINAL_V_PER_CELL * 30.0 * 3600.0


def test_describe_reports_applied(clean_reload, monkeypatch):
    monkeypatch.setenv("DUSTGYM_RHO_SURFACE", "1234")
    importlib.reload(constants)
    d = config.describe()
    assert d["overrides"]["RHO_SURFACE"] == 1234.0
    assert d["applied"]["terrain_authority.constants"]["RHO_SURFACE"][1] == 1234.0


def test_describe_clears_when_default(clean_reload, monkeypatch):
    monkeypatch.setenv("DUSTGYM_RHO_SURFACE", "1234")
    importlib.reload(constants)
    assert "terrain_authority.constants" in config.describe()["applied"]
    monkeypatch.delenv("DUSTGYM_RHO_SURFACE")
    importlib.reload(constants)
    # a clean reload must clear the stale applied record (no leak into describe)
    assert "terrain_authority.constants" not in config.describe()["applied"]
