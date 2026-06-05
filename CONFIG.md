# Configuration reference

Every tunable in dustgym is adjustable **without editing source**. There are three layers, from
most to least dynamic.

## 1. Per-call / runtime objects (already dynamic)

- **Terramechanics** — `terrain_authority.terramechanics.TerramechanicsParams` is a dataclass you
  construct, serialize to/from JSON (`from_json` / `to_json`), or calibrate from the SCM oracle.
  Pass it into the solver to change Bekker/slip behavior per run.
- **Planetary body** — `terrain_authority.bodies.params_for_body("mars")` returns the gravity-correct,
  literature-sourced parameter set for a body; `RoverSimEnv(..., body="ceres")` selects it. See
  [docs/bodies_sysrev.md](docs/bodies_sysrev.md).

## 2. Environment + file overlay over the module constants (PRD N15)

The module-level constants in `terrain_authority/constants.py` (regolith density, Bekker moduli,
crater/boulder statistics, rover mass, ...) and `terrain_authority/ipex_specs.py` (IPEx mass, speed,
battery, planner knobs) are overridable at import time by an overlay. **Environment wins over file.**

### Environment variables

Set `DUSTGYM_<NAME>` where `<NAME>` is the exact constant name:

```bash
DUSTGYM_RHO_SURFACE=1250 DUSTGYM_ROVER_MASS_DRY_KG=25 python -m planet_browser.server
DUSTGYM_BATTERY_SERIES_CELLS=14 python -c "from terrain_authority import ipex_specs; print(ipex_specs.battery_energy_wh())"
```

Values are coerced to `bool` (`true`/`false`), then `int`, then `float`, else left as text. A name that
is not already a numeric constant is **ignored** (a typo cannot inject a new global).

### TOML file

Point `DUSTGYM_CONFIG` at a TOML file. Top-level keys are constant names; optional `[constants]` and
`[ipex_specs]` tables are also read:

```toml
# my_site.toml
RHO_SURFACE = 1200.0          # looser polar top layer
ROVER_MASS_DRY_KG = 25.0

[ipex_specs]
BATTERY_SERIES_CELLS = 14     # a 14S pack variant
RECHARGE_POWER_W = 900.0
```

```bash
DUSTGYM_CONFIG=my_site.toml python -m planet_browser.server
```

(TOML parsing uses the stdlib `tomllib` on Python ≥ 3.11; on 3.10 `tomli` is installed automatically as
a declared dependency. If `DUSTGYM_CONFIG` is set with no parser available, it raises rather than silently skip.)

### Derived values recompute

Derived constants are recomputed from their (possibly overridden) inputs after the overlay, so override
the **primitive**, not the derived value:

| Override this primitive | Recomputed derived |
|---|---|
| `RHO_SURFACE` | `RHO_SPOIL` |
| `G_s` (or `RHO_WATER`) | `RHO_GRAIN` |
| `BATTERY_SERIES_CELLS`, `BATTERY_CAPACITY_AH`, ... | `battery_energy_j/_wh`, `drive_*`, `dig_*` (functions, read live) |

### Inspect the effective config

```python
from terrain_authority import config
config.describe()   # {'config_file': ..., 'overrides': {...}, 'applied': {'terrain_authority.constants': {'RHO_SURFACE': (1300.0, 1250.0)}}}
```

## 3. Source defaults (the provenance of record)

`constants.py` and `ipex_specs.py` remain the single, provenance-tagged source of truth: every default
carries its `[FIXED]`/`[CALIB]`/`[UNKNOWN]`/`MEASURED` tag and citation. The overlay changes the *value*
at runtime; it does not remove the documented default or its source. Override responsibly — the honesty
tags tell you which values are well-constrained and which are wide-envelope estimates.
