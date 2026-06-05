# Contributing to dustgym

Thanks for your interest. dustgym is released into the public domain (CC0 1.0). Contributions are
welcome, but the project holds a few **non-negotiable engineering conventions** that keep its results
trustworthy. Please read these before opening a pull request.

## The honesty conventions (binding)

These are what make a physics simulator citable. They are enforced in review:

- **No synthetic, mock, or randomly generated data** standing in for real measurements — in code, tests,
  figures, or notebooks. Use the real sample scenes (`samples/`), the real LOLA Haworth DEM, and the
  sourced physical constants. For fast tests, subsample real data into a small fixture; never fabricate.
- **No stubs, placeholder returns, or demo modes** substituting for the real pipeline. Implement it for
  real or leave it out and say so.
- **Provenance tags are part of the code.** Every physical constant carries its source and a status tag:
  `MEASURED` / `ESTIMATED` / `[CALIB]` (calibration-pending) / `[UNKNOWN]`. If you add or change a
  constant, tag it accurately and cite the source. Do not silently promote an `[UNKNOWN]` to a number.
- **Mass is conserved by construction.** Agents, planners, and learned policies **command**; only the
  terramechanics authority **mutates** terrain. Never let a learned component write terrain state
  directly — that is what makes the terrain-matching reward unhackable.

## Architecture you should not violate

- **Single physics authority.** Project Chrono / the conserved NumPy Tier-2 authority owns all dynamics.
  Renderers and sensor models are downstream consumers, not co-authorities.
- **Frozen seams.** Seam 1 (state fields on disk, `INTERFACE.md`) and Seam 2 (`sensors.json` + PNGs,
  `docs/sensor_bridge_contract.md`) are contracts. Extend them additively; do not break them.

## Development setup

```bash
# from the repo root, with a Python >= 3.10 environment
pip install -e .[dev]                  # ruff, pytest, planner extras
PYTHONPATH=. python -m pytest terrain_authority planet_browser -q
ruff check --select F terrain_authority planet_browser
```

The hot path (`terrain_authority/`) is **NumPy-only** — keep heavy/optional deps (torch, gymnasium,
matplotlib) behind imports or extras so the core stays importable on a bare install.

## Pull request checklist

Before you open a PR, confirm:

- [ ] The full test suite passes (`pytest terrain_authority planet_browser`).
- [ ] `ruff check --select F` is clean.
- [ ] New behavior is covered by a test built from **real** data (TDD: write the failing test first).
- [ ] New constants are provenance-tagged and cited.
- [ ] No synthetic data, stubs, demos, or TODO/placeholder markers were introduced.
- [ ] Public seams (`INTERFACE.md`, the sensor-bridge contract) are unchanged or extended additively.

CI runs the suite on every push. Keep changes focused; unrelated refactors belong in their own PR.

## Reporting bugs / requesting features

Use the issue templates. For security issues, see [`SECURITY.md`](SECURITY.md).
