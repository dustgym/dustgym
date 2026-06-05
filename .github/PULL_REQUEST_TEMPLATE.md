<!-- Thanks for contributing to dustgym. Please confirm the checklist below. -->

## What & why

<!-- One or two sentences: what does this change and why. -->

## Checklist

- [ ] Full test suite passes: `PYTHONPATH=. python -m pytest terrain_authority planet_browser`
- [ ] `ruff check --select F terrain_authority planet_browser` is clean
- [ ] New behavior is covered by a test built from **real** data (TDD: failing test first)
- [ ] Any new physical constant is provenance-tagged (`MEASURED` / `ESTIMATED` / `[CALIB]` / `[UNKNOWN]`) and cited
- [ ] No synthetic data, stubs, demos, or TODO/placeholder markers introduced
- [ ] Public seams (`INTERFACE.md`, `docs/sensor_bridge_contract.md`) unchanged or extended additively

See [CONTRIBUTING.md](../CONTRIBUTING.md) for the full conventions.
