"""Externalized configuration overlay (PRD N15 / area O).

Every module-level scalar constant in ``constants.py`` and ``ipex_specs.py`` is overridable
WITHOUT editing source, via two layers (environment wins over file):

  1. A TOML file pointed to by ``$DUSTGYM_CONFIG``. Top-level keys are constant names, and an
     optional ``[constants]`` / ``[ipex_specs]`` table is also read::

         RHO_SURFACE = 1250.0
         ROVER_MASS_DRY_KG = 25.0
         [ipex_specs]
         BATTERY_SERIES_CELLS = 14

  2. Environment variables ``DUSTGYM_<NAME>``, e.g. ``DUSTGYM_RHO_SURFACE=1250``.

Derived constants (``RHO_GRAIN``, ``RHO_SPOIL``, ...) are RECOMPUTED by the defining module
from their (possibly overridden) inputs after :func:`apply`, so override the PRIMITIVE
(``RHO_SURFACE``), not the derived value. ``ipex_specs`` exposes its energy/battery quantities
as functions that read the constants live, so overriding a constant there auto-recomputes.

Per-body terramechanics (``bodies.params_for_body``) and ``TerramechanicsParams`` are already
runtime-adjustable (JSON / kwargs); this overlay covers the module-level ``.py`` constants the
rest of the code reads as literals. See ``CONFIG.md``.
"""
from __future__ import annotations

import os

_ENV_PREFIX = "DUSTGYM_"
_CONFIG_ENV = "DUSTGYM_CONFIG"


def _coerce(v):
    """Coerce a raw env/TOML value to bool/int/float, else leave as-is."""
    if isinstance(v, (bool, int, float)):
        return v
    s = str(v).strip()
    low = s.lower()
    if low in ("true", "false"):
        return low == "true"
    try:
        return int(s)
    except ValueError:
        pass
    try:
        return float(s)
    except ValueError:
        return s


def _load_toml() -> dict:
    """Parse the $DUSTGYM_CONFIG TOML file (top-level + [constants]/[ipex_specs] tables)."""
    path = os.environ.get(_CONFIG_ENV)
    if not path:
        return {}
    try:
        import tomllib as _toml          # Python >= 3.11
    except ModuleNotFoundError:
        try:
            import tomli as _toml         # Python 3.10 backport
        except ModuleNotFoundError as e:  # honest failure, not a silent skip
            raise ModuleNotFoundError(
                f"{_CONFIG_ENV}={path!r} is set but no TOML parser is available; "
                "install 'tomli' on Python < 3.11 (it ships in the dev extra)."
            ) from e
    with open(path, "rb") as fh:
        data = _toml.load(fh)
    flat = {k: v for k, v in data.items() if not isinstance(v, dict)}
    for section in ("constants", "ipex_specs"):
        if isinstance(data.get(section), dict):
            flat.update(data[section])
    return flat


def get_overrides() -> dict:
    """The merged override map: TOML file first, then ``DUSTGYM_<NAME>`` env vars (env wins)."""
    out = {k: _coerce(v) for k, v in _load_toml().items()}
    for k, v in os.environ.items():
        if k.startswith(_ENV_PREFIX) and k != _CONFIG_ENV:
            out[k[len(_ENV_PREFIX):]] = _coerce(v)
    return out


# Record of what apply() changed, keyed by the namespace module name -> {NAME: (old, new)}.
_APPLIED: dict[str, dict] = {}


def apply(ns: dict) -> dict:
    """Override matching module-level numeric constants in ``ns`` (a module ``globals()``).

    Only names that ALREADY exist in ``ns`` and currently hold an int/float/bool are
    overridden (so a typo cannot inject a new global). An overridden float stays a float.
    Returns ``{name: (old, new)}`` for what changed.
    """
    overrides = get_overrides()
    modname = ns.get("__name__", "?")
    applied: dict = {}
    for name, val in overrides.items():
        if name in ns and isinstance(ns[name], (bool, int, float)):
            old = ns[name]
            # bool is an int subclass -> keep bool override as bool; keep float as float
            if isinstance(old, bool):
                new = bool(val)
            elif isinstance(old, float) and not isinstance(val, bool):
                new = float(val)
            else:
                new = val
            ns[name] = new
            applied[name] = (old, new)
    if applied:
        _APPLIED[modname] = applied
    else:
        _APPLIED.pop(modname, None)   # a clean reload clears any stale record
    return applied


def describe() -> dict:
    """The effective overlay: the config file, the merged overrides, and what was applied."""
    return {
        "config_file": os.environ.get(_CONFIG_ENV),
        "overrides": get_overrides(),
        "applied": {k: dict(v) for k, v in _APPLIED.items()},
    }
