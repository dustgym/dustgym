"""terrain_authority — weekend-slice PHYSICS AUTHORITY for the foss_ipex lunar sim.

A pure NumPy/SciPy analytical Tier-2 surrogate standing in for Project Chrono (see
INTERFACE.md header, spec §2/§3/§9). It PRODUCES the frozen on-disk state-field format
(INTERFACE.md); renderers/visualizers CONSUME it. It is geometry- and state-accurate, not
force-accurate (spec §9), and its headline guarantee is mass conservation (spec §10).

Public surface:
    constants                  — SI physical constants + calibration params (spec §5).
    ColumnState, StateLabel    — per-column data model; mass is the invariant (spec §5.3).
    save_scene / load_scene    — the frozen I/O contract (INTERFACE.md §7).
    write_preview_png / write_hillshade_png — human-inspection previews.
    procgen.*                  — fbm, rolling_hills, flat_compact, carve_crater, boulders.
    Sandpile                   — angle-of-repose relaxation CA (spec §7), the showpiece.
    rover.wheel_pass           — single-pass rut carving (spec §6).
"""

from . import constants
from .column_state import ColumnState, StateLabel, loose_mask
from .io_fields import (
    load_scene,
    save_scene,
    write_hillshade_png,
    write_preview_png,
)
from .sandpile import Sandpile
from . import procgen
from . import rover

__all__ = [
    "constants",
    "ColumnState",
    "StateLabel",
    "loose_mask",
    "save_scene",
    "load_scene",
    "write_preview_png",
    "write_hillshade_png",
    "Sandpile",
    "procgen",
    "rover",
    "register_envs",
]

# Register the Dust/* Gymnasium environments on import (no-op if gymnasium is absent, so the
# bare-numpy core stays importable). After `pip install`, the pyproject entry-point also triggers this.
from .registration import register_envs  # noqa: E402

try:
    register_envs()
except Exception:                          # pragma: no cover - never let registration break import
    pass
