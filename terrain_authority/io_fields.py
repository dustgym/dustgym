"""On-disk state-field I/O — the FROZEN decoupling seam (INTERFACE.md §1, §2, §7).

This module is the ONLY place raw raster bytes are written/read on the Python side. All
Python consumers import save_scene/load_scene; they never re-implement the format
(INTERFACE.md §7). Godot has its own GDScript loader against the same spec.

Format (INTERFACE.md §2):
    .rf32  little-endian IEEE-754 float32  (numpy dtype '<f4'), row-major C, no header.
    .r8    unsigned 8-bit                  (numpy dtype 'u1'),  row-major C, no header.
    element k = row*width + col.

metadata.json is written FIRST (INTERFACE.md §6 "Emit metadata.json first / atomically")
so a consumer that sees it can trust the rasters it then opens.
"""

from __future__ import annotations

import json
import os
from typing import Any

import numpy as np

#: Map field name -> on-disk dtype + filename (INTERFACE.md §1/§5).
_FIELD_SPEC: dict[str, tuple[str, str]] = {
    "heightmap": ("<f4", "heightmap.rf32"),
    "mass_areal": ("<f4", "mass_areal.rf32"),
    "density": ("<f4", "density.rf32"),
    "disturbance": ("<f4", "disturbance.rf32"),
    "state_label": ("u1", "state_label.r8"),
    "ice": ("<f4", "ice.rf32"),
}


def save_scene(scene_dir: str, fields: dict[str, np.ndarray], metadata: dict[str, Any]) -> None:
    """Write one scene snapshot to ``scene_dir`` per INTERFACE.md §1.

    metadata.json is written first; then each field is dumped as raw row-major bytes via
    ``arr.astype(dtype).tofile(path)`` (already C-order from NumPy). REQUIRED fields
    (INTERFACE.md §1): heightmap, mass_areal, density, disturbance, state_label.
    """
    os.makedirs(scene_dir, exist_ok=True)

    required = ["heightmap", "mass_areal", "density", "disturbance", "state_label"]
    missing = [f for f in required if f not in fields]
    if missing:
        raise ValueError(f"save_scene: missing REQUIRED fields {missing} (INTERFACE.md §1)")

    # Validate every raster matches the grid dims from metadata (INTERFACE.md §6).
    w = metadata["grid"]["width"]
    h = metadata["grid"]["height"]
    for name, arr in fields.items():
        if arr.shape != (h, w):
            raise ValueError(
                f"save_scene: field '{name}' shape {arr.shape} != (height,width)=({h},{w})"
            )

    # metadata.json FIRST (INTERFACE.md §6).
    with open(os.path.join(scene_dir, "metadata.json"), "w") as fh:
        json.dump(metadata, fh, indent=2)

    for name, arr in fields.items():
        if name not in _FIELD_SPEC:
            continue  # ignore non-contract extras
        dtype, fname = _FIELD_SPEC[name]
        arr.astype(dtype).tofile(os.path.join(scene_dir, fname))


def load_scene(scene_dir: str) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """Read a scene snapshot back into (fields, metadata) per INTERFACE.md §1/§2.

    Reads metadata first (INTERFACE.md §6), then reshapes each raw raster to
    (height, width) using the grid dims. OPTIONAL ice.rf32 is loaded if present.
    """
    with open(os.path.join(scene_dir, "metadata.json")) as fh:
        metadata = json.load(fh)
    w = metadata["grid"]["width"]
    h = metadata["grid"]["height"]

    fields: dict[str, np.ndarray] = {}
    for name, (dtype, fname) in _FIELD_SPEC.items():
        path = os.path.join(scene_dir, fname)
        if not os.path.exists(path):
            continue
        arr = np.fromfile(path, dtype=dtype).reshape(h, w)
        fields[name] = arr
    return fields, metadata


# ---------------------------------------------------------------------------
# Human-inspection previews (OPTIONAL, not consumed by Godot; INTERFACE.md §1).
# Uses matplotlib's Agg backend so it runs headless with no display.
# ---------------------------------------------------------------------------

def _agg_plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def write_preview_png(field: np.ndarray, path: str, cmap: str = "viridis",
                      title: str | None = None) -> None:
    """Write a colormapped PNG of a scalar field for human inspection (preview_*.png)."""
    plt = _agg_plt()
    fig, ax = plt.subplots(figsize=(5, 5), dpi=110)
    im = ax.imshow(field, origin="lower", cmap=cmap, interpolation="nearest")
    ax.set_xlabel("col (+X)")
    ax.set_ylabel("row (+Z)")
    if title:
        ax.set_title(title)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def write_hillshade_png(heightmap: np.ndarray, path: str, cell_m: float,
                        altdeg: float = 7.0, azdeg: float = 315.0,
                        title: str | None = None) -> None:
    """Hillshade preview with a grazing lunar sun (INTERFACE.md preview; spec §8).

    Uses matplotlib.colors.LightSource at altdeg~7 deg (polar grazing band, spec §5.1)
    to make the brutal low-sun long shadows that are exactly IPEx's perception challenge
    (spec §8 "grazing-angle conditions are exactly IPEx's perception challenge").
    """
    plt = _agg_plt()
    from matplotlib.colors import LightSource

    ls = LightSource(azdeg=azdeg, altdeg=altdeg)
    # vert_exag scaled so cm-scale relief reads against a metre-scale patch.
    shaded = ls.hillshade(heightmap, vert_exag=1.0, dx=cell_m, dy=cell_m)
    fig, ax = plt.subplots(figsize=(5, 5), dpi=110)
    ax.imshow(shaded, origin="lower", cmap="gray", interpolation="nearest")
    ax.set_xlabel("col (+X)")
    ax.set_ylabel("row (+Z)")
    ax.set_title(title or f"hillshade (sun alt={altdeg}deg, grazing)")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
