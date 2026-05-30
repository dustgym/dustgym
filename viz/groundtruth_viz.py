"""D1b — NATIVE (matplotlib) 3D ground-truth visualizer for the Chrono-side scene.

This renders the *spatial contents of the simulation* — the thing the robot's SLAM stack
would be trying to reconstruct (spec §2 "two ground-truth comparisons": built/observed
elevation map vs. Chrono true terrain *at time t*). It is the GROUND-TRUTH half of that
comparison drawn directly from the frozen state-field contract, NOT a beauty render and
NOT the Godot sensor-model path. Matplotlib / Agg, fully headless.

What is drawn (spec §4 "the quadtree manages space; the active grid is the physics
substrate"):

  * ACTIVE-ZONE COLUMNS as 3D CUBOIDS (bar3d), colored by state_label
    (VIRGIN grey / TREAD darker / EXCAVATED bright dense-sublayer /
     SPOIL loose-bright / COMPACTED_BERM). The fine uniform grid inside the active
    patch IS the terramechanics solve substrate (spec §4 "run the solve on a uniform
    fine grid inside each active patch"). We DOWNSAMPLE that window to <=32x32 cuboids
    so the schematic stays legible — see DOWNSAMPLE_MAX below and the caption.

  * QUADTREE non-leaf nodes as WIREFRAME boxes, so the LOD / hot-region space-management
    structure (spec §4 "the tree manages space; it is not the physics substrate") is
    visible. Each metadata quadtree[] entry is a [row0,col0]-anchored box of `size` cells.

  * CLASTS as marker spheres at their center_m, sized by radius_m (INTERFACE.md §5
    clasts[].center_m is world [x, height_up, z], Godot-ready order). Uncovered clasts are
    the loop-closure perception payoff (spec §6 "new hard shadow at grazing sun ->
    deceptive perception feature"); here we just show their true ground-truth positions.

Coordinate convention (INTERFACE.md §3): field index[row,col] -> world x = col*cell_m,
z = row*cell_m, height = value (up). We plot x and z as the ground plane and height as the
vertical axis, matching the Godot Y-up mapping so this view and the rendered view agree.

CLI:
    python viz/groundtruth_viz.py <scene_dir> [--out path] [--turntable]

Run with the project venv python. Outputs PNGs under viz/out/.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# Headless: force Agg before importing pyplot (INTERFACE.md previews are display-free).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Import the package the supported way (shared SHARED CONTEXT note: `import terrain_authority`).
# Make the repo root importable whether this is run as a script or a module.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from terrain_authority.io_fields import load_scene  # noqa: E402
from terrain_authority import constants as K  # noqa: E402

# ---------------------------------------------------------------------------
# State-label -> color palette (INTERFACE.md §4 enum; spec §6 transitions).
# Greyscale-anchored so it reads as "regolith", with brightness encoding how the
# material has been WORKED: EXCAVATED exposes the brighter dense sublayer (spec §6
# "expose dense sublayer ... brighter albedo"); SPOIL is loose-bright fresh ejecta;
# COMPACTED_BERM is the deliberately-built structure (spec §6 SPOIL->BERM).
# ---------------------------------------------------------------------------
STATE_COLORS = {
    K.STATE_VIRGIN:         "#8a8a8a",  # undisturbed mare regolith — neutral grey
    K.STATE_TREAD:          "#5c5c5c",  # wheel-paved rut — darker, compacted (spec §6)
    K.STATE_EXCAVATED:      "#d9c98a",  # fresh drum cut — bright dense sublayer exposed
    K.STATE_SPOIL:          "#e8e2c0",  # dumped loose spoil — loose-bright
    K.STATE_COMPACTED_BERM: "#b89a5a",  # built berm — compacted spoil
}
STATE_NAME = dict(enumerate(K.STATE_NAMES))

#: Max cuboids per side in the active window (legibility cap; documented in caption).
DOWNSAMPLE_MAX = 32

#: Fixed oblique camera (degrees) — a single canonical viewpoint for all stills.
CAM_ELEV = 32.0
CAM_AZIM = -55.0


def _downsample_block(field: np.ndarray, out_h: int, out_w: int, mode: bool = False) -> np.ndarray:
    """Block-reduce ``field`` to (out_h, out_w).

    mode=False -> block MEAN (continuous fields: height, disturbance, datum).
    mode=True  -> block MODE / majority (discrete state_label, so a downsampled cuboid
                  takes the label of the material that dominates its footprint).
    """
    h, w = field.shape
    out_h = min(out_h, h)
    out_w = min(out_w, w)
    row_idx = np.linspace(0, h, out_h + 1).astype(int)
    col_idx = np.linspace(0, w, out_w + 1).astype(int)
    out = np.zeros((out_h, out_w), dtype=(np.int64 if mode else np.float64))
    for i in range(out_h):
        for j in range(out_w):
            block = field[row_idx[i]:row_idx[i + 1], col_idx[j]:col_idx[j + 1]]
            if block.size == 0:
                continue
            if mode:
                vals, counts = np.unique(block, return_counts=True)
                out[i, j] = int(vals[np.argmax(counts)])
            else:
                out[i, j] = float(block.mean())
    return out, row_idx, col_idx


def _box_edges(x0, x1, y0, y1, z0, z1):
    """Return the 12 edge segments of an axis-aligned box for a Line3DCollection."""
    c = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # bottom
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # top
    ]
    pairs = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
    ]
    return [(c[a], c[b]) for a, b in pairs]


def render_scene(scene_dir: str, out_path: str, elev: float = CAM_ELEV,
                 azim: float = CAM_AZIM) -> str:
    """Render one ground-truth still of ``scene_dir`` to ``out_path`` (PNG). Returns the path."""
    fields, meta = load_scene(scene_dir)
    cell_m = float(meta["grid"]["cell_m"])
    H = fields["heightmap"]
    SL = fields["state_label"]

    # --- active zone window (spec §4 "under wheels/drums" fine-solve patch) ----------
    az = meta["active_zone"]
    r0, c0 = az["min_rc"]
    r1, c1 = az["max_rc"]
    h_sub = H[r0:r1, c0:c1]
    sl_sub = SL[r0:r1, c0:c1]

    # Downsample the active window to <=DOWNSAMPLE_MAX cuboids/side for legibility.
    nh, nw = h_sub.shape
    out_h = min(DOWNSAMPLE_MAX, nh)
    out_w = min(DOWNSAMPLE_MAX, nw)
    h_ds, _, _ = _downsample_block(h_sub, out_h, out_w, mode=False)
    sl_ds, _, _ = _downsample_block(sl_sub, out_h, out_w, mode=True)

    # World-space cuboid footprints. Each downsampled cuboid spans a block of the active
    # window; its base sits at the scene floor and it rises to the (mean) column height.
    # x = col*cell_m, z = row*cell_m (INTERFACE.md §3).
    x_left = (c0 + np.linspace(0, nw, out_w + 1)[:-1]) * cell_m
    z_left = (r0 + np.linspace(0, nh, out_h + 1)[:-1]) * cell_m
    dx_edges = (c0 + np.linspace(0, nw, out_w + 1)) * cell_m
    dz_edges = (r0 + np.linspace(0, nh, out_h + 1)) * cell_m
    dxw = np.diff(dx_edges)   # per-column cuboid width in x
    dzw = np.diff(dz_edges)   # per-row    cuboid width in z

    floor = float(min(H.min(), 0.0))  # common base so cuboid HEIGHT reads as relief

    # Flatten cuboids for bar3d (vectorized over the whole active window).
    XX, ZZ = np.meshgrid(x_left, z_left)          # (out_h, out_w)
    DXX, _ = np.meshgrid(dxw, dzw)                 # widths in x
    _, DZZ = np.meshgrid(dxw, dzw)                 # widths in z
    bottoms = np.full_like(h_ds, floor)
    heights = h_ds - floor
    colors = np.empty(h_ds.shape, dtype=object)
    for lbl, col in STATE_COLORS.items():
        colors[sl_ds == lbl] = col

    fig = plt.figure(figsize=(11, 9), dpi=120)
    ax = fig.add_subplot(111, projection="3d")

    # --- cuboids (the physics substrate) -------------------------------------------
    ax.bar3d(
        XX.ravel(), ZZ.ravel(), bottoms.ravel(),
        DXX.ravel(), DZZ.ravel(), heights.ravel(),
        color=colors.ravel(), shade=True, edgecolor="#222222", linewidth=0.15,
        zsort="max",
    )

    # --- quadtree wireframe boxes (the space-management structure, spec §4) ---------
    qt_segments = []
    top = float(H.max())
    for node in meta.get("quadtree", []):
        qr0, qc0 = node["row0"], node["col0"]
        size = node["size"]
        bx0, bx1 = qc0 * cell_m, (qc0 + size) * cell_m
        bz0, bz1 = qr0 * cell_m, (qr0 + size) * cell_m
        # Box spans from the floor up to slightly above the tallest relief so the LOD
        # extent is visible enclosing the cuboids.
        by0, by1 = floor, top + 0.05
        qt_segments.extend(_box_edges(bx0, bx1, by0, by1, bz0, bz1))
    if qt_segments:
        lc = Line3DCollection(qt_segments, colors="#1f77b4", linewidths=1.1, alpha=0.85)
        ax.add_collection3d(lc)

    # --- clasts as spheres/markers (INTERFACE.md §5: center_m = world [x, h_up, z]) --
    clasts = meta.get("clasts", [])
    if clasts:
        cx = np.array([c["center_m"][0] for c in clasts])
        ch = np.array([c["center_m"][1] for c in clasts])  # height_up
        cz = np.array([c["center_m"][2] for c in clasts])
        cr = np.array([c["radius_m"] for c in clasts])
        # marker area ~ projected sphere size; scale radius (m) to points^2 heuristically.
        sizes = np.clip((cr / cell_m) * 30.0, 6.0, 400.0)
        ax.scatter(cx, cz, ch, s=sizes, c="#c0392b", marker="o",
                   depthshade=True, edgecolors="#400000", linewidths=0.3, alpha=0.9,
                   label=f"clasts (n={len(clasts)})")

    # --- camera + axes (Godot-agreeing Y-up: vertical axis is height) ---------------
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x  (world m, +X = +col)")
    ax.set_ylabel("z  (world m, +Z = +row)")
    ax.set_zlabel("height (m, up)")
    # Keep x/z to true world bounds; exaggerate the vertical only via box aspect so the
    # cm-scale relief is legible against the metre-scale patch (documented schematic).
    wb = meta["world_bounds_m"]
    ax.set_xlim(wb["x0"], wb["x1"])
    ax.set_ylim(wb["y0"], wb["y1"])
    ax.set_zlim(floor, max(top + 0.05, floor + 0.05))
    ax.set_box_aspect((1, 1, 0.45))

    # --- title + legend mapping color -> state label (spec §2 ground truth at time t) -
    present = sorted(set(int(v) for v in np.unique(sl_ds)))
    legend_handles = [
        Patch(facecolor=STATE_COLORS[lbl], edgecolor="#222222",
              label=f"{lbl} {STATE_NAME[lbl]}")
        for lbl in present
    ]
    legend_handles.append(
        plt.Line2D([0], [0], color="#1f77b4", lw=1.5, label="quadtree node (LOD/space)")
    )
    if clasts:
        legend_handles.append(
            plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#c0392b",
                       markersize=8, label=f"clasts (n={len(clasts)})")
        )
    ax.legend(handles=legend_handles, loc="upper left", fontsize=8, framealpha=0.9,
              title="state_label -> color")

    scene_name = meta.get("scene_name", os.path.basename(scene_dir.rstrip("/")))
    fig.suptitle(
        f"GROUND TRUTH at time t  -  scene '{scene_name}'   (spec §2: Chrono true terrain "
        f"the SLAM map is scored against)",
        fontsize=12, y=0.97,
    )
    ax.set_title(
        f"active-window cuboids {out_h}x{out_w} (downsampled from {nh}x{nw} solve cells, "
        f"cap {DOWNSAMPLE_MAX}/side; schematic — spec §4 fine-solve substrate)\n"
        f"grid {meta['grid']['width']}x{meta['grid']['height']} @ {cell_m} m/cell  "
        f"|  vertical box-aspect exaggerated for cm-scale relief",
        fontsize=9,
    )

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    fig.savefig(out_path)
    plt.close(fig)
    return out_path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Native matplotlib 3D ground-truth visualizer (D1b, spec §2/§4).")
    ap.add_argument("scene_dir", help="scene directory with metadata.json + rasters")
    ap.add_argument("--out", default=None,
                    help="output PNG path (default viz/out/groundtruth_<scene>.png)")
    ap.add_argument("--turntable", action="store_true",
                    help="also emit a 3-frame elev/azim sweep for this scene")
    args = ap.parse_args(argv)

    if not os.path.isdir(args.scene_dir):
        print(f"scene_dir not found: {args.scene_dir}", file=sys.stderr)
        return 2

    _, meta = load_scene(args.scene_dir)
    scene_name = meta.get("scene_name", os.path.basename(args.scene_dir.rstrip("/")))
    out_dir = os.path.join(_REPO_ROOT, "viz", "out")
    out_path = args.out or os.path.join(out_dir, f"groundtruth_{scene_name}.png")

    written = [render_scene(args.scene_dir, out_path)]
    print(f"wrote {out_path}")

    if args.turntable:
        # 3-frame azimuth sweep around the fixed oblique elevation.
        for i, az in enumerate((-80.0, -55.0, -30.0)):
            tp = os.path.join(out_dir, f"groundtruth_{scene_name}_turn{i}.png")
            render_scene(args.scene_dir, tp, elev=CAM_ELEV, azim=az)
            written.append(tp)
            print(f"wrote {tp}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
