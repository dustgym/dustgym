"""gen_ipex_mesh.py — author a self-made, CC0 parametric IPEx render body (no third-party mesh).

The Godot sidecar assembles the rover from separable part-glbs placed at joint origins
(rover_body + wheel + drum + drum_arm; see godot_sidecar/sidecar.gd). The default parts are
converted from the MIT EZ-RASSOR URDF (a RASSOR-class *demonstration* robot). Because no public /
CC0 mesh of the actual IPEx flight vehicle exists, this script BUILDS one from primitives
(box chassis + cylinder wheels + cylinder bucket drums + box arms), dimensioned from the
published IPEx numbers. We author every vertex, so the output is CC0 (it removes the rover's one
third-party dependency) and stays in sync with the physics constants by construction.

Honest fidelity: this is a DIMENSIONALLY-FAITHFUL PRIMITIVE model, not the IPEx flight CAD. The
wheel (30.5 cm dia) and bucket drum (IPEx medium, 295x246 mm) are sourced; the chassis box,
wheelbase, wheel width and arm length carry NO published IPEx number and are tagged [CALIB] below
(plausible 30 kg-class values, refined when IPEx CAD becomes available). Nothing here is passed off
as flight geometry.

  Sourced: terrain_authority.ipex_specs (Schuler ASCEND 2024 / Zhang wheel testing / Schuler BD scaling).
  Run: python -m scripts.gen_ipex_mesh   ->  writes godot_sidecar/assets/ipex/*.glb
"""
from __future__ import annotations

import os

import numpy as np
import trimesh

from terrain_authority import ipex_specs as ix

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "godot_sidecar", "assets", "ipex")

# ---- sourced dimensions (from ipex_specs / the IPEx papers) --------------------------------
WHEEL_RADIUS_M = ix.WHEEL_RADIUS_M          # 0.1524 m  (30.5 cm dia) [WHEELTEST]
DRUM_DIAMETER_M = 0.295                      # IPEx medium bucket drum, 295.1 mm [BDSCALE]
DRUM_WIDTH_M = 0.246                         # IPEx medium bucket drum, 246.1 mm [BDSCALE]
TRACK_M = round(0.7 * ix.SKID_STEER_TRACK_M, 4)   # IPEx track = 0.7 x RASSOR-2 0.5207 m = 0.3645 m

# ---- [CALIB] render-only dimensions (no published IPEx number; 30 kg-class estimates) -------
WHEEL_WIDTH_M = 0.20                         # [CALIB] wide wheel for obstacle traversal
WHEELBASE_M = 0.30                           # [CALIB] fore/aft wheel spacing
CHASSIS_L_M = 0.34                           # [CALIB] body length
CHASSIS_W_M = 0.30                           # [CALIB] body width (wheels sit outboard of this)
CHASSIS_H_M = 0.18                           # [CALIB] body height
ARM_LENGTH_M = 0.30                          # [CALIB] drum-arm reach from its shoulder pivot
ARM_THICK_M = 0.06                           # [CALIB] arm link cross-section

def _finish(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Consistent outward normals so the part lights correctly (no normals -> the mesh renders black);
    no baked material -- the sidecar applies its own grey rover material, exactly as for the EZ-RASSOR glbs."""
    mesh.fix_normals()
    _ = mesh.vertex_normals          # force normals to exist so the glTF export writes a NORMAL accessor
    return mesh


def _wheel() -> trimesh.Trimesh:
    """Cylinder of the sourced flight radius, axis along Z (circular in the spin plane, centered)."""
    return _finish(trimesh.creation.cylinder(radius=WHEEL_RADIUS_M, height=WHEEL_WIDTH_M, sections=40))


def _drum() -> trimesh.Trimesh:
    """IPEx medium bucket drum as a cylinder (sourced dia/width), centered at its spin axis."""
    return _finish(trimesh.creation.cylinder(radius=DRUM_DIAMETER_M / 2.0, height=DRUM_WIDTH_M, sections=40))


def _chassis() -> trimesh.Trimesh:
    """Box body at base_link (centered)."""
    return _finish(trimesh.creation.box(extents=[CHASSIS_L_M, CHASSIS_W_M, CHASSIS_H_M]))


def _arm() -> trimesh.Trimesh:
    """Box arm link whose pivot end sits at the origin and which extends +X by ARM_LENGTH_M."""
    m = trimesh.creation.box(extents=[ARM_LENGTH_M, ARM_THICK_M, ARM_THICK_M])
    m.apply_translation([ARM_LENGTH_M / 2.0, 0.0, 0.0])     # pivot at origin (matches convert_part convention)
    return _finish(m)


def build_parts() -> dict[str, trimesh.Trimesh]:
    """The four assemble-parts the sidecar places at joint origins (CC0, self-authored)."""
    return {"rover_body": _chassis(), "wheel": _wheel(), "drum": _drum(), "drum_arm": _arm()}


def export_all(out_dir: str = OUT_DIR) -> str:
    os.makedirs(out_dir, exist_ok=True)
    for name, mesh in build_parts().items():
        mesh.export(os.path.join(out_dir, f"{name}.glb"))
    return out_dir


if __name__ == "__main__":
    d = export_all()
    print(f"IPEx primitive body (CC0, self-authored) -> {d}")
    for n, m in build_parts().items():
        print(f"  {n:10s} extents(m)={np.round(m.extents, 3).tolist()}")
