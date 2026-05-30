#!/usr/bin/env python3
"""Convert the EZ-RASSOR rover DAE meshes to glTF (.glb) for the Godot sidecar.

EZ-RASSOR ships its rover description as Collada/DAE (Z-up, meters, embedded vertex
colors, no textures) under ezrassor_sim_description/meshes/. Godot 4 imports glTF/GLB
natively and reads it at runtime via GLTFDocument; DAE import is legacy/partial. So we
convert once here.

Coordinate fix: DAE is Z-up (ROS/Gazebo, REP-103); Godot/our field-space is Y-up
(INTERFACE.md §3). We apply a -90 deg rotation about X (Z-up -> Y-up).

LICENSE: the EZ-RASSOR meshes are MIT (c) UCF / Florida Space Institute / NASA. They are
vendored under .vendor/ and the converted .glb keeps that license (see THIRD_PARTY.md);
this is NOT covered by the repo's CC0. The unlicensed extra_models/ props are NOT used.

Usage:
    .venv/bin/python scripts/convert_rover_mesh.py
"""
from __future__ import annotations

import os

import numpy as np
import trimesh

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_DIR = os.path.join(
    ROOT, ".vendor", "EZ-RASSOR", "packages", "simulation",
    "ezrassor_sim_description", "meshes")
OUT_DIR = os.path.join(ROOT, "godot_sidecar", "assets")

# Z-up (DAE) -> Y-up (Godot/INTERFACE.md §3): rotate -90 deg about X.
ZUP_TO_YUP = trimesh.transformations.rotation_matrix(-np.pi / 2.0, [1, 0, 0])
# URDF scale macro (ezrassor_assets.md §4): every rover mesh is referenced at 0.35.
URDF_SCALE = 0.35
SCALE_M = trimesh.transformations.scale_matrix(URDF_SCALE)


def _describe(scene, src_name: str, dst_name: str) -> None:
    lo, hi = scene.bounds
    size = hi - lo
    center = 0.5 * (lo + hi)
    nverts = sum(int(g.vertices.shape[0]) for g in scene.geometry.values())
    print(f"  {src_name} -> {dst_name}")
    print(f"     geometries={len(scene.geometry)}  vertices={nverts}")
    print(f"     AABB lo=({lo[0]:.4f},{lo[1]:.4f},{lo[2]:.4f}) "
          f"hi=({hi[0]:.4f},{hi[1]:.4f},{hi[2]:.4f}) m")
    print(f"     AABB size (x,y_up,z) = ({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f}) m")
    print(f"     AABB center = ({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}) "
          f"-> origin offset from center = "
          f"({-center[0]:.4f}, {-center[1]:.4f}, {-center[2]:.4f})")


def convert_chassis(dae_name: str, glb_name: str) -> None:
    """The standalone chassis: ground-re-origin so the glb's origin is the
    ground-contact point (center in X/Z, lowest vertex at y=0). This is correct
    ONLY for the chassis used by itself (the chassis-only fallback path), where
    the sidecar snaps the model's origin straight onto the terrain height.

    NOTE: the articulated build (default) ALSO uses this glb as the body, but it
    parents it under a rover-root and snaps the WHOLE assembly at the root, so the
    chassis-local ground offset is harmless there (it just shifts the body within
    the root, and the wheels/arms are positioned relative to the same root). We
    keep the native-origin sub-parts honest to their joint centers; the chassis is
    the one part whose re-origin we tolerate because it backstops the fallback.
    """
    src = os.path.join(SRC_DIR, dae_name)
    scene = trimesh.load(src, force="scene")
    scene.apply_transform(ZUP_TO_YUP)
    scene.apply_transform(SCALE_M)

    lo, hi = scene.bounds
    ground = trimesh.transformations.translation_matrix(
        [-0.5 * (lo[0] + hi[0]), -lo[1], -0.5 * (lo[2] + hi[2])])
    scene.apply_transform(ground)

    os.makedirs(OUT_DIR, exist_ok=True)
    dst = os.path.join(OUT_DIR, glb_name)
    scene.export(dst)
    _describe(scene, dae_name, glb_name)
    size = scene.bounds[1] - scene.bounds[0]
    print(f"     y-extent (height) = {size[1]:.4f} m  -> "
          f"{'looks upright' if size[1] < max(size[0], size[2]) else 'CHECK orientation'}")


def convert_part(dae_name: str, glb_name: str) -> None:
    """A kinematic SUB-PART (wheel / drum / drum_arm).

    Apply ONLY the Z-up->Y-up rotation + URDF 0.35 scale. Do NOT ground-re-origin.
    The DAEs are authored with each part's native local origin already AT the
    part's joint / rotation center (verified by inspecting bounds: the wheel and
    drum are symmetric about (0,0) in their spin plane; the drum_arm's origin sits
    at its proximal hinge end). Preserving that origin is exactly what lets the
    Godot sidecar place the part's Node3D at the URDF joint origin (mapped to Y-up)
    and have the hierarchy compose correctly -- a rotation applied to the Node3D
    spins/pitches the mesh about its true joint center, no per-part offset needed.
    Re-origining a sub-part would silently move its pivot off the joint axis and
    make articulation wrong.
    """
    src = os.path.join(SRC_DIR, dae_name)
    scene = trimesh.load(src, force="scene")
    scene.apply_transform(ZUP_TO_YUP)
    scene.apply_transform(SCALE_M)

    os.makedirs(OUT_DIR, exist_ok=True)
    dst = os.path.join(OUT_DIR, glb_name)
    scene.export(dst)
    _describe(scene, dae_name, glb_name)


def main() -> int:
    print(f"Converting EZ-RASSOR rover meshes (MIT, vendored) -> {OUT_DIR}")
    # Chassis, ground-re-origined: the STANDALONE chassis-only fallback path
    # (sidecar snaps this glb's origin straight onto the terrain height).
    convert_chassis("base_unit.dae", "rover_base.glb")
    # Sub-parts AND the assembly body: native-origin preserved (URDF base_link /
    # joint centers) so the Godot hierarchy composes at the §3 joint origins.
    # The body keeps its native base_link origin so the chassis floats above the
    # wheel centers exactly as in the URDF (body bottom ~ -0.06 m, wheels reach
    # -0.18 m); ground-snap happens ONCE at the rover root in the sidecar.
    convert_part("base_unit.dae", "rover_body.glb")
    convert_part("wheel.dae", "wheel.glb")
    convert_part("drum.dae", "drum.glb")
    convert_part("drum_arm.dae", "drum_arm.glb")
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
