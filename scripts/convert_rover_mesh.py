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


def convert(dae_name: str, glb_name: str) -> None:
    src = os.path.join(SRC_DIR, dae_name)
    scene = trimesh.load(src, force="scene")
    scene.apply_transform(ZUP_TO_YUP)

    # Re-origin so the glb's origin is the ground-contact point: center in X/Z and
    # rest the lowest vertex at y=0. Then the Godot sidecar can place the rover by
    # snapping its origin to the terrain height and it sits ON the surface (the raw
    # DAE origin is near the body center, which half-buries the model).
    lo, hi = scene.bounds
    ground = trimesh.transformations.translation_matrix(
        [-0.5 * (lo[0] + hi[0]), -lo[1], -0.5 * (lo[2] + hi[2])])
    scene.apply_transform(ground)

    os.makedirs(OUT_DIR, exist_ok=True)
    dst = os.path.join(OUT_DIR, glb_name)
    scene.export(dst)

    lo, hi = scene.bounds
    size = hi - lo
    nverts = sum(int(g.vertices.shape[0]) for g in scene.geometry.values())
    print(f"  {dae_name} -> {glb_name}")
    print(f"     geometries={len(scene.geometry)}  vertices={nverts}")
    print(f"     AABB size (x,y_up,z) = ({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f}) m")
    print(f"     y-extent (height) = {size[1]:.3f} m  -> {'looks upright' if size[1] < max(size[0], size[2]) else 'CHECK orientation'}")


def main() -> int:
    print(f"Converting EZ-RASSOR rover meshes (MIT, vendored) -> {OUT_DIR}")
    convert("base_unit.dae", "rover_base.glb")
    print("done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
