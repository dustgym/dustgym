# NASA RASSOR rover mesh

The real NASA RASSOR 3D model, pulled from NASA Science 3D Resources.

- **Source:** https://science.nasa.gov/3d-resources/regolith-advanced-surface-systems-operations-robot-rassor/
- **Credit:** NASA/Dewey L. Smith; NASA/Jason M. Schuler (KSC Swamp Works / IPEx team)
- **License/status:** NASA 3D Resources are "free and without copyright" (github.com/nasa/NASA-3D-Resources),
  subject to NASA media-usage guidelines (attribution, no implied endorsement). See `../../../THIRD_PARTY.md`.

## Files

| File | What |
|---|---|
| `rassor.glb` | The original NASA download (6.28 MB), **Draco-compressed** (`KHR_draco_mesh_compression`), one static `rassor_shrinkwrap` mesh, 2.03 M faces. Real RASSOR scale ~0.85 × 0.94 × 1.66 m (per the glTF POSITION accessor). |
| `rassor_godot.glb` | Decoded + decimated derivative (3.66 MB, 177 k faces, **no Draco**) so the vendored Godot 4.6 (which lacks a Draco plugin) can load it. Godot-load-verified; AABB (0.85, 0.93, 1.66) m. |

## Decode pipeline (one-time, not in CI)

The vendored Godot, Blender, and trimesh all lacked a Draco decoder here; `gltf-transform`'s CLI was
broken by its `sharp` dependency. The decode was done with **DracoPy** (decode the draco bufferView) +
**fast_simplification** (decimate to ~177 k faces) + trimesh export — both installed into an isolated
`--target` dir, not the shared venv. `rassor_godot.glb` is committed because that toolchain is not present
in CI / a fresh checkout.

## Use

RASSOR is the **precursor** to the modelled IPEx. This is a **single static visual mesh** (not the
articulated per-part assembly the IPEx/EZ-RASSOR render path uses), intended as a static render body for
the `rassor2` vehicle (`terrain_authority/vehicles.py`). A render preview is at `../../out/rassor_nasa.png`.
