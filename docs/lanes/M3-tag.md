# Lane M3-tag — 4-face all-LIT AprilTag lander bundle

Builds a 4-face `tag36h11` bundle (ids 0–3, 0.150 m) on the lander and captures it
under the grazing polar sun so detection **degrades in shadow** — GMRO's primary
interest. Owns `godot_sidecar/lander_bundle.gd`; **does not touch the frozen
`apriltag_gen.gd`** (the M1 unlit front-tag path).

## Run

```bash
cd godot_sidecar
xvfb-run -a ./render_layers.sh -- --scene ../samples/crater_boulders \
    --layers terrain,clasts,rover --lander-faces --drums-up
```

> `--lander-faces` does **not** auto-raise the drums or add the rover layer — pass
> `--drums-up` (so the lowered drum doesn't occlude the tag) and `--layers …,rover`.

## Outputs
`godot_sidecar/out/lander_faces/<scene>/` — one inspection PNG per face + the rig
stereo PNGs + `sensors.json` carrying `lander.apriltags[]` (4 entries). Gitignored.

## Geometry & decisions
- **All 4 faces LIT** (StandardMaterial3D, albedo ~0.85, roughness 0.7): anti-sun faces
  fall into deep shadow (id1 sun-facing luma ~0.26 vs anti-sun ~0.02 — the deliverable).
- Faces at yaw {0,90,180,270}° about lander +Y: id0 +X (front), id1 +Z, id2 −X, id3 −Z.
- **Lander origin stays at the front (id0) tag center** → `id0` has identity
  `pose_in_lander`, so `R_face(id0) == R_LANDER_TAG` (M1-invariance supersedes §4's
  body-center origin). Faces 1–3 offset by the body half-extents.
- **Re-baseline (intended):** because the bundle front is now lit, a pose read off the
  *bundle* front won't match the unlit M1 `--cameras` reading (12.7 mm / 7.15°). The M1
  `--cameras` single-tag path is **unchanged**; only the bundle's photometry differs.
- ids 1–3 are canonical AprilRobotics bitmaps, id0-anchored (byte-matches the frozen
  `TAG36H11_ID0`). **Detector decode of ids 1–3 is the ROS C1 lane's acceptance test**,
  not this Godot lane's.
