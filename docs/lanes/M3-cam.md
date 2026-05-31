# Lane M3-cam — full 8-camera rig

Branched off L0 `e2b0994`. Owner-files: `godot_sidecar/camera_rig.gd`, `docs/lanes/M3-cam.md`.

## What this lane delivers

The "all cameras working" deliverable: `camera_rig.gd` grows from the M1 2-camera
front-stereo table to the full **8-camera rig** reserved in sensor-bridge contract §4.
The six new cameras are pure additions to the declarative `CAMERAS` table, which
`sidecar.gd::_cameras_capture` already iterates generically — so the existing
`--cameras` path now renders all 8 with **no `sidecar.gd` edit**.

| name | frame_id | look | mount (rover-local, m) | notes |
|---|---|---|---|---|
| `front_left`  | `front_left_optical`  | +X (fwd) | (0.30, −0.10, +0.035) | **M1, UNCHANGED** |
| `front_right` | `front_right_optical` | +X (fwd) | (0.30, −0.10, −0.035) | **M1, UNCHANGED** |
| `rear_left`   | `rear_left_optical`   | −X (back) | (−0.30, −0.10, +0.035) | rear stereo, §4 `stereo_rear` |
| `rear_right`  | `rear_right_optical`  | −X (back) | (−0.30, −0.10, −0.035) | rear stereo, §4 `stereo_rear` |
| `left_mono`   | `left_mono_optical`   | +Z (left) | (0.0, −0.05, +0.285) | side monocular |
| `right_mono`  | `right_mono_optical`  | −Z (right) | (0.0, −0.05, −0.285) | side monocular |
| `drum_front_cam` | `drum_front_cam_optical` | aim `arm_front` | (0.10, 0.18, 0.0) | aims at live front drum joint |
| `drum_back_cam`  | `drum_back_cam_optical`  | aim `arm_back`  | (−0.10, 0.18, 0.0) | aims at live back drum joint |

### Geometry provenance
- Front pair: unchanged from M1 — URDF `camera_front_joint` (0.3, 0, −0.1)_zup → Y-up,
  ±`BASELINE_M`/2 (0.070 m) along the lateral (Z) axis.
- Rear pair: mirror of the front module to the back of `base_link` (`REAR_CAM_BACK_M` = −0.30,
  symmetric to `CAM_FORWARD_M`), separated by the **new `REAR_BASELINE_M` const** (0.070 m).
  Looks −X.
- Side monos: at the wheel-pivot track half-width (`SIDE_MONO_LAT_M` = 0.285 m, from
  `sidecar.gd WHEEL_ORIGINS` Z), looking straight out each side.
- Drum cams: mounted on the camera mast above `base_link`; their optical axis is a
  world-space `look_at` toward the **live** drum-arm joint node (`arm_front` / `arm_back`,
  the pivot names from `sidecar.gd::_build_rover`), so the aim tracks the arm's current
  pitch. If the rover is the chassis-only fallback (no named joints) they fall back to a
  fixed fwd+down / back+down look so `build()` never hard-fails.

### Generalized look-basis helper
`forward_look_basis()` is now `look_basis(Vector3(1,0,0))`. `look_basis(fwd_local)` builds a
proper right-handed camera `Basis` (det +1, camera −Z = `fwd_local`, up ≈ world +Y; falls
back to +Z up-hint for a straight up/down look). Verified `forward_look_basis()` is
byte-identical to the original hand-derived M1 basis, and all 8 built bases have det +1.

### Rear-stereo seam (no frozen-file edit)
`sensors_emit.build_sensors_json(...)` already accepts an optional `stereo_rear`
`{left,right,baseline_m}` (it emits a SEPARATE top-level `"stereo_rear"`, never replacing
`"stereo"` — the front pair). This lane exposes the rear pair in exactly that shape:
- `CameraRig.STEREO_PAIRS` — `{front:{...}, rear:{...}}` design-baseline descriptors.
- `CameraRig.rear_pair_descriptor(cams, mount)` — recomputes `baseline_m` from the ACTUAL
  built rear-camera extrinsics (identical-by-construction to
  `|extrinsic(rear_left).pos − rear_right.pos|`, the same way `sensors_emit` derives the
  front baseline), returns `null` if the rear pair was not built.

**Integration (orchestrator wires at merge — see lane structured output `integration_notes`):**
`sidecar.gd::_cameras_capture` (~line 673–676) currently calls
`build_sensors_json(..., sun, null, null)` with `stereo_rear = null`. Change that **one**
last argument to `CameraRigScript.rear_pair_descriptor(cams, rover_root)` and `stereo_rear`
emits. `sidecar.gd` is frozen for this lane, so the rig + descriptor are implemented here
and the 1-line call-site change is left for the merge.

## Run recipe

8-camera capture on an existing scene (deps are symlinked into the worktree;
`docker compose` v2 absent — irrelevant here, this is the Godot side):

```bash
cd godot_sidecar
./render_layers.sh -- \
    --scene ../samples/crater_boulders \
    --layers terrain,clasts,rover \
    --cameras \
    --size 640x480
# writes out/cam/crater_boulders/000/{front_left,front_right,rear_left,rear_right,
#   left_mono,right_mono,drum_front_cam,drum_back_cam}.png + sensors.json
```

`--cam-pitch <deg>` still tilts ONLY the front pair downward (the M1/M2 behavior); the
rear/side/drum cameras keep their deterministic fixed orientations.

## Verification (self-run, this lane)

Scene `crater_boulders`, `--layers terrain,clasts,rover --cameras --size 640x480`:

- **8 distinct camera PNGs** render (8 unique md5s, all non-trivial; per-camera luma
  max ranges 120–255, so none are black frames).
- **Front stereo pair is byte-identical to pre-lane** (captured the 2-camera output BEFORE
  the change as a baseline): `front_left`/`front_right` `extrinsic_in_base_link`,
  `pose_in_world`, and `intrinsics` all compare equal, and `stereo.baseline_m` is the
  identical `0.0699996948…`. `stereo` still names the front pair.
- **`forward_look_basis()` == `look_basis(+X)`** and all 8 camera bases are proper
  rotations (det +1).
- **Rear-stereo seam** proven end-to-end in a render-context harness mirroring
  `_cameras_capture`: `rear_pair_descriptor(cams, rover_root)` → `build_sensors_json`'s
  `stereo_rear` arg yields a SEPARATE top-level `"stereo_rear"` =
  `{left:"rear_left", right:"rear_right", baseline_m: 0.0700…}` while `"stereo"` stays the
  front pair and all 8 cameras appear in `cameras[]`. (In the committed `sidecar.gd` the
  call-site still passes `null`, so `stereo_rear` is absent until the orchestrator wires
  the 1-line change above; the rig + descriptor are ready.)

Scratch render artifacts (`out/cam/**`) are NOT committed.
