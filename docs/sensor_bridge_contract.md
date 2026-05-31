# Sensor-bridge contract — Godot camera egress ↔ ROS2 fiducial/SLAM (v1.0)

*Status: FROZEN seam for the M1 "basic comms" milestone (2026-05-30). This is the dev-time analogue of
[`../INTERFACE.md`](../INTERFACE.md): the Godot renderer (producer) and the ROS2 container (consumer)
**never share memory or types** — they agree only on the on-disk artifacts and conventions defined here,
so the camera-rig, boulder, and ROS-container tracks can be built in parallel worktrees and merge
cleanly. Anything not pinned here is each track's own business.*

Three tracks consume this:
- **G1 (camera rig)** — Godot side: produces `out/cam/.../sensors.json` + the camera PNGs, and the
  AprilTag-bearing lander, per §1–§2.
- **C1 (ROS container)** — consumes `sensors.json` + PNGs, writes a rosbag2, runs AprilTag detection,
  prints pose-vs-truth, per §2–§3. Buildable against the §2.4 **fixture** before G1 exists.
- **G2 (procgen boulders)** — does NOT touch this seam; listed only so it knows it is free of it.

The scope is **M1 front-stereo**: two cameras (`front_left`, `front_right`) and ONE AprilTag face.
Rear stereo, the side monos, the drum-arm cams, and the 4-face tag bundle are M3 — §4 reserves their
identifiers so adding them later is additive (no contract break).

---

## 1. AprilTag spec (the G1↔C1 fiducial seam)

- **Family:** `tag36h11` (LAC-compatible, ArUco-compatible, ships as `tags_36h11.yaml` in `apriltag_ros`).
- **M1 tag:** a single tag, **id = 0**, on the lander's rover-facing vertical face.
- **Tag size:** `size_m = 0.150` — defined as the side length of the tag's **black border square**
  (the apriltag detector's `size` parameter; the printed marker is the 10×10-cell `tag36h11` id-0 bitmap,
  the 6×6 payload framed by a 1-cell black border, rendered edge-to-edge across `size_m`). A 1-cell white
  quiet zone is added OUTSIDE `size_m` (not counted in `size_m`).
- **Bitmap provenance:** G1 must render a marker that C1's detector decodes as `(family=tag36h11, id=0)`.
  How the bitmap is produced is G1's choice (generate from the family codebook, or bake the canonical
  AprilRobotics `apriltag-imgs/tag36h11/tag36_11_00000.png`, BSD — data, not relicensed art). The
  **integration test (C1 detects G1's rendered tag as id 0) is the acceptance check**, not the pixels.
- **Tag = lander origin (M1 simplification):** the `lander` frame origin coincides with the M1 tag's
  **center**, with the lander +X axis = the tag's outward normal (pointing toward the rover start). So
  `apriltag.pose_in_lander` is **identity** for M1. (M3: origin moves to the lander body center with
  per-face offsets; §4.)
- **Tag frame = apriltag (pnp) convention (the orientation seam).** The `lander` frame above is a
  *placement* frame (+X = outward normal toward the rover, +Y = up). The DETECTOR (`apriltag_ros`,
  christianrauch 3.x) reports a tag frame whose origin is the tag center but whose AXES follow the
  pose-estimator. We use the **`pnp`** estimator (`tags_36h11.yaml: pose_estimation_method: "pnp"` — raw
  `cv::solvePnP`, which does NOT apply the `homography` estimator's "swap x/y, invert z" fix-up). The M1
  integration pins this build's convention empirically: a near-fronto-parallel tag reads
  `q_xyzw ≈ [0.998, 0.001, 0.007, −0.062]` in the optical frame (≈ a 180° rotation about optical +X), i.e.
  **+X = image-right (optical +X), +Y = image-UP (optical −Y), +Z = OUT of the tag toward the camera**
  (the outward normal). (Note: the often-quoted "+Z into the tag" applies to the `homography` estimator,
  which we are not using.) These two frames share an origin (pose_in_lander identity) but differ by a
  **fixed rotation** independent of the camera viewpoint, so C1's `/lander/apriltag_truth` MUST relabel the
  tag *orientation* into the detector convention — identity `pose_in_lander` does NOT make the orientation
  agree. The fixed lander→tag rotation (columns = detector tag axes in lander coords) is **`tag+X =
  lander+Y`, `tag+Y = lander+Z`, `tag+Z = lander+X`** (a 120° cyclic axis-permutation,
  `frames.R_LANDER_TAG`): `tag+Z = +lander+X` is the outward normal; the in-plane (X/Y) labelling follows
  the QuadMesh's rendered texture orientation (sidecar.gd `_build_lander`) and is pinned by the
  fronto-parallel reading. C1 applies this in `bag_writer._compute_truth` by right-multiplying the tag's
  own-frame transform; the TRANSLATION (tag center == lander origin) is untouched. (M3 per-face tags each
  carry this same relabel composed with their `pose_in_lander`.)

---

## 2. `sensors.json` schema + `out/cam/` layout (the G1↔C1 data seam)

### 2.1 Directory layout (G1 writes, under `godot_sidecar/out/`, git-ignored)
```
out/cam/<scene>/<NNN>/          # NNN = zero-padded frame index; M1 ships frame 000 only
   front_left.png               # rectified-pinhole RGB (distortion OFF for M1)
   front_right.png
   sensors.json
```

### 2.2 `sensors.json` (normative; all poses in the GODOT world frame — see §3 for the conversion)
```jsonc
{
  "schema_version": "sensor_bridge/1.0",
  "scene": "crater_boulders",
  "frame_index": 0,
  "frame_convention": "godot",        // ALL poses below are Godot world (Y-up, RH, camera looks -Z).
                                       // The Godot->ROS REP-103 conversion happens ONCE, in C1's bag_writer (§3).
  "rover":  { "frame_id": "base_link",
              "position_m": [x, y, z], "quaternion_xyzw": [x, y, z, w] },
  "lander": { "frame_id": "lander",
              "position_m": [x, y, z], "quaternion_xyzw": [x, y, z, w],
              "apriltag": { "family": "tag36h11", "id": 0, "size_m": 0.150,
                            "pose_in_lander": { "position_m": [0,0,0], "quaternion_xyzw": [0,0,0,1] } } },
  "cameras": [
    { "name": "front_left",
      "frame_id": "front_left_optical",
      "image": "front_left.png",
      "width": 1280, "height": 720,
      "intrinsics": { "model": "pinhole", "fx": 0, "fy": 0, "cx": 0, "cy": 0,
                      "distortion_model": "plumb_bob", "D": [0,0,0,0,0] },
      "pose_in_world":        { "position_m": [x,y,z], "quaternion_xyzw": [x,y,z,w] },  // camera optical origin, Godot frame
      "extrinsic_in_base_link": { "position_m": [x,y,z], "quaternion_xyzw": [x,y,z,w] } // camera rel rover, Godot frame
    },
    { "name": "front_right", "frame_id": "front_right_optical", "image": "front_right.png",
      "width": 1280, "height": 720, "intrinsics": { ... }, "pose_in_world": { ... },
      "extrinsic_in_base_link": { ... } }
  ],
  "stereo": { "left": "front_left", "right": "front_right", "baseline_m": 0.100 }
}
```
Rules:
- **Intrinsics** derive from the Godot `Camera3D.fov` (horizontal): `fx = fy = (width/2) / tan(fov_x/2)`,
  `cx = width/2`, `cy = height/2`. Distortion `D = [0,0,0,0,0]` for M1 (rectified pinhole; the
  `distortion.gdshader` Brown-Conrady stub stays OFF — it becomes a non-zero `plumb_bob` D later).
- **`baseline_m`** is the metric left↔right camera separation. M1 default **0.100 m** (flagged `[CALIB]`
  until an IPEx figure is sourced); it MUST equal `|extrinsic_in_base_link(left).pos − right.pos|`.
- **Authoritative truth** = the exact `pose_in_world` of each camera and of the lander/tag. The
  camera→tag ground-truth transform (the error target) is **computed by C1** as
  `inv(T_world_cam) · T_world_lander` *after* the §3 conversion — G1 does NOT pre-compose it (one less
  convention-fragile field). G1's job is to emit exact poses; C1's job is to convert + compose + compare.
- G1 produces this with a new `--cameras` mode (mirrors the proven `--probe-multicam` SubViewport
  capture: shared `World3D`, one `Camera3D` per view, `get_texture().get_image()` per camera).

### 2.3 ROS message mapping (C1 produces, from §2.2)
| sensors.json source | ROS2 topic | type |
|---|---|---|
| `front_left.png` + intrinsics | `/front_left/image_raw` + `/front_left/camera_info` | `sensor_msgs/Image`, `CameraInfo` |
| `front_right.png` + intrinsics | `/front_right/image_raw` + `/front_right/camera_info` | same |
| `rover.pose_in_world` (converted) | `/tf` (`map`→`base_link`) | `tf2_msgs/TFMessage` |
| `cameras[].extrinsic_in_base_link` (converted) | `/tf_static` (`base_link`→`<name>`) | static TF |
| `lander.pose_in_world` (converted) | `/tf_static` (`map`→`lander`) | static TF (identity for M1) |
| computed camera→tag truth | `/lander/apriltag_truth` | `geometry_msgs/PoseStamped` (in the detecting cam's optical frame) |
- `camera_info.P` right-cam baseline term: `P[3] = -fx · baseline_m` (else stereo depth scale is silently wrong).
- rosbag2 format: **MCAP** (`rosbag2_storage_mcap`). Written **inside the container** (or via the pure-Python
  `rosbags` lib **in the container**, NOT into the repo `.venv`).

### 2.4 The fixture (unblocks C1 before G1 lands)
C1 ships `scripts/ros2_bridge/fixtures/000/` — a hand-authored `sensors.json` (this exact schema) + two
small placeholder PNGs — so the bag_writer + detector + REP-103 unit test are built and green against the
fixture. When G1's real `out/cam/.../` appears, it is a drop-in (same schema) with zero C1 changes.

---

## 3. Frames + the REP-103 conversion (named-not-solved → solved, in ONE place)

The Godot↔ROS frame trap (`INTERFACE.md` §3, spec §11) is solved EXCLUSIVELY in C1's `bag_writer.py`.
`sensors.json` is 100% Godot-native; nothing else converts.

- **Godot world:** right-handed, **+X** right, **+Y** up, **+Z** toward viewer (camera looks **−Z**).
- **ROS world (`map`, REP-103):** right-handed, **+X** forward, **+Y** left, **+Z** up.
- **Camera optical (ROS REP-103):** **+Z** forward (into scene), **+X** right, **+Y** down.

**Normative point maps (C1 implements + unit-tests both):**
1. **World Y-up → Z-up** (a −90° rotation about X):  `(x, y, z)_ros = (x_g, −z_g, y_g)`.
2. **Godot camera → ROS optical** (a 180° rotation about X):  `(x, y, z)_opt = (x_gc, −y_gc, −z_gc)`.

Orientations convert via the corresponding basis/quaternion rotations (not just positions).
**Required unit tests** (`scripts/ros2_bridge/test_frames.py`): (a) a Godot point at world `+X`
maps to ROS `+X` (forward) and Godot `+Y` (up) maps to ROS `+Z`; (b) a camera looking along Godot `−Z`
yields a ROS optical `+Z` view direction; (c) round-trip of a known pose. A silent sign flip here is the
classic cause of plausible-but-wrong SLAM — the tests are the guard.

---

## 4. Reserved for M3 (additive — do not implement now, do not collide with)
- **Cameras:** `rear_left`, `rear_right` (rear stereo), `left_mono`, `right_mono`, `drum_front_cam`,
  `drum_back_cam`. Same `cameras[]` schema; `stereo` gains a `rear` pair.
- **Tag bundle:** ids `0,1,2,3`, one per lander vertical face; `lander` origin moves to the lander body
  center and each face gets a non-identity `apriltag.pose_in_lander`. Detection uses the AprilRobotics
  bundle feature (or a small per-face-TF fusion node atop the apt `apriltag_ros`).
- **Distortion:** non-zero `plumb_bob` `D` from the `distortion.gdshader` k1/k2, cameras un-rectified.

## 5. Acceptance (M1 "basic comms established")
1. G1: `render_layers.sh -- --scene <s> --cameras …` writes `out/cam/<s>/000/{front_left,front_right}.png`
   + a schema-valid `sensors.json`; the lander + id-0 tag are visible to the front cameras.
2. C1: `bag_writer.py` turns that dir into a valid rosbag2 (MCAP); `ros2 bag play` in the container feeds
   `apriltag_ros`, which **detects id 0**; a small node computes detected-vs-truth pose error and prints it.
3. C1: `test_frames.py` passes (the §3 conversions).
The number printed in (2) is the spec §10 pose-error channel's first real reading — the Workstream-C
(two-channel eval) north-star then consumes the same `/lander/apriltag_truth` + SLAM pose.
