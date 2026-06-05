---
title: "Sensor-bridge contract"
nav_order: 8
---

# Sensor-bridge contract — Godot camera egress ↔ ROS2 fiducial/SLAM (v1.1)

*Status: FROZEN seam for the M1 "basic comms" milestone (2026-05-30). This is the dev-time analogue of
[`../INTERFACE.md`](../INTERFACE.md): the Godot renderer (producer) and the ROS2 container (consumer)
**never share memory or types** — they agree only on the on-disk artifacts and conventions defined here,
so the camera-rig, boulder, and ROS-container tracks can be built in parallel worktrees and merge
cleanly. Anything not pinned here is each track's own business.*

### v1.1 additions (ADDITIVE, unknown-key-tolerant — v1.0 consumers MUST still work)

`schema_version` is bumped `sensor_bridge/1.0` → **`sensor_bridge/1.1`**. v1.1 is a strict superset of v1.0:
every field a v1.0 reader looks up **by name** is still present at the same path with the same meaning, and
every v1.1 addition is either an OPTIONAL new top-level/sub key (a reader that does not know it ignores it)
or a value-domain widening (`frame_index` 0 → real monotonic int). A conforming reader **MUST tolerate
unknown keys** (do not fail on a key you do not recognise). The seven additions, each detailed in the
section noted, are:

1. **Optional top-level `"sun"`** block (§2.2). Absent ⇒ the M1 default grazing sun. Now actually emitted
   by the `--cameras` path from the existing sidecar `_sun_elev_deg` / `_sun_azim_deg` members.
2. **`"stereo"` ALWAYS carries the FRONT pair** exactly as v1.0; rear stereo is a *separate* optional
   top-level `"stereo_rear"` (§2.2, §2.3) — never a replacement for `"stereo"`.
3. **`lander.apriltags[]`** (ids 0..3) added alongside the v1.0 single `lander.apriltag` (§1, §2.2). If
   `apriltags[]` is present it SUPERSEDES `apriltag{}`; front face id 0 keeps the existing identity
   `pose_in_lander` so M1 stays correct.
4. **`frame_index` becomes a real monotonic int** in multi-frame egress (§2.2, §2.5) — it is no longer
   hardcoded `0` once a producer emits a sequence.
5. **SLAM pose channel** — canonical TF `map`→`base_link` (loop-closed), optional `/slam/odom` relay —
   registered for the Workstream-C scorer (§2.3).
6. **Per-face AprilTag relabel** `R_face` (§1) — the front face (id 0) reduces exactly to the existing
   `frames.R_LANDER_TAG`, so the M1 reading is unchanged.
7. **Multi-frame egress directory convention** frozen as a contract artifact (§2.5).

Items 8 (`frame_convention='godot'`) and the §3 REP-103 maps are explicitly UNCHANGED.

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

### 1.1 Per-face relabel `R_face` (v1.1 — generalises `R_LANDER_TAG` to the 4-face bundle)

The single `R_LANDER_TAG` above is **specific to the front face**: it is derived from the front face's
outward normal (lander +X) composed with the +90° QuadMesh yaw that orients the printed bitmap
(`frames.py:69–87`, `sidecar.gd _build_lander`). When the lander grows a 4-face tag bundle (ids 0..3, one
per vertical face — §2.2 `lander.apriltags[]`), each face has its OWN normal and therefore its OWN
lander→tag rotation. v1.1 freezes the convention so **M3-tag (Godot, defines the face quad orientations)
and M3-bundle (ROS, `_compute_truth`) agree** without sharing code:

- For each face *f*, `R_face` is the rotation **from lander axes into THAT face's tag-quad axes**, derived
  from the face's own `pose_in_lander` basis — NOT the single front-face `R_LANDER_TAG`. Concretely,
  `R_face` is built from the same three detector-tag axis definitions as §1, re-expressed against face *f*:
  - **tag +Z** = the face's OUTWARD normal (the column of the face's `pose_in_lander` rotation that points
    away from the lander body — the rover-facing direction for that face),
  - **tag +Y** = the face quad's rendered "up" (the QuadMesh's +90° texture-yaw axis for that face — its
    rendered bitmap "up" in lander coords),
  - **tag +X** = right-handed completion (`tag+X = tag+Y × tag+Z`), so `R_face` is a proper rotation
    (det = +1), exactly as `R_LANDER_TAG` is.
  The columns of `R_face` are these three detector-tag axes expressed in lander coordinates, i.e. `R_face`
  is consumed identically to `R_LANDER_TAG`: C1 right-multiplies the tag's own-frame transform by `R_face`
  in `_compute_truth`; the TRANSLATION (each face's `pose_in_lander.position_m`) is untouched.
- **CONSTRAINT (M1 invariance):** for the **front face (id 0)**, `R_face` MUST reduce *exactly* to the
  existing `frames.R_LANDER_TAG` (`tag+X = lander+Y`, `tag+Y = lander+Z`, `tag+Z = lander+X`). Because id 0
  keeps its identity `pose_in_lander` (§1) and its rendered-up runs along lander +Z with outward normal
  lander +X, the construction above yields precisely `R_LANDER_TAG` — so the M1 reading
  (**12.7 mm / 7.15°**) is unchanged. M3-bundle SHOULD assert `R_face(id=0) == R_LANDER_TAG` as a guard.
- The per-face `pose_in_lander` (origin offset + basis) is authored by Godot (M3-tag) in `lander.apriltags[]`
  (§2.2); C1 derives `R_face` from that basis. No new on-disk field carries `R_face` itself — it is a
  *derived* relabel, kept out of the JSON for the same "one less convention-fragile field" reason as the
  camera→tag truth (§2.2).

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
  "schema_version": "sensor_bridge/1.1",          // v1.1: was "sensor_bridge/1.0". Additive; see header notice.
  "scene": "crater_boulders",
  "frame_index": 0,                                // v1.0: always 0. v1.1: real monotonic int in multi-frame egress (§2.5).
  "frame_convention": "godot",        // ALL poses below are Godot world (Y-up, RH, camera looks -Z). UNCHANGED in v1.1.
                                       // The Godot->ROS REP-103 conversion happens ONCE, in C1's bag_writer (§3).

  // --- v1.1 OPTIONAL top-level "sun" (absent => M1 default grazing sun). Now actually emitted by --cameras. ---
  "sun": { "elevation_deg": 5.0, "azimuth_deg": 215.0, "time_delta_s": 0.0 },

  "rover":  { "frame_id": "base_link",
              "position_m": [x, y, z], "quaternion_xyzw": [x, y, z, w] },
  "lander": { "frame_id": "lander",
              "position_m": [x, y, z], "quaternion_xyzw": [x, y, z, w],
              // v1.0 single tag — STAYS for back-compat (M1 reads this by name):
              "apriltag": { "family": "tag36h11", "id": 0, "size_m": 0.150,
                            "pose_in_lander": { "position_m": [0,0,0], "quaternion_xyzw": [0,0,0,1] } },
              // v1.1 OPTIONAL 4-face bundle. If present, SUPERSEDES "apriltag" above (see rule below).
              "apriltags": [
                { "family": "tag36h11", "id": 0, "size_m": 0.150,
                  "pose_in_lander": { "position_m": [0,0,0], "quaternion_xyzw": [0,0,0,1] } }  // id 0 == front, identity (M1-invariant)
                // , { "family":"tag36h11", "id":1, "size_m":0.150, "pose_in_lander": {...} }   // ids 1..3 = other vertical faces
              ] },
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
    // v1.1: rear/side/drum cameras appear here too (same schema) when their lane emits them (§4).
  ],
  // "stereo" ALWAYS the FRONT pair, exactly as v1.0 (write_bag reads ['left']/['right'] BY NAME — never breaks):
  "stereo": { "left": "front_left", "right": "front_right", "baseline_m": 0.100 }
  // v1.1 OPTIONAL, SEPARATE rear pair (never replaces "stereo"); present only when a rear pair is emitted:
  // "stereo_rear": { "left": "rear_left", "right": "rear_right", "baseline_m": 0.100 }
}
```
Rules:
- **`sun`** (v1.1, optional top-level): `{ "elevation_deg": float, "azimuth_deg": float, "time_delta_s": float }`,
  all in the Godot/scene sun model (degrees). It is emitted from the sidecar's existing `_sun_elev_deg` /
  `_sun_azim_deg` members (defaults **5.0 / 215.0** — the M1 grazing sun). `time_delta_s` is the elapsed
  lunar-day time the sun pose corresponds to and **defaults to `0`**. **Absent ⇒ a v1.0 producer / the M1
  default**: a consumer that does not find `sun` MUST assume the M1 grazing default (elev 5°, azim 215°,
  Δt 0). The detailed `time_delta_s → (azimuth, elevation)` lunar-day model lives in
  [`sun_sweep_manifest.md`](sun_sweep_manifest.md); this block carries only the resolved instantaneous pose.
- **Intrinsics** derive from the Godot `Camera3D.fov` (horizontal): `fx = fy = (width/2) / tan(fov_x/2)`,
  `cx = width/2`, `cy = height/2`. Distortion `D = [0,0,0,0,0]` for M1 (rectified pinhole; the
  `distortion.gdshader` Brown-Conrady stub stays OFF — it becomes a non-zero `plumb_bob` D later).
- **`baseline_m`** is the metric left↔right camera separation. M1 default **0.100 m** (flagged `[CALIB]`
  until an IPEx figure is sourced); it MUST equal `|extrinsic_in_base_link(left).pos − right.pos|`.
- **`stereo` is ALWAYS the FRONT pair** (`{left,right,baseline_m}`), exactly as v1.0, on EVERY emission. The
  frozen `bag_writer.write_bag` resolves the stereo cameras via `sensors['stereo']['left']` / `['right']`
  **by name**, so this key and those two sub-keys MUST never disappear or be repurposed. (v1.1)
- **`stereo_rear`** (v1.1, optional top-level): a SEPARATE `{left,right,baseline_m}` for the rear stereo
  pair, with the same shape as `stereo`. It is **NEVER a replacement for `stereo`** — both coexist when a
  rear pair is emitted; `stereo` keeps naming the front cameras. Absent ⇒ no rear stereo (the M1 case). A
  v1.0 reader ignores it.
- **`lander.apriltags[]` supersede rule** (v1.1): the v1.0 single `lander.apriltag` object STAYS for
  back-compat. The optional `lander.apriltags[]` array carries the 4-face bundle, each entry
  `{family, id, size_m, pose_in_lander}` with `id ∈ {0,1,2,3}`. **If `apriltags[]` is present it SUPERSEDES
  `apriltag{}`** — a v1.1 consumer that understands the bundle MUST read `apriltags[]` and ignore the
  singular `apriltag`. To keep M1 correct, the **front face (id 0)** in `apriltags[]` MUST carry the SAME
  identity `pose_in_lander` as the singular `apriltag` (so the §1.1 `R_face(id=0)` reduces to
  `R_LANDER_TAG` and the 12.7 mm / 7.15° reading is unchanged). A v1.0 reader (which does not know
  `apriltags`) keeps using the singular `apriltag` and stays correct for the front face.
- **`frame_index`** (v1.1 value-domain widening): a single-frame emission keeps `frame_index = 0` (M1).
  In the multi-frame egress (§2.5) it is a **real monotonic integer** matching the `<NNN>` sub-directory
  (000, 001, …). The field name/type are unchanged from v1.0; only its value range widens.
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
| `stereo_rear.left/right` + intrinsics *(v1.1, opt.)* | `/rear_left/image_raw`+`/rear_left/camera_info`, `/rear_right/…` | `sensor_msgs/Image`, `CameraInfo` (only when `stereo_rear` present) |
| rtabmap SLAM pose *(v1.1, see below)* | **TF `map`→`base_link`** (canonical) · `/slam/odom` (optional relay) | loop-closed pose via the TF tree; optional relay republishes it as `nav_msgs/msg/Odometry` (`header.frame_id == "map"`, `child_frame_id == "base_link"`) |
- `camera_info.P` right-cam baseline term: `P[3] = -fx · baseline_m` (else stereo depth scale is silently wrong).
  For the optional rear pair this uses `stereo_rear.baseline_m`.
- rosbag2 format: **MCAP** (`rosbag2_storage_mcap`). Written **inside the container** (or via the pure-Python
  `rosbags` lib **in the container**, NOT into the repo `.venv`).

**SLAM pose channel** (v1.1 — the Workstream-C scorer seam, lane C): the scorer (lane C) consumes the
loop-closed pose **rtabmap publishes as the TF transform `map`→`base_link`** (NOT the drifting `odom`
frame). Stock `rtabmap_slam` does NOT advertise a renamable `nav_msgs/Odometry` in the `map` frame — it
exposes the globally-consistent pose through the **TF tree**: it publishes the `map`→`odom` loop-closure
correction, and stereo VO (`rtabmap_odom`) publishes `odom`→`base_link`, which compose to `map`→`base_link`.
The drifting VO estimate stays on `/odom`. This channel is therefore PINNED as:
- **CANONICAL: TF `map`→`base_link`** — lane C samples this transform at each stereo-frame stamp as the
  SLAM estimate, and **ASSERTS the parent frame is `map`**, failing LOUD on any mismatch (an `odom`-parented
  sample is a hard error, not a silent drift source).
- **OPTIONAL relay `/slam/odom`** (`nav_msgs/msg/Odometry`, `header.frame_id == "map"`, `child == "base_link"`):
  a thin node MAY republish the sampled TF as a topic for convenience/recording; it is OPTIONAL — C consumes
  the TF directly when the relay is absent. (Refined v1.1: the original pin assumed rtabmap emitted the topic
  natively; the M2-slam lane confirmed it emits via TF, so TF is canonical and the topic is an optional relay.)
This is the SLAM half of the Workstream-C two-channel eval (the AprilTag half is `/lander/apriltag_truth`
above); see [`../scripts/ros2_bridge/eval_schema.py`](../scripts/ros2_bridge/eval_schema.py).

### 2.4 The fixture (unblocks C1 before G1 lands)
C1 ships `scripts/ros2_bridge/fixtures/000/` — a hand-authored `sensors.json` (this exact schema) + two
small placeholder PNGs — so the bag_writer + detector + REP-103 unit test are built and green against the
fixture. When G1's real `out/cam/.../` appears, it is a drop-in (same schema) with zero C1 changes.

### 2.5 Multi-frame egress directory convention (v1.1 — frozen so M2-slam can author against it)

This is the multi-frame generalisation of §2.1, frozen now as a contract artifact so the M2-slam lane can
build its sequence reader/bag-loop **before M2-egress exists**. The single-frame §2.1 layout is the N = 1
special case of this.

```
out/cam/<scene>/<NNN>/          # NNN = zero-padded 3-digit frame index, from 000 (000, 001, 002, …)
   front_left.png               # rectified-pinhole RGB; same camera set/naming as §2.1/§2.2
   front_right.png
   ...                          # any other emitted cameras (rear/side/drum), same names as cameras[]
   sensors.json                 # ONE per-frame sensors.json (v1.1) per <NNN> directory
```

Rules:
- `<NNN>` is **zero-padded to 3 digits** and starts at **`000`**, monotonically increasing by 1 per frame.
- Each `<NNN>/sensors.json` is a full v1.1 document carrying:
  - the **real monotonic `frame_index`** for that frame (equal to the integer value of `<NNN>`), and
  - the **per-frame rover `pose_in_world`** (and per-frame camera `pose_in_world`), i.e. the moving state.
- **Constant across frames** (do NOT re-derive per frame): camera **intrinsics**, `baseline_m` (and
  `stereo_rear.baseline_m`), and `extrinsic_in_base_link` (the rig is rigid). A consumer MAY read these
  once from frame 000.
- The producing flag is **`--cameras-seq`** (M2-egress lane). It MUST set `_drums_up = true`, inheriting the
  live `--cameras` side effect (`sidecar.gd` ~437–439) so the drum arms clear the front-stereo FOV exactly
  as the single-frame path does. (The single-frame `--cameras` path and §2.1 layout are UNCHANGED.)

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
  `drum_back_cam`. Same `cameras[]` schema. The rear pair is published through the **separate v1.1
  `stereo_rear`** top-level key (§2.2, §2.3) — NOT a `rear` sub-key of `stereo` (which always stays the
  front pair). Side monos / drum cams appear in `cameras[]` only.
- **Tag bundle:** ids `0,1,2,3`, one per lander vertical face, carried in the v1.1 `lander.apriltags[]`
  (§2.2). `lander` origin moves to the lander body center and each face gets a non-identity
  `pose_in_lander`; the per-face orientation relabel is the v1.1 `R_face` convention (§1.1, front face
  id 0 reduces to `R_LANDER_TAG`). Detection uses the AprilRobotics bundle feature (or a small per-face-TF
  fusion node atop the apt `apriltag_ros`).
- **Distortion:** non-zero `plumb_bob` `D` from the `distortion.gdshader` k1/k2, cameras un-rectified.

## 5. Acceptance (M1 "basic comms established")
1. G1: `render_layers.sh -- --scene <s> --cameras …` writes `out/cam/<s>/000/{front_left,front_right}.png`
   + a schema-valid `sensors.json`; the lander + id-0 tag are visible to the front cameras.
2. C1: `bag_writer.py` turns that dir into a valid rosbag2 (MCAP); `ros2 bag play` in the container feeds
   `apriltag_ros`, which **detects id 0**; a small node computes detected-vs-truth pose error and prints it.
3. C1: `test_frames.py` passes (the §3 conversions).
The number printed in (2) is the spec §10 pose-error channel's first real reading — the Workstream-C
(two-channel eval) north-star then consumes the same `/lander/apriltag_truth` + SLAM pose.
