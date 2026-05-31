# L0 contract — spiral-departure demo: AprilTag localization vs ground truth, with observed failure modes

*Frozen 2026-05-31. The contracts-first seam for the "larger, longer" visualization demo. Lanes confine to disjoint owned files and POPULATE these signatures; they do NOT restructure the frozen sensor-bridge (`docs/sensor_bridge_contract.md` v1.1 stays frozen) or the Wave-1/Wave-2 terrain seams. New code is additive; existing M1/M2/M3 paths stay byte-behavioural.*

## 0. Binding decisions (John, 2026-05-31)
- **Optics:** keep the **idealized noiseless pinhole** (`D=[0,0,0,0,0]`, no added distortion/noise). *The failure of this ideal, distortionless system is the valuable data point* — realism comes from the **scene**, not the lens.
- **Geometry:** the AprilTag **lander is FIXED at the scene CENTER**; the rover drives a **spiral** outward from it.
- **Goal:** observe the **runtime parameters of a larger, longer simulation**. Expected, wanted failures: **out-of-range** (tag too far to resolve), **boulder occlusion**, **shadow** (tag unlit at grazing sun).
- **Anchor / data:** the real Product-78 `_slp.tif` is fetched (roughness anchor, Wave-2). Build the demo thrust **in parallel** with Wave-2 terrain.

These have hard consequences the contract enforces:
1. **Fixed-center lander is a real behaviour change.** Today `sensors_emit.build_lander` / `capture_seq.gd` re-place the lander at `rover_pos + fwd*standoff` EVERY frame (it moves WITH the rover — verified in `out/cam/tread_track_4wheel`). The demo places the lander ONCE at frame 0 and holds its `Transform3D` constant. Ground truth depends on a **constant `T_map_lander`**; getting this wrong silently invalidates every pose comparison.
2. **Multi-face tag is required for a spiral.** A single front-facing tag is only visible over a narrow arc. Use the existing 4-face bundle (`lander_bundle.gd`, ids 0–3, one per box face, each with a known `pose_in_lander`) so ≥1 face is resolvable around most of the orbit. "No face visible" is then a *geometry* outcome, distinct from the three wanted failure modes.
3. **Failure is a first-class output, not an error.** Every frame is classified; a frame with no detection is logged with its `failure_cause`, never dropped.
4. **Channel hygiene (load-bearing).** The tag pose is compared against the **sub-cell float** rover pose (`sensors.json rover{}` via `frames.godot_world_pose_to_ros`), NEVER the ~20 mm-quantized `rover_rc` trajectory channel. `eval_schema.py:9-31` forbids summing the two channels.

## 1. Spiral trajectory — pure python (host)  — Lane DEMO-TRAJ
`scripts/demo/spiral_path.py`
```python
def spiral_rc(center_rc, n_frames, *, turns, r0_cells, r_growth_cells, cell_m) -> list[tuple[float,float]]:
    """Archimedean spiral rover_rc waypoints about center_rc (the lander cell):
    r(θ)=r0+r_growth*θ/2π, θ in [0, 2π*turns]. Monotonically increasing range
    so the rover progressively departs (drives the out-of-range failure)."""
def look_at_yaw(rover_rc, center_rc) -> float:
    """Heading that points the rover's +forward (front stereo) at the lander center,
    in the SAME yaw convention as scenes._heading_yaw / sidecar._heading_yaw
    (yaw = atan2(-dz, dx)); so the tag stays in the stereo frustum each step."""
```
Pure stdlib+math; host-testable (no engine). Output is authored into the demo scene's per-frame `rover_rc` + heading so the Godot sequence renders it.

## 2. Fixed-center lander + spiral egress (Godot)  — Lane DEMO-GODOT
`godot_sidecar/depart_spiral.gd` (new `--depart-spiral` entry; reuses, does not fork, the frozen rig)
- Place the **4-face lander bundle ONCE** at the scene-center cell at frame 0 (`lander_bundle.build_lander_faces`), surface-snapped; **hold its `Transform3D` constant** for all N frames.
- Per frame: place the rover at `rover_rc[i]` with `look_at_yaw` (front stereo faces the lander); render `front_left.png`/`front_right.png` + `sensors.json` to `out/cam/<scene>/<NNN>/` via the existing `_cameras_capture` egress + `sensors_emit.build_sensors_json` (emit `lander.apriltags[]` for the 4 faces + the constant lander pose + per-frame rover float pose).
- Reuses: `camera_rig.gd` (front stereo, `BASELINE_M=0.070`, `FOV_X_DEG=73.99`), `lander_bundle.gd`, `apriltag_gen.gd`, `render.sh` (xvfb+Vulkan). **No change** to `camera_rig`/`sensors_emit` schema beyond using the already-present `apriltags[]`/`stereo` fields.
- Honesty: `distortion_model='plumb_bob'`, `D=[0,0,0,0,0]` stays (idealized pinhole, §0).

## 3. Inverse localization, multi-id (container)  — Lane DEMO-LOCALIZE
`scripts/ros2_bridge/rover_localize.py` (container-only to RUN; pure-numpy transform math host-testable with synthetic inputs)
```python
def rover_pose_from_tag(T_optical_tag, *, base_link_T_optical, T_map_lander, lander_T_tag) -> tuple[pos, quat]:
    """Back the rover's map pose out of ONE detected face:
    T_map_baselink = T_map_lander @ lander_T_tag @ inv(T_optical_tag) @ inv(base_link_T_optical).
    base_link_T_optical comes from sensors.json cameras[left].extrinsic_in_base_link;
    lander_T_tag from lander.apriltags[id].pose_in_lander (+ frames.R_LANDER_TAG relabel)."""
def fuse_faces(per_face_poses) -> tuple[pos, quat, dict]:
    """Average/￼select across the faces detected this frame; report agreement spread."""
```
Detection itself reuses `fiducial_overlay.py` (`apriltag.detect` + `cv2.solvePnP IPPE_SQUARE`) or `apriltag_ros`; the **multi-id loop and the rover back-out are net-new** (`fiducial_overlay` today reports only camera→tag for id 0). `frames.py` supplies all transform helpers (`make_transform`, `transform_to_pos_quat`); unit-test the composition against `bag_writer._compute_truth`'s pure-numpy truth.

## 4. Per-step telemetry + failure attribution + report (host)  — Lane DEMO-REPORT
`scripts/demo/demo_spiral.py` (report-only, no invented pass/fail — mirrors `eval_harness.py`)
Per-frame record (the "runtime parameters" to observe):
```jsonc
{ "frame": i, "t_s": …, "range_m": …, "sun_az_deg": …, "sun_el_deg": …,
  "faces_detected": [ids], "n_faces": …,
  "rover_truth_map": {pos,quat},            // channel A: sensors.json rover{} via frames.py (sub-cell)
  "rover_est_map":   {pos,quat}|null,       // from rover_localize.fuse_faces
  "trans_err_mm": …|null, "rot_err_deg": …|null,   // score_pose.score_apriltag
  "face_illum": {id: "lit"|"shadow"},       // illumination.horizon_clip at the face cell (Wave-2)
  "occluded": bool,                         // tag in frustum but no detection w/ a clast between
  "resident_fine_tiles": …,                 // TileMosaic resident count (pipeline-visibility)
  "failure_cause": "none"|"out_of_range"|"occluded"|"shadowed"|"no_face_visible" }
```
Outputs: the per-frame JSON stream + a summary + **matplotlib visualizations** — (a) `trans_err_mm` & `rot_err_deg` vs `range_m`; (b) detection success/failure coloured along the spiral (x,y); (c) failure-cause breakdown. Reuses `score_pose.score_apriltag`, `compare_pose.rotation_error_deg`, `frames.godot_world_pose_to_ros`, `eval_schema.lift_trajectory` (trajectory channel, ATE — reported SEPARATELY, never summed). Host-runnable (numpy+matplotlib).

## 5. Hero run = controlled integration step (NOT an autonomous agent)
Code lanes write + host-test; the live end-to-end run (Godot render via `render.sh` → container detect via `docker run foss_ipex/ros2_bridge:jazzy` → host `demo_spiral.py`) is run under supervision (the images are already built locally; cv2/apriltag/rclpy are container-only, no-pip). The richest scene is the Wave-2 `scenes.build_from_dem` Haworth output (craters + boulders + illumination); a bouldered patch scene is the v1 fallback if Wave-2 has not merged.

## 6. Honesty rails (portfolio discipline)
- The sub-cm translation error is the **geometric/subpixel floor of a noiseless synthetic pinhole**, not distortion-and-noise-inclusive accuracy — state it plainly in the report header.
- The ~7° rotation residual is the **PnP near-fronto-parallel ambiguity** (IPPE_SQUARE degeneracy), expected to **persist/worsen** as the rover departs along a line toward/away from a face; it is not a frame bug.
- Passive-stereo depth is **sparse/black on low-texture regolith**; narrate it, use `--cam-pitch` down + the bouldered/cratered scene to give texture. Depth rides ROS as the live `/disparity` (`stereo_image_proc`) during bag playback + offline colorized PNGs (`depth_map.py`); no faked depth in the committed bag.
