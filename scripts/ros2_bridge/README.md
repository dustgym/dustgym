# foss_ipex Track C1 — ROS2 bridge (front-stereo → AprilTag → pose-vs-truth)

The ROS2-container side of the M1 *"basic comms established"* milestone
([`../../docs/sensor_bridge_contract.md`](../../docs/sensor_bridge_contract.md)). It consumes
the Godot camera egress (`out/cam/<scene>/<NNN>/` — `sensors.json` + two PNGs), writes a
rosbag2 in **MCAP** format, runs **AprilTag** detection on the front-left image, and prints the
detected-vs-truth camera→tag pose error (the spec §10 pose-error channel's first reading).

**This track builds and proves itself WITHOUT the Godot track**, using the committed
[`fixtures/000/`](fixtures/000/) (the contract §2.4 stand-in). When G1's real `out/cam/…/`
appears it is a drop-in — same schema, zero C1 changes.

> All paths below are relative to this directory (`scripts/ros2_bridge/`).

---

## The seam: who converts what

`sensors.json` is **100% Godot-native** (Y-up, RH, camera looks −Z). The
Godot→ROS REP-103 conversion (Z-up; optical +Z-forward) happens **exactly once**, in
[`bag_writer.py`](bag_writer.py), via [`frames.py`](frames.py) — contract §3. Nothing else in
the system converts. The two normative point maps and the three required unit tests live in
[`test_frames.py`](test_frames.py).

| `sensors.json` source | ROS2 topic | type |
|---|---|---|
| `front_{left,right}.png` + intrinsics | `/front_{left,right}/image_raw` + `/camera_info` | `sensor_msgs/Image`, `CameraInfo` |
| `rover.pose_in_world` (converted) | `/tf` `map`→`base_link` | `tf2_msgs/TFMessage` |
| `cameras[].extrinsic_in_base_link` (converted) | `/tf_static` `base_link`→`*_optical` | static TF |
| `lander.pose_in_world` (converted) | `/tf_static` `map`→`lander` | static TF |
| computed `inv(T_map_optical)·T_map_tag` | `/lander/apriltag_truth` | `geometry_msgs/PoseStamped` |

The right camera's `CameraInfo.P[3] = -fx · baseline_m` (the stereo baseline term); the left's
is 0.

---

## Host ↔ container handshake

Everything runs **inside the container** (`osrf/ros:jazzy-perception` + the packages below).
The repo's `.venv` is never used or modified — all Python deps (incl. the pure-python
`rosbags` writer) live in the image.

```
host                                   container (/bridge)
─────────────────────────────────────  ──────────────────────────────────────────
godot_sidecar/out/        ── ro ──▶     /data/out/         (G1 egress; out/cam/...)
scripts/ros2_bridge/bags/ ── rw ──▶     /bridge/bags/      (written MCAPs; gitignored)
fixtures/000/  (baked into image)       /bridge/fixtures/000/   (the M1 stand-in)
```

`docker-compose.yml` wires those mounts. `../../godot_sidecar/out` may not contain `cam/` yet
(G1 not landed) — that's fine; M1 acceptance uses the in-image fixtures.

---

## Build

```bash
docker compose build           # or: docker build -t foss_ipex/ros2_bridge:jazzy .
```

---

## M1 run recipe

### 0. (host) frame unit tests — no container needed if you have numpy
```bash
python3 test_frames.py         # 4 tests; the 3 contract-§3-required + 1 optical sanity
```
or in the container: `docker compose run --rm bridge python3 test_frames.py`.

### 1. Write a rosbag2 (MCAP) from the fixture
```bash
docker compose run --rm bridge python3 bag_writer.py --in fixtures/000 --out bags/fixture_000
docker compose run --rm bridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 bag info -s mcap bags/fixture_000'
```
With the real Godot egress instead: `--in /data/out/cam/<scene>/000`.

### 2. Detect the tag + print pose error (three processes; one shell each)
```bash
# (a) detector
docker compose run --rm bridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 launch apriltag_bringup.launch.py'

# (b) comparator
docker compose run --rm bridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && python3 compare_pose.py'

# (c) replay the bag in a loop so the detector/comparator have time to latch
docker compose run --rm bridge bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 bag play -s mcap --loop bags/fixture_000'
```
The detector publishes `/tf` child `tag36h11:0`; `compare_pose.py` joins it with
`/lander/apriltag_truth` and prints **translation error (mm)** and **rotation error (deg)**.

> **Interpreting the FIXTURE error (expected to be large):** against `fixtures/000/` the printed
> error is dominated by fixture artifacts, not by the bridge. The hand-pasted tag is drawn at a
> fixed pixel size and fronto-parallel, so the detector's PnP depth (`≈ fx·size_m / tag_px`) and
> orientation reflect *the placeholder pixels*, while `/lander/apriltag_truth` reflects the
> fixture's *declared* 3 m range and −90°-about-Y lander face. The two only converge once G1's
> real render projects a true-to-scale, true-to-pose tag — which is exactly the contract §1
> acceptance ("the integration test is the acceptance check, not the pixels"). The fixture's job
> here is to prove the **wiring**: detection of id 0 + truth computation + comparison all run.
> A first sample run printed `translation 2286 mm / rotation 120 deg` — a healthy non-zero
> reading on the spec §10 channel, expected to drop to cm/deg on the Godot egress.

### Quick in-process detection proof (no bag loop)
To prove the detector decodes the fixture tag without standing up the full play→detect→compare
loop, run the apriltag library directly on the fixture PNG inside the container:
```bash
docker compose run --rm bridge python3 - <<'PY'
import cv2, numpy as np
from apriltag import apriltag        # apt: python3-apriltag (pulled by apriltag_ros)
img = cv2.imread('fixtures/000/front_left.png', cv2.IMREAD_GRAYSCALE)
print([d['id'] for d in apriltag("tag36h11").detect(img)])
PY
```
(If the standalone `apriltag` python binding isn't present, use the launch loop above; the
detector node is the authoritative acceptance path.)

---

## Files

| file | purpose |
|---|---|
| `Dockerfile` | `osrf/ros:jazzy-perception` + apriltag_ros + stereo_image_proc + rosbag2-mcap + `rosbags` |
| `docker-compose.yml` | mounts `../../godot_sidecar/out` (ro) + `bags/` (rw); runs the bridge |
| `frames.py` | the §3 REP-103 maps (world Y-up→Z-up; cam→optical), positions + orientations |
| `test_frames.py` | the 3 required §3 unit tests (+1 optical sanity); pure-python/numpy |
| `bag_writer.py` | `out/cam/.../NNN/` → rosbag2 MCAP, applying the §3 conversion once |
| `tags_36h11.yaml` | detector config: family `36h11`, id 0, size 0.150 m |
| `apriltag_bringup.launch.py` | launches `apriltag_node` on `/front_left` (M2 stereo/SLAM stubbed) |
| `compare_pose.py` | joins detected `/tf` tag pose with `/lander/apriltag_truth`, prints error |
| `fixtures/000/` | hand-authored `sensors.json` + tag-bearing `front_left.png` + grey `front_right.png` |
| `fixtures/make_fixture_images.py` | regenerates the fixture PNGs (pure stdlib) |
| `fixtures/_assets/tag36_11_00000.png` | canonical AprilRobotics tag36h11 id-0 bitmap (BSD; see refs) |

---

## Confirmed package / image names (web-verified 2026-05)

- Base image **`osrf/ros:jazzy-desktop`** (ROS2 Jazzy / Ubuntu 24.04 Noble). *Fallback from the
  contract's preferred `osrf/ros:jazzy-perception`*: that tag's manifest did not resolve from
  this environment (osrf/ros ships only `jazzy-desktop` / `-desktop-full` / `-simulation` for
  Jazzy here, and the `library/ros` jazzy tags were unreachable). `jazzy-desktop` is a clean
  superset of `-perception` — bundles `image_pipeline`/`stereo_image_proc` + rclpy + the rosbag2
  stack. **No humble/Jammy fallback was needed**; all packages below are present on Jazzy.
- **`ros-jazzy-apriltag-ros`** — christianrauch/apriltag_ros 3.x; node `apriltag_node`,
  subscribes `image_rect` + `camera_info`, param `family: 36h11`, publishes `/tf`
  (`tag36h11:0`) + `/detections` (`apriltag_msgs/AprilTagDetectionArray`). Note apriltag_ros
  spells the family **`36h11`** (no `tag` prefix); same family as the contract's "tag36h11".
- **`ros-jazzy-stereo-image-proc`**, **`ros-jazzy-rosbag2-storage-mcap`**,
  **`ros-jazzy-rosbag2-transport`** — all available on Jazzy.
- **`rosbags`** (PyPI, ≥0.10) — pure-python rosbag2 reader/writer; **no ROS install needed to
  write**. `Writer(path, version=9, storage_plugin=StoragePlugin.MCAP)`,
  `get_typestore(Stores.ROS2_JAZZY)`. Installed into a `--system-site-packages` venv in the
  image (PEP 668), never the repo `.venv`.

No jazzy→humble/Jammy *package* fallbacks were needed — every required package is present on
Jazzy. The only deviation is the base-image tag (`-desktop` instead of the unreachable
`-perception`), noted above.

---

## Added third-party references (local note)

> Kept here, not in the shared `papers/CITATIONS.md`, to avoid a cross-track merge collision.
> This track's original code is **CC0-1.0** (repo `LICENSE`); the items below are external
> dependencies / data with their own licenses.

- **apriltag_ros** — C. Rauch, *ROS2 node for AprilTag detection*,
  <https://github.com/christianrauch/apriltag_ros> (BSD-2-Clause). Wraps the AprilRobotics
  AprilTag 3 library (Wang & Olson, *"AprilTag 2: Efficient and robust fiducial detection,"*
  IROS 2016; Olson, *"AprilTag: A robust and flexible visual fiducial system,"* ICRA 2011).
- **AprilRobotics `apriltag-imgs`** — canonical tag36h11 bitmaps,
  <https://github.com/AprilRobotics/apriltag-imgs> (BSD). `fixtures/_assets/tag36_11_00000.png`
  is the unmodified id-0 codebook bitmap (data, not relicensed art); the contract §1 acceptance
  is "the detector decodes it as id 0", not the pixels.
- **rosbags** — Ternaris, pure-python rosbag2 read/write,
  <https://gitlab.com/ternaris/rosbags> (Apache-2.0).
- **rtabmap / rtabmap_ros** (M2, not used in M1) — M. Labbé & F. Michaud,
  *"RTAB-Map as an open-source lidar and visual SLAM library…,"* J. Field Robotics 2019
  (BSD-3). Reserved for the §4 stereo-SLAM milestone; stubbed in the launch file.
- **ROS REP-103** — *Standard Units of Measure and Coordinate Conventions*,
  <https://www.ros.org/reps/rep-0103.html> — the frame convention `frames.py` implements.
