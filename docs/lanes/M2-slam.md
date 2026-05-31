# Lane M2-slam — stereo-SLAM bringup (authoring lane)

Stands up the stereo-SLAM half of the Workstream-C eval against the **frozen** sensor-bridge
contract (`docs/sensor_bridge_contract.md` §2.3 / §2.5 / §5). This lane is **authored against the
frozen artifacts** — the real end-to-end run also needs the M2-egress `--cameras-seq` producer
(`out/cam/<scene>/000..NNN/`), which is a *separate* lane not merged yet, so Wave-1 validates
against a synthesized stand-in egress + the frozen `bag_writer` core.

## Owned files

| file | role |
|---|---|
| `scripts/ros2_bridge/bag_seq_writer.py` | **NEW.** Outer driver: loops `out/cam/<scene>/000..NNN/`, opens ONE rosbag2 Writer, registers the §2.3 connections ONCE, calls the frozen `bag_writer.write_frame(...)` per frame with MONOTONIC timestamps → one MCAP. |
| `scripts/ros2_bridge/slam_bringup.launch.py` | **NEW.** rtabmap stereo VO (`rtabmap_odom/stereo_odometry`) + graph-SLAM (`rtabmap_slam/rtabmap`) on the front pair → loop-closed `map`-frame pose for `/slam/odom` (contract §5). |
| `scripts/ros2_bridge/apriltag_bringup.launch.py` | **EDITED.** Un-stubbed the M2 `stereo_image_proc` disparity container + `rtabmap_odom/stereo_odometry` VO front-end (was commented ~L46–54). Gated OFF by default (`stereo:=false`) so the M1 single-tag path is byte-for-byte unchanged. |
| `docs/lanes/M2-slam.md` | this run-recipe. |

Frozen (read/import only, **never edited**): `bag_writer.py`, `frames.py`, `eval_schema.py`,
`Dockerfile`, `sidecar.gd`, `sensors_emit.gd`, the contract + manifests.

## Architecture: why `bag_seq_writer` reuses `bag_writer.write_frame`

`bag_writer` already factored the reusable core out for exactly this lane (see its
`register_connections` / `write_frame` docstrings): a single MCAP carries ONE connection set, and
`write_frame` is "REUSABLE CORE … does NOT open/close the Writer and does NOT register
connections". `bag_seq_writer` is the N>1 driver around that core:

1. `discover_frames(scene_dir)` → sorted `[(frame_index, dir), …]` for every `<NNN>/sensors.json`.
2. open ONE `rosbags.rosbag2.Writer` (MCAP, version 9).
3. `bag_writer.register_connections(...)` ONCE, from frame 000's stereo pair (the rig is rigid —
   intrinsics/baseline/extrinsics are constant across frames, contract §2.5).
4. per frame: re-read that frame's `sensors.json` (the **moving** rover `pose_in_world`), then
   `bag_writer.write_frame(...)` at `t_ns = start + i/rate_hz`, strictly monotonic.

The Godot→ROS REP-103 conversion still happens **exactly once per frame inside `write_frame`** via
`frames` — this driver adds no new conversion (contract §3 stays in one place).

> **Writer-lib note (rosbags vs rosbag2_py):** the frozen core is written against the pure-python
> `rosbags` Writer API (no rclpy needed to *write* the bag — that's the seam, not the runtime), so
> `bag_seq_writer` uses the same `rosbags` Writer to call `write_frame` verbatim. It does **not**
> open a parallel `rosbag2_py` writer (that would duplicate the connection set + the conversion
> seam). The emitted MCAP is read back by the ROS runtime (`ros2 bag info/play`) identically — the
> Wave-1 verification below confirms `ros2 bag info` reads it as 7 valid topics.

## Run recipe

All commands run **inside** `foss_ipex/ros2_bridge:jazzy` (docker-compose v1 here — `docker-compose`,
not `docker compose`). The worktree is mounted; the repo `.venv` is never used.

### 1. Synthesize a stand-in egress (until M2-egress `--cameras-seq` lands)

```bash
# from the worktree root; copies fixtures/000 → out/cam/_smoke/{000,001,002} with a bumped
# frame_index and a per-frame-shifted rover pose (the moving state).
python3 - <<'PY'
import json, os, shutil
src = "scripts/ros2_bridge/fixtures/000"
base = json.load(open(f"{src}/sensors.json"))
for i in range(3):
    d = f"out/cam/_smoke/{i:03d}"; os.makedirs(d, exist_ok=True)
    for png in ("front_left.png", "front_right.png"):
        shutil.copyfile(f"{src}/{png}", f"{d}/{png}")
    s = json.loads(json.dumps(base))
    s["schema_version"] = "sensor_bridge/1.1"; s["scene"] = "_smoke"
    s["frame_index"] = i
    s["rover"]["position_m"] = [round(0.1*i, 3), 0.0, 0.0]
    json.dump(s, open(f"{d}/sensors.json", "w"), indent=2)
PY
```

The real producer drops the identical `out/cam/<scene>/<NNN>/` layout (§2.5) — drop-in, zero
changes to `bag_seq_writer`.

### 2. Sequence → ONE MCAP (the M2-slam headline)

```bash
docker-compose -f scripts/ros2_bridge/docker-compose.yml run --rm \
  -v "$PWD/scripts/ros2_bridge":/bridge_wt -v "$PWD/out":/data/out:ro \
  bridge bash -lc '
    source /opt/ros/jazzy/setup.bash && cd /bridge_wt
    python3 bag_seq_writer.py --scene-dir /data/out/cam/_smoke \
        --out /bridge_wt/bags/_smoke_seq --rate-hz 10'
```

(Or the plain `docker run … --entrypoint bash foss_ipex/ros2_bridge:jazzy -lc '…'` form used in
verification below — same mounts.) With the real egress: `--scene-dir /data/out/cam/<scene>`.

### 3. Confirm ONE valid MCAP, monotonic, expected topics

```bash
docker run --rm -v "$PWD/scripts/ros2_bridge":/bridge_wt \
  --entrypoint bash foss_ipex/ros2_bridge:jazzy -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 bag info -s mcap /bridge_wt/bags/_smoke_seq'
```

### 4. (Wave-2) Stand up SLAM and replay

```bash
# shell A — SLAM bringup (needs the rtabmap-rebuilt image, see "Wave-2" below)
ros2 launch slam_bringup.launch.py
# shell B — replay the sequence MCAP with the bag clock
ros2 bag play -s mcap --clock /bridge_wt/bags/_smoke_seq
# the M1 detector + the M2 stereo VO can also be co-launched off the same bag:
ros2 launch apriltag_bringup.launch.py stereo:=true
```

## Wave-1 verification (DONE, in `foss_ipex/ros2_bridge:jazzy`)

- `python3 bag_seq_writer.py --help` → imports/parses; `--scene-dir … --out …` on the 3-frame
  `_smoke` egress wrote **ONE** `_smoke_seq.mcap` (single MCAP, single connection set).
- `ros2 bag info -s mcap` reads it as **21 messages over 7 topics** (3 each):
  `/front_left/image_raw`, `/front_left/camera_info`, `/front_right/image_raw`,
  `/front_right/camera_info`, `/tf`, `/tf_static`, `/lander/apriltag_truth` — the full §2.3 set.
- Read-back confirms **every topic strictly monotonic** (`0, 0.1 s, 0.2 s` @ 10 Hz) and `/tf`
  `map→base_link` translation.x = `0.0, 0.1, 0.2` per frame — i.e. the per-frame **moving** rover
  pose flowed through with the §3 REP-103 conversion applied once inside `write_frame`
  (Godot +X → ROS +X forward).
- `ros2 launch slam_bringup.launch.py --show-args` and
  `ros2 launch apriltag_bringup.launch.py --show-args` both parse and list their declared args;
  `generate_launch_description()` builds (7 and 5 entities respectively) without error.

## Wave-2 (deferred — node-level rtabmap run)

The launch files are validated **structurally** (`--show-args` + `generate_launch_description()`
build + import). The full rtabmap NODE run is Wave-2 because the rtabmap packages are **not in the
currently-built `foss_ipex/ros2_bridge:jazzy` image** (it predates the L0 Dockerfile's rtabmap apt
line). The packages DO resolve from the base image — verified:

```
ros-jazzy-rtabmap-ros   0.22.1-1noble.…
ros-jazzy-rtabmap-slam  0.22.1-1noble.…
ros-jazzy-rtabmap-odom  0.22.1-1noble.…
ros-jazzy-rtabmap-sync  0.22.1-1noble.…
```

So a `docker-compose -f scripts/ros2_bridge/docker-compose.yml build` (which uses the FROZEN L0
Dockerfile — a runtime rebuild, **not** a frozen-file edit) installs the rtabmap stack and unblocks
the §4 run above. Deferred to keep Wave-1 fast; nothing in the launch source needs to change.

## The `/slam/odom` type seam (for the orchestrator at merge)

Contract §5 pins `/slam/odom` = `nav_msgs/msg/Odometry`, `header.frame_id=='map'`,
`child_frame_id=='base_link'`, with **no aliasing/republisher node**. Verified against
introlab/rtabmap_ros `CoreWrapper`: the stock `rtabmap_slam` node connects `map`→`base_link`
through the **TF tree** (it publishes the `map→odom` *correction* TF; the VO node publishes
`odom→base_link`). Its only **typed** map-frame pose message is `localization_pose`
(`geometry_msgs/PoseWithCovarianceStamped`, frame_id=`map`) — the **single `nav_msgs/Odometry`** in
the whole pipeline is the `rtabmap_odom` VO node, which is the **drifting `odom`** frame.

`slam_bringup.launch.py` configures rtabmap honestly (`map_frame_id:=map`, `frame_id:=base_link`,
`odom_frame_id:=odom`) and remaps `localization_pose → /slam/odom` so the topic **name** already
matches. It **never mislabels the drifting `odom` VO as `map`** (which is exactly the silent-drift
bug lane C's loud `frame_id=='map'` assert exists to catch). The remaining one-line decision — the
message **type** (Odometry vs PoseWithCovarianceStamped) — is recorded in `integration_notes` for
the orchestrator: either point lane C at `localization_pose` (assert frame there) or add a tiny
typed adapter (a *type* bridge carrying `map`/`base_link` verbatim, not an aliasing node).
