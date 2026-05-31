# Lane M2-egress — multi-frame front-stereo camera egress (`--cameras-seq`)

Owner files: `godot_sidecar/capture_seq.gd`, `docs/lanes/M2-egress.md`.
Branched off L0 `e2b0994`. Implements the FROZEN sensor-bridge contract v1.1 **§2.5
multi-frame egress** — the MOVING front-stereo sequence rtabmap / COLMAP consume.

## What it does

`capture_seq.gd::run_capture_seq(sidecar)` is the `--cameras-seq` driver. It is the
multi-frame generalisation of the single-frame `sidecar.gd::_cameras_capture`
(`--cameras`): same camera rig, same lander, same schema sink, but iterated over a
moving rover trajectory, emitting one `out/cam/<scene>/<NNN>/` directory per frame.

Per frame it:
1. places the articulated rover at that frame's `rover_rc` + path-heading yaw (drives
   `sidecar._rover_rc_override` / `_rover_yaw`, then `_clear_frame_nodes()` +
   `_build_layers()` — the exact per-frame rebuild `sidecar._run_sequence` uses);
2. builds the AprilTag-bearing procedural lander ahead of *that* rover pose via the
   FROZEN `sensors_emit.gd::build_lander(...)`;
3. builds the front-stereo rig (`front_left` / `front_right`) via the FROZEN
   `camera_rig.gd::build(...)` (shared-`World3D` SubViewports, riding the rover);
4. settles + renders (`await RenderingServer.frame_post_draw` ×3, the proven
   `_cameras_capture` capture pattern) and writes
   `out/cam/<scene>/<NNN>/{front_left,front_right}.png`;
5. assembles + writes the per-frame `sensors.json` via the FROZEN
   `sensors_emit.gd::build_sensors_json(...)`, passing the **real monotonic
   `frame_index`** (= `<NNN>`) and the moving rover/camera `pose_in_world`.

`<NNN>` is zero-padded 3 digits from `000`, +1 per frame. Intrinsics, `baseline_m`,
and `extrinsic_in_base_link` are constant across frames (rigid rig) by construction.
`stereo` always carries the FRONT pair; no `stereo_rear` (M1/M2 front-only).
`--cameras-seq` inherits `_drums_up = true` (wired in L0 `sidecar.gd::_parse_args`) so
the drum arms clear the front-stereo FOV.

The lane edits ONLY `capture_seq.gd`: the `--cameras-seq` flag/dispatch are L0-wired in
`sidecar.gd`, and `camera_rig.gd` / `sensors_emit.gd` are frozen seams it CALLS.

## Trajectory sourcing

The contract iterates a scene's per-frame `rover_rc` exactly as `sidecar._run_sequence`
does. The shipped scenes (`samples/tread_track_4wheel`, `samples/tread_track`, …) author
`rover_rc` on their FINAL driven frame only (earlier `tNNN` are the pre-drive null
frame, `rover_rc: null`). A SLAM/COLMAP sequence needs ≥2 frames whose rover position
DIFFERS, so the lane anchors on the scene's authored driven `rover_rc` and synthesises a
short straight APPROACH trajectory: `N` waypoints stepping the rover backward from the
anchor (`STEP_CELLS` grid-cells/frame) along its heading, so the rover genuinely moves
and ARRIVES at the authored driven pose on the last frame. The per-frame rover placement
+ heading-yaw math mirrors `sidecar.gd` (`_build_rover` rover_rc branch + `_heading_yaw`)
verbatim, so a real multi-`rover_rc` scene (when one ships) slots in unchanged.

Knobs (no new flags — reuse already-parsed sidecar members):
- frame count `N` = `--stride` (`sidecar._seq_stride`), default `6` when left at its
  default `2`; `STEP_CELLS = 6` (~0.12 m/frame on the 0.02 m/cell tread scenes).
- `--cam-pitch <deg>` tilts the pair down so terrain fills the frame (denser stereo).
- `--lander-standoff` / `--lander-yaw` / `--rover-sink` pass through to the lander/rover.

## Run recipe

```bash
cd godot_sidecar
# Multi-frame egress over the driven tread scene (front-stereo, 5 frames, ground-tilted):
./render_layers.sh -- \
    --scene ../samples/tread_track_4wheel/t018 \
    --layers terrain,clasts,rover \
    --cameras-seq \
    --stride 5 \
    --size 1280x720 \
    --cam-pitch 12
# Output: out/cam/tread_track_4wheel/{000,001,...}/{front_left,front_right}.png + sensors.json
```

`render_layers.sh` wraps `render.sh sidecar.tscn …` (xvfb + the vendored
`Godot_v4.6.3-stable` under `.tools/`, `--rendering-driver vulkan`; needs a GPU — RTX 4090
here). Render output under `out/cam/` is git-ignored (regenerable from this `.gd`).

### Verify the egress
```bash
python3 - <<'PY'
import json, glob, math
docs=[json.load(open(d)) for d in sorted(glob.glob("godot_sidecar/out/cam/*/0*/sensors.json"))]
fis=[d["frame_index"] for d in docs]
assert fis==list(range(len(fis))), fis                      # monotonic 0..N-1
poss=[d["rover"]["position_m"] for d in docs]
assert len({tuple(round(v,4) for v in p) for p in poss})==len(poss)  # rover MOVED
for d in docs:                                              # contract invariants
    assert d["schema_version"]=="sensor_bridge/1.1" and d["frame_convention"]=="godot"
    assert d["stereo"]["left"]=="front_left" and d["stereo"]["right"]=="front_right"
print("frame_index:", fis)
for fi,p in zip(fis,poss): print(f"  frame {fi}: ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})")
print("displacement %.3f m" % math.dist(poss[0],poss[-1]))
PY
```

## Verified (RTX 4090, Godot 4.6.3, scene `tread_track_4wheel`, 512x288, `--stride 5 --cam-pitch 12`)

- 5 frame dirs `000..004`, each with `front_left.png` + `front_right.png` + `sensors.json`.
- All 10 PNGs REAL renders (76–89 KB; 137k–156k non-zero bytes); left ≠ right every
  frame; all 5 left frames pixel-distinct (genuine motion + genuine stereo). The lunar
  grazing-sun terrain + the procedural lander with its id-0 AprilTag are visible.
- `frame_index` monotonic `0,1,2,3,4`; 5/5 distinct rover `position_m`:
  - 0: (3.100, 0.164, 3.380)  1: (3.160, 0.164, 3.480)  2: (3.220, 0.165, 3.580)
  - 3: (3.300, 0.167, 3.680)  4: (3.360, 0.170, 3.780)  — total displacement 0.477 m.
- intrinsics constant (fx=fy=339.79, cx=256.0, cy=144.0 at 512x288); `baseline_m`
  constant to float32 precision (~7e-7 m spread) and `== |extrinsic(L)−extrinsic(R)|`
  every frame; `front_left` extrinsic rigid across frames; no `stereo_rear`.

Verification simulated the corrected dispatch (see below) with a throwaway
`extends "res://sidecar.gd"` harness that `await`s `run_capture_seq`; the harness was
deleted (not committed). The lane source itself is `capture_seq.gd` only.

## REQUIRED integration fix (one word, in the FROZEN sidecar.gd — orchestrator applies at merge)

`run_capture_seq` MUST be `await`ed, like the single-frame `--cameras` call-site. The L0
dispatch currently is:

```gdscript
# sidecar.gd ~205-208 (L0, _cameras_seq_mode branch)
if _cameras_seq_mode:
    CaptureSeqScript.run_capture_seq(self)      # <-- NOT awaited
    get_tree().quit(0); return
```

Empirically (Godot 4.6.3): an un-awaited coroutine whose caller queues `get_tree().quit(0)`
on the next line gets **exactly ONE** `frame_post_draw` resume before the tree tears down,
so the sequence cannot render (black/empty frames). A SubViewport sharing the world only
renders freshly-(re)built geometry after a REAL process+draw frame elapses;
`RenderingServer.force_draw()` alone renders the environment background but NOT geometry
added the same tick (verified), so there is no synchronous workaround inside `capture_seq.gd`.

The fix is a single word — add `await` (mirrors `await _cameras_capture()` at
`sidecar.gd` ~201):

```gdscript
if _cameras_seq_mode:
    await CaptureSeqScript.run_capture_seq(self)   # awaited, like --cameras
    get_tree().quit(0); return
```

This is a `capture_seq.gd`-lane requirement on the frozen seam; this lane does NOT edit
`sidecar.gd`. With the `await`, the run recipe above produces the verified output.
