# ccsds_ros_nav — CCSDS/ROS 2 rover navigation on the Haworth DEM

Demonstrates **actual rover navigation** in the dustgym terramechanics sim, driven through a
**CCSDS Space Packet** command/telemetry link and a **ROS 2** message bus, with a **minimal-but-real
ground station** (move-and-wait). The rover follows planned waypoints across the real LOLA **Haworth**
lunar DEM using an onboard pure-pursuit controller over the conserved `terrain_authority` authority;
stereo imagery is rendered on the GPU and downlinked CFDP-style.

This is an integration layer, not a new physics model: it *commands* the existing authority
(`drive.drive_step`) and reuses the frozen seams (`INTERFACE.md`, `sensor_bridge_contract.md`,
`scripts/ros2_bridge/frames.py`, the Godot `--cameras-seq` egress). Moon-first; body-parameterized so
the same call path runs Earth (`g=9.81`, Wong dry-sand) for terrestrial rover testing.

## Architecture

```
 ground station ──CCSDS Space Packets (APID-routed) over UDP──▶ ccsds_bridge (rclpy)
   route plan, GoTo TC                                            │ decode → /cmd/nav_goal
   ◀── Pose/Leg TM, light-time delay ─────────────────────────   │
                                                                  ▼
                                              rover_executive (rclpy, timer-driven)
                                                pure-pursuit → drive.drive_step on Haworth
                                                publishes /tf /odom /rover/state /rover/leg
                                                onboard safing (entrapment, battery reserve)
 host GPU:  render/render_egress.py  ──▶  Godot --cameras-seq stereo + sensors.json ──▶ CFDP Img downlink
```

The pure-Python core (codec, link, controller, flight loop, ground station) is transport-agnostic and
runs without ROS — the rclpy nodes are a thin binding so ROS is genuinely the in-container bus
(rviz / rosbag2 / a future Nav2 attach to `/tf`, `/odom`, `/cmd/nav_goal`). See `CONTRACT.md` for the
APID registry, packet layouts, topics, and coordinate conventions.

## Run it

### 1. Pure-Python end-to-end (no ROS, no GPU) — verifiable in the .venv / CI
```bash
python scripts/ccsds_ros_nav/run_demo.py            # full Haworth rim-crest traverse (Moon)
python scripts/ccsds_ros_nav/run_demo.py --quick    # small window / short route (fast smoke)
```
Writes `out/ccsds_nav/{traverse.png, telemetry.json, telemetry.csv}`. `traverse.png` overlays the
slip-coloured rover path on the real Haworth hillshade plus the downlinked slip/sinkage/SOC series.

### 2. Containerized ROS 2 stack (CCSDS over UDP + ROS topics)
```bash
cd scripts/ccsds_ros_nav
docker-compose build           # NOTE: docker-compose (v1) on this host, not "docker compose"
docker-compose up              # flight stack (bridge + executive) + ground station traverse
```
Host networking shares loopback for the CCSDS/UDP link (52000/52001) and ROS 2 DDS. The flight service
runs `nav_bringup.launch.py`; the ground service runs `ground_station_main.py`.

### 3. Human-in-the-loop operator console (web, point-and-click)
The Earth side of the loop: click a goal on the real Haworth hillshade and watch the rover drive there
over a (configurable) light-time delay. Run the rover stack in the container and the console on the host
(it needs the GPU for the camera render):
```bash
cd scripts/ccsds_ros_nav && docker-compose up flight          # rover: bridge + executive (ROS 2)
python scripts/ccsds_ros_nav/console_server.py --port 8080    # operator console (host) -> open localhost:8080
```
- **Map page** (`/`): click to set a goal → a slope-aware route is commanded leg-by-leg; the rover marker,
  SOC/slip, mission clock, and Sun (az/el, lit %) update live. Sliders: **round-trip latency** (0–20 s,
  `0 = training/no delay`), **time factor** (accelerates the whole world — rover, mission clock, Sun, and
  wall-latency together), and **Sun elevation**. A `STOP` button sends the `Safe` telecommand.
- **Camera page** (`/cameras`): all 8 rover cameras (front/rear stereo, side mono, drum) rendered at the
  rover pose under the **mission Sun** — this is the imagery feed COLMAP consumes. Accelerating time
  sweeps the Sun's azimuth and walks terrain shadows across the patch.

The console **owns the latency model** (so the operator genuinely experiences the lag) and the **mission
clock** (Sun azimuth sweeps 360°/synodic-month; the start time is auto-chosen for good illumination).
Point-and-click *goals* — not continuous joystick teleop — is the latency-correct paradigm: the
stabilizing control loop runs onboard, so light-time only delays goal issuance and situational awareness.

### 4. Stereo imagery egress (host GPU) + CFDP-style downlink
```bash
# after a traverse (step 1 or 2) has written out/ccsds_nav/telemetry.json:
python scripts/ccsds_ros_nav/render/render_egress.py --legs 0,3,6 --frames-per-leg 6
```
Drives the frozen Godot `--cameras-seq` path per leg (xvfb + Vulkan, RTX-class GPU), then "downlinks"
each stereo frame as a real CCSDS `Img` packet into `out/ccsds_nav/downlink/` with `img_manifest.json`.

## Reused vs new

- **Reused (not reinvented):** `terrain_authority.drive.drive_step` (motion + slip + mass conservation),
  `scenes`/`io_fields` (Haworth crop), `bodies` (per-body g + soil), `ipex_specs` (battery/energy),
  the Godot `--cameras-seq` egress + `sensors.json`, `scripts/ros2_bridge/frames.py` (REP-103).
- **New here:** the CCSDS 133.0-B Space Packet codec (`ccsds.py`), the command/telemetry messages
  (`messages.py`), the UDP/loopback links (`link.py`), the onboard pure-pursuit executive (`flight.py`),
  the slope-aware route planner (`route.py`), the ground station (`ground_station.py`), and the rclpy
  bindings (`nodes/`).

## Conventions

- No third-party CCSDS serializer: the Space Packet codec is pure stdlib `struct`, so the wire format is
  auditable and the tests run on a bare CPU. The hot path stays NumPy-only.
- No stubs / no synthetic terrain: every test drives a small crop of the **real** Haworth DEM; the ground
  station is a real commander, not a placeholder. Mass is conserved by the authority (drift `0.0`).
- ROS-dependent tests are guarded with `pytest.importorskip("rclpy")` so CI (CPU) stays green; the same
  tests exercise the nodes inside the container.
- Earth note: `body=earth` threads `g=9.81` + Wong dry-sand soil through the identical path, but the
  Haworth DEM is lunar terrain — a real Earth DEM is the follow-up for terrestrial-analog testing.
