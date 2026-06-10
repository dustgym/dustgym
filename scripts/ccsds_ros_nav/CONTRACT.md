# ccsds_ros_nav — internal contract (frozen)

A teleoperated/​supervised rover nav loop wired through a **CCSDS Space Packet** command/telemetry
link and a **ROS 2** message bus, driving the conserved terramechanics authority
(`terrain_authority.drive.drive_step`) across the real LOLA Haworth DEM. The ground station is a
minimal-but-real mission-control commander (move-and-wait); there are no stubs or fake data.

This file is the seam the modules agree on. Treat it like `INTERFACE.md` / `sensor_bridge_contract.md`:
extend additively, do not break.

## 1. CCSDS Space Packet (CCSDS 133.0-B-2)

6-octet primary header, big-endian, then the packet data field:

```
 word0 (16b): version(3)=0 | type(1) | sec_hdr_flag(1) | APID(11)
 word1 (16b): seq_flags(2)=0b11 (unsegmented) | packet_seq_count(14)
 word2 (16b): packet_data_length = (octets in data field) - 1
```

`type`: 0 = TM (telemetry, rover→ground), 1 = TC (telecommand, ground→rover).

**Secondary header (mission convention):** when `sec_hdr_flag=1`, the first 8 octets of the data field
are a big-endian IEEE-754 `float64` **Mission Elapsed Time [s]** (a simplified CCSDS-301 time code).
The remaining octets are the user-data payload (§3). All packets in this stack carry the MET secondary
header.

## 2. APID registry

| APID  | Dir | Name        | Payload (§3) |
|-------|-----|-------------|--------------|
| 0x0C8 | TC  | CMD_GOTO    | GoTo waypoint |
| 0x0C9 | TC  | CMD_SAFE    | Safe / all-stop |
| 0x0CA | TC  | CMD_SETSIM  | Set sim time-acceleration factor |
| 0x064 | TM  | TLM_POSE    | Pose + drive telemetry sample |
| 0x065 | TM  | TLM_LEG     | Leg-complete summary |
| 0x066 | TM  | TLM_IMG     | Imagery file metadata (CFDP-style downlink announce) |
| 0x7FF | --  | IDLE        | reserved per 133.0-B (unused) |

## 3. Payload layouts (big-endian `struct`)

- **GoTo** `>I d d d d` — `leg_id:u32, goal_row:f64, goal_col:f64, v_max_mps:f64, goal_radius_cells:f64`
- **Safe** `>H` — `reason:u16`
- **SetSim** `>d` — `time_factor:f64` (sim seconds per wall second; the executive retimes its drive loop)
- **Pose** `>H 8d B` — `leg_id:u16, row, col, yaw_rad, v_achieved_mps, slip, sinkage_m, slope_rad, soc, entrapped:u8`
- **Leg**  `>H H 6d` — `leg_id:u16, status:u16, commanded_dist_m, achieved_dist_m, energy_J, mass_kg, final_row, final_col`
  - status: 0=REACHED 1=ENTRAPPED 2=LOW_BATTERY 3=MAX_STEPS 4=SAFED
- **Img**  `>H H H H I H` + utf-8 name — `leg_id:u16, frame_index:u16, width:u16, height:u16, size_bytes:u32, name_len:u16, name[name_len]`

## 4. Coordinate + physics conventions (from terrain_authority)

- Grid index `rc = (row, col)`, fractional. Heading `yaw=0` points +col; forward unit in (row,col) is
  `(sin yaw, cos yaw)`. To steer at a waypoint Δ=(drow,dcol): `desired_yaw = atan2(drow, dcol)`.
- World metres (local crop frame): `x = col*cell_m`, `z = row*cell_m`; ROS REP-103 conversion is done
  once downstream in `scripts/ros2_bridge/frames.py`.
- Body gravity threads through `drive.drive_step(..., g=bodies.get_body(name).g)`. Soil via
  `bodies.params_for_body(name)`. Moon first; Earth is `g=9.81` + Wong dry-sand (same call path).
- Mass is conserved by the authority; the executive **commands**, it never writes terrain directly.

## 5. ROS 2 topics (container binding)

| Topic | Type | Dir |
|-------|------|-----|
| (UDP)             | CCSDS Space Packets | ground↔bridge (off-bus, 52000/52001) |
| `/cmd/nav_goal`   | std_msgs/String (full GoTo as JSON — lossless: leg_id, row, col, v_max, radius) | bridge→executive |
| `/cmd/safe`       | std_msgs/Empty | bridge→executive |
| `/sim/time_factor`| std_msgs/Float64 (live sim acceleration; executive retimes its drive loop) | bridge→executive |
| `/tf`, `/odom`    | tf2_msgs, nav_msgs/Odometry (REP-103: x=East/col, y=North/-row, z=up, yaw_ros=-yaw) | executive→bus |
| `/rover/state`    | std_msgs/String (Pose + MET as JSON) | executive→bridge |
| `/rover/leg`      | std_msgs/String (Leg + MET as JSON) | executive→bridge |

MET (mission elapsed time) rides the CCSDS secondary header (§1), not the Pose/Leg payload struct; the
executive carries it in the JSON so the bridge can stamp the downlink packets. `/cmd/nav_goal` is JSON
(not a PointStamped) so the full GoTo survives — a native PoseStamped/Nav2 action goal is a clean
follow-up for richer ROS tooling.

The CCSDS↔ROS translation lives in exactly one place (`ccsds_bridge_node.py`), mirroring the
`frames.py` "convert once" discipline.

## 6. Modules

- `ccsds.py` — Space Packet codec (§1). Pure stdlib.
- `messages.py` — APID registry + payload codecs (§2, §3). Pure stdlib.
- `link.py` — `Link` interface; `LoopbackLink` (in-process, deterministic, exercises the wire bytes) and
  `UdpLink` (datagram, configurable light-time delay + loss) for the container.
- `flight.py` — `FlightModel`: load Haworth crop, onboard pure-pursuit waypoint follower over
  `drive.drive_step`, onboard safing, telemetry. Pure Python (no ROS).
- `ground_station.py` — minimal mission control: send GoTo, receive telemetry, trajectory artifact.
- `run_demo.py` — wires ground ⇆ link ⇆ flight end-to-end (no ROS); writes artifacts. Verifiable in `.venv`.
- `mission_clock.py` — lunar mission clock + Sun model (azimuth sweep / synodic month; illuminated start).
- `console_server.py` — the HITL operator console (host FastAPI): point-and-click map + camera (COLMAP)
  feed. OWNS the adjustable round-trip latency model (live, 0 = training) and the mission clock; the
  rover stack stays in the container. The bridge therefore runs delay-free for HITL.
- `nodes/` — thin rclpy bindings (container only; `importorskip` in tests).
- `render/` — host-GPU Godot stereo egress: `camera_render.py` (shared sun-aware render) + `render_egress.py`.
