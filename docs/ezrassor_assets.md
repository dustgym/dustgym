# EZ-RASSOR Asset / Integration Assessment — foss_ipex

**Repo:** [FlaSpaceInst/EZ-RASSOR](https://github.com/FlaSpaceInst/EZ-RASSOR) — Florida Space Institute / UCF, "EZ Regolith Advanced Surface Systems Operations Robot," a small-scale demonstration robot mimicking NASA's RASSOR for visitors at Kennedy Space Center.

**Method:** Shallow clone (`git clone --depth 1`) into `/home/john/Development/foss_ipex/.vendor/EZ-RASSOR`, files inspected only — their ROS/Gazebo stack was **not** built or run. `.vendor/` added to `.gitignore`.

- **Clone size:** **251 MB** on disk (includes `.git`). Commit pinned: `0c5911b` ("Revamp Swarm Control Package (#402)").
- **Status flag:** All findings below are from the actual cloned files, with exact paths. Anything not directly verifiable from the repo is flagged.

---

## 1. License / IP verdict

**Verdict: CONDITIONAL-YES for the rover code and the rover meshes; NO for the `extra_models/` art.**

### The code and rover model: MIT, with explicit NASA + FSI provenance
- Single license file: `.vendor/EZ-RASSOR/docs/LICENSE.txt` — **MIT License**, `Copyright (c) 2019 [10 named UCF students], The Florida Space Institute, and The National Aeronautics and Space Administration`. There is **no** separate art/asset license; the repo licenses code and bundled assets under one MIT grant.
- The rover description package carries its own SPDX-style tag: `.vendor/EZ-RASSOR/packages/simulation/ezrassor_sim_description/package.xml` → `<license>MIT</license>`.
- **NASA / U.S. Government provenance:** NASA is a named copyright holder in the MIT notice. This is *not* a 17 USC §105 public-domain dedication — it is a permissive (MIT) license on a work co-authored by university students + FSI + NASA. For our purposes that is fine: MIT is one of the spec's preferred permissive licenses (spec IP note). It does **not** make the work public domain, so we cannot simply absorb it as "our" Gov work; we **carry the MIT attribution** (copyright + permission notice) alongside any reused file.

**Conditions for reuse (rover meshes + URDF + code patterns):**
1. Reproduce the MIT copyright + permission notice from `docs/LICENSE.txt` wherever we ship/convert those files (e.g. a `THIRD_PARTY_LICENSES` note next to the converted glTF in `godot_sidecar/`).
2. This is compatible with our public release: MIT permits use, modification, and redistribution including in a U.S. Gov work, as long as the notice rides along. It does **not** "compromise public release."

### The `extra_models/` art: DO NOT REUSE without independent license clearance — public-release risk
This is the one thing that would compromise public release if treated like the rest. The Gazebo world props under `.vendor/EZ-RASSOR/extra_models/` are **third-party models re-hosted**, not original FSI/NASA work. Their `model.config` files credit external sources with **no stated license**:
- `extra_models/giant_circle/model.config`: *"Original lander model by: https://clara.io/view/..."*
- `extra_models/rock_1/model.config`: *"Original rock model by: https://3dwarehouse.sketchup.com/model/..."* (author "Joe Cloud")
- Similar external-origin credits on the lander, ISRU plant, etc.

The repo's blanket MIT cannot relicense art the FSI team did not author. **These (rocks, lander, giant_circle, ISRU plant, solar panels) are explicitly out of scope for reuse** — exactly the "third-party copyrighted dependency that would compromise public release" the spec warns against. If we ever want rock/clast meshes, source them from a known-CC0/public-domain set, not from here.

**Bottom line:** Reuse the **rover** assets (`ezrassor_sim_description/`) under MIT-with-attribution. Avoid everything under `extra_models/`.

---

## 2. ROS integration (version + transferable patterns)

**Version: ROS1, Melodic (with Python 2.7 heritage). NOT ROS2.** Our spec targets ROS2 (Foxy/Humble-era), so this is a **migration**, not a drop-in.

Evidence:
- `docs/README.rst` install prerequisites: **"ROS Melodic", "Python 2.7"**, `rosdep`, `build-essential`, catkin workspace via `develop.sh`.
- 12× `<buildtool_depend>catkin</buildtool_depend>` across `packages/*/package.xml`; deps on `rospy`/`roscpp`, `gazebo_ros`, `rviz`. **No** `ament_*`, no `rclpy/rclcpp`, no `*.launch.py` — all launch files are ROS1 XML `.launch`.

### Package inventory (`.vendor/EZ-RASSOR/packages/`)
| Group | Package | Role |
|---|---|---|
| actions | `ezrassor_teleop_actions` | actionlib teleop server |
| autonomy | `ezrassor_autonomous_control` | obstacle detection, point-cloud nav, park_ranger localization |
| autonomy | `ezrassor_swarm_control` | multi-rover coordination |
| communication | `ezrassor_controller_server`, `ezrassor_joy_translator`, `ezrassor_keyboard_controller`, `ezrassor_topic_switch` | teleop input → instruction topics |
| messages | `ezrassor_teleop_msgs` | `Teleop.action` |
| simulation | `ezrassor_sim_control` | maps high-level instruction topics → joint/diff-drive controllers |
| simulation | `ezrassor_sim_description` | **URDF/xacro + meshes (the asset goldmine)** |
| simulation | `ezrassor_sim_gazebo` | Gazebo world/launch |
| extras | `ezrassor_launcher` | top-level roslaunch orchestration |

### Transferable control architecture (this is the valuable part for our ROS2 bridge)
The control stack has a clean two-layer shape worth copying into ROS2, independent of ROS1 internals:

**Layer A — high-level "instruction" topics (the public command API):**
- `wheel_instructions` (`geometry_msgs/Twist`)
- `front_arm_instructions`, `back_arm_instructions` (`std_msgs/Float32`)
- `front_drum_instructions`, `back_drum_instructions` (`std_msgs/Float32`)

**Layer B — driver nodes fan these out to controllers** (`ezrassor_sim_control/source/ezrassor_sim_control/`):
- `sim_wheels_driver.py`: `wheel_instructions` (Twist) → scales by `MAX_VELOCITY` → republishes Twist to `diff_drive_controller/cmd_vel`.
- `sim_arms_driver.py`: arm instructions (Float32 in [-1,1]) → `× MAX_ARM_SPEED (0.75)` → `arm_{front,back}_velocity_controller/command` (Float64).
- `sim_drums_driver.py`: drum instructions (Float32) → `× MAX_DRUM_SPEED (5)` → `drum_{front,back}_velocity_controller/command` (Float64).

**Controller config** `ezrassor_sim_control/config/default_position_controllers.yaml` (ros_control / Gazebo):
- `diff_drive_controller/DiffDriveController`: left wheels `[left_wheel_front_hinge, left_wheel_back_hinge]`, right `[right_..._hinge]`; `wheel_separation: 0.4`, `wheel_radius: 0.18`; linear cap 1.0 m/s.
- Four `velocity_controllers/JointVelocityController` (arm_front, arm_back, drum_front, drum_back), PID `{p:100, i:0.01, d:10}`.
- `joint_state_controller` @ 50 Hz.

**Sensor stack** (`ezrassor_sim_description/urdf/ezrassor.gazebo`) — topic shapes worth mirroring:
- Depth camera (`libgazebo_ros_openni_kinect.so`): 640×480, `horizontal_fov 1.29154` rad (~74°), near 0.105 / far 10 m; topics `color/image_raw`, `color/camera_info`, `depth/image_raw`, `depth/points`; optical frame `depth_camera_optical_frame`. **Note: distortion coeffs all 0** — no lens distortion modeled (our spec §8 wants Brown-Conrady; we add that ourselves).
- IMU (`libgazebo_ros_imu.so`) @ 50 Hz on `imu`, gaussian noise 0.05.
- Autonomy consumes `depth/points`, `imu`, `odometry/filtered` and emits `obstacle_detection/*` (`ezrassor_autonomous_control/source/...`).

**What is transferable to our ROS2 bridge (spec §11):**
- The **instruction-topic abstraction** (5 simple command topics) is a clean, minimal command surface — adopt the same topic names/semantics in ROS2 so any of their teleop/autonomy clients could be ported with near-identical message shapes (Twist + Float32/Float64; all `std_msgs`/`geometry_msgs`, which exist 1:1 in ROS2).
- The **diff-drive + per-joint-velocity controller split** is a sound model for our Chrono::Vehicle command mapping: wheels via a single Twist, arms/drums via per-joint velocity. We can hand Chrono the same decomposition without `ros_control`.
- The **depth-camera + IMU topic contract** (`depth/points`, `color/image_raw`, `imu`, `odometry/filtered`) is a ready-made sensor topic spec for the Godot sensor model to publish into.

**What is NOT transferable (do not reuse):**
- All the `libgazebo_ros_*.so` plugins (camera, IMU, ros_control) — ROS1 Gazebo binaries, replaced wholesale by Chrono (physics) + Godot (sensors).
- catkin/`develop.sh` build machinery, actionlib `Teleop.action` (ROS2 uses a different action codegen).
- Their physics entirely: Gazebo rigid-contact wheels with `mu1/mu2`, `kp 1e7` — **no terramechanics, no soil deformation, no excavation**. Nothing here informs our Tier-2 Bekker/Janosi work.

**Migration cost:** Moderate but bounded if we only want the control *patterns*: re-author ~3 tiny driver nodes in `rclpy` (each is <50 lines), redefine the controller mapping against Chrono instead of `ros_control`. The autonomy/swarm stack is a larger Python port and is **not** needed for the sim — skip it.

---

## 3. URDF / xacro inventory

**A complete, usable kinematic tree exists** — including RASSOR's signature counter-rotating bucket-drum arms.

Files (`.vendor/EZ-RASSOR/packages/simulation/ezrassor_sim_description/urdf/`):
| File | Contents |
|---|---|
| `ezrassor.xacro` | **Main robot**: all links, joints, transmissions, sensor links |
| `macros.xacro` | inertia macros + mesh-reference macros (`base_unit`, `robot_wheel`, `robot_arm_drum`, `drum_arm`) |
| `materials.xacro` | named visual materials |
| `ezrassor.gazebo` | Gazebo plugins + sensors (ROS1, not reusable) |

(SDF files elsewhere — `extra_models/*/model.sdf`, `extra_worlds/*`, `dem_scripts/*` — are world props / DEM tooling, not the rover.)

### Link / joint tree (from `ezrassor.xacro`)
Root `base_link` → fixed `body` (mesh `base_unit`, mass 15 kg).

| Joint | Type | Parent → Child | Origin (m) | Axis | Notes |
|---|---|---|---|---|---|
| `left_wheel_front_hinge` | continuous | base_link → left_wheel_front | (0.20, 0.285, 0) | (0,1,0) | wheel mesh, m=5 |
| `right_wheel_front_hinge` | continuous | base_link → right_wheel_front | (0.20, −0.285, 0) | (0,1,0) | |
| `left_wheel_back_hinge` | continuous | base_link → left_wheel_back | (−0.20, 0.285, 0) | (0,1,0) | |
| `right_wheel_back_hinge` | continuous | base_link → right_wheel_back | (−0.20, −0.285, 0) | (0,1,0) | |
| `arm_front_hinge` | continuous | base_link → arm_front | (0.20, 0, 0), rpy(π,0,0) | (0,1,0) | drum_arm mesh |
| `arm_back_hinge` | continuous | base_link → arm_back | (−0.20, 0, 0) | (0,1,0) | drum_arm mesh |
| `drum_front_hinge` | continuous | **arm_front** → drum_front | (0.388245, 0, 0), rpy(π,0,0) | (0,1,0) | drum mesh; cylinder collision r=0.1839, L=1.0 |
| `drum_back_hinge` | continuous | **arm_back** → drum_back | (−0.388245, 0, 0), rpy(π,0,0) | (0,1,0) | drum mesh |
| `body_joint`, `imu_joint`, `camera_front_joint`, `depth_camera_optical_joint` | fixed | base_link → body/imu_link/depth_camera_front/optical_frame | camera at (0.3,0,−0.1) | | sensors |

- **The signature drums ARE modeled:** two arms, each with a drum at its end (`drum_front_hinge`/`drum_back_hinge`), all four wheel + 2 arm + 2 drum joints are independent `continuous` actuators (8 actuated DoF + diff-drive). The drum *counter-rotation* is not encoded in the URDF axes (both drum axes are +Y); it's produced at the **control layer** by commanding opposite signs (see `sim_drums_driver`). So the kinematic tree is symmetric; the "counter-rotating" behavior is a control convention, which we replicate the same way.
- Transmissions: 8 `SimpleTransmission` blocks (4 wheels, 2 arms, 2 drums), all `VelocityJointInterface`.

### Usability assessment
**(a) As a Chrono::Vehicle source of truth — usable as a reference spec, NOT a direct import.** Per spec §11 and `docs/chrono_integration.md` §6 gotcha #3, **Chrono has no URDF import**. But this URDF is small and clean enough to *hand-transcribe* into a Chrono::Vehicle JSON/C++ model or a small converter: 9 moving links, all `continuous` revolute about local Y, simple box/cylinder inertias from macro formulas, explicit joint origins. The geometry (wheel r=0.18, separation 0.4/0.57, drum r=0.1839×L1.0, arm reach 0.388) gives us real dimensions to seed the model. **Caveat:** these are toy-demo inertias (everything ~5 kg, hand-rounded tensors) for a KSC visitor robot — use the *topology and dimensions*, not the masses, for any IPEx-faithful model. **Frame caveat:** URDF/ROS is Z-up (REP-103); our Chrono runs Y-up (chrono_integration §5) — transcription must swap axes.

**(b) To drive a Godot rover model — usable as the kinematic recipe, needs a parser.** Godot has no URDF import either (spec §11). The tree is simple enough that the Godot sidecar can hardcode the same node hierarchy: a `base_link` Node3D with 4 wheel children + 2 arm children, each arm carrying a drum child, at the joint origins above (converted Z-up→Y-up). We animate joints from Chrono state, not from URDF. So the URDF is the **kinematic recipe**, and the meshes (next section) are the visuals hung on it.

**Verdict: a usable, complete URDF exists.** It is the kinematic source of truth for both engines via transcription/parsing — exactly the "rebuild kinematic tree or write a converter" the spec calls for, and it is small enough that this is hours, not days.

---

## 4. Mesh inventory + Godot import path

### Rover meshes (the high-value set) — `ezrassor_sim_description/meshes/`
| File | Size | Represents | Used by xacro macro | Scale in URDF |
|---|---|---|---|---|
| `base_unit.dae` | 1.33 MB | Chassis / body | `base_unit` | 0.35 |
| `wheel.dae` | 116 KB | One wheel (instanced ×4) | `robot_wheel` | 0.35 |
| `drum.dae` | 650 KB | Bucket drum (instanced ×2) | `robot_arm_drum` | 0.35 |
| `drum_arm.dae` | 1.99 MB | Drum arm (instanced ×2) | `drum_arm` | 0.35 |
| `base_station.dae` | 152 KB | Hopper/base station prop (not on rover) | — | — |

**Format: all DAE / Collada.** Verified properties (matter for conversion):
- `<up_axis>Z_UP</up_axis>`, `<unit name="meter" meter="1"/>` — **Z-up, meters** (REP-103 / URDF convention). Godot is **Y-up**, so a −90° X rotation is needed on import (Godot's glTF/Collada importers can bake this, or apply at the node).
- Clean **triangle** meshes (`<triangles>` only, no n-gons/polylist) — robust to convert.
- Embedded **materials with diffuse colors** (`<material>`/`<effect>`/`<diffuse>` present) but **no external/embedded texture images** (no `<image>`/`init_from`) — pure flat-colored geometry, so no texture-path rewriting needed.
- Authored in **Blender 2.79 / 2.82** (`<authoring_tool>` tags). Blender source also present: `.vendor/EZ-RASSOR/blender/EZRASSOR.blend` (3.33 MB, Blender 2.79) — full editable source if we want to re-export cleanly.

### Godot 4.6 import reality
Godot 4.6 (binary present at `.tools/godot/Godot_v4.6.3-stable_linux.x86_64`) imports **glTF/GLB and OBJ natively and reliably**; its **Collada (.dae) importer exists but is legacy/limited** (the Godot docs themselves recommend glTF and warn the built-in Collada path is best fed by the OpenCollada exporter, not arbitrary DAE). It does **not** render STL. So: the DAEs *might* import directly, but the robust, reproducible path is **DAE → glTF**.

### Conversion tooling on this box — current state
Checked: **no Blender, no assimp, no obj2gltf/gltf-pipeline/FBX2glTF, no meshlab** on PATH; the project `.venv` has **no trimesh/pycollada/pygltflib**. So a converter must be installed. Good news: `pip install trimesh pycollada pygltflib` **resolves cleanly** (dry-run into the existing `.venv` succeeded, exit 0) — a pure-Python, no-GUI, deterministic path. (Disk is at ~95% / 26 GiB free; this stack is a few MB, fine — but do NOT `apt install blender`, that is hundreds of MB.)

### Concrete weekend path to get a RASSOR mesh into the Godot sidecar
**Recommended (lightweight, deterministic):**
```bash
# one-time, into the existing project venv (~few MB, NOT blender)
/home/john/Development/foss_ipex/.venv/bin/pip install trimesh pycollada pygltflib

# DAE -> glTF (trimesh reads Collada via pycollada, exports glb via pygltflib)
/home/john/Development/foss_ipex/.venv/bin/python - <<'PY'
import trimesh
m = trimesh.load('/home/john/Development/foss_ipex/.vendor/EZ-RASSOR/'
                 'packages/simulation/ezrassor_sim_description/meshes/base_unit.dae')
m.export('/home/john/Development/foss_ipex/godot_sidecar/assets/rassor_base.glb')
PY
```
Then in Godot: place the `.glb` as the rover node's mesh (replacing the placeholder cube in `render_test.tscn`), and apply the Z-up→Y-up fix (rotate −90° about X, or set the import "up axis"). Scale 0.35 matches the URDF macro if you want physical sizing parity.

**Fallback if trimesh's Collada read is lossy:** import the `.dae` directly into the Godot editor once (Godot's Collada importer will produce a `.scn`/import), or re-export from `blender/EZRASSOR.blend` to glTF — but that requires installing Blender (disk cost), so try trimesh first.

**Directly usable vs. needs conversion:**
- **Needs conversion (all of them):** every rover mesh is DAE → convert to glTF for a clean Godot 4.6 result.
- **Directly usable:** none are glTF/OBJ already; none are STL (so nothing is *blocked*, just needs the one DAE→glTF hop).

---

## 5. Recommendation for foss_ipex

### (a) This weekend — cheap, high-value reuse
1. **Replace the placeholder cube with a real RASSOR chassis.** Convert `base_unit.dae` → `rassor_base.glb` via the trimesh path above; drop into `godot_sidecar/`. Single highest-value, lowest-effort upgrade — turns the visualization from "cube on a plane" into a recognizable RASSOR-class rover. (~1 hour incl. the Z-up fix.)
2. **Convert `wheel.dae` and `drum.dae` too** while the converter is set up — wheels for the rover viz, and the **drum mesh is a natural marker for the EXCAVATED-zone / drum-contact viz** (spec §4 "Under drums" zone, §6 EXCAVATED label). Instancing 4 wheels + 2 drums + 2 arms in Godot reconstructs the whole rover from the §3 tree.
3. **Adopt the instruction-topic names now** (`wheel_instructions`, `{front,back}_arm_instructions`, `{front,back}_drum_instructions`) as the command vocabulary for the eventual ROS2 bridge, so any future port lines up.
4. **Carry the MIT notice:** add `docs/LICENSE.txt`'s MIT text as a `THIRD_PARTY_LICENSES`/attribution note beside the converted meshes. (Cheap insurance for public release.)

### (b) Later
1. **URDF as the kinematic source for Chrono::Vehicle.** Hand-transcribe the §3 link/joint tree (dimensions good, masses to be re-derived) into a Chrono::Vehicle JSON/C++ model — Chrono can't import URDF (chrono_integration §6 #3), but this tree is small. Reuse the joint origins, wheel radius/separation, drum geometry; swap Z-up→Y-up.
2. **Mirror the control decomposition + sensor topic contract** in the ROS2 bridge: diff-drive Twist for wheels, per-joint velocity for arms/drums; publish `depth/points`, `color/image_raw`, `imu`, `odometry/filtered` from the Godot sensor model. Counter-rotation = opposite-sign drum commands (their convention).
3. **Optionally re-export from `blender/EZRASSOR.blend`** if we want higher-fidelity / re-UV'd meshes later.

### NOT worth reusing (explicit)
- **`extra_models/` art** (rocks, lander, giant_circle, ISRU plant, solar panels) — third-party, no clear license → **public-release risk; source clasts elsewhere** (§1).
- **All ROS1 Gazebo plugins / physics** (`libgazebo_ros_*.so`, `mu1/mu2/kp` rigid wheels) — no terramechanics, superseded by Chrono.
- **catkin/`develop.sh` build system, actionlib `Teleop.action`, the autonomy & swarm Python stacks** — ROS1, and out of scope for the sim.
- **The base_unit/drum_arm DAE inertias/masses** — toy demo values for a KSC exhibit robot, not IPEx-faithful; reuse topology+dimensions only.

---

### Couldn't fully determine / flags
- **Exact license of the `extra_models/` source assets:** their `model.config` links (clara.io, 3dwarehouse.sketchup.com) state no license — treated as **unknown → unusable**. Not chased further (out of scope, and avoiding them is the safe call).
- **Whether Godot 4.6's Collada importer ingests these specific DAEs cleanly** was not tested by actually importing (would require launching the editor); the recommendation routes around it via trimesh→glTF, which is the reproducible path regardless.
- **trimesh's Collada fidelity on these files** is asserted from the clean triangle/material structure + a successful pip dry-run, not from an executed conversion (no install was performed per the "inspect only" instruction). If it disappoints, the `EZRASSOR.blend` re-export is the fallback.
