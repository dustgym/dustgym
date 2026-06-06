# dustgym vs the robotics curriculum — topic-coverage diff (2026-06-05)

A diff of what the standard robotics references teach against what dustgym's PRD + software actually
implements, and **what's missing**. Corpus = the MIT *Robotic Manipulation* course (Tedrake, fetched ToC)
+ the `kang702/robotic-books` set: Siegwart *Introduction to Autonomous Mobile Robots* (the most relevant),
Lynch & Park *Modern Robotics*, Siciliano *Robotics: Modelling, Planning and Control*, Siciliano & Khatib
*Springer Handbook of Robotics*, Murray-Li-Sastry *A Mathematical Introduction to Robotic Manipulation*,
Craig *Introduction to Robotics*, Spong *Robot Dynamics and Control*, Corke *Robotics, Vision and Control*,
Kajita/Nenchev (humanoids), and the linear-algebra foundations (Strang/Axler/Lay).

Mapped against the canonical topic structure of those texts (these are standard curricula) + the fetched
manipulation ToC. dustgym status is grounded in real modules. Legend: ✅ covered · 🟡 partial · ⬜ missing
· ⛔ host/data-gated · N/A (manipulation/humanoid-specific, out of scope for a wheeled construction rover).

---

## 1. Coverage matrix

| Topic area | Corpus source | dustgym status | Where / the gap |
|---|---|---|---|
| **Linear algebra / optimization** | Strang, Axler, Lay; all texts | ✅ | numpy linalg, least-squares fits (`rassol_mass_model`, `self_optimizing`), Dijkstra, TSP/DP, PPO/CEM |
| **Rigid-body transforms SE(3)/SO(3), screw theory, twists** | Modern Robotics, MLS, Craig | 🟡 | planar pose + unicycle (`rover.step_pose`); REP-103 frames in the sensor bridge (`frames.py`); no full SE(3)/screw kinematics (we are heightfield + planar) |
| **Manipulator FK/IK, Jacobians, differential kinematics** | Craig, Spong, Siciliano, Modern Robotics, manip Ch3 | N/A | no articulated arm; the RASSOR drum-arm is modelled only as arm-raise *energy* (`rassor_mass_model.arm_raise_lift_energy_j`), not FK/IK |
| **Mobile-robot kinematics (differential drive, nonholonomic)** | Siegwart, manip Ch7 | ✅ | `rover.step_pose` unicycle integrator; `drive.drive_step`/`closed_loop_drive`; per-body gravity |
| **Manipulator dynamics (Lagrange / Newton-Euler)** | Spong, Siciliano, MLS, Modern Robotics | N/A | no arm |
| **Contact dynamics & friction (cones, wrench)** | manip Ch5, Springer Handbook | 🟡 | Coulomb-Mohr + Janosi-Hanamoto slip (`slip.py`), repose (`sandpile`); wheel-soil only — no grasp/contact-wrench (N/A) and no force-controlled contact |
| **Wheel-terrain interaction / terramechanics** | (barely in corpus; Siegwart locomotion only) | ✅+ | **our depth** — Bekker pressure-sinkage, Lyasko low-g, per-body `bodies.py`, `material.py`. Beyond the corpus. |
| **Trajectory generation (time-scaling, splines)** | Modern Robotics, Siciliano, manip Ch3/6 | 🟡 | a battery-aware execution `timeline` (forecast), not a smooth/jerk-bounded trajectory |
| **Sampling-based planning (RRT, PRM)** | Modern Robotics, Siciliano, manip Ch6 (LaValle) | ⬜ | **gap** — we use grid Dijkstra + discrete TSP, no RRT/PRM |
| **Graph search (A*, Dijkstra), GCS** | Siegwart, manip Ch6 | 🟡 | `route_least_cost` = 8-conn Dijkstra over a slope/keep-out costmap; no A* heuristic, no Graphs-of-Convex-Sets |
| **Time-optimal path parameterization** | Modern Robotics, manip Ch6 | ⬜ | **gap** — fixed drive speed; no velocity profiling |
| **Manipulator / Cartesian / force / impedance control** | Spong, Siciliano, Modern Robotics, manip Ch8 | N/A | no arm; force-controlled *digging* is the relevant analog → ⬜ (dig is an energy model, Tier-3 force is gated) |
| **Mobile path-tracking control (pure pursuit, MPC)** | Siegwart, Corke | 🟡 | `closed_loop_drive` tracks cmd_vel with live slip; no pure-pursuit/MPC tracker |
| **Camera models, calibration, stereo, depth** | Corke, Siegwart, manip Ch4 | 🟡⛔ | `obs_map_producer` stereo-rectify+SGBM; Brown-Conrady is a stub; render-gated |
| **Point clouds, ICP, registration, tracking** | manip Ch4, Corke, Springer Handbook | ⬜⛔ | **gap** — COLMAP SfM exists but is gated; no ICP, no continuous tracking |
| **Object detection & segmentation (deep)** | manip Ch9/10, Corke | ⬜ | **gap** — no rock/obstacle detector; obstacles are operator-supplied static keep-outs |
| **Photometry / BRDF (Hapke, non-Lambertian)** | (not in corpus) | ✅+ | **our depth** — Godot Hapke/Lommel-Seeliger; the non-Lambertian-MVS finding |
| **Bayes/Kalman/EKF/particle state estimation** | Siegwart, Springer Handbook (Probabilistic Robotics) | 🟡 | scalar Kalman belief (`autonomy.py` predict/update_*); no EKF/UKF/particle filter |
| **Localization (Markov, MCL)** | Siegwart, Springer Handbook | 🟡 | AprilTag fiducial fix (gated) + the Kalman belief; no Monte-Carlo localization |
| **SLAM (EKF/graph/FastSLAM)** | Siegwart, Springer Handbook | ⬜⛔ | **gap** — rtabmap graph-SLAM is wired but never run (container-gated); fiducial pose only |
| **Map representations (occupancy/feature/elevation)** | Siegwart, manip Ch7 | ✅ | the conserved **elevation/state-field map** (`io_fields`, mutable, time-varying) + the map-channel coverage layer |
| **Navigation: global + local + reactive avoidance (VFH/DWA)** | Siegwart | 🟡 | global Dijkstra costmap + static keep-out routing; **no local/reactive layer** for discovered/dynamic obstacles |
| **Active perception / next-best-view** | manip Ch7 | 🟡 | `active_perception_env` (NBV env + greedy/beam baselines); no learned policy |
| **Grasping, grasp synthesis, antipodal, wrench cones** | manip Ch3/5, MLS, Springer Handbook | N/A | drum excavator, not a gripper; the analog is mass-conserved excavate/haul/dump (✅ via the authority) |
| **Pick-and-place / task planning** | manip Ch3/5 | ✅ | the construction analog: `mission_planner` (balance→route→sequence→validate); 7 sequencers + multi-objective |
| **Task-and-motion planning (TAMP), behavior trees** | manip Ch5, Springer Handbook | 🟡 | the **Plan IR** (typed-action list + DAG) is the substrate; no BT executive / integrated TAMP |
| **Reinforcement learning (policy-grad, value, model-based)** | manip Ch11, Springer Handbook | ✅+ | PPO (SB3) + CEM + **model-based beam-search on the exact authority** + search-distillation, with the honest "learning earns its keep only in multi-objective scheduling" finding |
| **Imitation / behavior cloning / diffusion policy** | manip Ch10/11 | 🟡 | search-distillation is BC-like; no diffusion policy |
| **Simulation / physics engines / Gym** | manip appendix (Drake), Springer Handbook | ✅ | conserved authority (sub-ms) + Chrono (⛔) + Godot render; the `dustgym` Gymnasium suite |
| **ROS / middleware / real-time I/O** | Corke, Springer Handbook | 🟡⛔ | ROS2 Jazzy bridge (gated); FastAPI server batch-only (the streaming/cmd_vel seam is the open item) |
| **Humanoid / legged modelling & control (ZMP, gait)** | Kajita, Nenchev, Springer Handbook | N/A | wheeled rover |
| **Tactile / proprioceptive sensing** | manip Ch12, Springer Handbook | 🟡 | proprioceptive drum-current mass inference (`rassor_mass_model`, ICE-RASSOR); no contact/tactile at the dig interface |
| **Energy / power / endurance modelling** | (not a focus of corpus) | ✅+ | **our depth** — IPEx-grounded energy/battery, endurance, survival power, weight-coupling |
| **Multi-robot coordination / fleet** | Springer Handbook | 🟡 | `plan_multi` (site-exclusive allocation, parallel makespan, conflict detection); shared-charger + haul-path deconfliction are future MV |

---

## 2. What you missed (in the corpus, relevant to a real mobile construction rover, absent/partial here)

Prioritized by leverage for "vehicles on Earth, plan→verify→execute→reassess":

1. **Recursive localization + SLAM** (Siegwart, Probabilistic Robotics): EKF/UKF, Monte-Carlo localization, and
   graph/EKF-SLAM. We have a scalar Kalman belief + a gated fiducial; nothing continuously localizes the rover.
   This is the #1 gap for a live loop (matches the architecture review's SLAM finding).
2. **Continuous + sampling-based motion planning** (Modern Robotics, Siciliano, manip Ch6): RRT/PRM, A* with
   admissible heuristics, GCS, and **time-optimal trajectory parameterization** for dynamically-feasible,
   smooth drive paths. We have grid-Dijkstra + discrete TSP only — adequate for routing, not for tight
   maneuvering or kinodynamic feasibility.
3. **Local / reactive obstacle avoidance** (Siegwart): VFH / dynamic-window (DWA) / potential fields layered
   under the global plan, reacting to *discovered* obstacles between re-plans. We have only the global static
   keep-out costmap.
4. **Sensor-based perception: ICP / point-cloud registration + deep detection/segmentation** (manip Ch4/9/10,
   Corke): turning camera/depth into a rock/obstacle detector → dynamic keep-outs, and ICP for pose tracking.
   We have gated SfM and no detector.
5. **Force / impedance control of the dig interface + Tier-3 contact dynamics** (Spong, Siciliano, manip
   Ch5/Ch8): sensing-while-digging, force-controlled excavation. We model dig as a fixed energy, not a force
   interaction (the gated Chrono SCM oracle).
6. **Path-tracking control** (Siegwart, Corke): pure-pursuit / MPC trajectory tracking. We integrate cmd_vel
   with slip but don't *track* a reference path with a controller.
7. **SE(3) / screw-theory kinematics + manipulator modelling for articulated tooling** (Modern Robotics, MLS):
   if the drum arm (or any future articulated tool) needs FK/IK/Jacobians, none exists (arm-raise is energy-only).
8. **TAMP + a behavior-tree/executive** (manip Ch5): an executive that consumes the Plan IR, monitors
   preconditions, and re-plans — the layer above the IR we just built.

## 3. Out of scope (manipulation/humanoid-specific — correctly absent)

Grasp synthesis, dexterous/soft hands, antipodal grasps & contact-wrench cones, peg-in-hole assembly, tactile
*gripping*, visual servoing of an end-effector, manipulator dynamics, humanoid/legged ZMP/gait. dustgym is a
wheeled mass-conserving earthmover, not an arm — these are not gaps, they are a different machine.

## 4. What dustgym has that the corpus largely does NOT (our contribution)

Deep **terramechanics** (Bekker/Janosi-Hanamoto/Lyasko, per planetary body); **mass-conserved earthmoving**
as the state transition (cut/haul/dump/grade/sinter, unhackable terrain-matching reward); **IPEx-grounded
energy/battery/endurance**; **planetary regolith physics + illumination/PSR + Hapke photometry**; the
**conserved-vs-learned** design split (exact dynamics, learn only the expensive perception/scheduling); and
**multi-vehicle construction scheduling**. The standard texts are arm/humanoid/generic-mobile; the
construction-earthmoving-on-regolith vertical is dustgym's own.

**Bottom line:** dustgym is strong exactly where a *construction earthmover* must be (terramechanics,
mass-conserved earthmoving, energy, mobile kinematics, RL-where-it-helps, the elevation map) and is missing
the *general mobile-autonomy stack* the corpus teaches — SLAM/localization, sampling/continuous motion
planning, reactive obstacle avoidance, and sensor-based perception — which is the same execution-and-perception
plumbing the architecture review flagged. The manipulation-specific half of the corpus (grasping, dexterous
hands, humanoids) is a different machine and correctly out of scope.
