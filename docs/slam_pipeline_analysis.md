# SLAM pipeline analysis — dustgym vs *SLAM in Autonomous Driving* (Gao Xiang) → refines PRD P15

Source: gisbi-kim/slam_in_autonomous_driving_en (the English *SLAM in Autonomous Driving*). It is the most
directly-applicable reference for dustgym's #1 live-loop gap (P15 / R6 — localization + SLAM). This maps its
pipeline ("process / overlays") onto what dustgym has and what P15 must build.

## The book's pipeline (process), chapter by chapter

| Ch | Stage | Key methods |
|---|---|---|
| 2 | Math foundations | SE(3)/SO(3) Lie groups, Kalman theory |
| 3 | **State estimation front-end** | **ESKF** (error-state KF), inertial nav (IMU), GNSS, integrated navigation |
| 4 | Graph back-end | **IMU preintegration**, factor-graph / pose-graph optimization |
| 5 | Point-cloud processing | **KD-tree nearest-neighbour**, linear fitting, voxel grids |
| 6 | 2D laser mapping | scan matching, **likelihood fields**, submaps, **loop closure**, pose graphs |
| 7 | 3D laser odometry | **ICP variants (point-to-plane), NDT, NDT-LO, LOAM-like**; loosely-coupled LIO |
| 8 | Tightly-coupled LIO | **IESKF**, preintegrated tightly-coupled lidar-inertial |
| 9 | Offline mapping | front-end + back-end + batch loop detection + global optimization |
| 10 | **Fusion positioning in a PRIOR map** | laser positioning, initialization, **map loading, EKF fusion** (scan → prior map) |

The "process overlay" is the canonical SLAM stack: **front-end odometry** (register consecutive scans →
relative motion) → **state estimator** (ESKF/IESKF fusing IMU + odometry + GNSS) → **back-end** (pose/factor
graph) → **loop closure** → **map**, and (Ch10) **localization against a prior map** by registering the live
scan onto the stored map and EKF-fusing the result.

## What dustgym already has (the half the book assumes you must build)

- **A PRIOR MAP, already** — the real LOLA Haworth DEM (`samples/lunar_dem/haworth_10km_5m`), an elevation
  map with a frozen state-field contract (`io_fields`). The book's Ch9 (build the map) is largely *done* for
  us; the relevant chapter is **Ch10 (localize in a prior map)**.
- **A recursive estimator skeleton** — `autonomy.py` `Belief` + scalar Kalman `predict`/`update_pose`/
  `update_energy`/`update_drum`. It is a scalar KF, not yet an ESKF over SE(3), but the fuse-predict-update
  loop + the independent-fix correction (just fixed) are the right shape.
- **One real exteroceptive fix** — the AprilTag pose channel (12.7 mm/7.15°, container-gated) + `rover_localize`
  (inverse tag→rover-map pose, verified 4e-16 m round-trip). This is a fiducial *map-positioning* fix.
- **A point-cloud producer** — `obs_map_producer` (stereo-rectify → SGBM → world-frame grid), render-gated.
- **Proprioception** — drum-mass FDC; the unicycle integrator is dead-reckoning odometry.

## What P15 must build (the gap, in the book's terms) — and the cheaper path for dustgym

dustgym does **NOT** need full SLAM-from-scratch (it already has the map). The right pipeline is the book's
**Ch10 fusion-positioning ("overlay") path**, which is simpler and more robust:

1. **Front-end registration = scan/point-cloud → PRIOR DEM** (the "overlay"). Register the live observed
   heightfield/point-cloud (from `obs_map_producer`, render-gated) onto the stored LOLA DEM via **ICP /
   point-to-plane / NDT** (Ch5/7), giving an absolute map-relative pose. Because the map exists, this
   *replaces* drift-prone odometry-only front-ends with a direct map fix. Needs: a KD-tree NN (Ch5) + an
   ICP/NDT solver scoring against the DEM. Pure-numpy/scipy is feasible at rover-patch scale.
2. **State estimator = ESKF over SE(3)** (Ch3): promote the scalar `Belief` KF to an error-state filter that
   fuses **IMU preintegration** (Ch4) + the **unicycle odometry** (we have) + the **scan-to-DEM registration
   fix** (1) + the **AprilTag fix** (we have). This is the core of "localize continuously instead of
   dead-reckoning." It reuses the predict/update structure already in `autonomy.py`.
3. **Back-end pose/factor graph** (Ch4/6/9): OPTIONAL for map-relative localization (you're not building the
   map). Add only if drift between map fixes matters; loop closure is largely moot when every scan is
   map-anchored.
4. **Initialization + map loading** (Ch10): seed the ESKF from the globe site-pick (`latlon_to_dem_origin`,
   we have) + the first scan-to-DEM registration.

So the concrete P15 build is: **KD-tree NN + ICP/NDT scan-to-DEM registration (the overlay) → an SE(3) ESKF
fusing IMU + odometry + that registration + the AprilTag fix**, swapped into `autonomy.execute_leg`'s seam (the
single call site) in place of the self-simulated truth. The dense scan producer is render/CUDA-gated (P6/P17),
but the registration + ESKF math is pure-numpy and testable against the conserved truth (register a noised
observed patch back onto the real DEM, recover the pose). Full SLAM (Ch6-9) is only needed off a prior map; on
the Moon we have LOLA, so map-relative fusion positioning (Ch10) is the right, smaller, more robust target.

## Honest gating

The *registration + ESKF* (the new math) is buildable + testable now on the conserved truth. The *live scan*
that feeds it (Godot render → stereo / COLMAP point cloud) is host/container/CUDA-gated (P6/P17), and the
rtabmap full-SLAM container (P15's heavy option) is unrun. The recommended order: build the pure-numpy
scan-to-DEM ICP/NDT + the SE(3) ESKF against the conserved truth first (no gate), then wire the gated live
scan when the render/container is up.
