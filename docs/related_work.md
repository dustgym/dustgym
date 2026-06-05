---
title: "Related work"
nav_order: 3
---

# Related work — where dustgym lands in the field

This is the shareable synthesis of a five-field systematic review (NASA planetary rover autonomy,
lunar mining / ISRU robotics, world models, learned autonomous driving, and SLAM / 3D perception)
conducted to position dustgym against the literature. It cites published work only; the full review
with downloaded sources lives in a local (gitignored) `publication/` workspace. No copyrighted text
is reproduced here.

## The one-sentence position

dustgym occupies a sparsely-populated quadrant: **conserved-physics dynamics with a learned model
reserved only for perception, for terrain *transformation* (excavation / construction) rather than
navigation, scored against conserved truth (height / volume / mass) rather than photometric fidelity.**
The fields around it are converging the other way — learned dynamics, fleet-scale data, photometric
supervision — which is exactly what makes this corner defensible for the lunar surface-construction
problem (transform not traverse; GNSS-denied; no fleet; mass must balance).

## The five fields, compressed

**NASA planetary rover autonomy** is model-based, certifiable geometric search that *measures* slip
after the fact via visual odometry under a hard compute ceiling — GESTALT arc-voting and VO slip
detection (Maimone et al., 2006/2007), Field D\* replanning (Ferguson & Stentz, 2006), AEGIS
autonomous science targeting (Estlin et al., 2012; Francis et al., 2017), and recent onboard
map-matching localization and multi-robot autonomy (CADRE). It is extremely safe but **reactive, not
predictive**: no conserved terramechanics model sits inside the planning loop, so slip is a
measurement, never a forecast.

**Lunar mining / ISRU robotics** has a well-laddered hardware lineage (RASSOR → ICE-RASSOR drum-mass
inference → IPEx, TRL-5 design 2024) and a real public benchmark in the JHU APL Lunar Autonomy
Challenge (a CARLA/Unreal IPEx digital twin, ~5 cm mapping). But nothing has flown at bulk excavation
scale; Chang'e-5/6 is the only off-Earth digging, and it samples rather than moves bulk regolith. The
central weakness is a **sim-to-real chasm**: photoreal renderers optimize perception while soil
dynamics stay simplified, and the high-fidelity DEM/Chrono regime is ground-side and not real-time.

**World models** fall into learned-latent-dynamics (Ha & Schmidhuber, 2018; PlaNet/RSSM, Hafner et al.;
Dreamer v1–v3; MuZero), generative-video (GAIA-1, Genie, Cosmos, Vista), and non-generative
representation-prediction (JEPA / V-JEPA-2) families, with a thin **hybrid exact-dynamics +
learned-component** quadrant. The nearest construction analogue uses DreamerV3 to *learn* lunar
excavation dynamics — the principled inversion of our design; another line (exact-physics excavation
RL) supplies conserved dynamics but adds no learned perception/info-gain model on top. dustgym is the
combination of both: conserved dynamics, learned perception.

**Learned autonomous driving** spans modular-with-safety-case (Waymo), end-to-end jointly optimized
(PilotNet, Bojarski et al. 2016; UniAD, Hu et al. CVPR 2023; VAD), and world-model / neural-simulator
AV (MILE, GAIA-1/2, DriveDreamer, Vista), over a shared BEV/occupancy representation (Lift-Splat-Shoot,
Philion & Fidler 2020; BEVFormer, Li et al. 2022; occupancy networks). Its strength — joint
optimization, fleet-scale data, generative simulators — is its trap: behavior cloning is provably
insufficient, the long tail resists data, the generative simulators reward realism (FID/FVD) not
physical validity, and there is **no certification path for a learned policy** except wrapping an
auditable stack in a safety case.

**SLAM / CV / 3D perception** interleaves classical geometric methods (ORB-SLAM 1–3, Mur-Artal &
Tardós; VINS-Mono, Qin et al.; DSO; COLMAP, Schönberger & Frahm 2016; semi-global matching,
Hirschmüller 2008), learned perception (RAFT-Stereo, DROID-SLAM, SuperGlue, and the DUSt3R/MASt3R
feed-forward-3D frontier), and neural/explicit reconstruction (NeRF, Mildenhall et al. 2020;
Instant-NGP; 3D Gaussian Splatting, Kerbl et al. 2023; SplaTAM/MonoGS), with active-perception theory
giving greedy next-best-view a provable 1−1/e bound under submodularity (Krause & Guestrin). The field
is mature and real-time but degrades exactly where the Moon is hardest (low texture, grazing light,
GNSS-denied) and optimizes photometric fidelity, not conserved metric truth.

## The gap matrix

| Dimension | Field's center of mass | dustgym |
|---|---|---|
| Dynamics | learned, or coarse-empirical measured post-hoc | conserved, mass-exact, sub-ms, unhackable |
| Learning budget | everything, fleet-scale | only the expensive perception / render branch |
| Task | navigation (traverse) | transformation (excavate, grade, berm) |
| Verification | safety-case around a learned policy | dynamics provable; small learned surface |
| Perception eval | photometric (PSNR / FID / F-score) | conserved-truth (height / volume / mass) |
| Data regime | big data / fleet | low-data; out-physics the tail |
| Sim coupling | photoreal render *or* simplified dynamics | conserved dynamics + faithful render in one loop |
| Active perception | learned / heuristic | greedy near-optimal (submodular; measured) |

## Where dustgym lands

Every closest analogue holds one of dustgym's pieces but not the coupling: the DreamerV3 lunar
excavation work *learns* the dynamics we conserve; the exact-physics excavation RL line has no learned
perception; the Lunar Autonomy Challenge winning stack is strong perception/mapping on a CARLA/Unreal
twin, not a conserved-physics construction world model; the Project-Chrono lunar-construction co-sim is
the nearest published *system* but without a conserved-truth-scored map channel or a learned perception
branch; flight rover autonomy is predictive about geometry but measures slip post-hoc. dustgym is the
stack that **conserves the dynamics, learns only the perception, transforms rather than traverses, and
scores against conserved truth** — a low-data, verifiable point in a field sprinting toward
learned-everything.

What this implies, concretely:

- **Borrow** the BEV/3D-occupancy representation as the map layer; recast occupancy-*forecasting* as
  "how the regolith changes after a scoop"; use the world-model-as-on-policy-simulator pattern (the
  self-optimizing slip-energy loop already instantiates it); adopt VINS-style visual-inertial as a pose
  back-end and DEM anchoring as a third map tier (a design independently validated by 2026 DEM-anchored
  lunar stereo-SLAM work).
- **Do not adopt** the fleet/data-engine paradigm (no fleet; out-physics the tail), learned/generative
  dynamics (rejected by design), navigation-as-the-task (we transform terrain), or GNSS/HD-map priors.
- **Build, because nothing off-the-shelf exists**: a learned perception model that predicts info-gain
  over the expensive render branch (so the policy plans without rendering every candidate viewpoint);
  BRDF-aware / non-Lambertian MVS for regolith (motivated by a measured ~33% COLMAP point-loss under a
  physically-correct Hapke render versus an idealized Lambert one); and a conserved-truth reconstruction
  benchmark (height/volume/mass-conserving scoring, which has no direct competitor).

## Measured findings that fill named gaps

- A ~0.30 m onboard passive-stereo height-precision floor at the rover's grazing eye-height — a scarce
  datapoint in the degraded-conditions (low-texture / grazing-light) gap.
- The Hapke-vs-Lambert ~33% COLMAP point loss — a quantified instance of the non-Lambertian-MVS gap.
- Greedy next-best-view ties multi-step beam on active perception — the empirical face of submodular
  near-optimality (Krause & Guestrin), and the reason the onboard active-perception tool is cheap greedy
  while the learned model's value is the expensive-observation regime.

*Full per-field key-works tables, SWOT, gap analyses, and the downloaded sources are in the local
`publication/` workspace (gitignored). The cross-field positioning above is mirrored, in long form, in
`publication/systematic_review.md`.*
