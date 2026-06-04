# Speaker notes: foss_ipex / dustgym GMRO update (2026-06-04)

Notes for the 12-slide deck (`deck.md` / the rendered PDF). First person, John's voice. These enhance
the slides; they do not read the bullets aloud. Target time about 8 to 10 minutes.

---

**Slide 1: Title**
Open. This is foss_ipex, a sensor-faithful lunar terramechanics and autonomy sim. I am John McCardle;
Aaron Storey has been hosting and helping. It is CC0, an independent Godot plus Chrono plus ROS2 stack.
I reference the Lunar Autonomy Challenge as a benchmark, but this is not the official entry. This talk
covers what landed in the last few days.

**Slide 2: Where it stands**
Quick orientation so the new work has context. The core is a conserved Tier-2 terramechanics authority
in numpy, sub-millisecond per step, mass-exact, with load-bearing Bekker sinkage and a slip ladder. On
top of that are Gymnasium RL envs and a planner. The render side is a Godot sidecar with sourced Hapke
photometry, the LAC eight-camera rig, and an AprilTag pose channel closed at 12.7 mm. Two things are
new: the render track is now live on a GPU, and the section-10 map channel is closed.

**Slide 3: Godot render track live on the GPU**
The render track is genuinely running now, headless on a 3090 over Vulkan. This is a real render of one
of the crater-and-boulder scenes: 143 clasts, the 5-degree grazing sun, the long hard shadows that make
pole perception hard. The reflectance is Hapke / Lommel-Seeliger from Sato's LROC photometry, not
Lambert. Hold onto that distinction, it pays off three slides from now.

**Slide 4: The sensor-bridge output**
This is what the perception stack actually consumes, the rover's front-left camera. You can see the drum
at the bottom of the frame and the AprilTag lander ahead. The full rig is the LAC configuration: front
and rear stereo, side monos, two drum-arm cameras. These are the literal sensors.json plus PNG artifacts
our ROS2 stack reads, the same frozen contract on both sides.

**Slide 5: State-field contract render**
This is the seam between the physics and the render. The numpy authority writes a state-label field, the
Godot shader samples it: gray undisturbed, blue wheel tracks, yellow excavated, orange spoil. The point
is that the two engines agree on one scene, verified end to end on the GPU.

**Slide 6: Section-10 map channel closed (onboard tier)**
Here is the new capability. We build an observed elevation map from the rover's stereo and score it
against the conserved truth. This is the onboard tier: rectified stereo with exact extrinsics, SGBM,
back-projected and gridded, then scored. Real render, eight-station drive, no synthetic data.

**Slide 7: The honest perception result**
And here is the honest result, which I think is the useful part. Passive rover stereo at the rover's
15-centimeter eye height has about 30 centimeters of height noise at one sigma. Coverage grows as you
drive, 3 to 16 percent over eight stations. But our scenes have only about 5 centimeters of relief,
which sits below that noise floor. So the rover recovers the ground plane and where it has been, but not
the centimeter-scale micro-relief that decides whether a wheel sinks. That is not a defect, it is the
real limit, and it is the argument for active sensing or the ground tier.

**Slide 8: Visual-SLAM readiness**
On features: the render is not feature-starved. ORB saturates, six to eleven thousand features per lit
megapixel. The catch is the features in shadow. They are locked to the lighting and move as the sun
sweeps, so they are bad for a persistent map. For visual SLAM here the question is not whether there is
enough texture, it is feature repeatability under sun motion.

**Slide 9: Two perception tiers**
This is the architecture point. Two tiers. Onboard stereo is cheap and real-time but noisy, 32
centimeters. Ground COLMAP is offline and accurate, 4 centimeters, an order of magnitude better, with
cameras aligned to truth within 6 millimeters. That mirrors how you already work at GMRO, COLMAP the
image corpus into a map. The difference here is we have the ground truth to put an actual error number
on it, which you cannot do with real imagery.

**Slide 10: Photorealism for SfM is the BRDF (Hapke vs Lambert)**
This is the result I am most keen to show. We ran COLMAP on the same scene rendered two ways: physically
correct Hapke, and the idealized Lambert. Hapke gives COLMAP a third fewer 3-D points and 30 percent
less coverage. The non-Lambertian regolith reflectance costs you correspondences, which is exactly the
photoconsistency problem you hit on real lunar imagery. A Lambert render would tell you COLMAP performs
better than it really will. We can run that A/B and grade both against truth, and that is the whole value
of a sim that carries conserved ground truth.

**Slide 11: Real now vs honestly deferred**
Being explicit about what is real versus deferred. Real: the conserved physics, the closed loop, the RL
envs and planner, the render track on GPU, the AprilTag channel, and now both map-channel tiers scored
against truth. Deferred and named: dense MVS for full coverage, COLMAP on the grazing ground-level
moving sequence, secondary illumination in the permanently shadowed regions because our shadows are
near-black today and that is too dark, a real sensor noise model, and the live Chrono producer.

**Slide 12: Sources and lineage**
Sources, quickly. IPEx and GMRO are the target; we render the MIT-licensed EZ-RASSOR mesh. We reference
the Lunar Autonomy Challenge for scoring but this is an independent stack. The autonomy pattern follows
DS1 AutoNav. Photometry is Hapke and Sato's LROC maps. Terrain is the real LOLA Haworth south-pole DEM,
and the energy and battery numbers come from the IPEx ASCEND 2024 paper. Happy to go deeper on any of it.

---

## Likely questions

- **Why Godot, not Unreal like LAC?** For an airless single-hard-sun world the BRDF dominates and we
  have it sourced; we do not need Unreal-grade global illumination. The one place GI matters is secondary
  illumination in shadow, which is a cheap add and is on the deferred list.
- **Is 4 cm good enough?** For mapping and obstacle work, yes; for trafficability you want the micro-relief,
  which is the dense-MVS and sensor-model work. The honest number today is the point.
- **Can this run on the rover?** The onboard tier is the cheap real-time proxy; COLMAP is explicitly the
  ground tier, not flight compute. We are not claiming onboard COLMAP.
