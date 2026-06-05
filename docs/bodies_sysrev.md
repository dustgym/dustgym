---
title: "Per-planet constants"
nav_order: 4
---

# Per-Planet Terramechanics — Systematic Review (dustgym `Body` constants)

**Scope.** Source the surface/regolith mechanical constants for the planetary bodies that are real
targets for human **habitats** and/or ISRU **mining**, to replace placeholder gravity-scaled analogs in
`terrain_authority/bodies.py`. **Method:** parallel literature review per body (agency roadmaps, mission
results, returned-sample studies, peer-reviewed terramechanics). Every value is tagged **MEASURED**
(in-situ / returned-sample), **ESTIMATED** (simulant / model / analog), or **UNKNOWN**. **No value is
fabricated** — where the literature has none, the field is `None` in `bodies.py` and an explicit, flagged
analog stands in.

Date: 2026-06-02. Reviewers: 6 parallel research agents (1 target-ranking + 5 per-body).

---

## 1. Top targets for habitats / mining (ranked)

Ranked by maturity of a real habitat/resource plan, not science interest.

| Rank | Body | Role | Status | Anchor citation |
|---|---|---|---|---|
| 1 | **Moon** (south pole) | habitat **+** mining | funded hardware (Artemis ISRU) | NASA Moon-to-Mars Architecture, ADD Rev. B (2024); Sanders, NTRS 20240013906 |
| 2 | **Mars** | habitat **+** mining | ISRU flown (MOXIE) | Hecht et al., *Sci. Adv.* 8 (2022) eabp8636; NASA M2M Humans-to-Mars |
| 3 | **NEAs — Bennu / Ryugu** (C-type) | mining (water/volatiles) | sample-return done | Jin/Sanchez, *Acta Astronautica* 181 (2021) 291; Lauretta et al. (2024) |
| 4 | **Phobos / Deimos** | staging waypoint (+ possible ISRU) | precursor (JAXA MMX) | JAXA MMX; NASA NTRS 20160006319 (Phobos/Deimos waypoints) |
| 5 | **Ceres** | water/brine mining + habitat proposal | long-horizon (science now) | Dawn (De Sanctis/Raymond 2020 brines); Janhunen (2021) megasatellite |
| — | 16 Psyche | metal mining (speculative) | science-only mission, **no mining plan** | NASA Psyche; Shepard et al., arXiv:1306.2455 |

**Excluded — science-only (no habitat/mining case):** Europa (Clipper, astrobiology flyby), Enceladus
(Orbilander concept, plume science), Titan (Dragonfly, prebiotic chemistry), Callisto (concept only),
Vesta (Dawn, no volatiles). These are intentionally **not** in `BODIES`.

The defensible "real targets" tier for an excavation/rover sim is **Moon > Mars > Bennu/Ryugu >
Phobos**, with **Ceres** as a long-horizon water reserve. Only the Moon and Mars have funded hardware
flown or in build.

---

## 2. Per-body constants (as encoded in `bodies.py`)

Repo Bekker form: `p = (k_c/b + k_phi)·z^n`; at `n = 1`, `k_c` is N/m², `k_phi` is N/m³.

### Moon — `gravity-loaded`, fully MEASURED
| Param | Value | Tag | Source |
|---|---|---|---|
| gravity | 1.62 m/s² | MEASURED | standard |
| bulk density | 1300 kg/m³ surface → 1920 at ~1 m (Carrier hyperbola) | MEASURED | Carrier, *Lunar Sourcebook* 1991; ChaSTE 2025 (1940 kg/m³ polar) |
| cohesion | 170 Pa (0.1–1 kPa surface) | MEASURED | NASA LTV white paper |
| friction angle | 30–40° (→55° at depth) | MEASURED | NASA Eng. Guide NTRS 20220014634 |
| **Bekker** | **k_c=1400 N/m², k_phi=820,000 N/m³, n=1.0** | **MEASURED** | **NASA LTV NTRS 20220010732** |
| repose | static 40–50° / dynamic ~30° | MEASURED | Apollo obs. |

> **Key finding / DEFERRED flag.** The NASA LTV white paper gives `k_phi=820,000 N/m³, k_c=1400, n=1.0,
> c=170 Pa` **as the lunar reference values** — these are exactly the repo's `constants.py` numbers
> (labeled there "Earth/Apollo-era"). So the repo's `TerramechanicsParams.lunar()`, which applies an
> **additional** Lyasko 1g→⅙g reduction on top, would **double-count** the gravity correction if the
> base is already lunar. `bodies.params_for_body("moon")` therefore uses the sourced values **directly**
> (no second reduction) — this is the literature-correct Moon. (Flagged to John; does not change `lunar()`.)

### Mars — `gravity-loaded`, native Bekker UNKNOWN (simulant used)
| Param | Value | Tag | Source |
|---|---|---|---|
| gravity | 3.71 m/s² | MEASURED | NSSDCA |
| bulk density | 1500 kg/m³ (1000–2000 by terrain) | MEASURED/EST | Oravec 2020 (GRC review) |
| cohesion | 1000 Pa trafficable (0–2 kPa soil; 5.8 kPa duricrust; 0–15 kPa span) | MEASURED | Sullivan 2011; Spohn 2022 (InSight) |
| friction angle | 35° (30–37° soil; 15–20° drift) | MEASURED | Sullivan 2011; Pathfinder |
| **Bekker** | **k_c=23,200 N/m², k_phi=606,700 N/m³, n=1.0** (GRC-3 simulant) | **ESTIMATED** | **Oravec et al. 2020 NASA GRC** |
| repose | 30° (sand) / 37.7° (disturbed soil) | MEASURED | Atwood-Stone 2013; Sullivan 2011 |

> **No native-Mars bevameter exists.** The Bekker moduli are the GRC-3 simulant (Oravec 2020),
> unit-converted to repo N-units (kN→N). Flagged ESTIMATED-simulant, not Mars-measured.

### Ceres — `gravity-loaded`, Bekker UNKNOWN (lunar analog)
| Param | Value | Tag | Source |
|---|---|---|---|
| gravity | 0.284 m/s² | MEASURED | Park 2016 (Dawn) |
| bulk density | 1300 kg/m³ near-surface (crust 1200–1360; porosity 53–72%) | ESTIMATED | Ermakov 2017; Pan/Bland 2021 |
| cohesion | **UNKNOWN** (strength <~5 MPa, no Pa value) | UNKNOWN | Chilton 2019 |
| friction / repose | 34.5° ± 2.8° (repose; static-granular proxy) | MEASURED | Icarus 2024 |
| Bekker | **UNKNOWN** → lunar analog (flagged) | UNKNOWN | — |

> Friction = repose proxy. The 2–14° landslide values (Chilton 2019) are **ice-lubricated effective**
> friction for mass wasting — **not** a dry-granular angle; not used here. Water/brine-rich (Na₂CO₃,
> NaCl, MgSO₄·6H₂O; Dawn 2020) → mining; Janhunen 2021 habitat proposal.

### Bennu (C-type asteroid archetype) — `microgravity`, Bekker INVALID
| Param | Value | Tag | Source |
|---|---|---|---|
| gravity | ~4e-5 m/s² (3e-5 eq → 8.5e-5 pole; Ryugu ~1.1–1.5e-4) | MEASURED | Scheeres 2019; Watanabe 2019 |
| bulk density | 1190 kg/m³ (near-surface ~600; porosity 40–60%) | MEASURED/EST | Lauretta 2019; Walsh 2022 |
| cohesion | **≤ 2 Pa** (Bennu, best-fit ~0); Ryugu < 1.3 Pa | MEASURED (upper bound) | Walsh 2022; Arakawa 2020 |
| friction | ~33° (32.7 ± 2.5°) | ESTIMATED | Robin 2024 (boulder morphology) |
| **Bekker** | **N/A — model class wrong** | UNKNOWN/INVALID | Walsh 2022; Ballouz 2021 |

> **Bekker breaks down.** Pressure-sinkage assumes a gravity-loaded, overburden-confined soil; at ~1e-5 g
> there is no overburden — behavior is **cohesion-/contact-dominated** (granular Bond number), reproduced
> by **DEM / granular impact-drag**, not Bekker. Bennu's surface is near-fluidized, near-zero cohesion
> (TAGSAM). `bekker_regime="microgravity"`; the drive env **warns** and treats results as a placeholder.

### Phobos (Mars moon) — `microgravity`, Bekker UNKNOWN (analog)
| Param | Value | Tag | Source |
|---|---|---|---|
| gravity | ~0.0057 m/s² (varies ~210% shape / ~450% with Mars tides) | MEASURED | Ernst 2023; Andert 2010 |
| bulk density | 1850 kg/m³ (porosity ~30%) | MEASURED | Ernst 2023 |
| cohesion | ~500 Pa surface (model; 0–1 kPa band) | ESTIMATED | Hurford 2016 (tidal-fracture) |
| friction / repose | ~38° (33.5 ± 6.1° analog; slopes <40°) | ESTIMATED | Murdoch 2025 (IDEFIX); EPS 2021 |
| Bekker | **UNKNOWN** → lunar analog (flagged) | UNKNOWN | — |

> Regolith ≥100 m, very low thermal inertia (fine dust). Surface non-hydrated (no surface water);
> ISRU water is hypothetical (subsurface only). JAXA **MMX/IDEFIX** (~2027) will return first in-situ
> mechanics. `bekker_regime="microgravity"`.

### Earth — `gravity-loaded`, validation body
| gravity 9.81 | density 1600 | cohesion 1040 Pa | friction 28° | **Bekker k_c=990 N/m², k_phi=1,528,430 N/m³, n=1.1** (Wong dry sand) | repose 34° |

> Reference/validation only (Wong, *Theory of Ground Vehicles*, dry-sand table).

---

## 3. Cross-cutting findings

1. **Bekker is a gravity-loaded model.** It is valid for Moon/Mars/Ceres/Earth; for **Bennu and Phobos
   (micro-/milli-gravity)** it is out of regime — there is no overburden to confine the soil, so cohesion
   and granular dynamics (DEM, granular Bond number) govern. The sim flags these (`bekker_regime`,
   runtime warning) rather than pretending the numbers are physical. A future granular/DEM env is the
   honest path for asteroid/Phobos drive.
2. **Native Bekker moduli exist only for the Moon** (Apollo/LTV). Mars uses a simulant (GRC-3); Earth a
   terrestrial table; Ceres/Bennu/Phobos have none.
3. **The repo's `constants.py` Bekker values are the NASA *lunar* reference** — so `lunar()`'s extra
   Lyasko reduction likely double-counts (see Moon finding). Tracked as a DEFERRED fix; `bodies.py`
   sidesteps it by using sourced values directly.
4. **Cohesion spans 6 orders of magnitude** across these bodies: ~2 Pa (Bennu) → 170 Pa (Moon) → 1 kPa
   (Mars/Earth) → 5.8 kPa (Mars duricrust). Gravity spans 1.62 → 4e-5 m/s² (~40,000×).

---

## 4. Citations (primary)

**Targets:** NASA Moon-to-Mars Architecture ADD Rev. B (2024); Sanders, M2M ISRU, NTRS 20240013906;
Hecht et al., *Sci. Adv.* 8 (2022) eabp8636 (MOXIE); Jin & Sanchez, *Acta Astronautica* 181 (2021) 291;
JAXA MMX; NASA NTRS 20160006319 (Phobos/Deimos waypoints); Janhunen (2021) Ceres megasatellite.

**Moon:** Mitchell et al. (1972); Carrier, Olhoeft, Mendell (1991) *Lunar Sourcebook* Ch. 9; NASA LTV
terramechanics white paper NTRS 20220010732; NASA Eng. Guide NTRS 20220014634; ChaSTE/Chandrayaan-3,
*Sci. Reports* 15 (2025); Gasteiner et al., arXiv:2602.03829 (2026).

**Mars:** Oravec, Asnani, Creager, Moreland (2020) NASA GRC, NTRS 20200003046; Sullivan et al. (2011)
*JGR Planets* 116 E02006; Spohn et al. (2022) InSight HP3; Atwood-Stone & McEwen (2013) *GRL*; NSSDCA
Mars Fact Sheet.

**Ceres:** Park et al. (2016) *Nature* 537:515; Ermakov et al. (2017, 2019) *JGR Planets*; Chilton et al.
(2019) *JGR Planets* 10.1029/2018JE005634; Pan/Bland et al. (2021) *PSJ* 2:182; Icarus 2024
(S0019103524004135, repose); De Sanctis/Raymond et al. (2020) *Nat. Astron.* (brines).

**Bennu/Ryugu:** Scheeres et al. (2019) *Nat. Astron.* 3:352; Lauretta et al. (2019) *Nature* 568:55 &
*Science* 366 eaay3544; Walsh et al. (2022) *Sci. Adv.* 8 eabm6229; Ballouz et al. (2021) *MNRAS* 507:5087;
Watanabe et al. (2019) *Science* 364:268; Arakawa et al. (2020) *Science* 368:67; Robin et al. (2024)
*Nat. Commun.* 15:6203.

**Phobos:** Ernst et al. (2023); Andert et al. (2010) *GRL*; Hurford et al. (2016) *JGR Planets*
10.1002/2015JE004943; Ulamec et al. (2021) *EPS* (MMX rover); Murdoch et al. (2025) IDEFIX WheelCams;
Miyamoto et al. (2021) Phobos simulants PGI-1/PCA-1.

**Bekker/terramechanics foundations:** Bekker (1956, 1969); Wong, *Theory of Ground Vehicles* (1978).
