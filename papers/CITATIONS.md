# Research corpus — citation manifest

The `papers/` directory is John McCardle's private research library for this
project. The **PDFs themselves are third-party copyrighted material (journal,
IEEE, conference, and NASA documents) and are NOT redistributed** with this
repository — they are excluded by `.gitignore` for the same reason the
EZ-RASSOR `extra_models/` art is excluded (see [`../THIRD_PARTY.md`](../THIRD_PARTY.md)).
This repo is dedicated to the public domain (CC0); third-party copyrighted works
cannot be relicensed, so only this manifest of *references* is committed.

The repo cites these works **by filename** — in [`../README.md`](../README.md) §4
("what's papered over") and in each scene's `metadata.json`. This file maps those
filenames to what they are so the citations remain meaningful to anyone who
clones the repo; obtain the documents from their publisher / DOI / NASA NTRS.

| Filename (as cited) | What it is / how it's used here |
|---|---|
| `lyasko2010.pdf` | Lyasko, reduced-gravity terramechanics / slip-sinkage. Anchors the 1g→⅙g Bekker recalibration flagged `[CALIB]` (README §4 rows 1, 3; §5). |
| `ascend24-ipex-trl-5-design-overview.pdf` | IPEx TRL-5 design overview (AIAA ASCEND 2024). Anchors the Chrono authority model / single-authority design (row 2). |
| `asce-es-2024-isru-pilot-excavator-wheel-testing.pdf` | ASCE Earth & Space 2024 — IPEx wheel testing. Anchors single-pass rover / slip-sinkage discussion (row 3). |
| `asce-es-2022-isru-pilot-excavator-bd-scaling.pdf` | ASCE Earth & Space 2022 — IPEx bulk-density scaling. Background for the mass-areal column model. |
| `2021-ASCEND-Mass-Inference-RASSOR.pdf` | ASCEND 2021 — mass inference for RASSOR counter-rotating-drum excavation. Anchors the gentle-excavation dust model (row 5). |
| `rock-size-freq_abstract.txt` | Golombek (2003) rock size-frequency distribution abstract. Anchors the Golombek-SFD clast field F_k(D) (row 9). |
| `geosciences-15-00207-v3.pdf` | MDPI *Geosciences* 15:207. PSR / volatile-optics background for the inert ice field (row 10; §5). |
| `FULLTEXT01.pdf` | PSR frost / volatile-stability reference (row 10; §5 frost-optics direction). |
| `20-3 ICE-RASSOR.pdf` | ICE-RASSOR (icy-regolith RASSOR variant) reference. |
| `ice_rassor_learning_excavation.pdf` | Learning-based excavation control for RASSOR. |
| `ascend24-ipex-trl-5-design-overview.pdf` | (see above) |
| `Final IEEE paper formatted footnote added.pdf` | IEEE paper (IPEx / ISRU lineage) — supporting reference. |
| `s44461-025-00002-7.pdf` | Springer Nature article (2025) — supporting reference. |
| `1-s2.0-S001046552400119X-main.pdf` | Elsevier (ScienceDirect, PII S0010-4655…24) — supporting reference. |
| `LPSC 2023 Abstract_Connolly_Carrier_v2.pdf` | LPSC 2023 abstract (Connolly & Carrier) — lunar regolith reference. |
| `ipex_tent.pdf` | IPEx test-environment / tent facility note. |
| `329 Innovation Park - Site Plan Regolith STRIVES 3.pdf` | KSC GMRO regolith facility (STRIVES) site-plan reference. |
| `2603.17232v1.pdf` | arXiv preprint 2603.17232v1 — supporting reference. |
| `perceived_vs_measured_ai_progress.pdf` | "Perceived vs. measured AI progress" — context for the sensor-faithful-evaluation framing. |
| `trl5_2024_presentation/` | IPEx TRL-5 (2024) presentation slide images. |

*Descriptors are paraphrased from how each work is used in this repo; consult the
original for authoritative bibliographic data.*

## Photometry references (cited by author/year; not in the corpus)

The Hapke / Lommel–Seeliger terrain+clast BRDF (`godot_sidecar/*.gdshader`;
[`../docs/render_fidelity_spec.md`](../docs/render_fidelity_spec.md) §9) uses **foundational
photometry equations and published lunar parameters**, cited by author/year rather than by a local
PDF — these are textbook relations, appropriate to cite directly. They are *not* in `papers/`;
they are recommended additions to the private library.

| Reference | What it anchors here |
|---|---|
| Hapke, B. (1981) *Bidirectional reflectance spectroscopy 1. Theory.* JGR 86, 3039–3054. | The IMSA framework, the Lommel–Seeliger single-scattering core μ₀/(μ₀+μ), and the H-function multiple-scattering approximation. |
| Hapke, B. (2002) *…5. The coherent backscatter opposition effect and anisotropic scattering.* Icarus 157, 523–534. | The shadow-hiding opposition surge B(g)=B₀/(1+tan(g/2)/h). |
| Hapke, B. (1984) *…3. Correction for macroscopic roughness.* Icarus 59, 41–59. | The mean-slope θ̄ roughness/shadowing term S(i,e,g;θ̄) — **deferred** (spec §9), cited as the next refinement. |
| Sato, H. et al. (2014) *Resolved Hapke parameter maps of the Moon.* JGR Planets 119, 1775–1805 (doi:10.1002/2013JE004580; LROC). | The numeric lunar **mare** parameters used: 2-term Henyey–Greenstein `b=0.26, c=0.08` @643 nm (and the highlands envelope for the optional per-scene override). |
| Hapke, B. (2012) *Theory of Reflectance and Emittance Spectroscopy*, 2nd ed. Cambridge Univ. Press. | Consolidated reference for the double-HG phase function form and the IMSA bidirectional reflectance. |

## Clast-shape (boulder geometry) reference (cited by author/year; not in the corpus)

The procgen boulder clasts (`godot_sidecar/clast.gdshader` + `sidecar.gd` `_build_clasts`) render with a
literature-sourced **triaxial (non-spherical) shape** instead of identical spheres. The axial-ratio
distribution is sourced (not eyeballed); the per-instance triaxial scale is renormalized to geometric-mean
1.0 so the Golombek-SFD diameter the physics chose is preserved (shape varies, equivalent size does not).

| Reference | What it anchors here |
|---|---|
| Tsuchiyama, A. et al. (2022) *Three-dimensional shape distribution of lunar regolith particles collected by the Apollo and Luna programs.* Earth, Planets and Space 74:172 (doi:10.1186/s40623-022-01737-9; X-ray microtomography). | The lunar-fragment **three-axial ratios** used for the clast triaxial scale: whole-sample means S/I=0.770, I/L=0.758, **S/L=0.581** (short/long ≈ 0.58, "more equant than Itokawa / impact fragments"). The render samples b/a∈U(0.65,0.9) (≈I/L) and c/a∈U(0.5,0.75) (≈S/L) bracketing these means, short axis constrained ~vertical (rest face). |

## Lunar DEM / terrain-statistics references (DEM-terrain thrust; cited by author/year + dataset DOI)

For the real-DEM 10 km terrain work ([`../docs/lunar_dem_10km_eval.md`](../docs/lunar_dem_10km_eval.md),
[`../docs/dem_terrain_contract.md`](../docs/dem_terrain_contract.md)). **HONESTY TAGS carry onto the parameter**
per the binding "every parameter sourced, never eyeballed" rule: `[CALIB]` = a calibration choice;
`[prior]` = a global/equatorial/mare value applied to the pole because no polar in-situ measurement exists;
`[secondary]` = number taken from an abstract/secondary snippet, primary PDF not yet verified.

| Reference / dataset | What it anchors here | Tag |
|---|---|---|
| **PGDA LOLA 5 m South-Pole DEMs** — Barker et al. 2021, *Improved LOLA Elevation Maps for South Pole Landing Sites*, Planet. Space Sci. 203:105119 (doi:10.1016/j.pss.2020.105119); Mazarico et al. 2011, Icarus 211:1066 (doi:10.1016/j.icarus.2010.10.030). | The Haworth `_surf.tif` heightmap basis + `_slp`/`_toterr` anchors + the per-pixel effective-resolution / illumination story. NASA-GSFC US-Government work (no formal license string published). | data |
| **USGS down-selected Artemis III nav grids** (ScienceBase doi 10.5066/P1MEQ6UK). | Explicit **CC0** coordinate grids (reproject to IAU_2015:30135, record provenance). | data (CC0) |
| **2026 Shape-from-Shading 5 m DEMs** — Bertone et al. 2026, Planet. Sci. J. (doi:10.3847/PSJ/ae5b70; Zenodo 10.5281/zenodo.17954508). | Higher-detail fallback heightmap — **CC-BY-4.0, NOT CC0** (segregate or reference-only). | data (CC-BY) |
| Ivanov/Neukum/Hartmann 2001, *Production function*, Space Sci. Rev. 96:55. | Crater production polynomial (T=3.5 Gyr). Coeff vector cross-checked against MintonGroup/cratermaker (**GPL-3.0**, verified 2026-05-31 — numbers reused as uncopyrightable facts, **no code copied**; primary PDF still absent from `papers/` so the table is not yet directly verified). a0→8.173e-4 km⁻²Gyr⁻¹@1 km is the production constant term; 8.38e-4 is the *chronology* linear coeff and 8.25e-4 is the a10 shape coeff — three distinct numbers, not one normalization. | `[CALIB]` |
| Xiao & Werner 2015, JGR 120 (doi:10.1002/2015JE004860); Minton et al. 2019 (Icarus, arXiv:1902.07746). | Small-crater **equilibrium** cap (Xiao&Werner 1–10 % band for highland/polar; Minton mare fit = lower bound). | `[CALIB]` |
| Pike 1977 (repo, `constants.py:178`); Stöffler 2006, RiMG 60:519; Stopar 2017, Icarus. | Crater depth/diameter (0.196 >400 m; 0.11–0.17 below) + rim height (0.036 D). | `[FIXED]`/`[CALIB]` |
| McGetchin et al. 1973, EPSL 20; Settle & Head 1977; Melosh 1989. | Ejecta radial decay `(r/R)⁻³`, continuous extent 2.3–2.7 R. | `[FIXED]` |
| Golombek & Rapp 1997 (doi:10.1029/96JE03319); Golombek 2003 (repo `rock-size-freq_abstract.txt`); Bandfield 2011 (Diviner background <1 %). | Boulder cumulative-fractional-area SFD + spatial-k background vs ejecta. | `[FIXED]`/`[CALIB]` |
| Bernhardt/Boazman 2022 (PSJ doi 10.3847/PSJ/aca590); Watkins 2019 (JGR doi 10.1029/2019JE005963) + USGS LROC NAC Boulder DB v1; Bickel & Kring 2020 (Icarus doi 10.1016/j.icarus.2020.113850). | Boulder densities / clustering / max sizes (validate vs the USGS NAC Boulder DB). | `[secondary]` |
| Rosenburg et al. 2011, JGR (doi:10.1029/2010JE003716); Barker et al. 2025, PSJ (doi:10.3847/PSJ/adbc9d). | Self-affine **Hurst** exponent (south pole H≈0.95 highland-like) for the fbm spectral slope. | `[CALIB]` |
| Helfenstein & Shepard 1999, Icarus 141; Bandfield et al. 2015 (Diviner). | cm-scale Hurst (0.5–0.7) + terminal RMS slope (~20°) — **equatorial/global**. | `[prior]` |
| Durga Prasad et al. 2026, ApJ (doi:10.3847/1538-4357/ae5228); Mathew et al. 2025, Sci. Rep. (doi:10.1038/s41598-025-91866-4). | ChaSTE polar two-layer regolith density profile (69.4°S). | `[CALIB]` |
| Ruesch & Woehler 2021 (arXiv:2109.00052). | Boulder buried-fraction physics — **qualitative only, no numeric distribution** (stays `[UNKNOWN]`). | `[UNKNOWN]` |
