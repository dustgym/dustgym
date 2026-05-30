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
