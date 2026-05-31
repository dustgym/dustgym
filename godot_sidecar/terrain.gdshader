shader_type spatial;
// Lunar regolith terrain shader (spec §6 state-driven appearance, §8 lighting;
// detail pipeline render_fidelity_spec.md §4.2).
//
// Vertex geometry for the ACTIVE zone comes pre-displaced from the CPU mesh
// (heightmap is authoritative; INTERFACE.md §4). This fragment stage owns
// *appearance* only — the physics/render split of spec §4 ("physics owns
// geometry + material state at coarse res; rendering owns appearance").
//
// State-driven look (spec §6 transitions, §8 "exposed-sublayer albedo"):
//   EXCAVATED (drum cut) -> brighter + denser-looking: fresh firm sublayer
//     exposed, higher albedo (spec §6 "expose dense sublayer ... brighter albedo").
//   TREAD (wheel rut)    -> slightly compacted + darker: paved/packed (spec §6).
//   disturbance [0,1]    -> roughness/brightness modulation; no physics reads
//     it back (INTERFACE.md §4).
//
// DETAIL PASS (render_fidelity_spec.md §4.2, ADDITIVE — flat-normal default is
// recovered when track_dir_tex is neutral and disturbance is 0):
//   §4.2.2 base granularity : a high-frequency value-noise detail-normal so the
//     surface reads as granular regolith at the ~8 mm mesh scale instead of glassy,
//     strength x disturbance, FADED with camera distance so far tiles don't shimmer.
//   §4.2.3 cleat marks (TREAD): transverse grouser ridges oriented by the local
//     TRACK DIRECTION sampled from the baked §5.2/§4.3 track-dir field, period =
//     the grouser-cleat pitch (~1-3 cm; EZ-RASSOR wheel, asce-es-2024-isru-pilot-
//     excavator-wheel-testing.pdf), strength x disturbance.
//   §4.2.4 drum teeth (EXCAVATED/SPOIL): periodic scoop/teeth ridges oriented by the
//     drum-swath travel dir, period = teeth pitch (RASSOR drum,
//     2021-ASCEND-Mass-Inference-RASSOR.pdf), PLUS a capped hand-rolled layered
//     parallax raymarch (there is NO POM keyword in a custom shader — §4.2 note).
//
// All detail is a NORMAL (and parallax-UV) perturbation only; geometry stays the
// heightmap's. The tangent frame is built in-fragment from screen-space derivatives
// of VERTEX + field_uv (the ArrayMesh carries NO TANGENT array), so no mesh change
// is needed. When track_dir_tex is neutral the cleat/teeth contributions vanish.
//
// Lunar surface: no atmospheric scatter, near-black shadows, high specular-less
// roughness. Diffuse is owned by the custom light() below — a Hapke / Lommel-Seeliger
// BRDF for the airless, backscattering regolith (render_fidelity), NOT Lambert (spec §8).

render_mode specular_disabled;

uniform sampler2D disturbance_tex : source_color, filter_linear;  // R = [0,1]
uniform sampler2D state_tex : filter_nearest;                     // R = label/255
uniform sampler2D density_tex : filter_linear;                    // R = kg/m^3 (raw)

// Baked track-direction + phase field (§4.3 / INTERFACE.md §5.2). R,G = unit TRAVEL
// direction packed dir*0.5+0.5 (field space: x=+col/+X, y=+row/+Z); B = arc-length
// phase in [0,1]. NEUTRAL default (0.5,0.5,0) => zero direction => no marks, so an
// existing scene with no §5.2 keys (crater etc.) renders exactly as before. Always
// set by terrain.gd (sf.tex_track_dir()), so this uniform is never missing.
uniform sampler2D track_dir_tex : filter_linear;

uniform float density_lo = 1170.0;
uniform float density_hi = 1456.0;

// Base regolith albedo (mare-ish mid grey; spec §8 dusty surface).
uniform vec3 base_albedo : source_color = vec3(0.45, 0.44, 0.43);

// Macro albedo mottle (render_fidelity_spec.md §8 "exposed-sublayer albedo"; #2). A
// low-frequency value-noise variation applied to ALL states incl. VIRGIN so undisturbed
// regolith reads as a weathered, non-uniform surface instead of a flat grey slab. Pure
// appearance -- physics never reads albedo (INTERFACE.md §4). ~0.42 m primary cell +
// a finer fleck octave; amp is the peak +/- fractional brightness swing.
uniform float macro_albedo_amp = 0.13;
uniform float macro_scale_m = 0.42;

// Cut-depth / exposed-sublayer albedo (render_fidelity). Fresh-cut regolith reads brighter
// and slightly less reddened than the space-weathered surface; the brightening is graded by
// the EXCAVATED areal-mass deficit -> removed thickness (M0 - mass_areal)/surface_density [m].
// Immune to compaction (which leaves mass_areal untouched) and to natural topography (which
// lives in datum). Gated by cut_depth_enabled (uniform-mantle scenes only) -> fully additive.
uniform bool cut_depth_enabled = false;
uniform sampler2D mass_areal_tex : filter_linear;   // R = kg/m^2 (raw)
uniform float mantle_areal_m0 = 156.0;              // pristine mantle areal mass (kg/m^2)
uniform float surface_density_cd = 1300.0;          // density of the removed surface material
uniform float cut_depth_full_m = 0.08;              // removed thickness mapping to full fresh
uniform float fresh_albedo_gain = 1.5;              // albedo multiplier at full exposure

// --- PHOTOMETRY: Hapke bidirectional reflectance (render_fidelity; replaces Lambert) ---
// Lunar regolith is a backscattering, airless granular surface; Lambert (cos i) is wrong at
// the grazing/low-sun angles the IPEx POLAR mission cares about — it crushes low-incidence
// slopes to black. Lighting is owned by light() (below) using the Hapke IMSA model (isotropic
// multiple-scattering approximation), built up from the Lommel-Seeliger single-scattering core
// so each physical term stays legible. The single-scattering albedo w is the per-texel ALBEDO
// computed above (macro mottle + state tints + cut-depth all flow in as a spatially/chromatically
// varying w — standard Hapke usage). See the light() block for the full equation + sources.
//   hapke_enabled : false => Lambert fallback (the A/B comparison baseline; --brdf lambert).
//   b, c          : 2-term Henyey-Greenstein phase params, lunar MARE @643 nm
//                   (Sato et al. 2014, LROC-derived global Hapke parameter maps).
//   B0, h         : shadow-hiding opposition surge amplitude / angular width [rad] (Hapke 2002).
//   hapke_gain    : reflectance->radiance calibration (documented; absolute brightness is also
//                   set by the Sun light_energy + filmic exposure, so this is NOT a hidden knob).
uniform bool  hapke_enabled = true;
uniform float hapke_b  = 0.26;
uniform float hapke_c  = 0.08;
uniform float hapke_B0 = 1.0;
uniform float hapke_h  = 0.06;
// Radiance calibration: lands the nominal grazing-sun regolith at the same mid-tone as the
// prior Lambert look (measured Hapke/Lambert lit-terrain ~0.71 at gain 1.0 -> 1.4 equalizes),
// so the A/B comparison isolates BRDF SHAPE, not overall level. Reconciles the folded-PI scale
// with the established light_energy(3.0)+filmic exposure; documented, not a per-scene look knob.
uniform float hapke_gain = 1.4;

// --- detail pass tunables (render_fidelity_spec.md §4.2/§8 open params) ----------
// World metres spanned by the field UV (0..1) — terrain.gd sets it from grid extent
// so the in-shader noise/cleat periods are in real metres regardless of field size.
uniform float field_span_m = 5.12;
// Base granularity (§4.2.2): grain size and bump strength. ~8 mm grains read at the
// mesh scale; strength is the max normal tilt (radians-ish) scaled by disturbance.
uniform float grain_size_m = 0.013;   // pebble-scale bumps -> pronounced grazing-sun micro-shadows
uniform float grain_strength = 1.00;  // stronger relief (crisper, pebblier; Apollo-surface ref)
// Fine granular ALBEDO speckle (Apollo-surface grit; lpi.usra.edu apollo_14 ref). World-anchored
// high-frequency BRIGHTNESS variation at the granule scale so the ground reads as gritty regolith
// instead of a smooth/interpolated slab, AND so passive stereo has matchable intensity texture (it
// was previously NORMAL-only -> too low-texture for SGBM to match -> blank depth). ONE fine layer
// (+ a finer fleck) — the MACRO structure comes from the physics heightmap + macro mottle, so no
// deep fbm. Amplitude is per-state (state_grain_factor): TREAD turned way down (compaction smooths).
uniform float grain_albedo_amp = 0.22;    // peak +/- fractional brightness speckle
uniform float speckle_size_m = 0.020;     // granule cell size (m) for the albedo speckle
// Grouser cleat pitch + depth (§4.2.3; EZ-RASSOR wheel). 2 cm pitch sits in the
// 1-3 cm band; transverse ridges.
uniform float cleat_pitch_m = 0.02;
uniform float cleat_strength = 0.9;
// Drum teeth pitch + depth + parallax (§4.2.4; RASSOR drum). teeth pitch 2.5 cm.
uniform float teeth_pitch_m = 0.025;
uniform float teeth_strength = 1.1;
uniform float teeth_parallax_m = 0.006;   // apparent teeth depth for the raymarch
uniform int   teeth_parallax_layers = 8;  // CAPPED raymarch layers (cost bound, §4.2)
// Distance fade (§4.2 "fade detail strength with camera distance"): detail is full
// strength within fade_near_m and gone by fade_far_m (kills far-tile shimmer). Extended
// (#2) from the original 1.5/8.0 so the granular relief carries across the visible patch
// in the trailing-cam fly-through; the MSAA+SMAA+1.5x SSAA stack absorbs the extra
// high-frequency content without shimmer.
uniform float fade_near_m = 3.0;
uniform float fade_far_m = 20.0;

varying vec2 field_uv;   // 0..1 across the tile, u=+X/col, v=+Z/row

void vertex() {
	// UV0 is authored as field UV by the mesh builder.
	field_uv = UV;
}

// state enum -> integer (R8 stored as 0..4 / 255 by FORMAT_R8 sampling).
int state_label(vec2 uv) {
	float s = texture(state_tex, uv).r;          // 0..1 in steps of 1/255
	return int(round(s * 255.0));
}

// --- simplex noise (Ashima Arts / Stefan Gustavson "webgl-noise", MIT) --------
// The prior cheap-hash value noise was sampled on a SQUARE lattice and is not isotropic, so it
// showed axis-aligned square/diamond artifacts — and the crisp speckle made the lattice visible.
// Simplex noise uses a triangular simplex grid and IS isotropic, so the grit carries no grid
// pattern. snoise() returns ~[-1,1]; vnoise() wraps it to [0,1] so every existing call site (macro
// mottle, grain_height, speckle) is unchanged.
vec2 _sx_mod289(vec2 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec3 _sx_mod289(vec3 x) { return x - floor(x * (1.0 / 289.0)) * 289.0; }
vec3 _sx_permute(vec3 x) { return _sx_mod289(((x * 34.0) + 1.0) * x); }
float snoise(vec2 v) {
	const vec4 C = vec4(0.211324865405187, 0.366025403784439, -0.577350269189626, 0.024390243902439);
	vec2 i = floor(v + dot(v, C.yy));
	vec2 x0 = v - i + dot(i, C.xx);
	vec2 i1 = (x0.x > x0.y) ? vec2(1.0, 0.0) : vec2(0.0, 1.0);
	vec4 x12 = x0.xyxy + C.xxzz;
	x12.xy -= i1;
	i = _sx_mod289(i);
	vec3 p = _sx_permute(_sx_permute(i.y + vec3(0.0, i1.y, 1.0)) + i.x + vec3(0.0, i1.x, 1.0));
	vec3 m = max(0.5 - vec3(dot(x0, x0), dot(x12.xy, x12.xy), dot(x12.zw, x12.zw)), 0.0);
	m = m * m; m = m * m;
	vec3 x = 2.0 * fract(p * C.www) - 1.0;
	vec3 h = abs(x) - 0.5;
	vec3 ox = floor(x + 0.5);
	vec3 a0 = x - ox;
	m *= 1.79284291400159 - 0.85373472095314 * (a0 * a0 + h * h);
	vec3 g;
	g.x = a0.x * x0.x + h.x * x0.y;
	g.yz = a0.yz * x12.xz + h.yz * x12.yw;
	return 130.0 * dot(m, g);
}
// [0,1] isotropic noise — drop-in for the old value noise (same name/range, no grid artifacts).
float vnoise(vec2 p) { return snoise(p) * 0.5 + 0.5; }

// 2-octave granular height in [0,1] at a point scaled into "grain space".
float grain_height(vec2 gp) {
	return 0.65 * vnoise(gp) + 0.35 * vnoise(gp * 2.7 + 11.3);
}

// Per-state micro-texture amplitude (the "different noise profiles" of spec §6 states). VIRGIN is
// full natural grit; TREAD is turned WAY down because wheel compaction packs the fines smooth (the
// rut reads polished — spec §6 "compaction ... paving"); EXCAVATED/SPOIL stay coarse (fresh,
// un-compacted firm sublayer + loose clods); COMPACTED_BERM is deliberately packed -> smoother.
float state_grain_factor(int lbl) {
	if (lbl == 1) return 0.20;   // TREAD — compaction smooths
	if (lbl == 2) return 1.10;   // EXCAVATED — fresh coarse cut
	if (lbl == 3) return 1.15;   // SPOIL — loose dumped, grainiest
	if (lbl == 4) return 0.45;   // COMPACTED_BERM — packed smoother
	return 1.0;                  // VIRGIN — full natural grit
}

// 2-term Henyey-Greenstein single-particle phase function P(g) as a function of cos(phase
// angle). cg = cos(g), where g is the angle between the directions to the Sun and to the
// camera (g=0 at opposition / full backscatter). c>0 weights the BACKscattering lobe — lunar
// regolith is net backscattering at visible wavelengths (Sato et al. 2014 mare b,c). At g=0
// this returns >1 (the backscatter peak); normalization is carried by the w/4PI factor in r.
float hg_phase(float cg, float b, float c) {
	float b2 = b * b;
	float back = (1.0 - b2) / pow(max(1.0 - 2.0 * b * cg + b2, 1e-4), 1.5);
	float fwd  = (1.0 - b2) / pow(max(1.0 + 2.0 * b * cg + b2, 1e-4), 1.5);
	return 0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * fwd;
}

void fragment() {
	float dist = clamp(texture(disturbance_tex, field_uv).r, 0.0, 1.0);
	int lbl = state_label(field_uv);
	float dens = texture(density_tex, field_uv).r;
	float dens_n = clamp((dens - density_lo) / max(density_hi - density_lo, 1.0), 0.0, 1.0);
	float state_grain = state_grain_factor(lbl);   // per-state micro-texture amplitude (TREAD low)

	vec3 col = base_albedo;
	float rough = 0.96;  // dusty regolith: very rough, near-Lambertian (spec §8)

	// --- macro albedo mottle (#2): two octaves of value noise in REAL metres, applied
	// to every state incl. VIRGIN so undisturbed ground stops reading as a uniform slab.
	// field_uv -> metres via field_span_m, then / macro_scale_m -> mottle cells. ---
	{
		float msc = max(macro_scale_m, 1e-3);
		vec2 mp = field_uv * (field_span_m / msc);
		float m = 0.62 * vnoise(mp) + 0.38 * vnoise(mp * 3.3 + vec2(7.1, 2.7));  // [0,1]
		col *= (1.0 + macro_albedo_amp * (m * 2.0 - 1.0));                        // +/- amp
	}

	// disturbance darkens & roughens slightly (churned loose fines).
	col *= (1.0 - 0.18 * dist);

	if (lbl == 1) {
		// TREAD: compacted rut -> darker, smoother (paved). spec §6.
		col *= 0.82;
		rough = mix(rough, 0.80, 0.6);
	} else if (lbl == 2) {
		// EXCAVATED: fresh firm sublayer -> brighter + denser-looking. spec §6/§8.
		// When cut-depth albedo is active it OWNS the brightening (continuous, depth-graded,
		// applied below); otherwise keep the legacy flat tint so non-mantle scenes are unchanged.
		if (!cut_depth_enabled) {
			col = mix(col, base_albedo * 1.45, 0.65 + 0.35 * dens_n);
		}
		rough = mix(rough, 0.88, 0.5);
	} else if (lbl == 3) {
		// SPOIL: loose dumped pile -> slightly darker, very rough.
		col *= 0.90;
		rough = 0.98;
	} else if (lbl == 4) {
		// COMPACTED_BERM: packed mound -> a touch brighter and smoother.
		col = mix(col, base_albedo * 1.15, 0.5);
		rough = mix(rough, 0.85, 0.5);
	}
	// (lbl == 0 VIRGIN: untouched base look.)

	// --- fine granular ALBEDO speckle (world-anchored, persistent; Apollo-surface grit) ---
	// Peppered brightness variation at the granule scale so the ground reads as gritty regolith,
	// not a smooth/interpolated slab — and so passive stereo has matchable intensity texture.
	// field_uv->metres anchors it to the SURFACE, so both stereo cameras see identical speckle at
	// a world point (stereo-consistent) and it is STABLE across sun angles (matters for the
	// boulder-fiducial-across-the-lunar-day idea). Per-state amp (state_grain): TREAD smoothed.
	{
		vec2 sp = field_uv * (field_span_m / max(speckle_size_m, 1e-4));
		float spk = 0.6 * vnoise(sp) + 0.4 * vnoise(sp * 2.3 + vec2(19.7, 4.1));  // [0,1]
		spk = smoothstep(0.28, 0.72, spk);   // CRISP the flecks: distinct light/dark grains, not soft gradient
		col *= (1.0 + grain_albedo_amp * state_grain * (spk * 2.0 - 1.0));
	}

	// --- cut-depth / exposed-sublayer albedo (continuous; render_fidelity) ---
	// Removed thickness from the areal-mass deficit; brighten + slightly desaturate toward
	// fresh regolith, scaled by exposure. VIRGIN (mass_areal == M0) -> expo 0 -> no change.
	if (cut_depth_enabled) {
		float ma = texture(mass_areal_tex, field_uv).r;
		float removed_m = max(mantle_areal_m0 - ma, 0.0) / max(surface_density_cd, 1.0);
		float expo = clamp(removed_m / max(cut_depth_full_m, 1e-4), 0.0, 1.0);
		vec3 fresh = col * fresh_albedo_gain;                 // brighter fresh sublayer
		float g = dot(fresh, vec3(0.3333));
		fresh = mix(fresh, vec3(g), 0.18 * expo);             // ... and slightly less reddened
		col = mix(col, fresh, expo);
		rough = mix(rough, 0.90, 0.4 * expo);                 // fresh firm cut: a touch less rough
	}

	// =====================================================================
	// DETAIL NORMAL PASS (render_fidelity_spec.md §4.2). All ADDITIVE: with
	// neutral track_dir_tex and disturbance 0 the perturbation collapses to 0
	// and NORMAL is the smooth mesh normal (the v1.0.1 look).
	// =====================================================================

	// Tangent frame from screen-space derivatives (the mesh carries no TANGENT
	// array). VERTEX/NORMAL are VIEW-space in the fragment stage; we build a TBN
	// that maps a field-UV-space gradient into a view-space normal perturbation.
	vec3 dpdx = dFdx(VERTEX);
	vec3 dpdy = dFdy(VERTEX);
	vec2 duvdx = dFdx(field_uv);
	vec2 duvdy = dFdy(field_uv);
	float det = duvdx.x * duvdy.y - duvdy.x * duvdx.y;
	vec3 Nv = normalize(NORMAL);
	vec3 T = vec3(1.0, 0.0, 0.0);
	vec3 B = vec3(0.0, 1.0, 0.0);
	if (abs(det) > 1e-12) {
		float inv = 1.0 / det;
		T = normalize((dpdx * duvdy.y - dpdy * duvdx.y) * inv);  // d/du  (+col/+X)
		B = normalize((dpdy * duvdx.x - dpdx * duvdy.x) * inv);  // d/dv  (+row/+Z)
	}
	// Re-orthonormalize against the geometric normal (Gram-Schmidt). Keep B as the
	// derivative-derived bitangent (re-orthogonalized) rather than cross(N,T) so its
	// SIGN matches the +v/+row direction (a cross product would flip handedness for a
	// left-handed UV layout and mirror the v-gradient).
	T = normalize(T - Nv * dot(Nv, T));
	B = normalize(B - Nv * dot(Nv, B) - T * dot(T, B));

	// World metres per field-UV unit (square field assumed; §1 5.12 m).
	float m_per_uv = max(field_span_m, 1e-4);

	// Camera-distance fade (§4.2): VERTEX is view-space so its length ~ depth (m).
	float cam_dist = length(VERTEX);
	float fade = 1.0 - smoothstep(fade_near_m, fade_far_m, cam_dist);

	// gradient of total detail height in (u,v) space; accumulate contributions.
	vec2 grad_uv = vec2(0.0);

	// --- §4.2.2 base granularity (everywhere, x disturbance) ---
	// Grain coords: field-UV -> metres -> grains. Central-difference gradient.
	// Floor raised (#2) from 0.25 to 0.5 so VIRGIN/undisturbed ground still reads granular
	// (not glassy); disturbance pushes it to full strength.
	// Amplitude driven by the PER-STATE factor (state_grain) rather than disturbance alone, so
	// VIRGIN ground reads persistently granular and TREAD ruts read smooth (compaction); a small
	// disturbance term still adds extra churn on freshly worked cells.
	float grain_amp = grain_strength * state_grain * (0.85 + 0.15 * dist) * fade;
	if (grain_amp > 1e-4) {
		vec2 gp = field_uv * (m_per_uv / max(grain_size_m, 1e-4));
		float eps = 0.75;
		float hX = grain_height(gp + vec2(eps, 0.0)) - grain_height(gp - vec2(eps, 0.0));
		float hY = grain_height(gp + vec2(0.0, eps)) - grain_height(gp - vec2(0.0, eps));
		// d(grain coord)/d(uv) = m_per_uv/grain_size, fold into the amplitude.
		grad_uv += grain_amp * vec2(hX, hY);
	}

	// --- track-dir field (shared by cleats + teeth; §4.3) ---
	vec3 tdir = texture(track_dir_tex, field_uv).rgb;
	vec2 travel = tdir.xy * 2.0 - 1.0;     // unpack unit travel dir (x=+col, y=+row)
	float travel_len = length(travel);
	float phase = tdir.b;                  // arc-length phase [0,1]
	// transverse ridge axis = travel rotated 90deg; ridges run ACROSS travel.
	vec2 across = (travel_len > 1e-3) ? normalize(vec2(-travel.y, travel.x)) : vec2(0.0);

	// Field UV expressed in metres, for the periodic ridge phase. Cleats/teeth project
	// this onto the unit travel direction (dot(uv_m, tdir_uv)) so the ridge phase
	// advances ALONG travel (crests transverse) and combines with the baked B phase.
	vec2 uv_m = field_uv * m_per_uv;

	// --- §4.2.3 cleat marks where TREAD (lbl==1) ---
	// A 1-D sinusoidal ridge train whose phase advances ALONG the travel direction
	// (so the ridge crests are TRANSVERSE — across the wheel band). The baked B phase
	// keeps ridges continuous along the corridor across frames; along_m sharpens them
	// in-tile. The ridge HEIGHT gradient is d/d(along) = cos(...), and it points along
	// the travel direction, so the (u,v)-space gradient is ridge * unit-travel-in-uv.
	if (lbl == 1 && travel_len > 1e-3) {
		float cleat_amp = cleat_strength * (0.35 + 0.65 * dist) * fade;
		vec2 tdir_uv = travel / max(travel_len, 1e-4);      // unit travel in uv axes
		float s = (dot(uv_m, tdir_uv) + phase) / max(cleat_pitch_m, 1e-4);
		float ridge = cos(6.28318530718 * s) * cleat_amp;
		grad_uv += ridge * tdir_uv;
	}

	// --- §4.2.4 drum teeth where EXCAVATED(2) / SPOIL(3), with capped parallax ---
	// Same transverse-ridge construction as cleats, but deeper, plus a BEST-EFFORT
	// hand-rolled layered parallax (§4.2 note: a custom shader has NO POM keyword, so
	// we port the steep-parallax raymarch by hand). The teeth profile is 1-D ALONG the
	// travel direction; we march the view ray's along-travel component against that
	// profile to find an apparent along-offset, then phase the ridge by it. The layer
	// count is CAPPED (teeth_parallax_layers, hard-clamped <=16) and the whole effect
	// is gated by the camera fade so cost stays bounded and far tiles don't pay for it.
	if ((lbl == 2 || lbl == 3) && travel_len > 1e-3) {
		float teeth_amp = teeth_strength * (0.4 + 0.6 * dist) * fade;
		vec2 tdir_uv = travel / max(travel_len, 1e-4);   // unit travel in uv axes

		// Teeth height profile in [0,1] as a function of along-travel coordinate (m).
		// along0 = current fragment's along-travel position (m).
		float along0_m = dot(uv_m, tdir_uv);

		// Parallax: shift the along-coordinate toward the camera by up to
		// teeth_parallax_m of apparent depth. View vector is view-space (VERTEX is
		// view-space, camera at origin). Its component along the travel tangent (T,B
		// mapped to the uv travel dir) drives the lateral shift.
		vec3 Vv = normalize(-VERTEX);                    // toward camera (view space)
		float v_along = dot(Vv, T) * tdir_uv.x + dot(Vv, B) * tdir_uv.y;  // along-travel
		float ndotv = max(abs(dot(Vv, Nv)), 0.15);
		float max_shift_m = (teeth_parallax_m * fade) * (v_along / ndotv);

		float par_along_m = 0.0;
		int layers = clamp(teeth_parallax_layers, 1, 16);
		float layer_d = 1.0 / float(layers);
		float ray_h = 1.0;                               // ray height from 1 (surface) down
		float walked = 0.0;
		for (int i = 0; i < 16; i++) {
			if (i >= layers) break;
			float along_p = (along0_m + walked + phase) / max(teeth_pitch_m, 1e-4);
			float surf_h = 0.5 + 0.5 * sin(6.28318530718 * along_p);  // teeth profile
			if (ray_h <= surf_h) break;                  // ray hit the height field
			ray_h -= layer_d;
			walked += max_shift_m * layer_d;             // step along the parallax dir
			par_along_m = walked;
		}

		float s = (along0_m + par_along_m + phase) / max(teeth_pitch_m, 1e-4);
		float ridge = cos(6.28318530718 * s) * teeth_amp;
		grad_uv += ridge * tdir_uv;
	}

	// Map the (u,v) gradient into a view-space normal perturbation via the TBN.
	// A height gradient (gu,gv) tilts the normal by -(gu*T + gv*B).
	vec3 N = Nv;
	if (dot(grad_uv, grad_uv) > 1e-12) {
		vec3 pert = -(grad_uv.x * T + grad_uv.y * B);
		N = normalize(Nv + pert);
	}

	ALBEDO = col;
	ROUGHNESS = clamp(rough, 0.0, 1.0);
	METALLIC = 0.0;
	NORMAL = N;   // view-space perturbed normal (NORMAL output is view-space)
}

// =====================================================================
// LIGHT (render_fidelity): Hapke IMSA bidirectional reflectance, replacing Lambert.
// The headline fidelity term for the grazing polar sun the IPEx team cares about.
//
//   r(i,e,g) = (w/4PI) * mu0/(mu0+mu) * [ (1 + B(g))*P(g) + H(mu0)*H(mu) - 1 ]
//
//   mu0 = cos(incidence) = N.L,  mu = cos(emission) = N.V,  g = phase angle.
//   * mu0/(mu0+mu)  Lommel-Seeliger single-scattering geometry (Hapke 1981). THE headline
//                   change over Lambert: it cancels most of Lambert's limb/terminator
//                   darkening, so the surface reads as the near-uniform-brightness disk the
//                   Moon actually is, instead of a Lambert sphere driven to black at the
//                   grazing incidence of a ~5deg polar sun.
//   * P(g)          2-term Henyey-Greenstein single-particle phase fn; lunar mare is net
//                   backscattering (Sato et al. 2014 LROC b=0.26,c=0.08 @643 nm).
//   * B(g)          shadow-hiding opposition surge B0/(1 + tan(g/2)/h) — bright spike toward
//                   zero phase, Sun directly behind the camera (Hapke 2002).
//   * H(x)          Chandrasekhar isotropic multiple-scattering function, Hapke 1981 rational
//                   approximation (1+2x)/(1+2x*sqrt(1-w)); small on the dark Moon but included.
//   * w == ALBEDO   per-texel single-scattering albedo (chromatic, spatial: mottle/state/cut).
//
// NORMAL is the DETAIL-perturbed normal and ATTENUATION carries the real cast shadow, so both
// micro-relief and cast shadows feed the BRDF. The macroscopic-roughness shadowing term
// S(i,e,g;theta_bar) (Hapke 1984) is deliberately OMITTED — the detail-normal pass + real cast
// shadows already carry sub-mesh roughness shadowing geometrically; the statistical theta_bar
// correction is flagged in render_fidelity_spec as the next refinement. hapke_enabled=false
// falls back to Lambert so the same pipeline renders the A/B comparison.
// =====================================================================
void light() {
	// NB: Godot's light() processor forbids `return`, so this is structured as nested if/else.
	float mu0 = dot(NORMAL, LIGHT);
	if (mu0 > 0.0) {                                        // mu0<=0 => self-shadowed, no direct term
		if (!hapke_enabled) {                              // Lambert fallback (A/B baseline)
			DIFFUSE_LIGHT += ALBEDO * LIGHT_COLOR * ATTENUATION * mu0;
		} else {
			float mu = max(dot(NORMAL, VIEW), 1e-3);       // cos emission (camera side)
			float cg = clamp(dot(LIGHT, VIEW), -1.0, 1.0); // cos(phase angle); LIGHT,VIEW unit
			float g  = acos(cg);
			float P  = hg_phase(cg, hapke_b, hapke_c);                             // phase function
			float B  = hapke_B0 / (1.0 + tan(0.5 * g) / max(hapke_h, 1e-4));       // SHOE surge
			vec3  w  = clamp(ALBEDO, vec3(0.0), vec3(0.999));                      // single-scatter albedo
			vec3  gamma = sqrt(1.0 - w);
			vec3  Hi = (1.0 + 2.0 * mu0) / (1.0 + 2.0 * mu0 * gamma);              // H(mu0)
			vec3  He = (1.0 + 2.0 * mu)  / (1.0 + 2.0 * mu  * gamma);              // H(mu)
			float ls = mu0 / (mu0 + mu);                                          // Lommel-Seeliger core
			// w/(4PI), PI folded into Godot's diffuse-light scale (LIGHT_COLOR carries energy) -> w*0.25.
			vec3  r  = (w * 0.25) * ls * ((1.0 + B) * P + Hi * He - 1.0);
			DIFFUSE_LIGHT += LIGHT_COLOR * ATTENUATION * max(r, vec3(0.0)) * hapke_gain;
		}
	}
}
