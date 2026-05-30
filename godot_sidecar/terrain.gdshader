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
// roughness — a Lambertian-ish dusty surface (spec §8).

render_mode specular_disabled, diffuse_lambert;

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
uniform vec3 base_albedo : source_color = vec3(0.46, 0.43, 0.40);

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

// --- detail pass tunables (render_fidelity_spec.md §4.2/§8 open params) ----------
// World metres spanned by the field UV (0..1) — terrain.gd sets it from grid extent
// so the in-shader noise/cleat periods are in real metres regardless of field size.
uniform float field_span_m = 5.12;
// Base granularity (§4.2.2): grain size and bump strength. ~8 mm grains read at the
// mesh scale; strength is the max normal tilt (radians-ish) scaled by disturbance.
uniform float grain_size_m = 0.008;
uniform float grain_strength = 0.55;
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

// --- cheap hash / value noise (no texture asset; render_fidelity_spec.md §4.2.2) --
float hash21(vec2 p) {
	p = fract(p * vec2(123.34, 345.45));
	p += dot(p, p + 34.345);
	return fract(p.x * p.y);
}

// Smooth value noise in [0,1].
float vnoise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	vec2 u = f * f * (3.0 - 2.0 * f);
	float a = hash21(i + vec2(0.0, 0.0));
	float b = hash21(i + vec2(1.0, 0.0));
	float c = hash21(i + vec2(0.0, 1.0));
	float d = hash21(i + vec2(1.0, 1.0));
	return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// 2-octave granular height in [0,1] at a point scaled into "grain space".
float grain_height(vec2 gp) {
	return 0.65 * vnoise(gp) + 0.35 * vnoise(gp * 2.7 + 11.3);
}

void fragment() {
	float dist = clamp(texture(disturbance_tex, field_uv).r, 0.0, 1.0);
	int lbl = state_label(field_uv);
	float dens = texture(density_tex, field_uv).r;
	float dens_n = clamp((dens - density_lo) / max(density_hi - density_lo, 1.0), 0.0, 1.0);

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
	float grain_amp = grain_strength * (0.5 + 0.5 * dist) * fade;
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
