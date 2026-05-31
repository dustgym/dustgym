shader_type spatial;
// Clast (boulder) shader — the SAME Hapke / Lommel-Seeliger BRDF as the terrain (render_fidelity),
// so the rock clasts and the regolith read as ONE airless surface under the grazing sun instead of
// the boulders blowing out to white Lambert blobs next to the photometrically-correct ground.
// Clasts render as a SphereMesh MultiMesh (sidecar.gd _build_clasts); a per-instance albedo jitter
// (hashed from each instance's world position) breaks the CG-uniform look of 143 identical spheres.
// Rock is a touch brighter / rougher than the fines (single-scatter albedo w ~0.5 vs ~0.43), but is
// still backscattering airless regolith-coated rock, so the same phase fn / opposition / L-S apply.
//
// hapke_enabled=false -> Lambert fallback, so the --brdf lambert A/B baseline still shows the old
// (blown-white) clast look on its side of the comparison.
render_mode specular_disabled;

// Rock single-scatter albedo w. NEUTRAL-grey to MATCH the regolith tint (terrain base_albedo
// 0.45,0.44,0.43): lunar rocks don't stand out by COLOR — they read as grey blocks distinguished by
// shape/shadow/brightness, not hue. Same faint-warm-grey ratio as the fines (R/B~1.04), just
// marginally brighter (less-weathered block); was a warmer 0.55,0.52,0.49 (R/B~1.12, rendered brown).
uniform vec3 base_albedo : source_color = vec3(0.50, 0.49, 0.48);  // rock w, grey to match the fines
uniform float albedo_jitter = 0.12;     // +/- per-instance brightness spread (anti-uniformity)
uniform float clast_roughness = 0.93;

// --- PROCGEN SHAPE / RELIEF (render-fidelity; pure appearance, the physics contract is unchanged) ---
// Today every clast renders as an identical smooth SphereMesh. These add object-space relief so the
// boulders read as fractured rock, NOT CG spheres, while the physics-chosen diameter (Golombek SFD)
// is preserved: sidecar.gd already applies a TRIAXIAL non-uniform scale renormalized to geo-mean 1.0
// for the silhouette; THIS shader adds FACETED relief + a high-freq fracture detail-normal.
//
// ANGULARITY (spec §9 "Angular, minimally eroded — No water/wind rounding -> sharp grains"): lunar
// clasts are fractured/faceted rock, NOT water-rounded river pebbles. A smooth fbm bulge reads as a
// dome/bun (geologically wrong). So the low-freq displacement here is deliberately made ANGULAR two
// ways: (1) RIDGED noise — fold fbm through 1-|2v-1| so its crests become sharp arêtes instead of
// rounded humps; (2) TERRACING — quantize the displacement into a few discrete levels so the coarse
// (24x16) mesh resolves them as flat faces meeting at sharp edges (conchoidal-fracture facets). This
// also makes each boulder visually DISTINCTIVE (later fiducial track) where smooth domes would not.
//   surf_amp  : low-freq faceted displacement along the object normal, as a fraction of the unit radius
//               (bulges/cuts for an angular-but-rocklike silhouette). surf_freq sets facet spatial freq.
//   facet_levels : number of discrete terrace steps the displacement is quantized into (sharp edges).
//   ridge_mix    : 0=smooth fbm, 1=fully ridged (sharp crests). Blends rounded vs angular character.
//   void_amp  : OPTIONAL subtractive concavity ("void"/spall scar) — thresholded low-freq noise carved
//               INWARD. Conservative default; per-instance gated by INSTANCE_CUSTOM.z so only some
//               boulders show a scar. Kept small so buried_frac / resting geometry stay sane.
//   detail_amp: strength of the FRAGMENT-stage high-freq fracture detail-normal (faded with distance).
uniform float surf_amp   = 0.15;
uniform float surf_freq  = 1.7;
uniform float facet_levels = 4.0;   // discrete terrace steps in the low-freq displacement (faceting)
uniform float ridge_mix    = 0.8;   // 0=rounded fbm crests, 1=sharp ridged crests (angular arêtes)
uniform float void_amp   = 0.10;
uniform float void_freq  = 1.1;
uniform float detail_amp = 0.6;
uniform float detail_freq = 14.0;   // fracture micro-relief frequency (object space)
// Detail-normal camera-distance fade (mirror of terrain.gdshader): full strength within fade_near_m,
// gone by fade_far_m, so distant boulders don't shimmer under the grazing sun.
uniform float fade_near_m = 3.0;
uniform float fade_far_m  = 20.0;

// Hapke BRDF (mirror of terrain.gdshader; see that file's light() block for the full equation +
// sources: Sato et al. 2014 b,c; Hapke 2002 opposition; Hapke 1981/2012 IMSA).
uniform bool  hapke_enabled = true;
uniform float hapke_b  = 0.26;
uniform float hapke_c  = 0.08;
uniform float hapke_B0 = 1.0;
uniform float hapke_h  = 0.06;
uniform float hapke_gain = 1.4;

varying float v_jit;
varying vec3  v_objpos;   // object-space position (pre-scale) for the fragment detail-normal pass
varying float v_seed;     // stable per-instance seed (INSTANCE_CUSTOM.x) so noise doesn't swim

// Cheap 3D hash -> [0,1], for the per-instance albedo jitter.
float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.yzx + 33.33);
	return fract((p.x + p.y) * p.z);
}

// --- 3D value-noise kit (object space) — the 3D sibling of terrain.gdshader's hash21/vnoise/grain.
// hash31 -> [0,1]; vnoise3 trilinearly interpolates 8 lattice corners (smoothstepped); fbm3 sums
// octaves. Used for the vertex lump/void displacement and the fragment fracture detail-normal.
float hash31(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

float vnoise3(vec3 p) {
	vec3 i = floor(p);
	vec3 f = fract(p);
	vec3 u = f * f * (3.0 - 2.0 * f);
	float c000 = hash31(i + vec3(0.0, 0.0, 0.0));
	float c100 = hash31(i + vec3(1.0, 0.0, 0.0));
	float c010 = hash31(i + vec3(0.0, 1.0, 0.0));
	float c110 = hash31(i + vec3(1.0, 1.0, 0.0));
	float c001 = hash31(i + vec3(0.0, 0.0, 1.0));
	float c101 = hash31(i + vec3(1.0, 0.0, 1.0));
	float c011 = hash31(i + vec3(0.0, 1.0, 1.0));
	float c111 = hash31(i + vec3(1.0, 1.0, 1.0));
	float x00 = mix(c000, c100, u.x);
	float x10 = mix(c010, c110, u.x);
	float x01 = mix(c001, c101, u.x);
	float x11 = mix(c011, c111, u.x);
	float y0 = mix(x00, x10, u.y);
	float y1 = mix(x01, x11, u.y);
	return mix(y0, y1, u.z);
}

// 3-octave fractal value noise in [0,1] (matches terrain's 2-octave grain_height in spirit).
float fbm3(vec3 p) {
	float a = 0.0;
	float w = 0.5;
	float wsum = 0.0;
	for (int i = 0; i < 3; i++) {
		a += w * vnoise3(p);
		wsum += w;
		p = p * 2.03 + vec3(11.3, 7.1, 19.7);
		w *= 0.5;
	}
	return a / max(wsum, 1e-4);
}

// RIDGED fractal noise in [0,1]: each octave is folded through 1-|2v-1| so the noise CRESTS become
// sharp creases (V-shaped) instead of smooth domes, and squared to sharpen further. This is the
// angular-rock primitive (spec §9): ridges read as fracture arêtes, valleys as cleavage planes.
// Cheaper-friendly: reuses vnoise3, so no extra hash machinery vs fbm3.
float ridged3(vec3 p) {
	float a = 0.0;
	float w = 0.5;
	float wsum = 0.0;
	for (int i = 0; i < 3; i++) {
		float v = vnoise3(p);
		float r = 1.0 - abs(2.0 * v - 1.0);   // fold: crest at v=0.5 -> sharp ridge line
		r = r * r;                            // square -> crisper crease, flatter flanks
		a += w * r;
		wsum += w;
		p = p * 2.03 + vec3(5.7, 13.9, 3.3);
		w *= 0.5;
	}
	return a / max(wsum, 1e-4);
}

// 2-term Henyey-Greenstein phase function P(g) vs cos(phase angle) (mirror of terrain.gdshader).
float hg_phase(float cg, float b, float c) {
	float b2 = b * b;
	float back = (1.0 - b2) / pow(max(1.0 - 2.0 * b * cg + b2, 1e-4), 1.5);
	float fwd  = (1.0 - b2) / pow(max(1.0 + 2.0 * b * cg + b2, 1e-4), 1.5);
	return 0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * fwd;
}

void vertex() {
	// Per-instance brightness jitter from the instance world-space origin (MODEL_MATRIX column 3).
	// MultiMesh supplies a distinct MODEL_MATRIX per instance, so each boulder differs slightly.
	vec3 inst = MODEL_MATRIX[3].xyz;
	v_jit = 1.0 + albedo_jitter * (hash13(inst) * 2.0 - 1.0);

	// Per-instance seed from sidecar.gd's set_instance_custom_data (Color(seed, elong, void_i, 0)).
	// MUST be seeded from INSTANCE_CUSTOM (a stable per-instance constant), NOT from VERTEX — keying
	// the noise on VERTEX would make every boulder identical AND make the relief swim with the mesh.
	float seed = INSTANCE_CUSTOM.x;
	float void_i = INSTANCE_CUSTOM.z;     // 0/1-ish per-instance gate for the optional concavity scar
	vec3 sd = vec3(seed * 13.7, seed * 7.3 + 4.1, seed * 19.1 + 9.7);
	v_seed = seed;
	v_objpos = VERTEX;                    // pre-scale unit-sphere coords for the fragment detail pass

	vec3 n = normalize(NORMAL);

	// --- LOW-FREQ FACETED DISPLACEMENT (silhouette variety + ANGULARITY, spec §9). Displace along
	// the object normal, but build the height field so it reads as fractured rock, not a smooth dome:
	//   1) RIDGED noise (ridged3) gives sharp crests; blend with smooth fbm by ridge_mix so we keep
	//      some rounded mass but get angular arêtes where ridge_mix is high.
	//   2) TERRACE/QUANTIZE the result into facet_levels discrete steps -> flat faces separated by
	//      sharp edges that the coarse (24x16) mesh resolves as conchoidal-fracture facets.
	// Unit sphere -> displacement is a fraction of the unit radius; the per-instance triaxial scale
	// (sidecar.gd) then stretches it. ---
	float smooth_h = fbm3(VERTEX * surf_freq + sd);             // [0,1] rounded
	float ridge_h  = ridged3(VERTEX * surf_freq + sd);          // [0,1] sharp crests
	float h = mix(smooth_h, ridge_h, clamp(ridge_mix, 0.0, 1.0));
	// Quantize to discrete terraces (faces) — floor to the nearest of facet_levels steps. The fract
	// part is dropped, so adjacent vertices that land in different bins create a sharp step (edge).
	float L = max(facet_levels, 1.0);
	float terr = floor(h * L) / L;                              // [0, (L-1)/L] flat faces
	float lump = terr - 0.5;                                    // zero-centred so mass is preserved
	VERTEX += n * (lump * surf_amp);

	// --- OPTIONAL SUBTRACTIVE VOID / concavity (spall scar). Threshold a low-freq fbm and carve
	// INWARD only where it exceeds the threshold, gated per-instance by void_i so only some clasts
	// show it. Conservative amplitude keeps the boulder convex enough that buried_frac stays sane. ---
	if (void_amp > 1e-4 && void_i > 0.5) {
		float vn = fbm3(VERTEX * void_freq + sd * 1.7 + 23.0);
		float carve = smoothstep(0.62, 0.85, vn);       // only the highest lobes carve
		VERTEX -= n * (carve * void_amp);
	}
}

void fragment() {
	ALBEDO = clamp(base_albedo * v_jit, 0.0, 1.0);
	ROUGHNESS = clast_roughness;
	METALLIC = 0.0;

	// --- HIGH-FREQ FRACTURE DETAIL-NORMAL (render_fidelity; mirrors terrain.gdshader §4.2). The
	// SphereMesh is smooth at the ~cm clast scale; this perturbs the normal with a high-freq fbm so
	// the rock reads as fractured/angular under the grazing sun, NOT glassy. The height field is
	// sampled in OBJECT space (v_objpos, seeded per-instance) so it's stable per boulder; the tilt is
	// applied in VIEW space via screen-space derivatives + a TBN, exactly as terrain.gdshader does.
	//   Camera-distance fade (VERTEX is view-space => its length ~ depth in m) kills far-tile shimmer.
	float cam_dist = length(VERTEX);
	float fade = 1.0 - smoothstep(fade_near_m, fade_far_m, cam_dist);
	vec3 N = normalize(NORMAL);
	if (detail_amp > 1e-4 && fade > 1e-3) {
		// Object-space fracture height in [0,1], keyed per-instance so it doesn't swim. RIDGED (not
		// smooth fbm) so the micro-relief is sharp/angular (spec §9 "sharp grains"), not soft dimples;
		// gamma-contrast (pow) deepens the creases for unmistakable faceting under the grazing sun.
		vec3 hp = v_objpos * detail_freq + vec3(v_seed * 13.7, v_seed * 7.3 + 4.1, v_seed * 19.1 + 9.7);
		float h = pow(ridged3(hp), 1.6);
		// Screen-space gradient of the height -> tilt in view space, with a TBN built (like terrain)
		// from dFdx/dFdy(VERTEX). VERTEX/NORMAL are VIEW-space in the fragment stage.
		vec3 dpdx = dFdx(VERTEX);
		vec3 dpdy = dFdy(VERTEX);
		float hX = dFdx(h);
		float hY = dFdy(h);
		// Tangent basis from the surface derivatives, re-orthonormalized (Gram-Schmidt) against the
		// geometric normal, then B against N and T — exactly terrain.gdshader's construction.
		vec3 T = normalize(dpdx - N * dot(N, dpdx));
		vec3 Braw = dpdy - N * dot(N, dpdy);
		vec3 B = normalize(Braw - T * dot(T, Braw));
		// Height gradient (hX,hY) tilts the normal by -(hX*T + hY*B), scaled by amp x distance fade.
		vec3 pert = -(hX * T + hY * B) * (detail_amp * fade);
		if (dot(pert, pert) > 1e-12) {
			N = normalize(N + pert);
		}
	}
	NORMAL = N;   // view-space perturbed normal (NORMAL output is view-space)
}

// Hapke IMSA bidirectional reflectance — identical to terrain.gdshader's light() (the SphereMesh
// supplies proper per-fragment normals, so each boulder gets the L-S limb behaviour + backscatter).
// Godot's light() processor forbids `return` -> nested if/else.
void light() {
	float mu0 = dot(NORMAL, LIGHT);
	if (mu0 > 0.0) {
		if (!hapke_enabled) {
			DIFFUSE_LIGHT += ALBEDO * LIGHT_COLOR * ATTENUATION * mu0;
		} else {
			float mu = max(dot(NORMAL, VIEW), 1e-3);
			float cg = clamp(dot(LIGHT, VIEW), -1.0, 1.0);
			float g  = acos(cg);
			float P  = hg_phase(cg, hapke_b, hapke_c);
			float B  = hapke_B0 / (1.0 + tan(0.5 * g) / max(hapke_h, 1e-4));
			vec3  w  = clamp(ALBEDO, vec3(0.0), vec3(0.999));
			vec3  gamma = sqrt(1.0 - w);
			vec3  Hi = (1.0 + 2.0 * mu0) / (1.0 + 2.0 * mu0 * gamma);
			vec3  He = (1.0 + 2.0 * mu)  / (1.0 + 2.0 * mu  * gamma);
			float ls = mu0 / (mu0 + mu);
			vec3  r  = (w * 0.25) * ls * ((1.0 + B) * P + Hi * He - 1.0);
			DIFFUSE_LIGHT += LIGHT_COLOR * ATTENUATION * max(r, vec3(0.0)) * hapke_gain;
		}
	}
}
