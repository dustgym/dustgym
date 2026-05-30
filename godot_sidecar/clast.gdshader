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

uniform vec3 base_albedo : source_color = vec3(0.55, 0.52, 0.49);  // rock single-scatter albedo w
uniform float albedo_jitter = 0.12;     // +/- per-instance brightness spread (anti-uniformity)
uniform float clast_roughness = 0.93;

// Hapke BRDF (mirror of terrain.gdshader; see that file's light() block for the full equation +
// sources: Sato et al. 2014 b,c; Hapke 2002 opposition; Hapke 1981/2012 IMSA).
uniform bool  hapke_enabled = true;
uniform float hapke_b  = 0.26;
uniform float hapke_c  = 0.08;
uniform float hapke_B0 = 1.0;
uniform float hapke_h  = 0.06;
uniform float hapke_gain = 1.4;

varying float v_jit;

// Cheap 3D hash -> [0,1], for the per-instance albedo jitter.
float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.yzx + 33.33);
	return fract((p.x + p.y) * p.z);
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
}

void fragment() {
	ALBEDO = clamp(base_albedo * v_jit, 0.0, 1.0);
	ROUGHNESS = clast_roughness;
	METALLIC = 0.0;
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
