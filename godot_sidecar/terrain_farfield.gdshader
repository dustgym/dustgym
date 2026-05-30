shader_type spatial;
// FAR-FIELD LOD terrain (spec §4 "Far field: render only, no physics ...
// procedural heightfield, 10+ cm with shader detail"; efficiency note).
//
// The D2 LOD demonstration: a SINGLE low-subdivision PlaneMesh whose vertices
// are displaced *in the vertex shader* by sampling a decimated heightmap tile.
// This is how the big inactive quadtree regions render cheaply — no per-vertex
// CPU mesh, no fine subdivision, just one plane + a cheap vertex displace.
// Contrast with terrain.gdshader, used for the fine ACTIVE zone.

render_mode specular_disabled, diffuse_lambert;

uniform sampler2D height_lowres : filter_linear;   // R = elevation (m), decimated
// Aligned with terrain.gdshader's base_albedo (#2) so the fine ACTIVE window and this
// far-field plane don't show a brightness seam where they meet.
uniform vec3 base_albedo : source_color = vec3(0.46, 0.43, 0.40);
uniform float lod_step_m = 0.08;                   // world meters per low-res texel

// Macro albedo mottle (render_fidelity #2), mirrored from terrain.gdshader: the far-field
// plane FILLS the fly-through frame (the fine active mesh is only a sub-metre window around
// the rover), so the "textureless" grey was mostly THIS shader. Low-freq value mottle keyed
// to real metres so undisturbed far terrain reads weathered, not as a flat slab. Pure
// appearance. (GDShader has no shared include here, so hash/vnoise are duplicated.)
uniform float macro_albedo_amp = 0.13;
uniform float macro_scale_m = 0.42;

// State + disturbance at full field res (render_fidelity track fix): the far-field mesh is
// coarse, so the rover rut was rendering as undersampled geometric dimples. Reading the state
// label here makes the track a continuous TREAD albedo band that does NOT depend on mesh
// density (mirror of terrain.gdshader §6 state look, minus the in-shader detail-normal pass).
uniform sampler2D state_tex : filter_nearest;          // R = label/255 (0..4)
uniform sampler2D disturbance_tex : filter_linear;     // R = [0,1]

varying vec2 fuv;   // UV (0..1 across the field) carried to the fragment stage

// --- cheap hash / value noise (verbatim mirror of terrain.gdshader §4.2.2) ---
float hash21(vec2 p) {
	p = fract(p * vec2(123.34, 345.45));
	p += dot(p, p + 34.345);
	return fract(p.x * p.y);
}
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

void vertex() {
	fuv = UV;
	// PlaneMesh lies in XZ; UV runs 0..1. Sample the low-res tile and push Y up.
	float h = texture(height_lowres, UV).r;
	VERTEX.y += h;

	// Recompute the NORMAL from the heightmap gradient (central differences).
	// PlaneMesh ships flat (all-up) normals; without this the displaced far field
	// shades flat-grey and its relief vanishes (the boulder field sits on far-field
	// terrain, so this is what makes that render read as terrain, not a slab).
	vec2 ts = 1.0 / vec2(textureSize(height_lowres, 0));
	float hL = texture(height_lowres, UV - vec2(ts.x, 0.0)).r;
	float hR = texture(height_lowres, UV + vec2(ts.x, 0.0)).r;
	float hD = texture(height_lowres, UV - vec2(0.0, ts.y)).r;
	float hU = texture(height_lowres, UV + vec2(0.0, ts.y)).r;
	NORMAL = normalize(vec3(hL - hR, 2.0 * lod_step_m, hD - hU));
}

void fragment() {
	// Macro albedo mottle (#2): field span = lowres_width * lod_step_m == world extent.
	float span = float(textureSize(height_lowres, 0).x) * lod_step_m;
	vec2 mp = fuv * (span / max(macro_scale_m, 1e-3));
	float m = 0.62 * vnoise(mp) + 0.38 * vnoise(mp * 3.3 + vec2(7.1, 2.7));   // [0,1]
	vec3 col = base_albedo * (1.0 + macro_albedo_amp * (m * 2.0 - 1.0));

	// State-driven track look (mirror of terrain.gdshader §6), so the rut reads as a
	// continuous band on the coarse far field. disturbance darkens churned fines; the label
	// tints the band. This is what makes the track a track and not a row of shadowed pits.
	float dist = clamp(texture(disturbance_tex, fuv).r, 0.0, 1.0);
	col *= (1.0 - 0.18 * dist);
	float rough = 0.97;
	int lbl = int(round(texture(state_tex, fuv).r * 255.0));
	if (lbl == 1) {            // TREAD: compacted rut -> darker, a touch smoother
		col *= 0.82;
		rough = 0.90;
	} else if (lbl == 2) {     // EXCAVATED: fresh firm sublayer -> brighter
		col *= 1.30;
		rough = 0.92;
	} else if (lbl == 3) {     // SPOIL: loose pile -> slightly darker, very rough
		col *= 0.90;
		rough = 0.98;
	} else if (lbl == 4) {     // COMPACTED_BERM: packed mound -> a touch brighter
		col *= 1.12;
		rough = 0.92;
	}

	ALBEDO = col;
	ROUGHNESS = rough;
	METALLIC = 0.0;
}
