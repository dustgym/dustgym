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
uniform vec3 base_albedo : source_color = vec3(0.42, 0.39, 0.37);
uniform float lod_step_m = 0.08;                   // world meters per low-res texel

void vertex() {
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
	ALBEDO = base_albedo;
	ROUGHNESS = 0.97;
	METALLIC = 0.0;
}
