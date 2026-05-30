shader_type spatial;
// Layer 1: heightmap false-color (unlit). A diagnostic view of the raw
// elevation field — the authoritative geometry (INTERFACE.md §4). Unlit so
// the ramp reads cleanly regardless of the grazing lunar sun (spec §8).

render_mode unshaded;

uniform sampler2D height_tex : filter_linear;
uniform float h_lo = -0.2;
uniform float h_hi = 0.2;

varying vec2 field_uv;

void vertex() { field_uv = UV; }

// Simple perceptual-ish blue->cyan->green->yellow->red ramp.
vec3 ramp(float t) {
	t = clamp(t, 0.0, 1.0);
	vec3 c0 = vec3(0.03, 0.05, 0.35); // low  (deep)
	vec3 c1 = vec3(0.00, 0.55, 0.65); // cyan
	vec3 c2 = vec3(0.20, 0.70, 0.25); // green
	vec3 c3 = vec3(0.95, 0.85, 0.20); // yellow
	vec3 c4 = vec3(0.85, 0.18, 0.12); // high (rim)
	if (t < 0.25)      return mix(c0, c1, t / 0.25);
	else if (t < 0.5)  return mix(c1, c2, (t - 0.25) / 0.25);
	else if (t < 0.75) return mix(c2, c3, (t - 0.5) / 0.25);
	else               return mix(c3, c4, (t - 0.75) / 0.25);
}

void fragment() {
	float h = texture(height_tex, field_uv).r;
	float t = (h - h_lo) / max(h_hi - h_lo, 1e-4);
	ALBEDO = ramp(t);
}
