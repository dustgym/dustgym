shader_type spatial;
// Layer 2: state_label false-color (unlit). Discrete class map over the
// continuous state (spec §6 "Discrete class over continuous state").
// enum (INTERFACE.md §4): 0 VIRGIN, 1 TREAD, 2 EXCAVATED, 3 SPOIL, 4 COMPACTED_BERM.

render_mode unshaded;

uniform sampler2D state_tex : filter_nearest;  // R = label/255

varying vec2 field_uv;

void vertex() { field_uv = UV; }

void fragment() {
	int lbl = int(round(texture(state_tex, field_uv).r * 255.0));
	vec3 c = vec3(0.30, 0.30, 0.32);            // 0 VIRGIN  : neutral grey
	if (lbl == 1)      c = vec3(0.20, 0.45, 0.90); // TREAD          : blue
	else if (lbl == 2) c = vec3(0.95, 0.78, 0.10); // EXCAVATED      : amber
	else if (lbl == 3) c = vec3(0.85, 0.35, 0.15); // SPOIL          : orange-red
	else if (lbl == 4) c = vec3(0.35, 0.80, 0.40); // COMPACTED_BERM : green
	ALBEDO = c;
}
