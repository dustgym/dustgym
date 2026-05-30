shader_type spatial;
// Lunar regolith terrain shader (spec §6 state-driven appearance, §8 lighting).
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
// Lunar surface: no atmospheric scatter, near-black shadows, high specular-less
// roughness — a Lambertian-ish dusty surface (spec §8).

render_mode specular_disabled, diffuse_lambert;

uniform sampler2D disturbance_tex : source_color, filter_linear;  // R = [0,1]
uniform sampler2D state_tex : filter_nearest;                     // R = label/255
uniform sampler2D density_tex : filter_linear;                    // R = kg/m^3 (raw)

uniform float density_lo = 1170.0;
uniform float density_hi = 1456.0;

// Base regolith albedo (mare-ish mid grey; spec §8 dusty surface).
uniform vec3 base_albedo : source_color = vec3(0.46, 0.43, 0.40);

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

void fragment() {
	float dist = clamp(texture(disturbance_tex, field_uv).r, 0.0, 1.0);
	int lbl = state_label(field_uv);
	float dens = texture(density_tex, field_uv).r;
	float dens_n = clamp((dens - density_lo) / max(density_hi - density_lo, 1.0), 0.0, 1.0);

	vec3 col = base_albedo;
	float rough = 0.96;  // dusty regolith: very rough, near-Lambertian (spec §8)

	// disturbance darkens & roughens slightly (churned loose fines).
	col *= (1.0 - 0.18 * dist);

	if (lbl == 1) {
		// TREAD: compacted rut -> darker, smoother (paved). spec §6.
		col *= 0.82;
		rough = mix(rough, 0.80, 0.6);
	} else if (lbl == 2) {
		// EXCAVATED: fresh firm sublayer -> brighter + denser-looking. spec §6/§8.
		// Brighten and tint toward the denser-material look; modulate by density.
		col = mix(col, base_albedo * 1.45, 0.65 + 0.35 * dens_n);
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

	ALBEDO = col;
	ROUGHNESS = clamp(rough, 0.0, 1.0);
	METALLIC = 0.0;
}
