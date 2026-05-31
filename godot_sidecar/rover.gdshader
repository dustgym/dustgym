shader_type spatial;
// Worn rover surface (John: "the robot won't be this textureless — dents, scratches, dust").
// The flat grey StandardMaterial read as pristine CG AND, being textureless, fed the passive-stereo
// scanline streaks when the rover was in-frame. This breaks it up with an OBJECT-space procedural
// wear pattern: dust/grime darkening in patches, bright scratch streaks, and roughness/metallic
// variation. NOT the Hapke regolith shader — the rover is METAL, so it uses Godot's default PBR
// lighting (same DirectionalLight + shadows as the scene). Object-space noise => the wear is fixed to
// each part (stable across frames/sun angles), and gives stereo some intensity texture to match.

// NB: the rover uses Godot's default PBR (it's metal, not Hapke regolith), lit by the same
// energy-3.0 sun that is CALIBRATED for the Hapke terrain — so a normal-grey albedo overexposes
// to blown white and the wear washes out. Keep the base LOW (~0.18) so the lit metal sits near the
// terrain mid-tone and the dust/scratch wear actually reads.
uniform vec3 base_color : source_color = vec3(0.18, 0.18, 0.20);
uniform float base_metallic = 0.22;    // lower -> less mirror-blowout under the bright sun (wear reads)
uniform float base_roughness = 0.72;   // matte worn metal, not glossy CG
uniform float wear_scale_m = 0.05;     // object-space noise cell (~5 cm grime patches)
uniform float dust_amount = 0.42;      // darkening from low-lying dust/grime
uniform float scratch_amount = 0.26;   // bright bare-metal scratch streaks
uniform float scratch_freq = 4.3;      // finer scale for the scratches relative to grime

varying vec3 v_obj;   // object-space position (per rover part) for a STABLE wear pattern

float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.yzx + 33.33);
	return fract((p.x + p.y) * p.z);
}

float vnoise3(vec3 p) {
	vec3 i = floor(p);
	vec3 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	float n000 = hash13(i),               n100 = hash13(i + vec3(1, 0, 0));
	float n010 = hash13(i + vec3(0, 1, 0)), n110 = hash13(i + vec3(1, 1, 0));
	float n001 = hash13(i + vec3(0, 0, 1)), n101 = hash13(i + vec3(1, 0, 1));
	float n011 = hash13(i + vec3(0, 1, 1)), n111 = hash13(i + vec3(1, 1, 1));
	return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
			   mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}

void vertex() {
	v_obj = VERTEX;   // object space (per part); wear is fixed to the hardware
}

void fragment() {
	vec3 p = v_obj / max(wear_scale_m, 1e-4);
	float grime = vnoise3(p);                          // [0,1] coarse dust/grime
	float fine = vnoise3(p * scratch_freq + vec3(7.1, 2.3, 4.7));

	vec3 col = base_color;
	col *= (1.0 - dust_amount * (1.0 - grime) * 0.7);  // dust darkens recessed/grimy patches
	float scratch = smoothstep(0.80, 0.93, fine);      // thin bright bare-metal streaks
	col = mix(col, vec3(0.82, 0.82, 0.85), scratch_amount * scratch);

	ALBEDO = col;
	METALLIC = clamp(base_metallic * (1.0 - 0.45 * (1.0 - grime)), 0.0, 1.0);   // dusty => less metallic
	ROUGHNESS = clamp(base_roughness + 0.25 * (1.0 - grime) - 0.18 * scratch, 0.15, 1.0);
}
