shader_type canvas_item;
// Layer 6 (STRETCH): camera-intrinsics lens distortion post-process stub.
//
// Spec §8 / §"Camera intrinsics / dirty lens": Godot's camera is FOV-based;
// real sensor models set a custom projection matrix and apply Brown-Conrady
// (radial-tangential) distortion as a POST-PROCESS shader (CARLA gives this
// natively; here it's a few hundred lines you own).
//
// This is a *visible barrel-distortion stub*: radial terms k1,k2 only, applied
// to the screen texture. Real per-camera intrinsics calibration (fx,fy,cx,cy,
// p1,p2 tangential, distortion-center offset, undistort LUT) is FUTURE WORK —
// this proves the post chain hook-point, not a calibrated lens.

// Sample the rendered 3D frame from the back buffer (needs a BackBufferCopy
// node upstream; the sidecar adds one). This is what makes the post-process
// operate on the scene, not on the full-screen quad's own texture.
uniform sampler2D screen_tex : hint_screen_texture, filter_linear;

uniform float k1 = 0.35;   // primary radial coeff (positive -> barrel)
uniform float k2 = 0.10;   // secondary radial coeff
uniform vec2  center = vec2(0.5, 0.5);

void fragment() {
	vec2 uv = SCREEN_UV;
	vec2 d = uv - center;
	float r2 = dot(d, d);
	// Brown-Conrady radial-only forward warp: x_d = x(1 + k1 r^2 + k2 r^4).
	float f = 1.0 + k1 * r2 + k2 * r2 * r2;
	vec2 src = center + d * f;
	if (src.x < 0.0 || src.x > 1.0 || src.y < 0.0 || src.y > 1.0) {
		COLOR = vec4(0.0, 0.0, 0.0, 1.0);  // black border outside the lens circle
	} else {
		COLOR = texture(screen_tex, src);
	}
}
