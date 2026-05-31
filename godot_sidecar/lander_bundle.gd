extends RefCounted
class_name LanderBundle
# OWNER LANE: M3-tag (4-face AprilTag bundle on the lander).
#
# Fills the L0 NO-OP skeleton. This lane NEVER edits sidecar.gd (the --lander-faces
# flag + dispatch call-site are already wired by L0: sidecar.gd ~215-218 awaits
# build_lander_faces(self) then get_tree().quit(0)). It also NEVER edits the FROZEN
# seams it CALLS: apriltag_gen.gd (REFERENCE ONLY for the id-0 bitmap + the QuadMesh
# texture-yaw convention), sensors_emit.gd (the schema sink + the single-face
# build_lander), camera_rig.gd (the front-stereo rig), frames.py (R_LANDER_TAG).
#
# Contract (FROZEN v1.1 §1, §1.1, §2.2, §3, §4 superseded-by-1.1-pin):
#   Build the 4-face tag bundle (ids 0..3, one per lander vertical face) and produce
#   the v1.1 OPTIONAL "lander"."apriltags":[{family,id,size_m,pose_in_lander}] that
#   the shared sink SensorsEmit.build_sensors_json(faces=...) emits. apriltags[]
#   SUPERSEDES the single apriltag{}; the FRONT face (id 0) keeps the existing
#   IDENTITY pose_in_lander so the M1 R_face(id0)==frames.R_LANDER_TAG holds.
#
# CONTRACT §4 vs the M1-invariance pin: §4 says "lander origin moves to the body
# center with per-face offsets". The lander_bundle skeleton header + §1.1 + the §2.2
# supersede-rule SUPERSEDE that: following §4 literally would move the lander origin
# off the front tag center and break R_face(id0)==R_LANDER_TAG. So the lander origin
# STAYS at the FRONT tag center (id0 identity pose_in_lander); faces 1-3 carry
# non-identity pose_in_lander expressed RELATIVE to that origin (offset by the body
# half-extents). This is the M1-invariance reading of §1.1, not §4.
#
# DISPATCH ARITY: sidecar.gd awaits build_lander_faces(self) with NO consumer of a
# return value (it just quits 0). So this is a VOID-style capture coroutine (the
# capture_seq.gd pattern): it does the FULL capture + sensors.json write ITSELF. The
# skeleton's `-> Array` return annotation was explicitly TBD-by-lane; redefined here.

# --- SOURCED constants -----------------------------------------------------------
# tag36h11, size_m = 0.150 (the 8x8 black-border square side; APRILTAG_SIZE_M in
# sensors_emit.gd:29 + sidecar.gd:153). All four faces share family + size.
const TAG_FAMILY := "tag36h11"
# 2.5 m fiducial: sized for ~100 m theoretical detection range at the 1024px/74deg ideal
# pinhole (the 0.15 m M1 tag died ~6m = a ~17px floor; 17px*100m/679px_fx ~= 2.5m). A tag this
# large needs a lander to host it -> BODY_SIZE scaled to LM-class ~5m below. (DEMO range study;
# the M1 --cameras single-tag path uses sensors_emit/apriltag_gen's own 0.15 m, unchanged.)
const TAG_SIZE_M := 2.5
# The white quiet ring sits OUTSIDE size_m by 10/8 (apriltag_gen.gd QUIET_RATIO=1.25;
# the printed marker spans 10 cells, the metric size is the 8x8 black-border square).
const QUIET_RATIO := 1.25

# Lit-material params (JOHN DECISION: ALL 4 FACES LIT, shadow-degradable). The tag
# texture is the albedo of a LIT StandardMaterial3D so faces on the anti-sun side
# fall into deep shadow under the 5deg grazing sun -- that degradation IS the GMRO
# deliverable. White-cell albedo ~0.85 = painted-aluminium fiducial reflectance
# (a printed/painted Al fiducial, NOT regolith); roughness 0.7; metallic 0.0.
const TAG_WHITE_ALBEDO := 0.85         # painted-aluminium white-cell reflectance
const TAG_ALBEDO_ROUGHNESS := 0.7
const TAG_ALBEDO_METALLIC := 0.0
const TAG_PX_PER_CELL := 32            # matches apriltag_gen.make_texture default (crisp cells)

# RE-BASELINE NOTE (DOCUMENTED, intended): because the bundle FRONT face (id0) is now
# LIT (unlike the M1 --cameras single tag, which uses the FROZEN UNLIT
# apriltag_gen.build_tag_quad), a pose read off this bundle's front face will NOT
# match the M1 --cameras reading (12.7mm / 7.15deg). That M1 single-tag --cameras path
# is UNCHANGED and still uses the unlit quad. The geometric R_face(id0)==R_LANDER_TAG
# invariance still holds (identity pose_in_lander); only the photometric legibility
# of the front face changes under shadow. This is the experiment, not a regression.

# Body geometry SOURCED VERBATIM from sensors_emit.build_lander (frozen) so the 4-face
# bundle sits on the SAME procedural lander body M1 uses:
#   body_size = (0.55 depth-X, 0.6 height-Y, 0.9 width-Z); front face at local x=0
#   (the tag center == lander origin); body center pulled BEHIND the tag at
#   x = -body_size.x/2 - 0.02 (the 2cm "proud" gap so the tag never z-fights the body).
const BODY_SIZE := Vector3(4.0, 5.0, 5.0)    # x depth, y height, z width -- LM-class body to host the 2.5m tag (3.125m printed) on every vertical face
const TAG_PROUD_M := 0.1                      # tag sits 0.1m proud of the body face (no z-fight on the big body)
# Body center along lander -X (parametric -> scales with BODY_SIZE):
const BODY_CENTER_X := -BODY_SIZE.x * 0.5 - TAG_PROUD_M
const BODY_CENTER_Y := 0.0                    # tag center (y=0) = body vertical center -> tag mid-face on the big body

# Per-face capture: a small turntable -- ONE inspection view per face normal -- so each
# face is seen near fronto-parallel once, the sun-facing faces stay legible and the
# anti-sun faces render shadowed/low-contrast (the degradation). Orbit radius / pitch
# are [CALIB] inspection-camera values (NOT a sourced sensor rig); the front-stereo
# sensors.json poses come from the FROZEN camera_rig instead (see below).
const ORBIT_RADIUS_M := 1.6          # [CALIB] inspection-cam standoff from lander origin
const ORBIT_PITCH_DEG := 8.0         # [CALIB] slight downward tilt so the tag fills frame
const ORBIT_FOV_DEG := 50.0          # [CALIB] inspection FOV (NOT the URDF sensor FOV)

# Canonical tag36h11 bitmaps (1 = white, 0 = black), row-major, top-left origin, the
# full 10x10 printed marker (1-cell white quiet ring, 1-cell black border, 6x6 payload).
#
# PROVENANCE (BSD data, NOT relicensed art -- same provenance the existing id-0 grid in
# apriltag_gen.gd cites): decoded VERBATIM from AprilRobotics apriltag-imgs/tag36h11/
# tag36_11_0000{0,1,2,3}.png. id0 below is byte-identical to apriltag_gen.TAG36H11_ID0
# (copied -- NOT edited there); that match PROVES the decode of the source PNGs is
# correct, so ids 1-3 are the genuine canonical codebook entries (decode-verified, not
# placeholders). C1's detector decoding these as ids 0..3 is the ROS-lane acceptance
# test; this Godot lane only renders them.
const TAG_BITMAPS := {
	0: [
		"1111111111", "1000000001", "1011010101", "1001110101", "1001100001",
		"1010100001", "1001011001", "1000010001", "1000000001", "1111111111",
	],
	1: [
		"1111111111", "1000000001", "1011011001", "1001011101", "1011110001",
		"1001100001", "1010110101", "1000100101", "1000000001", "1111111111",
	],
	2: [
		"1111111111", "1000000001", "1011011101", "1001001001", "1010000001",
		"1000100101", "1000010001", "1000111001", "1000000001", "1111111111",
	],
	3: [
		"1111111111", "1000000001", "1011100101", "1000011101", "1010011101",
		"1010100101", "1011001001", "1001100001", "1000000001", "1111111111",
	],
}

# Per-face YAW about lander +Y that takes the FRONT quad rigidly onto face f, and the
# resulting outward normal (SOURCED placement: id0 +X front toward rover, id1 +Z,
# id2 -X, id3 -Z). Each entry: {id, yaw_deg, normal}. The front quad (id0) uses the
# SAME +90deg QuadMesh texture-yaw the M1 front tag uses (sensors_emit.gd:146); faces
# 1-3 are that quad PLUS this extra yaw, so each face's pose_in_lander rotation is
# exactly R_pil = Basis(+Y, yaw_deg) and C1 derives R_face = R_pil * R_LANDER_TAG.
# For id0 (yaw 0) R_pil = identity, so R_face(id0) == R_LANDER_TAG -> M1-invariant.
#   yaw   0 -> normal +X  (front, toward rover)   id0
#   yaw 270 -> normal +Z                          id1
#   yaw 180 -> normal -X                           id2
#   yaw  90 -> normal -Z                           id3
# (verified: Basis(+Y, deg).xform of front normal +X yields these normals.)
const FACES := [
	{"id": 0, "yaw_deg":   0.0, "normal": Vector3( 1, 0,  0)},
	{"id": 1, "yaw_deg": 270.0, "normal": Vector3( 0, 0,  1)},
	{"id": 2, "yaw_deg": 180.0, "normal": Vector3(-1, 0,  0)},
	{"id": 3, "yaw_deg":  90.0, "normal": Vector3( 0, 0, -1)},
]


# Build the marker ImageTexture for `tag_id` from TAG_BITMAPS, expanded
# nearest-neighbour px_per_cell per cell (mirrors apriltag_gen.make_texture, which is
# REFERENCE-ONLY / id-0-only -- we cannot call it for ids 1-3, so we replicate its
# loop here over our own multi-id bitmap table). RGB8, no mips.
static func _make_tag_texture(tag_id: int, px_per_cell: int = TAG_PX_PER_CELL) -> ImageTexture:
	px_per_cell = maxi(px_per_cell, 1)
	var grid: Array = TAG_BITMAPS[tag_id]
	var cells := grid.size()                       # 10 (CELLS_TOTAL)
	var dim := cells * px_per_cell
	var img := Image.create(dim, dim, false, Image.FORMAT_RGB8)
	# White cell = the painted-aluminium fiducial albedo (NOT pure 1.0): a LIT material
	# samples this as base reflectance, so the cell luma tracks the local irradiance ->
	# the anti-sun faces darken. Black cell stays 0 (the printed ink).
	var white := Color(TAG_WHITE_ALBEDO, TAG_WHITE_ALBEDO, TAG_WHITE_ALBEDO)
	var black := Color(0, 0, 0)
	for cell_r in range(cells):
		var line: String = grid[cell_r]
		for cell_c in range(cells):
			var col: Color = white if line[cell_c] == "1" else black
			var x0 := cell_c * px_per_cell
			var y0 := cell_r * px_per_cell
			for y in range(y0, y0 + px_per_cell):
				for x in range(x0, x0 + px_per_cell):
					img.set_pixel(x, y, col)
	return ImageTexture.create_from_image(img)


# Build ONE LIT tag quad for `tag_id`. Geometry matches apriltag_gen.build_tag_quad
# (QuadMesh side = size_m * QUIET_RATIO so the 8x8 black-border square spans exactly
# size_m; the quad faces local +Z; double-sided). The MATERIAL is the JOHN DECISION
# difference: LIT (default SHADING_MODE_PER_PIXEL), so the cells receive the grazing
# sun and self-shadow per face -- UNLIKE the FROZEN unlit M1 quad.
# DEMO illumination A/B (--tag-unlit): when true, tag quads render UNSHADED (high-contrast,
# sun-independent) instead of the default LIT plate -- isolates tag illumination's effect on
# detection / pose-vs-truth. Set by depart_spiral from sidecar._tag_unlit before the lander build.
static var unlit_tags := false
static func _build_lit_tag_quad(tag_id: int) -> MeshInstance3D:
	var quad := QuadMesh.new()
	var full := TAG_SIZE_M * QUIET_RATIO
	quad.size = Vector2(full, full)

	var mat := StandardMaterial3D.new()
	mat.albedo_texture = _make_tag_texture(tag_id)
	# LIT: the fiducial is treated as a painted-aluminium plate that takes the sun, so
	# anti-sun faces fall into shadow (the degradation deliverable). NEAREST keeps cell
	# edges hard; double-sided so an off-axis inspection view never sees a culled back.
	mat.albedo_color = Color(1, 1, 1)              # albedo carried by the texture
	mat.roughness = TAG_ALBEDO_ROUGHNESS
	mat.metallic = TAG_ALBEDO_METALLIC
	mat.texture_filter = BaseMaterial3D.TEXTURE_FILTER_NEAREST
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	if unlit_tags:
		mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED   # DEMO --tag-unlit: high-contrast, sun-independent
	quad.material = mat

	var mi := MeshInstance3D.new()
	mi.name = "AprilTag_id%d" % tag_id
	mi.mesh = quad
	return mi


# Per-face pose_in_lander Transform3D (lander-local): rotation = Basis(+Y, yaw_deg)
# (== R_pil, the rigid yaw of the front quad onto this face); translation = the face's
# tag-center offset on the body face. id0 is IDENTITY by construction (yaw 0, offset 0)
# so M1 stays invariant. Offsets SOURCED from the body half-extents + TAG_PROUD_M:
#   +X face (id0): x=0 (== lander origin / front tag center).
#   -X face (id2): body back plane = BODY_CENTER_X - BODY_SIZE.x/2 - TAG_PROUD_M.
#   +Z face (id1): body +Z plane   = BODY_CENTER_X (x), z = +BODY_SIZE.z/2 + TAG_PROUD_M.
#   -Z face (id3): body -Z plane   = BODY_CENTER_X (x), z = -BODY_SIZE.z/2 - TAG_PROUD_M.
# All faces share the tag-center height (lander local y = 0), since the M1 tag center
# is the lander origin and the body spans symmetrically about it in y.
static func _face_pose_in_lander(face: Dictionary) -> Transform3D:
	var basis := Basis(Vector3(0, 1, 0), deg_to_rad(float(face["yaw_deg"])))
	var n: Vector3 = face["normal"]
	var pos := Vector3.ZERO
	if n.x > 0.5:                                   # +X front face: lander origin
		pos = Vector3.ZERO
	elif n.x < -0.5:                                # -X back face
		pos = Vector3(BODY_CENTER_X - BODY_SIZE.x * 0.5 - TAG_PROUD_M, 0.0, 0.0)
	elif n.z > 0.5:                                 # +Z face
		pos = Vector3(BODY_CENTER_X, 0.0, BODY_SIZE.z * 0.5 + TAG_PROUD_M)
	else:                                           # -Z face
		pos = Vector3(BODY_CENTER_X, 0.0, -(BODY_SIZE.z * 0.5 + TAG_PROUD_M))
	return Transform3D(basis, pos)


# Quad-local transform that orients a face's quad: the FRONT quad's +90deg
# QuadMesh texture-yaw (sensors_emit.gd:146, maps quad +Z -> lander +X) PLUS the
# per-face yaw R_pil. Applied as the tag MeshInstance3D's local transform under the
# lander root, with the tag centered at the face's pose_in_lander position.
static func _face_quad_transform(face: Dictionary) -> Transform3D:
	# R_pil (face yaw) * front-quad-yaw(+90 about +Y). Both are yaws about +Y so they
	# commute and compose to a single Basis(+Y, yaw_deg + 90).
	var total_yaw := float(face["yaw_deg"]) + 90.0
	var basis := Basis(Vector3(0, 1, 0), deg_to_rad(total_yaw))
	var pose := _face_pose_in_lander(face)
	return Transform3D(basis, pose.origin)


# Build the 4-face LIT procedural lander as a child of `parent`, placed `standoff`
# metres ahead of the rover along `fwd`, surface-snapped -- the SAME placement recipe
# as sensors_emit.build_lander (which we cannot reuse because it builds the single
# UNLIT front tag only). Lander origin == FRONT (id0) tag center, lander +X = tag
# outward normal toward the rover (-fwd). Returns the lander root (its global_transform
# is the lander pose reported in sensors.json). Body + legs are the same grey CC0 body.
static func _build_4face_lander(parent: Node, sf, rover_pos: Vector3, fwd: Vector3,
		standoff_arg: float, lander_yaw_deg: float) -> Node3D:
	var standoff: float = standoff_arg if standoff_arg > 0.0 else 2.5   # LANDER_STANDOFF_M
	var ground := rover_pos + fwd * standoff
	var u: float = clampf((ground.x - sf.world_min.x) / maxf(sf.extent_m().x, 1e-6), 0.0, 1.0)
	var v: float = clampf((ground.z - sf.world_min.y) / maxf(sf.extent_m().y, 1e-6), 0.0, 1.0)
	var surf_y: float = sf.height_uv(u, v)

	# Lander basis: +X = tag outward normal = -fwd (toward rover); +Y = up (mirrors
	# sensors_emit.build_lander so the front face matches the M1 placement frame).
	var nx := (-fwd).normalized()
	var ny := Vector3(0, 1, 0)
	var nz := nx.cross(ny).normalized()
	ny = nz.cross(nx).normalized()
	var tag_h := surf_y + 4.0                        # [CALIB] tag center 4m up on the ~5m LM-class body (mid-face; legs ~1.5m)
	var lander_basis := Basis(nx, ny, nz)
	if absf(lander_yaw_deg) > 1e-3:
		lander_basis = Basis(Vector3(0, 1, 0), deg_to_rad(lander_yaw_deg)) * lander_basis

	var root := Node3D.new()
	root.name = "Lander"
	root.transform = Transform3D(lander_basis, Vector3(ground.x, tag_h, ground.z))
	parent.add_child(root)

	var grey := StandardMaterial3D.new()
	grey.albedo_color = Color(0.55, 0.56, 0.58)
	grey.metallic = 0.2
	grey.roughness = 0.7

	var foot_y := surf_y - tag_h
	var body := MeshInstance3D.new()
	body.name = "lander_body"
	var box := BoxMesh.new()
	box.size = BODY_SIZE
	box.material = grey
	body.mesh = box
	body.position = Vector3(BODY_CENTER_X, BODY_CENTER_Y, 0.0)
	root.add_child(body)

	# 4 cylinder legs from the body base corners to the surface (verbatim recipe).
	var body_base_y := BODY_CENTER_Y - BODY_SIZE.y * 0.5
	var leg_height: float = maxf(body_base_y - foot_y, 0.25)
	for sx in [-1.0, 1.0]:
		for sz in [-1.0, 1.0]:
			var leg := MeshInstance3D.new()
			var cyl := CylinderMesh.new()
			cyl.top_radius = 0.15
			cyl.bottom_radius = 0.20
			cyl.height = leg_height
			cyl.material = grey
			leg.mesh = cyl
			leg.position = Vector3(
				BODY_CENTER_X + sx * BODY_SIZE.x * 0.25,
				foot_y + leg_height * 0.5,
				sz * BODY_SIZE.z * 0.4)
			root.add_child(leg)

	# The 4 LIT tag quads, one per vertical face, at their pose_in_lander positions.
	for face in FACES:
		var tag := _build_lit_tag_quad(int(face["id"]))
		tag.transform = _face_quad_transform(face)
		root.add_child(tag)

	print("lander_bundle: built 4-face LIT lander at (%.2f,%.2f,%.2f); ids 0..3, size_m=%.3f" % [
		ground.x, tag_h, ground.z, TAG_SIZE_M])
	return root


# Build the v1.1 lander.apriltags[] superset (ids 0..3), each entry
# {family, id, size_m, pose_in_lander} with pose_in_lander = SensorsEmit.pose_dict of
# the face transform (so the quaternion is derived by the FROZEN sink helper, not
# transcribed). FRONT face (id0) is identity by construction -> M1-invariant.
static func _build_faces_array(sidecar) -> Array:
	var out: Array = []
	for face in FACES:
		var pil := _face_pose_in_lander(face)
		out.append({
			"family": TAG_FAMILY,
			"id": int(face["id"]),
			"size_m": TAG_SIZE_M,
			"pose_in_lander": sidecar.SensorsEmitScript.pose_dict(pil),
		})
	return out


# A turntable inspection Camera3D in the sidecar's MAIN viewport, orbiting the lander
# origin at azimuth `azim_deg` (degrees, world). Aims at the lander origin with a small
# downward pitch so the tag fills frame. This is an INSPECTION view (NOT a sensor pose
# reported in sensors.json -- those come from the frozen camera_rig front pair), so its
# intrinsics are deliberately separate [CALIB] values.
static func _place_orbit_camera(cam: Camera3D, lander_origin: Vector3, azim_deg: float) -> void:
	var a := deg_to_rad(azim_deg)
	# Orbit in the XZ plane; raise the eye by the pitch so the look-down angle frames the tag.
	var horiz := ORBIT_RADIUS_M * cos(deg_to_rad(ORBIT_PITCH_DEG))
	var eye := lander_origin + Vector3(horiz * cos(a), ORBIT_RADIUS_M * sin(deg_to_rad(ORBIT_PITCH_DEG)), horiz * sin(a))
	cam.fov = ORBIT_FOV_DEG
	cam.look_at_from_position(eye, lander_origin, Vector3(0, 1, 0))


# Mean luma (Rec.601) of an Image -- the per-face shadow-degradation evidence.
static func _mean_luma(img: Image) -> float:
	var w := img.get_width()
	var h := img.get_height()
	var step := maxi(1, int(w / 256))               # subsample wide frames for speed
	var sum := 0.0
	var n := 0
	for y in range(0, h, step):
		for x in range(0, w, step):
			var c := img.get_pixel(x, y)
			sum += 0.299 * c.r + 0.587 * c.g + 0.114 * c.b
			n += 1
	return sum / maxf(float(n), 1.0)


# ENTRY POINT (dispatched by sidecar.gd ~217: `await build_lander_faces(self)` then
# get_tree().quit(0)). VOID coroutine -- does the full capture + sensors.json write
# itself (the capture_seq.gd pattern). `sidecar` is the sidecar Node3D, already past
# _setup_environment + _build_layers (the scene + rover are in the World3D).
static func build_lander_faces(sidecar) -> void:
	var sf = sidecar.sf
	if sf == null:
		push_error("lander_bundle: --lander-faces requires a loaded scene (--scene <dir>)")
		sidecar.get_tree().quit(2)
		return
	var rover_root = sidecar._find_rover_root()
	if rover_root == null:
		push_error("lander_bundle: --lander-faces requires the 'rover' layer (no rover root); add 'rover' to --layers")
		sidecar.get_tree().quit(4)
		return

	# Rover forward (+X local) projected to XZ -> the lander stands ahead of the rover
	# (mirrors sensors_emit.build_lander placement so the front face faces the rover).
	var rover_xf: Transform3D = rover_root.global_transform
	var fwd: Vector3 = rover_xf.basis * Vector3(1, 0, 0)
	fwd.y = 0.0
	if fwd.length() < 1e-5:
		fwd = Vector3(1, 0, 0)
	fwd = fwd.normalized()

	# Build the 4-face LIT lander ahead of the rover (our own build -- the frozen
	# build_lander only does the single UNLIT front tag).
	var lander_root := _build_4face_lander(
		sidecar, sf, rover_xf.origin, fwd, sidecar._lander_standoff, sidecar._lander_yaw_deg)
	var lander_origin: Vector3 = lander_root.global_transform.origin

	# Front-stereo rig via the FROZEN camera_rig -> the sensors.json camera poses are
	# the SAME sensor poses the M1 --cameras path emits (so the bundle's sensors.json is
	# schema-identical to --cameras except apriltags[] is populated with 4 faces).
	var world: World3D = sidecar.get_viewport().world_3d
	var cams: Array = sidecar.CameraRigScript.build(
		sidecar, rover_root, world, sidecar._viewport_size, sidecar._cam_pitch_deg)

	# Orbit inspection camera in the MAIN viewport: one view per face normal so each of
	# the 4 faces is seen near fronto-parallel once. Anti-sun faces render shadowed
	# (lit material) while sun-facing ones stay legible -- the degradation deliverable.
	var orbit_cam := Camera3D.new()
	orbit_cam.near = 0.02
	orbit_cam.far = 100.0
	sidecar.add_child(orbit_cam)
	orbit_cam.current = true

	var scene: String = sf.scene_name
	# Egress dir: out/lander_faces/<scene>/ (documented convention for this lane; the
	# per-face inspection PNGs + the bundle sensors.json live here). Distinct from the
	# --cameras out/cam/<scene>/000 tree so it never collides with the M1 egress.
	var out_dir := "res://out/lander_faces/%s" % scene
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))

	# Per-face orbit capture. The orbit azimuth is chosen so the camera looks AT each
	# face: a face whose world outward normal points toward azimuth A is viewed from A.
	var lander_basis: Basis = lander_root.global_transform.basis
	var luma_report: Array = []
	for face in FACES:
		# Face world outward normal -> orbit azimuth (atan2(z,x) of the normal).
		var n_world: Vector3 = (lander_basis * (face["normal"] as Vector3)).normalized()
		var azim := rad_to_deg(atan2(n_world.z, n_world.x))
		_place_orbit_camera(orbit_cam, lander_origin, azim)

		# Settle the main viewport, then capture (first frame can sample a stale buffer;
		# geometry registers into the world scenario on the next tree frame -- the proven
		# sidecar._render_to / capture_seq.gd await pattern).
		for _w in range(3):
			await RenderingServer.frame_post_draw

		var img: Image = sidecar.get_viewport().get_texture().get_image()
		var png := "%s/face_id%d.png" % [out_dir, int(face["id"])]
		var err := img.save_png(png)
		var luma := _mean_luma(img)
		luma_report.append({"id": int(face["id"]), "luma": luma})
		if err != OK:
			push_error("lander_bundle: save_png failed (%d) for %s" % [err, png])
		else:
			print("lander_bundle: face id%d azim=%.1fdeg wrote %s (%dx%d) mean_luma=%.4f" % [
				int(face["id"]), azim, ProjectSettings.globalize_path(png),
				img.get_width(), img.get_height(), luma])

	# Settle the front-stereo subviewports + write the front stereo PNGs (so the bundle
	# egress also carries the sensor images the sensors.json poses describe).
	for _w in range(3):
		await RenderingServer.frame_post_draw
	for e in cams:
		var img: Image = e["sv"].get_texture().get_image()
		var path := "%s/%s" % [out_dir, e["image"]]
		var err := img.save_png(path)
		if err != OK:
			push_error("lander_bundle: save_png failed (%d) for %s" % [err, path])
		else:
			print("lander_bundle: wrote %s (%dx%d)" % [
				ProjectSettings.globalize_path(path), img.get_width(), img.get_height()])

	# --- assemble + write the bundle sensors.json via the FROZEN sink ----------------
	# faces[] = the 4-face superset (ids 0..3). build_sensors_json emits it as
	# lander.apriltags[] (superseding apriltag{}); id0 identity keeps M1 invariant.
	var faces := _build_faces_array(sidecar)
	var sun = sidecar.SensorsEmitScript.sun_block(sidecar._sun_elev_deg, sidecar._sun_azim_deg, 0.0)
	var doc = sidecar.SensorsEmitScript.build_sensors_json(
		scene, 0, sidecar._viewport_size, rover_root, lander_root, cams,
		Callable(sidecar.CameraRigScript, "intrinsics"), sidecar.CameraRigScript.FOV_X_DEG,
		sun, faces, sidecar.CameraRigScript.rear_pair_descriptor(cams, rover_root))
	var json_path := "%s/sensors.json" % out_dir
	var jf := FileAccess.open(json_path, FileAccess.WRITE)
	if jf == null:
		push_error("lander_bundle: cannot open %s for write" % json_path)
		sidecar.get_tree().quit(6)
		return
	jf.store_string(JSON.stringify(doc, "  "))
	jf.close()
	print("lander_bundle: wrote %s (apriltags=%d faces, baseline_m=%.4f)" % [
		ProjectSettings.globalize_path(json_path), (doc["lander"]["apriltags"] as Array).size(),
		doc["stereo"]["baseline_m"]])
	# Per-face luma summary (the shadow-degradation evidence, for the smoke log).
	for r in luma_report:
		print("lander_bundle: SMOKE face id%d mean_luma=%.4f" % [int(r["id"]), float(r["luma"])])
