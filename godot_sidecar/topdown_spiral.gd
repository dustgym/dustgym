extends RefCounted
class_name TopdownSpiral
# OWNER LANE: DEMO viz battery -- net-new TOP-DOWN render mode (--topdown-spiral).
#
# Renders the WHOLE patch from a fixed bird's-eye ORTHOGRAPHIC camera, ONCE per spiral
# frame, mirroring the --depart-spiral rover trajectory so the per-frame PNGs index 1:1
# with the rover-cam run (out/cam/haworth_spiral_{lit,unlit}/<NNN>/). Two variants drive
# the viz-battery composites (scripts/demo/spiral_composites.py):
#   * LIT   (default): the real grazing-sun Hapke terrain + clasts -- the same world the
#     rover stereo sees, viewed from above. Run with --sun-elev/--sun-azim matching the
#     rover-cam run so the lighting is identical.
#   * UNLIT (--scene-unlit): a BLAND diagnostic view -- Lambert (no Hapke), the directional
#     shadows OFF + an ambient flood (so relief reads softly with NO harsh cast shadows),
#     SPHERICAL clasts (sidecar._build_clasts zeroes the procgen relief when _scene_unlit),
#     and the live QUADTREE OVERLAY (terrain.gd::_build_quadtree_overlay) fed per frame from
#     a host-emitted qt_leaves.json (scripts/demo/instrument_spiral.py). This is the
#     pipeline-visibility headline: the sim's own demand-driven LOD over plain geometry.
#
# REUSE, NOT FORK (same discipline as depart_spiral.gd):
#   * depart_spiral.gd  -- DepartSpiral._spiral_rc / ._look_at_yaw (the SAME Archimedean
#     spiral + look-at-lander yaw, so the trajectory is byte-identical to the rover-cam run).
#   * lander_bundle.gd  -- LanderBundle._build_4face_lander() places the fixed-center lander
#     ONCE (held constant across the per-frame rebuild via the detach/re-attach dance).
#   * sidecar.gd        -- _build_layers / _clear_frame_nodes / _find_rover_root rebuild the
#     per-frame terrain+clasts+rover+overlay; _setup_environment's sun + WorldEnvironment
#     persist across the clear (only Camera3D/DirectionalLight3D/WorldEnvironment survive).
#   * state_fields.gd   -- sf.quadtree_nodes / active_leaves / quadtree_lod feed the overlay.
#     We DO NOT set sf.has_rover_rc, so terrain.gd keeps the FULL-patch fine mesh (the fine
#     window only follows the rover when has_rover_rc is true) -- the whole patch stays visible.
#
# DISPATCH (sidecar.gd, mirrors --depart-spiral): preload TopdownSpiralScript; in _parse_args
#   "--topdown-spiral": _topdown_spiral_mode = true (+ _drums_up); plus --scene-unlit /
#   --out-scene-name / --qt-leaves. In _ready(), before _setup_camera():
#       if _topdown_spiral_mode:
#           await TopdownSpiralScript.run_topdown_spiral(self); get_tree().quit(0); return
# MUST be awaited (coroutine awaiting frame_post_draw per frame; un-awaited -> black frames).

const DepartSpiralScript := preload("res://depart_spiral.gd")

# ENTRY POINT (dispatched by sidecar.gd). VOID coroutine. `sidecar` is the sidecar Node3D,
# already past _setup_environment() + _build_layers() (the first build), so the sun +
# WorldEnvironment exist and we tweak them once for the unlit variant below.
static func run_topdown_spiral(sidecar) -> void:
	var sf = sidecar.sf
	if sf == null:
		push_error("topdown_spiral: --topdown-spiral requires a loaded scene (--scene <dir>)")
		sidecar.get_tree().quit(2)
		return
	if not sidecar._layers.has("rover"):
		push_error("topdown_spiral: --topdown-spiral requires the 'rover' layer; add 'rover' to --layers")
		sidecar.get_tree().quit(4)
		return

	# Frame count: --stride doubles as the frame-count knob (the depart_spiral idiom).
	var n_frames: int = maxi(2, sidecar._seq_stride if sidecar._seq_stride > 2 else DepartSpiralScript.DEFAULT_FRAMES)

	# --- scene-center cell + world position (the FIXED lander site), mirrors depart_spiral ----
	var center_rc := Vector2(float(sf.height - 1) * 0.5, float(sf.width - 1) * 0.5)
	var cu: float = clampf(center_rc.y / float(sf.width - 1), 0.0, 1.0)
	var cv: float = clampf(center_rc.x / float(sf.height - 1), 0.0, 1.0)
	var center_world := Vector3(
		sf.world_min.x + center_rc.y * sf.cell_m,
		sf.height_uv(cu, cv),
		sf.world_min.y + center_rc.x * sf.cell_m)

	# --- spiral trajectory (rover_rc + look-at yaw per frame), reused from depart_spiral -------
	var rc_seq: Array = DepartSpiralScript._spiral_rc(center_rc, n_frames)
	var yaw_seq: Array = []
	for k in range(n_frames):
		yaw_seq.append(DepartSpiralScript._look_at_yaw(rc_seq[k], center_rc))

	var scene: String = sf.scene_name
	var out_scene: String = sidecar._out_scene_name if sidecar._out_scene_name != "" else (scene + "_topdown")

	# --- per-frame quadtree leaves for the overlay (optional; unlit variant) -------------------
	var qt_frames: Array = []
	if sidecar._qt_leaves_path != "":
		qt_frames = _load_qt_leaves(sidecar._qt_leaves_path)
		print("topdown_spiral: loaded %d qt-leaf records from %s" % [qt_frames.size(), sidecar._qt_leaves_path])

	# --- per-frame conform pose track (optional; rover-physics pass) ----------------------------
	# drive_spiral.py rover_pose.json: TRAVEL-TANGENT yaw + the wheel-plane tilt (up normal) per
	# frame. Absent => fall back to the in-engine look-at-lander yaw + flat pose (back-compat).
	var pose_track: Array = []
	if sidecar._rover_pose_path != "":
		pose_track = DepartSpiralScript.load_pose_track(sidecar._rover_pose_path)
		print("topdown_spiral: loaded %d rover-pose records from %s" % [pose_track.size(), sidecar._rover_pose_path])

	# --- full per-wheel track polylines for the ACCUMULATING cleat trail ------------------------
	# sf.wheel_tracks was populated at load_scene from the DRIVEN scene's §5.2 metadata. Keep the
	# FULL polylines; each frame we feed only [0..k] so the trail GROWS behind the rover (John's
	# "accumulate per-frame"). Empty when the base scene (no wheel_tracks) is rendered.
	var full_tracks := {}
	for tk in sf.wheel_tracks.keys():
		full_tracks[tk] = (sf.wheel_tracks[tk] as Dictionary)["points"]

	# --- fixed-center 4-face lander, built ONCE and held constant (the depart_spiral pattern) --
	var standoff: float = sidecar._lander_standoff if sidecar._lander_standoff > 0.0 else 2.5
	var fixed_fwd := Vector3(-1, 0, 0)
	var place_rover_pos := center_world - fixed_fwd * standoff
	sidecar.LanderBundleScript.unlit_tags = sidecar._tag_unlit
	var lander_root: Node3D = sidecar.LanderBundleScript._build_4face_lander(
		sidecar, sf, place_rover_pos, fixed_fwd, standoff, sidecar._lander_yaw_deg)
	var lander_xf_const: Transform3D = lander_root.global_transform

	# --- BLAND/UNLIT environment tweak (ONCE; sun + WorldEnvironment persist across the clear) -
	if sidecar._scene_unlit:
		_make_unlit(sidecar)

	# --- persistent ORTHOGRAPHIC top-down camera (Camera3D survives _clear_frame_nodes) --------
	# Frame the WHOLE patch from directly above; up-vector (0,0,-1) (a straight-down look along
	# -Y is degenerate with the default Vector3.UP). cam.size is the ortho frustum HEIGHT in world
	# metres -> the patch span + 1% margin. The pow2-padded quadtree overhang (field>patch) falls
	# outside this frame and is clipped. (John: whole-patch fixed framing.)
	var ext: Vector2 = sf.extent_m()
	var cx: float = sf.world_min.x + ext.x * 0.5
	var cz: float = sf.world_min.y + ext.y * 0.5
	var span: float = maxf(ext.x, ext.y)
	var cam := Camera3D.new()
	# PERSPECTIVE top-down from high up (an orthographic cam at this scene's absolute ~2840 m
	# elevation was not rasterizing the terrain ArrayMesh -- a near-overhead PERSPECTIVE cam framed
	# to the patch reads as effectively top-down with negligible parallax over the gentle relief).
	cam.projection = Camera3D.PROJECTION_PERSPECTIVE
	cam.fov = 60.0
	cam.near = 0.5
	cam.far = span * 6.0 + 200.0
	# Height so the 60deg vertical FOV frames the patch span with ~10% margin: H = (span/2)/tan(fov/2).
	var cam_h: float = (span * 0.55) / tan(deg_to_rad(30.0))
	var cam_y: float = sf.height_range.y + cam_h
	cam.look_at_from_position(
		Vector3(cx, cam_y, cz), Vector3(cx, sf.height_range.x, cz), Vector3(0, 0, -1))
	cam.current = true
	sidecar.add_child(cam)

	print("topdown_spiral: scene='%s' out='%s' frames=%d ortho_size=%.1fm cam_y=%.1f unlit=%s qt=%d" % [
		scene, out_scene, n_frames, cam.size, cam_y, str(sidecar._scene_unlit), qt_frames.size()])

	# --- per-frame render loop -----------------------------------------------------------------
	var n_written := 0
	# Per-frame camera projection of 3 world refs (origin + 10 m along +X/+Z) so the composite can
	# map the wheel-track polyline onto THIS panel (works for whole-patch OR frame-both framing,
	# robust to camera changes). The 2 cm cleat detail is sub-pixel at any rover+origin-in-frame
	# zoom, so the trail is drawn as polyline MARKUP off this affine, not rendered as cleats.
	var proj_records: Array = []
	for k in range(n_frames):
		var rc: Vector2 = rc_seq[k]
		# Drive the rover to this frame's spiral footprint. Default: look-at-lander yaw + flat.
		sidecar._rover_rc_override = Vector2i(int(round(rc.x)), int(round(rc.y)))
		sidecar._rover_yaw = yaw_seq[k]
		sidecar._rover_up = Vector3.UP
		# Conform pose track (rover-physics): TRAVEL-TANGENT yaw + wheel-plane tilt override.
		if k < pose_track.size():
			var prec: Dictionary = pose_track[k]
			sidecar._rover_yaw = float(prec.get("yaw_rad", yaw_seq[k]))
			var upv = prec.get("up", null)
			if typeof(upv) == TYPE_ARRAY and upv.size() == 3:
				sidecar._rover_up = Vector3(float(upv[0]), float(upv[1]), float(upv[2]))
			var prc = prec.get("rc", null)
			if typeof(prc) == TYPE_ARRAY and prc.size() == 2:
				sidecar._rover_rc_override = Vector2i(int(round(float(prc[0]))), int(round(float(prc[1]))))

		# Feed the live quadtree leaves to the overlay. We set ONLY the overlay inputs, NOT
		# sf.has_rover_rc, so terrain.gd keeps the static full-patch fine mesh (the overlay does
		# not depend on has_rover_rc).
		if k < qt_frames.size():
			var rec: Dictionary = qt_frames[k]
			sf.quadtree_nodes = rec.get("nodes", [])
			sf.active_leaves = _coerce_boxes(rec.get("active_leaves", []))
			var lod = rec.get("lod", {})
			sf.quadtree_lod = lod if typeof(lod) == TYPE_DICTIONARY else {}

		# ACCUMULATING cleat trail: feed only the polyline up to THIS frame, then invalidate the
		# baked track texture so terrain.gd re-bakes the trail-so-far on this _build_layers().
		if not full_tracks.is_empty():
			for tk in full_tracks.keys():
				var allpts: Array = full_tracks[tk]
				(sf.wheel_tracks[tk] as Dictionary)["points"] = allpts.slice(0, mini(k + 1, allpts.size()))
			sf._img_track_dir = null

		# Rebuild per-frame layers; preserve the fixed lander across the clear (it is not a
		# Camera3D/Light/WorldEnvironment, so _clear_frame_nodes would free it).
		if lander_root.get_parent() != null:
			lander_root.get_parent().remove_child(lander_root)
		sidecar._clear_frame_nodes()
		sidecar._build_layers()
		# The far-field terrain plane is displaced to the real (~2840 m) elevation ONLY in its
		# vertex shader, so its CPU AABB stays at Y=0 and Godot frustum-culls it from this high
		# camera. A large extra_cull_margin on the terrain meshes defeats that wrong-AABB cull.
		_uncull_terrain(sidecar)
		sidecar.add_child(lander_root)
		lander_root.global_transform = lander_xf_const

		var rover_root = sidecar._find_rover_root()
		if rover_root != null:
			# BIG heading marker only in the pure WHOLE-PATCH overview (rover is a speck there).
			# Zoomed views (follow / frame-both) show the real chassis + resolved ruts, so a small
			# dot just locates it without burying the tracks.
			_add_rover_marker(rover_root, sidecar._td_follow_m <= 0.0 and not sidecar._td_frameboth)
			# FOLLOW mode: an OBLIQUE chase cam tracking the rover at the requested span. Straight-
			# down foreshortens pitch/roll, so we view from ~55deg elevation -> the conform tilt,
			# wheel seating on the relief, and the trail all read. Span sets the framing distance.
			if sidecar._td_follow_m > 0.0:
				var rpos: Vector3 = rover_root.global_transform.origin
				var off: Vector3 = Vector3(0.45, 0.95, 0.45).normalized() * (sidecar._td_follow_m * 0.95)
				cam.look_at_from_position(rpos + off, rpos, Vector3(0, 1, 0))
			elif sidecar._td_frameboth:
				# Zoomed TOP-DOWN framing BOTH the rover and the lander (center), so the carved
				# ruts read as terrain features while origin + rover stay in frame for the
				# position-plot/unlit cross-reference. Span grows with the departure range.
				var rp: Vector3 = rover_root.global_transform.origin
				var mid := (rp + center_world) * 0.5
				var dist: float = Vector2(rp.x - center_world.x, rp.z - center_world.z).length()
				var fb_span: float = maxf(dist * 1.35 + 8.0, 24.0)
				var fb_h: float = (fb_span * 0.55) / tan(deg_to_rad(30.0))
				cam.look_at_from_position(Vector3(mid.x, sf.height_range.y + fb_h, mid.z),
					Vector3(mid.x, sf.height_range.x, mid.z), Vector3(0, 0, -1))
		else:
			push_warning("topdown_spiral: frame %d found no rover root after rebuild" % k)

		# Settle the viewport (geometry added this tick registers into the world scenario only on
		# the NEXT tree frame; first frame samples stale -- the proven depart_spiral 3-wait).
		for _w in range(3):
			await RenderingServer.frame_post_draw

		var nnn := "%03d" % k
		var out_dir := "res://out/cam/%s/%s" % [out_scene, nnn]
		DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))
		var img: Image = sidecar.get_viewport().get_texture().get_image()
		var path := "%s/topdown.png" % out_dir
		var err := img.save_png(path)
		if err != OK:
			push_error("topdown_spiral: save_png failed (%d) for %s" % [err, path])
		else:
			var rng: float = Vector2(rc.x - center_rc.x, rc.y - center_rc.y).length() * float(sf.cell_m)
			print("topdown_spiral: frame %s wrote %s (%dx%d) range=%.1fm" % [
				nnn, ProjectSettings.globalize_path(path), img.get_width(), img.get_height(), rng])
		# World->pixel affine refs for the composite polyline markup (cam transform final here).
		var ref_m := 10.0
		var po: Vector2 = cam.unproject_position(center_world)
		var pex: Vector2 = cam.unproject_position(center_world + Vector3(ref_m, 0.0, 0.0))
		var pez: Vector2 = cam.unproject_position(center_world + Vector3(0.0, 0.0, ref_m))
		proj_records.append({"frame": k, "ref_m": ref_m,
			"o": [po.x, po.y], "x": [pex.x, pex.y], "z": [pez.x, pez.y]})
		n_written += 1

	# Projection sidecar for the composite markup (one affine per frame next to the PNGs).
	var proj_path := "res://out/cam/%s/proj.json" % out_scene
	var pf := FileAccess.open(proj_path, FileAccess.WRITE)
	if pf != null:
		pf.store_string(JSON.stringify(proj_records))
		pf.close()
		print("topdown_spiral: wrote %d projection records -> %s" % [
			proj_records.size(), ProjectSettings.globalize_path(proj_path)])

	print("topdown_spiral: --topdown-spiral wrote %d frames to %s" % [
		n_written, ProjectSettings.globalize_path("res://out/cam/%s" % out_scene)])

# Flatten the scene for the UNLIT/bland diagnostic view: kill the directional shadows + dim the
# key, and flood a soft ambient so relief reads gently with NO harsh grazing cast shadows. The
# terrain/clast shaders are `specular_disabled` (NOT unshaded), so the environment ambient adds a
# flat term across both. The sun stays (dim) for soft form; pass a raised --sun-elev for an even
# top-down key. Lambert (no Hapke) comes from --scene-unlit also clearing _brdf_hapke in _parse_args.
static func _make_unlit(sidecar) -> void:
	for ch in sidecar.get_children():
		if ch is DirectionalLight3D:
			var dl := ch as DirectionalLight3D
			dl.shadow_enabled = false
			dl.light_energy = 1.5
		elif ch is WorldEnvironment:
			var e: Environment = (ch as WorldEnvironment).environment
			if e != null:
				e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
				e.ambient_light_color = Color(0.72, 0.72, 0.75)
				e.ambient_light_energy = 0.7
				e.background_color = Color(0.05, 0.05, 0.06)

# A bright UNSHADED marker pinning the rover position so it reads at the ~220 m ortho scale
# (the articulated rover is a speck from this height). Parented to the rover root so it is freed
# with the rover on the next _clear_frame_nodes (rebuilt each frame).
static func _add_rover_marker(rover_root: Node3D, big: bool) -> void:
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.albedo_color = Color(1.0, 0.12, 0.85)              # magenta: not a regolith/overlay hue
	if big:
		# WHOLE-PATCH scale: the articulated rover is a speck, so pin a HEADING marker -- a pivot
		# ball + a bar extending FORWARD (+X local) so the rover's facing reads from directly
		# above (and, via foreshortening, the conform tilt dips the bar). Parented to rover_root
		# so it inherits the yaw+tilt and is freed with the rover on the next _clear_frame_nodes.
		var sph := SphereMesh.new()
		sph.radius = 2.5
		sph.height = 5.0
		sph.material = mat
		var ball := MeshInstance3D.new()
		ball.mesh = sph
		ball.position = Vector3(0.0, 3.0, 0.0)             # lift above the chassis so top-down sees it
		ball.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		rover_root.add_child(ball)
		var bar := BoxMesh.new()
		bar.size = Vector3(9.0, 0.8, 1.6)                  # long axis = local +X (rover forward)
		var barmi := MeshInstance3D.new()
		barmi.mesh = bar
		barmi.material_override = mat
		barmi.position = Vector3(5.0, 3.0, 0.0)            # offset FORWARD of the pivot -> shows heading
		barmi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		rover_root.add_child(barmi)
	else:
		# FOLLOW close-up: the real articulated rover is visible at this span, so just a small
		# unobtrusive dot locates the pivot without burying the chassis/tilt.
		var sph := SphereMesh.new()
		sph.radius = 0.18
		sph.height = 0.36
		sph.material = mat
		var ball := MeshInstance3D.new()
		ball.mesh = sph
		ball.position = Vector3(0.0, 0.7, 0.0)
		ball.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		rover_root.add_child(ball)

# Load the host-emitted per-frame quadtree-leaf records (instrument_spiral.py qt_leaves.json):
# a plain JSON list [{frame, nodes:[{level,row0,col0,size,leaf}], active_leaves:[[r0,c0,r1,c1]],
# lod:{min_leaf,...}}, ...], or a {"records":[...]} wrapper. Returns [] (overlay simply empty) on
# any failure so a missing file never aborts the render.
static func _load_qt_leaves(path: String) -> Array:
	var p := path
	if not (p.begins_with("res://") or p.begins_with("user://") or p.begins_with("/")):
		p = "res://" + p
	var f := FileAccess.open(p, FileAccess.READ)
	if f == null:
		push_warning("topdown_spiral: cannot open qt-leaves '%s' (err %d) -- overlay will be empty" % [
			p, FileAccess.get_open_error()])
		return []
	var txt := f.get_as_text()
	f.close()
	var parsed = JSON.parse_string(txt)
	if typeof(parsed) == TYPE_ARRAY:
		return parsed
	if typeof(parsed) == TYPE_DICTIONARY and parsed.has("records"):
		return parsed["records"]
	push_warning("topdown_spiral: qt-leaves '%s' did not parse to a list" % p)
	return []

# Defeat the wrong-AABB frustum cull of the GPU-displaced far-field plane: set a large
# extra_cull_margin on every GeometryInstance3D under the terrain node.
static func _uncull_terrain(sidecar) -> void:
	for ch in sidecar.get_children():
		if ch.get_script() == sidecar.TerrainScript:
			_set_cull_margin(ch, 6000.0)

static func _set_cull_margin(node, m: float) -> void:
	if node is GeometryInstance3D:
		(node as GeometryInstance3D).extra_cull_margin = m
	for c in node.get_children():
		_set_cull_margin(c, m)


# Normalize [r0,c0,r1,c1] boxes (mirrors state_fields._coerce_boxes), skipping malformed entries.
static func _coerce_boxes(raw) -> Array:
	var out: Array = []
	if typeof(raw) != TYPE_ARRAY:
		return out
	for b in raw:
		if typeof(b) == TYPE_ARRAY and b.size() == 4:
			out.append([int(b[0]), int(b[1]), int(b[2]), int(b[3])])
	return out
