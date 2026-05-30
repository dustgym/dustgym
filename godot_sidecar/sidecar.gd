extends Node3D
# D4 — layer-toggle headless render CLI for the foss_ipex Godot sidecar.
#
# Render-only consumer of the frozen state fields (spec §2: Godot = renderer +
# sensor model only; it never authors physics). Parses CLI args after '--',
# composes a chosen set of LAYERS over a loaded scene, renders one frame
# headless, saves a PNG, and quits — the D2 (LOD/terrain) + D4 (layer flipbook)
# deliverables.
#
# CLI (after '--'):
#   --scene  <dir>            scene directory (INTERFACE.md layout)        [required]
#   --pose   x,y,z,tx,ty,tz   camera position + look-at target (meters)
#   --layers a,b,c            comma list of layers to enable (default terrain,clasts)
#   --out    <png>            output path (default res://out/sidecar.png)
#   --size   WxH              viewport size (default 1024x768)
#
# LAYERS (each toggled on/off by presence in --layers):
#   heightmap  : unlit false-color ramp by elevation        (layer 1)
#   state      : unlit false-color by state_label enum       (layer 2)
#   terrain    : lit PBR regolith terrain under the lunar sun (layer 3)
#   clasts     : sphere instances at metadata clasts          (layer 4)
#   rover      : real EZ-RASSOR chassis glb (MIT, vendored)    (asset; runtime glTF)
#   dust       : ballistic GPUParticles3D, g=1.62, no drag    (layer 5, stretch)
#   distortion : Brown-Conrady barrel post-process stub       (layer 6, stretch)

const DEFAULT_OUT := "res://out/sidecar.png"
const DEFAULT_LAYERS := "terrain,clasts"

# Preload sibling scripts explicitly. In headless ad-hoc scene loads the global
# class_name registry is not always warm, so we do not rely on it; preload is
# deterministic. (The class_name decls remain for editor/reviewer clarity.)
const StateFieldsScript := preload("res://state_fields.gd")
const TerrainScript := preload("res://terrain.gd")

var _viewport_size := Vector2i(1024, 768)
var _out_path := DEFAULT_OUT
var _scene_dir := ""
var _layers: Array = []
var _cam_pos := Vector3(2.56, 2.2, 5.6)
var _cam_target := Vector3(2.56, -0.1, 2.56)
var _has_pose := false

var sf                       # StateFields instance (preloaded script)
var _cam: Camera3D

func _ready() -> void:
	_parse_args()
	if _scene_dir == "":
		push_error("sidecar: --scene <dir> is required")
		get_tree().quit(2); return

	get_window().size = _viewport_size

	sf = StateFieldsScript.new()
	if not sf.load_scene(_scene_dir):
		push_error("sidecar: failed to load scene: " + sf.error_msg)
		get_tree().quit(3); return

	if _layers.is_empty():
		_layers = DEFAULT_LAYERS.split(",")

	_setup_environment()
	_setup_camera()
	_build_layers()

	# Two post-draw waits: first frame may sample a stale buffer (per render_test.gd).
	# Extra waits when post-processing reads the back buffer (distortion) or when
	# GPUParticles need a frame to advance into their ballistic arc (dust).
	var waits := 2
	if _has("distortion") or _has("dust"):
		waits = 4
	for _w in range(waits):
		await RenderingServer.frame_post_draw

	var img := get_viewport().get_texture().get_image()
	var err := img.save_png(_out_path)
	if err != OK:
		push_error("sidecar: save_png failed: %d" % err)
		get_tree().quit(4); return
	print("sidecar: wrote ", ProjectSettings.globalize_path(_out_path),
		" size=", img.get_width(), "x", img.get_height(),
		" scene=", sf.scene_name, " layers=", ",".join(_layers))
	get_tree().quit(0)

# ---------------------------------------------------------------------------
func _parse_args() -> void:
	var args := OS.get_cmdline_user_args()  # everything after '--'
	var i := 0
	while i < args.size():
		var a := String(args[i])
		match a:
			"--scene":
				i += 1; _scene_dir = String(args[i])
			"--out":
				i += 1; _out_path = _abs_out(String(args[i]))
			"--layers":
				i += 1
				_layers = []
				for L in String(args[i]).split(","):
					var t := L.strip_edges()
					if t != "": _layers.append(t)
			"--size":
				i += 1
				var wh := String(args[i]).split("x")
				if wh.size() == 2:
					_viewport_size = Vector2i(int(wh[0]), int(wh[1]))
			"--pose":
				i += 1
				var p := String(args[i]).split(",")
				if p.size() == 6:
					_cam_pos = Vector3(float(p[0]), float(p[1]), float(p[2]))
					_cam_target = Vector3(float(p[3]), float(p[4]), float(p[5]))
					_has_pose = true
			_:
				push_warning("sidecar: unknown arg '%s'" % a)
		i += 1

# Allow plain filesystem paths for --out (resolve relative to res://out/).
func _abs_out(p: String) -> String:
	if p.begins_with("res://") or p.begins_with("user://") or p.begins_with("/"):
		return p
	return "res://out/" + p

func _has(layer: String) -> bool:
	return _layers.has(layer)

# ---------------------------------------------------------------------------
# LUNAR ENVIRONMENT (spec §8): single hard sun at ~5deg elevation, no fill,
# disabled ambient, near-black background, no SSIL/SDFGI/glow indirect light.
func _setup_environment() -> void:
	var sun := DirectionalLight3D.new()
	# ~5deg elevation grazing sun (spec §8 "0-7deg polar; grazing -> extreme shadows").
	# Azimuth chosen to rake across the camera-facing terrain so relief reads,
	# while the far crater wall stays in deep shadow (the perception hazard).
	sun.rotation_degrees = Vector3(-5.0, 215.0, 0.0)
	sun.light_energy = 3.0   # bright disc; vacuum has no scatter to fill shadows
	sun.shadow_enabled = true
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_PARALLEL_4_SPLITS
	add_child(sun)

	var we := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = Color(0.01, 0.01, 0.015)   # near-black vacuum sky
	# No atmospheric scatter / indirect gradient (spec §8):
	e.ambient_light_source = Environment.AMBIENT_SOURCE_DISABLED
	e.ambient_light_energy = 0.0
	e.ssil_enabled = false
	e.sdfgi_enabled = false
	e.glow_enabled = false
	e.ssao_enabled = false
	e.tonemap_mode = Environment.TONE_MAPPER_FILMIC  # tame extreme dynamic range
	e.tonemap_exposure = 1.2
	we.environment = e
	add_child(we)

func _setup_camera() -> void:
	_cam = Camera3D.new()
	_cam.fov = 55.0
	_cam.near = 0.02
	_cam.far = 100.0
	add_child(_cam)
	if not _has_pose:
		# Default pose: oblique 3/4 view framing the active zone center.
		var ext: Vector2 = sf.extent_m()
		var cx: float = sf.world_min.x + ext.x * 0.5
		var cz: float = sf.world_min.y + ext.y * 0.5
		_cam_pos = Vector3(cx, maxf(ext.x, ext.y) * 0.55, cz + ext.y * 0.9)
		_cam_target = Vector3(cx, sf.height_range.x, cz)
	_cam.look_at_from_position(_cam_pos, _cam_target, Vector3.UP)

# ---------------------------------------------------------------------------
func _build_layers() -> void:
	# Terrain-family layers are mutually informative; precedence:
	# heightmap / state false-color override the lit terrain look if requested.
	var terrain = TerrainScript.new()
	if _has("heightmap"):
		terrain.build(sf, TerrainScript.Mode.FALSECOLOR_HEIGHT)
		add_child(terrain)
	elif _has("state"):
		terrain.build(sf, TerrainScript.Mode.FALSECOLOR_STATE)
		add_child(terrain)
	elif _has("terrain"):
		terrain.build(sf, TerrainScript.Mode.LIT_PBR)
		add_child(terrain)
	# else: no terrain mesh requested (e.g. clasts-only diagnostic).

	if _has("clasts"):
		_build_clasts()
	if _has("rover"):
		_build_rover()
	if _has("dust"):
		_build_dust()
	if _has("distortion"):
		_build_distortion()

# Layer 4 — clasts as sphere MultiMesh at metadata center_m/radius_m.
# center_m is world [x, height_up, z] (INTERFACE.md §5, Godot-ready order).
func _build_clasts() -> void:
	if sf.clasts.is_empty():
		print("sidecar: clasts layer requested but scene has 0 clasts")
		return
	var sphere := SphereMesh.new()
	sphere.radius = 1.0
	sphere.height = 2.0          # unit sphere; per-instance scale sets radius
	sphere.radial_segments = 16
	sphere.rings = 8
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.55, 0.52, 0.49)  # rock slightly brighter than fines
	mat.roughness = 0.92
	mat.metallic = 0.0
	sphere.material = mat

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	mm.mesh = sphere
	mm.instance_count = sf.clasts.size()
	for i in range(sf.clasts.size()):
		var c: Dictionary = sf.clasts[i]
		var ctr = c.get("center_m", [0, 0, 0])
		var rad := float(c.get("radius_m", 0.05))
		var pos := Vector3(float(ctr[0]), float(ctr[1]), float(ctr[2]))
		var xf := Transform3D(Basis().scaled(Vector3(rad, rad, rad)), pos)
		mm.set_instance_transform(i, xf)
	var mmi := MultiMeshInstance3D.new()
	mmi.multimesh = mm
	add_child(mmi)
	print("sidecar: placed %d clasts" % sf.clasts.size())

# Layer 5 (stretch) — ballistic dust. No atmosphere: gravity 1.62, NO drag
# (spec §8 "Dust is ballistic, not suspended"). Emission tied to disturbance
# (proxy for slip x load; spec §8 "tie dust emission to disturbed-mass-rate").
# Basic version: emitters seeded at the most-disturbed cells, lofted upward,
# falling back under lunar g in ballistic arcs (no drag).
# Layer (asset) — the real RASSOR chassis mesh from EZ-RASSOR (base_unit, MIT,
# vendored; see THIRD_PARTY.md), converted DAE->glb by scripts/convert_rover_mesh.py.
# Loaded at RUNTIME via GLTFDocument so it works headless with no editor import step.
func _build_rover() -> void:
	var res_path := "res://assets/rover_base.glb"
	if not FileAccess.file_exists(res_path):
		print("sidecar: rover layer requested but %s missing (run scripts/convert_rover_mesh.py)" % res_path)
		return
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(ProjectSettings.globalize_path(res_path), state)
	if err != OK:
		push_warning("sidecar: rover glTF load failed (%d)" % err)
		return
	var rover := doc.generate_scene(state)
	if rover == null:
		push_warning("sidecar: rover generate_scene returned null")
		return
	# The DAE carried material colors (not per-vertex); give the whole body one
	# faintly-metallic grey so it reads as hardware against the matte regolith.
	var rmat := StandardMaterial3D.new()
	rmat.albedo_color = Color(0.62, 0.63, 0.66)
	rmat.metallic = 0.35
	rmat.roughness = 0.55
	_apply_material_recursive(rover, rmat)
	# Place on the surface, offset from the active-zone center so it sits on the
	# plain/rim (not down a crater bowl). Snap Y to the terrain height.
	var ext: Vector2 = sf.extent_m()
	var rx: float = sf.world_min.x + ext.x * 0.5 + ext.x * 0.22
	var rz: float = sf.world_min.y + ext.y * 0.5 + ext.y * 0.12
	var u: float = clampf((rx - sf.world_min.x) / ext.x, 0.0, 1.0)
	var v: float = clampf((rz - sf.world_min.y) / ext.y, 0.0, 1.0)
	var ry: float = sf.height_uv(u, v)
	var basis := Basis(Vector3.UP, deg_to_rad(35.0))
	rover.transform = Transform3D(basis, Vector3(rx, ry, rz))
	add_child(rover)
	print("sidecar: placed RASSOR rover (EZ-RASSOR base_unit, MIT) at (%.2f,%.2f,%.2f)" % [rx, ry, rz])

# Override the material on every MeshInstance3D under a node (the imported glTF tree).
func _apply_material_recursive(node: Node, mat: Material) -> void:
	if node is MeshInstance3D:
		(node as MeshInstance3D).material_override = mat
	for ch in node.get_children():
		_apply_material_recursive(ch, mat)

func _build_dust() -> void:
	var seeds := _top_disturbance_cells(10)
	if seeds.is_empty():
		print("sidecar: dust layer requested but disturbance is ~0 (no action)")
		return
	var soft := _soft_particle_texture()
	for s in seeds:
		var p := GPUParticles3D.new()
		p.position = s["pos"]
		p.amount = 600
		p.lifetime = 3.2
		p.fixed_fps = 30
		p.one_shot = false
		p.explosiveness = 0.0
		p.local_coords = false

		var pm := ParticleProcessMaterial.new()
		pm.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_SPHERE
		pm.emission_sphere_radius = 0.06
		# Gentle low-velocity loft (counter-rotating drum excavation is gentle, spec §8);
		# magnitude scaled by local disturbance (slip x load proxy, spec §8).
		var v := 0.25 + 0.7 * float(s["dist"])
		pm.direction = Vector3(0, 1, 0)
		pm.spread = 65.0
		pm.initial_velocity_min = v * 0.4
		pm.initial_velocity_max = v
		# BALLISTIC: lunar gravity, NO drag/damping (vacuum; spec §8 "ballistic, not suspended").
		pm.gravity = Vector3(0, -sf.gravity_m_s2, 0)
		pm.damping_min = 0.0
		pm.damping_max = 0.0
		# Wispiness + smoke-like growth: light turbulence; puffs expand as they rise.
		pm.turbulence_enabled = true
		pm.turbulence_noise_strength = 0.4
		pm.turbulence_noise_scale = 1.2
		pm.scale_min = 0.6
		pm.scale_max = 1.3
		var scurve := Curve.new()
		scurve.add_point(Vector2(0.0, 0.4))
		scurve.add_point(Vector2(1.0, 1.7))
		var sct := CurveTexture.new(); sct.curve = scurve
		pm.scale_curve = sct
		# Low per-particle alpha so overlapping puffs ACCUMULATE into haze (vs hard
		# opaque sprites = the 'retro' look). Fade in, then out over life.
		var grad := Gradient.new()
		grad.set_color(0, Color(0.80, 0.77, 0.72, 0.0))
		grad.set_color(1, Color(0.78, 0.75, 0.70, 0.0))
		grad.add_point(0.2, Color(0.80, 0.77, 0.72, 0.30))
		var gt := GradientTexture1D.new(); gt.gradient = grad
		pm.color_ramp = gt
		p.process_material = pm

		# Soft round billboard puff (radial alpha falloff) — NOT a hard-edged quad.
		var dm := QuadMesh.new()
		dm.size = Vector2(0.12, 0.12)
		var dmat := StandardMaterial3D.new()
		dmat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		dmat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		dmat.billboard_mode = BaseMaterial3D.BILLBOARD_PARTICLES
		dmat.albedo_texture = soft
		dmat.vertex_color_use_as_albedo = true
		dm.material = dmat
		p.draw_pass_1 = dm

		# Pre-roll so the haze is mid-flight at capture (single-frame render).
		p.preprocess = 2.0
		p.emitting = true
		add_child(p)
	print("sidecar: %d soft dust emitters at disturbed cells" % seeds.size())

# A soft round particle sprite: radial gaussian alpha falloff, so overlapping
# billboards read as haze/smoke rather than hard-edged 'retro' sprites.
func _soft_particle_texture(n: int = 64) -> ImageTexture:
	var img := Image.create(n, n, false, Image.FORMAT_RGBA8)
	var cen := float(n - 1) * 0.5
	for y in range(n):
		for x in range(n):
			var dx := (float(x) - cen) / cen
			var dy := (float(y) - cen) / cen
			var rr := sqrt(dx * dx + dy * dy)
			var a := exp(-rr * rr * 3.0) * clampf(1.0 - rr, 0.0, 1.0)
			img.set_pixel(x, y, Color(1, 1, 1, a))
	return ImageTexture.create_from_image(img)

# Find the N most-disturbed cells (slip x load proxy). Returns world pos + value.
func _top_disturbance_cells(n: int) -> Array:
	var best: Array = []
	var step: int = maxi(1, int(sf.width / 64))  # coarse scan; plenty for emitters
	for r in range(0, sf.height, step):
		for c in range(0, sf.width, step):
			var d: float = sf.img_disturbance.get_pixel(c, r).r
			if d > 0.05:
				best.append({"d": d, "r": r, "c": c})
	best.sort_custom(func(a, b): return a["d"] > b["d"])
	var out: Array = []
	for k in range(mini(n, best.size())):
		var e = best[k]
		var pos: Vector3 = sf.world_pos(int(e["r"]), int(e["c"]))
		pos.y += 0.03
		out.append({"pos": pos, "dist": e["d"]})
	return out

# Layer 6 (stretch) — Brown-Conrady barrel distortion post-process stub.
# Applied as a full-screen CanvasLayer quad over the rendered 3D frame.
func _build_distortion() -> void:
	var cl := CanvasLayer.new()
	cl.layer = 100
	# Copy the rendered 3D frame into the back buffer so the post shader can
	# sample it via hint_screen_texture (Godot 4 screen-read pattern).
	var bbc := BackBufferCopy.new()
	bbc.copy_mode = BackBufferCopy.COPY_MODE_VIEWPORT
	cl.add_child(bbc)
	var rect := ColorRect.new()
	rect.anchor_right = 1.0
	rect.anchor_bottom = 1.0
	rect.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var sm := ShaderMaterial.new()
	sm.shader = load("res://distortion.gdshader")
	sm.set_shader_parameter("k1", 0.35)
	sm.set_shader_parameter("k2", 0.10)
	rect.material = sm
	cl.add_child(rect)
	add_child(cl)
	print("sidecar: distortion post-process stub enabled (Brown-Conrady radial)")
