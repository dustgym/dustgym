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

# Trailing chase-camera framing (sequence/fly-through mode). Behind + above the rover, offset
# to one side for a 3/4 view; tighter than the whole-field shot so the rover + its fresh track
# read clearly while the quadtree LOD cluster stays in frame.
const TRAIL_M := 3.0          # meters behind the rover (along -forward)
const TRAIL_SIDE_M := 1.6     # meters to one side (3/4 view, so the track + LOD cluster show)
const TRAIL_HEIGHT_M := 2.1   # meters above the local surface (elevated, sees the ground around)
const TRAIL_FOV := 50.0       # tighter than the 55deg whole-field default

# Directional shadow frustum depth (render_fidelity #1). The scenes are a ~5.12 m patch;
# elevated/oblique cameras sit a few metres off, so ~16 m comfortably covers patch + rover
# + camera standoff while keeping the 8192 atlas dense (the default 100 m wasted it on empty
# vacuum -> stair-stepped edges). One ORTHOGONAL cascade over this short range beats 4 splits.
const SHADOW_MAX_DIST_M := 16.0

# --- Articulated EZ-RASSOR assembly (README §4 #11 follow-on) -----------------
# Kinematic tree transcribed from the EZ-RASSOR URDF (docs/ezrassor_assets.md §3),
# Z-up(meters)->Y-up via (x,y,z)_zup -> (x,z,-y)_yup. The URDF scale=0.35 macro
# applies ONLY to the MESH geometry (baked into the glbs by convert_rover_mesh.py),
# NOT to joint <origin> positions -- those are absolute meters and are mapped Z-up->
# Y-up with NO scale (standard URDF semantics). So 0.35-scaled meshes hang at the
# full-meter joint origins, giving the real stance: track 0.57 m (wheels outboard of
# the 0.34 m-wide body), wheelbase 0.40 m, drum arms reaching ~0.59 m fore/aft.
# Every continuous joint (4 wheels, 2 arms, 2 drums) rotates about the SAME local
# axis after the Y-up map: URDF (0,1,0)_zup -> (0,0,-1)_yup, i.e. local -Z.
const ROVER_JOINT_AXIS := Vector3(0, 0, -1)        # local spin/pitch axis (Y-up)
const ROVER_SCALE := 0.35                          # URDF mesh scale macro (mesh-only)

# Wheel pivot origins (Y-up m, unscaled): X fwd/back, Z left/right. r ~ 0.18, bottom y=-0.179.
const WHEEL_ORIGINS := {
	"LF": Vector3(0.20, 0.0, -0.285),
	"RF": Vector3(0.20, 0.0, 0.285),
	"LB": Vector3(-0.20, 0.0, -0.285),
	"RB": Vector3(-0.20, 0.0, 0.285),
}
# Arm pivot origins (Y-up m) at base_link +-0.20 zup-X -> +-0.20 yup-X.
const ARM_FRONT_ORIGIN := Vector3(0.20, 0.0, 0.0)
const ARM_BACK_ORIGIN := Vector3(-0.20, 0.0, 0.0)
# Drum pivot origin RELATIVE to its arm pivot (Y-up m): 0.388245 zup-X -> yup-X.
const DRUM_FRONT_REL := Vector3(0.388245, 0.0, 0.0)
const DRUM_BACK_REL := Vector3(-0.388245, 0.0, 0.0)

# Default recognizable pose (radians). Wheels resting; FRONT arm lowered so its
# drum reaches down toward the surface (digging-approach), BACK arm raised clear
# (transport) so the two arms read as independently articulated.
# DRUM SPINS are opposite-signed: the RASSOR signature "counter-rotating buckets"
# is NOT a kinematic property (both URDF drum axes are +Y) -- it is a CONTROL-LAYER
# convention produced by commanding opposite-sign drum velocities (sim_drums_driver;
# ezrassor_assets.md §3). We mirror it here purely as a pose so the buckets read.
const WHEEL_SPIN := 0.0                 # wheels resting flat
const ARM_FRONT_PITCH := 0.20           # front arm lowered ~11.5deg, drum near surface
const ARM_BACK_PITCH := 0.65            # back arm raised ~37deg, drum lifted clear
const DRUM_FRONT_SPIN := 0.5            # +  (counter-rotation convention)
const DRUM_BACK_SPIN := -0.5            # -  (opposite sign = counter-rotating)

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

# --- sequence (fly-through) mode state (INTERFACE.md §5.1 driven rover) --------
# When _seq_dir != "" the sidecar iterates the tNNN frames in ONE process. For
# each frame the rover is placed at rover_rc (surface-snapped) and yawed along the
# local path heading (from consecutive rover_rc). The active window + quadtree
# overlay follow the rover because they read sf.rover_rc / sf.active_leaves.
var _seq_dir := ""
var _seq_stride := 2
# Per-frame rover override (set by the sequence loop before _build_rover). When
# _rover_rc_override.x >= 0 the rover is placed there with yaw _rover_yaw instead
# of the static demo offset, so single-frame rover renders stay unchanged.
var _rover_rc_override := Vector2i(-1, -1)
var _rover_yaw := 0.0

func _ready() -> void:
	_parse_args()

	if _layers.is_empty():
		_layers = DEFAULT_LAYERS.split(",")

	get_window().size = _viewport_size

	if _seq_dir != "":
		await _run_sequence()
		return

	# --- single-frame mode (unchanged) ---
	if _scene_dir == "":
		push_error("sidecar: --scene <dir> or --sequence <dir> is required")
		get_tree().quit(2); return

	sf = StateFieldsScript.new()
	if not sf.load_scene(_scene_dir):
		push_error("sidecar: failed to load scene: " + sf.error_msg)
		get_tree().quit(3); return

	_setup_environment()
	_setup_camera()
	_build_layers()

	await _render_to(_out_path)
	print("sidecar: wrote ", ProjectSettings.globalize_path(_out_path),
		" size=", _viewport_size.x, "x", _viewport_size.y,
		" scene=", sf.scene_name, " layers=", ",".join(_layers))
	get_tree().quit(0)

# Wait the appropriate number of post-draw frames, then save the viewport to `path`.
func _render_to(path: String) -> bool:
	# Two post-draw waits: first frame may sample a stale buffer (per render_test.gd).
	# Extra waits when post-processing reads the back buffer (distortion) or when
	# GPUParticles need a frame to advance into their ballistic arc (dust).
	var waits := 2
	if _has("distortion") or _has("dust"):
		waits = 4
	for _w in range(waits):
		await RenderingServer.frame_post_draw
	var img := get_viewport().get_texture().get_image()
	var err := img.save_png(path)
	if err != OK:
		push_error("sidecar: save_png failed: %d for %s" % [err, path])
		return false
	return true

# ---------------------------------------------------------------------------
# SEQUENCE (fly-through) MODE — INTERFACE.md §5.1 driven-rover D4 headline.
# In ONE process (scene rasters loaded once per frame dir, scripts preloaded):
# iterate the tNNN frames at _seq_stride, and for each frame place the articulated
# rover at rover_rc (surface-snapped), yaw it along the local path heading (from
# consecutive rover_rc), move the active fine-mesh window + quadtree overlay (both
# read sf.rover_rc / sf.active_leaves), and save out/quadtree_flythrough_NNN.png.
# The environment + camera are built ONCE; only the per-frame layer nodes rebuild.
# ---------------------------------------------------------------------------
func _run_sequence() -> void:
	var dir := _seq_dir.trim_suffix("/")
	var frames := _list_frames(dir)
	if frames.is_empty():
		push_error("sidecar: --sequence found no tNNN frames under " + dir)
		get_tree().quit(2); return

	# Pre-scan every frame's rover_rc so we can compute path headings (and skip the
	# pre-drive null frame). We render only frames that HAVE a rover_rc.
	var all_rc: Array = []           # parallel to frames: Vector2i or (-1,-1)
	for fdir in frames:
		all_rc.append(_peek_rover_rc(fdir))

	# Load the FIRST frame's fields up front so the camera framing (which reads grid
	# extent / height_range) is valid; grid dims are constant across the series.
	sf = StateFieldsScript.new()
	if not sf.load_scene(frames[0]):
		push_error("sidecar: --sequence cannot load first frame: " + sf.error_msg)
		get_tree().quit(3); return

	# Build the static stage once (env + camera). Camera frames the whole drive.
	_setup_environment()
	_setup_camera_for_drive()

	var out_dir := "res://out"
	var n_written := 0
	var idx := 0
	while idx < frames.size():
		var rc: Vector2i = all_rc[idx]
		if rc.x < 0:
			idx += _seq_stride
			continue   # skip pre-drive / rover-less frames

		# Load this frame's fields.
		sf = StateFieldsScript.new()
		if not sf.load_scene(frames[idx]):
			push_warning("sidecar: seq skip %s: %s" % [frames[idx], sf.error_msg])
			idx += _seq_stride
			continue

		# Path heading from consecutive rover_rc (look ahead, else look back).
		_rover_rc_override = rc
		_rover_yaw = _heading_yaw(all_rc, idx)

		# Trailing chase camera follows the rover each frame (unless --pose pinned it).
		if not _has_pose:
			_update_trailing_camera(rc, _rover_yaw)

		# Rebuild only the per-frame layer nodes (terrain/active-window/overlay/rover).
		_clear_frame_nodes()
		_build_layers()

		var fname := "%s/quadtree_flythrough_%03d.png" % [out_dir, n_written]
		var ok := await _render_to(fname)
		if ok:
			print("sidecar: seq frame %d <- %s rover_rc=%s yaw=%.1fdeg active_leaves=%d -> %s" % [
				n_written, frames[idx].get_file(), str(rc), rad_to_deg(_rover_yaw),
				sf.active_leaves.size(), ProjectSettings.globalize_path(fname)])
			n_written += 1
		idx += _seq_stride

	print("sidecar: sequence wrote %d frames to %s (stride=%d)" % [
		n_written, ProjectSettings.globalize_path(out_dir), _seq_stride])
	get_tree().quit(0 if n_written > 0 else 5)

# List tNNN frame directories under `dir`, sorted ascending.
func _list_frames(dir: String) -> Array:
	var out: Array = []
	var d := DirAccess.open(dir)
	if d == null:
		return out
	d.list_dir_begin()
	var name := d.get_next()
	while name != "":
		if d.current_is_dir() and name.begins_with("t") and name.length() == 4 \
				and name.substr(1).is_valid_int():
			out.append(dir + "/" + name)
		name = d.get_next()
	d.list_dir_end()
	out.sort()
	return out

# Read just the rover_rc from a frame's metadata.json (cheap; no raster load).
func _peek_rover_rc(fdir: String) -> Vector2i:
	var f := FileAccess.open(fdir + "/metadata.json", FileAccess.READ)
	if f == null:
		return Vector2i(-1, -1)
	var parsed = JSON.parse_string(f.get_as_text())
	f.close()
	if typeof(parsed) != TYPE_DICTIONARY:
		return Vector2i(-1, -1)
	var rc = parsed.get("rover_rc", null)
	if typeof(rc) == TYPE_ARRAY and rc.size() == 2:
		return Vector2i(int(rc[0]), int(rc[1]))
	return Vector2i(-1, -1)

# Local path-heading yaw (radians about +Y) at frame index i, from the rover_rc
# delta (col->+X, row->+Z). Looks ahead to the next valid rc; falls back to the
# previous one. The rover's FORWARD axis is local +X (front wheels LF/RF sit at +X,
# the gauge runs along Z, wheels spin about Z) -> yaw must point +X along travel.
# Basis(UP, yaw) maps local +X to (cos yaw, 0, -sin yaw); aligning that to the travel
# vector (dx along +X, dz along +Z) gives yaw = atan2(-dz, dx). (The old atan2(dx, dz)
# oriented a +Z-forward model, so the rover slid 90deg sideways across its path.)
func _heading_yaw(all_rc: Array, i: int) -> float:
	var here: Vector2i = all_rc[i]
	var nxt := Vector2i(-1, -1)
	for j in range(i + _seq_stride, all_rc.size(), _seq_stride):
		if all_rc[j].x >= 0:
			nxt = all_rc[j]; break
	var prv := Vector2i(-1, -1)
	for j in range(i - _seq_stride, -1, -_seq_stride):
		if all_rc[j].x >= 0:
			prv = all_rc[j]; break
	var a := here; var b := here
	if nxt.x >= 0:
		b = nxt
		if prv.x >= 0:
			a = prv      # central difference when both neighbors exist
	elif prv.x >= 0:
		a = prv          # last frame: use incoming direction
	var dx := float(b.y - a.y)   # col delta -> +X
	var dz := float(b.x - a.x)   # row delta -> +Z
	if absf(dx) < 1e-6 and absf(dz) < 1e-6:
		return 0.0
	return atan2(-dz, dx)        # point rover forward (+X) along travel (see header)

# Camera that frames the whole driven path (diagonal across the field), oblique 3/4.
func _setup_camera_for_drive() -> void:
	_cam = Camera3D.new()
	_cam.fov = 55.0
	_cam.near = 0.02
	_cam.far = 100.0
	add_child(_cam)
	if _has_pose:
		_cam.look_at_from_position(_cam_pos, _cam_target, Vector3.UP)
		return
	# tread_track drives from ~rc(60,51) to ~rc(204,179): down-right diagonal in the
	# field. Frame it from the +X/+Z corner looking back toward the origin so the
	# whole trail + the moving fine cluster stay in view across all frames.
	var ext: Vector2 = sf.extent_m()
	var cx: float = sf.world_min.x + ext.x * 0.5
	var cz: float = sf.world_min.y + ext.y * 0.55
	_cam_pos = Vector3(cx + ext.x * 0.30, maxf(ext.x, ext.y) * 0.95, cz + ext.y * 0.85)
	_cam_target = Vector3(cx, sf.height_range.x, cz)
	_cam.look_at_from_position(_cam_pos, _cam_target, Vector3.UP)

# Per-frame trailing chase camera (sequence mode): sit behind + above the rover, offset to
# one side for a 3/4 view, looking just past it. The rover's FORWARD is local +X under the
# yaw basis (front wheels LF/RF at +X), so -forward is "behind". Follows rover_rc + heading.
func _update_trailing_camera(rc: Vector2i, yaw: float) -> void:
	var u: float = clampf(float(rc.y) / float(sf.width - 1), 0.0, 1.0)
	var v: float = clampf(float(rc.x) / float(sf.height - 1), 0.0, 1.0)
	var rover_pos := Vector3(sf.world_min.x + rc.y * sf.cell_m,
							 sf.height_uv(u, v),
							 sf.world_min.y + rc.x * sf.cell_m)
	var fwd := (Basis(Vector3.UP, yaw) * Vector3(1, 0, 0)).normalized()  # world forward (+X)
	var side := Vector3(-fwd.z, 0.0, fwd.x)                              # perpendicular in XZ
	var cam_pos: Vector3 = rover_pos - fwd * TRAIL_M + side * TRAIL_SIDE_M \
		+ Vector3(0.0, TRAIL_HEIGHT_M, 0.0)
	var look_at: Vector3 = rover_pos + fwd * 0.10 + Vector3(0.0, 0.25, 0.0)
	_cam.fov = TRAIL_FOV
	_cam.look_at_from_position(cam_pos, look_at, Vector3.UP)

# Remove only the per-frame layer nodes (terrain/clasts/rover/overlay) between
# sequence frames, leaving the sun + WorldEnvironment + camera in place.
func _clear_frame_nodes() -> void:
	for ch in get_children():
		if ch is Camera3D or ch is DirectionalLight3D or ch is WorldEnvironment:
			continue
		remove_child(ch)
		ch.queue_free()

# ---------------------------------------------------------------------------
func _parse_args() -> void:
	var args := OS.get_cmdline_user_args()  # everything after '--'
	var i := 0
	while i < args.size():
		var a := String(args[i])
		match a:
			"--scene":
				i += 1; _scene_dir = String(args[i])
			"--sequence":
				i += 1; _seq_dir = String(args[i])
			"--stride":
				i += 1; _seq_stride = maxi(1, int(args[i]))
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
	# The Sun subtends ~0.5deg from the Moon. A non-zero angular size turns on Godot's
	# PCSS-style penumbra: shadow edges stay crisp at the occluder and soften with distance
	# from it (physically correct, not a uniform blur). render_fidelity #1; needs
	# soft_shadow_filter_quality>0 (set in project.godot).
	sun.light_angular_distance = 0.5
	sun.shadow_enabled = true
	# A SINGLE high-res ORTHOGONAL cascade over the short SHADOW_MAX_DIST_M range, rather than
	# 4 PSSM splits spread across the default 100 m frustum: the splits were giving the 8192
	# atlas a huge per-texel footprint on this ~5 m patch -> stair-stepped, swimming shadow
	# edges (the dominant "plasticy" tell). Pulling the frustum in jumps texel density ~6x.
	sun.directional_shadow_mode = DirectionalLight3D.SHADOW_ORTHOGONAL
	sun.directional_shadow_max_distance = SHADOW_MAX_DIST_M
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
	# The "quadtree" layer is an additive wireframe LOD overlay (built inside the
	# terrain node) that mirrors the 4a filmstrip colors (INTERFACE.md §5.1).
	var show_qt := _has("quadtree")
	var terrain = TerrainScript.new()
	if _has("heightmap"):
		terrain.build(sf, TerrainScript.Mode.FALSECOLOR_HEIGHT, show_qt)
		add_child(terrain)
	elif _has("state"):
		terrain.build(sf, TerrainScript.Mode.FALSECOLOR_STATE, show_qt)
		add_child(terrain)
	elif _has("terrain"):
		terrain.build(sf, TerrainScript.Mode.LIT_PBR, show_qt)
		add_child(terrain)
	elif show_qt:
		# quadtree overlay requested without a terrain mesh: still build the node
		# so the wireframe alone is visible (diagnostic).
		terrain.build(sf, TerrainScript.Mode.LIT_PBR, true)
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
# Layer (asset) — the real EZ-RASSOR rover (MIT, vendored; see THIRD_PARTY.md),
# assembled from the converted DAE->glb sub-parts by scripts/convert_rover_mesh.py.
# Loaded at RUNTIME via GLTFDocument so it works headless with no editor import step.
#
# DEFAULT path = the FULL ARTICULATED rover: a rover-root Node3D carrying the
# chassis (rover_body.glb, native base_link origin) + 4 wheels + 2 arms, each arm
# carrying a drum, all placed at the §3 joint origins (Y-up, in unscaled meters --
# only the MESHES are 0.35-scaled; joint origins are absolute). If the sub-part
# glbs are missing it FALLS BACK to the chassis-only path (rover_base.glb, the
# prior README #11 behavior) so the layer never hard-fails.
func _build_rover() -> void:
	var body_path := "res://assets/rover_body.glb"
	var have_parts := FileAccess.file_exists(body_path) \
		and FileAccess.file_exists("res://assets/wheel.glb") \
		and FileAccess.file_exists("res://assets/drum.glb") \
		and FileAccess.file_exists("res://assets/drum_arm.glb")
	if not have_parts:
		_build_rover_chassis_only()
		return

	# One faintly-metallic grey for the whole rover (the DAEs carried flat material
	# colors, no per-vertex); reads as hardware against the matte regolith.
	var rmat := StandardMaterial3D.new()
	rmat.albedo_color = Color(0.62, 0.63, 0.66)
	rmat.metallic = 0.35
	rmat.roughness = 0.55

	var root := Node3D.new()
	root.name = "RASSOR"

	# Chassis (base_link). rover_body.glb keeps its native origin so the body floats
	# above the wheel centers exactly as the URDF intends (body bottom ~ -0.06 m).
	var body := _load_rover_glb(body_path)
	if body == null:
		_build_rover_chassis_only()
		return
	body.name = "body"
	root.add_child(body)

	# 4 wheels — pivot Node3D at the joint origin, spin about local axis, mesh child.
	for key in WHEEL_ORIGINS.keys():
		var w := _make_joint("wheel_" + String(key), "res://assets/wheel.glb",
			WHEEL_ORIGINS[key], Basis.IDENTITY, WHEEL_SPIN, Basis.IDENTITY)
		if w != null:
			root.add_child(w)

	# 2 arms. URDF origin rpy bakes into the pivot's REST basis; the link's visual
	# rpy bakes into the mesh-child basis (so the arm mesh points the right way).
	#   front: origin rpy(pi,0,0) -> pivot rest Rx(pi); visual identity.
	#   back : origin rpy(0,0,0)  -> pivot rest identity; visual rpy(pi,0,pi) -> Rz(pi)*Rx(pi).
	var arm_front := _make_joint("arm_front", "res://assets/drum_arm.glb",
		ARM_FRONT_ORIGIN, Basis(Vector3.RIGHT, PI), ARM_FRONT_PITCH, Basis.IDENTITY)
	var arm_back := _make_joint("arm_back", "res://assets/drum_arm.glb",
		ARM_BACK_ORIGIN, Basis.IDENTITY, ARM_BACK_PITCH, Basis(Vector3(0, 0, 1), PI) * Basis(Vector3.RIGHT, PI))

	# 2 drums — children of their arm pivot, at the arm-relative joint origin.
	#   front drum: rel basis Rx(pi); visual identity.
	#   back  drum: rel basis Rx(pi); visual rpy(pi,0,pi) -> Rz(pi)*Rx(pi).
	if arm_front != null:
		var drum_front := _make_joint("drum_front", "res://assets/drum.glb",
			DRUM_FRONT_REL, Basis(Vector3.RIGHT, PI), DRUM_FRONT_SPIN, Basis.IDENTITY)
		if drum_front != null:
			arm_front.add_child(drum_front)
		root.add_child(arm_front)
	if arm_back != null:
		var drum_back := _make_joint("drum_back", "res://assets/drum.glb",
			DRUM_BACK_REL, Basis(Vector3.RIGHT, PI), DRUM_BACK_SPIN, Basis(Vector3(0, 0, 1), PI) * Basis(Vector3.RIGHT, PI))
		if drum_back != null:
			arm_back.add_child(drum_back)
		root.add_child(arm_back)

	_apply_material_recursive(root, rmat)

	# Placement. SEQUENCE/per-frame mode (override set, or rover_rc present): put
	# the rover at the driven footprint center rover_rc, snapped to the surface,
	# yawed along the path heading. Otherwise the static demo pose: offset from the
	# active-zone center so it sits on the plain/rim, yawed 35deg for the 3/4 view.
	var rx: float; var rz: float; var surf_y: float; var yaw: Basis
	var place_rc := _rover_rc_override
	if place_rc.x < 0 and sf.has_rover_rc:
		place_rc = sf.rover_rc
	if place_rc.x >= 0:
		var u: float = clampf(float(place_rc.y) / float(sf.width - 1), 0.0, 1.0)
		var v: float = clampf(float(place_rc.x) / float(sf.height - 1), 0.0, 1.0)
		rx = sf.world_min.x + place_rc.y * sf.cell_m   # col -> +X
		rz = sf.world_min.y + place_rc.x * sf.cell_m   # row -> +Z
		surf_y = sf.height_uv(u, v)
		yaw = Basis(Vector3.UP, _rover_yaw)
	else:
		var ext: Vector2 = sf.extent_m()
		rx = sf.world_min.x + ext.x * 0.5 + ext.x * 0.22
		rz = sf.world_min.y + ext.y * 0.5 + ext.y * 0.12
		var u: float = clampf((rx - sf.world_min.x) / ext.x, 0.0, 1.0)
		var v: float = clampf((rz - sf.world_min.y) / ext.y, 0.0, 1.0)
		surf_y = sf.height_uv(u, v)
		yaw = Basis(Vector3.UP, deg_to_rad(35.0))

	# GROUND-SNAP ONCE AT THE ROOT: orient (yaw) first, then measure the assembled
	# world AABB and offset so the LOWEST point (wheel bottoms) rests at surf_y. Do
	# NOT snap parts individually -- the wheels are the contact, drums hover above.
	root.transform = Transform3D(yaw, Vector3(rx, surf_y, rz))
	add_child(root)
	var aabb := _node_world_aabb(root)
	var drop := surf_y - aabb.position.y      # lift so min.y == surf_y
	root.position.y += drop
	print("sidecar: assembled articulated EZ-RASSOR (MIT) at (%.2f,%.2f,%.2f); " % [rx, root.position.y, rz],
		"AABB size=(%.2f,%.2f,%.2f) lowest_y=%.3f snapped_to=%.3f" % [
			aabb.size.x, aabb.size.y, aabb.size.z, aabb.position.y, surf_y])

# Build one revolute-joint subtree: a pivot Node3D at `origin` whose REST basis is
# `rest_basis`, rotated by `angle` about ROVER_JOINT_AXIS (the continuous joint),
# carrying a single mesh child with local basis `mesh_basis` (the link visual rpy).
# Returns the pivot, or null if the glb failed to load.
func _make_joint(node_name: String, glb_res: String, origin: Vector3,
		rest_basis: Basis, angle: float, mesh_basis: Basis) -> Node3D:
	var mesh := _load_rover_glb(glb_res)
	if mesh == null:
		return null
	var pivot := Node3D.new()
	pivot.name = node_name
	var spun := rest_basis * Basis(ROVER_JOINT_AXIS, angle)
	pivot.transform = Transform3D(spun, origin)
	mesh.transform = Transform3D(mesh_basis, Vector3.ZERO)
	pivot.add_child(mesh)
	return pivot

# Load a converted rover .glb at runtime (headless-safe, no editor import). Returns
# the generated scene root, or null on failure.
func _load_rover_glb(res_path: String) -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(ProjectSettings.globalize_path(res_path), state)
	if err != OK:
		push_warning("sidecar: rover glTF load failed for %s (%d)" % [res_path, err])
		return null
	var scene := doc.generate_scene(state)
	if scene == null:
		push_warning("sidecar: generate_scene returned null for %s" % res_path)
		return null
	return scene as Node3D

# World-space AABB enclosing every MeshInstance3D mesh under `node` (recursive),
# using each mesh's own AABB transformed by its global transform. Used for the
# single root ground-snap (wheel bottoms -> surface).
func _node_world_aabb(node: Node) -> AABB:
	var acc := AABB()
	var first := true
	for mi: MeshInstance3D in _collect_mesh_instances(node):
		if mi.mesh == null:
			continue
		var local: AABB = mi.mesh.get_aabb()
		var gx: Transform3D = mi.global_transform
		# Transform all 8 corners; union into world AABB.
		for ci in range(8):
			var corner := local.position + Vector3(
				local.size.x if (ci & 1) else 0.0,
				local.size.y if (ci & 2) else 0.0,
				local.size.z if (ci & 4) else 0.0)
			var wc: Vector3 = gx * corner
			if first:
				acc = AABB(wc, Vector3.ZERO); first = false
			else:
				acc = acc.expand(wc)
	return acc

func _collect_mesh_instances(node: Node) -> Array:
	var out: Array = []
	if node is MeshInstance3D:
		out.append(node)
	for ch in node.get_children():
		out.append_array(_collect_mesh_instances(ch))
	return out

# Chassis-only fallback (the prior README #11 behavior): the EZ-RASSOR base_unit
# chassis, ground-re-origined glb snapped straight to the terrain height.
func _build_rover_chassis_only() -> void:
	var res_path := "res://assets/rover_base.glb"
	if not FileAccess.file_exists(res_path):
		print("sidecar: rover layer requested but %s missing (run scripts/convert_rover_mesh.py)" % res_path)
		return
	var rover := _load_rover_glb(res_path)
	if rover == null:
		return
	var rmat := StandardMaterial3D.new()
	rmat.albedo_color = Color(0.62, 0.63, 0.66)
	rmat.metallic = 0.35
	rmat.roughness = 0.55
	_apply_material_recursive(rover, rmat)
	var ext: Vector2 = sf.extent_m()
	var rx: float = sf.world_min.x + ext.x * 0.5 + ext.x * 0.22
	var rz: float = sf.world_min.y + ext.y * 0.5 + ext.y * 0.12
	var u: float = clampf((rx - sf.world_min.x) / ext.x, 0.0, 1.0)
	var v: float = clampf((rz - sf.world_min.y) / ext.y, 0.0, 1.0)
	var ry: float = sf.height_uv(u, v)
	var basis := Basis(Vector3.UP, deg_to_rad(35.0))
	rover.transform = Transform3D(basis, Vector3(rx, ry, rz))
	add_child(rover)
	print("sidecar: placed RASSOR chassis-only (EZ-RASSOR base_unit, MIT) at (%.2f,%.2f,%.2f)" % [rx, ry, rz])

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
