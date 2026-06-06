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
#   --sun-elev / --sun-azim   sun elevation/azimuth deg (inspection; default grazing 5/215)
#   --exposure <f>            filmic tonemap exposure (default 1.2)
#   --brdf   hapke|lambert    terrain BRDF: Hapke/Lommel-Seeliger (default) or Lambert baseline
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
const AprilTagGenScript := preload("res://apriltag_gen.gd")
const CameraRigScript := preload("res://camera_rig.gd")
# THE shared schema-assembly sink (contract v1.1). _build_sensors_json + _build_lander
# moved here in L0 so the Wave-1 lanes never collide on sidecar.gd; sidecar keeps ONE
# delegating call-site per render path.
const SensorsEmitScript := preload("res://sensors_emit.gd")
# Wave-1 lane NO-OP skeletons (each owned by its lane; sidecar only DISPATCHES to them):
const CaptureSeqScript := preload("res://capture_seq.gd")        # M2-egress (--cameras-seq)
const SunSweepScript := preload("res://sun_sweep.gd")            # A2-sweep (--sun-sweep)
const LanderBundleScript := preload("res://lander_bundle.gd")    # M3-tag (--lander-faces)
const DepartSpiralScript := preload("res://depart_spiral.gd")    # DEMO (--depart-spiral)
const TopdownSpiralScript := preload("res://topdown_spiral.gd")  # DEMO (--topdown-spiral)

var _viewport_size := Vector2i(1024, 768)
var _out_path := DEFAULT_OUT
var _scene_dir := ""
var _layers: Array = []
var _cam_pos := Vector3(2.56, 2.2, 5.6)
var _cam_target := Vector3(2.56, -0.1, 2.56)
var _has_pose := false

# Sun direction (degrees). Defaults to the ~5deg grazing lunar sun (spec §8); overridable via
# --sun-elev / --sun-azim for INSPECTION renders (e.g. lighting an excavated floor that the
# grazing sun self-shadows). Elevation is the angle ABOVE the horizon; azimuth the compass dir.
var _sun_elev_deg := 5.0
var _sun_azim_deg := 215.0
# Tonemap exposure. Default tuned for the grazing sun; lower it for raised-sun inspection
# renders so a fully-lit surface doesn't clip to white (which flattens albedo differences).
var _exposure := 1.2
# Photometry / BRDF (render_fidelity). Hapke / Lommel-Seeliger is ON by default (every scene gets
# the airless-regolith BRDF); --brdf lambert flips it to the Lambert baseline for the A/B render.
var _brdf_hapke := true

# M0 SLAM-rig feasibility probe (--probe-multicam). When true, build the scene once and capture it
# from SEVERAL Camera3Ds via shared-World3D SubViewports, saving each independently — the mechanism
# the 8-camera rover rig (front/rear stereo, side mono, drum cams) will use. Retires the risk that
# multi-SubViewport capture works headless under xvfb+Vulkan and yields INDEPENDENT frames.
var _probe_multicam := false

# --bench-multicam: light render benchmark. Reuses the --probe-multicam
# shared-World3D SubViewport mechanism with _bench_cams cameras at --size, fps uncapped,
# timing _bench_frames frames in three phases (render-only / +readback / +PNG encode).
var _bench_multicam := false
var _bench_frames := 30
var _bench_cams := 8

# M1 front-stereo camera egress (--cameras). When true, build the scene + an
# AprilTag-bearing procedural lander ~2.5 m ahead of the rover, capture the
# front_left/front_right stereo pair via shared-World3D SubViewports (mirrors
# --probe-multicam), and write out/cam/<scene>/000/{front_left,front_right}.png +
# sensors.json per docs/sensor_bridge_contract.md §2. The Godot->ROS conversion is
# NOT done here (sensors.json is 100% Godot-frame; §3 is C1's job).
var _cameras_mode := false
# Wave-1 lane dispatch modes (skeletons land in L0; lanes fill their owned .gd in).
# --cameras-seq -> M2-egress multi-frame egress (capture_seq.gd). Inherits the live
#   --cameras side effect (_drums_up=true) so the drum arms clear the front-stereo FOV.
var _cameras_seq_mode := false
var _depart_spiral_mode := false        # --depart-spiral: fixed-center lander + spiral egress (DEMO)
var _tag_unlit := false                 # --tag-unlit: render lander tags UNSHADED (illumination A/B)
var _topdown_spiral_mode := false       # --topdown-spiral: bird's-eye ortho render of the spiral (DEMO)
var _scene_unlit := false               # --scene-unlit: bland Lambert + no shadows + spherical clasts (top-down diagnostic)
var _out_scene_name := ""               # --out-scene-name: override the out/cam/<name> dir (separate lit/unlit runs)
var _qt_leaves_path := ""               # --qt-leaves <path>: per-frame quadtree leaves for the top-down overlay
var _rover_pose_path := ""              # --rover-pose <path>: per-frame conform pose track (drive_spiral.py rover_pose.json)
var _td_follow_m := 0.0                 # --td-follow <span_m>: top-down camera FOLLOWS the rover at this span (0 = whole-patch fixed)
var _td_frameboth := false              # --td-frameboth: top-down cam frames BOTH rover + lander (zoomed lit so ruts resolve)
# --sun-sweep -> A2-sweep sun sweep + boulder manifest (sun_sweep.gd / boulder_manifest.gd).
var _sun_sweep_mode := false
# --lander-faces -> M3-tag 4-face AprilTag bundle (lander_bundle.gd).
var _lander_faces_mode := false
# --cameras tuning (detour ROS outputs): tag distance/angle + a rover sink (drop into a crater).
var _lander_standoff := -1.0    # metres ahead of rover; <0 -> LANDER_STANDOFF_M default (--lander-standoff)
var _lander_yaw_deg := 0.0      # yaw the tag face off-square for oblique fiducial views (--lander-yaw)
var _rover_sink := 0.0          # drop the rover this many metres into the terrain (--rover-sink)
var _cam_pitch_deg := 0.0       # downward pitch of the stereo pair so the ground fills frame (--cam-pitch)
var _drums_up := false          # raise BOTH drum arms clear of the camera module (--drums-up; auto in --cameras)
# selectable rover body (per-vehicle, terrain_authority/vehicles.py). Defaults = the EZ-RASSOR URDF stance,
# byte-identical to the WHEEL_ORIGINS/ARM consts below. assets/ipex + IPEx gauge/wheelbase render the CC0
# IPEx body (scripts/gen_ipex_mesh.py). The geometry numbers come from bodies.json _vehicles[name].
var _rover_assets := "res://assets"   # rover part-glb base dir (--rover-assets)
var _rover_gauge := 0.57              # lateral track [m] (--rover-gauge)
var _rover_wheelbase := 0.40          # fore/aft wheelbase [m] (--rover-wheelbase)

# Lander placement ahead of the rover along its forward (+X yawed) direction, so
# BOTH front cameras see the rover-facing tag face (contract §1/§5). [CALIB].
const LANDER_STANDOFF_M := 2.5         # metres in front of the rover
const APRILTAG_SIZE_M := 0.150         # §1 size_m (8x8 black-border square side)

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
# Terrain-conform tilt (rover-physics pass): the surface normal the rover's local +Y is
# tilted onto in _build_rover. Vector3.UP (the default) = flat -> non-conform paths render
# byte-identically. Set per frame by the spiral drivers from the drive_spiral.py pose track.
var _rover_up := Vector3.UP

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
	_build_layers()

	if _bench_multicam:
		await _bench_multicam_capture()
		get_tree().quit(0); return

	if _probe_multicam:
		await _probe_multicam_capture()
		get_tree().quit(0); return

	if _cameras_mode:
		await _cameras_capture()
		get_tree().quit(0); return

	# Wave-1 lane dispatch (skeletons land in L0; the owning lane fills its .gd in).
	if _cameras_seq_mode:
		# M2-egress: multi-frame egress (contract v1.1 §7). --cameras-seq set _drums_up.
		# MUST await: the entry is a coroutine (awaits frame_post_draw per frame); an
		# un-awaited call before quit(0) renders only one post-quit frame -> black egress.
		await CaptureSeqScript.run_capture_seq(self)
		get_tree().quit(0); return
	if _sun_sweep_mode:
		# A2-sweep: sun sweep + boulder manifest (docs/sun_sweep_manifest.md).
		await SunSweepScript.run_sun_sweep(self)
		get_tree().quit(0); return
	if _lander_faces_mode:
		# M3-tag: 4-face AprilTag bundle (contract v1.1 §3/§6). Reuses --cameras path.
		await LanderBundleScript.build_lander_faces(self)
		get_tree().quit(0); return

	if _depart_spiral_mode:
		# DEMO: fixed-center 4-face lander + spiral egress (demo_spiral_contract.md §2).
		# MUST await (coroutine awaiting frame_post_draw per frame; un-awaited -> black egress).
		await DepartSpiralScript.run_depart_spiral(self)
		get_tree().quit(0); return

	if _topdown_spiral_mode:
		# DEMO: bird's-eye orthographic render of the spiral (lit + unlit/quadtree variants).
		# MUST await (coroutine awaiting frame_post_draw per frame; un-awaited -> black frames).
		await TopdownSpiralScript.run_topdown_spiral(self)
		get_tree().quit(0); return

	_setup_camera()

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
			"--sun-elev":
				i += 1; _sun_elev_deg = float(args[i])
			"--sun-azim":
				i += 1; _sun_azim_deg = float(args[i])
			"--exposure":
				i += 1; _exposure = float(args[i])
			"--brdf":
				i += 1; _brdf_hapke = (String(args[i]).strip_edges().to_lower() != "lambert")
			"--probe-multicam":
				_probe_multicam = true
			"--bench-multicam":
				_bench_multicam = true
			"--bench-frames":
				i += 1; _bench_frames = int(args[i])
			"--bench-cams":
				i += 1; _bench_cams = int(args[i])
			"--cameras":
				_cameras_mode = true
				_drums_up = true                 # the camera module needs the drums lifted out of view
			"--cameras-seq":
				_cameras_seq_mode = true
				_drums_up = true                 # mirror --cameras: drums clear the front-stereo FOV
			"--sun-sweep":
				_sun_sweep_mode = true
			"--lander-faces":
				_lander_faces_mode = true
			"--depart-spiral":
				_depart_spiral_mode = true
				_drums_up = true                 # mirror --cameras: drums clear the front-stereo FOV
			"--tag-unlit":
				_tag_unlit = true                # DEMO illumination A/B: tags UNSHADED
			"--topdown-spiral":
				_topdown_spiral_mode = true
				_drums_up = true                 # mirror --depart-spiral: drums clear the FOV
			"--scene-unlit":
				_scene_unlit = true
				_brdf_hapke = false              # bland diagnostic: Lambert (no Hapke); topdown_spiral flattens shadows/ambient
			"--out-scene-name":
				i += 1; _out_scene_name = String(args[i])
			"--qt-leaves":
				i += 1; _qt_leaves_path = String(args[i])
			"--rover-pose":
				i += 1; _rover_pose_path = String(args[i])
			"--td-follow":
				i += 1; _td_follow_m = float(args[i])
			"--td-frameboth":
				_td_frameboth = true
			"--drums-up":
				_drums_up = true
			"--lander-standoff":
				i += 1; _lander_standoff = float(args[i])
			"--lander-yaw":
				i += 1; _lander_yaw_deg = float(args[i])
			"--rover-rc":
				i += 1
				var rc := String(args[i]).split(",")
				if rc.size() == 2:
					_rover_rc_override = Vector2i(int(rc[0]), int(rc[1]))
			"--rover-sink":
				i += 1; _rover_sink = float(args[i])
			"--rover-assets":
				i += 1; _rover_assets = String(args[i])
			"--rover-gauge":
				i += 1; _rover_gauge = float(args[i])
			"--rover-wheelbase":
				i += 1; _rover_wheelbase = float(args[i])
			"--cam-pitch":
				i += 1; _cam_pitch_deg = float(args[i])
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
	# Azimuth chosen to rake across the camera-facing terrain so relief reads, while the far
	# crater wall stays in deep shadow (the perception hazard). Overridable via --sun-elev /
	# --sun-azim for INSPECTION renders (the grazing default self-shadows excavated floors).
	sun.rotation_degrees = Vector3(-_sun_elev_deg, _sun_azim_deg, 0.0)
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
	e.tonemap_exposure = _exposure
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
# M0 — multi-SubViewport headless-capture feasibility probe (--probe-multicam).
# The scene geometry has already been built into the MAIN window's World3D by
# _build_layers(). Here we render that SAME world from several Camera3Ds, each in
# its own SubViewport that SHARES the world (sv.world_3d = main world), and save
# each SubViewport's texture independently. This is exactly the 8-camera rover-rig
# mechanism (M1): one scene, many cameras, many independent frames per render.
func _probe_multicam_capture() -> void:
	var world := get_viewport().world_3d
	var ext: Vector2 = sf.extent_m()
	var cx: float = sf.world_min.x + ext.x * 0.5
	var cz: float = sf.world_min.y + ext.y * 0.5
	var look := Vector3(cx, sf.height_range.x, cz)
	var span: float = maxf(ext.x, ext.y)
	# Four deliberately-distinct viewpoints — opposing obliques + a side + a top-down —
	# so independence is unambiguous (front vs rear must differ; top is clearly not either).
	var specs := [
		{"name": "front", "pos": Vector3(cx, span * 0.55, cz + ext.y * 0.9), "up": Vector3.UP},
		{"name": "rear",  "pos": Vector3(cx, span * 0.55, cz - ext.y * 0.9), "up": Vector3.UP},
		{"name": "left",  "pos": Vector3(cx - ext.x * 0.9, span * 0.45, cz), "up": Vector3.UP},
		{"name": "top",   "pos": Vector3(cx, span * 1.3, cz),                "up": Vector3(0, 0, -1)},
	]
	var probe_size := Vector2i(640, 480)
	var subs: Array = []
	for s in specs:
		var sv := SubViewport.new()
		sv.size = probe_size
		sv.world_3d = world                                       # SHARE the built scene
		sv.render_target_update_mode = SubViewport.UPDATE_ALWAYS
		sv.render_target_clear_mode = SubViewport.CLEAR_MODE_ALWAYS
		add_child(sv)
		var cam := Camera3D.new()
		cam.fov = 55.0
		cam.near = 0.02
		cam.far = 100.0
		sv.add_child(cam)
		cam.look_at_from_position(s["pos"], look, s["up"])
		cam.current = true                                        # active cam for THIS subviewport
		subs.append({"sv": sv, "name": String(s["name"])})
	# Let the subviewports render a few times (first frame can sample a stale buffer).
	for _w in range(3):
		await RenderingServer.frame_post_draw
	for e in subs:
		var img: Image = e["sv"].get_texture().get_image()
		var path := "res://out/probe_cam_%s.png" % e["name"]
		var err := img.save_png(path)
		print("sidecar: probe cam '%s' -> %s (%dx%d) err=%d" % [
			e["name"], ProjectSettings.globalize_path(path),
			img.get_width(), img.get_height(), err])
	print("sidecar: --probe-multicam wrote %d independent SubViewport frames" % subs.size())

# --bench-multicam — light render benchmark (additive). Builds _bench_cams
# cameras over the SAME world via the proven shared-World3D SubViewport mechanism, uncaps
# fps, and times _bench_frames frames in three phases so GPU draw is separated from the
# CPU readback + PNG-encode cost: A render-only cadence, B +get_image() readback, C +save_png.
func _bench_multicam_capture() -> void:
	Engine.max_fps = 0
	DisplayServer.window_set_vsync_mode(DisplayServer.VSYNC_DISABLED)
	var world := get_viewport().world_3d
	var ext: Vector2 = sf.extent_m()
	var cx: float = sf.world_min.x + ext.x * 0.5
	var cz: float = sf.world_min.y + ext.y * 0.5
	var look := Vector3(cx, sf.height_range.x, cz)
	var span: float = maxf(ext.x, ext.y)
	var subs: Array = []
	for k in range(_bench_cams):
		var ang: float = TAU * float(k) / float(_bench_cams)
		var sv := SubViewport.new()
		sv.size = _viewport_size
		sv.world_3d = world                                       # SHARE the built scene
		sv.render_target_update_mode = SubViewport.UPDATE_ALWAYS
		sv.render_target_clear_mode = SubViewport.CLEAR_MODE_ALWAYS
		add_child(sv)
		var cam := Camera3D.new()
		cam.fov = 55.0; cam.near = 0.02; cam.far = 200.0
		var pos := Vector3(cx + cos(ang) * ext.x * 0.7, span * 0.5, cz + sin(ang) * ext.y * 0.7)
		sv.add_child(cam)
		cam.look_at_from_position(pos, look, Vector3.UP)
		cam.current = true
		subs.append(sv)
	for _w in range(5):                                            # warm: shader compile + stale buffers
		await RenderingServer.frame_post_draw
	var n: int = _bench_frames
	var a0 := Time.get_ticks_usec()                                # A: render-only cadence
	for _f in range(n):
		await RenderingServer.frame_post_draw
	var tA: float = float(Time.get_ticks_usec() - a0) / 1000.0
	var b0 := Time.get_ticks_usec()                                # B: + GPU->CPU readback
	for _f in range(n):
		await RenderingServer.frame_post_draw
		for sv2 in subs:
			var _imgb: Image = sv2.get_texture().get_image()
	var tB: float = float(Time.get_ticks_usec() - b0) / 1000.0
	var c0 := Time.get_ticks_usec()                                # C: + PNG encode to disk
	for _f in range(n):
		await RenderingServer.frame_post_draw
		for ci in range(subs.size()):
			var imgc: Image = subs[ci].get_texture().get_image()
			imgc.save_png("res://out/bench_cam_%d.png" % ci)
	var tC: float = float(Time.get_ticks_usec() - c0) / 1000.0
	var mA := tA / n; var mB := tB / n; var mC := tC / n
	print("BENCH cams=%d size=%dx%d frames=%d (+main viewport)" % [_bench_cams, _viewport_size.x, _viewport_size.y, n])
	print("BENCH A render_only  %.2f ms/frame  %.1f fps" % [mA, 1000.0 / maxf(mA, 0.001)])
	print("BENCH B +readback    %.2f ms/frame  %.1f fps" % [mB, 1000.0 / maxf(mB, 0.001)])
	print("BENCH C +png_encode  %.2f ms/frame  %.1f fps  (%.2f ms/cam)" % [mC, 1000.0 / maxf(mC, 0.001), mC / _bench_cams])

# ---------------------------------------------------------------------------
# M1 — front-stereo camera egress (--cameras). docs/sensor_bridge_contract.md §2.
# The scene + rover were already built into this node's World3D by _build_layers().
# Here we (1) place an AprilTag-bearing procedural lander ~2.5 m IN FRONT of the
# rover so both front cameras see the id-0 tag, (2) build the front-stereo cameras
# via camera_rig.gd (shared-World3D SubViewports, exactly like the probe), (3)
# render a few frames, (4) save the two PNGs + a schema-valid sensors.json.
#
# All poses written are 100% GODOT-frame (contract §3: the REP-103 conversion is
# C1's job, NOT here). We do NOT pre-compose any camera->tag truth transform (C1
# computes it from the exact poses we emit).
func _cameras_capture() -> void:
	# The rover root added by _build_rover() (named "RASSOR" or "RASSOR_chassis").
	var rover_root := _find_rover_root()
	if rover_root == null:
		push_error("sidecar: --cameras requires the 'rover' layer (no rover root found); add 'rover' to --layers")
		get_tree().quit(4); return

	# Rover forward (+X local) in world, projected onto the XZ plane (yaw only) so
	# the lander stands at the rover's heading on the surface ahead of it.
	var rover_xf: Transform3D = rover_root.global_transform
	var fwd: Vector3 = rover_xf.basis * Vector3(1, 0, 0)
	fwd.y = 0.0
	if fwd.length() < 1e-5:
		fwd = Vector3(1, 0, 0)
	fwd = fwd.normalized()

	# Delegate the procedural lander build into the shared sink (SensorsEmit). Pass the
	# sidecar's live --lander-standoff / --lander-yaw tuning + the preloaded AprilTagGen
	# so the behavior is unchanged from the in-sidecar _build_lander it replaced.
	var lander_root := SensorsEmitScript.build_lander(
		self, sf, AprilTagGenScript, rover_xf.origin, fwd, _lander_standoff, _lander_yaw_deg)

	# Build the front-stereo cameras (shared World3D SubViewports riding the rover).
	var world := get_viewport().world_3d
	var cams: Array = CameraRigScript.build(self, rover_root, world, _viewport_size, _cam_pitch_deg)

	# Let the subviewports render a few times (first frame can sample a stale buffer).
	for _w in range(3):
		await RenderingServer.frame_post_draw

	# --- write the per-camera PNGs under out/cam/<scene>/000/ -----------------
	var scene: String = sf.scene_name
	var out_dir := "res://out/cam/%s/000" % scene
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(out_dir))
	for e in cams:
		var img: Image = e["sv"].get_texture().get_image()
		var path := "%s/%s" % [out_dir, e["image"]]
		var err := img.save_png(path)
		if err != OK:
			push_error("sidecar: --cameras save_png failed (%d) for %s" % [err, path])
		else:
			print("sidecar: --cameras wrote %s (%dx%d)" % [
				ProjectSettings.globalize_path(path), img.get_width(), img.get_height()])

	# --- assemble + write sensors.json (contract §2.2 / v1.1, all Godot-frame) ---
	# Delegate to the shared sink (SensorsEmit). Single-frame --cameras passes
	# frame_index 0 + the live sun block; faces[] stays null (single id-0 tag until
	# the M3-tag lane lands). The 8-cam rig (M3-cam) carries a rear pair, so
	# rear_pair_descriptor populates the v1.1 additive top-level "stereo_rear"
	# (NEVER replacing "stereo" = front pair; contract §2.2 / §4).
	var sun := SensorsEmitScript.sun_block(_sun_elev_deg, _sun_azim_deg, 0.0)
	var doc := SensorsEmitScript.build_sensors_json(
		scene, 0, _viewport_size, rover_root, lander_root, cams,
		Callable(CameraRigScript, "intrinsics"), CameraRigScript.FOV_X_DEG,
		sun, null, CameraRigScript.rear_pair_descriptor(cams, rover_root))
	var json_path := "%s/sensors.json" % out_dir
	var jf := FileAccess.open(json_path, FileAccess.WRITE)
	if jf == null:
		push_error("sidecar: --cameras cannot open %s for write" % json_path)
		return
	jf.store_string(JSON.stringify(doc, "  "))
	jf.close()
	print("sidecar: --cameras wrote %s (baseline_m=%.4f)" % [
		ProjectSettings.globalize_path(json_path), doc["stereo"]["baseline_m"]])

# Find the rover root added by _build_rover(): the articulated path names it
# "RASSOR". The chassis-only fallback adds an unnamed glTF scene root directly to
# self; we identify that by being a non-stage, non-terrain, non-clast Node3D that
# carries glTF mesh children. (In normal runs the assets exist -> "RASSOR".)
func _find_rover_root() -> Node3D:
	for ch in get_children():
		if ch is Node3D and (ch as Node3D).name == "RASSOR":
			return ch as Node3D
	for ch in get_children():
		if ch is Camera3D or ch is DirectionalLight3D or ch is WorldEnvironment:
			continue
		if ch is MultiMeshInstance3D:
			continue                              # clasts layer
		if ch.get_script() == TerrainScript:
			continue                              # terrain layer
		if ch is Node3D and _collect_mesh_instances(ch).size() > 0:
			return ch as Node3D                   # chassis-only glTF root
	return null

# ---------------------------------------------------------------------------
func _build_layers() -> void:
	# Photometry toggle (render_fidelity; --brdf lambert|hapke) flows into the terrain materials
	# via sf. Set here so it applies on every (re)build, including each --sequence frame.
	if sf != null:
		sf.hapke_enabled = _brdf_hapke
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
	# Denser tessellation so the per-instance triaxial scale + clast.gdshader's object-space lump/void
	# displacement read cleanly (no gross faceting at the silhouette under the grazing sun). Cheap for
	# the ~150 clasts in these scenes; the displacement is what makes them rock-like, not CG spheres.
	sphere.radial_segments = 24
	sphere.rings = 16
	# Hapke / Lommel-Seeliger BRDF, the SAME airless-regolith photometry as the terrain
	# (clast.gdshader), so the boulders read as lit rock rather than blown-white Lambert blobs.
	# The --brdf flag (sf.hapke_enabled, set in _build_layers) drives them in lockstep with the
	# terrain for the A/B comparison; params come from sf (literature defaults / scene override).
	var mat := ShaderMaterial.new()
	mat.shader = load("res://clast.gdshader")
	mat.set_shader_parameter("hapke_enabled", sf.hapke_enabled)
	mat.set_shader_parameter("hapke_b", sf.hapke_b)
	mat.set_shader_parameter("hapke_c", sf.hapke_c)
	mat.set_shader_parameter("hapke_B0", sf.hapke_B0)
	mat.set_shader_parameter("hapke_h", sf.hapke_h)
	mat.set_shader_parameter("hapke_gain", sf.hapke_gain)
	# Procgen relief tuned for ANGULAR / faceted rock (spec §9 "Angular, minimally eroded — sharp
	# grains, no water/wind rounding"). The shader's low-freq displacement is now ridged + terraced
	# (see clast.gdshader), so we push the amplitudes up enough that the faceting reads clearly at
	# both a grazing (~5 deg) and a raised (~28 deg) sun without breaking the silhouette: surf_amp
	# up from 0.15 -> 0.34 (deeper facets), detail_amp up from 0.6 -> 1.1 (crisper micro-fracture).
	# facet_levels=3 -> a few big flat faces; ridge_mix=0.95 -> nearly fully sharp (angular) crests.
	mat.set_shader_parameter("surf_amp", 0.34)
	mat.set_shader_parameter("surf_freq", 1.9)
	mat.set_shader_parameter("facet_levels", 3.0)
	mat.set_shader_parameter("ridge_mix", 0.95)
	mat.set_shader_parameter("detail_amp", 1.1)
	mat.set_shader_parameter("detail_freq", 16.0)
	if _scene_unlit:
		# DEMO bland top-down (--scene-unlit): render clasts as PLAIN spheres -- zero the procgen
		# facet/void/detail relief so the diagnostic view reads geometry + quadtree, not rock texture.
		mat.set_shader_parameter("surf_amp", 0.0)
		mat.set_shader_parameter("detail_amp", 0.0)
		mat.set_shader_parameter("void_amp", 0.0)
	sphere.material = mat

	var mm := MultiMesh.new()
	mm.transform_format = MultiMesh.TRANSFORM_3D
	# Per-instance custom data carries (seed, elong, void_gate, 0) to clast.gdshader's
	# INSTANCE_CUSTOM so its procgen relief is stable per boulder (NOT keyed on VERTEX, which would
	# swim/repeat). Godot 4.6 requires use_custom_data be set BEFORE instance_count is assigned.
	mm.use_custom_data = true
	mm.mesh = sphere
	mm.instance_count = sf.clasts.size()
	# Lunar fragments are equant-to-moderately-elongate, NOT spheres: Tsuchiyama et al. (2022, EPS
	# 74:172) report whole-sample mean three-axial ratios S/I=0.770, I/L=0.758, S/L=0.581 for Apollo/
	# Luna regolith (papers/CITATIONS.md). We sample a:b:c ~ 1.0 : U(0.65,0.9) : U(0.5,0.75) (b≈I/L,
	# c≈S/L bands bracketing those means), then RENORMALIZE to geometric-mean 1.0 so the Golombek-SFD
	# diameter the physics chose (radius_m) is preserved — the shape varies, the equivalent size does
	# not. The SHORT axis is constrained ~vertical (boulders rest on a flat face; also keeps the
	# CPU-side buried_frac, which assumes a sphere of radius_m, physically sane).
	for i in range(sf.clasts.size()):
		var c: Dictionary = sf.clasts[i]
		var ctr = c.get("center_m", [0, 0, 0])
		var rad := float(c.get("radius_m", 0.05))
		var pos := Vector3(float(ctr[0]), float(ctr[1]), float(ctr[2]))

		# Deterministic per-instance RNG seeded from the clast id (fall back to a center-derived
		# hash so seeding is stable even if id is missing). Same seed feeds the shader (custom data).
		var cid: int = int(c.get("id", -1))
		var seed_src: int = cid
		if cid < 0:
			seed_src = hash(pos)            # stable hash of the center vector
		var rng := RandomNumberGenerator.new()
		rng.seed = hash(seed_src)           # spread small sequential ids across the RNG space

		# Triaxial axial ratios (a=1 longest, b intermediate, c shortest), lunar-fragment stats.
		var b_ratio := rng.randf_range(0.65, 0.90)   # b/a  ~ I/L
		var c_ratio := rng.randf_range(0.50, 0.75)   # c/a  ~ S/L
		# Renormalize so geo-mean(1, b, c) == 1 -> the equivalent (Golombek SFD) radius is preserved.
		var gmean := pow(1.0 * b_ratio * c_ratio, 1.0 / 3.0)
		var sa := 1.0 / gmean                          # along longest axis
		var sb := b_ratio / gmean                       # along intermediate axis
		var sc := c_ratio / gmean                       # along shortest axis
		if _scene_unlit:                                 # DEMO bland top-down: TRUE spheres (no triaxial shape)
			sa = 1.0; sb = 1.0; sc = 1.0
		# elongation proxy for the shader (1 = sphere; higher = more elongate) — informational.
		var elong := clampf(sa / maxf(sc, 1e-3), 1.0, 4.0)

		# Hashed rest orientation with the SHORT axis (sc) ~vertical. Build a random yaw about Y plus
		# a small tilt so boulders aren't all axis-aligned, but the short axis stays near +Y (rests on
		# a flat face). Local axes: X=longest, Z=intermediate, Y=shortest(~up).
		var yaw := rng.randf_range(0.0, TAU)
		var tilt := rng.randf_range(-0.20, 0.20)       # ~+/-11.5 deg off vertical
		var tilt_dir := rng.randf_range(0.0, TAU)
		var rot := Basis(Vector3.UP, yaw)
		var tilt_axis := Vector3(cos(tilt_dir), 0.0, sin(tilt_dir))
		rot = Basis(tilt_axis, tilt) * rot

		# Compose: rotate, then scale per-axis by radius * axial ratio. X<-sa, Y<-sc, Z<-sb so the
		# short axis sits on local Y (~vertical after the small tilt).
		var basis := rot.scaled(Vector3(rad * sa, rad * sc, rad * sb))
		var xf := Transform3D(basis, pos)
		mm.set_instance_transform(i, xf)

		# Per-instance custom data for clast.gdshader: stable seed, elongation, and a void/concavity
		# gate (~35% of clasts carry a spall scar). Seed mapped to a bounded float so the shader hash
		# is well-conditioned; r is deterministic from cid via the same rng.
		var seed_f := float(absi(hash(seed_src)) % 100000) / 100000.0
		var void_gate := 1.0 if rng.randf() < 0.35 else 0.0
		mm.set_instance_custom_data(i, Color(seed_f, elong, void_gate, 0.0))
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
# Tilt a yaw-only basis so its local +Y aligns to `up` (a surface normal), preserving the
# heading as much as possible (geodesic up-align — the SAME idiom as the clast rest tilt at
# ~877). up==Vector3.UP (the flat default) returns the yaw basis UNCHANGED, so every
# non-conform path (static demo pose, sequence frames without --rover-pose) renders
# byte-identically. The ground-snap below still seats the lowest wheel at the surface.
func _tilt_to_up(yaw_basis: Basis, up: Vector3) -> Basis:
	var u := up.normalized()
	if u.length() < 0.5:
		return yaw_basis                      # degenerate / unset -> flat
	var ang := Vector3.UP.angle_to(u)
	if ang < 1e-4:
		return yaw_basis                      # already upright
	var axis := Vector3.UP.cross(u)
	if axis.length() < 1e-6:
		return yaw_basis                      # antiparallel guard (never for a surface normal)
	return Basis(axis.normalized(), ang) * yaw_basis


func _wheel_origins() -> Dictionary:
	# 4 wheel-pivot origins (Y-up m) from the selected body's track/wheelbase. Default 0.57/0.40
	# reproduces the EZ-RASSOR WHEEL_ORIGINS const exactly (X = +-wheelbase/2, Z = +-gauge/2).
	var hg := _rover_gauge * 0.5
	var hb := _rover_wheelbase * 0.5
	return {
		"LF": Vector3(hb, 0.0, -hg), "RF": Vector3(hb, 0.0, hg),
		"LB": Vector3(-hb, 0.0, -hg), "RB": Vector3(-hb, 0.0, hg),
	}


func _build_rover() -> void:
	var body_path := _rover_assets + "/rover_body.glb"
	var have_parts := FileAccess.file_exists(body_path) \
		and FileAccess.file_exists(_rover_assets + "/wheel.glb") \
		and FileAccess.file_exists(_rover_assets + "/drum.glb") \
		and FileAccess.file_exists(_rover_assets + "/drum_arm.glb")
	if not have_parts:
		_build_rover_chassis_only()
		return

	# One faintly-metallic grey for the whole rover (the DAEs carried flat material
	# colors, no per-vertex); reads as hardware against the matte regolith.
	var rmat := _rover_material()

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

	# 4 wheels — pivot Node3D at the joint origin, spin about local axis, mesh child. Origins computed
	# from the selected body's gauge/wheelbase (default 0.57/0.40 reproduces WHEEL_ORIGINS exactly).
	var origins := _wheel_origins()
	for key in origins.keys():
		var w := _make_joint("wheel_" + String(key), _rover_assets + "/wheel.glb",
			origins[key], Basis.IDENTITY, WHEEL_SPIN, Basis.IDENTITY)
		if w != null:
			root.add_child(w)

	# Drum-arm pitches. Default demo pose: front lowered (digging), back raised (transport). With
	# _drums_up (camera module active, --cameras/--drums-up) BOTH arms swing high & clear so the
	# drums don't occlude the forward stereo pair (John: "drums lift up and out of the way of the
	# camera module"). 1.15 rad > the 0.65 "raised clear" back-arm rest, so both are well lifted.
	var arm_front_pitch := ARM_FRONT_PITCH
	var arm_back_pitch := ARM_BACK_PITCH
	if _drums_up:
		arm_front_pitch = 1.15
		arm_back_pitch = 1.15

	# 2 arms. URDF origin rpy bakes into the pivot's REST basis; the link's visual
	# rpy bakes into the mesh-child basis (so the arm mesh points the right way).
	#   front: origin rpy(pi,0,0) -> pivot rest Rx(pi); visual identity.
	#   back : origin rpy(0,0,0)  -> pivot rest identity; visual rpy(pi,0,pi) -> Rz(pi)*Rx(pi).
	var arm_front_origin := Vector3(_rover_wheelbase * 0.5, 0.0, 0.0)
	var arm_back_origin := Vector3(-_rover_wheelbase * 0.5, 0.0, 0.0)
	var arm_front := _make_joint("arm_front", _rover_assets + "/drum_arm.glb",
		arm_front_origin, Basis(Vector3.RIGHT, PI), arm_front_pitch, Basis.IDENTITY)
	var arm_back := _make_joint("arm_back", _rover_assets + "/drum_arm.glb",
		arm_back_origin, Basis.IDENTITY, arm_back_pitch, Basis(Vector3(0, 0, 1), PI) * Basis(Vector3.RIGHT, PI))

	# 2 drums — children of their arm pivot, at the arm-relative joint origin.
	#   front drum: rel basis Rx(pi); visual identity.
	#   back  drum: rel basis Rx(pi); visual rpy(pi,0,pi) -> Rz(pi)*Rx(pi).
	if arm_front != null:
		var drum_front := _make_joint("drum_front", _rover_assets + "/drum.glb",
			DRUM_FRONT_REL, Basis(Vector3.RIGHT, PI), DRUM_FRONT_SPIN, Basis.IDENTITY)
		if drum_front != null:
			arm_front.add_child(drum_front)
		root.add_child(arm_front)
	if arm_back != null:
		var drum_back := _make_joint("drum_back", _rover_assets + "/drum.glb",
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
		# Terrain conform (rover-physics pass): tilt the yaw basis so local +Y aligns to the
		# wheel-plane normal _rover_up (drive_spiral.py conform_pose). _rover_up==UP -> unchanged.
		yaw = _tilt_to_up(Basis(Vector3.UP, _rover_yaw), _rover_up)
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
	root.position.y -= _rover_sink            # then drop INTO the terrain (--rover-sink; crater scenarios)
	var _body_label := "IPEx (CC0)" if _rover_assets.ends_with("/ipex") else "EZ-RASSOR (MIT)"
	print("sidecar: assembled articulated %s at (%.2f,%.2f,%.2f); " % [_body_label, rx, root.position.y, rz],
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
	var rmat := _rover_material()
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

# Worn rover material (rover.gdshader): procedural dust/scratch/grime over the metal so the rover
# reads used (not pristine CG) and isn't a textureless white that feeds passive-stereo streaks.
func _rover_material() -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = load("res://rover.gdshader")
	return m

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
