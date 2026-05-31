extends RefCounted
class_name StateFields
# GDScript loader for the FROZEN state-field interface contract (INTERFACE.md).
#
# This is the *consumer* half of the decoupling seam (spec §2 authority model,
# §4 physics->render interface). It never shares memory or types with the
# producer; it only parses the on-disk directory:
#   metadata.json  + heightmap/mass_areal/density/disturbance (.rf32 float32 LE)
#                  + state_label (.r8 uint8).
# Raw bytes only -> Image.create_from_data with FORMAT_RF / FORMAT_R8.
# No EXR, no PNG decode in the hot path (INTERFACE.md §2 — dependency-free).
#
# state_label enum (INTERFACE.md §4 / spec §6):
#   0 VIRGIN, 1 TREAD, 2 EXCAVATED, 3 SPOIL, 4 COMPACTED_BERM

# --- parsed metadata (kept as plain Dictionary; consumer reads what it needs) ---
var meta: Dictionary = {}
var width: int = 0
var height: int = 0
var cell_m: float = 0.0
var world_min := Vector2.ZERO          # (x0, z0) world min corner
var world_max := Vector2.ZERO          # (x1, z1)
var gravity_m_s2: float = 1.62
var height_range := Vector2(-1.0, 1.0) # [min,max] elevation (m), for false-color
var density_range := Vector2(1170.0, 1920.0)
var clasts: Array = []                 # [{center_m:[x,y,z], radius_m, ...}, ...]
var active_zone: Dictionary = {}       # {min_rc:[r,c], max_rc:[r,c]}
var quadtree: Array = []               # [{level,row0,col0,size,label}, ...]
var scene_name: String = ""

# --- OPTIONAL per-frame interaction-keyed quadtree (INTERFACE.md §5.1, v1.0.1) ---
# All ADDITIVE / back-compat: absent => has_rover_rc=false, empty arrays, and the
# consumer keeps its v1.0 behavior (static active_zone window). Boxes are
# [r0,c0,r1,c1] HALF-OPEN in cells (rows r0..r1-1, cols c0..c1-1) per §5.1.
var has_rover_rc: bool = false
var rover_rc := Vector2i(-1, -1)       # rover footprint CENTER [row,col], or (-1,-1)
var active_leaves: Array = []          # [[r0,c0,r1,c1], ...] FINE leaves under rover NOW
var touched_leaves: Array = []         # [[r0,c0,r1,c1], ...] cumulative refined trail
var quadtree_nodes: Array = []         # [{level,row0,col0,size,leaf}, ...] per-frame tiling
var quadtree_lod: Dictionary = {}      # {min_leaf,refine_factor,footprint_radius_cells,field_size}

# --- OPTIONAL per-frame per-wheel tracks & drum marks (INTERFACE.md §5.2, v1.0.2) ---
# ALL ADDITIVE / back-compat: absent => has_track_dir=false and tex_track_dir()
# returns a NEUTRAL texture (zero direction, zero phase) so the shader's cleat/teeth
# detail is fully suppressed and existing scenes (crater etc.) render exactly as
# v1.0.1. Each wheel/drum entry carries a contact polyline in BASE [row,col] cells,
# a heading (0=+col/+X, +pi/2=+row/+Z), a contact/swath width in metres, and (drums)
# a teeth pitch + phase. The renderer bakes these into a derived RGB direction+phase
# field (consumer-side, NOT a new on-disk raster; design: render_fidelity_spec.md §4.3).
var has_track_dir: bool = false
var wheel_tracks: Dictionary = {}      # {"LF":{points,heading_rad,slip,width_m}, ...}
var drum_marks: Array = []             # [{drum,swath,depth_m,width_m,teeth_count,teeth_pitch_m,phase}, ...]

# --- OPTIONAL variable-resolution refinement / tiles (INTERFACE.md §5.3, v1.0.2) ---
# ADDITIVE: the BASE rasters fully describe the scene; tiles[] are finer-resolution
# bundles over the rover corridor. Absent OR refinement.enabled=false => uniform base
# resolution = identical to v1.0.1. Parsed here for the consumer (terrain.gd) to
# optionally sample finer height in the corridor; a consumer that ignores tiles[]
# renders the base (coarser in the corridor, still correct).
var refinement: Dictionary = {}        # {enabled,base_cell_m,fine_cell_m,refine_where,fine_min_leaf}
var tiles: Array = []                  # [{id,region_rc:[r0,c0,r1,c1],cell_m,dir}, ...] valid tiles only

# Resolution of the baked track-direction+phase field (RGB8). It is sampled by
# terrain.gdshader in FULL-FIELD normalized UV (the same UV0 the active mesh carries),
# so it just needs to resolve the ~0.18 m wheel bands and the corridor at the field
# scale; 256 over a 5.12 m field = 2 cm/texel, matching the base raster. Kept modest
# so the per-frame CPU bake stays cheap (it is a sparse stamp along the polylines).
const TRACK_DIR_TEX_SIZE := 256
# Default neutral encoding: direction (R,G) packed as dir*0.5+0.5, so the zero vector
# encodes to 0.5; phase (B)=0. The shader reads length(dir_decoded) ~ 0 as "no mark".
const TRACK_DIR_NEUTRAL := Color(0.5, 0.5, 0.0, 1.0)
# Track-dir field, lazily built on first tex_track_dir() call (after load_scene).
var _img_track_dir: Image

# --- per-field Images (kept so we can both sample CPU-side and build textures) ---
var img_height: Image
var img_mass_areal: Image
var img_density: Image
var img_disturbance: Image
var img_state: Image

# --- OPTIONAL regolith model (INTERFACE.md additive; render_fidelity cut-depth albedo) ---
# When a scene declares the "uniform mantle" model (datum carries macro topography, a uniform
# regolith mantle of areal mass M0 sits on top), the renderer can compute a CUT-DEPTH /
# exposed-sublayer signal as the excavated areal-mass deficit (M0 - mass_areal): immune to
# compaction (which leaves mass_areal untouched) and to natural topography (which lives in
# datum). Absent => has_uniform_mantle=false and the cut-depth term is OFF (identical to before).
var has_uniform_mantle: bool = false
var mantle_areal_m0: float = 156.0        # Z_T(0.12 m) * RHO_SURFACE(1300) = pristine mantle kg/m^2
var mantle_surface_density: float = 1300.0
var cut_depth_full_m: float = 0.08        # removed thickness that maps to full fresh-albedo
# Fresh/mature reflectance ratio = albedo multiplier at full exposure. A SOURCED radiometric
# input (lunar soil maturity: immature/fresh soil is brighter than space-weathered; OMAT,
# Lucey et al.; Hapke 2001), NOT a look knob. Conservative default in the measured ~1.3-1.8 band.
var maturity_albedo_ratio: float = 1.4

# --- OPTIONAL photometry / BRDF (render_fidelity; Hapke / Lommel-Seeliger). These are PHYSICAL
# CONSTANTS of lunar regolith, not scene geometry, so they default to literature values and apply
# to EVERY scene (additive: existing scenes just get the correct airless-surface BRDF in place of
# Lambert). hapke_enabled is flipped to false by the sidecar --brdf lambert flag for the A/B
# comparison render. A scene MAY override the parameters via a "photometric_model" metadata block
# (e.g. highlands vs mare). Sources: Sato et al. 2014 (LROC-derived global Hapke maps, 643 nm) for
# the 2-term Henyey-Greenstein b,c; Hapke 2002 for the shadow-hiding opposition B0,h; Hapke
# 1981/2012 for the IMSA framework. The shader (terrain.gdshader light()) uses the per-texel
# ALBEDO as the single-scattering albedo w. See papers/CITATIONS.md.
var hapke_enabled: bool = true
var hapke_b: float = 0.26          # 2-term HG lobe width (mare, 643 nm; Sato et al. 2014)
var hapke_c: float = 0.08          # 2-term HG back/forward partition (mare; Sato et al. 2014)
var hapke_B0: float = 1.0          # shadow-hiding opposition amplitude (Hapke 2002, lunar)
var hapke_h: float = 0.06          # shadow-hiding opposition angular width [rad] (Hapke 2002)
var hapke_gain: float = 1.4        # radiance calibration -> Lambert-comparable mid-tone (documented; not a look knob)

# raw float views for CPU sampling (e.g. mesh vertex displacement, clast snap)
var _height_data: PackedFloat32Array
var _state_data: PackedByteArray

var loaded: bool = false
var error_msg: String = ""

# Load a scene directory per INTERFACE.md. Returns true on success.
func load_scene(dir_path: String) -> bool:
	loaded = false
	error_msg = ""
	var dir := dir_path.trim_suffix("/")

	# --- metadata first (INTERFACE.md §6: read metadata before opening rasters) ---
	var meta_path := dir + "/metadata.json"
	var mf := FileAccess.open(meta_path, FileAccess.READ)
	if mf == null:
		error_msg = "cannot open %s (err %d)" % [meta_path, FileAccess.get_open_error()]
		push_error(error_msg)
		return false
	var meta_txt := mf.get_as_text()
	mf.close()
	var parsed = JSON.parse_string(meta_txt)
	if typeof(parsed) != TYPE_DICTIONARY:
		error_msg = "metadata.json did not parse to a Dictionary"
		push_error(error_msg)
		return false
	meta = parsed

	var grid: Dictionary = meta.get("grid", {})
	width = int(grid.get("width", 0))
	height = int(grid.get("height", 0))
	cell_m = float(grid.get("cell_m", 0.0))
	if width <= 0 or height <= 0 or cell_m <= 0.0:
		error_msg = "bad grid dims in metadata: %dx%d cell=%f" % [width, height, cell_m]
		push_error(error_msg)
		return false

	var wb: Dictionary = meta.get("world_bounds_m", {})
	world_min = Vector2(float(wb.get("x0", 0.0)), float(wb.get("y0", 0.0)))
	world_max = Vector2(float(wb.get("x1", width * cell_m)), float(wb.get("y1", height * cell_m)))
	gravity_m_s2 = float(meta.get("gravity_m_s2", 1.62))
	scene_name = String(meta.get("scene_name", "scene"))

	var hr = meta.get("height_range_m", null)
	if typeof(hr) == TYPE_ARRAY and hr.size() == 2:
		height_range = Vector2(float(hr[0]), float(hr[1]))
	clasts = meta.get("clasts", [])
	active_zone = meta.get("active_zone", {})
	quadtree = meta.get("quadtree", [])

	# --- OPTIONAL per-frame keys (INTERFACE.md §5.1). Parsed only when present; ---
	# absent or null => leave has_rover_rc=false / empty so callers fall back to
	# the static active_zone window (back-compat with v1.0 frames).
	_parse_per_frame_keys()

	# --- rasters ---
	var fields: Dictionary = meta.get("fields", {})
	img_height = _load_rf(dir, _file_of(fields, "heightmap", "heightmap.rf32"))
	# mass_areal is OPTIONAL for the consumer (older renders never sampled it); load it when
	# present for the cut-depth term, but do NOT fail the scene if it is missing.
	img_mass_areal = _load_rf(dir, _file_of(fields, "mass_areal", "mass_areal.rf32"))
	img_density = _load_rf(dir, _file_of(fields, "density", "density.rf32"))
	img_disturbance = _load_rf(dir, _file_of(fields, "disturbance", "disturbance.rf32"))
	img_state = _load_r8(dir, _file_of(fields, "state_label", "state_label.r8"))
	if img_height == null or img_density == null or img_disturbance == null or img_state == null:
		return false  # error_msg already set by loader

	# CPU views for vertex sampling (heightmap is authoritative geometry).
	_cache_height_floats(dir, _file_of(fields, "heightmap", "heightmap.rf32"))

	# Derive a density display range from actual data (robust false-color).
	density_range = _image_minmax(img_density)

	loaded = true
	return true

func _file_of(fields: Dictionary, key: String, fallback: String) -> String:
	var f = fields.get(key, null)
	if typeof(f) == TYPE_DICTIONARY and f.has("file"):
		return String(f["file"])
	return fallback

# --- OPTIONAL v1.0.1 per-frame interaction-keyed quadtree (INTERFACE.md §5.1) ---
# Reads rover_rc / active_leaves / touched_leaves / quadtree_nodes / quadtree_lod
# when present. Everything stays back-compatible: a frame with none of these (or
# rover_rc:null, like tread_track/t000 pre-drive) leaves has_rover_rc=false and
# empty arrays, so consumers fall back to the static active_zone window.
func _parse_per_frame_keys() -> void:
	has_rover_rc = false
	rover_rc = Vector2i(-1, -1)
	active_leaves = []
	touched_leaves = []
	quadtree_nodes = []
	quadtree_lod = {}

	var rc = meta.get("rover_rc", null)
	if typeof(rc) == TYPE_ARRAY and rc.size() == 2:
		rover_rc = Vector2i(int(rc[0]), int(rc[1]))   # [row, col]
		has_rover_rc = true

	active_leaves = _coerce_boxes(meta.get("active_leaves", []))
	touched_leaves = _coerce_boxes(meta.get("touched_leaves", []))

	var qn = meta.get("quadtree_nodes", null)
	if typeof(qn) == TYPE_ARRAY:
		quadtree_nodes = qn
	var ql = meta.get("quadtree_lod", null)
	if typeof(ql) == TYPE_DICTIONARY:
		quadtree_lod = ql

	_parse_track_keys()
	_parse_refinement_keys()
	_parse_regolith_model()
	_parse_photometric_model()

# --- OPTIONAL uniform-mantle regolith model (cut-depth albedo). Feature-detect by the
# "regolith_model": {"uniform_mantle": true, ...} block; absent => term stays OFF. ---
func _parse_regolith_model() -> void:
	has_uniform_mantle = false
	var rm = meta.get("regolith_model", null)
	if typeof(rm) == TYPE_DICTIONARY and bool(rm.get("uniform_mantle", false)):
		has_uniform_mantle = true
		mantle_areal_m0 = float(rm.get("mantle_areal_kg_m2", 156.0))
		mantle_surface_density = float(rm.get("surface_density", 1300.0))
		cut_depth_full_m = float(rm.get("cut_depth_full_m", 0.08))
		maturity_albedo_ratio = float(rm.get("maturity_albedo_ratio", 1.4))

# --- OPTIONAL photometric override (render_fidelity; Hapke). Lets a scene specify highlands- vs
# mare-specific Hapke parameters (else the literature mare defaults stand). hapke_enabled is NOT
# touched here -- the sidecar --brdf flag owns it. Feature-detect by a "photometric_model" dict. ---
func _parse_photometric_model() -> void:
	var pm = meta.get("photometric_model", null)
	if typeof(pm) == TYPE_DICTIONARY:
		hapke_b = float(pm.get("hg_b", hapke_b))
		hapke_c = float(pm.get("hg_c", hapke_c))
		hapke_B0 = float(pm.get("opposition_b0", hapke_B0))
		hapke_h = float(pm.get("opposition_h", hapke_h))
		hapke_gain = float(pm.get("radiance_gain", hapke_gain))

# --- OPTIONAL v1.0.2 per-wheel tracks & drum marks (INTERFACE.md §5.2) ---
# Feature-detect by key presence. has_track_dir becomes true only if at least one
# usable wheel/drum entry is found; otherwise tex_track_dir() stays neutral and the
# shader detail is suppressed (identical to v1.0.1). _img_track_dir is invalidated
# so a re-load rebakes lazily.
func _parse_track_keys() -> void:
	has_track_dir = false
	wheel_tracks = {}
	drum_marks = []
	_img_track_dir = null

	var wt = meta.get("wheel_tracks", null)
	if typeof(wt) == TYPE_DICTIONARY:
		for key in wt.keys():
			var w = wt[key]
			if typeof(w) != TYPE_DICTIONARY:
				continue
			var pts := _coerce_points(w.get("points", []))
			if pts.is_empty():
				continue
			wheel_tracks[String(key)] = {
				"points": pts,
				"heading_rad": float(w.get("heading_rad", 0.0)),
				"slip": clampf(float(w.get("slip", 0.0)), 0.0, 1.0),
				"width_m": float(w.get("width_m", 0.18)),
			}

	var dm = meta.get("drum_marks", null)
	if typeof(dm) == TYPE_ARRAY:
		for d in dm:
			if typeof(d) != TYPE_DICTIONARY:
				continue
			var swath := _coerce_points(d.get("swath", []))
			if swath.is_empty():
				continue
			drum_marks.append({
				"drum": String(d.get("drum", "front")),
				"swath": swath,
				"depth_m": float(d.get("depth_m", 0.03)),
				"width_m": float(d.get("width_m", 0.20)),
				"teeth_count": int(d.get("teeth_count", 8)),
				"teeth_pitch_m": float(d.get("teeth_pitch_m", 0.025)),
				"phase": float(d.get("phase", 0.0)),
			})

	has_track_dir = not (wheel_tracks.is_empty() and drum_marks.is_empty())

# --- OPTIONAL v1.0.2 variable-resolution refinement / tiles (INTERFACE.md §5.3) ---
# Feature-detect by key presence. Tiles that violate the §5.3 robustness rules
# (bad region, non-integer k, or refinement disabled) are dropped here so the
# consumer always has a clean, base-aligned, integer-k tile set (or falls back to
# base). refinement.enabled=false / refine_where=="none" => empty tiles (uniform base).
func _parse_refinement_keys() -> void:
	refinement = {}
	tiles = []

	var rf = meta.get("refinement", null)
	if typeof(rf) == TYPE_DICTIONARY:
		refinement = rf
	# enabled defaults true ONLY when the block exists; absent block => no refinement.
	var enabled := bool(refinement.get("enabled", false)) if not refinement.is_empty() else false
	var where := String(refinement.get("refine_where", "none"))
	if not enabled or where == "none":
		return

	var base_cell := float(refinement.get("base_cell_m", cell_m))
	var raw = meta.get("tiles", null)
	if typeof(raw) != TYPE_ARRAY:
		return
	for t in raw:
		if typeof(t) != TYPE_DICTIONARY:
			continue
		var region := _coerce_boxes([t.get("region_rc", [])])
		if region.is_empty():
			continue
		var rb: Array = region[0]
		var tcell := float(t.get("cell_m", 0.0))
		if tcell <= 0.0:
			continue
		# §5.3: k = base_cell_m / cell_m MUST be a positive integer; drop the tile
		# (fall back to base for its region) if not, never crash.
		var kf := base_cell / tcell
		var k := int(round(kf))
		if k < 1 or absf(kf - float(k)) > 1e-6:
			push_warning("sidecar: dropping tile id=%s: non-integer k=%f (base/cell)" % [
				str(t.get("id", -1)), kf])
			continue
		tiles.append({
			"id": int(t.get("id", -1)),
			"region_rc": rb,             # [r0,c0,r1,c1] half-open base cells
			"cell_m": tcell,
			"k": k,
			"dir": String(t.get("dir", "")),
		})

# Normalize a list of [r0,c0,r1,c1] half-open boxes into PackedInt-ish Arrays of 4
# ints, skipping malformed entries. Keeps the §5.1 box convention intact.
func _coerce_boxes(raw) -> Array:
	var out: Array = []
	if typeof(raw) != TYPE_ARRAY:
		return out
	for b in raw:
		if typeof(b) == TYPE_ARRAY and b.size() == 4:
			out.append([int(b[0]), int(b[1]), int(b[2]), int(b[3])])
	return out

# Normalize a polyline of [row,col] base-cell samples (§5.2 points/swath) into an
# Array of Vector2(row_float, col_float), skipping malformed entries. Kept as floats
# (sub-cell contact centers are allowed) in BASE-cell index space (§2/§3).
func _coerce_points(raw) -> Array:
	var out: Array = []
	if typeof(raw) != TYPE_ARRAY:
		return out
	for p in raw:
		if typeof(p) == TYPE_ARRAY and p.size() == 2:
			out.append(Vector2(float(p[0]), float(p[1])))   # (row, col)
	return out

# rover_rc as field row (for height/world lookups). Valid only if has_rover_rc.
func rover_row() -> int: return rover_rc.x
func rover_col() -> int: return rover_rc.y

# Bounding box (half-open [r0,c0,r1,c1] in cells) of the current active_leaves, or
# an empty Rect2i-style [0,0,0,0] when there are none. Used to place the fine mesh.
func active_leaves_bbox() -> Array:
	if active_leaves.is_empty():
		return [0, 0, 0, 0]
	var r0 := 1 << 30; var c0 := 1 << 30
	var r1 := -(1 << 30); var c1 := -(1 << 30)
	for b in active_leaves:
		r0 = mini(r0, int(b[0])); c0 = mini(c0, int(b[1]))
		r1 = maxi(r1, int(b[2])); c1 = maxi(c1, int(b[3]))
	return [r0, c0, r1, c1]

# Load a .rf32 raster as a single-channel float Image (FORMAT_RF).
func _load_rf(dir: String, fname: String) -> Image:
	var path := dir + "/" + fname
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		error_msg = "cannot open %s (err %d)" % [path, FileAccess.get_open_error()]
		push_error(error_msg)
		return null
	var bytes := f.get_buffer(f.get_length())
	f.close()
	var need := width * height * 4
	if bytes.size() != need:
		error_msg = "%s: got %d bytes, expected %d (w*h*4)" % [path, bytes.size(), need]
		push_error(error_msg)
		return null
	# FORMAT_RF == 32-bit float, 1 channel, row-major — exactly INTERFACE.md §2.
	return Image.create_from_data(width, height, false, Image.FORMAT_RF, bytes)

# Load a .r8 raster as a single-channel uint8 Image (FORMAT_R8).
func _load_r8(dir: String, fname: String) -> Image:
	var path := dir + "/" + fname
	var f := FileAccess.open(path, FileAccess.READ)
	if f == null:
		error_msg = "cannot open %s (err %d)" % [path, FileAccess.get_open_error()]
		push_error(error_msg)
		return null
	var bytes := f.get_buffer(f.get_length())
	f.close()
	var need := width * height
	if bytes.size() != need:
		error_msg = "%s: got %d bytes, expected %d (w*h)" % [path, bytes.size(), need]
		push_error(error_msg)
		return null
	return Image.create_from_data(width, height, false, Image.FORMAT_R8, bytes)

func _cache_height_floats(dir: String, fname: String) -> void:
	var f := FileAccess.open(dir + "/" + fname, FileAccess.READ)
	if f == null:
		return
	var bytes := f.get_buffer(f.get_length())
	f.close()
	_height_data = bytes.to_float32_array()

# --- accessors -------------------------------------------------------------

# height (m) at field index [row, col]; clamps to bounds.
func height_at(row: int, col: int) -> float:
	if _height_data.is_empty():
		return 0.0
	row = clampi(row, 0, height - 1)
	col = clampi(col, 0, width - 1)
	return _height_data[row * width + col]

# Bilinear height sample in *normalized* field UV (u along +X/col, v along +Z/row).
func height_uv(u: float, v: float) -> float:
	var fc := clampf(u, 0.0, 1.0) * float(width - 1)
	var fr := clampf(v, 0.0, 1.0) * float(height - 1)
	var c0 := int(floor(fc)); var r0 := int(floor(fr))
	var c1 := mini(c0 + 1, width - 1); var r1 := mini(r0 + 1, height - 1)
	var tx := fc - c0; var ty := fr - r0
	var h00 := height_at(r0, c0); var h10 := height_at(r0, c1)
	var h01 := height_at(r1, c0); var h11 := height_at(r1, c1)
	return lerp(lerp(h00, h10, tx), lerp(h01, h11, tx), ty)

# World extent in meters (x size, z size).
func extent_m() -> Vector2:
	return Vector2(width * cell_m, height * cell_m)

# Field [row,col] -> Godot world (x, y=height, z) per INTERFACE.md §3.
func world_pos(row: int, col: int) -> Vector3:
	return Vector3(world_min.x + col * cell_m, height_at(row, col), world_min.y + row * cell_m)

# Build an ImageTexture for shader sampling.
func tex_height() -> ImageTexture: return ImageTexture.create_from_image(img_height)
# mass_areal as an FP texture for the cut-depth term. If the raster was absent, return a 1x1
# texture holding a huge areal mass so (M0 - mass_areal) < 0 -> zero exposure (safely neutral).
func tex_mass_areal() -> ImageTexture:
	if img_mass_areal == null:
		var im := Image.create(1, 1, false, Image.FORMAT_RF)
		im.set_pixel(0, 0, Color(1.0e9, 0.0, 0.0, 1.0))
		return ImageTexture.create_from_image(im)
	return ImageTexture.create_from_image(img_mass_areal)
func tex_density() -> ImageTexture: return ImageTexture.create_from_image(img_density)
func tex_disturbance() -> ImageTexture: return ImageTexture.create_from_image(img_disturbance)
func tex_state() -> ImageTexture: return ImageTexture.create_from_image(img_state)

# --- Baked track-direction + phase field (consumer-side; INTERFACE.md §5.2 / ---
# render_fidelity_spec.md §4.3). A small RGB8 texture over the FULL field, sampled by
# terrain.gdshader in the same field UV0 the active mesh carries:
#   R,G = unit travel direction (field space: x=+col/+X, y=+row/+Z) packed dir*0.5+0.5
#   B   = phase/accumulator in [0,1] (arc-length along the track, wrapped), advancing
#         the cleat/teeth ridge pattern continuously along the corridor.
# When NO §5.2 keys are present the texture is NEUTRAL (zero direction, zero phase)
# everywhere, so the shader detail is fully suppressed and the LIT_PBR look of v1.0.1
# scenes is unchanged. Built lazily + cached. The travel direction (not the transverse
# ridge axis) is encoded; the shader rotates by 90deg to make TRANSVERSE cleats/teeth.
func tex_track_dir() -> ImageTexture:
	if _img_track_dir == null:
		_img_track_dir = _bake_track_dir()
	return ImageTexture.create_from_image(_img_track_dir)

# True if the baked track-dir field carries any non-neutral marks this frame (the
# shader still works either way; this lets terrain.gd skip work / log).
func has_track_marks() -> bool:
	return has_track_dir

# Adaptive track-dir field size so the 0.18 m wheel bands resolve at the FIELD scale: texel
# pitch ~0.10 m, clamped [TRACK_DIR_TEX_SIZE, 2048]. Small (5 m) scenes stay 256 (2 cm/texel,
# unchanged); the 220 m spiral patch climbs to ~2048 (~0.11 m/texel) so the carved ruts read
# as terrain features in the zoomed lit top-down instead of sub-texel smears.
func _track_tex_size() -> int:
	var extent_m := maxf(float(width), float(height)) * cell_m
	return clampi(int(round(extent_m / 0.10)), TRACK_DIR_TEX_SIZE, 2048)

func _bake_track_dir() -> Image:
	var sz := _track_tex_size()
	var img := Image.create(sz, sz, false, Image.FORMAT_RGB8)
	img.fill(TRACK_DIR_NEUTRAL)
	if not has_track_dir or width <= 1 or height <= 1:
		return img   # neutral => no marks (v1.0.1-identical)

	# Field [row,col] -> texel [tx,ty]. The field is row=+Z (=v), col=+X (=u); the
	# texture's x axis = col/u, y axis = row/v, matching the mesh UV0 (terrain.gd
	# writes uv = (col/(width-1), row/(height-1))). So texel = uv * (sz-1).
	var sx := float(sz - 1) / float(width - 1)
	var sy := float(sz - 1) / float(height - 1)

	# Wheel tracks: cleat phase advances per metre of travel; ridge period is the
	# grouser pitch (a shader uniform). We bake a per-texel ARC-LENGTH (metres) into
	# B (wrapped to [0,1] over a fixed 1 m window) so the shader's frac(B * 1m/pitch)
	# gives continuous transverse ridges along the contact band. Direction = local
	# travel dir of the nearest segment.
	for key in wheel_tracks.keys():
		var w: Dictionary = wheel_tracks[key]
		_stamp_polyline(img, w["points"], float(w["width_m"]), sx, sy, 0.0)

	# Drum swaths: same machinery, with the drum's own phase offset so the periodic
	# teeth align to the producer-reported phase.
	for d in drum_marks:
		_stamp_polyline(img, d["swath"], float(d["width_m"]), sx, sy, float(d["phase"]))

	return img

# Stamp one contact polyline (Array of Vector2(row,col) in BASE cells) into the
# track-dir field: for each segment, write the unit travel direction (R,G) and a
# wrapped arc-length phase (B) into every texel within `width_m`/2 of the segment.
# A single point stamps a small disc using the polyline's mean heading (or 0).
func _stamp_polyline(img: Image, pts: Array, width_m: float, sx: float, sy: float,
		phase0: float) -> void:
	if pts.is_empty():
		return
	var sz := img.get_width()
	# Half-band in texels: width_m/2 in metres -> cells (/cell_m) -> texels (*sx).
	var half_cells: float = 0.5 * width_m / maxf(cell_m, 1e-6)
	var half_tex := maxf(half_cells * sx, 1.0)
	var arc_m := phase0 * 1.0   # running arc length (m); seed with the reported phase

	# Single-point contact: stamp a disc, direction from heading-less fallback (0,+col).
	if pts.size() == 1:
		var p: Vector2 = pts[0]
		var dir := Vector2(1.0, 0.0)   # default travel +col/+X
		_stamp_disc(img, p, dir, arc_m, half_tex, sx, sy)
		return

	for i in range(pts.size() - 1):
		var a: Vector2 = pts[i]        # (row, col)
		var b: Vector2 = pts[i + 1]
		# Travel direction in FIELD space (x=+col, y=+row) -> (dcol, drow).
		var d_field := Vector2(b.y - a.y, b.x - a.x)
		var seg_cells := d_field.length()
		if seg_cells < 1e-6:
			continue
		var dir := d_field / seg_cells
		var seg_m := seg_cells * cell_m
		_stamp_segment(img, a, b, dir, arc_m, seg_m, half_tex, sx, sy)
		arc_m += seg_m

# Rasterize one segment a->b (cells, row,col) as a thick band into the field texture.
# Walks the texel-space bounding box of the band; for each texel inside the band it
# writes dir*0.5+0.5 into R,G and a wrapped per-texel arc-length into B.
func _stamp_segment(img: Image, a: Vector2, b: Vector2, dir: Vector2, arc0_m: float,
		seg_m: float, half_tex: float, sx: float, sy: float) -> void:
	var sz := img.get_width()
	# Endpoints in texel space (x=col*sx, y=row*sy).
	var ax := a.y * sx; var ay := a.x * sy
	var bx := b.y * sx; var by := b.x * sy
	var minx := int(floor(minf(ax, bx) - half_tex))
	var maxx := int(ceil(maxf(ax, bx) + half_tex))
	var miny := int(floor(minf(ay, by) - half_tex))
	var maxy := int(ceil(maxf(ay, by) + half_tex))
	minx = clampi(minx, 0, sz - 1); maxx = clampi(maxx, 0, sz - 1)
	miny = clampi(miny, 0, sz - 1); maxy = clampi(maxy, 0, sz - 1)
	var seg := Vector2(bx - ax, by - ay)
	var seg_len2 := maxf(seg.length_squared(), 1e-9)
	var rg := Vector2(dir.x, dir.y) * 0.5 + Vector2(0.5, 0.5)
	for ty in range(miny, maxy + 1):
		for tx in range(minx, maxx + 1):
			var pt := Vector2(float(tx), float(ty))
			# Project onto the segment, clamp to [0,1].
			var s := clampf((pt - Vector2(ax, ay)).dot(seg) / seg_len2, 0.0, 1.0)
			var proj := Vector2(ax, ay) + seg * s
			if pt.distance_to(proj) > half_tex:
				continue
			# Arc length at this texel = segment start arc + s*seg_m; wrap to [0,1]
			# over a 1 m window so frac() in the shader tiles cleats every metre.
			var arc := arc0_m + s * seg_m
			var phase: float = arc - floorf(arc)
			img.set_pixel(tx, ty, Color(rg.x, rg.y, phase))

# Stamp a small filled disc for a single-point contact (orientation = `dir`).
func _stamp_disc(img: Image, center_rc: Vector2, dir: Vector2, arc_m: float,
		half_tex: float, sx: float, sy: float) -> void:
	var sz := img.get_width()
	var cx := center_rc.y * sx; var cy := center_rc.x * sy
	var rg := Vector2(dir.x, dir.y) * 0.5 + Vector2(0.5, 0.5)
	var phase: float = arc_m - floorf(arc_m)
	var minx := clampi(int(floor(cx - half_tex)), 0, sz - 1)
	var maxx := clampi(int(ceil(cx + half_tex)), 0, sz - 1)
	var miny := clampi(int(floor(cy - half_tex)), 0, sz - 1)
	var maxy := clampi(int(ceil(cy + half_tex)), 0, sz - 1)
	for ty in range(miny, maxy + 1):
		for tx in range(minx, maxx + 1):
			if Vector2(float(tx), float(ty)).distance_to(Vector2(cx, cy)) <= half_tex:
				img.set_pixel(tx, ty, Color(rg.x, rg.y, phase))

# A decimated copy of the heightmap for the far-field LOD demo (spec §4:
# far field renders from a low-res tile). step=4 -> 64x64 from 256x256.
func tex_height_lowres(step: int = 4) -> ImageTexture:
	step = maxi(step, 1)
	var lw := int(ceil(float(width) / step))
	var lh := int(ceil(float(height) / step))
	var lo := Image.create(lw, lh, false, Image.FORMAT_RF)
	for r in range(lh):
		for c in range(lw):
			lo.set_pixel(c, r, Color(height_at(r * step, c * step), 0, 0, 1))
	return ImageTexture.create_from_image(lo)

func _image_minmax(img: Image) -> Vector2:
	var lo := INF; var hi := -INF
	for r in range(img.get_height()):
		for c in range(img.get_width()):
			var v := img.get_pixel(c, r).r
			lo = minf(lo, v); hi = maxf(hi, v)
	if lo == hi:
		hi = lo + 1.0
	return Vector2(lo, hi)
