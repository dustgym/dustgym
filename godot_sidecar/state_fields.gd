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

# --- per-field Images (kept so we can both sample CPU-side and build textures) ---
var img_height: Image
var img_density: Image
var img_disturbance: Image
var img_state: Image

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
func tex_density() -> ImageTexture: return ImageTexture.create_from_image(img_density)
func tex_disturbance() -> ImageTexture: return ImageTexture.create_from_image(img_disturbance)
func tex_state() -> ImageTexture: return ImageTexture.create_from_image(img_state)

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
