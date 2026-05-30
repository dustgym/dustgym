extends Node3D
class_name TerrainNode
# Terrain builder: ACTIVE-zone fine mesh + FAR-FIELD LOD plane (spec §4).
#
# ACTIVE zone (spec §4 "Under wheels/drums", highest fidelity): an ArrayMesh
# whose vertices sample the authoritative heightmap per-vertex at fine
# resolution. INTERFACE.md §4: heightmap is authoritative for geometry.
#
# FAR FIELD (the D2 LOD demo, spec §4 efficiency note): a SINGLE low-subdivision
# PlaneMesh with a VERTEX shader (terrain_farfield.gdshader) that displaces from
# a decimated heightmap tile. Big inactive quadtree regions cost almost nothing.
#
# Field->Godot mapping is direct (INTERFACE.md §3): gx=x=col*cell, gy=height,
# gz=z=row*cell, origin at world_bounds min corner.

enum Mode { LIT_PBR, FALSECOLOR_HEIGHT, FALSECOLOR_STATE }

# Active-zone mesh subdivision: how many quads per cell-span across the active
# window. We sample the heightmap per vertex, so this is render resolution
# (spec §4: "render resolution may be 5-10x physics resolution").
const ACTIVE_VERTS_PER_SIDE := 192   # 192x192 verts over the active window

# When the rover-keyed window is used (v1.0.1 rover_rc/active_leaves present), the
# fine mesh is a square window CENTERED on the rover, sized to cover the active
# leaves plus a margin so it reads as "fine mesh follows the rover" (vs the small
# bare active_leaves bbox). Half-width in CELLS, clamped to the field.
const ROVER_WINDOW_HALF_CELLS := 40   # -> ~80x80 cell fine window (1.6 m at 0.02 m/cell)

var sf                       # StateFields instance (preloaded by caller)
var _active_mi: MeshInstance3D
var _far_mi: MeshInstance3D
var _quadtree_node: Node3D   # optional wireframe LOD overlay

func build(state, mode: int = Mode.LIT_PBR, show_quadtree: bool = false) -> void:
	sf = state
	_build_far_field(mode)
	_build_active_zone(mode)
	if show_quadtree:
		_build_quadtree_overlay()

# ---------------------------------------------------------------------------
# FAR FIELD: one low-poly plane displaced in the vertex shader (cheap LOD).
# ---------------------------------------------------------------------------
func _build_far_field(mode: int) -> void:
	var ext: Vector2 = sf.extent_m()
	var pm := PlaneMesh.new()
	pm.size = ext
	# Subdivision matched to the decimated height texel grid (width/4 = 128 for the 512 field)
	# so the vertex displace can actually resolve a ~18 cm rut: at 32 (16 cm quads) the thin
	# track was undersampled into a row of dimples (beads), which the #1 crisp grazing shadows
	# then lit as "circular indents". 128 quads (~4 cm) resolves the groove as a continuous
	# trough. Still one cheap plane + a single vertex displace (the LOD-cheap story holds).
	pm.subdivide_width = 128
	pm.subdivide_depth = 128
	# Center plane so it spans the whole field; PlaneMesh is centered at origin.
	_far_mi = MeshInstance3D.new()
	_far_mi.mesh = pm
	_far_mi.position = Vector3(sf.world_min.x + ext.x * 0.5, 0.0,
							   sf.world_min.y + ext.y * 0.5)

	if mode == Mode.LIT_PBR:
		var sm := ShaderMaterial.new()
		sm.shader = load("res://terrain_farfield.gdshader")
		sm.set_shader_parameter("height_lowres", sf.tex_height_lowres(4))
		# World meters per low-res texel, so the vertex shader can scale its
		# gradient-normal correctly (tex is decimated 4x from the full grid).
		var lw := int(ceil(float(sf.width) / 4.0))
		sm.set_shader_parameter("lod_step_m", ext.x / float(maxi(lw, 1)))
		# State + disturbance at FULL field res so the rover track reads as a continuous
		# TREAD albedo band on the far field regardless of this mesh's coarseness -- the track
		# is an APPEARANCE feature, not reliant on under-resolved geometry. Sampled in the same
		# 0..1 field UV the height displace already uses, so the dark band lands exactly on the
		# geometric groove. render_fidelity track fix.
		sm.set_shader_parameter("state_tex", sf.tex_state())
		sm.set_shader_parameter("disturbance_tex", sf.tex_disturbance())
		# Cut-depth albedo on the far field too (so the excavated swath reads fresh across the
		# whole patch, not just under the active window). Same gate/uniforms as the active mat.
		sm.set_shader_parameter("mass_areal_tex", sf.tex_mass_areal())
		sm.set_shader_parameter("cut_depth_enabled", sf.has_uniform_mantle)
		sm.set_shader_parameter("mantle_areal_m0", sf.mantle_areal_m0)
		sm.set_shader_parameter("surface_density_cd", sf.mantle_surface_density)
		sm.set_shader_parameter("cut_depth_full_m", sf.cut_depth_full_m)
		sm.set_shader_parameter("fresh_albedo_gain", sf.maturity_albedo_ratio)
		# Photometry: Hapke BRDF (render_fidelity). The far plane fills the frame, so it
		# carries the same airless-regolith photometry as the active window (no BRDF seam).
		sm.set_shader_parameter("hapke_enabled", sf.hapke_enabled)
		sm.set_shader_parameter("hapke_b", sf.hapke_b)
		sm.set_shader_parameter("hapke_c", sf.hapke_c)
		sm.set_shader_parameter("hapke_B0", sf.hapke_B0)
		sm.set_shader_parameter("hapke_h", sf.hapke_h)
		sm.set_shader_parameter("hapke_gain", sf.hapke_gain)
		_far_mi.material_override = sm
	else:
		# In false-color modes the far plane just uses the active material look;
		# keep it flat-displaced for context. Reuse height tex via a basic mat.
		_far_mi.material_override = _make_falsecolor_mat(mode)
		# Far plane carries no per-vertex displacement in fc mode (flat context).
	add_child(_far_mi)

# Choose the fine ACTIVE-zone window [r0,c0,r1,c1] (inclusive corners in cells).
# v1.0.1 (INTERFACE.md §5.1): when the per-frame rover keys are present, the fine
# mesh FOLLOWS the rover -- a square window centered on rover_rc, grown to cover
# the active_leaves bbox plus a margin (ROVER_WINDOW_HALF_CELLS). When absent,
# fall back to the static metadata active_zone (the v1.0 behavior). This is the
# render-side realization of spec §4: the tree manages SPACE, keyed to interaction.
func _active_window_rc() -> Array:
	if sf.has_rover_rc:
		var cr: int = sf.rover_row()
		var cc: int = sf.rover_col()
		var half := ROVER_WINDOW_HALF_CELLS
		# Grow the half-window so the active_leaves bbox is fully inside it (so the
		# fine mesh always covers the live LOD hot-set, not just a fixed box).
		var bb: Array = sf.active_leaves_bbox()
		if bb[2] > bb[0]:   # non-empty bbox
			half = maxi(half, int(ceil(maxf(
				maxf(abs(cr - bb[0]), abs(bb[2] - cr)),
				maxf(abs(cc - bb[1]), abs(bb[3] - cc))))) + 2)
		var r0 := clampi(cr - half, 0, sf.height - 2)
		var c0 := clampi(cc - half, 0, sf.width - 2)
		var r1 := clampi(cr + half, r0 + 1, sf.height - 1)
		var c1 := clampi(cc + half, c0 + 1, sf.width - 1)
		return [r0, c0, r1, c1]
	# --- static fallback (v1.0 active_zone) ---
	var az: Dictionary = sf.active_zone
	var min_rc = az.get("min_rc", [0, 0])
	var max_rc = az.get("max_rc", [sf.height - 1, sf.width - 1])
	var sr0 := clampi(int(min_rc[0]), 0, sf.height - 1)
	var sr1 := clampi(int(max_rc[0]), 1, sf.height - 1)
	var sc0 := clampi(int(min_rc[1]), 0, sf.width - 1)
	var sc1 := clampi(int(max_rc[1]), 1, sf.width - 1)
	return [sr0, sc0, sr1, sc1]

# ---------------------------------------------------------------------------
# ACTIVE ZONE: fine ArrayMesh, vertices sample the authoritative heightmap.
# ---------------------------------------------------------------------------
func _build_active_zone(mode: int) -> void:
	var win := _active_window_rc()
	var r0: int = win[0]; var c0: int = win[1]
	var r1: int = win[2]; var c1: int = win[3]

	var n := ACTIVE_VERTS_PER_SIDE
	var verts := PackedVector3Array()
	var uvs := PackedVector2Array()
	var normals := PackedVector3Array()
	verts.resize(n * n)
	uvs.resize(n * n)
	normals.resize(n * n)

	# field-space fractional row/col covered by this active window
	for iy in range(n):
		var fv := float(iy) / float(n - 1)               # 0..1 down the window
		var row := lerpf(float(r0), float(r1), fv)
		for ix in range(n):
			var fu := float(ix) / float(n - 1)           # 0..1 across the window
			var col := lerpf(float(c0), float(c1), fu)
			# BILINEAR height sample (not nearest): int(round()) snapping terraces the
			# surface where render verts are finer than physics cells, and terraces on the
			# steep crater wall band the shading under the 5deg grazing sun. height_uv() is
			# the contract's existing bilinear sampler.
			var h: float = sf.height_uv(col / float(sf.width - 1), row / float(sf.height - 1))
			var wx: float = sf.world_min.x + col * sf.cell_m
			var wz: float = sf.world_min.y + row * sf.cell_m
			var idx := iy * n + ix
			verts[idx] = Vector3(wx, h, wz)
			# UV0 = FIELD uv (full-field normalized), so shaders sample the
			# full-resolution disturbance/state/density textures correctly.
			uvs[idx] = Vector2(col / float(sf.width - 1), row / float(sf.height - 1))
			normals[idx] = Vector3.UP

	var indices := PackedInt32Array()
	for iy in range(n - 1):
		for ix in range(n - 1):
			var a := iy * n + ix
			var b := iy * n + ix + 1
			var c := (iy + 1) * n + ix
			var d := (iy + 1) * n + ix + 1
			indices.append_array([a, c, b, b, c, d])

	# Compute smooth normals from the displaced surface (finite differences).
	_compute_normals(verts, indices, normals, n)

	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_TEX_UV] = uvs
	arrays[Mesh.ARRAY_NORMAL] = normals
	arrays[Mesh.ARRAY_INDEX] = indices

	var am := ArrayMesh.new()
	am.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	_active_mi = MeshInstance3D.new()
	_active_mi.mesh = am

	if mode == Mode.LIT_PBR:
		var sm := ShaderMaterial.new()
		sm.shader = load("res://terrain.gdshader")
		sm.set_shader_parameter("disturbance_tex", sf.tex_disturbance())
		sm.set_shader_parameter("state_tex", sf.tex_state())
		sm.set_shader_parameter("density_tex", sf.tex_density())
		sm.set_shader_parameter("density_lo", sf.density_range.x)
		sm.set_shader_parameter("density_hi", sf.density_range.y)
		# Detail pass (render_fidelity_spec.md §4.2): the baked track-direction + phase
		# field (§4.3) orients cleat/teeth marks; it is NEUTRAL (no marks) for scenes
		# with no §5.2 wheel_tracks/drum_marks, so this is additive — crater etc. render
		# as before save for the AA + subtle base granularity. MUST set every uniform the
		# shader samples (track_dir_tex, field_span_m) so there are no missing-uniform
		# warnings; the §4.2 tunables keep their shader defaults.
		sm.set_shader_parameter("track_dir_tex", sf.tex_track_dir())
		# World metres spanned by the field UV (0..1). The field is square at cell_m
		# (INTERFACE.md §1/§3): span = width * cell_m. Lets the in-shader noise/cleat/
		# teeth periods be real metres regardless of field size.
		sm.set_shader_parameter("field_span_m", float(sf.width) * sf.cell_m)
		# Cut-depth / exposed-sublayer albedo (render_fidelity). Fresh-cut regolith reads
		# brighter than the space-weathered surface, graded by EXCAVATED areal-mass deficit
		# (M0 - mass_areal). Enabled only when the scene declares the uniform-mantle model, so
		# this is additive: scenes without it render exactly as before.
		sm.set_shader_parameter("mass_areal_tex", sf.tex_mass_areal())
		sm.set_shader_parameter("cut_depth_enabled", sf.has_uniform_mantle)
		sm.set_shader_parameter("mantle_areal_m0", sf.mantle_areal_m0)
		sm.set_shader_parameter("surface_density_cd", sf.mantle_surface_density)
		sm.set_shader_parameter("cut_depth_full_m", sf.cut_depth_full_m)
		sm.set_shader_parameter("fresh_albedo_gain", sf.maturity_albedo_ratio)
		# Photometry: Hapke BRDF (render_fidelity; replaces Lambert). Physical lunar-regolith
		# constants from state_fields (literature defaults, optional per-scene override). NORMAL
		# here is the detail-perturbed normal, so micro-relief feeds the BRDF. --brdf lambert
		# flips hapke_enabled for the A/B comparison render.
		sm.set_shader_parameter("hapke_enabled", sf.hapke_enabled)
		sm.set_shader_parameter("hapke_b", sf.hapke_b)
		sm.set_shader_parameter("hapke_c", sf.hapke_c)
		sm.set_shader_parameter("hapke_B0", sf.hapke_B0)
		sm.set_shader_parameter("hapke_h", sf.hapke_h)
		sm.set_shader_parameter("hapke_gain", sf.hapke_gain)
		_active_mi.material_override = sm
	else:
		_active_mi.material_override = _make_falsecolor_mat(mode)
	add_child(_active_mi)

func _make_falsecolor_mat(mode: int) -> ShaderMaterial:
	var sm := ShaderMaterial.new()
	if mode == Mode.FALSECOLOR_HEIGHT:
		sm.shader = load("res://falsecolor_height.gdshader")
		sm.set_shader_parameter("height_tex", sf.tex_height())
		sm.set_shader_parameter("h_lo", sf.height_range.x)
		sm.set_shader_parameter("h_hi", sf.height_range.y)
	else:
		sm.shader = load("res://falsecolor_state.gdshader")
		sm.set_shader_parameter("state_tex", sf.tex_state())
	return sm

# ---------------------------------------------------------------------------
# QUADTREE LOD OVERLAY (toggleable "quadtree" layer). Draws the per-frame
# quadtree_nodes leaf boxes as wireframe rectangles lifted to the heightmap,
# mirroring the 4a matplotlib colors (viz/out/quadtree_demo_filmstrip.png):
#   - FINE / active leaves (size == min_leaf)  -> WARM red, lifted highest
#   - mid leaves                                -> AMBER
#   - COARSE far-field leaves (big nodes)       -> COOL teal/blue, near-surface
# Plus the active_leaves hot-set drawn brightest, so the promotion-follows-rover
# is legible in 3D (spec §4: the tree manages SPACE keyed to interaction).
# Uses ImmediateMesh line lists (cheap, headless-safe) on an unshaded material.
# ---------------------------------------------------------------------------
func _build_quadtree_overlay() -> void:
	_quadtree_node = Node3D.new()
	_quadtree_node.name = "QuadtreeOverlay"

	var nodes: Array = sf.quadtree_nodes
	var min_leaf: int = int(sf.quadtree_lod.get("min_leaf", 8))

	# All quadtree leaves: color-graded by size (fine warm -> coarse cool).
	if not nodes.is_empty():
		var im := ImmediateMesh.new()
		im.surface_begin(Mesh.PRIMITIVE_LINES)
		for nd in nodes:
			if typeof(nd) != TYPE_DICTIONARY:
				continue
			if not bool(nd.get("leaf", false)):
				continue
			var r0 := int(nd.get("row0", 0))
			var c0 := int(nd.get("col0", 0))
			var sz := int(nd.get("size", min_leaf))
			var col := _lod_color(sz, min_leaf)
			# Lift fine boxes a touch higher so they pop above the surface relief.
			var lift := lerpf(0.045, 0.012, clampf(float(sz - min_leaf) / 64.0, 0.0, 1.0))
			_emit_box_outline(im, r0, c0, r0 + sz, c0 + sz, lift, col)
		im.surface_end()
		var mi := MeshInstance3D.new()
		mi.mesh = im
		mi.material_override = _overlay_line_mat()
		# Debug annotation, not geometry: must NOT cast shadows. Default is ON even for an
		# unshaded material, so the lifted (1-6 cm) wireframe was casting thin shadow stripes
		# onto the terrain -- invisible under the old coarse shadows, but the #1 dense/crisp
		# atlas resolves them as grid-correlated artifacts. (render_fidelity shadow fix)
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_quadtree_node.add_child(mi)

	# The live active_leaves hot-set: brightest warm, lifted highest, drawn on top.
	if not sf.active_leaves.is_empty():
		var ima := ImmediateMesh.new()
		ima.surface_begin(Mesh.PRIMITIVE_LINES)
		var hot := Color(1.0, 0.35, 0.15)
		for b in sf.active_leaves:
			_emit_box_outline(ima, int(b[0]), int(b[1]), int(b[2]), int(b[3]), 0.06, hot)
		ima.surface_end()
		var mia := MeshInstance3D.new()
		mia.mesh = ima
		mia.material_override = _overlay_line_mat()
		mia.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF   # debug overlay: no shadow (see above)
		_quadtree_node.add_child(mia)

	add_child(_quadtree_node)
	print("sidecar: quadtree overlay = %d nodes, %d active leaves" % [
		nodes.size(), sf.active_leaves.size()])

# LOD color ramp mirroring the 4a filmstrip: fine (min_leaf) reads WARM red, the
# largest coarse far-field leaves read COOL teal; mid sizes interpolate amber.
func _lod_color(size: int, min_leaf: int) -> Color:
	# t: 0 at min_leaf (fine) -> 1 at a big coarse node (>= 8*min_leaf).
	var t := clampf(log(float(maxi(size, 1)) / float(maxi(min_leaf, 1))) / log(8.0), 0.0, 1.0)
	var warm := Color(0.95, 0.45, 0.20)   # fine = warm orange-red
	var cool := Color(0.25, 0.55, 0.85)   # coarse = cool blue
	return warm.lerp(cool, t)

# Emit the 4 top edges (a lifted rectangle outline) of a half-open cell box
# [r0,c0,r1,c1] into an ImmediateMesh, each corner snapped to the heightmap + lift.
func _emit_box_outline(im: ImmediateMesh, r0: int, c0: int, r1: int, c1: int,
		lift: float, col: Color) -> void:
	# Inclusive cell corners in world meters; r1/c1 are half-open so the far edge
	# is the (r1,c1) grid line.
	var p00 := _surf_point(r0, c0, lift)
	var p01 := _surf_point(r0, c1, lift)
	var p11 := _surf_point(r1, c1, lift)
	var p10 := _surf_point(r1, c0, lift)
	im.surface_set_color(col); im.surface_add_vertex(p00)
	im.surface_set_color(col); im.surface_add_vertex(p01)
	im.surface_set_color(col); im.surface_add_vertex(p01)
	im.surface_set_color(col); im.surface_add_vertex(p11)
	im.surface_set_color(col); im.surface_add_vertex(p11)
	im.surface_set_color(col); im.surface_add_vertex(p10)
	im.surface_set_color(col); im.surface_add_vertex(p10)
	im.surface_set_color(col); im.surface_add_vertex(p00)

# Field grid corner (row,col may be == size for the far edge) -> world point on the
# heightmap surface, plus a small vertical lift so the wire floats above the mesh.
func _surf_point(row: int, col: int, lift: float) -> Vector3:
	var rr := clampi(row, 0, sf.height - 1)
	var cc := clampi(col, 0, sf.width - 1)
	var u := float(cc) / float(sf.width - 1)
	var v := float(rr) / float(sf.height - 1)
	var h: float = sf.height_uv(u, v) + lift
	return Vector3(sf.world_min.x + col * sf.cell_m, h, sf.world_min.y + row * sf.cell_m)

# Unshaded, vertex-colored line material for the quadtree wireframe. Depth-TESTED so the
# boxes are drawn ON the terrain (lifted a hair above the surface, see _emit_box_outline)
# and are correctly OCCLUDED by whatever is in front of them -- crucially the rover, which
# now draws ON TOP of the lines instead of having the wireframe painted across it.
# (Previously no_depth_test made it a HUD overlay that drew over the robot.)
func _overlay_line_mat() -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	m.vertex_color_use_as_albedo = true
	m.albedo_color = Color(1, 1, 1, 1)
	m.no_depth_test = false       # depth-tested: lines sit on the terrain; the rover occludes them
	return m

# Per-vertex normals via cross products of triangle edges, averaged.
func _compute_normals(verts: PackedVector3Array, indices: PackedInt32Array,
					  normals: PackedVector3Array, n: int) -> void:
	var accum := PackedVector3Array()
	accum.resize(verts.size())
	for i in range(accum.size()):
		accum[i] = Vector3.ZERO
	var t := 0
	while t < indices.size():
		var ia := indices[t]; var ib := indices[t + 1]; var ic := indices[t + 2]
		var nrm := (verts[ib] - verts[ia]).cross(verts[ic] - verts[ia])
		accum[ia] += nrm; accum[ib] += nrm; accum[ic] += nrm
		t += 3
	for i in range(accum.size()):
		var v := accum[i]
		normals[i] = v.normalized() if v.length() > 1e-9 else Vector3.UP
