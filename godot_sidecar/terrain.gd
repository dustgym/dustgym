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

var sf                       # StateFields instance (preloaded by caller)
var _active_mi: MeshInstance3D
var _far_mi: MeshInstance3D

func build(state, mode: int = Mode.LIT_PBR) -> void:
	sf = state
	_build_far_field(mode)
	_build_active_zone(mode)

# ---------------------------------------------------------------------------
# FAR FIELD: one low-poly plane displaced in the vertex shader (cheap LOD).
# ---------------------------------------------------------------------------
func _build_far_field(mode: int) -> void:
	var ext: Vector2 = sf.extent_m()
	var pm := PlaneMesh.new()
	pm.size = ext
	# Low subdivision on purpose — this is the cheap inactive-region mesh.
	pm.subdivide_width = 32
	pm.subdivide_depth = 32
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
		_far_mi.material_override = sm
	else:
		# In false-color modes the far plane just uses the active material look;
		# keep it flat-displaced for context. Reuse height tex via a basic mat.
		_far_mi.material_override = _make_falsecolor_mat(mode)
		# Far plane carries no per-vertex displacement in fc mode (flat context).
	add_child(_far_mi)

# ---------------------------------------------------------------------------
# ACTIVE ZONE: fine ArrayMesh, vertices sample the authoritative heightmap.
# ---------------------------------------------------------------------------
func _build_active_zone(mode: int) -> void:
	var az: Dictionary = sf.active_zone
	var min_rc = az.get("min_rc", [0, 0])
	var max_rc = az.get("max_rc", [sf.height - 1, sf.width - 1])
	var r0 := int(min_rc[0]); var c0 := int(min_rc[1])
	var r1 := int(max_rc[0]); var c1 := int(max_rc[1])
	r0 = clampi(r0, 0, sf.height - 1); r1 = clampi(r1, 1, sf.height - 1)
	c0 = clampi(c0, 0, sf.width - 1);  c1 = clampi(c1, 1, sf.width - 1)

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
