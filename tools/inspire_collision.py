"""Provisional FTP palm collision candidates; no dynamics qualification is implied.

``replace_palm_with_components`` runs on an imported USD stage before physics
initialization. It preserves the source triangles and rigid body, replacing one
whole-palm collider with separately cooked, connected source components. All
nonadjacent collision pairs remain enabled. The source mesh remains in place;
replacement meshes inherit its exact transform and material bindings.

The topology code requires only Python's standard library. USD imports are lazy
so its geometry invariants can also be checked outside Isaac Sim. PhysX schema
names below match the installed 107.3.26 schema; raw authored attributes avoid
requiring the simulator's plugin loader during CPU-only USD validation.
"""
from dataclasses import dataclass
import hashlib
import math
import operator
import struct


CANDIDATE_ID = "ftp_palm_components_v1"
SOURCE_COMMIT = "7d6075f7f58588b189b940130e3edab3c839b2df"
SOURCE_URL = "https://github.com/unitreerobotics/unitree_ros"
SOURCE_STL_SHA256 = "77930c4a5df7536f95883e3f50b3fc21a12cb34bfc0b03b71ab859d166595b70"
# Fingerprint of the pinned import's ordered points/counts/indices, not a cooked
# mesh or an assertion of equivalence to the installed RH56E2 hardware.
PINNED_GEOMETRY_SHA256 = "7b61ca7ab2534e03f0a61202331e0733dc0673f48c55a7cc28685606ab3d691d"
DECOMPOSITION = {
    "maxConvexHulls": 32,
    "hullVertexLimit": 64,
    "voxelResolution": 1000000,
    "errorPercentage": 1.0,
    "minThickness": 0.001,
    "shrinkWrap": True,
}


def _index(value):
    if isinstance(value, bool):
        raise ValueError("Boolean mesh index/count")
    try:
        return operator.index(value)
    except TypeError as exc:
        raise ValueError("Mesh indices/counts must be integers") from exc


def _coordinate(value):
    value = tuple(float(v) for v in value)
    if len(value) != 3 or not all(math.isfinite(v) for v in value):
        raise ValueError("Mesh points must have three finite coordinates")
    return value


def geometry_sha256(points, counts, indices, orientation="rightHanded"):
    """Versioned, deterministic fingerprint without tolerance/scale rounding."""
    digest = hashlib.sha256(b"panthera-indexed-triangles-v1\0")
    digest.update(orientation.encode("ascii") + b"\0")
    for values, packing in ((points, "<3d"), (counts, "<Q"), (indices, "<Q")):
        digest.update(struct.pack("<Q", len(values)))
        for value in values:
            digest.update(struct.pack(packing, *value) if packing == "<3d"
                          else struct.pack(packing, value))
    return digest.hexdigest()


@dataclass(frozen=True)
class TrianglePartition:
    """Exact-coordinate welded topology, with every original face represented."""

    points: tuple
    faces: tuple
    components: tuple
    original_point_count: int
    source_geometry_sha256: str
    boundary_edge_count: int
    nonmanifold_edge_count: int

    def component_mesh(self, component):
        """Return compact vertices/indices; retain face winding and source order."""
        remap = {}
        points, indices = [], []
        for face_index in self.components[component]:
            for vertex in self.faces[face_index]:
                if vertex not in remap:
                    remap[vertex] = len(points)
                    points.append(self.points[vertex])
                indices.append(remap[vertex])
        return points, indices

    def component_fingerprint(self, component):
        digest = hashlib.sha256(b"panthera-ordered-triangle-coordinates-v1\0")
        for index in self.components[component]:
            for vertex in self.faces[index]:
                digest.update(struct.pack("<3d", *self.points[vertex]))
        return digest.hexdigest()


def partition_triangles(points, face_vertex_counts, face_vertex_indices,
                        orientation="rightHanded"):
    """Split by shared undirected edges after *exact* coordinate welding.

    A common STL importer duplicates the three vertices of every triangle. Point
    indices therefore cannot establish connectivity. No distance tolerance is
    used here, and point-only contact does not join otherwise separate parts.
    Components are ordered by their first source face; face order and winding
    remain unchanged. Near-coincident coordinates remain distinct.
    """
    points = tuple(_coordinate(p) for p in points)
    counts = tuple(_index(c) for c in face_vertex_counts)
    indices = tuple(_index(i) for i in face_vertex_indices)
    if not points or not counts or any(c != 3 for c in counts):
        raise ValueError("A nonempty, already triangulated source mesh is required")
    if len(indices) != 3 * len(counts):
        raise ValueError("Face counts do not match the index array")
    if min(indices) < 0 or max(indices) >= len(points):
        raise ValueError("Mesh index outside the source point array")
    if orientation not in ("rightHanded", "leftHanded"):
        raise ValueError("Unknown mesh orientation")
    lookup, welded, remap = {}, [], []
    for point in points:
        if point not in lookup:
            lookup[point] = len(welded)
            welded.append(point)
        remap.append(lookup[point])
    faces = tuple(tuple(remap[i] for i in indices[start:start + 3])
                  for start in range(0, len(indices), 3))
    if any(len(set(face)) != 3 for face in faces):
        raise ValueError("Source contains a triangle with repeated coordinates")

    parents = list(range(len(faces)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    edges = {}
    for index, face in enumerate(faces):
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = (min(a, b), max(a, b))
            if edge in edges:
                previous, count = edges[edge]
                parents[find(index)] = find(previous)
                edges[edge] = (previous, count + 1)
            else:
                edges[edge] = (index, 1)
    grouped = {}
    for index in range(len(faces)):
        grouped.setdefault(find(index), []).append(index)
    components = tuple(tuple(c) for c in sorted(grouped.values(), key=lambda c: c[0]))
    assert sorted(i for c in components for i in c) == list(range(len(faces)))
    return TrianglePartition(tuple(welded), faces, components, len(points),
                             geometry_sha256(points, counts, indices, orientation),
                             sum(n == 1 for _, n in edges.values()),
                             sum(n > 2 for _, n in edges.values()))


def _authored_snapshot(prim, prefixes):
    return {str(a.GetName()): repr(a.Get()) for a in prim.GetAttributes()
            if str(a.GetName()).startswith(prefixes) and a.HasAuthoredValueOpinion()}


def replace_palm_with_components(stage, source_mesh_path, rigid_body_path, *,
                                 contact_offset_m, rest_offset_m=0.0,
                                 expected_geometry_sha256=PINNED_GEOMETRY_SHA256):
    """Replace one pinned palm collider with dynamic, same-body child colliders.

    Call before physics initialization, after importer wrapper collision APIs
    have been relocated onto the source Mesh. The stage must use meters. The
    explicit offsets should equal the measured whole-palm control, because
    PhysX's automatic offsets otherwise change with each component's extent.

    The returned manifest is JSON serializable and describes authored intent.
    Cooked hulls, live shape counts, collision clearance, moving-palm dynamics,
    and manipulation must still be measured. ``expected_geometry_sha256`` is an
    explicit provenance override for separately reviewed sources or CPU tests.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    contact_offset_m, rest_offset_m = float(contact_offset_m), float(rest_offset_m)
    if not (math.isfinite(contact_offset_m) and math.isfinite(rest_offset_m)
            and contact_offset_m > 0 and 0 <= rest_offset_m < contact_offset_m):
        raise ValueError("Require finite 0 <= rest offset < contact offset, in meters")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        raise ValueError("Collision candidate requires a stage authored in meters")
    source = stage.GetPrimAtPath(source_mesh_path)
    body = stage.GetPrimAtPath(rigid_body_path)
    if not source or not source.IsA(UsdGeom.Mesh) or source.IsInstanceProxy():
        raise ValueError("Expected an editable source Mesh; deinstance its ancestor first")
    if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError("Expected the existing palm rigid body")
    owner = source.GetParent()
    while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
        if owner.HasAPI(UsdPhysics.CollisionAPI):
            raise ValueError("Relocate ancestor collision APIs onto the Mesh first")
        owner = owner.GetParent()
    if owner != body:
        raise ValueError("Source Mesh is not owned by the specified rigid body")
    if not source.HasAPI(UsdPhysics.CollisionAPI):
        raise ValueError("Source Mesh must own the collision API before replacement")
    if UsdPhysics.MeshCollisionAPI(source).GetApproximationAttr().Get() != "convexDecomposition":
        raise ValueError("Expected the reviewed convex-decomposed source collider")
    if not UsdPhysics.CollisionAPI(source).GetCollisionEnabledAttr().Get():
        raise ValueError("Source collision must be enabled")
    if source.HasAPI(UsdPhysics.FilteredPairsAPI):
        raise ValueError("Mesh-local filtered pairs require an explicit separate review")
    if source.GetChildren():
        raise ValueError("Expected a leaf source Mesh; refusing repeated replacement")
    geometry = UsdGeom.Mesh(source)
    attrs = (geometry.GetPointsAttr(), geometry.GetFaceVertexCountsAttr(),
             geometry.GetFaceVertexIndicesAttr())
    if any(a.GetNumTimeSamples() for a in attrs) or geometry.GetHoleIndicesAttr().Get():
        raise ValueError("Animated geometry or authored holes require separate review")
    orientation = str(geometry.GetOrientationAttr().Get())
    partition = partition_triangles(*(a.Get() for a in attrs), orientation=orientation)
    if partition.source_geometry_sha256 != expected_geometry_sha256:
        raise ValueError("Source geometry fingerprint differs from the reviewed input: "
                         + partition.source_geometry_sha256)

    # Snapshot authored physical properties and all existing pair filters. Only
    # collision representation is changed; no new rigid bodies/joints or filters.
    masses_before = _authored_snapshot(body, ("physics:mass", "physics:centerOfMass",
        "physics:diagonalInertia", "physics:principalAxes", "physics:density"))
    filters_before = {str(p.GetPath()): tuple(map(str, p.GetRelationship("physics:filteredPairs").GetTargets()))
                      for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)}
    world_matrix = UsdGeom.Xformable(source).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    collision_attrs = [(a.GetName(), a.GetTypeName(), a.Get()) for a in source.GetAttributes()
                       if str(a.GetName()).startswith("physxCollision:") and a.HasAuthoredValueOpinion()]
    simulation_owners = source.GetRelationship("physics:simulationOwner").GetTargets()
    root_path = source.GetPath().AppendChild(CANDIDATE_ID)
    candidate_root = UsdGeom.Xform.Define(stage, root_path)
    candidate_root.CreatePurposeAttr("guide")
    records = []
    for component, face_ids in enumerate(partition.components):
        points, indices = partition.component_mesh(component)
        path = root_path.AppendChild(f"component_{component:03d}")
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        mesh.CreateFaceVertexCountsAttr([3] * len(face_ids))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateOrientationAttr(orientation)
        mesh.CreateSubdivisionSchemeAttr("none")
        prim = mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexDecomposition")
        prim.AddAppliedSchema("PhysxConvexDecompositionCollisionAPI")
        for name, value in DECOMPOSITION.items():
            kind = (Sdf.ValueTypeNames.Bool if isinstance(value, bool) else
                    Sdf.ValueTypeNames.Int if isinstance(value, int) else Sdf.ValueTypeNames.Float)
            prim.CreateAttribute("physxConvexDecompositionCollision:" + name, kind, custom=False).Set(value)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        for name, kind, value in collision_attrs:
            prim.CreateAttribute(name, kind, custom=False).Set(value)
        for name, value in (("contactOffset", contact_offset_m), ("restOffset", rest_offset_m)):
            prim.CreateAttribute("physxCollision:" + name, Sdf.ValueTypeNames.Float, custom=False).Set(value)
        if simulation_owners:
            UsdPhysics.CollisionAPI(prim).CreateSimulationOwnerRel().SetTargets(simulation_owners)
        if UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) != world_matrix:
            raise RuntimeError("Component transform differs from the original source Mesh")
        records.append({"index": component, "prim": str(path), "source_face_indices": list(face_ids),
                        "triangle_count": len(face_ids), "point_count": len(points),
                        "triangle_coordinates_sha256": partition.component_fingerprint(component)})

    # collisionEnabled=False retained backend shapes in an earlier diagnostic.
    # Remove collision APIs to demand exclusive replacement, then verify live
    # backend counts independently after reset in the caller's physics probe.
    source.RemoveAPI(UsdPhysics.CollisionAPI)
    source.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    source.RemoveAppliedSchema("PhysxConvexDecompositionCollisionAPI")
    source.RemoveAppliedSchema("PhysxCollisionAPI")
    masses_after = _authored_snapshot(body, ("physics:mass", "physics:centerOfMass",
        "physics:diagonalInertia", "physics:principalAxes", "physics:density"))
    filters_after = {str(p.GetPath()): tuple(map(str, p.GetRelationship("physics:filteredPairs").GetTargets()))
                     for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)}
    if masses_before != masses_after or filters_before != filters_after:
        raise RuntimeError("Unexpected physical property or collision-filter mutation")
    if geometry_sha256(*(a.Get() for a in attrs), orientation) != partition.source_geometry_sha256:
        raise RuntimeError("Original source geometry changed during collision replacement")
    return {
        "schema_version": 1, "candidate_id": CANDIDATE_ID,
        "status": "provisional_authored_candidate_requires_physics_validation",
        "exact_RH56E2_equivalence": False, "simulation_qualified": False,
        "moving_palm_qualified": False, "grasp_qualified": False, "hardware_authorized": False,
        "source_url": SOURCE_URL, "source_commit": SOURCE_COMMIT, "source_license": "BSD-3-Clause",
        "pinned_source_stl_sha256": SOURCE_STL_SHA256,
        "source_geometry_matches_pinned_donor": partition.source_geometry_sha256 == PINNED_GEOMETRY_SHA256,
        "source_geometry_sha256": partition.source_geometry_sha256,
        "source_prim": str(source.GetPath()), "rigid_body_prim": str(body.GetPath()),
        "candidate_root_prim": str(root_path), "source_orientation": orientation,
        "source_local_to_world_row_matrix": [list(row) for row in world_matrix],
        "source_point_count": partition.original_point_count,
        "exact_coordinate_unique_point_count": len(partition.points),
        "source_triangle_count": len(partition.faces), "component_count": len(records),
        "component_triangle_counts": [r["triangle_count"] for r in records],
        "source_boundary_edge_count_after_exact_weld": partition.boundary_edge_count,
        "source_nonmanifold_edge_count_after_exact_weld": partition.nonmanifold_edge_count,
        "partition_method": "exact_coordinate_weld_then_shared_undirected_edge_connected_components",
        "decomposition": dict(DECOMPOSITION),
        "preset_rationale": "Separate disconnected source parts before voxel decomposition to avoid "
            "hulls bridging between parts; retain 32-hull/64-vertex limits and 1 mm minimum thickness; "
            "1% error, 1M voxels and shrink-wrap refine each component. These are collision "
            "approximation settings, not measured hardware geometry or proof of clearance.",
        "contact_offset_m": contact_offset_m, "rest_offset_m": rest_offset_m,
        "contact_offset_policy": "explicit measured whole-palm control; no component-extent auto margin",
        "source_visual_geometry_changed": False, "source_triangles_changed": False,
        "source_face_coverage_exactly_once": True, "source_transform_changed": False,
        "articulation_mass_or_inertia_changed": False, "rigid_body_or_joint_count_changed": False,
        "collision_pairs_filtered": False, "existing_filter_relationships_changed": False,
        "authored_palm_mass_properties": masses_before,
        "source_collision_apis_removed": not source.HasAPI(UsdPhysics.CollisionAPI),
        "expected_authored_palm_collider_count": len(records),
        "cooked_hull_count": None, "live_backend_palm_shape_count": None,
        "expected_live_count_rule": "live palm shapes must equal the sum of independently returned "
            "component cooking hulls; no residual whole-palm shapes; same rigid body count",
        "maximum_requested_palm_hulls": len(records) * DECOMPOSITION["maxConvexHulls"],
        "components": records,
    }
