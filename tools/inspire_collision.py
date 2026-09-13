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
import itertools


CANDIDATE_ID = "ftp_palm_components_v1"
SLAB_CANDIDATE_ID = "ftp_palm_yz_slabs_v2"
LEFT_SLAB_CANDIDATE_ID = "ftp_left_palm_yz_slabs_v1"
LEFT_THUMB_CANDIDATE_ID = "ftp_left_thumb2_yz_slabs_v1"
LEFT_THUMB_GEOMETRY_SHA256 = "e9993d36cb9c3bdd0eef3a930f2aba50325cd18f0842255f670e1a80f7e22673"
LEFT_THUMB_STL_SHA256 = "9b92a6db621b47b906ae876e8d76466e9e2d401e59b2ceeb4e8cf9133d943ce1"
SOURCE_COMMIT = "7d6075f7f58588b189b940130e3edab3c839b2df"
SOURCE_URL = "https://github.com/unitreerobotics/unitree_ros"
SOURCE_STL_SHA256 = "77930c4a5df7536f95883e3f50b3fc21a12cb34bfc0b03b71ab859d166595b70"
# Fingerprint of the pinned import's ordered points/counts/indices, not a cooked
# mesh or an assertion of equivalence to the installed RH56E2 hardware.
PINNED_GEOMETRY_SHA256 = "7b61ca7ab2534e03f0a61202331e0733dc0673f48c55a7cc28685606ab3d691d"
PINNED_LEFT_GEOMETRY_SHA256 = "5a81cc75b60c00341c9a4ae1cc38880a6815bae89aba6e3c02e98f7cde9639a3"
LEFT_SOURCE_STL_SHA256 = "20eb4092a92fc26a14863f5dda257a1db618e6f6d617ea6736a95a819ae10f3f"
# Left is independently fingerprinted: it has 20 source components and different
# internal geometry. Its dimensions reflect across X, but it is not a byte- or
# surface-equivalent mirrored right mesh. Never substitute the right asset.
RIGHT_PROFILE = {"side": "right", "component_count": 43, "shell_component": 23,
                 "shell_faces": 43156, "retained_component": 40,
                 "geometry_sha256": PINNED_GEOMETRY_SHA256, "stl_sha256": SOURCE_STL_SHA256}
LEFT_PROFILE = {"side": "left", "component_count": 20, "shell_component": 1,
                "shell_faces": 43198, "retained_component": 7,
                "geometry_sha256": PINNED_LEFT_GEOMETRY_SHA256, "stl_sha256": LEFT_SOURCE_STL_SHA256}
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
                                 expected_geometry_sha256=None,
                                 candidate_id=CANDIDATE_ID):
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

    if candidate_id not in (CANDIDATE_ID, SLAB_CANDIDATE_ID, LEFT_SLAB_CANDIDATE_ID):
        raise ValueError("Unknown collision candidate version")
    profile = LEFT_PROFILE if candidate_id == LEFT_SLAB_CANDIDATE_ID else RIGHT_PROFILE
    if expected_geometry_sha256 is None:
        expected_geometry_sha256 = profile["geometry_sha256"]
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
    # Prepare geometry before changing the stage. v1 stays the default; v2 uses
    # the measured failing source component and retains its provenance index.
    slab_plan = build_slab_candidate(partition, profile) if candidate_id != CANDIDATE_ID else None
    root_path = source.GetPath().AppendChild(candidate_id)
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
    manifest = {
        "schema_version": 1, "candidate_id": candidate_id,
        "status": "provisional_authored_candidate_requires_physics_validation",
        "exact_RH56E2_equivalence": False, "simulation_qualified": False,
        "moving_palm_qualified": False, "grasp_qualified": False, "hardware_authorized": False,
        "source_url": SOURCE_URL, "source_commit": SOURCE_COMMIT, "source_license": "BSD-3-Clause",
        "pinned_source_stl_sha256": profile["stl_sha256"], "source_side": profile["side"],
        "source_geometry_matches_pinned_donor": partition.source_geometry_sha256 == profile["geometry_sha256"],
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
    if slab_plan is not None:
        apply_slab_candidate(stage, records, slab_plan, manifest, contact_offset_m, rest_offset_m, profile)
    return manifest


def clip_polygon(polygon, axis, plane, keep_greater):
    """Clip a polygon without moving surviving source points; cuts lie on plane."""
    result = []
    if not polygon:
        return result
    previous = polygon[-1]
    sign = 1 if keep_greater else -1
    previous_distance = (previous[axis] - plane) * sign
    for current in polygon:
        distance = (current[axis] - plane) * sign
        if (distance >= 0) != (previous_distance >= 0):
            fraction = previous_distance / (previous_distance - distance)
            point = tuple(previous[i] + fraction * (current[i] - previous[i]) for i in range(3))
            # Avoid a roundoff gap between neighboring cut planes.
            point = point[:axis] + (plane,) + point[axis + 1:]
            result.append(point)
        if distance >= 0:
            result.append(current)
        previous, previous_distance = current, distance
    return result


def closed_convex_hull(points):
    """A closed, outward-wound hull, including planar cut caps.

    The hull contains all supplied source/cut points. It is a conservative
    collision approximation, not a reconstructed CAD solid. None means a
    zero-volume boundary fragment; callers retain adjacent volumetric pieces.
    """
    import numpy as np
    from scipy.spatial import ConvexHull, QhullError

    points = np.asarray(sorted(set(tuple(map(float, p)) for p in points)))
    if len(points) < 4:
        return None
    try:
        hull = ConvexHull(points)
    except QhullError:
        if np.linalg.matrix_rank(points - points[0], tol=1e-12) < 3:
            return None
        raise
    remap = {int(old): new for new, old in enumerate(hull.vertices)}
    faces = []
    for face, equation in zip(hull.simplices, hull.equations):
        a, b, c = map(int, face)
        if np.dot(np.cross(points[b] - points[a], points[c] - points[a]), equation[:3]) < 0:
            b, c = c, b
        faces.append((remap[a], remap[b], remap[c]))
    return {"points": [tuple(p) for p in points[hull.vertices]], "faces": faces,
            "volume_m3": float(hull.volume)}


def split_hull_vertex_budget(hull, max_vertices=120, cuts=()):
    """Bisect a convex solid exactly until every piece fits the cooking budget.

    Closed caps are supplied by each child's convex hull. The sum of child
    volumes is checked against the parent before recursion. 120 input vertices
    also bounds triangular faces below 255, avoiding opaque PhysX simplification.
    """
    if len(hull["points"]) <= max_vertices:
        return [dict(hull, cuts=list(cuts))]
    if len(cuts) >= 24:
        raise ValueError("Convex partition did not converge within 24 cuts")
    points = hull["points"]
    bounds = [(min(p[i] for p in points), max(p[i] for p in points)) for i in range(3)]
    axis = max(range(3), key=lambda i: bounds[i][1] - bounds[i][0])
    plane = sum(bounds[axis]) / 2
    polygons = [[points[i] for i in face] for face in hull["faces"]]
    children = []
    for side in (False, True):
        clipped = [p for polygon in polygons for p in clip_polygon(polygon, axis, plane, side)]
        child = closed_convex_hull(clipped)
        if child is None:
            raise ValueError("Unexpected empty half of a nondegenerate convex solid")
        children.append(child)
    volume = sum(c["volume_m3"] for c in children)
    if abs(volume - hull["volume_m3"]) > max(1e-15, hull["volume_m3"] * 1e-8):
        raise ValueError("Cut caps failed convex-volume conservation")
    result = []
    for side, child in zip((False, True), children):
        result.extend(split_hull_vertex_budget(child, max_vertices,
            cuts + ({"axis": "XYZ"[axis], "plane_m": plane, "keep_greater": side},)))
    return result


def source_slab_hulls(triangles, axes=(1, 2), width_m=.004):
    """Conservative closed pieces of source material in two-axis spatial cells.

    The remaining axis is unbounded. Each cell's source-boundary fragments
    therefore contain the extremal points of the bounded material in that cell;
    their closed convex hull contains that material, including properly capped
    cuts. This would need an interior-cell rule for a three-axis voxel grid, so
    three-axis input is refused. Source visual triangles are never edited.
    """
    if len(axes) not in (1, 2) or len(set(axes)) != len(axes) or any(a not in (0, 1, 2) for a in axes):
        raise ValueError("Use one or two distinct spatial slab axes")
    if not math.isfinite(width_m) or width_m <= 0:
        raise ValueError("Slab width must be finite and positive")
    groups = {}
    for triangle in triangles:
        bins = [range(math.floor(min(p[a] for p in triangle) / width_m),
                      math.floor(max(p[a] for p in triangle) / width_m) + 1) for a in axes]
        for cell in itertools.product(*bins):
            polygon = list(triangle)
            for axis, index in zip(axes, cell):
                polygon = clip_polygon(polygon, axis, index * width_m, True)
                polygon = clip_polygon(polygon, axis, (index + 1) * width_m, False)
                if len(polygon) < 3:
                    break
            if len(polygon) >= 3:
                groups.setdefault(cell, set()).update(polygon)
    result, zero_volume_cells = [], []
    for cell, points in sorted(groups.items()):
        hull = closed_convex_hull(points)
        if hull is None:
            zero_volume_cells.append(list(cell))
            continue
        for piece in split_hull_vertex_budget(hull):
            result.append(dict(piece, slab_axes=["XYZ"[a] for a in axes], slab_cell=list(cell),
                slab_bounds_m=[[index * width_m, (index + 1) * width_m] for index in cell]))
    return result, zero_volume_cells


def build_slab_candidate(partition, profile=None):
    """Prepare slab geometry from an independently pinned palm source.

    The large shell (23) gets Y/Z cells selected by CPU cavity tests. Component
    40 retains v1 decomposition because its single hull creates a new cavity
    overlap. Other components use conservative closed hulls, split as needed to
    avoid hidden cooking vertex reduction. Added concavity volume is reported.
    """
    import numpy as np

    profile = RIGHT_PROFILE if profile is None else profile
    if (len(partition.components) != profile["component_count"] or
            len(partition.components[profile["shell_component"]]) != profile["shell_faces"]):
        raise ValueError("Slab candidate requires the reviewed " + profile["side"] + "-palm topology")
    plan = []
    for component, face_ids in enumerate(partition.components):
        points, indices = partition.component_mesh(component)
        triangles = np.asarray(points)[np.asarray(indices).reshape(-1, 3)]
        source_volume = abs(float(np.einsum("ij,ij->", triangles[:, 0],
            np.cross(triangles[:, 1], triangles[:, 2])) / 6))
        if component == profile["retained_component"]:
            plan.append({"source_component_index": component, "kind": "retain_v1_decomposition",
                         "source_signed_surface_volume_m3": source_volume})
            continue
        if component == profile["shell_component"]:
            pieces, zero_volume = source_slab_hulls([[tuple(p) for p in t] for t in triangles])
            kind = "source_shell_yz_slabs"
        else:
            pieces = split_hull_vertex_budget(closed_convex_hull(points))
            zero_volume = []
            kind = "conservative_component_hull"
        plan.append({"source_component_index": component, "kind": kind, "pieces": pieces,
            "source_signed_surface_volume_m3": source_volume,
            "sum_piece_volume_m3": sum(p["volume_m3"] for p in pieces),
            "zero_volume_boundary_cells": zero_volume})
    return plan


def apply_slab_candidate(stage, source_records, plan, manifest, contact_offset_m, rest_offset_m, profile):
    """Author prepared closed hulls on the existing moving palm rigid body."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    records, summaries = [], []
    for item in plan:
        component = item["source_component_index"]
        original = source_records[component]
        parent = stage.GetPrimAtPath(original["prim"])
        if item["kind"] == "retain_v1_decomposition":
            records.append(dict(original, source_component_index=component,
                                approximation="convexDecomposition", expected_hulls=32))
            summaries.append(item)
            continue
        parent.RemoveAPI(UsdPhysics.CollisionAPI)
        parent.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        parent.RemoveAppliedSchema("PhysxConvexDecompositionCollisionAPI")
        parent.RemoveAppliedSchema("PhysxCollisionAPI")
        for index, piece in enumerate(item["pieces"]):
            path = parent.GetPath().AppendChild(f"closed_piece_{index:03d}")
            mesh = UsdGeom.Mesh.Define(stage, path)
            mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in piece["points"]])
            mesh.CreateFaceVertexCountsAttr([3] * len(piece["faces"]))
            mesh.CreateFaceVertexIndicesAttr([v for face in piece["faces"] for v in face])
            mesh.CreateOrientationAttr("rightHanded")
            mesh.CreateSubdivisionSchemeAttr("none")
            prim = mesh.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            prim.AddAppliedSchema("PhysxConvexHullCollisionAPI")
            prim.CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int, custom=False).Set(255)
            prim.CreateAttribute("physxConvexHullCollision:minThickness", Sdf.ValueTypeNames.Float, custom=False).Set(.001)
            prim.AddAppliedSchema("PhysxCollisionAPI")
            for attr in parent.GetAttributes():
                if str(attr.GetName()).startswith("physxCollision:") and attr.HasAuthoredValueOpinion():
                    prim.CreateAttribute(attr.GetName(), attr.GetTypeName(), custom=False).Set(attr.Get())
            for name, value in (("contactOffset", contact_offset_m), ("restOffset", rest_offset_m)):
                prim.CreateAttribute("physxCollision:" + name, Sdf.ValueTypeNames.Float, custom=False).Set(value)
            owners = parent.GetRelationship("physics:simulationOwner").GetTargets()
            if owners:
                UsdPhysics.CollisionAPI(prim).CreateSimulationOwnerRel().SetTargets(owners)
            # Inherit the original source MaterialBindingAPI and exact transform
            # through identity children; never introduce an extra rigid body.
            metadata = {k: v for k, v in piece.items() if k not in ("points", "faces")}
            records.append(dict(metadata, prim=str(path), source_component_index=component,
                approximation="convexHull", expected_hulls=1, point_count=len(piece["points"]),
                triangle_count=len(piece["faces"]), collision_input_geometry_sha256=geometry_sha256(
                    piece["points"], [3] * len(piece["faces"]), [v for face in piece["faces"] for v in face])))
        summaries.append({k: v for k, v in item.items() if k != "pieces"} |
                         {"closed_hull_piece_count": len(item["pieces"])})
    manifest.update({"schema_version": 2, "source_connected_component_count": profile["component_count"],
        "source_components": source_records, "components": records, "component_count": len(records),
        "expected_authored_palm_collider_count": len(records),
        "maximum_requested_palm_hulls": sum(r["expected_hulls"] for r in records),
        "expected_palm_hulls_if_all_cooking_succeeds": sum(r["expected_hulls"] for r in records),
        "collision_input_triangles_retriangulated": True,
        "source_visual_triangles_and_dimensions_preserved": True,
        "collision_solid_policy": "conservative closed hulls contain clipped source material; concave voids may be filled and require measured clearance tests",
        "slab_axes": ["Y", "Z"], "slab_width_m": .004,
        "slab_grid_origin_m": 0., "slab_source_component_index": profile["shell_component"],
        "convex_decomposition_retained_for_source_components": [profile["retained_component"]],
        "convex_input_max_vertices": 120, "convex_cooking_vertex_limit": 255,
        "convex_min_thickness_m": .001, "component_geometry_summary": summaries,
        "preset_rationale": "The measured component23 cavity error was2.54mm. CPU tests against recorded thumb2 hulls found zero overlap for4mm Y/Z cells;2mm one-axis slabs and4mm X/Y or X/Z cells still overlapped. Closed caps come from convex hulls of clipped source boundaries. Convex pieces are bisected with volume conservation until at most120 vertices. Component40 retains v1 decomposition because its single hull introduces new cavity overlap. Other source components use conservative hulls; their added volume is explicit, not exact CAD fidelity.",
        "exact_RH56E2_equivalence": False, "moving_palm_qualified": False, "simulation_qualified": False})
    if profile["side"] == "left":
        manifest["preset_rationale"] = (
            "Apply the right-v2 geometric method to independently fingerprinted left source geometry: "
            "20 source components, shell1 cut into4mm Y/Z cells, component7 retains32-hull decomposition. "
            "The left source has different internal geometry and is not substituted with a mirrored right mesh. "
            "Closed convex caps and bounded-vertex cuts retain source material conservatively. "
            "Right-side results do not qualify the left; require independent left cooking, cavity and dynamics checks.")
        manifest["mirrored_right_geometry_substituted"] = False


def replace_left_thumb_with_slabs(stage, source_mesh_path, rigid_body_path, *,
                                  contact_offset_m=.0004905000096186996, rest_offset_m=0.,
                                  expected_geometry_sha256=LEFT_THUMB_GEOMETRY_SHA256):
    """Replace only the independently pinned left thumb2's collision shape.

    The default 16-hull cooking exceeded the source convex envelope and
    intersected real palm material at zero pose. Re-clip the source surface,
    rather than splitting already inflated cooked hulls. This candidate retains
    source material conservatively; neither cavity fidelity nor motion is proven
    by construction. The explicit hash override is for reviewed CPU fixtures.
    """
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    if not (math.isfinite(contact_offset_m) and math.isfinite(rest_offset_m)
            and contact_offset_m > rest_offset_m >= 0):
        raise ValueError("Require finite positive contact offset above nonnegative rest offset")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.:
        raise ValueError("Thumb candidate requires meters")
    source, body = stage.GetPrimAtPath(source_mesh_path), stage.GetPrimAtPath(rigid_body_path)
    if (not source or not source.IsA(UsdGeom.Mesh) or source.IsInstanceProxy()
            or source.GetChildren() or not body or body.GetName() != "left_thumb_2"
            or not body.HasAPI(UsdPhysics.RigidBodyAPI)):
        raise ValueError("Expected an editable leaf source mesh on left_thumb_2")
    owner = source.GetParent()
    while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
        if owner.HasAPI(UsdPhysics.CollisionAPI):
            raise ValueError("Relocate wrapper collision APIs first")
        owner = owner.GetParent()
    if (owner != body or not source.HasAPI(UsdPhysics.CollisionAPI)
            or not UsdPhysics.CollisionAPI(source).GetCollisionEnabledAttr().Get()
            or UsdPhysics.MeshCollisionAPI(source).GetApproximationAttr().Get() != "convexDecomposition"
            or source.HasAPI(UsdPhysics.FilteredPairsAPI)):
        raise ValueError("Unexpected collision ownership or local filters")
    mesh = UsdGeom.Mesh(source)
    attrs = (mesh.GetPointsAttr(), mesh.GetFaceVertexCountsAttr(), mesh.GetFaceVertexIndicesAttr())
    orientation = str(mesh.GetOrientationAttr().Get())
    if any(a.GetNumTimeSamples() for a in attrs) or mesh.GetHoleIndicesAttr().Get():
        raise ValueError("Static, complete source geometry required")
    partition = partition_triangles(*(a.Get() for a in attrs), orientation=orientation)
    if partition.source_geometry_sha256 != expected_geometry_sha256:
        raise ValueError("Left thumb geometry differs from the reviewed source pin")
    if len(partition.components) != 1 or partition.boundary_edge_count or partition.nonmanifold_edge_count:
        raise ValueError("Expected one closed manifold thumb source component")
    if partition.source_geometry_sha256 == LEFT_THUMB_GEOMETRY_SHA256 and len(partition.faces) != 11526:
        raise ValueError("Pinned thumb source triangle count differs")
    triangles = [[partition.points[v] for v in f] for f in partition.faces]
    pieces, zero_cells = source_slab_hulls(triangles, axes=(1, 2), width_m=.002)
    if not pieces:
        raise ValueError("Source clipping produced no solid pieces")
    mass_prefixes = ("physics:mass", "physics:centerOfMass", "physics:diagonalInertia", "physics:principalAxes", "physics:density")
    masses_before = _authored_snapshot(body, mass_prefixes)
    filters_before = {str(p.GetPath()): tuple(map(str, p.GetRelationship("physics:filteredPairs").GetTargets()))
                      for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)}
    matrix = UsdGeom.Xformable(source).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    copied = [(a.GetName(), a.GetTypeName(), a.Get()) for a in source.GetAttributes()
              if str(a.GetName()).startswith("physxCollision:") and a.HasAuthoredValueOpinion()]
    owners = source.GetRelationship("physics:simulationOwner").GetTargets()
    root_path = source.GetPath().AppendChild(LEFT_THUMB_CANDIDATE_ID)
    UsdGeom.Xform.Define(stage, root_path).CreatePurposeAttr("guide")
    records = []
    for index, piece in enumerate(pieces):
        child = UsdGeom.Mesh.Define(stage, root_path.AppendChild(f"closed_piece_{index:03d}"))
        child.CreatePointsAttr([Gf.Vec3f(*p) for p in piece["points"]])
        child.CreateFaceVertexCountsAttr([3] * len(piece["faces"]))
        indices = [v for f in piece["faces"] for v in f]
        child.CreateFaceVertexIndicesAttr(indices)
        child.CreateOrientationAttr("rightHanded")
        child.CreateSubdivisionSchemeAttr("none")
        prim = child.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        prim.AddAppliedSchema("PhysxConvexHullCollisionAPI")
        prim.CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int, custom=False).Set(255)
        prim.CreateAttribute("physxConvexHullCollision:minThickness", Sdf.ValueTypeNames.Float, custom=False).Set(.001)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        for name, kind, value in copied:
            prim.CreateAttribute(name, kind, custom=False).Set(value)
        for name, value in (("contactOffset", contact_offset_m), ("restOffset", rest_offset_m)):
            prim.CreateAttribute("physxCollision:" + name, Sdf.ValueTypeNames.Float, custom=False).Set(value)
        if owners:
            UsdPhysics.CollisionAPI(prim).CreateSimulationOwnerRel().SetTargets(owners)
        if UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) != matrix:
            raise RuntimeError("Thumb piece transform differs from source")
        records.append({k: v for k, v in piece.items() if k not in ("points", "faces")} |
            {"prim": str(prim.GetPath()), "expected_hulls": 1, "point_count": len(piece["points"]),
             "triangle_count": len(piece["faces"]), "collision_input_geometry_sha256": geometry_sha256(
                 child.GetPointsAttr().Get(), child.GetFaceVertexCountsAttr().Get(), child.GetFaceVertexIndicesAttr().Get())})
    source.RemoveAPI(UsdPhysics.CollisionAPI)
    source.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    source.RemoveAppliedSchema("PhysxConvexDecompositionCollisionAPI")
    source.RemoveAppliedSchema("PhysxCollisionAPI")
    filters_after = {str(p.GetPath()): tuple(map(str, p.GetRelationship("physics:filteredPairs").GetTargets()))
                     for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)}
    if _authored_snapshot(body, mass_prefixes) != masses_before or filters_after != filters_before:
        raise RuntimeError("Unexpected mass, inertia or filter mutation")
    if geometry_sha256(*(a.Get() for a in attrs), orientation) != partition.source_geometry_sha256:
        raise RuntimeError("Original thumb geometry changed")
    tri = np.asarray(triangles)
    source_volume = abs(float(np.einsum("ij,ij->", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])) / 6))
    return {"schema_version": 1, "candidate_id": LEFT_THUMB_CANDIDATE_ID, "source_side": "left",
        "source_url": SOURCE_URL, "source_commit": SOURCE_COMMIT, "source_license": "BSD-3-Clause",
        "pinned_source_stl_sha256": LEFT_THUMB_STL_SHA256, "source_geometry_sha256": partition.source_geometry_sha256,
        "source_geometry_matches_pinned_donor": partition.source_geometry_sha256 == LEFT_THUMB_GEOMETRY_SHA256,
        "source_prim": str(source.GetPath()), "rigid_body_prim": str(body.GetPath()),
        "source_local_to_world_row_matrix": [list(row) for row in matrix], "source_orientation": orientation,
        "source_triangle_count": len(partition.faces), "source_connected_component_count": 1,
        "source_signed_surface_volume_m3": source_volume, "sum_closed_piece_volume_m3": sum(p["volume_m3"] for p in pieces),
        "slab_axes": ["Y", "Z"], "slab_width_m": .002, "slab_grid_origin_m": 0.,
        "zero_volume_boundary_cells": zero_cells, "convex_input_max_vertices": 120,
        "convex_cooking_vertex_limit": 255, "convex_min_thickness_m": .001,
        "contact_offset_m": contact_offset_m, "rest_offset_m": rest_offset_m,
        "contact_offset_policy": "unchanged explicit left-thumb2 whole-collider control readback",
        "components": records, "expected_authored_collider_count": len(records), "expected_live_hulls": len(records),
        "source_collision_apis_removed": True, "source_visual_triangles_and_dimensions_preserved": True,
        "collision_input_triangles_retriangulated": True, "articulation_mass_or_inertia_changed": False,
        "collision_pairs_filtered": False, "source_transform_changed": False,
        "collision_solid_policy": "closed convex caps conservatively contain source material in Y/Z cells; voids may be filled",
        "rationale": "Left01 default thumb2 cooking exceeded the full source convex envelope by up to1.602mm and intersected source palm.2mm Y/Z source clips had zero intersections against recorded q0 palm hulls, with less added volume than one-axis or other two-axis candidates. Actual cooking and pose-domain validation remain required.",
        "simulation_qualified": False, "exact_RH56E2_equivalence": False, "grasp_qualified": False, "hardware_authorized": False}
