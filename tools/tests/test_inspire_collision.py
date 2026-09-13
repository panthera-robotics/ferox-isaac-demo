"""CPU geometry and USD ownership tests; these do not qualify physics."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from inspire_collision import (CANDIDATE_ID, DECOMPOSITION, geometry_sha256,
                               partition_triangles, replace_palm_with_components,
                               clip_polygon, closed_convex_hull, split_hull_vertex_budget,
                               source_slab_hulls, LEFT_SLAB_CANDIDATE_ID,
                               PINNED_LEFT_GEOMETRY_SHA256, PINNED_GEOMETRY_SHA256)

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
except ImportError:
    Usd = None

try:
    import numpy as np
    from scipy.spatial import ConvexHull
except ImportError:
    np = None


class TopologyTests(unittest.TestCase):
    def test_duplicated_stl_vertices_connect_by_exact_shared_edge(self):
        # Two triangles form a square; the third only touches a vertex.
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0),
                  (1, 0, 0), (1, 1, 0), (0, 1, 0),
                  (1, 1, 0), (2, 1, 0), (1, 2, 0)]
        result = partition_triangles(points, [3] * 3, range(9))
        self.assertEqual(result.components, ((0, 1), (2,)))
        self.assertEqual(len(result.points), 6)
        self.assertEqual(result.original_point_count, 9)
        # Reconstruct source triangle coordinates, including their winding.
        for component, face_ids in enumerate(result.components):
            vertices, indices = result.component_mesh(component)
            actual = [vertices[i] for i in indices]
            expected = [tuple(float(v) for v in p) for f in face_ids for p in points[f * 3:f * 3 + 3]]
            self.assertEqual(actual, expected)
        again = partition_triangles(points, [3] * 3, range(9))
        self.assertEqual(result, again)
        self.assertEqual(result.component_fingerprint(0), again.component_fingerprint(0))

    def test_no_tolerance_welding_or_chirality_change(self):
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0),
                  (1 + 1e-12, 0, 0), (1, 1, 0), (0, 1, 0)]
        result = partition_triangles(points, [3, 3], range(6), "leftHanded")
        self.assertEqual(result.components, ((0,), (1,)))
        self.assertEqual(len(result.points), 5)
        self.assertNotEqual(result.source_geometry_sha256,
                            partition_triangles(points, [3, 3], range(6)).source_geometry_sha256)
        self.assertEqual(result.component_mesh(1)[0][0], (1 + 1e-12, 0, 0))

    def test_rejects_malformed_geometry_instead_of_dropping_faces(self):
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
        for counts, indices in (([4], [0, 1, 2, 0]), ([3], [0, 1]),
                                ([3], [-1, 1, 2]), ([3], [0, 1, 3]),
                                ([3], [0, 0, 2]), ([3], [0, 1.0, 2])):
            with self.subTest(counts=counts, indices=indices), self.assertRaises(ValueError):
                partition_triangles(points, counts, indices)
        with self.assertRaises(ValueError):
            partition_triangles([(float("nan"), 0, 0)] + points[1:], [3], [0, 1, 2])

    def test_plane_clip_preserves_source_side_and_closes_at_exact_plane(self):
        polygon = [(0., 0., 0.), (2., 0., 0.), (0., 2., 0.)]
        clipped = clip_polygon(polygon, 0, 1., True)
        self.assertIn((2., 0., 0.), clipped)
        self.assertIn((1., 0., 0.), clipped)
        self.assertIn((1., 1., 0.), clipped)
        self.assertTrue(all(p[0] >= 1. for p in clipped))


@unittest.skipIf(np is None, "SciPy unavailable; use existing Isaac Sim Python for geometry tests")
class SlabGeometryTests(unittest.TestCase):
    @staticmethod
    def box(bounds):
        return closed_convex_hull([(x, y, z) for x in bounds[0] for y in bounds[1] for z in bounds[2]])

    def test_closed_caps_and_recursive_cuts_conserve_convex_material(self):
        # A finely sampled convex sphere needs multiple bounded-vertex pieces.
        points = []
        for i in range(1, 16):
            latitude = np.pi * i / 16
            for j in range(32):
                longitude = 2 * np.pi * j / 32
                points.append((.01 * np.sin(latitude) * np.cos(longitude),
                               .01 * np.sin(latitude) * np.sin(longitude), .01 * np.cos(latitude)))
        points += [(0., 0., -.01), (0., 0., .01)]
        hull = closed_convex_hull(points)
        pieces = split_hull_vertex_budget(hull, max_vertices=60)
        self.assertGreater(len(pieces), 1)
        self.assertTrue(all(len(p["points"]) <= 60 for p in pieces))
        self.assertAlmostEqual(sum(p["volume_m3"] for p in pieces), hull["volume_m3"], places=12)
        for piece in pieces:
            faces = np.asarray(piece["points"])[np.asarray(piece["faces"])]
            signed = np.einsum("ij,ij->", faces[:, 0], np.cross(faces[:, 1], faces[:, 2])) / 6
            self.assertGreater(signed, 0)
            self.assertAlmostEqual(signed, piece["volume_m3"], places=12)
            edges = {}
            for a, b, c in piece["faces"]:
                for edge in ((a, b), (b, c), (c, a)):
                    key = tuple(sorted(edge))
                    edges[key] = edges.get(key, 0) + 1
            self.assertTrue(all(count == 2 for count in edges.values()))

    def test_yz_cells_preserve_u_solid_and_leave_its_cavity_open(self):
        boxes = [self.box(((0., .004), (0., .004), (0., .016))),
                 self.box(((0., .004), (.012, .016), (0., .016))),
                 self.box(((0., .004), (.004, .012), (0., .004)))]
        triangles = [[box["points"][i] for i in face] for box in boxes for face in box["faces"]]
        pieces, _ = source_slab_hulls(triangles, axes=(1, 2), width_m=.004)
        # Spatial convexification may conservatively add corner volume. It must
        # cover the source solid while retaining this central open cavity.
        self.assertGreaterEqual(sum(p["volume_m3"] for p in pieces),
                                sum(b["volume_m3"] for b in boxes) - 1e-12)
        cavity = np.array([.002, .008, .010])
        equations = [ConvexHull(p["points"]).equations for p in pieces]
        self.assertFalse(any(np.all(e[:, :3] @ cavity + e[:, 3] <= 1e-10) for e in equations))
        # Every original surface vertex and each triangle centroid is covered.
        for triangle in triangles:
            for point in list(triangle) + [np.mean(triangle, axis=0)]:
                self.assertTrue(any(np.all(e[:, :3] @ point + e[:, 3] <= 1e-10) for e in equations))
        with self.assertRaises(ValueError):
            source_slab_hulls(triangles, axes=(0, 1, 2))


@unittest.skipIf(Usd is None, "USD bindings unavailable; run with existing Isaac Sim Python for USD tests")
class UsdReplacementTests(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        body = UsdGeom.Xform.Define(self.stage, "/Hand/palm")
        self.body = body.GetPrim()
        body.AddTranslateOp().Set(Gf.Vec3d(1, 2, 3))
        self.motion = body.AddRotateXYZOp()
        self.motion.Set(Gf.Vec3f(10, 20, 30))
        UsdPhysics.RigidBodyAPI.Apply(self.body)
        mass = UsdPhysics.MassAPI.Apply(self.body)
        mass.CreateMassAttr(.62266)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(.01, .02, .03))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(.001, .002, .003))
        UsdPhysics.FilteredPairsAPI.Apply(self.body).CreateFilteredPairsRel().AddTarget("/Hand/adjacent")
        self.mesh = UsdGeom.Mesh.Define(self.stage, "/Hand/palm/source")
        self.mesh.AddTranslateOp().Set(Gf.Vec3d(.01, .02, .03))
        self.mesh.AddScaleOp().Set(Gf.Vec3f(2, 3, 4))
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
                  (4, 0, 0), (5, 0, 0), (4, 1, 0), (4, 0, 1)]
        indices = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3]
        indices += [i + 4 for i in indices]
        self.mesh.CreatePointsAttr(points)
        self.mesh.CreateFaceVertexCountsAttr([3] * 8)
        self.mesh.CreateFaceVertexIndicesAttr(indices)
        self.mesh.CreateOrientationAttr("leftHanded")
        self.mesh.CreateSubdivisionSchemeAttr("none")
        UsdPhysics.CollisionAPI.Apply(self.mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(self.mesh.GetPrim()).CreateApproximationAttr("convexDecomposition")
        material = UsdShade.Material.Define(self.stage, "/Material")
        UsdPhysics.MaterialAPI.Apply(material.GetPrim()).CreateStaticFrictionAttr(.5)
        UsdShade.MaterialBindingAPI.Apply(self.mesh.GetPrim()).Bind(material, materialPurpose="physics")
        other = UsdGeom.Cube.Define(self.stage, "/Hand/other")
        UsdPhysics.CollisionAPI.Apply(other.GetPrim())
        self.other_spec_before = str(self.stage.GetRootLayer().GetPrimAtPath(other.GetPath()))
        self.source_hash = geometry_sha256(points, [3] * 8, indices, "leftHanded")

    def replace(self, **kwargs):
        return replace_palm_with_components(self.stage, str(self.mesh.GetPath()), str(self.body.GetPath()),
            contact_offset_m=.0012860533315688372, expected_geometry_sha256=self.source_hash, **kwargs)

    def test_preserves_geometry_material_moving_body_mass_and_filters(self):
        original_points = self.mesh.GetPointsAttr().Get()
        manifest = self.replace()
        self.assertEqual(manifest["component_count"], 2)
        self.assertEqual(manifest["component_triangle_counts"], [4, 4])
        self.assertEqual(manifest["maximum_requested_palm_hulls"], 64)
        self.assertFalse(manifest["exact_RH56E2_equivalence"])
        self.assertFalse(manifest["moving_palm_qualified"])
        self.assertFalse(manifest["source_geometry_matches_pinned_donor"])
        self.assertTrue(manifest["source_collision_apis_removed"])
        self.assertEqual(self.mesh.GetPointsAttr().Get(), original_points)
        self.assertAlmostEqual(UsdPhysics.MassAPI(self.body).GetMassAttr().Get(), .62266, places=6)
        self.assertEqual(UsdPhysics.FilteredPairsAPI(self.body).GetFilteredPairsRel().GetTargets(),
                         [Sdf.Path("/Hand/adjacent")])
        self.assertEqual(len([p for p in self.stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]), 1)
        self.assertTrue(self.stage.GetPrimAtPath("/Hand/other").HasAPI(UsdPhysics.CollisionAPI))
        source_indices = list(self.mesh.GetFaceVertexIndicesAttr().Get())
        # Both at q0 and after rotating the body, every source triangle retains
        # exactly the same world coordinates in the replacement geometry.
        for rotation in ((10, 20, 30), (40, -50, 60)):
            self.motion.Set(Gf.Vec3f(*rotation))
            transform = self.mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            for record in manifest["components"]:
                prim = self.stage.GetPrimAtPath(record["prim"])
                mesh = UsdGeom.Mesh(prim)
                self.assertEqual(mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default()), transform)
                self.assertEqual(str(mesh.GetOrientationAttr().Get()), "leftHanded")
                actual = [tuple(mesh.GetPointsAttr().Get()[i]) for i in mesh.GetFaceVertexIndicesAttr().Get()]
                expected = [tuple(original_points[source_indices[f * 3 + k]])
                            for f in record["source_face_indices"] for k in range(3)]
                self.assertEqual(actual, expected)
                self.assertFalse(prim.HasAPI(UsdPhysics.RigidBodyAPI))
                self.assertEqual(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), "convexDecomposition")
                for key, value in DECOMPOSITION.items():
                    self.assertAlmostEqual(prim.GetAttribute("physxConvexDecompositionCollision:" + key).Get(), value)
                self.assertAlmostEqual(prim.GetAttribute("physxCollision:contactOffset").Get(), .0012860533315688372)
                binding, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
                self.assertEqual(str(binding.GetPath()), "/Material")
        self.assertFalse(self.mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI))
        parsed = UsdPhysics.LoadUsdPhysicsFromRange(self.stage, [Sdf.Path("/Hand")])
        self.assertEqual(len(parsed[UsdPhysics.ObjectType.MeshShape][0]), 2)
        self.assertEqual(len(parsed[UsdPhysics.ObjectType.RigidBody][0]), 1)

    def test_rejects_wrong_provenance_and_repeated_replacement(self):
        before = self.stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            replace_palm_with_components(self.stage, str(self.mesh.GetPath()), str(self.body.GetPath()),
                                         contact_offset_m=.001)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        self.assertNotEqual(PINNED_LEFT_GEOMETRY_SHA256, PINNED_GEOMETRY_SHA256)
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            replace_palm_with_components(self.stage, str(self.mesh.GetPath()), str(self.body.GetPath()),
                contact_offset_m=.001, candidate_id=LEFT_SLAB_CANDIDATE_ID)
        self.assertEqual(self.stage.GetRootLayer().ExportToString(), before)
        self.replace()
        with self.assertRaises(ValueError):
            self.replace()

    def test_rejects_wrong_body_and_invalid_offsets_before_mutation(self):
        for offset in (0, -.001, float("nan")):
            with self.assertRaises(ValueError):
                replace_palm_with_components(self.stage, str(self.mesh.GetPath()), str(self.body.GetPath()),
                    contact_offset_m=offset, expected_geometry_sha256=self.source_hash)
        with self.assertRaisesRegex(ValueError, "rigid body"):
            replace_palm_with_components(self.stage, str(self.mesh.GetPath()), "/Hand/other",
                contact_offset_m=.001, expected_geometry_sha256=self.source_hash)
        self.assertTrue(self.mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI))


if __name__ == "__main__":
    unittest.main()
