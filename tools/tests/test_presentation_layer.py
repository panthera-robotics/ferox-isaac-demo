"""Sprint P presentation layer: camera/overlay declarations are bounded, and the overlay insertion neutralises every physics
schema under its root (exercised with lightweight stand-ins for the USD classes)."""
import os, sys, tempfile, unittest
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from isaac.twin.inspire import presentation_layer as P


class Attr:
    def __init__(self): self.value = None
    def Set(self, v): self.value = v


class FakePrim:
    def __init__(self, path, apis=(), isa=()):
        self.path, self.apis, self.isa, self.active, self.attrs = path, set(apis), set(isa), True, {}
    def GetPath(self): return self.path
    def IsA(self, t): return t in self.isa
    def HasAPI(self, t): return getattr(t, 'tag', t) in self.apis
    def RemoveAPI(self, t): self.apis.discard(getattr(t, 'tag', t))
    def SetActive(self, v): self.active = v
    def IsValid(self): return True


class FakeStage:
    def __init__(self, prims): self.prims = prims
    def Traverse(self): return list(self.prims)
    def GetPrimAtPath(self, p): return next(x for x in self.prims if x.path == p)


class FakeUsdPhysics:
    Scene, Joint, ArticulationRootAPI = 'Scene', 'Joint', 'ArticulationRootAPI'
    class RigidBodyAPI:
        tag = 'RigidBodyAPI'
        def __init__(self, prim): self.prim = prim
        def CreateRigidBodyEnabledAttr(self, v): self.prim.attrs['rigidBodyEnabled'] = v; return Attr()
    class CollisionAPI:
        tag = 'CollisionAPI'
        def __init__(self, prim): self.prim = prim
        def CreateCollisionEnabledAttr(self, v): self.prim.attrs['collisionEnabled'] = v; return Attr()


class FakeXformable:
    ops = []
    def __init__(self, prim): self.prim = prim
    def ClearXformOpOrder(self): FakeXformable.ops.append(('clear', self.prim.path))
    def AddTranslateOp(self):
        a = Attr(); FakeXformable.ops.append(('translate', self.prim.path, a)); return a


class FakeUsdGeom:
    Xformable = FakeXformable


class FakeGf:
    @staticmethod
    def Vec3d(*v): return tuple(v)


class PresentationLayerTests(unittest.TestCase):
    def test_camera_and_overlay_declarations_are_bounded(self):
        static = {'label': 'wide', 'position_m': [1.2, -1.4, 1.5], 'look_at_m': [.3, -.4, 1.2], 'resolution': [1280, 720], 'focal_length_mm': 24}
        track = {'label': 'grip', 'track': 'right_wrist_yaw_link', 'offset_m': [.05, -.35, .15], 'focal_length_mm': 35}
        obj = {'label': 'puck', 'track': 'object', 'offset_m': [.3, -.3, .2]}
        overlay = {'usd_path': '/workspace/hero/office.usd', 'root': '/World/Visual/HeroWorld', 'translate_m': [-5.6, 1.07, .21], 'note': 'render only'}
        P.validate_presentation({'cameras': [static, track, obj], 'visual_overlay': overlay, 'note': 'hero'})
        P.validate_presentation({'visual_overlay': {'usd_path': '/x/y.usda'}})
        for bad in ({'cameras': []}, {'cameras': [static, dict(static)]}, {'cameras': [{**static, 'resolution': [64, 64]}]}, {'cameras': [{**track, 'track': 'head'}]},
                    {'cameras': [{**track, 'position_m': [0, 0, 1]}]}, {'cameras': [{k: v for k, v in static.items() if k != 'look_at_m'}]}, {'cameras': [{**static, 'focal_length_mm': 500}]},
                    {'visual_overlay': {'root': '/World/Visual'}}, {'visual_overlay': {**overlay, 'usd_path': '/x.obj'}}, {'visual_overlay': {**overlay, 'root': '/World/G1'}},
                    {'visual_overlay': {**overlay, 'translate_m': [0, 0]}}, {'visual_overlay': {**overlay, 'scale': 2}}, {'extra': 1}, {'note': 3}, 'hero'):
            with self.assertRaises(ValueError):
                P.validate_presentation(bad)

    def test_overlay_insertion_neutralises_every_physics_schema_under_the_root(self):
        with tempfile.NamedTemporaryFile(suffix='.usda', delete=False) as f:
            f.write(b'#usda 1.0\n'); usd = f.name
        prims = [FakePrim('/World/Visual/Hero'), FakePrim('/World/Visual/Hero/Wall', apis=('CollisionAPI',)),
                 FakePrim('/World/Visual/Hero/Cart', apis=('RigidBodyAPI', 'CollisionAPI')), FakePrim('/World/Visual/Hero/Hinge', isa=('Joint',)),
                 FakePrim('/World/Visual/Hero/Robot', apis=('ArticulationRootAPI',)), FakePrim('/World/Visual/Hero/PhysicsScene', isa=('Scene',)),
                 FakePrim('/World/G1/pelvis', apis=('RigidBodyAPI', 'CollisionAPI')), FakePrim('/World/VisualOther', apis=('CollisionAPI',))]
        stage = FakeStage(prims); added = []
        audit = P.add_visual_overlay(stage, {'usd_path': usd, 'root': '/World/Visual/Hero', 'translate_m': [1., 2., 3.]}, lambda p, r: added.append((p, r)), FakeUsdGeom, FakeUsdPhysics, FakeGf)
        os.unlink(usd)
        self.assertEqual(added, [(usd, '/World/Visual/Hero')])
        self.assertEqual({k: audit[k] for k in ('prims', 'collision_disabled', 'rigid_bodies_disabled', 'joints_deactivated', 'articulation_roots_removed', 'physics_scenes_deactivated')},
                         {'prims': 6, 'collision_disabled': 2, 'rigid_bodies_disabled': 1, 'joints_deactivated': 1, 'articulation_roots_removed': 1, 'physics_scenes_deactivated': 1})
        self.assertEqual(prims[1].attrs, {'collisionEnabled': False}); self.assertEqual(prims[2].attrs, {'rigidBodyEnabled': False, 'collisionEnabled': False})
        self.assertFalse(prims[3].active); self.assertFalse(prims[5].active); self.assertNotIn('ArticulationRootAPI', prims[4].apis)
        self.assertEqual(prims[6].attrs, {}); self.assertEqual(prims[7].attrs, {})           # the robot and a sibling outside the root are untouched
        self.assertEqual([op for op in FakeXformable.ops if op[0] == 'translate'][-1][2].value, (1., 2., 3.))
        self.assertEqual(audit['translate_m'], [1., 2., 3.]); self.assertEqual(len(audit['usd_sha256']), 64)
        with self.assertRaises(ValueError):
            P.add_visual_overlay(stage, {'usd_path': '/nonexistent/x.usd'}, lambda p, r: None, FakeUsdGeom, FakeUsdPhysics, FakeGf)


if __name__ == '__main__':
    unittest.main()
