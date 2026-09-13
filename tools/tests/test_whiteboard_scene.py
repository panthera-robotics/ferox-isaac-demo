"""CPU-only whiteboard mechanics, frames, contact admission and USD structure checks."""
from dataclasses import asdict,replace
import importlib.util
import math
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'isaac'))
from twin.inspire.whiteboard_scene import (BoardFrame,BoardParameters,HolderParameters,SceneConfig,InvalidScene,
                                           build_scene,compression_from_poses,reduce_tip_contacts,rotate)
from twin.inspire.contact_ink import ContactSample,IntendedStroke,evaluate
spec=importlib.util.spec_from_file_location('board_contact',ROOT/'tools/probes/board_contact.py')
probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)


class ConfigurationTests(unittest.TestCase):
    def test_frame_is_right_handed_and_roundtrips_rotated_points(self):
        frame=BoardFrame();axes=[rotate(frame.orientation_world_qwxyz,p) for p in [(1,0,0),(0,1,0),(0,0,1)]]
        for actual,expected in zip(axes,[(1,0,0),(0,0,1),(0,-1,0)]):
            for a,b in zip(actual,expected):self.assertAlmostEqual(a,b)
        for point in [(.1,.2,.3),(-.2,.04,0.)]:
            for a,b in zip(frame.to_board(frame.to_world(point)),point):self.assertAlmostEqual(a,b)
        with self.assertRaises(InvalidScene):BoardFrame(orientation_world_qwxyz=(1,1,0,0))

    def test_nonfinite_unknown_and_unsafe_parameters_are_refused(self):
        for data in [{'holder':{'slider_travel_m':float('nan')}},{'holder':{'stiffness_n_m':-1}},
                     {'holder':{'nominal_compression_m':.020}},{'holder':{'preload_n':5,'drive_force_limit_n':1}},
                     {'holder':{'slider_friction_coefficient':-1}},{'hardware_authorized':True},
                     {'frame':{'matrix':[]}}, {'provenance':'measured hardware calibration'}]:
            with self.assertRaises((InvalidScene,TypeError)):SceneConfig.from_dict(data)

    def test_twenty_mm_travel_is_not_symmetric_at_five_mm(self):
        reserves=HolderParameters().travel_reserves(.005)
        self.assertEqual(reserves['toward_contact_loss_m'],.005)
        self.assertEqual(reserves['toward_bottomout_m'],.015)
        self.assertTrue(reserves['geometric_only_not_lateral_breakaway'])

    def test_preload_has_correct_force_sign_and_damping_units(self):
        h=HolderParameters()
        self.assertEqual(h.spring_target_m,-.005)
        self.assertEqual(h.force_model_toward_tip_n(0.,0.),1.)
        self.assertEqual(h.force_model_toward_tip_n(.005,0.),2.)
        self.assertAlmostEqual(h.force_model_toward_tip_n(.005,.1),2.1)
        self.assertEqual(h.force_model_toward_tip_n(1.,0.),h.drive_force_limit_n)

    def test_pose_measurement_uses_actual_rotated_relative_geometry(self):
        h=HolderParameters();frame=BoardFrame();p=frame.to_world((.04,.03,.2))
        for compression in [0.,.005,.020]:
            delta=rotate(frame.orientation_world_qwxyz,(0.,0.,-h.tip_offset_m+h.nib_radius_m+compression))
            nib=tuple(a+b for a,b in zip(p,delta))
            self.assertAlmostEqual(compression_from_poses(p,frame.orientation_world_qwxyz,nib,h),compression)

    def test_fixture_scope_never_certifies_grasp_or_writing(self):
        for mode in ['free_dynamic','driven_carriage']:
            scope=SceneConfig(holder_mode=mode).scope
            self.assertFalse(scope['grasp_qualified']);self.assertFalse(scope['writing_qualified'])
            self.assertEqual(scope['attachment_active'],mode=='driven_carriage')
            self.assertEqual(scope['fixture_support_active'],mode=='driven_carriage')

    def test_calibration_hash_changes_with_physical_or_frame_parameters(self):
        config=SceneConfig()
        self.assertEqual(SceneConfig.from_dict(asdict(config)).sha256,config.sha256)
        self.assertNotEqual(replace(config,holder=replace(config.holder,preload_n=.9)).sha256,config.sha256)
        self.assertNotEqual(replace(config,frame=BoardFrame(origin_world_m=(.01,0,.85))).sha256,config.sha256)


class ContactTests(unittest.TestCase):
    def reduction(self,rows):
        return reduce_tip_contacts(rows,tip_collider='/Tip',board_collider='/Board',frame=BoardFrame(),nib_position_world=(0.,-.01,.85),dt_s=.005)

    def contact(self,collider0='/Tip',collider1='/Board',impulse=(0.,-.01,0.),position=(.02,0.,.88)):
        return {'collider0':collider0,'collider1':collider1,'impulse_ns':impulse,'position_world_m':position}

    def test_absent_contacts_cannot_draw_perfect_requested_path(self):
        observation=self.reduction([])
        self.assertFalse(observation['nib_board_contact']);self.assertEqual(observation['normal_force_n'],0)
        stroke=IntendedStroke('I',((0.,0.),(.1,0.)))
        samples=[ContactSample(i,i*.005,.005,(i*.01,0.,0.),False,True,.005,'I',0,i/10,0.,0.,False,False,False) for i in range(11)]
        result=evaluate([stroke],samples)
        self.assertEqual(result['coverage_fraction'],0.)
        self.assertFalse(result['accepted'])

    def test_missing_observation_stays_invalid_and_not_zero(self):
        observation=self.reduction(None)
        self.assertIsNone(observation['nib_board_contact']);self.assertIsNone(observation['normal_force_n'])

    def test_shaft_contacts_and_proximity_do_not_mark(self):
        self.assertFalse(self.reduction([self.contact('/Shaft')])['nib_board_contact'])
        self.assertFalse(self.reduction([self.contact(impulse=(0,0,0))])['nib_board_contact'])
        self.assertFalse(self.reduction([self.contact(impulse=(1,0,0))])['nib_board_contact'])

    def test_contact_location_is_independent_impulse_weighted_measurement(self):
        a=self.contact(impulse=(0.,-.01,0.),position=(.01,0.,.85))
        b=self.contact('/Board','/Tip',impulse=(0.,.03,0.),position=(.03,0.,.85))
        result=self.reduction([a,b])
        self.assertTrue(result['nib_board_contact'])
        self.assertAlmostEqual(result['normal_force_n'],8.)
        self.assertAlmostEqual(result['position_board_m'][0],.025)
        self.assertEqual(result['position_source'],'measured_contact_impulse_weighted')

    def test_nonfinite_measured_contact_is_rejected(self):
        with self.assertRaises(InvalidScene):self.reduction([self.contact(impulse=(0,float('nan'),0))])

    def test_attached_real_contact_still_cannot_pass_writing(self):
        stroke=IntendedStroke('I',((0.,0.),(.01,0.)))
        samples=[ContactSample(i,i*.005,.005,(i*.001,0.,0.),True,True,.005,'I',0,i/10,2.,.01,True,True,False) for i in range(11)]
        report=evaluate([stroke],samples)
        self.assertAlmostEqual(report['coverage_fraction'],1.)
        self.assertFalse(report['accepted']);self.assertIn('attachment_active',report['failure_reasons'])

    def test_instrument_motion_preserves_stroke_order_and_releases(self):
        cfg=SceneConfig(holder_mode='driven_carriage');strokes,plan=probe.trajectory(cfg)
        self.assertEqual([s.stroke_id for s in strokes],['I','L'])
        down=[p for p in plan if p['pen_down']]
        keys=[]
        for p in down:
            key=(p['stroke_id'],p['segment_index'])
            if not keys or key!=keys[-1]:keys.append(key)
        self.assertEqual(keys,[('I',0),('L',0),('L',1)])
        self.assertFalse(plan[-1]['pen_down']);self.assertEqual(plan[-1]['carriage_target_m'][2],0.)
        self.assertLess(len(plan)*cfg.physics_dt_s,15.)


try:
    from pxr import Usd,UsdPhysics,PhysxSchema
    HAS_USD=True
except ImportError:
    HAS_USD=False


@unittest.skipUnless(HAS_USD,'Installed USD/PhysX CPU bindings are required for authoring checks')
class UsdAuthoringTests(unittest.TestCase):
    def test_free_object_has_only_internal_slider_and_no_world_attachment(self):
        stage=Usd.Stage.CreateInMemory();manifest=build_scene(stage)
        joints=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
        self.assertEqual(len(joints),1);self.assertTrue(joints[0].IsA(UsdPhysics.PrismaticJoint))
        joint=UsdPhysics.PrismaticJoint(joints[0]);self.assertTrue(joint.GetBody0Rel().GetTargets());self.assertTrue(joint.GetBody1Rel().GetTargets())
        for path in [manifest['holder_body'],manifest['nib_body']]:
            self.assertFalse(UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Get())
        self.assertFalse(manifest['scope']['attachment_active'])

    def test_carriage_has_three_drives_one_passive_slider_and_one_declared_fixture(self):
        stage=Usd.Stage.CreateInMemory();cfg=SceneConfig(holder_mode='driven_carriage');manifest=build_scene(stage,cfg)
        fixed=[p for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint)]
        prisms=[p for p in stage.Traverse() if p.IsA(UsdPhysics.PrismaticJoint)]
        self.assertEqual(len(fixed),1);self.assertEqual(len(prisms),4)
        self.assertEqual(len(manifest['carriage_joint_names']),3);self.assertTrue(manifest['scope']['attachment_active'])
        slider=stage.GetPrimAtPath('/World/Marker/NibCompression');drive=UsdPhysics.DriveAPI(slider,'linear')
        self.assertAlmostEqual(drive.GetTargetPositionAttr().Get(),-.005,places=7)
        self.assertAlmostEqual(drive.GetStiffnessAttr().Get(),200.)
        self.assertAlmostEqual(UsdPhysics.PrismaticJoint(slider).GetLowerLimitAttr().Get(),0.)
        self.assertAlmostEqual(UsdPhysics.PrismaticJoint(slider).GetUpperLimitAttr().Get(),.020,places=7)
        self.assertAlmostEqual(PhysxSchema.PhysxJointAPI(slider).GetJointFrictionAttr().Get(),.02,places=7)

    def test_free_mass_and_nib_collision_identity_are_explicit(self):
        stage=Usd.Stage.CreateInMemory();manifest=build_scene(stage)
        masses=[UsdPhysics.MassAPI(stage.GetPrimAtPath(path)).GetMassAttr().Get() for path in [manifest['holder_body'],manifest['nib_body']]]
        self.assertAlmostEqual(sum(masses),.088,places=7)
        self.assertTrue(stage.GetPrimAtPath(manifest['tip_collider']).HasAPI(UsdPhysics.CollisionAPI))
        self.assertTrue(stage.GetPrimAtPath(manifest['board_collider']).HasAPI(UsdPhysics.CollisionAPI))
        with self.assertRaises(InvalidScene):build_scene(stage)


if __name__=='__main__':unittest.main()
