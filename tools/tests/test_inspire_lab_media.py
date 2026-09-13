"""Actual returned-observation/reset mapping and bounded camera framing."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import unittest

path=Path(__file__).resolve().parents[1]/'probes/inspire_lab.py'
loader=importlib.util.spec_from_file_location('inspire_lab_media_probe',path)
probe=importlib.util.module_from_spec(loader);loader.loader.exec_module(probe)


class LabMediaTests(unittest.TestCase):
    def fixture(self,reset=False):
        current={'env_id':0,'episode_id':3 if reset else 2,'episode_step':0 if reset else 5,
            'measurement_joint_names':['joint_'+str(i) for i in range(53)],
            'q_rad':[.1]*53,'dq_rad_s':[0.]*53,
            'root_state_world_p_qwxyz_v_w':[2.,-4.,1.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.],
            'environment_origin_world_m':[2.,-4.,0.]}
        local=list(current['root_state_world_p_qwxyz_v_w']);local[:3]=[0.,0.,1.]
        previous={'env_id':0,'episode_id':2,'episode_step':100 if reset else 5,
            'sequence':99 if reset else 4,'physics_time_s':2. if reset else .1,
            'state_phase':'post_physics_before_any_automatic_reset'}
        return {'step':99 if reset else 4,'physics_s':previous['physics_time_s'],
            'observation':[current['q_rad']+current['dq_rad_s']+local],
            'environments':[current],'transitions':[previous],'reset_flags':[reset]}

    def test_optional_capture_is_disabled_by_default_and_max40pairs(self):
        self.assertEqual(probe.camera_schedule(False,200,.02),())
        steps=probe.camera_schedule(True,200,.02)
        self.assertEqual(len(steps),40);self.assertEqual((steps[0],steps[-1]),(4,199))
        for value in [1,'true',None]:
            with self.assertRaises(ValueError):probe.camera_schedule(value,200,.02)
        with self.assertRaises(ValueError):probe.camera_schedule(True,201,.02)
        with self.assertRaises(ValueError):probe.camera_schedule(True,200,.005)

    def test_capture_without_reset_matches_actual_origin_relative_observation(self):
        result=probe.render_state_record(**self.fixture())
        self.assertFalse(result['not_the_pre_reset_transition_image'])
        self.assertEqual(result['source_transitions'][0]['render_episode_step'],5)

    def test_reset_image_is_explicitly_new_episode_and_not_old_transition(self):
        result=probe.render_state_record(**self.fixture(True))
        link=result['source_transitions'][0]
        self.assertTrue(result['not_the_pre_reset_transition_image']);self.assertTrue(link['reset_before_image'])
        self.assertEqual(link['transition_episode_id'],2);self.assertEqual(link['transition_episode_step'],100)
        self.assertEqual(link['render_episode_id'],3);self.assertEqual(link['render_episode_step'],0)

    def test_pre_reset_observation_cannot_replace_returned_reset_observation(self):
        value=self.fixture(True);value['observation'][0][0]=.7
        with self.assertRaises(ValueError):probe.render_state_record(**value)

    def test_reset_flag_episode_and_step_mismatches_fail(self):
        for key,replacement in [('episode_id',2),('episode_step',100),('env_id',1)]:
            value=self.fixture(True);value['environments'][0][key]=replacement
            with self.assertRaises(ValueError):probe.render_state_record(**value)
        value=self.fixture(True);value['reset_flags']=[False]
        with self.assertRaises(ValueError):probe.render_state_record(**value)

    def test_clock_missing_names_nonfinite_and_width_mismatch_fail(self):
        value=self.fixture();value['physics_s']+=.02
        with self.assertRaises(ValueError):probe.render_state_record(**value)
        for key in ['q_rad','dq_rad_s','root_state_world_p_qwxyz_v_w','measurement_joint_names']:
            value=self.fixture();value['environments'][0][key].pop()
            with self.assertRaises(ValueError):probe.render_state_record(**value)
        value=self.fixture();value['observation'][0][3]=float('nan')
        with self.assertRaises(ValueError):probe.render_state_record(**value)

    def test_mixed_two_environment_reset_mapping_keeps_identity(self):
        a=self.fixture(True);b=self.fixture(False)
        b['environments'][0]['env_id']=1;b['transitions'][0]['env_id']=1
        b['transitions'][0]['physics_time_s']=a['physics_s']
        b['transitions'][0]['sequence']=a['step']
        for key in ['environments','transitions','reset_flags','observation']:a[key]+=b[key]
        result=probe.render_state_record(**a)
        self.assertEqual([r['reset_before_image'] for r in result['source_transitions']],[True,False])

    def test_camera_framing_contains_all_origin_scene_boxes(self):
        origins=[[-2.,0.,0.],[2.,0.,0.]];plan=probe.camera_plan(origins)
        lo,hi=plan['scene_bounds_world_m'];radius=math.dist(lo,hi)/2
        for view in plan['views'].values():
            distance=math.dist(view['position_world_m'],view['target_world_m'])
            vertical_half_angle=math.atan((view['horizontal_aperture_mm']/1.5/2)/view['focal_length_mm'])
            self.assertLess(math.asin(radius/distance),vertical_half_angle)
        moved=probe.camera_plan([[x+10,y-3,z+2] for x,y,z in origins])
        for actual,expected in zip([b-a for a,b in zip(plan['views']['front']['target_world_m'],moved['views']['front']['target_world_m'])],[10.,-3.,2.]):
            self.assertAlmostEqual(actual,expected)
        for value in [[],[[0.,float('nan'),0.]],[[0.,0.]],[[0.,0.,0.]]*9]:
            with self.assertRaises(ValueError):probe.camera_plan(value)

    def test_camera_receipts_reject_stale_mismatched_or_wrong_time_images(self):
        before={'front':10,'side':10}
        after={k:{'rendering_frame':11,'rendering_time':.1} for k in before}
        probe.validate_camera_receipts(before,after,.1)
        for field,value in [('rendering_frame',10),('rendering_frame',12),('rendering_time',.08),('rendering_time',float('nan'))]:
            broken=deepcopy(after);broken['side'][field]=value
            with self.assertRaises(ValueError):probe.validate_camera_receipts(before,broken,.1)
        with self.assertRaises(ValueError):probe.validate_camera_receipts(before,{'front':after['front']},.1)


if __name__=='__main__':unittest.main()
