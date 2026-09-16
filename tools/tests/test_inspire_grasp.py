"""Measured retention rejects missing, attached, short, discontinuous and drifting traces."""
from copy import deepcopy
import math
from pathlib import Path
import sys
import json
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from inspire_grasp import (GraspConfig,drift,relative_measurement,retention_result,wrist_target,
                           diagnostic_json,capture_readbacks,initialization_report,write_failed_grasp_receipt)


class RetentionTests(unittest.TestCase):
    def rows(self):
        return [{'sequence':i,'physics_s':2.+i*.005,'tip_drift_m':0.,'axis_drift_deg':0.,
                 'holder_hand_contact':True,'external_object_contact':False} for i in range(12000)]

    def test_full_60_seconds_of_actual_unattached_contact_can_pass_supported_hand_scope(self):
        r=retention_result(self.rows(),expected_seconds=60.,dt_s=.005)
        self.assertTrue(r['accepted']);self.assertFalse(r['rack_acquisition_qualified']);self.assertFalse(r['standing_qualified'])

    def test_perfect_but_short_or_empty_trace_fails(self):
        for rows in [[],self.rows()[:-1]]:self.assertFalse(retention_result(rows,expected_seconds=60.,dt_s=.005)['accepted'])

    def test_attachment_or_holder_support_fails_even_perfect_trace(self):
        for flag in ['attachment_active','fixture_support_active']:
            self.assertFalse(retention_result(self.rows(),expected_seconds=60.,dt_s=.005,**{flag:True})['accepted'])

    def test_missing_contact_or_external_support_or_one_threshold_breach_fails(self):
        for key,value in [('holder_hand_contact',False),('holder_hand_contact',None),('external_object_contact',True),
                          ('tip_drift_m',.003001),('axis_drift_deg',3.00001),('tip_drift_m',float('nan'))]:
            rows=self.rows();rows[200][key]=value
            self.assertFalse(retention_result(rows,expected_seconds=60.,dt_s=.005)['accepted'],key)
        rows=self.rows();del rows[200]['holder_hand_contact']
        self.assertFalse(retention_result(rows,expected_seconds=60.,dt_s=.005)['accepted'])

    def test_repeated_physics_or_clock_gap_fails(self):
        for key,value in [('sequence',199),('physics_s',2.995)]:
            rows=self.rows();rows[200][key]=value
            self.assertFalse(retention_result(rows,expected_seconds=60.,dt_s=.005)['accepted'])

    def test_config_cannot_relax_declared_thresholds_or_claim_hardware(self):
        for value in [{'retention_tip_limit_m':.010},{'retention_axis_limit_deg':10.},
                      {'holder_orientation_palm_qwxyz':[1,1,0,0]},{'hardware_authorized':True},
                      {'contact_static_friction':20},{'four_finger_initial_rad':float('inf')}]:
            with self.assertRaises(ValueError):GraspConfig.from_dict(value)

    def test_wrist_target_is_continuous_bounded_six_axis_physical_reference(self):
        self.assertEqual(wrist_target(0),[0.]*6)
        for i in range(1200):
            q=wrist_target(i*.05);self.assertEqual(len(q),6)
            self.assertTrue(all(abs(v)<=.005 for v in q[:3]));self.assertTrue(all(abs(v)<=math.radians(5) for v in q[3:]))

    def test_relative_measurement_cancels_actual_world_wrist_motion(self):
        try:import numpy
        except ImportError:self.skipTest('Installed image NumPy required')
        palm=[0,0,0,0,0,0,1];holder=[.02,.03,.14,0,0,0,1];nib=[.02,.03,.06,0,0,0,1]
        initial=relative_measurement(palm,holder,nib,.0015)
        # Rotate the entire physical scene90degrees aboutZ and translate it.
        def moved(p):return [2-p[1],3+p[0],4+p[2],0,0,math.sqrt(.5),math.sqrt(.5)]
        result=relative_measurement(moved(palm),moved(holder),moved(nib),.0015)
        error=drift(initial,result);self.assertLess(error['tip_drift_m'],1e-12);self.assertLess(error['axis_drift_deg'],1e-6)

    def test_actual_tip_and_axis_slip_are_independent(self):
        reference={'tip_palm_m':[0,0,0],'holder_axis_palm':[1,0,0]}
        r=deepcopy(reference);r['tip_palm_m'][0]=.004
        self.assertAlmostEqual(drift(reference,r)['tip_drift_m'],.004)
        angle=math.radians(4);r=deepcopy(reference);r['holder_axis_palm']=[math.cos(angle),math.sin(angle),0]
        self.assertAlmostEqual(drift(reference,r)['axis_drift_deg'],4.)


class FailureEvidenceTests(unittest.TestCase):
    def poses(self):return {n:[0.,0.,0.,0.,0.,0.,1.] for n in ['palm','holder','nib']}

    def test_initialization_gate_stays_strict_at_point_zero_one(self):
        self.assertTrue(initialization_report({'finger':1.},['finger'],[1.009],[0.],self.poses(),[])['admitted'])
        self.assertFalse(initialization_report({'finger':1.},['finger'],[1.01],[0.],self.poses(),[])['admitted'])

    def test_nonfinite_joint_and_pose_readback_cannot_admit(self):
        for field in ['q','dq','pose']:
            q=[1.];dq=[0.];poses=self.poses()
            if field=='q':q=[float('nan')]
            if field=='dq':dq=[float('inf')]
            if field=='pose':poses['holder'][2]=float('nan')
            self.assertFalse(initialization_report({'finger':1.},['finger'],q,dq,poses,[])['admitted'])

    def test_initial_collision_is_reported_without_relaxing_joint_gate(self):
        contact={'actor0':'/World/Hand/base','actor1':'/World/Hand/thumb','impulse_ns':[1.,0.,0.],'separation_m':-.015}
        report=initialization_report({'finger':1.},['finger'],[1.],[0.],self.poses(),[contact])
        self.assertFalse(report['admitted']);self.assertTrue(report['checks']['initial_preload_realized'])
        self.assertFalse(report['checks']['initial_no_deep_self_penetration'])

    def test_one_failed_getter_preserves_other_readbacks(self):
        def broken():raise RuntimeError('physics view lost')
        state=capture_readbacks({'q':lambda:[.2],'dq':broken,'poses':self.poses})
        self.assertEqual(state['q'],[.2]);self.assertIn('read_error',state['dq']);self.assertEqual(state['poses'],self.poses())

    def test_nan_failure_receipt_keeps_candidate_gates_state_and_all_qualification_false(self):
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder);out.joinpath('collision_candidate.json').write_text('{"candidate_id":"provisional"}')
            observation={'q':[float('nan')],'dq':[.3],'poses_xyzw':self.poses()}
            gates=initialization_report({'finger':1.},['finger'],observation['q'],observation['dq'],observation['poses_xyzw'],[])
            write_failed_grasp_receipt(out,error='initial rejected',traceback_text='trace',mode='preloaded-free-control',scope={},
                                      phase='initialization',steps=0,observation=observation,gates=gates)
            state=json.loads(out.joinpath('last_runtime_state.json').read_text());self.assertEqual(state['q'][0],{'invalid_numeric':'NaN'})
            self.assertEqual(state['dq'],[.3]);self.assertIn('$.q[0]',state['invalid_numeric_paths'])
            metrics=json.loads(out.joinpath('metrics.json').read_text());self.assertFalse(metrics['empty_hand_preload_control_pass'])
            self.assertFalse(metrics['checks']['execution_completed']);self.assertEqual(metrics['steps'],0)
            receipt=json.loads(out.joinpath('probe.json').read_text());self.assertEqual(receipt['status'],'FAIL')
            self.assertIn('collision_candidate.json',receipt['artifacts']);self.assertIn('metrics.json',receipt['artifacts'])
            write_failed_grasp_receipt(out,error='cleanup failed',traceback_text='',mode='preloaded-free-control',scope={},
                                      phase='cleanup',steps=0,observation=observation,gates=gates)
            self.assertEqual(json.loads(out.joinpath('failure.json').read_text())['previous_failure']['error'],'initial rejected')


if __name__=='__main__':unittest.main()


class PreloadSupportConfigTests(unittest.TestCase):
    def test_declared_preload_support_is_bounded_and_hashed(self):
        from inspire_grasp import GraspConfig
        base = GraspConfig()
        self.assertEqual(base.preload_support_s, 0.)
        supported = GraspConfig.from_dict({'preload_support_s': 1.5})
        self.assertNotEqual(supported.sha256, base.sha256)
        for bad in (-.1, 1.6, float('nan'), '1'):
            with self.assertRaises(ValueError):
                GraspConfig.from_dict({'preload_support_s': bad})

    def test_retention_with_any_supported_sample_cannot_pass(self):
        from inspire_grasp import retention_result
        rows = [{'sequence': i, 'physics_s': 2. + i * .005, 'tip_drift_m': .001, 'axis_drift_deg': .5,
                 'holder_hand_contact': True, 'external_object_contact': False} for i in range(800)]
        self.assertTrue(retention_result(rows, expected_seconds=4., dt_s=.005)['accepted'])
        self.assertFalse(retention_result(rows, expected_seconds=4., dt_s=.005, fixture_support_active=True)['accepted'])

    def test_solver_iterations_are_declared_integers_and_hashed(self):
        from inspire_grasp import GraspConfig
        base = GraspConfig()
        self.assertEqual((base.solver_position_iterations, base.solver_velocity_iterations), (32, 8))
        changed = GraspConfig.from_dict({'solver_velocity_iterations': 32})
        self.assertNotEqual(changed.sha256, base.sha256)
        for bad in (0, 256, 8.0, '8'):
            with self.assertRaises(ValueError):
                GraspConfig.from_dict({'solver_velocity_iterations': bad})

    def test_per_finger_initial_angles_are_optional_bounded_and_ordered(self):
        from inspire_grasp import GraspConfig
        base = GraspConfig()
        self.assertEqual(len(set(base.initial_targets()[k] for k in base.initial_targets() if 'thumb' not in k)), 1)
        per = GraspConfig.from_dict({'finger_initial_rad': [1.30, 1.25, 1.22, 1.20]})
        t = per.initial_targets()
        self.assertEqual([t['right_index_1_joint'], t['right_middle_1_joint'], t['right_ring_1_joint'], t['right_little_1_joint']], [1.30, 1.25, 1.22, 1.20])
        self.assertNotEqual(per.sha256, base.sha256)
        for bad in ([1.3, 1.2], [1.5, 1.2, 1.2, 1.2], [1.2, float('nan'), 1.2, 1.2], 'x'):
            with self.assertRaises(ValueError):
                GraspConfig.from_dict({'finger_initial_rad': bad})
