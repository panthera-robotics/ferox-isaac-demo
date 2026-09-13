"""Measured retention rejects missing, attached, short, discontinuous and drifting traces."""
from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from inspire_grasp import GraspConfig,drift,relative_measurement,retention_result,wrist_target


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


if __name__=='__main__':unittest.main()
