"""Physical rack geometry and adversarial actual-contact acquisition evidence."""
from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from inspire_rack import RackConfig,rack_boxes,rack_command,parse_rack_config,acquisition_result,build_rack
from inspire_grasp import GraspConfig,write_failed_grasp_receipt


class RackTests(unittest.TestCase):
    def rows(self):
        result=[]
        for i in range(2000):
            command=rack_command(i*.005,RackConfig());supported=command['external_support_allowed']
            result.append({'sequence':i,'physics_s':.015+i*.005,'phase':command['phase'],
                'rack_object_contact':supported,'external_object_contact':supported,'nonrack_external_object_contact':False,
                'holder_hand_contact':i>=200,'holder_lift_world_m':.08 if i>=900 else 0.,
                'rack_hand_contact':False,'tip_drift_m':0.,'axis_drift_deg':0.})
        return result

    def test_exact_source_tested_boxes_include_corrected_cheek_height(self):
        cfg=RackConfig();center=(-.010,.03139599,.13946113-.04)
        boxes=rack_boxes(cfg,center)
        self.assertEqual(len(boxes),6)
        for box in boxes:
            self.assertAlmostEqual(box['center_m'][0],-.097 if box['name'].startswith('end_0') else .077)
            if box['name'].endswith('floor'):
                self.assertEqual(box['size_m'],[.006,.040,.006]);self.assertAlmostEqual(box['center_m'][2],.12426113-.04)
            else:
                self.assertEqual(box['size_m'],[.006,.004,.020]);self.assertAlmostEqual(box['center_m'][2],.13346113-.04)
                self.assertAlmostEqual(box['center_m'][1],.01439599 if box['name'].endswith('near') else .04839599)

    def test_timing_full2000steps_and_fixed80mm_bounded_lift(self):
        counts={}
        commands=[rack_command(i*.005,RackConfig()) for i in range(2000)]
        for c in commands:
            counts[c['phase']]=counts.get(c['phase'],0)+1
            self.assertTrue(-.04<=c['wrist'][2]<=.04)
        self.assertEqual(list(counts.values()),[100,200,200,400,100,1000])
        self.assertEqual(commands[0]['wrist'][2],-.04);self.assertEqual(commands[-1]['wrist'][2],.04)
        self.assertLess(max(abs(b['wrist'][2]-a['wrist'][2])/.005 for a,b in zip(commands,commands[1:])),.15)
        self.assertFalse(commands[900]['external_support_allowed']);self.assertTrue(commands[1000]['retention_window'])

    def test_new_nested_config_cannot_leak_into_old_preload_schema(self):
        nested={'grasp':{'thumb_yaw_initial_rad':.407435,'thumb_flexion_initial_rad':.20524},'rack':{}}
        grasp,rack=parse_rack_config(nested);self.assertEqual(rack.holder_length_m,.19);self.assertEqual(rack.tip_offset_m,.125)
        self.assertEqual(grasp.thumb_flexion_initial_rad,.20524)
        with self.assertRaises(ValueError):GraspConfig.from_dict(nested)
        for value in [{}, {'grasp':{}}, {'grasp':{},'rack':{},'duration_s':1.},
                      {'grasp':{'retention_tip_limit_m':.1},'rack':{}},
                      {'grasp':{'holder_orientation_palm_qwxyz':[1,0,0,0]},'rack':{}}]:
            with self.assertRaises(ValueError):parse_rack_config(value)

    def test_geometry_and_lift_config_fail_closed(self):
        for value in [{'saddle_axis_offsets_m':[-.2,.2]},{'holder_length_m':.09},{'tip_offset_m':.09},
                      {'initial_wrist_z_m':-.06},{'lifted_wrist_z_m':.06},{'initial_surface_gap_m':float('nan')},
                      {'support_width_axis_m':True},{'hold_seconds':1.}]:
            with self.assertRaises(ValueError):RackConfig.from_dict(value)

    def test_complete_measured_single_lift_hold_is_only_a_diagnostic(self):
        result=acquisition_result(self.rows())
        self.assertTrue(result['lift_hold_diagnostic_pass']);self.assertFalse(result['controlled_release_tested'])
        self.assertFalse(result['three_repeat_acquisition_qualified'])

    def test_commanded_lift_without_actual_lift_fails(self):
        rows=self.rows()
        for row in rows:row['holder_lift_world_m']=0.
        result=acquisition_result(rows);self.assertFalse(result['checks']['measured_lift50mm'])

    def test_rack_support_or_one_lost_hand_contact_invalidates_hold(self):
        for key,value in [('external_object_contact',True),('rack_object_contact',True),('holder_hand_contact',False),
                          ('holder_hand_contact',None),('rack_hand_contact',True),('tip_drift_m',.00301),('axis_drift_deg',3.01)]:
            rows=self.rows();rows[1100][key]=value
            self.assertFalse(acquisition_result(rows)['lift_hold_diagnostic_pass'],key)

    def test_clearance_is_measured_and_cannot_be_skipped(self):
        for key,value in [('external_object_contact',True),('holder_lift_world_m',.0499),('holder_hand_contact',False)]:
            rows=self.rows();rows[950][key]=value
            self.assertFalse(acquisition_result(rows)['lift_hold_diagnostic_pass'],key)

    def test_initial_support_required_and_unexpected_support_never_allowed(self):
        rows=self.rows()
        for row in rows:row['rack_object_contact']=False
        self.assertFalse(acquisition_result(rows)['checks']['rack_initially_supported_marker'])
        rows=self.rows();rows[100]['nonrack_external_object_contact']=True
        self.assertFalse(acquisition_result(rows)['lift_hold_diagnostic_pass'])

    def test_partial_duplicate_nonfinite_or_missing_measurements_fail(self):
        self.assertFalse(acquisition_result(self.rows()[:-1])['lift_hold_diagnostic_pass'])
        self.assertFalse(acquisition_result([])['lift_hold_diagnostic_pass'])
        for key,value in [('sequence',1099),('physics_s',float('nan')),('holder_lift_world_m',float('inf')),('phase','rack_settle')]:
            rows=self.rows();rows[1100][key]=value
            self.assertFalse(acquisition_result(rows)['lift_hold_diagnostic_pass'],key)
        rows=self.rows();del rows[1100]['external_object_contact']
        self.assertFalse(acquisition_result(rows)['lift_hold_diagnostic_pass'])

    def test_rack_failure_preserves_failed_receipt_and_scope(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'rack_scene.json').write_text('{}')
            write_failed_grasp_receipt(p,error='lost grip',traceback_text='',mode='rack-lift-hold-one',
                scope={'holder_fixture_support_active':True},phase='rack_close',steps=100,observation={},gates={})
            receipt=json.loads((p/'probe.json').read_text());metrics=json.loads((p/'metrics.json').read_text())
            self.assertEqual(receipt['status'],'FAIL');self.assertIn('rack_scene.json',receipt['artifacts'])
            self.assertIn('rack',receipt['scope']);self.assertFalse(metrics['single_rack_lift_hold_diagnostic_pass'])

    def test_real_usd_rack_has_six_static_colliders_no_attachment_or_filters(self):
        try:from pxr import Usd,UsdPhysics,UsdGeom
        except ImportError:self.skipTest('Installed USD bindings required')
        stage=Usd.Stage.CreateInMemory();facts=build_rack(stage,RackConfig(),(-.01,.03139599,.09946113))
        prims=list(stage.Traverse());self.assertEqual(sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),6)
        self.assertFalse(any(p.HasAPI(UsdPhysics.RigidBodyAPI) or p.IsA(UsdPhysics.Joint) for p in prims))
        self.assertFalse(any('filteredPairs' in r.GetName() for p in prims for r in p.GetRelationships()))
        self.assertFalse(facts['marker_attachment']);self.assertEqual(len(facts['boxes']),6)
        with self.assertRaises(ValueError):build_rack(stage,RackConfig(),(-.01,.03139599,.09946113))

    def test_extended_marker_stays88g_with_one_internal_slider(self):
        try:from pxr import Usd,UsdPhysics
        except ImportError:self.skipTest('Installed USD bindings required')
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'isaac'))
        from twin.inspire.whiteboard_scene import SceneConfig,HolderParameters,build_scene
        stage=Usd.Stage.CreateInMemory()
        config=SceneConfig(holder=HolderParameters(body_length_m=.19,tip_offset_m=.125))
        facts=build_scene(stage,config)
        self.assertAlmostEqual(facts['total_free_object_mass_kg'],.088)
        joints=[p for p in stage.Traverse() if str(p.GetPath()).startswith('/World/Marker/') and p.IsA(UsdPhysics.Joint)]
        self.assertEqual(len(joints),1);self.assertTrue(joints[0].IsA(UsdPhysics.PrismaticJoint))
        self.assertFalse(facts['scope']['attachment_active']);self.assertFalse(facts['scope']['fixture_support_active'])


if __name__=='__main__':unittest.main()


class ThreeCycleSequenceTests(unittest.TestCase):
    def rows(self, config, mutate=None):
        from inspire_rack import cycle_command, CYCLE_STEPS
        rows = []
        for i in range(CYCLE_STEPS):
            c = cycle_command(i * .005, config)
            held = c['phase'] in ('cycle_clearance_settle', 'cycle_hold')
            free = c['phase'] in ('cycle_release_settle', 'cycle_retreat', 'cycle_withdraw', 'cycle_withdrawn_settle', 'cycle_approach', 'cycle_advance', 'cycle_settle')
            lift = .08 if c['phase'] in ('cycle_clearance_settle', 'cycle_hold') else 0.
            rows.append({'sequence': i, 'physics_s': .005 * (i + 1), 'phase': c['phase'], 'cycle': c['cycle'],
                         'rack_object_contact': not held, 'external_object_contact': not held, 'nonrack_external_object_contact': False,
                         'holder_hand_contact': not free, 'rack_hand_contact': False, 'holder_lift_world_m': lift,
                         'tip_drift_m': .0005, 'axis_drift_deg': .2})
        if mutate:
            mutate(rows)
        return rows

    def test_declared_sequence_timing_phases_and_wrist(self):
        from inspire_rack import cycle_command, RackConfig, CYCLE_SECONDS, CYCLE_STEPS, CYCLES
        c = RackConfig()
        self.assertEqual((CYCLES, CYCLE_STEPS), (3, 12300)); self.assertAlmostEqual(CYCLE_SECONDS, 20.5)
        self.assertEqual(cycle_command(0., c)['phase'], 'cycle_settle')
        hold = cycle_command(6., c); self.assertEqual((hold['phase'], hold['cycle'], hold['wrist'][2], hold['closing_fraction'], hold['opening_fraction']), ('cycle_hold', 0, .04, 1., 0.))
        self.assertFalse(hold['external_support_allowed']); self.assertTrue(hold['retention_window'])
        withdrawn = cycle_command(17.2, c); self.assertEqual((withdrawn['phase'], withdrawn['opening_fraction'], withdrawn['closing_fraction'], withdrawn['wrist'][1], withdrawn['wrist'][2]), ('cycle_withdrawn_settle', 1., 0., -.05, .04))
        retreat = cycle_command(14.4, c); self.assertEqual(retreat['phase'], 'cycle_retreat'); self.assertLess(retreat['wrist'][1], 0.)
        nxt = cycle_command(20.6, c); self.assertEqual((nxt['phase'], nxt['cycle'], nxt['wrist'][1], nxt['wrist'][2]), ('cycle_settle', 1, 0., -.04))
        last = cycle_command(61.4, c); self.assertEqual((last['phase'], last['cycle']), ('cycle_advance', 2))
        with self.assertRaises(ValueError): cycle_command(-1., c)
        with self.assertRaises(ValueError): RackConfig.from_dict({'release_opening_rad': .05})

    def test_three_good_cycles_qualify_and_any_missing_or_broken_cycle_does_not(self):
        from inspire_rack import cycle_result, RackConfig
        c = RackConfig()
        good = cycle_result(self.rows(c), c)
        self.assertTrue(good['three_repeat_acquisition_qualified']); self.assertEqual(good['cycles_completed'], 3)
        self.assertTrue(good['controlled_release_tested'])
        partial = cycle_result(self.rows(c)[:8200], c)
        self.assertFalse(partial['three_repeat_acquisition_qualified']); self.assertEqual(partial['cycles'][2]['status'], 'NOT_RUN')
        def hand_kept_marker(rows):
            for r in rows:
                if r['cycle'] == 1 and r['phase'] == 'cycle_withdrawn_settle': r['holder_hand_contact'] = True
        self.assertFalse(cycle_result(self.rows(c, hand_kept_marker), c)['cycles'][1]['checks']['controlled_release_hand_free'])
        def marker_lifted_after_release(rows):
            for r in rows:
                if r['cycle'] == 2 and r['phase'] == 'cycle_withdraw': r['holder_lift_world_m'] = .03; r['rack_object_contact'] = False; r['external_object_contact'] = False
        self.assertFalse(cycle_result(self.rows(c, marker_lifted_after_release), c)['cycles'][2]['checks']['marker_stays_racked_after_release'])
        def short_lift(rows):
            for r in rows:
                if r['cycle'] == 0 and r['phase'] == 'cycle_hold': r['holder_lift_world_m'] = .04
        self.assertFalse(cycle_result(self.rows(c, short_lift), c)['cycles'][0]['checks']['measured_lift50mm'])
        def rack_support_during_hold(rows):
            rows[1300]['external_object_contact'] = True; rows[1300]['rack_object_contact'] = True
        self.assertFalse(cycle_result(self.rows(c, rack_support_during_hold), c)['cycles'][0]['checks']['no_external_hold_support'])
