"""Declared preloaded grasp candidate and measured relative-retention checks.

The candidate was seeded by source-URDF FK and a deterministic CPU search of
source mesh vertex/cylinder distances. Vertex clearance is not a collision proof.
The actual PhysX contact/retention run must accept or reject the hypothesis.
"""
from dataclasses import asdict,dataclass,fields
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class GraspConfig:
    schema_version: int = 1
    palm_candidate_id: str = 'ftp_palm_yz_slabs_v2'
    holder_center_palm_m: tuple = (-.010,.03139599,.13946113)
    # local holder +Z -> palm +X; nib extends toward palm -X.
    holder_orientation_palm_qwxyz: tuple = (math.sqrt(.5),0.,math.sqrt(.5),0.)
    four_finger_initial_rad: float = 1.1563156754590562
    thumb_yaw_initial_rad: float = 1.0900916230524766
    thumb_flexion_initial_rad: float = .4752747391137899
    finger_closing_increment_rad: float = .12
    thumb_closing_increment_rad: float = .05
    finger_stiffness_nm_rad: float = 1.
    finger_damping_nm_s_rad: float = .05
    contact_static_friction: float = .7
    contact_dynamic_friction: float = .6
    retention_tip_limit_m: float = .003
    retention_axis_limit_deg: float = 3.
    # Declared temporary world support of the free holder while the fingers
    # close (a hand-over, not a weld); it must end before the retention window.
    preload_support_s: float = 0.
    # Declared articulation solver iterations (hashed with the candidate). 32/8 is the
    # retained bench control; the moving-wrist bench needed 32 velocity iterations.
    solver_position_iterations: int = 32
    solver_velocity_iterations: int = 8

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version!=1:raise ValueError('Unknown grasp schema')
        if self.palm_candidate_id!='ftp_palm_yz_slabs_v2':raise ValueError('This candidate uses the provisional right v2 palm')
        for name in ['holder_center_palm_m','holder_orientation_palm_qwxyz']:
            value=getattr(self,name)
            if not isinstance(value,(tuple,list)) or len(value)!=(3 if 'center' in name else 4):raise ValueError('Invalid pose shape')
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in value):raise ValueError('Invalid pose number')
            object.__setattr__(self,name,tuple(value))
        if not math.isclose(sum(v*v for v in self.holder_orientation_palm_qwxyz),1.,abs_tol=1e-8):raise ValueError('Nonunit quaternion')
        for f in fields(self):
            if f.name in {'schema_version','palm_candidate_id','holder_center_palm_m','holder_orientation_palm_qwxyz'}:continue
            if f.name in {'solver_position_iterations','solver_velocity_iterations'}:
                if type(getattr(self,f.name)) is not int or not 1<=getattr(self,f.name)<=255:raise ValueError('Solver iterations must be integers in 1..255')
                continue
            value=getattr(self,f.name)
            if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('Invalid grasp scalar')
        if not (0<=self.four_finger_initial_rad<=1.4381 and 0<=self.thumb_yaw_initial_rad<=1.1641
                and 0<=self.thumb_flexion_initial_rad<=.5864 and 0<=self.finger_closing_increment_rad<=.20
                and 0<=self.thumb_closing_increment_rad<=.10 and .1<=self.finger_stiffness_nm_rad<=2.
                and .01<=self.finger_damping_nm_s_rad<=.2 and 0<=self.preload_support_s<=1.5
                and 0<=self.contact_dynamic_friction<=self.contact_static_friction<=1.):raise ValueError('Grasp parameters outside declared bounds')
        if self.retention_tip_limit_m!=.003 or self.retention_axis_limit_deg!=3.:raise ValueError('Acceptance limits cannot be relaxed through configuration')

    @classmethod
    def from_dict(cls,data):
        if not isinstance(data,dict) or set(data)-{f.name for f in fields(cls)}:raise ValueError('Unknown grasp config key')
        return cls(**data)

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

    def initial_targets(self):
        result={'right_'+f+'_1_joint':self.four_finger_initial_rad for f in ['index','middle','ring','little']}
        result.update(right_thumb_1_joint=self.thumb_yaw_initial_rad,right_thumb_2_joint=self.thumb_flexion_initial_rad)
        return result

    def closed_targets(self,limits):
        return {n:min(limits[n][1],q+(self.thumb_closing_increment_rad if 'thumb' in n else self.finger_closing_increment_rad))
                for n,q in self.initial_targets().items()}


def relative_measurement(palm_pose_xyzw,holder_pose_xyzw,nib_pose_xyzw,nib_radius_m):
    """Actual tip and holder axis expressed in measured palm frame; no targets."""
    import numpy as np
    def matrix(pose):
        p=np.asarray(pose,dtype=float)
        if p.shape!=(7,) or not np.isfinite(p).all():raise ValueError('Invalid measured rigid-body pose')
        x,y,z,w=p[3:];norm=x*x+y*y+z*z+w*w
        if not math.isclose(norm,1.,abs_tol=1e-4):raise ValueError('Nonunit measured pose quaternion')
        x,y,z,w=p[3:]/math.sqrt(norm)
        r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        return p[:3],r
    pp,pr=matrix(palm_pose_xyzw);hp,hr=matrix(holder_pose_xyzw);np_,nr=matrix(nib_pose_xyzw)
    tip=np_-nr[:,2]*nib_radius_m
    return {'tip_palm_m':(pr.T@(tip-pp)).tolist(),'holder_center_palm_m':(pr.T@(hp-pp)).tolist(),
            'holder_axis_palm':(pr.T@(-hr[:,2])).tolist(),'tip_world_m':tip.tolist()}


def drift(reference,measurement):
    points=[reference['tip_palm_m'],measurement['tip_palm_m']]
    axes=[reference['holder_axis_palm'],measurement['holder_axis_palm']]
    for vector in points+axes:
        if len(vector)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in vector):raise ValueError('Invalid retention observation')
    for axis in axes:
        if not math.isclose(sum(v*v for v in axis),1.,abs_tol=1e-4):raise ValueError('Invalid measured axis')
    dot=sum(a*b for a,b in zip(*axes))/math.sqrt(sum(v*v for v in axes[0])*sum(v*v for v in axes[1]))
    angle=math.degrees(math.acos(max(-1.,min(1.,dot))))
    return {'tip_drift_m':math.dist(*points),'axis_drift_deg':angle}


def wrist_target(elapsed_s):
    """Six physical fixture drives, modest deterministic 5 mm / 5 degree motion."""
    if not math.isfinite(elapsed_s) or elapsed_s<0:raise ValueError('Invalid fixture time')
    envelope=min(1.,elapsed_s/3.)
    return [envelope*.005*math.sin(2*math.pi*elapsed_s/period) for period in [9.,11.,13.]]+[
        envelope*math.radians(5)*math.sin(2*math.pi*elapsed_s/period) for period in [15.,17.,19.]]


def retention_result(rows,*,expected_seconds,dt_s,fixture_support_active=False,attachment_active=False):
    """An uninterrupted measured window is required; partial traces cannot pass."""
    if expected_seconds<=0 or dt_s<=0:raise ValueError('Invalid retention duration')
    checks={'full_duration':len(rows)>=round(expected_seconds/dt_s),'continuous_samples':bool(rows),
            'tool_unattached':not attachment_active and not fixture_support_active,'measured_contacts':bool(rows),
            'tip_drift_3mm':bool(rows),'axis_drift_3deg':bool(rows),'no_external_object_support':bool(rows)}
    for i,row in enumerate(rows):
        try:
            values=[row['tip_drift_m'],row['axis_drift_deg'],row['physics_s']]
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):raise ValueError('Nonfinite measurement')
            if type(row['sequence']) is not int:raise ValueError('Invalid sequence')
            checks['tip_drift_3mm'] &= 0<=row['tip_drift_m']<=.003
            checks['axis_drift_3deg'] &= 0<=row['axis_drift_deg']<=3.
            checks['measured_contacts'] &= row['holder_hand_contact'] is True
            checks['no_external_object_support'] &= row['external_object_contact'] is False
            if i:
                checks['continuous_samples'] &= row['sequence']==rows[i-1]['sequence']+1 and math.isclose(row['physics_s']-rows[i-1]['physics_s'],dt_s,rel_tol=1e-5,abs_tol=1e-8)
        except (KeyError,TypeError,ValueError):
            checks['continuous_samples']=False
    return {'accepted':all(checks.values()),'checks':checks,'measured_samples':len(rows),'required_seconds':expected_seconds,
            'rack_acquisition_qualified':False,'exact_E2_qualified':False,'standing_qualified':False}


def diagnostic_json(value):
    """Preserve invalid numeric observations explicitly in strict JSON; never zero-fill."""
    invalid=[]
    def convert(item,path):
        if isinstance(item,dict):return {str(k):convert(v,path+'.'+str(k)) for k,v in item.items()}
        if isinstance(item,(list,tuple)):return [convert(v,path+f'[{i}]') for i,v in enumerate(item)]
        if isinstance(item,float) and not math.isfinite(item):
            invalid.append(path)
            return {'invalid_numeric':'NaN' if math.isnan(item) else '+Infinity' if item>0 else '-Infinity'}
        if item is None or isinstance(item,(str,bool,int,float)):return item
        if hasattr(item,'tolist'):return convert(item.tolist(),path)
        raise TypeError('Unsupported diagnostic value at '+path+': '+type(item).__name__)
    return convert(value,'$'),invalid


def capture_readbacks(readers):
    """A failed getter cannot discard the other available physical observations."""
    result={}
    for name,getter in readers.items():
        try:result[name]=getter()
        except Exception as error:result[name]={'read_error':type(error).__name__+': '+str(error)}
    return result


def initialization_report(expected,names,q,dq,poses,contacts):
    """Evaluate the unchanged 0.01 rad preload gate before admitting command motion."""
    def finite_vector(v,size):
        return isinstance(v,(list,tuple)) and len(v)==size and all(type(x) in (int,float) and math.isfinite(x) for x in v)
    named=isinstance(names,(list,tuple)) and all(isinstance(n,str) for n in names) and len(set(names))==len(names) and set(expected)<=set(names)
    numeric=named and finite_vector(q,len(names)) and finite_vector(dq,len(names))
    error=max(abs(q[names.index(n)]-target) for n,target in expected.items()) if numeric else None
    pose_ok=isinstance(poses,dict) and set(poses)=={'palm','holder','nib'}
    if pose_ok:pose_ok=all(finite_vector(p,7) and math.isclose(sum(x*x for x in p[3:]),1.,abs_tol=1e-4) for p in poses.values())
    _,bad=diagnostic_json(contacts)
    self_depth=0.;object_depth=0.
    for contact in contacts:
        if bad:break
        actors=[contact['actor0'],contact['actor1']]
        if any(a.startswith('/World/Marker/') for a in actors):object_depth=min(object_depth,contact['separation_m'])
        if all(a.startswith('/World/Hand/') for a in actors) and math.sqrt(sum(v*v for v in contact['impulse_ns']))>1e-10:
            self_depth=min(self_depth,contact['separation_m'])
    checks={'named_finite_joint_state':bool(numeric),'finite_measured_body_poses':bool(pose_ok),
            'finite_contact_observations':not bad,'initial_preload_realized':error is not None and error<.01,
            'initial_no_deep_self_penetration':not bad and self_depth>=-.0005,
            'initial_object_no_deep_penetration':not bad and object_depth>=-.0005}
    return {'admitted':all(checks.values()),'checks':checks,'initial_joint_error_rad':error,
            'preload_error_limit_rad':.01,'initial_self_min_separation_m':self_depth,
            'initial_object_min_separation_m':object_depth,'initial_contact_points':len(contacts)}


def write_failed_grasp_receipt(out,*,error,traceback_text,mode,scope,phase,steps,observation,gates):
    """Persist a complete FAIL receipt even when the rejected observation contains NaN."""
    out=Path(out)
    def write(name,data):
        safe,bad=diagnostic_json(data)
        if isinstance(safe,dict):safe['invalid_numeric_paths']=bad
        tmp=out/(name+'.tmp');tmp.write_text(json.dumps(safe,indent=2,allow_nan=False)+'\n');tmp.replace(out/name)
    previous=json.loads((out/'failure.json').read_text()) if (out/'failure.json').is_file() else None
    write('last_runtime_state.json',observation)
    write('failure.json',{'error':error,'traceback':traceback_text,'phase':phase,'scope':scope,'previous_failure':previous})
    write('metrics.json',{'schema_version':1,'checks':{'execution_completed':False,**gates.get('checks',{})},
          'steps':steps,'mode':mode,'scope':scope,'abort_phase':phase,'abort_reason':error,'initialization':gates,
          'supported_preloaded_retention_60s_qualified':False,'static_preloaded_diagnostic_pass':False,
          'empty_hand_preload_control_pass':False,'pickup_qualification':'NOT_RUN','writing_qualification':'NOT_RUN',
          'standing_qualification':'NOT_RUN','candidate_qualification':'posture_limited; failed initialization is not a grasp result'})
    names=['grasp_config.json','failure.json','metrics.json','last_runtime_state.json','preload_geometry.json','marker_scene.json',
           'collision_candidate.json','scene_before_reset.usda','initial_state.json','initialization_gates.json',
           'state.jsonl','contacts.jsonl','frames.jsonl','self_contact_summary.json']
    artifacts=[n for n in names if (out/n).is_file() and (out/n).stat().st_size]
    artifacts += [str(p.relative_to(out)) for p in sorted((out/'frames').rglob('*.png'))]
    if mode=='rack-lift-hold-one':
        metrics=json.loads((out/'metrics.json').read_text())
        metrics.update(single_rack_lift_hold_diagnostic_pass=False,three_repeat_acquisition_qualified=False,controlled_release_tested=False)
        write('metrics.json',metrics)
        artifacts += [n for n in ['rack_config.json','rack_scene.json'] if (out/n).is_file()]
    write('probe.json',{'status':'FAIL','scope':'provisional_supported_hand_single_physical_rack_lift_hold'
                       if mode=='rack-lift-hold-one' else 'provisional_supported_preloaded_hand_only','artifacts':artifacts})
