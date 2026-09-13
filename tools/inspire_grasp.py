"""Declared preloaded grasp candidate and measured relative-retention checks.

The candidate was seeded by source-URDF FK and a deterministic CPU search of
source mesh vertex/cylinder distances. Vertex clearance is not a collision proof.
The actual PhysX contact/retention run must accept or reject the hypothesis.
"""
from dataclasses import asdict,dataclass,fields
import hashlib
import json
import math


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
            value=getattr(self,f.name)
            if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('Invalid grasp scalar')
        if not (0<=self.four_finger_initial_rad<=1.4381 and 0<=self.thumb_yaw_initial_rad<=1.1641
                and 0<=self.thumb_flexion_initial_rad<=.5864 and 0<=self.finger_closing_increment_rad<=.20
                and 0<=self.thumb_closing_increment_rad<=.10 and .1<=self.finger_stiffness_nm_rad<=2.
                and .01<=self.finger_damping_nm_s_rad<=.2
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
