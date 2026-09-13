"""Physical marker rack and bounded acquisition sequence, never an attachment."""
from dataclasses import asdict, dataclass, fields
import hashlib
import json
import math


@dataclass(frozen=True)
class RackConfig:
    schema_version: int = 1
    holder_length_m: float = .19
    tip_offset_m: float = .125
    saddle_axis_offsets_m: tuple = (-.087,.087)
    support_width_axis_m: float = .006
    support_width_transverse_m: float = .040
    support_thickness_m: float = .006
    cheek_thickness_m: float = .004
    cheek_center_offset_m: float = .017
    cheek_height_m: float = .020
    initial_surface_gap_m: float = .0002
    initial_wrist_z_m: float = -.04
    lifted_wrist_z_m: float = .04

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version!=1:raise ValueError('Unknown rack schema')
        for f in fields(self):
            if f.name in {'schema_version','saddle_axis_offsets_m'}:continue
            value=getattr(self,f.name)
            if type(value) not in (int,float) or not math.isfinite(value):raise ValueError('Invalid rack scalar')
        offsets=self.saddle_axis_offsets_m
        if not isinstance(offsets,(list,tuple)) or len(offsets)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in offsets):raise ValueError('Invalid saddle offsets')
        object.__setattr__(self,'saddle_axis_offsets_m',tuple(offsets))
        if not .1<=self.holder_length_m<=.2 or not self.holder_length_m/2+.0015<self.tip_offset_m<=.2:raise ValueError('Invalid rack marker dimensions')
        if not offsets[0]<0<offsets[1] or any(abs(x)+self.support_width_axis_m/2>self.holder_length_m/2 for x in offsets):raise ValueError('Saddles must support barrel endpoints')
        for name in ['support_width_axis_m','support_width_transverse_m','support_thickness_m','cheek_thickness_m','cheek_center_offset_m','cheek_height_m']:
            if not .001<=getattr(self,name)<=.08:raise ValueError('Invalid physical rack size')
        if not 0<=self.initial_surface_gap_m<=.001:raise ValueError('Invalid rack clearance')
        if self.initial_wrist_z_m!=-.04 or self.lifted_wrist_z_m!=.04:raise ValueError('Fixed80mm diagnostic lift inside original50mm fixture bounds')

    @classmethod
    def from_dict(cls,data):
        if not isinstance(data,dict) or set(data)-{f.name for f in fields(cls)}:raise ValueError('Unknown rack field')
        return cls(**data)

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True,separators=(',',':')).encode()).hexdigest()


def rack_boxes(config,holder_center_world_m,radius_m=.012):
    """Axis aligned saddles for the declared horizontal palm-X marker axis."""
    if len(holder_center_world_m)!=3 or any(type(v) not in (int,float) or not math.isfinite(v) for v in holder_center_world_m):raise ValueError('Invalid measured rack origin')
    if type(radius_m) not in (int,float) or not math.isfinite(radius_m) or radius_m<=0:raise ValueError('Invalid barrel radius')
    x,y,z=holder_center_world_m
    top=z-radius_m-config.initial_surface_gap_m
    result=[]
    for i,offset in enumerate(config.saddle_axis_offsets_m):
        center_x=x+offset
        result.append({'name':f'end_{i}/floor','center_m':[center_x,y,top-config.support_thickness_m/2],
            'size_m':[config.support_width_axis_m,config.support_width_transverse_m,config.support_thickness_m]})
        for sign,label in [(-1,'near'),(1,'far')]:
            # The source-tested cheeks are centered 6mm BELOW barrel center;
            # top+halfheight raises them into a rejected thumb posture.
            result.append({'name':f'end_{i}/{label}','center_m':[center_x,y+sign*config.cheek_center_offset_m,z-.006],
                'size_m':[config.support_width_axis_m,config.cheek_thickness_m,config.cheek_height_m]})
    return result


def parse_rack_config(value):
    from inspire_grasp import GraspConfig
    if not isinstance(value,dict) or set(value)!={'grasp','rack'}:raise ValueError('Rack mode requires exactly grasp and rack nested configurations')
    grasp=GraspConfig.from_dict(value['grasp']);rack=RackConfig.from_dict(value['rack'])
    expected=(math.sqrt(.5),0.,math.sqrt(.5),0.)
    if not all(math.isclose(a,b,abs_tol=1e-9) for a,b in zip(grasp.holder_orientation_palm_qwxyz,expected)):
        raise ValueError('These physical saddles require the declared horizontal palm-X barrel axis')
    return grasp,rack


def build_rack(stage,config,holder_center_world_m,radius_m=.012,material=None):
    """Six static, unfiltered collision boxes; never constrain the marker."""
    from pxr import Gf,UsdGeom,UsdPhysics,UsdShade
    root='/World/MarkerRack'
    if stage.GetPrimAtPath(root):raise ValueError('Rack namespace already occupied')
    UsdGeom.Xform.Define(stage,root)
    boxes=rack_boxes(config,holder_center_world_m,radius_m)
    for box in boxes:
        cube=UsdGeom.Cube.Define(stage,root+'/'+box['name']);cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*box['center_m']));cube.AddScaleOp().Set(Gf.Vec3f(*box['size_m']))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.20,.33,.48)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if material is not None:UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    return {'rack_root':root,'boxes':boxes,'static_physical_support':True,'marker_attachment':False,
            'collision_filters_added':False,'config_sha256':config.sha256,
            'support_qualification':'acquisition fixture only; any loaded rack contact invalidates clearance/retention'}


def rack_command(elapsed_s,config):
    if type(elapsed_s) not in (int,float) or not math.isfinite(elapsed_s) or elapsed_s<0:raise ValueError('Invalid rack time')
    def smooth(value):
        u=min(1.,max(0.,value));return u*u*u*(10+u*(-15+6*u))
    close=smooth((elapsed_s-.5)/1.)
    lift=smooth((elapsed_s-2.5)/2.)
    phase='rack_settle' if elapsed_s<.5 else 'rack_close' if elapsed_s<1.5 else 'rack_grip_settle' if elapsed_s<2.5 else 'rack_lift' if elapsed_s<4.5 else 'rack_clearance_settle' if elapsed_s<5. else 'retention_after_rack'
    return {'phase':phase,'closing_fraction':close,
        'wrist':[0.,0.,config.initial_wrist_z_m+lift*(config.lifted_wrist_z_m-config.initial_wrist_z_m),0.,0.,0.],
        'retention_window':elapsed_s>=5.,'external_support_allowed':elapsed_s<4.5}


def acquisition_result(rows,*,dt_s=.005):
    from inspire_grasp import retention_result
    if type(dt_s) not in (int,float) or not math.isclose(dt_s,.005,abs_tol=1e-12):raise ValueError('The declared rack sequence requires5ms physics')
    retained=[r for r in rows if r.get('phase')=='retention_after_rack']
    clearance=[r for r in rows if r.get('phase')=='rack_clearance_settle']
    supported=[r for r in rows if r.get('phase')=='rack_settle']
    continuous=bool(rows) and len(rows)==2000
    valid=True;error=None
    try:
        for i,row in enumerate(rows):
            expected=rack_command(i*dt_s,RackConfig())
            continuous &= type(row['sequence']) is int and row['sequence']==i and row['phase']==expected['phase']
            if type(row['physics_s']) not in (int,float) or not math.isfinite(row['physics_s']):raise ValueError('Nonfinite physical clock')
            if i:continuous &= math.isclose(row['physics_s']-rows[i-1]['physics_s'],dt_s,rel_tol=1e-5,abs_tol=1e-8)
            for key in ['rack_object_contact','external_object_contact','nonrack_external_object_contact','holder_hand_contact','rack_hand_contact']:
                if type(row[key]) is not bool:raise ValueError('Missing actual contact classification')
            if type(row['holder_lift_world_m']) not in (int,float) or not math.isfinite(row['holder_lift_world_m']):raise ValueError('Missing measured lift')
            if row['rack_object_contact'] and not row['external_object_contact']:raise ValueError('Rack support cannot be hidden from external contact')
    except (KeyError,TypeError,ValueError) as exc:
        valid=False;error=str(exc)
    def actual(row,key):return row.get(key) is True
    def cleared(row):
        lift=row.get('holder_lift_world_m')
        return row.get('external_object_contact') is False and type(lift) in (int,float) and math.isfinite(lift) and lift>=.05
    retention=retention_result(retained,expected_seconds=5.,dt_s=dt_s)
    checks={'rack_initially_supported_marker':any(actual(r,'rack_object_contact') for r in supported),
        'full_contiguous_declared_sequence':bool(continuous),'finite_measured_rack_evidence':valid,
        'no_unexpected_nonrack_support':not any(actual(r,'nonrack_external_object_contact') for r in rows),
        'no_loaded_rack_hand_contact':not any(actual(r,'rack_hand_contact') for r in rows),
        'full5s_free_hold':len(retained)==round(5./dt_s),
        'clearance_settled_half_second':len(clearance)==round(.5/dt_s) and all(cleared(r) for r in clearance),
        'measured_lift50mm':bool(retained) and all(cleared(r) for r in retained),
        'no_external_hold_support':bool(retained) and all(r.get('external_object_contact') is False for r in retained),
        'actual_hand_contact_entire_clearance_and_hold':bool(retained) and all(actual(r,'holder_hand_contact') for r in clearance+retained),
        'unchanged3mm3deg_retention':retention['accepted']}
    return {'checks':checks,'lift_hold_diagnostic_pass':all(checks.values()),'repeats_completed':0,
        'retention':retention,'evidence_error':error,
        'controlled_release_tested':False,'three_repeat_acquisition_qualified':False,
        'scope':'single_rack_supported_closure_then_contact_only_lift_and_hold; release/reacquisition require separate evaluation'}
