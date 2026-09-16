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
    # Declared release opening (rad) applied to the four fingers and thumb bend beyond the preload
    # pose during place/release/withdraw/re-approach of the three-cycle acquisition sequence;
    # the thumb yaw abducts fully (to 0 rad) at full opening.
    release_opening_rad: float = .3

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
        if not .1<=self.release_opening_rad<=.6:raise ValueError('Release opening must be a declared 0.1..0.6 rad')

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


# --- Three-cycle acquisition sequence (approach, close, lift, hold, place, release, withdraw, re-approach) ---
CYCLE_PHASES=[('cycle_settle',.5),('cycle_close',1.),('cycle_grip_settle',1.),('cycle_lift',2.),('cycle_clearance_settle',.5),
              ('cycle_hold',5.),('cycle_lower',2.),('cycle_place_settle',.5),('cycle_release',1.),('cycle_release_settle',.5),
              ('cycle_retreat',1.),('cycle_withdraw',2.),('cycle_withdrawn_settle',.5),('cycle_approach',2.),('cycle_advance',1.)]
CYCLE_SECONDS=sum(d for _,d in CYCLE_PHASES)   # 20.5 s
CYCLES=3
CYCLE_STEPS=round(CYCLES*CYCLE_SECONDS/.005)    # 12300
RETREAT_Y_M=-.05   # declared palm retreat along fixture -y after release (axis limit); 40 mm left the barrel against the thumb tip (cycles-v4-04)


def cycle_command(elapsed_s,config):
    """Declared three-cycle acquisition command: phase, closing fraction (0..1 relative to the preload
    pose), opening fraction (0..1 of release_opening_rad beyond the preload pose), wrist, flags."""
    if type(elapsed_s) not in (int,float) or not math.isfinite(elapsed_s) or elapsed_s<0:raise ValueError('Invalid cycle time')
    def smooth(value):
        u=min(1.,max(0.,value));return u*u*u*(10+u*(-15+6*u))
    cycle=min(CYCLES-1,int(elapsed_s//CYCLE_SECONDS));t=elapsed_s-cycle*CYCLE_SECONDS
    if elapsed_s>=CYCLES*CYCLE_SECONDS:cycle=CYCLES-1;t=CYCLE_SECONDS
    start=0.;phase=CYCLE_PHASES[-1][0];local=0.;duration=CYCLE_PHASES[-1][1]
    for name,d in CYCLE_PHASES:
        if t<start+d:phase=name;local=t-start;duration=d;break
        start+=d
    lo,hi=config.initial_wrist_z_m,config.lifted_wrist_z_m
    closing={'cycle_settle':0.,'cycle_close':smooth(local/duration),'cycle_grip_settle':1.,'cycle_lift':1.,'cycle_clearance_settle':1.,'cycle_hold':1.,
             'cycle_lower':1.,'cycle_place_settle':1.,'cycle_release':1.-smooth(local/duration),'cycle_release_settle':0.,'cycle_retreat':0.,
             'cycle_withdraw':0.,'cycle_withdrawn_settle':0.,'cycle_approach':0.,'cycle_advance':0.}[phase]
    # Fingers stay open through withdraw, re-approach and settle; they close from the open pose during
    # cycle_close (open->preload->closed in one smooth ramp) so the descent never sweeps closed fingers.
    opening={'cycle_settle':1.,'cycle_close':1.-smooth(local/duration),'cycle_release':smooth(local/duration),'cycle_release_settle':1.,
             'cycle_retreat':1.,'cycle_withdraw':1.,'cycle_withdrawn_settle':1.,'cycle_approach':1.,'cycle_advance':1.}.get(phase,0.)
    z={'cycle_settle':lo,'cycle_close':lo,'cycle_grip_settle':lo,'cycle_lift':lo+smooth(local/duration)*(hi-lo),'cycle_clearance_settle':hi,'cycle_hold':hi,
       'cycle_lower':hi-smooth(local/duration)*(hi-lo),'cycle_place_settle':lo,'cycle_release':lo,'cycle_release_settle':lo,'cycle_retreat':lo,
       'cycle_withdraw':lo+smooth(local/duration)*(hi-lo),'cycle_withdrawn_settle':hi,'cycle_approach':hi-smooth(local/duration)*(hi-lo),'cycle_advance':lo}[phase]
    y={'cycle_retreat':RETREAT_Y_M*smooth(local/duration),'cycle_withdraw':RETREAT_Y_M,'cycle_withdrawn_settle':RETREAT_Y_M,'cycle_approach':RETREAT_Y_M,
       'cycle_advance':RETREAT_Y_M*(1.-smooth(local/duration))}.get(phase,0.)
    held={'cycle_clearance_settle','cycle_hold'}
    return {'phase':phase,'cycle':cycle,'closing_fraction':closing,'opening_fraction':opening,'wrist':[0.,y,z,0.,0.,0.],
            'retention_window':phase=='cycle_hold','external_support_allowed':phase not in held,
            'object_should_be_free_of_hand':phase in ('cycle_withdrawn_settle',)}


def cycle_result(rows,config,*,dt_s=.005):
    """Per-cycle scoring of the declared three-cycle sequence; missing cycles are NOT_RUN, never passes."""
    from inspire_grasp import retention_result
    if type(dt_s) not in (int,float) or not math.isclose(dt_s,.005,abs_tol=1e-12):raise ValueError('The declared cycle sequence requires5ms physics')
    valid=True;error=None;continuous=bool(rows) and len(rows)==CYCLE_STEPS
    try:
        for i,row in enumerate(rows):
            expected=cycle_command(i*dt_s,config)
            continuous &= type(row['sequence']) is int and row['sequence']==i and row['phase']==expected['phase'] and row.get('cycle')==expected['cycle']
            if type(row['physics_s']) not in (int,float) or not math.isfinite(row['physics_s']):raise ValueError('Nonfinite physical clock')
            if i:continuous &= math.isclose(row['physics_s']-rows[i-1]['physics_s'],dt_s,rel_tol=1e-5,abs_tol=1e-8)
            for key in ['rack_object_contact','external_object_contact','nonrack_external_object_contact','holder_hand_contact','rack_hand_contact']:
                if type(row[key]) is not bool:raise ValueError('Missing actual contact classification')
            if type(row['holder_lift_world_m']) not in (int,float) or not math.isfinite(row['holder_lift_world_m']):raise ValueError('Missing measured lift')
    except (KeyError,TypeError,ValueError) as exc:
        valid=False;error=str(exc)
    def actual(row,key):return row.get(key) is True
    def cleared(row):
        lift=row.get('holder_lift_world_m');return row.get('external_object_contact') is False and type(lift) in (int,float) and math.isfinite(lift) and lift>=.05
    cycles=[]
    for k in range(CYCLES):
        rk=[r for r in rows if r.get('cycle')==k]
        ph=lambda name:[r for r in rk if r.get('phase')==name]
        held=ph('cycle_hold');clearance=ph('cycle_clearance_settle');placed=ph('cycle_place_settle');released=ph('cycle_release_settle')
        withdraw=ph('cycle_withdraw');withdrawn=ph('cycle_withdrawn_settle');settle=ph('cycle_settle');retreat=ph('cycle_retreat');approach=ph('cycle_approach');advance=ph('cycle_advance')
        retention=retention_result(held,expected_seconds=5.,dt_s=dt_s)
        late_withdraw=withdraw[len(withdraw)//5:]   # last 80 % of the withdraw ramp must be hand-free
        checks={'rack_supported_marker_at_cycle_start':any(actual(r,'rack_object_contact') for r in settle),
            'full5s_free_hold':len(held)==round(5./dt_s),
            'clearance_settled_half_second':len(clearance)==round(.5/dt_s) and all(cleared(r) for r in clearance),
            'measured_lift50mm':bool(held) and all(cleared(r) for r in held),
            'no_external_hold_support':bool(held) and all(r.get('external_object_contact') is False for r in held),
            'actual_hand_contact_entire_clearance_and_hold':bool(held) and all(actual(r,'holder_hand_contact') for r in clearance+held),
            'unchanged3mm3deg_retention':retention['accepted'],
            'placed_back_on_rack':bool(placed) and all(actual(r,'rack_object_contact') and abs(r.get('holder_lift_world_m',1.))<=.005 for r in placed[-20:]),
            'controlled_release_hand_free':bool(released) and bool(withdrawn) and all(not actual(r,'holder_hand_contact') for r in released[-20:]+late_withdraw+withdrawn),
            'marker_stays_racked_after_release':bool(withdrawn) and all(actual(r,'rack_object_contact') and abs(r.get('holder_lift_world_m',1.))<=.010 for r in released+retreat+withdraw+withdrawn+approach+advance),
            'no_unexpected_nonrack_support':not any(actual(r,'nonrack_external_object_contact') for r in rk),
            'no_loaded_rack_hand_contact':not any(actual(r,'rack_hand_contact') for r in rk)}
        cycles.append({'cycle':k,'samples':len(rk),'status':'NOT_RUN' if not rk else ('PASS' if all(checks.values()) else 'FAIL'),'checks':checks,'retention':retention})
    checks={'full_contiguous_declared_sequence':bool(continuous),'finite_measured_cycle_evidence':valid,
            **{f'cycle_{c["cycle"]}_complete':c['status']=='PASS' for c in cycles}}
    return {'checks':checks,'cycles':cycles,'cycles_completed':sum(c['status']=='PASS' for c in cycles),'evidence_error':error,
            'controlled_release_tested':any(c['samples'] for c in cycles),
            'three_repeat_acquisition_qualified':all(checks.values()) and all(c['status']=='PASS' for c in cycles),
            'scope':'three declared rack acquisition cycles (approach, close, lift>=50mm, 5 s hold, place, release, withdraw, re-approach) on a driven-wrist fixture; provisional donor; not free-standing'}

