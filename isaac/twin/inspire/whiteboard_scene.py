"""Declared simulation board and free marker/axial-slider assembly.

Imports PhysX/USD lazily. Defaults are synthetic engineering fixtures, not measured
robot calibration. Board x/right, y/up, z/toward the pen form a right-handed frame.
The holder's writing axis is local -Z; positive slider q is compression along +Z.
Only the optional diagnostic carriage attaches the holder to the world. Free mode
has no external joint, kinematic body, fixed palm, or object-follow pose controller.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
import hashlib
import json
import math


class InvalidScene(ValueError):
    pass


def finite(value, name='value'):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise InvalidScene(f'{name} must be a finite number')
    return float(value)


def vector(value, size):
    if not isinstance(value, (tuple, list)) or len(value) != size:
        raise InvalidScene(f'Expected {size} coordinates')
    return tuple(finite(v) for v in value)


def rotate(qwxyz, point):
    w, x, y, z = qwxyz
    px, py, pz = point
    return ((1-2*(y*y+z*z))*px + 2*(x*y-z*w)*py + 2*(x*z+y*w)*pz,
            2*(x*y+z*w)*px + (1-2*(x*x+z*z))*py + 2*(y*z-x*w)*pz,
            2*(x*z-y*w)*px + 2*(y*z+x*w)*py + (1-2*(x*x+y*y))*pz)


@dataclass(frozen=True)
class BoardFrame:
    origin_world_m: tuple = (0., 0., .85)
    orientation_world_qwxyz: tuple = (math.sqrt(.5), math.sqrt(.5), 0., 0.)

    def __post_init__(self):
        object.__setattr__(self, 'origin_world_m', vector(self.origin_world_m, 3))
        object.__setattr__(self, 'orientation_world_qwxyz', vector(self.orientation_world_qwxyz, 4))
        if not math.isclose(sum(v*v for v in self.orientation_world_qwxyz), 1., abs_tol=1e-8):
            raise InvalidScene('Board orientation must be a unit quaternion in wxyz order')

    def to_world(self, point):
        delta = rotate(self.orientation_world_qwxyz, vector(point, 3))
        return tuple(a+b for a, b in zip(self.origin_world_m, delta))

    def to_board(self, point):
        q = self.orientation_world_qwxyz
        delta = tuple(a-b for a, b in zip(vector(point, 3), self.origin_world_m))
        return rotate((q[0], -q[1], -q[2], -q[3]), delta)

    @property
    def normal_world(self):
        return rotate(self.orientation_world_qwxyz, (0., 0., 1.))


@dataclass(frozen=True)
class BoardParameters:
    width_m: float = .50
    height_m: float = .35
    thickness_m: float = .012
    static_friction: float = .45
    dynamic_friction: float = .35
    support_mode: str = 'fixed_stand'
    stand_post_width_m: float = .035
    stand_foot_depth_m: float = .35
    stand_foot_height_m: float = .035
    ground_z_m: float = 0.

    def __post_init__(self):
        for field in fields(self):
            if field.name != 'support_mode': finite(getattr(self, field.name), field.name)
        if (not .10 <= self.width_m <= 2 or not .10 <= self.height_m <= 2
                or not .003 <= self.thickness_m <= .05
                or not 0 <= self.dynamic_friction <= self.static_friction <= 2
                or not .01 <= self.stand_post_width_m <= .10
                or not .10 <= self.stand_foot_depth_m <= 1
                or not .01 <= self.stand_foot_height_m <= .10):
            raise InvalidScene('Board dimensions/friction outside declared fixture envelope')
        if self.support_mode != 'fixed_stand':
            raise InvalidScene('Only the declared fixed stand is implemented; dynamic support is unqualified')


@dataclass(frozen=True)
class HolderParameters:
    body_mass_kg: float = .080
    body_radius_m: float = .012
    body_length_m: float = .100
    cartridge_mass_kg: float = .008
    cartridge_radius_m: float = .003
    cartridge_length_m: float = .050
    nib_radius_m: float = .0015
    tip_offset_m: float = .080
    slider_travel_m: float = .020
    nominal_compression_m: float = .005
    preload_n: float = 1.
    stiffness_n_m: float = 200.
    damping_n_s_m: float = 1.
    slider_friction_coefficient: float = .02
    drive_force_limit_n: float = 15.
    nib_static_friction: float = .40
    nib_dynamic_friction: float = .30
    contact_offset_m: float = .0001
    bottomout_margin_m: float = .0002

    def __post_init__(self):
        for field in fields(self): finite(getattr(self, field.name), field.name)
        if (not .01 <= self.body_mass_kg <= .5 or not .001 <= self.cartridge_mass_kg <= .05
                or not .005 <= self.body_radius_m <= .035 or not .03 <= self.body_length_m <= .2
                or not .0005 <= self.nib_radius_m <= .005
                or not self.nib_radius_m <= self.cartridge_radius_m < self.body_radius_m
                or not .01 <= self.cartridge_length_m <= .12
                or not self.body_length_m/2 + self.nib_radius_m < self.tip_offset_m <= .2
                or not .005 <= self.slider_travel_m <= .04
                or not 0 < self.nominal_compression_m < self.slider_travel_m
                or not 0 <= self.preload_n <= 5 or not 20 <= self.stiffness_n_m <= 2000
                or not 0 <= self.damping_n_s_m <= 20 or not 0 <= self.slider_friction_coefficient <= 1
                or not 1 <= self.drive_force_limit_n <= 30
                or self.preload_n+self.stiffness_n_m*self.nominal_compression_m > self.drive_force_limit_n
                or not 0 <= self.nib_dynamic_friction <= self.nib_static_friction <= 2
                or not 0 < self.contact_offset_m < self.nib_radius_m
                or not 0 < self.bottomout_margin_m < self.nominal_compression_m):
            raise InvalidScene('Holder parameters outside declared physical fixture envelope')

    @property
    def spring_target_m(self):
        # Fq = k(q_target-q)-c*dq = -preload-k*q-c*dq: toward the nib, not into the holder.
        return -self.preload_n/self.stiffness_n_m

    def force_model_toward_tip_n(self, compression_m, velocity_m_s):
        raw = self.preload_n + self.stiffness_n_m*finite(compression_m) + self.damping_n_s_m*finite(velocity_m_s)
        return max(-self.drive_force_limit_n, min(self.drive_force_limit_n, raw))

    def travel_reserves(self, compression_m):
        q=finite(compression_m)
        if not 0 <= q <= self.slider_travel_m: raise InvalidScene('Compression outside travel')
        return {'toward_contact_loss_m': q, 'toward_bottomout_m': self.slider_travel_m-q,
                'geometric_only_not_lateral_breakaway': True}


@dataclass(frozen=True)
class SceneConfig:
    schema_version: int = 1
    frame: BoardFrame = BoardFrame()
    board: BoardParameters = BoardParameters()
    holder: HolderParameters = HolderParameters()
    holder_mode: str = 'free_dynamic'
    initial_xy_board_m: tuple = (-.070, .030)
    initial_gap_m: float = .010
    carriage_stiffness_n_m: float = 4000.
    carriage_damping_n_s_m: float = 80.
    carriage_force_limit_n: float = 30.
    physics_dt_s: float = .005
    provenance: str = 'declared_synthetic_simulation_fixture_not_hardware_calibration'

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1: raise InvalidScene('Unknown scene schema')
        if (type(self.frame) is not BoardFrame or type(self.board) is not BoardParameters
                or type(self.holder) is not HolderParameters): raise InvalidScene('Validated nested parameters required')
        if self.holder_mode not in {'free_dynamic', 'driven_carriage'}: raise InvalidScene('Unknown holder mode')
        if self.provenance != 'declared_synthetic_simulation_fixture_not_hardware_calibration':
            raise InvalidScene('This constructor cannot assert measured hardware calibration')
        object.__setattr__(self, 'initial_xy_board_m', vector(self.initial_xy_board_m, 2))
        for name in ['initial_gap_m','carriage_stiffness_n_m','carriage_damping_n_s_m','carriage_force_limit_n','physics_dt_s']:
            finite(getattr(self, name), name)
        if (not .003 <= self.initial_gap_m <= .05 or not 500 <= self.carriage_stiffness_n_m <= 10000
                or not 1 <= self.carriage_damping_n_s_m <= 200 or not 5 <= self.carriage_force_limit_n <= 100
                or not .001 <= self.physics_dt_s <= .01
                or abs(self.initial_xy_board_m[0]) >= self.board.width_m/2-.02
                or abs(self.initial_xy_board_m[1]) >= self.board.height_m/2-.02):
            raise InvalidScene('Carriage/frame settings outside declared fixture envelope')

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict): raise InvalidScene('Scene config must be an object')
        if set(data)-{f.name for f in fields(cls)}: raise InvalidScene('Unknown scene config keys')
        copied=dict(data)
        for key, nested in [('frame',BoardFrame),('board',BoardParameters),('holder',HolderParameters)]:
            if key in copied:
                if not isinstance(copied[key],dict) or set(copied[key])-{f.name for f in fields(nested)}:
                    raise InvalidScene('Unknown nested scene keys')
                copied[key]=nested(**copied[key])
        return cls(**copied)

    @property
    def sha256(self):
        return hashlib.sha256(json.dumps(asdict(self),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

    @property
    def scope(self):
        attached=self.holder_mode == 'driven_carriage'
        return {'holder_mode':self.holder_mode,'attachment_active':attached,'fixture_support_active':attached,
                'board_fixed_fixture':True,'grasp_qualified':False,'writing_qualified':False,
                'standing_qualified':False,'task_node_integrated':False,
                'scope':'instrumentation_carriage_bench_only' if attached else 'unqualified_free_dynamic_marker_asset'}


def compression_from_poses(holder_position_world, holder_qwxyz, nib_position_world, holder):
    q=vector(holder_qwxyz,4)
    if not math.isclose(sum(v*v for v in q),1.,abs_tol=1e-5): raise InvalidScene('Nonunit runtime orientation')
    delta=tuple(a-b for a,b in zip(vector(nib_position_world,3),vector(holder_position_world,3)))
    local=rotate((q[0],-q[1],-q[2],-q[3]),delta)
    return local[2] - (-holder.tip_offset_m+holder.nib_radius_m)


def reduce_tip_contacts(rows, *, tip_collider, board_collider, frame, nib_position_world, dt_s):
    """Only actual tip-shape/board-shape impulses can produce a marked contact.

    None means unavailable telemetry. Zero-impulse proximity reports are retained in
    raw contacts but are not counted as load-bearing contacts. Position is an independent
    impulse-weighted physics contact location, never an intended/reference point.
    """
    dt=finite(dt_s)
    if dt <= 0: raise InvalidScene('Positive dt required')
    fallback=frame.to_board(nib_position_world)
    if rows is None:
        return {'nib_board_contact':None,'normal_impulse_ns':None,'normal_force_n':None,
                'position_board_m':fallback,'position_source':'measured_body_tip_no_contact_observation','matched_points':None}
    impulses=[]
    for row in rows:
        if {row.get('collider0'),row.get('collider1')} != {tip_collider,board_collider}: continue
        impulse=vector(row['impulse_ns'],3); point=vector(row['position_world_m'],3)
        normal_impulse=abs(sum(a*b for a,b in zip(impulse,frame.normal_world)))
        if normal_impulse > 0: impulses.append((normal_impulse,frame.to_board(point)))
    total=sum(a for a,_ in impulses)
    position=tuple(sum(weight*point[i] for weight,point in impulses)/total for i in range(3)) if total else fallback
    return {'nib_board_contact':bool(total),'normal_impulse_ns':total,'normal_force_n':total/dt,
            'position_board_m':position,'position_source':'measured_contact_impulse_weighted' if total else 'measured_body_tip_no_load_contact',
            'matched_points':len(impulses)}


def build_scene(stage, config=SceneConfig(), *, board_path='/World/Whiteboard', marker_path='/World/Marker'):
    """Author actual rigid bodies and joints. Runtime motion must use drives/contact only."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    if type(config) is not SceneConfig: raise InvalidScene('Validated SceneConfig required')
    if stage.GetPrimAtPath(board_path) or stage.GetPrimAtPath(marker_path): raise InvalidScene('Scene output paths already exist')
    frame, board, h=config.frame,config.board,config.holder
    q=Gf.Quatf(frame.orientation_world_qwxyz[0],Gf.Vec3f(*frame.orientation_world_qwxyz[1:]))
    def xform(path,position,orientation=q):
        obj=UsdGeom.Xform.Define(stage,path);obj.AddTranslateOp().Set(Gf.Vec3d(*position));obj.AddOrientOp().Set(orientation);return obj.GetPrim()
    def material(path,static,dynamic):
        obj=UsdShade.Material.Define(stage,path);api=UsdPhysics.MaterialAPI.Apply(obj.GetPrim())
        api.CreateStaticFrictionAttr(static);api.CreateDynamicFrictionAttr(dynamic);api.CreateRestitutionAttr(0.);return obj
    def collision(prim,mat):
        UsdPhysics.CollisionAPI.Apply(prim)
        api=PhysxSchema.PhysxCollisionAPI.Apply(prim);api.CreateContactOffsetAttr(h.contact_offset_m);api.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
    def cube(path,dimensions,position,color,mat,orientation=Gf.Quatf(1.)):
        obj=UsdGeom.Cube.Define(stage,path);obj.CreateSizeAttr(1.);obj.AddTranslateOp().Set(Gf.Vec3d(*position))
        obj.AddOrientOp().Set(orientation);obj.AddScaleOp().Set(Gf.Vec3f(*dimensions));obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        collision(obj.GetPrim(),mat);return obj.GetPrim()
    def rigid(path,position,mass,inertia,com=(0.,0.,0.)):
        prim=xform(path,position);UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(False)
        massapi=UsdPhysics.MassAPI.Apply(prim);massapi.CreateMassAttr(mass);massapi.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        massapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia));massapi.CreatePrincipalAxesAttr(Gf.Quatf(1.))
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.);return prim
    def cylinder(path,radius,length,position,color,mat):
        obj=UsdGeom.Cylinder.Define(stage,path);obj.CreateAxisAttr('Z');obj.CreateRadiusAttr(radius);obj.CreateHeightAttr(length)
        obj.AddTranslateOp().Set(Gf.Vec3d(*position));obj.CreateDisplayColorAttr([Gf.Vec3f(*color)]);collision(obj.GetPrim(),mat)
    def prism(name,body0,body1,axis,lower,upper,anchor=(0.,0.,0.),stiffness=0.,damping=0.,target=0.,force=0.):
        joint=UsdPhysics.PrismaticJoint.Define(stage,marker_path+'/'+name);joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)]);joint.CreateAxisAttr(axis)
        joint.CreateLocalPos0Attr(Gf.Vec3f(*anchor));joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.));joint.CreateLocalRot1Attr(Gf.Quatf(1.))
        joint.CreateLowerLimitAttr(lower);joint.CreateUpperLimitAttr(upper);joint.CreateCollisionEnabledAttr(False)
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(stiffness);drive.CreateDampingAttr(damping);drive.CreateTargetPositionAttr(target)
        drive.CreateTargetVelocityAttr(0.);drive.CreateMaxForceAttr(force)
        return joint
    UsdGeom.Xform.Define(stage,board_path);UsdGeom.Xform.Define(stage,marker_path)
    boardmat=material(board_path+'/BoardMaterial',board.static_friction,board.dynamic_friction)
    nibmat=material(marker_path+'/NibMaterial',h.nib_static_friction,h.nib_dynamic_friction)
    gripmat=material(marker_path+'/GripMaterial',.7,.6)
    panel=board_path+'/Panel'
    cube(panel,(board.width_m,board.height_m,board.thickness_m),frame.to_world((0.,0.,-board.thickness_m/2)),(.94,.95,.92),boardmat,q)
    for side in [-1,1]:
        end=frame.to_world((side*board.width_m*.35,-board.height_m/2,-board.thickness_m))
        height=end[2]-board.ground_z_m
        if height <= board.stand_foot_height_m: raise InvalidScene('Board pose cannot be supported by this fixed stand')
        label='Left' if side<0 else 'Right'
        cube(board_path+'/'+label+'Post',(board.stand_post_width_m,board.stand_post_width_m,height),
             (end[0],end[1],board.ground_z_m+height/2),(.18,.20,.22),boardmat)
        cube(board_path+'/'+label+'Foot',(.13,board.stand_foot_depth_m,board.stand_foot_height_m),
             (end[0],end[1],board.ground_z_m+board.stand_foot_height_m/2),(.15,.17,.19),boardmat)
    x,y=config.initial_xy_board_m; initial_z=h.tip_offset_m+config.initial_gap_m
    holder_path=marker_path+'/Holder';nib_path=marker_path+'/Nib'
    transverse=h.body_mass_kg*(3*h.body_radius_m**2+h.body_length_m**2)/12
    rigid(holder_path,frame.to_world((x,y,initial_z)),h.body_mass_kg,(transverse,transverse,.5*h.body_mass_kg*h.body_radius_m**2))
    cylinder(holder_path+'/Grip',h.body_radius_m,h.body_length_m,(0.,0.,0.),(.15,.25,.60),gripmat)
    # Cartridge mass model: 80% uniform cylinder, 20% spherical tip; explicit COM/inertia.
    shaft_mass=.8*h.cartridge_mass_kg;tip_mass=.2*h.cartridge_mass_kg;shaft_com=h.cartridge_length_m/2
    com_z=shaft_mass*shaft_com/h.cartridge_mass_kg
    transverse_n=(shaft_mass*(3*h.cartridge_radius_m**2+h.cartridge_length_m**2)/12
                  +shaft_mass*(shaft_com-com_z)**2+.4*tip_mass*h.nib_radius_m**2+tip_mass*com_z**2)
    axial_n=.5*shaft_mass*h.cartridge_radius_m**2+.4*tip_mass*h.nib_radius_m**2
    rest_z=-h.tip_offset_m+h.nib_radius_m
    rigid(nib_path,frame.to_world((x,y,initial_z+rest_z)),h.cartridge_mass_kg,(transverse_n,transverse_n,axial_n),(0.,0.,com_z))
    cylinder(nib_path+'/Shaft',h.cartridge_radius_m,h.cartridge_length_m,(0.,0.,shaft_com),(.70,.70,.72),gripmat)
    tip=UsdGeom.Sphere.Define(stage,nib_path+'/Tip');tip.CreateRadiusAttr(h.nib_radius_m)
    tip.CreateDisplayColorAttr([Gf.Vec3f(.03,.03,.04)]);collision(tip.GetPrim(),nibmat)
    slider=prism('NibCompression',holder_path,nib_path,'Z',0.,h.slider_travel_m,(0.,0.,rest_z),
                 h.stiffness_n_m,h.damping_n_s_m,h.spring_target_m,h.drive_force_limit_n)
    PhysxSchema.PhysxJointAPI.Apply(slider.GetPrim()).CreateJointFrictionAttr(h.slider_friction_coefficient)
    carriage_joints=[]
    if config.holder_mode=='driven_carriage':
        anchor=marker_path+'/CarriageAnchor'; cx=marker_path+'/CarriageX';cy=marker_path+'/CarriageY'
        for path,pos in [(anchor,(0.,0.,0.)),(cx,(x,0.,initial_z)),(cy,(x,y,initial_z))]:
            rigid(path,frame.to_world(pos),.2,(.0001,.0001,.0001))
        fixed=UsdPhysics.FixedJoint.Define(stage,marker_path+'/CarriageWorldFixture')
        fixed.CreateBody1Rel().SetTargets([Sdf.Path(anchor)]);fixed.CreateLocalPos0Attr(Gf.Vec3f(*frame.origin_world_m))
        fixed.CreateLocalRot0Attr(q);fixed.CreateLocalPos1Attr(Gf.Vec3f(0.));fixed.CreateLocalRot1Attr(Gf.Quatf(1.))
        for name,body0,body1,axis,limits,position,target in [
            ('CarriageHorizontal',anchor,cx,'X',(-board.width_m/2,board.width_m/2),(0.,0.,initial_z),x),
            ('CarriageVertical',cx,cy,'Y',(-board.height_m/2,board.height_m/2),(0.,0.,0.),y),
            ('CarriageNormal',cy,holder_path,'Z',(-(config.initial_gap_m+h.slider_travel_m+.01),.04),(0.,0.,0.),0.)]:
            prism(name,body0,body1,axis,*limits,position,config.carriage_stiffness_n_m,config.carriage_damping_n_s_m,target,config.carriage_force_limit_n)
            carriage_joints.append(name)
        root=fixed.GetPrim()
    else:
        root=stage.GetPrimAtPath(holder_path)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    articulation=PhysxSchema.PhysxArticulationAPI.Apply(root)
    articulation.CreateEnabledSelfCollisionsAttr(True);articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(8)
    return {'schema_version':1,'config':asdict(config),'config_sha256':config.sha256,'scope':config.scope,
            'articulation_path':marker_path,'holder_body':holder_path,'nib_body':nib_path,
            'tip_collider':nib_path+'/Tip','board_collider':panel,'carriage_joint_names':carriage_joints,
            'slider_joint_name':'NibCompression','holder_axis_toward_board_local':(0.,0.,-1.),
            'slider_positive_compression_axis_local':(0.,0.,1.),'spring_target_position_m':h.spring_target_m,
            'spring_force_model':'Fq = -preload - stiffness*q - damping*dq, bounded by drive limit',
            'friction_model':'PhysX joint coefficient times transmitted force/torque bound; damping is separate',
            'adjacent_holder_cartridge_collision_disabled':True,'other_collision_pairs_filtered':False,
            'mass_model':'holder uniform cylinder; cartridge 80% shaft cylinder and 20% tip sphere',
            'total_free_object_mass_kg':h.body_mass_kg+h.cartridge_mass_kg,
            'travel_reserves_at_nominal':h.travel_reserves(h.nominal_compression_m)}
