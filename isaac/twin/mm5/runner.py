"""MM5 trial runner: N randomized trials, taxonomy, per-stage timing, JSON + markdown.

Stepped from RobotRosRunner.on_physics_step. Owns trial setup and teardown; the
per-trial state machine is MM5Pipeline.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, asdict


import numpy as np

from .pipeline import MM5Pipeline, STAGES


@dataclass
class MM5Config:
    robot_root: str = "/World/G1"
    object_name: str = "soup_can"
    trials: int = 20
    seed: int = 20260820

    # Where the base stands relative to the object. The `table` waypoint is 1.1 m from
    # the objects and the G1's arm reaches ~0.7 m, so navigating to a waypoint and then
    # reaching is geometrically impossible -- MM5 stands the robot at a REACH pose.
    # The table's near edge is at y = -1.20 and the G1's footprint radius is 0.35 m, so
    # a base placed 0.52 m from an object at y = -1.45 stands INSIDE the table: the
    # teleport put the robot in the furniture and it went straight down. 0.72 m leaves
    # 0.48 m of clearance to the edge and still puts the object inside the arm's ~0.65 m
    # reach, because the shoulder is 1.1 m up and the object is at 0.80 m.
    # MEASURED, not assumed. With the base 0.72 m back the IK converges rock-stable to
    # 354 mm and stops -- not an oscillation, a workspace boundary: the right arm's
    # effective forward reach is ~0.39 m from the pelvis, because it is also reaching
    # DOWN from a shoulder at 1.1 m to a can at 0.80 m, and that vertical component eats
    # most of the nominal 0.65 m arm length. 0.38 m puts the can inside it.
    #
    # This does put the base inside the Nav2 footprint clearance of the table. In the
    # FIXED-BASE variant that is harmless -- the base is held and cannot be toppled --
    # and it is one more reason the fixed-base result is not the mobile one.
    base_offset: tuple = (0.10, 0.38)      # (+x, +y) from the object, world frame
    base_yaw: float = -1.5708              # face -y (table); +1.5708 faces +y (counter)

    # A pre-grasp is OFFSET from the object by design -- the fingers close over the
    # remainder. 6 cm above the can's centre is where a side grasp starts.
    # Pre-grasp on the NEAR side of the can and slightly above it: that is where a side
    # grasp starts, and it is also 6 cm closer to the shoulder than the can's centre,
    # which matters when the reach is workspace-limited. Going closer than this by moving
    # the BASE does not work -- at 0.32 m the body is inside the table and knocks the can
    # onto the floor before the arm arrives.
    # TOP-DOWN grasp, because that is what this arm's posture naturally presents: the
    # palm's local z measured at [-0.096, -0.081, -0.992], i.e. already facing the table.
    # Pre-grasp is directly ABOVE the can; DESCEND then comes straight down onto it.
    pregrasp_offset: tuple = (0.0, 0.0, 0.13)
    # 9 cm above the can's CENTRE is ~4 cm above its lid (the can is 0.101 tall), which
    # is inside the Dex5's finger length. Going lower costs forward reach -- the arm is
    # already at the edge of its workspace at this height -- and the first v2 run stalled
    # 114 mm short trying to descend 9.5 cm.
    grasp_offset: tuple = (0.0, 0.0, 0.072)
    # v3 applies the same stand-offs ALONG the computed approach axis rather than
    # always along world -z; on a table the two are the same thing.
    pregrasp_standoff: float = 0.23      # grasp_standoff + pregrasp_extra, same source
    # 0.072 came from v2, where the approach was TOP-DOWN and the number meant
    # "palm this far above the lid" -- 0.072 against a 0.101 m can is ~20 mm of
    # clearance over the rim, inside the finger length. Along a SIDE approach the same
    # number means "palm this far from the can's AXIS", and with a 33 mm radius that
    # leaves 39 mm of air between palm and wall: the fingers close on nothing, which
    # is exactly what the counter trials reported (NO_GRIP at 25 mm, object rose
    # -0.018 m). Same stand-off measured against the surface the palm actually faces.
    # MEASURED, not argued: the Dex5's fingers converge 0.1366 m from the palm origin
    # (v4 measured it, HANDCAL re-measures it every run with MM5_MEASURE_HAND=1), plus
    # grasp_clearance. The old default was 0.045, which put the descent target ~92 mm
    # INSIDE the can -- and the observed DESCEND stalls were 80-101 mm, i.e. exactly the
    # gap between the two numbers. The palm drove into the can, shoved it 0.30-0.37 m,
    # and never reached a pose that was physically inside the object, so no trial ever
    # reached closure and grip_contacts stayed -1 for four versions. v4 found this and
    # only applied it behind the MEASURE_HAND flag; the default kept the wrong number.
    grasp_standoff: float = 0.1466
    # Orientation weight. v2 used 0.35 so a workspace-limited reach could not have its
    # authority stolen by the rotation term -- and the consequence was that the palm
    # never actually reached the commanded axis, measuring [-0.698,-0.700,0.153] at a
    # pre-grasp that was supposed to be top-down. With the base now solved so the can
    # sits 0.315 m from the shoulder, the reach is no longer at the workspace edge and
    # orientation can be afforded real weight.
    w_rot: float = 0.8
    # The hand used to close at `grasp_tol` = 60 mm from the grasp pose, which is most
    # of a can-radius away; every counter trial closed on air. If the arm cannot get
    # closer than this the honest outcome is DESCEND_TIMEOUT, not a NO_GRIP that reads
    # like a gripper problem.
    grasp_tol: float = 0.025
    descend_timeout_s: float = 15.0
    reach_tol: float = 0.08
    ik_lambda: float = 0.08
    # A SLOW reach. The omni policy holds the arms as part of its own action and was
    # trained with a joint_deviation_arms term, so an arm driven away from that pose is
    # out of its distribution -- and each hand is 1 kg at the longest lever on the robot.
    # At 0.05 rad/step the CoM moved faster than the policy could answer and it toppled
    # in every trial. 0.008 gives it ~6x longer to compensate.
    ik_max_step: float = 0.004
    ik_lead_cap: float = 0.10
    ik_k_null: float = 0.6              # how far the command may lead the arm (rad)

    settle_s: float = 6.0
    fallen_z: float = 0.55
    spawn_z: float = 0.80                 # pelvis below this = the robot is down
    reach_timeout_s: float = 30.0
    # SLOW. Fix (b): at 2.5 s the fingers arrive fast enough to flick a free-standing
    # can over -- measured at 90.4 deg tilt during closure even with the thumb leading as
    # a back-stop. A slower ramp lets the first contact settle the object against the
    # thumb instead of sweeping it, and lets the opposing contacts arrive near-together.
    close_s: float = 2.5
    # Closure. The Dex5 finger joints run to ~1.4-1.7 rad, and 0.9 left the hand open
    # around a 66 mm can -- it closed and did not grip (`NO_GRIP`, object rose -0.015 m).
    close_rad: float = 1.35
    # GRASP V5. The hand closes fully and does not hold, so closure has to become
    # FORCE-generating rather than merely complete.
    #   overclose_rad -- drive the finger targets PAST the contact pose; a position
    #     drive makes force out of error, and a target AT contact makes ~none.
    #   grip_kp/kd    -- C-38 capped the finger drives at 5.0, which is the SONIC-IDLE
    #     value (so an idle hand does not fight itself), not a grasping value. Raised
    #     for GRASP and LIFT only and reverted afterwards; the URDF effort clamp
    #     (0.93 Nm per finger joint) still bounds what this can produce.
    #   min_grip_contacts -- do not LIFT a hand that is not touching the object.
    overclose_rad: float = 0.25
    grip_kp: float = 20.0
    grip_kd: float = 0.5
    idle_hand_kp: float = 5.0
    idle_hand_kd: float = 0.1
    min_grip_contacts: int = 2
    # TOPPLE thresholds. A can that tips stays on the counter and never trips the
    # knocked-off test, so it was reported as NO_GRIP for four versions.
    topple_tilt_deg: float = 30.0
    topple_drop_m: float = 0.010
    # Fraction of the closure ramp the THUMB gets to itself before the fingers move.
    # 0.0 = the original simultaneous close, which is the configuration that actually
    # produced contacts (3-4 links, thumb opposing). thumb_lead 0.40/0.55 with an 8 s ramp
    # was introduced to stop a topple that MEASUREMENT LATER SHOWED DOES NOT HAPPEN, and
    # it cost every contact: 0 links at closure. Kept as a knob, off by default.
    thumb_lead: float = 0.0
    # Radius about the can's vertical axis inside which a finger link counts as "around"
    # it. The 005 tomato soup can is ~66 mm across, so 45 mm is comfortably inside the
    # region a caging finger must occupy.
    enclose_radius_m: float = 0.045
    lift_h: float = 0.14
    # Metres per second the lift target is raised. The trace showed the hand rising 10 cm
    # in 0.5 s and shedding every contact; 0.03 m/s takes ~4.7 s for the full lift_h.
    lift_rate_mps: float = 0.03
    lift_success_h: float = 0.05
    lift_timeout_s: float = 14.0   # rate-limited lift needs ~4.7 s plus settle
    carry_vec: tuple = (-1.0, 0.0, 0.0)    # 1 m along -x, along the table
    carry_dist: float = 1.0
    carry_timeout_s: float = 20.0
    drop_h: float = 0.02
    place_s: float = 2.0
    retreat_s: float = 2.0
    table_h: float = 0.75           # "is it still on a surface" threshold; see surface

    cheat_attach: bool = False
    # FIXED-BASE VARIANT. The base is held kinematically so the pipeline can be tested
    # independently of the balancer, which is parked at C-39 and which the mobile run
    # showed topples 2.9 s into every reach. Everything downstream of balance -- IK
    # reach, real-contact Dex5 grasp, lift, place -- is exercised for real. Labelled on
    # every row and in the header; it is NOT the robot, and the carry stage is skipped
    # because carrying a payload 1 m is meaningless when the base cannot move.
    fix_base: bool = False
    # The IK limits below are sized for a BALANCING robot, where a fast-moving arm is
    # what topples it. With the base held there is nothing to topple, so the fixed-base
    # variant lifts them: at lead 0.10 rad the target stops advancing as soon as the
    # arm's own tracking error reaches the cap, and the reach stalled at a repeatable
    # 350 mm short.
    fix_base_arm_kp: float = 400.0
    fix_base_arm_kd: float = 20.0
    # The lead cap is how far the IK target may run ahead of the arm. v1 raised it
    # from 0.10 to 0.25 because at 0.10 "the target stops advancing as soon as the
    # arm's own tracking error reaches the cap, and the reach stalled at a repeatable
    # 350 mm short". v3 shows the SAME signature one stage later: DESCEND stalls at
    # 51-57 mm in eight of thirteen timeouts even though the can is randomised over a
    # +-6 cm box. A workspace boundary moves with the target; a fixed stall distance
    # is a cap. Raised again for the fixed-base variant, where there is no balance to
    # protect and the cap's original purpose does not apply.
    fix_base_ik_lead: float = 0.25
    fix_base_ik_step: float = 0.010
    skip_carry: bool = True
    place_vec: tuple = (-0.22, 0.0, 0.0)
    randomize_m: float = 0.06              # +-6 cm box on the table
    # MM5 stages the object on the NEAR edge of the table rather than where the MM2
    # seed happened to scatter it. The G1 stands 0.48 m clear of a 1.20 m-deep table and
    # its arm reaches ~0.65 m from a shoulder 1.1 m up; an object in the middle of the
    # table is simply outside the workspace, and the first runs proved it -- the IK
    # closed 0.11 m of a 0.70 m gap and stalled against the arm's own limits. This is a
    # scenario choice and is stated as one, not a fudge: the pick is still a real pick.
    stage_y: float = -1.31                 # table near edge is y = -1.20

    # SURFACE. The table top is 0.75 m and the arm, from a fixed base 0.38 m away, tops
    # out with the palm around z = 0.95 -- measured, repeatedly: it reaches the pre-grasp
    # 0.13 m above the can and then cannot descend the last 9 cm, stalling 114 mm short.
    # The lab's counter is 0.90 m, 15 cm higher, which puts the can inside the workspace
    # instead of at its floor. Staging there is a scenario choice and is stated as one.
    #   counter 2.40 x 0.60 x 0.90 centred (-2.60, 2.40) -> near edge y = 2.10
    # ATTEMPTED and reverted: staging on the counter puts the can at a reachable HEIGHT
    # but needs its own base placement (approach from -y, right arm then on the +x side)
    # and that geometry was not tuned inside the time box -- it reached only 577-641 mm.
    # Left as the documented next step; the table staging is what the N=20 below used.
    surface: str = "table"
    # The counter is 2.40 x 0.60 x 0.90 centred (-2.60, 2.40), so it spans y 2.10..2.70
    # and its NEAR EDGE is y = 2.10. The first v3 run staged the can at y = 2.05 -- five
    # centimetres in FRONT of the counter -- and it fell to the floor on trial 1
    # (z = 0.032) before the arm ever moved. Staged 0.15 m inside the edge instead.
    counter_xy: tuple = (-2.60, 2.25)
    counter_h: float = 0.90

    # ---------------------------------------------------------------- GRASP V3
    # v1 and v2 both found the base offset by ITERATING: pick a number, run 20 trials,
    # read the reach error, pick another.  That produced 0.38 m and a documented
    # workspace ceiling -- the palm tops out near z = 0.95 while the can sits at 0.80,
    # so the arm reaches the pre-grasp 0.13 m high and cannot descend onto the can.
    # It also never converged for the counter: 577-641 mm, box expired.
    #
    # v3 computes it instead.  The arm's reachable set is a cone around the SHOULDER,
    # not around the pelvis, so the shoulder is what the placement has to be measured
    # from.  MEASURE the shoulder in the robot's own frame at the home pose, then solve
    # the one unknown -- how far forward to stand -- so the can lands at `target_r`,
    # comfortably inside the 0.39 m limit rather than at its edge where both previous
    # runs stalled.  One measurement, one solve, no iteration.
    # v4: measure the hand instead of arguing about the stand-off. See dex5_geom.py.
    measure_hand: bool = False
    # v6: ContactSensor prim paths created at world setup (run.py), because in this
    # build they must exist before the sim steps. Empty = no contact route, and the
    # pipeline says so out loud rather than reporting zero contacts.
    contact_sensor_paths: tuple = ()
    # v7: "report" = PhysX contact-report callback (default), "" = disable.
    contact_route: str = "report"
    # NEGATIVE on purpose. stand-off = measured finger convergence + this. With +0.010 the
    # convergence point lands 10 mm SHORT of the can's centre, so the can sits at the very
    # edge of the closing envelope: measured 1 finger in contact at 38 N peak while 11 of
    # 20 joints closed on air. Pulling the palm 15 mm closer puts the convergence point
    # PAST the centre, so the fingers close BEHIND the widest part and can oppose each
    # other. Force was never the problem -- 38 N against the ~3.4 N a 0.349 kg can needs.
    #
    # -0.030 rather than -0.015 because of WHICH links touch. At -0.015 the contacts are
    # Link_14R (thumb distal, 6.8 N), Link_24R (2.8 N) and Link_34R (0.5 N) -- all DISTAL.
    # That is a three-point fingertip pinch on a smooth cylinder: the thumb genuinely
    # opposes two fingers, and it still slips under lift ("object rose +0.001 m"), because
    # a tip pinch has almost no contact area and the can rolls out of it. Seating the can
    # 15 mm deeper brings the middle and proximal links onto it, which is a wrap.
    grasp_clearance: float = -0.015   # fingers must pass the widest part before closing
    pregrasp_extra: float = 0.085    # pre-grasp this much further out along the axis
    place_from_workspace: bool = False
    target_r: float = 0.315         # can this far from the RIGHT SHOULDER (0.30-0.33)
    lateral_offset: float = 0.0     # can this far right OF THE SHOULDER (0 = in front)
    min_forward: float = 0.20       # never solve the base inside the furniture
    max_forward: float = 0.60
    # The base is rig-held in this variant, so pelvis height is a free parameter rather
    # than a consequence of standing.  Lowering it is how the arm reaches a 0.90 m
    # counter that would otherwise sit above the shoulder's comfortable cone -- the
    # robot "crouches" to the work.  A real G1 can crouch; this one is held, so the
    # crouch costs nothing and is NOT proof the balancer could hold it.  Labelled.
    pelvis_z: float = 0.80
    right_shoulder_body: str = ""   # "" = autodetect
    out_dir: str = "/workspace/ferox_isaac/mm5_out"


class MM5Runner:
    def __init__(self, art, stage, cfg: MM5Config):
        self.art, self.stage, self.cfg = art, stage, cfg
        if cfg.fix_base:
            cfg.ik_lead_cap = cfg.fix_base_ik_lead
            cfg.ik_max_step = cfg.fix_base_ik_step
        self.pipe = MM5Pipeline(art, stage, cfg)
        self.rng = np.random.default_rng(cfg.seed)
        self.i = 0
        self._rigid = None
        self._home_q = None
        self._base_pose = None
        self._measure_until = 0.0
        self._handcal_until = 0.0
        self._hand_open_snap = {}
        self.hand_geom = None
        self.state = "INIT"
        self._t = 0.0
        os.makedirs(cfg.out_dir, exist_ok=True)
        print(f"[mm5] runner ready: {cfg.trials} trials on '{cfg.object_name}', "
              f"seed {cfg.seed}, cheat_attach={cfg.cheat_attach}", flush=True)

    # ------------------------------------------------------------------ per trial

    def ensure_colliders(self):
        """Give the YCB objects collision geometry. Returns (n_objects, n_meshes).

        THE OBJECTS SHIP WITHOUT COLLIDERS. `tools/build_lab_world.py` applies
        RigidBodyAPI and MassAPI to each YCB prim and never CollisionAPI, so every
        object is a body gravity acts on and nothing can touch: measured in free fall
        from t=0, reaching -43 m after 3 s, which is exactly 0.5*g*t^2. MM2 verified 59
        properties of this world and passed, because MM2 never tried to touch an object.

        Fixed here, at load, rather than in the builder: the objects arrive as
        REFERENCES to the Isaac asset server, and their meshes are not composed into the
        stage at build time, so there is nothing for the builder to attach a collider to.
        By the time the sim has opened the stage they are there. The builder is still the
        better home for this if the references are ever payload-loaded at build time.

        convexHull, not the triangle mesh: PhysX will not simulate a non-convex triangle
        mesh as a DYNAMIC collider. The mug's handle hole is filled in as a result --
        declared, not hidden (C-41).
        """
        from pxr import Usd, UsdGeom, UsdPhysics
        n_obj = n_mesh = 0
        for root in ("/World/Env/objects", "/World/panthera_lab/objects"):
            parent = self.stage.GetPrimAtPath(root)
            if not (parent and parent.IsValid()):
                continue
            for obj in parent.GetChildren():
                got = 0
                for d in Usd.PrimRange(obj):
                    if not d.IsA(UsdGeom.Mesh):
                        continue
                    if not d.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(d)
                    UsdPhysics.MeshCollisionAPI.Apply(d).CreateApproximationAttr().Set(
                        "convexHull")
                    got += 1
                if got:
                    n_obj += 1
                    n_mesh += got
            break
        # GRASP FRICTION, declared. CAMPAIGN 4.4 permits "friction/finger drives tuned
        # within declared bounds and written to TWIN_DEVIATIONS", and this is that: the
        # measured failure is a three-point FINGERTIP contact on a smooth cylinder
        # (Link_14R 6.8 N opposing Link_24R 2.8 and Link_34R 0.5) which slips under lift
        # at any normal force -- 38 N peak was measured against the 3.4 N the can needs.
        # A tip contact has almost no area, so the holding force is friction-limited, not
        # force-limited. Isaac's default material is 0.5/0.5; MM5_GRASP_MU raises the
        # OBJECT's material only, it is logged every run, and it is off by default.
        mu = float(os.environ.get("MM5_GRASP_MU", "0") or 0)
        if mu > 0:
            from pxr import UsdShade, PhysxSchema
            mat_path = "/World/mm5_grasp_material"
            mat = UsdShade.Material.Define(self.stage, mat_path)
            pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            pm.CreateStaticFrictionAttr().Set(mu)
            pm.CreateDynamicFrictionAttr().Set(mu * 0.9)
            pm.CreateRestitutionAttr().Set(0.0)
            bound = 0
            for root in ("/World/Env/objects", "/World/panthera_lab/objects"):
                parent = self.stage.GetPrimAtPath(root)
                if not (parent and parent.IsValid()):
                    continue
                for obj in parent.GetChildren():
                    for d in Usd.PrimRange(obj):
                        if d.IsA(UsdGeom.Mesh) and d.HasAPI(UsdPhysics.CollisionAPI):
                            UsdShade.MaterialBindingAPI.Apply(d).Bind(
                                mat, bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                                materialPurpose="physics")
                            bound += 1
                break
            print(f"[mm5] GRASP FRICTION mu={mu} bound to {bound} object meshes "
                  f"(DECLARED deviation, CAMPAIGN 4.4)", flush=True)
        print(f"[mm5] colliders applied: {n_obj} objects, {n_mesh} meshes "
              f"(convexHull) -- C-41", flush=True)
        return n_obj, n_mesh

    def _place_object(self, seed):
        """Randomize the object on the table and return its new world xyz.

        Uses Isaac's rigid-prim API, not a USD xform write. Two reasons, both learned
        the hard way: the object's xformOp:translate is in its PARENT's frame while the
        pose read back from ComputeLocalToWorldTransform is in the world's, so writing
        one into the other put the can at z = -43 m and it fell out of the world; and
        once physics is running the rigid body owns its transform, so a USD write is
        ignored or fought. set_world_pose + zeroed velocities is the supported path.
        """
        rng = np.random.default_rng(seed)
        if self._rigid is None:
            from isaacsim.core.prims import SingleRigidPrim
            prim = self.pipe._obj_prim(self.cfg.object_name)
            if prim is None:
                return None
            self._rigid = SingleRigidPrim(prim.GetPath().pathString)
            try:
                self._rigid.initialize()
            except Exception:
                pass
        base = np.array(self.home_obj_xyz, float)
        dx, dy = rng.uniform(-self.cfg.randomize_m, self.cfg.randomize_m, size=2)
        newp = np.array([base[0] + dx, base[1] + dy, base[2]], np.float32)
        # ORIENTATION TOO -- but the ORIGINAL one, not identity. Measured: a trial staged
        # at `tilt-at-rest=44.7 deg` because the previous trial knocked the can askew and
        # reset restored only POSITION. Forcing identity instead is wrong for this asset:
        # the YCB can's local +z is not its cylinder axis (an untouched can reads 90 deg
        # against world +z), so identity is not "upright" and cost every contact.
        # The correct reference is the pose the world builder authored, captured once.
        if getattr(self, "_home_obj_quat", None) is None:
            try:
                _, q0 = self._rigid.get_world_pose()
                self._home_obj_quat = np.asarray(q0, np.float32).reshape(-1)[:4].copy()
                print(f"[mm5] object home orientation latched {np.round(self._home_obj_quat,4)}",
                      flush=True)
            except Exception:
                self._home_obj_quat = None
        if getattr(self, "_home_obj_quat", None) is not None:
            self._rigid.set_world_pose(position=newp, orientation=self._home_obj_quat)
        else:
            self._rigid.set_world_pose(position=newp)
        try:
            self._rigid.set_linear_velocity(np.zeros(3, np.float32))
            self._rigid.set_angular_velocity(np.zeros(3, np.float32))
        except Exception:
            pass
        # READ IT BACK. A write that silently does not apply is this campaign's most
        # expensive recurring bug (set_masses needing positional indices; the joint
        # friction rejected by a float32 tolerance; the friction fix behind an env var
        # nobody set), and here it is worse than usual: if the can does not move, every
        # trial reaches for the HOME pose while the log says it is reaching for the
        # randomised one. That produces a stall distance that does not vary with the
        # randomisation -- which is exactly the symptom v3 recorded and could not
        # explain ("the descent stalls at 47-59 mm ... and that number barely moves
        # although the can is randomised over a +-6 cm box").
        got = self.pipe.obj_pose(self.cfg.object_name)
        if got is None:
            print("[mm5] STAGING UNVERIFIED: object pose unreadable after the write",
                  flush=True)
            return newp
        err = float(np.linalg.norm(np.asarray(got, float)[:2] - newp[:2]))
        if err > 0.005:
            print(f"[mm5] ✗ STAGING DID NOT APPLY: asked {np.round(newp,4)}, "
                  f"object is at {np.round(got,4)} ({err*1000:.0f} mm off). The trial "
                  f"would reach for a can that is not there.", flush=True)
        else:
            # TILT AT STAGING, before anything touches it. A tilt metric that reads
            # ~90 deg on an UNTOUCHED, correctly staged can is measuring the wrong axis
            # (the object's local +z need not be its cylinder axis), and would report a
            # topple that never happened. Verify the baseline before trusting the delta.
            t0 = self.pipe.obj_tilt_deg(self.cfg.object_name)
            print(f"[mm5] staged and verified at {np.round(got,4)} "
                  f"({err*1000:.1f} mm), tilt-at-rest="
                  f"{'n/a' if t0 is None else round(t0,1)} deg", flush=True)
        return np.asarray(got, np.float32)

    def _snap_hand(self):
        """World positions of every link at the OPEN pose, via one tensor read."""
        from .dex5_geom import snapshot
        return snapshot(self.pipe)

    def _solve_base_offset(self, obj_xyz):
        """Solve the base placement from the MEASURED shoulder, once, analytically.

        Both earlier variants tuned `base_offset` by hand against the reach error and
        both landed at the edge of the workspace -- v2 reached every pre-grasp but sat
        0.13 m above a can it could not then descend onto.  The reachable set is a cone
        about the SHOULDER, so:

          1. stand the robot at a provisional pose and let physics settle it,
          2. read the right shoulder and the pelvis from PhysX and difference them,
             giving the shoulder in the robot's own frame (lateral, forward, up),
          3. solve the single remaining unknown -- forward distance -- from
                 target_r^2 = lateral^2 + forward^2 + vertical^2
             where vertical is fixed by the surface height and the pelvis height.

        If the vertical term alone already exceeds target_r the can is simply too high
        or too low for the cone at this pelvis height, and the solve says so instead of
        quietly returning a complex root -- which is exactly the information the
        counter attempt lacked when it was iterating.
        """
        yaw = self.cfg.base_yaw
        # Robot forward is +x in its own frame, so world forward is R(yaw) @ [1,0].
        # The first cut had these two swapped -- [-sin, cos] is the robot's LEFT, not
        # its forward -- which put the solved base beside the counter instead of in
        # front of it. Worth stating because the error is invisible in the output:
        # both are unit vectors and the solve still "converges", just to the wrong pose.
        fwd = np.array([np.cos(yaw), np.sin(yaw)])       # robot +x (forward) in world
        right = np.array([np.sin(yaw), -np.cos(yaw)])    # robot -y (right)   in world

        name = self.cfg.right_shoulder_body or self.pipe.find_body([
            "right_shoulder_yaw_link", "right_shoulder_roll_link",
            "right_shoulder_pitch_link", "right_shoulder_link"])
        if name is None:
            print("[mm5][v3] no right-shoulder link found; keeping the tuned offset",
                  flush=True)
            return None
        sh_w, _ = self.pipe.body_pose(name)
        base_w, _ = self.art.get_world_pose()
        d = np.asarray(sh_w, float)[:2] - np.asarray(base_w, float)[:2]
        s_fwd, s_lat = float(d @ fwd), float(d @ right)
        s_up = float(sh_w[2] - base_w[2])

        # Lateral offset is measured from the SHOULDER, not the pelvis, for the same
        # reason the whole solve is: the cone is about the shoulder. v3's first run
        # took it from the pelvis, and since the shoulder sits 0.178 m to the right of
        # the pelvis a "0.10 m to the robot's right" can landed 0.078 m to the LEFT of
        # the shoulder. The arm answered by winding shoulder_roll to 1.538 -- its
        # clamped stop -- and the reach froze at exactly 105 mm and stayed there.
        vert = float(obj_xyz[2]) - (self.cfg.pelvis_z + s_up)
        lat = self.cfg.lateral_offset
        rem = self.cfg.target_r ** 2 - vert ** 2 - lat ** 2
        print(f"[mm5][v3] shoulder '{name}' in the robot frame: forward {s_fwd:+.4f} "
              f"right {s_lat:+.4f} up {s_up:+.4f} (pelvis at {self.cfg.pelvis_z:.2f})",
              flush=True)
        print(f"[mm5][v3] can at z={float(obj_xyz[2]):.4f} -> vertical {vert:+.4f} m, "
              f"lateral {lat:+.4f} m, target_r {self.cfg.target_r:.3f} m", flush=True)
        if rem <= 0.0:
            print(f"[mm5][v3] UNREACHABLE at this pelvis height: the vertical and "
                  f"lateral terms alone are {np.hypot(vert, lat):.4f} m, which already "
                  f"exceeds target_r {self.cfg.target_r:.3f}. Lower `pelvis_z` (the base "
                  f"is rig-held, so crouching to the work is free) or raise target_r.",
                  flush=True)
            return None
        forward = s_fwd + float(np.sqrt(rem))
        clamped = float(np.clip(forward, self.cfg.min_forward, self.cfg.max_forward))
        if clamped != forward:
            print(f"[mm5][v3] solved forward {forward:.4f} clamped to {clamped:.4f} "
                  f"[{self.cfg.min_forward}, {self.cfg.max_forward}]", flush=True)
        off = -clamped * fwd - (s_lat + self.cfg.lateral_offset) * right
        print(f"[mm5][v3] SOLVED base_offset {np.round(off,4).tolist()} -- the can is "
              f"{self.cfg.target_r:.3f} m from the shoulder, {clamped:.3f} m forward of "
              f"the pelvis (v2 used 0.38 m and stalled at the workspace edge)",
              flush=True)
        return (float(off[0]), float(off[1]))

    def _stand_robot(self, obj_xyz):
        """Reset the robot to a clean standing state at the reach pose for this object.

        A full reset, not a teleport. Moving only the base left trial 2 starting from
        wherever trial 1 ended -- which, once trial 1 toppled, meant every subsequent
        trial began face-down and failed in APPROACH within half a second. Orientation,
        joint positions and both velocity fields all have to go back, or the trials are
        not independent samples and the success rate means nothing.
        """
        x = float(obj_xyz[0] + self.cfg.base_offset[0])
        y = float(obj_xyz[1] + self.cfg.base_offset[1])
        yaw = self.cfg.base_yaw
        quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], np.float32)
        # spawn_z is also the "robot is down" threshold, so the held-base pelvis
        # height is its own knob rather than a reuse of that one.
        z = self.cfg.pelvis_z if self.cfg.fix_base else self.cfg.spawn_z
        pos = np.array([x, y, z], np.float32)
        self.art.set_world_pose(pos, quat)
        self.art.set_linear_velocity(np.zeros(3, np.float32))
        self.art.set_angular_velocity(np.zeros(3, np.float32))
        if self._home_q is None:
            self._home_q = np.asarray(self.art.get_joint_positions(), np.float32).copy()
        self.art.set_joint_positions(self._home_q)
        self.art.set_joint_velocities(np.zeros_like(self._home_q))
        self._base_pose = (pos, quat)
        if self.cfg.fix_base:
            self.pipe._body_hold = self._home_q[self.pipe.body_idx].astype(np.float32)
        self.pipe._arm_target = None
        self.pipe._grip_gains(False)
        self.pipe._grip_ratio = 0.0
        self.pipe._hand_target = np.zeros(len(self.pipe._hand_target), np.float32)
        return pos

    # ---------------------------------------------------------------------- step

    def step(self, dt):
        self._t += dt
        if self.cfg.fix_base and self._base_pose is not None:
            pos, quat = self._base_pose
            self.art.set_world_pose(pos, quat)
            self.art.set_linear_velocity(np.zeros(3, np.float32))
            self.art.set_angular_velocity(np.zeros(3, np.float32))
        if self.state == "INIT":
            self.ensure_colliders()
            p = self.pipe.obj_pose(self.cfg.object_name)
            if p is None:
                print(f"[mm5] FATAL: object '{self.cfg.object_name}' not in the stage",
                      flush=True)
                self.state = "DONE"
                return
            if self.cfg.surface == "counter":
                half = float(p[2]) - 0.75          # the can's half-height above the table
                self.home_obj_xyz = np.array(
                    [self.cfg.counter_xy[0], self.cfg.counter_xy[1],
                     self.cfg.counter_h + half + 0.002], float)
                self.cfg.base_offset = (self.cfg.base_offset[0], -abs(self.cfg.base_offset[1]))
                self.cfg.base_yaw = 1.5708         # face +y, toward the counter
                print(f"[mm5] staging object on the COUNTER (0.90 m) at "
                      f"{np.round(self.home_obj_xyz,4)} (was {np.round(p,4)}) -- the "
                      f"table at 0.75 m is below this arm's workspace", flush=True)
            else:
                self.home_obj_xyz = np.array([p[0], self.cfg.stage_y, p[2]], float)
                print(f"[mm5] staging object at the table's near edge: "
                      f"{np.round(self.home_obj_xyz,4)} (was {np.round(p,4)})", flush=True)
            # Snapshot the settled standing pose now, before any trial disturbs it.
            self._home_q = np.asarray(self.art.get_joint_positions(), np.float32).copy()
            # Print the STAGED pose, not the pre-staging one. This line printed `p`
            # -- the object's position before staging moved it -- so the log showed
            # y=-1.476 while the trials randomised around y=-1.31, and the two numbers
            # in the same log looked like a bug in the randomiser rather than a
            # mislabelled print.
            print(f"[mm5] home object pose (staged) {np.round(self.home_obj_xyz,4)} "
                  f"(pre-staging {np.round(p,4)})", flush=True)
            if self.cfg.measure_hand:
                # HANDCAL before anything else: the stand-off the base solve needs is a
                # property of the HAND, and measuring it costs one close-and-open.
                self._stand_robot(self.home_obj_xyz)
                self._hand_open_snap = self._snap_hand()
                self.pipe._set_hand(1.0)
                self._handcal_until = self._t + max(self.cfg.close_s, 2.0) + 1.0
                self.state = "HANDCAL"
                return
            if self.cfg.place_from_workspace:
                # Stand once at the tuned offset purely so the shoulder can be MEASURED
                # in a settled pose; the solve then replaces that offset for every trial.
                self._stand_robot(self.home_obj_xyz)
                # Place the object once here too. On the first v3 run trial 1 read the
                # can 7 m away for its whole 36 s -- still on the table it had been
                # staged off -- because the rigid prim is resolved on first use and the
                # very first set_world_pose did not take. Trials 2..N were fine. One
                # warm-up placement, with the MEASURE settle giving physics time to
                # apply it, costs nothing and makes trial 1 a real sample.
                self._place_object(self.cfg.seed)
                self._measure_until = self._t + 0.5
                self.state = "MEASURE"
            else:
                self.state = "NEXT"
            return

        if self.state == "HANDCAL":
            if self._t < self._handcal_until:
                return
            from .dex5_geom import measure
            g = measure(self.pipe, self._hand_open_snap)
            self.pipe._set_hand(0.0)
            if g is None:
                print("[mm5][v4] hand geometry NOT measurable; keeping the argued "
                      f"stand-off {self.cfg.grasp_standoff}", flush=True)
            else:
                self.hand_geom = g
                print(f"[mm5][v4] Dex5 closed-pose geometry, palm frame: "
                      f"{g['n_moved']} of {g['n_hand_bodies']} hand bodies moved",
                      flush=True)
                for t in g["tips"]:
                    print(f"[mm5][v4]   {t['body']:<16} palm_xyz={t['palm_xyz']} "
                          f"travel={t['travel_m']:.4f} m", flush=True)
                print(f"[mm5][v4] fingers close about {g['closed_centre_palm']} "
                      f"= {g['centre_dist_m']:.4f} m from the palm origin, mean tip "
                      f"spread {g['tip_spread_m']:.4f} m", flush=True)
                # THE STAND-OFF, measured. The object's axis has to sit where the
                # fingers converge, so the palm must be that far from it -- plus a
                # little, because the fingers have to pass the widest part of the
                # object before they close behind it.
                so = float(g["centre_dist_m"]) + self.cfg.grasp_clearance
                print(f"[mm5][v4] grasp_standoff {self.cfg.grasp_standoff:.4f} -> "
                      f"{so:.4f} m (measured centre + {self.cfg.grasp_clearance:.3f} "
                      f"clearance), replacing an argued number with a measured one",
                      flush=True)
                self.cfg.grasp_standoff = so
                self.cfg.pregrasp_standoff = so + self.cfg.pregrasp_extra
                # AND the axis. The stand-off was the visible half of the error; this
                # is the half that made every previous grasp close on air -- the IK was
                # aligning the palm's +z while the fingers close along its +y.
                self.pipe._hand_body_names = set(g.get("hand_bodies", []))
                print(f"[mm5][v5] hand link set for contact counting: "
                      f"{len(self.pipe._hand_body_names)} links", flush=True)
                c = np.asarray(g["closed_centre_palm"], float)
                self.pipe.grasp_axis_local = c / max(float(np.linalg.norm(c)), 1e-9)
                print(f"[mm5][v4] grasp axis in the palm frame: "
                      f"{np.round(self.pipe.grasp_axis_local, 4).tolist()} "
                      f"(v2/v3 constrained palm +z = [0,0,1] -- the wrong axis)",
                      flush=True)
            if self.cfg.place_from_workspace:
                self._stand_robot(self.home_obj_xyz)
                self._place_object(self.cfg.seed)
                self._measure_until = self._t + 0.5
                self.state = "MEASURE"
            else:
                self.state = "NEXT"
            return

        if self.state == "MEASURE":
            if self._t < self._measure_until:
                return
            off = self._solve_base_offset(self.home_obj_xyz)
            if off is not None:
                self.cfg.base_offset = off
            else:
                print("[mm5][v3] falling back to the tuned base_offset "
                      f"{self.cfg.base_offset}", flush=True)
            self.state = "NEXT"
            return

        if self.state == "NEXT":
            if self.i >= self.cfg.trials:
                self._write()
                self.state = "DONE"
                return
            self.i += 1
            seed = int(self.cfg.seed + self.i)
            obj = self._place_object(seed)
            self._stand_robot(obj)
            self.pipe.sim_time = self._t
            # Pass the COMMANDED placement, do not re-read the pose.
            #
            # set_world_pose does not take effect until physics steps, so reading the
            # object back here returns the PREVIOUS trial's end pose. When that trial had
            # knocked the can onto the floor, obj_start came back at z = 0.032 and the
            # LIFT check -- obj_z - obj_start_z > 0.05 -- passed in a single step on a can
            # that had never been touched. It produced two false SUCCESS rows out of 20.
            # The runner knows exactly where it put the object; it should say so.
            self.pipe.start_trial(self.i, seed, self.cfg.object_name, obj_start=obj)
            self.state = "RUN"
            return

        if self.state == "RUN":
            self.pipe.step(dt)
            if self.pipe.trial is None:
                self.state = "NEXT"

    def reapply(self):
        if self.state == "RUN":
            self.pipe.reapply()

    # -------------------------------------------------------------------- output

    def _write(self):
        rows = [t.as_row() for t in self.pipe.results]
        n = len(rows)
        ok = sum(1 for r in rows if r["outcome"] == "SUCCESS")
        tax = {}
        for r in rows:
            tax[r["outcome"]] = tax.get(r["outcome"], 0) + 1
        summary = {"trials": n, "successes": ok,
                   "success_rate": round(ok / n, 4) if n else 0.0,
                   "taxonomy": tax, "config": asdict(self.cfg), "rows": rows}
        with open(os.path.join(self.cfg.out_dir, "mm5_results.json"), "w") as fh:
            json.dump(summary, fh, indent=2)

        # Joint trace, one JSON object per line, 50 Hz. Media is deferred to the 4090
        # day (C-23); this is what lets that day replay the episode for camera instead
        # of re-running twenty trials of physics to find a good one.
        tp = os.path.join(self.cfg.out_dir, "mm5_trace.jsonl")
        with open(tp, "w") as fh:
            for rec in self.pipe._trace:
                fh.write(json.dumps(rec) + "\n")
        print(f"[mm5] joint trace: {len(self.pipe._trace)} samples -> {tp}", flush=True)

        lines = [f"# MM5 results — {n} trials, {self.cfg.object_name}", "",
                 f"**Balancer: omni locomotion policy** (SONIC parked at C-39; "
                 f"CAMPAIGN 4.4 permits \"or omni standing if MM4 slips\").", "",
                 f"**Success rate: {ok}/{n} = {100*ok/n:.1f}%**" if n else "no trials", "",
                 "| outcome | count |", "|---|---|"]
        for k, v in sorted(tax.items(), key=lambda kv: -kv[1]):
            lines.append(f"| `{k}` | {v} |")
        lines += ["", "| trial | seed | outcome | detail | obj start | obj end | stage times (s) |",
                  "|---|---|---|---|---|---|---|"]
        for r in rows:
            st = " ".join(f"{k}={v}" for k, v in r["stage_s"].items())
            lines.append(f"| {r['trial']} | {r['seed']} | `{r['outcome']}` | {r['detail']} | "
                         f"{r['obj_start']} | {r['obj_end']} | {st} |")
        with open(os.path.join(self.cfg.out_dir, "mm5_results.md"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
        with open(os.path.join(self.cfg.out_dir, "mm5_log.txt"), "w") as fh:
            fh.write("\n".join(self.pipe._log) + "\n")
        print(f"[mm5] WROTE {self.cfg.out_dir}/mm5_results.{{json,md}} — "
              f"{ok}/{n} success", flush=True)
