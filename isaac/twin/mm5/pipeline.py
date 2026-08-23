"""MM5: scripted approach -> IK reach -> Dex5 grasp -> lift -> carry -> place.

Runs inside Isaac, stepped from RobotRosRunner.on_physics_step, as a state machine so
every stage is separately timed and separately blameable -- CAMPAIGN 4.4 asks for a
taxonomy, and a taxonomy is only worth having if a failure lands in exactly one bucket.

WHO IS BALANCING. The omni locomotion policy, not SONIC: MM4 is parked at C-39 and
CAMPAIGN 4.4 explicitly allows "or omni standing if MM4 slips -- say which". This says
which. The seam for SONIC is already built and is not this file -- it is
G1_CONTROL=handoff in run.py, which hands the whole articulation over once. When C-39
closes, MM5 runs unchanged behind that switch.

WHO OWNS THE ARM. The policy drives all 29 body joints, so this pipeline does not fight
it: after the policy has written its action, the seven right-arm joints are overwritten
with the IK solution through the articulation VIEW. Legs and torso stay the policy's;
the arm and the fingers are MM5's. That split is the same one MM4's POSE mode uses, and
it is why "arms-only while balancing" is a thing the twin can do at all.

NO CHEAT-ATTACH. The object is picked up by closing fingers on it and holding it with
friction. `--cheat-attach` exists, is off by default, and any trial run with it is
reported on its own separately-labelled row -- it is plumbing proof, never a success.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from .ik import dls_step, pose_error, topdown_quat, approach_quat

# SDK 22..28 -- the right arm. Named, never sliced: Isaac interleaves the hands through
# the body joints (RULE-HAND-NAME) and an index slice here would grab fingers.
RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
# The Dex5 right hand, official URDF document order. Index 12 is Roll_41R on the right
# and Link_41L on the left -- the hands are not name-symmetric.
RIGHT_HAND_JOINTS = [
    "Yaw_11R", "Roll_12R", "Pitch_13R", "Pitch_14R", "Roll_21R", "Pitch_22R",
    "Pitch_23R", "Pitch_24R", "Roll_31R", "Pitch_32R", "Pitch_33R", "Pitch_34R",
    "Roll_41R", "Pitch_42R", "Pitch_43R", "Pitch_44R", "Roll_51R", "Pitch_52R",
    "Pitch_53R", "Pitch_54R",
]
# Mechanically unactuated on the Dex5-1 (C-13). Commanded closed like everything else
# would be a lie about the hand, so they are held at 0.
PASSIVE_HAND_IDX = (4, 8, 12, 16)
RIGHT_PALM_BODY = "base_link00"      # left is base_link00L; the asymmetry is upstream

# Right-arm joint limits, SDK 22..28, from the driver's own g1_29dof.urdf. The solver
# needs them: without clamping, DLS wound right_shoulder_yaw into its -2.618 stop and
# held it there while the remaining joints oscillated, so the reach error bounced
# 457..513 mm instead of converging. A differential solver has no idea a joint has ended.
ARM_Q_MIN = np.array([-3.0892, -2.2515, -2.618, -1.0472, -1.9722, -1.6144, -1.6144])
ARM_Q_MAX = np.array([ 2.6704,  1.5882,  2.618,  2.0944,  1.9722,  1.6144,  1.6144])
ARM_MARGIN = 0.05                    # stay this far off the stop
# Posture the null space pulls toward: a natural reaching stance, mid-range on every
# joint, so the solver has room to move in both directions.
ARM_NOMINAL = np.array([0.30, -0.25, 0.0, 0.97, 0.0, 0.0, 0.0])

# SDK 0..21: both legs, waist, and the LEFT arm -- everything MM5 does not drive.
SDK_BODY_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

STAGES = ["APPROACH", "REACH", "DESCEND", "GRASP", "LIFT", "CARRY", "PLACE", "RELEASE", "RETREAT"]


@dataclass
class Trial:
    index: int
    seed: int
    object_name: str
    obj_start: tuple = (0.0, 0.0, 0.0)
    obj_end: tuple = (0.0, 0.0, 0.0)
    stage_times: dict = field(default_factory=dict)
    outcome: str = "RUNNING"
    detail: str = ""
    cheat_attach: bool = False
    grip_contacts: int = -1
    obj_tilt_deg: float = None
    grip_force_n: float = 0.0

    def as_row(self) -> dict:
        return {"trial": self.index, "seed": self.seed, "object": self.object_name,
                "outcome": self.outcome, "detail": self.detail,
                "cheat_attach": self.cheat_attach,
                "obj_start": [round(v, 4) for v in self.obj_start],
                "obj_end": [round(v, 4) for v in self.obj_end],
                "stage_s": {k: round(v, 3) for k, v in self.stage_times.items()},
                "grip_contacts": self.grip_contacts,
                "obj_tilt_deg": self.obj_tilt_deg,
                "grip_force_n": self.grip_force_n}


class MM5Pipeline:
    """One trial at a time; call step() from the physics callback."""

    def __init__(self, art, stage, cfg):
        from pxr import UsdGeom  # noqa: F401  (imported for the caller's benefit)

        self.art = art
        self.stage = stage
        self.cfg = cfg
        names = list(art.dof_names)
        self.arm_idx = np.array([names.index(n) for n in RIGHT_ARM_JOINTS], np.int32)
        self.hand_idx = np.array([names.index(n) for n in RIGHT_HAND_JOINTS
                                  if n in names], np.int32)
        self.has_hand = len(self.hand_idx) == len(RIGHT_HAND_JOINTS)
        try:
            self.palm_body = list(art._articulation_view.body_names).index(RIGHT_PALM_BODY)
        except (ValueError, AttributeError):
            self.palm_body = None
        self.view = art._articulation_view

        self.trial: Trial | None = None
        self.stage_name = "IDLE"
        self._t_stage = 0.0
        self.sim_time = 0.0
        self.results: list[Trial] = []
        self._arm_target = None
        self._hand_target = np.zeros(len(RIGHT_HAND_JOINTS), np.float32)
        self._grip_ratio = 0.0
        self._log = []
        self._trace = []
        self._trace_next = 0.0
        self._axis_cache = None
        # Palm-frame direction the fingers close along. +z is the WRONG axis for the
        # Dex5 and is only the default so this runs before HANDCAL has measured it.
        self.grasp_axis_local = np.array([0.0, 0.0, 1.0])
        self._hand_body_names = set()
        self._cs = None
        self._creporter = None
        # Legs + torso: every SDK joint that is not the right arm.
        self.body_idx = np.array(
            [names.index(n) for n in SDK_BODY_JOINTS if n in names], np.int32)
        self._body_hold = None
        self._palm_prim = None
        # FIXED BASE: stiffen the right arm. deploy.yaml gives it kp 40, which is right
        # for a robot that must not be thrown off balance by its own arm -- but it also
        # means the arm sits ~0.1 rad behind its target under gravity with a kilo on the
        # wrist, and that lag IS the servo problem: a small lead cap is entirely consumed
        # by it (reach stalls at 350 mm), and a large one closes the loop through the lag
        # and limit-cycles (326..443 mm). With the base held there is nothing to
        # destabilise, so the arm is stiffened and the lag goes away.
        if getattr(cfg, "fix_base", False):
            try:
                kp = np.full(len(self.arm_idx), cfg.fix_base_arm_kp, np.float32)
                kd = np.full(len(self.arm_idx), cfg.fix_base_arm_kd, np.float32)
                self.view.set_gains(kp, kd, joint_indices=self.arm_idx)
                print(f"[mm5] fixed-base: right-arm gains -> kp={cfg.fix_base_arm_kp} "
                      f"kd={cfg.fix_base_arm_kd} (deploy.yaml gives 40/1)", flush=True)
            except Exception as exc:
                print(f"[mm5] arm gain set failed: {exc!r}", flush=True)

    # ------------------------------------------------------------------ helpers

    def _surface_h(self) -> float:
        return (self.cfg.counter_h if getattr(self.cfg, "surface", "table") == "counter"
                else self.cfg.table_h)

    def _obj_prim(self, name):
        from pxr import UsdGeom, Usd
        for root in ("/World/Env/objects", "/World/panthera_lab/objects"):
            p = self.stage.GetPrimAtPath(f"{root}/{name}")
            if p and p.IsValid():
                return p
        return None

    def obj_pose(self, name):
        """World position of an object, from PHYSICS -- the same trap as palm_pose().

        This used ComputeLocalToWorldTransform, and that is exactly the defect the palm
        docstring above describes: PhysX writes to Fabric and does NOT write back to
        USD, so the USD route returns the AUTHORED pose and never moves. It was fixed
        for the palm and left in place for the OBJECT, where it is worse -- the object
        is the thing the whole pipeline servos toward.

        What it cost: `stage_object()` moves the can to the table's near edge (the far
        pose is outside this arm's workspace) and randomises it over a +-6 cm box, all
        in physics. Every reader here saw the authored pose regardless, 138 mm away and
        166 mm further out than the staged one, so every trial in v2..v6 reached for a
        can that was not there and stalled short. It also explains the one symptom v3
        recorded and could not account for: a stall distance that "barely moves although
        the can is randomised over a +-6 cm box" -- the measurement never moved, so the
        stall could not.

        One SingleRigidPrim per OBJECT, created once and cached. Never one per body:
        constructing 79 of those mid-run invalidates the physics view outright.
        """
        if not hasattr(self, "_obj_rigids"):
            self._obj_rigids = {}
        rp = self._obj_rigids.get(name)
        if rp is None:
            prim = self._obj_prim(name)
            if prim is None:
                return None
            try:
                from isaacsim.core.prims import SingleRigidPrim
                rp = SingleRigidPrim(prim.GetPath().pathString)
                try:
                    rp.initialize()
                except Exception:
                    pass
                self._obj_rigids[name] = rp
            except Exception as exc:
                self.log(f"[mm5] object physics handle unavailable ({exc!r}); "
                         f"falling back to the USD pose, WHICH DOES NOT MOVE")
                self._obj_rigids[name] = False
                rp = False
        if rp is not False:
            try:
                pos, _ = rp.get_world_pose()
                return np.asarray(pos, float)
            except Exception as exc:
                # DO NOT fall back to the USD pose here. It is the authored pose, it
                # never moves, and returning it silently is precisely the bug this
                # function was just fixed for: trial 1 staged and verified on the
                # counter at [-2.655, 2.211] and then reached toward [1.771, -1.476],
                # the table, 4.86 m away, because get_world_pose() raised while the
                # physics view was still warming and the fallback answered instead.
                # None makes the caller fail the trial as HARNESS, which is visible.
                if not getattr(self, "_objpose_warned", False):
                    self._objpose_warned = True
                    self.log(f"[mm5] object physics pose unreadable ({exc!r}) -- "
                             f"returning None rather than the authored USD pose, "
                             f"which does not move")
                return None
        from pxr import UsdGeom, Usd
        p = self._obj_prim(name)
        if p is None:
            return None
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        return np.array([t[0], t[1], t[2]], float)

    def obj_tilt_deg(self, name):
        """Angle the object has rotated since staging, in degrees, or None.

        QUATERNION ANGLE, not the tilt of a body axis. Preflight (GAUGE 2) caught the
        axis version under-reporting a scripted 30 deg tip as 18.45 deg: the YCB can's
        local +z is not its cylinder axis, so rotating about world +x moves that axis by
        less than the rotation. The quaternion angle is independent of the mesh's frame
        convention and reads a known 30 deg tip as 30 deg.

        The taxonomy could not see a topple at all before this: a can that tips stays on
        the counter, never drops below `surface - 0.05`, and was reported as NO_GRIP.
        """
        rp = getattr(self, "_obj_rigids", {}).get(name)
        if rp is None or rp is False:
            return None
        base = getattr(self, "_obj_axis0", {}).get(name)
        if base is None:
            return 0.0
        try:
            _, q = rp.get_world_pose()
            q = np.asarray(q, float).reshape(-1)[:4]
            rw = float(np.dot(q, np.asarray(base, float)))
            return float(np.degrees(2.0 * np.arccos(min(1.0, abs(rw)))))
        except Exception:
            return None

    def latch_obj_axis(self, name):
        """Record the object's at-rest axis so tilt can be measured as a CHANGE."""
        if not hasattr(self, "_obj_axis0"):
            self._obj_axis0 = {}
        if name in self._obj_axis0:
            # LATCH ONCE. Re-latching every trial made `tilt-at-rest` read 0.0 by
            # construction, which destroyed the very check that caught a can staged at
            # 44.7 deg. The reference is the object's authored upright pose, captured the
            # first time it is staged and never overwritten.
            return self._obj_axis0[name]
        rp = getattr(self, "_obj_rigids", {}).get(name)
        if rp is None or rp is False:
            return None
        try:
            _, q = rp.get_world_pose()
            q = np.asarray(q, float).reshape(-1)[:4].copy()
            self._obj_axis0[name] = q          # rest QUATERNION, the tilt reference
            return q
        except Exception:
            return None

    def palm_pose(self):
        """World pose of the right palm, (pos, quat_wxyz), from PHYSICS.

        Via SingleRigidPrim, not UsdGeom.ComputeLocalToWorldTransform. The USD route is
        the trap that cost this gate a day: PhysX writes link transforms to Fabric and
        does NOT write them back to USD, so ComputeLocalToWorldTransform returns the
        AUTHORED pose and never moves. The palm read a constant z = 0.706 while the arm
        was visibly being driven, the IK servoed against a frozen measurement, and the
        reach stalled at a repeatable 350 mm -- with a larger gain making it worse,
        because a stuck sensor plus more authority is just a bigger wrong step.

        `get_body_poses()` would have been the obvious API and does not exist on this
        Articulation; the fallback it forced is what was wrong, not the intent.
        """
        if self._palm_prim is None:
            from isaacsim.core.prims import SingleRigidPrim
            self._palm_prim = SingleRigidPrim(f"{self.cfg.robot_root}/{RIGHT_PALM_BODY}")
            try:
                self._palm_prim.initialize()
            except Exception:
                pass
        pos, quat = self._palm_prim.get_world_pose()
        return np.asarray(pos, float), np.asarray(quat, float)

    def body_pose(self, name: str):
        """World pose of any robot link, (pos, quat_wxyz), from PHYSICS.

        The generic form of `palm_pose`, and for the same reason -- PhysX writes link
        transforms to Fabric, not back to USD, so the UsdGeom route returns the
        authored pose and never moves.  v3 needs the right SHOULDER, because the base
        placement is now computed from where the shoulder actually is instead of being
        found by iterating on the reach error.
        """
        from isaacsim.core.prims import SingleRigidPrim
        if not hasattr(self, "_body_prims"):
            self._body_prims = {}
        if name not in self._body_prims:
            # RESOLVE the prim path, never assume `robot_root/name`. Links are nested
            # at whatever depth the URDF import produced, and constructing a
            # SingleRigidPrim on a path that does not exist does not merely fail --
            # it took the ARTICULATION VIEW down with it ("Provided pattern list did
            # not match any articulations"), after which set_world_pose raised and the
            # run was over. A wrong path here is not a missing measurement, it is a
            # corrupted sim.
            path = self._body_path(name)
            if path is None:
                raise KeyError(f"no prim for body {name!r}")
            pr = SingleRigidPrim(path)
            try:
                pr.initialize()
            except Exception:
                pass
            self._body_prims[name] = pr
        pos, quat = self._body_prims[name].get_world_pose()
        return np.asarray(pos, float), np.asarray(quat, float)

    def _body_path(self, name: str):
        """Full prim path for a body name, from one stage walk, cached."""
        if not hasattr(self, "_body_paths"):
            from pxr import Usd
            self._body_paths = {}
            root = self.stage.GetPrimAtPath(self.cfg.robot_root)
            if root and root.IsValid():
                for pr in Usd.PrimRange(root):
                    self._body_paths.setdefault(pr.GetName(), pr.GetPath().pathString)
        return self._body_paths.get(name)

    def find_body(self, candidates):
        """First of `candidates` that exists on this articulation, or None."""
        try:
            names = list(self.art._articulation_view.body_names)
        except Exception:
            return None
        for c in candidates:
            if c in names:
                return c
        return None

    def finger_contacts(self):
        """(n_in_contact, total |F|, per-finger |F|) for the right hand's links.

        v4 fixed WHERE the hand closes; the hand still closes fully and does not hold.
        Before adding force there is a prior question -- are the fingers touching the
        object at all, or closing through it? Zero contact at full closure would mean
        the finger COLLIDERS are missing, which is a different bug entirely and one
        this campaign has already hit once on the YCB objects (C-41).

        Same net-contact-force route as the foot audit: one tensor read, no per-body
        prim wrappers. Constructing those mid-run invalidates the physics view.
        """
        # ROUTE C (v7, preferred): the PhysX contact-report callback. No sensor prim
        # and no re-parenting -- the two things that defeated v6 -- and it answers the
        # question v5/v6 could not: are the fingers touching the object at all.
        if self._creporter is not None and self._creporter.available:
            got = self._creporter.counts()
            if got is not None:
                return got

        # ROUTE A: ContactSensor prims, the only working route in this build.
        paths = list(getattr(self.cfg, "contact_sensor_paths", ()) or ())
        if paths:
            if self._cs is None:
                from isaacsim.sensors.physics import ContactSensor
                self._cs = []
                for p in paths:
                    try:
                        s = ContactSensor(prim_path=p, name=f"cs_{len(self._cs)}")
                        s.initialize()
                        self._cs.append(s)
                    except Exception as exc:
                        self.log(f"contact sensor {p} unusable: {exc!r}")
            mags = []
            for s in self._cs:
                try:
                    fr = s.get_current_frame()
                except Exception:
                    continue
                f = float(fr.get("force", 0.0) or 0.0)
                mags.append(f)
            if mags:
                return sum(1 for m in mags if m > 0.05), float(sum(mags)), mags

        view = self.view
        try:
            names = list(view.body_names)
        except Exception:
            return None
        F = None
        tried = []
        for hn, holder in (("view", view), ("physx", getattr(view, "_physics_view", None))):
            if holder is None:
                continue
            fn = getattr(holder, "get_net_contact_forces", None)
            if fn is None:
                tried.append(f"{hn}: absent")
                continue
            try:
                A = np.asarray(fn())
                F = A.reshape(-1, len(names), A.shape[-1])[0][:, :3]
                tried.append(f"{hn}: OK {A.shape}")
                break
            except Exception as exc:
                tried.append(f"{hn}: {type(exc).__name__}")
                F = None
        if F is None:
            if not getattr(self, "_contact_api_warned", False):
                self._contact_api_warned = True
                self.log(f"NO net-contact-force API: {tried}. Finger contact cannot be "
                         f"counted, so the LIFT gate is inactive and any NO_GRIP row "
                         f"below is NOT evidence about colliders.")
            return None
        mags = []
        for i, n in enumerate(names):
            if n in self._hand_body_names:
                mags.append(float(np.linalg.norm(F[i])))
        if not mags:
            return None
        return sum(1 for m in mags if m > 0.5), float(sum(mags)), mags

    def _approach(self, obj):
        """(axis, pregrasp, graspose) for this target, computed from the geometry.

        The axis is the direction the PALM travels to close on the can, and it is
        chosen by where the can sits relative to the SHOULDER rather than fixed:

          * can well below the shoulder (a 0.75 m table) -> straight down, which is
            what v2 measured and what that geometry wants;
          * can near shoulder height (the 0.90 m counter) -> horizontal, in from the
            robot, because a top-down grasp at shoulder height asks the elbow to climb
            over the hand and the measured palm z at that pre-grasp came out nearly
            horizontal AND pointing away from the can.

        Blended across the band between so there is no discontinuity mid-reach. The
        stand-off distances are the same numbers v2 used, just applied along this axis
        instead of always along world -z.
        """
        # FROZEN PER TRIAL. The axis depends on the SHOULDER, and the shoulder link
        # moves as the arm moves -- so recomputing it every step puts the target in a
        # feedback loop with the thing chasing it. Observed directly: a reach sitting
        # at 164 mm walked steadily out to 1426 mm over four seconds with the shoulder
        # yaw winding to -2.0 rad. The geometry that picks the axis is the geometry at
        # the START of the reach, when the arm is still in its settled posture.
        if self._axis_cache is not None:
            return self._axis_cache
        obj = np.asarray(obj, float)
        sh = self._shoulder_w()
        if sh is None:
            a = np.array([0.0, 0.0, -1.0])
        else:
            drop = float(sh[2] - obj[2])          # how far the can is BELOW the shoulder
            horiz = obj[:2] - sh[:2]
            n = float(np.linalg.norm(horiz))
            h = np.array([horiz[0]/n, horiz[1]/n, 0.0]) if n > 1e-6 else np.array([1.0, 0, 0])
            # 1.0 at drop >= 0.25 m (table), 0.0 at drop <= 0.10 m (counter).
            w = float(np.clip((drop - 0.10) / 0.15, 0.0, 1.0))
            a = w * np.array([0.0, 0.0, -1.0]) + (1.0 - w) * h
            a = a / max(float(np.linalg.norm(a)), 1e-9)
        pre = obj - a * float(self.cfg.pregrasp_standoff)
        gr = obj - a * float(self.cfg.grasp_standoff)
        self._axis_cache = (a, pre, gr)
        return self._axis_cache

    def _shoulder_w(self):
        name = getattr(self, "_sh_name", None)
        if name is None:
            name = self.find_body(["right_shoulder_yaw_link", "right_shoulder_roll_link",
                                   "right_shoulder_pitch_link", "right_shoulder_link"])
            self._sh_name = name or ""
        if not self._sh_name:
            return None
        return self.body_pose(self._sh_name)[0]

    def arm_jacobian(self):
        """(6, 7) task Jacobian for the right palm w.r.t. the right-arm joints."""
        J = np.asarray(self.view.get_jacobians())[0]      # (nbody, 6, 6+ndof)
        Jb = J[self.palm_body]                            # (6, 6+ndof)
        return Jb[:, 6 + self.arm_idx]                    # floating base occupies 0..5

    def log(self, msg):
        line = f"[mm5] t={self.sim_time:7.2f} {self.stage_name:<8} {msg}"
        print(line, flush=True)
        self._log.append(line)

    # -------------------------------------------------------------------- stages

    def _attach_contact_reporter(self, object_name):
        """Attach the v7 contact-report route to THIS trial's object. Once per object.

        Finger link prim paths come from one stage walk against the articulation's own
        body names -- never from per-body prim wrappers, which invalidate the running
        physics view.
        """
        if getattr(self.cfg, "contact_route", "report") != "report":
            return
        obj = self._obj_prim(object_name)
        if obj is None:
            self.log(f"[contact] object {object_name} not on stage -- route inactive")
            return
        if self._creporter is not None and \
                getattr(self, "_creporter_obj", None) == object_name:
            return
        from .contact_report import ContactReporter
        # Right-hand links: everything the articulation calls a body whose name matches
        # the Dex5 right-hand naming, resolved to prim paths through one walk.
        try:
            names = list(self.view.body_names)
        except Exception as exc:
            self.log(f"[contact] no body_names: {exc!r}")
            return
        want = [n for n in names if self._is_right_hand_link(n)]
        paths = [self._body_path(n) for n in want]
        paths = [p for p in paths if p]
        if not paths:
            self.log(f"[contact] no right-hand link prims resolved from {len(names)} "
                     f"bodies -- route inactive, contacts stay UNKNOWN (not zero)")
            return
        self._creporter = ContactReporter(log=self.log)
        # Physics substep, so counts() can turn PhysX's per-substep impulse into a force.
        self._creporter.dt = float(getattr(self.cfg, "physics_dt", 1.0 / 200.0))
        if self._creporter.attach(self.stage, paths, obj.GetPath().pathString):
            self._creporter_obj = object_name
            self._hand_body_names = set(want)
        else:
            self._creporter = None

    @staticmethod
    def _is_right_hand_link(n: str) -> bool:
        # Dex5 right-hand naming is upstream's and has caught this campaign out before
        # (base_link00 right, base_link00L left) -- so match the right hand by NOT
        # ending in the left-hand suffix, and require a hand-ish stem.
        if n.endswith("L"):
            return False
        return n.startswith("Link") or n.startswith("base_link00") \
            or "thumb" in n.lower() or "index" in n.lower() \
            or "middle" in n.lower() or "ring" in n.lower() or "pinky" in n.lower()

    def start_trial(self, index, seed, object_name, obj_start=None):
        self._axis_cache = None
        self._attach_contact_reporter(object_name)
        self.latch_obj_axis(object_name)
        self.trial = Trial(index=index, seed=seed, object_name=object_name,
                           cheat_attach=self.cfg.cheat_attach)
        p = obj_start if obj_start is not None else self.obj_pose(object_name)
        self.trial.obj_start = tuple(float(v) for v in p) if p is not None else (0, 0, 0)
        if self.trial.obj_start[2] < self._surface_h() - 0.05:
            # The reset failed to get the object back on the table. Scoring this as
            # anything but a harness failure is how the false positives happened.
            self._enter("APPROACH")
            self._fail("HARNESS_OBJ_NOT_STAGED",
                       f"object staged at z={self.trial.obj_start[2]:.3f}")
            return
        self._enter("APPROACH")
        self.log(f"trial {index} seed={seed} object={object_name} at "
                 f"{np.round(self.trial.obj_start, 4)}")

    def _enter(self, name):
        if self.trial is not None and self.stage_name in STAGES:
            self.trial.stage_times[self.stage_name] = self.sim_time - self._t_stage
        self.stage_name = name
        self._t_stage = self.sim_time

    def _fail(self, outcome, detail):
        self.trial.outcome = outcome
        self.trial.detail = detail
        p = self.obj_pose(self.trial.object_name)
        self.trial.obj_end = tuple(p) if p is not None else (0, 0, 0)
        self._enter("DONE")
        self.log(f"FAIL {outcome}: {detail}")
        self.results.append(self.trial)
        self.trial = None

    def _succeed(self):
        self.trial.outcome = "SUCCESS"
        p = self.obj_pose(self.trial.object_name)
        self.trial.obj_end = tuple(p) if p is not None else (0, 0, 0)
        self._enter("DONE")
        self.log("SUCCESS")
        self.results.append(self.trial)
        self.trial = None

    # ---------------------------------------------------------------------- step

    def step(self, dt: float):
        self.sim_time += dt
        if self.trial is None:
            return
        t_in_stage = self.sim_time - self._t_stage
        obj = self.obj_pose(self.trial.object_name)
        palm, palm_q = self.palm_pose()
        if palm is None or obj is None:
            self._fail("HARNESS", "palm or object pose unavailable")
            return

        # A fallen robot fails THIS trial for the right reason instead of failing the
        # next four stages for the wrong ones. Without it the taxonomy filled up with
        # REACH_TIMEOUT rows that were really one topple.
        base_z = float(np.asarray(self.art.get_world_pose()[0], float)[2])

        # JOINT TRACE for the 4090 day (C-23).  Media is deferred, so this run produces
        # no video -- but a numeric trace of every joint the pipeline drives, the palm
        # and the object, at 50 Hz, is enough to replay the episode for camera later
        # without re-running the physics.  Text log lines are not: they say what the
        # stage machine decided, not where the arm was.
        if self.sim_time >= self._trace_next:
            self._trace_next = self.sim_time + 0.02
            q = np.asarray(self.art.get_joint_positions(), float)
            self._trace.append({
                "t": round(self.sim_time, 4), "trial": self.trial.index,
                "stage": self.stage_name,
                "arm_q": [round(float(q[i]), 5) for i in self.arm_idx],
                "hand_q": [round(float(q[i]), 5) for i in self.hand_idx],
                "palm": [round(float(v), 5) for v in palm],
                "palm_quat": [round(float(v), 5) for v in palm_q],
                "obj": [round(float(v), 5) for v in obj],
                "base_z": round(base_z, 5),
            })

        if base_z < self.cfg.fallen_z:
            self._fail("ROBOT_FELL", f"pelvis z={base_z:.3f} during {self.stage_name}")
            return

        if self.stage_name == "APPROACH":
            # The robot is spawned at the reach pose; Nav2 approach is C-26-blocked and
            # is reported as its own row rather than silently skipped.
            if t_in_stage > self.cfg.settle_s:
                self._enter("REACH")
                self.log(f"palm at {np.round(palm,3)}, object at {np.round(obj,3)}")

        elif self.stage_name == "REACH":
            axis, target, _ = self._approach(obj)
            err = pose_error(palm, palm_q, target,
                             approach_quat(palm_q, axis, self.grasp_axis_local),
                             w_rot=self.cfg.w_rot)
            d = float(np.linalg.norm(err[:3]))
            if d < self.cfg.reach_tol:
                # Palm frame at the pre-grasp, printed once per trial: the columns of R
                # are the palm's local x/y/z expressed in world. Needed to choose a grasp
                # orientation at all -- "point the palm at the can" is meaningless until
                # you know which local axis leaves the palm.
                w, x, y, z = palm_q
                R = np.array([
                    [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                    [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                    [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
                self.log(f"  palm quat={np.round(palm_q,4)}")
                self.log(f"  palm local x->{np.round(R[:,0],3)} y->{np.round(R[:,1],3)} "
                         f"z->{np.round(R[:,2],3)}")
                self._enter("DESCEND")
                self.log(f"pre-grasp reached, {d*1000:.0f} mm")
            elif t_in_stage > self.cfg.reach_timeout_s:
                self._fail("REACH_TIMEOUT", f"{d*1000:.0f} mm from pre-grasp")
            else:
                if int(t_in_stage * 2) != int((t_in_stage - dt) * 2) and \
                        int(t_in_stage) % 5 == 0:
                    self.log(f"  d={d*1000:6.0f} mm  palm={np.round(palm,3)} "
                             f"armq={np.round(np.asarray(self.art.get_joint_positions(), float)[self.arm_idx][:4],3)}")
                self._servo(err)

        elif self.stage_name == "DESCEND":
            # Straight down onto the can with the palm held level. Separated from REACH
            # so a failure to get ABOVE the object and a failure to come DOWN onto it are
            # different rows.
            # No orientation term here. The approach axis was established during REACH
            # and the palm is already on it; the last few centimetres are pure
            # translation, and spending DLS authority re-asserting an orientation the
            # palm already has is what left DESCEND timing out 102 mm short.
            _, _, target = self._approach(obj)
            err = pose_error(palm, palm_q, target, None)
            d = float(np.linalg.norm(err[:3]))
            if d < self.cfg.grasp_tol:
                self._enter("GRASP")
                self.log(f"at grasp pose, {d*1000:.0f} mm")
            elif obj[2] < self._surface_h() - 0.05:
                self._fail("KNOCKED_OFF_IN_DESCEND", f"object z={obj[2]:.3f}")
            elif t_in_stage > self.cfg.descend_timeout_s:
                self._fail("DESCEND_TIMEOUT", f"{d*1000:.0f} mm from grasp pose")
            else:
                self._servo(err)

        elif self.stage_name == "GRASP":
            # PHASE GAINS. C-38 capped the finger drives at kp 5.0 because the imported
            # value was 35810, which was never a real gain -- but 5.0 is the SONIC-IDLE
            # figure, chosen so an idle hand does not fight itself, and it cannot
            # generate grip. Raised for GRASP and LIFT only, and reverted when the trial
            # ends, so the idle hand is unchanged. The URDF effort clamp still applies.
            self._grip_gains(True)
            self._grip_ratio = min(1.0, self._grip_ratio + dt / self.cfg.close_s)
            self._set_hand(self._grip_ratio)
            if self.trial is not None and self.trial.obj_tilt_deg is None:
                t0 = self.obj_tilt_deg(self.trial.object_name)
                if t0 is not None and t0 > self.cfg.topple_tilt_deg:
                    self._fail("TOPPLED", f"tilt {t0:.1f} deg during closure")
                    return
            if self._grip_ratio >= 1.0 and t_in_stage > self.cfg.close_s + 0.5:
                fc = self.finger_contacts()
                if fc is None:
                    self.log("finger contact unreadable; lifting on closure alone")
                    self._enter("LIFT")
                    return
                n, tot, _ = fc
                self.trial.grip_contacts = n
                # THE GRIP DIAGNOSTIC. Three numbers decide whether ~0.5 N is a position
                # error problem, a gain problem, or a clamp problem, and they have never
                # been printed together:
                #   q vs target  -- is overclose actually commanding PAST contact, or did
                #                   the finger reach its target and stop generating error?
                #   |tau|        -- is the drive at the URDF effort clamp (ceiling) or
                #                   far below it (gain/error too small)?
                #   force        -- what the fingertips actually deliver, in newtons.
                try:
                    qh = np.asarray(self.art.get_joint_positions(), np.float32)[self.hand_idx]
                    tgt = np.asarray(self._hand_target, np.float32)
                    err = tgt - qh
                    eff = None
                    try:
                        eff = np.asarray(self.view.get_measured_joint_efforts())
                        eff = eff.reshape(-1, eff.shape[-1])[0][self.hand_idx]
                    except Exception:
                        pass
                    # WHICH joints carry the error. C-13 holds the Dex5's passive
                    # fingers at zero, and a passive joint has no drive -- so a large
                    # err[max] on a passive joint is not evidence of grip demand at all.
                    # WHICH links are touching. 4 contacts that are all on the same
                    # side of the can push it away; 2 that oppose each other hold it.
                    # The count alone cannot tell those apart, and "object rose -0.018 m"
                    # with 47 N of contact is exactly what non-opposing contact looks like.
                    try:
                        per = fc[2] if len(fc) > 2 else []
                        names_sorted = sorted(self._creporter._finger_paths)
                        hot = [(names_sorted[i].rsplit("/", 1)[-1], round(float(v), 1))
                               for i, v in enumerate(per) if v > 0.0]
                        self.log(f"[grip] contact links: {hot}")
                    except Exception as exc:
                        self.log(f"[grip] contact-link breakdown failed: {exc!r}")
                    j_err = int(np.argmax(err))
                    self.log(f"[grip] worst-err joint idx={j_err} "
                             f"q={float(qh[j_err]):+.3f} tgt={float(tgt[j_err]):+.3f} "
                             f"driven_reached={int(np.sum(np.abs(err) < 0.02))}/{len(err)}")
                    self.log(f"[grip] q[max]={float(qh.max()):+.3f} "
                             f"target[max]={float(tgt.max()):+.3f} "
                             f"err[max]={float(err.max()):+.3f} rad "
                             f"kp={self.cfg.grip_kp} -> demand={float(err.max())*self.cfg.grip_kp:.2f} Nm"
                             + (f" |tau|max={float(np.abs(eff).max()):.3f} Nm" if eff is not None else "")
                             + f"  contacts={n} force={tot:.2f} N")
                except Exception as exc:
                    self.log(f"[grip] diagnostic failed: {exc!r}")
                self.trial.grip_force_n = round(tot, 3)
                self.log(f"closed: {n} finger links in contact, {tot:.2f} N total")
                # LIFT GATE. Lifting on "the fingers finished moving" is what produced
                # six NO_GRIP rows that had already decided their outcome before LIFT
                # began. A hand that is not touching the object is not holding it.
                tilt = self.obj_tilt_deg(self.trial.object_name)
                drop = float(self.trial.obj_start[2] - obj[2])
                if tilt is not None:
                    self.trial.obj_tilt_deg = round(tilt, 1)
                    self.log(f"[grip] object tilt {tilt:.1f} deg, centre drop "
                             f"{drop*1000:+.0f} mm")
                if n > 0 and ((tilt is not None and tilt > self.cfg.topple_tilt_deg)
                              or drop > self.cfg.topple_drop_m):
                    # A TOPPLE, not a grip failure. Named so it stops being counted as
                    # one: the fingers are holding an object that fell over as they closed.
                    self._fail("TOPPLED",
                               f"tilt {tilt if tilt is None else round(tilt,1)} deg, "
                               f"centre dropped {drop*1000:.0f} mm with {n} contacts")
                    return
                if n < self.cfg.min_grip_contacts:
                    self._fail("NO_CONTACT_AT_CLOSURE",
                               f"{n} finger links in contact ({tot:.2f} N), "
                               f"need {self.cfg.min_grip_contacts}")
                    return
                self._enter("LIFT")

        elif self.stage_name == "LIFT":
            # LIFT STRAIGHT UP FROM THE GRASP POSE, preserving the approach offset.
            #
            # This used `obj_start + [0,0,lift_h] + pregrasp_offset`, and pregrasp_offset
            # is (0, 0, 0.13) -- a pure-vertical legacy offset from when every grasp was
            # top-down. At counter height the approach axis is HORIZONTAL, so the palm is
            # holding the can from ~0.12 m to the side; commanding it to a point 0.27 m
            # directly ABOVE the can asks it to rise AND swing sideways by 0.12 m, which
            # drags the object out of the fingers. Measured symptom: 3 opposing contacts
            # at 23.9-35.6 N and the can moving +0.001 / -0.017 m -- a grip that holds
            # while the hand travels somewhere the object cannot follow.
            _, _, gr = self._approach(obj)
            target = np.asarray(gr, float) + np.array([0.0, 0.0, self.cfg.lift_h])
            err = pose_error(palm, palm_q, target, None)
            self._servo(err)
            if obj[2] - self.trial.obj_start[2] > self.cfg.lift_success_h:
                self._enter("PLACE" if self.cfg.skip_carry else "CARRY")
                self.log(f"lifted {obj[2]-self.trial.obj_start[2]:.3f} m")
            elif obj[2] < self._surface_h() - 0.05:
                # The can left the table rather than rising with the hand. A different
                # problem from a hand that closed and did not grip, and it was hiding in
                # the same bucket: `rose -0.765 m` is the table height, not a grip failure.
                self._fail("KNOCKED_OFF_IN_LIFT",
                           f"object fell to z={obj[2]:.3f}")
            elif t_in_stage > self.cfg.lift_timeout_s:
                self._fail("NO_GRIP",
                           f"hand closed, object rose {obj[2]-self.trial.obj_start[2]:+.3f} m")

        elif self.stage_name == "CARRY":
            # Same correction as LIFT: carry from the grasp pose, not from a vertical
            # offset above the object's start.
            _, _, gr_c = self._approach(obj)
            tgt = (np.asarray(gr_c, float)
                   + np.array([0.0, 0.0, self.cfg.lift_h])
                   + np.array(self.cfg.carry_vec))
            err = pose_error(palm, palm_q, tgt, None)
            self._servo(err)
            moved = float(np.linalg.norm(obj[:2] - np.array(self.trial.obj_start)[:2]))
            if obj[2] - self.trial.obj_start[2] < self.cfg.drop_h:
                self._fail("DROPPED_IN_CARRY", f"after {moved:.2f} m")
            elif moved >= self.cfg.carry_dist:
                self._enter("PLACE")
                self.log(f"carried {moved:.2f} m")
            elif t_in_stage > self.cfg.carry_timeout_s:
                self._fail("CARRY_TIMEOUT", f"moved {moved:.2f} m of {self.cfg.carry_dist}")

        elif self.stage_name == "PLACE":
            move = (np.array(self.cfg.place_vec) if self.cfg.skip_carry
                    else np.array(self.cfg.carry_vec))
            tgt = np.array(self.trial.obj_start) + move + np.array([0, 0, 0.01])
            err = pose_error(palm, palm_q, tgt + np.array(self.cfg.pregrasp_offset), None)
            self._servo(err)
            if t_in_stage > self.cfg.place_s:
                self._enter("RELEASE")

        elif self.stage_name == "RELEASE":
            self._grip_ratio = max(0.0, self._grip_ratio - dt / self.cfg.close_s)
            self._set_hand(self._grip_ratio)
            if self._grip_ratio <= 0.0:
                self._enter("RETREAT")

        elif self.stage_name == "RETREAT":
            err = pose_error(palm, palm_q, palm + np.array([0, 0, 0.15]), None)
            self._servo(err)
            if t_in_stage > self.cfg.retreat_s:
                placed = float(np.linalg.norm(obj[:2] - np.array(self.trial.obj_start)[:2]))
                want = (float(np.linalg.norm(np.array(self.cfg.place_vec)[:2]))
                        if self.cfg.skip_carry else self.cfg.carry_dist)
                if obj[2] < self._surface_h() - 0.10:
                    self._fail("PLACED_OFF_TABLE", f"object z={obj[2]:.3f}")
                elif placed < want * 0.5:
                    self._fail("NOT_MOVED", f"object moved only {placed:.2f} m")
                else:
                    self._succeed()

    # ------------------------------------------------------------------ actuation

    def _servo(self, err):
        if self.palm_body is None:
            return
        q_now = np.asarray(self.art.get_joint_positions(), float)[self.arm_idx]
        J = self.arm_jacobian()
        dq = dls_step(J, err, lam=self.cfg.ik_lambda, max_step=self.cfg.ik_max_step,
                      q=q_now, q_nominal=ARM_NOMINAL, k_null=self.cfg.ik_k_null)
        q = np.asarray(self.art.get_joint_positions(), np.float32)[self.arm_idx]
        q_now = q.astype(float)
        # Integrate from the PREVIOUS TARGET, not from the measured position.
        #
        # target = measured + dq cannot outrun its own tracking error: the arm carries a
        # 1 kg hand against kp 40, so it lags its target by a few tenths of a radian, and
        # a target pinned one small step ahead of a lagging arm stalls the moment
        # kp*error balances gravity. Measured: the palm crept 0.11 m of a 0.70 m reach in
        # 25 s and stopped. Integrating the target lets the command lead the arm, which
        # is what a position servo needs.
        base = self._arm_target if self._arm_target is not None else q
        tgt = base + dq
        # Do not let the command run away from the arm either -- a target more than
        # this far ahead is asking for a lurch the balancer has to absorb.
        tgt = np.clip(tgt, q - self.cfg.ik_lead_cap, q + self.cfg.ik_lead_cap)
        tgt = np.clip(tgt, ARM_Q_MIN + ARM_MARGIN, ARM_Q_MAX - ARM_MARGIN)
        self._arm_target = tgt.astype(np.float32)

    def _grip_gains(self, on: bool) -> None:
        """Raise the finger drive gains for the grasp phases, and put them back."""
        if not self.has_hand or getattr(self, "_grip_gain_on", None) is on:
            return
        kp = self.cfg.grip_kp if on else self.cfg.idle_hand_kp
        kd = self.cfg.grip_kd if on else self.cfg.idle_hand_kd
        try:
            self.view.set_gains(np.full(len(self.hand_idx), kp, np.float32),
                                np.full(len(self.hand_idx), kd, np.float32),
                                joint_indices=self.hand_idx)
            self._grip_gain_on = on
            self.log(f"finger drives -> kp {kp} kd {kd}")
        except Exception as exc:
            self.log(f"finger gain change failed: {exc!r}")

    def _set_hand(self, ratio):
        if not self.has_hand:
            return
        q = np.zeros(len(RIGHT_HAND_JOINTS), np.float32)
        # THUMB FIRST. Closing all sixteen actuated joints on one ramp means whichever
        # finger reaches the can first pushes it, and a free-standing can gets swept over
        # before the opposing side arrives -- measured as tilt, and previously mis-filed
        # as NO_GRIP. The thumb (indices 0..3) closes over the first `thumb_lead` of the
        # ramp and becomes a BACK-STOP; the fingers then press the can INTO it rather
        # than across the counter.
        lead = float(np.clip(self.cfg.thumb_lead, 0.0, 0.9))
        r_thumb = float(np.clip(ratio / lead, 0.0, 1.0)) if lead > 0 else ratio
        r_fing = float(np.clip((ratio - lead) / max(1.0 - lead, 1e-6), 0.0, 1.0))
        for i in range(len(RIGHT_HAND_JOINTS)):
            if i in PASSIVE_HAND_IDX:
                continue                     # unactuated on the real Dex5-1 (C-13)
            ratio = r_thumb if i < 4 else r_fing
            # OVERCLOSE. A position drive makes force out of ERROR, so a target set
            # AT the contact pose produces a hand that touches with ~zero force. The
            # target is driven PAST contact by `overclose_rad`; the fingers cannot get
            # there, and the residual error is the grip.
            q[i] = ratio * (self.cfg.close_rad + self.cfg.overclose_rad)
        self._hand_target = q

    def reapply(self):
        """Re-assert MM5's joints AFTER the policy has written its own action.

        Uses apply_action, NOT the view's set_joint_position_targets, and it is one
        action covering arm and fingers together. Both details are the MM3 lesson
        repeating: ArticulationController keeps exactly ONE pending action per step, so
        the policy's action (all 29 body joints, arms included) silently replaced every
        view write -- the palm sat at z = 0.70 for 25 s while the IK dutifully computed
        targets nothing ever applied. Two apply_action calls would be just as bad, since
        the second replaces the first, which is why arm and hand share one.

        Joints not named in the action keep the target the policy set on the previous
        step, so the legs stay the policy's and lose at most one step of freshness.
        """
        if self._arm_target is None and not self.has_hand:
            return
        from isaacsim.core.utils.types import ArticulationAction
        idx = [self.arm_idx]
        val = [self._arm_target if self._arm_target is not None
               else np.asarray(self.art.get_joint_positions(), np.float32)[self.arm_idx]]
        # FIXED BASE: MM5 owns the whole body, not just the arm.
        #
        # With the base pinned the locomotion policy's observations never change, so its
        # output is whatever it happens to settle on -- measured, it drifted the arm up
        # to a palm height of 1.29 m with the can at 0.80. Holding legs and torso at the
        # settled home pose makes the fixed-base variant deterministic, which is the
        # whole point of having it: everything downstream of balance is then tested
        # against a body that is not moving for reasons of its own.
        if self.cfg.fix_base and self._body_hold is not None:
            idx.append(self.body_idx)
            val.append(self._body_hold)
        if self.has_hand:
            idx.append(self.hand_idx)
            val.append(self._hand_target)
        self.art.apply_action(ArticulationAction(
            joint_positions=np.concatenate(val).astype(np.float32),
            joint_indices=np.concatenate(idx).astype(np.int32)))
