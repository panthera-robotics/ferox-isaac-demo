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

    def as_row(self) -> dict:
        return {"trial": self.index, "seed": self.seed, "object": self.object_name,
                "outcome": self.outcome, "detail": self.detail,
                "cheat_attach": self.cheat_attach,
                "obj_start": [round(v, 4) for v in self.obj_start],
                "obj_end": [round(v, 4) for v in self.obj_end],
                "stage_s": {k: round(v, 3) for k, v in self.stage_times.items()}}


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
        from pxr import UsdGeom, Usd
        p = self._obj_prim(name)
        if p is None:
            return None
        m = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        return np.array([t[0], t[1], t[2]], float)

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

    def start_trial(self, index, seed, object_name, obj_start=None):
        self._axis_cache = None
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
            self._grip_ratio = min(1.0, self._grip_ratio + dt / self.cfg.close_s)
            self._set_hand(self._grip_ratio)
            if self._grip_ratio >= 1.0 and t_in_stage > self.cfg.close_s + 0.5:
                self._enter("LIFT")

        elif self.stage_name == "LIFT":
            target = np.array(self.trial.obj_start) + np.array([0, 0, self.cfg.lift_h])
            err = pose_error(palm, palm_q, target + np.array(self.cfg.pregrasp_offset), None)
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
            tgt = (np.array(self.trial.obj_start)
                   + np.array([0, 0, self.cfg.lift_h])
                   + np.array(self.cfg.carry_vec))
            err = pose_error(palm, palm_q, tgt + np.array(self.cfg.pregrasp_offset), None)
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

    def _set_hand(self, ratio):
        if not self.has_hand:
            return
        q = np.zeros(len(RIGHT_HAND_JOINTS), np.float32)
        for i in range(len(RIGHT_HAND_JOINTS)):
            if i in PASSIVE_HAND_IDX:
                continue                     # unactuated on the real Dex5-1 (C-13)
            q[i] = ratio * self.cfg.close_rad
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
