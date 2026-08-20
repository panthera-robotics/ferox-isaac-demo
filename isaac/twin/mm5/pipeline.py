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

from .ik import dls_step, pose_error

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

STAGES = ["APPROACH", "REACH", "GRASP", "LIFT", "CARRY", "PLACE", "RELEASE", "RETREAT"]


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

    # ------------------------------------------------------------------ helpers

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
        """World pose of the right palm, (pos, quat_wxyz)."""
        pos, quat = self.view.get_world_poses()
        # get_world_poses returns the ARTICULATION root; body poses come from the
        # body-level API when available, else fall back to the transform of the link.
        try:
            bp, bq = self.view.get_body_poses()          # (1, nbody, 3), (1, nbody, 4)
            return np.asarray(bp[0][self.palm_body], float), np.asarray(bq[0][self.palm_body], float)
        except Exception:
            from pxr import UsdGeom, Usd
            prim = self.stage.GetPrimAtPath(
                f"{self.cfg.robot_root}/{RIGHT_PALM_BODY}")
            if prim and prim.IsValid():
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                q = m.ExtractRotationQuat()
                return (np.array([t[0], t[1], t[2]], float),
                        np.array([q.GetReal(), *q.GetImaginary()], float))
            return None, None

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

    def start_trial(self, index, seed, object_name):
        self.trial = Trial(index=index, seed=seed, object_name=object_name,
                           cheat_attach=self.cfg.cheat_attach)
        p = self.obj_pose(object_name)
        self.trial.obj_start = tuple(p) if p is not None else (0, 0, 0)
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
            target = obj + np.array(self.cfg.pregrasp_offset)
            err = pose_error(palm, palm_q, target, None)
            d = float(np.linalg.norm(err[:3]))
            if d < self.cfg.reach_tol:
                self._enter("GRASP")
                self.log(f"pre-grasp reached, {d*1000:.0f} mm")
            elif t_in_stage > self.cfg.reach_timeout_s:
                self._fail("REACH_TIMEOUT", f"{d*1000:.0f} mm from pre-grasp")
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
                self._enter("CARRY")
                self.log(f"lifted {obj[2]-self.trial.obj_start[2]:.3f} m")
            elif t_in_stage > self.cfg.lift_timeout_s:
                self._fail("LIFT_FAILED",
                           f"object rose {obj[2]-self.trial.obj_start[2]:.3f} m")

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
            tgt = (np.array(self.trial.obj_start) + np.array(self.cfg.carry_vec)
                   + np.array([0, 0, 0.01]))
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
                if obj[2] < self.cfg.table_h - 0.10:
                    self._fail("PLACED_OFF_TABLE", f"object z={obj[2]:.3f}")
                elif placed < self.cfg.carry_dist * 0.5:
                    self._fail("NOT_MOVED", f"object moved only {placed:.2f} m")
                else:
                    self._succeed()

    # ------------------------------------------------------------------ actuation

    def _servo(self, err):
        if self.palm_body is None:
            return
        J = self.arm_jacobian()
        dq = dls_step(J, err, lam=self.cfg.ik_lambda, max_step=self.cfg.ik_max_step)
        q = np.asarray(self.art.get_joint_positions(), np.float32)[self.arm_idx]
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
        if self.has_hand:
            idx.append(self.hand_idx)
            val.append(self._hand_target)
        self.art.apply_action(ArticulationAction(
            joint_positions=np.concatenate(val).astype(np.float32),
            joint_indices=np.concatenate(idx).astype(np.int32)))
