"""The Isaac half of the low-level bridge.

Owns two things and deliberately nothing else:
  * sampling articulation + IMU state into the shm state segment every physics step;
  * evaluating PD at 500 Hz and writing the resulting torques to the articulation.

It does NOT publish DDS.  rt/lowstate is paced at 1041.68 Hz by dds_side.py in a
separate process, for the reasons in shm.py.

JOINT ORDER.  rt/lowstate.motor_state and rt/lowcmd.motor_cmd are indexed in UNITREE
SDK order (G1JointIndex, 29 entries, confirmed against the sdk2py example and the
driver's own g1_29dof.urdf -- the two agree).  Isaac orders DOFs breadth-first over
the whole articulation, which is a different order again, and with Dex5 hands
attached it interleaves 40 finger joints through the body joints.  Every conversion
here therefore goes through a NAME-built map.  RULE-HAND-NAME in the contract
forbids index-slicing the hand block, and the same trap applies to the body block
the moment hands exist, so nothing below indexes by position.

FAIL-CLOSED.  If no valid rt/lowcmd has been accepted for `cmd_timeout_ms` (100 ms
per the campaign), the bridge stops tracking the last commanded pose and applies
pure damping, tau = -kd_safe * dq.  It does NOT hold the last q_d: holding a stale
target is how a robot keeps driving into whatever it was doing when its controller
died, which is the failure the timeout exists to prevent.
"""

from __future__ import annotations

import os
import time

import numpy as np

from isaacsim.core.utils.types import ArticulationAction

from . import shm

# SDK order == the actuated-joint order of the driver's g1_29dof.urdf, verified.
SDK_JOINT_NAMES = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
N_SDK = len(SDK_JOINT_NAMES)

# Nm, from ref/panthera-g1-driver/evidence/fw2026q3/g1_29dof.urdf <limit effort=...>,
# in SDK order. The campaign requires the computed torque clamped to these.
URDF_EFFORT_LIMIT = np.array([
    88, 88, 88, 139, 35, 35,
    88, 88, 88, 139, 35, 35,
    88, 35, 35,
    25, 25, 25, 25, 25, 5, 5,
    25, 25, 25, 25, 25, 5, 5,
], dtype=np.float32)

# Damping used by the fail-closed path, Nm per rad/s. Scaled off each joint's effort
# limit so the heavy leg joints brake harder than the wrists rather than every joint
# getting one arbitrary constant.
SAFE_KD = 0.05 * URDF_EFFORT_LIMIT

# IDLE HOLD, used ONLY before the first rt/lowcmd ever arrives, in SDK order.
#
# This is not a softened fail-closed -- it is the twin's stand-in for the G1's own
# built-in motion mode. A real G1 holds its stance the moment it is powered, which is
# exactly why unitree_sdk2py's g1_low_level_example opens by calling
# MotionSwitcherClient.ReleaseMode(): control has to be TAKEN from the built-in
# controller. Without an equivalent here the twin has no controller at all between
# spawn and the first command, and it spent the ~40 s of ROS setup collapsing on the
# floor -- which then failed test (a) for a reason that has nothing to do with the
# wire. Fail-closed after a command HAS been seen stays damping-only; the two states
# are distinguished by _first_cmd_seen, never merged.
HOLD_KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40,
                    200, 40, 40, 40, 40, 40, 40, 40, 40, 40,
                    40, 40, 40, 40, 40, 40, 40], np.float32)
HOLD_KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5,
                    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], np.float32)


# Dex5-1P joint order per hand, taken from the official Unitree URDFs
# (ref/unitree_ros/robots/dexterous_hand_description/dex5_1/Dex5-URDF-{L,R}) in
# document order -- the same order panthera-g1-wbc's dex5/limits.py parses, which is
# what the driver speaks. Note index 12 is Link_41L on the left but Roll_41R on the
# right: the hands are NOT name-symmetric, so one list cannot be derived from the
# other by substituting L for R.
DEX5_JOINTS = {
    "left": ["Yaw_11L", "Roll_12L", "Pitch_13L", "Pitch_14L", "Roll_21L", "Pitch_22L",
             "Pitch_23L", "Pitch_24L", "Roll_31L", "Pitch_32L", "Pitch_33L", "Pitch_34L",
             "Link_41L", "Pitch_42L", "Pitch_43L", "Pitch_44L", "Roll_51L", "Pitch_52L",
             "Pitch_53L", "Pitch_54L"],
    "right": ["Yaw_11R", "Roll_12R", "Pitch_13R", "Pitch_14R", "Roll_21R", "Pitch_22R",
              "Pitch_23R", "Pitch_24R", "Roll_31R", "Pitch_32R", "Pitch_33R", "Pitch_34R",
              "Roll_41R", "Pitch_42R", "Pitch_43R", "Pitch_44R", "Roll_51R", "Pitch_52R",
              "Pitch_53R", "Pitch_54R"],
}
N_HAND_JOINTS = 20
# Roll/abduction of fingers 2..5: mechanically unactuated on the Dex5-1, and dead in
# the recorded dataset too (panthera-g1-wbc dex5/limits.py PASSIVE_INDICES). Commands
# to these indices are accepted on the wire and NOT applied -- C-13.
DEX5_PASSIVE_INDICES = (4, 8, 12, 16)


class LowLevelSimBridge:
    """Attach to a SingleArticulation and drive it from rt/lowcmd."""

    # The campaign asks for damping "within 100 ms" of command loss. That is a
    # DEADLINE, so the detector's threshold has to sit below it: a threshold of exactly
    # 100 ms can only ever fire after 100 ms, plus however long until the next check.
    # Measured, the first attempt engaged at 106.47 ms with the threshold at 100.
    #
    # The check now runs every physics step, so quantization is ~1.9 ms at the 0.52
    # RTF this box gives -- but that is not the binding term. Isaac's main loop stalls
    # for a render frame, and with the threshold at 95 ms five trials measured
    # 96.07 / 102.71 / 95.17 / 95.99 / 96.72 ms: the outlier is 7.7 ms of scheduling
    # jitter, not resolution. 85 ms buys a 15 ms budget for it.
    #
    # Damping early is safe and damping late is not, so the asymmetry is deliberate.
    # 100 ms here is a REQUIREMENT from CAMPAIGN 4.2, not a measured property of the
    # real G1 -- if the robot's own timeout is ever measured, this becomes a fidelity
    # question rather than a safety one and the number should be revisited.
    CMD_DEADLINE_MS = 100.0

    def __init__(self, articulation, physics_dt: float, pd_hz: float = 500.0,
                 cmd_timeout_ms: float = 85.0, verbose: bool = True):
        self.art = articulation
        self.physics_dt = float(physics_dt)
        self.cmd_timeout_ns = int(cmd_timeout_ms * 1e6)
        self.verbose = verbose

        # PD is applied every Nth physics step. Physics is deliberately allowed to
        # run FASTER than PD: at 1000 Hz physics the sim's millisecond clock advances
        # by exactly 1 per step, which is what makes the published `tick` field
        # advance +1 like the robot's does instead of skipping in twos.
        self.pd_every = max(1, int(round((1.0 / self.physics_dt) / pd_hz)))
        self.pd_hz_actual = (1.0 / self.physics_dt) / self.pd_every

        names = list(self.art.dof_names)
        missing = [n for n in SDK_JOINT_NAMES if n not in names]
        if missing:
            raise ValueError(f"articulation is missing SDK joints: {missing}")
        # sim index for each SDK slot, and its inverse. Built by name, never sliced.
        self.sdk_to_sim_idx = np.array([names.index(n) for n in SDK_JOINT_NAMES], dtype=np.int32)

        # Hand maps, also by name. RULE-HAND-NAME exists because Isaac interleaves the
        # two hands' DOFs (left 29..63, right 34..68, neither contiguous), so any
        # 20-long slice writes fingers of BOTH hands at random.
        self.hand_sim_idx = {}
        for side, jnames in DEX5_JOINTS.items():
            found = [names.index(n) for n in jnames if n in names]
            if len(found) == N_HAND_JOINTS:
                self.hand_sim_idx[side] = np.array(found, dtype=np.int32)
            elif found:
                raise ValueError(
                    f"{side} hand has {len(found)}/{N_HAND_JOINTS} Dex5 joints in the "
                    f"articulation -- a partial hand is a composition bug, not a variant")
        self.has_hands = len(self.hand_sim_idx) == 2
        if self.has_hands and verbose:
            # The fingers are driven by POSITION targets, which do nothing at all if
            # the URDF import gave their drives zero stiffness. Printed because a
            # silently inert hand looks exactly like a wire that never delivered.
            try:
                ks, ds = self.art._articulation_view.get_gains(
                    joint_indices=self.hand_sim_idx["left"])
                print(f"[lowlevel-sim] left-hand drive gains: kp[:4]={np.ravel(ks)[:4]} "
                      f"kd[:4]={np.ravel(ds)[:4]}", flush=True)
            except Exception as exc:
                print(f"[lowlevel-sim] could not read hand gains: {exc!r}", flush=True)
        if verbose:
            print(f"[lowlevel-sim] hands: {sorted(self.hand_sim_idx)} "
                  f"({'20-entry dex3 wire live' if self.has_hands else 'none -- HAND=none'})",
                  flush=True)

        # The SIM creates both segments, and the DDS side only ever attaches.
        # Ownership used to be split -- sim owned state, bridge owned cmd -- and that
        # is a trap: whoever attaches to a segment the other side later re-creates
        # keeps a mapping to the UNLINKED old one and reads frozen data forever, with
        # no error anywhere. It cost a full round of failed stand tests: the sim
        # attached to a dead cmd segment still holding cmd_count=31500 from the
        # previous run, concluded it had been commanded, and sat in fail-closed
        # damping while the robot folded. The sim outlives every bridge process, so
        # it owns both and create=True zeroes them at each boot.
        self.state = shm.open_state(create=True)
        self.cmd = shm.open_cmd(create=True)

        self.sim_time = 0.0
        self._t_wall0 = time.perf_counter()
        self.n_step = 0
        self.n_pd = 0
        self.n_failclosed = 0
        self.n_hold = 0
        self.n_hand_cmd = 0
        self._mode = 2
        self._tau_cmd = np.zeros(N_SDK, np.float32)
        self._last_cmd_count = 0
        self._last_cmd_rec = None
        self.n_torn_reads = 0
        self._first_cmd_seen = False
        self._trace_on = os.environ.get("G1_LL_TRACE", "0") == "1"
        self._implicit = False
        # The spawn stance, sampled before any gains are touched: run.py's G1
        # initialize() has just placed the articulation at deploy.yaml's default pose.
        self.q_hold = np.asarray(
            self.art.get_joint_positions(), dtype=np.float32)[self.sdk_to_sim_idx].copy()
        if os.environ.get("G1_LL_HOLD_POSE", "spawn") == "zero":
            # The sdk2py example's stage 1 drives to ZERO posture, and that is not
            # naivety: straight legs put the ground reaction almost through the knee
            # and ankle axes, so a joint-space PD needs very little torque to hold it.
            # The deploy.yaml stance (knees 0.3, arms forward) needs much more.
            self.q_hold = np.zeros(N_SDK, np.float32)
        self._fix_base = os.environ.get("G1_LL_FIX_BASE", "0") == "1"
        self._spawn_pos, self._spawn_quat = self.art.get_world_pose()
        if self._fix_base:
            print("[lowlevel-sim] TEST RIG ACTIVE: base pinned to spawn pose (C-30)",
                  flush=True)
        _p, _ = self.art.get_world_pose()
        print(f"[lowlevel-sim] spawn base_z={float(_p[2]):.4f} "
              f"hold_pose={os.environ.get('G1_LL_HOLD_POSE', 'spawn')} "
              f"q_hold[:6]={np.round(self.q_hold[:6], 3).tolist()}", flush=True)

        # Torque control: the campaign wants OUR PD, so Isaac's own position drive is
        # zeroed rather than left to fight the torques we write.
        #
        # ONLY on the 29 body joints. Zeroing gains across the whole articulation also
        # kills the position drives the URDF import gave the 40 Dex5 finger joints,
        # and since this bridge writes zero effort to them they would then hang limp
        # -- which reads as a hand-collider or coupling bug rather than as the gain
        # wipe it actually is. The fingers keep their drives and hold their pose,
        # exactly as they do under the locomotion policy (run.py, G1 initialize()).
        # G1_LL_PD=implicit is a DIAGNOSTIC, not a mode of the bridge. It hands the
        # same gains to Isaac's own implicit drive instead of computing torque here,
        # which separates two very different explanations for a twin that will not
        # stand: an explicit 500 Hz PD that is too soft, versus a humanoid that
        # genuinely cannot balance without CoM feedback. Only the second is a finding.
        self._implicit = os.environ.get("G1_LL_PD", "explicit") == "implicit"
        view = self.art._articulation_view
        kp0 = np.zeros(N_SDK, np.float32)
        if self._implicit:
            view.set_gains(HOLD_KP, HOLD_KD, joint_indices=self.sdk_to_sim_idx)
            self.art.get_articulation_controller().switch_control_mode("position")
            print("[lowlevel-sim] DIAGNOSTIC: implicit PD, bridge torques disabled",
                  flush=True)
            return
        print("[lowlevel-sim][trace] set_gains", flush=True)
        view.set_gains(kp0, kp0.copy(), joint_indices=self.sdk_to_sim_idx)
        print("[lowlevel-sim][trace] set_effort_modes", flush=True)
        self.art.get_articulation_controller().set_effort_modes("force")
        print("[lowlevel-sim][trace] switch_control_mode(effort)", flush=True)
        self.art.get_articulation_controller().switch_control_mode("effort")
        print("[lowlevel-sim][trace] control mode set", flush=True)
        if verbose:
            print(f"[lowlevel-sim] physics {1/self.physics_dt:.1f} Hz, PD every "
                  f"{self.pd_every} step(s) -> {self.pd_hz_actual:.1f} Hz, "
                  f"timeout {cmd_timeout_ms:.0f} ms", flush=True)

    # ------------------------------------------------------------------ state

    def _imu(self):
        """Body IMU as the robot reports it: quaternion w-first, gravity +z upright."""
        pos, quat = self.art.get_world_pose()          # Isaac gives quat as w,x,y,z
        lin_vel = self.art.get_linear_velocity()
        ang_vel = self.art.get_angular_velocity()
        w, x, y, z = (float(v) for v in quat)
        # World->body rotation applied to (gravity + linear acceleration). The robot's
        # accelerometer reads +9.8 on z when upright, so gravity is ADDED, not removed.
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
        ], dtype=np.float64)
        accel_world = np.array([0.0, 0.0, 9.80665])
        accel_body = R.T @ accel_world
        gyro_body = R.T @ np.asarray(ang_vel, dtype=np.float64)
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
        yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return (np.array([w, x, y, z], np.float32),
                gyro_body.astype(np.float32),
                accel_body.astype(np.float32),
                np.array([roll, pitch, yaw], np.float32))

    def _trace(self, what: str) -> None:
        # Isaac faults inside PhysX arrive as a bare SIGSEGV with no python frame, so
        # the only way to localise one is to know which call was last entered. Kept,
        # behind G1_LL_TRACE=1, because it is what found the C-23 camera crash.
        if self._trace_on and self.n_step <= 3:
            print(f"[lowlevel-sim][trace step={self.n_step}] {what}", flush=True)

    def _publish_state(self) -> None:
        self._trace("get_joint_positions")
        q_sim = np.asarray(self.art.get_joint_positions(), dtype=np.float32)
        self._trace("get_joint_velocities")
        dq_sim = np.asarray(self.art.get_joint_velocities(), dtype=np.float32)
        self._trace("get_measured_joint_efforts")
        try:
            tau_sim = np.asarray(self.art.get_measured_joint_efforts(), dtype=np.float32)
        except Exception as exc:
            self._trace(f"get_measured_joint_efforts raised {exc!r}")
            tau_sim = np.zeros_like(q_sim)

        q = np.zeros(shm.N_MOTOR, np.float32)
        dq = np.zeros(shm.N_MOTOR, np.float32)
        tau = np.zeros(shm.N_MOTOR, np.float32)
        q[:N_SDK] = q_sim[self.sdk_to_sim_idx]
        dq[:N_SDK] = dq_sim[self.sdk_to_sim_idx]
        tau[:N_SDK] = tau_sim[self.sdk_to_sim_idx]
        # Slots 29..34 stay zero: the robot leaves them zero in all 35998 bag msgs.

        self._trace("imu")
        quat, gyro, accel, rpy = self._imu()
        # Hands go into the flat 40-slot block as [left 0..19 | right 20..39], each in
        # the official URDF document order in DEX5_JOINTS -- never in Isaac's
        # articulation order, which interleaves the two hands (RULE-HAND-NAME).
        hq = np.zeros(shm.N_HAND, np.float32)
        hdq = np.zeros(shm.N_HAND, np.float32)
        htau = np.zeros(shm.N_HAND, np.float32)
        for k, side in enumerate(("left", "right")):
            idx = self.hand_sim_idx.get(side)
            if idx is None:
                continue
            o = k * N_HAND_JOINTS
            hq[o:o + N_HAND_JOINTS] = q_sim[idx]
            hdq[o:o + N_HAND_JOINTS] = dq_sim[idx]
            htau[o:o + N_HAND_JOINTS] = tau_sim[idx]

        self._trace("state.write")
        self.state.write(stamp_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                         sim_time=self.sim_time, physics_step=self.n_step,
                         q=q, dq=dq, tau_est=tau,
                         quat_wxyz=quat, gyro=gyro, accel=accel, rpy=rpy,
                         hand_q=hq, hand_dq=hdq, hand_tau=htau)

    # -------------------------------------------------------------------- PD

    def _cmd_fresh(self, rec=None) -> bool:
        """Is the last accepted rt/lowcmd still inside the watchdog window?

        Cheap enough to call every physics step, which is the point -- see the note in
        on_physics_step. Uses the cached record when the caller has none, so a torn
        seqlock read cannot masquerade as command loss here either.
        """
        if rec is None:
            rec = self._last_cmd_rec
            if rec is None:
                rec = self.cmd.read()
                if rec is not None:
                    self._last_cmd_rec = rec
        if rec is None or int(rec["cmd_count"]) == 0:
            return False
        age = time.clock_gettime_ns(time.CLOCK_MONOTONIC) - int(rec["stamp_ns"])
        return age <= self.cmd_timeout_ns

    def _apply_pd(self) -> None:
        if self._implicit:
            self.art.apply_action(ArticulationAction(
                joint_positions=self.q_hold, joint_indices=self.sdk_to_sim_idx))
            self.n_pd += 1
            self.n_hold += 1
            return
        self._trace("apply_pd: read joints")
        q_sim = np.asarray(self.art.get_joint_positions(), dtype=np.float32)
        dq_sim = np.asarray(self.art.get_joint_velocities(), dtype=np.float32)
        q = q_sim[self.sdk_to_sim_idx]
        dq = dq_sim[self.sdk_to_sim_idx]

        rec = self.cmd.read()
        if rec is None:
            # A torn seqlock read is NOT command loss. Reader and writer both run at
            # 500 Hz here, so they collide often, and treating a collision as silence
            # made fail-closed flap on and off every 50-300 ms through a stand that
            # was being commanded the whole time -- visible as "last command age
            # -1.00 ms", the sentinel for rec being None. The cached record keeps its
            # own stamp_ns, so a GENUINELY stale command still trips the watchdog on
            # time; only the false alarm goes away.
            rec = self._last_cmd_rec
            self.n_torn_reads += 1
        else:
            self._last_cmd_rec = rec

        fresh = self._cmd_fresh(rec)
        if fresh:
            # Latched on a FRESH command, never on the mere existence of a count.
            # "A command once arrived" is the thing that arms fail-closed, and a
            # stale count is not evidence of that.
            self._first_cmd_seen = True

        if fresh:
            if self._mode != 0:
                print(f"[lowlevel-sim] COMMANDED at t={self.sim_time:.3f}s "
                      f"(was {'fail-closed' if self._mode == 1 else 'idle hold'})", flush=True)
            self._mode = 0
            kp = np.asarray(rec["kp"][:N_SDK], np.float32)
            kd = np.asarray(rec["kd"][:N_SDK], np.float32)
            tau = (kp * (np.asarray(rec["q_d"][:N_SDK], np.float32) - q)
                   + kd * (np.asarray(rec["dq_d"][:N_SDK], np.float32) - dq)
                   + np.asarray(rec["tau_ff"][:N_SDK], np.float32))
        elif self._first_cmd_seen:
            if self._mode != 1:
                # MEASURED, not asserted. The 100 ms in the campaign is a requirement
                # on the twin's behaviour, and "the code has a 100 ms constant in it"
                # is not evidence that the behaviour has it. This prints the real age
                # of the last accepted command at the instant damping engages.
                age_ms = (time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                          - int(rec["stamp_ns"])) / 1e6 if rec is not None else -1.0
                verdict = "PASS" if 0 <= age_ms <= self.CMD_DEADLINE_MS else "FAIL"
                print(f"[lowlevel-sim] FAIL-CLOSED engaged at t={self.sim_time:.3f}s, "
                      f"last command age {age_ms:.2f} ms "
                      f"(deadline <= {self.CMD_DEADLINE_MS:.0f} ms) {verdict}", flush=True)
            self._mode = 1
            # FAIL-CLOSED. Damping only -- see the module docstring on why the last
            # q_d is deliberately dropped rather than held.
            tau = -SAFE_KD * dq
            self.n_failclosed += 1
        else:
            # IDLE HOLD, never yet commanded. Holds the spawn stance; see HOLD_KP.
            self._mode = 2
            tau = HOLD_KP * (self.q_hold - q) + HOLD_KD * (0.0 - dq)
            self.n_hold += 1

        tau = np.clip(tau, -URDF_EFFORT_LIMIT, URDF_EFFORT_LIMIT)

        if self.has_hands and rec is not None:
            self._apply_hand_cmd(rec)

        # Written against the body joint indices only, so the finger joints are not
        # handed a zero-effort command that would fight their own position drives.
        self._trace("apply_action(joint_efforts)")
        # ArticulationAction, not SingleArticulation.set_joint_efforts(). The direct
        # setter reports no error and leaves the joints unactuated -- with it, the
        # idle hold saturated a 139 Nm knee against a 1.3 rad error and the robot
        # still folded, which is not what a saturated actuator looks like. This is
        # also the path run.py already drives the articulation through.
        self.art.apply_action(ArticulationAction(
            joint_efforts=tau, joint_indices=self.sdk_to_sim_idx))
        self._trace("apply_action done")

        self._tau_cmd = tau
        self.n_pd += 1

    def _apply_hand_cmd(self, rec) -> None:
        """Drive the Dex5 fingers from the rt/dex3/*/cmd half of the command record.

        Position targets, not torques: the fingers keep the position drives the URDF
        import gave them (see the gain-zeroing note in __init__), so the hand wire
        lands as a target rather than as an effort. HandCmd_ carries kp/kd per entry
        and those are honoured as drive gains, not re-implemented as a second PD.
        """
        now = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        for k, side in enumerate(("left", "right")):
            if now - int(rec["hand_stamp_ns"][k]) > self.cmd_timeout_ns:
                continue    # same fail-closed rule as the body: stale is not a command
            idx = self.hand_sim_idx[side]
            o = k * N_HAND_JOINTS
            q_d = np.asarray(rec["hand_q_d"][o:o + N_HAND_JOINTS], np.float32).copy()
            # The four passive indices are accepted on the wire and dropped here: they
            # are mechanically unactuated on the real Dex5-1, so honouring a command to
            # them would make the twin able to do something the hand cannot. C-13.
            keep = [i for i in range(N_HAND_JOINTS) if i not in DEX5_PASSIVE_INDICES]
            # set_joint_position_targets on the VIEW, not a second apply_action.
            # ArticulationController keeps one pending action per step, so the body's
            # effort action later in this same _apply_pd silently replaced the hand's
            # position action: the counter incremented 8927 times while Yaw_11L sat
            # pinned at 0.6802 -- its URDF upper limit of 0.6803, i.e. the USD drive's
            # own target -- and never once moved toward the commanded 0.35.
            self.art._articulation_view.set_joint_position_targets(
                q_d[keep], joint_indices=idx[keep])
            self.n_hand_cmd += 1

    # ---------------------------------------------------------------- stepping

    def _report(self) -> None:
        r = np.asarray(self.art.get_joint_positions(), np.float32)[self.sdk_to_sim_idx]
        mode = ("cmd" if self._mode == 0 else "FAILCLOSED" if self._mode == 1 else "hold")
        pos, _ = self.art.get_world_pose()
        _, _, _, rpy = self._imu()
        # Base height and pitch, not just joint angles: a joint-space PD holds the
        # POSE while the whole robot topples like a statue, and joint error alone
        # cannot tell those two apart.
        sat = int((np.abs(self._tau_cmd) >= URDF_EFFORT_LIMIT - 1e-3).sum())
        # Real-time factor. It belongs in this line because everything else the twin
        # publishes is paced by WALL time while the physics it describes is paced by
        # SIM time: when RTF drops, every ROS sensor rate drops with it and
        # rt/lowstate quietly starts repeating states instead of carrying new ones.
        wall = time.perf_counter() - self._t_wall0
        rtf = self.sim_time / wall if wall > 0 else 0.0
        print(f"[lowlevel-sim] t={self.sim_time:7.2f} mode={mode:10s} "
              f"pd={self.n_pd} hold={self.n_hold} failclosed={self.n_failclosed} "
              f"rtf={rtf:.3f} base_z={float(pos[2]):+.3f} pitch={float(rpy[1]):+.3f} roll={float(rpy[0]):+.3f} "
              f"|tau|max={float(np.abs(self._tau_cmd).max()):6.2f} sat={sat}/29 "
              f"knee_L={float(r[3]):+.3f} hip_p_L={float(r[0]):+.3f} "
              f"torn={self.n_torn_reads} hand_cmd={self.n_hand_cmd} hand_q0={float(np.asarray(self.art.get_joint_positions(), np.float32)[self.hand_sim_idx['left'][0]]):+.4f}"
              if self.has_hands else
              f"knee_L={float(r[3]):+.3f} hip_p_L={float(r[0]):+.3f}", flush=True)

    def _hold_base(self) -> None:
        """Virtual gantry: pin the floating base to its spawn pose.

        A TEST RIG, declared as one (C-30), not a controller. It exists because a
        joint-space PD cannot balance a humanoid -- proven here against Isaac's own
        implicit drive using the locomotion policy's own gains, which topples
        identically -- so without a rig every hand and wire test would be measuring a
        robot lying face-down with its fingers trapped under it, and would measure
        that instead of the thing under test.

        This is how unitree_sdk2py's own g1_low_level_example is run on hardware: the
        robot hangs. The rig is never on by default and every result taken under it
        says so.
        """
        self.art.set_world_pose(self._spawn_pos, self._spawn_quat)
        self.art.set_linear_velocity(np.zeros(3, np.float32))
        self.art.set_angular_velocity(np.zeros(3, np.float32))

    def on_physics_step(self, step_size: float) -> None:
        self.sim_time += float(step_size)
        self.n_step += 1
        if self._fix_base:
            self._hold_base()
        if self.n_step % int(os.environ.get('G1_LL_REPORT_STEPS', '5000')) == 0:
            self._report()
        self._publish_state()
        # The WATCHDOG runs every physics step; the commanded PD runs at pd_every.
        #
        # Tying the watchdog to the PD decimation gave it a wall-clock resolution of
        # pd_hz x RTF -- about 3.9 ms here -- and three measured engagements came out
        # at 96.16, 99.78 and 102.05 ms against a 100 ms deadline: one in three late,
        # for no reason except when the check happened to run. Checking every step
        # halves the quantum. It also means damping, once engaged, is applied at the
        # full physics rate rather than at 500 Hz, which is the right way round: the
        # 500 Hz in CAMPAIGN 4.2 specifies the PD driven by rt/lowcmd, and fail-closed
        # damping is not that PD -- it is the thing that runs when there is no command.
        if self.n_step % self.pd_every == 0 or not self._cmd_fresh():
            self._apply_pd()

    def stats(self) -> dict:
        return {"physics_steps": self.n_step, "pd_updates": self.n_pd,
                "pd_hz_configured": self.pd_hz_actual, "sim_time": self.sim_time,
                "failclosed_updates": self.n_failclosed,
                "idle_hold_updates": self.n_hold,
                "hand_cmd_applied": self.n_hand_cmd,
                "torn_cmd_reads": self.n_torn_reads}

    def close(self) -> None:
        self.state.close()
        self.cmd.close()
