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

# SONIC's own nominal stance, from the reference config that fed W3's 17.53 m MuJoCo
# walk: gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml
# DEFAULT_DOF_ANGLES. Listed there in SDK order, which independently confirms the joint
# order this bridge writes.
#
# It matters because the omni policy hands over holding a DIFFERENT stance -- knee 0.124
# against SONIC's 0.300, and elbows at 0.97 with a 1 kg hand on each against SONIC's 0.0,
# which puts the CoM well forward of where SONIC expects it. Handing SONIC a robot in a
# pose it never trains from is what C-39 actually is.
SONIC_DEFAULT_Q = np.array([
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,      # left leg
    -0.1, 0.0, 0.0, 0.3, -0.2, 0.0,      # right leg
    0.0, 0.0, 0.0,                        # waist
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # left arm
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    # right arm
], dtype=np.float32)

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


def _quat_from_matrix(R):
    """Rotation matrix -> quaternion (w, x, y, z), via the largest-diagonal branch."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        S = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * S, (R[2, 1] - R[1, 2]) / S,
                         (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / S, 0.25 * S,
                         (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S])
    if R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S,
                         0.25 * S, (R[1, 2] + R[2, 1]) / S])
    S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S,
                     (R[1, 2] + R[2, 1]) / S, 0.25 * S])


def _quat_mul(a, b):
    """Hamilton product, both (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


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
                 cmd_timeout_ms: float = 85.0, verbose: bool = True,
                 passive: bool = False):
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
        self._rig_ignored_cmds = 0
        self.n_nan_tau = 0
        self._blend_to_nominal = os.environ.get("G1_HANDOFF_TO_NOMINAL", "1") == "1"
        # The Dex5 finger joints keep the USD import's position drives, and those are
        # STIFF -- measured kp 35809.9 with kd 0. Driving them from rt/dex3 at that
        # stiffness slams the fingers, and the reaction travels up the arm: SONIC's
        # own guard trips on body_dq[20] / body_dq[24], which are ARM joints, not
        # finger ones. G1_LL_DEX3_APPLY=0 isolates that path for diagnosis;
        # G1_LL_HAND_KP re-gains the finger drives to something an actuator could
        # actually produce.
        self._dex3_apply = os.environ.get("G1_LL_DEX3_APPLY", "1") == "1"
        self._hand_kp = float(os.environ.get("G1_LL_HAND_KP", "0"))
        self.active = not passive
        # Bumpless transfer. At hand-over the robot is in the pose the POLICY left it
        # in, while SONIC has been running its own policy against a robot it was not
        # driving -- so its q_d can be far away, and applying it as a step drives the
        # PD hard enough to spike joint velocity. SONIC then trips its own safety check
        # ("body_dq[20] = 36.95 > 35") and stops, which is a correct reaction to a real
        # transient rather than a false alarm. The target is ramped instead.
        self._blend_s = float(os.environ.get("G1_HANDOFF_BLEND_S", "1.0"))
        self._blend_from = None
        self._blend_until_ns = 0
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
        _hp = os.environ.get("G1_LL_HOLD_POSE", "spawn")
        if _hp == "sonic":
            # Hold SONIC'S OWN nominal stance while it boots, so that when the rig lets
            # go it inherits a robot already in the pose it trains from. This is the
            # C-39 fork: if the twin stands from here, C-39 is the TRANSITION (the omni
            # policy's stance is simply not SONIC's); if it still falls, the stance was
            # never the problem and the hand mass is.
            self.q_hold = SONIC_DEFAULT_Q.copy()
        elif _hp == "zero":
            # The sdk2py example's stage 1 drives to ZERO posture, and that is not
            # naivety: straight legs put the ground reaction almost through the knee
            # and ankle axes, so a joint-space PD needs very little torque to hold it.
            # The deploy.yaml stance (knees 0.3, arms forward) needs much more.
            self.q_hold = np.zeros(N_SDK, np.float32)
        # G1_LL_FIX_BASE: "0" off, "1" always, "until_commanded" = hold the base until
        # a controller has actually had authority for G1_LL_RIG_RELEASE_S seconds, then
        # let go. The third mode exists because SONIC needs ~15 s of wall time to
        # initialise and build/load its TensorRT engines, and the twin cannot stand on
        # its own for those 15 s (MM3 test (a)) -- so without it SONIC always inherits
        # a robot already face-down, and the test measures the bring-up race instead of
        # the controller. Releasing after the controller is live is how a real G1 is
        # brought up too: on a hoist, lowered once the controller has authority.
        _fb = os.environ.get("G1_LL_FIX_BASE", "0")
        self._fix_base = _fb in ("1", "until_commanded")
        self._rig_auto_release = _fb == "until_commanded"
        self._rig_release_after_ns = int(
            float(os.environ.get("G1_LL_RIG_RELEASE_S", "3")) * 1e9)
        self._commanded_since_ns = None
        self._spawn_pos, self._spawn_quat = self.art.get_world_pose()
        # G1_LL_RIG_LIFT_M raises the pinned base so the feet hang CLEAR of the floor.
        # Without it "suspended" is a misnomer: the base is pinned but the feet still
        # touch, which closes a kinematic chain through the ground, and ankle_roll --
        # the joint that takes the lateral mismatch -- sits at ~0.10 rad of error no
        # matter what gain it is given, while every other joint tracks to <0.01. That
        # is the rig fighting the floor, not the PD path failing.
        # G1_LL_RIG_YAW overrides the held base heading.
        #
        # The C-39 A/B found this: the reference MuJoCo sim spawns the robot at identity
        # yaw (quat w=0.9998) and the twin's hospital spawn sits at 90 degrees
        # (w=0.7074, z=0.7068). Projected gravity is yaw-invariant so it does not show up
        # there -- but the driver commands facing=(1,0,0), i.e. world +x, and SONIC
        # answers a 90 degree heading error by turning in place. In MuJoCo that command
        # is "keep facing forward" and costs nothing; on the twin it is "turn 90 degrees"
        # issued to a controller that is also trying to stand for the first time.
        _yaw = os.environ.get("G1_LL_RIG_YAW")
        if _yaw is not None:
            y = float(_yaw)
            self._spawn_quat = np.array(
                [np.cos(y / 2), 0.0, 0.0, np.sin(y / 2)], dtype=np.float32)
            print(f"[lowlevel-sim] rig yaw overridden to {y:.4f} rad "
                  f"(quat w={float(self._spawn_quat[0]):.4f}) -- C-39 A/B", flush=True)

        _lift = float(os.environ.get("G1_LL_RIG_LIFT_M", "0"))
        if _lift:
            self._spawn_pos = np.asarray(self._spawn_pos, dtype=np.float32).copy()
            self._spawn_pos[2] += _lift
            print(f"[lowlevel-sim] rig lift {_lift:.3f} m -> base held at "
                  f"z={float(self._spawn_pos[2]):.4f}", flush=True)
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
        # PASSIVE mode: publish rt/lowstate but do not touch the articulation. Used by
        # G1_CONTROL=handoff, where the omni locomotion policy holds the robot standing
        # while SONIC boots (it needs ~20-60 s, and MM3 established the twin cannot
        # stand unaided for even 1 s). The bridge must still publish state throughout,
        # or SONIC has nothing to initialise against. activate() takes the articulation.
        if passive:
            print("[lowlevel-sim] PASSIVE: publishing state only; the policy still drives",
                  flush=True)
            return

        # G1_LL_PD=implicit is a DIAGNOSTIC, not a mode of the bridge. It hands the
        # same gains to Isaac's own implicit drive instead of computing torque here,
        # which separates two very different explanations for a twin that will not
        # stand: an explicit 500 Hz PD that is too soft, versus a humanoid that
        # genuinely cannot balance without CoM feedback. Only the second is a finding.
        self._implicit = os.environ.get("G1_LL_PD", "explicit") == "implicit"
        self._pd_probe = os.environ.get("G1_LL_PD_PROBE", "0") == "1"
        self._hold_kinematic = os.environ.get("G1_LL_HOLD_KINEMATIC", "0") == "1"
        self._pd_probe_next = 0.0
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

        # Torso IMU, composed from the pelvis IMU through the waist chain. The real G1
        # carries a physically separate torso IMU; the twin has one pelvis IMU, so the
        # torso frame is derived by rotating through waist yaw->roll->pitch (SDK 12,13,14),
        # matching the URDF joint order. Declared as C-36: the twin's torso IMU is
        # COMPOSED, not independent, and its gyro omits the waist joints' own rates.
        wy, wr, wp = float(q[12]), float(q[13]), float(q[14])
        cz, sz = np.cos(wy), np.sin(wy)
        cx, sx = np.cos(wr), np.sin(wr)
        cy, sy = np.cos(wp), np.sin(wp)
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        R_waist = Rz @ Rx @ Ry
        t_gyro = (R_waist.T @ gyro.astype(np.float64)).astype(np.float32)
        t_accel = (R_waist.T @ accel.astype(np.float64)).astype(np.float32)
        qw = _quat_from_matrix(R_waist)
        t_quat = _quat_mul(quat.astype(np.float64), qw).astype(np.float32)
        tw, tx, ty, tz = (float(v) for v in t_quat)
        t_rpy = np.array([
            np.arctan2(2 * (tw * tx + ty * tz), 1 - 2 * (tx * tx + ty * ty)),
            np.arcsin(np.clip(2 * (tw * ty - tz * tx), -1.0, 1.0)),
            np.arctan2(2 * (tw * tz + tx * ty), 1 - 2 * (ty * ty + tz * tz)),
        ], np.float32)

        self._trace("state.write")
        self.state.write(stamp_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                         sim_time=self.sim_time, physics_step=self.n_step,
                         q=q, dq=dq, tau_est=tau,
                         quat_wxyz=quat, gyro=gyro, accel=accel, rpy=rpy,
                         hand_q=hq, hand_dq=hdq, hand_tau=htau,
                         torso_quat_wxyz=t_quat, torso_gyro=t_gyro,
                         torso_accel=t_accel, torso_rpy=t_rpy)

    # -------------------------------------------------------------------- PD

    def _cmd_fresh(self, rec=None) -> bool:
        """Is the last accepted rt/lowcmd still inside the watchdog window?

        Cheap enough to call every physics step, which is the point -- see the note in
        on_physics_step. Uses the cached record when the caller has none, so a torn
        seqlock read cannot masquerade as command loss here either.
        """
        if rec is None:
            # ALWAYS re-read; fall back to the cache only on a torn read. Preferring
            # the cache made this function latch: the first snapshot it took was kept
            # forever, its stamp_ns aged past the timeout, and it then returned False
            # for the rest of the run. In the active path _apply_pd happened to refresh
            # the cache every cycle and hid it; in PASSIVE mode nothing did, so the
            # hand-over never fired while SONIC was commanding at 500 Hz the whole time.
            rec = self.cmd.read()
            if rec is not None:
                self._last_cmd_rec = rec
            else:
                rec = self._last_cmd_rec
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

        # While the rig is holding, the twin keeps its stance and IGNORES rt/lowcmd.
        #
        # A pinned base LIES to a balance controller: SONIC's observations said
        # "upright, motionless" whatever it commanded, so it drove the knee to 1.011 rad
        # and the twin was dropped out of a crouch the instant the rig let go. A hoist
        # does not work that way -- the robot hangs in its stance, the controller boots
        # and watches, and only when it is lowered does it get authority.
        #
        # This has to sit BEFORE _first_cmd_seen latches, not after. Placed after, the
        # commands were ignored but fail-closed still armed, so the rig phase ran as
        # DAMPING instead of as a stance hold and the robot was limp at release.
        if fresh and self._fix_base and self._rig_auto_release:
            fresh = False
            self._rig_ignored_cmds += 1

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
            q_d = np.asarray(rec["q_d"][:N_SDK], np.float32)
            if self._blend_to_nominal:
                # Drive to SONIC'S OWN nominal stance first, then yield to its live
                # commands. Blending straight to q_d hands SONIC a robot in the omni
                # policy's pose, which it answers by crouching -- observed commanding
                # knee 0.669 against its own 0.300 default.
                q_d = SONIC_DEFAULT_Q
            if self._blend_from is not None:
                now_ns = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                if now_ns >= self._blend_until_ns:
                    self._blend_from = None
                    if self._blend_to_nominal:
                        self._blend_to_nominal = False
                        print(f"[lowlevel-sim] nominal stance reached at "
                              f"t={self.sim_time:.3f}s; SONIC now has live control",
                              flush=True)
                else:
                    a = 1.0 - (self._blend_until_ns - now_ns) / (self._blend_s * 1e9)
                    q_d = (1.0 - a) * self._blend_from + a * q_d
            tau = (kp * (q_d - q)
                   + kd * (np.asarray(rec["dq_d"][:N_SDK], np.float32) - dq)
                   + np.asarray(rec["tau_ff"][:N_SDK], np.float32))
            # C-39, 1b(4).  Comparing the wire against the twin's applied torque gave
            # an impossible pair: kp*(q_d - q) computed from rt/lowcmd and rt/lowstate
            # is 80.4 Nm on L_hip_p, while the twin reports COMMANDING 6.5 Nm with
            # sat=0/29.  One of the three inputs the PD actually uses is therefore not
            # the one on the wire.  This prints them from `rec` itself rather than
            # inferring: whichever of kp, q_d or q disagrees with the wire is C-39.
            if self._pd_probe and self._pd_probe_next <= self.sim_time:
                self._pd_probe_next = self.sim_time + 5.0
                err = kp * (q_d - q)
                order = np.argsort(-np.abs(err))[:6]
                print(f"[lowlevel-sim][PD-PROBE] t={self.sim_time:.2f} "
                      f"|tau|max={np.abs(tau).max():.2f} "
                      f"cmd_count={int(rec['cmd_count'])} age_ms="
                      f"{(time.clock_gettime_ns(time.CLOCK_MONOTONIC)-int(rec['stamp_ns']))/1e6:.1f}",
                      flush=True)
                for i in order:
                    print(f"[lowlevel-sim][PD-PROBE]   sdk[{int(i):2d}] kp={kp[i]:8.3f} "
                          f"kd={kd[i]:6.3f} q_d={q_d[i]:+8.4f} q={q[i]:+8.4f} "
                          f"dq={dq[i]:+7.3f} -> kp*e={err[i]:+8.2f} tau={tau[i]:+8.2f}",
                          flush=True)
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
            # C-39.  HOLD_KP is a TORQUE hold, and the PD probe showed it does not
            # actually reach the stance it is given: asked for SONIC's nominal pose it
            # settles at hip -0.522 against a target of -0.100 and drives the waist to
            # +0.520, its mechanical stop, against a target of 0.0.  So every fork run
            # so far -- including the turn-6 "SONIC from its own nominal stance" -- has
            # released SONIC from a pose ~0.4 rad away from the one it was told the
            # robot would be in.  That is not the experiment anyone intended.
            #
            # With the base already rigged, holding the JOINTS kinematically too costs
            # nothing and makes the release pose exactly the commanded one.  It is a
            # diagnostic for the initial condition, not a control mode: once the rig
            # releases, this branch stops running and the torque path resumes.
            if self._hold_kinematic:
                # set_joint_positions, NOT apply_action.  In effort control mode with
                # the drive gains zeroed a position TARGET is ignored -- the first
                # attempt at this used apply_action and moved the stance by 0.0000 rad,
                # which is a no-op dressed up as an experiment.  A state write is the
                # only thing that actually places the joints.
                qs = np.asarray(self.art.get_joint_positions(), np.float32).copy()
                vs = np.zeros_like(qs)
                qs[self.sdk_to_sim_idx] = self.q_hold
                self.art.set_joint_positions(qs)
                self.art.set_joint_velocities(vs)
            tau = HOLD_KP * (self.q_hold - q) + HOLD_KD * (0.0 - dq)
            self.n_hold += 1

        # Belt and braces against the same failure the DDS side now filters: whatever
        # arrives, a non-finite torque must never reach PhysX. np.clip does NOT remove
        # NaN -- it propagates it -- so this has to be an explicit check.
        if not np.all(np.isfinite(tau)):
            self.n_nan_tau += 1
            tau = -SAFE_KD * dq
        tau = np.clip(tau, -URDF_EFFORT_LIMIT, URDF_EFFORT_LIMIT)

        if self.has_hands and rec is not None and self._dex3_apply:
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

    def activate(self) -> None:
        """Take the articulation from the policy. One-way, and asserted to happen once."""
        if self.active:
            return
        names = list(self.art.dof_names)
        view = self.art._articulation_view
        kp0 = np.zeros(N_SDK, np.float32)
        view.set_gains(kp0, kp0.copy(), joint_indices=self.sdk_to_sim_idx)
        self.art.get_articulation_controller().set_effort_modes("force")
        self.art.get_articulation_controller().switch_control_mode("effort")
        # The hand-over pose is the stance the POLICY was holding a moment ago, so the
        # idle hold (if any command gap follows) targets something the robot is already
        # in rather than the spawn pose it has long since left.
        self.q_hold = np.asarray(
            self.art.get_joint_positions(), dtype=np.float32)[self.sdk_to_sim_idx].copy()
        self.active = True
        if self.has_hands and self._hand_kp > 0:
            for side, idx in self.hand_sim_idx.items():
                view.set_gains(np.full(len(idx), self._hand_kp, np.float32),
                               np.full(len(idx), self._hand_kp * 0.02, np.float32),
                               joint_indices=idx)
            print(f"[lowlevel-sim] hand drive gains re-set to kp={self._hand_kp} "
                  f"(was ~35810 from the URDF import)", flush=True)
        self._blend_from = self.q_hold.copy()
        self._blend_until_ns = (time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                                + int(self._blend_s * 1e9))
        print(f"[lowlevel-sim] ACTIVE: articulation taken from the policy at "
              f"t={self.sim_time:.3f}s", flush=True)

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
            if self._rig_auto_release:
                now = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                if self._cmd_fresh():
                    if self._commanded_since_ns is None:
                        self._commanded_since_ns = now
                    elif now - self._commanded_since_ns >= self._rig_release_after_ns:
                        self._fix_base = False
                        print(f"[lowlevel-sim] TEST RIG RELEASED at t={self.sim_time:.3f}s "
                              f"-- controller has had authority for "
                              f"{self._rig_release_after_ns/1e9:.1f}s (C-30)", flush=True)
                else:
                    # Authority lost again (fail-closed or idle) -- restart the clock
                    # rather than releasing onto a robot nothing is driving.
                    self._commanded_since_ns = None
        if self.n_step % int(os.environ.get('G1_LL_REPORT_STEPS', '5000')) == 0:
            self._report()
        self._publish_state()
        if not self.active:
            return
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
                "torn_cmd_reads": self.n_torn_reads,
                "rig_ignored_cmds": self._rig_ignored_cmds,
                "nan_tau_blocked": self.n_nan_tau}

    def close(self) -> None:
        self.state.close()
        self.cmd.close()
