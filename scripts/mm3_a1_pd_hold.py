#!/usr/bin/env python3
"""MM3 test (a1): suspended-base PD hold.

Base fixed (C-30 rig), all 29 joints commanded to a held pose over rt/lowcmd for 60 s,
gate: per-joint mean tracking error < 0.05 rad.

This is what test (a) was actually trying to establish -- that the bridge's PD path
carries a real controller's commands to the joints. The original wording ("stands the
twin for 60 s") also demanded balance, which a joint-space PD cannot provide and which
the real G1 does not provide either; balance under lowcmd is SONIC's job and is
gated in MM4. See RESULTS_MM3 for the topple finding and the ReleaseMode parallel.

Runs inside ferox/twin-lowlevel:humble.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29
GATE_RAD = 0.05

SDK_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]

_DEFAULT_SIM = [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.3,
                -0.2, -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15,
                0.0, 0.0, 0.0, 0.0]
_IDS_MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
            18, 25, 19, 26, 20, 27, 21, 28]
HOLD_Q = np.zeros(G1_NUM_MOTOR, np.float32)
for _sim, _sdk in enumerate(_IDS_MAP):
    HOLD_Q[_sdk] = _DEFAULT_SIM[_sim]

# TWO gain sets, both from the repo rather than invented here.
#
# EXAMPLE: unitree_sdk2py's g1_low_level_example. Written for a BARE G1.
# DEPLOY:  isaac/checkpoints/g1_baseline/params/deploy.yaml -- the gains this twin's
#          own locomotion policy runs, already matched to this asset. Better damped on
#          the legs (kd 2/4 against the example's 1/2), which matters: at the example's
#          damping the free-hanging ankle_roll limit-cycles at +-30 rad/s of velocity
#          while holding position to 0.014 rad, and that cycle reports as ~0.06 rad of
#          steady "error" that no amount of extra kp removes.
#
# Neither set sizes the WRISTS for this robot. The twin carries a 1.0 kg Dex5 hand at
# the end of each arm and both sets give the wrist kp=40, so the wrist sits at a large
# steady offset. Torques were measured against the URDF clamp first -- nothing
# saturates, so this is a gain problem, not an actuator one -- and --wrist-gain raises
# kp and kd together on the six wrist joints only. Raising WHOLE-arm gain instead was
# tried and is worse: it drives the ankle rolls from 0.04 to 0.10 rad through the
# body, because the links are coupled and an explicit PD at 2 ms is marginal on light
# links.
KP_EXAMPLE = np.array([60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40,
                       40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40], np.float32)
KD_EXAMPLE = np.array([1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
                       1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], np.float32)
KP_DEPLOY = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40,
                      200, 40, 40, 40, 40, 40, 40, 40, 40, 40,
                      40, 40, 40, 40, 40, 40, 40], np.float32)
KD_DEPLOY = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 5, 5, 5,
                      1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], np.float32)
WRIST_IDX = [19, 20, 21, 26, 27, 28]

class HoldClient:
    def __init__(self, domain, iface, control_hz, gains, wrist_gain,
                 leg_gain=1.0, arm_gain=1.0):
        ChannelFactoryInitialize(domain, iface) if iface else ChannelFactoryInitialize(domain)
        self.crc = CRC()
        self.dt = 1.0 / control_hz
        self.kp = (KP_DEPLOY if gains == "deploy" else KP_EXAMPLE).copy()
        self.kd = (KD_DEPLOY if gains == "deploy" else KD_EXAMPLE).copy()
        # Damping rule, all three clauses measured rather than chosen:
        #   legs  -> kd scales as sqrt(kp). Unscaled leaves the free-hanging ankle_roll
        #            limit-cycling at +-30 rad/s while holding position to 0.014 rad.
        #   arms  -> kd unscaled. Scaling it made the wrists WORSE (0.041 -> 0.210 rad
        #            at sqrt, 17 joints failing at full), because an explicit PD at
        #            2 ms destabilises on light links as kd*dt approaches the inertia.
        #   wrists-> kd as sqrt, since their kp moves furthest.
        self.kp[:12] *= leg_gain
        self.kd[:12] *= float(np.sqrt(leg_gain))
        self.kp[12:] *= arm_gain
        for i in WRIST_IDX:
            self.kp[i] *= wrist_gain
            self.kd[i] *= float(np.sqrt(wrist_gain))
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.mode_machine = None
        self.q_log: list[np.ndarray] = []
        self.stamps: list[float] = []
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on, 10)

    def _on(self, msg: LowState_) -> None:
        if self.mode_machine is None:
            self.mode_machine = msg.mode_machine
        self.stamps.append(time.perf_counter())
        self.q_log.append(np.array([msg.motor_state[i].q for i in range(G1_NUM_MOTOR)],
                                   dtype=np.float32))

    def wait(self, timeout=20.0) -> bool:
        t0 = time.perf_counter()
        while self.mode_machine is None:
            if time.perf_counter() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    def _write(self, q_d) -> None:
        self.low_cmd.mode_pr = 0
        self.low_cmd.mode_machine = self.mode_machine
        for i in range(G1_NUM_MOTOR):
            mc = self.low_cmd.motor_cmd[i]
            mc.mode = 1
            mc.q = float(q_d[i]); mc.dq = 0.0; mc.tau = 0.0
            mc.kp = float(self.kp[i]); mc.kd = float(self.kd[i])
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def run(self, ramp_s, hold_s):
        q0 = self.q_log[-1].copy()
        t0 = time.perf_counter()
        deadline = t0 + self.dt
        mark = len(self.stamps)
        n = 0
        while True:
            t = time.perf_counter() - t0
            if t >= ramp_s + hold_s:
                break
            r = min(t / ramp_s, 1.0) if ramp_s > 0 else 1.0
            self._write((1.0 - r) * q0 + r * HOLD_Q)
            n += 1
            deadline += self.dt
            slack = deadline - time.perf_counter()
            if slack > 250e-6:
                time.sleep(slack - 200e-6)
            while time.perf_counter() < deadline:
                pass
        # Only the hold window is scored; the ramp is a transient by construction.
        from_t = t0 + ramp_s
        idx = [i for i, s in enumerate(self.stamps[mark:], mark) if s >= from_t]
        q = np.array([self.q_log[i] for i in idx])
        return n, (time.perf_counter() - t0), q


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--control-hz", type=float, default=500.0)
    ap.add_argument("--ramp", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=60.0)
    ap.add_argument("--gains", choices=("deploy", "example"), default="deploy")
    ap.add_argument("--wrist-gain", type=float, default=1.0)
    ap.add_argument("--leg-gain", type=float, default=1.0)
    ap.add_argument("--arm-gain", type=float, default=1.0)
    args = ap.parse_args()

    c = HoldClient(args.domain, args.iface, args.control_hz, args.gains,
                   args.wrist_gain, args.leg_gain, args.arm_gain)
    print("[a1] waiting for rt/lowstate ...", flush=True)
    if not c.wait():
        print("[a1] FAIL: no rt/lowstate within 20 s")
        return 1
    print(f"[a1] rt/lowstate up, mode_machine={c.mode_machine}, "
          f"gains={args.gains} leg={args.leg_gain} arm={args.arm_gain} "
          f"wrist={args.wrist_gain}", flush=True)

    n_cmd, el, q = c.run(args.ramp, args.hold)
    err = np.abs(q - HOLD_Q)
    mean_err = err.mean(axis=0)
    max_err = err.max(axis=0)

    print(f"\n== MM3 (a1) suspended-base PD hold: {args.hold:.0f}s, "
          f"{n_cmd} commands at {n_cmd/el:.2f} Hz, {len(q)} state samples ==\n")
    print(f"{'idx':>3} {'joint':<22} {'target':>8} {'mean err':>9} {'max err':>9}  verdict")
    fails = []
    for i in range(G1_NUM_MOTOR):
        ok = mean_err[i] < GATE_RAD
        if not ok:
            fails.append(i)
        print(f"{i:3d} {SDK_NAMES[i]:<22} {HOLD_Q[i]:+8.3f} {mean_err[i]:9.5f} "
              f"{max_err[i]:9.5f}  {'PASS' if ok else 'FAIL'}")
    print(f"\n  worst joint: {SDK_NAMES[int(mean_err.argmax())]} "
          f"at {mean_err.max():.5f} rad (gate < {GATE_RAD})")
    print(f"  all-joint mean: {mean_err.mean():.5f} rad")
    print(f"\n(a1) VERDICT: {'PASS' if not fails else 'FAIL on ' + ', '.join(SDK_NAMES[i] for i in fails)}")
    return 0 if not fails else 1


if __name__ == "__main__":
    raise SystemExit(main())
