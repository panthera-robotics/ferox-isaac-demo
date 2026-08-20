#!/usr/bin/env python3
"""MM3 test (a) + (b): stand the twin from unitree_sdk2py over rt/lowcmd, then cut the
commands and check the bridge fails closed.

Shaped after unitree_sdk2py's own example/g1/low_level/g1_low_level_example.py: same
LowCmd_ construction, same CRC, same 500 Hz control thread, same G1JointIndex order.
Two deliberate departures, both because the twin is not a robot:

  * MotionSwitcherClient is not called. It is a robot-side service that releases the
    built-in motion mode; the twin has no built-in mode to release, and stubbing the
    service would prove nothing about the wire. Declared, not faked.
  * The stand target is the twin's own deploy.yaml default pose rather than the
    example's all-zero posture. Zero posture on a G1 is straight-legged and the
    example only survives it because a real robot is on a gantry when you run it.

Runs inside ferox/twin-lowlevel:humble.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

G1_NUM_MOTOR = 29
MODE_PR = 0

# deploy.yaml default_joint_pos is in SIM order; joint_ids_map[sim] = sdk. Reordered
# here once, at import, rather than in the control loop.
_DEFAULT_SIM = [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3, 0.3,
                -0.2, -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97, 0.15, -0.15,
                0.0, 0.0, 0.0, 0.0]
_IDS_MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
            18, 25, 19, 26, 20, 27, 21, 28]
STAND_Q = np.zeros(G1_NUM_MOTOR, dtype=np.float32)
for _sim, _sdk in enumerate(_IDS_MAP):
    STAND_Q[_sdk] = _DEFAULT_SIM[_sim]

# Gains in SDK order, from the sdk2py example (legs/waist/arms bands).
KP = np.array([60, 60, 60, 100, 40, 40, 60, 60, 60, 100, 40, 40, 60, 40, 40,
               40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40, 40], np.float32)
KD = np.array([1, 1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
               1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], np.float32)


class StandClient:
    def __init__(self, domain: int, iface: str, control_hz: float):
        ChannelFactoryInitialize(domain, iface) if iface else ChannelFactoryInitialize(domain)
        self.crc = CRC()
        self.control_dt = 1.0 / control_hz
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.mode_machine = None

        self.state_stamps: list[float] = []
        self.rpy: list[tuple] = []
        self.q_log: list[np.ndarray] = []

        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_state, 10)

    def _on_state(self, msg: LowState_) -> None:
        self.low_state = msg
        if self.mode_machine is None:
            # Learned from the twin exactly as the example learns it from the robot.
            self.mode_machine = msg.mode_machine
        self.state_stamps.append(time.perf_counter())
        self.rpy.append(tuple(float(v) for v in msg.imu_state.rpy))
        self.q_log.append(np.array([msg.motor_state[i].q for i in range(G1_NUM_MOTOR)],
                                   dtype=np.float32))

    def wait_for_state(self, timeout_s: float = 20.0) -> bool:
        t0 = time.perf_counter()
        while self.mode_machine is None:
            if time.perf_counter() - t0 > timeout_s:
                return False
            time.sleep(0.05)
        return True

    def _write(self, q_d: np.ndarray) -> None:
        self.low_cmd.mode_pr = MODE_PR
        self.low_cmd.mode_machine = self.mode_machine
        for i in range(G1_NUM_MOTOR):
            mc = self.low_cmd.motor_cmd[i]
            mc.mode = 1
            mc.q = float(q_d[i]); mc.dq = 0.0; mc.tau = 0.0
            mc.kp = float(KP[i]); mc.kd = float(KD[i])
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.pub.Write(self.low_cmd)

    def run(self, ramp_s: float, hold_s: float) -> dict:
        q0 = np.array([self.low_state.motor_state[i].q for i in range(G1_NUM_MOTOR)],
                      dtype=np.float32)
        t0 = time.perf_counter()
        deadline = t0 + self.control_dt
        n_cmd = 0
        state_mark = len(self.state_stamps)

        while True:
            t = time.perf_counter() - t0
            if t >= ramp_s + hold_s:
                break
            ratio = min(t / ramp_s, 1.0) if ramp_s > 0 else 1.0
            self._write((1.0 - ratio) * q0 + ratio * STAND_Q)
            n_cmd += 1
            deadline += self.control_dt
            slack = deadline - time.perf_counter()
            if slack > 250e-6:
                time.sleep(slack - 200e-6)
            while time.perf_counter() < deadline:
                pass

        el = time.perf_counter() - t0
        # Only the HOLD window is scored: the ramp is a transient by construction and
        # scoring tracking error through it would mostly measure the ramp's own slope.
        hold_from = t0 + ramp_s
        idx = [i for i, s in enumerate(self.state_stamps[state_mark:], state_mark)
               if s >= hold_from]
        rpy = np.array([self.rpy[i] for i in idx]) if idx else np.zeros((1, 3))
        qs = np.array([self.q_log[i] for i in idx]) if idx else np.zeros((1, G1_NUM_MOTOR))
        err = np.abs(qs - STAND_Q)

        stamps = np.array(self.state_stamps[state_mark:])
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
        return {
            "seconds": el, "lowcmd_sent": n_cmd, "lowcmd_hz": n_cmd / el,
            "lowstate_recv": len(stamps), "lowstate_hz": rate,
            "hold_samples": len(idx),
            "roll_abs_max": float(np.abs(rpy[:, 0]).max()),
            "pitch_abs_max": float(np.abs(rpy[:, 1]).max()),
            "roll_rms": float(np.sqrt((rpy[:, 0] ** 2).mean())),
            "pitch_rms": float(np.sqrt((rpy[:, 1] ** 2).mean())),
            "track_err_mean_rad": float(err.mean()),
            "track_err_max_rad": float(err.max()),
            "track_err_max_joint": int(err.max(axis=0).argmax()),
        }

    def observe_after_silence(self, seconds: float) -> dict:
        """Test (b): stop commanding and watch what the bridge does."""
        mark = len(self.state_stamps)
        t0 = time.perf_counter()
        time.sleep(seconds)
        idx = list(range(mark, len(self.state_stamps)))
        if not idx:
            return {"error": "no lowstate during silence -- bridge stopped publishing"}
        stamps = np.array([self.state_stamps[i] for i in idx])
        rpy = np.array([self.rpy[i] for i in idx])
        qs = np.array([self.q_log[i] for i in idx])
        # Under damping-only the pose must DRIFT off the commanded stand target; if it
        # held the target exactly, the bridge kept tracking a stale command, which is
        # the failure test (b) exists to catch.
        drift = np.abs(qs[-1] - STAND_Q)
        return {
            "silence_s": time.perf_counter() - t0,
            "lowstate_recv": len(idx),
            "lowstate_hz": (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0,
            "roll_abs_max": float(np.abs(rpy[:, 0]).max()),
            "pitch_abs_max": float(np.abs(rpy[:, 1]).max()),
            "pose_drift_from_target_mean_rad": float(drift.mean()),
            "pose_drift_from_target_max_rad": float(drift.max()),
            "joint_speed_final_max": float(np.abs(qs[-1] - qs[-2]).max()) if len(qs) > 1 else 0.0,
        }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--control-hz", type=float, default=500.0)
    ap.add_argument("--ramp", type=float, default=3.0)
    ap.add_argument("--hold", type=float, default=60.0)
    ap.add_argument("--silence", type=float, default=3.0,
                    help="test (b): seconds to stop commanding after the stand")
    args = ap.parse_args()

    c = StandClient(args.domain, args.iface, args.control_hz)
    print("[mm3-stand] waiting for rt/lowstate...", flush=True)
    if not c.wait_for_state():
        print("[mm3-stand] FAIL: no rt/lowstate within 20 s", flush=True)
        return 1
    print(f"[mm3-stand] rt/lowstate up, mode_machine={c.mode_machine}", flush=True)

    stand = c.run(args.ramp, args.hold)
    print("[mm3-stand] STAND " + " ".join(f"{k}={v}" for k, v in stand.items()), flush=True)

    if args.silence > 0:
        quiet = c.observe_after_silence(args.silence)
        print("[mm3-stand] SILENCE " + " ".join(f"{k}={v}" for k, v in quiet.items()), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
