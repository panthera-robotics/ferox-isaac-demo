"""The DDS half of the low-level bridge: rt/lowstate out, rt/lowcmd in.

Runs inside ferox/twin-lowlevel:humble, never inside Isaac's python.  Talks to the
sim through the seqlock segments in shm.py.

Rates, which are two different numbers (contract `lowlevel`):
  * rt/lowstate publishes at 1041.68 Hz -- MEASURED off the DT bag, not the 500 Hz
    the campaign text asked for.  See docs/mm/evidence/MM3/PREREQS.md.
  * PD runs at 500 Hz and is applied on the sim side, not here.

On `tick`.  The robot's tick is a millisecond counter (measured 1000.064/s), and it
publishes 1041.68 messages against it, so ~4% of its messages repeat a tick while
carrying fresh IMU.  This bridge does NOT synthesise that: it sets
tick = round(sim_time_ms) and lets the same arithmetic produce the same consequence.
Incrementing tick once per message -- what the upstream Unitree reference does --
would put the field at 1041.68/s and contradict the semantics we measured.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import shm  # noqa: E402

from unitree_sdk2py.core.channel import (  # noqa: E402
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (  # noqa: E402
    LowState_, LowCmd_, HandState_, HandCmd_, MotorState_, PressSensorState_, IMUState_)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_  # noqa: E402
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

# Field-semantics constants, all measured off the bag. Anything the robot leaves at
# zero is left at zero here too: a twin that fills fields the robot does not fill is
# a twin the consumer can distinguish from the robot.
N_LIVE = 29             # motor slots 0..28 driven; 29..34 identically zero
MODE_MACHINE = 5
MODE_PR = 0
MOTOR_MODE_LIVE = 1
IMU_TEMP_C = 80
MOTOR_TEMP_C = 40       # bag range 31..48; a constant mid-band, declared as C-26
MOTOR_VOL_V = 47.5      # bag range 45.5..49.5

# Dex5-1P is 20 DoF per hand and 12 contact zones. NOT the 7 that
# unitree_sdk2py.idl.default's HandCmd_/HandState_ factories pre-fill: those helpers
# are Dex3-shaped, and both the DDS IDL (types.sequence) and the ROS msg (MotorCmd[])
# are unbounded, so 20 is legal on the wire. The factories are deliberately unused
# here and the messages are built by hand -- contract lowlevel.dex3_wire.
N_HAND_JOINTS = 20
N_ZONES = 12
N_PRESSURE = 12
DEX3_DECIMATION = 5     # 1041.68 / 5 = 208.3 Hz, against a >=200 Hz gate


def _monotonic_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


class LowLevelDDS:
    def __init__(self, domain: int, iface: str, publish_hz: float, cmd_timeout_ms: float):
        self.publish_hz = publish_hz
        self.cmd_timeout_ns = int(cmd_timeout_ms * 1e6)
        self.crc = CRC()

        ChannelFactoryInitialize(domain, iface) if iface else ChannelFactoryInitialize(domain)

        # Attach only -- the sim owns both segments. See sim_side.py on why.
        self.state = self._attach(shm.open_state, "state")
        self.cmd = self._attach(shm.open_cmd, "cmd")

        self.low_state = unitree_hg_msg_dds__LowState_()
        self._prime_static_fields()

        self.pub = ChannelPublisher("rt/lowstate", LowState_)
        self.pub.Init()
        # rt/secondary_imu -- the torso IMU. SONIC's deploy subscribes to it
        # (robot_parameters.hpp:28 HG_IMU_TORSO) and will not enter its control loop
        # without it, failing with "LowState or IMUState is not available" and then
        # timing out the planner handshake. It is NOT part of LowState_, so a twin that
        # publishes a perfect rt/lowstate still looks like a robot with no torso IMU.
        self.imu_pub = ChannelPublisher("rt/secondary_imu", IMUState_)
        self.imu_pub.Init()
        self.imu_msg = IMUState_(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0] * 3,
                                 accelerometer=[0.0] * 3, rpy=[0.0] * 3,
                                 temperature=IMU_TEMP_C)
        self.n_imu = 0
        self.sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.sub.Init(self._on_lowcmd, 32)

        self.hand_pub = {}
        self.hand_msg = {}
        for side in ("left", "right"):
            p = ChannelPublisher(f"rt/dex3/{side}/state", HandState_)
            p.Init()
            self.hand_pub[side] = p
            self.hand_msg[side] = self._new_hand_state()
            sub = ChannelSubscriber(f"rt/dex3/{side}/cmd", HandCmd_)
            sub.Init(lambda msg, s=side: self._on_handcmd(msg, s), 16)
            setattr(self, f"_sub_{side}", sub)

        self.n_hand_pub = 0
        self.n_hand_cmd = {"left": 0, "right": 0}
        self.n_pub = 0
        self.n_cmd = 0
        self.n_crc_bad = 0
        self.n_nan_cmd = 0
        self._last_good = None
        # rt/lowstate is paced on WALL time, because the consumer -- an sdk2 client,
        # and SONIC in MM4 -- runs in wall time and expects ~1 kHz of state per real
        # second. The twin's SENSOR topics are paced in sim time (DT2 gated
        # /livox/lidar at "10.00 Hz exactly in sim time"), so the two halves of the
        # twin genuinely use different clocks and that is deliberate.
        #
        # The cost is measured rather than assumed: whenever the sim advances slower
        # than real time, consecutive messages carry the SAME state, and this counts
        # how often. It is the honest version of the robot's own 4% duplicate ticks.
        self._last_sim_time = None
        self.n_repeat = 0

    @staticmethod
    def _attach(opener, what: str, timeout_s: float = 120.0):
        t0 = time.perf_counter()
        while True:
            try:
                return opener(create=False)
            except FileNotFoundError:
                if time.perf_counter() - t0 > timeout_s:
                    raise RuntimeError(
                        f"no {what} segment after {timeout_s:.0f}s -- is the sim up "
                        f"with G1_CONTROL=lowcmd?")
                time.sleep(0.25)

    def _prime_static_fields(self) -> None:
        # MUJOCO_COMPAT -- a C-39 A/B probe, not a mode of the bridge.
        #
        # The A/B established that SONIC stands in the reference MuJoCo sim and falls in
        # the twin, with the same image, deploy binary, driver and DDS seam. The
        # field-by-field diff (evidence/MM4/ab/) shows every remaining divergence is one
        # where the TWIN IS MORE ROBOT-FAITHFUL than MuJoCo: a valid CRC where MuJoCo
        # sends 0, mode_machine 5 where it sends 0, motor mode 1 where it sends 0,
        # specific-force accel where it sends world linear acceleration (~0 standing),
        # populated rpy and imu temperature where it sends zeros.
        #
        # SONIC was only ever validated against the less faithful one. This flag makes
        # the twin lie in exactly MuJoCo's way so the hypothesis can be tested, and it is
        # DIAGNOSTIC: shipping it would mean degrading the twin to match a reference
        # bridge's omissions, which is the opposite of the point.
        self._mj_compat = os.environ.get("G1_LL_MUJOCO_COMPAT", "0") == "1"
        ls = self.low_state
        if self._mj_compat:
            print("[lowlevel-dds] DIAGNOSTIC MUJOCO_COMPAT: mode_machine=0, motor mode=0,"
                  " accel/rpy/temp zeroed, crc=0 -- NOT A DELIVERABLE", flush=True)
            ls.mode_pr = 0
            ls.mode_machine = 0
            ls.version = [0, 0]
            ls.imu_state.temperature = 0
            for ms in ls.motor_state:
                ms.mode = 0
                ms.temperature = [0, 0]
                ms.vol = 0.0
            return
        ls.mode_pr = MODE_PR
        ls.mode_machine = MODE_MACHINE
        ls.version = [0, 0]
        ls.imu_state.temperature = IMU_TEMP_C
        for i, ms in enumerate(ls.motor_state):
            live = i < N_LIVE
            ms.mode = MOTOR_MODE_LIVE if live else 0
            ms.temperature = [MOTOR_TEMP_C if live else 0, 0]
            ms.vol = MOTOR_VOL_V if live else 0.0
            # ddq stays 0.0 forever: the robot never populates it (whole-bag scan).

    @staticmethod
    def _new_hand_state() -> HandState_:
        """A 20-entry HandState_, built explicitly rather than from the Dex3 factory."""
        return HandState_(
            motor_state=[MotorState_(mode=1, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0,
                                     temperature=[0, 0], vol=0.0, sensor=[0, 0],
                                     motorstate=0, reserve=[0, 0, 0, 0])
                         for _ in range(N_HAND_JOINTS)],
            press_sensor_state=[PressSensorState_(pressure=[0.0] * N_PRESSURE,
                                                 temperature=[0.0] * N_PRESSURE,
                                                 lost=0, reserve=0)
                                for _ in range(N_ZONES)],
            imu_state=IMUState_(quaternion=[1.0, 0.0, 0.0, 0.0], gyroscope=[0.0] * 3,
                                accelerometer=[0.0] * 3, rpy=[0.0] * 3, temperature=0),
            power_v=0.0, power_a=0.0, system_v=0.0, device_v=0.0,
            error=[0, 0], reserve=[0, 0])

    def _on_handcmd(self, msg: HandCmd_, side: str) -> None:
        k = 0 if side == "left" else 1
        o = k * N_HAND_JOINTS
        rec = self.cmd.read()
        q_d = np.array(rec["hand_q_d"], np.float32) if rec is not None \
            else np.zeros(shm.N_HAND, np.float32)
        kp = np.array(rec["hand_kp"], np.float32) if rec is not None \
            else np.zeros(shm.N_HAND, np.float32)
        kd = np.array(rec["hand_kd"], np.float32) if rec is not None \
            else np.zeros(shm.N_HAND, np.float32)
        stamps = np.array(rec["hand_stamp_ns"], np.uint64) if rec is not None \
            else np.zeros(2, np.uint64)
        n = min(len(msg.motor_cmd), N_HAND_JOINTS)
        for i in range(n):
            mc = msg.motor_cmd[i]
            q_d[o + i], kp[o + i], kd[o + i] = mc.q, mc.kp, mc.kd
        stamps[k] = _monotonic_ns()
        self.n_hand_cmd[side] += 1
        # HandCmd_ carries no CRC field, so unlike rt/lowcmd there is nothing to
        # verify here; the arrival itself is the liveness signal. Stated because the
        # asymmetry with _on_lowcmd otherwise looks like an omission.
        self.cmd.write(hand_q_d=q_d, hand_kp=kp, hand_kd=kd, hand_stamp_ns=stamps)

    def _publish_hands(self, rec) -> None:
        for k, side in enumerate(("left", "right")):
            msg = self.hand_msg[side]
            o = k * N_HAND_JOINTS
            for i in range(N_HAND_JOINTS):
                ms = msg.motor_state[i]
                ms.q = float(rec["hand_q"][o + i])
                ms.dq = float(rec["hand_dq"][o + i])
                ms.tau_est = float(rec["hand_tau"][o + i])
            # press_sensor_state stays zero: the 12 contact zones per hand are not in
            # the asset yet (DT3 left them to DT6, which Mohammed deferred). The wire
            # SHAPE is real and gated; the values are declared absent -- C-27. Filling
            # them with synthetic pressure would be the one thing worse than zeros.
            for i in range(N_ZONES):
                msg.press_sensor_state[i].pressure[:] = [float(rec["contact"][k][i])] * N_PRESSURE
            self.hand_pub[side].Write(msg)
        self.n_hand_pub += 1

    # ------------------------------------------------------------ rt/lowcmd in

    def _on_lowcmd(self, msg: LowCmd_) -> None:
        # CRC first. A command that fails CRC is dropped WITHOUT refreshing the
        # watchdog stamp, so a sender emitting corrupt frames at full rate still
        # trips the 100 ms fail-closed path instead of holding the robot up with
        # garbage. That is the difference between "a message arrived" and "a
        # command arrived", and only the second one should count as liveness.
        if self.crc.Crc(msg) != msg.crc:
            self.n_crc_bad += 1
            return

        # Reject a command carrying NaN/Inf outright. Measured: once SONIC's own state
        # estimate diverges on a fallen robot it can emit non-finite targets, and the
        # bridge computed tau = kp*(NaN - q) and handed NaN to PhysX, which poisons the
        # articulation for good. A non-finite command is not a command -- dropping it
        # here means the watchdog sees silence and fail-closed damping takes over, which
        # is the correct response to a controller that has lost its mind.
        vals = [(mc.q, mc.dq, mc.tau, mc.kp, mc.kd) for mc in msg.motor_cmd]
        if not np.all(np.isfinite(np.asarray(vals, dtype=np.float64))):
            self.n_nan_cmd += 1
            return

        n = min(len(msg.motor_cmd), shm.N_MOTOR)
        q_d = np.zeros(shm.N_MOTOR, np.float32)
        dq_d = np.zeros(shm.N_MOTOR, np.float32)
        kp = np.zeros(shm.N_MOTOR, np.float32)
        kd = np.zeros(shm.N_MOTOR, np.float32)
        tau = np.zeros(shm.N_MOTOR, np.float32)
        for i in range(n):
            mc = msg.motor_cmd[i]
            q_d[i], dq_d[i], kp[i], kd[i], tau[i] = mc.q, mc.dq, mc.kp, mc.kd, mc.tau

        self.n_cmd += 1
        self.cmd.write(stamp_ns=_monotonic_ns(), cmd_count=self.n_cmd,
                       mode_pr=int(msg.mode_pr), mode_machine=int(msg.mode_machine),
                       q_d=q_d, dq_d=dq_d, kp=kp, kd=kd, tau_ff=tau)

    # --------------------------------------------------------- rt/lowstate out

    def _fill(self, rec) -> bool:
        ls = self.low_state
        q, dq, tau = rec["q"], rec["dq"], rec["tau_est"]
        for i in range(N_LIVE):
            ms = ls.motor_state[i]
            ms.q = float(q[i]); ms.dq = float(dq[i]); ms.tau_est = float(tau[i])

        imu = ls.imu_state
        imu.quaternion = [float(x) for x in rec["quat_wxyz"]]
        imu.gyroscope = [float(x) for x in rec["gyro"]]
        if self._mj_compat:
            imu.accelerometer = [0.0, 0.0, 0.0]
            imu.rpy = [0.0, 0.0, 0.0]
        else:
            imu.accelerometer = [float(x) for x in rec["accel"]]
            imu.rpy = [float(x) for x in rec["rpy"]]

        # tick is MILLISECONDS of sim time, not a message counter -- see module docstring.
        ls.tick = int(round(float(rec["sim_time"]) * 1000.0)) & 0xFFFFFFFF
        ls.crc = 0 if self._mj_compat else self.crc.Crc(ls)
        return True

    def spin(self, duration_s: float | None, stats_every_s: float = 5.0):
        period = 1.0 / self.publish_hz
        t0 = time.perf_counter()
        deadline = t0 + period
        next_stats = t0 + stats_every_s
        stale_reads = 0

        while True:
            now = time.perf_counter()
            if duration_s is not None and now - t0 >= duration_s:
                break

            rec = self.state.read()
            if rec is None:
                # A torn seqlock read (writer at 1000 Hz, reader at 1041.68 Hz, so
                # they collide) must NOT cost a message. The robot never skips a
                # publish; it repeats the state it has, which is exactly what its
                # duplicate ticks are. Skipping here instead cost ~0.13% of the rate
                # in the standalone run and would have been a self-inflicted miss
                # against a +-2% gate.
                stale_reads += 1
                rec = self._last_good
            else:
                self._last_good = rec
            if rec is not None:
                st = float(rec["sim_time"])
                if st == self._last_sim_time:
                    self.n_repeat += 1
                self._last_sim_time = st
                self._fill(rec)
                self.pub.Write(self.low_state)
                self.n_pub += 1
                # Published at the SAME rate as rt/lowstate: the driver probe measured
                # the robot's /secondary_imu at 1041.7 Hz, matching /lowstate to four
                # significant figures (docs/mm/evidence/MM3/PREREQS.md).
                imu = self.imu_msg
                imu.quaternion = [float(v) for v in rec["torso_quat_wxyz"]]
                imu.gyroscope = [float(v) for v in rec["torso_gyro"]]
                imu.accelerometer = [float(v) for v in rec["torso_accel"]]
                imu.rpy = [float(v) for v in rec["torso_rpy"]]
                self.imu_pub.Write(imu)
                self.n_imu += 1
                if self.n_pub % DEX3_DECIMATION == 0:
                    self._publish_hands(rec)

            if now >= next_stats:
                el = now - t0
                print(f"[lowlevel-dds] t={el:6.1f}s  lowstate={self.n_pub} "
                      f"({self.n_pub/el:8.3f} Hz)  lowcmd={self.n_cmd} "
                      f"({self.n_cmd/el:7.2f} Hz)  crc_bad={self.n_crc_bad} nan_cmd={self.n_nan_cmd} "
                      f"imu2={self.n_imu} ({self.n_imu/el:8.3f} Hz)  "
                      f"repeat={100*self.n_repeat/max(1,self.n_pub):5.1f}%  "
                      f"sim_stale={stale_reads}  dex3_state={self.n_hand_pub} "
                      f"({self.n_hand_pub/el:6.2f} Hz)  dex3_cmd={self.n_hand_cmd}",
                      flush=True)
                next_stats += stats_every_s

            # Absolute-deadline pacing. Sleeping `period` each pass would accumulate
            # the loop's own cost as drift -- at 1041.68 Hz a 40 us body is a 4%
            # rate error, which is the whole +-2% gate twice over. Sleep to just
            # short of the deadline, then spin: nanosleep granularity is ~50 us and
            # the period is only 960 us.
            deadline += period
            slack = deadline - time.perf_counter()
            if slack > 250e-6:
                time.sleep(slack - 200e-6)
            while time.perf_counter() < deadline:
                pass

        el = time.perf_counter() - t0
        return {"seconds": el, "lowstate_msgs": self.n_pub,
                "lowstate_hz": self.n_pub / el, "lowcmd_msgs": self.n_cmd,
                "lowcmd_hz": self.n_cmd / el, "crc_bad": self.n_crc_bad, "nan_cmd_dropped": self.n_nan_cmd,
                "sim_stale_reads": stale_reads}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--publish-hz", type=float, default=1041.68,
                    help="contract lowlevel.lowstate_publish_hz -- measured, not chosen")
    ap.add_argument("--cmd-timeout-ms", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    print(f"[lowlevel-dds] domain={args.domain} publish_hz={args.publish_hz} "
          f"cmd_timeout={args.cmd_timeout_ms} ms", flush=True)
    b = LowLevelDDS(args.domain, args.iface, args.publish_hz, args.cmd_timeout_ms)
    print("[lowlevel-dds] waiting for the sim to write state...", flush=True)
    stats = b.spin(args.duration)
    print("[lowlevel-dds] RESULT " + " ".join(f"{k}={v}" for k, v in stats.items()), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
