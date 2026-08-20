#!/usr/bin/env python3
"""MM3 test (c): rt/lowstate rate AND field-by-field parity against the DT bag.

The reference is docs/mm/evidence/MM3/lowstate_decode_bag.txt -- what the ROBOT
actually puts in each field, taken from all 35998 messages of the ground-truth
capture, not what the IDL happens to allow. A twin that fills fields the robot leaves
empty is as distinguishable from the robot as one that leaves fields empty that the
robot fills, so both directions are checked.

Runs inside ferox/twin-lowlevel:humble.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC

# Reference, measured off the bag. See lowstate_decode_bag.txt.
REF = dict(motor_len=35, live=range(0, 29), dead=range(29, 35), live_mode=1,
           dead_mode=0, version=[0, 0], mode_pr=0, mode_machine=5,
           bag_rate_hz=1041.677)
BAND = 0.02


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    ChannelFactoryInitialize(args.domain, args.iface) if args.iface \
        else ChannelFactoryInitialize(args.domain)
    msgs, stamps = [], []
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(lambda m: (msgs.append(m), stamps.append(time.perf_counter())), 64)

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < args.seconds:
        time.sleep(0.05)
    if len(msgs) < 10:
        print(f"FAIL: only {len(msgs)} rt/lowstate messages in {args.seconds}s")
        return 1

    ts = np.array(stamps)
    rate = (len(ts) - 1) / (ts[-1] - ts[0])
    lo, hi = REF["bag_rate_hz"] * (1 - BAND), REF["bag_rate_hz"] * (1 + BAND)
    checks: list[tuple[str, bool, str]] = []
    ok_rate = lo <= rate <= hi
    checks.append(("rate in bag band +-2%", ok_rate,
                   f"{rate:.3f} Hz in [{lo:.2f}, {hi:.2f}] (bag {REF['bag_rate_hz']:.3f})"))

    m = msgs[len(msgs) // 2]
    q = np.array([[ms.q for ms in x.motor_state] for x in msgs])
    ddq = np.array([[ms.ddq for ms in x.motor_state] for x in msgs])
    mode = np.array([[ms.mode for ms in x.motor_state] for x in msgs])

    checks.append(("motor_state length 35", len(m.motor_state) == REF["motor_len"],
                   str(len(m.motor_state))))
    checks.append(("slots 0..28 carry mode=1",
                   bool((mode[:, list(REF["live"])] == REF["live_mode"]).all()),
                   f"distinct={sorted(map(int, np.unique(mode[:, list(REF['live'])])))}"))
    checks.append(("slots 29..34 identically zero",
                   bool((q[:, list(REF["dead"])] == 0).all()
                        and (mode[:, list(REF["dead"])] == REF["dead_mode"]).all()),
                   f"|q|max={float(np.abs(q[:, list(REF['dead'])]).max()):.3e}"))
    checks.append(("ddq never populated (robot leaves it 0)",
                   bool((ddq == 0).all()), f"|ddq|max={float(np.abs(ddq).max()):.3e}"))
    checks.append(("version == [0, 0]", list(m.version) == REF["version"], str(list(m.version))))
    checks.append(("mode_pr == 0", int(m.mode_pr) == REF["mode_pr"], str(int(m.mode_pr))))
    checks.append(("mode_machine == 5", int(m.mode_machine) == REF["mode_machine"],
                   str(int(m.mode_machine))))
    checks.append(("wireless_remote all zero",
                   not any(m.wireless_remote), f"nonzero={int(np.count_nonzero(list(m.wireless_remote)))}"))
    checks.append(("reserve all zero", not any(m.reserve), str(list(m.reserve))))

    # tick is MILLISECONDS, so it must advance at ~1000/s against wall time, NOT at
    # the message rate. Getting this wrong is the single easiest way to look correct.
    tick = np.array([x.tick for x in msgs], dtype=np.int64)
    tick_rate = (tick[-1] - tick[0]) / (ts[-1] - ts[0])
    checks.append(("tick is a ms counter, not a message counter",
                   abs(tick_rate - 1000.0) / 1000.0 < 0.25,
                   f"{tick_rate:.1f}/s (robot 1000.064/s; message rate {rate:.1f} Hz)"))
    checks.append(("tick never decreases", bool((np.diff(tick) >= 0).all()),
                   f"min delta={int(np.diff(tick).min())}"))

    quat = np.array(m.imu_state.quaternion, dtype=np.float64)
    checks.append(("imu quaternion normalised, w-first",
                   abs(np.linalg.norm(quat) - 1.0) < 1e-3,
                   f"|q|={np.linalg.norm(quat):.6f} w={quat[0]:.4f}"))
    checks.append(("imu rpy populated (robot sends it, not derived)",
                   any(abs(float(v)) > 0 for v in m.imu_state.rpy)
                   or abs(float(m.imu_state.quaternion[0])) > 0.999,
                   str([round(float(v), 5) for v in m.imu_state.rpy])))
    acc = np.linalg.norm(np.array(m.imu_state.accelerometer, dtype=np.float64))
    checks.append(("accelerometer magnitude ~ g", abs(acc - 9.80665) < 0.5, f"{acc:.4f} m/s^2"))

    crc = CRC()
    bad = sum(1 for x in msgs[:200] if crc.Crc(x) != x.crc)
    checks.append(("crc populated and self-consistent", bad == 0,
                   f"{bad}/200 mismatched"))

    print(f"== MM3 (c) rt/lowstate parity, {len(msgs)} msgs over {ts[-1]-ts[0]:.2f}s ==\n")
    width = max(len(c[0]) for c in checks)
    for name, ok, detail in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<{width}}  {detail}")
    allok = all(c[1] for c in checks)
    print(f"\n(c) VERDICT: {'PASS' if allok else 'FAIL'}")
    return 0 if allok else 1


if __name__ == "__main__":
    raise SystemExit(main())
