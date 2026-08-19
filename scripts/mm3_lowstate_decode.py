#!/usr/bin/env python3
"""Decode /lowstate from the DT bag and dump the field-semantics reference for MM3 (c).

Runs only inside ferox/twin-lowlevel:humble -- the whole point of that image is that
`unitree_hg/msg/LowState` has no deserializer anywhere else on this box.

Two jobs:
  * the tick audit, which settles the 851-vs-1041 Hz question left open in
    docs/mm/evidence/MM3/PREREQS.md.  If `tick` advances 1:1 with messages then the
    bag stream is lossless and the driver probe's 851.4 Hz is downstream loss.
  * the parity reference: which fields the robot actually populates, and with what,
    so the twin is gated on matching real semantics instead of on filling in
    every field the IDL happens to have.
"""

from __future__ import annotations

import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from unitree_hg.msg import LowState

TOPIC = "/lowstate"


def read(bag_dir: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    while reader.has_next():
        _, data, stamp = reader.read_next()
        yield stamp, deserialize_message(data, LowState)


def main(bag_dir: str) -> int:
    stamps, msgs = [], []
    for stamp, m in read(bag_dir):
        stamps.append(stamp)
        msgs.append(m)
    if not msgs:
        print(f"no {TOPIC} messages in {bag_dir}", file=sys.stderr)
        return 2

    ts = np.array(stamps, dtype=np.int64)
    span = (ts[-1] - ts[0]) / 1e9
    msg_rate = (len(ts) - 1) / span
    tick = np.array([m.tick for m in msgs], dtype=np.int64)

    print(f"=== {TOPIC} decoded: {len(msgs)} msgs over {span:.4f} s ===\n")

    # -------------------------------------------------------------- tick audit
    print("--- tick audit (the 851 vs 1041 question) ---")
    dtick = np.diff(tick)
    tick_span = int(tick[-1] - tick[0])
    tick_rate = tick_span / span
    uniq = len(np.unique(tick))
    print(f"  tick range {tick[0]}..{tick[-1]}  span={tick_span}")
    print(f"  tick advance rate = {tick_rate:.3f} /s   vs message rate {msg_rate:.3f} Hz")
    print(f"  ratio tick/msg = {tick_rate/msg_rate:.6f}")
    print(f"  unique ticks {uniq} of {len(tick)}  -> duplicated-tick msgs "
          f"{len(tick)-uniq} ({100*(len(tick)-uniq)/len(tick):.2f}%)")
    vals, counts = np.unique(dtick, return_counts=True)
    print("  tick deltas (delta, count, %):")
    for v, c in sorted(zip(vals, counts), key=lambda x: -x[1])[:6]:
        print(f"    {v:6d}  {c:7d}  {100*c/len(dtick):6.2f}%")
    gaps = int((dtick > 1).sum())
    print(f"  ticks skipped (delta>1): {gaps} events, {int(dtick[dtick>1].sum() - gaps)} ticks lost")

    # Duplicate-tick pairs that carry DIFFERENT accelerometer values are the thing
    # the driver probe flagged; if they exist here too, the behaviour is upstream of
    # the driver and belongs in the deviations table rather than in a driver bug.
    disagree = 0
    for i in np.where(dtick == 0)[0]:
        if tuple(msgs[i].imu_state.accelerometer) != tuple(msgs[i + 1].imu_state.accelerometer):
            disagree += 1
    print(f"  duplicate-tick pairs disagreeing on accel: {disagree}\n")

    # ------------------------------------------------------- field semantics
    m = msgs[len(msgs) // 2]
    print("--- reference message (bag midpoint) ---")
    print(f"  version        = {list(m.version)}")
    print(f"  mode_pr        = {m.mode_pr}     mode_machine = {m.mode_machine}")
    print(f"  tick           = {m.tick}")
    print(f"  crc            = {m.crc}")
    print(f"  imu quaternion = {[round(float(x),6) for x in m.imu_state.quaternion]}  (w,x,y,z order per IDL)")
    print(f"  imu gyroscope  = {[round(float(x),6) for x in m.imu_state.gyroscope]}")
    print(f"  imu accel      = {[round(float(x),6) for x in m.imu_state.accelerometer]}")
    print(f"  imu rpy        = {[round(float(x),6) for x in m.imu_state.rpy]}")
    print(f"  imu temp       = {m.imu_state.temperature}")
    print(f"  wireless_remote nonzero bytes = {int(np.count_nonzero(list(m.wireless_remote)))} of 40")
    print(f"  reserve        = {list(m.reserve)}")
    print(f"  motor_state len= {len(m.motor_state)}")

    # Which of the 35 motor slots the robot actually drives, and which fields it fills.
    print("\n--- motor_state occupancy across the whole bag ---")
    q = np.array([[ms.q for ms in mm.motor_state] for mm in msgs], dtype=np.float64)
    dq = np.array([[ms.dq for ms in mm.motor_state] for mm in msgs], dtype=np.float64)
    tau = np.array([[ms.tau_est for ms in mm.motor_state] for mm in msgs], dtype=np.float64)
    mode = np.array([[ms.mode for ms in mm.motor_state] for mm in msgs], dtype=np.int64)
    live = np.where(~np.all(q == 0, axis=0) | ~np.all(mode == 0, axis=0))[0]
    print(f"  slots with any nonzero q or mode: {len(live)} -> {list(map(int, live))}")
    print(f"  slots identically zero everywhere: {sorted(set(range(35)) - set(map(int, live)))}")
    print(f"  distinct motor.mode values: {sorted(map(int, np.unique(mode)))}")
    print("\n  idx     q[min..max]            dq|max|     tau_est[min..max]     mode")
    for i in range(35):
        flag = "" if i in live else "   (dead slot)"
        print(f"  {i:3d}  {q[:,i].min():9.5f}..{q[:,i].max():9.5f}  {np.abs(dq[:,i]).max():9.5f}  "
              f"{tau[:,i].min():9.4f}..{tau[:,i].max():9.4f}  {sorted(map(int,np.unique(mode[:,i])))}{flag}")

    print(f"\n  ddq all zero: {bool(np.all(np.array([[ms.ddq for ms in mm.motor_state] for mm in msgs]) == 0))}")
    temps = np.array([[ms.temperature[0] for ms in mm.motor_state] for mm in msgs])
    print(f"  temperature[0] range over live slots: {temps[:, live].min()}..{temps[:, live].max()}")
    vol = np.array([[ms.vol for ms in mm.motor_state] for mm in msgs])
    print(f"  vol range over live slots: {vol[:, live].min():.3f}..{vol[:, live].max():.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else
                          "/bag"))
