#!/usr/bin/env python3
"""Dump rt/lowstate + rt/secondary_imu to JSON for a numerical A/B.

Written for the C-39 A/B: the same capture is taken from the reference MuJoCo sim and
from the twin, and the two JSONs are diffed field by field. Everything is reduced to
plain numbers -- means over the window, plus min/max where spread matters -- so the diff
is arithmetic and not eyeballing.
"""
from __future__ import annotations

import argparse, json, time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, IMUState_

N = 29

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="lo")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--label", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    ChannelFactoryInitialize(a.domain, a.iface)
    ls, imu, ts = [], [], []
    s1 = ChannelSubscriber("rt/lowstate", LowState_)
    s1.Init(lambda m: (ls.append(m), ts.append(time.perf_counter())), 64)
    s2 = ChannelSubscriber("rt/secondary_imu", IMUState_)
    s2.Init(lambda m: imu.append(m), 32)

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < a.seconds:
        time.sleep(0.05)
    if not ls:
        print(f"{a.label}: NO rt/lowstate received"); return 1

    q  = np.array([[m.motor_state[i].q       for i in range(N)] for m in ls])
    dq = np.array([[m.motor_state[i].dq      for i in range(N)] for m in ls])
    dd = np.array([[m.motor_state[i].ddq     for i in range(N)] for m in ls])
    tq = np.array([[m.motor_state[i].tau_est for i in range(N)] for m in ls])
    md = np.array([[m.motor_state[i].mode    for i in range(N)] for m in ls])
    m0 = ls[len(ls)//2]
    tick = np.array([m.tick for m in ls], dtype=np.int64)
    span = ts[-1] - ts[0] if len(ts) > 1 else 1.0

    d = {
        "label": a.label,
        "n_lowstate": len(ls), "n_secondary_imu": len(imu),
        "lowstate_hz": round((len(ls)-1)/span, 3),
        "secondary_imu_hz": round((len(imu)-1)/span, 3) if len(imu) > 1 else 0.0,
        "tick_rate_per_s": round(float(tick[-1]-tick[0])/span, 2),
        "mode_pr": int(m0.mode_pr), "mode_machine": int(m0.mode_machine),
        "version": [int(v) for v in m0.version],
        "imu_quat_wxyz": [round(float(v), 6) for v in m0.imu_state.quaternion],
        "imu_gyro":      [round(float(v), 6) for v in m0.imu_state.gyroscope],
        "imu_accel":     [round(float(v), 6) for v in m0.imu_state.accelerometer],
        "imu_accel_norm": round(float(np.linalg.norm(m0.imu_state.accelerometer)), 5),
        "imu_rpy":       [round(float(v), 6) for v in m0.imu_state.rpy],
        "imu_temp": int(m0.imu_state.temperature),
        "motor_mode_distinct": sorted(set(int(v) for v in md.ravel())),
        "ddq_absmax": round(float(np.abs(dd).max()), 6),
        "tau_absmax": round(float(np.abs(tq).max()), 4),
        "dq_absmax":  round(float(np.abs(dq).max()), 4),
        "q_mean":  [round(float(v), 5) for v in q.mean(axis=0)],
        "q_min":   [round(float(v), 5) for v in q.min(axis=0)],
        "q_max":   [round(float(v), 5) for v in q.max(axis=0)],
        "wireless_remote_nonzero": int(np.count_nonzero(list(m0.wireless_remote))),
        "reserve": [int(v) for v in m0.reserve],
        "crc": int(m0.crc),
    }
    if imu:
        i0 = imu[len(imu)//2]
        d["sec_imu_quat_wxyz"] = [round(float(v), 6) for v in i0.quaternion]
        d["sec_imu_gyro"]      = [round(float(v), 6) for v in i0.gyroscope]
        d["sec_imu_accel"]     = [round(float(v), 6) for v in i0.accelerometer]
        d["sec_imu_rpy"]       = [round(float(v), 6) for v in i0.rpy]
    json.dump(d, open(a.out, "w"), indent=2)
    print(f"{a.label}: {len(ls)} lowstate @ {d['lowstate_hz']} Hz, "
          f"{len(imu)} secondary_imu @ {d['secondary_imu_hz']} Hz -> {a.out}")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
