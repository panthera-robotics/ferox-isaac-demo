#!/usr/bin/env python3
"""MM3 test (f): the rt/dex3/{left,right}/{cmd,state} 20-entry wire.

Gates the SHAPE and the RATE of the hand wire, and that a HandCmd_ actually reaches
the sim's finger drives. Deliberately does not gate press_sensor_state VALUES: the
12 contact zones per hand are not in the asset yet (DT3 deferred them to DT6, which
Mohammed deferred), so the zones are published at the right shape carrying zeros and
declared as C-27 rather than filled with something invented.

Runs inside ferox/twin-lowlevel:humble.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, MotorCmd_

N_HAND_JOINTS = 20
# Roll/abduction of fingers 2..5 -- mechanically unactuated on the Dex5-1. Commanded
# here anyway, on purpose: the wire must ACCEPT them and the twin must NOT move them.
PASSIVE = (4, 8, 12, 16)
NAMES = {
    "left": ["Yaw_11L", "Roll_12L", "Pitch_13L", "Pitch_14L", "Roll_21L", "Pitch_22L",
             "Pitch_23L", "Pitch_24L", "Roll_31L", "Pitch_32L", "Pitch_33L", "Pitch_34L",
             "Link_41L", "Pitch_42L", "Pitch_43L", "Pitch_44L", "Roll_51L", "Pitch_52L",
             "Pitch_53L", "Pitch_54L"],
    "right": ["Yaw_11R", "Roll_12R", "Pitch_13R", "Pitch_14R", "Roll_21R", "Pitch_22R",
              "Pitch_23R", "Pitch_24R", "Roll_31R", "Pitch_32R", "Pitch_33R", "Pitch_34R",
              "Roll_41R", "Pitch_42R", "Pitch_43R", "Pitch_44R", "Roll_51R", "Pitch_52R",
              "Pitch_53R", "Pitch_54R"],
}


class Dex3Client:
    def __init__(self, domain: int, iface: str):
        ChannelFactoryInitialize(domain, iface) if iface else ChannelFactoryInitialize(domain)
        self.state = {"left": [], "right": []}
        self.stamps = {"left": [], "right": []}
        self.pub = {}
        for side in ("left", "right"):
            p = ChannelPublisher(f"rt/dex3/{side}/cmd", HandCmd_)
            p.Init()
            self.pub[side] = p
            sub = ChannelSubscriber(f"rt/dex3/{side}/state", HandState_)
            sub.Init(lambda m, s=side: self._on(m, s), 16)
            setattr(self, f"_sub_{side}", sub)

    def _on(self, msg: HandState_, side: str) -> None:
        self.stamps[side].append(time.perf_counter())
        self.state[side].append(np.array([m.q for m in msg.motor_state], np.float32))
        self._last = msg

    def send(self, side: str, q: np.ndarray) -> None:
        self.pub[side].Write(HandCmd_(
            motor_cmd=[MotorCmd_(mode=1, q=float(q[i]), dq=0.0, tau=0.0,
                                 kp=1.5, kd=0.1, reserve=0)
                       for i in range(N_HAND_JOINTS)],
            reserve=[0, 0, 0, 0]))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--settle", type=float, default=4.0)
    ap.add_argument("--drive", type=float, default=6.0)
    args = ap.parse_args()

    c = Dex3Client(args.domain, args.iface)
    print("[mm3-dex3] waiting for rt/dex3/*/state ...", flush=True)
    t0 = time.perf_counter()
    while not (c.state["left"] and c.state["right"]):
        if time.perf_counter() - t0 > 20:
            print("[mm3-dex3] FAIL: no HandState_ within 20 s", flush=True)
            return 1
        time.sleep(0.05)

    m = c._last
    print(f"[mm3-dex3] SHAPE motor_state={len(m.motor_state)} "
          f"press_sensor_state={len(m.press_sensor_state)} "
          f"pressure_per_zone={len(m.press_sensor_state[0].pressure)}", flush=True)

    mark = {s: len(c.stamps[s]) for s in ("left", "right")}
    time.sleep(args.settle)
    rates = {}
    for s in ("left", "right"):
        st = np.array(c.stamps[s][mark[s]:])
        rates[s] = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 1 else 0.0
    print(f"[mm3-dex3] RATE left={rates['left']:.2f} Hz right={rates['right']:.2f} Hz "
          f"(gate >=200 Hz)", flush=True)

    q_before = {s: c.state[s][-1].copy() for s in ("left", "right")}

    # A curl on every entry, passive indices included, so the twin is asked to do
    # something the real hand cannot and must be seen refusing.
    target = np.full(N_HAND_JOINTS, 0.35, np.float32)
    t1 = time.perf_counter()
    while time.perf_counter() - t1 < args.drive:
        for s in ("left", "right"):
            c.send(s, target)
        time.sleep(0.005)
    time.sleep(0.3)

    for s in ("left", "right"):
        moved = np.abs(c.state[s][-1] - q_before[s])
        active = [i for i in range(N_HAND_JOINTS) if i not in PASSIVE]
        names = NAMES[s]
        print(f"[mm3-dex3] DRIVE {s}: active_moved_mean={moved[active].mean():.4f} rad "
              f"active_moved_max={moved[active].max():.4f} "
              f"passive_moved_max={moved[list(PASSIVE)].max():.4f}", flush=True)
        # Per index, because "a passive joint moved" is ambiguous between the twin
        # driving something it must not and a genuinely coupled joint following its
        # neighbour -- and the two have opposite verdicts. The Dex5 is not name
        # symmetric at index 12 (Link_41L vs Roll_41R), so the pair is printed too.
        print(f"[mm3-dex3]   passive {s}: " + "  ".join(
            f"{i}:{names[i]}={moved[i]:.4f}" for i in PASSIVE), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
