"""A stand-in for the Isaac side: writes the state segment at a fixed physics rate.

Exists so the DDS side's rate control can be gated on its own, without Isaac and
without the GPU.  If rt/lowstate misses 1041.68 Hz +-2% here, the fault is in the
pacing loop; if it only misses once Isaac is attached, the fault is contention --
and those are different bugs with different fixes.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import shm  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--physics-hz", type=float, default=1000.0)
    ap.add_argument("--duration", type=float, default=30.0)
    args = ap.parse_args()

    ch = shm.open_state(create=True)
    period = 1.0 / args.physics_hz
    q = np.zeros(shm.N_MOTOR, np.float32)
    t0 = time.perf_counter()
    deadline = t0 + period
    n = 0
    while True:
        now = time.perf_counter()
        el = now - t0
        if el >= args.duration:
            break
        # A slow sweep so a reader can tell fresh samples from repeats.
        q[:29] = np.float32(np.sin(el) * 0.1)
        ch.write(stamp_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                 sim_time=el, physics_step=n, q=q,
                 quat_wxyz=np.array([1, 0, 0, 0], np.float32),
                 accel=np.array([0, 0, 9.80665], np.float32))
        n += 1
        deadline += period
        slack = deadline - time.perf_counter()
        if slack > 250e-6:
            time.sleep(slack - 200e-6)
        while time.perf_counter() < deadline:
            pass

    el = time.perf_counter() - t0
    print(f"[fake-sim] wrote {n} states in {el:.3f}s = {n/el:.3f} Hz", flush=True)
    ch.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
