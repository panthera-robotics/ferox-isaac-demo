#!/usr/bin/env python3
"""MM4: drive the SONIC deploy through its scripted sequence over ZMQ.

Modelled directly on panthera-g1-wbc `tools/scripted_walk.py` (the driver that closed
this loop on the Spark for W1/W2) and, like it, it IMPORTS NVIDIA's own wire-format
builders from `gear_sonic.utils.teleop.zmq.zmq_planner_sender` rather than
reimplementing them, so the wire format stays owned by upstream.

Reverse-engineering the format from the C++ headers got two thirds of the way and
three details wrong, every one of them silent rather than loud:

  * `WALK` is 2. LocomotionMode is IDLE/SLOW_WALK/WALK/RUN = 0/1/2/3, so a "1" that
    looks like a walk command is a SLOW_WALK.
  * `speed` and `height` use **-1.0** to mean "mode default". Sending 0.0 is a literal
    zero speed, i.e. a robot correctly obeying an order to go nowhere.
  * the start command has to be re-sent on EVERY beat, not fired once. ZMQ PUB drops
    everything published before a SUB finishes connecting, and the deploy spends a
    long time in TensorRT init; a single start lands in that hole and is simply lost.

Sequence per CAMPAIGN 4.2 / MM4: stand -> weight shift -> planner walking (vx, vy) ->
heading turn-in-place -> stop -> POSE-mode arms-only targets while balancing.

Arms-only is done with the planner message's optional `upper_body_position` (float[17]
= waist 3 + arms 14, SDK indices 12..28) while the planner keeps driving the legs --
that IS "arms-only targets while balancing". Switching the manager to STREAMED_MOTION
instead would hand it the whole body and stop it balancing, which is the opposite of
what the gate asks for.

Run inside ferox/twin-lowlevel:humble with PYTHONPATH pointed at the upstream repo.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Sequence

import zmq

try:
    from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
        build_command_message,
        build_planner_message,
    )
except ImportError as exc:  # pragma: no cover
    sys.exit(f"Cannot import NVIDIA's ZMQ builders ({exc}).\n"
             "Mount the upstream repo and set PYTHONPATH to it.")

# gear_sonic/scripts/pico_manager_thread_server.py:101-109
IDLE, SLOW_WALK, WALK, RUN = 0, 1, 2, 3
_MODE_NAMES = {IDLE: "IDLE", SLOW_WALK: "SLOW_WALK", WALK: "WALK", RUN: "RUN"}

N_UPPER = 17

# deploy.yaml's default stance for SDK joints 12..28, i.e. the pose SONIC already
# holds. Arms-only phases move AWAY from this and return to it.
UPPER_NEUTRAL = [
    0.0, 0.0, 0.0,                              # waist yaw / roll / pitch
    0.30, 0.25, 0.0, 0.97, 0.15, 0.0, 0.0,      # left  shoulder p/r/y, elbow, wrist r/p/y
    0.30, -0.25, 0.0, 0.97, -0.15, 0.0, 0.0,    # right shoulder p/r/y, elbow, wrist r/p/y
]
# Both arms raised forward and out: shoulder_pitch swings negative (arm forward) and
# the elbow opens. Deliberately inside the URDF limits with margin.
UPPER_REACH = [
    0.0, 0.0, 0.0,
    -0.60, 0.45, 0.0, 0.40, 0.15, 0.0, 0.0,
    -0.60, -0.45, 0.0, 0.40, -0.15, 0.0, 0.0,
]


@dataclass
class Segment:
    name: str
    seconds: float
    mode: int = IDLE
    movement: Sequence[float] = (0.0, 0.0, 0.0)   # body frame: +x fwd, +y left
    facing: Sequence[float] = (1.0, 0.0, 0.0)
    speed: float = -1.0                            # -1 = mode default
    height: float = -1.0                           # -1 = default
    upper: Sequence[float] | None = None           # 17 upper-body targets, or None


MM4_PLAN: list[Segment] = [
    Segment("stand",        20.0, mode=IDLE),
    # Weight shift: SONIC has no "shift" primitive, so it is commanded as a slow
    # lateral walk that reverses -- the robot loads one foot then the other, which is
    # what a weight shift IS. Named for what it is rather than for what it looks like.
    Segment("shift_left",    8.0, mode=SLOW_WALK, movement=(0.0, 1.0, 0.0)),
    Segment("shift_right",   8.0, mode=SLOW_WALK, movement=(0.0, -1.0, 0.0)),
    Segment("settle",        4.0, mode=IDLE),
    Segment("walk_fwd",     20.0, mode=WALK, movement=(1.0, 0.0, 0.0)),
    Segment("walk_back",    10.0, mode=WALK, movement=(-1.0, 0.0, 0.0)),
    Segment("strafe_left",  12.0, mode=WALK, movement=(0.0, 1.0, 0.0)),
    Segment("strafe_right", 12.0, mode=WALK, movement=(0.0, -1.0, 0.0)),
    Segment("settle2",       4.0, mode=IDLE),
    # Turn in place: no translation, the facing vector swings. NOT delta_heading --
    # that is a one-shot offset on the command topic, not a sustained turn.
    Segment("turn_left",    15.0, mode=WALK, movement=(0.0, 0.0, 0.0), facing=(0.0, 1.0, 0.0)),
    Segment("turn_right",   15.0, mode=WALK, movement=(0.0, 0.0, 0.0), facing=(0.0, -1.0, 0.0)),
    Segment("stop",          6.0, mode=IDLE),
    Segment("arms_reach",   12.0, mode=IDLE, upper=UPPER_REACH),
    Segment("arms_return",   8.0, mode=IDLE, upper=UPPER_NEUTRAL),
]


def log(msg: str) -> None:
    print(f"[mm4] {msg}", flush=True)


def send_planner(sock, seg: Segment) -> None:
    kw = {}
    if seg.upper is not None:
        if len(seg.upper) != N_UPPER:
            raise ValueError(f"{seg.name}: upper must be {N_UPPER} long, got {len(seg.upper)}")
        kw["upper_body_position"] = list(seg.upper)
        kw["upper_body_velocity"] = [0.0] * N_UPPER
    sock.send(build_planner_message(
        seg.mode, list(seg.movement), list(seg.facing), seg.speed, seg.height, **kw))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="tcp://*:5556")
    ap.add_argument("--rate-hz", type=float, default=50.0,
                    help="planner rate; 50 matches the deploy's control loop")
    ap.add_argument("--hold-s", type=float, default=60.0,
                    help="how long to repeat start, covering TensorRT init")
    ap.add_argument("--settle-s", type=float, default=3.0,
                    help="pause after bind so SUBs connect (ZMQ slow joiner)")
    ap.add_argument("--scale", type=float, default=1.0, help="scale every segment")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    plan = [Segment(s.name, s.seconds * args.scale, s.mode, s.movement, s.facing,
                    s.speed, s.height, s.upper) for s in MM4_PLAN]
    log(f"MM4 plan: {len(plan)} segments, {sum(s.seconds for s in plan):.0f}s")
    for s in plan:
        log(f"    {s.name:<13} {s.seconds:5.1f}s  {_MODE_NAMES[s.mode]:<9} "
            f"move={tuple(s.movement)} facing={tuple(s.facing)}"
            f"{'  +arms' if s.upper is not None else ''}")
    if args.dry_run:
        return 0

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 1000)
    sock.bind(args.bind)
    log(f"bound PUB {args.bind}; settling {args.settle_s:.1f}s for subscribers")
    time.sleep(args.settle_s)

    period = 1.0 / args.rate_hz
    idle = Segment("idle", 0.0, mode=IDLE)
    try:
        log(f"holding start+planner for {args.hold_s:.0f}s (covers TensorRT init)")
        t0 = time.monotonic()
        next_beat = t0
        announced = 0.0
        while time.monotonic() - t0 < args.hold_s:
            sock.send(build_command_message(start=True, stop=False, planner=True))
            send_planner(sock, idle)
            el = time.monotonic() - t0
            if el - announced >= 15.0:
                announced = el
                log(f"    still holding start ({el:.0f}/{args.hold_s:.0f}s)")
            next_beat += period
            time.sleep(max(0.0, next_beat - time.monotonic()))

        log("running plan")
        for seg in plan:
            log(f"  -> {seg.name} ({seg.seconds:.0f}s, {_MODE_NAMES[seg.mode]})")
            seg_start = time.monotonic()
            while time.monotonic() - seg_start < seg.seconds:
                send_planner(sock, seg)
                next_beat += period
                time.sleep(max(0.0, next_beat - time.monotonic()))

        log("stopping control")
        for _ in range(int(args.rate_hz)):
            send_planner(sock, idle)
            sock.send(build_command_message(start=False, stop=True, planner=False))
            time.sleep(period)
    except KeyboardInterrupt:
        log("interrupted; sending stop")
        for _ in range(10):
            sock.send(build_command_message(start=False, stop=True, planner=False))
            time.sleep(0.02)
        return 130
    finally:
        sock.close()
        ctx.term()

    log("done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
