#!/usr/bin/env python3
"""MM4: drive the SONIC deploy through its scripted sequence over ZMQ.

SONIC's `--input-type zmq_manager` subscribes to three topics on one host:port
(see gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/input_interface/zmq_manager.hpp):

    command  {start, stop, planner, delta_heading}   start/stop and mode switch
    planner  {mode, movement, facing, speed, height} per-frame locomotion
    pose     {joint_pos, joint_vel, body_quat, ...}  streamed frames, POSE mode

Wire format per zmq_packed_message_subscriber.hpp: one single-part ZMQ message of
    [topic prefix][1280-byte null-padded JSON header][concatenated binary fields]
with the header naming each field's dtype and shape. Single-part is deliberate --
it is what lets ZMQ_CONFLATE drop stale frames safely, so this publisher must never
split a message across parts.

The sequence is MM4's: stand -> weight shift -> planner walking -> heading
turn-in-place -> stop -> POSE arms-only. Each step prints what it sent so a run can
be read against the deploy's own log.
"""

from __future__ import annotations

import argparse
import json
import time

import numpy as np
import zmq

HEADER_SIZE = 1280

# LocomotionMode, from localmotion_kplanner.hpp. Only the ones this script uses.
MODE_IDLE = 0
MODE_WALK = 1


def pack(fields: dict[str, np.ndarray]) -> bytes:
    """Build one packed message: 1280-byte JSON header + concatenated payload."""
    hdr_fields, blobs = [], []
    for name, arr in fields.items():
        a = np.ascontiguousarray(arr)
        # dtype strings must be exactly what SONIC's decoder compares against
        # (zmq_manager.hpp / zmq_packed_message_subscriber.hpp): bool, u8, i8, i16,
        # i32, i64, f16, f32, f64. "b8" is NOT in that set -- sending it made every
        # command message parse as "missing fields (need: start, stop, planner)" and
        # SONIC held a static default pose through an entire scripted sequence while
        # looking perfectly healthy.
        dtype = {"float32": "f32", "float64": "f64", "float16": "f16",
                 "int8": "i8", "int16": "i16", "int32": "i32", "int64": "i64",
                 "uint8": "u8", "bool": "bool"}[a.dtype.name]
        hdr_fields.append({"name": name, "dtype": dtype, "shape": list(a.shape)})
        blobs.append(a.tobytes())
    header = json.dumps({"v": 1, "endian": "le", "count": 1, "fields": hdr_fields}).encode()
    if len(header) > HEADER_SIZE:
        raise ValueError(f"header {len(header)} B exceeds the fixed {HEADER_SIZE} B slot")
    return header.ljust(HEADER_SIZE, b"\0") + b"".join(blobs)


class SonicDriver:
    def __init__(self, host: str, port: int):
        self.ctx = zmq.Context()
        self.sock = self.ctx.socket(zmq.PUB)
        self.sock.bind(f"tcp://{host}:{port}")
        # PUB/SUB drops everything sent before the subscriber finishes connecting,
        # and SONIC connects on its own schedule. Without this pause the whole
        # sequence can be published into a void and the deploy simply never starts.
        time.sleep(1.0)

    def _send(self, topic: str, fields: dict[str, np.ndarray]) -> None:
        self.sock.send(topic.encode() + pack(fields))

    def command(self, start=False, stop=False, planner=True, delta_heading=0.0) -> None:
        self._send("command", {
            "start": np.array([start], np.bool_),
            "stop": np.array([stop], np.bool_),
            "planner": np.array([planner], np.bool_),
            "delta_heading": np.array([delta_heading], np.float32),
        })
        print(f"  [command] start={start} stop={stop} planner={planner} "
              f"delta_heading={delta_heading:+.3f}", flush=True)

    # movement and facing are THREE-vectors, not two. The decoder memcpy's a fixed
    # i<3 loop out of each buffer (zmq_manager.hpp OnPlannerReceived), so a 2-element
    # field is read one float past its end -- which is not a parse error, just a
    # garbage third component, and it presents as "Planner initialization timeout"
    # with no indication that the shape was wrong.
    def planner(self, mode=MODE_IDLE, movement=(0.0, 0.0, 0.0), facing=(1.0, 0.0, 0.0),
                speed=0.0, height=0.0) -> None:
        self._send("planner", {
            "mode": np.array([mode], np.int32),
            "movement": np.array(movement, np.float32),
            "facing": np.array(facing, np.float32),
            "speed": np.array([speed], np.float32),
            "height": np.array([height], np.float32),
        })

    def hold(self, seconds: float, label: str, **planner_kw) -> None:
        """Stream one planner frame per 20 ms for `seconds`.

        The manager resets locomotion to IDLE if no planner message arrives within
        1 s (PLANNER_TIMEOUT), so every phase has to keep streaming even when it is
        asking for nothing -- "stand" is a continuously re-sent zero command, not
        the absence of one.
        """
        print(f"[phase] {label} ({seconds:.1f}s) {planner_kw}", flush=True)
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < seconds:
            self.planner(**planner_kw)
            time.sleep(0.02)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--scale", type=float, default=1.0,
                    help="multiply every phase duration (0.25 for a quick smoke)")
    args = ap.parse_args()

    d = SonicDriver(args.host, args.port)
    s = args.scale

    print("== MM4 SONIC sequence ==", flush=True)
    d.command(start=True, planner=True)
    d.hold(6 * s, "stand", mode=MODE_IDLE, movement=(0.0, 0.0, 0.0), speed=0.0)
    d.hold(6 * s, "weight shift", mode=MODE_IDLE, movement=(0.0, 0.0, 0.0), speed=0.0, height=-0.03)
    d.hold(10 * s, "planner walking +vx", mode=MODE_WALK, movement=(1.0, 0.0, 0.0), speed=0.3)
    d.hold(8 * s, "planner walking +vy", mode=MODE_WALK, movement=(0.0, 1.0, 0.0), speed=0.2)

    print("[phase] heading turn-in-place", flush=True)
    # Turn is a delta_heading on the COMMAND topic, not a planner field: the planner
    # carries a facing vector, and asking for yaw by rotating `facing` would also
    # change where the robot walks. Kept separate for that reason.
    for _ in range(int(8 * s)):
        d.command(start=True, planner=True, delta_heading=0.4)
        d.hold(1.0, "  turning", mode=MODE_IDLE, movement=(0.0, 0.0, 0.0), speed=0.0)

    d.hold(4 * s, "stop", mode=MODE_IDLE, movement=(0.0, 0.0, 0.0), speed=0.0)

    print("[phase] POSE mode -- arms only, balancing", flush=True)
    d.command(start=True, planner=False)   # planner=false -> STREAMED_MOTION
    print("  NOTE: pose frames require a 29-DoF joint_pos stream; MM4 sends none here.",
          flush=True)
    time.sleep(2.0)
    d.command(start=True, planner=True)    # back to planner before stopping

    d.command(stop=True, planner=True)
    print("== sequence complete ==", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
