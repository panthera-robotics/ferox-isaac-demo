"""Verify the twin camera stream is what a vision consumer expects, and save frames.

This is the interface half of the DT2 `ferox_vision` item: does the twin put rgb8
colour and 16UC1 millimetre aligned depth on the hardware topic names, at the
hardware sizes and rates, with camera_info matching the contract's K. A consumer
"running unchanged" means exactly that its subscriptions are satisfied bit-for-bit.

Also writes PNGs of the colour frame and a depth visualisation, so the props the
detector is meant to fire on can be seen to be in the camera's field of view.

    python3 tools/check_twin_camera.py            # inside the nav container
"""
from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

NS = os.environ.get("TWIN_NS", "/ferox/g1_01")
OUT_DIR = os.environ.get("CAM_PNG_DIR", "/tmp/twin_camera")
REPORT = "/tmp/twin_camera.txt"
DURATION = float(os.environ.get("CAM_DURATION", "20"))

# What the robot's RealSense wrapper puts on the wire, from g1_contract.yaml.
EXPECT = {
    f"{NS}/camera/color/image_raw": ("rgb8", (720, 1280, 3)),
    f"{NS}/camera/aligned_depth_to_color/image_raw": ("16UC1", (720, 1280)),
}
EXPECT_K = [908.0, 0.0, 640.0, 0.0, 908.0, 360.0, 0.0, 0.0, 1.0]


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    report = open(REPORT, "w")

    def w(*a):
        report.write(" ".join(str(x) for x in a) + "\n")
        report.flush()

    rclpy.init()
    n = Node("twin_camera_check")
    q = QoSProfile(depth=5)
    q.reliability = ReliabilityPolicy.RELIABLE
    got = {}

    q_sensor = QoSProfile(depth=5)
    q_sensor.reliability = ReliabilityPolicy.BEST_EFFORT

    def sub(topic, msg_type, sensor_data=False):
        """Subscribe with the QoS the PUBLISHER offers, not a convenient default.

        The xyzrgb cloud is published SENSOR_DATA (BEST_EFFORT) by design -- that is
        what the driver does. A RELIABLE subscriber gets nothing from it and rclpy
        says so only in a warning: "incompatible QoS ... No messages will be
        received". This checker made exactly that mistake first, and read as a twin
        defect until the warning was read. Same trap as DT2's /scan checker.
        """
        got[topic] = []
        n.create_subscription(msg_type, topic,
                              lambda m, t=topic: got[t].append(m),
                              q_sensor if sensor_data else q)

    for topic in EXPECT:
        sub(topic, Image)
    sub(f"{NS}/camera/color/camera_info", CameraInfo)
    sub(f"{NS}/camera/aligned_depth_to_color/camera_info", CameraInfo)
    sub(f"{NS}/camera/depth/color/points", PointCloud2, sensor_data=True)

    t0 = time.time()
    while time.time() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.2)
    elapsed = time.time() - t0

    failures = 0
    w(f"observed {elapsed:.1f} s on {NS}")
    w("")
    w("ENCODINGS AND SIZES -- what a vision consumer subscribes to")
    for topic, (enc, shape) in EXPECT.items():
        msgs = got.get(topic, [])
        if not msgs:
            w(f"  FAIL {topic}: no messages")
            failures += 1
            continue
        m = msgs[-1]
        ok_enc = m.encoding == enc
        ok_dim = (m.height, m.width) == (shape[0], shape[1])
        step_ok = m.step == m.width * (3 if enc == "rgb8" else 2)
        w(f"  {'PASS' if (ok_enc and ok_dim and step_ok) else 'FAIL'} {topic}")
        w(f"       encoding {m.encoding!r} (want {enc!r}), "
          f"{m.width}x{m.height} (want {shape[1]}x{shape[0]}), "
          f"step {m.step}, {len(msgs)} msgs -> {len(msgs)/elapsed:.2f} Hz")
        if not (ok_enc and ok_dim and step_ok):
            failures += 1

    w("")
    w("CAMERA_INFO -- K against the contract")
    for topic in (f"{NS}/camera/color/camera_info",
                  f"{NS}/camera/aligned_depth_to_color/camera_info"):
        msgs = got.get(topic, [])
        if not msgs:
            w(f"  FAIL {topic}: no messages")
            failures += 1
            continue
        m = msgs[-1]
        k = [round(float(v), 4) for v in m.k]
        ok = all(abs(a - b) < 0.01 for a, b in zip(k, EXPECT_K))
        w(f"  {'PASS' if ok else 'FAIL'} {topic}")
        w(f"       K {k}")
        w(f"       {m.width}x{m.height} model {m.distortion_model!r} "
          f"frame {m.header.frame_id!r} -> {len(msgs)/elapsed:.2f} Hz")
        if not ok:
            failures += 1

    pts = got.get(f"{NS}/camera/depth/color/points", [])
    w("")
    w("POINTS -- the xyzrgb cloud the converter emits")
    if pts:
        m = pts[-1]
        flds = ", ".join(f.name for f in m.fields)
        w(f"  PASS {m.width}x{m.height}, point_step {m.point_step}, "
          f"fields [{flds}], frame {m.header.frame_id!r} -> "
          f"{len(pts)/elapsed:.2f} Hz")
    else:
        w("  FAIL no messages")
        failures += 1

    # --- frames, so the props can be seen to be in view --------------------
    w("")
    w("FRAMES")
    colour = got.get(f"{NS}/camera/color/image_raw", [])
    if colour:
        m = colour[-1]
        img = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
        # cv2, not PIL: the nav image has OpenCV and no Pillow, and per the
        # docker-immutability rule nothing gets pip-installed into a running one.
        # cv2 writes BGR, the topic is rgb8, hence the channel flip.
        import cv2
        p = os.path.join(OUT_DIR, "twin_camera_color.png")
        cv2.imwrite(p, img[:, :, ::-1])
        w(f"  {p}  mean {img.mean():.1f} std {img.std():.1f}")
    depth = got.get(f"{NS}/camera/aligned_depth_to_color/image_raw", [])
    if depth:
        m = depth[-1]
        d = np.frombuffer(bytes(m.data), dtype=np.uint16).reshape(m.height, m.width)
        valid = d[d > 0]
        w(f"  depth valid {len(valid)}/{d.size} ({100*len(valid)/d.size:.1f}%), "
          f"range {valid.min() if len(valid) else 0}-{valid.max() if len(valid) else 0} mm")
        vis = np.zeros_like(d, dtype=np.uint8)
        if len(valid):
            lo, hi = np.percentile(valid, [2, 98])
            vis = np.clip((d.astype(np.float32) - lo) / max(1.0, hi - lo), 0, 1)
            vis = (vis * 255).astype(np.uint8)
            vis[d == 0] = 0
        import cv2
        p = os.path.join(OUT_DIR, "twin_camera_depth.png")
        cv2.imwrite(p, vis)
        w(f"  {p}")

    w("")
    w(f"RESULT: {'PASS' if failures == 0 else f'{failures} FAILURE(S)'}")
    report.close()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
