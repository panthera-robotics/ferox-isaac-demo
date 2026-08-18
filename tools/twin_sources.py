"""Observation sources for twin_audit: live ROS 2 graph, rosbag, or driver evidence.

All three produce the same `Observation` shape so the comparison engine never needs to
know where the numbers came from. That matters: the audit's job is to compare the sim
against the contract, and the contract against the robot's own recorded evidence, using
one set of rules.
"""

from __future__ import annotations

import glob
import json
import os
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

# sensor_msgs/msg/PointField datatype enum -> name
POINTFIELD_TYPES = {
    1: "INT8", 2: "UINT8", 3: "INT16", 4: "UINT16",
    5: "INT32", 6: "UINT32", 7: "FLOAT32", 8: "FLOAT64",
}


def _quat_from_rpy(roll: float, pitch: float, yaw: float):
    import math
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@dataclass
class TopicObs:
    present: bool = False
    type: str = ""
    frame_id: str = ""
    rate_hz: float | None = None
    qos_reliability: str = ""
    qos_durability: str = ""
    encoding: str = ""
    rate_basis: str = ""
    extras: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Observation:
    label: str
    topics: Dict[str, TopicObs] = field(default_factory=dict)
    # (parent, child) -> {"xyz": [..], "quat": [x,y,z,w] or None}
    tf_static: Dict[Tuple[str, str], Dict[str, Any]] = field(default_factory=dict)
    tf_static_values_known: bool = True
    # Whether this source can see inside messages at all. The evidence source records
    # topic names, types, rates and QoS but no payloads, so frame_id / encoding /
    # geometry are UNKNOWN there rather than wrong. Reporting them as failures would
    # train the reader to ignore the audit.
    payloads_observable: bool = True
    notes: List[str] = field(default_factory=list)

    def topic(self, name: str) -> TopicObs:
        return self.topics.setdefault(name, TopicObs())


# ---------------------------------------------------------------- message probes


def probe_message(msg, type_str: str) -> Tuple[str, str, Dict[str, Any]]:
    """Pull (frame_id, encoding, extras) out of one message, per type.

    `extras` is where the payload-specific parity checks get their inputs: LaserScan
    geometry, PointCloud2 field layout, CameraInfo intrinsics. Anything not understood
    yields empty extras rather than an error -- an unknown type still gets its topic
    name, type and rate checked.
    """
    frame_id = ""
    header = getattr(msg, "header", None)
    if header is not None:
        frame_id = header.frame_id
    encoding = ""
    extras: Dict[str, Any] = {}
    short = type_str.split("/")[-1]

    if short == "LaserScan":
        ranges = list(msg.ranges)
        finite = [r for r in ranges if r == r and abs(r) != float("inf")]
        in_band = [r for r in finite if msg.range_min <= r <= msg.range_max]
        below = [r for r in finite if r < msg.range_min]
        extras.update(
            ray_count=len(ranges),
            angle_min=float(msg.angle_min),
            angle_max=float(msg.angle_max),
            angle_increment=float(msg.angle_increment),
            scan_time=float(msg.scan_time),
            time_increment=float(msg.time_increment),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            frac_in_range=(len(in_band) / len(finite)) if finite else 0.0,
            frac_below_range_min=(len(below) / len(ranges)) if ranges else 0.0,
            n_finite=len(finite),
        )
    elif short == "PointCloud2":
        extras.update(
            fields=[
                {
                    "name": f.name,
                    "offset": int(f.offset),
                    "datatype": POINTFIELD_TYPES.get(int(f.datatype), str(f.datatype)),
                    "count": int(f.count),
                }
                for f in msg.fields
            ],
            point_step=int(msg.point_step),
            height=int(msg.height),
            width=int(msg.width),
            is_dense=bool(msg.is_dense),
        )
    elif short == "CameraInfo":
        extras.update(
            width=int(msg.width),
            height=int(msg.height),
            distortion_model=msg.distortion_model,
            K=[float(v) for v in msg.k],
            D=[float(v) for v in msg.d],
        )
    elif short == "Image":
        encoding = msg.encoding
        extras.update(width=int(msg.width), height=int(msg.height), step=int(msg.step))
    elif short == "Odometry":
        extras.update(
            child_frame_id=msg.child_frame_id,
            pose_covariance_all_zero=all(v == 0.0 for v in msg.pose.covariance),
            twist_covariance_all_zero=all(v == 0.0 for v in msg.twist.covariance),
        )
    elif short == "Imu":
        extras.update(
            orientation_covariance_first=float(msg.orientation_covariance[0]),
        )
    return frame_id, encoding, extras


# ---------------------------------------------------------------- live ROS 2 graph


def observe_live(topic_names: List[str], duration: float, node_name: str = "twin_audit") -> Observation:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
    )
    from rosidl_runtime_py.utilities import get_message
    from tf2_msgs.msg import TFMessage

    obs = Observation(label="live ROS 2 graph")

    rclpy.init()
    node = Node(node_name)
    try:
        # Discovery needs a moment on a cold DDS graph, or every topic looks absent.
        deadline = node.get_clock().now().nanoseconds + int(3e9)
        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        graph = dict(node.get_topic_names_and_types())
        counts: Dict[str, int] = {}
        stamps: Dict[str, List[float]] = {}
        first: Dict[str, Any] = {}
        subs = []

        for name in topic_names:
            t = obs.topic(name)
            if name not in graph:
                continue
            infos = node.get_publishers_info_by_topic(name)
            if not infos:
                # Declared in the graph but with no publisher: that is "absent" for
                # parity purposes -- a topic nobody publishes is not an interface.
                obs.notes.append(f"{name}: declared in graph but has no publisher")
                continue
            t.present = True
            t.type = graph[name][0]
            qos = infos[0].qos_profile
            t.qos_reliability = "reliable" if qos.reliability == QoSReliabilityPolicy.RELIABLE else "best_effort"
            t.qos_durability = (
                "transient_local" if qos.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL else "volatile"
            )
            try:
                msg_cls = get_message(t.type)
            except Exception as exc:  # unknown/unbuilt type
                obs.notes.append(f"{name}: cannot resolve type {t.type}: {exc}")
                continue

            # Subscribe BEST_EFFORT: it is compatible with both reliable and
            # best-effort publishers. A RELIABLE subscriber would silently receive
            # nothing from a BEST_EFFORT publisher and we would report a dead topic.
            sub_qos = QoSProfile(
                depth=20,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=qos.durability,
            )
            counts[name] = 0

            def make_cb(topic_name: str, type_str: str):
                def cb(msg):
                    counts[topic_name] = counts.get(topic_name, 0) + 1
                    if topic_name not in first:
                        first[topic_name] = (msg, type_str)
                    # Record header stamps so the rate can be measured in SIM
                    # time. Wall-clock rate is the wrong quantity here: the sim
                    # runs at roughly 0.4x real time, so a 100 Hz publisher shows
                    # up as ~40 Hz on a wall clock and the audit would report a
                    # failure that says more about the GPU than the interface.
                    # What a consumer actually experiences is messages per second
                    # of SIM time, and that is what hardware's rate compares to.
                    h = getattr(msg, "header", None)
                    if h is not None:
                        t = h.stamp.sec + h.stamp.nanosec * 1e-9
                        st = stamps.setdefault(topic_name, [])
                        if not st or t > st[-1]:
                            st.append(t)
                return cb

            subs.append(node.create_subscription(msg_cls, name, make_cb(name, t.type), sub_qos))

        # /tf_static always, so the TF edge check has data even if the contract does
        # not list /tf_static as a topic row.
        tf_qos = QoSProfile(
            depth=100, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        def tf_cb(msg):
            for tr in msg.transforms:
                q = tr.transform.rotation
                p = tr.transform.translation
                obs.tf_static[(tr.header.frame_id, tr.child_frame_id)] = {
                    "xyz": [p.x, p.y, p.z],
                    "quat": [q.x, q.y, q.z, q.w],
                }

        subs.append(node.create_subscription(TFMessage, "/tf_static", tf_cb, tf_qos))

        start = node.get_clock().now().nanoseconds
        end = start + int(duration * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = (node.get_clock().now().nanoseconds - start) / 1e9

        for name, n in counts.items():
            t = obs.topic(name)
            st = stamps.get(name) or []
            if len(st) >= 2 and (st[-1] - st[0]) > 0.5:
                # SIM-time rate: messages per second of the clock the stamps are
                # in. This is the number that compares to hardware.
                t.rate_hz = (len(st) - 1) / (st[-1] - st[0])
                t.rate_basis = "sim-time stamps"
            else:
                t.rate_hz = (n / elapsed) if elapsed > 0 else None
                t.rate_basis = "wall clock"
            if name in first:
                msg, type_str = first[name]
                t.frame_id, t.encoding, t.extras = probe_message(msg, type_str)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
    return obs


# ---------------------------------------------------------------- rosbag


def observe_bag(bag_path: str, topic_names: List[str]) -> Observation:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    obs = Observation(label=f"rosbag {bag_path}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(topic_names) | {"/tf_static"}

    counts: Dict[str, int] = {}
    stamps: Dict[str, List[int]] = {}
    first: Dict[str, Any] = {}

    while reader.has_next():
        name, data, tns = reader.read_next()
        if name not in wanted:
            continue
        counts[name] = counts.get(name, 0) + 1
        stamps.setdefault(name, []).append(tns)
        if name not in first:
            try:
                msg = deserialize_message(data, get_message(types[name]))
            except Exception as exc:
                obs.notes.append(f"{name}: cannot deserialize: {exc}")
                continue
            first[name] = msg
            if name == "/tf_static":
                pass
        if name == "/tf_static":
            try:
                msg = deserialize_message(data, get_message(types[name]))
            except Exception:
                continue
            for tr in msg.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                obs.tf_static[(tr.header.frame_id, tr.child_frame_id)] = {
                    "xyz": [p.x, p.y, p.z], "quat": [q.x, q.y, q.z, q.w],
                }

    for name in topic_names:
        t = obs.topic(name)
        if name not in counts:
            continue
        t.present = True
        t.type = types.get(name, "")
        ts = stamps[name]
        span = (max(ts) - min(ts)) / 1e9 if len(ts) > 1 else 0.0
        t.rate_hz = ((len(ts) - 1) / span) if span > 0 else None
        # A bag records no QoS profile per message; leave blank rather than guess.
        if name in first:
            t.frame_id, t.encoding, t.extras = probe_message(first[name], t.type)
    obs.notes.append("bag source: QoS not recorded per message; QoS checks are skipped")
    return obs


# ---------------------------------------------------------------- driver evidence


def observe_evidence(evidence_dir: str) -> Observation:
    """Build an Observation from panthera-g1-driver/evidence/*.

    This is the no-bag fallback the campaign asks for. It carries the robot's own
    recorded truth: which topics existed, at what rate, with what QoS, and the exact
    /tf_static EDGE SET. Note the edge set is names only -- Session A recorded no
    transform values -- so TF value comparison is not possible from this source and is
    reported as such rather than silently passing.
    """
    obs = Observation(label=f"driver evidence {evidence_dir}", payloads_observable=False)

    def find(*rel: str) -> str | None:
        for r in rel:
            p = os.path.join(evidence_dir, r)
            if os.path.exists(p):
                return p
        hits = glob.glob(os.path.join(evidence_dir, "**", rel[0]), recursive=True)
        return hits[0] if hits else None

    # --- topic list + types:  "/topic [pkg/msg/Type]"
    p = find("fw2026q3/topic_list_raw.txt", "topic_list_raw.txt")
    if p:
        for line in open(p, encoding="utf-8", errors="replace"):
            m = re.match(r"^(\S+)\s+\[([^\]]+)\]\s*$", line.strip())
            if m:
                t = obs.topic(m.group(1))
                t.present = True
                t.type = m.group(2)
    else:
        obs.notes.append("no topic_list_raw.txt found")

    # --- rates:  "----- /topic -----" then "average rate: N"
    p = find("fw2026q3/rates_host.txt", "rates_host.txt")
    if p:
        cur = None
        for line in open(p, encoding="utf-8", errors="replace"):
            m = re.match(r"^-+\s*(\S+)\s*-+\s*$", line.strip())
            if m:
                cur = m.group(1)
                continue
            m = re.match(r"^average rate:\s*([0-9.]+)", line.strip())
            if m and cur:
                # keep the last reported window for that topic
                obs.topic(cur).rate_hz = float(m.group(1))
                obs.topic(cur).present = True

    # --- QoS table: TOPIC PUB SUB RELIABILITY DURABILITY PUB NODE
    p = find("fw2026q3/qos_summary_host.txt", "qos_summary_host.txt")
    if p:
        for line in open(p, encoding="utf-8", errors="replace"):
            parts = line.split()
            if len(parts) < 5 or not parts[0].startswith("/"):
                continue
            name, pub, _sub, rel, dur = parts[0], parts[1], parts[2], parts[3], parts[4]
            if not pub.isdigit() or int(pub) == 0:
                continue
            t = obs.topic(name)
            t.present = True
            if rel in ("RELIABLE", "BEST_EFFORT"):
                t.qos_reliability = rel.lower()
            if dur in ("VOLATILE", "TRANSIENT_LOCAL"):
                t.qos_durability = dur.lower()

    # --- /tf_static edge SET (names only)
    p = find("fw2026q3/session_a/31_tf_static.txt", "31_tf_static.txt")
    if p:
        for line in open(p, encoding="utf-8", errors="replace"):
            m = re.search(r"frame_id:\s*(\S+)\s+child_frame_id:\s*(\S+)", line)
            if m:
                obs.tf_static[(m.group(1), m.group(2))] = {"xyz": None, "quat": None}
        obs.tf_static_values_known = False
        obs.notes.append(
            "evidence source: 31_tf_static.txt records edge NAMES only, no transform "
            "values -- TF value comparison skipped (set comparison still enforced)"
        )

    # --- the Ferox contract evidence capture (session_a/30_evidence.txt)
    #
    # This is the strongest evidence in the repo and the only place that carries BOTH
    # QoS-matched rates for the /ferox/* topics AND resolved /tf_static VALUES. It is
    # produced by tools/g1_bringup/collect_evidence.py, whose own header insists on
    # "QoS-matched rclpy subscribers, never `ros2 topic hz`" -- so where it disagrees
    # with rates_host.txt (which used `ros2 topic hz`), this file is the better number.
    p = find("fw2026q3/session_a/30_evidence.txt", "30_evidence.txt")
    if p:
        section = None
        for line in open(p, encoding="utf-8", errors="replace"):
            if line.lstrip().startswith("## "):
                section = line.strip().lower()
                continue
            # "## 1. RATES"  ->  "/topic   MSGS   RATE  GAPmed  GAPmax"
            if section and "rates" in section:
                m = re.match(r"^(/\S+)\s+(\d+)\s+([0-9.]+)\s", line)
                if m:
                    t = obs.topic(m.group(1))
                    t.present = True
                    t.rate_hz = float(m.group(3))
                    obs.notes.append(
                        f"{m.group(1)}: {t.rate_hz} Hz from 30_evidence.txt "
                        "(QoS-matched subscriber)")
            # "## 4. TF"  ->  "parent -> child  xyz=(..)  rpy_rad=(..)  [note]"
            if section and section.startswith("## 4"):
                m = re.match(
                    r"^\s*(\S+)\s*->\s*(\S+)\s+xyz=\(([^)]*)\)\s+rpy_rad=\(([^)]*)\)",
                    line)
                if m:
                    parent, child = m.group(1), m.group(2)
                    if "dynamic" in line:
                        continue  # /tf, not /tf_static
                    if "chain resolves" in line:
                        # A derived end-to-end composition the capture prints as a
                        # sanity check (base_link -> livox_imu), NOT a published edge.
                        # Treating it as one would make the twin publish a frame the
                        # robot does not.
                        continue
                    try:
                        xyz = [float(v) for v in m.group(3).replace("+", "").split(",")]
                        rpy = [float(v) for v in m.group(4).replace("+", "").split(",")]
                    except ValueError:
                        continue
                    obs.tf_static[(parent, child)] = {
                        "xyz": xyz,
                        "quat": list(_quat_from_rpy(*rpy)),
                        "printed_precision": True,
                    }
        if any(v.get("printed_precision") for v in obs.tf_static.values()):
            obs.tf_static_values_known = True
            obs.notes.append(
                "30_evidence.txt prints TF to 4-5 decimals, so TF value comparison from "
                "this source uses a relaxed 1e-4 m / 1e-4 rad print tolerance rather than "
                "proving full precision")

    # --- the sidecar lidar capture (session_a/13_livox_lidar.txt)
    # /livox/lidar is published by our own livox_ros_driver2 sidecar, so it is absent
    # from topic_list_raw.txt (a census of the robot's native firmware bus). Its
    # evidence is this dedicated capture.
    p = find("fw2026q3/session_a/13_livox_lidar.txt", "13_livox_lidar.txt")
    if p:
        text = open(p, encoding="utf-8", errors="replace").read()
        m = re.search(r"rate on (/\S+)", text)
        if m:
            t = obs.topic(m.group(1))
            t.present = True
            t.type = "sensor_msgs/msg/PointCloud2"
            rates = [float(x) for x in re.findall(r"average rate:\s*([0-9.]+)", text)]
            if rates:
                t.rate_hz = rates[-1]
            fm = re.search(r"frame_id:\s*(\S+)", text)
            if fm:
                t.frame_id = fm.group(1)

    # --- PointCloud2 field layout, if any probe json carries one
    for jp in glob.glob(os.path.join(evidence_dir, "**", "*.json"), recursive=True):
        try:
            data = json.load(open(jp, encoding="utf-8"))
        except Exception:
            continue
        for entry in (data.get("topics") or []) if isinstance(data, dict) else []:
            extra = entry.get("extra") or {}
            if "fields" not in extra:
                continue
            name = entry.get("topic")
            if not name:
                continue
            t = obs.topic(name)
            t.present = True
            frames = entry.get("frame_ids") or {}
            if frames:
                t.frame_id = max(frames, key=frames.get)
            if entry.get("rate_hz"):
                t.rate_hz = float(entry["rate_hz"])
            t.extras.update(
                fields=[
                    {
                        "name": f[0], "offset": int(f[1]),
                        "datatype": POINTFIELD_TYPES.get(int(f[2]), str(f[2])),
                        "count": int(f[3]),
                    }
                    for f in extra["fields"]
                ],
                point_step=int(extra.get("point_step", 0)),
                height=int(extra.get("height", 0)),
                width=int(extra.get("width", 0)),
                is_dense=bool(extra.get("is_dense", True)),
                evidence_file=os.path.relpath(jp, evidence_dir),
            )
    return obs
