"""Load and validate a twin contract.

The contract (isaac/twin/<robot>_contract.yaml) is the single source of truth for the
digital twin: what the sim must publish, in which frames, at which rates, and where every
sensor physically sits. The sim reads it to place prims and configure publishers; the
audit reads it to decide whether the sim is telling the truth.

Every geometric value carries a `provenance` and a `source`. That is not documentation
politeness -- a value with provenance `assumed` is a promise to come back to it, and the
audit reports assumed values separately so they cannot quietly become load-bearing.
"""

from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Tuple

import yaml

SCHEMA_VERSION = 1

PROVENANCE = {"calibrated", "urdf_nominal", "datasheet", "measured", "configured", "assumed"}
PARITY_CLASSES = {"A", "B", "C"}
DIRECTIONS = {"publish", "subscribe"}


class ContractError(ValueError):
    """A contract that cannot be trusted. Always fatal -- never warn and continue."""


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise ContractError(msg)


def _vec(value: Any, n: int, where: str) -> List[float]:
    _require(
        isinstance(value, (list, tuple)) and len(value) == n,
        f"{where}: expected {n} numbers, got {value!r}",
    )
    out = []
    for i, v in enumerate(value):
        _require(isinstance(v, (int, float)), f"{where}[{i}]: not a number: {v!r}")
        out.append(float(v))
    return out


def _check_provenanced(block: Dict[str, Any], where: str) -> None:
    _require("provenance" in block, f"{where}: missing 'provenance'")
    _require(
        block["provenance"] in PROVENANCE,
        f"{where}: provenance {block['provenance']!r} not one of {sorted(PROVENANCE)}",
    )
    _require("source" in block and str(block["source"]).strip(), f"{where}: missing 'source'")


POINTFIELD_DATATYPES = {"INT8", "UINT8", "INT16", "UINT16", "INT32", "UINT32", "FLOAT32", "FLOAT64"}


def _validate_expect(expect: Any, where: str) -> None:
    """Validate a topic's `expect` block -- the payload-level parity assertions.

    These are what turn "the topic exists" into "the topic carries the same thing the
    robot carries": ray geometry, cloud field layout, image encoding, intrinsics.
    """
    if expect is None:
        return
    _require(isinstance(expect, dict), f"{where}.expect must be a mapping")

    ls = expect.get("laserscan")
    if ls is not None:
        _require(isinstance(ls, dict), f"{where}.expect.laserscan must be a mapping")
        if "ray_count" in ls:
            _require(isinstance(ls["ray_count"], int) and ls["ray_count"] > 0,
                     f"{where}.expect.laserscan.ray_count must be a positive integer")
        for key in ("angle_min", "angle_max", "angle_increment", "range_min", "range_max", "scan_time"):
            if key in ls:
                _require(isinstance(ls[key], (int, float)), f"{where}.expect.laserscan.{key} must be a number")
        if {"angle_min", "angle_max", "angle_increment", "ray_count"} <= set(ls):
            # The four are over-determined; if they disagree the contract is describing
            # a scan that cannot exist and every later comparison inherits the error.
            span = ls["angle_max"] - ls["angle_min"]
            implied = span / ls["angle_increment"]
            _require(
                abs(implied - (ls["ray_count"] - 1)) <= 1.0,
                f"{where}.expect.laserscan: ray_count {ls['ray_count']} is inconsistent with "
                f"angle span {span:.6f} / increment {ls['angle_increment']} "
                f"(implies {implied:.2f} + 1 rays)",
            )
        if {"range_min", "range_max"} <= set(ls):
            _require(ls["range_min"] < ls["range_max"],
                     f"{where}.expect.laserscan: range_min must be below range_max")

    pc = expect.get("pointcloud_fields")
    if pc is not None:
        _require(isinstance(pc, list) and pc, f"{where}.expect.pointcloud_fields must be a non-empty list")
        offset = None
        for i, f in enumerate(pc):
            w = f"{where}.expect.pointcloud_fields[{i}]"
            _require(isinstance(f, dict) and f.get("name"), f"{w}: needs a name")
            _require(f.get("datatype") in POINTFIELD_DATATYPES,
                     f"{w}: datatype {f.get('datatype')!r} not one of {sorted(POINTFIELD_DATATYPES)}")
            if "offset" in f:
                _require(isinstance(f["offset"], int) and f["offset"] >= 0, f"{w}: offset must be >= 0")
                if offset is not None:
                    _require(f["offset"] >= offset, f"{w}: offsets must be non-decreasing")
                offset = f["offset"]

    ci = expect.get("camera_info")
    if ci is not None:
        _require(isinstance(ci, dict), f"{where}.expect.camera_info must be a mapping")
        for key in ("width", "height"):
            if key in ci:
                _require(isinstance(ci[key], int) and ci[key] > 0, f"{where}.expect.camera_info.{key} must be > 0")
        if "K" in ci:
            _vec(ci["K"], 9, f"{where}.expect.camera_info.K")
        if "D" in ci:
            _require(isinstance(ci["D"], list), f"{where}.expect.camera_info.D must be a list")

    if "encoding" in expect:
        _require(isinstance(expect["encoding"], str) and expect["encoding"],
                 f"{where}.expect.encoding must be a non-empty string")


def validate(contract: Dict[str, Any]) -> Dict[str, Any]:
    """Validate a loaded contract. Raises ContractError on the first problem found."""
    _require(isinstance(contract, dict), "contract is not a mapping")
    _require(
        contract.get("schema_version") == SCHEMA_VERSION,
        f"schema_version must be {SCHEMA_VERSION}, got {contract.get('schema_version')!r}",
    )

    robot = contract.get("robot")
    _require(isinstance(robot, dict), "missing 'robot' block")
    for key in ("type", "id", "namespace", "base_frame", "odom_frame"):
        _require(bool(robot.get(key)), f"robot.{key} is required")
    _require(
        robot["namespace"].startswith("/") and not robot["namespace"].endswith("/"),
        f"robot.namespace must be absolute and not end in '/': {robot['namespace']!r}",
    )

    sensors = contract.get("sensors") or []
    _require(isinstance(sensors, list), "'sensors' must be a list")
    seen_sensor = set()
    for s in sensors:
        name = s.get("name", "<unnamed>")
        where = f"sensor {name}"
        _require(name not in seen_sensor, f"{where}: duplicate sensor name")
        seen_sensor.add(name)
        for key in ("model", "parent_link"):
            _require(bool(s.get(key)), f"{where}.{key} is required")
        pose = s.get("pose")
        _require(isinstance(pose, dict), f"{where}: missing 'pose'")
        _vec(pose.get("xyz"), 3, f"{where}.pose.xyz")
        _vec(pose.get("rpy"), 3, f"{where}.pose.rpy")
        _check_provenanced(pose, f"{where}.pose")

    topics = contract.get("topics") or []
    _require(isinstance(topics, list) and topics, "'topics' must be a non-empty list")
    seen_topic = set()
    for t in topics:
        name = t.get("name", "<unnamed>")
        where = f"topic {name}"
        _require(name.startswith("/"), f"{where}: topic must be absolute (start with '/')")
        key = (name, t.get("direction"))
        _require(key not in seen_topic, f"{where}: duplicate topic/direction")
        seen_topic.add(key)
        _require(t.get("direction") in DIRECTIONS, f"{where}: direction must be one of {sorted(DIRECTIONS)}")
        _require("/msg/" in str(t.get("type", "")), f"{where}: type must be fully qualified, got {t.get('type')!r}")
        _require(t.get("parity_class") in PARITY_CLASSES, f"{where}: parity_class must be A, B or C")
        _check_provenanced(t, where)
        rate = t.get("rate_hz")
        if rate is not None:
            _require(isinstance(rate, (int, float)) and rate > 0, f"{where}: rate_hz must be a positive number")
        qos = t.get("qos") or {}
        _require(isinstance(qos, dict), f"{where}: qos must be a mapping")
        if "reliability" in qos:
            _require(
                qos["reliability"] in {"reliable", "best_effort"},
                f"{where}: qos.reliability must be 'reliable' or 'best_effort'",
            )
        if "durability" in qos:
            _require(
                qos["durability"] in {"volatile", "transient_local"},
                f"{where}: qos.durability must be 'volatile' or 'transient_local'",
            )
        _validate_expect(t.get("expect"), where)

    edges = contract.get("tf_static") or []
    _require(isinstance(edges, list), "'tf_static' must be a list")
    seen_child = {}
    for e in edges:
        where = f"tf_static {e.get('parent')}->{e.get('child')}"
        for key in ("parent", "child"):
            _require(bool(e.get(key)), f"{where}: {key} is required")
        _require(e["parent"] != e["child"], f"{where}: an edge cannot be its own parent")
        # A frame with two parents is not a tree. This is baseline defect B-3; the
        # contract must make it unrepresentable rather than merely discouraged.
        prev = seen_child.get(e["child"])
        _require(
            prev is None,
            f"{where}: child frame {e['child']!r} already has parent {prev!r} -- "
            "a TF frame may have exactly one parent",
        )
        seen_child[e["child"]] = e["parent"]
        _vec(e.get("xyz"), 3, f"{where}.xyz")
        _vec(e.get("rpy"), 3, f"{where}.rpy")
        _check_provenanced(e, where)

    # The TF static edges must form a forest (no cycles).
    for child, parent in seen_child.items():
        seen = {child}
        cur = parent
        while cur in seen_child:
            _require(cur not in seen, f"tf_static: cycle detected through frame {cur!r}")
            seen.add(cur)
            cur = seen_child[cur]

    p2l = contract.get("pointcloud_to_laserscan")
    if p2l is not None:
        _require(isinstance(p2l, dict), "'pointcloud_to_laserscan' must be a mapping")
        _check_provenanced(p2l, "pointcloud_to_laserscan")
        for key in ("min_height", "max_height", "range_min", "range_max"):
            _require(key in p2l, f"pointcloud_to_laserscan.{key} is required")
        _require(
            p2l["min_height"] < p2l["max_height"],
            "pointcloud_to_laserscan: min_height must be below max_height",
        )
        _require(
            p2l["range_min"] < p2l["range_max"],
            "pointcloud_to_laserscan: range_min must be below range_max",
        )

    return contract


def load(path: str) -> Dict[str, Any]:
    _require(os.path.exists(path), f"contract not found: {path}")
    with open(path, "r", encoding="utf-8") as fh:
        return validate(yaml.safe_load(fh))


def topic_by_name(contract: Dict[str, Any], name: str) -> Dict[str, Any] | None:
    for t in contract.get("topics", []):
        if t["name"] == name:
            return t
    return None


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """RPY (radians, ZYX / fixed-axis xyz as REP-103 and tf2 use) -> quaternion xyzw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_from_quat(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Quaternion xyzw -> RPY radians, matching quat_from_rpy."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def angular_distance(q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]) -> float:
    """Smallest rotation angle (radians) between two xyzw quaternions.

    Compared as rotations, not as component tuples: q and -q are the same rotation, and
    comparing components would report a spurious 2*pi difference for an exact match.
    """
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))
