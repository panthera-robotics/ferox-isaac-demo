#!/usr/bin/env python3
"""Tests for the twin contracts.

Two jobs:

1. SCHEMA -- a malformed contract must fail loudly at load time, not produce a silently
   permissive audit. Every rule the validator enforces gets a test that proves the rule
   actually rejects something.

2. DRIFT -- the calibrated constants are duplicated here as literals ON PURPOSE. If
   someone edits the contract, this fails. If someone edits the driver repo and syncs
   the contract, this still fails. Either way a human is forced to look at a number that
   took a robot, a floor and a calibration session to earn. That is the point: the test
   is a tripwire, not a DRY violation.

Run standalone (no pytest needed):
    python3 tools/tests/test_twin_contract.py
Or under pytest:
    python3 -m pytest tools/tests/test_twin_contract.py -q
"""

from __future__ import annotations

import ast
import copy
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)

import twin_contract  # noqa: E402

G1_PATH = os.path.join(REPO, "isaac", "twin", "g1_contract.yaml")
GO2_PATH = os.path.join(REPO, "isaac", "twin", "go2_contract.yaml")
ISAAC_TWIN = os.path.join(REPO, "isaac", "twin")


def _edge(contract, parent, child):
    for e in contract["tf_static"]:
        if e["parent"] == parent and e["child"] == child:
            return e
    raise AssertionError(f"contract has no tf_static edge {parent} -> {child}")


def _sensor(contract, name):
    for s in contract["sensors"]:
        if s["name"] == name:
            return s
    raise AssertionError(f"contract has no sensor {name!r}")


def _topic(contract, name):
    t = twin_contract.topic_by_name(contract, name)
    assert t is not None, f"contract has no topic {name!r}"
    return t


def _close(a, b, tol=0.0, label=""):
    assert abs(a - b) <= tol, f"{label}: {a!r} != {b!r} (tol {tol})"


# ---------------------------------------------------------------- schema


def test_shipped_contracts_validate():
    twin_contract.load(G1_PATH)
    twin_contract.load(GO2_PATH)


def test_schema_rejects_two_parents_for_one_frame():
    """A frame with two parents is not a tree. Baseline defect B-3 was exactly this
    (`laser` parented by both the sim and the Ferox bridge), so the contract must make
    it unrepresentable rather than merely discouraged."""
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    c["tf_static"].append({
        "parent": "torso_link", "child": "livox_frame",
        "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0],
        "provenance": "assumed", "source": "test",
    })
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError as exc:
        assert "one parent" in str(exc), exc
        return
    raise AssertionError("validator accepted a frame with two parents")


def test_schema_rejects_missing_provenance():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    del c["tf_static"][0]["provenance"]
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError as exc:
        assert "provenance" in str(exc)
        return
    raise AssertionError("validator accepted a TF edge with no provenance")


def test_schema_rejects_unknown_provenance():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    c["tf_static"][0]["provenance"] = "vibes"
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError:
        return
    raise AssertionError("validator accepted an invented provenance value")


def test_schema_rejects_relative_topic():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    c["topics"][0]["name"] = "scan"
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError as exc:
        assert "absolute" in str(exc)
        return
    raise AssertionError("validator accepted a relative topic name")


def test_schema_rejects_unqualified_msg_type():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    c["topics"][0]["type"] = "LaserScan"
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError:
        return
    raise AssertionError("validator accepted an unqualified message type")


def test_schema_rejects_inconsistent_laserscan_geometry():
    """ray_count, angle span and increment are over-determined. If they disagree the
    contract describes a scan that cannot exist, and every later comparison inherits it."""
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    _topic(c, "/ferox/g1_01/scan")["expect"]["laserscan"]["ray_count"] = 3200
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError as exc:
        assert "inconsistent" in str(exc)
        return
    raise AssertionError("validator accepted 3200 rays over a 2*pi span at 0.0087 rad")


def test_schema_rejects_bad_pointfield_datatype():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    _topic(c, "/livox/lidar")["expect"]["pointcloud_fields"][0]["datatype"] = "float"
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError:
        return
    raise AssertionError("validator accepted an invalid PointField datatype")


def test_schema_rejects_inverted_p2l_slice():
    c = copy.deepcopy(twin_contract.load(G1_PATH))
    c["pointcloud_to_laserscan"]["min_height"] = 1.0
    try:
        twin_contract.validate(c)
    except twin_contract.ContractError:
        return
    raise AssertionError("validator accepted min_height above max_height")


def test_quaternion_roundtrip():
    for rpy in [(0.0, 0.0, 0.0), (3.090233, 0.161680, 0.0),
                (-0.000966, -0.010877, 0.0), (0.0, 0.2249, 0.0)]:
        q = twin_contract.quat_from_rpy(*rpy)
        back = twin_contract.rpy_from_quat(*q)
        for a, b in zip(rpy, back):
            _close(a, b, 1e-9, f"rpy roundtrip {rpy}")
        # A quaternion and its negation are the same rotation; the audit compares
        # rotations, so this must read as zero difference, not 2*pi.
        neg = tuple(-v for v in q)
        _close(twin_contract.angular_distance(q, neg), 0.0, 1e-9, "q vs -q")


# ---------------------------------------------------------------- drift tripwires
#
# Every literal below is duplicated from a driver repo on purpose. See module docstring.


def test_g1_lidar_mount_matches_driver():
    """torso_link -> livox_frame, waist-independent, calibrated standing 2026-08-07.
    panthera-g1-driver config/g1_driver.yaml:137-142 (waist_tf_bridge mount_*)."""
    p = _sensor(twin_contract.load(G1_PATH), "livox_mid360")["pose"]
    assert p["xyz"] == [-0.0440849, -0.0185556, 0.4429585], p["xyz"]
    assert p["rpy"] == [3.128688, 0.052979, 0.018520], p["rpy"]
    assert p["provenance"] == "calibrated"


def test_g1_standing_composite_lidar_edge_matches_driver():
    """base_link -> livox_frame in static mode.
    g1_driver_hw.launch.py:459-478 (lidar_x/y/z, lidar_roll_rad, lidar_pitch_rad)."""
    e = _edge(twin_contract.load(G1_PATH), "base_link", "livox_frame")
    assert e["xyz"] == [0.0, 0.0, 0.4995], e["xyz"]
    assert e["rpy"] == [3.090233, 0.161680, 0.0], e["rpy"]


def test_g1_dog_imu_edge_matches_driver():
    """g1_driver_hw.launch.py:501,508 -- imu_mount_roll_rad / imu_mount_pitch_rad."""
    e = _edge(twin_contract.load(G1_PATH), "base_link", "dog_imu_link")
    assert e["xyz"] == [0.0, 0.0, 0.0], e["xyz"]
    assert e["rpy"] == [-0.000966, -0.010877, 0.0], e["rpy"]


def test_g1_livox_imu_edge_matches_driver():
    """g1_driver_hw.launch.py:495-497 -- Livox datasheet offset. NOTE this is NOT
    identity, unlike the Go2's same-named edge."""
    e = _edge(twin_contract.load(G1_PATH), "livox_frame", "livox_imu")
    assert e["xyz"] == [0.011, 0.02329, -0.04412], e["xyz"]
    assert e["rpy"] == [0.0, 0.0, 0.0], e["rpy"]


def test_g1_camera_pose_matches_driver_urdf_nominal():
    """g1_driver.yaml:188-193 -- camera_mount_*, from URDF d435_joint. 47.600 deg down."""
    p = _sensor(twin_contract.load(G1_PATH), "d435i")["pose"]
    assert p["xyz"] == [0.0576235, 0.01753, 0.41987], p["xyz"]
    _close(p["rpy"][1], 0.8307767239493009, 0.0, "camera pitch")
    _close(math.degrees(p["rpy"][1]), 47.600, 1e-3, "camera pitch in degrees")
    assert p["provenance"] == "urdf_nominal", "the camera pose is NOT calibrated"


def test_g1_p2l_matches_driver():
    """g1_driver_hw.launch.py:230-266 with defaults :424-434. Class A, exact.
    These are NOT to be re-derived to suit the sim body -- TWIN_DEVIATIONS.md C-1."""
    p = twin_contract.load(G1_PATH)["pointcloud_to_laserscan"]
    assert p["target_frame"] == "base_link"
    assert p["min_height"] == -0.556, p["min_height"]
    assert p["max_height"] == 0.50, p["max_height"]
    assert p["range_min"] == 0.30, p["range_min"]
    assert p["range_max"] == 6.0, p["range_max"]
    assert p["angle_increment"] == 0.0087, p["angle_increment"]
    assert p["scan_time"] == 0.1
    assert p["transform_tolerance"] == 0.1
    assert p["use_inf"] is True
    # The G1 uses math.pi EXACTLY; the Go2 uses a truncated 3.14159. Different files,
    # different literals -- do not "tidy" either one into the other.
    _close(p["angle_min"], -math.pi, 0.0, "g1 angle_min is exactly -math.pi")
    _close(p["angle_max"], math.pi, 0.0, "g1 angle_max is exactly math.pi")


def test_g1_scan_geometry_is_723_rays():
    """The driver's angle span and increment produce 723 rays through p2l's
    ceil((angle_max - angle_min) / angle_increment)."""
    c = twin_contract.load(G1_PATH)
    ls = _topic(c, "/ferox/g1_01/scan")["expect"]["laserscan"]
    assert ls["ray_count"] == 723, ls["ray_count"]
    implied = math.ceil((ls["angle_max"] - ls["angle_min"]) / ls["angle_increment"])
    assert implied == 723, f"p2l would emit {implied} rays, contract says {ls['ray_count']}"


def test_g1_hardware_tf_static_edge_set_is_exactly_session_a():
    """panthera-g1-driver/evidence/fw2026q3/session_a/31_tf_static.txt -- 7 edges.
    An extra edge in the twin is a Class-A failure; the sim inventing frames is the
    baseline defect this campaign exists to remove."""
    c = twin_contract.load(G1_PATH)
    # Only edges published BY DEFAULT count towards the robot's live set. A gated edge
    # is still contracted -- its values must be right when someone enables it -- but its
    # absence is what the robot actually does.
    got = {(e["parent"], e["child"]) for e in c["tf_static"]
           if e.get("default_published", True) and not e.get("dynamic")}
    expected = {
        ("base_link", "dog_imu_link"),
        ("livox_frame", "livox_imu"),
        ("camera_link", "camera_color_frame"),
        ("camera_color_frame", "camera_color_optical_frame"),
        ("camera_link", "camera_depth_frame"),
        ("camera_depth_frame", "camera_depth_optical_frame"),
    }
    assert got == expected, f"missing {expected - got}, extra {got - expected}"
    assert len(got) == 6, f"static set must be 6 edges after DT2, got {len(got)}"
    # base_link -> livox_frame is DYNAMIC (/tf), composed from the live waist by the
    # twin bridge -- the driver's default lidar_tf_mode. Session A captured the static
    # fallback at one posture. Publishing it statically as well would give one edge two
    # owners, which is baseline defect B-2 in a new costume.
    assert ("base_link", "livox_frame") not in got
    # base_link -> camera_link is NOT in the default set: the driver ships
    # camera_tf_enable:false (g1_driver.yaml:186), so camera_link is an ORPHAN ROOT on
    # the real robot with the RealSense subtree beneath it. The twin mirrors that.
    assert ("base_link", "camera_link") not in got


def test_g1_livox_edge_is_dynamic_with_a_rate():
    """The lidar edge moved to /tf at DT2 (fork resolved: option A).

    Its xyz/rpy stay in the contract as the REFERENCE the waist round-trip must
    reproduce at the robot's standing waist angle -- they are not what is published
    at rest, because our policy stands with the waist at 0."""
    e = _edge(twin_contract.load(G1_PATH), "base_link", "livox_frame")
    assert e.get("dynamic") is True
    assert e["rate_hz"] == 100.0, e.get("rate_hz")
    assert e["xyz"] == [0.0, 0.0, 0.4995]
    assert e["rpy"] == [3.090233, 0.161680, 0.0]


def test_g1_camera_link_edge_is_contracted_but_gated_off():
    """F-6. The edge exists in the contract with URDF-exact values so CAMERA_TF=1 (sim)
    / camera_tf_enable (driver) produces the right transform -- but default_published is
    False, so twin_audit SKIPs it when absent instead of failing."""
    e = _edge(twin_contract.load(G1_PATH), "base_link", "camera_link")
    assert e["default_published"] is False
    assert "CAMERA_TF" in e["conditional"], e["conditional"]
    assert e["xyz"] == [0.0576235, 0.01753, 0.41987], e["xyz"]
    _close(e["rpy"][1], 0.8307767239493009, 0.0, "gated camera edge pitch")


def test_g1_cloud_layout_is_the_sidecar_driver_not_the_retired_unitree_stream():
    """OQ-2. /livox/lidar comes from OUR livox_ros_driver2 sidecar, so the default layout
    is that driver's: x,y,z,intensity FLOAT32 + tag,line UINT8 + timestamp FLOAT64,
    point_step 26 -- NOT the retired Unitree /utlidar/cloud_livox_mid360 layout
    (ring UINT16 + time FLOAT32, point_step 22). Marked assumed until an echo confirms."""
    t = _topic(twin_contract.load(G1_PATH), "/livox/lidar")
    fields = [(f["name"], f["datatype"]) for f in t["expect"]["pointcloud_fields"]]
    assert fields == [("x", "FLOAT32"), ("y", "FLOAT32"), ("z", "FLOAT32"),
                      ("intensity", "FLOAT32"), ("tag", "UINT8"), ("line", "UINT8"),
                      ("timestamp", "FLOAT64")], fields
    assert t["expect"]["point_step"] == 26
    assert t["provenance"] == "assumed", "the field layout is not measured yet"
    # The Go2's stream layout is genuinely unknown and stays on the captured Unitree one.
    go2 = _topic(twin_contract.load(GO2_PATH), "/unitree/slam_lidar/points")
    assert go2["expect"]["point_step"] == 22


def test_g1_camera_wire_matches_realsense_driver():
    """realsense_driver README.md:22-32, params.yaml:25. The encodings and frames are
    Class A: consumers gate on the exact strings."""
    c = twin_contract.load(G1_PATH)
    colour = _topic(c, "/ferox/g1_01/camera/color/image_raw")
    assert colour["expect"]["encoding"] == "rgb8"
    assert colour["frame_id"] == "camera_color_optical_frame"
    assert colour["expect"] is not None
    depth = _topic(c, "/ferox/g1_01/camera/aligned_depth_to_color/image_raw")
    assert depth["expect"]["encoding"] == "16UC1"
    # Aligned depth is stamped in the COLOUR optical frame, same stamp as its colour frame.
    assert depth["frame_id"] == "camera_color_optical_frame"
    for t in (colour, depth):
        assert t["qos"]["reliability"] == "reliable"
    cloud = _topic(c, "/ferox/g1_01/camera/depth/color/points")
    # Intentional asymmetry: the cloud is in the DEPTH optical frame though textured
    # from colour, and it is the ONLY camera topic that is BEST_EFFORT.
    assert cloud["frame_id"] == "camera_depth_optical_frame"
    assert cloud["qos"]["reliability"] == "best_effort"
    for name in ("/ferox/g1_01/camera/color/camera_info",
                 "/ferox/g1_01/camera/aligned_depth_to_color/camera_info"):
        ci = _topic(c, name)["expect"]["camera_info"]
        assert (ci["width"], ci["height"]) == (1280, 720), name


def test_g1_odom_rate_is_51_4_not_the_aliased_capture():
    """51.4 Hz, the driver's true pass-through of the 51.45 Hz source.

    NOT 26.2 Hz. evidence/fw2026q3/session_a/30_evidence.txt measures 26.20 Hz on this
    topic with a QoS-matched subscriber -- but that capture (companion now()
    1786465672, 2026-08-11T16:27:52Z) PREDATES driver commit abd0d0a (1786466558,
    2026-08-11T16:42:38Z) by 14.8 minutes. abd0d0a, "fix: odom rate aliasing (26.6 Hz
    not 50)", sets odom_publish_rate_hz 50.0 -> 0.0 because a 50 Hz gate against a
    51.45 Hz source (19.44 ms interval vs 20 ms gate period) drops nearly every second
    sample. The capture recorded the bug, not the interface.

    This test exists in BOTH directions: it stops anyone restoring 26.2 from the
    evidence file without reading the commit, and it pins the rate the twin must hit."""
    t = _topic(twin_contract.load(G1_PATH), "/ferox/g1_01/odom")
    assert t["rate_hz"] == 51.4, t["rate_hz"]
    # Guard the trap itself: 26.2 is the aliased value and must never come back.
    assert t["rate_hz"] != 26.2


def test_go2_mid360_mount_matches_driver():
    """go2_driver_hw.launch.py:308-317 -- livox_x/y/z and livox_pitch defaults."""
    p = _sensor(twin_contract.load(GO2_PATH), "livox_mid360")["pose"]
    assert p["xyz"] == [0.187, 0.0, 0.0803], p["xyz"]
    assert p["rpy"] == [0.0, 0.2249, 0.0], p["rpy"]


def test_go2_l1_extrinsics_match_driver():
    """go2_driver_hw.launch.py:299-306 -- lidar_* and utlidar_imu_*, defaults equal."""
    c = twin_contract.load(GO2_PATH)
    for child in ("utlidar_lidar", "utlidar_imu"):
        e = _edge(c, "base_link", child)
        assert e["xyz"] == [0.28, 0.0, 0.075], (child, e["xyz"])
        assert e["rpy"] == [0.0, 0.0, 0.0], (child, e["rpy"])
        # Both are flagged unverified in the launch file; neither may claim calibration.
        assert e["provenance"] == "assumed", child


def test_go2_livox_imu_is_identity_by_assumption_unlike_the_g1():
    """The same-named edge differs between robots and MUST NOT be harmonised:
    Go2 go2_driver_hw.launch.py:395-405 hardcodes identity 'by assumption';
    G1 g1_driver_hw.launch.py:495-497 uses the Livox datasheet offset."""
    go2 = _edge(twin_contract.load(GO2_PATH), "livox_frame", "livox_imu")
    g1 = _edge(twin_contract.load(G1_PATH), "livox_frame", "livox_imu")
    assert go2["xyz"] == [0.0, 0.0, 0.0]
    assert go2["provenance"] == "assumed"
    assert g1["xyz"] != go2["xyz"], "the two robots' livox_imu edges are not the same"


def test_go2_p2l_matches_driver():
    """go2_driver_hw.launch.py:155-180, presets :64-68, scan_max_height :286."""
    p = twin_contract.load(GO2_PATH)["pointcloud_to_laserscan"]
    assert p["target_frame"] == "base_link"
    assert p["min_height"] == -0.20, p["min_height"]
    assert p["max_height"] == 0.50, p["max_height"]
    assert p["range_min"] == 0.30, p["range_min"]
    assert p["range_max"] == 6.0, p["range_max"]
    assert p["angle_increment"] == 0.0087
    assert p["scan_time"] == 0.1
    # Truncated pi in this file, exact math.pi in the G1's. Recorded as written.
    assert p["angle_min"] == -3.14159, p["angle_min"]
    assert p["angle_max"] == 3.14159, p["angle_max"]
    assert p["angle_min"] != -math.pi, "the Go2 file does not use math.pi"


def test_go2_sensor_topics_are_root_not_namespaced():
    """go2_driver_hw.launch.py:5-6 'No namespace push here.' Ferox depends on it:
    go2_nav2.yaml:43-47 subscribes ABSOLUTE /scan and warns that a relative name
    resolves to /ferox/<id>/scan 'which nobody publishes -> AMCL silently starves'.
    The campaign brief section 3 lists /ferox/go2_01/scan; the file wins."""
    c = twin_contract.load(GO2_PATH)
    assert twin_contract.topic_by_name(c, "/scan") is not None
    assert twin_contract.topic_by_name(c, "/odom") is not None
    assert twin_contract.topic_by_name(c, "/ferox/go2_01/scan") is None
    assert twin_contract.topic_by_name(c, "/ferox/go2_01/odom") is None
    # Control plane IS namespaced, and that asymmetry is the contract.
    assert twin_contract.topic_by_name(c, "/ferox/go2_01/cmd_vel") is not None


def test_go2_has_no_imu_data_topic():
    """There is no IMU republisher in panthera_go2_driver (the package ships
    clock_util, cloud_accumulator, cmd_vel_to_sport, odom_tf_bridge, rate_limiter,
    sport_watchdog -- no imu node). The campaign brief lists /ferox/go2_01/imu/data;
    it does not exist and the twin must not invent it."""
    c = twin_contract.load(GO2_PATH)
    assert twin_contract.topic_by_name(c, "/ferox/go2_01/imu/data") is None


def test_go2_stvl_source_topic_is_published():
    """Ferox go2_nav2.yaml:289 points the STVL layer at /unitree/slam_lidar/points.
    If the twin does not publish that exact name the Go2 local costmap goes blind."""
    c = twin_contract.load(GO2_PATH)
    t = _topic(c, "/unitree/slam_lidar/points")
    assert t["frame_id"] == "livox_frame"
    assert t["qos"]["reliability"] == "reliable", "DEV_LOG corrected this from BEST_EFFORT"


def test_go2_tf_static_includes_the_two_edges_the_brief_omitted():
    c = twin_contract.load(GO2_PATH)
    got = {(e["parent"], e["child"]) for e in c["tf_static"]}
    assert ("base_link", "utlidar_imu") in got
    assert ("base_link", "robot_center") in got
    assert len(got) == 5, got


def test_mid360_rate_differs_between_robots():
    """The Go2's sidecar runs publish_freq:=20.0; the G1's runs at 10 Hz (measured
    10.000 in session_a/13_livox_lidar.txt). Reproducing 10 Hz on the Go2 would halve
    the real update rate its costmap sees."""
    g1 = _sensor(twin_contract.load(G1_PATH), "livox_mid360")["model_params"]
    go2 = _sensor(twin_contract.load(GO2_PATH), "livox_mid360")["model_params"]
    assert g1["scan_rate_hz"] == 10.0
    assert go2["scan_rate_hz"] == 20.0


# --- RULE-HAND-NAME (C-14) --------------------------------------------------

JOINTISH = ("dof", "joint_position", "joint_velocit", "joint_name", "joint_state",
            "joint_target", "joint_effort", "q_hand", "hand_q")


def _hand_dof_block(contract_path):
    """The forbidden index range, read from the contract rather than hardcoded."""
    c = twin_contract.load(contract_path)
    rules = {r["id"]: r for r in c.get("rules", [])}
    assert "RULE-HAND-NAME" in rules, f"{contract_path} does not declare RULE-HAND-NAME"
    lo, hi = rules["RULE-HAND-NAME"]["hand_dof_block"]
    return int(lo), int(hi)


def _int_constants(node):
    """Every integer literal appearing anywhere in a subscript's index."""
    out = []
    for sub in ast.walk(node):
        if isinstance(sub, ast.Constant) and isinstance(sub.value, int) \
                and not isinstance(sub.value, bool):
            out.append(sub.value)
    return out


def _subscript_target_name(node):
    """A readable name for whatever is being subscripted."""
    v = node.value
    while isinstance(v, ast.Call):
        v = v.func
    if isinstance(v, ast.Attribute):
        return v.attr
    if isinstance(v, ast.Name):
        return v.id
    if isinstance(v, ast.Subscript):
        return _subscript_target_name(v)
    return ""


def test_rule_hand_name_no_numeric_hand_indexing():
    """No joint array may be indexed with a literal in the hand DOF block.

    RULE-HAND-NAME, enforcing C-14. Isaac interleaves the hand DOFs (left 29..63,
    right 34..68, neither contiguous), so a literal index into that block is either
    already wrong or one Unitree URDF revision away from being wrong -- and it fails
    silently, moving fingers on the wrong hand.

    Body-block indices are deliberately NOT flagged: dof_names[:29] is legitimate and
    is asserted bit-identical by the Isaac suite. Only the hand block is forbidden.
    """
    lo, hi = _hand_dof_block(os.path.join(ISAAC_TWIN, "g1_contract.yaml"))
    roots = [os.path.join(REPO, d) for d in ("isaac", "tools")]
    offenders = []
    for root in roots:
        for dirpath, _dirs, files in os.walk(root):
            if "__pycache__" in dirpath:
                continue
            for fn in files:
                if not fn.endswith(".py"):
                    continue
                path = os.path.join(dirpath, fn)
                try:
                    tree = ast.parse(open(path, encoding="utf-8").read())
                except SyntaxError:
                    continue
                for node in ast.walk(tree):
                    if not isinstance(node, ast.Subscript):
                        continue
                    target = _subscript_target_name(node).lower()
                    if not any(tok in target for tok in JOINTISH):
                        continue
                    for val in _int_constants(node.slice):
                        if lo <= val <= hi:
                            offenders.append(
                                f"{os.path.relpath(path, REPO)}:{node.lineno}: "
                                f"{target}[...{val}...] indexes the hand DOF block "
                                f"[{lo},{hi}]")
    assert not offenders, (
        "RULE-HAND-NAME violated -- hand joints must map by name (C-14):\n  "
        + "\n  ".join(offenders))


def test_rule_hand_name_is_declared_everywhere():
    """The rule must be in both contracts and in CLAUDE.md, or it is not a rule."""
    for robot in ("g1", "go2"):
        lo, hi = _hand_dof_block(os.path.join(ISAAC_TWIN, f"{robot}_contract.yaml"))
        assert (lo, hi) == (29, 68), f"{robot}: hand block {(lo, hi)} != (29, 68)"
    claude = os.path.join(REPO, "CLAUDE.md")
    assert os.path.exists(claude), "CLAUDE.md is missing"
    text = open(claude, encoding="utf-8").read()
    assert "RULE-HAND-NAME" in text, "CLAUDE.md does not carry RULE-HAND-NAME"
    assert "never slice by index" in text


def test_rule_hand_name_catches_a_violation():
    """The check must actually fire -- a linter that cannot fail is decoration."""
    bad = ast.parse("hand = art.get_joint_positions()\nx = hand_q[33]\n")
    lo, hi = 29, 68
    hits = []
    for node in ast.walk(bad):
        if isinstance(node, ast.Subscript):
            target = _subscript_target_name(node).lower()
            if any(tok in target for tok in JOINTISH):
                hits += [v for v in _int_constants(node.slice) if lo <= v <= hi]
    assert hits == [33], f"the RULE-HAND-NAME check did not fire: {hits}"

    good = ast.parse("i = names.index('Pitch_13L')\nx = joint_positions[i]\n")
    hits = []
    for node in ast.walk(good):
        if isinstance(node, ast.Subscript):
            target = _subscript_target_name(node).lower()
            if any(tok in target for tok in JOINTISH):
                hits += [v for v in _int_constants(node.slice) if lo <= v <= hi]
    assert hits == [], f"the check fired on a name-based lookup: {hits}"


def main() -> int:
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:
            failed.append((name, exc))
            print(f"  FAIL  {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
