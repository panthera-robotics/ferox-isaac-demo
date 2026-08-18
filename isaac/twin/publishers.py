"""Publish the twin's ROS 2 interface, driven entirely by the contract.

Every topic name, frame id, QoS and TF edge below comes from
isaac/twin/<robot>_contract.yaml. Nothing is hardcoded here that the audit does
not also read from that same file, so the two cannot drift apart.

The contrast with the legacy path is the point. setup_static_tfs() in sim_utils.py
carries a hardcoded table commented "static transforms for Go2" and publishes it
for BOTH robots -- which is baseline defect B-1, the G1 advertising the Go2's
sensor offsets. Here the edge set IS the contract.
"""

from __future__ import annotations

import json
from typing import Any, Dict, List

# QoS is settable per publisher via inputs:qosProfile as a JSON string; this is
# the exact shape the C++ side consumes (OgnROS2QoSProfile.py).
QOS_RELIABLE = json.dumps({
    "history": "keepLast", "depth": 10, "reliability": "reliable",
    "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
    "liveliness": "systemDefault", "leaseDuration": 0.0,
})
QOS_SENSOR_DATA = json.dumps({
    "history": "keepLast", "depth": 5, "reliability": "bestEffort",
    "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
    "liveliness": "systemDefault", "leaseDuration": 0.0,
})
QOS_TF_STATIC = json.dumps({
    "history": "keepLast", "depth": 10, "reliability": "reliable",
    "durability": "transientLocal", "deadline": 0.0, "lifespan": 0.0,
    "liveliness": "systemDefault", "leaseDuration": 0.0,
})


class PublisherError(RuntimeError):
    """A publisher that does not match the contract. Always fatal."""


def _topic(contract: Dict[str, Any], suffix: str) -> Dict[str, Any]:
    """Look up a contract topic by its full name, or by namespace + suffix."""
    ns = contract["robot"]["namespace"]
    full = suffix if suffix.startswith("/") else f"{ns}/{suffix}"
    for t in contract.get("topics", []):
        if t["name"] == full:
            return t
    raise PublisherError(f"contract has no topic {full!r}")


def _topic_of_type(contract: Dict[str, Any], msg_type: str) -> Dict[str, Any]:
    """The contract's single topic of a given message type.

    Used where the NAME differs between robots but the role does not: the G1
    publishes odometry on /ferox/g1_01/odom and the Go2 on /odom at the root,
    because that is what each driver does. Looking it up by name would need a
    per-robot branch; looking it up by type asks the question that is actually
    being asked, and raises if a robot ever grows two.
    """
    hits = [t for t in contract.get("topics", []) if t["type"] == msg_type]
    if not hits:
        raise PublisherError(f"contract has no {msg_type} topic")
    if len(hits) > 1:
        raise PublisherError(
            f"contract has {len(hits)} {msg_type} topics: "
            f"{[t['name'] for t in hits]}; look it up by name")
    return hits[0]


def _qos_for(topic_spec: Dict[str, Any]) -> str:
    rel = (topic_spec.get("qos") or {}).get("reliability", "reliable")
    return QOS_SENSOR_DATA if rel == "best_effort" else QOS_RELIABLE


def tf_static_edges(contract: Dict[str, Any], camera_tf: bool) -> List[Dict[str, Any]]:
    """Exactly the edges the robot publishes, and no others.

    A gated edge (base_link->camera_link, driver default camera_tf_enable:false)
    is included only when its gate is on. With the gate off, camera_link is an
    orphan root carrying the RealSense subtree -- which is what the robot's own
    /tf_static shows, and what twin_audit expects.
    """
    out = []
    for e in contract.get("tf_static", []):
        # A dynamic edge belongs on /tf, published by the twin bridge's waist
        # composition -- not here. Publishing it statically as well would give one
        # edge two owners, which is baseline defect B-2 in a new costume.
        if e.get("dynamic"):
            continue
        if e.get("default_published") is False and not camera_tf:
            continue
        out.append(e)
    return out


def setup_tf_static(contract: Dict[str, Any], camera_tf: bool = False):
    """One ROS2PublishRawTransformTree per contract edge, on /tf_static."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid
    import twin_contract

    graph_path = "/TwinStaticTFGraph"
    if is_prim_path_valid(graph_path):
        return []

    edges = tf_static_edges(contract, camera_tf)
    create = [("OnTick", "omni.graph.action.OnTick"),
              ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
              ("Ctx", "isaacsim.ros2.bridge.ROS2Context")]
    connect, values = [], [("Ctx.inputs:useDomainIDEnvVar", True)]

    for i, e in enumerate(edges):
        n = f"TF{i}"
        create.append((n, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        connect += [("OnTick.outputs:tick", f"{n}.inputs:execIn"),
                    ("Clock.outputs:simulationTime", f"{n}.inputs:timeStamp"),
                    ("Ctx.outputs:context", f"{n}.inputs:context")]
        qx, qy, qz, qw = twin_contract.quat_from_rpy(*e["rpy"])
        values += [
            (f"{n}.inputs:parentFrameId", e["parent"]),
            (f"{n}.inputs:childFrameId", e["child"]),
            (f"{n}.inputs:topicName", "/tf_static"),
            (f"{n}.inputs:translation", [float(v) for v in e["xyz"]]),
            (f"{n}.inputs:rotation", [qx, qy, qz, qw]),
            (f"{n}.inputs:staticPublisher", True),
            (f"{n}.inputs:qosProfile", QOS_TF_STATIC),
        ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION},
        {og.Controller.Keys.CREATE_NODES: create,
         og.Controller.Keys.CONNECT: connect,
         og.Controller.Keys.SET_VALUES: values},
    )
    return edges


def _sensor_frame(contract: Dict[str, Any]) -> str:
    """The Mid-360's frame name, from the contract's sensors block."""
    for s in contract.get("sensors", []):
        if s["name"] == "livox_mid360":
            return s.get("frame_id", "livox_frame")
    return "livox_frame"


def setup_lidar_cloud(contract: Dict[str, Any], lidar_prim, resolution=(1, 1),
                      render_hz: float = 60.0):
    """RTX lidar -> PointCloud2 on the contract's cloud topic.

    NOTE ON FIELDS: RtxLidarROS2PublishPointCloud emits x,y,z only (measured on the
    legacy /unitree_lidar stream: 3 fields, point_step 12). The contract asks for
    the livox_ros_driver2 layout (x,y,z,intensity,tag,line,timestamp,
    point_step 26). The gap is a declared Class-C deviation, not a silent one --
    twin_audit reports it as a pointcloud/fields difference every run.
    """
    import omni.replicator.core as rep

    # The cloud topic is named differently on each robot (/livox/lidar on the G1,
    # /unitree/slam_lidar/points on the Go2) and both are PointCloud2 in the
    # lidar's own frame. Select on frame_id, which is the thing that identifies it.
    lidar_frame = _sensor_frame(contract)
    clouds = [t for t in contract.get("topics", [])
              if t["type"] == "sensor_msgs/msg/PointCloud2"
              and t["frame_id"] == lidar_frame
              and t.get("default_published", True)
              and t.get("produced_by", "sensor") == "sensor"]
    if len(clouds) != 1:
        raise PublisherError(
            f"expected exactly one published PointCloud2 in {lidar_frame!r}, got "
            f"{[t['name'] for t in clouds]}")
    spec = clouds[0]
    print(f"[TWIN] Mid-360 -> {spec['name']}", flush=True)
    rp = rep.create.render_product(
        lidar_prim.GetPath().pathString, resolution=resolution, name="twin_mid360_rp")
    w = rep.writers.get("RtxLidarROS2PublishPointCloud")
    w.initialize(frameId=spec["frame_id"], nodeNamespace="",
                 topicName=spec["name"], queueSize=10)
    w.attach([rp])

    # DECIMATE TO THE SENSOR'S SCAN RATE. Without this the writer fires once per
    # RENDER frame (~60 Hz of sim time), so every published message is a PARTIAL
    # sweep -- measured 66.44 Hz against a contract rate of 10. The robot emits one
    # message per full revolution. A partial sweep still looks like a valid cloud to
    # every consumer, just with a fraction of the points and a rate nothing expects,
    # which is exactly the sort of wrong that passes a topic-level check.
    if render_hz:
        step = max(1, int(round(render_hz / float(spec["rate_hz"]))))
        # set_node_attributes wants the render product PATH; rep.create.render_product
        # returns a HydraTexture object, whose .path carries it.
        rp_path = getattr(rp, "path", None) or str(rp)
        try:
            import omni.syntheticdata
            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "PostProcessDispatchIsaacSimulationGate", {"inputs:step": step}, rp_path)
        except Exception as exc:
            raise PublisherError(
                f"could not decimate {spec['name']} to {spec['rate_hz']} Hz "
                f"(render {render_hz} Hz, step {step}): {exc}. Without decimation "
                "every message is a partial sweep."
            ) from exc
    return rp, spec


def setup_imu(contract: Dict[str, Any], imu_prim_path: str, suffix: str, frame: str):
    """IsaacReadIMU -> ROS2PublishImu on a contract topic."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid

    spec = _topic(contract, suffix)
    graph_path = f"/TwinImuGraph_{spec['name'].strip('/').replace('/', '_')}"
    if is_prim_path_valid(graph_path):
        return spec

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution",
         "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION},
        {og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("Read", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("Pub", "isaacsim.ros2.bridge.ROS2PublishImu")],
         og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "Read.inputs:execIn"),
            ("Read.outputs:execOut", "Pub.inputs:execIn"),
            ("Ctx.outputs:context", "Pub.inputs:context"),
            ("Clock.outputs:simulationTime", "Pub.inputs:timeStamp"),
            ("Read.outputs:angVel", "Pub.inputs:angularVelocity"),
            ("Read.outputs:linAcc", "Pub.inputs:linearAcceleration"),
            ("Read.outputs:orientation", "Pub.inputs:orientation")],
         og.Controller.Keys.SET_VALUES: [
            ("Ctx.inputs:useDomainIDEnvVar", True),
            ("Read.inputs:imuPrim", imu_prim_path),
            ("Read.inputs:readGravity", True),
            # The graph publishes the ABSOLUTE topic name. The legacy path set
            # topicName="imu/data" with no namespace, which would have landed on
            # /imu/data while the Ferox bridge relayed /imu -- baseline defect B-4,
            # where the names could not have matched even if the graph had worked.
            ("Pub.inputs:topicName", spec["name"]),
            ("Pub.inputs:frameId", frame),
            ("Pub.inputs:queueSize", 10),
            ("Pub.inputs:qosProfile", _qos_for(spec))]},
    )
    return spec


def setup_camera_color(contract: Dict[str, Any], camera, ros_namespace: str):
    """Colour image on the contract topic, in the contract's optical frame."""
    import omni.replicator.core as rep
    import omni.syntheticdata as syn_data
    import omni.syntheticdata._syntheticdata as sd

    spec = _topic(contract, "camera/color/image_raw")
    rp = camera.get_render_product_path()
    if not rp:
        raise PublisherError("colour camera has no render product")
    rv = syn_data.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    w = rep.writers.get(rv + "ROS2PublishImage")
    # topicName is relative and the bridge prepends nodeNamespace, so pass the
    # suffix and let it compose -- matching how the legacy colour writer worked.
    w.initialize(frameId=spec["frame_id"], nodeNamespace=ros_namespace,
                 queueSize=10, topicName="camera/color/image_raw")
    w.attach([rp])
    return rp, spec


def setup_camera_depth_raw(contract: Dict[str, Any], camera, ros_namespace: str,
                           topic_suffix: str = "camera/depth/image_rect_raw_32f"):
    """Publish the raw 32FC1 depth for the twin bridge's converter to consume.

    The Isaac Sim 5.1 ROS 2 bridge cannot emit 16UC1, and cannot emit an xyzrgb
    PointCloud2 -- both are contract items. So the sim publishes what it CAN
    (32FC1 metres, from the SAME render product as the colour image, hence the
    same viewpoint and the same stamp) and ferox_nav_sim's twin bridge converts.

    This topic is NOT in the contract: it is an internal seam, deliberately named
    so it cannot be mistaken for a hardware topic.
    """
    import omni.replicator.core as rep
    import omni.syntheticdata as syn_data
    import omni.syntheticdata._syntheticdata as sd

    rp = camera.get_render_product_path()
    rv = syn_data.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name)
    w = rep.writers.get(rv + "ROS2PublishImage")
    w.initialize(frameId=_topic(contract, "camera/color/image_raw")["frame_id"],
                 nodeNamespace=ros_namespace, queueSize=10, topicName=topic_suffix)
    w.attach([rp])
    return topic_suffix
