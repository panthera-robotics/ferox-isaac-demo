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
import os
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
    rp_name = "twin_mid360_rp"
    rp = rep.create.render_product(
        lidar_prim.GetPath().pathString, resolution=resolution, name=rp_name)

    # FULL SCAN, NOT ONE RENDER FRAME'S SLICE.
    #
    # Isaac 5.1 registers TWO ROS 2 point-cloud writers for an RTX lidar, and the
    # difference between them is the whole of this defect:
    #
    #   RtxLidarROS2PublishPointCloud        -> IsaacExtractRTXSensorPointCloudNoAccumulator
    #                                           = IsaacCreateRTXLidarScanBuffer with
    #                                             init_params {"enablePerFrameOutput": True}
    #   RtxLidarROS2PublishPointCloudBuffer  -> IsaacCreateRTXLidarScanBuffer, accumulating
    #
    # (isaacsim.sensors.rtx/impl/extension.py register_nodes; isaacsim.ros2.bridge/
    # impl/extension.py:342-353. The bridge's own ROS2RtxLidarHelper picks between
    # them on its `fullScan` input, OgnROS2RtxLidarHelper.py:110-113.)
    #
    # The twin used the first one. Its output is whatever the renderer swept during
    # ONE RENDER FRAME, so each published cloud was a SECTOR: measured on the Go2 at
    # 190 deg of 360 (19 of 36 ten-degree bins, 4490 points against 10000 per
    # revolution), and on the G1 at ~72 deg / 4285 points against 20000. Worse, the
    # decimation below then sampled the SAME PHASE every time -- the union of five
    # consecutive messages was still the same 190 deg, so nothing downstream ever saw
    # the other half. SLAM built a fixed wedge and /scan was ~20% finite against the
    # robot's 70%. After the switch: 360 deg per message, 36 of 36 bins, 8977 valid
    # points against 4490.
    #
    # Nothing at the topic level could see it: right name, right frame, right
    # point_step, a plausible point count, and exactly the contract's 10 Hz. This is
    # why twin_audit now measures per-message azimuth coverage.
    #
    # NOTE: `omni:sensor:Core:accumulateOutputs` does NOT exist. It was the obvious
    # candidate and the prim carries no such attribute -- 82 omni:sensor:* attributes
    # were enumerated and none matches, the same way emitterStateCount turned out to
    # be JSON-profile-only (see lidar.py). Accumulation is an ANNOTATOR choice, not a
    # prim attribute. tickRate == scanRateBaseHz is still required and still asserted
    # in lidar.py; it is necessary and, on its own, was not sufficient.
    w = rep.writers.get("RtxLidarROS2PublishPointCloudBuffer")
    w.initialize(frameId=spec["frame_id"], nodeNamespace="",
                 topicName=spec["name"], queueSize=10)
    w.attach([rp])

    # DECIMATE TO THE SENSOR'S SCAN RATE -- still required with the Buffer writer.
    #
    # The accumulating annotator does NOT emit once per completed revolution: it emits
    # on every render frame, each time carrying the most recent complete scan. So the
    # gate below is what turns that into one message per revolution, and it works --
    # measured on the G1 in SIM time: 190 messages, sim deltas exactly 0.1000 s,
    # ZERO duplicate stamps, 10.00 Hz against a 10 Hz contract, while each message
    # carries a full 360 deg sweep.
    #
    # MEASURE THE STAMPS, NOT THE WALL CLOCK. `ros2 topic hz` reports wall-clock rate,
    # and the sim does not run at 1.0x -- the G1 in `hospital` measures a real-time
    # factor of 1.26, so its perfectly conformant 10 Hz cloud reads as 12.6 Hz on the
    # wall. That confounding very nearly got a rate deviation opened against a stream
    # that was exactly right. Header stamps are sim time and are the thing the
    # contract is about.
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
                f"(render {render_hz} Hz, step {step}): {exc}. Without decimation the "
                "accumulated scan is republished at the render rate."
            ) from exc

        # A read-back is not enough when the node it reads is not in the path --
        # worth writing down, because rule 7 is usually satisfied by exactly that.
        gates = _simulation_gates()
        for path, got in gates:
            print(f"[TWIN]   gate {path} step={got}", flush=True)
        if os.environ.get("TWIN_DUMP_SDG") == "1":
            for line in _sdg_nodes(rp_name):
                print(f"[TWIN]   sdg {line}", flush=True)
        if not any(got == step for _, got in gates):
            raise PublisherError(
                f"{spec['name']}: asked for simulation-gate step {step} but no gate "
                f"node carries it -- found {gates}.")
    return rp, spec


def _sdg_nodes(tag: str):
    """Every OmniGraph node whose path mentions `tag`, with type and step.

    Diagnostic only, behind TWIN_DUMP_SDG=1. Kept because working out which node
    in the synthetic-data pipeline actually gates a writer took several sim boots,
    and the next person should be able to look instead of guess.
    """
    import omni.graph.core as og
    out = []
    try:
        graphs = og.get_all_graphs()
    except Exception:
        return out
    for graph in graphs:
        try:
            nodes = graph.get_nodes()
        except Exception:
            continue
        for node in nodes:
            try:
                path = node.get_prim_path()
            except Exception:
                continue
            if tag not in path:
                continue
            try:
                tname = node.get_type_name()
            except Exception:
                tname = "?"
            extra = ""
            try:
                extra = f" step={og.Controller.get(og.Controller.attribute('inputs:step', node))}"
            except Exception:
                pass
            conns = []
            try:
                for attr in node.get_attributes():
                    an = attr.get_name()
                    if not an.startswith("inputs:"):
                        continue
                    try:
                        ups = attr.get_upstream_connections()
                    except Exception:
                        continue
                    for up in ups:
                        conns.append(f"{an} <- {up.get_node().get_prim_path().split('/')[-1]}.{up.get_name()}")
            except Exception:
                pass
            out.append(f"{path}  type={tname}{extra}" +
                       ("".join(f"\n         {c}" for c in conns) if conns else ""))
    return sorted(out)


def _simulation_gates():
    """Every IsaacSimulationGate node in the stage, with its inputs:step.

    Enumerated rather than looked up by template name, because the template name
    is what stopped resolving to the right node when the writer changed.
    """
    import omni.graph.core as og
    out = []
    try:
        graphs = og.get_all_graphs()
    except Exception:
        return out
    for graph in graphs:
        try:
            nodes = graph.get_nodes()
        except Exception:
            continue
        for node in nodes:
            try:
                path = node.get_prim_path()
                tname = node.get_type_name()
            except Exception:
                continue
            if "SimulationGate" not in path and "SimulationGate" not in tname:
                continue
            try:
                got = og.Controller.get(og.Controller.attribute("inputs:step", node))
            except Exception:
                got = None
            out.append((path, got))
    return out


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
