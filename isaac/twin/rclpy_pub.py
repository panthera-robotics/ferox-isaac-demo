"""rclpy publishers for the twin, where the OmniGraph path does not work.

This exists for one measured reason. The ROS 2 bridge's ROS2PublishImu node builds
without error, logs success, and then advertises nothing — that is baseline defect
B-4, and it is why /ferox/<id>/imu/data has been silent in this sim since before
the campaign started. The same class of failure already forced sim_utils to
publish cmd_vel through rclpy instead of OmniGraph ("OmniGraph version fails
silently on Isaac Sim 5.1"), so this follows the precedent the repo already set.

Publishing from Python also buys exact rate control: the contract asks for 100 Hz
on imu/data and 200 Hz on /livox/imu, and an OnTick graph would instead run at
whatever the render loop happens to manage.
"""

from __future__ import annotations

import threading
from typing import Optional


class TwinRclpyPublishers:
    def __init__(self, node_name: str = "ferox_twin_publishers") -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor

        if not rclpy.ok():
            rclpy.init()
        self._node: Node = rclpy.create_node(node_name)
        self._pubs = {}
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._stop = False
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        import rclpy
        try:
            while rclpy.ok() and not self._stop:
                self._executor.spin_once(timeout_sec=0.05)
        except Exception as exc:  # pragma: no cover - background thread
            print(f"[TWIN] rclpy publisher thread exited: {exc}", flush=True)

    # ------------------------------------------------------------------ imu

    def add_imu(self, key: str, topic: str, reliable: bool = True) -> None:
        from sensor_msgs.msg import Imu
        from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                               QoSHistoryPolicy, QoSDurabilityPolicy)
        qos = QoSProfile(
            depth=10, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=(QoSReliabilityPolicy.RELIABLE if reliable
                         else QoSReliabilityPolicy.BEST_EFFORT),
            durability=QoSDurabilityPolicy.VOLATILE)
        self._pubs[key] = self._node.create_publisher(Imu, topic, qos)

    def publish_imu(self, key: str, frame_id: str, sim_time: float,
                    lin_acc, ang_vel, orientation_wxyz: Optional[tuple] = None) -> None:
        """Publish one Imu stamped in SIM time.

        The stamp must come from the sim clock, not the wall clock: every twin
        consumer runs use_sim_time, and a wall-clock stamp here would look like a
        message from the far future and be dropped by every TF-buffered subscriber.
        """
        from sensor_msgs.msg import Imu
        pub = self._pubs.get(key)
        if pub is None:
            return
        m = Imu()
        m.header.stamp.sec = int(sim_time)
        m.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        m.header.frame_id = frame_id
        if orientation_wxyz is not None:
            w, x, y, z = (float(v) for v in orientation_wxyz)
            m.orientation.w, m.orientation.x = w, x
            m.orientation.y, m.orientation.z = y, z
        else:
            m.orientation.w = 1.0
        m.linear_acceleration.x = float(lin_acc[0])
        m.linear_acceleration.y = float(lin_acc[1])
        m.linear_acceleration.z = float(lin_acc[2])
        m.angular_velocity.x = float(ang_vel[0])
        m.angular_velocity.y = float(ang_vel[1])
        m.angular_velocity.z = float(ang_vel[2])
        # Hardware publishes all-zero covariance; the twin matches rather than
        # inventing an uncertainty the robot does not report.
        pub.publish(m)

    def shutdown(self) -> None:
        self._stop = True
