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

    # ----------------------------------------------------------------- odom

    def add_image(self, key: str, topic: str) -> None:
        """Image publisher, BEST_EFFORT like every real camera driver.

        RELIABLE on a 1280x720 stream backs the executor up behind a slow subscriber
        and the sim's frame rate goes with it; the D435i driver this twin mirrors is
        best-effort for exactly that reason.
        """
        from sensor_msgs.msg import Image
        from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                               QoSHistoryPolicy, QoSDurabilityPolicy)
        qos = QoSProfile(
            depth=2, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)
        self._pubs[key] = self._node.create_publisher(Image, topic, qos)

    def publish_image(self, key: str, msg) -> None:
        pub = self._pubs.get(key)
        if pub is not None:
            pub.publish(msg)

    def add_odom(self, key: str, topic: str) -> None:
        from nav_msgs.msg import Odometry
        from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                               QoSHistoryPolicy, QoSDurabilityPolicy)
        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self._pubs[key] = self._node.create_publisher(Odometry, topic, qos)

    def publish_odom(self, key: str, frame_id: str, child_frame_id: str,
                     sim_time: float, pos, quat_wxyz, lin_vel, ang_vel) -> None:
        """Publish nav_msgs/Odometry stamped in SIM time.

        Covariance is left all-zero, which is what the robot publishes:
        inject_static_covariance is false on the driver. Filling in a plausible
        covariance here would be inventing an uncertainty the hardware does not
        report, and any consumer that weights by it would behave differently on
        the robot.
        """
        from nav_msgs.msg import Odometry
        pub = self._pubs.get(key)
        if pub is None:
            return
        m = Odometry()
        m.header.stamp.sec = int(sim_time)
        m.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        m.header.frame_id = frame_id
        m.child_frame_id = child_frame_id
        m.pose.pose.position.x = float(pos[0])
        m.pose.pose.position.y = float(pos[1])
        m.pose.pose.position.z = float(pos[2])
        m.pose.pose.orientation.w = float(quat_wxyz[0])
        m.pose.pose.orientation.x = float(quat_wxyz[1])
        m.pose.pose.orientation.y = float(quat_wxyz[2])
        m.pose.pose.orientation.z = float(quat_wxyz[3])
        m.twist.twist.linear.x = float(lin_vel[0])
        m.twist.twist.linear.y = float(lin_vel[1])
        m.twist.twist.linear.z = float(lin_vel[2])
        m.twist.twist.angular.x = float(ang_vel[0])
        m.twist.twist.angular.y = float(ang_vel[1])
        m.twist.twist.angular.z = float(ang_vel[2])
        pub.publish(m)

    def shutdown(self) -> None:
        self._stop = True
