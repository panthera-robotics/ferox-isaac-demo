"""Dump real twin/bag data to .npz for the montage plots (nav container).

Plots are drawn on the host with PIL, because matplotlib lives in the Isaac image and
the data lives here. Dumping arrays keeps the two apart and means a plot can be
re-rendered without re-running the sim.

  MODE=live|bag  OUT=/tmp/mdata  BAG=/tmp/gt  python3 capture_data.py
"""
from __future__ import annotations
import os, time
import numpy as np

MODE = os.environ.get("MODE", "live")
OUT = os.environ.get("OUT", "/tmp/mdata")
NS = os.environ.get("TWIN_NS", "/ferox/g1_01")
SCAN = os.environ.get("SCAN_TOPIC", f"{NS}/scan")
CLOUD = os.environ.get("CLOUD_TOPIC", "/livox/lidar")
N_SCAN = int(os.environ.get("N_SCAN", "40"))
N_CLOUD = int(os.environ.get("N_CLOUD", "12"))
os.makedirs(OUT, exist_ok=True)


def save(name, **kw):
    np.savez_compressed(os.path.join(OUT, name), **kw)
    print(f"  wrote {name}: " + ", ".join(f"{k}{np.shape(v)}" for k, v in kw.items()))


if MODE == "bag":
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import sensor_msgs_py.point_cloud2 as pc2
    bag = os.environ.get("BAG", "/tmp/gt")
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    scans, clouds, meta = [], [], {}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic == SCAN and len(scans) < N_SCAN:
            m = deserialize_message(data, get_message(types[topic]))
            scans.append(np.asarray(m.ranges, dtype=np.float32))
            meta = dict(angle_min=float(m.angle_min), angle_max=float(m.angle_max),
                        angle_increment=float(m.angle_increment),
                        range_min=float(m.range_min), range_max=float(m.range_max))
        elif topic == CLOUD and len(clouds) < N_CLOUD:
            m = deserialize_message(data, get_message(types[topic]))
            a = np.array([[p[0], p[1], p[2]] for p in
                          pc2.read_points(m, field_names=("x", "y", "z"))],
                         dtype=np.float32)
            a = a[np.linalg.norm(a, axis=1) > 0]          # drop C-18 zero padding
            clouds.append(a[::3])
        if len(scans) >= N_SCAN and len(clouds) >= N_CLOUD:
            break
    save("scan_real.npz", scans=np.stack(scans), **meta)
    if clouds:
        save("cloud_real.npz", **{f"c{i}": c for i, c in enumerate(clouds)})
else:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from nav_msgs.msg import OccupancyGrid, Path, Odometry
    import sensor_msgs_py.point_cloud2 as pc2
    from tf2_ros import Buffer, TransformListener

    rclpy.init(); n = Node("mdata")
    qb = QoSProfile(depth=10); qb.reliability = ReliabilityPolicy.BEST_EFFORT
    qr = QoSProfile(depth=10); qr.reliability = ReliabilityPolicy.RELIABLE
    ql = QoSProfile(depth=1); ql.reliability = ReliabilityPolicy.RELIABLE
    ql.durability = DurabilityPolicy.TRANSIENT_LOCAL
    scans, clouds, maps, paths, odoms = [], [], [], [], []
    n.create_subscription(LaserScan, SCAN, lambda m: scans.append(m), qb)
    n.create_subscription(PointCloud2, CLOUD, lambda m: clouds.append(m), qr)
    n.create_subscription(OccupancyGrid, f"{NS}/map", lambda m: maps.append(m), ql)
    n.create_subscription(Path, f"{NS}/plan", lambda m: paths.append(m), qr)
    n.create_subscription(Odometry, f"{NS}/odom", lambda m: odoms.append(m), qr)
    buf = Buffer(); TransformListener(buf, n)
    t0 = time.time()
    dur = float(os.environ.get("DURATION", "35"))
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.2)

    if scans:
        m = scans[-1]
        save("scan_sim.npz", scans=np.stack([np.asarray(s.ranges, dtype=np.float32)
                                            for s in scans[:N_SCAN]]),
             angle_min=float(m.angle_min), angle_max=float(m.angle_max),
             angle_increment=float(m.angle_increment),
             range_min=float(m.range_min), range_max=float(m.range_max))
    if clouds:
        tr = None
        try:
            tr = buf.lookup_transform("base_link", clouds[-1].header.frame_id,
                                      rclpy.time.Time())
        except Exception:
            pass
        out = {}
        for i, cm in enumerate(clouds[:N_CLOUD]):
            a = np.array([[p[0], p[1], p[2]] for p in
                          pc2.read_points(cm, field_names=("x", "y", "z"),
                                          skip_nans=True)], dtype=np.float32)
            a = a[np.linalg.norm(a, axis=1) > 0]
            if tr is not None:
                q = tr.transform.rotation; t = tr.transform.translation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                a = (R @ a.T).T + np.array([t.x, t.y, t.z])
            out[f"c{i}"] = a[::2]
        save("cloud_sim.npz", **out)
    if maps:
        mm = maps[-1]
        save("map.npz", grid=np.asarray(mm.data, dtype=np.int8).reshape(
                mm.info.height, mm.info.width),
             res=mm.info.resolution, ox=mm.info.origin.position.x,
             oy=mm.info.origin.position.y)
    if paths and paths[-1].poses:
        p = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in paths[-1].poses],
                     dtype=np.float32)
        save("path.npz", path=p)
    if odoms:
        save("odom.npz", z=np.array([o.pose.pose.position.z for o in odoms], dtype=np.float32),
             x=np.array([o.pose.pose.position.x for o in odoms], dtype=np.float32),
             y=np.array([o.pose.pose.position.y for o in odoms], dtype=np.float32))
    rclpy.shutdown()
print("DONE")
