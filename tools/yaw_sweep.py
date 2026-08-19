import rclpy, math, time, json
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
def yaw_of(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
class S(Node):
    def __init__(s):
        super().__init__("yawsweep")
        s.pub=s.create_publisher(Twist,"/ferox/g1_01/cmd_vel",10)
        q=QoSProfile(depth=20,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
        s.create_subscription(Odometry,"/ferox/g1_01/odom",s.cb,q)
        s.create_subscription(Odometry,"/ferox/g1_01/odom",s.cb,20)
        s.last=None; s.seen=set()
    def cb(s,m):
        k=(m.header.stamp.sec,m.header.stamp.nanosec)
        if k in s.seen: return
        s.seen.add(k); s.last=m
    def spin(s,t):
        t0=time.time()
        while time.time()-t0<t: rclpy.spin_once(s,timeout_sec=0.05)
    def run(s,wz,dur=6.0):
        s.pub.publish(Twist()); s.spin(1.5)
        a=s.last; y0=yaw_of(a.pose.pose.orientation); t0=a.header.stamp.sec+a.header.stamp.nanosec*1e-9
        c=Twist(); c.angular.z=wz
        t=time.time()
        while time.time()-t<dur:
            s.pub.publish(c); rclpy.spin_once(s,timeout_sec=0.05)
        s.pub.publish(Twist()); s.spin(1.0)
        b=s.last; y1=yaw_of(b.pose.pose.orientation); t1=b.header.stamp.sec+b.header.stamp.nanosec*1e-9
        dy=math.atan2(math.sin(y1-y0),math.cos(y1-y0)); dt=max(1e-3,t1-t0)
        return {"cmd_wz":wz,"dyaw_rad":dy,"sim_dt":dt,"rate":dy/dt,"track_pct":100*abs(dy/dt)/abs(wz) if wz else 0}
rclpy.init(); n=S()
t=time.time()
while n.last is None and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.2)
if n.last is None: print("NO ODOM"); raise SystemExit(1)
rows=[]
for wz in (0.2,0.5,1.0,-1.0):
    r=n.run(wz); rows.append(r)
    print(f"cmd wz={wz:+.2f}  dyaw={r['dyaw_rad']:+.3f} rad over {r['sim_dt']:.2f}s sim  -> {r['rate']:+.4f} rad/s  ({r['track_pct']:.1f}% of commanded)",flush=True)
json.dump(rows,open("/tmp/yaw_sweep.json","w"),indent=2)
