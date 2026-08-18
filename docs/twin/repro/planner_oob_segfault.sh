#!/bin/bash
# OQ-5.3 — nav2 planner_server segfault when the robot is outside the costmap.
#
# Sets up the observed condition and watches for the crash. Changes nothing in
# Ferox: it drives the robot to the edge of the SLAM map, sends a goal so the
# planner is asked to plan from an out-of-bounds pose, and reports what happens.
#
#   ./docs/twin/repro/planner_oob_segfault.sh [attempts]
#
# INTERMITTENT. Observed once during DT5; a later session logged the same
# "Robot is out of bounds of the costmap!" warning continuously for minutes with
# planner_server staying alive. So a clean run is NOT evidence the bug is absent --
# it is evidence this attempt did not hit it. Default 3 attempts.
set -u
ATTEMPTS="${1:-3}"
NAV="${NAV_CONTAINER:-ferox_nav}"
NS="${NS:-/ferox/go2_01}"

R() { docker exec "$NAV" bash -lc "source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  $1"; }

docker ps --format '{{.Names}}' | grep -q "^${NAV}$" || {
  echo "  x $NAV not running. Bring up the Go2 twin first:"
  echo "      ROBOT=go2 TWIN=1 SIM_WORLD=hospital ./scripts/01_start_sim.sh"
  echo "      ROBOT=go2 MODE=twin ./scripts/02_start_ferox.sh"; exit 1; }

echo "== baseline"
R "ros2 lifecycle get $NS/planner_server" 2>/dev/null | head -1
BEFORE=$(R 'grep -c "process has died.*planner_server" /tmp/nav.log' 2>/dev/null | tr -d '\r\n ')
echo "   planner deaths so far: ${BEFORE:-0}"

for i in $(seq 1 "$ATTEMPTS"); do
  echo ""
  echo "== attempt $i/$ATTEMPTS"

  # 1. Drive until the robot leaves the mapped region. The SLAM map grows behind
  #    the robot, so driving forward long enough puts the base outside it.
  echo "   driving to the map edge..."
  R "timeout 60 python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
rclpy.init(); n=Node('oob_drive')
q=QoSProfile(depth=10); q.reliability=ReliabilityPolicy.RELIABLE
pub=n.create_publisher(Twist,'$NS/cmd_vel',q)
od=[]; mp=[]
n.create_subscription(Odometry,'/odom',lambda m: od.append(m),q)
mq=QoSProfile(depth=1); mq.reliability=ReliabilityPolicy.RELIABLE
mq.durability=DurabilityPolicy.TRANSIENT_LOCAL
n.create_subscription(OccupancyGrid,'$NS/map',lambda m: mp.append(m),mq)
t=Twist(); t.linear.x=0.4
t0=time.time()
while time.time()-t0<18:
    pub.publish(t); rclpy.spin_once(n,timeout_sec=0.02); time.sleep(0.05)
pub.publish(Twist())
for _ in range(20): rclpy.spin_once(n,timeout_sec=0.05)
if od and mp:
    p=od[-1].pose.pose.position; i=mp[-1].info
    x0,y0=i.origin.position.x,i.origin.position.y
    x1,y1=x0+i.width*i.resolution, y0+i.height*i.resolution
    inside = x0<=p.x<=x1 and y0<=p.y<=y1
    print(f'   robot ({p.x:.2f},{p.y:.2f})  map ({x0:.2f},{y0:.2f})..({x1:.2f},{y1:.2f})  inside={inside}')
rclpy.shutdown()
PY" 2>/dev/null | grep robot

  # 2. Ask the planner to plan from there.
  echo "   sending a goal (planner must plan from an out-of-bounds pose)..."
  R "timeout 45 ros2 action send_goal $NS/navigate_to_pose \
      nav2_msgs/action/NavigateToPose \
      '{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'" \
      >/dev/null 2>&1

  AFTER=$(R 'grep -c "process has died.*planner_server" /tmp/nav.log' 2>/dev/null | tr -d '\r\n ')
  STATE=$(R "ros2 lifecycle get $NS/planner_server" 2>/dev/null | head -1)
  echo "   planner deaths: ${AFTER:-0}   state: ${STATE:-<no response>}"
  if [ "${AFTER:-0}" -gt "${BEFORE:-0}" ] 2>/dev/null; then
    echo ""
    echo "   *** REPRODUCED on attempt $i ***"
    R 'grep -B2 -A2 "process has died.*planner_server" /tmp/nav.log | tail -12'
    exit 0
  fi
done

echo ""
echo "== not reproduced in $ATTEMPTS attempts."
echo "   The out-of-bounds warning alone does not crash it -- see the issue text."
echo "   Evidence of the observed crash is in docs/twin/ISSUE_planner_segfault.md."
exit 2
