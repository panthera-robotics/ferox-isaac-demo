#!/bin/bash
# ferox-isaac-demo — manual cmd_vel for sanity-driving the robot in sim.
# Publishes Twist on /ferox/$ROBOT_ID/cmd_vel; the Ferox sim bridge relays
# it to Isaac Sim's /cmd_vel.
#
# Usage:
#   ./03_teleop.sh forward   # 0.4 m/s
#   ./03_teleop.sh back      # -0.3 m/s
#   ./03_teleop.sh turn      # 0.5 rad/s in place
#   ./03_teleop.sh circle    # walk + turn
#   ./03_teleop.sh stop      # stop publisher → sim halts via cmd_vel timeout

set -e
source "$(dirname "$0")/lib/env.sh"

ACTION="${1:-forward}"
TOPIC="/ferox/$ROBOT_ID/cmd_vel"

case "$ACTION" in
  forward)  TWIST='{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {z: 0.0}}'; DESC="forward 0.4 m/s" ;;
  back)     TWIST='{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {z: 0.0}}'; DESC="back 0.3 m/s" ;;
  turn)     TWIST='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.5}}';  DESC="turn 0.5 rad/s" ;;
  circle)   TWIST='{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {z: 0.5}}';  DESC="circle (0.3 m/s + 0.5 rad/s)" ;;
  stop)     docker exec "$NAV_CONTAINER" pkill -f "topic pub.*$TOPIC" 2>/dev/null || true
            echo "Stopped publisher. Robot halts within ~1s (cmd_vel timeout)."
            exit 0 ;;
  *)        echo "Usage: $0 [forward|back|turn|circle|stop]"; exit 1 ;;
esac

# Kill any previous teleop publisher
docker exec "$NAV_CONTAINER" pkill -f "topic pub.*$TOPIC" 2>/dev/null || true
sleep 1

echo "Publishing $DESC on $TOPIC ..."
docker exec -d "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  ros2 topic pub $TOPIC geometry_msgs/Twist '$TWIST' -r 10
"

sleep 2
docker exec "$NAV_CONTAINER" pgrep -af "topic pub.*$TOPIC" | head -1
echo ""
echo "👀 Look at the sim viewport — robot is now $DESC."
echo "   ./03_teleop.sh stop  to halt."
