#!/bin/bash
# ferox-isaac-demo — open RViz showing the Ferox /ferox/$ROBOT_ID/* namespace.
#
# Displays:
#   - Map (SLAM)
#   - Global + Local costmaps
#   - Global + Local plans
#   - LaserScan, Odometry, TF, RobotModel
#   - 2D Pose Estimate (→ initialpose) + Nav2 Goal (→ goal_pose) tools

set -e
source "$(dirname "$0")/lib/env.sh"

NS="/ferox/$ROBOT_ID"

# Pre-flight
docker ps --format '{{.Names}}' | grep -q "^${NAV_CONTAINER}$" || {
  echo "  ✗ $NAV_CONTAINER not running. Run ./02_start_ferox.sh first."; exit 1; }

# X
sudo -u "$DESKTOP_USER" DISPLAY="$HOST_DISPLAY" XAUTHORITY="$XAUTH_FILE" \
  xhost +local: >/dev/null 2>&1 || true

# Use the committed RViz config (config/rviz/ferox_nav.rviz) as the single
# source of truth. The repo is bind-mounted at /workspace, so edits saved
# from RViz land here directly — no rebuild needed.
#
# The config uses RELATIVE topic names (map, scan, global_costmap/costmap, ...);
# we run rviz2 under __ns:=/ferox/$ROBOT_ID below so they resolve to that
# robot's namespace — one file serves any robot, the same mechanism as the
# launch's PushRosNamespace. Frames are plain (map/odom/base_link).
SRC_RVIZ="/workspace/src/ferox_nav/config/rviz/ferox_nav.rviz"

echo "Loading RViz config from $SRC_RVIZ (namespace → $NS) ..."
docker exec "$NAV_CONTAINER" bash -lc "
  test -f '$SRC_RVIZ' || { echo '  ✗ RViz config not found: $SRC_RVIZ — rebuild ferox_nav (config/rviz install rule).'; exit 1; }
"

# Kill prior rviz, launch new
docker exec "$NAV_CONTAINER" pkill -f rviz2 2>/dev/null || true
sleep 1

docker exec -d "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  export DISPLAY=$HOST_DISPLAY
  export XAUTHORITY=/tmp/.docker.xauth
  rviz2 -d $SRC_RVIZ --ros-args -r __ns:=$NS > /tmp/rviz.log 2>&1
"
sleep 5
docker exec "$NAV_CONTAINER" pgrep -af rviz2 >/dev/null \
  && echo "✓ rviz2 running — switch to your VNC desktop." \
  || echo "⚠ rviz2 didn't come up. Check: docker exec $NAV_CONTAINER cat /tmp/rviz.log"

echo ""
echo "Send a goal by mouse: '2D Goal Pose' tool in the toolbar, click on the map."
echo "Or via CLI:           ./scripts/05_send_goal.sh 2 0"
