#!/bin/bash
# ferox-isaac-demo — open RViz showing the Ferox /ferox/$ROBOT_ID/* namespace.
#
# Displays:
#   - Map (SLAM)
#   - Global + Local costmaps
#   - Global + Local plans
#   - LaserScan, Odometry, TF
#   - 2D Goal Pose tool (publishes to navigate_to_pose under namespace)

set -e
source "$(dirname "$0")/lib/env.sh"

NS="/ferox/$ROBOT_ID"
RVIZ_CFG="/tmp/ferox_${ROBOT_ID}.rviz"

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
# RViz bakes ABSOLUTE topic names, so the saved file is pinned to whatever
# robot_id it was captured with (e.g. /ferox/go2_01/...). Rewrite that
# prefix to the current $ROBOT_ID on the fly so one file serves any robot.
SRC_RVIZ="/workspace/src/ferox_nav/config/rviz/ferox_nav.rviz"

echo "Loading RViz config from $SRC_RVIZ (namespace → $NS) ..."
docker exec "$NAV_CONTAINER" bash -lc "
  set -e
  test -f '$SRC_RVIZ' || { echo '  ✗ RViz config not found: $SRC_RVIZ — save it from RViz first.'; exit 1; }
  sed -E 's#/ferox/[A-Za-z0-9_]+/#${NS}/#g' '$SRC_RVIZ' > '$RVIZ_CFG'
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
  rviz2 -d $RVIZ_CFG > /tmp/rviz.log 2>&1
"
sleep 5
docker exec "$NAV_CONTAINER" pgrep -af rviz2 >/dev/null \
  && echo "✓ rviz2 running — switch to your VNC desktop." \
  || echo "⚠ rviz2 didn't come up. Check: docker exec $NAV_CONTAINER cat /tmp/rviz.log"

echo ""
echo "Send a goal by mouse: '2D Goal Pose' tool in the toolbar, click on the map."
echo "Or via CLI:           ./scripts/05_send_goal.sh 2 0"
