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

echo "Writing RViz config ($RVIZ_CFG) inside $NAV_CONTAINER ..."
docker exec "$NAV_CONTAINER" bash -lc "cat > $RVIZ_CFG <<'RVIZ'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
Visualization Manager:
  Class: ''
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map (SLAM)
      Topic:
        Value: ${NS}/map
        Durability Policy: Transient Local
        Reliability Policy: Reliable
      Color Scheme: map
      Alpha: 0.7
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Global Costmap
      Topic:
        Value: ${NS}/global_costmap/costmap
        Durability Policy: Transient Local
        Reliability Policy: Reliable
      Color Scheme: costmap
      Alpha: 0.5
    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Local Costmap
      Topic:
        Value: ${NS}/local_costmap/costmap
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color Scheme: costmap
      Alpha: 0.5
    - Class: rviz_default_plugins/Path
      Enabled: true
      Name: Global Plan
      Topic: { Value: ${NS}/plan }
      Color: 0; 255; 0
      Line Width: 0.05
    - Class: rviz_default_plugins/Path
      Enabled: true
      Name: Local Plan
      Topic: { Value: ${NS}/local_plan }
      Color: 0; 200; 255
    - Class: rviz_default_plugins/Pose
      Enabled: true
      Name: Goal Pose
      Topic: { Value: ${NS}/goal_pose }
      Color: 255; 25; 0
    - Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: LaserScan
      Topic: { Value: ${NS}/scan }
      Size (m): 0.05
      Color: 255; 0; 0
    - Class: rviz_default_plugins/Odometry
      Enabled: true
      Name: Odometry
      Topic: { Value: ${NS}/odom }
      Shape: Axes
      Axes Length: 0.4
      Keep: 100
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/SetGoal
      Topic: { Value: ${NS}/goal_pose }
    - Class: rviz_default_plugins/SetInitialPose
      Topic: { Value: ${NS}/initialpose }
RVIZ
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
