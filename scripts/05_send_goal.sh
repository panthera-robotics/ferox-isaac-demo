#!/bin/bash
# ferox-isaac-demo — send a Nav2 goal to /ferox/$ROBOT_ID/navigate_to_pose.
#
# Usage:
#   ./05_send_goal.sh                    # default: x=2, y=0, yaw=0
#   ./05_send_goal.sh 5 0                # go to (5, 0)
#   ./05_send_goal.sh 3 -2 1.57          # go to (3, -2) facing 90°
#   ./05_send_goal.sh waypoint reception # named waypoint via Ferox service
#   ./05_send_goal.sh cancel             # cancel current goal
#   ./05_send_goal.sh status             # diagnostic of nav stack

set -e
source "$(dirname "$0")/lib/env.sh"

NS="/ferox/$ROBOT_ID"
ACTION="${1:-default}"

run_in_nav() {
  docker exec "$NAV_CONTAINER" bash -lc "
    source /opt/ros/humble/setup.bash
    source /opt/ferox_msgs_ws/install/setup.bash
    source /workspace/install/setup.bash
    $1
  "
}

case "$ACTION" in
  cancel)
    echo "Cancelling goal on $NS/navigate_to_pose ..."
    # Easiest cancel-all: invoke the goal client and Ctrl-C, but we can also
    # call the cancel service exposed by Nav2's bt_navigator.
    run_in_nav "ros2 action send_goal $NS/navigate_to_pose nav2_msgs/action/NavigateToPose \
      '{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}'" \
      &
    GOAL_PID=$!
    sleep 1; kill $GOAL_PID 2>/dev/null || true
    echo "Sent origin goal as a soft-stop."
    exit 0
    ;;

  status)
    echo "==============================================="
    echo " Ferox nav status (NS=$NS)"
    echo "==============================================="
    echo ""
    echo "[Containers]"
    for c in "$SIM_CONTAINER" "$NAV_CONTAINER"; do
      state=$(docker ps --filter "name=^${c}$" --format '{{.Status}}' 2>/dev/null)
      [ -z "$state" ] && state="NOT RUNNING"
      printf "  %-22s : %s\n" "$c" "$state"
    done
    echo ""
    echo "[Topic flow rates under $NS]"
    run_in_nav "for t in scan odom map plan cmd_vel global_costmap/costmap local_costmap/costmap; do
      rate=\$(timeout 3 ros2 topic hz $NS/\$t 2>&1 | grep 'average rate' | head -1 | sed 's/average rate: //')
      [ -z \"\$rate\" ] && rate='(silent)'
      printf '  %-32s : %s\n' \"$NS/\$t\" \"\$rate\"
    done" 2>&1 | grep -vE '^\[WARN\]|^WARNING:'
    echo ""
    echo "[Nav2 lifecycle states]"
    run_in_nav "for n in controller_server planner_server bt_navigator behavior_server smoother_server velocity_smoother; do
      state=\$(timeout 3 ros2 lifecycle get $NS/\$n 2>&1 | head -1)
      printf '  %-30s : %s\n' \"$NS/\$n\" \"\$state\"
    done" 2>&1 | grep -vE '^\[WARN\]|^WARNING:'
    exit 0
    ;;

  waypoint)
    NAME="${2:?usage: 05_send_goal.sh waypoint <name>}"
    echo "Calling $NS/go_to_waypoint name=$NAME ..."
    run_in_nav "ros2 service call $NS/go_to_waypoint ferox_msgs/srv/GoToWaypoint \
      '{name: $NAME, precise_pose: true}'"
    exit 0
    ;;

  help|--help|-h)
    sed -n '2,11p' "$0"
    exit 0
    ;;
esac

# ---- Geometric goal mode ----
GOAL_X="${1:-2.0}"
GOAL_Y="${2:-0.0}"
GOAL_YAW="${3:-0.0}"

[[ "$GOAL_X" =~ ^-?[0-9]+\.?[0-9]*$ ]] || { echo "ERROR: '$GOAL_X' not numeric"; exit 1; }

QZ=$(python3 -c "import math; print(math.sin($GOAL_YAW * 0.5))")
QW=$(python3 -c "import math; print(math.cos($GOAL_YAW * 0.5))")

echo "==============================================="
echo " Goal: x=$GOAL_X y=$GOAL_Y yaw=$GOAL_YAW rad → $NS/navigate_to_pose"
echo "==============================================="
echo ""

# Pre-flight
docker ps --format '{{.Names}}' | grep -q "^${NAV_CONTAINER}$" || {
  echo "  ✗ $NAV_CONTAINER not running. Run ./02_start_ferox.sh first."; exit 1; }

# Clear costmaps so we don't replan around stale obstacles
run_in_nav "
  ros2 service call $NS/global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap '{}' >/dev/null 2>&1 || true
  ros2 service call $NS/local_costmap/clear_entirely_local_costmap   nav2_msgs/srv/ClearEntireCostmap '{}' >/dev/null 2>&1 || true
" || true

# Send via the action interface (gives feedback). Ctrl-C to abort.
run_in_nav "ros2 action send_goal $NS/navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0, y: 0, z: $QZ, w: $QW}}}}' \
  --feedback"
