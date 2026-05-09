#!/bin/bash
# ferox-isaac-demo — bring up Ferox nav and launch the stack in sim mode.
#
# Inside the Ferox container, 'mode:=sim' triggers the Isaac bridge
# (ferox_nav_sim/launch/isaac_bridge.launch.py), which relays:
#   /scan, /odom, /imu (from Isaac Sim) → /ferox/$ROBOT_ID/...
#   /ferox/$ROBOT_ID/cmd_vel             → /cmd_vel  (consumed by sim)

set -e
source "$(dirname "$0")/lib/env.sh"

echo "==============================================="
echo " ferox-isaac-demo — start Ferox nav"
echo "==============================================="
echo ""

# ---- [1/4] Pre-flight: sim must be up ----
echo "[1/4] Pre-flight..."
if ! docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$"; then
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."
  exit 1
fi
[ -d "$FEROX_REPO" ] || { echo "  ✗ FEROX_REPO=$FEROX_REPO not found."; exit 1; }
echo "  ✓ sim container up; Ferox repo at $FEROX_REPO"

# ---- [2/4] Ferox compose up ----
echo ""
echo "[2/4] Bringing up $NAV_CONTAINER (image $FEROX_NAV_IMAGE)..."
( cd "$FEROX_REPO" && [ -f .env ] || cp .env.example .env )

# Push our identity into the .env so docker-compose picks it up
sed -i \
  -e "s/^ROBOT_ID=.*/ROBOT_ID=$ROBOT_ID/" \
  -e "s/^ROBOT_TYPE=.*/ROBOT_TYPE=$ROBOT/" \
  -e "s/^FEROX_MODE=.*/FEROX_MODE=sim/" \
  -e "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" \
  "$FEROX_REPO/.env"

( cd "$FEROX_REPO" && \
    docker compose -f docker/docker-compose.yml up -d nav )
sleep 2
echo "  ✓ $NAV_CONTAINER up"

# ---- [3/4] colcon build (idempotent, fast on warm install/) ----
echo ""
echo "[3/4] Building the colcon workspace inside $NAV_CONTAINER..."
docker exec "$NAV_CONTAINER" bash -lc '/workspace/scripts/build.sh' \
  | tail -10

# ---- [4/4] Launch nav stack with sim bridge ----
echo ""
echo "[4/4] Launching ferox_nav_bringup (robot=$ROBOT, robot_id=$ROBOT_ID, mode=sim)..."

# Build optional venue arg only when non-empty (ros2 launch rejects 'name:=')
VENUE_ARG=""
[ -n "$VENUE" ] && VENUE_ARG="venue:=$VENUE"

# Kill any prior launch in the container so we don't end up with two stacks
docker exec "$NAV_CONTAINER" bash -lc 'pkill -9 -f "ros2 launch" 2>/dev/null || true'
sleep 1

docker exec -d "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  ros2 launch ferox_nav_bringup bringup.launch.py \
    robot:=$ROBOT mode:=sim robot_id:=$ROBOT_ID $VENUE_ARG \
    > /tmp/nav.log 2>&1
"

echo "  Waiting for Nav2 to become active (timeout 60s)..."
ACTIVE=0
for i in $(seq 1 60); do
  sleep 1
  ACTIVE=$(docker exec "$NAV_CONTAINER" bash -lc \
    'source /opt/ros/humble/setup.bash 2>/dev/null
     source /workspace/install/setup.bash 2>/dev/null
     n=0
     for s in controller_server planner_server bt_navigator behavior_server smoother_server velocity_smoother waypoint_follower; do
       st=$(timeout 1 ros2 lifecycle get /ferox/'"$ROBOT_ID"'/$s 2>/dev/null | head -1)
       [ "$st" = "active [3]" ] && n=$((n+1))
     done
     echo $n' 2>/dev/null)
  ACTIVE=${ACTIVE:-0}
  if [ "$ACTIVE" -ge 7 ]; then
    echo "  ✓ Nav2 fully active after ${i}s (7/7 servers)"; break
  fi
done

if [ "$ACTIVE" -lt 7 ]; then
  echo "  ⚠ Nav2 not fully active after 60s ($ACTIVE/7 servers active)"
  echo "    Inspect: docker exec $NAV_CONTAINER tail -80 /tmp/nav.log"
fi

# Common gotcha: missing topic_tools in the nav image kills the sim bridge
# silently. Surface it explicitly.
if docker exec "$NAV_CONTAINER" bash -lc 'grep -q "package .topic_tools. not found" /tmp/nav.log'; then
  echo ""
  echo "  ✗ topic_tools missing from ferox/nav image — sim bridge can't start."
  echo "    Rebuild: cd $FEROX_REPO && docker compose -f docker/docker-compose.yml build nav"
fi

echo ""
echo "==============================================="
echo " Ferox nav started."
echo "==============================================="
echo ""
echo " Verify topics in the /ferox/$ROBOT_ID/ namespace:"
echo "   docker exec $NAV_CONTAINER bash -lc \\"
echo "     'source /workspace/install/setup.bash && ros2 topic list | grep ferox/$ROBOT_ID'"
echo ""
echo " Send a goal:    ./scripts/05_send_goal.sh 2 0"
echo " Manual drive:   ./scripts/03_teleop.sh"
echo " RViz:           ./scripts/04_view_rviz.sh"
echo ""
