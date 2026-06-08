#!/bin/bash
# ferox-isaac-demo — bring up Ferox nav and launch the stack in sim mode.
#
# Topic flow when mode:=sim:
#   Isaac Sim publishes  /scan /odom /imu  at root namespace.
#   isaac_bridge.launch.py runs topic_tools/relay to mirror them into
#       /ferox/$ROBOT_ID/{scan,odom,imu/data}  for Nav2 + SLAM.
#   Isaac Sim's cmd_vel listener subscribes DIRECTLY to
#       /ferox/$ROBOT_ID/cmd_vel  (set in 01_start_sim.sh via --cmd_vel_topic)
#   so Nav2's velocity_smoother output goes straight to sim — no relay,
#   no QoS war between transient_local/volatile publishers.
#   /tf and /tf_static stay global (multiple transient_local publishers
#   cannot be relayed; SLAM + Nav2 read them globally — see ferox_nav.launch).

set -e
source "$(dirname "$0")/lib/env.sh"
source "$(dirname "$0")/lib/kill_nav_stack.sh"

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

# Guard: sim's robot must match the nav stack's robot, or Nav2 publishes
# cmd_vel into the wrong namespace and the robot silently won't move.
# 01_start_sim.sh writes the sim's robot type to /tmp/sim_robot_type
# inside the sim container; we read it here. If the file is missing
# (sim started before this guard existed) the check skips silently —
# we never want a stale tag to block a valid run. Bypass entirely with
# FEROX_SKIP_SIM_CHECK=1 for multi-host setups where sim and nav run
# on separate machines.
if [ -z "${FEROX_SKIP_SIM_CHECK:-}" ]; then
  SIM_ROBOT=$(docker exec "$SIM_CONTAINER" cat /tmp/sim_robot_type 2>/dev/null | tr -d '[:space:]')
  if [ -n "$SIM_ROBOT" ] && [ "$SIM_ROBOT" != "$ROBOT" ]; then
    echo ""
    echo "  ERROR: Robot mismatch."
    echo "    Sim is running:  $SIM_ROBOT"
    echo "    Nav requested:   $ROBOT"
    echo "    Fix:  ROBOT=$SIM_ROBOT ./scripts/02_start_ferox.sh"
    echo "  Set FEROX_SKIP_SIM_CHECK=1 to bypass (multi-host setups only)."
    exit 1
  fi
fi

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

# Kill any prior nav stack so we don't end up with two sets of nodes
# fighting for the same DDS names. Killing only the `ros2 launch` parent
# leaves its children (waypoint_manager, the Nav2 nodes, the static TFs)
# running and orphaned to PID 1; a second launch then yields duplicate
# endpoints — e.g. two /ferox/<id>/move_to_named action servers — and the
# robot stops moving. kill_nav_stack (lib/kill_nav_stack.sh) is the single
# source of the match pattern and targets the node executables, so orphans
# from a prior launch are caught too.
if ! kill_nav_stack "$NAV_CONTAINER" --verify; then
  echo "    Brute-clean and retry:  docker exec $NAV_CONTAINER pkill -9 -f ros2"
  exit 1
fi

# Truncate any stale log first
docker exec "$NAV_CONTAINER" bash -lc ': > /tmp/nav.log'

# Launch in detached mode. `exec > /tmp/nav.log 2>&1` at the top of the
# bash command captures EVERY subsequent command's output (sources, launch,
# any errors) — without that line, only the final ros2 launch line writes
# to the log, so source-step failures used to vanish silently and we'd see
# an empty /tmp/nav.log with no clue what went wrong.
docker exec -d "$NAV_CONTAINER" bash -lc "
  exec > /tmp/nav.log 2>&1
  set -eo pipefail
  echo \"[run_nav] \$(date -Iseconds) starting nav stack\"
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  echo \"[run_nav] env sourced; ROBOT=$ROBOT ROBOT_ID=$ROBOT_ID VENUE_ARG=$VENUE_ARG\"
  exec ros2 launch ferox_nav_bringup bringup.launch.py \
    robot:=$ROBOT mode:=sim robot_id:=$ROBOT_ID $VENUE_ARG
"

# Poll the launch log for the lifecycle_manager's "Managed nodes are active"
# marker. This is the deterministic green light that all 7 lifecycle nodes
# transitioned to active. Polling the log file avoids DDS daemon cache races
# that made the previous `ros2 lifecycle get` poll unreliable on cold boot.
echo "  Waiting for Nav2 to become active (timeout 60s)..."
ACTIVE=0
for i in $(seq 1 60); do
  sleep 1
  if docker exec "$NAV_CONTAINER" bash -lc 'grep -q "Managed nodes are active" /tmp/nav.log 2>/dev/null'; then
    ACTIVE=1
    echo "  ✓ Nav2 fully active after ${i}s"
    break
  fi
  # Fast-fail if the launch process died early — nav.log will contain the traceback.
  if ! docker exec "$NAV_CONTAINER" bash -lc 'pgrep -f "ros2 launch ferox_nav_bringup" >/dev/null'; then
    echo "  ✗ ros2 launch exited unexpectedly. Tail of /tmp/nav.log:"
    docker exec "$NAV_CONTAINER" bash -lc 'tail -30 /tmp/nav.log'
    exit 1
  fi
done

if [ "$ACTIVE" -ne 1 ]; then
  echo "  ⚠ Nav2 not fully active after 60s. Tail of /tmp/nav.log:"
  docker exec "$NAV_CONTAINER" bash -lc 'tail -30 /tmp/nav.log'
fi

# Common gotcha: missing topic_tools in the nav image kills the sim bridge
# silently. Surface it explicitly.
if docker exec "$NAV_CONTAINER" bash -lc 'grep -q "package .topic_tools. not found" /tmp/nav.log 2>/dev/null'; then
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
