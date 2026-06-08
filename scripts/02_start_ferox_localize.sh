#!/bin/bash
# ferox-isaac-demo — bring up Ferox nav in AMCL (localization) mode.
#
# The production-mode counterpart to 02_start_ferox.sh. That script always
# launches SLAM Toolbox (use_slam:=true) and maps the space from scratch;
# this one flips use_slam:=false and localizes against a venue's already-
# saved map with map_server + AMCL — the nav path used once a venue has
# been mapped.
#
# Concretely, what differs from SLAM mode:
#   SLAM mode  — slam_toolbox self-manages; ONE lifecycle manager
#                (lifecycle_manager_navigation).
#   AMCL mode  — map_server + amcl come up under a SECOND manager
#                (lifecycle_manager_localization). We wait for BOTH.
#
# Prerequisite: the ferox_nav container must already be running with the
# colcon workspace built — i.e. 02_start_ferox.sh has run at least once, or
# `docker compose -f docker/docker-compose.yml up -d nav` + scripts/build.sh.
# This script only (re)launches the nav stack; it does not compose-up or
# build.
#
# Topic flow matches 02_start_ferox.sh's sim mode. The localization nodes
# add, under /ferox/<robot_id>/:
#   map             latched OccupancyGrid served by map_server
#   amcl_pose       AMCL's pose estimate
#   particle_cloud  AMCL's particle filter cloud
#
# Usage:
#   ./scripts/02_start_ferox_localize.sh                 # venue dso_block_a
#   VENUE=<id> ./scripts/02_start_ferox_localize.sh      # a different venue

set -e
source "$(dirname "$0")/lib/env.sh"
source "$(dirname "$0")/lib/kill_nav_stack.sh"

# lib/env.sh leaves VENUE empty — that's the SLAM-mapping default that
# 02_start_ferox.sh wants. Localization is meaningless without a map, so
# default to the committed demo venue here instead.
VENUE="${VENUE:-dso_block_a}"

# In-container path to the venue's installed map metadata YAML. This is the
# package-share copy a colcon build produces; the launch's _load_venue_config
# walks venue -> map the same way.
MAP_YAML="/workspace/install/ferox_nav/share/ferox_nav/maps/${VENUE}.yaml"

echo "==============================================="
echo " ferox-isaac-demo — start Ferox nav (AMCL localize)"
echo "==============================================="
echo "   venue=$VENUE  robot=$ROBOT  robot_id=$ROBOT_ID"
echo ""

# ---- [1/4] Pre-flight ----
echo "[1/4] Pre-flight..."

# FEROX_REPO is resolved in lib/env.sh (flat layout first, nested fallback).
[ -d "$FEROX_REPO" ] || { echo "  ✗ FEROX_REPO=$FEROX_REPO not found."; exit 1; }

# This script relaunches the stack inside an existing nav container — it
# does not create one. Bringing the container up (and the colcon build) is
# 02_start_ferox.sh's job.
if ! docker ps --format '{{.Names}}' | grep -q "^${NAV_CONTAINER}$"; then
  echo "  ✗ $NAV_CONTAINER is not running."
  echo "    Bring the nav container up first:"
  echo "      ./scripts/02_start_ferox.sh"
  echo "      (or: cd $FEROX_REPO && docker compose -f docker/docker-compose.yml up -d nav)"
  exit 1
fi

# Robot mismatch guard — identical to 02_start_ferox.sh. If the sim runs a
# different robot than the nav stack, Nav2 publishes cmd_vel into the wrong
# namespace and the robot silently won't move. /tmp/sim_robot_type is written
# by 01_start_sim.sh inside the sim container; a missing file (or a stopped
# sim) skips the check rather than blocking a valid run. Bypass with
# FEROX_SKIP_SIM_CHECK=1 for multi-host setups.
if [ -z "${FEROX_SKIP_SIM_CHECK:-}" ]; then
  SIM_ROBOT=$(docker exec "$SIM_CONTAINER" cat /tmp/sim_robot_type 2>/dev/null | tr -d '[:space:]')
  if [ -n "$SIM_ROBOT" ] && [ "$SIM_ROBOT" != "$ROBOT" ]; then
    echo ""
    echo "  ERROR: Robot mismatch."
    echo "    Sim is running:  $SIM_ROBOT"
    echo "    Nav requested:   $ROBOT"
    echo "    Fix:  ROBOT=$SIM_ROBOT ./scripts/02_start_ferox_localize.sh"
    echo "  Set FEROX_SKIP_SIM_CHECK=1 to bypass (multi-host setups only)."
    exit 1
  fi
fi

# The venue's map must be installed in the package share. Missing almost
# always means the colcon workspace wasn't rebuilt after M3.1 landed (the
# build that installs maps/ into install/ferox_nav/share). Fail loud now —
# the launch's own venue resolver would otherwise fail later, buried in
# /tmp/nav.log.
if ! docker exec "$NAV_CONTAINER" test -f "$MAP_YAML"; then
  echo "  ✗ Venue map not found inside $NAV_CONTAINER:"
  echo "      $MAP_YAML"
  echo "    Likely the colcon workspace wasn't built after pulling M3.1."
  echo "    Fix:  docker exec $NAV_CONTAINER bash -lc '/workspace/scripts/build.sh'"
  exit 1
fi
echo "  ✓ $NAV_CONTAINER up; venue map present"

# ---- [2/4] Clear residual nav processes ----
echo ""
echo "[2/4] Clearing residual nav processes in $NAV_CONTAINER..."

# A prior nav stack must be gone before we relaunch, or two sets of nodes
# fight over the same DDS names. The trap this script fell into: it killed
# only the Nav2 nodes + the launch parent and MISSED the ferox_nav package
# nodes (waypoint_manager, status_publisher, venue_manager) and the static
# TFs — so those survived, orphaned to PID 1, and the next launch added a
# SECOND waypoint_manager. Two /ferox/<id>/move_to_named action servers then
# cross the speech client's goal/result responses and the robot never moves.
#
# kill_nav_stack (lib/kill_nav_stack.sh) is the single source of the match
# pattern, covering the node executables — not just the launch parent — so
# orphans can't slip through. It is a bare `docker exec ... pkill` (never a
# wrapping shell) so it cannot kill the process that runs it.
if ! kill_nav_stack "$NAV_CONTAINER" --verify; then
  echo "    Brute-clean and retry:  docker exec $NAV_CONTAINER pkill -9 -f ros2"
  exit 1
fi
echo "  ✓ no residual nav processes"

# ---- [3/4] Launch the localization stack ----
echo ""
echo "[3/4] Launching ferox_nav_bringup (use_slam:=false venue:=$VENUE)..."

# Fresh log so the lifecycle-marker poll below only counts THIS launch.
docker exec "$NAV_CONTAINER" bash -lc ': > /tmp/nav.log'

# Detached launch. `exec > /tmp/nav.log 2>&1` at the top captures every
# following line — env sourcing, the launch, any traceback — so a failure
# in a source step can't vanish into an empty log (same reasoning as
# 02_start_ferox.sh).
docker exec -d "$NAV_CONTAINER" bash -lc "
  exec > /tmp/nav.log 2>&1
  set -eo pipefail
  echo \"[run_nav] \$(date -Iseconds) starting nav stack (AMCL localize)\"
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash
  source /workspace/install/setup.bash
  echo \"[run_nav] env sourced; ROBOT=$ROBOT ROBOT_ID=$ROBOT_ID VENUE=$VENUE use_slam=false\"
  exec ros2 launch ferox_nav_bringup bringup.launch.py \
    use_slam:=false venue:=$VENUE robot_id:=$ROBOT_ID robot:=$ROBOT mode:=sim
"
echo "  launch dispatched"

# ---- [4/4] Wait for both lifecycle managers, then verify structure ----
echo ""
echo "[4/4] Waiting for both lifecycle managers to report active (timeout 30s)..."

# AMCL mode runs TWO lifecycle managers — navigation (controller, planner,
# bt_navigator, ...) and localization (map_server + amcl) — and each logs
# "Managed nodes are active" once its set is up. So we wait for the marker
# to appear TWICE. SLAM mode has only the navigation manager, which is why
# 02_start_ferox.sh waits for a single occurrence.
#
# Poll the log every 0.5s, capped at 30s. Reading the log (vs `ros2
# lifecycle get`) sidesteps the DDS daemon cache races that make the CLI
# flaky right after a cold launch.
SECONDS=0
ACTIVE=0
for i in $(seq 1 60); do
  sleep 0.5
  COUNT=$(docker exec "$NAV_CONTAINER" \
    grep -c "Managed nodes are active" /tmp/nav.log 2>/dev/null || true)
  if [ "${COUNT:-0}" -ge 2 ]; then
    ACTIVE=1
    echo "  ✓ both lifecycle managers active after ${SECONDS}s"
    break
  fi
  # Fast-fail: if the launch process is gone, the log holds the traceback.
  if ! docker exec "$NAV_CONTAINER" pgrep -f "ros2 launch ferox_nav_bringup" >/dev/null 2>&1; then
    echo "  ✗ ros2 launch exited early. Last 40 lines of /tmp/nav.log:"
    docker exec "$NAV_CONTAINER" tail -40 /tmp/nav.log 2>/dev/null | sed 's/^/      /'
    exit 1
  fi
done

if [ "$ACTIVE" -ne 1 ]; then
  echo "  ✗ both lifecycle managers not active within 30s. Last 40 lines of /tmp/nav.log:"
  docker exec "$NAV_CONTAINER" tail -40 /tmp/nav.log 2>/dev/null | sed 's/^/      /'
  exit 1
fi

# Structural confirmation: map_server and amcl must be registered lifecycle
# nodes. Both sit under /ferox/$ROBOT_ID/, so the names appear with that
# namespace prefix — grep accordingly. Retry a few times: `ros2 lifecycle
# nodes` can lag discovery for a second or two even once the nodes are up.
echo "  Confirming map_server + amcl are registered lifecycle nodes..."
LC_OK=0
for _ in $(seq 1 8); do
  LC_NODES=$(docker exec "$NAV_CONTAINER" bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    ros2 lifecycle nodes 2>/dev/null
  ' 2>/dev/null || true)
  if echo "$LC_NODES" | grep -q 'map_server' && echo "$LC_NODES" | grep -q 'amcl'; then
    LC_OK=1
    break
  fi
  sleep 1.5
done
if [ "$LC_OK" -ne 1 ]; then
  echo "  ✗ map_server / amcl not found in 'ros2 lifecycle nodes'."
  echo "    Last output:"
  echo "$LC_NODES" | sed 's/^/      /'
  exit 1
fi
echo "  ✓ map_server + amcl registered as lifecycle nodes"

# ---- Summary ----
# Map metadata for the summary, read from the venue's installed map YAML
# (the symlink resolves inside the container). resolution + origin come
# straight from the YAML; pixel WxH is a best-effort read of the PGM header.
# None of this is load-bearing — the launch already succeeded.
MAP_TXT=$(docker exec "$NAV_CONTAINER" cat "$MAP_YAML" 2>/dev/null || true)
MAP_RES=$(printf '%s\n' "$MAP_TXT" | sed -n 's/^resolution:[[:space:]]*//p')
MAP_ORIGIN=$(printf '%s\n' "$MAP_TXT" | sed -n 's/^origin:[[:space:]]*//p')
MAP_IMG=$(printf '%s\n' "$MAP_TXT" | sed -n 's/^image:[[:space:]]*//p')

MAP_PX=""
if [ -n "$MAP_IMG" ]; then
  # PGM header is "P5 / optional #comment / W H / maxval" — print the W H line.
  MAP_PX=$(docker exec "$NAV_CONTAINER" \
    awk '/^P5/||/^#/{next}{print $1"x"$2; exit}' \
    "$(dirname "$MAP_YAML")/$MAP_IMG" 2>/dev/null || true)
  [[ "$MAP_PX" =~ ^[0-9]+x[0-9]+$ ]] || MAP_PX=""
fi

LAUNCH_PID=$(docker exec "$NAV_CONTAINER" \
  pgrep -f "ros2 launch ferox_nav_bringup" 2>/dev/null | head -1 || true)

echo ""
echo "==============================================="
echo " Ferox nav started — AMCL / localization mode."
echo "==============================================="
echo ""
printf '  %-11s : %s\n' "launch PID" "${LAUNCH_PID:-?}  (in $NAV_CONTAINER)"
printf '  %-11s : %s\n' "launch log" "$NAV_CONTAINER:/tmp/nav.log"
printf '  %-11s : %s\n' "lifecycle"  "navigation + localization — both managers active"
printf '  %-11s : %s\n' "venue"      "$VENUE"
printf '  %-11s : %s\n' "map"        "${MAP_IMG:-?}${MAP_PX:+  $MAP_PX px}${MAP_RES:+  @ $MAP_RES m/px}"
printf '  %-11s : %s\n' "map origin" "${MAP_ORIGIN:-?}"
echo ""
echo " Localization topics under /ferox/$ROBOT_ID/ :"
echo "   map             latched OccupancyGrid from map_server"
echo "   amcl_pose       AMCL pose estimate (PoseWithCovarianceStamped)"
echo "   particle_cloud  AMCL particle filter cloud"
echo ""
echo " Verify topics:"
echo "   docker exec $NAV_CONTAINER bash -lc \\"
echo "     'source /workspace/install/setup.bash && ros2 topic list | grep -E \"map|amcl|particle\"'"
echo ""
echo " For AMCL to converge, seed an initial pose (RViz '2D Pose Estimate',"
echo " or publish to /ferox/$ROBOT_ID/initialpose)."
echo ""
exit 0
