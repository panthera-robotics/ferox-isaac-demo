#!/bin/bash
# ferox-isaac-demo — clean shutdown. Containers persist (just stopped) so
# the next 01_start_sim.sh / 02_start_ferox.sh is faster (caches warm).

set +e
source "$(dirname "$0")/lib/env.sh"
source "$(dirname "$0")/lib/kill_nav_stack.sh"

echo "==============================================="
echo " ferox-isaac-demo — shutdown"
echo "==============================================="

echo "[1/3] Killing in-container processes..."
# Clear the WHOLE nav stack (node executables, not just the launch parent).
# The old `pkill -f "ros2 launch"` here killed only the parent and left its
# children orphaned to PID 1 — the same gap that produced duplicate action
# servers on the next launch. kill_nav_stack is the shared, complete killer.
kill_nav_stack "$NAV_CONTAINER"
# Then the debug helpers a session may have left running. Bare docker exec
# pkill (no wrapping shell that could carry — and match — the pattern).
docker exec "$NAV_CONTAINER" pkill -9 -f 'rviz2|ros2 topic pub|ros2 action send_goal' 2>/dev/null || true
docker exec "$SIM_CONTAINER" pkill -9 -f run.py 2>/dev/null || true
sleep 2

echo ""
echo "[2/3] Stopping containers..."
if [ -f "$FEROX_REPO/docker/docker-compose.yml" ]; then
  ( cd "$FEROX_REPO" && docker compose -f docker/docker-compose.yml down ) \
    >/dev/null 2>&1 || echo "  ⚠ compose down failed; falling back to docker stop"
fi
# Belt-and-suspenders: stop by name in case compose path was wrong.
docker stop "$NAV_CONTAINER" "$SIM_CONTAINER" >/dev/null 2>&1 || true

echo ""
echo "[3/3] Final state:"
docker ps --format "table {{.Names}}\t{{.Status}}" | head -10

echo ""
echo "==============================================="
echo " Shutdown complete."
echo "==============================================="
echo " Restart: ./scripts/01_start_sim.sh && ./scripts/02_start_ferox.sh"
