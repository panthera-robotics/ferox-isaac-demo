#!/bin/bash
# ferox-isaac-demo — clean shutdown. Containers persist (just stopped) so
# the next 01_start_sim.sh / 02_start_ferox.sh is faster (caches warm).

set +e
source "$(dirname "$0")/lib/env.sh"

echo "==============================================="
echo " ferox-isaac-demo — shutdown"
echo "==============================================="

echo "[1/3] Killing in-container processes..."
docker exec "$NAV_CONTAINER" bash -lc 'pkill -9 -f "ros2 launch|topic pub|rviz2|action send_goal"' 2>/dev/null
docker exec "$SIM_CONTAINER" bash -lc 'pkill -9 -f run.py' 2>/dev/null
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
