#!/bin/bash
# ferox-isaac-demo — render the G1 twin from front, side and top (DT2 visual pass).
#
# Offscreen, in its own headless Isaac. NOT a GUI screenshot: this box has no
# logged-in desktop X session, so the viewport renders empty.
#
#   ./scripts/14_capture_views.sh
set -e
source "$(dirname "$0")/lib/env.sh"

docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

docker exec -u root "$SIM_CONTAINER" bash -c 'mkdir -p /tmp/isaacrun && chmod 777 /tmp/isaacrun'
docker cp "$DEMO_DIR/tools/capture_robot_views.py" \
  "$SIM_CONTAINER:/tmp/isaacrun/capture_robot_views.py" >/dev/null

cat > /tmp/_views_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
exec(open("/tmp/isaacrun/capture_robot_views.py").read())
app.close()
PY
docker cp /tmp/_views_entry.py "$SIM_CONTAINER:/tmp/isaacrun/_views_entry.py" >/dev/null

echo "Rendering robot views (headless Isaac, ~2 min)..."
docker exec -e PYTHONDONTWRITEBYTECODE=1 "$SIM_CONTAINER" \
  /isaac-sim/python.sh /tmp/isaacrun/_views_entry.py >/dev/null 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/robot_views.txt

DST="$DEMO_DIR/docs/twin/evidence/DT2"
mkdir -p "$DST"
for v in front side top; do
  docker cp "$SIM_CONTAINER:/tmp/robot_views/g1_twin_${v}.png" "$DST/g1_twin_${v}.png" 2>/dev/null \
    && echo "  installed docs/twin/evidence/DT2/g1_twin_${v}.png" \
    || echo "  ✗ g1_twin_${v}.png not produced"
done
