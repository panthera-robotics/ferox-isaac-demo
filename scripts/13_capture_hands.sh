#!/bin/bash
# ferox-isaac-demo — render the Dex5-1P hands at rest / open / fist / thumb opposition.
#
# Deterministic and independent of the live sim: loads the merged asset in its own
# headless Isaac, sets each pose BY JOINT NAME, and writes PNGs.
#
#   ./scripts/13_capture_hands.sh
set -e
source "$(dirname "$0")/lib/env.sh"

docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

# Isaac scripts get their own directory, never bare /tmp -- sys.path[0] is the
# script's directory and a scratch file there can shadow a stdlib module.
docker exec -u root "$SIM_CONTAINER" bash -c 'mkdir -p /tmp/isaacrun && chmod 777 /tmp/isaacrun'
docker cp "$DEMO_DIR/tools/capture_hand_poses.py" \
  "$SIM_CONTAINER:/tmp/isaacrun/capture_hand_poses.py" >/dev/null

cat > /tmp/_hands_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
exec(open("/tmp/isaacrun/capture_hand_poses.py").read())
app.close()
PY
docker cp /tmp/_hands_entry.py "$SIM_CONTAINER:/tmp/isaacrun/_hands_entry.py" >/dev/null

echo "Rendering hand poses (headless Isaac, ~2 min)..."
docker exec -e PYTHONDONTWRITEBYTECODE=1 "$SIM_CONTAINER" \
  /isaac-sim/python.sh /tmp/isaacrun/_hands_entry.py >/dev/null 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/hand_poses.txt

DST="$DEMO_DIR/docs/twin/evidence/DT3"
mkdir -p "$DST"
for p in rest open fist thumb_opposition; do
  docker cp "$SIM_CONTAINER:/tmp/hand_poses/hand_${p}.png" "$DST/hand_${p}.png" 2>/dev/null \
    && echo "  installed docs/twin/evidence/DT3/hand_${p}.png" \
    || echo "  ✗ hand_${p}.png not produced"
done
