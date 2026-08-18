#!/bin/bash
# ferox-isaac-demo — merge the G1 29-DoF and Dex5-1P URDFs and import the result.
#
# A referenced hand composes into the stage but never joins the G1's PhysX
# articulation, so the fingers cannot be commanded. See tools/merge_dex5_urdf.py.
#
#   ./scripts/12_import_g1_dex5.sh
set -e
source "$(dirname "$0")/lib/env.sh"

REF="${UNITREE_ROS_DIR:-$HOME/panthera/ref/unitree_ros/robots}"
[ -d "$REF/g1_description" ] || { echo "  ✗ unitree_ros not found at $REF"; exit 1; }
docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

echo "Merging URDFs..."
rm -rf /tmp/g1_dex5_urdf
python3 "$DEMO_DIR/tools/merge_dex5_urdf.py" --ref "$REF" --out-dir /tmp/g1_dex5_urdf

# /tmp, not /workspace: docker cp writes as root while Isaac runs as UID 1234.
docker exec -u root "$SIM_CONTAINER" rm -rf /tmp/g1_dex5_urdf
docker cp /tmp/g1_dex5_urdf "$SIM_CONTAINER:/tmp/g1_dex5_urdf" >/dev/null
docker exec -u root "$SIM_CONTAINER" chmod -R a+rX /tmp/g1_dex5_urdf
docker cp "$DEMO_DIR/tools/import_g1_dex5.py" "$SIM_CONTAINER:/tmp/isaacrun/import_g1_dex5.py" >/dev/null

cat > /tmp/_g1dex5_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.argv = ["import_g1_dex5"]
exec(open("/tmp/isaacrun/import_g1_dex5.py").read())
app.close()
PY
docker exec -u root "$SIM_CONTAINER" bash -c "mkdir -p /tmp/isaacrun && chmod 777 /tmp/isaacrun"
# Isaac scripts go in their own dir, never bare /tmp: sys.path[0] is the script's
# directory, so a scratch file named after a stdlib module (a probe called bisect.py
# did this) shadows it for Isaac's own startup and every script in /tmp then dies
# at "from isaacsim import SimulationApp".
docker cp /tmp/_g1dex5_entry.py "$SIM_CONTAINER:/tmp/isaacrun/_g1dex5_entry.py" >/dev/null

echo "Importing merged G1+Dex5 (headless Isaac, ~2 min)..."
docker exec -e PYTHONDONTWRITEBYTECODE=1 "$SIM_CONTAINER" /isaac-sim/python.sh /tmp/isaacrun/_g1dex5_entry.py >/dev/null 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/g1_dex5_import.txt

# Copy the WHOLE tree -- the importer emits a stub plus configuration/*.usd layers.
rm -rf "$DEMO_DIR/isaac/assets/g1_dex5"
docker cp "$SIM_CONTAINER:/tmp/g1_dex5_usd" "$DEMO_DIR/isaac/assets/g1_dex5" >/dev/null
echo "  installed isaac/assets/g1_dex5 ($(du -sh "$DEMO_DIR/isaac/assets/g1_dex5" | cut -f1))"
