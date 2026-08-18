#!/bin/bash
# ferox-isaac-demo — import the Dex5-1P hands from Unitree's official URDFs.
#
# Reads ~/panthera/ref/unitree_ros/robots/dexterous_hand_description/dex5_1 and writes
# isaac/assets/hands/dex5_1p/{left,right}.usd. 1:1 scale, URDF inertias, no welds.
#
#   ./scripts/11_import_dex5.sh
set -e
source "$(dirname "$0")/lib/env.sh"

URDF_SRC="${DEX5_URDF_DIR:-$HOME/panthera/ref/unitree_ros/robots/dexterous_hand_description/dex5_1}"
[ -d "$URDF_SRC" ] || { echo "  ✗ Dex5 URDFs not found at $URDF_SRC"; exit 1; }
docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

# Stage the URDFs + meshes into the container (they live outside the mounted repo).
# /tmp, not /workspace: docker cp writes as root while Isaac runs as UID 1234, so the
# cleanup needs -u root and the destination needs to be somewhere 1234 can read.
docker exec -u root "$SIM_CONTAINER" rm -rf /tmp/dex5_urdf
docker cp "$URDF_SRC" "$SIM_CONTAINER:/tmp/dex5_urdf" >/dev/null
docker exec -u root "$SIM_CONTAINER" chmod -R a+rX /tmp/dex5_urdf
docker cp "$DEMO_DIR/tools/import_dex5.py" "$SIM_CONTAINER:/tmp/import_dex5.py" >/dev/null

cat > /tmp/_dex5_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.argv = ["import_dex5"]
exec(open("/tmp/import_dex5.py").read())
app.close()
PY
docker cp /tmp/_dex5_entry.py "$SIM_CONTAINER:/tmp/_dex5_entry.py" >/dev/null

echo "Importing Dex5-1P (headless Isaac, ~60 s)..."
docker exec -e DEX5_OUT_DIR=/tmp/dex5_usd "$SIM_CONTAINER" \
  /isaac-sim/python.sh /tmp/_dex5_entry.py >/dev/null 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/dex5_import.txt

# Copy the WHOLE tree. The importer emits the same layered structure the G1 asset
# uses -- a small top-level stub plus configuration/*_{base,physics,sensor}.usd, with
# the meshes in the base layer. Copying only the stub installs a 1.5 kB file that
# opens without error and contains no hand.
rm -rf "$DEMO_DIR/isaac/assets/hands/dex5_1p"
mkdir -p "$DEMO_DIR/isaac/assets/hands"
docker cp "$SIM_CONTAINER:/tmp/dex5_usd" "$DEMO_DIR/isaac/assets/hands/dex5_1p" >/dev/null
echo "  installed isaac/assets/hands/dex5_1p ($(du -sh "$DEMO_DIR/isaac/assets/hands/dex5_1p" | cut -f1))"
for s in l r; do
  f="$DEMO_DIR/isaac/assets/hands/dex5_1p/configuration/dex5_1p_${s}_base.usd"
  [ -s "$f" ] || { echo "  ✗ missing or empty $f"; exit 1; }
done
docker exec "$SIM_CONTAINER" bash -lc 'grep -q "RESULT: PASS" /tmp/dex5_import.txt' || {
  echo "  ✗ import verification FAILED — see the report above"; exit 1; }
