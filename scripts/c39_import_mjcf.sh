#!/bin/bash
# C-39 task 1 — import the reference MuJoCo model (g1_29dof_old.xml) into Isaac.
#
# The A/B this serves: SONIC stands in the reference MuJoCo sim and falls in the twin.
# Running the reference's OWN body inside OUR simulator says which half is at fault.
#
#   ./scripts/c39_import_mjcf.sh
set -e
source "$(dirname "$0")/lib/env.sh"

# The gear_sonic copy is the one docs/mm/evidence/C39/mujoco_ref.json was computed from,
# so it is the one used here. The gear_sonic_deploy copy differs ONLY in where the `imu`
# site sits (pelvis vs torso); a site has no dynamics, so this changes nothing measured.
SRC="${C39_MJCF_SRC:-$HOME/panthera/ref/upstream/GR00T-WholeBodyControl/gear_sonic/data/robots/g1}"
[ -f "$SRC/g1_29dof_old.xml" ] || { echo "  ✗ reference MJCF not at $SRC"; exit 1; }
[ -d "$SRC/meshes" ] || { echo "  ✗ meshes/ not beside the MJCF at $SRC"; exit 1; }
docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

echo "Staging the reference model (xml + $(ls "$SRC/meshes" | wc -l) meshes)..."
docker exec -u root "$SIM_CONTAINER" rm -rf /tmp/c39_ref /tmp/c39_ref_usd
docker exec -u root "$SIM_CONTAINER" mkdir -p /tmp/c39_ref
docker cp "$SRC/g1_29dof_old.xml" "$SIM_CONTAINER:/tmp/c39_ref/g1_29dof_old.xml" >/dev/null
docker cp "$SRC/meshes" "$SIM_CONTAINER:/tmp/c39_ref/meshes" >/dev/null
docker exec -u root "$SIM_CONTAINER" bash -c "mkdir -p /tmp/isaacrun && chmod 777 /tmp/isaacrun"
docker cp "$DEMO_DIR/tools/c39_import_mjcf.py" "$SIM_CONTAINER:/tmp/isaacrun/c39_import_mjcf.py" >/dev/null
# a+rwX, not a+rX: the MJCF importer converts every STL to a temporary USD written
# NEXT TO the mesh (meshes/<name>_tmp/<name>.tmp.usd), so a read-only mesh dir fails
# the conversion and then dies on the NULL stage -- "Asset convert failed with error
# status: Unknown" followed by a Fatal, with the permission never mentioned.
docker exec -u root "$SIM_CONTAINER" chmod -R a+rwX /tmp/c39_ref

cat > /tmp/_c39_mjcf_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
# The MJCF importer is not enabled in the default headless app.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.mjcf")
app.update()
import sys
sys.argv = ["c39_import_mjcf"]
exec(open("/tmp/isaacrun/c39_import_mjcf.py").read())
app.close()
PY
docker cp /tmp/_c39_mjcf_entry.py "$SIM_CONTAINER:/tmp/isaacrun/_c39_mjcf_entry.py" >/dev/null

echo "Importing (headless Isaac)..."
docker exec -e PYTHONDONTWRITEBYTECODE=1 "$SIM_CONTAINER" \
  /isaac-sim/python.sh /tmp/isaacrun/_c39_mjcf_entry.py >/tmp/c39_mjcf_stdout.log 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/c39_import_mjcf.txt || {
  echo "  ✗ no report written; last 40 lines of stdout:"; tail -40 /tmp/c39_mjcf_stdout.log; exit 1; }

rm -rf "$DEMO_DIR/isaac/assets/g1_ref_mjcf"
docker cp "$SIM_CONTAINER:/tmp/c39_ref_usd" "$DEMO_DIR/isaac/assets/g1_ref_mjcf" >/dev/null
echo "  installed isaac/assets/g1_ref_mjcf ($(du -sh "$DEMO_DIR/isaac/assets/g1_ref_mjcf" | cut -f1))"
