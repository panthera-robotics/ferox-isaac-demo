#!/bin/bash
# ferox-isaac-demo — twin sensor tests that need a live Isaac Sim.
#
# These assert that every omni:sensor attribute we SET is an attribute that TOOK,
# and that Isaac still renames the prim on an unknown lidar config (the silent
# fallback lidar.py guards against). Run after any Isaac image bump.
#
#   ./scripts/10_test_twin_isaac.sh
set -e
source "$(dirname "$0")/lib/env.sh"
docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

docker cp "$DEMO_DIR/tools/tests/test_twin_isaac.py" "$SIM_CONTAINER:/tmp/test_twin_isaac.py" >/dev/null
echo "Running twin Isaac sensor tests (boots a headless Isaac, ~60 s)..."
docker exec "$SIM_CONTAINER" /isaac-sim/python.sh /tmp/test_twin_isaac.py >/dev/null 2>&1 || true
docker exec "$SIM_CONTAINER" cat /tmp/twin_isaac_tests.txt
docker exec "$SIM_CONTAINER" bash -lc 'grep -q FAIL /tmp/twin_isaac_tests.txt && exit 1 || exit 0'
