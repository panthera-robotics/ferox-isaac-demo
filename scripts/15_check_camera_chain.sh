#!/bin/bash
# ferox-isaac-demo — prove the twin camera's 3D geometry (C-21).
#
# Three independent checks against a live twin: the floor from back-projected depth,
# the floor from the published xyzrgb cloud, and a known object's placement. Also
# asserts the LIDAR was not carrying the same convention error.
#
# Needs the twin sim AND the nav stack, with props for the object check:
#   ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=dso_block_a \
#     FEROX_SIM_TEST_PROPS=1 CAMERA_TF=1 ./scripts/01_start_sim.sh
#   ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh
#   ./scripts/15_check_camera_chain.sh
set -e
source "$(dirname "$0")/lib/env.sh"

docker ps --format '{{.Names}}' | grep -q "^${NAV_CONTAINER}$" || {
  echo "  ✗ $NAV_CONTAINER not running. Run ./02_start_ferox.sh first."; exit 1; }

docker cp "$DEMO_DIR/tools/check_twin_camera_chain.py" \
  "$NAV_CONTAINER:/tmp/check_twin_camera_chain.py" >/dev/null

docker exec -e PYTHONDONTWRITEBYTECODE=1 "$NAV_CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  python3 /tmp/check_twin_camera_chain.py' 2>&1 | grep -v "^\[INFO\|^\[WARN"
