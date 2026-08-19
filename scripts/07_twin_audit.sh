#!/bin/bash
# ferox-isaac-demo — run tools/twin_audit.py against the live ROS 2 graph.
#
# The audit needs rclpy and the message types, which live in the Ferox nav image, not
# on the host. This script stages tools/ + the contract into $NAV_CONTAINER and runs
# there. It is a COPY, not an install: per the repo's docker-immutability rule nothing
# is apt/pip-installed inside a running container, and everything the audit imports
# (rclpy, yaml, rosidl_runtime_py, rosbag2_py, numpy) is already baked into the image.
#
# Usage:
#   ./scripts/07_twin_audit.sh                    # audit ROBOT's contract, live
#   ROBOT=g1 ./scripts/07_twin_audit.sh
#   ./scripts/07_twin_audit.sh --duration 20 --quiet
#   ./scripts/07_twin_audit.sh --bag /path/to/bag
#
# --against-evidence needs no ROS at all; run it on the host instead:
#   python3 tools/twin_audit.py --contract isaac/twin/g1_contract.yaml \
#           --against-evidence ~/panthera/ref/panthera-g1-driver/evidence

set -e
source "$(dirname "$0")/lib/env.sh"

CONTRACT_HOST="$DEMO_DIR/isaac/twin/${ROBOT}_contract.yaml"
[ -f "$CONTRACT_HOST" ] || { echo "  ✗ no contract for ROBOT=$ROBOT at $CONTRACT_HOST"; exit 1; }

docker ps --format '{{.Names}}' | grep -q "^${NAV_CONTAINER}$" || {
  echo "  ✗ $NAV_CONTAINER not running. Run ./02_start_ferox.sh first."; exit 1; }

STAGE=/tmp/twin_audit
docker exec "$NAV_CONTAINER" rm -rf "$STAGE"
docker exec "$NAV_CONTAINER" mkdir -p "$STAGE/isaac/twin"
docker cp "$DEMO_DIR/tools/."          "$NAV_CONTAINER:$STAGE/tools" >/dev/null
docker cp "$DEMO_DIR/isaac/twin/."     "$NAV_CONTAINER:$STAGE/isaac/twin" >/dev/null

echo "Auditing ROBOT=$ROBOT against isaac/twin/${ROBOT}_contract.yaml ..."
echo ""
docker exec "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /opt/ferox_msgs_ws/install/setup.bash 2>/dev/null || true
  source /workspace/install/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
  cd $STAGE
  python3 tools/twin_audit.py --contract isaac/twin/${ROBOT}_contract.yaml $*
"
