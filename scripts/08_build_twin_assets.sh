#!/bin/bash
# ferox-isaac-demo — regenerate the twin sensor USD layer from the contract.
#
# Runs tools/build_twin_assets.py inside the sim container, which is the only
# place pxr is available. The generator rewrites ONLY
# configuration/<name>_sensor.usd; the base and physics layers are never touched.
#
#   ./scripts/08_build_twin_assets.sh          # g1
#   ./scripts/08_build_twin_assets.sh g1 go2
#
# The generator verifies its own output against the contract and exits non-zero
# on any mismatch, so a silent half-write cannot reach a commit.
set -e
source "$(dirname "$0")/lib/env.sh"

docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  ✗ $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }

# python.sh has yaml but not pxr; the omni.usd.libs extension supplies pxr and
# its shared objects. kit/python has pxr reachable the same way but no yaml, so
# python.sh + this PYTHONPATH is the only interpreter with both.
USD_LIBS=$(docker exec "$SIM_CONTAINER" bash -lc 'ls -d /isaac-sim/extscache/omni.usd.libs-*' | head -1)
# Isaac runs as UID 1234; the repo files are root-owned, so the generator stages
# into /tmp inside the container and we copy the result back. Beats chowning
# tracked files out from under git.
STAGE=/tmp/twin_assets
docker exec "$SIM_CONTAINER" mkdir -p "$STAGE"
docker exec \
  -e PYTHONPATH="$USD_LIBS:/workspace/ferox_tools" \
  -e LD_LIBRARY_PATH="$USD_LIBS/bin" \
  -e TWIN_ASSET_OUT_DIR="$STAGE" \
  "$SIM_CONTAINER" /isaac-sim/python.sh \
  /workspace/ferox_tools/build_twin_assets.py ${@:-g1}

for _r in ${@:-g1}; do
  case "$_r" in
    g1)  _name=g1_29dof_rev_1_0_sensor.usd; _dst="$DEMO_DIR/isaac/assets/g1/usd/configuration" ;;
    go2) _name=go2_description_sensor.usd;  _dst="$DEMO_DIR/isaac/assets/go2/usd/configuration" ;;
    *)   echo "  ✗ unknown robot $_r"; exit 1 ;;
  esac
  docker cp "$SIM_CONTAINER:$STAGE/$_name" "$_dst/$_name"
  echo "  installed $_dst/$_name ($(stat -c%s "$_dst/$_name") bytes)"
done
