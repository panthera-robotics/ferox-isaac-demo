#!/bin/bash
# Harvest one C-39 A/B side from the LIVE stack (sim + bridge + sonic + drive already up).
# Split out of c39_ab_asset.sh so a side that is already running can be captured without
# restarting it -- editing or re-running the launcher would throw the run away.
set -e
ASSET="${1:?usage: harvest.sh twin|ref}"
DEMO_DIR=/root/panthera/ferox-isaac-demo
OUT="$DEMO_DIR/docs/mm/evidence/C39/ab"
mkdir -p "$OUT"
docker exec ferox_isaac_sim cat /tmp/sim.log > "$OUT/${ASSET}_sim.log" 2>/dev/null || true
docker logs mm4_sonic  > "$OUT/${ASSET}_sonic.log" 2>&1 || true
docker logs mm3_bridge > "$OUT/${ASSET}_bridge.log" 2>&1 || true
docker logs mm4_drive  > "$OUT/${ASSET}_drive.log" 2>&1 || true
python3 "$DEMO_DIR/tools/c39_ab_verdict.py" "$OUT/${ASSET}_sim.log" "$ASSET" \
  | tee "$OUT/${ASSET}_verdict.txt"
