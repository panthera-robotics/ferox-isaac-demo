#!/bin/bash
# MM5 instrument preflight -- run this BEFORE any grasp session.
#
# Five instrument defects in one session produced five confident wrong numbers. Every one
# would have been caught by asking the gauge what it reports on a case whose answer is
# already known. Exit non-zero means DO NOT TRUST GRASP NUMBERS.
set -e
source "$(dirname "$0")/lib/env.sh"
docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$" || {
  echo "  x $SIM_CONTAINER not running. Run ./01_start_sim.sh first."; exit 1; }
docker exec -u root "$SIM_CONTAINER" bash -c 'mkdir -p /tmp/isaacrun && chmod 777 /tmp/isaacrun'
docker cp "$DEMO_DIR/tools/mm5_preflight.py" "$SIM_CONTAINER:/tmp/isaacrun/mm5_preflight.py" >/dev/null
cat > /tmp/_pref_entry.py <<'PY'
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import sys
sys.argv = ["mm5_preflight"]
rc = 0
try:
    exec(open("/tmp/isaacrun/mm5_preflight.py").read())
except SystemExit as e:
    rc = int(e.code or 0)
app.close()
PY
docker cp /tmp/_pref_entry.py "$SIM_CONTAINER:/tmp/isaacrun/_pref_entry.py" >/dev/null
docker exec -e PYTHONDONTWRITEBYTECODE=1 "$SIM_CONTAINER" \
  /isaac-sim/python.sh /tmp/isaacrun/_pref_entry.py 2>&1 | grep -E 'GAUGE|PASS|FAIL|PREFLIGHT'
docker exec "$SIM_CONTAINER" cat /tmp/mm5_preflight.json > "$DEMO_DIR/docs/mm/evidence/MM5/preflight_latest.json"
grep -q '"all_pass": true' "$DEMO_DIR/docs/mm/evidence/MM5/preflight_latest.json" || {
  echo "  x PREFLIGHT FAILED -- fix the gauge before measuring anything."; exit 1; }
