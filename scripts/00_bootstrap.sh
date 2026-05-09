#!/bin/bash
# ferox-isaac-demo — bootstrap a fresh machine.
#
# What this does (idempotent — safe to re-run):
#   1. Pulls Isaac Sim 5.1.0 image (~22 GB)
#   2. Builds the two Ferox images from the Ferox repo (msgs then nav)
#   3. Creates Isaac Sim cache dirs with UID 1234 ownership
#   4. Verifies walking-policy assets ship with this repo (no OM1 dep)
#
# After this, run ./scripts/01_start_sim.sh + ./scripts/02_start_ferox.sh.

set -e
source "$(dirname "$0")/lib/env.sh"

echo "==============================================="
echo " ferox-isaac-demo — bootstrap"
echo "==============================================="
echo " DEMO_DIR  : $DEMO_DIR"
echo " FEROX_REPO: $FEROX_REPO"
echo " ROBOT     : $ROBOT  (override with ROBOT=g1 ...)"
echo ""

# ---- [1/5] Sanity checks ----
echo "[1/5] Verifying repo layout..."
[ -d "$DEMO_DIR/isaac/checkpoints/$ROBOT" ] || {
  echo "  ✗ Missing $DEMO_DIR/isaac/checkpoints/$ROBOT"
  echo "    The walking policy ships in this repo. Re-clone or rsync."
  exit 1
}
[ -f "$DEMO_DIR/isaac/run.py" ] || {
  echo "  ✗ Missing $DEMO_DIR/isaac/run.py"; exit 1
}
[ -d "$FEROX_REPO/docker" ] || {
  echo "  ✗ Ferox repo not at $FEROX_REPO."
  echo "    Clone the Ferox nav repo or set FEROX_REPO=<path>."
  exit 1
}
echo "  ✓ assets + Ferox repo present"

# ---- [2/5] Pull Isaac Sim image ----
echo ""
echo "[2/5] Pulling Isaac Sim image ($ISAAC_IMAGE) ..."
docker pull "$ISAAC_IMAGE"

# ---- [3/5] Build Ferox images ----
echo ""
echo "[3/5] Building Ferox images (msgs → nav) ..."
( cd "$FEROX_REPO" && [ -f .env ] || cp .env.example .env )
( cd "$FEROX_REPO" && \
    docker compose -f docker/docker-compose.yml --profile build-only build msgs && \
    docker compose -f docker/docker-compose.yml build nav )

# ---- [4/5] Cache dirs (Isaac Sim runs as UID 1234) ----
echo ""
echo "[4/5] Creating Isaac Sim cache dirs at $CACHE_DIR ..."
mkdir -p "$CACHE_DIR"/{kit,ov,pip,gl,compute,warp}
chown -R 1234:1234 "$CACHE_DIR" 2>/dev/null || \
  echo "  (chown skipped — not root; Isaac Sim will fall back to user-writable cache)"
ls -la "$CACHE_DIR" | head -10

# ---- [5/5] Verify ----
echo ""
echo "[5/5] Verifying critical files..."
MISSING=0
chk() { [ -e "$1" ] && echo "  ✓ $1" || { echo "  ✗ MISSING: $1"; MISSING=$((MISSING+1)); }; }
chk "$DEMO_DIR/isaac/run.py"
chk "$DEMO_DIR/isaac/sim_utils.py"
chk "$DEMO_DIR/isaac/checkpoints/$ROBOT/exported/policy.pt"
chk "$DEMO_DIR/isaac/checkpoints/$ROBOT/params/env.yaml"
chk "$DEMO_DIR/isaac/assets/$ROBOT"
[ "$MISSING" -eq 0 ] || { echo "Bootstrap incomplete — $MISSING missing file(s)."; exit 1; }

echo ""
echo "==============================================="
echo " Bootstrap complete."
echo "==============================================="
echo ""
echo " Next:  ./scripts/01_start_sim.sh    (Isaac Sim + walking policy)"
echo "        ./scripts/02_start_ferox.sh  (Nav2 + SLAM in /ferox/$ROBOT_ID/)"
echo ""
