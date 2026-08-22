#!/bin/bash
# C-39 task 1 — the decisive A/B: the SAME SONIC fork against two BODIES.
#
#   ./scripts/c39_ab_asset.sh twin      # our asset (HAND=dex5_1p), the known FALL
#   ./scripts/c39_ab_asset.sh ref       # the reference g1_29dof_old.xml via the MJCF importer
#
# Everything except the body is held fixed: same world, same bridge, same deploy
# binary, same DDS seam on lo, same rig release at t=30 s, same duration. The whole
# value of this experiment is that only ONE thing differs between the two runs, so
# nothing here may branch on the asset except the two lines that choose it.
#
# Reads the verdict off the bridge's own report line (base_z / pitch), which is the
# same instrument every earlier C-39 run was judged by.
set -e
# ROBOT must be set BEFORE lib/env.sh is sourced. env.sh derives ROBOT_ID from ROBOT
# and EXPORTS it, and every later `ROBOT=g1 ... 01_start_sim.sh` re-sources env.sh with
# ROBOT_ID already in the environment, where `${ROBOT_ID:-...}` keeps the inherited
# value. Sourcing this file with ROBOT unset therefore pins ROBOT_ID=go2_01 for the
# rest of the run, and the G1 twin dies at world load with "contract namespace
# /ferox/g1_01 != --ros_namespace /ferox/go2_01".
export ROBOT=g1
source "$(dirname "$0")/lib/env.sh"

ASSET="${1:?usage: c39_ab_asset.sh twin|ref [seconds]}"
DUR="${2:-90}"
OUT="$DEMO_DIR/docs/mm/evidence/C39/ab"
mkdir -p "$OUT"

case "$ASSET" in
  twin) HAND_ARG=dex5_1p; OVERRIDE="" ;;
  # twin_bare is the RIGHT twin side for this A/B and the reason is the reference:
  # g1_29dof_old.xml has no hands at all, so comparing it against the Dex5 twin would
  # differ in TWO things, not one. It is also the side that runs: with the Dex5 hands
  # attached SONIC aborts on its own velocity limit ("body_dq[24] = 35.367 > 35")
  # before the rig ever releases -- see evidence/C39/ab/twin_dex5_abort_*.
  twin_bare) HAND_ARG=none; OVERRIDE="" ;;
  ref)  HAND_ARG=none
        OVERRIDE=/workspace/ferox_isaac/assets/g1_ref_mjcf/g1_29dof_old.usd
        [ -f "$DEMO_DIR/isaac/assets/g1_ref_mjcf/g1_29dof_old.usd" ] || {
          echo "  ✗ reference USD missing; run ./scripts/c39_import_mjcf.sh first"; exit 1; }
        ;;
  *) echo "  ✗ asset must be 'twin' or 'ref'"; exit 1 ;;
esac

echo "=== C-39 A/B: asset=$ASSET  hand=$HAND_ARG  dur=${DUR}s ==="

# ---- 1. sim -----------------------------------------------------------------
docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true
env -u ROBOT_ID ROBOT=g1 TWIN=1 TWIN_CAMERA=0 TWIN_LIDAR=0 HAND="$HAND_ARG" SIM_WORLD=hospital \
  G1_CONTROL=lowcmd G1_LL_FIX_BASE=until_commanded G1_LL_RIG_RELEASE_S=30 \
  G1_LL_GT_TRACE=1 G1_LL_REPORT_STEPS=500 G1_USD_OVERRIDE="$OVERRIDE" \
  G1_LL_RIG_YAW=0 \
  bash "$DEMO_DIR/scripts/01_start_sim.sh"

# ---- 2. bridge --------------------------------------------------------------
docker rm -f mm3_bridge >/dev/null 2>&1 || true
docker run -d --name mm3_bridge --network host --ipc host --user 1234:1234 \
  -v "$DEMO_DIR/isaac/twin/lowlevel_bridge":/bridge:ro ferox/twin-lowlevel:humble \
  python3 /bridge/dds_side.py --domain 0 --iface lo --publish-hz 1041.68 >/dev/null
sleep 5

# ---- 3. SONIC ---------------------------------------------------------------
# --iface lo is required: SONIC takes its DDS interface as argv[1] and a bridge on
# any other interface is simply never discovered (RESULTS_MM4 "Reproduce").
docker rm -f mm4_sonic >/dev/null 2>&1 || true
docker run -d --name mm4_sonic --network host --gpus all \
  -v trt_policy:/opt/gear_sonic_deploy/policy/sonic_v1_1 \
  -v trt_planner:/opt/gear_sonic_deploy/planner/target_vel/V2 \
  ferox/sonic-deploy:v1.1-x86_64 \
  /opt/gear_sonic_deploy/target/release/g1_deploy_onnx_ref \
    lo policy/sonic_v1_1/model_decoder.onnx reference/example/ \
    --obs-config policy/sonic_v1_1/observation_config.yaml \
    --encoder-file policy/sonic_v1_1/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type zmq_manager --output-type all --zmq-host localhost --zmq-port 5556 >/dev/null

# ---- 4. drive (start held the whole run; the stand IS the experiment) --------
docker rm -f mm4_drive >/dev/null 2>&1 || true
docker run -d --name mm4_drive --network host \
  -e PYTHONPATH=/upstream/external_dependencies/unitree_sdk2_python:/upstream \
  -v "$HOME/panthera/ref/upstream/GR00T-WholeBodyControl":/upstream:ro \
  -v "$DEMO_DIR/scripts":/scripts:ro ferox/twin-lowlevel:humble \
  python3 /scripts/mm4_sonic_drive.py --bind 'tcp://*:5556' --hold-s "$DUR" >/dev/null

# Wall-clock is the wrong clock. RTF on this box runs ~0.15 with 1 kHz physics under
# G1_CONTROL=lowcmd, and SONIC's FIRST run also has to build its TensorRT engines, so a
# fixed wall window can end before the rig has even released. Wait on the SIM clock
# instead: release happens at t=30 s, and the verdict wants ~30 s of standing after it.
WANT_T="${WANT_T:-60}"
echo "  waiting for sim t=${WANT_T}s (wall budget ${DUR}s) ..."
_t0=$SECONDS
_t=0
while [ $((SECONDS-_t0)) -lt "$DUR" ]; do
  # Copy the tail out and parse it on the HOST. Doing the grep inside `docker exec
  # bash -c '...'` nests three levels of quoting around a regex full of brackets and
  # dollars, and one bad expansion there turns into a shell error on an unrelated line.
  docker exec "$SIM_CONTAINER" tail -c 200000 /tmp/sim.log > /tmp/_ab_tail.txt 2>/dev/null || true
  _t=$(python3 - <<'PYEOF'
import re
try:
    txt = open('/tmp/_ab_tail.txt', errors='replace').read()
except OSError:
    txt = ''
m = re.findall(r'\[lowlevel-sim\] t=\s*([0-9.]+)', txt)
print(m[-1] if m else '0')
PYEOF
)
  [ -z "$_t" ] && _t=0
  if awk -v a="$_t" -v b="$WANT_T" 'BEGIN{exit !(a+0>=b+0)}'; then
    echo "  reached sim t=$_t"; break
  fi
  sleep 10
done
echo "  stopping at sim t=$_t after $((SECONDS-_t0))s wall"

# ---- 5. harvest -------------------------------------------------------------
# Same code path as scripts/c39_ab_harvest.sh, which exists so a run that is already
# up can be captured without being restarted.
bash "$DEMO_DIR/scripts/c39_ab_harvest.sh" "$ASSET"
