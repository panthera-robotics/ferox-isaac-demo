#!/bin/bash
# ferox-isaac-demo — start Isaac Sim with the Go2/G1 walking policy.
#
# Mounts $DEMO_DIR/isaac/ into /workspace/ferox_isaac inside the sim
# container, then runs run.py. The walking policy is a frozen .pt tensor
# loaded by Isaac Sim — no OM1 SDK at runtime.
#
# Sim publishes default-namespace topics (/scan, /odom, /imu, /cmd_vel).
# Ferox's sim bridge (started by 02_start_ferox.sh) relays these to
# /ferox/<robot_id>/...
#
# World selection (INDEPENDENT of robot):
#   Default world is the original warehouse venue (dso_block_a). Override with
#   SIM_WORLD=<name>:
#       SIM_WORLD=dso_block_a  ./01_start_sim.sh   # the warehouse venue (default)
#       SIM_WORLD=office       ./01_start_sim.sh   # NVIDIA Office env
#       SIM_WORLD=hospital     ./01_start_sim.sh   # NVIDIA Hospital env
#   ROBOT=go2|g1 selects the robot independently of the world.
#   Adding a world = ONE line in isaac/run.py SIM_WORLDS:
#       name -> { usd: <path under the assets root>, spawn: {xy, yaw} }
#   (z / standing height comes from the robot, so one entry works for any robot.)

set -e
source "$(dirname "$0")/lib/env.sh"

echo "==============================================="
echo " ferox-isaac-demo — start sim (ROBOT=$ROBOT)"
echo "==============================================="
echo ""

# X11 forwarding so the sim viewport renders in VNC/Selkies
echo "[1/4] X11 forwarding..."
echo "  DISPLAY      : $HOST_DISPLAY"
echo "  Xauthority   : $XAUTH_FILE"
if [ -f "$XAUTH_FILE" ]; then
  sudo -u "$DESKTOP_USER" DISPLAY="$HOST_DISPLAY" XAUTHORITY="$XAUTH_FILE" \
    xhost +local: > /dev/null 2>&1 && echo "  xhost +local: granted" \
    || echo "  xhost failed — run 'xhost +local:' manually in the VNC terminal if needed"
else
  echo "  No Xauthority — sim falls back to headless"
fi

echo ""
echo "[2/4] Stopping any prior sim container..."
docker rm -f "$SIM_CONTAINER" >/dev/null 2>&1 || true

echo ""
echo "[2.5/4] Rendering cyclone DDS config..."
CYCLONE_FILE="$("$(dirname "$0")/lib/render_cyclone.sh")"
echo "  ✓ rendered to $CYCLONE_FILE"

echo ""
echo "[3/4] Starting Isaac Sim container ($SIM_CONTAINER)..."

# Isaac Sim runs as UID 1234 inside the container; every cache mount must be
# writable by that uid or asset downloads fail ("Could not download local file")
# and the world loads EMPTY — a black viewport. A fresh checkout / fresh boot can
# leave these owned by the host user, so normalize ownership here (idempotent).
SIM_CONTAINER_UID="${SIM_CONTAINER_UID:-1234}"
mkdir -p "$CACHE_DIR"/{kit,ov,pip,gl,compute,warp}
if [ "$(stat -c '%u' "$CACHE_DIR" 2>/dev/null)" != "$SIM_CONTAINER_UID" ]; then
  echo "  Normalizing cache ownership ($CACHE_DIR -> $SIM_CONTAINER_UID)..."
  sudo chown -R "$SIM_CONTAINER_UID:$SIM_CONTAINER_UID" "$CACHE_DIR"
fi

# tools/ is mounted read-only so the sim can import the SAME contract loader and
# validator the audit uses (tools/twin_contract.py). One validator, one definition
# of a valid contract -- a second copy inside isaac/ would drift, and a contract
# the sim accepts but the audit rejects is worse than no contract at all.
#
# G1 policy source-of-truth: when the ferox-g1-locomotion repo is present
# (G1_POLICY_DIR resolved in lib/env.sh) and we're launching the G1, overlay
# its policy/ onto the G1 checkpoint slot so the sim runs that policy with no
# change to run.py. Empty for Go2 or when the repo is absent (falls back to the
# bundled isaac/checkpoints/g1). Unquoted on the docker line so it word-splits
# into `-v <src>:<dst>:ro`.
G1_POLICY_MOUNT=""
if [ "$ROBOT" = "g1" ] && [ -n "$G1_POLICY_DIR" ] && [ -f "$G1_POLICY_DIR/exported/policy.pt" ]; then
  G1_POLICY_MOUNT="-v $G1_POLICY_DIR:/workspace/ferox_isaac/checkpoints/g1:ro"
  echo "  G1 policy source: $G1_POLICY_DIR (overlay -> checkpoints/g1)"
fi

docker run -d --name "$SIM_CONTAINER" --runtime=nvidia --gpus all \
  --user 1234:1234 \
  --network host \
  --ipc host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -e HOME=/isaac-sim \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
  -e LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  -e DISPLAY="$HOST_DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v "$CYCLONE_FILE":/tmp/cyclonedds.xml:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTH_FILE":/tmp/.docker.xauth:ro \
  -v "$CACHE_DIR/kit":/isaac-sim/kit/cache:rw \
  -v "$CACHE_DIR/ov":/isaac-sim/.cache/ov:rw \
  -v "$CACHE_DIR/pip":/isaac-sim/.cache/pip:rw \
  -v "$CACHE_DIR/gl":/isaac-sim/.cache/nvidia/GLCache:rw \
  -v "$CACHE_DIR/compute":/isaac-sim/.nv/ComputeCache:rw \
  -v "$CACHE_DIR/warp":/isaac-sim/.cache/warp:rw \
  -v "$DEMO_DIR/isaac":/workspace/ferox_isaac:rw \
  -v "$DEMO_DIR/tools":/workspace/ferox_tools:ro \
  -e TWIN_FILM="${TWIN_FILM:-0}" \
  -e TWIN_FILM_SHOT="${TWIN_FILM_SHOT:-chase}" \
  -e TWIN_FILM_SECONDS="${TWIN_FILM_SECONDS:-0}" \
  -e TWIN_FILM_SUBFRAMES="${TWIN_FILM_SUBFRAMES:-32}" \
  -e TWIN_FILM_OUT=/tmp/film/live \
  -v "${FILM_OUT_DIR:-/tmp/film}":/tmp/film:rw \
  $G1_POLICY_MOUNT \
  --entrypoint bash \
  "$ISAAC_IMAGE" \
  -c "tail -f /dev/null" >/dev/null
sleep 3
echo "  ✓ container up"

# Tag the container with the robot it's running so 02_start_ferox.sh can
# detect a sim/nav robot mismatch before bringing up a wrong-namespaced
# nav stack. Single-line value, no formatting — the guard reads it via
# `docker exec ... cat /tmp/sim_robot_type`.
docker exec "$SIM_CONTAINER" sh -c "echo $ROBOT > /tmp/sim_robot_type"

echo ""
echo "[4/4] Launching run.py inside Isaac Sim (boot ~60 sec)..."
echo "  ROBOT=$ROBOT   SIM_WORLD=${SIM_WORLD:-dso_block_a}   TWIN=${TWIN:-0}   CAMERA_TF=${CAMERA_TF:-0}   HAND=${HAND:-none}"
# Subscribe directly to /ferox/<robot_id>/cmd_vel — matches what Nav2
# publishes inside its namespace, no relay needed. Avoids the QoS war
# that occurs when multiple Nav2 publishers (volatile + transient_local)
# share a relayed topic with manual `ros2 topic pub` clients.
SIM_CMD_VEL_TOPIC="/ferox/${ROBOT_ID}/cmd_vel"

# TWIN RENDER STEP. Isaac rounds rendering_dt to a whole number of physics substeps
# but get_rendering_dt() keeps reporting the value you asked for. With the default
# --physics_dt 1/200, a requested 1/60 (0.016667) actually runs at 0.015 s = 66.67 Hz,
# so decimating the lidar by round(60/10)=6 gave 11.11 Hz against a 10 Hz contract --
# a miss that reads as sensor jitter and is really arithmetic.
#
# The lidar is decimated from the render clock by an INTEGER step, so the render rate
# has to be an integer multiple of the contract's lidar rate or the sensor lands on
# the wrong one -- and it lands there silently, looking like jitter.
#
#   G1   10 Hz lidar: 0.020 s = 4 physics substeps = 50 Hz render, step 5 -> 10 Hz
#   Go2  20 Hz lidar: 0.025 s = 5 physics substeps = 40 Hz render, step 2 -> 20 Hz
#
# 0.02 s on the Go2 gives 50/20 = 2.5, which is not an integer: the gate rounds to 2
# and every cloud, scan and accumulated cloud comes out at 25 Hz. That is exactly
# what the first Go2 audit measured.
#
# physics_dt is untouched either way, so the walking policy (200 Hz physics,
# decimation 4, 50 Hz policy) is unaffected. Legacy mode:=sim keeps its old default.
#
# G1_CONTROL=lowcmd is the one case that DOES move physics_dt: run.py drops it to
# 1/1000 so rt/lowstate.tick advances one millisecond per step (MM3). The render
# steps above survive that unchanged -- 0.02 / 0.001 = 20 and 0.025 / 0.001 = 25,
# both integers -- so the lidar decimation arithmetic still lands on the contract.
TWIN_RENDER_ARG=""
if [ "${TWIN:-0}" = "1" ]; then
  case "$ROBOT" in
    go2) _default_render_dt=0.025 ;;
    *)   _default_render_dt=0.02 ;;
  esac
  TWIN_RENDER_ARG="--render_dt ${TWIN_RENDER_DT:-$_default_render_dt}"
  echo "  twin render step: ${TWIN_RENDER_DT:-$_default_render_dt}s (exact multiple of physics_dt)"
fi
# SIM_WORLD selects the environment USD (default dso_block_a); run.py reads it
# from the env. docker exec does not inherit the host env, so pass it explicitly.
docker exec -d \
  -e FEROX_SIM_TEST_PROPS="${FEROX_SIM_TEST_PROPS:-0}" \
  -e SIM_WORLD="${SIM_WORLD:-dso_block_a}" \
  -e TWIN="${TWIN:-0}" \
  -e CAMERA_TF="${CAMERA_TF:-0}" \
  -e TWIN_DUMP_SDG="${TWIN_DUMP_SDG:-0}" \
  -e TWIN_CAMERA="${TWIN_CAMERA:-1}" \
  -e TWIN_CONTACT_MATERIAL="${TWIN_CONTACT_MATERIAL:-0}" \
  -e TWIN_ARMATURE="${TWIN_ARMATURE:-}" \
  -e TWIN_JOINT_FRICTION="${TWIN_JOINT_FRICTION:-}" \
  -e TWIN_SENSOR_MASS_FIX="${TWIN_SENSOR_MASS_FIX:-0}" \
  -e TWIN_HAND_CONTACT_SENSORS="${TWIN_HAND_CONTACT_SENSORS:-0}" \
  -e HAND="${HAND:-none}" \
  -e MM5_PROBE="${MM5_PROBE:-0}" \
  -e C39_CAPTURE="${C39_CAPTURE:-}" \
  -e C39_OUT="${C39_OUT:-/workspace/ferox_isaac/c39_out}" \
  -e MM5="${MM5:-0}" \
  -e MM5_OBJECT="${MM5_OBJECT:-soup_can}" \
  -e MM5_TRIALS="${MM5_TRIALS:-20}" \
  -e MM5_SEED="${MM5_SEED:-20260820}" \
  -e MM5_CHEAT_ATTACH="${MM5_CHEAT_ATTACH:-0}" \
  -e MM5_FIX_BASE="${MM5_FIX_BASE:-0}" \
  -e MM5_OUT="${MM5_OUT:-/workspace/ferox_isaac/mm5_out}" \
  -e MM5_SURFACE="${MM5_SURFACE:-table}" \
  -e MM5_PLACE_FROM_WORKSPACE="${MM5_PLACE_FROM_WORKSPACE:-0}" \
  -e MM5_MEASURE_HAND="${MM5_MEASURE_HAND:-0}" \
  -e MM5_TARGET_R="${MM5_TARGET_R:-0.315}" \
  -e MM5_PELVIS_Z="${MM5_PELVIS_Z:-0.80}" \
  -e TWIN_HAND_KG="${TWIN_HAND_KG:-}" \
  -e TWIN_HAND_KP="${TWIN_HAND_KP:-5.0}" \
  -e TWIN_HAND_KD="${TWIN_HAND_KD:-0.1}" \
  -e G1_CONTROL="${G1_CONTROL:-policy}" \
  -e G1_HANDOFF_S="${G1_HANDOFF_S:-10}" \
  -e G1_HANDOFF_BLEND_S="${G1_HANDOFF_BLEND_S:-1.0}" \
  -e G1_LL_DEX3_APPLY="${G1_LL_DEX3_APPLY:-1}" \
  -e G1_LL_HAND_KP="${G1_LL_HAND_KP:-0}" \
  -e G1_HANDOFF_TO_NOMINAL="${G1_HANDOFF_TO_NOMINAL:-1}" \
  -e G1_PD_HZ="${G1_PD_HZ:-500}" \
  -e G1_CMD_TIMEOUT_MS="${G1_CMD_TIMEOUT_MS:-85}" \
  -e G1_LL_REPORT_STEPS="${G1_LL_REPORT_STEPS:-5000}" \
  -e G1_LL_TRACE="${G1_LL_TRACE:-0}" \
  -e G1_LL_HOLD_POSE="${G1_LL_HOLD_POSE:-spawn}" \
  -e G1_LL_PD="${G1_LL_PD:-explicit}" \
  -e G1_LL_PD_PROBE="${G1_LL_PD_PROBE:-0}" \
  -e G1_LL_HOLD_KINEMATIC="${G1_LL_HOLD_KINEMATIC:-0}" \
  -e G1_LL_GT_TRACE="${G1_LL_GT_TRACE:-0}" \
  -e G1_LL_RIG_RELEASE_AT_S="${G1_LL_RIG_RELEASE_AT_S:-0}" \
  -e G1_LL_ANKLE_TRACE="${G1_LL_ANKLE_TRACE:-0}" \
  -e G1_LL_CONTACT_REPORT="${G1_LL_CONTACT_REPORT:-0}" \
  -e G1_LL_RIG_TILT_DEG="${G1_LL_RIG_TILT_DEG:-0}" \
  -e G1_LL_RIG_TILT_START_S="${G1_LL_RIG_TILT_START_S:-8}" \
  -e G1_LL_RIG_TILT_RAMP_S="${G1_LL_RIG_TILT_RAMP_S:-6}" \
  -e G1_PHYSICS_HZ="${G1_PHYSICS_HZ:-1000}" \
  -e G1_LL_FIX_BASE="${G1_LL_FIX_BASE:-0}" \
  -e G1_LL_RIG_RELEASE_S="${G1_LL_RIG_RELEASE_S:-3}" \
  -e G1_LL_RIG_LIFT_M="${G1_LL_RIG_LIFT_M:-0}" \
  -e G1_LL_RIG_YAW="${G1_LL_RIG_YAW:-}" \
  "$SIM_CONTAINER" bash -c "
  cd /workspace/ferox_isaac && \
  /isaac-sim/python.sh run.py \
    --robot_type $ROBOT \
    --cmd_vel_topic $SIM_CMD_VEL_TOPIC \
    --ros_namespace /ferox/${ROBOT_ID} \
    --no_keyboard \
    $TWIN_RENDER_ARG \
    > /tmp/sim.log 2>&1
"

# Larger scenes (office, and any big networked USD) can take several minutes on
# a COLD first load while the world streams over the network. Wait up to
# SIM_BOOT_TIMEOUT seconds (default 600 = 10 min) so the bigger world does not
# false-fail this readiness check. Override with SIM_BOOT_TIMEOUT=<seconds>.
SIM_BOOT_TIMEOUT="${SIM_BOOT_TIMEOUT:-600}"
echo "  Waiting for sim main loop (cold first load of a large world can take"
echo "  several minutes; timeout ${SIM_BOOT_TIMEOUT}s)..."
_waited=0
_booted=0
while [ "$_waited" -lt "$SIM_BOOT_TIMEOUT" ]; do
  sleep 5
  _waited=$((_waited + 5))
  # -F: the marker is a literal string, not a regex (robust if run.py's
  # PANTHERA-MARK line is ever reformatted).
  if docker exec "$SIM_CONTAINER" bash -c 'grep -qF "before runner.run()" /tmp/sim.log 2>/dev/null'; then
    echo "  ✓ main loop reached at ${_waited}s"
    _booted=1
    break
  fi
  echo "  ...still booting (${_waited}s)"
done

# Surface a stall/crash LOUDLY instead of falling through to the success
# banner. run.py is launched detached (docker exec -d), so a Python failure —
# e.g. _resolve_world raising on an unknown/unreachable SIM_WORLD — surfaces
# here only as the marker never appearing. Tail the log so the reason is
# visible immediately. Keep exit 0 (don't trip set -e); the operator decides.
if [ "$_booted" -ne 1 ]; then
  echo ""
  echo "  ⚠ Sim did NOT reach its main loop within ${SIM_BOOT_TIMEOUT}s."
  echo "    Last lines of /tmp/sim.log (check SIM_WORLD + asset reachability):"
  docker exec "$SIM_CONTAINER" bash -c 'tail -30 /tmp/sim.log 2>/dev/null' || true
fi

echo ""
echo "==============================================="
echo " Sim started."
echo "==============================================="
echo ""
echo " Verify topics flowing on the host (or from any ROS container):"
echo "   ros2 topic hz /scan"
echo "   ros2 topic hz /odom"
echo ""
echo " Logs:    docker exec $SIM_CONTAINER tail -50 /tmp/sim.log"
echo " Next:    ./scripts/02_start_ferox.sh"
echo ""
