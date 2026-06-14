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
#   Default world is NVIDIA's built-in Office. Override with SIM_WORLD=<name>:
#       SIM_WORLD=office       ./01_start_sim.sh   # default
#       SIM_WORLD=hospital     ./01_start_sim.sh   # NVIDIA Hospital env
#       SIM_WORLD=dso_block_a  ./01_start_sim.sh   # the original warehouse world
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
echo "  ROBOT=$ROBOT   SIM_WORLD=${SIM_WORLD:-office}"
# Subscribe directly to /ferox/<robot_id>/cmd_vel — matches what Nav2
# publishes inside its namespace, no relay needed. Avoids the QoS war
# that occurs when multiple Nav2 publishers (volatile + transient_local)
# share a relayed topic with manual `ros2 topic pub` clients.
SIM_CMD_VEL_TOPIC="/ferox/${ROBOT_ID}/cmd_vel"
# SIM_WORLD selects the environment USD (default office); run.py reads it from
# the env. docker exec does not inherit the host env, so pass it explicitly.
docker exec -d \
  -e FEROX_SIM_TEST_PROPS="${FEROX_SIM_TEST_PROPS:-0}" \
  -e SIM_WORLD="${SIM_WORLD:-office}" \
  "$SIM_CONTAINER" bash -c "
  cd /workspace/ferox_isaac && \
  /isaac-sim/python.sh run.py \
    --robot_type $ROBOT \
    --cmd_vel_topic $SIM_CMD_VEL_TOPIC \
    --ros_namespace /ferox/${ROBOT_ID} \
    --no_keyboard \
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
