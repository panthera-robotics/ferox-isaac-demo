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
# Subscribe directly to /ferox/<robot_id>/cmd_vel — matches what Nav2
# publishes inside its namespace, no relay needed. Avoids the QoS war
# that occurs when multiple Nav2 publishers (volatile + transient_local)
# share a relayed topic with manual `ros2 topic pub` clients.
SIM_CMD_VEL_TOPIC="/ferox/${ROBOT_ID}/cmd_vel"
docker exec -d "$SIM_CONTAINER" bash -c "
  cd /workspace/ferox_isaac && \
  /isaac-sim/python.sh run.py \
    --robot_type $ROBOT \
    --cmd_vel_topic $SIM_CMD_VEL_TOPIC \
    --no_keyboard \
    > /tmp/sim.log 2>&1
"

echo "  Waiting for sim main loop (cold first boot can take 2–3 min)..."
for i in {1..36}; do
  sleep 5
  if docker exec "$SIM_CONTAINER" bash -c 'grep -q "before runner.run()" /tmp/sim.log 2>/dev/null'; then
    echo "  ✓ main loop reached at $((i*5))s"
    break
  fi
  echo "  ...still booting ($((i*5))s)"
done

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
