#!/bin/bash
# C-39 fork runner. SONIC alone from its own nominal stance.
#   $1 = TWIN_HAND_KG ("" = real mass)   $2 = label
# env: MJC=1 lie in MuJoCo's way | SONICFLAGS | PUBHZ | FRIC | PHYSHZ | ARM
set -u
SP=/tmp/claude-0/-root-panthera/459f4445-57a6-40cb-97cd-0dfccf354e94/scratchpad
cd ~/panthera/ferox-isaac-demo
docker rm -f mm3_bridge mm4_sonic mm4_drive ferox_isaac_sim >/dev/null 2>&1
sleep 2
nohup env ROBOT=g1 TWIN=1 TWIN_CAMERA=0 HAND=dex5_1p SIM_WORLD=hospital G1_CONTROL=lowcmd \
  TWIN_ARMATURE="${ARM:-0.01}" TWIN_JOINT_FRICTION="${JFRIC:-}" G1_PD_HZ="${PHYSHZ:-1000}" G1_PHYSICS_HZ="${PHYSHZ:-1000}" \
  TWIN_CONTACT_MATERIAL="${FRIC:-0}" \
  G1_LL_FIX_BASE=until_commanded G1_LL_RIG_RELEASE_S=20 \
  G1_LL_HOLD_POSE=sonic G1_LL_RIG_LIFT_M="${RIGLIFT:--0.0665}" G1_LL_RIG_YAW=0 G1_LL_HAND_KP=60 G1_LL_DEX3_APPLY=0 \
  G1_LL_PD_PROBE="${PDPROBE:-0}" G1_LL_HOLD_KINEMATIC="${HOLDK:-0}"  G1_LL_REPORT_STEPS="${REPS:-2500}" TWIN_HAND_KG="$1" \
  bash scripts/01_start_sim.sh > $SP/sim_fork.log 2>&1 &
until docker exec ferox_isaac_sim grep -q "entering main loop" /tmp/sim.log 2>/dev/null; do sleep 3; done

docker run -d --name mm3_bridge --network host --ipc host --user 1234:1234 \
  -e G1_LL_MUJOCO_COMPAT="${MJC:-0}" \
  -v $PWD/isaac/twin/lowlevel_bridge:/bridge:ro ferox/twin-lowlevel:humble \
  python3 /bridge/dds_side.py --domain 0 --iface lo --publish-hz "${PUBHZ:-1041.68}" >/dev/null 2>&1

docker run -d --name mm4_sonic --network host --gpus all \
  -v trt_policy:/opt/gear_sonic_deploy/policy/sonic_v1_1 \
  -v trt_planner:/opt/gear_sonic_deploy/planner/target_vel/V2 \
  ferox/sonic-deploy:v1.1-x86_64 \
  /opt/gear_sonic_deploy/target/release/g1_deploy_onnx_ref \
    lo policy/sonic_v1_1/model_decoder.onnx reference/example/ \
    --obs-config policy/sonic_v1_1/observation_config.yaml \
    --encoder-file policy/sonic_v1_1/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type zmq_manager --output-type all --zmq-host localhost --zmq-port 5556 \
    ${SONICFLAGS:-} >/dev/null 2>&1
until docker logs mm4_sonic 2>&1 | grep -q "Init Done"; do sleep 5; done

nohup docker run --rm --name mm4_drive --network host \
  -v ~/panthera/ref/upstream/GR00T-WholeBodyControl:/upstream:ro -e PYTHONPATH=/upstream \
  -v $PWD/scripts:/scripts:ro ferox/twin-lowlevel:humble \
  python3 /scripts/mm4_sonic_drive.py --bind 'tcp://*:5556' --hold-s 60 > $SP/fork_drive.log 2>&1 &

until docker exec ferox_isaac_sim grep -q "RIG RELEASED" /tmp/sim.log 2>/dev/null || \
      docker logs mm4_sonic 2>&1 | grep -q "Stopping control"; do sleep 4; done
sleep 45
echo "### $2  (TWIN_HAND_KG=${1:-REAL} MJC=${MJC:-0} FRIC=${FRIC:-def} PHYSHZ=${PHYSHZ:-1000} ARM=${ARM:-0.01})"
docker exec ferox_isaac_sim bash -c 'grep -E "DIAGNOSTIC hand mass|GROUND FRICTION" /tmp/sim.log | head -3'
docker exec ferox_isaac_sim bash -c 'grep -E "^\[lowlevel-sim\] t=" /tmp/sim.log | tail -2'
docker logs mm3_bridge 2>&1 | grep -E "lowcmd|crc_bad" | tail -2
