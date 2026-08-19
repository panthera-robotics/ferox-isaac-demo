#!/usr/bin/env bash
# MM1b — retrain the G1 omni locomotion policy (Ferox-G1-29dof-Velocity-v2).
#
# Runs detached. The retrain is the campaign's critical path but nothing else waits
# on it, so it must not hold a terminal: MM3 and MM4 proceed alongside it.
#
#   ./scripts/mm1b_train.sh [max_iterations] [num_envs]
set -euo pipefail
cd "$(dirname "$0")/.."
REPO="$PWD"

ITERS="${1:-6000}"
ENVS="${2:-2048}"          # Mohammed's cap for the 16 GB box
RUN="mm1b_g1_omni_v2"
LOGDIR="$REPO/isaac/training/logs"
mkdir -p "$LOGDIR" && chmod 777 "$LOGDIR"

# unitree_rl_lab reads the robot USD from UNITREE_MODEL_DIR, which upstream ships as
# the literal placeholder "path/to/unitree_model". MM1b must train on the TWIN USD --
# hands attached, real mass -- so the twin asset is staged into the layout the
# config expects rather than the config being edited to point at ours.
STAGE=/tmp/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0
mkdir -p "$STAGE"
cp -f "$REPO/isaac/assets/g1_dex5/"*.usd "$STAGE/" 2>/dev/null || true
if [ -f "$REPO/isaac/assets/g1_dex5/g1_dex5_1p.usd" ]; then
  cp -f "$REPO/isaac/assets/g1_dex5/g1_dex5_1p.usd" \
        "$STAGE/g1_29dof_rev_1_0.usd"
fi
cp -rf "$REPO/isaac/assets/g1_dex5/configuration" "$STAGE/" 2>/dev/null || true
chmod -R 777 /tmp/unitree_model
echo "[mm1b] staged twin USD -> $STAGE/g1_29dof_rev_1_0.usd"
ls -la "$STAGE/g1_29dof_rev_1_0.usd"

docker rm -f mm1b_train >/dev/null 2>&1 || true
docker run -d --name mm1b_train --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e PYTHONDONTWRITEBYTECODE=1 \
  -e PYTHONPATH=/workspace/ferox_training \
  -v /root/panthera/ref/IsaacLab:/workspace/IsaacLab \
  -v /root/panthera/ref/unitree_rl_lab:/workspace/unitree_rl_lab \
  -v "$REPO/isaac/training":/workspace/ferox_training \
  -v "$LOGDIR":/workspace/logs \
  -v /tmp/unitree_model:/tmp/unitree_model \
  -v "$REPO/cache/kit":/isaac-sim/kit/cache \
  -v "$REPO/cache/ov":/isaac-sim/.cache/ov \
  ferox/isaaclab:2.3.2 \
  bash -lc "
    cd /workspace/unitree_rl_lab
    # Import our overlay first so the v2 task is registered before train.py resolves it.
    export FEROX_TASK_MODULE=mm1b_g1_omni_v2
    /isaac-sim/python.sh -c \"import mm1b_g1_omni_v2\" 2>/dev/null || true
    /isaac-sim/python.sh scripts/rsl_rl/train.py \
      --task Ferox-G1-29dof-Velocity-v2 \
      --num_envs $ENVS --max_iterations $ITERS --headless \
      --logger tensorboard 2>&1 | tee /workspace/logs/mm1b_train.log
  "
echo "[mm1b] launched detached as container 'mm1b_train'"
echo "[mm1b]   progress: docker logs -f mm1b_train  |  tail $LOGDIR/mm1b_train.log"
