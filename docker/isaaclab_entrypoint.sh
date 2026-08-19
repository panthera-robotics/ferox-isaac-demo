#!/bin/bash
# Install the mounted trees editable, then hand off. Done at start rather than at
# build time because the trees are bind-mounted, not copied into the image.
set -euo pipefail

PY=/isaac-sim/python.sh

if [ -d /workspace/IsaacLab/source/isaaclab ]; then
  for p in isaaclab isaaclab_assets isaaclab_mimic isaaclab_rl isaaclab_tasks; do
    [ -d "/workspace/IsaacLab/source/$p" ] && \
      $PY -m pip install --no-deps -e "/workspace/IsaacLab/source/$p" -q || true
  done
fi
if [ -d /workspace/unitree_rl_lab/source/unitree_rl_lab ]; then
  $PY -m pip install --no-deps -e /workspace/unitree_rl_lab/source/unitree_rl_lab -q || true
fi

exec "$@"
