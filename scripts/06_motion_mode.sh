#!/bin/bash
# ferox-isaac-demo — runtime G1 motion-mode toggle (no nav restart).
#
# Flips the RUNNING ferox_nav controller (MPPI + velocity_smoother) between:
#
#   walk : DEMO LOCK — clean forward-only walk. No reverse, minimal strafe
#          (vy<=0.1), PreferForwardCritic ON. Goals must be placed AHEAD of the
#          robot's heading (it has no yaw to turn). This is the committed default
#          in g1_nav2.yaml, so a nav restart always boots back to 'walk'.
#
#   omni : holonomic — reaches side / off-axis goals by STRAFING (vy 0.3) and
#          reversing (vx -0.2), PreferForwardCritic OFF. This visibly crabs;
#          use it when you need to send side goals, not for the clean demo.
#
# Changes are live ros2 param sets only (lost on nav restart -> back to 'walk').
# G1 only. Touches nothing else (Go2, policy, cmd_vel wiring, arbiter untouched).
#
# Usage:
#   ./06_motion_mode.sh walk     # clean forward walk (goals ahead)
#   ./06_motion_mode.sh omni     # strafe/reverse to side goals (crabs)
#   ./06_motion_mode.sh status   # show current live values
set -e

export ROBOT="${ROBOT:-g1}"           # this toggle is G1-only
source "$(dirname "$0")/lib/env.sh"

CS="/ferox/$ROBOT_ID/controller_server"
VS="/ferox/$ROBOT_ID/velocity_smoother"
MODE="${1:-status}"

run() { docker exec "$NAV_CONTAINER" bash -lc "
  source /opt/ros/humble/setup.bash
  source /workspace/install/setup.bash 2>/dev/null
  $1"; }

case "$MODE" in
  walk)  VX_MIN=0.0;  VY_MAX=0.1; VY_STD=0.1; PREFER=true
         SM_MAX='[0.8, 0.1, 0.6]'; SM_MIN='[0.0, -0.1, -0.6]' ;;
  omni)  VX_MIN=-0.2; VY_MAX=0.3; VY_STD=0.2; PREFER=false
         SM_MAX='[0.8, 0.3, 0.6]'; SM_MIN='[-0.2, -0.3, -0.6]' ;;
  status)
    run "
      echo '  vx_min        =' \$(ros2 param get $CS FollowPath.vx_min 2>/dev/null | tail -1)
      echo '  vy_max        =' \$(ros2 param get $CS FollowPath.vy_max 2>/dev/null | tail -1)
      echo '  PreferForward =' \$(ros2 param get $CS FollowPath.PreferForwardCritic.enabled 2>/dev/null | tail -1)
      echo '  smoother max  =' \$(ros2 param get $VS max_velocity 2>/dev/null | tail -1)
      echo '  smoother min  =' \$(ros2 param get $VS min_velocity 2>/dev/null | tail -1)"
    echo "Usage: $0 [walk|omni|status]"
    exit 0 ;;
  *) echo "Usage: $0 [walk|omni|status]"; exit 1 ;;
esac

run "
  ros2 param set $CS FollowPath.vx_min $VX_MIN >/dev/null
  ros2 param set $CS FollowPath.vy_max $VY_MAX >/dev/null
  ros2 param set $CS FollowPath.vy_std $VY_STD >/dev/null
  ros2 param set $CS FollowPath.PreferForwardCritic.enabled $PREFER >/dev/null
  ros2 param set $VS max_velocity '$SM_MAX' >/dev/null
  ros2 param set $VS min_velocity '$SM_MIN' >/dev/null
"
echo "G1 motion mode -> $MODE"
[ "$MODE" = walk ] && echo "  clean forward walk; place goals AHEAD of the robot's heading."
[ "$MODE" = omni ] && echo "  holonomic; side/off-axis goals reachable by strafing (will crab)."
echo "  (runtime override; a nav restart reverts to the committed 'walk' default.)"
