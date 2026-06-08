#!/usr/bin/env bash
# Shared: clear a prior Ferox nav stack from a container before (re)launch.
#
# WHY THIS EXISTS
#   A `ros2 launch` only owns its direct children. Killing just the launch
#   parent (the old `pkill -f "ros2 launch"` idiom) leaves every node it
#   spawned — waypoint_manager, the Nav2 lifecycle nodes, the static TF
#   publishers — alive and reparented to PID 1. A later launch then starts a
#   SECOND copy of each, and two same-named nodes on one DDS domain advertise
#   duplicate endpoints: e.g. two /ferox/<id>/move_to_named action servers,
#   which crosses the speech client's goal/cancel/result responses so the
#   goal resolves `failed` and the robot never moves.
#
#   So the kill pattern MUST match the node executables, not the launch
#   parent. This file is the single source of that pattern so the start
#   scripts and the stop script can't drift apart — they did: the localize
#   start script matched only Nav2 nodes (lifecycle_manager|controller_server|
#   amcl|slam_toolbox|map_server) and missed the ferox_nav nodes + TFs, which
#   is exactly how the duplicate-action-server bug got in.
#
# SAFETY
#   Always a BARE `docker exec <c> pkill ...` — never a wrapping `bash -c`.
#   pkill/pgrep never report their own PID, and with no host- or container-
#   side shell carrying the pattern on its command line, the killer cannot
#   match and kill itself mid-run (the exit-137 trap the inline comments in
#   the start scripts warn about).

# Canonical match for every process a Ferox nav launch spawns. ERE. Path
# fragments (not absolute paths) so it holds whether the workspace is the
# sim bind-mount (/workspace/install/...) or a baked image on real hardware.
#   ros2 launch ferox_nav                the launch parent
#   ferox_nav/lib/                       waypoint_manager, status_publisher, venue_manager
#   /lib/nav2_                           every nav2_* binary (controller, planner,
#                                        bt_navigator, behavior, smoother,
#                                        velocity_smoother, waypoint_follower,
#                                        map_server, amcl, lifecycle_manager)
#   slam_toolbox                         SLAM (sync_slam_toolbox_node)
#   topic_tools/relay                    the sim-bridge relays
#   tf2_ros/static_transform_publisher   the sim static TFs
FEROX_NAV_PROC_RE='ros2 launch ferox_nav|ferox_nav/lib/|/lib/nav2_|slam_toolbox|topic_tools/relay|tf2_ros/static_transform_publisher'

# kill_nav_stack <container> [--verify]
#   Kills the nav stack in <container>. With --verify, returns 1 (and prints
#   the survivors to stderr) if anything still matches after the kill; the
#   caller decides whether to fail-loud. Without --verify, always returns 0.
kill_nav_stack() {
  local container="$1" verify="${2:-}"
  # Bare pkill — see SAFETY. `|| true`: pkill exits 1 when nothing matched,
  # which is a clean state here, not an error (and the scripts run `set -e`).
  docker exec "$container" pkill -9 -f "$FEROX_NAV_PROC_RE" 2>/dev/null || true
  sleep 3
  if [ "$verify" = "--verify" ]; then
    local residual
    residual=$(docker exec "$container" pgrep -af "$FEROX_NAV_PROC_RE" 2>/dev/null || true)
    if [ -n "$residual" ]; then
      echo "  ✗ residual nav processes survived the clean:" >&2
      echo "$residual" | sed 's/^/      /' >&2
      return 1
    fi
  fi
  return 0
}
