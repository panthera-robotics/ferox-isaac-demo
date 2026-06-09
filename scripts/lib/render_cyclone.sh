#!/usr/bin/env bash
#
# Renders the Isaac Sim cyclone DDS config on the host and prints the
# rendered file path to stdout (one line, suitable for command substitution).
#
# FEROX_DDS_INTERFACE  pins the network interface (else auto-detect).
# FEROX_DDS_PEERS      the <Peers> list. Normally tailnet-derived + exported
#                      by scripts/lib/env.sh before this runs; if unset (e.g.
#                      this script invoked standalone) it is derived here via
#                      scripts/lib/dds_peers.sh. Only currently-online tailnet
#                      nodes are listed, so a dead host is never a <Peer> and
#                      the ddsi_udp_conn_write flood can't recur. Set it
#                      explicitly to override ("" => multicast only).
#
# Usage:
#   RENDERED_PATH=$(scripts/lib/render_cyclone.sh)
#   docker run ... \
#     -v "$RENDERED_PATH":/tmp/cyclonedds.xml:ro \
#     -e CYCLONEDDS_URI=file:///tmp/cyclonedds.xml \
#     ...

set -e

# Resolve paths relative to this script — works regardless of caller's CWD.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEMPLATE="$REPO_ROOT/cyclone/cyclonedds.xml.template"

# Render output: per-repo, /tmp-style ephemeral path. Gitignored.
RENDER_DIR="/tmp/ferox-isaac-demo"
mkdir -p "$RENDER_DIR"
RENDERED="$RENDER_DIR/cyclonedds.xml"

# Interface block: pin if FEROX_DDS_INTERFACE set, else auto-detect.
if [[ -n "${FEROX_DDS_INTERFACE}" ]]; then
  export CYCLONE_INTERFACE_BLOCK="<NetworkInterface name=\"${FEROX_DDS_INTERFACE}\" presence_required=\"true\" />"
else
  export CYCLONE_INTERFACE_BLOCK="<NetworkInterface autodetermine=\"true\" />"
fi

# Peers block. The list is normally tailnet-derived and exported by
# scripts/lib/env.sh before this runs. If invoked standalone (FEROX_DDS_PEERS
# unset), derive it here too so the rendered XML never lists a stale/dead host.
# An explicit FEROX_DDS_PEERS (including "") is honored verbatim.
if [[ -z "${FEROX_DDS_PEERS+set}" ]]; then
  . "$(dirname "${BASH_SOURCE[0]}")/dds_peers.sh"
  FEROX_DDS_PEERS="$(ferox_derive_dds_peers || true)"
fi
export CYCLONE_PEERS_BLOCK=""
for peer in ${FEROX_DDS_PEERS}; do
  CYCLONE_PEERS_BLOCK+="<Peer Address=\"${peer}\"/>"$'\n        '
done

envsubst < "$TEMPLATE" > "$RENDERED"

# Diagnostic to stderr so stdout stays clean for command substitution.
echo "[cyclone] interface: ${FEROX_DDS_INTERFACE:-<auto>}" >&2
echo "[cyclone] peers:     ${FEROX_DDS_PEERS:-<none, multicast only>}" >&2
echo "[cyclone] rendered:  $RENDERED" >&2

# Stdout: the file path, nothing else.
echo "$RENDERED"
