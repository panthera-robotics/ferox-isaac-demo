#!/usr/bin/env bash
#
# Renders the Isaac Sim cyclone DDS config on the host, using the two
# optional FEROX_DDS_* env vars, and prints the rendered file path to
# stdout (one line, suitable for command substitution).
#
# Defaults are robot-LAN safe (no env vars => auto-detect + multicast).
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

# Peers block: empty unless FEROX_DDS_PEERS set (space-separated IPs).
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
