#!/usr/bin/env bash
#
# Tailnet-derived Cyclone DDS peer list (host-side).
#
# Defines ferox_derive_dds_peers(): echoes a space-separated, de-duplicated
# list of peer IPs for the <Peers> block of cyclonedds.xml, derived from the
# tailnet at startup so NOTHING is hardcoded.
#
# Why derive instead of hardcode:
#   Cyclone sprays SPDP across the participant-index range at every <Peer>.
#   A dead peer => repeating "ddsi_udp_conn_write to udp/<ip>:<port> failed"
#   (retcode -3) flooding the logs. By listing ONLY currently-online tailnet
#   nodes, an unreachable host is never listed — the flood is removed by
#   construction (offline => not .Online => never a peer). And the self entry
#   tracks `tailscale ip -4` instead of a stale literal, so it survives a
#   fresh (ephemeral-IP) Vast.ai instance.
#
# Rules:
#   - This node's own tailscale IP (`tailscale ip -4`) is ALWAYS included —
#     same-host sim+nav+speech discovery needs it because multicast is off
#     over the tunnel (see docs/NEXT_SESSION.md).
#   - Remote peers = currently-online tailnet nodes from
#     `tailscale status --json` (.Peer[] | select(.Online)).
#   - If tag:ferox is in use anywhere on the tailnet, remote peers are scoped
#     to online nodes carrying tag:ferox; if no node carries it, fall back to
#     ALL online peers. Works both ways.
#   - Degrades loudly (never a silent empty list): if tailscale/jq are missing
#     or tailscaled is down, it warns on stderr and emits whatever it can
#     (self-IP, or nothing => multicast-only).
#
# Diagnostics go to stderr; the peer list is the only thing on stdout, so the
# function is safe inside command substitution.
ferox_derive_dds_peers() {
  if ! command -v tailscale >/dev/null 2>&1; then
    echo "[dds] WARNING: 'tailscale' not found — cannot derive peers; multicast-only. Set FEROX_DDS_PEERS to override." >&2
    return 0
  fi
  if ! command -v jq >/dev/null 2>&1; then
    echo "[dds] WARNING: 'jq' not found — cannot derive peers; multicast-only. Set FEROX_DDS_PEERS to override." >&2
    return 0
  fi

  local self status tags_in_use remote
  # This node's own tailscale IP — always included for same-host discovery.
  self="$(tailscale ip -4 2>/dev/null | head -n1 || true)"
  [ -z "$self" ] && echo "[dds] WARNING: 'tailscale ip -4' returned nothing (tailscaled down?) — self-peer omitted." >&2

  status="$(tailscale status --json 2>/dev/null || true)"
  if [ -z "$status" ]; then
    echo "[dds] WARNING: 'tailscale status --json' empty — no remote peers derived." >&2
    [ -n "$self" ] && printf '%s\n' "$self"
    return 0
  fi

  # tag:ferox in use anywhere on the tailnet? Then scope remote peers to
  # online tag:ferox nodes; otherwise fall back to every online peer.
  tags_in_use="$(printf '%s' "$status" | jq -r 'any(.Peer[]?; (.Tags // []) | index("tag:ferox")) // false' 2>/dev/null || true)"
  if [ "$tags_in_use" = "true" ]; then
    echo "[dds] tag:ferox detected on the tailnet — scoping remote peers to online tagged nodes." >&2
    remote="$(printf '%s' "$status" | jq -r '.Peer[]? | select(.Online == true) | select((.Tags // []) | index("tag:ferox")) | .TailscaleIPs[0] // empty' 2>/dev/null || true)"
  else
    remote="$(printf '%s' "$status" | jq -r '.Peer[]? | select(.Online == true) | .TailscaleIPs[0] // empty' 2>/dev/null || true)"
  fi

  # self + remote, blanks dropped, de-duplicated (order preserved), space-joined.
  printf '%s\n%s\n' "$self" "$remote" | awk 'NF && !seen[$0]++' | paste -sd' ' -
}
