#!/bin/bash
# Shared env for the ferox-isaac demo. Sourced by every script.
# Override any value via environment before running, e.g.
#     ROBOT=g1 ./01_start_sim.sh

# ---- Optional .env file (loads FEROX_DDS_* and any other overrides) ----
# Source a .env at the repo root if present, but DO NOT clobber values
# that the caller already set in the shell environment. We read each
# KEY=VAL line ourselves so a caller-set value wins over the .env
# default — `set -a; . file; set +a` would overwrite unconditionally.
ENV_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/.env"
if [ -f "$ENV_FILE" ]; then
  while IFS= read -r _line || [ -n "$_line" ]; do
    _line="${_line%$'\r'}"
    _line="${_line#"${_line%%[![:space:]]*}"}"
    [ -z "$_line" ] && continue
    [ "${_line:0:1}" = "#" ] && continue
    [[ "$_line" != *=* ]] && continue
    _key="${_line%%=*}"
    _val="${_line#*=}"
    [[ "$_val" == \"*\" ]] && _val="${_val#\"}" && _val="${_val%\"}"
    [[ "$_val" == \'*\' ]] && _val="${_val#\'}" && _val="${_val%\'}"
    [ -z "${!_key+set}" ] && export "$_key=$_val"
  done < "$ENV_FILE"
  unset _line _key _val
fi

# ---- Repo locations ----
DEMO_DIR="${DEMO_DIR:-$HOME/panthera/ferox-isaac-demo}"
# Ferox repo layout: the repo root holds docker/, src/, install/. Older
# layouts nested under Ferox/ferox/ — fall back to that if the flat layout
# isn't there, so existing checkouts keep working.
if [ -z "${FEROX_REPO:-}" ]; then
  if   [ -d "$HOME/panthera/Ferox/src" ];        then FEROX_REPO="$HOME/panthera/Ferox"
  elif [ -d "$HOME/panthera/Ferox/ferox/src" ];  then FEROX_REPO="$HOME/panthera/Ferox/ferox"
  else FEROX_REPO="$HOME/panthera/Ferox"
  fi
fi

# ---- Robot ----
ROBOT="${ROBOT:-go2}"            # go2 | g1
ROBOT_ID="${ROBOT_ID:-${ROBOT}_01}"
VENUE="${VENUE:-}"               # empty → SLAM mapping mode

# ---- G1 locomotion policy source-of-truth (ferox-g1-locomotion) ----
# The G1 velocity policy is maintained in its own repo for reuse/retraining.
# If that repo is checked out alongside this one, the sim sources the policy
# from it (single source of truth) by overlay-mounting its policy/ dir onto the
# G1 checkpoint slot in 01_start_sim.sh. If absent, the sim falls back to the
# checkpoint bundled in isaac/checkpoints/g1. Override with G1_POLICY_DIR=<dir>
# (must contain exported/policy.pt + params/{env,deploy}.yaml). Empty => bundled.
if [ -z "${G1_POLICY_DIR+set}" ]; then
  _g1_src="$HOME/panthera/ferox-g1-locomotion/policy"
  if [ -f "$_g1_src/exported/policy.pt" ]; then G1_POLICY_DIR="$_g1_src"; else G1_POLICY_DIR=""; fi
  unset _g1_src
fi
export G1_POLICY_DIR

# ---- ROS / DDS ----
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# DDS peer list: tailnet-derived, NOT hardcoded. This exported value flows to
# BOTH the sim (render_cyclone.sh writes the <Peers> block) and the Ferox nav
# stack (docker compose interpolation — shell env wins over Ferox/.env). By
# listing only currently-online tailnet nodes (+ this node's own tailscale IP),
# an unreachable host is never a <Peer>, so the "ddsi_udp_conn_write ... failed"
# flood can't recur on a fresh Vast.ai instance or while the laptop is asleep.
# See scripts/lib/dds_peers.sh and docs/DEV_LOG.md (2026-06-09).
#
# Override: set FEROX_DDS_PEERS in the shell env or .env to bypass derivation
# entirely (explicit "" => no peers, multicast only). Caller/.env value wins.
. "$(dirname "${BASH_SOURCE[0]}")/dds_peers.sh"
if [ -z "${FEROX_DDS_PEERS+set}" ]; then
  FEROX_DDS_PEERS="$(ferox_derive_dds_peers || true)"
  echo "[dds] peers (tailnet-derived): ${FEROX_DDS_PEERS:-<none, multicast only>}" >&2
else
  echo "[dds] peers (FEROX_DDS_PEERS override): ${FEROX_DDS_PEERS:-<none, multicast only>}" >&2
fi

# ---- Container names ----
SIM_CONTAINER="${SIM_CONTAINER:-ferox_isaac_sim}"
NAV_CONTAINER="${NAV_CONTAINER:-ferox_nav}"

# ---- Images ----
ISAAC_IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
FEROX_NAV_IMAGE="${FEROX_NAV_IMAGE:-ferox/nav:humble}"
FEROX_MSGS_IMAGE="${FEROX_MSGS_IMAGE:-ferox/msgs:humble}"

# ---- Cache (Isaac Sim runs as UID 1234 and needs writable cache dirs) ----
CACHE_DIR="${CACHE_DIR:-$DEMO_DIR/cache}"

# ---- X / display (for sim viewport + rviz) ----
HOST_DISPLAY="${DISPLAY:-:0}"
DESKTOP_USER="${DESKTOP_USER:-user}"
XAUTH_FILE="${XAUTH_FILE:-/home/${DESKTOP_USER}/.Xauthority}"

# ---- Validation ----
case "$ROBOT" in
  go2|g1) ;;
  *) echo "ERROR: unknown ROBOT='$ROBOT'. Use go2 or g1." >&2; exit 1 ;;
esac

export DEMO_DIR FEROX_REPO ROBOT ROBOT_ID VENUE
export ROS_DOMAIN_ID RMW_IMPLEMENTATION
export FEROX_DDS_INTERFACE FEROX_DDS_PEERS
export SIM_CONTAINER NAV_CONTAINER
export ISAAC_IMAGE FEROX_NAV_IMAGE FEROX_MSGS_IMAGE
export CACHE_DIR HOST_DISPLAY DESKTOP_USER XAUTH_FILE
