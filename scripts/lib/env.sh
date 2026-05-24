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

# ---- ROS / DDS ----
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

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
