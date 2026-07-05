#!/bin/bash
# Auto-detect the live desktop X server (display number, owning user, Xauthority)
# so the sim viewport renders into whatever GUI session is actually running —
# DCV, VNC, or a physical seat — instead of hardcoded values that only matched
# one host.
#
# Why this exists: DCV/GDM assign the display number (:0 vs :1), the owning user
# (`ubuntu` vs `user`), and the Xauthority path differently across hosts AND
# across REBOOTS of the same host. On a fresh boot the only X server is the GDM
# login greeter (runs as user `gdm`); the real desktop session — the one the sim
# must render into — only appears on a *different* display AFTER a user logs in
# via the DCV/VNC client. Cookies stashed in /tmp are wiped on reboot too. So we
# detect at runtime and mint a fresh, world-readable cookie every run.
#
# Sets + exports: HOST_DISPLAY, DESKTOP_USER, XAUTH_FILE.
# Explicit overrides win: set DISPLAY + DESKTOP_USER + XAUTH_FILE in the shell to
# bypass detection entirely.

# Read xauth entries for a display from a source file, using sudo only if the
# file isn't directly readable (GDM cookies are 0700-owned by the desktop user).
_ferox_xauth_extract() {  # $1=src file  $2=display  -> prints xauth entries
  if [ -r "$1" ]; then
    xauth -f "$1" extract - "$2" 2>/dev/null
  else
    sudo -n xauth -f "$1" extract - "$2" 2>/dev/null
  fi
}

ferox_detect_display() {
  local cookie="/tmp/.ferox.xauth"

  # 1) Explicit override: caller set all three — trust them, skip detection.
  if [ -n "${DISPLAY:-}" ] && [ -n "${XAUTH_FILE:-}" ] && [ -n "${DESKTOP_USER:-}" ]; then
    HOST_DISPLAY="$DISPLAY"
    export HOST_DISPLAY DESKTOP_USER XAUTH_FILE
    return 0
  fi

  # 2) Find the X11 socket owned by a real logged-in user (uid >= 1000). The DCV/
  #    GNOME desktop session owns /tmp/.X11-unix/X<n>; the GDM greeter's socket is
  #    owned by user `gdm` (uid < 1000) and is skipped — rendering into the greeter
  #    would vanish the instant someone logs in.
  local sock uid uname disp="" duser=""
  for sock in /tmp/.X11-unix/X*; do
    [ -S "$sock" ] || continue
    uid="$(stat -c '%u' "$sock" 2>/dev/null)" || continue
    [ "$uid" -ge 1000 ] 2>/dev/null || continue
    uname="$(id -un "$uid" 2>/dev/null)" || continue
    disp=":${sock##*/X}"
    duser="$uname"
    break
  done

  if [ -z "$disp" ]; then
    # Only the login greeter (or nothing) is running — no session to render into.
    echo "[x11] No logged-in desktop X session found (only the GDM login screen)." >&2
    echo "[x11]   -> Open your DCV/VNC client, LOG IN to the desktop, then re-run." >&2
    echo "[x11]   (falling back to headless for now)" >&2
    HOST_DISPLAY="${DISPLAY:-:0}"
    DESKTOP_USER="${DESKTOP_USER:-$(id -un)}"
    XAUTH_FILE="${XAUTH_FILE:-/nonexistent}"
    export HOST_DISPLAY DESKTOP_USER XAUTH_FILE
    return 0
  fi

  HOST_DISPLAY="${DISPLAY:-$disp}"
  DESKTOP_USER="${DESKTOP_USER:-$duser}"
  echo "[x11] desktop session: DISPLAY=$HOST_DISPLAY user=$DESKTOP_USER" >&2

  # 3) Locate that user's real Xauthority. GDM keeps it under /run/user/<uid>/gdm/;
  #    classic setups use ~/.Xauthority.
  local src="" cand
  for cand in "/run/user/$uid/gdm/Xauthority" "$(getent passwd "$uid" | cut -d: -f6)/.Xauthority"; do
    if [ -r "$cand" ] || sudo -n test -r "$cand" 2>/dev/null; then src="$cand"; break; fi
  done

  # 4) Mint a fresh world-readable cookie the container (uid 1234) can authenticate
  #    with. Regenerate every run because /tmp is wiped on reboot.
  #
  #    Force-clear any stale path first. A prior HEADLESS run can leave a
  #    ROOT-OWNED DIRECTORY here: docker bind-mounts `-v $XAUTH_FILE:...`, and when
  #    the source file is missing at `docker run` time Docker creates the target as
  #    a root:root directory — which then blocks xauth from ever writing the cookie.
  if [ -e "$cookie" ] && { [ -d "$cookie" ] || [ ! -w "$cookie" ]; }; then
    sudo -n rm -rf "$cookie" 2>/dev/null || rm -rf "$cookie" 2>/dev/null
  fi
  rm -f "$cookie" 2>/dev/null
  if [ -n "$src" ] && _ferox_xauth_extract "$src" "$HOST_DISPLAY" | xauth -f "$cookie" merge - 2>/dev/null && [ -s "$cookie" ]; then
    chmod 644 "$cookie" 2>/dev/null
    XAUTH_FILE="$cookie"
  else
    # Couldn't build a cookie; fall back to the source path (xhost +local: in the
    # caller still lets local container connections through).
    XAUTH_FILE="${src:-/nonexistent}"
  fi

  export HOST_DISPLAY DESKTOP_USER XAUTH_FILE
}
