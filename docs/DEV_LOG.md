# Dev Log

Append-only log of demo-level changes that aren't obvious from `git log`
alone — guards, conventions, recurring footguns. Keep entries terse;
link to PRs / files for detail.

---

## 2026-05-09 — Sim/nav robot-mismatch guard

Added a pre-flight check to [scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh)
that compares `$ROBOT` against the robot type the sim is actually
running and aborts with a fix-suggestion if they differ.

**Why:** second occurrence of the same class of bug — sim launched with
`ROBOT=g1`, nav stack launched without the env var, nav defaulted to
`go2`, so Nav2 published cmd_vel to `/ferox/go2_01/cmd_vel` while the
sim was subscribed to `/ferox/g1_01/cmd_vel`. Robot stays still, no
errors anywhere obvious — `nav.log` looks healthy because Nav2 thinks
it's running fine. Cheap to detect, expensive to debug.

**How:** [scripts/01_start_sim.sh](../scripts/01_start_sim.sh) writes
`$ROBOT` to `/tmp/sim_robot_type` inside the sim container right after
the container comes up. The guard in `02_start_ferox.sh` reads it via
`docker exec ... cat`. Single-line plaintext value — no parsing risk
versus grepping `sim.log`.

**Bypass:** `FEROX_SKIP_SIM_CHECK=1 ./scripts/02_start_ferox.sh` for
multi-host setups where sim and nav run on different machines and
there's no shared `ferox_isaac_sim` container to query. Don't use it
on a single host — it just hides the bug it's meant to catch.

**Skip-on-missing-tag:** if the tag file isn't there (e.g. sim was
started by a build that predates this guard), the guard silently
proceeds. Stale or missing tags never block a valid run; only an
explicit mismatch aborts.

---

## 2026-05-24 — env-driven Cyclone DDS (alignment with Ferox 1.1 / ferox-speech 1.2)

Wired the demo into the same env-driven DDS pattern as the sibling
repos. Two optional env vars — `FEROX_DDS_INTERFACE` and
`FEROX_DDS_PEERS` — drive the cyclone XML; both empty means
auto-detect interface + multicast on the local LAN.

Mechanism differs from Ferox/ferox-speech because the Isaac Sim
image isn't ours to modify. Instead of baking `CYCLONEDDS_URI` into
the image's `Config.Env`, we render the cyclone XML on the host with
[scripts/lib/render_cyclone.sh](../scripts/lib/render_cyclone.sh),
mount it into the sim container, and pass
`CYCLONEDDS_URI=file:///tmp/cyclonedds.xml` via `docker run -e`. New
template at [cyclone/cyclonedds.xml.template](../cyclone/cyclonedds.xml.template).

[scripts/lib/env.sh](../scripts/lib/env.sh) now sources a repo-root
`.env` (gitignored) if present, so the same vars also reach the
downstream Ferox compose-up triggered by
[scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh) — no
changes to that script needed. Caller-set env wins over the .env
file: a naïve `set -a; . file; set +a` overrides unconditionally, so
we read each line and only export if the var is currently unset.

Single-machine sim (sim + nav co-located, no Tailscale) needs NO env
vars — multicast on the local LAN/loopback works.

See [.env.example](../.env.example) for setup scenarios.
