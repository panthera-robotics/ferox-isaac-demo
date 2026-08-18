# Dev Log

Append-only log of demo-level changes that aren't obvious from `git log`
alone — guards, conventions, recurring footguns. Keep entries terse;
link to PRs / files for detail.

---

## 2026-08-18 — Isaac rounds `rendering_dt` to whole physics substeps, and does not tell you

**What:** The twin's Mid-360 was decimated to 10 Hz by dividing the render rate by the
sensor's scan rate. It came out at **11.11 Hz**, consistently, against a 10 Hz contract.

**Why:** `World(physics_dt=1/200, rendering_dt=1/60)` does not render at 60 Hz. 1/60 =
0.016667 s is not a whole number of 0.005 s physics substeps, so Isaac runs **3** substeps
per render frame — 0.015 s, i.e. **66.67 Hz**. `world.get_rendering_dt()` still returns the
0.016667 you asked for. Dividing by the reported 60 gave decimation step 6, and
66.67 / 6 = 11.11.

**The trap:** the error is small, plausible, and looks like sensor jitter. Nothing warns.
Any rate derived from `get_rendering_dt()` inherits it.

**Rule:** pick a `rendering_dt` that is an exact multiple of `physics_dt`. `scripts/01_start_sim.sh`
passes `--render_dt 0.02` on the twin path — exactly four substeps, 50 Hz, decimation 5,
exactly 10 Hz — and leaves `physics_dt` alone so the walking policy (200 Hz physics,
decimation 4, 50 Hz policy) is untouched. If you need a different render rate, choose from
0.005 · n, and verify against a MEASURED topic rate rather than the reported dt.

**Related:** `world.current_time` has the same shape of problem — it advances once per
`world.step()`, not per physics substep, so a sim-time rate limiter driven by it emits at
most one message per render frame no matter what period you ask for. The twin accumulates
`step_size` inside `on_physics_step` instead.

---

## 2026-06-13 — DDS peers = live participants only (fresh-VM restore; camera 13→20 Hz)

**What:** On the fresh-VM restore (sim/vision VM `100.72.0.125`, laptop `wakeb`
`100.82.193.45`), `FEROX_DDS_PEERS` was pinned to both IPs per the restore
prompt. `wakeb` was a remote, high-latency (≈214 ms, jittery) tailnet peer
running **no DDS participant** — so cyclone's `tev` thread flooded failed SPDP
writes (`ddsi_udp_conn_write … 100.82.193.45 … retcode -3`, continuous) in both
the sim and vision containers, starving the camera publish/receive loop: the
[check_camera.py](../../ferox_vision/scripts/test/check_camera.py) gate sat at
~13 Hz (< 15 floor). Dropping `wakeb` (own-IP-only in isaac-demo `.env` and
ferox_vision `docker/.env`) cleared the flood; the rate returned to ~20 Hz —
gate PASS (geometry/frame/intrinsics were never affected).

**Standing rule (refines the "online tailnet node" heuristic of 2026-06-09):**
`FEROX_DDS_PEERS` lists **only hosts running a live DDS participant in the
current session**. Tailnet membership — even `Online` in `tailscale status` — is
**not** DDS participation: a pinned (or auto-derived) participant-less,
high-latency peer floods SPDP and starves publishers. Same-host sim+vision →
**own tailscale IP only**. `dds_peers.sh` derivation stays the default for
genuine multi-participant sessions, but an online-yet-idle host (no container)
must not be listed — re-add it as part of *its own* bring-up. Promoted to a
standing rule in [NEXT_SESSION.md](NEXT_SESSION.md).

---

## 2026-06-09 — DDS peers tailnet-derived (kill the ddsi_udp_conn_write flood)

**What:** Replaced the hardcoded `FEROX_DDS_PEERS` in `.env`
(`"100.82.193.45 100.112.212.89"` = laptop + this instance's VM IP) with a
peer list **derived from the tailnet at startup**. New
[scripts/lib/dds_peers.sh](../scripts/lib/dds_peers.sh) defines
`ferox_derive_dds_peers()`:
- This node's own `tailscale ip -4` — **always** included (same-host
  sim+nav+speech discovery needs it; multicast is off over the tunnel).
- Every currently-**online** node from `tailscale status --json`
  (`.Peer[] | select(.Online) | .TailscaleIPs[0]`). If `tag:ferox` is in use
  anywhere on the tailnet, remote peers are scoped to online `tag:ferox` nodes;
  if no node carries it, fall back to all online peers. Both paths tested.
- Degrades loudly (never a silent empty list): missing `tailscale`/`jq` or a
  down `tailscaled` → stderr warning + best-effort/multicast-only.

**Why:** Cyclone sprays SPDP across the participant-index range at every
`<Peer>`; an unreachable one floods the logs with `ddsi_udp_conn_write to
udp/<ip>:<port> failed` (retcode -3). Two hardcoded failure modes: (1) the
Vast.ai VM IP is **ephemeral**, so the baked-in self entry goes stale on the
next instance and floods forever; (2) the laptop was listed unconditionally, so
it floods whenever it's asleep. Listing only online nodes removes the flood **by
construction** — an offline host is never `.Online`, so it never becomes a
`<Peer>` — and the self entry tracks the live `tailscale ip -4` instead of a
stale literal.

**Where it runs:** derivation is host-side (tailscale + jq present there).
[scripts/lib/env.sh](../scripts/lib/env.sh) sources `dds_peers.sh` and sets
`FEROX_DDS_PEERS` when it isn't already set, so the one derived list feeds
**both** the sim (via [render_cyclone.sh](../scripts/lib/render_cyclone.sh),
which also self-derives if invoked standalone) **and** the Ferox nav stack (the
exported value wins over `Ferox/.env` in docker compose interpolation). The
minimal Isaac Sim / nav containers never re-derive — they consume the
host-rendered list.

**Override preserved:** set `FEROX_DDS_PEERS` explicitly (shell env or `.env`)
to bypass derivation for edge cases (non-Tailscale VPN, fixed pin); `""` =
multicast-only. Caller/.env value wins over the derived default.

**Unchanged (out of scope):** interface pinning (`FEROX_DDS_INTERFACE`),
`AllowMulticast=false`, `ParticipantIndex=auto`,
`MaxAutoParticipantIndex=120`, the domain, and the `<NetworkInterface
name="lo"/>` entry. Only peer-list construction changed. Other repos untouched.

**Files:** `+ scripts/lib/dds_peers.sh`; `~ scripts/lib/env.sh`,
`scripts/lib/render_cyclone.sh`, `.env`, `.env.example`,
[docs/NEXT_SESSION.md](NEXT_SESSION.md).

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

---

## 2026-06-03 — VM redeploy (100.120.30.7) + headless voice→drive validation

**What:** Stood up the Go2 voice→drive stack on the new Vast.ai VM (`ubuntu`,
100.120.30.7) and validated it headlessly (no mic). V1/V2/V3 pass. Three
containers on domain 42 / `tailscale0`: `ferox_isaac_sim`, `ferox_nav`,
`ferox_speech`.

**Validation:**
- **V1 stack health** — nav 9/9 lifecycle (7 servers + 2 costmaps) under
  `/ferox/go2_01/`; SLAM `/map` 399×470 @ 0.05 m (~170k known cells) after the
  warmup drive; `ferox_speech` subscribed to `/ferox/go2_01/audio/mic_raw`, no
  rmw crash. Cross-container discovery works with multicast off.
- **V2 action dry-run** — `MoveToNamed{charge_dock}` → cmd_vel 0.22 m/s, odom
  x 9.08→7.96, drove ~10 m to the dock.
- **V3 headless brain** — inject "take me to the charging dock" → Haiku →
  `MoveToNamed{charge_dock}` → motion + Piper "This way to the charging dock!"
  (20 chunks ~2 s on `speaker_out`). Re-confirmed with "reception".

**DDS scheme settled — A, everywhere:** `FEROX_DDS_INTERFACE` +
`FEROX_DDS_PEERS` (space-sep). Scheme B (`PEER_HOST`/`PEER_CLOUD`) exists only
in stale `ferox-audio-sim/CLAUDE.md`; nothing reads it. Template pins the
interface `presence_required="true"`, `AllowMulticast=false`, always adds
`<NetworkInterface name="lo"/>` → same-host sim+nav+speech discover via
loopback + the VM's own IP in the peer list.

**Files:**
- `+ .env` — master DDS (`tailscale0`, peers `"100.82.193.45 100.120.30.7"`);
  [scripts/lib/env.sh](../scripts/lib/env.sh) exports these to the sim
  (`render_cyclone`) **and** nav (compose passthrough via
  [scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh))
- `+ Ferox/.env` — defensive DDS for standalone nav; orchestrated runs
  override it via the env.sh shell-export
- `+ ferox-speech/docker/.env` — `ANTHROPIC_API_KEY` + DDS vars
- `+ ferox-speech/configs/agents/mall_concierge_debug.json5` — `= mall_concierge`
  + `stt_debug` input; V3 harness, bind-mounted `:ro`, no rebuild
- `− ferox-speech/.env` (root) — was silently ignored (see finding 2)

**Correctness findings:**
1. **`VENUE=dso_block_a` is required on
   [scripts/02_start_ferox.sh](../scripts/02_start_ferox.sh).** Bare →
   `VENUE=""` → empty waypoint DB → `MoveToNamed` rejected as unknown waypoint
   (fails V2/V3). Passing it loads the 4 DSO waypoints; `use_slam` stays true
   (not the AMCL variant). Works because `maps/dso_block_a.{yaml,pgm}` exist in
   the Ferox repo. (Latent: `ferox_nav.launch.py:104-116` runs the map-existence
   check even in SLAM mode — harmless only because the map exists.)
2. **ferox-speech key/DDS `.env` is `docker/.env`, NOT repo root.** Verified via
   `docker compose config`: Compose v2 loads the interpolation `.env` from the
   compose-file directory. The root `ferox-speech/.env` resolved to `""` → Haiku
   would 401, DDS would fall back to multicast. Moved to `docker/.env`, removed
   the misleading root copy.

**Note:** no `mall_concierge_anthropic.json5` — `mall_concierge.json5` is
already Anthropic (router: `tools:true` → `cloud=claude-haiku-4-5`). Enum = the
4 DSO waypoints, field = `name`.
