# NEXT_SESSION

## Current state (2026-06-03)
Go2 voice→drive validated **headless** on VM `100.120.30.7` (Vast.ai). Three containers up, domain 42 / `tailscale0`: `ferox_isaac_sim`, `ferox_nav`, `ferox_speech`. Production `mall_concierge` running, subscribed to `/ferox/go2_01/audio/mic_raw`; Piper publishing `/ferox/go2_01/audio/speaker_out`. **VM is laptop-ready** — the DDS peer list is tailnet-derived (`scripts/lib/dds_peers.sh`), so the laptop (`wakeb`, `100.82.193.45`) is picked up automatically whenever it's online; the wakeb thread needs zero VM change.

**Bring-up** (cold ≈5 min build + ≈180 s sim):
```
# ferox-isaac-demo
./scripts/00_bootstrap.sh
./scripts/01_start_sim.sh
VENUE=dso_block_a ./scripts/02_start_ferox.sh      # VENUE is mandatory — bare = empty waypoint DB
./scripts/03_teleop.sh                              # SLAM warmup drive before first goal
# ferox-speech
./scripts/02_start_speech.sh
# headless brain test (needs the mall_concierge_debug / stt_debug agent)
./scripts/03_test_pipeline.sh "take me to the charging dock"
```

## Done (2026-06-03)
- ✅ **`tool_use_id` 400 fixed** — multi-turn tool calls now work **without** `--force-recreate`. The accumulator preserves the Anthropic `tool_use` id, re-emits assistant `tool_use` blocks, pairs each with a synthetic `dispatched` `tool_result` (real outcomes arrive as `[nav]` feedback), and gates idle/in-flight ticks so no assistant turn wedges between a `tool_use` and its result. Baked into `ferox/speech:humble` and validated on the baked image (4-turn session, 0× 400, 77 unit tests). See ferox-speech `DEV_LOG.md` 2026-06-03.
- ✅ **`mall_concierge_debug.json5` committed** (ferox-speech `2caa429`); `docker/.env` confirmed gitignored.

## Do first (pre-demo hardening)
- **Bake the Whisper model into the ferox-speech image** — currently fetched from HuggingFace at runtime (cold-start network dependency + bake-deps violation). Demo-day risk on an unreliable venue network.
- **`ferox_nav.launch.py:104-116`** runs the venue map-existence check even in SLAM mode (contradicts its docstring). Harmless now (map exists); will **crash SLAM bring-up at a venue with waypoints but no saved map** → fix before the M2 duty-free pilot.

## wakeb (live audio) thread — separate, do nothing on the VM
On the laptop: set `ferox-audio-sim/.env` peers `"100.82.193.45 100.120.30.7"` + `FEROX_DDS_INTERFACE=tailscale0`, then `docker compose up -d`.
⚠️ **Latency ≈165 ms** (mostly DERP, occasional direct hop) — geographic (Cairo ↔ Paris relay), **above the 150 ms audio threshold regardless of path**. Decide before investing: co-locate `ferox-audio-sim` on the speech box, or run speech on an in-region (me-central-1) VM.
Also: `tools/live_audio.sh` has a stale IP `100.118.201.71` to update there.

## Gotchas (carry forward)
- DDS scheme = **A** (`FEROX_DDS_INTERFACE` + `FEROX_DDS_PEERS`). The peer list is **tailnet-derived** at startup by `scripts/lib/dds_peers.sh`, **not hardcoded**: this node's own `tailscale ip -4` (always — required for same-host discovery, multicast is off) + every currently-online tailnet node (scoped to `tag:ferox` if that tag is in use, else all online peers). Set `FEROX_DDS_PEERS` explicitly only to override.
- ferox-speech key/DDS `.env` = **`docker/.env`**, not repo root.
- **`VENUE=dso_block_a`** required on `02_start_ferox.sh`.
- DDS log spam (`ddsi_udp_conn_write … failed`) from an offline peer **no longer occurs** — only currently-online tailnet nodes are listed as `<Peer>`, so a dead host is never targeted by SPDP. (If you ever pin `FEROX_DDS_PEERS` manually to a host that goes offline, the old spam returns; filter with `grep -avE "tev:|ddsi_udp"`.)
