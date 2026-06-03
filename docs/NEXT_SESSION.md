# NEXT_SESSION

## Current state (2026-06-03)
Go2 voice→drive validated **headless** on VM `100.120.30.7` (Vast.ai). Three containers up, domain 42 / `tailscale0`: `ferox_isaac_sim`, `ferox_nav`, `ferox_speech`. Production `mall_concierge` running, subscribed to `/ferox/go2_01/audio/mic_raw`; Piper publishing `/ferox/go2_01/audio/speaker_out`. **VM is laptop-ready** — `100.82.193.45` already in the peer list, so the wakeb thread needs zero VM change.

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

## Do first
1. **[demo-blocking] Fix the `tool_use_id` 400** — `backends.py:284`, 2nd tick after a tool dispatch throws `anthropic.BadRequestError 400`. Temp clear: `docker compose up -d --force-recreate speech`. Real fix: track `tool_use` IDs, include only matching `tool_result` blocks in the next turn. **Any multi-turn demo needs this** (can't force-recreate between live utterances).
2. **Commit `mall_concierge_debug.json5`** (terse, no-prefix). **First** verify the `.env`s are gitignored — `git check-ignore ferox-speech/docker/.env` MUST return the path; it holds the live API key, and a root-anchored `/.env` rule would NOT cover `docker/.env`.

## Pre-demo hardening
- **Bake the Whisper model into the ferox-speech image** — currently fetched from HuggingFace at runtime (cold-start network dependency + bake-deps violation). Demo-day risk on an unreliable venue network.
- **`ferox_nav.launch.py:104-116`** runs the venue map-existence check even in SLAM mode (contradicts its docstring). Harmless now (map exists); will **crash SLAM bring-up at a venue with waypoints but no saved map** → fix before the M2 duty-free pilot.

## wakeb (live audio) thread — separate, do nothing on the VM
On the laptop: set `ferox-audio-sim/.env` peers `"100.82.193.45 100.120.30.7"` + `FEROX_DDS_INTERFACE=tailscale0`, then `docker compose up -d`.
⚠️ **Latency ≈165 ms** (mostly DERP, occasional direct hop) — geographic (Cairo ↔ Paris relay), **above the 150 ms audio threshold regardless of path**. Decide before investing: co-locate `ferox-audio-sim` on the speech box, or run speech on an in-region (me-central-1) VM.
Also: `tools/live_audio.sh` has a stale IP `100.118.201.71` to update there.

## Gotchas (carry forward)
- DDS scheme = **A** (`FEROX_DDS_INTERFACE` + `FEROX_DDS_PEERS`). Peer list **must include the VM's own IP** for same-host discovery (multicast is off).
- ferox-speech key/DDS `.env` = **`docker/.env`**, not repo root.
- **`VENUE=dso_block_a`** required on `02_start_ferox.sh`.
- DDS log spam from the offline laptop peer (`ddsi_udp_conn_write … failed`) is harmless — filter with `grep -avE "tev:|ddsi_udp"`.
