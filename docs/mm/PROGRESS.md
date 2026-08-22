# MM campaign — running log (4090 session, 2026-08-22)

> Newest entries at the bottom. This file is the MM campaign's task-boundary log and the
> resume point: **"Resume MM campaign from docs/mm/PROGRESS.md"**. The DT campaign's log
> at `docs/twin/PROGRESS.md` holds everything up to 2026-08-21 and is not superseded —
> the MM narrative simply continues here, because the brief names this path.

## Session header — 2026-08-22

| | |
|---|---|
| Box | **NVIDIA GeForce RTX 4090, 24564 MiB, driver 580.105.08** — gate PASSES |
| CPU / RAM / disk | 40 vCPU, 131 GB, 291 GB (270 GB free) |
| Docker | 29.0.3, Compose v2.40.3, no images at start |
| Desktop | X on `:0` |
| Repos | `ferox-isaac-demo` @ `mohammed/mm-campaign` (= `mm-persist-7`), `Ferox` @ `mohammed/mm-campaign`, `ferox-g1-locomotion` @ `main`, refs in `~/panthera/ref` |
| Mode | unattended, task to task; stop only on non-4090 / §5 fail-twice / destructive |

**Deviations from the RESUME box, both logged and both benign:**

1. **VRAM is 24 GB, not the 48 GB of the RESUME box.** Still an RTX 4090 and the gate is
   the model name, so this is not a stop. C-23's own evidence measured peak VRAM at
   **3776 MiB**, so 24 GB clears the camera path with 6× headroom. Locomotion retrains,
   if any, cap at 2048 envs rather than 4096.
2. **`ferox-g1-locomotion` has no `mohammed/mm-campaign` branch** — left on `main`
   (8d5501a). It is read-mostly here; a branch gets created if a commit is needed.
3. **Tailscale is `NeedsLogin`**, so `.env`'s `FEROX_DDS_PEERS` cannot be a tailnet
   address. Everything this session needs is host-local (Isaac + the DDS seam in one
   box), so the safer branch is loopback-only DDS with no external peer — chosen, and it
   also satisfies §0.4's "must never be reachable from a real robot" trivially.
4. **PAT lives in the scratchpad**, mode 0600, outside every repo, because the harness
   gives each shell a fresh environment and `GIT_CONFIG_*` cannot persist between
   commands. It is sourced per command, never written to a git config, and is deleted at
   the end of the session. Residue is checked after every push.

---

## Task 0 — record the physics correction — **DONE**

**The static-hold fall is not a discriminator, and the "solver" inference is withdrawn.**

An upright biped on ankle PD is an inverted pendulum: stable only if `2·kp > m·g·h`.
Measured from this campaign's own evidence (h = CoM height above the ankle-roll joint):

| model | mass | h | **m·g·h** |
|---|---|---|---|
| twin, Dex5 | 39.0048 kg | 0.689261 m | **263.7 Nm/rad** |
| twin, bare 29-DoF | 33.3411 kg | 0.653798 m | **213.8 Nm/rad** |
| **reference `g1_29dof_old.xml`** | 35.1121 kg | 0.679407 m | **234.0 Nm/rad** |

against `HOLD_KP` ankle 40 → **80 Nm/rad** for both ankles, and SONIC's wire kp 28.5 →
**57 Nm/rad**. A factor of three, so no error in h or kp changes the sign. **The
reference model fails the same test**, which is the whole point: the static hold would
fall in MuJoCo too, so it separates nothing — not simulator, not solver, not asset. A
balancer instead *moves its targets* (SONIC re-commands `q_d` at 50 Hz), which is the
mechanism a fixed-target hold does not have.

Written to `evidence/C39/CORRECTION.md` §4; `VERDICT.md` carries a superseded-in-part
banner and a withdrawal footer. Its mass diff, foot geometry and contact-API rows stand.

**Live question, and the only one: SONIC-in-twin vs SONIC-in-MuJoCo.**
