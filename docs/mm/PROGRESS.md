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

---

## Task 1 — the C-39 decisive A/B — **IN PROGRESS**

### 1a. Box brought up from nothing

| step | result |
|---|---|
| `nvidia-smi` gate | **RTX 4090, 24564 MiB, driver 580.105.08 — PASS** |
| Isaac Sim 5.1.0 image | pulled, 22.9 GB |
| `ferox/msgs:humble`, `ferox/nav:humble` | built by `00_bootstrap.sh` |
| `ferox/twin-lowlevel:humble` | built (context: `unitree_sdk2_python` + `unitree_ros2`'s `cyclonedds_ws/src/unitree`) |
| `ferox/sonic-deploy:v1.1-x86_64` | built from upstream pin `54d0b10`, HF artifacts `sha256` in `logs/sonic_artifacts.sha256`; `--help` runs |
| twin sim boot, `G1_CONTROL=lowcmd` | **reaches the main loop**, bridge PD running, GT trace emitting |

### 1b. The reference body is now an Isaac asset — **DONE**

`g1_29dof_old.xml` through the stock MJCF importer, `isaac/assets/g1_ref_mjcf/` (27 MB):

    revolute joints in USD: 29
    links with mass: 30  total 35.112142 kg
    OK: mass matches the offline MJCF sum (35.112142) within 0.05
    OK: all 29 MJCF hinge joints present by name

"Unmodified" is evidenced, not claimed: every field of `MJCFCreateImportConfig` is read
back into `evidence/C39/import_mjcf.txt` before the import runs, and the report lists
the five deltas with before/after values — `fix_base` False (the reference is a
floating-base model), `make_default_prim` True, `create_physics_scene` False (run.py
owns the world's scene), `import_inertia_tensor` True, `self_collision` False.

The bridge maps its 29 joints **by name** and the MJCF's names are the canonical G1
set, so the identical fork drives the reference body with no code branch at all. The
IMU is read off the articulation root pose, which is asset-agnostic; the hand maps are
already tolerant of a hand-less robot (`HAND=none` has always been a supported variant).

### 1c. Six defects found bringing this up — three mine, three latent in the repo

| # | defect | mine? | fix |
|---|---|---|---|
| 1 | repo cloned 0700 (a `umask 077` from PAT staging leaked into that clone), so Isaac's UID 1234 could not read the mount — `cd: /workspace/ferox_isaac: Permission denied` | **mine** | `chmod -R a+rX` on the repos; output dirs chowned to 1234. **No permission change to `/root`** — the bind mount short-circuits the path chain, so only the target dir's mode matters |
| 2 | `git lfs pull -I <pattern>` silently no-ops on this clone; `libunitree_sdk2.a` stayed a 133-byte pointer and the SONIC link died with "file format not recognized; treating as linker script" | **mine** | `git lfs fetch --include=… && git lfs checkout <dir>` |
| 3 | the reference **meshes** were also pointers (132 bytes) — the MJCF importer reported "Asset convert failed: Unsupported Format" and then a Fatal on a NULL stage | **mine** | fetched `gear_sonic/data/robots/g1/**` (998 MB) |
| 4 | the MJCF importer writes a temp USD **next to each STL**, so a read-only staged mesh dir fails conversion with the permission never mentioned | latent | stage `a+rwX`, documented in the script |
| 5 | `01_start_sim.sh` passes every knob as `-e VAR="${VAR:-}"`, so an unset `G1_LL_RIG_YAW` arrives as `""` and `float("")` **raises inside the physics callback**, taking Isaac down 370 s into a boot with the traceback buried in a warning storm | **latent, repo** | empty now means unset |
| 6 | `cyclonedds.xml.template` hardcoded a second `<NetworkInterface name="lo"/>`, so `FEROX_DDS_INTERFACE=lo` selected `lo` twice → Cyclone refused every `rmw_create_node` | **latent, repo** | loopback moved into the renderer, added only when it is not already the pinned interface; `tailscale0`/autodetect render byte-identically |
| 7 | `c39_ab_asset.sh` sourced `lib/env.sh` before setting `ROBOT`, pinning an exported `ROBOT_ID=go2_01` that survived into the child launcher → "contract namespace /ferox/g1_01 != --ros_namespace /ferox/go2_01" | **mine** | `export ROBOT=g1` before the source, `env -u ROBOT_ID` on the child |

### 1d. One confound removed before the A/B runs

`sim_side.py` already records that the reference MuJoCo sim spawns at **identity yaw**
while the twin's hospital spawn sits at **90°**, and that `facing=(1,0,0)` therefore
means "hold heading" in MuJoCo and "turn 90° right now" on the twin. Both sides of this
A/B run `G1_LL_RIG_YAW=0`, so the two runs differ in **body and nothing else** — which
is the entire point of the experiment.
