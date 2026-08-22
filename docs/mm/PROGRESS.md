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

### 1e. The twin side does not reach the balance question — and why

Three runs, each one ruling something out:

| run | hands | rig yaw | outcome |
|---|---|---|---|
| `twin_dex5_abort` | Dex5 | pinned to 0 | SONIC aborts: `body_dq[24] = 35.367 > 35` → **right_wrist_roll** |
| `twin_bare` (1st) | none | pinned to 0 | SONIC aborts: `body_dq[17] = 35.9822 > 35` → **left_ankle_roll** |
| `twin_bare` (2nd) | none | **not pinned** | *running* |

**SONIC never "fails to balance" in any of these — it stops itself.** Its own guard is
`body_dq[i] > 35` at `g1_deploy_onnx_ref.cpp:2832`, it aborts the entire control system,
and the rig's auto-release needs sustained authority, so the robot is never free-standing
and the verdict tool correctly says INVALID rather than FALLS. Full write-up with the
index mapping (`mujoco_to_isaaclab`, printed index is mujoco order, not SDK) in
`evidence/C39/SONIC_ABORT.md`.

**The second abort is my own fault and worth writing down.** I set `G1_LL_RIG_YAW=0` to
remove the spawn-yaw confound the code itself documents. That knob does not do what its
name suggests: `sim_side.py:310` captures the robot's real spawn pose, and the override
at `:333` then **replaces the quaternion** — so the rig pins the base to a yaw the robot
was never spawned at and twists it 90° against its own planted feet. The ankle saturates
at its 35 Nm limit (`|tau|max=35.00 sat=5/29` through the whole hold) and rings past
35 rad/s. Removed; both sides now spawn from the same world config, which is all the A/B
needs. **Never use `G1_LL_RIG_YAW` with a rig-held base** unless the spawn yaw is changed
to match.

**Two properties of the upstream guard nobody had recorded**, both from its source:

1. It is **one-sided** — `body_dq[i] > 35`, not `|body_dq[i]|`. A joint at −40 rad/s
   passes. It therefore looks intermittent when it is not.
2. **`--disable-crc-check` disables it too.** The same flag gates both. Every earlier
   C-39 run carrying `SONICFLAGS=--disable-crc-check` had **no velocity guard at all**,
   which is exactly why those runs reached the rig release and these did not. The flag
   is not the no-op its name implies, and any "SONIC fell" row should be re-read with
   that in mind.

### 1f. Two more latent repo defects fixed

* **The `HAND=none` report line printed no base height and no pitch.** The whole line was
  one conditional expression across implicitly concatenated f-strings, and Python
  concatenates *before* it applies the conditional — so the entire prefix
  (`[lowlevel-sim] t=`, mode, rtf, `base_z`, `pitch`, `roll`, `|tau|max`) belonged to the
  `has_hands` branch alone. Every bare-robot run in this campaign printed a bare
  `knee_L=… hip_p_L=…`, i.e. **the two numbers every C-39 verdict is read from were
  missing**, and a bare run looked silent rather than wrong.
* Editing a shell script while `bash` is still executing it corrupts the running
  instance — bash reads the file incrementally. That produced a phantom
  `line 81: is: command not found` on an untouched comment. Runs are no longer started
  from a script that is about to be edited.

---

## Task 1 — **CLOSED**. The asset is exonerated; no PhysX property stands SONIC; SONIC parked.

Nine runs, sole occupancy asserted before each, **zero `SECOND_INSTANCE` rows**,
harness `sha256` recorded in `evidence/C39/bisect/sha256.txt`.

| label | property | released | base_z | pitch | verdict |
|---|---|---|---|---|---|
| `baseline_twin` | control | 9.88 | +0.107 | +88.3° | FALLS |
| **`baseline_ref`** | **reference body, unmodified** | 11.58 | +0.098 | **+86.3°** | **FALLS** |
| `solver_iters` | `64,64` | 9.18 | +0.108 | +88.4° | FALLS |
| `friction_mult` | `combine=multiply` | 11.98 | +0.107 | +88.3° | FALLS |
| `contact_off` | `contact 0.002 / rest 0.0` | 11.96 | +0.107 | +88.3° | FALLS |
| `depen_vel` | `max_depen_vel=1.0` | 15.66 | +0.107 | +88.4° | FALLS |
| `self_coll` | `self_collision=1` | 11.40 | +0.107 | +88.4° | FALLS |
| `dt_200hz` | the reference's own rate | 27.57 | +0.067 | −86.6° | FALLS |
| `implicit_drive` | `G1_LL_PD=implicit` | 11.04 | +0.095 | +85.5° | FALLS |

**The reference MuJoCo body, imported unmodified, falls in our simulator** — and the
same body stands under the same deploy binary in the reference MuJoCo sim. So C-39 is
not our asset, not the wire, not the hands and not SONIC. Seven simulator properties
later it is also not any one of them, and the tell is that every row lands in the *same
place*: base ~0.10 m, pitch ~87°, ~2.5 s after release. A parameter that mattered would
move that number.

**SONIC parked** per the brief; finetune becomes a Spark item. What remains is the Isaac
harness against the MuJoCo one *as a whole* — actuator model, contact solver family,
MuJoCo's own 200 Hz constraint formulation — which is a larger piece of work than a
sweep and sits behind the manipulation gates.

### Void-rule accounting (Mohammed's instruction)

The first `twin_bare`/`ref` pair and the first `solver_iters` run were taken while a
second agent instance existed on this box and are **void**. They were re-run as
`baseline_twin`/`baseline_ref`/`solver_iters` under proven sole occupancy and **the
result reproduced**, so `AB_ASSET_VERDICT.md` moved from PROVISIONAL to CONFIRMED rather
than being withdrawn. `scripts/c39_bisect.sh` now asserts sole occupancy before every
run and records `SECOND_INSTANCE` instead of a number if it ever fails.

### Re-measurement notice filed

`RESULTS_MM3.md` and `RESULTS_MM4.md` now carry a banner naming the specific figures
invalidated by the 73% lowcmd drop (`LOWCMD_SEQLOCK.md`): MM3 (c) lowstate rate/parity
and (a) PD-stand tracking; MM4 policy latency, LowState age, the PD-tracking table, and
the 499 Hz lowcmd figure — counted at the **bridge**, which received everything, while
the robot did not. Convention/CRC/field-layout work and offline asset arithmetic are
explicitly unaffected.

## Task 3 — grasp v7, contact-report route — **the route attaches**

The thing v6 could not do:

    [contact] contact-report route ACTIVE: 22 prim(s), 21 finger link(s),
              target /World/Env/objects/soup_can

No sensor prim, no re-parenting, no per-body wrappers — `PhysxContactReportAPI` at
threshold 0 on links we already have, plus one subscription. `counts()` returns `None`
when unavailable so the caller cannot mistake "unknown" for "zero", which is the
mistake that cost v5/v6 two versions of collider-chasing.

### Grasp v7 — two real defects fixed, and neither one is what stops the grasp

**Correction to my own claim, recorded before anything is built on it.** I called the
`obj_pose` USD read "the root cause of the grasp workstream". It is not. It is a real
defect and it is fixed, but the outcome barely moved:

| | before the fix | after |
|---|---|---|
| trial 1 | `REACH_TIMEOUT` 93 mm | `REACH_TIMEOUT` **91 mm** |
| trial 2 | `DESCEND_TIMEOUT` 40 mm | `DESCEND_TIMEOUT` **39 mm** |

What the fix *did* do is make the measurement honest — trial 2 now stages and reads
`[1.815, -1.333]` consistently, where before the controller read a pose 138 mm away —
and it removes a defect that would have corrupted every future number. It is worth
having. It is not the answer.

**What the numbers actually say.** Palm bottoms out at **z ≈ 0.945** across the whole
stall, with the can at **z = 0.801**. That is the workspace limit this campaign already
measured — *"the arm is at the floor of its workspace at table height ... the palm tops
out around z = 0.95 with the can's centre at 0.80"* — and the remedy already exists in
the runner: `MM5_SURFACE=counter` stages on the 0.90 m counter, whose own code comment
says *"the table at 0.75 m is below this arm's workspace"*. Every v7 run so far used the
default `table`. Testing the counter now.

**Three defects of one family this session**, which is the pattern worth carrying:

| defect | did the write apply? | what lied |
|---|---|---|
| `set_masses` without positional `indices` | **no** | a silent no-op that read as a working fix |
| lowcmd seqlock, three writers, no lock | yes | the *reads* returned `None` — 73% lost |
| `obj_pose` through USD | **yes** | the *read* never moved |

In all three the code trusted its own intent instead of reading back. The staging
read-back, the `sat_j` joint names, the `G1_PHYSX_TWEAKS` zero-prim warning and the
"unavailable ≠ zero contacts" rule are all there to stop the next one.
