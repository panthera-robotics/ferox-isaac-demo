# RESUME — rebuild this box from zero, from GitHub alone

Written so a fresh instance with nothing but this repository can get back to where the
campaign is. Read [§0 You are here](#0-you-are-here) first, then work down.

Everything here was executed on the box it describes. Where a number matters (a driver
version, a `render_dt`, a tolerance) it is the number that actually worked, not a
plausible one.

---

## 0. You are here

**MM campaign, 2026-08-20. MM3 PASS. MM4 and mobile MM5 are both blocked on ONE thing:
C-39 — SONIC will not balance the twin, and it is not the twin's wire.**

| | State |
|---|---|
| **MM3 — low-level DDS bridge** | **PASS.** `rt/lowstate` at 1041.68 Hz (measured off the robot, not the 500 Hz the gate asked for), 16/16 field-parity checks, fail-closed 6/6 inside 100 ms, 20-entry dex3 wire at 208 Hz. Test (a) rewritten by Mohammed as (a1) suspended-base PD hold — 28/29 joints inside 0.05 rad — and (a2) balance, moved to MM4. |
| **MM4 — SONIC** | **PARTIAL, parked.** The x86_64 image builds and runs, and SONIC closes the loop on the twin: reads `rt/lowstate`, prints `G1 type: 5`, commands `rt/lowcmd` at 499 Hz with CRC on. It will not stand the robot. |
| **C-39 — the A/B is done** | **SONIC STANDS IN THE REFERENCE MUJOCO SIM AND FALLS IN THE TWIN** — same image, deploy binary, driver and DDS seam, only the simulator differs. The x86 port is exonerated. Seven hypotheses are dead (stance, mass, conventions, IDL, missing topic, spawn heading, publication rate). Every remaining divergence is a field where the twin is MORE robot-faithful than the reference. **Start at `docs/mm/evidence/MM4/AB_MUJOCO.md`, and run the one experiment it names: `MJC=1 SONICFLAGS=--disable-crc-check`.** |
| **C-39 — earlier finding** | **NOT the twin's wire.** Conventions diffed field-by-field against the reference MuJoCo bridge (the one that fed W3's 17.53 m walk) and against the DT bag: they match. Not the stance transition, and **not the hand mass** — with the palms zeroed to 0.34 kg/hand the result matches to three figures. SONIC's own commanded targets are out of range: 3 of 29 beyond the URDF limits, a 1.648 rad knee against its own 0.30 nominal, with the robot upright and its observations clean. **Start at `docs/mm/evidence/MM4/C39_FORK.md`.** |
| **MM5 — manipulation** | Mobile is 0/20, `ROBOT_FELL` ×20 — the omni policy cannot hold balance while the arm reaches, which is C-39 from the other side. The **fixed-base variant** is the reported result; see RESULTS_MM5. |
| **MM1b — retrain** | **PARKED.** Finished 6000/6000, reward −3.35 → +1.24, episode length 24 → 55 of a 1000-step ceiling. It learned to fall over more slowly. `ferox-g1-locomotion policy/PROVENANCE_v2_mm1b_c.md` retires the "one more overnight" claim. |
| **Media** | **None, everywhere. C-23** — this box cannot run the camera and cannot film the live sim. Every MM gate states it per-gate. |
| **MM6 / MM7** | Not started; both need the camera, so both need the 4090 day. |

**Next instance, do this first:** read `docs/mm/evidence/MM4/C39_FORK.md`, then run the
same fork against the reference MuJoCo sim rather than the twin. If SONIC commands
out-of-range targets there too, the twin is exonerated entirely and the fault is in how
this deploy is driven. That single experiment decides whether mobile manipulation is a
twin problem or an upstream one, and the wire is already proven identical either way.

---

## 0b. You are here — DT campaign (previous)

**G1: mount fixed, lidar 360°, camera unverified on this GPU. Go2 bag pending.
DT6 / DT7 deferred.**

| | State |
|---|---|
| **G1 — hand mount** | **FIXED.** Both Dex5 flanges carried `rpy="0 0 0"`; the Dex5 root is not wrist-aligned. Fingers **90.000° → 0.707°** off the forearm axis, both hands, chirality PASS. Isaac suite **13/13**. |
| **G1 — Mid-360** | **FIXED.** Each message was one render frame's slice. Now **360° per message**, 36/36 bins, **15466** valid points (was ~72° / 4285), **10.00 Hz** exactly in sim time. |
| **G1 — camera** | **UNVERIFIED ON THIS GPU.** Not a twin defect — **C-23**, and it is the GPU (Mohammed, 2026-08-19). See §1 before touching anything camera-shaped. |
| **G1 — nav** | 1 of 3 goals SUCCEEDED, the first this twin has ever reached. Accepted as **not a defect**; the evidence run needs re-scoping, see below. |
| **Go2** | **Ground-truth bag still pending** (OQ-5.1). It decides C-17 and closes the Go2's `pointcloud/fields` (C-2). Nothing else about the Go2 moved. |
| **DT4 / DT6 / DT7 / DT8** | **deferred by Mohammed — do not start.** |

| Gate | Verdict | Tag |
|---|---|---|
| DT0 | PASS-with-deviations (accepted) | `twin-DT0` |
| DT1 | PASS (accepted) | `twin-DT1` |
| DT2 | PASS-with-deviations; lidar sector fixed; nav 1 of 3 | `twin-DT2` |
| DT3 | PASS — hand mount orientation corrected 2026-08-19 | `twin-DT3` |
| DT5 | Interface PASS, navigation PARTIAL (C-17) | `twin-DT5` |
| G1 ground truth | OQ-2 + OQ-3 closed, C-18/19/20 opened | `twin-gt-g1` |
| Persistence | this document | `twin-persist` |
| **Video-review fixes** | **A fixed · B fixed · C blocked by C-23 (GPU)** | `twin-g1-fixed-2` |

### The three video-review defects

* **A — Dex5 mount orientation. FIXED.** DT3 verified the mount's *position*
  (exact to 0.0000 mm) and nothing verified its *orientation*, so both hands sat
  90° out — fingers laterally outward, palm forward, thumb down — for the whole
  gate. `rpy = (−π/2, −π/2, 0)`, both sides, derived by
  `tools/derive_hand_flange.py` from the Dex5 URDF against the convention
  extracted from Unitree's own G1+Inspire assembly, and self-tested by
  re-deriving Unitree's published flange to **1e-16**. Guarded on the built USD.
  Write-up: `RESULTS_DT3.md` §8.
* **B — the Mid-360 published a sector. FIXED.** Isaac 5.1 has two ROS 2 cloud
  writers and the twin used the non-accumulating one.
  `omni:sensor:Core:accumulateOutputs` **does not exist** — accumulation is an
  annotator choice, not a prim attribute. `/scan` **20% → 45%**, SLAM map's known
  cells **36/36 bins** (was a fixed wedge), first Nav2 goal ever **SUCCEEDED**.
* **C — the aligned-depth panel. NOT SETTLED, and not settleable here.** See
  C-23 and §1.

### Next steps, in the order they are worth doing

1. **Item C + E-1 + C-21's live re-proof + the montage's camera clip** — all four
   unblock together on a 4090 box, with no code change. `nvidia-smi` first (§1).
2. **G1 twin nav evidence — ≤1 h on the next box.** 1 of 3 is not a defect; the
   run was scoped wrong. Goals 2 (7.8, 6.5) and 3 (6.5, 5.0) sat outside the map
   as it stood at the time — 150×140 cells @ 0.05 m from origin (3.712, −2.773),
   i.e. y only reached **4.23 m**. Either:
   * **keep `hospital` and put the goals inside the mapped free space.** Measured
     at the end of that run (`docs/twin/evidence/DT2-360/`, map 200×232 @ 0.05 m,
     origin (3.161, −2.773)): free-space bounds **x [3.71, 12.16] m**, **y [−2.67,
     8.18] m**; robust interior (5–95th percentile of free cells) **x [5.16,
     10.76]**, **y [−1.42, 5.08]**. Pick three goals inside that interior, or
     drive a mapping lap first and re-measure; **or**
   * **spawn in an enclosed room** rather than mid-corridor, so the map closes
     around the robot and `range_max` 6.0 m reaches walls in every direction.
   Do not tune the footprint/inflation interaction — DT2 decided that is
   Ferox-side and deliberately untouched.
3. **Go2 ground truth (OQ-5.1)** — unchanged, still the item that decides C-17.

**Still open, unchanged:** E-1 detection half, E-2 Go2 puck/bracket, OQ-5.2,
OQ-5.3, OQ-6, C-3/C-4.

**Deviations:** C-1 … C-20 open, **C-21 closed**, **C-23 opened** (environment).
A **C-22 was opened and withdrawn**: `ros2 topic hz` is wall-clock and the sim does
not run at 1.0×, so a conformant 10 Hz cloud read as 12.6 Hz at RTF 1.26. On header
stamps it is exactly 10.00 Hz. **Measure the stamps, not the wall clock.**

**Standing rules that outlive the box:** `CLAUDE.md` (RULE-HAND-NAME and the rest)
and `docs/twin/CAMPAIGN.md` §0.

---

## 1. The VM that worked

Vast.ai instance, Ubuntu 22.04:

| | |
|---|---|
| GPU | **NVIDIA RTX 4090, 48 GB** (`nvidia-smi` reports 49140 MiB) |
| Driver | **580.105.08** — Isaac Sim 5.1 needs ≥ 570; do not accept an older image |
| CPU / RAM | 30 vCPU, 98 GB |
| Disk | **291 GB** — do not go below ~200 GB: Isaac's image is ~22 GB and `cache/` grows to ~5 GB |
| Docker | 29.0.3, Compose v2.40.3 |
| Desktop | KDE on `:0` (needed only for viewport screenshots; everything else is headless) |

Disk is the constraint people underestimate: 22 GB image + 5 GB cache + 100 MB repo +
captures, and Isaac writes shader cache on every new world.

### The GPU is not interchangeable — check it before any camera work

> **Camera path verified only on RTX 4090 / driver 580.105; RTX 4080 SUPER 16 GB
> (this box) segfaults the ROS 2 image writer — C-23 — check `nvidia-smi` before
> starting camera items.**

Five boots out of five, at `run.py:1201`, on the first `world.step(render=True)`
after a camera render product exists. Ruled out by measurement, not argument:
memory (peak VRAM **3776 MiB of 16376**, host RSS 5.7 GB of 47, no OOM, no Xid),
the asset, the world, and the depth annotator. The **Go2** twin — same box, same
RTX lidar, same bridge, no camera in its contract — boots every time, and the
**offscreen** render path works fine, so it is the **ROS 2 image writer**
specifically and not "any camera".

**Evidence:** [`evidence/C23/README.md`](evidence/C23/README.md) — the five boots
with what each one ruled out, the two controls, the raw crash tails and the 1 Hz
VRAM/RSS trace through boot 3.

**Disposition (Mohammed, 2026-08-19): it is the GPU. Do not work around it.**
`TWIN_CAMERA=0` exists only to keep the lidar/nav half workable on a box like this
one; it skips the camera *device* and changes nothing the audit checks. Anything
camera-shaped — item C, E-1, C-21's live re-proof, the montage's camera clip —
waits for a 4090.

```bash
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader
# want: NVIDIA GeForce RTX 4090, 49140 MiB, 580.105.08
# 16376 MiB => do not start camera items; C-23 will take the process down
```

---

## 2. Host tweaks (they die with the VM — redo them)

```bash
# 1. KDE screen locker OFF. The locker blanks the X root, and any viewport capture
#    silently returns a black frame. This cost a whole gate's screenshots once.
mkdir -p ~/.config
printf '[Daemon]\nAutolock=false\n' > ~/.config/kscreenlockerrc

# 2. Viewport capture, if you want one, needs a LOGGED-IN session on :0 --
#    not just a running X server. Without it Isaac falls back to headless and
#    the GUI viewport renders empty. `twin_viewport_hospital.png` in
#    evidence/DT2 is exactly that mistake preserved.
DISPLAY=:0 import -window root out.png

# 3. Offscreen rendering works with no desktop at all and is the route to prefer:
#    scripts/13_capture_hands.sh and scripts/14_capture_views.sh both do it.
```

**Two Isaac-scripting gotchas that will waste an hour if forgotten** (both now guarded
in the scripts, but they apply to anything new you write):

* Stage Isaac scripts in **`/tmp/isaacrun`, never bare `/tmp`**. `sys.path[0]` is the
  script's own directory, so a scratch file named after a stdlib module shadows it for
  Isaac's own startup. A probe called `bisect.py` did this and broke every Isaac script
  in `/tmp`, with a circular-import warning that named neither the file nor the module.
  It bit again months later on the *host*, while uploading a release asset.
* Run Isaac execs with **`PYTHONDONTWRITEBYTECODE=1`**. A root-owned `__pycache__` under
  Isaac's package tree breaks every later run as UID 1234.

---

## 3. Clone and configure

```bash
mkdir -p ~/panthera && cd ~/panthera
git clone https://github.com/panthera-robotics/ferox-isaac-demo.git
git clone https://github.com/panthera-robotics/Ferox.git
git clone https://github.com/panthera-robotics/ferox-g1-locomotion.git
cd ferox-isaac-demo && git checkout mohammed/twin-campaign
cd ../Ferox && git checkout mohammed/twin-campaign && cd ..
```

`ferox-g1-locomotion` is needed by `tools/tests/test_isaaclab_cfg.py`, which asserts the
Isaac Lab cfg agrees with `deploy.yaml`. Those tests **skip cleanly** if it is absent.

Reference repos (read-only; the commits this campaign actually read):

```bash
mkdir -p ~/panthera/ref && cd ~/panthera/ref
git clone https://github.com/panthera-robotics/panthera-g1-driver.git      # 01c557c main
git clone https://github.com/panthera-robotics/panthera-go2-driver.git     # cabb5de main
git clone https://github.com/panthera-robotics/realsense_driver.git        # 124c1c4 main
git clone https://github.com/unitreerobotics/unitree_ros.git               # daadf41 master
git clone -b mohammed/g1wb-review-w7-docs \
  https://github.com/panthera-robotics/panthera-g1-wbc.git                 # f9a960d
```

`unitree_ros` (286 MB) supplies the URDFs for the hand merge; `panthera-g1-wbc` supplies
`wholebody/dex5/limits.py`, the canonical hand joint order.

### `.env`

`ferox-isaac-demo/.env` is **not** committed (it holds a machine-specific IP). Create it:

```bash
cd ~/panthera/ferox-isaac-demo
cat > .env <<EOF
FEROX_DDS_INTERFACE=tailscale0
FEROX_DDS_PEERS=$(tailscale ip -4 | head -1)
EOF
```

`FEROX_DDS_PEERS` is **your own tailscale IP and live participants only**. The Vast
address is ephemeral — never hardcode a previous one. Note that historical evidence
under `docs/twin/evidence/` contains the *old* box's tailnet address (`100.70.223.52`)
baked into captured audit output. It is dead, it is a private tailnet address, and it is
left in place because editing evidence to tidy it would make the evidence a lie. Do not
copy it into `.env`. DDS settings that must hold:
`network_mode: host`, CycloneDDS, `ROS_DOMAIN_ID=42`, `MaxAutoParticipantIndex=120`,
`ParticipantIndex=auto`, unicast pinned to `tailscale0`.

---

## 4. Bootstrap

```bash
cd ~/panthera/ferox-isaac-demo
./scripts/00_bootstrap.sh          # Isaac 5.1.0 image (~22 GB), builds ferox/msgs + ferox/nav,
                                   # creates cache dirs owned by UID 1234
```

Isaac Sim runs as **UID 1234**. `docker cp` writes as root, so anything copied in needs
`chmod -R a+rX` and cleanup needs `-u root`.

Old containers `panthera_om1_demo`, `panthera_nav`, `cmd_vel_publisher` must **never**
run on domain 42.

---

## 5. Build the twin assets

The sensor layers are generated, not committed as final state. The merged G1+Dex5 asset
**is** committed (38 MB) but is regenerable.

```bash
# start the sim once so the container exists (the builders run inside it)
ROBOT=g1 ./scripts/01_start_sim.sh

# G1 + Go2 sensor frames, authored from the contracts and read back
./scripts/08_build_twin_assets.sh g1        # expect: verified 8/8 frames
./scripts/08_build_twin_assets.sh go2       # expect: verified 5/5 frames

# the merged G1 + Dex5-1P asset (only if isaac/assets/g1_dex5 is missing or stale)
./scripts/12_import_g1_dex5.sh              # ~2 min; rebuilds the sensor layer itself
./scripts/11_import_dex5.sh                 # standalone hands, evidence for DT3
```

`12_import_g1_dex5.sh` `rm -rf`s `isaac/assets/g1_dex5` and then rebuilds the sensor
layer into it — it has to, because the import destroys what `08` authored. If you ever
run the import by hand, **run `08_build_twin_assets.sh g1_dex5` after it** or the sim
dies a world-load later with `frame 'livox_frame' is not in the stage`.

---

## 6. Run it

```bash
# G1 twin, with hands, camera TF on, vision props
ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=dso_block_a \
  FEROX_SIM_TEST_PROPS=1 CAMERA_TF=1 ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh

# Go2 twin
ROBOT=go2 TWIN=1 SIM_WORLD=hospital ./scripts/01_start_sim.sh
ROBOT=go2 MODE=twin ./scripts/02_start_ferox.sh
```

`ROBOT` must match between the two, or the mismatch guard stops you — that guard is
deliberate (`/tmp/sim_robot_type`, bypass with `FEROX_SKIP_SIM_CHECK=1` for multi-host
only).

### `render_dt` per robot — this one is not cosmetic

The RTX lidar is decimated off the render clock by an **integer** step, so the render
rate must be a whole multiple of the contract's lidar rate:

| robot | lidar | `render_dt` | arithmetic |
|---|---|---|---|
| **G1** | 10 Hz | **0.020 s** | 4 physics substeps → 50 Hz render, step 5 → 10 Hz |
| **Go2** | 20 Hz | **0.025 s** | 5 physics substeps → 40 Hz render, step 2 → 20 Hz |

`01_start_sim.sh` picks these per robot. Using 0.020 on the Go2 gives 50/20 = 2.5, the
gate rounds to 2, and **every** cloud, scan and accumulated cloud silently comes out at
25 Hz. `physics_dt` stays at 1/200 either way — it is what the walking policy runs
against, and changing it to make a topic rate come out round would be a far larger
deviation than the one it fixes (C-15, C-16).

---

## 7. Verify

```bash
cd ~/panthera/ferox-isaac-demo

# host-only, no containers, no Isaac (see §9 for what a clean clone gives)
python3 tools/tests/test_twin_contract.py       # expect 38/38
python3 tools/tests/test_isaaclab_cfg.py        # 13/13 with ferox-g1-locomotion beside
                                               # this repo; 10/10 + 3 SKIPPED without it

# needs the SIM CONTAINER (boots its own headless Isaac, ~90 s)
./scripts/10_test_twin_isaac.sh                 # expect 12/12

# needs sim + nav running
ROBOT=g1  ./scripts/07_twin_audit.sh --duration 25
ROBOT=go2 ./scripts/07_twin_audit.sh --duration 25
./scripts/15_check_camera_chain.sh              # C-21 proofs, expect RESULT: PASS
./scripts/14_capture_views.sh                   # 4 visual PNGs
./scripts/13_capture_hands.sh                   # 4 hand-pose PNGs
```

**Expected audit verdicts.** G1 live: Class-A conformant. Go2 live: **45 pass, 0
Class-A FAIL, 3 Class-B fail** (`/odom` 100 Hz = C-15, `/utlidar/imu` 200 Hz = C-16,
`driver_heartbeat` absent = driver-only). Anything else is a regression.

### Against the robot capture

See [`CAPTURES.md`](CAPTURES.md) for the fetch command, the sha256 and the expected
result (36 pass / 11 Class-A, nine of which are the camera the capture did not have).

---

## 8. Git and the token

Branch `mohammed/twin-campaign` in every repo touched. Commit per gate, tag
`twin-<gate>`, **push immediately after tagging**. **No `Co-Authored-By` trailers** —
Mohammed owns all commits.

GitHub's **git transport rejects `Authorization: Bearer`** for fine-grained PATs (the
REST API accepts it, which makes this confusing: git fails with "could not read
Username"). Use Basic with `x-access-token`, from the environment, never in a file or
argv:

```bash
read -rsp 'PAT: ' T; echo
export GIT_CONFIG_COUNT=1 \
       GIT_CONFIG_KEY_0="http.https://github.com/.extraheader" \
       GIT_CONFIG_VALUE_0="Authorization: Basic $(printf 'x-access-token:%s' "$T" | base64 -w0)"
git push origin mohammed/twin-campaign
git push -f origin <tag>
unset T GIT_CONFIG_COUNT GIT_CONFIG_KEY_0 GIT_CONFIG_VALUE_0
git config --get-all http.https://github.com/.extraheader || echo clean   # verify no residue
```

For the **REST API** (release assets), `Bearer` is correct — but pass the token through
the environment into a script rather than onto a curl command line, where `ps` can see
it. `tools/`-adjacent example: the uploader used for `twin-gt-g1` reads `GH_PAT` from
`os.environ`.

---

## 9. What a clean clone can and cannot do

Verified by actually doing it — `git clone` into `/tmp` with no access to this box's
caches, 2026-08-18. The clone is **118 MB** working tree + 35 MB `.git`:

| | works from a bare clone? |
|---|---|
| `test_twin_contract.py` (38 tests) | **yes** — pure YAML + AST, no ROS, no Isaac |
| `test_isaaclab_cfg.py` (13 tests) | **yes** — `13/13` with `ferox-g1-locomotion` beside this repo, `10/10 passed, 3 SKIPPED` without it. The skips are counted and named, not folded into the pass total: a test that degrades to a no-op and still reports green is worse than a missing one, and this harness used to do exactly that |
| `tools/merge_dex5_urdf.py` | needs `~/panthera/ref/unitree_ros` (§3) |
| `test_twin_isaac.py` (12 tests) | **no** — needs the **sim container**; run via `scripts/10_test_twin_isaac.sh` |
| `07_twin_audit.sh`, `15_check_camera_chain.sh` | **no** — need the nav container and a live graph (or `--bag`) |
| `13_/14_capture_*.sh` | **no** — need the sim container |

Nothing in the host-side test suites depends on this box's state. The Isaac suite
genuinely cannot: it boots Isaac and reads the built USD.

---

## 10. Where to read next

| | |
|---|---|
| One-page status | [`RESULTS_FASTPATH.md`](RESULTS_FASTPATH.md) |
| Every deviation C-1 … C-21 | [`TWIN_DEVIATIONS.md`](TWIN_DEVIATIONS.md) |
| Per-gate detail | `RESULTS_DT0/1/2/3/5.md`, `RESULTS_GT_G1.md` |
| The brief and standing rules | [`CAMPAIGN.md`](CAMPAIGN.md) |
| Decision history, including resolved forks | [`PING.md`](PING.md) |
| Running log, newest at the bottom | [`PROGRESS.md`](PROGRESS.md) |
| Captures and how to fetch them | [`CAPTURES.md`](CAPTURES.md) |
| Nav2 planner crash for Ferox | [`ISSUE_planner_segfault.md`](ISSUE_planner_segfault.md) |
