# RESULTS_DT0 — box ready

Host: Vast.ai VM (KVM), AMD EPYC 7V73X 30 vCPU / 98 GiB, **RTX 4090 49140 MiB**, driver 580.105.08, CUDA 13.0
Date: 2026-08-17
Verdict: **PASS-with-deviations** — the gate's PASS criterion is met for both robots; two *prerequisites*
(step 2 clones, step-5 push) are blocked on a GitHub PAT that this session does not have.

---

## Scorecard

| # | Requirement | Status | Evidence |
|---|---|---|---|
| 1 | `nvidia-smi`, driver, VRAM | PASS — RTX 4090, 49140 MiB, driver 580.105.08 | §Host above |
| 1 | Docker + nvidia runtime + toolkit | PASS — Docker 29.0.3, `nvidia` runtime registered, toolkit 1.18.0-1 | `docker info`, `nvidia-ctk --version` |
| 1 | Disk ≥150 GB free | PASS — 241 G free of 291 G | `df -h /` |
| 1 | `tailscale ip -4` | PASS — 100.70.223.52 on `tailscale0` | — |
| 1 | VM with Docker (not a bare container) | PASS — `systemd-detect-virt` = `kvm`, no `/.dockerenv` | §8 Q5 answered |
| 2 | `Ferox`, `ferox-isaac-demo`, `ferox-g1-locomotion` cloned | PASS | `~/panthera/` |
| 2 | `panthera-g1-driver` read-only under `ref/` | PASS | `~/panthera/ref/panthera-g1-driver` |
| 2 | wbc W7 branch under `ref/` | PASS — branch `mohammed/g1wb-review-w7-docs` (full clone, 417 MB, not sparse) | `~/panthera/ref/panthera-g1-wbc` |
| 2 | `panthera-go2-driver`, `realsense_driver` | **BLOCKED** — private, no credentials on this box | `git ls-remote` → `could not read Username` |
| 3 | `.env` with `FEROX_DDS_INTERFACE` / `FEROX_DDS_PEERS` | PASS — `tailscale0`, own IP only, gitignored | `.env` |
| 4 | `00_bootstrap.sh` | PASS (run earlier this session, before this agent turn) — `cache/{kit,ov,pip,compute,gl,warp}` all present and owned by UID 1234 | `ls -la cache/` |
| 4 | `01_start_sim.sh` (Go2) | PASS — main loop reached | `go2_sim_topics.txt` |
| 4 | `01_start_sim.sh` (G1) | PASS — main loop at 60 s | `g1_sim_log_tail.txt` |
| 4 | `VENUE=dso_block_a 02_start_ferox.sh` | PASS both robots | `g1_nav_lifecycle.txt`, `go2_nav_lifecycle.txt` |
| 4 | **Nav2 7/7 lifecycle active** | **PASS both** — all 7 `active [3]` | same |
| 4 | One goal each | **PASS both** — `SUCCEEDED` | `g1_nav_goal.txt`, `go2_nav_goal.txt` |
| 4 | `sim.log` tail 100, `topic list`, `topic hz` `/scan /odom /imu` | PASS (`/imu` absent — see D-2) | `*_sim_{log_tail,topics,rates}.txt` |
| 4 | Viewport PNG | PASS — X capture on `:0` after dismissing the KDE locker | `g1_viewport.png`, `go2_viewport.png` |
| 5 | `docs/twin/DT0_BASELINE.md` | PASS | `docs/twin/DT0_BASELINE.md` |
| §9.3 | `CAMPAIGN.md` saved in repo | PASS | `docs/twin/CAMPAIGN.md` |
| §0.2 | Branch `mohammed/twin-campaign` | PASS in `ferox-isaac-demo` (only repo touched) | — |
| §0.2 | Commit, tag `twin-DT0`, **push** | **BLOCKED** — needs PAT | — |
| §0.8 | Forbidden containers absent | PASS — none of the three exist | `docker ps -a` |
| §0.6 | Sim/nav guard + UID 1234 | PASS — guard fired correctly, container runs as `uid=1234(isaac-sim)` | `docker exec ferox_isaac_sim id` |

---

## What changed

| File | One line |
|---|---|
| `docs/twin/CAMPAIGN.md` | the campaign brief, amendments 1–4 folded in (new) |
| `docs/twin/DT0_BASELINE.md` | the measured "before": topics, frames, rates, prim paths, offsets, stand-ins (new) |
| `docs/twin/RESULTS_DT0.md` | this file (new) |
| `docs/twin/evidence/DT0/*` | 20 evidence artefacts (topic lists, QoS, rates, message samples, TF dumps, SLAM maps, viewports) |

No source file was modified in DT0. One host-side config change outside the repo:
`/root/.config/kscreenlockerrc` → `Autolock=false` (the KDE locker blanks the X root and makes
viewport capture impossible; overnight gates would have lost every screenshot).

---

## Baseline defects found (all Class A, all fixed by DT2/DT5)

| ID | Defect | Where |
|---|---|---|
| **B-1** | On the **G1**, sensor prims sit at G1 offsets but `/tf_static` publishes the **Go2** offsets — camera off by 0.30 m x / 0.65 m z, scan origin off by 0.05 m x / 0.30 m z. `setup_static_tfs()` takes no pose arguments; its table is commented *"static transforms for Go2"*. | `isaac/sim_utils.py:566-612` vs `isaac/run.py:967-976` |
| **B-2** | The sim publishes a **static identity `map → odom`** while `slam_toolbox` publishes the same edge dynamically — two owners of one TF edge. | `isaac/sim_utils.py:611` |
| **B-3** | `laser` has two parents: sim `velodyne_base_link → laser`, Ferox sim bridge `lidar_l1_link → laser`. | sim + `isaac_bridge.launch.py` |
| **B-4** | **No IMU topic on either robot, two faults deep.** (a) `/ImuGraph` *is* built (`IsaacReadIMU`→`ROS2PublishImu`, `frameId imu_link`, `topicName imu/data`) and logs `[ROS2] IMU publisher -> imu/data`, yet neither `/imu` nor `/imu/data` is ever advertised. (b) It sets no `nodeNamespace`, so it would emit `/imu/data` while the bridge relays `/imu`. `/ferox/<id>/imu/data` is permanently silent. | `isaac/sim_utils.py:973-1006`, `isaac_bridge.launch.py:54` |
| **B-5** | `/scan` is 3200 rays in frame `laser`, 0.05–30 m, `angle_increment` 0.00196 — matches neither robot (hardware: 723 rays, `base_link`, 0.30–6.0 m, 0.0087). Identical on both robots. | measured |
| **B-6** | Camera is 480×270, depth `32FC1` metres, both in `camera_optical`, colour and depth on **different stamps**, no `depth/color/points`. Hardware: 1280×720, `16UC1` mm, real optical frames, same stamp. | measured |

Full detail and the 11-edge dump: `docs/twin/DT0_BASELINE.md`.

---

## Deviations (Class C) — none yet

DT0 changes nothing, so there is nothing to declare. `TWIN_DEVIATIONS.md` is created in DT1.

---

## Note on the SLAM cold start (not a defect)

At the warehouse spawn the nearest lidar return is **8.8 m**. The Go2 SLAM profile uses
`max_laser_range: 6.0` (`go2_slam.yaml:59`), so 0 of 2423 valid returns are usable, the map stays
0×0 and Nav2 logs `Received map message is malformed. Rejecting.` until the robot has driven
~3.5 m. The G1 profile uses `12.0` (`g1_slam.yaml:35`) and maps immediately. Both goals in this
gate were sent against a live map. Recorded so nobody re-diagnoses it as a bug.

---

## Open questions for Mohammed

1. **PAT needed (blocking).** `panthera-go2-driver` and `realsense_driver` are private and not on
   this box. DT1 cannot write `go2_contract.yaml` or the camera half of `g1_contract.yaml` without
   them. *Default while waiting:* none — this is a hard stop for those two files; the G1 lidar/odom/
   IMU/scan half of DT1 can proceed from `panthera-g1-driver`, which is present.
2. **PAT needed (blocking).** DT0's commit + tag `twin-DT0` + push. The work is committed locally
   the moment a token is available; nothing is pushed yet, so this instance dying loses it (§0.2).
3. Real `camera_info` (colour + `aligned_depth_to_color`) from G1 #1 → exact K/D.
   *Default taken:* D435i factory-typical values, marked `assumed`, in DT1.
4. A 30 s bag from each robot as `twin_audit` ground truth.
   *Default taken:* `--against-evidence` mode against `panthera-g1-driver/evidence/*`.
5. Dex5-1P wrist adapter offset on the G1. *Default taken:* Dex3 flange offset from
   `g1_29dof_with_hand.urdf`, flagged `assumed` (DT3).
6. Is a D435i mounted on Go2 #1, and where? *Default taken:* variant `camera=none` (DT5).
7. Colour/material reference photos of both robots. *Default taken:* Unitree product renders (DT2).

---

## Reproduce

```bash
cd ~/panthera/ferox-isaac-demo

# Go2
./scripts/00_bootstrap.sh
./scripts/01_start_sim.sh
VENUE=dso_block_a ./scripts/02_start_ferox.sh
./scripts/05_send_goal.sh 5.5 0            # after driving ~3.5 m, see the SLAM note

# G1
ROBOT=g1 ROBOT_ID=g1_01 ./scripts/01_start_sim.sh
ROBOT=g1 ROBOT_ID=g1_01 VENUE=dso_block_a ./scripts/02_start_ferox.sh
ROBOT=g1 ROBOT_ID=g1_01 ./scripts/05_send_goal.sh 5.5 0

# Evidence
docker exec ferox_isaac_sim tail -100 /tmp/sim.log
docker exec ferox_nav bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash;
  export ROS_DOMAIN_ID=42; ros2 topic list; ros2 topic hz /scan; ros2 topic hz /odom'
for n in controller_server smoother_server planner_server behavior_server \
         bt_navigator waypoint_follower velocity_smoother; do
  docker exec ferox_nav bash -lc "source /opt/ros/humble/setup.bash;
    source /workspace/install/setup.bash; ROS_DOMAIN_ID=42 ros2 lifecycle get /ferox/g1_01/$n"
done

# Viewport PNG (X on :0)
DISPLAY=:0 import -window root out.png
```

**Host-side prerequisite — dies with the VM, redo it on every fresh box.** KDE's screen locker
blanks the X root window, so `import -window root` captures a lock screen instead of the Isaac
Sim viewport, and an obscured window cannot be captured directly either
(`unable to read X window image ... Resource temporarily unavailable`). Disable autolock before
any gate that takes screenshots:

```bash
printf '[Daemon]\nAutolock=false\nLockOnResume=false\nTimeout=0\n' > ~/.config/kscreenlockerrc
# if a locker is already up:
DISPLAY=:0 loginctl unlock-sessions; pkill -f kscreenlocker_greet
```

This is host state, not repo state — it is not captured by any commit.
