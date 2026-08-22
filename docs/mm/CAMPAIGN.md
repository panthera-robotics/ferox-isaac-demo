# Ferox Motion & Manipulation campaign (MM0–MM9) — Claude Code prompt

> ## STATUS — 2026-08-20 (MM campaign, gates MM0–MM5 run)
>
> | gate | verdict | one line |
> |---|---|---|
> | MM0 | PASS-with-deviations | box conforms; camera items deferred (C-23) |
> | MM1 | PASS-with-deviations | locomotion contract + motion suite; yaw root-caused |
> | MM1b | **PARKED** | retrain finished 6000/6000 and is not a walking policy — `PROVENANCE_v2_mm1b_c.md` |
> | MM2 | PARTIAL | venue built and verified; nav 0/6 (C-26), door does not open |
> | MM3 | **PASS** | low-level DDS bridge; (a) rewritten as (a1)/(a2) |
> | MM4 | **RE-SCOPED — omni-hold** | C-39 closed: the *reference* body falls here too, so SONIC is **hardware-only** and the **omni policy is the twin's balancer** (6.159 m walk on film). SONIC x86 built and closed-loop; balancing requirement met by omni-hold |
> | MM5 | see RESULTS_MM5 | fixed-base variant is the result; mobile parked behind C-39 |
> | MM6–MM8 | not started | MM6/MM7 need the camera (C-23); MM8 is media |
> | MM9 | this pass | persistence |
>
> **The one thing blocking mobile manipulation is C-39**, and it is not the twin's wire:
> the conventions were diffed field-by-field against the reference MuJoCo bridge and the
> DT bag and they match (`evidence/MM4/CONVENTION_TABLE.md`). SONIC's own commanded
> targets are out of range — 3 of 29 beyond the URDF limits, a 1.648 rad knee against its
> own 0.30 nominal — with the robot upright and its observations clean.
>
> ## STATUS — 2026-08-19
>
> **MM0 RUNNING on an RTX 4080 SUPER 16 GB, by decision. The 4090 is required for
> CAMERA-DEPENDENT ITEMS ONLY.**
>
> Mohammed, 2026-08-19, amending §0/§5 of this brief as originally written:
>
> | | |
> |---|---|
> | Box | `NVIDIA GeForce RTX 4080 SUPER, 16376 MiB, driver 580.105.08` |
> | **camera-capable box** | **no** — C-23, the ROS 2 image writer segfaults here (5 boots for 5) |
> | Everything else | runs here with `TWIN_CAMERA=0`, **declared in every RESULTS header** |
>
> **Camera-dependent items, queued for a 4090 day:**
>
> * MM0.2 — aligned-depth check and the C-21 clip
> * anything `ferox_vision` (E-1, and MM5's `--pose vision` path)
> * **MM6** and **MM7** entirely — GR00T runs on twin pixels, and the recorder
>   records them
> * MM8 — the **PiP camera** track (the montage's other tracks are offscreen renders
>   and are fine here)
>
> **Everything else runs on this box**, including the SONIC x86 build (MM4) and the
> low-level bridge (MM3). If MM1b needs a locomotion retrain, use **≤2048 envs** —
> 16 GB is tight for 4096.
>
> The original text below is unchanged except where this header overrides it. Where
> it says "never a 4080", read: never for camera items.

You are the agent for the **Ferox Motion & Manipulation campaign** on the G1 digital twin. The Digital
Twin campaign (DT0–DT8, tag `twin-g1-fixed-2`) delivered a hardware-parity G1 in Isaac Sim 5.1: Unitree
body at 1:1, Dex5-1P hands, Mid-360 + D435i at calibrated poses, the real driver's ROS 2 topics/frames,
audited against a real bag. This campaign makes it **move and manipulate**: a full locomotion test suite
with working yaw, a lab environment with an articulated door and objects, the low-level `rt/lowcmd` /
`rt/lowstate` bridge, SONIC whole-body control running against the twin, a scripted navigate → approach →
pick → place pipeline, GR00T plumbing plus a data-collection loop, and a proper motion video. Everything
runs on a **fresh Vast.ai RTX 4090 VM (x86_64)** — never a 4080 (RESUME.md §1: the image writer segfaults
there, C-23). The DGX Spark is used only for GR00T finetuning (MM7); nothing here touches a robot.

> **AMENDED 2026-08-19 (see the status header):** the 4090 is required for
> **camera-dependent items only**. Everything else runs on a 4080 with
> `TWIN_CAMERA=0`, declared in every RESULTS header.

Go2 is out of scope for this campaign.

---

## 0. Standing rules (unchanged from the DT campaign, non-negotiable)

1. **Investigate before changing; diffs before edits; one gate per turn** unless the run is authorized
   overnight. PASS/FAIL with evidence, then continue or stop as the gate says.
2. **Git**: branch `mohammed/mm-campaign` in every repo you touch (ferox-isaac-demo, Ferox,
   ferox-g1-locomotion, panthera-g1-wbc if needed). Commit per task, push after every commit, tag
   `mm-MM<n>` per gate. **No `Co-Authored-By: Claude`.** PAT: shell env only, Basic `x-access-token`
   via `GIT_CONFIG_*` (RESUME.md §PAT), verify no residue after every push, unset when done.
3. **Docker immutability**: bake deps into Dockerfiles; nothing installed live survives an `up`.
4. **DDS**: CycloneDDS, `MaxAutoParticipantIndex=120`, `ParticipantIndex=auto`, own-IP peers only,
   Ferox on domain 42. Robot-emulation buses (`rt/lowcmd`, `rt/lowstate`, `rt/dex3/*`) get their own
   domain (default 0 like the robot, overridable) and **must never be reachable from a real robot** — the
   VM has no route to 192.168.123.0/24; assert it at boot.
5. Sim/nav robot-mismatch guard, seven-bug checklist, UID 1234, `00_bootstrap` before `01_start_sim`.
6. **Never scale a mesh or offset to fit; never weld a grasp into a success metric.** A `--cheat-attach`
   flag may exist for plumbing tests only, is logged as such, and never counts as a grasp.
7. **Derived artifacts get audited**; sim policies get their obs contract checked against training, not
   assumed.
8. Anything you can't measure, you don't claim. Numbers with units and provenance in every report.

---

## 1. Read first (ground truth, in order)

| Path | What it gives you |
|---|---|
| `ferox-isaac-demo/docs/twin/CAMPAIGN.md` (status header, §0, §3), `RESUME.md` (end to end), `RESULTS_FASTPATH.md`, `RESULTS_DT2/DT3.md`, `TWIN_DEVIATIONS.md`, `isaac/twin/g1_contract.yaml`, `isaac/twin/lidar.py`, `isaac/run.py`, `isaac/sim_utils.py`, `tools/build_twin_assets.py`, `12_import_g1_dex5.sh` | The twin: how it is built, its open items (C-13 passive fingers, C-23 GPU, hand-roll check, nav 3/3, camera items) |
| `ferox-g1-locomotion/CLAUDE.md`, `DEV_LOG.md`, `policy/PROVENANCE_g1_omni.md`, `policy/params/{env,agent,deploy}.yaml`, `scripts/validate_motion.py` | The G1 omni policy contract: 29 joints, obs/action layout, command ranges (vx [-0.6, 1.0], vy [-0.5, 0.5], wz [-1, 1]), the known rotate-in-place FAIL (wz +0.20 cmd → +0.015 measured) |
| `Ferox/src/ferox_nav/{config,launch}`, `ferox_nav/waypoint_manager.py` (`MoveToNamed`), venue YAMLs, `ferox_nav_sim` twin bridge | Nav side: `mode:=twin`, MPPI omni + `06_motion_mode.sh` walk/omni toggle, waypoint semantics |
| `panthera-g1-wbc` branch `mohammed/g1wb-review-w7-docs`: `RESULTS_W1..W6.md`, `docker/g1wb-*`, `configs/wbc/*.yaml`, `wholebody/dex5/*`, `tools/*`, `README`/`CLAUDE.md` of that branch; upstream `gear_sonic_deploy` (in-tree thirdparty) | SONIC v1.1 as we run it: same C++ binary `deploy.sh … sim\|real`; sim = DDS `rt/lowcmd`/`rt/lowstate` (unitree_mujoco convention); token mode obs `[154]`, VR `[178]`, out `[29]` normalized → PD 50 Hz; kinematic planner (vx vy wz heading); upper-body POSE mode; W3 loop closure in MuJoCo; W4 conditional (hand never reached the can); W5 manipulation port; W6 GR00T head on sim pixels; W7 robot bridge; data exporter (ZMQ 5555 cam / 5556 pose / 5557 state → LeRobot v2.1); Kevin's fail-closed PR |
| Project docs: `g1_wholebody_stack_guide` (three sims, gates G0–G5, hybrid split, what transfers) | Strategy context — the twin now replaces MuJoCo's job for anything with pixels |
| Upstream (read-only): `unitreerobotics/unitree_mujoco` (the DDS low-level bridge to copy semantics from), `unitree_sdk2_python` (`unitree_hg` `LowCmd_/LowState_`, `g1_low_level_example.py`), `unitree_sim_isaaclab` (Isaac Lab + DDS lowstate/lowcmd pattern, Dex3 tasks), `unitree_rl_lab` (G1 obs layout, gait-phase terms, training config), NVIDIA `Isaac-GR00T` (PolicyServer, N1.7), Isaac Sim assets `Isaac/Props/YCB/*`, `Isaac/Props/Cabinet`, `Isaac/Environments/*` | Vendor truth |

Read the omni policy's `env.yaml` observation block and `run.py`'s obs builder side by side before touching
yaw — the yaw defect may be a contract mismatch, not a policy weakness (§4.1).

---

## 2. Definition of done

| Capability | Acceptance |
|---|---|
| Locomotion suite | 8 directions × 3 speeds, rotate-in-place ±0.3/±0.6/±1.0 rad/s, turn-while-walking, stop-from-speed, 60 s each, hands on: velocity tracking error ≤20 % (≤0.1 rad/s absolute for yaw), never falls, base height in band; automated table + video per test |
| Environment | `panthera_lab` world: room, table, shelf, articulated door, objects at real dimensions/masses; nav map + venue YAML; lidar and camera see it; sim/nav guard knows it |
| Low-level bridge | `rt/lowstate` 500 Hz + `rt/lowcmd` PD applied at 500 Hz; unitree_sdk2py stand example stands the twin; fail-closed on cmd loss |
| SONIC in twin | `deploy.sh … sim` (x86 build) stands, walks on planner commands incl. heading turn, POSE-mode arms while balancing, hands attached; metrics table |
| Manipulation pipeline | nav to table → approach → grasp can → lift → carry 1 m → place on table; N=20 randomized trials, success rate reported honestly with failure taxonomy; no cheat-attach in the metric |
| GR00T plumbing + data | PolicyServer runs on the twin's D435i + proprio; token/arm actions flow to SONIC; 20 recorded expert episodes validate through `process_dataset.py` (LeRobot v2.1, Dex5 20-dim + tactile column) |
| Video | ≤3 min montage of real motion, chase/fixed/PiP cameras, no ghosting, plus 15–20 s per-capability clips |

---

## 3. Numbers and facts to carry (do not re-derive)

- G1 twin: 69 DOF articulation, first 29 = body in `deploy.yaml` order (asserted by test); Dex5 joints by
  NAME only (C-14); passive fingers held at zero (C-13) — couple them 1:1 (PhysX mimic joint API) in MM3
  or declare; standing pelvis 0.79 m in sim (0.731 real, C-1); mass 35.005 kg with hands.
- Sensors: Mid-360 inverted in head, 360° accumulated sweeps, floor plane ≤0.5°; D435i K from contract
  (`assumed` until captured), 1280×720 rgb8 + 16UC1 mm same stamp; camera verified only on 4090.
- Real robot control paths: Ferox drives Unitree native gait via sport `api_id 7105`; SONIC on hardware
  uses `rt/lowcmd` — **mutual exclusion**, also in sim: the omni-policy path (cmd_vel → policy) and the
  lowcmd path (SONIC) never both drive the same articulation; one env switch `G1_CONTROL=omni|lowcmd`.
- SONIC runs on x86 with TensorRT 10.13 per the guide; our W images are aarch64 (Spark) — an x86 image
  is part of MM4.
- Isaac Lab pin for any locomotion retrain: 2.3.2 (PROVENANCE); unitree_rl_lab G1 29dof task; export
  TorchScript + ONNX; regenerate `deploy.yaml`; new PROVENANCE file.
- Video: the DT montage had motion ghosting (temporal accumulation while the robot moved). Renders must
  converge per frame (`rep.orchestrator.step(rt_subframes=N)` or equivalent) — a frame-diff test proves
  no trails.

---

## 4. Locked architecture decisions

**4.1 Yaw is diagnosed before it is retrained.** Order: (a) play the g1_omni checkpoint in Isaac Lab
(unitree_rl_lab, same env config) with wz commands — does it turn there? (b) dump the obs vector from
`run.py` and from the Isaac Lab env at matched states; diff layout, scales, history stacking, gait-phase
terms, command index/sign/units, ang-vel frame; (c) if it turns in Isaac Lab and not in `run.py`, fix
`run.py` (contract bug); (d) only if it does not turn in Isaac Lab either → MM1b retrain on the 4090 with
the **twin USD** (hands attached, mass right), full ranges, direct yaw-rate command, ~4096 envs, export,
deploy.yaml, PROVENANCE, same acceptance suite. Either way the suite in MM1 is the arbiter.

**4.2 Two control paths, one switch.** `G1_CONTROL=omni` (default; cmd_vel → omni policy, what Ferox nav
uses) or `G1_CONTROL=lowcmd` (articulation driven only by `rt/lowcmd`; SONIC or the sdk2 example). The
low-level bridge is a Panthera module in `isaac/twin/lowlevel_bridge/`, semantics copied from
unitree_mujoco (`unitree_hg LowState_`: 29 motors q/dq/ddq/tau_est, IMU quaternion/gyro/accel/rpy,
`mode_machine`, tick; `LowCmd_`: per-motor mode/q/dq/kp/kd/tau, `mode_pr`, CRC), PD computed by us at
500 Hz physics (`tau = kp(q_d−q) + kd(dq_d−dq) + tau_ff`, clamped to URDF effort limits), fail-closed
damping mode when no cmd for 100 ms, and — the whole point of the twin — the same D435i/Mid-360/ROS
topics still publishing while SONIC drives.

**4.3 Environment as an asset with provenance.** `isaac/assets/worlds/panthera_lab/` built by
`tools/build_lab_world.py`: 8 × 6 m room, 2.7 m ceiling; table 1.2 × 0.8 × 0.75 m; counter/shelf; door
2.10 × 0.90 m in a frame, revolute hinge 0–110°, damping + a light closer spring, lever handle at 1.05 m
(all as a PhysX articulation, real masses ~35 kg leaf); objects from Isaac's YCB set at true scale (tomato
soup can 005, mustard 006, cracker box 003, sugar box 004, mug 025, banana 011) plus a 5 cm cube and a
brochure-sized box; masses/friction set explicitly (soup can 0.349 kg); floor material like the DSO
carpet/tiles; ceiling and walls so the Mid-360 gets returns everywhere. Ferox: `venues/panthera_lab.yaml`
with waypoints `home`, `table_approach`, `table`, `shelf`, `door_inside`, `door_outside`; map built by
SLAM once and saved; `SIM_WORLD=panthera_lab` in `01_start_sim.sh`; guard records it. Object placement
randomization via a seed argument (used by MM5/MM7).

**4.4 Manipulation = deterministic first, learned second.** The "hybrid split" from the guide: SONIC (or the
omni policy standing) balances; arms driven by IK; Dex5 closes on contact. Scripted expert lives in
`isaac/twin/manip/` (`approach.py`, `reach_ik.py`, `grasp.py`, `place.py`, `pipeline.py`) with object pose
from ground truth by default (`--pose gt`) and from `ferox_vision`/AprilTag when available (`--pose
vision`). Grasp physics is real PhysX: contact-sensor tactile proxy triggers close; friction/finger drives
tuned within declared bounds and written to TWIN_DEVIATIONS; failure taxonomy (miss, slip, drop, collision,
timeout). Success rate is reported as measured.

**4.5 GR00T pipeline mirrors the W-campaign, with the twin as the sim.** The W5 manipulation port + W7
robot bridge pattern (GR00T PolicyServer → token/arm targets → SONIC; state via ZMQ 5557, camera 5555)
is re-pointed at the twin: a small adapter publishes the twin's D435i color to ZMQ 5555 at 30 Hz and the
low-level state to 5557; the data exporter records LeRobot v2.1 with `observation.images.*` from the
twin's camera (real K, real mount), 29 body + 20 Dex5 + tactile columns. Finetuning stays on the Spark
(existing W6 pipeline); the 4090 does inference and data collection.

**4.6 Video is a first-class deliverable, produced by tooling.** `tools/film.py`: chase camera
(follow with lag), two fixed cams, PiP of the robot's D435i, optional lidar overlay; 30 fps; per-frame
convergence; title cards; ffmpeg stitch; frame-diff ghosting test. Every motion gate produces its clip
with this tool, so MM8 is assembly, not re-shooting.

---

## 5. Gates

Each gate: `docs/mm/RESULTS_MM<n>.md` (verdict, scorecard, evidence, deviations, open questions,
reproduce), evidence under `docs/mm/evidence/MM<n>/`, clip under `docs/mm/media/`, commit + tag pushed.

### MM0 — box + carry-over (½–1 day)
1. RESUME.md through verification. `nvidia-smi` model/VRAM is read and recorded before
   anything, but on a non-4090 box it is a **flag, not a gate**: report
   `camera-capable box: no` and carry the camera items forward as declared open.
2. DT carry-over, all short: hand-roll numeric check at URDF zero pose (per hand, world-frame:
   fingers·body+X, palm normal·toward-sagittal-plane, thumb·world+Z, all ≥0.9; renders top/front/side);
   aligned-depth check + C-21 clip; `/scan` clip re-shot in hospital (≥45 % finite beside the bag);
   nav 3/3 with goals inside the measured free-space bounds. Close or reopen the corresponding C items.
3. `tools/film.py` v1 with the ghosting test; 20 s test clip of the twin walking (omni policy) — no trails.
   **PASS**: box conforms, carry-over closed, film tool proven.
   On a non-camera box the achievable verdict is **PASS-with-deviations**: the five
   GPU-independent sub-items closed, the camera sub-items open and queued.

### MM1 — locomotion contract + motion suite (1–2 days; MM1b +½–1 day if retrain)
Per §4.1. Deliver `scripts/motion_suite.py` (extends `validate_motion.py`): 8 directions × {0.2, 0.5,
0.8} m/s, rotate ±0.3/±0.6/±1.0 rad/s, turn-while-walking (0.4 m/s + ±0.5 rad/s), stop-from-0.8, each 60 s,
hands on; metrics: tracking error, yaw-rate tracking, base height mean/σ, roll/pitch σ, foot slip proxy,
falls; markdown table + JSON; clip per test via `film.py`. Also update Ferox nav for real turning: MPPI
motion model back to forward+yaw when yaw works (keep the `06_motion_mode.sh` toggle), goal heading
tolerance tightened, `PreferForwardCritic` retained. **PASS**: §2 locomotion acceptance; yaw root cause
written down (contract bug vs retrain).

### MM2 — `panthera_lab` environment (1 day)
Per §4.3. Evidence: top-down render with dimensions, door articulation opening/closing under a scripted
force with joint-angle plot, object table (name, size, mass, friction, source), lidar cloud + `/scan` in the
room (finite ratio ≥60 %), camera view of the table with objects, SLAM map saved, Ferox `MoveToNamed`
to every waypoint 1/1 in `mode:=twin`. **PASS**: all six waypoints reached, door articulates, audit still
0 Class-A.

### MM3 — low-level DDS bridge (1–2 days)
Per §4.2. Tests: (a) `unitree_sdk2py` `g1_low_level_example`-style PD stand script stands the twin for 60 s;
(b) cmd-loss → damping within 100 ms; (c) `rt/lowstate` 500 Hz ±2 %, field-by-field parity with a real
`/lowstate` message from the DT bag (`captures/g1/…` decodes only in an image with `unitree_hg` — bake it
into the twin bridge image); (d) ROS topics keep publishing while `G1_CONTROL=lowcmd`; (e) passive Dex5
coupling done or declared; (f) `rt/dex3/{left,right}/{cmd,state}` 20-entry wire (DT6, folded in here:
HandCmd_ → hand drives, HandState_ ≥200 Hz with `press_sensor_state` from contact zones). **PASS**: a–f.

### MM4 — SONIC in the twin (2 days)
x86 image for `gear_sonic_deploy` (TensorRT 10.13, CUDA on the 4090; start from `docker/g1wb-*` recipes,
document every aarch64→x86 delta), Kevin's fail-closed semantics on, `deploy.sh … sim` pointed at the twin's
DDS domain. Sequence: stand → weight shift → planner walking (vx, vy, wz, heading) → heading turn-in-place
→ stop → POSE-mode arms-only targets while balancing → all with Dex5 hands attached. Metrics as MM1 plus
policy latency and PD tracking; clip per step. **PASS**: SONIC W3-equivalent closed in Isaac (lowstate →
policy → lowcmd), turns in place, POSE arms work; SONIC and omni never co-drive (asserted).

### MM5 — scripted navigate → pick → place (2 days; door stretch +1)
Per §4.4. Pipeline: `MoveToNamed(table_approach)` (omni policy) → hand-off to `G1_CONTROL=lowcmd` with SONIC
standing (or omni standing if MM4 slips — say which) → approach controller to a reach pose (pose from
`--pose gt`) → IK reach (Lula/cuRobo/Isaac Lab differential IK, document choice) → Dex5 close on contact →
lift → carry 1 m (SONIC planner or omni at 0.3 m/s) → place on the table → release → retreat. N=20 trials,
object pose randomized (seed), success rate + taxonomy + per-stage timing; failure clips kept. Stretch:
door — push it open from `door_inside` with the palm along a scripted arc, walk through to `door_outside`.
**PASS**: pipeline runs end to end without cheat-attach; success rate reported (no threshold — the number is
the deliverable), each failure classified.

### MM6 — GR00T plumbing + recorder (1–2 days)
Per §4.5. PolicyServer (N1.7 base or the W6 head) on the 4090 consuming twin camera + state; actions reach
SONIC/arms; measure loop latency; qualitative reach toward the can (expect no grasp — pipeline proof).
Recorder: 20 expert episodes from MM5 with twin pixels, validated by `process_dataset.py`; dataset manifest
(sha256, episode counts, modality.json incl. Dex5 + tactile). **PASS**: latency table, dataset validated.

### MM7 — data loop + finetune + closed-loop eval (2 days agent + Spark hours)
100–200 expert episodes (randomized poses, lighting jitter, 2 object types) → transfer to the Spark
(rsync over tailscale or release asset; sha256) → finetune the head with the W6 recipe (Spark, ~6k steps)
→ checkpoint back → closed-loop eval in the twin: success rate on 20 held-out trials vs the scripted expert;
open-loop error on held-out episodes. Write what transfers and what won't (pixels are now the twin's own
D435i — the argument for fewer real episodes later). **PASS**: numbers reported; go/no-go for real-data P4.

### MM8 — video (½ day)
`docs/mm/media/ferox_g1_motion_manip_<date>.mp4`, ≤3 min: locomotion suite highlights (all directions,
turning), lab tour, door, SONIC stand/walk/turn/POSE arms, pick-place full run + one failure, GR00T
reach, PiP camera + lidar overlay throughout, closing scorecard; plus one 15–20 s clip per capability.
Release upload with sha256 in CAPTURES.md. **PASS**: ghosting test green on every clip.

### MM9 — persistence (½ day)
RESUME.md, CAPTURES.md, TWIN_DEVIATIONS.md, CAMPAIGN status headers, tag `mm-persist`, all repos in sync;
"you are here" verbatim.

Order: MM0 → MM1 → MM2 → MM3 → MM4 → MM5 → MM8 (interim video) → MM6 → MM7 → MM8 (final) → MM9.
Overnight pre-authorized after MM0 passes; §5 ping rules of the DT campaign apply (fail twice, decision
needed, destructive); PROGRESS.md at every task boundary; time-box 90 min per task with the documented
fallback; instance at risk → push + PING.

Estimates: ~12–15 agent-days end to end; MM0–MM5 + interim video ≈ 8.

---

## 6. Reporting format

```
# RESULTS_MM<n> — <title>
Host: <vast id / GPU / driver>   Date:   Verdict: PASS | FAIL | PASS-with-deviations
## Scorecard  (requirement | status | evidence path)
## What changed (files, one line each; commit/tag)
## Numbers (tables; units; provenance)
## Clips (path, length, what it shows)
## Deviations (C-xx, one line each)
## Open questions for Mohammed (numbered; default taken)
## Reproduce (exact commands)
```

---

## 7. Open questions (ask at the first gate that needs them; proceed with the default)

1. First manipulation object/task: soup can table→table (default) vs brochure handover.
2. Door: push-open only (default) vs handle-turn.
3. SONIC x86 build: reuse `docker/g1wb-*` layering (default) vs upstream `gear_sonic_deploy` docs image.
4. Spark availability window for MM7 finetune; transfer path (rsync over tailscale default).
5. Lab dimensions/props to match a real venue corner (default: generic lab per §4.3).
6. Isaac Lab retrain budget if MM1 says the policy is weak (default: yes, one overnight run).

---

## 8. Session kickoff (Mohammed, once, on the VM)

1. Confirm `nvidia-smi` shows an RTX 4090; `tmux new -s mm`; install Claude Code CLI.
2. Fresh fine-grained PAT (`contents:write` on ferox-isaac-demo, Ferox, ferox-g1-locomotion,
   panthera-g1-wbc; `contents:read` on panthera-g1-driver, realsense_driver), 24 h expiry.
3. First message: `Fresh 4090 instance, empty ~/panthera. Clone per RESUME.md (branch
   mohammed/twin-campaign, tag twin-g1-fixed-2), read docs/twin/CAMPAIGN.md status + RESUME.md end to end,
   then read this file (docs/mm/CAMPAIGN.md — save it there and commit). Execute MM0. Stop after
   RESULTS_MM0.md. Token: <PAT>`
4. After MM0: `Continue MM1 → MM5 overnight per §5.`

---

## Carry-over triage — which of MM0.2 needs the 4090, measured on 2026-08-19

Recorded here because the next instance should not have to re-derive it, and because
"blocked" and "not yet done" are different states that deserve different words.

| MM0 sub-item | Needs a 4090? | Why |
|---|---|---|
| §1 RESUME verification pass | **Yes** — by definition | The step *is* the `nvidia-smi` assertion |
| §2 hand-roll numeric check at URDF zero pose | **No** | Pure URDF forward kinematics. `tools/hand_frames.py` + `tools/check_hand_orientation.py` are stdlib-only, no ROS, no Isaac, no GPU. The DT campaign already measured the wrist-frame form: fingers **0.707°** off wrist +X, palm normal 0.707° toward the midline, thumb 0.518° off wrist +Z, chirality PASS. MM0 asks for the **world-frame dot-product form at URDF zero pose** with a ≥0.9 threshold, which is a re-expression of the same geometry and is a few minutes' work anywhere |
| §2 renders top/front/side | **No** | The offscreen render path works on the 4080 — `14_capture_views.sh` and `13_capture_hands.sh` both completed here, and two `render_orbit.py` passes wrote 90 + 60 frames |
| §2 aligned-depth check + C-21 clip | **Yes** | Camera. C-23 |
| §2 `/scan` clip re-shot in hospital | **No** | Lidar only. Measured 45.0–45.8 % finite here against the bag's 70 % — see `RESULTS_DT2.md` for why 45 % is the correct answer for a mid-corridor spawn |
| §2 nav 3/3 inside measured free-space bounds | **No** | Lidar + nav only. Bounds already measured and recorded in `RESUME.md` §0 next-steps: free space x [3.71, 12.16] m, y [−2.67, 8.18] m; robust interior x [5.16, 10.76], y [−1.42, 5.08]. Estimated ≤1 h |
| §3 `tools/film.py` v1 + ghosting test + 20 s walk clip | **No** | Offscreen rendering plus the omni policy, both of which work here. The ghosting defect it must catch is real and is recorded against the DT montage |

So on a 4080 the achievable MM0 verdict is **PASS-with-deviations**: the five
GPU-independent sub-items close normally, and the aligned-depth check plus the C-21 clip
stay open and queued for a 4090 day. That is the decision recorded in the status header,
and it is why the `nvidia-smi` step is a flag rather than a gate.
