# RESULTS_MM3 — low-level DDS bridge

Host: RTX 4080 SUPER 16 GB / driver 580.105.08 (non-4090 box, C-23 applies)
Date: 2026-08-19 → 2026-08-20
Verdict: **PASS-with-deviations**

Five of six tests pass as written. Test (a) passes on a declared test rig and fails
free-standing, for a reason that is not the bridge and is proven not to be — see (a).
Two of the gate's own numbers were wrong against the robot and are corrected here with
measurements: the 500 Hz publication rate, and the 100 ms treated as a threshold
rather than a deadline.

---

## Scorecard

| # | requirement | status | evidence |
|---|---|---|---|
| a | `unitree_sdk2py` PD stand script stands the twin 60 s | **PASS on rig / FAIL free-standing** | `evidence/MM3/stand_on_rig.txt`, `stand_freestanding.txt`, `stand_implicit_pd_control.txt` |
| b | cmd-loss → damping within 100 ms | **PASS** (6/6, 85.3–98.6 ms) | `evidence/MM3/failclosed_latency.txt` |
| c | `rt/lowstate` rate + field parity vs the DT bag | **PASS** (1040.75 Hz, 16/16 fields) | `evidence/MM3/lowstate_parity_twin.txt`, `lowstate_{rate,decode}_bag.txt` |
| d | ROS topics keep publishing under `G1_CONTROL=lowcmd` | **PASS** (no topic degraded; two improved) | `evidence/MM3/ros_topics_under_lowcmd.txt` vs `ros_topics_policy_baseline.txt` |
| e | passive Dex5 coupling done or declared | **DECLARED** | this document, C-13 |
| f | `rt/dex3/{left,right}/{cmd,state}` 20-entry wire | **PASS** (20/12/12 shape, 208.3 Hz, fingers driven) | `evidence/MM3/dex3_wire.txt` |
| — | clip per gate | **DEFERRED** (C-23) | see *Clips* |

---

## The two numbers the gate got wrong

**500 Hz was two rates wearing one hat.** `§4.2` asked for `rt/lowstate` at 500 Hz ±2 %.
Measured off the DT bag, the robot publishes at **1041.677 Hz** — per-second counts
1039–1044, σ = 0.79 over 34 full seconds, a hardware clock. Nothing in the system
publishes at 500 Hz. The contract now carries the two rates separately:
`lowstate_publish_hz = 1041.68` and `pd_apply_hz = 500`, each with its own provenance.
Full argument in `evidence/MM3/PREREQS.md`.

**851 vs 1041 is settled, and not the way I first guessed.** Decoding the bag's `tick`
field: tick advances at **1000.064 /s**, i.e. it is a millisecond counter 0.006 % off
nominal. The robot publishes 1041.68 messages against that 1000 Hz tick, so ~4 % of its
messages repeat a tick while carrying *fresh* IMU — matching the driver probe's
independent 3.6 %. The bag loses 10 ticks out of 34 559 (0.03 %); the driver's 851.4 Hz
capture is the lossy one, its tick clock running 7.7 % slow. My written hypothesis —
that 851 was downstream loss from a 1041.7 Hz source, on the strength of
`/secondary_imu` also reading 1041.7 — was wrong in mechanism, and that coincidence
carries no weight.

**100 ms is a deadline, not a threshold.** A watchdog whose threshold *is* 100 ms can
only fire after 100 ms. First measurement: 106.47 ms. See (b).

---

## (a) The twin cannot stand under joint-space PD, and that is not the bridge

Free-standing, the twin topples in ~0.7 s while its joints track perfectly:

```
base_z 0.7975 -> 0.775 -> 0.693 -> 0.325 -> 0.094   over t=0..1.0 s
pitch  +0.050 -> +0.440 -> +1.258 -> +1.480
knee_L  0.344   0.322     0.309     0.311            (target 0.300 throughout)
```

The joints hold. The robot falls over like a statue. Over a 60 s commanded stand:
`lowcmd 499.98 Hz`, `lowstate 1040.92 Hz`, joint tracking **0.089 rad mean** — the wire
and the PD are healthy; attitude is not (`pitch_rms 1.485`).

**Controlled against Isaac's own implicit drive.** Handed the locomotion policy's own
deploy.yaml gains, with this bridge's torques disabled entirely
(`G1_LL_PD=implicit`), the twin topples *identically* — base_z 0.095, pitch 1.471, by
t=2–4 s. So the explicit 500 Hz PD is not too soft; a joint-space PD does not balance a
humanoid, whichever code evaluates it. The zero posture the sdk2py example uses fails
too, backward instead of forward.

This is fidelity, not failure: the real G1 does not stand from bare low-level PD either
— its stance comes from the built-in motion mode, which is precisely why
`g1_low_level_example` opens by calling `MotionSwitcherClient.ReleaseMode()` to take
control away from it. **Test (a) as written is not satisfiable by the robot either.**
Same class of finding as the 500 Hz.

On the declared rig (C-30, base pinned to spawn pose — how the example is run on
hardware, with the robot hanging), the twin holds the commanded stand for the full
60 s: **track_err 0.042 rad mean, 0.391 rad max**, `lowcmd 499.98 Hz`,
`lowstate 1041.08 Hz`, `mode_machine=5` learned from the twin by the client.

**Standing under `lowcmd` is a dependency on MM4.** CAMPAIGN §4.4 already assumes this
— MM5 hands off "to `G1_CONTROL=lowcmd` with SONIC standing" — so nothing downstream
needs re-planning, but MM4 is now load-bearing rather than incremental.

## (b) Fail-closed, measured

Six consecutive trials, each stopping a 500 Hz command stream cold:

```
85.32  98.60  96.36  89.48  86.31  94.04 ms      deadline <= 100 ms      6/6 PASS
```

Getting there required two corrections. The threshold started at 100 ms and engaged at
**106.47 ms** — a threshold equal to the deadline can only miss it. Moving to 95 ms
still missed one in three (96.07 / **102.71** / 95.17 / 95.99 / 96.72) because the
watchdog was evaluated on the PD decimation, giving it a wall-clock resolution of
`pd_hz × RTF` ≈ 3.9 ms, and Isaac's main loop stalls for a render frame on top. The
watchdog now runs every physics step and the threshold carries an explicit 15 ms jitter
budget. Damping early is safe; damping late is not.

Under damping the pose drifts off the commanded target (0.16 rad mean, 0.45 rad max)
and joint velocity goes to zero — it does *not* hold the last `q_d`, which is the
failure the timeout exists to prevent.

## (c) Rate and field parity

`rt/lowstate` measured live off the twin: **1040.752 Hz**, inside the bag's ±2 % band
(1020.84–1062.51). All sixteen field-semantics checks pass, gated on what the robot
*actually* populates across all 35 998 bag messages rather than on what the IDL allows:

`motor_state` length 35 with slots 0–28 at `mode=1` and 29–34 identically zero ·
`ddq` never populated · `version=[0,0]` · `mode_pr=0` · `mode_machine=5` ·
`wireless_remote` and `reserve` all zero · `tick` a millisecond counter and not a
message counter · quaternion normalised w-first · `rpy` populated · accel magnitude
9.8067 · CRC self-consistent 200/200.

## (d) ROS topics under lowcmd — measured in SIM time

`ros2 topic hz` is the wrong instrument for this twin: it counts wall arrivals, and DT2
gated these sensors at "10.00 Hz EXACTLY **in sim time**". The first pass read 15.4 Hz
on a 51.4 Hz topic with *every* topic low by the same factor — a real-time-factor
artefact, not a broken publisher. Re-measured from message stamps:

| topic | contract | `lowcmd` (1000 Hz phys) | policy baseline (200 Hz phys) |
|---|---|---|---|
| `odom` | 51.4 | 50.000 (−2.7 %) PASS | 50.000 (−2.7 %) PASS |
| `imu/data` | 93.7 | **90.909 (−3.0 %) PASS** | 66.667 (−28.9 %) FAIL |
| `/livox/lidar` | 10.0 | 10.000 PASS | 10.000 PASS |
| `/livox/imu` | 200.0 | **200.000 (exact) PASS** | 100.000 (−50 %) FAIL |
| `/tf` | 100.0 | 50.000 (−50 %) FAIL | 50.000 (−50 %) FAIL |
| `/scan` | 10.0 | absent | absent |

**`lowcmd` degrades nothing and strictly improves two.** `/tf` and `/scan` fail
identically in the baseline, so neither is caused by MM3: `/tf` is render-clock-driven
(C-31, pre-existing) and `/scan` is produced by the Ferox nav stack, not the sim, which
was not running for this measurement.

This only works because physics is at 1000 Hz. The twin's publishers decimate an
**integer** number of steps off the physics clock, so `physics_dt` decides which rates
are reachable at all — the same arithmetic as C-15/C-16 on the Go2. At the 500 Hz
`§4.2` specifies, three interfaces alias off contract (`/livox/imu` 500/3 = 166.67,
`imu/data` 500/6 = 83.33, `/tf` 500/10 = 50.00 — `ros_topics_physics500_aliased.txt`).
1000 Hz is a multiple of both 200 and 100; 500 is not. PD still runs at the specified
500 Hz, applied every second physics step.

## (e) Passive Dex5 coupling — declared

The four passive indices (4, 8, 12, 16 — the roll/abduction joints of fingers 2–5) are
accepted on the wire and **not** applied, so the twin cannot do something the hand
cannot. Commanded to 0.35 rad along with everything else, they move only by mechanical
coupling with their driven neighbours: left 0.019 / 0.003 / 0.038 / 0.009 rad against
driven joints moving up to 0.275.

One asymmetry to flag: right index 12 (`Roll_41R`) moved **0.205 rad** against the
left's `Link_41L` at 0.038. The Dex5 is not name-symmetric at that index, and the right
hand's URDF effort metadata is 0 — consistent with the documented L/R asymmetry, but
not explained. Open question 3.

## (f) The 20-entry hand wire

```
SHAPE  motor_state=20  press_sensor_state=12  pressure_per_zone=12
RATE   left 208.35 Hz   right 208.34 Hz        (gate >= 200 Hz)
DRIVE  left  active mean 0.054 rad, max 0.275   right  active mean 0.133, max 0.860
```

20 entries, not the 7 `unitree_sdk2py.idl.default` pre-fills — those factories are
Dex3-shaped and are deliberately unused; both the DDS IDL (`types.sequence`) and the
ROS msg (`MotorCmd[]`) are unbounded, so 20 is legal on the wire.
`press_sensor_state` carries the right shape and **zeros**: the 12 contact zones are
not in the asset (DT3 left them to DT6, deferred), so the values are declared absent
rather than invented. C-27.

---

## What changed

| file | one line |
|---|---|
| `docker/Dockerfile.lowlevel` | `ferox/twin-lowlevel:humble` — ROS `unitree_hg` + `unitree_sdk2py` + cyclonedds 0.10.2 + rosbag2 |
| `isaac/twin/lowlevel_bridge/shm.py` | seqlock shared-memory transport, fixed numpy layout |
| `isaac/twin/lowlevel_bridge/sim_side.py` | Isaac side: state sampling, 500 Hz PD, watchdog, hand wire |
| `isaac/twin/lowlevel_bridge/dds_side.py` | DDS side: `rt/lowstate` at 1041.68 Hz, `rt/lowcmd`, dex3 |
| `isaac/twin/lowlevel_bridge/fake_sim.py` | GPU-free stand-in to gate the pacing loop on its own |
| `isaac/run.py` | `G1_CONTROL=policy\|lowcmd`; 1000 Hz physics under lowcmd |
| `isaac/twin/g1_contract.yaml` | new `lowlevel:` block — both rates, field semantics, dex3 wire |
| `scripts/mm3_*.py`, `mm3_ros_alive.sh` | the six gates, each as a runnable check |

Commits `e5a689b`, `8f58e2c` on `mohammed/mm-campaign`.

## Bugs worth keeping

All four are the same family — **state that looks live and is not**:

1. **Split shm ownership.** Each side created one segment and attached to the other's.
   Whoever attaches before the other re-creates keeps a mapping to an *unlinked*
   segment and reads frozen data forever, silently. The sim attached to a dead cmd
   segment still holding `cmd_count=31500` from the previous run, concluded it had been
   commanded, and sat in fail-closed damping while the robot folded — which cost a full
   round of failed stand tests. The sim now owns both.
2. **A torn seqlock read read as command loss.** Reader and writer both at 500 Hz
   collide often; treating a collision as silence made fail-closed flap on and off every
   50–300 ms *through a stand that was being commanded the whole time*. Visible as
   `last command age -1.00 ms`. 5628 torn reads in one 45 s run.
3. **Two `apply_action` calls in one physics step.** `ArticulationController` keeps one
   pending action, so the body's effort action silently replaced the hand's position
   action: 8927 applications counted while `Yaw_11L` sat pinned at 0.6802 — its URDF
   upper limit, i.e. the USD drive's own target — never moving toward the commanded 0.35.
4. **`set_joint_efforts` reports success and does nothing.** The idle hold saturated a
   139 Nm knee against a 1.3 rad error and the robot still folded, which is not what a
   saturated actuator looks like. `ArticulationAction` is the working path.

Two build bugs of the same kind: `rosidl_generator_dds_idl` is absent from
`ros-humble-ros-base`, and overlays sourced from `/etc/bash.bashrc` are invisible to
non-interactive shells — so `unitree_hg` imported in an interactive shell and vanished
under `docker run img python3 -c`, while pip-installed `unitree_sdk2py` worked either
way. Also `unitree_sdk2py`'s `setup.py` declares no `package_data`, so pip silently
drops `crc_*.so` and the failure surfaces at the first real message; the image now
constructs a CRC at build time so a regression fails the build.

## Deviations

| id | one line |
|---|---|
| C-25 | robot repeats ~4 % of `rt/lowstate` messages against its 1000 Hz tick; twin does **not** emulate it |
| C-26 | motor `temperature` and `vol` published as constants (bag ranges 31–48 °C, 45.5–49.5 V) |
| C-27 | `press_sensor_state` right shape, zero values — contact zones not in the asset (DT6 deferred) |
| C-28 | *(withdrawn — 1000 Hz physics makes tick advance +1/ms as the robot does)* |
| C-29 | RTF ≈ 0.44–0.52 at 1000 Hz physics; `rt/lowstate` is wall-paced so it repeats state between physics steps (57 % measured) |
| C-30 | test rig: base pinned to spawn pose for (a) and (f); never on by default, every result taken under it says so |
| C-31 | `/tf` at 50 Hz against a 100 Hz contract — render-clock-driven, **pre-existing**, present identically in the policy baseline |

## Clips

**None. Deferred to the 4090 day, per C-23.** Both media routes are blocked on this
box and neither is a bug left to fix: the live-sim camera SIGSEGVs on creation
(reconfirmed this gate — the twin crashes at `world.step(render=True)` in the camera
render-product warmup whenever `TWIN_CAMERA=1`), and `film.py`'s standalone scene never
applies the policy's gains. Every MM3 gate is therefore evidenced by numbers and logs
rather than video, and the shot list carries the MM3 items forward.

## Open questions for Mohammed

1. **Is `lowstate_publish_hz` a wall rate or a sim rate?** It is wall-paced today, which
   is right for the consumer (an sdk2 client, and SONIC in MM4, both run in wall time),
   but at RTF 0.5 that means 57 % of messages repeat state. Sim-pacing would give 0 %
   repeats and a wall rate of ~520 Hz, failing gate (c) as written. **Default taken:
   wall pacing.**
2. **Does test (a) stand as a gate?** It is not satisfiable by the real robot either.
   Proposal: restate as "the sdk2 client drives the twin's joints at 500 Hz with
   tracking error < X on a declared rig", and move "stands for 60 s" into MM4 where a
   balance controller exists. **Default taken: reported both ways, rig and free.**
3. **`Roll_41R` moves 0.205 rad passively against `Link_41L`'s 0.038.** Real asymmetry
   or an asset defect? The two are different joints with different names at the same
   index, and the right hand's URDF effort metadata is 0.
4. **The DT bag's own joint angles look wrong for "standing still".** `left_hip_yaw`
   sits at 2.642 rad (151°) and `waist_yaw` at −0.897 rad (−51°) while the IMU says the
   robot is upright (quaternion 0.9999). Either the capture is not of a standing robot,
   or the SDK↔URDF index mapping for this firmware is not the canonical one — and the
   twin's mapping is derived from the canonical order. Does not affect MM3 (gated on
   wire semantics, not pose) but it would affect any joint-value diff against this bag.

## Reproduce

```bash
# the image (one-time)
docker build -t ferox/twin-lowlevel:humble -f docker/Dockerfile.lowlevel <ctx>

# the bag measurements, no GPU needed
python3 scripts/mm3_lowstate_rate.py ref/captures/g1/captures/g1_twin_gt/g1_twin_gt_0.db3 /lowstate
docker run --rm -v <bag>:/bag:ro -v $PWD/scripts:/scripts:ro ferox/twin-lowlevel:humble \
  python3 /scripts/mm3_lowstate_decode.py /bag

# the twin, then the bridge
ROBOT=g1 TWIN=1 TWIN_CAMERA=0 HAND=dex5_1p SIM_WORLD=hospital \
  G1_CONTROL=lowcmd [G1_LL_FIX_BASE=1] bash scripts/01_start_sim.sh
docker run -d --name mm3_bridge --network host --ipc host --user 1234:1234 \
  -v $PWD/isaac/twin/lowlevel_bridge:/bridge:ro ferox/twin-lowlevel:humble \
  python3 /bridge/dds_side.py --domain 0 --publish-hz 1041.68

# the gates (all inside ferox/twin-lowlevel:humble, --network host --ipc host --user 1234:1234)
python3 /scripts/mm3_sdk2_stand.py --domain 0 --ramp 3 --hold 60 --silence 3   # (a) (b)
python3 /scripts/mm3_parity.py     --domain 0 --seconds 20                      # (c)
python3 /scripts/mm3_dex3_wire.py  --domain 0 --settle 4 --drive 10             # (f)
# (d) runs in ferox/nav:humble on ROS_DOMAIN_ID=42:
python3 /scripts/mm3_ros_alive.py 16
```

`--ipc host` is required on every container: the bridge halves share memory, and the
100 ms watchdog compares `CLOCK_MONOTONIC` across them.
