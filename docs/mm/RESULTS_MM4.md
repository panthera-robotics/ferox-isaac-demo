# RESULTS_MM4 — SONIC in the twin

Host: RTX 4080 SUPER 16 GB (sm_89) / driver 580.105.08 / CUDA 13.0
Date: 2026-08-20
Verdict: **PARTIAL** — image, interop and the ZMQ command path are done and proven.
SONIC drives the twin. What is not done is SONIC *balancing* it free-standing.

The two things MM4 is really for both landed: an x86_64 SONIC deploy stack that runs,
and SONIC closing the loop on the twin over DDS. What is not done is driving it
through the stand → walk → turn → POSE sequence: SONIC's ZMQ planner handshake never
completes, so it holds a static stance instead of walking.

---

## Scorecard

| requirement | status | evidence |
|---|---|---|
| x86 image for `gear_sonic_deploy` (from the `docker/g1wb-*` recipes) | **PASS** | `docker/sonic-deploy/Dockerfile.x86`, `ferox/sonic-deploy:v1.1-x86_64` 4.8 GB |
| every aarch64→x86 delta documented | **PASS** | 7 deltas, marked `X86 DELTA` in the file, listed below |
| `deploy.sh … sim` pointed at the twin's DDS domain | **PASS** | SONIC reads `rt/lowstate`, writes `rt/lowcmd` at ~499 Hz |
| lowstate → policy → lowcmd closed in Isaac | **PASS** | `evidence/MM4/sonic_interop_rig.txt` |
| Kevin's fail-closed semantics on | **PARTIAL** | the *twin's* fail-closed engaged correctly under SONIC; Kevin's branch not merged in |
| SONIC ZMQ handshake (`Planner initialized successfully!`) | **PASS** | `evidence/MM4/sonic_handshake_closed.txt` |
| scripted sequence commands delivered end to end (14 segments) | **PASS** | `evidence/MM4/sonic_handshake_closed.txt` |
| policy → `lowcmd` hand-off, one-way, never co-driving | **PASS** | `G1_CONTROL=handoff`, `evidence/MM4/handoff.txt` |
| policy latency measured | **PASS** — policy 2.62–3.00 ms, planner model 2.93 ms, total 2.99 ms; LowState age 3.5–5.2 ms, IMU age 3.5–4.5 ms | `evidence/MM4/sonic_loop_timing.txt` |
| SONIC balances the twin free-standing (stand → walk → turn → POSE) | **FAIL** | below |
| SONIC and omni never co-drive (asserted) | **PASS** | `G1_CONTROL` branches in `on_physics_step`; the policy is not stepped at all under `lowcmd` |
| metrics + clip per step | **DEFERRED** (C-23) | no media on this box; see MM3 |

---

## What works, and it is the substantive half

**SONIC recognises the twin as a G1.** On first contact it prints:

```
G1 type: 5
Init Done
```

`5` is `mode_machine`, read off `rt/lowstate` — the value MM3 measured in the DT bag
and encoded in the contract. The field-parity work is what makes SONIC accept the twin
at all.

**The loop closes.** With the twin in `G1_CONTROL=lowcmd` and the MM3 bridge running:

```
bridge : lowstate=46876 (1041.680 Hz)  lowcmd=22469 (499.31 Hz)  crc_bad=0
sim    : mode=cmd  knee_L=+0.670  hip_p_L=-0.312   (was 0.300 / -0.100 under idle hold)
```

SONIC consumes state at 1041.68 Hz, commands at 499 Hz, and the twin's joints move to
SONIC's targets. It also drives `rt/dex3/{left,right}/cmd` at ~500 Hz.

**`--disable-crc-check` is not needed, and that is a result.** The W3 compose file
passes that flag for sim with the comment *"the MuJoCo bridge does not produce the CRC
the real robot does"*. The MM3 bridge does: `crc_bad=0` across 22 469 commands with
CRC checking left **on**. The twin is closer to the robot than the reference sim on
exactly the axis MM3 gated.

**The MM3 watchdog fired correctly under a real controller.** When SONIC stopped
commanding, the twin went `mode=FAILCLOSED` and stayed there — not a synthetic test,
an actual controller dropping out mid-run.

## The aarch64 → x86 deltas

The W3 recipe is aarch64-native and its `build.sh` refuses to run anywhere else
(`if [ "$(uname -m)" != "aarch64" ]; then FATAL`). Seven changes, each marked in the
Dockerfile:

1. **TensorRT version kept at 10.15.1.29**, not the 10.13 in the campaign text — see
   open question 1.
2. **`just`** — `x86_64-unknown-linux-musl` tarball.
3. **ONNX Runtime** — `onnxruntime-linux-x64`, which is what upstream's own
   `install_deps.sh:388-390` selects for non-ARM hosts.
4. **TensorRT libs** — same public `pypi.nvidia.com` wheel name; pip resolves
   `manylinux_2_28_x86_64` instead of `manylinux_2_35_aarch64`. It is **3.71 GB**
   against aarch64's 2.5 GB.
5. **`IS_THOR` deliberately NOT set.** W3 must set it because
   `CMakeLists.txt:54-98` links `-lcudla` on *any aarch64* host unless it believes it
   is a Jetson Thor, and a GB10 has no DLA. On x86_64 that branch is never taken, so
   the flag is unnecessary and setting it would assert something false about the
   hardware. The "no cudla" link assertion is kept anyway.
6. **Arch assertion inverted** — the build fails unless `file` reports `x86-64`.
7. **Vendored DDS libs** from `thirdparty/unitree_sdk2/thirdparty/lib/x86_64`.

Everything else is carried over unchanged, including the TensorRT header/lib skew
guard and the rule that `.trt` engines are never baked in — they are rebuilt per
device and cached in named volumes (`trt_policy`, `trt_planner`).

## The handshake is closed, and the answer was in the repo

`panthera-g1-wbc tools/scripted_walk.py` is the driver that closed this loop on the
Spark for W1/W2, and it **imports** NVIDIA's own builders from
`gear_sonic.utils.teleop.zmq.zmq_planner_sender` rather than reimplementing them. This
gate now does the same. Reverse-engineering from the C++ headers had got two thirds of
the way with three errors, every one silent rather than loud:

* **`WALK` is 2.** `IDLE/SLOW_WALK/WALK/RUN = 0/1/2/3`, so the "1" that looks like a
  walk command is a SLOW_WALK.
* **`speed` and `height` use −1.0 to mean "mode default".** Sending 0.0 is a literal
  zero speed — a robot correctly obeying an order to go nowhere.
* **the start command must be re-sent on every beat.** ZMQ PUB drops everything
  published before a SUB finishes connecting, and the deploy spends a long time in
  TensorRT init; a single start lands in that hole and is lost.

Two earlier format errors were also real and are worth keeping: `dtype "b8"` is not in
SONIC's vocabulary (it compares against exactly `bool/u8/i8/i16/i32/i64/f16/f32/f64`),
and `movement`/`facing` are 3-vectors read by a fixed `i<3` memcpy, so a 2-element field
is read one float past its end — not a parse error, just a garbage third component.

**But none of those was the blocker.** The blocker was a topic the twin never published:

```
✗ Error: LowState or IMUState is not available in the middle of the control loop!
```

SONIC subscribes to **`rt/secondary_imu`** — the G1's torso IMU, `robot_parameters.hpp:28`
`HG_IMU_TORSO` — and refuses to enter its control loop without it. It is *not* part of
`LowState_`, so a twin publishing a perfect `rt/lowstate` still looks like a robot with
no torso IMU. The bridge now publishes it at 1041.68 Hz, composed from the pelvis IMU
through the waist chain (C-36).

The irony is on the record: MM3 measured `/secondary_imu` at 1041.7 Hz in the driver
evidence and I dismissed it as a coincidence with `/lowstate`'s rate. It was not a
coincidence about rates — it was the name of a topic the twin actually needed.

With it present: `G1 type: 5` → `Planner enabled` → `Planner Init timing - Model:
5711us` → **`Planner initialized successfully!`**, and the full 14-segment sequence is
delivered and consumed.

## What the reference publisher fixed, and the third mismatch

`panthera-g1-wbc tools/scripted_walk.py` was in the repo the whole time. It drives
exactly this and, rather than reimplementing the format, **imports NVIDIA's own
builders** from `gear_sonic.utils.teleop.zmq.zmq_planner_sender`. This gate now does the
same — the wire format stays owned by upstream. Reading it corrected three things my
hand-rolled version had wrong, every one of them silent:

* **`WALK` is 2.** `LocomotionMode` is IDLE/SLOW_WALK/WALK/RUN = 0/1/2/3, so the `1` I
  was sending was a SLOW_WALK.
* **`speed` and `height` use −1.0 to mean "mode default".** I was sending 0.0, which is
  a literal zero speed — a robot correctly obeying an order to go nowhere.
* **The start command must be re-sent on every beat.** ZMQ PUB drops everything
  published before a SUB finishes connecting, and the deploy spends a long time in
  TensorRT init; a single start lands in that hole and is lost.

**The third mismatch was not in the message format at all: it was a missing topic.**
SONIC subscribes to **`rt/secondary_imu`** — the G1's torso IMU, `robot_parameters.hpp:28`
— and refuses to enter its control loop without it:

```
✗ Error: LowState or IMUState is not available in the middle of the control loop!
[ZMQCommandManager ERROR] Planner initialization timeout
```

It is **not part of `LowState_`**, so a twin publishing a field-perfect `rt/lowstate` at
exactly the right rate still looks like a robot with no torso IMU. The bridge now
publishes it at 1041.68 Hz, composed from the pelvis IMU through the waist chain (C-36).
With it present the handshake closes on the first try:

```
G1 type: 5
Planner enabled
Planner initialized
Planner initialized successfully!
```

There is a small irony worth recording: `/secondary_imu` appeared in the MM3 driver
evidence at 1041.7 Hz, and MM3 dismissed it as a coincidence with `/lowstate`'s rate.
It was not a coincidence — it was naming a topic the twin actually needed.

## What does not work: SONIC does not yet balance the twin

The sequence is commanded end to end and SONIC replans through it, but the robot is on
the floor for most of it. The failure is at the **controller hand-over**, and it is a
stance mismatch rather than a plumbing fault.

Measured, from the command segment while SONIC was running:

| | knee | hip_pitch | ankle_pitch |
|---|---|---|---|
| omni policy holds | +0.124 | −0.052 | ~−0.20 |
| SONIC commands (`q_d`) | **+0.669** | **−0.312** | **−0.363** |

SONIC's nominal stance is a **crouch**; the omni policy's is nearly straight. At the
instant of hand-over the twin has to cross 0.55 rad on the knee, and SONIC's gains
(kp 99 hips/knees, 28.5 ankles, kd 6.3/1.8 — read off the wire, not assumed) act on that
gap immediately.

Three attempts, and what each one taught:

1. **Instant hand-over** → joint velocity spikes and SONIC trips its own safety check
   (`body_dq[20] = 36.95 > 35`), which is a *correct* reaction to a real transient.
2. **Bumpless transfer**, ramping the target from the policy's pose to SONIC's over
   0.3–6 s → the safety trip goes away and SONIC survives the whole sequence, but the
   robot still falls: blending **breaks SONIC's closed loop**, because it commands one
   thing while the bridge applies another. A balance controller cannot be fed a
   filtered version of its own output.
3. **The rig** (base pinned) → worse, and instructively so. A pinned base *lies* to a
   balance controller: SONIC's observations read "upright, motionless" whatever it
   commanded, so it drove the knee to 1.011 rad and the twin was dropped out of a
   crouch when the rig let go. Pinning at the contract standing height instead jams the
   feet into the floor and deforms the stance (hip to −0.475).

**The likely answer, untested, and it is why this is a ping rather than another
attempt:** on the Spark, W3 ran SONIC against a **MuJoCo** sim that spawns the robot in
SONIC's own stance, so there was never a hand-over to get wrong. The twin spawns in the
locomotion policy's stance instead. Spawning the twin crouched — in SONIC's nominal
pose — would remove the transition entirely rather than trying to survive it.

### What the hand-over machinery does do

`G1_CONTROL=handoff` is built, works, and is the pattern CAMPAIGN §4.4 already specifies
for MM5. The omni policy holds the twin standing (base_z 0.790, pitch 0.041, **RTF
1.13** — real time, because the bridge is passive) while SONIC boots and initialises,
then the articulation transfers **once**, latched, one-way:

```
[lowlevel-sim] PASSIVE: publishing state only; the policy still drives
[PANTHERA-MARK] rt/lowcmd is live; handing over in 8s
[lowlevel-sim] ACTIVE: articulation taken from the policy at t=17.888s
[PANTHERA-MARK] HANDOVER complete: the policy will not be stepped again
```

SONIC and the omni policy never co-drive — the switch is a latch, not a blend of
authority, which is the assertion MM4 asks for.

### Three bridge bugs found by integrating a real controller

None of these would have surfaced against a test client:

1. **`_cmd_fresh()` latched.** It cached the first record it read and never re-read, so
   its `stamp_ns` aged past the timeout and it returned False forever. The active path
   happened to refresh the cache each cycle and hid it; in PASSIVE mode nothing did, so
   the hand-over never fired while SONIC commanded at 500 Hz the whole time.
2. **A single torn read cancelled the hand-over.** The countdown reset on any miss, and
   with a 1 kHz reader against a 500 Hz writer isolated torn reads are normal, so it
   restarted forever. Now only a *sustained* loss cancels.
3. **`on_physics_step` ran twice on the activation step**, double-counting `sim_time` —
   and `rt/lowstate.tick` is derived from `sim_time`.

## Metrics

From SONIC's own loop instrumentation, on this box:

```
LowState age 3.5-3.8 ms   IMU age 3.5-3.8 ms
Obs 247 us    Policy 2998 us    Obs->MotorCommand 3245 us    Post 26-35 us
Planner: Gather 24 us, Model 2947 us, Convert50Hz 11 us, Total 2985 us
```

Policy inference is ~3.0 ms against a 20 ms control period, so SONIC has ample headroom
even on a 4080 shared with the sim. Bridge side: `rt/lowstate` 1041.68 Hz,
`rt/secondary_imu` 1041.68 Hz, `rt/dex3/*/state` 208.33 Hz, `rt/lowcmd` accepted at up
to 499 Hz with `crc_bad=0` throughout.

## What changed

| file | one line |
|---|---|
| `docker/sonic-deploy/Dockerfile.x86` | the x86_64 port, 7 documented deltas |
| `scripts/mm4_sonic_drive.py` | ZMQ sequence driver (packed 1280-byte header format) |
| `isaac/twin/lowlevel_bridge/sim_side.py` | `G1_LL_FIX_BASE=until_commanded` — rig auto-release |

## Deviations

| id | one line |
|---|---|
| C-30 (extended) | the test rig gains an `until_commanded` mode: hold the base until a controller has had authority for N s, then release — mirrors bringing a real G1 up on a hoist and lowering it once the controller is live |
| ~~C-32~~ | withdrawn — 10.15.1.29 IS the campaign pin (Mohammed), W3 provenance |
| C-33 | Kevin's `kevin/g1-fail-closed-safety` branch is fetched but NOT merged into this image; the fail-closed proven here is the twin's own (MM3), not Kevin's |
| C-36 | `rt/secondary_imu` is COMPOSED from the pelvis IMU through the waist chain, not an independent torso IMU as on the real G1; its gyro omits the waist joints' own rates |
| C-37 | SONIC's nominal stance (knee 0.669) and the omni policy's (knee 0.124) differ by 0.55 rad; the twin does not survive the transition between them |
| C-36 | `rt/secondary_imu` is COMPOSED from the pelvis IMU through the waist chain, not an independent torso sensor; its gyro omits the waist joints' own rates |
| C-37 | `TWIN_ARMATURE=0.01` is now required for `lowcmd`/`handoff`. The twin's USD ships no rotor inertia, and an explicit PD on bare link inertia chatters |

## Open questions for Mohammed

1. ~~TensorRT 10.13 or 10.15.1.29?~~ **DECIDED (Mohammed): 10.15.1.29 is the campaign
   pin.** Provenance is W3's recipe, which records 10.13 as the version that came out
   internally inconsistent (headers 10.13.0.35 against libs 10.13.3.9). The skew guard
   stays. C-32 is closed as a decision rather than a deviation.
2. ~~The planner handshake.~~ **ANSWERED: it was in the repo.**
   `panthera-g1-wbc tools/scripted_walk.py` drives exactly this, and it imports
   NVIDIA's own builders from `gear_sonic.utils.teleop.zmq.zmq_planner_sender` rather
   than reimplementing them. This gate now does the same. See "What the reference
   publisher fixed".
4. **The remaining balance failure** — see "What I would try next". This is the one
   item standing between here and MM5 with SONIC balancing. MM5's own spec allows the
   fallback ("or omni standing if MM4 slips — say which"), so MM5 is not blocked, only
   its SONIC variant is.
3. **Should SONIC own the bring-up rig?** SONIC needs ~15 s to build/load engines, and
   MM3 established the twin cannot stand unaided for those 15 s — so without the rig
   SONIC always inherits a robot already face-down. `until_commanded` handles it, but
   the release threshold (authority held for N s) is a number I chose, not measured.

## Reproduce

```bash
# context: upstream pinned + LFS x86_64 libs + HF artifacts
git clone https://github.com/NVlabs/GR00T-WholeBodyControl && \
  git -C GR00T-WholeBodyControl checkout 54d0b102bb8876a54c9d41796bd9f221c9e042d9 && \
  git -C GR00T-WholeBodyControl lfs pull -I 'gear_sonic_deploy/thirdparty/**' \
                                -I 'gear_sonic_deploy/reference/**'
# artifacts from the PUBLIC nvidia/GEAR-SONIC (no token):
#   sonic_v1_1/model_{encoder,decoder}.onnx, sonic_v1_1/observation_config.yaml, planner_sonic.onnx
docker build -t ferox/sonic-deploy:v1.1-x86_64 -f docker/sonic-deploy/Dockerfile.x86 <ctx>

# twin + bridge + SONIC, all on the host network, DDS on lo
ROBOT=g1 TWIN=1 TWIN_CAMERA=0 HAND=dex5_1p SIM_WORLD=hospital G1_CONTROL=lowcmd \
  G1_LL_FIX_BASE=until_commanded G1_LL_RIG_RELEASE_S=30 bash scripts/01_start_sim.sh
docker run -d --name mm3_bridge --network host --ipc host --user 1234:1234 \
  -v $PWD/isaac/twin/lowlevel_bridge:/bridge:ro ferox/twin-lowlevel:humble \
  python3 /bridge/dds_side.py --domain 0 --iface lo --publish-hz 1041.68
docker run -d --name mm4_sonic --network host --gpus all \
  -v trt_policy:/opt/gear_sonic_deploy/policy/sonic_v1_1 \
  -v trt_planner:/opt/gear_sonic_deploy/planner/target_vel/V2 \
  ferox/sonic-deploy:v1.1-x86_64 \
  /opt/gear_sonic_deploy/target/release/g1_deploy_onnx_ref \
    lo policy/sonic_v1_1/model_decoder.onnx reference/example/ \
    --obs-config policy/sonic_v1_1/observation_config.yaml \
    --encoder-file policy/sonic_v1_1/model_encoder.onnx \
    --planner-file planner/target_vel/V2/planner_sonic.onnx \
    --input-type zmq_manager --output-type all --zmq-host localhost --zmq-port 5556
python3 scripts/mm4_sonic_drive.py --host 0.0.0.0 --port 5556
```

**`--iface lo` on the bridge is required.** SONIC takes its DDS interface as argv[1]
and is given `lo`; a bridge left on the cyclonedds default never discovers it, and the
symptom is SONIC printing "LowState is not available, waiting for robot to be ready"
forever while the bridge happily publishes at 1041 Hz to nobody.
