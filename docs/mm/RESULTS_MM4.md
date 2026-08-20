# RESULTS_MM4 — SONIC in the twin

Host: RTX 4080 SUPER 16 GB (sm_89) / driver 580.105.08 / CUDA 13.0
Date: 2026-08-20
Verdict: **PARTIAL** — the image and the interop are done and proven; the scripted
sequence is not.

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
| stand → weight shift → walking → turn → stop → POSE arms | **FAIL** | `evidence/MM4/sonic_zmq_sequence.txt` |
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

## What does not work

SONIC never completes its planner handshake:

```
Planner Init timing - Model: 5494us, Extract: 30us
[ZMQManager] Waiting for planner to be initialized      (x14)
[ZMQCommandManager ERROR] Planner initialization timeout
Stop
```

The planner **model runs** — so the message is being parsed and inference happens —
but the manager never marks it initialized. Two bugs were found and fixed on the way
here, and both are worth keeping because both presented as silence rather than error:

* **`dtype: "b8"` is not in SONIC's vocabulary.** Its decoder compares against exactly
  `bool, u8, i8, i16, i32, i64, f16, f32, f64`. A `b8` boolean made every command
  message fail the `start/stop/planner` field check, and SONIC held a static default
  pose through an entire scripted sequence while looking perfectly healthy.
* **`movement` and `facing` are 3-vectors, not 2.** `OnPlannerReceived` memcpy's a
  fixed `i<3` loop out of each buffer, so a 2-element field is read one float past its
  end — not a parse error, just a garbage third component, surfacing only as
  "Planner initialization timeout".

A third mismatch remains and is not yet identified. The likely candidates, untested:
the planner may require `upper_body_position`/`upper_body_velocity` or the hand-joint
fields to be present rather than optional; or it may need the `pose` topic streaming
concurrently; or the handshake may expect a specific `count`/shape in the header that
this publisher does not set.

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
