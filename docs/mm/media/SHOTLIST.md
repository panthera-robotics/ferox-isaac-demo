# MM campaign — shot list

Every gate ends with a clip. MM8 assembles from this list; **nothing gets
re-shot**, so anything wrong here is wrong in the final video.

**How every clip is shot.** `tools/film.py`, converged path
(`rep.orchestrator.step(rt_subframes=32)`), offscreen, 30 fps, 1920×1080, title
card carrying the gate and a one-line result. The legacy `world.step(render=True)`
path is what produced the DT montage's hand trails and is never used here — see
`RESULTS_MM0` and `evidence/MM0/ghosting/`.

**Ghost gate.** Every clip on this box is marked **"visually clean, numeric ghost
gate deferred to 4090"**. The mask-based metric is implemented in `film.py`
(`ghost_pixels()`, threshold 0.5 % of mask area) but cannot run here: the
instance-segmentation annotator's `get_data()` segfaults in the same warp
device-to-host copy as the ROS 2 image writer (**C-23**, widened). The number is
measured once on a 4090 day, against these same clips.

**PiP.** The D435i picture-in-picture track is a 4090 item. `film.py --pip` raises
rather than quietly emitting a montage with a missing track.

---

## Clips

| # | file | gate | length | what it shows | business caption | ghost gate |
|---|---|---|---|---|---|---|
| — | *(none yet)* | | | | | |

## Hero shots

Marketing-grade 10–15 s, one per gate that has one.

| # | file | gate | what it shows | caption |
|---|---|---|---|---|
| — | *(none yet — MM1's is the turning walk, and it waits on the yaw diagnosis)* | | | |

---

## Status 2026-08-19: both routes to a clip are blocked on this box

| route | state |
|---|---|
| **live sim** (`isaac/twin/film_live.py`, `TWIN_FILM=1`) | **delivered**, and segfaults this box on camera creation — **C-23**, proven by A/B against an otherwise identical `TWIN_FILM=0` boot that runs fine |
| **standalone** (`tools/film.py --drive policy`) | renders fine, but the policy's gains are never applied (`kp` 35809.9, `kd` 0.000), so the robot is not driven by the deployed controller |

Neither is a coding error left to fix. The live hook is the right answer and runs on
a 4090 day. Until then, **MM1 and MM2 are PASS-with-deviations for media**, and say
so in their RESULTS.

## Why MM1's clips are not here yet

**Second, harder reason as of 2026-08-19: `film.py` cannot yet film the real policy
at all.** Its `orbit_walk` drive is a sine on six joints — the tool's self-test, not
locomotion. `--drive policy` was added and three genuine bugs fixed behind it
(paused timeline → 600 static frames; robot spawned lying down at z=0.116; a 1/60 s
physics step that turned 50 Hz control into 15 Hz), but the robot is still flung
rather than walked. Details and evidence in `RESULTS_MM1` §4. **Nothing is to be
shot with `--drive policy` until that is fixed** — the twin walks fine, the camera
harness does not.



MM1's hero shot is **the turning walk**, and yaw does not work yet
(`RESULTS_MM1` §2). Filming a robot that cannot turn and captioning it "turning
walk" would be the montage lying, which is the one thing the DT campaign's media
rules forbid. MM1's clips land when the yaw item closes — either because it starts
turning, or because the gate reports honestly that it does not and the clip shows
what it actually does.

The nav fix (MM1 §1) does have a shootable result, but it too needs the live sim
rather than `film.py`'s standalone scene, so it moves to MM2 with the walk clip.
