# RESULTS_MM0 — box + carry-over

**Host:** Vast.ai · **NVIDIA GeForce RTX 4080 SUPER, 16376 MiB** · driver 580.105.08 · Isaac Sim 5.1.0
**camera-capable box: NO** (C-23) — everything below ran with `TWIN_CAMERA=0`
**Date:** 2026-08-19 · **Verdict: PASS-with-deviations**

Two of the eight sub-items are blocked by the box and are declared open, per the
campaign status header. One more — the film tool's ghosting test — is delivered but
**its power is unproven**, and that is written up rather than reported as green.

---

## Scorecard

| Requirement | Status | Evidence |
|---|---|---|
| §1 `nvidia-smi` model/VRAM recorded | **flag, not gate** — RTX 4080 SUPER 16376 MiB, `camera-capable: no` | this header |
| §1 RESUME verification: contract suite | **PASS** 38/38 | `tools/tests/test_twin_contract.py` |
| §1 RESUME verification: Isaac suite | **PASS** 13/13 | DT tag `twin-g1-fixed-2` |
| §1 RESUME verification: sim boots `mode:=twin` | **PASS** (main loop 35 s, `TWIN_CAMERA=0`) | `/tmp/mm0_sim.log` |
| §2 hand-roll numeric check, world frame at URDF zero pose | **PASS** 6/6 dots ≥ 0.9 | `evidence/MM0/hand_roll.{txt,json}` |
| §2 renders top/front/side | **PASS** (re-rendered on the corrected asset) | `docs/twin/evidence/DT2/g1_twin_{front,side,top}.png` |
| §2 aligned-depth check + C-21 clip | **BLOCKED — C-23** | queued for a 4090 day |
| §2 `/scan` re-shot in hospital, ≥45 % finite | **PASS** 45.2–45.5 % | `evidence/MM0/scan_hospital.txt` |
| §2 nav 3/3 inside measured free-space bounds | **FAIL — 0/3**, and not for the reason expected | `evidence/MM0/nav_goals.txt` |
| §3 `tools/film.py` v1 | **PASS** (delivered, runs, calibrates) | `tools/film.py` |
| §3 ghosting test | **DELIVERED, POWER UNPROVEN** — negative control also passes | `evidence/MM0/film_negative_control.txt` |
| §3 20 s walk clip, no trails | **not shot** — see the ghosting finding | — |

---

## Numbers

### Hand-roll, world frame at URDF zero pose (MM0.2's form)

Three mutually orthogonal hand vectors against three different reference axes.

| dot | left | right | min |
|---|---|---|---|
| fingers · body +X | **+0.9999** | **+0.9999** | 0.9 |
| palm normal · toward sagittal plane | **+0.9999** | **+0.9999** | 0.9 |
| thumb · world +Z | **+1.0000** | **+1.0000** | 0.9 |

### The same geometry in the wrist frame (DT tooling) — they agree

| | left | right |
|---|---|---|
| fingers vs wrist +X | 0.707° (cos 0.99992) | 0.707° |
| palm normal vs the midline | 0.707° | 0.707° |
| thumb vs wrist +Z | 0.518° (cos 0.99996) | 0.518° |
| chirality | **PASS** | **PASS** |

At URDF zero pose the pelvis is identity and the wrist frame coincides with the body
frame (wrist X = body X, wrist Z = world Z), so the two tables are the same
measurement in two coordinate systems and must agree. They do, to 4 decimal places.

**Correction to my own earlier objection.** I said the ≥0.9 triple looked
unsatisfiable at zero pose because "fingers along the forearm" and "thumb forward"
are near-orthogonal. That was wrong: the three vectors are compared to *three
different* reference axes, which are themselves orthogonal, so all three can be ≥0.9
at once. The spec is satisfiable as written and the twin satisfies it.

### `/scan` in hospital, beside the bag

| | finite | rays | geometry |
|---|---|---|---|
| twin | **45.2–45.5 %** | 723 | ∓3.14159, inc 0.0087, 0.30–6.0 m, `base_link` |
| robot (`g1_twin_gt` bag) | 70 % | 723 | identical |

45 % is the correct answer for a mid-corridor spawn against a 6.0 m `range_max` —
see `RESULTS_DT2.md`. Requirement was ≥45 %.

### Nav — 0 of 3, and the cause is NOT scoping

The hypothesis under test was that DT's 1-of-3 was a scoping error. It was not, or
at least not only:

| step | measurement |
|---|---|
| map at spawn | 2884 free cells, interior x [7.28, 9.86] y [−0.42, 2.33] |
| first attempt, 3 goals inside that interior | goal 1 accepted, robot moved 0.27 m, BT looped `ComputePathToPose`, no terminal status in 15 min |
| mapping lap (`validate_motion.py`, ~60 s of driving) | map **2884 → 12692 free cells**, interior x [5.54, 10.19] y [−2.17, 2.78] |
| second attempt, goal (9.00, 1.00) — comfortably inside | robot drove to **(8.257, 0.800)**, i.e. **0.76 m from the goal**, and stopped |
| terminal status | **never returned.** `timeout -s KILL 130` did not reap `ros2 action send_goal`; the client was killed by hand. Goals 2 and 3 were never sent |

So: the robot **plans and drives**, closes to sub-metre, parks, and the action never
resolves. A goal outside the map is ruled out — this one was inside a map four times
larger than the one DT used. That points back at the goal-tolerance /
footprint-inflation interaction DT2 recorded and deliberately left untouched
(footprint 0.35 m against an inflation radius of 0.35 m; Nav2 itself warns the
inscribed radius is 0.360), not at where the goals were put.

**Nothing was tuned.** DT2's standing decision is that this is Ferox-side.

### Film tool — and why its ghosting test is not yet a test

`tools/film.py` delivers chase + fixed cameras with authored look-at matrices, and a
ghost test scored as `mean|X−Y|/255` where X and Y are two renders of the *same* pose
whose only difference is the pose rendered before them.

| subframes | ghost score | verdict |
|---|---|---|
| 1 | 0.00067 | PASS |
| 4 | 0.00083 | PASS |
| 8 | 0.00091 | PASS |
| 16 | 0.00103 | PASS |
| 32 | 0.00114 | PASS |
| 64 | 0.00117 | PASS |

Every count passes, and the score *rises* slightly with more subframes — sampling
noise, not accumulation. That is the shape of a knob that is not connected to
anything, so I added a **negative control**: the same test run through the legacy
unconverged path (`world.step(render=True)` + `get_rgba()`, which is what
`render_orbit.py` and the DT montage used).

```
negative control (legacy unconverged path): ghost 0.00000
  ALSO PASSES -- test has no power here
```

**The control scores zero.** The test cannot tell the two paths apart, because in
this render mode there is no temporal accumulation to detect. A test that has never
been seen to fail certifies nothing, so I am not reporting "ghosting test green".

### I could not reproduce the ghosting defect at all

Three independent measurements, none of which finds a trail. The signature used is
the correlation between consecutive difference images: smooth motion gives a
**negative** correlation (the region a body vacates is the region it arrives in next
step); a trail gives a **positive** one.

| artifact | mean consecutive abs-diff | echo correlation | verdict |
|---|---|---|---|
| source PNGs (orbit, camera moving) | 3.52 | **−0.385** | no trail signature |
| the same clip after H.264 encode | 3.63 | **−0.375** | no trail signature |
| `twin_progress_20260818.mp4` (the DT montage §3 attributes ghosting to), early clip | 0.84 | **−0.094** | no trail signature |
| same montage, later clip | 2.29 | **−0.337** | no trail signature |
| `twin_progress_g1_20260819.mp4` (mine) | 3.63 | **−0.375** | no trail signature |

Source and encoded are statistically indistinguishable, so it is not the encoder
either.

**I therefore cannot confirm the premise in campaign §3.** Either the ghosting was
seen in an artifact I do not have, or in a viewport recording rather than an
offscreen render, or it is a mis-attribution. This matters because MM8's PASS
criterion is "ghosting test green on every clip", and as things stand that criterion
would be satisfied by a test that cannot fail.

---

## What changed

| File | One line | |
|---|---|---|
| `docs/mm/CAMPAIGN.md` | the brief, saved per §8.3, with the amended GPU scoping in its status header | new |
| `tools/film.py` | chase/fixed cameras, authored look-at, per-frame convergence, ghost test + negative control | new |
| `tools/check_hand_orientation.py` | added MM0.2's world-frame dot products at URDF zero pose beside the wrist-frame table | modified |
| `docs/mm/evidence/MM0/**` | hand-roll, `/scan`, nav, film calibration + control, ghosting measurements | new |

Branch `mohammed/mm-campaign`. Tag `mm-MM0`.

---

## Deviations

* **C-23** — the box cannot run the camera. Aligned-depth and the C-21 clip stay open,
  queued for a 4090 day. Everything else ran with `TWIN_CAMERA=0`.
* **New, unnumbered pending your call:** the ghosting premise is unconfirmed and the
  ghosting test has no power. I have not opened a C-item because it is not yet clear
  there is a defect to declare — see open question 2.

---

## Open questions for Mohammed

1. **Nav.** Goals inside a 4×-larger map still do not resolve; the robot parks 0.76 m
   short. This is the DT2 footprint/inflation item, not scoping. Do you want me to
   touch the Ferox goal tolerance / inflation radius in MM1 (DT2 said do not), or
   carry nav forward as a known-open Ferox-side item? **Default taken: carried
   forward, nothing tuned.**
2. **Ghosting.** I cannot reproduce it in any artifact I have, including the DT
   montage itself. Can you point me at the clip and timestamp where you saw trails?
   Without that I would rather delete the vacuous test than ship a green light.
   **Default taken: test kept, its powerlessness documented, MM8's criterion flagged
   as currently unsatisfiable-in-spirit.**
3. The 20 s walk clip was not shot, because shooting it to certify "no trails" against
   a test that cannot fail would be theatre. It is ~15 min of work once (2) is
   settled.

---

## Reproduce

```bash
# hand-roll, both tables (no GPU, no Isaac, no ROS)
python3 tools/merge_dex5_urdf.py
python3 tools/check_hand_orientation.py --json docs/mm/evidence/MM0/hand_roll.json

# the twin, camera off
TWIN_CAMERA=0 ROBOT=g1 TWIN=1 HAND=dex5_1p SIM_WORLD=hospital \
  FEROX_SIM_TEST_PROPS=1 CAMERA_TF=1 ./scripts/01_start_sim.sh
ROBOT=g1 MODE=twin ./scripts/02_start_ferox.sh

# /scan
docker exec ferox_nav python3 /tmp/scanprobe.py

# film tool: calibrate, then prove the test can fail
docker cp tools/film.py ferox_isaac_sim:/tmp/isaacrun/film.py
docker exec -e PYTHONDONTWRITEBYTECODE=1 ferox_isaac_sim \
  /isaac-sim/python.sh /tmp/isaacrun/film.py --calibrate --shots chase --out /tmp/film/cal
docker exec -e PYTHONDONTWRITEBYTECODE=1 ferox_isaac_sim \
  /isaac-sim/python.sh /tmp/isaacrun/film.py --ghost-test --negative-control \
  --shots chase --subframes 16 --out /tmp/film/ctl
```
