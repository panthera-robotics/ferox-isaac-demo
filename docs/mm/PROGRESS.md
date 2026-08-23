# MM campaign — session 15 (2026-08-23) — MANIPULATION PARKED

## Verdict: manipulation is KINEMATICALLY CHARACTERIZED, not mistuned

**Next session starts here: `docs/mm/MANIPULATION_VERDICT.md`** — the answer, the three
viable next paths, and the standalone collision-aware-planning item.
Run-by-run evidence: `docs/mm/evidence/MM5/ENVELOPE_VERDICT.md`.

**Two hard numbers.** Dex5 closed fingertip spread **21.5 mm**. Palm vertical ceiling
**0.946 m max / 0.935 m median** (`pelvis_z` 0.80, fixed base) — measured this session
from 10 converged REACH equilibria. The ceiling retro-explains the campaign: the 66 mm
can centres at 0.95 m, *at* the ceiling, and is the only object that ever closed.

**The reachable band and the graspable band do not overlap.**

| object | surface | centre z | reachable | cages | lifts |
|---|---|---|---|---|---|
| can 66 mm | 0.90 | 0.950 | yes | no — pinch (0 links within 45 mm of axis, 0 below centre, 43.83 N) | no |
| cube 50 mm | 0.90 | 0.950 | marginal | no closure | no |
| block 30 mm | 0.90 | 0.915 | no (67–173 mm, elbow+wrist pinned) | untested | no |
| block 30 mm | 1.02 | 1.037 | no (flat 95–118 mm, above palm ceiling) | untested | no |
| block 30 mm | **0.933** | 0.952 | **yes, 5/10 in <1.3 s** | **no — topples it (36.9°)** | no |
| block 30 mm | riser pad | 1.037 | **VOID** (my pad blocked the approach) | — | — |

Size and height are **coupled**: a smaller object sits lower, and lower is further from
a shoulder already at its limit — shrinking the object to fit the hand moves it out of
reach. There is no object size in this scene the hand both reaches and cages.

## Deviations / decisions

1. **Surface-height ruling amended by the user mid-session**, then the height itself was
   chosen from data rather than the brief: the brief said ~1.0–1.05 m "or wherever the IK
   classifier says the grasp pose sits mid-range". At 1.02 m the object sat ~100 mm ABOVE
   the measured 0.946 m palm ceiling (all 10 trials flat at 95–118 mm), so the final cell
   was run at 0.933 m — putting the block's centre at the can's proven 0.95 m.
2. **Three apparatus defects caught, all mine, all logged** (ENVELOPE_VERDICT §4):
   a 45 s descend timeout that hid convergence; a riser pad that put a vertical face in
   the approach corridor (palm stalled 45–54 mm short of it, 9/10 trials — the DLS servo
   has no obstacle term); and `MM5_COUNTER_H` missing from the `01_start_sim.sh` `-e`
   allowlist, which staged the block inside a 1.02 m slab and sent all 10 trials chasing
   it on the floor at an identical 568 mm. The identical constant is what exposed it.
3. **New planner finding, independent of grasping:** the DLS IK has no obstacle term and
   will drive the hand through furniture. Needed before any cluttered scene.
4. Preflight GREEN on all 4 gauges before every number quoted here.

## Kept instruments

Enclosure logger, IK classifier (`IK_INFEASIBLE` names pinned joints vs `SERVO_SLOW`),
4-gauge preflight. Fixed a hardcoded "(0.90 m)" in the staging log that reported a 1.02 m
run as 0.90 m — a misleading log line is how instrument defects start.

## Next

Manipulation is parked. It needs a graspable object (feature ≤21.5 mm at the contact
band), a pinch-grasp primitive, or a gripper — NOT tuning. Obstacle-aware IK is the one
open engineering item this session created.

---

# MORNING BLOCK — 2026-08-23 session 2 (grasp closed, real-only reel)

## Task 1 — **REACHABLE-BUT-PINCHES.** The three-session question is answered.

**(a) worked; (b) was not needed and not run.** Pulling the IK null space toward **wrist
mid-range** cleared the `right_wrist_pitch`/`right_wrist_yaw` pinning that half of all
descent timeouts showed. Closure went from rare to repeatable.

**The enclosure test finally ran, and it is decisive:**

    [enclose] links within 45 mm of the object axis: 0;  BELOW its centre: 0
    closed: 6 finger links in contact, 43.83 N total
    NO_GRIP: object rose -0.002 m

**Six contacts at 43.8 N and not one link around the object.** The hand **pinches**; it
does not **cage**. That single fact explains every failed lever: force (38–91 N), friction
(μ 0.5→1.2), contact count (1→6), thumb opposition, lift vector, lift rate and grip
maintenance were each genuinely fixed, and **none can lift an object the hand is not
around**.

**A trap worth carrying:** `ik_wrist_null_gain = 4.0` removes the pinning but creates a
**null-space equilibrium** — three trials converged to the *identical* arm pose with the
residual frozen at 59–60 mm. An identical repeated pose is a fixed point; a spread is a
reach limit. **1.5 is correct.**

## Task 2 — 60 s reel, **everything in it is real**

No cheat-attach, no choreography. A closing card states manipulation is excluded and why.

**One real defect fixed at source:** `film.py`'s `scene()` accepted a `world_usd`
parameter **and never used it**, so every previous reel was an infinite grid — nothing for
the eye to measure motion against, which *is* the "sliding" look. Now referenced and
**logged every run** (`[film] world: …`), because a flag that is accepted and silently
ignored looks identical to one that works: I filmed 960 frames of grid before a frame
check caught it. Added a `hero` shot that arcs and dollies with smoothstep easing.

## Media — release `mm-persist-13`

https://github.com/panthera-robotics/ferox-isaac-demo/releases/tag/mm-persist-13

| asset | caption |
|---|---|
| `ferox_g1_real_reel_20260823.mp4` (60 s) | "The reel, all real: omni walk in a lit hospital plus live D435i colour+depth. Manipulation is excluded and the film says so." |
| `mm_lit_hero_20260823.mp4` | "Omni policy walking 6.159 m vs 6.0 commanded, hero camera on an eased arc-and-dolly." |
| `mm_lit_chase_20260823.mp4` | "The same walk, lagged chase camera, textured floor and shadows so travel reads as travel." |
| `mm_lit_pip_20260823.mp4` | "Lit-hospital chase with live D435i colour + depth insets (captured separately, composited)." |

sha256 for each is in `docs/mm/CAPTURES.md`.

## Top 3 decisions for Mohammed

1. **Grasp needs an aperture/approach change, and nothing else.** The object must sit
   *between finger segments*, not at their tips: the Dex5 converges 0.1366 m from the palm
   with a 21.5 mm tip spread against a 66 mm can. The enclosure logger now measures this
   directly, so the next attempt is measurable rather than arguable.
2. **Every instrument is verified and the preflight is mandatory.** Five instrument
   defects in one session produced five confident wrong numbers. Treat any grasp figure
   without a green preflight as unverified.
3. **Two blockers stay closed and neither costs money:** C-39 (omni balances — do not fund
   the harness comparison) and C-23 (`headless: False` — buy no GPU).

## Exact next command

```bash
cd ~/panthera/ferox-isaac-demo && git checkout mohammed/mm-campaign && git pull
ROBOT=g1 TWIN=1 TWIN_HEADLESS=1 HAND=dex5_1p SIM_WORLD=panthera_lab \
  bash scripts/01_start_sim.sh && bash scripts/mm5_preflight.sh   # gauges FIRST, always
```

---

# MORNING BLOCK — 2026-08-23 (final: reel shipped, everything survives this box)

## A fresh instance can resume from GitHub + the release alone — **verified, not asserted**

Clean clone of tag `mm-persist-12` into /tmp, no access to this box's state:

    HEAD 9083220 (correct)                    RESUME.md / CAPTURES.md / PROGRESS.md present
    TASK1_IK_VERDICT.md, C23_RESOLVED.md, mm5_preflight.sh present
    test_twin_contract.py    38/38 passed
    test_isaaclab_cfg.py     10/10 passed, 3 SKIPPED (named, needs ferox-g1-locomotion beside)
    release assets           8/8 state=uploaded
    downloaded asset sha256  ad0de00dcc95765e == CAPTURES.md  MATCH

**One real persistence bug was caught doing this**: creating the release via the API
auto-made the tag on the **default branch**, so `mm-persist-12` pointed at `d03f2e9` —
old main, none of this session's work. A fresh clone would have got the wrong tree. The
tag was re-pointed at `9083220` and re-verified.

## Task 1 verdict — descent/IK, not grip

Half the `DESCEND_TIMEOUT`s are `IK_INFEASIBLE` with **`right_wrist_pitch`/`right_wrist_yaw`
pinned at their stops**; the rest are `SERVO_SLOW`, stalled 43–70 mm with nothing pinned
and **unchanged at a 45 s window**. The commanded **palm orientation** at counter height is
not achievable by this wrist. **Lever (b) — moving the object central — was deliberately
NOT run**: (a) showed the pose is reachable, so (b) addresses a constraint that is not
binding. **Enclosure is still unmeasured** (closure too rarely reached).

## What is real vs choreographed in the reel

| segment | status |
|---|---|
| omni walk, chase + fixed | **REAL** — 6.159 m vs 6.0 m commanded, measured from base pose |
| D435i PiP (colour + depth) | **REAL** — live camera; insets captured in the hospital world, composited over the walk (two captures, not one shot) |
| pick sequence | **CHOREOGRAPHY** — scripted trajectory + cheat-attach, CAMPAIGN §0.6, banner burned into every frame |
| film-tool clips | **PIPELINE PROOF ONLY** — a scripted joint sweep; the robot is not walking |

## Media — release https://github.com/panthera-robotics/ferox-isaac-demo/releases/tag/mm-persist-12

sha256 + an honest one-line caption for all 8 assets are in `docs/mm/CAPTURES.md`.
Headline: `ferox_g1_mm_reel_20260823.mp4` (74 s, 17.3 MB, `e34f01454d9819fb…`) —
*"Real omni walk + live D435i camera + a clearly-labelled scripted placement; nothing here
is a real grasp."*

## Top 3 decisions for Mohammed

1. **Grasp next lever is an orientation question, not a grip one.** Either re-derive/relax
   the commanded palm orientation at counter height (the approach axis blends horizontal
   there, which is what loads the wrist), or reposition the base so the wrist sits
   mid-range. Both are one change; I opened neither, per instruction.
2. **Preflight is now mandatory and it changed the campaign.** Five instrument defects in
   one session produced five confident wrong numbers — none robot defects. Treat any grasp
   figure without a green preflight as unverified.
3. **Two blockers are closed and neither needs money:** C-39 (omni balances; do not fund
   the harness comparison) and C-23 (`headless: False`; buy no GPU).

## Exact next command

```bash
cd ~/panthera/ferox-isaac-demo && git checkout mohammed/mm-campaign && git pull
ROBOT=g1 TWIN=1 TWIN_HEADLESS=1 HAND=dex5_1p SIM_WORLD=panthera_lab \
  bash scripts/01_start_sim.sh && bash scripts/mm5_preflight.sh   # gauges FIRST, always
```

---

# MORNING BLOCK — 2026-08-23 (final: verified instruments, grasp verdict written)

## The one-line answer

**Descent, not grip, is the binding constraint — and the instruments, not the robot, were
the bottleneck for most of this campaign.**

## Grasp — verdict, per the decision rule set before the run

| branch | what happened |
|---|---|
| cube lifts → enclosure proven | — |
| cube closes, no lift → pinch-not-cage | — |
| **cube never reaches closure → descent is the limit** | **THIS.** 0/8, `DESCEND_TIMEOUT` ×6 (43–153 mm), `REACH_TIMEOUT` ×2 |

The cube was chosen because it cannot topple or roll out of a pinch, so it was the clean
test of enclosure. **The test never ran — the hand never arrived.** Enclosure is therefore
**unmeasured, not disproven**, and the can could not answer it either: 2 of 4 can trials
ended `TOPPLED` (30.0°, 30.3°) *during* the closure ramp, before the gate the enclosure
logger fires at.

## Eliminated by measurement — do not re-run these

grip force (38–91 N vs 3.4 N needed) · finger gain · URDF effort clamp (`|tau|` 0.006 vs
0.93) · overclose (joint blocked 0.680/1.600) · thumb opposition (`Link_14R` carries the
most force) · contact count (1 → 6) · friction (μ 0.5 → 1.2) · lift target geometry ·
lift rate · **grip maintenance through LIFT** (fixed: contacts now held 1–3 links at
35–91 N under a 0.03 m/s rate limit)

**Still never true: the object rises. `d_obj ≤ +4 mm` in every trial, all session.**

## Instruments — the finding that outlives the session

**Five instrument defects, all mine, none robot defects.** Each produced a confident wrong
number; one I quoted to Mohammed as confirmation before catching it.

`scripts/mm5_preflight.sh` is now **mandatory before grasp work** and asserts every gauge
against a known answer — including, as of today, a **sustained-contact** gauge that
distinguishes a steady 20 N hold from a 91 N impact spike, because the existing gauge
reports peak-over-window and would have justified a false lift claim.

## Corrections I made to my own record

* the "0.5 N vs 3.4 N" force premise — **the 0.5 N was never newtons** (summed impulses)
* `obj_pose` as "the root cause of the grasp workstream" — real defect, **not** the cause
* the topple, claimed then retracted — **both overstatements**; it is real and
  intermittent (30.0°/30.3° on some approaches, 0.0° on others)
* `--drive policy` crashing in the C-23 subsystem — **wrong**, it was a missing
  `FEROX_REUSE_KIT_APP=1`

## Top 3 for Mohammed

1. **Descent is now a SPREAD (37–171 mm), not a constant.** A constant meant a wrong
   number and was fixed (`grasp_standoff` 0.045 → 0.1466). A spread means the arm cannot
   always achieve the pose: measure per-trial IK residual against joint limits, or move
   the object to a more central part of the workspace and test enclosure there.
2. **Enclosure is still the open question** and needs closure to be reliable first. The
   logger is written and fires at the closure gate.
3. **C-39 and C-23 are both closed** — omni is the balancer (6.159 m on film), and the
   camera works headless. Neither needs more hardware.

## Exact next command

```bash
cd ~/panthera/ferox-isaac-demo && git checkout mohammed/mm-campaign && git pull
ROBOT=g1 TWIN=1 TWIN_HEADLESS=1 HAND=dex5_1p SIM_WORLD=panthera_lab \
  bash scripts/01_start_sim.sh && bash scripts/mm5_preflight.sh   # gauges FIRST
```

---

# MORNING BLOCK — 2026-08-23 (method reset: verify the gauges, then trace the transition)

## The headline is a method finding, not a robot finding

**Five instrument defects in one session, each of which produced a confident wrong
number. None were robot defects.** Mohammed called it before I did.

| defect | what it claimed |
|---|---|
| contact "force" summed PhysX **impulses** (N·s), L1-normed, printed as newtons | "0.48 N vs 3.4 N needed" — a comparison that could not be made |
| the fix reported only the **last substep** | real contacts read as `contacts=0` |
| contact count carried **no identity** | 4 pushing one way looked like 2 opposing |
| tilt measured against **world +z** on an asset whose local +z is not its cylinder axis | an **untouched** can read 90° → two false `TOPPLED` verdicts, one of which I quoted as confirmation |
| tilt baseline **re-latched per trial** | `tilt-at-rest` read 0.0 by construction, destroying the check that had caught a can staged at 44.7° |

## 1. PREFLIGHT — permanent, and it earned its keep immediately

`scripts/mm5_preflight.sh` asserts every gauge against a known state. Non-zero exit means
**do not trust grasp numbers**.

    [PASS] tilt at rest ............ 1.11 deg   (< 2)
    [PASS] centre drop at rest ..... -0.0 mm    (< 5)
    [PASS] scripted 30 deg tip ..... 29.86 deg  (25..35)
    [PASS] scripted +80 mm rise .... +78.7 mm   (> 50)
    PREFLIGHT: ALL GAUGES VERIFIED

It **failed twice on its first run, for two different reasons** — which is the argument
for known-answer checks in one line:

1. a **real gauge defect** — tilt via a body axis under-reported a known 30° tip as
   18.45°; now the frame-independent quaternion angle;
2. a **defect in the test itself** — the can is a dynamic body, so stepping physics let
   gravity settle it before the read. I nearly "fixed" a gauge that had just become correct.

## 2. THE LIFT TRANSITION — traced, and it picked a hypothesis

Before (target jumped the full 0.14 m at once):

    GRASP closed: 6 contacts, 69.81 N
    LIFT t=0.0s hand_z=0.9574 obj_z=0.9508 contacts=0 force=0.0N
    LIFT t=7.6s hand_z=1.0802 obj_z=0.9508 contacts=0 tilt=0.0

**(a) confirmed** — contacts 6 → 0 *before the hand moved*. **(b) refuted** — the hand
rose 12.5 cm, IK error 135 → 9 mm. **(c) refuted** — object never moved, never tipped.
Two causes fixed: the grip was **never re-commanded during LIFT** (GRASP re-issues
`_set_hand` every step, LIFT did not), and the lift target jumped instantly so the servo
yanked the hand 10 cm in 0.5 s.

After (rate-limited 0.03 m/s + grip held):

    t=0.5s hand_z=0.9595 contacts=0
    t=2.0s hand_z=0.9816 contacts=2 force=35.2N tilt=8.5 d_obj=+4mm
    t=2.5s hand_z=0.9983 contacts=3 force=91.5N d_obj=+1mm
    t=3.5s hand_z=1.0382 contacts=1 force=47.3N d_obj=+2mm

Contacts are now **maintained through the lift** (1–3 links, 35–91 N) and the hand rises
smoothly. **`d_obj` never exceeds +4 mm.**

## 3. What that means, and it is a new failure

**The grasp closes without ENCLOSING.** The fingers touch the can; they do not cage it.
The hand slides up past it, brushing it — which is why force, friction, contact count,
lift vector and lift rate have each been fixed without producing a lift. Nothing that
acts on *contact* can lift an object the hand is not *around*.

**0 lifts. No capability is claimed.**

## Ruled out by measurement, so nobody re-runs them

grip force (38–91 N vs 3.4 N needed) · finger gain · URDF effort clamp (`|tau|` 0.006 vs
0.93) · overclose (joint blocked 0.680/1.600) · thumb not opposing (`Link_14R` carries the
most) · contact count (1 → 6) · friction (μ 0.5 → 1.2) · lift target geometry · lift rate ·
grip maintenance during lift · **topple (tilt 0.0–1.7° with a verified gauge)**

## Next session — measure, do not guess

1. `scripts/mm5_preflight.sh` first. If it fails, fix the gauge before anything else.
2. **Measure the enclosure**: at closure, log every finger link's position in the CAN's
   frame and the can's radius/height. The question is whether any finger passes *under or
   around* the can's widest point. `dex5_geom.py` already does this maths for the palm
   frame; point it at the object.
3. Only then change geometry — and add a **sustained**-contact gauge, since the current
   one reports peak-over-window, which is right for "did it touch" and wrong for
   "is it holding".

# MORNING BLOCK — 2026-08-22 (session 2: decisions 1-4)

## Real vs pipeline-proof — the distinction that matters most in this file

| thing | real? |
|---|---|
| **Omni policy walks 6.159 m** vs 6.0 m commanded, upright throughout | **REAL** — measured from base pose |
| **C-23 fixed**, camera runs, aligned-depth check green | **REAL** — `fx=908.0`, depth 306/1668/2574 mm, zero-fraction 0.0 |
| **Grasp reaches closure**, contacts measured | **REAL but partial** — 4/20 closures, **0/20 lifts** |
| `mm_filmtool_*` clips | **PIPELINE PROOF ONLY** — scripted joint sweep, not a capability |
| MM5 end-to-end | **NOT RUN** — needs a grasp that holds |
| PiP camera track | **NOT SHOT** — unblocked now, but not filmed |

## Decisions 1–4, all closed

1. **C-39 parked.** Omni is the twin's balancer; SONIC is hardware-only, finetune → Spark.
   RESUME + CAMPAIGN updated; MM4's balancing requirement is met by omni-hold.
2. **C-23 RESOLVED — it was `headless: False`.** Eight probes, one variable each: A–G all
   survive (camera → bridge → articulation → hospital → the twin's own `create_camera` →
   rclpy threads → TorchScript on CUDA). Probe H is G with that one boolean flipped and
   dies on render step #2. `run.py` hardcoded it; this box has no logged-in X session.
   `TWIN_HEADLESS=1` added. **No GPU was ever needed.**
3. **Grasp descent fixed; grasp not achieved.** `grasp_standoff` defaulted to 0.045 m
   against a *measured* 0.1366 m finger convergence — target ~92 mm inside the can, stalls
   80–101 mm. v4 measured it right and applied it only behind `MM5_MEASURE_HAND=1`.
4. **MM3/MM4 re-measured.** See `evidence/REMEASURE/`.

## Numbers

**Grasp v7, N=20 (10 can, 10 cube):** `0/20 lifts`. `DESCEND_TIMEOUT 16 · NO_CONTACT_AT_CLOSURE 4`.
First closure in campaign history: `at grasp pose, 23 mm` → `1 finger link, 0.48 N`.
DESCEND went from a 15 s timeout to **1.08 s**. Closure forces 0.00–0.48 N against a
0.349 kg can that needs ~3.4 N — an order of magnitude short.

**Re-measurement (post-seqlock):** `lowcmd_sent 499.976 Hz` (sender), **`lowstate_recv
206.530 Hz`** (client — a fifth of what is published, never measured before),
`track_err mean 0.1216 rad / max 1.0085 rad` (right_elbow), fail-closed engages, torn
reads stop growing. MM4's SONIC-side latency figures stay **flagged, not quoted**.

## Media

| file | one-line honest caption |
|---|---|
| `ferox_g1_motion_manip_20260822.mp4` (52 s) | "The twin's omni policy walks 6.16 m; everything else here is a pipeline proof or a scorecard." |
| `mm_omni_walk_{chase,front}_20260822.mp4` | "Omni locomotion policy walking 6.159 m against 6.0 m commanded, measured from the base pose." |
| `mm_filmtool_{chase,front}_20260822.mp4` | "Film-tool self-test: a scripted joint sweep. The robot is not walking." |
| `evidence/C23/aligned_depth_20260822.json` | "First working D435i frame from this twin: rgb and depth aligned at 720×1280." |

## Top 3 decisions for Mohammed

1. **Grasp is one defect from working, and it is force, not geometry.** Descent and
   contact sensing are fixed; closure delivers ~0.5 N where ~3.4 N is needed. Test in this
   order: GRASP-phase finger gains (kp 20/kd 0.5), `overclose_rad` (0.25), and whether the
   thumb opposes at all — tip spread is 21.5 mm against a 66 mm can. **That is the whole
   remaining gap to MM5.**
2. **Descent is inconsistent, not blocked** — stalls now spread 49–160 mm where they were
   a constant 92. A constant meant a wrong number; a spread means the IK cannot always
   reach the pose. Measure per-trial IK residual against joint limits before tuning.
3. **Audit the config defaults against the measurements.** Three separate multi-day
   stalls this campaign came from a measured value that never reached the default:
   `grasp_standoff` (0.045 vs 0.1366), `obj_pose` (fixed for the palm, not the object),
   and C-23 (diagnosed from what the crash touched, not what the process differed in).

## Exact next command

```bash
cd ~/panthera/ferox-isaac-demo && git checkout mohammed/mm-campaign && git pull
# grasp force, the one gap left to MM5:
ROBOT=g1 TWIN=1 TWIN_HEADLESS=1 TWIN_CAMERA=0 HAND=dex5_1p SIM_WORLD=panthera_lab \
  MM5=1 MM5_OBJECT=soup_can MM5_TRIALS=5 MM5_FIX_BASE=1 MM5_SURFACE=counter \
  MM5_MEASURE_HAND=1 TWIN_HAND_KP=60 MM5_OUT=/workspace/ferox_isaac/mm5_force \
  bash scripts/01_start_sim.sh
```

---

# MORNING BLOCK — 2026-08-22 (4090 session)

## Tasks

| # | task | result |
|---|---|---|
| 0 | record the physics correction | **DONE** — static-hold fall is not a discriminator; the reference fails the same test |
| 1 | C-39 decisive A/B | **DONE — ANSWERED.** Asset exonerated; no PhysX property stands SONIC; **SONIC parked** |
| 2 | full MM4 | **SKIPPED** per the brief (SONIC did not stand) |
| 3 | grasp v7 | **DONE — 0/10.** Contact route attaches; failure moved reach → descend |
| 4 | mobile MM5 | **NOT RUN** — needs a working grasp; would have been 0/N with a known cause |
| 5 | camera backlog | **PARKED — C-23-v2.** Not the GPU, not the writer; five components cleared |
| 6 | montage | **PARTIAL — the real-motion clip landed**, measured 6.16 m walk. No PiP (C-23) |
| 7 | this block | **DONE** — `mm-persist-8` |

## The C-39 verdict

**The reference MuJoCo body, imported unmodified, falls in our simulator exactly as ours
does** — `base_z 0.778 → 0.098`, `pitch +86.3°`, against our own `+88.3°`. The same body
stands under the same binary in the reference MuJoCo sim. Nine runs, zero void rows:

    baseline_twin  FALLS   baseline_ref  FALLS   solver_iters  FALLS
    friction_mult  FALLS   contact_off   FALLS   depen_vel     FALLS
    self_coll      FALLS   dt_200hz      FALLS   implicit_drive FALLS

Every row lands in the same place — base ~0.10 m, pitch ~87°, ~2.5 s after release. A
parameter that mattered would move that number. **C-39 is not the asset, the wire, the
hands, SONIC, or any single PhysX setting.** What is left is the Isaac harness against
MuJoCo *as a whole* — actuator model, contact solver family, MuJoCo's own 200 Hz
constraint formulation — and that is bigger than a sweep.

**Margin, honestly: there isn't one.** It does not hold and lose it; it goes from
released stance to face-down in ~2.5 s, in all nine configurations, on both bodies.

## MM4 / MM5 / grasp numbers

* **MM4** — not run beyond C-39; SONIC parked, finetune → Spark.
* **Grasp v7, N=10 soup can, counter: 0/10.** `DESCEND_TIMEOUT` 9, `REACH_TIMEOUT` 1.
  **REACH is now solved** — 2.4 s on 9/10, where the table surface timed out at 30 s.
  The can is displaced 0.30–0.37 m in 8/10: the palm drives *into* it. `grip_contacts`
  is still `-1` on every row — **no trial reached GRASP, so the contact route is
  unexercised at closure and no `NO_GRIP` row is evidence about colliders.**
* **MM5 mobile** — not run. It needs a grasp.

## Media

| clip | what it is |
|---|---|
| `mm_omni_walk_{chase,front}_20260822.mp4` | **the real one** — omni policy, **6.159 m measured** vs 6.0 commanded, upright throughout |
| `mm_filmtool_{chase,front}_20260822.mp4` | pipeline proof only (scripted swing) — **not** a capability demo |

Ghost gate **PASS 0.00095/0.01**. sha256 for all four in `media/README_20260822.md`.

## Top 3 decisions for Mohammed

1. **C-39: fund the harness comparison, or accept the twin cannot host SONIC.** Every
   cheap hypothesis is dead. The remaining work is comparing our Isaac harness to MuJoCo
   as a whole (actuator model, contact solver, constraint formulation) — days, not hours.
   The alternative is to keep the omni policy as the twin's balancer, which **works and is
   now on film**, and treat SONIC as hardware-only.
2. **C-23: stop buying GPUs for it.** Reproduced on 4080 and 4090 with five components
   individually cleared. The untested lead is rclpy executor threads racing the SDG
   pipeline; the next experiment is one 40-line probe, written up in `C23_v2.md`.
3. **Grasp: the next fix is the descent geometry, not the gripper.** REACH is solved and
   the hand now arrives; it drives into the can because the approach axis is computed for
   a target below the shoulder while the counter puts it level. That is a bounded change.

## Exact next command

```bash
cd ~/panthera/ferox-isaac-demo && git checkout mohammed/mm-campaign && git pull
# then, in the sim container, the one probe that moves C-23:
#   probe E from C23_v2.md + two rclpy spin threads, and step.
```

## Health warning on older numbers

`RESULTS_MM3.md` / `RESULTS_MM4.md` carry a banner: the lowcmd seqlock dropped **73 %**
of commands (three writers, no lock), so every torque, PD-tracking and latency figure
taken before that fix must be re-measured. Convention/CRC work and offline asset
arithmetic are unaffected.

---

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
