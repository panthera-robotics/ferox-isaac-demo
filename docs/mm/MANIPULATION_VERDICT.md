# Manipulation verdict — start here

**Status: PARKED. Kinematically characterized, not mistuned.**
Ferox G1 29-DoF + Dex5-1P, Isaac Sim 5.1.0 / PhysX, omni locomotion policy as balancer.

Read this before doing any manipulation work. The search is finished; this is the answer.
Do not re-tune gains, clearances, friction, or the null space — section 3 explains why
none of them can move any number in section 2.

Detailed run-by-run evidence: `docs/mm/evidence/MM5/ENVELOPE_VERDICT.md` and
`docs/mm/evidence/MM5/env/`. Measurement conditions: base rig-held (`MM5_FIX_BASE=1`),
`pelvis_z` 0.80 m, `target_r` 0.315 m from the right shoulder, declared friction
`MM5_GRASP_MU=1.2` (CAMPAIGN 4.4). Every number below was taken with the 4-gauge
preflight GREEN (`scripts/mm5_preflight.sh`).

---

## 1. The two constants

| constant | value | how measured |
|---|---|---|
| **Dex5 closed fingertip spread** | **21.5 mm** | direct hand measurement (`MM5_MEASURE_HAND=1`) |
| **Palm vertical ceiling** | **0.946 m max, 0.935 m median** | 10 converged REACH equilibria (`pelvis_z` 0.80, fixed base) |

These two numbers determine everything else. The tip spread sets what the hand can
close *around*; the palm ceiling sets how high it can reach. The 66 mm soup can centres
at 0.95 m — *at* the ceiling — which is why it is the only object that has ever reached
closure in this campaign.

## 2. The envelope

| object | size | surface | centre z | REACHABLE | CAGES | LIFTS | evidence |
|---|---|---|---|---|---|---|---|
| soup can | 66 mm dia | 0.90 | 0.950 | **yes** | **no — pinch** | no | 6 links, 43.83 N, **0 within 45 mm of axis, 0 below centre** |
| cube | 50 mm | 0.90 | 0.950 | marginal | no closure | no | 0/8 |
| block | 30 mm | 0.90 | 0.915 | **no** | untested | no | 0/10, converged 67–173 mm; elbow 2.059/2.044, wrist 1.565/1.564 |
| block | 30 mm | 1.02 | 1.037 | **no** | untested | no | 0/10, flat 95–118 mm — ~100 mm **above the palm ceiling** |
| block | 30 mm | **0.933** | 0.952 | **yes — 5/10 in <1.3 s** | **no — topples it** | no | 1 TOPPLED at 36.9° during closure; 5 DESCEND stalls 47–152 mm |
| block | 30 mm | riser pad 1.02 | 1.037 | **VOID** | — | — | apparatus defect: pad face blocked the approach |

## 3. Reachable vs graspable: the non-overlap

**There is no object size in this scene that this hand both reaches and cages.**

* **Too large to cage (50–66 mm).** Reachable at ~0.95 m, but both exceed the 21.5 mm
  tip spread, so the hand meets them tip-first. Measured at closure: 43.83 N across 6
  finger links, **zero links within 45 mm of the object's axis and zero below its
  centre**. That is the geometric signature of a pinch. A pinch on a smooth 66 mm
  cylinder has no force closure, which is why every lift attempt failed regardless of
  grip force, friction, thumb opposition, lift vector, lift rate, or grip maintenance —
  each of those was fixed in turn, and none produced a lift.
* **Small enough to cage (~30 mm), but unreachable — or toppled.** A 30 mm object is
  reachable only in a narrow height window near 0.95 m. Below it (0.915 m at the 0.90 m
  counter) the arm pins its elbow and wrist; above it (1.037 m) the object sits above the
  palm ceiling. Inside the window, reach is solved (0.72–1.26 s) — and closure **topples
  the block at 36.9°** instead of caging it.

**Size and surface height are coupled.** A smaller object sits lower on the same surface,
and lower is further from a shoulder already at its limit. Shrinking the object to fit the
hand moves it out of reach. This coupling is why "just use a smaller object" does not work
and why a surface-height change alone does not rescue it.

---

## 4. Three viable next paths

### Path 1 — Parallel / 2-finger gripper sized for 50–66 mm objects
**End-effector swap. Not tuning.**
Targets the demo objects directly (can, cube, mustard, sugar box) at the reach height
that is already proven (~0.95 m).

Requires:
* A parallel-jaw gripper with **stroke ≥ 70 mm** (66 mm can + clearance) and enough
  finger depth to contact **below the object's centre** — the enclosure logger's two
  criteria are the acceptance test.
* URDF/USD for the gripper + a mount at the G1 wrist; mass and inertia stated, not
  guessed (CLAUDE.md: real dimensions or a flagged TODO).
* Rework of the hand-joint layer: `RULE-HAND-NAME` maps Dex5 joints by name; a gripper
  has a different joint set, so `mm5/pipeline.py`'s grip commands need a new backend.
* Re-run the preflight and the enclosure logger unchanged — they are end-effector
  agnostic and are the pass/fail gate.

Fastest path to a real lift on the existing demo objects.

### Path 2 — Pinch-grasp mode for small flat objects at ~0.933 m
**Software. Uses the proven reach.**
The hand demonstrably delivers 43.83 N across 6 links tip-first. That is a *capable
pinch* — it is simply the wrong primitive for a 66 mm cylinder. For thin/flat objects
(≤~30 mm, low profile) a pinch is the correct grasp, not a failure mode.

Requires:
* A **pinch primitive** distinct from the power-grasp close: oppose thumb against
  index/middle at a commanded aperture, approach normal to the object's flat face,
  and close to a **force** target rather than a joint-position target.
* Objects with a graspable feature **≤ 21.5 mm at the contact band** — a thin plate, a
  card, a stem, a handle. Authored primitives at stated sizes are fine; rescaling an
  asset to fit is not.
* Staging near **0.933 m surface / ~0.95 m object centre** — the one height where reach
  is solved (5/10 trials in <1.3 s).
* A fix for the toppling failure: a 40 g block tips at 36.9° on contact, so the pinch
  must close **symmetrically** or brace the object. This is the real open problem in
  this path.

Lowest cost, no hardware, but delivers a pinch of small objects — not a power grasp of
the demo objects.

### Path 3 — Real-G1 grasp data
**Hardware session.**
Answers a question PhysX cannot: whether the real Dex5's **tactile sensing and finger
compliance** change the enclosure story. Simulated rigid links either contact or do not;
real compliant pads deform around an object and can achieve force closure where rigid
tip contact cannot.

Requires:
* Real G1 + Dex5-1P, the same objects, the same ~0.95 m surface height.
* Tactile readout logged per finger link, plus the **same enclosure criteria** applied to
  real contact data — links within 45 mm of the object axis, links below its centre — so
  sim and hardware are compared on one instrument.
* The deploy stack: SONIC is hardware-only (parked at C-39 for the twin), so this session
  doubles as the SONIC-on-hardware check.
* Acceptance: a real lift >5 cm sustained >0.5 s, measured on the same sustained-contact
  gauge, or a measured statement that the real hand pinches too.

This is the only path that can overturn section 3 rather than work around it.

---

## 5. Standalone engineering item — collision-aware planning

**The DLS IK has no obstacle term and will drive the hand through furniture.**

Measured, independently of grasping: a 0.12 m riser pad on the counter put a vertical
face in the approach corridor, and the servo drove the hand into it — palm stalled at
y = 2.02, **45–54 mm short of the pad's front face at y = 2.07, inside its 0.90–1.02 m
z-band, on 9 of 10 trials**. The straight-line servo has no notion that the pad exists.

This is not a grasping problem and does not block the paths above in an empty scene, but
it **must be fixed before any real-robot reach near obstacles** — the same code driving a
real arm into a real counter is a collision, not a stalled trial.

Requires a collision-aware planner — **cuRobo** (GPU, Isaac-native, closest fit) or
**MoveIt** — with the scene's collision geometry published, replacing the straight-line
DLS servo in `mm5/pipeline.py`'s REACH/DESCEND stages. Keep the DLS solver for the final
centimetres where obstacle-free motion is guaranteed.

---

## 6. Instruments — keep all three

| instrument | what it is for |
|---|---|
| `scripts/mm5_preflight.sh` | 4 gauges, **mandatory before any grasp number is quoted**. Non-zero exit means DO NOT TRUST GRASP NUMBERS. |
| **Enclosure logger** | links within N mm of the object axis, and links below its centre. This is what turned "43.83 N of contact" into "pinch, not cage". |
| **IK classifier** | `IK_INFEASIBLE` (names the pinned joints and their limits) vs `SERVO_SLOW` (converging, nothing at a stop). Separates a kinematic limit from a timeout. |

Nine instrument/apparatus defects were caught across this campaign, every one of which
produced a confident wrong number, and none of which were robot defects. The gauges above
are the reason the numbers in section 2 can be trusted. Verify them before believing any
future grasp result.
