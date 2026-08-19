# MM3 — prerequisites surveyed, and one spec-vs-reality conflict

## What exists

| item | state |
|---|---|
| `isaac/twin/lowlevel_bridge/` | **not built** — this gate builds it |
| `unitree_sdk2py` | **not installed** anywhere on this box |
| `unitree_hg` IDL (`LowState_`, `LowCmd_`) | **not present**; the campaign notes the DT bag "decodes only in an image with `unitree_hg`", so it must be baked into the twin bridge image |
| a real `/lowstate` capture to diff against | `ref/captures/g1/captures/g1_twin_gt/g1_twin_gt_0.db3` is the **twin's own** ground-truth bag, not a robot `/lowstate` capture |
| driver-side measurements of the real thing | `ref/panthera-g1-driver/evidence/fw2026q3/` — rate, tick and duplicate analyses |

## The conflict, before any code is written

MM3 test (c) asks for **`rt/lowstate` at 500 Hz ±2 %**. The driver's own measurement
of the real robot says otherwise:

```
=== /lowstate ===
  msgs=21284 span=25.00s  RATE=851.4 Hz
  unique ticks=20522  duplicated-tick msgs=762 (3.6%)
  tick range 12105664..12128744 (span 23080) -> tick rate 923.3/s
  ticks whose duplicate copies DISAGREE on accel: 605
```

**851.4 Hz, not 500** — and 3.6 % of messages repeat a tick while carrying *different*
accelerometer values, so the stream is not a clean 500 Hz clock resampled.

This matters because the twin's whole premise is that it reproduces the robot. A
bridge built to 500 Hz would pass MM3's stated gate and be wrong against the robot;
one built to 851 Hz would fail the stated gate and be right. That is a decision about
which number is authoritative, not something to resolve by picking one quietly —
and it is the same class of error as C-24, where I "fixed" the twin toward a
component the robot does not have.

Note the measurement is of the driver's **ROS** `/lowstate`, which is republished
from the DDS `rt/lowstate`; whether the wire rate equals the republished rate has not
been established here, and the campaign's 500 Hz may be the PD/control rate rather
than the state-publication rate. Both readings are consistent with the text, which is
why it needs a call.

## What the bridge needs regardless

`§4.2`: `G1_CONTROL=lowcmd` drives the articulation only from `rt/lowcmd`; PD computed
by us at 500 Hz physics as `tau = kp(q_d−q) + kd(dq_d−dq) + tau_ff`, clamped to URDF
effort limits; fail-closed damping when no command arrives for 100 ms; and every
D435i/Mid-360/ROS topic still publishing while SONIC drives.

Two of those are already in hand from earlier gates: the effort limits are in the
contract, and `TWIN_CAMERA=0`/C-23 already tells us the camera half of "topics still
publishing" cannot be shown on this box.

---

# ANSWER (measured, 2026-08-19)

## The bag is a robot capture, not the twin's own

The survey above recorded `g1_twin_gt` as "the **twin's** own ground-truth bag".
That was wrong, and the correction matters because it is the whole basis for
using it as the reference:

* the bag carries `/lowstate` typed `unitree_hg/msg/LowState`;
* **no image on this box can produce that type** — that is the finding the survey
  itself opened with, and building the bridge that could is what MM3 *is*.

So the twin cannot have written those 35 998 messages. `g1_twin_gt` is the
ground truth **for** the twin, captured on the robot with the Ferox stack live
(`/ferox/g1_01/clock_offset`, a robot↔cloud sync topic, is in the same bag).
It is a legitimate reference for test (c).

## Measured rate — `scripts/mm3_lowstate_rate.py`, evidence `lowstate_rate_bag.txt`

```
/lowstate (unitree_hg/msg/LowState)
  msgs=35998  span=34.5568 s   RATE = 1041.677 Hz   period = 0.9600 ms
  dt ms: mean=0.9600 std=0.2381 min=0.0106 max=6.4207
  dt ms: p1=0.0681 p25=0.8748 p50=0.9743 p75=1.0255 p99=1.4738
  per-second counts: mean=1041.68 min=1039 max=1044 std=0.79  (n=34 full seconds)
```

The distribution is a clean unimodal 0.96 ms with a thin sub-100 µs tail: 293
gaps (0.81 %) below 50 µs, which are message *pairs* landing inside one cycle —
the bag-side signature of the duplicate-tick behaviour the driver probe measured
directly. Per-second counts vary by ±3 out of ~1042 (σ = 0.79), so the rate is a
hardware clock, not a jittery software timer.

## The 851 vs 1041 discrepancy

Both numbers are real; they are measurements of **different points in the same
pipeline**, and the bag is the upstream one.

| source | topic | rate | notes |
|---|---|---|---|
| DT bag (this measurement) | `/lowstate` | **1041.68 Hz** | σ=0.79 msg/s, 0.81 % burst pairs |
| driver probe `fw2026q3` | `/lowstate` | 851.4 Hz | 3.6 % duplicated ticks, tick-range rate 923.3/s |
| driver probe `fw2026q3` | `/secondary_imu` | 1041.7 Hz | — |

The driver probe's *own* `/secondary_imu` sits at 1041.7 Hz, matching this bag's
`/lowstate` to four significant figures. That points at a shared ~0.96 ms device
clock on the G1, with the driver's 851.4 Hz being that stream **after loss in the
DDS→ROS republish path** — its tick-range rate of 923.3/s falls short of 1041.7
the same way, which is what dropping messages looks like and is not what a
genuinely slower publisher looks like.

### Tested, and the hypothesis was wrong in its mechanism

That check is now run — `scripts/mm3_lowstate_decode.py`, evidence
`lowstate_decode_bag.txt` — and it gives a better answer than the guess:

```
tick range 7570406..7604965  span=34559 over 34.5568 s
tick advance rate = 1000.064 /s   vs message rate 1041.677 Hz
unique ticks 34550 of 35998  -> duplicated-tick msgs 1448 (4.02%)
tick deltas:  +1 = 95.95%   0 = 4.02%   +2 = 0.03%
ticks skipped (delta>1): 10 events, 10 ticks lost
duplicate-tick pairs disagreeing on accel: 1158
```

`tick` is a **millisecond counter**: 1000.064 /s, 0.006 % off nominal 1 kHz. So the
G1's internal state clock is 1000 Hz, and the robot publishes **1041.68 messages
per second against it** — it genuinely re-sends about 4 % of states, which is the
driver probe's 3.6 % measured independently in a second capture. The duplicate-tick
behaviour is real and upstream of the driver.

This kills the "downstream loss" story: the bag loses **10 ticks out of 34 559**
(0.03 %), so it is not a lossy capture. What separates it from the driver capture
is the other way round — the driver capture's tick clock ran at 923.3 /s against a
robot that ticks at 1000.064 /s, i.e. **7.7 % of its robot-time is missing**, which
is what a gappy capture looks like, not what a slower publisher looks like. The
`/secondary_imu` coincidence noted above is just a coincidence and carries no weight.

**Prefer the bag** — now on evidence rather than on instruction: it is within
0.006 % of the robot's own nominal clock, and the driver capture is not.

## The two rates, stated separately

The 500 Hz in `§4.2` conflated a control rate with a publication rate. They are
separate numbers in the contract and neither is 500 Hz-as-published:

| number | value | what it is |
|---|---|---|
| `lowstate_publish_hz` | **1041.68 Hz** (band 1020.84 – 1062.51) | rate the twin publishes `rt/lowstate`, measured from the robot |
| `pd_apply_hz` | **500 Hz** | rate the bridge evaluates `tau = kp(q_d−q) + kd(dq_d−dq) + tau_ff` inside the sim |

**Revised gate (c):** `rt/lowstate` matches the bag's measured rate **1041.68 Hz
±2 %** with the same field semantics — *not* 500 Hz. 500 Hz survives as
`pd_apply_hz` and is gated separately.

### The consequence, recorded rather than hidden

Publishing at 1041.68 Hz off a sim stepping physics at 500 Hz means ~2.08 published
messages per physics step, so consecutive `rt/lowstate` messages will repeat joint
state. That is a real deviation and it is **logged as Class C, not emulated** — the
twin will not synthesise fake tick duplication to imitate the robot's 0.81 %. The
irony is noted: the robot does this too, for its own unrelated reasons.

## Still blocking, unchanged

`unitree_sdk2py`, ROS `unitree_hg` and the bridge itself remain unbuilt; the image
that fixes the first two is `docker/Dockerfile.lowlevel` → `ferox/twin-lowlevel:humble`.
