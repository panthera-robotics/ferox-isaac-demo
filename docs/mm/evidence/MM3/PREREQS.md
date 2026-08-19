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
