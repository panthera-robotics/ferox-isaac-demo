# Re-measurement after the lowcmd seqlock fix (decision 4)

The seqlock had three writers and no lock and was losing **73.2 %** of reads
(`evidence/C39/LOWCMD_SEQLOCK.md`). Every torque, PD-tracking and latency number taken
before that fix was measured over a transport that was dropping most of its commands.
This is the re-measurement, same box, same bridge, `mm3_sdk2_stand.py` at 500 Hz.

## Measured now

    lowcmd_sent    23999   lowcmd_hz        499.976     <- sender, unchanged
    lowstate_recv   9923   lowstate_hz      206.530     <- CLIENT receive rate
    hold_samples    9105
    track_err_mean_rad   0.12162
    track_err_max_rad    1.00845   (joint 25, right_elbow)
    roll/pitch_abs_max   8.2e-11   (rig-held base, as expected)
    fail-closed on 3.03 s silence: ENGAGED

Bridge, same window: `lowstate 778–1000 Hz published, lowcmd 22077 received, crc_bad=0`.
Sim side: `torn` **stopped growing** once commands flowed (17368 constant across
samples), where pre-fix it climbed with every PD step.

## Which documented numbers change, and how

| number | as documented | now | verdict |
|---|---|---|---|
| `rt/lowcmd` publication | **499 Hz** | **499.976 Hz** | **stands** — but it was, and still is, counted at the **sender**; it never meant "commands applied to the robot" |
| `rt/lowstate` publication | 1041.68 Hz | 778–1000 Hz published | **stands within jitter**; RTF-dependent |
| `rt/lowstate` **received by a 500 Hz client** | *never measured* | **206.53 Hz** | **NEW, and it is the honest number** — the client sees a fifth of what is published |
| PD tracking | table taken pre-fix | mean **0.1216 rad**, max **1.0085 rad** (right_elbow) | **replaced** — quote these |
| torn-read growth | climbed continuously | **stops once commands flow** | **fixed** |
| fail-closed watchdog | engages within 100 ms | engages on 3 s silence | **stands** |
| policy latency 2.62–3.00 ms | SONIC-side | **not re-measured** | SONIC is parked; leave flagged rather than quote |

## The correction that matters

The pre-fix "499 Hz lowcmd" was never wrong as a *sender* figure — it was wrong as
evidence that the robot was being commanded at 499 Hz. It was not: the sim was discarding
most of those commands at the seqlock, and nothing in the gate measured the far side. The
new `lowstate_recv` row exists for the same reason — publication rate and delivery rate
are different numbers and this campaign had only ever recorded the first.

**Still not re-measured:** MM4's policy latency and LowState-age figures, which are
SONIC-side. SONIC is parked (decision 1), so they stay flagged in `RESULTS_MM4.md` rather
than being quoted or quietly dropped.
