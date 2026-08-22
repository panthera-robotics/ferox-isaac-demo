# The twin has been dropping `rt/lowcmd`. A seqlock with three writers and no lock.

Found 2026-08-22 while chasing "rig released at: NEVER". This is a **transport defect in
our own bridge**, and it sits underneath an unknown amount of this campaign's C-39
evidence.

## The contradiction that led here

SONIC, running clean, with the velocity guard disabled:

```
bridge : lowstate=100196 (364 Hz)  lowcmd=63857 (232 Hz)  crc_bad=0  nan_cmd=0
sonic  : G1 type: 5 / Init Done / zero aborts for the whole run
sim    : mode=hold  x141 samples, t 0.50 .. 70.50      rig released at: NEVER
```

An external probe attached to the same named segment while all of that was live:

    cmd_count=73575  age_ms=6.95  kp0=99.1  qd0=-0.1211      <- perfect data
    ...and 2 of its 3 reads returned None

And the sim's own instrumentation said it had **never completed a single read**:

    cmd_age=-1.0ms  auth=-1.0s  rig_ign=0  fix_base=1

`cmd_age=-1.0` is the sentinel for `_last_cmd_rec is None`. `rig_ign=0` means the rig
never once saw a fresh command to ignore. So `rt/lowcmd` was arriving at 232 Hz, being
written correctly, and **never reaching the robot**.

## The mechanism

A seqlock has **exactly one writer by construction**: `seq++` (odd, write in flight),
mutate, `seq++` (even, whole). Readers retry while `seq` is odd.

The command channel has **three** writers — `_on_lowcmd`, and `_on_handcmd` for each
hand — and Cyclone DDS delivers them on **separate listener threads**. Their `seq++`
pairs interleave, the counter is left **odd at rest**, and every reader spins out its
eight retries and returns `None` — forever, over a record whose contents are fine.

The `lowstate` channel has one writer (the sim) and works flawlessly. That asymmetry is
precisely what disguised this: the same shared-memory code, the same `--ipc host`, the
same segment names, working perfectly in one direction.

## Measured, both ways

Same channel, three writer threads, two seconds, `read()` in a tight loop:

| | reads OK | reads `None` | lost |
|---|---|---|---|
| **no lock** (the code as it was) | 12 266 | 33 544 | **73.2 %** |
| **with the lock** | 216 455 | 0 | **0 %** |

73 % matches the 2-in-3 the live probe saw. The sim's loss ran at effectively 100 %,
which a tighter retry loop against three busier writers will do.

## What this retro-explains

`sim_side.py` already carried this note, written before the cause was known:

> *"treating a collision as silence made fail-closed flap on and off every 50–300 ms
> through a stand that was being commanded the whole time"*

That is this bug. The `_last_cmd_rec` cache was added to paper over it and mostly
succeeded: at ~27 % successful reads the PD path stays alive, so runs looked like they
worked. What the cache **cannot** paper over is any condition requiring an *unbroken run*
of successes — and the rig's auto-release is exactly that (`_commanded_since_ns` resets
on a single false). So the release is the thing this breaks hardest, and it is why
`until_commanded` never fired.

## Scope — how much C-39 evidence is affected

Honestly: **unknown, and it must not be guessed at.** Any earlier run in which SONIC
"drove the twin" was driving it through a channel losing a large fraction of its
commands, at a loss rate that varies with how hard the hand callbacks are firing. That
does not automatically invalidate those runs — a PD holding stale targets at 27 %
refresh is still being commanded — but it does mean **no torque, tracking or latency
number taken before this fix should be quoted without re-measuring it.**

The three C-39 conclusions that do **not** depend on the transport still stand: the
`m·g·h` arithmetic (`CORRECTION.md` §4), the per-link mass diff (`VERDICT.md`), and the
reference-vs-twin asset import.

## The fix

`SeqlockChannel.write()` now serialises on a `threading.Lock` held in the channel, so
any multi-writer use is safe rather than only the two call sites known today. It is
per-process and uncontended for the single-writer `lowstate` side, which pays nothing.
