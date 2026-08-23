# TASK 1 — **REACHABLE-BUT-PINCHES.** Enclosure measured at last.

2026-08-23. Preflight green (4 gauges + sustained contact) before any number below.

## (a) worked: the wrist limits are cleared

The Task-1 verdict had half of all `DESCEND_TIMEOUT`s pinning `right_wrist_pitch` /
`right_wrist_yaw`. Making the IK null space pull the **wrists** toward the centre of their
own travel removed that:

| | before | after (`ik_wrist_null_gain` 1.5) |
|---|---|---|
| wrist pinned | ~50 % of timeouts | **none** |
| closure reached | rarely | **3 closures in 10 trials** |
| remaining pins | wrists | one elbow (joint 3), one wrist at 1.599 vs 1.564 |

A first pass at gain **4.0** removed the pinning but created a **null-space equilibrium** —
three trials converged to the *same* arm pose with the residual frozen at 59–60 mm. An
identical repeated pose is a fixed point; a spread is a reach limit. Gain 1.5 keeps the
wrists off their stops without out-pulling the reach. **(b) — base repositioning — was
therefore not needed and not run.**

## The enclosure answer, measured

    [enclose] links within 45 mm of the object axis: 0;  BELOW its centre: 0  []
    closed: 6 finger links in contact, 43.83 N total
    NO_GRIP: hand closed, object rose -0.002 m

**Six links in contact at 43.8 N, and not one of them is around the object.** Zero within
45 mm of its axis; zero below its centre. The hand **pinches** the object against its own
fingertips instead of **caging** it.

That is the mechanism behind every failed lever: force (38–91 N), friction (μ 0.5→1.2),
contact count (1→6), thumb opposition, lift vector, lift rate and grip maintenance were
each fixed, and none could lift an object the hand is not around.

## Verdict, in the terms set

**"reachable-but-pinches, needs wrap/aperture work."** Pre-grasp is now reachable and
closure is repeatable; the grasp does not cage. Per instruction the grasp line stops here
and the reel films **walk + nav + camera only, no manipulation**.

## What a wrap would need — stated, not started

The Dex5's closed-finger convergence sits **0.1366 m** from the palm origin with a
**21.5 mm** tip spread, against a 66 mm can and a 50 mm cube. For the fingers to pass the
widest point the palm must approach far enough that the object sits *between* the finger
segments rather than at their tips — an **aperture and approach-pose** question. The
enclosure logger now reports it directly, so the next attempt can be measured rather than
argued.
