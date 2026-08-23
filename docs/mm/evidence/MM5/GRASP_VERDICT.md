# Grasp — written verdict, 2026-08-23. **Descent is the dominant limit; enclosure is UNTESTED.**

Mohammed set the decision rule before the run: cube lifts → enclosure proven; cube closes
but does not lift → pinch-not-cage; cube does not reach closure → descent is still the
limit. **The third branch is what happened**, and it is recorded as such rather than
reinterpreted.

## The decisive cube run

```
cube_5cm, N=8, counter, mu=1.2, verified gauges
0/8.  DESCEND_TIMEOUT 6 (43, 51, 63, 100, 143, 153 mm) · REACH_TIMEOUT 2 (148, 170 mm)
```

**No trial reached closure.** A cube cannot topple and cannot roll out of a pinch, so it
was the clean way to test enclosure — and the test never ran, because the hand never
arrived. Enclosure therefore remains **unmeasured**, not disproven.

## Why the can could not answer it either

Same session, verified tilt gauge: 2 of 4 can trials ended `TOPPLED` at **30.0°** and
**30.3°** *during the closure ramp*, before the closure gate. The other two timed out in
descent. So the enclosure logger — which fires at the gate — never printed for either
object.

## The corrected picture of the can's topple

I claimed the topple confirmed (from 90.4°, a miscalibrated axis), then retracted it
wholesale when the corrected gauge read 0.0°. **Both were overstatements.** With a
verified instrument the can topples on *some* approaches (30.0°, 30.3°) and not others
(0.0°, 1.1°). The phenomenon is real and intermittent; what was false was the 90° figure
and the confidence in either direction.

## What is established, and it is not nothing

| stage | state |
|---|---|
| REACH | works when the descent geometry is right — 2.4 s on 9/10 at one point |
| DESCEND | **the dominant failure.** Stalls 37–171 mm across both objects, inconsistent across randomised positions |
| GRASP closure | reachable on the can: up to **6 links, 35–91 N**, thumb opposing |
| Grip through LIFT | **fixed and verified** — contacts maintained 1–3 links at 35–91 N under a 0.03 m/s rate limit |
| Object rises | **never.** `d_obj <= +4 mm` |
| Enclosure | **UNTESTED** — closure too rarely reached to measure it |

## The honest read

The failure ordering has been re-established: **descent, not grip, is the binding
constraint.** Force, friction, contact count, thumb opposition, lift vector, lift rate and
grip maintenance are all eliminated by measurement. What remains untested is whether the
hand *cages* the object, and that cannot be measured until the descent reliably delivers
the palm to the grasp pose.

A constant stall said "wrong number" and was fixed (`grasp_standoff` 0.045 → 0.1466). The
present stall is a **spread** (37–171 mm), which says the arm cannot always achieve the
pose — an IK/workspace question, not a parameter.

**Next lever is Mohammed's to choose.** The evidence supports either: (1) measure
per-trial IK residual against joint limits at the grasp pose, or (2) move the object into
a more central part of the workspace and re-test enclosure there. No sub-hypothesis is
opened here.
