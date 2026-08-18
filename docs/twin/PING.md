# PING — DT2, decision needed: the sim stands with a different waist pose than the robot

**Raised:** 2026-08-18T03:55Z · **Gate:** DT2 · **Rule:** §5 ping (decision needed) and the
listed fallback *"waist round-trip beyond tolerance → publish static composite edge + PING"*.

Everything else in DT2 is green. This is one geometric fork I should not pick for you.

## What is wrong

`tools/twin_geometry_check.py`, run against the live twin:

```
1. base_link -> livox_frame vs the driver's standing composite
   got  xyz=(0, 0, +0.499500)  rpy=(+3.090233, +0.161680, +0.000000)
   want xyz=(0, 0, +0.499500)  rpy=(+3.090233, +0.161680, +0.000000)
   => PASS      <-- but see "why this check is tautological" below

2. floor plane in base_link vs +z
   normal (+0.09685, +0.03729, +0.99460)  tilt 5.9568 deg  (tol 0.5 deg)
   mean |residual| 1.72 mm      floor height in base_link -0.5461 m
   => FAIL

3. /scan geometry: 723 rays in base_link, range [0.30, 6.00], increment 0.0087,
   100.0% of finite returns in band
   => PASS
```

The floor fit is *excellent* — 1.72 mm mean residual over 1884 points. The plane is real and
flat. It is simply tilted by 5.96° relative to where TF says it should be.

## Why

Two poses that agree on the robot do not agree in the sim:

| | roll | pitch | yaw |
|---|---|---|---|
| `torso_link → livox_frame` — the calibrated, waist-independent mount | 3.128688 | 0.052979 | 0.018520 |
| `base_link → livox_frame` — the driver's **standing** composite | 3.090233 | 0.161680 | 0.0 |
| difference (≈ the robot's waist attitude when standing) | −0.038 | **+0.109** | −0.019 |

0.109 rad ≈ **6.2°**, which is the 5.96° the floor fit measures.

The USD mounts `livox_frame` under `torso_link` at the calibrated mount — physically right, and
it follows the waist. But **the sim's policy stands with all three waist joints at 0**
(`deploy.yaml` `default_joint_pos` is 0.0 at the waist indices), whereas the real G1 stands with
its torso pitched ~6.2° forward. So the cloud is generated at the mount attitude while TF
announces the composite, and every consumer that transforms the cloud is 6° out.

**Why check 1 is tautological as written:** the twin publishes the contract's `tf_static` values
directly, so comparing the published edge to the driver constant compares the contract with
itself. It cannot fail. The floor-plane check is the one with teeth, which is why it is in the
harness. I have left check 1 in place but it should be read as "the twin publishes what the
contract says", not "the geometry is right".

## The fork — your call

**A. Publish `base_link → livox_frame` dynamically**, composed from the live USD transform
(this is what the driver's `waist_tf_bridge` does, and it is the driver's *default* mode).
The cloud and TF become self-consistent, the floor plane passes, and at the robot's standing
waist the value equals the driver composite. Cost: the edge moves from `/tf_static` to `/tf`,
so the static edge set drops to 6 and `twin_audit` reports a Class-A structural difference
against Session A — which captured the driver in **static** mode.

**B. Keep the static composite (current behaviour).** Class-A parity with Session A's captured
`/tf_static` is exact, and the interface is string-perfect. Cost: sim lidar data is 6° out
relative to its own TF, so anything that reprojects the cloud — SLAM's scan match, any
depth-to-map transform, anything trained on transformed points — inherits a rotation the robot
does not have. Transfer is exactly what this campaign exists to protect.

**C. Re-author the USD mount so the composite is exact at the sim's standing pose**
(mount := `(pelvis→torso)⁻¹ ∘ composite`). Floor plane passes *and* the static edge stays
Class-A. Cost: the USD no longer carries the calibrated waist-independent mount, so if the sim's
posture ever changes — or DT8 drives the waist — the error returns silently. It also means the
sim body and the robot disagree about where the sensor is bolted, which is the sort of quiet
divergence rule 9 exists to prevent.

**My recommendation: A.** The driver's default mode *is* the waist bridge; static mode is its
fallback. Reproducing the default is truer parity than reproducing one capture of the fallback,
and it is the only option where the sim's own data is self-consistent. The audit difference is
honest and describable: "the twin runs the driver's waist-bridge mode; Session A captured static
mode". If you prefer B, the 6° needs to become a loud Class-C entry, not a footnote.

Related, and not the same thing: **C-7** (the sim USD's `pelvis→torso_link` is 10 mm shorter than
the URDF) is a translation error and stays regardless of which option you pick.

## State

DT2 is otherwise complete and pushed. `twin_audit`: **84 pass, 0 Class-A FAIL**, 5 Class-B
(rates, see C-10). Not yet done, blocked behind this decision only in the sense that the
evidence should be captured once: 3 nav goals, 4 visual PNGs, `validate_motion` re-run,
`ferox_vision`, `RESULTS_DT2.md`, tag.

Per §5 I am stopping here rather than picking a geometry for you.
