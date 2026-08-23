# TASK 1 verdict — descent is **half wrist-limit infeasible, half non-converging**

Two levers were on the table: (a) classify each `DESCEND_TIMEOUT` as IK-infeasible vs
servo-slow, then (b) move the object to the shoulder-optimal reach. **(a) settled it and
(b) was not run, because (a) refuted its premise.**

## (a) First pass — 15 s window

    SERVO_SLOW x4   residual 51-153 mm   pinned_joints=0

Every joint mid-range, well off its stops. The pose is **reachable**; relocating the
object addresses reach, and reach was not the constraint. That is why (b) was dropped —
it would have changed a variable the classifier had just shown is not binding.

## (a) Second pass — 45 s window, the direct implication of "SERVO_SLOW"

Giving the servo three times as long does **not** fix it (0/10), and the classification
splits:

| class | n | evidence |
|---|---|---|
| `SERVO_SLOW` | 4 | residual **43–70 mm**, `pinned_joints=0`, still not converged at 45 s |
| `IK_INFEASIBLE` | 4 | joint **5** @ 1.578 vs 1.564 · joint **6** @ −1.614 vs −1.564 · joint **6** @ 1.564 · joints **3+5** |

Arm indices 5 and 6 are **`right_wrist_pitch`** and **`right_wrist_yaw`**.

## What that means

**The grasp pose's ORIENTATION is not achievable by this wrist.** In half the trials the
solver drives the wrists into their stops trying to align the palm; in the other half it
stalls 43–70 mm out without pinning anything — consistent with a solver fighting an
orientation term it cannot satisfy while position error remains.

This is *not* the "constant stall" defect that was fixed earlier (`grasp_standoff`
0.045 → 0.1466, a wrong number). It is a genuine kinematic limit of the arm + the
commanded palm orientation at counter height.

## Verdict, in the terms set

**"IK-infeasible at that pose"** for half the trials, and non-convergent for the rest.
Neither is a grip problem. **Enclosure remains UNMEASURED** — closure is still not
reached, so caging-vs-pinching could not be tested for a third time.

## What the next lever should address — Mohammed's call, not opened here

The evidence points at the **orientation constraint**, not position: relax or re-derive
the commanded palm orientation at counter height (the approach axis blends to horizontal
there, which is what loads the wrist), or reposition the base so the wrist sits mid-range
rather than near its stops. Both are stated as options; neither is started.

## Time-box

Task 1 is at its 90-minute box with a written verdict, per the standing rule. The grasp
line stops here and the session moves to the montage.
