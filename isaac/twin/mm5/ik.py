"""Damped-least-squares differential IK on the G1's arm chain.

CHOICE OF SOLVER, since CAMPAIGN 4.4 asks for it to be documented. Isaac ships
Lula (`isaacsim.robot_motion`) and cuRobo is available, and both are better tools for
a full motion plan. This uses neither, deliberately:

  * both want a robot description (Lula) or a collision world (cuRobo) authored for
    THIS articulation, and this articulation is the twin's own 69-DoF composed asset
    with Dex5 hands merged in -- not a stock G1 either of them ships a description for;
  * the reach here is short and unobstructed (a pre-grasp pose 0.15 m off an object on
    an open table), so a plan buys nothing a servo loop does not already give;
  * the Jacobian is already exposed by the articulation view, verified shape
    (1, 79, 6, 75) on this asset, so DLS is ~40 lines with no new dependency and no new
    description file to keep in sync with the asset.

If MM5 later needs to reach into the shelf or around the door, that is the point to
bring in cuRobo -- and this module's interface is the same shape either way.

Damping matters: near a singular arm configuration the pseudo-inverse blows up and a
differential solver answers a millimetre of position error with radians of joint
motion. lambda^2 on the diagonal bounds that, at the cost of slowing convergence near
singularities, which is the right trade for a robot that is balancing at the time.
"""

from __future__ import annotations

import numpy as np


def dls_step(J: np.ndarray, err: np.ndarray, lam: float = 0.08,
             max_step: float = 0.06, q: np.ndarray | None = None,
             q_nominal: np.ndarray | None = None, k_null: float = 0.6) -> np.ndarray:
    """One damped-least-squares increment.

    J    : (6, n) task Jacobian, rows [vx vy vz wx wy wz]
    err  : (6,)   [position error (m), orientation error (rad, axis-angle)]
    lam  : damping. 0.08 is chosen so the solver stays civil through the wrist
           singularity the G1 hits when the forearm straightens.
    max_step: per-iteration joint clamp (rad), so a large task error cannot become a
           large joint jump on a balancing robot.
    """
    JT = J.T
    n = J.shape[0]
    A = J @ JT + (lam ** 2) * np.eye(n)
    Jpinv = JT @ np.linalg.inv(A)
    dq = Jpinv @ err

    # NULL-SPACE POSTURE BIAS. The arm is 7-DoF against a 6-DoF task, so there is a
    # one-dimensional family of joint motions that does not move the palm at all --
    # and without using it the solver walked right_shoulder_yaw into its -2.618 stop
    # and sat there while the task error oscillated 370..435 mm, unable to escape a
    # configuration it had chosen. Projecting a pull toward the nominal posture through
    # (I - J+J) drifts the arm off its limits without disturbing the reach.
    if q is not None and q_nominal is not None:
        null = np.eye(dq.shape[0]) - Jpinv @ J
        dq = dq + null @ (k_null * (np.asarray(q_nominal, float) - np.asarray(q, float)))

    m = float(np.abs(dq).max())
    if m > max_step:
        dq *= max_step / m
    return dq


def pose_error(cur_pos: np.ndarray, cur_quat_wxyz: np.ndarray,
               tgt_pos: np.ndarray, tgt_quat_wxyz: np.ndarray | None) -> np.ndarray:
    """6-vector task error. Orientation term is zero when no target is given."""
    err = np.zeros(6)
    err[:3] = np.asarray(tgt_pos, float) - np.asarray(cur_pos, float)
    if tgt_quat_wxyz is None:
        return err
    q_c = np.asarray(cur_quat_wxyz, float)
    q_t = np.asarray(tgt_quat_wxyz, float)
    if float(q_c @ q_t) < 0.0:
        q_t = -q_t
    # q_e = q_t * conj(q_c); its vector part is half the rotation axis-angle for small
    # angles, which is all a differential step needs.
    cw, cx, cy, cz = q_c
    tw, tx, ty, tz = q_t
    ew = tw * cw + tx * cx + ty * cy + tz * cz
    ex = -tw * cx + tx * cw - ty * cz + tz * cy
    ey = -tw * cy + tx * cz + ty * cw - tz * cx
    ez = -tw * cz - tx * cy + ty * cx + tz * cw
    v = np.array([ex, ey, ez])
    n = float(np.linalg.norm(v))
    if n > 1e-9:
        err[3:] = 2.0 * np.arctan2(n, abs(ew)) * (v / n) * (1.0 if ew >= 0 else -1.0)
    return err
