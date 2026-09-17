"""Bounded model-based gravity feed-forward for the replay probe's body joints (controller v4 accounting).

The physics engine's generalized gravity force g(q) is the joint effort that HOLDS the current configuration against
gravity (PhysX sign convention: the force to counteract gravity), so it is applied with a positive sign. The implicit
PD drive's contribution is estimated explicitly as kp*(q_target - q) - kd*dq; the feed-forward is reduced per joint
so that the estimated total stays inside the URDF effort limit, and every reduction is reported. Pure NumPy, unit-tested.
"""
import numpy as np


def bounded_gravity_feedforward(kp, kd, q_target, q, dq, gravity_hold, effort_limit, *, ramp=1.0, scale=1.0):
    """Return (ff_applied, estimated_total, pd_estimate, capped_mask) for one set of body joints (all arrays aligned)."""
    kp, kd, q_target, q, dq, g, lim = (np.asarray(x, dtype=np.float64) for x in (kp, kd, q_target, q, dq, gravity_hold, effort_limit))
    if not (0.0 <= ramp <= 1.0 and 0.0 < scale <= 1.0):
        raise ValueError('ramp in [0,1], scale in (0,1]')
    pd_est = kp * (q_target - q) - kd * dq
    total = pd_est + g * scale * ramp
    capped = np.clip(total, -lim, lim)
    ff_applied = capped - pd_est
    capped_mask = np.abs(total) > lim + 1e-9
    if not np.isfinite(ff_applied).all():
        raise ValueError('non-finite feed-forward')
    return ff_applied, capped, pd_est, capped_mask
