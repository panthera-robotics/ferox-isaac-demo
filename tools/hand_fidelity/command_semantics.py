"""Named command conventions for the six Inspire hand actuators and explicit, tested conversions between them.

Every convention is a source contract in the exact shape `isaac.twin.inspire.embodiment.HandCommandAdapter` consumes
(axis_order, open_value, closed_value, optional per_axis_endpoints), plus provenance and a per-axis evidence class.
A conversion goes through the normalized closure c in [0, 1] (0 = the convention's open endpoint, 1 = its closed
endpoint) and is refused when a convention is unknown, a vector has the wrong length, a value is nonfinite, or a
value lies outside the declared range under the 'reject' policy. Nothing is padded, truncated or reinterpreted.

Two cross-model import policies are named because they are different physical assumptions:
  closure_preserving: c is preserved (closed maps to closed) — the only policy applicable to counts and normalized data;
  radian_identity:    the radian value is copied between two radian conventions — assumes identical joint datums and
                      travels, which the Unitree `inspire_hand` URDF (1.7 / 0.5 / [-0.1, 1.3] rad) and the FTP donor
                      (1.4381 / 0.5864 / [0, 1.1641] rad) do NOT share; the discrepancy is reported, never hidden.
"""
from __future__ import annotations

import math

from isaac.twin.inspire.embodiment import ContractError, HAND_ACTUATORS

VERIFIED_SOURCE, INFERRED, UNVERIFIED = 'VERIFIED_SOURCE', 'INFERRED', 'UNVERIFIED'
NATIVE_ORDER = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
CARD_ORDER = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')

# Unitree `inspire_hand` URDF (xr_teleoperate @ 817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b, assets/inspire_hand/inspire_hand_right.urdf,
# sha256 3dc82ee57e8918ce6316b4d0a0572450e719474b437f91dca4fea1253547b8c1, Apache-2.0) joint limits; the same three ranges are the
# hard-coded normalize/denormalize constants of xr_teleoperate robot_hand_inspire.py and unitree_sim_isaaclab dds/inspire_dds.py (@ e30c25b).
UNITREE_INSPIRE_HAND_URDF_LIMITS = {'index': (0.0, 1.7), 'middle': (0.0, 1.7), 'ring': (0.0, 1.7), 'little': (0.0, 1.7), 'thumb_bend': (0.0, 0.5), 'thumb_rotation': (-0.1, 1.3)}
# FTP donor (unitree_ros @ 7d6075f7, g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf sha256 63097d73…): independent joint limits.
FTP_DONOR_LIMITS = {'index': (0.0, 1.4381), 'middle': (0.0, 1.4381), 'ring': (0.0, 1.4381), 'little': (0.0, 1.4381), 'thumb_bend': (0.0, 0.5864), 'thumb_rotation': (0.0, 1.1641)}

CONVENTIONS = {
    'inspire_e2_angle_register_v1': {
        'axis_order': list(NATIVE_ORDER), 'open_value': 1000.0, 'closed_value': 0.0, 'units': 'counts', 'saturation_policy': 'reject',
        'source': 'Inspire RH56E2 user manual V1.0.0 (sha256 add7aab7…) Table 45 registers 1486-1497 (little, ring, middle, index, thumb bending, thumb rotation) and 2.6.12 example: actual angle 1000 = fully open, set 0 -> bends towards the palm',
        'evidence': {'axis_order': VERIFIED_SOURCE, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': INFERRED, 'direction_thumb_rotation': INFERRED, 'linearity_counts_to_angle': UNVERIFIED, 'installed_firmware': UNVERIFIED},
    },
    'inspire_e2_actuator_position_register_v1': {
        'axis_order': list(NATIVE_ORDER), 'open_value': 0.0, 'closed_value': 2000.0, 'units': 'counts', 'saturation_policy': 'reject',
        'source': 'RH56E2 manual Table 44 registers 1474-1485: 0 = minimum actuator stroke = fingers open, 2000 = maximum stroke = fingers bent; manual: not recommended for setting the angle',
        'evidence': {'axis_order': VERIFIED_SOURCE, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': INFERRED, 'direction_thumb_rotation': INFERRED, 'linearity_counts_to_angle': UNVERIFIED, 'installed_firmware': UNVERIFIED},
    },
    'unitree_inspire_dds_normalized_v1': {
        'axis_order': list(NATIVE_ORDER), 'open_value': 1.0, 'closed_value': 0.0, 'units': 'normalized', 'saturation_policy': 'reject',
        'source': 'xr_teleoperate @ 817fb00 robot_hand_inspire.py (sha256 e47d60e5…): normalize = (max - q)/(max - min), FTP command int(val*1000), state angle_act/1000; order table pinky, ring, middle, index, thumb-bend, thumb-rotation (right ids 0-5, left 6-11); unitree_sim_isaaclab @ e30c25b dds/inspire_dds.py denormalize = (1 - norm)*(max - min) + min with the same constants',
        'evidence': {'axis_order': VERIFIED_SOURCE, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': VERIFIED_SOURCE, 'direction_thumb_rotation': VERIFIED_SOURCE, 'recording_pipeline_identity': UNVERIFIED},
    },
    'unitree_inspire_hand_urdf_radians_v1': {
        'axis_order': list(NATIVE_ORDER), 'open_value': 0.0, 'closed_value': 1.7, 'units': 'rad', 'saturation_policy': 'reject',
        'per_axis_endpoints': {a: {'open_value': lo, 'closed_value': hi} for a, (lo, hi) in UNITREE_INSPIRE_HAND_URDF_LIMITS.items()},
        'source': 'Unitree inspire_hand_right.urdf @ 817fb00 (sha256 3dc82ee5…): R_{pinky,ring,middle,index}_proximal_joint [0, 1.7], R_thumb_proximal_pitch_joint [0, 0.5], R_thumb_proximal_yaw_joint [-0.1, 1.3]; open = lower limit per inspire_dds.py denormalize (norm 1.0 -> min)',
        'evidence': {'axis_order': VERIFIED_SOURCE, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': VERIFIED_SOURCE, 'direction_thumb_rotation': VERIFIED_SOURCE, 'datum_identity_with_ftp_donor': UNVERIFIED},
    },
    'ftp_donor_radians_v1': {
        'axis_order': list(NATIVE_ORDER), 'open_value': 0.0, 'closed_value': 1.4381, 'units': 'rad', 'saturation_policy': 'reject',
        'per_axis_endpoints': {a: {'open_value': lo, 'closed_value': hi} for a, (lo, hi) in FTP_DONOR_LIMITS.items()},
        'source': 'manifest g1_edu29_rh56dftp_donor_v1 actuators (donor URDF limits; open = URDF zero pose, bench convention; NOT an E2 angle datum)',
        'evidence': {'axis_order': VERIFIED_SOURCE, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': VERIFIED_SOURCE, 'direction_thumb_rotation': VERIFIED_SOURCE, 'datum_identity_with_e2': UNVERIFIED},
    },
    'g1_wbt_pick_up_drinks_card_v1': {
        'axis_order': list(CARD_ORDER), 'open_value': 1.0, 'closed_value': 0.0, 'units': 'normalized', 'saturation_policy': 'reject',
        'source': 'unitreerobotics/G1_WBT_Inspire_Pick_Up_Drinks_v0 @ 621467e8 README (order index, middle, ring, little, thumb open/close, thumb lateral tilt; "0.0-1.0 open -> close" wording ambiguous); direction 1.0 = open from source normalization + episode-0 timeline + video (sprint H hand-semantics decision record, private campaign evidence)',
        'evidence': {'axis_order': INFERRED, 'direction_fingers': VERIFIED_SOURCE, 'direction_thumb_bend': VERIFIED_SOURCE, 'direction_thumb_rotation': INFERRED, 'thumb_rotation_sense_vs_donor': UNVERIFIED},
    },
}


def convention(name):
    if name not in CONVENTIONS:
        raise ContractError('unknown hand command convention %r (known: %s)' % (name, sorted(CONVENTIONS)))
    return CONVENTIONS[name]


def endpoints(conv, axis):
    per = conv.get('per_axis_endpoints') or {}
    if axis in per:
        return float(per[axis]['open_value']), float(per[axis]['closed_value'])
    return float(conv['open_value']), float(conv['closed_value'])


def _check_vector(values, order, name):
    if not isinstance(values, (list, tuple)) or len(values) != len(order):
        raise ContractError('%s expects exactly %d values in order %s; got %s' % (name, len(order), list(order), None if not isinstance(values, (list, tuple)) else len(values)))
    out = []
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ContractError('%s: nonfinite or non-numeric value %r' % (name, v))
        out.append(float(v))
    return out


def to_closure(name, values, *, tolerance=0.0):
    """Vector in convention `name` (its declared order) -> {actuator: closure in [0, 1]}. Out-of-range values are refused
    (beyond `tolerance`); a value inside the tolerance band is clipped to the endpoint and reported."""
    conv = convention(name)
    vals = _check_vector(values, conv['axis_order'], name)
    closure, clipped = {}, []
    for axis, v in zip(conv['axis_order'], vals):
        o, c = endpoints(conv, axis)
        lo, hi = sorted((o, c))
        if v < lo - tolerance or v > hi + tolerance:
            raise ContractError('%s.%s = %r outside the declared range [%s, %s]' % (name, axis, v, lo, hi))
        if v < lo or v > hi:
            clipped.append(axis); v = min(max(v, lo), hi)
        closure[axis] = (v - o) / (c - o)
    return closure, clipped


def from_closure(name, closure):
    """{actuator: closure} -> vector in convention `name` (its declared order)."""
    conv = convention(name)
    if set(closure) != set(HAND_ACTUATORS):
        raise ContractError('closure must name exactly the six actuators; got %s' % sorted(closure))
    out = []
    for axis in conv['axis_order']:
        c = closure[axis]
        if isinstance(c, bool) or not isinstance(c, (int, float)) or not math.isfinite(c) or not 0.0 <= c <= 1.0:
            raise ContractError('closure for %s must be within [0, 1]; got %r' % (axis, c))
        o, cl = endpoints(conv, axis)
        out.append(o + float(c) * (cl - o))
    return out


def convert(src, dst, values, *, policy='closure_preserving', tolerance=0.0):
    """Convert a six-vector from convention `src` to `dst`. Returns (vector, report). `radian_identity` is allowed only
    between two radian conventions and reports, per axis, how far it departs from the closure-preserving result."""
    if policy not in ('closure_preserving', 'radian_identity'):
        raise ContractError('unknown import policy %r' % policy)
    s, d = convention(src), convention(dst)
    closure, clipped = to_closure(src, values, tolerance=tolerance)
    preserving = from_closure(dst, closure)
    report = {'source': src, 'destination': dst, 'policy': policy, 'closure': closure, 'clipped_axes': clipped,
              'evidence_source': s['evidence'], 'evidence_destination': d['evidence']}
    if policy == 'closure_preserving':
        return preserving, report
    if s['units'] != 'rad' or d['units'] != 'rad':
        raise ContractError('radian_identity needs two radian conventions (got %s -> %s)' % (s['units'], d['units']))
    by_axis = dict(zip(s['axis_order'], _check_vector(values, s['axis_order'], src)))
    out, delta, refused = [], {}, {}
    for axis, pres in zip(d['axis_order'], preserving):
        v = by_axis[axis]; o, c = endpoints(d, axis); lo, hi = sorted((o, c))
        if v < lo or v > hi:
            refused[axis] = 'radian %.4f outside destination range [%s, %s]' % (v, lo, hi)
        out.append(v); delta[axis] = v - pres
    report['radian_identity_minus_closure_preserving_rad'] = delta
    if refused:
        raise ContractError('radian_identity refused: %s' % refused)
    return out, report


def endpoint_discrepancy_table(src, dst):
    """For two radian conventions: the radian difference between radian-identity and closure-preserving imports at the
    open and closed endpoints of the source (what a naive copy of source radians would do to the destination model)."""
    s, d = convention(src), convention(dst)
    if s['units'] != 'rad' or d['units'] != 'rad':
        raise ContractError('endpoint table needs two radian conventions')
    rows = {}
    for axis in HAND_ACTUATORS:
        so, sc = endpoints(s, axis); do, dc = endpoints(d, axis)
        rows[axis] = {'source_open_rad': so, 'source_closed_rad': sc, 'destination_open_rad': do, 'destination_closed_rad': dc,
                      'identity_error_at_source_open_rad': so - do, 'identity_error_at_source_closed_rad': sc - dc,
                      'source_travel_deg': round(math.degrees(abs(sc - so)), 3), 'destination_travel_deg': round(math.degrees(abs(dc - do)), 3)}
    return rows


def adapter_contract(name):
    """The convention as a HandCommandAdapter source contract (order, endpoints, saturation policy, per-axis endpoints)."""
    conv = convention(name)
    out = {'axis_order': list(conv['axis_order']), 'open_value': conv['open_value'], 'closed_value': conv['closed_value'], 'saturation_policy': conv['saturation_policy']}
    if conv.get('per_axis_endpoints'):
        out['per_axis_endpoints'] = {k: dict(v) for k, v in conv['per_axis_endpoints'].items()}
    return out


def unsupported_axes(name):
    """Axes whose direction or order is not VERIFIED_SOURCE in this convention: a qualified route must refuse them."""
    ev = convention(name)['evidence']
    bad = [k for k, v in ev.items() if v != VERIFIED_SOURCE and (k.startswith('direction_') or k == 'axis_order')]
    return bad
