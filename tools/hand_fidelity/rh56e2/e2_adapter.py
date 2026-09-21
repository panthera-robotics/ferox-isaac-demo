"""Six-axis semantic adapter for the PUBLIC exact-E2 prior hand (g1_edu29_rh56e2_e2prior_v1) and the single source of the
E2 kinematic contract: joint names, endpoints, coupling with child-limit clamps, count mapping, donor->E2 mapping.

Everything numeric here is a PUBLIC PRIOR (renesas-rdk/inspire_rh56e2_hand @ 81bdb56 URDF limits, manual count convention),
never an installed measurement; the per-field exactness labels travel with the contract (EXACT_PUBLIC_SOURCE = read verbatim
from the pinned source, INFERRED = derived by us from the source, DECLARED_POLICY = a twin decision, UNVERIFIED = needs hardware).
Orders: NATIVE (register order) little/ring/middle/index/thumb_bend/thumb_rotation; CONTRACT (donor manifest order)
index/middle/ring/little/thumb_bend/thumb_rotation. Counts: 1000 = fully open, 0 = closed, on all six axes (thumb rotation
1000 = out = URDF yaw 0). Closure c in [0, 1] is the exchange quantity between conventions (closure-preserving); radian identity
between donor and E2 is NOT valid (thumb ranges 1.1641 -> 1.658 and 0.5864 -> 0.62 differ).
"""
from __future__ import annotations

import math

SIDES = ('left', 'right')
NATIVE_ORDER = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
CONTRACT_ORDER = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')
FINGERS = ('index', 'middle', 'ring', 'little')
E2_FINGER_NAME = {'index': 'index', 'middle': 'middle', 'ring': 'ring', 'little': 'pinky'}   # public URDF calls the little finger 'pinky'

# independent (active) joint ranges, public URDF limits; open = lower, closed = upper
E2_LIMITS = {'index': (0.0, 1.4381), 'middle': (0.0, 1.4381), 'ring': (0.0, 1.4381), 'little': (0.0, 1.4381), 'thumb_bend': (0.0, 0.62), 'thumb_rotation': (0.0, 1.658)}
DONOR_LIMITS = {'index': (0.0, 1.4381), 'middle': (0.0, 1.4381), 'ring': (0.0, 1.4381), 'little': (0.0, 1.4381), 'thumb_bend': (0.0, 0.5864), 'thumb_rotation': (0.0, 1.1641)}
# coupled joints: child suffix -> (parent axis, multiplier, offset, child upper limit); the child TARGET is clamped at its own limit
COUPLING = {
    'index_intermediate_joint': ('index', 1.0843, 0.0, 1.476374), 'middle_intermediate_joint': ('middle', 1.0843, 0.0, 1.476374),
    'ring_intermediate_joint': ('ring', 1.0843, 0.0, 1.476374), 'pinky_intermediate_joint': ('little', 1.0843, 0.0, 1.476374),
    'thumb_intermediate_joint': ('thumb_bend', 0.8392, 0.0, 0.5410520681182421), 'thumb_distal_joint': ('thumb_bend', 0.7477272, 0.0, 0.45553093477052004),
}
DRIVE_MODEL = {
    'kind': 'NON_BACKDRIVABLE_POSITION_SOURCE_WITH_BOUNDED_COMPLIANCE', 'label': 'DECLARED_POLICY',
    'statement': 'The real RH56E2 actuator is a non-backdrivable screw: the finger cannot be flexed by hand, holds position across power loss and draws '
                 'no current at rest. The twin models each of the six actuators as a position source (stiff implicit position drive) and leaves the '
                 'structural/contact compliance to the links and the contact solver; it must NOT be made infinitely rigid (drive stiffness and the '
                 'contact offsets are tuned in the physics pass, not here) and the coupled joints follow their parent by the table below.',
    'urdf_limits': {'effort_Nm': 10.0, 'velocity_rad_s': 1.0, 'policy': 'DONOR_DRIVE_LIMITS_CARRIED', 'public_placeholders': 'effort 1 / velocity 1-2 recorded per joint in E2_PRIOR_ASSET_MANIFEST.json', 'installed': 'NOT_MEASURED'},
}


def e2_joint(side, axis):
    """Independent joint name of one contract axis inside the assembled body."""
    if side not in SIDES: raise ValueError('side must be left or right')
    if axis == 'thumb_bend': return side + '_thumb_proximal_pitch_joint'
    if axis == 'thumb_rotation': return side + '_thumb_proximal_yaw_joint'
    if axis in FINGERS: return side + '_' + E2_FINGER_NAME[axis] + '_proximal_joint'
    raise ValueError('unknown axis %r' % axis)


def donor_joint(side, axis):
    """Independent joint name of the same axis on the frozen FTP donor."""
    if axis == 'thumb_bend': return side + '_thumb_2_joint'
    if axis == 'thumb_rotation': return side + '_thumb_1_joint'
    if axis in FINGERS: return side + '_' + axis + '_1_joint'
    raise ValueError('unknown axis %r' % axis)


def independent_joint_names(side, order=CONTRACT_ORDER):
    return [e2_joint(side, a) for a in order]


def coupled_joints(side):
    """{child joint name: {parent joint name, multiplier, offset, limit_rad, clamp_at_child_limit}}"""
    return {side + '_' + child: {'parent': e2_joint(side, parent), 'multiplier': mult, 'offset': off, 'limit_rad': [0.0, upper], 'clamp_at_child_limit': True}
            for child, (parent, mult, off, upper) in COUPLING.items()}


def _check_len(values, n, what):
    values = list(values)
    if len(values) != n: raise ValueError('%s needs %d values, got %d' % (what, n, len(values)))
    for v in values:
        if not math.isfinite(float(v)): raise ValueError('%s contains a non-finite value' % what)
    return values


def reorder(values, from_order, to_order):
    values = _check_len(values, 6, 'axis vector'); d = dict(zip(from_order, values)); return [d[a] for a in to_order]


def counts_to_closure(counts, order=NATIVE_ORDER):
    """0..1000 counts (1000 = open) -> closure c = 1 - count/1000 per axis; out-of-range counts are refused (no clipping)."""
    out = {}
    for a, v in zip(order, _check_len(counts, 6, 'counts')):
        v = float(v)
        if not 0 <= v <= 1000: raise ValueError('%s count %r outside 0..1000' % (a, v))
        out[a] = 1.0 - v / 1000.0
    return out


def closure_to_counts(closure_by_axis, order=NATIVE_ORDER):
    out = []
    for a in order:
        c = float(closure_by_axis[a])
        if not 0 <= c <= 1: raise ValueError('%s closure %r outside 0..1' % (a, c))
        out.append(int(round((1.0 - c) * 1000.0)))
    return out


def closure_to_rad(closure_by_axis, limits=E2_LIMITS):
    return {a: limits[a][0] + float(closure_by_axis[a]) * (limits[a][1] - limits[a][0]) for a in limits}


def rad_to_closure(rad_by_axis, limits=E2_LIMITS, tolerance=1e-9):
    out = {}
    for a, (lo, hi) in limits.items():
        q = float(rad_by_axis[a])
        if q < lo - tolerance or q > hi + tolerance: raise ValueError('%s = %r outside [%s, %s]' % (a, q, lo, hi))
        out[a] = min(max((q - lo) / (hi - lo), 0.0), 1.0)
    return out


def full_hand_targets(active_rad_by_axis, side):
    """The 12 joint targets of one hand from the six independent radians: coupled targets = multiplier * parent, clamped at the child limit."""
    if side not in SIDES: raise ValueError('side must be left or right')
    q = {}
    for a, (lo, hi) in E2_LIMITS.items():
        v = float(active_rad_by_axis[a])
        if v < lo - 1e-9 or v > hi + 1e-9: raise ValueError('%s = %r outside [%s, %s]' % (a, v, lo, hi))
        q[e2_joint(side, a)] = v
    for child, (parent, mult, off, upper) in COUPLING.items():
        q[side + '_' + child] = min(max(mult * float(active_rad_by_axis[parent]) + off, 0.0), upper)
    return q


def counts_to_e2(counts, side, order=NATIVE_ORDER):
    return full_hand_targets(closure_to_rad(counts_to_closure(counts, order)), side)


def e2_to_counts(q_by_joint, side, order=NATIVE_ORDER):
    rad = {a: q_by_joint[e2_joint(side, a)] for a in E2_LIMITS}
    return closure_to_counts(rad_to_closure(rad), order)


def donor_rows_to_e2(q_donor, side, donor_order=CONTRACT_ORDER):
    """Six donor independent radians (donor manifest order) -> the 12 E2 joint targets, closure-preserving through the donor limits."""
    rad = dict(zip(donor_order, _check_len(q_donor, 6, 'donor row'))); c = rad_to_closure(rad, DONOR_LIMITS)
    return full_hand_targets(closure_to_rad(c), side)


def name_map(side):
    m = {donor_joint(side, a): e2_joint(side, a) for a in CONTRACT_ORDER}
    donor_children = {'index': 'index_2_joint', 'middle': 'middle_2_joint', 'ring': 'ring_2_joint', 'little': 'little_2_joint'}
    for a, dc in donor_children.items(): m[side + '_' + dc] = side + '_' + E2_FINGER_NAME[a] + '_intermediate_joint'
    m[side + '_thumb_3_joint'] = side + '_thumb_intermediate_joint'; m[side + '_thumb_4_joint'] = side + '_thumb_distal_joint'
    return m


def contract(asset_manifest=None, audit=None):
    """E2_KINEMATIC_CONTRACT.json content. Optional: the asset manifest (hashes) and the audit JSON (self-collision envelope)."""
    env = {}
    if audit:
        for c in audit.get('checks', []):
            if c['check'] == 'self_collision_envelope': env[c['side']] = {k: v for k, v in c['detail'].items() if k != 'note'}
    return {
        'contract_id': 'E2_KINEMATIC_CONTRACT_v1', 'asset_id': 'g1_edu29_rh56e2_e2prior_v1', 'label': 'PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE',
        'source': {'url': 'https://github.com/renesas-rdk/inspire_rh56e2_hand', 'commit_short': '81bdb56', 'merged_urdf_sha256': (asset_manifest or {}).get('merged_urdf', {}).get('sha256')},
        'orders': {'native_register': list(NATIVE_ORDER), 'contract': list(CONTRACT_ORDER), 'label': 'EXACT_PUBLIC_SOURCE (manual Table 45 register order; donor manifest contract order)'},
        'independent_joints': {s: {a: {'joint': e2_joint(s, a), 'open_rad': E2_LIMITS[a][0], 'closed_rad': E2_LIMITS[a][1], 'limit_rad': list(E2_LIMITS[a]), 'label': 'EXACT_PUBLIC_SOURCE (URDF limit); open = lower = URDF zero pose, closed = upper; NOT an installed datum'} for a in CONTRACT_ORDER} for s in SIDES},
        'coupled_joints': {s: coupled_joints(s) for s in SIDES},
        'coupling_notes': {'child_limits_below_composed_range': {'finger_intermediate': 'upper 1.476374 < 1.0843 x 1.4381 = 1.559332: saturates when the root passes 1.361592 rad', 'thumb_distal': 'upper 0.45553093 < 0.7477272 x 0.62 = 0.463591: saturates when the bend passes 0.609221 rad', 'thumb_intermediate': 'upper 0.54105207 > 0.8392 x 0.62 = 0.520304: never saturates'},
                           'policy': 'DECLARED_POLICY: public limits kept verbatim, root ranges kept, coupled targets clamped at the child limit (no drive pushed against a limit); label of the ratios EXACT_PUBLIC_SOURCE, of the clamp DECLARED_POLICY',
                           'thumb_distal_reference_form': 'the reference URDF chains thumb_4 on thumb_3 (x0.891); the macro (used here) mimics the pitch joint directly with 0.8392 x 0.891 = 0.7477272 — same composed value'},
        'count_mapping': {'closure': 'c = 1 - ANGLE_COUNT/1000 (1000 = fully open, 0 = closed; manual 2.6.11/2.6.12, public driver open reset = 1000; the public rh56e2_register_map.md line "0 (Fully open) to 1000 (Fully closed)" is reversed and NOT followed)',
                          'finger_proximal_rad': 'c * 1.4381', 'finger_intermediate_rad': 'min(1.0843 * proximal, 1.476374)', 'thumb_bend_rad': 'c * 0.62', 'thumb_intermediate_rad': 'min(0.8392 * bend, 0.54105207)', 'thumb_distal_rad': 'min(0.7477272 * bend, 0.45553093)', 'thumb_rotation_rad': 'c * 1.658 (count 1000 = out = URDF yaw 0; INFERRED direction, hardware-unverified)',
                          'labels': {'direction_fingers': 'EXACT_PUBLIC_SOURCE (manual)', 'direction_thumb_bend': 'INFERRED', 'direction_thumb_rotation': 'INFERRED', 'linearity_counts_to_rad': 'UNVERIFIED (public prior; installed native-to-radian map = hand-req-Q01 conversion template)'}},
        'donor_to_e2': {'name_map': {s: name_map(s) for s in SIDES}, 'policy': 'closure_preserving through the donor limits (index/middle/ring/little identical ranges; thumb_bend 0.5864 -> 0.62; thumb_rotation 1.1641 -> 1.658); radian_identity is NOT admissible for the thumb axes',
                        'function': 'tools.hand_fidelity.rh56e2.e2_adapter.donor_rows_to_e2(q_donor[6], side, donor_order=CONTRACT_ORDER) -> {12 joint: rad}'},
        'drive_model': DRIVE_MODEL,
        'chirality': {'rule': 'same joint semantics on both sides (positive = closing on every joint, positive yaw = thumb towards opposition); left links/joints carry the left_ prefix; the left hand is the y-mirror of the right in the hand-base frame', 'label': 'EXACT_PUBLIC_SOURCE (URDF), verified by the static audit (handedness triple product)'},
        'self_collision_envelope': env or 'see E2_PRIOR_ASSET_AUDIT (self_collision_envelope)',
        'hand_base': {s: {'link': s + '_base', 'parent': s + '_wrist_yaw_link', 'frame': 'fingers +z, across +y (index side), palm normal +x', 'mount': 'donor flange (0.0415, 0, 0; rpy 0, +-pi/2, 0) composed with R_z(+90 deg); INFERRED from the donor mount, not measured'} for s in SIDES},
        'tactile': {'frames_per_hand': 17, 'taxels_per_hand': 1062, 'label': 'TACTILE_LAYOUT_PUBLIC_PRIOR', 'file': 'E2_TACTILE_FRAMES.json'},
    }
