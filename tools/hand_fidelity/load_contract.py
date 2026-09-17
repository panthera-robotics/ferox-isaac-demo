"""hand_load_contract_v1 — the shared, versioned load contract for one wrist that both the arm controller lane (gravity
compensation) and the whole-body lane (loaded-body model) consume.

Contract (field names are those of `HAND_LOAD_RECORD.json` bundle v1 already consumed by the WBC lane; this module adds
the checks, not a new format):

  frame                : "<side>_wrist_yaw_link" — the URDF frame of the last arm joint; x along the forearm toward the
                         hand; left and right frames are mirror images across the robot's sagittal plane (y -> -y)
  reference point      : the frame ORIGIN for `com_m`; `inertia_about_com_kg_m2` is about the component's own COM, in
                         the frame's axes (no parallel-axis term is baked in)
  units                : kg, m, kg*m^2; angles are not part of this contract
  per-side records     : records[side][configuration] with configuration in {"open", "closed"}: TWO SAMPLES of a
                         configuration-dependent quantity, not an identified continuous model; the donor value at any
                         other configuration comes from `donor_profile.HandUrdf.mass_properties(q, in_wrist_frame=True)`
  components           : hand (URDF subtree), wrist_adapter (unknown -> null), tool (declared per job -> null here)
  evidence class       : component.provenance in {URDF, MEASURED, NOMINAL, declared}; 'assumed' masses are refused
  double counting      : a consumer that already carries the hand inside its model (mass_accounting =
                         source_inertias_only) must NOT add hand_only again; the contract exposes `consumer_rules`

Checks: schema/frames/units; symmetric positive-definite inertia with the triangle inequalities; parallel-axis and rigid
re-expression round trips reproduce the stored values; totals equal composed components; mirror consistency of the two
sides is REPORTED (the donor is known to be asymmetric — F4), never enforced; unknown stays null.
"""
from __future__ import annotations

import math

import numpy as np

SCHEMA = 'hand_load_contract_v1'
BUNDLE_SCHEMA = 'hand_load_record_bundle_v1'
CONFIGURATIONS = ('open', 'closed')
PROVENANCE_ADMISSIBLE = ('URDF', 'MEASURED', 'NOMINAL', 'declared')


class LoadContractError(ValueError):
    pass


def _inertia(I, what):
    a = np.asarray(I, dtype=float)
    if a.shape != (3, 3) or not np.isfinite(a).all():
        raise LoadContractError('%s: inertia must be a finite 3x3' % what)
    if not np.allclose(a, a.T, atol=1e-12):
        raise LoadContractError('%s: inertia must be symmetric' % what)
    w = np.linalg.eigvalsh(a)
    if w.min() <= 0:
        raise LoadContractError('%s: inertia must be positive definite' % what)
    if not (w[0] + w[1] >= w[2] * (1 - 1e-9)):
        raise LoadContractError('%s: principal moments violate the triangle inequality' % what)
    return a


def parallel_axis(I_com, mass, r):
    """Inertia about a point displaced by r from the COM (same axes)."""
    r = np.asarray(r, float)
    return np.asarray(I_com, float) + mass * ((r @ r) * np.eye(3) - np.outer(r, r))


def reexpress(I, R):
    """Inertia in axes rotated by R (columns = new axes in old coordinates): I' = R^T I R."""
    R = np.asarray(R, float)
    return R.T @ np.asarray(I, float) @ R


def compose(components):
    """Rigid composition of {'mass_kg','com_m','inertia_about_com_kg_m2'} dicts in one frame; None if any is unknown."""
    if any(c is None or c.get('mass_kg') is None or c.get('com_m') is None for c in components):
        return None
    m = sum(float(c['mass_kg']) for c in components)
    com = sum(float(c['mass_kg']) * np.asarray(c['com_m'], float) for c in components) / m
    I = None
    if all(c.get('inertia_about_com_kg_m2') is not None for c in components):
        I = sum(parallel_axis(c['inertia_about_com_kg_m2'], float(c['mass_kg']), np.asarray(c['com_m'], float) - com) for c in components)
    return {'mass_kg': m, 'com_m': com, 'inertia_about_com_kg_m2': I}


def validate_component(c, name, frame):
    if c is None:
        return None
    if c.get('frame') != frame:
        raise LoadContractError('%s: frame %r != %r' % (name, c.get('frame'), frame))
    if c.get('provenance') not in PROVENANCE_ADMISSIBLE:
        raise LoadContractError('%s: provenance %r not admissible (assumed masses are refused; unknown stays null)' % (name, c.get('provenance')))
    m = c.get('mass_kg')
    if m is not None and (isinstance(m, bool) or not isinstance(m, (int, float)) or not math.isfinite(m) or m <= 0):
        raise LoadContractError('%s: mass must be finite and > 0 or null' % name)
    com = c.get('com_m')
    if com is not None:
        a = np.asarray(com, float)
        if a.shape != (3,) or not np.isfinite(a).all():
            raise LoadContractError('%s: com_m must be three finite numbers' % name)
        if m is None:
            raise LoadContractError('%s: a COM without a mass is not a load' % name)
    if c.get('inertia_about_com_kg_m2') is not None:
        if m is None or com is None:
            raise LoadContractError('%s: inertia without mass/COM' % name)
        _inertia(c['inertia_about_com_kg_m2'], name)
    return c


def validate_bundle(bundle, *, tol_rel=1e-6):
    """Full contract validation of a HAND_LOAD_RECORD bundle. Returns a report (never mutates the bundle)."""
    if bundle.get('schema') != BUNDLE_SCHEMA:
        raise LoadContractError('bundle schema must be %s' % BUNDLE_SCHEMA)
    report = {'contract': SCHEMA, 'sides': {}, 'mirror_consistency': {}, 'consumer_rules': {
        'hand_only': 'add to a body model whose hand links are absent or massless (e.g. the bare g1_29dof rubber_hand model minus its 0.170 kg placeholder)',
        'never_add_when': 'the consumer model already carries the donor hand subtree (mass_accounting = source_inertias_only, e.g. the twin and the SIM planner route)',
        'configuration': 'open and closed are two samples; compute the donor load at the actual configuration with HandUrdf.mass_properties(q, in_wrist_frame=True) when the hand pose is known',
        'unknown': 'wrist_adapter / tool null => the total that needs them is null; a null is never zero'}}
    for side in ('right', 'left'):
        recs = bundle['records'][side]
        frame = side + '_wrist_yaw_link'
        for cfg in CONFIGURATIONS:
            rec = recs[cfg]
            if rec['frame'] != frame:
                raise LoadContractError('%s/%s: frame %r' % (side, cfg, rec['frame']))
            comps = {k: validate_component(v, '%s/%s/%s' % (side, cfg, k), frame) for k, v in rec['components'].items()}
            hand = comps['hand']
            if hand is None or hand['mass_kg'] is None:
                raise LoadContractError('%s/%s: hand component required' % (side, cfg))
            # totals must equal composition; unknown totals must be null
            t = rec['totals']
            ho = compose([hand])
            if abs(t['hand_only']['mass_kg'] - ho['mass_kg']) > tol_rel * ho['mass_kg'] or not np.allclose(t['hand_only']['com_m'], ho['com_m'], atol=1e-9):
                raise LoadContractError('%s/%s: hand_only total != hand component' % (side, cfg))
            for key, parts in (('hand_and_adapter', [hand, comps['wrist_adapter']]), ('hand_and_tool', [hand, comps['tool']]), ('hand_adapter_and_tool', [hand, comps['wrist_adapter'], comps['tool']])):
                c = compose(parts)
                if (c is None) != (t[key] is None):
                    raise LoadContractError('%s/%s: total %s must be %s' % (side, cfg, key, 'null (unknown component)' if c is None else 'present'))
            I = _inertia(hand['inertia_about_com_kg_m2'], side + '/' + cfg)
            # parallel-axis round trip: about the frame origin and back
            I_origin = parallel_axis(I, hand['mass_kg'], -np.asarray(hand['com_m']))          # about the frame origin
            d = np.asarray(hand['com_m'], float)
            back = I_origin - hand['mass_kg'] * ((d @ d) * np.eye(3) - np.outer(d, d))         # inverse shift back to the COM
            if not np.allclose(back, I, atol=1e-12):
                raise LoadContractError('%s/%s: parallel-axis round trip failed' % (side, cfg))
            # rigid re-expression round trip and invariants
            th = 0.7; R = np.array([[math.cos(th), -math.sin(th), 0], [math.sin(th), math.cos(th), 0], [0, 0, 1]])
            if not np.allclose(reexpress(reexpress(I, R), R.T), I, atol=1e-12) or not np.allclose(np.linalg.eigvalsh(reexpress(I, R)), np.linalg.eigvalsh(I), atol=1e-12):
                raise LoadContractError('%s/%s: rigid re-expression round trip failed' % (side, cfg))
            report['sides'].setdefault(side, {})[cfg] = {'mass_kg': hand['mass_kg'], 'com_m': list(hand['com_m']), 'principal_kg_m2': np.linalg.eigvalsh(I).round(9).tolist(),
                                                         'first_moment_x_kg_m': round(hand['mass_kg'] * hand['com_m'][0], 6), 'gravity_moment_at_horizontal_extension_N_m': round(9.81 * hand['mass_kg'] * hand['com_m'][0], 4), 'provenance': hand['provenance']}
        # the two samples must be the same rigid body (mass conserved) and differ in COM (fingers moved)
        mo, mc = recs['open']['components']['hand'], recs['closed']['components']['hand']
        if abs(mo['mass_kg'] - mc['mass_kg']) > tol_rel * mo['mass_kg']:
            raise LoadContractError('%s: open/closed samples have different masses' % side)
    # mirror consistency (reported, not enforced): right (x, y, z) vs left (x, -y, z)
    for cfg in CONFIGURATIONS:
        r = bundle['records']['right'][cfg]['components']['hand']; l = bundle['records']['left'][cfg]['components']['hand']
        mirrored = np.asarray(l['com_m'], float) * np.array([1.0, -1.0, 1.0])
        report['mirror_consistency'][cfg] = {'right_com_m': list(r['com_m']), 'left_com_mirrored_m': mirrored.round(6).tolist(), 'deviation_m': (np.asarray(r['com_m']) - mirrored).round(6).tolist(),
                                             'status': 'MIRROR_CONSISTENT' if np.linalg.norm(np.asarray(r['com_m']) - mirrored) < 0.001 else 'ASYMMETRIC (source F4; reported, not corrected)'}
    return report


def load_at_configuration(hand_urdf, q):
    """Donor load at an actual configuration (the continuous model behind the two samples), in the wrist frame."""
    mp = hand_urdf.mass_properties(q, in_wrist_frame=True)
    return {'frame': mp['frame'], 'mass_kg': mp['mass_kg'], 'com_m': mp['com_m'], 'inertia_about_com_kg_m2': mp['inertia_about_com_kg_m2'], 'provenance': 'URDF', 'configuration': mp['configuration_rad']}


def compare_with_samples(bundle, side, load):
    """Where an actual-configuration load sits relative to the open/closed samples (not an interpolation claim)."""
    o = bundle['records'][side]['open']['components']['hand']; c = bundle['records'][side]['closed']['components']['hand']
    x = load['com_m'][0]
    return {'com_x_m': x, 'open_x_m': o['com_m'][0], 'closed_x_m': c['com_m'][0], 'between_samples': min(o['com_m'][0], c['com_m'][0]) - 1e-9 <= x <= max(o['com_m'][0], c['com_m'][0]) + 1e-9,
            'note': 'samples bracket the range for finger closure only; thumb rotation moves the COM off this line'}


def declared_tool_component(*, mass_kg, com_m, frame, provenance='declared', inertia_about_com_kg_m2=None, replaces_hand=False, note=''):
    """A held tool/holder declared per job. `replaces_hand=True` marks a TOTAL hung off the wrist (hand + holder + marker,
    the driver's tool_board `tool.mass_kg` semantics): such a component must never be composed with hand_only again."""
    if isinstance(mass_kg, bool) or not isinstance(mass_kg, (int, float)) or not math.isfinite(mass_kg) or mass_kg <= 0:
        raise LoadContractError('tool mass must be finite and > 0')
    if com_m is None:
        raise LoadContractError('a declared tool needs a COM in the wrist frame; a mass without a COM is not a load (the driver tool_board total carries no COM today)')
    c = {'mass_kg': float(mass_kg), 'com_m': [float(v) for v in com_m], 'inertia_about_com_kg_m2': inertia_about_com_kg_m2, 'provenance': provenance, 'frame': frame, 'replaces_hand': bool(replaces_hand), 'note': note}
    return validate_component(c, 'tool', frame)


def hand_and_tool(hand, tool):
    """Compose hand + tool unless the tool is declared as a wrist TOTAL (then it replaces the hand and hand_only must not be added)."""
    if tool is None:
        return None
    if tool.get('replaces_hand'):
        raise LoadContractError('tool is declared as the wrist TOTAL (hand + holder + marker): it replaces the hand; composing it with hand_only would double count')
    return compose([hand, tool])
