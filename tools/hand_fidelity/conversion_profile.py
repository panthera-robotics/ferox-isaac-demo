"""Opt-in typed conversion profile: raw six-axis hand values of a declared source convention -> effective donor joint
targets, with the full raw-to-effective trace, per-axis evidence class, bound kinds and dependency hashes.

This module does not modify the twin's active adapter; the closure arithmetic is `HandCommandAdapter` (reused). What it
adds is what a qualified caller needs to fail closed and what a reviewer needs to read a command back:

* value TYPES are distinct: 'radians' (a joint coordinate of a named model), 'closure' (0 = open endpoint, 1 = closed
  endpoint of a named convention), 'counts' (native registers), 'physical_deg' (an angle defined by the manufacturer's
  datum, NOMINAL); a value never changes type implicitly;
* every bound carries a KIND: 'coordinate_endpoint' (a limit of a model's coordinate — not a verified physical stop),
  'physical_stop' (only when measured), 'operating_margin' (a declared diagnostic margin kept inside the endpoint);
* every axis carries an EVIDENCE class for direction/order/scale (VERIFIED_SOURCE / INFERRED / UNRESOLVED); a profile
  built with require_verified=True refuses any axis whose required semantics are not VERIFIED_SOURCE;
* refusals are explicit: non-finite, wrong dimension, wrong side, wrong declared order, out-of-range (unless the profile
  is EXPLORATORY and declares clipping, in which case the clip is recorded as an intervention), unknown provenance.
"""
from __future__ import annotations

import hashlib
import json
import math

from isaac.twin.inspire.embodiment import ContractError, EmbodimentManifest, HAND_ACTUATORS, HandCommandAdapter, canonical_sha256

from . import command_semantics as cs
from .coupling import coupling_table, read_joints

VALUE_TYPES = ('radians', 'closure', 'counts', 'physical_deg')
BOUND_KINDS = ('coordinate_endpoint', 'physical_stop', 'operating_margin')
POLICIES = ('closure_preserving', 'radian_identity')
UNITS_TO_TYPE = {'rad': 'radians', 'normalized': 'closure', 'counts': 'counts', 'deg': 'physical_deg'}


class ConversionProfile:
    """A pinned, hashed description of ONE conversion route for ONE side."""

    def __init__(self, *, profile_id, side, source_convention, manifest: EmbodimentManifest, policy='closure_preserving', operating_margin_rad=0.02,
                 margin_ends=('open',), exploratory=True, declared_clip_tolerance=0.0, require_verified=False, provenance=None, urdf_sha256=None):
        if side not in ('left', 'right'):
            raise ContractError('side must be left or right')
        if policy not in POLICIES:
            raise ContractError('policy must be one of %s' % (POLICIES,))
        conv = cs.convention(source_convention) if isinstance(source_convention, str) else dict(source_convention)
        self.source_name = source_convention if isinstance(source_convention, str) else conv.get('name', 'custom')
        self.conv = conv
        self.source_type = UNITS_TO_TYPE.get(conv.get('units'))
        if self.source_type not in VALUE_TYPES:
            raise ContractError('source convention must declare units rad/normalized/counts/deg')
        if policy == 'radian_identity' and self.source_type != 'radians':
            raise ContractError('radian_identity is only defined for a radians source')
        self.side, self.policy, self.manifest, self.exploratory = side, policy, manifest, bool(exploratory)
        if not (isinstance(operating_margin_rad, (int, float)) and math.isfinite(operating_margin_rad) and 0.0 <= operating_margin_rad < 0.2):
            raise ContractError('operating margin must be a finite value in [0, 0.2) rad')
        self.margin, self.margin_ends = float(operating_margin_rad), tuple(margin_ends)
        if any(e not in ('open', 'closed') for e in self.margin_ends):
            raise ContractError('margin_ends must be open and/or closed')
        if declared_clip_tolerance and not self.exploratory:
            raise ContractError('a declared clip is only admissible on an EXPLORATORY profile')
        self.clip_tol = float(declared_clip_tolerance)
        self.require_verified = bool(require_verified)
        self.provenance = dict(provenance or {})
        for k in ('source_model', 'source_datum', 'target_model', 'target_datum'):
            if k not in self.provenance:
                raise ContractError('provenance must name %s' % k)
        self.urdf_sha256 = urdf_sha256
        # target contract (what HandCommandAdapter maps onto): closure over the donor endpoints, or identity over them
        if policy == 'closure_preserving':
            contract = cs.adapter_contract(source_convention) if isinstance(source_convention, str) else {k: conv[k] for k in ('axis_order', 'open_value', 'closed_value')} | {'saturation_policy': 'reject'} | ({'per_axis_endpoints': conv['per_axis_endpoints']} if conv.get('per_axis_endpoints') else {})
        else:
            contract = {'axis_order': list(conv['axis_order']), 'open_value': 0.0, 'closed_value': 1.4381, 'saturation_policy': 'reject',
                        'per_axis_endpoints': {a: {'open_value': manifest.hand_actuator(side, a)['open_rad'], 'closed_value': manifest.hand_actuator(side, a)['closed_rad']} for a in HAND_ACTUATORS}}
        contract['saturation_policy'] = 'clip_declared' if self.clip_tol > 0 else 'reject'
        contract['endpoint_tolerance'] = self.clip_tol
        self.adapter = HandCommandAdapter(manifest, side, contract)
        self.axis_order = tuple(conv['axis_order'])
        ev = conv.get('evidence', {})
        self.evidence = {a: {'direction': ev.get('direction_' + ('fingers' if a in ('index', 'middle', 'ring', 'little') else a), 'UNRESOLVED'), 'order': ev.get('axis_order', 'UNRESOLVED'),
                             'scale': 'UNRESOLVED' if policy == 'radian_identity' or ev.get('datum_identity_with_ftp_donor') == 'UNVERIFIED' or ev.get('linearity_counts_to_angle') == 'UNVERIFIED' else ev.get('axis_order', 'UNRESOLVED')}
                         for a in self.axis_order}
        for a, e in self.evidence.items():
            for k, v in e.items():
                if v == 'UNVERIFIED':
                    e[k] = 'UNRESOLVED'
        if self.require_verified:
            bad = {a: {k: v for k, v in e.items() if v != cs.VERIFIED_SOURCE} for a, e in self.evidence.items()}
            bad = {a: v for a, v in bad.items() if v}
            if bad:
                raise ContractError('qualified profile refused: unresolved semantics %s' % json.dumps(bad, sort_keys=True))
        self.bounds = {}
        for a in self.axis_order:
            spec = manifest.hand_actuator(side, a); lo, hi = spec['limit_rad']
            o, c = cs.endpoints(conv, a)
            self.bounds[a] = {'source_open': o, 'source_closed': c, 'source_bound_kind': 'coordinate_endpoint',
                              'target_limit_rad': [lo, hi], 'target_bound_kind': 'coordinate_endpoint (donor URDF limit; not a verified physical stop)',
                              'effective_min_rad': lo + (self.margin if 'open' in self.margin_ends else 0.0), 'effective_max_rad': hi - (self.margin if 'closed' in self.margin_ends else 0.0),
                              'margin_kind': 'operating_margin (declared diagnostic; %s end%s)' % ('/'.join(self.margin_ends), 's' if len(self.margin_ends) > 1 else '')}
        self.profile_id = profile_id
        self.sha256 = canonical_sha256({'profile_id': profile_id, 'side': side, 'source': self.source_name, 'source_units': conv.get('units'), 'axis_order': self.axis_order, 'policy': policy,
                                        'margin': self.margin, 'margin_ends': self.margin_ends, 'exploratory': self.exploratory, 'clip_tolerance': self.clip_tol,
                                        'require_verified': self.require_verified, 'contract': self.adapter.contract_sha256, 'manifest': manifest.sha256, 'urdf_sha256': urdf_sha256, 'provenance': self.provenance})

    # ---- conversion ------------------------------------------------------------------------------------------
    def apply(self, raw_values, *, side, declared_order=None, declared_type=None):
        """Raw six-vector -> trace with the effective donor targets, or a ContractError naming the refusal."""
        if side != self.side:
            raise ContractError('profile %s is bound to the %s hand; refusing %s values' % (self.profile_id, self.side, side))
        if declared_order is not None and tuple(declared_order) != self.axis_order:
            raise ContractError('declared order %s differs from the profile order %s' % (list(declared_order), list(self.axis_order)))
        if declared_type is not None and declared_type != self.source_type:
            raise ContractError('declared value type %r differs from the profile source type %r' % (declared_type, self.source_type))
        if not isinstance(raw_values, (list, tuple)) or len(raw_values) != len(self.axis_order):
            raise ContractError('expected %d values in order %s' % (len(self.axis_order), list(self.axis_order)))
        for a, v in zip(self.axis_order, raw_values):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ContractError('%s: non-finite or non-numeric raw value %r' % (a, v))
        targets, info = self.adapter.to_joint_targets(list(raw_values))
        # the adapter snaps values inside its endpoint tolerance without flagging them; this trace reports EVERY out-of-range raw value
        outside = [a for a, v in zip(self.axis_order, raw_values) if not (min(self.adapter.endpoints(a)) <= v <= max(self.adapter.endpoints(a)))]   # contract range: source endpoints (closure) or donor endpoints (identity)
        trace = {'profile_id': self.profile_id, 'profile_sha256': self.sha256, 'side': side, 'policy': self.policy, 'raw': [float(v) for v in raw_values], 'raw_type': self.source_type, 'axis_order': list(self.axis_order),
                 'closure': info['closure'], 'clipped_axes': outside, 'interventions': [], 'target_rad_unmargined': {}, 'margin_applied': {}, 'effective_rad': {}, 'bounds': self.bounds, 'evidence': self.evidence}
        if outside:
            trace['interventions'].append({'kind': 'declared_clip', 'axes': list(outside), 'raw': {a: float(v) for a, v in zip(self.axis_order, raw_values) if a in outside}, 'tolerance': self.clip_tol, 'exploratory_only': True,
                                           'note': 'raw value outside the declared source range; snapped to the nearest endpoint by the declared clip — never a verified mapping'})
        for a in self.axis_order:
            joint = self.manifest.hand_actuator(side, a)['joint']; q = targets[joint]; b = self.bounds[a]
            eff = min(max(q, b['effective_min_rad']), b['effective_max_rad'])
            trace['target_rad_unmargined'][joint] = q; trace['effective_rad'][joint] = eff; trace['margin_applied'][joint] = round(eff - q, 9)
        return trace

    def coupled_consistency(self, trace, urdf_path):
        """Composed coupled-joint values implied by the effective targets, checked against their own limits (should all be inside with the open margin)."""
        table = coupling_table(read_joints(urdf_path), self.side + '_')
        out = {}
        for name, row in table.items():
            q = trace['effective_rad'].get(row['driver'])
            if q is None:
                continue
            v = row['composed_multiplier'] * q + row['composed_offset']; lo, hi = row['own_limit_rad']
            out[name] = {'implied_rad': round(v, 6), 'own_limit_rad': [lo, hi], 'inside': lo < v < hi, 'margin_to_lower_rad': round(v - lo, 6)}
        return out

    def dependencies(self):
        return {'profile_sha256': self.sha256, 'manifest_sha256': self.manifest.sha256, 'urdf_sha256': self.urdf_sha256, 'source_contract_sha256': self.adapter.contract_sha256,
                'policy': self.policy, 'operating_margin_rad': self.margin, 'exploratory': self.exploratory,
                'invalidates_when_changed': ['any of the above; a changed profile_sha256 marks every replay package built with the old profile as a different route (its package hashes differ) and stales qualifications bound to source_contract_sha256']}

    def describe(self):
        return {'profile_id': self.profile_id, 'sha256': self.sha256, 'side': self.side, 'source_convention': self.source_name, 'source_type': self.source_type, 'policy': self.policy, 'exploratory': self.exploratory,
                'operating_margin_rad': self.margin, 'margin_ends': list(self.margin_ends), 'declared_clip_tolerance': self.clip_tol, 'require_verified': self.require_verified,
                'axis_order': list(self.axis_order), 'evidence': self.evidence, 'bounds': self.bounds, 'provenance': self.provenance, 'dependencies': self.dependencies()}


PISTON_ROUTE_PROVENANCE = {
    'source_model': 'Unitree inspire_hand URDF joint space (xr_teleoperate 817fb00, sha256 3dc82ee5…; same limits as unitree_sim_isaaclab e30c25b dds/inspire_dds.py denormalize constants)',
    'source_datum': 'lower limit = open (fingers 0, thumb pitch 0, thumb yaw -0.1 rad); dataset asset identity INFERRED from feature names thumb_pitch/thumb_yaw and the constant -0.1',
    'capture_asset': 'birbirll/g1-inspire-piston-pick-place (Isaac Lab replay of piston CSVs; asset revision not pinned in the dataset card)',
    'modality_code': 'GR00T n1d6 statistics.json new_embodiment min/max normalization; right_hand action range [0, 1.3] fingers, thumb columns constant (0, -0.1) — constant columns cannot identify their scale',
    'target_model': 'FTP donor g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf sha256 63097d73…', 'target_datum': 'URDF zero = open (bench convention); limits 1.4381 / 0.5864 / 1.1641 rad are coordinate endpoints, not verified physical stops',
}
