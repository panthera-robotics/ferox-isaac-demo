"""ONE six-axis hand action contract for both consumers of the hand (sprint M, hand lane).

The scripted route (ReplaySequence / HandCommandAdapter) and the learned route (model_action_adapter) each carried their own
source contract. This module holds the single description they derive from, per axis and per side:

* ``twin``: the provisional donor's joint, units, open/closed endpoints, closing direction, velocity cap and coupled children —
  VERIFIED on the twin (URDF limits; cap verified by the hand-req-M01 readback: PhysX honours the URDF velocity field on the six
  drive joints, the mimic children carry none).
* ``installed_e2``: the fields that would make the same six commands physically meaningful on the installed RH56E2 — endpoint
  datum, travel, speed, tactile semantics — every one explicit with a status; ``UNRESOLVED`` fields carry ``value: None`` and
  the schema refuses a number there.

Fail-closed: ``route('scripted')`` and ``route('model', ...)`` return the source contract a consumer feeds to HandCommandAdapter
only when every field the route needs is VERIFIED on the declared target; anything INFERRED/EXPLORATORY/UNRESOLVED needs
``exploratory=True`` and is written into the returned declaration (and into the contract hash); the target ``installed_e2``
never yields a contract while any of its fields is UNRESOLVED.
"""
import json
from pathlib import Path

from .embodiment import HAND_ACTUATORS, SIDES, ContractError, canonical_sha256, hand_contract_descriptor

STATUS = ('VERIFIED', 'INFERRED', 'EXPLORATORY', 'UNRESOLVED', 'NOMINAL')
TARGETS = ('twin', 'installed_e2')
ROUTES = ('scripted', 'model')
MODEL_DATASET_ORDERS = {'piston_n16': ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')}   # dataset pinky, ring, middle, index, thumb_pitch, thumb_yaw


def _status(field, path):
    if not isinstance(field, dict) or field.get('status') not in STATUS:
        raise ContractError('%s needs a status in %s' % (path, STATUS))
    if field['status'] == 'UNRESOLVED' and field.get('value') is not None:
        raise ContractError('%s is UNRESOLVED but carries a value %r (an unresolved installed-hand field must stay empty)' % (path, field['value']))
    if field['status'] in ('VERIFIED', 'INFERRED', 'NOMINAL') and field.get('value') is None:
        raise ContractError('%s is %s but has no value' % (path, field['status']))
    return field['status']


class HandActionContract:
    REQUIRED_TWIN = ('joint', 'unit', 'open', 'closed', 'closing_direction', 'velocity_cap_rad_s', 'coupled_children')
    REQUIRED_E2 = ('endpoint_datum', 'travel', 'speed', 'tactile')

    def __init__(self, data):
        if data.get('schema') != 'hand_action_contract_v1':
            raise ContractError('schema must be hand_action_contract_v1')
        self.data = data
        for side in SIDES:
            if side not in data['sides']:
                raise ContractError('missing side %s' % side)
            axes = data['sides'][side]['axes']
            if tuple(axes) != HAND_ACTUATORS:
                raise ContractError('%s axes must be exactly %s in this order' % (side, HAND_ACTUATORS))
            for a, ax in axes.items():
                for k in self.REQUIRED_TWIN:
                    if k not in ax['twin']:
                        raise ContractError('%s.%s.twin.%s missing' % (side, a, k))
                    _status(ax['twin'][k], '%s.%s.twin.%s' % (side, a, k))
                lo, hi = ax['twin']['open']['value'], ax['twin']['closed']['value']
                if lo == hi:
                    raise ContractError('%s.%s open and closed endpoints coincide' % (side, a))
                for k in self.REQUIRED_E2:
                    if k not in ax['installed_e2']:
                        raise ContractError('%s.%s.installed_e2.%s missing (must be explicit, even when UNRESOLVED)' % (side, a, k))
                    _status(ax['installed_e2'][k], '%s.%s.installed_e2.%s' % (side, a, k))
        for name, m in data.get('model_maps', {}).items():
            _status(m['axis_identity'], 'model_maps.%s.axis_identity' % name)
            if tuple(m['dataset_order']) != MODEL_DATASET_ORDERS.get(name, tuple(m['dataset_order'])) or sorted(m['dataset_order']) != sorted(HAND_ACTUATORS):
                raise ContractError('model map %s dataset_order must be a permutation of the six axes' % name)
        self.sha256 = canonical_sha256(data)

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text()))

    # ---- what each route needs, and what is only permitted under an explicit exploratory declaration --------------------
    def unresolved(self, side, target, route, model_map=None):
        """Fields that are not VERIFIED for this target/route (empty == the route may run as qualified)."""
        axes = self.data['sides'][side]['axes']; out = []
        for a, ax in axes.items():
            for k in self.REQUIRED_TWIN:
                if ax['twin'][k]['status'] != 'VERIFIED':
                    out.append('%s.%s.twin.%s:%s' % (side, a, k, ax['twin'][k]['status']))
            if target == 'installed_e2':
                for k in self.REQUIRED_E2:
                    if ax['installed_e2'][k]['status'] != 'VERIFIED':
                        out.append('%s.%s.installed_e2.%s:%s' % (side, a, k, ax['installed_e2'][k]['status']))
        if route == 'model':
            m = self.data['model_maps'].get(model_map)
            if m is None:
                raise ContractError('unknown model map %r' % model_map)
            if m['axis_identity']['status'] != 'VERIFIED':
                out.append('model_maps.%s.axis_identity:%s' % (model_map, m['axis_identity']['status']))
        return out

    def route(self, route, side, *, target='twin', model_map=None, exploratory=False, saturation_policy='reject', endpoint_tolerance=0.0):
        """Source contract for HandCommandAdapter (+ declaration). scripted: normalized closure 0..1 per axis. model: the
        dataset's axis order and units (radians, identity to the twin endpoints, declared clip for out-of-range values)."""
        if route not in ROUTES or target not in TARGETS or side not in SIDES:
            raise ContractError('route/target/side invalid')
        missing = self.unresolved(side, target, route, model_map)
        if missing and not exploratory:
            raise ContractError('fail-closed: %s route on %s needs an explicit exploratory=True for %s' % (route, target, missing))
        if target == 'installed_e2' and any(':UNRESOLVED' in m for m in missing):
            raise ContractError('no contract for the installed hand while fields are UNRESOLVED: %s' % [m for m in missing if ':UNRESOLVED' in m])
        axes = self.data['sides'][side]['axes']
        if route == 'scripted':
            src = {'axis_order': list(HAND_ACTUATORS), 'open_value': 0.0, 'closed_value': 1.0, 'saturation_policy': saturation_policy, 'endpoint_tolerance': endpoint_tolerance,
                   'axis_semantics': {a: {'direction': axes[a]['twin']['closing_direction']['status'], 'order': 'VERIFIED', 'scale': axes[a]['twin']['closed']['status']} for a in HAND_ACTUATORS}}
        else:
            m = self.data['model_maps'][model_map]
            src = {'axis_order': list(m['dataset_order']), 'open_value': 0.0, 'closed_value': 1.0,
                   'per_axis_endpoints': {a: {'open_value': axes[a]['twin']['open']['value'], 'closed_value': axes[a]['twin']['closed']['value']} for a in HAND_ACTUATORS},
                   'saturation_policy': 'clip_declared', 'endpoint_tolerance': endpoint_tolerance,
                   'axis_semantics': {a: {'direction': m['axis_identity']['status'], 'order': 'VERIFIED', 'scale': m['axis_identity']['status']} for a in HAND_ACTUATORS},
                   'note': m.get('note', '')}
        decl = {'contract_sha256': self.sha256, 'route': route, 'side': side, 'target': target, 'model_map': model_map, 'exploratory': bool(exploratory), 'fields_not_verified': missing,
                'profile_sha256': hand_contract_descriptor({side: src})}
        return src, decl

    def velocity_caps(self, side):
        return {a: ax['twin']['velocity_cap_rad_s']['value'] for a, ax in self.data['sides'][side]['axes'].items()}
