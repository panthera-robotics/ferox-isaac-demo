"""Coordinator-consumable six-axis hand command sets (sprint N, hand lane).

Every set is expressed through the ONE hand action contract (``hand_action_contract.py``): six named axes in the fixed order
index, middle, ring, little, thumb_bend, thumb_rotation, as normalized closure (0 = open endpoint, 1 = closed endpoint of the
twin) or as twin radians, converted to joint targets by ``HandCommandAdapter`` exactly as the scripted replay route does. The sets
carry their provenance (the qualified spec / grasp file and its sha256) and the applied-profile hash that the qualification
graph binds (``hand_contract_sha256``). Nothing here changes a gain, a limit or a fixture; a set reproduces a command, not a
result — the run that qualified it is named in ``provenance``.
"""
import json
import math
from pathlib import Path

from .embodiment import HAND_ACTUATORS, ContractError, HandCommandAdapter, canonical_sha256, hand_contract_descriptor
from .hand_action_contract import HandActionContract

DEFAULT_SETS = Path(__file__).resolve().parent / 'embodiments' / 'hand_command_sets_v1.json'
DEFAULT_CONTRACT = Path(__file__).resolve().parent / 'embodiments' / 'hand_action_contract_rh56dftp_donor_v1.json'


def _blend(a, kind):
    if kind == 'cosine':
        return 0.5 - 0.5 * math.cos(math.pi * a)
    if kind == 'linear':
        return a
    raise ContractError('unknown blend %r' % kind)


class HandCommandSets:
    def __init__(self, data, contract: HandActionContract):
        if data.get('schema') != 'hand_command_sets_v1':
            raise ContractError('schema must be hand_command_sets_v1')
        self.data, self.contract = data, contract
        if data['contract_sha256'] != contract.sha256:
            raise ContractError('command sets were written for contract %s, loaded %s' % (data['contract_sha256'][:12], contract.sha256[:12]))
        for name, s in data['sets'].items():
            if s['units'] not in ('normalized_closure', 'twin_rad'):
                raise ContractError('%s: units must be normalized_closure or twin_rad' % name)
            for stage, vec in s['stages'].items():
                if len(vec) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vec):
                    raise ContractError('%s.%s must be six finite values' % (name, stage))
        self.sha256 = canonical_sha256(data)

    @classmethod
    def load(cls, path=DEFAULT_SETS, contract_path=DEFAULT_CONTRACT):
        return cls(json.loads(Path(path).read_text()), HandActionContract.load(contract_path))

    # ---- closure vectors ---------------------------------------------------------------------------------------
    def closure(self, set_id, stage, side='right'):
        """Six normalized closure values (contract order) for a named stage of a set."""
        s = self.data['sets'][set_id]; vec = list(s['stages'][stage])
        if s['units'] == 'twin_rad':
            axes = self.contract.data['sides'][side]['axes']
            vec = [(v - axes[a]['twin']['open']['value']) / (axes[a]['twin']['closed']['value'] - axes[a]['twin']['open']['value']) for a, v in zip(HAND_ACTUATORS, vec)]
        return vec

    def ramp(self, set_id, stage_from, stage_to, duration_s, rate_hz, blend='cosine', side='right'):
        """Rows [(t_s, six closure values)] blending one stage into another (the shape the scripted specs use)."""
        a0, a1 = self.closure(set_id, stage_from, side), self.closure(set_id, stage_to, side); n = max(1, int(round(duration_s * rate_hz)))
        return [(k / rate_hz, [x + (y - x) * _blend(k / n, blend) for x, y in zip(a0, a1)]) for k in range(n + 1)]

    # ---- conversion through the contract (the scripted route) --------------------------------------------------
    def adapter(self, manifest, side='right', **route_kwargs):
        src, decl = self.contract.route('scripted', side, **route_kwargs)
        return HandCommandAdapter(manifest, side, src), decl

    def joint_targets(self, manifest, set_id, stage, side='right'):
        ad, decl = self.adapter(manifest, side)
        targets, info = ad.to_joint_targets(self.closure(set_id, stage, side))
        return targets, {'clipped_axes': info['clipped_axes'], 'closure': info['closure'], 'profile_sha256': decl['profile_sha256'], 'contract_sha256': self.contract.sha256, 'sets_sha256': self.sha256, 'set': set_id, 'stage': stage}

    def binding(self, manifest, side='right'):
        """The value a qualification claim binds for this hand command path (== hand_contract_sha256 of the applied profile)."""
        ad, _ = self.adapter(manifest, side); return hand_contract_descriptor({side: ad})


def verify_against_spec(sets: HandCommandSets, manifest, spec_path, set_id, side='right', tolerance=1e-9):
    """Re-convert every hand row of a qualified source spec through the contract route and compare with the spec's own
    contract (the path ReplaySequence used); returns {'rows', 'max_abs_diff_rad', 'stage_extremes'}. Raises on mismatch."""
    spec = json.loads(Path(spec_path).read_text()); legacy = HandCommandAdapter(manifest, side, spec['hand_contracts'][side]); ours, _ = sets.adapter(manifest, side)
    worst = 0.0; n = 0; ext = {}
    for row in spec['rows']:
        vals = row['hands'].get(side)
        if vals is None:
            continue
        a, _ = legacy.to_joint_targets(vals); b, _ = ours.to_joint_targets(vals); n += 1
        worst = max(worst, max(abs(a[k] - b[k]) for k in a))
        st = row.get('stage'); ext.setdefault(st, [vals, vals]); ext[st][1] = vals
    if worst > tolerance:
        raise ContractError('contract route differs from the qualified spec conversion by %.3e rad' % worst)
    s = sets.data['sets'][set_id]
    for stage, key in (('open', 'open'), ('closed', 'closed')):
        want = sets.closure(set_id, key, side); have = ext.get(s['spec_stage_for'][key])
        if have is None or max(abs(x - y) for x, y in zip(have[1] if key == 'closed' else have[0], want)) > 1e-6:
            raise ContractError('%s: the %s vector of the set does not match the spec stage %s' % (set_id, key, s['spec_stage_for'][key]))
    return {'rows': n, 'max_abs_diff_rad': worst, 'stages': sorted(ext)}
