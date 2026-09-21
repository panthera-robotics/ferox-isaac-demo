"""Versioned embodiment and command contract for the G1 + Inspire twin (simulation and data replay).

One manifest separates, per version: hardware identity from source-asset identity and qualification
status; the 29 named body coordinates, the six independent hand actuators per side and the coupled
(mimic) joints they drive; command versus feedback conventions (units, signs, endpoints,
normalization, saturation policy); wrist/hand/tool transforms with provenance and validity hashes;
the controller description (type, gains provenance, rate, latency, clock domains); camera and
tactile/force descriptors with their availability. Nothing here is a hardware calibration: unknown
E2 parameters stay ``None`` and are reported as unknown.

A logical 29 + 6 + 6 interface is not a packet width: sources declare their own axis order and scale,
conversion is by name, coupled joints are never commanded, and invalid commands fail before any
transmission. Changing the asset, mount, grasp or holder invalidates the dependent transforms and the
qualifications that rest on them (``check_validity``).
"""
from __future__ import annotations

import hashlib
import json
import math
from typing import Mapping

MANIFEST_SCHEMA_VERSION = 1

# Unitree G1 29-DoF SDK joint index order (G1JointIndex); the twin's body arbiter maps these by name.
BODY_JOINT_ORDER_UNITREE_29 = (
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint')
HAND_ACTUATORS = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')
SIDES = ('left', 'right')
SATURATION_POLICIES = ('reject', 'clip_declared')
# Qualification claims whose evidence was produced through a hand command conversion and an arm reference datum: they
# must bind the applied conversion profile (`hand_contract_sha256`, manifest-independent profile hash of every applied side)
# and the arm datum (`arm_datum`), or they can never be ACTIVE_COMPATIBLE (a hash proves identity, not physical correctness).
HAND_COMMAND_DEPENDENT_CLAIMS = ('command_replay_integration', 'learned_policy_evaluation', 'real_data_agreement')
HAND_COMMAND_DEPENDENCIES = ('hand_contract_sha256', 'arm_datum')
ARM_DATUM_SCRIPTED = 'body_q_rad:urdf_absolute'    # ReplaySequence rows are absolute URDF radians
SOURCE_KINDS = ('real_recording', 'simulator_recording', 'synthetic_test_sequence', 'model_generated')   # model outputs are never relabelled as recordings


class ContractError(ValueError):
    """Refusal before transmission: the command, manifest or source contract is invalid."""


def _finite(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ContractError('%s must be finite numeric data' % name)
    if (low is not None and value < low) or (high is not None and value > high):
        raise ContractError('%s outside its admitted bound [%s, %s]' % (name, low, high))
    return float(value)


def canonical_sha256(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _fixed_joint(root, parent, child):
    for j in root.findall('joint'):
        if j.get('type') == 'fixed' and j.find('parent').get('link') == parent and j.find('child').get('link') == child:
            o = j.find('origin')
            return {'parent_link': parent, 'child_link': child, 'xyz_m': [float(v) for v in (o.get('xyz') or '0 0 0').split()], 'rpy_rad': [float(v) for v in (o.get('rpy') or '0 0 0').split()]}
    raise ContractError('no fixed joint %s -> %s' % (parent, child))


# The hand flange (wrist -> hand base) fixed joint, per asset family: the FTP donor names the hand base right_base_link, the
# public exact-E2 description names it right_base. A manifest may declare the pair explicitly through
# transforms.right.wrist_to_hand.frame ("<parent> -> <child>"); otherwise the first known pair present in the URDF is used.
WRIST_MOUNT_PAIRS = (('right_wrist_yaw_link', 'right_base_link'), ('right_wrist_yaw_link', 'right_base'))


def wrist_mount_joint(root, pair=None):
    """Origin of the right hand flange joint: the declared (parent, child) pair, else the first known pair found in the URDF."""
    if pair is not None:
        return _fixed_joint(root, pair[0], pair[1])
    for parent, child in WRIST_MOUNT_PAIRS:
        try:
            return _fixed_joint(root, parent, child)
        except ContractError:
            continue
    raise ContractError('no hand flange joint among %s' % (WRIST_MOUNT_PAIRS,))


# Collision model of the mounted asset, declared by source_asset.collision_model {kind, cooking}; absent = the FTP donor's
# provisional palm/thumb slab candidates exactly as every qualified donor run applied them (cooking string unchanged).
COLLISION_MODELS = {
    'ftp_donor_slabs_v2': 'right=ftp_palm_yz_slabs_v2;left=ftp_left_palm_yz_slabs_v1;contact_offset_m=0.0012860533315688372;rest_offset_m=0',
    'public_stl_meshes': 'public_stl_meshes;approximation=convexDecomposition(importer);offsets=importer_default;no_slab_substitution',
    # Q02-r4 (coord-dec-R1-02): hand fixed links folded into their parent bodies at the URDF level, palm collider = e2_palm_yz_slabs_v1
    # (conservative convex pieces from 4 mm x/z cells of the exact E2 palm meshes), fingers/thumb importer convex-decomposed;
    # the representation is baked into the delivered URDFs, so every probe that imports them as delivered applies it.
    'e2_r4_folded_slabs': 'e2_r4:fixed_hand_links_folded;palm=e2_palm_yz_slabs_v1(4mm_xz_exact_meshes);fingers=convexDecomposition(importer);offsets=importer_default;baked_into_urdf',
    # Q02-r5 (coord-dec-R1-04): r4 + thumb-cavity carve of the palm pieces (cells within 1.5 mm of the thumb's swept collision surface rebuilt at 1 mm, sub-pieces within the margin dropped)
    'e2_r5_folded_slabs': 'e2_r5:fixed_hand_links_folded;palm=e2_palm_yz_slabs_v2_thumb_cavity(4mm_xz_exact_meshes+1mm_thumb_cavity_carve_1.5mm_margin);fingers=convexDecomposition(importer);offsets=importer_default;baked_into_urdf',
}


def collision_policy(manifest_data):
    """{'kind', 'cooking'} for the asset the manifest names. Unknown kinds are refused (never silently substituted)."""
    declared = (manifest_data.get('source_asset') or {}).get('collision_model')
    if declared is None:
        return {'kind': 'ftp_donor_slabs_v2', 'cooking': COLLISION_MODELS['ftp_donor_slabs_v2']}
    if not isinstance(declared, Mapping) or declared.get('kind') not in COLLISION_MODELS:
        raise ContractError('source_asset.collision_model.kind must be one of %s' % sorted(COLLISION_MODELS))
    cooking = declared.get('cooking', COLLISION_MODELS[declared['kind']])
    if cooking != COLLISION_MODELS[declared['kind']]:
        raise ContractError('source_asset.collision_model.cooking does not match the declared kind')
    return {'kind': declared['kind'], 'cooking': cooking}


def hand_link_names(manifest_data, side):
    """Asset-specific hand link roles for one side, declared by hands.<side>.link_names or defaulting to the FTP donor names:
    base_link = root of the hand subtree under the wrist (spawn clearance, kinematics); palm_body = the palm RIGID BODY the
    probes observe (views, close-up cameras, palm pose in traces); palm_body_rpy_from_base = fixed rotation from base_link to
    palm_body (zero when they are the same link). The public E2 asset declares base_link <side>_base (a pure frame) and
    palm_body <side>_hand_base_link (rpy pi, 0, 0 from the base)."""
    if side not in SIDES:
        raise ContractError('side must be left or right')
    declared = (manifest_data.get('hands', {}).get(side) or {}).get('link_names') or {}
    out = {'base_link': declared.get('base_link', side + '_base_link'), 'palm_body': declared.get('palm_body', side + '_base_link'),
           'palm_body_rpy_from_base': list(declared.get('palm_body_rpy_from_base', [0.0, 0.0, 0.0]))}
    for k in ('base_link', 'palm_body'):
        if not isinstance(out[k], str) or not out[k].startswith(side + '_'):
            raise ContractError('hands.%s.link_names.%s must be a link of that side' % (side, k))
    if len(out['palm_body_rpy_from_base']) != 3:
        raise ContractError('hands.%s.link_names.palm_body_rpy_from_base must be three angles' % side)
    return out


def wrist_mount_pair_from_manifest(manifest_data):
    """(parent, child) declared by transforms.right.wrist_to_hand.frame as "<parent> -> <child>", or None."""
    t = (manifest_data.get('transforms', {}).get('right') or {}).get('wrist_to_hand') or {}
    frame = t.get('frame')
    if isinstance(frame, str) and '->' in frame:
        parent, child = [x.strip() for x in frame.split('->', 1)]
        if parent and child:
            return parent, child
    return None


def fixed_joint_matrix(origin):
    """4x4 transform from a URDF origin dict (xyz + rpy, URDF convention Rz*Ry*Rx)."""
    r, p_, y = origin['rpy_rad']; xyz = origin['xyz_m']
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p_), math.sin(p_), math.cos(y), math.sin(y)
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
    return [[*R[0], xyz[0]], [*R[1], xyz[1]], [*R[2], xyz[2]], [0.0, 0.0, 0.0, 1.0]]


def coupling_map_from_urdf(root):
    """{side: {child: {parent, multiplier, offset, limit_rad}}} for the mimic joints of both hands, from the URDF."""
    joints = {j.get('name'): j for j in root.findall('joint') if j.get('type') in ('revolute', 'prismatic')}
    out = {}
    for side in SIDES:
        out[side] = {n: {'parent': j.find('mimic').get('joint'), 'multiplier': float(j.find('mimic').get('multiplier', 1)), 'offset': float(j.find('mimic').get('offset', 0)),
                         'limit_rad': [float(j.find('limit').get('lower')), float(j.find('limit').get('upper'))]}
                     for n, j in joints.items() if n.startswith(side + '_') and j.find('mimic') is not None}
    return out


def dependency_values_from_urdf(urdf_path, *, collision_cooking, physics_dt_s='0.005', solver='TGS_32_8', wrist_mount=None):
    """Live dependency values a runtime can compare against manifest bindings (identity, not physical correctness).
    wrist_mount: optional (parent, child) of the hand flange joint (see wrist_mount_joint)."""
    import xml.etree.ElementTree as ET
    data = open(urdf_path, 'rb').read(); root = ET.fromstring(data)
    coupling = coupling_map_from_urdf(root)
    return {'urdf_sha256': hashlib.sha256(data).hexdigest(),
            'coupling_map_sha256': hashlib.sha256(json.dumps(coupling, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
            'collision_cooking': collision_cooking,
            'wrist_mount_sha256': hashlib.sha256(json.dumps(fixed_joint_matrix(wrist_mount_joint(root, wrist_mount))).encode()).hexdigest(),
            'camera_mount_sha256': hashlib.sha256(json.dumps(_fixed_joint(root, 'torso_link', 'd435_link'), sort_keys=True).encode()).hexdigest(),
            'physics_dt_s': physics_dt_s, 'solver': solver}


def hand_profile_sha256(side, source_contract):
    """Manifest-independent hash of one side's applied conversion profile (axis order, endpoints, per-axis endpoints,
    saturation policy, endpoint tolerance, side). Unlike HandCommandAdapter.contract_sha256 it does not include the manifest
    hash, so a manifest can bind it without a circular dependency."""
    if side not in SIDES:
        raise ContractError('side must be left or right')
    order = source_contract.get('axis_order')
    if not isinstance(order, (list, tuple)) or sorted(order) != sorted(HAND_ACTUATORS):
        raise ContractError('source axis_order must be a permutation of the six named actuators')
    per_axis = {}
    for axis, spec in (source_contract.get('per_axis_endpoints') or {}).items():
        if axis not in HAND_ACTUATORS:
            raise ContractError('per_axis_endpoints names an unknown actuator %s' % axis)
        per_axis[axis] = [_finite(spec.get('open_value'), axis + '.open_value'), _finite(spec.get('closed_value'), axis + '.closed_value')]
    policy = source_contract.get('saturation_policy', 'reject')
    if policy not in SATURATION_POLICIES:
        raise ContractError('unknown saturation policy')
    return canonical_sha256({'axis_order': list(order), 'open_value': _finite(source_contract.get('open_value'), 'open_value'),
                             'closed_value': _finite(source_contract.get('closed_value'), 'closed_value'), 'per_axis_endpoints': dict(sorted(per_axis.items())),
                             'saturation_policy': policy, 'endpoint_tolerance': _finite(source_contract.get('endpoint_tolerance', 0.0), 'endpoint_tolerance', 0.0), 'side': side})


def hand_contract_descriptor(contracts_by_side):
    """One bindable value for the applied hand conversion: sha256 over {side: profile hash} of every applied side.
    Accepts HandCommandAdapter objects or raw source contracts."""
    if not contracts_by_side:
        return 'none'      # an arm-only run applies no hand conversion; binds as the literal 'none'
    profiles = {side: (c.profile_sha256 if hasattr(c, 'profile_sha256') else hand_profile_sha256(side, c)) for side, c in contracts_by_side.items()}
    return canonical_sha256(dict(sorted(profiles.items())))


def runtime_dependency_values(urdf_path, *, collision_cooking, physics_dt_s, support, controller, hand_contracts, arm_datum, solver='TGS_32_8', wrist_mount=None):
    """The live dependency values of one run, composed the same way by the packager and the probe: asset identities from the
    mounted URDF, the ACTUAL physics step (a DIAGNOSTIC refinement stales every dt-bound claim by construction), the support,
    the controller descriptor, the applied hand conversion profile(s) and the arm reference datum."""
    dt = float(physics_dt_s)
    if not (dt > 0.0 and math.isfinite(dt)):
        raise ContractError('physics_dt_s must be a positive finite number')
    return dict(dependency_values_from_urdf(urdf_path, collision_cooking=collision_cooking, physics_dt_s='%g' % dt, solver=solver, wrist_mount=wrist_mount), support=support, controller=controller,
                hand_contract_sha256=hand_contract_descriptor(hand_contracts), arm_datum=str(arm_datum))


class EmbodimentManifest:
    """Validated, hashed view of one manifest version. ``data`` is never mutated."""

    REQUIRED = ('schema_version', 'manifest_id', 'hardware_identity', 'source_asset', 'qualification', 'body', 'hands',
                'transforms', 'controller', 'cameras', 'tactile_force')

    def __init__(self, data: Mapping):
        if not isinstance(data, Mapping):
            raise ContractError('manifest must be a mapping')
        missing = [k for k in self.REQUIRED if k not in data]
        if missing:
            raise ContractError('manifest missing required fields: %s' % ', '.join(missing))
        if data['schema_version'] != MANIFEST_SCHEMA_VERSION:
            raise ContractError('unsupported manifest schema version')
        if not isinstance(data['manifest_id'], str) or not data['manifest_id']:
            raise ContractError('manifest_id required')
        ident = data['hardware_identity']
        for k in ('robot', 'right_hand', 'left_hand', 'identity_evidence'):
            if k not in ident:
                raise ContractError('hardware_identity.%s required' % k)
        qual = data['qualification']
        if not isinstance(qual, Mapping) or 'claims' not in qual or not isinstance(qual['claims'], Mapping):
            raise ContractError('qualification.claims must map claim names to bound claims')
        for name, claim in qual['claims'].items():
            if not isinstance(claim, Mapping) or claim.get('status') not in ('PASS', 'FAIL', 'EXECUTED_NOT_QUALIFIED', 'NOT_RUN', 'NOT_QUALIFIED'):
                raise ContractError('qualification claim %s needs a status of PASS/FAIL/EXECUTED_NOT_QUALIFIED/NOT_RUN/NOT_QUALIFIED' % name)
            if claim['status'] != 'NOT_RUN':
                cfg = claim.get('configuration')
                if not isinstance(cfg, Mapping) or not cfg or any(not isinstance(v, str) or not v for v in cfg.values()):
                    raise ContractError('qualification claim %s must bind a non-empty configuration of named content hashes/descriptors' % name)
                if not claim.get('evidence'):
                    raise ContractError('qualification claim %s needs an evidence reference' % name)
        if 'installed_hand_similarity_percent' not in qual:
            raise ContractError('qualification.installed_hand_similarity_percent must be present (null until independent real-hand evidence exists)')
        asset = data['source_asset']
        for k in ('asset_id', 'kind', 'exact_hand_model', 'urdf_sha256'):
            if k not in asset:
                raise ContractError('source_asset.%s required' % k)
        if asset['exact_hand_model'] is not True and asset['exact_hand_model'] is not False:
            raise ContractError('source_asset.exact_hand_model must be an explicit boolean')
        body = data['body']
        names = body.get('joint_names')
        if not isinstance(names, list) or len(names) != 29 or len(set(names)) != 29:
            raise ContractError('body.joint_names must list the 29 distinct body coordinates')
        if tuple(names) != BODY_JOINT_ORDER_UNITREE_29:
            raise ContractError('body.joint_names must follow the declared Unitree 29-DoF order by name')
        lim = body.get('limits_rad')
        if not isinstance(lim, Mapping) or set(lim) != set(names):
            raise ContractError('body.limits_rad must cover exactly the 29 body joints')
        for n, v in lim.items():
            lo, hi = _finite(v[0], n + '.lower'), _finite(v[1], n + '.upper')
            if lo >= hi:
                raise ContractError('%s lower limit must precede upper' % n)
        if body.get('floating_base', {}).get('state_fields') is None or body.get('floating_base', {}).get('support') is None:
            raise ContractError('body.floating_base must declare state_fields and support')
        hands = data['hands']
        if set(hands) != set(SIDES):
            raise ContractError('hands must describe exactly left and right')
        self._hand_joint_names = {}
        for side, hand in hands.items():
            act = hand.get('actuators')
            if not isinstance(act, Mapping) or tuple(act) != HAND_ACTUATORS:
                raise ContractError('%s hand must declare the six actuators in the contract order' % side)
            seen = set()
            for a, spec in act.items():
                for k in ('joint', 'open_rad', 'closed_rad', 'limit_rad'):
                    if k not in spec:
                        raise ContractError('%s.%s.%s required' % (side, a, k))
                if not isinstance(spec['joint'], str) or not spec['joint'].startswith(side + '_'):
                    raise ContractError('%s.%s joint must belong to that side' % (side, a))
                if spec['joint'] in seen:
                    raise ContractError('%s hand maps two actuators to one joint' % side)
                seen.add(spec['joint'])
                lo, hi = _finite(spec['limit_rad'][0], a + '.limit.lower'), _finite(spec['limit_rad'][1], a + '.limit.upper')
                o, c = _finite(spec['open_rad'], a + '.open_rad', lo, hi), _finite(spec['closed_rad'], a + '.closed_rad', lo, hi)
                if o == c:
                    raise ContractError('%s.%s open and closed endpoints coincide' % (side, a))
            coupled = hand.get('coupled_joints')
            if not isinstance(coupled, Mapping) or not coupled:
                raise ContractError('%s hand must declare its coupled joints' % side)
            for child, m in coupled.items():
                if child in seen or not child.startswith(side + '_'):
                    raise ContractError('coupled joint %s is not a distinct joint of the %s hand' % (child, side))
                if m.get('parent') not in seen and m.get('parent') not in coupled:
                    raise ContractError('coupled joint %s has no declared parent' % child)
                _finite(m.get('multiplier'), child + '.multiplier')
                _finite(m.get('offset', 0.0), child + '.offset')
            if hand.get('feedback', {}).get('independent_axes_measured') is None:
                raise ContractError('%s hand must declare whether independent axes are measured' % side)
            hand_link_names(data, side)   # validates hands.<side>.link_names when declared
            self._hand_joint_names[side] = tuple(spec['joint'] for spec in act.values())
        collision_policy(data)             # validates source_asset.collision_model when declared
        for key in ('wrist_to_hand', 'hand_to_tool'):
            for side in SIDES:
                t = data['transforms'].get(side, {}).get(key)
                if t is None:
                    continue
                for k in ('matrix_4x4', 'frame', 'provenance', 'valid_for'):
                    if k not in t:
                        raise ContractError('transforms.%s.%s.%s required' % (side, key, k))
                if len(t['matrix_4x4']) != 4 or any(len(r) != 4 for r in t['matrix_4x4']):
                    raise ContractError('transform matrix must be 4x4')
                if not isinstance(t['valid_for'], Mapping) or not t['valid_for']:
                    raise ContractError('transform validity hashes required')
        ctl = data['controller']
        for k in ('type', 'gains_provenance', 'rate_hz', 'latency_assumption', 'clock_domains'):
            if k not in ctl:
                raise ContractError('controller.%s required' % k)
        _finite(ctl['rate_hz'], 'controller.rate_hz', 1.0, 10000.0)
        for cam_id, cam in data['cameras'].items():
            for k in ('mount_frame', 'image_format', 'calibration', 'timestamp_domain'):
                if k not in cam:
                    raise ContractError('cameras.%s.%s required' % (cam_id, k))
        for f, spec in data['tactile_force'].items():
            if spec.get('availability') not in ('measured', 'estimated', 'simulator_only', 'unavailable'):
                raise ContractError('tactile_force.%s.availability must be declared' % f)
            if 'units' not in spec:
                raise ContractError('tactile_force.%s.units required' % f)
        self.data = json.loads(json.dumps(data))
        self.sha256 = canonical_sha256(self.data)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(json.load(f))

    @property
    def body_names(self):
        return tuple(self.data['body']['joint_names'])

    def body_limit(self, name):
        lo, hi = self.data['body']['limits_rad'][name]
        return float(lo), float(hi)

    def hand_joint_names(self, side):
        return self._hand_joint_names[side]

    def hand_actuator(self, side, actuator):
        return self.data['hands'][side]['actuators'][actuator]

    def check_validity(self, current: Mapping[str, str]):
        """Compare every bound dependency with the live content hashes/descriptors.

        ``current`` maps dependency names (urdf_sha256, coupling_map_sha256, collision_cooking, wrist_mount_sha256,
        hand_drive_gains, grasp_sha256, support, source_image, camera_mount_sha256, physics_dt_s, hand_contract_sha256,
        arm_datum, ...) to their live values (``runtime_dependency_values`` composes them for a run). A claim in
        HAND_COMMAND_DEPENDENT_CLAIMS that does not bind every HAND_COMMAND_DEPENDENCIES key is reported ``unbound`` and is
        UNVERIFIED at best: an applied conversion profile or datum it never named cannot be compatible with it. Each qualification claim keeps its historical status and gets an active compatibility:
        ACTIVE_COMPATIBLE when every bound dependency is present and equal, STALE when any differs, UNVERIFIED
        when any is missing. Transforms are checked the same way. Never mutates the manifest; never fills
        missing hashes.
        """
        report = {'transforms': {}, 'claims': {}, 'valid': True}
        for side, block in self.data['transforms'].items():
            for key, t in block.items():
                if t is None:
                    continue
                missing = [k for k in t['valid_for'] if k not in current]
                bad = {k: (current.get(k), v) for k, v in t['valid_for'].items() if k in current and current[k] != v}
                status = 'VALID' if not (bad or missing) else ('UNVERIFIED' if missing and not bad else 'INVALID')
                report['transforms']['%s.%s' % (side, key)] = {'status': status, 'mismatched': bad, 'missing': missing}
                if status != 'VALID':
                    report['valid'] = False
        for name, claim in self.data['qualification']['claims'].items():
            if claim['status'] == 'NOT_RUN':
                report['claims'][name] = {'historical_status': 'NOT_RUN', 'active_compatibility': 'NOT_APPLICABLE', 'mismatched': {}, 'missing': []}
                continue
            cfg = claim['configuration']
            missing = [k for k in cfg if k not in current]
            bad = {k: (current[k], v) for k, v in cfg.items() if k in current and current[k] != v}
            unbound = [k for k in HAND_COMMAND_DEPENDENCIES if k not in cfg] if name in HAND_COMMAND_DEPENDENT_CLAIMS else []
            active = 'ACTIVE_COMPATIBLE' if not (bad or missing or unbound) else ('STALE' if bad else 'UNVERIFIED')
            report['claims'][name] = {'historical_status': claim['status'], 'active_compatibility': active, 'mismatched': bad, 'missing': missing, 'unbound': unbound}
            if active != 'ACTIVE_COMPATIBLE':
                report['valid'] = False
        return report


class HandCommandAdapter:
    """Convert one side's six actuator values between a declared source contract and joint targets.

    A source contract declares ``axis_order`` (names from HAND_ACTUATORS, any permutation), the
    numeric ``scale`` (``open_value``, ``closed_value``), and its ``saturation_policy``. Values are
    normalized to closure c in [0, 1] (0 = open endpoint, 1 = closed endpoint) and mapped by name to the
    manifest's joint endpoints. Coupled joints are never emitted. Left and right never mix.
    """

    def __init__(self, manifest: EmbodimentManifest, side: str, source_contract: Mapping):
        if side not in SIDES:
            raise ContractError('side must be left or right')
        order = source_contract.get('axis_order')
        if not isinstance(order, (list, tuple)) or sorted(order) != sorted(HAND_ACTUATORS):
            raise ContractError('source axis_order must be a permutation of the six named actuators')
        self.open_value = _finite(source_contract.get('open_value'), 'open_value')
        self.closed_value = _finite(source_contract.get('closed_value'), 'closed_value')
        if self.open_value == self.closed_value:
            raise ContractError('source open and closed values coincide')
        policy = source_contract.get('saturation_policy', 'reject')
        if policy not in SATURATION_POLICIES:
            raise ContractError('unknown saturation policy')
        self.policy = policy
        self.tolerance = _finite(source_contract.get('endpoint_tolerance', 0.0), 'endpoint_tolerance', 0.0)
        # Optional per-axis endpoints (e.g. a source in joint radians whose open/closed values differ per axis);
        # each pair overrides the global pair for that axis only and must be finite and distinct.
        self.per_axis = {}
        for axis, spec in (source_contract.get('per_axis_endpoints') or {}).items():
            if axis not in HAND_ACTUATORS:
                raise ContractError('per_axis_endpoints names an unknown actuator %s' % axis)
            o, c = _finite(spec.get('open_value'), axis + '.open_value'), _finite(spec.get('closed_value'), axis + '.closed_value')
            if o == c:
                raise ContractError('per-axis open and closed values coincide for %s' % axis)
            self.per_axis[axis] = (o, c)
        self.manifest, self.side, self.order = manifest, side, tuple(order)
        self.contract_sha256 = canonical_sha256({'axis_order': self.order, 'open_value': self.open_value, 'closed_value': self.closed_value, 'per_axis_endpoints': {k: list(v) for k, v in sorted(self.per_axis.items())},
                                                 'saturation_policy': policy, 'endpoint_tolerance': self.tolerance, 'manifest': manifest.sha256, 'side': side})
        self.profile_sha256 = hand_profile_sha256(side, source_contract)   # manifest-independent, bindable by a manifest claim

    def endpoints(self, actuator):
        return self.per_axis.get(actuator, (self.open_value, self.closed_value))

    def closure(self, value, name, actuator=None):
        """Normalized closure from a source value; rejects or clips per the declared policy."""
        v = _finite(value, name)
        open_value, closed_value = self.endpoints(actuator) if actuator else (self.open_value, self.closed_value)
        lo, hi = sorted((open_value, closed_value))
        clipped = False
        if v < lo - self.tolerance or v > hi + self.tolerance:
            if self.policy == 'reject':
                raise ContractError('%s = %r outside the source range [%s, %s]' % (name, value, lo, hi))
            clipped = True
        v = min(max(v, lo), hi)
        return (v - open_value) / (closed_value - open_value), clipped

    def to_joint_targets(self, values):
        """Source vector (declared order) -> {joint_name: target_rad}; also returns clipping/closure info."""
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ContractError('hand command needs exactly six values in the declared order')
        targets, closures, clipped_axes = {}, {}, []
        for actuator, value in zip(self.order, values):
            c, clipped = self.closure(value, '%s.%s' % (self.side, actuator), actuator)
            spec = self.manifest.hand_actuator(self.side, actuator)
            q = spec['open_rad'] + c * (spec['closed_rad'] - spec['open_rad'])
            lo, hi = spec['limit_rad']
            targets[spec['joint']] = _finite(q, spec['joint'] + '.target', lo, hi)
            closures[actuator] = c
            if clipped:
                clipped_axes.append(actuator)
        return targets, {'closure': closures, 'clipped_axes': clipped_axes}

    def from_joint_state(self, joint_values: Mapping[str, float]):
        """Joint positions by name -> source vector (declared order); inverse of to_joint_targets."""
        out = []
        for actuator in self.order:
            spec = self.manifest.hand_actuator(self.side, actuator)
            if spec['joint'] not in joint_values:
                raise ContractError('missing joint %s for %s' % (spec['joint'], actuator))
            q = _finite(joint_values[spec['joint']], spec['joint'])
            c = (q - spec['open_rad']) / (spec['closed_rad'] - spec['open_rad'])
            o, cl = self.endpoints(actuator)
            out.append(o + c * (cl - o))
        return out


class ReplaySequence:
    """Validated command rows for one replay: monotonic time, complete fields, finite values, bounds.

    Each row: ``{"t_s": float, "body_q_rad": {name: rad} | null, "hands": {"right": [6 values] | null, "left": ...}}``.
    Body targets are by name (any subset of the 29; unnamed joints keep the initial hold). Rows are
    converted once, before transmission; any refusal aborts the whole sequence with the row index.
    """

    def __init__(self, manifest: EmbodimentManifest, rows, *, hand_contracts: Mapping[str, Mapping], source: Mapping,
                 maximum_step_s=1.0):
        if not isinstance(rows, list) or not rows:
            raise ContractError('replay needs at least one command row')
        for k in ('source_id', 'kind', 'provenance'):
            if k not in source:
                raise ContractError('source.%s required' % k)
        if source['kind'] not in SOURCE_KINDS:
            raise ContractError('source.kind must be one of %s' % (SOURCE_KINDS,))
        if source['kind'] == 'model_generated' and not all(k in source for k in ('model_id', 'observation_ref', 'adapter')):
            raise ContractError('model_generated sources must name model_id, observation_ref and adapter')
        self.manifest, self.source = manifest, dict(source)
        self.adapters = {s: HandCommandAdapter(manifest, s, c) for s, c in hand_contracts.items()}
        self.rows, self.converted, self.clipped_rows = list(rows), [], []
        last_t = None
        for i, row in enumerate(self.rows):
            if not isinstance(row, Mapping) or 't_s' not in row or 'body_q_rad' not in row or 'hands' not in row:
                raise ContractError('row %d missing required fields (t_s, body_q_rad, hands)' % i)
            t = _finite(row['t_s'], 'row %d t_s' % i, 0.0)
            if last_t is not None and not (0.0 < t - last_t <= maximum_step_s):
                raise ContractError('row %d time does not advance within (0, %s] s (stale, duplicate or gapped sample)' % (i, maximum_step_s))
            last_t = t
            body = {}
            if row['body_q_rad'] is not None:
                if not isinstance(row['body_q_rad'], Mapping) or not row['body_q_rad']:
                    raise ContractError('row %d body_q_rad must be a non-empty name map or null' % i)
                for n, v in row['body_q_rad'].items():
                    if n not in manifest.body_names:
                        raise ContractError('row %d names unknown body joint %s' % (i, n))
                    lo, hi = manifest.body_limit(n)
                    body[n] = _finite(v, 'row %d %s' % (i, n), lo, hi)
            hands = {}
            if not isinstance(row['hands'], Mapping) or set(row['hands']) - set(SIDES):
                raise ContractError('row %d hands must map sides to six values or null' % i)
            for side, values in row['hands'].items():
                if values is None:
                    continue
                if side not in self.adapters:
                    raise ContractError('row %d commands the %s hand without a declared source contract' % (i, side))
                targets, info = self.adapters[side].to_joint_targets(values)
                hands[side] = {'targets_rad': targets, **info}
                if info['clipped_axes']:
                    self.clipped_rows.append(i)
            self.converted.append({'row': i, 't_s': t, 'body_targets_rad': body, 'hands': hands})
        self.duration_s = last_t - self.converted[0]['t_s']
        self.contract_sha256 = canonical_sha256({'manifest': manifest.sha256, 'hands': {s: a.contract_sha256 for s, a in self.adapters.items()},
                                                 'source': self.source, 'rows': self.rows})

    def active_row(self, t_s):
        """Zero-order hold: the last converted row whose time is <= t_s (None before the first)."""
        active = None
        for c in self.converted:
            if c['t_s'] <= t_s:
                active = c
            else:
                break
        return active

    def summary(self):
        return {'rows': len(self.rows), 'duration_s': self.duration_s, 'clipped_rows': list(self.clipped_rows),
                'body_joints_commanded': sorted({n for c in self.converted for n in c['body_targets_rad']}),
                'hands_commanded': sorted({s for c in self.converted for s in c['hands']}), 'contract_sha256': self.contract_sha256,
                'source': self.source, 'manifest_id': self.manifest.data['manifest_id'], 'manifest_sha256': self.manifest.sha256}
