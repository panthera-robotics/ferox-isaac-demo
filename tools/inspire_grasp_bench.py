"""Hand-agnostic bench binding for the grasp probes (RH56E2 acceptance lane B, additive; the donor path is byte-identical).

A probe config may carry a `hand` block that names the bench hand; absent, every value below is the frozen donor bench:
  {"hand": {"source_urdf": "E2_right_hand_bench.urdf", "root_link": "right_base", "mount_rpy": [0, 0, 1.5707963267948966],
            "axis_joints": {"index": "right_index_proximal_joint", ..., "thumb_bend": "right_thumb_proximal_pitch_joint",
                            "thumb_rotation": "right_thumb_proximal_yaw_joint"},
            "axis_upper_rad": {"index": 1.4381, ..., "thumb_bend": 0.62, "thumb_rotation": 1.658},
            "palm_collision": "mesh_as_delivered", "label": "PUBLIC_E2_PRIOR__INSTALLED_CALIBRATION_INCOMPLETE"}}
Grasp configs stay in DONOR terms (GraspConfig validates donor bounds); the bench maps every donor-space joint target to the bench
hand closure-preservingly (c = q/donor_upper, q_bench = c * bench_upper — the declared six-axis adapter semantics) and renames it.
mount_rpy is the declared fixed rotation between the wrist fixture and the bench root that presents the hand to the world (holder,
rack, gravity) exactly like the donor palm frame (derived from the two merged twin URDFs: T_wrist->E2root * Rz(+90 deg) = T_wrist->donor palm).
"""
from dataclasses import dataclass, field
import math

AXES = ('index', 'middle', 'ring', 'little', 'thumb_bend', 'thumb_rotation')
DONOR_JOINTS = {'index': 'right_index_1_joint', 'middle': 'right_middle_1_joint', 'ring': 'right_ring_1_joint', 'little': 'right_little_1_joint',
                'thumb_bend': 'right_thumb_2_joint', 'thumb_rotation': 'right_thumb_1_joint'}
DONOR_UPPER = {'index': 1.4381, 'middle': 1.4381, 'ring': 1.4381, 'little': 1.4381, 'thumb_bend': .5864, 'thumb_rotation': 1.1641}


@dataclass(frozen=True)
class HandBench:
    source_urdf: str = 'FTP_right_hand_bench.urdf'
    root_link: str = 'right_base_link'
    mount_rpy: tuple = None
    axis_joints: dict = field(default_factory=lambda: dict(DONOR_JOINTS))
    axis_upper_rad: dict = field(default_factory=lambda: dict(DONOR_UPPER))
    palm_collision: str = 'donor_slabs_v2'
    fold_massless_frames: bool = False   # E2: fold the massless flange root into the palm body and drop massless tcp leaves before import (record returned)
    label: str = 'DONOR_BASELINE_FROZEN (provisional RH56DFTP donor bench)'

    def __post_init__(self):
        if set(self.axis_joints) != set(AXES) or set(self.axis_upper_rad) != set(AXES): raise ValueError('hand block needs all six axes')
        if self.palm_collision not in ('donor_slabs_v2', 'mesh_as_delivered'): raise ValueError('palm_collision must be donor_slabs_v2 or mesh_as_delivered')
        if self.mount_rpy is not None:
            v = tuple(float(x) for x in self.mount_rpy)
            if len(v) != 3 or any(not math.isfinite(x) for x in v): raise ValueError('mount_rpy needs three finite values')
            object.__setattr__(self, 'mount_rpy', v)
        for a in AXES:
            if not (0 < float(self.axis_upper_rad[a]) <= 3.2): raise ValueError('axis upper out of range')

    @classmethod
    def from_config(cls, config_data):
        h = dict(config_data.get('hand') or {})
        return cls(**h)

    @property
    def is_donor(self):
        return self.source_urdf == 'FTP_right_hand_bench.urdf' and self.axis_joints == DONOR_JOINTS and self.axis_upper_rad == DONOR_UPPER and self.palm_collision == 'donor_slabs_v2' and self.mount_rpy is None and not self.fold_massless_frames

    def axis_of_donor_joint(self, donor_name):
        for a, n in DONOR_JOINTS.items():
            if n == donor_name: return a
        raise KeyError(donor_name)

    def bench_name(self, donor_name):
        return self.axis_joints[self.axis_of_donor_joint(donor_name)]

    def scale(self, donor_name):
        a = self.axis_of_donor_joint(donor_name); return float(self.axis_upper_rad[a]) / DONOR_UPPER[a]

    def map_targets(self, donor_targets):
        """{donor joint: rad} -> {bench joint: rad} closure-preservingly (identity on the donor)."""
        return {self.bench_name(n): float(q) * self.scale(n) for n, q in donor_targets.items()}

    def map_value(self, donor_name, value):
        return None if value is None else float(value) * self.scale(donor_name)

    def facts(self):
        return {'source_urdf': self.source_urdf, 'root_link': self.root_link, 'mount_rpy': self.mount_rpy, 'axis_joints': dict(self.axis_joints), 'axis_upper_rad': dict(self.axis_upper_rad),
                'donor_upper_rad': dict(DONOR_UPPER), 'palm_collision': self.palm_collision, 'fold_massless_frames': self.fold_massless_frames, 'label': self.label, 'is_donor': self.is_donor,
                'mapping': 'identity' if self.is_donor else 'closure-preserving: q_bench = (q_donor / donor_upper) * bench_upper per axis; names renamed; increments/releases scaled likewise'}


@dataclass(frozen=True)
class WriterHand:
    """Assembled-body hand binding for the writer probe (R9). Defaults = the frozen donor (byte-identical path)."""
    source_urdf: str = 'g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf'
    importer: str = 'donor'                       # donor (inspire_body_asset.import_body + slab candidates) | e2 (inspire_e2_asset.import_body_e2, meshes as delivered)
    palm_body_link: str = 'right_base_link'       # the rigid body carrying the palm (E2: right_hand_base_link)
    donor_palm_in_palm_body: tuple = ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.))   # T(palm body -> donor palm frame); identity on the donor
    axis_joints: dict = field(default_factory=lambda: dict(DONOR_JOINTS))
    axis_upper_rad: dict = field(default_factory=lambda: dict(DONOR_UPPER))
    label: str = 'DONOR_BASELINE_FROZEN (provisional RH56DFTP donor assembly)'

    def __post_init__(self):
        if self.importer not in ('donor', 'e2'): raise ValueError('importer must be donor or e2')
        if set(self.axis_joints) != set(AXES) or set(self.axis_upper_rad) != set(AXES): raise ValueError('hand block needs all six axes')
        M = [[float(x) for x in row] for row in self.donor_palm_in_palm_body]
        if len(M) != 4 or any(len(r) != 4 for r in M) or any(not math.isfinite(x) for r in M for x in r): raise ValueError('donor_palm_in_palm_body must be a finite 4x4')
        object.__setattr__(self, 'donor_palm_in_palm_body', tuple(tuple(r) for r in M))

    @classmethod
    def from_config(cls, config_data):
        return cls(**dict(config_data.get('hand') or {}))

    @property
    def is_donor(self):
        return self.importer == 'donor' and self.axis_joints == DONOR_JOINTS and self.axis_upper_rad == DONOR_UPPER and self.palm_body_link == 'right_base_link'

    def _axis(self, donor_name):
        for a, n in DONOR_JOINTS.items():
            if n == donor_name: return a
        raise KeyError(donor_name)

    def map_targets(self, donor_targets):
        return {self.axis_joints[self._axis(n)]: float(q) * (float(self.axis_upper_rad[self._axis(n)]) / DONOR_UPPER[self._axis(n)]) for n, q in donor_targets.items()}   # ratio first: exact identity on the donor

    def facts(self):
        return {'source_urdf': self.source_urdf, 'importer': self.importer, 'palm_body_link': self.palm_body_link, 'donor_palm_in_palm_body': [list(r) for r in self.donor_palm_in_palm_body],
                'axis_joints': dict(self.axis_joints), 'axis_upper_rad': dict(self.axis_upper_rad), 'donor_upper_rad': dict(DONOR_UPPER), 'label': self.label, 'is_donor': self.is_donor,
                'mapping': 'identity' if self.is_donor else 'closure-preserving per axis; holder pose re-expressed through donor_palm_in_palm_body'}


def fold_massless_frames(source, output):
    """Bench pre-processing for hands whose URDF carries massless, collision-less frames (public E2: the flange root `<side>_base` and the
    tcp_* convenience leaves): removes massless leaf links (fixed joints, no children) and folds a massless ROOT with exactly one fixed
    child joint into that child (the child becomes the root; the removed fixed transform must be composed into the declared mount, see
    the returned record). Physical links, joints, limits, mimics, meshes and inertials are untouched (asserted). Returns the record."""
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from copy import deepcopy
    src, out = Path(source).resolve(), Path(output).resolve()
    tree = ET.parse(src); root = tree.getroot()
    links = {l.get('name'): l for l in root.findall('link')}; joints = list(root.findall('joint'))
    def blob(e):
        c = deepcopy(e); c.tail = None; return ET.tostring(c)
    before_links = {n: blob(l) for n, l in links.items()}; before_joints = {j.get('name'): blob(j) for j in joints}
    massless = {n for n, l in links.items() if l.find('inertial') is None}
    parents = {j.find('child').get('link'): j for j in joints}; children = {}
    for j in joints: children.setdefault(j.find('parent').get('link'), []).append(j)
    removed_leaves = []
    for n in sorted(massless):
        if n in parents and not children.get(n) and parents[n].get('type') == 'fixed' and links[n].find('collision') is None:
            root.remove(links[n]); root.remove(parents[n]); removed_leaves.append({'link': n, 'joint': parents[n].get('name')}); joints.remove(parents[n])
    links = {l.get('name'): l for l in root.findall('link')}; joints = list(root.findall('joint'))
    child_set = {j.find('child').get('link') for j in joints}; roots = [n for n in links if n not in child_set]
    folded_root = None
    if len(roots) == 1 and roots[0] in massless:
        r = roots[0]; kids = [j for j in joints if j.find('parent').get('link') == r]
        if len(kids) == 1 and kids[0].get('type') == 'fixed':
            o = kids[0].find('origin')
            folded_root = {'removed_root': r, 'removed_joint': kids[0].get('name'), 'new_root': kids[0].find('child').get('link'),
                           'removed_joint_xyz': [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()], 'removed_joint_rpy': [float(v) for v in (o.get('rpy') if o is not None else '0 0 0').split()],
                           'mount_rule': 'declared mount = base mount composed with this removed fixed transform (root -> new_root)'}
            root.remove(links[r]); root.remove(kids[0])
    orig_mesh = {l.get('name'): [m.get('filename') for m in l.iter('mesh')] for l in root.findall('link')}
    for mesh in root.iter('mesh'):
        fn = Path(mesh.get('filename'))
        if not fn.is_absolute(): mesh.set('filename', str((src.parent / fn).resolve()))
    # every remaining link/joint byte-identical to the source apart from mesh path resolution
    for l in root.findall('link'):
        a = deepcopy(l); a.tail = None
        for m, fn0 in zip(a.iter('mesh'), orig_mesh[l.get('name')]): m.set('filename', fn0)
        assert ET.tostring(a) == before_links[l.get('name')], l.get('name')
    for j in root.findall('joint'): assert blob(j) == before_joints[j.get('name')], j.get('name')
    out.parent.mkdir(parents=True, exist_ok=True); ET.indent(root); tree.write(out, encoding='utf-8', xml_declaration=True)
    return {'source': str(src), 'output': str(out), 'removed_massless_leaves': removed_leaves, 'folded_root': folded_root,
            'new_root': folded_root['new_root'] if folded_root else roots[0], 'n_links': len(root.findall('link')), 'n_joints': len(root.findall('joint'))}
