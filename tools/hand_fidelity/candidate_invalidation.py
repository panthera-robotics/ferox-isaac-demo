"""What a candidate asset change would do to the bound qualifications of an embodiment manifest — computed on a temporary
copy of the URDF, never on the active asset or manifest.

A candidate is a named, reversible edit (corrected inertial, widened coupled-joint lower limits, changed coupling
reference). Its dependency values (`urdf_sha256`, `coupling_map_sha256`, ...) are compared with every claim's bound
configuration via `EmbodimentManifest.check_validity`; claims whose bound hashes differ become STALE. The manifest is
read only.
"""
from __future__ import annotations

import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from isaac.twin.inspire.embodiment import EmbodimentManifest, dependency_values_from_urdf

# Manifest v1 collision-cooking descriptor (the live value a runtime would report); passed through unchanged.
COLLISION_COOKING_V1 = 'right=ftp_palm_yz_slabs_v2;left=ftp_left_palm_yz_slabs_v1;contact_offset_m=0.0012860533315688372;rest_offset_m=0'


def _write_variant(urdf_path, edit, out_dir):
    tree = ET.parse(urdf_path); root = tree.getroot()
    edit(root)
    out = Path(out_dir) / ('candidate_' + Path(urdf_path).name)
    tree.write(out)
    return out


def widen_coupled_lower_limits(margin_rad, sides=('right', 'left')):
    """Candidate: give every mimic joint's own lower limit a negative margin (the six actuator limits are untouched)."""
    def edit(root):
        for j in root.findall('joint'):
            if j.find('mimic') is not None and any(j.get('name').startswith(s + '_') for s in sides):
                lim = j.find('limit'); lim.set('lower', repr(float(lim.get('lower')) - margin_rad))
    return edit


def mirror_base_link_inertial(from_side='right', to_side='left'):
    """Candidate: copy one side's base_link inertial to the other (mirrored x); which side is right is unknown."""
    def edit(root):
        links = {l.get('name'): l for l in root.findall('link')}
        src = links[from_side + '_base_link'].find('inertial'); dst = links[to_side + '_base_link'].find('inertial')
        xyz = [float(v) for v in src.find('origin').get('xyz').split()]
        dst.find('origin').set('xyz', '%r %r %r' % (-xyz[0], xyz[1], xyz[2]))
        dst.find('mass').set('value', src.find('mass').get('value'))
        for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
            v = float(src.find('inertia').get(k)); dst.find('inertia').set(k, repr(-v if k in ('ixy', 'ixz') else v))
    return edit


def invalidation_report(manifest_path, urdf_path, edit, *, live_extra=None):
    """Return {claim: active_compatibility} for the original and for the candidate URDF, plus the changed dependency keys."""
    m = EmbodimentManifest.load(manifest_path)
    extra = dict(live_extra or {})
    base = dependency_values_from_urdf(urdf_path, collision_cooking=COLLISION_COOKING_V1)
    with tempfile.TemporaryDirectory() as tmp:
        shutil.copy(urdf_path, Path(tmp) / Path(urdf_path).name)     # the candidate is written beside a private copy only
        cand_path = _write_variant(Path(tmp) / Path(urdf_path).name, edit, tmp)
        cand = dependency_values_from_urdf(cand_path, collision_cooking=COLLISION_COOKING_V1)
    changed = sorted(k for k in base if base[k] != cand.get(k))
    r0 = m.check_validity({**base, **extra}); r1 = m.check_validity({**cand, **extra})
    return {'changed_dependencies': changed,
            'original': {k: v['active_compatibility'] for k, v in r0['claims'].items()},
            'candidate': {k: v['active_compatibility'] for k, v in r1['claims'].items()},
            'transforms_candidate': {k: v['status'] for k, v in r1['transforms'].items()}}
