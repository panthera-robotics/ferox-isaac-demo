"""Presentation-only declarations shared by the fixed-pelvis probes (Sprint P): external camera placement and a RENDER-ONLY
visual overlay (hero-world set dressing). Cameras never touch physics; the overlay is a referenced USD layer under a declared
root whose physics schemas are disabled or deactivated at insertion — collision, rigid bodies, joints, articulation roots,
physics scenes — with the counts returned for the run's metrics. The probes' live PhysX shape statistics are the audit that
the physics path is unchanged. Nothing here changes the robot, the task scene, gains or timing."""
import hashlib
import math
from pathlib import Path

DEFAULT_TRACK_TARGETS = ('right_base_link', 'right_wrist_yaw_link', 'object')


def validate_cameras(cams, track_targets=DEFAULT_TRACK_TARGETS):
    if not isinstance(cams, list) or not 1 <= len(cams) <= 6:
        raise ValueError('presentation.cameras: 1..6 cameras')
    labels = set()
    for c in cams:
        if not isinstance(c, dict) or not isinstance(c.get('label'), str) or not c['label'].isidentifier() or c['label'] in labels:
            raise ValueError('presentation.cameras: unique identifier labels')
        labels.add(c['label'])
        res = c.get('resolution', [640, 640])
        if not (isinstance(res, list) and len(res) == 2 and all(type(v) is int and 160 <= v <= 1280 for v in res)):
            raise ValueError('presentation.cameras.resolution: two ints 160..1280')
        for key in ('position_m', 'look_at_m', 'offset_m'):
            if key in c and not (isinstance(c[key], list) and len(c[key]) == 3 and all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) < 10. for v in c[key])):
                raise ValueError('presentation.cameras.%s: three finite metres' % key)
        if 'track' in c:
            if c['track'] not in track_targets or 'offset_m' not in c:
                raise ValueError('presentation.cameras.track: one of %s with offset_m' % (tuple(track_targets),))
            if 'look_at_m' in c or 'position_m' in c:
                raise ValueError('a tracking camera is placed by offset_m from its target only')
        elif 'position_m' not in c or 'look_at_m' not in c:
            raise ValueError('a static camera needs position_m and look_at_m')
        if 'focal_length_mm' in c and not (isinstance(c['focal_length_mm'], (int, float)) and 4. <= c['focal_length_mm'] <= 200.):
            raise ValueError('presentation.cameras.focal_length_mm: 4..200')
    return cams


def validate_visual_overlay(spec):
    if spec is None:
        return None
    if not isinstance(spec, dict) or not set(spec) <= {'usd_path', 'root', 'translate_m', 'note'} or 'usd_path' not in spec:
        raise ValueError('presentation.visual_overlay: usd_path [root, translate_m, note] only')
    if not isinstance(spec['usd_path'], str) or not spec['usd_path'].endswith(('.usd', '.usda', '.usdc', '.usdz')):
        raise ValueError('presentation.visual_overlay.usd_path: a USD file path')
    root = spec.get('root', '/World/Visual')
    if not isinstance(root, str) or not root.startswith('/World/Visual') or not all(part.isidentifier() for part in root.strip('/').split('/')):
        raise ValueError('presentation.visual_overlay.root: a prim path under /World/Visual')
    if 'translate_m' in spec and not (isinstance(spec['translate_m'], list) and len(spec['translate_m']) == 3 and
                                      all(isinstance(v, (int, float)) and math.isfinite(v) and abs(v) < 10. for v in spec['translate_m'])):
        raise ValueError('presentation.visual_overlay.translate_m: three finite metres')
    if 'note' in spec and not isinstance(spec['note'], str):
        raise ValueError('presentation.visual_overlay.note: string')
    return spec


def validate_presentation(pres, track_targets=DEFAULT_TRACK_TARGETS):
    """Sprint P: {cameras?, visual_overlay?, note?}. Returns the validated block (or None)."""
    if pres is None:
        return None
    if not isinstance(pres, dict) or not set(pres) <= {'cameras', 'visual_overlay', 'note'}:
        raise ValueError('presentation: cameras / visual_overlay / note only')
    if 'cameras' in pres:
        validate_cameras(pres['cameras'], track_targets)
    validate_visual_overlay(pres.get('visual_overlay'))
    if 'note' in pres and not isinstance(pres['note'], str):
        raise ValueError('presentation.note: string')
    return pres


def add_visual_overlay(stage, spec, add_reference_to_stage, UsdGeom, UsdPhysics, Gf):
    """Reference the overlay USD under spec.root, apply the declared translation and strip physics under the root.
    Returns the audit for metrics. Call it after the task scene is complete and before the world reset, so the live
    PhysX statistics taken after the reset cover the overlay."""
    root = spec.get('root', '/World/Visual'); usd_path = Path(spec['usd_path'])
    if not usd_path.is_file():
        raise ValueError('presentation.visual_overlay.usd_path not found: %s' % usd_path)
    add_reference_to_stage(str(usd_path), root)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(root)); xf.ClearXformOpOrder()
    if 'translate_m' in spec:
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in spec['translate_m']]))
    audit = {'usd_path': str(usd_path), 'usd_sha256': hashlib.sha256(usd_path.read_bytes()).hexdigest(), 'root': root,
             'translate_m': list(spec.get('translate_m', [0., 0., 0.])), 'prims': 0, 'collision_disabled': 0, 'rigid_bodies_disabled': 0,
             'joints_deactivated': 0, 'articulation_roots_removed': 0, 'physics_scenes_deactivated': 0,
             'declaration': 'render-only set dressing: no collider, body, joint, articulation or physics scene from this layer reaches PhysX'}
    under_root = [prim for prim in stage.Traverse() if str(prim.GetPath()) == root or str(prim.GetPath()).startswith(root + '/')]
    for prim in under_root:
        audit['prims'] += 1
        if prim.IsA(UsdPhysics.Scene):
            prim.SetActive(False); audit['physics_scenes_deactivated'] += 1; continue
        if prim.IsA(UsdPhysics.Joint):
            prim.SetActive(False); audit['joints_deactivated'] += 1; continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI); audit['articulation_roots_removed'] += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False); audit['rigid_bodies_disabled'] += 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False); audit['collision_disabled'] += 1
    return audit
