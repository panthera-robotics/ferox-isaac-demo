"""Opt-in hand adapters for the cached piston route (GR00T N1.6 `g1-inspire-piston`) built from a named ConversionProfile,
usable with the existing `model_action_adapter.validate_piston_chunk(manifest, actions, hand_adapters=...)` without
modifying it. The active route keeps its own adapters; this module only offers the alternative, with the profile's
provenance/evidence attached for the package record.
"""
from __future__ import annotations

from isaac.twin.inspire.embodiment import EmbodimentManifest

from .conversion_profile import PISTON_ROUTE_PROVENANCE, ConversionProfile

SOURCE_CONVENTION = 'unitree_inspire_hand_urdf_radians_v1'   # piston dataset radians live in this model's joint space (INFERRED, strong)


def profiles(manifest: EmbodimentManifest, *, policy, operating_margin_rad=0.02, declared_clip=None):
    """{side: ConversionProfile} for the piston route under `policy` (closure_preserving | radian_identity).
    radian_identity gets a declared clip (the dataset thumb_yaw -0.1 lies below the donor limit; every clip is recorded)."""
    if declared_clip is None:
        declared_clip = policy == 'radian_identity'
    return {side: ConversionProfile(profile_id='piston-%s-%s-v1' % (policy, side), side=side, source_convention=SOURCE_CONVENTION, manifest=manifest, policy=policy,
                                    operating_margin_rad=operating_margin_rad, exploratory=True, declared_clip=declared_clip, provenance=PISTON_ROUTE_PROVENANCE,
                                    urdf_sha256=manifest.data['source_asset']['urdf_sha256']) for side in ('left', 'right')}


def hand_adapters(manifest: EmbodimentManifest, *, policy, **kw):
    """Drop-in `hand_adapters` for validate_piston_chunk: the profiles' HandCommandAdapters (order little..thumb_rotation = the dataset order)."""
    return {side: p.adapter for side, p in profiles(manifest, policy=policy, **kw).items()}


def route_record(manifest: EmbodimentManifest, *, policy, **kw):
    """What to store beside a package built with these adapters: profile ids, hashes, evidence, policy — EXPLORATORY."""
    ps = profiles(manifest, policy=policy, **kw)
    return {'route': 'piston_route_%s' % policy, 'exploratory': True, 'profiles': {s: p.describe() for s, p in ps.items()},
            'note': 'neither policy is verified against the installed hand; a qualified route must refuse both until the datum is resolved'}
