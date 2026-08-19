#!/usr/bin/env python3
"""Author isaac/assets/worlds/panthera_lab/panthera_lab.usd — MM2, campaign §4.3.

Runs INSIDE the Isaac container (needs pxr). Pure USD authoring: no app, no
rendering, no GPU. Verification is a separate pass over the written file, because
this repo's rule is that every generated artefact gets read back — that rule
already caught a reference resolving one directory too shallow and an importer
writing a 2.4 kB asset containing nothing, neither of which reported an error.

WHAT IS AUTHORED
  * 8.0 x 6.0 m room, 2.7 m ceiling. Walls, floor AND ceiling are real collision
    geometry, because the Mid-360 sweeps a full sphere and a room without a
    ceiling returns nothing above the horizon -- the twin's /scan finite ratio is
    a gate here (>=60 %) and an open-topped box would fail it for the wrong
    reason.
  * A door as a PhysX articulation: frame (static) + leaf 2.10 x 0.90 m on a
    revolute hinge limited 0..110 deg, with damping and a light closer spring, and
    a lever handle at 1.05 m. The leaf's mass is set explicitly.
  * Table 1.2 x 0.8 x 0.75 m, a counter, and a shelf.
  * Six YCB objects at TRUE SCALE, referenced from the Isaac asset server rather
    than copied, plus a 5 cm cube and a brochure-sized box. Masses and friction are
    authored explicitly, from the YCB dataset's published values.

WHAT IS DELIBERATELY NOT DONE
  * Nothing is scaled to fit. If a YCB asset arrives at the wrong size that is a
    finding to report, not a scale factor to apply (RULE in CLAUDE.md).
  * Object placement is randomised only through --seed, and the seed is written
    into the layer's customLayerData so any render can be traced back to it.

USAGE
  python3 build_lab_world.py --out /workspace/ferox_isaac/assets/worlds/panthera_lab
  python3 build_lab_world.py --verify-only --out <same dir>
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import sys

# pxr and PhysxSchema live inside kit, not on the plain interpreter's path, so
# the app has to come up before they can be imported -- even though nothing here
# renders. Headless and no window: this is an authoring pass, not a simulation.
from isaacsim import SimulationApp  # noqa: E402

_app = SimulationApp({"headless": True})

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

ASSET_ROOT = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com"
              "/Assets/Isaac/5.1")

ROOM_X, ROOM_Y, ROOM_Z = 8.0, 6.0, 2.7
WALL_T = 0.10

DOOR_W, DOOR_H, DOOR_T = 0.90, 2.10, 0.045
DOOR_LEAF_KG = 35.0            # §4.3: "real masses ~35 kg leaf"
DOOR_LIMIT_DEG = 110.0
HANDLE_Z = 1.05                # §4.3: lever handle at 1.05 m
FRAME_T = 0.06                 # jamb thickness
DOOR_UNDERCUT = 0.010          # real doors clear the floor; ours did not, and a
                               # 35 kg leaf resting on the slab is a brake

TABLE = dict(sx=1.20, sy=0.80, h=0.75, x=2.20, y=-1.60)
COUNTER = dict(sx=2.40, sy=0.60, h=0.90, x=-2.60, y=2.40)
SHELF = dict(sx=1.60, sy=0.40, h=1.80, x=-3.40, y=-1.80)

# An apron of floor OUTSIDE the door, with low side walls. Without it the
# `door_outside` waypoint stands on nothing: the floor slab ends at y=3.10 and the
# waypoint is at y=3.60, so "walk through the door" would have been a walk off the
# edge of the world. Found by checking the venue's coordinates against the authored
# geometry rather than by driving into it.
APRON = dict(sx=3.60, sy=2.40)

# name -> (usd, mass_kg, static_friction, dynamic_friction, source)
# Masses are the YCB dataset's published values; the soup can's 0.349 kg is the
# one the campaign brief names explicitly, which makes it the check on the rest.
YCB = {
    "soup_can":    ("005_tomato_soup_can.usd", 0.349, 0.70, 0.60, "YCB 005"),
    "mustard":     ("006_mustard_bottle.usd",  0.603, 0.70, 0.60, "YCB 006"),
    "cracker_box": ("003_cracker_box.usd",     0.411, 0.60, 0.50, "YCB 003"),
    "sugar_box":   ("004_sugar_box.usd",       0.514, 0.60, 0.50, "YCB 004"),
    "mug":         ("025_mug.usd",             0.118, 0.70, 0.60, "YCB 025"),
    "banana":      ("011_banana.usd",          0.066, 0.50, 0.40, "YCB 011"),
}

# Published YCB physical dimensions (metres), from the dataset's own object
# sheets. Used to prove the referenced assets arrive at TRUE SCALE. If one of
# these disagrees, that is a finding to report -- never a scale factor to apply.
YCB_TRUE_SIZE_M = {
    "soup_can":    (0.066, 0.066, 0.101),
    "mustard":     (0.058, 0.095, 0.190),
    "cracker_box": (0.060, 0.158, 0.210),
    "sugar_box":   (0.038, 0.089, 0.175),
    "mug":         (0.080, 0.116, 0.082),
    # The banana's published 36 x 190 x 39 gives its THICKNESS, but the fruit is
    # curved, so the mesh's own box spans ~74 mm across the arc. Measured from the
    # asset at 0.197 x 0.039 x 0.074 -- length agrees with the published 190 mm to
    # 7 mm, and the five other objects agree to <=12 mm, so the asset is true scale
    # and the published cross-section is simply not the box. Recorded as measured,
    # with the reason, rather than silently widening the tolerance.
    "banana":      (0.039, 0.074, 0.197),
}

# Objects with a well-defined standing orientation, and the height they stand at.
# YCB's Axis_Aligned assets are not authored Z-up in the twin's sense: the soup can
# arrives lying on its side (its 0.068 m diameter along Z, its 0.102 m height along
# Y). Standing them up is a PLACEMENT decision, not a fit -- nothing is scaled, and
# the resulting height is checked against the published figure. The mug and the
# banana are left as authored: a banana lies flat, and a mug's "up" is already up.
UPRIGHT_HEIGHT_M = {
    "soup_can": 0.101,
    "mustard": 0.190,
    "cracker_box": 0.210,
    "sugar_box": 0.175,
}

# Authored primitives, not references: size is ours to state exactly.
PRIMS = {
    "cube_5cm":    dict(size=(0.05, 0.05, 0.05), mass=0.100, sf=0.80, df=0.70),
    "brochure_box": dict(size=(0.21, 0.15, 0.03), mass=0.180, sf=0.60, df=0.50),
}


# --------------------------------------------------------------------------
_BBOX = {}


def _bbox_cache(stage):
    key = id(stage)
    if key not in _BBOX:
        _BBOX[key] = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    return _BBOX[key]


def _xform(stage, path):
    return UsdGeom.Xform.Define(stage, Sdf.Path(path))


def _box(stage, path, size, pos, *, collide=True, rigid=False, mass=None,
         color=(0.55, 0.55, 0.58)):
    """A cube scaled to `size`, positioned at `pos` (centre)."""
    c = UsdGeom.Cube.Define(stage, Sdf.Path(path))
    c.CreateSizeAttr(1.0)                       # unit cube, scaled below
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    x = UsdGeom.Xformable(c)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    # Extent must match the authored size or the renderer and the physics cooker
    # disagree about the bounds; cheap to author, expensive to debug.
    c.CreateExtentAttr([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    if rigid:
        UsdPhysics.RigidBodyAPI.Apply(c.GetPrim())
    if mass is not None:
        UsdPhysics.MassAPI.Apply(c.GetPrim()).CreateMassAttr(float(mass))
    return c


def _phys_material(stage, path, static_f, dynamic_f, restitution=0.0):
    m = UsdShade.Material.Define(stage, Sdf.Path(path))
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(float(static_f))
    api.CreateDynamicFrictionAttr(float(dynamic_f))
    api.CreateRestitutionAttr(float(restitution))
    return m


def _bind_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")


# --------------------------------------------------------------------------
def build(out_dir: str, seed: int) -> str:
    os.makedirs(out_dir, exist_ok=True)
    usd_path = os.path.join(out_dir, "panthera_lab.usd")
    if os.path.exists(usd_path):
        os.remove(usd_path)          # CreateNew refuses to clobber
    stage = Usd.Stage.CreateNew(usd_path)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = _xform(stage, "/panthera_lab")
    stage.SetDefaultPrim(root.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/panthera_lab/physicsScene"))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)

    # --- materials -------------------------------------------------------
    mats = _xform(stage, "/panthera_lab/materials")
    # "floor material like the DSO carpet/tiles": carpet is the higher-friction
    # of the two and the conservative choice for a locomotion venue.
    m_floor = _phys_material(stage, "/panthera_lab/materials/floor_carpet", 0.90, 0.80)
    m_wall = _phys_material(stage, "/panthera_lab/materials/wall", 0.60, 0.50)
    m_wood = _phys_material(stage, "/panthera_lab/materials/wood", 0.55, 0.45)

    # --- shell -----------------------------------------------------------
    shell = _xform(stage, "/panthera_lab/shell")
    hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0
    floor = _box(stage, "/panthera_lab/shell/floor",
                 (ROOM_X + 2 * WALL_T, ROOM_Y + 2 * WALL_T, WALL_T), (0, 0, -WALL_T / 2),
                 color=(0.32, 0.30, 0.28))
    _bind_material(floor.GetPrim(), m_floor)
    ceiling = _box(stage, "/panthera_lab/shell/ceiling",
                   (ROOM_X + 2 * WALL_T, ROOM_Y + 2 * WALL_T, WALL_T),
                   (0, 0, ROOM_Z + WALL_T / 2),
                   color=(0.85, 0.85, 0.87))
    _bind_material(ceiling.GetPrim(), m_wall)

    # North wall carries the doorway, so it is authored as two piers and a
    # header rather than one slab.
    door_cx = 2.0                       # doorway centre along +x on the north wall
    # The piers frame the ROUGH OPENING (leaf + both jambs), not the leaf. Sizing
    # them to the leaf buried the west jamb 60 x 60 mm through the pier's full
    # 2.16 m height: a rigid articulation base link penetrating a static collider,
    # which is why the door reported a healthy 'hinge' DOF and would not move off
    # -0.33 deg. Found by intersecting the authored boxes, not by watching it.
    rough_w = DOOR_W + 2 * FRAME_T
    for name, sx, cx in (
        ("north_pier_w", (door_cx - rough_w / 2.0) + hx,
         (-hx + (door_cx - rough_w / 2.0)) / 2.0),
        ("north_pier_e", hx - (door_cx + rough_w / 2.0),
         (hx + (door_cx + rough_w / 2.0)) / 2.0),
    ):
        if sx <= 0.01:
            continue
        w = _box(stage, f"/panthera_lab/shell/{name}", (sx, WALL_T, ROOM_Z),
                 (cx, hy + WALL_T / 2, ROOM_Z / 2), color=(0.78, 0.78, 0.80))
        _bind_material(w.GetPrim(), m_wall)
    hdr = _box(stage, "/panthera_lab/shell/north_header",
               (DOOR_W + 2 * FRAME_T, WALL_T, ROOM_Z - DOOR_H),
               (door_cx, hy + WALL_T / 2, DOOR_H + (ROOM_Z - DOOR_H) / 2),
               color=(0.78, 0.78, 0.80))
    _bind_material(hdr.GetPrim(), m_wall)

    # Walls sit OUTSIDE the interior, not centred on its boundary. ROOM_X/ROOM_Y
    # are the clear space the robot can actually walk; centring the walls on the
    # boundary would make the usable floor 7.9 x 5.9 while every document said
    # 8 x 6, and the first verification pass caught exactly that (measured 8.100).
    for name, size, pos in (
        ("south_wall", (ROOM_X + 2 * WALL_T, WALL_T, ROOM_Z), (0, -hy - WALL_T / 2, ROOM_Z / 2)),
        ("west_wall", (WALL_T, ROOM_Y, ROOM_Z), (-hx - WALL_T / 2, 0, ROOM_Z / 2)),
        ("east_wall", (WALL_T, ROOM_Y, ROOM_Z), (hx + WALL_T / 2, 0, ROOM_Z / 2)),
    ):
        w = _box(stage, f"/panthera_lab/shell/{name}", size, pos,
                 color=(0.78, 0.78, 0.80))
        _bind_material(w.GetPrim(), m_wall)

    # Outside apron: floor + two low side walls, so the Mid-360 still gets returns
    # out there and `door_outside` is a real place to stand.
    apron_y0 = hy + WALL_T
    apron = _box(stage, "/panthera_lab/shell/apron_floor",
                 (APRON["sx"], APRON["sy"], WALL_T),
                 (door_cx, apron_y0 + APRON["sy"] / 2, -WALL_T / 2),
                 color=(0.40, 0.40, 0.42))
    _bind_material(apron.GetPrim(), m_floor)
    for sgn in (-1, 1):
        w = _box(stage,
                 f"/panthera_lab/shell/apron_wall_{'e' if sgn > 0 else 'w'}",
                 (WALL_T, APRON["sy"], 1.20),
                 (door_cx + sgn * APRON["sx"] / 2,
                  apron_y0 + APRON["sy"] / 2, 0.60),
                 color=(0.72, 0.72, 0.74))
        _bind_material(w.GetPrim(), m_wall)
    w = _box(stage, "/panthera_lab/shell/apron_wall_n",
             (APRON["sx"], WALL_T, 1.20),
             (door_cx, apron_y0 + APRON["sy"], 0.60), color=(0.72, 0.72, 0.74))
    _bind_material(w.GetPrim(), m_wall)

    # --- door articulation ----------------------------------------------
    door = _xform(stage, "/panthera_lab/door")
    UsdPhysics.ArticulationRootAPI.Apply(door.GetPrim())

    # Frame is the static parent. Hinge on the west edge of the opening.
    hinge_x = door_cx - DOOR_W / 2.0
    # The frame is the articulation's BASE LINK, so it is a rigid body pinned to
    # the world by a fixed joint -- not a static collider. A static collider cannot
    # be a link, and a hinge attached to one (or to the world directly) yields an
    # articulation with ZERO DOF while passing every static USD check: the joint
    # exists, the axis is Z, limits and drive are present, the root API is applied,
    # and the door still cannot move. Only a physics-stepping test finds that.
    frame = _box(stage, "/panthera_lab/door/frame",
                 (FRAME_T, WALL_T + 0.02, DOOR_H + FRAME_T),
                 (hinge_x - FRAME_T / 2, hy, (DOOR_H + FRAME_T) / 2),
                 rigid=True, mass=50.0, color=(0.45, 0.32, 0.22))
    _bind_material(frame.GetPrim(), m_wood)
    base = UsdPhysics.FixedJoint.Define(
        stage, Sdf.Path("/panthera_lab/door/frame_to_world"))
    base.CreateBody1Rel().SetTargets([frame.GetPath()])   # body0 empty = world

    leaf = _box(stage, "/panthera_lab/door/leaf",
                (DOOR_W, DOOR_T, DOOR_H),
                (hinge_x + DOOR_W / 2.0, hy, DOOR_UNDERCUT + DOOR_H / 2.0),
                rigid=True, mass=DOOR_LEAF_KG, color=(0.62, 0.45, 0.30))
    _bind_material(leaf.GetPrim(), m_wood)

    handle = _box(stage, "/panthera_lab/door/leaf_handle",
                  (0.12, 0.03, 0.03),
                  (hinge_x + DOOR_W - 0.10, hy - DOOR_T / 2 - 0.02, HANDLE_Z),
                  rigid=False, color=(0.80, 0.78, 0.35))

    hinge = UsdPhysics.RevoluteJoint.Define(
        stage, Sdf.Path("/panthera_lab/door/hinge"))
    # body0 is left EMPTY, i.e. the world, rather than pointing at the frame.
    # Pointing it at the frame passes every static USD check -- the joint exists,
    # the axis is Z, the limits and drive are there, the articulation root is
    # applied -- and then PhysX builds an articulation with ZERO DOF, because the
    # frame is a static collider with no rigid body and so cannot be a link. The
    # door rendered correctly and could not move, which is the exact shape of
    # defect this repo's audit rule exists for; only a physics-stepping test finds
    # it. See tools/test_door_articulation.py.
    hinge.CreateBody0Rel().SetTargets([frame.GetPath()])
    hinge.CreateBody1Rel().SetTargets([leaf.GetPath()])
    hinge.CreateAxisAttr("Z")
    # With body0 = world, LocalPos0 is in WORLD coordinates: the hinge line on the
    # west edge of the opening. LocalPos1 stays in the leaf's own frame, which is
    # a scaled unit cube, so its west edge is at x = -0.5.
    # Both anchors are now in their own body's local frame; both bodies are
    # scaled unit cubes, so the frame's centre is its origin and the leaf's west
    # edge is at x = -0.5.
    hinge.CreateLocalPos0Attr(Gf.Vec3f(0.5, 0.0, 0.0))
    # (leaf sits DOOR_UNDERCUT above the frame's base, so the anchors differ in z
    #  by that amount expressed in each body's own scaled-unit-cube frame)
    hinge.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0.0, 0.0))
    hinge.CreateLowerLimitAttr(0.0)
    hinge.CreateUpperLimitAttr(DOOR_LIMIT_DEG)

    drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)     # the closer's rest position
    drive.CreateStiffnessAttr(2.0)          # light closer spring
    drive.CreateDampingAttr(8.0)            # damping, so it does not slam
    drive.CreateMaxForceAttr(50.0)
    PhysxSchema.PhysxJointAPI.Apply(hinge.GetPrim())

    # --- furniture -------------------------------------------------------
    furn = _xform(stage, "/panthera_lab/furniture")

    def _slab(name, cfg, thick=0.05, color=(0.60, 0.45, 0.30)):
        top = _box(stage, f"/panthera_lab/furniture/{name}_top",
                   (cfg["sx"], cfg["sy"], thick),
                   (cfg["x"], cfg["y"], cfg["h"] - thick / 2), color=color)
        _bind_material(top.GetPrim(), m_wood)
        for sx_sign in (-1, 1):
            for sy_sign in (-1, 1):
                _box(stage,
                     f"/panthera_lab/furniture/{name}_leg_{'p' if sx_sign>0 else 'm'}"
                     f"{'p' if sy_sign>0 else 'm'}",
                     (0.06, 0.06, cfg["h"] - thick),
                     (cfg["x"] + sx_sign * (cfg["sx"] / 2 - 0.08),
                      cfg["y"] + sy_sign * (cfg["sy"] / 2 - 0.08),
                      (cfg["h"] - thick) / 2), color=(0.35, 0.26, 0.18))
        return top

    _slab("table", TABLE)
    _slab("counter", COUNTER, thick=0.06, color=(0.70, 0.70, 0.72))
    # Shelf: three boards, so the lidar sees structure at several heights.
    for i, z in enumerate((0.45, 1.05, SHELF["h"])):
        b = _box(stage, f"/panthera_lab/furniture/shelf_board_{i}",
                 (SHELF["sx"], SHELF["sy"], 0.04),
                 (SHELF["x"], SHELF["y"], z), color=(0.58, 0.44, 0.30))
        _bind_material(b.GetPrim(), m_wood)
    for sx_sign in (-1, 1):
        _box(stage,
             f"/panthera_lab/furniture/shelf_side_{'p' if sx_sign>0 else 'm'}",
             (0.04, SHELF["sy"], SHELF["h"]),
             (SHELF["x"] + sx_sign * SHELF["sx"] / 2, SHELF["y"], SHELF["h"] / 2),
             color=(0.50, 0.38, 0.26))

    # --- objects ---------------------------------------------------------
    rng = random.Random(seed)
    objs = _xform(stage, "/panthera_lab/objects")
    table_top_z = TABLE["h"]
    manifest = []

    def _place_on_table(i, n):
        """Evenly spaced along the table's long axis, jittered by the seed."""
        span = TABLE["sx"] - 0.30
        x = TABLE["x"] - span / 2 + span * (i / max(1, n - 1))
        y = TABLE["y"] + rng.uniform(-0.18, 0.18)
        return x + rng.uniform(-0.04, 0.04), y

    names = list(YCB)
    for i, name in enumerate(names):
        usd, mass, sf, df, src = YCB[name]
        p = f"/panthera_lab/objects/{name}"
        x = _xform(stage, p)
        x.GetPrim().GetReferences().AddReference(
            f"{ASSET_ROOT}/Isaac/Props/YCB/Axis_Aligned/{usd}")
        px, py = _place_on_table(i, len(names))
        xf = UsdGeom.Xformable(x)
        xf.ClearXformOpOrder()

        # Which local axis is the object's height? Found by measuring the asset,
        # not assumed, so a changed asset is caught rather than mis-posed.
        rng_yaw = rng.uniform(0.0, 360.0)
        up_axis, stand_h, half = None, None, None
        if name in UPRIGHT_HEIGHT_M:
            rr = _bbox_cache(stage).ComputeUntransformedBound(
                x.GetPrim()).ComputeAlignedRange()
            ext = [rr.GetMax()[k] - rr.GetMin()[k] for k in range(3)]
            target = UPRIGHT_HEIGHT_M[name]
            best = min(range(3), key=lambda k: abs(ext[k] - target))
            if abs(ext[best] - target) < 0.015:
                up_axis, stand_h = best, ext[best]
            else:
                print(f"  WARNING {name}: no local axis matches the published "
                      f"height {target} m (extents {ext}); left as authored")
        base_z = table_top_z + 0.002
        if up_axis is None:
            rr = _bbox_cache(stage).ComputeUntransformedBound(
                x.GetPrim()).ComputeAlignedRange()
            half = (rr.GetMax()[2] - rr.GetMin()[2]) / 2.0
        else:
            half = stand_h / 2.0
        xf.AddTranslateOp().Set(Gf.Vec3d(px, py, base_z + half))
        xf.AddRotateZOp().Set(rng_yaw)
        # Innermost op: bring the measured height axis onto +Z.
        if up_axis == 0:
            xf.AddRotateYOp().Set(90.0)
        elif up_axis == 1:
            xf.AddRotateXOp().Set(-90.0)
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(float(mass))
        m = _phys_material(stage, f"/panthera_lab/materials/{name}_mat", sf, df)
        _bind_material(x.GetPrim(), m)
        manifest.append(dict(name=name, kind="ycb", source=src, mass_kg=mass,
                             static_friction=sf, dynamic_friction=df, usd=usd,
                             stood_upright=up_axis is not None,
                             yaw_deg=round(rng_yaw, 3),
                             pos=[round(px, 4), round(py, 4),
                                  round(base_z + half, 4)]))

    for j, (name, cfg) in enumerate(PRIMS.items()):
        px, py = _place_on_table(len(names) + j, len(names) + len(PRIMS))
        p = f"/panthera_lab/objects/{name}"
        c = _box(stage, p, cfg["size"],
                 (px, py, table_top_z + cfg["size"][2] / 2 + 0.002),
                 rigid=True, mass=cfg["mass"], color=(0.85, 0.25, 0.20))
        m = _phys_material(stage, f"/panthera_lab/materials/{name}_mat",
                           cfg["sf"], cfg["df"])
        _bind_material(c.GetPrim(), m)
        manifest.append(dict(name=name, kind="authored", source="authored primitive",
                             mass_kg=cfg["mass"], static_friction=cfg["sf"],
                             dynamic_friction=cfg["df"],
                             size_m=list(cfg["size"]),
                             pos=[round(px, 4), round(py, 4),
                                  round(table_top_z + cfg["size"][2] / 2 + 0.002, 4)]))

    # The seed goes in the layer itself, so that any render or bag can be traced
    # back to the placement that produced it without a side file.
    layer = stage.GetRootLayer()
    layer.customLayerData = {
        "panthera_lab_seed": int(seed),
        "panthera_lab_room_m": ",".join(str(v) for v in (ROOM_X, ROOM_Y, ROOM_Z)),
        "panthera_lab_builder": "tools/build_lab_world.py",
    }
    layer.Save()

    with open(os.path.join(out_dir, "objects.json"), "w") as fh:
        json.dump({"seed": seed, "room_m": [ROOM_X, ROOM_Y, ROOM_Z],
                   "table": TABLE, "counter": COUNTER, "shelf": SHELF,
                   "apron": dict(APRON, y0=hy + WALL_T,
                                 y1=hy + WALL_T + APRON["sy"],
                                 x0=door_cx - APRON["sx"] / 2,
                                 x1=door_cx + APRON["sx"] / 2),
                   "door": {"w": DOOR_W, "h": DOOR_H, "leaf_kg": DOOR_LEAF_KG,
                            "limit_deg": DOOR_LIMIT_DEG, "handle_z": HANDLE_Z,
                            "hinge_x": hinge_x, "wall_y": hy},
                   "objects": manifest}, fh, indent=2)
    return usd_path


# --------------------------------------------------------------------------
def verify(usd_path: str, report_path: str = "") -> int:
    """Read the written file back. Nothing here trusts the authoring pass.

    The report is written to a FILE as well as printed, because kit swallows this
    script's stdout: the first successful build produced both artefacts and showed
    not one line of its own verification, which is indistinguishable from having
    skipped it.
    """
    fails = []
    lines = []
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"FAIL: could not open {usd_path}")
        return 1

    def chk(cond, msg):
        line = ("  ok   " if cond else "  FAIL ") + msg
        print(line)
        lines.append(line)
        if not cond:
            fails.append(msg)

    hdr = f"verifying {usd_path} ({os.path.getsize(usd_path)} bytes)"
    print(hdr)
    lines.append(hdr)
    chk(UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z, "up axis is Z")
    chk(abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9,
        "metersPerUnit is 1.0")

    # Room extent, measured from the authored geometry rather than the constants.
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    shell = stage.GetPrimAtPath("/panthera_lab/shell")
    chk(bool(shell), "/panthera_lab/shell exists")
    if shell:
        # Measured from the WALL FACES, not from the shell group's bounding box.
        # The group now contains the outside apron, so its box says the room is
        # 8.65 m deep -- true of the group, false of the room. A bounding box over
        # a group is a measurement of the group, not of the thing you meant.
        def face(path):
            return bbox.ComputeWorldBound(
                stage.GetPrimAtPath(path)).ComputeAlignedRange()

        west = face("/panthera_lab/shell/west_wall")
        east = face("/panthera_lab/shell/east_wall")
        south = face("/panthera_lab/shell/south_wall")
        north = face("/panthera_lab/shell/north_header")
        ceil = face("/panthera_lab/shell/ceiling")
        floor_r = face("/panthera_lab/shell/floor")

        clear_x = east.GetMin()[0] - west.GetMax()[0]
        clear_y = north.GetMin()[1] - south.GetMax()[1]
        clear_z = ceil.GetMin()[2] - floor_r.GetMax()[2]
        chk(abs(clear_x - ROOM_X) < 0.02,
            f"CLEAR interior X = {clear_x:.3f} m (want {ROOM_X})")
        chk(abs(clear_y - ROOM_Y) < 0.02,
            f"CLEAR interior Y = {clear_y:.3f} m (want {ROOM_Y})")
        chk(abs(clear_z - ROOM_Z) < 0.02,
            f"CLEAR ceiling height = {clear_z:.3f} m (want {ROOM_Z})")
        chk(abs((east.GetMax()[0] - west.GetMin()[0]) - (ROOM_X + 2 * WALL_T)) < 0.02,
            f"outer X = {east.GetMax()[0] - west.GetMin()[0]:.3f} m "
            f"(want {ROOM_X + 2 * WALL_T} incl. walls)")

    # Door. These checks were briefly LOST when the interior-dimension checks above
    # were rewritten and the replacement swallowed the block between them; the
    # report going from 46 lines to 16 is what gave it away. Verification code
    # needs the same read-back discipline as the thing it verifies.
    leaf = stage.GetPrimAtPath("/panthera_lab/door/leaf")
    chk(bool(leaf), "door leaf exists")
    if leaf:
        r = bbox.ComputeWorldBound(leaf).ComputeAlignedRange()
        chk(abs((r.GetMax()[0] - r.GetMin()[0]) - DOOR_W) < 0.01,
            f"door leaf width = {r.GetMax()[0]-r.GetMin()[0]:.3f} m (want {DOOR_W})")
        chk(abs((r.GetMax()[2] - r.GetMin()[2]) - DOOR_H) < 0.01,
            f"door leaf height = {r.GetMax()[2]-r.GetMin()[2]:.3f} m (want {DOOR_H})")
        chk(abs(r.GetMin()[2] - DOOR_UNDERCUT) < 0.002,
            f"door leaf clears the floor by {r.GetMin()[2]*1000:.1f} mm "
            f"(want {DOOR_UNDERCUT*1000:.0f})")
        m = UsdPhysics.MassAPI(leaf).GetMassAttr().Get()
        chk(m is not None and abs(m - DOOR_LEAF_KG) < 1e-6,
            f"door leaf mass = {m} kg (want {DOOR_LEAF_KG})")

    hinge = stage.GetPrimAtPath("/panthera_lab/door/hinge")
    chk(bool(hinge), "hinge joint exists")
    if hinge:
        j = UsdPhysics.RevoluteJoint(hinge)
        chk(j.GetAxisAttr().Get() == "Z", "hinge axis is Z")
        chk(abs(j.GetLowerLimitAttr().Get() - 0.0) < 1e-6, "hinge lower limit 0 deg")
        chk(abs(j.GetUpperLimitAttr().Get() - DOOR_LIMIT_DEG) < 1e-6,
            f"hinge upper limit {DOOR_LIMIT_DEG} deg")
        d = UsdPhysics.DriveAPI(hinge, "angular")
        chk(d.GetStiffnessAttr().Get() > 0.0, "closer spring stiffness > 0")
        chk(d.GetDampingAttr().Get() > 0.0, "hinge damping > 0")
    chk(bool(UsdPhysics.ArticulationRootAPI(
        stage.GetPrimAtPath("/panthera_lab/door"))),
        "door is an articulation root")
    if hinge:
        b0 = UsdPhysics.RevoluteJoint(hinge).GetBody0Rel().GetTargets()
        chk(bool(b0), "hinge body0 is the frame link")
        chk(bool(UsdPhysics.RigidBodyAPI(
            stage.GetPrimAtPath("/panthera_lab/door/frame"))),
            "frame is a RIGID BODY (a static collider cannot be an articulation "
            "link, and a hinge on one gives zero DOF while looking correct)")
        chk(bool(stage.GetPrimAtPath("/panthera_lab/door/frame_to_world")),
            "frame is pinned to the world by a fixed joint (fixed-base articulation)")

    ap = stage.GetPrimAtPath("/panthera_lab/shell/apron_floor")
    chk(bool(ap), "outside apron floor exists")
    if ap:
        r = bbox.ComputeWorldBound(ap).ComputeAlignedRange()
        chk(r.GetMax()[1] >= 3.60 + 0.30,
            f"apron reaches y = {r.GetMax()[1]:.3f} m, past the door_outside "
            f"waypoint at 3.60 with >=0.30 m to spare")
        chk(r.GetMin()[1] <= ROOM_Y / 2 + WALL_T + 1e-6,
            f"apron starts at y = {r.GetMin()[1]:.3f} m, flush with the wall "
            f"(no gap to step over)")

    handle = stage.GetPrimAtPath("/panthera_lab/door/leaf_handle")
    chk(bool(handle), "lever handle exists")
    if handle:
        r = bbox.ComputeWorldBound(handle).ComputeAlignedRange()
        zc = (r.GetMax()[2] + r.GetMin()[2]) / 2.0
        chk(abs(zc - HANDLE_Z) < 0.02, f"handle centre z = {zc:.3f} m (want {HANDLE_Z})")

    # Every object reference must actually compose. An unresolved reference is
    # silent in USD -- the prim exists and is empty -- which is exactly the
    # failure mode the audit rule was written for.
    for name in list(YCB) + list(PRIMS):
        p = stage.GetPrimAtPath(f"/panthera_lab/objects/{name}")
        chk(bool(p), f"object {name} exists")
        if not p:
            continue
        mass = UsdPhysics.MassAPI(p).GetMassAttr().Get()
        want = YCB[name][1] if name in YCB else PRIMS[name]["mass"]
        chk(mass is not None and abs(mass - want) < 1e-6,
            f"  {name} mass = {mass} kg (want {want})")
        if name in YCB:
            kids = [k for k in Usd.PrimRange(p)]
            chk(len(kids) > 1,
                f"  {name} reference composed ({len(kids)} prims; 1 means unresolved)")
            # UNTRANSFORMED, not world: the objects carry a seeded random yaw, and
            # the world-aligned box of a rotated object is bigger than the object.
            # Measuring the world box made every asset look 30-90 mm oversized and
            # would have been reported as "the YCB assets are the wrong scale" --
            # a fabricated defect produced entirely by the measurement.
            r = bbox.ComputeUntransformedBound(p).ComputeAlignedRange()
            dims = tuple(r.GetMax()[i] - r.GetMin()[i] for i in range(3))
            want = YCB_TRUE_SIZE_M[name]
            # Compare the SORTED extents: that tests true scale without assuming
            # which way up the asset was authored, and reports the orientation
            # separately rather than hiding it inside a size check.
            got_s, want_s = sorted(dims), sorted(want)
            err = max(abs(g - w) for g, w in zip(got_s, want_s))
            chk(err < 0.015,
                f"  {name} extents {dims[0]:.3f} x {dims[1]:.3f} x {dims[2]:.3f} m "
                f"vs YCB {want[0]:.3f} x {want[1]:.3f} x {want[2]:.3f} "
                f"(max axis error {err*1000:.0f} mm, orientation-independent)")
            if name in UPRIGHT_HEIGHT_M:
                wr = bbox.ComputeWorldBound(p).ComputeAlignedRange()
                wz = wr.GetMax()[2] - wr.GetMin()[2]
                chk(abs(wz - UPRIGHT_HEIGHT_M[name]) < 0.015,
                    f"  {name} stands {wz:.3f} m tall "
                    f"(published {UPRIGHT_HEIGHT_M[name]})")
                chk(abs(wr.GetMin()[2] - TABLE["h"]) < 0.02,
                    f"  {name} rests on the table top "
                    f"(min z {wr.GetMin()[2]:.3f}, table {TABLE['h']})")

    tail = f"{'PASS' if not fails else 'FAIL'}: {len(fails)} failed check(s)"
    print("\n" + tail)
    lines.append("")
    lines.append(tail)
    if report_path:
        with open(report_path, "w") as fh:
            fh.write("\n".join(lines) + "\n")
    return 0 if not fails else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/workspace/ferox_isaac/assets/worlds/panthera_lab")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--verify-only", action="store_true")
    ap.add_argument("--report", default="", help="also write the check list here")
    a = ap.parse_args()
    usd = os.path.join(a.out, "panthera_lab.usd")
    if not a.verify_only:
        usd = build(a.out, a.seed)
        print(f"wrote {usd}")
    return verify(usd, a.report)


if __name__ == "__main__":
    _rc = main()
    _app.close()
    raise SystemExit(_rc)
