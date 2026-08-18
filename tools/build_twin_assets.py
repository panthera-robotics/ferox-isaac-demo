#!/usr/bin/env python3
"""Author the twin sensor USD layer from <robot>_contract.yaml.

Runs INSIDE the Isaac Sim container (it needs pxr). Use
scripts/08_build_twin_assets.sh, which supplies the right PYTHONPATH/LD_LIBRARY_PATH.

WHAT THIS WRITES
----------------
Only isaac/assets/<robot>/usd/configuration/<name>_sensor.usd -- the layer the
asset already composes through its Sensor='Sensors' variant and which ships as an
empty one-prim stub. The base and physics layers are NEVER touched: the physics
layer is the tuned one the campaign says to reuse verbatim, and it is also the
only layer holding mass and inertia.

WHAT IT AUTHORS
---------------
The calibrated sensor FRAMES, and nothing else:

    over torso_link
      livox_frame              calibrated, waist-independent
        livox_imu              Livox datasheet offset
      camera_link              URDF-nominal D435i mount
        camera_color_frame  -> camera_color_optical_frame
        camera_depth_frame  -> camera_depth_optical_frame
    over pelvis                (pelvis IS base_link on this robot)
      dog_imu_link             standing residual

The sensor DEVICES (OmniLidar, Camera, IMU) are created at runtime as identity
children of these frames, because they need omni.kit.commands. That split is
deliberate: geometry lives in the asset where it is diffable and auditable, and
only the device instantiation is runtime.

WHY FRAMES IN USD AT ALL
------------------------
Because the alternative silently loses. isaac/sim_utils.py's ensure_link_xform()
does ClearXformOpOrder() then re-adds ops with the CALLER's values, so a runtime
call against an already-authored prim overwrites the calibrated pose in the
session layer while the file on disk still reads correctly. Authoring the frames
here and deleting the runtime placement is the only version where the pose on
disk is the pose in the stage.
"""

from __future__ import annotations

import math
import os
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)

# Two layouts. On the host this file is <repo>/tools/build_twin_assets.py. Inside
# the sim container the repo is split across two read-only-ish mounts:
# tools/ -> /workspace/ferox_tools and isaac/ -> /workspace/ferox_isaac. Resolve
# the isaac/ root explicitly rather than assuming ../isaac.
if os.path.isdir("/workspace/ferox_isaac"):
    ISAAC_ROOT = "/workspace/ferox_isaac"
else:
    ISAAC_ROOT = os.path.join(os.path.dirname(TOOLS), "isaac")
REPO = os.path.dirname(ISAAC_ROOT)

import twin_contract  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

SENSOR_LAYERS = {
    "g1": "assets/g1/usd/configuration/g1_29dof_rev_1_0_sensor.usd",
    "go2": "assets/go2/usd/configuration/go2_description_sensor.usd",
}
# The robot's base frame is this USD prim. On the G1 the ROS base_link IS the
# pelvis -- the driver relabels /state_estimator/odom_pelvis's child -- so a
# sensor whose contract parent_link is base_link is authored under pelvis.
BASE_PRIM = {"g1": "pelvis", "go2": "base"}


def _frames_from_contract(contract):
    """Contract tf_static -> parent-keyed tree of (child, xyz, rpy, gated)."""
    by_parent = {}
    for e in contract.get("tf_static", []):
        by_parent.setdefault(e["parent"], []).append(e)
    return by_parent


def _author(stage, parent_path, edge):
    """Author one frame as an Xform child, translate + orient (quatd).

    translate+orient matches the op vocabulary every existing prim in this asset
    uses, and avoids Euler-order ambiguity: UsdGeom rotateXYZ is not the same
    convention as ROS rpy in general, and 'they agree for small angles' is not a
    property to rely on for a 179-degree inverted lidar mount.
    """
    path = f"{parent_path}/{edge['child']}"
    prim = stage.DefinePrim(path, "Xform")
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*edge["xyz"]))
    qx, qy, qz, qw = twin_contract.quat_from_rpy(*edge["rpy"])
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(qw, Gf.Vec3d(qx, qy, qz)))
    prim.SetCustomDataByKey("twin:provenance", edge["provenance"])
    prim.SetCustomDataByKey("twin:source", edge["source"].strip().replace("\n", " "))
    return path


def _out_path(robot: str) -> str:
    """Where to write the sensor layer.

    Inside the sim container Isaac runs as UID 1234 while the repo files are
    root-owned, so a direct write to the mounted asset fails. TWIN_ASSET_OUT_DIR
    lets the wrapper stage the layer somewhere writable and copy it back, rather
    than chowning repo files out from under git.
    """
    staged = os.environ.get("TWIN_ASSET_OUT_DIR")
    real = os.path.join(ISAAC_ROOT, SENSOR_LAYERS[robot])
    if staged:
        return os.path.join(staged, os.path.basename(real))
    return real


def build(robot: str) -> str:
    contract = twin_contract.load(os.path.join(ISAAC_ROOT, "twin", f"{robot}_contract.yaml"))
    stub = os.path.join(ISAAC_ROOT, SENSOR_LAYERS[robot])
    if not os.path.exists(stub):
        raise SystemExit(f"sensor layer stub missing: {stub}")
    out = _out_path(robot)
    if out != stub:
        import shutil
        shutil.copyfile(stub, out)

    stage = Usd.Stage.Open(out)
    default_name = stage.GetDefaultPrim().GetName()
    root = f"/{default_name}"

    # Wipe only what we author, so re-running is idempotent and a removed sensor
    # actually disappears instead of lingering from a previous build.
    for child in list(stage.GetPrimAtPath(root).GetChildren()):
        stage.RemovePrim(child.GetPath())

    by_parent = _frames_from_contract(contract)
    base_prim = BASE_PRIM[robot]
    authored = []

    # Map a contract parent frame to its USD prim path.
    def usd_parent(frame_name: str) -> str:
        if frame_name == contract["robot"]["base_frame"]:
            return f"{root}/{base_prim}"
        for p in authored:
            if p.rsplit("/", 1)[-1] == frame_name:
                return p
        # A sensor whose parent is a link (e.g. torso_link) that exists in the
        # base layer: author an `over` on it.
        return f"{root}/{frame_name}"

    # Sensors declare their true parent_link (torso_link for the G1 head), which
    # tf_static cannot express -- the hardware edge is base_link->livox_frame
    # because the driver composes the waist chain. In the USD the frame must hang
    # off the link it is physically bolted to, or it will not move with the waist.
    sensor_parent = {s["name"]: s["parent_link"] for s in contract.get("sensors", [])}
    frame_of_sensor = {"livox_mid360": "livox_frame", "d435i": "camera_link",
                       "dog_imu": "dog_imu_link", "livox_imu": "livox_imu"}

    sensor_pose = {s["name"]: s["pose"] for s in contract.get("sensors", [])}

    def placement(child_frame: str, edge):
        """Where a frame hangs, and with WHICH pose.

        The two are coupled and getting it wrong is silent. tf_static states the
        hardware edge base_link->livox_frame, whose value is the STANDING
        COMPOSITE of the waist chain with the mount. The USD must instead hang the
        frame off the link it is physically bolted to (torso_link) with the
        WAIST-INDEPENDENT mount pose from the sensors block -- otherwise the frame
        does not move with the waist, and the composite is applied twice.

        Returns (usd_parent_path, edge_with_the_right_pose).
        """
        for sname, fname in frame_of_sensor.items():
            if fname != child_frame or sname not in sensor_parent:
                continue
            pl = sensor_parent[sname]
            if pl == edge["parent"]:
                break  # tf_static parent already IS the mount link; values agree
            pose = sensor_pose[sname]
            redirected = dict(edge)
            redirected["xyz"] = pose["xyz"]
            redirected["rpy"] = pose["rpy"]
            redirected["provenance"] = pose["provenance"]
            redirected["source"] = pose["source"]
            usd_p = (f"{root}/{base_prim}" if pl == contract["robot"]["base_frame"]
                     else f"{root}/{pl}")
            return usd_p, redirected
        return usd_parent(edge["parent"]), edge

    # Breadth-first so a parent frame exists before its children.
    pending = [contract["robot"]["base_frame"]]
    seen = set()
    while pending:
        parent = pending.pop(0)
        if parent in seen:
            continue
        seen.add(parent)
        for edge in by_parent.get(parent, []):
            if edge.get("default_published") is False:
                # Gated on hardware (camera_tf_enable / CAMERA_TF=1). The FRAME is
                # still authored -- the camera has to hang somewhere and its pose is
                # known -- but the sim only PUBLISHES the TF edge when enabled, so
                # camera_link stays an orphan root on the wire exactly like the robot.
                pass
            usd_p, placed = placement(edge["child"], edge)
            p = _author(stage, usd_p, placed)
            authored.append(p)
            pending.append(edge["child"])

    stage.GetRootLayer().Save()
    return out, authored


def verify(robot: str) -> None:
    """Read the layer back and assert every pose equals the contract.

    Rule 7: a generated artifact is not trusted because it was generated.
    """
    contract = twin_contract.load(os.path.join(ISAAC_ROOT, "twin", f"{robot}_contract.yaml"))
    out = _out_path(robot)
    stage = Usd.Stage.Open(out)
    # Compare against the pose actually authored: for a redirected sensor frame
    # that is the sensors-block mount pose, not the tf_static composite.
    sensor_pose = {s["name"]: s["pose"] for s in contract.get("sensors", [])}
    sensor_parent = {s["name"]: s["parent_link"] for s in contract.get("sensors", [])}
    frame_of_sensor = {"livox_mid360": "livox_frame", "d435i": "camera_link",
                       "dog_imu": "dog_imu_link", "livox_imu": "livox_imu"}
    want = {}
    for e in contract.get("tf_static", []):
        placed = e
        for sname, fname in frame_of_sensor.items():
            if fname == e["child"] and sname in sensor_parent \
                    and sensor_parent[sname] != e["parent"]:
                placed = dict(e)
                placed["xyz"] = sensor_pose[sname]["xyz"]
                placed["rpy"] = sensor_pose[sname]["rpy"]
        want[e["child"]] = placed
    found = 0
    for prim in stage.Traverse():
        name = prim.GetName()
        if name not in want:
            continue
        found += 1
        e = want[name]
        ops = UsdGeom.Xformable(prim).GetOrderedXformOps()
        t = [o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeTranslate][0].Get()
        q = [o for o in ops if o.GetOpType() == UsdGeom.XformOp.TypeOrient][0].Get()
        for i, (a, b) in enumerate(zip((t[0], t[1], t[2]), e["xyz"])):
            if abs(a - b) > 1e-9:
                raise SystemExit(f"{robot} {name}: xyz[{i}] {a} != contract {b}")
        im = q.GetImaginary()
        got = (im[0], im[1], im[2], q.GetReal())
        wq = twin_contract.quat_from_rpy(*e["rpy"])
        d = twin_contract.angular_distance(got, wq)
        if d > 1e-9:
            raise SystemExit(f"{robot} {name}: orientation off by {d:.3e} rad from contract")
    if found != len(want):
        missing = set(want) - {p.GetName() for p in stage.Traverse()}
        raise SystemExit(f"{robot}: authored {found}/{len(want)} frames, missing {sorted(missing)}")
    print(f"  verified {found}/{len(want)} frames against the contract")


# ---------------------------------------------------------------- hands
#
# Mount offsets. Campaign section 8 Q3: Unitree does not publish a Dex5 adapter
# transform, so the DEFAULT is the Dex3 flange offset from g1_29dof_with_hand,
# flagged `assumed`. Verbatim from that URDF:
#     left_hand_palm_joint   origin xyz="0.0415  0.003 0" rpy="0 0 0"
#     right_hand_palm_joint  origin xyz="0.0415 -0.003 0" rpy="0 0 0"
# The y offset is MIRRORED, which is why both are written out rather than one
# being derived from the other.
HAND_FLANGE = {
    "left":  {"parent": "left_wrist_yaw_link",  "xyz": (0.0415, 0.003, 0.0)},
    "right": {"parent": "right_wrist_yaw_link", "xyz": (0.0415, -0.003, 0.0)},
}
# The hand USD's own root link, from Unitree's URDF. Note the LEFT file suffixes it
# and the RIGHT file does not -- an upstream naming inconsistency, not a typo here.
HAND_PALM_LINK = {"left": "base_link00L", "right": "base_link00"}
HAND_VARIANTS = {
    # variant name -> {side: path relative to ISAAC_ROOT}. Kept anchored at the repo
    # root, NOT relative to the layer: the layer is authored in a staging dir and
    # installed into usd/configuration/, so a path written relative to "where the
    # layer is" resolves against the wrong directory in one of those two places.
    # _hand_ref() converts to a layer-relative path against the INSTALL location.
    "dex5_1p": {"left": "assets/hands/dex5_1p/dex5_1p_l.usd",
                "right": "assets/hands/dex5_1p/dex5_1p_r.usd"},
}


def _hand_ref(variant: str, side: str, install_dir: str) -> str:
    """Layer-relative reference to a hand asset, verified to resolve.

    USD resolves a relative reference against the layer that AUTHORS it. Returning
    a path here without checking it is how the hand silently composes to an empty
    Xform: the mount transform is still exact, the flange joint is still there, and
    only the mass and joint count betray that nothing arrived.
    """
    target = os.path.join(ISAAC_ROOT, HAND_VARIANTS[variant][side])
    if not os.path.exists(target):
        raise SystemExit(f"hand asset missing: {target}  (run scripts/11_import_dex5.sh)")
    rel = os.path.relpath(target, install_dir)
    if not os.path.exists(os.path.normpath(os.path.join(install_dir, rel))):
        raise SystemExit(f"hand reference {rel!r} does not resolve from {install_dir}")
    return rel


def build_hands(robot: str, variant: str = "dex5_1p"):
    """Author the hand layer and add a `Hand` variant set to the robot's stub.

    Mirrors the asset's existing Physics/Sensor pattern exactly: a payload per
    variant, plus a `None` variant that carries nothing. `None` is what DT4's
    `hand=none` selects and what keeps the bare-wristed robot reproducible.

    The hand is REFERENCED, never merged. Unitree's file stays the single source of
    the geometry, masses and limits, so re-importing it cannot silently diverge from
    what is mounted.
    """
    if variant not in HAND_VARIANTS:
        raise SystemExit(f"unknown hand variant {variant!r}; known: {sorted(HAND_VARIANTS)}")
    usd_dir = os.path.dirname(os.path.join(ISAAC_ROOT, SENSOR_LAYERS[robot]))
    usd_dir = os.path.dirname(usd_dir)                      # .../usd
    stub = os.path.join(usd_dir, f"{robot}.usd")
    layer_name = f"{os.path.basename(SENSOR_LAYERS[robot])}".replace(
        "_sensor.usd", f"_hands_{variant}.usd")

    staged = os.environ.get("TWIN_ASSET_OUT_DIR")
    install_dir = os.path.join(usd_dir, "configuration")
    layer_path = os.path.join(staged or install_dir, layer_name)
    stub_out = os.path.join(staged, f"{robot}.usd") if staged else stub

    # --- the hand layer -----------------------------------------------------
    # Author FRESH, never open-and-edit. USD list-ops append: re-running an
    # open-and-edit build left the previous (broken) reference in place alongside
    # the new one, and the composed result kept resolving to the stale path. This
    # layer is 100% generated, so regenerating it is always the correct semantics.
    if os.path.exists(layer_path):
        os.remove(layer_path)
    hl = Usd.Stage.CreateNew(layer_path)
    # Hold the stage. Usd.Stage.Open(...).GetDefaultPrim() frees the stage as soon
    # as the temporary goes out of scope, and the prim expires with it -- the error
    # is "Accessed invalid expired prim", several lines from the real cause.
    stub_stage = Usd.Stage.Open(stub)
    default_name = stub_stage.GetDefaultPrim().GetName()
    root = hl.OverridePrim(f"/{default_name}")
    hl.SetDefaultPrim(root)
    for side, spec in HAND_FLANGE.items():
        parent = hl.OverridePrim(f"/{default_name}/{spec['parent']}")
        node = hl.DefinePrim(f"{parent.GetPath()}/{variant}_{side}", "Xform")
        node.GetReferences().SetReferences([Sdf.Reference(_hand_ref(variant, side, install_dir))])
        x = UsdGeom.Xformable(node)
        x.ClearXformOpOrder()
        x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*spec["xyz"]))
        # A reference alone does not ATTACH the hand -- it places a separate
        # articulation next to the wrist, and PhysX drops it on the floor the moment
        # the sim steps. The flange is a fixed joint in the URDF
        # (left_hand_palm_joint, type="fixed"), so it is a fixed joint here.
        palm = f"{node.GetPath()}/{HAND_PALM_LINK[side]}"
        jpath = f"/{default_name}/joints/{variant}_{side}_flange"
        joint = UsdPhysics.FixedJoint.Define(hl, jpath)
        joint.CreateBody0Rel().SetTargets([f"/{default_name}/{spec['parent']}"])
        joint.CreateBody1Rel().SetTargets([palm])
        # localPos0 carries the flange offset; the hand's own root sits at identity
        # in its frame, so localPos1 is zero.
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*spec["xyz"]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        # Each imported hand carries its OWN ArticulationRootAPI, because it was
        # imported as a standalone robot. PhysX honours that: it builds three
        # articulations (pelvis + two palms) and the flange joint above becomes a
        # link BETWEEN articulations, not a link within one. The stage still shows
        # all 40 finger joints and the correct 2.003615 kg -- but the G1's
        # articulation reports 29 DOF, so nothing can command a finger. Deleting
        # the API on the palms is what merges the fingers into the G1's own
        # articulation, which is the whole point of mounting them.
        palm_over = hl.OverridePrim(palm)
        palm_over.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        for schema in ("PhysxArticulationAPI",):
            palm_over.RemoveAppliedSchema(schema)
        node.SetCustomDataByKey("twin:provenance", "assumed")
        node.SetCustomDataByKey(
            "twin:source",
            "Dex3 flange offset from unitree_ros g1_29dof_with_hand_rev_1_0.urdf "
            f"({spec['parent']} -> {side}_hand_palm_link). Unitree publishes no Dex5 "
            "adapter transform; campaign section 8 Q3 default, flagged assumed.")
    hl.GetRootLayer().Save()

    # --- the variant set on the stub ---------------------------------------
    if stub_out != stub:
        import shutil
        shutil.copyfile(stub, stub_out)
    st = Usd.Stage.Open(stub_out)
    prim = st.GetDefaultPrim()
    vsets = prim.GetVariantSets()
    vs = vsets.AddVariantSet("Hand")
    for name in ("None", variant):
        if name not in vs.GetVariantNames():
            vs.AddVariant(name)
    vs.SetVariantSelection(variant)
    with vs.GetVariantEditContext():
        prim.GetPayloads().AddPayload(f"configuration/{layer_name}")
    vs.SetVariantSelection(variant)
    st.GetRootLayer().Save()
    return layer_path, stub_out, vs.GetVariantNames()


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--hands" in sys.argv:
        robot = args[0] if args else "g1"
        variant = args[1] if len(args) > 1 else "dex5_1p"
        layer, stub, names = build_hands(robot, variant)
        print(f"{robot}: hand layer {layer}")
        print(f"{robot}: stub {stub}  Hand variants {names}")
        return 0
    robots = args or ["g1"]
    for robot in robots:
        if robot not in SENSOR_LAYERS:
            raise SystemExit(f"unknown robot {robot!r}; known: {sorted(SENSOR_LAYERS)}")
        out, authored = build(robot)
        print(f"{robot}: wrote {out}")
        for p in authored:
            print(f"    {p}")
        verify(robot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
