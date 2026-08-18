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

from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

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


def main() -> int:
    robots = sys.argv[1:] or ["g1"]
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
