"""Create the twin's sensor DEVICES and set their intrinsics from the contract.

The calibrated FRAMES are authored into the USD sensor layer by
tools/build_twin_assets.py. This module only instantiates devices as identity
children of those frames, so there is exactly one place a pose can come from.

Nothing here calls sim_utils.ensure_link_xform(). That helper does
ClearXformOpOrder() then re-adds ops with the caller's values, so calling it on an
authored frame would silently replace the calibrated pose with a placeholder in
the session layer while the file on disk still read correctly. Devices go in at
identity; the frame supplies the geometry.
"""

from __future__ import annotations

import math
from typing import Any, Dict

import lidar as twin_lidar  # isaac/twin/lidar.py


class SensorAuthoringError(RuntimeError):
    """A sensor that is not what the contract says. Always fatal."""


def _sensor(contract: Dict[str, Any], name: str) -> Dict[str, Any]:
    for s in contract.get("sensors", []):
        if s["name"] == name:
            return s
    raise SensorAuthoringError(f"contract has no sensor {name!r}")


def _topic(contract: Dict[str, Any], name: str) -> Dict[str, Any]:
    for t in contract.get("topics", []):
        if t["name"] == name:
            return t
    raise SensorAuthoringError(f"contract has no topic {name!r}")


# --------------------------------------------------------------- camera


def set_camera_intrinsics(prim, K, width: int, height: int) -> Dict[str, float]:
    """Drive focal length and aperture from the contract's K, then read K back.

    Isaac's camera is specified as aperture + focal length; ROS wants K. The
    relations are

        fx = focalLength * width  / horizontalAperture
        fy = focalLength * height / verticalAperture
        cx = width/2  + horizontalApertureOffset * width  / horizontalAperture
        cy = height/2 + verticalApertureOffset   * height / verticalAperture

    Solving from K rather than from a field of view is the whole point: the sim's
    previous CAMERA_HFOV_DEG=69 gives fx = 931.2 px at 1280 wide, which misses the
    contract's 908 by 2.6 % and would fail the audit's 1 % K tolerance. FOV is a
    derived quantity here, not an input.
    """
    from pxr import UsdGeom

    fx, fy = float(K[0]), float(K[4])
    cx, cy = float(K[2]), float(K[5])
    if fx <= 0 or fy <= 0:
        raise SensorAuthoringError(f"contract K has non-positive focal lengths: fx={fx} fy={fy}")

    cam = UsdGeom.Camera(prim)
    h_ap = float(cam.GetHorizontalApertureAttr().Get() or 20.955)

    focal = fx * h_ap / width
    v_ap = fy and (focal * height / fy)

    cam.GetFocalLengthAttr().Set(focal)
    cam.GetHorizontalApertureAttr().Set(h_ap)
    cam.GetVerticalApertureAttr().Set(v_ap)
    cam.GetHorizontalApertureOffsetAttr().Set((cx - width / 2.0) * h_ap / width)
    cam.GetVerticalApertureOffsetAttr().Set((cy - height / 2.0) * v_ap / height)

    got = read_back_K(prim, width, height)
    for label, want, have in (("fx", fx, got["fx"]), ("fy", fy, got["fy"]),
                              ("cx", cx, got["cx"]), ("cy", cy, got["cy"])):
        # cx/cy at the exact centre make the relative test degenerate, so compare
        # those in pixels instead.
        if label in ("cx", "cy"):
            if abs(have - want) > 0.5:
                raise SensorAuthoringError(
                    f"{prim.GetPath()}: {label} read back {have:.4f}, contract {want:.4f}")
        elif abs(have - want) / want > 0.01:
            raise SensorAuthoringError(
                f"{prim.GetPath()}: {label} read back {have:.4f}, contract {want:.4f} "
                f"({abs(have - want) / want:.2%} > 1% tolerance)")
    return got


def read_back_K(prim, width: int, height: int) -> Dict[str, float]:
    from pxr import UsdGeom
    cam = UsdGeom.Camera(prim)
    focal = float(cam.GetFocalLengthAttr().Get())
    h_ap = float(cam.GetHorizontalApertureAttr().Get())
    v_ap = float(cam.GetVerticalApertureAttr().Get())
    h_off = float(cam.GetHorizontalApertureOffsetAttr().Get() or 0.0)
    v_off = float(cam.GetVerticalApertureOffsetAttr().Get() or 0.0)
    return {
        "fx": focal * width / h_ap,
        "fy": focal * height / v_ap,
        "cx": width / 2.0 + h_off * width / h_ap,
        "cy": height / 2.0 + v_off * height / v_ap,
        "hfov_deg": 2.0 * math.degrees(math.atan(h_ap / (2.0 * focal))),
        "vfov_deg": 2.0 * math.degrees(math.atan(v_ap / (2.0 * focal))),
    }


def create_camera(contract: Dict[str, Any], robot_root: str):  # noqa: C901
    """Create the D435i colour camera at the authored optical frame.

    ONE camera prim, at camera_color_optical_frame. Depth is rendered from this
    same prim, which is what makes "aligned_depth_to_color" true by construction
    rather than by a registration step: same viewpoint, same projection, same
    stamp. A second prim at the depth frame would produce a depth image that is
    NOT aligned to the colour one, while still being published on a topic whose
    name promises it is.
    """
    import numpy as np
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.prims import is_prim_path_valid

    cam_spec = _sensor(contract, "d435i")
    mp = cam_spec["model_params"]
    width, height = mp["color_resolution"]

    optical = (f"{robot_root}/{cam_spec['parent_link']}/camera_link"
               f"/camera_color_frame/camera_color_optical_frame")
    if not is_prim_path_valid(optical):
        raise SensorAuthoringError(
            f"{optical} is not in the stage. The sensor USD layer was not composed -- "
            "run scripts/08_build_twin_assets.sh and check the Sensor variant is 'Sensors'.")

    # The optical frame is an Xform carrying the REP-103 optical convention
    # (+x right, +y down, +z forward). A USD camera looks along -Z with +Y up. So
    # the camera prim is a child rotated 180 degrees about X: that maps ROS +Z
    # forward onto USD -Z forward and ROS +y down onto USD +y up. Placing the
    # camera at identity here instead would point it BACKWARDS -- and it would
    # still publish happily on the right topic in the right frame, which is
    # exactly the kind of wrong that survives every string-level check.
    path = f"{optical}/camera"
    cam = Camera(
        prim_path=path, name="d435i_color", resolution=(width, height),
        translation=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([0.0, 1.0, 0.0, 0.0]),  # wxyz: 180 deg about X
    )
    cam.initialize()
    # MinZ from the datasheet; far clip past clip_distance so the converter, not
    # the renderer, is what enforces the contract's 4.0 m Z clip.
    cam.set_clipping_range(mp["min_z_m"], 100.0)
    cam.add_distance_to_image_plane_to_frame()

    ci = _topic(contract, f"{contract['robot']['namespace']}/camera/color/camera_info")
    K = ci["expect"]["camera_info"]["K"]
    got = set_camera_intrinsics(cam.prim, K, width, height)
    return cam, got


# --------------------------------------------------------------- lidar


def create_lidar(contract: Dict[str, Any], robot_root: str):
    """Create the Mid-360 as an identity child of the authored livox_frame."""
    from isaacsim.core.utils.prims import is_prim_path_valid

    spec = _sensor(contract, "livox_mid360")
    frame = f"{robot_root}/{spec['parent_link']}/livox_frame"
    if not is_prim_path_valid(frame):
        raise SensorAuthoringError(
            f"{frame} is not in the stage -- run scripts/08_build_twin_assets.sh")
    return twin_lidar.create_mid360(
        frame, "mid360", spec["model_params"],
        translation=(0.0, 0.0, 0.0), orientation=(1.0, 0.0, 0.0, 0.0))


# --------------------------------------------------------------- imu


def create_imu(contract: Dict[str, Any], robot_root: str):
    """Create the body IMU as an identity child of the authored dog_imu_link."""
    import numpy as np
    from isaacsim.sensors.physics import IMUSensor
    from isaacsim.core.utils.prims import is_prim_path_valid

    spec = _sensor(contract, "dog_imu")
    parent = spec["parent_link"]
    if parent == contract["robot"]["base_frame"]:
        parent = "pelvis"  # base_link IS pelvis on the G1
    path = f"{robot_root}/{parent}/dog_imu_link/imu"
    frame = f"{robot_root}/{parent}/dog_imu_link"
    if not is_prim_path_valid(frame):
        raise SensorAuthoringError(
            f"{frame} is not in the stage -- run scripts/08_build_twin_assets.sh")

    rate = _topic(contract, f"{contract['robot']['namespace']}/imu/data")["rate_hz"]
    return IMUSensor(
        prim_path=path, name="dog_imu", frequency=int(round(rate)),
        translation=np.array([0.0, 0.0, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
