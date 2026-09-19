# PANTHERA (Sprint M, wb-cand-M03): headless frame capture of the running deployment sim. No-op unless G1_CAPTURE_DIR is
# set. A chase camera prim (/World/ChaseCam) + a replicator render product + the rgb annotator; after every render step the
# camera is moved to a 3/4 chase position derived from the pelvis pose (same geometry as viewport_follow.py) and every
# G1_CAPTURE_EVERY-th render step the rgb frame is written as PNG with its index <-> sim time in frames.jsonl.
# Reads the stage and robot pose; never writes physics, decimation, gains or the policy path. Never raises into the loop.
import json
import math
import os

_S = {"err": False, "tick": 0, "frame": 0, "every": 3, "annot": None, "cam": None, "f": None, "dir": None,
      # chase geometry: metres behind / to the left / above the pelvis, look-at height, focal length (mm). v1 used
      # 3.0/1.2/1.8/0.5/24 (feet cut off); G1_CAPTURE_CAM="back,left,up,target_z,focal" overrides.
      "cam_geom": (2.8, 1.6, 1.0, 0.35, 18.0)}


def enabled() -> bool:
    return bool(os.environ.get("G1_CAPTURE_DIR", "").strip())


def setup(world, width: int = 1280, height: int = 720) -> None:
    if not enabled() or _S["err"]:
        return
    try:
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom

        _S["every"] = max(1, int(os.environ.get("G1_CAPTURE_EVERY", "3")))
        _S["max_frames"] = int(os.environ.get("G1_CAPTURE_MAX_FRAMES", "0") or 0)   # Sprint P disk cap: stop writing after this many frames (0 = no cap)
        geom = os.environ.get("G1_CAPTURE_CAM", "").strip()
        if geom:
            _S["cam_geom"] = tuple(float(v) for v in geom.split(","))
        res = os.environ.get("G1_CAPTURE_RES", "").strip()      # optional WxH (Sprint P: 960x540 keeps a long mission capture under the disk cap)
        if res:
            width, height = (int(v) for v in res.lower().split("x"))
        _S["dir"] = os.path.join(os.environ["G1_CAPTURE_DIR"], "frames")
        os.makedirs(_S["dir"], exist_ok=True)
        cam = UsdGeom.Camera.Define(world.stage, "/World/ChaseCam")
        cam.CreateFocalLengthAttr(float(_S["cam_geom"][4]))
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 200.0))
        _S["cam"] = cam
        rp = rep.create.render_product("/World/ChaseCam", (width, height))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach([rp])
        _S["annot"] = annot
        _S["f"] = open(os.path.join(os.environ["G1_CAPTURE_DIR"], "frames.jsonl"), "w", encoding="utf-8")
        print(f"[capture] ON -> {_S['dir']} ({width}x{height}), every {_S['every']} render steps, cam {_S['cam_geom']}", flush=True)
    except Exception as exc:  # capture is optional: report and disable
        _S["err"] = True
        print(f"[capture] setup failed, capture disabled: {exc!r}", flush=True)


def _look_at(eye, target):
    from pxr import Gf

    f = (target - eye).GetNormalized()
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    r = Gf.Cross(f, up).GetNormalized()
    u = Gf.Cross(r, f)
    # camera looks down its local -Z, +Y up: columns = right, up, -forward
    m = Gf.Matrix4d(
        r[0], r[1], r[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -f[0], -f[1], -f[2], 0.0,
        eye[0], eye[1], eye[2], 1.0,
    )
    return m


def maybe_step(runner) -> None:
    if _S["err"] or _S["annot"] is None:
        return
    try:
        import numpy as np
        from pxr import Gf, UsdGeom

        pos, quat = runner._robot.robot.get_world_pose()
        pos = np.asarray(pos, float)
        w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        back = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        b, l, u, tz, _ = _S["cam_geom"]
        eye = pos + (-b) * back + l * left + np.array([0.0, 0.0, u])
        target = pos + np.array([0.0, 0.0, tz])
        xf = UsdGeom.Xformable(_S["cam"].GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(_look_at(Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*target.tolist())))
        _S["tick"] += 1
        if _S["tick"] % _S["every"] != 0:
            return
        if _S["max_frames"] and _S["frame"] >= _S["max_frames"]:
            if not _S.get("capped"):
                _S["capped"] = True; print(f"[capture] frame cap {_S['max_frames']} reached; no more frames", flush=True)
            return
        data = _S["annot"].get_data()
        if data is None or getattr(data, "size", 0) == 0:
            return
        arr = np.asarray(data)
        if arr.ndim == 3 and arr.shape[2] == 4:
            arr = arr[:, :, :3]
        name = f"frame_{_S['frame']:06d}.png"
        path = os.path.join(_S["dir"], name)
        try:
            from PIL import Image

            Image.fromarray(arr.astype(np.uint8)).save(path)
        except Exception:
            import imageio

            imageio.imwrite(path, arr.astype(np.uint8))
        sim_t = None
        try:
            sim_t = float(runner._world.current_time)
        except Exception:
            pass
        _S["f"].write(json.dumps({"frame": _S["frame"], "file": name, "render_tick": _S["tick"], "sim_time": sim_t,
                                  "robot_pos": [round(float(v), 4) for v in pos], "robot_yaw": round(yaw, 4)}) + "\n")
        _S["f"].flush()
        _S["frame"] += 1
    except Exception as exc:
        _S["err"] = True
        print(f"[capture] step failed, capture disabled: {exc!r}", flush=True)
