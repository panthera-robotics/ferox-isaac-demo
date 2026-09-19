# PANTHERA world lane (Sprint N, candidate wb-cand-N-world-capture): headless multi-camera capture of the running
# deployment sim from the hero world's authored camera rigs. Same interface as the seam tree's capture_frames.py
# (enabled / setup / maybe_step) so the wholebody lane can swap it in behind its existing G1_CAPTURE_DIR gate.
# No-op unless G1_CAPTURE_DIR and G1_CAPTURE_CAMS are set.
#
#   G1_CAPTURE_CAMS   comma list of camera prim paths in the loaded stage, e.g.
#                     /World/Env/Cameras/A_corridor_side,/World/Env/Cameras/B_hand_closeup,/World/Env/Cameras/A_chase_34
#   G1_CAPTURE_EVERY  write every N-th render step (default 3)
#   G1_CAPTURE_RES    optional WxH override for every product (default: the camera's panthera:resolution customData)
#   G1_RENDER_PRESET  optional path to a JSON {"kit_settings": {"/rtx/...": value}} applied through carb.settings
#                     BEFORE the render products exist (render-only; never a physics setting — keys must start with /rtx/)
#
# Camera modes come from the camera prim's customData (authored by tools/build_world.py):
#   panthera:mode = "static"  -> the authored pose is used as is
#   panthera:mode = "track"   -> re-posed every render step: panthera:track = pelvis | right_hand | nib,
#                                panthera:offset_m (world-frame offset from the tracked point, yaw-rotated for pelvis),
#                                panthera:look_at_offset_m
#   panthera:mode = "orbit"   -> the authored time samples are stepped at panthera:orbit fps (render-step counter)
# Reads the stage and robot pose; never writes physics, decimation, gains or the policy path. Never raises into the loop.
import json
import math
import os

_S = {"err": False, "tick": 0, "frame": 0, "every": 3, "cams": [], "f": None, "dir": None}
TRACK_LINKS = {"right_hand": ("right_wrist_yaw_link", "right_rubber_hand", "right_hand_base_link"), "nib": ("/World/Marker/Nib", "nib")}


def enabled() -> bool:
    return bool(os.environ.get("G1_CAPTURE_DIR", "").strip()) and bool(os.environ.get("G1_CAPTURE_CAMS", "").strip())


def _apply_preset():
    path = os.environ.get("G1_RENDER_PRESET", "").strip()
    if not path:
        return
    try:
        import carb.settings
        cfg = json.load(open(path))
        s = carb.settings.get_settings()
        n = 0
        for k, v in cfg.get("kit_settings", {}).items():
            if not k.startswith("/rtx/"):
                print(f"[capture] preset key refused (not /rtx/): {k}", flush=True)
                continue
            s.set(k, v); n += 1
        print(f"[capture] render preset applied: {n} /rtx/ keys from {path}", flush=True)
    except Exception as exc:
        print(f"[capture] render preset skipped: {exc!r}", flush=True)


def setup(world, width: int = 1280, height: int = 720) -> None:
    if not enabled() or _S["err"]:
        return
    try:
        import omni.replicator.core as rep
        from pxr import Gf, UsdGeom

        _apply_preset()
        _S["every"] = max(1, int(os.environ.get("G1_CAPTURE_EVERY", "3")))
        res_override = os.environ.get("G1_CAPTURE_RES", "").strip()
        _S["dir"] = os.path.join(os.environ["G1_CAPTURE_DIR"], "frames")
        os.makedirs(_S["dir"], exist_ok=True)
        stage = world.stage
        for path in [p.strip() for p in os.environ["G1_CAPTURE_CAMS"].split(",") if p.strip()]:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsA(UsdGeom.Camera):
                print(f"[capture] camera not found / not a Camera: {path}", flush=True)
                continue
            cd = prim.GetCustomData()
            res = json.loads(cd.get("panthera:resolution", "[1280, 720]"))
            if res_override:
                res = [int(v) for v in res_override.lower().split("x")]
            label = prim.GetName()
            os.makedirs(os.path.join(_S["dir"], label), exist_ok=True)
            rp = rep.create.render_product(path, (int(res[0]), int(res[1])))
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([rp])
            cam = {"path": path, "label": label, "prim": prim, "annot": annot, "mode": cd.get("panthera:mode", "static"),
                   "track": cd.get("panthera:track"), "offset": json.loads(cd.get("panthera:offset_m", "[0,0,0]")),
                   "look_off": json.loads(cd.get("panthera:look_at_offset_m", "[0,0,0]")), "orbit": json.loads(cd["panthera:orbit"]) if "panthera:orbit" in cd else None,
                   "frames": int(cd.get("panthera:frames", 0)), "res": res}
            _S["cams"].append(cam)
            print(f"[capture] {label}: {cam['mode']} {res} {cam['track'] or ''}", flush=True)
        _S["f"] = open(os.path.join(os.environ["G1_CAPTURE_DIR"], "frames.jsonl"), "w", encoding="utf-8")
        print(f"[capture] ON -> {_S['dir']}: {len(_S['cams'])} cameras, every {_S['every']} render steps", flush=True)
        if not _S["cams"]:
            _S["err"] = True
    except Exception as exc:
        _S["err"] = True
        print(f"[capture] setup failed, capture disabled: {exc!r}", flush=True)


def _look_at(eye, target):
    from pxr import Gf

    f = (target - eye).GetNormalized()
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    r = Gf.Cross(f, up)
    if r.GetLength() < 1e-6:
        r = Gf.Cross(f, Gf.Vec3d(0, 1, 0))
    r = r.GetNormalized()
    u = Gf.Cross(r, f)
    return Gf.Matrix4d(r[0], r[1], r[2], 0.0, u[0], u[1], u[2], 0.0, -f[0], -f[1], -f[2], 0.0, eye[0], eye[1], eye[2], 1.0)


def _find_link(stage, robot_root, names):
    from pxr import Usd
    for n in names:
        if n.startswith("/"):
            p = stage.GetPrimAtPath(n)
            if p:
                return p
    for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_root)):
        if prim.GetName() in names:
            return prim
    return None


def _tracked_point(runner, cam):
    import numpy as np
    from pxr import Gf, Usd, UsdGeom

    pos, quat = runner._robot.robot.get_world_pose()
    pos = np.asarray(pos, float)
    w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    if cam["track"] == "pelvis":
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0]); left = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        o = cam["offset"]
        eye = pos + o[0] * fwd + o[1] * left + np.array([0.0, 0.0, o[2]])
        return Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*(pos + np.array(cam["look_off"])).tolist())
    stage = runner._world.stage
    if "link_prim" not in cam:
        cam["link_prim"] = _find_link(stage, runner._robot.prim_path if hasattr(runner._robot, "prim_path") else "/World/G1", TRACK_LINKS.get(cam["track"], (cam["track"],)))
        cam["xcache"] = UsdGeom.XformCache(Usd.TimeCode.Default())
    if cam["link_prim"] is None:
        raise RuntimeError(f"tracked link for {cam['track']} not found")
    cam["xcache"].Clear()
    m = cam["xcache"].GetLocalToWorldTransform(cam["link_prim"])
    p = m.ExtractTranslation()
    o = cam["offset"]
    return Gf.Vec3d(p[0] + o[0], p[1] + o[1], p[2] + o[2]), Gf.Vec3d(p[0] + cam["look_off"][0], p[1] + cam["look_off"][1], p[2] + cam["look_off"][2])


def maybe_step(runner) -> None:
    if _S["err"] or not _S["cams"]:
        return
    try:
        import numpy as np
        from pxr import Gf, Usd, UsdGeom

        _S["tick"] += 1
        for cam in _S["cams"]:
            if cam["mode"] == "track":
                eye, target = _tracked_point(runner, cam)
                xf = UsdGeom.Xformable(cam["prim"]); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(_look_at(eye, target))
            elif cam["mode"] == "orbit" and cam["orbit"] and cam["frames"]:
                # step the authored time samples with the render-step counter (fps-independent orbit)
                tc = Usd.TimeCode((_S["tick"] // max(1, _S["every"])) % (cam["frames"] + 1))
                attr = cam["prim"].GetAttribute("xformOp:transform")
                if attr:
                    val = attr.Get(tc)
                    xf = UsdGeom.Xformable(cam["prim"]); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(val)
        if _S["tick"] % _S["every"] != 0:
            return
        sim_t = None
        try:
            sim_t = float(runner._world.current_time)
        except Exception:
            pass
        pos, quat = runner._robot.robot.get_world_pose()
        row = {"frame": _S["frame"], "render_tick": _S["tick"], "sim_time": sim_t, "robot_pos": [round(float(v), 4) for v in pos], "files": {}}
        for cam in _S["cams"]:
            data = cam["annot"].get_data()
            if data is None or getattr(data, "size", 0) == 0:
                continue
            arr = np.asarray(data)
            if arr.ndim == 3 and arr.shape[2] == 4:
                arr = arr[:, :, :3]
            name = f"{cam['label']}/frame_{_S['frame']:06d}.png"
            try:
                from PIL import Image
                Image.fromarray(arr.astype(np.uint8)).save(os.path.join(_S["dir"], name))
            except Exception:
                import imageio
                imageio.imwrite(os.path.join(_S["dir"], name), arr.astype(np.uint8))
            row["files"][cam["label"]] = name
        _S["f"].write(json.dumps(row) + "\n"); _S["f"].flush()
        _S["frame"] += 1
    except Exception as exc:
        _S["err"] = True
        print(f"[capture] step failed, capture disabled: {exc!r}", flush=True)
