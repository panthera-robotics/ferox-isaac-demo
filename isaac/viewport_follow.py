# Additive, flag-gated: make the default viewport camera FOLLOW the robot so an external
# screen grab (Xvfb + ffmpeg x11grab) captures the robot in action. No-op unless
# VIEWPORT_FOLLOW is truthy. Never raises into the sim loop.
import os

_S = {"err": False, "count": 0, "every": 1}


def enabled() -> bool:
    return os.environ.get("VIEWPORT_FOLLOW", "").strip().lower() in ("1", "true", "yes", "on")


def maybe_step(runner) -> None:
    if _S["err"] or not enabled():
        return
    try:
        import numpy as np
        try:
            from isaacsim.core.utils.viewports import set_camera_view
        except Exception:
            from isaacsim.core.utils.viewport import set_camera_view  # type: ignore
        pos, quat = runner._robot.robot.get_world_pose()
        pos = np.asarray(pos, float)
        w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        back = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        # 3/4 chase: behind-left + up, looking at the torso
        left = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
        eye = pos + (-4.0) * back + 1.6 * left + np.array([0.0, 0.0, 2.2])
        target = pos + np.array([0.0, 0.0, 0.55])
        set_camera_view(eye.tolist(), target.tolist())
    except Exception:
        _S["err"] = True
