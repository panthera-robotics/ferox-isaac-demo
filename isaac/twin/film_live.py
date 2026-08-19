"""Film the LIVE twin — the sim that is actually running, not a standalone scene.

WHY THIS REPLACES film.py's --drive policy
------------------------------------------
`tools/film.py` builds its own World and constructs a second G1VelocityPolicy in it.
Three real bugs were fixed inside that path (a paused timeline that produced 600
identical frames, a robot spawned lying on the floor, and a 1/60 s physics step that
turned 50 Hz control into 15 Hz), and it still could not walk. The gain read-back
added there says why in one line:

    policy gains: kp[min,max]=(40.0, 35809.9)   kd[min,max]=(0.000, 5.000)

`deploy.yaml` tops out near 150 and no joint has zero damping. `initialize()` never
applied the deployed controller's gains in that context, so the robot in every clip
was never driven by the controller the clip was supposed to show.

The live sim has none of that: `run.py` already loads the policy, applies
`deploy.yaml`'s gains, steps physics at 200 Hz and drives the robot from `/cmd_vel`.
MM1 measured it walking at 2.4 % error. So the camera goes to the robot rather than
the robot being rebuilt around the camera.

WHAT IT DOES
    Adds offscreen cameras to the running stage, follows the robot, and writes PNGs
    on the converged path (`rep.orchestrator.step(rt_subframes=N)`) — the same
    convergence `film.py` uses, for the same reason: a frame grabbed mid-accumulation
    still carries the previous pose.

    Flag-gated (`TWIN_FILM=1`) and does nothing otherwise, so the default sim is
    byte-for-byte what it was.

C-23: the instance-segmentation annotator segfaults on this box, so the numeric
ghost gate is not computed here. Clips are marked "visually clean, numeric ghost
gate deferred to 4090", exactly as the other MM clips are.
"""
from __future__ import annotations

import os

import numpy as np


def _look_at(cam_prim, eye, target, up=(0.0, 0.0, 1.0)):
    """Author the look-at matrix directly on the prim.

    Camera(orientation=...) is silently ignored and set_world_pose applies its own
    world-to-USD conversion on top (RESULTS_DT3 §4 F-6, C-21). Authoring the matrix
    is the one route that behaves, and this is a verbatim port of film.py's.
    """
    from pxr import Gf, UsdGeom

    eye = Gf.Vec3d(*[float(v) for v in eye])
    tgt = Gf.Vec3d(*[float(v) for v in target])
    fwd = (tgt - eye).GetNormalized()
    right = Gf.Cross(fwd, Gf.Vec3d(*up)).GetNormalized()
    if right.GetLength() < 1e-6:
        right = Gf.Vec3d(1.0, 0.0, 0.0)
    trueup = Gf.Cross(right, fwd).GetNormalized()
    m = Gf.Matrix4d(right[0], right[1], right[2], 0.0,
                    trueup[0], trueup[1], trueup[2], 0.0,
                    -fwd[0], -fwd[1], -fwd[2], 0.0,
                    eye[0], eye[1], eye[2], 1.0)
    x = UsdGeom.Xformable(cam_prim)
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(m)


class LiveFilm:
    """Offscreen follow camera on the running sim. Inert unless TWIN_FILM=1."""

    def __init__(self, robot_getter):
        self.enabled = os.environ.get("TWIN_FILM", "0") == "1"
        self._robot = robot_getter
        self.cam = None
        self.kind = os.environ.get("TWIN_FILM_SHOT", "chase")
        self.subframes = int(os.environ.get("TWIN_FILM_SUBFRAMES", "32"))
        self.out = os.environ.get("TWIN_FILM_OUT", "/tmp/film/live")
        self.fps = float(os.environ.get("TWIN_FILM_FPS", "30"))
        self.seconds = float(os.environ.get("TWIN_FILM_SECONDS", "0"))
        self.res = (1920, 1080)
        self._lag = None
        self._n = 0
        self._next_t = 0.0
        self._active = False

    def start(self):
        if not self.enabled or self.cam is not None:
            return
        from isaacsim.sensors.camera import Camera

        os.makedirs(self.out, exist_ok=True)
        self.cam = Camera(prim_path="/World/twin_film_cam",
                          name="twin_film_cam", resolution=self.res)
        self.cam.initialize()
        self.cam.set_clipping_range(0.1, 200.0)
        self._active = True
        print(f"[FILM] live camera up: shot={self.kind} subframes={self.subframes} "
              f"-> {self.out} ({self.seconds:.0f}s at {self.fps:.0f} fps)", flush=True)

    def _pose(self):
        art = self._robot()
        if art is None:
            return None
        try:
            p, _ = art.get_world_pose()
            p = np.asarray(p).reshape(-1)
            return float(p[0]), float(p[1]), float(p[2])
        except Exception:
            return None

    def _place(self, x, y, z, lag=0.12):
        if self.kind == "chase":
            if self._lag is None:
                self._lag = np.array([x, y], dtype=float)
            else:
                self._lag += lag * (np.array([x, y]) - self._lag)
            lx, ly = self._lag
            _look_at(self.cam.prim, (lx - 3.4, ly - 2.6, z + 1.5), (x, y, z + 0.15))
        elif self.kind == "front":
            _look_at(self.cam.prim, (x + 4.2, y, z + 0.9), (x, y, z + 0.1))
        elif self.kind == "side":
            _look_at(self.cam.prim, (x, y + 4.2, z + 0.9), (x, y, z + 0.1))
        elif self.kind == "top":
            _look_at(self.cam.prim, (x, y, z + 6.0), (x, y, z), up=(1.0, 0.0, 0.0))
        else:
            raise RuntimeError(f"unknown TWIN_FILM_SHOT {self.kind!r}")

    def tick(self, sim_time):
        """Called once per render frame from the main loop."""
        if not self._active:
            return
        if sim_time < self._next_t:
            return
        pose = self._pose()
        if pose is None:
            return
        self._next_t = sim_time + 1.0 / self.fps
        self._place(*pose)
        try:
            import omni.replicator.core as rep
            from PIL import Image

            # The converged path, as in film.py: N subframes for one output frame,
            # timeline paused so rendering does not advance sim time. The live sim
            # keeps stepping physics from its own loop.
            rep.orchestrator.step(rt_subframes=self.subframes, delta_time=0.0,
                                  pause_timeline=True)
            a = self.cam.get_rgba()
            if a is None or a.size == 0 or a.ndim != 3:
                return
            Image.fromarray(a[:, :, :3].astype(np.uint8)).save(
                os.path.join(self.out, f"f{self._n:05d}.png"))
            self._n += 1
            if self._n % 30 == 0:
                print(f"[FILM] {self._n} frames", flush=True)
            if self.seconds and self._n >= int(self.seconds * self.fps):
                print(f"[FILM] done: {self._n} frames -> {self.out}", flush=True)
                self._active = False
        except Exception as exc:  # noqa: BLE001
            print(f"[FILM] frame {self._n} failed: {exc}", flush=True)
            self._active = False
