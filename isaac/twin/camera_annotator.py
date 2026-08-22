"""Camera capture that never touches the OmniGraph image writer.

C-23 kills the process on the first `world.step()` after a render product exists, inside
`libomni.syntheticdata` + `libomni.graph.image.core`, and it does it on an RTX 4080 AND
on an RTX 4090 with driver 580.105.08 (`docs/twin/evidence/C23/C23_ON_A_4090.md`). So it
is not the GPU and no bigger box fixes it. What the crash is reached THROUGH is
`rep.writers.get(<rendervar>ROS2PublishImage)` — the OmniGraph ROS 2 image writer.

This module takes the DT-era offscreen path instead: attach ANNOTATORS to the same render
product, pull the frames into host memory with `get_data()`, and publish them from Python
with rclpy. Same topics, same frame_ids, same sim-time stamps as the contract; rgb8 and
16UC1 millimetres, which is what the contract asks for and what the OmniGraph bridge could
never emit anyway (`publishers.setup_camera_depth_raw` says so in its own docstring).

Two routes, in the order Mohammed set:

* ``annotator`` — `rgb` + `distance_to_image_plane` annotators on the render product.
* ``get_rgba``  — the exact call C-23's own control proved works on this box
  (`Camera.get_rgba()`), plus the depth annotator, for the case where the annotator
  host-copy is itself what dies.

Messages are built by hand rather than through cv_bridge: the payload is a contiguous
numpy buffer either way, and it removes a dependency that is not in this image.
"""
from __future__ import annotations

import numpy as np


class AnnotatorCamera:
    """Offscreen RGB + depth capture, published from Python."""

    def __init__(self, route: str = "annotator", log=print):
        self.route = route
        self.log = log
        self._rgb_annot = None
        self._depth_annot = None
        self._camera = None
        self._rp = None
        self.n_rgb = 0
        self.n_depth = 0
        self.last_shapes = (None, None)

    # ------------------------------------------------------------------ setup
    def attach(self, camera):
        import omni.replicator.core as rep

        self._camera = camera
        self._rp = camera.get_render_product_path()
        if not self._rp:
            raise RuntimeError("camera has no render product")

        if self.route in ("annotator", "auto"):
            try:
                self._rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
                self._rgb_annot.attach([self._rp])
            except Exception as exc:
                self.log(f"[cam] rgb annotator unavailable: {exc!r}")
                self._rgb_annot = None
        # Depth is an annotator on BOTH routes: get_rgba() has no depth equivalent, and
        # distance_to_image_plane is the render var the contract's metres come from.
        try:
            self._depth_annot = rep.AnnotatorRegistry.get_annotator(
                "distance_to_image_plane")
            self._depth_annot.attach([self._rp])
        except Exception as exc:
            self.log(f"[cam] depth annotator unavailable: {exc!r}")
            self._depth_annot = None
        self.log(f"[cam] offscreen route '{self.route}' attached to {self._rp} "
                 f"(rgb_annot={self._rgb_annot is not None}, "
                 f"depth_annot={self._depth_annot is not None}) -- "
                 f"NO OmniGraph image writer")

    # ------------------------------------------------------------------ frames
    def rgb(self):
        """HxWx3 uint8, or None. Never raises into the physics loop."""
        try:
            if self._rgb_annot is not None:
                a = self._rgb_annot.get_data()
                if a is None:
                    return None
                a = np.asarray(a)
                if a.size == 0:
                    return None
                if a.ndim == 3 and a.shape[2] == 4:
                    a = a[:, :, :3]
                self.n_rgb += 1
                return np.ascontiguousarray(a.astype(np.uint8))
            if self._camera is not None:
                a = self._camera.get_rgba()
                if a is None:
                    return None
                a = np.asarray(a)
                if a.size == 0:
                    return None
                if a.ndim == 3 and a.shape[2] == 4:
                    a = a[:, :, :3]
                if a.dtype != np.uint8:
                    a = (np.clip(a, 0.0, 1.0) * 255.0).astype(np.uint8)
                self.n_rgb += 1
                return np.ascontiguousarray(a)
        except Exception as exc:
            self.log(f"[cam] rgb fetch failed: {exc!r}")
        return None

    def depth_mm(self):
        """HxW uint16 millimetres, or None. Non-finite -> 0, which is the
        contract's 'no return' value and what the real D435i emits."""
        try:
            if self._depth_annot is None:
                return None
            d = self._depth_annot.get_data()
            if d is None:
                return None
            d = np.asarray(d, dtype=np.float32)
            if d.size == 0:
                return None
            mm = d * 1000.0
            mm[~np.isfinite(mm)] = 0.0
            mm[mm < 0] = 0.0
            np.clip(mm, 0, 65535, out=mm)
            self.n_depth += 1
            return np.ascontiguousarray(mm.astype(np.uint16))
        except Exception as exc:
            self.log(f"[cam] depth fetch failed: {exc!r}")
        return None


def make_image_msg(arr, frame_id: str, sim_time: float, encoding: str):
    """sensor_msgs/Image from a contiguous numpy array, stamped in SIM time.

    Sim time, not wall time: every twin consumer runs use_sim_time, and a wall stamp
    reads as a message from the far future and is dropped by TF-buffered subscribers.
    """
    from sensor_msgs.msg import Image

    m = Image()
    m.header.frame_id = frame_id
    m.header.stamp.sec = int(sim_time)
    m.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
    m.height = int(arr.shape[0])
    m.width = int(arr.shape[1])
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = int(arr.strides[0])
    m.data = arr.tobytes()
    return m
