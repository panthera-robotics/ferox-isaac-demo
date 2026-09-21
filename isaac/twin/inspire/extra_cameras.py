"""Declared render-only ADAPTER cameras attached to robot links for closed-loop probes (Sprint O, model lane proposal).

Pure Python, no simulator: parses and validates the OPTIONAL probe-config key ``closed_loop.extra_cameras`` and builds the
link -> USD-camera transform with the same conventions the command-replay probe uses for its policy camera (URDF camera
link: +X = optical axis, +Z = image up; USD camera: looks along -Z with +Y up). The probe creates one render-only
``Camera`` prim per entry under ``/World/G1/<link>/extra_camera_<name>`` and publishes ``obs/<k:06d>_<name>.png`` (sha256
in the observation JSON) next to the policy frame of the SAME render at every inference. The donor carries no such
sensors: these are adapter inputs for a model that expects them (e.g. wrist views), never a donor sensor and never
"native" anything. An absent or empty key yields ``[]`` and the probe's output stays byte-identical.

Entry schema (all keys required except hfov_deg/width/height):
  {"name": <identifier, unique, not a probe camera label>, "link": <robot link name under /World/G1>,
   "xyz": [3 floats, m, link frame], "rpy": [3 floats, rad, URDF fixed-axis roll/pitch/yaw in the link frame],
   "hfov_deg": float in (1, 179) (default 69.0), "width": int 64..1920 (default 640), "height": int 64..1080 (default 480)}
"""
import math

RESERVED_NAMES = ('policy', 'front', 'side', 'closeup_right_hand')          # the probe's own camera labels
ALLOWED_KEYS = ('name', 'link', 'xyz', 'rpy', 'hfov_deg', 'width', 'height')
R_LINK_CAM = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))            # identical to command_replay.py R_link_cam
MAX_CAMERAS = 8


class ExtraCameraError(ValueError):
    pass


def _finite(v, what):
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ExtraCameraError('%s must be a finite number, got %r' % (what, v))
    return float(v)


def _vec3(v, what):
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ExtraCameraError('%s must be a list of three numbers' % what)
    return [_finite(x, what) for x in v]


def rpy_matrix(r, p, y):
    """URDF fixed-axis roll/pitch/yaw -> rotation matrix Rz(y) Ry(p) Rx(r) (nested lists), the probe's own formula."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def parse_extra_cameras(closed_loop):
    """closed_loop block (dict or None) -> list of validated camera specs; [] when the key is absent or empty."""
    if not closed_loop:
        return []
    raw = closed_loop.get('extra_cameras')
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ExtraCameraError('closed_loop.extra_cameras must be a list')
    if len(raw) > MAX_CAMERAS:
        raise ExtraCameraError('at most %d extra cameras' % MAX_CAMERAS)
    specs, names = [], set()
    for i, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise ExtraCameraError('extra_cameras[%d] must be an object' % i)
        unknown = sorted(set(entry) - set(ALLOWED_KEYS))
        if unknown:
            raise ExtraCameraError('extra_cameras[%d]: unknown keys %s' % (i, unknown))
        name = entry.get('name')
        if not isinstance(name, str) or not name.isidentifier() or name in RESERVED_NAMES or name in names:
            raise ExtraCameraError('extra_cameras[%d]: name must be a unique identifier not in %s, got %r' % (i, RESERVED_NAMES, name))
        link = entry.get('link')
        if not isinstance(link, str) or not link or '/' in link or '.' in link or link.strip() != link:
            raise ExtraCameraError('extra_cameras[%d] (%s): link must be a bare robot link name' % (i, name))
        xyz = _vec3(entry.get('xyz'), 'extra_cameras[%d].xyz' % i)
        rpy = _vec3(entry.get('rpy'), 'extra_cameras[%d].rpy' % i)
        hfov = _finite(entry.get('hfov_deg', 69.0), 'extra_cameras[%d].hfov_deg' % i)
        if not 1.0 < hfov < 179.0:
            raise ExtraCameraError('extra_cameras[%d] (%s): hfov_deg must be in (1, 179)' % (i, name))
        w, h = entry.get('width', 640), entry.get('height', 480)
        if isinstance(w, bool) or isinstance(h, bool) or not isinstance(w, int) or not isinstance(h, int) or not (64 <= w <= 1920 and 64 <= h <= 1080):
            raise ExtraCameraError('extra_cameras[%d] (%s): width must be an int in 64..1920 and height in 64..1080' % (i, name))
        names.add(name)
        specs.append({'name': name, 'link': link, 'xyz': xyz, 'rpy': rpy, 'hfov_deg': hfov, 'width': w, 'height': h})
    return specs


def camera_transform(spec):
    """4x4 link -> USD camera transform (row-major nested lists): R = Rrpy @ R_LINK_CAM, t = xyz. Same construction as the
    probe's policy camera (T[:3,:3] = R_link @ R_link_cam; T[:3,3] = mount xyz)."""
    R = _matmul(rpy_matrix(*spec['rpy']), [list(r) for r in R_LINK_CAM])
    T = [[R[i][j] for j in range(3)] + [spec['xyz'][i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
    return T


def optical_axis_and_up(spec):
    """(optical axis, image up) unit vectors in the link frame: the camera's -Z and +Y after camera_transform."""
    T = camera_transform(spec)
    return [-T[i][2] for i in range(3)], [T[i][1] for i in range(3)]


def focal_length(horizontal_aperture, hfov_deg):
    """Focal length for a horizontal field of view, the probe's own formula for the policy camera."""
    return float(horizontal_aperture) / (2.0 * math.tan(math.radians(float(hfov_deg)) / 2.0))


__all__ = ['ExtraCameraError', 'RESERVED_NAMES', 'R_LINK_CAM', 'MAX_CAMERAS', 'parse_extra_cameras', 'camera_transform', 'optical_axis_and_up', 'focal_length', 'rpy_matrix']
