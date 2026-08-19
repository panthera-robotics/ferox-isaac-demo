"""URDF forward kinematics at zero pose, for hand-mount orientation questions.

Pure stdlib: no ROS, no Isaac, no numpy. It exists so the hand-mount question can
be asked and answered OFFLINE, against the URDFs that are the actual source of
truth, and so the answer is reproducible on a box with no GPU.

Why this module rather than reading the numbers off a rendered picture: a picture
shows that a hand is wrong, it does not say by which rotation. DT3's mount defect
(fingers laterally outward, palm forward, thumb down) survived a 214-line gate
report, a 38-test contract suite and an 11-test Isaac suite because every one of
them asked about POSITION -- the flange offset is exact to 0.0000 mm -- and none
asked about ORIENTATION. This module asks about orientation.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Sequence, Tuple

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

IDENT: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


# --------------------------------------------------------------------------
# small linear algebra
# --------------------------------------------------------------------------
def mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def mat_vec(m: Mat3, v: Sequence[float]) -> Vec3:
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))  # type: ignore


def transpose(m: Mat3) -> Mat3:
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))  # type: ignore


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(c * c for c in v))


def unit(v: Sequence[float]) -> Vec3:
    n = norm(v)
    if n < 1e-12:
        raise ValueError("cannot normalise a zero-length vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def cross(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    """Angle between two vectors, in degrees, numerically safe at 0 and 180."""
    c = max(-1.0, min(1.0, dot(unit(a), unit(b))))
    return math.degrees(math.acos(c))


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> Mat3:
    """URDF rpy -> rotation matrix. Fixed-axis XYZ, i.e. R = Rz(yaw)Ry(pitch)Rx(roll).

    This is the URDF spec's convention (and ROS's). Getting it backwards is a
    classic way to produce a rotation that is right for symmetric cases and wrong
    for everything else, so it is stated here rather than assumed.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def mat_to_rpy(m: Mat3) -> Vec3:
    """Rotation matrix -> URDF rpy (fixed-axis XYZ). Inverse of rpy_to_mat."""
    sp = -m[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(abs(sp) - 1.0) < 1e-9:          # gimbal lock: roll and yaw degenerate
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    else:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    return (roll, pitch, yaw)


def axis_angle_to_mat(axis: Sequence[float], theta: float) -> Mat3:
    """Rodrigues. Used for revolute joints at non-zero angle."""
    x, y, z = unit(axis)
    c, s, t = math.cos(theta), math.sin(theta), 1.0 - math.cos(theta)
    return (
        (t * x * x + c,     t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c,     t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def is_rotation(m: Mat3, tol: float = 1e-9) -> bool:
    """Orthonormal with det +1 -- a rotation, not a reflection or a scale.

    Chirality bugs love reflections: a mirrored hand is a reflection, and a
    reflection composed into a transform chain silently swaps handedness while
    every length in the model stays correct.
    """
    p = mat_mul(transpose(m), m)
    for i in range(3):
        for j in range(3):
            if abs(p[i][j] - (1.0 if i == j else 0.0)) > tol:
                return False
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    return abs(det - 1.0) <= tol


# --------------------------------------------------------------------------
# URDF model
# --------------------------------------------------------------------------
class Joint:
    __slots__ = ("name", "type", "parent", "child", "xyz", "rpy", "axis")

    def __init__(self, el: ET.Element):
        self.name = el.get("name")
        self.type = el.get("type")
        self.parent = el.find("parent").get("link")
        self.child = el.find("child").get("link")
        o = el.find("origin")
        self.xyz = tuple(float(v) for v in (o.get("xyz", "0 0 0")).split()) if o is not None else (0.0, 0.0, 0.0)
        self.rpy = tuple(float(v) for v in (o.get("rpy", "0 0 0")).split()) if o is not None else (0.0, 0.0, 0.0)
        a = el.find("axis")
        self.axis = tuple(float(v) for v in a.get("xyz").split()) if a is not None else (1.0, 0.0, 0.0)


class Urdf:
    """Link tree with zero-pose (or given-pose) forward kinematics."""

    def __init__(self, path: str):
        self.path = path
        root = ET.parse(path).getroot()
        self.name = root.get("name")
        self.joints: List[Joint] = [Joint(j) for j in root.findall("joint")]
        self.by_child: Dict[str, Joint] = {j.child: j for j in self.joints}
        self.links: List[str] = [l.get("name") for l in root.findall("link")]
        children = {j.child for j in self.joints}
        roots = [l for l in self.links if l not in children]
        if len(roots) != 1:
            raise ValueError(f"{path}: expected exactly one root link, got {roots}")
        self.root = roots[0]

    def chain(self, link: str) -> List[Joint]:
        """Joints from the root down to `link`, root-first."""
        out: List[Joint] = []
        cur = link
        while cur != self.root:
            j = self.by_child.get(cur)
            if j is None:
                raise KeyError(f"{self.path}: link {cur!r} is not in the tree")
            out.append(j)
            cur = j.parent
        out.reverse()
        return out

    def pose(self, link: str, q: Dict[str, float] | None = None) -> Tuple[Mat3, Vec3]:
        """(R, t) of `link` in the root frame. Unlisted joints are at zero."""
        q = q or {}
        R: Mat3 = IDENT
        t: Vec3 = (0.0, 0.0, 0.0)
        for j in self.chain(link):
            Rj = rpy_to_mat(*j.rpy)
            t = vec_add(t, mat_vec(R, j.xyz))
            R = mat_mul(R, Rj)
            if j.type in ("revolute", "continuous"):
                theta = float(q.get(j.name, 0.0))
                if theta:
                    R = mat_mul(R, axis_angle_to_mat(j.axis, theta))
        return R, t

    def pose_in(self, link: str, frame: str,
                q: Dict[str, float] | None = None) -> Tuple[Mat3, Vec3]:
        """(R, t) of `link` expressed in `frame` rather than in the root."""
        Rl, tl = self.pose(link, q)
        Rf, tf = self.pose(frame, q)
        Rft = transpose(Rf)
        return mat_mul(Rft, Rl), mat_vec(Rft, vec_sub(tl, tf))

    def origin_in(self, link: str, frame: str,
                  q: Dict[str, float] | None = None) -> Vec3:
        return self.pose_in(link, frame, q)[1]

    def tip_of(self, prefix: str) -> str:
        """The deepest link whose name starts with `prefix` -- a fingertip link."""
        cand = [l for l in self.links if l.startswith(prefix)]
        if not cand:
            raise KeyError(f"{self.path}: no link starting with {prefix!r}")
        return max(cand, key=lambda l: len(self.chain(l)))


def fmt_vec(v: Sequence[float], nd: int = 4) -> str:
    return "(" + ", ".join(f"{c:+.{nd}f}" for c in v) + ")"


def nearest_axis(v: Sequence[float]) -> str:
    """Name the closest signed principal axis, with its angular error."""
    axes = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0),
            "-Y": (0, -1, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}
    best = min(axes.items(), key=lambda kv: angle_deg(v, kv[1]))
    return f"{best[0]} ({angle_deg(v, best[1]):.2f} deg off)"
