"""C-39 offline: total mass, CoM and the ankle static margin from an MJCF.

No GPU, no Isaac. This is the reference half of the diff; the twin half comes from
`isaac/twin/c39_capture.py` (which needs the sim) as `links_<variant>.csv`.

Computes the CoM at the NOMINAL STANCE, not at the model's zero pose: the stance is
what the static-margin question is about. Joint angles are applied by walking the
body tree and composing each joint's rotation about its own axis.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

# SONIC's nominal stance, SDK order (sim_side.SONIC_DEFAULT_Q).
NOMINAL = {
    "left_hip_pitch_joint": -0.1, "left_knee_joint": 0.3, "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.1, "right_knee_joint": 0.3, "right_ankle_pitch_joint": -0.2,
}


def _v(s, n=3, d=0.0):
    if s is None:
        return np.full(n, d, float)
    p = [float(x) for x in s.replace(",", " ").split()]
    return np.array(p + [d] * (n - len(p)), float)[:n]


def _rot(axis, ang):
    a = np.asarray(axis, float)
    a = a / max(float(np.linalg.norm(a)), 1e-12)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)


def _quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def walk(body, R0, p0, out, angles):
    p = p0 + R0 @ _v(body.get("pos"))
    R = R0
    if body.get("quat"):
        R = R @ _quat_to_R(_v(body.get("quat"), 4))
    for j in body.findall("joint"):
        nm = j.get("name") or ""
        if j.get("type") in ("free", "slide"):
            continue
        R = R @ _rot(_v(j.get("axis"), 3, 0.0) if j.get("axis") else [0, 0, 1],
                     float(angles.get(nm, 0.0)))
    ine = body.find("inertial")
    if ine is not None:
        m = float(ine.get("mass", 0.0))
        c = p + R @ _v(ine.get("pos"))
        dg = ine.get("diaginertia")
        out.append({"link": body.get("name", "?"), "mass_kg": m,
                    "com_world": [round(float(v), 6) for v in c],
                    "com_local": [round(float(v), 6) for v in _v(ine.get("pos"))],
                    "inertia_diag": [round(float(v), 8) for v in _v(dg)] if dg else None,
                    "body_world": [round(float(v), 6) for v in p]})
    for ch in body.findall("body"):
        walk(ch, R, p, out, angles)


ap = argparse.ArgumentParser()
ap.add_argument("xml")
ap.add_argument("--out-csv", default="")
ap.add_argument("--out-json", default="")
ap.add_argument("--zero-pose", action="store_true")
a = ap.parse_args()

root = ET.parse(a.xml).getroot()
wb = root.find("worldbody")
if wb is None:
    sys.exit("no <worldbody>")
angles = {} if a.zero_pose else NOMINAL
links = []
for b in wb.findall("body"):
    walk(b, np.eye(3), np.zeros(3), links, angles)

M = np.array([l["mass_kg"] for l in links])
C = np.array([l["com_world"] for l in links])
tot = float(M.sum())
com = (C * M[:, None]).sum(axis=0) / max(tot, 1e-9)
ank = [l for l in links if "ankle_roll" in l["link"]]
ax = float(np.mean([l["body_world"][0] for l in ank])) if ank else float("nan")
d = float(com[0]) - ax
tau = tot * 9.81 * abs(d)

res = {
    "source": a.xml, "pose": "zero" if a.zero_pose else "SONIC nominal",
    "n_links": len(links), "total_mass_kg": round(tot, 6),
    "com_world": [round(float(v), 6) for v in com],
    "ankle_mean_x": round(ax, 6),
    "com_x_minus_ankle_x": round(d, 6),
    "required_ankle_torque_Nm": round(tau, 4),
    # A joint-space PD makes torque only out of error, so this is the bias the
    # controller must hold to stand -- the number to compare against joint travel.
    "bias_rad_at_kp_28_5": round(tau / 28.5, 4),
    "bias_rad_at_kp_40": round(tau / 40.0, 4),
    "heaviest": sorted([{"link": l["link"], "mass_kg": round(l["mass_kg"], 6)}
                        for l in links], key=lambda r: -r["mass_kg"])[:12],
}
print(json.dumps(res, indent=2))
if a.out_json:
    open(a.out_json, "w").write(json.dumps({**res, "links": links}, indent=2))
if a.out_csv:
    with open(a.out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["link", "mass_kg", "com_local_x", "com_local_y", "com_local_z",
                    "world_x", "world_y", "world_z", "ixx", "iyy", "izz"])
        for l in links:
            cl, bw = l["com_local"], l["body_world"]
            di = l["inertia_diag"] or [None] * 3
            w.writerow([l["link"], l["mass_kg"], *cl, *bw, *di])
