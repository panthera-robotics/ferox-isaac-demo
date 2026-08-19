#!/usr/bin/env python3
"""Is each Dex5-1P hand mounted in the orientation the real robot has?

WHAT THIS ASKS THAT NOTHING ELSE DID
------------------------------------
DT3 verified the hand mount's POSITION -- "exact to 0.0000 mm, read from the G1's
own *_hand_palm_joint" -- and never verified its ORIENTATION. The flange joint was
written with `rpy="0 0 0"`, on the assumption that the Dex5 root frame is aligned
with the wrist. It is not: the Dex5's fingers run along its own +Y while the G1's
forearm runs along wrist +X, so an identity flange rotates the whole hand 90 deg.
Every position check still passed, because the flange ORIGIN is right; only the
rotation was wrong. Same failure class as C-21, where a camera with the right
topic, frame_id, K and encoding pointed out of the robot's right side.

THE THREE DIRECTIONS, AND WHY THEY ARE THESE THREE
--------------------------------------------------
A hand's pose needs three independent facts; any two leave a rotation free.

  finger axis   the MIDDLE finger's own axis, tip minus its own metacarpal.
                The middle finger because Unitree splay the others by joint rpy
                and give the middle one identity -- it is their axial reference.
                Measuring from the PALM ROOT instead folds the finger's lateral
                offset in: a 4.3 deg bias on the Dex5 and 10.9 on the Inspire.

  palm normal   the direction a fingertip MOVES when the finger flexes. Derived,
                not declared: computed by actually flexing the middle MCP and
                differencing the tip. A palm normal read off a mesh face or a
                link's +Z would be a guess about mesh authoring; the direction
                the finger closes is a property of the kinematics.

  thumb axis    unit(thumb tip - thumb root), zero pose.

TWO FRAMES, BECAUSE THE CLAIMS LIVE IN DIFFERENT ONES
-----------------------------------------------------
"Fingers along the forearm" is a statement about the MOUNT and is true in the
wrist frame at any arm pose. "Palm toward the midline" and "thumb forward" are
statements about the ROBOT and only mean anything in a specific posture. So both
are checked:

  wrist frame, zero pose  -- the mount itself. f -> +X, n -> -+Y, thumb -> +Z.
  base_link, STANDING     -- the posture the sim actually stands in, from
                             isaac/checkpoints/g1/params/env.yaml: shoulder_pitch
                             0.3, shoulder_roll +-0.25, elbow 0.97, wrist_roll
                             +-0.15. This is the pose the progress montage shows,
                             so it is the pose the montage's defect is judged in.

CHIRALITY
---------
Left and right Dex5 differ by a mirror. If the two are swapped, the fingers still
lie along the forearm and the palms still face somewhere sensible, but both thumbs
point OUTWARD instead of forward. Nothing that looks at one hand can see that,
which is why both are always checked and why the thumb is reported as a signed
direction rather than an angle.

USAGE
  python3 tools/check_hand_orientation.py [--urdf PATH] [--json OUT]
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hand_frames import (  # noqa: E402
    Urdf, angle_deg, cross, dot, fmt_vec, mat_vec, nearest_axis, transpose,
    unit, vec_sub,
)

DEFAULT_URDF = "/tmp/g1_dex5_urdf/g1_29dof_dex5_1p.urdf"

# The posture the twin stands in. isaac/checkpoints/g1/params/env.yaml init state,
# arm joints only -- the legs do not move the wrists.
STANDING = {
    "left_shoulder_pitch_joint": 0.3, "right_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.25, "right_shoulder_roll_joint": -0.25,
    "left_elbow_joint": 0.97, "right_elbow_joint": 0.97,
    "left_wrist_roll_joint": 0.15, "right_wrist_roll_joint": -0.15,
}

SIDES = {
    "L": {
        "wrist": "left_wrist_yaw_link",
        "hand_root": "base_link00L",
        "mid_mcp": "Link_31L", "mid_tip": "Link_34L", "mid_flex": "Pitch_32L",
        "index_tip": "Link_24L",
        "thumb_root": "Link_11L", "thumb_tip": "Link_14L",
        # base_link +y is the robot's LEFT, so the left palm facing the midline
        # means its normal points to the robot's right.
        "palm_toward_base": (0.0, -1.0, 0.0),
        "palm_toward_wrist": (0.0, -1.0, 0.0),
    },
    "R": {
        "wrist": "right_wrist_yaw_link",
        "hand_root": "base_link00",
        "mid_mcp": "Link_31R", "mid_tip": "Link_34R", "mid_flex": "Pitch_32R",
        "index_tip": "Link_24R",
        "thumb_root": "Link_11R", "thumb_tip": "Link_14R",
        "palm_toward_base": (0.0, +1.0, 0.0),
        "palm_toward_wrist": (0.0, +1.0, 0.0),
    },
}

FORWARD = (1.0, 0.0, 0.0)      # base_link +X
WRIST_FINGER = (1.0, 0.0, 0.0)  # wrist +X -- the forearm axis
WRIST_THUMB = (0.0, 0.0, 1.0)   # wrist +Z -- forward once the arm is at rest

# Acceptance. 5 deg on the finger axis is the campaign's stated criterion. The
# palm and thumb are wide-margin sign tests: they exist to catch a 90 deg
# permutation or a swapped pair, and a palm 30 deg off the midline is still
# unambiguously facing it. Tightening them would be measuring the standing
# posture, not the mount.
FINGER_TOL_DEG = 5.0
PALM_TOL_DEG = 60.0
THUMB_TOL_DEG = 75.0

FLEX_PROBE_RAD = 0.20


def _axes(u: Urdf, s: dict, frame: str, q: dict | None) -> dict:
    """finger axis, palm normal, thumb axis and the raw vectors, in `frame`."""
    palm_root = u.origin_in(s["hand_root"], frame, q)
    mcp = u.origin_in(s["mid_mcp"], frame, q)
    tip = u.origin_in(s["mid_tip"], frame, q)
    index_tip = u.origin_in(s["index_tip"], frame, q)
    thumb_root = u.origin_in(s["thumb_root"], frame, q)
    thumb_tip = u.origin_in(s["thumb_tip"], frame, q)

    finger = unit(vec_sub(tip, mcp))
    thumb = unit(vec_sub(thumb_tip, thumb_root))

    qq = dict(q or {})
    qq[s["mid_flex"]] = FLEX_PROBE_RAD
    flexed = u.origin_in(s["mid_tip"], frame, qq)
    motion = vec_sub(flexed, tip)
    perp = tuple(motion[i] - dot(motion, finger) * finger[i] for i in range(3))
    palm = unit(perp)

    return {
        "palm_root": list(palm_root),
        "finger_axis": list(finger),
        "palm_normal": list(palm),
        "thumb_axis": list(thumb),
        # The vector the campaign brief named, kept because it is what a reader
        # will reach for -- and reported next to the middle-finger axis so the
        # difference between them is visible rather than silent.
        "index_tip_minus_palm_root": list(unit(vec_sub(index_tip, palm_root))),
        "finger_nearest": nearest_axis(finger),
        "palm_nearest": nearest_axis(palm),
        "thumb_nearest": nearest_axis(thumb),
    }


def measure(u: Urdf, side: str) -> dict:
    s = SIDES[side]
    wrist = s["wrist"]

    R_wrist_hand, t_wrist_hand = u.pose_in(s["hand_root"], wrist)
    hand_axes = {k: tuple(R_wrist_hand[i][c] for i in range(3))
                 for c, k in enumerate(("+X", "+Y", "+Z"))}

    out = {
        "side": side, "wrist": wrist, "hand_root": s["hand_root"],
        "flange_xyz_in_wrist": list(t_wrist_hand),
        "hand_axes_in_wrist": {k: list(v) for k, v in hand_axes.items()},
        "hand_axes_in_wrist_nearest": {k: nearest_axis(v) for k, v in hand_axes.items()},
        # zero pose, wrist frame -- the mount
        "in_wrist": _axes(u, s, wrist, None),
        # standing posture, body frame -- the robot
        "in_base_standing": _axes(u, s, u.root, STANDING),
        # zero pose, body frame -- kept for continuity with the DT3 numbers
        "in_base_zero": _axes(u, s, u.root, None),
    }

    finger_err = angle_deg(out["in_wrist"]["finger_axis"], WRIST_FINGER)
    palm_wrist_err = angle_deg(out["in_wrist"]["palm_normal"], s["palm_toward_wrist"])
    thumb_wrist_err = angle_deg(out["in_wrist"]["thumb_axis"], WRIST_THUMB)
    palm_base_err = angle_deg(out["in_base_standing"]["palm_normal"], s["palm_toward_base"])
    thumb_base_err = angle_deg(out["in_base_standing"]["thumb_axis"], FORWARD)

    out["checks"] = {
        "fingers_along_wrist_x": {
            "value_deg": finger_err, "tol_deg": FINGER_TOL_DEG,
            "pass": finger_err <= FINGER_TOL_DEG,
            "what": "middle finger axis vs wrist +X (the forearm), zero pose",
        },
        "palm_toward_midline_wrist": {
            "value_deg": palm_wrist_err, "tol_deg": PALM_TOL_DEG,
            "pass": palm_wrist_err <= PALM_TOL_DEG,
            "what": f"palm normal vs wrist {fmt_vec(s['palm_toward_wrist'], 0)}, zero pose",
        },
        "thumb_along_wrist_z": {
            "value_deg": thumb_wrist_err, "tol_deg": THUMB_TOL_DEG,
            "pass": thumb_wrist_err <= THUMB_TOL_DEG,
            "what": "thumb axis vs wrist +Z, zero pose",
        },
        "palm_toward_midline_standing": {
            "value_deg": palm_base_err, "tol_deg": PALM_TOL_DEG,
            "pass": palm_base_err <= PALM_TOL_DEG,
            "what": f"palm normal vs base_link {fmt_vec(s['palm_toward_base'], 0)}, standing",
        },
        "thumb_forward_standing": {
            "value_deg": thumb_base_err, "tol_deg": THUMB_TOL_DEG,
            "pass": thumb_base_err <= THUMB_TOL_DEG,
            "what": "thumb axis vs base_link +X (forward), standing",
        },
    }
    out["pass"] = all(c["pass"] for c in out["checks"].values())
    return out


def chirality(res: dict) -> dict:
    """Are these two hands a mirrored PAIR, or the same hand twice?

    Standing posture, base_link. A real pair has palm normals pointing at each
    other across the body and thumbs both forward. A swapped pair has both thumbs
    pointing OUTWARD -- away from each other -- which is a negative lateral
    component on both sides at once and is invisible one hand at a time.
    """
    L = res["L"]["in_base_standing"]
    R = res["R"]["in_base_standing"]
    l_out = L["thumb_axis"][1]       # >0: left thumb points further left
    r_out = -R["thumb_axis"][1]      # >0: right thumb points further right
    both_outward = l_out > 0.5 and r_out > 0.5
    palms_inward = (L["palm_normal"][1] < 0.0) and (R["palm_normal"][1] > 0.0)
    return {
        "left_thumb_lateral": l_out,
        "right_thumb_lateral": r_out,
        "both_thumbs_outward": both_outward,
        "palms_face_each_other": palms_inward,
        "pass": (not both_outward) and palms_inward,
        "what": "thumbs outward on BOTH hands at once means L and R are swapped",
    }


def report(urdf_path: str) -> dict:
    u = Urdf(urdf_path)
    res = {side: measure(u, side) for side in SIDES}
    out = {"urdf": urdf_path, "robot": u.name, "standing_pose": STANDING,
           "sides": res, "chirality": chirality(res)}
    out["pass"] = all(res[s]["pass"] for s in res) and out["chirality"]["pass"]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    if not os.path.exists(args.urdf):
        print(f"no such URDF: {args.urdf}\n"
              f"build it with:  python3 tools/merge_dex5_urdf.py", file=sys.stderr)
        return 2

    out = report(args.urdf)
    print(f"URDF   : {out['urdf']}")
    print(f"robot  : {out['robot']}")
    for side in ("L", "R"):
        r = out["sides"][side]
        print(f"\n=== {side} : {r['hand_root']} on {r['wrist']} ===")
        print(f"  flange xyz in wrist   {fmt_vec(r['flange_xyz_in_wrist'], 6)}")
        print("  hand root axes in wrist:")
        for k in ("+X", "+Y", "+Z"):
            print(f"    hand {k} -> wrist {fmt_vec(r['hand_axes_in_wrist'][k])}"
                  f"   nearest {r['hand_axes_in_wrist_nearest'][k]}")
        for key, label in (("in_wrist", "wrist_yaw_link, zero pose"),
                           ("in_base_standing", "base_link, STANDING")):
            d = r[key]
            print(f"  in {label}:")
            print(f"    finger axis (middle finger)  {fmt_vec(d['finger_axis'])}"
                  f"   nearest {d['finger_nearest']}")
            print(f"    palm normal (flex direction) {fmt_vec(d['palm_normal'])}"
                  f"   nearest {d['palm_nearest']}")
            print(f"    thumb axis  (tip - root)     {fmt_vec(d['thumb_axis'])}"
                  f"   nearest {d['thumb_nearest']}")
            print(f"    [ref] index tip - palm root  {fmt_vec(d['index_tip_minus_palm_root'])}")
        print("  checks:")
        for name, c in r["checks"].items():
            mark = "PASS" if c["pass"] else "FAIL"
            print(f"    [{mark}] {name:30s} {c['value_deg']:8.3f} deg (tol {c['tol_deg']:.0f})"
                  f"  {c['what']}")

    ch = out["chirality"]
    print(f"\n=== chirality (standing) ===")
    print(f"  left thumb lateral component  {ch['left_thumb_lateral']:+.4f}")
    print(f"  right thumb lateral component {ch['right_thumb_lateral']:+.4f}")
    print(f"  palms face each other         {ch['palms_face_each_other']}")
    print(f"  [{'PASS' if ch['pass'] else 'FAIL'}] {ch['what']}")

    print(f"\nRESULT: {'PASS' if out['pass'] else 'FAIL'}")
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w") as fh:
            json.dump(out, fh, indent=2)
        print(f"wrote {args.json}")
    return 0 if out["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
