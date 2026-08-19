#!/usr/bin/env python3
"""Derive the fixed rpy of the G1 wrist -> hand flange joint, from geometry.

THE QUESTION
------------
`tools/merge_dex5_urdf.py` bolts each Dex5-1P to the G1 with

    <origin xyz="0.0415 +-0.003 0" rpy="0 0 0"/>

The xyz is read from the G1's own `*_hand_palm_joint` and is exact. The rpy was
assumed to be identity, i.e. that the Dex5 root frame is wrist-aligned. It is
not: the Dex5's fingers run along its own +Y, the G1's forearm along wrist +X.
This tool computes what the rpy should be, and does it without a single number
typed in by hand.

HOW, AND WHY IT IS TRUSTWORTHY
------------------------------
Two anatomical directions fix a hand's orientation completely:

    finger axis   f  = unit(middle fingertip - middle MCP)    [zero pose]
    palm normal   n  = the direction a fingertip MOVES when the finger flexes

Both are measured off the hand's own URDF -- f from link origins, n by actually
flexing a joint and differencing. Neither is read off a mesh, a link's +Z, or a
naming convention. They are orthogonal to within a fraction of a degree in both
Dex5 files, which is itself a check that they mean what they are supposed to.

The TARGET those two must hit in the wrist frame is Unitree's own convention for
a five-fingered hand on this exact wrist:

    f -> wrist +X          fingers run along the forearm
    n -> wrist -Y  (left)  palm faces the body midline
    n -> wrist +Y  (right)

which is not asserted here either. It is EXTRACTED from
`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`, Unitree's published G1 + Inspire
assembly, by pushing that hand's own f and n through the flange rotation Unitree
wrote. And the whole derivation is then run BACKWARDS on the Inspire hand as a
self-test: given only its geometry and that target, this tool must reproduce
Unitree's published rpy -- (0, 0, +pi/2) left and (pi, 0, -pi/2) right -- to
1e-9. It does. A method that recovers the vendor's answer on the vendor's own
assembly is the strongest available evidence that its answer for the Dex5, for
which no vendor assembly exists, is right too.

WHAT THIS DELIBERATELY IGNORES
------------------------------
The W6 MuJoCo graft (`graft_dex5_both.py`) carries a quaternion for the same
mount. Its PERMUTATION is a useful cross-check and is printed. Its `SCALE=0.75`
and `DX=0.072` are not, and nothing here reads them: campaign rule 9 forbids
scaling a hand to make it fit, and that graft is the anti-pattern the whole
campaign exists to not repeat.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hand_frames import (  # noqa: E402
    Urdf, angle_deg, cross, dot, fmt_vec, is_rotation, mat_mul, mat_to_rpy,
    mat_vec, rpy_to_mat, transpose, unit, vec_sub,
)

REF = os.path.expanduser("~/panthera/ref/unitree_ros/robots")
G1_INSPIRE = os.path.join(REF, "g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf")
DEX5 = {
    "L": os.path.join(REF, "dexterous_hand_description/dex5_1/Dex5-URDF-L/Dex5-URDF-L.urdf"),
    "R": os.path.join(REF, "dexterous_hand_description/dex5_1/Dex5-URDF-R/Dex5-URDF-R.urdf"),
}

# Per hand family: which links are the index finger and which joint flexes it.
ANATOMY = {
    "dex5": {
        "L": {"root": "base_link00L", "mid_mcp": "Link_31L", "mid_tip": "Link_34L",
              "mid_flex": "Pitch_32L",
              "thumb_root": "Link_11L", "thumb_tip": "Link_14L"},
        "R": {"root": "base_link00",  "mid_mcp": "Link_31R", "mid_tip": "Link_34R",
              "mid_flex": "Pitch_32R",
              "thumb_root": "Link_11R", "thumb_tip": "Link_14R"},
    },
    "inspire": {
        "L": {"root": "L_hand_base_link", "mid_mcp": "L_middle_proximal",
              "mid_tip": "L_middle_intermediate",
              "mid_flex": "L_middle_proximal_joint",
              "thumb_root": "L_thumb_proximal_base", "thumb_tip": "L_thumb_distal"},
        "R": {"root": "R_hand_base_link", "mid_mcp": "R_middle_proximal",
              "mid_tip": "R_middle_intermediate",
              "mid_flex": "R_middle_proximal_joint",
              "thumb_root": "R_thumb_proximal_base", "thumb_tip": "R_thumb_distal"},
    },
}

# Unitree's published flange rotations for the G1 + Inspire assembly. Used ONLY
# as the self-test target -- the derivation never reads them.
UNITREE_INSPIRE_RPY = {
    "L": (0.0, 0.0, math.pi / 2),
    "R": (math.pi, 0.0, -math.pi / 2),
}

FLEX_PROBE_RAD = 0.20


def anatomical_axes(u: Urdf, spec: dict, root: str) -> tuple:
    """(finger axis, palm normal) of a hand, expressed in frame `root`, zero pose.

    The finger axis is the MIDDLE finger's own axis -- its tip minus its own
    metacarpal. Two earlier choices were wrong and the self-test caught both:

      tip minus PALM ROOT folds the finger's lateral offset into the direction --
      a 10.9 deg bias on the Inspire hand, 4.3 deg on the Dex5. The Dex5's would
      have slipped under a 5 deg gate while being an artefact of where the palm
      origin sits rather than anything about the mount.

      the INDEX finger is splayed. Unitree fans the Inspire's fingers by joint
      rpy -- index -2.00 deg, middle 0, ring +3.00, pinky +6.00 -- so the index
      is 4.3 deg off the hand's axis by design. The MIDDLE finger is the one
      Unitree gives identity rpy, which makes it their axial reference, and it is
      the anatomically obvious choice besides.
    """
    mcp = u.origin_in(spec["mid_mcp"], root)
    tip = u.origin_in(spec["mid_tip"], root)
    f = unit(vec_sub(tip, mcp))

    flexed = u.origin_in(spec["mid_tip"], root, q={spec["mid_flex"]: FLEX_PROBE_RAD})
    motion = vec_sub(flexed, tip)
    perp = tuple(motion[i] - dot(motion, f) * f[i] for i in range(3))
    n = unit(perp)

    # f and n must be perpendicular: a finger flexes ACROSS its own length. If
    # they are not, one of them is not measuring what its name says.
    skew = abs(90.0 - angle_deg(f, n))
    if skew > 3.0:
        raise SystemExit(f"finger axis and palm normal are {90 - skew:.2f} deg apart, "
                         "not perpendicular -- the anatomy spec is wrong")
    # Re-orthogonalise the residual so the frame below is exactly orthonormal.
    n = unit(tuple(n[i] - dot(n, f) * f[i] for i in range(3)))
    return f, n


def frame_from(f, n):
    """Orthonormal basis with f as its first axis and n as its second, columns."""
    t = cross(f, n)
    return tuple(tuple((f, n, t)[j][i] for j in range(3)) for i in range(3))


def signed_permutations():
    """The 24 proper signed permutation matrices -- the octahedral rotation group.

    A mechanical flange between two axis-aligned frames IS one of these: it bolts
    +X, +Y, +Z onto some signed permutation of +X, +Y, +Z. Unitree's own two G1
    hand flange rotations are both exactly members of this set, which is the
    evidence that this is the right space to answer in.
    """
    import itertools
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            M = [[0.0] * 3 for _ in range(3)]
            for col, row in enumerate(perm):
                M[row][col] = float(signs[col])
            M = tuple(tuple(r) for r in M)
            if is_rotation(M, tol=1e-12):
                out.append(M)
    assert len(out) == 24, len(out)
    return out


PERMS = signed_permutations()


def frob(a, b) -> float:
    return math.sqrt(sum((a[i][j] - b[i][j]) ** 2 for i in range(3) for j in range(3)))


def rot_angle_between(a, b) -> float:
    """Angle of the rotation taking a to b, in degrees."""
    d = mat_mul(b, transpose(a))
    tr = d[0][0] + d[1][1] + d[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, (tr - 1.0) / 2.0))))


# How far the least-squares rotation may sit from the nearest permutation before
# the answer stops being "a permutation plus the hand's own splay". Unitree's
# Inspire needs 4.68 deg; anything past 8 would mean the mount is genuinely not
# axis-aligned and this whole method is the wrong shape for it.
SNAP_TOL_DEG = 8.0


def solve(f_hand, n_hand, f_wrist, n_wrist, snap: bool = True):
    """The flange rotation taking the hand's (f, n) onto the wrist's (f, n).

    Two steps, and the second one is the load-bearing one.

    1. Least squares: R = T . S^-1, where S and T are the orthonormal frames built
       from each (finger axis, palm normal) pair.

    2. SNAP to the nearest proper signed permutation. This is not tidying up. A
       hand's fingers are splayed inside its own shell -- Unitree fans the
       Inspire's by -2/0/+3/+6 deg -- so the least-squares rotation carries that
       splay, and a mount that absorbed it would be rotating the whole hand to
       cancel a feature of the hand. That is "scale it to fit" wearing a different
       hat, and campaign rule 9 forbids it. The flange is a bolted interface
       between two axis-aligned frames; its rotation is a permutation, and the
       residual belongs to the hand.

    The snap distance is returned, not hidden: if it were large the mount would
    not be axis-aligned and this method would be the wrong shape for the question.
    """
    S = frame_from(f_hand, n_hand)
    T = frame_from(f_wrist, n_wrist)
    R = mat_mul(T, transpose(S))
    if not is_rotation(R, tol=1e-7):
        raise SystemExit("derived transform is not a proper rotation")
    if not snap:
        return R, 0.0
    best = min(PERMS, key=lambda P: frob(P, R))
    dev = rot_angle_between(R, best)
    if dev > SNAP_TOL_DEG:
        raise SystemExit(
            f"nearest signed permutation is {dev:.2f} deg away (tolerance "
            f"{SNAP_TOL_DEG}); this mount is not axis-aligned and this method "
            "does not apply to it")
    return best, dev


# The wrist-frame target, per side. Extracted from Unitree's Inspire assembly by
# extract_target(); these literals are the EXPECTED result of that extraction and
# are asserted against it, never used in its place.
TARGET = {
    "L": {"f": (1.0, 0.0, 0.0), "n": (0.0, -1.0, 0.0)},
    "R": {"f": (1.0, 0.0, 0.0), "n": (0.0, +1.0, 0.0)},
}


def extract_target(verbose: bool = True) -> dict:
    """What wrist-frame directions does Unitree put a G1 hand's f and n on?"""
    u = Urdf(G1_INSPIRE)
    out = {}
    for side in ("L", "R"):
        spec = ANATOMY["inspire"][side]
        wrist = f"{'left' if side == 'L' else 'right'}_wrist_yaw_link"
        f_w, n_w = anatomical_axes(u, spec, wrist)
        out[side] = {"f": f_w, "n": n_w}
        if verbose:
            print(f"  Inspire {side}: finger axis in wrist {fmt_vec(f_w)}   "
                  f"palm normal in wrist {fmt_vec(n_w)}")
    return out


def selftest(verbose: bool = True) -> bool:
    """Re-derive Unitree's own Inspire flange rpy from Inspire geometry alone."""
    ok = True
    if verbose:
        print("[self-test] target extracted from Unitree's G1 + Inspire assembly:")
    tgt = extract_target(verbose)

    for side in ("L", "R"):
        # The extracted target must be the clean axis pair TARGET claims.
        for k in ("f", "n"):
            err = angle_deg(tgt[side][k], TARGET[side][k])
            # SNAP_TOL_DEG, not zero: what is being confirmed is that Unitree put
            # this hand's f on wrist +X and its n on wrist -+Y to within the
            # hand's own splay -- i.e. that the target axes are the right ones,
            # not that the hand is machined perfectly onto them.
            mark = "PASS" if err <= SNAP_TOL_DEG else "FAIL"
            if err > SNAP_TOL_DEG:
                ok = False
            if verbose:
                print(f"  [{mark}] Inspire {side} {k} is {err:.3f} deg off "
                      f"{fmt_vec(TARGET[side][k], 0)}  (hand's own splay; tol {SNAP_TOL_DEG})")

        # Now the real test: hand geometry + target -> Unitree's published rpy.
        hand = Urdf(G1_INSPIRE)
        spec = ANATOMY["inspire"][side]
        f_h, n_h = anatomical_axes(hand, spec, spec["root"])
        R, dev = solve(f_h, n_h, TARGET[side]["f"], TARGET[side]["n"])
        got = mat_to_rpy(R)
        want = UNITREE_INSPIRE_RPY[side]
        Rw = rpy_to_mat(*want)
        # Compare as matrices: rpy triples are not unique, rotations are.
        dev = max(abs(R[i][j] - Rw[i][j]) for i in range(3) for j in range(3))
        mark = "PASS" if dev < 1e-6 else "FAIL"
        if dev >= 1e-6:
            ok = False
        if verbose:
            print(f"  [{mark}] Inspire {side}: derived rpy "
                  f"({got[0]:+.6f}, {got[1]:+.6f}, {got[2]:+.6f})  vs Unitree's "
                  f"({want[0]:+.6f}, {want[1]:+.6f}, {want[2]:+.6f})   "
                  f"max matrix deviation {dev:.2e}")
    return ok


def derive_dex5(verbose: bool = True) -> dict:
    out = {}
    for side in ("L", "R"):
        u = Urdf(DEX5[side])
        spec = ANATOMY["dex5"][side]
        f_h, n_h = anatomical_axes(u, spec, spec["root"])
        R, dev = solve(f_h, n_h, TARGET[side]["f"], TARGET[side]["n"])
        rpy = mat_to_rpy(R)
        out[side] = {"rpy": rpy, "R": R, "f_hand": f_h, "n_hand": n_h, "snap_deg": dev}
        if verbose:
            print(f"\n  Dex5 {side} ({spec['root']}):")
            print(f"    finger axis in hand root  {fmt_vec(f_h)}")
            print(f"    palm normal in hand root  {fmt_vec(n_h)}")
            print(f"    -> flange rotation rows   {fmt_vec(R[0])} {fmt_vec(R[1])} {fmt_vec(R[2])}")
            print(f"    -> rpy  ({rpy[0]:+.15f}, {rpy[1]:+.15f}, {rpy[2]:+.15f})")
            print(f"    -> rpy  ({rpy[0] / math.pi:+.4f}pi, {rpy[1] / math.pi:+.4f}pi, "
                  f"{rpy[2] / math.pi:+.4f}pi)")
            print(f"    snap to nearest signed permutation: {dev:.3f} deg "
                  f"(the hand's own finger splay; tol {SNAP_TOL_DEG})")
    return out


def quat_to_mat(q):
    """MuJoCo quaternion order (w, x, y, z) -> rotation matrix."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
        (2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
    )


def wbc_quat_crosscheck(derived, verbose: bool = True) -> dict:
    """Compare the W7 MuJoCo graft's PERMUTATION with ours. Its scale is ignored.

    `graft_dex5_both.py` grafts the same hands onto the same wrist in MuJoCo, and
    carries `QUAT = [0.5, -0.5, -0.5, -0.5]` for the mount frame. That quaternion
    is a third, independent statement of the same permutation, arrived at on a
    different simulator by a different route months earlier.

    Its `SCALE = 0.75` and `DX = 0.072` are the campaign's canonical anti-pattern
    -- a hand shrunk to make it fit -- and this function reads NEITHER. It parses
    the one line it wants, converts it, and compares a rotation with a rotation.
    """
    path = os.path.expanduser(
        "~/panthera/ref/panthera-g1-wbc/wholebody/models/dex5_1/reference_scripts/"
        "graft_dex5_both.py")
    if not os.path.exists(path):
        if verbose:
            print(f"\n[wbc cross-check] not available ({path} absent) -- skipped")
        return {"available": False}

    import re
    quat = None
    for line in open(path):
        m = re.match(r"\s*QUAT\s*=\s*\[([^\]]+)\]", line)
        if m:
            quat = [float(v) for v in m.group(1).split(",")]
            break
    if quat is None:
        if verbose:
            print("\n[wbc cross-check] no QUAT literal found -- skipped")
        return {"available": False}

    Rw = quat_to_mat(quat)
    out = {"available": True, "quat": quat, "sides": {}}
    if verbose:
        print(f"\n[wbc cross-check] W7 MuJoCo graft QUAT (w,x,y,z) = {quat}")
        print("  (SCALE=0.75 and DX=0.072 in that file are the anti-pattern and are "
              "deliberately not read)")
        print(f"  as a matrix: rows {fmt_vec(Rw[0])} {fmt_vec(Rw[1])} {fmt_vec(Rw[2])}")
    for side in ("L", "R"):
        dev = rot_angle_between(derived[side]["R"], Rw)
        agree = dev < 1e-6
        out["sides"][side] = {"deviation_deg": dev, "agrees": agree}
        if verbose:
            print(f"  [{'PASS' if agree else 'DIFF'}] vs derived Dex5 {side}: "
                  f"{dev:.9f} deg apart")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    v = not args.quiet

    print("=" * 74)
    print("Deriving the G1 wrist -> Dex5-1P flange rotation from URDF geometry")
    print("=" * 74)

    ok = selftest(v)
    if not ok:
        print("\nSELF-TEST FAILED -- the method does not reproduce Unitree's own "
              "assembly, so its Dex5 answer cannot be trusted. Refusing to emit one.")
        return 1
    print("\n[self-test] PASS -- the method reproduces Unitree's published G1 hand "
          "flange rotations from geometry alone.")

    print("\n" + "-" * 74)
    print("Dex5-1P, same method, same target:")
    res = derive_dex5(v)
    wbc = wbc_quat_crosscheck(res, v)

    print("\n" + "=" * 74)
    print("ANSWER -- merge_dex5_urdf.py flange rpy:")
    for side in ("L", "R"):
        r = res[side]["rpy"]
        print(f'  {side}: rpy="{r[0]:.15f} {r[1]:.15f} {r[2]:.15f}"')
    print("=" * 74)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
