#!/usr/bin/env python3
"""Merge Unitree's G1 29-DoF and Dex5-1P URDFs into one robot description.

WHY A MERGED URDF AND NOT USD COMPOSITION
-----------------------------------------
Referencing each hand into the G1 stage and bolting it on with a PhysicsFixedJoint
composes perfectly at the USD level -- 40 finger joints, 2.003615 kg, mount offsets
exact to the micron -- and is still WRONG, because PhysX does not absorb the hands
into the G1's articulation. The articulation stays at 29 DOF and 30 bodies, so
nothing can command a finger. Three placements were tried and all behave the same:
the flange joint under the parent link, the flange joint in the robot's `joints/`
scope, and the hand's own ArticulationRootAPI deleted so it cannot claim its links.
The articulation is built at parse time from ONE URDF/robot; a body introduced by a
later reference joins the scene as a maximal-coordinate rigid body instead.

NVIDIA's documented answer for attaching a gripper to an arm is to import the
combined URDF, and it is what Unitree do themselves -- see
g1_description/merge_g1_29dof_and_inspire_hand.ipynb upstream. So we merge.

WHAT THIS DOES AND DOES NOT DO
------------------------------
Does: concatenate. Every link, joint, inertial, limit and mesh comes across byte-for
-byte from Unitree's files, and the only NEW elements are the two fixed flange joints.
Those joints carry an ORIGIN read from the G1's own wrist->palm joints and a ROTATION
derived by tools/derive_hand_flange.py -- see FLANGE_RPY below. Neither is a fitted
number: the origin is copied and the rotation is a signed axis permutation whose
derivation reproduces Unitree's own published hand flange to 1e-16.
Does not: scale anything, retune any inertia, or rename any joint. Campaign rule 9 --
never scale a hand, a mesh, or an offset to make it fit -- is the whole reason the
W6 MuJoCo model (dex5 meshes at scale="0.75", palm geom with density="0") is not a
usable source for the flange pose either.

The robot name is preserved as g1_29dof_rev_1_0 so every prim path in the twin's
sensor layer keeps resolving against the merged asset.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import xml.etree.ElementTree as ET

# The G1 URDF already carries left_hand_palm_joint / right_hand_palm_joint: fixed
# joints from the wrist to a `*_rubber_hand` link, which is the bare end-effector cap
# the robot ships with (0.170 kg, visual only, no collision). The Dex5 REPLACES it, so
# the merge deletes the cap and reuses the joint name for the real flange.
#
# The flange origin is READ from those joints rather than written here. It happens to
# equal the value in g1_29dof_with_hand_rev_1_0.urdf (0.0415, +-0.003, 0), and that
# agreement is the point: the wrist flange is a property of the ARM, not of whichever
# hand is bolted to it. That retires the `assumed` provenance the contract carried for
# this offset -- it now comes from the G1's own description.
FLANGE = {
    "L": {"parent": "left_wrist_yaw_link",  "child": "base_link00L",
          "joint": "left_hand_palm_joint",  "replaces": "left_rubber_hand"},
    "R": {"parent": "right_wrist_yaw_link", "child": "base_link00",
          "joint": "right_hand_palm_joint", "replaces": "right_rubber_hand"},
}

# The flange ROTATION. Originally "0 0 0", on the assumption that the Dex5 root
# frame is wrist-aligned. It is not, and that assumption is DT3's mount defect:
# the Dex5's fingers run along its own +Y while the G1's forearm runs along wrist
# +X, so an identity flange left both hands rotated 90 deg -- fingers laterally
# outward, palm forward, thumb down. The flange ORIGIN was exact all along, which
# is why every DT3 position check passed and none of them saw it.
#
# This value is NOT typed in from a picture or tuned until it looked right. It is
# the output of tools/derive_hand_flange.py, which:
#   - measures the hand's finger axis and palm normal from the Dex5 URDF itself
#     (palm normal by actually flexing a joint, not by reading a mesh face),
#   - maps them onto the wrist-frame convention EXTRACTED from Unitree's own
#     published G1 + Inspire assembly (g1_29dof_rev_1_0_with_inspire_hand_DFQ),
#   - snaps to the nearest signed axis permutation, since a bolted flange is one,
#   - and SELF-TESTS by re-deriving Unitree's published Inspire flange rpy from
#     Inspire geometry alone: it reproduces (0,0,pi/2) and (pi,0,-pi/2) to 1e-16.
#
# Independently corroborated by the W7 MuJoCo graft (graft_dex5_both.py), whose
# QUAT = [0.5,-0.5,-0.5,-0.5] is this exact rotation, 0.000000000 deg apart. Its
# SCALE=0.75 and DX=0.072 are the campaign's anti-pattern and are not read here.
#
# Both sides take the SAME rotation: the Dex5 L and R URDFs are already mirrored
# internally (thumb at -Z on the left, +Z on the right), so the flange does not
# have to carry the mirror. Unitree's Inspire files are NOT internally mirrored
# in the same way, which is why theirs differ per side. Asserted, not assumed --
# check_hand_orientation.py's chirality test fails if the pair comes out swapped.
_Q = math.pi / 2
FLANGE_RPY = {
    "L": (-_Q, -_Q, 0.0),
    "R": (-_Q, -_Q, 0.0),
}
EXPECT_FLANGE_XYZ = {"L": (0.0415, 0.003, 0.0), "R": (0.0415, -0.003, 0.0)}
FLANGE_TOL = 1e-9
MESH_SUBDIR = {"L": "dex5_meshes_L", "R": "dex5_meshes_R"}
EXPECT = {
    "body_revolute": 29,
    "hand_revolute_per_side": 20,
    "hand_links_per_side": 21,
    "hand_mass": {"L": 1.025045, "R": 0.978570},
}
MASS_TOL = 1e-5


def _revolute(root):
    return [j for j in root.findall("joint")
            if j.get("type") in ("revolute", "continuous")]


def _mass(root):
    return sum(float(m.get("value")) for m in root.iter("mass"))


def merge(g1_path: str, hand_paths: dict, out_urdf: str) -> dict:
    g1_tree = ET.parse(g1_path)
    g1 = g1_tree.getroot()

    body_joints_before = [j.get("name") for j in _revolute(g1)]

    # Lift the flange origin off the G1's own wrist->palm joints, then delete those
    # joints and the rubber-hand caps they carry. Reusing the joint NAME without
    # deleting the original produced two joints called left_hand_palm_joint; the
    # importer then resolved the tree to a hand as its root link and wrote a 2.4 kB
    # asset containing nothing. Duplicate FIXED joints are as fatal as duplicate
    # links, and the revolute-only checks below never saw them.
    flange_xyz = {}
    for side, spec in FLANGE.items():
        jt = next((j for j in g1.findall("joint") if j.get("name") == spec["joint"]), None)
        if jt is None:
            raise SystemExit(f"{spec['joint']} not found in the G1 URDF")
        got = tuple(float(v) for v in jt.find("origin").get("xyz").split())
        want = EXPECT_FLANGE_XYZ[side]
        if max(abs(a - b) for a, b in zip(got, want)) > FLANGE_TOL:
            raise SystemExit(f"{spec['joint']} origin {got} != {want}; the G1 flange "
                             "moved upstream -- update the contract, do not shim it")
        if jt.find("child").get("link") != spec["replaces"]:
            raise SystemExit(f"{spec['joint']} child is "
                             f"{jt.find('child').get('link')!r}, expected "
                             f"{spec['replaces']!r}")
        flange_xyz[side] = jt.find("origin").get("xyz")
        g1.remove(jt)
        cap = next((l for l in g1.findall("link")
                    if l.get("name") == spec["replaces"]), None)
        if cap is None:
            raise SystemExit(f"{spec['replaces']} link not found")
        g1.remove(cap)

    body_links = {l.get("name") for l in g1.findall("link")}
    body_mass = _mass(g1)

    report = {"body_joints": body_joints_before, "hands": {}}

    for side, path in hand_paths.items():
        hand = ET.parse(path).getroot()
        links = hand.findall("link")
        joints = _revolute(hand)
        if len(links) != EXPECT["hand_links_per_side"]:
            raise SystemExit(f"{side}: {len(links)} links, expected "
                             f"{EXPECT['hand_links_per_side']}")
        if len(joints) != EXPECT["hand_revolute_per_side"]:
            raise SystemExit(f"{side}: {len(joints)} revolute joints, expected "
                             f"{EXPECT['hand_revolute_per_side']}")
        hm = _mass(hand)
        if abs(hm - EXPECT["hand_mass"][side]) > MASS_TOL:
            raise SystemExit(f"{side}: mass {hm:.6f} != {EXPECT['hand_mass'][side]:.6f}. "
                             "The URDF is not the one this tool was written against.")

        # Name collisions would silently reparent a link. There are none between
        # the G1 and either hand, and there must not be any after a Unitree update.
        clash = body_links & {l.get("name") for l in links}
        if clash:
            raise SystemExit(f"{side}: link names collide with the G1: {sorted(clash)}")

        # Unitree's Dex5 files declare every visual material as <material name="">.
        # That is legal enough on its own -- importing a hand alone works -- but in a
        # robot that also has NAMED materials the importer resolves the empty name to
        # nothing and dies with "Used null prim" after writing a stub asset. Name them
        # from their own rgba so the name is deterministic and the colour is untouched:
        # this renames, it does not restyle.
        for vis in hand.iter("visual"):
            for mat in vis.findall("material"):
                if mat.get("name"):
                    continue
                col = mat.find("color")
                rgba = col.get("rgba") if col is not None else "0 0 0 1"
                key = "_".join(f"{float(v):.3f}".replace(".", "") for v in rgba.split())
                mat.set("name", f"dex5_{side}_{key}")

        # Mesh filenames are relative to the hand's own package; repoint them at the
        # per-side subdir the staging step fills.
        for mesh in hand.iter("mesh"):
            fn = mesh.get("filename")
            mesh.set("filename", f"{MESH_SUBDIR[side]}/{os.path.basename(fn)}")

        for l in links:
            g1.append(l)
        for j in hand.findall("joint"):
            g1.append(j)

        spec = FLANGE[side]
        rpy = FLANGE_RPY[side]
        fj = ET.SubElement(g1, "joint", {"name": spec["joint"], "type": "fixed"})
        ET.SubElement(fj, "origin", {
            "xyz": flange_xyz[side],
            "rpy": " ".join(f"{v:.15f}" for v in rpy)})
        ET.SubElement(fj, "parent", {"link": spec["parent"]})
        ET.SubElement(fj, "child", {"link": spec["child"]})
        body_links |= {l.get("name") for l in links}
        report["hands"][side] = {"mass": hm, "joints": [j.get("name") for j in joints]}

    # --- self-verification on the MERGED tree -------------------------------
    merged_rev = [j.get("name") for j in _revolute(g1)]
    if merged_rev[:len(body_joints_before)] != body_joints_before:
        raise SystemExit("body joint order changed by the merge -- the walk policy "
                         "indexes these; refusing to write.")
    want_rev = EXPECT["body_revolute"] + 2 * EXPECT["hand_revolute_per_side"]
    if len(merged_rev) != want_rev:
        raise SystemExit(f"merged revolute count {len(merged_rev)} != {want_rev}")
    if len(set(merged_rev)) != len(merged_rev):
        raise SystemExit("duplicate revolute joint names in the merged URDF")
    all_joint_names = [j.get("name") for j in g1.findall("joint")]
    dupes = {n for n in all_joint_names if all_joint_names.count(n) > 1}
    if dupes:
        raise SystemExit(f"duplicate joint names (any type): {sorted(dupes)}")

    # A URDF must be exactly one tree. Two parentless links import as a robot rooted
    # at whichever the parser reaches first, silently.
    children = {j.find("child").get("link") for j in g1.findall("joint")}
    parentless = [l.get("name") for l in g1.findall("link")
                  if l.get("name") not in children]
    if parentless != ["pelvis"]:
        raise SystemExit(f"expected pelvis as the sole root link, got {parentless}")
    total = _mass(g1)
    want_mass = body_mass + sum(EXPECT["hand_mass"].values())
    if abs(total - want_mass) > 1e-4:
        raise SystemExit(f"merged mass {total:.6f} != {want_mass:.6f}")

    # No material may reach the importer unnamed, from either source.
    anon = [m for m in g1.iter("material") if not m.get("name")]
    if anon:
        raise SystemExit(f"{len(anon)} unnamed <material> elements remain")

    # Every joint must reference links that exist, or the importer builds a
    # disconnected tree and reports it as a warning nobody reads.
    names = {l.get("name") for l in g1.findall("link")}
    for j in g1.findall("joint"):
        for end in ("parent", "child"):
            ln = j.find(end).get("link")
            if ln not in names:
                raise SystemExit(f"joint {j.get('name')} {end} link {ln!r} does not exist")

    os.makedirs(os.path.dirname(out_urdf), exist_ok=True)
    g1_tree.write(out_urdf, encoding="utf-8", xml_declaration=True)
    report.update({"body_mass": body_mass, "total_mass": total,
                   "revolute": merged_rev, "out": out_urdf})
    return report


def stage_meshes(g1_dir: str, hand_dirs: dict, out_dir: str):
    """Copy meshes next to the merged URDF, preserving each source's own names."""
    src = os.path.join(g1_dir, "meshes")
    dst = os.path.join(out_dir, "meshes")
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    for side, d in hand_dirs.items():
        hdst = os.path.join(out_dir, MESH_SUBDIR[side])
        if os.path.isdir(hdst):
            shutil.rmtree(hdst)
        shutil.copytree(os.path.join(d, "meshes"), hdst)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", default=os.path.expanduser("~/panthera/ref/unitree_ros/robots"))
    ap.add_argument("--out-dir", default="/tmp/g1_dex5_urdf")
    args = ap.parse_args()

    g1_dir = os.path.join(args.ref, "g1_description")
    hand_dirs = {
        "L": os.path.join(args.ref, "dexterous_hand_description/dex5_1/Dex5-URDF-L"),
        "R": os.path.join(args.ref, "dexterous_hand_description/dex5_1/Dex5-URDF-R"),
    }
    out_urdf = os.path.join(args.out_dir, "g1_29dof_dex5_1p.urdf")
    stage_meshes(g1_dir, hand_dirs, args.out_dir)
    rep = merge(os.path.join(g1_dir, "g1_29dof_rev_1_0.urdf"),
                {s: os.path.join(d, f"Dex5-URDF-{s}.urdf") for s, d in hand_dirs.items()},
                out_urdf)
    print(f"merged -> {rep['out']}")
    print(f"  revolute joints : {len(rep['revolute'])} "
          f"({EXPECT['body_revolute']} body + 40 hand)")
    print(f"  body mass       : {rep['body_mass']:.6f} kg")
    print(f"  total mass      : {rep['total_mass']:.6f} kg")
    for s, h in rep["hands"].items():
        print(f"  hand {s}          : {h['mass']:.6f} kg, {len(h['joints'])} joints")
    print("  body joint order preserved: PASS")


if __name__ == "__main__":
    main()
