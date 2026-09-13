#!/usr/bin/env python3
"""Build a pinned Unitree FTP donor for simulation, without asserting RH56E2 equivalence.

Only existing files from an explicit source checkout are read. No downloads,
hardware transports, mass scaling, or host installs are performed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


SOURCE_URL = "https://github.com/unitreerobotics/unitree_ros"
SOURCE_SHA = "7d6075f7f58588b189b940130e3edab3c839b2df"
DESCRIPTION = Path("robots/g1_description")
FULL_URDF = "g1_29dof_rev_1_0_with_inspire_hand_FTP.urdf"
SEMANTICS = ("little", "ring", "middle", "index", "thumb_bend", "thumb_rotation")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numbers(text: str, count: int, label: str) -> tuple[float, ...]:
    values = tuple(float(x) for x in text.split())
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError(f"{label}: expected {count} finite values")
    return values


def audit_urdf(path: str | Path) -> dict:
    """Inspect topology, finite masses/inertias, named limits, and mimic graph."""
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("expected a URDF robot")
    links = root.findall("link")
    names = [x.get("name") for x in links]
    if None in names or len(names) != len(set(names)):
        raise ValueError("duplicate or missing link name")
    masses, unmodelled = {}, []
    for link in links:
        inertial = link.find("inertial")
        if inertial is None:
            unmodelled.append(link.get("name"))
            continue
        mass = numbers(inertial.find("mass").get("value"), 1, "mass")[0]
        tensor = inertial.find("inertia")
        a, b, c, d, e, f = numbers(" ".join(tensor.get(k) for k in
                                          ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")), 6, "inertia")
        det = a*d*f + 2*b*c*e - a*e*e - d*c*c - f*b*b
        if mass <= 0 or a <= 0 or a*d-b*b <= 0 or det <= 0:
            raise ValueError(f"{link.get('name')}: mass/inertia must be positive definite")
        masses[link.get("name")] = mass
    joints, parents, children = {}, {}, set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name or name in joints:
            raise ValueError("duplicate or missing joint name")
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        if parent not in names or child not in names or child in children or parent == child:
            raise ValueError(f"{name}: invalid parent/child topology")
        children.add(child); parents[child] = parent
        origin = joint.find("origin")
        xyz = numbers(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", 3, "origin xyz")
        rpy = numbers(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", 3, "origin rpy")
        item = dict(type=joint.get("type"), parent=parent, child=child, xyz=xyz, rpy=rpy)
        if joint.get("type") != "fixed":
            if joint.get("type") != "revolute":
                raise ValueError(f"{name}: unsupported donor joint type")
            limit = joint.find("limit")
            lo, hi, effort, velocity = numbers(" ".join(limit.get(k) for k in
                                          ("lower", "upper", "effort", "velocity")), 4, "limit")
            axis = numbers(joint.find("axis").get("xyz"), 3, "axis")
            if lo > hi or effort <= 0 or velocity <= 0 or not math.isclose(sum(x*x for x in axis), 1., abs_tol=1e-5):
                raise ValueError(f"{name}: invalid joint limits or axis")
            item.update(limits=(lo, hi), effort_nm=effort, velocity_rad_s=velocity, axis=axis)
            mimic = joint.find("mimic")
            if mimic is not None:
                multiplier, offset = numbers(f"{mimic.get('multiplier', '1')} {mimic.get('offset', '0')}", 2, "mimic")
                item["mimic"] = dict(joint=mimic.get("joint"), multiplier=multiplier, offset=offset)
        joints[name] = item
    roots = set(names) - children
    if len(roots) != 1:
        raise ValueError("donor must be one rooted kinematic tree")
    for child in names:
        seen = set()
        while child in parents:
            if child in seen:
                raise ValueError("kinematic cycle")
            seen.add(child); child = parents[child]
    moving = {k: v for k, v in joints.items() if v["type"] != "fixed"}
    independent = [k for k, v in moving.items() if "mimic" not in v]
    for name, item in moving.items():
        seen = {name}
        while "mimic" in item:
            target = item["mimic"]["joint"]
            if target in seen or target not in moving:
                raise ValueError("unknown or cyclic mimic target")
            seen.add(target); item = moving[target]
    return dict(link_count=len(links), root_link=next(iter(roots)),
                authored_mass_kg=sum(masses.values()), link_masses_kg=masses,
                links_without_authored_inertia=unmodelled, joints=joints,
                physical_joint_names=list(moving), independent_joint_names=independent,
                authored_revolute_count=len(moving), independent_command_count=len(independent),
                usd_runtime_dof_count=None)


def coupled_positions(audit: dict, targets: dict[str, float]) -> dict[str, float]:
    """Expand authored linear mimic relations in radians; no register calibration."""
    if set(targets) != set(audit["independent_joint_names"]):
        raise ValueError("expected exactly the independent donor joint names")
    values = {}
    def resolve(name):
        if name in values:
            return values[name]
        joint = audit["joints"][name]
        if "mimic" in joint:
            rule = joint["mimic"]
            q = resolve(rule["joint"]) * rule["multiplier"] + rule["offset"]
        else:
            q = targets[name]
        if isinstance(q, bool) or not isinstance(q, (int, float)) or not math.isfinite(q):
            raise ValueError("nonfinite or nonnumeric joint target")
        lo, hi = joint["limits"]
        if not lo-1e-12 <= q <= hi+1e-12:
            raise ValueError(f"{name}: coupled target outside donor limits")
        values[name] = q
        return q
    for name in audit["physical_joint_names"]:
        resolve(name)
    return values


def hand_summary(audit: dict, side: str) -> dict:
    active = [f"{side}_{digit}_1_joint" for digit in ("little", "ring", "middle", "index")]
    active += [f"{side}_thumb_2_joint", f"{side}_thumb_1_joint"]
    if len(audit["physical_joint_names"]) != 12 or set(active) != set(audit["independent_joint_names"]):
        raise ValueError(f"{side}: donor requires twelve physical joints and six known independent joints")
    if any(not name.startswith(side+"_") for name in audit["joints"]):
        raise ValueError(f"{side}: hand chirality/name mismatch")
    mass = audit["authored_mass_kg"]
    return dict(side=side, source_model="Unitree FTP donor", exact_RH56E2_equivalence=False,
                semantic_to_donor_joint=dict(zip(SEMANTICS, active)),
                semantic_map_scope="donor kinematic names only; installed register calibration unverified",
                authored_mass_kg=mass, target_manufacturer_nominal_mass_kg=.79,
                target_manufacturer_nominal_tolerance_kg=.01,
                target_nominal_mass_source="https://en.inspire-robots.com/product/rh56e2/",
                within_target_nominal_hand_mass=abs(mass-.79) <= .01,
                mass_rescaled=False, mechanical_audit=audit)


def bench_hand(source: str | Path, output: str | Path, side: str) -> dict:
    """Remove only the empty wrist fixture parent, retaining its flange transform."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    tree = ET.parse(source)
    root = tree.getroot()
    wrist = root.find(f"link[@name='{side}_wrist_yaw_link']")
    flange = root.find(f"joint[@name='{side}_base_joint']")
    if wrist is None or list(wrist) or flange is None or flange.get("type") != "fixed":
        raise ValueError("expected empty donor wrist parent and fixed flange joint")
    if flange.find("parent").get("link") != wrist.get("name") or flange.find("child").get("link") != f"{side}_base_link":
        raise ValueError("unexpected donor flange topology")
    origin = flange.find("origin")
    transform = dict(xyz=numbers(origin.get("xyz", "0 0 0"), 3, "flange xyz"),
                     rpy=numbers(origin.get("rpy", "0 0 0"), 3, "flange rpy"))
    before = audit_urdf(source)
    root.remove(wrist); root.remove(flange)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    after = audit_urdf(output)
    if before["link_masses_kg"] != after["link_masses_kg"] or before["physical_joint_names"] != after["physical_joint_names"]:
        raise ValueError("fixture removal changed hand mass or moving joints")
    return dict(removed_empty_fixture_parent=wrist.get("name"), removed_fixed_joint=flange.get("name"),
                retained_flange_transform=transform, bench_root=f"{side}_base_link",
                requires_explicit_fixed_base_fixture_in_simulator=True,
                hand_geometry_modified=False, hand_mass_modified=False, audit=after)


def build(source_root: str | Path, output: str | Path) -> dict:
    source, output = Path(source_root).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError("output directory must be absent or empty")
    head = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    if head != SOURCE_SHA:
        raise ValueError(f"source must be pinned to {SOURCE_SHA}")
    urdfs = {FULL_URDF: DESCRIPTION/FULL_URDF,
             "FTP_left_hand.urdf": DESCRIPTION/"inspire_hand/FTP_left_hand.urdf",
             "FTP_right_hand.urdf": DESCRIPTION/"inspire_hand/FTP_right_hand.urdf"}
    files = {Path("LICENSE"): Path("LICENSE")}
    audits = {}
    for dest, relative in urdfs.items():
        audits[dest] = audit_urdf(source/relative)
        files[Path(dest)] = relative
        for mesh in ET.parse(source/relative).findall(".//mesh"):
            name = Path(mesh.get("filename"))
            if name.is_absolute() or len(name.parts) != 2 or name.parts[0] != "meshes" or ".." in name.parts:
                raise ValueError("unexpected donor mesh path")
            files[name] = DESCRIPTION/name
    # Verify every copied file against the pinned Git tree, not merely HEAD.
    tree = subprocess.check_output(["git", "-C", str(source), "ls-tree", "-r", SOURCE_SHA], text=True)
    blobs = {line.split("\t", 1)[1]: line.split()[2] for line in tree.splitlines()}
    for relative in files.values():
        path = source/relative
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"missing or symbolic-link donor file: {relative}")
        raw = path.read_bytes()
        blob = hashlib.sha1(b"blob "+str(len(raw)).encode()+b"\0"+raw).hexdigest()
        if blobs.get(str(relative)) != blob:
            raise ValueError(f"source file differs from pinned Git tree: {relative}")
    hands = {side: hand_summary(audits[f"FTP_{side}_hand.urdf"], side) for side in ("left", "right")}
    full = audits[FULL_URDF]
    if full["authored_revolute_count"] != 53 or full["independent_command_count"] != 41:
        raise ValueError("unexpected merged donor joint counts")
    output.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for dest, relative in files.items():
        path = output/dest
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/relative, path)
        hashes[str(dest)] = dict(source_path=str(relative), sha256=sha256(path), bytes=path.stat().st_size)
    benches = {}
    for side in ("left", "right"):
        name = f"FTP_{side}_hand_bench.urdf"
        benches[name] = bench_hand(output/f"FTP_{side}_hand.urdf", output/name, side)
        benches[name]["sha256"] = sha256(output/name)
    manifest = dict(schema_version=1, artifact_kind="unqualified_FTP_simulation_donor",
                    source_url=SOURCE_URL, source_commit=SOURCE_SHA, source_license="BSD-3-Clause",
                    source_files_verified=True, exact_RH56E2_equivalence=False,
                    simulation_qualified=False, hardware_authorized=False,
                    source_geometry_modified=False, source_masses_modified=False,
                    source_sensor_frames_inherited=True, tactile_sensor_data_implemented=False,
                    merged_urdf=FULL_URDF, hands=hands, bench_derivatives=benches,
                    merged_mechanical_audit=full, files=hashes)
    (output/"donor_manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = build(args.source_root, args.output)
    print(json.dumps({"manifest": str(args.output/"donor_manifest.json"),
                      "merged_urdf": str(args.output/FULL_URDF),
                      "exact_RH56E2_equivalence": result["exact_RH56E2_equivalence"],
                      "mass_kg": result["merged_mechanical_audit"]["authored_mass_kg"]}, indent=2))


if __name__ == "__main__":
    main()
