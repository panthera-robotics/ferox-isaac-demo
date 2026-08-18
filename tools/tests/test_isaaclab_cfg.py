"""DT7-lite: the Isaac Lab cfg must agree with the policy and the contract.

Runs WITHOUT Isaac Lab -- isaac/twin/isaaclab/g1_dex5.py is plain data until a
build_* function is called, and these tests never call one. They check the three
agreements that would otherwise only surface as a silently-wrong policy:

  * the 29 body joints, in the SIM order, are exactly what deploy.yaml's
    joint_ids_map produces from its SDK order;
  * the actuator gains match deploy.yaml joint for joint;
  * the camera matches the contract.

    python3 tools/tests/test_isaaclab_cfg.py
"""
from __future__ import annotations

import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.join(REPO, "isaac", "twin", "isaaclab"))

import twin_contract  # noqa: E402
import g1_dex5 as cfg  # noqa: E402

G1_CONTRACT = os.path.join(REPO, "isaac", "twin", "g1_contract.yaml")
LOCOMOTION = os.path.join(os.path.dirname(REPO), "ferox-g1-locomotion")
DEPLOY = os.path.join(LOCOMOTION, "policy", "params", "deploy.yaml")
ENV = os.path.join(LOCOMOTION, "policy_g1_baseline", "params", "env.yaml")


class _TupleTolerantLoader(yaml.SafeLoader):
    """env.yaml carries python/tuple tags from Isaac Lab's own dump."""


_TupleTolerantLoader.add_multi_constructor(
    "tag:yaml.org,2002:python/", lambda loader, suffix, node: None)


def _deploy():
    return yaml.safe_load(open(DEPLOY, encoding="utf-8"))


def _env_robot():
    with open(ENV, encoding="utf-8") as fh:
        return yaml.load(fh, Loader=_TupleTolerantLoader)["scene"]["robot"]


def _skipped(reason):
    print(f"  SKIP  {reason}")
    return True


# --------------------------------------------------------------- the core claim

def test_body_joint_order_matches_deploy_yaml_map():
    """BODY_JOINTS_SIM == deploy.yaml's joint_ids_map applied to the SDK order.

    This is the assertion the whole file exists for. run.py maps SDK->sim with
    `sim[i] = sdk[joint_ids_map[i]]`, so applying that map to joint_sdk_names must
    reproduce the articulation's own DOF order. If it ever stops doing so, a policy
    trained through this cfg would drive the wrong joints -- and nothing else in the
    stack would notice, because every array would still be 29 long.
    """
    if not os.path.exists(DEPLOY):
        return _skipped("ferox-g1-locomotion not present")
    ids = _deploy()["joint_ids_map"]
    assert len(ids) == 29, len(ids)
    sdk = cfg.BODY_JOINTS_SDK
    assert sorted(ids) == list(range(29)), "joint_ids_map is not a permutation"
    derived = [sdk[i] for i in ids]
    assert derived == cfg.BODY_JOINTS_SIM, (
        "sim joint order disagrees with deploy.yaml:\n"
        + "\n".join(f"  [{i:2d}] cfg={a!r} deploy={b!r}"
                    for i, (a, b) in enumerate(zip(cfg.BODY_JOINTS_SIM, derived))
                    if a != b))


def test_sdk_order_matches_env_yaml():
    """The SDK order is copied from env.yaml, not retyped from memory."""
    if not os.path.exists(ENV):
        return _skipped("ferox-g1-locomotion not present")
    assert _env_robot()["joint_sdk_names"] == cfg.BODY_JOINTS_SDK


def test_body_joint_names_match_the_merged_asset():
    """The same 29 names the Isaac suite asserts against the built USD."""
    isaac_test = os.path.join(TOOLS, "tests", "test_twin_isaac.py")
    src = open(isaac_test, encoding="utf-8").read()
    block = re.search(r"BODY_DOF_ORDER = \[(.*?)\]", src, re.S).group(1)
    names = re.findall(r'"([^"]+)"', block)
    assert names == cfg.BODY_JOINTS_SIM, "cfg and the Isaac asset test disagree"


# ------------------------------------------------------------------- actuators

def _resolve(spec_value, joint):
    """Isaac Lab resolves a dict of regex->value against each joint name."""
    if not isinstance(spec_value, dict):
        return spec_value
    for pattern, value in spec_value.items():
        if re.fullmatch(pattern, joint):
            return value
    return None


def _group_for(joint):
    hits = [name for name, spec in cfg.ACTUATORS.items()
            if any(re.fullmatch(p, joint) for p in spec["joint_names_expr"])]
    return hits


def test_every_body_joint_lands_in_exactly_one_group():
    for joint in cfg.BODY_JOINTS_SIM:
        hits = _group_for(joint)
        assert len(hits) == 1, f"{joint} matched {hits}"


def test_actuator_gains_match_deploy_yaml():
    """Stiffness and damping, joint for joint, against the deployed policy.

    deploy.yaml's stiffness/damping arrays are in SDK order, not sim order --
    run.py converts them with _sdk_to_sim() before handing them to PhysX. So the
    gain for the joint named sdk[k] is stiffness[k]. Indexing them by sim position
    instead reads the right array at the wrong offset and mismatches exactly the
    joints whose SDK and sim positions differ, which is most of them.
    """
    if not os.path.exists(DEPLOY):
        return _skipped("ferox-g1-locomotion not present")
    d = _deploy()
    ids, sdk = d["joint_ids_map"], cfg.BODY_JOINTS_SDK
    bad = []
    for sim_idx, joint in enumerate(cfg.BODY_JOINTS_SIM):
        sdk_idx = ids[sim_idx]
        assert sdk[sdk_idx] == joint
        want_k, want_d = d["stiffness"][sdk_idx], d["damping"][sdk_idx]
        group = cfg.ACTUATORS[_group_for(joint)[0]]
        got_k = _resolve(group["stiffness"], joint)
        got_d = _resolve(group["damping"], joint)
        if got_k != want_k or got_d != want_d:
            bad.append(f"{joint}: cfg k={got_k} d={got_d} vs deploy k={want_k} d={want_d}")
    assert not bad, "actuator gains disagree with deploy.yaml:\n  " + "\n  ".join(bad)


# --------------------------------------------------------- RULE-HAND-NAME (C-14)

def test_hand_group_lists_names_and_never_a_pattern():
    """The hand group must be 40 literal names -- no regex metacharacters.

    A pattern like `Pitch_.*` would sweep both hands in articulation order, which is
    interleaved and non-contiguous. That is C-14, and it is the one place in this cfg
    where Isaac Lab's regex convenience is actively dangerous.
    """
    exprs = cfg.ACTUATORS["dex5_1p"]["joint_names_expr"]
    assert len(exprs) == 40, len(exprs)
    assert exprs == cfg.hand_joint_names(), "hand group is not the canonical order"
    meta = set(".*+?[]()|^$\\")
    for e in exprs:
        assert not (meta & set(e)), f"{e!r} is a pattern, not a name (C-14)"


def test_no_body_group_pattern_can_match_a_hand_joint():
    """The body regexes must not reach into the hands, in either direction."""
    for joint in cfg.hand_joint_names():
        for name, spec in cfg.ACTUATORS.items():
            if name == "dex5_1p":
                continue
            for pattern in spec["joint_names_expr"]:
                assert not re.fullmatch(pattern, joint), \
                    f"body group {name!r} pattern {pattern!r} matches hand joint {joint!r}"
    for joint in cfg.BODY_JOINTS_SIM:
        assert joint not in cfg.ACTUATORS["dex5_1p"]["joint_names_expr"], \
            f"{joint} is a body joint but is listed in the hand group"


def test_hand_joint_names_and_passive_indices():
    for side in ("left", "right"):
        names = cfg.HAND_JOINTS[side]
        assert len(names) == 20, side
        assert len(set(names)) == 20
        suffix = "L" if side == "left" else "R"
        assert all(n.endswith(suffix) for n in names)
    # Unitree's index-12 asymmetry: Link_41L on the left, Roll_41R on the right.
    assert cfg.HAND_JOINTS["left"][12] == "Link_41L"
    assert cfg.HAND_JOINTS["right"][12] == "Roll_41R"
    assert len(cfg.active_hand_joint_names()) == 32
    for side in ("left", "right"):
        passive = {cfg.HAND_JOINTS[side][i] for i in cfg.PASSIVE_INDICES}
        assert not (passive & set(cfg.active_hand_joint_names(side)))


def test_dof_totals():
    assert len(cfg.BODY_JOINTS_SIM) + len(cfg.hand_joint_names()) == cfg.EXPECT_DOF


# ---------------------------------------------------------------------- camera

def test_camera_matches_the_contract():
    c = twin_contract.load(G1_CONTRACT)
    info = [t for t in c["topics"]
            if t["name"].endswith("camera/color/camera_info")][0]["expect"]["camera_info"]
    assert cfg.CAMERA["K"] == info["K"], "K disagrees with the contract"
    assert cfg.CAMERA["width"] == info["width"]
    assert cfg.CAMERA["height"] == info["height"]
    assert cfg.CAMERA["distortion_model"] == info["distortion_model"]
    d435i = [s for s in c["sensors"] if s["name"] == "d435i"][0]
    assert list(cfg.CAMERA["mount_xyz"]) == d435i["pose"]["xyz"]
    assert list(cfg.CAMERA["mount_rpy"]) == d435i["pose"]["rpy"]
    assert cfg.CAMERA["mount_link"] == d435i["pose"].get("parent_link", d435i["parent_link"])
    mp = d435i["model_params"]
    assert [cfg.CAMERA["width"], cfg.CAMERA["height"]] == mp["color_resolution"]
    assert cfg.CAMERA["rate_hz"] == float(mp["color_rate_hz"])
    assert cfg.CAMERA["clipping_range"] == (mp["min_z_m"], mp["clip_distance_m"])


def test_camera_optical_frame_is_the_contract_leaf():
    c = twin_contract.load(G1_CONTRACT)
    info = [t for t in c["topics"]
            if t["name"].endswith("camera/color/camera_info")][0]
    assert cfg.CAMERA["optical_frame"] == info["frame_id"]
    assert cfg.CAMERA["prim_path"].endswith(info["frame_id"])


def test_focal_length_reproduces_fx():
    """Solving focal from K must invert back to fx within a tenth of a pixel."""
    ap, w = 20.955, cfg.CAMERA["width"]
    f = cfg.focal_from_K(cfg.CAMERA["K"], w, ap)
    fx_back = f * w / ap
    assert abs(fx_back - cfg.CAMERA["K"][0]) < 0.1, (f, fx_back)


def test_module_imports_without_isaac_lab():
    """The whole point of the lazy builders."""
    assert "isaaclab" not in sys.modules, "importing the cfg pulled in isaaclab"
    src = open(os.path.join(REPO, "isaac", "twin", "isaaclab", "g1_dex5.py"),
               encoding="utf-8").read()
    top_level = [ln for ln in src.split("\n")
                 if re.match(r"^(import|from)\s+isaaclab", ln)]
    assert not top_level, f"isaaclab imported at module scope: {top_level}"


def main() -> int:
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    failed = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:
            failed.append(name)
            print(f"  FAIL  {name}: {type(exc).__name__}: {exc}")
    print(f"\n{len(tests) - len(failed)}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
