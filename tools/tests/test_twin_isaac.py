"""Twin sensor tests that need a live Isaac Sim. Run via scripts/10_test_twin_isaac.sh.

These exist because the alternative is trusting that an attribute we set is an
attribute that took. Isaac's sensor creation does not raise on a bad config -- it
logs a warning, builds a DEFAULT sensor, and silently renames the prim. Every
assertion below is a read-back.

Kept as a permanent test rather than a one-off probe: the Isaac image is pinned
today, but the whole point of a parity campaign is that the day it moves, we find
out from a failing test and not from a training run.
"""

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import os  # noqa: E402
import sys  # noqa: E402

sys.path.insert(0, "/workspace/ferox_tools")
sys.path.insert(0, "/workspace/ferox_isaac/twin")

import yaml  # noqa: E402
import lidar as twin_lidar  # noqa: E402
from isaacsim.core.utils.stage import create_new_stage  # noqa: E402
from isaacsim.core.utils.prims import define_prim  # noqa: E402

CONTRACTS = "/workspace/ferox_isaac/twin"
RESULTS = []


def check(name, fn):
    try:
        fn()
        RESULTS.append((name, "PASS", ""))
    except Exception as exc:
        RESULTS.append((name, "FAIL", f"{type(exc).__name__}: {exc}"))


def _contract(robot):
    return yaml.safe_load(open(os.path.join(CONTRACTS, f"{robot}_contract.yaml")))


def _mid360(robot, path_name):
    c = _contract(robot)
    spec = [s for s in c["sensors"] if s["name"] == "livox_mid360"][0]
    return twin_lidar.create_mid360(
        "/World/frames", path_name, spec["model_params"],
        translation=(0, 0, 0), orientation=(1, 0, 0, 0)), spec


def test_mid360_attributes_read_back(robot, expect_scan_hz, expect_report_hz):
    """Every omni:sensor attribute we set must read back as we set it."""
    (prim, derived), spec = _mid360(robot, f"mid360_{robot}")
    mp = spec["model_params"]
    got = {a: prim.GetAttribute(a).Get() for a in (
        "omni:sensor:Core:scanRateBaseHz", "omni:sensor:Core:reportRateBaseHz",
        "omni:sensor:Core:numberOfEmitters", "omni:sensor:Core:numberOfChannels",
        "omni:sensor:Core:nearRangeM", "omni:sensor:Core:farRangeM",
        "omni:sensor:Core:rangeAccuracyM", "omni:sensor:Core:scanType",
        "omni:sensor:Core:outputFrameOfReference", "omni:sensor:tickRate",
        "omni:sensor:modelName")}
    assert got["omni:sensor:Core:scanRateBaseHz"] == expect_scan_hz, got
    assert got["omni:sensor:Core:reportRateBaseHz"] == expect_report_hz, got
    assert got["omni:sensor:Core:numberOfEmitters"] == 40, got
    assert got["omni:sensor:Core:numberOfChannels"] == 40, got
    assert abs(got["omni:sensor:Core:nearRangeM"] - mp["range_m"][0]) < 1e-4, got
    assert abs(got["omni:sensor:Core:farRangeM"] - mp["range_m"][1]) < 1e-4, got
    assert abs(got["omni:sensor:Core:rangeAccuracyM"] - mp["range_accuracy_m"]) < 1e-4, got
    assert got["omni:sensor:Core:scanType"] == "ROTARY", got
    # The real cloud is raw, in the SENSOR frame, and not self-filtered -- which is
    # exactly why p2l needs range_min 0.30. WORLD here would silently pre-transform.
    assert got["omni:sensor:Core:outputFrameOfReference"] == "SENSOR", got
    # Campaign 4.3: tickRate must equal scanRateBaseHz or a published message is a
    # partial sweep rather than a full revolution.
    assert abs(got["omni:sensor:tickRate"] - float(expect_scan_hz)) < 1e-6, got
    assert got["omni:sensor:modelName"] == "Livox_Mid360", got
    # Points per second must equal the datasheet figure exactly.
    assert derived["points_per_second"] == mp["points_per_second"], derived


def test_per_emitter_arrays_are_resized(robot):
    """Every per-emitter array must be length N, not the base asset's 128.

    Setting elevationDeg to 40 while horOffsetM stays at 128 leaves the sensor
    internally inconsistent and the renderer's behaviour undefined.
    """
    (prim, _), _ = _mid360(robot, f"arrays_{robot}")
    s = "omni:sensor:Core:emitterState:s001:"
    for a in ("azimuthDeg", "elevationDeg", "fireTimeNs", "channelId",
              "distanceCorrectionM", "focalDistM", "focalSlope", "horOffsetM",
              "reportRateDiv", "vertOffsetM"):
        v = prim.GetAttribute(s + a).Get()
        assert v is not None and len(v) == 40, f"{a}: {None if v is None else len(v)} != 40"


def test_elevation_span_matches_datasheet(robot):
    (prim, _), spec = _mid360(robot, f"elev_{robot}")
    lo, hi = spec["model_params"]["elevation_deg"]
    el = list(prim.GetAttribute("omni:sensor:Core:emitterState:s001:elevationDeg").Get())
    assert abs(el[0] - lo) < 1e-3, el[0]
    assert abs(el[-1] - hi) < 1e-3, el[-1]
    assert all(el[i] < el[i + 1] for i in range(len(el) - 1)), "elevations not monotonic"


def test_unknown_config_is_never_silently_accepted():
    """The trap this module exists for.

    IsaacSensorCreateRtxLidar does NOT raise on an unknown config: it warns,
    builds a full default sensor, and renames the prim. create_mid360 must catch
    that. If this test ever fails, Isaac changed its behaviour and lidar.py's
    guarantee needs re-deriving.
    """
    import omni.kit.commands
    _, bad = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar", path="trap", parent="/World/frames",
        config="Definitely_Not_A_Real_Sensor")
    assert bad is not None, "Isaac now returns None for an unknown config (behaviour changed)"
    assert str(bad.GetPath()) != "/World/frames/trap", (
        "Isaac no longer renames on config failure -- lidar.py's path guard is now "
        "the only thing standing between us and a silent default sensor")


def test_camera_intrinsics_solve_from_contract_K():
    """K -> aperture/focal -> K must round-trip inside 1 %."""
    sys.path.insert(0, "/workspace/ferox_isaac/twin")
    import sensors as twin_sensors
    from pxr import UsdGeom
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.DefinePrim("/World/frames/testcam", "Camera")
    UsdGeom.Camera(prim).GetHorizontalApertureAttr().Set(20.955)

    c = _contract("g1")
    ci = [t for t in c["topics"]
          if t["name"].endswith("camera/color/camera_info")][0]["expect"]["camera_info"]
    K, w, h = ci["K"], ci["width"], ci["height"]
    got = twin_sensors.set_camera_intrinsics(prim, K, w, h)
    for label, want, have in (("fx", K[0], got["fx"]), ("fy", K[4], got["fy"])):
        assert abs(have - want) / want <= 0.01, f"{label} {have} vs {want}"
    assert abs(got["cx"] - K[2]) < 0.5 and abs(got["cy"] - K[5]) < 0.5, got
    # The FOV the contract's K implies -- reported, not asserted, because K wins.
    print(f"  [info] contract K implies HFOV {got['hfov_deg']:.2f} deg, "
          f"VFOV {got['vfov_deg']:.2f} deg")


# --- merged G1+Dex5 asset (DT3) ---------------------------------------------
# Drift tripwires for the MERGED asset. The four numbers below are the ones that
# were green on the USD-composition attempt that could not move a finger -- joint
# count, mass, mount offset, limits -- plus the one that was not: DOF. Only the
# articulation DOF count proves the hands are part of the robot, so it leads.
HAND_STUB = "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd"
EXPECT_DOF = 69                      # 29 body + 2 x 20 hand
EXPECT_BODIES = 79
EXPECT_TOTAL_MASS = 35.004757        # 33.001142 body (rubber caps removed) + hands
EXPECT_HAND_MASS = 1.025045 + 0.978570
HAND_MOUNT = {
    "left":  ("left_hand_palm_joint",  "left_wrist_yaw_link",  "base_link00L",
              (0.0415, 0.003, 0.0)),
    "right": ("right_hand_palm_joint", "right_wrist_yaw_link", "base_link00",
              (0.0415, -0.003, 0.0)),
}
# The flange offset is checked on the JOINT, where it is authored, not by composing
# world transforms down the chain. USD stores xform ops as float32 and the palm sits
# ~10 links from the root, so the composed route accumulates ~8 um -- real, harmless,
# and enough to fail a micron-level assertion for a reason that has nothing to do
# with the mount. CHAIN_TOL is what the composed cross-check is allowed.
CHAIN_TOL = 5e-5
# The pre-hand body DOF order. The walk policy indexes these, so the merge is only
# safe while this list is reproduced exactly.
BODY_DOF_ORDER = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint", "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint", "left_ankle_roll_joint",
    "right_ankle_roll_joint", "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint", "left_wrist_roll_joint",
    "right_wrist_roll_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
]

_ART = {}


def _articulation():
    """One physics-enabled load of the merged asset, shared by the tests below."""
    if "art" not in _ART:
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.prims import Articulation
        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        add_reference_to_stage(HAND_STUB, "/World/g1_dex5")
        world.reset()
        art = Articulation("/World/g1_dex5")
        art.initialize()
        _ART["art"] = art
        _ART["world"] = world
    return _ART["art"]


def test_merged_asset_is_one_articulation_of_69_dof():
    art = _articulation()
    assert len(art.dof_names) == EXPECT_DOF, \
        f"{len(art.dof_names)} DOF, expected {EXPECT_DOF} -- the hands are in the " \
        "stage but not in the articulation"
    assert len(art.body_names) == EXPECT_BODIES, \
        f"{len(art.body_names)} bodies, expected {EXPECT_BODIES}"


def test_body_dof_order_unchanged_by_the_merge():
    got = list(_articulation().dof_names)[:29]
    assert got == BODY_DOF_ORDER, \
        f"body DOF order changed: first difference at " \
        f"{next(i for i, (a, b) in enumerate(zip(got, BODY_DOF_ORDER)) if a != b)}"


def test_every_hand_joint_is_a_dof():
    import xml.etree.ElementTree as ET
    names = set(_articulation().dof_names)
    for side, path in (("L", "/tmp/dex5_urdf/Dex5-URDF-L/Dex5-URDF-L.urdf"),
                       ("R", "/tmp/dex5_urdf/Dex5-URDF-R/Dex5-URDF-R.urdf")):
        if not os.path.exists(path):
            continue                     # URDFs are staged only by the import script
        root = ET.parse(path).getroot()
        want = [j.get("name") for j in root.findall("joint")
                if j.get("type") in ("revolute", "continuous")]
        missing = [n for n in want if n not in names]
        assert not missing, f"{side}: {missing} absent from the articulation"


def test_hand_mass_and_mount_survive_the_merge():
    from pxr import Usd, UsdGeom, UsdPhysics
    st = Usd.Stage.Open(HAND_STUB)
    root = st.GetDefaultPrim()
    assert root.GetName() == "g1_29dof_rev_1_0", \
        f"default prim {root.GetName()!r} -- the sensor layer overrides " \
        "/g1_29dof_rev_1_0/... and would stop composing"
    total = 0.0
    for prim in st.Traverse():
        if prim.HasAPI(UsdPhysics.MassAPI):
            m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
            if m:
                total += float(m)
    assert abs(total - EXPECT_TOTAL_MASS) < 1e-4, \
        f"total mass {total:.6f} != {EXPECT_TOTAL_MASS:.6f}"
    joints = {p.GetName(): p for p in st.Traverse()
              if p.IsA(UsdPhysics.FixedJoint)}
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for side, (jname, parent, palm, want) in HAND_MOUNT.items():
        jp = joints.get(jname)
        assert jp is not None, f"{side}: fixed joint {jname} missing"
        pos = UsdPhysics.FixedJoint(jp).GetLocalPos0Attr().Get()
        for i, axis in enumerate("xyz"):
            assert abs(pos[i] - want[i]) < 1e-6, \
                f"{side} {axis} on {jname}: {pos[i]:.6f} != {want[i]:.6f}"
        # Cross-check through the composed chain, at float32 tolerance.
        pp = st.GetPrimAtPath(f"/{root.GetName()}/{parent}")
        cp = st.GetPrimAtPath(f"/{root.GetName()}/{palm}")
        assert cp and cp.IsValid(), f"{side}: palm link {palm} missing"
        rel = (cache.GetLocalToWorldTransform(cp)
               * cache.GetLocalToWorldTransform(pp).GetInverse()).ExtractTranslation()
        for i, axis in enumerate("xyz"):
            assert abs(rel[i] - want[i]) < CHAIN_TOL, \
                f"{side} {axis} composed: {rel[i]:.6f} != {want[i]:.6f} " \
                f"(tol {CHAIN_TOL})"


# --- DT3 mount defect: the flange ROTATION -----------------------------------
# The companion to test_hand_mass_and_mount_survive_the_merge, which checks the
# flange POSITION and passed throughout the period both hands were rotated 90 deg.
# The Dex5 root frame is not wrist-aligned -- its fingers run along its own +Y
# while the G1's forearm runs along wrist +X -- so an identity flange rpy left the
# fingers pointing laterally outward, the palm forward and the thumb down, with
# every position check still exact to 0.0000 mm. Same shape as C-21.
#
# Asserted here on the BUILT USD, not on the intermediate URDF, because the USD is
# what the sim loads. The expected rotation is derived by tools/derive_hand_flange.py
# from Dex5 geometry against the convention in Unitree's own G1 + Inspire assembly,
# and that derivation reproduces Unitree's published flange rpy to 1e-16.
HAND_FLANGE_R = [[0.0, 1.0, 0.0],      # hand +X -> wrist +Z
                 [0.0, 0.0, 1.0],      # hand +Y -> wrist +X  (fingers on the forearm)
                 [1.0, 0.0, 0.0]]      # hand +Z -> wrist +Y  (palm across the body)
HAND_ANATOMY = {
    "L": {"wrist": "left_wrist_yaw_link", "root": "base_link00L",
          "mid_mcp": "Link_31L", "mid_tip": "Link_34L",
          "thumb_root": "Link_11L", "thumb_tip": "Link_14L"},
    "R": {"wrist": "right_wrist_yaw_link", "root": "base_link00",
          "mid_mcp": "Link_31R", "mid_tip": "Link_34R",
          "thumb_root": "Link_11R", "thumb_tip": "Link_14R"},
}
FINGER_TOL_DEG = 5.0


def test_hand_mount_rotation_and_anatomy():
    import numpy as np
    from pxr import Usd, UsdGeom

    st = Usd.Stage.Open(HAND_STUB)
    root = st.GetDefaultPrim().GetName()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    def xf(link):
        prim = st.GetPrimAtPath(f"/{root}/{link}")
        assert prim and prim.IsValid(), f"link {link} missing from the built USD"
        return cache.GetLocalToWorldTransform(prim)

    def R_of(m):
        return np.array([[m[0][0], m[1][0], m[2][0]],
                         [m[0][1], m[1][1], m[2][1]],
                         [m[0][2], m[1][2], m[2][2]]])

    def t_of(m):
        return np.array([m[3][0], m[3][1], m[3][2]])

    def ang(a, b):
        c = float(np.clip(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)), -1, 1))
        return np.degrees(np.arccos(c))

    for side, spec in HAND_ANATOMY.items():
        Mw, Mh = xf(spec["wrist"]), xf(spec["root"])
        Rw = R_of(Mw)
        # The flange rotation itself, wrist -> hand root.
        R_rel = Rw.T @ R_of(Mh)
        assert np.allclose(R_rel, np.array(HAND_FLANGE_R), atol=1e-5), (
            f"{side}: wrist->hand rotation is not the derived flange permutation "
            f"-- DT3 mount regression. got\n{np.round(R_rel, 6)}")

        # And the anatomy that rotation is FOR, measured on the same USD.
        in_wrist = lambda link: Rw.T @ (t_of(xf(link)) - t_of(Mw))  # noqa: E731
        finger = in_wrist(spec["mid_tip"]) - in_wrist(spec["mid_mcp"])
        thumb = in_wrist(spec["thumb_tip"]) - in_wrist(spec["thumb_root"])
        f_err = ang(finger, np.array([1.0, 0.0, 0.0]))
        t_err = ang(thumb, np.array([0.0, 0.0, 1.0]))
        assert f_err <= FINGER_TOL_DEG, (
            f"{side}: middle finger is {f_err:.2f} deg off wrist +X (the forearm "
            f"axis), tolerance {FINGER_TOL_DEG}")
        assert t_err <= 5.0, f"{side}: thumb is {t_err:.2f} deg off wrist +Z"
    # Chirality, on the thumb offsets rather than the frames: the left thumb sits
    # on the hand's -Z side and the right on +Z, so in the WRIST frame the two
    # thumb roots must fall on opposite sides of the palm plane. Both on the same
    # side means the pair is swapped -- invisible to every per-hand check above.
    lat = {}
    for side, spec in HAND_ANATOMY.items():
        Mw = xf(spec["wrist"])
        Rw, tw = R_of(Mw), t_of(Mw)
        lat[side] = float((Rw.T @ (t_of(xf(spec["thumb_root"])) - tw))[1])
    assert lat["L"] * lat["R"] < 0, (
        "both thumbs sit on the same side of their palms in the wrist frame "
        f"({lat}) -- the left and right hands are swapped")


def test_rubber_hand_caps_are_gone():
    from pxr import Usd
    st = Usd.Stage.Open(HAND_STUB)
    left = [str(p.GetPath()) for p in st.Traverse() if "rubber_hand" in p.GetName()]
    assert not left, f"the Dex5 replaces the rubber caps, but they are still here: {left}"


# --- C-21: the camera's optical-to-USD rotation ------------------------------
# The root-cause guard. C-21 was Isaac's Camera wrapper applying its own
# world-to-USD-camera conversion ON TOP of a hand-written 180-degree flip, so the
# prim ended up with a 120-degree permutation and the camera pointed out of the
# robot's right side. Every string-level check still passed. This asserts the
# rotation itself, offline and deterministically, so the regression cannot return.
def test_camera_optical_to_usd_rotation_is_exactly_rx180():
    import numpy as np
    from pxr import Usd, UsdGeom
    import sensors as twin_sensors

    contract = twin_contract_load("g1")
    root = "/World/G1"
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api import World
    world = World(stage_units_in_meters=1.0)
    add_reference_to_stage(
        "/workspace/ferox_isaac/assets/g1_dex5/g1_dex5_1p.usd", root)
    world.reset()
    cam, _ = twin_sensors.create_camera(contract, root)
    stage = world.stage
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    optical = (f"{root}/torso_link/camera_link/camera_color_frame"
               f"/camera_color_optical_frame")

    def R_of(path):
        m = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        return np.array([[m[0][0], m[1][0], m[2][0]],
                         [m[0][1], m[1][1], m[2][1]],
                         [m[0][2], m[1][2], m[2][2]]])

    R_child = R_of(optical).T @ R_of(f"{optical}/camera")
    want = np.diag([1.0, -1.0, -1.0])          # Rx(180)
    assert np.allclose(R_child, want, atol=1e-6), (
        "camera prim local rotation is not Rx(180) -- C-21 regression. got\n"
        f"{np.round(R_child, 6)}")
    # The USD camera looks along its own -Z; that must be the optical frame's +Z.
    view_in_optical = R_child @ np.array([0.0, 0.0, -1.0])
    assert np.allclose(view_in_optical, [0.0, 0.0, 1.0], atol=1e-6), view_in_optical


def twin_contract_load(robot):
    import twin_contract
    return twin_contract.load(f"{CONTRACTS}/{robot}_contract.yaml")


def main():
    create_new_stage()
    define_prim("/World", "Xform")
    define_prim("/World/frames", "Xform")

    check("mid360 g1 attributes read back", lambda: test_mid360_attributes_read_back("g1", 10, 5000))
    check("mid360 go2 attributes read back", lambda: test_mid360_attributes_read_back("go2", 20, 5000))
    check("mid360 g1 per-emitter arrays resized", lambda: test_per_emitter_arrays_are_resized("g1"))
    check("mid360 g1 elevation span", lambda: test_elevation_span_matches_datasheet("g1"))
    check("unknown lidar config never silently accepted", test_unknown_config_is_never_silently_accepted)
    check("camera intrinsics solve from contract K", test_camera_intrinsics_solve_from_contract_K)
    check("merged asset is one articulation of 69 dof",
          test_merged_asset_is_one_articulation_of_69_dof)
    check("body dof order unchanged by the merge", test_body_dof_order_unchanged_by_the_merge)
    check("every hand joint is a dof", test_every_hand_joint_is_a_dof)
    check("hand mass and mount survive the merge", test_hand_mass_and_mount_survive_the_merge)
    check("rubber hand caps are gone", test_rubber_hand_caps_are_gone)
    check("hand mount rotation and anatomy", test_hand_mount_rotation_and_anatomy)
    check("camera optical->USD rotation is exactly Rx(180)",
          test_camera_optical_to_usd_rotation_is_exactly_rx180)

    out = open("/tmp/twin_isaac_tests.txt", "w")
    failed = 0
    for name, status, detail in RESULTS:
        line = f"  {status}  {name}" + (f"  -- {detail}" if detail else "")
        out.write(line + "\n")
        if status == "FAIL":
            failed += 1
    summary = f"\n{len(RESULTS) - failed}/{len(RESULTS)} passed"
    out.write(summary + "\n")
    out.close()
    app.close()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
