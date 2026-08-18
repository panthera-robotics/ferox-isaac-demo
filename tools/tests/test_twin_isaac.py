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
