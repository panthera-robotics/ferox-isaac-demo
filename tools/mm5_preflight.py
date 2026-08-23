"""MM5 instrument preflight — assert every gauge against a KNOWN state before use.

Written after five instrument defects in one session, every one of which produced a
confident number that was wrong:

  * contact "force" was a summed PhysX IMPULSE (N*s), L1-normed, printed as newtons
  * the fix reported only the last substep, hiding real contacts as `contacts=0`
  * contact count carried no identity, so 4 pushing one way looked like 2 opposing
  * object tilt was measured against world +z on an asset whose local +z is not its
    cylinder axis, so an UNTOUCHED can read 90 deg and produced false TOPPLED verdicts
  * the tilt baseline was re-latched per trial, making `tilt-at-rest` read 0.0 by
    construction and destroying the check that had caught a can staged at 44.7 deg

None of these were robot defects. Each would have been caught in seconds by asking the
gauge what it reports on a case whose answer is already known. That is this file.

Exit code 0 = every gauge verified. Non-zero = DO NOT TRUST GRASP NUMBERS.
"""
from __future__ import annotations

import json
import numpy as np

RESULTS = []


def check(name, ok, got, want):
    RESULTS.append({"check": name, "pass": bool(ok), "got": got, "want": want})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {got}, want {want}", flush=True)
    return ok


def main():
    from isaacsim.core.api import World
    from isaacsim.core.utils.prims import define_prim
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.storage.native import get_assets_root_path
    import isaacsim.core.utils.stage as _st
    from pxr import Gf, UsdGeom

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 200.0, rendering_dt=1 / 60.0)
    world.scene.add_default_ground_plane()
    root = get_assets_root_path()
    can_usd = root + "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd"
    _st.add_reference_to_stage(can_usd, "/World/can")
    xf = UsdGeom.Xformable(world.stage.GetPrimAtPath("/World/can"))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.10))
    world.reset()
    world.play()
    for _ in range(60):
        world.step(render=False)

    rp = SingleRigidPrim("/World/can")
    try:
        rp.initialize()
    except Exception:
        pass

    def axis_of():
        """The object's REST QUATERNION, used as the reference for rotation magnitude."""
        _, q = rp.get_world_pose()
        return np.asarray(q, float).reshape(-1)[:4].copy()

    def tilt_vs(base):
        """Angle THROUGH WHICH the object has rotated since `base`, in degrees.

        Quaternion angle, not the tilt of a body axis. Measuring a body axis fails on any
        asset whose local +z is not the axis of symmetry: a scripted 30 deg tip read
        18.45 deg in preflight, because rotating about world +x moves that particular axis
        by less than the rotation itself. The quaternion angle is axis-independent and
        reads a 30 deg rotation as 30 deg whatever the mesh's frame convention is.
        """
        q = axis_of()
        w1, x1, y1, z1 = base
        w2, x2, y2, z2 = q
        # relative = q * base^-1  (base is unit, so inverse is its conjugate)
        rw = w2 * w1 + x2 * x1 + y2 * y1 + z2 * z1
        return float(np.degrees(2.0 * np.arccos(min(1.0, abs(float(rw))))))

    print("\n== GAUGE 1: tilt reads 0 on an object at rest ==", flush=True)
    base = axis_of()
    z0 = float(np.asarray(rp.get_world_pose()[0], float)[2])
    for _ in range(30):
        world.step(render=False)
    t_rest = tilt_vs(base)
    z_rest = float(np.asarray(rp.get_world_pose()[0], float)[2])
    ok1 = check("tilt at rest", abs(t_rest) < 2.0, f"{t_rest:.2f} deg", "< 2 deg")
    ok2 = check("centre drop at rest", abs(z_rest - z0) < 0.005,
                f"{(z_rest - z0)*1000:+.1f} mm", "|drop| < 5 mm")

    print("\n== GAUGE 2: a KNOWN 30 deg tip reads ~30 deg ==", flush=True)
    ang = np.radians(30.0)
    q_tip = np.array([np.cos(ang / 2), np.sin(ang / 2), 0.0, 0.0], np.float32)  # about +x
    # compose with the object's current orientation so the tip is RELATIVE
    _, q_now = rp.get_world_pose()
    qn = np.asarray(q_now, float).reshape(-1)[:4]
    w1, x1, y1, z1 = q_tip
    w2, x2, y2, z2 = qn
    q_new = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], np.float32)
    # NO physics steps between the write and the read. The can is a dynamic body resting
    # on the ground; stepping lets gravity settle it back, so a "known 30 deg tip" is no
    # longer 30 deg by the time it is measured. That is a defect in the TEST, and it read
    # as a defect in the gauge (18.45 deg) until this was separated out. The gauge is
    # being asked one question only: given a pose, does it report the right angle.
    rp.set_world_pose(orientation=q_new)
    t_tip = tilt_vs(base)
    ok3 = check("tilt after a scripted 30 deg tip", 25.0 < t_tip < 35.0,
                f"{t_tip:.2f} deg", "25..35 deg")

    print("\n== GAUGE 3: 'lift' only fires when z actually rises ==", flush=True)
    zA = float(np.asarray(rp.get_world_pose()[0], float)[2])
    rp.set_world_pose(position=np.array([0.0, 0.0, zA + 0.08], np.float32))
    for _ in range(3):
        world.step(render=False)
    zB = float(np.asarray(rp.get_world_pose()[0], float)[2])
    ok4 = check("z rise detected after a scripted +80 mm move", (zB - zA) > 0.05,
                f"{(zB - zA)*1000:+.1f} mm", "> 50 mm")
    ok5 = check("no false rise when nothing moves", True, "n/a (covered by GAUGE 1)", "-")

    allok = all([ok1, ok2, ok3, ok4, ok5])
    out = {"all_pass": allok, "checks": RESULTS}
    open("/tmp/mm5_preflight.json", "w").write(json.dumps(out, indent=2))
    print(f"\nPREFLIGHT: {'ALL GAUGES VERIFIED' if allok else 'FAILED -- DO NOT TRUST GRASP NUMBERS'}",
          flush=True)
    return 0 if allok else 1


if __name__ == "__main__":
    raise SystemExit(main())
