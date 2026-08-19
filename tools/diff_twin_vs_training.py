#!/usr/bin/env python3
"""MM1b pre-flight: what does the twin's physics differ from the training env's?

Required before the retrain. Everything the policy learns is conditioned on these
numbers, so a retrain that inherits a mismatch just relocates the problem.

The comparison is asymmetric on purpose. Isaac Lab does NOT use the USD's drive
gains, armature or materials -- ArticulationCfg OVERRIDES them at spawn, and event
terms randomise friction every episode. So the USD is read for what it alone
decides (collision geometry, masses), and the config is read for what it overrides.

Reference: unitree_rl_lab
  assets/robots/unitree.py            UNITREE_G1_29DOF_CFG
  tasks/.../g1/29dof/velocity_env_cfg.py  terrain + events
"""
from __future__ import annotations

import argparse
import json
import os
import re

from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})

from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

# --- what unitree_rl_lab imposes at spawn, quoted from the config ------------
TRAIN = {
    "armature_all_joints": 0.01,
    "stiffness": {"hip_pitch": 100.0, "hip_yaw": 100.0, "waist_yaw": 200.0,
                  "hip_roll": 100.0, "knee": 150.0, "shoulder": 40.0,
                  "elbow": 40.0, "ankle": 40.0, "wrist_roll": 40.0,
                  "wrist_pitch": 40.0, "wrist_yaw": 40.0},
    "damping": {"hip_pitch": 2.0, "hip_yaw": 2.0, "waist_yaw": 5.0,
                "hip_roll": 2.0, "knee": 4.0, "shoulder": 1.0, "elbow": 1.0,
                "ankle": 2.0, "wrist_roll": 1.0, "wrist_pitch": 1.0,
                "wrist_yaw": 1.0},
    "effort_limit": {"hip_pitch": 88, "hip_yaw": 88, "waist_yaw": 88,
                     "hip_roll": 139, "knee": 139, "shoulder": 25, "elbow": 25,
                     "ankle": 25, "wrist_roll": 25, "wrist_pitch": 5,
                     "wrist_yaw": 5},
    "ground_material": {"static_friction": 1.0, "dynamic_friction": 1.0,
                        "restitution": 0.0, "friction_combine": "multiply"},
    "friction_randomisation": {"static": (0.3, 1.0), "dynamic": (0.3, 1.0),
                               "restitution": (0.0, 0.0)},
    "terrain": "generator COBBLESTONE_ROAD_CFG (50% flat sub-terrain)",
}

GROUPS = [("hip_pitch", "hip_pitch"), ("hip_yaw", "hip_yaw"),
          ("hip_roll", "hip_roll"), ("knee", "knee"), ("ankle", "ankle"),
          ("shoulder", "shoulder"), ("elbow", "elbow"),
          ("waist_yaw", "waist_yaw"), ("wrist_roll", "wrist_roll"),
          ("wrist_pitch", "wrist_pitch"), ("wrist_yaw", "wrist_yaw")]


def group_of(name):
    for key, pat in GROUPS:
        if pat in name:
            return key
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True)
    ap.add_argument("--out", default="/out/twin_vs_training.md")
    a = ap.parse_args()

    stage = Usd.Stage.Open(a.usd)
    if stage is None:
        print("could not open", a.usd)
        return 1

    joints, feet, mats = {}, [], []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if prim.IsA(UsdPhysics.RevoluteJoint):
            nm = prim.GetName()
            arm = None
            if prim.HasAPI(PhysxSchema.PhysxJointAPI):
                at = prim.GetAttribute("physxJoint:armature")
                arm = at.Get() if at and at.IsValid() else None
            d = UsdPhysics.DriveAPI(prim, "angular")
            joints[nm] = {
                "armature": arm,
                "stiffness": d.GetStiffnessAttr().Get() if d else None,
                "damping": d.GetDampingAttr().Get() if d else None,
                "max_force": d.GetMaxForceAttr().Get() if d else None,
            }
        if prim.HasAPI(UsdPhysics.CollisionAPI) and (
                "ankle_roll" in p or "foot" in p.lower()):
            mca = UsdPhysics.MeshCollisionAPI(prim)
            feet.append({
                "path": p, "type": prim.GetTypeName(),
                "approximation": (mca.GetApproximationAttr().Get()
                                  if prim.HasAPI(UsdPhysics.MeshCollisionAPI)
                                  else "(not a mesh collider)"),
            })
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            m = UsdPhysics.MaterialAPI(prim)
            mats.append({"path": p,
                         "static": m.GetStaticFrictionAttr().Get(),
                         "dynamic": m.GetDynamicFrictionAttr().Get(),
                         "restitution": m.GetRestitutionAttr().Get()})

    L = []
    L.append("# Twin USD vs unitree_rl_lab training env — MM1b pre-flight\n")
    L.append(f"USD: `{a.usd}`  ·  revolute joints found: **{len(joints)}**\n")

    L.append("## 1. Armature\n")
    arms = {k: v["armature"] for k, v in joints.items()}
    nonzero = {k: v for k, v in arms.items() if v not in (None, 0.0)}
    L.append(f"- training imposes **{TRAIN['armature_all_joints']}** on every actuator group")
    L.append(f"- twin USD: {len(nonzero)} of {len(arms)} joints carry a non-zero armature\n")
    L.append("**MATCH**\n" if len(nonzero) == len(arms) else
             "**DIFF — the twin USD's armature is unset/zero.** Isaac Lab overrides it "
             "at spawn, so TRAINING is unaffected; the gap is on the twin side, and "
             "`TWIN_ARMATURE=0.01` exists to close it. MM1 §2.11 measured that it does "
             "not fix yaw, and MM1 §3 measured that it makes walking worse.\n")

    L.append("## 2. Drive gains (what the USD says vs what training imposes)\n")
    L.append("| group | USD stiffness | training | USD damping | training | USD maxForce | training effort |")
    L.append("|---|---|---|---|---|---|---|")
    seen = set()
    for nm, v in sorted(joints.items()):
        g = group_of(nm)
        if g is None or g in seen:
            continue
        seen.add(g)
        L.append(f"| {g} | {v['stiffness']} | {TRAIN['stiffness'].get(g)} | "
                 f"{v['damping']} | {TRAIN['damping'].get(g)} | "
                 f"{v['max_force']} | {TRAIN['effort_limit'].get(g)} |")
    L.append("\nIsaac Lab replaces these at spawn from `ImplicitActuatorCfg`, and "
             "`run.py` replaces them at load from `deploy.yaml`. Neither side uses the "
             "USD values, so a difference here is not itself a sim-to-sim gap — what "
             "matters is whether `deploy.yaml` equals the training config.\n")

    L.append("## 3. Friction\n")
    L.append("| | static | dynamic | restitution |")
    L.append("|---|---|---|---|")
    L.append(f"| training ground material | {TRAIN['ground_material']['static_friction']} | "
             f"{TRAIN['ground_material']['dynamic_friction']} | "
             f"{TRAIN['ground_material']['restitution']} |")
    fr = TRAIN["friction_randomisation"]
    L.append(f"| training, **randomised every episode** | {fr['static']} | {fr['dynamic']} | {fr['restitution']} |")
    for m in mats[:6]:
        L.append(f"| twin `{m['path']}` | {m['static']} | {m['dynamic']} | {m['restitution']} |")
    if not mats:
        L.append("| twin USD | *(no UsdPhysics.MaterialAPI found)* | | |")
    L.append("\n**This is the important one.** Training does not train against a single "
             "friction: an event term samples static and dynamic friction in "
             "**[0.3, 1.0]** every episode, combined `multiply` on top of a 1.0/1.0 "
             "ground. PhysX's default 0.5 — what the twin used before "
             "`TWIN_CONTACT_MATERIAL` — sits INSIDE that training distribution. "
             "Forcing 1.0 puts the twin at the distribution's edge, which is "
             "consistent with MM1 §3 measuring the walk as *worse* with the flag on. "
             "The retrain should keep this randomisation; the twin should not pin "
             "friction to either end of it.\n")

    L.append("## 4. Terrain\n")
    L.append(f"- training: **{TRAIN['terrain']}**")
    L.append("- twin: a single flat floor\n")
    L.append("Half the training sub-terrains are flat, so flat is in-distribution. "
             "Untested as a yaw cause (MM1 §2.12) and left alone here.\n")

    L.append("## 5. Foot collision geometry\n")
    if feet:
        L.append("| prim | type | approximation |")
        L.append("|---|---|---|")
        for f in feet:
            L.append(f"| `{f['path']}` | {f['type']} | {f['approximation']} |")
    else:
        L.append("*(no ankle_roll/foot collider found — check the link naming)*")
    L.append("\n`UNITREE_MODEL_DIR` in `unitree_rl_lab` is the literal placeholder "
             "`path/to/unitree_model`, and our G1 USD was staged there (MM1 §2.9). "
             "Training and the twin therefore load **the same mesh**, so foot "
             "collision cannot differ between them — it is shared by construction. "
             "That closes the last of MM1 §2.12's untested candidates that the retrain "
             "could inherit.\n")

    txt = "\n".join(L) + "\n"
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, "w").write(txt)
    print(txt)
    return 0


if __name__ == "__main__":
    rc = main()
    _app.close()
    raise SystemExit(rc)
