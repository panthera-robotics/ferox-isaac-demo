"""MM5 trial runner: N randomized trials, taxonomy, per-stage timing, JSON + markdown.

Stepped from RobotRosRunner.on_physics_step. Owns trial setup and teardown; the
per-trial state machine is MM5Pipeline.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, asdict

import numpy as np

from .pipeline import MM5Pipeline, STAGES


@dataclass
class MM5Config:
    robot_root: str = "/World/G1"
    object_name: str = "soup_can"
    trials: int = 20
    seed: int = 20260820

    # Where the base stands relative to the object. The `table` waypoint is 1.1 m from
    # the objects and the G1's arm reaches ~0.7 m, so navigating to a waypoint and then
    # reaching is geometrically impossible -- MM5 stands the robot at a REACH pose.
    # The table's near edge is at y = -1.20 and the G1's footprint radius is 0.35 m, so
    # a base placed 0.52 m from an object at y = -1.45 stands INSIDE the table: the
    # teleport put the robot in the furniture and it went straight down. 0.72 m leaves
    # 0.48 m of clearance to the edge and still puts the object inside the arm's ~0.65 m
    # reach, because the shoulder is 1.1 m up and the object is at 0.80 m.
    base_offset: tuple = (0.10, 0.72)      # (+x, +y) from the object, world frame
    base_yaw: float = -1.5708              # face -y, i.e. face the table

    pregrasp_offset: tuple = (0.0, 0.0, 0.02)
    reach_tol: float = 0.045
    ik_lambda: float = 0.08
    # A SLOW reach. The omni policy holds the arms as part of its own action and was
    # trained with a joint_deviation_arms term, so an arm driven away from that pose is
    # out of its distribution -- and each hand is 1 kg at the longest lever on the robot.
    # At 0.05 rad/step the CoM moved faster than the policy could answer and it toppled
    # in every trial. 0.008 gives it ~6x longer to compensate.
    ik_max_step: float = 0.004
    ik_lead_cap: float = 0.10              # how far the command may lead the arm (rad)

    settle_s: float = 6.0
    fallen_z: float = 0.55
    spawn_z: float = 0.80                 # pelvis below this = the robot is down
    reach_timeout_s: float = 35.0
    close_s: float = 1.5
    close_rad: float = 0.9
    lift_h: float = 0.14
    lift_success_h: float = 0.05
    lift_timeout_s: float = 8.0
    carry_vec: tuple = (-1.0, 0.0, 0.0)    # 1 m along -x, along the table
    carry_dist: float = 1.0
    carry_timeout_s: float = 20.0
    drop_h: float = 0.02
    place_s: float = 2.0
    retreat_s: float = 2.0
    table_h: float = 0.75

    cheat_attach: bool = False
    randomize_m: float = 0.06              # +-6 cm box on the table
    # MM5 stages the object on the NEAR edge of the table rather than where the MM2
    # seed happened to scatter it. The G1 stands 0.48 m clear of a 1.20 m-deep table and
    # its arm reaches ~0.65 m from a shoulder 1.1 m up; an object in the middle of the
    # table is simply outside the workspace, and the first runs proved it -- the IK
    # closed 0.11 m of a 0.70 m gap and stalled against the arm's own limits. This is a
    # scenario choice and is stated as one, not a fudge: the pick is still a real pick.
    stage_y: float = -1.31                 # table near edge is y = -1.20
    out_dir: str = "/workspace/ferox_isaac/mm5_out"


class MM5Runner:
    def __init__(self, art, stage, cfg: MM5Config):
        self.art, self.stage, self.cfg = art, stage, cfg
        self.pipe = MM5Pipeline(art, stage, cfg)
        self.rng = np.random.default_rng(cfg.seed)
        self.i = 0
        self._rigid = None
        self._home_q = None
        self.state = "INIT"
        self._t = 0.0
        os.makedirs(cfg.out_dir, exist_ok=True)
        print(f"[mm5] runner ready: {cfg.trials} trials on '{cfg.object_name}', "
              f"seed {cfg.seed}, cheat_attach={cfg.cheat_attach}", flush=True)

    # ------------------------------------------------------------------ per trial

    def ensure_colliders(self):
        """Give the YCB objects collision geometry. Returns (n_objects, n_meshes).

        THE OBJECTS SHIP WITHOUT COLLIDERS. `tools/build_lab_world.py` applies
        RigidBodyAPI and MassAPI to each YCB prim and never CollisionAPI, so every
        object is a body gravity acts on and nothing can touch: measured in free fall
        from t=0, reaching -43 m after 3 s, which is exactly 0.5*g*t^2. MM2 verified 59
        properties of this world and passed, because MM2 never tried to touch an object.

        Fixed here, at load, rather than in the builder: the objects arrive as
        REFERENCES to the Isaac asset server, and their meshes are not composed into the
        stage at build time, so there is nothing for the builder to attach a collider to.
        By the time the sim has opened the stage they are there. The builder is still the
        better home for this if the references are ever payload-loaded at build time.

        convexHull, not the triangle mesh: PhysX will not simulate a non-convex triangle
        mesh as a DYNAMIC collider. The mug's handle hole is filled in as a result --
        declared, not hidden (C-41).
        """
        from pxr import Usd, UsdGeom, UsdPhysics
        n_obj = n_mesh = 0
        for root in ("/World/Env/objects", "/World/panthera_lab/objects"):
            parent = self.stage.GetPrimAtPath(root)
            if not (parent and parent.IsValid()):
                continue
            for obj in parent.GetChildren():
                got = 0
                for d in Usd.PrimRange(obj):
                    if not d.IsA(UsdGeom.Mesh):
                        continue
                    if not d.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI.Apply(d)
                    UsdPhysics.MeshCollisionAPI.Apply(d).CreateApproximationAttr().Set(
                        "convexHull")
                    got += 1
                if got:
                    n_obj += 1
                    n_mesh += got
            break
        print(f"[mm5] colliders applied: {n_obj} objects, {n_mesh} meshes "
              f"(convexHull) -- C-41", flush=True)
        return n_obj, n_mesh

    def _place_object(self, seed):
        """Randomize the object on the table and return its new world xyz.

        Uses Isaac's rigid-prim API, not a USD xform write. Two reasons, both learned
        the hard way: the object's xformOp:translate is in its PARENT's frame while the
        pose read back from ComputeLocalToWorldTransform is in the world's, so writing
        one into the other put the can at z = -43 m and it fell out of the world; and
        once physics is running the rigid body owns its transform, so a USD write is
        ignored or fought. set_world_pose + zeroed velocities is the supported path.
        """
        rng = np.random.default_rng(seed)
        if self._rigid is None:
            from isaacsim.core.prims import SingleRigidPrim
            prim = self.pipe._obj_prim(self.cfg.object_name)
            if prim is None:
                return None
            self._rigid = SingleRigidPrim(prim.GetPath().pathString)
            try:
                self._rigid.initialize()
            except Exception:
                pass
        base = np.array(self.home_obj_xyz, float)
        dx, dy = rng.uniform(-self.cfg.randomize_m, self.cfg.randomize_m, size=2)
        newp = np.array([base[0] + dx, base[1] + dy, base[2]], np.float32)
        self._rigid.set_world_pose(position=newp)
        try:
            self._rigid.set_linear_velocity(np.zeros(3, np.float32))
            self._rigid.set_angular_velocity(np.zeros(3, np.float32))
        except Exception:
            pass
        return newp

    def _stand_robot(self, obj_xyz):
        """Reset the robot to a clean standing state at the reach pose for this object.

        A full reset, not a teleport. Moving only the base left trial 2 starting from
        wherever trial 1 ended -- which, once trial 1 toppled, meant every subsequent
        trial began face-down and failed in APPROACH within half a second. Orientation,
        joint positions and both velocity fields all have to go back, or the trials are
        not independent samples and the success rate means nothing.
        """
        x = float(obj_xyz[0] + self.cfg.base_offset[0])
        y = float(obj_xyz[1] + self.cfg.base_offset[1])
        yaw = self.cfg.base_yaw
        quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], np.float32)
        pos = np.array([x, y, self.cfg.spawn_z], np.float32)
        self.art.set_world_pose(pos, quat)
        self.art.set_linear_velocity(np.zeros(3, np.float32))
        self.art.set_angular_velocity(np.zeros(3, np.float32))
        if self._home_q is None:
            self._home_q = np.asarray(self.art.get_joint_positions(), np.float32).copy()
        self.art.set_joint_positions(self._home_q)
        self.art.set_joint_velocities(np.zeros_like(self._home_q))
        self.pipe._arm_target = None
        self.pipe._grip_ratio = 0.0
        self.pipe._hand_target = np.zeros(len(self.pipe._hand_target), np.float32)
        return pos

    # ---------------------------------------------------------------------- step

    def step(self, dt):
        self._t += dt
        if self.state == "INIT":
            self.ensure_colliders()
            p = self.pipe.obj_pose(self.cfg.object_name)
            if p is None:
                print(f"[mm5] FATAL: object '{self.cfg.object_name}' not in the stage",
                      flush=True)
                self.state = "DONE"
                return
            self.home_obj_xyz = np.array([p[0], self.cfg.stage_y, p[2]], float)
            print(f"[mm5] staging object at the table's near edge: "
                  f"{np.round(self.home_obj_xyz,4)} (was {np.round(p,4)})", flush=True)
            # Snapshot the settled standing pose now, before any trial disturbs it.
            self._home_q = np.asarray(self.art.get_joint_positions(), np.float32).copy()
            print(f"[mm5] home object pose {np.round(p,4)}", flush=True)
            self.state = "NEXT"
            return

        if self.state == "NEXT":
            if self.i >= self.cfg.trials:
                self._write()
                self.state = "DONE"
                return
            self.i += 1
            seed = int(self.cfg.seed + self.i)
            obj = self._place_object(seed)
            self._stand_robot(obj)
            self.pipe.sim_time = self._t
            self.pipe.start_trial(self.i, seed, self.cfg.object_name)
            self.state = "RUN"
            return

        if self.state == "RUN":
            self.pipe.step(dt)
            if self.pipe.trial is None:
                self.state = "NEXT"

    def reapply(self):
        if self.state == "RUN":
            self.pipe.reapply()

    # -------------------------------------------------------------------- output

    def _write(self):
        rows = [t.as_row() for t in self.pipe.results]
        n = len(rows)
        ok = sum(1 for r in rows if r["outcome"] == "SUCCESS")
        tax = {}
        for r in rows:
            tax[r["outcome"]] = tax.get(r["outcome"], 0) + 1
        summary = {"trials": n, "successes": ok,
                   "success_rate": round(ok / n, 4) if n else 0.0,
                   "taxonomy": tax, "config": asdict(self.cfg), "rows": rows}
        with open(os.path.join(self.cfg.out_dir, "mm5_results.json"), "w") as fh:
            json.dump(summary, fh, indent=2)

        lines = [f"# MM5 results — {n} trials, {self.cfg.object_name}", "",
                 f"**Balancer: omni locomotion policy** (SONIC parked at C-39; "
                 f"CAMPAIGN 4.4 permits \"or omni standing if MM4 slips\").", "",
                 f"**Success rate: {ok}/{n} = {100*ok/n:.1f}%**" if n else "no trials", "",
                 "| outcome | count |", "|---|---|"]
        for k, v in sorted(tax.items(), key=lambda kv: -kv[1]):
            lines.append(f"| `{k}` | {v} |")
        lines += ["", "| trial | seed | outcome | detail | obj start | obj end | stage times (s) |",
                  "|---|---|---|---|---|---|---|"]
        for r in rows:
            st = " ".join(f"{k}={v}" for k, v in r["stage_s"].items())
            lines.append(f"| {r['trial']} | {r['seed']} | `{r['outcome']}` | {r['detail']} | "
                         f"{r['obj_start']} | {r['obj_end']} | {st} |")
        with open(os.path.join(self.cfg.out_dir, "mm5_results.md"), "w") as fh:
            fh.write("\n".join(lines) + "\n")
        with open(os.path.join(self.cfg.out_dir, "mm5_log.txt"), "w") as fh:
            fh.write("\n".join(self.pipe._log) + "\n")
        print(f"[mm5] WROTE {self.cfg.out_dir}/mm5_results.{{json,md}} — "
              f"{ok}/{n} success", flush=True)
