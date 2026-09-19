# PANTHERA (Sprint N, wb-cand-N02): the rod30 MANIPULATION OWNER for the ownership arbiter. Off unless G1_MANIP_REFERENCE is set.
#
# Plays the coordinator's manipulation reference schedule (manip-rod30-v1: 14 arm joints in ABSOLUTE URDF radians + the
# 6 independent right-hand joints, sim-time zero-order hold) through the arbiter as the manipulation owner:
#   WAIT -> (arbiter grants MANIP) -> BLEND_IN (policy arm targets -> row 0, hand open -> row 0, blend_in_s)
#        -> PLAY (rows by sim time since the end of the blend) -> BLEND_OUT (last row -> policy DEFAULT arm pose, blend_out_s)
#        -> release (the arbiter's RETURNING ramp then hands the arms back to the live policy targets)
# On grant: records the previous drive gains, sets the manipulation gains (arm kp/kd and hand kp/kd from the schedule), spawns the
# pelvis-relative scene (static table, free cylinder, visual destination disc) at the MEASURED pelvis pose, journals the pelvis
# pose; every step in MANIP: gravity feed-forward = the articulation's generalized gravity forces on the commanded arm joints,
# ramped over 1 s, capped at the drive's own max effort; on RETURNING->WALK: gains restored, efforts off. Object pose traced.
# Nothing here writes joint STATE; the runtime remains the only actuator writer (targets/efforts flow through the arbiter).
import json
import math
import os
import time

import numpy as np

from ownership import ARMS


def enabled() -> bool:
    return bool(os.environ.get("G1_MANIP_REFERENCE", "").strip())


def _quat_wxyz_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=float)


def _rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2])), math.atan2(R[1, 0], R[0, 0]))


class Rod30Source:
    def __init__(self, schedule_path, arbiter, policy, journal_dir, *, spawn_scene=True, gravity_ff=True):
        self.spec = json.load(open(schedule_path, "r", encoding="utf-8"))
        self.arb = arbiter; self.policy = policy; self.jdir = journal_dir
        self.arm_joints_all = list(self.spec["arm_joints"]); self.hand_joints = list(self.spec["hand_joints_right_independent"])
        # seam knob (Sprint N, first failing seam of n4a/n4b): with G1_MANIP_LEFT_ARM=policy the manipulation owner commands
        # only the RIGHT arm; the left arm stays with the walking policy instead of being driven to URDF zero by the schedule.
        self.left_arm_mode = os.environ.get("G1_MANIP_LEFT_ARM", "schedule").strip().lower()
        self.arm_joints = [n for n in self.arm_joints_all if not (self.left_arm_mode == "policy" and n.startswith("left_"))]
        self.arm_col = {n: self.arm_joints_all.index(n) for n in self.arm_joints}   # column of each commanded joint in arm_q rows
        self.rows = self.spec["rows"]; self.row_t = np.array([float(r["t"]) for r in self.rows])
        hc = self.spec.get("handoff_contract", {}); self.blend_in = float(os.environ.get("G1_MANIP_BLEND_IN_S", "") or hc.get("blend_in_s", 2.0)); self.blend_out = float(hc.get("blend_out_s", 2.0))
        g = self.spec.get("gains_while_manipulation_owner_holds", {})
        self.arm_kp = g.get("arm_kp_nm_rad", {}); self.arm_kd = g.get("arm_kd_nm_s_rad", g.get("arm_kd", {}))
        hk = g.get("hand_kp_nm_rad", 1.0); hd = g.get("hand_kd_nm_s_rad", 0.05)
        self.hand_kp = hk if isinstance(hk, dict) else {n: float(hk) for n in self.hand_joints}
        self.hand_kd = hd if isinstance(hd, dict) else {n: float(hd) for n in self.hand_joints}
        self.request_at = float(os.environ.get("G1_MANIP_REQUEST_AT_S", "0") or 0)   # > 0: request MANIP once at this sim time (from WALK)
        # Sprint O W3 split mode: play rows up to the first row of stage G1_MANIP_SPLIT_STAGE, then hand the arms back to the walk
        # owner as a CARRY HOLD (arms at that row's targets, hand keeps that row's grip targets) and release; at
        # G1_MANIP_RESUME_AT_S (sim s) request MANIP again and play the remaining rows (place/release), then clear the hold.
        self.split_stage = os.environ.get("G1_MANIP_SPLIT_STAGE", "").strip() or None
        self.resume_at = float(os.environ.get("G1_MANIP_RESUME_AT_S", "0") or 0)
        self.stop_stage = os.environ.get("G1_MANIP_STOP_STAGE", "").strip() or None   # ladder A: play up to this stage's end, hold, then blend out (no split)
        # Sprint O (declared before a run, default off): defer the scene spawn from the grant to the first row of stage
        # G1_MANIP_SPAWN_AT_STAGE, and realise the pelvis-relative scene block in G1_MANIP_SPAWN_FRAME = pelvis | torso. The
        # walking policy holds a 13-20 deg forward torso lean (pelvis pitch + waist pitch, pelvis z -1..-2 cm) for as long as the
        # right arm is forward (W2 prefilter traces, hold-phase averages); the arm rows are torso-relative, so "torso" realises the
        # planner's upright-pelvis / zero-waist geometry in the frame the arm chain actually hangs from (torso_link, with the
        # pelvis->torso_link zero-waist offset removed) at the moment the dwell ends.
        self.spawn_at_stage = os.environ.get("G1_MANIP_SPAWN_AT_STAGE", "").strip() or None
        # Sprint P M1 schedule 2: stages during which the station keeper is muted (base command 0 while the fingers close)
        self.station_quiet_stages = [x.strip() for x in os.environ.get("G1_STATION_QUIET_STAGES", "").split(",") if x.strip()]
        # Sprint O lB declaration: the scene given directly in WORLD coordinates (a world-fixed desk, as the integrated task needs):
        # G1_MANIP_SCENE_WORLD = path to {"object_center_world_m", "table_center_top_world_m", "destination_center_world_m",
        # "object_bottom_to_table_top_gap_m"}; spawned at the grant with props world-upright (overrides the deferred/torso options)
        self.scene_world = os.environ.get("G1_MANIP_SCENE_WORLD", "").strip() or None
        self.spawn_frame = os.environ.get("G1_MANIP_SPAWN_FRAME", "pelvis").strip().lower() or "pelvis"
        if self.spawn_frame == "torso_link": self.spawn_frame = "torso"      # alias used in the coordinator's declaration
        self._spawn_pending = False
        self._split_row = None; self._resume_pending = False; self._grants = 0
        self.spawn_scene = spawn_scene; self.gravity_ff = gravity_ff
        self.phase = "WAIT"; self._t_grant = None; self._arm_start = None; self._grant_walltime = None
        self._prev_gains = None; self._scene = None; self._obj = None; self._trace = None; self._n = 0
        self._pelvis0 = None; self._ff_clips = 0; self._contacts = {"rod_hand": 0, "rod_table": 0, "rod_stem": 0, "rod_other": 0, "rows": 0}; self._contact_sub = None
        self._flags = {"lifted": False, "held_after_lift": False, "transported": False, "released_resting": False, "max_clearance_m": 0.0, "lifted_task": False}; self._grasp_seen = False
        # Sprint O declared task flag (coordinator, 08:2xZ, before any W3 run): lifted_task = rod bottom clearance above the
        # support (stem) top >= 0.02 m held continuously >= 1.0 s; reported BESIDE the frozen 0.06-centre flag, never instead
        self._clear_since = None; self.lifted_task_rule = {"clearance_m": 0.02, "held_s": 1.0}
        self.default_arm = None
        self._log = open(os.path.join(journal_dir, "manip_owner.jsonl"), "a", encoding="utf-8")
        arbiter.hooks["grant"].append(self._on_grant); arbiter.hooks["walk"].append(self._on_walk)
        self._j("init", {"schedule": self.spec.get("version"), "rows": len(self.rows), "duration_s": self.spec.get("duration_s"),
                         "arm_joints_commanded": self.arm_joints, "left_arm_mode": self.left_arm_mode, "hand_joints": self.hand_joints,
                         "blend_in_s": self.blend_in, "blend_out_s": self.blend_out, "spawn_scene": spawn_scene, "gravity_ff": gravity_ff,
                         "spawn_at_stage": self.spawn_at_stage, "spawn_frame": self.spawn_frame, "scene_world": self.scene_world})

    def _j(self, event, detail=None):
        row = {"t": round(self.arb.t, 4), "wall": time.time(), "event": event, "phase": self.phase, "detail": detail}
        self._log.write(json.dumps(row) + "\n"); self._log.flush()
        print(f"[manip_owner] t={self.arb.t:8.3f} {event} {json.dumps(detail)[:300] if detail else ''}", flush=True)

    # ---- articulation helpers ---------------------------------------------------------------------------------------
    def _view(self):
        return self.policy.robot._articulation_view

    def _all_names(self):
        return list(self.policy.robot.dof_names)

    def _on_grant(self, arb):
        robot = self.policy.robot
        pos, quat = robot.get_world_pose(); R = _quat_wxyz_to_R(quat); rpy = _rpy(R)
        self._j("grant", {"pelvis_pos": [round(float(v), 4) for v in pos], "pelvis_quat_wxyz": [round(float(v), 5) for v in quat],
                          "pelvis_rpy_deg": [round(math.degrees(v), 2) for v in rpy]})
        names = self._all_names(); view = self._view()
        # policy arm targets at handoff = the policy's current target vector for the arm joints (policy order)
        pt = self.policy._action_offset + self.policy._action_scale * self.policy.action
        self._arm_start = {n: float(pt[self.policy_names().index(n)]) for n in self.arm_joints}
        if self._split_row is not None and self.arb._carry_arm:
            self._arm_start = {n: float(self.arb._carry_arm.get(n, self._arm_start[n])) for n in self.arm_joints}
        self.default_arm = {n: float(self.policy.default_pos[self.policy_names().index(n)]) for n in self.arm_joints}
        # gains: record, then set the manipulation gains on the arm and the right hand's independent joints
        kps, kds = view.get_gains(); kps = np.asarray(kps).reshape(-1).copy(); kds = np.asarray(kds).reshape(-1).copy()
        if self._prev_gains is None:   # record the walk owner's gains once (a resume grant must not capture the manipulation gains)
            self._prev_gains = (kps.copy(), kds.copy())
        idx = [names.index(n) for n in self.arm_joints + self.hand_joints]
        new_kp = np.array([float(self.arm_kp.get(n, 60.0)) for n in self.arm_joints] + [float(self.hand_kp.get(n, 1.0)) for n in self.hand_joints], dtype=np.float32)
        new_kd = np.array([float(self.arm_kd.get(n, 1.5)) for n in self.arm_joints] + [float(self.hand_kd.get(n, 0.05)) for n in self.hand_joints], dtype=np.float32)
        view.set_gains(new_kp, new_kd, joint_indices=np.array(idx, dtype=np.int64))
        self._max_eff = np.asarray(view.get_max_efforts()).reshape(-1)
        self._j("gains_set", {"arm_kp": float(new_kp[0]), "arm_kd": float(new_kd[0]), "hand_kp": float(new_kp[-1]), "hand_kd": float(new_kd[-1]),
                              "prev_arm_kp": float(kps[names.index(self.arm_joints[0])]), "prev_hand_kp": float(kps[names.index(self.hand_joints[0])]),
                              "arm_max_effort": {n: float(self._max_eff[names.index(n)]) for n in self.arm_joints}})
        self._grants += 1
        if self._grants == 1:
            self._pelvis0 = (np.asarray(pos, float).copy(), rpy)
            if self.spawn_scene and self.scene_world and not self.spawn_at_stage:
                self._spawn_world_now()
            elif self.spawn_scene and self.spawn_at_stage:
                self._spawn_pending = True; self._j("scene_spawn_deferred", {"until_stage": self.spawn_at_stage, "frame": "world" if self.scene_world else self.spawn_frame})
            elif self.spawn_scene:
                self._spawn(pos, R, quat)
                self._subscribe_contacts()
        self._t_grant = arb.t; self.phase = "BLEND_IN"
        self._j("BLEND_IN", {"arm_start": self._arm_start, "grant": self._grants, "resume_from_row": self._split_row})

    PELVIS_TO_TORSO_ZERO_WAIST = np.array([-0.0039635, 0.0, 0.044])   # pelvis -> torso_link origin with the waist joints at zero (donor URDF waist_roll_joint origin)

    def _spawn_world_now(self):
        """World-coordinate scene (declared file): object centre, desk-top centre, destination in world metres; realised with the
        identity frame at the grant, props world-upright and stacked under the object."""
        import copy
        w = json.load(open(self.scene_world)); sc = copy.deepcopy(self.spec["scene_pelvis_relative"])
        obj = [float(v) for v in w["object_center_world_m"]]; tab = [float(v) for v in w["table_center_top_world_m"]]; dst = [float(v) for v in w["destination_center_world_m"]]
        gap = float(w.get("object_bottom_to_table_top_gap_m", 0.0))
        sc["object"]["center_pelvis_m"] = obj; sc.setdefault("destination", {})["center_xy_m"] = dst[:2]; sc["destination"].pop("center_xy_pelvis_m", None)
        sc["table"]["center_xy_m"] = tab[:2]; sc["table"]["top_z_pelvis_m"] = obj[2] - float(sc["object"]["length_m"]) / 2.0 - gap
        sc["support_top_z_pelvis_m"] = sc["table"]["top_z_pelvis_m"] + gap; sc["_effective_from"] = f"world coordinates ({os.path.basename(self.scene_world)})"
        sc["_world_declared"] = {"object_center_world_m": obj, "table_center_top_world_m": tab, "destination_center_world_m": dst, "gap_m": gap,
                                 "declared_table_top_world_z": tab[2], "table_top_used_world_z": sc["table"]["top_z_pelvis_m"]}
        self._spawn(np.zeros(3), np.eye(3), [1.0, 0.0, 0.0, 0.0], upright_from_object=True, sc=sc)
        self._subscribe_contacts()
        self._j("scene_spawned_world", {"file": self.scene_world, "declared": sc["_world_declared"]})

    def _spawn_at_stage_now(self, t, stage):
        """Deferred spawn: realise the declared scene at this instant — world coordinates if declared, else the pelvis-relative
        block in the declared frame."""
        if self.scene_world:
            self._spawn_world_now(); self._spawn_pending = False; self._j("scene_spawned_at_stage", {"t": round(t, 3), "stage": stage, "frame": {"kind": "world", "file": self.scene_world}}); return
        robot = self.policy.robot; pos, quat = robot.get_world_pose(); R = _quat_wxyz_to_R(quat)
        frame = {"kind": "pelvis", "pos": [round(float(v), 4) for v in pos], "rpy_deg": [round(math.degrees(v), 2) for v in _rpy(R)]}
        if self.spawn_frame == "torso":
            view = self._view(); names = list(view.body_names); i = names.index("torso_link")
            tf = np.asarray(view._physics_view.get_link_transforms()).reshape(-1, len(names), 7)[0][i]
            tpos = np.asarray(tf[:3], float); qx, qy, qz, qw = [float(v) for v in tf[3:7]]      # physics tensor API: xyzw
            Rt = _quat_wxyz_to_R([qw, qx, qy, qz]); R = Rt
            yaw = _rpy(Rt)[2]; quat = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]              # props stay world-upright (yaw only)
            # the scene block's own frame: "pelvis_zero_waist" (default: the planner's upright pelvis with the waist at zero, so the
            # torso_link origin sits at PELVIS_TO_TORSO_ZERO_WAIST) or "torso_link" (coordinates already relative to torso_link)
            scene_frame = (os.environ.get("G1_MANIP_SCENE_FRAME", "").strip() or str(self.spec["scene_pelvis_relative"].get("frame", "pelvis_zero_waist"))).lower()
            pos = tpos if scene_frame == "torso_link" else tpos - Rt @ self.PELVIS_TO_TORSO_ZERO_WAIST   # the planner's pelvis origin = torso origin minus the zero-waist offset, in the torso frame
            frame = {"kind": "torso", "scene_block_frame": scene_frame, "torso_pos": [round(float(v), 4) for v in tpos], "torso_rpy_deg": [round(math.degrees(v), 2) for v in _rpy(Rt)],
                     "effective_pelvis_pos": [round(float(v), 4) for v in pos], "props_orientation": "world-upright, torso yaw", "pelvis_measured": {"pos": [round(float(v), 4) for v in robot.get_world_pose()[0]], "rpy_deg": [round(math.degrees(v), 2) for v in _rpy(_quat_wxyz_to_R(robot.get_world_pose()[1]))]}}
        self._spawn(pos, R, quat, upright_from_object=(self.spawn_frame == "torso"), sc=self._scene_block(scene_frame if self.spawn_frame == "torso" else "pelvis_zero_waist"))
        self._subscribe_contacts(); self._spawn_pending = False
        # hand-vs-object offset at the spawn instant (world and spawn-frame), so a spawn next to / inside the fingers is visible
        wr = self._wrist_pose(); off = None
        if wr is not None and self._scene is not None:
            dw = np.asarray(self._scene["rod_world"], float) - np.asarray(wr[:3], float)
            off = {"puck_minus_wrist_world_m": [round(float(v), 4) for v in dw], "puck_minus_wrist_frame_m": [round(float(v), 4) for v in (np.asarray(R).T @ dw)], "wrist_world": wr[:3],
                   "note": "wrist_yaw_link origin; the palm/fingers extend ~0.1-0.2 m beyond it"}
        self._j("scene_spawned_at_stage", {"t": round(t, 3), "stage": stage, "frame": frame, "hand_object_offset": off})

    def _scene_block(self, scene_frame):
        """Effective scene block. A block declared in the torso_link frame carries its coordinates in a 'torso_link_frame' sub-block
        (object_center_torso_link_m, destination_center_torso_link_m, table_center_top_torso_link_m, object_bottom_to_table_top_gap_m);
        those replace the pelvis-frame numbers when the spawn happens in the torso frame."""
        import copy, re
        sc = copy.deepcopy(self.spec["scene_pelvis_relative"]); tl = sc.get("torso_link_frame") or {}
        if scene_frame == "torso_link":
            # The planner's arm rows hang from a torso_link PITCHED by the planned waist angle (e.g. 0.34 rad) on an upright pelvis.
            # The physically correct torso_link coordinates of a pelvis-frame point p are R_y(waist)^T (p - t); a declaration that
            # lists R = identity for the pelvis->torso transform is only a translation (p - t) and would place the object ~15 cm too
            # low on a torso that really is pitched. We therefore derive the torso_link coordinates from the unambiguous pelvis-
            # frame numbers with the declared planned waist pitch (explicit 'planned_waist_pitch_rad', else parsed from the
            # 'pelvis_T_torso_link_at_waist_<rad>' key), and journal both for the record.
            key = next((k for k in tl if k.startswith("pelvis_T_torso_link")), None); tr = tl.get(key, {}) if key else {}
            if "planned_waist_pitch_rad" not in tl and "planned_waist_pitch_rad" not in sc and not key:
                raise RuntimeError("scene block declared torso_link without planned_waist_pitch_rad (and no torso_link_frame sub-block)")
            t = np.asarray(tr.get("t", self.PELVIS_TO_TORSO_ZERO_WAIST), float)
            m = re.search(r"at_waist_([0-9.]+)", key or ""); waist = float(tl.get("planned_waist_pitch_rad", sc.get("planned_waist_pitch_rad", m.group(1) if m else 0.0)))
            c, sn = math.cos(waist), math.sin(waist); RyT = np.array([[c, 0.0, -sn], [0.0, 1.0, 0.0], [sn, 0.0, c]])   # R_y(waist)^T
            def T(p_pelvis): return (RyT @ (np.asarray(p_pelvis, float) - t)).tolist()
            obj_p = list(sc["object"]["center_pelvis_m"]); obj_t = T(obj_p)
            dest_p = (sc.get("destination", {}).get("center_xy_pelvis_m") or sc.get("destination", {}).get("center_xy_m") or [0.57, -0.14])
            dest_t = T([dest_p[0], dest_p[1], obj_p[2]])                     # destination at the object's height
            tab = sc["table"]; tab_t = T([tab["center_xy_m"][0], tab["center_xy_m"][1], float(tab["top_z_pelvis_m"])])
            gap = float(sc.get("support_top_z_pelvis_m", tab["top_z_pelvis_m"])) - float(tab["top_z_pelvis_m"])
            sc["object"]["center_pelvis_m"] = obj_t
            sc.setdefault("destination", {})["center_xy_m"] = dest_t[:2]; sc["destination"].pop("center_xy_pelvis_m", None)
            sc["table"]["center_xy_m"] = tab_t[:2]; sc["table"]["top_z_pelvis_m"] = obj_t[2] - float(sc["object"]["length_m"]) / 2.0 - gap   # stack under the object (props go world-upright anyway)
            sc["support_top_z_pelvis_m"] = sc["table"]["top_z_pelvis_m"] + gap
            sc["_effective_from"] = "torso_link_frame (pelvis-frame numbers rotated by the planned waist pitch)"
            sc["_torso_link_derivation"] = {"planned_waist_pitch_rad": waist, "t": t.tolist(), "object_torso_link_true": obj_t, "object_torso_link_declared": tl.get("object_center_torso_link_m"),
                                            "destination_torso_link_true": dest_t, "table_top_center_torso_link_true": tab_t}
        return sc

    def _spawn(self, pos, R, quat, upright_from_object=False, sc=None):
        """Realise the declared pelvis-relative scene: positions = pos + R @ p_rel; orientations = quat. With upright_from_object
        (torso-frame spawn) the object keeps its transformed position but the table / stems / destination are placed WORLD-
        UPRIGHT with the table top at the object's resting bottom minus the declared table-top-to-support gap, so the object
        rests on a horizontal surface exactly where the leaned-torso arm rows reach it."""
        try:
            from isaacsim.core.api.objects import DynamicCylinder, FixedCuboid, FixedCylinder, VisualCylinder
            from isaacsim.core.api.materials import PhysicsMaterial
            sc = sc or self.spec["scene_pelvis_relative"]; pos = np.asarray(pos, float); self._sc_eff = sc
            def W(rel): return (pos + R @ np.asarray(rel, float)).tolist()
            tb = sc["table"]; sz = tb["size_m"]; tc = [tb["center_xy_m"][0], tb["center_xy_m"][1], float(tb["top_z_pelvis_m"]) - sz[2] / 2.0]
            ob = sc["object"]; support_z = float(sc.get("support_top_z_pelvis_m", tb["top_z_pelvis_m"])); gap = support_z - float(tb["top_z_pelvis_m"])
            rod_w = W(ob["center_pelvis_m"])
            if upright_from_object:
                top_w = rod_w[2] - float(ob["length_m"]) / 2.0 - gap                 # world z of the table top
                Ry = _quat_wxyz_to_R(quat)[:2, :2]; oxy = np.asarray(ob["center_pelvis_m"][:2], float)
                def WZ(rel, z):                                                       # horizontal offset from the OBJECT as planned (yaw-rotated), z absolute
                    xy = np.asarray(rod_w[:2], float) + Ry @ (np.asarray(rel[:2], float) - oxy); return [float(xy[0]), float(xy[1]), z]
                tc_w = WZ(tc, top_w - sz[2] / 2.0)
            else:
                top_w = None; tc_w = W(tc)
            mat = PhysicsMaterial(prim_path="/World/ManipScene/Material", static_friction=float(sc.get("static_friction", 0.7)), dynamic_friction=float(sc.get("dynamic_friction", 0.6)), restitution=0.0)
            table = FixedCuboid(prim_path="/World/ManipScene/Table", position=np.array(tc_w), orientation=np.array(quat, dtype=float), scale=np.array(sz, dtype=float), color=np.array([0.55, 0.4, 0.25]), physics_material=mat)
            self._obj = DynamicCylinder(prim_path="/World/ManipScene/Rod", position=np.array(rod_w), orientation=np.array(quat, dtype=float), radius=float(ob["radius_m"]), height=float(ob["length_m"]), mass=float(ob["mass_kg"]), color=np.array([0.9, 0.2, 0.2]), physics_material=mat)
            base = self._spawn_object_base(ob, mat)   # Sprint O declared prop: a base disc as a second collider of the SAME rigid body (desk-stand shape)
            # Sprint O v11-rod30-stems: static pedestals ("pedestals": 8 mm-radius, 100 mm-tall cylinders standing on the bench
            # top at the pick and destination xy); the object stands on the pick stem, and the support top used by the
            # lifted/placed flags is scene.support_top_z_pelvis_m (declared before physics), not the bench top
            stems = []
            for i, pd in enumerate(sc.get("pedestals", [])):
                pc = [pd["center_xy_m"][0], pd["center_xy_m"][1], float(tb["top_z_pelvis_m"]) + float(pd["height_m"]) / 2.0]
                pc_w = WZ(pc, top_w + float(pd["height_m"]) / 2.0) if upright_from_object else W(pc)
                FixedCylinder(prim_path=f"/World/ManipScene/Stem{i}", position=np.array(pc_w), orientation=np.array(quat, dtype=float), radius=float(pd["radius_m"]), height=float(pd["height_m"]), color=np.array([0.3, 0.3, 0.35]), physics_material=mat)
                stems.append({"prim": f"/World/ManipScene/Stem{i}", "world": pc_w, "radius_m": float(pd["radius_m"]), "height_m": float(pd["height_m"]), "role": pd.get("role")})
            dest = sc.get("destination", {}); dc = dest.get("center_xy_pelvis_m") or dest.get("center_xy_m") or [0.57, -0.14]
            dest_w = WZ([dc[0], dc[1], support_z], top_w + gap) if upright_from_object else W([dc[0], dc[1], support_z])
            VisualCylinder(prim_path="/World/ManipScene/Destination", position=np.array([dest_w[0], dest_w[1], dest_w[2] + 0.001]), orientation=np.array(quat, dtype=float), radius=float(dest.get("radius_m", 0.03)), height=0.002, color=np.array([0.2, 0.8, 0.3]))
            support_top_w = (top_w + gap) if upright_from_object else W([ob["center_pelvis_m"][0], ob["center_pelvis_m"][1], support_z])[2]
            self._scene = {"table_world": tc_w, "rod_world": rod_w, "destination_world": dest_w, "stems": stems, "object_base": base, "upright_from_object": bool(upright_from_object),
                           "support_top_z_pelvis_m": support_z, "support_top_world_z": support_top_w, "scene_block_effective_from": sc.get("_effective_from", "scene_pelvis_relative"),
                           "object_center_rel": list(ob["center_pelvis_m"]), "table_top_rel": float(tb["top_z_pelvis_m"])}
            self._j("scene_spawned", dict(self._scene, lifted_task_rule=self.lifted_task_rule, lifted_frozen_rule="rod centre >= 0.06 m above the support top", torso_link_derivation=sc.get("_torso_link_derivation"),
                                          lifted_frozen_rule_trivial_at_rest=bool(float(ob["length_m"]) / 2.0 >= 0.06)))   # a >= 12 cm object satisfies the rod30 rule while resting
        except Exception as exc:
            self._scene = None; self._obj = None
            self._j("scene_spawn_failed", {"error": repr(exc)})

    def _spawn_object_base(self, ob, mat):
        """object.base = {"radius_m", "thickness_m", "mass_kg"}: a disc under the rod's bottom end (its top at the rod bottom) added as a
        child collider of the rod's rigid body; mass/COM/inertia of the compound set explicitly (same arithmetic as the coordinator's
        command_replay.py probe knob). Declared before physics; a declared base that cannot be built aborts the spawn (no silent rod-only run)."""
        base_ = ob.get("base")
        if not base_:
            return None
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
        import omni.usd
        stage = omni.usd.get_context().get_stage(); body = stage.GetPrimAtPath("/World/ManipScene/Rod")
        r_, L_, m_ = float(ob["radius_m"]), float(ob["length_m"]), float(ob["mass_kg"]); Rb, Tb, Mb = float(base_["radius_m"]), float(base_["thickness_m"]), float(base_["mass_kg"])
        disc = UsdGeom.Cylinder.Define(stage, "/World/ManipScene/Rod/Base"); disc.CreateAxisAttr("Z"); disc.CreateRadiusAttr(Rb); disc.CreateHeightAttr(Tb)
        disc.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -L_ / 2.0 - Tb / 2.0)); disc.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.28)])
        UsdPhysics.CollisionAPI.Apply(disc.GetPrim())
        try:
            UsdShade.MaterialBindingAPI.Apply(disc.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(mat.prim_path)), UsdShade.Tokens.weakerThanDescendants, "physics")
        except Exception as exc:
            self._j("object_base_material_bind_failed", {"error": repr(exc)})
        Mt = m_ + Mb; zc = (Mb * (-L_ / 2.0 - Tb / 2.0)) / Mt
        Irod_xy = m_ * (3 * r_ * r_ + L_ * L_) / 12.0; Idisc_xy = Mb * (3 * Rb * Rb + Tb * Tb) / 12.0
        Ixy = Irod_xy + m_ * zc * zc + Idisc_xy + Mb * (-L_ / 2.0 - Tb / 2.0 - zc) ** 2; Iz = m_ * r_ * r_ / 2.0 + Mb * Rb * Rb / 2.0
        massapi = UsdPhysics.MassAPI.Apply(body); massapi.CreateMassAttr(Mt); massapi.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, zc))
        massapi.CreateDiagonalInertiaAttr(Gf.Vec3f(Ixy, Ixy, Iz)); massapi.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
        return {"prim": "/World/ManipScene/Rod/Base", "radius_m": Rb, "thickness_m": Tb, "mass_kg": Mb, "total_mass_kg": Mt, "com_z_m": zc, "inertia_diag": [Ixy, Ixy, Iz]}

    def _subscribe_contacts(self):
        try:
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysxSchema
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            prim = stage.GetPrimAtPath("/World/ManipScene/Rod")
            api = PhysxSchema.PhysxContactReportAPI.Apply(prim); api.CreateThresholdAttr(0.0)

            def _cb(headers, data):
                for h in headers:
                    a = str(h.actor0); b = str(h.actor1)
                    if "/World/ManipScene/Rod" not in (a, b):
                        continue
                    other = b if a.endswith("Rod") else a
                    if "/World/G1/right_" in other or "/World/G1/" in other and any(k in other for k in ("index", "middle", "ring", "little", "thumb", "wrist", "base_link")):
                        self._contacts["rod_hand"] += 1
                    elif "ManipScene/Table" in other:
                        self._contacts["rod_table"] += 1
                    elif "ManipScene/Stem" in other:
                        self._contacts["rod_stem"] += 1
                    else:
                        self._contacts["rod_other"] += 1
                    self._contacts["rows"] += 1

            self._contact_sub = get_physx_simulation_interface().subscribe_contact_report_events(_cb)
            self._j("contact_report_subscribed", {"prim": "/World/ManipScene/Rod"})
        except Exception as exc:
            self._j("contact_report_unavailable", {"error": repr(exc)})

    def _gravity_forces(self, names):
        """Joint-ordered gravity-compensation torques (Nm) for all DOFs. The physics tensor view of a FLOATING-BASE articulation
        prepends the 6 root DOFs; ArticulationView.get_generalized_gravity_forces() (Isaac Sim 5.1) slices the first num_dof
        columns of that wider tensor, so on a floating base it silently returns the root wrench followed by the joints shifted
        by six (found in Sprint O lA: the elbow's -3.6 Nm landed on the wrist yaw, the shoulder pitch got the hip yaw's +1.0 Nm,
        2.6-5 deg of arm sag). We read the raw tensor and drop the root entries; any other size disables the feed-forward."""
        view = self._view(); n = len(names)
        raw = np.asarray(view._physics_view.get_gravity_compensation_forces()).reshape(-1)
        if raw.size == n + 6:
            g = raw[6:]; layout = {"raw_entries": int(raw.size), "dofs": n, "root_entries_skipped": 6, "base": "floating"}
        elif raw.size == n:
            g = raw; layout = {"raw_entries": int(raw.size), "dofs": n, "root_entries_skipped": 0, "base": "fixed"}
        else:
            raise RuntimeError(f"gravity compensation forces have {raw.size} entries for {n} DOFs")
        if not getattr(self, "_ff_layout_logged", False):
            self._ff_layout_logged = True; self._j("gravity_ff_layout", layout)
        return g

    def _wrist_pose(self):
        try:
            view = self._view(); names = list(view.body_names); i = names.index("right_wrist_yaw_link")
            tf = np.asarray(view._physics_view.get_link_transforms()).reshape(-1, len(names), 7)[0][i]
            return [round(float(v), 4) for v in tf]
        except Exception:
            return None

    def _trace_row(self, t):
        robot = self.policy.robot; pos, quat = robot.get_world_pose(); R = _quat_wxyz_to_R(quat); rpy = _rpy(R)
        row = {"t": round(t, 4), "phase": self.phase, "owners": self.arb.owner_by_group(), "pelvis": {"z": round(float(pos[2]), 4), "rpy_deg": [round(math.degrees(v), 2) for v in rpy]},
               "pelvis_drift_since_handoff": {"xy_m": round(float(np.hypot(pos[0] - self._pelvis0[0][0], pos[1] - self._pelvis0[0][1])), 4), "z_m": round(float(pos[2] - self._pelvis0[0][2]), 4), "yaw_deg": round(math.degrees(rpy[2] - self._pelvis0[1][2]), 2)} if self._pelvis0 is not None else None,
               "wrist_yaw_link_world": self._wrist_pose(), "ff_clip_count": self._ff_clips, "contacts": dict(self._contacts)}
        if self._obj is not None and self._scene is not None:
            try:
                p, q = self._obj.get_world_pose(); v = self._obj.get_linear_velocity()
                top = float(self._scene.get("support_top_world_z", float(self._scene["table_world"][2]) + float(self.spec["scene_pelvis_relative"]["table"]["size_m"][2]) / 2.0))
                dest = np.asarray(self._scene["destination_world"][:2]); above = float(p[2]) - top
                dxy = float(np.hypot(p[0] - dest[0], p[1] - dest[1])); speed = float(np.linalg.norm(np.asarray(v)))
                in_hand = self._contacts["rod_hand"] > 0 and speed >= 0.0
                if above >= 0.06 and self.phase == "PLAY": self._flags["lifted"] = True; self._grasp_seen = True
                if self._flags["lifted"] and above >= 0.06: self._flags["held_after_lift"] = True
                if self._flags["lifted"] and dxy <= 0.03: self._flags["transported"] = True
                if self._flags["transported"] and abs(above - float(self.spec["scene_pelvis_relative"]["object"]["length_m"]) / 2.0) < 0.01 and speed < 0.02 and self.phase in ("BLEND_OUT", "RELEASED"): self._flags["released_resting"] = True
                clearance = above - float(self.spec["scene_pelvis_relative"]["object"]["length_m"]) / 2.0   # rod bottom above the support top
                self._flags["max_clearance_m"] = round(max(float(self._flags.get("max_clearance_m") or 0.0), clearance), 4)
                if clearance >= self.lifted_task_rule["clearance_m"]:
                    if self._clear_since is None: self._clear_since = t
                    if t - self._clear_since >= self.lifted_task_rule["held_s"] and not self._flags["lifted_task"]:
                        self._flags["lifted_task"] = True; self._j("lifted_task", {"t": round(t, 3), "clearance_m": round(clearance, 4), "rule": self.lifted_task_rule})
                else:
                    self._clear_since = None
                row["rod"] = {"pos": [round(float(x), 4) for x in p], "quat_wxyz": [round(float(x), 4) for x in q], "above_table_top_m": round(above, 4), "clearance_m": round(clearance, 4), "dist_to_destination_m": round(dxy, 4), "speed_m_s": round(speed, 4)}
                row["flags"] = dict(self._flags)
            except Exception as exc:
                row["rod_error"] = repr(exc)
        if self._trace is None:
            self._trace = open(os.path.join(self.jdir, "manip_trace.jsonl"), "a", encoding="utf-8")
        self._trace.write(json.dumps(row) + "\n"); self._trace.flush()

    def _on_walk(self, arb):
        if getattr(arb, "station", None) is not None: arb.station.mute = False
        if self.phase == "CARRY":
            # ladder C: body ownership returned while the hand keeps the grip; gains stay as the manipulation owner set them
            # for the held joints so the grip force does not change hands mid-carry (restored at the final release)
            self._j("WALK_CARRY", {"arms_held": self.arm_joints, "hand_held": self.hand_joints})
            return
        if self._prev_gains is not None:
            kps, kds = self._prev_gains; names = self._all_names()
            idx = np.array([names.index(n) for n in self.arm_joints + self.hand_joints], dtype=np.int64)
            self._view().set_gains(kps[idx].astype(np.float32), kds[idx].astype(np.float32), joint_indices=idx)
            self._j("gains_restored", {"arm_kp": float(kps[idx[0]]), "hand_kp": float(kps[idx[-1]])})
        self.arb.set_efforts(None); self.arb.clear_carry_hold(source="rod30"); self.phase = "DONE"
        self._j("DONE", {"flags": dict(self._flags), "contacts": dict(self._contacts), "ff_clip_count": self._ff_clips})

    def policy_names(self):
        return self.arb.policy_names

    # ---- per step ----------------------------------------------------------------------------------------------------
    def step(self, t, state):
        if state == "WALK" and self.phase == "WAIT" and self.request_at > 0 and t >= self.request_at:
            self.request_at = 0.0; self.arb.request_manip(source="rod30")
        if self.phase == "CARRY":
            if state == "WALK" and self.resume_at > 0 and t >= self.resume_at and not self._resume_pending:
                self._resume_pending = True; self.arb.request_manip(source="rod30-resume"); self._j("resume_requested", {"t": round(t, 2)})
            if state == "MANIP" and self._resume_pending:
                self._resume_pending = False   # _on_grant already ran (phase BLEND_IN set there)
            elif state != "MANIP":
                return
        if state != "MANIP" or self.phase in ("WAIT", "DONE", "RELEASED"):
            return
        s = t - self._t_grant
        if self.phase == "BLEND_IN":
            a = min(1.0, s / max(1e-6, self.blend_in)); k0 = self._split_row if self._split_row is not None else 0; r0 = self.rows[k0]
            hand0 = list(self._carry_hand_vals) if (self._split_row is not None and getattr(self, "_carry_hand_vals", None)) else [0.0] * len(self.hand_joints)
            arm = [(1 - a) * self._arm_start[n] + a * float(r0["arm_q"][self.arm_col[n]]) for n in self.arm_joints]
            hand = [(1 - a) * hand0[i] + a * float(r0["hand_q_right"][i]) for i in range(len(self.hand_joints))]
            if a >= 1.0:
                self.phase = "PLAY"; self._t_play = t - float(self.row_t[k0]); self._j("PLAY", {"from_row": k0, "stage": r0.get("stage")})
        elif self.phase == "PLAY":
            sp = t - self._t_play; k = int(np.searchsorted(self.row_t, sp, side="right") - 1); k = max(0, min(k, len(self.rows) - 1))
            r = self.rows[k]; arm = [float(r["arm_q"][self.arm_col[n]]) for n in self.arm_joints]; hand = [float(v) for v in r["hand_q_right"]]
            if self._spawn_pending and r.get("stage") == self.spawn_at_stage:
                self._spawn_at_stage_now(t, r.get("stage"))
            if self.station_quiet_stages and getattr(self.arb, "station", None) is not None:
                self.arb.station.mute = r.get("stage") in self.station_quiet_stages
            if self._n % 200 == 0:
                self._j("row", {"k": k, "stage": r.get("stage"), "sp": round(sp, 3)})
            if self.split_stage and self._split_row is None and r.get("stage") == self.split_stage:
                # ladder C: hand the arms to the carry hold at this row, keep the grip, release the manipulation owner
                self._split_row = k; self._carry_hand_vals = list(hand)
                self.arb.set_targets(self.arm_joints + self.hand_joints, arm + hand, source="rod30")
                self.arb.set_carry_hold(dict(zip(self.arm_joints, arm)), dict(zip(self.hand_joints, hand)), source="rod30")
                self.arb.release(source="rod30"); self.phase = "CARRY"; self._j("SPLIT->CARRY", {"row": k, "stage": self.split_stage, "resume_at_s": self.resume_at})
                return
            if self.stop_stage and r.get("stage") != self.stop_stage and any(x.get("stage") == self.stop_stage for x in self.rows[:k]):
                self.phase = "BLEND_OUT"; self._t_out = t; self._last = (arm, hand); self._j("BLEND_OUT", {"to": "policy default arm pose", "after_stage": self.stop_stage})
                if getattr(self.arb, "station", None) is not None: self.arb.station.mute = False
            elif sp >= float(self.row_t[-1]):
                self.phase = "BLEND_OUT"; self._t_out = t; self._last = (arm, hand); self._j("BLEND_OUT", {"to": "policy default arm pose"})
                if getattr(self.arb, "station", None) is not None: self.arb.station.mute = False
        else:  # BLEND_OUT
            a = min(1.0, (t - self._t_out) / max(1e-6, self.blend_out)); la, lh = self._last
            arm = [(1 - a) * la[i] + a * self.default_arm[n] for i, n in enumerate(self.arm_joints)]
            hand = [(1 - a) * lh[i] for i in range(len(self.hand_joints))]
            if a >= 1.0:
                self.arb.set_targets(self.arm_joints + self.hand_joints, arm + hand, source="rod30")
                self.arb.release(source="rod30"); self.phase = "RELEASED"; self._j("released", {})
                return
        self.arb.set_targets(self.arm_joints + self.hand_joints, arm + hand, source="rod30")
        # gravity feed-forward on the commanded arm joints, ramped over 1 s after grant, capped at the drive max effort
        if self.gravity_ff:
            try:
                names = self._all_names(); g = self._gravity_forces(names)
                ramp = min(1.0, s / 1.0); eff = {}
                for n in self.arm_joints:
                    i = names.index(n); cap = float(self._max_eff[i]) if self._max_eff is not None else 25.0
                    raw = ramp * float(g[i]); eff[n] = float(np.clip(raw, -cap, cap))
                    if abs(raw) > cap: self._ff_clips += 1
                self.arb.set_efforts(eff)
                if self._n % 200 == 0:
                    self._j("gravity_ff", {"ramp": round(ramp, 2), "effort_nm": {k: round(v, 3) for k, v in eff.items()}})
            except Exception as exc:
                self.gravity_ff = False; self.arb.set_efforts(None); self._j("gravity_ff_disabled", {"error": repr(exc)})
        if self._n % 10 == 0:
            try:
                self._trace_row(t)
            except Exception as exc:
                if self._n % 2000 == 0: self._j("trace_error", {"error": repr(exc)})
        self._n += 1
